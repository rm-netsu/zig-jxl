//! Lossless modular frame encoder.
//!
//! Pipeline (mirrors libjxl enc_frame + enc_modular for the lossless
//! modular path): headers -> channel planes -> global transforms
//! (palette/RCT/squeeze, the cjxl default heuristics) -> stream images
//! (global + AC groups + DC groups, with per-group local transforms) ->
//! trained tree -> tokens -> histograms -> sections -> TOC.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const bits = @import("../base/bits.zig");

const ans_common = @import("../entropy/ans_common.zig");
const enc_ans = @import("../entropy/enc_ans.zig");

const size_mod = @import("../header/size.zig");
const color_mod = @import("../header/color.zig");
const metadata_mod = @import("../header/metadata.zig");
const frame_mod = @import("../header/frame.zig");
const toc_mod = @import("../header/toc.zig");
const container_mod = @import("../header/container.zig");
const enc_icc = @import("../header/enc_icc.zig");

const modular_common = @import("../modular/common.zig");
const options = @import("../modular/options.zig");
const image_mod = @import("../modular/image.zig");
const context_predict = @import("../modular/context_predict.zig");
const ma = @import("../modular/ma.zig");
const ma_encode = @import("../modular/ma_encode.zig");
const stream = @import("../modular/stream.zig");
const transforms = @import("../modular/transforms.zig");
const enc_transforms = @import("../modular/enc_transforms.zig");
const enc_patches = @import("../patches/enc_patches.zig");
const patches_mod = @import("../patches/dictionary.zig");
const fields_ext = @import("../header/fields_ext.zig");
const lossy_mod = @import("../vardct/lossy.zig");

const pixel_mod = @import("pixel.zig");

const Allocator = std.mem.Allocator;
const BitWriter = bit_io.BitWriter;
const PixelBuffer = pixel_mod.PixelBuffer;
const FloatPixelBuffer = pixel_mod.FloatPixelBuffer;
const Image = image_mod.Image;
const Channel = image_mod.Channel;
const Token = ma_encode.Token;
const ImageMetadata = metadata_mod.ImageMetadata;
const FrameHeader = frame_mod.FrameHeader;
const float_convert = @import("../base/float_convert.zig");

pub const EncodeError = error{
    UnsupportedBitDepth,
    UnsupportedDimensions,
    UnrepresentableFloat,
    FloatExponentOutOfRange,
    FloatLosesPrecision,
} || Allocator.Error || anyerror;

/// The encoder input surface: integer samples (PNM-style, optional alpha),
/// float samples (PFM-style fp32/fp16, no alpha), or raw modular planes.
pub const InputImage = union(enum) {
    int: *const PixelBuffer,
    float: *const FloatPixelBuffer,
    /// Raw modular-domain planes (the patch reference image): signed
    /// integer samples (patch content is a difference image — negative
    /// wherever the fragment is darker than its background), no
    /// resampling, exactly one plane per channel (color first, then the
    /// extra channels, e.g. a zero alpha).
    planes: *const ModularPlanes,
};

/// Companion payload of `InputImage.planes`.
pub const ModularPlanes = struct {
    /// `n_color + n_ec` planes of `xsize * ysize` samples; may be
    /// negative.
    planes: []const []const i32,
    xsize: usize,
    ysize: usize,
    gray: bool,
    /// Number of extra channels (trailing planes); alpha, zero-filled by
    /// the patch path (the reference frame carries the codestream's
    /// channel count, like libjxl RoundtripPatchFrame's placeholder
    /// channels).
    num_extra: usize,
    bitdepth: u8,
};

pub const EncodeOptions = struct {
    /// 0: 128, 1: 256 (default), 2: 512, 3: 1024.
    group_size_shift: u32 = 1,
    /// cparams.responsive: enable the Squeeze transform (cjxl `-R 1`).
    responsive: bool = false,
    /// cparams.qprogressive_mode (cjxl `-p` for lossless input): a
    /// 2-pass progressive codestream — the responsive squeeze residuals
    /// are split across passes (pass 0 carries the shift-1 channels, the
    /// last pass the full-resolution channels). Forces `responsive`.
    progressive: bool = false,
    /// cparams.palette_colors: 0 disables palettes; negative selects image
    /// order instead of luma order (cjxl `--modular_palette_colors`).
    palette_colors: i32 = 1 << 10,
    /// cparams.channel_colors_pre_transform_percent (global pass).
    channel_colors_global_percent: f32 = 95.0,
    /// cparams.channel_colors_percent (per-group pass).
    channel_colors_group_percent: f32 = 80.0,
    /// cparams.colorspace: -1 auto (per-group RCT search), 0 none,
    /// 1..41 a fixed RCT type (cjxl `-C`).
    colorspace: i32 = -1,
    /// Raw ICC profile to embed (sets ColorEncoding.want_icc; the profile
    /// travels compressed in the codestream headers like libjxl's
    /// WriteICC). Borrowed; must outlive the `encode` call.
    icc: []const u8 = &.{},
    /// cparams.resampling (cjxl `--resampling`): 1 (none), 2, 4 or 8. The
    /// input is box-downsampled (libjxl DownsampleColorChannels) and the
    /// frame header declares the matching upsampling; the decoder
    /// reconstructs with the NLEE filter.
    resampling: u32 = 1,
    /// cparams.ec_resampling; 0 follows `resampling` (the cjxl default).
    ec_resampling: u32 = 0,
    /// Encode float input as fp16 instead of fp32 (the values must be
    /// exactly representable in binary16).
    float16: bool = false,
    /// Encoder effort, cjxl `-e` (1..10; 7 is the default). Maps to
    /// libjxl SpeedTier = 10 - effort: 7 = kSquirrel (the default
    /// heuristics), 8 = kKitten, 9 = kTortoise, 10 = kGlacier — the slower
    /// tiers add cost-based palette reverts, wider property sets, more
    /// RCT candidates and the weighted-predictor mode search; 1..6 are the
    /// faster tiers (falcon/thunder/lightning use fixed trees, cheetah
    /// and faster skip the local transform search).
    effort: u32 = 7,
    /// cparams.patches (cjxl `--patches=0|1`, PatchOverride):
    /// `.default` enables the search at effort >= 7 (libjxl
    /// FindTextLikePatches: speed_tier <= kSquirrel unless forced),
    /// `.on` forces it at effort >= 5 (the modular call requires
    /// tier < kCheetah), `.off` disables it. Integer input only, at
    /// resampling 1 (libjxl enc_modular: bits_per_sample <= 16 and the
    /// search runs on the full-resolution color planes).
    patches: enc_patches.PatchOverride = .default,
    /// cparams.centerfirst (cjxl `--group_order=1`): write the TOC with
    /// the center-first section permutation so decoders can start at the
    /// image center.
    group_order: bool = false,
    /// cjxl `-d`: the Butteraugli distance of the lossy VarDCT path
    /// (`encodeLossy`); 0 (the default) keeps the lossless modular path.
    distance: f32 = 0,
};

/// libjxl SpeedTier (enum value = 10 - cjxl effort).
pub const SpeedTier = enum(u8) {
    glacier = 0,
    tortoise = 1,
    kitten = 2,
    squirrel = 3,
    wombat = 4,
    hare = 5,
    cheetah = 6,
    falcon = 7,
    thunder = 8,
    lightning = 9,
};

/// Everything the modular heuristics vary with the speed tier (libjxl
/// enc_modular.cc Init/PrepareStreamParams/ComputeTree + enc_ans.cc
/// HistogramParams::ForModular, the lossless modular subset).
const TierParams = struct {
    /// `nb_rcts_to_try` (the local RCT search; 0 at cheetah and faster,
    /// where the global YCoCg applies instead). Counts the no-op baseline.
    nb_rcts: usize,
    /// Number of entries of the libjxl non-responsive `prop_order` used
    /// for tree training (before the few-streams group-property drop).
    n_props: usize,
    /// ModularOptions::max_property_values.
    max_property_values: usize,
    /// ModularOptions::splitting_heuristics_node_threshold
    /// (libjxl: 75 + 14 * tier + 10 * decoding_speed_tier).
    node_threshold: f32,
    /// Real EstimateCost for the palette color-count heuristic
    /// (tier <= kSquirrel, i.e. efforts 7-10).
    real_palette_cost: bool,
    /// Cost-based palette reverts (`maybe_do_transform`,
    /// tier < kSquirrel, i.e. efforts 8-10).
    palette_reverts: bool,
    /// Local (per-group) palette pass (tier < kCheetah, i.e. efforts 5+).
    local_palettes: bool,
    /// Number of weighted-predictor modes searched per stream
    /// (tortoise/glacier 5, kitten 2, otherwise 1 = mode 0).
    wp_modes: usize,
    /// Fixed tree instead of a learned one (thunder = gradient-over-
    /// gradient-property, falcon = weighted-over-WP-property, lightning =
    /// a single gradient leaf; null at cheetah and slower).
    fixed_tree: ?enum { gradient_leaf, gradient_fixed, wp_fixed } = null,
    /// Drop the group-id property when there are few streams
    /// (tier > kTortoise, i.e. efforts 1-8).
    drop_group_prop_few_streams: bool,
    /// HistogramParams (HistogramParams::ForModular, lossless modular).
    histo: enc_ans.HistogramParams,
};

fn tierParams(effort: u32) TierParams {
    const tier: u8 = @intCast(10 - effort);
    const t: SpeedTier = @enumFromInt(tier);
    return switch (t) {
        .glacier, .tortoise => .{
            .nb_rcts = 19,
            .n_props = 16,
            .max_property_values = 256,
            .node_threshold = @floatFromInt(75 + 14 * @as(i32, tier)),
            .real_palette_cost = true,
            .palette_reverts = true,
            .local_palettes = true,
            .wp_modes = 5,
            .fixed_tree = null,
            .drop_group_prop_few_streams = false,
            .histo = .{
                .clustering = .kBest,
                .uint_method = .kBest,
                .lz77_method = .kOptimal,
                .ans_histogram_strategy = .kPrecise,
            },
        },
        .kitten => .{
            .nb_rcts = 9,
            .n_props = 10,
            .max_property_values = 128,
            .node_threshold = 103,
            .real_palette_cost = true,
            .palette_reverts = true,
            .local_palettes = true,
            .wp_modes = 2,
            .fixed_tree = null,
            .drop_group_prop_few_streams = true,
            .histo = .{
                .clustering = .kBest,
                .uint_method = .kBest,
                .lz77_method = .kLZ77,
                .ans_histogram_strategy = .kPrecise,
            },
        },
        .squirrel => .{
            .nb_rcts = 7,
            .n_props = 7,
            .max_property_values = 96,
            .node_threshold = 117,
            .real_palette_cost = true,
            .palette_reverts = false,
            .local_palettes = true,
            .wp_modes = 1,
            .fixed_tree = null,
            .drop_group_prop_few_streams = true,
            .histo = .{
                .clustering = .kFast,
                .uint_method = .kFast,
                .lz77_method = .kRLE,
                .ans_histogram_strategy = .kApproximate,
            },
        },
        // hare (e5) and wombat (e6): learned trees with narrower property
        // sets; the local RCT search still runs (4 / 5 candidates).
        .wombat => .{
            .nb_rcts = 5,
            .n_props = 5,
            .max_property_values = 64,
            .node_threshold = 131,
            .real_palette_cost = false,
            .palette_reverts = false,
            .local_palettes = true,
            .wp_modes = 1,
            .fixed_tree = null,
            .drop_group_prop_few_streams = true,
            .histo = .{
                .clustering = .kFast,
                .uint_method = .kFast,
                .lz77_method = .kRLE,
                .ans_histogram_strategy = .kApproximate,
            },
        },
        .hare => .{
            .nb_rcts = 4,
            .n_props = 4,
            .max_property_values = 48,
            .node_threshold = 145,
            .real_palette_cost = false,
            .palette_reverts = false,
            .local_palettes = true,
            .wp_modes = 1,
            .fixed_tree = null,
            .drop_group_prop_few_streams = true,
            .histo = .{
                .clustering = .kFast,
                .uint_method = .kFast,
                .lz77_method = .kRLE,
                .ans_histogram_strategy = .kApproximate,
            },
        },
        // cheetah (e4): learned tree over the default 8-property set; no
        // local transform search (the global YCoCg applies instead), no
        // local palettes, no LZ77 in the entropy streams.
        .cheetah => .{
            .nb_rcts = 0,
            .n_props = 8,
            .max_property_values = 32,
            .node_threshold = 159,
            .real_palette_cost = false,
            .palette_reverts = false,
            .local_palettes = false,
            .wp_modes = 1,
            .fixed_tree = null,
            .drop_group_prop_few_streams = true,
            .histo = .{
                .clustering = .kFast,
                .uint_method = .kNone,
                .lz77_method = .kNone,
                .ans_histogram_strategy = .kApproximate,
            },
        },
        // falcon (e3): the fixed kWPFixedDC tree (weighted predictor over
        // the WP property); thunder (e2): the fixed kGradientFixedDC tree;
        // lightning (e1): a single gradient leaf.
        .falcon => .{
            .nb_rcts = 0,
            .n_props = 0,
            .max_property_values = 0,
            .node_threshold = 173,
            .real_palette_cost = false,
            .palette_reverts = false,
            .local_palettes = false,
            .wp_modes = 1,
            .fixed_tree = .wp_fixed,
            .drop_group_prop_few_streams = true,
            .histo = .{
                .clustering = .kFast,
                .uint_method = .kNone,
                .lz77_method = .kNone,
                .ans_histogram_strategy = .kApproximate,
            },
        },
        .thunder => .{
            .nb_rcts = 0,
            .n_props = 0,
            .max_property_values = 0,
            .node_threshold = 187,
            .real_palette_cost = false,
            .palette_reverts = false,
            .local_palettes = false,
            .wp_modes = 1,
            .fixed_tree = .gradient_fixed,
            .drop_group_prop_few_streams = true,
            .histo = .{
                .clustering = .kFast,
                .uint_method = .kNone,
                .lz77_method = .kNone,
                .ans_histogram_strategy = .kApproximate,
            },
        },
        .lightning => .{
            .nb_rcts = 0,
            .n_props = 0,
            .max_property_values = 0,
            .node_threshold = 201,
            .real_palette_cost = false,
            .palette_reverts = false,
            .local_palettes = false,
            .wp_modes = 1,
            .fixed_tree = .gradient_leaf,
            .drop_group_prop_few_streams = true,
            .histo = .{
                .clustering = .kFast,
                .uint_method = .kNone,
                .lz77_method = .kNone,
                .ans_histogram_strategy = .kFast,
            },
        },
    };
}

const ANSToken = enc_ans.Token;

// ---------------------------------------------------------------------------
// Fixed trees (libjxl enc_encoding.cc MakeFixedTree, the kWPFixedDC /
// kGradientFixedDC trees of the fast speed tiers)
// ---------------------------------------------------------------------------

/// Cutoffs shared by the kWPFixedDC and kGradientFixedDC fixed trees.
const fixed_tree_cutoffs = [_]i32{
    -500, -392, -255, -191, -127, -95, -63, -47, -31, -23, -15,
    -11,  -7,   -4,   -3,   -1,   0,   1,   3,   5,   7,   11,
    15,   23,   31,   47,   63,   95,   127, 191, 255, 392, 500,
};

fn ceilLog2Nonzero(v: usize) usize {
    std.debug.assert(v != 0);
    const l = std.math.log2_int(usize, v);
    return l + @as(usize, @intFromBool(@popCount(v) != 1));
}

/// Port of `MakeFixedTree` (libjxl enc_encoding.cc): a balanced binary tree
/// over `cutoffs` on `property` with `pred` leaves. The height is reduced for
/// small images (min_gap) and the cutoffs scaled by the bit depth above 11
/// bits (shift up to 4), exactly as libjxl builds the fixed DC trees.
fn makeFixedTree(
    allocator: Allocator,
    property: i32,
    cutoffs: []const i32,
    pred: options.Predictor,
    num_pixels: usize,
    bitdepth: u32,
) !ma.Tree {
    const log_px = ceilLog2Nonzero(num_pixels);
    var min_gap: usize = 0;
    if (log_px < 14) min_gap = 8 * (14 - log_px);
    const shift: u5 = if (bitdepth > 11) @intCast(@min(4, bitdepth - 11)) else 0;
    const mul: i64 = @as(i64, 1) << shift;

    var tree = ma.Tree.empty;
    errdefer tree.deinit(allocator);
    const NodeInfo = struct { begin: usize, end: usize, pos: usize };
    var queue: std.ArrayList(NodeInfo) = .empty;
    defer queue.deinit(allocator);

    try tree.append(allocator, ma.PropertyDecisionNode.leaf(pred, 0, 1));
    try queue.append(allocator, .{ .begin = 0, .end = cutoffs.len, .pos = 0 });

    var head: usize = 0;
    while (head < queue.items.len) : (head += 1) {
        const info = queue.items[head];
        if (info.begin + min_gap >= info.end) continue;
        const split = (info.begin + info.end) / 2;
        const cutoff: i32 = @intCast(cutoffs[split] * mul);
        tree.items[info.pos] = ma.PropertyDecisionNode.split(property, cutoff, @intCast(tree.items.len), null);
        // lchild takes the values above the cutoff, rchild the rest —
        // matching the child order of libjxl's Split (rchild = lchild + 1).
        try queue.append(allocator, .{ .begin = split + 1, .end = info.end, .pos = tree.items.len });
        try tree.append(allocator, ma.PropertyDecisionNode.leaf(pred, 0, 1));
        try queue.append(allocator, .{ .begin = info.begin, .end = split, .pos = tree.items.len });
        try tree.append(allocator, ma.PropertyDecisionNode.leaf(pred, 0, 1));
    }
    return tree;
}

fn toAnsTokens(allocator: Allocator, src: []const Token) !std.ArrayList(ANSToken) {
    var out: std.ArrayList(ANSToken) = .empty;
    errdefer out.deinit(allocator);
    try out.ensureTotalCapacity(allocator, src.len);
    for (src) |t| {
        out.appendAssumeCapacity(.{ .is_lz77_length = t.is_lz77_length, .context = t.context, .value = t.value });
    }
    return out;
}

/// One non-global modular stream: the stream image (channel rect copies)
/// plus its local transforms, its weighted-predictor header (the WP-mode
/// search result at slower efforts) and the TOC section it is written to.
const StreamData = struct {
    image: Image,
    stream_id: usize,
    /// TOC section index this stream's bytes go to.
    section_index: usize,
    /// Transform list serialized into this stream's GroupHeader.
    transform_list: std.ArrayList(transforms.Transform) = .empty,
    /// Weighted-predictor header serialized into this stream's
    /// GroupHeader and used to tokenize it.
    wp_header: context_predict.weighted.Header = context_predict.weighted.Header.defaults,
    allocator: Allocator,

    fn init(allocator: Allocator, image: Image, stream_id: usize, section_index: usize) StreamData {
        return .{ .image = image, .stream_id = stream_id, .section_index = section_index, .allocator = allocator };
    }

    fn deinit(self: *StreamData) void {
        self.image.deinit();
        for (self.transform_list.items) |*t| t.deinit();
        self.transform_list.deinit(self.allocator);
    }
};

/// Encoder state for one frame.
const FrameEncoder = struct {
    allocator: Allocator,
    input: InputImage,
    opts: EncodeOptions,
    metadata: ImageMetadata,
    frame: FrameHeader,
    fd: stream.FrameDim,
    /// Channel planes in modular order: color first, then extra channels.
    full_image: Image = undefined,
    /// Global-stream transform list (applied to `full_image`).
    global_transforms: std.ArrayList(transforms.Transform) = .empty,
    /// Non-global streams (AC groups, DC groups) with their local transforms.
    group_streams: std.ArrayList(StreamData) = .empty,
    tokens: std.ArrayList(std.ArrayList(Token)) = .empty,
    /// Per-stream max channel widths (LZ77 distance multipliers).
    image_widths: std.ArrayList(usize) = .empty,
    num_streams: usize = 0,
    tree: ma.Tree = .empty,
    tree_tokens: std.ArrayList(Token) = .empty,
    /// Weighted-predictor header of the global stream (the WP-mode search
    /// result when the global stream gets the local pass — single-group
    /// frames — otherwise the default mode 0).
    global_wp_header: context_predict.weighted.Header = context_predict.weighted.Header.defaults,
    modular_options: options.ModularOptions = .{},
    /// Speed-tier parameters for `opts.effort` (computed once in init).
    tier: TierParams = undefined,
    /// Lossy VarDCT companions: force xyb_encoded metadata (the patch
    /// reference frame of a lossy codestream) and custom DequantDC values
    /// (the frame's DC quants, written instead of the all_default bit).
    force_xyb_metadata: bool = false,
    custom_dequant_dc: ?[3]f32 = null,

    fn init(allocator: Allocator, input: InputImage, opts: EncodeOptions) FrameEncoder {
        var fe = FrameEncoder{
            .allocator = allocator,
            .input = input,
            .opts = opts,
            .metadata = ImageMetadata.init(allocator),
            .frame = FrameHeader.init(allocator),
            .fd = undefined,
        };
        fe.tier = tierParams(opts.effort);
        fe.full_image = Image.create(allocator);
        return fe;
    }

    /// True for float-sample input.
    fn isFloat(self: *const FrameEncoder) bool {
        return self.input == .float;
    }

    /// The float format of float input: (bits, exp_bits).
    fn floatBits(self: *const FrameEncoder) struct { bits: u6, exp: u6 } {
        if (self.opts.float16) return .{ .bits = 16, .exp = 5 };
        return .{ .bits = 32, .exp = 8 };
    }

    /// Input dimensions.
    fn inXsize(self: *const FrameEncoder) usize {
        return switch (self.input) {
            .int => |pb| pb.xsize,
            .float => |fb| fb.xsize,
            .planes => |mp| mp.xsize,
        };
    }

    fn inYsize(self: *const FrameEncoder) usize {
        return switch (self.input) {
            .int => |pb| pb.ysize,
            .float => |fb| fb.ysize,
            .planes => |mp| mp.ysize,
        };
    }

    fn inGray(self: *const FrameEncoder) bool {
        return switch (self.input) {
            .int => |pb| pb.layout == .gray,
            .float => |fb| fb.layout == .gray,
            .planes => |mp| mp.gray,
        };
    }

    fn deinit(self: *FrameEncoder) void {
        self.metadata.deinit();
        self.frame.deinit();
        self.full_image.deinit();
        for (self.global_transforms.items) |*t| t.deinit();
        self.global_transforms.deinit(self.allocator);
        for (self.group_streams.items) |*gs| gs.deinit();
        self.group_streams.deinit(self.allocator);
        for (self.tokens.items) |*list| {
            list.deinit(self.allocator);
        }
        self.tokens.deinit(self.allocator);
        self.image_widths.deinit(self.allocator);
        self.tree.deinit(self.allocator);
        self.tree_tokens.deinit(self.allocator);
    }

    fn buildMetadata(self: *FrameEncoder) !void {
        const m = &self.metadata;
        m.xyb_encoded = self.force_xyb_metadata;
        switch (self.input) {
            .int => |pb| {
                m.bit_depth.bits_per_sample = pb.bitdepth;
                m.modular_16_bit_buffer_sufficient = pb.bitdepth <= 16;
            },
            .planes => |mp| {
                m.bit_depth.bits_per_sample = mp.bitdepth;
                m.modular_16_bit_buffer_sufficient = mp.bitdepth <= 16;
            },
            .float => {
                const fb = self.floatBits();
                m.bit_depth.floating_point_sample = true;
                m.bit_depth.bits_per_sample = fb.bits;
                m.bit_depth.exponent_bits_per_sample = fb.exp;
                // fp16 patterns fit a 16-bit buffer; fp32 patterns need 32.
                m.modular_16_bit_buffer_sufficient = fb.bits <= 16;
            },
        }
        if (self.opts.icc.len > 0) {
            // libjxl SetICCRaw: only the flag and the color space are
            // serialized in the bundle; the profile bytes go to the
            // codestream-level ICC stream.
            m.color_encoding = color_mod.ColorEncoding.wantIcc(self.inGray());
            try m.icc.appendSlice(self.allocator, self.opts.icc);
        } else {
            m.color_encoding = if (self.inGray())
                color_mod.ColorEncoding.graySrgb()
            else
                color_mod.ColorEncoding.srgb();
        }
        if (self.input == .int and self.input.int.has_alpha) {
            const pb = self.input.int;
            var eci = metadata_mod.ExtraChannelInfo{};
            eci.all_default = false;
            eci.type = .alpha;
            eci.bit_depth.bits_per_sample = pb.bitdepth;
            try m.extra_channel_info.append(self.allocator, eci);
        }
        if (self.input == .planes and self.input.planes.num_extra > 0) {
            const mp = self.input.planes;
            var eci = metadata_mod.ExtraChannelInfo{};
            eci.all_default = false;
            eci.type = .alpha;
            eci.bit_depth.bits_per_sample = mp.bitdepth;
            for (0..mp.num_extra) |_| {
                try m.extra_channel_info.append(self.allocator, eci);
            }
        }
    }

    fn buildFrameHeader(self: *FrameEncoder) !void {
        const fh = &self.frame;
        fh.encoding = .modular;
        fh.color_transform = .none;
        fh.group_size_shift = self.opts.group_size_shift;
        fh.is_last = true;
        fh.loop_filter = .{ .gab = false, .epf_iters = 0 };
        fh.passes = .{};
        if (self.opts.progressive) {
            // cjxl -p (lossless): qprogressive_mode — the
            // `progressive_passes_dc_quant_ac_full_ac` mode, a 2-pass
            // layout with (downsample, last_pass) = (2, 0)
            // (libjxl enc_progressive_split.h InitPasses).
            fh.passes.num_passes = 2;
            fh.passes.num_downsample = 1;
            fh.passes.shift = .{ 1, 0, 0, 0 };
            fh.passes.downsample = .{ 2, 1, 1, 1 };
            fh.passes.last_pass = .{ 0, 0, 0, 0 };
        }
        const n_extra = self.metadata.extra_channel_info.items.len;
        try fh.extra_channel_blending_info.resize(self.allocator, n_extra);
        try fh.extra_channel_upsampling.resize(self.allocator, n_extra);
        @memset(fh.extra_channel_blending_info.items, .{});
        // cjxl default: ec_resampling follows resampling.
        const ec_resampling = if (self.opts.ec_resampling != 0) self.opts.ec_resampling else self.opts.resampling;
        @memset(fh.extra_channel_upsampling.items, ec_resampling);
        fh.upsampling = self.opts.resampling;
    }

    /// Channel values are raw samples; channels are full-frame planes.
    /// With `resampling` (or a distinct ec_resampling) the input planes are
    /// first converted to floats and box-downsampled exactly like libjxl's
    /// DownsampleColorChannels (DoDownsampleImage: f32 sum / count), then
    /// quantized back to integer samples with `v * maxval + 0.5` (for
    /// integer input) or converted to the custom-float bit pattern (for
    /// float input, enc_modular float_to_int).
    fn buildChannels(self: *FrameEncoder) !void {
        const n_color: usize = if (self.inGray()) 1 else 3;
        const has_alpha = (self.input == .int and self.input.int.has_alpha) or
            (self.input == .planes and self.input.planes.num_extra > 0);
        const n_total = n_color + @intFromBool(has_alpha);
        // (Cleanup on error is handled by FrameEncoder.deinit, which always
        // frees whatever channels were appended so far.)

        const resampling = self.opts.resampling;
        const ec_resampling = if (self.opts.ec_resampling != 0) self.opts.ec_resampling else resampling;

        // Full-resolution sample getters.
        const GetSample = struct {
            fe: *FrameEncoder,
            /// Value in the nominal [0, 1] domain for integer input, raw
            /// float for float input.
            fn atFloat(gs: @This(), x: usize, y: usize, c: usize) f32 {
                switch (gs.fe.input) {
                    .int => |pb| {
                        const factor: f32 = @floatCast(1.0 / @as(f64, @floatFromInt((@as(u64, 1) << @intCast(pb.bitdepth)) - 1)));
                        return @as(f32, @floatFromInt(pb.at(x, y, c))) * factor;
                    },
                    .planes => |mp| {
                        const factor: f32 = @floatCast(1.0 / @as(f64, @floatFromInt((@as(u64, 1) << @intCast(mp.bitdepth)) - 1)));
                        return @as(f32, @floatFromInt(mp.planes[c][y * mp.xsize + x])) * factor;
                    },
                    .float => |fb| return fb.at(x, y, c),
                }
            }
            fn atInt(gs: @This(), x: usize, y: usize, c: usize) modular_common.pixel_type {
                return switch (gs.fe.input) {
                    .int => |pb| pb.at(x, y, c),
                    .planes => |mp| mp.planes[c][y * mp.xsize + x],
                    // atInt is only called for non-float input.
                    .float => unreachable,
                };
            }
        };
        const getter = GetSample{ .fe = self };

        const in_w = self.inXsize();
        const in_h = self.inYsize();

        var c: usize = 0;
        while (c < n_total) : (c += 1) {
            const factor: u32 = if (c < n_color) resampling else ec_resampling;
            const ch_w = std.math.divCeil(usize, in_w, factor) catch 1;
            const ch_h = std.math.divCeil(usize, in_h, factor) catch 1;
            const ch = try Channel.create(self.allocator, ch_w, ch_h, 0, 0);
            try self.full_image.channels.append(self.allocator, ch);
            const dst = self.full_image.channels.items[c];

            const sample_bits: u6 = switch (self.input) {
                .int => |pb| @intCast(pb.bitdepth),
                .planes => |mp| @intCast(mp.bitdepth),
                .float => self.floatBits().bits,
            };
            const exp_bits: u6 = switch (self.input) {
                .int => 0,
                .planes => 0,
                .float => self.floatBits().exp,
            };
            const is_fp = self.isFloat();
            const maxval_f64: f64 = @floatFromInt((@as(u64, 1) << @intCast(sample_bits)) - 1);

            for (0..ch_h) |y| {
                for (0..ch_w) |x| {
                    if (factor == 1) {
                        // No downsampling: integer input keeps its exact
                        // samples; float input converts to the bit pattern.
                        dst.row(y)[x] = if (is_fp)
                            try float_convert.floatToInt(getter.atFloat(x, y, c), sample_bits, exp_bits)
                        else
                            getter.atInt(x, y, c);
                        continue;
                    }
                    // Box average over the factor x factor input block
                    // (f32 accumulation like DoDownsampleImage).
                    var sum: f32 = 0;
                    var cnt: u32 = 0;
                    var iy: usize = 0;
                    while (iy < factor) : (iy += 1) {
                        const yy = y * factor + iy;
                        if (yy >= in_h) break;
                        var ix: usize = 0;
                        while (ix < factor) : (ix += 1) {
                            const xx = x * factor + ix;
                            if (xx >= in_w) break;
                            sum = sum + getter.atFloat(xx, yy, c);
                            cnt += 1;
                        }
                    }
                    const avg = sum / @as(f32, @floatFromInt(cnt));
                    dst.row(y)[x] = if (is_fp)
                        try float_convert.floatToInt(avg, sample_bits, exp_bits)
                    else
                        float_convert.floatToIntQuantize(avg, maxval_f64, sample_bits);
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Global transforms (libjxl ComputeEncodingData, lossless squirrel path)
    // -----------------------------------------------------------------------

    /// First non-meta channel bigger than the group size.
    fn firstBigChannel(self: *const FrameEncoder) usize {
        var c: usize = self.full_image.nb_meta_channels;
        while (c < self.full_image.channels.items.len) : (c += 1) {
            const fc = &self.full_image.channels.items[c];
            if (fc.w > self.fd.group_dim or fc.h > self.fd.group_dim) break;
        }
        return c;
    }

    /// The bit depth the transform heuristics operate on (libjxl
    /// `gi.bitdepth`: the float width for float input).
    fn transformBitDepth(self: *const FrameEncoder) u32 {
        return switch (self.input) {
            .int => |pb| pb.bitdepth,
            .planes => |mp| mp.bitdepth,
            .float => self.floatBits().bits,
        };
    }

    /// RCT headroom guard. Integer input follows the level-5 semantics the
    /// port always used (`bitdepth + 1 < 16`); float input follows libjxl's
    /// level-10 cap (`max_bitdepth + 1 < 32`): fp16 allows transforms,
    /// fp32 (bit patterns spanning the full range) does not.
    fn rctAllowed(self: *const FrameEncoder) bool {
        if (self.isFloat()) {
            return self.transformBitDepth() + 1 < 32;
        }
        return self.transformBitDepth() + 1 < 16;
    }

    fn squeezeAllowed(self: *const FrameEncoder) bool {
        if (self.isFloat()) {
            return self.transformBitDepth() + 2 < 32;
        }
        return self.transformBitDepth() + 2 < 16;
    }

    fn applyGlobalTransforms(self: *FrameEncoder) !void {
        const ctx = enc_transforms.TransformContext{ .bit_depth = self.transformBitDepth() };
        // Global palettes (skipped for responsive lossless, as libjxl does:
        // `!(responsive && lossless)`). Lightning (effort 1) only runs the
        // pre-transform pass for >8-bit images (libjxl
        // `speed_tier <= kThunder || bits_per_sample > 8`).
        const tier = self.tier;
        const pretransform_ok = (self.opts.effort >= 2) or self.transformBitDepth() > 8;
        if (!self.opts.responsive and
            (self.opts.palette_colors != 0 or self.opts.channel_colors_global_percent > 0) and
            pretransform_ok)
        {
            try enc_transforms.tryPalettes(
                self.allocator,
                &self.full_image,
                .{
                    .palette_colors = self.opts.palette_colors,
                    .channel_colors_percent = self.opts.channel_colors_global_percent,
                    .reverts = tier.palette_reverts,
                    .real_cost = tier.real_palette_cost,
                },
                ctx,
                &self.global_transforms,
            );
        }

        // Global RCT (only when there are three color channels and spare
        // bit headroom). libjxl: a fixed colorspace always gets its RCT;
        // the auto colorspace (YCoCg) applies when the local RCT search
        // does not run — responsive frames (the search is disabled there)
        // and the faster tiers (cheetah and up, `speed_tier > kHare`).
        const n_nonmeta = self.full_image.channels.items.len - self.full_image.nb_meta_channels;
        if (n_nonmeta >= 3 and self.rctAllowed()) {
            const ycocg = self.opts.responsive or self.opts.effort <= 4;
            if (self.opts.colorspace < 0 and ycocg) {
                // libjxl: with Squeeze, RCT selection is bugged; YCoCg works.
                var t = transforms.Transform.init(self.allocator);
                defer t.deinit();
                t.id = .rct;
                t.begin_c = @intCast(self.full_image.nb_meta_channels);
                t.rct_type = 6;
                _ = try enc_transforms.doTransform(self.allocator, &self.full_image, &t, ctx, &self.global_transforms);
            } else if (self.opts.colorspace > 0) {
                var t = transforms.Transform.init(self.allocator);
                defer t.deinit();
                t.id = .rct;
                t.begin_c = @intCast(self.full_image.nb_meta_channels);
                t.rct_type = @intCast(self.opts.colorspace);
                _ = try enc_transforms.doTransform(self.allocator, &self.full_image, &t, ctx, &self.global_transforms);
            }
        }

        // Global squeeze (responsive).
        if (self.opts.responsive and self.full_image.channels.items.len > 0 and self.squeezeAllowed()) {
            // Check the default squeeze script against the shift cap.
            var scratch = transforms.Transform.init(self.allocator);
            defer scratch.deinit();
            try transforms.defaultSqueezeParametersInto(&self.full_image, &scratch.squeezes);
            var t = transforms.Transform.init(self.allocator);
            defer t.deinit();
            t.id = .squeeze;
            if (scratch.squeezes.items.len > 0) {
                var truncated = try enc_transforms.truncateDefaultSqueeze(
                    self.allocator,
                    scratch.squeezes.items,
                    self.opts.group_size_shift,
                );
                defer truncated.deinit(self.allocator);
                if (truncated.items.len != scratch.squeezes.items.len) {
                    try t.squeezes.appendSlice(self.allocator, truncated.items);
                }
            }
            _ = try enc_transforms.doTransform(self.allocator, &self.full_image, &t, ctx, &self.global_transforms);
        }
    }

    /// The per-stream local transform pass (libjxl PrepareStreamParams):
    /// local palettes at the group percentage, the local RCT search and the
    /// weighted-predictor mode search; returns this stream's WP header.
    fn applyLocalTransforms(self: *FrameEncoder, image: *Image, transform_list: *std.ArrayList(transforms.Transform)) !context_predict.weighted.Header {
        const ctx = enc_transforms.TransformContext{ .bit_depth = self.transformBitDepth() };
        const tier = self.tier;
        if (!self.opts.responsive and
            (self.opts.palette_colors != 0 or self.opts.channel_colors_group_percent > 0) and
            tier.local_palettes)
        {
            try enc_transforms.tryPalettes(
                self.allocator,
                image,
                .{
                    .palette_colors = self.opts.palette_colors,
                    .channel_colors_percent = self.opts.channel_colors_group_percent,
                    .reverts = tier.palette_reverts,
                    .real_cost = tier.real_palette_cost,
                },
                ctx,
                transform_list,
            );
        }
        // Lossless, no fixed colorspace, not responsive: search RCTs.
        // The RCT headroom guard mirrors libjxl's "force no group RCTs if
        // we don't have a spare bit" (max_bitdepth + 1 > level cap forces
        // colorspace = 0): fp32 patterns get no local RCT search, fp16 does.
        if (!self.opts.responsive and self.opts.colorspace < 0 and self.rctAllowed()) {
            try enc_transforms.tryLocalRct(self.allocator, image, tier.nb_rcts, transform_list);
        }
        // Weighted-predictor mode search (libjxl: kTortoise/kGlacier try 5
        // modes, kKitten 2, the rest keeps mode 0).
        if (tier.wp_modes > 1) {
            return try enc_transforms.searchWpMode(self.allocator, image, tier.wp_modes);
        }
        return context_predict.weighted.Header.defaults;
    }

    // -----------------------------------------------------------------------
    // Stream splitting (libjxl AddModularGroupImage / PrepareStreamParams)
    // -----------------------------------------------------------------------

    /// Channel-rect copies for one group stream, mirroring the decoder's
    /// selection: channels from the first big one on whose min-shift is in
    /// [min_shift, max_shift], rect shifted and clamped per channel.
    fn buildGroupImage(
        self: *FrameEncoder,
        rect: stream.Rect,
        min_shift: i32,
        max_shift: i32,
    ) !?Image {
        const beginc = self.firstBigChannel();
        var gi = Image.create(self.allocator);
        errdefer gi.deinit();
        for (beginc..self.full_image.channels.items.len) |ci| {
            const fc = &self.full_image.channels.items[ci];
            const shift = @min(fc.hshift, fc.vshift);
            if (shift > max_shift) continue;
            if (shift < min_shift) continue;
            const hshift: usize = if (fc.hshift > 0) @intCast(fc.hshift) else 0;
            const vshift: usize = if (fc.vshift > 0) @intCast(fc.vshift) else 0;
            const r = (stream.Rect{
                .x0 = rect.x0 >> @intCast(hshift),
                .y0 = rect.y0 >> @intCast(vshift),
                .xsize = rect.xsize >> @intCast(hshift),
                .ysize = rect.ysize >> @intCast(vshift),
            }).intersect(fc.w, fc.h);
            if (r.xsize == 0 or r.ysize == 0) continue;
            var gc = try Channel.create(self.allocator, r.xsize, r.ysize, fc.hshift, fc.vshift);
            for (0..r.ysize) |y| {
                @memcpy(gc.row(y), fc.row(r.y0 + y)[r.x0 .. r.x0 + r.xsize]);
            }
            try gi.channels.append(self.allocator, gc);
        }
        if (gi.channels.items.len == 0) {
            gi.deinit();
            return null;
        }
        return gi;
    }

    fn buildStreams(self: *FrameEncoder) !void {
        const fd = &self.fd;
        const group_dim = fd.group_dim;
        const num_passes: usize = if (self.opts.progressive) 2 else 1;
        self.num_streams = stream.ModularStreamId.num(fd, num_passes);

        try self.tokens.resize(self.allocator, self.num_streams);
        for (self.tokens.items) |*list| list.* = .empty;

        const all_in_global = self.fd.xsize <= group_dim and self.fd.ysize <= group_dim;
        if (all_in_global) {
            // Single group: everything decodes in the global stream, which
            // then also gets the local transform pass (libjxl pushes a
            // Global stream param when there is exactly one group).
            // Multi-pass frames still emit the per-pass AC group sections
            // (empty) in the TOC.
            self.global_wp_header = try self.applyLocalTransforms(&self.full_image, &self.global_transforms);
            return;
        }

        // AC groups: one section per (pass, group), each carrying the
        // channels whose min-shift falls inside the pass bracket
        // (libjxl enc_modular.cc per-pass AddModularGroupImage).
        for (0..num_passes) |pass| {
            var min_shift: i32 = 0;
            var max_shift: i32 = 0;
            self.frame.passes.getDownsamplingBracket(pass, &min_shift, &max_shift);
            for (0..fd.num_groups) |g| {
                const stream_id = stream.ModularStreamId.modularAC(g, pass).id(fd);
                const sec_index = toc_mod.acGroupIndex(pass, g, fd.num_groups, fd.num_dc_groups);
                const rect = fd.modularGroupRect(g);
                const maybe_gi = try self.buildGroupImage(rect, min_shift, max_shift);
                const gi = maybe_gi orelse continue;
                var sd = StreamData.init(self.allocator, gi, stream_id, sec_index);
                errdefer sd.deinit();
                sd.wp_header = try self.applyLocalTransforms(&sd.image, &sd.transform_list);
                try self.group_streams.append(self.allocator, sd);
            }
        }
        // DC groups: shift bracket [3, infinity). Empty for images that
        // fit in one DC group (2048px) — the shift>=3 channels are then all
        // smaller than a group and decoded in the global stream.
        for (0..fd.num_dc_groups) |g| {
            const stream_id = stream.ModularStreamId.modularDC(g).id(fd);
            const sec_index = 1 + g;
            const rect = fd.modularDcGroupRect(g);
            const maybe_gi = try self.buildGroupImage(rect, 3, 1000);
            const gi = maybe_gi orelse continue;
            var sd = StreamData.init(self.allocator, gi, stream_id, sec_index);
            errdefer sd.deinit();
            sd.wp_header = try self.applyLocalTransforms(&sd.image, &sd.transform_list);
            try self.group_streams.append(self.allocator, sd);
        }
    }

    fn trainAndTokenize(self: *FrameEncoder) !void {
        const tier = self.tier;
        self.modular_options.max_chan_size = self.fd.group_dim;

        // Tree: fixed trees for the fast tiers (libjxl ComputeTree's
        // `speed_tier >= kFalcon` branch), the learned tree otherwise.
        if (tier.fixed_tree) |kind| {
            // Total pixels over the tree-relevant streams (libjxl counts the
            // channels up to the first one bigger than the group size).
            const bitdepth = self.transformBitDepth();
            var total_pixels: usize = self.treeTotalPixels(&self.full_image);
            for (self.group_streams.items) |*gs| {
                total_pixels += self.treeTotalPixels(&gs.image);
            }
            switch (kind) {
                .gradient_leaf => {
                    try self.tree.append(self.allocator, ma.PropertyDecisionNode.leaf(.gradient, 0, 1));
                },
                .gradient_fixed => {
                    self.tree = try makeFixedTree(self.allocator, @intCast(context_predict.kGradientProp), &fixed_tree_cutoffs, .gradient, total_pixels, bitdepth);
                },
                .wp_fixed => {
                    self.tree = try makeFixedTree(self.allocator, @intCast(context_predict.kWPProp), &fixed_tree_cutoffs, .weighted, total_pixels, bitdepth);
                },
            }
        } else {
            // libjxl non-responsive `prop_order` sliced per speed tier, with
            // the group-id property dropped for few-stream frames at tiers
            // above kTortoise.
            const prop_order = [_]u32{ 0, 1, 15, 9, 10, 11, 12, 13, 14, 2, 3, 4, 5, 6, 7, 8 };
            const n = tier.n_props;
            var props_buf: [prop_order.len]u32 = undefined;
            @memcpy(props_buf[0..n], prop_order[0..n]);
            var n_props = n;
            if (self.num_streams < 30 and tier.drop_group_prop_few_streams) {
                // Drop property 1 (group id).
                std.mem.copyForwards(u32, props_buf[1 .. n - 1], props_buf[2..n]);
                n_props = n - 1;
            }
            self.modular_options.splitting_heuristics_properties = props_buf[0..n_props];
            self.modular_options.max_property_values = tier.max_property_values;
            self.modular_options.splitting_heuristics_node_threshold = tier.node_threshold;

            var train_streams: std.ArrayList(stream.TrainStream) = .empty;
            defer train_streams.deinit(self.allocator);
            try train_streams.append(self.allocator, .{
                .image = &self.full_image,
                .group_id = 0,
                .wp_header = &self.global_wp_header,
            });
            for (self.group_streams.items) |*gs| {
                try train_streams.append(self.allocator, .{
                    .image = &gs.image,
                    .group_id = gs.stream_id,
                    .wp_header = &gs.wp_header,
                });
            }

            self.tree = try stream.trainTree(self.allocator, train_streams.items, &self.modular_options);
        }
        var decoded_tree = ma.Tree.empty;
        try stream.tokenizeTree(self.allocator, self.tree.items, &self.tree_tokens, &decoded_tree);
        self.tree.deinit(self.allocator);
        self.tree = decoded_tree;

        // Tokenize every stream with the decoder-side tree and the stream's
        // own weighted-predictor header (the WP-mode search result).
        try stream.encodeStreamTokens(self.allocator, &self.full_image, 0, self.fd.group_dim, self.tree.items, &self.global_wp_header, &self.tokens.items[0]);
        for (self.group_streams.items) |*gs| {
            try stream.encodeStreamTokens(self.allocator, &gs.image, gs.stream_id, self.fd.group_dim, self.tree.items, &gs.wp_header, &self.tokens.items[gs.stream_id]);
        }

        // Per-stream distance multipliers for the LZ77 RLE encoder: the
        // decoder activates the special-distance table with the maximum
        // decodable channel width of each stream (libjxl fills
        // `HistogramParams::image_widths` the same way), so the encoder
        // must use the same values or the RLE distance symbols resolve to
        // the wrong distances on the decoder side.
        self.image_widths = .empty;
        try self.image_widths.resize(self.allocator, self.num_streams);
        @memset(self.image_widths.items, 0);
        self.image_widths.items[0] = stream.maxStreamChannelWidth(&self.full_image, self.fd.group_dim);
        for (self.group_streams.items) |*gs| {
            self.image_widths.items[gs.stream_id] = stream.maxStreamChannelWidth(&gs.image, self.fd.group_dim);
        }
    }

    /// Pixels of the channels that a stream's tree covers (libjxl
    /// ComputeTree: every channel up to the first one bigger than the group
    /// size; the fixed trees only need the total for the small-image height
    /// reduction).
    fn treeTotalPixels(self: *const FrameEncoder, image: *const Image) usize {
        var total: usize = 0;
        for (image.channels.items, 0..) |*ch, c| {
            if (c >= image.nb_meta_channels and (ch.w > self.fd.group_dim or ch.h > self.fd.group_dim)) break;
            total += ch.w * ch.h;
        }
        return @max(total, 1);
    }
};

const Rect = stream.Rect;


/// Validates shared encode options.
fn checkOpts(opts: EncodeOptions) EncodeError!void {
    if (opts.effort < 1 or opts.effort > 10) return error.InvalidEffort;
    if (opts.resampling != 1 and opts.resampling != 2 and opts.resampling != 4 and opts.resampling != 8) {
        return error.InvalidResampling;
    }
    const ec = if (opts.ec_resampling != 0) opts.ec_resampling else opts.resampling;
    if (ec != 1 and ec != 2 and ec != 4 and ec != 8) return error.InvalidResampling;
    if (ec < opts.resampling) return error.InvalidResampling; // spec rule
    if (opts.progressive and opts.resampling != 1) {
        // cjxl keeps -p orthogonal to --resampling in the lossless path;
        // the pass brackets assume un-resampled channel geometry.
        return error.InvalidResampling;
    }
}

/// Encodes a full JXL codestream (raw, no container) for `pixels`.
/// Sample accessor for `lossy_mod.buildXybPlanes`.
fn pixelAt(pb: *const PixelBuffer, x: usize, y: usize, c: usize) u16 {
    return pb.at(x, y, c);
}

/// Lossy VarDCT encode (cjxl `-d D`, the XYB patch subset of
/// vardct/lossy.zig): sRGB integer input -> XYB opsin -> patch search +
/// reference frame + subtraction -> gaborish -> DCT-8 with the Libshroom
/// weights as a RAW quant table, constant quant field. RGB input only (no
/// alpha); patches default on at effort >= 7 like libjxl's
/// `LossyFrameHeuristics` gate.
pub fn encodeLossy(allocator: Allocator, pixels: *const PixelBuffer, opts: EncodeOptions, writer: *BitWriter) EncodeError!void {
    try checkOpts(opts);
    if (opts.distance <= 0) return error.InvalidDistance;
    if (pixels.layout != .rgb) return error.UnsupportedLossyInput;
    if (pixels.has_alpha) return error.UnsupportedLossyInput;
    if (pixels.bitdepth == 0 or pixels.bitdepth > 16) return error.UnsupportedBitDepth;
    if (pixels.xsize == 0 or pixels.ysize == 0) return error.UnsupportedDimensions;

    const xsize = pixels.xsize;
    const ysize = pixels.ysize;

    // 1. XYB opsin planes (zero-padded to whole blocks).
    var xyb = try lossy_mod.buildXybPlanes(allocator, xsize, ysize, pixels.bitdepth, pixels, pixelAt);
    defer for (&xyb) |pl| allocator.free(pl);

    // 2. Patch phase (libjxl enc_heuristics.cc LossyFrameHeuristics:
    // ApplyOverride(patches, speed_tier <= kSquirrel)).
    const patches_enabled = switch (opts.patches) {
        .off => false,
        .on => true,
        .default => opts.effort >= 7,
    };
    var phase = PatchPhase.init(allocator);
    defer phase.deinit();
    var packed_patches: ?enc_patches.PackedPatchesXyb = null;
    defer if (packed_patches) |*pp| pp.deinit(allocator);

    if (patches_enabled) {
        var info = try enc_patches.findTextLikePatchesXyb(
            allocator,
            &xyb,
            xsize,
            ysize,
            opts.patches,
        );
        if (info.items.len != 0) {
            // Ownership moves to packPatchesXyb (it frees on all paths).
            packed_patches = try enc_patches.packPatchesXyb(allocator, &info, 0, opts.distance);
        } else {
            info.deinit(allocator);
        }
    }

    if (packed_patches != null and packed_patches.?.positions.items.len > 0) {
        const pp = &packed_patches.?;
        // Reference store slot 3: the reconstructed reference floats in
        // color order (exactly what a decoder computes from the frame we
        // are about to write).
        {
            const n_pix = pp.ref_xsize * pp.ref_ysize;
            const plane_list = try allocator.alloc([]f32, 3);
            var made: usize = 0;
            errdefer {
                for (plane_list[0..made]) |q| allocator.free(q);
                allocator.free(plane_list);
            }
            for (0..3) |c| {
                const q = try allocator.alloc(f32, n_pix);
                made += 1;
                @memcpy(q, pp.ref_floats[c]);
                plane_list[c] = q;
            }
            phase.refs[patches_mod.kPatchFrameReferenceId] = .{
                .planes = plane_list,
                .xsize = pp.ref_xsize,
                .ysize = pp.ref_ysize,
                .before_color_transform = true,
            };
        }
        // Dictionary + subtraction on the raw XYB planes.
        try phase.pd.setPositions(pp.positions.items, pp.ref_positions.items, pp.blendings.items, 1);
        phase.pd.refs = &phase.refs;
        try lossy_mod.subtractPatchesXyb(allocator, &phase.pd, xyb, xsize);
        phase.found = true;

        // Reference frame bytes.
        {
            var rw = BitWriter.init(allocator);
            errdefer rw.deinit();
            try writePatchReferenceFrameXyb(allocator, pp, pixels.bitdepth, &rw);
            rw.zeroPadToByte();
            phase.ref_frame = try rw.takeBytes();
            rw.deinit();
        }
    }

    // 3. Frame data (gaborish + DCT + quantization).
    var data = try lossy_mod.buildFrameData(allocator, &xyb, xsize, ysize, opts.distance);
    defer data.deinit();

    // 4. Codestream: headers, reference frame, main frame.
    try lossy_mod.encodeLossyFrame(
        allocator,
        &data,
        xsize,
        ysize,
        pixels.bitdepth,
        phase.ref_frame,
        if (phase.found) &phase.pd else null,
        .{
            .distance = opts.distance,
            .patches = opts.patches,
            .effort = opts.effort,
            .coeff_orders = opts.effort >= 3,
        },
        writer,
    );
}

/// Encodes the XYB patch reference image as its own codestream frame: a
/// `kReferenceOnly` modular frame saved into slot 3 before the color
/// transform, carrying the [Y, X, B-Y] planes quantized onto the dc_quant
/// grid and the matching custom DequantDC (libjxl `RoundtripPatchFrame` +
/// enc_modular's XYB modular path).
fn writePatchReferenceFrameXyb(
    allocator: Allocator,
    packed_patches: *const enc_patches.PackedPatchesXyb,
    bitdepth: u8,
    writer: *BitWriter,
) anyerror!void {
    const ref_planes = try allocator.alloc([]const i32, 3);
    defer allocator.free(ref_planes);
    for (0..3) |c| {
        ref_planes[c] = packed_patches.ref_planes.items[c];
    }

    var ropts = EncodeOptions{};
    ropts.responsive = false;
    ropts.progressive = false;
    ropts.resampling = 1;
    ropts.ec_resampling = 1;
    ropts.patches = .off;

    const mp = ModularPlanes{
        .planes = ref_planes,
        .xsize = packed_patches.ref_xsize,
        .ysize = packed_patches.ref_ysize,
        .gray = false,
        .num_extra = 0,
        .bitdepth = bitdepth,
    };
    var rfe = FrameEncoder.init(allocator, .{ .planes = &mp }, ropts);
    defer rfe.deinit();
    rfe.force_xyb_metadata = true;
    rfe.custom_dequant_dc = packed_patches.dc_quant;
    try prepareFrame(&rfe, null);

    rfe.frame.flags = 0;
    rfe.frame.frame_type = .reference_only;
    rfe.frame.is_last = false;
    rfe.frame.save_as_reference = @intCast(patches_mod.kPatchFrameReferenceId);
    rfe.frame.save_before_color_transform = true;
    rfe.frame.custom_size_or_origin = true;
    rfe.frame.frame_size_x = @intCast(packed_patches.ref_xsize);
    rfe.frame.frame_size_y = @intCast(packed_patches.ref_ysize);
    rfe.frame.frame_origin_x = 0;
    rfe.frame.frame_origin_y = 0;
    rfe.frame.upsampling = 1;
    rfe.frame.passes = .{};

    try writeFrame(&rfe, writer, null);
}

pub fn encode(allocator: Allocator, pixels: *const PixelBuffer, opts: EncodeOptions, writer: *BitWriter) EncodeError!void {
    try checkOpts(opts);
    var fe = FrameEncoder.init(allocator, .{ .int = pixels }, opts);
    defer fe.deinit();

    if (pixels.bitdepth == 0 or pixels.bitdepth > 16) return error.UnsupportedBitDepth;
    if (pixels.xsize == 0 or pixels.ysize == 0) return error.UnsupportedDimensions;

    var phase = PatchPhase.init(allocator);
    defer phase.deinit();
    try prepareFrame(&fe, &phase);

    // --- Headers + [patch reference frame] + the main frame ---
    try writeCodestreamHeaders(&fe, writer);
    if (phase.ref_frame) |rf| appendFrameBytes(writer, rf);
    try writeFrame(&fe, writer, &phase);
}

/// Encodes a full JXL codestream (raw, no container) for float `pixels`
/// (fp32 by default, fp16 with `opts.float16`; every sample must be
/// exactly representable in the target format).
pub fn encodeFloat(allocator: Allocator, pixels: *const FloatPixelBuffer, opts: EncodeOptions, writer: *BitWriter) EncodeError!void {
    try checkOpts(opts);
    var fe = FrameEncoder.init(allocator, .{ .float = pixels }, opts);
    defer fe.deinit();

    if (pixels.xsize == 0 or pixels.ysize == 0) return error.UnsupportedDimensions;
    if (!opts.float16 and pixels.bits_per_sample != 32) return error.UnsupportedBitDepth;
    if (opts.float16 and pixels.bits_per_sample != 32 and pixels.bits_per_sample != 16) {
        return error.UnsupportedBitDepth;
    }

    var phase = PatchPhase.init(allocator);
    defer phase.deinit();
    try prepareFrame(&fe, &phase);

    // --- Headers + the single frame (float input: patch search runs for
    // fp16, libjxl's `bits_per_sample <= 16` gate; fp32 excluded like
    // libjxl) ---
    try writeCodestreamHeaders(&fe, writer);
    if (phase.ref_frame) |rf| appendFrameBytes(writer, rf);
    try writeFrame(&fe, writer, &phase);
}

pub const AnimationError = error{
    EmptyAnimation,
    MismatchedFrames,
    UnsupportedBitDepth,
    UnsupportedDimensions,
} || EncodeError;

pub const AnimationEncodeOptions = struct {
    /// Per-frame durations in ticks. The animation header is written with
    /// tps = 1000/1, so ticks are milliseconds (cjxl uses 1000/1 for APNG
    /// input, 100/1 for GIF).
    durations: []const u32,
    /// Animation loops (metadata.animation.num_loops, 0 = infinite).
    num_loops: u32 = 0,
};

/// Encodes a sequence of same-sized frames as an animation: the codestream
/// headers once, then one full-canvas modular frame per input (all replace,
/// every non-last frame saved as reference slot 1, like cjxl's coalesced
/// animation output).
pub fn encodeAnimation(
    allocator: Allocator,
    frames: []const *const PixelBuffer,
    anim_opts: AnimationEncodeOptions,
    opts: EncodeOptions,
    writer: *BitWriter,
) AnimationError!void {
    if (frames.len == 0) return error.EmptyAnimation;
    if (anim_opts.durations.len != frames.len) return error.MismatchedFrames;
    const first = frames[0];
    if (first.bitdepth == 0 or first.bitdepth > 16) return error.UnsupportedBitDepth;
    if (first.xsize == 0 or first.ysize == 0) return error.UnsupportedDimensions;
    for (frames[1..]) |f| {
        if (f.xsize != first.xsize or f.ysize != first.ysize or
            f.layout != first.layout or f.has_alpha != first.has_alpha or
            f.bitdepth != first.bitdepth) return error.MismatchedFrames;
    }

    for (frames, 0..) |pixels, i| {
        var fe = FrameEncoder.init(allocator, .{ .int = pixels }, opts);
        defer fe.deinit();

        var phase = PatchPhase.init(allocator);
        defer phase.deinit();
        try prepareFrame(&fe, &phase);

        // Animation metadata (identical for every frame's metadata bundle).
        fe.metadata.have_animation = true;
        fe.metadata.animation.tps_numerator = 1000;
        fe.metadata.animation.tps_denominator = 1;
        fe.metadata.animation.num_loops = anim_opts.num_loops;
        fe.metadata.animation.have_timecodes = false;

        // Frame header overrides: duration, is_last, reference saving.
        fe.frame.animation_frame_duration = anim_opts.durations[i];
        fe.frame.is_last = i == frames.len - 1;
        fe.frame.save_as_reference = if (fe.frame.is_last) 0 else 1;
        fe.frame.save_before_color_transform = false;

        if (i == 0) {
            try writeCodestreamHeaders(&fe, writer);
        }
        if (phase.ref_frame) |rf| appendFrameBytes(writer, rf);
        try writeFrame(&fe, writer, &phase);
    }
}

/// Encoder-side patch artifacts for one frame (everything owned by the
/// phase; freed by `deinit`).
///
/// Pipeline position (libjxl ComputeEncodingData): the search runs on the
/// raw full-resolution color planes after `buildChannels` and before the
/// global transforms; `subtractFrom` then removes the patch content from
/// the frame channels, so the transform heuristics (palette/RCT/squeeze)
/// and the entropy coding see only the background residual.
pub const PatchPhase = struct {
    allocator: Allocator,
    /// The dictionary (positions + blendings) serialized into the DC
    /// global section; `refs` is attached for the encoder-side subtraction.
    pd: patches_mod.PatchDictionary,
    /// Reference-frame store; slot `kPatchFrameReferenceId` (3) holds the
    /// patch reference image as float [0,1] planes — exactly the planes a
    /// decoder of this codestream will store after decoding the reference
    /// frame (the modular path is lossless, so the roundtrip is exact by
    /// construction; libjxl decodes the frame back for the same effect).
    refs: patches_mod.RefStore = .{ null, null, null, null },
    /// Encoded reference-frame bytes (already byte-aligned), written just
    /// before the main frame — libjxl appends the special frames to the
    /// same frame's output (`AppendByteAligned`).
    ref_frame: ?[]u8 = null,
    /// Whether patches were found and applied.
    found: bool = false,

    pub fn init(allocator: Allocator) PatchPhase {
        return .{ .allocator = allocator, .pd = patches_mod.PatchDictionary.init(allocator) };
    }

    pub fn deinit(self: *PatchPhase) void {
        self.pd.deinit();
        patches_mod.freeRefStore(self.allocator, &self.refs);
        if (self.ref_frame) |rf| self.allocator.free(rf);
        self.ref_frame = null;
    }
};

/// Builds metadata, frame header, channels, transforms, streams and tokens
/// for one frame (the shared part of `encode`, `encodeFloat` and
/// `encodeAnimation`). `phase` receives the patch artifacts (the search
/// runs between `buildChannels` and `applyGlobalTransforms`); pass null
/// for frames that must not search (the patch reference frame itself).
fn prepareFrame(fe: *FrameEncoder, phase: ?*PatchPhase) !void {
    // cjxl -p implies responsive: "Modular has to be squeezed to show
    // progressive HF passes" (libjxl enc_frame.cc).
    if (fe.opts.progressive) fe.opts.responsive = true;
    try fe.buildMetadata();
    fe.fd = stream.FrameDim.init(fe.inXsize(), fe.inYsize(), fe.opts.group_size_shift, fe.opts.resampling);
    try fe.buildFrameHeader();
    try fe.buildChannels();
    if (phase) |ph| try tryPatches(fe, ph);
    try fe.applyGlobalTransforms();
    try fe.buildStreams();
    try fe.trainAndTokenize();
}

/// The encoder-side patch search for one frame (libjxl
/// FindBestPatchDictionary, the lossless non-XYB branch): find repeated
/// fragments, bin-pack them into a reference image, encode that image as
/// a `kReferenceOnly` frame saved into slot 3 before the color transform,
/// and subtract the patch content from the frame channels.
fn tryPatches(fe: *FrameEncoder, phase: *PatchPhase) !void {
    // libjxl enc_modular ComputeEncodingData gate: bits_per_sample <= 16
    // (any format — integer or float; binary16 patterns pass, fp32 does
    // not, exactly like libjxl), full-resolution planes (no resampling),
    // color transform none.
    const format: patches_mod.SampleFormat = switch (fe.input) {
        .int => |pb| .{ .bits = pb.bitdepth },
        .planes => |mp| .{ .bits = mp.bitdepth },
        .float => blk: {
            const fb = fe.floatBits();
            break :blk .{ .bits = fb.bits, .exp_bits = fb.exp, .floating = true };
        },
    };
    if (fe.opts.resampling != 1) return;
    if (fe.opts.ec_resampling != 0 and fe.opts.ec_resampling != 1) return;
    if (format.bits == 0 or format.bits > 16) return;
    const enabled = switch (fe.opts.patches) {
        .off => false,
        // The modular call requires speed_tier < kCheetah (effort > 4);
        // by default FindTextLikePatches then also requires
        // speed_tier <= kSquirrel (effort >= 7).
        .on => fe.opts.effort >= 5,
        .default => fe.opts.effort >= 7,
    };
    if (!enabled) return;
    if (fe.full_image.channels.items.len == 0) return;
    const n_color: usize = if (fe.inGray()) 1 else 3;
    const num_ec = fe.metadata.extra_channel_info.items.len;
    const xsize = fe.inXsize();
    const ysize = fe.inYsize();
    if (n_color == 0 or xsize == 0 or ysize == 0) return;

    // Raw color planes (built by buildChannels at resampling 1: exact
    // input samples — integers or float bit patterns — color channels
    // first).
    const planes = try fe.allocator.alloc([]const modular_common.pixel_type, n_color);
    defer fe.allocator.free(planes);
    for (0..n_color) |c| {
        planes[c] = fe.full_image.channels.items[c].plane.data;
    }

    var info = try enc_patches.findTextLikePatches(
        fe.allocator,
        planes,
        n_color,
        xsize,
        ysize,
        format,
        fe.opts.patches,
    );
    if (info.items.len == 0) {
        // Nothing detected (an empty shell with possible spare capacity).
        info.deinit(fe.allocator);
        return;
    }

    // Ownership note: from here on, `info` belongs to packPatches (it
    // frees the entries and the list, including on its error paths).
    var packed_patches = try enc_patches.packPatches(fe.allocator, &info, n_color, num_ec, format);
    defer packed_patches.deinit(fe.allocator);
    if (packed_patches.positions.items.len == 0) return;
    const n_total = n_color + num_ec;

    // Reference store slot 3: the packed reference image as float planes
    // (color content; extra channels zero, like the reference frame's own
    // alpha plane). Integer content scales by 1/maxval; float content
    // converts the bit patterns via int_to_float. This is what the
    // decoder will hold after decoding the reference frame we are about
    // to write.
    {
        if (phase.refs[patches_mod.kPatchFrameReferenceId]) |*old| {
            for (old.planes) |p| fe.allocator.free(p);
            fe.allocator.free(old.planes);
            phase.refs[patches_mod.kPatchFrameReferenceId] = null;
        }
        const n_pixels = packed_patches.ref_xsize * packed_patches.ref_ysize;
        const plane_list = try fe.allocator.alloc([]f32, n_total);
        var initialized: usize = 0;
        errdefer {
            for (plane_list[0..initialized]) |p| fe.allocator.free(p);
            fe.allocator.free(plane_list);
        }
        for (0..n_color) |c| {
            const p = try fe.allocator.alloc(f32, n_pixels);
            initialized += 1;
            for (0..n_pixels) |i| {
                p[i] = patches_mod.sampleToFloat(packed_patches.ref_planes.items[c][i], format);
            }
            plane_list[c] = p;
        }
        while (initialized < n_total) : (initialized += 1) {
            const p = try fe.allocator.alloc(f32, n_pixels);
            @memset(p, 0);
            plane_list[initialized] = p;
        }
        phase.refs[patches_mod.kPatchFrameReferenceId] = .{
            .planes = plane_list,
            .xsize = packed_patches.ref_xsize,
            .ysize = packed_patches.ref_ysize,
            .before_color_transform = true,
        };
    }

    // Dictionary with the packed positions (computes the y-interval tree).
    try phase.pd.setPositions(
        packed_patches.positions.items,
        packed_patches.ref_positions.items,
        packed_patches.blendings.items,
        num_ec + 1,
    );
    phase.pd.refs = &phase.refs;

    // Encode the reference frame (its bytes go right before the main
    // frame; see writeCodestream-order in encode).
    {
        var rw = BitWriter.init(fe.allocator);
        errdefer rw.deinit();
        try writePatchReferenceFrame(fe, &packed_patches, &rw);
        rw.zeroPadToByte();
        phase.ref_frame = try rw.takeBytes();
        rw.deinit();
    }

    // Subtract the patch content from the frame channels (integer domain
    // for integer samples, float arithmetic on the patterns for float
    // samples — exact in both; see PatchDictionary.subtractFrom).
    {
        const formats = try fe.allocator.alloc(patches_mod.SampleFormat, n_total);
        defer fe.allocator.free(formats);
        for (0..n_color) |c| formats[c] = format;
        for (0..num_ec) |i| {
            const ebd = fe.metadata.extra_channel_info.items[i].bit_depth;
            formats[n_color + i] = .{
                .bits = ebd.bits_per_sample,
                .exp_bits = ebd.exponent_bits_per_sample,
                .floating = ebd.floating_point_sample,
            };
        }
        try phase.pd.subtractFrom(&fe.full_image, n_color, formats);
    }

    phase.found = true;
}

/// Encodes the patch reference image as its own codestream frame: a
/// `kReferenceOnly` modular frame saved into slot `kPatchFrameReferenceId`
/// before the color transform, written byte-aligned just ahead of the main
/// frame (libjxl `RoundtripPatchFrame`: modular mode, no squeeze, no
/// progressive, no resampling, recursive patches off; the palette/RCT
/// heuristics carry over from the main cparams).
fn writePatchReferenceFrame(fe: *FrameEncoder, packed_patches: *const enc_patches.PackedPatches, writer: *BitWriter) anyerror!void {
    const allocator = fe.allocator;
    // The sample format of the codestream (integer bit depth, or the
    // float pattern width — the frame inherits the codestream-level bit
    // depth, so float difference patterns travel as float samples).
    const format: patches_mod.SampleFormat = switch (fe.input) {
        .int => |pb| .{ .bits = pb.bitdepth },
        .planes => |mp| .{ .bits = mp.bitdepth },
        .float => blk: {
            const fb = fe.floatBits();
            break :blk .{ .bits = fb.bits, .exp_bits = fb.exp, .floating = true };
        },
    };
    const n_color: usize = if (fe.inGray()) 1 else 3;
    // Float input carries no extra channels (no float alpha); integer
    // input may carry alpha, which the reference frame zero-fills.
    const has_alpha: bool = if (fe.input == .int) fe.input.int.has_alpha else false;
    const num_extra: usize = if (has_alpha) 1 else 0;
    const n_total = n_color + num_extra;

    // The reference image as raw modular planes: color planes carry the
    // patch content (a difference image — negative where the fragment is
    // darker than its background; integer samples or float bit patterns,
    // only the exact values, no clamps), extra channels (alpha)
    // zero-filled so the frame carries the codestream's channel count.
    const ref_planes = try allocator.alloc([]const i32, n_total);
    // Sentinel-init so the cleanup defer is valid at any allocation point.
    for (ref_planes) |*slot| slot.* = &.{};
    defer {
        for (ref_planes) |p| {
            if (p.len != 0) allocator.free(p[0..p.len]);
        }
        allocator.free(ref_planes);
    }
    for (0..n_color) |c| {
        ref_planes[c] = try allocator.dupe(i32, packed_patches.ref_planes.items[c]);
    }
    for (n_color..n_total) |c| {
        const p = try allocator.alloc(i32, packed_patches.ref_xsize * packed_patches.ref_ysize);
        @memset(p, 0);
        ref_planes[c] = p;
    }

    var ropts = fe.opts;
    ropts.responsive = false;
    ropts.progressive = false;
    ropts.resampling = 1;
    ropts.ec_resampling = 1;
    ropts.patches = .off;

    const mp = ModularPlanes{
        .planes = ref_planes,
        .xsize = packed_patches.ref_xsize,
        .ysize = packed_patches.ref_ysize,
        .gray = fe.inGray(),
        .num_extra = num_extra,
        .bitdepth = @intCast(format.bits),
    };
    var rfe = FrameEncoder.init(allocator, .{ .planes = &mp }, ropts);
    defer rfe.deinit();
    try prepareFrame(&rfe, null);

    // Frame-header overrides (libjxl FrameInfo of the patch frame):
    // reference-only, saved into slot 3 before the color transform,
    // never last, its own frame size.
    rfe.frame.flags = 0;
    rfe.frame.frame_type = .reference_only;
    rfe.frame.is_last = false;
    rfe.frame.save_as_reference = @intCast(patches_mod.kPatchFrameReferenceId);
    rfe.frame.save_before_color_transform = true;
    rfe.frame.custom_size_or_origin = true;
    rfe.frame.frame_size_x = @intCast(packed_patches.ref_xsize);
    rfe.frame.frame_size_y = @intCast(packed_patches.ref_ysize);
    rfe.frame.frame_origin_x = 0;
    rfe.frame.frame_origin_y = 0;
    rfe.frame.upsampling = 1;
    rfe.frame.passes = .{};
    for (rfe.frame.extra_channel_upsampling.items) |*ecup| ecup.* = 1;

    try writeFrame(&rfe, writer, null);
}

/// Appends byte-aligned frame bytes (the patch reference frame) to the
/// codestream (libjxl `writer.AppendByteAligned(special_frames)`).
fn appendFrameBytes(writer: *BitWriter, bytes: []const u8) void {
    for (bytes) |b| {
        writer.write(8, b);
    }
}

/// Writes the codestream-level headers: signature, SizeHeader,
/// ImageMetadata, CustomTransformData, optional ICC, byte alignment.
fn writeCodestreamHeaders(fe: *FrameEncoder, writer: *BitWriter) !void {
    writer.write(8, 0xFF);
    writer.write(8, container_mod.raw_signature[1]);
    var sh: size_mod.SizeHeader = .{};
    try sh.set(@intCast(fe.inXsize()), @intCast(fe.inYsize()));
    try sh.write(writer);
    try fe.metadata.write(writer);
    const ctd = metadata_mod.CustomTransformData{};
    try ctd.write(writer);
    if (fe.metadata.color_encoding.want_icc) {
        try enc_icc.writeICC(fe.allocator, fe.metadata.icc.items, writer);
    }
    writer.zeroPadToByte();
}

/// Writes one prepared frame: FrameHeader, TOC and all sections. When
/// `phase` reports found patches, the frame header carries `kPatches` and
/// the DC global section starts with the patch dictionary (libjxl order:
/// patches, splines, noise, dequant DC, modular global info).
fn writeFrame(fe: *FrameEncoder, writer: *BitWriter, phase: ?*PatchPhase) !void {
    const allocator = fe.allocator;
    if (phase) |ph| if (ph.found) {
        fe.frame.flags |= frame_mod.Flags.kPatches;
    };
    try fe.frame.write(&fe.metadata, writer);

    // --- Sections ---
    const num_passes: usize = if (fe.opts.progressive) 2 else 1;
    // Single-section frames are exactly the num_groups == 1 && np == 1
    // shape (toc.numTocEntries); progressive single-group frames still
    // carry the 2 + ndc + np TOC entries with empty pass sections.
    const is_small = fe.fd.num_groups == 1 and num_passes == 1;
    const num_toc_entries = toc_mod.numTocEntries(fe.fd.num_groups, fe.fd.num_dc_groups, num_passes);
    var sections: std.ArrayList(BitWriter) = .empty;
    defer {
        for (sections.items) |*sw| sw.deinit();
        sections.deinit(allocator);
    }
    try sections.resize(allocator, num_toc_entries);
    for (sections.items) |*sw| sw.* = BitWriter.init(allocator);

    const main_section = &sections.items[0];

    // DC global head: the patch dictionary (libjxl
    // PatchDictionaryEncoder::Encode goes first, before the dequant
    // matrices DC bit).
    if (phase) |ph| if (ph.found) {
        try ph.pd.encode(main_section);
    };

    // DC global: dequant DC (all-default bit, or the custom F16 values of
    // an XYB patch reference frame) + global tree + histograms + global
    // stream.
    if (fe.custom_dequant_dc) |dcq| {
        main_section.write(1, 0); // DequantMatrices DC custom
        for (dcq) |q| {
            try fields_ext.writeF16(q * 128.0, main_section);
        }
    } else {
        main_section.write(1, 1); // DequantMatrices DC all_default
    }
    main_section.write(1, 1); // has_tree

    // Tree histograms + tree tokens (6 contexts). The histogram parameters
    // follow the speed tier (libjxl HistogramParams::ForModular).
    var tree_tokens_ans = try toAnsTokens(allocator, fe.tree_tokens.items);
    defer tree_tokens_ans.deinit(allocator);
    var tree_token_streams = [_]std.ArrayList(ANSToken){tree_tokens_ans};
    var tree_code = enc_ans.EntropyEncodingData.init(allocator);
    defer tree_code.deinit();
    var histo_params = fe.tier.histo;
    _ = try stream.buildHistograms(allocator, &histo_params, ma.kNumTreeContexts, tree_token_streams[0..], &tree_code, main_section);
    _ = try stream.writeTokens(allocator, tree_tokens_ans.items, &tree_code, 0, main_section);

    // Stream histograms over all streams' tokens.
    var all_tokens: std.ArrayList(std.ArrayList(ANSToken)) = .empty;
    defer all_tokens.deinit(allocator);
    for (fe.tokens.items) |*list| {
        const conv = try toAnsTokens(allocator, list.items);
        try all_tokens.append(allocator, conv);
    }
    defer for (all_tokens.items) |*l| l.deinit(allocator);

    var stream_code = enc_ans.EntropyEncodingData.init(allocator);
    defer stream_code.deinit();
    // The stream histograms carry the per-stream distance multipliers so the
    // RLE distance symbols match the decoder's special-distance table (the
    // tree token stream keeps a zero multiplier, like libjxl's
    // EncodeGlobalInfo).
    var stream_histo_params = histo_params;
    stream_histo_params.image_widths = fe.image_widths.items;
    _ = try stream.buildHistograms(allocator, &stream_histo_params, (fe.tree.items.len + 1) / 2, all_tokens.items, &stream_code, main_section);

    // Global stream: GroupHeader (with the frame transform list) + tokens.
    {
        var gh = stream.GroupHeader.init(allocator);
        defer gh.deinit();
        gh.use_global_tree = true;
        gh.wp_header = fe.global_wp_header;
        for (fe.global_transforms.items) |*t| {
            try gh.transforms.append(allocator, t.*);
        }
        try gh.write(main_section);
        const sid = stream.ModularStreamId.global().id(&fe.fd);
        _ = try stream.writeTokens(allocator, all_tokens.items[sid].items, &stream_code, 0, main_section);
    }

    if (!is_small) {
        // Group sections (DC groups at 1+g, AC groups after AC global);
        // empty streams write nothing, as libjxl's EncodeStream does.
        for (fe.group_streams.items) |*gs| {
            const sec = &sections.items[gs.section_index];
            var gh = stream.GroupHeader.init(allocator);
            defer gh.deinit();
            gh.use_global_tree = true;
            gh.wp_header = gs.wp_header;
            for (gs.transform_list.items) |*t| {
                try gh.transforms.append(allocator, t.*);
            }
            try gh.write(sec);
            _ = try stream.writeTokens(allocator, all_tokens.items[gs.stream_id].items, &stream_code, 0, sec);
        }
    }

    // Zero-pad every section (single-section images have nothing extra).
    for (sections.items) |*sw| {
        sw.zeroPadToByte();
    }

    // --- TOC + section bytes ---
    // `--group-order` (cjxl `--group_order=1`, libjxl `centerfirst`): the
    // center-first section permutation; sections are then emitted in
    // physical order (libjxl PermuteGroups moves the writers the same way).
    var permutation: ?[]u32 = null;
    defer if (permutation) |p| allocator.free(p);
    if (fe.opts.group_order) {
        permutation = try toc_mod.permuteGroups(
            allocator,
            fe.fd.xsize,
            fe.fd.ysize,
            fe.fd.group_dim,
            fe.fd.xsize_groups,
            fe.fd.num_groups,
            fe.fd.num_dc_groups,
            num_passes,
        );
    }
    if (permutation) |p| {
        // physical[permutation[i]] = logical[i]; sizes stay in the
        // physical (listed) order.
        var physical = try allocator.alloc(BitWriter, num_toc_entries);
        defer allocator.free(physical);
        for (0..num_toc_entries) |i| {
            physical[p[i]] = sections.items[i];
            sections.items[i] = BitWriter.init(allocator);
        }
        // (After takeBytes the physical writers end up empty, so the outer
        // `sections` defer — which now owns only fresh empty writers —
        // frees nothing twice.)
        const sizes = try allocator.alloc(u32, num_toc_entries);
        defer allocator.free(sizes);
        for (physical, 0..) |*sw, i| {
            sizes[i] = @intCast(sw.bitsWritten() / 8);
        }
        try toc_mod.writeTocWithPermutation(allocator, p, sizes, writer);
        for (physical) |*sw| {
            const bytes = try sw.takeBytes();
            defer allocator.free(bytes);
            for (bytes) |b| writer.write(8, b);
        }
    } else {
        const sizes = try allocator.alloc(u32, num_toc_entries);
        defer allocator.free(sizes);
        for (sections.items, 0..) |*sw, i| {
            sizes[i] = @intCast(sw.bitsWritten() / 8);
        }
        try toc_mod.writeToc(sizes, writer);
        for (sections.items) |*sw| {
            const bytes = try sw.takeBytes();
            defer allocator.free(bytes);
            for (bytes) |b| writer.write(8, b);
        }
    }
    writer.zeroPadToByte();
}
