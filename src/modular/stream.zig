//! Modular stream orchestration: frame dimensions, stream ids, group
//! headers, tree training, and the per-stream encode/decode entry points.
//! Mirrors libjxl `frame_dimensions.h`, `dec_modular.h/cc`,
//! `enc_modular.cc` and `modular/encoding/{encoding,enc_encoding}.cc`.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const bits = @import("../base/bits.zig");

const ans_common = @import("../entropy/ans_common.zig");
const enc_ans = @import("../entropy/enc_ans.zig");
const dec_ans = @import("../entropy/dec_ans.zig");

const common = @import("common.zig");
const options = @import("options.zig");
const image_mod = @import("image.zig");
const context_predict = @import("context_predict.zig");
const ma = @import("ma.zig");
const ma_encode = @import("ma_encode.zig");
const transforms = @import("transforms.zig");

const Allocator = std.mem.Allocator;
const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const Image = image_mod.Image;
const Channel = image_mod.Channel;
const pixel_type = common.pixel_type;
const pixel_type_w = common.pixel_type_w;
const Token = ma_encode.Token;
const weighted = context_predict.weighted;



pub const EntropyCode = struct {
    code: dec_ans.ANSCode = .{},
    context_map: std.ArrayList(u8) = .empty,
    tree: ma.Tree = .empty,
    allocator: Allocator,

    pub fn init(allocator: Allocator) EntropyCode {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *EntropyCode) void {
        self.context_map.deinit(self.allocator);
        self.tree.deinit(self.allocator);
        self.code.deinit(self.allocator);
    }

    pub fn numContexts(self: *const EntropyCode) usize {
        return (self.tree.items.len + 1) / 2;
    }
};

// ---------------------------------------------------------------------------
// Frame dimensions
// ---------------------------------------------------------------------------

pub const FrameDim = struct {
    xsize: usize = 0,
    ysize: usize = 0,
    /// Pixel size before upsampling is applied (the frame's own size).
    xsize_upsampled: usize = 0,
    ysize_upsampled: usize = 0,
    /// Padded size in 8x8 blocks (VarDCT; chroma-subsampling padded).
    xsize_blocks: usize = 0,
    ysize_blocks: usize = 0,
    group_dim: usize = 0,
    dc_group_dim: usize = 0,
    xsize_groups: usize = 0,
    ysize_groups: usize = 0,
    num_groups: usize = 0,
    xsize_dc_groups: usize = 0,
    ysize_dc_groups: usize = 0,
    num_dc_groups: usize = 0,

    pub const kGroupDim: usize = 256;
    pub const kBlockDim: usize = 8;
    pub const kGroupDimInBlocks: usize = kGroupDim / kBlockDim;

    /// Modular frames have no block padding; upsampling divides the frame
    /// size (libjxl FrameDimensions::Set).
    pub fn init(xsize_px: usize, ysize_px: usize, group_size_shift: u32, upsampling: usize) FrameDim {
        const group_dim = (kGroupDim >> 1) << @intCast(group_size_shift);
        var fd = FrameDim{};
        fd.group_dim = group_dim;
        fd.dc_group_dim = group_dim * kBlockDim;
        fd.xsize_upsampled = xsize_px;
        fd.ysize_upsampled = ysize_px;
        fd.xsize = @intCast(bits.divCeil(xsize_px, upsampling));
        fd.ysize = @intCast(bits.divCeil(ysize_px, upsampling));
        fd.xsize_blocks = @intCast(bits.divCeil(fd.xsize, kBlockDim));
        fd.ysize_blocks = @intCast(bits.divCeil(fd.ysize, kBlockDim));
        fd.xsize_groups = @intCast(bits.divCeil(fd.xsize, group_dim));
        fd.ysize_groups = @intCast(bits.divCeil(fd.ysize, group_dim));
        fd.num_groups = fd.xsize_groups * fd.ysize_groups;
        // Modular mode without transforms has no DC groups' content, but the
        // grammar still derives their count from block dimensions.
        fd.xsize_dc_groups = @intCast(bits.divCeil(fd.xsize, fd.dc_group_dim));
        fd.ysize_dc_groups = @intCast(bits.divCeil(fd.ysize, fd.dc_group_dim));
        fd.num_dc_groups = fd.xsize_dc_groups * fd.ysize_dc_groups;
        return fd;
    }

    /// Port of `FrameDimensions::Set` for VarDCT frames (group_size_shift is
    /// not serialized and fixed at 1 => 256px groups; upsampling is 1).
    /// `max_hshift`/`max_vshift` are the chroma-subsampling maxima; the block
    /// grid is padded to a multiple of 8 << max_shift.
    pub fn initVarDct(xsize_px: usize, ysize_px: usize, max_hshift: usize, max_vshift: usize) FrameDim {
        var fd = FrameDim{};
        fd.group_dim = kGroupDim;
        fd.dc_group_dim = kGroupDim * kBlockDim;
        fd.xsize = xsize_px;
        fd.ysize = ysize_px;
        fd.xsize_blocks = (bits.divCeil(xsize_px, kBlockDim << @intCast(max_hshift)) << @intCast(max_hshift));
        fd.ysize_blocks = (bits.divCeil(ysize_px, kBlockDim << @intCast(max_vshift)) << @intCast(max_vshift));
        fd.xsize_groups = @intCast(bits.divCeil(fd.xsize, kGroupDim));
        fd.ysize_groups = @intCast(bits.divCeil(fd.ysize, kGroupDim));
        fd.num_groups = fd.xsize_groups * fd.ysize_groups;
        fd.xsize_dc_groups = @intCast(bits.divCeil(fd.xsize_blocks, kGroupDim));
        fd.ysize_dc_groups = @intCast(bits.divCeil(fd.ysize_blocks, kGroupDim));
        fd.num_dc_groups = fd.xsize_dc_groups * fd.ysize_dc_groups;
        return fd;
    }

    pub fn groupRect(self: *const FrameDim, group_index: usize) Rect {
        const gx = group_index % self.xsize_groups;
        const gy = group_index / self.xsize_groups;
        return Rect{
            .x0 = gx * self.group_dim,
            .y0 = gy * self.group_dim,
            .xsize = @min(self.group_dim, self.xsize - gx * self.group_dim),
            .ysize = @min(self.group_dim, self.ysize - gy * self.group_dim),
        };
    }

    /// The AC group rect for modular streams, unclamped (libjxl
    /// ProcessFramesForGroup: `Rect(x, y, group_dim, group_dim)` — the
    /// clamping to image/channel bounds happens per channel via the rect
    /// intersection). A clamped group rect would lose coverage of
    /// subsampled/squeezed channels at the right/bottom frame edge.
    pub fn modularGroupRect(self: *const FrameDim, group_index: usize) Rect {
        const gx = group_index % self.xsize_groups;
        const gy = group_index / self.xsize_groups;
        return Rect{
            .x0 = gx * self.group_dim,
            .y0 = gy * self.group_dim,
            .xsize = self.group_dim,
            .ysize = self.group_dim,
        };
    }

    /// Port of `BlockGroupRect`: pixel group rect expressed in blocks.
    pub fn blockGroupRect(self: *const FrameDim, group_index: usize) Rect {
        const gx = group_index % self.xsize_groups;
        const gy = group_index / self.xsize_groups;
        return Rect{
            .x0 = gx * (self.group_dim >> 3),
            .y0 = gy * (self.group_dim >> 3),
            .xsize = @min(self.group_dim >> 3, self.xsize_blocks - gx * (self.group_dim >> 3)),
            .ysize = @min(self.group_dim >> 3, self.ysize_blocks - gy * (self.group_dim >> 3)),
        };
    }

    /// Port of `DCGroupRect`: 256x256-block rect of a DC group, clipped to
    /// the padded block grid.
    pub fn dcGroupRect(self: *const FrameDim, dc_group_index: usize) Rect {
        const gx = dc_group_index % self.xsize_dc_groups;
        const gy = dc_group_index / self.xsize_dc_groups;
        return Rect{
            .x0 = gx * kGroupDim,
            .y0 = gy * kGroupDim,
            .xsize = @min(kGroupDim, self.xsize_blocks - gx * kGroupDim),
            .ysize = @min(kGroupDim, self.ysize_blocks - gy * kGroupDim),
        };
    }

    /// The DC group rect in pixels, as used by the modular DC group streams
    /// (libjxl ProcessDCGroup: `(gx * dc_group_dim, gy * dc_group_dim,
    /// dc_group_dim, dc_group_dim)`, unclamped — the per-channel rect
    /// intersection does the clipping).
    pub fn modularDcGroupRect(self: *const FrameDim, dc_group_index: usize) Rect {
        const gx = dc_group_index % self.xsize_dc_groups;
        const gy = dc_group_index / self.xsize_dc_groups;
        return Rect{
            .x0 = gx * self.dc_group_dim,
            .y0 = gy * self.dc_group_dim,
            .xsize = self.dc_group_dim,
            .ysize = self.dc_group_dim,
        };
    }
};

pub const Rect = struct {
    x0: usize = 0,
    y0: usize = 0,
    xsize: usize = 0,
    ysize: usize = 0,

    pub fn intersect(self: Rect, bounds_xsize: usize, bounds_ysize: usize) Rect {
        const x_start = @min(self.x0, bounds_xsize);
        const y_start = @min(self.y0, bounds_ysize);
        return .{
            .x0 = x_start,
            .y0 = y_start,
            .xsize = @min(self.xsize, bounds_xsize - x_start),
            .ysize = @min(self.ysize, bounds_ysize - y_start),
        };
    }
};

// ---------------------------------------------------------------------------
// Stream ids (libjxl ModularStreamId)
// ---------------------------------------------------------------------------

pub const StreamKind = enum {
    global_data,
    var_dct_dc,
    modular_dc,
    ac_metadata,
    quant_table,
    modular_ac,
};

pub const ModularStreamId = struct {
    kind: StreamKind,
    group_id: usize = 0,
    pass_id: usize = 0,
    quant_table_id: usize = 0,

    pub fn global() ModularStreamId {
        return .{ .kind = .global_data };
    }

    pub fn varDCTDC(group_id: usize) ModularStreamId {
        return .{ .kind = .var_dct_dc, .group_id = group_id };
    }

    pub fn modularDC(group_id: usize) ModularStreamId {
        return .{ .kind = .modular_dc, .group_id = group_id };
    }

    pub fn acMetadata(group_id: usize) ModularStreamId {
        return .{ .kind = .ac_metadata, .group_id = group_id };
    }

    pub fn quantTable(quant_table_id: usize) ModularStreamId {
        return .{ .kind = .quant_table, .quant_table_id = quant_table_id };
    }

    pub fn modularAC(group_id: usize, pass_id: usize) ModularStreamId {
        return .{ .kind = .modular_ac, .group_id = group_id, .pass_id = pass_id };
    }

    pub fn id(self: ModularStreamId, fd: *const FrameDim) usize {
        return switch (self.kind) {
            .global_data => 0,
            .var_dct_dc => 1 + self.group_id,
            .modular_dc => 1 + fd.num_dc_groups + self.group_id,
            .ac_metadata => 1 + 2 * fd.num_dc_groups + self.group_id,
            .quant_table => 1 + 3 * fd.num_dc_groups + self.quant_table_id,
            .modular_ac => 1 + 3 * fd.num_dc_groups + kNumQuantTables +
                fd.num_groups * self.pass_id + self.group_id,
        };
    }

    pub fn num(fd: *const FrameDim, passes: usize) usize {
        return (ModularStreamId{ .kind = .modular_ac, .group_id = 0, .pass_id = passes }).id(fd);
    }

    /// libjxl `DequantMatrices::kNumQuantTables` (QuantTable enum size).
    pub const kNumQuantTables: usize = 17;
};

// ---------------------------------------------------------------------------
// GroupHeader
// ---------------------------------------------------------------------------

pub const GroupHeader = struct {
    use_global_tree: bool = false,
    wp_header: weighted.Header = weighted.Header.defaults,
    transforms: std.ArrayList(transforms.Transform) = .empty,
    allocator: Allocator,

    pub fn init(allocator: Allocator) GroupHeader {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *GroupHeader) void {
        for (self.transforms.items) |*t| t.deinit();
        self.transforms.deinit(self.allocator);
    }

    pub fn read(self: *GroupHeader, br: *BitReader) !void {
        self.* = .{ .allocator = self.allocator };
        self.use_global_tree = (br.readBits(1)) != 0;
        self.wp_header = try weighted.Header.read(br);
        const num_transforms = fields.U32Coder.read(transforms_count_enc, br);
        try self.transforms.ensureTotalCapacity(self.allocator, num_transforms);
        for (0..num_transforms) |_| {
            var t = transforms.Transform.init(self.allocator);
            errdefer t.deinit();
            try t.read(br);
            try self.transforms.append(self.allocator, t);
        }
    }

    pub fn write(self: *const GroupHeader, writer: *BitWriter) !void {
        writer.write(1, @intFromBool(self.use_global_tree));
        try self.wp_header.write(self.allocator, writer);
        try fields.U32Coder.write(transforms_count_enc, @intCast(self.transforms.items.len), writer);
        for (self.transforms.items) |*t| {
            try t.write(writer);
        }
    }

    const transforms_count_enc = fields.U32Enc.init(fields.val(0), fields.val(1), fields.bitsOffset(4, 2), fields.bitsOffset(8, 18));
};

// ---------------------------------------------------------------------------
// ANS-backed symbol reader matching the MA tree reader interface
// ---------------------------------------------------------------------------

pub const AnsMaReader = struct {
    ans: dec_ans.ANSSymbolReader,
    context_map: []const u8,
    property_degenerate: bool,

    pub fn create(
        allocator: Allocator,
        code: *const dec_ans.ANSCode,
        context_map: []const u8,
        distance_multiplier: usize,
        br: *BitReader,
    ) !AnsMaReader {
        const ans = try dec_ans.ANSSymbolReader.create(allocator, code, br, @intCast(distance_multiplier));
        var property_degenerate = false;
        if (context_map.len > 0 and code.degenerate_symbols.len > context_map[0]) {
            property_degenerate = code.degenerate_symbols[context_map[0]] > 0;
        }
        return .{ .ans = ans, .context_map = context_map, .property_degenerate = property_degenerate };
    }

    pub fn readHybridUint(self: *AnsMaReader, ctx: usize, br: *BitReader) u32 {
        return self.ans.readHybridUint(ctx, br, self.context_map);
    }

    pub fn checkFinalState(self: *const AnsMaReader) bool {
        return self.ans.checkANSFinalState();
    }

    pub fn isPropertyContextDegenerate(self: *const AnsMaReader) bool {
        return self.property_degenerate;
    }
};

// ---------------------------------------------------------------------------
// Channel decode/encode with an MA tree
// ---------------------------------------------------------------------------

/// Port of EncodeModularChannelMAANS (general path).
pub fn encodeChannelMAANS(
    allocator: Allocator,
    image: *const Image,
    channel_index: usize,
    group_id: usize,
    tree: []const ma.PropertyDecisionNode,
    wp_header: *const weighted.Header,
    tokens: *std.ArrayList(Token),
) !void {
    const ch = &image.channels.items[channel_index];
    const static_props = [context_predict.kNumStaticProperties]pixel_type{
        @intCast(channel_index), @intCast(group_id),
    };
    var num_props: usize = 0;
    var use_wp = false;
    var wp_only = false;
    var gradient_only = false;
    var flat_tree = context_predict.FlatTree.empty;
    defer flat_tree.deinit(allocator);
    try context_predict.filterTree(allocator, tree, &static_props, &num_props, &use_wp, &wp_only, &gradient_only, &flat_tree);
    const tree_lookup = context_predict.MATreeLookup.init(flat_tree.items);
    const properties = try allocator.alloc(pixel_type, num_props);
    defer allocator.free(properties);
    var references = try Channel.create(allocator, num_props - context_predict.kNumNonrefProperties, ch.w, 0, 0);
    defer references.deinit();
    var wp_state = try weighted.State.create(allocator, wp_header, ch.w, ch.h);
    defer wp_state.deinit(allocator);

    var y: usize = 0;
    while (y < ch.h) : (y += 1) {
        const row = ch.row(y);
        context_predict.precomputeReferences(ch, y, image, channel_index, &references);
        context_predict.initPropsRow(properties, &static_props, @intCast(y));
        for (0..ch.w) |x| {
            const res = if (use_wp)
                context_predict.predictTreeWP(properties, ch.w, &ch.plane, x, y, &tree_lookup, &references, &wp_state)
            else
                context_predict.predictTreeNoWP(properties, ch.w, &ch.plane, x, y, &tree_lookup, &references);
            const residual: pixel_type_w = row[x] - res.guess;
            const val = common.packSigned(@truncate(@divTrunc(residual, @as(pixel_type_w, res.multiplier))));
            try tokens.append(allocator, .{ .context = @intCast(res.context), .value = val });
            if (use_wp) {
                wp_state.updateErrors(row[x], x, y, ch.w);
            }
        }
    }
}

/// Port of the DecodeModularChannelMAANS general path.
pub fn decodeChannelMAANS(
    allocator: Allocator,
    image: *const Image,
    channel_index: usize,
    group_id: usize,
    reader: *AnsMaReader,
    tree: []const ma.PropertyDecisionNode,
    wp_header: *const weighted.Header,
    br: *BitReader,
) !void {
    const ch = &image.channels.items[channel_index];
    const static_props = [context_predict.kNumStaticProperties]pixel_type{
        @intCast(channel_index), @intCast(group_id),
    };
    var num_props: usize = 0;
    var use_wp = false;
    var wp_only = false;
    var gradient_only = false;
    var flat_tree = context_predict.FlatTree.empty;
    defer flat_tree.deinit(allocator);
    try context_predict.filterTree(allocator, tree, &static_props, &num_props, &use_wp, &wp_only, &gradient_only, &flat_tree);
    const tree_lookup = context_predict.MATreeLookup.init(flat_tree.items);
    const properties = try allocator.alloc(pixel_type, num_props);
    defer allocator.free(properties);
    var references = try Channel.create(allocator, num_props - context_predict.kNumNonrefProperties, ch.w, 0, 0);
    defer references.deinit();
    var wp_state = try weighted.State.create(allocator, wp_header, ch.w, ch.h);
    defer wp_state.deinit(allocator);

    var token_count: usize = 0;
    var y: usize = 0;
    while (y < ch.h) : (y += 1) {
        const row = ch.row(y);
        context_predict.precomputeReferences(ch, y, image, channel_index, &references);
        context_predict.initPropsRow(properties, &static_props, @intCast(y));
        for (0..ch.w) |x| {
            const res = if (use_wp)
                context_predict.predictTreeWP(properties, ch.w, &ch.plane, x, y, &tree_lookup, &references, &wp_state)
            else
                context_predict.predictTreeNoWP(properties, ch.w, &ch.plane, x, y, &tree_lookup, &references);
            const v = reader.readHybridUint(@intCast(res.context), br);
            const val: pixel_type_w = common.unpackSigned(v);
            const made: pixel_type = @truncate(val *% @as(pixel_type_w, res.multiplier) +% res.guess);
            row[x] = made;
            token_count += 1;
            if (use_wp) {
                wp_state.updateErrors(row[x], x, y, ch.w);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Stream decode (libjxl ModularDecode)
// ---------------------------------------------------------------------------

/// Number of leading non-meta channels that belong to this stream.
/// Channels bigger than max_chan_size stop the scan (they live in groups).
pub fn numStreamChannels(image: *const Image, max_chan_size: usize) usize {
    var count: usize = 0;
    for (image.channels.items, 0..) |*ch, i| {
        if (i >= image.nb_meta_channels and (ch.w > max_chan_size or ch.h > max_chan_size)) break;
        count += 1;
    }
    return count;
}

/// The LZ77 distance multiplier of a stream: the maximum width among the
/// channels the stream actually decodes (the same value `decodeStream`
/// feeds `ANSSymbolReader.create`, which activates the special-distance
/// table when it is non-zero).
pub fn maxStreamChannelWidth(image: *const Image, max_chan_size: usize) usize {
    const nb_channels = numStreamChannels(image, max_chan_size);
    var width: usize = 0;
    for (image.channels.items[0..nb_channels]) |*ch| {
        if (ch.w > width) width = ch.w;
    }
    return width;
}

pub const DecodeStreamArgs = struct {
    allocator: Allocator,
    br: *BitReader,
    image: *Image,
    group_id: usize,
    max_chan_size: usize,
    global: ?*const EntropyCode,
    /// distance multiplier for LZ77 distance contexts (max channel width).
    distance_multiplier: usize = 0,
    /// Tree size limit (bytes-derived); default computed from channel sizes.
    tree_size_limit: ?usize = null,
    /// Undo the stream's transforms right after decoding (group streams).
    /// Otherwise they are appended to `transforms_out` for the caller.
    undo_transforms: bool = false,
    transforms_out: ?*std.ArrayList(transforms.Transform) = null,
    wp_header_out: ?*weighted.Header = null,
    bitdepth: u32 = 8,
};

/// Port of ModularDecode: header, transforms, tree, histograms, channels.
pub fn decodeStream(args: DecodeStreamArgs) !void {
    const allocator = args.allocator;
    const image = args.image;
    if (image.channels.items.len == 0) return;

    var header = GroupHeader.init(allocator);
    defer header.deinit();
    try header.read(args.br);

    // Export the header state (WP header, transforms) before anything can
    // return early: libjxl stores the transforms in `image.transform` inside
    // ModularDecode regardless of how many channels get decoded, so the
    // frame-level inverse still runs when this stream decodes no channels
    // (e.g. the global stream of a multi-group image whose channels all live
    // in the group streams).
    if (args.wp_header_out) |wp| wp.* = header.wp_header;

    // Apply the channel-layout part first: MetaSqueeze materializes the
    // default squeeze scheme into the transform itself (libjxl mutates
    // `transform.squeezes`), and the exported copy below must carry those
    // parameters so the frame-level inverse can replay them.
    for (header.transforms.items) |*t| {
        try transforms.metaApply(image, t);
    }

    if (!args.undo_transforms) {
        if (args.transforms_out) |out| {
            for (header.transforms.items) |*t| {
                var copy = transforms.Transform.init(args.allocator);
                try copy.squeezes.appendSlice(args.allocator, t.squeezes.items);
                copy.id = t.id;
                copy.begin_c = t.begin_c;
                copy.rct_type = t.rct_type;
                copy.num_c = t.num_c;
                copy.nb_colors = t.nb_colors;
                copy.nb_deltas = t.nb_deltas;
                copy.predictor = t.predictor;
                copy.ordered_palette = t.ordered_palette;
                copy.lossy_palette = t.lossy_palette;
                try out.append(args.allocator, copy);
            }
        }
    }

    const nb_channels = numStreamChannels(image, args.max_chan_size);

    var num_chans: usize = 0;
    var distance_multiplier = args.distance_multiplier;
    for (image.channels.items[0..nb_channels]) |*ch| {
        if (ch.w == 0 or ch.h == 0) continue;
        if (ch.w > distance_multiplier) distance_multiplier = ch.w;
        num_chans += 1;
    }
    if (num_chans == 0) return;

    var local: EntropyCode = EntropyCode.init(allocator);
    defer local.deinit();

    const stream_code: *const EntropyCode = blk: {
        if (!header.use_global_tree) {
            var max_tree_size: usize = 1024;
            for (image.channels.items[0..nb_channels]) |*ch| {
                max_tree_size += ch.w * ch.h;
            }
            max_tree_size = @min(@as(usize, 1) << 20, max_tree_size);
            if (args.tree_size_limit) |limit| max_tree_size = @min(max_tree_size, limit);
            // Inline tree: 6 tree contexts, then the stream histograms.
            var tree_code = dec_ans.ANSCode{};
            defer tree_code.deinit(allocator);
            var tree_ctx_map: std.ArrayList(u8) = .empty;
            defer tree_ctx_map.deinit(allocator);
            try dec_ans.decodeHistograms(allocator, args.br, ma.kNumTreeContexts, &tree_code, &tree_ctx_map, false);
            if (tree_code.degenerate_symbols.len > tree_ctx_map.items[0] and
                tree_code.degenerate_symbols[tree_ctx_map.items[0]] > 0)
            {
                return error.InfiniteTree;
            }
            var tree_reader = try AnsMaReader.create(allocator, &tree_code, tree_ctx_map.items, 0, args.br);
            defer tree_reader.ans.deinit();
            try ma.decodeTree(args.br, &tree_reader, &local.tree, allocator, max_tree_size);
            try dec_ans.decodeHistograms(allocator, args.br, local.numContexts(), &local.code, &local.context_map, false);
            break :blk &local;
        }
        if (args.global == null or args.global.?.tree.items.len == 0) {
            return error.NoGlobalTree;
        }
        break :blk args.global.?;
    };

    var reader = try AnsMaReader.create(allocator, &stream_code.code, stream_code.context_map.items, distance_multiplier, args.br);
    defer reader.ans.deinit();

    for (0..nb_channels) |c| {
        const ch = &image.channels.items[c];
        if (c >= image.nb_meta_channels and (ch.w > args.max_chan_size or ch.h > args.max_chan_size)) break;
        if (ch.w == 0 or ch.h == 0) continue;
        try decodeChannelMAANS(allocator, image, c, args.group_id, &reader, stream_code.tree.items, &header.wp_header, args.br);
        if (!args.br.allReadsWithinBounds()) return error.OutOfBounds;
    }

    if (!reader.checkFinalState()) {
        return error.ANSFinalState;
    }

    if (args.undo_transforms) {
        var i: usize = header.transforms.items.len;
        while (i > 0) {
            i -= 1;
            try transforms.inverse(image, &header.transforms.items[i], &header.wp_header, args.bitdepth);
        }
    }
}

// ---------------------------------------------------------------------------
// Stream encode: tokenization
// ---------------------------------------------------------------------------

/// Tokenizes every decodable channel of the stream image with `tree`.
pub fn encodeStreamTokens(
    allocator: Allocator,
    image: *const Image,
    group_id: usize,
    max_chan_size: usize,
    tree: []const ma.PropertyDecisionNode,
    wp_header: *const weighted.Header,
    tokens: *std.ArrayList(Token),
) !void {
    if (image.channels.items.len == 0) return;
    const nb_channels = numStreamChannels(image, max_chan_size);
    for (0..nb_channels) |c| {
        const ch = &image.channels.items[c];
        if (c >= image.nb_meta_channels and (ch.w > max_chan_size or ch.h > max_chan_size)) break;
        if (ch.w == 0 or ch.h == 0) continue;
        try encodeChannelMAANS(allocator, image, c, group_id, tree, wp_header, tokens);
    }
}

// ---------------------------------------------------------------------------
// Tree training (libjxl LearnTree / GatherTreeData)
// ---------------------------------------------------------------------------

pub const TrainStream = struct {
    image: *const Image,
    group_id: usize,
    /// The stream's weighted-predictor header (the WP-mode search result
    /// at slower efforts): the tree samples must reflect the same
    /// Weighted-predictor constants the stream is later tokenized with.
    wp_header: *const weighted.Header = &default_wp_header,
};

/// The default (mode-0) WP header used when a stream has no searched one.
const default_wp_header = weighted.Header.defaults;

/// Samples every pixel (nb_repeats = 1) with the Variable predictor set.
pub fn trainTree(
    allocator: Allocator,
    streams: []const TrainStream,
    opts: *const options.ModularOptions,
) !ma.Tree {
    var ts = ma_encode.TreeSamples.init(allocator);
    defer ts.deinit();
    try ts.setPredictor(.variable, opts.wp_tree_mode);
    try ts.setProperties(opts.splitting_heuristics_properties, opts.wp_tree_mode);

    var max_c: u32 = 0;
    for (streams) |s| {
        max_c = @max(max_c, @as(u32, @intCast(s.image.channels.items.len)));
    }

    var group_pixel_count: std.ArrayList(u32) = .empty;
    defer group_pixel_count.deinit(allocator);
    var channel_pixel_count: std.ArrayList(u32) = .empty;
    defer channel_pixel_count.deinit(allocator);
    var pixel_samples: std.ArrayList(pixel_type) = .empty;
    defer pixel_samples.deinit(allocator);
    var diff_samples: std.ArrayList(pixel_type) = .empty;
    defer diff_samples.deinit(allocator);

    for (streams) |s| {
        try ma_encode.collectPixelSamples(allocator, s.image, opts, @intCast(s.group_id), &group_pixel_count, &channel_pixel_count, &pixel_samples, &diff_samples);
    }

    const range = [2][2]u32{ .{ 0, max_c }, .{ 0, @intCast(streams.len) } };
    try ts.preQuantizeProperties(
        range,
        &.{},
        group_pixel_count.items,
        channel_pixel_count.items,
        pixel_samples.items,
        diff_samples.items,
        opts.max_property_values,
    );

    var total_pixels: usize = 0;

    for (streams) |s| {
        const image = s.image;
        if (image.channels.items.len == 0) continue;
        const nb_channels = numStreamChannels(image, opts.max_chan_size);
        for (0..nb_channels) |c| {
            const ch = &image.channels.items[c];
            if (c >= image.nb_meta_channels and (ch.w > opts.max_chan_size or ch.h > opts.max_chan_size)) break;
            if (ch.w == 0 or ch.h == 0) continue;
            try gatherTreeData(allocator, image, c, s.group_id, opts, &ts, s.wp_header, &total_pixels);
        }
    }

    var tree = ma.Tree.empty;
    errdefer tree.deinit(allocator);
    if (!ts.hasSamples()) {
        try tree.append(allocator, ma.PropertyDecisionNode.leaf(ts.predictorFromIndex(0), 0, 1));
        return tree;
    }
    // All pixels are sampled: pixel_fraction == 1.
    const required_cost: f32 = 1.0 * 0.9 + 0.1;
    ts.allSamplesDone();
    try ma_encode.computeBestTree(
        allocator,
        &ts,
        opts.splitting_heuristics_node_threshold * required_cost,
        &.{},
        range,
        opts.fast_decode_multiplier,
        &tree,
    );
    return tree;
}

fn gatherTreeData(
    allocator: Allocator,
    image: *const Image,
    channel_index: usize,
    group_id: usize,
    opts: *const options.ModularOptions,
    ts: *ma_encode.TreeSamples,
    wp_header: *const weighted.Header,
    total_pixels: *usize,
) !void {
    const ch = &image.channels.items[channel_index];
    const static_props = [context_predict.kNumStaticProperties]pixel_type{
        @intCast(channel_index), @intCast(group_id),
    };
    const properties = try allocator.alloc(pixel_type, context_predict.kNumNonrefProperties + context_predict.kExtraPropsPerChannel * @as(usize, @intCast(opts.max_properties)));
    defer allocator.free(properties);

    var references = try Channel.create(allocator, @as(usize, @intCast(opts.max_properties)), ch.w, 0, 0);
    defer references.deinit();
    var wp_learn = try weighted.State.create(allocator, wp_header, ch.w, ch.h);
    defer wp_learn.deinit(allocator);
    var predictions = [_]pixel_type_w{0} ** options.kNumModularEncoderPredictors;

    try ts.prepareForSamples(ch.w * ch.h + 64);

    for (0..ch.h) |y| {
        context_predict.precomputeReferences(ch, y, image, channel_index, &references);
        context_predict.initPropsRow(properties, &static_props, @intCast(y));
        for (0..ch.w) |x| {
            context_predict.predictLearnAll(properties, ch.w, &ch.plane, x, y, &references, &wp_learn, &predictions);
            total_pixels.* += 1;
            try ts.addSample(ch.row(y)[x], properties, &predictions);
            wp_learn.updateErrors(ch.row(y)[x], x, y, ch.w);
        }
    }
}

/// Tokenizes the tree itself into 6 tree contexts; returns the decoder-side
/// tree through `decoded_tree` (identical content, validated ordering).
pub fn tokenizeTree(
    allocator: Allocator,
    tree: []const ma.PropertyDecisionNode,
    tree_tokens: *std.ArrayList(Token),
    decoded_tree: *ma.Tree,
) !void {
    try ma_encode.tokenizeTree(allocator, tree, tree_tokens, decoded_tree);
}

/// Builds and writes histograms for `tokens` (per context) and returns the
/// resulting code for subsequent writeTokens calls. Takes entropy-library
/// tokens; use codec-level conversion helpers for MA tokens.
pub fn buildHistograms(
    allocator: Allocator,
    params: *const enc_ans.HistogramParams,
    num_contexts: usize,
    tokens: []std.ArrayList(enc_ans.Token),
    codes: *enc_ans.EntropyEncodingData,
    writer: ?*BitWriter,
) !usize {
    return enc_ans.buildAndEncodeHistograms(allocator, params, num_contexts, tokens, codes, writer);
}

pub fn writeTokens(
    allocator: Allocator,
    tokens: []const enc_ans.Token,
    codes: *const enc_ans.EntropyEncodingData,
    context_offset: usize,
    writer: *BitWriter,
) !usize {
    return enc_ans.writeTokens(allocator, tokens, codes, context_offset, writer);
}

test "frame dim layout" {
    const fd = FrameDim.init(257, 100, 1, 1);
    try std.testing.expectEqual(@as(usize, 256), fd.group_dim);
    try std.testing.expectEqual(@as(usize, 2), fd.xsize_groups);
    try std.testing.expectEqual(@as(usize, 1), fd.ysize_groups);
    try std.testing.expectEqual(@as(usize, 2), fd.num_groups);
    try std.testing.expectEqual(@as(usize, 1), fd.num_dc_groups);
    const r0 = fd.groupRect(0);
    try std.testing.expectEqual(@as(usize, 256), r0.xsize);
    try std.testing.expectEqual(@as(usize, 100), r0.ysize);
    const r1 = fd.groupRect(1);
    try std.testing.expectEqual(@as(usize, 1), r1.xsize);
    try std.testing.expectEqual(@as(usize, 100), r1.ysize);
}

test "stream id mapping" {
    var fd = FrameDim.init(700, 700, 1, 1);
    try std.testing.expectEqual(@as(usize, 0), ModularStreamId.global().id(&fd));
    const ng = ModularStreamId.modularAC(0, 0).id(&fd);
    try std.testing.expectEqual(@as(usize, 1 + 3 * fd.num_dc_groups + ModularStreamId.kNumQuantTables), ng);
}
