//! Lossy VarDCT still frames with reference patches — the encoder half
//! (`EncodeFrame`'s VarDCT path of libjxl, restricted to the DCT-8,
//! no-CfL, no-EPF subset) and the pixel decoder half (the VarDCT branch
//! of `dec_frame.cc`/`dec_group.cc` + the render-pipeline order
//! `dec_cache.cc`: gaborish -> patches -> XYB-to-linear).
//!
//! Encoder flow (libjxl `LossyFrameHeuristics` + `ComputeCoefficients`):
//!   linear sRGB -> XYB opsin (`enc_xyb.cc`)
//!   -> patch search on the raw XYB planes (`FindBestPatchDictionary`,
//!      is_xyb=true) + reference-frame quantization + float subtraction
//!   -> gaborish compensation (`enc_gaborish.cc` 5x5 kernel)
//!   -> per-block scaled DCT-8, DC separately (`AddVarDCTDC` quantizer)
//!   -> AC quantization against the Libshroom DCT-8 weights, serialized
//!      as a RAW quant table (den 2^-16), constant quant field
//!      (`SetQuant`: q = 0.79/distance, `InitialQuantDC`).
//! The simple constant-quantizer branch (libjxl speed > kHare or
//! `disable_perceptual_optimizations`) is used instead of
//! Butteraugli-driven adaptive quantization; files stay valid, spec-legal
//! VarDCT and djxl-decodable.
//!
//! Decoder: DC global (patches dict first) -> DC groups (VarDCTDC +
//! ACMetadata + adaptive DC smoothing) -> AC global (RAW or default
//! library dequant tables, orders, histograms) -> AC groups ->
//! dequant(+bias) -> DC -> IDCT-8 -> gaborish -> patches -> XYB-to-linear.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const fields_ext = @import("../header/fields_ext.zig");

const size_mod = @import("../header/size.zig");
const color_mod = @import("../header/color.zig");
const metadata_mod = @import("../header/metadata.zig");
const frame_mod = @import("../header/frame.zig");
const toc_mod = @import("../header/toc.zig");

const stream = @import("../modular/stream.zig");
const image_mod = @import("../modular/image.zig");
const options = @import("../modular/options.zig");
const ma = @import("../modular/ma.zig");
const ma_encode = @import("../modular/ma_encode.zig");
const context_predict = @import("../modular/context_predict.zig");
const enc_ans = @import("../entropy/enc_ans.zig");
const dec_ans = @import("../entropy/dec_ans.zig");

const patches_mod = @import("../patches/dictionary.zig");
const enc_patches = @import("../patches/enc_patches.zig");

fn ceilLog2Nonzero(v: usize) usize {
    std.debug.assert(v != 0);
    const l = std.math.log2_int(usize, v);
    return l + @as(usize, @intFromBool(@popCount(v) != 1));
}

const block_context = @import("block_context.zig");
const coeff_order_mod = @import("coeff_order.zig");
const ac_coefficient = @import("ac_coefficient.zig");
const quant_mod = @import("quant.zig");
const transform_lossy = @import("transform_lossy.zig");
const opsin_mod = @import("opsin.zig");

const Allocator = std.mem.Allocator;
const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const Image = image_mod.Image;
const Channel = image_mod.Channel;
const Token = ma_encode.Token;
const ANSToken = enc_ans.Token;
const FrameDim = stream.FrameDim;
const ModularStreamId = stream.ModularStreamId;
const TrainStream = stream.TrainStream;
const FrameHeader = frame_mod.FrameHeader;

pub const Error = error{
    UnsupportedLossyInput,
    UnsupportedVarDct,
    InvalidDistance,
    OutOfMemory,
} || anyerror;

/// Encoder options of the lossy path.
pub const LossyEncodeOptions = struct {
    /// cjxl `-d`: the Butteraugli distance (0.1 .. ~25; must be > 0).
    distance: f32 = 1.0,
    /// cjxl `--patches` (default: search at effort >= 7).
    patches: enc_patches.PatchOverride = .default,
    effort: u32 = 7,
    /// Custom (learned) coefficient orders (on by default like cjxl at
    /// the default effort).
    coeff_orders: bool = true,
};

// ---------------------------------------------------------------------------
// Quantizer (quantizer.cc + enc_adaptive_quantization.cc)
// ---------------------------------------------------------------------------

const kGlobalScaleDenom: f32 = 65536.0;
const kGlobalScaleNumerator: f32 = 4096.0;
const kQuantMax: i32 = 256;
const kDcQuantPow: f32 = 0.83;
const kDcQuant: f32 = 1.095924047623553;

/// libjxl `InitialQuantDC`.
pub fn initialQuantDC(distance: f32) f32 {
    const kDcMul: f32 = 0.3;
    const target_dc = @max(
        0.5 * distance,
        @min(
            distance,
            kDcMul * std.math.pow(
                f32,
                (1.0 / kDcMul) * distance,
                kDcQuantPow,
            ),
        ),
    );
    return @min(kDcQuant / target_dc, 50.0);
}

fn clampQuantVal(v: f32) i32 {
    return @intFromFloat(
        std.math.clamp(
            v,
            1.0,
            @as(f32, @floatFromInt(kQuantMax)),
        ),
    );
}

/// libjxl `Quantizer`: the serialized pair (global_scale, quant_dc) plus
/// the derived floats both sides compute from them.
pub const Quantizer = struct {
    global_scale: u32 = 1,
    quant_dc: u32 = 1,
    inv_global_scale: f32 = 1,
    global_scale_float: f32 = 1.0 / kGlobalScaleDenom,

    /// `ComputeGlobalScaleAndQuant` + `SetQuant` for a constant quant
    /// field of `quant_ac`.
    pub fn init(quant_dc_in: f32, quant_ac: f32) Quantizer {
        const kQuantFieldTarget: f32 = 5.0;
        var scale = kGlobalScaleDenom * quant_ac / kQuantFieldTarget;
        if (scale < 1) scale = 1;
        if (scale > (1 << 15)) scale = 1 << 15;
        var new_global_scale: i64 = @intFromFloat(scale);
        const scaled_quant_dc: i64 = @intFromFloat(
            quant_dc_in * kGlobalScaleNumerator * 1.6,
        );
        if (new_global_scale > scaled_quant_dc) {
            new_global_scale = scaled_quant_dc;
            if (new_global_scale <= 0) new_global_scale = 1;
        }
        var q = Quantizer{
            .global_scale = @intCast(new_global_scale),
        };
        q.recompute();
        var fval = quant_dc_in * q.inv_global_scale + 0.5;
        if (fval > (1 << 16)) fval = 1 << 16;
        q.quant_dc = @intFromFloat(fval);
        q.recompute();
        return q;
    }

    fn recompute(self: *Quantizer) void {
        self.global_scale_float = @as(f32, @floatFromInt(self.global_scale)) *
            (1.0 / kGlobalScaleDenom);
        self.inv_global_scale = @floatCast(
            1.0 * kGlobalScaleDenom /
                @as(f64, @floatFromInt(self.global_scale)),
        );
    }

    pub fn fromParams(global_scale: u32, quant_dc: u32) Quantizer {
        var q = Quantizer{
            .global_scale = global_scale,
            .quant_dc = quant_dc,
        };
        q.recompute();
        return q;
    }

    /// GetInvDcStep(c) = InvDCQuant(c) * (global_scale_float * quant_dc)
    /// with the all-default DequantDC tables.
    pub fn invDcStep(self: *const Quantizer, c: usize) f32 {
        return quant_mod.kInvDCQuant[c] *
            (self.global_scale_float * @as(f32, @floatFromInt(self.quant_dc)));
    }

    /// MulDC(c) = inv_quant_dc * DCQuant(c) (the decoder-side DC step).
    pub fn dcStep(self: *const Quantizer, c: usize) f32 {
        const inv_quant_dc = self.inv_global_scale /
            @as(f32, @floatFromInt(self.quant_dc));
        return inv_quant_dc * (1.0 / quant_mod.kInvDCQuant[c]);
    }

    /// The constant raw-quant-field value for a target AC quant
    /// (`SetQuant` tail).
    pub fn quantFieldValue(self: *const Quantizer, quant_ac: f32) i32 {
        return clampQuantVal(quant_ac * self.inv_global_scale + 0.5);
    }
};

// ---------------------------------------------------------------------------
// RAW dequant table from the Libshroom DCT-8 weights
// ---------------------------------------------------------------------------

/// den = 2^-16 (exactly representable in F16 as a power of two).
pub const raw_table_den: f32 = 1.0 / 65536.0;

/// Builds the RAW quant table (17-entry layout: only table 0 = DCT is
/// RAW; values `qtable[c*64 + 8*y + x]`, image-store layout) from the
/// Libshroom weights: int = round(1 / (weight * den)). Returns the ints
/// and the effective inverse weights (= den * int) the encoder must
/// quantize against.
pub fn buildRawQuantTable() struct { ints: [3 * 64]i32, inv_weights: [3 * 64]f32 } {
    const weights = transform_lossy.libshroomDct8QuantWeights();
    var ints: [3 * 64]i32 = undefined;
    var invw: [3 * 64]f32 = undefined;
    for (0..3 * 64) |i| {
        // dequant table value = 1 / weight, serialized as den * int.
        const v: f32 = 1.0 / (weights[i] * raw_table_den);
        const clamped = std.math.clamp(v, 1.0, 65535.0);
        const n: i32 = @intFromFloat(@floor(clamped + 0.5));
        ints[i] = n;
        invw[i] = 1.0 / (raw_table_den * @as(f32, @floatFromInt(n)));
    }
    return .{ .ints = ints, .inv_weights = invw };
}

// ---------------------------------------------------------------------------
// XYB planes
// ---------------------------------------------------------------------------

/// Converts integer RGB samples to the three raw XYB opsin planes
/// (linearized through the exact sRGB EOTF, then `LinearRGBToXYB`).
/// The planes are `xsize_blocks*8 x ysize_blocks*8` (zero-padded) so the
/// DCT sees whole blocks. `src` is sampled through the comptime `sample`
/// function (the PixelBuffer accessor of the codec layer).
pub fn buildXybPlanes(
    allocator: Allocator,
    xsize: usize,
    ysize: usize,
    bitdepth: u8,
    src: anytype,
    comptime sample: fn (@TypeOf(src), usize, usize, usize) u16,
) ![3][]f32 {
    const maxval: f32 = @floatFromInt((@as(u64, 1) << @intCast(bitdepth)) - 1);
    var planes: [3][]f32 = undefined;
    errdefer for (&planes) |p| {
        if (p.len != 0) allocator.free(p);
    };
    for (&planes) |*p| {
        p.* = try allocator.alloc(f32, xsize * ysize);
        @memset(p.*, 0);
    }
    for (0..ysize) |y| {
        for (0..xsize) |x| {
            const r = opsin_mod.srgbToLinear(@as(f32, @floatFromInt(sample(src, x, y, 0))) / maxval);
            const g = opsin_mod.srgbToLinear(@as(f32, @floatFromInt(sample(src, x, y, 1))) / maxval);
            const b = opsin_mod.srgbToLinear(@as(f32, @floatFromInt(sample(src, x, y, 2))) / maxval);
            const xyb = opsin_mod.linearRgbToXyb(r, g, b);
            planes[0][y * xsize + x] = xyb[0];
            planes[1][y * xsize + x] = xyb[1];
            planes[2][y * xsize + x] = xyb[2];
        }
    }
    return planes;
}

/// Float-domain kAdd patch subtraction on the XYB planes
/// (`PatchDictionaryEncoder::SubtractFrom` — only kAdd/kNone/kReplace
/// occur, and the VarDCT search emits kAdd only).
pub fn subtractPatchesXyb(
    allocator: Allocator,
    pd: *const patches_mod.PatchDictionary,
    planes: [3][]f32,
    bw: usize,
) !void {
    // `bw` is the plane stride (xsize for the raw opsin planes).
    var row_patches: std.ArrayList(usize) = .empty;
    defer row_patches.deinit(allocator);
    const ysize = pd.num_patches.items.len;
    var y: usize = 0;
    while (y < ysize) : (y += 1) {
        try pd.getPatchesForRow(allocator, y, &row_patches);
        for (row_patches.items) |pos_idx| {
            const blending_idx = pos_idx * pd.blendings_stride;
            const pos = &pd.positions.items[pos_idx];
            const ref_pos = &pd.ref_positions.items[pos.ref_pos_idx];
            const mode = pd.blendings.items[blending_idx].mode;
            if (mode != .add and mode != .none and mode != .replace) {
                return error.UnsupportedPatchSubtraction;
            }
            const by = pos.y;
            const bx = pos.x;
            std.debug.assert(y >= by and y < by + ref_pos.ysize);
            const iy = y - by;
            const ref_frame = &(pd.refs.?[ref_pos.ref] orelse
                return error.InvalidPatchReferenceFrame);
            var ix: usize = 0;
            while (ix < ref_pos.xsize) : (ix += 1) {
                if (bx + ix >= bw) break;
                for (0..3) |c| {
                    const src = ref_frame.planes[c][
                        (ref_pos.y0 + iy) *
                            ref_frame.xsize + ref_pos.x0 + ix
                    ];
                    const dst = bx + ix;
                    switch (mode) {
                        .add => planes[c][y * bw + dst] -= src,
                        .replace => planes[c][y * bw + dst] = 0,
                        else => {},
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Encoder: frame data
// ---------------------------------------------------------------------------

/// Everything the DCT/quantization stage produces.
pub const LossyFrameData = struct {
    allocator: Allocator,
    fd: FrameDim,
    /// Quantized DC per channel (full grid, channel dims = block dims).
    dc: [3][]i32 = .{ &.{}, &.{}, &.{} },
    /// Quantized AC blocks per channel (block-major, JXL layout).
    ac: [3][]i32 = .{ &.{}, &.{}, &.{} },
    /// DC threshold buckets (all zero: default BlockCtxMap).
    quant_dc: []u8 = &.{},
    /// The raw quant field value (constant).
    qf: i32 = 1,
    quantizer: Quantizer = .{},

    pub fn deinit(self: *LossyFrameData) void {
        for (&self.dc) |p| self.allocator.free(p);
        for (&self.ac) |p| self.allocator.free(p);
        self.allocator.free(self.quant_dc);
        self.* = undefined;
    }
};

/// The frame-data stage: gaborish-compensated XYB planes in, quantized
/// coefficients out. `xyb_planes` are the padded (bw x bh) planes; they
/// are modified in place (gaborish compensation).
pub fn buildFrameData(
    allocator: Allocator,
    xyb_planes: *[3][]f32,
    xsize: usize,
    ysize: usize,
    distance: f32,
) !LossyFrameData {
    const fd = FrameDim.initVarDct(
        xsize,
        ysize,
        0,
        0,
    );
    const bw = fd.xsize_blocks * 8;
    const bh = fd.ysize_blocks * 8;

    // Copy into zero-padded whole-block planes (the caller's planes are
    // xsize-strided; the gaborish/DCT stages need bw x bh).
    var padded: [3][]f32 = undefined;
    var padded_ok = false;
    defer if (padded_ok) for (&padded) |p| allocator.free(p);
    for (&padded) |*p| {
        p.* = try allocator.alloc(f32, bw * bh);
        @memset(p.*, 0);
    }
    padded_ok = true;
    for (0..ysize) |y| {
        for (0..xsize) |x| {
            for (0..3) |c| {
                padded[c][y * bw + x] = xyb_planes.*[c][y * xsize + x];
            }
        }
    }

    // Gaborish compensation (encoder side).
    {
        var scratch: [3][]f32 = undefined;
        var scratch_ok = false;
        errdefer if (scratch_ok) for (&scratch) |p| allocator.free(p);
        for (&scratch) |*p| {
            p.* = try allocator.alloc(f32, bw * bh);
        }
        scratch_ok = true;
        transform_lossy.gaborishForward(
            padded,
            bw,
            bh,
            1.0,
            scratch,
        );
        for (&scratch) |p| allocator.free(p);
    }

    const q = 0.79 / distance;
    const quantizer = Quantizer.init(
        initialQuantDC(distance),
        q,
    );
    const qf = quantizer.quantFieldValue(q);

    var data = LossyFrameData{
        .allocator = allocator,
        .fd = fd,
        .quantizer = quantizer,
        .qf = qf,
    };
    errdefer data.deinit();

    const table = buildRawQuantTable();

    for (0..3) |c| {
        data.dc[c] = try allocator.alloc(i32, fd.xsize_blocks * fd.ysize_blocks);
        data.ac[c] = try allocator.alloc(i32, fd.xsize_blocks * fd.ysize_blocks * 64);
        @memset(data.ac[c], 0);
    }
    data.quant_dc = try allocator.alloc(u8, fd.xsize_blocks * fd.ysize_blocks);
    @memset(data.quant_dc, 0);

    const biases = opsin_mod.kDefaultQuantBias;
    _ = biases;

    var pixels: [64]f32 = undefined;
    var coefs: [64]f32 = undefined;
    var by: usize = 0;
    while (by < fd.ysize_blocks) : (by += 1) {
        var bx: usize = 0;
        while (bx < fd.xsize_blocks) : (bx += 1) {
            const block_base = (by * fd.xsize_blocks + bx) * 64;
            for ([3]usize{ 1, 0, 2 }) |c| {
                // Gather the (gaborished) block pixels.
                for (0..8) |yy| {
                    for (0..8) |xx| {
                        pixels[yy * 8 + xx] = padded[c][(by * 8 + yy) * bw + bx * 8 + xx];
                    }
                }
                transform_lossy.dct8Forward(&pixels, &coefs);
                // DC.
                data.dc[c][by * fd.xsize_blocks + bx] =
                    roundToInt(coefs[0] * quantizer.invDcStep(c));
                // AC: quantize against the effective inverse weights.
                const acb = data.ac[c][block_base..][0..64];
                for (1..64) |k| {
                    const w = table.inv_weights[c * 64 + k];
                    const v = coefs[k] *
                        (w * (quantizer.global_scale_float * @as(f32, @floatFromInt(qf))));
                    acb[k] = if (@abs(v) < 0.5) 0 else roundToInt(v);
                }
            }
        }
    }
    return data;
}

fn roundToInt(v: f32) i32 {
    return @intFromFloat(@round(v));
}

// ---------------------------------------------------------------------------
// Encoder: codestream
// ---------------------------------------------------------------------------

/// One modular stream scheduled for serialization.
const StreamEntry = struct {
    image: Image,
    sid: usize,
};

fn toAnsTokens(
    allocator: Allocator,
    src: []const Token,
) !std.ArrayList(ANSToken) {
    var out: std.ArrayList(ANSToken) = .empty;
    errdefer out.deinit(allocator);
    try out.ensureTotalCapacity(allocator, src.len);
    for (src) |t| {
        out.appendAssumeCapacity(.{
            .is_lz77_length = t.is_lz77_length,
            .context = t.context,
            .value = t.value,
        });
    }
    return out;
}

/// Writes the complete lossy codestream: headers, the byte-aligned patch
/// reference frame bytes (already encoded by the caller), and the main
/// VarDCT frame. `patch_phase` null = no patches.
pub fn encodeLossyFrame(
    allocator: Allocator,
    data: *LossyFrameData,
    xsize: usize,
    ysize: usize,
    bitdepth: u8,
    ref_frame_bytes: ?[]const u8,
    pd: ?*const patches_mod.PatchDictionary,
    opts: LossyEncodeOptions,
    writer: *BitWriter,
) !void {
    const fd = &data.fd;

    // --- Headers ---
    writer.write(8, 0xFF);
    writer.write(8, 0x0A);
    var sh: size_mod.SizeHeader = .{};
    try sh.set(@intCast(xsize), @intCast(ysize));
    try sh.write(writer);
    var metadata = metadata_mod.ImageMetadata.init(allocator);
    defer metadata.deinit();
    metadata.xyb_encoded = true;
    metadata.bit_depth.bits_per_sample = bitdepth;
    metadata.modular_16_bit_buffer_sufficient = true;
    metadata.color_encoding = color_mod.ColorEncoding.srgb();
    try metadata.write(writer);
    const ctd = metadata_mod.CustomTransformData{};
    try ctd.write(writer);
    writer.zeroPadToByte();

    if (ref_frame_bytes) |bytes| {
        for (bytes) |b| writer.write(8, b);
    }

    // --- Frame header ---
    var frame = FrameHeader.init(allocator);
    defer frame.deinit();
    frame.encoding = .var_dct;
    frame.frame_type = .regular;
    frame.color_transform = .xyb;
    frame.flags = if (pd != null) frame_mod.Flags.kPatches else 0;
    frame.is_last = true;
    frame.upsampling = 1;
    frame.x_qm_scale = 2;
    frame.b_qm_scale = 2;
    frame.loop_filter = .{ .gab = true, .epf_iters = 0 };
    try frame.write(&metadata, writer);

    // --- Modular stream images: VarDCTDC, ACMetadata, RAW quant table ---
    var entries: std.ArrayList(StreamEntry) = .empty;
    defer {
        for (entries.items) |*e| e.image.deinit();
        entries.deinit(allocator);
    }

    for (0..fd.num_dc_groups) |g| {
        const r = fd.dcGroupRect(g);
        var img = Image.create(allocator);
        errdefer img.deinit();
        for ([3]usize{ 1, 0, 2 }) |c| {
            var ch = try Channel.create(
                allocator,
                r.xsize,
                r.ysize,
                0,
                0,
            );
            for (0..r.ysize) |y| {
                for (0..r.xsize) |x| {
                    ch.row(y)[x] = data.dc[c][(r.y0 + y) * fd.xsize_blocks + r.x0 + x];
                }
            }
            try img.channels.append(allocator, ch);
        }
        const sid = ModularStreamId.varDCTDC(g).id(fd);
        try entries.append(
            allocator,
            .{ .image = img, .sid = sid },
        );
    }

    {
        var ac_metadata_size = try allocator.alloc(
            usize,
            fd.num_dc_groups,
        );
        defer allocator.free(ac_metadata_size);
        for (0..fd.num_dc_groups) |g| {
            const r = fd.dcGroupRect(g);
            var img = Image.create(allocator);
            errdefer img.deinit();
            const cr_w = (r.xsize + 7) >> 3;
            const cr_h = (r.ysize + 7) >> 3;
            for (0..2) |_| {
                const ch = try Channel.create(
                    allocator,
                    cr_w,
                    cr_h,
                    3,
                    3,
                );
                @memset(ch.plane.data, 0);
                try img.channels.append(allocator, ch);
            }
            {
                const count = r.xsize * r.ysize;
                const ch = try Channel.create(
                    allocator,
                    count,
                    2,
                    0,
                    0,
                );
                @memset(ch.plane.data, 0);
                var i: usize = 0;
                while (i < count) : (i += 1) {
                    ch.row(1)[i] = data.qf - 1;
                }
                try img.channels.append(allocator, ch);
            }
            {
                const ch = try Channel.create(
                    allocator,
                    r.xsize,
                    r.ysize,
                    0,
                    0,
                );
                @memset(ch.plane.data, 4); // epf_sharpness default fill
                try img.channels.append(allocator, ch);
            }
            ac_metadata_size[g] = r.xsize * r.ysize;
            const sid = ModularStreamId.acMetadata(g).id(fd);
            try entries.append(
                allocator,
                .{ .image = img, .sid = sid },
            );
        }
    }

    {
        const table = buildRawQuantTable();
        var raw_table = quant_mod.RawQuantTable{
            .den = raw_table_den,
        };
        raw_table.values = table.ints;
        const img = try quant_mod.buildQuantTableImage(
            allocator,
            &raw_table,
            0,
        );
        const sid = ModularStreamId.quantTable(0).id(fd);
        try entries.append(
            allocator,
            .{ .image = img, .sid = sid },
        );
    }

    // --- Train the global tree over all streams ---
    var train_streams: std.ArrayList(TrainStream) = .empty;
    defer train_streams.deinit(allocator);
    for (entries.items) |*e| {
        try train_streams.append(
            allocator,
            .{ .image = &e.image, .group_id = e.sid },
        );
    }
    var mopts = options.ModularOptions{};
    mopts.splitting_heuristics_properties = &.{ 0, 15, 9, 10, 11, 12, 13 };
    mopts.max_property_values = 96;
    mopts.max_chan_size = 0xFFFFFF;
    var tree = try stream.trainTree(
        allocator,
        train_streams.items,
        &mopts,
    );
    defer tree.deinit(allocator);
    var decoded_tree = ma.Tree.empty;
    errdefer decoded_tree.deinit(allocator);
    var tree_tokens: std.ArrayList(Token) = .empty;
    defer tree_tokens.deinit(allocator);
    try stream.tokenizeTree(
        allocator,
        tree.items,
        &tree_tokens,
        &decoded_tree,
    );
    tree.deinit(allocator);
    tree = decoded_tree;

    // --- Tokenize each stream ---
    const num_streams = ModularStreamId.num(fd, 1);
    var tokens: std.ArrayList(std.ArrayList(Token)) = .empty;
    defer {
        for (tokens.items) |*list| list.deinit(allocator);
        tokens.deinit(allocator);
    }
    try tokens.resize(allocator, num_streams);
    for (tokens.items) |*list| list.* = .empty;
    var image_widths: std.ArrayList(usize) = .empty;
    defer image_widths.deinit(allocator);
    try image_widths.resize(allocator, num_streams);
    @memset(image_widths.items, 0);
    const wp_header = context_predict.weighted.Header.defaults;
    for (entries.items) |*e| {
        try stream.encodeStreamTokens(
            allocator,
            &e.image,
            e.sid,
            mopts.max_chan_size,
            tree.items,
            &wp_header,
            &tokens.items[e.sid],
        );
        image_widths.items[e.sid] = stream.maxStreamChannelWidth(
            &e.image,
            mopts.max_chan_size,
        );
    }

    // --- AC tokens per group ---
    var ac_tokens: std.ArrayList(std.ArrayList(Token)) = .empty;
    defer {
        for (ac_tokens.items) |*list| list.deinit(allocator);
        ac_tokens.deinit(allocator);
    }
    try ac_tokens.resize(allocator, fd.num_groups);
    for (ac_tokens.items) |*list| list.* = .empty;

    var orders: [3][64]u32 = .{
        coeff_order_mod.naturalOrder8x8(),
        coeff_order_mod.naturalOrder8x8(),
        coeff_order_mod.naturalOrder8x8(),
    };
    var used_orders: u32 = 0;
    if (opts.coeff_orders) {
        const res = try coeff_order_mod.computeCoeffOrder(
            allocator,
            fd,
            .{ 0, 0, 0 },
            .{ 0, 0, 0 },
            .{
                .data = .{ data.ac[0], data.ac[1], data.ac[2] },
                .widths = .{
                    fd.xsize_blocks,
                    fd.xsize_blocks,
                    fd.xsize_blocks,
                },
            },
        );
        orders = res.orders;
        used_orders = res.used_orders;
    }
    {
        const order_slices = [3][]const u32{
            orders[0][0..],
            orders[1][0..],
            orders[2][0..],
        };
        var cache = try ac_coefficient.GroupCoeffCache.init(
            allocator,
        );
        defer cache.deinit();
        const coeffs = ac_coefficient.GroupCoefficients{
            .planes = .{ data.ac[0], data.ac[1], data.ac[2] },
            .widths = .{
                fd.xsize_blocks,
                fd.xsize_blocks,
                fd.xsize_blocks,
            },
        };
        const qdc = ac_coefficient.QdcImage{
            .data = data.quant_dc,
            .stride = fd.xsize_blocks,
        };
        var bcm = block_context.BlockCtxMap.init(allocator);
        defer bcm.deinit();
        try bcm.setDefault();
        for (0..fd.num_groups) |g| {
            const rect = fd.blockGroupRect(g);
            try ac_coefficient.tokenizeCoefficients(
                allocator,
                order_slices,
                .{
                    .x0 = rect.x0,
                    .y0 = rect.y0,
                    .xsize = rect.xsize,
                    .ysize = rect.ysize,
                },
                &coeffs,
                .{ 0, 0, 0 },
                .{ 0, 0, 0 },
                &cache,
                &qdc,
                &bcm,
                &ac_tokens.items[g],
            );
        }
    }

    // AC tokens converted to ANS tokens (shared by the histograms and the
    // group sections).
    var ac_tokens_ans: std.ArrayList(std.ArrayList(ANSToken)) = .empty;
    defer {
        for (ac_tokens_ans.items) |*l| l.deinit(allocator);
        ac_tokens_ans.deinit(allocator);
    }
    for (ac_tokens.items) |*list| {
        const conv = try toAnsTokens(
            allocator,
            list.items,
        );
        try ac_tokens_ans.append(allocator, conv);
    }

    // --- Sections ---
    const num_toc_entries = toc_mod.numTocEntries(
        fd.num_groups,
        fd.num_dc_groups,
        1,
    );
    const single_section = num_toc_entries == 1;
    var sections: std.ArrayList(BitWriter) = .empty;
    defer {
        for (sections.items) |*sw| sw.deinit();
        sections.deinit(allocator);
    }
    try sections.resize(allocator, num_toc_entries);
    for (sections.items) |*sw| sw.* = BitWriter.init(allocator);

    var histogram_params = enc_ans.HistogramParams{
        .clustering = .kFast,
        .uint_method = .kFast,
        .lz77_method = .kRLE,
        .ans_histogram_strategy = .kApproximate,
    };

    var all_tokens: std.ArrayList(std.ArrayList(ANSToken)) = .empty;
    defer {
        for (all_tokens.items) |*l| l.deinit(allocator);
        all_tokens.deinit(allocator);
    }
    try all_tokens.resize(allocator, num_streams);
    for (all_tokens.items) |*l| l.* = .empty;
    for (tokens.items, 0..) |*list, sid| {
        for (list.items) |t| {
            try all_tokens.items[sid].append(
                allocator,
                .{
                    .is_lz77_length = t.is_lz77_length,
                    .context = t.context,
                    .value = t.value,
                },
            );
        }
    }

    var stream_code = enc_ans.EntropyEncodingData.init(
        allocator,
    );
    defer stream_code.deinit();
    var ac_code = enc_ans.EntropyEncodingData.init(
        allocator,
    );
    defer ac_code.deinit();

    // DC global section.
    {
        const sec = &sections.items[0];
        if (pd) |p| {
            try p.encode(sec);
        }
        var ddc = quant_mod.DequantDC{}; // all_default
        try ddc.write(sec);
        const qp = quant_mod.QuantizerParams{
            .global_scale = data.quantizer.global_scale,
            .quant_dc = data.quantizer.quant_dc,
        };
        try qp.write(sec);
        var bcm = block_context.BlockCtxMap.init(allocator);
        defer bcm.deinit();
        try bcm.setDefault();
        try bcm.write(sec);
        // CfL DC: non-default, zero base correlations and zero DC factors
        // (no chroma-from-luma anywhere).
        writeCflDCZero(sec);
        sec.write(1, 1); // has_tree
        var tree_token_streams: [1]std.ArrayList(ANSToken) = .{.empty};
        defer tree_token_streams[0].deinit(allocator);
        for (tree_tokens.items) |t| {
            try tree_token_streams[0].append(
                allocator,
                .{
                    .is_lz77_length = t.is_lz77_length,
                    .context = t.context,
                    .value = t.value,
                },
            );
        }
        var tree_code = enc_ans.EntropyEncodingData.init(
            allocator,
        );
        defer tree_code.deinit();
        _ = try stream.buildHistograms(
            allocator,
            &histogram_params,
            ma.kNumTreeContexts,
            tree_token_streams[0..],
            &tree_code,
            sec,
        );
        _ = try stream.writeTokens(
            allocator,
            tree_token_streams[0].items,
            &tree_code,
            0,
            sec,
        );
        var stream_histo_params = histogram_params;
        stream_histo_params.image_widths = image_widths.items;
        _ = try stream.buildHistograms(
            allocator,
            &stream_histo_params,
            (tree.items.len + 1) / 2,
            all_tokens.items,
            &stream_code,
            sec,
        );
    }

    // DC group sections.
    for (0..fd.num_dc_groups) |g| {
        const sec = if (single_section)
            &sections.items[0]
        else
            &sections.items[1 + g];
        try writeDCGroupSection(
            allocator,
            sec,
            fd,
            entries.items,
            all_tokens.items,
            &stream_code,
            g,
        );
    }

    // AC global section: dequant matrices, num_histograms, orders and the
    // shared AC histograms (ac_code comes back for the group sections).
    {
        const sec = if (single_section)
            &sections.items[0]
        else
            &sections.items[1 + fd.num_dc_groups];
        try writeAcGlobalSection(
            allocator,
            sec,
            fd,
            entries.items,
            all_tokens.items,
            &stream_code,
            &histogram_params,
            used_orders,
            &orders,
            ac_tokens_ans.items,
            &ac_code,
        );
    }

    // AC group sections.
    for (0..fd.num_groups) |g| {
        const acw = if (single_section)
            &sections.items[0]
        else
            &sections.items[
                toc_mod.acGroupIndex(
                    0,
                    g,
                    fd.num_groups,
                    fd.num_dc_groups,
                )
            ];
        _ = try stream.writeTokens(
            allocator,
            ac_tokens_ans.items[g].items,
            &ac_code,
            0,
            acw,
        );
    }

    for (sections.items) |*sw| {
        sw.zeroPadToByte();
    }

    // --- TOC + section bytes ---
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
    writer.zeroPadToByte();
}

/// CfL DC header with all-zero correlations (no chroma-from-luma).
fn writeCflDCZero(writer: *BitWriter) void {
    writer.write(1, 0); // not all-default
    const color_factor_dist = fields.U32Enc.init(
        fields.val(84),
        fields.val(256),
        fields.bitsOffset(8, 2),
        fields.bitsOffset(16, 258),
    );
    U32Coder.write(
        color_factor_dist,
        84,
        writer,
    ) catch unreachable;
    fields_ext.writeF16(0.0, writer) catch unreachable;
    fields_ext.writeF16(0.0, writer) catch unreachable;
    writer.write(8, 128); // ytox_dc = 0
    writer.write(8, 128); // ytob_dc = 0
}

fn writeDCGroupSection(
    allocator: Allocator,
    writer: *BitWriter,
    fd: *const FrameDim,
    entries: []const StreamEntry,
    tokens: []const std.ArrayList(ANSToken),
    code: *const enc_ans.EntropyEncodingData,
    g: usize,
) !void {
    writer.write(2, 0); // extra_dc_precision

    const vdc_sid = ModularStreamId.varDCTDC(g).id(fd);
    const meta_sid = ModularStreamId.acMetadata(g).id(fd);
    for (entries) |*e| {
        if (e.sid != vdc_sid) continue;
        try writeModularStream(
            allocator,
            writer,
            tokens[e.sid].items,
            code,
        );
    }

    const r = fd.dcGroupRect(g);
    const nb_bits = ceilLog2Nonzero(r.xsize * r.ysize);
    if (nb_bits != 0) {
        const count = r.xsize * r.ysize; // all blocks are DCT-8
        writer.write(@intCast(nb_bits), count - 1);
    }
    for (entries) |*e| {
        if (e.sid != meta_sid) continue;
        try writeModularStream(
            allocator,
            writer,
            tokens[e.sid].items,
            code,
        );
    }
}

fn writeModularStream(
    allocator: Allocator,
    writer: *BitWriter,
    ans_tokens: []const ANSToken,
    code: *const enc_ans.EntropyEncodingData,
) !void {
    var gh = stream.GroupHeader.init(allocator);
    defer gh.deinit();
    gh.use_global_tree = true;
    try gh.write(writer);
    _ = try stream.writeTokens(
        allocator,
        ans_tokens,
        code,
        0,
        writer,
    );
}

/// Writes the AC global section: DequantMatrices (RAW table 0 + library
/// rest), num_histograms, coefficient orders and the shared AC
/// histograms. `ac_code` (caller-owned) receives the entropy code the AC
/// group sections must use.
fn writeAcGlobalSection(
    allocator: Allocator,
    writer: *BitWriter,
    fd: *const FrameDim,
    entries: []const StreamEntry,
    tokens: []const std.ArrayList(ANSToken),
    code: *const enc_ans.EntropyEncodingData,
    histogram_params: *const enc_ans.HistogramParams,
    used_orders: u32,
    orders: *const [3][64]u32,
    ac_tokens_ans: []std.ArrayList(ANSToken),
    ac_code: *enc_ans.EntropyEncodingData,
) !void {
    writer.write(1, 0); // DequantMatrices not all default
    const qt_sid = ModularStreamId.quantTable(0).id(fd);
    for (0..quant_mod.kNumQuantTables) |i| {
        if (i == 0) {
            writer.write(3, @intFromEnum(quant_mod.QuantMode.raw));
            try fields_ext.writeF16(raw_table_den, writer);
            for (entries) |*e| {
                if (e.sid != qt_sid) continue;
                try writeModularStream(
                    allocator,
                    writer,
                    tokens[e.sid].items,
                    code,
                );
            }
        } else {
            writer.write(
                3,
                @intFromEnum(quant_mod.QuantMode.library),
            );
        }
    }
    const num_histo_bits = ceilLog2Nonzero(fd.num_groups);
    if (num_histo_bits != 0) {
        writer.write(@intCast(num_histo_bits), 0); // num_histograms - 1
    }
    try U32Coder.write(
        coeff_order_mod.kOrderEnc,
        used_orders,
        writer,
    );
    try coeff_order_mod.encodeCoeffOrders(
        allocator,
        used_orders,
        orders,
        writer,
    );

    var bcm = block_context.BlockCtxMap.init(allocator);
    defer bcm.deinit();
    try bcm.setDefault();
    _ = try stream.buildHistograms(
        allocator,
        histogram_params,
        bcm.numACContexts(),
        ac_tokens_ans,
        ac_code,
        writer,
    );
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

/// The decoded lossy image: three linear-sRGB float planes of
/// `xsize * ysize` samples.
pub const LossyImage = struct {
    allocator: Allocator,
    xsize: usize,
    ysize: usize,
    planes: [3][]f32,

    pub fn deinit(self: *LossyImage) void {
        for (&self.planes) |p| self.allocator.free(p);
        self.* = undefined;
    }
};

/// CfL map state (ColorCorrelation): the DC header factors plus the
/// per-color-tile AC maps.
const CflState = struct {
    color_factor: f32 = 84,
    base_x: f32 = 0,
    base_b: f32 = 1.0, // kYToBRatio
    ytox_dc: i32 = 0,
    ytob_dc: i32 = 0,
    ytox_map: []i32 = &.{},
    ytob_map: []i32 = &.{},
    map_w: usize = 0,
    map_h: usize = 0,

    fn readDC(self: *CflState, br: *BitReader) !void {
        self.* = .{};
        if (br.readBits(1) != 0) {
            // All-default: base_b = kYToBRatio.
            return;
        }
        const color_factor_dist = fields.U32Enc.init(
            fields.val(84),
            fields.val(256),
            fields.bitsOffset(8, 2),
            fields.bitsOffset(16, 258),
        );
        const cf = U32Coder.read(color_factor_dist, br);
        if (cf == 0) return error.InvalidCflDC;
        self.color_factor = @floatFromInt(cf);
        self.base_x = try fields_ext.readF16(br);
        if (@abs(self.base_x) > 4.0) return error.InvalidCflDC;
        self.base_b = try fields_ext.readF16(br);
        if (@abs(self.base_b) > 4.0) return error.InvalidCflDC;
        self.ytox_dc = @as(i32, @intCast(br.readBits(8))) + std.math.minInt(i8);
        self.ytob_dc = @as(i32, @intCast(br.readBits(8))) + std.math.minInt(i8);
    }

    fn ytoXRatio(self: *const CflState, factor: i32) f32 {
        return self.base_x + @as(f32, @floatFromInt(factor)) /
            self.color_factor;
    }

    fn ytoBRatio(self: *const CflState, factor: i32) f32 {
        return self.base_b + @as(f32, @floatFromInt(factor)) /
            self.color_factor;
    }
};

/// One frame's decoded dequant tables (RAW or the default library set).
const DequantTables = struct {
    /// table[c * 64 + k]: the AC dequant value for block index k.
    table: [3 * 64]f32 = std.mem.zeroes([3 * 64]f32),

    /// Parses `DequantMatrices::Decode`; supports the RAW table 0 (this
    /// codec's lossy output and the JPEG transcode shape) and the
    /// all-default library tables (cjxl). Everything else is rejected.
    fn read(
        self: *DequantTables,
        allocator: Allocator,
        br: *BitReader,
        global: *stream.EntropyCode,
        fd: *const FrameDim,
        library_only: bool,
    ) !void {
        const all_default = br.readBits(1) != 0;
        if (all_default) {
            // Library table 0 (Libshroom) computed with the accurate pow.
            const weights = transform_lossy.libshroomDct8QuantWeights();
            for (0..3 * 64) |i| {
                self.table[i] = 1.0 / weights[i];
            }
            return;
        }
        if (library_only) return error.UnsupportedVarDct;
        for (0..quant_mod.kNumQuantTables) |i| {
            const mode = br.readBits(3);
            if (i == 0) {
                if (mode != @intFromEnum(quant_mod.QuantMode.raw)) {
                    return error.UnsupportedVarDct;
                }
                var raw = quant_mod.RawQuantTable{};
                try raw.read(br); // F16 denominator
                if (raw.den < quant_mod.kAlmostZero)
                    return error.InvalidQuantTable;
                var img = try quant_mod.buildQuantTableImage(
                    allocator,
                    &.{},
                    0,
                );
                defer img.deinit();
                const sid = ModularStreamId.quantTable(0).id(fd);
                try stream.decodeStream(.{
                    .allocator = allocator,
                    .br = br,
                    .image = &img,
                    .group_id = sid,
                    .max_chan_size = 0xFFFFFF,
                    .global = global,
                    .undo_transforms = true,
                    .bitdepth = 8,
                });
                try quant_mod.readQuantTableImage(
                    &img,
                    0,
                    &raw,
                );
                for (0..3 * 64) |k| {
                    if (raw.values[k] <= 0 or raw.values[k] >= 65536) {
                        return error.InvalidQuantTable;
                    }
                    self.table[k] = raw.den *
                        @as(f32, @floatFromInt(raw.values[k]));
                }
            } else {
                if (mode != @intFromEnum(quant_mod.QuantMode.library)) {
                    return error.UnsupportedVarDct;
                }
            }
        }
    }
};

/// Adaptive DC smoothing (compressed_dc.cc): 3x3 kernel with the
/// quant-gap-scaled strength. `dc_factors` are the DC dequant steps.
fn adaptiveDCSmoothing(
    dc: *[3][]f32,
    xsize: usize,
    ysize: usize,
    dc_factors: *const [3]f32,
    allocator: Allocator,
) !void {
    if (ysize <= 2 or xsize <= 2) return;
    const w1: f32 = 0.20345139757231578;
    const w2: f32 = 0.0334829185968739;
    const w0: f32 = 1.0 - 4.0 * (w1 + w2);
    var smoothed: [3][]f32 = undefined;
    var ok = false;
    errdefer if (ok) for (&smoothed) |p| allocator.free(p);
    for (&smoothed) |*p| {
        p.* = try allocator.alloc(f32, xsize * ysize);
    }
    ok = true;
    defer for (&smoothed) |p| allocator.free(p);

    for (0..3) |c| {
        @memcpy(smoothed[c][0..xsize], dc[c][0..xsize]);
        @memcpy(
            smoothed[c][(ysize - 1) * xsize ..][0..xsize],
            dc[c][(ysize - 1) * xsize ..][0..xsize],
        );
    }
    var y: usize = 1;
    while (y + 1 < ysize) : (y += 1) {
        for (0..3) |c| {
            smoothed[c][y * xsize] = dc[c][y * xsize];
            smoothed[c][y * xsize + xsize - 1] = dc[c][y * xsize + xsize - 1];
        }
        var x: usize = 1;
        while (x + 1 < xsize) : (x += 1) {
            var mc: [3]f32 = undefined;
            var sm: [3]f32 = undefined;
            var gap: f32 = 0.5;
            for (0..3) |c| {
                const plane = dc[c];
                const tl = plane[(y - 1) * xsize + x - 1];
                const tc = plane[(y - 1) * xsize + x];
                const tr = plane[(y - 1) * xsize + x + 1];
                const ml = plane[y * xsize + x - 1];
                const m = plane[y * xsize + x];
                const mr = plane[y * xsize + x + 1];
                const bl = plane[(y + 1) * xsize + x - 1];
                const bc = plane[(y + 1) * xsize + x];
                const br = plane[(y + 1) * xsize + x + 1];
                const corner = (tl + tr) + (bl + br);
                const side = (ml + mr) + (tc + bc);
                // MulAdd chain of ComputePixelChannel.
                sm[c] = @mulAdd(
                    f32,
                    corner,
                    w2,
                    @mulAdd(f32, side, w1, m * w0),
                );
                mc[c] = m;
                gap = @max(gap, @abs((m - sm[c]) / dc_factors[c]));
            }
            var factor: f32 = @mulAdd(f32, -4.0, gap, 3.0);
            if (factor < 0) factor = 0;
            for (0..3) |c| {
                smoothed[c][y * xsize + x] = @mulAdd(
                    f32,
                    sm[c] - mc[c],
                    factor,
                    mc[c],
                );
            }
        }
    }
    for (0..3) |c| {
        @memcpy(dc[c], smoothed[c]);
    }
}

/// The full lossy VarDCT frame decode. `codestream` is the frame's
/// section bytes with `section_starts/sizes` from the TOC (the same
/// layout `decodeTranscodeFrame` consumes). `refs` carries the decoded
/// patch reference frames. Produces linear-sRGB float planes.
pub fn decodeLossyFrame(
    allocator: Allocator,
    metadata: *const metadata_mod.ImageMetadata,
    frame: *const FrameHeader,
    xsize: usize,
    ysize: usize,
    sections: []const u8,
    section_starts: []const usize,
    section_sizes: []const u32,
    refs: *patches_mod.RefStore,
) !LossyImage {
    if (frame.encoding != .var_dct) return error.UnsupportedVarDct;
    if (frame.frame_type != .regular) return error.UnsupportedFrameFeatures;
    if (!frame.is_last) return error.UnsupportedFrameFeatures;
    if (frame.upsampling != 1) return error.UnsupportedFrameFeatures;
    if (frame.passes.num_passes != 1) return error.UnsupportedFrameFeatures;
    if (frame.color_transform != .xyb) return error.UnsupportedColorTransform;
    if (frame.loop_filter.epf_iters != 0) return error.UnsupportedFrameFeatures;
    if (metadata.numExtraChannels() != 0) return error.UnsupportedFrameFeatures;
    if (frame.custom_size_or_origin) return error.UnsupportedFrameFeatures;
    if (frame.flags &
        (frame_mod.Flags.kNoise |
            frame_mod.Flags.kSplines |
            frame_mod.Flags.kUseDcFrame) != 0)
    {
        return error.UnsupportedFrameFeatures;
    }
    const wants_patches = frame.flags & frame_mod.Flags.kPatches != 0;

    const fd = FrameDim.initVarDct(
        xsize,
        ysize,
        0,
        0,
    );
    const num_toc = section_sizes.len;
    const single_section = num_toc == 1;

    // Single-section frames (one group, one pass) carry everything in
    // section 0, read sequentially; otherwise one reader per section.
    var sec0: BitReader = undefined;
    var sec0_open = false;
    defer if (sec0_open) sec0.close() catch {};
    if (single_section) {
        const end0 = section_starts[0] + section_sizes[0];
        sec0 = BitReader.init(sections[section_starts[0]..end0]);
        sec0_open = true;
    }

    var st_global = stream.EntropyCode.init(allocator);
    defer st_global.deinit();
    var block_ctx_map = block_context.BlockCtxMap.init(allocator);
    defer block_ctx_map.deinit();
    var cfl = CflState{};
    var ddc = quant_mod.DequantDC{};
    var quantizer: ?Quantizer = null;
    var tables = DequantTables{};
    var pd = patches_mod.PatchDictionary.init(allocator);
    defer pd.deinit();

    var dc: [3][]f32 = .{ &.{}, &.{}, &.{} }; // dequantized DC planes
    var dc_ok = false;
    errdefer if (dc_ok) for (&dc) |p| allocator.free(p);
    defer if (dc_ok) for (&dc) |p| allocator.free(p);
    var quant_dc_buckets: []u8 = &.{};
    var qf_field: []i32 = &.{};
    var qf_ok = false;
    var qdc_ok = false;
    errdefer if (qdc_ok) allocator.free(quant_dc_buckets);
    errdefer if (qf_ok) allocator.free(qf_field);
    defer if (qdc_ok) allocator.free(quant_dc_buckets);
    defer if (qf_ok) allocator.free(qf_field);
    var orders: [3][64]u32 = .{
        coeff_order_mod.naturalOrder8x8(),
        coeff_order_mod.naturalOrder8x8(),
        coeff_order_mod.naturalOrder8x8(),
    };
    var ac_code = dec_ans.ANSCode{};
    defer ac_code.deinit(allocator);
    var ac_context_map: std.ArrayList(u8) = .empty;
    defer ac_context_map.deinit(allocator);

    // --- DC global ---
    {
        var sec_fresh: BitReader = undefined;
        var sec_fresh_open = false;
        defer if (sec_fresh_open) sec_fresh.close() catch {};
        var sec: *BitReader = undefined;
        if (single_section) {
            sec = &sec0;
        } else {
            const end0 = section_starts[0] + section_sizes[0];
            sec_fresh = BitReader.init(sections[section_starts[0]..end0]);
            sec_fresh_open = true;
            sec = &sec_fresh;
        }
        if (wants_patches) {
            var uses_ec = false;
            try pd.decode(sec, fd.xsize_blocks * 8, fd.ysize_blocks * 8, 0, refs, &uses_ec);
            if (uses_ec) return error.UnsupportedFrameFeatures;
        }
        try ddc.read(sec);
        var qp = quant_mod.QuantizerParams{};
        try qp.read(sec);
        quantizer = Quantizer.fromParams(
            qp.global_scale,
            qp.quant_dc,
        );
        try block_ctx_map.read(sec);
        try cfl.readDC(sec);
        // Modular global info.
        const has_tree = sec.readBits(1) != 0;
        if (!has_tree) return error.UnsupportedVarDct;
        const tree_size_limit = @min(
            @as(usize, 1) << 22,
            1024 + fd.xsize * fd.ysize * 3 / 16,
        );
        var tree_code = dec_ans.ANSCode{};
        defer tree_code.deinit(allocator);
        var tree_ctx_map: std.ArrayList(u8) = .empty;
        defer tree_ctx_map.deinit(allocator);
        try dec_ans.decodeHistograms(
            allocator,
            sec,
            ma.kNumTreeContexts,
            &tree_code,
            &tree_ctx_map,
            false,
        );
        var tree_reader = try stream.AnsMaReader.create(
            allocator,
            &tree_code,
            tree_ctx_map.items,
            0,
            sec,
        );
        defer tree_reader.ans.deinit();
        try ma.decodeTree(
            sec,
            &tree_reader,
            &st_global.tree,
            allocator,
            tree_size_limit,
        );
        try dec_ans.decodeHistograms(
            allocator,
            sec,
            st_global.numContexts(),
            &st_global.code,
            &st_global.context_map,
            false,
        );
    }

    const q = quantizer.?;
    // DC dequant factors (MulDC) and CfL DC factors.
    var dc_factors: [3]f32 = undefined;
    for (0..3) |c| {
        dc_factors[c] = q.dcStep(c);
    }
    const cfl_dc_factors = [3]f32{
        cfl.ytoXRatio(cfl.ytox_dc),
        0,
        cfl.ytoBRatio(cfl.ytob_dc),
    };

    for (0..3) |c| {
        dc[c] = try allocator.alloc(
            f32,
            fd.xsize_blocks * fd.ysize_blocks,
        );
        @memset(dc[c], 0);
    }
    dc_ok = true;
    quant_dc_buckets = try allocator.alloc(
        u8,
        fd.xsize_blocks * fd.ysize_blocks,
    );
    @memset(quant_dc_buckets, 0);
    qdc_ok = true;
    qf_field = try allocator.alloc(i32, fd.xsize_blocks * fd.ysize_blocks);
    @memset(qf_field, 1);
    qf_ok = true;

    cfl.map_w = (fd.xsize_blocks + 7) >> 3;
    cfl.map_h = (fd.ysize_blocks + 7) >> 3;
    cfl.ytox_map = try allocator.alloc(i32, cfl.map_w * cfl.map_h);
    cfl.ytob_map = try allocator.alloc(i32, cfl.map_w * cfl.map_h);
    @memset(cfl.ytox_map, 0);
    @memset(cfl.ytob_map, 0);
    defer {
        allocator.free(cfl.ytox_map);
        allocator.free(cfl.ytob_map);
    }

    // --- DC groups ---
    for (0..fd.num_dc_groups) |g| {
        var sec_fresh: BitReader = undefined;
        var sec_fresh_open = false;
        defer if (sec_fresh_open) sec_fresh.close() catch {};
        var sec: *BitReader = undefined;
        if (single_section) {
            sec = &sec0;
        } else {
            const sg = 1 + g;
            const start = section_starts[sg];
            sec_fresh = BitReader.init(
                sections[start .. start + section_sizes[sg]],
            );
            sec_fresh_open = true;
            sec = &sec_fresh;
        }
        const r = fd.dcGroupRect(g);

        const extra_precision = sec.readBits(2);
        if (extra_precision != 0) return error.UnsupportedFrameFeatures;
        // VarDCTDC stream.
        {
            var img = Image.create(allocator);
            defer img.deinit();
            for (0..3) |_| {
                const ch = try Channel.create(
                    allocator,
                    r.xsize,
                    r.ysize,
                    0,
                    0,
                );
                try img.channels.append(allocator, ch);
            }
            const sid = ModularStreamId.varDCTDC(g).id(&fd);
            try stream.decodeStream(.{
                .allocator = allocator,
                .br = sec,
                .image = &img,
                .group_id = sid,
                .max_chan_size = 0xFFFFFF,
                .global = &st_global,
                .undo_transforms = true,
                .bitdepth = 8,
            });
            // DequantDC: Y from ch0, X from ch1 (+ Y*cfl_x), B from
            // ch2 (+ Y*cfl_b) — "XYB is encoded as YX(B-Y)".
            for (0..r.ysize) |y| {
                for (0..r.xsize) |x| {
                    const gy = r.y0 + y;
                    const gx = r.x0 + x;
                    const qy: f32 = @floatFromInt(img.channels.items[0].row(y)[x]);
                    const qx: f32 = @floatFromInt(img.channels.items[1].row(y)[x]);
                    const qb: f32 = @floatFromInt(img.channels.items[2].row(y)[x]);
                    const dy = qy * dc_factors[1];
                    dc[1][gy * fd.xsize_blocks + gx] = dy;
                    dc[0][gy * fd.xsize_blocks + gx] = @mulAdd(
                        f32,
                        dy,
                        cfl_dc_factors[0],
                        qx * dc_factors[0],
                    );
                    dc[2][gy * fd.xsize_blocks + gx] = @mulAdd(
                        f32,
                        dy,
                        cfl_dc_factors[2],
                        qb * dc_factors[2],
                    );
                }
            }
        }

        // ACMetadata stream.
        {
            const upper_bound = r.xsize * r.ysize;
            const nb_bits = ceilLog2Nonzero(upper_bound);
            const count: usize = if (nb_bits != 0)
                @intCast(sec.readBits(nb_bits) + 1)
            else
                1;
            var img = Image.create(allocator);
            defer img.deinit();
            const cr_w = (r.xsize + 7) >> 3;
            const cr_h = (r.ysize + 7) >> 3;
            for (0..2) |_| {
                const ch = try Channel.create(
                    allocator,
                    cr_w,
                    cr_h,
                    3,
                    3,
                );
                try img.channels.append(allocator, ch);
            }
            {
                const ch = try Channel.create(
                    allocator,
                    count,
                    2,
                    0,
                    0,
                );
                try img.channels.append(allocator, ch);
            }
            {
                const ch = try Channel.create(
                    allocator,
                    r.xsize,
                    r.ysize,
                    0,
                    0,
                );
                try img.channels.append(allocator, ch);
            }
            const sid = ModularStreamId.acMetadata(g).id(&fd);
            try stream.decodeStream(.{
                .allocator = allocator,
                .br = sec,
                .image = &img,
                .group_id = sid,
                .max_chan_size = 0xFFFFFF,
                .global = &st_global,
                .undo_transforms = true,
                .bitdepth = 8,
            });
            // CfL maps.
            {
                const map_x0 = r.x0 >> 3;
                const map_y0 = r.y0 >> 3;
                for (0..cr_h) |my| {
                    for (0..cr_w) |mx| {
                        cfl.ytox_map[(map_y0 + my) * cfl.map_w + map_x0 + mx] =
                            std.math.clamp(
                                img.channels.items[0].row(my)[mx],
                                -128,
                                127,
                            );
                        cfl.ytob_map[(map_y0 + my) * cfl.map_w + map_x0 + mx] =
                            std.math.clamp(
                                img.channels.items[1].row(my)[mx],
                                -128,
                                127,
                            );
                    }
                }
            }
            if (count != r.xsize * r.ysize) return error.UnsupportedVarDct; // DCT-8 only
            for (0..count) |i| {
                const acs = img.channels.items[2].row(0)[i];
                const qf = img.channels.items[2].row(1)[i];
                if (acs != 0) return error.UnsupportedVarDct; // only DCT-8
                if (qf < 0 or qf >= quant_mod_kQuantMax())
                    return error.InvalidQuantField;
            }
            // Unpack the qf field and DC buckets row-wise.
            var num: usize = 0;
            for (0..r.ysize) |y| {
                for (0..r.xsize) |x| {
                    const sharpness = img.channels.items[3].row(y)[x];
                    if (sharpness < 0 or sharpness >= 8)
                        return error.UnsupportedFrameFeatures;
                    const qf = img.channels.items[2].row(1)[num];
                    qf_field[(r.y0 + y) * fd.xsize_blocks + r.x0 + x] = 1 +
                        std.math.clamp(
                            qf,
                            0,
                            quant_mod_kQuantMax() - 1,
                        );
                    num += 1;
                }
            }
            // DC buckets (num_dc_ctxs == 1 -> all zero; the general form
            // follows DequantDC's threshold scan).
            if (block_ctx_map.num_dc_ctxs > 1) {
                const thresholds_x = block_ctx_map.dc_thresholds[0].items;
                const thresholds_y = block_ctx_map.dc_thresholds[1].items;
                const thresholds_b = block_ctx_map.dc_thresholds[2].items;
                // DC planes hold floats now; libjxl buckets the *quantized*
                // DC ints. Reconstruct the ints through the inverse steps.
                for (0..r.ysize) |y| {
                    for (0..r.xsize) |x| {
                        const gy = r.y0 + y;
                        const gx = r.x0 + x;
                        const qy = intFromFloatRounded(dc[1][gy * fd.xsize_blocks + gx] / dc_factors[1]);
                        const qx = intFromFloatRounded(dc[0][gy * fd.xsize_blocks + gx] / dc_factors[0]);
                        const qb = intFromFloatRounded(dc[2][gy * fd.xsize_blocks + gx] / dc_factors[2]);
                        var bucket_x: u8 = 0;
                        var bucket_y: u8 = 0;
                        var bucket_b: u8 = 0;
                        for (thresholds_x) |t| {
                            if (qx > t) bucket_x += 1;
                        }
                        for (thresholds_y) |t| {
                            if (qy > t) bucket_y += 1;
                        }
                        for (thresholds_b) |t| {
                            if (qb > t) bucket_b += 1;
                        }
                        var bucket = @as(usize, bucket_x);
                        bucket *= thresholds_b.len + 1;
                        bucket += bucket_b;
                        bucket *= thresholds_y.len + 1;
                        bucket += bucket_y;
                        quant_dc_buckets[gy * fd.xsize_blocks + gx] = @intCast(bucket);
                    }
                }
            }
        }
    }

    // Adaptive DC smoothing (between DC and AC processing).
    if (frame.flags & frame_mod.Flags.kSkipAdaptiveDCSmoothing == 0) {
        try adaptiveDCSmoothing(
            &dc,
            fd.xsize_blocks,
            fd.ysize_blocks,
            &dc_factors,
            allocator,
        );
    }

    // --- AC global ---
    {
        var sec_fresh: BitReader = undefined;
        var sec_fresh_open = false;
        defer if (sec_fresh_open) sec_fresh.close() catch {};
        var sec: *BitReader = undefined;
        if (single_section) {
            sec = &sec0;
        } else {
            const s = 1 + fd.num_dc_groups;
            const start = section_starts[s];
            sec_fresh = BitReader.init(
                sections[start .. start + section_sizes[s]],
            );
            sec_fresh_open = true;
            sec = &sec_fresh;
        }
        try tables.read(
            allocator,
            sec,
            &st_global,
            &fd,
            false,
        );
        const num_histo_bits = ceilLog2Nonzero(fd.num_groups);
        const num_histograms: usize = 1 +
            (if (num_histo_bits != 0)
                @as(usize, @intCast(sec.readBits(num_histo_bits)))
            else
                0);
        if (num_histograms != 1) return error.UnsupportedFrameFeatures;
        const used_orders = U32Coder.read(
            coeff_order_mod.kOrderEnc,
            sec,
        );
        try coeff_order_mod.decodeCoeffOrders(
            allocator,
            used_orders,
            &orders,
            sec,
        );
        try dec_ans.decodeHistograms(
            allocator,
            sec,
            block_ctx_map.numACContexts(),
            &ac_code,
            &ac_context_map,
            false,
        );
    }

    // --- Reconstruction: XYB planes ---
    var xyb: [3][]f32 = undefined;
    var xyb_ok = false;
    errdefer if (xyb_ok) for (&xyb) |p| allocator.free(p);
    defer if (xyb_ok) for (&xyb) |p| allocator.free(p);
    for (&xyb) |*p| {
        p.* = try allocator.alloc(f32, xsize * ysize);
        @memset(p.*, 0);
    }
    xyb_ok = true;

    const biases = opsin_mod.kDefaultQuantBias;
    // dec_cache.h Init: pow(1/1.25, qm_scale - 2) — the reciprocal of the
    // encoder-side multiplier.
    const x_dm: f32 = std.math.pow(f32, 1.0 / 1.25, @as(f32, @floatFromInt(frame.x_qm_scale)) - 2.0);
    const b_dm: f32 = std.math.pow(f32, 1.0 / 1.25, @as(f32, @floatFromInt(frame.b_qm_scale)) - 2.0);

    for (0..fd.num_groups) |g| {
        var sec_fresh: BitReader = undefined;
        var sec_fresh_open = false;
        defer if (sec_fresh_open) sec_fresh.close() catch {};
        var sec: *BitReader = undefined;
        if (single_section) {
            sec = &sec0;
        } else {
            const s = toc_mod.acGroupIndex(
                0,
                g,
                fd.num_groups,
                fd.num_dc_groups,
            );
            const start = section_starts[s];
            sec_fresh = BitReader.init(
                sections[start .. start + section_sizes[s]],
            );
            sec_fresh_open = true;
            sec = &sec_fresh;
        }
        const rect = fd.blockGroupRect(g);
        const order_slices = [3][]const u32{
            orders[0][0..],
            orders[1][0..],
            orders[2][0..],
        };

        var reader = try dec_ans.ANSSymbolReader.create(
            allocator,
            &ac_code,
            sec,
            0,
        );
        defer reader.deinit();
        var cache = try ac_coefficient.GroupCoeffCache.init(
            allocator,
        );
        defer cache.deinit();

        var block: [3][64]i32 = undefined;
        var fblock: [64]f32 = undefined;
        var dequant_y_cache: [64]f32 = undefined;
        var pixels: [64]f32 = undefined;

        for (0..rect.ysize) |by| {
            const qdc_row = quant_dc_buckets[(rect.y0 + by) * fd.xsize_blocks + rect.x0 ..][0..rect.xsize];
            const qf_row = qf_field[(rect.y0 + by) * fd.xsize_blocks + rect.x0 ..][0..rect.xsize];
            for (0..rect.xsize) |bx| {
                const ty = (rect.y0 + by) >> 3;
                const tx = (rect.x0 + bx) >> 3;
                const cc_x = cfl.ytoXRatio(cfl.ytox_map[ty * cfl.map_w + tx]);
                const cc_b = cfl.ytoBRatio(cfl.ytob_map[ty * cfl.map_w + tx]);
                const quant = qf_row[bx];
                const scaled = q.inv_global_scale / @as(f32, @floatFromInt(quant));

                for ([3]usize{ 1, 0, 2 }) |c| {
                    @memset(&block[c], 0);
                    const row = cache.nzeros[c][by * 32 ..][0..32];
                    const top: ?[]const i32 = if (by == 0)
                        null
                    else
                        cache.nzeros[c][(by - 1) * 32 ..][0..32];
                    try ac_coefficient.decodeACVarBlock(
                        &reader,
                        sec,
                        ac_context_map.items,
                        order_slices[c],
                        row,
                        top,
                        c,
                        bx,
                        bx,
                        qdc_row,
                        &block_ctx_map,
                        &block[c],
                    );

                    // Dequant (+ bias, + CfL from the dequantized Y).
                    const dm: f32 = if (c == 0) x_dm else if (c == 2) b_dm else 1.0;
                    for (0..64) |k| {
                        fblock[k] = transform_lossy.adjustQuantBias(
                            c,
                            block[c][k],
                            &biases,
                        ) * (tables.table[c * 64 + k] * (scaled * dm));
                    }
                    if (c == 1) {
                        // Y first: no CfL.
                    } else {
                        const cc = if (c == 0) cc_x else cc_b;
                        if (cc != 0) {
                            for (0..64) |k| {
                                fblock[k] += cc * dequant_y_cache[k];
                            }
                        }
                    }
                    if (c == 1) {
                        @memcpy(&dequant_y_cache, &fblock);
                    }
                    // DC in, IDCT, clipped pixel writes.
                    fblock[0] = dc[c][(rect.y0 + by) * fd.xsize_blocks + rect.x0 + bx];
                    transform_lossy.dct8Inverse(&fblock, &pixels);
                    const px0 = (rect.x0 + bx) * 8;
                    const py0 = (rect.y0 + by) * 8;
                    var yy: usize = 0;
                    while (yy < 8 and py0 + yy < ysize) : (yy += 1) {
                        var xx: usize = 0;
                        while (xx < 8 and px0 + xx < xsize) : (xx += 1) {
                            xyb[c][(py0 + yy) * xsize + px0 + xx] = pixels[yy * 8 + xx];
                        }
                    }
                }
            }
        }
        if (!reader.checkANSFinalState()) return error.InvalidANSStream;
    }

    // --- Gaborish ---
    if (frame.loop_filter.gab) {
        var scratch: [3][]f32 = undefined;
        var scratch_ok = false;
        errdefer if (scratch_ok) for (&scratch) |p| allocator.free(p);
        for (&scratch) |*p| {
            p.* = try allocator.alloc(f32, xsize * ysize);
        }
        scratch_ok = true;
        transform_lossy.gaborishInverse(
            xyb,
            xsize,
            ysize,
            scratch,
        );
        for (&scratch) |p| allocator.free(p);
    }

    // --- Patches ---
    if (wants_patches) {
        if (!pd.hasAny()) return error.InvalidPatchDictionary;
        try pd.applyToFloatImage(
            allocator,
            xyb,
            xsize,
            ysize,
            refs,
        );
    }

    // --- XYB -> linear ---
    var out = LossyImage{
        .allocator = allocator,
        .xsize = xsize,
        .ysize = ysize,
        .planes = undefined,
    };
    errdefer for (&out.planes) |p| allocator.free(p);
    for (&out.planes) |*p| {
        p.* = try allocator.alloc(f32, xsize * ysize);
    }
    for (0..xsize * ysize) |i| {
        const lin = opsin_mod.xybToLinearRgb(
            xyb[0][i],
            xyb[1][i],
            xyb[2][i],
        );
        out.planes[0][i] = lin[0];
        out.planes[1][i] = lin[1];
        out.planes[2][i] = lin[2];
    }
    return out;
}

fn quant_mod_kQuantMax() i32 {
    return 256;
}

fn intFromFloatRounded(v: f32) i32 {
    return @intFromFloat(@round(v));
}
