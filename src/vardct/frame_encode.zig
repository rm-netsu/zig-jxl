//! VarDCT JPEG-transcode frame encoder, port of the JPEG path of libjxl
//! `enc_frame.cc` (`ComputeJPEGTranscodingData`, `EncodeGroups`,
//! `enc_modular.cc` AddVarDCTDC/AddACMetadata/AddQuantTable) with the
//! section layout of `EncodeGroups`:
//!
//!   codestream = signature, SizeHeader, ImageMetadata, CustomTransformData,
//!                FrameHeader, TOC,
//!   section 0 (DC global): DequantDC, QuantizerParams, BlockCtxMap,
//!                CfL-DC, modular global info (tree + histograms),
//!   sections 1..num_dc_groups (DC groups): extra_dc_precision,
//!                VarDCTDC modular stream, ACMetadata size + stream,
//!   section num_dc_groups+1 (AC global): DequantMatrices (the RAW JPEG
//!                tables ride a modular stream here), num_histograms,
//!                used_orders, AC histograms,
//!   remaining sections (AC groups): ANS-coded coefficient tokens.
//!
//! The output is bit-level compatible with libjxl's JPEG recompression:
//! `djxl` decodes it both to pixels and back to the original JPEG bytes.

const std = @import("std");
const bits = @import("../base/bits.zig");
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
const enc_icc = @import("../header/enc_icc.zig");
const ma_encode = @import("../modular/ma_encode.zig");
const context_predict = @import("../modular/context_predict.zig");
const enc_ans = @import("../entropy/enc_ans.zig");

const jd = @import("../jpeg/jpeg_data.zig");

const block_context = @import("block_context.zig");
const coeff_order_mod = @import("coeff_order.zig");
const ac_coefficient = @import("ac_coefficient.zig");
const quant_mod = @import("quant.zig");

const Allocator = std.mem.Allocator;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const U32Enc = fields.U32Enc;
const val = fields.val;
const Image = image_mod.Image;
const Channel = image_mod.Channel;
const Token = ma_encode.Token;
const ANSToken = enc_ans.Token;
const FrameDim = stream.FrameDim;
const ModularStreamId = stream.ModularStreamId;
const TrainStream = stream.TrainStream;
const FrameHeader = frame_mod.FrameHeader;
const YCbCrChromaSubsampling = frame_mod.YCbCrChromaSubsampling;
const ColorTransform = frame_mod.ColorTransform;
const JPEGData = jd.JPEGData;
const BlockCtxMap = block_context.BlockCtxMap;

pub const Error = error{
    UnsupportedJpegForTranscode,
    InvalidSubsampleMode,
    InvalidJpegDimensions,
    OutOfMemory,
} || anyerror;

/// Encoder-side options of the VarDCT JPEG-transcode frame. Both default
/// to the cjxl-default (kSquirrel effort) behavior of official libjxl:
/// `force_cfl_jpeg_recompression = true` and coefficient reordering
/// enabled (ComputeUsedOrders at any speed slower than kFalcon).
pub const TranscodeOptions = struct {
    /// Chroma-from-luma for 444 3-component frames (FindCorrelation +
    /// QCR subtraction). Matches libjxl `cparams.force_cfl_jpeg_recompression`.
    cfl: bool = true,
    /// Custom (learned) coefficient orders for DCT-8 blocks
    /// (ComputeCoeffOrder, transmitted as Lehmer-coded permutations).
    coeff_orders: bool = true,
    /// cparams.centerfirst (cjxl `--group_order=1`): write the TOC with
    /// the center-first AC-group permutation.
    group_order: bool = false,
};

/// libjxl `kColorFactorDist` (CfL DC color factor).
pub const color_factor_dist = U32Enc.init(
    val(84),
    val(256),
    fields.bitsOffset(8, 2),
    fields.bitsOffset(16, 258),
);

/// libjxl `kCFLFixedPointPrecision`.
pub const kCFLFixedPointPrecision: u6 = 11;

/// jxl channel c -> JPEG component index (libjxl `JpegOrder`).
pub fn jpegOrder(color_transform: ColorTransform, is_gray: bool) [3]usize {
    if (is_gray) return .{ 0, 0, 0 };
    return switch (color_transform) {
        .ycbcr => .{ 1, 0, 2 },
        else => .{ 0, 1, 2 },
    };
}

/// Detects the color transform from the JPEG markers (libjxl
/// `SetColorTransformFromJpegData`): JFIF implies YCbCr; otherwise an
/// Adobe APP14 marker or 'R','G','B' component ids imply RGB.
pub fn colorTransformFromJpeg(jpg: *const JPEGData) ColorTransform {
    const nbcomp = jpg.components.items.len;
    var is_rgb = false;
    const markers = jpg.marker_order.items;
    var has_jfif = false;
    for (markers) |m| {
        if (m == 0xE0) has_jfif = true;
    }
    if (!has_jfif) {
        var app_markers: usize = 0;
        var found_adobe = false;
        for (markers) |m| {
            if ((m & 0xF0) == 0xE0) {
                if (m == 0xEE and app_markers < jpg.app_data.items.len) {
                    const data = jpg.app_data.items[app_markers].items;
                    if (data.len == 15 and data[3] == 'A' and data[4] == 'd' and
                        data[5] == 'o' and data[6] == 'b' and data[7] == 'e')
                    {
                        is_rgb = data[14] == 0;
                        found_adobe = true;
                        break;
                    }
                }
                app_markers += 1;
            }
        }
        if (!found_adobe) {
            is_rgb = nbcomp == 3 and jpg.components.items[0].id == 'R' and
                jpg.components.items[1].id == 'G' and
                jpg.components.items[2].id == 'B';
        }
    }
    return if (!is_rgb or nbcomp == 1) .ycbcr else .none;
}

/// Port of `SetChromaSubsamplingFromJpegData`.
pub fn chromaSubsamplingFromJpeg(jpg: *const JPEGData) !YCbCrChromaSubsampling {
    var cs = YCbCrChromaSubsampling{};
    const nbcomp = jpg.components.items.len;
    var hsample: [3]u32 = undefined;
    var vsample: [3]u32 = undefined;
    if (nbcomp == 3) {
        for (0..3) |i| {
            hsample[i] = @intCast(jpg.components.items[i].h_samp_factor);
            vsample[i] = @intCast(jpg.components.items[i].v_samp_factor);
        }
    } else {
        for (0..3) |i| {
            hsample[i] = @intCast(jpg.components.items[0].h_samp_factor);
            vsample[i] = @intCast(jpg.components.items[0].v_samp_factor);
        }
    }
    try cs.setFromJpegSampling(hsample, vsample);
    return cs;
}

const Plane32 = struct {
    data: []i32 = &.{},
    w: usize = 0,
    h: usize = 0,

    fn at(self: *const Plane32, x: usize, y: usize) i32 {
        return self.data[y * self.w + x];
    }
};

const BlockPlane = struct {
    data: []i32 = &.{},
    w: usize = 0,
    h: usize = 0,

    fn block(self: *const BlockPlane, bx: usize, by: usize) []i32 {
        const off = (by * self.w + bx) * 64;
        return self.data[off .. off + 64];
    }
};

/// All derived per-frame data of the transcode (what
/// `ComputeJPEGTranscodingData` produces on the libjxl side).
pub const TranscodeData = struct {
    allocator: Allocator,
    is_gray: bool,
    color_transform: ColorTransform,
    jpeg_c_map: [3]usize,
    cs: YCbCrChromaSubsampling,
    fd: FrameDim,
    /// Transposed quant tables: qt[64*c + 8*x + y] = jpeg_dct[8*y + x]
    /// of the table used by jxl channel c.
    qt: [3 * 64]i32 = @splat(0),
    /// DC modular values per channel (idc); channel dims are subsampled.
    dc: [3]Plane32 = .{ .{}, .{}, .{} },
    /// AC coefficient blocks per channel, transposed layout, block-major.
    ac: [3]BlockPlane = .{ .{}, .{}, .{} },
    block_ctx_map: BlockCtxMap,
    /// Per-block DC threshold buckets (full grid).
    quant_dc: []u8 = &.{},
    /// Chroma-from-luma state (libjxl `shared.cmap` + `scaled_qtable`):
    /// enabled for 444 3-component frames. `scaled_qtable` carries the
    /// transposed (X-table / c-table) fixed-point ratios used by both
    /// FindCorrelation and the QCR subtraction; the int8 maps cover one
    /// 8x8-block tile each.
    use_cfl: bool = false,
    scaled_qtable: [3 * 64]i64 = @splat(0),
    cmap_ytox: []i8 = &.{},
    cmap_ytob: []i8 = &.{},
    cmap_w: usize = 0,
    cmap_h: usize = 0,

    fn deinit(self: *TranscodeData) void {
        for (&self.dc) |*p| self.allocator.free(p.data);
        for (&self.ac) |*p| self.allocator.free(p.data);
        self.allocator.free(self.quant_dc);
        self.allocator.free(self.cmap_ytox);
        self.allocator.free(self.cmap_ytob);
        self.block_ctx_map.deinit();
        self.* = undefined;
    }
};

pub fn ceilLog2Nonzero(v: usize) usize {
    std.debug.assert(v != 0);
    const l = std.math.log2_int(usize, v);
    return l + @as(usize, @intFromBool(@popCount(v) != 1));
}

/// Builds the per-frame transcode data from a parsed JPEG.
pub fn buildTranscodeData(
    allocator: Allocator,
    jpg: *const JPEGData,
    topts: TranscodeOptions,
) !TranscodeData {
    const nbcomp = jpg.components.items.len;
    if (nbcomp != 1 and nbcomp != 3) return Error.UnsupportedJpegForTranscode;
    if (jpg.width <= 0 or jpg.height <= 0 or
        jpg.width > 0xffff or jpg.height > 0xffff)
    {
        return Error.InvalidJpegDimensions;
    }
    const color_transform = colorTransformFromJpeg(jpg);
    const cs = try chromaSubsamplingFromJpeg(jpg);
    const is_gray = nbcomp == 1;
    const jpeg_c_map = jpegOrder(
        color_transform,
        is_gray,
    );

    var td = TranscodeData{
        .allocator = allocator,
        .is_gray = is_gray,
        .color_transform = color_transform,
        .jpeg_c_map = jpeg_c_map,
        .cs = cs,
        .fd = FrameDim.initVarDct(
            @intCast(jpg.width),
            @intCast(jpg.height),
            cs.maxHShift(),
            cs.maxVShift(),
        ),
        .block_ctx_map = BlockCtxMap.init(allocator),
    };
    errdefer td.deinit();

    const fd = &td.fd;
    const hshift = [3]usize{ cs.hShift(0), cs.hShift(1), cs.hShift(2) };
    const vshift = [3]usize{ cs.vShift(0), cs.vShift(1), cs.vShift(2) };

    // Quant tables (transposed; all channels of a grayscale image use the
    // single component's table).
    for (0..3) |c| {
        const jpeg_c = jpeg_c_map[c];
        const comp = &jpg.components.items[jpeg_c];
        if (comp.quant_idx >= jpg.quant.items.len)
            return Error.UnsupportedJpegForTranscode;
        const quant = &jpg.quant.items[comp.quant_idx].values;
        for (0..8) |y| {
            for (0..8) |x| {
                // JPEG XL transposes the DCT, JPEG doesn't.
                td.qt[64 * c + 8 * x + y] = quant[8 * y + x];
            }
        }
    }
    for (0..3) |c| {
        if (td.qt[64 * c] <= 0) return Error.UnsupportedJpegForTranscode;
    }
    // Ratio of two entries in a JPEG quantization table must stay within
    // the fixed-point CfL range (libjxl check). The same ratios feed the
    // CfL machinery (scaled_qtable, transposed like libjxl's).
    for (0..3) |c| {
        for (0..64) |i| {
            const num = td.qt[64 + i];
            const den = td.qt[64 * c + i];
            if (num <= 0 or den <= 0 or num >= 0x10000 or den >= 0x10000) {
                return Error.UnsupportedJpegForTranscode;
            }
            const ratio = @divTrunc(
                (@as(i64, 1) << kCFLFixedPointPrecision) * num,
                den,
            );
            if (ratio > (@as(i64, 1) << kCFLFixedPointPrecision) * 256 - 1) {
                return Error.UnsupportedJpegForTranscode;
            }
            td.scaled_qtable[64 * c + (i % 8) * 8 + (i / 8)] = ratio;
        }
    }

    // Chroma-from-luma (libjxl `force_cfl_jpeg_recompression`, default on):
    // only for 444 3-component frames (FindCorrelation + QCR subtraction).
    td.use_cfl = topts.cfl and !is_gray and cs.is444();
    if (td.use_cfl) {
        td.cmap_w = (fd.xsize_blocks + 7) >> 3;
        td.cmap_h = (fd.ysize_blocks + 7) >> 3;
        td.cmap_ytox = try allocator.alloc(i8, td.cmap_w * td.cmap_h);
        td.cmap_ytob = try allocator.alloc(i8, td.cmap_w * td.cmap_h);
        @memset(td.cmap_ytox, 0);
        @memset(td.cmap_ytob, 0);
        try findCorrelation(&td, jpg);
    }

    // DC planes and AC coefficient planes (channel dims subsampled).
    for (0..3) |c| {
        const w = fd.xsize_blocks >> @intCast(hshift[c]);
        const h = fd.ysize_blocks >> @intCast(vshift[c]);
        td.dc[c] = .{
            .data = try allocator.alloc(i32, w * h),
            .w = w,
            .h = h,
        };
        td.ac[c] = .{
            .data = try allocator.alloc(i32, w * h * 64),
            .w = w,
            .h = h,
        };
    }

    const dczero = color_transform == .ycbcr;
    // JPEG DC is from -1024 to 1023; the non-YCbCr path stores
    // idc = dc + 1024 / qt_dc (libjxl).
    const dcoff: [3]i32 = if (dczero) .{ 0, 0, 0 } else .{
        @divTrunc(@as(i32, 1024), td.qt[0]),
        @divTrunc(@as(i32, 1024), td.qt[64]),
        @divTrunc(@as(i32, 1024), td.qt[128]),
    };

    var dc_counts = [_]u32{0} ** 2048;
    var total_dc = [3]usize{ 0, 0, 0 };

    for ([3]usize{ 1, 0, 2 }) |c| {
        const jpeg_c = jpeg_c_map[c];
        const comp = &jpg.components.items[jpeg_c];
        const w_c = fd.xsize_blocks >> @intCast(hshift[c]);
        const h_c = fd.ysize_blocks >> @intCast(vshift[c]);
        if (is_gray and c != 1) {
            @memset(td.dc[c].data, 0);
            @memset(td.ac[c].data, 0);
            // Ensure no division by 0.
            total_dc[c] = 1;
            continue;
        }
        if (comp.width_in_blocks != w_c or comp.height_in_blocks != h_c or
            comp.coeffs.items.len != w_c * h_c * 64)
        {
            return Error.UnsupportedJpegForTranscode;
        }
        // The Y row (full grid; CfL only runs on 444 frames, so the Y
        // component spans exactly the full block grid).
        const y_comp = &jpg.components.items[jpeg_c_map[1]];
        const y_row = y_comp.coeffs.items;
        const map_row: ?[]const i8 = if (td.use_cfl and c != 1)
            (if (c == 0) td.cmap_ytox else td.cmap_ytob)
        else
            null;
        for (0..h_c) |by| {
            for (0..w_c) |bx| {
                const base = (by * w_c + bx) * 64;
                const idc = @as(i32, comp.coeffs.items[base]) + dcoff[c];
                td.dc[c].data[by * w_c + bx] = idc;
                if (c == 1) {
                    const bucket = @min(@max(idc + 1024, 0), 2047);
                    dc_counts[@intCast(bucket)] += 1;
                }
                total_dc[c] += 1;
                // Transposed AC coefficients: block[8*x + y] = jpeg[8*y + x];
                // chroma channels carry the CfL residual QCR = QChroma -
                // cfl_factor when the map at this tile is active (libjxl
                // ComputeJPEGTranscodingData).
                const out_block = td.ac[c].data[base .. base + 64];
                if (map_row) |map| {
                    const map_v: i32 = map[(by >> 3) * td.cmap_w + (bx >> 3)];
                    if (map_v != 0) {
                        // ColorCorrelation::RatioJPEG(v) = v * 2^11 / 84.
                        const scale: i64 = @divTrunc(@as(i64, map_v) * 2048, 84);
                        const y_base = (by * w_c + bx) * 64; // 444: Y grid == this grid
                        for (0..8) |y| {
                            for (0..8) |x| {
                                const coeffpos = y * 8 + x;
                                const y_coeff: i64 = y_row[y_base + coeffpos];
                                const q_chroma: i64 = comp.coeffs.items[base + coeffpos];
                                const coeff_scale: i64 =
                                    (scale * td.scaled_qtable[64 * c + coeffpos] + 1024) >> 11;
                                const cfl_factor: i64 = (y_coeff * coeff_scale + 1024) >> 11;
                                out_block[8 * x + y] = @intCast(q_chroma - cfl_factor);
                            }
                        }
                        continue;
                    }
                }
                for (0..8) |y| {
                    for (0..8) |x| {
                        out_block[8 * x + y] = comp.coeffs.items[base + 8 * y + x];
                    }
                }
            }
        }
    }

    // DC thresholds and the JPEG block context map.
    {
        const qt_sum: usize = @intCast(
            @max(1, @as(i64, td.qt[1]) + td.qt[2] + td.qt[3] + td.qt[4] + td.qt[5]),
        );
        var num_thresholds: i32 = @as(
            i32,
            @intCast(ceilLog2Nonzero(total_dc[1])),
        ) - @as(i32, @intCast(ceilLog2Nonzero(qt_sum))) - 7;
        num_thresholds = std.math.clamp(num_thresholds, 1, 7);
        var cumsum: usize = 0;
        var cut = total_dc[1] / @as(usize, @intCast(num_thresholds + 1));
        for (0..2048) |j| {
            cumsum += dc_counts[j];
            if (cumsum > cut) {
                try td.block_ctx_map.dc_thresholds[1].append(
                    allocator,
                    @as(i32, @intCast(j)) - 1025,
                );
                cut = total_dc[1] *
                    (td.block_ctx_map.dc_thresholds[1].items.len + 1) /
                    @as(usize, @intCast(num_thresholds + 1));
            }
        }
        td.block_ctx_map.num_dc_ctxs = td.block_ctx_map.dc_thresholds[1].items.len + 1;
        const num_dc_ctxs = td.block_ctx_map.num_dc_ctxs;
        const total = 3 * block_context.kNumOrders * num_dc_ctxs;
        try td.block_ctx_map.ctx_map.resize(allocator, total);
        @memset(td.block_ctx_map.ctx_map.items, 0);
        for (0..num_dc_ctxs) |i| {
            // luma: one context per luma DC bucket
            td.block_ctx_map.ctx_map.items[i] = @intCast(i);
            if (is_gray) {
                td.block_ctx_map.ctx_map.items[block_context.kNumOrders * num_dc_ctxs + i] = @intCast(num_dc_ctxs);
                td.block_ctx_map.ctx_map.items[2 * block_context.kNumOrders * num_dc_ctxs + i] = @intCast(num_dc_ctxs);
            } else {
                td.block_ctx_map.ctx_map.items[block_context.kNumOrders * num_dc_ctxs + i] = @intCast(num_dc_ctxs + i / 2);
                td.block_ctx_map.ctx_map.items[2 * block_context.kNumOrders * num_dc_ctxs + i] =
                    @intCast(num_dc_ctxs + (num_dc_ctxs - 1) / 2 + 1 + i / 2);
            }
        }
        var max_ctx: u8 = 0;
        for (td.block_ctx_map.ctx_map.items) |v| max_ctx = @max(max_ctx, v);
        td.block_ctx_map.num_ctxs = @as(usize, max_ctx) + 1;

        // Per-block DC threshold buckets (only luma thresholds are used).
        td.quant_dc = try allocator.alloc(
            u8,
            fd.xsize_blocks * fd.ysize_blocks,
        );
        const thresholds = td.block_ctx_map.dc_thresholds[1].items;
        const y_plane = &td.dc[1];
        for (0..fd.ysize_blocks) |by| {
            for (0..fd.xsize_blocks) |bx| {
                const y_dc = y_plane.at(bx, by);
                var bucket: u8 = 0;
                for (thresholds) |t| {
                    if (y_dc > t) bucket += 1;
                }
                td.quant_dc[by * fd.xsize_blocks + bx] = bucket;
            }
        }
    }

    return td;
}

/// libjxl `kZeroBiasDefault` (quantizer.h).
const k_zero_bias_default = [3]f32{ 0.5, 0.5, 0.5 };

/// Port of the FindCorrelation tile loop of `ComputeJPEGTranscodingData`
/// (enc_frame.cc): for every 8x8-block tile, searches the CfL map value in
/// [-127, 128] that turns the largest number of chroma AC coefficients
/// into zero residuals QCR = QChroma - cfl_factor(Y), via the
/// d_num_zeros difference array and `FindAvgIndexOfSumMaximum<256>`.
///
/// Float arithmetic mirrors the C++ f32 evaluation order exactly (the
/// zero-threshold interval [from, to] is computed per coefficient); the
/// map stays 0 when the best candidate does not beat the zero value by
/// more than one (`best_sum > offset_sum + 1`).
fn findCorrelation(td: *TranscodeData, jpg: *const JPEGData) !void {
    const k_scale: f32 = 84.0; // kDefaultColorFactor
    const k_offset: usize = 127;
    // XYB=false cmap: both base correlations are 0, so
    // (kOffset - kBase * kScale) is exactly 127.
    const offset_term: f32 = 127.0;
    const fd = &td.fd;
    const y_comp = &jpg.components.items[td.jpeg_c_map[1]];

    for ([2]usize{ 0, 2 }) |c| {
        const s_comp = &jpg.components.items[td.jpeg_c_map[c]];
        const map: []i8 = if (c == 0) td.cmap_ytox else td.cmap_ytob;
        const zero_thresh: f32 = k_scale * k_zero_bias_default[c] * 0.9999;
        const y_w: usize = @intCast(y_comp.width_in_blocks);
        const s_w: usize = @intCast(s_comp.width_in_blocks);

        for (0..td.cmap_h) |ty| {
            const y0 = ty * 8;
            const y1 = @min(fd.ysize_blocks, (ty + 1) * 8);
            for (0..td.cmap_w) |tx| {
                const x0 = tx * 8;
                const x1 = @min(fd.xsize_blocks, (tx + 1) * 8);
                var d_num_zeros: [257]i32 = @splat(0);
                var by = y0;
                while (by < y1) : (by += 1) {
                    const row_m = y_comp.coeffs.items[by * y_w * 64 ..][0 .. y_w * 64];
                    const row_s = s_comp.coeffs.items[by * s_w * 64 ..][0 .. s_w * 64];
                    var bx = x0;
                    while (bx < x1) : (bx += 1) {
                        var coeffpos: usize = 1;
                        while (coeffpos < 64) : (coeffpos += 1) {
                            const scaled_m = @as(
                                f32,
                                @floatFromInt(row_m[bx * 64 + coeffpos]),
                            ) *
                                (1.0 / @as(f32, 1 << kCFLFixedPointPrecision)) *
                                @as(
                                    f32,
                                    @floatFromInt(
                                        td.scaled_qtable[64 * c + coeffpos],
                                    ),
                                );
                            const scaled_s = k_scale *
                                @as(
                                    f32,
                                    @floatFromInt(row_s[bx * 64 + coeffpos]),
                                ) + offset_term * scaled_m;
                            if (@abs(scaled_m) > 1e-8) {
                                var from: f32 = undefined;
                                var to: f32 = undefined;
                                if (scaled_m > 0) {
                                    from = (scaled_s - zero_thresh) / scaled_m;
                                    to = (scaled_s + zero_thresh) / scaled_m;
                                } else {
                                    from = (scaled_s + zero_thresh) / scaled_m;
                                    to = (scaled_s - zero_thresh) / scaled_m;
                                }
                                if (from < 0.0) from = 0.0;
                                if (to > 255.0) to = 255.0;
                                // Instead of clamping both values, just
                                // check that the range is sane.
                                if (from <= to) {
                                    d_num_zeros[@intFromFloat(@ceil(from))] += 1;
                                    d_num_zeros[@intFromFloat(@floor(to + 1.0))] -= 1;
                                }
                            }
                        }
                    }
                }
                // FindAvgIndexOfSumMaximum<256>.
                var maxval: i32 = 0;
                var run_val: i32 = 0;
                var maxidx_begin: usize = 0;
                var maxidx_end: usize = 0;
                for (0..256) |i| {
                    run_val += d_num_zeros[i];
                    if (run_val > maxval) {
                        maxval = run_val;
                        maxidx_begin = i;
                    }
                    if (run_val == maxval) {
                        maxidx_end = i;
                    }
                }
                const best: usize = (maxidx_begin + maxidx_end + 1) >> 1;
                const best_sum: i32 = maxval;
                var offset_sum: i32 = 0;
                for (0..k_offset + 1) |i| {
                    offset_sum += d_num_zeros[i];
                }
                map[ty * td.cmap_w + tx] = 0;
                if (best_sum > offset_sum + 1) {
                    // int8 wraparound for best == 255, matching the C++
                    // int8_t assignment.
                    const v: i32 = @as(i32, @intCast(best)) -
                        @as(i32, @intCast(k_offset));
                    map[ty * td.cmap_w + tx] = @truncate(v);
                }
            }
        }
    }
}

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

/// Encodes the complete VarDCT transcode codestream for `jpg`. When
/// `icc` is non-empty it is embedded as the real ICC profile of the image
/// (libjxl `SetColorEncodingFromJpegData` + `WriteICC`): the headers then
/// carry `want_icc` and the compressed profile instead of the sRGB enum,
/// and the `jbrd` APP2/ICC marker payloads are reconstructed from it on
/// decode.
pub fn encodeTranscodeFrame(
    allocator: Allocator,
    jpg: *const JPEGData,
    icc: []const u8,
    writer: *BitWriter,
) Error!void {
    return encodeTranscodeFrameOptions(
        allocator,
        jpg,
        icc,
        writer,
        .{},
    );
}

/// `encodeTranscodeFrame` with explicit encoder options (chroma-from-luma
/// and custom coefficient orders, both on by default to match the cjxl
/// default-effort output shape).
pub fn encodeTranscodeFrameOptions(
    allocator: Allocator,
    jpg: *const JPEGData,
    icc: []const u8,
    writer: *BitWriter,
    topts: TranscodeOptions,
) Error!void {
    var td = try buildTranscodeData(
        allocator,
        jpg,
        topts,
    );
    defer td.deinit();
    const fd = &td.fd;
    const cs = &td.cs;
    const hshift = [3]usize{ cs.hShift(0), cs.hShift(1), cs.hShift(2) };
    const vshift = [3]usize{ cs.vShift(0), cs.vShift(1), cs.vShift(2) };

    // --- Modular streams: VarDCTDC (per DC group), ACMetadata (per DC
    // group), RAW quant table. ---
    var entries: std.ArrayList(StreamEntry) = .empty;
    defer {
        for (entries.items) |*e| e.image.deinit();
        entries.deinit(allocator);
    }

    // VarDCTDC images: 3 channels [Y, X, B] (channel c is stored at index
    // c<2 ? c^1 : c), dims from the DC group rect subsampled per channel.
    for (0..fd.num_dc_groups) |g| {
        const r = fd.dcGroupRect(g);
        var img = Image.create(allocator);
        errdefer img.deinit();
        for ([3]usize{ 1, 0, 2 }) |c| {
            const w = r.xsize >> @intCast(hshift[c]);
            const h = r.ysize >> @intCast(vshift[c]);
            var ch = try Channel.create(
                allocator,
                w,
                h,
                0,
                0,
            );
            for (0..h) |y| {
                for (0..w) |x| {
                    ch.row(y)[x] = td.dc[c].at(
                        (r.x0 >> @intCast(hshift[c])) + x,
                        (r.y0 >> @intCast(vshift[c])) + y,
                    );
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

    // ACMetadata images: [ytox, ytob, acs+qf, epf]; the CfL maps carry
    // the FindCorrelation results for 444 3-component frames (zero
    // otherwise), the remaining channels are JPEG constants.
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
        // Channels 0/1: ytox/ytob maps (hshift=vshift=3 meta channels),
        // copied from the global cmap over the group's tile rect
        // (libjxl ConvertPlaneAndClamp of cmap.ytox_map/ytob_map).
        for (0..2) |mi| {
            const src = if (mi == 0) td.cmap_ytox else td.cmap_ytob;
            const ch = try Channel.create(
                allocator,
                cr_w,
                cr_h,
                3,
                3,
            );
            for (0..cr_h) |my| {
                for (0..cr_w) |mx| {
                    const ty = (r.y0 >> 3) + my;
                    const tx = (r.x0 >> 3) + mx;
                    ch.row(my)[mx] = if (src.len > 0) src[ty * td.cmap_w + tx] else 0;
                }
            }
            try img.channels.append(allocator, ch);
        }
        // Channel 2: acs (row 0) + qf-1 (row 1); count = all blocks.
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
            try img.channels.append(allocator, ch);
        }
        // Channel 3: epf sharpness per block.
        {
            const ch = try Channel.create(
                allocator,
                r.xsize,
                r.ysize,
                0,
                0,
            );
            @memset(ch.plane.data, 0);
            try img.channels.append(allocator, ch);
        }
        ac_metadata_size[g] = r.xsize * r.ysize;
        const sid = ModularStreamId.acMetadata(g).id(fd);
        try entries.append(
            allocator,
            .{ .image = img, .sid = sid },
        );
    }

    // RAW quant table stream (table 0 = DCT).
    {
        var raw_table = quant_mod.RawQuantTable{};
        for (0..3 * 64) |i| {
            raw_table.values[i] = td.qt[i];
        }
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

    // --- Train the global tree over all streams. ---
    var train_streams: std.ArrayList(TrainStream) = .empty;
    defer train_streams.deinit(allocator);
    for (entries.items) |*e| {
        try train_streams.append(
            allocator,
            .{ .image = &e.image, .group_id = e.sid },
        );
    }
    var opts = options.ModularOptions{};
    // Few streams: drop the group-id property from the heuristics.
    opts.splitting_heuristics_properties = &.{ 0, 15, 9, 10, 11, 12, 13 };
    opts.max_property_values = 96;
    // VarDCT streams are never split into groups.
    opts.max_chan_size = 0xFFFFFF;
    var tree = try stream.trainTree(
        allocator,
        train_streams.items,
        &opts,
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

    // --- Tokenize each stream. ---
    const num_streams = ModularStreamId.num(fd, 1);
    var tokens: std.ArrayList(std.ArrayList(Token)) = .empty;
    defer {
        for (tokens.items) |*list| list.deinit(allocator);
        tokens.deinit(allocator);
    }
    try tokens.resize(allocator, num_streams);
    for (tokens.items) |*list| list.* = .empty;
    // Per-stream LZ77 distance multipliers, matching what the decoder feeds
    // ANSSymbolReader.create for each stream (max decodable channel width);
    // without them the RLE distance symbols resolve to the special
    // "one row up" distance instead of 1 on the decoder side.
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
            opts.max_chan_size,
            tree.items,
            &wp_header,
            &tokens.items[e.sid],
        );
        image_widths.items[e.sid] = stream.maxStreamChannelWidth(
            &e.image,
            opts.max_chan_size,
        );
    }

    // --- AC tokens per group. ---
    var ac_tokens: std.ArrayList(std.ArrayList(Token)) = .empty;
    defer {
        for (ac_tokens.items) |*list| list.deinit(allocator);
        ac_tokens.deinit(allocator);
    }
    try ac_tokens.resize(allocator, fd.num_groups);
    for (ac_tokens.items) |*list| list.* = .empty;

    // Custom coefficient orders (libjxl ComputeUsedOrders/ComputeCoeffOrder
    // at the cjxl default effort): learned from the (post-CfL) coefficient
    // planes and transmitted as Lehmer-coded permutations in the AC global
    // section; cleared back to natural when the heuristic changes nothing.
    var orders: [3][64]u32 = .{
        coeff_order_mod.naturalOrder8x8(),
        coeff_order_mod.naturalOrder8x8(),
        coeff_order_mod.naturalOrder8x8(),
    };
    var used_orders: u32 = 0;
    if (topts.coeff_orders) {
        const res = try coeff_order_mod.computeCoeffOrder(
            allocator,
            fd,
            hshift,
            vshift,
            .{
                .data = .{
                    td.ac[0].data,
                    td.ac[1].data,
                    td.ac[2].data,
                },
                .widths = .{ td.ac[0].w, td.ac[1].w, td.ac[2].w },
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
        var cache = try ac_coefficient.GroupCoeffCache.init(allocator);
        defer cache.deinit();
        const coeffs = ac_coefficient.GroupCoefficients{
            .planes = .{
                td.ac[0].data,
                td.ac[1].data,
                td.ac[2].data,
            },
            .widths = .{ td.ac[0].w, td.ac[1].w, td.ac[2].w },
        };
        const qdc = ac_coefficient.QdcImage{
            .data = td.quant_dc,
            .stride = fd.xsize_blocks,
        };
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
                hshift,
                vshift,
                &cache,
                &qdc,
                &td.block_ctx_map,
                &ac_tokens.items[g],
            );
        }
    }

    // --- Headers ---
    writer.write(8, 0xFF);
    writer.write(8, 0x0A);
    var sh: size_mod.SizeHeader = .{};
    try sh.set(@intCast(jpg.width), @intCast(jpg.height));
    try sh.write(writer);
    var metadata = metadata_mod.ImageMetadata.init(allocator);
    defer metadata.deinit();
    metadata.xyb_encoded = false;
    metadata.bit_depth.bits_per_sample = 8;
    metadata.modular_16_bit_buffer_sufficient = true;
    if (icc.len > 0) {
        // The color space comes from the profile's data colour space field
        // ("RGB " / "GRAY" at offset 16), like libjxl's SetFieldsFromICC;
        // tiny profiles fall back to the component count.
        const profile_gray = icc.len >= 20 and std.mem.eql(
            u8,
            icc[16..20],
            "GRAY",
        );
        const profile_rgb = icc.len >= 20 and std.mem.eql(
            u8,
            icc[16..20],
            "RGB ",
        );
        const gray = if (profile_gray)
            true
        else if (profile_rgb)
            false
        else
            td.is_gray;
        metadata.color_encoding = color_mod.ColorEncoding.wantIcc(gray);
    } else {
        metadata.color_encoding = if (td.is_gray)
            color_mod.ColorEncoding.graySrgb()
        else
            color_mod.ColorEncoding.srgb();
    }
    try metadata.write(writer);
    const ctd = metadata_mod.CustomTransformData{};
    try ctd.write(writer);
    if (metadata.color_encoding.want_icc) {
        try enc_icc.writeICC(
            allocator,
            icc,
            writer,
        );
    }
    writer.zeroPadToByte();

    var frame = FrameHeader.init(allocator);
    defer frame.deinit();
    frame.encoding = .var_dct;
    frame.frame_type = .regular;
    frame.color_transform = td.color_transform;
    frame.chroma_subsampling = td.cs;
    frame.flags = frame_mod.Flags.kSkipAdaptiveDCSmoothing;
    frame.is_last = true;
    frame.upsampling = 1;
    frame.loop_filter = .{ .gab = false, .epf_iters = 0 };
    try frame.write(&metadata, writer);

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
        // cjxl default (SpeedTier::kSquirrel) keeps the HistogramParams
        // default lz77_method = kRLE for AC/DC histograms.
        .lz77_method = .kRLE,
        .ans_histogram_strategy = .kApproximate,
    };

    // Global modular entropy code over all stream tokens (built while
    // writing the DC global section).
    var stream_code = enc_ans.EntropyEncodingData.init(
        allocator,
    );
    defer stream_code.deinit();

    // Per-stream ANS token lists, exactly the storage passed to
    // buildHistograms below (which may replace them in place with the
    // LZ77-processed lists — libjxl `tokens = std::move(tokens_lz77)`).
    // Every later writeTokens for these streams must use these lists.
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

    // DC global section.
    {
        const sec = &sections.items[0];
        // DequantMatrices DC (custom values).
        {
            var ddc = quant_mod.DequantDC{};
            for (0..3) |c| {
                ddc.dc_quant[c] = jpegDCQuant(&td, c);
            }
            try ddc.write(sec);
        }
        // QuantizerParams: global scale 1.0, quant_dc 1 (identity).
        const qp = quant_mod.QuantizerParams{
            .global_scale = 65536,
            .quant_dc = 1,
        };
        try qp.write(sec);
        try td.block_ctx_map.write(sec);
        // CfL DC: base correlations zero (JPEG-compatible), non-default.
        try writeCflDC(sec);
        // Modular global info: has_tree + tree histograms + tree tokens +
        // stream histograms.
        sec.write(1, 1); // has_tree
        var tree_token_streams: [1]std.ArrayList(ANSToken) = .{.empty};
        defer tree_token_streams[0].deinit(allocator);
        for (tree_tokens.items) |t| {
            try tree_token_streams[0].append(allocator, .{
                .is_lz77_length = t.is_lz77_length,
                .context = t.context,
                .value = t.value,
            });
        }
        var tree_code = enc_ans.EntropyEncodingData.init(allocator);
        defer tree_code.deinit();
        // NOTE: buildHistograms follows libjxl's `tokens =
        // std::move(tokens_lz77)` semantics — when LZ77 is adopted the input
        // lists are replaced in place, so the tokens written afterwards must
        // come from tree_token_streams[0], not from a copy.
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
            ac_metadata_size[g],
        );
    }

    // AC global section.
    {
        const sec = if (single_section)
            &sections.items[0]
        else
            &sections.items[1 + fd.num_dc_groups];
        try writeDequantMatrices(
            allocator,
            sec,
            fd,
            entries.items,
            all_tokens.items,
            &stream_code,
        );
        const num_histo_bits = ceilLog2Nonzero(fd.num_groups);
        if (num_histo_bits != 0) {
            sec.write(@intCast(num_histo_bits), 0); // num_histograms - 1
        }
        // used_orders: the learned DCT-8 permutation bit (0 = natural
        // zig-zag only), followed by the Lehmer-coded per-channel orders.
        try U32Coder.write(
            coeff_order_mod.kOrderEnc,
            used_orders,
            sec,
        );
        try coeff_order_mod.encodeCoeffOrders(
            allocator,
            used_orders,
            &orders,
            sec,
        );
        // AC histograms over all groups' tokens.
        var ac_tokens_ans: std.ArrayList(std.ArrayList(ANSToken)) = .empty;
        defer {
            for (ac_tokens_ans.items) |*l| l.deinit(allocator);
            ac_tokens_ans.deinit(allocator);
        }
        for (ac_tokens.items) |*list| {
            const conv = try toAnsTokens(allocator, list.items);
            try ac_tokens_ans.append(allocator, conv);
        }
        var ac_code = enc_ans.EntropyEncodingData.init(
            allocator,
        );
        defer ac_code.deinit();
        _ = try stream.buildHistograms(
            allocator,
            &histogram_params,
            td.block_ctx_map.numACContexts(),
            ac_tokens_ans.items,
            &ac_code,
            sec,
        );

        // AC group sections.
        for (0..fd.num_groups) |g| {
            const acw = if (single_section)
                sec
            else
                &sections.items[
                    toc_mod.acGroupIndex(
                        0,
                        g,
                        fd.num_groups,
                        fd.num_dc_groups,
                    )
                ];
            // num_histograms == 1: no histogram selector bits.
            var group_tokens = try toAnsTokens(
                allocator,
                ac_tokens.items[g].items,
            );
            defer group_tokens.deinit(allocator);
            _ = try stream.writeTokens(
                allocator,
                group_tokens.items,
                &ac_code,
                0,
                acw,
            );
        }
    }

    // Zero-pad every section.
    for (sections.items) |*sw| {
        sw.zeroPadToByte();
    }

    // --- TOC + section bytes ---
    // `--group-order` (cjxl `--group_order=1`, libjxl `centerfirst`): the
    // center-first section permutation; sections are emitted in physical
    // order (libjxl PermuteGroups moves the writers the same way).
    var permutation: ?[]u32 = null;
    defer if (permutation) |p| allocator.free(p);
    if (topts.group_order) {
        permutation = try toc_mod.permuteGroups(
            allocator,
            @intCast(jpg.width),
            @intCast(jpg.height),
            fd.group_dim,
            fd.xsize_groups,
            fd.num_groups,
            fd.num_dc_groups,
            1,
        );
    }
    var sizes = try allocator.alloc(u32, num_toc_entries);
    defer allocator.free(sizes);
    if (permutation) |p| {
        // physical[permutation[i]] = logical[i]; sizes in physical order.
        var physical = try allocator.alloc(
            BitWriter,
            num_toc_entries,
        );
        defer allocator.free(physical);
        for (0..num_toc_entries) |i| {
            physical[p[i]] = sections.items[i];
            sections.items[i] = BitWriter.init(allocator);
        }
        for (physical, 0..) |*sw, i| {
            sizes[i] = @intCast(sw.bitsWritten() / 8);
        }
        try toc_mod.writeTocWithPermutation(
            allocator,
            p,
            sizes,
            writer,
        );
        for (physical) |*sw| {
            const bytes = try sw.takeBytes();
            defer allocator.free(bytes);
            for (bytes) |b| writer.write(8, b);
        }
    } else {
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

/// libjxl `DequantMatricesSetCustomDC` semantics: dcquantization[c] =
/// 255*8/quant_dc and the serialized value is its reciprocal times 128.
fn jpegDCQuant(td: *const TranscodeData, c: usize) f32 {
    const quant_dc: f32 = @floatFromInt(td.qt[64 * c]);
    return quant_dc / (8.0 * 255.0);
}

/// CfL DC parameters as written by the JPEG transcode path: all-zero base
/// correlations with the default color factor (non-default serialization
/// because libjxl's default base B ratio differs).
fn writeCflDC(writer: *BitWriter) !void {
    writer.write(1, 0); // not all-default
    try U32Coder.write(color_factor_dist, 84, writer);
    try fields_ext.writeF16(0.0, writer);
    try fields_ext.writeF16(0.0, writer);
    writer.write(8, 128); // ytox_dc = 0 + 128
    writer.write(8, 128); // ytob_dc = 0 + 128
}

/// Writes one DC group section: extra_dc_precision, the VarDCTDC modular
/// stream, the ACMetadata size and the ACMetadata modular stream.
/// `tokens` are the per-stream ANS token lists exactly as passed to
/// buildHistograms (they may have been replaced in place by the
/// LZ77-processed lists — libjxl `tokens = std::move(tokens_lz77)`).
fn writeDCGroupSection(
    allocator: Allocator,
    writer: *BitWriter,
    fd: *const FrameDim,
    entries: []const StreamEntry,
    tokens: []const std.ArrayList(ANSToken),
    code: *const enc_ans.EntropyEncodingData,
    g: usize,
    ac_metadata_size: usize,
) !void {
    writer.write(2, 0); // extra_dc_precision

    // VarDCTDC stream: find the entry with the matching stream id.
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
        writer.write(@intCast(nb_bits), ac_metadata_size - 1);
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

/// Writes a non-empty modular stream: GroupHeader (use_global_tree) plus
/// the ANS-coded tokens with the global code.
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

/// Writes `DequantMatrices` (AC global): non-default bit, 17 table entries;
/// table 0 (DCT) is RAW and carries the quant table modular stream inline.
fn writeDequantMatrices(
    allocator: Allocator,
    writer: *BitWriter,
    fd: *const FrameDim,
    entries: []const StreamEntry,
    tokens: []const std.ArrayList(ANSToken),
    code: *const enc_ans.EntropyEncodingData,
) !void {
    writer.write(1, 0); // not all default
    const qt_sid = ModularStreamId.quantTable(0).id(fd);
    for (0..quant_mod.kNumQuantTables) |i| {
        if (i == 0) {
            writer.write(3, @intFromEnum(quant_mod.QuantMode.raw));
            // F16 denominator: 1 / (8 * 255).
            try fields_ext.writeF16(1.0 / (8.0 * 255.0), writer);
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
            // Library table 0 (predefined index needs 0 bits).
            writer.write(
                3,
                @intFromEnum(quant_mod.QuantMode.library),
            );
        }
    }
}

test "buildTranscodeData: cfl maps engage for 444 and stay zero otherwise" {
    const alloc = std.testing.allocator;
    const jd_mod = @import("../jpeg/jpeg_data.zig");
    const reader_mod = @import("../jpeg/reader.zig");

    // 444 3-component: FindCorrelation must produce at least one active
    // map tile (this file has strong chroma correlation).
    {
        const data = @embedFile(
            "../jpeg/testdata/baseline_q98_444.jpg",
        );
        var jpg = jd_mod.JPEGData{};
        defer jpg.deinit(alloc);
        try reader_mod.readJpeg(
            data,
            .read_all,
            &jpg,
            alloc,
        );
        var td = try buildTranscodeData(
            alloc,
            &jpg,
            .{},
        );
        defer td.deinit();
        try std.testing.expect(td.use_cfl);
        var active: usize = 0;
        for (td.cmap_ytox) |v| {
            if (v != 0) active += 1;
        }
        for (td.cmap_ytob) |v| {
            if (v != 0) active += 1;
        }
        try std.testing.expect(active > 0);
        // Map values are valid int8 CfL factors.
        for (td.cmap_ytox) |v|
            try std.testing.expect(v >= -128 and v <= 127);
        // With CfL disabled the maps stay empty.
        var td_off = try buildTranscodeData(
            alloc,
            &jpg,
            .{ .cfl = false },
        );
        defer td_off.deinit();
        try std.testing.expect(!td_off.use_cfl);
        try std.testing.expectEqual(
            @as(usize, 0),
            td_off.cmap_ytox.len,
        );
    }

    // Subsampled (420): CfL inactive, maps empty, chroma planes untouched.
    {
        const data = @embedFile(
            "../jpeg/testdata/baseline_420.jpg",
        );
        var jpg = jd_mod.JPEGData{};
        defer jpg.deinit(alloc);
        try reader_mod.readJpeg(
            data,
            .read_all,
            &jpg,
            alloc,
        );
        var td = try buildTranscodeData(
            alloc,
            &jpg,
            .{},
        );
        defer td.deinit();
        try std.testing.expect(!td.use_cfl);
        try std.testing.expectEqual(
            @as(usize, 0),
            td.cmap_ytox.len,
        );
    }
}

test "buildTranscodeData: cfl residual roundtrip through the decoder math" {
    // The encoder's QCR subtraction and the decoder's applyCfl must be
    // exact inverses: reconstructing QChroma = QCR + cfl_factor from the
    // stored transposed block reproduces the original JPEG coefficients.
    const alloc = std.testing.allocator;
    const jd_mod = @import("../jpeg/jpeg_data.zig");
    const reader_mod = @import("../jpeg/reader.zig");
    const data = @embedFile(
        "../jpeg/testdata/baseline_q98_444.jpg",
    );
    var jpg = jd_mod.JPEGData{};
    defer jpg.deinit(alloc);
    try reader_mod.readJpeg(
        data,
        .read_all,
        &jpg,
        alloc,
    );
    var td = try buildTranscodeData(
        alloc,
        &jpg,
        .{},
    );
    defer td.deinit();
    try std.testing.expect(td.use_cfl);

    for ([2]usize{ 0, 2 }) |c| {
        const comp = &jpg.components.items[td.jpeg_c_map[c]];
        const y_comp = &jpg.components.items[td.jpeg_c_map[1]];
        const w_c: usize = @intCast(comp.width_in_blocks);
        for (0..td.ac[c].h) |by| {
            for (0..td.ac[c].w) |bx| {
                const map_v: i32 = cflMapOf(&td, c)[by / 8 * td.cmap_w + bx / 8];
                const block = td.ac[c].block(bx, by);
                const y_base = (by * w_c + bx) * 64;
                for (0..8) |y| {
                    for (0..8) |x| {
                        const coeffpos = y * 8 + x;
                        const orig = comp.coeffs.items[y_base + coeffpos];
                        const qcr = block[8 * x + y];
                        if (map_v == 0) {
                            // No CfL at this tile: plain transpose.
                            try std.testing.expectEqual(
                                orig,
                                qcr,
                            );
                        } else {
                            const scale: i64 = @divTrunc(
                                @as(i64, map_v) * 2048,
                                84,
                            );
                            const y_coeff: i64 = y_comp.coeffs.items[y_base + coeffpos];
                            const coeff_scale: i64 = (scale * td.scaled_qtable[64 * c + coeffpos] + 1024) >> 11;
                            const cfl_factor: i64 = (y_coeff * coeff_scale + 1024) >> 11;
                            try std.testing.expectEqual(
                                orig,
                                qcr + @as(i32, @intCast(cfl_factor)),
                            );
                        }
                    }
                }
            }
        }
    }
}

/// Picks the CfL map of channel c (ytox for X, ytob for B).
fn cflMapOf(td: *const TranscodeData, c: usize) []const i8 {
    return if (c == 0) td.cmap_ytox else td.cmap_ytob;
}
