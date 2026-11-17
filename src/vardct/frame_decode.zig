//! VarDCT JPEG-transcode frame decoder, port of the JPEG path of libjxl
//! `dec_frame.cc` (`ProcessDCGlobal`, `ProcessDCGroup`, `ProcessACGlobal`,
//! `ProcessACGroup`), `dec_modular.cc` (`DecodeVarDCTDC`, `DecodeAcMetadata`,
//! `DecodeQuantTable`) and `dec_group.cc` (`DecodeGroupImpl` /
//! `DecodeACVarBlock`). Reconstructs the JPEG's DCT coefficients from the
//! codestream and fills a JPEGData (already carrying the `jbrd` metadata)
//! with dimensions, sampling factors, quant tables and coefficients.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const fields_ext = @import("../header/fields_ext.zig");

const size_mod = @import("../header/size.zig");
const metadata_mod = @import("../header/metadata.zig");
const frame_mod = @import("../header/frame.zig");
const toc_mod = @import("../header/toc.zig");
const icc_mod = @import("../header/icc.zig");

const stream = @import("../modular/stream.zig");
const image_mod = @import("../modular/image.zig");
const dec_ans = @import("../entropy/dec_ans.zig");
const ma = @import("../modular/ma.zig");
const transforms = @import("../modular/transforms.zig");

const jd = @import("../jpeg/jpeg_data.zig");

const block_context = @import("block_context.zig");
const coeff_order_mod = @import("coeff_order.zig");
const ac_coefficient = @import("ac_coefficient.zig");
const quant_mod = @import("quant.zig");
const frame_encode = @import("frame_encode.zig");

const Allocator = std.mem.Allocator;
const BitReader = bit_io.BitReader;
const U32Coder = fields.U32Coder;
const Image = image_mod.Image;
const Channel = image_mod.Channel;
const FrameDim = stream.FrameDim;
const ModularStreamId = stream.ModularStreamId;
const EntropyCode = stream.EntropyCode;
const FrameHeader = frame_mod.FrameHeader;
const ColorTransform = frame_mod.ColorTransform;
const JPEGData = jd.JPEGData;

pub const Error = error{
    UnsupportedVarDct,
    UnsupportedFrameFeatures,
    UnsupportedColorTransform,
    NotAJpegTranscodeFrame,
    NotEnoughBytes,
    OutOfMemory,
} || anyerror;

/// CfL DC parameters (read side); JPEG-compatible requires all-zero base
/// correlations and the default color factor.
const CflDC = struct {
    color_factor: u32 = 84,
    base_x: f32 = 0.0,
    base_b: f32 = 1.0,
    ytox_dc: i32 = 0,
    ytob_dc: i32 = 0,

    fn read(self: *CflDC, br: *BitReader) !void {
        self.* = .{};
        if (br.readBits(1) != 0) {
            // All-default: base_b = kYToBRatio (1.0) - not JPEG compatible.
            self.base_b = 1.0;
            return;
        }
        self.color_factor = U32Coder.read(
            frame_encode.color_factor_dist,
            br,
        );
        self.base_x = try fields_ext.readF16(br);
        if (@abs(self.base_x) > 4.0) return error.UnsupportedColorTransform;
        self.base_b = try fields_ext.readF16(br);
        if (@abs(self.base_b) > 4.0) return error.UnsupportedColorTransform;
        self.ytox_dc = @as(i32, @intCast(br.readBits(8))) + std.math.minInt(i8);
        self.ytob_dc = @as(i32, @intCast(br.readBits(8))) + std.math.minInt(i8);
    }

    fn isJpegCompatible(self: *const CflDC) bool {
        return self.base_x == 0.0 and self.base_b == 0.0 and
            self.ytox_dc == 0 and self.ytob_dc == 0 and self.color_factor == 84;
    }
};

/// Decoded frame state.
const FrameState = struct {
    fn naturalOrders() [64]u32 {
        return coeff_order_mod.naturalOrder8x8();
    }

    /// Chroma-from-luma applies to 3-component 444 frames only
    /// (libjxl: `Is444() && components == 3`).
    fn is444Cfl(self: *const FrameState) bool {
        return !self.is_gray and self.cs.is444();
    }

    /// True when either chroma map at tile (ty, tx) is active.
    fn cmapHasCfl(self: *const FrameState, ty: usize, tx: usize) bool {
        return self.cmap_ytox[ty * self.cmap_w + tx] != 0 or
            self.cmap_ytob[ty * self.cmap_w + tx] != 0;
    }

    allocator: Allocator,
    fd: FrameDim,
    cs: frame_mod.YCbCrChromaSubsampling,
    color_transform: ColorTransform,
    is_gray: bool,
    jpeg_c_map: [3]usize,
    /// DC values per channel (subsampled dims).
    dc: [3][]i32 = .{ &.{}, &.{}, &.{} },
    dc_w: [3]usize = .{ 0, 0, 0 },
    dc_h: [3]usize = .{ 0, 0, 0 },
    /// Per-block DC threshold buckets (full grid).
    quant_dc: []u8 = &.{},
    /// RAW quant table values (image-store layout: values[64c + 8y + x]).
    qtable: [3 * 64]i32 = @splat(0),
    qtable_den: f32 = 0,
    /// Global modular entropy code (tree + code + context map).
    global: EntropyCode = undefined,
    global_initialized: bool = false,
    block_ctx_map: block_context.BlockCtxMap,
    /// AC entropy code + context map.
    ac_code: dec_ans.ANSCode = .{},
    ac_context_map: std.ArrayList(u8) = .empty,
    ac_initialized: bool = false,
    num_histograms: usize = 1,
    /// AC coefficient scan order per channel (natural by default; custom
    /// when the codestream carries used_orders).
    orders: [3][64]u32 = .{ naturalOrders(), naturalOrders(), naturalOrders() },
    /// Chroma-from-luma AC color maps (YToX for c=0, YToB for c=2),
    /// full-image tiles of 8x8 blocks (cjxl emits them for 444 JPEGs
    /// with `--jpeg_reconstruction_cfl`, default on).
    cmap_ytox: []i32 = &.{},
    cmap_ytob: []i32 = &.{},
    cmap_w: usize = 0,
    cmap_h: usize = 0,
    /// CfL fixed-point ratio table (dec_group.cc `scaled_qtable`):
    /// scaled_qtable[64c + 8y + x] = (1<<11) * qY[x,y] / qC[x,y], in JPEG
    /// block order for direct use on the transposed-back blocks.
    scaled_qtable: [3 * 64]i64 = @splat(0),

    fn deinit(self: *FrameState) void {
        for (&self.dc) |p| self.allocator.free(p);
        self.allocator.free(self.quant_dc);
        self.allocator.free(self.cmap_ytox);
        self.allocator.free(self.cmap_ytob);
        self.cmap_ytox = &.{};
        self.cmap_ytob = &.{};
        if (self.global_initialized) {
            self.global.deinit();
            self.global_initialized = false;
        }
        self.block_ctx_map.deinit();
        if (self.ac_initialized) {
            self.ac_code.deinit(self.allocator);
            self.ac_context_map.deinit(self.allocator);
            self.ac_initialized = false;
        }
        self.* = undefined;
    }
};

/// Decodes the complete VarDCT transcode codestream into `jpg`
/// (whose non-pixel fields come from the `jbrd` box). When the headers
/// carry an ICC profile (`want_icc`) it is decoded and, when `icc_out` is
/// set, returned as an allocator-owned copy for the caller to fill the
/// ICC-typed APP2 marker payloads (libjxl `SetJPEGDataFromICC`).
pub fn decodeTranscodeFrame(
    allocator: Allocator,
    codestream: []const u8,
    jpg: *JPEGData,
    icc_out: ?*?[]u8,
) Error!void {
    var br = BitReader.init(codestream);
    if (br.readFixedBits(8) != 0xFF or br.readFixedBits(8) != 0x0A) {
        return error.NotAJpegTranscodeFrame;
    }

    var sh: size_mod.SizeHeader = .{};
    try sh.read(&br);
    var metadata = metadata_mod.ImageMetadata.init(allocator);
    defer metadata.deinit();
    try metadata.read(&br);
    var ctd = metadata_mod.CustomTransformData{};
    try ctd.read(&br, metadata.xyb_encoded);
    if (metadata.color_encoding.want_icc) {
        const icc_bytes = icc_mod.readICC(
            allocator,
            &br,
        ) catch |e| switch (e) {
            error.OutOfMemory => return error.OutOfMemory,
            else => return error.NotAJpegTranscodeFrame,
        };
        if (icc_out) |out| {
            out.* = icc_bytes;
        } else {
            allocator.free(icc_bytes);
        }
    }
    try br.jumpToByteBoundary();

    var frame = FrameHeader.init(allocator);
    defer frame.deinit();
    try frame.read(&metadata, &br);

    if (frame.encoding != .var_dct) return error.UnsupportedVarDct;
    if (frame.frame_type != .regular) return error.UnsupportedFrameFeatures;
    if (metadata.xyb_encoded) return error.UnsupportedColorTransform;
    if (frame.color_transform == .xyb) return error.UnsupportedColorTransform;
    if (frame.upsampling != 1) return error.UnsupportedFrameFeatures;
    if (!frame.is_last) return error.UnsupportedFrameFeatures;
    if (frame.custom_size_or_origin) return error.UnsupportedFrameFeatures;
    if (frame.passes.num_passes != 1) return error.UnsupportedFrameFeatures;
    if ((frame.flags & frame_mod.Flags.kSkipAdaptiveDCSmoothing) == 0) {
        // Adaptive DC smoothing would destroy the coefficient roundtrip.
        return error.NotAJpegTranscodeFrame;
    }
    if ((frame.flags & frame_mod.Flags.kUseDcFrame) != 0)
        return error.UnsupportedFrameFeatures;
    // Patches in a VarDCT frame would land in the DC global section before
    // the JPEG-recompression fields; the transcode path does not read them.
    if ((frame.flags & frame_mod.Flags.kPatches) != 0)
        return error.UnsupportedFrameFeatures;

    const nbcomp = jpg.components.items.len;
    if (nbcomp != 1 and nbcomp != 3) return error.NotAJpegTranscodeFrame;
    const is_gray = nbcomp == 1;
    // The frame's color transform must match what the encoder detected.
    if (frame.color_transform != frame_encode.colorTransformFromJpeg(jpg)) {
        return error.NotAJpegTranscodeFrame;
    }

    const cs = frame.chroma_subsampling;
    // The jbrd payload carries no dimensions: they come from the codestream
    // (libjxl sets jpeg_data->width from frame_dim in InitFrameOutput).
    jpg.width = @intCast(sh.xsizeOf());
    jpg.height = @intCast(sh.ysizeOf());
    const fd = FrameDim.initVarDct(
        sh.xsizeOf(),
        sh.ysizeOf(),
        cs.maxHShift(),
        cs.maxVShift(),
    );

    const num_toc = toc_mod.numTocEntries(
        fd.num_groups,
        fd.num_dc_groups,
        1,
    );
    var toc = try toc_mod.readTocWithPermutation(
        allocator,
        num_toc,
        &br,
    );
    defer toc.deinit(allocator);
    var toc_sizes: []u32 = undefined;
    var section_starts: []usize = undefined;
    try toc.logicalLayout(
        allocator,
        &toc_sizes,
        &section_starts,
    );
    defer allocator.free(toc_sizes);
    defer allocator.free(section_starts);
    const total_len = toc.totalSize();
    const base = br.totalBitsConsumed() / 8;
    if (base + total_len > codestream.len) return error.NotEnoughBytes;
    const sections = codestream[base .. base + total_len];

    var st = FrameState{
        .allocator = allocator,
        .fd = fd,
        .cs = cs,
        .color_transform = frame.color_transform,
        .is_gray = is_gray,
        .jpeg_c_map = frame_encode.jpegOrder(
            frame.color_transform,
            is_gray,
        ),
        .block_ctx_map = block_context.BlockCtxMap.init(allocator),
    };
    defer st.deinit();
    st.global = stream.EntropyCode.init(allocator);
    st.global_initialized = true;

    const hshift = [3]usize{ cs.hShift(0), cs.hShift(1), cs.hShift(2) };
    const vshift = [3]usize{ cs.vShift(0), cs.vShift(1), cs.vShift(2) };
    for (0..3) |c| {
        const w = fd.xsize_blocks >> @intCast(hshift[c]);
        const h = fd.ysize_blocks >> @intCast(vshift[c]);
        st.dc[c] = try allocator.alloc(i32, w * h);
        st.dc_w[c] = w;
        st.dc_h[c] = h;
    }
    st.quant_dc = try allocator.alloc(u8, fd.xsize_blocks * fd.ysize_blocks);

    // Chroma-from-luma AC maps: one tile per 8x8 blocks (full image).
    st.cmap_w = (fd.xsize_blocks + 7) >> 3;
    st.cmap_h = (fd.ysize_blocks + 7) >> 3;
    st.cmap_ytox = try allocator.alloc(i32, st.cmap_w * st.cmap_h);
    st.cmap_ytob = try allocator.alloc(i32, st.cmap_w * st.cmap_h);
    @memset(st.cmap_ytox, 0);
    @memset(st.cmap_ytob, 0);

    // Set up the JPEG component dimensions and sampling factors.
    for (0..nbcomp) |c| {
        const comp = &jpg.components.items[st.jpeg_c_map[c]];
        comp.width_in_blocks = @intCast(fd.xsize_blocks >> @intCast(hshift[c]));
        comp.height_in_blocks = @intCast(fd.ysize_blocks >> @intCast(vshift[c]));
        comp.h_samp_factor = @intCast(@as(u32, 1) << @intCast(cs.rawHShift(c)));
        comp.v_samp_factor = @intCast(@as(u32, 1) << @intCast(cs.rawVShift(c)));
        const num_blocks: usize = @intCast(
            comp.width_in_blocks * comp.height_in_blocks,
        );
        try comp.coeffs.resize(
            allocator,
            num_blocks * jd.kDCTBlockSize,
        );
    }

    const single_section = num_toc == 1;

    if (single_section) {
        // Everything lives in section 0, read sequentially.
        const s0 = section_starts[0] + toc_sizes[0];
        var sec = BitReader.init(sections[section_starts[0]..s0]);
        try decodeDCGlobal(allocator, &sec, &st);
        try decodeDCGroup(allocator, &sec, &st, 0);
        try decodeACGlobal(allocator, &sec, &st);
        try decodeACGroup(allocator, &sec, &st, jpg, 0);
        try sec.close();
    } else {
        {
            const end0 = section_starts[0] + toc_sizes[0];
            var sec = BitReader.init(
                sections[section_starts[0]..end0],
            );
            try decodeDCGlobal(allocator, &sec, &st);
            try sec.close();
        }
        for (0..fd.num_dc_groups) |g| {
            const sg = 1 + g;
            const start = section_starts[sg];
            var sec = BitReader.init(
                sections[start .. start + toc_sizes[sg]],
            );
            try decodeDCGroup(allocator, &sec, &st, g);
            try sec.close();
        }
        {
            const ac_global_sec = 1 + fd.num_dc_groups;
            const start = section_starts[ac_global_sec];
            var sec = BitReader.init(
                sections[start .. start + toc_sizes[ac_global_sec]],
            );
            try decodeACGlobal(allocator, &sec, &st);
            try sec.close();
        }
        for (0..fd.num_groups) |g| {
            const sec_index = toc_mod.acGroupIndex(
                0,
                g,
                fd.num_groups,
                fd.num_dc_groups,
            );
            const start = section_starts[sec_index];
            var sec = BitReader.init(
                sections[start .. start + toc_sizes[sec_index]],
            );
            try decodeACGroup(
                allocator,
                &sec,
                &st,
                jpg,
                g,
            );
            try sec.close();
        }
    }

    // Fill the JPEG quant tables from the RAW codestream tables
    // (ProcessACGlobal "Set JPEG decoding data").
    try fillQuantTables(&st, jpg);
}

fn decodeDCGlobal(allocator: Allocator, br: *BitReader, st: *FrameState) !void {
    // DequantMatrices DC.
    var ddc = quant_mod.DequantDC{};
    try ddc.read(br);
    // QuantizerParams.
    var qp = quant_mod.QuantizerParams{};
    try qp.read(br);
    // BlockCtxMap.
    try st.block_ctx_map.read(br);
    // CfL DC: must be JPEG-compatible.
    var cfl = CflDC{};
    try cfl.read(br);
    if (!cfl.isJpegCompatible()) return error.NotAJpegTranscodeFrame;

    // Modular global info: has_tree + tree + global histograms.
    const has_tree = br.readBits(1) != 0;
    if (!has_tree) return error.NotAJpegTranscodeFrame; // jpeg frames carry data streams
    const tree_size_limit = @min(
        @as(usize, 1) << 22,
        1024 + st.fd.xsize * st.fd.ysize * 3 / 16,
    );
    var tree_code = dec_ans.ANSCode{};
    defer tree_code.deinit(allocator);
    var tree_ctx_map: std.ArrayList(u8) = .empty;
    defer tree_ctx_map.deinit(allocator);
    try dec_ans.decodeHistograms(
        allocator,
        br,
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
        br,
    );
    defer tree_reader.ans.deinit();
    try ma.decodeTree(
        br,
        &tree_reader,
        &st.global.tree,
        allocator,
        tree_size_limit,
    );
    try dec_ans.decodeHistograms(
        allocator,
        br,
        st.global.numContexts(),
        &st.global.code,
        &st.global.context_map,
        false,
    );
}

/// One DC group section: extra_dc_precision, VarDCTDC stream, ACMetadata.
fn decodeDCGroup(
    allocator: Allocator,
    br: *BitReader,
    st: *FrameState,
    g: usize,
) !void {
    const fd = &st.fd;
    const hshift = [3]usize{ st.cs.hShift(0), st.cs.hShift(1), st.cs.hShift(2) };
    const vshift = [3]usize{ st.cs.vShift(0), st.cs.vShift(1), st.cs.vShift(2) };
    const r = fd.dcGroupRect(g);

    // extra_dc_precision: only 0 is supported (we never emit 1).
    const extra_precision = br.readBits(2);
    if (extra_precision != 0) return error.UnsupportedFrameFeatures;

    // VarDCTDC stream: 3 channels [Y, X, B] with per-channel subsampled dims.
    {
        var img = Image.create(allocator);
        defer img.deinit();
        for ([3]usize{ 1, 0, 2 }) |c| {
            const w = r.xsize >> @intCast(hshift[c]);
            const h = r.ysize >> @intCast(vshift[c]);
            const ch = try Channel.create(
                allocator,
                w,
                h,
                0,
                0,
            );
            try img.channels.append(allocator, ch);
        }
        const sid = ModularStreamId.varDCTDC(g).id(fd);
        try stream.decodeStream(.{
            .allocator = allocator,
            .br = br,
            .image = &img,
            .group_id = sid,
            .max_chan_size = 0xFFFFFF,
            .global = &st.global,
            .undo_transforms = true,
            .bitdepth = 8,
        });
        // Store the DC values into the frame planes.
        for ([3]usize{ 1, 0, 2 }) |c| {
            // Channel c lives at stream-image index (c < 2 ? c ^ 1 : c):
            // the stream image order is [Y, X, B].
            const ch = &img.channels.items[if (c < 2) c ^ 1 else c];
            const base_x = r.x0 >> @intCast(hshift[c]);
            const base_y = r.y0 >> @intCast(vshift[c]);
            for (0..ch.h) |y| {
                for (0..ch.w) |x| {
                    st.dc[c][(base_y + y) * st.dc_w[c] + base_x + x] = ch.row(y)[x];
                }
            }
        }
    }

    // Per-block DC threshold buckets for this DC group (DequantDC bctx).
    {
        const thresholds = st.block_ctx_map.dc_thresholds[1].items;
        const y_plane_w = st.dc_w[1];
        const y_plane = st.dc[1];
        for (0..r.ysize) |iy| {
            for (0..r.xsize) |ix| {
                const y_dc = y_plane[(r.y0 + iy) * y_plane_w + r.x0 + ix];
                var bucket: u8 = 0;
                for (thresholds) |t| {
                    if (y_dc > t) bucket += 1;
                }
                st.quant_dc[(r.y0 + iy) * st.fd.xsize_blocks + r.x0 + ix] = bucket;
            }
        }
    }

    // ACMetadata: count, then the 4-channel stream.
    {
        const upper_bound = r.xsize * r.ysize;
        const nb_bits = frame_encode.ceilLog2Nonzero(upper_bound);
        const count: usize = if (nb_bits != 0)
            @intCast(br.readBits(nb_bits) + 1)
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
        const sid = ModularStreamId.acMetadata(g).id(fd);
        try stream.decodeStream(.{
            .allocator = allocator,
            .br = br,
            .image = &img,
            .group_id = sid,
            .max_chan_size = 0xFFFFFF,
            .global = &st.global,
            .undo_transforms = true,
            .bitdepth = 8,
        });
        // Validate the metadata values (JPEG transcode: DCT-8, qf 1,
        // zero CfL maps become the ytox/ytob color maps, epf sharpness 0).
        //
        // Channels 0/1 carry the chroma-from-luma AC maps (YToX/YToB per
        // tile of 8x8 blocks, clamped to int8 like libjxl's
        // ConvertPlaneAndClamp); store them into the frame planes.
        {
            const map_x0 = r.x0 >> 3;
            const map_y0 = r.y0 >> 3;
            for (0..cr_h) |my| {
                const src_row_x = img.channels.items[0].row(my);
                const src_row_b = img.channels.items[1].row(my);
                for (0..cr_w) |mx| {
                    st.cmap_ytox[(map_y0 + my) * st.cmap_w + map_x0 + mx] =
                        std.math.clamp(src_row_x[mx], -128, 127);
                    st.cmap_ytob[(map_y0 + my) * st.cmap_w + map_x0 + mx] =
                        std.math.clamp(src_row_b[mx], -128, 127);
                }
            }
        }
        if (count != r.xsize * r.ysize) return error.NotAJpegTranscodeFrame;
        for (0..count) |i| {
            const acs = img.channels.items[2].row(0)[i];
            const qf = img.channels.items[2].row(1)[i];
            if (acs != 0) return error.UnsupportedVarDct; // only DCT-8
            if (qf != 0) return error.NotAJpegTranscodeFrame; // raw quant = 1
        }
        for (img.channels.items[3].plane.data) |v| {
            if (v < 0 or v >= 8) return error.UnsupportedFrameFeatures;
            if (v != 0) return error.NotAJpegTranscodeFrame; // jpeg writes 0
        }
    }
}

/// The AC global section: DequantMatrices (RAW table + library tables),
/// num_histograms, used_orders (must be 0), AC histograms.
fn decodeACGlobal(allocator: Allocator, br: *BitReader, st: *FrameState) !void {
    const fd = &st.fd;

    // DequantMatrices.
    const all_default = br.readBits(1) != 0;
    if (all_default) return error.NotAJpegTranscodeFrame; // jpeg needs RAW
    for (0..quant_mod.kNumQuantTables) |i| {
        const mode = br.readBits(3);
        if (i == 0) {
            if (mode != @intFromEnum(quant_mod.QuantMode.raw)) {
                return error.NotAJpegTranscodeFrame;
            }
            var raw = quant_mod.RawQuantTable{};
            try raw.read(br); // F16 denominator
            st.qtable_den = raw.den;
            // The quant table modular stream.
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
                .global = &st.global,
                .undo_transforms = true,
                .bitdepth = 8,
            });
            try quant_mod.readQuantTableImage(
                &img,
                0,
                &raw,
            );
            st.qtable = raw.values;
        } else {
            if (mode != @intFromEnum(quant_mod.QuantMode.library)) {
                return error.UnsupportedVarDct;
            }
        }
    }
    // djxl requires the denominator to stay within 1e-8 of 1/(8*255).
    if (@abs(st.qtable_den - 1.0 / (8.0 * 255.0)) > 1e-8) {
        return error.NotAJpegTranscodeFrame;
    }

    // num_histograms.
    const num_histo_bits = frame_encode.ceilLog2Nonzero(fd.num_groups);
    st.num_histograms = 1 + (if (num_histo_bits != 0)
        @as(usize, @intCast(br.readBits(num_histo_bits)))
    else
        0);
    if (st.num_histograms != 1) return error.UnsupportedFrameFeatures; // we write 1

    // CfL fixed-point ratio table (dec_group.cc): transposed so that it
    // applies directly to JPEG-order blocks.
    for (0..3) |c| {
        for (0..64) |i| {
            const n = st.qtable[64 + i];
            const d = st.qtable[64 * c + i];
            if (n <= 0 or d <= 0 or n >= 65536 or d >= 65536) {
                return error.InvalidQuantTable;
            }
            st.scaled_qtable[64 * c + (i % 8) * 8 + (i / 8)] =
                @divTrunc((@as(i64, 1) << 11) * @as(i64, n), @as(i64, d));
        }
    }

    // used_orders: natural (0) or custom (Lehmer-coded per-channel)
    // coefficient orders, as cjxl emits for JPEG recompression.
    const used_orders = U32Coder.read(
        coeff_order_mod.kOrderEnc,
        br,
    );
    try coeff_order_mod.decodeCoeffOrders(
        allocator,
        used_orders,
        &st.orders,
        br,
    );

    // AC histograms.
    try dec_ans.decodeHistograms(
        allocator,
        br,
        st.block_ctx_map.numACContexts(),
        &st.ac_code,
        &st.ac_context_map,
        false,
    );
    st.ac_initialized = true;
}

/// One AC group: ANS-coded coefficient tokens -> JPEG coefficients.
fn decodeACGroup(
    allocator: Allocator,
    br: *BitReader,
    st: *FrameState,
    jpg: *JPEGData,
    g: usize,
) !void {
    const fd = &st.fd;
    const hshift = [3]usize{ st.cs.hShift(0), st.cs.hShift(1), st.cs.hShift(2) };
    const vshift = [3]usize{ st.cs.vShift(0), st.cs.vShift(1), st.cs.vShift(2) };
    const rect = fd.blockGroupRect(g);
    // Per-channel scan orders (custom when used_orders was set).
    const orders: [3][]const u32 = .{
        st.orders[0][0..],
        st.orders[1][0..],
        st.orders[2][0..],
    };

    var reader = try dec_ans.ANSSymbolReader.create(
        allocator,
        &st.ac_code,
        br,
        0,
    );
    defer reader.deinit();

    var cache = try ac_coefficient.GroupCoeffCache.init(
        allocator,
    );
    defer cache.deinit();
    // Y block saved for chroma-from-luma restoration (CfL).
    var y_block: ?[64]i32 = null;

    // dcoff for color_transform kNone (idc = dc + 1024/qt_dc).
    const dcoff: [3]i32 = if (st.color_transform == .none) .{
        @divTrunc(@as(i32, 1024), st.qtable[0]),
        @divTrunc(@as(i32, 1024), st.qtable[64]),
        @divTrunc(@as(i32, 1024), st.qtable[128]),
    } else .{ 0, 0, 0 };

    var block: [64]i32 = @splat(0);

    for (0..rect.ysize) |by| {
        const sby = [3]usize{
            by >> @intCast(vshift[0]),
            by >> @intCast(vshift[1]),
            by >> @intCast(vshift[2]),
        };
        // DC bucket row for this group row (full grid).
        const qdc_row = st.quant_dc[(rect.y0 + by) *
            fd.xsize_blocks + rect.x0 ..][0..rect.xsize];
        // DC values rows per channel for the JPEG output.
        for (0..rect.xsize) |bx| {
            const sbx = [3]usize{
                bx >> @intCast(hshift[0]),
                bx >> @intCast(hshift[1]),
                bx >> @intCast(hshift[2]),
            };
            for ([3]usize{ 1, 0, 2 }) |c| {
                if ((sbx[c] << @intCast(hshift[c])) != bx) continue;
                if ((sby[c] << @intCast(vshift[c])) != by) continue;

                @memset(&block, 0);
                const row = cache.nzeros[c][sby[c] * 32 ..][0..32];
                const top: ?[]const i32 = if (sby[c] == 0)
                    null
                else
                    cache.nzeros[c][(sby[c] - 1) * 32 ..][0..32];
                try ac_coefficient.decodeACVarBlock(
                    &reader,
                    br,
                    st.ac_context_map.items,
                    orders[c],
                    row,
                    top,
                    c,
                    sbx[c],
                    bx,
                    qdc_row,
                    &st.block_ctx_map,
                    &block,
                );

                // In-place transpose to JPEG order (the decoded block is
                // stored transposed in the codestream).
                transposeInPlace(&block);

                // Chroma-from-luma (cjxl `--jpeg_reconstruction_cfl`, default
                // on): for 444 3-component frames, chroma AC blocks carry
                // QCR = QChroma - cfl_factor(Y); restore by adding it back.
                if (st.is444Cfl()) {
                    const ty = (rect.y0 + by) >> 3;
                    const tx = (rect.x0 + bx) >> 3;
                    if (c == 1) {
                        // Y channel: save the (JPEG-order) block when any
                        // chroma map at this tile is active (reset otherwise).
                        y_block = if (st.cmapHasCfl(ty, tx))
                            block
                        else
                            null;
                    } else if (c == 0 or c == 2) {
                        const map_v: i32 = if (c == 0)
                            st.cmap_ytox[ty * st.cmap_w + tx]
                        else
                            st.cmap_ytob[ty * st.cmap_w + tx];
                        if (map_v != 0) {
                            if (y_block == null) return error.InvalidANSStream;
                            applyCfl(
                                &block,
                                &y_block.?,
                                c,
                                map_v,
                                &st.scaled_qtable,
                            );
                        }
                    }
                }

                // Store the block into the JPEG component coefficients
                // (already in JPEG order); the DC slot is filled from
                // the DC plane. Grayscale propagates only the Y channel
                // (libjxl: "Propagate only Y for grayscale").
                if (!(st.is_gray and c != 1)) {
                    const gb = ac_coefficient.globalBlock(
                        hshift[c],
                        vshift[c],
                        .{
                            .x0 = rect.x0,
                            .y0 = rect.y0,
                            .xsize = rect.xsize,
                            .ysize = rect.ysize,
                        },
                        sbx[c],
                        sby[c],
                    );
                    try storeBlock(
                        st,
                        jpg,
                        c,
                        &block,
                        gb[0],
                        gb[1],
                        dcoff[c],
                    );
                }
            }
        }
    }

    if (!reader.checkANSFinalState()) return error.InvalidANSStream;
}

/// In-place transpose of an 8x8 block: position (y, x) gets the value
/// from (x, y).
fn transposeInPlace(block: *[64]i32) void {
    for (0..8) |x| {
        for (x + 1..8) |y| {
            const a: usize = 8 * y + x;
            const b: usize = 8 * x + y;
            const tmp = block[a];
            block[a] = block[b];
            block[b] = tmp;
        }
    }
}

/// Restores chroma-from-luma on a JPEG-order chroma block (dec_group.cc):
/// QChroma = QCR + cfl_factor where
/// cfl_factor = (Y * ((scaled_qtable * scale + half) >> 11) + half) >> 11.
fn applyCfl(
    block: *[64]i32,
    y_block: *const [64]i32,
    c: usize,
    map_value: i32,
    scaled_qtable: *const [3 * 64]i64,
) void {
    // libjxl ColorCorrelation::RatioJPEG: v * (1<<11) / kDefaultColorFactor.
    const scale: i64 = @divTrunc(@as(i64, map_value) * 2048, 84);
    const round: i64 = @as(i64, 1) << 10;
    for (0..64) |i| {
        const qt = scaled_qtable[64 * c + i];
        const coeff_scale: i64 = (qt * scale + round) >> 11;
        const cfl_factor: i64 = (@as(i64, y_block[i]) * coeff_scale + round) >> 11;
        block[i] += @truncate(cfl_factor);
    }
}

/// Writes a decoded (already JPEG-order) block into the JPEG component
/// coefficients (fill DC, range check).
fn storeBlock(
    st: *FrameState,
    jpg: *JPEGData,
    c: usize,
    block: *[64]i32,
    sbx: usize,
    sby: usize,
    dcoff: i32,
) !void {
    const comp = &jpg.components.items[st.jpeg_c_map[c]];
    const w_c: usize = @intCast(comp.width_in_blocks);
    const base = (sby * w_c + sbx) * jd.kDCTBlockSize;

    // DC from the DC plane (idc - dcoff).
    const dc_value = st.dc[c][sby * st.dc_w[c] + sbx];
    block[0] = dc_value - dcoff;

    for (0..jd.kDCTBlockSize) |i| {
        const v = block[i];
        if (v < -4095 or v > 4095) return error.InvalidJpegCoefficients;
        comp.coeffs.items[base + i] = @intCast(v);
    }
}

/// Fills the JPEG quant tables from the RAW codestream table (the inverse
/// of the encoder's table construction): values[x*8+y] of table qpos equals
/// qtable[quant_c * 64 + y*8 + x].
fn fillQuantTables(st: *FrameState, jpg: *JPEGData) !void {
    const nbcomp = jpg.components.items.len;
    const is_gray = nbcomp == 1;
    var qt_set: u32 = 0;
    for (0..nbcomp) |c| {
        const quant_c: usize = if (is_gray) 1 else c;
        const comp = &jpg.components.items[st.jpeg_c_map[c]];
        const qpos = comp.quant_idx;
        if (qpos >= jpg.quant.items.len) return error.NotAJpegTranscodeFrame;
        qt_set |= @as(u32, 1) << @intCast(qpos);
        for (0..8) |x| {
            for (0..8) |y| {
                jpg.quant.items[qpos].values[x * 8 + y] = st.qtable[quant_c * 64 + y * 8 + x];
            }
        }
    }
    for (0..jpg.quant.items.len) |i| {
        if (qt_set & (@as(u32, 1) << @intCast(i)) != 0) continue;
        if (i == 0) return error.NotAJpegTranscodeFrame; // first table unused
        // Unused quant table is a copy of the previous one.
        for (0..jd.kDCTBlockSize) |j| {
            jpg.quant.items[i].values[j] = jpg.quant.items[i - 1].values[j];
        }
    }
}
