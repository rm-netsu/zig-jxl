//! Patch dictionary, port of libjxl `patch_dictionary_internal.h`,
//! `dec_patch_dictionary.{h,cc}` and the `PatchDictionaryEncoder::Encode` /
//! `SubtractFrom` halves of `enc_patch_dictionary.cc`.
//!
//! Patches ("reference patches", spec 18181-1 `patches`): repeated image
//! fragments are coded once in a *patch reference frame* (a
//! `kReferenceOnly` frame saved into reference slot 3 before the color
//! transform) and referenced from the main frame through a dictionary of
//! positions serialized in the DC global section. The main frame carries
//! the background (the encoder subtracts the patch content), the decoder
//! blends it back with `PerformBlending` (blending.cc/alpha.cc).
//!
//! The bit-level layout, the ANS contexts, the y-interval tree and the
//! blending math are line-faithful ports; the only deliberate difference
//! is the application point: libjxl blends patches on float render-
//! pipeline rows, this port converts the decoded modular rows to/from
//! float around the identical math (bit-exact for integer samples, since
//! every intermediate is an exact multiple of 1/maxval well inside fp32
//! precision — the same argument that makes libjxl's own lossless patch
//! roundtrip exact).

const std = @import("std");

const bit_io = @import("../base/bit_io.zig");

const dec_ans = @import("../entropy/dec_ans.zig");
const enc_ans = @import("../entropy/enc_ans.zig");
const stream = @import("../modular/stream.zig");
const image_mod = @import("../modular/image.zig");
const common = @import("../modular/common.zig");
const metadata_mod = @import("../header/metadata.zig");
const float_convert = @import("../base/float_convert.zig");

const Allocator = std.mem.Allocator;
const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const Image = image_mod.Image;
const packSigned = common.packSigned;
const unpackSigned = common.unpackSigned;
const ExtraChannelInfo = metadata_mod.ExtraChannelInfo;

/// The sample format of one channel plane (modular integer samples or a
/// custom float bit pattern). Integer channels carry `v` in [0, 2^bits);
/// float channels carry the [bits]/[exp_bits] bit pattern produced by
/// libjxl `float_to_int` (see `float_convert`).
pub const SampleFormat = struct {
    bits: u32 = 8,
    exp_bits: u32 = 0,
    floating: bool = false,
};

/// Plane sample -> float: integers scale by 1/maxval (libjxl
/// `ConvertFromExternal`), float patterns convert via `int_to_float`.
pub fn sampleToFloat(v: i32, fmt: SampleFormat) f32 {
    if (!fmt.floating) {
        const maxval: f64 = @floatFromInt(
            (@as(u64, 1) << @intCast(fmt.bits)) - 1,
        );
        return @as(f32, @floatFromInt(v)) * @as(f32, @floatCast(1.0 / maxval));
    }
    return float_convert.intToFloat(
        v,
        @intCast(fmt.bits),
        @intCast(fmt.exp_bits),
    );
}

/// Float -> plane sample: the output conversion of `applyToImage`
/// (clamp01, x maxval, round-to-nearest-even, clamp) for integers; the
/// `float_to_int` pattern conversion for float channels (errors when the
/// value is not exactly representable — libjxl fails the same way in
/// `enc_modular.cc` `float_to_int`).
pub fn floatToSample(v: f32, fmt: SampleFormat) float_convert.FloatError!i32 {
    if (!fmt.floating) {
        const maxval: i64 = (@as(i64, 1) << @intCast(fmt.bits)) - 1;
        const scaled = clamp01(v) * @as(f32, @floatFromInt(maxval));
        const rounded = nearestEven(scaled);
        return @intCast(std.math.clamp(rounded, 0, maxval));
    }
    return float_convert.floatToInt(
        v,
        @intCast(fmt.bits),
        @intCast(fmt.exp_bits),
    );
}

/// Context numbers as specified in Section C.4.5, Listing C.2
/// (libjxl `patch_dictionary_internal.h`).
pub const Contexts = struct {
    pub const kNumRefPatchContext: usize = 0;
    pub const kReferenceFrameContext: usize = 1;
    pub const kPatchSizeContext: usize = 2;
    pub const kPatchReferencePositionContext: usize = 3;
    pub const kPatchPositionContext: usize = 4;
    pub const kPatchBlendModeContext: usize = 5;
    pub const kPatchOffsetContext: usize = 6;
    pub const kPatchCountContext: usize = 7;
    pub const kPatchAlphaChannelContext: usize = 8;
    pub const kPatchClampContext: usize = 9;
    pub const kNumPatchDictionaryContexts: usize = 10;
};

/// libjxl `PatchBlendMode` (dec_patch_dictionary.h).
pub const PatchBlendMode = enum(u8) {
    none = 0,
    replace = 1,
    add = 2,
    mul = 3,
    blend_above = 4,
    blend_below = 5,
    alpha_weighted_add_above = 6,
    alpha_weighted_add_below = 7,

    pub const kNumPatchBlendModes: usize = 8;
};

pub fn usesAlpha(mode: PatchBlendMode) bool {
    return mode == .blend_above or mode == .blend_below or
        mode == .alpha_weighted_add_above or mode == .alpha_weighted_add_below;
}

pub fn usesClamp(mode: PatchBlendMode) bool {
    return usesAlpha(mode) or mode == .mul;
}

/// libjxl `PatchBlending`.
pub const PatchBlending = struct {
    mode: PatchBlendMode = .none,
    alpha_channel: u32 = 0,
    clamp: bool = false,
};

/// Position and size of the patch in the reference frame.
pub const PatchReferencePosition = struct {
    ref: usize,
    x0: usize,
    y0: usize,
    xsize: usize,
    ysize: usize,
};

/// Position of the top-left corner of the patch in the image.
pub const PatchPosition = struct {
    x: usize,
    y: usize,
    ref_pos_idx: usize,
};

/// libjxl `kPatchFrameReferenceId` (enc_patch_dictionary.cc): the reference
/// slot the encoder stores the patch reference frame in.
pub const kPatchFrameReferenceId: usize = 3;

pub const kMaxNumReferenceFrames: usize = 4;

/// One reference-frame slot: float planes (color channels first, then the
/// extra channels) on the nominal [0,1] grid, plus the libjxl
/// `ReferenceFrame::ib_is_in_xyb` flag (`save_before_color_transform` of
/// the frame that filled the slot — patches may only use pre-transform
/// references).
pub const ReferenceFrame = struct {
    /// `n_color + n_ec` planes of `xsize * ysize` f32 samples.
    planes: []const []f32,
    xsize: usize,
    ysize: usize,
    origin_x: i32 = 0,
    origin_y: i32 = 0,
    /// libjxl `ReferenceFrame::ib_is_in_xyb`.
    before_color_transform: bool,

    pub fn isEmpty(self: *const ReferenceFrame) bool {
        return self.xsize == 0 or self.ysize == 0;
    }
};

/// The four reference slots (empty = null).
pub const RefStore = [4]?ReferenceFrame;

/// Frees the owned float planes of a reference store.
pub fn freeRefStore(allocator: Allocator, refs: *RefStore) void {
    for (refs) |*slot| {
        if (slot.*) |*rf| {
            for (rf.planes) |p| allocator.free(p);
            allocator.free(rf.planes);
            slot.* = null;
        }
    }
}

/// Node of the y-interval tree over the patches (libjxl `PatchTreeNode`).
const PatchTreeNode = struct {
    left_child: isize = -1,
    right_child: isize = -1,
    y_center: usize = 0,
    /// Range of patches in sorted_patches_y0_/sorted_patches_y1_ that
    /// contain the row y_center.
    start: usize = 0,
    num: usize = 0,
};

const PatchIdxY = struct { y: usize, idx: usize };

/// Port of `PatchDictionary` (dec_patch_dictionary.h/cc) plus the
/// encoder-side `PatchDictionaryEncoder::Encode` / `SetPositions` /
/// `SubtractFrom` entry points.
pub const PatchDictionary = struct {
    allocator: Allocator,
    positions: std.ArrayList(PatchPosition) = .empty,
    ref_positions: std.ArrayList(PatchReferencePosition) = .empty,
    blendings: std.ArrayList(PatchBlending) = .empty,
    blendings_stride: usize = 0,
    /// Borrowed reference-frame store used at apply time.
    refs: ?*const RefStore = null,

    // Interval tree on the y coordinates of the patches.
    patch_tree: std.ArrayList(PatchTreeNode) = .empty,
    num_patches: std.ArrayList(usize) = .empty,
    sorted_patches_y0: std.ArrayList(PatchIdxY) = .empty,
    sorted_patches_y1: std.ArrayList(PatchIdxY) = .empty,

    pub fn init(allocator: Allocator) PatchDictionary {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *PatchDictionary) void {
        self.positions.deinit(self.allocator);
        self.ref_positions.deinit(self.allocator);
        self.blendings.deinit(self.allocator);
        self.patch_tree.deinit(self.allocator);
        self.num_patches.deinit(self.allocator);
        self.sorted_patches_y0.deinit(self.allocator);
        self.sorted_patches_y1.deinit(self.allocator);
    }

    pub fn hasAny(self: *const PatchDictionary) bool {
        return self.positions.items.len != 0;
    }

    /// Reference-frame dependencies as a bit mask (bits 0-3).
    pub fn getReferences(self: *const PatchDictionary) u8 {
        var result: u8 = 0;
        for (self.ref_positions.items) |*rp| {
            result |= @as(u8, 1) << @intCast(rp.ref);
        }
        return result;
    }

    /// libjxl `PatchDictionaryEncoder::SetPositions`.
    pub fn setPositions(
        self: *PatchDictionary,
        positions: []const PatchPosition,
        ref_positions: []const PatchReferencePosition,
        blendings: []const PatchBlending,
        blendings_stride: usize,
    ) !void {
        self.positions.clearRetainingCapacity();
        try self.positions.appendSlice(self.allocator, positions);
        self.ref_positions.clearRetainingCapacity();
        try self.ref_positions.appendSlice(
            self.allocator,
            ref_positions,
        );
        self.blendings.clearRetainingCapacity();
        try self.blendings.appendSlice(self.allocator, blendings);
        self.blendings_stride = blendings_stride;
        self.computePatchTree();
    }

    // -----------------------------------------------------------------------
    // Bitstream decode (PatchDictionary::Decode)
    // -----------------------------------------------------------------------

    /// Reads a patch dictionary from the DC global section. `xsize`/`ysize`
    /// are the frame's padded dimensions (libjxl passes
    /// `frame_dim.xsize_padded`/`ysize_padded`).
    pub fn decode(
        self: *PatchDictionary,
        br: *BitReader,
        xsize: usize,
        ysize: usize,
        num_extra_channels: usize,
        refs: *const RefStore,
        uses_extra_channels: *bool,
    ) !void {
        self.refs = refs;
        self.positions.clearRetainingCapacity();
        self.ref_positions.clearRetainingCapacity();
        self.blendings.clearRetainingCapacity();
        self.blendings_stride = num_extra_channels + 1;

        var code: dec_ans.ANSCode = .{};
        defer code.deinit(self.allocator);
        var context_map: std.ArrayList(u8) = .empty;
        defer context_map.deinit(self.allocator);
        try dec_ans.decodeHistograms(
            self.allocator,
            br,
            Contexts.kNumPatchDictionaryContexts,
            &code,
            &context_map,
            false,
        );
        var reader = try dec_ans.ANSSymbolReader.create(
            self.allocator,
            &code,
            br,
            0,
        );
        defer reader.deinit();
        const cmap = context_map.items;
        const readNum = struct {
            fn f(
                r: *dec_ans.ANSSymbolReader,
                ctx: usize,
                b: *BitReader,
                m: []const u8,
            ) usize {
                return r.readHybridUint(ctx, b, m);
            }
        }.f;

        const num_ref_patch = readNum(
            &reader,
            Contexts.kNumRefPatchContext,
            br,
            cmap,
        );
        // Limit max memory usage of patches to about 66 bytes per pixel
        // (assuming 8 bytes per size_t).
        const num_pixels = xsize * ysize;
        const max_ref_patches = 1024 + num_pixels / 4;
        const max_patches = max_ref_patches * 4;
        const max_blending_infos = max_patches * 4;
        if (num_ref_patch > max_ref_patches) return error.TooManyPatches;

        var total_patches: usize = 0;
        var next_size: usize = 1;

        for (0..num_ref_patch) |_| {
            var ref_pos: PatchReferencePosition = undefined;
            ref_pos.ref = readNum(
                &reader,
                Contexts.kReferenceFrameContext,
                br,
                cmap,
            );
            if (ref_pos.ref >= kMaxNumReferenceFrames)
                return error.InvalidPatchReferenceFrame;
            const ref_frame = &(refs[ref_pos.ref] orelse
                return error.InvalidPatchReferenceFrame);
            if (ref_frame.isEmpty()) return error.InvalidPatchReferenceFrame;
            if (!ref_frame.before_color_transform) {
                // libjxl: "Patches cannot use frames saved post color
                // transforms".
                return error.InvalidPatchReferenceFrame;
            }
            ref_pos.x0 = readNum(
                &reader,
                Contexts.kPatchReferencePositionContext,
                br,
                cmap,
            );
            ref_pos.y0 = readNum(
                &reader,
                Contexts.kPatchReferencePositionContext,
                br,
                cmap,
            );
            ref_pos.xsize = readNum(
                &reader,
                Contexts.kPatchSizeContext,
                br,
                cmap,
            ) + 1;
            ref_pos.ysize = readNum(
                &reader,
                Contexts.kPatchSizeContext,
                br,
                cmap,
            ) + 1;
            if (ref_pos.x0 + ref_pos.xsize > ref_frame.xsize) {
                return error.InvalidPatchReferencePosition;
            }
            if (ref_pos.y0 + ref_pos.ysize > ref_frame.ysize) {
                return error.InvalidPatchReferencePosition;
            }
            var id_count = readNum(
                &reader,
                Contexts.kPatchCountContext,
                br,
                cmap,
            );
            if (id_count > max_patches) return error.TooManyPatches;
            id_count += 1;
            total_patches += id_count;
            if (total_patches > max_patches) return error.TooManyPatches;
            if (next_size < total_patches) {
                next_size *= 2;
                next_size = @min(next_size, max_patches);
            }
            if (next_size * self.blendings_stride > max_blending_infos) {
                return error.TooManyPatches;
            }
            try self.positions.ensureTotalCapacity(
                self.allocator,
                next_size,
            );
            try self.blendings.ensureTotalCapacity(
                self.allocator,
                next_size * self.blendings_stride,
            );
            const choose_alpha = (num_extra_channels > 1);
            for (0..id_count) |i| {
                var pos: PatchPosition = undefined;
                pos.ref_pos_idx = self.ref_positions.items.len;
                if (i == 0) {
                    pos.x = readNum(
                        &reader,
                        Contexts.kPatchPositionContext,
                        br,
                        cmap,
                    );
                    pos.y = readNum(
                        &reader,
                        Contexts.kPatchPositionContext,
                        br,
                        cmap,
                    );
                } else {
                    const deltax = unpackSigned(
                        @intCast(
                            readNum(
                                &reader,
                                Contexts.kPatchOffsetContext,
                                br,
                                cmap,
                            ),
                        ),
                    );
                    const back = self.positions.items[self.positions.items.len - 1];
                    if (deltax < 0 and @as(u64, @intCast(-deltax)) > back.x) {
                        return error.InvalidPatchPosition;
                    }
                    pos.x = @intCast(@as(i64, @intCast(back.x)) + deltax);
                    const deltay = unpackSigned(
                        @intCast(
                            readNum(
                                &reader,
                                Contexts.kPatchOffsetContext,
                                br,
                                cmap,
                            ),
                        ),
                    );
                    if (deltay < 0 and @as(u64, @intCast(-deltay)) > back.y) {
                        return error.InvalidPatchPosition;
                    }
                    pos.y = @intCast(@as(i64, @intCast(back.y)) + deltay);
                }
                if (pos.x + ref_pos.xsize > xsize) return error.InvalidPatchPosition;
                if (pos.y + ref_pos.ysize > ysize) return error.InvalidPatchPosition;
                for (0..self.blendings_stride) |j| {
                    const blend_mode = readNum(
                        &reader,
                        Contexts.kPatchBlendModeContext,
                        br,
                        cmap,
                    );
                    if (blend_mode >= PatchBlendMode.kNumPatchBlendModes) {
                        return error.InvalidPatchBlendMode;
                    }
                    var info: PatchBlending = .{};
                    info.mode = @enumFromInt(blend_mode);
                    if (usesAlpha(info.mode)) {
                        uses_extra_channels.* = true;
                    }
                    if (info.mode != .none and j > 0) {
                        uses_extra_channels.* = true;
                    }
                    if (usesAlpha(info.mode) and choose_alpha) {
                        info.alpha_channel = @intCast(
                            readNum(
                                &reader,
                                Contexts.kPatchAlphaChannelContext,
                                br,
                                cmap,
                            ),
                        );
                        if (info.alpha_channel >= num_extra_channels) {
                            return error.InvalidPatchAlphaChannel;
                        }
                    } else {
                        info.alpha_channel = 0;
                    }
                    if (usesClamp(info.mode)) {
                        info.clamp = readNum(
                            &reader,
                            Contexts.kPatchClampContext,
                            br,
                            cmap,
                        ) != 0;
                    } else {
                        info.clamp = false;
                    }
                    try self.blendings.append(self.allocator, info);
                }
                try self.positions.append(self.allocator, pos);
            }
            try self.ref_positions.append(self.allocator, ref_pos);
        }

        if (!reader.checkANSFinalState()) {
            return error.PatchANSFinalState;
        }

        self.computePatchTree();
    }

    // -----------------------------------------------------------------------
    // Bitstream encode (PatchDictionaryEncoder::Encode)
    // -----------------------------------------------------------------------

    /// Writes the patch dictionary (histograms + tokens over the 10 patch
    /// contexts) into `writer` (the DC global section, before the dequant
    /// matrices DC bit). Uses default `HistogramParams`, exactly like
    /// libjxl.
    pub fn encode(self: *const PatchDictionary, writer: *BitWriter) !void {
        std.debug.assert(self.hasAny());
        const allocator = self.allocator;
        var tokens: std.ArrayList(enc_ans.Token) = .empty;
        defer tokens.deinit(allocator);

        const addNum = struct {
            fn f(
                list: *std.ArrayList(enc_ans.Token),
                alloc: Allocator,
                context: usize,
                num: usize,
            ) !void {
                try list.append(
                    alloc,
                    .{
                        .context = @intCast(context),
                        .value = @intCast(num),
                    },
                );
            }
        }.f;

        // Number of distinct ref_pos runs.
        var num_ref_patch: usize = 0;
        {
            var i: usize = 0;
            while (i < self.positions.items.len) {
                const ref_pos_idx = self.positions.items[i].ref_pos_idx;
                while (i < self.positions.items.len and
                    self.positions.items[i].ref_pos_idx == ref_pos_idx) : (i += 1)
                {}
                num_ref_patch += 1;
            }
        }
        try addNum(
            &tokens,
            allocator,
            Contexts.kNumRefPatchContext,
            num_ref_patch,
        );

        const blending_stride = self.blendings_stride;
        // blending_stride == num_ec + 1; num_ec > 1 => choose_alpha.
        const choose_alpha = (blending_stride > 1 + 1);
        var blend_pos: usize = 0;
        var i: usize = 0;
        while (i < self.positions.items.len) {
            const i_start = i;
            const ref_pos_idx = self.positions.items[i].ref_pos_idx;
            const ref_pos = &self.ref_positions.items[ref_pos_idx];
            while (i < self.positions.items.len and
                self.positions.items[i].ref_pos_idx == ref_pos_idx) : (i += 1)
            {}
            const num = i - i_start;
            std.debug.assert(num > 0);
            try addNum(&tokens, allocator, Contexts.kReferenceFrameContext, ref_pos.ref);
            try addNum(&tokens, allocator, Contexts.kPatchReferencePositionContext, ref_pos.x0);
            try addNum(&tokens, allocator, Contexts.kPatchReferencePositionContext, ref_pos.y0);
            try addNum(&tokens, allocator, Contexts.kPatchSizeContext, ref_pos.xsize - 1);
            try addNum(&tokens, allocator, Contexts.kPatchSizeContext, ref_pos.ysize - 1);
            try addNum(&tokens, allocator, Contexts.kPatchCountContext, num - 1);
            for (i_start..i) |j| {
                const pos = &self.positions.items[j];
                if (j == i_start) {
                    try addNum(&tokens, allocator, Contexts.kPatchPositionContext, pos.x);
                    try addNum(&tokens, allocator, Contexts.kPatchPositionContext, pos.y);
                } else {
                    const prev = &self.positions.items[j - 1];
                    try addNum(&tokens, allocator, Contexts.kPatchOffsetContext, packSigned(@intCast(@as(i64, @intCast(pos.x)) - @as(i64, @intCast(prev.x)))));
                    try addNum(&tokens, allocator, Contexts.kPatchOffsetContext, packSigned(@intCast(@as(i64, @intCast(pos.y)) - @as(i64, @intCast(prev.y)))));
                }
                var k: usize = 0;
                while (k < blending_stride) : ({
                    k += 1;
                    blend_pos += 1;
                }) {
                    const info = &self.blendings.items[blend_pos];
                    try addNum(
                        &tokens,
                        allocator,
                        Contexts.kPatchBlendModeContext,
                        @intFromEnum(info.mode),
                    );
                    if (usesAlpha(info.mode) and choose_alpha) {
                        try addNum(
                            &tokens,
                            allocator,
                            Contexts.kPatchAlphaChannelContext,
                            info.alpha_channel,
                        );
                    }
                    if (usesClamp(info.mode)) {
                        try addNum(
                            &tokens,
                            allocator,
                            Contexts.kPatchClampContext,
                            @intFromBool(info.clamp),
                        );
                    }
                }
            }
        }

        var codes = enc_ans.EntropyEncodingData.init(
            allocator,
        );
        defer codes.deinit();
        var params = enc_ans.HistogramParams{}; // libjxl: HistogramParams()
        var token_streams = [_]std.ArrayList(enc_ans.Token){
            tokens,
        };
        _ = try stream.buildHistograms(
            allocator,
            &params,
            Contexts.kNumPatchDictionaryContexts,
            token_streams[0..],
            &codes,
            writer,
        );
        _ = try stream.writeTokens(
            allocator,
            tokens.items,
            &codes,
            0,
            writer,
        );
    }

    // -----------------------------------------------------------------------
    // Interval tree (ComputePatchTree / GetPatchesForRow)
    // -----------------------------------------------------------------------

    fn computePatchTree(self: *PatchDictionary) void {
        const allocator = self.allocator;
        self.patch_tree.clearRetainingCapacity();
        self.num_patches.clearRetainingCapacity();
        self.sorted_patches_y0.clearRetainingCapacity();
        self.sorted_patches_y1.clearRetainingCapacity();
        if (self.positions.items.len == 0) return;

        // Create a y-interval for each patch.
        const PatchInterval = struct { idx: usize, y0: usize, y1: usize };
        const intervals = allocator.alloc(
            PatchInterval,
            self.positions.items.len,
        ) catch return;
        defer allocator.free(intervals);
        for (self.positions.items, 0..) |*pos, i| {
            intervals[i] = .{
                .idx = i,
                .y0 = pos.y,
                .y1 = pos.y + self.ref_positions.items[pos.ref_pos_idx].ysize,
            };
        }
        const lessY0 = struct {
            fn f(_: void, a: PatchInterval, b: PatchInterval) bool {
                return a.y0 < b.y0;
            }
        }.f;
        const lessY1 = struct {
            fn f(_: void, a: PatchInterval, b: PatchInterval) bool {
                return a.y1 < b.y1;
            }
        }.f;

        // Count the number of patches for each row.
        std.sort.pdq(
            PatchInterval,
            intervals,
            {},
            lessY1,
        );
        const max_y1 = intervals[intervals.len - 1].y1;
        self.num_patches.appendNTimes(
            allocator,
            0,
            max_y1,
        ) catch return;
        for (intervals) |iv| {
            for (iv.y0..iv.y1) |y| self.num_patches.items[y] += 1;
        }
        const root: PatchTreeNode = .{
            .start = 0,
            .num = intervals.len,
        };
        self.patch_tree.append(allocator, root) catch return;
        var next: usize = 0;
        while (next < self.patch_tree.items.len) : (next += 1) {
            const node = &self.patch_tree.items[next];
            const start = node.start;
            const end = node.start + node.num;
            // Choose the y_center for this node to be the median of
            // interval starts.
            std.sort.pdq(
                PatchInterval,
                intervals[start..end],
                {},
                lessY0,
            );
            const middle_idx = start + node.num / 2;
            node.y_center = intervals[middle_idx].y0;
            // Divide the intervals in [start, end) into three groups:
            //  * completely right of y_center: [right_start, end)
            //  * overlapping y_center: [left_end, right_start)
            //  * completely left of y_center: [start, left_end)
            var right_start = middle_idx;
            while (right_start < end and
                intervals[right_start].y0 == node.y_center)
            {
                right_start += 1;
            }
            std.sort.pdq(
                PatchInterval,
                intervals[start..right_start],
                {},
                lessY1,
            );
            var left_end = right_start;
            while (left_end > start and
                intervals[left_end - 1].y1 > node.y_center)
            {
                left_end -= 1;
            }
            // Fill in sorted_patches_y0/y1 for the current node.
            node.num = right_start - left_end;
            node.start = self.sorted_patches_y0.items.len;
            var ii: isize = @as(isize, @intCast(right_start)) - 1;
            while (ii >= @as(isize, @intCast(left_end))) : (ii -= 1) {
                const iv = intervals[@intCast(ii)];
                self.sorted_patches_y1.append(
                    allocator,
                    .{ .y = iv.y1, .idx = iv.idx },
                ) catch return;
            }
            std.sort.pdq(
                PatchInterval,
                intervals[left_end..right_start],
                {},
                lessY0,
            );
            for (intervals[left_end..right_start]) |iv| {
                self.sorted_patches_y0.append(
                    allocator,
                    .{ .y = iv.y0, .idx = iv.idx },
                ) catch return;
            }
            // Create the left and right nodes (if not empty).
            node.left_child = -1;
            node.right_child = -1;
            if (left_end > start) {
                const left: PatchTreeNode = .{
                    .start = start,
                    .num = left_end - start,
                };
                self.patch_tree.items[next].left_child = @intCast(
                    self.patch_tree.items.len,
                );
                self.patch_tree.append(allocator, left) catch return;
            }
            if (right_start < end) {
                const right: PatchTreeNode = .{
                    .start = right_start,
                    .num = end - right_start,
                };
                self.patch_tree.items[next].right_child = @intCast(
                    self.patch_tree.items.len,
                );
                self.patch_tree.append(allocator, right) catch return;
            }
        }
    }

    /// Patch indices affecting row `y`, in patch order (sorted); appends to
    /// `out` (cleared first). Port of `GetPatchesForRow`.
    pub fn getPatchesForRow(
        self: *const PatchDictionary,
        allocator: Allocator,
        y: usize,
        out: *std.ArrayList(usize),
    ) !void {
        out.clearRetainingCapacity();
        if (y >= self.num_patches.items.len or
            self.num_patches.items[y] == 0) return;
        try out.ensureTotalCapacity(
            allocator,
            self.num_patches.items[y],
        );
        var tree_idx: isize = 0;
        while (tree_idx != -1) {
            const node = &self.patch_tree.items[@intCast(tree_idx)];
            if (y <= node.y_center) {
                for (0..node.num) |i| {
                    const p = &self.sorted_patches_y0.items[node.start + i];
                    if (y < p.y) break;
                    out.appendAssumeCapacity(p.idx);
                }
                tree_idx = if (y < node.y_center) node.left_child else -1;
            } else {
                for (0..node.num) |i| {
                    const p = &self.sorted_patches_y1.items[node.start + i];
                    if (y >= p.y) break;
                    out.appendAssumeCapacity(p.idx);
                }
                tree_idx = node.right_child;
            }
        }
        // Ensure that the relative order of patches that affect the same
        // pixels is preserved (important for non-kAdd blend modes).
        std.sort.pdq(
            usize,
            out.items,
            {},
            std.sort.asc(usize),
        );
    }

    // -----------------------------------------------------------------------
    // Application (AddOneRow + PerformBlending on decoded channel planes)
    // -----------------------------------------------------------------------

    /// Everything `applyToImage` needs to know about the frame layout.
    pub const ApplyInfo = struct {
        /// Decoded modular channels: `n_color` color planes then the extra
        /// channels (global transforms already undone).
        image: *Image,
        n_color: usize,
        /// Per-channel sample formats (color: metadata bit depth; extra
        /// channels: their own). Float channels carry bit patterns and
        /// blend through `int_to_float`/`float_to_int` (fp16 patches,
        /// libjxl `bits_per_sample <= 16` gate).
        formats: []const SampleFormat,
        ec_info: []const ExtraChannelInfo,
        /// The reference-frame store (the same one the dictionary decoded
        /// against).
        refs: *const RefStore,
    };

    /// Applies every patch to the decoded channel planes. Equivalent to
    /// libjxl's Patches render-pipeline stage: for each row, the row is
    /// converted to float (int: v/maxval, float: int_to_float of the bit
    /// pattern), every patch affecting it is blended with
    /// `performBlending` (in patch order), and the row is written back
    /// with the output conversion (int: clamp01, NearestInt, clamp;
    /// float: float_to_int — bit-exact for the lossless domain this codec
    /// targets, since residual + content is exactly the original sample).
    pub fn applyToImage(
        self: *const PatchDictionary,
        allocator: Allocator,
        info: ApplyInfo,
    ) !void {
        const image = info.image;
        const n_planes = image.channels.items.len;
        if (n_planes == 0) return;
        const ysize = image.channels.items[0].h;
        const xsize = image.channels.items[0].w;
        if (info.formats.len < n_planes) return error.InvalidPatchLayout;

        // Per-channel int<->float factors (SingleFromSingle's
        // `double factor = 1.0 / ((1u << bits) - 1)` narrowed to f32).
        const factors = try allocator.alloc(f32, n_planes);
        defer allocator.free(factors);
        for (0..n_planes) |c| {
            const bits: u32 = info.formats[c].bits;
            const maxval: i64 = (@as(i64, 1) << @intCast(bits)) - 1;
            factors[c] = @floatCast(1.0 / @as(f64, @floatFromInt(maxval)));
        }

        var has_alpha = false;
        for (info.ec_info) |*eci| {
            if (eci.type == .alpha) {
                has_alpha = true;
                break;
            }
        }

        // Scratch rows: the current row in float (`rows`) plus a per-patch
        // snapshot (`pre`). libjxl's PerformBlending reads every blend from
        // the *original* row (`bg`), writes a tmp and copies it back — so
        // each patch blends against the row state before that patch, never
        // against a half-updated alpha plane.
        const rows = try allocator.alloc([]f32, n_planes);
        defer {
            for (rows) |r| allocator.free(r);
            allocator.free(rows);
        }
        const pre = try allocator.alloc([]f32, n_planes);
        defer {
            for (pre) |r| allocator.free(r);
            allocator.free(pre);
        }
        for (0..n_planes) |c| {
            rows[c] = try allocator.alloc(f32, xsize);
            pre[c] = try allocator.alloc(f32, xsize);
        }
        var row_patches: std.ArrayList(usize) = .empty;
        defer row_patches.deinit(allocator);

        var y: usize = 0;
        while (y < ysize) : (y += 1) {
            try self.getPatchesForRow(
                self.allocator,
                y,
                &row_patches,
            );
            if (row_patches.items.len == 0) continue;

            // plane -> float for the affected row (int: v/maxval;
            // float: int_to_float of the bit pattern).
            for (0..n_planes) |c| {
                const ch_row = image.channels.items[c].row(y);
                const fmt = info.formats[c];
                if (fmt.floating) {
                    for (0..xsize) |x| {
                        rows[c][x] = sampleToFloat(ch_row[x], fmt);
                    }
                } else {
                    for (0..xsize) |x| {
                        rows[c][x] = @as(f32, @floatFromInt(ch_row[x])) * factors[c];
                    }
                }
            }

            for (row_patches.items) |pos_idx| {
                const blending_idx = pos_idx * self.blendings_stride;
                const pos = &self.positions.items[pos_idx];
                const ref_pos = &self.ref_positions.items[pos.ref_pos_idx];
                const by = pos.y;
                const bx = pos.x;
                const patch_xsize = ref_pos.xsize;
                std.debug.assert(y >= by and y < by + ref_pos.ysize);
                const iy = y - by;
                if (bx >= xsize) continue;
                const patch_x0 = bx;
                const patch_x1 = @min(bx + patch_xsize, xsize);
                if (patch_x1 <= patch_x0) continue;

                const ref_frame = &(info.refs[ref_pos.ref] orelse
                    return error.InvalidPatchReferenceFrame);
                if (ref_frame.planes.len < n_planes)
                    return error.InvalidPatchReferenceFrame;

                for (0..n_planes) |c| {
                    @memcpy(pre[c], rows[c]);
                }
                try performBlending(
                    pre,
                    ref_frame,
                    ref_pos.x0,
                    ref_pos.y0 + iy,
                    rows,
                    patch_x0,
                    patch_x1 - patch_x0,
                    info.n_color,
                    info.ec_info,
                    self.blendings.items[blending_idx],
                    self.blendings.items[blending_idx + 1 ..],
                    has_alpha,
                );
            }

            // float -> plane (output stage). Int: clamp01, NearestInt,
            // clamp. Float: float_to_int of the bit pattern — exact for
            // lossless files (residual + content reproduces the original
            // pattern), an error on exotic/lossy shapes (mirrors libjxl
            // float_to_int failing on unrepresentable values).
            for (0..n_planes) |c| {
                const ch_row = image.channels.items[c].row(y);
                const fmt = info.formats[c];
                if (fmt.floating) {
                    for (0..xsize) |x| {
                        ch_row[x] = try floatToSample(rows[c][x], fmt);
                    }
                } else {
                    const maxv: i64 = (@as(i64, 1) << @intCast(fmt.bits)) - 1;
                    for (0..xsize) |x| {
                        const v = rows[c][x];
                        const scaled = clamp01(v) * @as(f32, @floatFromInt(maxv));
                        const rounded = nearestEven(scaled);
                        ch_row[x] = @intCast(
                            std.math.clamp(rounded, 0, maxv),
                        );
                    }
                }
            }
        }
    }

    /// Applies every patch to raw float planes (the VarDCT patch path:
    /// the render-pipeline `PatchesStage` blends on the XYB float rows
    /// after gaborish and before the XYB->linear conversion). `planes`
    /// are `xsize * ysize` color-channel rows (X, Y, B), blended in place
    /// with `performBlending` exactly like `applyToImage` does for the
    /// modular domain.
    pub fn applyToFloatImage(
        self: *const PatchDictionary,
        allocator: Allocator,
        planes: [3][]f32,
        xsize: usize,
        ysize: usize,
        refs: *const RefStore,
    ) !void {
        const has_alpha = false; // no extra channels on the VarDCT subset
        const pre = try allocator.alloc([]f32, 3);
        defer {
            for (pre) |r| allocator.free(r);
            allocator.free(pre);
        }
        for (0..3) |c| {
            pre[c] = try allocator.alloc(f32, xsize);
        }
        var row_patches: std.ArrayList(usize) = .empty;
        defer row_patches.deinit(allocator);

        var rowViews: [3][]f32 = undefined;
        var y: usize = 0;
        while (y < ysize) : (y += 1) {
            try self.getPatchesForRow(
                self.allocator,
                y,
                &row_patches,
            );
            if (row_patches.items.len == 0) continue;
            rowViews[0] = planes[0][y * xsize ..][0..xsize];
            rowViews[1] = planes[1][y * xsize ..][0..xsize];
            rowViews[2] = planes[2][y * xsize ..][0..xsize];
            for (row_patches.items) |pos_idx| {
                const blending_idx = pos_idx * self.blendings_stride;
                const pos = &self.positions.items[pos_idx];
                const ref_pos = &self.ref_positions.items[pos.ref_pos_idx];
                const by = pos.y;
                const bx = pos.x;
                const patch_xsize = ref_pos.xsize;
                std.debug.assert(y >= by and y < by + ref_pos.ysize);
                const iy = y - by;
                if (bx >= xsize) continue;
                const patch_x0 = bx;
                const patch_x1 = @min(bx + patch_xsize, xsize);
                if (patch_x1 <= patch_x0) continue;

                const ref_frame = &(refs[ref_pos.ref] orelse
                    return error.InvalidPatchReferenceFrame);
                if (ref_frame.planes.len < 3)
                    return error.InvalidPatchReferenceFrame;

                for (0..3) |c| {
                    @memcpy(pre[c], rowViews[c]);
                }
                try performBlending(
                    pre,
                    ref_frame,
                    ref_pos.x0,
                    ref_pos.y0 + iy,
                    &rowViews,
                    patch_x0,
                    patch_x1 - patch_x0,
                    3,
                    &.{},
                    self.blendings.items[blending_idx],
                    self.blendings.items[blending_idx + 1 ..],
                    has_alpha,
                );
            }
        }
    }

    /// Encoder-side subtraction (PatchDictionaryEncoder::SubtractFrom).
    /// The patch content is read from the reference store (`self.refs`,
    /// float planes — libjxl reads `reference_frames_` the same way) and
    /// subtracted from the frame channels: integer channels reconstruct
    /// the exact integer sample (clamp01, x maxval, round-to-nearest-even
    /// — no clamps, the modular domain is signed), float channels do the
    /// arithmetic on the decoded values and convert back with
    /// `float_to_int` (libjxl subtracts on the float image and lets
    /// `float_to_int` of the main frame produce the pattern — failing
    /// there on unrepresentable residuals, exactly like the `try` here).
    /// `formats` covers every plane of `image` (color: metadata bit
    /// depth; extra channels: their own).
    pub fn subtractFrom(
        self: *const PatchDictionary,
        image: *Image,
        n_color: usize,
        formats: []const SampleFormat,
    ) !void {
        if (image.channels.items.len == 0) return;
        if (formats.len < image.channels.items.len)
            return error.InvalidPatchLayout;
        const ysize = image.channels.items[0].h;
        var row_patches: std.ArrayList(usize) = .empty;
        defer row_patches.deinit(self.allocator);

        var y: usize = 0;
        while (y < ysize) : (y += 1) {
            try self.getPatchesForRow(
                self.allocator,
                y,
                &row_patches,
            );
            for (row_patches.items) |pos_idx| {
                const blending_idx = pos_idx * self.blendings_stride;
                const pos = &self.positions.items[pos_idx];
                const ref_pos = &self.ref_positions.items[pos.ref_pos_idx];
                const mode = self.blendings.items[blending_idx].mode;
                if (mode != .add and mode != .none and mode != .replace) {
                    return error.UnsupportedPatchSubtraction;
                }
                const by = pos.y;
                const bx = pos.x;
                const xsize = ref_pos.xsize;
                std.debug.assert(y >= by and y < by + ref_pos.ysize);
                const iy = y - by;
                const ref_frame = &(self.refs.?[ref_pos.ref] orelse
                    return error.InvalidPatchReferenceFrame);
                if (ref_frame.planes.len < n_color)
                    return error.InvalidPatchReferenceFrame;
                for (0..xsize) |ix| {
                    var c: usize = 0;
                    while (c < n_color) : (c += 1) {
                        const dst = image.channels.items[c].row(y);
                        // libjxl: one blending info per color group — the
                        // color channels all follow blendings[blending_idx].
                        const mode_c = self.blendings.items[blending_idx].mode;
                        if (mode_c == .none) continue;
                        const fmt = formats[c];
                        // The reference planes hold the decoded patch
                        // content floats (integer-sourced v/maxval or
                        // int_to_float of the pattern), negative where the
                        // patch is darker than its background.
                        const src_f = ref_frame.planes[c][
                            (ref_pos.y0 + iy) *
                                ref_frame.xsize + ref_pos.x0 + ix
                        ];
                        if (fmt.floating) {
                            // Float channels: pattern -> value, subtract,
                            // value -> pattern (float_to_int errors on
                            // unrepresentable residuals, like libjxl).
                            const img_f = sampleToFloat(
                                dst[bx + ix],
                                fmt,
                            );
                            if (mode_c == .add) {
                                dst[bx + ix] = try floatToSample(
                                    img_f - src_f,
                                    fmt,
                                );
                            } else if (mode_c == .replace) {
                                dst[bx + ix] = 0;
                            }
                        } else {
                            const maxv: i64 = (@as(i64, 1) << @intCast(fmt.bits)) - 1;
                            const scaled = src_f * @as(f32, @floatFromInt(maxv));
                            const src: i64 = nearestEven(scaled);
                            if (mode_c == .add) {
                                dst[bx + ix] -= @intCast(src);
                            } else if (mode_c == .replace) {
                                dst[bx + ix] = 0;
                            }
                        }
                    }
                }
            }
        }
    }
};

// ---------------------------------------------------------------------------
// PerformBlending (blending.cc / alpha.cc) — the patch blend math
// ---------------------------------------------------------------------------

inline fn clamp01(v: f32) f32 {
    return @min(@max(v, 0.0), 1.0);
}

/// Round-to-nearest, ties-to-even (libjxl `NearestInt`).
pub fn nearestEven(v: f32) i32 {
    if (v >= 2147483520.0) return std.math.maxInt(i32);
    if (v <= -2147483520.0) return std.math.minInt(i32);
    const f = @floor(v);
    const diff = v - f;
    if (diff > 0.5) return @intFromFloat(f + 1);
    if (diff < 0.5) return @intFromFloat(f);
    const fi: i32 = @intFromFloat(f);
    return if (@mod(fi, 2) == 0) fi else fi + 1;
}

/// `PerformAlphaWeightedAdd` (alpha.cc): out = bg + fg * fga, with the
/// `fg == fga` alias shortcut (out = bg).
fn alphaWeightedAdd(
    bg: f32,
    fg: f32,
    fga: f32,
    is_alpha_plane: bool,
    clamp: bool,
) f32 {
    if (is_alpha_plane) return bg;
    return bg + fg * (if (clamp) clamp01(fga) else fga);
}

/// Single-plane `PerformAlphaBlending` (alpha.cc), non-aliased form;
/// `out_a` receives the new alpha.
fn alphaBlendEc(
    bg: f32,
    bg_a: f32,
    fg: f32,
    fg_a: f32,
    premultiplied: bool,
    clamp: bool,
    out_a: *f32,
) f32 {
    const fa = if (clamp) clamp01(fg_a) else fg_a;
    if (premultiplied) {
        out_a.* = 1.0 - (1.0 - fa) * (1.0 - bg_a);
        return fg + bg * (1.0 - fa);
    } else {
        const new_a = 1.0 - (1.0 - fa) * (1.0 - bg_a);
        out_a.* = new_a;
        const rnew_a = if (new_a > 0) 1.0 / new_a else 0.0;
        return (fg * fa + bg * bg_a * (1.0 - fa)) * rnew_a;
    }
}

/// Port of `PerformBlending` (blending.cc) for one row segment.
/// `bg` are the current (already partially patched) full-width float rows;
/// `ref` is the patch's reference frame, `(ref_x, ref_y)` the source
/// position; the segment `[x0, x0 + xsize)` of every plane is blended into
/// `out` (which the caller aliases with `bg`, like libjxl's in-place
/// `inout` rows). Plane layout: `[0, n_color)` color, `[n_color, ...)` the
/// extra channels.
fn performBlending(
    bg: [][]f32,
    ref: *const ReferenceFrame,
    ref_x: usize,
    ref_y: usize,
    out: [][]f32,
    x0: usize,
    xsize: usize,
    n_color: usize,
    ec_info: []const ExtraChannelInfo,
    color_blending: PatchBlending,
    ec_blending: []const PatchBlending,
    has_alpha: bool,
) !void {
    const num_ec = ec_info.len;
    if (ec_blending.len < num_ec) return error.InvalidPatchLayout;
    const ref_w = ref.xsize;
    if (ref_y >= ref.ysize or ref_x + xsize > ref_w)
        return error.InvalidPatchReferencePosition;

    // Blend extra channels first so that we use the pre-blending alpha.
    for (0..num_ec) |i| {
        const info = &ec_blending[i];
        const plane = n_color + i;
        const a_plane = n_color + info.alpha_channel;
        const is_alpha_plane = (plane == a_plane);
        switch (info.mode) {
            .add => {
                for (0..xsize) |x| {
                    out[plane][x0 + x] = bg[plane][x0 + x] +
                        ref.planes[plane][ref_y * ref_w + ref_x + x];
                }
            },
            .blend_above, .blend_below => {
                const premul = ec_info[info.alpha_channel].alpha_associated;
                // kBlendAbove: bottom = bg, top = fg (the patch);
                // kBlendBelow: bottom = fg, top = bg.
                const swap = info.mode == .blend_below;
                for (0..xsize) |x| {
                    const b = bg[plane][x0 + x];
                    const ba = bg[a_plane][x0 + x];
                    const f = ref.planes[plane][ref_y * ref_w + ref_x + x];
                    const fa = ref.planes[a_plane][ref_y * ref_w + ref_x + x];
                    const bottom_v = if (swap) f else b;
                    const bottom_a = if (swap) fa else ba;
                    const top_v = if (swap) b else f;
                    const top_a = if (swap) ba else fa;
                    if (is_alpha_plane) {
                        // The alpha channel blending with itself: only the
                        // new alpha (alpha.cc `bg == bga && fg == fga`).
                        const tfa = if (info.clamp) clamp01(top_a) else top_a;
                        out[plane][x0 + x] = 1.0 - (1.0 - tfa) * (1.0 - bottom_a);
                    } else {
                        var out_a: f32 = undefined;
                        out[plane][x0 + x] = alphaBlendEc(
                            bottom_v,
                            bottom_a,
                            top_v,
                            top_a,
                            premul,
                            info.clamp,
                            &out_a,
                        );
                    }
                }
            },
            .alpha_weighted_add_above, .alpha_weighted_add_below => {
                // Both directions weight with the *patch* alpha
                // (blending.cc passes fg[3+alpha] in both cases).
                const swap = info.mode == .alpha_weighted_add_below;
                for (0..xsize) |x| {
                    const b = bg[plane][x0 + x];
                    const f = ref.planes[plane][ref_y * ref_w + ref_x + x];
                    const fa = ref.planes[a_plane][ref_y * ref_w + ref_x + x];
                    const bottom = if (swap) f else b;
                    const top = if (swap) b else f;
                    out[plane][x0 + x] = alphaWeightedAdd(
                        bottom,
                        top,
                        fa,
                        is_alpha_plane,
                        info.clamp,
                    );
                }
            },
            .mul => {
                for (0..xsize) |x| {
                    const b = bg[plane][x0 + x];
                    const f = ref.planes[plane][ref_y * ref_w + ref_x + x];
                    out[plane][x0 + x] = if (info.clamp) b * clamp01(f) else b * f;
                }
            },
            .replace => {
                for (0..xsize) |x| {
                    out[plane][x0 + x] = ref.planes[plane][ref_y * ref_w + ref_x + x];
                }
            },
            .none => {
                for (0..xsize) |x| {
                    out[plane][x0 + x] = bg[plane][x0 + x];
                }
            },
        }
    }

    const alpha = color_blending.alpha_channel;
    const a_plane = n_color + alpha;

    switch (color_blending.mode) {
        .add => {
            for (0..n_color) |p| {
                for (0..xsize) |x| {
                    out[p][x0 + x] = bg[p][x0 + x] +
                        ref.planes[p][ref_y * ref_w + ref_x + x];
                }
            }
        },
        .alpha_weighted_add_above, .alpha_weighted_add_below => {
            if (has_alpha) {
                // add_weighted(bottom, top) = bottom + top * top_alpha;
                // above: bottom = bg, top = fg; below: bottom = fg,
                // top = bg (so the *background* alpha weights it).
                const swap = color_blending.mode == .alpha_weighted_add_below;
                for (0..n_color) |p| {
                    for (0..xsize) |x| {
                        const b = bg[p][x0 + x];
                        const f = ref.planes[p][ref_y * ref_w + ref_x + x];
                        const top_a = if (swap)
                            bg[a_plane][x0 + x]
                        else
                            ref.planes[a_plane][ref_y * ref_w + ref_x + x];
                        const bottom = if (swap) f else b;
                        const top = if (swap) b else f;
                        out[p][x0 + x] = bottom + top *
                            (if (color_blending.clamp)
                                clamp01(top_a)
                            else
                                top_a);
                    }
                }
            } else {
                for (0..n_color) |p| {
                    for (0..xsize) |x| {
                        out[p][x0 + x] = bg[p][x0 + x] +
                            ref.planes[p][ref_y * ref_w + ref_x + x];
                    }
                }
            }
        },
        .blend_above, .blend_below => {
            if (has_alpha) {
                const premul = ec_info[alpha].alpha_associated;
                const swap = color_blending.mode == .blend_below;
                for (0..n_color) |p| {
                    for (0..xsize) |x| {
                        const b = bg[p][x0 + x];
                        const ba = bg[a_plane][x0 + x];
                        const f = ref.planes[p][ref_y * ref_w + ref_x + x];
                        const fa = ref.planes[a_plane][ref_y * ref_w + ref_x + x];
                        const bottom_v = if (swap) f else b;
                        const bottom_a = if (swap) fa else ba;
                        const top_v = if (swap) b else f;
                        const top_a = if (swap) ba else fa;
                        var out_a: f32 = undefined;
                        const v = alphaBlendEc(
                            bottom_v,
                            bottom_a,
                            top_v,
                            top_a,
                            premul,
                            color_blending.clamp,
                            &out_a,
                        );
                        out[p][x0 + x] = v;
                        // The 4-plane form writes the blended alpha to the
                        // alpha output plane (color blend runs last).
                        out[a_plane][x0 + x] = out_a;
                    }
                }
            } else {
                // No alpha: copy the patch (kBlendAbove and kBlendBelow
                // both `copy(fg)`).
                for (0..n_color) |p| {
                    for (0..xsize) |x| {
                        out[p][x0 + x] = ref.planes[p][ref_y * ref_w + ref_x + x];
                    }
                }
            }
        },
        .mul => {
            for (0..n_color) |p| {
                for (0..xsize) |x| {
                    const b = bg[p][x0 + x];
                    const f = ref.planes[p][ref_y * ref_w + ref_x + x];
                    out[p][x0 + x] = if (color_blending.clamp)
                        b * clamp01(f)
                    else
                        b * f;
                }
            }
        },
        .replace => {
            for (0..n_color) |p| {
                for (0..xsize) |x| {
                    out[p][x0 + x] = ref.planes[p][ref_y * ref_w + ref_x + x];
                }
            }
        },
        .none => {
            for (0..n_color) |p| {
                for (0..xsize) |x| {
                    out[p][x0 + x] = bg[p][x0 + x];
                }
            }
        },
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "patch dictionary bitstream roundtrip" {
    const alloc = testing.allocator;
    var pd = PatchDictionary.init(alloc);
    defer pd.deinit();

    const positions = [_]PatchPosition{
        .{ .x = 10, .y = 5, .ref_pos_idx = 0 },
        .{ .x = 40, .y = 7, .ref_pos_idx = 0 },
        .{ .x = 3, .y = 100, .ref_pos_idx = 1 },
    };
    const ref_positions = [_]PatchReferencePosition{
        .{
            .ref = 3,
            .x0 = 0,
            .y0 = 0,
            .xsize = 6,
            .ysize = 8,
        },
        .{
            .ref = 3,
            .x0 = 10,
            .y0 = 2,
            .xsize = 4,
            .ysize = 4,
        },
    };
    const blendings = [_]PatchBlending{
        .{ .mode = .add }, .{ .mode = .none },
        .{ .mode = .add }, .{ .mode = .none },
        .{ .mode = .add }, .{ .mode = .none },
    };
    try pd.setPositions(
        &positions,
        &ref_positions,
        &blendings,
        2,
    );
    try testing.expect(pd.hasAny());
    try testing.expectEqual(
        @as(u8, 1 << 3),
        pd.getReferences(),
    );

    var w = BitWriter.init(alloc);
    defer w.deinit();
    try pd.encode(&w);
    w.zeroPadToByte();

    const bytes = try w.takeBytes();
    defer alloc.free(bytes);

    var refs: RefStore = .{ null, null, null, null };
    const plane_len = 16 * 16;
    const planes = try alloc.alloc(f32, plane_len * 4);
    defer alloc.free(planes);
    @memset(planes, 0);
    const ref_planes = try alloc.alloc([]f32, 4);
    defer alloc.free(ref_planes);
    for (0..4) |c| ref_planes[c] = planes[c * plane_len .. (c + 1) * plane_len];
    refs[3] = .{
        .planes = ref_planes,
        .xsize = 16,
        .ysize = 16,
        .before_color_transform = true,
    };

    var pd2 = PatchDictionary.init(alloc);
    defer pd2.deinit();
    var br = BitReader.init(bytes);
    var uses_ec = false;
    try pd2.decode(
        &br,
        128,
        128,
        1,
        &refs,
        &uses_ec,
    );
    try testing.expect(!uses_ec);

    try testing.expectEqual(
        pd.positions.items.len,
        pd2.positions.items.len,
    );
    try testing.expectEqual(
        pd.ref_positions.items.len,
        pd2.ref_positions.items.len,
    );
    for (pd.positions.items, pd2.positions.items) |a, b| {
        try testing.expectEqual(a.x, b.x);
        try testing.expectEqual(a.y, b.y);
        try testing.expectEqual(a.ref_pos_idx, b.ref_pos_idx);
    }
    for (pd.ref_positions.items, pd2.ref_positions.items) |a, b| {
        try testing.expectEqual(a.ref, b.ref);
        try testing.expectEqual(a.x0, b.x0);
        try testing.expectEqual(a.y0, b.y0);
        try testing.expectEqual(a.xsize, b.xsize);
        try testing.expectEqual(a.ysize, b.ysize);
    }
    for (pd.blendings.items, pd2.blendings.items) |a, b| {
        try testing.expectEqual(a.mode, b.mode);
        try testing.expectEqual(a.alpha_channel, b.alpha_channel);
        try testing.expectEqual(a.clamp, b.clamp);
    }
}

test "patch tree: single and overlapping patches" {
    const alloc = testing.allocator;
    var pd = PatchDictionary.init(alloc);
    defer pd.deinit();

    const positions = [_]PatchPosition{
        .{ .x = 0, .y = 0, .ref_pos_idx = 0 },
        .{ .x = 4, .y = 1, .ref_pos_idx = 0 },
        .{ .x = 8, .y = 3, .ref_pos_idx = 0 },
    };
    const ref_positions = [_]PatchReferencePosition{
        .{
            .ref = 3,
            .x0 = 0,
            .y0 = 0,
            .xsize = 4,
            .ysize = 2,
        },
    };
    const blendings = [_]PatchBlending{
        .{ .mode = .add },
    };
    try pd.setPositions(
        &positions,
        &ref_positions,
        &blendings,
        1,
    );

    var out: std.ArrayList(usize) = .empty;
    defer out.deinit(alloc);
    try pd.getPatchesForRow(alloc, 0, &out);
    try testing.expectEqualSlices(usize, &.{0}, out.items);
    try pd.getPatchesForRow(alloc, 1, &out);
    try testing.expectEqualSlices(usize, &.{ 0, 1 }, out.items);
    try pd.getPatchesForRow(alloc, 2, &out);
    try testing.expectEqualSlices(usize, &.{1}, out.items);
    try pd.getPatchesForRow(alloc, 3, &out);
    try testing.expectEqualSlices(usize, &.{2}, out.items);
    try pd.getPatchesForRow(alloc, 4, &out);
    try testing.expectEqualSlices(usize, &.{2}, out.items);
    try pd.getPatchesForRow(alloc, 5, &out);
    try testing.expectEqualSlices(usize, &.{}, out.items);
}

test "patch dictionary rejects post-color-transform references" {
    const alloc = testing.allocator;
    var pd = PatchDictionary.init(alloc);
    defer pd.deinit();
    const positions = [_]PatchPosition{
        .{ .x = 0, .y = 0, .ref_pos_idx = 0 },
    };
    const ref_positions = [_]PatchReferencePosition{
        .{
            .ref = 2,
            .x0 = 0,
            .y0 = 0,
            .xsize = 1,
            .ysize = 1,
        },
    };
    const blendings = [_]PatchBlending{
        .{ .mode = .add },
    };
    try pd.setPositions(
        &positions,
        &ref_positions,
        &blendings,
        1,
    );
    var w = BitWriter.init(alloc);
    defer w.deinit();
    try pd.encode(&w);
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer alloc.free(bytes);

    const planes = try alloc.alloc(f32, 4);
    defer alloc.free(planes);
    @memset(planes, 0);
    const ref_planes = try alloc.alloc([]f32, 1);
    defer alloc.free(ref_planes);
    ref_planes[0] = planes;
    var refs: RefStore = .{ null, null, null, null };
    refs[2] = .{
        .planes = ref_planes,
        .xsize = 2,
        .ysize = 2,
        .before_color_transform = false,
    };
    var pd2 = PatchDictionary.init(alloc);
    defer pd2.deinit();
    var br = BitReader.init(bytes);
    var uses_ec = false;
    try testing.expectError(
        error.InvalidPatchReferenceFrame,
        pd2.decode(
            &br,
            8,
            8,
            0,
            &refs,
            &uses_ec,
        ),
    );
}

test "applyToImage: kAdd restores subtracted content" {
    const alloc = testing.allocator;
    // Reference frame 16x8: plane values = 5 at (x0..x0+4, y0..y0+4).
    const ref_w = 16;
    const ref_h = 8;
    const n_planes = 3;
    const planes = try alloc.alloc(f32, n_planes * ref_w * ref_h);
    defer alloc.free(planes);
    @memset(planes, 0);
    const ref_planes = try alloc.alloc([]f32, n_planes);
    defer alloc.free(ref_planes);
    for (0..n_planes) |c| {
        ref_planes[c] = planes[c * ref_w * ref_h .. (c + 1) * ref_w * ref_h];
        for (0..ref_h) |y| {
            for (0..ref_w) |x| {
                ref_planes[c][y * ref_w + x] = if (x >= 2 and x < 6 and y >= 1 and y < 5)
                    5.0 / 255.0
                else
                    0;
            }
        }
    }
    var refs: RefStore = .{ null, null, null, null };
    refs[3] = .{
        .planes = ref_planes,
        .xsize = ref_w,
        .ysize = ref_h,
        .before_color_transform = true,
    };

    // Main image 32x8: background 100 everywhere (patch content was
    // subtracted from (10..14, 2..6): values 95 remain there... simulate
    // by writing 95 in the patch area and 100 elsewhere).
    var image = Image.create(alloc);
    defer image.deinit();
    const W = 32;
    const H = 8;
    for (0..n_planes) |_| {
        _ = try image.addChannel(W, H);
    }
    for (0..H) |y| {
        for (0..W) |x| {
            for (0..n_planes) |c| {
                const in_patch = x >= 10 and x < 14 and y >= 2 and y < 6;
                image.channels.items[c].row(y)[x] = if (in_patch) 95 else 100;
            }
        }
    }

    var pd = PatchDictionary.init(alloc);
    defer pd.deinit();
    const positions = [_]PatchPosition{
        .{ .x = 10, .y = 2, .ref_pos_idx = 0 },
    };
    const ref_positions = [_]PatchReferencePosition{
        .{
            .ref = 3,
            .x0 = 2,
            .y0 = 1,
            .xsize = 4,
            .ysize = 4,
        },
    };
    const blendings = [_]PatchBlending{
        .{ .mode = .add },
    };
    try pd.setPositions(
        &positions,
        &ref_positions,
        &blendings,
        1,
    );

    const formats = [_]SampleFormat{
        .{ .bits = 8 },
        .{ .bits = 8 },
        .{ .bits = 8 },
    };
    try pd.applyToImage(alloc, .{
        .image = &image,
        .n_color = 3,
        .formats = &formats,
        .ec_info = &.{},
        .refs = &refs,
    });

    for (0..H) |y| {
        for (0..W) |x| {
            for (0..n_planes) |c| {
                const in_patch = x >= 10 and x < 14 and y >= 2 and y < 6;
                const want: i32 = if (in_patch) 95 + 5 else 100;
                try testing.expectEqual(
                    want,
                    image.channels.items[c].row(y)[x],
                );
            }
        }
    }
}
