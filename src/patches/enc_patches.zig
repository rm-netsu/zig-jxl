//! Patch detection, port of libjxl `enc_patch_dictionary.cc`
//! (`FindTextLikePatches` + the bin-packing tail of
//! `FindBestPatchDictionary`).
//!
//! The heuristic looks for "text-like" content: naturally aligned 4x4
//! blocks of equal color (screenshot-like areas) seed a background flood
//! fill (BFS over "similar enough" pixels, limited distance from the
//! seed); the connected components of the remaining pixels are candidate
//! glyphs — kept when their border background is uniform, they are small
//! (<= 32x32), their quantized difference from the background peaks above
//! the noise floor and the same quantized patch occurs at least twice.
//! Surviving patches are bin-packed into a small reference image.
//!
//! Input samples are integer channel planes; the thresholds run on the
//! [0,1] float domain exactly like libjxl (`v * (1.0f / maxval)`, the
//! `ConvertFromExternal` scale).

const std = @import("std");

const common = @import("../modular/common.zig");
const patches_mod = @import("dictionary.zig");
const float_convert = @import("../base/float_convert.zig");
const bit_io = @import("../base/bit_io.zig");
const fields_ext = @import("../header/fields_ext.zig");

const Allocator = std.mem.Allocator;
const pixel_type = common.pixel_type;
const SampleFormat = patches_mod.SampleFormat;

/// libjxl `kMaxPatchSize` (enc_patch_dictionary.h).
pub const kMaxPatchSize: usize = 32;

/// cjxl `--patches` (libjxl `cparams.patches`).
pub const PatchOverride = enum { default, off, on };

/// libjxl `ApplyOverride`.
pub fn applyOverride(o: PatchOverride, default: bool) bool {
    return switch (o) {
        .on => true,
        .off => false,
        .default => default,
    };
}

pub const XY = struct { x: u32, y: u32 };

/// Port of `QuantizedPatch` (enc_patch_dictionary.h): the quantized
/// (dedup/comparison) pixels plus the exact integer difference from the
/// background (libjxl keeps float `fpixels`; for integer samples the
/// modular patch frame stores exactly these integers — see
/// `findTextLikePatches`).
pub const QuantizedPatch = struct {
    xsize: usize = 0,
    ysize: usize = 0,
    /// Per-channel quantized samples (comparison domain).
    pixels: [3][kMaxPatchSize * kMaxPatchSize]i8 = undefined,
    /// Per-channel exact integer differences (the reference image
    /// content).
    fpixels: [3][kMaxPatchSize * kMaxPatchSize]i32 = undefined,

    pub fn eql(a: *const QuantizedPatch, b: *const QuantizedPatch) bool {
        if (a.xsize != b.xsize or a.ysize != b.ysize) return false;
        const n = a.xsize * a.ysize;
        for (0..3) |c| {
            if (!std.mem.eql(i8, a.pixels[c][0..n], b.pixels[c][0..n]))
                return false;
        }
        return true;
    }

    /// libjxl `operator<` (memcmp semantics: unsigned byte compare).
    pub fn less(a: *const QuantizedPatch, b: *const QuantizedPatch) bool {
        if (a.xsize != b.xsize) return a.xsize < b.xsize;
        if (a.ysize != b.ysize) return a.ysize < b.ysize;
        const n = a.xsize * a.ysize;
        for (0..3) |c| {
            const ca = std.mem.sliceAsBytes(a.pixels[c][0..n]);
            const cb = std.mem.sliceAsBytes(b.pixels[c][0..n]);
            switch (std.mem.order(u8, ca, cb)) {
                .lt => return true,
                .gt => return false,
                .eq => {},
            }
        }
        return false;
    }
};

/// Pair (patch, vector of occurrences) — libjxl `PatchInfo`.
pub const PatchInfo = struct {
    patch: QuantizedPatch,
    positions: std.ArrayList(XY) = .empty,

    pub fn deinit(self: *PatchInfo, allocator: Allocator) void {
        self.positions.deinit(allocator);
    }

    /// std::pair operator<: the patch first, then the positions
    /// lexicographically.
    fn less(a: *const PatchInfo, b: *const PatchInfo) bool {
        if (a.patch.less(&b.patch)) return true;
        if (b.patch.less(&a.patch)) return false;
        const n = @min(a.positions.items.len, b.positions.items.len);
        for (0..n) |i| {
            const pa = a.positions.items[i];
            const pb = b.positions.items[i];
            if (pa.x != pb.x) return pa.x < pb.x;
            if (pa.y != pb.y) return pa.y < pb.y;
        }
        return a.positions.items.len < b.positions.items.len;
    }
};

/// `PatchColorspaceInfo` (both libjxl branches): the non-XYB lossless
/// constants and the is_xyb=true VarDCT constants of
/// `enc_patch_dictionary.cc`.
const PatchColorspaceInfo = struct {
    is_xyb: bool = false,

    const kChannelDequantRgb = [3]f32{
        @as(f32, 20.0) / @as(f32, 255.0),
        @as(f32, 22.0) / @as(f32, 255.0),
        @as(f32, 20.0) / @as(f32, 255.0),
    };
    const kChannelWeightsRgb = [3]f32{
        @as(f32, 0.017) * @as(f32, 255.0),
        @as(f32, 0.02) * @as(f32, 255.0),
        @as(f32, 0.017) * @as(f32, 255.0),
    };
    const kChannelDequantXyb = [3]f32{ 0.01615, 0.08875, 0.1922 };
    const kChannelWeightsXyb = [3]f32{ 30.0, 3.0, 1.0 };

    fn dequant(self: PatchColorspaceInfo, c: usize) f32 {
        return if (self.is_xyb)
            kChannelDequantXyb[c]
        else
            kChannelDequantRgb[c];
    }

    fn weight(self: PatchColorspaceInfo, c: usize) f32 {
        return if (self.is_xyb)
            kChannelWeightsXyb[c]
        else
            kChannelWeightsRgb[c];
    }

    fn quantize(self: PatchColorspaceInfo, fval: f32, c: usize) i32 {
        const scaled = fval / self.dequant(c);
        const clamped = std.math.clamp(
            scaled,
            -32768.0,
            32767.0,
        );
        return @intFromFloat(@trunc(clamped));
    }

    fn isSimilarV(
        self: PatchColorspaceInfo,
        v1: [3]f32,
        v2: [3]f32,
        threshold: f32,
    ) bool {
        var distance: f32 = 0;
        for (0..3) |c| {
            distance += @abs(v1[c] - v2[c]) * self.weight(c);
        }
        return distance <= threshold;
    }
};

const kPatchSide: usize = 4;
const kSimilarThreshold: f32 = 0.8;
const kVerySimilarThreshold: f32 = 0.03;
const kHasSimilarThreshold: f32 = 0.03;
const kSearchRadius: i32 = 1;
const kDistanceLimit: i32 = 50;
const kMinPeak: i32 = 2;
const kHasSimilarRadius: i32 = 2;
const kMinPatchOccurrences: usize = 2;
const kMinMaxPatchSize: usize = 20;
const kBinPackingSlackness: f32 = 1.05;

/// The float view of a channel plane set (libjxl's `opsin`):
/// `n_planes` planes of `xsize * ysize` samples. Integer channels convert
/// per libjxl `ConvertFromExternal` (`v * (1.0f / maxval)` in f32); float
/// channels carry [bits]/[exp_bits] bit patterns and convert via
/// `int_to_float` (the actual sample values — libjxl runs the search on
/// the float Image3F either way). Gray images use one plane (libjxl's
/// three identical planes collapse to it). The VarDCT patch search passes
/// raw XYB float planes instead (`xyb_planes`).
const Opsin = struct {
    planes: []const []const pixel_type,
    n_color: usize,
    xsize: usize,
    ysize: usize,
    scale: f32,
    format: SampleFormat,
    /// VarDCT mode: three raw XYB float planes (`opsin` in the libjxl
    /// sense — the image *before* the gaborish pre-filter).
    xyb_planes: ?[]const []const f32 = null,

    inline fn chan(self: *const Opsin, c: usize) usize {
        return if (self.n_color == 1) 0 else c;
    }

    inline fn at(self: *const Opsin, c: usize, x: usize, y: usize) f32 {
        if (self.xyb_planes) |xp| {
            return xp[c][y * self.xsize + x];
        }
        const v = self.planes[self.chan(c)][y * self.xsize + x];
        if (self.format.floating) {
            return float_convert.intToFloat(
                v,
                @intCast(self.format.bits),
                @intCast(self.format.exp_bits),
            );
        }
        return @as(f32, @floatFromInt(v)) * self.scale;
    }

    inline fn colorAt(self: *const Opsin, x: usize, y: usize) [3]f32 {
        return .{
            self.at(0, x, y),
            self.at(1, x, y),
            self.at(2, x, y),
        };
    }
};

/// `is_same_color` (enc_patch_dictionary.cc): all three channels within
/// 1e-4 of the base color (same integer sample -> exactly equal floats,
/// so identical samples always match; on 16-bit images values up to 6
/// apart also match, exactly like libjxl).
fn isSameColorAt(o: *const Opsin, p: XY, c: [3]f32) bool {
    for (0..3) |i| {
        if (@abs(c[i] - o.at(i, p.x, p.y)) > 1e-4) return false;
    }
    return true;
}

/// `flat_patch`: a whole kPatchSide x kPatchSide block of the same color.
fn flatPatch(o: *const Opsin, ox: usize, oy: usize, base: [3]f32) bool {
    var iy: usize = 0;
    while (iy < kPatchSide) : (iy += 1) {
        var ix: usize = 0;
        while (ix < kPatchSide) : (ix += 1) {
            if (!isSameColorAt(
                o,
                .{ .x = @intCast(ox + ix), .y = @intCast(oy + iy) },
                base,
            )) {
                return false;
            }
        }
    }
    return true;
}

/// Port of `FindTextLikePatches` (the is_xyb=false branch used by the
/// lossless modular path). `channels` are the color planes (`n_color`
/// planes of `xsize * ysize` samples — integers, or the [bits]/[exp_bits]
/// bit patterns of float samples, selected by `format`; libjxl's gate
/// allows any `bits_per_sample <= 16` format, which includes binary16).
/// Returns the surviving patches (empty when none): deduplicated, >= 2
/// occurrences each, sorted by area descending (the
/// `FindBestPatchDictionary` order).
///
/// `fpixels` semantics per format: integers keep the exact integer
/// difference (round-half-away of `fval * maxval`); float formats keep
/// the raw f32 difference bits (`@bitCast`) — `packPatches` converts
/// them to bit patterns with `float_to_int`, which fails on
/// unrepresentable differences exactly where libjxl's patch-reference
/// frame encode fails.
pub fn findTextLikePatches(
    allocator: Allocator,
    channels: []const []const pixel_type,
    n_color: usize,
    xsize: usize,
    ysize: usize,
    format: SampleFormat,
    patches_override: PatchOverride,
) !std.ArrayList(PatchInfo) {
    const maxval: f32 = @floatFromInt((@as(u64, 1) << @intCast(format.bits)) - 1);
    const opsin = Opsin{
        .planes = channels,
        .n_color = n_color,
        .xsize = xsize,
        .ysize = ysize,
        .scale = @as(f32, 1.0) / maxval,
        .format = format,
    };
    return findTextLikePatchesImpl(
        allocator,
        opsin,
        PatchColorspaceInfo{},
        maxval,
        patches_override,
    );
}

/// VarDCT branch of `FindTextLikePatches` (is_xyb=true): the search runs
/// on the raw XYB float planes of the opsin image (before the gaborish
/// pre-filter), with the XYB similarity weights and quantization steps of
/// `PatchColorspaceInfo(/*is_xyb=*/true)`. `fpixels` keep the raw f32
/// difference bits; `packPatchesXyb` quantizes them into the reference
/// frame's modular planes.
pub fn findTextLikePatchesXyb(
    allocator: Allocator,
    xyb_planes: []const []const f32,
    xsize: usize,
    ysize: usize,
    patches_override: PatchOverride,
) !std.ArrayList(PatchInfo) {
    const opsin = Opsin{
        .planes = &.{},
        .n_color = 3,
        .xsize = xsize,
        .ysize = ysize,
        .scale = 0,
        .format = .{},
        .xyb_planes = xyb_planes,
    };
    return findTextLikePatchesImpl(
        allocator,
        opsin,
        PatchColorspaceInfo{ .is_xyb = true },
        1.0,
        patches_override,
    );
}

fn findTextLikePatchesImpl(
    allocator: Allocator,
    opsin: Opsin,
    pci: PatchColorspaceInfo,
    maxval: f32,
    patches_override: PatchOverride,
) !std.ArrayList(PatchInfo) {
    var info: std.ArrayList(PatchInfo) = .empty;
    errdefer {
        for (info.items) |*p| p.deinit(allocator);
        info.deinit(allocator);
    }
    if (patches_override == .off) return info;
    const format = opsin.format;
    const xsize = opsin.xsize;
    const ysize = opsin.ysize;
    const n_color = opsin.n_color;
    if (opsin.xyb_planes == null and
        (n_color == 0 or opsin.planes.len < n_color)) return info;

    const isSameColor = isSameColorAt;

    // Look for kPatchSide size squares, naturally aligned, that all have
    // the same pixel values.
    const seed_w = std.math.divCeil(
        usize,
        xsize,
        kPatchSide,
    ) catch 0;
    const seed_h = std.math.divCeil(
        usize,
        ysize,
        kPatchSide,
    ) catch 0;
    const is_screenshot_like = try allocator.alloc(
        u8,
        seed_w * seed_h,
    );
    defer allocator.free(is_screenshot_like);
    @memset(is_screenshot_like, 0);
    const pw = xsize / kPatchSide;
    const ph = ysize / kPatchSide;

    var num_seeds: usize = 0;
    const can_have_seeds = (pw >= 3) and (ph >= 3);
    if (can_have_seeds) {
        // libjxl runs process_row over RunOnPool(pool, 1, ph - 2) — the
        // half-open range [1, ph-2).
        var py: usize = 1;
        while (py + 2 < ph) : (py += 1) { // py in [1, ph-2)
            var px: usize = 1;
            while (px + 2 <= pw) : (px += 1) { // px <= pw - 2, inclusive
                const ox = px * kPatchSide;
                const oy = py * kPatchSide;
                const base = opsin.colorAt(ox, oy);
                if (!flatPatch(&opsin, ox, oy, base)) continue;
                var num_same: usize = 0;
                var y = (py - 1) * kPatchSide;
                while (y <= (py + 1) * kPatchSide) : (y += kPatchSide) {
                    var x = (px - 1) * kPatchSide;
                    while (x <= (px + 1) * kPatchSide) : (x += kPatchSide) {
                        num_same += @intFromBool(
                            isSameColor(
                                &opsin,
                                .{
                                    .x = @intCast(x),
                                    .y = @intCast(y),
                                },
                                base,
                            ),
                        );
                    }
                }
                // Too few equal pixels nearby.
                if (num_same < 8) continue;
                is_screenshot_like[py * seed_w + px] = 1;
                num_seeds += 1;
            }
        }
    }

    if (!applyOverride(patches_override, num_seeds > 0)) {
        return info;
    }

    // Search for "similar enough" pixels near the screenshot-like areas.
    const is_background = try allocator.alloc(u8, xsize * ysize);
    defer allocator.free(is_background);
    @memset(is_background, 0);
    const background = try allocator.alloc(f32, 3 * xsize * ysize);
    defer allocator.free(background);
    const bg_planes = [3][]f32{
        background[0 .. xsize * ysize],
        background[xsize * ysize .. 2 * xsize * ysize],
        background[2 * xsize * ysize .. 3 * xsize * ysize],
    };

    const QueueEntry = struct { cur: XY, src: XY };
    var queue: std.ArrayList(QueueEntry) = .empty;
    defer queue.deinit(allocator);
    try queue.ensureTotalCapacity(
        allocator,
        2 * num_seeds * kPatchSide * kPatchSide,
    );
    if (can_have_seeds) {
        var py: usize = 1;
        while (py + 1 < ph) : (py += 1) { // py < ph - 1
            var px: usize = 1;
            while (px + 1 < pw) : (px += 1) { // px < pw - 1
                if (is_screenshot_like[py * seed_w + px] == 0) continue;
                var y = py * kPatchSide;
                while (y < (py + 1) * kPatchSide) : (y += 1) {
                    var x = px * kPatchSide;
                    while (x < (px + 1) * kPatchSide) : (x += 1) {
                        const p = XY{
                            .x = @intCast(x),
                            .y = @intCast(y),
                        };
                        try queue.append(
                            allocator,
                            .{ .cur = p, .src = p },
                        );
                        is_background[p.y * xsize + p.x] = 1;
                    }
                }
            }
        }
    }
    var queue_front: usize = 0;
    while (queue_front < queue.items.len) : (queue_front += 1) {
        const cur = queue.items[queue_front].cur;
        const src = queue.items[queue_front].src;
        const src_color = opsin.colorAt(src.x, src.y);
        for (0..3) |c| {
            bg_planes[c][cur.y * xsize + cur.x] = src_color[c];
        }
        var dy: i32 = -kSearchRadius;
        while (dy <= kSearchRadius) : (dy += 1) {
            var dx: i32 = -kSearchRadius;
            while (dx <= kSearchRadius) : (dx += 1) {
                const nx = @as(i32, @intCast(cur.x)) + dx;
                const ny = @as(i32, @intCast(cur.y)) + dy;
                if (nx < 0 or ny < 0 or
                    nx >= @as(i32, @intCast(xsize)) or ny >= @as(i32, @intCast(ysize)))
                {
                    continue;
                }
                const next = XY{
                    .x = @intCast(nx),
                    .y = @intCast(ny),
                };
                if (is_background[next.y * xsize + next.x] != 0) continue;
                if (@abs(nx - @as(i32, @intCast(src.x))) +
                    @abs(ny - @as(i32, @intCast(src.y))) >
                    kDistanceLimit)
                {
                    continue;
                }
                // is_similar(src, next) with kSimilarThreshold.
                const v1 = opsin.colorAt(src.x, src.y);
                const v2 = opsin.colorAt(next.x, next.y);
                if (pci.isSimilarV(
                    v1,
                    v2,
                    kSimilarThreshold,
                )) {
                    try queue.append(
                        allocator,
                        .{ .cur = next, .src = src },
                    );
                    is_background[next.y * xsize + next.x] = 1;
                }
            }
        }
    }
    queue.clearRetainingCapacity();

    // Find small connected components outside the "similar enough" areas,
    // compute bounding boxes, and run heuristics to exclude some patches.
    const visited = try allocator.alloc(u8, xsize * ysize);
    defer allocator.free(visited);
    @memset(visited, 0);
    var stack: std.ArrayList(XY) = .empty;
    defer stack.deinit(allocator);
    for (0..ysize) |y| {
        for (0..xsize) |x| {
            if (is_background[y * xsize + x] != 0) continue;
            stack.clearRetainingCapacity();
            try stack.append(
                allocator,
                .{ .x = @intCast(x), .y = @intCast(y) },
            );
            var min_x = x;
            var max_x = x;
            var min_y = y;
            var max_y = y;
            var reference: XY = .{ .x = 0, .y = 0 };
            var found_border = false;
            var all_similar = true;
            while (stack.items.len > 0) {
                const cur = stack.pop().?;
                if (visited[cur.y * xsize + cur.x] != 0) continue;
                visited[cur.y * xsize + cur.x] = 1;
                if (cur.x < min_x) min_x = cur.x;
                if (cur.x > max_x) max_x = cur.x;
                if (cur.y < min_y) min_y = cur.y;
                if (cur.y > max_y) max_y = cur.y;
                var dy: i32 = -kSearchRadius;
                while (dy <= kSearchRadius) : (dy += 1) {
                    var dx: i32 = -kSearchRadius;
                    while (dx <= kSearchRadius) : (dx += 1) {
                        if (dx == 0 and dy == 0) continue;
                        const nx = @as(i32, @intCast(cur.x)) + dx;
                        const ny = @as(i32, @intCast(cur.y)) + dy;
                        if (nx < 0 or ny < 0 or
                            nx >= @as(i32, @intCast(xsize)) or
                            ny >= @as(i32, @intCast(ysize)))
                        {
                            continue;
                        }
                        const next = XY{
                            .x = @intCast(nx),
                            .y = @intCast(ny),
                        };
                        if (is_background[next.y * xsize + next.x] == 0) {
                            try stack.append(allocator, next);
                        } else {
                            if (!found_border) {
                                reference = next;
                                found_border = true;
                            } else {
                                // is_similar_b(next, reference) over the
                                // background image.
                                const v1 = .{
                                    bg_planes[0][reference.y * xsize + reference.x],
                                    bg_planes[1][reference.y * xsize + reference.x],
                                    bg_planes[2][reference.y * xsize + reference.x],
                                };
                                const v2 = .{
                                    bg_planes[0][next.y * xsize + next.x],
                                    bg_planes[1][next.y * xsize + next.x],
                                    bg_planes[2][next.y * xsize + next.x],
                                };
                                if (!pci.isSimilarV(
                                    v1,
                                    v2,
                                    kVerySimilarThreshold,
                                )) {
                                    all_similar = false;
                                }
                            }
                        }
                    }
                }
            }
            if (!found_border or !all_similar or
                max_x - min_x >= kMaxPatchSize or
                max_y - min_y >= kMaxPatchSize)
            {
                continue;
            }
            const ref = [3]f32{
                bg_planes[0][reference.y * xsize + reference.x],
                bg_planes[1][reference.y * xsize + reference.x],
                bg_planes[2][reference.y * xsize + reference.x],
            };
            var has_similar = false;
            {
                const iy_begin: usize = @intCast(
                    @max(0, @as(i32, @intCast(min_y)) - kHasSimilarRadius),
                );
                const iy_end: usize = @min(
                    max_y + @as(usize, @intCast(kHasSimilarRadius)) + 1,
                    ysize,
                );
                const ix_begin: usize = @intCast(
                    @max(0, @as(i32, @intCast(min_x)) - kHasSimilarRadius),
                );
                const ix_end: usize = @min(
                    max_x + @as(usize, @intCast(kHasSimilarRadius)) + 1,
                    xsize,
                );
                var iy = iy_begin;
                while (iy < iy_end) : (iy += 1) {
                    var ix = ix_begin;
                    while (ix < ix_end) : (ix += 1) {
                        if (pci.isSimilarV(
                            ref,
                            opsin.colorAt(ix, iy),
                            kHasSimilarThreshold,
                        )) {
                            has_similar = true;
                        }
                    }
                }
            }
            if (!has_similar) continue;

            var pi = PatchInfo{ .patch = .{} };
            pi.patch.xsize = max_x - min_x + 1;
            pi.patch.ysize = max_y - min_y + 1;
            var too_big = false;
            var too_small = true;
            const c_order = [_]usize{ 1, 0, 2 };
            for (c_order) |c| {
                var iy = min_y;
                while (iy <= max_y) : (iy += 1) {
                    var ix = min_x;
                    while (ix <= max_x) : (ix += 1) {
                        const offset = (iy - min_y) * pi.patch.xsize + ix - min_x;
                        const fval = opsin.at(c, ix, iy) - ref[c];
                        if (opsin.xyb_planes != null) {
                            // VarDCT: keep the raw XYB difference bits;
                            // packPatchesXyb quantizes them onto the
                            // reference frame's DC-quant grid.
                            pi.patch.fpixels[c][offset] = @bitCast(fval);
                        } else if (format.floating) {
                            // Float formats: keep the raw f32 difference
                            // bits; packPatches converts them to the
                            // [bits]/[exp_bits] pattern (failing there on
                            // unrepresentable values, like libjxl's
                            // patch-frame float_to_int).
                            pi.patch.fpixels[c][offset] = @bitCast(fval);
                        } else {
                            // Integer formats: the reference frame stores
                            // round-half-away (fval * maxval) — the exact
                            // integer difference.
                            const fquant = fval * maxval +
                                (if (fval < 0)
                                    @as(f32, -0.5)
                                else
                                    @as(f32, 0.5));
                            pi.patch.fpixels[c][offset] = @intFromFloat(@trunc(fquant));
                        }
                        const val = pci.quantize(fval, c);
                        const qval: i8 = @truncate(val);
                        pi.patch.pixels[c][offset] = qval;
                        too_big |= (val != qval);
                        too_small &= (val < kMinPeak) and (val > -kMinPeak);
                    }
                }
            }
            if (too_small or too_big) continue;
            try pi.positions.append(
                allocator,
                .{ .x = @intCast(min_x), .y = @intCast(min_y) },
            );
            try info.append(allocator, pi);
        }
    }

    if (info.items.len == 0) return info;

    // Remove duplicates (std::sort + run merge on the pair ordering).
    std.sort.pdq(
        PatchInfo,
        info.items,
        {},
        struct {
            fn f(_: void, a: PatchInfo, b: PatchInfo) bool {
                return a.less(&b);
            }
        }.f,
    );
    var unique: usize = 0;
    for (1..info.items.len) |i| {
        if (info.items[i].patch.eql(&info.items[unique].patch)) {
            try info.items[unique].positions.appendSlice(
                allocator,
                info.items[i].positions.items,
            );
            info.items[i].deinit(allocator);
        } else {
            if (info.items[unique].positions.items.len >= kMinPatchOccurrences) {
                unique += 1;
            }
            info.items[unique] = info.items[i];
        }
    }
    if (info.items[unique].positions.items.len >= kMinPatchOccurrences) {
        unique += 1;
    }
    info.shrinkRetainingCapacity(unique);

    var max_patch_size: usize = 0;
    for (info.items) |*p| {
        const pixels = p.patch.xsize * p.patch.ysize;
        if (pixels > max_patch_size) max_patch_size = pixels;
    }
    // don't use patches if all patches are smaller than this
    if (max_patch_size < kMinMaxPatchSize) {
        for (info.items) |*p| p.deinit(allocator);
        info.clearRetainingCapacity();
        return info;
    }

    return info;
}

/// The result of packing the detected patches into the reference image.
pub const PackedPatches = struct {
    /// Patch occurrences (position -> ref position index).
    positions: std.ArrayList(patches_mod.PatchPosition) = .empty,
    /// Per-unique-patch rectangles in the reference image.
    ref_positions: std.ArrayList(patches_mod.PatchReferencePosition) = .empty,
    /// Blending infos (kAdd on color, kNone on extra channels), stride
    /// `num_extra_channels + 1`.
    blendings: std.ArrayList(patches_mod.PatchBlending) = .empty,
    /// The bin-packed reference image: `n_color` planes of
    /// `ref_xsize * ref_ysize` integer samples.
    ref_planes: std.ArrayList([]i32) = .empty,
    ref_xsize: usize = 0,
    ref_ysize: usize = 0,

    pub fn deinit(self: *PackedPatches, allocator: Allocator) void {
        self.positions.deinit(allocator);
        self.ref_positions.deinit(allocator);
        self.blendings.deinit(allocator);
        for (self.ref_planes.items) |p| allocator.free(p);
        self.ref_planes.deinit(allocator);
    }
};

/// The tail of `FindBestPatchDictionary`: sort by area, bin-pack into a
/// grown-until-fits reference image, and emit positions / reference
/// positions / blendings. `info` is consumed (freed). Float `format`
/// converts the raw f32 difference bits kept in `fpixels` into the
/// [bits]/[exp_bits] bit patterns of the reference frame — the exact
/// point where libjxl's `float_to_int` fails on unrepresentable
/// differences (the patch-reference frame encode), so the error surfaces
/// here identically.
pub fn packPatches(
    allocator: Allocator,
    info: *std.ArrayList(PatchInfo),
    n_color: usize,
    num_extra_channels: usize,
    format: SampleFormat,
) !PackedPatches {
    var out = PackedPatches{};
    errdefer out.deinit(allocator);
    // On error the caller no longer owns `info` (this function owns it
    // from entry to the success tail that frees it).
    errdefer {
        for (info.items) |*p| p.deinit(allocator);
        info.deinit(allocator);
    }

    // Sort by area, descending.
    std.sort.pdq(
        PatchInfo,
        info.items,
        {},
        struct {
            fn f(_: void, a: PatchInfo, b: PatchInfo) bool {
                return a.patch.xsize * a.patch.ysize >
                    b.patch.xsize * b.patch.ysize;
            }
        }.f,
    );

    var max_x_size: usize = 0;
    var max_y_size: usize = 0;
    var total_pixels: usize = 0;
    for (info.items) |*p| {
        const pixels = p.patch.xsize * p.patch.ysize;
        if (max_x_size < p.patch.xsize) max_x_size = p.patch.xsize;
        if (max_y_size < p.patch.ysize) max_y_size = p.patch.ysize;
        total_pixels += pixels;
    }

    // Bin-packing & conversion of patches.
    var ref_xsize: usize = @intFromFloat(@max(
        @as(f32, @floatFromInt(max_x_size)),
        @as(f32, @floatCast(@sqrt(@as(f64, @floatFromInt(total_pixels))))),
    ));
    var ref_ysize: usize = @intFromFloat(@max(
        @as(f32, @floatFromInt(max_y_size)),
        @as(f32, @floatCast(@sqrt(@as(f64, @floatFromInt(total_pixels))))),
    ));
    const ref_positions = try allocator.alloc(XY, info.items.len);
    defer allocator.free(ref_positions);
    var max_y: usize = 0;
    var occupied: []u8 = &.{};
    defer if (occupied.len > 0) allocator.free(occupied);
    while (true) {
        max_y = 0;
        // Increase packed image size.
        ref_xsize = @intFromFloat(
            @as(f32, @floatFromInt(ref_xsize)) * kBinPackingSlackness + 1.0,
        );
        ref_ysize = @intFromFloat(
            @as(f32, @floatFromInt(ref_ysize)) * kBinPackingSlackness + 1.0,
        );

        if (occupied.len != ref_xsize * ref_ysize) {
            if (occupied.len > 0) allocator.free(occupied);
            occupied = try allocator.alloc(u8, ref_xsize * ref_ysize);
        }
        @memset(occupied, 0);

        var success = true;
        // For every patch...
        for (info.items, 0..) |*p, patch| {
            const xsize = p.patch.xsize;
            const ysize = p.patch.ysize;
            var found = false;
            var x0: usize = 0;
            var y0: usize = 0;
            // For every possible start position ...
            while (y0 + ysize <= ref_ysize) : (y0 += 1) {
                x0 = 0;
                while (x0 + xsize <= ref_xsize) : (x0 += 1) {
                    var has_occupied_pixel = false;
                    var x = x0;
                    // Check if it is possible to place the patch in this
                    // position in the reference frame.
                    var y = y0;
                    while (y < y0 + ysize) : (y += 1) {
                        x = x0;
                        while (x < x0 + xsize) : (x += 1) {
                            if (occupied[y * ref_xsize + x] != 0) {
                                has_occupied_pixel = true;
                                break;
                            }
                        }
                    }
                    if (!has_occupied_pixel) {
                        found = true;
                        break;
                    }
                    x0 = x; // Jump to next pixel after the occupied one.
                }
                if (found) break;
            }

            // We didn't find a possible position: repeat with a larger
            // reference frame size.
            if (!found) {
                success = false;
                break;
            }

            // Mark the corresponding positions as used.
            ref_positions[patch] = .{
                .x = @intCast(x0),
                .y = @intCast(y0),
            };
            var yy = y0;
            while (yy < y0 + ysize) : (yy += 1) {
                var xx = x0;
                while (xx < x0 + xsize) : (xx += 1) {
                    occupied[yy * ref_xsize + xx] = 1;
                }
            }
            max_y = @max(max_y, y0 + ysize);
        }
        if (success) break;
    }
    std.debug.assert(ref_ysize >= max_y);
    ref_ysize = max_y;

    // Build the reference image planes (integer differences, or the bit
    // patterns of float differences).
    for (0..n_color) |_| {
        const plane = try allocator.alloc(
            i32,
            ref_xsize * ref_ysize,
        );
        @memset(plane, 0);
        try out.ref_planes.append(allocator, plane);
    }
    const num_ec = num_extra_channels;
    for (info.items, 0..) |*p, i| {
        const rp = ref_positions[i];
        var y: usize = 0;
        while (y < p.patch.ysize) : (y += 1) {
            var x: usize = 0;
            while (x < p.patch.xsize) : (x += 1) {
                for (0..n_color) |c| {
                    const f = p.patch.fpixels[c][y * p.patch.xsize + x];
                    const sample: i32 = if (format.floating)
                        try float_convert.floatToInt(
                            @as(f32, @bitCast(f)),
                            @intCast(format.bits),
                            @intCast(format.exp_bits),
                        )
                    else
                        f;
                    out.ref_planes.items[c][(y + rp.y) * ref_xsize + x + rp.x] = sample;
                }
            }
        }
        for (p.positions.items) |pos| {
            try out.positions.append(allocator, .{
                .x = pos.x,
                .y = pos.y,
                .ref_pos_idx = out.ref_positions.items.len,
            });
            // Add blending for color channels, ignore other channels.
            try out.blendings.append(
                allocator,
                .{
                    .mode = .add,
                    .alpha_channel = 0,
                    .clamp = false,
                },
            );
            for (0..num_ec) |_| {
                try out.blendings.append(
                    allocator,
                    .{
                        .mode = .none,
                        .alpha_channel = 0,
                        .clamp = false,
                    },
                );
            }
        }
        try out.ref_positions.append(allocator, .{
            .ref = patches_mod.kPatchFrameReferenceId,
            .x0 = rp.x,
            .y0 = rp.y,
            .xsize = p.patch.xsize,
            .ysize = p.patch.ysize,
        });
    }
    out.ref_xsize = ref_xsize;
    out.ref_ysize = ref_ysize;

    for (info.items) |*p| p.deinit(allocator);
    info.deinit(allocator);
    return out;
}

/// The result of the VarDCT (XYB) patch packing.
pub const PackedPatchesXyb = struct {
    positions: std.ArrayList(patches_mod.PatchPosition) = .empty,
    ref_positions: std.ArrayList(patches_mod.PatchReferencePosition) = .empty,
    blendings: std.ArrayList(patches_mod.PatchBlending) = .empty,
    /// The bin-packed reference image as *modular stream* planes, in the
    /// [Y, X, B-Y] channel order a modular XYB frame carries
    /// (`enc_modular.cc`: `c_out = 1 - c` for c < 2, B stores
    /// `trunc(B * factor + 0.5) - Y`), already quantized onto the
    /// `dc_quant` grid through the exact F16 round trip the decoder will
    /// see.
    ref_planes: std.ArrayList([]i32) = .empty,
    /// The reconstructed reference frame floats in color order [X, Y, B]
    /// — exactly what a decoder computes from `ref_planes` and `dc_quant`
    /// (`ModularImageToDecodedRect`: X = ch1 * dcq, Y = ch0 * dcq,
    /// B = (ch2 + ch0) * dcq). Feeds both the reference store and the
    /// encoder-side subtraction.
    ref_floats: [3][]f32 = .{ &.{}, &.{}, &.{} },
    ref_xsize: usize = 0,
    ref_ysize: usize = 0,
    /// The effective DC quants (1/enc_factors after the F16 round trip) —
    /// the DequantDC values of the patch reference frame.
    dc_quant: [3]f32 = .{ 0, 0, 0 },

    pub fn deinit(self: *PackedPatchesXyb, allocator: Allocator) void {
        self.positions.deinit(allocator);
        self.ref_positions.deinit(allocator);
        self.blendings.deinit(allocator);
        for (self.ref_planes.items) |p| allocator.free(p);
        self.ref_planes.deinit(allocator);
        for (&self.ref_floats) |p| {
            if (p.len != 0) allocator.free(p);
        }
    }
};

/// The VarDCT tail of `FindBestPatchDictionary` + the patch-frame
/// quantization of `enc_modular.cc`'s XYB modular path: bin-pack the
/// patches, quantize the XYB differences onto the `dc_quant` grid
/// (the `enc_factors = {65536/(1+23d), 4096/(1+14d), 4096/(1+14d)}`
/// scaling of libjxl, F16-round-tripped like the DequantDC field), and
/// reconstruct the exact floats the decoder will blend back. `info` is
/// consumed (freed).
pub fn packPatchesXyb(
    allocator: Allocator,
    info: *std.ArrayList(PatchInfo),
    num_extra_channels: usize,
    distance: f32,
) !PackedPatchesXyb {
    var out = PackedPatchesXyb{};
    errdefer out.deinit(allocator);
    errdefer {
        for (info.items) |*p| p.deinit(allocator);
        info.deinit(allocator);
    }

    // enc_modular.cc: distance-scaled XYB factors.
    var enc_factors: [3]f32 = .{ 65536.0, 4096.0, 4096.0 };
    if (distance > 0) {
        enc_factors[0] *= 1.0 / (1.0 + 23.0 * distance);
        enc_factors[1] *= 1.0 / (1.0 + 14.0 * distance);
        enc_factors[2] *= 1.0 / (1.0 + 14.0 * distance);
    }
    // DequantMatricesSetCustomDC: dc_quant = 1/factor, serialized as
    // F16(dc_quant * 128) and read back as value / 128 — apply the same
    // round trip so the reconstruction matches the decoder bit for bit.
    for (0..3) |c| {
        out.dc_quant[c] = 1.0 / enc_factors[c];
        var w = bit_io.BitWriter.init(allocator);
        defer w.deinit();
        try fields_ext.writeF16(out.dc_quant[c] * 128.0, &w);
        var br = bit_io.BitReader.init(w.getSpan());
        out.dc_quant[c] = (try fields_ext.readF16(&br)) * (1.0 / 128.0);
        if (out.dc_quant[c] < 1.0 / 65536.0) return error.InvalidDCQuant;
    }

    // Sort by area, descending (shared with the lossless packer).
    std.sort.pdq(
        PatchInfo,
        info.items,
        {},
        struct {
            fn f(_: void, a: PatchInfo, b: PatchInfo) bool {
                return a.patch.xsize * a.patch.ysize >
                    b.patch.xsize * b.patch.ysize;
            }
        }.f,
    );

    var max_x_size: usize = 0;
    var max_y_size: usize = 0;
    var total_pixels: usize = 0;
    for (info.items) |*p| {
        const pixels = p.patch.xsize * p.patch.ysize;
        if (max_x_size < p.patch.xsize) max_x_size = p.patch.xsize;
        if (max_y_size < p.patch.ysize) max_y_size = p.patch.ysize;
        total_pixels += pixels;
    }

    // Bin-packing (identical loop to packPatches).
    var ref_xsize: usize = @intFromFloat(@max(
        @as(f32, @floatFromInt(max_x_size)),
        @as(f32, @floatCast(@sqrt(@as(f64, @floatFromInt(total_pixels))))),
    ));
    var ref_ysize: usize = @intFromFloat(@max(
        @as(f32, @floatFromInt(max_y_size)),
        @as(f32, @floatCast(@sqrt(@as(f64, @floatFromInt(total_pixels))))),
    ));
    const ref_positions = try allocator.alloc(XY, info.items.len);
    defer allocator.free(ref_positions);
    var max_y: usize = 0;
    var occupied: []u8 = &.{};
    defer if (occupied.len > 0) allocator.free(occupied);
    while (true) {
        max_y = 0;
        ref_xsize = @intFromFloat(@as(f32, @floatFromInt(ref_xsize)) *
            kBinPackingSlackness + 1.0);
        ref_ysize = @intFromFloat(@as(f32, @floatFromInt(ref_ysize)) *
            kBinPackingSlackness + 1.0);
        if (occupied.len != ref_xsize * ref_ysize) {
            if (occupied.len > 0) allocator.free(occupied);
            occupied = try allocator.alloc(u8, ref_xsize * ref_ysize);
        }
        @memset(occupied, 0);
        var success = true;
        for (info.items, 0..) |*p, patch| {
            const xsize = p.patch.xsize;
            const ysize = p.patch.ysize;
            var found = false;
            var x0: usize = 0;
            var y0: usize = 0;
            while (y0 + ysize <= ref_ysize) : (y0 += 1) {
                x0 = 0;
                while (x0 + xsize <= ref_xsize) : (x0 += 1) {
                    var has_occupied_pixel = false;
                    var x = x0;
                    var y = y0;
                    while (y < y0 + ysize) : (y += 1) {
                        x = x0;
                        while (x < x0 + xsize) : (x += 1) {
                            if (occupied[y * ref_xsize + x] != 0) {
                                has_occupied_pixel = true;
                                break;
                            }
                        }
                    }
                    if (!has_occupied_pixel) {
                        found = true;
                        break;
                    }
                    x0 = x;
                }
                if (found) break;
            }
            if (!found) {
                success = false;
                break;
            }
            ref_positions[patch] = .{
                .x = @intCast(x0),
                .y = @intCast(y0),
            };
            var yy = y0;
            while (yy < y0 + ysize) : (yy += 1) {
                var xx = x0;
                while (xx < x0 + xsize) : (xx += 1) {
                    occupied[yy * ref_xsize + xx] = 1;
                }
            }
            max_y = @max(max_y, y0 + ysize);
        }
        if (success) break;
    }
    std.debug.assert(ref_ysize >= max_y);
    ref_ysize = max_y;
    const n_pix = ref_xsize * ref_ysize;

    // Modular stream planes [Y, X, B-Y] + reconstructed floats [X, Y, B].
    for (0..3) |_| {
        const plane = try allocator.alloc(i32, n_pix);
        @memset(plane, 0);
        try out.ref_planes.append(allocator, plane);
    }
    for (0..3) |c| {
        out.ref_floats[c] = try allocator.alloc(f32, n_pix);
        @memset(out.ref_floats[c], 0);
    }
    const num_ec = num_extra_channels;
    for (info.items, 0..) |*p, i| {
        const rp = ref_positions[i];
        var y: usize = 0;
        while (y < p.patch.ysize) : (y += 1) {
            var x: usize = 0;
            while (x < p.patch.xsize) : (x += 1) {
                const off = (y + rp.y) * ref_xsize + x + rp.x;
                const src = y * p.patch.xsize + x;
                const fx: f32 = @bitCast(p.patch.fpixels[0][src]); // X
                const fy: f32 = @bitCast(p.patch.fpixels[1][src]); // Y
                const fb: f32 = @bitCast(p.patch.fpixels[2][src]); // B
                // float_to_int (round half away from zero) for Y and X;
                // the B plane carries trunc(B*factor + 0.5) - Y.
                const vy = floatToIntXyb(fy, out.dc_quant[1]);
                const vx = floatToIntXyb(fx, out.dc_quant[0]);
                const vb = floatToIntBXyb(fb, out.dc_quant[2]) - vy;
                out.ref_planes.items[0][off] = vy;
                out.ref_planes.items[1][off] = vx;
                out.ref_planes.items[2][off] = vb;
                out.ref_floats[0][off] = @as(f32, @floatFromInt(vx)) * out.dc_quant[0];
                out.ref_floats[1][off] = @as(f32, @floatFromInt(vy)) * out.dc_quant[1];
                out.ref_floats[2][off] = @as(f32, @floatFromInt(vb + vy)) * out.dc_quant[2];
            }
        }
        for (p.positions.items) |pos| {
            try out.positions.append(allocator, .{
                .x = pos.x,
                .y = pos.y,
                .ref_pos_idx = out.ref_positions.items.len,
            });
            try out.blendings.append(
                allocator,
                .{
                    .mode = .add,
                    .alpha_channel = 0,
                    .clamp = false,
                },
            );
            for (0..num_ec) |_| {
                try out.blendings.append(
                    allocator,
                    .{
                        .mode = .none,
                        .alpha_channel = 0,
                        .clamp = false,
                    },
                );
            }
        }
        try out.ref_positions.append(allocator, .{
            .ref = patches_mod.kPatchFrameReferenceId,
            .x0 = rp.x,
            .y0 = rp.y,
            .xsize = p.patch.xsize,
            .ysize = p.patch.ysize,
        });
    }
    out.ref_xsize = ref_xsize;
    out.ref_ysize = ref_ysize;

    for (info.items) |*p| p.deinit(allocator);
    info.deinit(allocator);
    return out;
}

/// `float_to_int` (non-fp): `trunc(v * factor + (v < 0 ? -0.5 : 0.5))`
/// with factor = 1/dc_quant.
fn floatToIntXyb(v: f32, dc_quant: f32) i32 {
    const factor = 1.0 / dc_quant;
    const scaled = v * factor + (if (v < 0) @as(f32, -0.5) else @as(f32, 0.5));
    const clamped = std.math.clamp(
        scaled,
        -2147483648.0,
        2147483520.0,
    );
    return @intFromFloat(@trunc(clamped));
}

/// The B-plane variant (`enc_modular.cc` line 803): always +0.5.
fn floatToIntBXyb(v: f32, dc_quant: f32) i32 {
    const factor = 1.0 / dc_quant;
    const scaled = v * factor + 0.5;
    const clamped = std.math.clamp(
        scaled,
        -2147483648.0,
        2147483520.0,
    );
    return @intFromFloat(@trunc(clamped));
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

const t_glyph = [_][]const u8{
    &.{ 1, 1, 1, 1, 1, 1 },
    &.{ 0, 0, 1, 1, 0, 0 },
    &.{ 0, 0, 1, 1, 0, 0 },
    &.{ 0, 0, 1, 1, 0, 0 },
    &.{ 0, 0, 1, 1, 0, 0 },
    &.{ 0, 0, 1, 1, 0, 0 },
    &.{ 0, 0, 1, 1, 0, 0 },
};

/// A "screenshot": uniform 200 background, three occurrences of a dark
/// glyph (difference -60 from the background). Returns 3 planes; gray
/// callers use planes[0..1].
fn makeScreenshot(
    alloc: Allocator,
    w: usize,
    h: usize,
    fg: i32,
    bg: i32,
) ![3][]i32 {
    var planes: [3][]i32 = undefined;
    for (&planes) |*p| {
        p.* = try alloc.alloc(i32, w * h);
        @memset(p.*, bg);
    }
    const spots = [_]XY{
        .{ .x = 8, .y = 6 },
        .{ .x = 30, .y = 6 },
        .{ .x = 8, .y = 24 },
    };
    for (spots) |s| {
        for (t_glyph, 0..) |row, y| {
            for (row, 0..) |m, x| {
                if (m == 0) continue;
                for (0..3) |c| {
                    planes[c][(s.y + y) * w + s.x + x] = fg;
                }
            }
        }
    }
    return planes;
}

fn freePlanes(alloc: Allocator, planes: []const []const i32) void {
    for (planes) |p| alloc.free(p);
}

test "findTextLikePatches: text-like image yields one repeated patch" {
    const alloc = testing.allocator;
    const w = 64;
    const h = 48;
    const planes = try makeScreenshot(
        alloc,
        w,
        h,
        140,
        200,
    );
    defer freePlanes(alloc, &planes);

    var info = try findTextLikePatches(
        alloc,
        &planes,
        3,
        w,
        h,
        .{ .bits = 8 },
        .default,
    );
    defer {
        for (info.items) |*p| p.deinit(alloc);
        info.deinit(alloc);
    }
    try testing.expectEqual(@as(usize, 1), info.items.len);
    try testing.expectEqual(@as(usize, 3), info.items[0].positions.items.len);
    // The glyph bbox is 6x7.
    try testing.expectEqual(@as(usize, 6), info.items[0].patch.xsize);
    try testing.expectEqual(@as(usize, 7), info.items[0].patch.ysize);
}

test "findTextLikePatches: gray text-like image works" {
    const alloc = testing.allocator;
    const w = 64;
    const h = 48;
    const planes = try makeScreenshot(
        alloc,
        w,
        h,
        140,
        200,
    );
    defer freePlanes(alloc, &planes);
    const chans: []const []const i32 = planes[0..1];

    var info = try findTextLikePatches(
        alloc,
        chans,
        1,
        w,
        h,
        .{ .bits = 8 },
        .default,
    );
    defer {
        for (info.items) |*p| p.deinit(alloc);
        info.deinit(alloc);
    }
    try testing.expectEqual(@as(usize, 1), info.items.len);
    try testing.expectEqual(@as(usize, 3), info.items[0].positions.items.len);
}

test "findTextLikePatches: 16-bit low contrast is too small" {
    const alloc = testing.allocator;
    const w = 64;
    const h = 48;
    // Glyph difference 60 on 16-bit: quantized peak 60*12.75/65535 < 2
    // -> too_small, no patches (libjxl needs |diff| > ~10280 there).
    const planes = try makeScreenshot(alloc, w, h, 39940, 40000);
    defer freePlanes(alloc, &planes);

    var info = try findTextLikePatches(
        alloc,
        &planes,
        3,
        w,
        h,
        .{ .bits = 16 },
        .default,
    );
    defer {
        for (info.items) |*p| p.deinit(alloc);
        info.deinit(alloc);
    }
    try testing.expectEqual(@as(usize, 0), info.items.len);
}

test "findTextLikePatches: noise yields no patches" {
    const alloc = testing.allocator;
    const w = 64;
    const h = 48;
    var planes: [3][]i32 = undefined;
    for (&planes) |*p| {
        p.* = try alloc.alloc(i32, w * h);
        var prng = std.Random.DefaultPrng.init(0x5EED);
        for (p.*) |*v| v.* = @intCast(prng.random().uintLessThan(usize, 256));
    }
    defer freePlanes(alloc, &planes);

    var info = try findTextLikePatches(
        alloc,
        &planes,
        3,
        w,
        h,
        .{ .bits = 8 },
        .default,
    );
    defer {
        for (info.items) |*p| p.deinit(alloc);
        info.deinit(alloc);
    }
    try testing.expectEqual(@as(usize, 0), info.items.len);
}

test "findTextLikePatches: override off disables detection" {
    const alloc = testing.allocator;
    const w = 64;
    const h = 48;
    const planes = try makeScreenshot(alloc, w, h, 140, 200);
    defer freePlanes(alloc, &planes);

    var info = try findTextLikePatches(
        alloc,
        &planes,
        3,
        w,
        h,
        .{ .bits = 8 },
        .off,
    );
    defer {
        for (info.items) |*p| p.deinit(alloc);
        info.deinit(alloc);
    }
    try testing.expectEqual(@as(usize, 0), info.items.len);
}

test "packPatches: no overlap, all patches inside the reference image" {
    const alloc = testing.allocator;
    const w = 64;
    const h = 48;
    const planes = try makeScreenshot(alloc, w, h, 140, 200);
    defer freePlanes(alloc, &planes);

    var info = try findTextLikePatches(
        alloc,
        &planes,
        3,
        w,
        h,
        .{ .bits = 8 },
        .default,
    );
    var pk = try packPatches(
        alloc,
        &info,
        3,
        0,
        .{ .bits = 8 },
    );
    defer pk.deinit(alloc);

    try testing.expect(pk.ref_positions.items.len > 0);
    try testing.expect(pk.ref_xsize >= 6);
    try testing.expect(pk.ref_ysize >= 7);
    for (pk.ref_positions.items) |*rp| {
        try testing.expect(rp.x0 + rp.xsize <= pk.ref_xsize);
        try testing.expect(rp.y0 + rp.ysize <= pk.ref_ysize);
    }
    // The reference planes contain the glyph pattern (fg - bg = -60).
    const rp = &pk.ref_positions.items[0];
    var found_fg = false;
    for (0..rp.ysize) |y| {
        for (0..rp.xsize) |x| {
            for (pk.ref_planes.items) |plane| {
                if (plane[(rp.y0 + y) * pk.ref_xsize + rp.x0 + x] == -60)
                    found_fg = true;
            }
        }
    }
    try testing.expect(found_fg);
    // Positions: 3 occurrences, blendings stride 1 (no extra channels).
    try testing.expectEqual(
        @as(usize, 3),
        pk.positions.items.len,
    );
    for (pk.blendings.items) |*b| {
        try testing.expect(b.mode == .add);
    }
}
