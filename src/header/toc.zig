//! Table of contents, port of libjxl `toc.h/cc` and `enc_toc.cc`.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
// DecodePermutation lives with the coefficient-order machinery
// (libjxl also has toc.cc depend on coeff_order.h: the same
// kPermutationContexts histograms + Lehmer code serve both).
const coeff_order = @import("../vardct/coeff_order.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const U32Enc = fields.U32Enc;

/// libjxl kTocDist: sizes are patchable at 2/3/4 byte granularity.
pub const kTocDist = U32Enc.init(
    fields.bitsDistr(10),
    fields.bitsOffset(14, 1024),
    fields.bitsOffset(22, 17408),
    fields.bitsOffset(30, 4211712),
);

/// Section index of AC group `group` of pass `pass`.
/// Layout: 0 = DC global, 1..num_dc_groups = DC groups,
/// num_dc_groups+1 = AC global, then AC groups per pass.
pub fn acGroupIndex(pass: usize, group: usize, num_groups: usize, num_dc_groups: usize) usize {
    return 2 + num_dc_groups + pass * num_groups + group;
}

pub fn numTocEntries(num_groups: usize, num_dc_groups: usize, num_passes: usize) usize {
    if (num_groups == 1 and num_passes == 1) return 1;
    return acGroupIndex(0, 0, num_groups, num_dc_groups) + num_groups * num_passes;
}

/// A decoded TOC: `sizes` are in *physical* order (the order of the size
/// entries in the bitstream = the order of the section bytes in the
/// frame); `permutation[i]` is the physical slot holding logical section
/// `i` (`null` when no permutation is present). Mirrors libjxl
/// `ReadToc`/`ReadGroupOffsets`: logical section i lives at byte offset
/// `starts[i]` with size `logical_sizes[i]`, where
/// `starts[i] = prefix_sum(sizes)[permutation[i]]`.
pub const Toc = struct {
    sizes: []u32,
    permutation: ?[]u32 = null,

    pub fn deinit(self: *Toc, allocator: std.mem.Allocator) void {
        allocator.free(self.sizes);
        if (self.permutation) |p| allocator.free(p);
        self.* = undefined;
    }

    /// Per-logical-section byte ranges: `starts[i]` is the byte offset of
    /// logical section `i` within the frame's section area and
    /// `logical_sizes[i]` its length. Both are re-indexed to logical
    /// order (identity when no permutation is present). Callers own the
    /// allocations.
    pub fn logicalLayout(
        self: *const Toc,
        allocator: std.mem.Allocator,
        logical_sizes: *[]u32,
        starts: *[]usize,
    ) !void {
        const n = self.sizes.len;
        const lsizes = try allocator.alloc(u32, n);
        errdefer allocator.free(lsizes);
        const st = try allocator.alloc(usize, n);
        errdefer allocator.free(st);
        if (self.permutation) |p| {
            var prefix: usize = 0;
            var prefix_sums = try allocator.alloc(usize, n);
            defer allocator.free(prefix_sums);
            for (self.sizes, 0..) |s, i| {
                prefix_sums[i] = prefix;
                prefix += s;
            }
            for (0..n) |i| {
                const slot: usize = @intCast(p[i]);
                st[i] = prefix_sums[slot];
                lsizes[i] = self.sizes[slot];
            }
        } else {
            var prefix: usize = 0;
            for (self.sizes, 0..) |s, i| {
                st[i] = prefix;
                prefix += s;
                lsizes[i] = s;
            }
        }
        logical_sizes.* = lsizes;
        starts.* = st;
    }

    pub fn totalSize(self: *const Toc) usize {
        var total: usize = 0;
        for (self.sizes) |s| total += s;
        return total;
    }
};

/// Reads TOC sizes (and permutation, when present — e.g. cjxl
/// `--group_order=1` / center-first group order). Sizes come back in
/// physical order; use `Toc.logicalLayout` for logical indices.
pub fn readTocWithPermutation(allocator: std.mem.Allocator, toc_entries: usize, br: *BitReader) !Toc {
    if (toc_entries > 65536) return error.TooManyTocEntries;
    const sizes = try allocator.alloc(u32, toc_entries);
    errdefer allocator.free(sizes);
    var permutation: ?[]u32 = null;
    errdefer {
        if (permutation) |p| allocator.free(p);
    }
    if ((br.readBits(1)) != 0) {
        const perm = try allocator.alloc(u32, toc_entries);
        permutation = perm;
        try coeff_order.decodePermutation(allocator, 0, toc_entries, perm, br);
    }
    try br.jumpToByteBoundary();
    for (sizes) |*s| {
        s.* = U32Coder.read(kTocDist, br);
    }
    try br.jumpToByteBoundary();
    // Sanity: a permutation must be a bijection of 0..n-1.
    if (permutation) |p| {
        var seen = try allocator.alloc(bool, toc_entries);
        defer allocator.free(seen);
        @memset(seen, false);
        for (p) |v| {
            if (v >= toc_entries) return error.InvalidTocPermutation;
            if (seen[v]) return error.InvalidTocPermutation;
            seen[v] = true;
        }
    }
    return .{ .sizes = sizes, .permutation = permutation };
}

/// Reads TOC sizes; permutation is unsupported and rejected.
pub fn readToc(allocator: std.mem.Allocator, toc_entries: usize, br: *BitReader) ![]u32 {
    if (toc_entries > 65536) return error.TooManyTocEntries;
    const sizes = try allocator.alloc(u32, toc_entries);
    errdefer allocator.free(sizes);
    if ((br.readBits(1)) != 0) return error.UnsupportedTocPermutation;
    try br.jumpToByteBoundary();
    for (sizes) |*s| {
        s.* = U32Coder.read(kTocDist, br);
    }
    try br.jumpToByteBoundary();
    return sizes;
}

/// Writes the no-permutation TOC for the given section sizes.
pub fn writeToc(sizes: []const u32, writer: *BitWriter) !void {
    writer.write(1, 0); // no permutation
    writer.zeroPadToByte();
    for (sizes) |s| {
        try U32Coder.write(kTocDist, s, writer);
    }
    writer.zeroPadToByte();
}

/// Writes the TOC with a section permutation (libjxl `WriteTocPermutation` +
/// `WriteTocSizes`): `permutation[i]` is the physical slot holding logical
/// section `i`, `sizes` are listed in physical order.
pub fn writeTocWithPermutation(
    allocator: std.mem.Allocator,
    permutation: []const u32,
    sizes: []const u32,
    writer: *BitWriter,
) !void {
    writer.write(1, 1); // permutation present
    try coeff_order.encodePermutation(allocator, 0, permutation.len, permutation, writer);
    writer.zeroPadToByte(); // before TOC entries
    for (sizes) |s| {
        try U32Coder.write(kTocDist, s, writer);
    }
    writer.zeroPadToByte(); // before first group
}

// ---------------------------------------------------------------------------
// Center-first group order (libjxl enc_frame.cc PermuteGroups, the cjxl
// `--group_order=1` layout)
// ---------------------------------------------------------------------------

/// IEEE 754 remainder (C `std::remainder`): x - rint(x/y)*y, where rint
/// rounds to nearest with ties to even (the default FP mode). The tie
/// behavior matters: angle values of exactly +/-pi must keep the positive
/// sign (quotient +0.5 rounds to 0), or the center-first ring rotates.
fn roundTiesToEven(v: f64) f64 {
    const t = @trunc(v);
    if (@abs(v - t) != 0.5) return @round(v);
    // Exact tie: the nearest even integer of the two candidates.
    if (@mod(t, 2) == 0) return t;
    return t + std.math.sign(v);
}

fn ieeeRemainder(x: f64, y: f64) f64 {
    return x - roundTiesToEven(x / y) * y;
}

/// Sort key of one AC group: Chebyshev distance from the central group plus
/// the rotated angle (concentric squares in clockwise order, starting at the
/// side of the central group the image center lies closest to).
const GroupKey = struct {
    gid: usize,
    dist: i64,
    angle: f64,
};

/// Port of `PermuteGroups` (libjxl enc_frame.cc): the center-first
/// `--group_order=1` permutation. Returns `permutation[i]` = the physical
/// slot of logical section `i` (the first `num_dc_groups + 2` entries are the
/// globals and DC groups and stay in place; the AC-group blocks of every
/// pass are reordered center-first), or null when no permutation is written
/// (not center-first, or a single-pass single-group frame).
pub fn permuteGroups(
    allocator: std.mem.Allocator,
    xsize: usize,
    ysize: usize,
    group_dim: usize,
    xsize_groups: usize,
    num_groups: usize,
    num_dc_groups: usize,
    num_passes: usize,
) !?[]u32 {
    if (num_passes == 1 and num_groups == 1) return null;

    const n_fixed = num_dc_groups + 2;
    var permutation = try allocator.alloc(u32, n_fixed + num_passes * num_groups);
    errdefer allocator.free(permutation);
    for (0..n_fixed) |i| permutation[i] = @intCast(i);

    // The center of the image (cjxl defaults: middle of the image).
    const imag_cx: i64 = @intCast(xsize / 2);
    const imag_cy: i64 = @intCast(ysize / 2);
    // Center of the group containing the center of the image.
    const cx: i64 = @divTrunc(imag_cx, @as(i64, @intCast(group_dim))) * @as(i64, @intCast(group_dim)) + @as(i64, @intCast(group_dim / 2));
    const cy: i64 = @divTrunc(imag_cy, @as(i64, @intCast(group_dim))) * @as(i64, @intCast(group_dim)) + @as(i64, @intCast(group_dim / 2));
    // Side of the central group the image center lies closest to
    // (0, 1, 2, 3 = left, bottom, right, top).
    const direction: f64 = -std.math.atan2(
        @as(f64, @floatFromInt(imag_cy - cy)),
        @as(f64, @floatFromInt(imag_cx - cx)),
    );
    const side: i64 = @intFromFloat(@trunc(
        @rem(direction + 5 * std.math.pi / 4.0, 2.0 * std.math.pi) * 2.0 / std.math.pi,
    ));

    var keys = try allocator.alloc(GroupKey, num_groups);
    defer allocator.free(keys);
    for (0..num_groups) |gid| {
        const gx: i64 = @intCast((gid % xsize_groups) * group_dim);
        const gy: i64 = @intCast((gid / xsize_groups) * group_dim);
        const gcx: i64 = gx + @as(i64, @intCast(group_dim / 2));
        const gcy: i64 = gy + @as(i64, @intCast(group_dim / 2));
        const dx: i64 = gcx - cx;
        const dy: i64 = gcy - cy;
        const adx: i64 = if (dx < 0) -dx else dx;
        const ady: i64 = if (dy < 0) -dy else dy;
        const angle: f64 = ieeeRemainder(
            std.math.atan2(@as(f64, @floatFromInt(dy)), @as(f64, @floatFromInt(dx))) +
                std.math.pi / 4.0 + @as(f64, @floatFromInt(side)) * (std.math.pi / 2.0),
            2.0 * std.math.pi,
        );
        keys[gid] = .{ .gid = gid, .dist = @max(adx, ady), .angle = angle };
    }
    // Stable sort: ties keep the group order (libjxl sorts on the same pair;
    // its std::sort is unstable but exact ties only occur for symmetric
    // layouts, where any resolution yields a valid permutation).
    std.mem.sort(GroupKey, keys, {}, struct {
        fn lessThan(_: void, a: GroupKey, b: GroupKey) bool {
            if (a.dist != b.dist) return a.dist < b.dist;
            return a.angle < b.angle;
        }
    }.lessThan);

    var inv_ac_group_order = try allocator.alloc(u32, num_groups);
    defer allocator.free(inv_ac_group_order);
    for (keys, 0..) |key, i| inv_ac_group_order[key.gid] = @intCast(i);

    for (0..num_passes) |pass| {
        const pass_start: u32 = @intCast(n_fixed + pass * num_groups);
        for (0..num_groups) |i| {
            permutation[pass_start + i] = pass_start + inv_ac_group_order[i];
        }
    }
    return permutation;
}

test "toc roundtrip" {
    const alloc = std.testing.allocator;
    const sizes = [_]u32{ 0, 12, 1024, 99999, 7 };
    var w = BitWriter.init(alloc);
    defer w.deinit();
    try writeToc(&sizes, &w);
    var br = BitReader.init(w.getSpan());
    const got = try readToc(alloc, sizes.len, &br);
    defer alloc.free(got);
    try std.testing.expectEqualSlices(u32, &sizes, got);
}

test "toc permutation write/read roundtrip" {
    const alloc = std.testing.allocator;
    // A shuffled section order: globals (0..3) stay in place, AC group
    // sections (4..11) reversed — the shape a `--group_order=1` file has.
    const n = 12;
    var perm: [n]u32 = undefined;
    for (&perm, 0..) |*p, i| p.* = @intCast(if (i < 4) i else n - 1 - (i - 4));
    const sizes = [_]u32{ 100, 0, 5, 3, 70, 1, 2, 9, 40, 0, 8, 17 };

    var w = BitWriter.init(alloc);
    defer w.deinit();
    w.write(1, 1); // permutation present
    try coeff_order.encodePermutation(alloc, 0, n, &perm, &w);
    w.zeroPadToByte();
    for (sizes) |s| {
        try U32Coder.write(kTocDist, s, &w);
    }
    w.zeroPadToByte();

    var br = BitReader.init(w.getSpan());
    var toc = try readTocWithPermutation(alloc, n, &br);
    defer toc.deinit(alloc);
    // Sizes come back in physical (listed) order.
    try std.testing.expectEqualSlices(u32, &sizes, toc.sizes);
    const p = toc.permutation.?;
    for (0..n) |i| {
        try std.testing.expectEqual(perm[i], p[i]);
    }

    // Logical layout: logical section i lives at prefix(sizes)[perm[i]].
    var logical_sizes: []u32 = undefined;
    var starts: []usize = undefined;
    try toc.logicalLayout(alloc, &logical_sizes, &starts);
    defer alloc.free(logical_sizes);
    defer alloc.free(starts);
    var prefix: usize = 0;
    var prefix_of = [_]usize{0} ** n;
    for (sizes, 0..) |s, i| {
        prefix_of[i] = prefix;
        prefix += s;
    }
    try std.testing.expectEqual(prefix, toc.totalSize());
    for (0..n) |i| {
        const slot: usize = @intCast(p[i]);
        try std.testing.expectEqual(prefix_of[slot], starts[i]);
        try std.testing.expectEqual(sizes[slot], logical_sizes[i]);
    }
}

test "toc permutation roundtrip with identity layout fallback" {
    const alloc = std.testing.allocator;
    const sizes = [_]u32{ 3, 4, 5 };
    var w = BitWriter.init(alloc);
    defer w.deinit();
    try writeToc(&sizes, &w);
    var br = BitReader.init(w.getSpan());
    var toc = try readTocWithPermutation(alloc, sizes.len, &br);
    defer toc.deinit(alloc);
    try std.testing.expect(toc.permutation == null);
    var logical_sizes: []u32 = undefined;
    var starts: []usize = undefined;
    try toc.logicalLayout(alloc, &logical_sizes, &starts);
    defer alloc.free(logical_sizes);
    defer alloc.free(starts);
    try std.testing.expectEqual(@as(usize, 0), starts[0]);
    try std.testing.expectEqual(@as(usize, 3), starts[1]);
    try std.testing.expectEqual(@as(usize, 7), starts[2]);
    try std.testing.expectEqualSlices(u32, &sizes, logical_sizes);
}

// ---------------------------------------------------------------------------
// permuteGroups (round 8): the center-first --group_order=1 layout
// ---------------------------------------------------------------------------

test "permuteGroups: single-pass single-group writes no permutation" {
    const alloc = std.testing.allocator;
    const p = try permuteGroups(alloc, 100, 100, 256, 1, 1, 1, 1);
    defer if (p) |q| alloc.free(q);
    try std.testing.expect(p == null);
}

test "permuteGroups: 2x2 groups center-first order" {
    const alloc = std.testing.allocator;
    // 512x512 with 256px groups: center group is (1,1); the remaining
    // three groups form one ring; clockwise from the left side this
    // yields the physical order [g2, g0, g1] (verified equal to
    // cjxl --group_order=1 on the same geometry).
    const p = (try permuteGroups(alloc, 512, 512, 256, 2, 4, 1, 1)).?;
    defer alloc.free(p);
    const expected = [_]u32{ 0, 1, 2, 5, 6, 4, 3 };
    try std.testing.expectEqualSlices(u32, &expected, p);
}

test "permuteGroups: invariants of the center-first layout" {
    const alloc = std.testing.allocator;
    const cases = [_]struct { w: usize, h: usize, gd: usize, xg: usize, ng: usize, ndc: usize, np: usize }{
        .{ .w = 1024, .h = 768, .gd = 256, .xg = 4, .ng = 12, .ndc = 1, .np = 1 },
        .{ .w = 999, .h = 777, .gd = 256, .xg = 4, .ng = 12, .ndc = 1, .np = 1 },
        .{ .w = 640, .h = 480, .gd = 128, .xg = 5, .ng = 20, .ndc = 1, .np = 1 },
        .{ .w = 600, .h = 400, .gd = 256, .xg = 3, .ng = 6, .ndc = 1, .np = 2 },
    };
    for (cases) |c| {
        const n_fixed = c.ndc + 2;
        const total = n_fixed + c.np * c.ng;
        const p = (try permuteGroups(alloc, c.w, c.h, c.gd, c.xg, c.ng, c.ndc, c.np)).?;
        defer alloc.free(p);
        try std.testing.expectEqual(total, p.len);
        // Bijection.
        var seen = try alloc.alloc(bool, total);
        defer alloc.free(seen);
        @memset(seen, false);
        for (p) |v| {
            try std.testing.expect(v < total);
            try std.testing.expect(!seen[v]);
            seen[v] = true;
        }
        // Globals and DC groups stay in place.
        for (0..n_fixed) |i| try std.testing.expectEqual(@as(u32, @intCast(i)), p[i]);
        // Every pass block encodes the same center-first inverse order.
        for (0..c.np) |pass| {
            const base = n_fixed + pass * c.ng;
            const center_gx = (c.w / 2) / c.gd;
            const center_gy = (c.h / 2) / c.gd;
            // Rank 0 is the group containing the image center: its logical
            // section maps to the first physical slot of the pass block.
            var first: usize = 0;
            for (0..c.ng) |i| {
                if (p[base + i] == base) {
                    first = i;
                    break;
                }
            }
            try std.testing.expectEqual(@as(u32, @intCast(center_gy * c.xg + center_gx)), @as(u32, @intCast(first)));
        }
    }
}
