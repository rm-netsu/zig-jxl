//! Port of the non-linear-edge-enhancing (NLEE) upsampling filter
//! (`lib/jxl/render_pipeline/stage_upsampling.cc` UpsamplingStage +
//! the default kernels from `lib/jxl/image_metadata.cc`).
//!
//! Each output sub-pixel is a 5x5 FIR of the (mirrored at image edges)
//! input neighborhood, clamped to the neighborhood's min/max. The filter
//! is per-channel and translation-invariant, so whole-image processing is
//! bit-identical to libjxl's group-based render pipeline; the arithmetic
//! uses fused multiply-adds (`@mulAdd`) matching the AVX2 `MulAdd`
//! accumulation order of the reference implementation.

const std = @import("std");

pub const kWeights2 = [15]f32{
    -0.01716200, -0.03452303, -0.04022174, -0.02921014, -0.00624645, 0.14111091, 0.28896755,
    0.00278718, -0.01610267, 0.56661550, 0.03777607, -0.01986694, -0.03144731, -0.01185068,
    -0.00213539,
};

pub const kWeights4 = [55]f32{
    -0.02419067, -0.03491987, -0.03693351, -0.03094285, -0.00529785, -0.01663432,
    -0.03556863, -0.03888905, -0.03516850, -0.00989469, 0.23651958, 0.33392945, -0.01073543,
    -0.01313181, -0.03556694, 0.13048175, 0.40103025, 0.03951150, -0.02077584, 0.46914198,
    -0.00209270, -0.01484589, -0.04064806, 0.18942530, 0.56279892, 0.06674400, -0.02335494,
    -0.03551682, -0.00754830, -0.02267919, -0.02363578, 0.00315804, -0.03399098,
    -0.01359519, -0.00091653, -0.00335467, -0.01163294, -0.01610294, -0.00974088,
    -0.00191622, -0.01095446, -0.03198464, -0.04455121, -0.02799790, -0.00645912,
    0.06390599, 0.22963888, 0.00630981, -0.01897349, 0.67537268, 0.08483369, -0.02534994,
    -0.02205197, -0.01667999, -0.00384443,
};

pub const kWeights8 = [210]f32{
    -0.02928613, -0.03706353, -0.03783812, -0.03324558, -0.00447632, -0.02519406,
    -0.03752601, -0.03901508, -0.03663285, -0.00646649, -0.02066407, -0.03838633,
    -0.04002101, -0.03900035, -0.00901973, -0.01626393, -0.03954148, -0.04046620,
    -0.03979621, -0.01224485, 0.29895328, 0.35757708, -0.02447552, -0.01081748, -0.04314594,
    0.23903219, 0.41119301, -0.00573046, -0.01450239, -0.04246845, 0.17567618, 0.45220643,
    0.02287757, -0.01936783, -0.03583255, 0.11572472, 0.47416733, 0.06284440, -0.02685066,
    0.42720050, -0.02248939, -0.01155273, -0.04562755, 0.28689496, 0.49093869, -0.00007891,
    -0.01545926, -0.04562659, 0.21238920, 0.53980934, 0.03369474, -0.02070211, -0.03866988,
    0.14229550, 0.56593398, 0.08045181, -0.02888298, -0.03680918, -0.00542229, -0.02920477,
    -0.02788574, -0.02118180, -0.03942402, -0.00775547, -0.02433614, -0.03193943,
    -0.02030828, -0.04044014, -0.01074016, -0.01930822, -0.03620399, -0.01974125,
    -0.03919545, -0.01456093, -0.00045072, -0.00360110, -0.01020207, -0.01231907,
    -0.00638988, -0.00071592, -0.00279122, -0.00957115, -0.01288327, -0.00730937,
    -0.00107783, -0.00210156, -0.00890705, -0.01317668, -0.00813895, -0.00153491,
    -0.02128481, -0.04173044, -0.04831487, -0.03293190, -0.00525260, -0.01720322,
    -0.04052736, -0.05045706, -0.03607317, -0.00738030, -0.01341764, -0.03965629,
    -0.05151616, -0.03814886, -0.01005819, 0.18968273, 0.33063684, -0.01300105, -0.01372950,
    -0.04017465, 0.13727832, 0.36402234, 0.01027890, -0.01832107, -0.03365072, 0.08734506,
    0.38194295, 0.04338228, -0.02525993, 0.56408126, 0.00458352, -0.01648227, -0.04887868,
    0.24585519, 0.62026135, 0.04314807, -0.02213737, -0.04158014, 0.16637289, 0.65027023,
    0.09621636, -0.03101388, -0.04082742, -0.00904519, -0.02790922, -0.02117818, 0.00798662,
    -0.03995711, -0.01243427, -0.02231705, -0.02946266, 0.00992055, -0.03600283,
    -0.01684920, -0.00111684, -0.00411204, -0.01297130, -0.01723725, -0.01022545,
    -0.00165306, -0.00313110, -0.01218016, -0.01763266, -0.01125620, -0.00231663,
    -0.01374149, -0.03797620, -0.05142937, -0.03117307, -0.00581914, -0.01064003,
    -0.03608089, -0.05272168, -0.03375670, -0.00795586, 0.09628104, 0.27129991, -0.00353779,
    -0.01734151, -0.03153981, 0.05686230, 0.28500998, 0.02230594, -0.02374955, 0.68214326,
    0.05018048, -0.02320852, -0.04383616, 0.18459474, 0.71517975, 0.10805613, -0.03263677,
    -0.03637639, -0.01394373, -0.02511203, -0.01728636, 0.05407331, -0.02867568,
    -0.01893131, -0.00240854, -0.00446511, -0.01636187, -0.02377053, -0.01522848,
    -0.00333334, -0.00819975, -0.02964169, -0.04499287, -0.02745350, -0.00612408,
    0.02727416, 0.19446600, 0.00159832, -0.02232473, 0.74982506, 0.11452620, -0.03348048,
    -0.01605681, -0.02070339, -0.00458223,
};

/// libjxl `Mirror`: out-of-bounds coordinates reflect, repeating the edge
/// pixel (`-1 -> 0`, `-2 -> 1`, `n -> n-1`, `n+1 -> n-2`).
pub fn mirror(coord: i64, size: i64) i64 {
    std.debug.assert(size != 0);
    var x = coord;
    while (x < 0 or x >= size) {
        if (x < 0) {
            x = -x - 1;
        } else {
            x = 2 * size - 1 - x;
        }
    }
    return x;
}

fn clamp(v: f32, lo: f32, hi: f32) f32 {
    return @min(@max(v, lo), hi);
}

/// The upsampling kernels for one factor: N*N 5x5 kernels (N = 1 << shift,
/// shift in 1..3), expanded from the `H*5 x H*5`-symmetric weight triangle
/// exactly like the `UpsamplingStage` constructor.
pub const Upsampler = struct {
    shift: u5,
    /// kernel[(ky*N + kx)*25 + py*5 + px]
    kernel: [64 * 25]f32 = @splat(0),

    /// `weights` selects the factor's weight set: for shift 1/2/3 the
    /// 15/55/210-entry triangular arrays (defaults or CustomTransformData
    /// overrides).
    pub fn init(shift: u6, weights: []const f32) Upsampler {
        std.debug.assert(shift >= 1 and shift <= 3);
        const N: usize = @as(usize, 1) << @intCast(shift);
        const H: usize = N / 2;
        var self = Upsampler{ .shift = @intCast(shift) };
        const expected = triSize(H);
        std.debug.assert(weights.len == expected);
        var ky: usize = 0;
        while (ky < H) : (ky += 1) {
            var kx: usize = 0;
            while (kx < H) : (kx += 1) {
                const offset0 = (ky * N + kx) * 25;
                const offset1 = (ky * N + (N - 1 - kx)) * 25;
                const offset2 = ((N - 1 - ky) * N + kx) * 25;
                const offset3 = ((N - 1 - ky) * N + (N - 1 - kx)) * 25;
                var py: usize = 0;
                while (py < 5) : (py += 1) {
                    var px: usize = 0;
                    while (px < 5) : (px += 1) {
                        const j = 5 * ky + py;
                        const i = 5 * kx + px;
                        const my = @min(i, j);
                        const mx = @max(i, j);
                        const w = weights[triIndex(H, my, mx)];
                        self.kernel[offset0 + py * 5 + px] = w;
                        self.kernel[offset1 + py * 5 + (4 - px)] = w;
                        self.kernel[offset2 + (4 - py) * 5 + px] = w;
                        self.kernel[offset3 + (4 - py) * 5 + (4 - px)] = w;
                    }
                }
            }
        }
        return self;
    }

    pub fn default(shift: u6) Upsampler {
        const weights: []const f32 = switch (shift) {
            1 => &kWeights2,
            2 => &kWeights4,
            else => &kWeights8,
        };
        return init(shift, weights);
    }

    inline fn kernelAt(self: *const Upsampler, k: usize, i: usize) f32 {
        return self.kernel[k * 25 + i];
    }

    /// Upsamples one f32 plane of `w`x`h` into `out` (`out_w` x `out_h`,
    /// with out_w/out_h <= w*N/h*N; the surplus right/bottom output
    /// pixels are not written). Mirrored borders like the render pipeline.
    pub fn upsamplePlane(
        self: *const Upsampler,
        allocator: std.mem.Allocator,
        in: []const f32,
        w: usize,
        h: usize,
        out: []f32,
        out_w: usize,
        out_h: usize,
    ) !void {
        const N: usize = @as(usize, 1) << self.shift;
        std.debug.assert(in.len >= w * h);
        const full_w = w * N;
        const full_h = h * N;
        std.debug.assert(out_w <= full_w and out_h <= full_h);
        std.debug.assert(out.len >= out_w * out_h);

        const wi: i64 = @intCast(w);
        const hi: i64 = @intCast(h);

        // Per-input-row column min/max over the five neighborhood rows
        // (PreProcessRowImpl), then the sliding 5-wide min/max (mins/maxs).
        const col_min = try allocator.alloc(f32, w + 4);
        defer allocator.free(col_min);
        const col_max = try allocator.alloc(f32, w + 4);
        defer allocator.free(col_max);
        const mins = try allocator.alloc(f32, w);
        defer allocator.free(mins);
        const maxs = try allocator.alloc(f32, w);
        defer allocator.free(maxs);

        var y: usize = 0;
        while (y < h) : (y += 1) {
            // rows y-2 .. y+2, mirrored
            var row_ptrs: [5][]const f32 = undefined;
            var iy: i64 = -2;
            while (iy <= 2) : (iy += 1) {
                const my = mirror(@as(i64, @intCast(y)) + iy, hi);
                row_ptrs[@intCast(iy + 2)] = in[@as(usize, @intCast(my)) * w ..][0..w];
            }
            // col min/max over the 5 rows, columns x-2 .. x+1+w (mirrored)
            var cx: usize = 0;
            while (cx < w + 4) : (cx += 1) {
                const mx = mirror(@as(i64, @intCast(cx)) - 2, wi);
                const col: usize = @intCast(mx);
                var mn = row_ptrs[0][col];
                var mxv = mn;
                for (1..5) |r| {
                    const v = row_ptrs[r][col];
                    mn = @min(mn, v);
                    mxv = @max(mxv, v);
                }
                col_min[cx] = mn;
                col_max[cx] = mxv;
            }
            // sliding 5-wide min/max
            for (0..w) |x| {
                var mn = col_min[x];
                var mxv = col_max[x];
                for (1..5) |d| {
                    mn = @min(mn, col_min[x + d]);
                    mxv = @max(mxv, col_max[x + d]);
                }
                mins[x] = mn;
                maxs[x] = mxv;
            }

            // 5x5 taps per sub-pixel (ProcessRowImpl), FMA accumulation in
            // the reference's acc0/acc1/acc2 order.
            var oy: usize = 0;
            while (oy < N) : (oy += 1) {
                const out_row = y * N + oy;
                if (out_row >= out_h) break;
                const dst = out[out_row * out_w ..][0..out_w];
                var ox: usize = 0;
                while (ox < N) : (ox += 1) {
                    const k = N * oy + ox;
                    var x: usize = 0;
                    while (x < w) : (x += 1) {
                        if (x * N + ox >= out_w) break;
                        // input[5*(iy+2) + (ix+2)] for iy,ix in -2..2,
                        // mirrored coordinates
                        var taps: [25]f32 = undefined;
                        var t: usize = 0;
                        var ty: i64 = -2;
                        while (ty <= 2) : (ty += 1) {
                            var tx: i64 = -2;
                            while (tx <= 2) : (tx += 1) {
                                const sy = mirror(@as(i64, @intCast(y)) + ty, hi);
                                const sx = mirror(@as(i64, @intCast(x)) + tx, wi);
                                taps[t] = in[@as(usize, @intCast(sy)) * w + @as(usize, @intCast(sx))];
                                t += 1;
                            }
                        }
                        var acc0 = taps[0] * self.kernelAt(k, 0);
                        var acc1 = taps[1] * self.kernelAt(k, 1);
                        var acc2 = taps[2] * self.kernelAt(k, 2);
                        var i: usize = 3;
                        while (i < 24) : (i += 3) {
                            acc0 = @mulAdd(f32, taps[i], self.kernelAt(k, i), acc0);
                            acc1 = @mulAdd(f32, taps[i + 1], self.kernelAt(k, i + 1), acc1);
                            acc2 = @mulAdd(f32, taps[i + 2], self.kernelAt(k, i + 2), acc2);
                        }
                        acc0 = @mulAdd(f32, taps[24], self.kernelAt(k, 24), acc0);
                        const result = clamp((acc1 + acc2) + acc0, mins[x], maxs[x]);
                        dst[x * N + ox] = result;
                    }
                }
            }
        }
    }
};

/// Number of entries of the symmetric (5H)x(5H) weight triangle.
fn triSize(h: usize) usize {
    const n = 5 * h;
    return n * (n + 1) / 2;
}

/// Index into the upper triangle of the (5H)x(5H) symmetric weight matrix
/// (libjxl: `5*H*my - my*(my-1)/2 + mx - my`).
fn triIndex(h: usize, my: usize, mx: usize) usize {
    const tri: usize = if (my == 0) 0 else my * (my - 1) / 2;
    return 5 * h * my - tri + mx - my;
}

test "triangular weight indices" {
    // H = 2 (factor 4): 10x10 symmetric matrix, 55 entries.
    var seen: [55]bool = @splat(false);
    var my: usize = 0;
    while (my < 10) : (my += 1) {
        var mx: usize = my;
        while (mx < 10) : (mx += 1) {
            const idx = triIndex(2, my, mx);
            try std.testing.expect(idx < 55);
            seen[idx] = true;
        }
    }
    for (seen) |s| try std.testing.expect(s);
    try std.testing.expectEqual(@as(usize, 55), triSize(2));
    try std.testing.expectEqual(@as(usize, 15), triSize(1));
    try std.testing.expectEqual(@as(usize, 210), triSize(4));
}

test "default weights parse and expand to near-unity kernels" {
    // Sanity for the transcribed weights and the triangular expansion:
    // every sub-pixel kernel must sum to approximately one (the NLEE
    // design); bit-level fidelity of the constants is cross-validated
    // against djxl output in the external test corpus.
    for ([_]u6{ 1, 2, 3 }) |shift| {
        const up = Upsampler.default(shift);
        const n: usize = @as(usize, 1) << @intCast(shift);
        var k: usize = 0;
        while (k < n * n) : (k += 1) {
            var sum: f32 = 0;
            for (0..25) |i| sum += up.kernel[k * 25 + i];
            try std.testing.expect(@abs(sum - 1.0) < 0.05);
        }
    }
}

test "upsample2 constant plane stays constant" {
    const alloc = std.testing.allocator;
    const up = Upsampler.default(1);
    const w = 8;
    const h = 6;
    var input: [w * h]f32 = @splat(0.25);
    var out: [w * 2 * h * 2]f32 = @splat(-1);
    try up.upsamplePlane(alloc, &input, w, h, &out, w * 2, h * 2);
    for (out) |v| try std.testing.expectEqual(@as(f32, 0.25), v);
}

test "upsample2 output size and border clamp" {
    const alloc = std.testing.allocator;
    const up = Upsampler.default(1);
    const w = 10;
    const h = 9;
    var input: [w * h]f32 = undefined;
    for (&input, 0..) |*v, i| v.* = @as(f32, @floatFromInt(i % 7)) / 7.0;
    // Output cropped to odd upsampled size (e.g. 19x17 from a 10x9 plane).
    const ow = 19;
    const oh = 17;
    var out: [ow * oh]f32 = @splat(-99);
    try up.upsamplePlane(alloc, &input, w, h, &out, ow, oh);
    // Every output is clamped to the 5x5 neighborhood min/max of the input,
    // hence inside [0, 6/7].
    for (out) |v| {
        try std.testing.expect(v >= 0.0 and v <= 6.0 / 7.0 + 1e-6);
    }
}

test "mirror matches libjxl semantics" {
    try std.testing.expectEqual(@as(i64, 0), mirror(-1, 5));
    try std.testing.expectEqual(@as(i64, 1), mirror(-2, 5));
    try std.testing.expectEqual(@as(i64, 4), mirror(5, 5));
    try std.testing.expectEqual(@as(i64, 3), mirror(6, 5));
    try std.testing.expectEqual(@as(i64, 2), mirror(2, 5));
    // Tiny images wrap repeatedly.
    try std.testing.expectEqual(@as(i64, 0), mirror(-2, 1));
    try std.testing.expectEqual(@as(i64, 0), mirror(2, 1));
    try std.testing.expectEqual(@as(i64, 1), mirror(-2, 2));
    try std.testing.expectEqual(@as(i64, 0), mirror(3, 2));
}
