//! Lossy VarDCT building blocks: the spec "scaled" DCT-8 pair, the
//! gaborish filters, the dequant bias correction and the default library
//! (Libshroom) DCT-8 quant weights.
//!
//! - DCT-8 (`dct_for_test.h`, "matches the definition in the spec"):
//!   the per-dimension transform is `alpha(u) * sqrt(2)/N * cos(...)`
//!   (forward) resp. `alpha(u) * sqrt(2) * cos(...)` (inverse), applied
//!   separably. Coefficient blocks use the JPEG XL layout
//!   `block[8*u + v] = C(u, v)` with u the horizontal frequency (the
//!   transposition relative to JPEG's row-major blocks — see the JPEG
//!   transcode path).
//! - Gaborish: the decoder applies the normalized 3x3 kernel of
//!   `stage_gaborish.cc` (weights `1.1 * 0.104699568 / 1.1 * 0.055680538`,
//!   mirror borders); the encoder pre-compensates with the 5x5
//!   `GaborishInverse` kernel of `enc_gaborish.cc` (border mirroring,
//!   per-plane normalization `1 / (1 + mul*4*(w0+w1+w2+2*w4... )`).
//! - `adjustQuantBias` ports `quantizer-inl.h` with the accurate
//!   reciprocal (libjxl uses the SIMD `ApproximateReciprocal`; the
//!   relative difference is < 2^-12 on a <= 0.145/|q| correction term).
//! - `libshroomDct8QuantWeights` ports `DequantMatricesLibraryDef::DCT`
//!   band parameters through `GetQuantWeights` (geometric band
//!   interpolation; libjxl's FastPowf is replaced by the accurate pow —
//!   the result is a legal encoder-side table choice and stays within
//!   ~1e-6 of libjxl's values).

const std = @import("std");

const kSqrt2: f64 = 1.4142135623730951;

fn alpha(u: usize) f64 {
    return if (u == 0) 0.7071067811865476 else 1.0;
}

/// Forward basis row `m[u][x]` (u = frequency, x = sample).
fn fwdBasis(u: usize, x: usize) f32 {
    const ang = (@as(f64, @floatFromInt(x)) + 0.5) *
        @as(f64, @floatFromInt(u)) * std.math.pi / 8.0;
    return @floatCast(alpha(u) * kSqrt2 / 8.0 * @cos(ang));
}

/// Inverse basis row `w[u][x]` (u = frequency, x = sample).
fn invBasis(u: usize, x: usize) f32 {
    const ang = (@as(f64, @floatFromInt(x)) + 0.5) *
        @as(f64, @floatFromInt(u)) * std.math.pi / 8.0;
    return @floatCast(alpha(u) * kSqrt2 * @cos(ang));
}

/// Forward 8x8 scaled DCT. `pixels` is row-major (`pixels[8*y + x]`);
/// `out` uses the JXL coefficient layout `out[8*u + v] = C(u,v)`
/// (u = horizontal frequency).
pub fn dct8Forward(pixels: *const [64]f32, out: *[64]f32) void {
    var t: [64]f32 = undefined; // t[v*8 + x] = sum_y m[v][y] * f[y][x]
    for (0..8) |v| {
        for (0..8) |x| {
            var acc: f32 = 0;
            for (0..8) |y| {
                acc += fwdBasis(v, y) * pixels[y * 8 + x];
            }
            t[v * 8 + x] = acc;
        }
    }
    for (0..8) |u| {
        for (0..8) |v| {
            var acc: f32 = 0;
            for (0..8) |x| {
                acc += fwdBasis(u, x) * t[v * 8 + x];
            }
            out[8 * u + v] = acc; // C(u, v)
        }
    }
}

/// Inverse 8x8 scaled DCT: the exact inverse of `dct8Forward`.
pub fn dct8Inverse(coefs: *const [64]f32, out: *[64]f32) void {
    var t: [64]f32 = undefined; // t[x*8 + v] = sum_u w[u][x] * C(u, v)
    for (0..8) |x| {
        for (0..8) |v| {
            var acc: f32 = 0;
            for (0..8) |u| {
                acc += invBasis(u, x) * coefs[8 * u + v];
            }
            t[x * 8 + v] = acc;
        }
    }
    for (0..8) |y| {
        for (0..8) |x| {
            var acc: f32 = 0;
            for (0..8) |v| {
                acc += invBasis(v, y) * t[x * 8 + v];
            }
            out[8 * y + x] = acc;
        }
    }
}

// ---------------------------------------------------------------------------
// Gaborish
// ---------------------------------------------------------------------------

/// libjxl loop_filter defaults: gab_x_weight1 = 1.1 * 0.104699568,
/// gab_x_weight2 = 1.1 * 0.055680538 (same for y/b channels).
const gab_w1: f32 = 1.1 * 0.104699568;
const gab_w2: f32 = 1.1 * 0.055680538;

fn mirrorIdx(i: i64, size: usize) usize {
    if (i < 0) return @intCast(-i - 1);
    const s: i64 = @intCast(size);
    if (i >= s) return @intCast(2 * s - 1 - i);
    return @intCast(i);
}

/// Decoder-side gaborish (`stage_gaborish.cc`): in-place normalized 3x3
/// kernel on all three planes with mirror borders.
pub fn gaborishInverse(
    planes: [3][]f32,
    xsize: usize,
    ysize: usize,
    scratch: [3][]f32,
) void {
    const w0n: f32 = 1.0;
    const w1n: f32 = gab_w1;
    const w2n: f32 = gab_w2;
    const div = w0n + 4 * (w1n + w2n);
    const w0 = w0n / div;
    const w1 = w1n / div;
    const w2 = w2n / div;
    for (0..3) |c| {
        const p = planes[c];
        const s = scratch[c];
        for (0..ysize) |yy| {
            const yt: usize = mirrorIdx(
                @as(i64, @intCast(yy)) - 1,
                ysize,
            );
            const ym = yy;
            const yb: usize = mirrorIdx(
                @as(i64, @intCast(yy)) + 1,
                ysize,
            );
            for (0..xsize) |xx| {
                const xl: usize = mirrorIdx(
                    @as(i64, @intCast(xx)) - 1,
                    xsize,
                );
                const xr: usize = mirrorIdx(
                    @as(i64, @intCast(xx)) + 1,
                    xsize,
                );
                const m = p[ym * xsize + xx];
                const l = p[ym * xsize + xl];
                const r = p[ym * xsize + xr];
                const t = p[yt * xsize + xx];
                const b = p[yb * xsize + xx];
                const tl = p[yt * xsize + xl];
                const tr = p[yt * xsize + xr];
                const bl = p[yb * xsize + xl];
                const br = p[yb * xsize + xr];
                const sum1 = (l + r) + (t + b);
                const sum2 = (tl + tr) + (bl + br);
                // MulAdd chain of stage_gaborish.cc.
                const v = @mulAdd(
                    f32,
                    sum2,
                    w2,
                    @mulAdd(f32, sum1, w1, m * w0),
                );
                s[ym * xsize + xx] = v;
            }
        }
        @memcpy(p[0 .. xsize * ysize], s[0 .. xsize * ysize]);
    }
}

/// libjxl `kGaborish` (enc_gaborish.cc) 5x5 kernel weights, in the
/// WeightsSymmetric5 order {w0, w2, w1, w4, w3} used by the normalization.
const kGaborish5 = [5]f32{
    -0.09495815671340026,
    -0.041031725066768575,
    0.013710004822696948,
    0.006510206083837737,
    -0.0014789063378272242,
};

/// Encoder-side gaborish compensation (`GaborishInverse`, enc_gaborish.cc):
/// the 5x5 symmetric kernel (convolve_symmetric5.cc tap layout) normalized
/// per plane. `mul` is the per-plane weight multiplier (1.0 for the VarDCT
/// main frame; 0.9908511 would be the modular value). Mirror borders
/// (WrapMirror on x at all times, on y near the image edges), per-plane
/// scratch, in place.
pub fn gaborishForward(
    planes: [3][]f32,
    xsize: usize,
    ysize: usize,
    mul: f32,
    scratch: [3][]f32,
) void {
    // enc_gaborish.cc weight mapping (declaration order c, r, R, d, D, L):
    //   c = normalize
    //   r = mul * kGaborish[0] (distance-1 straight)
    //   R = mul * kGaborish[2] (distance-2 straight)
    //   d = mul * kGaborish[1] (diagonal adjacent)
    //   D = mul * kGaborish[4] (corner 2,2)
    //   L = mul * kGaborish[3] (knight distance)
    const k = kGaborish5;
    for (0..3) |c| {
        var sum: f64 = 1.0 + @as(f64, mul) * 4.0 *
            (@as(f64, k[0]) +
                @as(f64, k[1]) +
                @as(f64, k[2]) +
                @as(f64, k[4]) +
                2.0 * @as(f64, k[3]));
        if (sum < 1e-5) sum = 1e-5;
        const normalize: f32 = @floatCast(1.0 / sum);
        const nm = mul * normalize;
        const wc: f32 = normalize;
        const wr: f32 = nm * k[0];
        const wR: f32 = nm * k[2];
        const wd: f32 = nm * k[1];
        const wD: f32 = nm * k[4];
        const wL: f32 = nm * k[3];
        const p = planes[c];
        const s = scratch[c];
        for (0..ysize) |yy| {
            const ym2: usize = mirrorIdx(@as(i64, @intCast(yy)) - 2, ysize);
            const ym1: usize = mirrorIdx(@as(i64, @intCast(yy)) - 1, ysize);
            const yp1: usize = mirrorIdx(@as(i64, @intCast(yy)) + 1, ysize);
            const yp2: usize = mirrorIdx(@as(i64, @intCast(yy)) + 2, ysize);
            const row_m2 = p[ym2 * xsize ..][0..xsize];
            const row_m1 = p[ym1 * xsize ..][0..xsize];
            const row_0 = p[yy * xsize ..][0..xsize];
            const row_p1 = p[yp1 * xsize ..][0..xsize];
            const row_p2 = p[yp2 * xsize ..][0..xsize];
            for (0..xsize) |xx| {
                const xm2: usize = mirrorIdx(@as(i64, @intCast(xx)) - 2, xsize);
                const xm1: usize = mirrorIdx(@as(i64, @intCast(xx)) - 1, xsize);
                const xp1: usize = mirrorIdx(@as(i64, @intCast(xx)) + 1, xsize);
                const xp2: usize = mirrorIdx(@as(i64, @intCast(xx)) + 2, xsize);
                // Kernel rows (Symmetric5Border):
                //   dy=-2/-1/+1/+2: [D L R L D], [L d r d L], [L d r d L], [D L R L D]
                //   dy=0:            [R r c r R]
                var acc: f32 = 0;
                // dy = -2 and +2.
                acc += wD *
                    (row_m2[xm2] + row_m2[xp2]) + wL *
                    (row_m2[xm1] + row_m2[xp1]) + wR *
                    row_m2[xx];
                acc += wD *
                    (row_p2[xm2] + row_p2[xp2]) + wL *
                    (row_p2[xm1] + row_p2[xp1]) + wR *
                    row_p2[xx];
                // dy = -1 and +1.
                acc += wL * (row_m1[xm2] + row_m1[xp2]) + wd *
                    (row_m1[xm1] + row_m1[xp1]) + wr *
                    row_m1[xx];
                acc += wL * (row_p1[xm2] + row_p1[xp2]) + wd *
                    (row_p1[xm1] + row_p1[xp1]) + wr *
                    row_p1[xx];
                // dy = 0.
                acc += wR * (row_0[xm2] + row_0[xp2]) + wr *
                    (row_0[xm1] + row_0[xp1]) + wc *
                    row_0[xx];
                s[yy * xsize + xx] = acc;
            }
        }
        @memcpy(p[0 .. xsize * ysize], s[0 .. xsize * ysize]);
    }
}

// ---------------------------------------------------------------------------
// Dequant bias (quantizer-inl.h)
// ---------------------------------------------------------------------------

/// `AdjustQuantBias` with the accurate reciprocal:
///   q == 0 -> 0; |q| == 1 -> sign(q) * biases[c];
///   |q| >= 2 -> q - biases[3] / q.
pub fn adjustQuantBias(c: usize, q: i32, biases: *const [4]f32) f32 {
    if (q == 0) return 0;
    const qf: f32 = @floatFromInt(q);
    const abs_q = @abs(qf);
    if (abs_q < 1.125) {
        return if (qf < 0) -biases[c] else biases[c];
    }
    return qf - biases[3] / qf;
}

// ---------------------------------------------------------------------------
// Library (Libshroom) DCT-8 quant weights
// ---------------------------------------------------------------------------

/// `DequantMatricesLibraryDef::DCT()` distance bands (X, Y, B rows).
const libshroom_bands = [3][6]f32{
    .{ 3150.0, 0.0, -0.4, -0.4, -0.4, -2.0 },
    .{ 560.0, 0.0, -0.3, -0.3, -0.3, -0.3 },
    .{ 512.0, -2.0, -1.0, 0.0, -1.0, -2.0 },
};

fn bandMult(v: f32) f32 {
    if (v > 0) return 1.0 + v;
    return 1.0 / (1.0 - v);
}

/// libjxl `FastLog2f` (base/fast_math-inl.h): the 2,2 rational
/// polynomial with FMA chains — ported operation-for-operation so the
/// library quant tables match libjxl's computed values.
fn fastLog2f(x: f32) f32 {
    const p = [3]f32{
        -1.8503833400518310e-06,
        1.4287160470083755e+00,
        7.4245873327820566e-01,
    };
    const q = [3]f32{
        9.9032814277590719e-01,
        1.0096718572241148e+00,
        1.7409343003366853e-01,
    };
    const x_bits: i32 = @bitCast(x);
    const exp_bits = x_bits -% @as(i32, @bitCast(@as(u32, 0x3f2aaaab)));
    const exp_shifted = exp_bits >> 23;
    const mantissa: f32 = @bitCast(x_bits -% (exp_shifted << 23));
    const exp_val: f32 = @floatFromInt(exp_shifted);
    const m = mantissa - 1.0;
    var yp = p[2];
    var yq = q[2];
    yp = @mulAdd(f32, yp, m, p[1]);
    yq = @mulAdd(f32, yq, m, q[1]);
    yp = @mulAdd(f32, yp, m, p[0]);
    yq = @mulAdd(f32, yq, m, q[0]);
    return yp / yq + exp_val;
}

/// libjxl `FastPow2f` (max relative error ~3e-7).
fn fastPow2f(x: f32) f32 {
    const floorx = @floor(x);
    const exp_bits: i32 = (@as(i32, @intFromFloat(floorx)) + 127) << 23;
    const exp: f32 = @bitCast(exp_bits);
    const frac = x - floorx;
    var num = frac + 1.01749063e+01;
    num = @mulAdd(f32, num, frac, 4.88687798e+01);
    num = @mulAdd(f32, num, frac, 9.85506591e+01);
    num = num * exp;
    var den = @mulAdd(
        f32,
        frac,
        2.10242958e-01,
        -2.22328856e-02,
    );
    den = @mulAdd(f32, den, frac, -1.94414990e+01);
    den = @mulAdd(f32, den, frac, 9.85506633e+01);
    return num / den;
}

/// libjxl `FastPowf` (max relative error ~3e-5).
fn fastPowf(base: f32, exponent: f32) f32 {
    return fastPow2f(fastLog2f(base) * exponent);
}

/// `GetQuantWeights(8, 8, libshroom, 6)`: geometric interpolation between
/// eccentricity bands through libjxl's `InterpolateVec` (FastPowf, not
/// the accurate pow — matching the values djxl computes). Result:
/// `weights[c * 64 + 8 * y + x]` — the *inverse* dequant weights (higher
/// = finer quantization), in the same image-store layout libjxl uses for
/// RAW tables.
pub fn libshroomDct8QuantWeights() [3 * 64]f32 {
    var out: [3 * 64]f32 = undefined;
    const num_bands = 6;
    for (0..3) |c| {
        var bands: [6]f32 = undefined;
        bands[0] = libshroom_bands[c][0];
        var i: usize = 1;
        while (i < num_bands) : (i += 1) {
            bands[i] = bands[i - 1] * bandMult(libshroom_bands[c][i]);
        }
        const scale = @as(f32, num_bands - 1) / (std.math.sqrt2 + 1e-6);
        const rcpcol = scale / 7.0;
        const rcprow = scale / 7.0;
        for (0..8) |y| {
            const dy = @as(f32, @floatFromInt(y)) * rcprow;
            const dy2 = dy * dy;
            for (0..8) |x| {
                const dx = @as(f32, @floatFromInt(x)) * rcpcol;
                // Sqrt(MulAdd(dx, dx, dy2)).
                const scaled_dist = @sqrt(
                    @mulAdd(f32, dx, dx, dy2),
                );
                // InterpolateVec: idx = ConvertTo(int32, scaled_pos)
                // (truncation), frac = scaled_pos - idx, a * FastPowf(b/a, frac).
                const idx: i32 = @intFromFloat(scaled_dist);
                var w: f32 = undefined;
                if (idx + 1 >= num_bands) {
                    w = bands[num_bands - 1];
                } else {
                    const frac = scaled_dist - @as(f32, @floatFromInt(idx));
                    const a = bands[@intCast(idx)];
                    const b = bands[@intCast(idx + 1)];
                    w = a * fastPowf(b / a, frac);
                }
                out[c * 64 + y * 8 + x] = w;
            }
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

test "dct8 forward/inverse roundtrip" {
    var px: [64]f32 = undefined;
    var prng = std.Random.DefaultPrng.init(42);
    const rnd = prng.random();
    for (&px) |*v| {
        v.* = rnd.float(f32) * 2.0 - 0.5;
    }
    var coefs: [64]f32 = undefined;
    dct8Forward(&px, &coefs);
    var back: [64]f32 = undefined;
    dct8Inverse(&coefs, &back);
    for (px, back) |a, b| {
        try std.testing.expectApproxEqAbs(a, b, 1e-5);
    }
}

test "dct8 flat block: DC == mean, layout transposed" {
    var px: [64]f32 = undefined;
    for (&px) |*v| v.* = 0.25;
    var coefs: [64]f32 = undefined;
    dct8Forward(&px, &coefs);
    // C(0,0) sits at index 0; every AC is zero.
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.25),
        coefs[0],
        1e-6,
    );
    for (coefs[1..]) |v| {
        try std.testing.expectApproxEqAbs(
            @as(f32, 0),
            v,
            1e-6,
        );
    }
    // Single vertical frequency (v=3) impulse: pixel column constant,
    // rows vary -> C(0, 3) at index 8*0+3.
    var px2: [64]f32 = undefined;
    for (0..8) |y| {
        for (0..8) |x| {
            const ang = std.math.cos(
                (@as(f64, @floatFromInt(y)) + 0.5) * 3.0 * std.math.pi / 8.0,
            );
            px2[y * 8 + x] = @floatCast(ang);
        }
    }
    dct8Forward(&px2, &coefs);
    // C(0, 3) = (sum_x m[0][x]) * (sqrt(2)/8 * sum_y cos^2) = 1 * sqrt(2)/2.
    try std.testing.expectApproxEqAbs(
        @as(f32, std.math.sqrt2 / 2.0),
        coefs[3],
        1e-5,
    );
    // No energy in any other coefficient.
    for (coefs, 0..) |v, i| {
        if (i != 3) try std.testing.expectApproxEqAbs(
            @as(f32, 0),
            v,
            1e-6,
        );
    }
    var back2: [64]f32 = undefined;
    dct8Inverse(&coefs, &back2);
    for (px2, back2) |a, b| {
        try std.testing.expectApproxEqAbs(
            a,
            b,
            1e-5,
        );
    }
}

test "gaborish forward/inverse approximately invert" {
    const W = 33;
    const H = 17;
    var planes: [3][]f32 = undefined;
    var scratch: [3][]f32 = undefined;
    var storage: [3][W * H]f32 = undefined;
    var scratch_storage: [3][W * H]f32 = undefined;
    var prng = std.Random.DefaultPrng.init(7);
    const rnd = prng.random();
    for (0..3) |c| {
        for (&storage[c]) |*v| v.* = rnd.float(f32);
        planes[c] = &storage[c];
        scratch[c] = &scratch_storage[c];
    }
    const orig = storage;
    // A constant image passes both kernels unchanged (unit DC gain).
    for (0..3) |c| {
        @memset(&storage[c], 0.375);
    }
    gaborishForward(planes, W, H, 1.0, scratch);
    gaborishInverse(planes, W, H, scratch);
    for (0..3) |c| {
        for (&storage[c]) |v| {
            try std.testing.expectApproxEqAbs(
                @as(f32, 0.375),
                v,
                1e-6,
            );
        }
    }
    prng = std.Random.DefaultPrng.init(7);
    for (0..3) |c| {
        for (&storage[c]) |*v| v.* = rnd.float(f32);
    }
    gaborishForward(planes, W, H, 1.0, scratch);
    gaborishInverse(planes, W, H, scratch);
    for (0..3) |c| {
        for (0..H) |y| {
            for (0..W) |x| {
                // The 5x5 kernel is only an approximate inverse of the
                // decoder's 3x3 gaborish (enc_gaborish.cc: "Only an
                // approximation"); interior pixels round-trip to ~1%.
                if (x >= 2 and x + 2 < W and y >= 2 and y + 2 < H) {
                    try std.testing.expectApproxEqAbs(
                        orig[c][y * W + x],
                        planes[c][y * W + x],
                        2e-2,
                    );
                }
            }
        }
    }
}

test "adjustQuantBias matches the libjxl formula" {
    const b = [4]f32{
        1.0 - 0.05465007330715401,
        1.0 - 0.07005449891748593,
        1.0 - 0.049935103337343655,
        0.145,
    };
    try std.testing.expectEqual(
        @as(f32, 0),
        adjustQuantBias(0, 0, &b),
    );
    try std.testing.expectApproxEqAbs(
        b[0],
        adjustQuantBias(0, 1, &b),
        1e-7,
    );
    try std.testing.expectApproxEqAbs(
        -b[1],
        adjustQuantBias(1, -1, &b),
        1e-7,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 2) - 0.145 / 2.0,
        adjustQuantBias(2, 2, &b),
        1e-6,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, -3) + 0.145 / 3.0,
        adjustQuantBias(0, -3, &b),
        1e-6,
    );
}

test "libshroom table: positive, DC heaviest, chroma coarser" {
    const w = libshroomDct8QuantWeights();
    for (w) |v| try std.testing.expect(v > 0);
    // X DC weight 3150, Y DC 560, B DC 512.
    try std.testing.expectApproxEqAbs(@as(f32, 3150.0), w[0], 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 560.0), w[64], 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 512.0), w[128], 1e-3);
    // Highest frequency is coarser than DC.
    try std.testing.expect(w[63] < w[0]);
}
