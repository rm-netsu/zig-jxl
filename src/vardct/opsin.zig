//! XYB opsin color transforms and the sRGB transfer function, ports of
//! libjxl `enc_xyb.cc` (LinearRGBToXYB / ComputePremulAbsorb),
//! `dec_xyb.cc` + `dec_xyb-inl.h` (OpsinParams::Init / XybToRgb) and
//! `cms/opsin_params.h` (the frozen matrices and biases).
//!
//! Encode (linear sRGB -> XYB):
//!   mixed = kOpsinAbsorbanceMatrix * rgb + kOpsinAbsorbanceBias  (>= 0)
//!   gamma = cbrt(mixed) - cbrt(bias)
//!   X = (gamma0 - gamma1) / 2,  Y = (gamma0 + gamma1) / 2,  B = gamma2
//! Decode (XYB -> linear sRGB): exact inverse
//!   gamma0 = Y + X, gamma1 = Y - X, gamma2 = B
//!   gamma_i += cbrt(bias);  mixed_i = gamma_i^3 - bias
//!   linear = kDefaultInverseOpsinAbsorbanceMatrix * mixed
//! (libjxl writes the cbrt terms via its fast `CubeRootAndAdd`; this port
//! uses the accurate cube root — an encoder-side choice, see CHANGES.md
//! §36; the decoder-side arithmetic keeps libjxl's f32/FMA operation order
//! so decoded values track djxl to ~1 ulp per stage.)
//!
//! The sRGB transfer functions use the exact IEC 61966-2-1 curve (libjxl's
//! TF_SRGB is a fast rational approximation used on the encode side only,
//! where any valid curve is a legal encoder choice).

const std = @import("std");

/// libjxl `cms::kOpsinAbsorbanceMatrix` (frozen).
pub const kOpsinAbsorbanceMatrix = [3][3]f32{
    .{ 0.30, 0.622, 0.078 },
    .{ 0.23, 0.692, 0.078 },
    .{ 0.24342268924547819, 0.20476744424496821, 0.5518098665095536 },
};

/// libjxl `cms::kDefaultInverseOpsinAbsorbanceMatrix` (the spec values;
/// `INVERSE_OPSIN_FROM_SPEC 1`).
pub const kInverseOpsinAbsorbanceMatrix = [3][3]f32{
    .{ 11.031566901960783, -9.866943921568629, -0.16462299647058826 },
    .{ -3.254147380392157, 4.418770392156863, -0.16462299647058826 },
    .{ -3.6588512862745097, 2.7129230470588235, 1.9459282392156863 },
};

/// libjxl `cms::kOpsinAbsorbanceBias` (all three entries equal).
pub const kOpsinAbsorbanceBias: f32 = 0.0037930732552754493;

pub const cbrtBias: f32 = @floatCast(std.math.cbrt(@as(f64, kOpsinAbsorbanceBias)));

/// libjxl `kYToBRatio` (v0.12: exactly 1.0) — the default Y->B base
/// correlation of the CfL map DC header.
pub const kYToBRatio: f32 = 1.0;

/// libjxl `kDefaultQuantBias` (quantizer.h): the dequant bias correction
/// applied by `AdjustQuantBias` on decode.
pub const kDefaultQuantBias = [4]f32{
    1.0 - 0.05465007330715401,
    1.0 - 0.07005449891748593,
    1.0 - 0.049935103337343655,
    0.145,
};

/// Exact sRGB EOTF: encoded [0,1] -> linear.
pub fn srgbToLinear(v: f32) f32 {
    if (v <= 0.04045) return v / 12.92;
    const a = @as(f64, v) + 0.055;
    return @floatCast(std.math.pow(f64, a / 1.055, 2.4));
}

/// Exact inverse sRGB EOTF: linear -> encoded [0,1].
pub fn linearToSrgb(v: f32) f32 {
    if (v <= 0.0031308) return v * 12.92;
    return @floatCast(1.055 * std.math.pow(f64, @as(f64, v), 1.0 / 2.4) - 0.055);
}

/// Linear sRGB -> XYB for one pixel (libjxl `LinearRGBToXYB`,
/// `intensity_target` 255). Channel order: (X, Y, B).
pub fn linearRgbToXyb(r: f32, g: f32, b: f32) [3]f32 {
    const m = &kOpsinAbsorbanceMatrix;
    var mixed: [3]f32 = undefined;
    for (0..3) |i| {
        // OpsinAbsorbance: MulAdd chains (matching libjxl's FMA order).
        var v = @mulAdd(f32, m[i][2], b, kOpsinAbsorbanceBias);
        v = @mulAdd(f32, m[i][1], g, v);
        v = @mulAdd(f32, m[i][0], r, v);
        mixed[i] = @max(v, 0.0); // ZeroIfNegative
    }
    const g0 = cubeRoot(mixed[0]) - cbrtBias;
    const g1 = cubeRoot(mixed[1]) - cbrtBias;
    const g2 = cubeRoot(mixed[2]) - cbrtBias;
    return .{
        0.5 * (g0 - g1),
        0.5 * (g0 + g1),
        g2,
    };
}

/// XYB -> linear sRGB for one pixel (libjxl `XybToRgb` with the spec
/// inverse matrix; `intensity_target` 255). The operation order mirrors
/// dec_xyb-inl.h (FMA for the cubes and the matrix application).
pub fn xybToLinearRgb(x: f32, y: f32, b: f32) [3]f32 {
    var gamma: [3]f32 = .{ y + x, y - x, b };
    for (&gamma) |*gp| {
        gp.* += cbrtBias; // -= cbrt(neg bias)
    }
    var mixed: [3]f32 = undefined;
    for (0..3) |i| {
        const g = gamma[i];
        const g2 = g * g;
        mixed[i] = @mulAdd(f32, g2, g, -kOpsinAbsorbanceBias);
    }
    const im = &kInverseOpsinAbsorbanceMatrix;
    var out: [3]f32 = undefined;
    for (0..3) |i| {
        var t = im[i][0] * mixed[0];
        t = @mulAdd(f32, im[i][1], mixed[1], t);
        t = @mulAdd(f32, im[i][2], mixed[2], t);
        out[i] = t;
    }
    return out;
}

fn cubeRoot(v: f32) f32 {
    return @floatCast(std.math.cbrt(@as(f64, v)));
}

/// Gray output luminance (linear sRGB -> linear gray, Rec. 709 primaries).
pub fn linearLuminance(rgb: [3]f32) f32 {
    return 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2];
}

test "xyb roundtrip: linear <-> xyb <-> linear" {
    const cases = [_][3]f32{
        .{ 0.0, 0.0, 0.0 },
        .{ 1.0, 1.0, 1.0 },
        .{ 0.5, 0.5, 0.5 },
        .{ 1.0, 0.0, 0.0 },
        .{ 0.0, 1.0, 0.0 },
        .{ 0.0, 0.0, 1.0 },
        .{ 0.8, 0.2, 0.35 },
        .{ 0.05, 0.9, 0.6 },
    };
    for (cases) |rgb| {
        const xyb = linearRgbToXyb(rgb[0], rgb[1], rgb[2]);
        const back = xybToLinearRgb(xyb[0], xyb[1], xyb[2]);
        for (0..3) |i| {
            // Near-black pixels carry the f32 cbrt/cube roundtrip error
            // (~1e-6, the same magnitude libjxl's fast path shows).
            const tol = @max(2e-5 * @abs(rgb[i]), 2e-5);
            try std.testing.expectApproxEqAbs(rgb[i], back[i], tol);
        }
    }
}

test "srgb transfer roundtrip" {
    var v: f32 = 0;
    while (v <= 1.0) : (v += 0.001) {
        const lin = srgbToLinear(v);
        const back = linearToSrgb(lin);
        try std.testing.expectApproxEqAbs(v, back, 1e-6);
    }
}
