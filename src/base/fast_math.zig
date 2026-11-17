//! Fast log2 approximation, port of libjxl `base/fast_math-inl.h`
//! `FastLog2f` (highway scalar semantics): 2,2 rational polynomial with the
//! same coefficients and range reduction. Used only by encoder heuristics
//! (clustering, LZ77 cost estimation), so tiny floating point differences
//! vs. libjxl do not affect bitstream validity.

const std = @import("std");

/// Computes base-2 logarithm like std::log2. Undefined if negative / NaN.
/// L1 error ~3.9E-6.
pub fn fastLog2f(x: f32) f32 {
    // 2,2 rational polynomial approximation of std::log1p(x) / std::log(2).
    const p = [3]f32{
        -1.8503833400518310E-06,
        1.4287160470083755E+00,
        7.4245873327820566E-01,
    };
    const q = [3]f32{
        9.9032814277590719E-01,
        1.0096718572241148E+00,
        1.7409343003366853E-01,
    };

    const x_bits: i32 = @bitCast(x);

    // Range reduction to [-1/3, 1/3] - 3 integer, 2 float ops.
    const exp_bits = x_bits -% 0x3f2aaaab; // = 2/3
    // Shifted exponent = log2; also used to clear mantissa.
    const exp_shifted = exp_bits >> 23;
    const mantissa: f32 = @bitCast(x_bits -% (exp_shifted << 23));
    const exp_val: f32 = @floatFromInt(exp_shifted);
    const m = mantissa - 1.0;

    // EvalRationalPolynomial (degree 2 / degree 2), Horner form
    // p0 + x*(p1 + x*p2) over q0 + x*(q1 + x*q2), fused multiply-adds.
    const num = @mulAdd(f32, m, @mulAdd(f32, m, p[2], p[1]), p[0]);
    const den = @mulAdd(f32, m, @mulAdd(f32, m, q[2], q[1]), q[0]);
    return num / den + exp_val;
}

test "fastLog2f close to std log2" {
    var x: f32 = 1.0 / 64.0;
    while (x < 70000.0) : (x *= 1.337) {
        const approx = fastLog2f(x);
        const exact = @log2(x);
        // L1 error ~3.9E-6 in libjxl; allow a bit more for f32 math.
        try std.testing.expectApproxEqAbs(exact, approx, 0.01);
    }
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), fastLog2f(1.0), 1e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 12.0), fastLog2f(4096.0), 1e-3);
}
