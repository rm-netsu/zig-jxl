//! Ports of the modular float conversions:
//!  - `intToFloat`  (lib/jxl/dec_modular.cc): custom [bits]-bit float with
//!    [exp_bits] exponent bits, stored as its bit pattern in a pixel_type,
//!    converted back to binary32.
//!  - `floatToInt`  (lib/jxl/enc_modular.cc): the inverse — a binary32 that
//!    is exactly representable in the custom float format converted to its
//!    bit-pattern integer (or, for integer formats, the libjxl
//!    `v * maxval + ±0.5` quantization used by the resampling pipeline).
//!
//! For bits == 32 / exp_bits == 8 (fp32) the conversion is the identity
//! (the bit pattern *is* the binary32 pattern).

const std = @import("std");

pub const FloatError = error{
    /// The float cannot be represented in the target format
    /// (libjxl: "Invalid float number: ... cannot be represented").
    UnrepresentableFloat,
    /// The exponent does not fit the target format
    /// (libjxl: "Invalid float exponent").
    FloatExponentOutOfRange,
    /// The mantissa loses precision (libjxl: "losing precision").
    FloatLosesPrecision,
};

/// Port of `int_to_float`: one sample.
pub fn intToFloat(value: i32, bits: u6, exp_bits: u6) f32 {
    if (bits == 32) {
        return @bitCast(value);
    }
    const exp_bias: i32 = (@as(i32, 1) << @intCast(exp_bits - 1)) - 1;
    const sign_shift: u5 = @intCast(bits - 1);
    const mant_bits: u5 = @intCast(bits - exp_bits - 1);
    const mant_shift: i32 = 23 - @as(i32, mant_bits);
    var f: u32 = @bitCast(value);
    const signbit: u32 = f >> sign_shift;
    f &= (@as(u32, 1) << sign_shift) - 1;
    if (f == 0) {
        return if (signbit != 0) -0.0 else 0.0;
    }
    var exp: i32 = @intCast(f >> mant_bits);
    var mantissa: u32 = f & ((@as(u32, 1) << mant_bits) - 1);
    if (exp == (@as(i32, 1) << @intCast(exp_bits)) - 1) {
        // NaN or infinity.
        var out: u32 = if (signbit != 0) 0x80000000 else 0;
        out |= @as(u32, 0b11111111) << 23;
        out |= mantissa << @intCast(mant_shift);
        return @bitCast(out);
    }
    mantissa <<= @intCast(mant_shift);
    // Try to normalize only if there is space for maneuver.
    if (exp == 0 and exp_bits < 8) {
        // subnormal number
        while ((mantissa & 0x800000) == 0) {
            mantissa <<= 1;
            exp -= 1;
        }
        exp += 1;
        // remove leading 1 because it is implicit now
        mantissa &= 0x7fffff;
    }
    exp -= exp_bias;
    // broke up the arbitrary float into its parts, now reassemble into
    // binary32
    exp += 127;
    // (libjxl JXL_ENSUREs exp >= 0; only reachable for exotic custom
    // formats, saturate to zero instead of failing the decode.)
    if (exp < 0) {
        return if (signbit != 0) -0.0 else 0.0;
    }
    var out: u32 = if (signbit != 0) 0x80000000 else 0;
    out |= @as(u32, @intCast(exp)) << 23;
    out |= mantissa;
    return @bitCast(out);
}

/// Port of the fp path of `float_to_int`: one sample. The binary32 `value`
/// must be exactly representable with [bits]/[exp_bits]; its custom-float
/// bit pattern (in [0, 2^bits)) is returned as the modular integer.
pub fn floatToInt(value: f32, bits: u6, exp_bits: u6) FloatError!i32 {
    if (bits == 32) {
        return @bitCast(value);
    }
    const exp_bias: i32 = (@as(i32, 1) << @intCast(exp_bits - 1)) - 1;
    const max_exp: i32 = (@as(i32, 1) << @intCast(exp_bits)) - 1;
    const sign: u32 = @as(u32, 1) << @intCast(bits - 1);
    const mant_bits: u5 = @intCast(bits - exp_bits - 1);
    const mant_shift: i32 = 23 - @as(i32, mant_bits);
    var f: u32 = @bitCast(value);
    const signbit: u32 = f >> 31;
    f &= 0x7fffffff;
    if (f == 0) {
        return @intCast(if (signbit != 0) sign else 0);
    }
    var exp: i32 = @as(i32, @intCast(f >> 23)) - 127;
    var mantissa: u32 = f & 0x007fffff;
    // broke up the binary32 into its parts, now reassemble into
    // arbitrary float
    if (exp == 128) {
        // NaN or infinity
        var out: u32 = if (signbit != 0) sign else 0;
        out |= @as(u32, @intCast((@as(u32, 1) << @intCast(exp_bits)) - 1)) << mant_bits;
        out |= mantissa >> @intCast(mant_shift);
        return @intCast(out);
    }
    exp += exp_bias;
    if (exp <= 0) { // will become a subnormal number
        // add implicit leading 1 to mantissa
        mantissa |= 0x00800000;
        if (exp < -@as(i32, mant_bits)) {
            return FloatError.UnrepresentableFloat;
        }
        mantissa >>= @intCast(1 - exp);
        exp = 0;
    }
    // exp should be representable in exp_bits, otherwise input was
    // invalid; max_exp is NaN or infinity
    if (exp >= max_exp) return FloatError.FloatExponentOutOfRange;
    if (mantissa & ((@as(u32, 1) << @intCast(mant_shift)) - 1) != 0) {
        return FloatError.FloatLosesPrecision;
    }
    mantissa >>= @intCast(mant_shift);
    var out: u32 = if (signbit != 0) sign else 0;
    out |= @as(u32, @intCast(exp)) << mant_bits;
    out |= mantissa;
    return @intCast(out);
}

/// Port of the integer path of `float_to_int` (enc_modular.cc): quantizes
/// a nominal [0, 1] float to the integer sample range with
/// `v * factor + (v < 0 ? -0.5 : 0.5)` truncated toward zero.
/// `bits > 22` uses double math like the C++ (`SingleFromSingleAccurate`
/// is the decode counterpart), otherwise float32 math.
pub fn floatToIntQuantize(value: f32, factor: f64, bits: u6) i32 {
    if (bits > 22) {
        const v: f64 = value;
        const d: f64 = if (value < 0) -0.5 else 0.5;
        const q = v * factor + d;
        return @intFromFloat(q);
    }
    const f: f32 = @floatCast(factor);
    const d: f32 = if (value < 0) -0.5 else 0.5;
    const q = value * f + d;
    return @intFromFloat(q);
}

/// Round-to-nearest, ties-to-even (libjxl `NearestInt`); used by the
/// decoder-side float->int output conversion.
pub fn nearestEven(v: f32) i32 {
    if (v >= 2147483520.0) return std.math.maxInt(i32);
    if (v <= -2147483520.0) return std.math.minInt(i32);
    const f = @floor(v);
    const diff = v - f;
    if (diff > 0.5) return @intFromFloat(f + 1);
    if (diff < 0.5) return @intFromFloat(f);
    // Exactly halfway: ties to even.
    const fi: i32 = @intFromFloat(f);
    return if (@mod(fi, 2) == 0) fi else fi + 1;
}

test "intToFloat/floatToInt identity for fp32" {
    const vals = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, 1e30, -2.5e-8, 3.14159 };
    for (vals) |v| {
        const i = try floatToInt(v, 32, 8);
        try std.testing.expectEqual(v, intToFloat(i, 32, 8));
    }
    // NaN/Inf roundtrip through the bit pattern
    const specials = [_]f32{ std.math.inf(f32), -std.math.inf(f32), std.math.nan(f32) };
    for (specials) |v| {
        const i = try floatToInt(v, 32, 8);
        try std.testing.expectEqual(@as(u32, @bitCast(v)), @as(u32, @bitCast(intToFloat(i, 32, 8))));
    }
}

test "fp16 roundtrip intToFloat(floatToInt)" {
    // Values exactly representable in binary16.
    const vals = [_]f32{ 0.0, 1.0, -1.0, 0.5, 0.25, 65504.0, 5.9604644775390625e-08, -1024.0, 0.0001220703125 };
    for (vals) |v| {
        const i = try floatToInt(v, 16, 5);
        try std.testing.expect(i >= 0 and i < (1 << 16));
        try std.testing.expectEqual(v, intToFloat(i, 16, 5));
    }
    // f32 values not representable in fp16 must be rejected.
    try std.testing.expectError(FloatError.FloatLosesPrecision, floatToInt(0.3, 16, 5));
    try std.testing.expectError(FloatError.FloatExponentOutOfRange, floatToInt(1e10, 16, 5));
}

test "intToFloat fp16 reference values" {
    // Known binary16 patterns -> binary32.
    // 0x3C00 = 1.0, 0x4000 = 2.0, 0xC000 = -2.0, 0x3800 = 0.5,
    // 0x0001 = 5.960464477539063e-08 (smallest subnormal),
    // 0x7BFF = 65504 (largest finite), 0x7C00 = +inf, 0xFC00 = -inf.
    const cases = [_]struct { bits: u16, want: f32 }{
        .{ .bits = 0x3C00, .want = 1.0 },
        .{ .bits = 0x4000, .want = 2.0 },
        .{ .bits = 0xC000, .want = -2.0 },
        .{ .bits = 0x3800, .want = 0.5 },
        .{ .bits = 0x0001, .want = 5.960464477539063e-08 },
        .{ .bits = 0x03FF, .want = 6.097555160522461e-05 },
        .{ .bits = 0x7BFF, .want = 65504.0 },
        .{ .bits = 0x7C00, .want = std.math.inf(f32) },
        .{ .bits = 0xFC00, .want = -std.math.inf(f32) },
        .{ .bits = 0x8000, .want = -0.0 },
    };
    for (cases) |c| {
        const got = intToFloat(@intCast(c.bits), 16, 5);
        try std.testing.expectEqual(c.want, got);
    }
    // roundtrip every pattern
    var bits: u32 = 0;
    while (bits < 65536) : (bits += 1) {
        const v = intToFloat(@intCast(bits), 16, 5);
        const back = try floatToInt(v, 16, 5);
        try std.testing.expectEqual(@as(i32, @intCast(bits)), back);
    }
}

test "floatToIntQuantize matches libjxl v*max+0.5 semantics" {
    // 8-bit: factor 255; values on the k/255 grid map back to k.
    var k: u32 = 0;
    while (k <= 255) : (k += 1) {
        const v = @as(f32, @floatFromInt(k)) / 255.0;
        try std.testing.expectEqual(@as(i32, @intCast(k)), floatToIntQuantize(v, 255.0, 8));
    }
    // 16-bit spot checks
    try std.testing.expectEqual(@as(i32, 0), floatToIntQuantize(0.0, 65535.0, 16));
    try std.testing.expectEqual(@as(i32, 65535), floatToIntQuantize(1.0, 65535.0, 16));
}
