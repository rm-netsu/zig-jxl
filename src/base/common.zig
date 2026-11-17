//! Порт `lib/jxl/pack_signed.h` и мелких общих утилит libjxl.
//!
//! pixel_type  = int32_t (значения каналов/свойств)
//! pixel_type_w = int64_t (широкая арифметика предикторов)

const std = @import("std");

pub const pixel_type = i32;
pub const pixel_type_w = i64;

/// Порт `PackSigned` (pack_signed.h).
/// Неотрицательное X -> 2*X, отрицательное -X -> 2*X-1 (с заворачиванием).
pub inline fn packSigned(value: pixel_type) u32 {
    const u: u32 = @bitCast(value);
    // (~value) как u32, затем ЛОГИЧЕСКИЙ сдвиг вправо на 31 (как в C++).
    const nv: u32 = @bitCast(~value);
    const sign: u32 = (nv >> 31) -% 1;
    // u << 1 с заворачиванием (в C++ разрешено unsigned-переполнение).
    return (u *% 2) ^ sign;
}

/// Порт `UnpackSigned` (pack_signed.h). Обратен к packSigned.
pub inline fn unpackSigned(value: u32) pixel_type {
    const t: u32 = (value >> 1) ^ ((~value & 1) -% 1);
    return @bitCast(t);
}

/// Порт `jxl::Clamp1` (base/common.h): min(max(x, lo), hi).
pub inline fn clamp1(x: anytype, lo: @TypeOf(x), hi: @TypeOf(x)) @TypeOf(x) {
    return @min(@max(x, lo), hi);
}

/// DivCeil из base/common.h: (a + b - 1) / b.
pub inline fn divCeil(a: anytype, b: @TypeOf(a)) @TypeOf(a) {
    return (a + b - 1) / b;
}

test "packSigned/unpackSigned round trip" {
    const values = [_]pixel_type{ 0, 1, -1, 2, -2, 3, -3, 127, -128, 255, 510, -511, 1 << 30, -(1 << 30) };
    for (values) |v| {
        const packed_val = packSigned(v);
        try std.testing.expectEqual(v, unpackSigned(packed_val));
    }
    // Известные значения: 0 -> 0, 1 -> 2, -1 -> 1, -2 -> 3.
    try std.testing.expectEqual(@as(u32, 0), packSigned(0));
    try std.testing.expectEqual(@as(u32, 2), packSigned(1));
    try std.testing.expectEqual(@as(u32, 1), packSigned(-1));
    try std.testing.expectEqual(@as(u32, 3), packSigned(-2));
}
