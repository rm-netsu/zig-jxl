//! Bit helpers, port of libjxl `base/bits.h` and `base/common.h`.

const std = @import("std");

/// Base-2 logarithm, rounded down. Undefined for x == 0
/// (libjxl: FloorLog2Nonzero).
pub fn floorLog2Nonzero(x: anytype) usize {
    const T = @TypeOf(x);
    std.debug.assert(x != 0);
    return @typeInfo(T).int.bits - 1 - @clz(x);
}

/// Base-2 logarithm, rounded up (libjxl: CeilLog2Nonzero).
pub fn ceilLog2Nonzero(x: anytype) usize {
    const floor_log2 = floorLog2Nonzero(x);
    if ((x & (x - 1)) == 0) return floor_log2; // power of two
    return floor_log2 + 1;
}


/// Port of `Num0BitsBelowLS1Bit_Nonzero` (ctz).
pub inline fn num0BitsBelowLS1BitNonzero(x: anytype) usize {
    return @ctz(x);
}

/// Port of `Num0BitsAboveMS1Bit_Nonzero`; for x == 0 returns the bit width.
pub inline fn num0BitsAboveMS1Bit(x: anytype) usize {
    if (x == 0) return @bitSizeOf(@TypeOf(x));
    return @bitSizeOf(@TypeOf(x)) - @clz(x);
}

/// DivCeil from libjxl base/common.h.
pub fn divCeil(a: anytype, b: anytype) @TypeOf(a, b) {
    return (a + b - 1) / b;
}

/// RoundUpBitsToByteMultiple from libjxl base/common.h.
pub fn roundUpBitsToByteMultiple(a: anytype) @TypeOf(a) {
    return roundUpTo(a, 8);
}

/// RoundUpTo from libjxl base/common.h.
pub fn roundUpTo(a: anytype, b: anytype) @TypeOf(a, b) {
    return divCeil(a, b) * b;
}

test "log2 helpers" {
    try std.testing.expectEqual(@as(usize, 0), floorLog2Nonzero(@as(u32, 1)));
    try std.testing.expectEqual(@as(usize, 12), floorLog2Nonzero(@as(u32, 4096)));
    try std.testing.expectEqual(@as(usize, 11), floorLog2Nonzero(@as(u32, 4095)));
    try std.testing.expectEqual(@as(usize, 12), ceilLog2Nonzero(@as(u32, 4096)));
    try std.testing.expectEqual(@as(usize, 12), ceilLog2Nonzero(@as(u32, 4095)));
    try std.testing.expectEqual(@as(usize, 12), ceilLog2Nonzero(@as(u32, 2049)));
    try std.testing.expectEqual(@as(usize, 1), ceilLog2Nonzero(@as(u32, 2)));
    try std.testing.expectEqual(@as(usize, 0), ceilLog2Nonzero(@as(u32, 1)));
}

test "divCeil" {
    try std.testing.expectEqual(@as(usize, 3), divCeil(@as(usize, 17), 8));
    try std.testing.expectEqual(@as(usize, 2), divCeil(@as(usize, 16), 8));
    try std.testing.expectEqual(@as(usize, 0), divCeil(@as(usize, 0), 8));
}
