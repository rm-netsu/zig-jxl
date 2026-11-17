//! Codestream field coders beyond base/fields: enums, names, F16.
//! Ports of libjxl `field_encodings.h` EnumCoder and `fields.cc` F16Coder.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const U32Enc = fields.U32Enc;

/// JXL enum coding: 00 -> 0, 01 -> 1, 10xxxx -> 2..17, 11yyyyyy -> 18..81.
const enum_enc = U32Enc.init(fields.val(0), fields.val(1), fields.bitsOffset(4, 2), fields.bitsOffset(6, 18));

pub fn EnumCoder(comptime E: type) type {
    return struct {
        const max_value: u32 = blk: {
            var m: u32 = 0;
            for (@typeInfo(E).@"enum".fields) |f| {
                m = @max(m, f.value);
            }
            break :blk m;
        };

        pub fn read(default_value: E, br: *BitReader) !E {
            _ = default_value; // only affects bit-width selection in libjxl CanEncode
            const raw = U32Coder.read(enum_enc, br);
            if (raw > max_value) return error.InvalidEnum;
            const value: E = @enumFromInt(raw);
            if (!isValid(value)) return error.InvalidEnum;
            return value;
        }

        pub fn write(value: E, writer: *BitWriter) !void {
            try U32Coder.write(enum_enc, @intFromEnum(value), writer);
        }

        pub fn bitSize(value: E) usize {
            var sw = fields.SizeWriter{};
            U32Coder.write(enum_enc, @intFromEnum(value), &sw) catch unreachable;
            return sw.size;
        }

        pub fn isValid(value: E) bool {
            return E.valid_mask & (@as(u64, 1) << @intCast(@intFromEnum(value))) != 0;
        }
    };
}

/// Port of F16Coder::Read: IEEE half, subnormals supported, inf/NaN rejected.
pub fn readF16(br: *BitReader) !f32 {
    const bits16: u16 = @intCast(br.readFixedBits(16));
    const sign: u32 = bits16 >> 15;
    const biased_exp: u32 = (bits16 >> 10) & 0x1F;
    const mantissa: u32 = bits16 & 0x3FF;

    if (biased_exp == 31) return error.InvalidF16;

    if (biased_exp == 0) {
        var value = (1.0 / 16384.0) * (@as(f32, @floatFromInt(mantissa)) * (1.0 / 1024.0));
        if (sign != 0) value = -value;
        return value;
    }
    const biased_exp32: u32 = biased_exp + (127 - 15);
    const mantissa32: u32 = mantissa << 13;
    const bits32: u32 = (sign << 31) | (biased_exp32 << 23) | mantissa32;
    return @bitCast(bits32);
}

/// Port of F16Coder::Write with round-to-nearest-even conversion.
pub fn writeF16(value: f32, writer: *BitWriter) !void {
    var f = value;
    const sign: u16 = if (std.math.signbit(f)) 1 else 0;
    if (f == 0) {
        writer.write(16, @as(u64, sign) << 15);
        return;
    }
    if (std.math.isNan(f) or std.math.isInf(f)) return error.InvalidF16;
    if (f < 0) f = -f;

    const bits32: u32 = @bitCast(f);
    const biased_exp: i32 = @intCast((bits32 >> 23) & 0xFF);
    const mantissa32: u32 = bits32 & 0x7FFFFF;

    var bits16: u16 = undefined;
    const unbiased = biased_exp - 127;
    if (unbiased < -14) {
        // Subnormal (or underflow to zero).
        const shift: u5 = @intCast(@min(24, -14 - unbiased));
        const full: u32 = mantissa32 | 0x800000;
        var mant: u32 = full >> @intCast(shift + 13);
        // Round to nearest even.
        const rem: u32 = full & ((@as(u32, 1) << @intCast(shift + 13)) - 1);
        const halfway: u32 = @as(u32, 1) << @intCast(shift + 12);
        if (rem > halfway or (rem == halfway and (mant & 1) != 0)) mant += 1;
        if (mant >= 1024) {
            bits16 = 1 << 10; // smallest normal
        } else {
            bits16 = @intCast(mant);
        }
    } else if (unbiased > 15) {
        return error.InvalidF16;
    } else {
        const mant_shift: u5 = 13;
        var mant: u32 = mantissa32 >> mant_shift;
        const rem: u32 = mantissa32 & ((@as(u32, 1) << mant_shift) - 1);
        if (rem > 0x1000 or (rem == 0x1000 and (mant & 1) != 0)) {
            mant += 1;
            if (mant == 1024) {
                mant = 0;
                return writeF16Unchecked(sign, @intCast(unbiased + 1), mant, writer);
            }
        }
        bits16 = (@as(u16, @intCast(unbiased + 15)) << 10) | @as(u16, @intCast(mant));
    }
    writer.write(16, (@as(u64, sign) << 15) | bits16);
}

fn writeF16Unchecked(sign: u16, biased_exp: u16, mant: u32, writer: *BitWriter) !void {
    writer.write(16, (@as(u64, sign) << 15) | (@as(u64, biased_exp) << 10) | mant);
}

/// Port of VisitNameString: length via U32, then raw bytes.
pub const Name = struct {
    value: []u8 = &.{},
    allocator: ?std.mem.Allocator = null,

    const len_enc = U32Enc.init(fields.val(0), fields.bitsDistr(4), fields.bitsOffset(5, 16), fields.bitsOffset(10, 48));

    pub fn read(self: *Name, allocator: std.mem.Allocator, br: *BitReader) !void {
        const length = U32Coder.read(len_enc, br);
        if (self.allocator) |a| a.free(self.value);
        self.value = try allocator.alloc(u8, length);
        self.allocator = allocator;
        for (self.value) |*ch| {
            ch.* = @intCast(br.readFixedBits(8));
        }
    }

    pub fn readLength(br: *BitReader) u32 {
        return U32Coder.read(len_enc, br);
    }

    pub fn write(value: []const u8, writer: *BitWriter) !void {
        try U32Coder.write(len_enc, @intCast(value.len), writer);
        for (value) |ch| writer.write(8, ch);
    }
};

test "f16 roundtrip" {
    const alloc = std.testing.allocator;
    var w = BitWriter.init(alloc);
    defer w.deinit();
    const values = [_]f32{ 0, -0.0, 1, -1, 0.5, 65504, 1e-5, -0.25, 100 };
    for (values) |v| try writeF16(v, &w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    for (values) |v| {
        const got = try readF16(&br);
        // 1e-5 is subnormal in half precision; allow its coarse granularity.
        try std.testing.expectApproxEqRel(v, got, 5e-3);
    }
    try br.close();
}
