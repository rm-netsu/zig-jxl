//! Field coders: U32Coder, U64Coder, variable-length integers and the
//! LZ77Params bundle. Port of the parts of libjxl `field_encodings.h`,
//! `fields.h/cc`, `dec_ans.cc` (DecodeVarLenUint8/16) and `enc_ans.cc`
//! (StoreVarLenUint8/16) used by the ANS stack.

const std = @import("std");
const bit_io = @import("bit_io.zig");
const bits = @import("bits.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;

/// Distribution of U32 values for one particular selector
/// (libjxl: U32Distr). Represents either a power-of-two-sized range, or a
/// single direct value.
pub const U32Distr = struct {
    pub const kDirect: u32 = 0x80000000;

    d: u32,

    pub fn isDirect(self: U32Distr) bool {
        return (self.d & kDirect) != 0;
    }

    /// Only call if isDirect().
    pub fn direct(self: U32Distr) u32 {
        return self.d & (kDirect - 1);
    }

    /// Only call if !isDirect().
    pub fn extraBits(self: U32Distr) usize {
        return @as(usize, self.d & 0x1F) + 1;
    }

    pub fn offset(self: U32Distr) u32 {
        return (self.d >> 5) & 0x3FFFFFF;
    }
};

/// A direct-coded 31-bit value occupying 2 bits in the bitstream.
pub fn val(value: u32) U32Distr {
    return .{ .d = value | U32Distr.kDirect };
}

/// Value - `offset` will be signaled in `bits` extra bits.
pub fn bitsOffset(bits_count: u32, offset_v: u32) U32Distr {
    return .{ .d = ((bits_count - 1) & 0x1F) + ((offset_v & 0x3FFFFFF) << 5) };
}

/// Value will be signaled in `bits` extra bits.
pub fn bitsDistr(bits_count: u32) U32Distr {
    return bitsOffset(bits_count, 0);
}

pub const U32Enc = struct {
    d: [4]U32Distr,

    pub fn init(d0: U32Distr, d1: U32Distr, d2: U32Distr, d3: U32Distr) U32Enc {
        return .{ .d = .{ d0, d1, d2, d3 } };
    }

    pub fn getDistr(self: U32Enc, selector: u32) U32Distr {
        std.debug.assert(selector < 4);
        return self.d[selector];
    }
};

// ---------------------------------------------------------------------------
// U32Coder
// ---------------------------------------------------------------------------

pub const U32Coder = struct {
    pub fn read(enc: U32Enc, reader: *BitReader) u32 {
        const selector: u32 = @intCast(reader.readFixedBits(2));
        const d = enc.getDistr(selector);
        if (d.isDirect()) {
            return d.direct();
        } else {
            return @as(u32, @intCast(reader.readBits(d.extraBits()))) +% d.offset();
        }
    }

    pub fn canEncode(enc: U32Enc, value: u32) bool {
        var selector: u32 = undefined;
        var total_bits: usize = undefined;
        return chooseSelector(enc, value, &selector, &total_bits);
    }

    pub fn write(enc: U32Enc, value: u32, writer: anytype) !void {
        var selector: u32 = 0;
        var total_bits: usize = 0;
        if (!chooseSelector(enc, value, &selector, &total_bits)) {
            return error.NoFeasibleSelector;
        }
        const d = enc.getDistr(selector);
        writer.write(2, selector);
        if (d.isDirect()) {
            // nothing
        } else {
            writer.write(d.extraBits(), value - d.offset());
        }
    }

    /// "private" in libjxl: selects the smallest encoding for `value`.
    fn chooseSelector(enc: U32Enc, value: u32, selector: *u32, total_bits: *usize) bool {
        selector.* = 0;
        // It is difficult to verify whether the distributions are sorted, so
        // check all selectors and keep the one with the fewest total bits.
        total_bits.* = 64; // more than any valid encoding
        for (0..4) |s| {
            const d = enc.getDistr(@intCast(s));
            if (d.isDirect()) {
                if (d.direct() == value) {
                    selector.* = @intCast(s);
                    total_bits.* = 2;
                    return true; // Done, direct is always the best possible.
                }
                continue;
            }
            const extra_bits = d.extraBits();
            const off = d.offset();
            if (value < off or value >= off + (@as(u64, 1) << @intCast(extra_bits))) continue;

            // Better than prior encoding, remember it:
            if (2 + extra_bits < total_bits.*) {
                selector.* = @intCast(s);
                total_bits.* = 2 + extra_bits;
            }
        }

        return total_bits.* != 64;
    }
};

// ---------------------------------------------------------------------------
// U64Coder
// ---------------------------------------------------------------------------

pub const U64Coder = struct {
    pub fn read(reader: *BitReader) u64 {
        const selector = reader.readFixedBits(2);
        if (selector == 0) {
            return 0;
        }
        if (selector == 1) {
            return 1 + reader.readFixedBits(4);
        }
        if (selector == 2) {
            return 17 + reader.readFixedBits(8);
        }

        // selector 3, varint, groups have first 12, then 8, and last 4 bits.
        var result: u64 = reader.readFixedBits(12);

        var shift: u6 = 12;
        while (reader.readFixedBits(1) != 0) {
            if (shift == 60) {
                result |= reader.readFixedBits(4) << shift;
                break;
            }
            result |= reader.readFixedBits(8) << shift;
            shift += 8;
        }

        return result;
    }

    pub fn canEncode(value: u64) usize {
        if (value == 0) {
            return 2; // 2 selector bits
        } else if (value <= 16) {
            return 2 + 4; // 2 selector bits + 4 payload bits
        } else if (value <= 272) {
            return 2 + 8; // 2 selector bits + 8 payload bits
        } else {
            var bits_count: usize = 2 + 12; // 2 selector bits + 12 payload bits
            var v = value >> 12;
            var shift: usize = 12;
            while (v > 0 and shift < 60) {
                bits_count += 1 + 8; // 1 continuation bit + 8 payload bits
                v >>= 8;
                shift += 8;
            }
            if (v > 0) {
                bits_count += 1 + 4; // 1 continuation bit + 4 payload bits
            } else {
                bits_count += 1; // 1 stop bit
            }
            return bits_count;
        }
    }

    pub fn write(value_in: u64, writer: *BitWriter) void {
        var value = value_in;
        if (value == 0) {
            writer.write(2, 0);
        } else if (value <= 16) {
            writer.write(2, 1);
            writer.write(4, value - 1);
        } else if (value <= 272) {
            writer.write(2, 2);
            writer.write(8, value - 17);
        } else {
            // varint, first a 12-bit group, after that per 8-bit group.
            writer.write(2, 3);
            writer.write(12, value & 4095);
            value >>= 12;
            var shift: usize = 12;
            while (value > 0 and shift < 60) {
                // Indicate varint not done
                writer.write(1, 1);
                writer.write(8, value & 255);
                value >>= 8;
                shift += 8;
            }
            if (value > 0) {
                // This only could happen if shift == N - 4.
                writer.write(1, 1);
                writer.write(4, value & 15);
                // Implicitly closed sequence, no extra stop bit is required.
            } else {
                // Indicate end of varint
                writer.write(1, 0);
            }
        }
    }
};

// ---------------------------------------------------------------------------
// Variable-length integers
// ---------------------------------------------------------------------------

/// Encodes a number in the range [0..255] with 1 - 11 bits
/// (libjxl: StoreVarLenUint8).
pub fn storeVarLenUint8(n: usize, writer: anytype) void {
    std.debug.assert(n <= 255);
    if (n == 0) {
        writer.write(1, 0);
    } else {
        writer.write(1, 1);
        const nbits = bits.floorLog2Nonzero(n);
        writer.write(3, nbits);
        writer.write(nbits, n - (@as(usize, 1) << @intCast(nbits)));
    }
}

/// Encodes a number in the range [0..65535] with 1 - 21 bits
/// (libjxl: StoreVarLenUint16).
pub fn storeVarLenUint16(n: usize, writer: anytype) void {
    std.debug.assert(n <= 65535);
    if (n == 0) {
        writer.write(1, 0);
    } else {
        writer.write(1, 1);
        const nbits = bits.floorLog2Nonzero(n);
        writer.write(4, nbits);
        writer.write(nbits, n - (@as(usize, 1) << @intCast(nbits)));
    }
}

/// Decodes a number in the range [0..255], by reading 1 - 11 bits
/// (libjxl: DecodeVarLenUint8).
pub fn decodeVarLenUint8(input: *BitReader) u32 {
    if (input.readFixedBits(1) != 0) {
        const nbits: u32 = @intCast(input.readFixedBits(3));
        if (nbits == 0) {
            return 1;
        } else {
            return @as(u32, @intCast(input.readBits(nbits))) + (@as(u32, 1) << @intCast(nbits));
        }
    }
    return 0;
}

/// Decodes a number in the range [0..65535], by reading 1 - 21 bits
/// (libjxl: DecodeVarLenUint16).
pub fn decodeVarLenUint16(input: *BitReader) u32 {
    if (input.readFixedBits(1) != 0) {
        const nbits: u32 = @intCast(input.readFixedBits(4));
        if (nbits == 0) {
            return 1;
        } else {
            return @as(u32, @intCast(input.readBits(nbits))) + (@as(u32, 1) << @intCast(nbits));
        }
    }
    return 0;
}

// ---------------------------------------------------------------------------
// LZ77Params (dec_ans.h) — the only "bundle" used by the ANS stack.
// ---------------------------------------------------------------------------

pub const LZ77Params = struct {
    enabled: bool = false,

    /// Symbols above min_symbol use a special hybrid uint encoding and
    /// represent a length, to be added to min_length.
    min_symbol: u32 = 224,
    min_length: u32 = 3,

    /// Not serialized.
    length_uint_config: @import("../entropy/ans_common.zig").HybridUintConfig = .{ .split_exponent = 0, .split_token = 1, .msb_in_token = 0, .lsb_in_token = 0 },

    /// Serialized distance context (nonserialized_distance_context).
    nonserialized_distance_context: u32 = 0,

    /// Port of LZ77Params::VisitFields reading side (Bundle::Read).
    pub fn read(self: *LZ77Params, reader: *BitReader) !void {
        // Defaults (Bundle::Init):
        self.* = .{};
        self.enabled = (reader.readFixedBits(1) != 0);
        if (!self.enabled) return;
        self.min_symbol = U32Coder.read(U32Enc.init(val(224), val(512), val(4096), bitsOffset(15, 8)), reader);
        self.min_length = U32Coder.read(U32Enc.init(val(3), val(4), bitsOffset(2, 5), bitsOffset(8, 9)), reader);
    }

    /// Port of LZ77Params::VisitFields writing side (Bundle::Write).
    pub fn write(self: *const LZ77Params, writer: anytype) void {
        writer.write(1, @intFromBool(self.enabled));
        if (!self.enabled) return;
        U32Coder.write(U32Enc.init(val(224), val(512), val(4096), bitsOffset(15, 8)), self.min_symbol, writer) catch unreachable;
        U32Coder.write(U32Enc.init(val(3), val(4), bitsOffset(2, 5), bitsOffset(8, 9)), self.min_length, writer) catch unreachable;
    }

    /// Bit size of the serialized form (Bundle::CanEncode).
    pub fn bitSize(self: *const LZ77Params) usize {
        var sw = SizeWriter{};
        self.write(&sw);
        return sw.size;
    }
};

/// libjxl `SizeWriter`: counts bits instead of writing them.
pub const SizeWriter = struct {
    size: usize = 0,

    pub fn write(self: *SizeWriter, num: usize, bits_v: u64) void {
        _ = bits_v;
        self.size += num;
    }
};

// ---------------------------------------------------------------------------

test "u32 coder roundtrip" {
    const alloc = std.testing.allocator;
    const enc = U32Enc.init(val(0), bitsOffset(1, 1), bitsOffset(2, 3), bitsOffset(8, 8));
    var w = BitWriter.init(alloc);
    defer w.deinit();
    const values = [_]u32{ 0, 1, 2, 3, 6, 8, 100, 263 };
    for (values) |v| {
        try U32Coder.write(enc, v, &w);
    }
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    for (values) |v| {
        try std.testing.expectEqual(v, U32Coder.read(enc, &br));
    }
    try br.close();
}

test "u32 coder lz77 distributions" {
    const alloc = std.testing.allocator;
    const enc_sym = U32Enc.init(val(224), val(512), val(4096), bitsOffset(15, 8));
    const enc_len = U32Enc.init(val(3), val(4), bitsOffset(2, 5), bitsOffset(8, 9));
    var w = BitWriter.init(alloc);
    defer w.deinit();
    const sym_values = [_]u32{ 224, 512, 4096, 8, 30000 };
    const len_values = [_]u32{ 3, 4, 5, 8, 9, 264 };
    for (sym_values) |v| try U32Coder.write(enc_sym, v, &w);
    for (len_values) |v| try U32Coder.write(enc_len, v, &w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    for (sym_values) |v| try std.testing.expectEqual(v, U32Coder.read(enc_sym, &br));
    for (len_values) |v| try std.testing.expectEqual(v, U32Coder.read(enc_len, &br));
    try br.close();
}

test "u64 coder roundtrip" {
    const alloc = std.testing.allocator;
    var w = BitWriter.init(alloc);
    defer w.deinit();
    const values = [_]u64{ 0, 1, 16, 17, 272, 273, 4095, 4096, 1 << 20, (1 << 60) + 12345 };
    for (values) |v| U64Coder.write(v, &w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    for (values) |v| try std.testing.expectEqual(v, U64Coder.read(&br));
    try br.close();
}

test "varlen uint8/16 roundtrip" {
    const alloc = std.testing.allocator;
    var w = BitWriter.init(alloc);
    defer w.deinit();
    const values8 = [_]u32{ 0, 1, 2, 3, 4, 8, 127, 128, 255 };
    const values16 = [_]u32{ 0, 1, 17, 65535 };
    for (values8) |v| storeVarLenUint8(v, &w);
    for (values16) |v| storeVarLenUint16(v, &w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    for (values8) |v| try std.testing.expectEqual(v, decodeVarLenUint8(&br));
    for (values16) |v| try std.testing.expectEqual(v, decodeVarLenUint16(&br));
    try br.close();
}

test "lz77 params roundtrip" {
    const alloc = std.testing.allocator;
    var params: LZ77Params = .{ .enabled = true, .min_symbol = 512, .min_length = 5 };
    params.length_uint_config = @import("../entropy/ans_common.zig").HybridUintConfig.init(2, 0, 1);

    var w = BitWriter.init(alloc);
    defer w.deinit();
    params.write(&w);
    w.zeroPadToByte();

    var decoded: LZ77Params = .{};
    var br = BitReader.init(w.getSpan());
    try decoded.read(&br);
    try br.close();

    try std.testing.expectEqual(params.enabled, decoded.enabled);
    try std.testing.expectEqual(params.min_symbol, decoded.min_symbol);
    try std.testing.expectEqual(params.min_length, decoded.min_length);

    // Disabled case.
    var disabled: LZ77Params = .{};
    var w2 = BitWriter.init(alloc);
    defer w2.deinit();
    disabled.write(&w2);
    w2.zeroPadToByte();
    var decoded2: LZ77Params = .{ .enabled = true };
    var br2 = BitReader.init(w2.getSpan());
    try decoded2.read(&br2);
    try br2.close();
    try std.testing.expectEqual(false, decoded2.enabled);
    try std.testing.expectEqual(@as(u32, 224), decoded2.min_symbol);
    try std.testing.expectEqual(@as(u32, 3), decoded2.min_length);
}
