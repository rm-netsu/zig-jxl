//! Порт гибридного целочисленного кодирования из `lib/jxl/dec_ans.h`
//! (`HybridUintConfig`, `ReadHybridUintConfig`) и `dec_ans.cc`
//! (`DecodeUintConfig`).
//!
//! Схема: числа < split_token получают выделенный токен; для остальных
//! токен кодирует (число бит) + msb_in_token старших бит мантиссы +
//! lsb_in_token младших бит, а остаток идёт в поток как «nibbles».

const std = @import("std");
const bits = @import("../base/bits.zig");
const BitReader = @import("../base/bit_io.zig").BitReader;

pub const HybridUintConfig = struct {
    split_exponent: u32 = 4,
    split_token: u32 = 16,
    msb_in_token: u32 = 2,
    lsb_in_token: u32 = 0,

    /// Порт конструктора `HybridUintConfig(split_exponent, msb_in_token, lsb_in_token)`.
    pub fn init(split_exponent: u32, msb_in_token: u32, lsb_in_token: u32) HybridUintConfig {
        std.debug.assert(split_exponent >= msb_in_token + lsb_in_token);
        return .{
            .split_exponent = split_exponent,
            .split_token = @as(u32, 1) << @intCast(split_exponent),
            .msb_in_token = msb_in_token,
            .lsb_in_token = lsb_in_token,
        };
    }

    pub const Token = struct {
        token: u32 = 0,
        nbits: u32 = 0,
        bits: u32 = 0,
    };

    /// Порт `HybridUintConfig::Encode`.
    pub fn encode(self: HybridUintConfig, value: u32) Token {
        if (value < self.split_token) {
            return .{ .token = value, .nbits = 0, .bits = 0 };
        }
        const n: u32 = @intCast(bits.floorLog2Nonzero(value));
        const m: u32 = value -% (@as(u32, 1) << @intCast(n));
        const pow_mslsb: u32 = @as(u32, 1) << @intCast(self.msb_in_token + self.lsb_in_token);
        const pow_lsb: u32 = @as(u32, 1) << @intCast(self.lsb_in_token);
        const token: u32 = self.split_token +%
            ((n -% self.split_exponent) *% pow_mslsb) +%
            (((m >> @intCast(n - self.msb_in_token)) *% pow_lsb)) +%
            (m & (pow_lsb -% 1));
        const nbits: u32 = n - self.msb_in_token - self.lsb_in_token;
        const b: u32 = @truncate((@as(u64, value) >> @intCast(self.lsb_in_token)) &
            ((@as(u64, 1) << @intCast(nbits)) - 1));
        return .{ .token = token, .nbits = nbits, .bits = b };
    }

    pub inline fn lsbMask(self: HybridUintConfig) u32 {
        return (@as(u32, 1) << @intCast(self.lsb_in_token)) - 1;
    }

    /// Порт `ANSSymbolReader::ReadHybridUintConfig(config, token, br)`:
    /// по токену и «nibbles», прочитанным из битстрима, восстанавливает число.
    pub fn readValue(self: HybridUintConfig, token: u32, br: *BitReader) u32 {
        const split_token: u32 = self.split_token;
        const msb_in_token: u32 = self.msb_in_token;
        const lsb_in_token: u32 = self.lsb_in_token;
        const split_exponent: u32 = self.split_exponent;
        // Быстрый путь гибридного декодирования.
        if (token < split_token) return token;
        var nbits: u32 = split_exponent - (msb_in_token + lsb_in_token) +
            ((token - split_token) >> @intCast(msb_in_token + lsb_in_token));
        // ReadBits поддерживает максимум 32 бита; ограничиваем nbits,
        // как это делает libjxl (битстрим в этом месте некорректен).
        nbits &= 31;
        const low: u32 = token & ((@as(u32, 1) << @intCast(lsb_in_token)) - 1);
        const tok: u32 = token >> @intCast(lsb_in_token);
        const b: u64 = br.peekBits(nbits);
        br.consume(nbits);
        const ret: u64 = ((((@as(u64, 1) << @intCast(msb_in_token)) |
            (tok & ((@as(u64, 1) << @intCast(msb_in_token)) - 1))) << @intCast(nbits)) | b) <<
            @intCast(lsb_in_token) | low;
        return @truncate(ret);
    }
};

/// Порт `DecodeUintConfig` (dec_ans.cc): читает конфигурацию из битстрима.
pub fn decodeUintConfig(log_alpha_size: usize, br: *BitReader) !HybridUintConfig {
    br.refill();
    const split_exponent: u32 = @intCast(br.readBits(bits.ceilLog2Nonzero(log_alpha_size + 1)));
    var msb_in_token: u32 = 0;
    var lsb_in_token: u32 = 0;
    if (split_exponent != log_alpha_size) {
        // иначе msb/lsb не имеют значения.
        var nbits = bits.ceilLog2Nonzero(split_exponent + 1);
        msb_in_token = @intCast(br.readBits(nbits));
        if (msb_in_token > split_exponent) {
            return error.InvalidHybridUintConfig;
        }
        nbits = bits.ceilLog2Nonzero(split_exponent - msb_in_token + 1);
        lsb_in_token = @intCast(br.readBits(nbits));
    }
    if (lsb_in_token + msb_in_token > split_exponent) {
        return error.InvalidHybridUintConfig;
    }
    return HybridUintConfig.init(split_exponent, msb_in_token, lsb_in_token);
}

// ---------------------------------------------------------------------------
// Тесты
// ---------------------------------------------------------------------------

const testing = std.testing;

test "HybridUintConfig: golden-векторы из комментария dec_ans.h" {
    // Example with split_exponent = 4, msb_in_token = 2, lsb_in_token = 0.
    const cfg = HybridUintConfig.init(4, 2, 0);

    // N = 0 - 15: (token=N, nbits=0, bits='')
    inline for (0..16) |n| {
        const t = cfg.encode(n);
        try testing.expectEqual(@as(u32, n), t.token);
        try testing.expectEqual(@as(u32, 0), t.nbits);
    }
    // N = 16 (10000): (token=16, nbits=2, bits='00')
    {
        const t = cfg.encode(16);
        try testing.expectEqual(@as(u32, 16), t.token);
        try testing.expectEqual(@as(u32, 2), t.nbits);
        try testing.expectEqual(@as(u32, 0), t.bits);
    }
    // N = 17 (10001): (token=16, nbits=2, bits='01')
    {
        const t = cfg.encode(17);
        try testing.expectEqual(@as(u32, 16), t.token);
        try testing.expectEqual(@as(u32, 2), t.nbits);
        try testing.expectEqual(@as(u32, 1), t.bits);
    }
    // N = 20 (10100): (token=17, nbits=2, bits='00')
    {
        const t = cfg.encode(20);
        try testing.expectEqual(@as(u32, 17), t.token);
        try testing.expectEqual(@as(u32, 2), t.nbits);
        try testing.expectEqual(@as(u32, 0), t.bits);
    }
    // N = 24 (11000): (token=18, nbits=2, bits='00')
    {
        const t = cfg.encode(24);
        try testing.expectEqual(@as(u32, 18), t.token);
        try testing.expectEqual(@as(u32, 2), t.nbits);
        try testing.expectEqual(@as(u32, 0), t.bits);
    }
    // N = 28 (11100): (token=19, nbits=2, bits='00')
    {
        const t = cfg.encode(28);
        try testing.expectEqual(@as(u32, 19), t.token);
        try testing.expectEqual(@as(u32, 2), t.nbits);
        try testing.expectEqual(@as(u32, 0), t.bits);
    }
    // N = 32 (100000): (token=20, nbits=3, bits='000')
    {
        const t = cfg.encode(32);
        try testing.expectEqual(@as(u32, 20), t.token);
        try testing.expectEqual(@as(u32, 3), t.nbits);
        try testing.expectEqual(@as(u32, 0), t.bits);
    }
    // N = 65535: (token=63, nbits=13, bits='1111111111111')
    {
        const t = cfg.encode(65535);
        try testing.expectEqual(@as(u32, 63), t.token);
        try testing.expectEqual(@as(u32, 13), t.nbits);
        try testing.expectEqual(@as(u32, 0x1FFF), t.bits);
    }
}

test "HybridUintConfig: round trip encode/readValue" {
    const configs = [_]HybridUintConfig{
        HybridUintConfig.init(0, 0, 0),
        HybridUintConfig.init(4, 2, 0),
        HybridUintConfig.init(4, 1, 2),
        HybridUintConfig.init(7, 3, 1),
        HybridUintConfig.init(15, 2, 2),
    };
    for (configs) |cfg| {
        // Все значения до 1<<16 плюс крупные границы.
        var sweep: [65545]u64 = undefined;
        var count: usize = 0;
        var value: u64 = 0;
        while (value <= (1 << 16)) : (value += 1) {
            sweep[count] = value;
            count += 1;
        }
        for ([_]u64{ (1 << 17) + 123, (1 << 20) + 7, (1 << 24) - 1, (1 << 30) + 5 }) |v| {
            sweep[count] = v;
            count += 1;
        }
        for (sweep[0..count]) |v| {
            const t = cfg.encode(@intCast(v));
            // Кладём биты в буфер вручную и читаем их обратно.
            var buf: [16]u8 = .{0} ** 16;
            var bitpos: usize = 0;
            for (0..t.nbits) |i| {
                const bit = (t.bits >> @intCast(i)) & 1;
                buf[bitpos >> 3] |= @as(u8, @intCast(bit)) << @intCast(bitpos & 7);
                bitpos += 1;
            }
            var br = BitReader.init(&buf);
            const decoded = cfg.readValue(t.token, &br);
            try testing.expectEqual(@as(u32, @intCast(v)), decoded);
            try testing.expectEqual(bitpos, br.totalBitsConsumed());
        }
    }
}

test "decodeUintConfig: round trip через битстрим" {
    const gpa = testing.allocator;
    const BitWriter = @import("../base/bit_io.zig").BitWriter;
    // Замечание: конфигурация с split_exponent == log_alpha_size кодирует
    // msb/lsb нулями (они не читаются), как в libjxl.
    const configs = [_]HybridUintConfig{
        HybridUintConfig.init(4, 2, 0),
        HybridUintConfig.init(0, 0, 0),
        HybridUintConfig.init(14, 5, 5),
        HybridUintConfig.init(9, 9, 0),
        HybridUintConfig.init(15, 0, 0),
    };
    // Пишем для log_alpha_size = 15: split_exponent занимает ceilLog2(16) = 4 бита.
    var w = BitWriter.init(gpa);
    defer w.deinit();
    for (configs) |cfg| {
        w.write(4, cfg.split_exponent);
        if (cfg.split_exponent != 15) {
            const msb_bits = bits.ceilLog2Nonzero(cfg.split_exponent + 1);
            w.write(msb_bits, cfg.msb_in_token);
            const lsb_bits = bits.ceilLog2Nonzero(cfg.split_exponent - cfg.msb_in_token + 1);
            w.write(lsb_bits, cfg.lsb_in_token);
        }
    }
    w.zeroPadToByte();
    w.zeroPadToByte(); const bytes = try w.takeBytes();
    defer gpa.free(bytes);
    var br = BitReader.init(bytes);
    for (configs) |cfg| {
        const decoded = try decodeUintConfig(15, &br);
        try testing.expectEqual(cfg.split_exponent, decoded.split_exponent);
        try testing.expectEqual(cfg.msb_in_token, decoded.msb_in_token);
        try testing.expectEqual(cfg.lsb_in_token, decoded.lsb_in_token);
    }
}
