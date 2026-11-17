//! Порт `lib/jxl/base/random.h` — Rng (xorshift128+) с геометрическим
//! распределением. Нужен для `CollectPixelSamples` (enc_ma.cc).

const std = @import("std");

pub const Rng = struct {
    s: [2]u64,

    pub fn init(seed: u64) Rng {
        return .{ .s = .{
            0x94D049BB133111EB,
            0xBF58476D1CE4E5B9 +% seed,
        } };
    }

    /// Xorshift128+.
    pub fn next(self: *Rng) u64 {
        var s1 = self.s[0];
        const s0 = self.s[1];
        const bits = s1 +% s0; // b, c
        self.s[0] = s0;
        s1 ^= s1 << 23;
        s1 ^= s0 ^ (s1 >> 18) ^ (s0 >> 5);
        self.s[1] = s1;
        return bits;
    }

    /// Равномерное u64 в [begin, end).
    pub fn uniformU(self: *Rng, begin: u64, end: u64) u64 {
        std.debug.assert(end > begin);
        return self.next() % (end - begin) + begin;
    }

    /// Равномерный float в [begin, end); только 23 бита случайности.
    pub fn uniformF(self: *Rng, begin: f32, end: f32) f32 {
        const u: u32 = @as(u32, @intCast(self.next() >> (64 - 23))) | 0x3F800000;
        const f: f32 = @bitCast(u);
        return (end - begin) * (f - 1.0) + begin;
    }

    /// Состояние геометрического распределения (inv_log_1mp).
    pub const GeometricDistribution = f32;

    pub fn makeGeometric(p: f32) GeometricDistribution {
        return 1.0 / @log(1.0 - p);
    }

    pub fn geometric(self: *Rng, dist: GeometricDistribution) u32 {
        const f = self.uniformF(0, 1);
        const l = @log(1.0 - f) * dist;
        // C++: static_cast<uint32_t>(log) — усечение к нулю.
        if (l <= 0) return 0;
        if (l >= 4294967296.0) return std.math.maxInt(u32);
        return @intFromFloat(l);
    }
};

test "Rng: детерминированность и диапазоны" {
    var a = Rng.init(42);
    var b = Rng.init(42);
    for (0..100) |_| {
        try std.testing.expectEqual(a.next(), b.next());
    }
    var r = Rng.init(7);
    for (0..1000) |_| {
        const v = r.uniformU(0, 10);
        try std.testing.expect(v < 10);
    }
    const dist = Rng.makeGeometric(0.1);
    var r2 = Rng.init(9);
    for (0..1000) |_| {
        const g = r2.geometric(dist);
        try std.testing.expect(g < (1 << 30));
    }
}
