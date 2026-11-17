//! Интерфейс чтения символов (аналог `ANSSymbolReader` из lib/jxl/dec_ans.h)
//! и две реализации:
//!  * `MockSymbolReader` — заглушка для юнит-тестов DecodeTree (последовательность
//!    значений на контекст);
//!  * `PrefixSymbolReader` — Brotli-совместимые prefix-коды (без ANS/LZ77),
//!    собранные в prefix_code.zig.
//!
//! Полный ANS-декодер не входит в объём порта: `ma.decodeTree` принимает
//! любой тип (comptime anytype), предоставляющий:
//!   readHybridUint(ctx: usize, br: *BitReader) u32   — контекст уже
//!       спроецирован через context_map внутри ридера;
//!   checkFinalState() bool                           — аналог CheckANSFinalState;
//!   isPropertyContextDegenerate() bool               — аналог проверки
//!       degenerate_symbols[context_map[kPropertyContext]] > 0 ("Infinite tree").

const std = @import("std");
const hybrid_uint = @import("hybrid_uint.zig");
const prefix_code = @import("prefix_code.zig");
const BitReader = @import("../base/bit_io.zig").BitReader;

/// Один префиксный код + его гибридная uint-конфигурация
/// (аналог пары (HuffmanDecodingData, HybridUintConfig) из ANSCode).
pub const PrefixCode = struct {
    huffman: prefix_code.HuffmanDecodingData = .{},
    uint_config: hybrid_uint.HybridUintConfig = .{},

    pub fn deinit(self: *PrefixCode, gpa: std.mem.Allocator) void {
        self.huffman.deinit(gpa);
    }
};

/// Ридер на префиксных кодах. Аналог ANSSymbolReader при use_prefix_code=1
/// и без LZ77 (LZ77 — часть ANS-слоя, не портируется).
pub const PrefixSymbolReader = struct {
    codes: []PrefixCode,
    context_map: []const u8,

    /// ctx — "кластеризованный" индекс (уже после context_map).
    pub fn readHybridUintClustered(self: *const PrefixSymbolReader, ctx: usize, br: *BitReader) u32 {
        // Аналог br->Refill() из ReadHybridUintClusteredInlined: покрывает
        // и чтение символа (<= 15 бит), и nibbles (<= 29 бит).
        br.refill();
        const code = &self.codes[ctx];
        const token = code.huffman.readSymbol(br);
        return code.uint_config.readValue(token, br);
    }

    /// Аналог ReadHybridUint(ctx, br, context_map).
    pub fn readHybridUint(self: *const PrefixSymbolReader, ctx: usize, br: *BitReader) u32 {
        return self.readHybridUintClustered(self.context_map[ctx], br);
    }

    pub fn checkFinalState(self: *const PrefixSymbolReader) bool {
        _ = self;
        return true;
    }

    /// Аналог tree_code.degenerate_symbols[context_map[kPropertyContext]] > 0.
    /// Код "вырожден", если в нём ровно один символ; если при этом символ
    /// ненулевой, дерево не сможет завершиться ("Infinite tree").
    pub fn isPropertyContextDegenerate(self: *const PrefixSymbolReader) bool {
        // kPropertyContext == 1 (ma_common.h).
        const h = &self.codes[self.context_map[1]].huffman;
        if (h.table.items.len == 0) return false;
        const first = h.table.items[0];
        for (h.table.items) |e| {
            if (e.bits != 0 or e.value != first.value) return false;
        }
        return first.value != 0;
    }
};

/// Заглушка для юнит-тестов: выдаёт заранее заданные значения по контекстам.
pub const MockSymbolReader = struct {
    values: []const []const u32, // последовательность значений на контекст
    pos: []usize,
    final_state_ok: bool = true,

    pub fn init(gpa: std.mem.Allocator, values: []const []const u32) !MockSymbolReader {
        const pos = try gpa.alloc(usize, values.len);
        @memset(pos, 0);
        return .{ .values = values, .pos = pos };
    }

    pub fn deinit(self: *MockSymbolReader, gpa: std.mem.Allocator) void {
        gpa.free(self.pos);
    }

    pub fn readHybridUint(self: *MockSymbolReader, ctx: usize, br: *BitReader) u32 {
        _ = br;
        const seq = self.values[ctx];
        const v = seq[self.pos[ctx]];
        self.pos[ctx] += 1;
        return v;
    }

    pub fn checkFinalState(self: *const MockSymbolReader) bool {
        return self.final_state_ok;
    }

    pub fn isPropertyContextDegenerate(self: *const MockSymbolReader) bool {
        _ = self;
        return false;
    }
};

test "PrefixSymbolReader: базовое чтение через простой код" {
    const gpa = std.testing.allocator;
    const BitWriter = @import("../base/bit_io.zig").BitWriter;

    // Код: 2 символа {0, 1}, гибрид-конфиг (0,0,0) (значение = токен).
    var w = BitWriter.init(gpa);
    defer w.deinit();
    w.write(2, 1); // simple
    w.write(2, 1); // nsym=2
    w.write(1, 0); // alphabet_size=2 -> max_bits=1
    w.write(1, 1);
    // Символы: 0 (значение 0), 1 (значение 1), 1, 0
    w.write(1, 0);
    w.write(1, 1);
    w.write(1, 1);
    w.write(1, 0);
    w.zeroPadToByte(); const bytes = try w.takeBytes();
    defer gpa.free(bytes);

    var codes = [_]PrefixCode{.{ .uint_config = hybrid_uint.HybridUintConfig.init(0, 0, 0) }};
    var br = BitReader.init(bytes);
    try std.testing.expect(codes[0].huffman.readFromBitStream(gpa, 2, &br));
    const context_map = [_]u8{0} ** 8;
    const reader = PrefixSymbolReader{ .codes = &codes, .context_map = &context_map };
    try std.testing.expectEqual(@as(u32, 0), reader.readHybridUint(3, &br));
    try std.testing.expectEqual(@as(u32, 1), reader.readHybridUint(5, &br));
    try std.testing.expectEqual(@as(u32, 1), reader.readHybridUint(0, &br));
    try std.testing.expectEqual(@as(u32, 0), reader.readHybridUint(1, &br));
    try std.testing.expect(reader.checkFinalState());
    codes[0].deinit(gpa);
}

test "MockSymbolReader: последовательности" {
    const gpa = std.testing.allocator;
    const seqs = [_][]const u32{ &.{ 1, 0, 2 }, &.{5} };
    var reader = try MockSymbolReader.init(gpa, &seqs);
    defer reader.deinit(gpa);
    try std.testing.expectEqual(@as(u32, 1), reader.readHybridUint(0, undefined));
    try std.testing.expectEqual(@as(u32, 0), reader.readHybridUint(0, undefined));
    try std.testing.expectEqual(@as(u32, 5), reader.readHybridUint(1, undefined));
    try std.testing.expectEqual(@as(u32, 2), reader.readHybridUint(0, undefined));
}
