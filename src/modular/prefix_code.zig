//! Порт `lib/jxl/huffman_table.cc` (`BuildHuffmanTable`) и
//! `lib/jxl/dec_huffman.cc` (`HuffmanDecodingData`, чтение Brotli-совместимых
//! prefix-кодов: простые коды NSYM=1..4/5 и сложные коды с RLE длин).
//!
//! Полный ANS-декодер (dec_ans.cc) не портируется по условиям задачи:
//! `SymbolReader`-интерфейс (см. symbol_reader.zig) принимает префиксные
//! коды, собранные этим модулем. Здесь же — вспомогательный канонический
//! кодировщик для round-trip-тестов.

const std = @import("std");
const bits = @import("../base/bits.zig");
const BitReader = @import("../base/bit_io.zig").BitReader;

pub const kHuffmanTableBits = 8;
pub const kPrefixMaxBits = 15; // PREFIX_MAX_BITS

pub const HuffmanCode = struct {
    bits: u8 = 0, // число бит для этого символа (или смещение таблицы)
    value: u16 = 0, // значение символа (или смещение таблицы)
};

/// Возвращает reverse(reverse(key, len) + 1, len).
fn getNextKey(key: u32, len: u32) u32 {
    var step: u32 = @as(u32, 1) << @intCast(len - 1);
    while ((key & step) != 0) {
        step >>= 1;
    }
    // step может дойти до 0 (кодовое пространство исчерпано) — в C++ это
    // заворачивание int, в Zig нужно явное %.
    return (key & (step -% 1)) +% step;
}

/// Кладёт code в table[start], table[start+step], ... — до end (не вкл.).
/// Аналог ReplicateValue из huffman_table.cc: индексы start+end-step ... start+0.
fn replicateValue(table: []HuffmanCode, start: usize, step: usize, end: usize, code: HuffmanCode) void {
    var e = end;
    while (e > 0) {
        e -= step;
        table[start + e] = code;
    }
}

/// Ширина таблицы следующего уровня.
fn nextTableBitSize(count: *const [16]u16, len_in: usize, root_bits: usize) usize {
    var len = len_in;
    var left: usize = @as(usize, 1) << @intCast(len - root_bits);
    while (len < kPrefixMaxBits) {
        if (left <= count[len]) break;
        left -= count[len];
        len += 1;
        left <<= 1;
    }
    return len - root_bits;
}

/// Порт `BuildHuffmanTable` (huffman_table.cc).
/// `counts` — гистограмма длин кодов (индексы 1..15); МОДИФИЦИРУЕТСЯ на месте,
/// как в C++. Возвращает итоговый размер таблицы или 0 при ошибке.
pub fn buildHuffmanTable(
    gpa: std.mem.Allocator,
    root_table: []HuffmanCode,
    root_bits: usize,
    code_lengths: []const u8,
    counts: *[16]u16,
) usize {
    var count = counts.*;
    var max_length: usize = 1;

    if (code_lengths.len > (@as(usize, 1) << kPrefixMaxBits)) return 0;

    // Сортированные по длине символы.
    const sorted = gpa.alloc(u16, code_lengths.len) catch return 0;
    defer gpa.free(sorted);

    // Смещения в сортированной таблице по длинам.
    var offset: [kPrefixMaxBits + 1]u16 = undefined;
    {
        var sum: u16 = 0;
        for (1..kPrefixMaxBits + 1) |len| {
            offset[len] = sum;
            if (count[len] != 0) {
                sum +%= count[len];
                max_length = len;
            }
        }
    }

    // Сортировка символов по длине (внутри длины — по номеру символа).
    for (code_lengths, 0..) |l, symbol| {
        if (l != 0) {
            sorted[offset[l]] = @intCast(symbol);
            offset[l] += 1;
        }
    }

    var total_size: usize = @as(usize, 1) << @intCast(root_bits);

    // Особый случай: код с единственным значением (offset[15] — сумма count[1..14]).
    if (offset[kPrefixMaxBits] == 1) {
        const code = HuffmanCode{ .bits = 0, .value = sorted[0] };
        for (0..total_size) |key| root_table[key] = code;
        return total_size;
    }

    var table_off: usize = 0; // смещение "table" от root_table
    var table_bits: usize = root_bits;
    var table_size: usize = @as(usize, 1) << @intCast(table_bits);

    // Уменьшить корневую таблицу, если возможно.
    if (table_bits > max_length) {
        table_bits = max_length;
        table_size = @as(usize, 1) << @intCast(table_bits);
    }
    var key: u32 = 0;
    var symbol: usize = 0;
    var code_bits: usize = 1;
    var step: usize = 2;
    while (true) : (code_bits += 1) {
        while (count[code_bits] != 0) : (count[code_bits] -= 1) {
            const code = HuffmanCode{ .bits = @intCast(code_bits), .value = sorted[symbol] };
            symbol += 1;
            replicateValue(root_table, key, step, table_size, code);
            key = getNextKey(key, @intCast(code_bits));
        }
        step <<= 1;
        if (code_bits >= table_bits) break;
    }

    // Если root_bits != table_bits — заполнена лишь часть таблицы; повторить.
    while (total_size != table_size) {
        @memcpy(root_table[table_size .. table_size * 2], root_table[0..table_size]);
        table_size <<= 1;
    }

    // Таблицы второго уровня + указатели на них в корневой таблице.
    const mask: u32 = @intCast(total_size - 1);
    var low: i64 = -1;
    var len: usize = root_bits + 1;
    step = 2;
    while (len <= max_length) : ({
        len += 1;
        step <<= 1;
    }) {
        while (count[len] != 0) : (count[len] -= 1) {
            if (@as(i64, key & mask) != low) {
                table_off += table_size;
                table_bits = nextTableBitSize(&count, len, root_bits);
                table_size = @as(usize, 1) << @intCast(table_bits);
                total_size += table_size;
                low = @intCast(key & mask);
                root_table[@intCast(low)].bits = @intCast(table_bits + root_bits);
                root_table[@intCast(low)].value = @intCast(table_off - @as(usize, @intCast(low)));
            }
            const code = HuffmanCode{
                .bits = @intCast(len - root_bits),
                .value = sorted[symbol],
            };
            symbol += 1;
            replicateValue(root_table, table_off + (key >> @intCast(root_bits)), step, table_size, code);
            key = getNextKey(key, @intCast(len));
        }
    }

    return total_size;
}

const kCodeLengthCodes = 18;
pub const kCodeLengthCodeOrder = [kCodeLengthCodes]u8{
    1, 2, 3, 4, 0, 5, 17, 6, 16, 7, 8, 9, 10, 11, 12, 13, 14, 15,
};
const kDefaultCodeLength = 8;
const kCodeLengthRepeatCode = 16;

/// Порт `ReadHuffmanCodeLengths` (dec_huffman.cc).
fn readHuffmanCodeLengths(
    gpa: std.mem.Allocator,
    code_length_code_lengths: *const [kCodeLengthCodes]u8,
    num_symbols: usize,
    code_lengths: []u8,
    br: *BitReader,
) bool {
    var symbol: usize = 0;
    var prev_code_len: u8 = kDefaultCodeLength;
    var repeat: i32 = 0;
    var repeat_code_len: u8 = 0;
    var space: i32 = 32768;
    var table: [32]HuffmanCode = .{HuffmanCode{}} ** 32;

    var counts: [16]u16 = .{0} ** 16;
    for (0..kCodeLengthCodes) |i| {
        counts[code_length_code_lengths[i]] += 1;
    }
    if (buildHuffmanTable(gpa, &table, 5, code_length_code_lengths, &counts) == 0) {
        return false;
    }

    while (symbol < num_symbols and space > 0) {
        br.refill();
        const p = table[br.peekFixedBits(5)];
        br.consume(p.bits);
        const code_len: u8 = @intCast(p.value);
        if (code_len < kCodeLengthRepeatCode) {
            repeat = 0;
            code_lengths[symbol] = code_len;
            symbol += 1;
            if (code_len != 0) {
                prev_code_len = code_len;
                space -= @as(i32, 32768) >> @as(u5, @intCast(code_len));
            }
        } else {
            const extra_bits: usize = code_len - 14; // 16 -> 2, 17 -> 3
            const old_repeat = repeat;
            var new_len: u8 = 0;
            if (code_len == kCodeLengthRepeatCode) {
                new_len = prev_code_len;
            }
            if (repeat_code_len != new_len) {
                repeat = 0;
                repeat_code_len = new_len;
            }
            if (repeat > 0) {
                repeat -= 2;
                repeat <<= @intCast(extra_bits);
                // Защита от переполнения на некорректных потоках (в C++ тут UB).
                if (repeat > 1 << 25) return false;
            }
            repeat +%= @intCast(br.readBits(extra_bits) + 3);
            const repeat_delta = repeat - old_repeat;
            if (repeat_delta < 0 or symbol + @as(usize, @intCast(repeat_delta)) > num_symbols) {
                return false;
            }
            @memset(code_lengths[symbol .. symbol + @as(usize, @intCast(repeat_delta))], repeat_code_len);
            symbol += @intCast(repeat_delta);
            if (repeat_code_len != 0) {
                space -= repeat_delta << @as(u5, @intCast(15 - @as(i32, repeat_code_len)));
            }
        }
    }
    if (space != 0) {
        return false;
    }
    @memset(code_lengths[symbol..], 0);
    return true;
}

/// Порт `ReadSimpleCode` (dec_huffman.cc).
fn readSimpleCode(alphabet_size: usize, br: *BitReader, table: []HuffmanCode) bool {
    const max_bits: usize = if (alphabet_size > 1) bits.floorLog2Nonzero(alphabet_size - 1) + 1 else 0;

    var num_symbols: usize = @intCast(br.readFixedBits(2) + 1);

    var symbols: [4]u16 = .{ 0, 0, 0, 0 };
    for (0..num_symbols) |i| {
        const symbol: u16 = @intCast(br.readBits(max_bits));
        if (symbol >= alphabet_size) return false;
        symbols[i] = symbol;
    }

    for (0..num_symbols - 1) |i| {
        for (i + 1..num_symbols) |j| {
            if (symbols[i] == symbols[j]) return false;
        }
    }

    // Для 4 символов есть опция закодировать пятый.
    if (num_symbols == 4) num_symbols += @intCast(br.readFixedBits(1));

    const swapSymbols = struct {
        fn swap(s: *[4]u16, i: usize, j: usize) void {
            const t = s[j];
            s[j] = s[i];
            s[i] = t;
        }
    }.swap;

    var table_size: usize = 1;
    switch (num_symbols) {
        1 => table[0] = .{ .bits = 0, .value = symbols[0] },
        2 => {
            if (symbols[0] > symbols[1]) swapSymbols(&symbols, 0, 1);
            table[0] = .{ .bits = 1, .value = symbols[0] };
            table[1] = .{ .bits = 1, .value = symbols[1] };
            table_size = 2;
        },
        3 => {
            if (symbols[1] > symbols[2]) swapSymbols(&symbols, 1, 2);
            table[0] = .{ .bits = 1, .value = symbols[0] };
            table[2] = .{ .bits = 1, .value = symbols[0] };
            table[1] = .{ .bits = 2, .value = symbols[1] };
            table[3] = .{ .bits = 2, .value = symbols[2] };
            table_size = 4;
        },
        4 => {
            for (0..3) |i| {
                for (i + 1..4) |j| {
                    if (symbols[i] > symbols[j]) swapSymbols(&symbols, i, j);
                }
            }
            table[0] = .{ .bits = 2, .value = symbols[0] };
            table[2] = .{ .bits = 2, .value = symbols[1] };
            table[1] = .{ .bits = 2, .value = symbols[2] };
            table[3] = .{ .bits = 2, .value = symbols[3] };
            table_size = 4;
        },
        5 => {
            if (symbols[2] > symbols[3]) swapSymbols(&symbols, 2, 3);
            table[0] = .{ .bits = 1, .value = symbols[0] };
            table[1] = .{ .bits = 2, .value = symbols[1] };
            table[2] = .{ .bits = 1, .value = symbols[0] };
            table[3] = .{ .bits = 3, .value = symbols[2] };
            table[4] = .{ .bits = 1, .value = symbols[0] };
            table[5] = .{ .bits = 2, .value = symbols[1] };
            table[6] = .{ .bits = 1, .value = symbols[0] };
            table[7] = .{ .bits = 3, .value = symbols[3] };
            table_size = 8;
        },
        else => return false,
    }

    const goal_size: usize = @as(usize, 1) << kHuffmanTableBits;
    while (table_size != goal_size) {
        @memcpy(table[table_size .. table_size * 2], table[0..table_size]);
        table_size <<= 1;
    }
    return true;
}

/// Порт `HuffmanDecodingData` (dec_huffman.h/cc).
pub const HuffmanDecodingData = struct {
    table: std.ArrayList(HuffmanCode) = .empty,

    pub fn deinit(self: *HuffmanDecodingData, gpa: std.mem.Allocator) void {
        self.table.deinit(gpa);
    }

    /// Порт `HuffmanDecodingData::ReadFromBitStream`.
    /// Возвращает false, если код не удалось прочитать.
    pub fn readFromBitStream(self: *HuffmanDecodingData, gpa: std.mem.Allocator, alphabet_size: usize, br: *BitReader) bool {
        if (alphabet_size > (@as(usize, 1) << kPrefixMaxBits)) return false;

        // simple_code_or_skip: 1 — простой код; 0/2/3 — число пропускаемых длин.
        const simple_code_or_skip: u32 = @intCast(br.readFixedBits(2));
        if (simple_code_or_skip == 1) {
            self.table.resize(gpa, 1 << kHuffmanTableBits) catch return false;
            @memset(self.table.items, HuffmanCode{});
            return readSimpleCode(alphabet_size, br, self.table.items);
        }

        const code_lengths = gpa.alloc(u8, alphabet_size) catch return false;
        defer gpa.free(code_lengths);
        @memset(code_lengths, 0);
        var code_length_code_lengths: [kCodeLengthCodes]u8 = .{0} ** kCodeLengthCodes;
        var space: i32 = 32;
        var num_codes: usize = 0;
        // Статический Хаффман-код для длин кода длин (brotli).
        const huff = [16]HuffmanCode{
            .{ .bits = 2, .value = 0 }, .{ .bits = 2, .value = 4 }, .{ .bits = 2, .value = 3 }, .{ .bits = 3, .value = 2 },
            .{ .bits = 2, .value = 0 }, .{ .bits = 2, .value = 4 }, .{ .bits = 2, .value = 3 }, .{ .bits = 4, .value = 1 },
            .{ .bits = 2, .value = 0 }, .{ .bits = 2, .value = 4 }, .{ .bits = 2, .value = 3 }, .{ .bits = 3, .value = 2 },
            .{ .bits = 2, .value = 0 }, .{ .bits = 2, .value = 4 }, .{ .bits = 2, .value = 3 }, .{ .bits = 4, .value = 5 },
        };
        var i: usize = simple_code_or_skip;
        while (i < kCodeLengthCodes and space > 0) : (i += 1) {
            const code_len_idx = kCodeLengthCodeOrder[i];
            br.refill();
            const p = huff[br.peekFixedBits(4)];
            br.consume(p.bits);
            const v: u8 = @intCast(p.value);
            code_length_code_lengths[code_len_idx] = v;
            if (v != 0) {
                space -= @as(i32, 32) >> @as(u5, @intCast(v));
                num_codes += 1;
            }
        }
        const ok = (num_codes == 1 or space == 0) and
            readHuffmanCodeLengths(gpa, &code_length_code_lengths, alphabet_size, code_lengths, br);
        if (!ok) return false;

        var counts: [16]u16 = .{0} ** 16;
        for (code_lengths) |l| {
            counts[l] += 1;
        }
        self.table.resize(gpa, alphabet_size + 376) catch return false;
        @memset(self.table.items, HuffmanCode{});
        const table_size = buildHuffmanTable(gpa, self.table.items, kHuffmanTableBits, code_lengths, &counts);
        if (table_size == 0 or table_size > self.table.items.len) return false;
        self.table.resize(gpa, table_size) catch return false;
        return true;
    }

    /// Порт `HuffmanDecodingData::ReadSymbol`.
    pub fn readSymbol(self: *const HuffmanDecodingData, br: *BitReader) u16 {
        var table: []const HuffmanCode = self.table.items;
        table = table[br.peekBits(kHuffmanTableBits)..];
        var n_bits: usize = table[0].bits;
        if (n_bits > kHuffmanTableBits) {
            br.consume(kHuffmanTableBits);
            n_bits -= kHuffmanTableBits;
            table = table[table[0].value..];
            table = table[br.peekBits(n_bits)..];
        }
        br.consume(table[0].bits);
        return table[0].value;
    }
};

// ---------------------------------------------------------------------------
// Вспомогательный канонический кодировщик (для тестов).
// ---------------------------------------------------------------------------

/// Канонические коды Хаффмана по длинам (порядок: длина, затем номер символа).
pub fn canonicalCodes(code_lengths: []const u8, codes: []u32) void {
    var next_code: [kPrefixMaxBits + 2]u32 = .{0} ** (kPrefixMaxBits + 2);
    var bl_count: [kPrefixMaxBits + 2]u32 = .{0} ** (kPrefixMaxBits + 2);
    for (code_lengths) |l| bl_count[l] += 1;
    bl_count[0] = 0;
    var code: u32 = 0;
    for (1..kPrefixMaxBits + 2) |l| {
        code = (code + bl_count[l - 1]) << 1;
        next_code[l] = code;
    }
    for (code_lengths, 0..) |l, sym| {
        if (l != 0) {
            codes[sym] = next_code[l];
            next_code[l] += 1;
        }
    }
}

pub inline fn reverseBits(value: u32, n_bits: usize) u32 {
    var v: u32 = value;
    var r: u32 = 0;
    for (0..n_bits) |_| {
        r = (r << 1) | (v & 1);
        v >>= 1;
    }
    return r;
}

/// Построить длины кода Хаффмана по частотам (для тестового энкодера).
/// Простая O(n^2) реализация слияния двух минимальных; для тестовых
/// алфавитов (<= 4096 символов) этого достаточно.
pub fn huffmanCodeLengths(gpa: std.mem.Allocator, freqs: []const u32, lengths: []u8) !void {
    @memset(lengths, 0);
    const Node = struct { weight: u64, left: i32 = -1, right: i32 = -1, sym: i32 = -1 };
    var nodes: std.ArrayList(Node) = .empty;
    defer nodes.deinit(gpa);
    var active: std.ArrayList(usize) = .empty;
    defer active.deinit(gpa);
    for (freqs, 0..) |f, i| {
        if (f != 0) {
            try nodes.append(gpa, .{ .weight = f, .sym = @intCast(i) });
            try active.append(gpa, nodes.items.len - 1);
        }
    }
    if (active.items.len == 0) return;
    if (active.items.len == 1) {
        lengths[@intCast(nodes.items[active.items[0]].sym)] = 1;
        return;
    }
    while (active.items.len > 1) {
        // Найти два минимальных по весу активных узла.
        var m1: usize = 0;
        var m2: usize = 1;
        if (nodes.items[active.items[m1]].weight > nodes.items[active.items[m2]].weight) {
            m1 = 1;
            m2 = 0;
        }
        for (active.items[2..]) |idx| {
            const w = nodes.items[idx].weight;
            if (w < nodes.items[active.items[m1]].weight) {
                m2 = m1;
                m1 = active_pos: {
                    for (active.items, 0..) |a, pos| {
                        if (a == idx) break :active_pos pos;
                    }
                    unreachable;
                };
            } else if (w < nodes.items[active.items[m2]].weight) {
                for (active.items, 0..) |a, pos| {
                    if (a == idx) {
                        m2 = pos;
                        break;
                    }
                }
            }
        }
        const n1 = active.items[m1];
        const n2 = active.items[m2];
        const w = nodes.items[n1].weight + nodes.items[n2].weight;
        try nodes.append(gpa, .{ .weight = w, .left = @intCast(n1), .right = @intCast(n2) });
        // Заменить i1 новым узлом, удалить i2 (порядок не важен).
        active.items[m1] = nodes.items.len - 1;
        _ = active.swapRemove(m2);
    }
    const root = active.items[0];

    const Ctx = struct {
        nodes: []const Node,
        depths: []usize,
        fn walk(c: *const @This(), idx: usize, depth: usize) void {
            const nd = c.nodes[idx];
            if (nd.sym >= 0) {
                c.depths[@intCast(nd.sym)] = depth;
                return;
            }
            c.walk(@intCast(nd.left), depth + 1);
            c.walk(@intCast(nd.right), depth + 1);
        }
    };
    const depths = try gpa.alloc(usize, freqs.len);
    defer gpa.free(depths);
    @memset(depths, 0);
    const ctx = Ctx{ .nodes = nodes.items, .depths = depths };
    ctx.walk(root, 0);
    for (depths, 0..) |d, i| {
        lengths[i] = @intCast(@min(d, kPrefixMaxBits));
    }
}

/// Записать prefix-код (заголовок) в Brotli-формате: простой код при <= 4
/// ненулевых символах, иначе сложный код с code-length-кодом.
pub fn writePrefixCodeHeader(gpa: std.mem.Allocator, w: anytype, alphabet_size: usize, code_lengths: []const u8) !void {
    _ = gpa;
    var nonzero: usize = 0;
    for (code_lengths) |l| {
        if (l != 0) nonzero += 1;
    }
    if (nonzero == 0) {
        // Пустой код не используется; сюда не должны попадать.
        return error.EmptyPrefixCode;
    }
    if (nonzero <= 4) {
        // Простой код: 2 бита = 1, затем NSYM-1 (2 бита), символы, опционально 5-й.
        w.write(2, 1);
        var syms: [5]u16 = .{ 0, 0, 0, 0, 0 };
        var k: usize = 0;
        for (code_lengths, 0..) |l, i| {
            if (l != 0) {
                syms[k] = @intCast(i);
                k += 1;
            }
        }
        const nsym = k;
        w.write(2, nsym - 1);
        const max_bits: usize = if (alphabet_size > 1) bits.floorLog2Nonzero(alphabet_size - 1) + 1 else 0;
        for (0..nsym) |i| {
            w.write(max_bits, syms[i]);
        }
        if (nsym == 4) {
            w.write(1, 0); // без пятого символа
        }
        return;
    }
    // Сложный код: 2 бита = 0 (без пропусков), затем длины code-length-кода
    // через статическую таблицу dec_huffman.cc, затем сами длины.
    w.write(2, 0);
    // Фиксированная схема code-length-кода: символы 1..4 -> длина 3,
    // символы 0 и 5 -> длина 4, символы 6..17 -> длина 5.
    // Проверка Крафта: 4/8 + 2/16 + 12/32 = 1 (полный код).
    const enc: [6][2]usize = .{
        .{ 2, 0 }, // значение 0
        .{ 4, 7 }, // значение 1
        .{ 3, 3 }, // значение 2
        .{ 2, 2 }, // значение 3
        .{ 2, 1 }, // значение 4
        .{ 4, 15 }, // значение 5
    };
    var clcl: [kCodeLengthCodes]u8 = undefined;
    for (0..kCodeLengthCodes) |i| {
        const sym = kCodeLengthCodeOrder[i];
        clcl[i] = if (sym >= 1 and sym <= 4) 3 else if (sym == 0 or sym == 5) 4 else 5;
        w.write(enc[clcl[i]][0], enc[clcl[i]][1]);
    }
    // Канонические коды для code-length-кода: длины должны быть
    // проиндексованы НОМЕРОМ СИМВОЛА (0..17), а не порядком эмиссии.
    var sym_lens: [kCodeLengthCodes]u8 = undefined;
    for (0..kCodeLengthCodes) |s| {
        sym_lens[s] = if (s >= 1 and s <= 4) 3 else if (s == 0 or s == 5) 4 else 5;
    }
    var clcl_codes: [kCodeLengthCodes]u32 = undefined;
    canonicalCodes(&sym_lens, &clcl_codes);
    // Пишем длины символов алфавита (только до последнего ненулевого);
    // декодер сам дополнит нулями после исчерпания места (space == 0).
    var last: usize = 0;
    for (code_lengths, 0..) |l, i| {
        if (l != 0) last = i;
    }
    for (0..last + 1) |sym| {
        const l = code_lengths[sym];
        std.debug.assert(l < kCodeLengthCodes);
        // Длина кода символа l в code-length-алфавите — sym_lens[l].
        const nb = sym_lens[l];
        w.write(nb, reverseBits(clcl_codes[l], nb));
    }
}

// ---------------------------------------------------------------------------
// Тесты
// ---------------------------------------------------------------------------

const testing = std.testing;

test "prefix code: простой код NSYM=2 round trip" {
    const gpa = testing.allocator;
    const BitWriter = @import("../base/bit_io.zig").BitWriter;
    var w = BitWriter.init(gpa);
    defer w.deinit();
    // Простой код с 2 символами {5, 2}: таблица сортирует пару по возрастанию.
    w.write(2, 1); // simple
    w.write(2, 1); // nsym = 2
    w.write(8, 5); // alphabet_size = 256 -> max_bits = 8
    w.write(8, 2);
    w.write(1, 1); // бит 1 -> второй символ пары (5)
    w.zeroPadToByte(); const bytes = try w.takeBytes();
    defer gpa.free(bytes);

    var br = BitReader.init(bytes);
    var data = HuffmanDecodingData{};
    defer data.deinit(gpa);
    try testing.expect(data.readFromBitStream(gpa, 256, &br));
    try testing.expectEqual(@as(u16, 5), data.readSymbol(&br));
    try testing.expect(br.allReadsWithinBounds());
}

test "prefix code: сложный код round trip" {
    const gpa = testing.allocator;
    const BitWriter = @import("../base/bit_io.zig").BitWriter;
    // Полный код (Крафт: 2/4 + 3/8 + 1/16 + 2/32 = 1).
    const lengths = [_]u8{ 2, 2, 3, 3, 3, 4, 5, 5 };
    var w = BitWriter.init(gpa);
    defer w.deinit();
    try writePrefixCodeHeader(gpa, &w, 8, &lengths);
    var codes: [8]u32 = undefined;
    canonicalCodes(&lengths, &codes);
    const symbols = [_]u16{ 0, 6, 1, 7, 5, 3 };
    for (symbols) |s| {
        w.write(lengths[s], reverseBits(codes[s], lengths[s]));
    }
    w.zeroPadToByte(); const bytes = try w.takeBytes();
    defer gpa.free(bytes);

    var br = BitReader.init(bytes);
    var data = HuffmanDecodingData{};
    defer data.deinit(gpa);
    try testing.expect(data.readFromBitStream(gpa, 8, &br));
    for (symbols) |s| {
        try testing.expectEqual(s, data.readSymbol(&br));
    }
    try testing.expect(br.allReadsWithinBounds());
}

test "prefix code: RLE repeat-prev в ReadHuffmanCodeLengths" {
    const gpa = testing.allocator;
    const BitWriter = @import("../base/bit_io.zig").BitWriter;
    // Полный код из 8 символов: [3 x 8] (Kraft: 8/8 = 1). Пишем через
    // repeat-prev: первые 3 тройки символом 3, затем repeat-prev на 5 троек:
    // repeat = 3 + extra; нужен delta = 5 -> extra = 2 (3 + 2 = 5).
    var lengths: [8]u8 = .{3} ** 8;
    _ = &lengths;
    var w = BitWriter.init(gpa);
    defer w.deinit();
    w.write(2, 0); // сложный код
    const enc: [6][2]usize = .{ .{ 2, 0 }, .{ 4, 7 }, .{ 3, 3 }, .{ 2, 2 }, .{ 2, 1 }, .{ 4, 15 } };
    var clcl: [kCodeLengthCodes]u8 = undefined;
    for (0..kCodeLengthCodes) |i| {
        const sym = kCodeLengthCodeOrder[i];
        clcl[i] = if (sym >= 1 and sym <= 4) 3 else if (sym == 0 or sym == 5) 4 else 5;
        w.write(enc[clcl[i]][0], enc[clcl[i]][1]);
    }
    var sym_lens: [kCodeLengthCodes]u8 = undefined;
    for (0..kCodeLengthCodes) |sn| {
        sym_lens[sn] = if (sn >= 1 and sn <= 4) 3 else if (sn == 0 or sn == 5) 4 else 5;
    }
    var clcl_codes: [kCodeLengthCodes]u32 = undefined;
    canonicalCodes(&sym_lens, &clcl_codes);
    // Три тройки символом "3":
    w.write(3, reverseBits(clcl_codes[3], 3));
    w.write(3, reverseBits(clcl_codes[3], 3));
    w.write(3, reverseBits(clcl_codes[3], 3));
    // repeat-prev (символ 16): extra = 2 бита, значение 2 => 5 повторов.
    w.write(5, reverseBits(clcl_codes[16], 5));
    w.write(2, 2);
    // Итого 3 + 5 = 8 символов со значением 3; space: 8 * (32768>>3) = 32768 ✓.
    // Канонические коды: символы 0..7 получают коды 0..7 длины 3.
    for (0..8) |sym| {
        w.write(3, reverseBits(@intCast(sym), 3));
    }
    w.zeroPadToByte(); const bytes = try w.takeBytes();
    defer gpa.free(bytes);

    var br = BitReader.init(bytes);
    var data = HuffmanDecodingData{};
    defer data.deinit(gpa);
    try testing.expect(data.readFromBitStream(gpa, 8, &br));
    for (0..8) |sym| {
        try testing.expectEqual(@as(u16, @intCast(sym)), data.readSymbol(&br));
    }
    try testing.expect(br.allReadsWithinBounds());
}

test "buildHuffmanTable: единственный символ" {
    const gpa = testing.allocator;
    var counts: [16]u16 = .{0} ** 16;
    counts[1] = 1;
    var lengths = [_]u8{ 1, 0 };
    var table: [256]HuffmanCode = undefined;
    const size = buildHuffmanTable(gpa, &table, kHuffmanTableBits, &lengths, &counts);
    try testing.expect(size > 0);
    try testing.expectEqual(@as(u8, 0), table[0].bits);
    try testing.expectEqual(@as(u16, 0), table[0].value);
}

test "huffmanCodeLengths: полный код" {
    const gpa = testing.allocator;
    const freqs = [_]u32{ 10, 2, 3, 1, 8, 2, 1, 5 };
    var lengths: [8]u8 = undefined;
    try huffmanCodeLengths(gpa, &freqs, &lengths);
    // Проверка Крафта: сумма 2^-len == 1.
    var kraft: u32 = 0;
    var max_len: usize = 0;
    for (lengths) |l| {
        if (l != 0) kraft += @as(u32, 1) << @intCast(kPrefixMaxBits - l);
        max_len = @max(max_len, l);
    }
    try testing.expectEqual(@as(u32, 1) << kPrefixMaxBits, kraft);
    try testing.expect(max_len <= kPrefixMaxBits);
    // Кодируем и декодируем.
    const BitWriter = @import("../base/bit_io.zig").BitWriter;
    var w = BitWriter.init(gpa);
    defer w.deinit();
    try writePrefixCodeHeader(gpa, &w, 8, &lengths);
    var codes: [8]u32 = undefined;
    canonicalCodes(&lengths, &codes);
    const symbols = [_]u16{ 0, 4, 0, 0, 7, 1, 4 };
    for (symbols) |s| {
        w.write(lengths[s], reverseBits(codes[s], lengths[s]));
    }
    w.zeroPadToByte(); const bytes = try w.takeBytes();
    defer gpa.free(bytes);
    var br = BitReader.init(bytes);
    var data = HuffmanDecodingData{};
    defer data.deinit(gpa);
    try testing.expect(data.readFromBitStream(gpa, 8, &br));
    for (symbols) |s| {
        try testing.expectEqual(s, data.readSymbol(&br));
    }
    try testing.expect(br.allReadsWithinBounds());
}

test "prefix code: простой код NSYM=1 (нулевые биты на символ)" {
    const gpa = testing.allocator;
    const BitWriter = @import("../base/bit_io.zig").BitWriter;
    var lengths = [_]u8{0} ** 300;
    lengths[42] = 1;
    var w = BitWriter.init(gpa);
    defer w.deinit();
    try writePrefixCodeHeader(gpa, &w, 300, &lengths);
    w.zeroPadToByte(); const bytes = try w.takeBytes();
    defer gpa.free(bytes);
    var br = BitReader.init(bytes);
    var data = HuffmanDecodingData{};
    defer data.deinit(gpa);
    try testing.expect(data.readFromBitStream(gpa, 300, &br));
    for (0..5) |_| {
        // Единственный символ кодируется нулём бит.
        try testing.expectEqual(@as(u16, 42), data.readSymbol(&br));
    }
    try testing.expect(br.allReadsWithinBounds());
}
