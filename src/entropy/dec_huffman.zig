//! Huffman decoding for JPEG XL prefix codes. Bit-exact port of libjxl
//! `huffman_table.h/cc` and `dec_huffman.h/cc`.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const bits = @import("../base/bits.zig");
const ans_common = @import("ans_common.zig");

const BitReader = bit_io.BitReader;
const prefix_max_bits = ans_common.prefix_max_bits;

/// Number of table bits for the root Huffman table (libjxl: kHuffmanTableBits).
pub const k_huffman_table_bits: usize = 8;

pub const HuffmanCode = struct {
    /// number of bits used for this symbol
    bits: u8 = 0,
    /// symbol value or table offset
    value: u16 = 0,
};

/// Port of `jxl::BuildHuffmanTable`: builds a (two-level) Huffman lookup
/// table assuming code lengths are given in symbol order. `count` must be
/// the histogram of code lengths (count[0..16]); it is modified in place,
/// exactly as in libjxl. Returns the populated size of the table, or 0 on
/// error. `root_table` must have space for at least
/// `alphabet_size + 376` entries (as allocated by callers in libjxl).
pub fn buildHuffmanTable(
    allocator: std.mem.Allocator,
    root_table: []HuffmanCode,
    root_bits: usize,
    code_lengths: []const u8,
    count: *[16]u16,
) !u32 {
    var key: i32 = 0;
    var step: i32 = 0;
    var low: i32 = 0;
    var mask: i32 = 0;

    if (code_lengths.len > (@as(usize, 1) << prefix_max_bits)) return 0;

    // Offsets in sorted table for each length.
    var offset: [prefix_max_bits + 1]u16 = undefined;
    var max_length: usize = 1;

    {
        var sum: u16 = 0;
        var len: usize = 1;
        while (len <= prefix_max_bits) : (len += 1) {
            offset[len] = sum;
            if (count[len] != 0) {
                sum +%= count[len];
                max_length = len;
            }
        }
    }
    offset[0] = 0;

    // Symbols sorted by code length, by symbol order within each length.
    const sorted = try allocator.alloc(u16, code_lengths.len);
    defer allocator.free(sorted);
    for (code_lengths, 0..) |l, symbol| {
        if (l != 0) {
            sorted[offset[l]] = @intCast(symbol);
            offset[l] += 1;
        }
    }

    var table: []HuffmanCode = root_table;
    var table_bits: usize = root_bits;
    var table_size: usize = @as(usize, 1) << @intCast(table_bits);
    var total_size: usize = table_size;

    // Special case: code with only one value.
    if (offset[prefix_max_bits] == 1) {
        const code = HuffmanCode{ .bits = 0, .value = sorted[0] };
        for (0..total_size) |k| {
            table[k] = code;
        }
        return @intCast(total_size);
    }

    // Fill in root table; reduce the table size if possible and create
    // repetitions by copying in the coming loop.
    if (table_bits > max_length) {
        table_bits = max_length;
        table_size = @as(usize, 1) << @intCast(table_bits);
    }
    key = 0;
    var symbol: usize = 0;
    var code = HuffmanCode{ .bits = 1, .value = 0 };
    step = 2;
    while (true) {
        while (count[code.bits] != 0) : (count[code.bits] -= 1) {
            code.value = sorted[symbol];
            symbol += 1;
            replicateValue(table[@intCast(key)..], step, table_size, code);
            key = getNextKey(key, code.bits);
        }
        step <<= 1;
        code.bits += 1;
        if (code.bits > table_bits) break;
    }

    // If root_bits != table_bits we only created one fraction of the table,
    // and we need to replicate it now. After this, table_size == total_size.
    while (total_size != table_size) {
        @memcpy(table[table_size..][0..table_size], table[0..table_size]);
        table_size <<= 1;
    }

    // Fill in 2nd level tables and add pointers to root table.
    // `table_offset` mirrors the C++ `table - root_table` cursor.
    var table_offset: usize = 0;
    mask = @as(i32, @intCast(total_size)) - 1;
    low = -1;
    var len: usize = root_bits + 1;
    step = 2;
    while (len <= max_length) : ({
        len += 1;
        step <<= 1;
    }) {
        while (count[len] != 0) : (count[len] -= 1) {
            if ((key & mask) != low) {
                table_offset += table_size; // advance by previous table size
                table_bits = nextTableBitSize(count, len, root_bits);
                table_size = @as(usize, 1) << @intCast(table_bits);
                total_size += table_size;
                low = key & mask;
                if (total_size > root_table.len) return error.TableTooSmall;
                root_table[@intCast(low)] = .{
                    .bits = @intCast(table_bits + root_bits),
                    .value = @intCast(table_offset - @as(usize, @intCast(low))),
                };
            }
            code.bits = @intCast(len - root_bits);
            code.value = sorted[symbol];
            symbol += 1;
            replicateValue(root_table[table_offset + @as(usize, @intCast(key >> @intCast(root_bits)))..], step, table_size, code);
            key = getNextKey(key, @intCast(len));
        }
    }

    return @intCast(total_size);
}

/// Returns reverse(reverse(key, len) + 1, len), where reverse(key, len) is
/// the bit-wise reversal of the len least significant bits of key.
fn getNextKey(key: i32, len: u8) i32 {
    var step: i32 = @as(i32, 1) << @intCast(len - 1);
    while ((key & step) != 0) {
        step >>= 1;
    }
    return (key & (step - 1)) + step;
}

/// Stores code in table[0], table[step], table[2*step], ..., table[end-1].
fn replicateValue(table: []HuffmanCode, step: i32, end: usize, code: HuffmanCode) void {
    var end_i: i64 = @intCast(end);
    while (true) {
        end_i -= step;
        table[@intCast(end_i)] = code;
        if (end_i <= 0) break;
    }
}

/// Returns the table width of the next 2nd level table. `count` is the
/// histogram of bit lengths for the remaining symbols, `len` is the code
/// length of the next processed symbol.
fn nextTableBitSize(count: *const [16]u16, len0: usize, root_bits: usize) usize {
    var left: usize = @as(usize, 1) << @intCast(len0 - root_bits);
    var len = len0;
    while (len < prefix_max_bits) {
        if (left <= count[len]) break;
        left -= count[len];
        len += 1;
        left <<= 1;
    }
    return len - root_bits;
}

// ---------------------------------------------------------------------------
// dec_huffman.h/cc
// ---------------------------------------------------------------------------

const k_code_length_codes = 18;
const k_code_length_code_order = [k_code_length_codes]u8{
    1, 2, 3, 4, 0, 5, 17, 6, 16, 7, 8, 9, 10, 11, 12, 13, 14, 15,
};
const k_default_code_length: u8 = 8;
const k_code_length_repeat_code: u8 = 16;

pub const HuffmanDecodingData = struct {
    table: []HuffmanCode = &.{},

    pub fn deinit(self: *HuffmanDecodingData, allocator: std.mem.Allocator) void {
        if (self.table.len > 0) allocator.free(self.table);
        self.table = &.{};
    }

    /// Decodes the next Huffman coded symbol from the bit-stream.
    pub fn readSymbol(self: *const HuffmanDecodingData, br: *BitReader) u16 {
        var table: []const HuffmanCode = self.table;
        table = table[br.peekBits(k_huffman_table_bits)..];
        var n_bits = table[0].bits;
        if (n_bits > k_huffman_table_bits) {
            br.consume(k_huffman_table_bits);
            n_bits -= k_huffman_table_bits;
            table = table[table[0].value..];
            table = table[br.peekBits(n_bits)..];
        }
        br.consume(table[0].bits);
        return table[0].value;
    }

    /// Port of `HuffmanDecodingData::ReadFromBitStream`.
    pub fn readFromBitStream(self: *HuffmanDecodingData, allocator: std.mem.Allocator, alphabet_size: usize, br: *BitReader) !void {
        if (alphabet_size > (@as(usize, 1) << prefix_max_bits)) return error.InvalidData;

        // simple_code_or_skip is used as follows:
        // 1 for simple code; 0 for no skipping, 2 skips 2 code lengths,
        // 3 skips 3 code lengths.
        const simple_code_or_skip = br.readFixedBits(2);
        if (simple_code_or_skip == 1) {
            self.table = try allocator.alloc(HuffmanCode, @as(usize, 1) << k_huffman_table_bits);
            errdefer allocator.free(self.table);
            return readSimpleCode(allocator, alphabet_size, br, self.table);
        }

        const code_lengths = try allocator.alloc(u8, alphabet_size);
        defer allocator.free(code_lengths);
        @memset(code_lengths, 0);

        var code_length_code_lengths = [_]u8{0} ** k_code_length_codes;
        var space: i32 = 32;
        var num_codes: usize = 0;
        // Static Huffman code for the code length code lengths.
        const huff = [16]HuffmanCode{
            .{ .bits = 2, .value = 0 }, .{ .bits = 2, .value = 4 }, .{ .bits = 2, .value = 3 }, .{ .bits = 3, .value = 2 },
            .{ .bits = 2, .value = 0 }, .{ .bits = 2, .value = 4 }, .{ .bits = 2, .value = 3 }, .{ .bits = 4, .value = 1 },
            .{ .bits = 2, .value = 0 }, .{ .bits = 2, .value = 4 }, .{ .bits = 2, .value = 3 }, .{ .bits = 3, .value = 2 },
            .{ .bits = 2, .value = 0 }, .{ .bits = 2, .value = 4 }, .{ .bits = 2, .value = 3 }, .{ .bits = 4, .value = 5 },
        };
        var i: usize = simple_code_or_skip;
        while (i < k_code_length_codes and space > 0) : (i += 1) {
            const code_len_idx = k_code_length_code_order[i];
            br.refill();
            var p: []const HuffmanCode = &huff;
            p = p[br.peekFixedBits(4)..];
            br.consume(p[0].bits);
            const v: u8 = @intCast(p[0].value);
            code_length_code_lengths[code_len_idx] = v;
            if (v != 0) {
                space -= @as(i32, 32) >> @intCast(v);
                num_codes += 1;
            }
        }

        const ok = (num_codes == 1 or space == 0) and
            readHuffmanCodeLengths(allocator, &code_length_code_lengths, alphabet_size, code_lengths, br);
        if (!ok) return error.InvalidData;

        var counts = [_]u16{0} ** 16;
        for (code_lengths) |l| {
            counts[l] += 1;
        }
        self.table = try allocator.alloc(HuffmanCode, alphabet_size + 376);
        errdefer allocator.free(self.table);
        const table_size = try buildHuffmanTable(allocator, self.table, k_huffman_table_bits, code_lengths, &counts);
        // Shrink to the actual size.
        const shrunk = try allocator.realloc(self.table, table_size);
        self.table = shrunk;
        if (table_size == 0) return error.InvalidData;
    }
};

/// Port of `jxl::ReadHuffmanCodeLengths`.
fn readHuffmanCodeLengths(
    allocator: std.mem.Allocator,
    code_length_code_lengths: *const [k_code_length_codes]u8,
    num_symbols: usize,
    code_lengths: []u8,
    br: *BitReader,
) bool {
    var symbol: usize = 0;
    var prev_code_len: u8 = k_default_code_length;
    var repeat: i32 = 0;
    var repeat_code_len: u8 = 0;
    var space: i32 = 32768;
    var table: [32]HuffmanCode = undefined;

    var counts = [_]u16{0} ** 16;
    for (0..k_code_length_codes) |i| {
        counts[code_length_code_lengths[i]] += 1;
    }
    const built = buildHuffmanTable(allocator, &table, 5, code_length_code_lengths, &counts) catch return false;
    if (built == 0) return false;

    while (symbol < num_symbols and space > 0) {
        br.refill();
        var p: []const HuffmanCode = table[0..built];
        p = p[br.peekFixedBits(5)..];
        br.consume(p[0].bits);
        const code_len: u8 = @intCast(p[0].value);
        if (code_len < k_code_length_repeat_code) {
            repeat = 0;
            code_lengths[symbol] = code_len;
            symbol += 1;
            if (code_len != 0) {
                prev_code_len = code_len;
                space -= @as(i32, 32768) >> @intCast(code_len);
            }
        } else {
            const extra_bits: usize = code_len - 14;
            var new_len: u8 = 0;
            if (code_len == k_code_length_repeat_code) {
                new_len = prev_code_len;
            }
            if (repeat_code_len != new_len) {
                repeat = 0;
                repeat_code_len = new_len;
            }
            // NOTE: old_repeat is captured AFTER the repeat_code_len reset
            // (libjxl order); capturing it earlier breaks mixed 16/17 runs.
            const old_repeat = repeat;
            if (repeat > 0) {
                repeat -= 2;
                repeat <<= @intCast(extra_bits);
            }
            repeat +%= @as(i32, @intCast(br.readBits(extra_bits))) + 3;
            const repeat_delta: usize = @intCast(repeat - old_repeat);
            if (symbol + repeat_delta > num_symbols) {
                return false;
            }
            @memset(code_lengths[symbol..][0..repeat_delta], repeat_code_len);
            symbol += repeat_delta;
            if (repeat_code_len != 0) {
                space -= @as(i32, @intCast(repeat_delta)) << @intCast(15 - @as(u6, @intCast(repeat_code_len)));
            }
        }
    }
    if (space != 0) {
        return false;
    }
    @memset(code_lengths[symbol..], 0);
    return true;
}

/// Port of `jxl::ReadSimpleCode` (anonymous namespace in dec_huffman.cc).
fn readSimpleCode(allocator: std.mem.Allocator, alphabet_size: usize, br: *BitReader, table: []HuffmanCode) !void {
    _ = allocator;
    const max_bits: usize = if (alphabet_size > 1) bits.floorLog2Nonzero(alphabet_size - 1) + 1 else 0;

    var num_symbols: usize = @as(usize, @intCast(br.readFixedBits(2))) + 1;

    var symbols = [_]u16{0} ** 4;
    for (0..num_symbols) |i| {
        const symbol: u16 = @intCast(br.readBits(max_bits));
        if (symbol >= alphabet_size) {
            return error.InvalidData;
        }
        symbols[i] = symbol;
    }

    for (0..num_symbols - 1) |i| {
        for (i + 1..num_symbols) |j| {
            if (symbols[i] == symbols[j]) return error.InvalidData;
        }
    }

    // 4 symbols have the option to encode 5.
    if (num_symbols == 4) num_symbols += @as(usize, @intCast(br.readFixedBits(1)));

    const swapSymbols = struct {
        fn f(arr: *[4]u16, i: usize, j: usize) void {
            const t = arr[j];
            arr[j] = arr[i];
            arr[i] = t;
        }
    }.f;

    var table_size: usize = 1;
    switch (num_symbols) {
        1 => {
            table[0] = .{ .bits = 0, .value = symbols[0] };
        },
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
        else => return error.InvalidData,
    }

    const goal_size = @as(usize, 1) << k_huffman_table_bits;
    while (table_size != goal_size) {
        @memcpy(table[table_size..][0..table_size], table[0..table_size]);
        table_size <<= 1;
    }
}

// ---------------------------------------------------------------------------

test "huffman simple 2-symbol code" {
    const alloc = std.testing.allocator;
    var data = HuffmanDecodingData{};
    defer data.deinit(alloc);
    var w = @import("../base/bit_io.zig").BitWriter.init(alloc);
    defer w.deinit();

    // simple_code_or_skip = 1 (simple code)
    w.write(2, 1);
    // num_symbols - 1 = 1
    w.write(2, 1);
    // max_bits for alphabet_size 2 = 1
    w.write(1, 0); // symbol 0
    w.write(1, 1); // symbol 1
    // Payload: symbols 0 then 1.
    w.write(1, 0);
    w.write(1, 1);

    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    try data.readFromBitStream(alloc, 2, &br);

    // Reading bit 0 -> symbol 0 (1 bit), bit 1 -> symbol 1 (1 bit).
    try std.testing.expectEqual(@as(u16, 0), data.readSymbol(&br));
    try std.testing.expectEqual(@as(u16, 1), data.readSymbol(&br));
    try br.close();
}

test "huffman simple 1-symbol code" {
    const alloc = std.testing.allocator;
    var data = HuffmanDecodingData{};
    defer data.deinit(alloc);
    var w = @import("../base/bit_io.zig").BitWriter.init(alloc);
    defer w.deinit();

    w.write(2, 1); // simple code
    w.write(2, 0); // num_symbols - 1 = 0
    w.write(3, 5); // max_bits = 3 for alphabet 8, symbol 5

    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    try data.readFromBitStream(alloc, 8, &br);
    try std.testing.expectEqual(@as(u16, 5), data.readSymbol(&br));
    try std.testing.expectEqual(@as(u16, 5), data.readSymbol(&br));
    try br.close();
}

test "buildHuffmanTable: full alphabet" {
    const alloc = std.testing.allocator;
    // 4 symbols with lengths 2, 2, 2, 2 -> complete code replicated to the
    // root table size (256), matching libjxl BuildHuffmanTable semantics.
    const lengths = [_]u8{ 2, 2, 2, 2 };
    var counts = [_]u16{0} ** 16;
    for (lengths) |l| counts[l] += 1;
    const table = try alloc.alloc(HuffmanCode, lengths.len + 376);
    defer alloc.free(table);
    const size = try buildHuffmanTable(alloc, table, k_huffman_table_bits, &lengths, &counts);
    try std.testing.expectEqual(@as(u32, 256), size);
    // Codes are assigned in reversed-key order: 00->s0, 10->s1, 01->s2,
    // 11->s3, i.e. table index i (peeked bits) maps:
    //   0->sym0, 1->sym2, 2->sym1, 3->sym3, replicated 64 times.
    for (0..64) |rep| {
        try std.testing.expectEqual(@as(u16, 0), table[rep * 4 + 0].value);
        try std.testing.expectEqual(@as(u16, 2), table[rep * 4 + 1].value);
        try std.testing.expectEqual(@as(u16, 1), table[rep * 4 + 2].value);
        try std.testing.expectEqual(@as(u16, 3), table[rep * 4 + 3].value);
        for (0..4) |k| try std.testing.expectEqual(@as(u8, 2), table[rep * 4 + k].bits);
    }
}
