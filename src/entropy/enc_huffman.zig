//! Huffman tree construction and serialization for prefix codes.
//! Bit-exact port of libjxl `enc_huffman_tree.h/cc` and `enc_huffman.h/cc`.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const bits = @import("../base/bits.zig");
const ans_common = @import("ans_common.zig");

const BitWriter = bit_io.BitWriter;
const prefix_max_bits = ans_common.prefix_max_bits;

pub const HuffmanTree = struct {
    total_count: u32,
    index_left: i16,
    index_right_or_value: i16,
};

fn setDepth(p: *const HuffmanTree, pool: []const HuffmanTree, depth: []u8, level: u8) void {
    if (p.index_left >= 0) {
        const lvl = level + 1;
        setDepth(&pool[@intCast(p.index_left)], pool, depth, lvl);
        setDepth(&pool[@intCast(p.index_right_or_value)], pool, depth, lvl);
    } else {
        depth[@intCast(p.index_right_or_value)] = level;
    }
}

/// Compare the root nodes, least popular first; indices are in decreasing
/// order before sorting is applied.
fn compare(_: void, v0: HuffmanTree, v1: HuffmanTree) bool {
    if (v0.total_count != v1.total_count) {
        return v0.total_count < v1.total_count;
    }
    return v0.index_right_or_value > v1.index_right_or_value;
}

/// Port of `jxl::CreateHuffmanTree`: builds code depths (tree limited to
/// `tree_limit` levels) for the given histogram `data`.
pub fn createHuffmanTree(allocator: std.mem.Allocator, data: []const u32, tree_limit: u5, depth: []u8) void {
    std.debug.assert(depth.len >= data.len);
    // For block sizes below 64 kB, we never need a second iteration of this
    // loop (see libjxl comment).
    var count_limit: u32 = 1;
    while (true) : (count_limit *= 2) {
        var tree: std.ArrayList(HuffmanTree) = .empty;
        defer tree.deinit(allocator);

        {
            var i: usize = data.len;
            while (i != 0) {
                i -= 1;
                if (data[i] != 0) {
                    const count = @max(data[i], count_limit - 1);
                    tree.append(allocator, .{
                        .total_count = count,
                        .index_left = -1,
                        .index_right_or_value = @intCast(i),
                    }) catch @panic("OOM");
                }
            }
        }

        const n = tree.items.len;
        if (n == 1) {
            // Fake value; will be fixed on upper level.
            depth[@intCast(tree.items[0].index_right_or_value)] = 1;
            return;
        }
        if (n == 0) {
            return; // No symbols; nothing to do (defensive, libjxl asserts).
        }

        std.mem.sort(HuffmanTree, tree.items, {}, compare);

        // The nodes are:
        // [0, n): the sorted leaf nodes that we start with.
        // [n]: sentinel.
        // [n + 1, 2n): new parent nodes are added here, starting from (n+1).
        // [2n]: final sentinel.
        const sentinel = HuffmanTree{
            .total_count = std.math.maxInt(u32),
            .index_left = -1,
            .index_right_or_value = -1,
        };
        tree.append(allocator, sentinel) catch @panic("OOM");
        tree.append(allocator, sentinel) catch @panic("OOM");

        var i: usize = 0; // Points to the next leaf node.
        var j: usize = n + 1; // Points to the next non-leaf node.
        var k: usize = n - 1;
        while (k != 0) : (k -= 1) {
            var left: usize = undefined;
            var right: usize = undefined;
            if (tree.items[i].total_count <= tree.items[j].total_count) {
                left = i;
                i += 1;
            } else {
                left = j;
                j += 1;
            }
            if (tree.items[i].total_count <= tree.items[j].total_count) {
                right = i;
                i += 1;
            } else {
                right = j;
                j += 1;
            }

            // The sentinel node becomes the parent node.
            const j_end = tree.items.len - 1;
            tree.items[j_end].total_count =
                tree.items[left].total_count + tree.items[right].total_count;
            tree.items[j_end].index_left = @intCast(left);
            tree.items[j_end].index_right_or_value = @intCast(right);

            // Add back the last sentinel node.
            tree.append(allocator, sentinel) catch @panic("OOM");
        }
        std.debug.assert(tree.items.len == 2 * n + 1);
        setDepth(&tree.items[2 * n - 1], tree.items, depth, 0);

        // We need to pack the Huffman tree in tree_limit bits. If this was
        // not successful, add fake entities to the lowest values and retry.
        var max_depth: u8 = 0;
        for (depth[0..data.len]) |d| {
            max_depth = @max(max_depth, d);
        }
        if (max_depth <= tree_limit) {
            return;
        }
    }
}

fn reverseArr(v: []u8, start0: usize, end0: usize) void {
    var start = start0;
    var end = end0 - 1;
    while (start < end) {
        const tmp = v[start];
        v[start] = v[end];
        v[end] = tmp;
        start += 1;
        end -= 1;
    }
}

/// Port of `jxl::WriteHuffmanTreeRepetitions`.
fn writeHuffmanTreeRepetitions(
    previous_value: u8,
    value: u8,
    repetitions0: usize,
    tree_size: *usize,
    tree: []u8,
    extra_bits_data: []u8,
) void {
    var repetitions = repetitions0;
    std.debug.assert(repetitions > 0);
    if (previous_value != value) {
        tree[tree_size.*] = value;
        extra_bits_data[tree_size.*] = 0;
        tree_size.* += 1;
        repetitions -= 1;
    }
    if (repetitions == 7) {
        tree[tree_size.*] = value;
        extra_bits_data[tree_size.*] = 0;
        tree_size.* += 1;
        repetitions -= 1;
    }
    if (repetitions < 3) {
        for (0..repetitions) |_| {
            tree[tree_size.*] = value;
            extra_bits_data[tree_size.*] = 0;
            tree_size.* += 1;
        }
    } else {
        repetitions -= 3;
        const start = tree_size.*;
        while (true) {
            tree[tree_size.*] = 16;
            extra_bits_data[tree_size.*] = @intCast(repetitions & 0x3);
            tree_size.* += 1;
            repetitions >>= 2;
            if (repetitions == 0) {
                break;
            }
            repetitions -= 1;
        }
        reverseArr(tree, start, tree_size.*);
        reverseArr(extra_bits_data, start, tree_size.*);
    }
}

/// Port of `jxl::WriteHuffmanTreeRepetitionsZeros`.
fn writeHuffmanTreeRepetitionsZeros(
    repetitions0: usize,
    tree_size: *usize,
    tree: []u8,
    extra_bits_data: []u8,
) void {
    var repetitions = repetitions0;
    if (repetitions == 11) {
        tree[tree_size.*] = 0;
        extra_bits_data[tree_size.*] = 0;
        tree_size.* += 1;
        repetitions -= 1;
    }
    if (repetitions < 3) {
        for (0..repetitions) |_| {
            tree[tree_size.*] = 0;
            extra_bits_data[tree_size.*] = 0;
            tree_size.* += 1;
        }
    } else {
        repetitions -= 3;
        const start = tree_size.*;
        while (true) {
            tree[tree_size.*] = 17;
            extra_bits_data[tree_size.*] = @intCast(repetitions & 0x7);
            tree_size.* += 1;
            repetitions >>= 3;
            if (repetitions == 0) {
                break;
            }
            repetitions -= 1;
        }
        reverseArr(tree, start, tree_size.*);
        reverseArr(extra_bits_data, start, tree_size.*);
    }
}

/// Port of `jxl::DecideOverRleUse`.
fn decideOverRleUse(depth: []const u8, use_rle_for_non_zero: *bool, use_rle_for_zero: *bool) void {
    var total_reps_zero: usize = 0;
    var total_reps_non_zero: usize = 0;
    var count_reps_zero: usize = 1;
    var count_reps_non_zero: usize = 1;
    var i: usize = 0;
    while (i < depth.len) {
        const value = depth[i];
        var reps: usize = 1;
        var k = i + 1;
        while (k < depth.len and depth[k] == value) : (k += 1) {
            reps += 1;
        }
        if (reps >= 3 and value == 0) {
            total_reps_zero += reps;
            count_reps_zero += 1;
        }
        if (reps >= 4 and value != 0) {
            total_reps_non_zero += reps;
            count_reps_non_zero += 1;
        }
        i += reps;
    }
    use_rle_for_non_zero.* = total_reps_non_zero > count_reps_non_zero * 2;
    use_rle_for_zero.* = total_reps_zero > count_reps_zero * 2;
}

/// Port of `jxl::WriteHuffmanTree`.
pub fn writeHuffmanTree(depth: []const u8, tree_size: *usize, tree: []u8, extra_bits_data: []u8) void {
    var previous_value: u8 = 8;

    // Throw away trailing zeros.
    var new_length = depth.len;
    for (0..depth.len) |i| {
        if (depth[depth.len - i - 1] == 0) {
            new_length -= 1;
        } else {
            break;
        }
    }

    // First gather statistics on if it is a good idea to do rle.
    var use_rle_for_non_zero = false;
    var use_rle_for_zero = false;
    if (depth.len > 50) {
        decideOverRleUse(depth[0..new_length], &use_rle_for_non_zero, &use_rle_for_zero);
    }

    // Actual rle coding.
    var i: usize = 0;
    while (i < new_length) {
        const value = depth[i];
        var reps: usize = 1;
        if ((value != 0 and use_rle_for_non_zero) or
            (value == 0 and use_rle_for_zero))
        {
            var k = i + 1;
            while (k < new_length and depth[k] == value) : (k += 1) {
                reps += 1;
            }
        }
        if (value == 0) {
            writeHuffmanTreeRepetitionsZeros(reps, tree_size, tree, extra_bits_data);
        } else {
            writeHuffmanTreeRepetitions(previous_value, value, reps, tree_size, tree, extra_bits_data);
            previous_value = value;
        }
        i += reps;
    }
}

fn reverseBits(num_bits: usize, bits0: u16) u16 {
    const k_lut = [16]u16{
        0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
        0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
    };
    var retval: u16 = k_lut[bits0 & 0xf];
    var b = bits0;
    var i: usize = 4;
    while (i < num_bits) : (i += 4) {
        retval <<= 4;
        b >>= 4;
        retval |= k_lut[b & 0xf];
    }
    retval >>= @intCast((-@as(i32, @intCast(num_bits))) & 0x3);
    return retval;
}

/// Port of `jxl::ConvertBitDepthsToSymbols`.
pub fn convertBitDepthsToSymbols(depth: []const u8, bits_out: []u16) void {
    // In Brotli, all bit depths are [1..15]; 0 means the symbol does not
    // exist.
    const k_max_bits = 16; // 0..15 are values for bits
    var bl_count = [_]u16{0} ** k_max_bits;
    for (depth) |d| {
        bl_count[d] += 1;
    }
    bl_count[0] = 0;
    var next_code = [_]u16{0} ** k_max_bits;
    {
        var code: u32 = 0;
        for (1..k_max_bits) |i| {
            code = (code + bl_count[i - 1]) << 1;
            next_code[i] = @intCast(code);
        }
    }
    for (depth, 0..) |d, i| {
        if (d != 0) {
            bits_out[i] = reverseBits(d, next_code[d]);
            next_code[d] += 1;
        }
    }
}

// ---------------------------------------------------------------------------
// enc_huffman.cc
// ---------------------------------------------------------------------------

const k_code_length_codes = 18;

/// Port of `StoreHuffmanTreeOfHuffmanTreeToBitMask`.
fn storeHuffmanTreeOfHuffmanTreeToBitMask(num_codes: usize, code_length_bitdepth: *const [k_code_length_codes]u8, writer: *BitWriter) void {
    const k_storage_order = [k_code_length_codes]u8{
        1, 2, 3, 4, 0, 5, 17, 6, 16, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    };
    // The bit lengths of the Huffman code over the code length alphabet
    // are compressed with the following static Huffman code:
    //   Symbol   Code
    //   ------   ----
    //   0          00
    //   1        1110
    //   2         110
    //   3          01
    //   4          10
    //   5        1111
    const k_symbols = [6]u8{ 0, 7, 3, 2, 1, 15 };
    const k_bit_lengths = [6]u8{ 2, 4, 3, 2, 2, 4 };

    // Throw away trailing zeros.
    var codes_to_store: usize = k_code_length_codes;
    if (num_codes > 1) {
        while (codes_to_store > 0) : (codes_to_store -= 1) {
            if (code_length_bitdepth[k_storage_order[codes_to_store - 1]] != 0) {
                break;
            }
        }
    }
    var skip_some: usize = 0; // skips none.
    if (code_length_bitdepth[k_storage_order[0]] == 0 and
        code_length_bitdepth[k_storage_order[1]] == 0)
    {
        skip_some = 2; // skips two.
        if (code_length_bitdepth[k_storage_order[2]] == 0) {
            skip_some = 3; // skips three.
        }
    }
    writer.write(2, skip_some);
    var i: usize = skip_some;
    while (i < codes_to_store) : (i += 1) {
        const l: usize = code_length_bitdepth[k_storage_order[i]];
        writer.write(k_bit_lengths[l], k_symbols[l]);
    }
}

/// Port of `StoreHuffmanTreeToBitMask`.
fn storeHuffmanTreeToBitMask(
    huffman_tree_size: usize,
    huffman_tree: []const u8,
    huffman_tree_extra_bits: []const u8,
    code_length_bitdepth: *const [k_code_length_codes]u8,
    code_length_bitdepth_symbols: *const [k_code_length_codes]u16,
    writer: *BitWriter,
) !void {
    for (0..huffman_tree_size) |i| {
        const ix: usize = huffman_tree[i];
        if (ix > 17) return error.InvalidData;
        writer.write(code_length_bitdepth[ix], code_length_bitdepth_symbols[ix]);
        // Extra bits
        switch (ix) {
            16 => writer.write(2, huffman_tree_extra_bits[i]),
            17 => writer.write(3, huffman_tree_extra_bits[i]),
            else => {}, // no-op
        }
    }
}

/// Port of `StoreSimpleHuffmanTree`.
fn storeSimpleHuffmanTree(depths: []const u8, symbols0: [4]usize, num_symbols: usize, max_bits: usize, writer: *BitWriter) void {
    var symbols = symbols0;
    // value of 1 indicates a simple Huffman code
    writer.write(2, 1);
    writer.write(2, num_symbols - 1); // NSYM - 1

    // Sort
    for (0..num_symbols) |i| {
        for (i + 1..num_symbols) |j| {
            if (depths[symbols[j]] < depths[symbols[i]]) {
                std.mem.swap(usize, &symbols[j], &symbols[i]);
            }
        }
    }

    if (num_symbols == 2) {
        writer.write(max_bits, symbols[0]);
        writer.write(max_bits, symbols[1]);
    } else if (num_symbols == 3) {
        writer.write(max_bits, symbols[0]);
        writer.write(max_bits, symbols[1]);
        writer.write(max_bits, symbols[2]);
    } else {
        writer.write(max_bits, symbols[0]);
        writer.write(max_bits, symbols[1]);
        writer.write(max_bits, symbols[2]);
        writer.write(max_bits, symbols[3]);
        // tree-select
        writer.write(1, @intFromBool(depths[symbols[0]] == 1));
    }
}

/// Port of `StoreHuffmanTree`.
fn storeHuffmanTree(allocator: std.mem.Allocator, depths: []const u8, writer: *BitWriter) !void {
    const num = depths.len;
    const arena = try allocator.alloc(u8, 2 * num);
    defer allocator.free(arena);
    const huffman_tree = arena[0..num];
    const huffman_tree_extra_bits = arena[num..][0..num];
    var huffman_tree_size: usize = 0;
    writeHuffmanTree(depths, &huffman_tree_size, huffman_tree, huffman_tree_extra_bits);

    // Calculate the statistics of the Huffman tree in the compact
    // representation.
    var huffman_tree_histogram = [_]u32{0} ** k_code_length_codes;
    for (0..huffman_tree_size) |i| {
        huffman_tree_histogram[huffman_tree[i]] += 1;
    }

    var num_codes: usize = 0;
    var code: usize = 0;
    for (0..k_code_length_codes) |i| {
        if (huffman_tree_histogram[i] != 0) {
            if (num_codes == 0) {
                code = i;
                num_codes = 1;
            } else if (num_codes == 1) {
                num_codes = 2;
                break;
            }
        }
    }

    // Calculate another Huffman tree to use for compressing the earlier
    // Huffman tree with.
    var code_length_bitdepth = [_]u8{0} ** k_code_length_codes;
    var code_length_bitdepth_symbols = [_]u16{0} ** k_code_length_codes;
    createHuffmanTree(allocator, &huffman_tree_histogram, 5, &code_length_bitdepth);
    convertBitDepthsToSymbols(&code_length_bitdepth, &code_length_bitdepth_symbols);

    // Now, we have all the data, let's start storing it.
    storeHuffmanTreeOfHuffmanTreeToBitMask(num_codes, &code_length_bitdepth, writer);

    if (num_codes == 1) {
        code_length_bitdepth[code] = 0;
    }

    // Store the real Huffman tree now.
    try storeHuffmanTreeToBitMask(huffman_tree_size, huffman_tree, huffman_tree_extra_bits, &code_length_bitdepth, &code_length_bitdepth_symbols, writer);
}

/// Port of `jxl::BuildAndStoreHuffmanTree`.
pub fn buildAndStoreHuffmanTree(
    allocator: std.mem.Allocator,
    histogram: []const u32,
    depth: []u8,
    bits_out: []u16,
    writer: *BitWriter,
) !void {
    var count: usize = 0;
    var s4 = [4]usize{ 0, 0, 0, 0 };
    for (histogram, 0..) |h, i| {
        if (h != 0) {
            if (count < 4) {
                s4[count] = i;
            } else if (count > 4) {
                break;
            }
            count += 1;
        }
    }

    var max_bits_counter = histogram.len - 1;
    var max_bits: usize = 0;
    while (max_bits_counter != 0) {
        max_bits_counter >>= 1;
        max_bits += 1;
    }

    if (count <= 1) {
        // Output symbol bits and depths are initialized with 0.
        @memset(depth[0..histogram.len], 0);
        @memset(bits_out[0..histogram.len], 0);
        writer.write(4, 1);
        writer.write(max_bits, s4[0]);
        return;
    }

    @memset(depth[0..histogram.len], 0);
    createHuffmanTree(allocator, histogram, 15, depth);
    convertBitDepthsToSymbols(depth, bits_out);
    if (count <= 4) {
        storeSimpleHuffmanTree(depth, s4, count, max_bits, writer);
    } else {
        try storeHuffmanTree(allocator, depth, writer);
    }
}

// ---------------------------------------------------------------------------

test "createHuffmanTree simple" {
    // Two symbols with equal counts -> depths 1, 1.
    const histo = [_]u32{ 5, 5 };
    var depth = [_]u8{ 0, 0 };
    createHuffmanTree(std.testing.allocator, &histo, 15, &depth);
    try std.testing.expectEqualSlices(u8, &.{ 1, 1 }, &depth);
}

test "createHuffmanTree skewed" {
    // Fib-like counts: 1,1,2,3,5,8 -> depths at most 4ish.
    const histo = [_]u32{ 1, 1, 2, 3, 5, 8 };
    var depth = [_]u8{ 0, 0, 0, 0, 0, 0 };
    createHuffmanTree(std.testing.allocator, &histo, 15, &depth);
    for (depth) |d| {
        try std.testing.expect(d > 0 and d <= 15);
    }
    // Kraft equality for a complete code:
    var kraft: f64 = 0;
    for (depth) |d| kraft += std.math.pow(f64, 0.5, @floatFromInt(d));
    try std.testing.expectApproxEqAbs(@as(f64, 1.0), kraft, 1e-9);
}

test "convertBitDepthsToSymbols canonical" {
    const depths = [_]u8{ 1, 2, 3, 3 };
    var bits_out = [_]u16{ 0, 0, 0, 0 };
    convertBitDepthsToSymbols(&depths, &bits_out);
    // Canonical codes: 0, 10, 110, 111; bit-reversed for LSB-first writing.
    try std.testing.expectEqual(@as(u16, 0), bits_out[0]); // 0 -> rev 0
    try std.testing.expectEqual(@as(u16, 1), bits_out[1]); // 10 -> rev(01) = 10? no: rev2(10)=01=1
    // 110 (3 bits) reversed = 011 = 3; 111 reversed = 111 = 7.
    try std.testing.expectEqual(@as(u16, 3), bits_out[2]);
    try std.testing.expectEqual(@as(u16, 7), bits_out[3]);
}

test "buildAndStoreHuffmanTree roundtrip with decoder" {
    const alloc = std.testing.allocator;
    // Histogram over 8 symbols with skewed counts.
    const histo = [_]u32{ 40, 20, 10, 10, 5, 5, 5, 5 };
    var depth = [_]u8{0} ** 8;
    var bits_out = [_]u16{0} ** 8;
    var w = BitWriter.init(alloc);
    defer w.deinit();
    try buildAndStoreHuffmanTree(alloc, &histo, &depth, &bits_out, &w);
    w.zeroPadToByte();

    // Decode the code with the decoder and check symbol depths.
    const br = @import("../base/bit_io.zig").BitReader;
    var reader = br.init(w.getSpan());
    const dec_huffman = @import("dec_huffman.zig");
    var data = dec_huffman.HuffmanDecodingData{};
    defer data.deinit(alloc);
    try data.readFromBitStream(alloc, 8, &reader);

    // Encode a symbol sequence canonically and decode it back.
    const sequence = [_]usize{ 0, 0, 1, 2, 3, 4, 5, 6, 7, 0 };
    var w2 = BitWriter.init(alloc);
    defer w2.deinit();
    for (sequence) |sym| {
        w2.write(depth[sym], bits_out[sym]);
    }
    w2.zeroPadToByte();
    var reader2 = br.init(w2.getSpan());
    for (sequence) |sym| {
        const got = data.readSymbol(&reader2);
        try std.testing.expectEqual(@as(u16, @intCast(sym)), got);
    }
    try reader2.close();
    try reader.close();
}
