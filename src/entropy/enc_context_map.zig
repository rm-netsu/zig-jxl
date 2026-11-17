//! Context map encoding. Port of libjxl `enc_context_map.h/cc`
//! (MoveToFrontTransform, EncodeContextMap).

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const bits = @import("../base/bits.zig");
const fields = @import("../base/fields.zig");
const enc_ans = @import("enc_ans.zig");
const ans_common = @import("ans_common.zig");

const BitWriter = bit_io.BitWriter;
const Token = enc_ans.Token;
const HistogramParams = enc_ans.HistogramParams;

fn indexOf(v: []const u8, value: u8) usize {
    var i: usize = 0;
    while (i < v.len) : (i += 1) {
        if (v[i] == value) return i;
    }
    return i;
}

fn moveToFront(v: []u8, index: usize) void {
    const value = v[index];
    var i = index;
    while (i != 0) : (i -= 1) {
        v[i] = v[i - 1];
    }
    v[0] = value;
}

fn moveToFrontTransform(allocator: std.mem.Allocator, v: []const u8) ![]u8 {
    if (v.len == 0) return allocator.alloc(u8, 0);
    var max_value: u8 = 0;
    for (v) |x| max_value = @max(max_value, x);
    const mtf = try allocator.alloc(u8, max_value + 1);
    defer allocator.free(mtf);
    for (0..max_value + 1) |i| mtf[i] = @intCast(i);
    const result = try allocator.alloc(u8, v.len);
    errdefer allocator.free(result);
    for (v, 0..) |x, i| {
        const index = indexOf(mtf, x);
        std.debug.assert(index < mtf.len);
        result[i] = @intCast(index);
        moveToFront(mtf, index);
    }
    return result;
}

/// Port of `jxl::EncodeContextMap`.
pub fn encodeContextMap(
    allocator: std.mem.Allocator,
    context_map: []const u8,
    num_histograms: usize,
    writer: *BitWriter,
) ans_common.EncodeError!void {
    if (num_histograms == 1) {
        // Simple code
        writer.write(1, 1);
        // 0 bits per entry.
        writer.write(2, 0);
        return;
    }

    const transformed_symbols = try moveToFrontTransform(allocator, context_map);
    defer allocator.free(transformed_symbols);

    var tokens: [1]std.ArrayList(Token) = .{.empty};
    var mtf_tokens: [1]std.ArrayList(Token) = .{.empty};
    defer {
        for (&tokens) |*t| t.deinit(allocator);
        for (&mtf_tokens) |*t| t.deinit(allocator);
    }
    for (context_map) |ctx| {
        try tokens[0].append(allocator, Token.init(0, ctx));
    }
    for (transformed_symbols) |sym| {
        try mtf_tokens[0].append(allocator, Token.init(0, sym));
    }
    var params = HistogramParams{};
    params.uint_method = .kContextMap;
    var ans_codes = enc_ans.EntropyEncodingData.init(allocator);
    defer ans_codes.deinit();
    const ans_cost = try enc_ans.buildAndEncodeHistograms(allocator, &params, 1, &tokens, &ans_codes, null);
    var mtf_codes = enc_ans.EntropyEncodingData.init(allocator);
    defer mtf_codes.deinit();
    const mtf_cost = try enc_ans.buildAndEncodeHistograms(allocator, &params, 1, &mtf_tokens, &mtf_codes, null);
    const use_mtf = mtf_cost < ans_cost;
    // Rebuild token list.
    tokens[0].clearRetainingCapacity();
    for (0..transformed_symbols.len) |i| {
        try tokens[0].append(allocator, Token.init(0, if (use_mtf) transformed_symbols[i] else context_map[i]));
    }
    const entry_bits = bits.ceilLog2Nonzero(num_histograms);
    const simple_cost = entry_bits * context_map.len;
    if (entry_bits < 4 and simple_cost < ans_cost and simple_cost < mtf_cost) {
        writer.write(1, 1);
        writer.write(2, entry_bits);
        for (context_map) |entry| {
            writer.write(entry_bits, entry);
        }
    } else {
        writer.write(1, 0);
        writer.write(1, @intFromBool(use_mtf)); // Use/don't use MTF.
        var codes = enc_ans.EntropyEncodingData.init(allocator);
        defer codes.deinit();
        _ = try enc_ans.buildAndEncodeHistograms(allocator, &params, 1, &tokens, &codes, writer);
        _ = try enc_ans.writeTokens(allocator, tokens[0].items, &codes, 0, writer);
    }
}

// ---------------------------------------------------------------------------

test "moveToFrontTransform" {
    const alloc = std.testing.allocator;
    const v = [_]u8{ 1, 1, 0, 2, 1 };
    const r = try moveToFrontTransform(alloc, &v);
    defer alloc.free(r);
    // mtf=[0,1,2]: 1->1, mtf=[1,0,2]; 1->0 (no move);
    // 0->1, mtf=[0,1,2]; 2->2, mtf=[2,0,1]; 1->2, mtf=[1,2,0].
    try std.testing.expectEqualSlices(u8, &.{ 1, 0, 1, 2, 2 }, r);
}
