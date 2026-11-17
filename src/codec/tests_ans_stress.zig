//! ANS stack stress test: random token streams with many contexts and
//! kFast clustering, exercising buildAndEncodeHistograms/writeTokens
//! against decodeHistograms/ANSSymbolReader.

const std = @import("std");
const ans_common = @import("../entropy/ans_common.zig");
const enc_ans = @import("../entropy/enc_ans.zig");
const dec_ans = @import("../entropy/dec_ans.zig");
const bit_io = @import("../base/bit_io.zig");

const Allocator = std.mem.Allocator;
const BitWriter = bit_io.BitWriter;
const BitReader = bit_io.BitReader;
const ANSToken = enc_ans.Token;

pub fn stressRoundtrip(allocator: Allocator, num_contexts: usize, num_tokens: usize, seed: u64, params: *const enc_ans.HistogramParams) !void {
    var prng = std.Random.DefaultPrng.init(seed);
    const rand = prng.random();

    var tokens: std.ArrayList(std.ArrayList(ANSToken)) = .empty;
    defer {
        for (tokens.items) |*list| list.deinit(allocator);
        tokens.deinit(allocator);
    }
    const num_streams: usize = 3;
    try tokens.resize(allocator, num_streams);
    for (tokens.items) |*list| list.* = .empty;
    for (0..num_streams) |si| {
        const count = if (si == 0) num_tokens else num_tokens / 2;
        for (0..count) |_| {
            const ctx = rand.uintLessThan(u32, @intCast(num_contexts));
            // Skewed value distribution: mostly small, sometimes large.
            const value: u32 = if (rand.uintLessThan(u8, 4) == 0)
                rand.uintLessThan(u32, @as(u32, 1) << @intCast(rand.uintLessThan(u5, 17)))
            else
                rand.uintLessThan(u32, 32);
            try tokens.items[si].append(allocator, .{ .context = ctx, .value = value });
        }
    }

    var codes = enc_ans.EntropyEncodingData.init(allocator);
    defer codes.deinit();
    var writer = BitWriter.init(allocator);
    defer writer.deinit();

    _ = try enc_ans.buildAndEncodeHistograms(allocator, params, num_contexts, tokens.items, &codes, &writer);
    var stream_writers: [3]BitWriter = .{ BitWriter.init(allocator), BitWriter.init(allocator), BitWriter.init(allocator) };
    defer for (&stream_writers) |*sw| sw.deinit();
    for (tokens.items, 0..) |*list, si| {
        _ = try enc_ans.writeTokens(allocator, list.items, &codes, 0, &stream_writers[si]);
        stream_writers[si].zeroPadToByte();
    }
    writer.zeroPadToByte();
    const bytes = try writer.takeBytes();
    defer allocator.free(bytes);

    var br = BitReader.init(bytes);
    var code = dec_ans.ANSCode{};
    defer code.deinit(allocator);
    var context_map: std.ArrayList(u8) = .empty;
    defer context_map.deinit(allocator);
    try dec_ans.decodeHistograms(allocator, &br, num_contexts, &code, &context_map, true);

    for (tokens.items, 0..) |*list, si| {
        const stream_bytes = try stream_writers[si].takeBytes();
        defer allocator.free(stream_bytes);
        var sbr = BitReader.init(stream_bytes);
        var reader = try dec_ans.ANSSymbolReader.create(allocator, &code, &sbr, 0);
        defer reader.deinit();
        for (list.items) |expected| {
            const got = reader.readHybridUint(expected.context, &sbr, context_map.items);
            if (got != expected.value) {
                std.debug.print("mismatch: stream {d} ctx={d} expected={d} got={d}\n", .{ si, expected.context, expected.value, got });
                return error.ANSMismatch;
            }
        }
        if (!reader.checkANSFinalState()) return error.ANSFinalState;
    }
}

test "ans stress: 59 contexts kFast" {
    const alloc = std.testing.allocator;
    const params = enc_ans.HistogramParams{
        .clustering = .kFast,
        .uint_method = .kFast,
        .lz77_method = .kNone,
        .ans_histogram_strategy = .kFast,
    };
    try stressRoundtrip(alloc, 59, 20000, 0x5EED, &params);
}

test "ans stress: 15 contexts kFast" {
    const alloc = std.testing.allocator;
    const params = enc_ans.HistogramParams{
        .clustering = .kFast,
        .uint_method = .kFast,
        .lz77_method = .kNone,
        .ans_histogram_strategy = .kFast,
    };
    try stressRoundtrip(alloc, 15, 20000, 0x1234, &params);
}

test "ans stress: 9 clusters worth, wide values" {
    const alloc = std.testing.allocator;
    const params = enc_ans.HistogramParams{
        .clustering = .kFast,
        .uint_method = .kFast,
        .lz77_method = .kNone,
        .ans_histogram_strategy = .kFast,
    };
    try stressRoundtrip(alloc, 59, 25600, 99, &params);
}

test "ans stress: 75 contexts kFast" {
    const alloc = std.testing.allocator;
    const params = enc_ans.HistogramParams{
        .clustering = .kFast,
        .uint_method = .kFast,
        .lz77_method = .kNone,
        .ans_histogram_strategy = .kFast,
    };
    try stressRoundtrip(alloc, 75, 66000, 0xC0FFEE, &params);
}

test "ans stress: 128 contexts kFast" {
    const alloc = std.testing.allocator;
    const params = enc_ans.HistogramParams{
        .clustering = .kFast,
        .uint_method = .kFast,
        .lz77_method = .kNone,
        .ans_histogram_strategy = .kFast,
    };
    try stressRoundtrip(alloc, 128, 40000, 7, &params);
}

test "ans stress: 200 contexts kFast" {
    const alloc = std.testing.allocator;
    const params = enc_ans.HistogramParams{
        .clustering = .kFast,
        .uint_method = .kFast,
        .lz77_method = .kNone,
        .ans_histogram_strategy = .kFast,
    };
    try stressRoundtrip(alloc, 200, 40000, 12345, &params);
}
