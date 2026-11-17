//! Интеграционные тесты: полный цикл «обучение дерева -> энтропийное
//! кодирование -> битстрим -> декодирование дерева -> декодирование канала»
//! и round-trip отдельных конфигураций (WP-only, offset+multiplier).
//!
//! Энтропийный слой: Brotli-совместимые prefix-коды из prefix_code.zig
//! (без ANS/LZ77 — они вне объёма порта).

const std = @import("std");
const testing = std.testing;

const common = @import("../base/common.zig");
const options = @import("options.zig");
const image_mod = @import("image.zig");
const context_predict = @import("context_predict.zig");
const ma = @import("ma.zig");
const ma_encode = @import("ma_encode.zig");
const hybrid_uint = @import("hybrid_uint.zig");
const prefix_code = @import("prefix_code.zig");
const symbol_reader = @import("symbol_reader.zig");
const bit_reader_mod = @import("../base/bit_io.zig");
const bit_writer_mod = @import("../base/bit_io.zig");
const rng_mod = @import("rng.zig");

const Allocator = std.mem.Allocator;
const BitReader = bit_reader_mod.BitReader;
const BitWriter = bit_writer_mod.BitWriter;
const pixel_type = common.pixel_type;
const pixel_type_w = common.pixel_type_w;
const Predictor = options.Predictor;

const W = 24;
const H = 16;

/// Детерминированное тестовое «изображение»: плавный градиент + шум.
fn makeTestChannel(gpa: Allocator) !image_mod.Channel {
    var ch = try image_mod.Channel.create(gpa, W, H, 0, 0);
    errdefer ch.deinit();
    var r = rng_mod.Rng.init(0xC0FFEE);
    for (0..H) |y| {
        const row = ch.row(y);
        for (0..W) |x| {
            const left: i64 = if (x > 0) row[x - 1] else if (y > 0) ch.row(y - 1)[x] else 128;
            const top: i64 = if (y > 0) ch.row(y - 1)[x] else left;
            var v: i64 = @divTrunc(left + top, 2);
            const noise: i64 = @as(i64, @intCast(r.uniformU(0, 7))) - 3;
            v += noise;
            row[x] = @intCast(std.math.clamp(v, 0, 255));
        }
    }
    return ch;
}

/// Символьный энкодер поверх prefix-кода: пишет (символ, доп. биты) для
/// гибридной uint-конфигурации.
const SymbolEncoder = struct {
    lengths: []u8, // длины кодов символов
    codes: []u32, // канонические коды

    fn init(gpa: Allocator, freqs: []const u32) !SymbolEncoder {
        const lengths = try gpa.alloc(u8, freqs.len);
        errdefer gpa.free(lengths);
        try prefix_code.huffmanCodeLengths(gpa, freqs, lengths);
        const codes = try gpa.alloc(u32, freqs.len);
        errdefer gpa.free(codes);
        prefix_code.canonicalCodes(lengths, codes);
        return .{ .lengths = lengths, .codes = codes };
    }

    fn deinit(self: *SymbolEncoder, gpa: Allocator) void {
        gpa.free(self.lengths);
        gpa.free(self.codes);
    }

    fn writeToken(self: *const SymbolEncoder, w: *BitWriter, value: u32, cfg: hybrid_uint.HybridUintConfig) !void {
        const enc = cfg.encode(value);
        w.write(self.lengths[enc.token], prefix_code.reverseBits(self.codes[enc.token], self.lengths[enc.token]));
        w.write(enc.nbits, enc.bits);
    }
};

/// Кодирует канал деревом (общий путь enc_encoding.cc): предсказание по
/// дереву (+WP), вычет, PackSigned(residual / multiplier).
fn encodeChannelWithTree(
    gpa: Allocator,
    ch: *const image_mod.Channel,
    img: *const image_mod.Image,
    chan_idx: usize,
    global_tree: []const ma.PropertyDecisionNode,
    wp_header: *const context_predict.weighted.Header,
    tokens: *std.ArrayList(ma_encode.Token),
) !void {
    const static_props = [context_predict.kNumStaticProperties]pixel_type{ @intCast(chan_idx), 0 };
    var num_props: usize = 0;
    var use_wp = false;
    var wp_only = false;
    var gradient_only = false;
    var flat_tree = context_predict.FlatTree.empty;
    defer flat_tree.deinit(gpa);
    try context_predict.filterTree(gpa, global_tree, &static_props, &num_props, &use_wp, &wp_only, &gradient_only, &flat_tree);
    const tree_lookup = context_predict.MATreeLookup.init(flat_tree.items);
    const properties = try gpa.alloc(pixel_type, num_props);
    defer gpa.free(properties);
    var references = try image_mod.Channel.create(gpa, num_props - context_predict.kNumNonrefProperties, ch.w, 0, 0);
    defer references.deinit();
    var wp_state = try context_predict.weighted.State.create(gpa, wp_header, ch.w, ch.h);
    defer wp_state.deinit(gpa);

    for (0..ch.h) |y| {
        const row = ch.row(y);
        context_predict.precomputeReferences(ch, y, img, chan_idx, &references);
        context_predict.initPropsRow(properties, &static_props, @intCast(y));
        for (0..ch.w) |x| {
            const res = if (use_wp)
                context_predict.predictTreeWP(properties, ch.w, &ch.plane, x, y, &tree_lookup, &references, &wp_state)
            else
                context_predict.predictTreeNoWP(properties, ch.w, &ch.plane, x, y, &tree_lookup, &references);
            const residual: pixel_type_w = row[x] - res.guess;
            try testing.expectEqual(@as(pixel_type_w, 0), @mod(residual, @as(pixel_type_w, res.multiplier)));
            const val = common.packSigned(@as(pixel_type, @truncate(@divTrunc(residual, @as(pixel_type_w, res.multiplier)))));
            try tokens.append(gpa, .{ .context = @intCast(res.context), .value = val });
            if (use_wp) {
                wp_state.updateErrors(row[x], x, y, ch.w);
            }
        }
    }
}

/// Декодирует канал деревом (общий путь encoding.cc): make_pixel(v) =
/// unpackSigned(v) * multiplier + guess.
fn decodeChannelWithTree(
    gpa: Allocator,
    ch: *image_mod.Channel,
    img: *const image_mod.Image,
    chan_idx: usize,
    global_tree: []const ma.PropertyDecisionNode,
    wp_header: *const context_predict.weighted.Header,
    reader: *const symbol_reader.PrefixSymbolReader,
    br: *BitReader,
) !void {
    const static_props = [context_predict.kNumStaticProperties]pixel_type{ @intCast(chan_idx), 0 };
    var num_props: usize = 0;
    var use_wp = false;
    var wp_only = false;
    var gradient_only = false;
    var flat_tree = context_predict.FlatTree.empty;
    defer flat_tree.deinit(gpa);
    try context_predict.filterTree(gpa, global_tree, &static_props, &num_props, &use_wp, &wp_only, &gradient_only, &flat_tree);
    const tree_lookup = context_predict.MATreeLookup.init(flat_tree.items);
    const properties = try gpa.alloc(pixel_type, num_props);
    defer gpa.free(properties);
    var references = try image_mod.Channel.create(gpa, num_props - context_predict.kNumNonrefProperties, ch.w, 0, 0);
    defer references.deinit();
    var wp_state = try context_predict.weighted.State.create(gpa, wp_header, ch.w, ch.h);
    defer wp_state.deinit(gpa);

    for (0..ch.h) |y| {
        const row = ch.row(y);
        context_predict.precomputeReferences(ch, y, img, chan_idx, &references);
        context_predict.initPropsRow(properties, &static_props, @intCast(y));
        for (0..ch.w) |x| {
            const res = if (use_wp)
                context_predict.predictTreeWP(properties, ch.w, &ch.plane, x, y, &tree_lookup, &references, &wp_state)
            else
                context_predict.predictTreeNoWP(properties, ch.w, &ch.plane, x, y, &tree_lookup, &references);
            const v = reader.readHybridUint(@intCast(res.context), br);
            const val: pixel_type_w = common.unpackSigned(v);
            const made: pixel_type = @truncate(val *% @as(pixel_type_w, res.multiplier) +% res.guess);
            row[x] = made;
            if (use_wp) {
                wp_state.updateErrors(row[x], x, y, ch.w);
            }
        }
    }
}

/// Собирает по токенам частоты uint-символов (после гибридного кодирования).
fn tokenSymbolFreqs(gpa: Allocator, tokens: []const ma_encode.Token, cfg: hybrid_uint.HybridUintConfig, num_contexts: usize) ![]u32 {
    // Один общий histogram (context_map тривиальный).
    _ = num_contexts;
    var max_sym: usize = 0;
    var encs = try gpa.alloc(hybrid_uint.Token, tokens.len);
    defer gpa.free(encs);
    for (tokens, 0..) |t, i| {
        encs[i] = cfg.encode(t.value);
        max_sym = @max(max_sym, encs[i].token);
    }
    const freqs = try gpa.alloc(u32, max_sym + 1);
    @memset(freqs, 0);
    for (encs) |e| freqs[e.token] += 1;
    return freqs;
}

test "round trip: обученное дерево (.variable) через битстрим" {
    const gpa = testing.allocator;

    var img = image_mod.Image.create(gpa);
    defer img.deinit();
    {
        const ch = try makeTestChannel(gpa);
        try img.channels.append(gpa, ch);
    }
    const ch = &img.channels.items[0];
    const wp_header = context_predict.weighted.Header{};

    // --- Обучение дерева ---
    var opts = options.ModularOptions{};
    var ts = ma_encode.TreeSamples.init(gpa);
    defer ts.deinit();
    try ts.setPredictor(.variable, opts.wp_tree_mode);
    try ts.setProperties(opts.splitting_heuristics_properties, opts.wp_tree_mode);

    var group_pixel_count: std.ArrayList(u32) = .empty;
    defer group_pixel_count.deinit(gpa);
    var channel_pixel_count: std.ArrayList(u32) = .empty;
    defer channel_pixel_count.deinit(gpa);
    var pixel_samples: std.ArrayList(pixel_type) = .empty;
    defer pixel_samples.deinit(gpa);
    var diff_samples: std.ArrayList(pixel_type) = .empty;
    defer diff_samples.deinit(gpa);
    try ma_encode.collectPixelSamples(gpa, &img, &opts, 0, &group_pixel_count, &channel_pixel_count, &pixel_samples, &diff_samples);
    try ts.preQuantizeProperties(
        .{ .{ 0, 1 }, .{ 0, 1 } },
        &.{},
        group_pixel_count.items,
        channel_pixel_count.items,
        pixel_samples.items,
        diff_samples.items,
        opts.max_property_values,
    );
    try ts.prepareForSamples(1024);

    var predictions: [options.kNumModularEncoderPredictors]pixel_type_w = .{0} ** options.kNumModularEncoderPredictors;
    const properties = try gpa.alloc(pixel_type, context_predict.kNumNonrefProperties);
    defer gpa.free(properties);
    var wp_learn = try context_predict.weighted.State.create(gpa, &wp_header, ch.w, ch.h);
    defer wp_learn.deinit(gpa);
    var references = try image_mod.Channel.create(gpa, 0, ch.w, 0, 0);
    defer references.deinit();
    for (0..ch.h) |y| {
        context_predict.initPropsRow(properties, &.{ 0, 0 }, @intCast(y));
        for (0..ch.w) |x| {
            context_predict.predictLearnAll(properties, ch.w, &ch.plane, x, y, &references, &wp_learn, &predictions);
            try ts.addSample(ch.row(y)[x], properties, &predictions);
            wp_learn.updateErrors(ch.row(y)[x], x, y, ch.w);
        }
    }
    ts.allSamplesDone();

    var learn_tree = ma.Tree.empty;
    defer learn_tree.deinit(gpa);
    try ma_encode.computeBestTree(gpa, &ts, opts.splitting_heuristics_node_threshold, &.{}, .{ .{ 0, 1 }, .{ 0, 1 } }, opts.fast_decode_multiplier, &learn_tree);
    try testing.expect(ma.validateTree(gpa, learn_tree.items));
    // Дерево обучилось: больше одного узла (иначе тест бессмысленный).
    try testing.expect(learn_tree.items.len > 1);

    // --- Токенизация дерева ---
    var tree_tokens: std.ArrayList(ma_encode.Token) = .empty;
    defer tree_tokens.deinit(gpa);
    var decoder_tree = ma.Tree.empty;
    defer decoder_tree.deinit(gpa);
    try ma_encode.tokenizeTree(gpa, learn_tree.items, &tree_tokens, &decoder_tree);

    // --- Кодирование канала ---
    var chan_tokens: std.ArrayList(ma_encode.Token) = .empty;
    defer chan_tokens.deinit(gpa);
    try encodeChannelWithTree(gpa, ch, &img, 0, learn_tree.items, &wp_header, &chan_tokens);
    try testing.expectEqual(@as(usize, W * H), chan_tokens.items.len);

    // --- Энтропийное кодирование ---
    const cfg = hybrid_uint.HybridUintConfig.init(4, 2, 0);
    var w = BitWriter.init(gpa);
    defer w.deinit();

    // Дерево: 6 контекстов -> один histogram.
    {
        const freqs = try tokenSymbolFreqs(gpa, tree_tokens.items, cfg, ma.kNumTreeContexts);
        defer gpa.free(freqs);
        var enc = try SymbolEncoder.init(gpa, freqs);
        defer enc.deinit(gpa);
        try prefix_code.writePrefixCodeHeader(gpa, &w, freqs.len, enc.lengths);
        for (tree_tokens.items) |t| {
            try enc.writeToken(&w, t.value, cfg);
        }
    }
    // Канал: контексты листьев -> один histogram.
    {
        const freqs = try tokenSymbolFreqs(gpa, chan_tokens.items, cfg, 16);
        defer gpa.free(freqs);
        var enc = try SymbolEncoder.init(gpa, freqs);
        defer enc.deinit(gpa);
        try prefix_code.writePrefixCodeHeader(gpa, &w, freqs.len, enc.lengths);
        for (chan_tokens.items) |t| {
            try enc.writeToken(&w, t.value, cfg);
        }
    }
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer gpa.free(bytes);

    // --- Декодирование ---
    var br = BitReader.init(bytes);
    var tree_codes = [_]symbol_reader.PrefixCode{.{ .uint_config = cfg }};
    const tree_alphabet = blk: {
        var mx: usize = 0;
        for (tree_tokens.items) |t| mx = @max(mx, cfg.encode(t.value).token);
        break :blk mx + 1;
    };
    try testing.expect(tree_codes[0].huffman.readFromBitStream(gpa, tree_alphabet, &br));
    var tree_ctx_map = [_]u8{0} ** ma.kNumTreeContexts;
    const tree_reader = symbol_reader.PrefixSymbolReader{ .codes = &tree_codes, .context_map = &tree_ctx_map };
    var decoded_tree = ma.Tree.empty;
    defer decoded_tree.deinit(gpa);
    try ma.decodeTree(&br, &tree_reader, &decoded_tree, gpa, 1 << 20);
    // Дерево совпадает с тем, что выдал TokenizeTree.
    try testing.expectEqual(decoder_tree.items.len, decoded_tree.items.len);
    for (decoder_tree.items, decoded_tree.items) |a, b| {
        try testing.expectEqual(a.property, b.property);
        try testing.expectEqual(a.splitval, b.splitval);
        try testing.expectEqual(a.predictor, b.predictor);
        try testing.expectEqual(a.predictor_offset, b.predictor_offset);
        try testing.expectEqual(a.multiplier, b.multiplier);
    }

    var chan_codes = [_]symbol_reader.PrefixCode{.{ .uint_config = cfg }};
    const chan_alphabet = blk: {
        var mx: usize = 0;
        for (chan_tokens.items) |t| mx = @max(mx, cfg.encode(t.value).token);
        break :blk mx + 1;
    };
    try testing.expect(chan_codes[0].huffman.readFromBitStream(gpa, chan_alphabet, &br));
    const chan_ctx_map = try gpa.alloc(u8, 16);
    defer gpa.free(chan_ctx_map);
    @memset(chan_ctx_map, 0);
    const chan_reader = symbol_reader.PrefixSymbolReader{ .codes = &chan_codes, .context_map = chan_ctx_map };

    var out_img = image_mod.Image.create(gpa);
    defer out_img.deinit();
    var out_ch = try image_mod.Channel.create(gpa, W, H, 0, 0);
    try out_img.channels.append(gpa, out_ch);
    try decodeChannelWithTree(gpa, &out_ch, &out_img, 0, decoded_tree.items, &wp_header, &chan_reader, &br);

    // Пиксели совпали.
    for (0..H) |y| {
        for (0..W) |x| {
            try testing.expectEqual(ch.row(y)[x], out_ch.row(y)[x]);
        }
    }
    try testing.expect(br.allReadsWithinBounds());
    chan_codes[0].deinit(gpa);
    tree_codes[0].deinit(gpa);
}

test "round trip: WP-only (одиночный лист Weighted)" {
    const gpa = testing.allocator;

    var ch = try makeTestChannel(gpa);
    defer ch.deinit();
    const wp_header = context_predict.weighted.Header{};
    const tree = [_]ma.PropertyDecisionNode{ma.PropertyDecisionNode.leaf(.weighted, 0, 1)};

    var tokens: std.ArrayList(ma_encode.Token) = .empty;
    defer tokens.deinit(gpa);
    // img с одним каналом (для интерфейса precomputeReferences).
    var img = image_mod.Image.create(gpa);
    defer img.deinit();
    var ch_copy = try image_mod.Channel.create(gpa, W, H, 0, 0);
    // Владение ch_copy передаётся img (deinit у img освободит плоскость).
    for (0..H) |y| @memcpy(ch_copy.row(y), ch.row(y));
    try img.channels.append(gpa, ch_copy);
    try encodeChannelWithTree(gpa, &ch_copy, &img, 0, &tree, &wp_header, &tokens);
    try testing.expectEqual(@as(usize, W * H), tokens.items.len);

    const cfg = hybrid_uint.HybridUintConfig.init(4, 2, 0);
    var w = BitWriter.init(gpa);
    defer w.deinit();
    const freqs = try tokenSymbolFreqs(gpa, tokens.items, cfg, 1);
    defer gpa.free(freqs);
    var enc = try SymbolEncoder.init(gpa, freqs);
    defer enc.deinit(gpa);
    try prefix_code.writePrefixCodeHeader(gpa, &w, freqs.len, enc.lengths);
    for (tokens.items) |t| {
        try enc.writeToken(&w, t.value, cfg);
    }
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer gpa.free(bytes);

    var br = BitReader.init(bytes);
    var codes = [_]symbol_reader.PrefixCode{.{ .uint_config = cfg }};
    defer codes[0].deinit(gpa);
    var mx: usize = 0;
    for (tokens.items) |t| mx = @max(mx, cfg.encode(t.value).token);
    try testing.expect(codes[0].huffman.readFromBitStream(gpa, mx + 1, &br));
    const ctx_map = [_]u8{0};
    const reader = symbol_reader.PrefixSymbolReader{ .codes = &codes, .context_map = &ctx_map };

    var out_img = image_mod.Image.create(gpa);
    defer out_img.deinit();
    var out_ch = try image_mod.Channel.create(gpa, W, H, 0, 0);
    try out_img.channels.append(gpa, out_ch);
    try decodeChannelWithTree(gpa, &out_ch, &out_img, 0, &tree, &wp_header, &reader, &br);
    for (0..H) |y| {
        for (0..W) |x| {
            try testing.expectEqual(ch.row(y)[x], out_ch.row(y)[x]);
        }
    }
    try testing.expect(br.allReadsWithinBounds());
}

test "round trip: offset и multiplier в листе дерева" {
    const gpa = testing.allocator;

    // Канал, где v = left + 10 + 2*k (k мал) — вычеты кратны 2.
    var ch = try image_mod.Channel.create(gpa, W, H, 0, 0);
    var r = rng_mod.Rng.init(7);
    for (0..H) |y| {
        const row = ch.row(y);
        for (0..W) |x| {
            const left: i64 = if (x > 0) row[x - 1] else 0;
            const k: i64 = @intCast(r.uniformU(0, 4));
            row[x] = @intCast(@mod(left + 10 + 2 * k, 256));
        }
    }
    const wp_header = context_predict.weighted.Header{};
    // Дерево: лист Left, offset 10, multiplier 2. (Лист для декодера:
    // property=-1, lchild=0 — id контекста.)
    const tree = [_]ma.PropertyDecisionNode{ma.PropertyDecisionNode.leaf(.left, 10, 2)};

    var tokens: std.ArrayList(ma_encode.Token) = .empty;
    defer tokens.deinit(gpa);
    var img = image_mod.Image.create(gpa);
    defer img.deinit();
    try img.channels.append(gpa, ch);
    try encodeChannelWithTree(gpa, &ch, &img, 0, &tree, &wp_header, &tokens);

    const cfg = hybrid_uint.HybridUintConfig.init(4, 2, 0);
    var w = BitWriter.init(gpa);
    defer w.deinit();
    const freqs = try tokenSymbolFreqs(gpa, tokens.items, cfg, 1);
    defer gpa.free(freqs);
    var enc = try SymbolEncoder.init(gpa, freqs);
    defer enc.deinit(gpa);
    try prefix_code.writePrefixCodeHeader(gpa, &w, freqs.len, enc.lengths);
    for (tokens.items) |t| {
        try enc.writeToken(&w, t.value, cfg);
    }
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer gpa.free(bytes);

    // Дерево кодируем mock-ридером (в этом тесте проверяется только канал).
    var br = BitReader.init(bytes);
    var codes = [_]symbol_reader.PrefixCode{.{ .uint_config = cfg }};
    defer codes[0].deinit(gpa);
    var mx: usize = 0;
    for (tokens.items) |t| mx = @max(mx, cfg.encode(t.value).token);
    try testing.expect(codes[0].huffman.readFromBitStream(gpa, mx + 1, &br));
    const ctx_map = [_]u8{0};
    const reader = symbol_reader.PrefixSymbolReader{ .codes = &codes, .context_map = &ctx_map };

    var out_img = image_mod.Image.create(gpa);
    defer out_img.deinit();
    var out_ch = try image_mod.Channel.create(gpa, W, H, 0, 0);
    try out_img.channels.append(gpa, out_ch);
    try decodeChannelWithTree(gpa, &out_ch, &out_img, 0, &tree, &wp_header, &reader, &br);
    for (0..H) |y| {
        for (0..W) |x| {
            try testing.expectEqual(ch.row(y)[x], out_ch.row(y)[x]);
        }
    }
    try testing.expect(br.allReadsWithinBounds());
}

test "NEC-путь совпадает с общим путём предсказания" {
    const gpa = testing.allocator;
    var ch = try makeTestChannel(gpa);
    defer ch.deinit();
    // Дерево из трёх узлов: расщепление по свойству 9 (локальный градиент).
    const tree = [_]ma.PropertyDecisionNode{
        ma.PropertyDecisionNode.split(9, 2, 1, 3),
        ma.PropertyDecisionNode.leaf(.gradient, 0, 1),
        ma.PropertyDecisionNode.leaf(.gradient, 0, 1),
        ma.PropertyDecisionNode.leaf(.gradient, 0, 1),
    };
    var flat_tree = context_predict.FlatTree.empty;
    defer flat_tree.deinit(gpa);
    var num_props: usize = 0;
    var use_wp = false;
    var wp_only = false;
    var gradient_only = false;
    try context_predict.filterTree(gpa, &tree, &.{ 0, 0 }, &num_props, &use_wp, &wp_only, &gradient_only, &flat_tree);
    try testing.expect(!use_wp);
    const tree_lookup = context_predict.MATreeLookup.init(flat_tree.items);
    const properties = try gpa.alloc(pixel_type, num_props);
    defer gpa.free(properties);
    var references = try image_mod.Channel.create(gpa, 0, ch.w, 0, 0);
    defer references.deinit();

    for (2..ch.h) |y| {
        context_predict.initPropsRow(properties, &.{ 0, 0 }, @intCast(y));
        for (2..ch.w - 2) |x| {
            const r_general = context_predict.predictTreeNoWP(properties, ch.w, &ch.plane, x, y, &tree_lookup, &references);
            context_predict.initPropsRow(properties, &.{ 0, 0 }, @intCast(y));
            const r_nec = context_predict.predictTreeNoWPNEC(properties, ch.w, &ch.plane, x, y, &tree_lookup, &references);
            try testing.expectEqual(r_general.context, r_nec.context);
            try testing.expectEqual(r_general.guess, r_nec.guess);
            try testing.expectEqual(r_general.predictor, r_nec.predictor);
            try testing.expectEqual(r_general.multiplier, r_nec.multiplier);
        }
    }
}
