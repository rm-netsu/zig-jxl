//! Regression tests for the group-local palette transform (the
//! `PrepareStreamParams` pass of the encoder): the stream image is
//! transformed with a palette, tokenized with a trained tree, entropy-coded
//! with kRLE LZ77 (exactly like a real group stream), and decoded back
//! through `decodeStream`.
//!
//! These shapes guard two encoder/decoder asymmetries that were found by
//! cross-validation against libjxl 0.12:
//!   * the implicit palette cube lookup (`% kLargeCube`, not `kLargeCube-1`);
//!   * the per-stream LZ77 distance multipliers (`image_widths`), without
//!     which the RLE distance symbol 0 resolves to "one row up" instead
//!     of the previous symbol.

const std = @import("std");
const enc_transforms = @import("enc_transforms.zig");
const transforms = @import("transforms.zig");
const stream = @import("stream.zig");
const image_mod = @import("image.zig");
const options_mod = @import("options.zig");
const context_predict = @import("context_predict.zig");
const ma_encode = @import("ma_encode.zig");
const enc_ans = @import("../entropy/enc_ans.zig");
const dec_ans = @import("../entropy/dec_ans.zig");
const ma = @import("ma.zig");

const bit_io = @import("../base/bit_io.zig");
const BitWriter = bit_io.BitWriter;
const BitReader = bit_io.BitReader;

/// `w` x `h` RGB image mimicking the group-edge shapes that failed: R and B
/// depend on position, G is constant, so a palette with 40-130 colors and
/// long RLE runs in the palette channel results.
fn buildTestImage(allocator: std.mem.Allocator, comptime shape: []const usize) !image_mod.Image {
    var img = image_mod.Image.create(allocator);
    const w = shape[0];
    const h = shape[1];
    for (0..3) |_| _ = try img.addChannel(w, h);
    for (0..h) |y| {
        for (0..w) |x| {
            img.channels.items[0].row(y)[x] = @intCast((x * 255) / 512);
            img.channels.items[1].row(y)[x] = 255;
            img.channels.items[2].row(y)[x] = @intCast(((x + y + 256) * 255) / 768);
        }
    }
    return img;
}

fn roundtripGroupImage(allocator: std.mem.Allocator, comptime shape: []const usize, comptime use_palette: bool, comptime use_rle: bool) !void {
    var img = try buildTestImage(allocator, shape);
    defer img.deinit();

    // Originals for comparison.
    var orig = image_mod.Image.create(allocator);
    defer orig.deinit();
    for (img.channels.items) |*ch| {
        var oc = try image_mod.Channel.create(allocator, ch.w, ch.h, ch.hshift, ch.vshift);
        for (0..ch.h) |y| @memcpy(oc.row(y), ch.row(y));
        try orig.channels.append(allocator, oc);
    }

    // The encoder's local transform pass.
    var tlist: std.ArrayList(transforms.Transform) = .empty;
    defer {
        for (tlist.items) |*t| t.deinit();
        tlist.deinit(allocator);
    }
    if (use_palette) {
        try enc_transforms.tryPalettes(allocator, &img, .{}, .{ .bit_depth = 8 }, &tlist);
    }

    // Train a tree on this stream, tokenize, histogram-encode with the
    // per-stream distance multiplier, then decode through decodeStream.
    var opts = options_mod.ModularOptions{};
    var props_buf: [7]u32 = .{ 0, 15, 9, 10, 11, 12, 13 };
    opts.splitting_heuristics_properties = props_buf[0..7];
    opts.max_property_values = 96;
    opts.max_chan_size = 128;

    const train = [_]stream.TrainStream{.{ .image = &img, .group_id = 0 }};
    var tree = try stream.trainTree(allocator, &train, &opts);
    defer tree.deinit(allocator);
    var decoded_tree = ma.Tree.empty;
    var tree_tokens: std.ArrayList(ma_encode.Token) = .empty;
    defer tree_tokens.deinit(allocator);
    try stream.tokenizeTree(allocator, tree.items, &tree_tokens, &decoded_tree);
    tree.deinit(allocator);
    tree = decoded_tree;

    const wp = context_predict.weighted.Header.defaults;
    var tokens: std.ArrayList(ma_encode.Token) = .empty;
    defer tokens.deinit(allocator);
    try stream.encodeStreamTokens(allocator, &img, 0, 128, tree.items, &wp, &tokens);

    // Serialize like a real group stream with the global tree: tree
    // histograms + tree tokens + stream histograms + GroupHeader (with the
    // transform list) + stream tokens.
    var wbuf = BitWriter.init(allocator);
    defer wbuf.deinit();

    var tree_ans: std.ArrayList(enc_ans.Token) = .empty;
    defer tree_ans.deinit(allocator);
    try tree_ans.ensureTotalCapacity(allocator, tree_tokens.items.len);
    for (tree_tokens.items) |t| {
        tree_ans.appendAssumeCapacity(.{ .is_lz77_length = t.is_lz77_length, .context = t.context, .value = t.value });
    }
    var tree_streams = [_]std.ArrayList(enc_ans.Token){tree_ans};
    var tree_code = enc_ans.EntropyEncodingData.init(allocator);
    defer tree_code.deinit();
    var params = enc_ans.HistogramParams{
        .clustering = .kFast,
        .uint_method = .kFast,
        .lz77_method = if (use_rle) .kRLE else .kNone,
        .ans_histogram_strategy = .kApproximate,
    };
    _ = try stream.buildHistograms(allocator, &params, ma.kNumTreeContexts, tree_streams[0..], &tree_code, &wbuf);
    _ = try stream.writeTokens(allocator, tree_ans.items, &tree_code, 0, &wbuf);

    const stream_widths = [_]usize{stream.maxStreamChannelWidth(&img, 128)};
    var all_tokens: std.ArrayList(std.ArrayList(enc_ans.Token)) = .empty;
    defer {
        for (all_tokens.items) |*l| l.deinit(allocator);
        all_tokens.deinit(allocator);
    }
    {
        // Ownership of this list moves into `all_tokens`.
        var stream_ans: std.ArrayList(enc_ans.Token) = .empty;
        errdefer stream_ans.deinit(allocator);
        try stream_ans.ensureTotalCapacity(allocator, tokens.items.len);
        for (tokens.items) |t| {
            stream_ans.appendAssumeCapacity(.{ .is_lz77_length = t.is_lz77_length, .context = t.context, .value = t.value });
        }
        try all_tokens.append(allocator, stream_ans);
    }
    var stream_code = enc_ans.EntropyEncodingData.init(allocator);
    defer stream_code.deinit();
    var stream_params = params;
    stream_params.image_widths = &stream_widths;
    _ = try stream.buildHistograms(allocator, &stream_params, (tree.items.len + 1) / 2, all_tokens.items, &stream_code, &wbuf);

    var gh = stream.GroupHeader.init(allocator);
    defer gh.deinit();
    gh.use_global_tree = true;
    for (tlist.items) |*t| try gh.transforms.append(allocator, t.*);
    try gh.write(&wbuf);
    _ = try stream.writeTokens(allocator, all_tokens.items[0].items, &stream_code, 0, &wbuf);
    wbuf.zeroPadToByte();
    const bytes = try wbuf.takeBytes();
    defer allocator.free(bytes);

    // Decode like the frame decoder does.
    var br = BitReader.init(bytes);
    var ec = stream.EntropyCode.init(allocator);
    defer ec.deinit();
    var tree_ec = stream.EntropyCode.init(allocator);
    defer tree_ec.deinit();
    try dec_ans.decodeHistograms(allocator, &br, ma.kNumTreeContexts, &tree_ec.code, &tree_ec.context_map, false);
    var tree_reader = try stream.AnsMaReader.create(allocator, &tree_ec.code, tree_ec.context_map.items, 0, &br);
    defer tree_reader.ans.deinit();
    try ma.decodeTree(&br, &tree_reader, &ec.tree, allocator, 1 << 20);
    try dec_ans.decodeHistograms(allocator, &br, ec.numContexts(), &ec.code, &ec.context_map, false);

    var dec_img = try buildTestImage(allocator, shape);
    defer dec_img.deinit();
    var dec_transforms: std.ArrayList(transforms.Transform) = .empty;
    defer {
        for (dec_transforms.items) |*t| t.deinit();
        dec_transforms.deinit(allocator);
    }
    try stream.decodeStream(.{
        .allocator = allocator,
        .br = &br,
        .image = &dec_img,
        .group_id = 0,
        .max_chan_size = 128,
        .global = &ec,
        .undo_transforms = false,
        .transforms_out = &dec_transforms,
        .bitdepth = 8,
    });
    try br.close();
    var ti: usize = dec_transforms.items.len;
    while (ti > 0) {
        ti -= 1;
        try transforms.inverse(&dec_img, &dec_transforms.items[ti], &context_predict.weighted.Header.defaults, 8);
    }

    // Compare.
    try std.testing.expectEqual(orig.channels.items.len, dec_img.channels.items.len);
    for (orig.channels.items, dec_img.channels.items) |*a, *b| {
        try std.testing.expectEqual(a.w, b.w);
        try std.testing.expectEqual(a.h, b.h);
        for (0..a.h) |y| {
            for (0..a.w) |x| {
                try std.testing.expectEqual(a.row(y)[x], b.row(y)[x]);
            }
        }
    }
}

test "group palette roundtrip: 128x1 edge row (implicit cube colors)" {
    const alloc = std.testing.allocator;
    try roundtripGroupImage(alloc, &.{ 128, 1 }, true, true);
}

test "group palette roundtrip: 1x128 edge column" {
    const alloc = std.testing.allocator;
    try roundtripGroupImage(alloc, &.{ 1, 128 }, true, true);
}

test "group palette roundtrip: plain 64x48" {
    const alloc = std.testing.allocator;
    try roundtripGroupImage(alloc, &.{ 64, 48 }, true, true);
}

test "group stream roundtrip without palette (RLE across channels)" {
    const alloc = std.testing.allocator;
    try roundtripGroupImage(alloc, &.{ 1, 128 }, false, true);
}

test "group stream roundtrip without RLE (palette only)" {
    const alloc = std.testing.allocator;
    try roundtripGroupImage(alloc, &.{ 1, 128 }, true, false);
}

test "group stream roundtrip with neither" {
    const alloc = std.testing.allocator;
    try roundtripGroupImage(alloc, &.{ 1, 128 }, false, false);
}
