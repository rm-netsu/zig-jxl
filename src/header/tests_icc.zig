//! ICC codec tests: varint/shuffle roundtrips, predict/unpredict over
//! synthetic profiles, random inputs and a real profile extracted from the
//! JPEG test corpus, the entropy-coded stream roundtrip (writeICC <->
//! readICC) and full codestream encode/decode with an embedded profile.

const std = @import("std");
const icc = @import("icc.zig");
const enc_icc = @import("enc_icc.zig");
const bit_io = @import("../base/bit_io.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;

// ---------------------------------------------------------------------------
// Synthetic profile builder
// ---------------------------------------------------------------------------

const TagSpec = struct {
    keyword: [4]u8,
    data: []const u8,
};

/// Assembles a profile-shaped byte string: 128-byte header (with a valid
/// size, "acsp" signature and an "RGB " data colour space so it passes
/// `looksLikeValidProfile`), a tag table of `tags` entries and the tag
/// payloads in order.
fn buildProfile(allocator: std.mem.Allocator, header_fill: []const u8, tags: []const TagSpec) ![]u8 {
    var out: std.ArrayList(u8) = .empty;
    errdefer out.deinit(allocator);

    var header: [128]u8 = undefined;
    const base = icc.iccInitialHeaderPrediction(0);
    @memcpy(&header, &base);
    if (header_fill.len > 0) {
        const n = @min(header_fill.len, 128);
        @memcpy(header[0..n], header_fill[0..n]);
    }
    // Data colour space + "acsp" signature for a header-valid profile.
    header[16..20].* = .{ 'R', 'G', 'B', ' ' };
    header[36..40].* = .{ 'a', 'c', 's', 'p' };
    try out.appendSlice(allocator, &header);

    // Tag count.
    var count: [4]u8 = undefined;
    std.mem.writeInt(u32, &count, @intCast(tags.len), .big);
    try out.appendSlice(allocator, &count);

    // Tag table.
    const table_end: u32 = @intCast(128 + 4 + 12 * tags.len);
    var offset: u32 = table_end;
    for (tags) |t| {
        try out.appendSlice(allocator, &t.keyword);
        var be: [4]u8 = undefined;
        std.mem.writeInt(u32, &be, offset, .big);
        try out.appendSlice(allocator, &be);
        std.mem.writeInt(u32, &be, @intCast(t.data.len), .big);
        try out.appendSlice(allocator, &be);
        offset += @intCast(t.data.len);
    }
    for (tags) |t| {
        try out.appendSlice(allocator, t.data);
    }

    // Fix the total size in the header.
    std.mem.writeInt(u32, out.items[0..4], @intCast(out.items.len), .big);
    return out.toOwnedSlice(allocator);
}

/// "XYZ " type payload: keyword + 4 zero bytes + 12 data bytes (20 total).
fn xyzPayload(data: [12]u8) [20]u8 {
    var out: [20]u8 = @splat(0);
    out[0..4].* = .{ 'X', 'Y', 'Z', ' ' };
    out[8..].* = data;
    return out;
}

/// "curv" type payload: keyword + count + big-endian u16 curve samples.
fn curvPayload(comptime n: usize, samples: [n]u16) [n * 2 + 8]u8 {
    var out: [n * 2 + 8]u8 = @splat(0);
    out[0..4].* = .{ 'c', 'u', 'r', 'v' };
    for (samples, 0..) |s, i| {
        out[8 + i * 2] = @truncate(s >> 8);
        out[8 + i * 2 + 1] = @truncate(s);
    }
    return out;
}

// ---------------------------------------------------------------------------
// Basic roundtrips
// ---------------------------------------------------------------------------

test "shuffle inverts unshuffle" {
    const alloc = std.testing.allocator;
    const sizes = [_]usize{ 1, 2, 3, 7, 16, 17, 63, 64, 65, 100 };
    const widths = [_]usize{ 2, 4 };
    var prng = std.Random.DefaultPrng.init(1234);
    const rand = prng.random();
    for (sizes) |n| {
        for (widths) |w| {
            const buf = try alloc.alloc(u8, n);
            defer alloc.free(buf);
            rand.bytes(buf);
            const orig = try alloc.dupe(u8, buf);
            defer alloc.free(orig);
            // shuffle then unshuffle restores the input.
            try icc.shuffle(alloc, buf, w);
            try enc_icc.unshuffleForTest(alloc, buf, w);
            try std.testing.expectEqualSlices(u8, orig, buf);
        }
    }
}

test "varint roundtrip" {
    const alloc = std.testing.allocator;
    const values = [_]u64{ 0, 1, 127, 128, 16383, 16384, 65535, 1 << 20, (1 << 31) - 1, 1 << 32, std.math.maxInt(u64) };
    for (values) |v| {
        var list: std.ArrayList(u8) = .empty;
        defer list.deinit(alloc);
        try enc_icc.encodeVarIntForTest(v, &list, alloc);
        var pos: usize = 0;
        const got = try icc.decodeVarInt(list.items, &pos);
        try std.testing.expectEqual(v, got);
        try std.testing.expectEqual(list.items.len, pos);
    }
}

// ---------------------------------------------------------------------------
// predict / unpredict roundtrips
// ---------------------------------------------------------------------------

fn expectPredictRoundtrip(alloc: std.mem.Allocator, profile: []const u8) !void {
    var enc: std.ArrayList(u8) = .empty;
    defer enc.deinit(alloc);
    try enc_icc.predictICC(alloc, profile, &enc);
    const dec = try icc.unpredictICC(alloc, enc.items);
    defer alloc.free(dec);
    try std.testing.expectEqualSlices(u8, profile, dec);
}

test "predict roundtrip: tiny and header-only inputs" {
    const alloc = std.testing.allocator;
    try expectPredictRoundtrip(alloc, &.{});
    try expectPredictRoundtrip(alloc, &.{1});
    try expectPredictRoundtrip(alloc, "short but longer than one byte");
    // Exactly 128 bytes (header-only path).
    const hdr = try alloc.alloc(u8, 128);
    defer alloc.free(hdr);
    for (hdr, 0..) |*b, i| b.* = @truncate(i *% 7 +% (i >> 2));
    try expectPredictRoundtrip(alloc, hdr);
}

test "predict roundtrip: minimal rgb profile with xyz and trc" {
    const alloc = std.testing.allocator;
    // rXYZ/gXYZ/bXYZ as consecutive equal-size XYZ payloads (the encoder
    // merges them into a single kCommandTagXYZ), plus a rTRC triple.
    const xyz_r = xyzPayload(.{ 0, 0, 0, 0, 'r', 1, 2, 3, 4, 5, 6, 7 });
    const xyz_g = xyzPayload(.{ 0, 1, 2, 3, 'g', 4, 5, 6, 7, 8, 9, 10 });
    const xyz_b = xyzPayload(.{ 9, 8, 7, 6, 'b', 5, 4, 3, 2, 1, 0, 255 });
    const xyz_w = xyzPayload(.{ 0, 0, 0xf6, 0xd6, 0, 1, 0, 0, 0, 0, 0xd3, 0x2d });
    var samples: [17]u16 = undefined;
    for (&samples, 0..) |*s, i| s.* = @intCast(i * 4096 / 16);
    const curv = curvPayload(17, samples);
    const profile = try buildProfile(alloc, "RGB XYZ acsp", &.{
        .{ .keyword = .{ 'r', 'X', 'Y', 'Z' }, .data = &xyz_r },
        .{ .keyword = .{ 'g', 'X', 'Y', 'Z' }, .data = &xyz_g },
        .{ .keyword = .{ 'b', 'X', 'Y', 'Z' }, .data = &xyz_b },
        .{ .keyword = .{ 'r', 'T', 'R', 'C' }, .data = &curv },
        .{ .keyword = .{ 'g', 'T', 'R', 'C' }, .data = &curv },
        .{ .keyword = .{ 'b', 'T', 'R', 'C' }, .data = &curv },
        .{ .keyword = .{ 'w', 't', 'p', 't' }, .data = &xyz_w },
    });
    defer alloc.free(profile);
    try expectPredictRoundtrip(alloc, profile);
}

test "predict roundtrip: desc and text tag types" {
    const alloc = std.testing.allocator;
    const profile = try buildProfile(alloc, "", &.{
        .{ .keyword = .{ 'd', 'e', 's', 'c' }, .data = "desc\x00\x00\x00\x00\x00\x00\x00\x0bhello world\x00\x00\x00\x00\x00\x00" },
        .{ .keyword = .{ 'c', 'p', 'r', 't' }, .data = "text\x00\x00\x00\x00\x00\x00\x00\x07my icc!" },
    });
    defer alloc.free(profile);
    try expectPredictRoundtrip(alloc, profile);
}

test "predict roundtrip: mluc tag" {
    const alloc = std.testing.allocator;
    // mluc: 4-byte count + 12-byte records (lang, country, length, offset)
    // followed by strings. num = tagsize - 8 must be even for shuffle2.
    var mluc: [8 + 12 * 2 + 8]u8 = @splat(0);
    mluc[0..4].* = .{ 'm', 'l', 'u', 'c' };
    mluc[4] = 0;
    mluc[7] = 2;
    // record 1: en US, 4 bytes at offset 0
    mluc[8..12].* = .{ 'e', 'n', 'U', 'S' };
    std.mem.writeInt(u32, mluc[12..16], 4, .big);
    std.mem.writeInt(u32, mluc[16..20], 0, .big);
    // record 2: de DE, 4 bytes at offset 4
    mluc[20..24].* = .{ 'd', 'e', 'D', 'E' };
    std.mem.writeInt(u32, mluc[24..28], 4, .big);
    std.mem.writeInt(u32, mluc[28..32], 4, .big);
    mluc[32..40].* = "abcdABCD".*;
    const profile = try buildProfile(alloc, "", &.{
        .{ .keyword = .{ 'd', 'e', 's', 'c' }, .data = &mluc },
    });
    defer alloc.free(profile);
    try expectPredictRoundtrip(alloc, profile);
}

test "predict roundtrip: random inputs are always lossless" {
    const alloc = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(42);
    const rand = prng.random();
    const sizes = [_]usize{ 1, 5, 20, 129, 132, 133, 200, 257, 400 };
    for (sizes) |n| {
        const buf = try alloc.alloc(u8, n);
        defer alloc.free(buf);
        rand.bytes(buf);
        try expectPredictRoundtrip(alloc, buf);
    }
}

test "predict roundtrip: random profiles with plausible tag tables" {
    const alloc = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(777);
    const rand = prng.random();
    var iter: usize = 0;
    while (iter < 24) : (iter += 1) {
        // Random-ish profile: header + random tag table + random payloads.
        const numtags = 1 + rand.uintLessThan(usize, 6);
        var buf: std.ArrayList(u8) = .empty;
        defer buf.deinit(alloc);
        const hdr = try alloc.alloc(u8, 128);
        defer alloc.free(hdr);
        rand.bytes(hdr);
        try buf.appendSlice(alloc, hdr);
        var count: [4]u8 = undefined;
        std.mem.writeInt(u32, &count, @intCast(numtags), .big);
        try buf.appendSlice(alloc, &count);
        const table_start = buf.items.len;
        try buf.appendNTimes(alloc, 0, numtags * 12);
        const keywords = [_][4]u8{
            .{ 'r', 'X', 'Y', 'Z' }, .{ 'g', 'X', 'Y', 'Z' }, .{ 'b', 'X', 'Y', 'Z' },
            .{ 'r', 'T', 'R', 'C' }, .{ 'g', 'T', 'R', 'C' }, .{ 'b', 'T', 'R', 'C' },
            .{ 'w', 't', 'p', 't' }, .{ 'c', 'h', 'a', 'd' }, .{ 'd', 'e', 's', 'c' },
            .{ 'q', 'q', 'q', 'q' },
        };
        var offset: u32 = @intCast(table_start + numtags * 12);
        var t: usize = 0;
        while (t < numtags) : (t += 1) {
            const kw = keywords[rand.uintLessThan(usize, keywords.len)];
            const dlen = rand.uintLessThan(usize, 40);
            const pos = table_start + t * 12;
            buf.items[pos..][0..4].* = kw;
            std.mem.writeInt(u32, buf.items[pos + 4 ..][0..4], offset, .big);
            std.mem.writeInt(u32, buf.items[pos + 8 ..][0..4], @intCast(dlen), .big);
            var d: usize = 0;
            while (d < dlen) : (d += 1) {
                try buf.append(alloc, rand.int(u8));
            }
            offset += @intCast(dlen);
        }
        std.mem.writeInt(u32, buf.items[0..4], @intCast(buf.items.len), .big);
        try expectPredictRoundtrip(alloc, buf.items);
    }
}

// ---------------------------------------------------------------------------
// Entropy-coded stream roundtrip
// ---------------------------------------------------------------------------

fn expectStreamRoundtrip(alloc: std.mem.Allocator, profile: []const u8) !void {
    var w = BitWriter.init(alloc);
    defer w.deinit();
    try enc_icc.writeICC(alloc, profile, &w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    const got = try icc.readICC(alloc, &br);
    defer alloc.free(got);
    try br.close();
    try std.testing.expectEqualSlices(u8, profile, got);
}

test "icc stream roundtrip: synthetic profiles" {
    const alloc = std.testing.allocator;
    const xyz_r = xyzPayload(.{ 0, 0, 0, 0, 'x', 1, 2, 3, 4, 5, 6, 7 });
    var samples: [17]u16 = undefined;
    for (&samples, 0..) |*s, i| s.* = @intCast(i * 4000);
    const curv = curvPayload(17, samples);
    const profile = try buildProfile(alloc, "RGB XYZ acsp", &.{
        .{ .keyword = .{ 'r', 'T', 'R', 'C' }, .data = &curv },
        .{ .keyword = .{ 'r', 'X', 'Y', 'Z' }, .data = &xyz_r },
    });
    defer alloc.free(profile);
    try expectStreamRoundtrip(alloc, profile);
}

test "icc stream roundtrip: random data (lz77 + prefix code paths)" {
    const alloc = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(2024);
    const rand = prng.random();
    const sizes = [_]usize{ 200, 4096, 17000 }; // <16384 -> kOptimal, else kLZ77
    for (sizes) |n| {
        const buf = try alloc.alloc(u8, n);
        defer alloc.free(buf);
        // Mix of runs and noise so both the LZ77 and literal paths are
        // exercised.
        var i: usize = 0;
        while (i < n) {
            if (rand.boolean()) {
                const run = @min(1 + rand.uintLessThan(usize, 40), n - i);
                const v = rand.int(u8);
                @memset(buf[i .. i + run], v);
                i += run;
            } else {
                buf[i] = rand.int(u8);
                i += 1;
            }
        }
        try expectStreamRoundtrip(alloc, buf);
    }
}

test "icc stream roundtrip: header-only and tiny inputs" {
    const alloc = std.testing.allocator;
    try expectStreamRoundtrip(alloc, "t");
    const hdr = try alloc.alloc(u8, 128);
    defer alloc.free(hdr);
    for (hdr, 0..) |*b, i| b.* = @truncate(i *% 3);
    try expectStreamRoundtrip(alloc, hdr);
}

// ---------------------------------------------------------------------------
// Real profile from the JPEG corpus + codestream roundtrip
// ---------------------------------------------------------------------------

test "real profile from icc_444.jpg: extract, predict, stream, refit markers" {
    const alloc = std.testing.allocator;
    const jd = @import("../jpeg/jpeg_data.zig");
    const reader = @import("../jpeg/reader.zig");
    const jbrd = @import("../jpeg/jbrd.zig");
    const data = @embedFile("../jpeg/testdata/icc_444.jpg");

    var jpg = jd.JPEGData{};
    defer jpg.deinit(alloc);
    try reader.readJpeg(data, .read_all, &jpg, alloc);
    try jbrd.detectIccProfile(&jpg);
    const icc_bytes = (try jbrd.extractIcc(alloc, &jpg)).?;
    defer alloc.free(icc_bytes);
    try std.testing.expect(icc_bytes.len > 128);
    // This fixture's profile is truncated (the header claims 66051 bytes,
    // only 768 travel in the single APP2 chunk), so it must NOT qualify
    // for embedding (libjxl rejects such JPEGs outright).
    try std.testing.expect(!icc.looksLikeValidProfile(icc_bytes));

    // The transform still roundtrips arbitrary bytes losslessly.
    try expectStreamRoundtrip(alloc, icc_bytes);

    // The jbrd marker flow: zero the payloads, then refill them from the
    // profile exactly like SetJPEGDataFromICC does.
    for (jpg.app_data.items, 0..) |*marker, i| {
        if (jpg.app_marker_type.items[i] != .icc) continue;
        for (marker.items[17..]) |*b| b.* = 0;
    }
    try jbrd.setJpegDataFromIcc(icc_bytes, &jpg);
    // Compare against the original file's APP2 payloads by re-reading.
    var jpg2 = jd.JPEGData{};
    defer jpg2.deinit(alloc);
    try reader.readJpeg(data, .read_all, &jpg2, alloc);
    try std.testing.expectEqual(jpg2.app_data.items.len, jpg.app_data.items.len);
    for (jpg2.app_data.items, 0..) |*m, i| {
        try std.testing.expectEqualSlices(u8, m.items, jpg.app_data.items[i].items);
    }
}

test "jpeg transcode embeds a valid icc and reconstructs it" {
    const alloc = std.testing.allocator;
    const transcode = @import("../jpeg/transcode.zig");
    const jd = @import("../jpeg/jpeg_data.zig");
    const reader = @import("../jpeg/reader.zig");
    const writer = @import("../jpeg/writer.zig");
    const jbrd = @import("../jpeg/jbrd.zig");
    const data = @embedFile("../jpeg/testdata/icc_444.jpg");

    // Patch the fixture's truncated APP2 ICC payload in place with a
    // header-valid synthetic profile of the same length (768 bytes), so the
    // marker segment length stays untouched.
    var jpg = jd.JPEGData{};
    defer jpg.deinit(alloc);
    try reader.readJpeg(data, .read_all, &jpg, alloc);
    try jbrd.detectIccProfile(&jpg);
    var icc_len: usize = 0;
    for (jpg.app_data.items, 0..) |*marker, i| {
        if (jpg.app_marker_type.items[i] == .icc) icc_len = marker.items.len - 17;
    }
    try std.testing.expect(icc_len >= 128);

    const icc_bytes = try alloc.alloc(u8, icc_len);
    defer alloc.free(icc_bytes);
    {
        // Build a valid profile and pad it to the payload length; the size
        // field must equal the payload length, so generate enough tags.
        var profile: std.ArrayList(u8) = .empty;
        defer profile.deinit(alloc);
        var tags: std.ArrayList(TagSpec) = .empty;
        defer tags.deinit(alloc);
        const xyz_r = try alloc.create([20]u8);
        defer alloc.destroy(xyz_r);
        xyz_r.* = xyzPayload(.{ 0, 0, 0, 0, 'r', 1, 2, 3, 4, 5, 6, 7 });
        try tags.append(alloc, .{ .keyword = .{ 'r', 'X', 'Y', 'Z' }, .data = xyz_r });
        var curv_samples: [17]u16 = undefined;
        for (&curv_samples, 0..) |*s, i| s.* = @intCast(i * 321);
        const curv = try alloc.create([42]u8);
        defer alloc.destroy(curv);
        curv.* = curvPayload(17, curv_samples);
        try tags.append(alloc, .{ .keyword = .{ 'r', 'T', 'R', 'C' }, .data = curv });
        // rpad: filler tag data to reach exactly icc_len bytes.
        const base = try buildProfile(alloc, "RGB XYZ acsp", tags.items);
        defer alloc.free(base);
        try std.testing.expect(base.len <= icc_len);
        try profile.appendSlice(alloc, base);
        try profile.appendNTimes(alloc, 0, icc_len - base.len);
        std.mem.writeInt(u32, profile.items[0..4], @intCast(icc_len), .big);
        @memcpy(icc_bytes, profile.items);
    }
    try std.testing.expect(icc.looksLikeValidProfile(icc_bytes));

    for (jpg.app_data.items, 0..) |*marker, i| {
        if (jpg.app_marker_type.items[i] != .icc) continue;
        @memcpy(marker.items[17..], icc_bytes);
    }
    var patched: std.ArrayList(u8) = .empty;
    defer patched.deinit(alloc);
    try writer.writeJpeg(&jpg, &patched, alloc);

    // Transcode: the headers must carry the real ICC and the JPEG must come
    // back byte-exact (the APP2 payload is rebuilt from the header ICC).
    const jxl = try transcode.encodeTranscode(alloc, patched.items);
    defer alloc.free(jxl);
    const jpeg = try transcode.decodeTranscode(alloc, jxl);
    defer alloc.free(jpeg);
    try std.testing.expectEqualSlices(u8, patched.items, jpeg);

    // And the embedded profile decodes back byte-exact through the
    // codestream-header ICC stream (same sequence as decodeTranscode).
    const frame_decode = @import("../vardct/frame_decode.zig");
    const container_mod = @import("../header/container.zig");
    const cs = try container_mod.extractCodestream(alloc, jxl);
    defer alloc.free(cs);
    const jbrd_payload = container_mod.findBox(jxl, "jbrd").?;
    var jpg2 = jd.JPEGData{};
    defer jpg2.deinit(alloc);
    try jbrd.decode(alloc, jbrd_payload, &jpg2);
    var icc2: ?[]u8 = null;
    defer if (icc2) |b| alloc.free(b);
    try frame_decode.decodeTranscodeFrame(alloc, cs, &jpg2, &icc2);
    try std.testing.expect(icc2 != null);
    try std.testing.expectEqualSlices(u8, icc_bytes, icc2.?);
}

test "codestream roundtrip with embedded icc (rgb + gray)" {
    const alloc = std.testing.allocator;
    const encoder = @import("../codec/encoder.zig");
    const decoder = @import("../codec/decoder.zig");
    const pixel = @import("../codec/pixel.zig");

    // A header-valid synthetic RGB profile (the corpus fixture's profile is
    // truncated, see the test above).
    const xyz_r = xyzPayload(.{ 0, 0, 0, 0, 'r', 1, 2, 3, 4, 5, 6, 7 });
    var samples: [17]u16 = undefined;
    for (&samples, 0..) |*s, i| s.* = @intCast(i * 4000);
    const curv = curvPayload(17, samples);
    const icc_bytes = try buildProfile(alloc, "RGB XYZ acsp", &.{
        .{ .keyword = .{ 'r', 'T', 'R', 'C' }, .data = &curv },
        .{ .keyword = .{ 'r', 'X', 'Y', 'Z' }, .data = &xyz_r },
    });
    defer alloc.free(icc_bytes);
    try std.testing.expect(icc.looksLikeValidProfile(icc_bytes));

    // RGB and gray variants (the profile color space only affects the
    // serialized color_space enum).
    for ([_]pixel.Layout{ .rgb, .gray }) |layout| {
        var pb = try pixel.PixelBuffer.create(alloc, 39, 17, layout, false, 8);
        defer pb.deinit();
        var prng = std.Random.DefaultPrng.init(9);
        const rand = prng.random();
        const nch = pb.numChannels();
        for (0..pb.ysize) |y| {
            for (0..pb.xsize) |x| {
                for (0..nch) |c| {
                    pb.set(x, y, c, rand.int(u8));
                }
            }
        }
        var w = bit_io.BitWriter.init(alloc);
        defer w.deinit();
        try encoder.encode(alloc, &pb, .{ .icc = icc_bytes }, &w);
        w.zeroPadToByte();

        var icc_out: ?[]u8 = null;
        var got = try decoder.decode(alloc, w.getSpan(), .{ .icc_out = &icc_out });
        defer got.deinit();
        defer if (icc_out) |b| alloc.free(b);
        try std.testing.expect(icc_out != null);
        try std.testing.expectEqualSlices(u8, icc_bytes, icc_out.?);
        try std.testing.expect(got == .int);
        try std.testing.expectEqual(pb.xsize, got.int.xsize);
        // Pixel-exact decode.
        for (0..pb.ysize) |y| {
            for (0..pb.xsize) |x| {
                for (0..nch) |c| {
                    try std.testing.expectEqual(pb.at(x, y, c), got.int.at(x, y, c));
                }
            }
        }
    }
}
