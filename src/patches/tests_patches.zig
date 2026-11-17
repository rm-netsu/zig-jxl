//! End-to-end patch tests (encode with patches -> decode), shared by the
//! library test build. Filled in with the codec integration: roundtrips
//! over text-like images (the shape FindTextLikePatches detects), the
//! codestream shape checks (a kReferenceOnly frame + the kPatches flag)
//! and the negative cases (patches off / no patches found).

const std = @import("std");

const pixel_mod = @import("../codec/pixel.zig");
const encoder = @import("../codec/encoder.zig");
const decoder = @import("../codec/decoder.zig");
const bit_io = @import("../base/bit_io.zig");
const size_mod = @import("../header/size.zig");
const metadata_mod = @import("../header/metadata.zig");
const frame_mod = @import("../header/frame.zig");
const container_mod = @import("../header/container.zig");

const Allocator = std.mem.Allocator;
const PixelBuffer = pixel_mod.PixelBuffer;
const BitWriter = bit_io.BitWriter;

// 6x8 "glyph" bitmaps: 0 = ink, 1 = paper.
const kGlyphs = [4][8][6]u8{
    .{
        .{ 0, 1, 1, 1, 1, 0 },
        .{ 1, 1, 0, 0, 1, 1 },
        .{ 0, 0, 0, 0, 1, 1 },
        .{ 0, 1, 1, 1, 1, 1 },
        .{ 1, 1, 0, 0, 1, 1 },
        .{ 1, 1, 0, 0, 1, 1 },
        .{ 1, 1, 0, 0, 1, 1 },
        .{ 0, 0, 0, 0, 0, 0 },
    },
    .{
        .{ 0, 1, 1, 1, 1, 0 },
        .{ 1, 1, 0, 0, 1, 1 },
        .{ 1, 1, 1, 1, 1, 1 },
        .{ 1, 1, 1, 1, 1, 0 },
        .{ 1, 1, 0, 0, 0, 0 },
        .{ 1, 1, 0, 0, 1, 1 },
        .{ 0, 1, 1, 1, 1, 0 },
        .{ 0, 0, 0, 0, 0, 0 },
    },
    .{
        .{ 1, 1, 1, 1, 1, 1 },
        .{ 0, 0, 1, 1, 0, 0 },
        .{ 0, 0, 1, 1, 0, 0 },
        .{ 0, 0, 1, 1, 0, 0 },
        .{ 0, 0, 1, 1, 0, 0 },
        .{ 0, 0, 1, 1, 0, 0 },
        .{ 0, 0, 1, 1, 0, 0 },
        .{ 0, 0, 0, 0, 0, 0 },
    },
    .{
        .{ 0, 1, 1, 1, 1, 1 },
        .{ 1, 1, 0, 0, 0, 0 },
        .{ 1, 1, 0, 0, 0, 0 },
        .{ 0, 1, 1, 1, 1, 0 },
        .{ 0, 0, 0, 0, 1, 1 },
        .{ 0, 0, 0, 0, 1, 1 },
        .{ 0, 1, 1, 1, 1, 0 },
        .{ 0, 0, 0, 0, 0, 0 },
    },
};

fn glyphInk(gi: usize, c: usize) u16 {
    // dark ink, slightly different per glyph
    const base = [3]u16{ 30, 25, 35 };
    const delta = [3]u16{ 7, 3, 11 };
    return base[c] + delta[c] * @as(u16, @intCast(gi));
}

/// White background with four text lines of repeated glyphs (the shape
/// libjxl FindTextLikePatches detects: flat background + repeated small
/// connected components). Alpha (when present): paper = max, ink = 128
/// (a nontrivial plane patches must not touch).
fn fillTextImage(pb: *PixelBuffer) void {
    const maxv = pb.maxValue();
    const scale: u16 = if (pb.bitdepth == 16) 257 else 1;
    const nch = pb.numChannels();
    const gray = pb.layout == .gray;
    for (0..pb.ysize) |y| {
        for (0..pb.xsize) |x| {
            var ink: ?usize = null;
            for (0..4) |li| {
                const ly = 40 + li * 48;
                if (y < ly or y >= ly + 16) continue;
                if (x < 8) continue;
                const ci = (x - 8) / 16;
                if (ci >= 15) continue;
                const lx = (x - 8) % 16;
                const gi = (ci * 3 + li * 7) % 4;
                const g = kGlyphs[gi];
                if (y - ly >= 4 and y - ly < 12 and lx >= 5 and lx < 11) {
                    if (g[y - ly - 4][lx - 5] == 0) ink = gi;
                }
            }
            var c: usize = 0;
            while (c < nch) : (c += 1) {
                const is_alpha = pb.has_alpha and c == nch - 1;
                if (ink) |gi| {
                    if (is_alpha) {
                        pb.set(x, y, c, @min(128 * scale, maxv));
                    } else {
                        const cc: usize = if (gray) 0 else @min(c, 2);
                        pb.set(
                            x,
                            y,
                            c,
                            @min(glyphInk(gi, cc) * scale, maxv),
                        );
                    }
                } else if (is_alpha) {
                    pb.set(x, y, c, maxv);
                } else {
                    pb.set(x, y, c, maxv);
                }
            }
        }
    }
}

fn fillNoise(pb: *PixelBuffer, seed: u64) void {
    var prng = std.Random.DefaultPrng.init(seed);
    const rand = prng.random();
    const bound: usize = @as(usize, pb.maxValue()) + 1;
    for (pb.data) |*v| {
        v.* = @intCast(rand.uintLessThan(usize, bound));
    }
}

fn encodeBytes(
    allocator: Allocator,
    pb: *const PixelBuffer,
    opts: encoder.EncodeOptions,
) ![]u8 {
    var w = BitWriter.init(allocator);
    defer w.deinit();
    try encoder.encode(allocator, pb, opts, &w);
    w.zeroPadToByte();
    return w.takeBytes();
}

fn roundtripPatches(
    allocator: Allocator,
    pb: *const PixelBuffer,
    opts: encoder.EncodeOptions,
) !void {
    const bytes = try encodeBytes(allocator, pb, opts);
    defer allocator.free(bytes);
    var got = try decoder.decode(
        allocator,
        bytes,
        .{},
    );
    defer got.deinit();
    try std.testing.expect(got == .int);
    try std.testing.expect(pb.eql(&got.int));
}

/// Parsed codestream frame skeleton (frame headers only, no sections).
const FrameSkeleton = struct {
    frame_type: frame_mod.FrameType,
    is_last: bool,
    save_as_reference: u32,
    save_before_color_transform: bool,
    flags: u64,
};

fn parseFrames(
    allocator: Allocator,
    bytes: []const u8,
) !std.ArrayList(FrameSkeleton) {
    var out: std.ArrayList(FrameSkeleton) = .empty;
    errdefer out.deinit(allocator);
    var br = bit_io.BitReader.init(bytes);
    _ = br.readFixedBits(8);
    _ = br.readFixedBits(8);
    var sh: size_mod.SizeHeader = .{};
    try sh.read(&br);
    var m = metadata_mod.ImageMetadata.init(allocator);
    defer m.deinit();
    try m.read(&br);
    var ctd = metadata_mod.CustomTransformData{};
    try ctd.read(&br, m.xyb_encoded);
    try br.jumpToByteBoundary();
    while (true) {
        var fh = frame_mod.FrameHeader.init(allocator);
        defer fh.deinit();
        try fh.read(&m, &br);
        try out.append(allocator, .{
            .frame_type = fh.frame_type,
            .is_last = fh.is_last,
            .save_as_reference = fh.save_as_reference,
            .save_before_color_transform = fh.save_before_color_transform,
            .flags = fh.flags,
        });
        if (fh.is_last) break;
        // Skip this frame's sections: TOC + payload.
        const fd = @import("../modular/stream.zig").FrameDim.init(
            if (fh.custom_size_or_origin) fh.frame_size_x else sh.xsizeOf(),
            if (fh.custom_size_or_origin) fh.frame_size_y else sh.ysizeOf(),
            fh.group_size_shift,
            1,
        );
        const toc_mod = @import("../header/toc.zig");
        const num_toc = toc_mod.numTocEntries(
            fd.num_groups,
            fd.num_dc_groups,
            fh.passes.num_passes,
        );
        var toc = try toc_mod.readTocWithPermutation(
            allocator,
            num_toc,
            &br,
        );
        defer toc.deinit(allocator);
        const total = toc.totalSize();
        _ = br.skipBits(total * 8);
    }
    return out;
}

test "patch roundtrip: text-like rgb 8-bit" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        false,
        8,
    );
    defer pb.deinit();
    fillTextImage(&pb);
    try roundtripPatches(
        alloc,
        &pb,
        .{ .patches = .on },
    );
}

test "patch roundtrip: text-like rgb 8-bit default effort" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        false,
        8,
    );
    defer pb.deinit();
    fillTextImage(&pb);
    try roundtripPatches(alloc, &pb, .{});
}

test "patch roundtrip: text-like rgb with alpha" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        true,
        8,
    );
    defer pb.deinit();
    fillTextImage(&pb);
    try roundtripPatches(
        alloc,
        &pb,
        .{ .patches = .on },
    );
}

test "patch roundtrip: text-like gray 8-bit" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(
        alloc,
        256,
        256,
        .gray,
        false,
        8,
    );
    defer pb.deinit();
    fillTextImage(&pb);
    try roundtripPatches(
        alloc,
        &pb,
        .{ .patches = .on },
    );
}

test "patch roundtrip: text-like rgb 16-bit" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        false,
        16,
    );
    defer pb.deinit();
    fillTextImage(&pb);
    try roundtripPatches(
        alloc,
        &pb,
        .{ .patches = .on },
    );
}

test "patch roundtrip: text-like + responsive + progressive is orthogonal" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        false,
        8,
    );
    defer pb.deinit();
    fillTextImage(&pb);
    try roundtripPatches(
        alloc,
        &pb,
        .{
            .patches = .on,
            .responsive = true,
            .progressive = true,
        },
    );
}

test "patch codestream shape: reference frame + kPatches flag" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        false,
        8,
    );
    defer pb.deinit();
    fillTextImage(&pb);
    const bytes = try encodeBytes(
        alloc,
        &pb,
        .{ .patches = .on },
    );
    defer alloc.free(bytes);

    var frames = try parseFrames(
        alloc,
        bytes,
    );
    defer frames.deinit(alloc);
    try std.testing.expectEqual(@as(usize, 2), frames.items.len);
    // The patch reference frame: kReferenceOnly, saved into slot 3 before
    // the color transform, never last, no kPatches of its own.
    try std.testing.expect(frames.items[0].frame_type == .reference_only);
    try std.testing.expect(!frames.items[0].is_last);
    try std.testing.expectEqual(
        @as(u32, 3),
        frames.items[0].save_as_reference & 3,
    );
    try std.testing.expect(frames.items[0].save_before_color_transform);
    try std.testing.expectEqual(
        @as(u64, 0),
        frames.items[0].flags & frame_mod.Flags.kPatches,
    );
    // The main frame: regular, last, carrying kPatches.
    try std.testing.expect(frames.items[1].frame_type == .regular);
    try std.testing.expect(frames.items[1].is_last);
    try std.testing.expect(frames.items[1].flags & frame_mod.Flags.kPatches != 0);
}

test "no patches found: single regular frame, no kPatches flag" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(
        alloc,
        96,
        96,
        .rgb,
        false,
        8,
    );
    defer pb.deinit();
    fillNoise(&pb, 0x1234_5678);
    const bytes = try encodeBytes(
        alloc,
        &pb,
        .{ .patches = .on },
    );
    defer alloc.free(bytes);

    var frames = try parseFrames(
        alloc,
        bytes,
    );
    defer frames.deinit(alloc);
    try std.testing.expectEqual(@as(usize, 1), frames.items.len);
    try std.testing.expect(frames.items[0].frame_type == .regular);
    try std.testing.expect(frames.items[0].is_last);
    try std.testing.expectEqual(
        @as(u64, 0),
        frames.items[0].flags & frame_mod.Flags.kPatches,
    );
    try roundtripPatches(
        alloc,
        &pb,
        .{ .patches = .on },
    );
}

test "patch effectiveness: patches shrink the text-like image" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        false,
        8,
    );
    defer pb.deinit();
    fillTextImage(&pb);
    const with = try encodeBytes(
        alloc,
        &pb,
        .{ .patches = .on },
    );
    defer alloc.free(with);
    const without = try encodeBytes(
        alloc,
        &pb,
        .{ .patches = .off },
    );
    defer alloc.free(without);
    // The reference frame + dictionary must cost less than the residual
    // coding of the full glyphs (cjxl shows the same relation).
    try std.testing.expect(with.len < without.len);
}

// ---------------------------------------------------------------------------
// fp16 (binary16) patches: libjxl's enc_modular gate enables the patch
// search for bits_per_sample <= 16, which includes 16-bit float samples.
// ---------------------------------------------------------------------------

const FloatPixelBuffer = pixel_mod.FloatPixelBuffer;

/// The text-like pattern with exact dyadic samples (v * 2^-8): every
/// value and every glyph/background difference is representable in
/// binary16, so the fp16 patch path is lossless.
fn fillTextImageFloat(fb: *FloatPixelBuffer) void {
    const nch = fb.numChannels();
    const gray = fb.layout == .gray;
    const bg: f32 = 224.0 / 256.0;
    for (0..fb.ysize) |y| {
        for (0..fb.xsize) |x| {
            var ink: ?usize = null;
            for (0..4) |li| {
                const ly = 40 + li * 48;
                if (y < ly or y >= ly + 16) continue;
                if (x < 8) continue;
                const ci = (x - 8) / 16;
                if (ci >= 15) continue;
                const lx = (x - 8) % 16;
                const gi = (ci * 3 + li * 7) % 4;
                const g = kGlyphs[gi];
                if (y - ly >= 4 and y - ly < 12 and lx >= 5 and lx < 11) {
                    if (g[y - ly - 4][lx - 5] == 0) ink = gi;
                }
            }
            var c: usize = 0;
            while (c < nch) : (c += 1) {
                const v: f32 = if (ink) |gi|
                    @as(f32, @floatFromInt(glyphInk(
                        gi,
                        if (gray) 0 else @min(c, 2),
                    ))) / 256.0
                else
                    bg;
                fb.set(x, y, c, v);
            }
        }
    }
}

fn encodeFloatBytes(
    allocator: Allocator,
    fb: *const FloatPixelBuffer,
    opts: encoder.EncodeOptions,
) ![]u8 {
    var w = BitWriter.init(allocator);
    defer w.deinit();
    try encoder.encodeFloat(
        allocator,
        fb,
        opts,
        &w,
    );
    w.zeroPadToByte();
    return w.takeBytes();
}

fn roundtripFloatPatches(
    allocator: Allocator,
    fb: *const FloatPixelBuffer,
    opts: encoder.EncodeOptions,
) !void {
    const bytes = try encodeFloatBytes(
        allocator,
        fb,
        opts,
    );
    defer allocator.free(bytes);
    var got = try decoder.decode(
        allocator,
        bytes,
        .{},
    );
    defer got.deinit();
    try std.testing.expect(got == .float);
    try std.testing.expect(fb.eql(&got.float));
}

test "patch roundtrip: text-like fp16 rgb" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        16,
        5,
    );
    defer fb.deinit();
    fillTextImageFloat(&fb);
    try roundtripFloatPatches(
        alloc,
        &fb,
        .{ .patches = .on, .float16 = true },
    );
}

test "patch roundtrip: text-like fp16 gray" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(
        alloc,
        256,
        256,
        .gray,
        16,
        5,
    );
    defer fb.deinit();
    fillTextImageFloat(&fb);
    try roundtripFloatPatches(
        alloc,
        &fb,
        .{ .patches = .on, .float16 = true },
    );
}

test "patch roundtrip: text-like fp16 default effort (patches on at >= 7)" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        16,
        5,
    );
    defer fb.deinit();
    fillTextImageFloat(&fb);
    try roundtripFloatPatches(
        alloc,
        &fb,
        .{ .float16 = true },
    );
}

test "patch roundtrip: text-like fp16 progressive + responsive" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        16,
        5,
    );
    defer fb.deinit();
    fillTextImageFloat(&fb);
    try roundtripFloatPatches(
        alloc,
        &fb,
        .{
            .patches = .on,
            .float16 = true,
            .progressive = true,
            .responsive = true,
        },
    );
}

test "patch codestream shape: fp16 reference frame + kPatches flag" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        16,
        5,
    );
    defer fb.deinit();
    fillTextImageFloat(&fb);
    const bytes = try encodeFloatBytes(
        alloc,
        &fb,
        .{ .patches = .on, .float16 = true },
    );
    defer alloc.free(bytes);

    var frames = try parseFrames(
        alloc,
        bytes,
    );
    defer frames.deinit(alloc);
    try std.testing.expectEqual(
        @as(usize, 2),
        frames.items.len,
    );
    try std.testing.expect(frames.items[0].frame_type == .reference_only);
    try std.testing.expect(!frames.items[0].is_last);
    try std.testing.expectEqual(
        @as(u32, 3),
        frames.items[0].save_as_reference & 3,
    );
    try std.testing.expect(frames.items[0].save_before_color_transform);
    try std.testing.expect(frames.items[1].frame_type == .regular);
    try std.testing.expect(frames.items[1].is_last);
    try std.testing.expect(
        frames.items[1].flags & frame_mod.Flags.kPatches != 0,
    );
}

test "patch effectiveness: fp16 patches shrink the text-like image" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        16,
        5,
    );
    defer fb.deinit();
    fillTextImageFloat(&fb);
    const with = try encodeFloatBytes(
        alloc,
        &fb,
        .{ .patches = .on, .float16 = true },
    );
    defer alloc.free(with);
    const without = try encodeFloatBytes(
        alloc,
        &fb,
        .{ .patches = .off, .float16 = true },
    );
    defer alloc.free(without);
    try std.testing.expect(with.len < without.len);
}

test "patch fp16 hostile: unrepresentable difference fails the encode" {
    // bg 2^-13, ink 1.0: every sample is fp16-exact, but the glyph/
    // background difference (1 - 2^-13 = 8191 * 2^-13) needs 13
    // significant bits and cannot be coded in the patch reference frame —
    // libjxl's float_to_int fails the same way (its whole encode errors
    // out at default settings).
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        16,
        5,
    );
    defer fb.deinit();
    for (0..fb.ysize) |y| {
        for (0..fb.xsize) |x| {
            // Reuse the text-like layout: ink pixels 1.0, background 2^-13.
            var ink = false;
            for (0..4) |li| {
                const ly = 40 + li * 48;
                if (y < ly or y >= ly + 16) continue;
                if (x < 8) continue;
                const ci = (x - 8) / 16;
                if (ci >= 15) continue;
                const lx = (x - 8) % 16;
                const gi = (ci * 3 + li * 7) % 4;
                const g = kGlyphs[gi];
                if (y - ly >= 4 and y - ly < 12 and lx >= 5 and lx < 11) {
                    if (g[y - ly - 4][lx - 5] == 0) ink = true;
                }
            }
            const v: f32 = if (ink) 1.0 else 0.0001220703125;
            for (0..3) |c| fb.set(x, y, c, v);
        }
    }
    // The patch path must refuse (float_to_int of the difference loses
    // precision), exactly like libjxl; with patches off it succeeds.
    try std.testing.expectError(
        error.FloatLosesPrecision,
        encodeFloatBytes(alloc, &fb, .{ .patches = .on, .float16 = true }),
    );
    try std.testing.expectError(
        error.FloatLosesPrecision,
        encodeFloatBytes(
            alloc,
            &fb,
            .{ .float16 = true },
        ),
    );
    const off = try encodeFloatBytes(
        alloc,
        &fb,
        .{ .patches = .off, .float16 = true },
    );
    defer alloc.free(off);
}

test "patch fp32: search skipped (libjxl gate bits_per_sample <= 16)" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(
        alloc,
        256,
        256,
        .rgb,
        32,
        8,
    );
    defer fb.deinit();
    fillTextImageFloat(&fb);
    const bytes = try encodeFloatBytes(
        alloc,
        &fb,
        .{ .patches = .on },
    );
    defer alloc.free(bytes);
    // No patch reference frame: a single regular frame without kPatches.
    var frames = try parseFrames(
        alloc,
        bytes,
    );
    defer frames.deinit(alloc);
    try std.testing.expectEqual(@as(usize, 1), frames.items.len);
    try std.testing.expect(frames.items[0].frame_type == .regular);
    try std.testing.expect(frames.items[0].flags & frame_mod.Flags.kPatches == 0);
    // And the roundtrip still works.
    try roundtripFloatPatches(
        alloc,
        &fb,
        .{ .patches = .on },
    );
}

test {
    _ = @import("dictionary.zig");
    _ = @import("enc_patches.zig");
}
