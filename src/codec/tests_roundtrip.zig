//! End-to-end roundtrip tests: PixelBuffer -> encode -> decode.

const std = @import("std");
const pixel_mod = @import("pixel.zig");
const encoder = @import("encoder.zig");
const decoder = @import("decoder.zig");
const bit_io = @import("../base/bit_io.zig");

const Allocator = std.mem.Allocator;
const PixelBuffer = pixel_mod.PixelBuffer;
const BitWriter = bit_io.BitWriter;

fn fillSolid(pb: *PixelBuffer, value: u16) void {
    for (pb.data) |*v| v.* = value;
}

fn fillGradient(pb: *PixelBuffer) void {
    const nch = pb.numChannels();
    const max_value = pb.maxValue();
    for (0..pb.ysize) |y| {
        for (0..pb.xsize) |x| {
            for (0..nch) |c| {
                const raw: u32 = switch (c) {
                    0 => @intCast((x * 255) / @max(1, pb.xsize - 1)),
                    1 => @intCast((y * 255) / @max(1, pb.ysize - 1)),
                    else => @as(u32, @intCast((x + y) & 0xFF)),
                };
                const v: u16 = @intCast(if (pb.bitdepth == 16) raw * 257 else raw);
                pb.set(x, y, c, @min(v, max_value));
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

fn roundtrip(allocator: Allocator, pb: *const PixelBuffer, group_size_shift: u32) !void {
    var w = BitWriter.init(allocator);
    defer w.deinit();
    try encoder.encode(allocator, pb, .{ .group_size_shift = group_size_shift }, &w);
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer allocator.free(bytes);

    var got = try decoder.decode(allocator, bytes, .{});
    defer got.deinit();
    try std.testing.expect(got == .int);
    try std.testing.expect(pb.eql(&got.int));
}

test "roundtrip solid 8-bit gray" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 33, 17, .gray, false, 8);
    defer pb.deinit();
    fillSolid(&pb, 128);
    try roundtrip(alloc, &pb, 1);
}

test "roundtrip gradient 8-bit rgb" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 64, 48, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtrip(alloc, &pb, 1);
}

test "roundtrip noise 8-bit rgba single group" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 40, 40, .rgb, true, 8);
    defer pb.deinit();
    fillNoise(&pb, 0xDEADBEEF);
    try roundtrip(alloc, &pb, 1);
}

test "roundtrip gradient 16-bit gray" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 32, 32, .gray, false, 16);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtrip(alloc, &pb, 1);
}

test "roundtrip noise 16-bit rgb" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 48, 24, .rgb, false, 16);
    defer pb.deinit();
    fillNoise(&pb, 42);
    try roundtrip(alloc, &pb, 1);
}

test "roundtrip noise 16-bit rgba multi group" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 257, 100, .rgb, true, 16);
    defer pb.deinit();
    fillNoise(&pb, 7);
    try roundtrip(alloc, &pb, 1);
}

test "roundtrip gradient 8-bit rgb multi group" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 300, 30, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtrip(alloc, &pb, 1);
}

test "roundtrip noise 8-bit gray multi group tall" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 100, 300, .gray, false, 8);
    defer pb.deinit();
    fillNoise(&pb, 99);
    try roundtrip(alloc, &pb, 0);
}

fn fillFewColors(pb: *PixelBuffer, seed: u64) void {
    const palette = [_]u16{ 0, 0xFFFF, 0xF800, 0x07E0, 0x001F, 0x7BEF };
    var prng = std.Random.DefaultPrng.init(seed);
    const rand = prng.random();
    const nch = pb.numChannels();
    for (0..pb.ysize) |y| {
        for (0..pb.xsize) |x| {
            const c = palette[rand.uintLessThan(usize, palette.len)];
            const scaled: u16 = if (pb.bitdepth == 16) c else @intCast((@as(u32, c) * 255) / 0xFFFF);
            for (0..nch) |k| pb.set(x, y, k, scaled);
        }
    }
}

fn roundtripOpts(allocator: Allocator, pb: *const PixelBuffer, opts: encoder.EncodeOptions) !void {
    var w = BitWriter.init(allocator);
    defer w.deinit();
    try encoder.encode(allocator, pb, opts, &w);
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer allocator.free(bytes);

    var got = try decoder.decode(allocator, bytes, .{});
    defer got.deinit();
    try std.testing.expect(got == .int);
    try std.testing.expect(pb.eql(&got.int));
}

test "roundtrip few-colors 8-bit rgb (palette transform)" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 70, 50, .rgb, false, 8);
    defer pb.deinit();
    fillFewColors(&pb, 5);
    try roundtripOpts(alloc, &pb, .{});
}

test "roundtrip few-colors rgba multi group (group palettes)" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 300, 130, .rgb, true, 8);
    defer pb.deinit();
    fillFewColors(&pb, 6);
    try roundtripOpts(alloc, &pb, .{});
}

test "roundtrip gradient with fixed colorspace RCT" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 120, 90, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtripOpts(alloc, &pb, .{ .colorspace = 6 });
    try roundtripOpts(alloc, &pb, .{ .colorspace = 2 });
}

test "roundtrip responsive (squeeze) gradient single group" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 200, 150, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtripOpts(alloc, &pb, .{ .responsive = true });
}

test "roundtrip responsive noise multi group" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 400, 260, .rgb, false, 8);
    defer pb.deinit();
    fillNoise(&pb, 1234);
    try roundtripOpts(alloc, &pb, .{ .responsive = true });
}

test "roundtrip responsive odd size with small groups" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 257, 129, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtripOpts(alloc, &pb, .{ .responsive = true, .group_size_shift = 0 });
    try roundtripOpts(alloc, &pb, .{ .responsive = true, .group_size_shift = 3 });
}

test "roundtrip no-palette path still works" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 90, 60, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtripOpts(alloc, &pb, .{ .palette_colors = 0 });
}

// --- Float samples (PFM-style fp32 / fp16) ---

const FloatPixelBuffer = pixel_mod.FloatPixelBuffer;

fn fillFloatRange(fb: *FloatPixelBuffer) void {
    // Values across sign and magnitude: negatives (HDR), subnormal-ish
    // fractions and values > 1 (outside [0,1]) must all survive the
    // custom-float bit-pattern roundtrip exactly.
    const nch = fb.numChannels();
    var i: usize = 0;
    for (0..fb.ysize) |y| {
        for (0..fb.xsize) |x| {
            for (0..nch) |c| {
                const t: f32 = @floatFromInt(i);
                const v: f32 = switch (i % 5) {
                    0 => t * 0.03125,
                    1 => -t * 0.0625 - 0.5,
                    2 => 1.0 + t * 0.25,
                    3 => 0.0,
                    else => -3.0 + t * 0.5,
                };
                fb.set(x, y, c, v);
                i += 1;
            }
        }
    }
}

fn roundtripFloat(allocator: Allocator, fb: *const FloatPixelBuffer, opts: encoder.EncodeOptions) !void {
    var w = BitWriter.init(allocator);
    defer w.deinit();
    try encoder.encodeFloat(allocator, fb, opts, &w);
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer allocator.free(bytes);

    var got = try decoder.decode(allocator, bytes, .{});
    defer got.deinit();
    try std.testing.expect(got == .float);
    // The decoded buffer reports the *encoded* format (fp16 here), so
    // compare the geometry and the raw sample values, not the metadata.
    if (opts.float16) {
        try std.testing.expectEqual(fb.xsize, got.float.xsize);
        try std.testing.expectEqual(fb.ysize, got.float.ysize);
        try std.testing.expectEqual(fb.layout, got.float.layout);
        try std.testing.expectEqual(@as(u8, 16), got.float.bits_per_sample);
        try std.testing.expectEqual(@as(u8, 5), got.float.exponent_bits_per_sample);
        try std.testing.expect(std.mem.eql(f32, fb.data, got.float.data));
        return;
    }
    try std.testing.expect(fb.eql(&got.float));
}

test "roundtrip float fp32 rgb" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(alloc, 37, 23, .rgb, 32, 8);
    defer fb.deinit();
    fillFloatRange(&fb);
    try roundtripFloat(alloc, &fb, .{});
}

test "roundtrip float fp32 gray multi group" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(alloc, 300, 120, .gray, 32, 8);
    defer fb.deinit();
    fillFloatRange(&fb);
    try roundtripFloat(alloc, &fb, .{});
}

test "roundtrip float fp16" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(alloc, 33, 21, .rgb, 32, 8);
    defer fb.deinit();
    // Only fp16-representable values survive the binary16 bit pattern.
    const vals = [_]f32{ 0.0, 0.25, -0.5, 1.0, -2.0, 0.0078125, 4.0, -0.0078125 };
    var i: usize = 0;
    for (0..fb.ysize) |y| {
        for (0..fb.xsize) |x| {
            for (0..fb.numChannels()) |c| {
                fb.set(x, y, c, vals[i % vals.len]);
                i += 1;
            }
        }
    }
    try roundtripFloat(alloc, &fb, .{ .float16 = true });
}

test "roundtrip float fp32 responsive" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(alloc, 180, 140, .rgb, 32, 8);
    defer fb.deinit();
    fillFloatRange(&fb);
    try roundtripFloat(alloc, &fb, .{ .responsive = true });
}

test "roundtrip progressive single group" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 200, 150, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtripOpts(alloc, &pb, .{ .progressive = true });
}

test "roundtrip progressive multi group" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 600, 400, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtripOpts(alloc, &pb, .{ .progressive = true });
    try roundtripOpts(alloc, &pb, .{ .progressive = true, .group_size_shift = 0 });
    try roundtripOpts(alloc, &pb, .{ .progressive = true, .group_size_shift = 3 });
}

test "roundtrip progressive noise multi group" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 400, 260, .rgb, false, 8);
    defer pb.deinit();
    fillNoise(&pb, 4242);
    try roundtripOpts(alloc, &pb, .{ .progressive = true });
}

test "roundtrip progressive odd size" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 257, 129, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtripOpts(alloc, &pb, .{ .progressive = true, .group_size_shift = 0 });
}

test "roundtrip progressive 16-bit gray" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 300, 200, .gray, false, 16);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtripOpts(alloc, &pb, .{ .progressive = true });
}

test "roundtrip progressive rgba" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 320, 200, .rgb, true, 8);
    defer pb.deinit();
    fillFewColors(&pb, 99);
    try roundtripOpts(alloc, &pb, .{ .progressive = true });
}

test "roundtrip progressive large (multi DC group)" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 2100, 300, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    // 2100px wide spans two 2048 DC groups: the shift>=3 squeeze channels
    // travel in DC group sections.
    try roundtripOpts(alloc, &pb, .{ .progressive = true });
}

// ---------------------------------------------------------------------------
// Effort ladder (rounds 8): cjxl -e 1..10 equivalents — the faster tiers'
// fixed trees, the slower tiers' palette reverts / WP-mode search / wider
// property sets — and --group-order TOC permutations, all through the
// real encode -> decode path.
// ---------------------------------------------------------------------------

test "roundtrip effort ladder 1-10 noise multi group" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 300, 260, .rgb, false, 8);
    defer pb.deinit();
    fillNoise(&pb, 1234);
    for ([_]u32{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 }) |effort| {
        try roundtripOpts(alloc, &pb, .{ .effort = effort });
    }
}

test "roundtrip effort ladder gradient few colors" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 500, 300, .rgb, false, 8);
    defer pb.deinit();
    fillFewColors(&pb, 40);
    for ([_]u32{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 }) |effort| {
        try roundtripOpts(alloc, &pb, .{ .effort = effort });
    }
}

test "roundtrip effort 8-10 16-bit alpha" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 280, 190, .rgb, true, 16);
    defer pb.deinit();
    fillNoise(&pb, 77);
    for ([_]u32{ 8, 9, 10 }) |effort| {
        try roundtripOpts(alloc, &pb, .{ .effort = effort });
    }
}

test "roundtrip effort 9 responsive and progressive" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 400, 240, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtripOpts(alloc, &pb, .{ .effort = 9, .responsive = true });
    try roundtripOpts(alloc, &pb, .{ .effort = 8, .progressive = true });
}

test "roundtrip group order multi group" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 700, 500, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtripOpts(alloc, &pb, .{ .group_order = true });
    try roundtripOpts(alloc, &pb, .{ .group_order = true, .group_size_shift = 0 });
}

test "roundtrip group order noise single pass multi dc" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 2200, 300, .rgb, false, 8);
    defer pb.deinit();
    fillNoise(&pb, 555);
    try roundtripOpts(alloc, &pb, .{ .group_order = true });
}

test "roundtrip group order progressive 2-pass" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 600, 400, .rgb, false, 8);
    defer pb.deinit();
    fillGradient(&pb);
    try roundtripOpts(alloc, &pb, .{ .progressive = true, .group_order = true });
}

test "roundtrip group order + effort 9" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 512, 384, .rgb, false, 8);
    defer pb.deinit();
    fillNoise(&pb, 999);
    try roundtripOpts(alloc, &pb, .{ .group_order = true, .effort = 9 });
}
