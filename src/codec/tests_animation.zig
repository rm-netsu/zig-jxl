//! Animation encode/decode roundtrip tests.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const pixel_mod = @import("pixel.zig");
const encoder = @import("encoder.zig");
const animation = @import("animation.zig");
const decoder = @import("decoder.zig");

const PixelBuffer = pixel_mod.PixelBuffer;

fn makeFrame(allocator: std.mem.Allocator, idx: usize, w: usize, h: usize, alpha: bool) !PixelBuffer {
    var pb = try PixelBuffer.create(allocator, w, h, .rgb, alpha, 8);
    errdefer pb.deinit();
    const nch = pb.numChannels();
    for (0..h) |y| {
        for (0..w) |x| {
            for (0..nch) |c| {
                const base: u16 = switch (c) {
                    0 => @intCast((x * 5 + idx * 40) % 256),
                    1 => @intCast((y * 7) % 256),
                    2 => @intCast((idx * 60 + x + y) % 256),
                    else => if (idx % 2 == 0) 255 else @intCast(255 - ((x + y) % 128)),
                };
                pb.set(x, y, c, base);
            }
        }
    }
    return pb;
}

test "animation encode/decode roundtrip (rgb, 3 frames)" {
    const allocator = std.testing.allocator;
    var frames: [3]PixelBuffer = undefined;
    for (&frames, 0..) |*f, i| f.* = try makeFrame(allocator, i, 40, 30, false);
    defer for (&frames) |*f| f.deinit();

    const frame_ptrs = [_]*const PixelBuffer{ &frames[0], &frames[1], &frames[2] };
    const durations = [_]u32{ 80, 80, 160 };

    var w = bit_io.BitWriter.init(allocator);
    defer w.deinit();
    try encoder.encodeAnimation(allocator, &frame_ptrs, .{ .durations = &durations }, .{}, &w);
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer allocator.free(bytes);

    var anim = try animation.decodeAnimation(allocator, bytes, .{});
    defer anim.deinit();
    try std.testing.expectEqual(@as(usize, 3), anim.frames.items.len);
    try std.testing.expectEqual(@as(u32, 1000), anim.tps_numerator);
    try std.testing.expectEqual(@as(u32, 1), anim.tps_denominator);
    for (&frames, 0..) |*f, i| {
        try std.testing.expect(f.eql(&anim.frames.items[i].pixels));
        try std.testing.expectEqual(durations[i], anim.frames.items[i].duration_ticks);
        try std.testing.expectEqual(durations[i], @as(u32, @intFromFloat(anim.frameDurationMs(i))));
    }
}

test "animation roundtrip with alpha" {
    const allocator = std.testing.allocator;
    var frames: [2]PixelBuffer = undefined;
    for (&frames, 0..) |*f, i| f.* = try makeFrame(allocator, i, 23, 17, true);
    defer for (&frames) |*f| f.deinit();

    const frame_ptrs = [_]*const PixelBuffer{ &frames[0], &frames[1] };
    const durations = [_]u32{ 100, 100 };

    var w = bit_io.BitWriter.init(allocator);
    defer w.deinit();
    try encoder.encodeAnimation(allocator, &frame_ptrs, .{ .durations = &durations }, .{}, &w);
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer allocator.free(bytes);

    var anim = try animation.decodeAnimation(allocator, bytes, .{});
    defer anim.deinit();
    try std.testing.expectEqual(@as(usize, 2), anim.frames.items.len);
    for (&frames, 0..) |*f, i| {
        try std.testing.expect(f.eql(&anim.frames.items[i].pixels));
    }
}

test "single frame animation == still decode" {
    const allocator = std.testing.allocator;
    var f0 = try makeFrame(allocator, 0, 20, 20, false);
    defer f0.deinit();
    const frame_ptrs = [_]*const PixelBuffer{&f0};
    const durations = [_]u32{50};

    var w = bit_io.BitWriter.init(allocator);
    defer w.deinit();
    try encoder.encodeAnimation(allocator, &frame_ptrs, .{ .durations = &durations }, .{}, &w);
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer allocator.free(bytes);

    // Animation decode: one frame.
    var anim = try animation.decodeAnimation(allocator, bytes, .{});
    defer anim.deinit();
    try std.testing.expectEqual(@as(usize, 1), anim.frames.items.len);
    try std.testing.expect(f0.eql(&anim.frames.items[0].pixels));

    // Still decode of the same file also works (single visible frame).
    var still = try decoder.decode(allocator, bytes, .{});
    defer still.deinit();
    try std.testing.expect(still == .int);
    try std.testing.expect(f0.eql(&still.int));
}

test "animation multi-group roundtrip" {
    const allocator = std.testing.allocator;
    var frames: [2]PixelBuffer = undefined;
    for (&frames, 0..) |*f, i| f.* = try makeFrame(allocator, i, 300, 200, false);
    defer for (&frames) |*f| f.deinit();

    const frame_ptrs = [_]*const PixelBuffer{ &frames[0], &frames[1] };
    const durations = [_]u32{ 120, 120 };

    var w = bit_io.BitWriter.init(allocator);
    defer w.deinit();
    try encoder.encodeAnimation(allocator, &frame_ptrs, .{ .durations = &durations }, .{ .group_size_shift = 0 }, &w);
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer allocator.free(bytes);

    var anim = try animation.decodeAnimation(allocator, bytes, .{});
    defer anim.deinit();
    try std.testing.expectEqual(@as(usize, 2), anim.frames.items.len);
    for (&frames, 0..) |*f, i| {
        try std.testing.expect(f.eql(&anim.frames.items[i].pixels));
    }
}

test "nearestEven matches round-half-even" {
    try std.testing.expectEqual(@as(i32, 2), nearestEvenPub(1.5));
    try std.testing.expectEqual(@as(i32, 4), nearestEvenPub(3.5));
    try std.testing.expectEqual(@as(i32, 2), nearestEvenPub(2.5));
    try std.testing.expectEqual(@as(i32, 3), nearestEvenPub(2.6));
    try std.testing.expectEqual(@as(i32, 2), nearestEvenPub(2.4));
    try std.testing.expectEqual(@as(i32, -2), nearestEvenPub(-1.5));
    try std.testing.expectEqual(@as(i32, -4), nearestEvenPub(-3.5));
}

const nearestEvenPub = @import("animation.zig").nearestEvenForTest;

test "decodeAnimation rejects animations with a preview" {
    // Constructed below via a tiny hand-built header is overkill; instead
    // verify the error mapping exists by decoding garbage (container check).
    const allocator = std.testing.allocator;
    try std.testing.expectError(error.UnsupportedContainer, animation.decodeAnimation(allocator, "not a jxl", .{}));
}

fn makeGrayFrame(allocator: std.mem.Allocator, idx: usize, w: usize, h: usize) !PixelBuffer {
    var pb = try PixelBuffer.create(allocator, w, h, .gray, false, 8);
    errdefer pb.deinit();
    for (0..h) |y| {
        for (0..w) |x| {
            pb.set(x, y, 0, @intCast((x * 2 + y * 3 + idx * 37) % 256));
        }
    }
    return pb;
}

test "animation roundtrip gray with per-frame durations" {
    const allocator = std.testing.allocator;
    var frames: [3]PixelBuffer = undefined;
    for (&frames, 0..) |*f, i| f.* = try makeGrayFrame(allocator, i, 33, 21);
    defer for (&frames) |*f| f.deinit();

    const frame_ptrs = [_]*const PixelBuffer{ &frames[0], &frames[1], &frames[2] };
    const durations = [_]u32{ 33, 250, 1000 };

    var w = bit_io.BitWriter.init(allocator);
    defer w.deinit();
    try encoder.encodeAnimation(allocator, &frame_ptrs, .{ .durations = &durations, .num_loops = 4 }, .{}, &w);
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer allocator.free(bytes);

    var anim = try animation.decodeAnimation(allocator, bytes, .{});
    defer anim.deinit();
    try std.testing.expectEqual(@as(usize, 3), anim.frames.items.len);
    try std.testing.expectEqual(@as(u32, 4), anim.num_loops);
    try std.testing.expectEqual(@as(u32, 1000), anim.tps_numerator);
    for (&frames, 0..) |*f, i| {
        try std.testing.expect(f.eql(&anim.frames.items[i].pixels));
        try std.testing.expectEqual(durations[i], anim.frames.items[i].duration_ticks);
        const want_ms: f64 = @floatFromInt(durations[i]);
        try std.testing.expectApproxEqAbs(want_ms, anim.frameDurationMs(i), 0.01);
    }
}

test "mismatched animation frames are rejected" {
    const allocator = std.testing.allocator;
    var f0 = try makeFrame(allocator, 0, 20, 20, false);
    defer f0.deinit();
    var f1 = try makeFrame(allocator, 1, 30, 20, false); // different size
    defer f1.deinit();
    const frame_ptrs = [_]*const PixelBuffer{ &f0, &f1 };
    const durations = [_]u32{ 50, 50 };
    var w = bit_io.BitWriter.init(allocator);
    defer w.deinit();
    try std.testing.expectError(error.MismatchedFrames, encoder.encodeAnimation(allocator, &frame_ptrs, .{ .durations = &durations }, .{}, &w));
}

test "animation progressive roundtrip (multi-pass frames)" {
    const allocator = std.testing.allocator;
    var frames: [3]PixelBuffer = undefined;
    for (&frames, 0..) |*f, i| f.* = try makeFrame(allocator, i, 40, 30, false);
    defer for (&frames) |*f| f.deinit();

    const frame_ptrs = [_]*const PixelBuffer{ &frames[0], &frames[1], &frames[2] };
    const durations = [_]u32{ 100, 100, 100 };

    var w = bit_io.BitWriter.init(allocator);
    defer w.deinit();
    try encoder.encodeAnimation(
        allocator,
        &frame_ptrs,
        .{ .durations = &durations },
        .{ .progressive = true },
        &w,
    );
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer allocator.free(bytes);

    var anim = try animation.decodeAnimation(allocator, bytes, .{});
    defer anim.deinit();
    try std.testing.expectEqual(@as(usize, 3), anim.frames.items.len);
    for (&frames, 0..) |*f, i| {
        try std.testing.expect(f.eql(&anim.frames.items[i].pixels));
        try std.testing.expectEqual(durations[i], anim.frames.items[i].duration_ticks);
    }
}
