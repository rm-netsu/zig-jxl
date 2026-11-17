//! Lossy VarDCT (XYB patch subset) roundtrip and codestream-shape tests.

const std = @import("std");
const pixel_mod = @import("pixel.zig");
const encoder = @import("encoder.zig");
const decoder = @import("decoder.zig");
const bit_io = @import("../base/bit_io.zig");
const container_mod = @import("../header/container.zig");
const size_mod = @import("../header/size.zig");
const metadata_mod = @import("../header/metadata.zig");
const frame_mod = @import("../header/frame.zig");
const toc_mod = @import("../header/toc.zig");
const stream = @import("../modular/stream.zig");

const Allocator = std.mem.Allocator;
const PixelBuffer = pixel_mod.PixelBuffer;
const BitWriter = bit_io.BitWriter;

/// The classic patch corpus shape: uniform light page, repeated identical
/// "glyph" connected components (>= 20 pixels, quantized XYB peaks).
fn makeTextImage(allocator: Allocator, w: usize, h: usize) !PixelBuffer {
    var pb = try PixelBuffer.create(allocator, w, h, .rgb, false, 8);
    errdefer pb.deinit();
    for (pb.data) |*v| v.* = 200;
    const glyph = [_][]const u8{
        &.{ 1, 1, 1, 1, 1, 1 },
        &.{ 0, 0, 1, 1, 0, 0 },
        &.{ 0, 0, 1, 1, 0, 0 },
        &.{ 0, 0, 1, 1, 0, 0 },
        &.{ 0, 0, 1, 1, 0, 0 },
        &.{ 0, 0, 1, 1, 0, 0 },
        &.{ 0, 0, 1, 1, 0, 0 },
        &.{ 0, 0, 1, 1, 0, 0 },
    };
    var gy: usize = 1;
    while (gy * 16 + 8 < h) : (gy += 1) {
        var gx: usize = 1;
        while (gx * 24 + 6 < w) : (gx += 1) {
            for (glyph, 0..) |row, j| {
                for (row, 0..) |ink, i| {
                    if (ink == 0) continue;
                    const x = gx * 24 + i;
                    const y = gy * 16 + j;
                    pb.set(x, y, 0, 30);
                    pb.set(x, y, 1, 25);
                    pb.set(x, y, 2, 45);
                }
            }
        }
    }
    return pb;
}

fn encodeLossyBytes(allocator: Allocator, pb: *const PixelBuffer, distance: f32, patches: anytype) ![]u8 {
    var w = BitWriter.init(allocator);
    defer w.deinit();
    try encoder.encodeLossy(allocator, pb, .{
        .distance = distance,
        .patches = patches,
        .effort = 7,
    }, &w);
    w.zeroPadToByte();
    return w.takeBytes();
}

/// Walks the frame headers of a codestream and returns their types.
const FrameInfo = struct {
    frame_type: frame_mod.FrameType,
    encoding: frame_mod.FrameEncoding,
    flags: u64,
    save_as_reference: u32,
    color_transform: frame_mod.ColorTransform,
};

fn frameInfos(allocator: Allocator, bytes: []const u8) !std.ArrayList(FrameInfo) {
    var br = bit_io.BitReader.init(bytes);
    _ = br.readFixedBits(16);
    var sh: size_mod.SizeHeader = .{};
    try sh.read(&br);
    var metadata = metadata_mod.ImageMetadata.init(allocator);
    defer metadata.deinit();
    try metadata.read(&br);
    var ctd = metadata_mod.CustomTransformData{};
    try ctd.read(&br, metadata.xyb_encoded);
    if (metadata.color_encoding.want_icc) {
        const icc = try @import("../header/icc.zig").readICC(allocator, &br);
        allocator.free(icc);
    }
    try br.jumpToByteBoundary();

    var out: std.ArrayList(FrameInfo) = .empty;
    errdefer out.deinit(allocator);
    var done = false;
    while (!done) {
        var frame = frame_mod.FrameHeader.init(allocator);
        defer frame.deinit();
        try frame.read(&metadata, &br);
        try out.append(allocator, .{
            .frame_type = frame.frame_type,
            .encoding = frame.encoding,
            .flags = frame.flags,
            .save_as_reference = frame.save_as_reference,
            .color_transform = frame.color_transform,
        });
        const fd = stream.FrameDim.init(sh.xsizeOf(), sh.ysizeOf(), frame.group_size_shift, 1);
        const num_toc = toc_mod.numTocEntries(fd.num_groups, fd.num_dc_groups, frame.passes.num_passes);
        const toc_sizes = try toc_mod.readToc(allocator, num_toc, &br);
        defer allocator.free(toc_sizes);
        var total: usize = 0;
        for (toc_sizes) |s| total += s;
        _ = br.skipBits(total * 8);
        done = frame.is_last;
    }
    return out;
}

test "lossy roundtrip: gradient decodes with reasonable quality" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 64, 48, .rgb, false, 8);
    defer pb.deinit();
    for (0..pb.ysize) |y| {
        for (0..pb.xsize) |x| {
            pb.set(x, y, 0, @intCast((x * 255) / 63));
            pb.set(x, y, 1, @intCast((y * 255) / 47));
            pb.set(x, y, 2, @intCast(((x + y) * 255) / 110));
        }
    }
    const bytes = try encodeLossyBytes(alloc, &pb, 1.0, @import("../patches/enc_patches.zig").PatchOverride.off);
    defer alloc.free(bytes);

    var got = try decoder.decode(alloc, bytes, .{});
    defer got.deinit();
    try std.testing.expect(got == .int);
    try std.testing.expectEqual(@as(usize, 64), got.int.xsize);
    try std.testing.expectEqual(@as(usize, 48), got.int.ysize);
    // Smooth gradient at d=1: mean abs error stays small.
    var sum: usize = 0;
    for (0..pb.ysize) |y| {
        for (0..pb.xsize) |x| {
            for (0..3) |c| {
                const a: usize = @intCast(pb.at(x, y, c));
                const b: usize = @intCast(got.int.at(x, y, c));
                sum += if (a > b) a - b else b - a;
            }
        }
    }
    const mean = @as(f64, @floatFromInt(sum)) / @as(f64, @floatFromInt(64 * 48 * 3));
    try std.testing.expect(mean < 4.0);
}

test "lossy patch roundtrip: glyph pixels survive nearly exactly" {
    const alloc = std.testing.allocator;
    var pb = try makeTextImage(alloc, 128, 96);
    defer pb.deinit();
    const bytes = try encodeLossyBytes(alloc, &pb, 1.0, @import("../patches/enc_patches.zig").PatchOverride.on);
    defer alloc.free(bytes);

    var got = try decoder.decode(alloc, bytes, .{});
    defer got.deinit();
    try std.testing.expect(got == .int);

    // The patch content rides in the reference frame quantized onto the
    // enc_factors grid (XYB steps ~1/273 at d=1), so glyph pixels come
    // back well within a few 8-bit steps of the original.
    var max_err: usize = 0;
    for (0..pb.ysize) |y| {
        for (0..pb.xsize) |x| {
            for (0..3) |c| {
                const a: usize = @intCast(pb.at(x, y, c));
                const b: usize = @intCast(got.int.at(x, y, c));
                const d = if (a > b) a - b else b - a;
                if (d > max_err) max_err = d;
            }
        }
    }
    try std.testing.expect(max_err <= 8);
}

test "lossy patches: codestream shape (XYB reference frame + kPatches VarDCT)" {
    const alloc = std.testing.allocator;
    var pb = try makeTextImage(alloc, 128, 96);
    defer pb.deinit();
    const bytes = try encodeLossyBytes(alloc, &pb, 1.0, @import("../patches/enc_patches.zig").PatchOverride.on);
    defer alloc.free(bytes);

    var infos = try frameInfos(alloc, bytes);
    defer infos.deinit(alloc);
    try std.testing.expectEqual(@as(usize, 2), infos.items.len);
    // Frame 0: the patch reference (modular, XYB, slot 3).
    try std.testing.expectEqual(frame_mod.FrameType.reference_only, infos.items[0].frame_type);
    try std.testing.expectEqual(frame_mod.FrameEncoding.modular, infos.items[0].encoding);
    try std.testing.expectEqual(frame_mod.ColorTransform.xyb, infos.items[0].color_transform);
    try std.testing.expectEqual(@as(u32, 3), infos.items[0].save_as_reference);
    // Frame 1: the VarDCT main frame carrying kPatches.
    try std.testing.expectEqual(frame_mod.FrameType.regular, infos.items[1].frame_type);
    try std.testing.expectEqual(frame_mod.FrameEncoding.var_dct, infos.items[1].encoding);
    try std.testing.expectEqual(frame_mod.ColorTransform.xyb, infos.items[1].color_transform);
    try std.testing.expect(infos.items[1].flags & frame_mod.Flags.kPatches != 0);
    try std.testing.expect(infos.items[1].flags & frame_mod.Flags.kSkipAdaptiveDCSmoothing == 0);

    // ... and without patches the codestream is a single VarDCT frame.
    const bytes_np = try encodeLossyBytes(alloc, &pb, 1.0, @import("../patches/enc_patches.zig").PatchOverride.off);
    defer alloc.free(bytes_np);
    var infos_np = try frameInfos(alloc, bytes_np);
    defer infos_np.deinit(alloc);
    try std.testing.expectEqual(@as(usize, 1), infos_np.items.len);
    try std.testing.expectEqual(frame_mod.FrameEncoding.var_dct, infos_np.items[0].encoding);
    try std.testing.expect(infos_np.items[0].flags & frame_mod.Flags.kPatches == 0);
}

test "lossy patches shrink text-like images" {
    const alloc = std.testing.allocator;
    var pb = try makeTextImage(alloc, 256, 192);
    defer pb.deinit();
    const with_patches = try encodeLossyBytes(alloc, &pb, 1.0, @import("../patches/enc_patches.zig").PatchOverride.on);
    defer alloc.free(with_patches);
    const without = try encodeLossyBytes(alloc, &pb, 1.0, @import("../patches/enc_patches.zig").PatchOverride.off);
    defer alloc.free(without);
    try std.testing.expect(with_patches.len * 3 < without.len);
}

test "lossy: gray and alpha inputs rejected" {
    const alloc = std.testing.allocator;
    var gray = try PixelBuffer.create(alloc, 32, 32, .gray, false, 8);
    defer gray.deinit();
    var w = BitWriter.init(alloc);
    defer w.deinit();
    const r1 = encoder.encodeLossy(alloc, &gray, .{ .distance = 1 }, &w);
    try std.testing.expectError(error.UnsupportedLossyInput, r1);

    var rgba = try PixelBuffer.create(alloc, 32, 32, .rgb, true, 8);
    defer rgba.deinit();
    const r2 = encoder.encodeLossy(alloc, &rgba, .{ .distance = 1 }, &w);
    try std.testing.expectError(error.UnsupportedLossyInput, r2);

    var rgb = try PixelBuffer.create(alloc, 32, 32, .rgb, false, 8);
    defer rgb.deinit();
    const r3 = encoder.encodeLossy(alloc, &rgb, .{ .distance = 0 }, &w);
    try std.testing.expectError(error.InvalidDistance, r3);
}

test "lossy: distance ladder monotonicity (smaller distance -> bigger file)" {
    const alloc = std.testing.allocator;
    var pb = try makeTextImage(alloc, 128, 96);
    defer pb.deinit();
    const b_half = try encodeLossyBytes(alloc, &pb, 0.5, @import("../patches/enc_patches.zig").PatchOverride.off);
    defer alloc.free(b_half);
    const b_one = try encodeLossyBytes(alloc, &pb, 1.0, @import("../patches/enc_patches.zig").PatchOverride.off);
    defer alloc.free(b_one);
    const b_three = try encodeLossyBytes(alloc, &pb, 3.0, @import("../patches/enc_patches.zig").PatchOverride.off);
    defer alloc.free(b_three);
    try std.testing.expect(b_half.len >= b_one.len);
    try std.testing.expect(b_one.len >= b_three.len);
}

test "lossy: linear float decode output" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 40, 24, .rgb, false, 8);
    defer pb.deinit();
    for (pb.data) |*v| v.* = 128;
    const bytes = try encodeLossyBytes(alloc, &pb, 1.0, @import("../patches/enc_patches.zig").PatchOverride.off);
    defer alloc.free(bytes);
    var got = try decoder.decode(alloc, bytes, .{ .prefer_linear_float = true });
    defer got.deinit();
    try std.testing.expect(got == .float);
    try std.testing.expectEqual(@as(usize, 40), got.float.xsize);
    // A mid-gray constant decodes to a mid-gray linear constant.
    const v = got.float.at(10, 10, 1);
    try std.testing.expect(v > 0.1 and v < 0.4);
}
