//! Parse -> reconstruct roundtrip tests on real JPEG files.

const std = @import("std");
const jd = @import("jpeg_data.zig");
const reader = @import("reader.zig");
const writer = @import("writer.zig");
const transcode = @import("transcode.zig");

fn roundtrip(comptime name: []const u8) !void {
    const data = @embedFile("testdata/" ++ name ++ ".jpg");
    const alloc = std.testing.allocator;
    var jpg = jd.JPEGData{};
    defer jpg.deinit(alloc);
    try reader.readJpeg(data, .read_all, &jpg, alloc);
    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(alloc);
    try writer.writeJpeg(&jpg, &out, alloc);
    try std.testing.expectEqualSlices(u8, data, out.items);
}

/// Full JPEG -> JXL container -> JPEG transcode roundtrip.
fn transcodeRoundtrip(comptime name: []const u8) !void {
    const data = @embedFile("testdata/" ++ name ++ ".jpg");
    const alloc = std.testing.allocator;
    const jxl = try transcode.encodeTranscode(alloc, data);
    defer alloc.free(jxl);
    const jpeg = try transcode.decodeTranscode(alloc, jxl);
    defer alloc.free(jpeg);
    try std.testing.expectEqualSlices(u8, data, jpeg);
}

test "baseline 4:4:4 roundtrip" {
    try roundtrip("baseline_444");
}

test "baseline 4:2:0 roundtrip" {
    try roundtrip("baseline_420");
}

test "baseline 4:2:2 roundtrip" {
    try roundtrip("baseline_422");
}

test "gray baseline roundtrip" {
    try roundtrip("gray_baseline");
}

test "progressive 4:4:4 roundtrip" {
    try roundtrip("progressive_444");
}

test "progressive 4:2:0 roundtrip" {
    try roundtrip("progressive_420");
}

test "gray progressive roundtrip" {
    try roundtrip("gray_progressive");
}

test "low quality baseline roundtrip" {
    try roundtrip("baseline_q30");
}

test "high quality baseline roundtrip" {
    try roundtrip("baseline_q98_444");
}

test "restart intervals 4:2:0 roundtrip" {
    try roundtrip("restart_420");
}

test "restart intervals gray roundtrip" {
    try roundtrip("restart_gray");
}

test "icc profile roundtrip" {
    try roundtrip("icc_444");
}

test "exif roundtrip" {
    try roundtrip("exif_420");
}

test "transcode baseline 4:4:4" {
    try transcodeRoundtrip("baseline_444");
}

test "transcode baseline 4:2:0" {
    try transcodeRoundtrip("baseline_420");
}

test "transcode gray baseline" {
    try transcodeRoundtrip("gray_baseline");
}

test "transcode progressive 4:4:4" {
    try transcodeRoundtrip("progressive_444");
}

test "transcode progressive 4:2:0" {
    try transcodeRoundtrip("progressive_420");
}

test "transcode gray progressive" {
    try transcodeRoundtrip("gray_progressive");
}

test "transcode restart 4:2:0" {
    try transcodeRoundtrip("restart_420");
}

test "transcode icc profile" {
    try transcodeRoundtrip("icc_444");
}

test "transcode exif" {
    try transcodeRoundtrip("exif_420");
}

test "transcode low quality" {
    try transcodeRoundtrip("baseline_q30");
}

test "transcode high quality" {
    try transcodeRoundtrip("baseline_q98_444");
}

test "transcode baseline 4:2:2" {
    try transcodeRoundtrip("baseline_422");
}

test "transcode restart gray" {
    try transcodeRoundtrip("restart_gray");
}

test "transcode multi-group 4:4:4" {
    try transcodeRoundtrip("big_444");
}

test "transcode multi-group 4:2:0" {
    try transcodeRoundtrip("big_420");
}

test "transcode multi-group 4:2:2" {
    try transcodeRoundtrip("big_422");
}

test "transcode multi-group gray" {
    try transcodeRoundtrip("big_gray");
}

test "transcode multi-DC-group 4:4:4" {
    try transcodeRoundtrip("wide_444");
}

test "transcode multi-DC-group 4:2:0" {
    try transcodeRoundtrip("wide_420");
}

test "transcode two quant tables" {
    try transcodeRoundtrip("two_dqt_444");
}

/// Transcode roundtrip with an explicit option combination; returns the
/// encoded container size.
fn transcodeVariant(comptime name: []const u8, opts: transcode.EncodeOptions) !usize {
    const data = @embedFile("testdata/" ++ name ++ ".jpg");
    const alloc = std.testing.allocator;
    const jxl = try transcode.encodeTranscodeOptions(alloc, data, opts);
    defer alloc.free(jxl);
    const jpeg = try transcode.decodeTranscode(alloc, jxl);
    defer alloc.free(jpeg);
    try std.testing.expectEqualSlices(u8, data, jpeg);
    return jxl.len;
}

test "transcode cfl and coeff-order variants roundtrip (444)" {
    // All four option combinations reconstruct byte-exactly; CfL engages
    // on this chroma-correlated 444 file (smaller than the no-CfL variant).
    const none = try transcodeVariant("baseline_q98_444", .{ .cfl = false, .coeff_orders = false });
    const cfl = try transcodeVariant("baseline_q98_444", .{ .cfl = true, .coeff_orders = false });
    const ord = try transcodeVariant("baseline_q98_444", .{ .cfl = false, .coeff_orders = true });
    const both = try transcodeVariant("baseline_q98_444", .{});
    try std.testing.expect(cfl < none);
    try std.testing.expect(both < ord);
    try std.testing.expect(both < none);
}

test "transcode cfl and coeff-order variants roundtrip (420 + gray)" {
    // CfL is inactive for subsampled/gray frames (maps stay zero) but the
    // files must still roundtrip byte-exactly with every option vector.
    _ = try transcodeVariant("big_420", .{ .cfl = false, .coeff_orders = false });
    _ = try transcodeVariant("big_420", .{});
    _ = try transcodeVariant("gray_baseline", .{ .cfl = false, .coeff_orders = false });
    _ = try transcodeVariant("gray_baseline", .{});
    _ = try transcodeVariant("restart_420", .{ .cfl = false, .coeff_orders = false });
    _ = try transcodeVariant("restart_420", .{});
    _ = try transcodeVariant("two_dqt_444", .{});
    _ = try transcodeVariant("wide_444", .{});
    _ = try transcodeVariant("progressive_444", .{});
}

test "parsed structure sanity" {
    const alloc = std.testing.allocator;
    const data = @embedFile("testdata/progressive_420.jpg");
    var jpg = jd.JPEGData{};
    defer jpg.deinit(alloc);
    try reader.readJpeg(data, .read_all, &jpg, alloc);
    try std.testing.expectEqual(@as(i32, 40), jpg.width);
    try std.testing.expectEqual(@as(i32, 32), jpg.height);
    try std.testing.expectEqual(@as(usize, 3), jpg.components.items.len);
    // Progressive: several scans; every scan has at least one component.
    try std.testing.expect(jpg.scan_info.items.len > 1);
    for (jpg.scan_info.items) |s| {
        try std.testing.expect(s.num_components >= 1);
    }
    // Coefficient planes fully populated (luma is components[0]).
    for (jpg.components.items) |c| {
        const num_blocks: usize = @as(usize, @intCast(c.height_in_blocks)) * @as(usize, @intCast(c.width_in_blocks));
        try std.testing.expect(num_blocks > 0);
        try std.testing.expectEqual(num_blocks * 64, c.coeffs.items.len);
    }
    const luma = jpg.components.items[0];
    try std.testing.expectEqual(@as(usize, 6), @as(usize, @intCast(luma.width_in_blocks)));
    try std.testing.expectEqual(@as(usize, 4), @as(usize, @intCast(luma.height_in_blocks)));
}
