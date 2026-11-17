//! JPEG XL container (ISOBMFF-style box) handling.
//! The encoder emits bare codestreams; the decoder accepts both.
//! Brotli-compressed metadata boxes (`brob`, ISOBMFF + ISOBMFF-style)
//! are decoded via the brotli-zig dependency, as libjxl does.

const std = @import("std");
const brotli = @import("brotli");

pub const signature = [12]u8{ 0, 0, 0, 0xC, 'J', 'X', 'L', ' ', 0xD, 0xA, 0x87, 0xA };
pub const raw_signature = [2]u8{ 0xFF, 0x0A };

/// True when `data` starts with the bare codestream signature.
pub fn isRawCodestream(data: []const u8) bool {
    return data.len >= 2 and data[0] == 0xFF and data[1] == 0x0A;
}

/// Appends one ISOBMFF box (type + payload) to `out`.
pub fn writeBox(allocator: std.mem.Allocator, out: *std.ArrayList(u8), box_type: *const [4]u8, payload: []const u8) !void {
    var header: [16]u8 = undefined;
    if (payload.len + 8 <= 0xFFFFFFFF) {
        std.mem.writeInt(u32, header[0..4], @intCast(payload.len + 8), .big);
        @memcpy(header[4..8], box_type);
        try out.appendSlice(allocator, header[0..8]);
    } else {
        std.mem.writeInt(u32, header[0..4], 1, .big);
        @memcpy(header[4..8], box_type);
        std.mem.writeInt(u64, header[8..16], @intCast(payload.len + 16), .big);
        try out.appendSlice(allocator, header[0..16]);
    }
    try out.appendSlice(allocator, payload);
}

/// Returns the payload of the first top-level box with the given type,
/// or null when absent.
pub fn findBox(data: []const u8, box_type: *const [4]u8) ?[]const u8 {
    if (!isContainer(data)) return null;
    var pos: usize = signature.len;
    while (pos + 8 <= data.len) {
        const size: u64 = std.mem.readInt(u32, data[pos..][0..4], .big);
        const t: [4]u8 = data[pos + 4 ..][0..4].*;
        var header_len: usize = 8;
        var box_size: u64 = size;
        if (size == 0) {
            box_size = data.len - pos;
        } else if (size == 1) {
            if (pos + 16 > data.len) break;
            box_size = std.mem.readInt(u64, data[pos + 8 ..][0..8], .big);
            header_len = 16;
        }
        if (box_size < header_len) return null;
        const payload_len: usize = @intCast(box_size - header_len);
        const box_end = pos + header_len + payload_len;
        if (box_end > data.len) return null;
        if (std.mem.eql(u8, &t, box_type)) {
            return data[pos + header_len .. box_end];
        }
        pos = box_end;
    }
    return null;
}

/// True when `data` starts with the container signature.
pub fn isContainer(data: []const u8) bool {
    return data.len >= signature.len and
        std.mem.eql(u8, data[0..signature.len], &signature);
}

pub const Box = struct {
    type: [4]u8,
    /// Payload bytes; empty unless the box is fully present in `data`.
    data: []const u8,
};

/// Lists all top-level boxes of a container (payload slices into `data`).
/// Caller owns the returned list.
pub fn listBoxes(allocator: std.mem.Allocator, data: []const u8) !std.ArrayList(Box) {
    var out: std.ArrayList(Box) = .empty;
    errdefer out.deinit(allocator);
    if (!isContainer(data)) return error.NotAContainer;
    var pos: usize = signature.len;
    while (pos + 8 <= data.len) {
        const size: u64 = std.mem.readInt(u32, data[pos..][0..4], .big);
        const box_type: [4]u8 = data[pos + 4 ..][0..4].*;
        var header_len: usize = 8;
        var box_size: u64 = size;
        if (size == 0) {
            box_size = data.len - pos; // extends to end of file
        } else if (size == 1) {
            if (pos + 16 > data.len) break;
            box_size = std.mem.readInt(u64, data[pos + 8 ..][0..8], .big);
            header_len = 16;
        }
        if (box_size < header_len) return error.InvalidBox;
        const payload_len: usize = @intCast(box_size - header_len);
        const box_end = pos + header_len + payload_len;
        if (box_end > data.len) break; // truncated final box: skip
        try out.append(allocator, .{
            .type = box_type,
            .data = data[pos + header_len .. box_end],
        });
        pos = box_end;
    }
    return out;
}

pub const BrobError = error{
    InvalidBox,
    BrotliCorrupt,
    OutOfMemory,
};

/// A decoded `brob` box: the original box type plus its decompressed
/// payload (libjxl box_content_decoder: 4-byte real type + brotli stream).
pub const DecodedBrob = struct {
    /// The real box type the payload belongs to ("Exif", "xml ", ...).
    kind: [4]u8,
    content: []u8,

    pub fn deinit(self: *const DecodedBrob, allocator: std.mem.Allocator) void {
        allocator.free(self.content);
    }
};

/// Decompresses a `brob` box payload (the 4-byte real box type followed by
/// a Brotli stream) — the counterpart of libjxl's `--compress_boxes`.
pub fn decodeBrob(allocator: std.mem.Allocator, payload: []const u8) BrobError!DecodedBrob {
    if (payload.len < 4) return BrobError.InvalidBox;
    const kind: [4]u8 = payload[0..4].*;
    const content: []u8 = brotli.decompress(allocator, payload[4..], .{}) catch |e| switch (e) {
        error.OutOfMemory => return BrobError.OutOfMemory,
        else => return BrobError.BrotliCorrupt,
    };
    return .{ .kind = kind, .content = content };
}

fn boxTypeEql(a: [4]u8, comptime name: []const u8) bool {
    const expected: [4]u8 = name[0..4].*;
    return std.mem.eql(u8, &a, &expected);
}

/// Extracts the codestream payload from a JXL container by concatenating
/// jxlp and jxlc boxes. Returns null when the container carries no
/// codestream yet. Brotli-compressed metadata boxes (brob) are skipped.
pub fn extractCodestream(allocator: std.mem.Allocator, data: []const u8) ![]u8 {
    if (!isContainer(data)) return error.NotAContainer;
    var out: std.ArrayList(u8) = .empty;
    errdefer out.deinit(allocator);

    var pos: usize = signature.len;
    while (pos + 8 <= data.len) {
        const size: u64 = std.mem.readInt(u32, data[pos..][0..4], .big);
        const box_type: [4]u8 = data[pos + 4 ..][0..4].*;
        var header_len: usize = 8;
        var box_size: u64 = size;
        if (size == 0) {
            box_size = data.len - pos; // extends to end of file
        } else if (size == 1) {
            if (pos + 16 > data.len) break;
            box_size = std.mem.readInt(u64, data[pos + 8 ..][0..8], .big);
            header_len = 16;
        }
        if (box_size < header_len) return error.InvalidBox;
        const payload_len: usize = @intCast(box_size - header_len);
        const box_end = pos + header_len + payload_len;
        if (box_end > data.len) {
            // Truncated final box: keep what is available.
            if (boxTypeEql(box_type, "jxlp") or boxTypeEql(box_type, "jxlc")) {
                try out.appendSlice(allocator, data[pos + header_len ..]);
            }
            break;
        }
        const payload = data[pos + header_len .. box_end];
        if (boxTypeEql(box_type, "jxlc")) {
            try out.appendSlice(allocator, payload);
            break; // jxlc holds the whole remaining codestream
        } else if (boxTypeEql(box_type, "jxlp")) {
            if (payload.len < 4) return error.InvalidBox;
            try out.appendSlice(allocator, payload[4..]); // skip partial index
        }
        // jxll (level), brob (compressed metadata), jbrd (JPEG reconstruction)
        // and unknown boxes are skipped.
        pos = box_end;
    }
    if (out.items.len == 0) return error.NoCodestream;
    return out.toOwnedSlice(allocator);
}

test "container roundtrip through jxlc" {
    const alloc = std.testing.allocator;
    var buf: std.ArrayList(u8) = .empty;
    defer buf.deinit(alloc);
    try buf.appendSlice(alloc, &signature);
    const payload = [_]u8{ 0xFF, 0x0A, 1, 2, 3, 4, 5 };
    var header: [8]u8 = undefined;
    std.mem.writeInt(u32, header[0..4], @intCast(8 + payload.len), .big);
    @memcpy(header[4..8], "jxlc");
    try buf.appendSlice(alloc, &header);
    try buf.appendSlice(alloc, &payload);

    try std.testing.expect(isContainer(buf.items));
    const cs = try extractCodestream(alloc, buf.items);
    defer alloc.free(cs);
    try std.testing.expectEqualSlices(u8, &payload, cs);
}

test "container with jxlp and jxlc" {
    const alloc = std.testing.allocator;
    var buf: std.ArrayList(u8) = .empty;
    defer buf.deinit(alloc);
    try buf.appendSlice(alloc, &signature);
    // jxlp with partial index 0
    const p1 = [_]u8{ 0, 0, 0, 0, 10, 11 };
    var h1: [8]u8 = undefined;
    std.mem.writeInt(u32, h1[0..4], @intCast(8 + p1.len), .big);
    @memcpy(h1[4..8], "jxlp");
    try buf.appendSlice(alloc, &h1);
    try buf.appendSlice(alloc, &p1);
    // final jxlp (index bit 0x80000000) then jxlc
    const p2 = [_]u8{ 0x80, 0, 0, 0, 12, 13 };
    var h2: [8]u8 = undefined;
    std.mem.writeInt(u32, h2[0..4], @intCast(8 + p2.len), .big);
    @memcpy(h2[4..8], "jxlp");
    try buf.appendSlice(alloc, &h2);
    try buf.appendSlice(alloc, &p2);

    const cs = try extractCodestream(alloc, buf.items);
    defer alloc.free(cs);
    try std.testing.expectEqualSlices(u8, &.{ 10, 11, 12, 13 }, cs);
}
