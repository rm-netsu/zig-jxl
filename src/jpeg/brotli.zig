//! Brotli support for the `jbrd` payload, backed by brotli-zig (a full
//! native Zig implementation of RFC 7932 used as a build dependency).
//!
//! The writer compresses the `jbrd` blob with real Brotli (the same thing
//! libjxl's `EncodeJPEGData` does — BrotliEncoderCompressStream at
//! `brotli_effort` quality, default 9 to match cjxl); the reader decodes
//! any valid Brotli stream, so both cjxl/libjxl-produced `jbrd` boxes and
//! this codec's own output are accepted.
//!
//! The raw-block (stored meta-block) writer is kept as the quality-0
//! fallback shape and for tests; standard Brotli decoders accept those
//! streams verbatim.

const std = @import("std");
const brotli = @import("brotli");

pub const Error = error{
    BrotliCorrupt,
    OutOfMemory,
};

/// Options mirroring the libjxl/cjxl knobs used for the `jbrd` payload.
pub const Options = struct {
    /// Brotli quality 0..11. libjxl's `EncodeJPEGData` uses
    /// `cparams.brotli_effort` (cjxl default: 9, `--brotli_effort`).
    quality: u32 = 9,
    /// Log2 of the sliding window (libjxl leaves Brotli's default, 22).
    lgwin: u5 = 22,
};

/// Compresses `data` into a complete Brotli stream (real compression —
/// the encoder-side counterpart of libjxl `EncodeJPEGData`'s
/// BrotliEncoderCompressStream loop).
pub fn compress(allocator: std.mem.Allocator, data: []const u8, options: Options) Error![]u8 {
    const quality: u32 = if (options.quality > 11) 11 else options.quality;
    return brotli.compress(allocator, data, .{
        .quality = quality,
        .lgwin = options.lgwin,
    }) catch |e| switch (e) {
        error.OutOfMemory => Error.OutOfMemory,
        else => Error.BrotliCorrupt,
    };
}

/// Decompresses a complete Brotli stream: any valid stream is accepted,
/// including libjxl/libbrotli-produced `jbrd` payloads and the raw-block
/// streams written by `writeRaw`. Trailing bytes after the stream are
/// rejected (matching libjxl's "Unused data after brotli stream").
pub fn decompress(allocator: std.mem.Allocator, data: []const u8) Error![]u8 {
    return brotli.decompress(allocator, data, .{}) catch |e| switch (e) {
        error.OutOfMemory => Error.OutOfMemory,
        else => Error.BrotliCorrupt,
    };
}

/// Appends `data` into `out` as a complete Brotli stream consisting of
/// uncompressed (raw) meta-blocks (WBITS=16, LSB-first bit order, RFC 7932).
/// Uncompressed meta-blocks cannot be last, so the stream terminates with an
/// empty last meta-block (ISLAST=1, ISLASTEMPTY=1).
pub fn writeRaw(allocator: std.mem.Allocator, data: []const u8, out: *std.ArrayList(u8)) Error!void {
    var bw = BitWriter{ .out = out, .allocator = allocator };
    bw.writeBits(1, 0); // WBITS = 16

    if (data.len == 0) {
        bw.writeBits(1, 1); // ISLAST
        bw.writeBits(1, 1); // ISLASTEMPTY
        bw.flushByte();
        return;
    }
    const max_block = 1 << 16;
    var pos: usize = 0;
    while (pos < data.len) {
        const chunk: usize = @min(data.len - pos, max_block);
        bw.writeBits(1, 0); // ISLAST = 0
        bw.writeBits(2, 0); // MNIBBLES = 4
        bw.writeBits(16, @intCast(chunk - 1));
        bw.writeBits(1, 1); // UNCOMPRESSED = 1
        bw.flushByte(); // raw data starts on a byte boundary
        try out.appendSlice(allocator, data[pos .. pos + chunk]);
        pos += chunk;
    }
    bw.writeBits(1, 1); // ISLAST
    bw.writeBits(1, 1); // ISLASTEMPTY
    bw.flushByte();
}

/// Decodes a Brotli stream (any valid stream; kept for compatibility with
/// earlier call sites that expected raw-block streams).
pub fn readRaw(allocator: std.mem.Allocator, data: []const u8) Error![]u8 {
    return decompress(allocator, data);
}

const BitWriter = struct {
    out: *std.ArrayList(u8),
    allocator: std.mem.Allocator,
    acc: u8 = 0,
    nbits: u4 = 0,

    fn writeBits(self: *BitWriter, nbits: u6, value: u64) void {
        var n: u6 = 0;
        while (n < nbits) : (n += 1) {
            const bit: u8 = @intCast((value >> @intCast(n)) & 1);
            self.acc |= bit << @as(u3, @intCast(self.nbits));
            if (self.nbits == 7) {
                self.out.append(self.allocator, self.acc) catch unreachable;
                self.acc = 0;
                self.nbits = 0;
            } else {
                self.nbits += 1;
            }
        }
    }

    fn flushByte(self: *BitWriter) void {
        if (self.nbits > 0) {
            self.out.append(self.allocator, self.acc) catch unreachable;
            self.acc = 0;
            self.nbits = 0;
        }
    }
};

test "brotli raw roundtrip" {
    const alloc = std.testing.allocator;
    const payloads = [_][]const u8{
        "",
        "a",
        "hello world",
        "x" ** 17,
        "y" ** 65535,
        "z" ** 65536,
        "w" ** 65537,
        &.{ 0xFF, 0x00, 0x81, 0xAB, 0xCD },
    };
    for (payloads) |p| {
        var stream: std.ArrayList(u8) = .empty;
        defer stream.deinit(alloc);
        try writeRaw(alloc, p, &stream);
        const decoded = try readRaw(alloc, stream.items);
        defer alloc.free(decoded);
        try std.testing.expectEqualSlices(u8, p, decoded);
    }
}

test "brotli real compression roundtrip" {
    const alloc = std.testing.allocator;
    const pattern = [_]u8{ 0xFF, 0x00, 0x81, 0xAB, 0xCD };
    const noisy = pattern ** 100;
    const payloads = [_][]const u8{
        "",
        "a",
        "hello world hello world hello world",
        "The quick brown fox jumps over the lazy dog. " ** 20,
        &noisy,
    };
    for (payloads) |p| {
        for ([_]u32{ 0, 1, 2, 5, 9, 11 }) |q| {
            const compressed = try compress(alloc, p, .{ .quality = q });
            defer alloc.free(compressed);
            const decoded = try decompress(alloc, compressed);
            defer alloc.free(decoded);
            try std.testing.expectEqualSlices(u8, p, decoded);
        }
    }
}

test "brotli trailing data rejected" {
    const alloc = std.testing.allocator;
    const compressed = try compress(alloc, "hello", .{});
    defer alloc.free(compressed);
    var padded: std.ArrayList(u8) = .empty;
    defer padded.deinit(alloc);
    try padded.appendSlice(alloc, compressed);
    try padded.append(alloc, 0x5A);
    try std.testing.expectError(
        Error.BrotliCorrupt,
        decompress(alloc, padded.items),
    );
}
