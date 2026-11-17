//! PNM (Netpbm) reading and writing: P5 (gray), P6 (RGB), with optional
//! P7 (PAM) style RGBA via P7 header. 8/16-bit, big-endian samples.
//! PFM (PF/Pf) float images: 32-bit IEEE-754 samples, endianness from the
//! scale-factor sign, rows stored bottom-up (libjxl lib/extras/pnm.cc).

const std = @import("std");
const pixel_mod = @import("codec/pixel.zig");

const Allocator = std.mem.Allocator;
const PixelBuffer = pixel_mod.PixelBuffer;
const FloatPixelBuffer = pixel_mod.FloatPixelBuffer;

pub const Error = error{
    InvalidPnmHeader,
    UnsupportedPnmKind,
    UnsupportedPnmDepth,
    UnexpectedEof,
    SampleOutOfRange,
    InvalidPamHeader,
    InvalidPfmHeader,
} || Allocator.Error;

const Header = struct {
    kind: u8, // 5 = gray, 6 = rgb, 7 = PAM
    xsize: usize,
    ysize: usize,
    maxval: usize,
    depth: usize = 1, // PAM only
    tuple_type: TupleType = .gray,

    pub const TupleType = enum { gray, gray_alpha, rgb, rgb_alpha };
    pub const tuple_type_default: TupleType = .gray;
};

fn isSpace(ch: u8) bool {
    return ch == ' ' or ch == '\t' or ch == '\r' or ch == '\n';
}

const Scanner = struct {
    data: []const u8,
    pos: usize = 0,

    fn skipWhitespaceAndComments(self: *Scanner) !void {
        while (self.pos < self.data.len) {
            if (isSpace(self.data[self.pos])) {
                self.pos += 1;
            } else if (self.data[self.pos] == '#') {
                while (self.pos < self.data.len and self.data[self.pos] != '\n')
                    self.pos += 1;
            } else break;
        }
        if (self.pos >= self.data.len) return error.UnexpectedEof;
    }

    fn readUint(self: *Scanner) !usize {
        try self.skipWhitespaceAndComments();
        var value: usize = 0;
        var digits: usize = 0;
        while (self.pos < self.data.len and
            self.data[self.pos] >= '0' and
            self.data[self.pos] <= '9')
        {
            value = value * 10 + (self.data[self.pos] - '0');
            self.pos += 1;
            digits += 1;
            if (digits > 10) return error.InvalidPnmHeader;
        }
        if (digits == 0) return error.InvalidPnmHeader;
        return value;
    }

    fn readToken(self: *Scanner) ![]const u8 {
        try self.skipWhitespaceAndComments();
        const start = self.pos;
        while (self.pos < self.data.len and !isSpace(self.data[self.pos]))
            self.pos += 1;
        if (self.pos == start) return error.InvalidPamHeader;
        return self.data[start..self.pos];
    }
};

fn parseHeader(
    data: []const u8,
) !struct { header: Header, data_offset: usize } {
    if (data.len < 2 or data[0] != 'P') return error.InvalidPnmHeader;
    const kind = data[1];
    if (kind != '5' and kind != '6' and kind != '7')
        return error.UnsupportedPnmKind;
    var sc = Scanner{ .data = data };
    sc.pos = 2;

    var xsize: usize = 0;
    var ysize: usize = 0;
    var maxval: usize = 0;
    var depth: usize = 1;
    var tuple: @TypeOf(Header.tuple_type_default) = .gray;

    if (kind == '7') {
        // PAM: WIDTH HEIGHT DEPTH MAXVAL TUPLTYPE ENDHDR
        while (true) {
            const tok = try sc.readToken();
            if (std.mem.eql(u8, tok, "ENDHDR")) break;
            if (std.mem.eql(u8, tok, "WIDTH")) {
                xsize = try sc.readUint();
            } else if (std.mem.eql(u8, tok, "HEIGHT")) {
                ysize = try sc.readUint();
            } else if (std.mem.eql(u8, tok, "DEPTH")) {
                depth = try sc.readUint();
            } else if (std.mem.eql(u8, tok, "MAXVAL")) {
                maxval = try sc.readUint();
            } else if (std.mem.eql(u8, tok, "TUPLTYPE")) {
                const tt = try sc.readToken();
                tuple = if (std.mem.eql(u8, tt, "GRAYSCALE"))
                    .gray
                else if (std.mem.eql(u8, tt, "GRAYSCALE_ALPHA"))
                    .gray_alpha
                else if (std.mem.eql(u8, tt, "RGB"))
                    .rgb
                else if (std.mem.eql(u8, tt, "RGB_ALPHA"))
                    .rgb_alpha
                else
                    return error.InvalidPamHeader;
            } else {
                return error.InvalidPamHeader;
            }
        }
        if (depth < 1 or depth > 4) return error.InvalidPamHeader;
        if (maxval == 0) return error.InvalidPamHeader;
    } else {
        xsize = try sc.readUint();
        ysize = try sc.readUint();
        maxval = try sc.readUint();
    }
    // Exactly one whitespace byte separates the header from the raster.
    if (sc.pos >= data.len) return error.UnexpectedEof;
    sc.pos += 1;

    if (xsize == 0 or ysize == 0) return error.InvalidPnmHeader;
    if (maxval == 0 or maxval > 65535) return error.InvalidPnmHeader;
    return .{
        .header = .{
            .kind = kind,
            .xsize = xsize,
            .ysize = ysize,
            .maxval = maxval,
            .depth = depth,
            .tuple_type = tuple,
        },
        .data_offset = sc.pos,
    };
}

/// Reads PNM/PAM data into a PixelBuffer; PFM (PF/Pf) data into a
/// FloatPixelBuffer.
pub fn read(allocator: Allocator, data: []const u8) Error!PixelBuffer {
    const parsed = try parseHeader(data);
    const hdr = parsed.header;

    const bitdepth: u8 = if (hdr.maxval > 255) 16 else 8;
    var layout: pixel_mod.Layout = undefined;
    var has_alpha = false;
    var nch: usize = 0;
    if (hdr.kind == '5') {
        layout = .gray;
        nch = 1;
    } else if (hdr.kind == '6') {
        layout = .rgb;
        nch = 3;
    } else {
        switch (hdr.tuple_type) {
            .gray => {
                layout = .gray;
                nch = 1;
            },
            .gray_alpha => {
                layout = .gray;
                has_alpha = true;
                nch = 2;
            },
            .rgb => {
                layout = .rgb;
                nch = 3;
            },
            .rgb_alpha => {
                layout = .rgb;
                has_alpha = true;
                nch = 4;
            },
        }
        if (hdr.depth != nch) return error.InvalidPamHeader;
    }

    var pb = try PixelBuffer.create(
        allocator,
        hdr.xsize,
        hdr.ysize,
        layout,
        has_alpha,
        bitdepth,
    );
    errdefer pb.deinit();

    const raster = data[parsed.data_offset..];
    const wide = bitdepth == 16;
    const bytes_per_sample: usize = if (wide) 2 else 1;
    const need = hdr.xsize * hdr.ysize * nch * bytes_per_sample;
    if (raster.len < need) return error.UnexpectedEof;

    const max_value = pb.maxValue();
    const limit: usize = hdr.maxval;
    var i: usize = 0;
    var sample: usize = 0;
    const total_samples = hdr.xsize * hdr.ysize * nch;
    while (sample < total_samples) : (sample += 1) {
        var v: usize = 0;
        if (wide) {
            v = (@as(usize, raster[i]) << 8) | raster[i + 1];
            i += 2;
        } else {
            v = raster[i];
            i += 1;
        }
        if (v > limit) return error.SampleOutOfRange;
        // Scale maxval != 2^bitdepth-1 into the full range (as libjxl does).
        const scaled: u16 = if (limit == max_value)
            @intCast(v)
        else
            @intCast((v * max_value + limit / 2) / limit);
        pb.data[sample] = scaled;
    }
    return pb;
}

/// Serializes a PixelBuffer as PNM (no alpha) or PAM (with alpha).
pub fn write(allocator: Allocator, pb: *const PixelBuffer) ![]u8 {
    const nch = pb.numChannels();
    const max_value: usize = pb.maxValue();
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    const w = &out.writer;

    if (pb.has_alpha) {
        const tuple: []const u8 = switch (pb.layout) {
            .gray => "GRAYSCALE_ALPHA",
            .rgb => "RGB_ALPHA",
        };
        try w.print(
            "P7\nWIDTH {d}\nHEIGHT {d}\nDEPTH {d}\nMAXVAL {d}\nTUPLTYPE {s}\nENDHDR\n",
            .{ pb.xsize, pb.ysize, nch, max_value, tuple },
        );
    } else {
        const kind: u8 = if (pb.layout == .gray) '5' else '6';
        try w.print(
            "P{c}\n{d} {d}\n{d}\n",
            .{ kind, pb.xsize, pb.ysize, max_value },
        );
    }

    const wide = pb.bitdepth == 16;
    for (pb.data) |v| {
        if (wide) {
            try w.writeByte(@intCast(v >> 8));
            try w.writeByte(@intCast(v & 0xFF));
        } else {
            try w.writeByte(@intCast(v & 0xFF));
        }
    }
    return out.toOwnedSlice();
}

test "pnm gray 8-bit roundtrip" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(
        alloc,
        4,
        3,
        .gray,
        false,
        8,
    );
    defer pb.deinit();
    for (pb.data, 0..) |*v, i| v.* = @intCast(i % 256);
    const bytes = try write(alloc, &pb);
    defer alloc.free(bytes);
    var got = try read(alloc, bytes);
    defer got.deinit();
    try std.testing.expect(pb.eql(&got));
}

test "pnm rgb 16-bit roundtrip" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(
        alloc,
        5,
        2,
        .rgb,
        false,
        16,
    );
    defer pb.deinit();
    for (pb.data, 0..) |*v, i| v.* = @intCast(i * 17 % 65536);
    const bytes = try write(alloc, &pb);
    defer alloc.free(bytes);
    var got = try read(alloc, bytes);
    defer got.deinit();
    try std.testing.expect(pb.eql(&got));
}

test "pam rgba 8-bit roundtrip" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(
        alloc,
        3,
        3,
        .rgb,
        true,
        8,
    );
    defer pb.deinit();
    for (pb.data, 0..) |*v, i| v.* = @intCast((i * 7) % 256);
    const bytes = try write(alloc, &pb);
    defer alloc.free(bytes);
    var got = try read(alloc, bytes);
    defer got.deinit();
    try std.testing.expect(pb.eql(&got));
}

// ---------------------------------------------------------------------------
// PFM (portable float map): PF = RGB, Pf = gray. 32-bit IEEE-754 samples;
// the sign of the scale factor selects the byte order (negative = little
// endian, lib/extras/dec/pnm.cc), and rows are stored bottom-up.
// ---------------------------------------------------------------------------

/// Parses "PF"/"Pf" header: magic, whitespace, xsize, blank, ysize,
/// whitespace, scale factor, single whitespace.
fn parsePfmHeader(
    data: []const u8,
) !struct { xsize: usize, ysize: usize, big_endian: bool, data_offset: usize } {
    if (data.len < 3 or data[0] != 'P' or (data[1] != 'F' and data[1] != 'f')) {
        return error.UnsupportedPnmKind;
    }
    var sc = Scanner{ .data = data };
    sc.pos = 2;
    try sc.skipWhitespaceAndComments();
    const xsize = try sc.readUint();
    try sc.skipWhitespaceAndComments();
    const ysize = try sc.readUint();
    try sc.skipWhitespaceAndComments();
    // The scale has no meaning as a multiplier, only its sign selects the
    // byte order; zero is invalid.
    const scale = try readSignedFloat(&sc);
    if (scale == 0) return error.InvalidPfmHeader;
    if (sc.pos >= data.len) return error.UnexpectedEof;
    // Exactly one whitespace byte before the raster.
    sc.pos += 1;
    if (xsize == 0 or ysize == 0) return error.InvalidPfmHeader;
    return .{
        .xsize = xsize,
        .ysize = ysize,
        .big_endian = scale > 0,
        .data_offset = sc.pos,
    };
}

fn readSignedFloat(sc: *Scanner) !f64 {
    try sc.skipWhitespaceAndComments();
    const start = sc.pos;
    if (sc.pos < sc.data.len and
        (sc.data[sc.pos] == '-' or sc.data[sc.pos] == '+')) sc.pos += 1;
    var digits: usize = 0;
    while (sc.pos < sc.data.len) : (sc.pos += 1) {
        const ch = sc.data[sc.pos];
        if ((ch >= '0' and ch <= '9') or ch == '.' or ch == 'e' or ch == 'E' or
            ((ch == '-' or ch == '+') and
                (sc.data[sc.pos - 1] == 'e' or sc.data[sc.pos - 1] == 'E')))
        {
            if (ch >= '0' and ch <= '9') digits += 1;
            continue;
        }
        break;
    }
    if (digits == 0) return error.InvalidPfmHeader;
    return std.fmt.parseFloat(
        f64,
        sc.data[start..sc.pos],
    ) catch error.InvalidPfmHeader;
}

/// True when `data` carries a PFM (float) image.
pub fn isPfm(data: []const u8) bool {
    return data.len >= 2 and data[0] == 'P' and
        (data[1] == 'F' or data[1] == 'f');
}

/// Reads a PFM (PF = RGB, Pf = gray) into a FloatPixelBuffer.
pub fn readFloat(
    allocator: Allocator,
    data: []const u8,
) Error!FloatPixelBuffer {
    const hdr = try parsePfmHeader(data);
    const layout: pixel_mod.Layout = if (data[1] == 'F') .rgb else .gray;
    const nch: usize = if (layout == .gray) 1 else 3;
    var fb = try FloatPixelBuffer.create(
        allocator,
        hdr.xsize,
        hdr.ysize,
        layout,
        32,
        8,
    );
    errdefer fb.deinit();

    const raster = data[hdr.data_offset..];
    const need = hdr.xsize * hdr.ysize * nch * 4;
    if (raster.len < need) return error.UnexpectedEof;

    var i: usize = 0;
    var sample: usize = 0;
    const total = hdr.xsize * hdr.ysize * nch;
    // PFMs are stored bottom-up; flip while reading.
    const stride = hdr.xsize * nch;
    while (sample < total) : (sample += 1) {
        const b0 = raster[i];
        const b1 = raster[i + 1];
        const b2 = raster[i + 2];
        const b3 = raster[i + 3];
        i += 4;
        const raw: u32 = if (hdr.big_endian)
            (@as(u32, b0) << 24) |
                (@as(u32, b1) << 16) |
                (@as(u32, b2) << 8) |
                b3
        else
            (@as(u32, b3) << 24) |
                (@as(u32, b2) << 16) |
                (@as(u32, b1) << 8) |
                b0;
        const v: f32 = @bitCast(raw);
        const row = sample / stride;
        const col = sample % stride;
        const y_out = hdr.ysize - 1 - row;
        fb.data[y_out * stride + col] = v;
    }
    return fb;
}

/// Serializes a FloatPixelBuffer as PFM. Big-endian samples with a +1.0
/// scale factor, byte-identical to djxl's PFM output.
pub fn writeFloat(allocator: Allocator, fb: *const FloatPixelBuffer) ![]u8 {
    const nch = fb.numChannels();
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    const w = &out.writer;
    const kind: u8 = if (fb.layout == .gray) 'f' else 'F';
    try w.print(
        "P{c}\n{d} {d}\n1.0\n",
        .{ kind, fb.xsize, fb.ysize },
    );
    const stride = fb.xsize * nch;
    var y: usize = fb.ysize;
    while (y > 0) {
        y -= 1;
        const row = fb.data[y * stride .. (y + 1) * stride];
        for (row) |v| {
            const raw: u32 = @bitCast(v);
            try w.writeByte(@intCast((raw >> 24) & 0xFF));
            try w.writeByte(@intCast((raw >> 16) & 0xFF));
            try w.writeByte(@intCast((raw >> 8) & 0xFF));
            try w.writeByte(@intCast(raw & 0xFF));
        }
    }
    return out.toOwnedSlice();
}

test "pfm rgb roundtrip" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(
        alloc,
        5,
        3,
        .rgb,
        32,
        8,
    );
    defer fb.deinit();
    for (fb.data, 0..) |*v, i| v.* = @as(
        f32,
        @floatFromInt(i),
    ) * 0.125 - 3.0;
    const bytes = try writeFloat(alloc, &fb);
    defer alloc.free(bytes);
    var got = try readFloat(alloc, bytes);
    defer got.deinit();
    try std.testing.expect(fb.eql(&got));
}

test "pfm gray roundtrip and endianness" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(
        alloc,
        4,
        4,
        .gray,
        32,
        8,
    );
    defer fb.deinit();
    for (fb.data, 0..) |*v, i|
        v.* = if (i % 3 == 0) -0.5 else @as(f32, @floatFromInt(i)) / 64.0;
    const bytes = try writeFloat(alloc, &fb);
    defer alloc.free(bytes);
    var got = try readFloat(alloc, bytes);
    defer got.deinit();
    try std.testing.expect(fb.eql(&got));

    // Big-endian PFM parses to the same values (rows bottom-up, like the
    // writer emits them).
    var be: std.ArrayList(u8) = .empty;
    defer be.deinit(alloc);
    const hdr = try std.fmt.allocPrint(
        alloc,
        "Pf\n{d} {d}\n1.0\n",
        .{ fb.xsize, fb.ysize },
    );
    defer alloc.free(hdr);
    try be.appendSlice(alloc, hdr);
    var yb: usize = fb.ysize;
    while (yb > 0) {
        yb -= 1;
        for (fb.data[yb * fb.xsize .. (yb + 1) * fb.xsize]) |v| {
            const raw: u32 = @bitCast(v);
            try be.append(alloc, @intCast((raw >> 24) & 0xFF));
            try be.append(alloc, @intCast((raw >> 16) & 0xFF));
            try be.append(alloc, @intCast((raw >> 8) & 0xFF));
            try be.append(alloc, @intCast(raw & 0xFF));
        }
    }
    var got_be = try readFloat(
        alloc,
        be.items,
    );
    defer got_be.deinit();
    try std.testing.expect(fb.eql(&got_be));
}
