//! SizeHeader / PreviewHeader, port of libjxl `headers.h/cc`.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const U32Enc = fields.U32Enc;

pub const kBlockDim: u32 = 8;

const dim_enc = U32Enc.init(
    fields.bitsOffset(9, 1),
    fields.bitsOffset(13, 1),
    fields.bitsOffset(18, 1),
    fields.bitsOffset(30, 1),
);

const preview_div8_enc = U32Enc.init(
    fields.val(16),
    fields.val(32),
    fields.bitsOffset(5, 1),
    fields.bitsOffset(9, 33),
);

const preview_enc = U32Enc.init(
    fields.bitsOffset(6, 1),
    fields.bitsOffset(8, 65),
    fields.bitsOffset(10, 321),
    fields.bitsOffset(12, 1345),
);

/// One of 7 fixed aspect ratios, or 0 = send xsize explicitly.
fn fixedAspectRatio(ratio: u32) Rational {
    const ratios = [7]Rational{
        .{ .num = 1, .den = 1 },
        .{ .num = 12, .den = 10 },
        .{ .num = 4, .den = 3 },
        .{ .num = 3, .den = 2 },
        .{ .num = 16, .den = 9 },
        .{ .num = 5, .den = 4 },
        .{ .num = 2, .den = 1 },
    };
    return ratios[ratio - 1];
}

const Rational = struct {
    num: u32,
    den: u32,

    fn mulTruncate(self: Rational, multiplicand: u32) u32 {
        return @intCast(@as(u64, multiplicand) * self.num / self.den);
    }
};

fn findAspectRatio(xsize: u32, ysize: u32) u32 {
    var r: u32 = 1;
    while (r < 8) : (r += 1) {
        if (xsize == fixedAspectRatio(r).mulTruncate(ysize)) return r;
    }
    return 0;
}

pub const SizeHeader = struct {
    small: bool = false,
    ysize_div8_minus_1: u32 = 0,
    ysize: u32 = 0,
    ratio: u32 = 0,
    xsize_div8_minus_1: u32 = 0,
    xsize: u32 = 0,

    pub fn xsizeOf(self: *const SizeHeader) u32 {
        if (self.ratio != 0) return fixedAspectRatio(self.ratio).mulTruncate(self.ysizeOf());
        return if (self.small) (self.xsize_div8_minus_1 + 1) * kBlockDim else self.xsize;
    }

    pub fn ysizeOf(self: *const SizeHeader) u32 {
        return if (self.small) (self.ysize_div8_minus_1 + 1) * kBlockDim else self.ysize;
    }

    /// Chooses the same encoding as libjxl SizeHeader::Set.
    pub fn set(self: *SizeHeader, xsize: u32, ysize: u32) !void {
        if (xsize == 0 or ysize == 0) return error.EmptyImage;
        self.ratio = findAspectRatio(xsize, ysize);
        self.small = ysize <= 256 and (ysize % kBlockDim) == 0 and
            (self.ratio != 0 or (xsize <= 256 and (xsize % kBlockDim) == 0));
        if (self.small) {
            self.ysize_div8_minus_1 = ysize / kBlockDim - 1;
        } else {
            self.ysize = ysize;
        }
        if (self.ratio == 0) {
            if (self.small) {
                self.xsize_div8_minus_1 = xsize / kBlockDim - 1;
            } else {
                self.xsize = xsize;
            }
        }
        std.debug.assert(self.xsizeOf() == xsize);
    }

    pub fn read(self: *SizeHeader, br: *BitReader) !void {
        self.* = .{};
        self.small = (br.readBits(1)) != 0;
        if (self.small) {
            self.ysize_div8_minus_1 = @intCast(br.readBits(5));
        } else {
            self.ysize = U32Coder.read(dim_enc, br);
        }
        self.ratio = @intCast(br.readBits(3));
        if (self.ratio > 7) return error.InvalidRatio;
        if (self.ratio == 0 and self.small) {
            self.xsize_div8_minus_1 = @intCast(br.readBits(5));
        }
        if (self.ratio == 0 and !self.small) {
            self.xsize = U32Coder.read(dim_enc, br);
        }
    }

    pub fn write(self: *const SizeHeader, writer: *BitWriter) !void {
        writer.write(1, @intFromBool(self.small));
        if (self.small) {
            writer.write(5, self.ysize_div8_minus_1);
        } else {
            try U32Coder.write(dim_enc, self.ysize, writer);
        }
        writer.write(3, self.ratio);
        if (self.ratio == 0 and self.small) {
            writer.write(5, self.xsize_div8_minus_1);
        }
        if (self.ratio == 0 and !self.small) {
            try U32Coder.write(dim_enc, self.xsize, writer);
        }
    }
};

pub const PreviewHeader = struct {
    div8: bool = false,
    ysize_div8: u32 = 0,
    ysize: u32 = 0,
    ratio: u32 = 0,
    xsize_div8: u32 = 0,
    xsize: u32 = 0,

    pub fn xsizeOf(self: *const PreviewHeader) u32 {
        if (self.ratio != 0) return fixedAspectRatio(self.ratio).mulTruncate(self.ysizeOf());
        return if (self.div8) self.xsize_div8 * kBlockDim else self.xsize;
    }

    pub fn ysizeOf(self: *const PreviewHeader) u32 {
        return if (self.div8) self.ysize_div8 * kBlockDim else self.ysize;
    }

    pub fn read(self: *PreviewHeader, br: *BitReader) !void {
        self.* = .{};
        self.div8 = (br.readBits(1)) != 0;
        if (self.div8) {
            self.ysize_div8 = U32Coder.read(preview_div8_enc, br);
        } else {
            self.ysize = U32Coder.read(preview_enc, br);
        }
        self.ratio = @intCast(br.readBits(3));
        if (self.ratio > 7) return error.InvalidRatio;
        if (self.ratio == 0 and self.div8) {
            self.xsize_div8 = U32Coder.read(preview_div8_enc, br);
        }
        if (self.ratio == 0 and !self.div8) {
            self.xsize = U32Coder.read(preview_enc, br);
        }
    }

    pub fn write(self: *const PreviewHeader, writer: *BitWriter) !void {
        writer.write(1, @intFromBool(self.div8));
        if (self.div8) {
            try U32Coder.write(preview_div8_enc, self.ysize_div8, writer);
        } else {
            try U32Coder.write(preview_enc, self.ysize, writer);
        }
        writer.write(3, self.ratio);
        if (self.ratio == 0 and self.div8) {
            try U32Coder.write(preview_div8_enc, self.xsize_div8, writer);
        }
        if (self.ratio == 0 and !self.div8) {
            try U32Coder.write(preview_enc, self.xsize, writer);
        }
    }
};

test "size header roundtrip small" {
    const alloc = std.testing.allocator;
    var sh: SizeHeader = .{};
    try sh.set(64, 64);
    try std.testing.expect(sh.small);
    var w = BitWriter.init(alloc);
    defer w.deinit();
    try sh.write(&w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    var got: SizeHeader = .{};
    try got.read(&br);
    try br.close();
    try std.testing.expectEqual(@as(u32, 64), got.xsizeOf());
    try std.testing.expectEqual(@as(u32, 64), got.ysizeOf());
}

test "size header roundtrip arbitrary" {
    const alloc = std.testing.allocator;
    var sh: SizeHeader = .{};
    try sh.set(257, 100);
    try std.testing.expect(!sh.small);
    var w = BitWriter.init(alloc);
    defer w.deinit();
    try sh.write(&w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    var got: SizeHeader = .{};
    try got.read(&br);
    try br.close();
    try std.testing.expectEqual(@as(u32, 257), got.xsizeOf());
    try std.testing.expectEqual(@as(u32, 100), got.ysizeOf());
}

test "size header ratio" {
    var sh: SizeHeader = .{};
    try sh.set(16, 9);
    try std.testing.expectEqual(@as(u32, 16), sh.xsizeOf());
    try std.testing.expectEqual(@as(u32, 9), sh.ysizeOf());
}
