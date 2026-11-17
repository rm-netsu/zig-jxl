//! Interleaved pixel buffer used by the codec front-end.

const std = @import("std");

pub const Layout = enum {
    gray,
    rgb,
};

pub const PixelBuffer = struct {
    xsize: usize,
    ysize: usize,
    /// 1 for gray, 3 for rgb.
    layout: Layout,
    has_alpha: bool,
    /// 8 or 16.
    bitdepth: u8,
    /// Interleaved samples: (channels + alpha) per pixel, row-major.
    data: []u16,
    allocator: std.mem.Allocator,

    pub fn numChannels(self: *const PixelBuffer) usize {
        var nch: usize = if (self.layout == .gray) 1 else 3;
        nch += @intFromBool(self.has_alpha);
        return nch;
    }

    pub fn create(allocator: std.mem.Allocator, xsize: usize, ysize: usize, layout: Layout, has_alpha: bool, bitdepth: u8) !PixelBuffer {
        var nch: usize = if (layout == .gray) 1 else 3;
        nch += @intFromBool(has_alpha);
        const data = try allocator.alloc(u16, xsize * ysize * nch);
        return .{
            .xsize = xsize,
            .ysize = ysize,
            .layout = layout,
            .has_alpha = has_alpha,
            .bitdepth = bitdepth,
            .data = data,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *PixelBuffer) void {
        self.allocator.free(self.data);
    }

    pub fn maxValue(self: *const PixelBuffer) u16 {
        return @intCast((@as(u32, 1) << @intCast(self.bitdepth)) - 1);
    }

    /// Sample at (x, y), channel `c` where c < numChannels(); alpha is last.
    pub fn at(self: *const PixelBuffer, x: usize, y: usize, c: usize) u16 {
        const nch = self.numChannels();
        return self.data[(y * self.xsize + x) * nch + c];
    }

    pub fn set(self: *PixelBuffer, x: usize, y: usize, c: usize, value: u16) void {
        const nch = self.numChannels();
        self.data[(y * self.xsize + x) * nch + c] = value;
    }

    pub fn eql(self: *const PixelBuffer, other: *const PixelBuffer) bool {
        return self.xsize == other.xsize and self.ysize == other.ysize and
            self.layout == other.layout and self.has_alpha == other.has_alpha and
            self.bitdepth == other.bitdepth and std.mem.eql(u16, self.data, other.data);
    }
};

test "pixel buffer indexing" {
    const alloc = std.testing.allocator;
    var pb = try PixelBuffer.create(alloc, 4, 3, .rgb, true, 8);
    defer pb.deinit();
    try std.testing.expectEqual(@as(usize, 4), pb.numChannels());
    pb.set(2, 1, 3, 255);
    try std.testing.expectEqual(@as(u16, 255), pb.at(2, 1, 3));
    try std.testing.expectEqual(@as(usize, 4 * 3 * 4), pb.data.len);
}

/// Float pixel surface for IEEE-754 sample formats (fp32 = bits 32 / exp 8,
/// fp16 = bits 16 / exp 5): raw f32 sample values, no alpha channel
/// (PFM-style images). The modular codec carries the custom-float bit
/// patterns as integers internally (`float_convert`).
pub const FloatPixelBuffer = struct {
    xsize: usize,
    ysize: usize,
    /// 1 for gray, 3 for rgb.
    layout: Layout,
    /// 16 (fp16) or 32 (fp32).
    bits_per_sample: u8,
    exponent_bits_per_sample: u8,
    /// Interleaved f32 samples: 1 or 3 per pixel, row-major.
    data: []f32,
    allocator: std.mem.Allocator,

    pub fn numChannels(self: *const FloatPixelBuffer) usize {
        return if (self.layout == .gray) 1 else 3;
    }

    pub fn create(
        allocator: std.mem.Allocator,
        xsize: usize,
        ysize: usize,
        layout: Layout,
        bits_per_sample: u8,
        exponent_bits_per_sample: u8,
    ) !FloatPixelBuffer {
        const nch: usize = if (layout == .gray) 1 else 3;
        const data = try allocator.alloc(f32, xsize * ysize * nch);
        return .{
            .xsize = xsize,
            .ysize = ysize,
            .layout = layout,
            .bits_per_sample = bits_per_sample,
            .exponent_bits_per_sample = exponent_bits_per_sample,
            .data = data,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *FloatPixelBuffer) void {
        self.allocator.free(self.data);
    }

    pub fn at(self: *const FloatPixelBuffer, x: usize, y: usize, c: usize) f32 {
        const nch = self.numChannels();
        return self.data[(y * self.xsize + x) * nch + c];
    }

    pub fn set(self: *FloatPixelBuffer, x: usize, y: usize, c: usize, value: f32) void {
        const nch = self.numChannels();
        self.data[(y * self.xsize + x) * nch + c] = value;
    }

    pub fn eql(self: *const FloatPixelBuffer, other: *const FloatPixelBuffer) bool {
        return self.xsize == other.xsize and self.ysize == other.ysize and
            self.layout == other.layout and
            self.bits_per_sample == other.bits_per_sample and
            self.exponent_bits_per_sample == other.exponent_bits_per_sample and
            std.mem.eql(f32, self.data, other.data);
    }
};

test "float pixel buffer indexing" {
    const alloc = std.testing.allocator;
    var fb = try FloatPixelBuffer.create(alloc, 4, 3, .rgb, 32, 8);
    defer fb.deinit();
    try std.testing.expectEqual(@as(usize, 3), fb.numChannels());
    fb.set(2, 1, 2, 0.25);
    try std.testing.expectEqual(@as(f32, 0.25), fb.at(2, 1, 2));
    try std.testing.expectEqual(@as(usize, 4 * 3 * 3), fb.data.len);
}
