//! Минимальный порт `lib/jxl/modular/modular_image.h`: Plane, Channel, Image.
//! Упрощение: строки плоскости хранятся плотно (stride == xsize), без
//! выравнивания и memory manager libjxl.

const std = @import("std");
const common = @import("common.zig");

pub const pixel_type = common.pixel_type;

/// Порт `jxl::Plane<pixel_type>` (упрощённый).
pub fn Plane(comptime T: type) type {
    return struct {
        const Self = @This();

        data: []T = &.{},
        xsize: usize = 0,
        ysize: usize = 0,
        gpa: std.mem.Allocator,

        pub fn create(gpa: std.mem.Allocator, xsize: usize, ysize: usize) !Self {
            const data = try gpa.alloc(T, xsize * ysize);
            return .{ .data = data, .xsize = xsize, .ysize = ysize, .gpa = gpa };
        }

        pub fn deinit(self: *Self) void {
            if (self.data.len > 0) self.gpa.free(self.data);
            self.* = .{ .gpa = self.gpa };
        }

        pub inline fn row(self: *const Self, y: usize) []T {
            std.debug.assert(y < self.ysize);
            return self.data[y * self.xsize .. (y + 1) * self.xsize];
        }

        pub inline fn rowConst(self: *const Self, y: usize) []const T {
            return self.row(y);
        }
    };
}

/// Порт `jxl::Channel`.
pub const Channel = struct {
    plane: Plane(pixel_type) = undefined,
    w: usize = 0,
    h: usize = 0,
    hshift: i32 = 0,
    vshift: i32 = 0,
    component: i32 = -1,
    gpa: std.mem.Allocator = undefined,

    pub fn create(gpa: std.mem.Allocator, iw: usize, ih: usize, hsh: i32, vsh: i32) !Channel {
        var ch = Channel{
            .w = iw,
            .h = ih,
            .hshift = hsh,
            .vshift = vsh,
            .gpa = gpa,
        };
        ch.plane = try Plane(pixel_type).create(gpa, iw, ih);
        return ch;
    }

    pub fn deinit(self: *Channel) void {
        self.plane.deinit();
    }

    /// Port of `Channel::shrink`: resize the plane to the current (w, h),
    /// preserving the top-left region.
    pub fn shrink(self: *Channel) !void {
        if (self.plane.xsize == self.w and self.plane.ysize == self.h) return;
        var new_plane = try Plane(pixel_type).create(self.gpa, self.w, self.h);
        const min_x = @min(self.plane.xsize, self.w);
        const min_y = @min(self.plane.ysize, self.h);
        for (0..min_y) |y| {
            @memcpy(new_plane.row(y)[0..min_x], self.plane.row(y)[0..min_x]);
        }
        self.plane.deinit();
        self.plane = new_plane;
    }

    pub inline fn row(self: *const Channel, y: usize) []pixel_type {
        return self.plane.row(y);
    }
};

/// Порт `jxl::Image` (modular).
pub const Image = struct {
    channels: std.ArrayList(Channel) = .empty,
    nb_meta_channels: usize = 0,
    gpa: std.mem.Allocator,

    pub fn create(gpa: std.mem.Allocator) Image {
        return .{ .gpa = gpa };
    }

    pub fn deinit(self: *Image) void {
        for (self.channels.items) |*ch| ch.deinit();
        self.channels.deinit(self.gpa);
    }

    pub fn addChannel(self: *Image, w: usize, h: usize) !usize {
        try self.channels.append(self.gpa, try Channel.create(self.gpa, w, h, 0, 0));
        return self.channels.items.len - 1;
    }
};

/// Порт `ZeroFillImage`.
pub fn zeroFillImage(plane: anytype) void {
    @memset(plane.data, 0);
}

test "Channel: доступ к строкам" {
    const gpa = std.testing.allocator;
    var ch = try Channel.create(gpa, 8, 4, 0, 0);
    defer ch.deinit();
    ch.row(0)[3] = 42;
    ch.row(3)[7] = -1;
    try std.testing.expectEqual(@as(pixel_type, 42), ch.row(0)[3]);
    try std.testing.expectEqual(@as(pixel_type, -1), ch.row(3)[7]);
}

