//! ColorEncoding bundle, port of libjxl `color_encoding_internal.h/cc`.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const fields_ext = @import("fields_ext.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const U32Enc = fields.U32Enc;
const EnumCoder = fields_ext.EnumCoder;

pub const ColorSpace = enum(u32) {
    rgb = 0,
    gray = 1,
    xyb = 2,
    unknown = 3,

    pub const valid_mask: u64 = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
};

pub const WhitePoint = enum(u32) {
    d65 = 1,
    custom = 2,
    e = 10,
    dci = 11,

    pub const valid_mask: u64 = (1 << 1) | (1 << 2) | (1 << 10) | (1 << 11);
};

pub const Primaries = enum(u32) {
    srgb = 1,
    custom = 2,
    bt2100 = 9,
    p3 = 11,

    pub const valid_mask: u64 = (1 << 1) | (1 << 2) | (1 << 9) | (1 << 11);
};

pub const TransferFunction = enum(u32) {
    bt709 = 1,
    unknown = 2,
    linear = 8,
    srgb = 13,
    pq = 16,
    dci = 17,
    hlg = 18,

    pub const valid_mask: u64 = (1 << 1) | (1 << 2) | (1 << 8) | (1 << 13) | (1 << 16) | (1 << 17) | (1 << 18);
};

pub const RenderingIntent = enum(u32) {
    perceptual = 0,
    relative = 1,
    saturation = 2,
    absolute = 3,

    pub const valid_mask: u64 = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
};

const CIExy = struct {
    x: i32 = 0, // 1e6-scale
    y: i32 = 0,
};

const custom_xy_enc = U32Enc.init(
    fields.bitsDistr(19),
    fields.bitsOffset(19, 524288),
    fields.bitsOffset(20, 1048576),
    fields.bitsOffset(21, 2097152),
);

const Customxy = struct {
    x: i32 = 0,
    y: i32 = 0,

    fn read(self: *Customxy, br: *BitReader) !void {
        self.* = .{};
        var ux: u32 = @bitCast(self.x);
        ux = U32Coder.read(custom_xy_enc, br);
        self.x = @bitCast(ux);
        var uy: u32 = @bitCast(self.y);
        uy = U32Coder.read(custom_xy_enc, br);
        self.y = @bitCast(uy);
    }

    fn write(self: *const Customxy, writer: *BitWriter) !void {
        try U32Coder.write(custom_xy_enc, @bitCast(self.x), writer);
        try U32Coder.write(custom_xy_enc, @bitCast(self.y), writer);
    }
};

pub const CustomTransferFunction = struct {
    have_gamma: bool = false,
    gamma: u32 = 0, // gamma * 1e7
    transfer_function: TransferFunction = .srgb,

    pub fn isDefault(self: *const CustomTransferFunction) bool {
        return !self.have_gamma and self.transfer_function == .srgb;
    }

    fn read(self: *CustomTransferFunction, color_space: ColorSpace, br: *BitReader) !void {
        self.* = .{};
        // SetImplicit: XYB color space implies non-linear sRGB transfer.
        if (color_space == .xyb) return;
        self.have_gamma = (br.readBits(1)) != 0;
        if (self.have_gamma) {
            self.gamma = @intCast(br.readBits(24));
            if (self.gamma == 0 or self.gamma > kGammaMul) return error.InvalidGamma;
        } else {
            self.transfer_function = try EnumCoder(TransferFunction).read(.srgb, br);
        }
    }

    fn write(self: *const CustomTransferFunction, color_space: ColorSpace, writer: *BitWriter) !void {
        if (color_space == .xyb) return;
        writer.write(1, @intFromBool(self.have_gamma));
        if (self.have_gamma) {
            writer.write(24, self.gamma);
        } else {
            try EnumCoder(TransferFunction).write(self.transfer_function, writer);
        }
    }

    const kGammaMul: u32 = 10000000;
};

pub const ColorEncoding = struct {
    all_default: bool = true,
    want_icc: bool = false,
    color_space: ColorSpace = .rgb,
    white_point: WhitePoint = .d65,
    custom_white: CIExy = .{},
    primaries: Primaries = .srgb,
    custom_red: CIExy = .{},
    custom_green: CIExy = .{},
    custom_blue: CIExy = .{},
    tf: CustomTransferFunction = .{},
    rendering_intent: RenderingIntent = .relative,

    pub fn srgb() ColorEncoding {
        return .{ .all_default = true };
    }

    pub fn graySrgb() ColorEncoding {
        return .{ .all_default = false, .color_space = .gray };
    }

    /// An ICC-carrying ColorEncoding (libjxl `SetICCRaw` semantics: the
    /// direct fields stay untouched, only the flag and the color space are
    /// serialized; the profile bytes travel in the codestream ICC stream
    /// after CustomTransformData).
    pub fn wantIcc(gray: bool) ColorEncoding {
        return .{ .all_default = false, .want_icc = true, .color_space = if (gray) .gray else .rgb };
    }

    pub fn isGray(self: *const ColorEncoding) bool {
        return self.color_space == .gray;
    }

    pub fn isAllDefault(self: *const ColorEncoding) bool {
        return self.want_icc == false and
            self.color_space == .rgb and
            self.white_point == .d65 and
            self.primaries == .srgb and
            self.tf.isDefault() and
            self.rendering_intent == .relative;
    }

    pub fn read(self: *ColorEncoding, br: *BitReader) !void {
        self.* = .{};
        self.all_default = (br.readBits(1)) != 0;
        if (self.all_default) {
            self.* = .{ .all_default = true };
            return;
        }
        self.want_icc = (br.readBits(1)) != 0;
        self.color_space = try EnumCoder(ColorSpace).read(.rgb, br);
        if (!self.want_icc) {
            // White point is always serialized except for XYB.
            if (self.color_space != .xyb) {
                self.white_point = try EnumCoder(WhitePoint).read(.d65, br);
                if (self.white_point == .custom) {
                    var white: Customxy = .{};
                    try white.read(br);
                    self.custom_white = .{ .x = white.x, .y = white.y };
                }
            }
            if (self.hasPrimaries()) {
                self.primaries = try EnumCoder(Primaries).read(.srgb, br);
                if (self.primaries == .custom) {
                    var red: Customxy = .{};
                    var green: Customxy = .{};
                    var blue: Customxy = .{};
                    try red.read(br);
                    try green.read(br);
                    try blue.read(br);
                    self.custom_red = .{ .x = red.x, .y = red.y };
                    self.custom_green = .{ .x = green.x, .y = green.y };
                    self.custom_blue = .{ .x = blue.x, .y = blue.y };
                }
            }
            try self.tf.read(self.color_space, br);
            self.rendering_intent = try EnumCoder(RenderingIntent).read(.relative, br);
            if (self.color_space == .unknown or self.tf.have_gamma == false and self.tf.transfer_function == .unknown) {
                return error.NoIccButUnknownFields;
            }
        }
        // When want_icc is set the codestream carries a compressed ICC
        // profile right after the CustomTransformData bundle (see
        // header/icc.zig); the ColorEncoding bundle itself serializes only
        // the color space.
    }

    pub fn write(self: *const ColorEncoding, writer: *BitWriter) !void {
        const all_default = self.isAllDefault();
        writer.write(1, @intFromBool(all_default));
        if (all_default) return;
        writer.write(1, @intFromBool(self.want_icc));
        try EnumCoder(ColorSpace).write(self.color_space, writer);
        // The remaining fields are only serialized when the profile is not
        // carried by the codestream ICC stream (libjxl
        // `visitor->Conditional(!want_icc_)`).
        if (self.want_icc) return;
        if (self.color_space != .xyb) {
            try EnumCoder(WhitePoint).write(self.white_point, writer);
            if (self.white_point == .custom) {
                const white = Customxy{ .x = self.custom_white.x, .y = self.custom_white.y };
                try white.write(writer);
            }
        }
        if (self.hasPrimaries()) {
            try EnumCoder(Primaries).write(self.primaries, writer);
            if (self.primaries == .custom) {
                try (Customxy{ .x = self.custom_red.x, .y = self.custom_red.y }).write(writer);
                try (Customxy{ .x = self.custom_green.x, .y = self.custom_green.y }).write(writer);
                try (Customxy{ .x = self.custom_blue.x, .y = self.custom_blue.y }).write(writer);
            }
        }
        try self.tf.write(self.color_space, writer);
        try EnumCoder(RenderingIntent).write(self.rendering_intent, writer);
    }

    fn hasPrimaries(self: *const ColorEncoding) bool {
        return self.color_space == .rgb or self.color_space == .unknown;
    }
};

test "color encoding srgb default roundtrip" {
    const alloc = std.testing.allocator;
    var w = BitWriter.init(alloc);
    defer w.deinit();
    const enc = ColorEncoding.srgb();
    try enc.write(&w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    var got: ColorEncoding = .{};
    try got.read(&br);
    try br.close();
    try std.testing.expect(got.all_default);
}

test "color encoding gray roundtrip" {
    const alloc = std.testing.allocator;
    var w = BitWriter.init(alloc);
    defer w.deinit();
    const enc = ColorEncoding.graySrgb();
    try enc.write(&w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    var got: ColorEncoding = .{};
    try got.read(&br);
    try br.close();
    try std.testing.expectEqual(ColorSpace.gray, got.color_space);
    try std.testing.expectEqual(TransferFunction.srgb, got.tf.transfer_function);
    try std.testing.expectEqual(RenderingIntent.relative, got.rendering_intent);
    try std.testing.expectEqual(WhitePoint.d65, got.white_point);
}
