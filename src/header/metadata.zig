//! ImageMetadata bundle family, port of libjxl `image_metadata.h/cc`.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const nlee = @import("../base/nlee.zig");
const fields_ext = @import("fields_ext.zig");
const size_mod = @import("size.zig");
const color_mod = @import("color.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const U32Enc = fields.U32Enc;
const EnumCoder = fields_ext.EnumCoder;
const Name = fields_ext.Name;
const SizeHeader = size_mod.SizeHeader;
const ColorEncoding = color_mod.ColorEncoding;

pub const ExtraChannel = enum(u32) {
    alpha = 0,
    depth = 1,
    spot_color = 2,
    selection_mask = 3,
    black = 4,
    cfa = 5,
    thermal = 6,
    reserved0 = 7,
    reserved1 = 8,
    reserved2 = 9,
    reserved3 = 10,
    reserved4 = 11,
    reserved5 = 12,
    reserved6 = 13,
    reserved7 = 14,
    unknown = 15,
    optional = 16,

    pub const valid_mask: u64 = (1 << 17) - 1;
};

pub const Orientation = enum(u32) {
    identity = 1,
    flip_horizontal = 2,
    rotate_180 = 3,
    flip_vertical = 4,
    transpose = 5,
    rotate_90_cw = 6,
    anti_transpose = 7,
    rotate_90_ccw = 8,

    pub const valid_mask: u64 = (1 << 8) - 1 - 1; // values 1..8
};

const bits_per_sample_enc = U32Enc.init(fields.val(8), fields.val(10), fields.val(12), fields.bitsOffset(6, 1));
const float_bits_per_sample_enc = U32Enc.init(fields.val(32), fields.val(16), fields.val(24), fields.bitsOffset(6, 1));
const num_extra_channels_enc = U32Enc.init(fields.val(0), fields.val(1), fields.bitsOffset(4, 2), fields.bitsOffset(12, 1));
const dim_shift_enc = U32Enc.init(fields.val(0), fields.val(3), fields.val(4), fields.bitsOffset(3, 1));

pub const BitDepth = struct {
    floating_point_sample: bool = false,
    bits_per_sample: u32 = 8,
    exponent_bits_per_sample: u32 = 0,

    pub fn isDefault(self: *const BitDepth) bool {
        return !self.floating_point_sample and self.bits_per_sample == 8;
    }

    pub fn read(self: *BitDepth, br: *BitReader) !void {
        self.* = .{};
        self.floating_point_sample = (br.readBits(1)) != 0;
        if (!self.floating_point_sample) {
            self.bits_per_sample = U32Coder.read(bits_per_sample_enc, br);
            self.exponent_bits_per_sample = 0;
            if (self.bits_per_sample > 31) return error.InvalidBitDepth;
        } else {
            self.bits_per_sample = U32Coder.read(float_bits_per_sample_enc, br);
            // The encoded value is exponent_bits_per_sample - 1 (4 bits),
            // so the decoded value is in [1, 16] before validation
            // (libjxl BitDepth::VisitFields applies the +1 offset).
            self.exponent_bits_per_sample = @intCast(br.readBits(4) + 1);
            if (self.exponent_bits_per_sample < 2 or self.exponent_bits_per_sample > 8) {
                return error.InvalidBitDepth;
            }
            const mantissa = self.bits_per_sample - self.exponent_bits_per_sample - 1;
            if (mantissa < 2 or mantissa > 23) return error.InvalidBitDepth;
        }
    }

    pub fn write(self: *const BitDepth, writer: *BitWriter) !void {
        writer.write(1, @intFromBool(self.floating_point_sample));
        if (!self.floating_point_sample) {
            try U32Coder.write(bits_per_sample_enc, self.bits_per_sample, writer);
        } else {
            try U32Coder.write(float_bits_per_sample_enc, self.bits_per_sample, writer);
            writer.write(4, self.exponent_bits_per_sample - 1);
        }
    }
};

pub const ExtraChannelInfo = struct {
    all_default: bool = true,
    type: ExtraChannel = .alpha,
    bit_depth: BitDepth = .{},
    dim_shift: u32 = 0,
    name: Name = .{},
    alpha_associated: bool = false,

    pub fn isDefault(self: *const ExtraChannelInfo) bool {
        return self.type == .alpha and self.bit_depth.isDefault() and
            self.dim_shift == 0 and self.name.value.len == 0 and !self.alpha_associated;
    }

    pub fn read(self: *ExtraChannelInfo, allocator: std.mem.Allocator, br: *BitReader) !void {
        self.* = .{};
        self.all_default = (br.readBits(1)) != 0;
        if (self.all_default) return;
        self.type = try EnumCoder(ExtraChannel).read(.alpha, br);
        try self.bit_depth.read(br);
        self.dim_shift = U32Coder.read(dim_shift_enc, br);
        if (self.dim_shift > 3) return error.InvalidDimShift;
        try self.name.read(allocator, br);
        switch (self.type) {
            .alpha => self.alpha_associated = (br.readBits(1)) != 0,
            .spot_color => {
                for (0..3) |_| _ = try fields_ext.readF16(br);
            },
            .cfa => {
                _ = U32Coder.read(cfa_channel_enc, br);
            },
            .unknown, .reserved0, .reserved1, .reserved2, .reserved3, .reserved4, .reserved5, .reserved6, .reserved7 => {
                return error.UnknownExtraChannel;
            },
            else => {},
        }
    }

    pub fn write(self: *const ExtraChannelInfo, writer: *BitWriter) !void {
        const all_default = self.isDefault();
        writer.write(1, @intFromBool(all_default));
        if (all_default) return;
        try EnumCoder(ExtraChannel).write(self.type, writer);
        try self.bit_depth.write(writer);
        try U32Coder.write(dim_shift_enc, self.dim_shift, writer);
        try Name.write(self.name.value, writer);
        if (self.type == .alpha) {
            writer.write(1, @intFromBool(self.alpha_associated));
        }
    }

    const cfa_channel_enc = U32Enc.init(fields.val(1), fields.bitsDistr(2), fields.bitsOffset(4, 3), fields.bitsOffset(8, 19));
};

pub const ToneMapping = struct {
    all_default: bool = true,
    intensity_target: f32 = 255.0,
    min_nits: f32 = 0,
    relative_to_max_display: bool = false,
    linear_below: f32 = 0,

    pub fn isDefault(self: *const ToneMapping) bool {
        return self.all_default;
    }

    pub fn read(self: *ToneMapping, br: *BitReader) !void {
        self.* = .{};
        self.all_default = (br.readBits(1)) != 0;
        if (self.all_default) return;
        self.intensity_target = try fields_ext.readF16(br);
        self.min_nits = try fields_ext.readF16(br);
        self.relative_to_max_display = (br.readBits(1)) != 0;
        self.linear_below = try fields_ext.readF16(br);
    }

    pub fn write(self: *const ToneMapping, writer: *BitWriter) !void {
        writer.write(1, @intFromBool(self.all_default));
        if (self.all_default) return;
        try fields_ext.writeF16(self.intensity_target, writer);
        try fields_ext.writeF16(self.min_nits, writer);
        writer.write(1, @intFromBool(self.relative_to_max_display));
        try fields_ext.writeF16(self.linear_below, writer);
    }
};

pub const AnimationHeader = struct {
    tps_numerator: u32 = 1000,
    tps_denominator: u32 = 1,
    num_loops: u32 = 0,
    have_timecodes: bool = false,

    const tps_num_enc = U32Enc.init(fields.val(100), fields.val(1000), fields.bitsOffset(10, 1), fields.bitsOffset(30, 1));
    const tps_den_enc = U32Enc.init(fields.val(1), fields.val(1001), fields.bitsOffset(8, 1), fields.bitsOffset(10, 1));
    const num_loops_enc = U32Enc.init(fields.val(0), fields.bitsDistr(3), fields.bitsDistr(16), fields.bitsDistr(32));

    pub fn read(self: *AnimationHeader, br: *BitReader) !void {
        self.* = .{};
        self.tps_numerator = U32Coder.read(tps_num_enc, br);
        self.tps_denominator = U32Coder.read(tps_den_enc, br);
        self.num_loops = U32Coder.read(num_loops_enc, br);
        self.have_timecodes = (br.readBits(1)) != 0;
    }
};

pub const ImageMetadata = struct {
    all_default: bool = true,
    extra_fields: bool = false,
    orientation: Orientation = .identity,
    have_intrinsic_size: bool = false,
    intrinsic_size: SizeHeader = .{},
    have_preview: bool = false,
    preview_size: size_mod.PreviewHeader = .{},
    have_animation: bool = false,
    animation: AnimationHeader = .{},
    bit_depth: BitDepth = .{},
    modular_16_bit_buffer_sufficient: bool = true,
    extra_channel_info: std.ArrayList(ExtraChannelInfo) = .empty,
    xyb_encoded: bool = true,
    color_encoding: ColorEncoding = .{},
    tone_mapping: ToneMapping = .{},
    extensions: u64 = 0,
    /// Raw ICC profile bytes when `color_encoding.want_icc` is set. Not part
    /// of the ImageMetadata bundle serialization: the compressed ICC stream
    /// is written (read) right after CustomTransformData at the codestream
    /// level, exactly like libjxl's `WriteICC`/`ICCReader`.
    icc: std.ArrayList(u8) = .empty,

    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) ImageMetadata {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *ImageMetadata) void {
        for (self.extra_channel_info.items) |*eci| {
            if (eci.name.allocator) |a| a.free(eci.name.value);
        }
        self.extra_channel_info.deinit(self.allocator);
        self.icc.deinit(self.allocator);
    }

    pub fn numExtraChannels(self: *const ImageMetadata) usize {
        return self.extra_channel_info.items.len;
    }

    pub fn hasAlpha(self: *const ImageMetadata) bool {
        for (self.extra_channel_info.items) |eci| {
            if (eci.type == .alpha) return true;
        }
        return false;
    }

    pub fn read(self: *ImageMetadata, br: *BitReader) !void {
        self.all_default = (br.readBits(1)) != 0;
        if (self.all_default) {
            self.* = .{ .allocator = self.allocator };
            return;
        }
        self.extra_fields = (br.readBits(1)) != 0;
        if (self.extra_fields) {
            const orientation_raw: u32 = @intCast(br.readBits(3));
            if (orientation_raw + 1 > 8) return error.InvalidOrientation;
            self.orientation = @enumFromInt(orientation_raw + 1);
            self.have_intrinsic_size = (br.readBits(1)) != 0;
            if (self.have_intrinsic_size) {
                try self.intrinsic_size.read(br);
            }
            self.have_preview = (br.readBits(1)) != 0;
            if (self.have_preview) {
                try self.preview_size.read(br);
            }
            self.have_animation = (br.readBits(1)) != 0;
            if (self.have_animation) {
                try self.animation.read(br);
            }
        } else {
            self.orientation = .identity;
        }
        try self.bit_depth.read(br);
        self.modular_16_bit_buffer_sufficient = (br.readBits(1)) != 0;
        const num_extra = U32Coder.read(num_extra_channels_enc, br);
        try self.extra_channel_info.resize(self.allocator, num_extra);
        for (self.extra_channel_info.items) |*eci| {
            try eci.read(self.allocator, br);
        }
        self.xyb_encoded = (br.readBits(1)) != 0;
        try self.color_encoding.read(br);
        if (self.extra_fields) {
            try self.tone_mapping.read(br);
        }
        self.extensions = fields.U64Coder.read(br);
    }

    pub fn write(self: *ImageMetadata, writer: *BitWriter) !void {
        const tone_mapping_default = self.tone_mapping.isDefault();
        const all_default = self.bit_depth.isDefault() and
            self.modular_16_bit_buffer_sufficient and
            self.extra_channel_info.items.len == 0 and
            self.xyb_encoded and
            self.color_encoding.isAllDefault() and
            self.orientation == .identity and
            !self.have_intrinsic_size and !self.have_preview and !self.have_animation and
            tone_mapping_default;
        self.all_default = all_default;
        writer.write(1, @intFromBool(self.all_default));
        if (self.all_default) return;

        const extra_fields = self.orientation != .identity or self.have_preview or
            self.have_animation or self.have_intrinsic_size or !tone_mapping_default;
        writer.write(1, @intFromBool(extra_fields));
        if (extra_fields) {
            writer.write(3, @intFromEnum(self.orientation) - 1);
            writer.write(1, @intFromBool(self.have_intrinsic_size));
            if (self.have_intrinsic_size) try self.intrinsic_size.write(writer);
            writer.write(1, @intFromBool(self.have_preview));
            if (self.have_preview) try self.preview_size.write(writer);
            writer.write(1, @intFromBool(self.have_animation));
            if (self.have_animation) {
                try U32Coder.write(AnimationHeader.tps_num_enc, self.animation.tps_numerator, writer);
                try U32Coder.write(AnimationHeader.tps_den_enc, self.animation.tps_denominator, writer);
                try U32Coder.write(AnimationHeader.num_loops_enc, self.animation.num_loops, writer);
                writer.write(1, @intFromBool(self.animation.have_timecodes));
            }
        }
        try self.bit_depth.write(writer);
        writer.write(1, @intFromBool(self.modular_16_bit_buffer_sufficient));
        try U32Coder.write(num_extra_channels_enc, @intCast(self.extra_channel_info.items.len), writer);
        for (self.extra_channel_info.items) |*eci| {
            try eci.write(writer);
        }
        writer.write(1, @intFromBool(self.xyb_encoded));
        try self.color_encoding.write(writer);
        if (extra_fields) {
            try self.tone_mapping.write(writer);
        }
        fields.U64Coder.write(self.extensions, writer);
    }
};

/// CustomTransformData bundle: the opsin inverse matrix (XYB only, out of
/// scope) and the custom upsampling weight sets (libjxl
/// image_metadata.cc CustomTransformData::VisitFields). The defaults are
/// the NLEE kernel triangles from `base/nlee.zig`.
pub const CustomTransformData = struct {
    all_default: bool = true,
    /// Bit i set: the factor-(2^(i+1)) weight set is serialized.
    custom_weights_mask: u32 = 0,
    upsampling2_weights: [15]f32 = nlee.kWeights2,
    upsampling4_weights: [55]f32 = nlee.kWeights4,
    upsampling8_weights: [210]f32 = nlee.kWeights8,

    pub fn read(self: *CustomTransformData, br: *BitReader, xyb_encoded: bool) !void {
        self.* = .{};
        self.all_default = (br.readBits(1)) != 0;
        if (self.all_default) return;
        // OpsinInverseMatrix: only serialized when xyb_encoded (a
        // nonserialized flag set by the caller from ImageMetadata, not a
        // bit in the stream). The port's scope is non-XYB; consume the 9
        // F16 entries and reject, like the XYB paths elsewhere.
        if (xyb_encoded) {
            for (0..9) |_| _ = try fields_ext.readF16(br);
            return error.UnsupportedCustomTransform;
        }
        self.custom_weights_mask = @intCast(br.readBits(3));
        if (self.custom_weights_mask & 1 != 0) {
            for (&self.upsampling2_weights) |*w| w.* = try fields_ext.readF16(br);
        }
        if (self.custom_weights_mask & 2 != 0) {
            for (&self.upsampling4_weights) |*w| w.* = try fields_ext.readF16(br);
        }
        if (self.custom_weights_mask & 4 != 0) {
            for (&self.upsampling8_weights) |*w| w.* = try fields_ext.readF16(br);
        }
    }

    pub fn write(self: *const CustomTransformData, writer: *BitWriter) !void {
        // (The opsin matrix is XYB-only and never written by this port, so
        // mask == 0 implies the all-default bundle.)
        const all_default = self.custom_weights_mask == 0;
        writer.write(1, @intFromBool(all_default));
        if (all_default) return;
        writer.write(3, self.custom_weights_mask);
        if (self.custom_weights_mask & 1 != 0) {
            for (self.upsampling2_weights) |w| try fields_ext.writeF16(w, writer);
        }
        if (self.custom_weights_mask & 2 != 0) {
            for (self.upsampling4_weights) |w| try fields_ext.writeF16(w, writer);
        }
        if (self.custom_weights_mask & 4 != 0) {
            for (self.upsampling8_weights) |w| try fields_ext.writeF16(w, writer);
        }
    }

    /// The weight triangle for an upsampling factor (2, 4 or 8); defaults
    /// when the mask bit is clear.
    pub fn weightsFor(self: *const CustomTransformData, factor: u32) []const f32 {
        return switch (factor) {
            2 => if (self.custom_weights_mask & 1 != 0) &self.upsampling2_weights else &nlee.kWeights2,
            4 => if (self.custom_weights_mask & 2 != 0) &self.upsampling4_weights else &nlee.kWeights4,
            8 => if (self.custom_weights_mask & 4 != 0) &self.upsampling8_weights else &nlee.kWeights8,
            else => &nlee.kWeights2,
        };
    }
};

test "image metadata minimal rgb8 roundtrip" {
    const alloc = std.testing.allocator;
    var m = ImageMetadata.init(alloc);
    defer m.deinit();
    var w = BitWriter.init(alloc);
    defer w.deinit();
    try m.write(&w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    var got = ImageMetadata.init(alloc);
    defer got.deinit();
    try got.read(&br);
    try br.close();
    try std.testing.expect(got.all_default);
    try std.testing.expectEqual(@as(u32, 8), got.bit_depth.bits_per_sample);
}

test "image metadata 16bit with alpha roundtrip" {
    const alloc = std.testing.allocator;
    var m = ImageMetadata.init(alloc);
    defer m.deinit();
    m.bit_depth.bits_per_sample = 16;
    m.color_encoding = ColorEncoding.graySrgb();
    try m.extra_channel_info.append(alloc, .{ .all_default = false });
    m.extra_channel_info.items[0].all_default = false;
    m.extra_channel_info.items[0].bit_depth.bits_per_sample = 16;
    var w = BitWriter.init(alloc);
    defer w.deinit();
    try m.write(&w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    var got = ImageMetadata.init(alloc);
    defer got.deinit();
    try got.read(&br);
    try br.close();
    try std.testing.expectEqual(@as(u32, 16), got.bit_depth.bits_per_sample);
    try std.testing.expectEqual(@as(usize, 1), got.extra_channel_info.items.len);
    try std.testing.expectEqual(ExtraChannel.alpha, got.extra_channel_info.items[0].type);
    try std.testing.expectEqual(@as(u32, 16), got.extra_channel_info.items[0].bit_depth.bits_per_sample);
    try std.testing.expect(!got.all_default);
}
