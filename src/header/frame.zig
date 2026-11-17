//! FrameHeader bundle, port of libjxl `frame_header.h/cc`.
//! The modular path is fully supported; the VarDCT path is parsed and used
//! by the JPEG-transcode stage (kVarDCT frames carrying JPEG coefficients
//! with RAW quant tables, cf. libjxl ComputeJPEGTranscodingData).

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const fields_ext = @import("fields_ext.zig");
const metadata_mod = @import("metadata.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const U64Coder = fields.U64Coder;
const U32Enc = fields.U32Enc;
const Name = fields_ext.Name;
const ImageMetadata = metadata_mod.ImageMetadata;

pub const ColorTransform = enum(u32) {
    xyb = 0,
    none = 1,
    ycbcr = 2,
};

pub const FrameType = enum(u32) {
    regular = 0,
    dc_frame = 1,
    reference_only = 2,
    skip_progressive = 3,

    pub const valid_mask: u64 = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
};

pub const FrameEncoding = enum(u32) {
    var_dct = 0,
    modular = 1,
};

pub const BlendMode = enum(u32) {
    replace = 0,
    add = 1,
    blend = 2,
    alpha_weighted_add = 3,
    mul = 4,

    pub const valid_mask: u64 = (1 << 4) * 2 - 1;
};

const frame_type_enc = U32Enc.init(fields.val(0), fields.val(1), fields.val(2), fields.val(3));
const blend_mode_enc = U32Enc.init(fields.val(0), fields.val(1), fields.val(2), fields.bitsOffset(2, 3));
const duration_enc = U32Enc.init(fields.val(0), fields.val(1), fields.bitsDistr(8), fields.bitsDistr(32));
const alpha_channel_enc = U32Enc.init(fields.val(0), fields.val(1), fields.val(2), fields.bitsOffset(3, 3));
const source_enc = U32Enc.init(fields.val(0), fields.val(1), fields.val(2), fields.val(3));
const num_passes_enc = U32Enc.init(fields.val(1), fields.val(2), fields.val(3), fields.bitsOffset(3, 4));
const num_downsample_enc = U32Enc.init(fields.val(0), fields.val(1), fields.val(2), fields.bitsOffset(1, 3));
const downsample_enc = U32Enc.init(fields.val(1), fields.val(2), fields.val(4), fields.val(8));
const last_pass_enc = U32Enc.init(fields.val(0), fields.val(1), fields.val(2), fields.bitsDistr(3));
const upsampling_enc = U32Enc.init(fields.val(1), fields.val(2), fields.val(4), fields.val(8));
const dc_level_enc = U32Enc.init(fields.val(1), fields.val(2), fields.val(3), fields.val(4));
const frame_size_enc = U32Enc.init(
    fields.bitsDistr(8),
    fields.bitsOffset(11, 256),
    fields.bitsOffset(14, 2304),
    fields.bitsOffset(30, 18688),
);

/// FrameHeader::Flags bit values.
pub const Flags = struct {
    pub const kNoise: u64 = 1;
    pub const kPatches: u64 = 2;
    pub const kSplines: u64 = 16;
    pub const kUseDcFrame: u64 = 32;
    pub const kSkipAdaptiveDCSmoothing: u64 = 128;
};

/// Port of `YCbCrChromaSubsampling` (3 x 2-bit channel modes).
/// kHShift/kVShift: per-mode raw subsampling factor exponents; the effective
/// shift of channel c is maxhs - kHShift[channel_mode[c]] (JPEG component
/// sampling factors normalized by the maximum).
pub const YCbCrChromaSubsampling = struct {
    channel_mode: [3]u32 = .{ 0, 0, 0 },
    maxhs: u8 = 0,
    maxvs: u8 = 0,

    pub const kHShift = [4]u8{ 0, 1, 1, 0 };
    pub const kVShift = [4]u8{ 0, 1, 0, 1 };

    fn recompute(self: *YCbCrChromaSubsampling) void {
        var maxhs: u8 = 0;
        var maxvs: u8 = 0;
        for (self.channel_mode) |m| {
            maxhs = @max(maxhs, kHShift[m]);
            maxvs = @max(maxvs, kVShift[m]);
        }
        self.maxhs = maxhs;
        self.maxvs = maxvs;
    }

    pub fn hShift(self: *const YCbCrChromaSubsampling, c: usize) usize {
        return self.maxhs - kHShift[self.channel_mode[c]];
    }

    pub fn vShift(self: *const YCbCrChromaSubsampling, c: usize) usize {
        return self.maxvs - kVShift[self.channel_mode[c]];
    }

    pub fn maxHShift(self: *const YCbCrChromaSubsampling) usize {
        return self.maxhs;
    }

    pub fn maxVShift(self: *const YCbCrChromaSubsampling) usize {
        return self.maxvs;
    }

    pub fn rawHShift(self: *const YCbCrChromaSubsampling, c: usize) usize {
        return kHShift[self.channel_mode[c]];
    }

    pub fn rawVShift(self: *const YCbCrChromaSubsampling, c: usize) usize {
        return kVShift[self.channel_mode[c]];
    }

    pub fn is444(self: *const YCbCrChromaSubsampling) bool {
        return self.hShift(0) == 0 and self.vShift(0) == 0 and
            self.hShift(2) == 0 and self.vShift(2) == 0 and
            self.hShift(1) == 0 and self.vShift(1) == 0;
    }

    /// Sets modes from JPEG component sampling factors. `hsample`/`vsample`
    /// are indexed in JPEG component order (Y, Cb, Cr); jxl channel c maps to
    /// JPEG component c<2 ? c^1 : c (cf. `YCbCrChromaSubsampling::Set`).
    pub fn setFromJpegSampling(self: *YCbCrChromaSubsampling, hsample: [3]u32, vsample: [3]u32) !void {
        for (0..3) |c| {
            const cjpeg: usize = if (c < 2) c ^ 1 else c;
            var found: ?u32 = null;
            for (0..4) |i| {
                if ((@as(u32, 1) << @intCast(kHShift[i])) == hsample[cjpeg] and
                    (@as(u32, 1) << @intCast(kVShift[i])) == vsample[cjpeg])
                {
                    found = @intCast(i);
                    break;
                }
            }
            self.channel_mode[c] = found orelse return error.InvalidSubsampleMode;
        }
        self.recompute();
    }

    pub fn read(self: *YCbCrChromaSubsampling, br: *BitReader) !void {
        for (&self.channel_mode) |*m| {
            m.* = @intCast(br.readBits(2));
        }
        self.recompute();
    }

    pub fn write(self: *const YCbCrChromaSubsampling, writer: *BitWriter) !void {
        for (self.channel_mode) |m| {
            writer.write(2, m);
        }
    }
};

pub const Passes = struct {
    num_passes: u32 = 1,
    num_downsample: u32 = 0,
    shift: [4]u32 = .{ 0, 0, 0, 0 },
    downsample: [4]u32 = .{ 1, 1, 1, 1 },
    last_pass: [4]u32 = .{ 0, 0, 0, 0 },

    pub fn isDefault(self: *const Passes) bool {
        return self.num_passes == 1;
    }

    pub fn read(self: *Passes, br: *BitReader) !void {
        self.* = .{};
        self.num_passes = U32Coder.read(num_passes_enc, br);
        if (self.num_passes > 8) return error.InvalidNumPasses;
        if (self.num_passes != 1) {
            self.num_downsample = U32Coder.read(num_downsample_enc, br);
            if (self.num_downsample > self.num_passes) return error.InvalidNumPasses;
            for (0..self.num_passes - 1) |i| {
                self.shift[i] = @intCast(br.readBits(2));
            }
            self.shift[self.num_passes - 1] = 0;
            for (0..self.num_downsample) |i| {
                self.downsample[i] = U32Coder.read(downsample_enc, br);
                if (i > 0 and self.downsample[i] >= self.downsample[i - 1]) {
                    return error.InvalidDownsample;
                }
            }
            for (0..self.num_downsample) |i| {
                self.last_pass[i] = U32Coder.read(last_pass_enc, br);
                if (i > 0 and self.last_pass[i] <= self.last_pass[i - 1]) {
                    return error.InvalidDownsample;
                }
                if (self.last_pass[i] >= self.num_passes) return error.InvalidDownsample;
            }
        }
    }

    pub fn write(self: *const Passes, writer: *BitWriter) !void {
        try U32Coder.write(num_passes_enc, self.num_passes, writer);
        if (self.num_passes != 1) {
            try U32Coder.write(num_downsample_enc, self.num_downsample, writer);
            for (0..self.num_passes - 1) |i| {
                writer.write(2, self.shift[i]);
            }
            for (0..self.num_downsample) |i| {
                try U32Coder.write(downsample_enc, self.downsample[i], writer);
            }
            for (0..self.num_downsample) |i| {
                try U32Coder.write(last_pass_enc, self.last_pass[i], writer);
            }
        }
    }

    /// Port of `Passes::GetDownsamplingBracket` (frame_header.h): the
    /// inclusive [min_shift, max_shift] range of channel min-shifts
    /// decoded in the AC group sections of `pass`. Channels with
    /// min-shift >= 3 always travel in the modular DC group streams;
    /// the last pass always carries the shift-0 (full resolution)
    /// channels.
    pub fn getDownsamplingBracket(self: *const Passes, pass: usize, min_shift: *i32, max_shift: *i32) void {
        max_shift.* = 2;
        min_shift.* = 3;
        var i: usize = 0;
        while (true) : (i += 1) {
            for (0..self.num_downsample) |j| {
                if (i == self.last_pass[j]) {
                    // downsample 8 -> minShift 3; 4 -> 2; 2 -> 1; 1 -> 0.
                    min_shift.* = switch (self.downsample[j]) {
                        8 => 3,
                        4 => 2,
                        2 => 1,
                        1 => 0,
                        else => min_shift.*,
                    };
                }
            }
            if (i == self.num_passes - 1) min_shift.* = 0;
            if (i == pass) return;
            max_shift.* = min_shift.* - 1;
        }
    }
};

pub const BlendingInfo = struct {
    mode: BlendMode = .replace,
    alpha_channel: u32 = 0,
    clamp: bool = false,
    source: u32 = 1,
    nonserialized_num_extra_channels: usize = 0,
    nonserialized_is_partial_frame: bool = false,

    pub fn isDefault(self: *const BlendingInfo) bool {
        return self.mode == .replace and self.source == 1;
    }

    pub fn read(self: *BlendingInfo, br: *BitReader) !void {
        // Preserve the caller-set nonserialized fields across the reset
        // (frame.zig sets them before calling; wiping them here would make
        // partial-frame replace sources and blend-mode alpha channels
        // unparseable - libjxl reads them via the same conditions).
        const num_extra_channels = self.nonserialized_num_extra_channels;
        const is_partial_frame = self.nonserialized_is_partial_frame;
        self.* = .{};
        self.nonserialized_num_extra_channels = num_extra_channels;
        self.nonserialized_is_partial_frame = is_partial_frame;
        const mode_raw = U32Coder.read(blend_mode_enc, br);
        if (mode_raw > 4) return error.InvalidBlendMode;
        self.mode = @enumFromInt(mode_raw);
        const blending = self.nonserialized_num_extra_channels > 0 and
            (self.mode == .blend or self.mode == .alpha_weighted_add);
        if (blending) {
            self.alpha_channel = U32Coder.read(alpha_channel_enc, br);
            if (self.alpha_channel >= self.nonserialized_num_extra_channels) {
                return error.InvalidBlendMode;
            }
        }
        if (blending or self.mode == .mul) {
            self.clamp = (br.readBits(1)) != 0;
        }
        if (self.mode != .replace or self.nonserialized_is_partial_frame) {
            self.source = U32Coder.read(source_enc, br);
        }
    }

    pub fn write(self: *const BlendingInfo, writer: *BitWriter) !void {
        try U32Coder.write(blend_mode_enc, @intFromEnum(self.mode), writer);
        const blending = self.nonserialized_num_extra_channels > 0 and
            (self.mode == .blend or self.mode == .alpha_weighted_add);
        if (blending) {
            try U32Coder.write(alpha_channel_enc, self.alpha_channel, writer);
        }
        if (blending or self.mode == .mul) {
            writer.write(1, @intFromBool(self.clamp));
        }
        if (self.mode != .replace or self.nonserialized_is_partial_frame) {
            try U32Coder.write(source_enc, self.source, writer);
        }
    }
};

pub const LoopFilter = struct {
    all_default: bool = true,
    gab: bool = false,
    epf_iters: u32 = 0,
    /// Nonserialized: true for modular frames — gates the
    /// `epf_sharp_custom` bit (VarDCT-only) and the
    /// `epf_sigma_for_modular` F16 (modular-only) of loop_filter.cc's
    /// `nonserialized_is_modular`.
    nonserialized_is_modular: bool = true,

    pub fn isDefault(self: *const LoopFilter) bool {
        return !self.gab and self.epf_iters == 0;
    }

    pub fn read(self: *LoopFilter, br: *BitReader) !void {
        const is_modular = self.nonserialized_is_modular;
        self.all_default = (br.readBits(1)) != 0;
        if (self.all_default) {
            // Format default is gab=true, epf_iters=2 (VarDCT-oriented).
            self.gab = true;
            self.epf_iters = 2;
            return;
        }
        self.gab = (br.readBits(1)) != 0;
        if (self.gab) {
            const gab_custom = (br.readBits(1)) != 0;
            if (gab_custom) return error.UnsupportedLoopFilter;
        }
        self.epf_iters = @intCast(br.readBits(2));
        if (self.epf_iters > 0) {
            // epf_sharp_custom is only read for non-modular frames;
            // custom weights/sigma remain rejected either way.
            if (!is_modular) {
                const epf_sharp_custom = (br.readBits(1)) != 0;
                if (epf_sharp_custom) return error.UnsupportedLoopFilter;
            }
            const epf_weight_custom = (br.readBits(1)) != 0;
            if (epf_weight_custom) return error.UnsupportedLoopFilter;
            const epf_sigma_custom = (br.readBits(1)) != 0;
            if (epf_sigma_custom) return error.UnsupportedLoopFilter;
            // For modular frames the sigma scale factor follows
            // (`epf_sigma_for_modular`, F16).
            if (is_modular) {
                _ = try fields_ext.readF16(br);
            }
        }
        // LoopFilter carries its own extensions field (libjxl
        // `BeginExtensions(&extensions)`); nonzero extensions are rejected.
        const lf_extensions = U64Coder.read(br);
        if (lf_extensions != 0) return error.UnsupportedLoopFilter;
    }

    pub fn write(self: *const LoopFilter, writer: *BitWriter) !void {
        // all_default refers to the format defaults (gab=true, epf=2), which
        // are VarDCT-oriented; modular lossless never matches them.
        const all_default = self.gab and self.epf_iters == 2;
        writer.write(1, @intFromBool(all_default));
        if (all_default) {
            U64Coder.write(0, writer); // LoopFilter extensions
            return;
        }
        writer.write(1, @intFromBool(self.gab));
        if (self.gab) writer.write(1, 0); // gab_custom = false
        writer.write(2, self.epf_iters);
        if (self.epf_iters > 0) {
            if (!self.nonserialized_is_modular) {
                writer.write(1, 0); // epf_sharp_custom = false
            }
            writer.write(1, 0); // epf_weight_custom = false
            writer.write(1, 0); // epf_sigma_custom = false
            if (self.nonserialized_is_modular) {
                try fields_ext.writeF16(1.0, writer); // epf_sigma_for_modular
            }
        }
        U64Coder.write(0, writer); // LoopFilter extensions
    }
};

/// libjxl `PackSigned`: non-negative X -> 2X, negative -X -> 2X-1.
fn packSigned(v: i32) u32 {
    const u: u32 = @bitCast(v);
    return (u << 1) ^ ((~u >> 31) -% 1);
}

/// libjxl `UnpackSigned` (inverse of packSigned).
fn unpackSigned(v: u32) i32 {
    return @bitCast((v >> 1) ^ ((~v & 1) -% 1));
}

pub const FrameHeader = struct {
    all_default: bool = false,
    frame_type: FrameType = .regular,
    encoding: FrameEncoding = .modular,
    flags: u64 = 0,
    color_transform: ColorTransform = .none,
    chroma_subsampling: YCbCrChromaSubsampling = .{},
    upsampling: u32 = 1,
    extra_channel_upsampling: std.ArrayList(u32) = .empty,
    group_size_shift: u32 = 1,
    x_qm_scale: u32 = 2,
    b_qm_scale: u32 = 2,
    passes: Passes = .{},
    dc_level: u32 = 0,
    custom_size_or_origin: bool = false,
    frame_origin_x: i32 = 0,
    frame_origin_y: i32 = 0,
    frame_size_x: u32 = 0,
    frame_size_y: u32 = 0,
    blending_info: BlendingInfo = .{},
    extra_channel_blending_info: std.ArrayList(BlendingInfo) = .empty,
    animation_frame_duration: u32 = 0,
    animation_frame_timecode: u32 = 0,
    is_last: bool = true,
    save_as_reference: u32 = 0,
    save_before_color_transform: bool = false,
    name: Name = .{},
    loop_filter: LoopFilter = .{},
    extensions: u64 = 0,

    allocator: std.mem.Allocator,
    nonserialized_is_preview: bool = false,

    pub fn init(allocator: std.mem.Allocator) FrameHeader {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *FrameHeader) void {
        self.extra_channel_upsampling.deinit(self.allocator);
        self.extra_channel_blending_info.deinit(self.allocator);
        if (self.name.allocator) |a| a.free(self.name.value);
    }

    pub fn isModular(self: *const FrameHeader) bool {
        return self.encoding == .modular;
    }

    /// Port of FrameHeader::CanBeReferenced: DC frames cannot be referenced,
    /// the last frame cannot, and a duration-0 frame is always referenced
    /// (it only exists to be referenced).
    pub fn canBeReferenced(self: *const FrameHeader) bool {
        return !self.is_last and self.frame_type != .dc_frame and
            (self.animation_frame_duration == 0 or self.save_as_reference != 0);
    }

    /// The default kRegularFrame/kModular frame used by the lossless encoder.
    pub fn modularDefault(allocator: std.mem.Allocator, m: *const ImageMetadata, group_size_shift: u32) FrameHeader {
        _ = m;
        var fh = FrameHeader.init(allocator);
        fh.encoding = .modular;
        fh.color_transform = .none;
        fh.group_size_shift = group_size_shift;
        fh.is_last = true;
        fh.loop_filter = .{ .gab = false, .epf_iters = 0 };
        return fh;
    }

    pub fn read(self: *FrameHeader, m: *const ImageMetadata, br: *BitReader) !void {
        self.all_default = (br.readBits(1)) != 0;
        if (self.all_default) {
            // Default frame: VarDCT kRegularFrame. Not usable for modular.
            return error.UnsupportedVarDct;
        }
        const frame_type_raw = U32Coder.read(frame_type_enc, br);
        if (frame_type_raw > 3) return error.InvalidFrameType;
        self.frame_type = @enumFromInt(frame_type_raw);
        const is_modular = (br.readBits(1)) != 0;
        self.encoding = if (is_modular) .modular else .var_dct;
        self.flags = U64Coder.read(br);
        // kPatches is decoded (the patch dictionary travels in the DC
        // global section; the still/animation decoders read it); noise,
        // splines and DC frames remain unsupported.
        if (self.flags & Flags.kNoise != 0 or self.flags & Flags.kSplines != 0 or
            self.flags & Flags.kUseDcFrame != 0)
        {
            return error.UnsupportedFrameFeatures;
        }

        if (m.xyb_encoded) {
            self.color_transform = .xyb;
        } else {
            const alternate = (br.readBits(1)) != 0;
            self.color_transform = if (alternate) .ycbcr else .none;
        }

        // Chroma subsampling bundle: present iff YCbCr and no DC frame.
        if (self.color_transform == .ycbcr and (self.flags & Flags.kUseDcFrame) == 0) {
            try self.chroma_subsampling.read(br);
        }

        const num_extra_channels = m.numExtraChannels();
        if ((self.flags & Flags.kUseDcFrame) == 0) {
            self.upsampling = U32Coder.read(upsampling_enc, br);
            if (num_extra_channels != 0) {
                try self.extra_channel_upsampling.resize(self.allocator, num_extra_channels);
                for (m.extra_channel_info.items, 0..) |eci, i| {
                    const dim_shift: u5 = @intCast(@min(31, eci.dim_shift));
                    var ec_upsampling = self.extra_channel_upsampling.items[i] >> dim_shift;
                    ec_upsampling = U32Coder.read(upsampling_enc, br);
                    ec_upsampling <<= dim_shift;
                    self.extra_channel_upsampling.items[i] = ec_upsampling;
                    if (ec_upsampling < self.upsampling or ec_upsampling > 8) {
                        return error.InvalidUpsampling;
                    }
                }
            }
        }

        if (is_modular) {
            self.group_size_shift = @intCast(br.readBits(2));
        } else if (m.xyb_encoded) {
            // VarDCT + XYB: x_qm_scale/b_qm_scale (3 bits each).
            self.x_qm_scale = @intCast(br.readBits(3));
            self.b_qm_scale = @intCast(br.readBits(3));
        } else {
            // VarDCT with a non-XYB color transform: defaults, nothing read.
            self.x_qm_scale = 2;
            self.b_qm_scale = 2;
        }

        if (self.frame_type != .reference_only) {
            try self.passes.read(br);
        }

        if (self.frame_type == .dc_frame) {
            self.dc_level = U32Coder.read(dc_level_enc, br);
        }

        var is_partial_frame = false;
        if (self.frame_type != .dc_frame) {
            self.custom_size_or_origin = (br.readBits(1)) != 0;
            if (self.custom_size_or_origin) {
                if (self.frame_type == .regular or self.frame_type == .skip_progressive) {
                    const ux0 = U32Coder.read(frame_size_enc, br);
                    const uy0 = U32Coder.read(frame_size_enc, br);
                    self.frame_origin_x = unpackSigned(ux0);
                    self.frame_origin_y = unpackSigned(uy0);
                }
                self.frame_size_x = U32Coder.read(frame_size_enc, br);
                self.frame_size_y = U32Coder.read(frame_size_enc, br);
                if (self.frame_size_x == 0 or self.frame_size_y == 0) return error.InvalidFrameSize;
                if (self.frame_type == .regular or self.frame_type == .skip_progressive) {
                    is_partial_frame = self.frame_origin_x > 0 or self.frame_origin_y > 0 or
                        (self.frame_size_x + @as(u32, @intCast(@max(0, self.frame_origin_x)))) < m_xsize(m) or
                        (self.frame_size_y + @as(u32, @intCast(@max(0, self.frame_origin_y)))) < m_ysize(m);
                }
            }
        }

        if (self.frame_type == .regular or self.frame_type == .skip_progressive) {
            self.blending_info.nonserialized_num_extra_channels = num_extra_channels;
            self.blending_info.nonserialized_is_partial_frame = is_partial_frame;
            try self.blending_info.read(br);
            var replace_all = self.blending_info.mode == .replace;
            try self.extra_channel_blending_info.resize(self.allocator, num_extra_channels);
            for (self.extra_channel_blending_info.items) |*ec_bi| {
                ec_bi.nonserialized_is_partial_frame = is_partial_frame;
                ec_bi.nonserialized_num_extra_channels = num_extra_channels;
                try ec_bi.read(br);
                replace_all = replace_all and ec_bi.mode == .replace;
            }
            if (m.have_animation) {
                self.animation_frame_duration = U32Coder.read(duration_enc, br);
                if (m.animation.have_timecodes) {
                    self.animation_frame_timecode = @intCast(br.readBits(32));
                }
            }
            self.is_last = (br.readBits(1)) != 0;
        } else {
            self.is_last = false;
        }

        if (self.frame_type != .dc_frame and !self.is_last) {
            self.save_as_reference = U32Coder.read(source_enc, br);
        }

        if (self.frame_type == .dc_frame) {
            self.save_before_color_transform = true;
        } else if (self.frame_type == .reference_only) {
            // Reference-only frames always carry the bit (libjxl
            // `Bool(true, &save_before_color_transform)`).
            self.save_before_color_transform = (br.readBits(1)) != 0;
        } else {
            if (self.canBeReferenced() and self.blending_info.mode == .replace and !is_partial_frame and
                (self.frame_type == .regular or self.frame_type == .skip_progressive))
            {
                self.save_before_color_transform = (br.readBits(1)) != 0;
            }
        }

        try self.name.read(self.allocator, br);

        self.loop_filter = .{};
        self.loop_filter.nonserialized_is_modular = is_modular;
        try self.loop_filter.read(br);

        self.extensions = U64Coder.read(br);
    }

    fn m_xsize(m: *const ImageMetadata) u32 {
        return m.intrinsic_size.xsizeOf();
    }
    fn m_ysize(m: *const ImageMetadata) u32 {
        return m.intrinsic_size.ysizeOf();
    }

    pub fn write(self: *FrameHeader, m: *const ImageMetadata, writer: *BitWriter) !void {
        writer.write(1, @intFromBool(self.all_default));
        if (self.all_default) return;
        try U32Coder.write(frame_type_enc, @intFromEnum(self.frame_type), writer);
        writer.write(1, @intFromBool(self.encoding == .modular));
        U64Coder.write(self.flags, writer);

        if (m.xyb_encoded) {
            self.color_transform = .xyb;
        } else {
            const alternate = self.color_transform == .ycbcr;
            writer.write(1, @intFromBool(alternate));
        }

        if (self.color_transform == .ycbcr and (self.flags & Flags.kUseDcFrame) == 0) {
            try self.chroma_subsampling.write(writer);
        }

        const num_extra_channels = m.numExtraChannels();
        if ((self.flags & Flags.kUseDcFrame) == 0) {
            try U32Coder.write(upsampling_enc, self.upsampling, writer);
            if (num_extra_channels != 0) {
                for (m.extra_channel_info.items, 0..) |eci, i| {
                    const dim_shift: u5 = @intCast(@min(31, eci.dim_shift));
                    var ec_upsampling = self.extra_channel_upsampling.items[i] >> dim_shift;
                    try U32Coder.write(upsampling_enc, ec_upsampling, writer);
                    ec_upsampling <<= dim_shift;
                    self.extra_channel_upsampling.items[i] = ec_upsampling;
                }
            }
        }

        if (self.encoding == .modular) {
            writer.write(2, self.group_size_shift);
        } else if (m.xyb_encoded) {
            writer.write(3, self.x_qm_scale);
            writer.write(3, self.b_qm_scale);
        }
        // VarDCT + non-XYB: no bits here.

        if (self.frame_type != .reference_only) {
            try self.passes.write(writer);
        }

        if (self.frame_type == .dc_frame) {
            try U32Coder.write(dc_level_enc, self.dc_level, writer);
        }

        var is_partial_frame = false;
        if (self.frame_type != .dc_frame) {
            writer.write(1, @intFromBool(self.custom_size_or_origin));
            if (self.custom_size_or_origin) {
                if (self.frame_type == .regular or self.frame_type == .skip_progressive) {
                    try U32Coder.write(frame_size_enc, packSigned(self.frame_origin_x), writer);
                    try U32Coder.write(frame_size_enc, packSigned(self.frame_origin_y), writer);
                }
                try U32Coder.write(frame_size_enc, self.frame_size_x, writer);
                try U32Coder.write(frame_size_enc, self.frame_size_y, writer);
                if (self.frame_type == .regular or self.frame_type == .skip_progressive) {
                    is_partial_frame = self.frame_origin_x > 0 or self.frame_origin_y > 0 or
                        (self.frame_size_x + @as(u32, @intCast(@max(0, self.frame_origin_x)))) < m_xsize(m) or
                        (self.frame_size_y + @as(u32, @intCast(@max(0, self.frame_origin_y)))) < m_ysize(m);
                }
            }
        }

        if (self.frame_type == .regular or self.frame_type == .skip_progressive) {
            self.blending_info.nonserialized_num_extra_channels = num_extra_channels;
            self.blending_info.nonserialized_is_partial_frame = is_partial_frame;
            try self.blending_info.write(writer);
            for (self.extra_channel_blending_info.items) |*ec_bi| {
                ec_bi.nonserialized_num_extra_channels = num_extra_channels;
                ec_bi.nonserialized_is_partial_frame = is_partial_frame;
                try ec_bi.write(writer);
            }
            if (m.have_animation) {
                try U32Coder.write(duration_enc, self.animation_frame_duration, writer);
                if (m.animation.have_timecodes) {
                    writer.write(32, self.animation_frame_timecode);
                }
            }
            writer.write(1, @intFromBool(self.is_last));
        }

        if (self.frame_type != .dc_frame and !self.is_last) {
            try U32Coder.write(source_enc, self.save_as_reference, writer);
        }

        if (self.frame_type == .dc_frame) {
            self.save_before_color_transform = true;
        } else if (self.frame_type == .reference_only) {
            writer.write(1, @intFromBool(self.save_before_color_transform));
        } else {
            if (self.canBeReferenced() and self.blending_info.mode == .replace and !is_partial_frame and
                (self.frame_type == .regular or self.frame_type == .skip_progressive))
            {
                writer.write(1, @intFromBool(self.save_before_color_transform));
            }
        }

        try Name.write(self.name.value, writer);
        self.loop_filter.nonserialized_is_modular = self.encoding == .modular;
        try self.loop_filter.write(writer);
        U64Coder.write(self.extensions, writer);
    }
};

test "getDownsamplingBracket matches libjxl" {
    const p = struct {
        fn br(passes: Passes, pass: usize) [2]i32 {
            var mn: i32 = 0;
            var mx: i32 = 0;
            passes.getDownsamplingBracket(pass, &mn, &mx);
            return .{ mn, mx };
        }
    };
    // Single pass: everything below shift 3 in the AC sections.
    {
        const passes = Passes{};
        try std.testing.expectEqual([2]i32{ 0, 2 }, p.br(passes, 0));
    }
    // qprogressive (cjxl -p lossless): np=2, (downsample, last_pass)=(2,0).
    {
        const passes = Passes{
            .num_passes = 2,
            .num_downsample = 1,
            .shift = .{ 1, 0, 0, 0 },
            .downsample = .{ 2, 1, 1, 1 },
            .last_pass = .{ 0, 0, 0, 0 },
        };
        try std.testing.expectEqual([2]i32{ 1, 2 }, p.br(passes, 0));
        try std.testing.expectEqual([2]i32{ 0, 0 }, p.br(passes, 1));
    }
    // 3-pass progressive mode (dc_vlf_lf): (4,0), (2,1).
    {
        const passes = Passes{
            .num_passes = 3,
            .num_downsample = 2,
            .shift = .{ 0, 0, 0, 0 },
            .downsample = .{ 4, 2, 1, 1 },
            .last_pass = .{ 0, 1, 0, 0 },
        };
        try std.testing.expectEqual([2]i32{ 2, 2 }, p.br(passes, 0));
        try std.testing.expectEqual([2]i32{ 1, 1 }, p.br(passes, 1));
        try std.testing.expectEqual([2]i32{ 0, 0 }, p.br(passes, 2));
    }
    // (8, 0) bracket: pass 0 has an empty AC bracket — the coarse data
    // travels in the DC group sections (shift >= 3).
    {
        const passes = Passes{
            .num_passes = 3,
            .num_downsample = 2,
            .shift = .{ 0, 0, 0, 0 },
            .downsample = .{ 8, 2, 1, 1 },
            .last_pass = .{ 0, 1, 0, 0 },
        };
        try std.testing.expectEqual([2]i32{ 3, 2 }, p.br(passes, 0));
        try std.testing.expectEqual([2]i32{ 1, 2 }, p.br(passes, 1));
        try std.testing.expectEqual([2]i32{ 0, 0 }, p.br(passes, 2));
    }
    // 4-pass with (8,0), (4,1), (2,2).
    {
        const passes = Passes{
            .num_passes = 4,
            .num_downsample = 3,
            .shift = .{ 0, 0, 0, 0 },
            .downsample = .{ 8, 4, 2, 1 },
            .last_pass = .{ 0, 1, 2, 0 },
        };
        try std.testing.expectEqual([2]i32{ 3, 2 }, p.br(passes, 0));
        try std.testing.expectEqual([2]i32{ 2, 2 }, p.br(passes, 1));
        try std.testing.expectEqual([2]i32{ 1, 1 }, p.br(passes, 2));
        try std.testing.expectEqual([2]i32{ 0, 0 }, p.br(passes, 3));
    }
}

test "passes read/write roundtrip with downsample pairs" {
    const alloc = std.testing.allocator;
    const passes = Passes{
        .num_passes = 3,
        .num_downsample = 2,
        .shift = .{ 1, 2, 0, 0 },
        .downsample = .{ 8, 4, 1, 1 },
        .last_pass = .{ 0, 1, 0, 0 },
    };
    var w = BitWriter.init(alloc);
    defer w.deinit();
    try passes.write(&w);
    var br = BitReader.init(w.getSpan());
    var got = Passes{};
    try got.read(&br);
    try std.testing.expectEqual(passes.num_passes, got.num_passes);
    try std.testing.expectEqual(passes.num_downsample, got.num_downsample);
    try std.testing.expectEqualSlices(u32, &passes.shift, &got.shift);
    try std.testing.expectEqualSlices(u32, &passes.downsample, &got.downsample);
    try std.testing.expectEqualSlices(u32, &passes.last_pass, &got.last_pass);
}
