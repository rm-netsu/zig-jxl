//! Lossless modular frame decoder.
//!
//! Mirrors libjxl dec_frame for the modular path: headers, TOC, DC-global
//! (dequant bit + global tree + histograms + global stream), DC groups
//! (empty for transform-free modular), AC groups.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const float_convert = @import("../base/float_convert.zig");
const dither = @import("../base/dither.zig");
const nlee = @import("../base/nlee.zig");

const size_mod = @import("../header/size.zig");
const metadata_mod = @import("../header/metadata.zig");
const frame_mod = @import("../header/frame.zig");
const toc_mod = @import("../header/toc.zig");
const container_mod = @import("../header/container.zig");
const icc_mod = @import("../header/icc.zig");

const dec_ans = @import("../entropy/dec_ans.zig");
const modular_common = @import("../modular/common.zig");
const image_mod = @import("../modular/image.zig");
const transforms = @import("../modular/transforms.zig");
const context_predict = @import("../modular/context_predict.zig");
const ma = @import("../modular/ma.zig");
const stream = @import("../modular/stream.zig");
const patches_mod = @import("../patches/dictionary.zig");
const lossy_mod = @import("../vardct/lossy.zig");
const opsin_mod = @import("../vardct/opsin.zig");
const quant_mod = @import("../vardct/quant.zig");

const pixel_mod = @import("pixel.zig");

const Allocator = std.mem.Allocator;
const BitReader = bit_io.BitReader;
const PixelBuffer = pixel_mod.PixelBuffer;
const FloatPixelBuffer = pixel_mod.FloatPixelBuffer;
const Image = image_mod.Image;
const Channel = image_mod.Channel;

/// The decode result: integer samples (`PixelBuffer`) or float samples
/// (`FloatPixelBuffer`, PFM-style: no alpha channel carried).
pub const DecodedImage = union(enum) {
    int: PixelBuffer,
    float: FloatPixelBuffer,

    pub fn deinit(self: *DecodedImage) void {
        switch (self.*) {
            .int => |*p| p.deinit(),
            .float => |*f| f.deinit(),
        }
    }
};

pub const DecodeError = error{
    UnsupportedContainer,
    UnsupportedVarDct,
    UnsupportedFrameFeatures,
    UnsupportedColorTransform,
    UnsupportedCustomTransform,
    UnsupportedTocPermutation,
    NotLossless,
    BadPixelRange,
    NotEnoughBytes,
    UnsupportedFloatSample, // float extra channels (alpha) are out of scope
} || Allocator.Error || anyerror;

pub const DecodeOptions = struct {
    /// When set and the file carries an ICC profile, receives an
    /// allocator-owned copy of the raw ICC bytes (caller frees).
    icc_out: ?*?[]u8 = null,
    /// Lossy (VarDCT) frames decode to linear-sRGB float samples instead
    /// of the sRGB-encoded integer output (djxl PFM-style).
    prefer_linear_float: bool = false,
};

/// Decodes a complete codestream (raw or inside a container) into pixels.
/// `input` must contain the whole file. Integer-sample files decode into
/// `.int` (with NLEE upsampling applied when the frame header says so),
/// float-sample files (fp32/fp16 and other custom float formats) into
/// `.float` raw f32 values.
pub fn decode(allocator: Allocator, input: []const u8, options: DecodeOptions) DecodeError!DecodedImage {
    var codestream_owned: ?[]u8 = null;
    defer if (codestream_owned) |cs| allocator.free(cs);

    var codestream: []const u8 = input;
    if (container_mod.isContainer(input)) {
        codestream_owned = try container_mod.extractCodestream(allocator, input);
        codestream = codestream_owned.?;
    } else if (!container_mod.isRawCodestream(input)) {
        return error.UnsupportedContainer;
    }

    var br = BitReader.init(codestream);
    if (br.readFixedBits(8) != 0xFF or br.readFixedBits(8) != 0x0A) {
        return error.UnsupportedContainer;
    }

    var sh: size_mod.SizeHeader = .{};
    try sh.read(&br);
    var metadata = metadata_mod.ImageMetadata.init(allocator);
    defer metadata.deinit();
    try metadata.read(&br);
    var ctd = metadata_mod.CustomTransformData{};
    try ctd.read(&br, metadata.xyb_encoded);
    if (metadata.color_encoding.want_icc) {
        // Compressed ICC profile (libjxl ICCReader::Init/Process).
        const icc_bytes = icc_mod.readICC(allocator, &br) catch |e| switch (e) {
            error.OutOfMemory => return error.OutOfMemory,
            else => return error.InvalidICCProfile,
        };
        defer allocator.free(icc_bytes);
        if (options.icc_out) |out| {
            const copy = try allocator.dupe(u8, icc_bytes);
            out.* = copy;
        }
    }
    try br.jumpToByteBoundary();

    // Reference-frame store for patches (planes owned here). Frames saved
    // before the color transform fill their slot; the main frame's patch
    // dictionary (kPatches) references them (libjxl: patches may only use
    // pre-transform references).
    var refs: patches_mod.RefStore = .{ null, null, null, null };
    defer freeRefStore(allocator, &refs);

    while (true) {
        var frame = frame_mod.FrameHeader.init(allocator);
        defer frame.deinit();
        try frame.read(&metadata, &br);

        if (frame.encoding == .var_dct) {
            // Lossy VarDCT still (the XYB patch subset; see
            // vardct/lossy.zig). The frame must be the last one; patch
            // reference frames ahead of it were decoded above.
            if (frame.frame_type != .regular) return error.UnsupportedFrameFeatures;
            if (!frame.is_last) return error.UnsupportedFrameFeatures;
            if (!metadata.xyb_encoded) return error.UnsupportedVarDct;
            if (frame.custom_size_or_origin) return error.UnsupportedFrameFeatures;

            const fd = stream.FrameDim.initVarDct(sh.xsizeOf(), sh.ysizeOf(), 0, 0);
            const num_toc = toc_mod.numTocEntries(fd.num_groups, fd.num_dc_groups, 1);
            var toc = try toc_mod.readTocWithPermutation(allocator, num_toc, &br);
            defer toc.deinit(allocator);
            var toc_sizes: []u32 = undefined;
            var section_starts: []usize = undefined;
            try toc.logicalLayout(allocator, &toc_sizes, &section_starts);
            defer allocator.free(toc_sizes);
            defer allocator.free(section_starts);
            const total_len = toc.totalSize();
            const base = br.totalBitsConsumed() / 8;
            if (base + total_len > codestream.len) return error.NotEnoughBytes;

            var img = try lossy_mod.decodeLossyFrame(
                allocator,
                &metadata,
                &frame,
                sh.xsizeOf(),
                sh.ysizeOf(),
                codestream[base .. base + total_len],
                section_starts,
                toc_sizes,
                &refs,
            );
            defer img.deinit();
            const out = try lossyToDecodedImage(allocator, &metadata, &img, options.prefer_linear_float);
            return out;
        }

        if (frame.encoding != .modular) return error.UnsupportedVarDct;
        // Modular frames: color transform none, except the XYB reference
        // frames of a lossy (xyb_encoded) codestream (the patch reference
        // frame of the VarDCT path).
        if (frame.color_transform != .none) {
            if (!(metadata.xyb_encoded and frame.color_transform == .xyb and
                frame.frame_type == .reference_only))
            {
                return error.UnsupportedColorTransform;
            }
        }
        // (kNoise/kSplines/kUseDcFrame flag bits are rejected by read();
        //  frame_type dc_frame carries the flag bit, so only the type
        //  check remains here.)
        if (frame.frame_type == .dc_frame) return error.UnsupportedFrameFeatures;

        if (frame.frame_type == .reference_only) {
            // Patch (or other) reference frame: decode, store planes into
            // its reference slot, continue with the next frame. Patches
            // inside reference frames (recursive dictionaries) are a valid
            // libjxl construct but never produced by cjxl or this encoder;
            // rejected.
            if (frame.flags & frame_mod.Flags.kPatches != 0) {
                return error.UnsupportedFrameFeatures;
            }
            if (frame.upsampling != 1) return error.UnsupportedFrameFeatures;
            for (frame.extra_channel_upsampling.items) |ecups| {
                if (ecups != 1) return error.UnsupportedFrameFeatures;
            }
            var fxsize: u32 = sh.xsizeOf();
            var fysize: u32 = sh.ysizeOf();
            if (frame.custom_size_or_origin) {
                fxsize = frame.frame_size_x;
                fysize = frame.frame_size_y;
            }
            if (fxsize == 0 or fysize == 0) return error.UnsupportedFrameFeatures;

            const fd = stream.FrameDim.init(fxsize, fysize, frame.group_size_shift, 1);
            const num_toc = toc_mod.numTocEntries(fd.num_groups, fd.num_dc_groups, frame.passes.num_passes);
            var toc = try toc_mod.readTocWithPermutation(allocator, num_toc, &br);
            defer toc.deinit(allocator);
            var toc_sizes: []u32 = undefined;
            var section_starts: []usize = undefined;
            try toc.logicalLayout(allocator, &toc_sizes, &section_starts);
            defer allocator.free(toc_sizes);
            defer allocator.free(section_starts);
            const total_len = toc.totalSize();
            const base = br.totalBitsConsumed() / 8;
            if (base + total_len > codestream.len) return error.NotEnoughBytes;
            _ = br.skipBits(total_len * 8);

            var dec = FrameDecoder{
                .allocator = allocator,
                .metadata = &metadata,
                .frame = &frame,
                .fd = fd,
                .ctd = &ctd,
                .codestream = codestream[base .. base + total_len],
                .section_starts = section_starts,
                .section_sizes = toc_sizes,
                .xsize = fxsize,
                .ysize = fysize,
            };
            defer dec.deinitState();
            try dec.runChannels();

            // int planes -> float [0,1] planes in the reference slot
            // (libjxl stores the decoded ImageBundle; SingleFromSingle's
            // per-channel factor — or, for the XYB reference frames of a
            // lossy codestream, the YX(B-Y) reconstruction of
            // ModularImageToDecodedRect with this frame's DequantDC).
            try storePatchReference(allocator, &metadata, &refs, &frame, &dec.full_image, &dec.dc_quants);
            continue;
        }

        if (frame.frame_type != .regular and frame.frame_type != .skip_progressive) {
            return error.UnsupportedFrameFeatures;
        }
        if (frame.upsampling != 1 and frame.upsampling != 2 and
            frame.upsampling != 4 and frame.upsampling != 8)
        {
            return error.UnsupportedFrameFeatures;
        }
        if (!frame.is_last) return error.UnsupportedFrameFeatures;
        if (frame.custom_size_or_origin) return error.UnsupportedFrameFeatures;
        // Multi-pass (progressive, cjxl -p) modular frames are supported: the
        // AC group sections are laid out per pass with shift brackets.

        const fd = stream.FrameDim.init(sh.xsizeOf(), sh.ysizeOf(), frame.group_size_shift, frame.upsampling);
        const num_toc = toc_mod.numTocEntries(fd.num_groups, fd.num_dc_groups, frame.passes.num_passes);
        var toc = try toc_mod.readTocWithPermutation(allocator, num_toc, &br);
        defer toc.deinit(allocator);
        var toc_sizes: []u32 = undefined;
        var section_starts: []usize = undefined;
        try toc.logicalLayout(allocator, &toc_sizes, &section_starts);
        defer allocator.free(toc_sizes);
        defer allocator.free(section_starts);

        const total_len = toc.totalSize();
        const base = br.totalBitsConsumed() / 8;
        if (base + total_len > codestream.len) return error.NotEnoughBytes;

        var pd = patches_mod.PatchDictionary.init(allocator);
        defer pd.deinit();
        const wants_patches = frame.flags & frame_mod.Flags.kPatches != 0;

        var dec = FrameDecoder{
            .allocator = allocator,
            .metadata = &metadata,
            .frame = &frame,
            .fd = fd,
            .ctd = &ctd,
            .codestream = codestream[base .. base + total_len],
            .section_starts = section_starts,
            .section_sizes = toc_sizes,
            .xsize = sh.xsizeOf(),
            .ysize = sh.ysizeOf(),
            .patch_refs = if (wants_patches) &refs else null,
            .patches = if (wants_patches) &pd else null,
        };
        defer dec.deinitState();
        try dec.run();
        const out = dec.output.?;
        dec.output = null; // ownership moves to the caller
        return out;
    }
}

/// Frees the owned float planes of a patch reference store.
pub fn freeRefStore(allocator: Allocator, refs: *patches_mod.RefStore) void {
    patches_mod.freeRefStore(allocator, refs);
}

/// Converts a decoded reference frame (channel planes — integer samples
/// or float bit patterns) to float planes and stores it into its
/// reference slot. Color channels use the metadata bit depth, extra
/// channels their own (libjxl SingleFromSingle per channel for integers,
/// int_to_float for float samples). XYB reference frames (the VarDCT
/// patch path) reconstruct through the frame's DequantDC instead:
/// X = ch1*dcq, Y = ch0*dcq, B = (ch2+ch0)*dcq.
fn storePatchReference(
    allocator: Allocator,
    metadata: *const metadata_mod.ImageMetadata,
    refs: *patches_mod.RefStore,
    frame: *const frame_mod.FrameHeader,
    image: *const Image,
    dc_quants: *const [3]f32,
) !void {
    const slot: usize = @intCast(frame.save_as_reference & 3);
    if (slot == 0) return; // not saved anywhere
    const n_planes = image.channels.items.len;
    if (n_planes == 0) return error.UnsupportedFrameFeatures;
    const w = image.channels.items[0].w;
    const h = image.channels.items[0].h;

    // Free the previous occupant of the slot.
    if (refs[slot]) |*old| {
        for (old.planes) |p| allocator.free(p);
        allocator.free(old.planes);
        refs[slot] = null;
    }

    const n_color: usize = if (metadata.color_encoding.isGray()) 1 else 3;
    const is_xyb = frame.color_transform == .xyb;
    const planes = try allocator.alloc([]f32, n_planes);
    var initialized: usize = 0;
    errdefer {
        for (planes[0..initialized]) |p| allocator.free(p);
        allocator.free(planes);
    }
    for (0..n_planes) |c| {
        const p = try allocator.alloc(f32, w * h);
        initialized += 1;
        const ch = image.channels.items[c];
        if (is_xyb and c < 3) {
            // ModularImageToDecodedRect: XYB modular frames store
            // [Y, X, B-Y]; c_in = 1 - c for c < 2.
            const c_in: usize = if (c < 2) 1 - c else 2;
            const ch_in = image.channels.items[c_in];
            const ch_y = image.channels.items[0];
            for (0..h) |y| {
                for (0..w) |x| {
                    if (c == 2) {
                        p[y * w + x] = @mulAdd(f32, @floatFromInt(ch_in.row(y)[x] + ch_y.row(y)[x]), dc_quants[2], 0);
                    } else {
                        p[y * w + x] = @as(f32, @floatFromInt(ch_in.row(y)[x])) * dc_quants[c];
                    }
                }
            }
        } else {
            // Per-channel sample format (libjxl stores the decoded
            // ImageBundle; SingleFromSingle's per-channel factor for
            // integers, int_to_float of the bit patterns for float
            // samples — fp16 patch frames).
            const bd = if (c < n_color)
                metadata.bit_depth
            else
                metadata.extra_channel_info.items[c - n_color].bit_depth;
            const fmt = patches_mod.SampleFormat{
                .bits = bd.bits_per_sample,
                .exp_bits = bd.exponent_bits_per_sample,
                .floating = bd.floating_point_sample,
            };
            for (0..h) |y| {
                const row = ch.row(y);
                for (0..w) |x| {
                    p[y * w + x] = patches_mod.sampleToFloat(row[x], fmt);
                }
            }
        }
        planes[c] = p;
    }
    refs[slot] = .{
        .planes = planes,
        .xsize = w,
        .ysize = h,
        .origin_x = 0,
        .origin_y = 0,
        .before_color_transform = frame.save_before_color_transform,
    };
}

/// Converts the linear-sRGB float planes of a lossy frame into the decode
/// result: integer sRGB samples (the codestream bit depth, the exact
/// inverse sRGB EOTF) or raw float values for `prefer_linear_float`.
fn lossyToDecodedImage(allocator: Allocator, metadata: *const metadata_mod.ImageMetadata, img: *const lossy_mod.LossyImage, prefer_linear: bool) !DecodedImage {
    const bits: u8 = @intCast(metadata.bit_depth.bits_per_sample);
    if (prefer_linear or metadata.bit_depth.floating_point_sample or bits > 16) {
        // Float-sample metadata: keep the linear floats as-is.
        var fb = try FloatPixelBuffer.create(
            allocator,
            img.xsize,
            img.ysize,
            .rgb,
            @intCast(metadata.bit_depth.bits_per_sample),
            @intCast(metadata.bit_depth.exponent_bits_per_sample),
        );
        for (0..3) |c| {
            for (0..img.ysize) |y| {
                for (0..img.xsize) |x| {
                    fb.set(x, y, c, img.planes[c][y * img.xsize + x]);
                }
            }
        }
        return .{ .float = fb };
    }
    var pb = try PixelBuffer.create(allocator, img.xsize, img.ysize, .rgb, false, bits);
    const maxval: f32 = @floatFromInt((@as(u64, 1) << @intCast(bits)) - 1);
    for (0..img.ysize) |y| {
        for (0..img.xsize) |x| {
            for (0..3) |c| {
                const lin = img.planes[c][y * img.xsize + x];
                const enc = opsin_mod.linearToSrgb(std.math.clamp(lin, 0.0, 1.0));
                const vmax: i64 = @intFromFloat(maxval);
                const v = std.math.clamp(@as(i64, @intFromFloat(@round(enc * maxval))), 0, vmax);
                pb.set(x, y, c, @intCast(v));
            }
        }
    }
    return .{ .int = pb };
}

/// Decodes the sections of one modular frame into its channel planes
/// (color channels first, then extra channels; all global transforms
/// undone). `xsize`/`ysize` are the frame's own pixel dimensions (the
/// crop size for frames with `custom_size_or_origin`, the image size for
/// full frames); `codestream` is the frame's section bytes, with
/// `section_starts[i]`/`section_sizes[i]` giving logical section i's byte
/// range (from `Toc.logicalLayout`, permutation applied). Used by the
/// animation decoder; ownership of the returned `Image` moves to the
/// caller.
///
/// Patch support: when `patches` is non-null and the frame header carries
/// `kPatches`, the patch dictionary is read from the head of the DC global
/// section (spec 18181-2: patches precede the dequant matrices there) and
/// applied to the decoded planes after all transforms are undone;
/// `patch_refs` must then be the live reference-frame store. Both are
/// borrowed, never freed here.
pub fn decodeFrameImage(
    allocator: Allocator,
    metadata: *const metadata_mod.ImageMetadata,
    frame: *const frame_mod.FrameHeader,
    xsize: u32,
    ysize: u32,
    upsampling: usize,
    codestream: []const u8,
    section_starts: []const usize,
    section_sizes: []const u32,
    patch_refs: ?*const patches_mod.RefStore,
    patches: ?*patches_mod.PatchDictionary,
) DecodeError!Image {
    const fd = stream.FrameDim.init(xsize, ysize, frame.group_size_shift, upsampling);
    var dec = FrameDecoder{
        .allocator = allocator,
        .metadata = metadata,
        .frame = frame,
        .fd = fd,
        .codestream = codestream,
        .section_starts = section_starts,
        .section_sizes = section_sizes,
        .xsize = xsize,
        .ysize = ysize,
        .patch_refs = patch_refs,
        .patches = patches,
    };
    defer dec.deinitState();
    try dec.runChannels();
    try dec.applyPatches();
    dec.full_image_initialized = false; // ownership moves out
    return dec.full_image;
}

const bounds_relax_pixels = true;

const FrameDecoder = struct {
    allocator: Allocator,
    metadata: *const metadata_mod.ImageMetadata,
    frame: *const frame_mod.FrameHeader,
    fd: stream.FrameDim,
    /// CustomTransformData (upsampling weight sets); borrowed.
    ctd: *const metadata_mod.CustomTransformData = &default_ctd,
    codestream: []const u8,
    /// Byte range of logical section i (from Toc.logicalLayout: with a
    /// TOC permutation, section i's bytes live at
    /// starts[i]..starts[i]+sizes[i] regardless of neighboring sections).
    section_starts: []const usize,
    section_sizes: []const u32,
    xsize: u32,
    ysize: u32,
    /// Reference-frame store for patch application (borrowed; required
    /// when the frame header carries kPatches).
    patch_refs: ?*const patches_mod.RefStore = null,
    /// Patch dictionary to decode into (borrowed; required when the frame
    /// header carries kPatches).
    patches: ?*patches_mod.PatchDictionary = null,

    global: stream.EntropyCode = undefined,
    global_initialized: bool = false,
    full_image: Image = undefined,
    full_image_initialized: bool = false,
    global_transforms: std.ArrayList(transforms.Transform) = .empty,
    global_wp_header: context_predict.weighted.Header = context_predict.weighted.Header.defaults,
    output: ?DecodedImage = null,
    /// DequantMatrices DC values of this frame (custom values parsed
    /// from the DC global; the kDCQuant defaults otherwise). XYB
    /// reference frames reconstruct their planes through them.
    dc_quants: [3]f32 = .{ 1.0 / 4096.0, 1.0 / 512.0, 1.0 / 256.0 },

    const default_ctd = metadata_mod.CustomTransformData{};

    fn numTotalChannels(self: *const FrameDecoder) usize {
        const n_color: usize = if (self.metadata.xyb_encoded) 3 else if (self.metadata.color_encoding.isGray()) 1 else 3;
        return n_color + self.metadata.extra_channel_info.items.len;
    }

    fn deinitState(self: *FrameDecoder) void {
        for (self.global_transforms.items) |*t| t.deinit();
        self.global_transforms.deinit(self.allocator);
        if (self.global_initialized) {
            self.global.deinit();
            self.global_initialized = false;
        }
        if (self.full_image_initialized) {
            self.full_image.deinit();
            self.full_image_initialized = false;
        }
        if (self.output) |*o| o.deinit();
        self.output = null;
    }

    fn run(self: *FrameDecoder) !void {
        try self.runChannels();
        try self.applyPatches();
        try self.assembleOutput();
    }

    /// Applies the decoded patch dictionary (if any) to the channel planes.
    /// Runs after all global transforms are undone and before the output
    /// assembly (upsampling / float conversion), i.e. on the same
    /// frame-sized grid the patch positions address. Float-sample frames
    /// (fp16 patch files) blend through int_to_float/float_to_int of the
    /// bit patterns (libjxl `bits_per_sample <= 16` gate includes
    /// binary16).
    fn applyPatches(self: *FrameDecoder) !void {
        const pd = self.patches orelse return;
        if (!pd.hasAny()) return;
        const m = self.metadata;
        const n_color: usize = if (m.color_encoding.isGray()) 1 else 3;
        const n_ec = m.extra_channel_info.items.len;
        const formats = try self.allocator.alloc(patches_mod.SampleFormat, n_color + n_ec);
        defer self.allocator.free(formats);
        for (0..n_color) |i| formats[i] = .{
            .bits = m.bit_depth.bits_per_sample,
            .exp_bits = m.bit_depth.exponent_bits_per_sample,
            .floating = m.bit_depth.floating_point_sample,
        };
        for (m.extra_channel_info.items, 0..) |*eci, i| {
            formats[n_color + i] = .{
                .bits = eci.bit_depth.bits_per_sample,
                .exp_bits = eci.bit_depth.exponent_bits_per_sample,
                .floating = eci.bit_depth.floating_point_sample,
            };
        }
        try pd.applyToImage(self.allocator, .{
            .image = &self.full_image,
            .n_color = n_color,
            .formats = formats,
            .ec_info = m.extra_channel_info.items,
            .refs = self.patch_refs orelse return error.InvalidPatchReferenceFrame,
        });
    }

    /// Decodes all sections and undoes the global transforms, leaving the
    /// channel planes in `self.full_image`.
    fn runChannels(self: *FrameDecoder) !void {
        self.global = stream.EntropyCode.init(self.allocator);
        self.global_initialized = true;
        self.full_image = Image.create(self.allocator);
        self.full_image_initialized = true;

        // Section 0: DC global (and, for single-section frames, everything).
        var sec0 = self.sectionReader(0);
        try self.decodeDCGlobal(&sec0);

        if (self.fd.num_groups == 1 and self.frame.passes.num_passes == 1) {
            // Single-section frame: DC group 0 and AC global carry nothing
            // for single-group modular frames; AC group 0 has no channels
            // either (all channels decoded in the global stream) and
            // early-returns.
            try self.decodeACGroup(&sec0, 0, 0, 0, 2);
            try sec0.close();
        } else {
            try sec0.close();
            for (0..self.fd.num_dc_groups) |g| {
                const sec = 1 + g;
                var sec_br = self.sectionReader(sec);
                try self.decodeDCGroup(&sec_br, g);
                try sec_br.close();
            }
            const ac_global_sec = 1 + self.fd.num_dc_groups;
            if (self.section_sizes[ac_global_sec] != 0) {
                return error.UnsupportedFrameFeatures;
            }
            // AC group sections, one per (pass, group): each pass decodes
            // the channels whose min-shift falls inside its bracket
            // (libjxl dec_frame ProcessACGroup / GetDownsamplingBracket).
            for (0..self.frame.passes.num_passes) |pass| {
                var min_shift: i32 = 0;
                var max_shift: i32 = 0;
                self.frame.passes.getDownsamplingBracket(pass, &min_shift, &max_shift);
                for (0..self.fd.num_groups) |g| {
                    const sec = toc_mod.acGroupIndex(pass, g, self.fd.num_groups, self.fd.num_dc_groups);
                    var sec_br = self.sectionReader(sec);
                    try self.decodeACGroup(&sec_br, g, pass, min_shift, max_shift);
                    try sec_br.close();
                }
            }
        }

        // Undo global transforms (reverse order).
        var ti: usize = self.global_transforms.items.len;
        while (ti > 0) {
            ti -= 1;
            try transforms.inverse(&self.full_image, &self.global_transforms.items[ti], &self.global_wp_header, self.metadata.bit_depth.bits_per_sample);
        }
    }

    fn decodeDCGlobal(self: *FrameDecoder, br: *BitReader) !void {
        // Patch dictionary (spec 18181-2 DC global layout, libjxl
        // FrameDecoder::ProcessDCGlobal): patches are serialized first,
        // before the dequant matrices DC.
        if (self.frame.flags & frame_mod.Flags.kPatches != 0) {
            const pd = self.patches orelse return error.UnsupportedFrameFeatures;
            var uses_extra_channels: bool = false;
            try pd.decode(
                br,
                self.fd.xsize,
                self.fd.ysize,
                self.metadata.extra_channel_info.items.len,
                self.patch_refs orelse return error.InvalidPatchReferenceFrame,
                &uses_extra_channels,
            );
            if (self.frame.upsampling != 1) {
                // "Cannot use extra channels in patches if color channels
                // are subsampled differently from extra channels".
                for (self.frame.extra_channel_upsampling.items) |ecups| {
                    if (ecups != self.frame.upsampling) {
                        return error.UnsupportedFrameFeatures;
                    }
                }
            }
        }

        // DequantMatrices DC: all_default bit or custom F16 values.
        {
            var ddc = quant_mod.DequantDC{};
            try ddc.read(br);
            self.dc_quants = ddc.dc_quant;
        }

        // ModularFrameDecoder::DecodeGlobalInfo: has_tree bit + optional
        // global tree + histograms, then the global modular stream.
        const has_tree = (br.readBits(1)) != 0;
        if (has_tree) {
            const tree_size_limit = @min(@as(usize, 1) << 22, 1024 + self.xsize * self.ysize * self.numTotalChannels() / 16);
            var tree_code: stream.EntropyCode = stream.EntropyCode.init(self.allocator);
            defer tree_code.deinit();
            var tree_ctx_map: std.ArrayList(u8) = .empty;
            defer tree_ctx_map.deinit(self.allocator);
            try dec_ans.decodeHistograms(self.allocator, br, ma.kNumTreeContexts, &tree_code.code, &tree_ctx_map, false);
            var tree_reader = try stream.AnsMaReader.create(self.allocator, &tree_code.code, tree_ctx_map.items, 0, br);
            defer tree_reader.ans.deinit();
            try ma.decodeTree(br, &tree_reader, &self.global.tree, self.allocator, tree_size_limit);
            try dec_ans.decodeHistograms(self.allocator, br, self.global.numContexts(), &self.global.code, &self.global.context_map, false);
        }

        // Global stream: full-frame planes, decode limited to group size.
        // Channel geometry (libjxl DecodeGlobalInfo): color channels at
        // frame_dim size (the frame size divided by `upsampling`), extra
        // channels at DivCeil(upsampled size, ec_upsampling) with their
        // relative shift vs. the color channels. XYB codestreams always
        // carry three color channels, even for gray color encodings.
        const n_color: usize = if (self.metadata.xyb_encoded) 3 else if (self.metadata.color_encoding.isGray()) 1 else 3;
        for (0..n_color) |_| {
            _ = try self.full_image.addChannel(self.fd.xsize, self.fd.ysize);
        }
        const color_shift: i32 = @intCast(ceilLog2Nonzero(self.frame.upsampling));
        for (self.metadata.extra_channel_info.items, 0..) |_, i| {
            const ecups: usize = self.frame.extra_channel_upsampling.items[i];
            const w = std.math.divCeil(usize, self.fd.xsize_upsampled, ecups) catch 1;
            const h = std.math.divCeil(usize, self.fd.ysize_upsampled, ecups) catch 1;
            const shift: i32 = @as(i32, @intCast(ceilLog2Nonzero(@intCast(ecups)))) - color_shift;
            const ch = try Channel.create(self.allocator, w, h, shift, shift);
            try self.full_image.channels.append(self.allocator, ch);
        }
        try stream.decodeStream(.{
            .allocator = self.allocator,
            .br = br,
            .image = &self.full_image,
            .group_id = 0,
            .max_chan_size = self.fd.group_dim,
            .global = if (has_tree) &self.global else null,
            .undo_transforms = false,
            .transforms_out = &self.global_transforms,
            .wp_header_out = &self.global_wp_header,
            .bitdepth = self.metadata.bit_depth.bits_per_sample,
        });
    }

    /// First non-meta channel bigger than the group size; channels from
    /// here on are decoded in the group streams (libjxl DecodeGroup /
    /// AddModularGroupImage).
    fn firstBigChannel(self: *const FrameDecoder) usize {
        var c: usize = self.full_image.nb_meta_channels;
        while (c < self.full_image.channels.items.len) : (c += 1) {
            const fc = &self.full_image.channels.items[c];
            if (fc.w > self.fd.group_dim or fc.h > self.fd.group_dim) break;
        }
        return c;
    }

    /// Builds the per-group channel list: channels from `beginc` on whose
    /// min-shift falls inside `[min_shift, max_shift]`, with the group rect
    /// scaled by the per-channel shifts (libjxl DecodeGroup channel
    /// selection).
    fn groupChannelList(
        self: *FrameDecoder,
        rect: stream.Rect,
        min_shift: i32,
        max_shift: i32,
        gi: *Image,
        source_indices: *std.ArrayList(usize),
    ) !void {
        const beginc = self.firstBigChannel();
        for (beginc..self.full_image.channels.items.len) |ci| {
            const fc = &self.full_image.channels.items[ci];
            const shift = @min(fc.hshift, fc.vshift);
            if (shift > max_shift) continue;
            if (shift < min_shift) continue;
            const hshift: usize = if (fc.hshift > 0) @intCast(fc.hshift) else 0;
            const vshift: usize = if (fc.vshift > 0) @intCast(fc.vshift) else 0;
            const r = (stream.Rect{
                .x0 = rect.x0 >> @intCast(hshift),
                .y0 = rect.y0 >> @intCast(vshift),
                .xsize = rect.xsize >> @intCast(hshift),
                .ysize = rect.ysize >> @intCast(vshift),
            }).intersect(fc.w, fc.h);
            if (r.xsize == 0 or r.ysize == 0) continue;
            const gc = try Channel.create(self.allocator, r.xsize, r.ysize, fc.hshift, fc.vshift);
            try gi.channels.append(self.allocator, gc);
            try source_indices.append(self.allocator, ci);
        }
    }

    /// Copies a decoded group image back into the full image channels.
    fn copyGroupImageBack(
        self: *FrameDecoder,
        rect: stream.Rect,
        gi: *Image,
        source_indices: []const usize,
    ) !void {
        for (gi.channels.items, 0..) |*gc, k| {
            const fc = &self.full_image.channels.items[source_indices[k]];
            const hshift: usize = if (fc.hshift > 0) @intCast(fc.hshift) else 0;
            const vshift: usize = if (fc.vshift > 0) @intCast(fc.vshift) else 0;
            const r = (stream.Rect{
                .x0 = rect.x0 >> @intCast(hshift),
                .y0 = rect.y0 >> @intCast(vshift),
                .xsize = rect.xsize >> @intCast(hshift),
                .ysize = rect.ysize >> @intCast(vshift),
            }).intersect(fc.w, fc.h);
            for (0..r.ysize) |y| {
                @memcpy(fc.row(r.y0 + y)[r.x0 .. r.x0 + r.xsize], gc.row(y));
            }
        }
    }

    /// Modular DC group stream: channels with min-shift >= 3 over the
    /// (2048-sized) DC group rect. Carries the high-shift half of a
    /// Squeeze transform's channels for large images.
    fn decodeDCGroup(self: *FrameDecoder, br: *BitReader, group_index: usize) !void {
        const rect = self.fd.modularDcGroupRect(group_index);
        var gi = Image.create(self.allocator);
        defer gi.deinit();
        var source_indices: std.ArrayList(usize) = .empty;
        defer source_indices.deinit(self.allocator);
        try self.groupChannelList(rect, 3, 1000, &gi, &source_indices);
        if (gi.channels.items.len == 0) return; // nothing to decode

        const sid = stream.ModularStreamId.modularDC(group_index).id(&self.fd);
        try stream.decodeStream(.{
            .allocator = self.allocator,
            .br = br,
            .image = &gi,
            .group_id = sid,
            .max_chan_size = self.fd.group_dim,
            .global = &self.global,
            .undo_transforms = true,
            .bitdepth = self.metadata.bit_depth.bits_per_sample,
        });
        try self.copyGroupImageBack(rect, &gi, source_indices.items);
    }

    fn decodeACGroup(
        self: *FrameDecoder,
        br: *BitReader,
        group_index: usize,
        pass: usize,
        min_shift: i32,
        max_shift: i32,
    ) !void {
        const rect = self.fd.modularGroupRect(group_index);
        var gi = Image.create(self.allocator);
        defer gi.deinit();

        // Channels from the first bigger-than-group channel onward, with
        // min-shift inside this pass's bracket.
        var source_indices: std.ArrayList(usize) = .empty;
        defer source_indices.deinit(self.allocator);
        try self.groupChannelList(rect, min_shift, max_shift, &gi, &source_indices);
        if (gi.channels.items.len == 0) return; // nothing to decode

        const sid = stream.ModularStreamId.modularAC(group_index, pass).id(&self.fd);
        try stream.decodeStream(.{
            .allocator = self.allocator,
            .br = br,
            .image = &gi,
            .group_id = sid,
            .max_chan_size = self.fd.group_dim,
            .global = &self.global,
            .undo_transforms = true,
            .bitdepth = self.metadata.bit_depth.bits_per_sample,
        });
        try self.copyGroupImageBack(rect, &gi, source_indices.items);
    }

    /// Output assembly. Fast paths for the transform-less cases (direct
    /// int copy / direct int_to_float), the NLEE pipeline when the frame
    /// (or an extra channel) is upsampled (libjxl: dec_modular's
    /// int->float conversion, the render-pipeline UpsamplingStage, and
    /// dec_external_image's float->int output conversion).
    fn assembleOutput(self: *FrameDecoder) !void {
        const m = self.metadata;
        const n_color: usize = if (m.color_encoding.isGray()) 1 else 3;
        const fp = m.bit_depth.floating_point_sample;
        // Float extra channels would need a float alpha surface; out of
        // scope (PFM-style output has no alpha).
        for (m.extra_channel_info.items) |*eci| {
            if (eci.bit_depth.floating_point_sample) return error.UnsupportedFloatSample;
        }

        var any_upsampling = self.frame.upsampling != 1;
        for (self.frame.extra_channel_upsampling.items) |ecups| {
            if (ecups != 1) any_upsampling = true;
        }
        if (!any_upsampling) {
            if (fp) {
                try self.assembleFloatFast(n_color);
            } else {
                try self.assembleIntFast(n_color);
            }
            return;
        }
        try self.assembleUpsampled(n_color, fp);
    }

    /// No upsampling, integer samples: direct channel copy (the
    /// previously-validated path).
    fn assembleIntFast(self: *FrameDecoder, n_color: usize) !void {
        _ = n_color;
        const has_alpha = self.metadata.hasAlpha();
        const bitdepth = self.metadata.bit_depth.bits_per_sample;
        if (bitdepth == 0 or bitdepth > 16) return error.UnsupportedBitDepth;
        const lay: pixel_mod.Layout = if (self.metadata.color_encoding.isGray()) .gray else .rgb;
        var pb = try PixelBuffer.create(self.allocator, self.xsize, self.ysize, lay, has_alpha, @intCast(bitdepth));
        errdefer pb.deinit();
        const nch = pb.numChannels();
        const max_value = pb.maxValue();
        for (0..self.ysize) |y| {
            for (0..self.xsize) |x| {
                for (0..nch) |c| {
                    const v = self.full_image.channels.items[c].row(y)[x];
                    if (v < 0 or v > max_value) {
                        if (bounds_relax_pixels) {
                            pb.set(x, y, c, @intCast(std.math.clamp(v, 0, @as(i64, max_value))));
                            continue;
                        }
                        return error.BadPixelRange;
                    }
                    pb.set(x, y, c, @intCast(v));
                }
            }
        }
        self.output = .{ .int = pb };
    }

    /// No upsampling, float samples: int_to_float per sample (dec_modular).
    fn assembleFloatFast(self: *FrameDecoder, n_color: usize) !void {
        const bits: u6 = @intCast(self.metadata.bit_depth.bits_per_sample);
        const exp_bits: u6 = @intCast(self.metadata.bit_depth.exponent_bits_per_sample);
        const lay: pixel_mod.Layout = if (n_color == 1) .gray else .rgb;
        var fb = try FloatPixelBuffer.create(
            self.allocator,
            self.xsize,
            self.ysize,
            lay,
            @intCast(bits),
            @intCast(exp_bits),
        );
        errdefer fb.deinit();
        const nch = n_color;
        for (0..self.ysize) |y| {
            for (0..self.xsize) |x| {
                for (0..nch) |c| {
                    const v = self.full_image.channels.items[c].row(y)[x];
                    fb.set(x, y, c, float_convert.intToFloat(v, bits, exp_bits));
                }
            }
        }
        self.output = .{ .float = fb };
    }

    /// The upsampled pipeline: per channel int -> f32 (scaled for integer
    /// samples, int_to_float for float samples), NLEE upsampling with the
    /// CustomTransformData weights, then the output conversion
    /// (NearestInt(clamp01(v) * maxval) for integer samples, raw values
    /// for float samples).
    fn assembleUpsampled(self: *FrameDecoder, n_color: usize, fp: bool) !void {
        const m = self.metadata;
        const bits: u6 = @intCast(m.bit_depth.bits_per_sample);
        const exp_bits: u6 = @intCast(m.bit_depth.exponent_bits_per_sample);
        const out_w = self.xsize;
        const out_h = self.ysize;

        // Integer output needs <= 16 bits.
        if (!fp and (bits == 0 or bits > 16)) return error.UnsupportedBitDepth;

        // Per-channel output factor: color channels use the frame
        // upsampling, extra channels their ec_upsampling.
        var factors: std.ArrayList(u32) = .empty;
        defer factors.deinit(self.allocator);
        const n_total = self.full_image.channels.items.len;
        for (0..n_total) |c| {
            try factors.append(self.allocator, if (c < n_color) self.frame.upsampling else self.frame.extra_channel_upsampling.items[c - n_color]);
        }

        // Upsample every channel to the output grid.
        var planes: std.ArrayList([]f32) = .empty;
        defer {
            for (planes.items) |p| self.allocator.free(p);
            planes.deinit(self.allocator);
        }
        for (self.full_image.channels.items, 0..) |*ch, c| {
            const plane = try self.channelToUpsampledF32(ch, factors.items[c], fp, bits, exp_bits, out_w, out_h);
            try planes.append(self.allocator, plane);
        }

        if (fp) {
            var fb = try FloatPixelBuffer.create(self.allocator, out_w, out_h, if (n_color == 1) .gray else .rgb, @intCast(bits), @intCast(exp_bits));
            errdefer fb.deinit();
            for (0..n_color) |c| {
                const plane = planes.items[c];
                for (0..out_h) |y| {
                    for (0..out_w) |x| {
                        fb.set(x, y, c, plane[y * out_w + x]);
                    }
                }
            }
            self.output = .{ .float = fb };
            return;
        }

        const has_alpha = m.hasAlpha();
        const lay: pixel_mod.Layout = if (n_color == 1) .gray else .rgb;
        var pb = try PixelBuffer.create(self.allocator, out_w, out_h, lay, has_alpha, @intCast(bits));
        errdefer pb.deinit();
        const nch = pb.numChannels();
        const pb_max: i64 = (@as(i64, 1) << @intCast(bits)) - 1;
        const mul: f32 = @floatFromInt((@as(u64, 1) << @intCast(bits)) - 1);
        for (0..nch) |c| {
            const plane = planes.items[c];
            for (0..out_h) |y| {
                for (0..out_w) |x| {
                    // The output stage's unsigned conversion: dithered for
                    // 8-bit output (stage_write.cc MakeUnsigned), plain
                    // NearestInt for 9..16 bits.
                    const iv = dither.sampleToUint(plane[y * out_w + x], mul, bits, x, y, c);
                    pb.set(x, y, c, @intCast(std.math.clamp(@as(i64, @intCast(iv)), 0, pb_max)));
                }
            }
        }
        self.output = .{ .int = pb };
    }

    /// One decoded channel to an f32 plane on the full output grid:
    /// int -> float, then NLEE upsampling by `factor` when > 1.
    fn channelToUpsampledF32(
        self: *FrameDecoder,
        ch: *const Channel,
        factor: u32,
        fp: bool,
        bits: u6,
        exp_bits: u6,
        out_w: usize,
        out_h: usize,
    ) ![]f32 {
        const n = ch.w * ch.h;
        var plane = try self.allocator.alloc(f32, n);
        errdefer self.allocator.free(plane);
        if (fp) {
            for (0..n) |i| {
                plane[i] = float_convert.intToFloat(ch.plane.data[i], bits, exp_bits);
            }
        } else {
            // SingleFromSingle: v * (f32)(1.0 / maxval).
            const factor_f32: f32 = @floatCast(1.0 / @as(f64, @floatFromInt((@as(u64, 1) << @intCast(bits)) - 1)));
            for (0..n) |i| {
                plane[i] = @as(f32, @floatFromInt(ch.plane.data[i])) * factor_f32;
            }
        }
        if (factor == 1) return plane;
        const shift: u6 = @intCast(std.math.log2_int(u32, factor));
        const up = nlee.Upsampler.init(shift, self.ctd.weightsFor(factor));
        const outp = try self.allocator.alloc(f32, out_w * out_h);
        errdefer self.allocator.free(outp);
        try up.upsamplePlane(self.allocator, plane, ch.w, ch.h, outp, out_w, out_h);
        self.allocator.free(plane);
        return outp;
    }

    fn sectionReader(self: *FrameDecoder, sec: usize) BitReader {
        const start = self.section_starts[sec];
        const end = start + self.section_sizes[sec];
        return BitReader.init(self.codestream[start..end]);
    }
};

/// libjxl CeilLog2Nonzero.
fn ceilLog2Nonzero(v: u32) u32 {
    std.debug.assert(v != 0);
    return std.math.log2_int(u32, v) + @as(u32, @intFromBool(@popCount(v) != 1));
}
