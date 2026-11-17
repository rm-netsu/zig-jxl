//! Animation (multi-frame) decoding, port of the frame loop of libjxl
//! `decode.cc`/`dec_frame.cc` and the blend arithmetic of `blending.cc` /
//! `alpha.cc` for the modular lossless path.
//!
//! Model (matching libjxl's render pipeline with coalescing):
//!   - a float [0,1] canvas of all channels (color + extra), zero-initialized;
//!   - every frame decodes into its own rect (frame_size at frame_origin);
//!   - inside the rect, channels are blended onto the reference frame
//!     selected by each BlendingInfo's `source` (empty slot = zeros), using
//!     the float math of PerformAlphaBlending/PerformAlphaWeightedAdd/
//!     PerformMulBlending; extra channels blend first with the pre-blend
//!     alpha, then the color channels (a kBlend color blend also rewrites
//!     the alpha output, like libjxl's PerformBlending);
//!   - the blended frame-sized result goes to the canvas (visible part) and,
//!     when the frame CanBeReferenced and is not saved before the color
//!     transform, into reference slot `save_as_reference`;
//!   - frames with duration > 0, plus the last frame, snapshot the canvas
//!     into the output animation.

const std = @import("std");

const bit_io = @import("../base/bit_io.zig");
const size_mod = @import("../header/size.zig");
const metadata_mod = @import("../header/metadata.zig");
const frame_mod = @import("../header/frame.zig");
const toc_mod = @import("../header/toc.zig");
const container_mod = @import("../header/container.zig");
const icc_mod = @import("../header/icc.zig");

const image_mod = @import("../modular/image.zig");
const stream = @import("../modular/stream.zig");
const decoder = @import("decoder.zig");
const pixel_mod = @import("pixel.zig");
const patches_mod = @import("../patches/dictionary.zig");

const Allocator = std.mem.Allocator;
const BitReader = bit_io.BitReader;
const PixelBuffer = pixel_mod.PixelBuffer;
const Image = image_mod.Image;

pub const DecodeError = error{
    UnsupportedFrameFeatures,
    UnsupportedColorTransform,
    UnsupportedLoopFilter,
    UnsupportedExtraChannels,
    UnsupportedBitDepth,
    InvalidBlendSource,
    InvalidICCProfile,
    NotEnoughBytes,
    NoFrames,
} || decoder.DecodeError || Allocator.Error || anyerror;

pub const DecodeOptions = struct {
    /// When set and the file carries an ICC profile, receives an
    /// allocator-owned copy of the raw ICC bytes (caller frees).
    icc_out: ?*?[]u8 = null,
};

pub const AnimationFrame = struct {
    /// Full-canvas snapshot (color + optional alpha).
    pixels: PixelBuffer,
    /// Frame duration in ticks (see Animation.tps_*).
    duration_ticks: u32,
    /// 0xHHMMSSFF (only meaningful when have_timecodes).
    timecode: u32,
};

pub const Animation = struct {
    frames: std.ArrayList(AnimationFrame) = .empty,
    tps_numerator: u32 = 1000,
    tps_denominator: u32 = 1,
    num_loops: u32 = 0,
    allocator: Allocator,

    pub fn deinit(self: *Animation) void {
        for (self.frames.items) |*f| f.pixels.deinit();
        self.frames.deinit(self.allocator);
    }

    /// Duration of frame `i` in milliseconds (ticks * 1000 * den / num).
    pub fn frameDurationMs(self: *const Animation, i: usize) f64 {
        if (self.tps_numerator == 0) return 0;
        const ticks: f64 = @floatFromInt(self.frames.items[i].duration_ticks);
        return ticks * 1000.0 * @as(f64, @floatFromInt(self.tps_denominator)) /
            @as(f64, @floatFromInt(self.tps_numerator));
    }
};

/// One reference-frame slot: a frame-sized float image at an origin.
const RefFrame = struct {
    planes: [][]f32,
    xsize: usize,
    ysize: usize,
    origin_x: i32,
    origin_y: i32,

    fn deinit(self: *RefFrame, allocator: Allocator) void {
        for (self.planes) |p| allocator.free(p);
        allocator.free(self.planes);
    }
};

/// Float planes: `data.len` planes of `xsize * ysize` f32.
const Planes = struct {
    data: [][]f32,
    xsize: usize,
    ysize: usize,

    fn init(allocator: Allocator, n_planes: usize, xsize: usize, ysize: usize) !Planes {
        const data = try allocator.alloc([]f32, n_planes);
        errdefer allocator.free(data);
        var initialized: usize = 0;
        errdefer for (data[0..initialized]) |p| allocator.free(p);
        for (data) |*p| {
            p.* = try allocator.alloc(f32, xsize * ysize);
            @memset(p.*, 0);
            initialized += 1;
        }
        return .{ .data = data, .xsize = xsize, .ysize = ysize };
    }

    fn deinit(self: *Planes, allocator: Allocator) void {
        for (self.data) |p| allocator.free(p);
        allocator.free(self.data);
    }

    fn dupe(self: *const Planes, allocator: Allocator) !Planes {
        var out = try Planes.init(allocator, self.data.len, self.xsize, self.ysize);
        errdefer out.deinit(allocator);
        for (self.data, 0..) |src, c| {
            @memcpy(out.data[c], src);
        }
        return out;
    }

    inline fn at(self: *const Planes, c: usize, x: usize, y: usize) f32 {
        return self.data[c][y * self.xsize + x];
    }

    inline fn set(self: *Planes, c: usize, x: usize, y: usize, v: f32) void {
        self.data[c][y * self.xsize + x] = v;
    }
};

inline fn clamp01(v: f32) f32 {
    return @min(@max(v, 0.0), 1.0);
}

pub fn nearestEvenForTest(v: f32) i32 {
    return nearestEven(v);
}

/// Round-to-nearest, ties-to-even (libjxl `NearestInt` / nearbyint with the
/// default rounding mode, used by the float->int output conversion).
fn nearestEven(v: f32) i32 {
    if (v >= 2147483520.0) return std.math.maxInt(i32);
    if (v <= -2147483520.0) return std.math.minInt(i32);
    const f = @floor(v);
    const diff = v - f;
    if (diff > 0.5) return @intFromFloat(f + 1);
    if (diff < 0.5) return @intFromFloat(f);
    // Exactly halfway: ties to even.
    const fi: i32 = @intFromFloat(f);
    return if (@mod(fi, 2) == 0) fi else fi + 1;
}

/// libjxl `PerformAlphaBlending` (single-plane form, for extra channels and
/// the alpha channel itself); `out_a` receives the new alpha.
fn alphaBlendEc(bg: f32, bg_a: f32, fg: f32, fg_a: f32, premultiplied: bool, clamp: bool, out_a: *f32) f32 {
    const fa = if (clamp) clamp01(fg_a) else fg_a;
    if (premultiplied) {
        out_a.* = 1.0 - (1.0 - fa) * (1.0 - bg_a);
        return fg + bg * (1.0 - fa);
    } else {
        const new_a = 1.0 - (1.0 - fa) * (1.0 - bg_a);
        out_a.* = new_a;
        const rnew_a = if (new_a > 0) 1.0 / new_a else 0.0;
        return (fg * fa + bg * bg_a * (1.0 - fa)) * rnew_a;
    }
}

/// Decodes a complete animation (or still image: one frame) into frames.
pub fn decodeAnimation(allocator: Allocator, input: []const u8, options: DecodeOptions) DecodeError!Animation {
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

    if (metadata.have_preview) return error.UnsupportedFrameFeatures; // preview frame first
    // Float animations are out of scope (integer-only pipeline); reject
    // explicitly instead of silently scaling bit patterns into garbage.
    if (metadata.bit_depth.floating_point_sample) return error.UnsupportedFloatSample;
    const bitdepth = metadata.bit_depth.bits_per_sample;
    if (bitdepth == 0 or bitdepth > 16) return error.UnsupportedBitDepth;
    // The pixel buffer carries color + one alpha channel; other extra
    // channel shapes (spot colors, multiple alphas, dim_shift) are rejected.
    var alpha_ec: ?usize = null;
    for (metadata.extra_channel_info.items, 0..) |eci, i| {
        if (eci.dim_shift != 0) return error.UnsupportedExtraChannels;
        switch (eci.type) {
            .alpha => {
                if (alpha_ec != null) return error.UnsupportedExtraChannels;
                alpha_ec = i;
            },
            else => return error.UnsupportedExtraChannels,
        }
    }
    const n_color: usize = if (metadata.color_encoding.isGray()) 1 else 3;
    const n_ec = metadata.extra_channel_info.items.len;
    const n_planes: usize = n_color + n_ec;

    const image_xsize: usize = sh.xsizeOf();
    const image_ysize: usize = sh.ysizeOf();

    var anim = Animation{ .allocator = allocator };
    errdefer anim.deinit();
    if (metadata.have_animation) {
        anim.tps_numerator = metadata.animation.tps_numerator;
        anim.tps_denominator = metadata.animation.tps_denominator;
        anim.num_loops = metadata.animation.num_loops;
    }

    // Canvas + reference slots.
    var canvas = try Planes.init(allocator, n_planes, image_xsize, image_ysize);
    defer canvas.deinit(allocator);
    var refs: [4]?RefFrame = .{ null, null, null, null };
    defer for (&refs) |*r| {
        if (r.*) |*rf| rf.deinit(allocator);
    };
    // Patch reference store (float [0,1] planes; the patch dictionary
    // references these slots). Parallel to `refs`, filled at the same
    // save points (pre-transform saves and reference-only frames).
    var patch_refs: patches_mod.RefStore = .{ null, null, null, null };
    defer decoder.freeRefStore(allocator, &patch_refs);

    // int -> float factor: libjxl dec_modular uses
    // (1.0 / ((1 << bits) - 1)) computed in double, narrowed to f32.
    const scale: f32 = @floatCast(1.0 / @as(f64, @floatFromInt((@as(u64, 1) << @intCast(bitdepth)) - 1)));
    const max_val: f32 = @floatFromInt((@as(u64, 1) << @intCast(bitdepth)) - 1);

    var done = false;
    var frames_decoded: usize = 0;
    while (!done) {
        var frame = frame_mod.FrameHeader.init(allocator);
        defer frame.deinit();
        try frame.read(&metadata, &br);
        frames_decoded += 1;
        if (frames_decoded > (1 << 20)) return error.NoFrames;

        try checkFrame(&frame);

        if (frame.frame_type == .reference_only) {
            // Reference-only frame (the patch dictionary's reference image,
            // cjxl `--patches`): decoded, stored into its reference slot,
            // never composited into the canvas and not visible. Recursive
            // patch dictionaries inside it are rejected (never produced by
            // cjxl or this encoder).
            if (frame.flags & frame_mod.Flags.kPatches != 0) {
                return error.UnsupportedFrameFeatures;
            }
            var rxsize: u32 = @intCast(image_xsize);
            var rysize: u32 = @intCast(image_ysize);
            if (frame.custom_size_or_origin) {
                rxsize = frame.frame_size_x;
                rysize = frame.frame_size_y;
            }
            if (rxsize == 0 or rysize == 0) return error.UnsupportedFrameFeatures;
            const rfd = stream.FrameDim.init(rxsize, rysize, frame.group_size_shift, 1);
            const rnum_toc = toc_mod.numTocEntries(rfd.num_groups, rfd.num_dc_groups, frame.passes.num_passes);
            var rtoc = try toc_mod.readTocWithPermutation(allocator, rnum_toc, &br);
            defer rtoc.deinit(allocator);
            var rtoc_sizes: []u32 = undefined;
            var rsection_starts: []usize = undefined;
            try rtoc.logicalLayout(allocator, &rtoc_sizes, &rsection_starts);
            defer allocator.free(rtoc_sizes);
            defer allocator.free(rsection_starts);
            const rtotal = rtoc.totalSize();
            const rbase = br.totalBitsConsumed() / 8;
            if (rbase + rtotal > codestream.len) return error.NotEnoughBytes;
            const rsections = codestream[rbase .. rbase + rtotal];
            br.skipBits(rtotal * 8);

            var rimg = try decoder.decodeFrameImage(allocator, &metadata, &frame, rxsize, rysize, 1, rsections, rsection_starts, rtoc_sizes, null, null);
            defer rimg.deinit();
            var rplanes = try Planes.init(allocator, n_planes, rxsize, rysize);
            defer rplanes.deinit(allocator);
            {
                var y: usize = 0;
                while (y < rysize) : (y += 1) {
                    var x: usize = 0;
                    while (x < rxsize) : (x += 1) {
                        for (0..n_planes) |c| {
                            const v: f32 = @floatFromInt(rimg.channels.items[c].row(y)[x]);
                            rplanes.set(c, x, y, v * scale);
                        }
                    }
                }
            }
            if (frame.canBeReferenced()) {
                try saveReference(allocator, &refs, frame.save_as_reference, &rplanes, 0, 0);
                try storePatchRef(allocator, &patch_refs, frame.save_as_reference, &rplanes, true);
            }
            continue;
        }

        // Frame dimensions: the crop size, or the image size for full frames.
        var fxsize: u32 = @intCast(image_xsize);
        var fysize: u32 = @intCast(image_ysize);
        if (frame.custom_size_or_origin) {
            fxsize = frame.frame_size_x;
            fysize = frame.frame_size_y;
        }
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
        const sections = codestream[base .. base + total_len];
        br.skipBits(total_len * 8);

        // Decode the frame's channel planes (patch dictionary applied
        // inside when the frame header carries kPatches).
        var pd = patches_mod.PatchDictionary.init(allocator);
        defer pd.deinit();
        const wants_patches = frame.flags & frame_mod.Flags.kPatches != 0;
        var fimg = try decoder.decodeFrameImage(allocator, &metadata, &frame, fxsize, fysize, 1, sections, section_starts, toc_sizes, if (wants_patches) &patch_refs else null, if (wants_patches) &pd else null);
        defer fimg.deinit();

        // Frame planes in float [0,1].
        var fplanes = try Planes.init(allocator, n_planes, fxsize, fysize);
        defer fplanes.deinit(allocator);
        {
            var y: usize = 0;
            while (y < fysize) : (y += 1) {
                var x: usize = 0;
                while (x < fxsize) : (x += 1) {
                    for (0..n_planes) |c| {
                        const v: f32 = @floatFromInt(fimg.channels.items[c].row(y)[x]);
                        fplanes.set(c, x, y, v * scale);
                    }
                }
            }
        }

        // Pre-blend reference save (save_before_color_transform; for
        // color_transform == none the "before" image is the raw,
        // frame-sized planes at the frame origin).
        if (frame.canBeReferenced() and frame.save_before_color_transform) {
            try saveReference(allocator, &refs, frame.save_as_reference, &fplanes, frame.frame_origin_x, frame.frame_origin_y);
            try storePatchRef(allocator, &patch_refs, frame.save_as_reference, &fplanes, true);
        }

        // Does this frame need the blending stage? (libjxl NeedsBlending:
        // crop or any non-replace mode.) Full-frame replace frames skip it
        // entirely - no float math, so lossless roundtrips stay exact.
        var needs_blending = frame.custom_size_or_origin or
            frame.blending_info.mode != .replace;
        for (frame.extra_channel_blending_info.items) |*ecb| {
            if (ecb.mode != .replace) needs_blending = true;
        }

        if (needs_blending) {
            // The blending stage switches the render pipeline to full-canvas
            // dimensions (libjxl `SwitchToImageDimensions`): inside the
            // frame rect pixels blend against the reference frames, outside
            // they are copied from the color source reference (padding
            // rows). The full-canvas result becomes both the canvas state
            // and (post-transform) the saved reference at origin (0,0).
            var out = try blendFrameCanvas(allocator, &fplanes, &frame, &metadata, &refs, alpha_ec, n_color, n_ec, image_xsize, image_ysize);
            defer out.deinit(allocator);
            copyPlanes(&canvas, &out);
            if (frame.canBeReferenced() and !frame.save_before_color_transform) {
                try saveReference(allocator, &refs, frame.save_as_reference, &out, 0, 0);
            }
        } else {
            // Full-frame replace: the frame covers the whole canvas.
            copyToCanvas(&canvas, &fplanes, 0, 0, n_planes);
            if (frame.canBeReferenced() and !frame.save_before_color_transform) {
                try saveReference(allocator, &refs, frame.save_as_reference, &fplanes, 0, 0);
            }
        }

        // Visible frames snapshot the canvas.
        const visible = frame.is_last or frame.animation_frame_duration > 0;
        if (visible and (frame.frame_type == .regular or frame.frame_type == .skip_progressive)) {
            var pb = try snapshotPixels(allocator, &canvas, image_xsize, image_ysize, n_color, alpha_ec != null, @intCast(bitdepth), max_val);
            errdefer pb.deinit();
            try anim.frames.append(allocator, .{
                .pixels = pb,
                .duration_ticks = frame.animation_frame_duration,
                .timecode = frame.animation_frame_timecode,
            });
        }

        done = frame.is_last;
    }

    if (anim.frames.items.len == 0) return error.NoFrames;
    return anim;
}

fn checkFrame(frame: *const frame_mod.FrameHeader) DecodeError!void {
    if (frame.encoding != .modular) return error.UnsupportedVarDct;
    if (frame.frame_type == .dc_frame) return error.UnsupportedFrameFeatures;
    if (frame.color_transform != .none) return error.UnsupportedColorTransform;
    if (frame.upsampling != 1) return error.UnsupportedFrameFeatures;
    for (frame.extra_channel_upsampling.items) |ecup| {
        if (ecup != 1) return error.UnsupportedFrameFeatures;
    }
    // Progressive (multi-pass) modular frames are decoded like any other
    // modular frame (the pass loop lives in the frame decoder).
    if (frame.loop_filter.gab or frame.loop_filter.epf_iters != 0) {
        return error.UnsupportedLoopFilter;
    }
}

fn saveReference(allocator: Allocator, refs: *[4]?RefFrame, slot: u32, planes: *const Planes, origin_x: i32, origin_y: i32) !void {
    if (slot > 3) return error.InvalidBlendSource;
    if (refs[slot]) |*old| old.deinit(allocator);
    const plane_list = try allocator.alloc([]f32, planes.data.len);
    var initialized: usize = 0;
    errdefer {
        for (plane_list[0..initialized]) |p| allocator.free(p);
        allocator.free(plane_list);
    }
    for (planes.data, 0..) |src, i| {
        plane_list[i] = try allocator.dupe(f32, src);
        initialized += 1;
    }
    refs[slot] = .{
        .planes = plane_list,
        .xsize = planes.xsize,
        .ysize = planes.ysize,
        .origin_x = origin_x,
        .origin_y = origin_y,
    };
}

/// Stores a copy of `planes` (float [0,1]) into the patch reference store
/// slot (ownership of the copies moves to the store; freed by
/// `decoder.freeRefStore` on the caller's side).
fn storePatchRef(allocator: Allocator, patch_refs: *patches_mod.RefStore, slot: u32, planes: *const Planes, before_color_transform: bool) !void {
    if (slot > 3) return error.InvalidBlendSource;
    if (slot == 0) return; // not saved anywhere
    if (patch_refs[slot]) |*old| {
        for (old.planes) |p| allocator.free(p);
        allocator.free(old.planes);
        patch_refs[slot] = null;
    }
    const plane_list = try allocator.alloc([]f32, planes.data.len);
    var initialized: usize = 0;
    errdefer {
        for (plane_list[0..initialized]) |p| allocator.free(p);
        allocator.free(plane_list);
    }
    for (planes.data, 0..) |src, i| {
        plane_list[i] = try allocator.dupe(f32, src);
        initialized += 1;
    }
    patch_refs[slot] = .{
        .planes = plane_list,
        .xsize = planes.xsize,
        .ysize = planes.ysize,
        .origin_x = 0,
        .origin_y = 0,
        .before_color_transform = before_color_transform,
    };
}

/// Copies all planes wholesale (same dimensions).
fn copyPlanes(dst: *Planes, src: *const Planes) void {
    for (src.data, 0..) |s, c| {
        @memcpy(dst.data[c], s);
    }
}

/// Copies the visible part of a frame-sized plane set into the canvas.
fn copyToCanvas(canvas: *Planes, frame_planes: *const Planes, origin_x: i32, origin_y: i32, n_planes: usize) void {
    var y: i32 = 0;
    while (y < frame_planes.ysize) : (y += 1) {
        const cy = origin_y + y;
        if (cy < 0 or cy >= canvas.ysize) continue;
        var x: i32 = 0;
        while (x < frame_planes.xsize) : (x += 1) {
            const cx = origin_x + x;
            if (cx < 0 or cx >= canvas.xsize) continue;
            for (0..n_planes) |c| {
                canvas.set(c, @intCast(cx), @intCast(cy), frame_planes.at(c, @intCast(x), @intCast(y)));
            }
        }
    }
}

/// Port of the BlendingStage + PerformBlending at full-canvas dimensions
/// (libjxl `SwitchToImageDimensions`): inside the frame rect the channels
/// blend against the reference frames selected by each BlendingInfo's
/// source; outside the rect the color channels copy the color source
/// reference and each extra channel copies its own source reference
/// (ProcessPaddingRow; empty references read as zero).
fn blendFrameCanvas(
    allocator: Allocator,
    fplanes: *Planes,
    frame: *const frame_mod.FrameHeader,
    metadata: *const metadata_mod.ImageMetadata,
    refs: *[4]?RefFrame,
    alpha_ec: ?usize,
    n_color: usize,
    n_ec: usize,
    image_xsize: usize,
    image_ysize: usize,
) !Planes {
    const color_info = frame.blending_info;

    // Background references (validated to cover the canvas when non-empty;
    // the color source is always used here because the padding rows read it).
    const bg_color = try getBg(refs, color_info, true, image_xsize, image_ysize);
    var bg_ecs: [4]?*const RefFrame = .{ null, null, null, null };
    for (frame.extra_channel_blending_info.items, 0..) |*ecb, i| {
        if (i >= 4) return error.UnsupportedExtraChannels;
        bg_ecs[i] = try getBg(refs, ecb.*, true, image_xsize, image_ysize);
    }

    const has_alpha = alpha_ec != null;
    const color_alpha_ec = color_info.alpha_channel;
    const ec_base = n_color;
    const x0 = frame.frame_origin_x;
    const y0 = frame.frame_origin_y;

    var out = try Planes.init(allocator, n_planes_total(n_color, n_ec), image_xsize, image_ysize);
    errdefer out.deinit(allocator);

    var cy: i32 = 0;
    while (cy < image_ysize) : (cy += 1) {
        var cx: i32 = 0;
        while (cx < image_xsize) : (cx += 1) {
            const ux: usize = @intCast(cx);
            const uy: usize = @intCast(cy);
            const inside = cx >= x0 and cy >= y0 and
                cx < x0 + @as(i32, @intCast(fplanes.xsize)) and
                cy < y0 + @as(i32, @intCast(fplanes.ysize));

            if (!inside) {
                // Padding row: copy the backgrounds (zeros when empty).
                for (0..n_color) |c| {
                    out.set(c, ux, uy, if (bg_color) |rf| refAt(rf, c, cx, cy) else 0);
                }
                for (0..n_ec) |i| {
                    out.set(ec_base + i, ux, uy, if (bg_ecs[i]) |rf| refAt(rf, ec_base + i, cx, cy) else 0);
                }
                continue;
            }

            const fx: usize = @intCast(cx - x0);
            const fy: usize = @intCast(cy - y0);

            // ---- extra channels first (pre-blend alpha) ----
            var ec_val: [4]f32 = .{ 0, 0, 0, 0 };
            for (frame.extra_channel_blending_info.items, 0..) |*ecb, i| {
                const plane = ec_base + i;
                const fg = fplanes.at(plane, fx, fy);
                const bg: f32 = if (bg_ecs[i]) |rf| refAt(rf, plane, cx, cy) else 0;
                switch (ecb.mode) {
                    .replace => ec_val[i] = fg,
                    .add => ec_val[i] = bg + fg,
                    .alpha_weighted_add => {
                        const fga = fplanes.at(ec_base + ecb.alpha_channel, fx, fy);
                        ec_val[i] = bg + fg * (if (ecb.clamp) clamp01(fga) else fga);
                    },
                    .blend => {
                        const a_plane = ec_base + ecb.alpha_channel;
                        const fga = fplanes.at(a_plane, fx, fy);
                        const bga: f32 = if (bg_ecs[i]) |rf| refAt(rf, a_plane, cx, cy) else 0;
                        const premul = metadata.extra_channel_info.items[ecb.alpha_channel].alpha_associated;
                        var out_a: f32 = 0;
                        const v = alphaBlendEc(bg, bga, fg, fga, premul, ecb.clamp, &out_a);
                        ec_val[i] = if (ecb.alpha_channel == i) out_a else v;
                    },
                    .mul => {
                        ec_val[i] = if (ecb.clamp) bg * clamp01(fg) else bg * fg;
                    },
                }
            }

            // ---- color channels ----
            var color_val: [3]f32 = .{ 0, 0, 0 };
            for (0..n_color) |c| {
                const fg = fplanes.at(c, fx, fy);
                const bg: f32 = if (bg_color) |rf| refAt(rf, c, cx, cy) else 0;
                color_val[c] = switch (color_info.mode) {
                    .replace => fg,
                    .add => bg + fg,
                    .mul => if (color_info.clamp) bg * clamp01(fg) else bg * fg,
                    .alpha_weighted_add => blk: {
                        if (!has_alpha) break :blk bg + fg;
                        const fga = fplanes.at(ec_base + color_alpha_ec, fx, fy);
                        break :blk bg + fg * (if (color_info.clamp) clamp01(fga) else fga);
                    },
                    .blend => blk: {
                        if (!has_alpha) break :blk fg;
                        const a_plane = ec_base + color_alpha_ec;
                        const fga = fplanes.at(a_plane, fx, fy);
                        const bga: f32 = if (bg_color) |rf| refAt(rf, a_plane, cx, cy) else 0;
                        const premul = metadata.extra_channel_info.items[color_alpha_ec].alpha_associated;
                        var out_a: f32 = 0;
                        const v = alphaBlendEc(bg, bga, fg, fga, premul, color_info.clamp, &out_a);
                        // PerformBlending writes the blended alpha back to
                        // the alpha output row (color blend runs last).
                        ec_val[color_alpha_ec] = out_a;
                        break :blk v;
                    },
                };
            }

            for (0..n_color) |c| out.set(c, ux, uy, color_val[c]);
            for (0..n_ec) |i| out.set(ec_base + i, ux, uy, ec_val[i]);
        }
    }
    return out;
}

inline fn n_planes_total(n_color: usize, n_ec: usize) usize {
    return n_color + n_ec;
}

/// Returns the background reference for a blending info; `null` for an
/// empty slot (zero background). A non-empty reference that does not cover
/// the canvas is an error (libjxl `verify_bg_size`).
fn getBg(refs: *[4]?RefFrame, info: frame_mod.BlendingInfo, used: bool, canvas_xsize: usize, canvas_ysize: usize) !?*const RefFrame {
    if (!used) return null;
    if (refs[info.source] == null) return null;
    const rf = &refs[info.source].?;
    if (rf.origin_x != 0 or rf.origin_y != 0 or
        rf.xsize < canvas_xsize or rf.ysize < canvas_ysize)
    {
        return error.InvalidBlendSource;
    }
    return rf;
}

inline fn refAt(rf: *const RefFrame, plane: usize, canvas_x: i32, canvas_y: i32) f32 {
    const rx = canvas_x - rf.origin_x;
    const ry = canvas_y - rf.origin_y;
    if (rx < 0 or ry < 0) return 0;
    const ux: usize = @intCast(rx);
    const uy: usize = @intCast(ry);
    if (ux >= rf.xsize or uy >= rf.ysize) return 0;
    if (plane >= rf.planes.len) return 0;
    return rf.planes[plane][uy * rf.xsize + ux];
}

/// Converts the float canvas to a PixelBuffer (libjxl float->int:
/// NearestInt(clamp(v, 0, 1) * max) with round-half-to-even).
fn snapshotPixels(
    allocator: Allocator,
    canvas: *const Planes,
    xsize: usize,
    ysize: usize,
    n_color: usize,
    has_alpha: bool,
    bitdepth: u8,
    max_val: f32,
) !PixelBuffer {
    const layout: pixel_mod.Layout = if (n_color == 1) .gray else .rgb;
    var pb = try PixelBuffer.create(allocator, xsize, ysize, layout, has_alpha, bitdepth);
    errdefer pb.deinit();
    const nch = pb.numChannels();
    const max_int: i32 = @intCast((@as(u32, 1) << @intCast(bitdepth)) - 1);
    for (0..ysize) |y| {
        for (0..xsize) |x| {
            for (0..nch) |c| {
                const v = canvas.at(c, x, y);
                const scaled = clamp01(v) * max_val;
                const rounded = nearestEven(scaled);
                pb.set(x, y, c, @intCast(std.math.clamp(rounded, 0, max_int)));
            }
        }
    }
    return pb;
}
