//! Lossless JPEG <-> JXL transcoding, stage 2: the VarDCT path.
//!
//! JPEG -> JPEGData parse -> DCT coefficients stored in a kVarDCT frame
//! with RAW quant tables (bit-exactly as libjxl's ComputeJPEGTranscodingData
//! does) -> container: signature, ftyp, `jbrd` (bit-exact reconstruction
//! metadata), unbounded jxlc holding the codestream. The `.jxl` output is
//! decodable by libjxl (`djxl`), both to pixels and back to the original
//! JPEG bytes; the older stage-1 carrier (a modular frame plus private
//! `jzzd` box) is gone.

const std = @import("std");

const jd = @import("jpeg_data.zig");
const reader = @import("reader.zig");
const writer = @import("writer.zig");
const jbrd = @import("jbrd.zig");

const container_mod = @import("../header/container.zig");
const bit_io = @import("../base/bit_io.zig");
const icc_mod = @import("../header/icc.zig");
const frame_encode = @import("../vardct/frame_encode.zig");
const frame_decode = @import("../vardct/frame_decode.zig");

const BitWriter = bit_io.BitWriter;

/// Transcode options (encoder side).
pub const EncodeOptions = struct {
    /// Brotli quality for the `jbrd` payload (libjxl `brotli_effort`,
    /// cjxl default 9). 0 = stored meta-blocks (no compression).
    brotli_quality: u32 = 9,
    /// Chroma-from-luma for 444 3-component frames (libjxl
    /// `force_cfl_jpeg_recompression`, default on).
    cfl: bool = true,
    /// Custom (learned) coefficient orders (libjxl ComputeCoeffOrder,
    /// default on at every effort slower than kFalcon).
    coeff_orders: bool = true,
    /// cparams.centerfirst (cjxl `--group_order=1`): write the TOC with
    /// the center-first AC-group permutation.
    group_order: bool = false,
};

pub const Error = error{
    NotAJpeg,
    NotAJxlContainer,
    MissingJbrdBox,
    OutOfMemory,
} || frame_encode.Error || frame_decode.Error || jbrd.Error || reader.Error || writer.Error || anyerror;

/// ftyp payload: major brand "jxl ", minor version 0, compatible "jxl ".
const ftyp_payload = [12]u8{
    'j', 'x', 'l', ' ', 0, 0, 0, 0, 'j', 'x', 'l', ' ',
};

/// Encodes a JPEG into a JXL container (transcode).
pub fn encodeTranscode(allocator: std.mem.Allocator, jpeg_bytes: []const u8) Error![]u8 {
    return encodeTranscodeOptions(allocator, jpeg_bytes, .{});
}

/// Encodes a JPEG into a JXL container (transcode) with explicit options.
pub fn encodeTranscodeOptions(allocator: std.mem.Allocator, jpeg_bytes: []const u8, options: EncodeOptions) Error![]u8 {
    if (jpeg_bytes.len < 2 or jpeg_bytes[0] != 0xFF or jpeg_bytes[1] != 0xD8) {
        return Error.NotAJpeg;
    }
    var jpg = jd.JPEGData{};
    defer jpg.deinit(allocator);
    try reader.readJpeg(jpeg_bytes, .read_all, &jpg, allocator);

    // Detect a canonically chunked APP2 ICC profile (libjxl
    // `DetectIccProfile`): when the profile is a complete RGB/GRAY profile
    // (what libjxl's SetFieldsFromICC effectively requires) the payloads are
    // reconstructed from the codestream headers instead of riding the
    // `jbrd` brotli blob, and the headers carry the real profile
    // (`want_icc`). Truncated or otherwise invalid profiles — which libjxl
    // rejects outright — fall back to the sRGB-headers path with the raw
    // payload kept in `jbrd`.
    var icc: []const u8 = &.{};
    if (jbrd.detectIccProfile(&jpg)) {
        if (try jbrd.extractIcc(allocator, &jpg)) |icc_owned| {
            if (icc_mod.looksLikeValidProfile(icc_owned)) {
                icc = icc_owned;
            } else {
                allocator.free(icc_owned);
                @memset(jpg.app_marker_type.items, .unknown);
            }
        }
    } else |_| {
        @memset(jpg.app_marker_type.items, .unknown);
    }
    defer if (icc.len > 0) allocator.free(icc);

    // Codestream (VarDCT frame with the JPEG's coefficients).
    var codestream: BitWriter = .init(allocator);
    defer codestream.deinit();
    try frame_encode.encodeTranscodeFrameOptions(allocator, &jpg, icc, &codestream, .{
        .cfl = options.cfl,
        .coeff_orders = options.coeff_orders,
        .group_order = options.group_order,
    });
    codestream.zeroPadToByte();
    const cs_bytes = try codestream.takeBytes();
    defer allocator.free(cs_bytes);

    // jbrd payload.
    const jbrd_payload = try jbrd.encode(allocator, &jpg, .{ .brotli_quality = options.brotli_quality });
    defer allocator.free(jbrd_payload);

    // Container: signature, ftyp, jbrd, unbounded jxlc.
    var out: std.ArrayList(u8) = .empty;
    errdefer out.deinit(allocator);
    try out.appendSlice(allocator, &container_mod.signature);
    try container_mod.writeBox(allocator, &out, "ftyp", &ftyp_payload);
    try container_mod.writeBox(allocator, &out, "jbrd", jbrd_payload);
    // Unbounded jxlc: header with size 0, payload to end of file.
    var header: [8]u8 = undefined;
    std.mem.writeInt(u32, header[0..4], 0, .big);
    @memcpy(header[4..8], "jxlc");
    try out.appendSlice(allocator, &header);
    try out.appendSlice(allocator, cs_bytes);
    return out.toOwnedSlice(allocator);
}

/// Decodes a transcode container back into JPEG bytes.
pub fn decodeTranscode(allocator: std.mem.Allocator, jxl_bytes: []const u8) Error![]u8 {
    if (!container_mod.isContainer(jxl_bytes)) return Error.NotAJxlContainer;
    const jbrd_payload = container_mod.findBox(jxl_bytes, "jbrd") orelse return Error.MissingJbrdBox;
    const cs = try container_mod.extractCodestream(allocator, jxl_bytes);
    defer allocator.free(cs);

    var jpg = jd.JPEGData{};
    defer jpg.deinit(allocator);
    try jbrd.decode(allocator, jbrd_payload, &jpg);

    // Metadata boxes: cjxl moves Exif/XMP payloads out of the `jbrd` blob
    // into container boxes (compressed as `brob` when --compress_boxes,
    // the default). Restore them into the typed APP1 markers, exactly as
    // libjxl's JxlToJpegDecoder::SetExif/SetXMP do.
    try restoreMetadataBoxes(allocator, jxl_bytes, &jpg);

    // The ICC-typed APP2 marker payloads are not in the `jbrd` blob: they
    // are rebuilt from the codestream-header ICC profile (libjxl
    // `SetJPEGDataFromICC`).
    var icc: ?[]u8 = null;
    defer if (icc) |b| allocator.free(b);
    try frame_decode.decodeTranscodeFrame(allocator, cs, &jpg, &icc);
    if (icc) |b| try jbrd.setJpegDataFromIcc(b, &jpg);

    var out: std.ArrayList(u8) = .empty;
    errdefer out.deinit(allocator);
    try writer.writeJpeg(&jpg, &out, allocator);
    return out.toOwnedSlice(allocator);
}

/// Decompresses `brob` metadata boxes and injects Exif/`xml ` content into
/// the typed APP1 markers of `jpg` (cjxl-produced containers).
fn restoreMetadataBoxes(allocator: std.mem.Allocator, jxl_bytes: []const u8, jpg: *jd.JPEGData) !void {
    var boxes = container_mod.listBoxes(allocator, jxl_bytes) catch return;
    defer boxes.deinit(allocator);
    for (boxes.items) |box| {
        if (!std.mem.eql(u8, &box.type, "brob")) continue;
        var decoded = container_mod.decodeBrob(allocator, box.data) catch continue;
        defer decoded.deinit(allocator);
        if (std.mem.eql(u8, &decoded.kind, "Exif")) {
            try jbrd.setExif(decoded.content, jpg);
        } else if (std.mem.eql(u8, &decoded.kind, "xml ") or
            std.mem.eql(u8, &decoded.kind, "xml"))
        {
            try jbrd.setXmp(decoded.content, jpg);
        }
    }
}
