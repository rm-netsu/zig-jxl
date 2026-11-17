//! Bit-exact serialization of the JPEG reconstruction metadata ("jbrd"),
//! port of libjxl `jpeg/jpeg_data.cc` JPEGData::VisitFields.
//!
//! Layout: [field bits, LSB-first] [pad to byte] [brotli blob with the
//! raw APP/COM/inter-marker/tail data].

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const jd = @import("jpeg_data.zig");
const brotli = @import("brotli.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const U32Enc = fields.U32Enc;
const val = fields.val;
const bitsOffset = fields.bitsOffset;
const bitsDistr = fields.bitsDistr;

const JPEGData = jd.JPEGData;
const JPEGScanInfo = jd.JPEGScanInfo;

pub const Error = error{
    InvalidJpegData,
    NoFeasibleSelector,
    BrotliCorrupt,
    OutOfMemory,
};

/// Options for the `jbrd` box payload (Brotli compression of the raw
/// APP/COM/inter-marker/tail blob, mirroring libjxl `EncodeJPEGData`
/// which uses `cparams.brotli_effort` — cjxl default 9).
pub const Options = struct {
    /// Brotli quality, 0..11. 0 emits stored (raw) meta-blocks like the
    /// original stub writer; 1..11 use real Brotli.
    brotli_quality: u32 = 9,
};

const app_marker_type_enc = U32Enc.init(val(0), val(1), bitsOffset(1, 2), bitsOffset(2, 4));
const num_quant_tables_enc = U32Enc.init(val(1), val(2), val(3), val(4));
const num_components_enc = U32Enc.init(val(1), val(2), val(3), val(4));
const huff_count_enc = U32Enc.init(val(0), val(1), bitsOffset(3, 2), bitsDistr(8));
const huff_value_enc = U32Enc.init(bitsDistr(2), bitsOffset(2, 4), bitsOffset(4, 8), bitsOffset(8, 1));
const num_huff_enc = U32Enc.init(val(4), bitsOffset(3, 2), bitsOffset(4, 10), bitsOffset(6, 26));
const last_needed_pass_enc = U32Enc.init(val(0), val(1), val(2), bitsOffset(3, 3));
const count_enc = U32Enc.init(val(0), bitsOffset(2, 1), bitsOffset(4, 4), bitsOffset(16, 20));
const reset_point_enc = U32Enc.init(val(0), bitsOffset(3, 1), bitsOffset(5, 9), bitsOffset(28, 41));
const extra_zero_runs_enc = U32Enc.init(val(1), bitsOffset(2, 2), bitsOffset(4, 5), bitsOffset(8, 20));
const tail_data_len_enc = U32Enc.init(val(0), bitsOffset(8, 1), bitsOffset(16, 257), bitsOffset(22, 65793));

const JPEGComponentType = enum(u2) {
    gray = 0,
    ycbcr = 1,
    rgb = 2,
    custom = 3,
};

const JpegInfo = struct {
    num_app_markers: usize = 0,
    num_com_markers: usize = 0,
    num_scans: usize = 0,
    num_intermarker: usize = 0,
    has_dri: bool = false,
};

fn visitMarker(marker: *u8, info: *JpegInfo) void {
    marker.* = marker.* -% 0xc0;
    const m = marker.* +% 0xc0;
    if ((m & 0xf0) == 0xe0) info.num_app_markers += 1;
    if (m == 0xfe) info.num_com_markers += 1;
    if (m == 0xda) info.num_scans += 1;
    if (m == 0xff) info.num_intermarker += 1;
    if (m == 0xdd) info.has_dri = true;
}

/// Marker payload without type byte and length; null on malformed markers.
fn getMarkerPayload(marker: []const u8) ?[]const u8 {
    if (marker.len < 3) return null;
    const internal_size: usize = (@as(usize, marker[1]) << 8) | marker[2];
    if (internal_size != marker.len - 1) return null;
    return marker[3..];
}

/// See if there is a canonically chunked ICC profile and mark the
/// corresponding APP2 markers with AppMarkerType::kICC.
pub fn detectIccProfile(jpg: *JPEGData) Error!void {
    if (jpg.app_data.items.len != jpg.app_marker_type.items.len) return Error.InvalidJpegData;
    var num_icc: usize = 0;
    var num_icc_jpeg: usize = 0;
    for (jpg.app_data.items, 0..) |app, i| {
        if (app.items.len < 3) return Error.InvalidJpegData;
        var pos: usize = 0;
        if (app.items[pos] != 0xE2) continue;
        pos += 1;
        const tag_length: usize = (@as(usize, app.items[pos]) << 8) + app.items[pos + 1];
        pos += 2;
        if (app.items.len != tag_length + 1) return Error.InvalidJpegData;
        if (tag_length < 2 + kIccProfileTag.len + 2) continue;
        if (!std.mem.eql(u8, app.items[pos..][0..kIccProfileTag.len], kIccProfileTag)) continue;
        pos += kIccProfileTag.len;
        const chunk_id = app.items[pos];
        pos += 1;
        const num_chunks = app.items[pos];
        pos += 1;
        if (chunk_id != num_icc + 1) continue;
        if (num_icc_jpeg == 0) num_icc_jpeg = num_chunks;
        if (num_icc_jpeg != num_chunks) continue;
        num_icc += 1;
        jpg.app_marker_type.items[i] = .icc;
    }
    if (num_icc != num_icc_jpeg) return Error.InvalidJpegData;
}

/// Marks Exif (APP1 with "Exif\0") and XMP (APP1 with the XMP namespace)
/// markers.
pub fn detectBlobs(jpg: *JPEGData) Error!void {
    if (jpg.app_data.items.len != jpg.app_marker_type.items.len) return Error.InvalidJpegData;
    var have_exif = false;
    var have_xmp = false;
    for (jpg.app_data.items, 0..) |marker, i| {
        if (marker.items.len == 0 or marker.items[0] != jd.kApp1) continue;
        const payload = getMarkerPayload(marker.items) orelse continue;
        if (!have_exif and payload.len > kExifTag.len and
            std.mem.eql(u8, payload[0..kExifTag.len], kExifTag))
        {
            jpg.app_marker_type.items[i] = .exif;
            have_exif = true;
        }
        if (!have_xmp and payload.len >= kXMPTag.len and
            std.mem.eql(u8, payload[0..kXMPTag.len], kXMPTag))
        {
            jpg.app_marker_type.items[i] = .xmp;
            have_xmp = true;
        }
    }
}

/// Extracts the raw ICC profile from the ICC-typed APP2 markers (port of
/// `ParseChunkedMarker` for the ICC tag; requires the canonical chunking
/// validated by `detectIccProfile`). Returns null when no ICC markers are
/// present.
pub fn extractIcc(allocator: std.mem.Allocator, jpg: *const JPEGData) Error!?[]u8 {
    if (jpg.app_data.items.len != jpg.app_marker_type.items.len) return Error.InvalidJpegData;
    var out: std.ArrayList(u8) = .empty;
    errdefer out.deinit(allocator);
    var num_chunks_expected: usize = 0;
    var ordinal: usize = 0;
    for (jpg.app_data.items, 0..) |marker, i| {
        if (jpg.app_marker_type.items[i] != .icc) continue;
        const payload = getMarkerPayload(marker.items) orelse return Error.InvalidJpegData;
        if (payload.len < kIccProfileTag.len + 2) return Error.InvalidJpegData;
        if (!std.mem.eql(u8, payload[0..kIccProfileTag.len], kIccProfileTag)) {
            return Error.InvalidJpegData;
        }
        const index = payload[kIccProfileTag.len];
        const total = payload[kIccProfileTag.len + 1];
        ordinal += 1;
        if (index != ordinal) return Error.InvalidJpegData; // Invalid chunk order.
        if (total == 0) return Error.InvalidJpegData;
        if (num_chunks_expected == 0) {
            num_chunks_expected = total;
        } else if (num_chunks_expected != total) {
            return Error.InvalidJpegData;
        }
        if (index > total) return Error.InvalidJpegData; // Invalid chunk index.
        try out.appendSlice(allocator, payload[kIccProfileTag.len + 2 ..]);
    }
    if (ordinal == 0) return null;
    if (ordinal != num_chunks_expected) return Error.InvalidJpegData; // Missing chunk.
    return try out.toOwnedSlice(allocator);
}

/// Fills the payloads of the ICC-typed APP2 markers from the raw ICC
/// profile (port of `SetJPEGDataFromICC`): the `jbrd` box carries only the
/// marker skeletons, the profile itself travels in the codestream headers.
pub fn setJpegDataFromIcc(icc: []const u8, jpg: *JPEGData) Error!void {
    if (jpg.app_data.items.len != jpg.app_marker_type.items.len) return Error.InvalidJpegData;
    var icc_pos: usize = 0;
    for (jpg.app_data.items, 0..) |*marker, i| {
        if (jpg.app_marker_type.items[i] != .icc) continue;
        if (marker.items.len < 17) return Error.InvalidJpegData;
        const len = marker.items.len - 17;
        if (icc_pos + len > icc.len) return Error.InvalidJpegData;
        @memcpy(marker.items[17..], icc[icc_pos .. icc_pos + len]);
        icc_pos += len;
    }
    if (icc_pos != icc.len and icc_pos != 0) return Error.InvalidJpegData;
}

const kIccProfileTag = jd.kIccProfileTag;
const kExifTag = jd.kExifTag;
const kXMPTag = jd.kXMPTag;

/// Fills the content of the first Exif-typed APP1 marker from a decoded
/// `Exif` box (port of `JxlToJpegDecoder::SetExif`): the box content starts
/// with a 4-byte TIFF header that is not part of the JPEG marker.
pub fn setExif(data: []const u8, jpg: *JPEGData) Error!void {
    if (data.len < 4) return Error.InvalidJpegData;
    for (jpg.app_data.items, 0..) |*marker, i| {
        if (jpg.app_marker_type.items[i] != .exif) continue;
        if (marker.items.len != data.len + 3 + kExifTag.len - 4) {
            return Error.InvalidJpegData;
        }
        marker.items[0] = 0xE1;
        // Bytes 1-2 (marker size) are already set by the jbrd decode.
        @memcpy(marker.items[3..][0..kExifTag.len], kExifTag);
        @memcpy(marker.items[3 + kExifTag.len ..], data[4..]);
        return;
    }
    return Error.InvalidJpegData;
}

/// Fills the content of the first XMP-typed APP1 marker from a decoded
/// `xml ` box (port of `JxlToJpegDecoder::SetXMP`).
pub fn setXmp(data: []const u8, jpg: *JPEGData) Error!void {
    for (jpg.app_data.items, 0..) |*marker, i| {
        if (jpg.app_marker_type.items[i] != .xmp) continue;
        if (marker.items.len != data.len + 3 + kXMPTag.len) {
            return Error.InvalidJpegData;
        }
        marker.items[0] = 0xE1;
        @memcpy(marker.items[3..][0..kXMPTag.len], kXMPTag);
        @memcpy(marker.items[3 + kXMPTag.len ..], data);
        return;
    }
    return Error.InvalidJpegData;
}

/// Serializes the JPEGData fields into `writer` (port of Bundle::Write).
/// APP markers marked with a non-unknown type are reconstructed by the
/// decoder from the codestream, so the caller must ensure their payloads
/// are available (e.g. via SetJPEGDataFromICC); when unsure, leave all
/// types unknown, which routes the raw bytes through the brotli blob.
pub fn writeFields(jpg: *JPEGData, writer: *BitWriter, allocator: std.mem.Allocator) Error!void {
    // Mirrors EncodeJPEGData: missing type entries default to unknown;
    // callers may pre-fill them via detectAppMarkerTypes.
    const old_len = jpg.app_marker_type.items.len;
    try jpg.app_marker_type.resize(allocator, jpg.app_data.items.len);
    for (jpg.app_marker_type.items[old_len..]) |*t| {
        t.* = .unknown;
    }
    const is_gray = jpg.components.items.len == 1;
    writer.write(1, @intFromBool(is_gray));

    var info = JpegInfo{};
    if (jpg.marker_order.items.len > 16384) return Error.InvalidJpegData;
    if (jpg.marker_order.items.len == 0 or jpg.marker_order.items[jpg.marker_order.items.len - 1] != 0xd9) {
        return Error.InvalidJpegData;
    }
    for (jpg.marker_order.items) |m| {
        var marker = m;
        visitMarker(&marker, &info);
        writer.write(6, marker);
    }
    if (info.num_scans == 0) return Error.InvalidJpegData;

    std.debug.assert(jpg.app_data.items.len == info.num_app_markers);
    for (jpg.app_data.items, 0..) |*app, i| {
        const t: u32 = @intFromEnum(jpg.app_marker_type.items[i]);
        U32Coder.write(app_marker_type_enc, t, writer) catch return Error.NoFeasibleSelector;
        const len: u32 = @intCast(app.items.len - 1);
        writer.write(16, len);
        if (app.items.len < 3) return Error.InvalidJpegData;
    }
    for (jpg.com_data.items) |*com| {
        const len: u32 = @intCast(com.items.len - 1);
        writer.write(16, len);
        if (com.items.len < 3) return Error.InvalidJpegData;
    }

    const num_quant_tables: u32 = @intCast(jpg.quant.items.len);
    if (num_quant_tables == 4 or num_quant_tables == 0) return Error.InvalidJpegData;
    U32Coder.write(num_quant_tables_enc, num_quant_tables, writer) catch return Error.NoFeasibleSelector;
    for (jpg.quant.items) |*q| {
        if (q.precision > 1) return Error.InvalidJpegData;
        writer.write(1, q.precision);
        writer.write(2, q.index);
        writer.write(1, @intFromBool(q.is_last));
    }

    var component_type: JPEGComponentType = .custom;
    if (jpg.components.items.len == 1 and jpg.components.items[0].id == 1) {
        component_type = .gray;
    } else if (jpg.components.items.len == 3 and jpg.components.items[0].id == 1 and
        jpg.components.items[1].id == 2 and jpg.components.items[2].id == 3)
    {
        component_type = .ycbcr;
    } else if (jpg.components.items.len == 3 and jpg.components.items[0].id == 'R' and
        jpg.components.items[1].id == 'G' and jpg.components.items[2].id == 'B')
    {
        component_type = .rgb;
    }
    writer.write(2, @intFromEnum(component_type));
    if (component_type == .custom) {
        for (jpg.components.items) |*c| {
            writer.write(8, c.id);
        }
    }
    var used_tables: u32 = 0;
    for (jpg.components.items) |*c| {
        if (c.quant_idx >= jpg.quant.items.len) return Error.InvalidJpegData;
        writer.write(2, c.quant_idx);
        used_tables |= @as(u32, 1) << @intCast(c.quant_idx);
    }
    for (jpg.quant.items, 0..) |*q, i| {
        if (used_tables & (@as(u32, 1) << @intCast(i)) != 0) continue;
        if (i == 0) return Error.InvalidJpegData; // first quant table unused
        // Unused quant table must be a copy of the previous one.
        if (!std.mem.eql(i32, &q.values, &jpg.quant.items[i - 1].values)) {
            return Error.InvalidJpegData;
        }
    }

    const num_huff: u32 = @intCast(jpg.huffman_code.items.len);
    U32Coder.write(num_huff_enc, num_huff, writer) catch return Error.NoFeasibleSelector;
    for (jpg.huffman_code.items) |*hc| {
        const is_ac = (hc.slot_id >> 4) != 0;
        const id: u32 = @intCast(hc.slot_id & 0xF);
        writer.write(1, @intFromBool(is_ac));
        writer.write(2, id);
        writer.write(1, @intFromBool(hc.is_last));
        var num_symbols: usize = 0;
        for (0..jd.kJpegHuffmanMaxBitLength + 1) |i| {
            U32Coder.write(huff_count_enc, hc.counts[i], writer) catch return Error.NoFeasibleSelector;
            num_symbols += hc.counts[i];
        }
        if (num_symbols == 0) continue; // empty DHT marker
        if (num_symbols > hc.values.len) return Error.InvalidJpegData;
        var value_slots = [_]u64{0} ** 5;
        for (0..num_symbols) |i| {
            U32Coder.write(huff_value_enc, hc.values[i], writer) catch return Error.NoFeasibleSelector;
            value_slots[hc.values[i] >> 6] |= @as(u64, 1) << @intCast(hc.values[i] & 0x3F);
        }
        if (hc.values[num_symbols - 1] != jd.kJpegHuffmanAlphabetSize) return Error.InvalidJpegData;
        var num_values: usize = 1;
        for (0..4) |i| num_values += @popCount(value_slots[i]);
        if (num_values != num_symbols) return Error.InvalidJpegData;
        if (!is_ac) {
            const only_dc = ((value_slots[0] >> jd.kJpegDCAlphabetSize) | value_slots[1] | value_slots[2] | value_slots[3]) == 0;
            if (!only_dc) return Error.InvalidJpegData;
        }
    }

    for (jpg.scan_info.items) |*scan| {
        U32Coder.write(num_components_enc, scan.num_components, writer) catch return Error.NoFeasibleSelector;
        if (scan.num_components >= 4) return Error.InvalidJpegData;
        writer.write(6, scan.Ss);
        writer.write(6, scan.Se);
        writer.write(4, scan.Al);
        writer.write(4, scan.Ah);
        for (0..scan.num_components) |i| {
            if (scan.components[i].comp_idx >= jpg.components.items.len) return Error.InvalidJpegData;
            writer.write(2, scan.components[i].comp_idx);
            writer.write(2, scan.components[i].ac_tbl_idx);
            writer.write(2, scan.components[i].dc_tbl_idx);
        }
        U32Coder.write(last_needed_pass_enc, scan.last_needed_pass, writer) catch return Error.NoFeasibleSelector;
    }

    if (info.has_dri) {
        writer.write(16, jpg.restart_interval);
    }

    for (jpg.scan_info.items) |*scan| {
        const num_reset_points: u32 = @intCast(scan.reset_points.items.len);
        U32Coder.write(count_enc, num_reset_points, writer) catch return Error.NoFeasibleSelector;
        var last_block_idx: i64 = -1;
        for (scan.reset_points.items) |block_idx_in| {
            var block_idx = block_idx_in;
            block_idx -%= @intCast(last_block_idx + 1);
            U32Coder.write(reset_point_enc, block_idx, writer) catch return Error.NoFeasibleSelector;
            block_idx +%= @intCast(last_block_idx + 1);
            if (block_idx >= (3 << 26)) return Error.InvalidJpegData;
            last_block_idx = block_idx;
        }
        const num_extra_zero_runs: u32 = @intCast(scan.extra_zero_runs.items.len);
        U32Coder.write(count_enc, num_extra_zero_runs, writer) catch return Error.NoFeasibleSelector;
        last_block_idx = -1;
        for (scan.extra_zero_runs.items) |*ezr| {
            var extra_zero_runs = ezr.num_extra_zero_runs;
            U32Coder.write(extra_zero_runs_enc, extra_zero_runs, writer) catch return Error.NoFeasibleSelector;
            if (extra_zero_runs > 4) return Error.InvalidJpegData;
            var block_idx = ezr.block_idx;
            block_idx -%= @intCast(last_block_idx + 1);
            U32Coder.write(reset_point_enc, block_idx, writer) catch return Error.NoFeasibleSelector;
            block_idx +%= @intCast(last_block_idx + 1);
            if (block_idx > (3 << 26)) return Error.InvalidJpegData;
            last_block_idx = block_idx;
            _ = &extra_zero_runs;
        }
    }
    for (jpg.inter_marker_data.items) |*imd| {
        const len: u32 = @intCast(imd.items.len);
        writer.write(16, len);
    }
    const tail_data_len: u32 = @intCast(jpg.tail_data.items.len);
    if (tail_data_len > 4260096) return Error.InvalidJpegData;
    U32Coder.write(tail_data_len_enc, tail_data_len, writer) catch return Error.NoFeasibleSelector;

    writer.write(1, @intFromBool(jpg.has_zero_padding_bit));
    if (jpg.has_zero_padding_bit) {
        const nbit: u32 = @intCast(jpg.padding_bits.items.len);
        writer.write(24, nbit);
        for (jpg.padding_bits.items) |bit| {
            writer.write(1, bit);
        }
    }

}

/// Parses the JPEGData fields (port of Bundle::Read); the brotli blob
/// starts at the next byte boundary afterwards (returned position).
pub fn readFields(br: *BitReader, jpg: *JPEGData, allocator: std.mem.Allocator) Error!void {
    const is_gray = (br.readFixedBits(1)) != 0;
    if (is_gray) {
        try jpg.components.resize(allocator, 1);
        jpg.components.items[0] = .{};
    } else {
        try jpg.components.resize(allocator, 3);
        for (jpg.components.items) |*c| c.* = .{};
    }

    var info = JpegInfo{};
    var marker: u8 = 0xc0;
    while (true) {
        visitMarker(&marker, &info);
        const m: u8 = @intCast(br.readFixedBits(6));
        marker = m +% 0xc0;
        try jpg.marker_order.append(allocator, marker);
        if (jpg.marker_order.items.len > 16384) return Error.InvalidJpegData;
        if (marker == 0xd9) break;
    }
    if (info.num_scans == 0) return Error.InvalidJpegData;

    try jpg.app_data.resize(allocator, info.num_app_markers);
    try jpg.app_marker_type.resize(allocator, info.num_app_markers);
    for (jpg.app_marker_type.items) |*t| t.* = .unknown;
    for (jpg.app_data.items) |*app| app.* = .empty;
    try jpg.com_data.resize(allocator, info.num_com_markers);
    for (jpg.com_data.items) |*com| com.* = .empty;
    try jpg.scan_info.resize(allocator, info.num_scans);
    for (jpg.scan_info.items) |*s| s.* = .{};

    for (0..jpg.app_data.items.len) |i| {
        const t = U32Coder.read(app_marker_type_enc, br);
        if (t > 3) return Error.InvalidJpegData;
        jpg.app_marker_type.items[i] = @enumFromInt(t);
        const len = br.readFixedBits(16);
        try jpg.app_data.items[i].resize(allocator, len + 1);
        @memset(jpg.app_data.items[i].items, 0);
        if (jpg.app_data.items[i].items.len < 3) return Error.InvalidJpegData;
    }
    for (0..jpg.com_data.items.len) |i| {
        const len = br.readFixedBits(16);
        try jpg.com_data.items[i].resize(allocator, len + 1);
        @memset(jpg.com_data.items[i].items, 0);
        if (jpg.com_data.items[i].items.len < 3) return Error.InvalidJpegData;
    }

    const num_quant_tables = U32Coder.read(num_quant_tables_enc, br);
    if (num_quant_tables == 4 or num_quant_tables == 0) return Error.InvalidJpegData;
    try jpg.quant.resize(allocator, num_quant_tables);
    for (jpg.quant.items) |*q| q.* = .{};
    for (jpg.quant.items) |*q| {
        q.precision = @intCast(br.readFixedBits(1));
        if (q.precision > 1) return Error.InvalidJpegData;
        q.index = @intCast(br.readFixedBits(2));
        q.is_last = (br.readFixedBits(1)) != 0;
    }

    const component_type_raw: u8 = @intCast(br.readFixedBits(2));
    const component_type: JPEGComponentType = @enumFromInt(component_type_raw);
    var num_components: usize = 0;
    switch (component_type) {
        .gray => num_components = 1,
        .ycbcr, .rgb => num_components = 3,
        .custom => {
            const n = U32Coder.read(num_components_enc, br);
            if (n != 1 and n != 3) return Error.InvalidJpegData;
            num_components = n;
        },
    }
    if (jpg.components.items.len != num_components) return Error.InvalidJpegData;
    switch (component_type) {
        .custom => {
            for (jpg.components.items) |*c| {
                c.id = @intCast(br.readFixedBits(8));
            }
        },
        .gray => jpg.components.items[0].id = 1,
        .rgb => {
            jpg.components.items[0].id = 'R';
            jpg.components.items[1].id = 'G';
            jpg.components.items[2].id = 'B';
        },
        .ycbcr => {
            jpg.components.items[0].id = 1;
            jpg.components.items[1].id = 2;
            jpg.components.items[2].id = 3;
        },
    }
    var used_tables: u32 = 0;
    for (jpg.components.items) |*c| {
        c.quant_idx = @intCast(br.readFixedBits(2));
        if (c.quant_idx >= jpg.quant.items.len) return Error.InvalidJpegData;
        used_tables |= @as(u32, 1) << @intCast(c.quant_idx);
    }
    for (jpg.quant.items, 0..) |*q, i| {
        if (used_tables & (@as(u32, 1) << @intCast(i)) != 0) continue;
        if (i == 0) return Error.InvalidJpegData;
        if (!std.mem.eql(i32, &q.values, &jpg.quant.items[i - 1].values)) {
            return Error.InvalidJpegData;
        }
    }

    const num_huff = U32Coder.read(num_huff_enc, br);
    try jpg.huffman_code.resize(allocator, num_huff);
    for (jpg.huffman_code.items) |*hc| hc.* = .{};
    for (jpg.huffman_code.items) |*hc| {
        const is_ac = (br.readFixedBits(1)) != 0;
        const id = br.readFixedBits(2);
        hc.slot_id = @intCast((@as(u32, @intFromBool(is_ac)) << 4) | id);
        hc.is_last = (br.readFixedBits(1)) != 0;
        var num_symbols: usize = 0;
        for (0..jd.kJpegHuffmanMaxBitLength + 1) |i| {
            hc.counts[i] = U32Coder.read(huff_count_enc, br);
            num_symbols += hc.counts[i];
        }
        if (num_symbols == 0) continue;
        if (num_symbols > hc.values.len) return Error.InvalidJpegData;
        var value_slots = [_]u64{0} ** 5;
        for (0..num_symbols) |i| {
            hc.values[i] = U32Coder.read(huff_value_enc, br);
            value_slots[hc.values[i] >> 6] |= @as(u64, 1) << @intCast(hc.values[i] & 0x3F);
        }
        if (hc.values[num_symbols - 1] != jd.kJpegHuffmanAlphabetSize) return Error.InvalidJpegData;
        var num_values: usize = 1;
        for (0..4) |i| num_values += @popCount(value_slots[i]);
        if (num_values != num_symbols) return Error.InvalidJpegData;
        if (!is_ac) {
            const only_dc = ((value_slots[0] >> jd.kJpegDCAlphabetSize) | value_slots[1] | value_slots[2] | value_slots[3]) == 0;
            if (!only_dc) return Error.InvalidJpegData;
        }
    }

    for (jpg.scan_info.items) |*scan| {
        scan.num_components = U32Coder.read(num_components_enc, br);
        if (scan.num_components >= 4) return Error.InvalidJpegData;
        scan.Ss = @intCast(br.readFixedBits(6));
        scan.Se = @intCast(br.readFixedBits(6));
        scan.Al = @intCast(br.readFixedBits(4));
        scan.Ah = @intCast(br.readFixedBits(4));
        for (0..scan.num_components) |i| {
            scan.components[i].comp_idx = @intCast(br.readFixedBits(2));
            if (scan.components[i].comp_idx >= jpg.components.items.len) return Error.InvalidJpegData;
            scan.components[i].ac_tbl_idx = @intCast(br.readFixedBits(2));
            scan.components[i].dc_tbl_idx = @intCast(br.readFixedBits(2));
        }
        scan.last_needed_pass = U32Coder.read(last_needed_pass_enc, br);
    }

    if (info.has_dri) {
        jpg.restart_interval = @intCast(br.readFixedBits(16));
    }

    for (jpg.scan_info.items) |*scan| {
        const num_reset_points = U32Coder.read(count_enc, br);
        try scan.reset_points.resize(allocator, num_reset_points);
        var last_block_idx: i64 = -1;
        for (scan.reset_points.items) |*block_idx_p| {
            var block_idx = U32Coder.read(reset_point_enc, br);
            block_idx +%= @intCast(last_block_idx + 1);
            if (block_idx >= (3 << 26)) return Error.InvalidJpegData;
            block_idx_p.* = block_idx;
            last_block_idx = block_idx;
        }
        const num_extra_zero_runs = U32Coder.read(count_enc, br);
        try scan.extra_zero_runs.resize(allocator, num_extra_zero_runs);
        for (scan.extra_zero_runs.items) |*ezr| {
            ezr.* = .{ .block_idx = 0, .num_extra_zero_runs = 0 };
            ezr.num_extra_zero_runs = U32Coder.read(extra_zero_runs_enc, br);
            if (ezr.num_extra_zero_runs > 4 or ezr.num_extra_zero_runs < 1) return Error.InvalidJpegData;
            var block_idx = U32Coder.read(reset_point_enc, br);
            block_idx +%= @intCast(last_block_idx + 1);
            if (block_idx > (3 << 26)) return Error.InvalidJpegData;
            ezr.block_idx = block_idx;
            last_block_idx = block_idx;
        }
    }
    var inter_marker_sizes: std.ArrayList(u32) = .empty;
    defer inter_marker_sizes.deinit(allocator);
    for (0..info.num_intermarker) |_| {
        try inter_marker_sizes.append(allocator, @intCast(br.readFixedBits(16)));
    }
    const tail_data_len = U32Coder.read(tail_data_len_enc, br);
    jpg.has_zero_padding_bit = (br.readFixedBits(1)) != 0;
    if (jpg.has_zero_padding_bit) {
        const nbit = br.readFixedBits(24);
        for (0..nbit) |_| {
            try jpg.padding_bits.append(allocator, @intCast(br.readFixedBits(1)));
        }
    }

    // Validation pass: Huffman tables must be defined before use.
    {
        var dht_index: usize = 0;
        var scan_index: usize = 0;
        var is_progressive = false;
        var ac_ok = [_]bool{false} ** jd.kMaxHuffmanTables;
        var dc_ok = [_]bool{false} ** jd.kMaxHuffmanTables;
        for (jpg.marker_order.items) |m| {
            if (m == 0xC2) {
                is_progressive = true;
            } else if (m == 0xC4) {
                while (dht_index < jpg.huffman_code.items.len) {
                    const huff = &jpg.huffman_code.items[dht_index];
                    dht_index += 1;
                    var index: usize = @intCast(huff.slot_id);
                    if (index & 0x10 != 0) {
                        index -= 0x10;
                        ac_ok[index] = true;
                    } else {
                        dc_ok[index] = true;
                    }
                    if (huff.is_last) break;
                }
            } else if (m == 0xDA) {
                const si = &jpg.scan_info.items[scan_index];
                scan_index += 1;
                for (0..si.num_components) |i| {
                    const csi = &si.components[i];
                    const want_dc = !is_progressive or (si.Ss == 0);
                    if (want_dc and !dc_ok[csi.dc_tbl_idx]) return Error.InvalidJpegData;
                    const want_ac = !is_progressive or (si.Ss != 0) or (si.Se != 0);
                    if (want_ac and !ac_ok[csi.ac_tbl_idx]) return Error.InvalidJpegData;
                }
            }
        }
    }

    // Postponed actions.
    try jpg.tail_data.resize(allocator, tail_data_len);
    @memset(jpg.tail_data.items, 0);
    try jpg.inter_marker_data.resize(allocator, info.num_intermarker);
    for (jpg.inter_marker_data.items, 0..) |*imd, i| {
        imd.* = .empty;
        try imd.resize(allocator, inter_marker_sizes.items[i]);
        @memset(imd.items, 0);
    }
}

/// Assembles the full `jbrd` box payload: serialized fields followed by
/// the byte-aligned brotli blob carrying the raw APP/COM/inter-marker/
/// tail data (port of EncodeJPEGData, with real Brotli compression when
/// `options.brotli_quality` > 0 and raw meta-blocks at 0).
pub fn encode(allocator: std.mem.Allocator, jpg: *JPEGData, options: Options) Error![]u8 {
    const old_len = jpg.app_marker_type.items.len;
    try jpg.app_marker_type.resize(allocator, jpg.app_data.items.len);
    for (jpg.app_marker_type.items[old_len..]) |*t| {
        t.* = .unknown;
    }

    var blob: std.ArrayList(u8) = .empty;
    defer blob.deinit(allocator);
    for (jpg.app_data.items, 0..) |*app, i| {
        if (jpg.app_marker_type.items[i] != .unknown) continue;
        try blob.appendSlice(allocator, app.items);
    }
    for (jpg.com_data.items) |*com| {
        try blob.appendSlice(allocator, com.items);
    }
    for (jpg.inter_marker_data.items) |*imd| {
        try blob.appendSlice(allocator, imd.items);
    }
    try blob.appendSlice(allocator, jpg.tail_data.items);

    var out: std.ArrayList(u8) = .empty;
    errdefer out.deinit(allocator);
    var w = BitWriter.init(allocator);
    defer w.deinit();
    try writeFields(jpg, &w, allocator);
    w.zeroPadToByte();
    try out.appendSlice(allocator, w.getSpan());
    if (options.brotli_quality == 0) {
        try brotli.writeRaw(allocator, blob.items, &out);
    } else {
        const compressed = brotli.compress(allocator, blob.items, .{ .quality = options.brotli_quality }) catch |e| switch (e) {
            error.OutOfMemory => return Error.OutOfMemory,
            else => return Error.BrotliCorrupt,
        };
        defer allocator.free(compressed);
        try out.appendSlice(allocator, compressed);
    }
    return out.toOwnedSlice(allocator);
}

/// Parses a full `jbrd` box payload (port of DecodeJPEGData; the trailing
/// blob is a full Brotli stream — libjxl/cjxl-produced boxes and this
/// codec's output are both accepted); fills the raw marker bytes from the
/// blob.
pub fn decode(allocator: std.mem.Allocator, bytes: []const u8, jpg: *JPEGData) Error!void {
    var br = BitReader.init(bytes);
    try readFields(&br, jpg, allocator);
    br.jumpToByteBoundary() catch return Error.InvalidJpegData;
    const consumed = br.totalBitsConsumed() / 8;
    br.close() catch return Error.InvalidJpegData;
    if (consumed > bytes.len) return Error.InvalidJpegData;

    var blob: std.ArrayList(u8) = .empty;
    defer blob.deinit(allocator);
    const raw = brotli.decompress(allocator, bytes[consumed..]) catch |e| switch (e) {
        error.BrotliCorrupt => return Error.InvalidJpegData,
        error.OutOfMemory => return Error.OutOfMemory,
    };
    defer allocator.free(raw);
    try blob.appendSlice(allocator, raw);

    var pos: usize = 0;
    var num_icc: usize = 0;
    for (jpg.app_data.items, 0..) |*marker, i| {
        if (marker.items.len < 3) return Error.InvalidJpegData;
        if (jpg.app_marker_type.items[i] != .unknown) {
            // Rebuilt from the type; ICC content comes from the codestream.
            const size_minus_1 = marker.items.len - 1;
            marker.items[1] = @intCast(size_minus_1 >> 8);
            marker.items[2] = @truncate(size_minus_1);
            if (jpg.app_marker_type.items[i] == .icc) {
                if (marker.items.len < 17) return Error.InvalidJpegData;
                marker.items[0] = 0xE2;
                @memcpy(marker.items[3..][0..kIccProfileTag.len], kIccProfileTag);
                num_icc += 1;
                marker.items[15] = @intCast(num_icc);
            }
        } else {
            if (pos + marker.items.len > blob.items.len) return Error.InvalidJpegData;
            @memcpy(marker.items, blob.items[pos .. pos + marker.items.len]);
            pos += marker.items.len;
            const size: usize = (@as(usize, marker.items[1]) << 8) | marker.items[2];
            if (size + 1 != marker.items.len) return Error.InvalidJpegData;
        }
    }
    for (jpg.app_data.items, 0..) |*marker, i| {
        if (jpg.app_marker_type.items[i] == .icc) {
            marker.items[16] = @intCast(num_icc);
        }
        if (jpg.app_marker_type.items[i] == .exif) {
            if (marker.items.len < 3 + kExifTag.len) return Error.InvalidJpegData;
            marker.items[0] = 0xE1;
            @memcpy(marker.items[3..][0..kExifTag.len], kExifTag);
        }
        if (jpg.app_marker_type.items[i] == .xmp) {
            if (marker.items.len < 3 + kXMPTag.len) return Error.InvalidJpegData;
            marker.items[0] = 0xE1;
            @memcpy(marker.items[3..][0..kXMPTag.len], kXMPTag);
        }
    }
    for (jpg.com_data.items) |*marker| {
        if (pos + marker.items.len > blob.items.len) return Error.InvalidJpegData;
        @memcpy(marker.items, blob.items[pos .. pos + marker.items.len]);
        pos += marker.items.len;
        const size: usize = (@as(usize, marker.items[1]) << 8) | marker.items[2];
        if (size + 1 != marker.items.len) return Error.InvalidJpegData;
    }
    for (jpg.inter_marker_data.items) |*imd| {
        if (pos + imd.items.len > blob.items.len) return Error.InvalidJpegData;
        @memcpy(imd.items, blob.items[pos .. pos + imd.items.len]);
        pos += imd.items.len;
    }
    if (pos + jpg.tail_data.items.len > blob.items.len) return Error.InvalidJpegData;
    @memcpy(jpg.tail_data.items, blob.items[pos .. pos + jpg.tail_data.items.len]);
    pos += jpg.tail_data.items.len;
    if (pos != blob.items.len) return Error.InvalidJpegData;
}

test "jbrd fields roundtrip" {
    const alloc = std.testing.allocator;
    const reader = @import("reader.zig");
    const writer = @import("writer.zig");
    const data = @embedFile("testdata/baseline_420.jpg");
    var jpg = JPEGData{};
    defer jpg.deinit(alloc);
    try reader.readJpeg(data, .read_all, &jpg, alloc);

    const encoded = try encode(alloc, &jpg, .{});
    defer alloc.free(encoded);

    var jpg2 = JPEGData{};
    defer jpg2.deinit(alloc);
    try decode(alloc, encoded, &jpg2);

    // Dimensions, sampling factors, coefficients and quant values come from
    // the codestream, not from jbrd; simulate that restoration here.
    jpg2.width = jpg.width;
    jpg2.height = jpg.height;
    for (jpg.components.items, 0..) |*src, i| {
        const dst = &jpg2.components.items[i];
        dst.h_samp_factor = src.h_samp_factor;
        dst.v_samp_factor = src.v_samp_factor;
        dst.width_in_blocks = src.width_in_blocks;
        dst.height_in_blocks = src.height_in_blocks;
        try dst.coeffs.appendSlice(alloc, src.coeffs.items);
    }
    for (jpg.quant.items, 0..) |*src, i| {
        jpg2.quant.items[i].values = src.values;
    }

    // Re-encoding must reproduce the exact same jbrd payload.
    const encoded2 = try encode(alloc, &jpg2, .{});
    defer alloc.free(encoded2);
    try std.testing.expectEqualSlices(u8, encoded, encoded2);

    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(alloc);
    try writer.writeJpeg(&jpg2, &out, alloc);
    try std.testing.expectEqualSlices(u8, data, out.items);
}

test "jbrd fields roundtrip progressive" {
    const alloc = std.testing.allocator;
    const reader = @import("reader.zig");
    const writer = @import("writer.zig");
    const data = @embedFile("testdata/progressive_444.jpg");
    var jpg = JPEGData{};
    defer jpg.deinit(alloc);
    try reader.readJpeg(data, .read_all, &jpg, alloc);

    const encoded = try encode(alloc, &jpg, .{});
    defer alloc.free(encoded);

    var jpg2 = JPEGData{};
    defer jpg2.deinit(alloc);
    try decode(alloc, encoded, &jpg2);

    // Restore codestream-provided fields.
    jpg2.width = jpg.width;
    jpg2.height = jpg.height;
    for (jpg.components.items, 0..) |*src, i| {
        const dst = &jpg2.components.items[i];
        dst.h_samp_factor = src.h_samp_factor;
        dst.v_samp_factor = src.v_samp_factor;
        dst.width_in_blocks = src.width_in_blocks;
        dst.height_in_blocks = src.height_in_blocks;
        try dst.coeffs.appendSlice(alloc, src.coeffs.items);
    }
    for (jpg.quant.items, 0..) |*src, i| {
        jpg2.quant.items[i].values = src.values;
    }

    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(alloc);
    try writer.writeJpeg(&jpg2, &out, alloc);
    try std.testing.expectEqualSlices(u8, data, out.items);
}
