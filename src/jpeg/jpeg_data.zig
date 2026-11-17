//! Non-pixel content of a JPEG file, port of libjxl `jpeg/jpeg_data.h`.

const std = @import("std");

pub const kMaxComponents = 4;
pub const kMaxQuantTables = 4;
pub const kMaxHuffmanTables = 4;
pub const kJpegHuffmanMaxBitLength = 16;
pub const kJpegHuffmanAlphabetSize = 256;
pub const kJpegDCAlphabetSize = 12;
pub const kMaxDHTMarkers = 512;
pub const kMaxDimPixels = 65535;
pub const kDCTBlockSize = 64;
pub const kApp1: u8 = 0xE1;
pub const kApp2: u8 = 0xE2;
pub const kIccProfileTag = "ICC_PROFILE\x00";
pub const kExifTag = "Exif\x00\x00";
pub const kXMPTag = "http://ns.adobe.com/xap/1.0/\x00";
pub const kBrunsliMaxSampling = 15;

/// zig-zag index -> natural (row-major) index; 64 entries plus 16 guard
/// entries mirroring libjxl's decoder-safety padding.
pub const kJPEGNaturalOrder = [80]u32{
    0,  1,  8,  16, 9,  2,  3,  10,
    17, 24, 32, 25, 18, 11, 4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13, 6,  7,  14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
    63, 63, 63, 63, 63, 63, 63, 63,
    63, 63, 63, 63, 63, 63, 63, 63,
};

/// natural index -> zig-zag index.
pub const kJPEGZigZagOrder = [64]u32{
    0,  1,  5,  6,  14, 15, 27, 28,
    2,  4,  7,  13, 16, 26, 29, 42,
    3,  8,  12, 17, 25, 30, 41, 43,
    9,  11, 18, 24, 31, 40, 44, 53,
    10, 19, 23, 32, 39, 45, 52, 54,
    20, 22, 33, 38, 46, 51, 55, 60,
    21, 34, 37, 47, 50, 56, 59, 61,
    35, 36, 48, 49, 57, 58, 62, 63,
};

pub const coeff_t = i16;

pub const JPEGQuantTable = struct {
    values: [kDCTBlockSize]i32 = @splat(0),
    precision: u32 = 0,
    /// DQT 'index' field as parsed from the input JPEG (0..3).
    index: u32 = 0,
    /// True when this table is the last one within its DQT segment.
    is_last: bool = true,
};

pub const JPEGHuffmanCode = struct {
    /// Bit length histogram; counts[0] is always 0.
    counts: [kJpegHuffmanMaxBitLength + 1]u32 = @splat(0),
    /// Symbol values sorted by increasing bit length; the last entry is the
    /// synthetic all-ones EOI symbol (kJpegHuffmanAlphabetSize).
    values: [kJpegHuffmanAlphabetSize + 1]u32 = @splat(0),
    /// 0x00-0x03 DC slots, 0x10-0x13 AC slots.
    slot_id: i32 = 0,
    is_last: bool = true,
};

pub const JPEGComponentScanInfo = struct {
    comp_idx: u32 = 0,
    dc_tbl_idx: u32 = 0,
    ac_tbl_idx: u32 = 0,
};

pub const ExtraZeroRunInfo = struct {
    block_idx: u32,
    num_extra_zero_runs: u32,
};

pub const JPEGScanInfo = struct {
    Ss: u32 = 0,
    Se: u32 = 0,
    Ah: u32 = 0,
    Al: u32 = 0,
    num_components: u32 = 0,
    components: [kMaxComponents]JPEGComponentScanInfo = @splat(.{}),
    last_needed_pass: u32 = 0,
    /// Block indexes (in scan order) where EOB runs / refinement bits must be
    /// flushed for bit-exact reconstruction.
    reset_points: std.ArrayList(u32) = .empty,
    extra_zero_runs: std.ArrayList(ExtraZeroRunInfo) = .empty,

    pub fn deinit(self: *JPEGScanInfo, allocator: std.mem.Allocator) void {
        self.reset_points.deinit(allocator);
        self.extra_zero_runs.deinit(allocator);
    }
};

pub const JPEGComponent = struct {
    id: u32 = 0,
    h_samp_factor: i32 = 1,
    v_samp_factor: i32 = 1,
    quant_idx: u32 = 0,
    width_in_blocks: u32 = 0,
    height_in_blocks: u32 = 0,
    /// Per-block DCT coefficients (quantized), block-major layout.
    coeffs: std.ArrayList(coeff_t) = .empty,

    pub fn deinit(self: *JPEGComponent, allocator: std.mem.Allocator) void {
        self.coeffs.deinit(allocator);
    }
};

pub const AppMarkerType = enum(u32) {
    unknown = 0,
    icc = 1,
    exif = 2,
    xmp = 3,
};

pub const JpegReadMode = enum {
    read_headers,
    read_tables,
    read_all,
};

pub const JPEGData = struct {
    width: i32 = 0,
    height: i32 = 0,
    restart_interval: u32 = 0,
    app_data: std.ArrayList(std.ArrayList(u8)) = .empty,
    app_marker_type: std.ArrayList(AppMarkerType) = .empty,
    com_data: std.ArrayList(std.ArrayList(u8)) = .empty,
    quant: std.ArrayList(JPEGQuantTable) = .empty,
    huffman_code: std.ArrayList(JPEGHuffmanCode) = .empty,
    components: std.ArrayList(JPEGComponent) = .empty,
    scan_info: std.ArrayList(JPEGScanInfo) = .empty,
    /// Marker sequence 0xC0..0xD9; 0xFF is the synthetic inter-marker entry.
    marker_order: std.ArrayList(u8) = .empty,
    inter_marker_data: std.ArrayList(std.ArrayList(u8)) = .empty,
    tail_data: std.ArrayList(u8) = .empty,

    has_zero_padding_bit: bool = false,
    padding_bits: std.ArrayList(u8) = .empty,

    pub fn deinit(self: *JPEGData, allocator: std.mem.Allocator) void {
        for (self.app_data.items) |*m| m.deinit(allocator);
        self.app_data.deinit(allocator);
        self.app_marker_type.deinit(allocator);
        for (self.com_data.items) |*m| m.deinit(allocator);
        self.com_data.deinit(allocator);
        self.quant.deinit(allocator);
        self.huffman_code.deinit(allocator);
        for (self.components.items) |*c| c.deinit(allocator);
        self.components.deinit(allocator);
        for (self.scan_info.items) |*s| s.deinit(allocator);
        self.scan_info.deinit(allocator);
        self.marker_order.deinit(allocator);
        for (self.inter_marker_data.items) |*m| m.deinit(allocator);
        self.inter_marker_data.deinit(allocator);
        self.tail_data.deinit(allocator);
        self.padding_bits.deinit(allocator);
        self.* = .{};
    }

    /// Port of JPEGData::CalculateMcuSize.
    pub fn calculateMcuSize(self: *const JPEGData, scan: *const JPEGScanInfo, mcus_per_row: *i32, mcu_rows: *i32) void {
        const is_interleaved = scan.num_components > 1;
        const base_component = self.components.items[scan.components[0].comp_idx];
        const h_group: i32 = if (is_interleaved) 1 else base_component.h_samp_factor;
        const v_group: i32 = if (is_interleaved) 1 else base_component.v_samp_factor;
        var max_h_samp_factor: i32 = 1;
        var max_v_samp_factor: i32 = 1;
        for (self.components.items) |c| {
            max_h_samp_factor = @max(c.h_samp_factor, max_h_samp_factor);
            max_v_samp_factor = @max(c.v_samp_factor, max_v_samp_factor);
        }
        const w: u64 = @intCast(self.width);
        const h: u64 = @intCast(self.height);
        mcus_per_row.* = @intCast(std.math.divCeil(u64, w * @as(u64, @intCast(h_group)), @as(u64, @intCast(8 * max_h_samp_factor))) catch 0);
        mcu_rows.* = @intCast(std.math.divCeil(u64, h * @as(u64, @intCast(v_group)), @as(u64, @intCast(8 * max_v_samp_factor))) catch 0);
    }
};

test "zigzag tables are inverse permutations" {
    for (0..64) |natural| {
        const zz = kJPEGZigZagOrder[natural];
        try std.testing.expectEqual(@as(u32, @intCast(natural)), kJPEGNaturalOrder[zz]);
    }
}
