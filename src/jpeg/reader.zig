//! JPEG bitstream parser, port of libjxl `jpeg/enc_jpeg_data_reader.cc`.
//! Extracts marker structure, tables and DCT coefficients; records the
//! extra information needed for bit-exact reconstruction (padding bits,
//! reset points, extra zero runs).

const std = @import("std");
const jd = @import("jpeg_data.zig");

const JPEGData = jd.JPEGData;
const JPEGComponent = jd.JPEGComponent;
const JPEGHuffmanCode = jd.JPEGHuffmanCode;
const JPEGQuantTable = jd.JPEGQuantTable;
const JPEGScanInfo = jd.JPEGScanInfo;
const coeff_t = jd.coeff_t;

pub const Error = error{
    InvalidJpeg,
    UnexpectedEndOfInput,
    UnsupportedMarker,
    OutOfMemory,
};

const HuffmanTableEntry = struct {
    bits: u8 = 0,
    value: u16 = 0xffff,
};

const kJpegHuffmanRootTableBits = 8;
const kJpegHuffmanLutSize = 1024;

fn nextTableBitSize(count: *const [jd.kJpegHuffmanMaxBitLength + 1]i32, len_in: i32) i32 {
    var len = len_in;
    var left: i32 = @as(i32, 1) << @intCast(len - kJpegHuffmanRootTableBits);
    while (len < jd.kJpegHuffmanMaxBitLength) {
        left -= count[@intCast(len)];
        if (left <= 0) break;
        len += 1;
        left <<= 1;
    }
    return len - kJpegHuffmanRootTableBits;
}

/// Builds the two-level jpeg decoding LUT (root 8 bits + sub-tables).
fn buildJpegHuffmanTable(counts: *const [jd.kJpegHuffmanMaxBitLength + 1]u32, symbols: *const [jd.kJpegHuffmanAlphabetSize + 1]u32, lut: []HuffmanTableEntry) void {
    var tmp_count: [jd.kJpegHuffmanMaxBitLength + 1]i32 = @splat(0);
    var total_count: i32 = 0;
    for (1..jd.kJpegHuffmanMaxBitLength + 1) |len| {
        tmp_count[len] = @intCast(counts[len]);
        total_count += tmp_count[len];
    }

    var table_idx: usize = 0;
    var table_bits: u5 = kJpegHuffmanRootTableBits;
    var table_size: usize = 1 << kJpegHuffmanRootTableBits;

    if (total_count == 1) {
        var key: usize = 0;
        while (key < table_size) : (key += 1) {
            lut[key] = .{ .bits = 0, .value = @intCast(symbols[0]) };
        }
        return;
    }

    var key: usize = 0;
    var idx: usize = 0;
    // Root table.
    for (1..kJpegHuffmanRootTableBits + 1) |len| {
        while (tmp_count[len] > 0) : (tmp_count[len] -= 1) {
            const reps: usize = @intCast(@as(i32, 1) << @intCast(kJpegHuffmanRootTableBits - @as(i32, @intCast(len))));
            var r: usize = 0;
            while (r < reps) : (r += 1) {
                lut[key + r] = .{ .bits = @intCast(len), .value = @intCast(symbols[idx]) };
            }
            key += reps;
            idx += 1;
        }
    }
    // Second-level tables.
    table_idx += table_size;
    table_size = 0;
    var low: usize = 0;
    for (kJpegHuffmanRootTableBits + 1..jd.kJpegHuffmanMaxBitLength + 1) |len| {
        while (tmp_count[len] > 0) : (tmp_count[len] -= 1) {
            if (low >= table_size) {
                table_idx += table_size;
                table_bits = @intCast(nextTableBitSize(&tmp_count, @intCast(len)));
                table_size = @as(usize, 1) << table_bits;
                low = 0;
                lut[key] = .{
                    .bits = table_bits + kJpegHuffmanRootTableBits,
                    .value = @intCast(table_idx - key),
                };
                key += 1;
            }
            const code_bits: u5 = @intCast(len - kJpegHuffmanRootTableBits);
            const reps: usize = @as(usize, 1) << @intCast(table_bits - code_bits);
            var r: usize = 0;
            while (r < reps) : (r += 1) {
                lut[table_idx + low + r] = .{ .bits = code_bits, .value = @intCast(symbols[idx]) };
            }
            low += reps;
            idx += 1;
        }
    }
}

/// Entropy-coded-data bit reader; skips 0xff/0x00 escapes and stops
/// (virtually) at the next marker.
const BitReaderState = struct {
    data: []const u8,
    pos: usize,
    val: u64 = 0,
    bits_left: i32 = 0,
    next_marker_pos: usize,

    fn init(data: []const u8, pos: usize) BitReaderState {
        var br = BitReaderState{
            .data = data,
            .pos = pos,
            .next_marker_pos = data.len - 2,
        };
        br.fillBitWindow();
        return br;
    }

    fn reset(self: *BitReaderState, pos: usize) void {
        self.pos = pos;
        self.val = 0;
        self.bits_left = 0;
        self.next_marker_pos = self.data.len - 2;
        self.fillBitWindow();
    }

    fn getNextByte(self: *BitReaderState) u8 {
        if (self.pos >= self.next_marker_pos) {
            self.pos += 1;
            return 0;
        }
        const c = self.data[self.pos];
        self.pos += 1;
        if (c == 0xff) {
            const escape = self.data[self.pos];
            if (escape == 0) {
                self.pos += 1;
            } else {
                self.next_marker_pos = self.pos - 1;
            }
        }
        return c;
    }

    fn fillBitWindow(self: *BitReaderState) void {
        if (self.bits_left <= 16) {
            while (self.bits_left <= 56) {
                self.val <<= 8;
                self.val |= self.getNextByte();
                self.bits_left += 8;
            }
        }
    }

    fn readBits(self: *BitReaderState, nbits: u5) u32 {
        self.fillBitWindow();
        const v: u64 = (self.val >> @intCast(self.bits_left - @as(i32, nbits))) & ((@as(u64, 1) << nbits) - 1);
        self.bits_left -= nbits;
        return @intCast(v);
    }

    /// Position where parsing should continue; enqueues padding bits.
    fn finishStream(self: *BitReaderState, allocator: std.mem.Allocator, jpg: *JPEGData, pos: *usize) Error!void {
        const npadbits: i32 = self.bits_left & 7;
        if (npadbits > 0) {
            const padmask: u64 = (@as(u64, 1) << @intCast(npadbits)) - 1;
            const padbits: u64 = (self.val >> @intCast(self.bits_left - npadbits)) & padmask;
            if (padbits != padmask) {
                jpg.has_zero_padding_bit = true;
            }
            var i: i32 = npadbits - 1;
            while (i >= 0) : (i -= 1) {
                try jpg.padding_bits.append(allocator, @intCast((padbits >> @intCast(i)) & 1));
            }
        }
        var unused_bytes_left: i32 = self.bits_left >> 3;
        while (unused_bytes_left > 0) : (unused_bytes_left -= 1) {
            self.pos -= 1;
            if (self.pos < self.next_marker_pos and self.data[self.pos] == 0 and
                self.data[self.pos - 1] == 0xff)
            {
                self.pos -= 1;
            }
        }
        if (self.pos > self.next_marker_pos) {
            return Error.UnexpectedEndOfInput;
        }
        pos.* = self.pos;
    }
};

fn readSymbol(table: []const HuffmanTableEntry, br: *BitReaderState) u16 {
    br.fillBitWindow();
    var idx: usize = @intCast((br.val >> @intCast(br.bits_left - 8)) & 0xff);
    var entry = table[idx];
    const nbits: i32 = @as(i32, entry.bits) - 8;
    if (nbits > 0) {
        br.bits_left -= 8;
        idx += entry.value;
        const val2: u64 = (br.val >> @intCast(br.bits_left - nbits)) & ((@as(u64, 1) << @intCast(nbits)) - 1);
        idx += @intCast(val2);
        entry = table[idx];
    }
    br.bits_left -= entry.bits;
    return entry.value;
}

/// DC diff / AC value for extra bits `x` and prefix code `s` (F.1.2.1.1).
fn huffExtend(x: u32, s: u5) i32 {
    const half: i32 = @as(i32, 1) << @intCast(s - 1);
    const xi: i32 = @intCast(x);
    if (xi >= half) return xi;
    return xi - (@as(i32, 1) << s) + 1;
}

fn processSOF(data: []const u8, pos: *usize, jpg: *JPEGData, allocator: std.mem.Allocator) Error!void {
    if (jpg.width != 0) return Error.InvalidJpeg; // duplicate SOF
    const start_pos = pos.*;
    if (pos.* + 8 > data.len) return Error.UnexpectedEndOfInput;
    const marker_len: usize = readU16(data, pos);
    const precision = readU8(data, pos);
    const height = readU16(data, pos);
    const width = readU16(data, pos);
    const num_components = readU8(data, pos);
    if (precision != 8) return Error.InvalidJpeg;
    if (height < 1 or height > jd.kMaxDimPixels) return Error.InvalidJpeg;
    if (width < 1 or width > jd.kMaxDimPixels) return Error.InvalidJpeg;
    if (num_components < 1 or num_components > jd.kMaxComponents) return Error.InvalidJpeg;
    if (pos.* + 3 * num_components > data.len) return Error.UnexpectedEndOfInput;
    jpg.height = height;
    jpg.width = width;
    try jpg.components.resize(allocator, num_components);
    for (jpg.components.items) |*c| c.* = .{};

    var ids_seen = [_]bool{false} ** 256;
    var max_h_samp_factor: i32 = 1;
    var max_v_samp_factor: i32 = 1;
    for (jpg.components.items) |*component| {
        const id = readU8(data, pos);
        if (ids_seen[id]) return Error.InvalidJpeg;
        ids_seen[id] = true;
        component.id = id;
        const factor = readU8(data, pos);
        const h_samp_factor: i32 = factor >> 4;
        const v_samp_factor: i32 = factor & 0xf;
        if (h_samp_factor < 1 or h_samp_factor > jd.kBrunsliMaxSampling) return Error.InvalidJpeg;
        if (v_samp_factor < 1 or v_samp_factor > jd.kBrunsliMaxSampling) return Error.InvalidJpeg;
        component.h_samp_factor = h_samp_factor;
        component.v_samp_factor = v_samp_factor;
        component.quant_idx = readU8(data, pos);
        max_h_samp_factor = @max(max_h_samp_factor, h_samp_factor);
        max_v_samp_factor = @max(max_v_samp_factor, v_samp_factor);
    }

    const mcu_rows: i32 = @intCast(std.math.divCeil(u32, @as(u32, @intCast(jpg.height)), @as(u32, @intCast(max_v_samp_factor * 8))) catch return Error.InvalidJpeg);
    const mcu_cols: i32 = @intCast(std.math.divCeil(u32, @as(u32, @intCast(jpg.width)), @as(u32, @intCast(max_h_samp_factor * 8))) catch return Error.InvalidJpeg);
    for (jpg.components.items) |*c| {
        if (@rem(max_h_samp_factor, c.h_samp_factor) != 0 or
            @rem(max_v_samp_factor, c.v_samp_factor) != 0)
        {
            return Error.InvalidJpeg; // non-integral subsampling ratios
        }
        c.width_in_blocks = @intCast(mcu_cols * c.h_samp_factor);
        c.height_in_blocks = @intCast(mcu_rows * c.v_samp_factor);
    }
    if (start_pos + marker_len != pos.*) return Error.InvalidJpeg;
}

fn processSOS(data: []const u8, pos: *usize, jpg: *JPEGData, allocator: std.mem.Allocator) Error!void {
    const start_pos = pos.*;
    if (pos.* + 3 > data.len) return Error.UnexpectedEndOfInput;
    const marker_len: usize = readU16(data, pos);
    const comps_in_scan = readU8(data, pos);
    if (comps_in_scan < 1 or comps_in_scan > jpg.components.items.len) return Error.InvalidJpeg;

    var scan_info = JPEGScanInfo{};
    scan_info.num_components = comps_in_scan;
    if (pos.* + 2 * comps_in_scan > data.len) return Error.UnexpectedEndOfInput;
    var ids_seen = [_]bool{false} ** 256;
    for (0..comps_in_scan) |i| {
        const id = readU8(data, pos);
        if (ids_seen[id]) return Error.InvalidJpeg;
        ids_seen[id] = true;
        var found_index = false;
        for (jpg.components.items, 0..) |*component, j| {
            if (component.id == id) {
                scan_info.components[i].comp_idx = @intCast(j);
                found_index = true;
            }
        }
        if (!found_index) return Error.InvalidJpeg;
        const c = readU8(data, pos);
        const dc_tbl_idx: u32 = c >> 4;
        const ac_tbl_idx: u32 = c & 0xf;
        if (dc_tbl_idx > 3 or ac_tbl_idx > 3) return Error.InvalidJpeg;
        scan_info.components[i].dc_tbl_idx = dc_tbl_idx;
        scan_info.components[i].ac_tbl_idx = ac_tbl_idx;
    }
    if (pos.* + 3 > data.len) return Error.UnexpectedEndOfInput;
    scan_info.Ss = readU8(data, pos);
    scan_info.Se = readU8(data, pos);
    if (scan_info.Ss > 63 or scan_info.Se < scan_info.Ss or scan_info.Se > 63) return Error.InvalidJpeg;
    const c = readU8(data, pos);
    scan_info.Ah = c >> 4;
    scan_info.Al = c & 0xf;
    try jpg.scan_info.append(allocator, scan_info);
    if (start_pos + marker_len != pos.*) return Error.InvalidJpeg;
}

fn processDHT(data: []const u8, pos: *usize, jpg: *JPEGData, mode: jd.JpegReadMode, dc_huff_lut: *[jd.kMaxHuffmanTables * kJpegHuffmanLutSize]HuffmanTableEntry, ac_huff_lut: *[jd.kMaxHuffmanTables * kJpegHuffmanLutSize]HuffmanTableEntry, allocator: std.mem.Allocator) Error!void {
    const start_pos = pos.*;
    if (pos.* + 2 > data.len) return Error.UnexpectedEndOfInput;
    const marker_len: usize = readU16(data, pos);
    if (marker_len == 2) {
        // Empty DHT marker; represent with a dummy all-zeroes table.
        try jpg.huffman_code.append(allocator, .{ .is_last = true });
        return;
    }
    while (pos.* < start_pos + marker_len) {
        if (pos.* + 1 + jd.kJpegHuffmanMaxBitLength > data.len) return Error.UnexpectedEndOfInput;
        var huff = JPEGHuffmanCode{};
        huff.slot_id = readU8(data, pos);
        var huffman_index: usize = @intCast(huff.slot_id);
        const is_ac_table = (huff.slot_id & 0x10) != 0;
        var huff_lut: []HuffmanTableEntry = undefined;
        if (is_ac_table) {
            huffman_index -= 0x10;
            if (huffman_index > 3) return Error.InvalidJpeg;
            huff_lut = ac_huff_lut[huffman_index * kJpegHuffmanLutSize ..][0..kJpegHuffmanLutSize];
        } else {
            if (huffman_index > 3) return Error.InvalidJpeg;
            huff_lut = dc_huff_lut[huffman_index * kJpegHuffmanLutSize ..][0..kJpegHuffmanLutSize];
        }
        huff.counts[0] = 0;
        var total_count: i32 = 0;
        var space: i32 = 1 << jd.kJpegHuffmanMaxBitLength;
        var max_depth: u5 = 1;
        for (1..jd.kJpegHuffmanMaxBitLength + 1) |i| {
            const count = readU8(data, pos);
            if (count != 0) max_depth = @intCast(i);
            huff.counts[i] = count;
            total_count += count;
            space -= @as(i32, count) * (@as(i32, 1) << @intCast(jd.kJpegHuffmanMaxBitLength - @as(u32, @intCast(i))));
        }
        if (is_ac_table) {
            if (total_count < 0 or total_count > jd.kJpegHuffmanAlphabetSize) return Error.InvalidJpeg;
        } else {
            if (total_count < 0 or total_count > jd.kJpegDCAlphabetSize) return Error.InvalidJpeg;
        }
        if (pos.* + @as(usize, @intCast(total_count)) > data.len) return Error.UnexpectedEndOfInput;
        var values_seen = [_]bool{false} ** 256;
        var i: usize = 0;
        while (i < total_count) : (i += 1) {
            const value = readU8(data, pos);
            if (!is_ac_table and value >= jd.kJpegDCAlphabetSize) return Error.InvalidJpeg;
            if (values_seen[value]) return Error.InvalidJpeg;
            values_seen[value] = true;
            huff.values[i] = value;
        }
        // Add an invalid symbol that will have the all 1 code.
        huff.counts[max_depth] += 1;
        huff.values[@intCast(total_count)] = jd.kJpegHuffmanAlphabetSize;
        space -= @as(i32, 1) << @intCast(jd.kJpegHuffmanMaxBitLength - max_depth);
        if (space < 0) {
            return Error.InvalidJpeg;
        } else if (space > 0 and huff_lut[0].value != 0xffff) {
            for (huff_lut) |*entry| {
                entry.* = .{ .bits = 0, .value = 0xffff };
            }
        }
        huff.is_last = (pos.* == start_pos + marker_len);
        if (mode == .read_all) {
            buildJpegHuffmanTable(&huff.counts, &huff.values, huff_lut);
        }
        try jpg.huffman_code.append(allocator, huff);
    }
    if (start_pos + marker_len != pos.*) return Error.InvalidJpeg;
}

fn processDQT(data: []const u8, pos: *usize, jpg: *JPEGData, allocator: std.mem.Allocator) Error!void {
    const start_pos = pos.*;
    if (pos.* + 2 > data.len) return Error.UnexpectedEndOfInput;
    const marker_len: usize = readU16(data, pos);
    if (marker_len == 2) return Error.InvalidJpeg;
    while (pos.* < start_pos + marker_len and jpg.quant.items.len < jd.kMaxQuantTables) {
        if (pos.* + 1 > data.len) return Error.UnexpectedEndOfInput;
        var quant_table_index = readU8(data, pos);
        const quant_table_precision: u32 = quant_table_index >> 4;
        if (quant_table_precision > 1) return Error.InvalidJpeg;
        quant_table_index &= 0xf;
        if (quant_table_index > 3) return Error.InvalidJpeg;
        if (pos.* + (quant_table_precision + 1) * jd.kDCTBlockSize > data.len) return Error.UnexpectedEndOfInput;
        var table = JPEGQuantTable{};
        table.index = quant_table_index;
        table.precision = quant_table_precision;
        for (0..jd.kDCTBlockSize) |i| {
            const quant_val: i32 = if (quant_table_precision != 0) readU16(data, pos) else readU8(data, pos);
            if (quant_val < 1 or quant_val > 65535) return Error.InvalidJpeg;
            table.values[jd.kJPEGNaturalOrder[i]] = quant_val;
        }
        table.is_last = (pos.* == start_pos + marker_len);
        try jpg.quant.append(allocator, table);
    }
    if (start_pos + marker_len != pos.*) return Error.InvalidJpeg;
}

fn processDRI(data: []const u8, pos: *usize, found_dri: *bool, jpg: *JPEGData) Error!void {
    if (found_dri.*) return Error.InvalidJpeg; // duplicate DRI
    found_dri.* = true;
    const start_pos = pos.*;
    if (pos.* + 4 > data.len) return Error.UnexpectedEndOfInput;
    const marker_len: usize = readU16(data, pos);
    const restart_interval = readU16(data, pos);
    jpg.restart_interval = restart_interval;
    if (start_pos + marker_len != pos.*) return Error.InvalidJpeg;
}

fn processAPP(data: []const u8, pos: *usize, jpg: *JPEGData, allocator: std.mem.Allocator) Error!void {
    if (pos.* + 2 > data.len) return Error.UnexpectedEndOfInput;
    const marker_len: usize = readU16(data, pos);
    if (marker_len < 2 or marker_len > 65535) return Error.InvalidJpeg;
    if (pos.* + marker_len - 2 > data.len) return Error.UnexpectedEndOfInput;
    // Marker type byte + length + payload.
    const app_str_start = pos.* - 3;
    var app_str: std.ArrayList(u8) = .empty;
    try app_str.appendSlice(allocator, data[app_str_start .. app_str_start + marker_len + 1]);
    try jpg.app_data.append(allocator, app_str);
    // libjxl's ProcessAPP: every APP marker starts out unknown; the jbrd
    // encoder re-types ICC/Exif/XMP markers via DetectIccProfile/DetectBlobs.
    try jpg.app_marker_type.append(allocator, .unknown);
    pos.* += marker_len - 2;
}

fn processCOM(data: []const u8, pos: *usize, jpg: *JPEGData, allocator: std.mem.Allocator) Error!void {
    if (pos.* + 2 > data.len) return Error.UnexpectedEndOfInput;
    const marker_len: usize = readU16(data, pos);
    if (marker_len < 2 or marker_len > 65535) return Error.InvalidJpeg;
    if (pos.* + marker_len - 2 > data.len) return Error.UnexpectedEndOfInput;
    const com_str_start = pos.* - 3;
    var com_str: std.ArrayList(u8) = .empty;
    try com_str.appendSlice(allocator, data[com_str_start .. com_str_start + marker_len + 1]);
    try jpg.com_data.append(allocator, com_str);
    pos.* += marker_len - 2;
}

fn decodeDCTBlock(dc_huff: []const HuffmanTableEntry, ac_huff: []const HuffmanTableEntry, ss_in: i32, se: i32, al: u5, eobrun: *i32, reset_state: *bool, num_zero_runs: *i32, br: *BitReaderState, last_dc_coeff: *coeff_t, coeffs: []coeff_t) Error!void {
    var ss = ss_in;
    const am: i32 = @as(i32, 1) << al;
    const eobrun_allowed = ss > 0;
    if (ss == 0) {
        const s = readSymbol(dc_huff, br);
        if (s >= jd.kJpegDCAlphabetSize) return Error.InvalidJpeg;
        var diff: i32 = 0;
        if (s > 0) {
            const bits = br.readBits(@intCast(s));
            diff = huffExtend(bits, @intCast(s));
        }
        const coeff: i32 = diff + last_dc_coeff.*;
        const dc_coeff: i32 = coeff * am;
        if (dc_coeff != @as(i32, @as(coeff_t, @truncate(dc_coeff)))) return Error.InvalidJpeg;
        coeffs[0] = @truncate(dc_coeff);
        last_dc_coeff.* = @truncate(coeff);
        ss += 1;
    }
    if (ss > se) return;
    if (eobrun.* > 0) {
        eobrun.* -= 1;
        return;
    }
    num_zero_runs.* = 0;
    var k: i32 = ss;
    while (k <= se) : (k += 1) {
        const sr = readSymbol(ac_huff, br);
        if (sr >= jd.kJpegHuffmanAlphabetSize) return Error.InvalidJpeg;
        const r: i32 = sr >> 4;
        const s: i32 = sr & 15;
        if (s > 0) {
            k += r;
            if (k > se) return Error.InvalidJpeg;
            if (s + al >= jd.kJpegDCAlphabetSize) return Error.InvalidJpeg;
            const bits = br.readBits(@intCast(s));
            const coeff = huffExtend(bits, @intCast(s));
            coeffs[jd.kJPEGNaturalOrder[@intCast(k)]] = @truncate(@as(i32, coeff) * am);
            num_zero_runs.* = 0;
        } else if (r == 15) {
            k += 15;
            num_zero_runs.* += 1;
        } else {
            if (eobrun_allowed and k == ss and eobrun.* == 0) {
                reset_state.* = true;
            }
            eobrun.* = @as(i32, 1) << @intCast(r);
            if (r > 0) {
                if (!eobrun_allowed) return Error.InvalidJpeg;
                eobrun.* += @intCast(br.readBits(@intCast(r)));
            }
            break;
        }
    }
    eobrun.* -= 1;
}

fn refineDCTBlock(ac_huff: []const HuffmanTableEntry, ss_in: i32, se: i32, al: u5, eobrun: *i32, reset_state: *bool, br: *BitReaderState, coeffs: []coeff_t) Error!void {
    var ss = ss_in;
    const am: i32 = @as(i32, 1) << al;
    const eobrun_allowed = ss > 0;
    if (ss == 0) {
        const s = br.readBits(1);
        var dc_coeff = coeffs[0];
        if (s != 0) dc_coeff +%= @truncate(am);
        coeffs[0] = dc_coeff;
        ss += 1;
    }
    if (ss > se) return;
    const p1: coeff_t = @truncate(am);
    const m1: coeff_t = @truncate(-am);
    var k: i32 = ss;
    var s: i32 = 0;
    var in_zero_run = false;
    if (eobrun.* <= 0) {
        while (k <= se) : (k += 1) {
            var sym = readSymbol(ac_huff, br);
            if (sym >= jd.kJpegHuffmanAlphabetSize) return Error.InvalidJpeg;
            var r: i32 = sym >> 4;
            sym &= 15;
            s = sym;
            if (s != 0) {
                if (s != 1) return Error.InvalidJpeg;
                s = if (br.readBits(1) != 0) p1 else m1;
                in_zero_run = false;
            } else {
                if (r != 15) {
                    if (eobrun_allowed and k == ss and eobrun.* == 0) {
                        reset_state.* = true;
                    }
                    eobrun.* = @as(i32, 1) << @intCast(r);
                    if (r > 0) {
                        if (!eobrun_allowed) return Error.InvalidJpeg;
                        eobrun.* += @intCast(br.readBits(@intCast(r)));
                    }
                    break;
                }
                in_zero_run = true;
            }
            while (true) {
                var thiscoef = coeffs[jd.kJPEGNaturalOrder[@intCast(k)]];
                if (thiscoef != 0) {
                    if (br.readBits(1) != 0) {
                        if ((thiscoef & p1) == 0) {
                            thiscoef +%= if (thiscoef >= 0) p1 else m1;
                        }
                    }
                    coeffs[jd.kJPEGNaturalOrder[@intCast(k)]] = thiscoef;
                } else {
                    r -= 1;
                    if (r < 0) break;
                }
                k += 1;
                if (k > se) break;
            }
            if (s != 0) {
                if (k > se) return Error.InvalidJpeg;
                coeffs[jd.kJPEGNaturalOrder[@intCast(k)]] = @truncate(s);
            }
        }
    }
    if (in_zero_run) return Error.InvalidJpeg;
    if (eobrun.* > 0) {
        while (k <= se) : (k += 1) {
            var thiscoef = coeffs[jd.kJPEGNaturalOrder[@intCast(k)]];
            if (thiscoef != 0) {
                if (br.readBits(1) != 0) {
                    if ((thiscoef & p1) == 0) {
                        thiscoef +%= if (thiscoef >= 0) p1 else m1;
                    }
                }
                coeffs[jd.kJPEGNaturalOrder[@intCast(k)]] = thiscoef;
            }
        }
    }
    eobrun.* -= 1;
}

fn processRestart(data: []const u8, next_restart_marker: *i32, br: *BitReaderState, allocator: std.mem.Allocator, jpg: *JPEGData) Error!bool {
    var pos: usize = 0;
    try br.finishStream(allocator, jpg, &pos);
    const expected_marker: u8 = @intCast(0xd0 + next_restart_marker.*);
    if (pos + 2 > data.len or data[pos] != 0xff) return Error.InvalidJpeg;
    const marker = data[pos + 1];
    if (marker != expected_marker) return Error.InvalidJpeg;
    br.reset(pos + 2);
    next_restart_marker.* += 1;
    next_restart_marker.* &= 0x7;
    return true;
}

fn processScan(data: []const u8, dc_huff_lut: *const [jd.kMaxHuffmanTables * kJpegHuffmanLutSize]HuffmanTableEntry, ac_huff_lut: *const [jd.kMaxHuffmanTables * kJpegHuffmanLutSize]HuffmanTableEntry, scan_progression: *[jd.kMaxComponents][jd.kDCTBlockSize]u16, is_progressive: bool, pos: *usize, jpg: *JPEGData, allocator: std.mem.Allocator) Error!void {
    try processSOS(data, pos, jpg, allocator);
    const scan_info = &jpg.scan_info.items[jpg.scan_info.items.len - 1];
    const is_interleaved = scan_info.num_components > 1;
    var max_h_samp_factor: i32 = 1;
    var max_v_samp_factor: i32 = 1;
    for (jpg.components.items) |*component| {
        max_h_samp_factor = @max(max_h_samp_factor, component.h_samp_factor);
        max_v_samp_factor = @max(max_v_samp_factor, component.v_samp_factor);
    }

    var mcu_rows: i32 = @intCast(std.math.divCeil(u32, @as(u32, @intCast(jpg.height)), @as(u32, @intCast(max_v_samp_factor * 8))) catch return Error.InvalidJpeg);
    var mcus_per_row: i32 = @intCast(std.math.divCeil(u32, @as(u32, @intCast(jpg.width)), @as(u32, @intCast(max_h_samp_factor * 8))) catch return Error.InvalidJpeg);
    if (!is_interleaved) {
        const c = &jpg.components.items[scan_info.components[0].comp_idx];
        mcus_per_row = @intCast(std.math.divCeil(u32, @as(u32, @intCast(jpg.width)) * @as(u32, @intCast(c.h_samp_factor)), @as(u32, @intCast(8 * max_h_samp_factor))) catch return Error.InvalidJpeg);
        mcu_rows = @intCast(std.math.divCeil(u32, @as(u32, @intCast(jpg.height)) * @as(u32, @intCast(c.v_samp_factor)), @as(u32, @intCast(8 * max_v_samp_factor))) catch return Error.InvalidJpeg);
    }
    var last_dc_coeff = [_]coeff_t{0} ** jd.kMaxComponents;
    var br = BitReaderState.init(data, pos.*);
    var restarts_to_go: i32 = @intCast(jpg.restart_interval);
    var next_restart_marker: i32 = 0;
    var eobrun: i32 = -1;
    var block_scan_index: u32 = 0;
    const al: u5 = @intCast(if (is_progressive) scan_info.Al else 0);
    const ah: u5 = @intCast(if (is_progressive) scan_info.Ah else 0);
    const ss: i32 = if (is_progressive) @intCast(scan_info.Ss) else 0;
    const se: i32 = if (is_progressive) @intCast(scan_info.Se) else 63;
    const scan_bitmask: u16 = @truncate(if (ah == 0) (@as(u32, 0xffff) << al) else (@as(u32, 1) << al));
    const refinement_bitmask: u16 = @truncate((@as(u32, 1) << al) - 1);
    for (0..scan_info.num_components) |i| {
        const comp_idx = scan_info.components[i].comp_idx;
        var k: usize = @intCast(ss);
        while (k <= se) : (k += 1) {
            if (scan_progression[comp_idx][k] & scan_bitmask != 0) {
                return Error.InvalidJpeg; // overlapping scans
            }
            if (scan_progression[comp_idx][k] & refinement_bitmask != 0) {
                return Error.InvalidJpeg; // more refined scan already done
            }
            scan_progression[comp_idx][k] |= scan_bitmask;
        }
    }
    if (al > 10) return Error.InvalidJpeg;

    for (jpg.components.items) |*c| {
        if (c.coeffs.items.len == 0) {
            const num_blocks: u64 = @as(u64, c.width_in_blocks) * c.height_in_blocks;
            try c.coeffs.resize(allocator, num_blocks * jd.kDCTBlockSize);
            @memset(c.coeffs.items, 0);
        }
    }

    var mcu_y: i32 = 0;
    while (mcu_y < mcu_rows) : (mcu_y += 1) {
        var mcu_x: i32 = 0;
        while (mcu_x < mcus_per_row) : (mcu_x += 1) {
            if (jpg.restart_interval > 0) {
                if (restarts_to_go == 0) {
                    if (!try processRestart(data, &next_restart_marker, &br, allocator, jpg)) {
                        return Error.InvalidJpeg;
                    }
                    restarts_to_go = @intCast(jpg.restart_interval);
                    @memset(&last_dc_coeff, 0);
                    if (eobrun > 0) return Error.InvalidJpeg;
                    eobrun = -1; // fresh start
                }
                restarts_to_go -= 1;
            }
            // Decode one MCU.
            for (0..scan_info.num_components) |i| {
                const si = &scan_info.components[i];
                const c = &jpg.components.items[si.comp_idx];
                const dc_lut = dc_huff_lut[si.dc_tbl_idx * kJpegHuffmanLutSize ..][0..kJpegHuffmanLutSize];
                const ac_lut = ac_huff_lut[si.ac_tbl_idx * kJpegHuffmanLutSize ..][0..kJpegHuffmanLutSize];
                const nblocks_y: i32 = if (is_interleaved) c.v_samp_factor else 1;
                const nblocks_x: i32 = if (is_interleaved) c.h_samp_factor else 1;
                var iy: i32 = 0;
                while (iy < nblocks_y) : (iy += 1) {
                    var ix: i32 = 0;
                    while (ix < nblocks_x) : (ix += 1) {
                        const block_y: i32 = mcu_y * nblocks_y + iy;
                        const block_x: i32 = mcu_x * nblocks_x + ix;
                        const block_idx: usize = @intCast(@as(i64, block_y) * c.width_in_blocks + block_x);
                        var reset_state = false;
                        var num_zero_runs: i32 = 0;
                        const coeffs = c.coeffs.items[block_idx * jd.kDCTBlockSize ..][0..jd.kDCTBlockSize];
                        if (ah == 0) {
                            try decodeDCTBlock(dc_lut, ac_lut, ss, se, al, &eobrun, &reset_state, &num_zero_runs, &br, &last_dc_coeff[si.comp_idx], coeffs);
                        } else {
                            try refineDCTBlock(ac_lut, ss, se, al, &eobrun, &reset_state, &br, coeffs);
                        }
                        if (reset_state) {
                            try scan_info.reset_points.append(allocator, block_scan_index);
                        }
                        if (num_zero_runs > 0) {
                            try scan_info.extra_zero_runs.append(allocator, .{
                                .block_idx = block_scan_index,
                                .num_extra_zero_runs = @intCast(num_zero_runs),
                            });
                        }
                        block_scan_index += 1;
                    }
                }
            }
        }
    }
    if (eobrun > 0) return Error.InvalidJpeg;
    try br.finishStream(allocator, jpg, pos);
    if (pos.* > data.len) return Error.UnexpectedEndOfInput;
}

fn fixupIndexes(jpg: *JPEGData) Error!void {
    for (jpg.components.items) |*c| {
        var found_index = false;
        for (jpg.quant.items, 0..) |q, j| {
            if (q.index == c.quant_idx) {
                c.quant_idx = @intCast(j);
                found_index = true;
                break;
            }
        }
        if (!found_index) return Error.InvalidJpeg;
    }
}

fn findNextMarker(data: []const u8, pos_in: usize) usize {
    // kIsValidMarker[i] == 1 means (0xc0 + i) is a valid marker.
    const kIsValidMarker = [_]u8{
        1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
        1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    };
    var pos = pos_in;
    var num_skipped: usize = 0;
    while (pos + 1 < data.len and (data[pos] != 0xff or data[pos + 1] < 0xc0 or
        kIsValidMarker[data[pos + 1] - 0xc0] == 0))
    {
        pos += 1;
        num_skipped += 1;
    }
    return num_skipped;
}

fn readU8(data: []const u8, pos: *usize) u8 {
    const v = data[pos.*];
    pos.* += 1;
    return v;
}

fn readU16(data: []const u8, pos: *usize) u16 {
    const v = (@as(u16, data[pos.*]) << 8) + data[pos.* + 1];
    pos.* += 2;
    return v;
}

fn expectMarker(data: []const u8, pos: usize) Error!void {
    if (pos + 2 > data.len or data[pos] != 0xff) return Error.InvalidJpeg;
}

/// Parses a JPEG file into `jpg` (port of ReadJpeg).
pub fn readJpeg(data: []const u8, mode: jd.JpegReadMode, jpg: *JPEGData, allocator: std.mem.Allocator) Error!void {
    var pos: usize = 0;
    try expectMarker(data, pos);
    var marker = data[pos + 1];
    pos += 2;
    if (marker != 0xd8) return Error.InvalidJpeg; // SOI

    var dc_huff_lut: [jd.kMaxHuffmanTables * kJpegHuffmanLutSize]HuffmanTableEntry = @splat(.{});
    var ac_huff_lut: [jd.kMaxHuffmanTables * kJpegHuffmanLutSize]HuffmanTableEntry = @splat(.{});
    var found_sof = false;
    var found_sos = false;
    var found_dri = false;
    var scan_progression = [_][jd.kDCTBlockSize]u16{@splat(0)} ** jd.kMaxComponents;
    var is_progressive = false;

    while (true) {
        const num_skipped = findNextMarker(data, pos);
        if (num_skipped > 0) {
            try jpg.marker_order.append(allocator, 0xff);
            var imd: std.ArrayList(u8) = .empty;
            try imd.appendSlice(allocator, data[pos .. pos + num_skipped]);
            try jpg.inter_marker_data.append(allocator, imd);
            pos += num_skipped;
        }
        try expectMarker(data, pos);
        marker = data[pos + 1];
        pos += 2;
        switch (marker) {
            0xc0, 0xc1, 0xc2 => {
                is_progressive = (marker == 0xc2);
                try processSOF(data, &pos, jpg, allocator);
                found_sof = true;
            },
            0xc4 => try processDHT(data, &pos, jpg, mode, &dc_huff_lut, &ac_huff_lut, allocator),
            0xd0...0xd7 => {}, // RST markers have no data
            0xd9 => {}, // EOI
            0xda => {
                if (mode == .read_all) {
                    try processScan(data, &dc_huff_lut, &ac_huff_lut, &scan_progression, is_progressive, &pos, jpg, allocator);
                }
                found_sos = true;
            },
            0xdb => try processDQT(data, &pos, jpg, allocator),
            0xdd => try processDRI(data, &pos, &found_dri, jpg),
            0xe0...0xef => {
                if (mode != .read_tables) {
                    try processAPP(data, &pos, jpg, allocator);
                }
            },
            0xfe => {
                if (mode != .read_tables) {
                    try processCOM(data, &pos, jpg, allocator);
                }
            },
            else => return Error.UnsupportedMarker,
        }
        try jpg.marker_order.append(allocator, marker);
        if (mode == .read_headers and found_sof) break;
        if (marker == 0xd9) break;
    }

    if (!found_sof) return Error.InvalidJpeg;
    if (!found_sos) return Error.InvalidJpeg;

    if (mode == .read_all) {
        if (pos < data.len) {
            try jpg.tail_data.appendSlice(allocator, data[pos..]);
        }
        try fixupIndexes(jpg);
        if (jpg.huffman_code.items.len == 0) return Error.InvalidJpeg;
        if (jpg.huffman_code.items.len >= jd.kMaxDHTMarkers) return Error.InvalidJpeg;
    }
}
