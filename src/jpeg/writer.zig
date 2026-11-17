//! JPEG reconstruction writer, port of libjxl `jpeg/dec_jpeg_data_writer.cc`.
//! Re-emits the entropy-coded scan from decoded coefficients plus the
//! bit-exactness side data (padding bits, reset points, extra zero runs).

const std = @import("std");
const jd = @import("jpeg_data.zig");

const JPEGData = jd.JPEGData;
const JPEGScanInfo = jd.JPEGScanInfo;
const JPEGHuffmanCode = jd.JPEGHuffmanCode;
const coeff_t = jd.coeff_t;

pub const Error = error{
    InvalidJpegData,
    OutOfMemory,
};

fn floorLog2Nonzero(x: u32) u5 {
    std.debug.assert(x != 0);
    return @intCast(31 - @clz(x));
}

/// Returns non-zero if and only if x has a zero byte.
fn hasZeroByte(x: u64) u64 {
    return (x -% 0x0101010101010101) & ~x & 0x8080808080808080;
}

/// Bit writer with 0xff/0x00 stuffing, port of JpegBitWriter on top of a
/// growable byte buffer.
const JpegBitWriter = struct {
    out: *std.ArrayList(u8),
    allocator: std.mem.Allocator,
    put_buffer: u64 = 0,
    put_bits: i32 = 64,
    healthy: bool = true,

    fn emitByte(self: *JpegBitWriter, byte: u8) Error!void {
        try self.out.append(self.allocator, byte);
        if (byte == 0xFF) try self.out.append(self.allocator, 0);
    }

    fn dischargeBitBuffer(self: *JpegBitWriter, bits: u64) Error!void {
        self.put_buffer |= bits >> @intCast(-self.put_bits);
        if (hasZeroByte(~self.put_buffer) != 0) {
            // Slow path: some byte is 0xFF, stuff zeros.
            var shift: i32 = 56;
            while (shift >= 0) : (shift -= 8) {
                try self.emitByte(@truncate((self.put_buffer >> @intCast(shift)) & 0xFF));
            }
        } else {
            var bytes: [8]u8 = undefined;
            std.mem.writeInt(u64, &bytes, self.put_buffer, .big);
            try self.out.appendSlice(self.allocator, &bytes);
        }
        self.put_bits += 64;
        self.put_buffer = bits << @intCast(self.put_bits);
    }

    fn writeBits(self: *JpegBitWriter, nbits: u7, bits: u64) Error!void {
        std.debug.assert(nbits > 0);
        self.put_bits -= nbits;
        if (self.put_bits < 0) {
            if (nbits > 64) {
                self.put_bits += nbits;
                self.healthy = false;
            } else {
                try self.dischargeBitBuffer(bits);
            }
        } else {
            self.put_buffer |= bits << @intCast(self.put_bits);
        }
    }

    fn emitMarker(self: *JpegBitWriter, marker: u8) Error!void {
        try self.out.append(self.allocator, 0xFF);
        try self.out.append(self.allocator, marker);
    }
};

/// Iterator over recorded padding bits (empty slice means: pad with ones).
const PadBits = struct {
    bits: []const u8 = &.{},
    pos: usize = 0,
};

fn jumpToByteBoundary(bw: *JpegBitWriter, pad: *PadBits) Error!bool {
    var n_bits: u32 = @intCast(bw.put_bits & 7);
    var dangling_bits: u8 = 0;
    var pad_pattern: u8 = undefined;
    if (pad.bits.len == 0) {
        pad_pattern = @intCast((@as(u32, 1) << @intCast(n_bits)) - 1);
    } else {
        pad_pattern = 0;
        while (n_bits > 0) : (n_bits -= 1) {
            pad_pattern <<= 1;
            if (pad.pos >= pad.bits.len) return false;
            const bit = pad.bits[pad.pos];
            pad.pos += 1;
            dangling_bits |= bit;
            pad_pattern |= bit;
        }
    }
    if ((dangling_bits & ~@as(u8, 1)) != 0) return false;

    while (bw.put_bits <= 56) {
        const c: u8 = @truncate((bw.put_buffer >> 56) & 0xFF);
        try bw.emitByte(c);
        bw.put_buffer <<= 8;
        bw.put_bits += 8;
    }
    if (bw.put_bits < 64) {
        const pad_mask: u8 = @truncate(@as(u32, 0xFF) >> @intCast(64 - bw.put_bits));
        const c: u8 = @truncate(((bw.put_buffer >> 56) & ~@as(u64, pad_mask)) | pad_pattern);
        try bw.emitByte(c);
    }
    bw.put_buffer = 0;
    bw.put_bits = 64;
    return true;
}

/// Huffman encoding table (symbol -> code/depth).
const HuffmanCodeTable = struct {
    depth: [256]i8 = @splat(0),
    code: [256]u16 = @splat(0),
    initialized: bool = false,

    fn initDepths(self: *HuffmanCodeTable, value: i8) void {
        self.depth = @splat(value);
    }
};

fn buildHuffmanCodeTable(huff: *const JPEGHuffmanCode, table: *HuffmanCodeTable) bool {
    var huff_code: [jd.kJpegHuffmanAlphabetSize]i32 = undefined;
    // +1 for a sentinel element.
    var huff_size: [jd.kJpegHuffmanAlphabetSize + 1]u32 = undefined;
    var p: usize = 0;
    for (1..jd.kJpegHuffmanMaxBitLength + 1) |l| {
        var i = huff.counts[l];
        if (p + i > jd.kJpegHuffmanAlphabetSize + 1) return false;
        while (i > 0) : (i -= 1) {
            huff_size[p] = @intCast(l);
            p += 1;
        }
    }
    if (p == 0) return true; // empty table stays uninitialized

    // Reuse sentinel element.
    const last_p = p - 1;
    huff_size[last_p] = 0;

    var code: i32 = 0;
    var si: u32 = huff_size[0];
    p = 0;
    while (huff_size[p] != 0) {
        while (huff_size[p] == si) {
            huff_code[p] = code;
            p += 1;
            code += 1;
        }
        code <<= 1;
        si += 1;
    }
    for (0..last_p) |i| {
        const sym = huff.values[i];
        table.depth[sym] = @intCast(huff_size[i]);
        table.code[sym] = @intCast(huff_code[i]);
    }
    return true;
}

fn writeSymbol(symbol: u32, table: *const HuffmanCodeTable, bw: *JpegBitWriter) Error!void {
    try bw.writeBits(@intCast(table.depth[symbol]), table.code[symbol]);
}

fn writeSymbolBits(symbol: u32, table: *const HuffmanCodeTable, bw: *JpegBitWriter, nbits: u7, bits: u64) Error!void {
    try bw.writeBits(@intCast(nbits + @as(u7, @intCast(table.depth[symbol]))), bits | (@as(u64, table.code[symbol]) << @as(u6, @intCast(nbits))));
}

/// Holds data buffered between 8x8 blocks in progressive mode.
const DCTCodingState = struct {
    eob_run: i32 = 0,
    cur_ac_huff: ?*const HuffmanCodeTable = null,
    refinement_bits: std.ArrayList(u16) = .empty,
    refinement_bits_count: usize = 0,

    fn deinit(self: *DCTCodingState, allocator: std.mem.Allocator) void {
        self.refinement_bits.deinit(allocator);
    }
};

/// Emit all buffered data to the bit stream.
fn flushCodingState(s: *DCTCodingState, bw: *JpegBitWriter) Error!void {
    if (s.eob_run > 0) {
        const nbits = floorLog2Nonzero(@intCast(s.eob_run));
        const symbol: u32 = @as(u32, nbits) << 4;
        try writeSymbol(symbol, s.cur_ac_huff.?, bw);
        if (nbits > 0) {
            try bw.writeBits(nbits, @as(u64, @intCast(s.eob_run)) & ((@as(u64, 1) << @as(u6, @intCast(nbits))) - 1));
        }
        s.eob_run = 0;
    }
    const num_words = s.refinement_bits_count >> 4;
    var i: usize = 0;
    while (i < num_words) : (i += 1) {
        try bw.writeBits(16, s.refinement_bits.items[i]);
    }
    const tail = s.refinement_bits_count & 0xF;
    if (tail > 0) {
        try bw.writeBits(@intCast(tail), s.refinement_bits.items[s.refinement_bits.items.len - 1]);
    }
    s.refinement_bits.clearRetainingCapacity();
    s.refinement_bits_count = 0;
}

/// Buffer data at the end-of-band.
fn bufferEndOfBand(s: *DCTCodingState, ac_huff: *const HuffmanCodeTable, new_bits_array: []const u32, bw: *JpegBitWriter) Error!void {
    if (s.eob_run == 0) {
        s.cur_ac_huff = ac_huff;
    }
    s.eob_run += 1;
    var new_bits_count = new_bits_array.len;
    if (new_bits_count > 0) {
        var new_bits: u64 = 0;
        for (new_bits_array) |b| {
            new_bits = (new_bits << 1) | b;
        }
        const tail = s.refinement_bits_count & 0xF;
        if (tail > 0) { // First stuff the tail item.
            const stuff_bits_count: usize = @min(16 - tail, new_bits_count);
            var stuff_bits: u16 = @intCast(new_bits >> @intCast(new_bits_count - stuff_bits_count));
            stuff_bits &= @intCast((@as(u32, 1) << @intCast(stuff_bits_count)) - 1);
            const back = &s.refinement_bits.items[s.refinement_bits.items.len - 1];
            back.* = (back.* << @intCast(stuff_bits_count)) | stuff_bits;
            new_bits_count -= stuff_bits_count;
            s.refinement_bits_count += stuff_bits_count;
        }
        while (new_bits_count >= 16) {
            try s.refinement_bits.append(bw.allocator, @intCast(new_bits >> @intCast(new_bits_count - 16)));
            new_bits_count -= 16;
            s.refinement_bits_count += 16;
        }
        if (new_bits_count > 0) {
            try s.refinement_bits.append(bw.allocator, @intCast(new_bits & ((@as(u64, 1) << @intCast(new_bits_count)) - 1)));
            s.refinement_bits_count += new_bits_count;
        }
    }
    if (s.eob_run == 0x7FFF) {
        try flushCodingState(s, bw);
    }
}

fn encodeDCTBlockSequential(coeffs: []const coeff_t, dc_huff: *const HuffmanCodeTable, ac_huff: *const HuffmanCodeTable, num_zero_runs: i32, last_dc_coeff: *coeff_t, bw: *JpegBitWriter) Error!bool {
    var temp2: i32 = coeffs[0];
    var temp: i32 = temp2 - last_dc_coeff.*;
    last_dc_coeff.* = @truncate(temp2);
    var sign_fill: i32 = temp >> 15;
    temp += sign_fill;
    temp2 = sign_fill ^ temp;

    const dc_nbits: u7 = if (temp2 == 0) 0 else @intCast(floorLog2Nonzero(@bitCast(temp2)) + 1);
    try writeSymbol(@intCast(dc_nbits), dc_huff, bw);
    if (dc_nbits > 0) {
        const mask: u32 = @truncate((@as(u64, 1) << @as(u6, @intCast(dc_nbits))) - 1);
        try bw.writeBits(dc_nbits, @as(u32, @bitCast(temp)) & mask);
    }
    var r: i32 = 0;
    var litmus: i32 = 0;

    for (1..64) |i| {
        temp = coeffs[jd.kJPEGNaturalOrder[i]];
        if (temp == 0) {
            r += 1;
        } else {
            sign_fill = temp >> 15;
            temp += sign_fill;
            temp2 = sign_fill ^ temp;
            if (r > 15) {
                try writeSymbol(0xf0, ac_huff, bw);
                r -= 16;
                if (r > 15) {
                    try writeSymbol(0xf0, ac_huff, bw);
                    r -= 16;
                }
                if (r > 15) {
                    try writeSymbol(0xf0, ac_huff, bw);
                    r -= 16;
                }
            }
            litmus |= temp2;
            const ac_nbits: u7 = @intCast(floorLog2Nonzero(@as(u16, @bitCast(@as(i16, @truncate(temp2))))) + 1);
            const symbol: u32 = @as(u32, @intCast(r)) << 4 | ac_nbits;
            try writeSymbolBits(symbol, ac_huff, bw, ac_nbits, @as(u32, @bitCast(temp)) & @as(u32, @truncate((@as(u64, 1) << @as(u6, @intCast(ac_nbits))) - 1)));
            r = 0;
        }
    }

    var i: i32 = 0;
    while (i < num_zero_runs) : (i += 1) {
        try writeSymbol(0xf0, ac_huff, bw);
        r -= 16;
    }
    if (r > 0) {
        try writeSymbol(0, ac_huff, bw);
    }
    return litmus >= 0;
}

fn encodeDCTBlockProgressive(coeffs: []const coeff_t, dc_huff: *const HuffmanCodeTable, ac_huff: *const HuffmanCodeTable, ss_in: i32, se: i32, al: u5, num_zero_runs: i32, coding_state: *DCTCodingState, last_dc_coeff: *coeff_t, bw: *JpegBitWriter) Error!bool {
    var ss = ss_in;
    const eob_run_allowed = ss > 0;
    if (ss == 0) {
        var temp2: i32 = @as(i32, coeffs[0]) >> al;
        var temp: i32 = temp2 - last_dc_coeff.*;
        last_dc_coeff.* = @truncate(temp2);
        temp2 = temp;
        if (temp < 0) {
            temp = -temp;
            if (temp < 0) return false;
            temp2 -= 1;
        }
        const nbits: u7 = if (temp == 0) 0 else @intCast(floorLog2Nonzero(@intCast(temp)) + 1);
        try writeSymbol(@intCast(nbits), dc_huff, bw);
        if (nbits > 0) {
            const mask: u32 = @truncate((@as(u64, 1) << @as(u6, @intCast(nbits))) - 1);
            try bw.writeBits(nbits, @as(u32, @bitCast(temp2)) & mask);
        }
        ss += 1;
    }
    if (ss > se) return true;
    var r: i32 = 0;
    var k: i32 = ss;
    while (k <= se) : (k += 1) {
        var temp: i32 = coeffs[jd.kJPEGNaturalOrder[@intCast(k)]];
        if (temp == 0) {
            r += 1;
            continue;
        }
        var temp2: i32 = undefined;
        if (temp < 0) {
            temp = -temp;
            if (temp < 0) return false;
            temp >>= al;
            temp2 = ~temp;
        } else {
            temp >>= al;
            temp2 = temp;
        }
        if (temp == 0) {
            r += 1;
            continue;
        }
        try flushCodingState(coding_state, bw);
        while (r > 15) {
            try writeSymbol(0xf0, ac_huff, bw);
            r -= 16;
        }
        const nbits: u7 = @intCast(floorLog2Nonzero(@intCast(temp)) + 1);
        const symbol: u32 = @as(u32, @intCast(r)) << 4 | nbits;
        try writeSymbol(symbol, ac_huff, bw);
        const mask: u32 = @truncate((@as(u64, 1) << @as(u6, @intCast(nbits))) - 1);
        try bw.writeBits(nbits, @as(u32, @bitCast(temp2)) & mask);
        r = 0;
    }
    if (num_zero_runs > 0) {
        try flushCodingState(coding_state, bw);
        var i: i32 = 0;
        while (i < num_zero_runs) : (i += 1) {
            try writeSymbol(0xf0, ac_huff, bw);
            r -= 16;
        }
    }
    if (r > 0) {
        try bufferEndOfBand(coding_state, ac_huff, &.{}, bw);
        if (!eob_run_allowed) {
            try flushCodingState(coding_state, bw);
        }
    }
    return true;
}

fn encodeRefinementBits(coeffs: []const coeff_t, ac_huff: *const HuffmanCodeTable, ss_in: i32, se: i32, al: u5, coding_state: *DCTCodingState, bw: *JpegBitWriter) Error!bool {
    var ss = ss_in;
    const eob_run_allowed = ss > 0;
    if (ss == 0) {
        // Emit next bit of DC component.
        try bw.writeBits(1, @as(u32, @bitCast(@as(i32, coeffs[0]) >> al)) & 1);
        ss += 1;
    }
    if (ss > se) return true;
    var abs_values: [jd.kDCTBlockSize]i32 = undefined;
    var eob: i32 = 0;
    var k: i32 = ss;
    while (k <= se) : (k += 1) {
        const abs_val: i32 = @intCast(@abs(@as(i32, coeffs[jd.kJPEGNaturalOrder[@intCast(k)]])));
        abs_values[@intCast(k)] = abs_val >> al;
        if (abs_values[@intCast(k)] == 1) {
            eob = k;
        }
    }
    var r: i32 = 0;
    var refinement_bits: [jd.kDCTBlockSize]u32 = undefined;
    var refinement_bits_count: usize = 0;
    k = ss;
    while (k <= se) : (k += 1) {
        if (abs_values[@intCast(k)] == 0) {
            r += 1;
            continue;
        }
        while (r > 15 and k <= eob) {
            try flushCodingState(coding_state, bw);
            try writeSymbol(0xf0, ac_huff, bw);
            r -= 16;
            for (refinement_bits[0..refinement_bits_count]) |bit| {
                try bw.writeBits(1, bit);
            }
            refinement_bits_count = 0;
        }
        if (abs_values[@intCast(k)] > 1) {
            refinement_bits[refinement_bits_count] = @intCast(abs_values[@intCast(k)] & 1);
            refinement_bits_count += 1;
            continue;
        }
        try flushCodingState(coding_state, bw);
        const symbol: u32 = @as(u32, @intCast(r)) << 4 | 1;
        const new_non_zero_bit: u32 = if (coeffs[jd.kJPEGNaturalOrder[@intCast(k)]] < 0) 0 else 1;
        try writeSymbol(symbol, ac_huff, bw);
        try bw.writeBits(1, new_non_zero_bit);
        for (refinement_bits[0..refinement_bits_count]) |bit| {
            try bw.writeBits(1, bit);
        }
        refinement_bits_count = 0;
        r = 0;
    }
    if (r > 0 or refinement_bits_count > 0) {
        try bufferEndOfBand(coding_state, ac_huff, refinement_bits[0..refinement_bits_count], bw);
        if (!eob_run_allowed) {
            try flushCodingState(coding_state, bw);
        }
    }
    return true;
}

fn encodeSOI(out: *std.ArrayList(u8), allocator: std.mem.Allocator) Error!void {
    try out.appendSlice(allocator, &.{ 0xFF, 0xD8 });
}

fn encodeEOI(jpg: *const JPEGData, out: *std.ArrayList(u8), allocator: std.mem.Allocator) Error!void {
    try out.appendSlice(allocator, &.{ 0xFF, 0xD9 });
    try out.appendSlice(allocator, jpg.tail_data.items);
}

fn encodeSOF(jpg: *const JPEGData, marker: u8, out: *std.ArrayList(u8), allocator: std.mem.Allocator) Error!bool {
    const n_comps = jpg.components.items.len;
    const marker_len = 8 + 3 * n_comps;
    try out.ensureUnusedCapacity(allocator, marker_len + 2);
    out.appendAssumeCapacity(0xFF);
    out.appendAssumeCapacity(marker);
    out.appendAssumeCapacity(@intCast(marker_len >> 8));
    out.appendAssumeCapacity(@truncate(marker_len));
    out.appendAssumeCapacity(8); // precision
    out.appendAssumeCapacity(@intCast(jpg.height >> 8));
    out.appendAssumeCapacity(@intCast(jpg.height & 0xFF));
    out.appendAssumeCapacity(@intCast(jpg.width >> 8));
    out.appendAssumeCapacity(@intCast(jpg.width & 0xFF));
    out.appendAssumeCapacity(@intCast(n_comps));
    for (jpg.components.items) |*c| {
        out.appendAssumeCapacity(@intCast(c.id));
        out.appendAssumeCapacity(@intCast((c.h_samp_factor << 4) | c.v_samp_factor));
        if (c.quant_idx >= jpg.quant.items.len) return false;
        out.appendAssumeCapacity(@intCast(jpg.quant.items[c.quant_idx].index));
    }
    return true;
}

fn encodeSOS(jpg: *const JPEGData, scan_info: *const JPEGScanInfo, out: *std.ArrayList(u8), allocator: std.mem.Allocator) Error!bool {
    const n_scans = scan_info.num_components;
    const marker_len = 6 + 2 * n_scans;
    try out.ensureUnusedCapacity(allocator, marker_len + 2);
    out.appendAssumeCapacity(0xFF);
    out.appendAssumeCapacity(0xDA);
    out.appendAssumeCapacity(@intCast(marker_len >> 8));
    out.appendAssumeCapacity(@truncate(marker_len));
    out.appendAssumeCapacity(@intCast(n_scans));
    for (0..n_scans) |i| {
        const si = &scan_info.components[i];
        if (si.comp_idx >= jpg.components.items.len) return false;
        out.appendAssumeCapacity(@intCast(jpg.components.items[si.comp_idx].id));
        out.appendAssumeCapacity(@intCast((si.dc_tbl_idx << 4) + si.ac_tbl_idx));
    }
    out.appendAssumeCapacity(@intCast(scan_info.Ss));
    out.appendAssumeCapacity(@intCast(scan_info.Se));
    out.appendAssumeCapacity(@intCast((scan_info.Ah << 4) | scan_info.Al));
    return true;
}

fn encodeDHT(jpg: *const JPEGData, state: *SerializationState, out: *std.ArrayList(u8), allocator: std.mem.Allocator) Error!bool {
    const huffman_code = jpg.huffman_code.items;
    var marker_len: usize = 2;
    var i = state.dht_index;
    while (i < huffman_code.len) : (i += 1) {
        const huff = &huffman_code[i];
        for (huff.counts) |count| marker_len += count;
        // Special case: empty DHT marker.
        if (marker_len == 2) break;
        marker_len += jd.kJpegHuffmanMaxBitLength;
        if (huff.is_last) break;
    }
    try out.ensureUnusedCapacity(allocator, marker_len + 2);
    out.appendAssumeCapacity(0xFF);
    out.appendAssumeCapacity(0xC4);
    out.appendAssumeCapacity(@intCast(marker_len >> 8));
    out.appendAssumeCapacity(@truncate(marker_len));
    while (true) {
        const huffman_code_index = state.dht_index;
        state.dht_index += 1;
        if (huffman_code_index >= huffman_code.len) return false;
        const huff = &huffman_code[huffman_code_index];
        var index: usize = @intCast(huff.slot_id);
        var huff_table: *HuffmanCodeTable = undefined;
        var total_count: usize = 0;
        var max_length: usize = 0;
        for (huff.counts, 0..) |count, ci| {
            if (count != 0) max_length = ci;
            total_count += count;
        }
        // Empty DHT marker.
        if (total_count == 0) break;
        if (index & 0x10 != 0) {
            index -= 0x10;
            huff_table = &state.ac_huff_table[index];
        } else {
            huff_table = &state.dc_huff_table[index];
        }
        huff_table.initDepths(127);
        if (!buildHuffmanCodeTable(huff, huff_table)) return false;
        huff_table.initialized = true;
        total_count -= 1;
        out.appendAssumeCapacity(@intCast(huff.slot_id));
        for (1..jd.kJpegHuffmanMaxBitLength + 1) |ci| {
            const count: u8 = @intCast(if (ci == max_length) huff.counts[ci] - 1 else huff.counts[ci]);
            out.appendAssumeCapacity(count);
        }
        for (0..total_count) |vi| {
            out.appendAssumeCapacity(@intCast(huff.values[vi]));
        }
        if (huff.is_last) break;
    }
    return true;
}

fn encodeDQT(jpg: *const JPEGData, state: *SerializationState, out: *std.ArrayList(u8), allocator: std.mem.Allocator) Error!bool {
    var marker_len: usize = 2;
    var i = state.dqt_index;
    while (i < jpg.quant.items.len) : (i += 1) {
        const table = &jpg.quant.items[i];
        marker_len += 1 + (if (table.precision != 0) @as(usize, 2) else @as(usize, 1)) * jd.kDCTBlockSize;
        if (table.is_last) break;
    }
    try out.ensureUnusedCapacity(allocator, marker_len + 2);
    out.appendAssumeCapacity(0xFF);
    out.appendAssumeCapacity(0xDB);
    out.appendAssumeCapacity(@intCast(marker_len >> 8));
    out.appendAssumeCapacity(@truncate(marker_len));
    while (true) {
        const idx = state.dqt_index;
        state.dqt_index += 1;
        if (idx >= jpg.quant.items.len) return false;
        const table = &jpg.quant.items[idx];
        out.appendAssumeCapacity(@intCast((table.precision << 4) + table.index));
        for (0..jd.kDCTBlockSize) |qi| {
            const val_idx = jd.kJPEGNaturalOrder[qi];
            const val = table.values[val_idx];
            if (table.precision != 0) {
                out.appendAssumeCapacity(@intCast(val >> 8));
            }
            out.appendAssumeCapacity(@intCast(val & 0xFF));
        }
        if (table.is_last) break;
    }
    return true;
}

fn encodeDRI(jpg: *const JPEGData, state: *SerializationState, out: *std.ArrayList(u8), allocator: std.mem.Allocator) Error!void {
    state.seen_dri_marker = true;
    try out.appendSlice(allocator, &.{ 0xFF, 0xDD, 0, 4, @intCast(jpg.restart_interval >> 8), @intCast(jpg.restart_interval & 0xFF) });
}

fn encodeRestart(marker: u8, out: *std.ArrayList(u8), allocator: std.mem.Allocator) Error!void {
    try out.appendSlice(allocator, &.{ 0xFF, marker });
}

fn encodeAPP(jpg: *const JPEGData, state: *SerializationState, out: *std.ArrayList(u8), allocator: std.mem.Allocator) Error!bool {
    const app_index = state.app_index;
    state.app_index += 1;
    if (app_index >= jpg.app_data.items.len) return false;
    try out.append(allocator, 0xFF);
    try out.appendSlice(allocator, jpg.app_data.items[app_index].items);
    return true;
}

fn encodeCOM(jpg: *const JPEGData, state: *SerializationState, out: *std.ArrayList(u8), allocator: std.mem.Allocator) Error!bool {
    const com_index = state.com_index;
    state.com_index += 1;
    if (com_index >= jpg.com_data.items.len) return false;
    try out.append(allocator, 0xFF);
    try out.appendSlice(allocator, jpg.com_data.items[com_index].items);
    return true;
}

fn encodeInterMarkerData(jpg: *const JPEGData, state: *SerializationState, out: *std.ArrayList(u8), allocator: std.mem.Allocator) Error!bool {
    const index = state.data_index;
    state.data_index += 1;
    if (index >= jpg.inter_marker_data.items.len) return false;
    try out.appendSlice(allocator, jpg.inter_marker_data.items[index].items);
    return true;
}

const EncodeScanState = struct {
    stage_head: bool = true,
    mcu_y: i32 = 0,
    bw: JpegBitWriter = undefined,
    last_dc_coeff: [jd.kMaxComponents]coeff_t = @splat(0),
    restarts_to_go: i32 = 0,
    next_restart_marker: i32 = 0,
    block_scan_index: u32 = 0,
    coding_state: DCTCodingState = .{},
    extra_zero_runs_pos: usize = 0,
    next_extra_zero_run_index: i64 = -1,
    next_reset_point_pos: usize = 0,
    next_reset_point: i64 = -1,
};

const kModeSequential = 0;
const kModeProgressive = 1;
const kModeRefinement = 2;

fn doEncodeScan(jpg: *const JPEGData, state: *SerializationState, out: *std.ArrayList(u8), scan_index: usize, mode: u8, bw: *JpegBitWriter) Error!bool {
    const scan_info = &jpg.scan_info.items[scan_index];
    const ss = &state.scan_state;
    const restart_interval: u32 = if (state.seen_dri_marker) jpg.restart_interval else 0;

    if (ss.stage_head) {
        ss.stage_head = false;
        if (!try encodeSOS(jpg, scan_info, out, bw.allocator)) return false;
        bw.* = .{ .out = out, .allocator = bw.allocator };
        ss.coding_state.deinit(bw.allocator);
        ss.coding_state = .{};
        ss.restarts_to_go = @intCast(restart_interval);
        ss.next_restart_marker = 0;
        ss.block_scan_index = 0;
        ss.extra_zero_runs_pos = 0;
        ss.next_extra_zero_run_index = if (ss.extra_zero_runs_pos < scan_info.extra_zero_runs.items.len)
            @intCast(scan_info.extra_zero_runs.items[ss.extra_zero_runs_pos].block_idx)
        else
            -1;
        ss.next_reset_point_pos = 0;
        ss.next_reset_point = if (ss.next_reset_point_pos < scan_info.reset_points.items.len)
            @intCast(scan_info.reset_points.items[ss.next_reset_point_pos])
        else
            -1;
        ss.mcu_y = 0;
        ss.last_dc_coeff = @splat(0);
    }

    const is_interleaved = scan_info.num_components > 1;
    var mcus_per_row: i32 = 0;
    var mcu_rows: i32 = 0;
    jpg.calculateMcuSize(scan_info, &mcus_per_row, &mcu_rows);
    const is_progressive = state.is_progressive;
    const al: u5 = @intCast(if (is_progressive) scan_info.Al else 0);
    const ss_band: i32 = if (is_progressive) @intCast(scan_info.Ss) else 0;
    const se_band: i32 = if (is_progressive) @intCast(scan_info.Se) else 63;
    const want_ac = (ss_band != 0) or (se_band != 0);
    const want_dc = (ss_band == 0);

    var mcu_x: i32 = 0;
    while (ss.mcu_y < mcu_rows) : (ss.mcu_y += 1) {
        mcu_x = 0;
        while (mcu_x < mcus_per_row) : (mcu_x += 1) {
            // Possibly emit a restart marker.
            if (restart_interval > 0 and ss.restarts_to_go == 0) {
                try flushCodingState(&ss.coding_state, bw);
                if (!try jumpToByteBoundary(bw, &state.pad_bits)) return false;
                try bw.emitMarker(@intCast(0xD0 + ss.next_restart_marker));
                ss.next_restart_marker += 1;
                ss.next_restart_marker &= 0x7;
                ss.restarts_to_go = @intCast(restart_interval);
                ss.last_dc_coeff = @splat(0);
            }

            // Encode one MCU.
            for (0..scan_info.num_components) |i| {
                const si = &scan_info.components[i];
                const c = &jpg.components.items[si.comp_idx];
                const dc_huff = &state.dc_huff_table[si.dc_tbl_idx];
                const ac_huff = &state.ac_huff_table[si.ac_tbl_idx];
                if (want_dc and !dc_huff.initialized) return false;
                if (want_ac and !ac_huff.initialized) return false;
                const n_blocks_y: i32 = if (is_interleaved) c.v_samp_factor else 1;
                const n_blocks_x: i32 = if (is_interleaved) c.h_samp_factor else 1;
                var iy: i32 = 0;
                while (iy < n_blocks_y) : (iy += 1) {
                    var ix: i32 = 0;
                    while (ix < n_blocks_x) : (ix += 1) {
                        const block_y = ss.mcu_y * n_blocks_y + iy;
                        const block_x = mcu_x * n_blocks_x + ix;
                        const block_idx: usize = @intCast(@as(i64, block_y) * c.width_in_blocks + block_x);
                        if (ss.block_scan_index == ss.next_reset_point) {
                            try flushCodingState(&ss.coding_state, bw);
                            ss.next_reset_point_pos += 1;
                            ss.next_reset_point = if (ss.next_reset_point_pos < scan_info.reset_points.items.len)
                                @intCast(scan_info.reset_points.items[ss.next_reset_point_pos])
                            else
                                -1;
                        }
                        var num_zero_runs: i32 = 0;
                        if (ss.block_scan_index == ss.next_extra_zero_run_index) {
                            num_zero_runs = @intCast(scan_info.extra_zero_runs.items[ss.extra_zero_runs_pos].num_extra_zero_runs);
                            ss.extra_zero_runs_pos += 1;
                            ss.next_extra_zero_run_index = if (ss.extra_zero_runs_pos < scan_info.extra_zero_runs.items.len)
                                @intCast(scan_info.extra_zero_runs.items[ss.extra_zero_runs_pos].block_idx)
                            else
                                -1;
                        }
                        const coeffs = c.coeffs.items[block_idx * jd.kDCTBlockSize ..][0..jd.kDCTBlockSize];
                        const ok = switch (mode) {
                            kModeSequential => try encodeDCTBlockSequential(coeffs, dc_huff, ac_huff, num_zero_runs, &ss.last_dc_coeff[si.comp_idx], bw),
                            kModeProgressive => try encodeDCTBlockProgressive(coeffs, dc_huff, ac_huff, ss_band, se_band, al, num_zero_runs, &ss.coding_state, &ss.last_dc_coeff[si.comp_idx], bw),
                            else => try encodeRefinementBits(coeffs, ac_huff, ss_band, se_band, al, &ss.coding_state, bw),
                        };
                        if (!ok) return false;
                        ss.block_scan_index += 1;
                    }
                }
            }
            ss.restarts_to_go -= 1;
        }
    }
    try flushCodingState(&ss.coding_state, bw);
    if (!try jumpToByteBoundary(bw, &state.pad_bits)) return false;
    ss.stage_head = true;
    if (!bw.healthy) return false;
    return true;
}

const SerializationState = struct {
    dht_index: usize = 0,
    dqt_index: usize = 0,
    app_index: usize = 0,
    com_index: usize = 0,
    data_index: usize = 0,
    scan_index: usize = 0,
    dc_huff_table: [jd.kMaxHuffmanTables]HuffmanCodeTable = @splat(.{}),
    ac_huff_table: [jd.kMaxHuffmanTables]HuffmanCodeTable = @splat(.{}),
    pad_bits: PadBits = .{},
    seen_dri_marker: bool = false,
    is_progressive: bool = false,
    scan_state: EncodeScanState = .{ .stage_head = true },
};

/// Serializes a JPEGData back into a complete JPEG byte stream.
pub fn writeJpeg(jpg: *const JPEGData, out: *std.ArrayList(u8), allocator: std.mem.Allocator) Error!void {
    if (jpg.marker_order.items.len == 0) return Error.InvalidJpegData;
    var state = SerializationState{};
    if (jpg.has_zero_padding_bit) {
        state.pad_bits = .{ .bits = jpg.padding_bits.items };
    }

    try encodeSOI(out, allocator);
    for (jpg.marker_order.items) |marker| {
        const ok = switch (marker) {
            0xC0, 0xC1, 0xC2, 0xC9, 0xCA => blk: {
                if (marker <= 0xC2) state.is_progressive = (marker == 0xC2);
                break :blk try encodeSOF(jpg, marker, out, allocator);
            },
            0xC4 => try encodeDHT(jpg, &state, out, allocator),
            0xD0...0xD7 => blk: {
                try encodeRestart(marker, out, allocator);
                break :blk true;
            },
            0xD9 => blk: {
                try encodeEOI(jpg, out, allocator);
                break :blk true;
            },
            0xDA => blk: {
                const scan_info = &jpg.scan_info.items[state.scan_index];
                const is_progressive = state.is_progressive;
                const ah: u5 = @intCast(if (is_progressive) scan_info.Ah else 0);
                const ss: i32 = if (is_progressive) @intCast(scan_info.Ss) else 0;
                const se: i32 = if (is_progressive) @intCast(scan_info.Se) else 63;
                const need_sequential = !is_progressive or (ah == 0 and scan_info.Al == 0 and ss == 0 and se == 63);
                var bw = JpegBitWriter{ .out = out, .allocator = allocator };
                const mode: u8 = if (need_sequential) kModeSequential else if (ah == 0) kModeProgressive else kModeRefinement;
                break :blk try doEncodeScan(jpg, &state, out, state.scan_index, mode, &bw);
            },
            0xDB => try encodeDQT(jpg, &state, out, allocator),
            0xDD => blk: {
                try encodeDRI(jpg, &state, out, allocator);
                break :blk true;
            },
            0xE0...0xEF => try encodeAPP(jpg, &state, out, allocator),
            0xFE => try encodeCOM(jpg, &state, out, allocator),
            0xFF => try encodeInterMarkerData(jpg, &state, out, allocator),
            else => false,
        };
        if (!ok) return Error.InvalidJpegData;
        if (marker == 0xDA) state.scan_index += 1;
    }
    if (state.pad_bits.bits.len > 0 and state.pad_bits.pos != state.pad_bits.bits.len) {
        state.scan_state.coding_state.deinit(allocator);
        return Error.InvalidJpegData;
    }
    state.scan_state.coding_state.deinit(allocator);
}
