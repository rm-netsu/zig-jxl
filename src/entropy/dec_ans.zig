//! ANS/prefix histogram decoding and the ANS symbol reader.
//! Bit-exact port of libjxl `dec_ans.h/cc`, `dec_context_map.h/cc` and the
//! inverse MTF transform (`inverse_mtf-inl.h`).

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const bits = @import("../base/bits.zig");
const ans_common = @import("ans_common.zig");
const fields = @import("../base/fields.zig");
const dec_huffman = @import("dec_huffman.zig");

const BitReader = bit_io.BitReader;
const HybridUintConfig = ans_common.HybridUintConfig;
const AliasEntry = ans_common.AliasEntry;
const aliasLookup = ans_common.aliasLookup;
const LZ77Params = fields.LZ77Params;
const HuffmanDecodingData = dec_huffman.HuffmanDecodingData;

pub const DecodingError = error{
    InvalidData,
    NotEnoughBytes,
    OutOfMemory,
};

// ---------------------------------------------------------------------------
// Special distance codes (from WebP lossless), dec_ans.h
// ---------------------------------------------------------------------------

pub const k_window_size: usize = 1 << 20;
pub const k_num_special_distances: usize = 120;

pub const k_special_distances = [k_num_special_distances][2]i8{
    .{ 0, 1 },  .{ 1, 0 },  .{ 1, 1 },  .{ -1, 1 }, .{ 0, 2 },  .{ 2, 0 },  .{ 1, 2 },  .{ -1, 2 },
    .{ 2, 1 },  .{ -2, 1 }, .{ 2, 2 },  .{ -2, 2 }, .{ 0, 3 },  .{ 3, 0 },  .{ 1, 3 },  .{ -1, 3 },
    .{ 3, 1 },  .{ -3, 1 }, .{ 2, 3 },  .{ -2, 3 }, .{ 3, 2 },  .{ -3, 2 }, .{ 0, 4 },  .{ 4, 0 },
    .{ 1, 4 },  .{ -1, 4 }, .{ 4, 1 },  .{ -4, 1 }, .{ 3, 3 },  .{ -3, 3 }, .{ 2, 4 },  .{ -2, 4 },
    .{ 4, 2 },  .{ -4, 2 }, .{ 0, 5 },  .{ 3, 4 },  .{ -3, 4 }, .{ 4, 3 },  .{ -4, 3 }, .{ 5, 0 },
    .{ 1, 5 },  .{ -1, 5 }, .{ 5, 1 },  .{ -5, 1 }, .{ 2, 5 },  .{ -2, 5 }, .{ 5, 2 },  .{ -5, 2 },
    .{ 4, 4 },  .{ -4, 4 }, .{ 3, 5 },  .{ -3, 5 }, .{ 5, 3 },  .{ -5, 3 }, .{ 0, 6 },  .{ 6, 0 },
    .{ 1, 6 },  .{ -1, 6 }, .{ 6, 1 },  .{ -6, 1 }, .{ 2, 6 },  .{ -2, 6 }, .{ 6, 2 },  .{ -6, 2 },
    .{ 4, 5 },  .{ -4, 5 }, .{ 5, 4 },  .{ -5, 4 }, .{ 3, 6 },  .{ -3, 6 }, .{ 6, 3 },  .{ -6, 3 },
    .{ 0, 7 },  .{ 7, 0 },  .{ 1, 7 },  .{ -1, 7 }, .{ 5, 5 },  .{ -5, 5 }, .{ 7, 1 },  .{ -7, 1 },
    .{ 4, 6 },  .{ -4, 6 }, .{ 6, 4 },  .{ -6, 4 }, .{ 2, 7 },  .{ -2, 7 }, .{ 7, 2 },  .{ -7, 2 },
    .{ 3, 7 },  .{ -3, 7 }, .{ 7, 3 },  .{ -7, 3 }, .{ 5, 6 },  .{ -5, 6 }, .{ 6, 5 },  .{ -6, 5 },
    .{ 8, 0 },  .{ 4, 7 },  .{ -4, 7 }, .{ 7, 4 },  .{ -7, 4 }, .{ 8, 1 },  .{ 8, 2 },  .{ 6, 6 },
    .{ -6, 6 }, .{ 8, 3 },  .{ 5, 7 },  .{ -5, 7 }, .{ 7, 5 },  .{ -7, 5 }, .{ 8, 4 },  .{ 6, 7 },
    .{ -6, 7 }, .{ 7, 6 },  .{ -7, 6 }, .{ 8, 5 },  .{ 7, 7 },  .{ -7, 7 }, .{ 8, 6 },  .{ 8, 7 },
};

pub fn specialDistance(index: usize, multiplier: u32) u32 {
    const dist = @as(i32, k_special_distances[index][0]) +
        @as(i32, @intCast(multiplier)) * @as(i32, k_special_distances[index][1]);
    return @intCast(@max(dist, 1));
}

// ---------------------------------------------------------------------------
// ANSCode
// ---------------------------------------------------------------------------

pub const ANSCode = struct {
    /// Flat array of alias tables; histogram `c` uses the slice
    /// `alias_tables[c << log_alpha_size ..][0 .. 1 << log_alpha_size]`.
    /// Only used when `use_prefix_code == false`.
    alias_tables: []AliasEntry = &.{},
    /// Decoding data for prefix codes; only used when `use_prefix_code`.
    huffman_data: []HuffmanDecodingData = &.{},
    uint_config: []HybridUintConfig = &.{},
    degenerate_symbols: []i32 = &.{},
    use_prefix_code: bool = false,
    /// For ANS. Range [5, 8].
    log_alpha_size: u8 = 0,
    lz77: LZ77Params = .{},
    /// Maximum number of bits necessary to represent the result of a
    /// readHybridUint call done with this ANSCode.
    max_num_bits: usize = 0,

    pub fn deinit(self: *ANSCode, allocator: std.mem.Allocator) void {
        if (self.alias_tables.len > 0) allocator.free(self.alias_tables);
        for (self.huffman_data) |*h| {
            h.deinit(allocator);
        }
        if (self.huffman_data.len > 0) allocator.free(self.huffman_data);
        if (self.uint_config.len > 0) allocator.free(self.uint_config);
        if (self.degenerate_symbols.len > 0) allocator.free(self.degenerate_symbols);
        self.* = .{};
    }

    pub fn updateMaxNumBits(self: *ANSCode, ctx: usize, symbol_in: usize) void {
        var cfg = &self.uint_config[ctx];
        var symbol = symbol_in;
        // LZ77 symbols use a different uint config.
        if (self.lz77.enabled and self.lz77.nonserialized_distance_context != ctx and
            symbol >= self.lz77.min_symbol)
        {
            symbol -= self.lz77.min_symbol;
            cfg = &self.lz77.length_uint_config;
        }
        const split_token: usize = cfg.split_token;
        const msb_in_token: usize = cfg.msb_in_token;
        const lsb_in_token: usize = cfg.lsb_in_token;
        const split_exponent: usize = cfg.split_exponent;
        if (symbol < split_token) {
            self.max_num_bits = @max(self.max_num_bits, split_exponent);
            return;
        }
        const n_extra_bits: usize = split_exponent - (msb_in_token + lsb_in_token) +
            ((symbol - split_token) >> @intCast(msb_in_token + lsb_in_token));
        const total_bits: usize = msb_in_token + lsb_in_token + n_extra_bits + 1;
        self.max_num_bits = @max(self.max_num_bits, total_bits);
    }
};

// ---------------------------------------------------------------------------
// ReadHistogram (dec_ans.cc, anonymous namespace)
// ---------------------------------------------------------------------------

/// Port of `jxl::ReadHistogram`: reads one histogram of ANS counts into
/// `counts` (which is resized as needed). `precision_bits` is 12 for ANS.
pub fn readHistogram(allocator: std.mem.Allocator, precision_bits: u5, counts: *std.ArrayList(ans_common.ANSHistBin), input: *BitReader) !void {
    var counts_list = counts;
    const range: i32 = @as(i32, 1) << precision_bits;
    const simple_code = input.readBits(1);
    if (simple_code == 1) {
        var symbols = [2]u32{ 0, 0 };
        var max_symbol: u32 = 0;
        const num_symbols: usize = @as(usize, @intCast(input.readBits(1))) + 1;
        for (0..num_symbols) |i| {
            symbols[i] = fields.decodeVarLenUint8(input);
            if (symbols[i] > max_symbol) max_symbol = symbols[i];
        }
        try counts_list.resize(allocator, max_symbol + 1);
        @memset(counts_list.items, 0);
        if (num_symbols == 1) {
            counts_list.items[symbols[0]] = range;
        } else {
            if (symbols[0] == symbols[1]) { // corrupt data
                return error.InvalidData;
            }
            counts_list.items[symbols[0]] = @intCast(input.readBits(precision_bits));
            counts_list.items[symbols[1]] = range - counts_list.items[symbols[0]];
        }
    } else {
        const is_flat = input.readBits(1);
        if (is_flat == 1) {
            const alphabet_size: usize = @as(usize, fields.decodeVarLenUint8(input)) + 1;
            if (alphabet_size > range) return error.InvalidData;
            const flat = try ans_common.createFlatHistogram(allocator, @intCast(alphabet_size), range);
            defer allocator.free(flat);
            try counts_list.resize(allocator, flat.len);
            @memcpy(counts_list.items, flat);
            return;
        }

        const shift: u32 = blk: {
            // Elias gamma-like code for the shift. upper_bound_log is
            // FloorLog2Nonzero(ANS_LOG_TAB_SIZE + 1) == 3.
            const upper_bound_log = bits.floorLog2Nonzero(ans_common.ans_log_tab_size + 1);
            var log: u32 = 0;
            while (log < upper_bound_log) : (log += 1) {
                if (input.readFixedBits(1) == 0) break;
            }
            const s = (@as(u32, @intCast(input.readBits(log))) | (@as(u32, 1) << @intCast(log))) - 1;
            if (s > ans_common.ans_log_tab_size + 1) {
                return error.InvalidData; // Invalid shift value
            }
            break :blk s;
        };

        const length: usize = @as(usize, fields.decodeVarLenUint8(input)) + 3;
        try counts_list.resize(allocator, length);
        @memset(counts_list.items, 0);
        var total_count: i32 = 0;

        const huff = [128][2]u8{
            .{ 3, 10 }, .{ 7, 12 }, .{ 3, 7 },  .{ 4, 3 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 5 },
            .{ 3, 10 }, .{ 4, 4 },  .{ 3, 7 },  .{ 4, 1 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 2 },
            .{ 3, 10 }, .{ 5, 0 },  .{ 3, 7 },  .{ 4, 3 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 5 },
            .{ 3, 10 }, .{ 4, 4 },  .{ 3, 7 },  .{ 4, 1 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 2 },
            .{ 3, 10 }, .{ 6, 11 }, .{ 3, 7 },  .{ 4, 3 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 5 },
            .{ 3, 10 }, .{ 4, 4 },  .{ 3, 7 },  .{ 4, 1 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 2 },
            .{ 3, 10 }, .{ 5, 0 },  .{ 3, 7 },  .{ 4, 3 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 5 },
            .{ 3, 10 }, .{ 4, 4 },  .{ 3, 7 },  .{ 4, 1 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 2 },
            .{ 3, 10 }, .{ 7, 13 }, .{ 3, 7 },  .{ 4, 3 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 5 },
            .{ 3, 10 }, .{ 4, 4 },  .{ 3, 7 },  .{ 4, 1 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 2 },
            .{ 3, 10 }, .{ 5, 0 },  .{ 3, 7 },  .{ 4, 3 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 5 },
            .{ 3, 10 }, .{ 4, 4 },  .{ 3, 7 },  .{ 4, 1 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 2 },
            .{ 3, 10 }, .{ 6, 11 }, .{ 3, 7 },  .{ 4, 3 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 5 },
            .{ 3, 10 }, .{ 4, 4 },  .{ 3, 7 },  .{ 4, 1 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 2 },
            .{ 3, 10 }, .{ 5, 0 },  .{ 3, 7 },  .{ 4, 3 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 5 },
            .{ 3, 10 }, .{ 4, 4 },  .{ 3, 7 },  .{ 4, 1 },  .{ 3, 6 },  .{ 3, 8 },  .{ 3, 9 },  .{ 4, 2 },
        };

        const logcounts = try allocator.alloc(i32, length);
        defer allocator.free(logcounts);
        @memset(logcounts, 0);
        var omit_log: i32 = -1;
        var omit_pos: i64 = -1;
        // This array remembers which symbols have an RLE length.
        const same = try allocator.alloc(i32, length);
        defer allocator.free(same);
        @memset(same, 0);
        {
            var i: usize = 0;
            while (i < length) : (i += 1) {
                input.refill(); // for peekFixedBits + consume
                const idx: usize = @intCast(input.peekFixedBits(7));
                input.consume(huff[idx][0]);
                logcounts[i] = @as(i32, huff[idx][1]) - 1;
                // The RLE symbol.
                if (logcounts[i] == ans_common.ans_log_tab_size) {
                    const rle_length = fields.decodeVarLenUint8(input);
                    same[i] = @intCast(rle_length + 5);
                    i += rle_length + 3;
                    continue;
                }
                if (logcounts[i] > omit_log) {
                    omit_log = logcounts[i];
                    omit_pos = @intCast(i);
                }
            }
        }
        // Invalid input, e.g. due to invalid usage of RLE.
        if (omit_pos < 0) return error.InvalidData;
        if (@as(usize, @intCast(omit_pos)) + 1 < length and
            logcounts[@as(usize, @intCast(omit_pos)) + 1] == ans_common.ans_log_tab_size)
        {
            return error.InvalidData;
        }
        var prev: i32 = 0;
        var numsame: i32 = 0;
        for (0..length) |i| {
            if (same[i] != 0) {
                // RLE sequence: output the same count for the next iterations.
                numsame = same[i] - 1;
                prev = if (i > 0) counts_list.items[i - 1] else 0;
            }
            if (numsame > 0) {
                counts_list.items[i] = prev;
                numsame -= 1;
            } else {
                const code = logcounts[i];
                // omit_pos may not be negative at this point (checked before).
                if (i == @as(usize, @intCast(omit_pos)) or code < 0) {
                    continue;
                } else if (shift == 0 or code == 0) {
                    // `shift = 0` means `bitcount = 0`
                    counts_list.items[i] = @as(i32, 1) << @intCast(code);
                } else {
                    const bitcount = ans_common.getPopulationCountPrecision(@intCast(code), shift);
                    counts_list.items[i] = (@as(i32, 1) << @intCast(code)) +
                        (@as(i32, @intCast(input.readBits(bitcount))) << @intCast(code - @as(i32, @intCast(bitcount))));
                }
            }
            total_count += counts_list.items[i];
        }
        counts_list.items[@intCast(omit_pos)] = range - total_count;
        if (counts_list.items[@intCast(omit_pos)] <= 0) {
            // The histogram we've read sums to more than total_count
            // (including at least 1 for the omitted value).
            return error.InvalidData;
        }
    }
}

// ---------------------------------------------------------------------------
// DecodeANSCodes / DecodeUintConfigs / DecodeHistograms (dec_ans.cc)
// ---------------------------------------------------------------------------

/// Port of `jxl::DecodeANSCodes`.
pub fn decodeANSCodes(
    allocator: std.mem.Allocator,
    num_histograms: usize,
    max_alphabet_size: usize,
    in: *BitReader,
    result: *ANSCode,
) ans_common.DecodeError!void {
    result.degenerate_symbols = try allocator.alloc(i32, num_histograms);
    @memset(result.degenerate_symbols, -1);
    errdefer allocator.free(result.degenerate_symbols);
    if (result.use_prefix_code) {
        if (max_alphabet_size > (@as(usize, 1) << ans_common.prefix_max_bits)) return error.InvalidData;
        result.huffman_data = try allocator.alloc(HuffmanDecodingData, num_histograms);
        for (result.huffman_data) |*h| h.* = .{};
        errdefer {
            for (result.huffman_data) |*h| h.deinit(allocator);
            allocator.free(result.huffman_data);
        }
        const alphabet_sizes = try allocator.alloc(u16, num_histograms);
        defer allocator.free(alphabet_sizes);
        for (0..num_histograms) |c| {
            alphabet_sizes[c] = @intCast(fields.decodeVarLenUint16(in) + 1);
            if (alphabet_sizes[c] > max_alphabet_size) {
                return error.InvalidData; // Alphabet size is too long
            }
        }
        for (0..num_histograms) |c| {
            if (alphabet_sizes[c] > 1) {
                try result.huffman_data[c].readFromBitStream(allocator, alphabet_sizes[c], in);
            } else {
                // 0-bit codes do not require extension tables.
                result.huffman_data[c].deinit(allocator);
                result.huffman_data[c].table = try allocator.alloc(dec_huffman.HuffmanCode, @as(usize, 1) << dec_huffman.k_huffman_table_bits);
                @memset(result.huffman_data[c].table, .{});
            }
            for (result.huffman_data[c].table) |h| {
                if (h.bits <= dec_huffman.k_huffman_table_bits) {
                    result.updateMaxNumBits(c, h.value);
                }
            }
        }
    } else {
        if (max_alphabet_size > ans_common.ans_max_alphabet_size) return error.InvalidData;
        const alloc_size = num_histograms * (@as(usize, 1) << @intCast(result.log_alpha_size));
        result.alias_tables = try allocator.alloc(AliasEntry, alloc_size);
        errdefer allocator.free(result.alias_tables);
        const alias_tables = result.alias_tables;
        for (0..num_histograms) |c| {
            var counts: std.ArrayList(ans_common.ANSHistBin) = .empty;
            defer counts.deinit(allocator);
            try readHistogram(allocator, @intCast(ans_common.ans_log_tab_size), &counts, in);
            if (counts.items.len > max_alphabet_size) {
                return error.InvalidData; // Alphabet size is too long
            }
            while (counts.items.len > 0 and counts.items[counts.items.len - 1] == 0) {
                _ = counts.pop();
            }
            for (0..counts.items.len) |s| {
                if (counts.items[s] != 0) {
                    result.updateMaxNumBits(c, s);
                }
            }
            // initAliasTable "fixes" empty counts to contain a degenerate
            // "0" symbol.
            var degenerate_symbol: i32 = if (counts.items.len == 0) 0 else @as(i32, @intCast(counts.items.len)) - 1;
            var s: i32 = 0;
            while (s < degenerate_symbol) : (s += 1) {
                if (counts.items[@intCast(s)] != 0) {
                    degenerate_symbol = -1;
                    break;
                }
            }
            result.degenerate_symbols[c] = degenerate_symbol;
            const table_slice = alias_tables[c * (@as(usize, 1) << @intCast(result.log_alpha_size))..];
            try ans_common.initAliasTable(allocator, counts.items, ans_common.ans_log_tab_size, @intCast(result.log_alpha_size), table_slice[0 .. @as(usize, 1) << @intCast(result.log_alpha_size)]);
        }
    }
}

/// Port of `jxl::DecodeUintConfig`.
pub fn decodeUintConfig(log_alpha_size: usize, uint_config: *HybridUintConfig, br: *BitReader) !void {
    br.refill();
    const split_exponent: u32 = @intCast(br.readBits(bits.ceilLog2Nonzero(log_alpha_size + 1)));
    var msb_in_token: u32 = 0;
    var lsb_in_token: u32 = 0;
    if (split_exponent != log_alpha_size) {
        // otherwise, msb/lsb don't matter.
        var nbits = bits.ceilLog2Nonzero(split_exponent + 1);
        msb_in_token = @intCast(br.readBits(nbits));
        if (msb_in_token > split_exponent) {
            return error.InvalidData;
        }
        nbits = bits.ceilLog2Nonzero(split_exponent - msb_in_token + 1);
        lsb_in_token = @intCast(br.readBits(nbits));
    }
    if (lsb_in_token + msb_in_token > split_exponent) {
        return error.InvalidData;
    }
    uint_config.* = HybridUintConfig.init(split_exponent, msb_in_token, lsb_in_token);
}

/// Port of `jxl::DecodeUintConfigs`.
pub fn decodeUintConfigs(log_alpha_size: usize, uint_config: []HybridUintConfig, br: *BitReader) !void {
    for (uint_config) |*cfg| {
        try decodeUintConfig(log_alpha_size, cfg, br);
    }
}

/// Port of `jxl::DecodeHistograms`. Reads the ANS histograms header for
/// `num_contexts` contexts (including the context map) into `code` and
/// `context_map`.
pub fn decodeHistograms(
    allocator: std.mem.Allocator,
    br: *BitReader,
    num_contexts_in: usize,
    code: *ANSCode,
    context_map: *std.ArrayList(u8),
    disallow_lz77: bool,
) ans_common.DecodeError!void {
    var num_contexts = num_contexts_in;
    try code.lz77.read(br);
    if (code.lz77.enabled) {
        num_contexts += 1;
        try decodeUintConfig(8, &code.lz77.length_uint_config, br);
    }
    if (code.lz77.enabled and disallow_lz77) {
        return error.InvalidData; // Using LZ77 when explicitly disallowed
    }
    var num_histograms: usize = 1;
    try context_map.resize(allocator, num_contexts);
    @memset(context_map.items, 0);
    if (num_contexts > 1) {
        try decodeContextMap(allocator, context_map, &num_histograms, br);
    }
    code.lz77.nonserialized_distance_context = context_map.items[context_map.items.len - 1];
    code.use_prefix_code = (br.readFixedBits(1) != 0);
    if (code.use_prefix_code) {
        code.log_alpha_size = @intCast(ans_common.prefix_max_bits);
    } else {
        code.log_alpha_size = @intCast(br.readFixedBits(2) + 5);
    }
    code.uint_config = try allocator.alloc(HybridUintConfig, num_histograms);
    @memset(code.uint_config, HybridUintConfig.default);
    try decodeUintConfigs(code.log_alpha_size, code.uint_config, br);
    const max_alphabet_size = @as(usize, 1) << @intCast(code.log_alpha_size);
    try decodeANSCodes(allocator, num_histograms, max_alphabet_size, br, code);
}

// ---------------------------------------------------------------------------
// Context map (dec_context_map.cc)
// ---------------------------------------------------------------------------

/// Context map uses uint8_t.
const k_max_clusters = 256;

fn verifyContextMap(allocator: std.mem.Allocator, context_map: []const u8, num_htrees: usize) !void {
    const have_htree = try allocator.alloc(bool, num_htrees);
    defer allocator.free(have_htree);
    @memset(have_htree, false);
    var num_found: usize = 0;
    for (context_map) |htree| {
        if (htree >= num_htrees) {
            return error.InvalidData; // Invalid histogram index in context map
        }
        if (!have_htree[htree]) {
            have_htree[htree] = true;
            num_found += 1;
        }
    }
    if (num_found != num_htrees) {
        return error.InvalidData; // Incomplete context map
    }
}

/// Scalar port of `InverseMoveToFrontTransform` (inverse_mtf-inl.h).
pub fn inverseMoveToFrontTransform(v: []u8) void {
    var mtf: [256]u8 = undefined;
    for (0..256) |i| {
        mtf[i] = @intCast(i);
    }
    for (v) |*slot| {
        const index = slot.*;
        slot.* = mtf[index];
        if (index != 0) {
            // MoveToFront
            var i: u8 = index;
            const value = mtf[index];
            while (i != 0) : (i -= 1) {
                mtf[i] = mtf[i - 1];
            }
            mtf[0] = value;
        }
    }
}

/// Port of `jxl::DecodeContextMap`.
pub fn decodeContextMap(
    allocator: std.mem.Allocator,
    context_map: *std.ArrayList(u8),
    num_htrees: *usize,
    input: *BitReader,
) ans_common.DecodeError!void {
    const is_simple = (input.readFixedBits(1) != 0);
    if (is_simple) {
        const bits_per_entry: usize = @intCast(input.readFixedBits(2));
        if (bits_per_entry != 0) {
            for (context_map.items) |*entry| {
                entry.* = @intCast(input.readBits(bits_per_entry));
            }
        } else {
            @memset(context_map.items, 0);
        }
    } else {
        const use_mtf = (input.readFixedBits(1) != 0);
        var code = ANSCode{};
        defer code.deinit(allocator);
        var sink_ctx_map: std.ArrayList(u8) = .empty;
        defer sink_ctx_map.deinit(allocator);
        // Usage of LZ77 is disallowed if decoding only two symbols.
        try decodeHistograms(allocator, input, 1, &code, &sink_ctx_map, context_map.items.len <= 2);
        var reader = try ANSSymbolReader.create(allocator, &code, input, 0);
        defer reader.deinit();
        var i: usize = 0;
        var maxsym: u32 = 0;
        while (i < context_map.items.len) : (i += 1) {
            const sym = reader.readHybridUintInlined(true, 0, input, sink_ctx_map.items);
            maxsym = @max(sym, maxsym);
            context_map.items[i] = @intCast(sym);
        }
        if (maxsym >= k_max_clusters) {
            return error.InvalidData; // Invalid cluster ID
        }
        if (!reader.checkANSFinalState()) {
            return error.InvalidData; // Invalid context map
        }
        if (use_mtf) {
            inverseMoveToFrontTransform(context_map.items);
        }
    }
    num_htrees.* = @as(usize, std.mem.max(u8, context_map.items)) + 1;
    try verifyContextMap(allocator, context_map.items, num_htrees.*);
}

// ---------------------------------------------------------------------------
// ANSSymbolReader (dec_ans.h)
// ---------------------------------------------------------------------------

pub const ANSSymbolReader = struct {
    alias_tables: []const AliasEntry, // not owned
    huffman_data: []const HuffmanDecodingData, // not owned
    use_prefix_code: bool,
    state: u32 = ans_common.ans_final_state,
    configs: []const HybridUintConfig, // not owned
    log_alpha_size: u32 = 0,
    log_entry_size: u32 = 0,
    entry_size_minus_1: u32 = 0,

    // LZ77 structures and constants.
    allocator: ?std.mem.Allocator = null,
    lz77_window: ?[]u32 = null,
    num_decoded: u32 = 0,
    num_to_copy: u32 = 0,
    copy_pos: u32 = 0,
    lz77_ctx: u32 = 0,
    lz77_min_length: u32 = 0,
    lz77_threshold: u32 = 1 << 20, // bigger than any symbol
    lz77_length_uint: HybridUintConfig = .{ .split_exponent = 0, .split_token = 1, .msb_in_token = 0, .lsb_in_token = 0 },
    special_distances: [k_num_special_distances]u32 = [_]u32{0} ** k_num_special_distances,
    num_special_distances: u32 = 0,

    pub const k_window_mask: usize = k_window_size - 1;

    /// Port of `ANSSymbolReader::Create` + private ctor.
    pub fn create(
        allocator: std.mem.Allocator,
        code: *const ANSCode,
        br: *BitReader,
        distance_multiplier: u32,
    ) !ANSSymbolReader {
        var self = ANSSymbolReader{
            .alias_tables = code.alias_tables,
            .huffman_data = code.huffman_data,
            .use_prefix_code = code.use_prefix_code,
            .configs = code.uint_config,
            .allocator = allocator,
        };
        if (!code.use_prefix_code) {
            self.state = @intCast(br.readFixedBits(32));
            self.log_alpha_size = code.log_alpha_size;
            self.log_entry_size = ans_common.ans_log_tab_size - code.log_alpha_size;
            self.entry_size_minus_1 = (@as(u32, 1) << @intCast(self.log_entry_size)) - 1;
        } else {
            self.state = ans_common.ans_final_state;
        }
        if (!code.lz77.enabled) return self;
        const window = try allocator.alloc(u32, k_window_size);
        @memset(window, 0);
        self.lz77_window = window;
        self.lz77_ctx = code.lz77.nonserialized_distance_context;
        self.lz77_length_uint = code.lz77.length_uint_config;
        self.lz77_threshold = code.lz77.min_symbol;
        self.lz77_min_length = code.lz77.min_length;
        self.num_special_distances = if (distance_multiplier == 0) 0 else k_num_special_distances;
        for (0..self.num_special_distances) |i| {
            self.special_distances[i] = specialDistance(i, distance_multiplier);
        }
        return self;
    }

    pub fn deinit(self: *ANSSymbolReader) void {
        if (self.lz77_window) |w| {
            if (self.allocator) |a| a.free(w);
        }
        self.lz77_window = null;
    }

    pub fn readSymbolANSWithoutRefill(self: *ANSSymbolReader, histo_idx: usize, br: *BitReader) usize {
        const res: u32 = self.state & (ans_common.ans_tab_size - 1);

        const table = self.alias_tables[histo_idx << @intCast(self.log_alpha_size)..];
        const symbol = aliasLookup(table, res, @intCast(self.log_entry_size), self.entry_size_minus_1);
        // Computed in 64-bit; result is guaranteed < 2^32.
        self.state = @truncate(@as(u64, symbol.freq) * (@as(u64, self.state) >> ans_common.ans_log_tab_size) + symbol.offset);

        // Renormalize (branchless version in libjxl; equivalent here).
        const new_state = (self.state << 16) | @as(u32, @intCast(br.peekFixedBits(16)));
        const normalize = self.state < (@as(u32, 1) << 16);
        if (normalize) {
            self.state = new_state;
            br.consume(16);
        }

        return symbol.value;
    }

    pub fn readSymbolHuffWithoutRefill(self: *ANSSymbolReader, histo_idx: usize, br: *BitReader) usize {
        return self.huffman_data[histo_idx].readSymbol(br);
    }

    pub fn readSymbolWithoutRefill(self: *ANSSymbolReader, histo_idx: usize, br: *BitReader) usize {
        if (self.use_prefix_code) {
            return self.readSymbolHuffWithoutRefill(histo_idx, br);
        }
        return self.readSymbolANSWithoutRefill(histo_idx, br);
    }

    pub fn readSymbol(self: *ANSSymbolReader, histo_idx: usize, br: *BitReader) usize {
        br.refill();
        return self.readSymbolWithoutRefill(histo_idx, br);
    }

    pub fn checkANSFinalState(self: *const ANSSymbolReader) bool {
        return self.state == ans_common.ans_final_state;
    }

    /// Fast-track hybrid integer decoding
    /// (libjxl: ANSSymbolReader::ReadHybridUintConfig).
    pub fn readHybridUintConfigStatic(config: *const HybridUintConfig, token_in: usize, br: *BitReader) u32 {
        const split_token: usize = config.split_token;
        const msb_in_token: usize = config.msb_in_token;
        const lsb_in_token: usize = config.lsb_in_token;
        const split_exponent: usize = config.split_exponent;
        // Fast-track version of hybrid integer decoding.
        if (token_in < split_token) return @intCast(token_in);
        var nbits: u32 = @intCast(split_exponent - (msb_in_token + lsb_in_token) +
            ((token_in - split_token) >> @intCast(msb_in_token + lsb_in_token)));
        // Max amount of bits for readBits is 32; if nbits > 29, the code
        // stream is invalid, but no error is propagated here (like libjxl).
        nbits &= 31;
        const low: u32 = @intCast(token_in & ((@as(usize, 1) << @intCast(lsb_in_token)) - 1));
        const token = token_in >> @intCast(lsb_in_token);
        const bits_v: u32 = @intCast(br.peekBits(nbits));
        br.consume(nbits);
        const ret: u64 = (((((@as(u64, 1) << @intCast(msb_in_token)) | (token & ((@as(usize, 1) << @intCast(msb_in_token)) - 1)))
            << @intCast(nbits)) | bits_v) << @intCast(lsb_in_token)) | low;
        return @truncate(ret);
    }

    /// Takes a *clustered* idx. Can only use if isHuffRleOnly() is true.
    pub fn readHybridUintClusteredHuffRleOnly(
        self: *ANSSymbolReader,
        ctx: usize,
        br: *BitReader,
        value: *u32,
        run: *u32,
    ) void {
        br.refill(); // covers readSymbolWithoutRefill + peekBits
        const token = self.readSymbolHuffWithoutRefill(ctx, br);
        if (token >= self.lz77_threshold) {
            run.* = readHybridUintConfigStatic(&self.lz77_length_uint, token - self.lz77_threshold, br) +
                self.lz77_min_length - 1;
            return;
        }
        value.* = readHybridUintConfigStatic(&self.configs[ctx], token, br);
    }

    pub fn isHuffRleOnly(self: *const ANSSymbolReader) bool {
        if (self.lz77_window == null) return false;
        if (!self.use_prefix_code) return false;
        for (0..dec_huffman.k_huffman_table_bits) |i| {
            if (self.huffman_data[self.lz77_ctx].table[i].bits != 0) return false;
            if (self.huffman_data[self.lz77_ctx].table[i].value != 1) return false;
        }
        if (self.configs[self.lz77_ctx].split_token > 1) return false;
        return true;
    }

    pub fn usesLZ77(self: *const ANSSymbolReader) bool {
        return self.lz77_window != null;
    }

    /// Takes a *clustered* context index (libjxl:
    /// ReadHybridUintClusteredInlined<uses_lz77>).
    pub fn readHybridUintClusteredInlined(
        self: *ANSSymbolReader,
        comptime uses_lz77: bool,
        ctx: usize,
        br: *BitReader,
        context_map: []const u8,
    ) u32 {
        // Note: libjxl takes `ctx` as already-clustered index; the
        // non-inlined variants map through the context map first.
        _ = context_map;
        if (uses_lz77) {
            if (self.num_to_copy > 0) {
                const ret = self.lz77_window.?[(self.copy_pos) & k_window_mask];
                self.copy_pos +%= 1;
                self.num_to_copy -= 1;
                self.lz77_window.?[(self.num_decoded) & k_window_mask] = ret;
                self.num_decoded +%= 1;
                return ret;
            }
        }

        br.refill(); // covers readSymbolWithoutRefill + peekBits
        const token = self.readSymbolWithoutRefill(ctx, br);
        if (uses_lz77) {
            if (token >= self.lz77_threshold) {
                self.num_to_copy = readHybridUintConfigStatic(&self.lz77_length_uint, token - self.lz77_threshold, br) +
                    self.lz77_min_length;
                br.refill(); // covers readSymbolWithoutRefill + peekBits
                // Distance code.
                const d_token = self.readSymbolWithoutRefill(self.lz77_ctx, br);
                var distance = readHybridUintConfigStatic(&self.configs[self.lz77_ctx], d_token, br);
                if (distance < self.num_special_distances) {
                    distance = self.special_distances[distance];
                } else {
                    distance = distance + 1 - self.num_special_distances;
                }
                if (distance > self.num_decoded) {
                    distance = self.num_decoded;
                }
                if (distance > k_window_size) {
                    distance = @intCast(k_window_size);
                }
                self.copy_pos = self.num_decoded - distance;
                if (distance == 0) {
                    // distance 0 -> num_decoded == copy_pos == 0
                    const to_fill = @min(self.num_to_copy, @as(u32, @intCast(k_window_size)));
                    @memset(self.lz77_window.?[0..to_fill], 0);
                }
                if (self.num_to_copy < self.lz77_min_length) {
                    // num_to_copy wrapped; stop accepting further data so
                    // the corruption surfaces at the next bounds check.
                    self.num_to_copy = 0;
                    br.markUnhealthy();
                    return 0;
                }
                const ret = self.lz77_window.?[(self.copy_pos) & k_window_mask];
                self.copy_pos +%= 1;
                self.num_to_copy -= 1;
                self.lz77_window.?[(self.num_decoded) & k_window_mask] = ret;
                self.num_decoded +%= 1;
                return ret;
            }
        }
        const ret = readHybridUintConfigStatic(&self.configs[ctx], token, br);
        if (uses_lz77 and self.lz77_window != null) {
            self.lz77_window.?[(self.num_decoded) & k_window_mask] = ret;
            self.num_decoded +%= 1;
        }
        return ret;
    }

    /// Same but not inlined (libjxl: ReadHybridUintClustered). NOTE: `ctx`
    /// must already be a *clustered* index; the context map is applied by
    /// the callers (readHybridUint / readHybridUintInlined), exactly as in
    /// libjxl.
    pub fn readHybridUintClustered(
        self: *ANSSymbolReader,
        comptime uses_lz77: bool,
        ctx: usize,
        br: *BitReader,
        context_map: []const u8,
    ) u32 {
        return self.readHybridUintClusteredInlined(uses_lz77, ctx, br, context_map);
    }

    /// inlined only in the no-lz77 case (libjxl:
    /// ReadHybridUintClusteredMaybeInlined).
    pub fn readHybridUintClusteredMaybeInlined(
        self: *ANSSymbolReader,
        comptime uses_lz77: bool,
        ctx: usize,
        br: *BitReader,
        context_map: []const u8,
    ) u32 {
        if (uses_lz77) {
            return self.readHybridUintClustered(uses_lz77, ctx, br, context_map);
        } else {
            return self.readHybridUintClusteredInlined(uses_lz77, ctx, br, context_map);
        }
    }

    /// libjxl: ReadHybridUintInlined<uses_lz77>.
    pub fn readHybridUintInlined(
        self: *ANSSymbolReader,
        comptime uses_lz77: bool,
        ctx: usize,
        br: *BitReader,
        context_map: []const u8,
    ) u32 {
        return self.readHybridUintClustered(uses_lz77, context_map[ctx], br, context_map);
    }

    /// libjxl: ReadHybridUint (non-hot path).
    pub fn readHybridUint(self: *ANSSymbolReader, ctx: usize, br: *BitReader, context_map: []const u8) u32 {
        return self.readHybridUintClustered(true, context_map[ctx], br, context_map);
    }

    /// ctx is a *clustered* context! This function will modify the ANS state
    /// as if `count` symbols have been decoded.
    pub fn isSingleValueAndAdvance(self: *ANSSymbolReader, ctx: usize, value: *u32, count: usize) bool {
        // No optimization for Huffman mode yet.
        if (self.use_prefix_code) return false;
        if (self.num_to_copy != 0) return false;
        const res: u32 = self.state & (ans_common.ans_tab_size - 1);
        const table = self.alias_tables[ctx << @intCast(self.log_alpha_size)..];
        const symbol = aliasLookup(table, res, @intCast(self.log_entry_size), self.entry_size_minus_1);
        if (symbol.freq != ans_common.ans_tab_size) return false;
        if (self.configs[ctx].split_token <= symbol.value) return false;
        if (symbol.value >= self.lz77_threshold) return false;
        value.* = @intCast(symbol.value);
        if (self.lz77_window) |w| {
            for (0..count) |_| {
                w[(self.num_decoded) & k_window_mask] = @intCast(symbol.value);
                self.num_decoded +%= 1;
            }
        }
        return true;
    }

    // -- Checkpointing -----------------------------------------------------

    pub const k_max_checkpoint_interval: usize = 512;

    pub const Checkpoint = struct {
        state: u32,
        num_to_copy: u32,
        copy_pos: u32,
        num_decoded: u32,
        lz77_window: [k_max_checkpoint_interval]u32,
    };

    pub fn save(self: *const ANSSymbolReader, checkpoint: *Checkpoint) void {
        checkpoint.state = self.state;
        checkpoint.num_decoded = self.num_decoded;
        checkpoint.num_to_copy = self.num_to_copy;
        checkpoint.copy_pos = self.copy_pos;
        if (self.lz77_window) |w| {
            const win_start = self.num_decoded & k_window_mask;
            const win_end = (self.num_decoded + k_max_checkpoint_interval) & k_window_mask;
            if (win_end > win_start) {
                @memcpy(checkpoint.lz77_window[0..(win_end - win_start)], w[win_start..win_end]);
            } else {
                @memcpy(checkpoint.lz77_window[0 .. k_window_size - win_start], w[win_start..k_window_size]);
                @memcpy(checkpoint.lz77_window[k_window_size - win_start ..][0..win_end], w[0..win_end]);
            }
        }
    }

    pub fn restore(self: *ANSSymbolReader, checkpoint: *const Checkpoint) void {
        self.state = checkpoint.state;
        self.num_decoded = checkpoint.num_decoded;
        self.num_to_copy = checkpoint.num_to_copy;
        self.copy_pos = checkpoint.copy_pos;
        if (self.lz77_window) |w| {
            const win_start = self.num_decoded & k_window_mask;
            const win_end = (self.num_decoded + k_max_checkpoint_interval) & k_window_mask;
            if (win_end > win_start) {
                @memcpy(w[win_start..win_end], checkpoint.lz77_window[0 .. win_end - win_start]);
            } else {
                @memcpy(w[win_start..k_window_size], checkpoint.lz77_window[0 .. k_window_size - win_start]);
                @memcpy(w[0..win_end], checkpoint.lz77_window[k_window_size - win_start ..][0..win_end]);
            }
        }
    }
};

// ---------------------------------------------------------------------------

test "readHistogram simple 1-symbol" {
    const alloc = std.testing.allocator;
    var w = bit_io.BitWriter.init(alloc);
    defer w.deinit();
    // simple_code = 1
    w.write(1, 1);
    // num_symbols - 1 = 0
    w.write(1, 0);
    // symbol 0: varlen uint8 (0)
    fields.storeVarLenUint8(0, &w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    var counts: std.ArrayList(ans_common.ANSHistBin) = .empty;
    defer counts.deinit(alloc);
    try readHistogram(alloc, 12, &counts, &br);
    try br.close();
    try std.testing.expectEqual(@as(usize, 1), counts.items.len);
    try std.testing.expectEqual(@as(i32, 4096), counts.items[0]);
}

test "readHistogram flat" {
    const alloc = std.testing.allocator;
    var w = bit_io.BitWriter.init(alloc);
    defer w.deinit();
    w.write(1, 0); // not simple
    w.write(1, 1); // flat
    fields.storeVarLenUint8(7, &w); // alphabet_size - 1 = 7 -> 8 symbols
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    var counts: std.ArrayList(ans_common.ANSHistBin) = .empty;
    defer counts.deinit(alloc);
    try readHistogram(alloc, 12, &counts, &br);
    try br.close();
    try std.testing.expectEqual(@as(usize, 8), counts.items.len);
    var total: i32 = 0;
    for (counts.items) |c| total += c;
    try std.testing.expectEqual(@as(i32, 4096), total);
}

test "decodeUintConfigs roundtrip exhaustive" {
    const alloc = std.testing.allocator;
    // Port of ANSTest.UintConfigRoundtrip.
    var w = bit_io.BitWriter.init(alloc);
    defer w.deinit();
    for (5..9) |log_alpha_size| {
        var configs: std.ArrayList(HybridUintConfig) = .empty;
        defer configs.deinit(alloc);
        for (0..log_alpha_size) |i| {
            for (0..i + 1) |j| {
                for (0..i - j + 1) |k| {
                    try configs.append(alloc, HybridUintConfig.init(@intCast(i), @intCast(j), @intCast(k)));
                }
            }
        }
        try configs.append(alloc, HybridUintConfig.init(@intCast(log_alpha_size), 0, 0));
        // Write (continuing the same writer across iterations).
        const start_bits = w.bitsWritten();
        for (configs.items) |cfg| {
            writeUintConfigForTest(cfg, &w, log_alpha_size);
        }
        w.zeroPadToByte();
        var br = BitReader.init(w.getSpan());
        br.skipBits(start_bits);
        const decoded = try alloc.alloc(HybridUintConfig, configs.items.len);
        defer alloc.free(decoded);
        @memset(decoded, HybridUintConfig.default);
        try decodeUintConfigs(log_alpha_size, decoded, &br);
        try br.close();
        for (configs.items, decoded) |a, b| {
            try std.testing.expectEqual(a.split_token, b.split_token);
            try std.testing.expectEqual(a.msb_in_token, b.msb_in_token);
            try std.testing.expectEqual(a.lsb_in_token, b.lsb_in_token);
        }
        // Reset for the next iteration (w keeps growing, that's fine).
    }
}

/// libjxl `EncodeUintConfig` (enc_ans.cc); duplicated here for the test.
fn writeUintConfigForTest(uint_config: HybridUintConfig, writer: anytype, log_alpha_size: usize) void {
    writer.write(bits.ceilLog2Nonzero(log_alpha_size + 1), uint_config.split_exponent);
    if (uint_config.split_exponent == log_alpha_size) {
        return; // msb/lsb don't matter.
    }
    var nbits = bits.ceilLog2Nonzero(uint_config.split_exponent + 1);
    writer.write(nbits, uint_config.msb_in_token);
    nbits = bits.ceilLog2Nonzero(uint_config.split_exponent - uint_config.msb_in_token + 1);
    writer.write(nbits, uint_config.lsb_in_token);
}

test "inverseMoveToFrontTransform" {
    var v = [_]u8{ 1, 1, 0, 2 };
    inverseMoveToFrontTransform(&v);
    // mtf starts as [0,1,2,...]:
    // 1 -> v=1, mtf=[1,0,2]; 1 -> v=0, mtf=[0,1,2]; 0 -> v=0;
    // 2 -> v=2, mtf=[2,0,1].
    try std.testing.expectEqualSlices(u8, &.{ 1, 0, 0, 2 }, &v);
}
