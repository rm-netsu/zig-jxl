//! ANS/prefix histogram encoding and token stream writing.
//! Bit-exact port of libjxl `enc_ans.h/cc`, `enc_ans_params.h` and the
//! scalar paths of `enc_ans_simd.cc`.

const std = @import("std");
const ans_common = @import("ans_common.zig");
const bits = @import("../base/bits.zig");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const enc_cluster = @import("enc_cluster.zig");
const fast_math = @import("../base/fast_math.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const SizeWriter = fields.SizeWriter;
const HybridUintConfig = ans_common.HybridUintConfig;
const AliasEntry = ans_common.AliasEntry;
const aliasLookup = ans_common.aliasLookup;
const initAliasTable = ans_common.initAliasTable;
const ANSHistBin = ans_common.ANSHistBin;
const LZ77Params = fields.LZ77Params;
const fastLog2f = fast_math.fastLog2f;

pub const ans_log_tab_size = ans_common.ans_log_tab_size;
pub const ans_tab_size = ans_common.ans_tab_size;
pub const ans_max_alphabet_size = ans_common.ans_max_alphabet_size;
pub const prefix_max_bits = ans_common.prefix_max_bits;

/// precision must be equal to: #bits(state_) + #bits(freq)
/// (libjxl: RECIPROCAL_PRECISION).
pub const reciprocal_precision: u6 = 32 + @as(u6, @intCast(ans_log_tab_size));

const k_max_num_symbols_for_small_code = 2;

/// libjxl: kNumFixedHistograms.
pub const k_num_fixed_histograms: usize = 1;

// ---------------------------------------------------------------------------
// Token (enc_ans.h)
// ---------------------------------------------------------------------------

/// Integer to be encoded by an entropy coder, either ANS or Huffman.
pub const Token = struct {
    is_lz77_length: bool = false,
    context: u32 = 0,
    value: u32 = 0,

    pub fn init(context: u32, value: u32) Token {
        return .{ .context = context, .value = value };
    }
};

// ---------------------------------------------------------------------------
// ANSEncSymbolInfo / ANSCoder (enc_ans.h)
// ---------------------------------------------------------------------------

/// Data structure representing one element of the encoding table built from
/// a distribution.
pub const ANSEncSymbolInfo = struct {
    freq: u16 = 0,
    reverse_map: []u16 = &.{},
    ifreq: u64 = 0,
    // Prefix coding.
    depth: u8 = 0,
    bits: u16 = 0,
};

pub const ANSCoder = struct {
    state: u32 = ans_common.ans_final_state,

    /// Port of `ANSCoder::PutSymbol`. Returns the emitted bits; `nbits`
    /// receives the number of emitted bits (0 or 16).
    pub fn putSymbol(self: *ANSCoder, t: *const ANSEncSymbolInfo, nbits: *u8) u32 {
        var bits_out: u32 = 0;
        nbits.* = 0;
        if ((self.state >> @intCast(32 - ans_log_tab_size)) >= t.freq) {
            bits_out = self.state & 0xffff;
            self.state >>= 16;
            nbits.* = 16;
        }
        // Mult-by-reciprocal trick, 64b calculation.
        const v: u32 = @intCast((@as(u64, self.state) * t.ifreq) >> reciprocal_precision);
        const offset = t.reverse_map[self.state - v * t.freq];
        self.state = (v << @intCast(ans_log_tab_size)) + offset;
        return bits_out;
    }
};

// ---------------------------------------------------------------------------
// HistogramParams (enc_ans_params.h)
// ---------------------------------------------------------------------------

pub const ClusteringType = enum { kFastest, kFast, kBest };
pub const HybridUintMethod = enum { kNone, k000, kFast, kContextMap, kBest };
pub const LZ77Method = enum { kNone, kRLE, kLZ77, kOptimal };
pub const ANSHistogramStrategy = enum { kFast, kApproximate, kPrecise };

pub const HistogramParams = struct {
    clustering: ClusteringType = .kBest,
    uint_method: HybridUintMethod = .kBest,
    lz77_method: LZ77Method = .kRLE,
    ans_histogram_strategy: ANSHistogramStrategy = .kPrecise,
    image_widths: []const usize = &.{},
    max_histograms: usize = std.math.maxInt(usize),
    force_huffman: bool = false,
    initialize_global_state: bool = true,
    streaming_mode: bool = false,
    add_missing_symbols: bool = false,
    add_fixed_histograms: bool = false,

    pub fn uintConfig(self: *const HistogramParams) HybridUintConfig {
        if (self.uint_method == .kContextMap) {
            return HybridUintConfig.init(2, 0, 1);
        }
        if (self.uint_method == .k000) {
            return HybridUintConfig.init(0, 0, 0);
        }
        // Default config for clustering.
        return HybridUintConfig.default;
    }
};

// ---------------------------------------------------------------------------
// Histogram (enc_ans_params.h)
// ---------------------------------------------------------------------------

pub const Histogram = struct {
    counts: std.ArrayList(ANSHistBin) = .empty,
    total_count: usize = 0,
    entropy: f32 = 0, // WARNING: not kept up-to-date.
    allocator: std.mem.Allocator,

    /// libjxl: kRounding.
    pub const k_rounding: usize = 8;

    pub fn init(allocator: std.mem.Allocator) Histogram {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *Histogram) void {
        self.counts.deinit(self.allocator);
        self.* = .{ .allocator = self.allocator };
    }

    pub fn clone(self: *const Histogram) Histogram {
        var h = Histogram{ .allocator = self.allocator };
        h.counts.appendSlice(self.allocator, self.counts.items) catch @panic("OOM");
        h.total_count = self.total_count;
        h.entropy = self.entropy;
        return h;
    }

    /// Create flat histogram (libjxl: Histogram::Flat).
    pub fn flat(allocator: std.mem.Allocator, length: i32, total_count: i32) Histogram {
        var h = Histogram.init(allocator);
        const counts = ans_common.createFlatHistogram(allocator, length, total_count) catch @panic("OOM");
        defer allocator.free(counts);
        h.counts.appendSlice(allocator, counts) catch @panic("OOM");
        h.total_count = @intCast(total_count);
        return h;
    }

    pub fn clear(self: *Histogram) void {
        self.counts.clearRetainingCapacity();
        self.total_count = 0;
        self.entropy = 0.0;
    }

    pub fn add(self: *Histogram, allocator: std.mem.Allocator, symbol: usize) !void {
        if (self.counts.items.len <= symbol) {
            const new_len = bits.divCeil(symbol + 1, k_rounding) * k_rounding;
            const old_len = self.counts.items.len;
            try self.counts.resize(allocator, new_len);
            @memset(self.counts.items[old_len..], 0);
        }
        self.counts.items[symbol] += 1;
        self.total_count += 1;
    }

    /// Use this before fastAdd sequence (libjxl: EnsureCapacity).
    pub fn ensureCapacity(self: *Histogram, allocator: std.mem.Allocator, length: usize) !void {
        const new_len = bits.divCeil(length, k_rounding) * k_rounding;
        const old_len = self.counts.items.len;
        try self.counts.resize(allocator, new_len);
        if (new_len > old_len) {
            @memset(self.counts.items[old_len..], 0);
        }
    }

    /// Just increment symbol counter; caller must stretch Histogram before.
    pub fn fastAdd(self: *Histogram, symbol: usize) void {
        self.counts.items[symbol] += 1;
    }

    /// Should be called after a sequence of fastAdd (libjxl: Condition).
    pub fn condition(self: *Histogram) void {
        enc_cluster.histogramCondition(self);
    }

    pub fn addHistogram(self: *Histogram, allocator: std.mem.Allocator, other: *const Histogram) !void {
        if (other.counts.items.len > self.counts.items.len) {
            const old_len = self.counts.items.len;
            try self.counts.resize(allocator, other.counts.items.len);
            @memset(self.counts.items[old_len..], 0);
        }
        for (other.counts.items, 0..) |c, i| {
            self.counts.items[i] += c;
        }
        self.total_count += other.total_count;
    }

    pub fn alphabetSize(self: *const Histogram) usize {
        var i: isize = @as(isize, @intCast(self.counts.items.len)) - 1;
        while (i >= 0) : (i -= 1) {
            if (self.counts.items[@intCast(i)] > 0) {
                return @intCast(i + 1);
            }
        }
        return 0;
    }

    pub fn maxSymbol(self: *const Histogram) usize {
        if (self.total_count == 0) return 0;
        var i: isize = @as(isize, @intCast(self.counts.items.len)) - 1;
        while (i > 0) : (i -= 1) {
            if (self.counts.items[@intCast(i)] != 0) return @intCast(i);
        }
        return 0;
    }

    /// Returns an estimate of the number of bits required to encode the
    /// given histogram (libjxl: ANSPopulationCost).
    pub fn ansPopulationCost(self: *const Histogram, allocator: std.mem.Allocator) !f32 {
        if (self.counts.items.len > ans_max_alphabet_size) {
            return std.math.floatMax(f32);
        }
        var normalized = try ANSEncodingHistogram.computeBest(allocator, self, .kFast);
        defer normalized.deinit();
        return normalized.cost_;
    }

    pub fn shannonEntropy(self: *Histogram) f32 {
        enc_cluster.histogramEntropy(self);
        return self.entropy;
    }
};

// ---------------------------------------------------------------------------
// ANSEncodingHistogram (enc_ans.cc, anonymous namespace)
// ---------------------------------------------------------------------------

/// Fixed-point log2 LUT for values of [0, 4096] (libjxl: AEH::lg2).
const lg2: [ans_tab_size + 1]u32 = blk: {
    @setEvalBranchQuota(100000);
    var l: [ans_tab_size + 1]u32 = undefined;
    l[0] = 0; // for entropy calculations it is OK
    for (1..l.len) |i| {
        const v: f64 = std.math.log2(@as(f64, @floatFromInt(i))) / @as(f64, ans_log_tab_size) * (@as(f64, 1 << 31));
        l[i] = @intFromFloat(@round(v));
    }
    break :blk l;
};

pub fn smallestIncrementLog(count: u32, shift: u32) u32 {
    if (count == 0) return 0;
    const b: u32 = @intCast(bits.floorLog2Nonzero(count));
    const drop = b - ans_common.getPopulationCountPrecision(b, shift);
    return drop;
}

const CountsEntropy = struct {
    count: ANSHistBin = 0,
    step_log: ANSHistBin = 0,
    delta_lg2: i32 = 0,
};

/// Array is sorted by decreasing allowed counts for each possible shift.
const AllowedCounts = struct {
    array: [ans_log_tab_size][ans_tab_size]CountsEntropy,
    index: [ans_log_tab_size][ans_tab_size]u16,
};

const allowed_counts: AllowedCounts = blk: {
    @setEvalBranchQuota(100000000);
    var result: AllowedCounts = undefined;
    for (0..ans_log_tab_size) |shift| {
        const ac = &result.array[shift];
        const ai = &result.index[shift];
        for (ac) |*e| e.* = .{};
        for (ai) |*e| e.* = 0;
        var last: i64 = -1;
        var slot: usize = 0;
        ac[0].delta_lg2 = 0;
        ac[0].step_log = 0;
        var i: i64 = ans_tab_size - 1;
        while (i >= 0) : (i -= 1) {
            const drop = smallestIncrementLog(@intCast(i), @intCast(shift));
            const curr: i64 = i & ~((@as(i64, 1) << @intCast(drop)) - 1);
            if (curr == last) continue;
            last = curr;
            ac[slot].count = @intCast(curr);
            ai[@intCast(curr)] = @intCast(slot);
            if (curr == 0) {
                // Guards against non-possible steps: at max value [0] - 0
                // (by init), at min value - max.
                ac[slot].delta_lg2 = std.math.maxInt(i32);
                ac[slot].step_log = 0;
            } else if (slot > 0) {
                const prev = ac[slot - 1].count;
                const ratio: f64 = @as(f64, @floatFromInt(prev)) / @as(f64, @floatFromInt(curr));
                const d: f64 = std.math.log2(ratio) / @as(f64, ans_log_tab_size) * (@as(f64, 1 << 31));
                ac[slot].delta_lg2 = @intFromFloat(@round(d));
                ac[slot].step_log = @intCast(bits.floorLog2Nonzero(prev - curr));
            }
            slot += 1;
        }
        // Entries beyond `slot` stay zero-initialized (libjxl leaves them
        // uninitialized; they are never read for valid indices).
        for (ac[slot..]) |*e| {
            e.* = .{ .count = 0, .step_log = 0, .delta_lg2 = std.math.maxInt(i32) };
        }
    }
    break :blk result;
};

/// Port of libjxl `ANSEncodingHistogram`.
pub const ANSEncodingHistogram = struct {
    cost_: f32 = 0,
    method_: u32 = 0,
    omit_pos_: usize = 0,
    alphabet_size_: usize = 0,
    num_symbols_: usize = 0,
    symbols_: [k_max_num_symbols_for_small_code]usize = .{ 0, 0 },
    counts_: std.ArrayList(ANSHistBin) = .empty,
    allocator: std.mem.Allocator,

    pub fn deinit(self: *ANSEncodingHistogram) void {
        self.counts_.deinit(self.allocator);
    }

    fn estimateDataBits(self: *const ANSEncodingHistogram, histo: *const Histogram) f32 {
        var sum: i64 = 0;
        for (0..self.alphabet_size_) |i| {
            // += histogram[i] * -log(counts[i]/total_counts)
            sum += @as(i64, histo.counts.items[i]) * @as(i64, lg2[@intCast(self.counts_.items[i])]);
        }
        const sum_f: f32 = @as(f32, @floatFromInt(sum)) * @as(f32, @floatCast(std.math.ldexp(@as(f64, 1), -31)));
        return (@as(f32, @floatFromInt(histo.total_count)) - sum_f) * @as(f32, ans_log_tab_size);
    }

    fn estimateDataBitsFlat(histo: *const Histogram) f32 {
        const len = histo.alphabetSize();
        const flat_bits: i64 = @as(i64, lg2[len]) * @as(i64, ans_log_tab_size);
        const v: i64 = @as(i64, @intCast(histo.total_count)) * flat_bits;
        return @as(f32, @floatFromInt(v)) * @as(f32, @floatCast(std.math.ldexp(@as(f64, 1), -31)));
    }

    /// The only way to construct a valid histogram for ANS encoding.
    pub fn computeBest(allocator: std.mem.Allocator, histo: *const Histogram, strategy: ANSHistogramStrategy) !ANSEncodingHistogram {
        var result = ANSEncodingHistogram{ .allocator = allocator };
        errdefer result.deinit();

        result.alphabet_size_ = histo.alphabetSize();
        if (result.alphabet_size_ > ans_max_alphabet_size) {
            return error.TooManyEntries;
        }

        if (result.alphabet_size_ > 0) {
            // Flat code
            result.method_ = 0;
            result.num_symbols_ = result.alphabet_size_;
            const flat = try ans_common.createFlatHistogram(allocator, @intCast(result.alphabet_size_), @intCast(ans_tab_size));
            defer allocator.free(flat);
            try result.counts_.appendSlice(allocator, flat);
            // In this case length can be non-suitable for SIMD - fix it.
            try result.counts_.resize(allocator, histo.counts.items.len);
            @memset(result.counts_.items[flat.len..], 0);
            var size_writer = SizeWriter{};
            try result.encode(&size_writer);
            result.cost_ = @as(f32, @floatFromInt(size_writer.size)) + estimateDataBitsFlat(histo);
        } else {
            // Empty histogram
            result.method_ = 1;
            result.num_symbols_ = 0;
            result.cost_ = 3;
            return result;
        }

        var symbol_count: usize = 0;
        for (0..result.alphabet_size_) |n| {
            if (histo.counts.items[n] > 0) {
                if (symbol_count < k_max_num_symbols_for_small_code) {
                    result.symbols_[symbol_count] = n;
                }
                symbol_count += 1;
            }
        }
        result.num_symbols_ = symbol_count;
        if (symbol_count == 1) {
            // Single-bin histogram
            result.method_ = 1;
            try result.counts_.resize(allocator, histo.counts.items.len);
            @memset(result.counts_.items, 0);
            result.counts_.items[result.symbols_[0]] = @intCast(ans_tab_size);
            var size_writer = SizeWriter{};
            try result.encode(&size_writer);
            result.cost_ = @as(f32, @floatFromInt(size_writer.size));
            return result;
        }

        // Here min 2 symbols.
        var normalized = ANSEncodingHistogram{ .allocator = allocator };
        defer normalized.deinit();
        // C++: `normalized = result` — deep copy of the flat-histogram
        // baseline; rebalanceHistogram works on this copy.
        try normalized.counts_.appendSlice(allocator, result.counts_.items);
        normalized.alphabet_size_ = result.alphabet_size_;
        normalized.num_symbols_ = result.num_symbols_;
        normalized.symbols_ = result.symbols_;

        var shift: u32 = 0;
        while (shift < ans_log_tab_size) : (shift += 1) {
            switch (strategy) {
                .kPrecise => {},
                .kApproximate => {
                    if (shift % 2 != 0) continue;
                },
                .kFast => {
                    if (shift != 0 and shift != ans_log_tab_size / 2 and shift != ans_log_tab_size) {
                        continue;
                    }
                },
            }
            // `shift = 12` and `shift = 11` are the same
            normalized.method_ = @min(shift, ans_log_tab_size - 1) + 1;

            if (!try normalized.rebalanceHistogram(histo)) {
                return error.RebalanceFailed;
            }
            var size_writer = SizeWriter{};
            try normalized.encode(&size_writer);
            normalized.cost_ = @as(f32, @floatFromInt(size_writer.size)) + normalized.estimateDataBits(histo);
            if (normalized.cost_ < result.cost_) {
                // result = normalized (deep copy)
                result.method_ = normalized.method_;
                result.omit_pos_ = normalized.omit_pos_;
                result.num_symbols_ = normalized.num_symbols_;
                result.symbols_ = normalized.symbols_;
                result.cost_ = normalized.cost_;
                try result.counts_.resize(allocator, normalized.counts_.items.len);
                @memcpy(result.counts_.items, normalized.counts_.items);
            }
        }

        return result;
    }

    /// Port of `ANSEncodingHistogram::Encode`. Works with any writer that
    /// has a `write(nbits, bits)` method (BitWriter or SizeWriter).
    pub fn encode(self: *const ANSEncodingHistogram, writer: anytype) !void {
        // The check ensures also that all RLE sequences can be encoded by
        // storeVarLenUint8.
        std.debug.assert(self.alphabet_size_ <= ans_max_alphabet_size);

        // Flat histogram.
        if (self.method_ == 0) {
            // Mark non-small tree.
            writer.write(1, 0);
            // Mark uniform histogram.
            writer.write(1, 1);
            std.debug.assert(self.alphabet_size_ > 0);
            // Encode alphabet size.
            fields.storeVarLenUint8(self.alphabet_size_ - 1, writer);
            return;
        }

        // Small tree.
        if (self.num_symbols_ <= k_max_num_symbols_for_small_code) {
            // Small tree marker to encode 1-2 symbols.
            writer.write(1, 1);
            if (self.num_symbols_ == 0) {
                writer.write(1, 0);
                fields.storeVarLenUint8(0, writer);
            } else {
                writer.write(1, self.num_symbols_ - 1);
                for (0..self.num_symbols_) |i| {
                    fields.storeVarLenUint8(self.symbols_[i], writer);
                }
            }
            if (self.num_symbols_ == 2) {
                writer.write(@intCast(ans_log_tab_size), @intCast(self.counts_.items[self.symbols_[0]]));
            }
            return;
        }

        // General tree.
        // Mark non-small tree.
        writer.write(1, 0);
        // Mark non-flat histogram.
        writer.write(1, 0);

        // Elias gamma-like code for `shift = method - 1`. Only difference is
        // that if the number of bits to be encoded is equal to
        // `upper_bound_log`, we skip the terminating 0 in unary coding.
        const upper_bound_log = bits.floorLog2Nonzero(ans_log_tab_size + 1);
        const log: u32 = @intCast(bits.floorLog2Nonzero(self.method_));
        writer.write(log, (@as(u64, 1) << @intCast(log)) - 1);
        if (log != upper_bound_log) writer.write(1, 0);
        writer.write(log, ((@as(u64, 1) << @intCast(log)) - 1) & self.method_);

        // Since `num_symbols_ >= 3`, we know that `alphabet_size_ >= 3`,
        // therefore we encode `alphabet_size_ - 3`.
        fields.storeVarLenUint8(self.alphabet_size_ - 3, writer);

        // Precompute sequences for RLE encoding. Contains the number of
        // identical values starting at a given index. Only contains that
        // value at the first element of the series.
        var same = [_]u8{0} ** ans_max_alphabet_size;
        var last: usize = 0;
        {
            var i: usize = 1;
            while (i <= self.alphabet_size_) : (i += 1) {
                // Store the sequence length once different symbol reached,
                // or we are near the omit_pos_, or we're at the end. We
                // don't support including the omit_pos_ in an RLE sequence
                // because this value may use a different amount of log2
                // bits than standard, it is too complex to handle in the
                // decoder.
                if (i == self.alphabet_size_ or i == self.omit_pos_ or
                    i == self.omit_pos_ + 1 or
                    (i < self.alphabet_size_ and self.counts_.items[i] != self.counts_.items[last]))
                {
                    same[last] = @intCast(i - last);
                    last = i;
                }
            }
        }

        var bit_width = [_]u8{0} ** ans_max_alphabet_size;
        // Use shortest possible Huffman code to encode `omit_pos`.
        var omit_width: i32 = 10;
        for (0..self.alphabet_size_) |i| {
            if (i != self.omit_pos_ and self.counts_.items[i] > 0) {
                bit_width[i] = @intCast(bits.floorLog2Nonzero(@as(u32, @intCast(self.counts_.items[i]))) + 1);
                omit_width = @max(omit_width, @as(i32, bit_width[i]) + @intFromBool(i < self.omit_pos_));
            }
        }
        bit_width[self.omit_pos_] = @intCast(omit_width);

        // The bit widths are encoded with a static Huffman code.
        // The last symbol is used as RLE sequence.
        const k_bit_width_lengths = [ans_log_tab_size + 2]u8{
            5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 6, 7, 7,
        };
        const k_bit_width_symbols = [ans_log_tab_size + 2]u8{
            17, 11, 15, 3, 9, 7, 4, 2, 5, 6, 0, 33, 1, 65,
        };
        const k_min_reps = 5;
        const rep: usize = ans_log_tab_size + 1;
        // Encode count bit widths.
        {
            var i: usize = 0;
            while (i < self.alphabet_size_) : (i += 1) {
                writer.write(k_bit_width_lengths[bit_width[i]], k_bit_width_symbols[bit_width[i]]);
                if (same[i] >= k_min_reps) {
                    // Encode the RLE symbol and skip the repeated ones.
                    writer.write(k_bit_width_lengths[rep], k_bit_width_symbols[rep]);
                    fields.storeVarLenUint8(same[i] - k_min_reps, writer);
                    i += same[i] - 1;
                }
            }
        }
        // Encode additional bits of accuracy.
        const shift = self.method_ - 1;
        if (shift != 0) { // otherwise `bitcount = 0`
            var i: usize = 0;
            while (i < self.alphabet_size_) : (i += 1) {
                if (bit_width[i] > 1 and i != self.omit_pos_) {
                    const bitcount = ans_common.getPopulationCountPrecision(bit_width[i] - 1, shift);
                    const drop_bits = bit_width[i] - 1 - bitcount;
                    std.debug.assert((@as(u32, @intCast(self.counts_.items[i])) & ((@as(u32, 1) << @intCast(drop_bits)) - 1)) == 0);
                    const extra: u32 = (@as(u32, @intCast(self.counts_.items[i])) >> @as(u5, @intCast(drop_bits))) - (@as(u32, 1) << @intCast(bitcount));
                    writer.write(bitcount, extra);
                }
                if (same[i] >= k_min_reps) {
                    // Skip symbols encoded by RLE.
                    i += same[i] - 1;
                }
            }
        }
    }

    /// Returns the difference between the largest count that can be
    /// represented and is smaller than "count", and the smallest
    /// representable count larger than "count".
    fn rebalanceHistogram(self: *ANSEncodingHistogram, histo: *const Histogram) !bool {
        const table_size: i32 = @intCast(ans_tab_size);
        const shift = self.method_ - 1;

        const EntropyDelta = struct {
            freq: ANSHistBin,
            count_ind: usize,
            bin_ind: usize,
        };
        var balance_inc = [_]i64{0} ** (ans_log_tab_size - 1);
        var balance_dec = [_]i64{0} ** (ans_log_tab_size - 1);
        const ac = allowed_counts.array[shift];
        const ai = allowed_counts.index[shift];

        const deltaEntropyInc = struct {
            fn f(a: EntropyDelta, ac_: []const CountsEntropy, balance_inc_: *const [ans_log_tab_size - 1]i64) i64 {
                return @as(i64, a.freq) * @as(i64, ac_[a.count_ind].delta_lg2) - balance_inc_[@intCast(ac_[a.count_ind].step_log)];
            }
        }.f;
        const deltaEntropyDec = struct {
            fn f(a: EntropyDelta, ac_: []const CountsEntropy, balance_dec_: *const [ans_log_tab_size - 1]i64) i64 {
                return @as(i64, a.freq) * @as(i64, ac_[a.count_ind + 1].delta_lg2) - balance_dec_[@intCast(ac_[a.count_ind + 1].step_log)];
            }
        }.f;

        // Vector of adjustable bins from `allowed_counts`.
        var bins: std.ArrayList(EntropyDelta) = .empty;
        defer bins.deinit(self.allocator);

        const norm: f64 = @as(f64, @floatFromInt(table_size)) / @as(f64, @floatFromInt(histo.total_count));

        var remainder_pos: usize = 0; // highest balancing bin in the histogram
        var max_freq: i64 = 0;
        var rest: ANSHistBin = table_size; // reserve of histogram counts
        for (0..self.alphabet_size_) |n| {
            const freq: ANSHistBin = histo.counts.items[n];
            if (freq > max_freq) {
                remainder_pos = n;
                max_freq = freq;
            }

            const target: f64 = @as(f64, @floatFromInt(freq)) * norm; // rounding
            // Keep zeros and clamp nonzero freq counts to [1, table_size)
            var count: ANSHistBin = @max(@as(ANSHistBin, @intFromFloat(@round(target))), @intFromBool(freq > 0));
            count = @min(count, table_size - 1);
            const step_log = smallestIncrementLog(@intCast(count), shift);
            const inc: ANSHistBin = @as(ANSHistBin, 1) << @intCast(step_log);
            count &= ~(inc - 1);

            self.counts_.items[n] = count;
            rest -= count;
            if (target > 1.0) {
                try bins.append(self.allocator, .{ .freq = freq, .count_ind = ai[@intCast(count)], .bin_ind = n });
            }
        }

        // Delete the highest balancing bin from adjustable bins.
        for (bins.items, 0..) |a, idx| {
            if (a.bin_ind == remainder_pos) {
                _ = bins.orderedRemove(idx);
                break;
            }
        }
        // From now on `rest` is the height of the balancing bin; here it
        // can be negative, but will be tracted into positive domain later.
        rest += self.counts_.items[remainder_pos];

        if (bins.items.len > 0) {
            const max_log: u32 = @intCast(ac[1].step_log);
            while (true) {
                // Update balancing bin penalties setting guards/tractors.
                var log: u32 = 0;
                while (log <= max_log) : (log += 1) {
                    const delta: ANSHistBin = @as(ANSHistBin, 1) << @intCast(log);
                    if (rest >= table_size) {
                        // Tract large `rest` into allowed domain:
                        balance_inc[log] = 0; // permit all inc steps
                        balance_dec[log] = 0; // forbid all dec steps
                    } else if (rest > 1) {
                        // `rest` is OK, put guards against non-possible steps
                        balance_inc[log] = if (rest > delta)
                            max_freq * @as(i64, lg2[@intCast(rest)] - lg2[@intCast(rest - delta)])
                        else
                            std.math.maxInt(i64); // forbidden
                        balance_dec[log] = if (rest + delta < table_size)
                            max_freq * @as(i64, lg2[@intCast(rest + delta)] - lg2[@intCast(rest)])
                        else
                            0; // forbidden
                    } else {
                        // Tract negative or zero `rest` into positive:
                        balance_inc[log] = std.math.maxInt(i64); // forbid all inc
                        balance_dec[log] = std.math.maxInt(i64); // permit all dec
                    }
                }
                // Try to increase entropy: find best bin by delta_entropy_inc
                // (libjxl max_element with IncLess; first max wins ties).
                var best_inc: usize = 0;
                for (1..bins.items.len) |idx| {
                    const a = bins.items[idx];
                    const b = bins.items[best_inc];
                    const va = deltaEntropyInc(a, &ac, &balance_inc) >> @intCast(ac[a.count_ind].step_log);
                    const vb = deltaEntropyInc(b, &ac, &balance_inc) >> @intCast(ac[b.count_ind].step_log);
                    if (vb < va) best_inc = idx;
                }
                if (deltaEntropyInc(bins.items[best_inc], &ac, &balance_inc) > 0) {
                    // Grow the bin with the best histogram entropy increase.
                    rest -= @as(ANSHistBin, 1) << @intCast(ac[bins.items[best_inc].count_ind].step_log);
                    bins.items[best_inc].count_ind -= 1;
                } else {
                    // Find best bin by delta_entropy_dec (min_element).
                    var best_dec: usize = 0;
                    for (1..bins.items.len) |idx| {
                        const a = bins.items[idx];
                        const b = bins.items[best_dec];
                        const va = deltaEntropyDec(a, &ac, &balance_dec) >> @intCast(ac[a.count_ind + 1].step_log);
                        const vb = deltaEntropyDec(b, &ac, &balance_dec) >> @intCast(ac[b.count_ind + 1].step_log);
                        if (va < vb) best_dec = idx;
                    }
                    // Break if no reverse steps can grow entropy (or valid).
                    if (deltaEntropyDec(bins.items[best_dec], &ac, &balance_dec) >= 0) break;
                    // Decrease the bin with the best entropy increase.
                    bins.items[best_dec].count_ind += 1;
                    rest += @as(ANSHistBin, 1) << @intCast(ac[bins.items[best_dec].count_ind].step_log);
                }
            }
            // Set counts besides the balancing bin.
            for (bins.items) |a| {
                self.counts_.items[a.bin_ind] = ac[a.count_ind].count;
            }

            // The scheme works fine if we have room to grow `bit_width` of
            // the balancing bin, otherwise put it to the first bin of 12-bit
            // width.
            for (0..remainder_pos) |n| {
                if (self.counts_.items[n] >= 2048) {
                    self.counts_.items[remainder_pos] = self.counts_.items[n];
                    remainder_pos = n;
                    break;
                }
            }
        }
        // Set balancing bin
        self.counts_.items[remainder_pos] = rest;
        self.omit_pos_ = remainder_pos;

        return self.counts_.items[remainder_pos] > 0;
    }

    /// Port of `ANSEncodingHistogram::ANSBuildInfoTable`. `info` must have
    /// space for at least max(1, alphabet_size) entries.
    pub fn ansBuildInfoTable(
        self: *const ANSEncodingHistogram,
        allocator: std.mem.Allocator,
        table: []const AliasEntry,
        log_alpha_size: usize,
        info: []ANSEncSymbolInfo,
    ) !void {
        // Create valid alias table for empty streams.
        for (0..@max(1, self.alphabet_size_)) |s| {
            const freq: ANSHistBin = if (s == self.alphabet_size_) @intCast(ans_tab_size) else self.counts_.items[s];
            info[s].freq = @intCast(freq);
            if (freq != 0) {
                info[s].ifreq = ((@as(u64, 1) << reciprocal_precision) + @as(u64, @intCast(info[s].freq)) - 1) /
                    @as(u64, @intCast(info[s].freq));
            } else {
                info[s].ifreq = 1; // Shouldn't matter (symbol shouldn't occur).
            }
            info[s].reverse_map = try allocator.alloc(u16, @intCast(freq));
        }
        const log_entry_size = ans_log_tab_size - log_alpha_size;
        const entry_size_minus_1 = (@as(usize, 1) << @intCast(log_entry_size)) - 1;
        for (0..ans_tab_size) |i| {
            const s = aliasLookup(table, i, @intCast(log_entry_size), entry_size_minus_1);
            info[s.value].reverse_map[s.offset] = @intCast(i);
        }
    }
};

// ---------------------------------------------------------------------------
// EntropyEncodingData (enc_ans.h)
// ---------------------------------------------------------------------------

pub const EntropyEncodingData = struct {
    encoding_info: std.ArrayList(std.ArrayList(ANSEncSymbolInfo)) = .empty,
    use_prefix_code: bool = false,
    uint_config: std.ArrayList(HybridUintConfig) = .empty,
    log_alpha_size: usize = 0,
    lz77: LZ77Params = .{},
    encoded_histograms: std.ArrayList(BitWriter) = .empty,
    context_map: std.ArrayList(u8) = .empty,
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) EntropyEncodingData {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *EntropyEncodingData) void {
        for (self.encoding_info.items) |*per_histo| {
            for (per_histo.items) |*info| {
                if (info.reverse_map.len > 0) self.allocator.free(info.reverse_map);
            }
            per_histo.deinit(self.allocator);
        }
        self.encoding_info.deinit(self.allocator);
        self.uint_config.deinit(self.allocator);
        for (self.encoded_histograms.items) |*w| {
            w.deinit();
        }
        self.encoded_histograms.deinit(self.allocator);
        self.context_map.deinit(self.allocator);
        self.* = .{ .allocator = self.allocator };
    }
};

// ---------------------------------------------------------------------------
// uint config (de)serialization helpers (enc_ans.cc)
// ---------------------------------------------------------------------------

pub fn encodeUintConfig(uint_config: HybridUintConfig, writer: anytype, log_alpha_size: usize) void {
    writer.write(bits.ceilLog2Nonzero(log_alpha_size + 1), uint_config.split_exponent);
    if (uint_config.split_exponent == log_alpha_size) {
        return; // msb/lsb don't matter.
    }
    var nbits = bits.ceilLog2Nonzero(uint_config.split_exponent + 1);
    writer.write(nbits, uint_config.msb_in_token);
    nbits = bits.ceilLog2Nonzero(uint_config.split_exponent - uint_config.msb_in_token + 1);
    writer.write(nbits, uint_config.lsb_in_token);
}

pub fn encodeUintConfigs(uint_configs: []const HybridUintConfig, writer: anytype, log_alpha_size: usize) void {
    for (uint_configs) |cfg| {
        encodeUintConfig(cfg, writer, log_alpha_size);
    }
}

// ---------------------------------------------------------------------------
// Scalar helpers replacing enc_ans_simd.cc
// ---------------------------------------------------------------------------

/// Scalar port of `EstimateTokenCost`: encodes all values and returns the
/// total number of extra (non-token) bits. `out` receives the tokens.
pub fn estimateTokenCost(values: []const u32, cfg: HybridUintConfig, out: []u32) u32 {
    var extra_bits: u32 = 0;
    for (values, 0..) |v, i| {
        const e = cfg.encode(v);
        extra_bits += e.nbits;
        out[i] = e.token;
    }
    return extra_bits;
}

pub fn maxValue(values: []const u32) u32 {
    var m: u32 = 0;
    for (values) |v| m = @max(m, v);
    return m;
}

// ---------------------------------------------------------------------------
// BuildAndStoreANSEncodingData (enc_ans.cc)
// ---------------------------------------------------------------------------

/// Port of `EntropyEncodingData::BuildAndStoreANSEncodingData` for one
/// histogram. Returns the cost in bits. `info` is
/// `codes.encoding_info.back()`.
pub fn buildAndStoreANSEncodingData(
    allocator: std.mem.Allocator,
    info: []ANSEncSymbolInfo,
    ans_histogram_strategy: ANSHistogramStrategy,
    use_prefix_code: bool,
    log_alpha_size: usize,
    histogram: *const Histogram,
    writer: ?*BitWriter,
) !usize {
    const size = histogram.alphabetSize();
    if (use_prefix_code) {
        var cost: usize = 0;
        if (size <= 1) return 0;
        const histo = try allocator.alloc(u32, size);
        defer allocator.free(histo);
        for (0..size) |i| {
            histo[i] = @intCast(@max(histogram.counts.items[i], 0));
        }
        const depths = try allocator.alloc(u8, size);
        defer allocator.free(depths);
        @memset(depths, 0);
        const bits_out = try allocator.alloc(u16, size);
        defer allocator.free(bits_out);
        @memset(bits_out, 0);
        if (writer == null) {
            var tmp_writer = BitWriter.init(allocator);
            defer tmp_writer.deinit();
            const enc_huffman = @import("enc_huffman.zig");
            try enc_huffman.buildAndStoreHuffmanTree(allocator, histo, depths, bits_out, &tmp_writer);
            cost = tmp_writer.bitsWritten();
        } else {
            const start = writer.?.bitsWritten();
            const enc_huffman = @import("enc_huffman.zig");
            try enc_huffman.buildAndStoreHuffmanTree(allocator, histo, depths, bits_out, writer.?);
            cost = writer.?.bitsWritten() - start;
        }
        for (0..size) |i| {
            info[i].bits = if (depths[i] == 0) 0 else bits_out[i];
            info[i].depth = depths[i];
        }
        // Estimate data cost.
        for (0..size) |i| {
            cost += @as(usize, @intCast(histo[i])) * info[i].depth;
        }
        return cost;
    }
    var normalized = try ANSEncodingHistogram.computeBest(allocator, histogram, ans_histogram_strategy);
    defer normalized.deinit();

    // Alias table for this histogram.
    const alias_table = try allocator.alloc(AliasEntry, @as(usize, 1) << @intCast(log_alpha_size));
    defer allocator.free(alias_table);
    try initAliasTable(allocator, normalized.counts_.items, ans_log_tab_size, @intCast(log_alpha_size), alias_table);
    try normalized.ansBuildInfoTable(allocator, alias_table, log_alpha_size, info);
    if (writer) |w| {
        try normalized.encode(w);
    }
    const cost: usize = @intFromFloat(@ceil(normalized.cost_));
    return cost;
}

fn histogramFromSymbolInfo(allocator: std.mem.Allocator, encoding_info: []const ANSEncSymbolInfo, use_prefix_code: bool) Histogram {
    var histo = Histogram.init(allocator);
    const rounded_len = bits.divCeil(encoding_info.len, Histogram.k_rounding) * Histogram.k_rounding;
    histo.counts.resize(allocator, rounded_len) catch @panic("OOM");
    @memset(histo.counts.items, 0);
    histo.total_count = 0;
    for (encoding_info, 0..) |info, i| {
        const count: i32 = if (use_prefix_code)
            (if (info.depth != 0) @as(i32, 1) << @intCast(prefix_max_bits - info.depth) else 0)
        else
            info.freq;
        histo.counts.items[i] = count;
        histo.total_count += @intCast(count);
    }
    return histo;
}

// ---------------------------------------------------------------------------
// ChooseUintConfigs (enc_ans.cc)
// ---------------------------------------------------------------------------

fn chooseUintConfigs(
    allocator: std.mem.Allocator,
    codes: *EntropyEncodingData,
    params: *const HistogramParams,
    tokens: []const std.ArrayList(Token),
    clustered_histograms: []Histogram,
) !void {
    // Set sane default `log_alpha_size`.
    if (codes.use_prefix_code) {
        codes.log_alpha_size = prefix_max_bits;
    } else if (params.streaming_mode) {
        codes.log_alpha_size = 8;
    } else if (codes.lz77.enabled) {
        codes.log_alpha_size = 8;
    } else {
        codes.log_alpha_size = 7;
    }

    codes.uint_config.clearRetainingCapacity();
    try codes.uint_config.appendNTimes(allocator, params.uintConfig(), clustered_histograms.len);
    // If the uint config is fixed, just use it.
    if (params.uint_method != .kBest and params.uint_method != .kFast) {
        return;
    }
    // Even if the uint config is adaptive, just stick with the default in
    // streaming mode.
    if (params.streaming_mode) {
        return;
    }

    // Brute-force method that tries a few options.
    var configs: std.ArrayList(HybridUintConfig) = .empty;
    defer configs.deinit(allocator);
    if (params.uint_method == .kBest) {
        try configs.appendSlice(allocator, &.{
            HybridUintConfig.init(4, 2, 0), // default
            HybridUintConfig.init(4, 1, 0), // less precise
            HybridUintConfig.init(4, 2, 1), // add sign
            HybridUintConfig.init(4, 2, 2), // add sign+parity
            HybridUintConfig.init(4, 1, 2), // add parity but less msb
            // Same as above, but more direct coding.
            HybridUintConfig.init(5, 2, 0), HybridUintConfig.init(5, 1, 0),
            HybridUintConfig.init(5, 2, 1), HybridUintConfig.init(5, 2, 2),
            HybridUintConfig.init(5, 1, 2),
            // Same as above, but less direct coding.
            HybridUintConfig.init(3, 2, 0), HybridUintConfig.init(3, 1, 0),
            HybridUintConfig.init(3, 2, 1), HybridUintConfig.init(3, 1, 2),
            // For near-lossless.
            HybridUintConfig.init(4, 1, 3), HybridUintConfig.init(5, 1, 4),
            HybridUintConfig.init(5, 2, 3), HybridUintConfig.init(6, 1, 5),
            HybridUintConfig.init(6, 2, 4), HybridUintConfig.init(6, 0, 0),
            // Other
            HybridUintConfig.init(0, 0, 0), // varlenuint
            HybridUintConfig.init(2, 0, 1), // works well for ctx map
            HybridUintConfig.init(7, 0, 0), // direct coding
            HybridUintConfig.init(8, 0, 0), // direct coding
            HybridUintConfig.init(9, 0, 0), // direct coding
            HybridUintConfig.init(10, 0, 0), // direct coding
            HybridUintConfig.init(11, 0, 0), // direct coding
            HybridUintConfig.init(12, 0, 0), // direct coding
        });
    } else {
        try configs.appendSlice(allocator, &.{
            HybridUintConfig.init(4, 2, 0), // default
            HybridUintConfig.init(4, 1, 2), // add parity but less msb
            HybridUintConfig.init(0, 0, 0), // smallest histograms
            HybridUintConfig.init(2, 0, 1), // works well for ctx map
        });
    }

    const num_histo = clustered_histograms.len;
    const is_valid = try allocator.alloc(u8, num_histo);
    defer allocator.free(is_valid);
    @memset(is_valid, 0);
    const histo_volume = try allocator.alloc(usize, 2 * num_histo);
    defer allocator.free(histo_volume);
    @memset(histo_volume, 0);
    const histo_offset = try allocator.alloc(usize, 2 * num_histo + 1);
    defer allocator.free(histo_offset);
    @memset(histo_offset, 0);
    const max_value_per_histo = try allocator.alloc(u32, 2 * num_histo);
    defer allocator.free(max_value_per_histo);
    @memset(max_value_per_histo, 0);

    // TODO(veluca): do not ignore lz77 commands.

    for (tokens) |stream| {
        for (stream.items) |token| {
            const histo = codes.context_map.items[token.context];
            histo_volume[histo + (if (token.is_lz77_length) num_histo else 0)] += 1;
        }
    }
    var max_histo_volume: usize = 0;
    for (0..2 * num_histo) |h| {
        max_histo_volume = @max(max_histo_volume, histo_volume[h]);
        histo_offset[h + 1] = histo_offset[h] + histo_volume[h];
    }

    const transposed = try allocator.alloc(u32, histo_offset[num_histo * 2]);
    defer allocator.free(transposed);
    {
        const next_offset = try allocator.dupe(usize, histo_offset);
        defer allocator.free(next_offset);
        for (tokens) |stream| {
            for (stream.items) |token| {
                const histo = codes.context_map.items[token.context] +
                    (if (token.is_lz77_length) num_histo else 0);
                transposed[next_offset[histo]] = token.value;
                next_offset[histo] += 1;
            }
        }
    }
    for (0..2 * num_histo) |h| {
        max_value_per_histo[h] = maxValue(transposed[histo_offset[h]..][0..histo_volume[h]]);
    }

    // Wider histograms are assigned max cost in PopulationCost anyway and
    // therefore will not be used.
    const max_alpha = ans_max_alphabet_size;

    const tmp = try allocator.alloc(u32, max_histo_volume);
    defer allocator.free(tmp);
    for (0..num_histo) |h| {
        var best_cost: f32 = std.math.floatMax(f32);
        for (configs.items) |cfg| {
            const max_v = max_value_per_histo[h];
            var capacity: usize = undefined;
            {
                const e = cfg.encode(max_v);
                const tok = e.token | cfg.lsbMask();
                if (tok >= max_alpha or (codes.lz77.enabled and tok >= codes.lz77.min_symbol)) {
                    continue; // Not valid config for this context
                }
                capacity = tok + 1;
            }

            var histo = Histogram.init(allocator);
            defer histo.deinit();
            try histo.ensureCapacity(allocator, capacity);
            const len = histo_volume[h];
            const data = transposed[histo_offset[h]..][0..len];
            const extra_bits = estimateTokenCost(data, cfg, tmp[0..len]);
            const tmp_tokens = tmp;
            for (0..len) |i| {
                histo.fastAdd(tmp_tokens[i]);
            }
            histo.condition();
            var cost = try histo.ansPopulationCost(allocator);
            cost += @floatFromInt(extra_bits);
            // Add signaling cost of the hybriduintconfig itself.
            cost += @floatFromInt(bits.ceilLog2Nonzero(cfg.split_exponent + 1));
            cost += @floatFromInt(bits.ceilLog2Nonzero(cfg.split_exponent - cfg.msb_in_token + 1));
            if (cost < best_cost) {
                codes.uint_config.items[h] = cfg;
                best_cost = cost;
                std.mem.swap(Histogram, &clustered_histograms[h], &histo);
            }
        }
    }

    var max_tok: usize = 0;
    for (0..num_histo) |h| {
        const histo = &clustered_histograms[h];
        max_tok = @max(max_tok, histo.maxSymbol());
        const len = histo_volume[num_histo + h];
        if (len == 0) continue; // E.g. when lz77 not enabled
        const max_histo_tok = max_value_per_histo[num_histo + h];
        const e = codes.lz77.length_uint_config.encode(max_histo_tok);
        const tok = (e.token | codes.lz77.length_uint_config.lsbMask()) + codes.lz77.min_symbol;
        try histo.ensureCapacity(allocator, tok + 1);
        const data = transposed[histo_offset[num_histo + h]..][0..len];
        _ = estimateTokenCost(data, codes.lz77.length_uint_config, tmp[0..len]);
        for (0..len) |i| {
            histo.fastAdd(tmp[i] + codes.lz77.min_symbol);
        }
        histo.condition();
        max_tok = @max(max_tok, histo.maxSymbol());
    }

    // `log_alpha_size - 5` is encoded in the header, so min is 5.
    var log_size: usize = 5;
    while (max_tok >= (@as(usize, 1) << @intCast(log_size))) log_size += 1;

    const max_log_alpha_size = if (codes.use_prefix_code) prefix_max_bits else 8;
    if (log_size > max_log_alpha_size) return error.InvalidData;

    if (codes.use_prefix_code) {
        codes.log_alpha_size = prefix_max_bits;
    } else {
        codes.log_alpha_size = log_size;
    }
}

// ---------------------------------------------------------------------------
// BuildAndStoreEntropyCodes (enc_ans.cc)
// ---------------------------------------------------------------------------

/// Returns cost (in bits). `writer` may be null for cost estimation.
pub fn buildAndStoreEntropyCodes(
    allocator: std.mem.Allocator,
    codes: *EntropyEncodingData,
    params: *const HistogramParams,
    tokens: []const std.ArrayList(Token),
    builder: []Histogram,
    writer: ?*BitWriter,
) ans_common.EncodeError!usize {
    const prev_histograms = codes.encoding_info.items.len;
    var clustered_histograms: std.ArrayList(Histogram) = .empty;
    defer {
        for (clustered_histograms.items) |*h| h.deinit();
        clustered_histograms.deinit(allocator);
    }
    for (0..prev_histograms) |i| {
        try clustered_histograms.append(allocator, histogramFromSymbolInfo(allocator, codes.encoding_info.items[i].items, codes.use_prefix_code));
    }
    const context_offset = codes.context_map.items.len;
    const cm_old_len = codes.context_map.items.len;
    try codes.context_map.resize(allocator, cm_old_len + builder.len);
    // std::vector::resize zero-initializes; ArrayList does not.
    @memset(codes.context_map.items[cm_old_len..], 0);
    if (builder.len > 1) {
        // Cluster histograms (no fuzzer-friendly mode in this port).
        var histogram_symbols: std.ArrayList(u32) = .empty;
        defer histogram_symbols.deinit(allocator);
        try enc_cluster.clusterHistograms(allocator, params, builder, enc_cluster.k_clusters_limit, &clustered_histograms, &histogram_symbols);
        for (0..builder.len) |c| {
            codes.context_map.items[context_offset + c] = @intCast(histogram_symbols.items[c]);
        }
        if (writer != null) {
            const enc_context_map = @import("enc_context_map.zig");
            try enc_context_map.encodeContextMap(allocator, codes.context_map.items, clustered_histograms.items.len, writer.?);
        }
    } else {
        std.debug.assert(codes.encoding_info.items.len == 0);
        try clustered_histograms.append(allocator, builder[0].clone());
    }

    try chooseUintConfigs(allocator, codes, params, tokens, clustered_histograms.items);

    var size_writer = SizeWriter{}; // Used if writer == null to estimate costs.
    var cost: usize = if (codes.use_prefix_code) 1 else 3;

    if (writer) |w| {
        w.write(1, @intFromBool(codes.use_prefix_code));
    }
    if (writer == null) {
        encodeUintConfigs(codes.uint_config.items, &size_writer, codes.log_alpha_size);
    } else {
        if (!codes.use_prefix_code) writer.?.write(2, codes.log_alpha_size - 5);
        encodeUintConfigs(codes.uint_config.items, writer.?, codes.log_alpha_size);
    }
    if (codes.use_prefix_code) {
        for (clustered_histograms.items) |*histo| {
            const alphabet_size = @max(1, histo.alphabetSize());
            if (writer) |w| {
                fields.storeVarLenUint16(alphabet_size - 1, w);
            } else {
                fields.storeVarLenUint16(alphabet_size - 1, &size_writer);
            }
        }
    }
    cost += size_writer.size;
    for (prev_histograms..clustered_histograms.items.len) |c| {
        const alphabet_size = clustered_histograms.items[c].alphabetSize();
        var per_histo: std.ArrayList(ANSEncSymbolInfo) = .empty;
        try per_histo.resize(allocator, alphabet_size);
        @memset(per_histo.items, .{});
        try codes.encoding_info.append(allocator, per_histo);
        const histo_writer = writer;
        const ans_cost = try buildAndStoreANSEncodingData(
            allocator,
            codes.encoding_info.items[codes.encoding_info.items.len - 1].items,
            params.ans_histogram_strategy,
            codes.use_prefix_code,
            codes.log_alpha_size,
            &clustered_histograms.items[c],
            histo_writer,
        );
        cost += ans_cost;
    }
    return cost;
}

// ---------------------------------------------------------------------------
// WriteTokens (enc_ans.cc)
// ---------------------------------------------------------------------------

fn addBitsReverse(
    allocator: std.mem.Allocator,
    out: *std.ArrayList(u64),
    out_nbits: *std.ArrayList(u8),
    allbits: *u64,
    numallbits: *usize,
    bits_v: usize,
    nbits: usize,
) !void {
    if (nbits != 0) {
        std.debug.assert(bits_v >> @intCast(nbits) == 0);
        if (numallbits.* + nbits > bit_io.kMaxBitsPerCall) {
            try out.append(allocator, allbits.*);
            try out_nbits.append(allocator, @intCast(numallbits.*));
            numallbits.* = 0;
            allbits.* = 0;
        }
        allbits.* <<= @intCast(nbits);
        allbits.* |= bits_v;
        numallbits.* += nbits;
    }
}

/// Port of `jxl::WriteTokens` (the non-status overload). Returns the number
/// of extra (non-token) bits written.
pub fn writeTokens(
    allocator: std.mem.Allocator,
    tokens: []const Token,
    codes: *const EntropyEncodingData,
    context_offset: usize,
    writer: *BitWriter,
) !usize {
    var num_extra_bits: usize = 0;
    if (codes.use_prefix_code) {
        for (tokens) |token| {
            const histo = codes.context_map.items[context_offset + token.context];
            const enc = if (token.is_lz77_length)
                &codes.lz77.length_uint_config
            else
                &codes.uint_config.items[histo];
            const e = enc.encode(token.value);
            var tok = e.token;
            if (token.is_lz77_length) tok += codes.lz77.min_symbol;
            const info = &codes.encoding_info.items[histo].items[tok];
            // Combine two calls to the BitWriter.
            const data: u64 = @as(u64, info.bits) | (@as(u64, e.bits) << @intCast(info.depth));
            writer.write(info.depth + e.nbits, data);
            num_extra_bits += e.nbits;
        }
        return num_extra_bits;
    }
    var out: std.ArrayList(u64) = .empty;
    defer out.deinit(allocator);
    var out_nbits: std.ArrayList(u8) = .empty;
    defer out_nbits.deinit(allocator);
    var allbits: u64 = 0;
    var numallbits: usize = 0;
    // Writes in *reversed* order.
    var ans = ANSCoder{};
    if (codes.lz77.enabled or codes.context_map.items.len > 1) {
        var i: isize = @intCast(tokens.len);
        while (i > 0) {
            i -= 1;
            const token = tokens[@intCast(i)];
            const histo = codes.context_map.items[context_offset + token.context];
            const enc = if (token.is_lz77_length)
                &codes.lz77.length_uint_config
            else
                &codes.uint_config.items[histo];
            const e = enc.encode(token.value);
            var tok = e.token;
            if (token.is_lz77_length) tok += codes.lz77.min_symbol;
            const info = &codes.encoding_info.items[histo].items[tok];
            std.debug.assert(info.freq > 0);
            // Extra bits first as this is reversed.
            try addBitsReverse(allocator, &out, &out_nbits, &allbits, &numallbits, e.bits, e.nbits);
            num_extra_bits += e.nbits;
            var ans_nbits: u8 = 0;
            const ans_bits = ans.putSymbol(info, &ans_nbits);
            try addBitsReverse(allocator, &out, &out_nbits, &allbits, &numallbits, ans_bits, ans_nbits);
        }
    } else {
        var i: isize = @intCast(tokens.len);
        while (i > 0) {
            i -= 1;
            const token = tokens[@intCast(i)];
            const e = codes.uint_config.items[0].encode(token.value);
            const info = &codes.encoding_info.items[0].items[e.token];
            // Extra bits first as this is reversed.
            try addBitsReverse(allocator, &out, &out_nbits, &allbits, &numallbits, e.bits, e.nbits);
            num_extra_bits += e.nbits;
            var ans_nbits: u8 = 0;
            const ans_bits = ans.putSymbol(info, &ans_nbits);
            try addBitsReverse(allocator, &out, &out_nbits, &allbits, &numallbits, ans_bits, ans_nbits);
        }
    }
    const state = ans.state;
    writer.write(32, state);
    writer.write(numallbits, allbits);
    var j = out.items.len;
    while (j > 0) : (j -= 1) {
        writer.write(out_nbits.items[j - 1], out.items[j - 1]);
    }
    return num_extra_bits;
}

// ---------------------------------------------------------------------------
// EncodeHistograms (streaming mode, enc_ans.cc)
// ---------------------------------------------------------------------------

/// Writes the context map to the bitstream and concatenates the individual
/// histogram bitstreams in codes.encoded_histograms. Used in streaming mode.
pub fn encodeHistograms(allocator: std.mem.Allocator, codes: *const EntropyEncodingData, writer: *BitWriter) !void {
    try codes.lz77.write(writer);
    if (codes.lz77.enabled) {
        encodeUintConfig(codes.lz77.length_uint_config, writer, 8);
    }
    const enc_context_map = @import("enc_context_map.zig");
    try enc_context_map.encodeContextMap(allocator, codes.context_map.items, codes.encoding_info.items.len, writer);
    writer.write(1, @intFromBool(codes.use_prefix_code));
    var log_alpha_size: usize = 8;
    if (codes.use_prefix_code) {
        log_alpha_size = prefix_max_bits;
    } else {
        log_alpha_size = 8; // streaming_mode
        writer.write(2, log_alpha_size - 5);
    }
    encodeUintConfigs(codes.uint_config.items, writer, log_alpha_size);
    if (codes.use_prefix_code) {
        for (codes.encoding_info.items) |*info| {
            fields.storeVarLenUint16(info.items.len - 1, writer);
        }
    }
    for (codes.encoded_histograms.items) |*histo_writer| {
        writer.appendUnaligned(histo_writer);
    }
}

// ---------------------------------------------------------------------------
// BuildAndEncodeHistograms (enc_ans.cc)
// ---------------------------------------------------------------------------

/// Port of `jxl::BuildAndEncodeHistograms`. Applies context clustering,
/// computes histograms and encodes them. Returns an estimate of the total
/// bits used. `tokens` is modified in place if LZ77 is applied.
pub fn buildAndEncodeHistograms(
    allocator: std.mem.Allocator,
    params: *const HistogramParams,
    num_contexts_in: usize,
    tokens: []std.ArrayList(Token),
    codes: *EntropyEncodingData,
    writer: ?*BitWriter,
) ans_common.EncodeError!usize {
    var num_contexts = num_contexts_in;
    codes.lz77.nonserialized_distance_context = @intCast(num_contexts);
    codes.lz77.min_symbol = if (params.force_huffman) 512 else 224;

    const enc_lz77 = @import("enc_lz77.zig");
    const tokens_lz77 = try enc_lz77.applyLZ77(allocator, params, num_contexts, tokens, &codes.lz77);
    var owned_lz77 = tokens_lz77;
    defer {
        // If not adopted below, free the processed streams.
        if (owned_lz77.len > 0 and !codes.lz77.enabled) {
            for (owned_lz77) |*l| l.deinit(allocator);
        }
        if (owned_lz77.len > 0) allocator.free(owned_lz77);
        owned_lz77 = &.{};
    }
    if (tokens_lz77.len > 0) codes.lz77.enabled = true;

    var cost: usize = 0;
    const max_contexts = @min(num_contexts, enc_cluster.k_clusters_limit);

    if (writer) |w| {
        codes.lz77.write(w);
    } else {
        cost += codes.lz77.bitSize();
    }
    if (codes.lz77.enabled) {
        if (writer) |w| {
            const b = w.bitsWritten();
            encodeUintConfig(codes.lz77.length_uint_config, w, 8);
            cost += w.bitsWritten() - b;
        } else {
            var size_writer = SizeWriter{};
            encodeUintConfig(codes.lz77.length_uint_config, &size_writer, 8);
            cost += size_writer.size;
        }
        num_contexts += 1;
        // tokens = std::move(tokens_lz77)
        for (tokens, 0..) |*stream, i| {
            stream.deinit(allocator);
            stream.* = tokens_lz77[i];
        }
    }

    var total_tokens: usize = 0;
    // Build histograms.
    var builder: std.ArrayList(Histogram) = .empty;
    defer {
        for (builder.items) |*h| h.deinit();
        builder.deinit(allocator);
    }
    try builder.resize(allocator, num_contexts);
    for (builder.items) |*h| h.* = Histogram.init(allocator);
    const token_uint_config = params.uintConfig();
    for (tokens) |stream| {
        if (codes.lz77.enabled) {
            for (stream.items) |token| {
                total_tokens += 1;
                const enc = if (token.is_lz77_length) &codes.lz77.length_uint_config else &token_uint_config;
                const e = enc.encode(token.value);
                var tok = e.token;
                if (token.is_lz77_length) tok += codes.lz77.min_symbol;
                try builder.items[token.context].add(allocator, tok);
            }
        } else if (num_contexts == 1) {
            for (stream.items) |token| {
                total_tokens += 1;
                const e = token_uint_config.encode(token.value);
                try builder.items[0].add(allocator, e.token);
            }
        } else {
            for (stream.items) |token| {
                total_tokens += 1;
                const e = token_uint_config.encode(token.value);
                try builder.items[token.context].add(allocator, e.token);
            }
        }
    }

    if (params.add_missing_symbols) {
        for (0..num_contexts) |c| {
            for (0..ans_max_alphabet_size) |symbol| {
                try builder.items[c].add(allocator, symbol);
            }
        }
    }

    if (params.initialize_global_state) {
        var use_prefix_code = params.force_huffman or total_tokens < 100 or
            params.clustering == .kFastest;
        if (!use_prefix_code) {
            var all_singleton = true;
            for (0..num_contexts) |i| {
                if (builder.items[i].shannonEntropy() >= 1e-5) {
                    all_singleton = false;
                }
            }
            if (all_singleton) {
                use_prefix_code = true;
            }
        }
        codes.use_prefix_code = use_prefix_code;
    }

    if (params.add_fixed_histograms) {
        const alphabet_size = ans_max_alphabet_size;
        codes.log_alpha_size = 8;
        var per_histo: std.ArrayList(ANSEncSymbolInfo) = .empty;
        try per_histo.resize(allocator, alphabet_size);
        @memset(per_histo.items, .{});
        try codes.encoding_info.append(allocator, per_histo);
        const histo_writer = BitWriter.init(allocator);
        try codes.encoded_histograms.append(allocator, histo_writer);
        var flat = Histogram.flat(allocator, @intCast(alphabet_size), @intCast(ans_tab_size));
        defer flat.deinit();
        _ = try buildAndStoreANSEncodingData(
            allocator,
            codes.encoding_info.items[codes.encoding_info.items.len - 1].items,
            params.ans_histogram_strategy,
            codes.use_prefix_code,
            codes.log_alpha_size,
            &flat,
            &codes.encoded_histograms.items[codes.encoded_histograms.items.len - 1],
        );
    }

    // Encode histograms.
    const entropy_bits = try buildAndStoreEntropyCodes(allocator, codes, params, tokens, builder.items, writer);
    cost += entropy_bits;
    _ = max_contexts;
    return cost;
}
