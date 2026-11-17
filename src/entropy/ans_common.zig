//! Common ANS definitions: parameters, hybrid-uint configuration, alias
//! tables. Bit-exact port of libjxl `ans_params.h`, `ans_common.h/cc` and
//! the `HybridUintConfig` part of `dec_ans.h`.

const std = @import("std");
const bits = @import("../base/bits.zig");

// ---------------------------------------------------------------------------
// ans_params.h
// ---------------------------------------------------------------------------

/// Probability precision in bits (libjxl: ANS_LOG_TAB_SIZE).
pub const ans_log_tab_size: u32 = 12;
/// Total size of the ANS table (libjxl: ANS_TAB_SIZE).
pub const ans_tab_size: u32 = 1 << ans_log_tab_size;
/// Mask of the table (libjxl: ANS_TAB_MASK).
pub const ans_tab_mask: u32 = ans_tab_size - 1;

/// Largest possible symbol to be encoded by either ANS or prefix coding.
pub const prefix_max_alphabet_size: usize = 4096;
pub const ans_max_alphabet_size: usize = 256;

/// Max number of bits for prefix coding.
pub const prefix_max_bits: usize = 15;

/// Initial state, used as CRC (libjxl: ANS_SIGNATURE). The final ANS state
/// of a stream is always `ans_signature << 16`.
pub const ans_signature: u32 = 0x13;
pub const ans_final_state: u32 = ans_signature << 16;

/// Type of histogram counts (libjxl: ANSHistBin).
pub const ANSHistBin = i32;

/// Common error set for the encoder pipeline (breaks inferred-error-set
/// dependency cycles between enc_ans / enc_cluster / enc_context_map).
pub const EncodeError = error{
    OutOfMemory,
    InvalidData,
    TooManyEntries,
    RebalanceFailed,
    NoFeasibleSelector,
    TableTooSmall,
};

/// Error set for the decoder pipeline (breaks the inferred-error-set cycle
/// between decodeHistograms and decodeContextMap).
pub const DecodeError = error{
    OutOfMemory,
    InvalidData,
    NotEnoughBytes,
    TableTooSmall,
};

// ---------------------------------------------------------------------------
// GetPopulationCountPrecision / CreateFlatHistogram (ans_common.h)
// ---------------------------------------------------------------------------

/// Returns the precision (number of bits) that should be used to store
/// a histogram count such that log2floor(count) == logcount.
pub fn getPopulationCountPrecision(logcount: u32, shift: u32) u32 {
    const r: i32 = @min(
        @as(i32, @intCast(logcount)),
        @as(i32, @intCast(shift)) - @as(i32, @intCast((ans_log_tab_size - logcount) >> 1)),
    );
    if (r < 0) return 0;
    return @intCast(r);
}

/// Returns a histogram where the counts are positive, differ by at most 1,
/// and add up to total_count. The bigger counts (if any) are at the beginning.
pub fn createFlatHistogram(allocator: std.mem.Allocator, length: i32, total_count: i32) ![]ANSHistBin {
    std.debug.assert(length > 0);
    std.debug.assert(length <= total_count);
    const count = @divTrunc(total_count, length);
    const result = try allocator.alloc(ANSHistBin, @intCast(length));
    @memset(result, count);
    const rem_counts = @mod(total_count, length);
    for (0..@intCast(rem_counts)) |i| {
        result[i] += 1;
    }
    return result;
}

// ---------------------------------------------------------------------------
// HybridUintConfig (dec_ans.h)
// ---------------------------------------------------------------------------

/// Hybrid-uint encoding configuration. Numbers below `split_token` are
/// encoded directly as tokens; larger numbers are split into a token
/// (containing (bits - split_exponent) + msb_in_token most significant
/// mantissa bits + lsb_in_token least significant bits) and extra data bits.
pub const HybridUintConfig = struct {
    split_exponent: u32 = 4,
    split_token: u32 = 16, // 1 << split_exponent
    msb_in_token: u32 = 2,
    lsb_in_token: u32 = 0,

    pub fn init(split_exponent: u32, msb_in_token: u32, lsb_in_token: u32) HybridUintConfig {
        std.debug.assert(split_exponent >= msb_in_token + lsb_in_token);
        return .{
            .split_exponent = split_exponent,
            .split_token = @as(u32, 1) << @intCast(split_exponent),
            .msb_in_token = msb_in_token,
            .lsb_in_token = lsb_in_token,
        };
    }

    /// Default config (libjxl: HybridUintConfig(4, 2, 0)).
    pub const default: HybridUintConfig = .{ .split_exponent = 4, .split_token = 16, .msb_in_token = 2, .lsb_in_token = 0 };

    pub const kHybridUint420Config = default;
    pub const kHybridUint000Config: HybridUintConfig = .{ .split_exponent = 0, .split_token = 1, .msb_in_token = 0, .lsb_in_token = 0 };
    pub const kHybridUint201Config: HybridUintConfig = .{ .split_exponent = 2, .split_token = 4, .msb_in_token = 0, .lsb_in_token = 1 };

    pub const Encoded = struct {
        token: u32,
        nbits: u32,
        bits: u32,
    };

    /// Splits `value` into a token and (nbits, bits) of extra data.
    pub fn encode(self: HybridUintConfig, value: u32) Encoded {
        if (value < self.split_token) {
            return .{ .token = value, .nbits = 0, .bits = 0 };
        } else {
            const n: u32 = @intCast(bits.floorLog2Nonzero(value));
            const m: u32 = value - (@as(u32, 1) << @intCast(n));
            const token: u32 = self.split_token +
                ((n - self.split_exponent) << @intCast(self.msb_in_token + self.lsb_in_token)) +
                ((m >> @intCast(n - self.msb_in_token)) << @intCast(self.lsb_in_token)) +
                (m & ((@as(u32, 1) << @intCast(self.lsb_in_token)) - 1));
            const nbits: u32 = n - self.msb_in_token - self.lsb_in_token;
            const bits_: u32 = @intCast((@as(u64, value) >> @intCast(self.lsb_in_token)) & ((@as(u64, 1) << @intCast(nbits)) - 1));
            return .{ .token = token, .nbits = nbits, .bits = bits_ };
        }
    }

    pub fn lsbMask(self: HybridUintConfig) u32 {
        return (@as(u32, 1) << @intCast(self.lsb_in_token)) - 1;
    }
};

// ---------------------------------------------------------------------------
// AliasTable (ans_common.h / ans_common.cc)
// ---------------------------------------------------------------------------

/// Alias table entry. Packed exactly like `AliasTable::Entry` in libjxl
/// (`#pragma pack(push, 1)`), so its little-endian 8-byte representation
/// matches libjxl's on x86-64.
pub const AliasEntry = packed struct(u64) {
    cutoff: u8, // < entry_size when used by ANS
    right_value: u8, // < alphabet size
    freq0: u16,
    /// Only used if pos >= cutoff (see lookup).
    offsets1: u16, // <= ans_tab_size
    freq1_xor_freq0: u16, // for branchless ternary in lookup
};

pub const AliasSymbol = struct {
    value: usize,
    offset: usize,
    freq: usize,
};

/// Dividing `value` by `entry_size` determines `i`, the entry which is
/// responsible for the input. If the remainder is below `cutoff`, the mapped
/// symbol is `i`, otherwise `right_value`.
pub fn aliasLookup(table: []const AliasEntry, value: usize, log_entry_size: u6, entry_size_minus_1: usize) AliasSymbol {
    const i = value >> log_entry_size;
    const pos = value & entry_size_minus_1;
    const entry = table[i];
    const greater = pos >= entry.cutoff;

    return .{
        .value = if (greater) entry.right_value else i,
        .offset = (if (greater) entry.offsets1 else 0) + pos,
        .freq = if (greater) (entry.freq0 ^ entry.freq1_xor_freq0) else entry.freq0,
    };
}

/// Computes an alias table for a given distribution. Port of
/// `jxl::InitAliasTable`.
///
/// `distribution` is copied; it is not modified. The caller provides the
/// output table of `1 << log_alpha_size` entries.
pub fn initAliasTable(
    allocator: std.mem.Allocator,
    distribution_in: []const ANSHistBin,
    log_range: u32,
    log_alpha_size: u6,
    a: []AliasEntry,
) !void {
    const range = @as(u32, 1) << @intCast(log_range);
    const table_size = @as(usize, 1) << log_alpha_size;
    if (table_size > range) return error.InvalidData;

    // Copy, dropping trailing zeros.
    var dist: std.ArrayList(ANSHistBin) = .empty;
    defer dist.deinit(allocator);
    try dist.appendSlice(allocator, distribution_in);
    while (dist.items.len > 0 and dist.items[dist.items.len - 1] == 0) {
        _ = dist.pop();
    }
    // Ensure that a valid table is always returned, even for an empty
    // alphabet (specially-crafted stream crash protection).
    if (dist.items.len == 0) {
        try dist.append(allocator, @intCast(range));
    }
    if (dist.items.len > table_size) return error.InvalidData;

    const entry_size: u32 = range >> @as(u5, @intCast(log_alpha_size)); // exact
    var single_symbol: i32 = -1;
    var sum: i64 = 0;
    // Special case for single-symbol distributions: ensures the state does
    // not change when decoding from such a distribution.
    for (dist.items, 0..) |v, sym| {
        sum += v;
        if (v == ans_tab_size) {
            if (single_symbol != -1) return error.InvalidData;
            single_symbol = @intCast(sym);
        }
    }
    if (sum != range) return error.InvalidData;

    if (single_symbol != -1) {
        const sym: u8 = @intCast(single_symbol);
        for (0..table_size) |i| {
            a[i] = .{
                .right_value = sym,
                .cutoff = 0,
                .offsets1 = @intCast(entry_size * @as(u32, @intCast(i))),
                .freq0 = 0,
                .freq1_xor_freq0 = @intCast(ans_tab_size),
            };
        }
        return;
    }

    var underfull_posn: std.ArrayList(u32) = .empty;
    defer underfull_posn.deinit(allocator);
    var overfull_posn: std.ArrayList(u32) = .empty;
    defer overfull_posn.deinit(allocator);
    const cutoffs = try allocator.alloc(u32, table_size);
    defer allocator.free(cutoffs);
    @memset(cutoffs, 0);

    // Initialize entries.
    for (dist.items, 0..) |d, i| {
        cutoffs[i] = @intCast(d);
        if (cutoffs[i] > entry_size) {
            try overfull_posn.append(allocator, @intCast(i));
        } else if (cutoffs[i] < entry_size) {
            try underfull_posn.append(allocator, @intCast(i));
        }
    }
    for (dist.items.len..table_size) |i| {
        cutoffs[i] = 0;
        try underfull_posn.append(allocator, @intCast(i));
    }

    // Reassign overflow/underflow values.
    while (overfull_posn.items.len > 0) {
        const overfull_i = overfull_posn.pop().?;
        if (underfull_posn.items.len == 0) return error.InvalidData;
        const underfull_i = underfull_posn.pop().?;
        const underfull_by = entry_size - cutoffs[underfull_i];
        cutoffs[overfull_i] -= underfull_by;
        // Overfull positions have their original symbols.
        a[underfull_i].right_value = @intCast(overfull_i);
        a[underfull_i].offsets1 = @intCast(cutoffs[overfull_i]);
        // Slots in the right part of entry underfull_i were taken from the
        // end of the symbols in entry overfull_i.
        if (cutoffs[overfull_i] < entry_size) {
            try underfull_posn.append(allocator, overfull_i);
        } else if (cutoffs[overfull_i] > entry_size) {
            try overfull_posn.append(allocator, overfull_i);
        }
    }

    for (0..table_size) |i| {
        if (cutoffs[i] == entry_size) {
            a[i].right_value = @intCast(i);
            a[i].offsets1 = 0;
            a[i].cutoff = 0;
        } else {
            // a[i].offsets1 was initialized with (overfull cutoff) -
            // (entry_size - a[i].cutoff); subtracting a[i].cutoff cannot
            // make it negative.
            a[i].offsets1 -= @intCast(cutoffs[i]);
            a[i].cutoff = @intCast(cutoffs[i]);
        }
        const freq0: usize = if (i < dist.items.len) @intCast(dist.items[i]) else 0;
        const right_sym: usize = a[i].right_value;
        const freq1: usize = if (right_sym < dist.items.len) @intCast(dist.items[right_sym]) else 0;
        a[i].freq0 = @intCast(freq0);
        a[i].freq1_xor_freq0 = @intCast(freq1 ^ freq0);
    }
}

// ---------------------------------------------------------------------------

test "getPopulationCountPrecision" {
    // Port of libjxl checks in enc_ans.cc.
    try std.testing.expectEqual(@as(u32, 0), getPopulationCountPrecision(0, 0));
    // shift=0: bitcount always 0 for any logcount
    try std.testing.expectEqual(@as(u32, 0), getPopulationCountPrecision(5, 0));
    // shift=12: full precision
    try std.testing.expectEqual(@as(u32, 5), getPopulationCountPrecision(5, 12));
    // shift=6, logcount=4: 6 - (12-4)/2 = 2
    try std.testing.expectEqual(@as(u32, 2), getPopulationCountPrecision(4, 6));
}

test "createFlatHistogram" {
    const alloc = std.testing.allocator;
    const h = try createFlatHistogram(alloc, 5, 13);
    defer alloc.free(h);
    try std.testing.expectEqualSlices(i32, &.{ 3, 3, 3, 2, 2 }, h);
}

test "hybrid uint encode roundtrip semantics" {
    // From the libjxl comment: split_exponent=4, msb=2, lsb=0.
    const cfg = HybridUintConfig.init(4, 2, 0);
    const cases = [_]struct { v: u32, tok: u32, nbits: u32, bits: u32 }{
        .{ .v = 0, .tok = 0, .nbits = 0, .bits = 0 },
        .{ .v = 15, .tok = 15, .nbits = 0, .bits = 0 },
        .{ .v = 16, .tok = 16, .nbits = 2, .bits = 0 }, // 10000
        .{ .v = 17, .tok = 16, .nbits = 2, .bits = 1 }, // 10001
        .{ .v = 20, .tok = 17, .nbits = 2, .bits = 0 }, // 10100
        .{ .v = 24, .tok = 18, .nbits = 2, .bits = 0 }, // 11000
        .{ .v = 28, .tok = 19, .nbits = 2, .bits = 0 }, // 11100
        .{ .v = 32, .tok = 20, .nbits = 3, .bits = 0 }, // 100000
        .{ .v = 65535, .tok = 63, .nbits = 13, .bits = 0x1FFF },
    };
    for (cases) |c| {
        const e = cfg.encode(c.v);
        try std.testing.expectEqual(c.tok, e.token);
        try std.testing.expectEqual(c.nbits, e.nbits);
        try std.testing.expectEqual(c.bits, e.bits);
    }
}

test "alias table basic distribution" {
    const alloc = std.testing.allocator;
    const log_alpha_size: u6 = 8; // 256 entries, entry_size 16
    const table = try alloc.alloc(AliasEntry, 1 << log_alpha_size);
    defer alloc.free(table);

    const dist = [_]ANSHistBin{ 4096 / 2, 4096 / 4, 4096 / 4 };
    try initAliasTable(alloc, &dist, ans_log_tab_size, log_alpha_size, table);

    // Every value in [0, 4096) maps to the right symbol with right offset.
    var histogram: [3]usize = .{ 0, 0, 0 };
    const log_entry_size: u6 = @intCast(ans_log_tab_size - log_alpha_size);
    for (0..4096) |v| {
        const s = aliasLookup(table, v, log_entry_size, (1 << log_entry_size) - 1);
        try std.testing.expect(s.freq > 0);
        try std.testing.expect(s.offset < s.freq);
        histogram[s.value] += 1;
    }
    try std.testing.expectEqualSlices(usize, &.{ 2048, 1024, 1024 }, &histogram);
}

test "alias table single symbol" {
    const alloc = std.testing.allocator;
    const log_alpha_size: u6 = 5;
    const table = try alloc.alloc(AliasEntry, 1 << log_alpha_size);
    defer alloc.free(table);
    const dist = [_]ANSHistBin{0, 0, ans_tab_size};
    try initAliasTable(alloc, &dist, ans_log_tab_size, log_alpha_size, table);
    const log_entry_size: u6 = @intCast(ans_log_tab_size - log_alpha_size);
    for (0..4096) |v| {
        const s = aliasLookup(table, v, log_entry_size, (1 << log_entry_size) - 1);
        try std.testing.expectEqual(@as(usize, 2), s.value);
        try std.testing.expectEqual(@as(usize, v), s.offset);
        try std.testing.expectEqual(@as(usize, ans_tab_size), s.freq);
    }
}

test "alias table empty distribution is degenerate symbol 0" {
    const alloc = std.testing.allocator;
    const log_alpha_size: u6 = 5;
    const table = try alloc.alloc(AliasEntry, 1 << log_alpha_size);
    defer alloc.free(table);
    try initAliasTable(alloc, &.{}, ans_log_tab_size, log_alpha_size, table);
    const log_entry_size: u6 = @intCast(ans_log_tab_size - log_alpha_size);
    const s = aliasLookup(table, 1234, log_entry_size, (1 << log_entry_size) - 1);
    try std.testing.expectEqual(@as(usize, 0), s.value);
    try std.testing.expectEqual(@as(usize, ans_tab_size), s.freq);
}
