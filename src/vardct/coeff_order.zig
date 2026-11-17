//! VarDCT coefficient orders: the `kOrderEnc` field, the natural (zig-zag)
//! order of 8x8 DCT coefficients, and the decoding of *used* (custom)
//! coefficient orders — ports of libjxl `frame_header.h` (kOrderEnc),
//! `coeff_order.h/cc` (DecodeCoeffOrders/DecodeCoeffOrder/ReadPermutation),
//! `lehmer_code.h` (DecodeLehmerCode) and
//! `AcStrategy::ComputeNaturalCoeffOrder` for the DCT-8 strategy.
//!
//! Custom orders are the shape libjxl/cjxl emits for JPEG-recompression
//! frames: the AC global section carries a `used_orders` bitfield; for each
//! set bit, a Lehmer-coded permutation reorders the 8x8 zig-zag scan per
//! color channel, tokenized with `kPermutationContexts` ANS histograms.
//! This port supports the DCT-8x8 order (bucket 0) — the only bucket the
//! JPEG-transcode subset can encounter (all blocks are 8x8).

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const bits = @import("../base/bits.zig");
const fields = @import("../base/fields.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const U32Enc = fields.U32Enc;
const val = fields.val;

const block_context = @import("block_context.zig");
const dec_ans = @import("../entropy/dec_ans.zig");
const enc_ans = @import("../entropy/enc_ans.zig");
const ans_common = @import("../entropy/ans_common.zig");
const stream_mod = @import("../modular/stream.zig");

const Allocator = std.mem.Allocator;

/// libjxl `kNumOrders` re-export (used by the order bitfield width).
pub const kNumOrders = block_context.kNumOrders;

/// libjxl `kOrderEnc`: value distribution for the used_orders bitfield.
pub const kOrderEnc = U32Enc.init(
    val(0x5F),
    val(0x13),
    val(0),
    fields.bitsDistr(@intCast(kNumOrders)),
);

/// libjxl `kPermutationContexts`: ANS contexts for the order tokens.
pub const kPermutationContexts: usize = 8;

/// Order bucket of the DCT-8x8 strategy (libjxl `kStrategyOrder[0]`).
pub const kDct8x8OrderBucket: u5 = 0;

/// Natural (zig-zag) coefficient order of an 8x8 block, computed by the
/// `CoeffOrderAndLut<false>` algorithm specialized to a single 8x8 block
/// (cx == cy == 1): position k holds the linear index of the k-th
/// coefficient visited in zig-zag order.
pub fn naturalOrder8x8() [64]u32 {
    var order: [64]u32 = undefined;
    var cur: usize = 1 * 1; // cx * cy
    // First half: diagonals i = 0..cx*kBlockDim-1 (8x8: i = 0..7).
    for (0..8) |i| {
        for (0..i + 1) |j| {
            var x = j;
            var y = i - j;
            if (i % 2 == 1) std.mem.swap(usize, &x, &y);
            const val_pos: u32 = if (x < 1 and y < 1)
                @intCast(y * 1 + x)
            else blk: {
                const v: u32 = @intCast(cur);
                cur += 1;
                break :blk v;
            };
            order[val_pos] = @intCast(y * 8 + x);
        }
    }
    // Second half: anti-diagonals from the bottom-right corner.
    var ip: usize = 8 - 1;
    while (ip > 0) : (ip -= 1) {
        const i = ip - 1;
        for (0..i + 1) |j| {
            var x = 8 - 1 - (i - j);
            var y = 8 - 1 - j;
            if (i % 2 == 1) std.mem.swap(usize, &x, &y);
            const val_pos: u32 = @intCast(cur);
            cur += 1;
            order[val_pos] = @intCast(y * 8 + x);
        }
    }
    return order;
}

// ---------------------------------------------------------------------------
// Custom ("used") coefficient order decoding
// ---------------------------------------------------------------------------

/// Port of `CoeffOrderContext`: the ANS context for an order token is the
/// `HybridUintConfig(0,0,0)` token of its value, capped at
/// kPermutationContexts-1.
fn coeffOrderContext(value: u32) usize {
    const token = ans_common.HybridUintConfig.kHybridUint000Config.encode(
        value,
    ).token;
    return @min(token, kPermutationContexts - 1);
}

/// `ValueOfLowest1Bit(i)` = value (2^k) of the lowest set bit of i>0.
fn lowestBitValue(x: u32) u32 {
    return x & (0 -% x);
}

/// Port of `DecodeLehmerCode` (lehmer_code.h). `temp` must have at least
/// `1 << ceilLog2(n)` elements; `permutation` receives the decoded
/// permutation of 0..n-1.
fn decodeLehmerCode(
    code: []const u32,
    temp: []u32,
    n: usize,
    permutation: []u32,
) void {
    std.debug.assert(n != 0);
    const log2n = bits.ceilLog2Nonzero(n);
    const padded_n = @as(usize, 1) << @intCast(log2n);

    for (0..padded_n) |i| {
        temp[i] = lowestBitValue(@intCast(i + 1));
    }

    for (0..n) |i| {
        var rank: u32 = code[i] + 1;

        // Extract i-th unused element via implicit order-statistics tree.
        var bit: usize = padded_n;
        var next: usize = 0;
        var b: usize = 0;
        while (b <= log2n) : (b += 1) {
            const cand = next + bit;
            bit >>= 1;
            if (temp[cand - 1] < rank) {
                next = cand;
                rank -= temp[cand - 1];
            }
        }
        permutation[i] = @intCast(next);

        // Mark as used.
        var idx = next + 1;
        while (idx <= padded_n) {
            temp[idx - 1] -= 1;
            idx += lowestBitValue(@intCast(idx));
        }
    }
}

/// Port of `ReadPermutation`: reads a Lehmer-coded permutation of
/// `size` elements (only positions `skip..end` are transmitted).
/// `order` may be null to consume (and discard) the permutation.
fn readPermutation(
    allocator: Allocator,
    skip: usize,
    size: usize,
    order: ?[]u32,
    br: *BitReader,
    reader: *dec_ans.ANSSymbolReader,
    context_map: []const u8,
) !void {
    const lehmer = try allocator.alloc(u32, size);
    defer allocator.free(lehmer);
    @memset(lehmer, 0);
    // temp space needs to be as large as the next power of 2, so doubling the
    // allocated size is enough.
    const temp = try allocator.alloc(u32, size * 2);
    defer allocator.free(temp);

    const end = reader.readHybridUint(
        coeffOrderContext(@intCast(size)),
        br,
        context_map,
    ) + skip;
    if (end > size) return error.InvalidPermutationSize;
    var last: u32 = 0;
    var i = skip;
    while (i < end) : (i += 1) {
        lehmer[i] = reader.readHybridUint(
            coeffOrderContext(last),
            br,
            context_map,
        );
        last = lehmer[i];
        if (lehmer[i] >= size - i) return error.InvalidLehmerCode;
    }
    if (order == null) return;
    decodeLehmerCode(lehmer, temp, size, order.?);
}

/// Port of libjxl `DecodePermutation` (coeff_order.cc): a standalone
/// Lehmer-coded permutation with its own `kPermutationContexts` ANS
/// histograms — used by the TOC (`ReadToc`) to permute the section
/// order (`--group_order=1` / center-first files) and by the
/// coefficient-order machinery. `order` receives a permutation of
/// 0..size-1.
pub fn decodePermutation(
    allocator: Allocator,
    skip: usize,
    size: usize,
    order: []u32,
    br: *BitReader,
) !void {
    var code = dec_ans.ANSCode{};
    defer code.deinit(allocator);
    var context_map: std.ArrayList(u8) = .empty;
    defer context_map.deinit(allocator);
    try dec_ans.decodeHistograms(
        allocator,
        br,
        kPermutationContexts,
        &code,
        &context_map,
        false,
    );
    var reader = try dec_ans.ANSSymbolReader.create(
        allocator,
        &code,
        br,
        0,
    );
    defer reader.deinit();
    try readPermutation(
        allocator,
        skip,
        size,
        order,
        br,
        &reader,
        context_map.items,
    );
    if (!reader.checkANSFinalState()) return error.InvalidANSStream;
}

/// Port of libjxl `EncodePermutation` (enc_coeff_order.cc): the inverse of
/// `decodePermutation` — tokenizes the Lehmer code of `order` and writes it
/// as one `kPermutationContexts` ANS stream. Used by the TOC-permutation
/// tests (the encoder itself writes unpermuted TOCs, like libjxl's
/// `WriteToc` without a permutation).
pub fn encodePermutation(
    allocator: Allocator,
    skip: usize,
    size: usize,
    order: []const u32,
    writer: *BitWriter,
) !void {
    var streams: [1]std.ArrayList(enc_ans.Token) = .{.empty};
    defer streams[0].deinit(allocator);
    try tokenizePermutation(
        allocator,
        order[0..size],
        skip,
        &streams[0],
    );
    var codes = enc_ans.EntropyEncodingData.init(
        allocator,
    );
    defer codes.deinit();
    const params = enc_ans.HistogramParams{};
    _ = try enc_ans.buildAndEncodeHistograms(
        allocator,
        &params,
        kPermutationContexts,
        streams[0..],
        &codes,
        writer,
    );
    _ = try enc_ans.writeTokens(
        allocator,
        streams[0].items,
        &codes,
        0,
        writer,
    );
}

/// Decodes the `used_orders` block of the AC global section for the
/// JPEG-transcode subset (all blocks 8x8, so `used_acs == 1`): fills
/// `orders` (one [64]u32 scan order per color channel). Mirrors libjxl
/// `DecodeCoeffOrders`; order buckets other than the DCT-8x8 one are
/// rejected (they require varblocks this codec does not support).
pub fn decodeCoeffOrders(
    allocator: Allocator,
    used_orders: u32,
    orders: *[3][64]u32,
    br: *BitReader,
) !void {
    // Only the DCT-8x8 bucket can occur in a JPEG-transcode frame; any
    // other used order implies varblocks (unsupported elsewhere already).
    if (used_orders & ~@as(u32, 1) != 0) return error.UnsupportedVarDct;

    var code = dec_ans.ANSCode{};
    defer code.deinit(allocator);
    var context_map: std.ArrayList(u8) = .empty;
    defer context_map.deinit(allocator);
    var reader: ?dec_ans.ANSSymbolReader = null;
    defer if (reader) |*r| r.deinit();

    // The bitstream carries histograms only if some order is used.
    if (used_orders != 0) {
        try dec_ans.decodeHistograms(
            allocator,
            br,
            kPermutationContexts,
            &code,
            &context_map,
            false,
        );
        reader = try dec_ans.ANSSymbolReader.create(
            allocator,
            &code,
            br,
            0,
        );
    }

    const natural = naturalOrder8x8();

    if (used_orders & 1 != 0) {
        // DCT-8x8: llf = 1, size = 64; one custom order per channel.
        for (0..3) |c| {
            var perm: [64]u32 = undefined;
            try readPermutation(
                allocator,
                1,
                64,
                &perm,
                br,
                &reader.?,
                context_map.items,
            );
            for (0..64) |k| {
                orders[c][k] = natural[perm[k]];
            }
        }
    } else {
        for (0..3) |c| {
            orders[c] = natural;
        }
    }
    if (used_orders != 0) {
        if (!reader.?.checkANSFinalState()) return error.InvalidANSStream;
    }
}

// ---------------------------------------------------------------------------
// Custom ("used") coefficient order encoding
// (ports of enc_coeff_order.cc ComputeUsedOrders/ComputeCoeffOrder/
//  EncodeCoeffOrders/TokenizePermutation and lehmer_code.h
//  ComputeLehmerCode), specialized to the JPEG-transcode subset: single
//  pass, DCT-8 blocks only, cjxl default effort (kSquirrel).
// ---------------------------------------------------------------------------

/// Port of `ComputeLehmerCode` (lehmer_code.h): factorial-basis code of a
/// permutation of n unique indices in [0..n). `temp` must have n + 1
/// elements. N log N Fenwick-tree version.
pub fn computeLehmerCode(
    permutation: []const u32,
    temp: []u32,
    n: usize,
    code: []u32,
) void {
    @memset(temp[0 .. n + 1], 0);
    for (0..n) |idx| {
        const s = permutation[idx];
        // Compute sum in Fenwick tree.
        var penalty: u32 = 0;
        var i: usize = s + 1;
        while (i != 0) : (i &= i - 1) {
            penalty += temp[i];
        }
        std.debug.assert(s >= penalty);
        code[idx] = s - penalty;
        // Add operation in Fenwick tree.
        i = s + 1;
        while (i < n + 1) : (i += lowestBitValue(@intCast(i))) {
            temp[i] += 1;
        }
    }
}

/// Port of `TokenizePermutation` (enc_coeff_order.cc): the Lehmer code of
/// `order` (a permutation in zig-zag-index space), with the trailing zero
/// digits trimmed; the count of transmitted digits is tokenized first.
fn tokenizePermutation(
    allocator: Allocator,
    order: []const u32,
    skip: usize,
    tokens: *std.ArrayList(enc_ans.Token),
) !void {
    const size = order.len;
    const lehmer = try allocator.alloc(u32, size);
    defer allocator.free(lehmer);
    const temp = try allocator.alloc(u32, size + 1);
    defer allocator.free(temp);
    computeLehmerCode(order, temp, size, lehmer);
    var end = size;
    while (end > skip and lehmer[end - 1] == 0) end -= 1;
    try tokens.append(
        allocator,
        .{
            .context = @intCast(coeffOrderContext(@intCast(size))),
            .value = @intCast(end - skip),
        },
    );
    var last: u32 = 0;
    var i = skip;
    while (i < end) : (i += 1) {
        try tokens.append(
            allocator,
            .{
                .context = @intCast(coeffOrderContext(last)),
                .value = lehmer[i],
            },
        );
        last = lehmer[i];
    }
}

/// xorshift128+ with the fixed seeds of `ComputeCoeffOrder`
/// (enc_coeff_order.cc); `use_sample` keeps the exact libjxl semantics:
/// the compared bits are the pre-update state sum.
const OrderSampler = struct {
    s: [2]u64 = .{ 0x94D049BB133111EB, 0xBF58476D1CE4E5B9 },

    fn useSample(self: *OrderSampler, threshold: u64) bool {
        var s1 = self.s[0];
        const s0 = self.s[1];
        const sum_bits = s1 +% s0;
        self.s[0] = s0;
        s1 ^= s1 << 23;
        s1 ^= s0 ^ (s1 >> 18) ^ (s0 >> 5);
        self.s[1] = s1;
        return (sum_bits >> 32) <= threshold;
    }
};

/// Result of the order heuristic: the per-channel scan orders and the
/// `used_orders` bitfield value to serialize (bit 0 set only when some
/// channel's order differs from the natural one, mirroring the
/// `is_nondefault` clearing in ComputeCoeffOrder).
pub const CoeffOrderResult = struct {
    orders: [3][64]u32,
    used_orders: u32,
};

/// Per-channel full-frame coefficient plane view for the heuristic.
pub const CoeffPlanes = struct {
    /// Block-major transposed (JXL-order) coefficient planes.
    data: [3][]const i32 = .{ &.{}, &.{}, &.{} },
    /// Per-channel width in blocks.
    widths: [3]usize = .{ 0, 0, 0 },
};

/// Port of `ComputeUsedOrders` + `ComputeCoeffOrder` for the
/// JPEG-transcode subset (all blocks DCT-8, single pass, kSquirrel speed).
///
/// The per-position zero counts are gathered over a 50% block sample drawn
/// by the fixed-seed xorshift128+ stream; exactly as libjxl, the sample
/// counter `ac_offset` advances only for sampled blocks and indexes every
/// channel's group storage uniformly (for subsampled layouts this reads
/// beyond the channel's written blocks - where libjxl reads whatever the
/// ACImage plane holds, this port reads deterministic zeros; the orders are
/// transmitted either way, so the codestream stays valid).
pub fn computeCoeffOrder(
    allocator: Allocator,
    fd: *const stream_mod.FrameDim,
    hshift: [3]usize,
    vshift: [3]usize,
    planes: CoeffPlanes,
) !CoeffOrderResult {
    const natural = naturalOrder8x8();
    // ComputeUsedOrders: default orders for small images (< 5x5 blocks in
    // both dimensions).
    if (fd.xsize_blocks < 5 and fd.ysize_blocks < 5) {
        return .{
            .orders = .{ natural, natural, natural },
            .used_orders = 0,
        };
    }

    // block_fraction = 0.5 at kSquirrel (and every faster tier) when
    // orders are enabled; threshold = (2^32 - 1) * 0.5, truncated.
    const threshold: u64 = 2147483647;
    var sampler = OrderSampler{};

    var num_zeros: [3][64]i64 = @splat(@splat(0));

    // Per-group contiguous channel storage: the channel's blocks in
    // full-grid (by, bx) scan order of the clipped 32x32-block group rect,
    // zero-filled beyond (libjxl's ACImage group planes are 32x32 blocks
    // per channel whatever the subsampling).
    const kGroupBlocks: usize = 32;
    var storage: [3][]i32 = .{
        try allocator.alloc(i32, kGroupBlocks * kGroupBlocks * 64),
        try allocator.alloc(i32, kGroupBlocks * kGroupBlocks * 64),
        try allocator.alloc(i32, kGroupBlocks * kGroupBlocks * 64),
    };
    defer {
        for (&storage) |s| allocator.free(s);
    }

    for (0..fd.num_groups) |g| {
        const rect = fd.blockGroupRect(g);
        for (0..3) |c| @memset(storage[c], 0);
        // Materialize each channel's blocks in full-grid scan order.
        for (0..3) |c| {
            var m: usize = 0;
            for (0..rect.ysize) |by| {
                if ((by >> @intCast(vshift[c])) << @intCast(vshift[c]) != by)
                    continue;
                const gy = (rect.y0 >> @intCast(vshift[c])) +
                    (by >> @intCast(vshift[c]));
                for (0..rect.xsize) |bx| {
                    if ((bx >> @intCast(hshift[c])) << @intCast(hshift[c]) != bx)
                        continue;
                    const gx = (rect.x0 >> @intCast(hshift[c])) +
                        (bx >> @intCast(hshift[c]));
                    const src = planes.data[c][(gy * planes.widths[c] + gx) * 64 ..][0..64];
                    @memcpy(storage[c][m * 64 ..][0..64], src);
                    m += 1;
                }
            }
        }

        // Sampled zero counting with the shared ac_offset.
        var ac_offset: usize = 0;
        for (0..rect.ysize) |_| {
            for (0..rect.xsize) |_| {
                // All blocks are DCT-8 first blocks.
                if (!sampler.useSample(threshold)) continue;
                for (0..3) |c| {
                    for (0..64) |k| {
                        if (storage[c][ac_offset + k] == 0)
                            num_zeros[c][k] += 1;
                    }
                    // Ensure the LLF is first in the order (DCT-8: pos 0).
                    num_zeros[c][0] = -1;
                }
                ac_offset += 64;
            }
        }
    }

    // Sort each channel's positions by quantized zero count (stable via
    // the index baked into the key, exactly like libjxl's count_and_idx).
    const PosAndCount = struct {
        pos: u32,
        count_and_idx: u64,
    };
    var pav: [64]PosAndCount = undefined;
    var orders: [3][64]u32 = undefined;
    var is_nondefault = false;
    for (0..3) |c| {
        const inv_sqrt_sz: f32 = 0.125; // 1/sqrt(64)
        for (0..64) |i| {
            const pos = natural[i];
            pav[i].pos = pos;
            // Quantize the count to reduce permutation churn
            // (num_zeros * inv_sqrt_sz + 0.1, truncated toward zero; the
            // forced LLF -1 lands at 0 exactly like the C++ cast).
            const f = @as(f32, @floatFromInt(num_zeros[c][pos])) * inv_sqrt_sz + 0.1;
            const t = @trunc(f);
            const count: u64 = if (t <= 0) 0 else @intFromFloat(t);
            pav[i].count_and_idx = (count << 16) | i;
        }
        std.mem.sort(PosAndCount, &pav, {}, struct {
            fn lt(_: void, a: PosAndCount, b: PosAndCount) bool {
                return a.count_and_idx < b.count_and_idx;
            }
        }.lt);
        for (0..64) |i| {
            orders[c][i] = pav[i].pos;
            if (natural[i] != pav[i].pos) is_nondefault = true;
        }
    }
    return .{
        .orders = orders,
        .used_orders = if (is_nondefault) 1 else 0,
    };
}

/// Port of `EncodeCoeffOrders` for the DCT-8 subset: serializes the
/// per-channel orders as Lehmer-coded permutations (kPermutationContexts
/// histograms with the default HistogramParams, then one ANS token stream).
/// `used_orders` must be 0 (nothing written) or 1 (DCT-8 bucket only).
pub fn encodeCoeffOrders(
    allocator: Allocator,
    used_orders: u32,
    orders: *const [3][64]u32,
    writer: *BitWriter,
) !void {
    if (used_orders & ~@as(u32, 1) != 0) return error.UnsupportedVarDct;
    // Do not write anything if no order is used.
    if (used_orders == 0) return;

    // natural_order_lut: linear position -> zig-zag scan index.
    const natural = naturalOrder8x8();
    var lut: [64]u32 = undefined;
    for (0..64) |k| lut[natural[k]] = @intCast(k);

    var streams: [1]std.ArrayList(enc_ans.Token) = .{.empty};
    defer streams[0].deinit(allocator);
    for (0..3) |c| {
        var order_zigzag: [64]u32 = undefined;
        for (0..64) |i| order_zigzag[i] = lut[orders[c][i]];
        try tokenizePermutation(
            allocator,
            &order_zigzag,
            1,
            &streams[0],
        );
    }

    // libjxl EncodeCoeffOrders: BuildAndEncodeHistograms with a default
    // HistogramParams (kBest clustering, kRLE LZ77, kPrecise), then the
    // tokens. NOTE: buildAndEncodeHistograms follows libjxl's
    // `tokens = std::move(tokens_lz77)` semantics — when LZ77 is adopted it
    // replaces the input lists in place, so the tokens written afterwards
    // must come from the same list storage (streams[0], not a copy).
    var codes = enc_ans.EntropyEncodingData.init(
        allocator,
    );
    defer codes.deinit();
    const params = enc_ans.HistogramParams{};
    _ = try enc_ans.buildAndEncodeHistograms(
        allocator,
        &params,
        kPermutationContexts,
        streams[0..],
        &codes,
        writer,
    );
    _ = try enc_ans.writeTokens(
        allocator,
        streams[0].items,
        &codes,
        0,
        writer,
    );
}

test "natural order matches JPEG zig-zag" {
    const jd = @import("../jpeg/jpeg_data.zig");
    const order = naturalOrder8x8();
    // k-th zig-zag step visits kJPEGNaturalOrder[k]; order is the inverse
    // permutation: position k of the scan order holds linear index
    // kJPEGNaturalOrder[k].
    for (jd.kJPEGNaturalOrder[0..64], 0..) |nat, k| {
        try std.testing.expectEqual(nat, order[k]);
    }
}

test "lehmer code encode is the inverse of decode" {
    const alloc = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(12345);
    const rnd = prng.random();
    const perm = try alloc.alloc(u32, 64);
    defer alloc.free(perm);
    const code = try alloc.alloc(u32, 64);
    defer alloc.free(code);
    const temp_enc = try alloc.alloc(u32, 65);
    defer alloc.free(temp_enc);
    const temp_dec = try alloc.alloc(u32, 128);
    defer alloc.free(temp_dec);
    const decoded = try alloc.alloc(u32, 64);
    defer alloc.free(decoded);
    var round: usize = 0;
    while (round < 50) : (round += 1) {
        for (perm, 0..) |*p, i| p.* = @intCast(i);
        rnd.shuffle(u32, perm);
        computeLehmerCode(perm, temp_enc, 64, code);
        decodeLehmerCode(code, temp_dec, 64, decoded);
        try std.testing.expectEqualSlices(
            u32,
            perm,
            decoded,
        );
    }
}

test "order permutation write/read roundtrip" {
    const alloc = std.testing.allocator;
    // A hand-made order: zig-zag but with positions 1 and 2 swapped and
    // the tail partially reversed (ensures a non-trivial Lehmer code).
    var orders: [3][64]u32 = undefined;
    for (0..3) |c| {
        orders[c] = naturalOrder8x8();
        std.mem.swap(u32, &orders[c][1], &orders[c][2]);
        std.mem.reverse(u32, orders[c][40..64]);
        // LLF must stay first (the decoder assumes lehmer[0] == 0).
        orders[c][0] = 0;
    }
    // Fix the permutation property after the surgery: rebuild from the
    // multiset of positions.
    for (0..3) |c| {
        var seen = [_]bool{false} ** 64;
        for (orders[c]) |pos| {
            try std.testing.expect(!seen[pos]);
            seen[pos] = true;
        }
    }

    var w = BitWriter.init(alloc);
    defer w.deinit();
    try encodeCoeffOrders(
        alloc,
        1,
        &orders,
        &w,
    );
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    var decoded: [3][64]u32 = undefined;
    try decodeCoeffOrders(
        alloc,
        1,
        &decoded,
        &br,
    );
    defer {
        // decodeCoeffOrders allocates nothing persistent on success.
    }
    for (0..3) |c| {
        try std.testing.expectEqualSlices(
            u32,
            &orders[c],
            &decoded[c],
        );
    }
    try br.close();
}

test "used_orders 0 writes nothing" {
    const alloc = std.testing.allocator;
    const natural = naturalOrder8x8();
    var w = BitWriter.init(alloc);
    defer w.deinit();
    try encodeCoeffOrders(
        alloc,
        0,
        &.{ natural, natural, natural },
        &w,
    );
    try std.testing.expectEqual(
        @as(usize, 0),
        w.bitsWritten(),
    );
}

test "order sampler matches the xorshift128+ reference sequence" {
    // Reference values computed from the fixed seeds
    // {0x94D049BB133111EB, 0xBF58476D1CE4E5B9} with the exact libjxl
    // ComputeCoeffOrder state update (pre-update sum compared, 50%
    // threshold 2147483647).
    var sampler = OrderSampler{};
    const threshold: u64 = 2147483647;
    // Draw 0: (s0 + s1) >> 32 = 0x54289128 <= threshold.
    try std.testing.expect(sampler.useSample(threshold));
    try std.testing.expectEqual(
        @as(u64, 0xBF58476D1CE4E5B9),
        sampler.s[0],
    );
    try std.testing.expectEqual(
        @as(u64, 0xF3FB4633E6FE2AD3),
        sampler.s[1],
    );
    // Draw 1: 0xB3538DA1 > threshold.
    try std.testing.expect(!sampler.useSample(threshold));
    // Draw 2: 0xF1ADF19C > threshold.
    try std.testing.expect(!sampler.useSample(threshold));
    // Draw 3: 0x0E09E900 <= threshold.
    try std.testing.expect(sampler.useSample(threshold));
}

test "computeCoeffOrder: natural for small images, learned for large" {
    const alloc = std.testing.allocator;
    const FrameDim = stream_mod.FrameDim;

    // Small (< 5x5 blocks): default orders, used_orders = 0.
    {
        const fd = FrameDim.initVarDct(
            32,
            32,
            0,
            0,
        );
        const res = try computeCoeffOrder(
            alloc,
            &fd,
            .{ 0, 0, 0 },
            .{ 0, 0, 0 },
            .{
                .data = .{ &.{}, &.{}, &.{} },
                .widths = .{ 4, 4, 4 },
            },
        );
        const natural = naturalOrder8x8();
        try std.testing.expectEqual(
            @as(u32, 0),
            res.used_orders,
        );
        for (0..3) |c|
            try std.testing.expectEqualSlices(
                u32,
                &natural,
                &res.orders[c],
            );
    }

    // Large image, all-zero coefficients: every position has the same zero
    // count, so the order stays natural (is_nondefault = false).
    {
        const fd = FrameDim.initVarDct(
            256,
            256,
            0,
            0,
        );
        const w_blocks = fd.xsize_blocks;
        const planes = try alloc.alloc(
            i32,
            w_blocks * fd.ysize_blocks * 64,
        );
        defer alloc.free(planes);
        @memset(planes, 0);
        const res = try computeCoeffOrder(
            alloc,
            &fd,
            .{ 0, 0, 0 },
            .{ 0, 0, 0 },
            .{
                .data = .{ planes, planes, planes },
                .widths = .{ w_blocks, w_blocks, w_blocks },
            },
        );
        try std.testing.expectEqual(
            @as(u32, 0),
            res.used_orders,
        );
    }

    // Large image with a strongly asymmetric zero pattern: high zig-zag
    // positions always zero, low positions often nonzero -> the learned
    // order differs from natural and used_orders = 1. The DC/LLF stays
    // first in the scan.
    {
        const fd = FrameDim.initVarDct(
            256,
            256,
            0,
            0,
        );
        const w_blocks = fd.xsize_blocks;
        const n_blocks = w_blocks * fd.ysize_blocks;
        const planes = try alloc.alloc(i32, n_blocks * 64);
        defer alloc.free(planes);
        @memset(planes, 0);
        const natural = naturalOrder8x8();
        var prng = std.Random.DefaultPrng.init(7);
        const rnd = prng.random();
        var b: usize = 0;
        while (b < n_blocks) : (b += 1) {
            const block = planes[b * 64 ..][0..64];
            for (0..12) |k| {
                if (rnd.boolean()) block[natural[k]] = @intCast(
                    1 + rnd.uintLessThan(u32, 8),
                );
            }
        }
        const res = try computeCoeffOrder(
            alloc,
            &fd,
            .{ 0, 0, 0 },
            .{ 0, 0, 0 },
            .{
                .data = .{ planes, planes, planes },
                .widths = .{ w_blocks, w_blocks, w_blocks },
            },
        );
        try std.testing.expectEqual(
            @as(u32, 1),
            res.used_orders,
        );
        for (0..3) |c| {
            try std.testing.expectEqual(
                @as(u32, 0),
                res.orders[c][0],
            ); // LLF first
            var differs = false;
            for (0..64) |k| {
                if (res.orders[c][k] != natural[k]) differs = true;
            }
            try std.testing.expect(differs);
        }
    }
}

test "kOrderEnc roundtrip of zero" {
    const alloc = std.testing.allocator;
    var w = BitWriter.init(alloc);
    defer w.deinit();
    try U32Coder.write(kOrderEnc, 0, &w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    try std.testing.expectEqual(
        @as(u32, 0),
        U32Coder.read(kOrderEnc, &br),
    );
}

test "lehmer code roundtrip" {
    const alloc = std.testing.allocator;
    // (permutation, its lehmer code) pairs; lehmer[i] = #{j > i : perm[j] < perm[i]}.
    const cases = [_]struct { perm: [8]u32, code: [8]u32 }{
        .{ .perm = .{ 0, 1, 2, 3, 4, 5, 6, 7 }, .code = .{ 0, 0, 0, 0, 0, 0, 0, 0 } },
        .{ .perm = .{ 7, 6, 5, 4, 3, 2, 1, 0 }, .code = .{ 7, 6, 5, 4, 3, 2, 1, 0 } },
        .{ .perm = .{ 1, 0, 2, 3, 4, 5, 6, 7 }, .code = .{ 1, 0, 0, 0, 0, 0, 0, 0 } },
        .{ .perm = .{ 2, 0, 1, 3, 4, 5, 6, 7 }, .code = .{ 2, 0, 0, 0, 0, 0, 0, 0 } },
        .{ .perm = .{ 5, 2, 0, 1, 3, 4, 6, 7 }, .code = .{ 5, 2, 0, 0, 0, 0, 0, 0 } },
        .{ .perm = .{ 0, 7, 1, 2, 3, 4, 5, 6 }, .code = .{ 0, 6, 0, 0, 0, 0, 0, 0 } },
        .{ .perm = .{ 3, 4, 5, 0, 1, 2, 6, 7 }, .code = .{ 3, 3, 3, 0, 0, 0, 0, 0 } },
    };
    const temp = try alloc.alloc(u32, 16);
    defer alloc.free(temp);
    const perm = try alloc.alloc(u32, 8);
    defer alloc.free(perm);
    for (cases) |case| {
        decodeLehmerCode(&case.code, temp, 8, perm);
        try std.testing.expectEqualSlices(
            u32,
            &case.perm,
            perm,
        );
    }
}

test "coeff order context values" {
    // HybridUintConfig(0,0,0) token: 0 -> 0; 1 -> 1; 2,3 -> 2; 4..7 -> 3...
    try std.testing.expectEqual(@as(usize, 0), coeffOrderContext(0));
    try std.testing.expectEqual(@as(usize, 1), coeffOrderContext(1));
    try std.testing.expectEqual(@as(usize, 2), coeffOrderContext(2));
    try std.testing.expectEqual(@as(usize, 3), coeffOrderContext(4));
    try std.testing.expectEqual(@as(usize, 7), coeffOrderContext(100));
    try std.testing.expectEqual(@as(usize, 7), coeffOrderContext(64));
}
