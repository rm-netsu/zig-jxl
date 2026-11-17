//! LZ77 token-stream compression (encoder side). Port of libjxl
//! `enc_lz77.h/cc`: ApplyLZ77 with RLE, greedy LZ77 and optimal parsing
//! strategies, including the hash-chain matcher.

const std = @import("std");
const ans_common = @import("ans_common.zig");
const bits = @import("../base/bits.zig");
const dec_ans = @import("dec_ans.zig");
const enc_ans = @import("enc_ans.zig");
const fast_math = @import("../base/fast_math.zig");

const Token = enc_ans.Token;
const Histogram = enc_ans.Histogram;
const HistogramParams = enc_ans.HistogramParams;
const HybridUintConfig = ans_common.HybridUintConfig;
const LZ77Params = @import("../base/fields.zig").LZ77Params;
const fastLog2f = fast_math.fastLog2f;

const k_window_size = dec_ans.k_window_size;
const k_num_special_distances = dec_ans.k_num_special_distances;

/// Port of `SymbolCostEstimator`.
const SymbolCostEstimator = struct {
    max_alphabet_size: usize,
    bits_: []f32,
    add_symbol_cost_: []f32,
    allocator: std.mem.Allocator,

    fn init(
        allocator: std.mem.Allocator,
        num_contexts: usize,
        force_huffman: bool,
        tokens: []const std.ArrayList(Token),
        lz77: *const LZ77Params,
    ) !SymbolCostEstimator {
        var builder: std.ArrayList(Histogram) = .empty;
        defer {
            for (builder.items) |*h| h.deinit();
            builder.deinit(allocator);
        }
        try builder.resize(allocator, num_contexts);
        for (builder.items) |*h| h.* = Histogram.init(allocator);
        // Build histograms for estimating lz77 savings.
        const uint_config = HybridUintConfig.default;
        for (tokens) |stream| {
            for (stream.items) |token| {
                const enc = if (token.is_lz77_length) &lz77.length_uint_config else &uint_config;
                const e = enc.encode(token.value);
                var tok = e.token;
                if (token.is_lz77_length) tok += lz77.min_symbol;
                try builder.items[token.context].add(allocator, tok);
            }
        }
        var max_alphabet_size: usize = 0;
        for (0..num_contexts) |i| {
            max_alphabet_size = @max(max_alphabet_size, builder.items[i].counts.items.len);
        }
        const bits_ = try allocator.alloc(f32, num_contexts * max_alphabet_size);
        errdefer allocator.free(bits_);
        const add_symbol_cost = try allocator.alloc(f32, num_contexts);
        errdefer allocator.free(add_symbol_cost);

        for (0..num_contexts) |i| {
            const inv_total: f32 = 1.0 / (@as(f32, @floatFromInt(builder.items[i].total_count)) + 1e-8);
            var total_cost: f32 = 0;
            for (0..builder.items[i].counts.items.len) |j| {
                const cnt: usize = @intCast(builder.items[i].counts.items[j]);
                var cost: f32 = 0;
                if (cnt != 0 and cnt != builder.items[i].total_count) {
                    cost = -fastLog2f(@as(f32, @floatFromInt(cnt)) * inv_total);
                    if (force_huffman) cost = @ceil(cost);
                } else if (cnt == 0) {
                    cost = @floatFromInt(ans_common.ans_log_tab_size); // Highest possible cost.
                }
                bits_[i * max_alphabet_size + j] = cost;
                total_cost += cost * @as(f32, @floatFromInt(cnt));
            }
            // Penalty for adding an lz77 symbol to this context (only used
            // for the static cost model). Higher penalty for contexts that
            // have a very low per-symbol entropy.
            add_symbol_cost[i] = @max(0.0, 6.0 - total_cost * inv_total);
        }

        return .{
            .max_alphabet_size = max_alphabet_size,
            .bits_ = bits_,
            .add_symbol_cost_ = add_symbol_cost,
            .allocator = allocator,
        };
    }

    fn deinit(self: *SymbolCostEstimator) void {
        self.allocator.free(self.bits_);
        self.allocator.free(self.add_symbol_cost_);
    }

    fn bitsAt(self: *const SymbolCostEstimator, ctx: usize, sym: usize) f32 {
        return self.bits_[ctx * self.max_alphabet_size + sym];
    }

    fn lenCost(self: *const SymbolCostEstimator, ctx: usize, len: usize, lz77: *const LZ77Params) f32 {
        const e = lz77.length_uint_config.encode(@intCast(len));
        var tok = e.token;
        tok += lz77.min_symbol;
        return @as(f32, @floatFromInt(e.nbits)) + self.bitsAt(ctx, tok);
    }

    fn distCost(self: *const SymbolCostEstimator, len: usize, lz77: *const LZ77Params) f32 {
        const e = (HybridUintConfig{}).encode(@intCast(len));
        return @as(f32, @floatFromInt(e.nbits)) + self.bitsAt(lz77.nonserialized_distance_context, e.token);
    }

    fn addSymbolCost(self: *const SymbolCostEstimator, idx: usize) f32 {
        return self.add_symbol_cost_[idx];
    }
};

/// Port of `ApplyLZ77_RLE`.
fn applyLZ77RLE(
    allocator: std.mem.Allocator,
    params: *const HistogramParams,
    num_contexts: usize,
    tokens: []const std.ArrayList(Token),
    lz77: *const LZ77Params,
) ![]std.ArrayList(Token) {
    const tokens_lz77 = try allocator.alloc(std.ArrayList(Token), tokens.len);
    for (tokens_lz77) |*t| t.* = .empty;
    errdefer {
        for (tokens_lz77) |*t| t.deinit(allocator);
        allocator.free(tokens_lz77);
    }
    var sce = try SymbolCostEstimator.init(allocator, num_contexts, params.force_huffman, tokens, lz77);
    defer sce.deinit();
    var bit_decrease: f32 = 0;
    var total_symbols: usize = 0;
    var sym_cost: std.ArrayList(f32) = .empty;
    defer sym_cost.deinit(allocator);
    const uint_config = HybridUintConfig.default;
    for (tokens, 0..) |in, stream| {
        const distance_multiplier: usize = if (params.image_widths.len > stream) params.image_widths[stream] else 0;
        const out = &tokens_lz77[stream];
        total_symbols += in.items.len;
        // Cumulative sum of bit costs.
        try sym_cost.resize(allocator, in.items.len + 1);
        sym_cost.items[0] = 0;
        for (in.items, 0..) |tok_in, i| {
            const e = uint_config.encode(tok_in.value);
            sym_cost.items[i + 1] = sce.bitsAt(tok_in.context, e.token) + @as(f32, @floatFromInt(e.nbits)) + sym_cost.items[i];
        }
        try out.ensureTotalCapacity(allocator, in.items.len);
        var i: usize = 0;
        while (i < in.items.len) : (i += 1) {
            var num_to_copy: usize = 0;
            var distance_symbol: usize = 0; // 1 for RLE.
            if (distance_multiplier != 0) {
                distance_symbol = 1; // Special distance 1 if enabled.
            }
            if (i > 0) {
                while (i + num_to_copy < in.items.len) : (num_to_copy += 1) {
                    if (in.items[i + num_to_copy].value != in.items[i - 1].value) {
                        break;
                    }
                }
            }
            if (num_to_copy == 0) {
                try out.append(allocator, in.items[i]);
                continue;
            }
            const cost = sym_cost.items[i + num_to_copy] - sym_cost.items[i];
            // This subtraction might overflow, but that's OK.
            const lz77_len = num_to_copy -% lz77.min_length;
            const lz77_cost: f32 = if (num_to_copy >= lz77.min_length)
                @as(f32, @floatFromInt(bits.ceilLog2Nonzero(lz77_len + 1))) + 1
            else
                0;
            if (num_to_copy < lz77.min_length or cost <= lz77_cost) {
                for (0..num_to_copy) |j| {
                    try out.append(allocator, in.items[i + j]);
                }
                i += num_to_copy - 1;
                continue;
            }
            // Output the LZ77 length.
            try out.append(allocator, .{ .is_lz77_length = true, .context = in.items[i].context, .value = @intCast(lz77_len) });
            i += num_to_copy - 1;
            bit_decrease += cost - lz77_cost;
            // Output the LZ77 copy distance.
            try out.append(allocator, .{ .is_lz77_length = false, .context = lz77.nonserialized_distance_context, .value = @intCast(distance_symbol) });
        }
    }

    if (bit_decrease > @as(f32, @floatFromInt(total_symbols)) * 0.2 + 16) {
        return tokens_lz77;
    }
    for (tokens_lz77) |*t| t.deinit(allocator);
    allocator.free(tokens_lz77);
    return &.{};
}

/// Hash chain for LZ77 matching (port of libjxl `HashChain`).
const HashChain = struct {
    size: usize,
    data: []u32,

    hash_num_values: u32 = 32768,
    hash_shift: u5 = 5,

    head: []i32,
    chain: []u32,
    val: []i32,

    // Speed up repetitions of zero.
    headz: []i32,
    chainz: []u32,
    zeros: []u32,
    numzeros: u32 = 0,

    window_size: usize,
    window_mask: usize,
    min_length: usize,
    max_length: usize,

    // Map of special distance codes.
    special_dist_table: std.AutoHashMapUnmanaged(i32, i32) = .empty,
    num_special_distances: usize = 0,

    maxchainlength: u32 = 256, // window_size to allow all

    allocator: std.mem.Allocator,

    fn init(
        allocator: std.mem.Allocator,
        data: []const Token,
        window_size: usize,
        min_length: usize,
        max_length: usize,
        distance_multiplier: usize,
    ) !HashChain {
        const size = data.len;
        const data_vals = try allocator.alloc(u32, size);
        errdefer allocator.free(data_vals);
        for (data, 0..) |t, i| {
            data_vals[i] = t.value;
        }

        const head = try allocator.alloc(i32, 32768);
        errdefer allocator.free(head);
        @memset(head, -1);
        const val = try allocator.alloc(i32, window_size);
        errdefer allocator.free(val);
        @memset(val, -1);
        const chain = try allocator.alloc(u32, window_size);
        errdefer allocator.free(chain);
        for (0..window_size) |i| {
            chain[i] = @intCast(i); // same value as index indicates uninitialized
        }

        const zeros = try allocator.alloc(u32, window_size);
        errdefer allocator.free(zeros);
        @memset(zeros, 0);
        const headz = try allocator.alloc(i32, window_size + 1);
        errdefer allocator.free(headz);
        @memset(headz, -1);
        const chainz = try allocator.alloc(u32, window_size);
        errdefer allocator.free(chainz);
        for (0..window_size) |i| {
            chainz[i] = @intCast(i);
        }

        var self = HashChain{
            .size = size,
            .data = data_vals,
            .head = head,
            .chain = chain,
            .val = val,
            .headz = headz,
            .chainz = chainz,
            .zeros = zeros,
            .window_size = window_size,
            .window_mask = window_size - 1,
            .min_length = min_length,
            .max_length = max_length,
            .allocator = allocator,
        };
        // Translate distance to special distance code.
        if (distance_multiplier != 0) {
            // Count down, so if due to small distance multiplier multiple
            // distances map to the same code, the smallest code will be used
            // in the end.
            var i: isize = @as(isize, @intCast(k_num_special_distances)) - 1;
            while (i >= 0) : (i -= 1) {
                try self.special_dist_table.put(allocator, @intCast(dec_ans.specialDistance(@intCast(i), @intCast(distance_multiplier))), @intCast(i));
            }
            self.num_special_distances = k_num_special_distances;
        }
        return self;
    }

    fn deinit(self: *HashChain) void {
        self.special_dist_table.deinit(self.allocator);
        self.allocator.free(self.data);
        self.allocator.free(self.head);
        self.allocator.free(self.chain);
        self.allocator.free(self.val);
        self.allocator.free(self.headz);
        self.allocator.free(self.chainz);
        self.allocator.free(self.zeros);
    }

    fn getHash(self: *const HashChain, pos: usize) u32 {
        var result: u32 = 0;
        if (pos + 2 < self.size) {
            result ^= self.data[pos + 0];
            result ^= self.data[pos + 1] << self.hash_shift;
            result ^= self.data[pos + 2] << (self.hash_shift * 2);
        } else {
            // No need to compute hash of last 2 bytes, the length 2 is too
            // short.
            return 0;
        }
        return result & (self.hash_num_values - 1);
    }

    fn countZeros(self: *const HashChain, pos: usize, prevzeros: u32) u32 {
        var end = pos + self.window_size;
        if (end > self.size) end = self.size;
        if (prevzeros > 0) {
            if (prevzeros >= self.window_mask and self.data[end - 1] == 0 and
                end == pos + self.window_size)
            {
                return prevzeros;
            } else {
                return prevzeros - 1;
            }
        }
        var num: u32 = 0;
        while (pos + num < end and self.data[pos + num] == 0) num += 1;
        return num;
    }

    fn update(self: *HashChain, pos: usize) void {
        const hashval = self.getHash(pos);
        const wpos = pos & self.window_mask;

        self.val[wpos] = @intCast(hashval);
        if (self.head[hashval] != -1) self.chain[wpos] = @intCast(self.head[hashval]);
        self.head[hashval] = @intCast(wpos);

        if (pos > 0 and self.data[pos] != self.data[pos - 1]) self.numzeros = 0;
        self.numzeros = self.countZeros(pos, self.numzeros);

        self.zeros[wpos] = self.numzeros;
        if (self.headz[self.numzeros] != -1) self.chainz[wpos] = @intCast(self.headz[self.numzeros]);
        self.headz[self.numzeros] = @intCast(wpos);
    }

    fn updateRange(self: *HashChain, pos: usize, len: usize) void {
        for (0..len) |i| {
            self.update(pos + i);
        }
    }

    /// Calls `found_match(len, dist_symbol)` for candidate matches.
    /// The callback must be a function `(usize, usize) void`.
    fn findMatches(
        self: *const HashChain,
        pos: usize,
        max_dist: i32,
        context: anytype,
        comptime found_match: fn (@TypeOf(context), usize, usize) void,
    ) void {
        _ = max_dist; // unused in libjxl either (kept for signature parity)
        const wpos = pos & self.window_mask;
        const hashval = self.getHash(pos);
        var hashpos: u32 = self.chain[wpos];

        var prev_dist: i32 = 0;
        const end: i32 = @intCast(@min(pos + self.max_length, self.size));
        var chainlength: u32 = 0;
        var best_len: usize = 0;
        while (true) {
            const wpos_i: i32 = @intCast(wpos);
            const hashpos_i: i32 = @intCast(hashpos);
            const dist: i32 = if (hashpos_i <= wpos_i)
                (wpos_i - hashpos_i)
            else
                (wpos_i - hashpos_i + @as(i32, @intCast(self.window_mask)) + 1);
            if (dist < prev_dist) break;
            prev_dist = dist;
            var len: usize = 0;
            if (dist > 0) {
                var i: i32 = @intCast(pos);
                var j: i32 = @as(i32, @intCast(pos)) - dist;
                if (self.numzeros > 3) {
                    const zpos: usize = @intCast(hashpos);
                    var r: i32 = @min(@as(i32, @intCast(self.numzeros)) - 1, @as(i32, @intCast(self.zeros[zpos])));
                    if (i + r >= end) r = end - i - 1;
                    i += r;
                    j += r;
                }
                while (i < end and self.data[@intCast(i)] == self.data[@intCast(j)]) {
                    i += 1;
                    j += 1;
                }
                len = @intCast(i - @as(i32, @intCast(pos)));
                // This can trigger even if the new length is slightly smaller
                // than the best length, because it is possible for a
                // slightly cheaper distance symbol to occur.
                if (len >= self.min_length and len + 2 >= best_len) {
                    const dist_symbol: usize = blk: {
                        if (self.special_dist_table.get(dist)) |ds| {
                            break :blk @intCast(ds);
                        }
                        break :blk self.num_special_distances + @as(usize, @intCast(dist)) - 1;
                    };
                    found_match(context, len, dist_symbol);
                    if (len > best_len) best_len = len;
                }
            }

            chainlength += 1;
            if (chainlength >= self.maxchainlength) break;

            const hp: usize = @intCast(hashpos);
            if (self.numzeros >= 3 and len > self.numzeros) {
                if (hashpos == self.chainz[hp]) break;
                hashpos = self.chainz[hp];
                if (self.zeros[hashpos] != self.numzeros) break;
            } else {
                if (hashpos == self.chain[hp]) break;
                hashpos = self.chain[hp];
                if (self.val[hashpos] != @as(i32, @intCast(hashval))) {
                    // outdated hash value
                    break;
                }
            }
        }
    }

    fn findMatch(self: *const HashChain, pos: usize, max_dist: i32, result_dist_symbol: *usize, result_len: *usize) void {
        result_dist_symbol.* = 0;
        result_len.* = 1;
        const Ctx = struct {
            len: *usize,
            dist_symbol: *usize,
            fn found(ctx2: @This(), l: usize, ds: usize) void {
                if (l > ctx2.len.* or (l == ctx2.len.* and ctx2.dist_symbol.* > ds)) {
                    ctx2.len.* = l;
                    ctx2.dist_symbol.* = ds;
                }
            }
        };
        const ctx = Ctx{ .len = result_len, .dist_symbol = result_dist_symbol };
        self.findMatches(pos, max_dist, ctx, Ctx.found);
    }
};

fn lenCostStatic(len: usize) f32 {
    const e = (HybridUintConfig.init(1, 0, 0)).encode(@intCast(len));
    var tok: usize = e.token;
    const k_cost_table = [_]f32{
        2.797667318563126,  3.213177690381199,  2.5706009246743737,
        2.408392498667534,  2.829649191872326,  3.3923087753324577,
        4.029267451554331,  4.415576699706408,  4.509357574741465,
        9.21481543803004,   10.020590190114898, 11.858671627804766,
        12.45853300490526,  11.713105831990857, 12.561996324849314,
        13.775477692278367, 13.174027068768641,
    };
    if (tok >= k_cost_table.len) tok = k_cost_table.len - 1;
    return k_cost_table[tok] + @as(f32, @floatFromInt(e.nbits));
}

fn distCostStatic(dist: usize) f32 {
    const e = (HybridUintConfig.init(7, 0, 0)).encode(@intCast(dist));
    var tok: usize = e.token;
    const k_cost_table = [_]f32{
        6.368282626312716,  5.680793277090298,  8.347404197105247,
        7.641619201599141,  6.914328374119438,  7.959808291537444,
        8.70023120759855,   8.71378518934703,   9.379132523982769,
        9.110472749092708,  9.159029569270908,  9.430936766731973,
        7.278284055315169,  7.8278514904267755, 10.026641158289236,
        9.976049229827066,  9.64351607048908,   9.563403863480442,
        10.171474111762747, 10.45950155077234,  9.994813912104219,
        10.322524683741156, 8.465808729388186,  8.756254166066853,
        10.160930174662234, 10.247329273413435, 10.04090403724809,
        10.129398517544082, 9.342311691539546,  9.07608009102374,
        10.104799540677513, 10.378079384990906, 10.165828974075072,
        10.337595322341553, 7.940557464567944,  10.575665823319431,
        11.023344321751955, 10.736144698831827, 11.118277044595054,
        7.468468230648442,  10.738305230932939, 10.906980780216568,
        10.163468216353817, 10.17805759656433,  11.167283670483565,
        11.147050200274544, 10.517921919244333, 10.651764778156886,
        10.17074446448919,  11.217636876224745, 11.261630721139484,
        11.403140815247259, 10.892472096873417, 11.1859607804481,
        8.017346947551262,  7.895143720278828,  11.036577113822025,
        11.170562110315794, 10.326988722591086, 10.40872184751056,
        11.213498225466386, 11.30580635516863,  10.672272515665442,
        10.768069466228063, 11.145257364153565, 11.64668307145549,
        10.593156194627339, 11.207499484844943, 10.767517766396908,
        10.826629811407042, 10.737764794499988, 10.6200448518045,
        10.191315385198092, 8.468384171390085,  11.731295299170432,
        11.824619886654398, 10.41518844301179,  10.16310536548649,
        10.539423685097576, 10.495136599328031, 10.469112847728267,
        11.72057686174922,  10.910326337834674, 11.378921834673758,
        11.847759036098536, 11.92071647623854,  10.810628276345282,
        11.008601085273893, 11.910326337834674, 11.949212023423133,
        11.298614839104337, 11.611603659010392, 10.472930394619985,
        11.835564720850282, 11.523267392285337, 12.01055816679611,
        8.413029688994023,  11.895784139536406, 11.984679534970505,
        11.220654278717394, 11.716311684833672, 10.61036646226114,
        10.89849965960364,  10.203762898863669, 10.997560826267238,
        11.484217379438984, 11.792836176993665, 12.24310468755171,
        11.464858097919262, 12.212747017409377, 11.425595666074955,
        11.572048533398757, 12.742093965163013, 11.381874288645637,
        12.191870445817015, 11.683156920035426, 11.152442115262197,
        11.90303691580457,  11.653292787169159, 11.938615382266098,
        16.970641701570223, 16.853602280380002, 17.26240782594733,
        16.644655390108507, 17.14310889757499,  16.910935455445955,
        17.505678976959697, 17.213498225466388, 2.4162310293553024,
        3.494587244462329,  3.5258600986408344, 3.4959806589517095,
        3.098390886949687,  3.343454654302911,  3.588847442290287,
        4.14614790111827,   5.152948641990529,  7.433696808092598,
        9.716311684833672,
    };
    if (tok >= k_cost_table.len) tok = k_cost_table.len - 1;
    return k_cost_table[tok] + @as(f32, @floatFromInt(e.nbits));
}

/// Port of `ApplyLZ77_LZ77`.
fn applyLZ77LZ77(
    allocator: std.mem.Allocator,
    params: *const HistogramParams,
    num_contexts: usize,
    tokens: []const std.ArrayList(Token),
    lz77: *const LZ77Params,
) ![]std.ArrayList(Token) {
    const tokens_lz77 = try allocator.alloc(std.ArrayList(Token), tokens.len);
    for (tokens_lz77) |*t| t.* = .empty;
    errdefer {
        for (tokens_lz77) |*t| t.deinit(allocator);
        allocator.free(tokens_lz77);
    }
    var sce = try SymbolCostEstimator.init(allocator, num_contexts, params.force_huffman, tokens, lz77);
    defer sce.deinit();
    var bit_decrease: f32 = 0;
    var total_symbols: usize = 0;
    var sym_cost: std.ArrayList(f32) = .empty;
    defer sym_cost.deinit(allocator);
    const uint_config = HybridUintConfig.default;
    for (tokens, 0..) |in, stream| {
        const distance_multiplier: usize = if (params.image_widths.len > stream) params.image_widths[stream] else 0;
        const out = &tokens_lz77[stream];
        total_symbols += in.items.len;
        // Cumulative sum of bit costs.
        try sym_cost.resize(allocator, in.items.len + 1);
        sym_cost.items[0] = 0;
        for (in.items, 0..) |tok_in, i| {
            const e = uint_config.encode(tok_in.value);
            sym_cost.items[i + 1] = sce.bitsAt(tok_in.context, e.token) + @as(f32, @floatFromInt(e.nbits)) + sym_cost.items[i];
        }

        try out.ensureTotalCapacity(allocator, in.items.len);
        const max_distance: usize = in.items.len;
        const min_length: usize = lz77.min_length;
        std.debug.assert(min_length >= 3);
        const max_length: usize = in.items.len;

        // Use next power of two as window size.
        var window_size: usize = 1;
        while (window_size < max_distance and window_size < k_window_size) {
            window_size <<= 1;
        }

        var chain = try HashChain.init(allocator, in.items, window_size, min_length, max_length, distance_multiplier);
        defer chain.deinit();
        var len: usize = undefined;
        var dist_symbol: usize = undefined;

        const max_lazy_match_len = 256; // 0 to disable lazy matching

        // Whether the next symbol was already updated (to test lazy matching)
        var already_updated = false;
        var i: usize = 0;
        while (i < in.items.len) : (i += 1) {
            try out.append(allocator, in.items[i]);
            if (!already_updated) chain.update(i);
            already_updated = false;
            chain.findMatch(i, @intCast(max_distance), &dist_symbol, &len);
            if (len >= min_length) {
                if (len < max_lazy_match_len and i + 1 < in.items.len) {
                    // Try length at next symbol lazy matching
                    chain.update(i + 1);
                    already_updated = true;
                    var len2: usize = undefined;
                    var dist_symbol2: usize = undefined;
                    chain.findMatch(i + 1, @intCast(max_distance), &dist_symbol2, &len2);
                    if (len2 > len) {
                        // Use the lazy match. Add literal, and use the next
                        // length starting from the next byte.
                        i += 1;
                        already_updated = false;
                        len = len2;
                        dist_symbol = dist_symbol2;
                        try out.append(allocator, in.items[i]);
                    }
                }

                const cost = sym_cost.items[i + len] - sym_cost.items[i];
                const lz77_len = len - lz77.min_length;
                const lz77_cost = lenCostStatic(lz77_len) + distCostStatic(dist_symbol) +
                    sce.addSymbolCost(out.items[out.items.len - 1].context);

                if (lz77_cost <= cost) {
                    out.items[out.items.len - 1].value = @intCast(len - lz77.min_length);
                    out.items[out.items.len - 1].is_lz77_length = true;
                    try out.append(allocator, .{ .is_lz77_length = false, .context = lz77.nonserialized_distance_context, .value = @intCast(dist_symbol) });
                    bit_decrease += cost - lz77_cost;
                } else {
                    // LZ77 match ignored, and symbol already pushed. Push all
                    // other symbols and skip.
                    for (1..len) |j| {
                        try out.append(allocator, in.items[i + j]);
                    }
                }

                if (already_updated) {
                    chain.updateRange(i + 2, len - 2);
                    already_updated = false;
                } else {
                    chain.updateRange(i + 1, len - 1);
                }
                i += len - 1;
            } else {
                // Literal, already pushed
            }
        }
    }

    if (bit_decrease > @as(f32, @floatFromInt(total_symbols)) * 0.2 + 16) {
        return tokens_lz77;
    }
    for (tokens_lz77) |*t| t.deinit(allocator);
    allocator.free(tokens_lz77);
    return &.{};
}

/// Port of `ApplyLZ77_Optimal`.
fn applyLZ77Optimal(
    allocator: std.mem.Allocator,
    params: *const HistogramParams,
    num_contexts: usize,
    tokens: []const std.ArrayList(Token),
    lz77: *const LZ77Params,
) ![]std.ArrayList(Token) {
    const tokens_for_cost_estimate = try applyLZ77LZ77(allocator, params, num_contexts, tokens, lz77);
    // If greedy-LZ77 does not give better compression than no-lz77, no
    // reason to run the optimal matching.
    if (tokens_for_cost_estimate.len == 0) {
        return &.{};
    }
    const owned_estimate = tokens_for_cost_estimate;
    defer {
        for (owned_estimate) |*t| t.deinit(allocator);
        allocator.free(owned_estimate);
    }
    var sce = try SymbolCostEstimator.init(allocator, num_contexts + 1, params.force_huffman, tokens_for_cost_estimate, lz77);
    defer sce.deinit();
    const tokens_lz77 = try allocator.alloc(std.ArrayList(Token), tokens.len);
    for (tokens_lz77) |*t| t.* = .empty;
    errdefer {
        for (tokens_lz77) |*t| t.deinit(allocator);
        allocator.free(tokens_lz77);
    }
    var sym_cost: std.ArrayList(f32) = .empty;
    defer sym_cost.deinit(allocator);
    var dist_symbols: std.ArrayList(u32) = .empty;
    defer dist_symbols.deinit(allocator);
    const uint_config = HybridUintConfig.default;
    for (tokens, 0..) |in, stream| {
        const distance_multiplier: usize = if (params.image_widths.len > stream) params.image_widths[stream] else 0;
        const out = &tokens_lz77[stream];
        // Cumulative sum of bit costs.
        try sym_cost.resize(allocator, in.items.len + 1);
        sym_cost.items[0] = 0;
        for (in.items, 0..) |tok_in, i| {
            const e = uint_config.encode(tok_in.value);
            sym_cost.items[i + 1] = sce.bitsAt(tok_in.context, e.token) + @as(f32, @floatFromInt(e.nbits)) + sym_cost.items[i];
        }

        const max_distance: usize = in.items.len;
        const min_length: usize = lz77.min_length;
        std.debug.assert(min_length >= 3);
        const max_length: usize = in.items.len;

        // Use next power of two as window size.
        var window_size: usize = 1;
        while (window_size < max_distance and window_size < k_window_size) {
            window_size <<= 1;
        }

        var chain = try HashChain.init(allocator, in.items, window_size, min_length, max_length, distance_multiplier);
        defer chain.deinit();

        const MatchInfo = struct {
            len: u32 = 0,
            dist_symbol: u32 = 0,
            ctx: u32 = 0,
            total_cost: f32 = std.math.floatMax(f32),
        };
        // Total cost to encode the first N symbols.
        const prefix_costs = try allocator.alloc(MatchInfo, in.items.len + 1);
        defer allocator.free(prefix_costs);
        @memset(prefix_costs, .{});
        prefix_costs[0].total_cost = 0;

        var rle_length: usize = 0;
        var skip_lz77: usize = 0;
        for (0..in.items.len) |i| {
            chain.update(i);
            const lit_cost = prefix_costs[i].total_cost + sym_cost.items[i + 1] - sym_cost.items[i];
            if (prefix_costs[i + 1].total_cost > lit_cost) {
                prefix_costs[i + 1] = .{
                    .len = 1,
                    .dist_symbol = 0,
                    .ctx = in.items[i].context,
                    .total_cost = lit_cost,
                };
            }
            if (skip_lz77 > 0) {
                skip_lz77 -= 1;
                continue;
            }
            dist_symbols.clearRetainingCapacity();
            const Ctx = struct {
                list: *std.ArrayList(u32),
                allocator: std.mem.Allocator,
                fn found(self: @This(), l: usize, ds: usize) void {
                    // C++: resize(len + 1, dist_symbol) fills new slots with
                    // the current dist_symbol.
                    while (self.list.items.len <= l) {
                        self.list.append(self.allocator, @intCast(ds)) catch @panic("OOM");
                    }
                    if (ds < self.list.items[l]) {
                        self.list.items[l] = @intCast(ds);
                    }
                }
            };
            chain.findMatches(i, @intCast(max_distance), Ctx{ .list = &dist_symbols, .allocator = allocator }, Ctx.found);
            if (dist_symbols.items.len <= min_length) continue;
            {
                var best_cost: usize = dist_symbols.items[dist_symbols.items.len - 1];
                var j: usize = dist_symbols.items.len - 1;
                while (j >= min_length) : (j -= 1) {
                    if (dist_symbols.items[j] < best_cost) {
                        best_cost = dist_symbols.items[j];
                    }
                    dist_symbols.items[j] = @intCast(best_cost);
                    if (j == 0) break;
                }
            }
            for (min_length..dist_symbols.items.len) |j| {
                // Cost model that uses results from lazy LZ77.
                const lz77_cost = sce.lenCost(in.items[i].context, j - lz77.min_length, lz77) +
                    sce.distCost(dist_symbols.items[j], lz77);
                const cost = prefix_costs[i].total_cost + lz77_cost;
                if (prefix_costs[i + j].total_cost > cost) {
                    prefix_costs[i + j].len = @intCast(j);
                    prefix_costs[i + j].dist_symbol = dist_symbols.items[j] + 1;
                    prefix_costs[i + j].ctx = in.items[i].context;
                    prefix_costs[i + j].total_cost = cost;
                }
            }
            // We are in an RLE sequence: skip all the symbols except the
            // first 8 and the last 8. This avoids quadratic costs for
            // sequences with long runs of the same symbol.
            if ((dist_symbols.items[dist_symbols.items.len - 1] == 0 and distance_multiplier == 0) or
                (dist_symbols.items[dist_symbols.items.len - 1] == 1 and distance_multiplier != 0))
            {
                rle_length += 1;
            } else {
                rle_length = 0;
            }
            if (rle_length >= 8 and dist_symbols.items.len > 9) {
                skip_lz77 = dist_symbols.items.len - 10;
                rle_length = 0;
            }
        }
        var pos: usize = in.items.len;
        while (pos > 0) {
            const is_lz77_length = prefix_costs[pos].dist_symbol != 0;
            if (is_lz77_length) {
                const dist_symbol = prefix_costs[pos].dist_symbol - 1;
                try out.append(allocator, .{ .is_lz77_length = false, .context = lz77.nonserialized_distance_context, .value = dist_symbol });
            }
            const val: u32 = if (is_lz77_length)
                (prefix_costs[pos].len - @as(u32, @intCast(lz77.min_length)))
            else
                in.items[pos - 1].value;
            try out.append(allocator, .{ .is_lz77_length = is_lz77_length, .context = prefix_costs[pos].ctx, .value = val });
            pos -= prefix_costs[pos].len;
        }
        std.mem.reverse(Token, out.items);
    }
    return tokens_lz77;
}

/// Port of `jxl::ApplyLZ77` dispatcher. Returns an empty slice if LZ77 is
/// not beneficial (caller keeps original tokens).
pub fn applyLZ77(
    allocator: std.mem.Allocator,
    params: *const HistogramParams,
    num_contexts: usize,
    tokens: []const std.ArrayList(Token),
    lz77: *const LZ77Params,
) ![]std.ArrayList(Token) {
    switch (params.lz77_method) {
        .kRLE => return applyLZ77RLE(allocator, params, num_contexts, tokens, lz77),
        .kLZ77 => return applyLZ77LZ77(allocator, params, num_contexts, tokens, lz77),
        .kOptimal => return applyLZ77Optimal(allocator, params, num_contexts, tokens, lz77),
        .kNone => return &.{},
    }
}
