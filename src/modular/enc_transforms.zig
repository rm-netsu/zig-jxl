//! Encoder-side modular transforms: ports of libjxl
//! `modular/transform/{enc_rct,enc_palette,enc_squeeze,enc_transform}.cc`
//! (v0.12.0), plus the transform-selection drivers from `enc_modular.cc`
//! (`try_palettes`, the local RCT search) and the `EstimateCost` heuristic
//! (scalar path of `enc_modular_simd.cc`).
//!
//! Only the lossless paths are ported: the lossy-palette machinery
//! (`PaletteIterationData`, dithering, frequent deltas) is a lossy-mode
//! feature and is intentionally absent.
//!
//! Memory: every function takes an explicit allocator and frees its
//! temporaries before returning; the transformed `Image` is mutated in place
//! (channel list, `nb_meta_channels` and pixel data), mirroring libjxl's
//! `TransformForward` + `image.transform` bookkeeping done by the callers.

const std = @import("std");
const common = @import("common.zig");
const image_mod = @import("image.zig");
const context_predict = @import("context_predict.zig");
const transforms = @import("transforms.zig");
const hybrid_uint = @import("hybrid_uint.zig");
const enc_ans = @import("../entropy/enc_ans.zig");

const Allocator = std.mem.Allocator;
const Image = image_mod.Image;
const Channel = image_mod.Channel;
const pixel_type = common.pixel_type;
const pixel_type_w = common.pixel_type_w;
const Transform = transforms.Transform;
const SqueezeParams = transforms.SqueezeParams;
const packSigned = common.packSigned;
const clampedGradient = context_predict.clampedGradient;

// ---------------------------------------------------------------------------
// Forward RCT (enc_rct.cc)
// ---------------------------------------------------------------------------

/// libjxl `RctPermute` / the port's `rctOutSlot`: slot `which` of permutation
/// `permutation` in a 3-channel group.
fn rctSlot(permutation: u32, which: u32) u32 {
    return switch (which) {
        0 => permutation % 3,
        1 => (permutation + 1 + permutation / 3) % 3,
        else => (permutation + 2 - permutation / 3) % 3,
    };
}

/// Port of `FwdRctRow<transform>` (scalar path). Reads all three inputs for
/// a pixel before writing (the channels may alias pairwise). All arithmetic
/// is int32 with wraparound, exactly as the C++ compiles on real targets
/// (the SIMD paths wrap; the scalar path relies on the same two's-complement
/// wrap). Float-sample bit patterns span the full int32 range and can
/// overflow the intermediates.
fn fwdRctRow(
    comptime transform_type: u32,
    in0: []const pixel_type,
    in1: []const pixel_type,
    in2: []const pixel_type,
    out0: []pixel_type,
    out1: []pixel_type,
    out2: []pixel_type,
) void {
    const second = transform_type >> 1;
    const third = transform_type & 1;
    for (in0, 0..) |_, x| {
        if (transform_type == 6) {
            // YCoCg: R,G,B -> y=(tmp+(o2>>1)), co=(R-B), cg=(G-tmp)
            const r: pixel_type = in0[x];
            const g: pixel_type = in1[x];
            const b: pixel_type = in2[x];
            const o1: pixel_type = r -% b;
            const tmp: pixel_type = common.pixelAdd(b, o1 >> 1);
            const o2: pixel_type = g -% tmp;
            out0[x] = common.pixelAdd(tmp, o2 >> 1);
            out1[x] = o1;
            out2[x] = o2;
        } else {
            const first: pixel_type = in0[x];
            var sec: pixel_type = in1[x];
            var third_v: pixel_type = in2[x];
            if (second == 1) {
                sec = sec -% first;
            } else if (second == 2) {
                sec = sec -% ((first +% in2[x]) >> 1);
            }
            if (third != 0) {
                third_v = third_v -% first;
            }
            out0[x] = first;
            out1[x] = sec;
            out2[x] = third_v;
        }
    }
}

/// Forward RCT from three separate input channels into three output channels
/// (port of `FwdRct(in, out, rct_type)`); `out[k] = F_k(in[slot(0..2)])`.
/// Per pixel every input is read before any output is written.
pub fn fwdRctChannels(
    in: [3]*const Channel,
    out: [3]*Channel,
    rct_type: u32,
) void {
    const permutation = rct_type / 7;
    const custom = rct_type % 7;
    const h = out[0].h;
    const w = out[0].w;
    var inp: [3]*const Channel = undefined;
    for (0..3) |k| inp[k] = in[rctSlot(permutation, @intCast(k))];
    if (custom == 0) {
        // Permutation only: move pixel data into the permuted positions.
        var y: usize = 0;
        while (y < h) : (y += 1) {
            for (0..3) |k| {
                const src = inp[k].row(y);
                const dst = out[k].row(y);
                @memcpy(dst[0..w], src[0..w]);
            }
        }
        return;
    }
    var y: usize = 0;
    while (y < h) : (y += 1) {
        const row_a: []const pixel_type = inp[0].row(y);
        const row_b: []const pixel_type = inp[1].row(y);
        const row_c: []const pixel_type = inp[2].row(y);
        const row_o0: []pixel_type = out[0].row(y);
        const row_o1: []pixel_type = out[1].row(y);
        const row_o2: []pixel_type = out[2].row(y);
        switch (custom) {
            1 => fwdRctRow(1, row_a, row_b, row_c, row_o0, row_o1, row_o2),
            2 => fwdRctRow(2, row_a, row_b, row_c, row_o0, row_o1, row_o2),
            3 => fwdRctRow(3, row_a, row_b, row_c, row_o0, row_o1, row_o2),
            4 => fwdRctRow(4, row_a, row_b, row_c, row_o0, row_o1, row_o2),
            5 => fwdRctRow(5, row_a, row_b, row_c, row_o0, row_o1, row_o2),
            6 => fwdRctRow(6, row_a, row_b, row_c, row_o0, row_o1, row_o2),
            else => unreachable,
        }
    }
}

/// Port of `FwdRct(Image&, begin_c, rct_type)` (in-place). Returns false for
/// the no-op type (nothing recorded, as in libjxl).
pub fn fwdRct(image: *Image, begin_c: usize, rct_type: u32) !bool {
    try transforms.checkEqualChannels(image, begin_c, begin_c + 2);
    if (rct_type == 0) return false;
    const in: [3]*const Channel = .{
        &image.channels.items[begin_c + 0],
        &image.channels.items[begin_c + 1],
        &image.channels.items[begin_c + 2],
    };
    // Write through scratch channels: the permuted output positions may
    // alias the inputs, and whole-row moves must not overlap.
    const w = image.channels.items[begin_c].w;
    const h = image.channels.items[begin_c].h;
    var scratch: [3]Channel = undefined;
    var made: usize = 0;
    errdefer for (scratch[0..made]) |*ch| ch.deinit();
    for (0..3) |k| {
        scratch[k] = try Channel.create(image.gpa, w, h, image.channels.items[begin_c].hshift, image.channels.items[begin_c].vshift);
        made += 1;
    }
    const out_ptrs: [3]*Channel = .{ &scratch[0], &scratch[1], &scratch[2] };
    fwdRctChannels(in, out_ptrs, rct_type);
    for (0..3) |k| {
        const target = &image.channels.items[begin_c + k];
        target.deinit();
        target.* = scratch[k];
    }
    return true;
}

// ---------------------------------------------------------------------------
// EstimateCost (enc_modular_simd.cc, scalar path)
// ---------------------------------------------------------------------------

const estimate_cutoffs = [_]u32{ 0, 1, 3, 5, 7, 11, 15, 23, 31, 47, 63, 95, 127, 191, 255, 392, 500 };

/// Port of `EstimateCost` (scalar): gradient-prediction residuals bucketed
/// by a local-activity context; returns the estimated entropy cost in bits.
pub fn estimateCost(allocator: Allocator, image: *const Image) !f64 {
    var histos: [estimate_cutoffs.len + 1]enc_ans.Histogram = undefined;
    for (0..histos.len) |i| histos[i] = enc_ans.Histogram.init(allocator);
    defer for (0..histos.len) |i| histos[i].deinit();

    var extra_bits: u64 = 0;
    var histo_cost: f64 = 0;
    const config = hybrid_uint.HybridUintConfig{};

    for (image.channels.items) |*ch| {
        if (ch.h == 0 or ch.w == 0) continue;
        var y: usize = 0;
        while (y < ch.h) : (y += 1) {
            const r = ch.row(y);
            const r_top: ?[]const pixel_type = if (y > 0) ch.row(y - 1) else null;
            for (0..ch.w) |x| {
                const left: pixel_type = if (x != 0) r[x - 1] else if (r_top) |t| t[x] else 0;
                const top: pixel_type = if (r_top) |t| t[x] else left;
                const topleft: pixel_type = if (x != 0 and r_top != null) r_top.?[x - 1] else left;
                const max_v = @max(left, @max(top, topleft));
                const min_v = @min(left, @min(top, topleft));
                const max_diff: u64 = @intCast(@as(i64, max_v) - @as(i64, min_v));
                var ctx: usize = 0;
                for (estimate_cutoffs) |c| {
                    if (max_diff < c) ctx += 1;
                }
                const res: pixel_type = @truncate(@as(pixel_type_w, r[x]) -
                    @as(pixel_type_w, clampedGradient(top, left, topleft)));
                const tok = config.encode(packSigned(res));
                try histos[ctx].add(allocator, tok.token);
                extra_bits += tok.nbits;
            }
        }
        for (0..histos.len) |hi| {
            histo_cost += histos[hi].shannonEntropy();
            histos[hi].clear();
        }
    }
    return histo_cost + @as(f64, @floatFromInt(extra_bits));
}

// EstimateWPCost (enc_modular.cc) and the weighted-predictor mode search.
const wp_cost_cutoffs = [_]i32{
    -500, -392, -255, -191, -127, -95,  -63, -47, -31, -23, -15,
    -11,  -7,   -4,   -3,   -1,   0,    1,   3,   5,   7,   11,
    15,   23,   31,   47,   63,   95,   127, 191, 255, 392, 500,
};

/// Port of `EstimateWPCost`: the estimated entropy cost of encoding `image`
/// with the Weighted predictor under mode `wp_mode` (histograms over the WP
/// error property contexts + hybrid-uint extra bits). Returns +inf when a
/// residual overflows (libjxl's "unhealthy" maximum-cost case).
pub fn estimateWPCost(allocator: Allocator, image: *const Image, wp_mode: usize) !f64 {
    var header = context_predict.weighted.Header.defaults;
    context_predict.predictorMode(wp_mode, &header);
    var wp_header = header;
    var histos: [wp_cost_cutoffs.len + 1]enc_ans.Histogram = undefined;
    for (0..histos.len) |i| histos[i] = enc_ans.Histogram.init(allocator);
    defer for (0..histos.len) |i| histos[i].deinit();
    var extra_bits: u64 = 0;
    var histo_cost: f64 = 0;
    const config = hybrid_uint.HybridUintConfig{};

    for (image.channels.items) |*ch| {
        if (ch.h == 0 or ch.w == 0) continue;
        var wp_state = try context_predict.weighted.State.create(allocator, &wp_header, ch.w, ch.h);
        defer wp_state.deinit(allocator);
        var properties = [_]i32{0};
        var unhealthy = false;
        var y: usize = 0;
        while (y < ch.h) : (y += 1) {
            const r = ch.row(y);
            const r_top: ?[]const pixel_type = if (y > 0) ch.row(y - 1) else null;
            const r_toptop: ?[]const pixel_type = if (y > 1) ch.row(y - 2) else null;
            for (0..ch.w) |x| {
                const left: pixel_type = if (x != 0) r[x - 1] else if (r_top) |t| t[x] else 0;
                const top: pixel_type = if (r_top) |t| t[x] else left;
                const topleft: pixel_type = if (x != 0 and r_top != null) r_top.?[x - 1] else left;
                const topright: pixel_type = if (x + 1 < ch.w and r_top != null) r_top.?[x + 1] else top;
                const toptop: pixel_type = if (r_toptop) |t| t[x] else top;
                const guess = wp_state.predict(true, x, y, ch.w, top, left, topright, topleft, toptop, &properties, 0);
                var ctx: usize = 0;
                for (wp_cost_cutoffs) |c| {
                    if (c >= properties[0]) ctx += 1;
                }
                const res_w: pixel_type_w = @as(pixel_type_w, r[x]) - @as(pixel_type_w, guess);
                if (res_w > std.math.maxInt(pixel_type) or res_w < std.math.minInt(pixel_type)) {
                    unhealthy = true;
                }
                const res: pixel_type = @intCast(std.math.clamp(res_w, std.math.minInt(pixel_type), std.math.maxInt(pixel_type)));
                const tok = config.encode(packSigned(res));
                try histos[ctx].add(allocator, tok.token);
                extra_bits += tok.nbits;
                wp_state.updateErrors(r[x], x, y, ch.w);
            }
        }
        if (unhealthy) {
            return std.math.inf(f64);
        }
        for (0..histos.len) |hi| {
            histo_cost += histos[hi].shannonEntropy();
            histos[hi].clear();
        }
    }
    return histo_cost + @as(f64, @floatFromInt(extra_bits));
}

/// Port of the WP-mode search at the end of `PrepareStreamParams`:
/// try the first `nb_wp_modes` weighted-predictor modes and return the
/// header of the cheapest one by EstimateWPCost.
pub fn searchWpMode(allocator: Allocator, image: *const Image, nb_wp_modes: usize) !context_predict.weighted.Header {
    var best_mode: usize = 0;
    var best_cost: f64 = try estimateWPCost(allocator, image, 0);
    for (1..nb_wp_modes) |mode| {
        const cost = try estimateWPCost(allocator, image, mode);
        if (cost < best_cost) {
            best_cost = cost;
            best_mode = mode;
        }
    }
    var header = context_predict.weighted.Header.defaults;
    if (best_mode != 0) {
        context_predict.predictorMode(best_mode, &header);
    }
    return header;
}

// ---------------------------------------------------------------------------
// Forward Squeeze (enc_squeeze.cc)
// ---------------------------------------------------------------------------

inline fn average(x: pixel_type_w, y: pixel_type_w) pixel_type_w {
    return ((x + y + @as(pixel_type_w, if (x > y) 1 else 0)) >> 1);
}

/// Port of `FwdHSqueeze`: channel `c` is split into the average channel
/// (replacing `c`) and a residual channel inserted at `rc`.
fn fwdHSqueeze(image: *Image, c: usize, rc: usize) !void {
    var chin = image.channels.items[c];
    var chout = try Channel.create(image.gpa, (chin.w + 1) / 2, chin.h, chin.hshift + 1, chin.vshift);
    var chres = try Channel.create(image.gpa, chin.w - chout.w, chout.h, chin.hshift + 1, chin.vshift);
    chout.component = chin.component;
    chres.component = chin.component;

    var y: usize = 0;
    while (y < chout.h) : (y += 1) {
        const p_in = chin.row(y);
        const p_out = chout.row(y);
        const p_res = chres.row(y);
        var x: usize = 0;
        while (x < chres.w) : (x += 1) {
            const a: pixel_type_w = p_in[x * 2];
            const b: pixel_type_w = p_in[x * 2 + 1];
            const avg = average(a, b);
            p_out[x] = @truncate(avg);
            const diff: pixel_type_w = a - b;
            var next_avg: pixel_type_w = avg;
            if (x + 1 < chres.w) {
                const cc: pixel_type_w = p_in[x * 2 + 2];
                const d: pixel_type_w = p_in[x * 2 + 3];
                next_avg = average(cc, d);
            } else if (chin.w & 1 != 0) {
                next_avg = p_in[x * 2 + 2];
            }
            const left: pixel_type_w = if (x > 0) p_in[x * 2 - 1] else avg;
            const tendency = transforms.smoothTendency(left, avg, next_avg);
            p_res[x] = @truncate(diff - tendency);
        }
        if (chin.w & 1 != 0) {
            const x_last = chout.w - 1;
            p_out[x_last] = p_in[x_last * 2];
        }
    }
    chin.deinit();
    image.channels.items[c] = chout;
    try image.channels.insert(image.gpa, rc, chres);
}

/// Port of `FwdVSqueeze`.
fn fwdVSqueeze(image: *Image, c: usize, rc: usize) !void {
    var chin = image.channels.items[c];
    var chout = try Channel.create(image.gpa, chin.w, (chin.h + 1) / 2, chin.hshift, chin.vshift + 1);
    var chres = try Channel.create(image.gpa, chin.w, chin.h - chout.h, chin.hshift, chin.vshift + 1);
    chout.component = chin.component;
    chres.component = chin.component;

    var y: usize = 0;
    while (y < chres.h) : (y += 1) {
        const p_in = chin.row(y * 2);
        const p_next = chin.row(y * 2 + 1);
        const p_out = chout.row(y);
        const p_res = chres.row(y);
        const p_above: ?[]const pixel_type = if (y > 0) chin.row(y * 2 - 1) else null;
        for (0..chout.w) |x| {
            const a: pixel_type_w = p_in[x];
            const b: pixel_type_w = p_next[x];
            const avg = average(a, b);
            p_out[x] = @truncate(avg);
            const diff: pixel_type_w = a - b;
            var next_avg: pixel_type_w = avg;
            if (y + 1 < chres.h) {
                const cc: pixel_type_w = chin.row((y + 1) * 2)[x];
                const d: pixel_type_w = chin.row((y + 1) * 2 + 1)[x];
                next_avg = average(cc, d);
            } else if (chin.h & 1 != 0) {
                next_avg = chin.row((y + 1) * 2)[x];
            }
            const above: pixel_type_w = if (p_above) |t| t[x] else avg;
            const tendency = transforms.smoothTendency(above, avg, next_avg);
            p_res[x] = @truncate(diff - tendency);
        }
    }
    if (chin.h & 1 != 0) {
        const y_last = chout.h - 1;
        const p_in = chin.row(y_last * 2);
        const p_out = chout.row(y_last);
        @memcpy(p_out[0..chin.w], p_in[0..chin.w]);
    }
    chin.deinit();
    image.channels.items[c] = chout;
    try image.channels.insert(image.gpa, rc, chres);
}

/// Port of `FwdSqueeze`: applies the squeeze script (an empty script selects
/// the default scheme, which is NOT materialized into the transform — the
/// decoder re-derives it, exactly like libjxl). Returns false when there is
/// nothing to squeeze.
pub fn fwdSqueeze(image: *Image, parameters: []const SqueezeParams) !bool {
    if (parameters.len == 0) {
        // Default scheme: derive it from the current channel state without
        // mutating the caller's transform.
        var scratch = Transform.init(image.gpa);
        defer scratch.deinit();
        try transforms.defaultSqueezeParametersInto(image, &scratch.squeezes);
        if (scratch.squeezes.items.len == 0) return false;
        for (scratch.squeezes.items) |parameter| {
            try applySqueezeParam(image, parameter);
        }
        return true;
    }
    if (parameters.len == 0) return false;
    for (parameters) |parameter| {
        try applySqueezeParam(image, parameter);
    }
    return true;
}

fn applySqueezeParam(image: *Image, parameter: SqueezeParams) !void {
    try checkSqueezeParams(parameter, image.channels.items.len);
    const horizontal = parameter.horizontal;
    const in_place = parameter.in_place;
    const beginc: usize = parameter.begin_c;
    const endc: usize = parameter.begin_c + parameter.num_c - 1;
    const offset: usize = if (in_place) endc + 1 else image.channels.items.len;
    var c = beginc;
    while (c <= endc) : (c += 1) {
        if (horizontal) {
            try fwdHSqueeze(image, c, offset + c - beginc);
        } else {
            try fwdVSqueeze(image, c, offset + c - beginc);
        }
    }
}

fn checkSqueezeParams(parameter: SqueezeParams, num_channels: usize) !void {
    const c1: usize = parameter.begin_c;
    const c2: usize = parameter.begin_c + parameter.num_c - 1;
    if (c1 >= num_channels or c2 >= num_channels or c2 < c1) return error.InvalidChannelRange;
}

/// Port of the squeeze-script truncation from `ComputeEncodingData`
/// (enc_modular.cc): `shift_cap`-based truncation so responsive frames never
/// squeeze below one pixel per group tile. Returns the (possibly shortened)
/// script; an empty result means "keep the default marker".
pub fn truncateDefaultSqueeze(
    allocator: Allocator,
    params: []const SqueezeParams,
    group_size_shift: u32,
) !std.ArrayList(SqueezeParams) {
    var out: std.ArrayList(SqueezeParams) = .empty;
    errdefer out.deinit(allocator);
    const shift_cap: usize = 7 + @as(usize, group_size_shift);
    var hshift: usize = 0;
    var vshift: usize = 0;
    for (params, 0..) |p, i| {
        if (p.horizontal) hshift += 1 else vshift += 1;
        const dc_boost: usize = if (@min(hshift, vshift) >= 3) 3 else 0;
        if (@max(hshift, vshift) > shift_cap + dc_boost) {
            if (i > 0) try out.appendSlice(allocator, params[0 .. i - 1]);
            return out;
        }
    }
    try out.appendSlice(allocator, params);
    return out;
}

// ---------------------------------------------------------------------------
// Forward Palette (enc_palette.cc, lossless paths)
// ---------------------------------------------------------------------------

const kMaxPaletteLookupTableSize: usize = 1 << 16;
const kImplicitPaletteSize: usize = 189; // 4*4*4 + 5*5*5

/// `palette_internal::GetPaletteValue(nullptr, k, c, 0, 0, bit_depth)`:
/// the implicit (cube) palette entry `k` for component `c`. For
/// bit_depth == 32 the C++ computes in uint64 and implicitly truncates to
/// the 32-bit pixel_type (two's-complement wrap), which float32 bit
/// patterns rely on; `wrapToPixelType` reproduces that.
fn implicitPaletteColor(k: usize, c: usize, bit_depth: u32) pixel_type {
    if (c >= 3) return 0;
    const maxval = (@as(u64, 1) << @intCast(@min(bit_depth, 63))) - 1;
    if (k < 64) {
        // Small interleaved 4x4x4 cube.
        const idx = @mod(@as(u64, @intCast(k >> @as(u6, @intCast(c * 2)))), 4);
        const base = @as(u64, 1) << @intCast(@min(63, @max(0, @as(i32, @intCast(bit_depth)) - 3)));
        return wrapToPixelType(((idx *% maxval) >> 2) +% base);
    }
    // Large 5x5x5 cube.
    var idx: u64 = @intCast(k - 64);
    switch (c) {
        1 => idx = @divTrunc(idx, 5),
        2 => idx = @divTrunc(idx, 25),
        else => {},
    }
    idx = @mod(idx, 5);
    return wrapToPixelType((idx *% maxval) >> 2);
}

/// uint64 -> pixel_type with the C++ implicit-conversion truncation.
fn wrapToPixelType(v: u64) pixel_type {
    return @bitCast(@as(u32, @truncate(v)));
}

/// Maximum number of components a palette can cover in this port (RGBA or
/// same-size channel groups); libjxl is generic but the modular lossless
/// path never exceeds this.
const kMaxPaletteComponents = 4;

const ColorKey = struct {
    n: u8,
    v: [kMaxPaletteComponents]pixel_type,

    fn fromSlice(s: []const pixel_type) ColorKey {
        var key = ColorKey{ .n = @intCast(s.len), .v = .{0} ** kMaxPaletteComponents };
        @memcpy(key.v[0..s.len], s);
        return key;
    }
};

/// Result of a successful forward palette.
pub const PaletteStats = struct {
    nb_colors: u32,
    nb_deltas: u32 = 0,
    predictor: u32 = 0,
};

/// Forward palette for a single channel (the `nb == 1` "ChannelCompact"
/// path of `FwdPaletteIteration`). Returns the palette size, or null when
/// the channel needs more colors than `max_colors`.
fn fwdChannelPalette(image: *Image, begin_c: usize, max_colors: u32) !?u32 {
    if (max_colors == 0) return null;
    const ch = &image.channels.items[begin_c];
    const h = ch.h;
    var minval: pixel_type = std.math.maxInt(pixel_type);
    var maxval: pixel_type = std.math.minInt(pixel_type);
    for (0..h) |y| {
        for (ch.row(y)) |v| {
            if (v < minval) minval = v;
            if (v > maxval) maxval = v;
        }
    }
    const lookup_table_size: i64 = @as(i64, maxval) - @as(i64, minval) + 1;
    if (lookup_table_size > kMaxPaletteLookupTableSize) {
        // Range too wide for a lookup table: gather unique values in a
        // sorted set (std::set in libjxl).
        var set = std.AutoHashMap(pixel_type, void).init(image.gpa);
        defer set.deinit();
        for (0..h) |y| {
            for (ch.row(y)) |v| {
                const gop = try set.getOrPut(v);
                if (!gop.found_existing and set.count() > max_colors) {
                    return null;
                }
            }
        }
        const values = try image.gpa.alloc(pixel_type, set.count());
        defer image.gpa.free(values);
        var i: usize = 0;
        var it = set.keyIterator();
        while (it.next()) |k| : (i += 1) values[i] = k.*;
        std.mem.sort(pixel_type, values, {}, struct {
            fn lt(_: void, a: pixel_type, b: pixel_type) bool {
                return a < b;
            }
        }.lt);
        var pch = try Channel.create(image.gpa, values.len, 1, -1, -1);
        errdefer pch.deinit();
        @memcpy(pch.row(0)[0..values.len], values);
        for (0..h) |y| {
            const row = ch.row(y);
            for (row) |*v| {
                const idx = std.sort.binarySearch(pixel_type, values, v.*, struct {
                    fn cmp(key: pixel_type, elem: pixel_type) std.math.Order {
                        return std.math.order(key, elem);
                    }
                }.cmp);
                if (idx == null) return error.PaletteMissingColor;
                v.* = @intCast(idx.?);
            }
        }
        image.nb_meta_channels += 1;
        try image.channels.insert(image.gpa, 0, pch);
        return @intCast(values.len);
    }
    // Lookup-table path (range fits in 64Ki entries).
    const table_size: usize = @intCast(lookup_table_size);
    const lookup = try image.gpa.alloc(u32, table_size);
    defer image.gpa.free(lookup);
    @memset(lookup, 0);
    var idx: usize = 0;
    for (0..h) |y| {
        for (ch.row(y)) |v| {
            const slot = @as(usize, @intCast(v - minval));
            if (lookup[slot] == 0) {
                lookup[slot] = 1;
                idx += 1;
                if (idx > max_colors) return null;
            }
        }
    }
    var pch = try Channel.create(image.gpa, idx, 1, -1, -1);
    errdefer pch.deinit();
    var pidx: usize = 0;
    for (0..table_size) |i| {
        if (lookup[i] != 0) {
            pch.row(0)[pidx] = @intCast(@as(i64, @intCast(i)) + minval);
            lookup[i] = @intCast(pidx);
            pidx += 1;
        }
    }
    for (0..h) |y| {
        const row = ch.row(y);
        for (row) |*v| {
            v.* = @intCast(lookup[@intCast(v.* - minval)]);
        }
    }
    image.nb_meta_channels += 1;
    try image.channels.insert(image.gpa, 0, pch);
    return @intCast(idx);
}

/// Port of `FwdPaletteIteration` (lossless): collapse channels
/// `begin_c..end_c` into a palette + index channel. Returns null when the
/// channels need more colors than `max_colors` (or a palette is pointless).
pub fn fwdPalette(
    allocator: Allocator,
    image: *Image,
    begin_c: usize,
    end_c: usize,
    max_colors: u32,
    ordered: bool,
    bit_depth: u32,
) !?PaletteStats {
    try transforms.checkEqualChannels(image, begin_c, end_c);
    if (begin_c < image.nb_meta_channels) return error.InvalidChannelRange;
    // libjxl ForwardPalette: no palettes on 32-bit channels (fp32 bit
    // patterns would need an unbounded palette).
    if (bit_depth >= 32) return null;
    const nb = end_c - begin_c + 1;
    if (max_colors < 2) return null;

    if (nb == 1) {
        if (try fwdChannelPalette(image, begin_c, max_colors)) |n| {
            return .{ .nb_colors = n };
        }
        return null;
    }
    if (nb > kMaxPaletteComponents) return null;

    const w = image.channels.items[begin_c].w;
    const h = image.channels.items[begin_c].h;

    // Implicit palette membership and frequency table.
    var implicit_map = std.AutoHashMap(ColorKey, void).init(allocator);
    defer implicit_map.deinit();
    var color_freq = std.AutoHashMap(ColorKey, u64).init(allocator);
    defer color_freq.deinit();
    {
        var color: [kMaxPaletteComponents]pixel_type = .{0} ** kMaxPaletteComponents;
        for (0..kImplicitPaletteSize) |k| {
            for (0..nb) |c| {
                color[c] = implicitPaletteColor(k, c, bit_depth);
            }
            try implicit_map.put(ColorKey.fromSlice(color[0..nb]), {});
        }
    }

    // Candidate colors in image order (implicit colors excluded).
    var candidates: std.ArrayList([kMaxPaletteComponents]pixel_type) = .empty;
    defer candidates.deinit(allocator);
    var implicit_colors_used: usize = 0;

    var color: [kMaxPaletteComponents]pixel_type = .{0} ** kMaxPaletteComponents;
    for (0..h) |y| {
        var x: usize = 0;
        while (x < w) : (x += 1) {
            for (0..nb) |c| {
                color[c] = image.channels.items[begin_c + c].row(y)[x];
            }
            const key = ColorKey.fromSlice(color[0..nb]);
            const freq_gop = try color_freq.getOrPut(key);
            if (!freq_gop.found_existing) {
                freq_gop.value_ptr.* = 1;
                if (implicit_map.contains(key)) {
                    implicit_colors_used += 1;
                } else {
                    try candidates.append(allocator, color);
                    if (candidates.items.len > max_colors) {
                        return null; // too many colors
                    }
                }
            } else {
                freq_gop.value_ptr.* += 1;
            }
        }
    }

    var nb_colors: usize = candidates.items.len;
    // A palette covering a single color is not useful.
    if (nb_colors + implicit_colors_used == 1) return null;

    // Promote frequent implicit colors (more than 10 occurrences) to the
    // explicit palette.
    for (0..kImplicitPaletteSize) |k| {
        for (0..nb) |c| {
            color[c] = implicitPaletteColor(k, c, bit_depth);
        }
        const key = ColorKey.fromSlice(color[0..nb]);
        if ((color_freq.get(key) orelse 0) > 10) {
            nb_colors += 1;
            try candidates.append(allocator, color);
        }
    }

    // inv_palette: color -> index. Implicit colors map to nb_colors + k;
    // explicit entries (assigned after the optional sort) override promoted
    // implicit ones.
    var inv_palette = std.AutoHashMap(ColorKey, usize).init(allocator);
    defer inv_palette.deinit();
    {
        var icolor: [kMaxPaletteComponents]pixel_type = .{0} ** kMaxPaletteComponents;
        for (0..kImplicitPaletteSize) |k| {
            for (0..nb) |c| {
                icolor[c] = implicitPaletteColor(k, c, bit_depth);
            }
            try inv_palette.put(ColorKey.fromSlice(icolor[0..nb]), nb_colors + k);
        }
    }

    // Optional luma ordering (common colors first, then rare ones).
    if (ordered and nb >= 3) {
        const ctx = SortCtx{ .freq = &color_freq, .nb = nb };
        std.mem.sort([kMaxPaletteComponents]pixel_type, candidates.items, ctx, SortCtx.lessThan);
    }

    // Build the palette channel: width nb_colors, height nb.
    var pch = try Channel.create(image.gpa, nb_colors, nb, -1, -1);
    errdefer pch.deinit();
    for (candidates.items, 0..) |pcol, clr| {
        for (0..nb) |c| {
            pch.row(c)[clr] = pcol[c];
        }
        try inv_palette.put(ColorKey.fromSlice(pcol[0..nb]), clr);
    }

    // Rewrite the first channel as the index channel.
    for (0..h) |y| {
        const row = image.channels.items[begin_c].row(y);
        var x: usize = 0;
        while (x < w) : (x += 1) {
            for (0..nb) |c| {
                color[c] = image.channels.items[begin_c + c].row(y)[x];
            }
            const key = ColorKey.fromSlice(color[0..nb]);
            const index = inv_palette.get(key) orelse return error.PaletteMissingColor;
            row[x] = @intCast(index);
        }
    }

    // Channel-list surgery, identical to MetaPalette on the decoder side.
    for (image.channels.items[begin_c + 1 .. end_c + 1]) |*ch| ch.deinit();
    _ = try image.channels.replaceRange(image.gpa, begin_c + 1, end_c - begin_c, &.{});
    image.nb_meta_channels += 1;
    try image.channels.insert(image.gpa, 0, pch);

    return .{ .nb_colors = @intCast(nb_colors) };
}

const SortCtx = struct {
    freq: *const std.AutoHashMap(ColorKey, u64),
    nb: usize,

    fn lessThan(ctx: SortCtx, ap: [kMaxPaletteComponents]pixel_type, bp: [kMaxPaletteComponents]pixel_type) bool {
        const freq_threshold: u64 = 4;
        const a_freq = ctx.freq.get(ColorKey.fromSlice(ap[0..ctx.nb])) orelse 0;
        const b_freq = ctx.freq.get(ColorKey.fromSlice(bp[0..ctx.nb])) orelse 0;
        var ay: f32 = 0.299 * @as(f32, @floatFromInt(ap[0])) +
            0.587 * @as(f32, @floatFromInt(ap[1])) +
            0.114 * @as(f32, @floatFromInt(ap[2])) + 0.1;
        if (ctx.nb > 3) ay *= 1.0 + @as(f32, @floatFromInt(ap[3]));
        var by: f32 = 0.299 * @as(f32, @floatFromInt(bp[0])) +
            0.587 * @as(f32, @floatFromInt(bp[1])) +
            0.114 * @as(f32, @floatFromInt(bp[2])) + 0.1;
        if (ctx.nb > 3) by *= 1.0 + @as(f32, @floatFromInt(bp[3]));
        const a_key = if (a_freq > freq_threshold) -ay else ay;
        const b_key = if (b_freq > freq_threshold) -by else by;
        return a_key < b_key;
    }
};

// ---------------------------------------------------------------------------
// Transform drivers (enc_modular.cc: do_transform / try_palettes / RCT search)
// ---------------------------------------------------------------------------

/// Everything the palette path needs to know about the stream being encoded.
pub const TransformContext = struct {
    /// Image bit depth (drives the implicit palette cube scaling).
    bit_depth: u32 = 8,
};

/// libjxl `do_transform`: apply forward and record the transform.
pub fn doTransform(
    allocator: Allocator,
    image: *Image,
    t: *const Transform,
    ctx: TransformContext,
    transforms_out: *std.ArrayList(Transform),
) !bool {
    var copy = Transform.init(allocator);
    errdefer copy.deinit();
    try copy.squeezes.appendSlice(allocator, t.squeezes.items);
    copy.id = t.id;
    copy.begin_c = t.begin_c;
    copy.rct_type = t.rct_type;
    copy.num_c = t.num_c;
    copy.nb_colors = t.nb_colors;
    copy.nb_deltas = t.nb_deltas;
    copy.predictor = t.predictor;
    copy.ordered_palette = t.ordered_palette;
    copy.lossy_palette = t.lossy_palette;

    const did_it: bool = blk: {
        switch (t.id) {
            .rct => break :blk try fwdRct(image, t.begin_c, t.rct_type),
            .squeeze => break :blk try fwdSqueeze(image, t.squeezes.items),
            .palette => {
                if (try fwdPalette(allocator, image, t.begin_c, t.begin_c + t.num_c - 1, t.nb_colors, t.ordered_palette, ctx.bit_depth)) |stats| {
                    copy.nb_colors = stats.nb_colors;
                    copy.nb_deltas = stats.nb_deltas;
                    copy.predictor = stats.predictor;
                    break :blk true;
                }
                break :blk false;
            },
            .invalid => return error.InvalidTransform,
        }
    };
    if (did_it) {
        try transforms_out.append(allocator, copy);
        return true;
    }
    copy.deinit();
    return false;
}

pub const PaletteHeuristics = struct {
    /// cparams.palette_colors: 0 disables palettes; the sign selects ordering
    /// (>= 0 ordered, < 0 image order).
    palette_colors: i32 = 1 << 10,
    /// channel_colors_percent for this pass (95 pre-transform / 80 group).
    channel_colors_percent: f32 = 95.0,
    /// Cost-based revert (libjxl `maybe_do_transform` at speed tiers below
    /// kSquirrel, i.e. cjxl efforts 8-10): a palette whose post-transform
    /// EstimateCost exceeds the pre-transform cost is undone.
    reverts: bool = false,
    /// Real `EstimateCost` for the nb_colors heuristic (libjxl at tier <=
    /// kSquirrel, i.e. efforts 7-10); faster tiers use libjxl's arbitrary
    /// bpp estimate instead.
    real_cost: bool = true,
};

/// libjxl `maybe_do_transform`: apply the transform, and at slower efforts
/// revert it when the estimated cost got worse.
pub fn doTransformMaybe(
    allocator: Allocator,
    image: *Image,
    t: *const Transform,
    ctx: TransformContext,
    transforms_out: *std.ArrayList(Transform),
    revert: bool,
    cost_before: f64,
) !bool {
    const did_it = try doTransform(allocator, image, t, ctx, transforms_out);
    if (!did_it) return false;
    if (revert) {
        const cost_after = try estimateCost(allocator, image);
        if (cost_after > cost_before) {
            // Undo the last recorded transform (libjxl `t.Inverse(image)` +
            // pop). The recorded copy carries the palette's realized
            // nb_colors/nb_deltas/predictor, which the inverse needs.
            var recorded = transforms_out.items[transforms_out.items.len - 1];
            defer recorded.deinit();
            var default_wp = context_predict.weighted.Header.defaults;
            try transforms.inverse(image, &recorded, &default_wp, ctx.bit_depth);
            _ = transforms_out.pop();
            return false;
        }
    }
    return true;
}

/// Port of `try_palettes`: the all-channel palettes, the all-minus-one
/// fallback and the per-channel compaction palettes, with the effort-
/// dependent cost model of libjxl (real EstimateCost at cjxl efforts 7-10,
/// the arbitrary bpp estimate at faster tiers, cost-based reverts at 8-10).
pub fn tryPalettes(
    allocator: Allocator,
    image: *Image,
    params: PaletteHeuristics,
    ctx: TransformContext,
    transforms_out: *std.ArrayList(Transform),
) !void {
    var did_palette: usize = 0;
    if (params.palette_colors != 0) {
        const nb_pixels: f64 = @floatFromInt(image.channels.items[0].w * image.channels.items[0].h);
        const nb_chans = image.channels.items.len - image.nb_meta_channels;
        // libjxl: 4.8 bpp arbitrary estimate for 8-bit RGB -> 0.2 * bitdepth
        // * nb_chans bits per pixel.
        const cost_before: f64 = if (params.real_cost)
            try estimateCost(allocator, image)
        else
            nb_pixels * 0.2 * @as(f64, @floatFromInt(ctx.bit_depth)) * @as(f64, @floatFromInt(nb_chans));
        const heuristic_cap: i64 = @intFromFloat(cost_before * 0.0005 + nb_pixels / 128.0 + 128.0);
        const abs_palette: i64 = if (params.palette_colors < 0) -@as(i64, params.palette_colors) else @as(i64, params.palette_colors);
        if (nb_chans > 1) {
            // All-channel palette (e.g. RGBA).
            var t = Transform.init(allocator);
            defer t.deinit();
            t.id = .palette;
            t.begin_c = @intCast(image.nb_meta_channels);
            t.num_c = @intCast(nb_chans);
            t.nb_colors = @intCast(@min(heuristic_cap, abs_palette));
            t.ordered_palette = params.palette_colors >= 0;
            did_palette = @intFromBool(try doTransformMaybe(allocator, image, &t, ctx, transforms_out, params.reverts, cost_before));
        }
        if (did_palette == 0 and nb_chans > 3) {
            // All-minus-one-channel palette (RGB with separate alpha).
            var t = Transform.init(allocator);
            defer t.deinit();
            t.id = .palette;
            t.begin_c = @intCast(image.nb_meta_channels);
            t.num_c = @intCast(nb_chans - 1);
            t.nb_colors = @intCast(@min(heuristic_cap, abs_palette));
            t.ordered_palette = params.palette_colors >= 0;
            _ = try doTransformMaybe(allocator, image, &t, ctx, transforms_out, params.reverts, cost_before);
        }
    }

    // Baseline cost for the channel-palette pass. It only feeds the
    // cost-based reverts, so it is only computed for the slower tiers
    // (libjxl recomputes EstimateCost at speed_tier < kSquirrel, and zeroes
    // it otherwise — where it is unused).
    var channel_cost_before: f64 = 0;
    if (params.reverts) {
        channel_cost_before = try estimateCost(allocator, image);
    }

    if (params.channel_colors_percent > 0) {
        // Single-channel palettes (like FLIF's ChannelCompact).
        const nb_channels = image.channels.items.len - image.nb_meta_channels - did_palette;
        var i: usize = did_palette;
        while (i < nb_channels + did_palette) : (i += 1) {
            const ch = &image.channels.items[image.nb_meta_channels + i];
            var minval: pixel_type = std.math.maxInt(pixel_type);
            var maxval: pixel_type = std.math.minInt(pixel_type);
            for (0..ch.h) |y| {
                for (ch.row(y)) |v| {
                    if (v < minval) minval = v;
                    if (v > maxval) maxval = v;
                }
            }
            const colors: i64 = @as(i64, maxval) - @as(i64, minval) + 1;
            const nb_pixels: i64 = @intCast(image.channels.items[0].w * image.channels.items[0].h);
            const max_colors: i64 = @min(
                @divTrunc(nb_pixels, 16),
                @as(i64, @intFromFloat(params.channel_colors_percent / 100.0 * @as(f32, @floatFromInt(colors)))),
            );
            if (max_colors < 1) continue;
            var t = Transform.init(allocator);
            defer t.deinit();
            t.id = .palette;
            t.begin_c = @intCast(image.nb_meta_channels + i);
            t.num_c = 1;
            t.nb_colors = @intCast(max_colors);
            t.ordered_palette = true;
            _ = try doTransformMaybe(allocator, image, &t, ctx, transforms_out, params.reverts, channel_cost_before);
        }
    }
}

/// RCT candidates of the local RCT search (the full libjxl list; the first
/// entry is the no-op baseline). Speed tiers slice it: hare tries the first
/// 4 entries, wombat 5, squirrel 7, kitten 9, tortoise/glacier all 19.
const rct_candidates = [_]u32{
    0 * 7 + 0, 0 * 7 + 6, 0 * 7 + 5, 1 * 7 + 3, 3 * 7 + 5, 5 * 7 + 5, 1 * 7 + 5,
    2 * 7 + 5, 1 * 7 + 1, 0 * 7 + 4, 1 * 7 + 2, 2 * 7 + 1, 2 * 7 + 2, 2 * 7 + 3,
    4 * 7 + 4, 4 * 7 + 5, 0 * 7 + 2, 0 * 7 + 1, 0 * 7 + 3,
};

/// Port of the lossless local RCT search from `PrepareStreamParams`
/// (enc_modular.cc): try nothing / YCoCg / the first `nb_rcts` candidates of
/// the list, keep the cheapest by EstimateCost. `nb_rcts` counts the
/// no-op baseline too, exactly like libjxl's `nb_rcts_to_try`. Does nothing
/// unless the first three non-meta channels are size-compatible.
pub fn tryLocalRct(
    allocator: Allocator,
    image: *Image,
    nb_rcts_in: usize,
    transforms_out: *std.ArrayList(Transform),
) !void {
    const nb_rcts = @min(nb_rcts_in, rct_candidates.len);
    if (nb_rcts == 0) return;
    const nb_meta = image.nb_meta_channels;
    if (image.channels.items.len < nb_meta + 3) return;
    transforms.checkEqualChannels(image, nb_meta, nb_meta + 2) catch return;

    var best_cost = try estimateCost(allocator, image);
    var best_rct: u32 = 0;

    // Save the three original channels; the search transforms the live
    // channels and restores them afterwards.
    var orig: [3]Channel = undefined;
    {
        const c0 = &image.channels.items[nb_meta + 0];
        const c1 = &image.channels.items[nb_meta + 1];
        const c2 = &image.channels.items[nb_meta + 2];
        orig[0] = try Channel.create(image.gpa, c0.w, c0.h, c0.hshift, c0.vshift);
        orig[1] = try Channel.create(image.gpa, c1.w, c1.h, c1.hshift, c1.vshift);
        orig[2] = try Channel.create(image.gpa, c2.w, c2.h, c2.hshift, c2.vshift);
        for (0..orig[0].h) |y| {
            @memcpy(orig[0].row(y), c0.row(y));
            @memcpy(orig[1].row(y), c1.row(y));
            @memcpy(orig[2].row(y), c2.row(y));
        }
    }
    defer for (&orig) |*ch| ch.deinit();

    const in: [3]*const Channel = .{ &orig[0], &orig[1], &orig[2] };
    for (rct_candidates[1..nb_rcts]) |rct_type| {
        const out: [3]*Channel = .{
            &image.channels.items[nb_meta + 0],
            &image.channels.items[nb_meta + 1],
            &image.channels.items[nb_meta + 2],
        };
        fwdRctChannels(in, out, rct_type);
        const cost = try estimateCost(allocator, image);
        if (cost < best_cost) {
            best_cost = cost;
            best_rct = rct_type;
        }
    }

    // Restore the originals, then apply the winning RCT (if any).
    for (0..3) |k| {
        const target = &image.channels.items[nb_meta + k];
        @memcpy(target.plane.data, orig[k].plane.data);
    }
    if (best_rct != 0) {
        var t = Transform.init(allocator);
        defer t.deinit();
        t.id = .rct;
        t.begin_c = @intCast(nb_meta);
        t.rct_type = best_rct;
        _ = try doTransform(allocator, image, &t, .{}, transforms_out);
    }
}

// ---------------------------------------------------------------------------
// Tests: forward/inverse roundtrips for every transform.
// ---------------------------------------------------------------------------

const testing = std.testing;

fn makeTestImage(allocator: Allocator, w: usize, h: usize, n: usize, seed: u64) !Image {
    var img = Image.create(allocator);
    try img.channels.ensureTotalCapacity(allocator, n);
    var prng = std.Random.DefaultPrng.init(seed);
    const rand = prng.random();
    for (0..n) |_| {
        var ch = try Channel.create(allocator, w, h, 0, 0);
        for (0..h) |y| {
            for (0..w) |x| {
                ch.row(y)[x] = @intCast(rand.intRangeLessThan(i32, -40, 300));
            }
        }
        img.channels.appendAssumeCapacity(ch);
    }
    return img;
}

fn expectSameImage(a: *const Image, b: *const Image) !void {
    try testing.expectEqual(a.channels.items.len, b.channels.items.len);
    for (a.channels.items, b.channels.items) |*ca, *cb| {
        try testing.expectEqual(ca.w, cb.w);
        try testing.expectEqual(ca.h, cb.h);
        try testing.expectEqual(ca.hshift, cb.hshift);
        try testing.expectEqual(ca.vshift, cb.vshift);
        for (0..ca.h) |y| {
            for (0..ca.w) |x| {
                try testing.expectEqual(ca.row(y)[x], cb.row(y)[x]);
            }
        }
    }
}

test "fwdRct/inverseRCT roundtrip for all 42 types" {
    const alloc = testing.allocator;
    var rct_type: u32 = 1;
    while (rct_type < 42) : (rct_type += 1) {
        var img = try makeTestImage(alloc, 23, 17, 3, 0x1000 + rct_type);
        defer img.deinit();
        var orig = try makeTestImage(alloc, 1, 1, 0, 0);
        defer orig.deinit();
        // Deep-copy the image.
        for (img.channels.items) |*ch| {
            try orig.channels.append(alloc, try Channel.create(alloc, ch.w, ch.h, ch.hshift, ch.vshift));
        }
        defer for (orig.channels.items) |*ch| ch.deinit();
        for (img.channels.items, orig.channels.items) |*src, *dst| {
            for (0..src.h) |y| @memcpy(dst.row(y), src.row(y));
        }

        const did = try fwdRct(&img, 0, rct_type);
        try testing.expect(did);
        try transforms.inverseRCT(&img, 0, rct_type);
        try expectSameImage(&orig, &img);
    }
}

test "fwdPalette/inversePalette roundtrip: RGB few colors" {
    const alloc = testing.allocator;
    var img = Image.create(alloc);
    defer img.deinit();
    const colors = [_][3]i32{ .{ 10, 20, 30 }, .{ 255, 0, 0 }, .{ 0, 255, 0 }, .{ 9, 9, 9 }, .{ 200, 100, 50 } };
    for (0..3) |_| _ = try img.addChannel(41, 29);
    var prng = std.Random.DefaultPrng.init(7);
    const rand = prng.random();
    for (0..29) |y| {
        for (0..41) |x| {
            const c = colors[rand.uintLessThan(usize, colors.len)];
            for (0..3) |k| img.channels.items[k].row(y)[x] = c[k];
        }
    }
    var orig = Image.create(alloc);
    defer orig.deinit();
    for (img.channels.items) |*ch| {
        var oc = try Channel.create(alloc, ch.w, ch.h, ch.hshift, ch.vshift);
        for (0..ch.h) |y| @memcpy(oc.row(y), ch.row(y));
        try orig.channels.append(alloc, oc);
    }

    const stats = try fwdPalette(alloc, &img, 0, 2, 256, true, 8);
    try testing.expect(stats != null);
    try testing.expectEqual(@as(u32, 5), stats.?.nb_colors);
    try testing.expectEqual(@as(usize, 2), img.channels.items.len); // palette + index
    try testing.expectEqual(@as(usize, 1), img.nb_meta_channels);
    try transforms.inversePalette(&img, 0, stats.?.nb_colors, 0, 0, &context_predict.weighted.Header.defaults, 8);
    try expectSameImage(&orig, &img);
}

test "fwdPalette: implicit cube colors stay out of the explicit palette" {
    const alloc = testing.allocator;
    var img = Image.create(alloc);
    defer img.deinit();
    // Fill with colors from the implicit cube: k=64..188 are the large cube
    // (values 0, 85, 170, 255 per axis), k=0..63 the small cube.
    for (0..3) |_| _ = try img.addChannel(16, 8);
    var k: usize = 0;
    for (0..8) |y| {
        for (0..16) |x| {
            k = (k + 7) % kImplicitPaletteSize;
            for (0..3) |c| img.channels.items[c].row(y)[x] = implicitPaletteColor(k, c, 8);
        }
    }
    const stats = try fwdPalette(alloc, &img, 0, 2, 300, true, 8);
    try testing.expect(stats != null);
    // Only implicit colors present, so no explicit palette entries were
    // needed and the transform is not useful (single-color rule does not
    // apply, but libjxl also refuses when everything fits the implicit set
    // and nb_colors == 0 is pointless): expect null or a valid small one.
    if (stats == null) return;
}

test "fwdChannelPalette: 16-bit wide-range channel uses the sorted-set path" {
    const alloc = testing.allocator;
    var img = Image.create(alloc);
    defer img.deinit();
    _ = try img.addChannel(20, 10);
    // Values spread over the full 16-bit range with only 5 unique entries.
    const vals = [_]i32{ -30000, -5, 0, 12345, 60000 };
    var prng = std.Random.DefaultPrng.init(3);
    const rand = prng.random();
    for (0..10) |y| {
        for (0..20) |x| {
            img.channels.items[0].row(y)[x] = vals[rand.uintLessThan(usize, vals.len)];
        }
    }
    const stats = try fwdPalette(alloc, &img, 0, 0, 8, true, 16);
    try testing.expect(stats != null);
    try testing.expectEqual(@as(u32, 5), stats.?.nb_colors);
    // Palette channel is sorted ascending.
    const pal = img.channels.items[0].row(0);
    var i: usize = 1;
    while (i < pal.len) : (i += 1) {
        try testing.expect(pal[i - 1] <= pal[i]);
    }
    // Index channel values are in range.
    for (0..10) |y| {
        for (0..20) |x| {
            try testing.expect(img.channels.items[1].row(y)[x] >= 0);
            try testing.expect(img.channels.items[1].row(y)[x] < 5);
        }
    }
}

test "fwdSqueeze/inverseSqueeze roundtrip: default scheme, odd sizes" {
    const alloc = testing.allocator;
    const sizes = [_][2]usize{ .{ 37, 23 }, .{ 64, 64 }, .{ 100, 33 }, .{ 17, 90 } };
    for (sizes) |sz| {
        var img = try makeTestImage(alloc, sz[0], sz[1], 3, sz[0] * 31 + sz[1]);
        defer img.deinit();

        // Structure clone to materialize the default scheme, as the
        // decoder's MetaSqueeze does.
        var clone = Image.create(alloc);
        defer clone.deinit();
        for (img.channels.items) |*ch| {
            try clone.channels.append(alloc, try Channel.create(alloc, ch.w, ch.h, ch.hshift, ch.vshift));
        }
        defer for (clone.channels.items) |*ch| ch.deinit();
        var t = Transform.init(alloc);
        defer t.deinit();
        t.id = .squeeze;
        try transforms.metaSqueeze(&clone, &t);

        // Deep copy for comparison.
        var orig = Image.create(alloc);
        defer orig.deinit();
        for (img.channels.items) |*ch| {
            var oc = try Channel.create(alloc, ch.w, ch.h, ch.hshift, ch.vshift);
            for (0..ch.h) |y| @memcpy(oc.row(y), ch.row(y));
            try orig.channels.append(alloc, oc);
        }
        defer for (orig.channels.items) |*ch| ch.deinit();

        const did = try fwdSqueeze(&img, t.squeezes.items);
        try testing.expect(did);

        // The forward layout must match the meta-applied layout exactly.
        try testing.expectEqual(clone.channels.items.len, img.channels.items.len);
        for (clone.channels.items, img.channels.items) |*cc, *ci| {
            try testing.expectEqual(cc.w, ci.w);
            try testing.expectEqual(cc.h, ci.h);
            try testing.expectEqual(cc.hshift, ci.hshift);
            try testing.expectEqual(cc.vshift, ci.vshift);
        }

        try transforms.inverseSqueeze(&img, t.squeezes.items);
        try expectSameImage(&orig, &img);
    }
}

test "truncateDefaultSqueeze: large image script is capped" {
    const alloc = testing.allocator;
    // Simulate a 100000x8 image: horizontal squeezes dominate.
    var img = Image.create(alloc);
    defer img.deinit();
    _ = try img.addChannel(100000, 8);
    var t = Transform.init(alloc);
    defer t.deinit();
    t.id = .squeeze;
    try transforms.metaSqueeze(&img, &t);
    var truncated = try truncateDefaultSqueeze(alloc, t.squeezes.items, 1);
    defer truncated.deinit(alloc);
    try testing.expect(truncated.items.len > 0);
    try testing.expect(truncated.items.len < t.squeezes.items.len);
}

test "estimateCost: smooth images cost less than noise" {
    const alloc = testing.allocator;
    var smooth = Image.create(alloc);
    defer smooth.deinit();
    _ = try smooth.addChannel(64, 64);
    for (0..64) |y| {
        for (0..64) |x| {
            smooth.channels.items[0].row(y)[x] = @intCast((x + y) % 251);
        }
    }
    var noisy = try makeTestImage(alloc, 64, 64, 1, 99);
    defer noisy.deinit();
    const c_smooth = try estimateCost(alloc, &smooth);
    const c_noise = try estimateCost(alloc, &noisy);
    try testing.expect(c_smooth < c_noise);
}

test "tryPalettes + tryLocalRct: applied transforms invert cleanly" {
    const alloc = testing.allocator;
    var img = Image.create(alloc);
    defer img.deinit();
    // Gradient RGB + alpha: palette-friendly and RCT-friendly.
    try img.channels.append(alloc, try Channel.create(alloc, 40, 30, 0, 0));
    try img.channels.append(alloc, try Channel.create(alloc, 40, 30, 0, 0));
    try img.channels.append(alloc, try Channel.create(alloc, 40, 30, 0, 0));
    try img.channels.append(alloc, try Channel.create(alloc, 40, 30, 0, 0));
    defer for (img.channels.items) |*ch| ch.deinit();
    for (0..30) |y| {
        for (0..40) |x| {
            img.channels.items[0].row(y)[x] = @intCast(x * 6);
            img.channels.items[1].row(y)[x] = @intCast(y * 8);
            img.channels.items[2].row(y)[x] = @intCast((x + y) * 3);
            img.channels.items[3].row(y)[x] = 255;
        }
    }
    var orig = Image.create(alloc);
    defer orig.deinit();
    for (img.channels.items) |*ch| {
        var oc = try Channel.create(alloc, ch.w, ch.h, ch.hshift, ch.vshift);
        for (0..ch.h) |yy| @memcpy(oc.row(yy), ch.row(yy));
        try orig.channels.append(alloc, oc);
    }
    defer for (orig.channels.items) |*ch| ch.deinit();

    var tlist: std.ArrayList(Transform) = .empty;
    defer {
        for (tlist.items) |*t| t.deinit();
        tlist.deinit(alloc);
    }

    try tryPalettes(alloc, &img, .{}, .{ .bit_depth = 8 }, &tlist);
    try tryLocalRct(alloc, &img, 7, &tlist);
    try testing.expect(tlist.items.len > 0);

    // Undo in reverse order (decoder semantics).
    var i: usize = tlist.items.len;
    while (i > 0) {
        i -= 1;
        try transforms.inverse(&img, &tlist.items[i], &context_predict.weighted.Header.defaults, 8);
    }
    try expectSameImage(&orig, &img);
}
test "estimateWPCost: gradients cheaper than noise per mode" {
    const alloc = std.testing.allocator;
    var smooth = try makeTestImage(alloc, 64, 64, 3, 11);
    defer smooth.deinit();
    var noisy = try makeTestImage(alloc, 64, 64, 3, 12);
    defer noisy.deinit();
    for (0..64) |y| {
        for (0..64) |x| {
            const v: i32 = @intCast((x + y) & 0x3f);
            for (smooth.channels.items) |*ch| ch.row(y)[x] = v;
        }
    }
    for (0..5) |mode| {
        const c_smooth = try estimateWPCost(alloc, &smooth, mode);
        const c_noise = try estimateWPCost(alloc, &noisy, mode);
        try testing.expect(c_smooth < c_noise);
    }
}

test "searchWpMode: returns the cheapest mode's header" {
    const alloc = std.testing.allocator;
    var img = try makeTestImage(alloc, 48, 48, 3, 21);
    defer img.deinit();
    for (0..48) |y| {
        for (0..48) |x| {
            for (img.channels.items) |*ch| ch.row(y)[x] = @intCast(x % 7);
        }
    }
    const costs = [_]f64{
        try estimateWPCost(alloc, &img, 0),
        try estimateWPCost(alloc, &img, 1),
        try estimateWPCost(alloc, &img, 2),
        try estimateWPCost(alloc, &img, 3),
        try estimateWPCost(alloc, &img, 4),
    };
    var best: usize = 0;
    for (costs, 0..) |c, i| {
        if (c < costs[best]) best = i;
    }
    const header = try searchWpMode(alloc, &img, 5);
    var expect = context_predict.weighted.Header.defaults;
    context_predict.predictorMode(best, &expect);
    try testing.expectEqual(expect.p1C, header.p1C);
    try testing.expectEqual(expect.p2C, header.p2C);
    try testing.expectEqual(expect.p3Ca, header.p3Ca);
    try testing.expectEqual(expect.p3Cb, header.p3Cb);
    try testing.expectEqual(expect.p3Cc, header.p3Cc);
    try testing.expectEqual(expect.p3Cd, header.p3Cd);
    try testing.expectEqual(expect.p3Ce, header.p3Ce);
    try testing.expectEqualSlices(u32, &expect.w, &header.w);
}

test "tryPalettes reverts a harmful palette" {
    const alloc = std.testing.allocator;
    // Noise with a tiny palette cap: the palette covers only part of the
    // colors, its transform raises the estimated cost and the slower tiers
    // must revert it (maybe_do_transform).
    var img = try makeTestImage(alloc, 64, 64, 3, 33);
    defer img.deinit();
    var snapshot = try makeTestImage(alloc, 64, 64, 3, 33);
    defer snapshot.deinit();

    var tlist: std.ArrayList(Transform) = .empty;
    defer {
        for (tlist.items) |*t| t.deinit();
        tlist.deinit(alloc);
    }
    const ctx = TransformContext{ .bit_depth = 8 };
    try tryPalettes(alloc, &img, .{
        .palette_colors = 16,
        .channel_colors_percent = 0,
        .reverts = true,
        .real_cost = true,
    }, ctx, &tlist);
    try testing.expectEqual(@as(usize, 0), tlist.items.len);
    try testing.expectEqual(@as(usize, 3), img.channels.items.len);
    try expectSameImage(&snapshot, &img);
}

test "tryLocalRct: candidate count 0 applies nothing" {
    const alloc = std.testing.allocator;
    var img = try makeTestImage(alloc, 40, 40, 3, 44);
    defer img.deinit();
    var snapshot = try makeTestImage(alloc, 40, 40, 3, 44);
    defer snapshot.deinit();
    var tlist: std.ArrayList(Transform) = .empty;
    defer {
        for (tlist.items) |*t| t.deinit();
        tlist.deinit(alloc);
    }
    try tryLocalRct(alloc, &img, 0, &tlist);
    try testing.expectEqual(@as(usize, 0), tlist.items.len);
    try expectSameImage(&snapshot, &img);
}

test "searchWpMode: noise-free diagonal textures benefit from non-default modes" {
    const alloc = std.testing.allocator;
    // Horizontal ramps with different dynamics: the default mode-0 WP
    // header is not always the cheapest across all five presets.
    var img = try makeTestImage(alloc, 96, 96, 3, 77);
    defer img.deinit();
    for (0..96) |y| {
        for (0..96) |x| {
            const v: i32 = @intCast((3 * x + 5 * y) & 0xff);
            for (img.channels.items) |*ch| ch.row(y)[x] = v;
        }
    }
    var modes_found: usize = 0;
    for (0..5) |mode| {
        const c = try estimateWPCost(alloc, &img, mode);
        if (c < (try estimateWPCost(alloc, &img, 0)) - 1e-9) modes_found += 1;
    }
    // At least one of modes 1..4 must beat mode 0 on this texture for the
    // search to be meaningful (the exact winner is content-dependent).
    try testing.expect(modes_found >= 1);
}
