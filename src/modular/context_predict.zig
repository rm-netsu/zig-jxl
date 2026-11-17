//! Порт `lib/jxl/modular/encoding/context_predict.h` — контекстное
//! предсказание модульного режима JPEG XL:
//!  * namespace weighted: Header (WeightedPredictorHeader) и State
//!    (самокорректирующийся взвешенный предиктор);
//!  * FlatDecisionNode / MATreeLookup — плоское дерево с обходом по два узла;
//!  * ClampedGradient, Select;
//!  * PrecomputeReferences — свойства предыдущих каналов;
//!  * detail.Predict с comptime-режимами (kUseTree/kUseWP/...) и обёртки.

const std = @import("std");
const common = @import("common.zig");
const options = @import("options.zig");
const bits = @import("../base/bits.zig");
const image_mod = @import("image.zig");
const BitReader = @import("../base/bit_io.zig").BitReader;
const BitWriter = @import("../base/bit_io.zig").BitWriter;

pub const pixel_type = common.pixel_type;
pub const pixel_type_w = common.pixel_type_w;
pub const PropertyVal = options.PropertyVal;
pub const Properties = options.Properties;
pub const Predictor = options.Predictor;
pub const Channel = image_mod.Channel;
pub const Image = image_mod.Image;

pub const kNumStaticProperties = options.kNumStaticProperties;
pub const kExtraPropsPerChannel = options.kExtraPropsPerChannel;
pub const kNumNonrefProperties = options.kNumNonrefProperties;
pub const kWPProp = options.kWPProp;
pub const kGradientProp = options.kGradientProp;

// ===========================================================================
// namespace weighted
// ===========================================================================

pub const weighted = struct {
    pub const kNumPredictors = 4;
    pub const kPredExtraBits = 3;
    pub const kPredictionRound: i64 = ((1 << kPredExtraBits) >> 1) - 1;
    pub const kNumProperties = 1;

    /// Порт `weighted::Header` (WeightedPredictorHeader).
    /// Битстрим (Bundle): 1 бит all_default; иначе 5,5,7,7,7,0,0 бит для
    /// p1C..p3Ce и 4x4 бит для w[0..3].
    pub const Header = struct {
        all_default: bool = true,
        p1C: pixel_type = 16,
        p2C: pixel_type = 10,
        p3Ca: pixel_type = 7,
        p3Cb: pixel_type = 7,
        p3Cc: pixel_type = 7,
        p3Cd: pixel_type = 0,
        p3Ce: pixel_type = 0,
        w: [kNumPredictors]u32 = .{ 0xd, 0xc, 0xc, 0xc },

        pub const defaults = Header{};

        /// Порт Bundle::Read + VisitFields.
        pub fn read(br: *BitReader) !Header {
            var h = Header{};
            // visitor->AllDefault: 1 бит; 1 = все поля по умолчанию.
            h.all_default = (br.readBits(1)) != 0;
            if (h.all_default) {
                return defaults;
            }
            // Все семь полей — Bits(5) (visit_p в libjxl фиксирует 5 бит).
            h.p1C = try visit_p(br, 16, 5);
            h.p2C = try visit_p(br, 10, 5);
            h.p3Ca = try visit_p(br, 7, 5);
            h.p3Cb = try visit_p(br, 7, 5);
            h.p3Cc = try visit_p(br, 7, 5);
            h.p3Cd = try visit_p(br, 0, 5);
            h.p3Ce = try visit_p(br, 0, 5);
            h.w[0] = @intCast(br.readBits(4));
            h.w[1] = @intCast(br.readBits(4));
            h.w[2] = @intCast(br.readBits(4));
            h.w[3] = @intCast(br.readBits(4));
            h.all_default = false;
            return h;
        }

        /// Порт Bundle::Write + VisitFields.
        pub fn write(
            self: *const Header,
            gpa: std.mem.Allocator,
            w: *BitWriter,
        ) !void {
            _ = gpa;
            const is_default = std.meta.eql(self.*, defaults);
            // Бит all_default: 1 = все поля по умолчанию.
            w.write(1, @intFromBool(is_default));
            if (is_default) return;
            w.write(5, @intCast(self.p1C));
            w.write(5, @intCast(self.p2C));
            w.write(5, @intCast(self.p3Ca));
            w.write(5, @intCast(self.p3Cb));
            w.write(5, @intCast(self.p3Cc));
            w.write(5, @intCast(self.p3Cd));
            w.write(5, @intCast(self.p3Ce));
            w.write(4, self.w[0]);
            w.write(4, self.w[1]);
            w.write(4, self.w[2]);
            w.write(4, self.w[3]);
        }

        fn visit_p(
            br: *BitReader,
            default_val: pixel_type,
            comptime nbits: usize,
        ) !pixel_type {
            const up: u64 = br.readBits(nbits);
            const v: pixel_type = @intCast(up);
            // При nbits == 0 значение обязано совпадать с дефолтом.
            if (nbits == 0) return default_val;
            return v;
        }
    };

    /// Порт `weighted::State`.
    pub const State = struct {
        prediction: [kNumPredictors]pixel_type_w = .{ 0, 0, 0, 0 },
        /// До удаления добавленных бит (kPredExtraBits).
        pred: pixel_type_w = 0,
        pred_errors: [kNumPredictors][]u32 = undefined,
        @"error": []i32 = &.{},
        header: *const Header,

        // Приближение деления на числа 1..64: divlookup[i] = (1 << 24) / (i + 1).
        const divlookup = [64]u32{
            16777216, 8388608, 5592405, 4194304, 3355443, 2796202, 2396745, 2097152,
            1864135,  1677721, 1525201, 1398101, 1290555, 1198372, 1118481, 1048576,
            986895,   932067,  883011,  838860,  798915,  762600,  729444,  699050,
            671088,   645277,  621378,  599186,  578524,  559240,  541200,  524288,
            508400,   493447,  479349,  466033,  453438,  441505,  430185,  419430,
            409200,   399457,  390167,  381300,  372827,  364722,  356962,  349525,
            342392,   335544,  328965,  322638,  316551,  310689,  305040,  299593,
            294337,   289262,  284359,  279620,  275036,  270600,  266305,  262144,
        };

        pub fn create(
            gpa: std.mem.Allocator,
            header: *const Header,
            xsize: usize,
            ysize: usize,
        ) !State {
            _ = ysize;
            var st = State{ .header = header };
            // Дополнительный отступ, чтобы избежать выхода за границы;
            // место на две строки данных.
            for (&st.pred_errors) |*pe| {
                pe.* = try gpa.alloc(u32, (xsize + 2) * 2);
                @memset(pe.*, 0);
            }
            st.@"error" = try gpa.alloc(i32, (xsize + 2) * 2);
            @memset(st.@"error", 0);
            return st;
        }

        pub fn deinit(self: *State, gpa: std.mem.Allocator) void {
            for (&self.pred_errors) |pe| gpa.free(pe);
            if (self.@"error".len > 0) gpa.free(self.@"error");
        }

        inline fn addBits(x: pixel_type_w) pixel_type_w {
            // C++: static_cast<uint64_t>(x) << kPredExtraBits (старшие биты
            // теряются намеренно) — в Zig эквивалентно умножению с заворачиванием.
            return x *% (1 << kPredExtraBits);
        }

        /// Порт `ErrorWeight`: приближение 4 + (maxweight << 24) / (x + 1).
        inline fn errorWeight(self: *const State, x: u64, maxweight: u32) u32 {
            _ = self;
            var shift: i32 = @intCast(bits.floorLog2Nonzero(x +% 1));
            shift -= 5;
            if (shift < 0) shift = 0;
            const shx: u6 = @intCast(@min(shift, 63));
            const shd: u5 = @intCast(@min(shift, 31));
            const di = @min(x >> shx, 63);
            return 4 +% ((maxweight *% divlookup[di]) >> shd);
        }

        /// Порт `WeightedAverage`: взвешенное среднее без деления.
        /// Веса должны в сумме давать не менее 16.
        inline fn weightedAverage(
            self: *const State,
            p: *const [kNumPredictors]pixel_type_w,
            w_in: [kNumPredictors]u32,
        ) pixel_type_w {
            _ = self;
            var w = w_in;
            var weight_sum: u32 = 0;
            for (w) |wi| weight_sum +%= wi;
            std.debug.assert(weight_sum > 15);
            const log_weight: u5 = @intCast(bits.floorLog2Nonzero(weight_sum)); // минимум 4
            weight_sum = 0;
            for (&w) |*wi| {
                wi.* >>= log_weight - 4;
                weight_sum +%= wi.*;
            }
            // Для округления.
            var sum: pixel_type_w = @as(pixel_type_w, weight_sum >> 1) - 1;
            for (w, 0..) |wi, i| {
                sum += p[i] * @as(pixel_type_w, wi);
            }
            return (sum *% @as(pixel_type_w, divlookup[weight_sum - 1])) >> 24;
        }

        /// Порт `weighted::State::Predict<compute_properties>`.
        pub fn predict(
            self: *State,
            comptime compute_properties: bool,
            x: usize,
            y: usize,
            xsize: usize,
            N_in: pixel_type_w,
            W_in: pixel_type_w,
            NE_in: pixel_type_w,
            NW_in: pixel_type_w,
            NN_in: pixel_type_w,
            properties: ?Properties,
            offset_in: usize,
        ) pixel_type_w {
            const cur_row = if (y & 1 != 0) 0 else xsize + 2;
            const prev_row = if (y & 1 != 0) xsize + 2 else 0;
            const pos_N = prev_row + x;
            const pos_NE = if (x < xsize - 1) pos_N + 1 else pos_N;
            const pos_NW = if (x > 0) pos_N - 1 else pos_N;
            var weights: [kNumPredictors]u32 = undefined;
            for (0..kNumPredictors) |i| {
                // pred_errors[pos_N] содержит и ошибку пикселя W;
                // pred_errors[pos_NW] — и ошибку пикселя WW.
                weights[i] = self.pred_errors[i][pos_N] +% self.pred_errors[i][pos_NE] +% self.pred_errors[i][pos_NW];
                weights[i] = self.errorWeight(weights[i], self.header.w[i]);
            }

            const N = addBits(N_in);
            const W = addBits(W_in);
            const NE = addBits(NE_in);
            const NW = addBits(NW_in);
            const NN = addBits(NN_in);

            const teW: pixel_type_w = if (x == 0) 0 else self.@"error"[cur_row + x - 1];
            const teN: pixel_type_w = self.@"error"[pos_N];
            const teNW: pixel_type_w = self.@"error"[pos_NW];
            const sumWN: pixel_type_w = teN +% teW;
            const teNE: pixel_type_w = self.@"error"[pos_NE];

            var offset = offset_in;
            if (compute_properties) {
                var p = teW;
                if (@abs(teN) > @abs(p)) p = teN;
                if (@abs(teNW) > @abs(p)) p = teNW;
                if (@abs(teNE) > @abs(p)) p = teNE;
                properties.?[offset] = @truncate(p);
                offset += 1;
            }

            self.prediction[0] = W +% NE -% N;
            self.prediction[1] = N -% (((sumWN +% teNE) *% @as(pixel_type_w, self.header.p1C)) >> 5);
            self.prediction[2] = W -% (((sumWN +% teNW) *% @as(pixel_type_w, self.header.p2C)) >> 5);
            self.prediction[3] =
                N -% ((teNW *% @as(pixel_type_w, self.header.p3Ca) +% teN *% @as(pixel_type_w, self.header.p3Cb) +%
                    teNE *% @as(pixel_type_w, self.header.p3Cc) +% (NN -% N) *% @as(pixel_type_w, self.header.p3Cd) +%
                    (NW -% W) *% @as(pixel_type_w, self.header.p3Ce)) >> 5);

            self.pred = self.weightedAverage(&self.prediction, weights);

            // Если у всех трёх одинаковый знак — клэмпинг не нужен.
            if (((teN ^ teW) | (teN ^ teNW)) > 0) {
                return (self.pred +% kPredictionRound) >> kPredExtraBits;
            }

            // Иначе клэмп к min/max соседних пикселей (W, NE, N).
            const mx = @max(W, @max(NE, N));
            const mn = @min(W, @min(NE, N));
            self.pred = @max(mn, @min(mx, self.pred));
            return (self.pred +% kPredictionRound) >> kPredExtraBits;
        }

        /// Порт `weighted::State::UpdateErrors`.
        pub fn updateErrors(
            self: *State,
            val_in: pixel_type_w,
            x: usize,
            y: usize,
            xsize: usize,
        ) void {
            const cur_row = if (y & 1 != 0) 0 else xsize + 2;
            const prev_row = if (y & 1 != 0) xsize + 2 else 0;
            const val = addBits(val_in);
            self.@"error"[cur_row + x] = @truncate(self.pred -% val);
            for (0..kNumPredictors) |i| {
                const err: u32 = @truncate(
                    (@abs(self.prediction[i] -% val) +% kPredictionRound) >> kPredExtraBits,
                );
                // Для предсказания следующей строки.
                self.pred_errors[i][cur_row + x] = err;
                // Прибавить ошибку этого пикселя к ошибке NE-пикселя: эффект —
                // добавление ошибки этого пикселя к пикселям E и EE.
                self.pred_errors[i][prev_row + x + 1] +%= err;
            }
        }
    };
};

/// Порт `PredictorMode`: пресеты параметров взвешенного предиктора.
pub fn predictorMode(i: usize, header: *weighted.Header) void {
    header.all_default = false;
    switch (i) {
        0 => {
            // ~ lossless16
            header.w = .{ 0xd, 0xc, 0xc, 0xc };
            header.p1C = 16;
            header.p2C = 10;
            header.p3Ca = 7;
            header.p3Cb = 7;
            header.p3Cc = 7;
            header.p3Cd = 0;
            header.p3Ce = 0;
        },
        1 => {
            // ~ default lossless8
            header.w = .{ 0xd, 0xc, 0xc, 0xb };
            header.p1C = 8;
            header.p2C = 8;
            header.p3Ca = 4;
            header.p3Cb = 0;
            header.p3Cc = 3;
            header.p3Cd = 23;
            header.p3Ce = 2;
        },
        2 => {
            // ~ west lossless8
            header.w = .{ 0xd, 0xc, 0xd, 0xc };
            header.p1C = 10;
            header.p2C = 9;
            header.p3Ca = 7;
            header.p3Cb = 0;
            header.p3Cc = 0;
            header.p3Cd = 16;
            header.p3Ce = 9;
        },
        3 => {
            // ~ north lossless8
            header.w = .{ 0xd, 0xd, 0xc, 0xc };
            header.p1C = 16;
            header.p2C = 8;
            header.p3Ca = 0;
            header.p3Cb = 16;
            header.p3Cc = 0;
            header.p3Cd = 23;
            header.p3Ce = 0;
        },
        else => {
            // something else, because why not
            header.w = .{ 0xd, 0xc, 0xc, 0xc };
            header.p1C = 10;
            header.p2C = 10;
            header.p3Ca = 5;
            header.p3Cb = 5;
            header.p3Cc = 5;
            header.p3Cd = 12;
            header.p3Ce = 4;
        },
    }
}

/// Порт `PredictorHasWeighted`.
pub fn predictorHasWeighted(predictor: Predictor) bool {
    return switch (predictor) {
        .zero, .left, .top, .average0, .select, .gradient => false,
        .weighted => true,
        .top_right, .top_left, .left_left => false,
        .average1, .average2, .average3, .average4 => false,
        .best, .variable => true,
        _ => false,
    };
}

// ===========================================================================
// Плоское дерево
// ===========================================================================

/// Порт `FlatDecisionNode`: хранит узел и его два дочерних узла одновременно
/// (два уровня дерева за одну проверку), что заметно сокращает число ветвей.
pub const FlatDecisionNode = struct {
    /// Свойство + splitval верхнего узла.
    property0: i32 = -1, // -1 для листа
    predictor: Predictor = .zero,
    splitval0: PropertyVal = 0,
    /// Свойство+splitval двух дочерних узлов (для листа: multiplier).
    splitvals: [2]PropertyVal = .{ 0, 0 },
    multiplier: u32 = 1,
    /// childID — индекс потомка; для листа — id контекста.
    childID: u32 = 0,
    properties: [2]i16 = .{ 0, 0 },
    predictor_offset: i32 = 0,
};

pub const FlatTree = std.ArrayList(FlatDecisionNode);

/// Порт `MATreeLookup`.
pub const MATreeLookup = struct {
    nodes: []const FlatDecisionNode,

    pub fn init(tree: []const FlatDecisionNode) MATreeLookup {
        return .{ .nodes = tree };
    }

    pub const LookupResult = struct {
        context: u32,
        predictor: Predictor,
        offset: i32,
        multiplier: u32,
    };

    /// Порт `Lookup` с развёрткой TRAVERSE_THE_TREE: за итерацию
    /// обрабатываются два уровня дерева.
    pub fn lookup(
        self: *const MATreeLookup,
        properties: Properties,
    ) LookupResult {
        var pos: u32 = 0;
        while (true) {
            // Первый узел.
            {
                const node = self.nodes[pos];
                if (node.property0 < 0) {
                    return .{
                        .context = node.childID,
                        .predictor = node.predictor,
                        .offset = node.predictor_offset,
                        .multiplier = node.multiplier,
                    };
                }
                const p0 = properties[@intCast(node.property0)] <= node.splitval0;
                const off0: u32 = @intFromBool(
                    properties[@intCast(node.properties[0])] <= node.splitvals[0],
                );
                const off1: u32 = 2 | @as(
                    u32,
                    @intFromBool(
                        properties[@intCast(node.properties[1])] <= node.splitvals[1],
                    ),
                );
                pos = node.childID + (if (p0) off1 else off0);
            }
            // Второй узел.
            {
                const node = self.nodes[pos];
                if (node.property0 < 0) {
                    return .{
                        .context = node.childID,
                        .predictor = node.predictor,
                        .offset = node.predictor_offset,
                        .multiplier = node.multiplier,
                    };
                }
                const p0 = properties[@intCast(node.property0)] <= node.splitval0;
                const off0: u32 = @intFromBool(
                    properties[@intCast(node.properties[0])] <= node.splitvals[0],
                );
                const off1: u32 = 2 | @as(
                    u32,
                    @intFromBool(
                        properties[@intCast(node.properties[1])] <= node.splitvals[1],
                    ),
                );
                pos = node.childID + (if (p0) off1 else off0);
            }
        }
    }
};

// ===========================================================================
// Свойства
// ===========================================================================

/// Порт `ClampedGradient`: клэмп градиента к min/max(n, w) (l учитывается
/// неявно). Промежуточная арифметика — uint32 с заворачиванием, как в C++.
pub inline fn clampedGradient(n: i32, w: i32, l: i32) i32 {
    const m = @min(n, w);
    const M = @max(n, w);
    // grad = n + w - l (с заворачиванием; результат в [m, M] не переполняется).
    const grad: i32 = @bitCast(
        @as(u32, @bitCast(n)) +% @as(u32, @bitCast(w)) -% @as(u32, @bitCast(l)),
    );
    const grad_clamp_M: i32 = if (l < m) M else grad;
    return if (l > M) m else grad_clamp_M;
}

/// Порт `Select`.
pub inline fn select(
    a: pixel_type_w,
    b: pixel_type_w,
    c: pixel_type_w,
) pixel_type_w {
    const p = a + b - c;
    const pa = @abs(p - a);
    const pb = @abs(p - b);
    return if (pa < pb) a else b;
}

/// Порт `PrecomputeReferences`: заполняет свойства из предыдущих каналов
/// (|v|, v, |v-pred|, v-pred для каждого подходящего канала).
pub fn precomputeReferences(
    ch: *const Channel,
    y: usize,
    img: *const Image,
    i: usize,
    references: *Channel,
) void {
    image_mod.zeroFillImage(&references.plane);
    var offset: usize = 0;
    const num_extra_props = references.w;
    const ci = &img.channels.items[i];
    var j: i64 = @as(i64, @intCast(i)) - 1;
    while (j >= 0 and offset < num_extra_props) : (j -= 1) {
        const cj = &img.channels.items[@intCast(j)];
        if (cj.w != ci.w or cj.h != ci.h) continue;
        if (cj.hshift != ci.hshift) continue;
        if (cj.vshift != ci.vshift) continue;
        const rpp = cj.row(y);
        const rpprev = cj.row(if (y != 0) y - 1 else 0);
        for (0..ch.w) |x| {
            const v: pixel_type_w = rpp[x];
            const base = references.plane.row(x);
            base[offset + 0] = @intCast(@abs(v));
            base[offset + 1] = @intCast(v);
            const vleft: pixel_type_w = if (x != 0) rpp[x - 1] else 0;
            const vtop: pixel_type_w = if (y != 0) rpprev[x] else vleft;
            const vtopleft: pixel_type_w = if (x != 0 and y != 0)
                rpprev[x - 1]
            else
                vleft;
            const vpredicted = clampedGradient(
                @intCast(vleft),
                @intCast(vtop),
                @intCast(vtopleft),
            );
            base[offset + 2] = @intCast(@abs(v - vpredicted));
            base[offset + 3] = @intCast(v - vpredicted);
        }
        offset += kExtraPropsPerChannel;
    }
}

/// Порт `PredictionResult`.
pub const PredictionResult = struct {
    context: i32 = 0,
    guess: pixel_type_w = 0,
    predictor: Predictor = .zero,
    multiplier: i32 = 1,
};

/// Порт `InitPropsRow`.
pub fn initPropsRow(
    p: Properties,
    static_props: *const [kNumStaticProperties]pixel_type,
    y: i32,
) void {
    for (0..kNumStaticProperties) |i| {
        p[i] = static_props[i];
    }
    p[2] = y;
    p[9] = 0; // локальный градиент
}

// ===========================================================================
// detail::Predict
// ===========================================================================

pub const detail = struct {
    pub const kUseTree: u32 = 1;
    pub const kUseWP: u32 = 2;
    pub const kForceComputeProperties: u32 = 4;
    pub const kAllPredictions: u32 = 8;
    pub const kNoEdgeCases: u32 = 16;

    /// Порт `PredictOne`.
    pub inline fn predictOne(
        p: Predictor,
        left: pixel_type_w,
        top: pixel_type_w,
        toptop: pixel_type_w,
        topleft: pixel_type_w,
        topright: pixel_type_w,
        leftleft: pixel_type_w,
        toprightright: pixel_type_w,
        wp_pred: pixel_type_w,
    ) pixel_type_w {
        switch (p) {
            .zero => return 0,
            .left => return left,
            .top => return top,
            .select => return select(left, top, topleft),
            .weighted => return wp_pred,
            .gradient => return clampedGradient(
                @truncate(left),
                @truncate(top),
                @truncate(topleft),
            ),
            .top_left => return topleft,
            .top_right => return topright,
            .left_left => return leftleft,
            .average0 => return @divTrunc(left + top, 2),
            .average1 => return @divTrunc(left + topleft, 2),
            .average2 => return @divTrunc(topleft + top, 2),
            .average3 => return @divTrunc(top + topright, 2),
            .average4 => return @divTrunc(
                6 * top -
                    2 * toptop +
                    7 * left +
                    1 * leftleft +
                    1 * toprightright +
                    3 * topright +
                    8,
                16,
            ),
            else => return 0,
        }
    }

    /// Порт `detail::Predict<mode>`.
    /// Вместо указателя `pp` в строку передаются плоскость и координаты.
    pub fn predict(
        comptime mode: u32,
        p: ?Properties,
        w: usize,
        plane: *const image_mod.Plane(pixel_type),
        x: usize,
        y: usize,
        predictor_in: Predictor,
        lookup: ?*const MATreeLookup,
        references: ?*const Channel,
        wp_state: ?*weighted.State,
        predictions: ?[]pixel_type_w,
    ) PredictionResult {
        const compute_properties = (mode & kUseTree) != 0 or
            (mode & kForceComputeProperties) != 0;
        const nec = (mode & kNoEdgeCases) != 0;
        var offset: usize = 3; // 2 статических свойства + y

        const cur = plane.row(y);
        // Соседние строки: при nec контракт гарантирует y > 1, x > 1.
        const prev: []const pixel_type = if (!nec and y == 0) cur else plane.row(y -| 1);
        const prev2: []const pixel_type = if (!nec and y < 2) prev else plane.row(y -| 2);

        const left: pixel_type_w = if (nec or x != 0)
            cur[x - 1]
        else
            (if (y != 0) prev[x] else 0);
        const top: pixel_type_w = if (nec or y != 0) prev[x] else left;
        const topleft: pixel_type_w = if (nec or (x != 0 and y != 0))
            prev[x - 1]
        else
            left;
        const topright: pixel_type_w = if (nec or (x + 1 < w and y != 0))
            prev[x + 1]
        else
            top;
        const leftleft: pixel_type_w = if (nec or x > 1) cur[x - 2] else left;
        const toptop: pixel_type_w = if (nec or y > 1) prev2[x] else top;
        const toprightright: pixel_type_w = if (nec or (x + 2 < w and y != 0))
            prev[x + 2]
        else
            topright;

        if (compute_properties) {
            // Расположение
            p.?[offset] = @intCast(x);
            offset += 1;
            // Соседи (C++ кладёт pixel_type_w в pixel_type — усечение)
            p.?[offset] = @truncate(if (top > 0) top else -top);
            offset += 1;
            p.?[offset] = @truncate(if (left > 0) left else -left);
            offset += 1;
            p.?[offset] = @truncate(top);
            offset += 1;
            p.?[offset] = @truncate(left);
            offset += 1;
            // Локальный градиент (использует старое значение p[9]).
            p.?[offset] = @truncate(left -% p.?[offset + 1]);
            offset += 1;
            // Локальный градиент
            p.?[offset] = @truncate(left +% top -% topleft);
            offset += 1;
            // Свойства контекста FFV1
            p.?[offset] = @truncate(left -% topleft);
            offset += 1;
            p.?[offset] = @truncate(topleft -% top);
            offset += 1;
            p.?[offset] = @truncate(top -% topright);
            offset += 1;
            p.?[offset] = @truncate(top -% toptop);
            offset += 1;
            p.?[offset] = @truncate(left -% leftleft);
            offset += 1;
        }

        var wp_pred: pixel_type_w = 0;
        if ((mode & kUseWP) != 0) {
            wp_pred = wp_state.?.predict(
                compute_properties,
                x,
                y,
                w,
                top,
                left,
                topright,
                topleft,
                toptop,
                p,
                offset,
            );
        }
        if (!nec and compute_properties) {
            offset += 1; // weighted::kNumProperties
            // Дополнительные свойства из ссылочных каналов.
            const refs = references.?;
            if (refs.w > 0) {
                const rp = refs.plane.row(x);
                for (0..refs.w) |i| {
                    p.?[offset] = rp[i];
                    offset += 1;
                }
            }
        }
        var result = PredictionResult{};
        var predictor = predictor_in;
        if ((mode & kUseTree) != 0) {
            const lr = lookup.?.lookup(p.?);
            result.context = @intCast(lr.context);
            result.guess = lr.offset;
            result.multiplier = @intCast(lr.multiplier);
            predictor = lr.predictor;
        }
        if ((mode & kAllPredictions) != 0) {
            for (0..options.kNumModularPredictors) |i| {
                predictions.?[i] = predictOne(
                    @enumFromInt(@as(u8, @intCast(i))),
                    left,
                    top,
                    toptop,
                    topleft,
                    topright,
                    leftleft,
                    toprightright,
                    wp_pred,
                );
            }
        }
        result.guess += predictOne(
            predictor,
            left,
            top,
            toptop,
            topleft,
            topright,
            leftleft,
            toprightright,
            wp_pred,
        );
        result.predictor = predictor;

        return result;
    }
};

// ===========================================================================
// Обёртки (интерфейсPredict* из context_predict.h)
// ===========================================================================

pub fn predictNoTreeNoWP(
    w: usize,
    plane: *const image_mod.Plane(pixel_type),
    x: usize,
    y: usize,
    predictor: Predictor,
) PredictionResult {
    return detail.predict(
        0,
        null,
        w,
        plane,
        x,
        y,
        predictor,
        null,
        null,
        null,
        null,
    );
}

pub fn predictNoTreeWP(
    w: usize,
    plane: *const image_mod.Plane(pixel_type),
    x: usize,
    y: usize,
    predictor: Predictor,
    wp_state: *weighted.State,
) PredictionResult {
    return detail.predict(
        detail.kUseWP,
        null,
        w,
        plane,
        x,
        y,
        predictor,
        null,
        null,
        wp_state,
        null,
    );
}

pub fn predictTreeNoWP(
    p: Properties,
    w: usize,
    plane: *const image_mod.Plane(pixel_type),
    x: usize,
    y: usize,
    tree_lookup: *const MATreeLookup,
    references: *const Channel,
) PredictionResult {
    return detail.predict(
        detail.kUseTree,
        p,
        w,
        plane,
        x,
        y,
        .zero,
        tree_lookup,
        references,
        null,
        null,
    );
}

/// Только для y > 1, x > 1, x < w-2 и пустых references.
pub fn predictTreeNoWPNEC(
    p: Properties,
    w: usize,
    plane: *const image_mod.Plane(pixel_type),
    x: usize,
    y: usize,
    tree_lookup: *const MATreeLookup,
    references: *const Channel,
) PredictionResult {
    return detail.predict(
        detail.kUseTree | detail.kNoEdgeCases,
        p,
        w,
        plane,
        x,
        y,
        .zero,
        tree_lookup,
        references,
        null,
        null,
    );
}

pub fn predictTreeWP(
    p: Properties,
    w: usize,
    plane: *const image_mod.Plane(pixel_type),
    x: usize,
    y: usize,
    tree_lookup: *const MATreeLookup,
    references: *const Channel,
    wp_state: *weighted.State,
) PredictionResult {
    return detail.predict(
        detail.kUseTree | detail.kUseWP,
        p,
        w,
        plane,
        x,
        y,
        .zero,
        tree_lookup,
        references,
        wp_state,
        null,
    );
}

pub fn predictTreeWPNEC(
    p: Properties,
    w: usize,
    plane: *const image_mod.Plane(pixel_type),
    x: usize,
    y: usize,
    tree_lookup: *const MATreeLookup,
    references: *const Channel,
    wp_state: *weighted.State,
) PredictionResult {
    return detail.predict(
        detail.kUseTree | detail.kUseWP | detail.kNoEdgeCases,
        p,
        w,
        plane,
        x,
        y,
        .zero,
        tree_lookup,
        references,
        wp_state,
        null,
    );
}

pub fn predictLearn(
    p: Properties,
    w: usize,
    plane: *const image_mod.Plane(pixel_type),
    x: usize,
    y: usize,
    predictor: Predictor,
    references: *const Channel,
    wp_state: *weighted.State,
) PredictionResult {
    return detail.predict(
        detail.kForceComputeProperties | detail.kUseWP,
        p,
        w,
        plane,
        x,
        y,
        predictor,
        null,
        references,
        wp_state,
        null,
    );
}

pub fn predictLearnAll(
    p: Properties,
    w: usize,
    plane: *const image_mod.Plane(pixel_type),
    x: usize,
    y: usize,
    references: *const Channel,
    wp_state: *weighted.State,
    predictions: []pixel_type_w,
) void {
    _ = detail.predict(
        detail.kForceComputeProperties | detail.kUseWP | detail.kAllPredictions,
        p,
        w,
        plane,
        x,
        y,
        .zero,
        null,
        references,
        wp_state,
        predictions,
    );
}

pub fn predictLearnNEC(
    p: Properties,
    w: usize,
    plane: *const image_mod.Plane(pixel_type),
    x: usize,
    y: usize,
    predictor: Predictor,
    references: *const Channel,
    wp_state: *weighted.State,
) PredictionResult {
    return detail.predict(
        detail.kForceComputeProperties | detail.kUseWP | detail.kNoEdgeCases,
        p,
        w,
        plane,
        x,
        y,
        predictor,
        null,
        references,
        wp_state,
        null,
    );
}

pub fn predictAllNoWP(
    w: usize,
    plane: *const image_mod.Plane(pixel_type),
    x: usize,
    y: usize,
    predictions: []pixel_type_w,
) void {
    _ = detail.predict(
        detail.kAllPredictions,
        null,
        w,
        plane,
        x,
        y,
        .zero,
        null,
        null,
        null,
        predictions,
    );
}

// ===========================================================================
// FilterTree (encoding.cc)
// ===========================================================================

/// Порт `FilterTree` (lib/jxl/modular/encoding/encoding.cc): обрезка ветвей
/// по статическим свойствам (channel/group), схлопывание по два уровня,
/// дублирование листьев-плейсхолдеров. Результат — FlatTree для MATreeLookup.
pub fn filterTree(
    gpa: std.mem.Allocator,
    global_tree: []const @import("ma.zig").PropertyDecisionNode,
    static_props: *const [kNumStaticProperties]pixel_type,
    num_props: *usize,
    use_wp: *bool,
    wp_only: *bool,
    gradient_only: *bool,
    output: *FlatTree,
) !void {
    num_props.* = 0;
    var has_wp = false;
    var has_non_wp = false;
    gradient_only.* = true;
    const Mark = struct {
        fn markProperty(
            p: i32,
            has_wp_: *bool,
            has_non_wp_: *bool,
            gradient_only_: *bool,
        ) void {
            if (p == kWPProp) {
                has_wp_.* = true;
            } else if (p >= kNumStaticProperties) {
                has_non_wp_.* = true;
            }
            if (p >= kNumStaticProperties and p != kGradientProp) {
                gradient_only_.* = false;
            }
        }
    };

    output.clearRetainingCapacity();
    var nodes: std.ArrayList(usize) = .empty;
    defer nodes.deinit(gpa);
    var head: usize = 0;
    try nodes.append(gpa, 0);
    while (head < nodes.items.len) {
        var cur = nodes.items[head];
        head += 1;
        // Пропускаем узлы, решаемые прямо сейчас, переходя к детям.
        while (global_tree[cur].property < kNumStaticProperties and
            global_tree[cur].property != -1)
        {
            if (static_props[@intCast(global_tree[cur].property)] > global_tree[cur].splitval) {
                cur = global_tree[cur].lchild;
            } else {
                cur = global_tree[cur].rchild;
            }
        }
        var flat = FlatDecisionNode{};
        if (global_tree[cur].property == -1) {
            flat.property0 = -1;
            flat.childID = global_tree[cur].lchild;
            flat.predictor = global_tree[cur].predictor;
            flat.predictor_offset = @truncate(global_tree[cur].predictor_offset);
            flat.multiplier = global_tree[cur].multiplier;
            gradient_only.* = gradient_only.* and flat.predictor == .gradient;
            has_wp = has_wp or flat.predictor == .weighted;
            has_non_wp = has_non_wp or flat.predictor != .weighted;
            try output.append(gpa, flat);
            continue;
        }
        flat.childID = @intCast(output.items.len + (nodes.items.len - head) + 1);

        flat.property0 = global_tree[cur].property;
        num_props.* = @max(@as(usize, @intCast(flat.property0 + 1)), num_props.*);
        flat.splitval0 = global_tree[cur].splitval;

        for (0..2) |i| {
            var cur_child = if (i == 0) global_tree[cur].lchild else global_tree[cur].rchild;
            // Пропускаем узлы, решаемые по статическим свойствам.
            while (global_tree[cur_child].property < kNumStaticProperties and
                global_tree[cur_child].property != -1)
            {
                if (static_props[@intCast(global_tree[cur_child].property)] > global_tree[cur_child].splitval) {
                    cur_child = global_tree[cur_child].lchild;
                } else {
                    cur_child = global_tree[cur_child].rchild;
                }
            }
            // Лист: добавляем плейсхолдер и две копии листа.
            if (global_tree[cur_child].property == -1) {
                flat.properties[i] = 0;
                flat.splitvals[i] = 0;
                try nodes.append(gpa, cur_child);
                try nodes.append(gpa, cur_child);
            } else {
                flat.properties[i] = global_tree[cur_child].property;
                flat.splitvals[i] = global_tree[cur_child].splitval;
                try nodes.append(gpa, global_tree[cur_child].lchild);
                try nodes.append(gpa, global_tree[cur_child].rchild);
                num_props.* = @max(@as(usize, @intCast(flat.properties[i] + 1)), num_props.*);
            }
        }

        Mark.markProperty(flat.properties[0], &has_wp, &has_non_wp, gradient_only);
        Mark.markProperty(flat.properties[1], &has_wp, &has_non_wp, gradient_only);
        Mark.markProperty(flat.property0, &has_wp, &has_non_wp, gradient_only);
        try output.append(gpa, flat);
    }
    if (num_props.* > kNumNonrefProperties) {
        num_props.* =
            common.divCeil(num_props.* - kNumNonrefProperties, kExtraPropsPerChannel) *
            kExtraPropsPerChannel + kNumNonrefProperties;
    } else {
        num_props.* = kNumNonrefProperties;
    }
    use_wp.* = has_wp;
    wp_only.* = has_wp and !has_non_wp;
}

// ---------------------------------------------------------------------------
// Тесты
// ---------------------------------------------------------------------------

const testing = std.testing;

test "clampedGradient: клэмп к диапазону" {
    // Обычный градиент.
    try testing.expectEqual(@as(i32, 10), clampedGradient(10, 10, 10));
    // l меньше min: результат M.
    try testing.expectEqual(@as(i32, 20), clampedGradient(10, 20, 0));
    // l больше max: результат m.
    try testing.expectEqual(@as(i32, 10), clampedGradient(10, 20, 100));
    // Градиент внутри диапазона — без изменений.
    try testing.expectEqual(@as(i32, 25), clampedGradient(20, 30, 25));
}

test "select: выбор более близкого" {
    // p = a + b - c; |p - a| < |p - b| -> a, иначе b (при равенстве — b).
    try testing.expectEqual(@as(pixel_type_w, 20), select(10, 20, 15));
    try testing.expectEqual(@as(pixel_type_w, 10), select(10, 20, 16));
}

test "predictOne: все базовые предикторы" {
    const left: pixel_type_w = 30;
    const top: pixel_type_w = 20;
    const toptop: pixel_type_w = 22;
    const topleft: pixel_type_w = 26;
    const topright: pixel_type_w = 24;
    const leftleft: pixel_type_w = 28;
    const toprightright: pixel_type_w = 25;
    try testing.expectEqual(@as(pixel_type_w, 0), detail.predictOne(.zero, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    try testing.expectEqual(left, detail.predictOne(.left, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    try testing.expectEqual(top, detail.predictOne(.top, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    try testing.expectEqual(topleft, detail.predictOne(.top_left, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    try testing.expectEqual(topright, detail.predictOne(.top_right, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    try testing.expectEqual(leftleft, detail.predictOne(.left_left, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    try testing.expectEqual(@divTrunc(left + top, 2), detail.predictOne(.average0, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    try testing.expectEqual(@divTrunc(left + topleft, 2), detail.predictOne(.average1, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    try testing.expectEqual(@divTrunc(topleft + top, 2), detail.predictOne(.average2, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    try testing.expectEqual(@divTrunc(top + topright, 2), detail.predictOne(.average3, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    const avg4 = @divTrunc(6 * top - 2 * toptop + 7 * left + leftleft + toprightright + 3 * topright + 8, 16);
    try testing.expectEqual(avg4, detail.predictOne(.average4, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    // Select: p = 30+20-26 = 24; |24-30| = 6 > |24-20| = 4 -> top (20).
    try testing.expectEqual(@as(pixel_type_w, 20), detail.predictOne(.select, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    // Gradient: n=top=20, w=left=30, l=topleft=26 -> grad=24, в [20,30] -> 24.
    try testing.expectEqual(@as(pixel_type_w, 24), detail.predictOne(.gradient, left, top, toptop, topleft, topright, leftleft, toprightright, 0));
    // Weighted -> wp_pred.
    try testing.expectEqual(@as(pixel_type_w, 99), detail.predictOne(.weighted, left, top, toptop, topleft, topright, leftleft, toprightright, 99));
}

test "weighted.Header: round trip битстрима" {
    const gpa = testing.allocator;
    // Дефолтный заголовок: 1 бит.
    {
        var w = BitWriter.init(gpa);
        defer w.deinit();
        const h = weighted.Header{};
        try h.write(gpa, &w);
        w.zeroPadToByte();
        const bytes = try w.takeBytes();
        defer gpa.free(bytes);
        var br = BitReader.init(bytes);
        const h2 = try weighted.Header.read(&br);
        try testing.expect(h2.all_default);
        try testing.expectEqual(@as(pixel_type, 16), h2.p1C);
        try testing.expectEqual(@as(u32, 0xd), h2.w[0]);
    }
    // Ненулевой заголовок (пресет 1).
    {
        var w = BitWriter.init(gpa);
        defer w.deinit();
        var h = weighted.Header{};
        predictorMode(1, &h);
        try h.write(gpa, &w);
        w.zeroPadToByte();
        const bytes = try w.takeBytes();
        defer gpa.free(bytes);
        var br = BitReader.init(bytes);
        const h2 = try weighted.Header.read(&br);
        try testing.expect(!h2.all_default);
        try testing.expectEqual(@as(pixel_type, 8), h2.p1C);
        try testing.expectEqual(@as(pixel_type, 2), h2.p3Ce);
        try testing.expectEqual(@as(u32, 0xb), h2.w[3]);
    }
}

test "weighted.State: детерминированное предсказание и ошибки" {
    const gpa = testing.allocator;
    const header = weighted.Header{};
    var st = try weighted.State.create(gpa, &header, 8, 4);
    defer st.deinit(gpa);

    // Первый пиксель: все соседи 0.
    var props: [options.kNumNonrefProperties]PropertyVal = .{0} ** options.kNumNonrefProperties;
    const p0 = st.predict(true, 0, 0, 8, 0, 0, 0, 0, 0, &props, 15);
    try testing.expectEqual(@as(pixel_type_w, 0), p0);
    st.updateErrors(5, 0, 0, 8);
    // После обновления ошибок предсказание меняется.
    const p1 = st.predict(false, 1, 0, 8, 0, 5, 0, 0, 0, null, 0);
    try testing.expect(p1 != 0);
}

test "MATreeLookup: обход плоского дерева" {
    // Реальная структура плоского дерева: childID указывает на первый из
    // четырёх слотов потомков (два уровня). Для дерева с одним расщеплением:
    // узел 0 — внутренний; потомки-листья дублируются (плейсхолдеры
    // properties=0, splitvals=0).
    // Слева (p0 > splitval0): слоты 1/2 — лист A; справа (p0 <= splitval0):
    // слоты 3/4 — лист B.
    const leafA = FlatDecisionNode{ .property0 = -1, .childID = 7, .predictor = .left, .predictor_offset = -3, .multiplier = 4 };
    const leafB = FlatDecisionNode{ .property0 = -1, .childID = 3, .predictor = .gradient, .predictor_offset = 5, .multiplier = 2 };
    const nodes = [_]FlatDecisionNode{
        .{ .property0 = 0, .splitval0 = 10, .splitvals = .{ 0, 0 }, .childID = 1, .properties = .{ 0, 0 } },
        leafA,
        leafA,
        leafB,
        leafB,
    };
    const tl = MATreeLookup.init(&nodes);
    var props: [kNumNonrefProperties]PropertyVal = .{0} ** kNumNonrefProperties;
    // p[0] = 5 <= 10 -> правая ветка (слоты 3/4), плейсхолдер p[0] <= 0 -> 0 -> слот 3.
    props[0] = 5;
    var r = tl.lookup(&props);
    try testing.expectEqual(@as(u32, 3), r.context);
    try testing.expectEqual(Predictor.gradient, r.predictor);
    try testing.expectEqual(@as(i32, 5), r.offset);
    try testing.expectEqual(@as(u32, 2), r.multiplier);
    // p[0] = 50 > 10 -> левая ветка (слоты 1/2), плейсхолдер p[0] <= 0 -> 0 -> слот 1.
    props[0] = 50;
    r = tl.lookup(&props);
    try testing.expectEqual(@as(u32, 7), r.context);
    try testing.expectEqual(Predictor.left, r.predictor);
    try testing.expectEqual(@as(i32, -3), r.offset);
    try testing.expectEqual(@as(u32, 4), r.multiplier);
}

test "precomputeReferences: свойства предыдущего канала" {
    const gpa = testing.allocator;
    var img = Image.create(gpa);
    defer img.deinit();
    _ = try img.addChannel(4, 2); // канал 0
    _ = try img.addChannel(4, 2); // канал 1
    img.channels.items[0].row(0)[0] = 10;
    img.channels.items[0].row(0)[1] = 20;
    img.channels.items[0].row(0)[2] = 30;
    img.channels.items[0].row(0)[3] = 40;
    img.channels.items[0].row(1)[0] = 15;

    var references = try Channel.create(gpa, 4 * kExtraPropsPerChannel, 4, 0, 0);
    defer references.deinit();

    const ch = &img.channels.items[1];
    precomputeReferences(ch, 0, &img, 1, &references);
    // Для x=0: v=10, vleft=0, vtop=vleft=0, vtopleft=0, pred = clampedGradient(0,0,0)=0.
    const base0 = references.plane.row(0);
    try testing.expectEqual(@as(pixel_type, 10), base0[0]); // |v|
    try testing.expectEqual(@as(pixel_type, 10), base0[1]); // v
    try testing.expectEqual(@as(pixel_type, 10), base0[2]); // |v - pred|
    try testing.expectEqual(@as(pixel_type, 10), base0[3]); // v - pred
}
