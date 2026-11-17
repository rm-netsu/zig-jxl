//! Порт `lib/jxl/modular/encoding/enc_ma.h` / `enc_ma.cc` — построение и
//! запись MA-деревьев (энкодер). SIMD (Highway) заменён скалярной
//! реализацией `estimateBits` с тем же приближением `FastLog2f`.

const std = @import("std");
const common = @import("common.zig");
const options = @import("options.zig");
const bits = @import("../base/bits.zig");
const image_mod = @import("image.zig");
const hybrid_uint = @import("hybrid_uint.zig");
const ma = @import("ma.zig");
const rng_mod = @import("rng.zig");
const context_predict = @import("context_predict.zig");

const Allocator = std.mem.Allocator;
const pixel_type = common.pixel_type;
const pixel_type_w = common.pixel_type_w;
const PropertyVal = options.PropertyVal;
const Properties = options.Properties;
const Predictor = options.Predictor;
const Tree = ma.Tree;
const PropertyDecisionNode = ma.PropertyDecisionNode;
const StaticPropRange = options.StaticPropRange;
const ModularMultiplierInfo = options.ModularMultiplierInfo;
const TreeMode = options.TreeMode;

pub const kNumTreeContexts = ma.kNumTreeContexts;
pub const kNumStaticProperties = options.kNumStaticProperties;
pub const kWPProp = options.kWPProp;
pub const kGradientProp = options.kGradientProp;
pub const kNumNonrefProperties = options.kNumNonrefProperties;

/// ANS_TAB_SIZE (ans_params.h).
pub const kANSLogTabSize = 12;
pub const kANSTabSize = 1 << kANSLogTabSize;

/// Порт `Token` (enc_ans.h).
pub const Token = struct {
    is_lz77_length: bool = false,
    context: u32,
    value: u32,
};

/// Порт `ResidualToken` (enc_ma.h).
pub const ResidualToken = struct {
    tok: u8,
    nbits: u8,
};

/// Приближение log2 рациональным полиномом (порт FastLog2f из
/// base/fast_math-inl.h, скалярная версия).
fn fastLog2f(x: f32) f32 {
    const p = [3]f32{
        -1.8503833400518310E-06,
        1.4287160470083755E+00,
        7.4245873327820566E-01,
    };
    const q = [3]f32{
        9.9032814277590719E-01,
        1.0096718572241148E+00,
        1.7409343003366853E-01,
    };

    const x_bits: i32 = @bitCast(x);
    // Приведение диапазона к [-1/3, 1/3].
    const exp_bits = x_bits -% 0x3f2aaaab; // = 2/3
    const exp_shifted = exp_bits >> 23;
    const mantissa: f32 = @bitCast(x_bits -% (exp_shifted << 23));
    const exp_val: f32 = @floatFromInt(exp_shifted);
    const m = mantissa - 1.0;
    var num = p[2] * m + p[1];
    num = num * m + p[0];
    var den = q[2] * m + q[1];
    den = den * m + q[0];
    return num / den + exp_val;
}

/// Порт `EstimateBits` (enc_ma.cc): энтропия гистограммы с учётом
/// минимальной вероятности для символов с ненулевыми счётчиками.
/// Скалярный эквивалент SIMD-версии (Padded(x) == x при одном "ланe").
pub fn estimateBits(counts: []const i32, num_symbols: usize) f32 {
    var total: i32 = 0;
    for (counts[0..num_symbols]) |c| total +%= c;

    const minprob: f32 = 1.0 / @as(f32, kANSTabSize);
    const inv_total: f32 = 1.0 / @as(f32, @floatFromInt(total));
    var bits_sum: f32 = 0;
    for (counts[0..num_symbols]) |c| {
        const counts_f: f32 = @floatFromInt(c);
        const probs = counts_f * inv_total;
        const mprobs = @max(probs, minprob);
        var nbps = fastLog2f(mprobs);
        // Если символ один (count == total) — стоимость нулевая.
        if (c == total) nbps = 0;
        bits_sum -= counts_f * nbps;
    }
    return bits_sum;
}

/// Порт `MakeSplitNode`. Заметьте: дерево расщепляет по «строго больше»;
/// lchild получает предиктор rpred (ветка "> splitval"), rchild — lpred
/// (ветка "<= splitval"); именование перекрёстное, но согласовано с
/// декодером и MATreeLookup.
pub fn makeSplitNode(
    tree: *Tree,
    gpa: Allocator,
    pos: usize,
    property: i32,
    splitval: i32,
    lpred: Predictor,
    loff: i64,
    rpred: Predictor,
    roff: i64,
) !void {
    tree.items[pos].lchild = @intCast(tree.items.len);
    tree.items[pos].rchild = @intCast(tree.items.len + 1);
    tree.items[pos].splitval = splitval;
    tree.items[pos].property = @intCast(property);
    // Первый добавленный узел (lchild) — ветка "> splitval" с rpred.
    try tree.append(gpa, .{});
    tree.items[tree.items.len - 1].property = -1;
    tree.items[tree.items.len - 1].predictor = rpred;
    tree.items[tree.items.len - 1].predictor_offset = roff;
    tree.items[tree.items.len - 1].multiplier = 1;
    // Второй узел (rchild) — ветка "<= splitval" с lpred.
    try tree.append(gpa, .{});
    tree.items[tree.items.len - 1].property = -1;
    tree.items[tree.items.len - 1].predictor = lpred;
    tree.items[tree.items.len - 1].predictor_offset = loff;
    tree.items[tree.items.len - 1].multiplier = 1;
}

pub const IntersectionType = enum { none, partial, inside };

/// Порт `BoxIntersects`.
pub fn boxIntersects(
    needle: StaticPropRange,
    haystack: StaticPropRange,
    partial_axis: *u32,
    partial_val: *u32,
) IntersectionType {
    var partial = false;
    for (0..kNumStaticProperties) |i| {
        if (haystack[i][0] >= needle[i][1]) {
            return .none;
        }
        if (haystack[i][1] <= needle[i][0]) {
            return .none;
        }
        if (haystack[i][0] <= needle[i][0] and haystack[i][1] >= needle[i][1]) {
            continue;
        }
        partial = true;
        partial_axis.* = @intCast(i);
        if (haystack[i][0] > needle[i][0] and haystack[i][0] < needle[i][1]) {
            partial_val.* = haystack[i][0] - 1;
        } else {
            partial_val.* = haystack[i][1] - 1;
        }
    }
    return if (partial) .partial else .inside;
}

/// Порт `TreeSamples` (enc_ma.h) — collects all data needed to build a tree.
pub const TreeSamples = struct {
    gpa: Allocator,
    /// Информация о вычетах: токен и число доп. бит, по предиктору.
    residuals: std.ArrayList(std.ArrayList(ResidualToken)) = .empty,
    /// Число вхождений каждого образца.
    sample_counts: std.ArrayList(u16) = .empty,
    num_static_props: usize = 0,
    /// Квантованные статические свойства.
    static_props: [kNumStaticProperties]std.ArrayList(u32) = .{ .empty, .empty },
    /// Значения свойств, квантованные до <= 256 различных значений.
    props: std.ArrayList(std.ArrayList(u8)) = .empty,
    /// Информация о деквантовании `props`.
    compact_properties: std.ArrayList(std.ArrayList(i32)) = .empty,
    /// Используемые свойства.
    props_to_use: std.ArrayList(u32) = .empty,
    /// Используемые предикторы.
    predictors: std.ArrayList(Predictor) = .empty,
    /// Отображение значения свойства -> квантованное значение.
    static_property_mapping: [kNumStaticProperties]std.ArrayList(u16) = .{ .empty, .empty },
    property_mapping: std.ArrayList(std.ArrayList(u8)) = .empty,
    num_samples: usize = 0,
    dedup_table: std.ArrayList(u32) = .empty,

    pub const kPropertyRange: i32 = 511;
    pub const kDedupEntryUnused: u32 = std.math.maxInt(u32);

    pub fn init(gpa: Allocator) TreeSamples {
        return .{ .gpa = gpa };
    }

    pub fn deinit(self: *TreeSamples) void {
        const gpa = self.gpa;
        for (self.residuals.items) |*r| r.deinit(gpa);
        self.residuals.deinit(gpa);
        self.sample_counts.deinit(gpa);
        for (&self.static_props) |*s| s.deinit(gpa);
        for (self.props.items) |*p| p.deinit(gpa);
        self.props.deinit(gpa);
        for (self.compact_properties.items) |*p| p.deinit(gpa);
        self.compact_properties.deinit(gpa);
        self.props_to_use.deinit(gpa);
        self.predictors.deinit(gpa);
        for (&self.static_property_mapping) |*s| s.deinit(gpa);
        for (self.property_mapping.items) |*p| p.deinit(gpa);
        self.property_mapping.deinit(gpa);
        self.dedup_table.deinit(gpa);
    }

    pub fn hasSamples(self: *const TreeSamples) bool {
        return self.residuals.items.len > 0 and self.residuals.items[0].items.len > 0;
    }
    pub fn numDistinctSamples(self: *const TreeSamples) usize {
        return self.sample_counts.items.len;
    }
    pub fn numSamples(self: *const TreeSamples) usize {
        return self.num_samples;
    }
    pub fn rTokens(self: *const TreeSamples, pred: usize) []const ResidualToken {
        return self.residuals.items[pred].items;
    }
    pub fn rToken(self: *const TreeSamples, pred: usize, i: usize) ResidualToken {
        return self.residuals.items[pred].items[i];
    }
    pub fn token(self: *const TreeSamples, pred: usize, i: usize) usize {
        return self.residuals.items[pred].items[i].tok;
    }
    pub fn count(self: *const TreeSamples, i: usize) usize {
        return self.sample_counts.items[i];
    }

    pub fn predictorIndex(self: *const TreeSamples, predictor: Predictor) usize {
        for (self.predictors.items, 0..) |p, i| {
            if (p == predictor) return i;
        }
        unreachable;
    }

    pub fn propertyIndex(self: *const TreeSamples, property: u32) usize {
        for (self.props_to_use.items, 0..) |p, i| {
            if (p == property) return i;
        }
        unreachable;
    }

    pub fn numPropertyValues(self: *const TreeSamples, property_index: usize) usize {
        return self.compact_properties.items[property_index].items.len + 1;
    }

    pub fn numStaticProps(self: *const TreeSamples) usize {
        return self.num_static_props;
    }

    /// Возвращает *квантованное* значение свойства.
    pub fn propertyStatic(self: *const TreeSamples, property_index: usize, i: usize) u32 {
        return self.static_props[property_index].items[i];
    }
    pub fn propertyNonStatic(self: *const TreeSamples, property_index: usize, i: usize) u32 {
        return self.props.items[property_index].items[i];
    }

    pub fn unquantizeProperty(self: *const TreeSamples, property_index: usize, quant: u32) i32 {
        std.debug.assert(quant < self.compact_properties.items[property_index].items.len);
        return self.compact_properties.items[property_index].items[quant];
    }

    pub fn predictorFromIndex(self: *const TreeSamples, index: usize) Predictor {
        return self.predictors.items[index];
    }

    pub fn propertyFromIndex(self: *const TreeSamples, index: usize) u32 {
        return self.props_to_use.items[index];
    }

    pub fn numPredictors(self: *const TreeSamples) usize {
        return self.predictors.items.len;
    }

    pub fn numProperties(self: *const TreeSamples) usize {
        return self.props_to_use.items.len;
    }

    /// Порт `SetPredictor`.
    pub fn setPredictor(self: *TreeSamples, predictor: Predictor, wp_tree_mode: TreeMode) !void {
        const gpa = self.gpa;
        self.predictors.clearRetainingCapacity();
        if (wp_tree_mode == .wp_only) {
            try self.predictors.append(gpa, .weighted);
            try self.residuals.resize(gpa, 1);
            self.residuals.items[0] = .empty;
            return;
        }
        if (wp_tree_mode == .no_wp and predictor == .weighted) {
            return error.InvalidPredictorSettings;
        }
        if (predictor == .variable) {
            for (0..options.kNumModularPredictors) |i| {
                try self.predictors.append(gpa, @enumFromInt(@as(u8, @intCast(i))));
            }
            std.mem.swap(Predictor, &self.predictors.items[0], &self.predictors.items[@intFromEnum(Predictor.weighted)]);
            std.mem.swap(Predictor, &self.predictors.items[1], &self.predictors.items[@intFromEnum(Predictor.gradient)]);
        } else if (predictor == .best) {
            try self.predictors.append(gpa, .weighted);
            try self.predictors.append(gpa, .gradient);
        } else {
            try self.predictors.append(gpa, predictor);
        }
        if (wp_tree_mode == .no_wp) {
            // Удалить Weighted.
            var i: usize = 0;
            while (i < self.predictors.items.len) {
                if (self.predictors.items[i] == .weighted) {
                    _ = self.predictors.orderedRemove(i);
                } else {
                    i += 1;
                }
            }
        }
        try self.residuals.resize(gpa, self.predictors.items.len);
        for (self.residuals.items) |*r| r.* = .empty;
    }

    /// Порт `SetProperties`.
    pub fn setProperties(self: *TreeSamples, properties: []const u32, wp_tree_mode: TreeMode) !void {
        const gpa = self.gpa;
        self.props_to_use.clearRetainingCapacity();
        try self.props_to_use.appendSlice(gpa, properties);
        if (wp_tree_mode == .wp_only) {
            self.props_to_use.clearRetainingCapacity();
            try self.props_to_use.append(gpa, @intCast(kWPProp));
        }
        if (wp_tree_mode == .gradient_only) {
            self.props_to_use.clearRetainingCapacity();
            try self.props_to_use.append(gpa, @intCast(kGradientProp));
        }
        if (wp_tree_mode == .no_wp) {
            var i: usize = 0;
            while (i < self.props_to_use.items.len) {
                if (self.props_to_use.items[i] == kWPProp) {
                    _ = self.props_to_use.orderedRemove(i);
                } else {
                    i += 1;
                }
            }
        }
        if (self.props_to_use.items.len == 0) {
            return error.InvalidPropertySetConfiguration;
        }
        self.num_static_props = 0;
        // Статические свойства, если есть, идут первыми.
        for (self.props_to_use.items, 0..) |prop, i| {
            if (prop < kNumStaticProperties) {
                std.debug.assert(i == prop);
                self.num_static_props += 1;
            }
        }
        try self.props.resize(gpa, self.props_to_use.items.len - self.num_static_props);
        for (self.props.items) |*p| p.* = .empty;
    }

    /// Порт `InitTable`.
    fn initTable(self: *TreeSamples, log_size: usize) !void {
        const gpa = self.gpa;
        const size = @as(usize, 1) << @intCast(log_size);
        if (self.dedup_table.items.len == size) return;
        try self.dedup_table.resize(gpa, size);
        @memset(self.dedup_table.items, kDedupEntryUnused);
        for (0..self.numDistinctSamples()) |i| {
            if (self.sample_counts.items[i] != std.math.maxInt(u16)) {
                self.addToTable(i);
            }
        }
    }

    /// Порт `AddToTableAndMerge`. Возвращает true, если образец уже был.
    fn addToTableAndMerge(self: *TreeSamples, a: usize) bool {
        const pos1 = self.hash1(a);
        const pos2 = self.hash2(a);
        if (self.dedup_table.items[pos1] != kDedupEntryUnused and
            self.isSameSample(a, self.dedup_table.items[pos1]))
        {
            self.sample_counts.items[self.dedup_table.items[pos1]] += 1;
            if (self.sample_counts.items[self.dedup_table.items[pos1]] == std.math.maxInt(u16)) {
                self.dedup_table.items[pos1] = kDedupEntryUnused;
            }
            return true;
        }
        if (self.dedup_table.items[pos2] != kDedupEntryUnused and
            self.isSameSample(a, self.dedup_table.items[pos2]))
        {
            self.sample_counts.items[self.dedup_table.items[pos2]] += 1;
            if (self.sample_counts.items[self.dedup_table.items[pos2]] == std.math.maxInt(u16)) {
                self.dedup_table.items[pos2] = kDedupEntryUnused;
            }
            return true;
        }
        self.addToTable(a);
        return false;
    }

    fn addToTable(self: *TreeSamples, a: usize) void {
        const pos1 = self.hash1(a);
        const pos2 = self.hash2(a);
        if (self.dedup_table.items[pos1] == kDedupEntryUnused) {
            self.dedup_table.items[pos1] = @intCast(a);
        } else if (self.dedup_table.items[pos2] == kDedupEntryUnused) {
            self.dedup_table.items[pos2] = @intCast(a);
        }
    }

    /// Порт `PrepareForSamples`.
    pub fn prepareForSamples(self: *TreeSamples, extra_num_samples: usize) !void {
        const gpa = self.gpa;
        for (self.residuals.items) |*res| {
            try res.ensureUnusedCapacity(gpa, extra_num_samples);
        }
        for (0..self.num_static_props) |i| {
            try self.static_props[i].ensureUnusedCapacity(gpa, extra_num_samples);
        }
        for (self.props.items) |*p| {
            try p.ensureUnusedCapacity(gpa, extra_num_samples);
        }
        const total_num_samples = extra_num_samples + self.sample_counts.items.len;
        const next_size = bits.ceilLog2Nonzero(@max(total_num_samples * 3 / 2, 1));
        try self.initTable(next_size);
    }

    /// Порт `Hash1`.
    fn hash1(self: *const TreeSamples, a: usize) usize {
        const constant: u64 = 0x1e35a7bd;
        var h: u64 = constant;
        for (self.residuals.items) |r| {
            h = h *% constant +% r.items[a].tok;
            h = h *% constant +% r.items[a].nbits;
        }
        for (0..self.num_static_props) |i| {
            h = h *% constant +% self.static_props[i].items[a];
        }
        for (self.props.items) |p| {
            h = h *% constant +% p.items[a];
        }
        return @intCast((h >> 16) & (self.dedup_table.items.len - 1));
    }

    /// Порт `Hash2`.
    fn hash2(self: *const TreeSamples, a: usize) usize {
        const constant: u64 = 0x1e35a7bd1e35a7bd;
        var h: u64 = constant;
        for (0..self.num_static_props) |i| {
            h = h *% constant ^ self.static_props[i].items[a];
        }
        for (self.props.items) |p| {
            h = h *% constant ^ p.items[a];
        }
        for (self.residuals.items) |r| {
            h = h *% constant ^ r.items[a].tok;
            h = h *% constant ^ r.items[a].nbits;
        }
        return @intCast((h >> 16) & (self.dedup_table.items.len - 1));
    }

    fn isSameSample(self: *const TreeSamples, a: usize, b: usize) bool {
        for (self.residuals.items) |r| {
            if (r.items[a].tok != r.items[b].tok) return false;
            if (r.items[a].nbits != r.items[b].nbits) return false;
        }
        for (0..self.num_static_props) |i| {
            if (self.static_props[i].items[a] != self.static_props[i].items[b]) return false;
        }
        for (self.props.items) |p| {
            if (p.items[a] != p.items[b]) return false;
        }
        return true;
    }

    /// `prop` — ИНДЕКС в props_to_use (как в C++ QuantizeProperty).
    pub fn quantizeProperty(self: *const TreeSamples, prop: u32, v_in: i32) u32 {
        std.debug.assert(prop >= self.num_static_props);
        const v = common.clamp1(v_in, -TreeSamples.kPropertyRange, TreeSamples.kPropertyRange) + TreeSamples.kPropertyRange;
        return self.property_mapping.items[prop - self.num_static_props].items[@intCast(v)];
    }

    pub fn quantizeStaticProperty(self: *const TreeSamples, prop: u32, v_in: i32) u32 {
        std.debug.assert(prop < self.num_static_props);
        const v = common.clamp1(v_in, -TreeSamples.kPropertyRange, TreeSamples.kPropertyRange) + TreeSamples.kPropertyRange;
        return self.static_property_mapping[prop].items[@intCast(v)];
    }

    /// Порт `AddSample`.
    pub fn addSample(
        self: *TreeSamples,
        pixel: pixel_type_w,
        properties: Properties,
        predictions: []const pixel_type_w,
    ) !void {
        const gpa = self.gpa;
        for (self.predictors.items, 0..) |pred, i| {
            const v: pixel_type = @truncate(pixel - predictions[@intFromEnum(pred)]);
            const enc = hybrid_uint.HybridUintConfig.init(4, 1, 2).encode(common.packSigned(v));
            if (enc.token >= 256 or enc.nbits >= 256) {
                return error.SampleTooLarge;
            }
            try self.residuals.items[i].append(gpa, .{ .tok = @intCast(enc.token), .nbits = @intCast(enc.nbits) });
        }
        for (0..self.num_static_props) |i| {
            try self.static_props[i].append(gpa, self.quantizeStaticProperty(@intCast(i), properties[i]));
        }
        var i: usize = self.num_static_props;
        while (i < self.props_to_use.items.len) : (i += 1) {
            try self.props.items[i - self.num_static_props].append(gpa, @intCast(self.quantizeProperty(@intCast(i), properties[self.props_to_use.items[i]])));
        }
        try self.sample_counts.append(gpa, 1);
        self.num_samples += 1;
        if (self.addToTableAndMerge(self.sample_counts.items.len - 1)) {
            for (self.residuals.items) |*r| r.items.len -= 1;
            for (0..self.num_static_props) |j| self.static_props[j].items.len -= 1;
            for (self.props.items) |*p| p.items.len -= 1;
            self.sample_counts.items.len -= 1;
        }
    }

    /// Порт `Swap`.
    pub fn swap(self: *TreeSamples, a: usize, b: usize) void {
        if (a == b) return;
        for (self.residuals.items) |*r| {
            std.mem.swap(ResidualToken, &r.items[a], &r.items[b]);
        }
        for (0..self.num_static_props) |i| {
            std.mem.swap(u32, &self.static_props[i].items[a], &self.static_props[i].items[b]);
        }
        for (self.props.items) |*p| {
            std.mem.swap(u8, &p.items[a], &p.items[b]);
        }
        std.mem.swap(u16, &self.sample_counts.items[a], &self.sample_counts.items[b]);
    }

    pub fn allSamplesDone(self: *TreeSamples) void {
        const gpa = self.gpa;
        self.dedup_table.clearAndFree(gpa);
    }

    /// Порт `PreQuantizeProperties`.
    pub fn preQuantizeProperties(
        self: *TreeSamples,
        range: StaticPropRange,
        multiplier_info: []const ModularMultiplierInfo,
        group_pixel_count: []const u32,
        channel_pixel_count: []const u32,
        pixel_samples: []const pixel_type,
        diff_samples: []const pixel_type,
        max_property_values: usize,
    ) !void {
        const gpa = self.gpa;
        // Пороги для канала и группы при принудительных расщеплениях.
        var channel_multiplier_thresholds: std.ArrayList(i32) = .empty;
        defer channel_multiplier_thresholds.deinit(gpa);
        var group_multiplier_thresholds: std.ArrayList(i32) = .empty;
        defer group_multiplier_thresholds.deinit(gpa);
        for (multiplier_info) |v| {
            if (v.range[0][0] != range[0][0]) {
                try channel_multiplier_thresholds.append(gpa, @intCast(v.range[0][0] - 1));
            }
            if (v.range[0][1] != range[0][1]) {
                try channel_multiplier_thresholds.append(gpa, @intCast(v.range[0][1] - 1));
            }
            if (v.range[1][0] != range[1][0]) {
                try group_multiplier_thresholds.append(gpa, @intCast(v.range[1][0] - 1));
            }
            if (v.range[1][1] != range[1][1]) {
                try group_multiplier_thresholds.append(gpa, @intCast(v.range[1][1] - 1));
            }
        }
        sortAndDedup(&channel_multiplier_thresholds);
        sortAndDedup(&group_multiplier_thresholds);

        try self.compact_properties.resize(gpa, self.props_to_use.items.len);
        for (self.compact_properties.items) |*cp| cp.* = .empty;

        var pixel_thresholds: std.ArrayList(i32) = .empty;
        defer pixel_thresholds.deinit(gpa);
        var abs_pixel_thresholds: std.ArrayList(i32) = .empty;
        defer abs_pixel_thresholds.deinit(gpa);
        var diff_thresholds: std.ArrayList(i32) = .empty;
        defer diff_thresholds.deinit(gpa);
        var abs_diff_thresholds: std.ArrayList(i32) = .empty;
        defer abs_diff_thresholds.deinit(gpa);

        // mutable копии сэмплов (C++ модифицирует по ссылке).
        var pixel_samples_mut: std.ArrayList(pixel_type) = .empty;
        defer pixel_samples_mut.deinit(gpa);
        try pixel_samples_mut.appendSlice(gpa, pixel_samples);
        var diff_samples_mut: std.ArrayList(pixel_type) = .empty;
        defer diff_samples_mut.deinit(gpa);
        try diff_samples_mut.appendSlice(gpa, diff_samples);

        try self.property_mapping.resize(gpa, self.props_to_use.items.len - self.num_static_props);
        for (self.property_mapping.items) |*pm| pm.* = .empty;

        for (self.props_to_use.items, 0..) |prop, i| {
            var cp: std.ArrayList(i32) = .empty;
            errdefer cp.deinit(gpa);
            if (prop == 0) {
                cp = try quantizeChannelOr(gpa, channel_multiplier_thresholds.items, channel_pixel_count, max_property_values);
            } else if (prop == 1) {
                cp = try quantizeGroupIdOr(gpa, group_multiplier_thresholds.items, group_pixel_count, max_property_values);
            } else if (prop == 2 or prop == 3) {
                cp = try quantizeCoordinate(gpa, max_property_values);
            } else if (prop == 6 or prop == 7 or prop == 8 or
                (prop >= kNumNonrefProperties and (prop - kNumNonrefProperties) % 4 == 1))
            {
                cp = try quantizePixelPropCopy(gpa, &pixel_samples_mut, &pixel_thresholds, max_property_values);
            } else if (prop == 4 or prop == 5 or
                (prop >= kNumNonrefProperties and (prop - kNumNonrefProperties) % 4 == 0))
            {
                cp = try quantizeAbsPixelPropCopy(gpa, &pixel_samples_mut, &pixel_thresholds, &abs_pixel_thresholds, max_property_values);
            } else if (prop >= kNumNonrefProperties and (prop - kNumNonrefProperties) % 4 == 2) {
                cp = try quantizeAbsDiffPropCopy(gpa, &diff_samples_mut, &diff_thresholds, &abs_diff_thresholds, max_property_values);
            } else if (prop == kWPProp) {
                cp = try quantizeWPCopy(gpa, max_property_values);
            } else {
                cp = try quantizeDiffPropCopy(gpa, &diff_samples_mut, &diff_thresholds, max_property_values);
            }
            self.compact_properties.items[i] = cp;

            if (i < self.num_static_props) {
                try quantMap(u16, gpa, self.compact_properties.items[i].items, &self.static_property_mapping[i], TreeSamples.kPropertyRange * 2 + 1, TreeSamples.kPropertyRange);
            } else {
                try quantMap(u8, gpa, self.compact_properties.items[i].items, &self.property_mapping.items[i - self.num_static_props], TreeSamples.kPropertyRange * 2 + 1, TreeSamples.kPropertyRange);
            }
        }

        // Примечание: в C++ pixel_samples/diff_samples модифицируются на
        // месте (берутся модули); здесь они только читаются, копии живут
        // внутри функции.
    }
};

// Хелперы PreQuantizeProperties (каждый возвращает копию порогов).

fn quantizeChannelOr(gpa: Allocator, cmt: []const i32, cpc: []const u32, mpv: usize) !std.ArrayList(i32) {
    if (cmt.len != 0) {
        var out: std.ArrayList(i32) = .empty;
        try out.appendSlice(gpa, cmt);
        return out;
    }
    return quantizeHistogram(gpa, cpc, mpv);
}

fn quantizeGroupIdOr(gpa: Allocator, gmt: []const i32, gpc: []const u32, mpv: usize) !std.ArrayList(i32) {
    if (gmt.len != 0) {
        var out: std.ArrayList(i32) = .empty;
        try out.appendSlice(gpa, gmt);
        return out;
    }
    return quantizeHistogram(gpa, gpc, mpv);
}

fn quantizePixelPropCopy(gpa: Allocator, pixel_mut: *std.ArrayList(pixel_type), pt: *std.ArrayList(i32), mpv: usize) !std.ArrayList(i32) {
    if (pt.items.len == 0) {
        clampSamples(pixel_mut.items);
        pt.deinit(gpa);
        pt.* = try quantizeSamples(gpa, pixel_mut.items, mpv);
    }
    var out: std.ArrayList(i32) = .empty;
    try out.appendSlice(gpa, pt.items);
    return out;
}

fn quantizeAbsPixelPropCopy(gpa: Allocator, pixel_mut: *std.ArrayList(pixel_type), pt: *std.ArrayList(i32), apt: *std.ArrayList(i32), mpv: usize) !std.ArrayList(i32) {
    if (apt.items.len == 0) {
        // The pixel thresholds are filled through `pt` as a side effect;
        // the returned copy has no use here and must be freed (libjxl's C++
        // drops the temporary vector, whose buffer is owned by the
        // std::vector destructor).
        var tmp = try quantizePixelPropCopy(gpa, pixel_mut, pt, mpv);
        tmp.deinit(gpa);
        for (pixel_mut.items) |*v| v.* = @intCast(@abs(v.*));
        apt.deinit(gpa);
        apt.* = try quantizeSamples(gpa, pixel_mut.items, mpv);
    }
    var out: std.ArrayList(i32) = .empty;
    try out.appendSlice(gpa, apt.items);
    return out;
}

fn quantizeDiffPropCopy(gpa: Allocator, diff_mut: *std.ArrayList(pixel_type), dt: *std.ArrayList(i32), mpv: usize) !std.ArrayList(i32) {
    if (dt.items.len == 0) {
        clampSamples(diff_mut.items);
        dt.deinit(gpa);
        dt.* = try quantizeSamples(gpa, diff_mut.items, mpv);
    }
    var out: std.ArrayList(i32) = .empty;
    try out.appendSlice(gpa, dt.items);
    return out;
}

fn quantizeAbsDiffPropCopy(gpa: Allocator, diff_mut: *std.ArrayList(pixel_type), dt: *std.ArrayList(i32), adt: *std.ArrayList(i32), mpv: usize) !std.ArrayList(i32) {
    if (adt.items.len == 0) {
        // Side-effect call (fills `dt`); free the returned copy.
        var tmp = try quantizeDiffPropCopy(gpa, diff_mut, dt, mpv);
        tmp.deinit(gpa);
        for (diff_mut.items) |*v| v.* = @intCast(@abs(v.*));
        adt.deinit(gpa);
        adt.* = try quantizeSamples(gpa, diff_mut.items, mpv);
    }
    var out: std.ArrayList(i32) = .empty;
    try out.appendSlice(gpa, adt.items);
    return out;
}

fn quantizeWPCopy(gpa: Allocator, mpv: usize) !std.ArrayList(i32) {
    var out: std.ArrayList(i32) = .empty;
    errdefer out.deinit(gpa);
    if (mpv < 32) {
        try out.appendSlice(gpa, &.{ -127, -63, -31, -15, -7, -3, -1, 0, 1, 3, 7, 15, 31, 63, 127 });
    } else if (mpv < 64) {
        try out.appendSlice(gpa, &.{ -255, -191, -127, -95, -63, -47, -31, -23, -15, -11, -7, -5, -3, -1, 0, 1, 3, 5, 7, 11, 15, 23, 31, 47, 63, 95, 127, 191, 255 });
    } else {
        try out.appendSlice(gpa, &.{ -255, -223, -191, -159, -127, -111, -95, -79, -63, -55, -47, -39, -31, -27, -23, -19, -15, -13, -11, -9, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 15, 19, 23, 27, 31, 39, 47, 55, 63, 79, 95, 111, 127, 159, 191, 223, 255 });
    }
    return out;
}

fn sortAndDedup(list: *std.ArrayList(i32)) void {
    std.mem.sort(i32, list.items, {}, std.sort.asc(i32));
    var out_len: usize = 0;
    for (list.items, 0..) |v, i| {
        if (i == 0 or v != list.items[out_len - 1]) {
            list.items[out_len] = v;
            out_len += 1;
        }
    }
    list.items.len = out_len;
}

/// Порт `QuantizeHistogram` (enc_ma.cc): пороги-квантили гистограммы.
pub fn quantizeHistogram(gpa: Allocator, histogram: []const u32, num_chunks: usize) !std.ArrayList(i32) {
    var out: std.ArrayList(i32) = .empty;
    errdefer out.deinit(gpa);
    if (histogram.len == 0 or num_chunks == 0) return out;
    var sum: u64 = 0;
    for (histogram) |h| sum += h;
    if (sum == 0) return out;
    var cumsum: u64 = 0;
    var threshold: u64 = 1;
    for (histogram, 0..) |h, i| {
        cumsum += h;
        if (cumsum * num_chunks >= threshold * sum) {
            try out.append(gpa, @intCast(i));
            while (cumsum * num_chunks >= threshold * sum) threshold += 1;
        }
    }
    // Последнее значение собирает всю гистограмму и порогом не является.
    _ = out.pop();
    return out;
}

const kSampleRange = 512;

/// Порт `ClampSamples`.
fn clampSamples(samples: []pixel_type) void {
    for (samples) |*v| {
        v.* = common.clamp1(v.*, -kSampleRange, kSampleRange);
    }
}

/// Порт `QuantizeSamples`: вызывать для значений в [-kSampleRange..kSampleRange].
pub fn quantizeSamples(gpa: Allocator, samples: []const pixel_type, num_chunks: usize) !std.ArrayList(i32) {
    var out: std.ArrayList(i32) = .empty;
    errdefer out.deinit(gpa);
    if (samples.len == 0) return out;
    var min_val: i32 = std.math.maxInt(i32);
    for (samples) |s| min_val = @min(min_val, s);
    var counts: [2 * kSampleRange + 1]u32 = .{0} ** (2 * kSampleRange + 1);
    for (samples) |s| {
        const sample_offset: u32 = @intCast(s - min_val);
        counts[sample_offset] += 1;
    }
    var thresholds = try quantizeHistogram(gpa, &counts, num_chunks);
    defer thresholds.deinit(gpa);
    for (thresholds.items) |*v| v.* += min_val;
    try out.appendSlice(gpa, thresholds.items);
    return out;
}

/// Порт `quantize_coordinate`.
fn quantizeCoordinate(gpa: Allocator, max_property_values: usize) !std.ArrayList(i32) {
    var out: std.ArrayList(i32) = .empty;
    errdefer out.deinit(gpa);
    var i: usize = 0;
    while (i + 1 < max_property_values) : (i += 1) {
        try out.append(gpa, @intCast((i + 1) * 256 / max_property_values - 1));
    }
    return out;
}

/// Порт `QuantMap`: to[i] = число порогов from, меньших i - bias.
fn quantMap(comptime T: type, gpa: Allocator, from: []const i32, to: *std.ArrayList(T), num_pegs: usize, bias: i32) !void {
    try to.resize(gpa, num_pegs);
    var mapped: usize = 0;
    for (0..num_pegs) |i| {
        while (mapped < from.len and @as(i32, @intCast(i)) - bias > from[mapped]) {
            mapped += 1;
        }
        to.items[i] = @intCast(mapped);
    }
}

/// Порт `SplitTreeSamples<S>`: переставляет образцы так, что [begin, pos)
/// содержит значения <= val, а [pos, end) — > val.
fn splitTreeSamplesStatic(tree_samples: *TreeSamples, begin: usize, pos: usize, end: usize, prop: usize, val: u32) void {
    var begin_pos = begin;
    var end_pos = pos;
    while (true) {
        while (begin_pos < pos and tree_samples.propertyStatic(prop, begin_pos) <= val) {
            begin_pos += 1;
        }
        while (end_pos < end and tree_samples.propertyStatic(prop, end_pos) > val) {
            end_pos += 1;
        }
        if (begin_pos < pos and end_pos < end) {
            tree_samples.swap(begin_pos, end_pos);
        }
        begin_pos += 1;
        end_pos += 1;
        if (!(begin_pos < pos and end_pos < end)) break;
    }
}

fn splitTreeSamplesNonStatic(tree_samples: *TreeSamples, begin: usize, pos: usize, end: usize, prop: usize, val: u32) void {
    var begin_pos = begin;
    var end_pos = pos;
    while (true) {
        while (begin_pos < pos and tree_samples.propertyNonStatic(prop, begin_pos) <= val) {
            begin_pos += 1;
        }
        while (end_pos < end and tree_samples.propertyNonStatic(prop, end_pos) > val) {
            end_pos += 1;
        }
        if (begin_pos < pos and end_pos < end) {
            tree_samples.swap(begin_pos, end_pos);
        }
        begin_pos += 1;
        end_pos += 1;
        if (!(begin_pos < pos and end_pos < end)) break;
    }
}

/// Порт `CollectExtraBitsIncrease<S>`.
fn collectExtraBitsIncreaseStatic(
    tree_samples: *const TreeSamples,
    rtokens: []const ResidualToken,
    count_increase: []i32,
    extra_bits_increase: []usize,
    begin: usize,
    end: usize,
    prop_idx: usize,
    max_symbols: usize,
) void {
    for (begin..end) |idx| {
        const rt = rtokens[idx];
        const cnt = tree_samples.count(idx);
        const p = tree_samples.propertyStatic(prop_idx, idx);
        const sym: usize = rt.tok;
        const ebi: usize = @as(usize, rt.nbits) * cnt;
        count_increase[p * max_symbols + sym] += @intCast(cnt);
        extra_bits_increase[p] += ebi;
    }
}

fn collectExtraBitsIncreaseNonStatic(
    tree_samples: *const TreeSamples,
    rtokens: []const ResidualToken,
    count_increase: []i32,
    extra_bits_increase: []usize,
    begin: usize,
    end: usize,
    prop_idx: usize,
    max_symbols: usize,
) void {
    for (begin..end) |idx| {
        const rt = rtokens[idx];
        const cnt = tree_samples.count(idx);
        const p = tree_samples.propertyNonStatic(prop_idx, idx);
        const sym: usize = rt.tok;
        const ebi: usize = @as(usize, rt.nbits) * cnt;
        count_increase[p * max_symbols + sym] += @intCast(cnt);
        extra_bits_increase[p] += ebi;
    }
}

/// Порт `FindBestSplit` — жадное построение дерева по выборке образцов.
pub fn findBestSplit(
    gpa: Allocator,
    tree_samples: *TreeSamples,
    threshold: f32,
    mul_info: []const ModularMultiplierInfo,
    initial_static_prop_range: StaticPropRange,
    fast_decode_multiplier: f32,
    tree: *Tree,
) !void {
    const NodeInfo = struct {
        pos: usize,
        begin: usize,
        end: usize,
        static_prop_range: StaticPropRange,
    };
    var nodes: std.ArrayList(NodeInfo) = .empty;
    defer nodes.deinit(gpa);
    try nodes.append(gpa, .{
        .pos = 0,
        .begin = 0,
        .end = tree_samples.numDistinctSamples(),
        .static_prop_range = initial_static_prop_range,
    });

    const num_predictors = tree_samples.numPredictors();
    const num_properties = tree_samples.numProperties();

    while (nodes.items.len > 0) {
        const node_info = nodes.pop().?;
        const pos = node_info.pos;
        const begin = node_info.begin;
        const end = node_info.end;
        const static_prop_range = node_info.static_prop_range;
        if (begin == end) continue;

        const SplitInfo = struct {
            prop: usize = 0,
            val: u32 = 0,
            pos: usize = 0,
            lcost: f32 = std.math.floatMax(f32),
            rcost: f32 = std.math.floatMax(f32),
            lpred: Predictor = .zero,
            rpred: Predictor = .zero,

            fn cost(self: *const @This()) f32 {
                return self.lcost + self.rcost;
            }
        };

        var best_split_static_constant = SplitInfo{};
        var best_split_static = SplitInfo{};
        var best_split_nonstatic = SplitInfo{};
        var best_split_nowp = SplitInfo{};

        // Максимум токенов в диапазоне.
        var max_symbols: usize = 0;
        for (0..num_predictors) |pred| {
            for (begin..end) |i| {
                const tok = tree_samples.token(pred, i);
                max_symbols = if (max_symbols > tok + 1) max_symbols else tok + 1;
            }
        }

        const counts = try gpa.alloc(i32, max_symbols * num_predictors);
        defer gpa.free(counts);
        @memset(counts, 0);
        const tot_extra_bits = try gpa.alloc(usize, num_predictors);
        defer gpa.free(tot_extra_bits);
        @memset(tot_extra_bits, 0);
        for (0..num_predictors) |pred| {
            var extra_bits: usize = 0;
            const rtokens = tree_samples.rTokens(pred);
            for (begin..end) |i| {
                const rt = rtokens[i];
                const cnt = tree_samples.count(i);
                const eb = @as(usize, rt.nbits) * cnt;
                counts[pred * max_symbols + rt.tok] += @intCast(cnt);
                extra_bits += eb;
            }
            tot_extra_bits[pred] = extra_bits;
        }

        var base_bits: f32 = undefined;
        {
            const pred = tree_samples.predictorIndex(tree.items[pos].predictor);
            base_bits = estimateBits(counts[pred * max_symbols ..], max_symbols) + @as(f32, @floatFromInt(tot_extra_bits[pred]));
        }

        var best: *SplitInfo = &best_split_nonstatic;

        var forced_split = SplitInfo{};
        // Расщепления по диапазонам множителей.
        for (mul_info) |mmi| {
            var axis: u32 = 0;
            var val: u32 = 0;
            const t = boxIntersects(static_prop_range, mmi.range, &axis, &val);
            if (t == .none) continue;
            if (t == .inside) {
                tree.items[pos].multiplier = mmi.multiplier;
                break;
            }
            if (t == .partial) {
                forced_split.val = tree_samples.quantizeStaticProperty(axis, @intCast(val));
                forced_split.prop = axis;
                forced_split.lcost = base_bits / 2 - threshold;
                forced_split.rcost = base_bits / 2 - threshold;
                forced_split.lpred = tree.items[pos].predictor;
                forced_split.rpred = tree.items[pos].predictor;
                best = &forced_split;
                best.pos = begin;
                if (best.prop < tree_samples.numStaticProps()) {
                    for (begin..end) |x| {
                        if (tree_samples.propertyStatic(best.prop, x) <= best.val) {
                            best.pos += 1;
                        }
                    }
                } else {
                    const prop = best.prop - tree_samples.numStaticProps();
                    for (begin..end) |x| {
                        if (tree_samples.propertyNonStatic(prop, x) <= best.val) {
                            best.pos += 1;
                        }
                    }
                }
                break;
            }
        }

        if (best != &forced_split) {
            var prop_value_used_count: std.ArrayList(i32) = .empty;
            defer prop_value_used_count.deinit(gpa);
            var count_increase: std.ArrayList(i32) = .empty;
            defer count_increase.deinit(gpa);
            var extra_bits_increase: std.ArrayList(usize) = .empty;
            defer extra_bits_increase.deinit(gpa);

            const CostInfo = struct {
                cost: f32 = std.math.floatMax(f32),
                extra_cost: f32 = 0,
                pred: Predictor = .zero,

                fn total(self: *const @This()) f32 {
                    return self.cost + self.extra_cost;
                }
            };
            var costs_l: std.ArrayList(CostInfo) = .empty;
            defer costs_l.deinit(gpa);
            var costs_r: std.ArrayList(CostInfo) = .empty;
            defer costs_r.deinit(gpa);

            const counts_above = try gpa.alloc(i32, max_symbols);
            defer gpa.free(counts_above);
            const counts_below = try gpa.alloc(i32, max_symbols);
            defer gpa.free(counts_below);

            // Чем ниже порог, тем шумнее оценка; не спешим менять предиктор.
            const change_pred_penalty: f32 = 800.0 / (100.0 + threshold);
            for (0..num_properties) |prop| {
                if (!(base_bits > threshold)) break;
                costs_l.clearRetainingCapacity();
                costs_r.clearRetainingCapacity();
                const prop_size = tree_samples.numPropertyValues(prop);
                if (extra_bits_increase.items.len < prop_size) {
                    try count_increase.resize(gpa, prop_size * max_symbols);
                    @memset(count_increase.items, 0);
                    try extra_bits_increase.resize(gpa, prop_size);
                    @memset(extra_bits_increase.items, 0);
                }
                prop_value_used_count.clearRetainingCapacity();
                try prop_value_used_count.resize(gpa, prop_size);
                @memset(prop_value_used_count.items, 0);

                var first_used: usize = prop_size;
                var last_used: usize = 0;

                if (prop < tree_samples.numStaticProps()) {
                    for (begin..end) |i| {
                        const p = tree_samples.propertyStatic(prop, i);
                        prop_value_used_count.items[p] += 1;
                        last_used = @max(last_used, p);
                        first_used = @min(first_used, p);
                    }
                } else {
                    const prop_idx = prop - tree_samples.numStaticProps();
                    for (begin..end) |i| {
                        const p = tree_samples.propertyNonStatic(prop_idx, i);
                        prop_value_used_count.items[p] += 1;
                        last_used = @max(last_used, p);
                        first_used = @min(first_used, p);
                    }
                }
                try costs_l.resize(gpa, last_used - first_used);
                @memset(costs_l.items, CostInfo{});
                try costs_r.resize(gpa, last_used - first_used);
                @memset(costs_r.items, CostInfo{});

                // Для всех предикторов — стоимость каждой стороны расщепления.
                for (0..num_predictors) |pred| {
                    const rtokens = tree_samples.rTokens(pred);
                    if (prop < tree_samples.numStaticProps()) {
                        collectExtraBitsIncreaseStatic(tree_samples, rtokens, count_increase.items, extra_bits_increase.items, begin, end, prop, max_symbols);
                    } else {
                        collectExtraBitsIncreaseNonStatic(tree_samples, rtokens, count_increase.items, extra_bits_increase.items, begin, end, prop - tree_samples.numStaticProps(), max_symbols);
                    }
                    @memcpy(counts_above, counts[pred * max_symbols .. pred * max_symbols + max_symbols]);
                    @memset(counts_below, 0);
                    var extra_bits_below: usize = 0;
                    // last_used исключается: ни одна сторона не пуста.
                    var i = first_used;
                    while (i < last_used) : (i += 1) {
                        if (prop_value_used_count.items[i] == 0) continue;
                        extra_bits_below += extra_bits_increase.items[i];
                        extra_bits_increase.items[i] = 0;
                        for (0..max_symbols) |sym| {
                            counts_above[sym] -= count_increase.items[i * max_symbols + sym];
                            counts_below[sym] += count_increase.items[i * max_symbols + sym];
                            count_increase.items[i * max_symbols + sym] = 0;
                        }
                        const rcost = estimateBits(counts_above, max_symbols) +
                            @as(f32, @floatFromInt(tot_extra_bits[pred] - extra_bits_below));
                        const lcost = estimateBits(counts_below, max_symbols) +
                            @as(f32, @floatFromInt(extra_bits_below));
                        var penalty: f32 = 0;
                        // Не отговариваем от ухода от Weighted.
                        if (tree_samples.predictorFromIndex(pred) != tree.items[pos].predictor and
                            tree.items[pos].predictor != .weighted)
                        {
                            penalty = change_pred_penalty;
                        }
                        // При равенстве — предпочесть Zero, штрафовать Weighted.
                        if (tree_samples.predictorFromIndex(pred) == .weighted) {
                            penalty += 1e-8;
                        }
                        if (tree_samples.predictorFromIndex(pred) == .zero) {
                            penalty -= 1e-8;
                        }
                        if (rcost + penalty < costs_r.items[i - first_used].total()) {
                            costs_r.items[i - first_used].cost = rcost;
                            costs_r.items[i - first_used].extra_cost = penalty;
                            costs_r.items[i - first_used].pred = tree_samples.predictorFromIndex(pred);
                        }
                        if (lcost + penalty < costs_l.items[i - first_used].total()) {
                            costs_l.items[i - first_used].cost = lcost;
                            costs_l.items[i - first_used].extra_cost = penalty;
                            costs_l.items[i - first_used].pred = tree_samples.predictorFromIndex(pred);
                        }
                    }
                }
                // Поиск расщепления с минимальной суммой стоимостей сторон.
                var split = begin;
                var i = first_used;
                while (i < last_used) : (i += 1) {
                    if (prop_value_used_count.items[i] == 0) continue;
                    split += @intCast(prop_value_used_count.items[i]);
                    const rcost = costs_r.items[i - first_used].cost;
                    const lcost = costs_l.items[i - first_used].cost;

                    const uses_wp = tree_samples.propertyFromIndex(prop) == kWPProp or
                        costs_l.items[i - first_used].pred == .weighted or
                        costs_r.items[i - first_used].pred == .weighted;
                    const zero_entropy_side = rcost == 0 or lcost == 0;

                    const best_ref: *SplitInfo =
                        if (tree_samples.propertyFromIndex(prop) < kNumStaticProperties)
                            (if (zero_entropy_side) &best_split_static_constant else &best_split_static)
                        else
                            (if (uses_wp) &best_split_nonstatic else &best_split_nowp);
                    if (lcost + rcost < best_ref.cost()) {
                        best_ref.prop = prop;
                        best_ref.val = @intCast(i);
                        best_ref.pos = split;
                        best_ref.lcost = lcost;
                        best_ref.lpred = costs_l.items[i - first_used].pred;
                        best_ref.rcost = rcost;
                        best_ref.rpred = costs_r.items[i - first_used].pred;
                    }
                }
                // Очистка для last_used.
                if (last_used < extra_bits_increase.items.len) {
                    extra_bits_increase.items[last_used] = 0;
                    for (0..max_symbols) |sym| {
                        count_increase.items[last_used * max_symbols + sym] = 0;
                    }
                }
            }

            // Постараться не вводить WP.
            if (best_split_nowp.cost() + threshold < base_bits and
                best_split_nowp.cost() <= fast_decode_multiplier * best.cost())
            {
                best = &best_split_nowp;
            }
            // Расщепление по статическим свойствам, если не сильно дороже.
            if (best_split_static.cost() + threshold < base_bits and
                best_split_static.cost() <= fast_decode_multiplier * best.cost())
            {
                best = &best_split_static;
            }
            // Статические свойства для создания константных узлов.
            if (best_split_static_constant.cost() + threshold < base_bits) {
                best = &best_split_static_constant;
            }
        }

        if (best.cost() + threshold < base_bits) {
            const p = tree_samples.propertyFromIndex(best.prop);
            const dequant = tree_samples.unquantizeProperty(best.prop, best.val);
            // Расщепить узел и продолжить с потомками.
            try makeSplitNode(tree, gpa, pos, @intCast(p), dequant, best.lpred, 0, best.rpred, 0);
            // "Сортировка" по победившему свойству.
            if (best.prop < tree_samples.numStaticProps()) {
                splitTreeSamplesStatic(tree_samples, begin, best.pos, end, best.prop, best.val);
            } else {
                splitTreeSamplesNonStatic(tree_samples, begin, best.pos, end, best.prop - tree_samples.numStaticProps(), best.val);
            }
            var new_sp_range = static_prop_range;
            if (p < kNumStaticProperties) {
                new_sp_range[p][1] = @intCast(dequant + 1);
            }
            // rchild: [begin, best->pos), значения <= splitval.
            try nodes.append(gpa, .{ .pos = tree.items[pos].rchild, .begin = begin, .end = best.pos, .static_prop_range = new_sp_range });
            new_sp_range = static_prop_range;
            if (p < kNumStaticProperties) {
                new_sp_range[p][0] = @intCast(dequant + 1);
            }
            // lchild: [best->pos, end), значения > splitval.
            try nodes.append(gpa, .{ .pos = tree.items[pos].lchild, .begin = best.pos, .end = end, .static_prop_range = new_sp_range });
        }
    }
}

/// Порт `ComputeBestTree`.
pub fn computeBestTree(
    gpa: Allocator,
    tree_samples: *TreeSamples,
    threshold: f32,
    mul_info: []const ModularMultiplierInfo,
    static_prop_range: StaticPropRange,
    fast_decode_multiplier: f32,
    tree: *Tree,
) !void {
    tree.clearRetainingCapacity();
    try tree.append(gpa, .{});
    tree.items[0].property = -1;
    tree.items[0].predictor = tree_samples.predictorFromIndex(0);
    tree.items[0].predictor_offset = 0;
    tree.items[0].multiplier = 1;
    if (tree_samples.numProperties() >= 64) return error.TooManyProperties;
    if (tree_samples.numDistinctSamples() > std.math.maxInt(u32)) return error.TooManySamples;
    try findBestSplit(gpa, tree_samples, threshold, mul_info, static_prop_range, fast_decode_multiplier, tree);
}

/// Порт `CollectPixelSamples`.
pub fn collectPixelSamples(
    gpa: Allocator,
    img: *const image_mod.Image,
    opts: *const options.ModularOptions,
    group_id: u32,
    group_pixel_count: *std.ArrayList(u32),
    channel_pixel_count: *std.ArrayList(u32),
    pixel_samples: *std.ArrayList(pixel_type),
    diff_samples: *std.ArrayList(pixel_type),
) !void {
    if (opts.nb_repeats == 0) return;
    while (group_pixel_count.items.len <= group_id) {
        try group_pixel_count.append(gpa, 0);
    }
    while (channel_pixel_count.items.len < img.channels.items.len) {
        try channel_pixel_count.append(gpa, 0);
    }
    var rng = rng_mod.Rng.init(group_id);
    // 10% от итогового числа образцов для квантования свойств.
    const fraction = @min(opts.nb_repeats * 0.1, 0.99);
    const dist = rng_mod.Rng.makeGeometric(fraction);
    var total_pixels: usize = 0;
    var channel_ids: std.ArrayList(usize) = .empty;
    defer channel_ids.deinit(gpa);
    for (img.channels.items, 0..) |*ch, i| {
        if (i >= img.nb_meta_channels and
            (ch.w > opts.max_chan_size or ch.h > opts.max_chan_size))
        {
            break;
        }
        if (ch.w <= 1 or ch.h == 0) continue; // пропустить пустые/узкие каналы
        try channel_ids.append(gpa, i);
        group_pixel_count.items[group_id] += @intCast(ch.w * ch.h);
        channel_pixel_count.items[i] += @intCast(ch.w * ch.h);
        total_pixels += ch.w * ch.h;
    }
    if (channel_ids.items.len == 0) return;
    try pixel_samples.ensureUnusedCapacity(gpa, @intFromFloat(@as(f64, fraction) * @as(f64, @floatFromInt(total_pixels))));
    try diff_samples.ensureUnusedCapacity(gpa, @intFromFloat(@as(f64, fraction) * @as(f64, @floatFromInt(total_pixels))));
    var i: usize = 0;
    var y: usize = 0;
    var x: usize = 0;
    const Advance = struct {
        img: *const image_mod.Image,
        ids: []const usize,
        x: *usize,
        y: *usize,
        i: *usize,

        fn advance(c: *const @This(), amount: usize) void {
            c.x.* += amount;
            while (c.i.* < c.ids.len and c.x.* >= c.img.channels.items[c.ids[c.i.*]].w) {
                c.x.* -= c.img.channels.items[c.ids[c.i.*]].w;
                c.y.* += 1;
                if (c.y.* == c.img.channels.items[c.ids[c.i.*]].h) {
                    c.i.* += 1;
                    c.y.* = 0;
                    if (c.i.* >= c.ids.len) {
                        return;
                    }
                }
            }
        }
    };
    const adv = Advance{ .img = img, .ids = channel_ids.items, .x = &x, .y = &y, .i = &i };
    adv.advance(rng.geometric(dist));
    while (i < channel_ids.items.len) : (adv.advance(rng.geometric(dist) + 1)) {
        const ch = &img.channels.items[channel_ids.items[i]];
        const row_data = ch.row(y);
        try pixel_samples.append(gpa, row_data[x]);
        const xp: usize = if (x == 0) 1 else x - 1;
        try diff_samples.append(gpa, @truncate(@as(i64, row_data[x]) - row_data[xp]));
    }
}

/// Порт `TokenizeTree` (enc_ma.cc): обход дерева в ширину, выдача токенов и
/// перестроенного дерева для декодера.
pub fn tokenizeTree(
    gpa: Allocator,
    tree: []const PropertyDecisionNode,
    tokens: *std.ArrayList(Token),
    decoder_tree: *Tree,
) !void {
    if (tree.len > ma.kMaxTreeSize) return error.TreeTooLarge;
    var q: std.ArrayList(usize) = .empty;
    defer q.deinit(gpa);
    var head: usize = 0;
    try q.append(gpa, 0);
    var leaf_id: usize = 0;
    decoder_tree.clearRetainingCapacity();
    while (head < q.items.len) {
        const cur = q.items[head];
        head += 1;
        if (tree[cur].property < -1) return error.InvalidTree;
        try tokens.append(gpa, .{ .context = @intCast(@intFromEnum(ma.MATreeContext.property)), .value = @intCast(tree[cur].property + 1) });
        if (tree[cur].property == -1) {
            try tokens.append(gpa, .{
                .context = @intCast(@intFromEnum(ma.MATreeContext.predictor)),
                .value = @intFromEnum(tree[cur].predictor),
            });
            try tokens.append(gpa, .{
                .context = @intCast(@intFromEnum(ma.MATreeContext.offset)),
                .value = common.packSigned(@as(pixel_type, @truncate(tree[cur].predictor_offset))),
            });
            const mul_log: u32 = @intCast(bits.num0BitsBelowLS1BitNonzero(tree[cur].multiplier));
            const mul_bits: u32 = (tree[cur].multiplier >> @intCast(mul_log)) - 1;
            try tokens.append(gpa, .{ .context = @intCast(@intFromEnum(ma.MATreeContext.multiplier_log)), .value = mul_log });
            try tokens.append(gpa, .{ .context = @intCast(@intFromEnum(ma.MATreeContext.multiplier_bits)), .value = mul_bits });
            if (@intFromEnum(tree[cur].predictor) >= @intFromEnum(Predictor.best)) {
                return error.InvalidPredictor;
            }
            try decoder_tree.append(gpa, PropertyDecisionNode.leaf(tree[cur].predictor, tree[cur].predictor_offset, tree[cur].multiplier));
            decoder_tree.items[decoder_tree.items.len - 1].lchild = @intCast(leaf_id);
            leaf_id += 1;
            continue;
        }
        const child_base = decoder_tree.items.len + (q.items.len - head) + 1;
        try decoder_tree.append(gpa, PropertyDecisionNode.split(
            tree[cur].property,
            tree[cur].splitval,
            @intCast(child_base),
            @intCast(child_base + 1),
        ));
        try q.append(gpa, tree[cur].lchild);
        try q.append(gpa, tree[cur].rchild);
        try tokens.append(gpa, .{
            .context = @intCast(@intFromEnum(ma.MATreeContext.split_val)),
            .value = common.packSigned(tree[cur].splitval),
        });
    }
}

// ---------------------------------------------------------------------------
// Тесты
// ---------------------------------------------------------------------------

const testing = std.testing;

test "estimateBits: нулевая энтропия для одного символа" {
    const counts = [_]i32{ 0, 0, 100, 0 };
    const b = estimateBits(&counts, counts.len);
    try testing.expectEqual(@as(f32, 0), b);
}

test "estimateBits: равномерное распределение ~ log2(num_symbols)" {
    const counts = [_]i32{ 10, 10, 10, 10 };
    const b = estimateBits(&counts, counts.len);
    // 4 символа по 25% -> ~2 бита/символ * 40 = ~80 бит.
    try testing.expect(b > 70 and b < 90);
}

test "fastLog2f: близко к log2" {
    try testing.expectApproxEqAbs(@as(f32, 3.0), fastLog2f(8.0), 0.01);
    try testing.expectApproxEqAbs(@as(f32, 0.0), fastLog2f(1.0), 0.01);
    try testing.expectApproxEqAbs(@as(f32, -2.0), fastLog2f(0.25), 0.01);
}

test "makeSplitNode: перекрёстное именование потомков" {
    const gpa = testing.allocator;
    var tree = Tree.empty;
    defer tree.deinit(gpa);
    try tree.append(gpa, .{ .property = -1, .predictor = .zero });
    try makeSplitNode(&tree, gpa, 0, 2, 42, .left, 7, .gradient, -7);
    // Корень стал внутренним.
    try testing.expectEqual(@as(i16, 2), tree.items[0].property);
    try testing.expectEqual(@as(i32, 42), tree.items[0].splitval);
    try testing.expectEqual(@as(u32, 1), tree.items[0].lchild);
    try testing.expectEqual(@as(u32, 2), tree.items[0].rchild);
    // lchild -> ветка "> splitval" -> rpred (gradient).
    try testing.expectEqual(Predictor.gradient, tree.items[1].predictor);
    try testing.expectEqual(@as(i64, -7), tree.items[1].predictor_offset);
    // rchild -> ветка "<= splitval" -> lpred (left).
    try testing.expectEqual(Predictor.left, tree.items[2].predictor);
    try testing.expectEqual(@as(i64, 7), tree.items[2].predictor_offset);
    try testing.expect(ma.validateTree(gpa, tree.items));
}

test "TreeSamples: addSample с дедупликацией" {
    const gpa = testing.allocator;
    var ts = TreeSamples.init(gpa);
    defer ts.deinit();
    try ts.setPredictor(.zero, .default);
    try ts.setProperties(&.{ 0, 1, 15, 9, 10, 11, 12, 13 }, .default);
    // Пропустим preQuantizeProperties: используем прямое заполнение
    // компактных свойств нельзя — значит используем quantize* через
    // подготовленные отображения. Для простоты: prepareForSamples + addSample
    // требует property_mapping; вместо этого проверим дедупликацию на
    // статических свойствах через предварительное квантование.
    try ts.preQuantizeProperties(
        .{ .{ 0, 100 }, .{ 0, 100 } },
        &.{},
        &.{100},
        &.{100},
        &.{ 10, 20, 30 },
        &.{ 1, 2, 3 },
        8,
    );
    try ts.prepareForSamples(16);

    var predictions: [options.kNumModularEncoderPredictors]pixel_type_w = .{0} ** options.kNumModularEncoderPredictors;
    var props: [options.kNumNonrefProperties]pixel_type = .{0} ** options.kNumNonrefProperties;
    props[0] = 0; // channel
    props[1] = 0; // group
    props[15] = 5; // WP prop
    props[9] = 3;
    props[10] = -2;
    props[11] = 7;
    props[12] = 1;
    props[13] = 4;

    // Одинаковые образцы: 5 раз.
    for (0..5) |_| {
        try ts.addSample(100, &props, &predictions);
    }
    try testing.expectEqual(@as(usize, 5), ts.numSamples());
    try testing.expectEqual(@as(usize, 1), ts.numDistinctSamples());
    // Другой образец (квантованное значение WP-свойства изменится:
    // пороги {..., 3, 7, ...} -> 8 квантуется в 11, а 5 в 10).
    props[15] = 8;
    try ts.addSample(110, &props, &predictions);
    try testing.expectEqual(@as(usize, 6), ts.numSamples());
    try testing.expectEqual(@as(usize, 2), ts.numDistinctSamples());
    ts.allSamplesDone();
}

test "tokenizeTree: одиночный лист" {
    const gpa = testing.allocator;
    var tree = Tree.empty;
    defer tree.deinit(gpa);
    try tree.append(gpa, PropertyDecisionNode.leaf(.gradient, 3, 8));

    var tokens: std.ArrayList(Token) = .empty;
    defer tokens.deinit(gpa);
    var decoder_tree = Tree.empty;
    defer decoder_tree.deinit(gpa);
    try tokenizeTree(gpa, tree.items, &tokens, &decoder_tree);

    // property+1=0, predictor=5, offset=PackSigned(3)=6, mul_log=3, mul_bits=0
    try testing.expectEqual(@as(usize, 5), tokens.items.len);
    try testing.expectEqual(@as(u32, @intFromEnum(ma.MATreeContext.property)), tokens.items[0].context);
    try testing.expectEqual(@as(u32, 0), tokens.items[0].value);
    try testing.expectEqual(@as(u32, 5), tokens.items[1].value); // gradient
    try testing.expectEqual(@as(u32, 6), tokens.items[2].value); // PackSigned(3)
    try testing.expectEqual(@as(u32, 3), tokens.items[3].value); // mul_log
    try testing.expectEqual(@as(u32, 0), tokens.items[4].value); // mul_bits
    // Дерево декодера совпадает.
    try testing.expectEqual(@as(usize, 1), decoder_tree.items.len);
    try testing.expectEqual(Predictor.gradient, decoder_tree.items[0].predictor);
    try testing.expectEqual(@as(i64, 3), decoder_tree.items[0].predictor_offset);
    try testing.expectEqual(@as(u32, 8), decoder_tree.items[0].multiplier);
}

test "tokenizeTree/decodeTree: round trip дерева" {
    const gpa = testing.allocator;
    var tree = Tree.empty;
    defer tree.deinit(gpa);
    try tree.append(gpa, .{ .property = -1 });
    try makeSplitNode(&tree, gpa, 0, 9, 5, .left, 0, .gradient, 0);
    try makeSplitNode(&tree, gpa, 1, 10, 7, .top, 0, .zero, 0);

    var tokens: std.ArrayList(Token) = .empty;
    defer tokens.deinit(gpa);
    var decoder_tree = Tree.empty;
    defer decoder_tree.deinit(gpa);
    try tokenizeTree(gpa, tree.items, &tokens, &decoder_tree);

    // Декодируем mock-ридером.
    const MockSymbolReader = @import("symbol_reader.zig").MockSymbolReader;
    var seqs: [kNumTreeContexts][]const u32 = undefined;
    var storage: [kNumTreeContexts]std.ArrayList(u32) = .{ .empty, .empty, .empty, .empty, .empty, .empty };
    defer for (&storage) |*s| s.deinit(gpa);
    for (tokens.items) |t| {
        try storage[t.context].append(gpa, t.value);
    }
    for (&seqs, 0..) |*s, i| s.* = storage[i].items;
    var reader = try MockSymbolReader.init(gpa, &seqs);
    defer reader.deinit(gpa);

    var decoded = Tree.empty;
    defer decoded.deinit(gpa);
    const BitReader = @import("../base/bit_io.zig").BitReader;
    var br = BitReader.init(&.{});
    try ma.decodeTree(&br, &reader, &decoded, gpa, 1 << 20);
    try testing.expectEqual(decoder_tree.items.len, decoded.items.len);
    for (decoder_tree.items, decoded.items) |a, b| {
        try testing.expectEqual(a.property, b.property);
        try testing.expectEqual(a.splitval, b.splitval);
        try testing.expectEqual(a.predictor, b.predictor);
        try testing.expectEqual(a.predictor_offset, b.predictor_offset);
        try testing.expectEqual(a.multiplier, b.multiplier);
    }
}

test "computeBestTree: простое расщепление по свойству" {
    const gpa = testing.allocator;
    var ts = TreeSamples.init(gpa);
    defer ts.deinit();
    // Два предиктора: Zero и Gradient (как Predictor::Best, но без WP-штрафа
    // сложности).
    try ts.setPredictor(.best, .default);
    try ts.setProperties(&.{ 0, 1, 15, 9, 10, 11, 12, 13 }, .default);
    try ts.preQuantizeProperties(
        .{ .{ 0, 1 }, .{ 0, 1 } },
        &.{},
        &.{32},
        &.{ 16, 16 },
        &.{ 0, 0, 0, 0, 0, 0, 0, 0 },
        &.{ 0, 0, 0, 0, 0, 0, 0, 0 },
        8,
    );
    try ts.prepareForSamples(64);

    // Образцы: первая группа (channel=0) — хорошо предсказывается Zero,
    // вторая (channel=1) — Gradient. Свойство 0 (channel) должно дать
    // расщепление с разными предикторами.
    var props: [options.kNumNonrefProperties]pixel_type = .{0} ** options.kNumNonrefProperties;
    var predictions: [options.kNumModularEncoderPredictors]pixel_type_w = .{0} ** options.kNumModularEncoderPredictors;

    // Группа A: channel=0, остатки ~ 0 для Zero.
    for (0..16) |_| {
        props[0] = 0;
        props[1] = 0;
        // predictions[zero] = 10, predictions[gradient] = 40: residual 0/30.
        predictions[@intFromEnum(Predictor.zero)] = 10;
        predictions[@intFromEnum(Predictor.gradient)] = 40;
        try ts.addSample(10, &props, &predictions);
    }
    // Группа B: channel=1, остатки ~ 0 для Gradient.
    for (0..16) |_| {
        props[0] = 1;
        props[1] = 0;
        predictions[@intFromEnum(Predictor.zero)] = 100;
        predictions[@intFromEnum(Predictor.gradient)] = 50;
        try ts.addSample(50, &props, &predictions);
    }
    ts.allSamplesDone();

    var tree = Tree.empty;
    defer tree.deinit(gpa);
    try computeBestTree(gpa, &ts, 0.5, &.{}, .{ .{ 0, 2 }, .{ 0, 2 } }, 1.01, &tree);
    // Дерево должно содержать расщепление по каналу (property 0).
    try testing.expect(tree.items.len > 1);
    var found_channel_split = false;
    for (tree.items) |n| {
        if (n.property == 0) found_channel_split = true;
    }
    try testing.expect(found_channel_split);
    try testing.expect(ma.validateTree(gpa, tree.items));
}

test "quantizeHistogram: пороги-квантили" {
    const gpa = testing.allocator;
    const hist = [_]u32{ 10, 10, 10, 10 };
    var th = try quantizeHistogram(gpa, &hist, 2);
    defer th.deinit(gpa);
    // 2 chunks: порог на границе половины суммы.
    try testing.expect(th.items.len <= 1);
}

test "quantizeSamples: базовый случай" {
    const gpa = testing.allocator;
    const samples = [_]pixel_type{ -10, 0, 10, 20, 30, 40, 50, 60 };
    var th = try quantizeSamples(gpa, &samples, 4);
    defer th.deinit(gpa);
    // Пороги внутри диапазона сэмплов.
    for (th.items) |v| {
        try testing.expect(v >= -10 and v <= 60);
    }
}
