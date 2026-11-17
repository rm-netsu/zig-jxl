//! Порт `lib/jxl/modular/options.h` (типы свойств и предикторов) и
//! констант из `lib/jxl/modular/encoding/context_predict.h`.

const std = @import("std");
const common = @import("common.zig");

pub const pixel_type = common.pixel_type;
pub const pixel_type_w = common.pixel_type_w;

/// Порт `PropertyVal` / `Properties` (options.h).
pub const PropertyVal = i32;
/// Свойства — изменяемый срез, принадлежащий вызывающему коду.
pub const Properties = []PropertyVal;

/// Порт `enum class Predictor` (options.h).
pub const Predictor = enum(u8) {
    zero = 0,
    left = 1,
    top = 2,
    average0 = 3,
    select = 4,
    gradient = 5,
    weighted = 6,
    top_right = 7,
    top_left = 8,
    left_left = 9,
    average1 = 10,
    average2 = 11,
    average3 = 12,
    average4 = 13,
    // Следующие предикторы — только для энкодера.
    best = 14, // лучший из Gradient и Weighted
    variable = 15, // построить лучшее дерево по предикторам
    _,

    pub const undefined_predictor: Predictor = @enumFromInt(0xFF);
};

/// kNumModularPredictors = Average4 + 1.
pub const kNumModularPredictors: usize = @intFromEnum(Predictor.average4) + 1;
/// kNumModularEncoderPredictors = Variable + 1.
pub const kNumModularEncoderPredictors: usize = @intFromEnum(Predictor.variable) + 1;

/// Порт `kNumStaticProperties = 2` (channel, group_id).
pub const kNumStaticProperties = 2;

/// Порт `StaticPropRange`.
pub const StaticPropRange = [kNumStaticProperties][2]u32;

/// Порт `ModularMultiplierInfo`.
pub const ModularMultiplierInfo = struct {
    range: StaticPropRange,
    multiplier: u32,
};

// --- Константы контекстного предсказания (context_predict.h) ---

/// Число свойств на ссылочный канал (|v|, v, |v-pred|, v-pred).
pub const kExtraPropsPerChannel = 4;
/// kNumNonrefProperties = kNumStaticProperties + 13 + weighted::kNumProperties.
pub const kNumNonrefProperties = kNumStaticProperties + 13 + 1;
/// Индекс свойства взвешенного предиктора (макс. |ошибка| соседей).
pub const kWPProp: usize = kNumNonrefProperties - 1; // 15
/// Индекс "локального градиента" (left + top - topleft).
pub const kGradientProp: usize = 9;

/// Порт `ModularOptions::TreeMode`.
pub const TreeMode = enum {
    gradient_only,
    wp_only,
    no_wp,
    default,
};

/// Порт `ModularOptions` (сокращённый: только поля, нужные enc_ma).
pub const ModularOptions = struct {
    /// Останов при канале больше этого размера (используется при сборе
    /// образцов, CollectPixelSamples).
    max_chan_size: usize = 0xFFFFFF,
    /// Доля пикселей, просматриваемых для обучения MA-дерева.
    nb_repeats: f32 = 0.5,
    /// Максимум свойств предыдущих каналов.
    max_properties: u32 = 0,
    /// Альтернативные эвристики; по умолчанию {0, 1, 15, 9, 10, 11, 12, 13}.
    splitting_heuristics_properties: []const u32 = &.{ 0, 1, 15, 9, 10, 11, 12, 13 },
    splitting_heuristics_node_threshold: f32 = 96,
    max_property_values: usize = 32,
    /// Предиктор для каждого канала.
    predictor: Predictor = Predictor.undefined_predictor,
    wp_mode: i32 = 0,
    fast_decode_multiplier: f32 = 1.01,
    wp_tree_mode: TreeMode = .default,
};

test "Predictor: значения перечисления совпадают с C++" {
    try std.testing.expectEqual(@as(u8, 0), @intFromEnum(Predictor.zero));
    try std.testing.expectEqual(@as(u8, 1), @intFromEnum(Predictor.left));
    try std.testing.expectEqual(@as(u8, 2), @intFromEnum(Predictor.top));
    try std.testing.expectEqual(@as(u8, 3), @intFromEnum(Predictor.average0));
    try std.testing.expectEqual(@as(u8, 4), @intFromEnum(Predictor.select));
    try std.testing.expectEqual(@as(u8, 5), @intFromEnum(Predictor.gradient));
    try std.testing.expectEqual(@as(u8, 6), @intFromEnum(Predictor.weighted));
    try std.testing.expectEqual(@as(u8, 7), @intFromEnum(Predictor.top_right));
    try std.testing.expectEqual(@as(u8, 8), @intFromEnum(Predictor.top_left));
    try std.testing.expectEqual(@as(u8, 9), @intFromEnum(Predictor.left_left));
    try std.testing.expectEqual(@as(u8, 10), @intFromEnum(Predictor.average1));
    try std.testing.expectEqual(@as(u8, 11), @intFromEnum(Predictor.average2));
    try std.testing.expectEqual(@as(u8, 12), @intFromEnum(Predictor.average3));
    try std.testing.expectEqual(@as(u8, 13), @intFromEnum(Predictor.average4));
    try std.testing.expectEqual(@as(u8, 14), @intFromEnum(Predictor.best));
    try std.testing.expectEqual(@as(u8, 15), @intFromEnum(Predictor.variable));
    try std.testing.expectEqual(@as(usize, 14), kNumModularPredictors);
    try std.testing.expectEqual(@as(usize, 16), kNumNonrefProperties);
    try std.testing.expectEqual(@as(usize, 15), kWPProp);
}
