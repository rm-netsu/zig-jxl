//! Порт `lib/jxl/modular/encoding/dec_ma.h` и `dec_ma.cc` — декодирование
//! MA-деревьев (meta-adaptive context modeling) JPEG XL.
//!
//! Дерево читается в BFS-порядке: свойство, затем для листа — предиктор,
//! смещение и множитель, для внутреннего узла — порог splitval.

const std = @import("std");
const options = @import("options.zig");
const common = @import("common.zig");
const bits = @import("../base/bits.zig");
const BitReader = @import("../base/bit_io.zig").BitReader;

pub const PropertyVal = options.PropertyVal;
pub const Predictor = options.Predictor;

/// Порт `PropertyDecisionNode` (dec_ma.h).
pub const PropertyDecisionNode = struct {
    splitval: PropertyVal = 0,
    /// -1: лист; иначе индекс свойства. lchild указывает на лист-узел.
    property: i16 = -1,
    lchild: u32 = 0,
    rchild: u32 = 0,
    predictor: Predictor = .zero,
    predictor_offset: i64 = 0,
    multiplier: u32 = 1,

    /// PropertyDecisionNode::Leaf.
    pub fn leaf(predictor: Predictor, offset: i64, multiplier: u32) PropertyDecisionNode {
        return .{
            .property = -1,
            .splitval = 0,
            .lchild = 0,
            .rchild = 0,
            .predictor = predictor,
            .predictor_offset = offset,
            .multiplier = multiplier,
        };
    }

    /// PropertyDecisionNode::Split. При rchild == null берётся lchild + 1.
    pub fn split(p: i32, split_val: i32, lchild: u32, rchild_opt: ?u32) PropertyDecisionNode {
        const r = rchild_opt orelse (lchild + 1);
        return .{
            .property = @intCast(p),
            .splitval = split_val,
            .lchild = lchild,
            .rchild = r,
            .predictor = .zero,
            .predictor_offset = 0,
            .multiplier = 1,
        };
    }
};

/// Порт `using Tree = std::vector<PropertyDecisionNode>`.
pub const Tree = std.ArrayList(PropertyDecisionNode);

/// Порт `enum MATreeContext` (ma_common.h).
pub const MATreeContext = enum(usize) {
    split_val = 0,
    property = 1,
    predictor = 2,
    offset = 3,
    multiplier_log = 4,
    multiplier_bits = 5,
};

pub const kNumTreeContexts = 6;
/// Порт `kMaxTreeSize` (ma_common.h).
pub const kMaxTreeSize: usize = 1 << 22;

const NextAction = enum { check_and_go_left, go_right, pop };

const WorkItem = struct {
    node_index: usize,
    orig_l: common.pixel_type,
    orig_u: common.pixel_type,
    action: NextAction,
};

/// Порт `ValidateTree` (dec_ma.cc): проверка непротиворечивости диапазонов
/// свойств и высоты дерева.
pub fn validateTree(gpa: std.mem.Allocator, tree: []const PropertyDecisionNode) bool {
    if (tree.len == 0) return true;

    var num_properties: i32 = 0;
    for (tree) |node| {
        if (node.property >= num_properties) {
            num_properties = node.property + 1;
        }
    }

    const prop_range_min = std.math.minInt(common.pixel_type);
    const prop_range_max = std.math.maxInt(common.pixel_type);

    const Range = struct { first: common.pixel_type, second: common.pixel_type };
    const property_ranges = gpa.alloc(Range, @intCast(num_properties)) catch return false;
    defer gpa.free(property_ranges);
    for (property_ranges) |*r| r.* = .{ .first = prop_range_min, .second = prop_range_max };

    const kHeightLimit = 2048;

    var stack: std.ArrayList(WorkItem) = .empty;
    defer stack.deinit(gpa);
    stack.append(gpa, .{
        .node_index = 0,
        .orig_l = 0,
        .orig_u = 0,
        .action = .check_and_go_left,
    }) catch return false;

    while (stack.items.len > 0) {
        if (stack.items.len >= kHeightLimit) return false; // Tree too tall
        const item = &stack.items[stack.items.len - 1];
        const node = tree[item.node_index];
        switch (item.action) {
            .check_and_go_left => {
                const p = node.property;
                if (p == -1) {
                    _ = stack.pop();
                    continue;
                }
                const v: PropertyVal = node.splitval;
                const l = property_ranges[@intCast(p)].first;
                const u = property_ranges[@intCast(p)].second;
                if (l > v or u <= v) {
                    return false; // Invalid tree
                }
                item.orig_l = l;
                item.orig_u = u;
                item.action = .go_right;
                property_ranges[@intCast(node.property)].first = node.splitval + 1;
                stack.append(gpa, .{
                    .node_index = node.lchild,
                    .orig_l = 0,
                    .orig_u = 0,
                    .action = .check_and_go_left,
                }) catch return false;
                continue;
            },
            .go_right => {
                item.action = .pop;
                property_ranges[@intCast(node.property)].first = item.orig_l;
                property_ranges[@intCast(node.property)].second = node.splitval;
                stack.append(gpa, .{
                    .node_index = node.rchild,
                    .orig_l = 0,
                    .orig_u = 0,
                    .action = .check_and_go_left,
                }) catch return false;
                continue;
            },
            .pop => {
                property_ranges[@intCast(node.property)].second = item.orig_u;
                _ = stack.pop();
                continue;
            },
        }
    }

    return true;
}

/// Порт внутреннего `DecodeTree` (dec_ma.cc, анонимное пространство имён).
/// `reader` — любой тип с методами:
///   readHybridUint(ctx: usize, br: *BitReader) u32
///   checkFinalState() bool
///   isPropertyContextDegenerate() bool
pub fn decodeTreeInner(
    br: *BitReader,
    reader: anytype,
    tree: *Tree,
    gpa: std.mem.Allocator,
    tree_size_limit: usize,
) !void {
    var leaf_id: usize = 0;
    var to_decode: usize = 1;
    tree.clearRetainingCapacity();
    while (to_decode > 0) {
        if (!br.allReadsWithinBounds()) return error.OutOfBounds;
        if (tree.items.len > tree_size_limit) return error.TreeTooLarge;
        to_decode -= 1;
        const prop1 = reader.readHybridUint(@intFromEnum(MATreeContext.property), br);
        if (prop1 > 256) return error.InvalidTreeProperty;
        const property: i32 = @as(i32, @intCast(prop1)) - 1;
        if (property == -1) {
            const predictor = reader.readHybridUint(@intFromEnum(MATreeContext.predictor), br);
            if (predictor >= options.kNumModularPredictors) {
                return error.InvalidPredictor;
            }
            const predictor_offset = common.unpackSigned(reader.readHybridUint(@intFromEnum(MATreeContext.offset), br));
            const mul_log = reader.readHybridUint(@intFromEnum(MATreeContext.multiplier_log), br);
            if (mul_log >= 31) return error.InvalidMultiplierLog;
            const mul_bits = reader.readHybridUint(@intFromEnum(MATreeContext.multiplier_bits), br);
            if (mul_bits >= (@as(u32, 1) << @intCast(31 - mul_log)) - 1) {
                return error.InvalidMultiplier;
            }
            const multiplier = (mul_bits +% 1) << @intCast(mul_log);
            try tree.append(gpa, .{
                .property = -1,
                .splitval = 0,
                .lchild = @intCast(leaf_id),
                .rchild = 0,
                .predictor = @enumFromInt(@as(u8, @intCast(predictor))),
                .predictor_offset = predictor_offset,
                .multiplier = multiplier,
            });
            leaf_id += 1;
            continue;
        }
        const splitval = common.unpackSigned(reader.readHybridUint(@intFromEnum(MATreeContext.split_val), br));
        try tree.append(gpa, .{
            .property = @intCast(property),
            .splitval = splitval,
            .lchild = @intCast(tree.items.len + to_decode + 1),
            .rchild = @intCast(tree.items.len + to_decode + 2),
            .predictor = .zero,
            .predictor_offset = 0,
            .multiplier = 1,
        });
        to_decode += 2;
    }
    if (!validateTree(gpa, tree.items)) return error.InvalidTree;
}

/// Порт внешнего `DecodeTree` (dec_ma.cc): проверка вырожденного кода,
/// декодирование и финальная проверка состояния ридера.
/// Сборка гистограмм (DecodeHistograms) — обязанность вызывающего кода:
/// reader уже создан по кодам всех контекстов дерева.
pub fn decodeTree(
    br: *BitReader,
    reader: anytype,
    tree: *Tree,
    gpa: std.mem.Allocator,
    tree_size_limit: usize,
) !void {
    if (reader.isPropertyContextDegenerate()) return error.InfiniteTree;
    try decodeTreeInner(br, reader, tree, gpa, @min(tree_size_limit, kMaxTreeSize));
    if (!reader.checkFinalState()) return error.ANSFinalState;
}

// ---------------------------------------------------------------------------
// Тесты
// ---------------------------------------------------------------------------

const testing = std.testing;
const MockSymbolReader = @import("symbol_reader.zig").MockSymbolReader;

test "decodeTree: одиночный лист" {
    const gpa = testing.allocator;
    // property=0 (лист), predictor=Gradient(5), offset=PackSigned(3)=6,
    // mul_log=2, mul_bits=1 -> multiplier = (1+1)<<2 = 8.
    const seqs = [_][]const u32{
        &.{}, // split_val (не используется)
        &.{0}, // property -> 0 = лист
        &.{5}, // predictor -> Gradient
        &.{6}, // offset
        &.{2}, // mul_log
        &.{1}, // mul_bits
    };
    var reader = try MockSymbolReader.init(gpa, &seqs);
    defer reader.deinit(gpa);
    var tree = Tree.empty;
    defer tree.deinit(gpa);

    var br = BitReader.init(&.{});
    try decodeTree(&br, &reader, &tree, gpa, 1 << 20);
    try testing.expectEqual(@as(usize, 1), tree.items.len);
    try testing.expectEqual(@as(i16, -1), tree.items[0].property);
    try testing.expectEqual(Predictor.gradient, tree.items[0].predictor);
    try testing.expectEqual(@as(i64, 3), tree.items[0].predictor_offset);
    try testing.expectEqual(@as(u32, 8), tree.items[0].multiplier);
}

test "decodeTree: дерево с одним расщеплением" {
    const gpa = testing.allocator;
    // BFS: корень (property=0 -> свойство 0, splitval=10), потомки: лист, лист.
    const seqs = [_][]const u32{
        &.{common.packSigned(10)}, // split_val
        &.{ 1, 0, 0 }, // property: корень -> 0; лист; лист
        &.{ 0, 1 }, // predictor: Zero; Left(1)
        &.{ 0, common.packSigned(-2) }, // offset: 0; -2
        &.{ 0, 3 }, // mul_log
        &.{ 0, 0 }, // mul_bits -> multiplier 1; 8
    };
    var reader = try MockSymbolReader.init(gpa, &seqs);
    defer reader.deinit(gpa);
    var tree = Tree.empty;
    defer tree.deinit(gpa);

    var br = BitReader.init(&.{});
    try decodeTree(&br, &reader, &tree, gpa, 1 << 20);
    try testing.expectEqual(@as(usize, 3), tree.items.len);
    // Корень: внутренний узел, свойство 0, splitval 10, дети 1 и 2.
    try testing.expectEqual(@as(i16, 0), tree.items[0].property);
    try testing.expectEqual(@as(i32, 10), tree.items[0].splitval);
    try testing.expectEqual(@as(u32, 1), tree.items[0].lchild);
    try testing.expectEqual(@as(u32, 2), tree.items[0].rchild);
    // Первый декодированный лист: Zero (идёт в lchild).
    try testing.expectEqual(@as(i16, -1), tree.items[1].property);
    try testing.expectEqual(Predictor.zero, tree.items[1].predictor);
    // Второй лист: Left, offset -2, multiplier 8.
    try testing.expectEqual(Predictor.left, tree.items[2].predictor);
    try testing.expectEqual(@as(i64, -2), tree.items[2].predictor_offset);
    try testing.expectEqual(@as(u32, 8), tree.items[2].multiplier);
}

test "decodeTree: некорректное свойство" {
    const gpa = testing.allocator;
    const seqs = [_][]const u32{
        &.{},
        &.{258}, // property > 256
    };
    var reader = try MockSymbolReader.init(gpa, &seqs);
    defer reader.deinit(gpa);
    var tree = Tree.empty;
    defer tree.deinit(gpa);
    var br = BitReader.init(&.{});
    try testing.expectError(error.InvalidTreeProperty, decodeTree(&br, &reader, &tree, gpa, 1 << 20));
}

test "decodeTree: некорректный предиктор" {
    const gpa = testing.allocator;
    const seqs = [_][]const u32{
        &.{},
        &.{0}, // лист
        &.{14}, // predictor 14 >= kNumModularPredictors
    };
    var reader = try MockSymbolReader.init(gpa, &seqs);
    defer reader.deinit(gpa);
    var tree = Tree.empty;
    defer tree.deinit(gpa);
    var br = BitReader.init(&.{});
    try testing.expectError(error.InvalidPredictor, decodeTree(&br, &reader, &tree, gpa, 1 << 20));
}

test "validateTree: противоречивые диапазоны отвергаются" {
    // Дерево: корень property 0 splitval 0; lchild: property 0 splitval 0
    // (диапазон слева [min, 0] -> splitval 0 недопустим: u <= v).
    const tree = [_]PropertyDecisionNode{
        PropertyDecisionNode.split(0, 0, 1, 3),
        PropertyDecisionNode.split(0, 0, 2, 2), // невалидно на левой ветке
        PropertyDecisionNode.leaf(.zero, 0, 1),
        PropertyDecisionNode.leaf(.zero, 0, 1),
    };
    try testing.expect(!validateTree(testing.allocator, &tree));

    // Валидный вариант: на левой ветке splitval -1.
    const tree_ok = [_]PropertyDecisionNode{
        PropertyDecisionNode.split(0, 0, 1, 3),
        PropertyDecisionNode.split(0, 1, 2, 2),
        PropertyDecisionNode.leaf(.zero, 0, 1),
        PropertyDecisionNode.leaf(.zero, 0, 1),
    };
    try testing.expect(validateTree(testing.allocator, &tree_ok));
}

test "PropertyDecisionNode: split по умолчанию rchild = lchild + 1" {
    const n = PropertyDecisionNode.split(3, 7, 5, null);
    try testing.expectEqual(@as(u32, 5), n.lchild);
    try testing.expectEqual(@as(u32, 6), n.rchild);
}
