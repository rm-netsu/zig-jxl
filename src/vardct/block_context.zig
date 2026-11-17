//! VarDCT block context map, port of libjxl `ac_context.h`
//! (`BlockCtxMap`) plus the bit serialization (`entropy_coder.cc`
//! DecodeBlockCtxMap / `enc_context_map.cc` EncodeBlockCtxMap).

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const common = @import("../modular/common.zig");
const enc_context_map = @import("../entropy/enc_context_map.zig");
const dec_ans = @import("../entropy/dec_ans.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const U32Enc = fields.U32Enc;
const bitsDistr = fields.bitsDistr;
const bitsOffset = fields.bitsOffset;

/// Number of coefficient-order buckets (libjxl `kNumOrders`).
pub const kNumOrders: usize = 13;

/// libjxl `kDefaultCtxMap`: default context map for the 3 x kNumOrders
/// (single DC context) layout.
pub const kDefaultCtxMap = [3 * kNumOrders]u8{
    0, 1, 2, 2, 3,  3,  4,  5,  6,  6,  6,  6,  6,
    7, 8, 9, 9, 10, 11, 12, 13, 14, 14, 14, 14, 14,
    7, 8, 9, 9, 10, 11, 12, 13, 14, 14, 14, 14, 14,
};

/// libjxl `kDCThresholdDist` / `kQFThresholdDist`.
const dc_threshold_dist = U32Enc.init(
    bitsDistr(4),
    bitsOffset(8, 16),
    bitsOffset(16, 272),
    bitsOffset(32, 65808),
);
const qf_threshold_dist = U32Enc.init(
    bitsDistr(2),
    bitsOffset(3, 4),
    bitsOffset(5, 12),
    bitsOffset(8, 44),
);

/// libjxl `kNonZeroBuckets` / `kZeroDensityContextCount`.
pub const kNonZeroBuckets: usize = 37;
pub const kZeroDensityContextCount: usize = 458;

/// libjxl `kCoeffFreqContext` (64 entries).
const kCoeffFreqContext = [64]u16{
    0xBAD, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14,
    15,    15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22,
    23,    23, 23, 23, 24, 24, 24, 24, 25, 25, 25, 25, 26, 26, 26, 26,
    27,    27, 27, 27, 28, 28, 28, 28, 29, 29, 29, 29, 30, 30, 30, 30,
};

/// libjxl `kCoeffNumNonzeroContext` (64 entries).
const kCoeffNumNonzeroContext = [64]u16{
    0xBAD, 0,   31,  62,  62,  93,  93,  93,  93,  123, 123, 123, 123,
    152,   152, 152, 152, 152, 152, 152, 152, 180, 180, 180, 180, 180,
    180,   180, 180, 180, 180, 180, 180, 206, 206, 206, 206, 206, 206,
    206,   206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206,
    206,   206, 206, 206, 206, 206, 206, 206, 206, 206, 206, 206,
};

/// libjxl `ZeroDensityContext`.
pub fn zeroDensityContext(
    nonzeros_left: usize,
    k: usize,
    covered_blocks: usize,
    log2_covered_blocks: usize,
    prev: usize,
) usize {
    const nl = (nonzeros_left + covered_blocks - 1) >> @intCast(log2_covered_blocks);
    const kk = k >> @intCast(log2_covered_blocks);
    return (@as(usize, kCoeffNumNonzeroContext[nl]) +
        kCoeffFreqContext[kk]) * 2 + prev;
}

/// libjxl `PredictFromTopAndLeft`.
pub fn predictFromTopAndLeft(
    row_top: ?[]const i32,
    row: []const i32,
    x: usize,
    default_val: i32,
) i32 {
    if (x == 0) {
        return if (row_top) |rt| rt[x] else default_val;
    }
    if (row_top == null) {
        return row[x - 1];
    }
    return @divTrunc(row_top.?[x] + row[x - 1] + 1, 2);
}

pub const BlockCtxMap = struct {
    dc_thresholds: [3]std.ArrayList(i32) = .{ .empty, .empty, .empty },
    qf_thresholds: std.ArrayList(u32) = .empty,
    ctx_map: std.ArrayList(u8) = .empty,
    num_ctxs: usize = 15,
    num_dc_ctxs: usize = 1,
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) BlockCtxMap {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *BlockCtxMap) void {
        for (&self.dc_thresholds) |*t| t.deinit(self.allocator);
        self.qf_thresholds.deinit(self.allocator);
        self.ctx_map.deinit(self.allocator);
    }

    pub fn setDefault(self: *BlockCtxMap) !void {
        for (&self.dc_thresholds) |*t|
            t.clearRetainingCapacity();
        self.qf_thresholds.clearRetainingCapacity();
        self.ctx_map.clearRetainingCapacity();
        try self.ctx_map.appendSlice(
            self.allocator,
            &kDefaultCtxMap,
        );
        self.num_ctxs = 15;
        self.num_dc_ctxs = 1;
    }

    pub fn context(
        self: *const BlockCtxMap,
        dc_idx: usize,
        qf: u32,
        ord: usize,
        c: usize,
    ) usize {
        var qf_idx: usize = 0;
        for (self.qf_thresholds.items) |t| {
            if (qf > t) qf_idx += 1;
        }
        var idx: usize = if (c < 2) c ^ 1 else 2;
        idx = idx * kNumOrders + ord;
        idx = idx * (self.qf_thresholds.items.len + 1) + qf_idx;
        idx = idx * self.num_dc_ctxs + dc_idx;
        return self.ctx_map.items[idx];
    }

    pub fn zeroDensityContextsOffset(
        self: *const BlockCtxMap,
        block_ctx: usize,
    ) usize {
        return self.num_ctxs * kNonZeroBuckets +
            kZeroDensityContextCount * block_ctx;
    }

    pub fn numACContexts(self: *const BlockCtxMap) usize {
        return self.num_ctxs * (kNonZeroBuckets + kZeroDensityContextCount);
    }

    pub fn nonZeroContext(
        self: *const BlockCtxMap,
        non_zeros: usize,
        block_ctx: usize,
    ) usize {
        var nz = non_zeros;
        if (nz >= 64) nz = 64;
        const ctx: usize = if (nz < 8) nz else 4 + nz / 2;
        return ctx * self.num_ctxs + block_ctx;
    }

    /// Port of `DecodeBlockCtxMap`.
    pub fn read(self: *BlockCtxMap, br: *BitReader) !void {
        for (&self.dc_thresholds) |*t|
            t.clearRetainingCapacity();
        self.qf_thresholds.clearRetainingCapacity();
        self.ctx_map.clearRetainingCapacity();

        const is_default = br.readBits(1) != 0;
        if (is_default) {
            try self.setDefault();
            return;
        }
        self.num_dc_ctxs = 1;
        for (&self.dc_thresholds) |*t| {
            const count: usize = @intCast(br.readBits(4));
            self.num_dc_ctxs *= count + 1;
            try t.resize(self.allocator, count);
            for (t.items) |*v| {
                const packed_val = U32Coder.read(
                    dc_threshold_dist,
                    br,
                );
                v.* = common.unpackSigned(packed_val);
            }
        }
        const qft_count: usize = @intCast(br.readBits(4));
        try self.qf_thresholds.resize(self.allocator, qft_count);
        for (self.qf_thresholds.items) |*v| {
            v.* = U32Coder.read(qf_threshold_dist, br) + 1;
        }
        if (self.num_dc_ctxs * (self.qf_thresholds.items.len + 1) > 64) {
            return error.InvalidBlockCtxMap;
        }
        try self.ctx_map.resize(
            self.allocator,
            3 * kNumOrders * self.num_dc_ctxs *
                (self.qf_thresholds.items.len + 1),
        );
        var num_ctxs: usize = 0;
        try dec_ans.decodeContextMap(
            self.allocator,
            &self.ctx_map,
            &num_ctxs,
            br,
        );
        if (num_ctxs == 0 or num_ctxs > 16) {
            return error.InvalidBlockCtxMap;
        }
        self.num_ctxs = num_ctxs;
    }

    /// Port of `EncodeBlockCtxMap`.
    pub fn write(self: *const BlockCtxMap, writer: *BitWriter) !void {
        const is_default = self.dc_thresholds[0].items.len == 0 and
            self.dc_thresholds[1].items.len == 0 and
            self.dc_thresholds[2].items.len == 0 and
            self.qf_thresholds.items.len == 0 and
            self.ctx_map.items.len == kDefaultCtxMap.len and
            std.mem.eql(u8, self.ctx_map.items, &kDefaultCtxMap);
        if (is_default) {
            writer.write(1, 1);
            return;
        }
        writer.write(1, 0);
        for (self.dc_thresholds) |t| {
            writer.write(4, @intCast(t.items.len));
            for (t.items) |v| {
                try U32Coder.write(
                    dc_threshold_dist,
                    common.packSigned(v),
                    writer,
                );
            }
        }
        writer.write(4, @intCast(self.qf_thresholds.items.len));
        for (self.qf_thresholds.items) |v| {
            try U32Coder.write(qf_threshold_dist, v - 1, writer);
        }
        try enc_context_map.encodeContextMap(
            self.allocator,
            self.ctx_map.items,
            self.num_ctxs,
            writer,
        );
    }
};

test "block ctx map roundtrip (jpeg thresholds)" {
    const alloc = std.testing.allocator;
    var m = BlockCtxMap.init(alloc);
    defer m.deinit();
    try m.dc_thresholds[1].append(alloc, -3);
    try m.dc_thresholds[1].append(alloc, 12);
    try m.dc_thresholds[1].append(alloc, 100);
    m.num_dc_ctxs = 4;
    const total = 3 * kNumOrders * 4;
    try m.ctx_map.ensureTotalCapacity(alloc, total);
    for (0..total) |i| {
        try m.ctx_map.append(alloc, @intCast(@min(i % 9, 8)));
    }
    m.num_ctxs = 9;

    var w = BitWriter.init(alloc);
    defer w.deinit();
    try m.write(&w);
    w.zeroPadToByte();

    var m2 = BlockCtxMap.init(alloc);
    defer m2.deinit();
    var br = BitReader.init(w.getSpan());
    try m2.read(&br);
    try std.testing.expectEqualSlices(
        i32,
        m.dc_thresholds[1].items,
        m2.dc_thresholds[1].items,
    );
    try std.testing.expectEqualSlices(
        u8,
        m.ctx_map.items,
        m2.ctx_map.items,
    );
    try std.testing.expectEqual(m.num_ctxs, m2.num_ctxs);
    try std.testing.expectEqual(m.num_dc_ctxs, m2.num_dc_ctxs);
}
