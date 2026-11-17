//! VarDCT AC coefficient coding, port of libjxl `enc_entropy_coder.cc`
//! `TokenizeCoefficients` and `dec_group.cc` `DecodeACVarBlock`
//! specialized to 8x8 DCT blocks (the only strategy JPEG transcoding
//! uses; larger strategies are rejected by the caller).

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const common = @import("../modular/common.zig");
const dec_ans = @import("../entropy/dec_ans.zig");
const ma_encode = @import("../modular/ma_encode.zig");

const block_context = @import("block_context.zig");

const BitReader = bit_io.BitReader;
const Token = ma_encode.Token;
const BlockCtxMap = block_context.BlockCtxMap;

pub const kDCTBlockSize: usize = 64;

/// The per-group nzeros cache: one 32x32 block grid per channel; channel
/// planes are row-major with stride 32.
pub const GroupCoeffCache = struct {
    nzeros: [3][]i32 = .{ &.{}, &.{}, &.{} },
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !GroupCoeffCache {
        var cache = GroupCoeffCache{
            .allocator = allocator,
        };
        for (&cache.nzeros) |*plane| {
            plane.* = try allocator.alloc(i32, 32 * 32);
            @memset(plane.*, 0);
        }
        return cache;
    }

    pub fn deinit(self: *GroupCoeffCache) void {
        for (&self.nzeros) |plane| self.allocator.free(plane);
        self.nzeros = .{ &.{}, &.{}, &.{} };
    }
};

/// Number of non-zero coefficients of an 8x8 block, excluding the DC
/// (`NumNonZero8x8ExceptDC`); also records the value for the block.
fn numNonZero8x8ExceptDC(block: []const i32, nzeros_pos: *i32) i32 {
    var count: i32 = 0;
    // The DC (offset 0) is excluded, exactly as the masked first row of
    // libjxl's NumNonZero8x8ExceptDC.
    for (1..kDCTBlockSize) |k| {
        if (block[k] != 0) count += 1;
    }
    nzeros_pos.* = count;
    return count;
}

/// Encoder-side view of the quantized coefficient planes of a frame.
pub const GroupCoefficients = struct {
    /// Per-channel blocks: block (bx, by) at offset (by * width + bx) * 64,
    /// values in JXL transposed order (block[8*x + y] = JPEG coefficient
    /// at (y, x)).
    planes: [3][]const i32 = .{ &.{}, &.{}, &.{} },
    /// Per-channel width in blocks (xsize_blocks >> hshift).
    widths: [3]usize = .{ 0, 0, 0 },

    pub fn block(
        self: *const GroupCoefficients,
        c: usize,
        sbx: usize,
        sby: usize,
    ) []const i32 {
        const off = (sby * self.widths[c] + sbx) * kDCTBlockSize;
        return self.planes[c][off .. off + kDCTBlockSize];
    }
};

/// Converts a group-local channel block coordinate to the global one
/// (group origins are multiples of 32 blocks, so the shifts commute).
pub fn globalBlock(
    hs: usize,
    vs: usize,
    rect: BlockRect,
    sbx: usize,
    sby: usize,
) [2]usize {
    return .{
        (rect.x0 >> @intCast(hs)) + sbx,
        (rect.y0 >> @intCast(vs)) + sby,
    };
}

/// Rect of a group expressed in (full-grid) blocks.
pub const BlockRect = struct {
    x0: usize,
    y0: usize,
    xsize: usize,
    ysize: usize,
};

/// Per-block DC bucket image accessor (full grid, row-major).
pub const QdcImage = struct {
    data: []const u8,
    stride: usize,

    pub fn at(self: *const QdcImage, x: usize, y: usize) u8 {
        return self.data[y * self.stride + x];
    }
};

/// Port of `TokenizeCoefficients` for one group (DCT-8 only, single
/// pass). `orders` holds one scan order per color channel (custom orders
/// when the AC global section transmits them, natural zig-zag otherwise).
pub fn tokenizeCoefficients(
    allocator: std.mem.Allocator,
    orders: [3][]const u32,
    rect: BlockRect,
    coeffs: *const GroupCoefficients,
    hshift: [3]usize,
    vshift: [3]usize,
    cache: *GroupCoeffCache,
    qdc: *const QdcImage,
    block_ctx_map: *const BlockCtxMap,
    output: *std.ArrayList(Token),
) !void {
    const xsize_blocks = rect.xsize;
    const ysize_blocks = rect.ysize;
    const nzeros_stride: usize = 32;

    for (0..ysize_blocks) |by| {
        const sby = [3]usize{
            by >> @intCast(vshift[0]),
            by >> @intCast(vshift[1]),
            by >> @intCast(vshift[2]),
        };
        for (0..xsize_blocks) |bx| {
            const sbx = [3]usize{
                bx >> @intCast(hshift[0]),
                bx >> @intCast(hshift[1]),
                bx >> @intCast(hshift[2]),
            };
            // Channel order Y, X, B (libjxl `for (int c : {1, 0, 2})`).
            for ([3]usize{ 1, 0, 2 }) |c| {
                if ((sbx[c] << @intCast(hshift[c])) != bx) continue;
                if ((sby[c] << @intCast(vshift[c])) != by) continue;

                const gb = globalBlock(
                    hshift[c],
                    vshift[c],
                    rect,
                    sbx[c],
                    sby[c],
                );
                const block = coeffs.block(
                    c,
                    gb[0],
                    gb[1],
                );
                const order = orders[c][0..kDCTBlockSize];
                var nzeros: i32 = 0;
                {
                    const nz_pos = &cache.nzeros[c][sby[c] * nzeros_stride + sbx[c]];
                    nzeros = numNonZero8x8ExceptDC(
                        block,
                        nz_pos,
                    );
                }

                const ord: usize = 0; // kStrategyOrder[DCT] == 0

                // Predicted nzeros from top and left, in the channel's own
                // (subsampled) grid.
                const row = cache.nzeros[c][sby[c] * nzeros_stride ..][0..32];
                const top: ?[]const i32 = if (sby[c] == 0)
                    null
                else
                    cache.nzeros[c][(sby[c] - 1) * nzeros_stride ..][0..32];
                const predicted_nzeros = block_context.predictFromTopAndLeft(
                    top,
                    row,
                    sbx[c],
                    32,
                );
                // Per-block raw quant field is constant 1 for JPEG transcode.
                const qf: u32 = 1;
                const block_ctx = block_ctx_map.context(
                    qdc.at(rect.x0 + bx, rect.y0 + by),
                    qf,
                    ord,
                    c,
                );
                const nzero_ctx = block_ctx_map.nonZeroContext(
                    @intCast(@max(predicted_nzeros, 0)),
                    block_ctx,
                );

                try output.append(
                    allocator,
                    .{
                        .context = @intCast(nzero_ctx),
                        .value = @intCast(nzeros),
                    },
                );

                const histo_offset = block_ctx_map.zeroDensityContextsOffset(
                    block_ctx,
                );
                var prev: usize = if (nzeros > @as(i32, @intCast(kDCTBlockSize / 16))) 0 else 1;
                var nz_remaining: usize = @intCast(nzeros);
                var k: usize = 1; // covered_blocks == 1
                while (k < kDCTBlockSize and nz_remaining != 0) : (k += 1) {
                    const coeff = block[order[k]];
                    const ctx = histo_offset + block_context.zeroDensityContext(
                        nz_remaining,
                        k,
                        1,
                        0,
                        prev,
                    );
                    try output.append(
                        allocator,
                        .{
                            .context = @intCast(ctx),
                            .value = common.packSigned(coeff),
                        },
                    );
                    prev = @intFromBool(coeff != 0);
                    nz_remaining -= prev;
                }
                std.debug.assert(nz_remaining == 0);
            }
        }
    }
}

/// Port of `DecodeACVarBlock` for 8x8 blocks (single pass). `qdc_row` is
/// the DC-bucket row of the current group row (group-relative slice).
pub fn decodeACVarBlock(
    reader: *dec_ans.ANSSymbolReader,
    br: *BitReader,
    context_map: []const u8,
    orders: []const u32,
    row_nzeros: []i32,
    row_nzeros_top: ?[]const i32,
    c: usize,
    bx: usize,
    lbx: usize,
    qdc_row: []const u8,
    block_ctx_map: *const BlockCtxMap,
    block: *[kDCTBlockSize]i32,
) !void {
    const predicted_nzeros = block_context.predictFromTopAndLeft(
        row_nzeros_top,
        row_nzeros,
        bx,
        32,
    );

    const ord: usize = 0; // kStrategyOrder[DCT]
    const order = orders[0..kDCTBlockSize];

    // qf is constant 1 for JPEG transcode.
    const qf: u32 = 1;
    const block_ctx = block_ctx_map.context(
        qdc_row[lbx],
        qf,
        ord,
        c,
    );
    const nzero_ctx = block_ctx_map.nonZeroContext(
        @intCast(@max(predicted_nzeros, 0)),
        block_ctx,
    );

    const nzeros = reader.readHybridUint(
        nzero_ctx,
        br,
        context_map,
    );
    if (nzeros > kDCTBlockSize - 1) return error.InvalidACNzeros;
    row_nzeros[bx] = @intCast(nzeros);

    const histo_offset = block_ctx_map.zeroDensityContextsOffset(
        block_ctx,
    );
    var prev: usize = if (nzeros > kDCTBlockSize / 16) 0 else 1;
    var nz_remaining: usize = nzeros;
    var k: usize = 1;
    while (k < kDCTBlockSize and nz_remaining != 0) : (k += 1) {
        const ctx = histo_offset + block_context.zeroDensityContext(
            nz_remaining,
            k,
            1,
            0,
            prev,
        );
        const u_coeff = reader.readHybridUint(
            ctx,
            br,
            context_map,
        );
        // Hand-rolled UnpackSigned (avoids UB on negative shifts).
        const magnitude = u_coeff >> 1;
        const neg_sign = (~u_coeff) & 1;
        const coeff: i32 = @bitCast((magnitude ^ (neg_sign -% 1)));
        block[order[k]] += coeff;
        prev = @intFromBool(u_coeff != 0);
        nz_remaining -= prev;
    }
    if (nz_remaining != 0) return error.InvalidACNzeros;
}
