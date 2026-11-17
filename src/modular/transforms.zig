//! Modular transform syntax and the parts needed for decoding:
//! MetaApply (channel layout changes) and the RCT inverse.
//! Ports of libjxl `modular/transform/{transform,rct,palette,squeeze}.cc`.

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const options = @import("options.zig");
const context_predict = @import("context_predict.zig");
const image_mod = @import("image.zig");
const common = @import("common.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const U32Enc = fields.U32Enc;
const SizeWriter = fields.SizeWriter;
const Channel = image_mod.Channel;
const Image = image_mod.Image;
const pixel_type = common.pixel_type;
const pixel_type_w = common.pixel_type_w;
const PixelAdd = common.pixelAdd;

pub const TransformId = enum(u32) {
    rct = 0,
    palette = 1,
    squeeze = 2,
    invalid = 3,
};

const transform_id_enc = U32Enc.init(fields.val(0), fields.val(1), fields.val(2), fields.val(3));
const begin_c_enc = U32Enc.init(fields.bitsDistr(3), fields.bitsOffset(6, 8), fields.bitsOffset(10, 72), fields.bitsOffset(13, 1096));
const rct_type_enc = U32Enc.init(fields.val(6), fields.bitsDistr(2), fields.bitsOffset(4, 2), fields.bitsOffset(6, 10));
const palette_num_c_enc = U32Enc.init(fields.val(1), fields.val(3), fields.val(4), fields.bitsOffset(13, 1));
const nb_colors_enc = U32Enc.init(fields.bitsOffset(8, 0), fields.bitsOffset(10, 256), fields.bitsOffset(12, 1280), fields.bitsOffset(16, 5376));
const nb_deltas_enc = U32Enc.init(fields.val(0), fields.bitsOffset(8, 1), fields.bitsOffset(10, 257), fields.bitsOffset(16, 1281));
const num_squeezes_enc = U32Enc.init(fields.val(0), fields.bitsOffset(4, 1), fields.bitsOffset(6, 9), fields.bitsOffset(8, 41));

pub const SqueezeParams = struct {
    horizontal: bool = false,
    in_place: bool = false,
    begin_c: u32 = 0,
    num_c: u32 = 2,

    const sq_begin_c_enc = U32Enc.init(fields.bitsDistr(3), fields.bitsOffset(6, 8), fields.bitsOffset(10, 72), fields.bitsOffset(13, 1096));
    const sq_num_c_enc = U32Enc.init(fields.val(1), fields.val(2), fields.val(3), fields.bitsOffset(4, 4));

    pub fn read(self: *SqueezeParams, br: *BitReader) !void {
        self.* = .{};
        self.horizontal = (br.readBits(1)) != 0;
        self.in_place = (br.readBits(1)) != 0;
        self.begin_c = U32Coder.read(sq_begin_c_enc, br);
        self.num_c = U32Coder.read(sq_num_c_enc, br);
    }

    pub fn write(self: *const SqueezeParams, writer: *BitWriter) !void {
        writer.write(1, @intFromBool(self.horizontal));
        writer.write(1, @intFromBool(self.in_place));
        try U32Coder.write(sq_begin_c_enc, self.begin_c, writer);
        try U32Coder.write(sq_num_c_enc, self.num_c, writer);
    }
};

pub const Transform = struct {
    id: TransformId = .invalid,
    begin_c: u32 = 0,
    rct_type: u32 = 6,
    num_c: u32 = 3,
    nb_colors: u32 = 256,
    nb_deltas: u32 = 0,
    predictor: u32 = 0, // Predictor::Zero
    ordered_palette: bool = true,
    lossy_palette: bool = false,
    squeezes: std.ArrayList(SqueezeParams) = .empty,
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) Transform {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *Transform) void {
        self.squeezes.deinit(self.allocator);
    }

    pub fn read(self: *Transform, br: *BitReader) !void {
        self.* = .{ .allocator = self.allocator };
        const id_raw = U32Coder.read(transform_id_enc, br);
        if (id_raw > 3) return error.InvalidTransform;
        self.id = @enumFromInt(id_raw);
        if (self.id == .invalid) return error.InvalidTransform;
        if (self.id == .rct or self.id == .palette) {
            self.begin_c = U32Coder.read(begin_c_enc, br);
        }
        if (self.id == .rct) {
            self.rct_type = U32Coder.read(rct_type_enc, br);
            if (self.rct_type >= 42) return error.InvalidTransform;
        }
        if (self.id == .palette) {
            self.num_c = U32Coder.read(palette_num_c_enc, br);
            self.nb_colors = U32Coder.read(nb_colors_enc, br);
            self.nb_deltas = U32Coder.read(nb_deltas_enc, br);
            self.predictor = @intCast(br.readBits(4));
            if (self.predictor >= @intFromEnum(options.Predictor.best)) return error.InvalidTransform;
            // ordered_palette/lossy_palette are not serialized in v0.12.
        }
        if (self.id == .squeeze) {
            const num_squeezes = U32Coder.read(num_squeezes_enc, br);
            try self.squeezes.resize(self.allocator, num_squeezes);
            for (self.squeezes.items) |*sq| {
                try sq.read(br);
            }
        }
    }

    pub fn write(self: *const Transform, writer: *BitWriter) !void {
        try U32Coder.write(transform_id_enc, @intFromEnum(self.id), writer);
        if (self.id == .rct or self.id == .palette) {
            try U32Coder.write(begin_c_enc, self.begin_c, writer);
        }
        if (self.id == .rct) {
            try U32Coder.write(rct_type_enc, self.rct_type, writer);
        }
        if (self.id == .palette) {
            try U32Coder.write(palette_num_c_enc, self.num_c, writer);
            try U32Coder.write(nb_colors_enc, self.nb_colors, writer);
            try U32Coder.write(nb_deltas_enc, self.nb_deltas, writer);
            writer.write(4, self.predictor);
            // ordered_palette/lossy_palette are not serialized in v0.12.
        }
        if (self.id == .squeeze) {
            try U32Coder.write(num_squeezes_enc, @intCast(self.squeezes.items.len), writer);
            for (self.squeezes.items) |*sq| {
                try sq.write(writer);
            }
        }
    }
};

pub fn checkEqualChannels(image: *const Image, c1: usize, c2: usize) !void {
    if (c1 >= image.channels.items.len or c2 >= image.channels.items.len or c2 < c1) {
        return error.InvalidChannelRange;
    }
    const first = &image.channels.items[c1];
    for (image.channels.items[c1 + 1 .. c2 + 1]) |*ch| {
        if (first.w != ch.w or first.h != ch.h or first.hshift != ch.hshift or first.vshift != ch.vshift) {
            return error.InvalidChannelRange;
        }
    }
}

/// Port of MetaPalette: collapses channels into a palette + index channel.
pub fn metaPalette(image: *Image, begin_c: usize, end_c: usize, nb_colors: usize, nb_deltas: usize) !void {
    try checkEqualChannels(image, begin_c, end_c);
    const nb = end_c - begin_c + 1;
    if (begin_c >= image.nb_meta_channels) {
        image.nb_meta_channels += 1;
    } else {
        std.debug.assert(end_c < image.nb_meta_channels);
        image.nb_meta_channels += 2 - nb;
    }
    // Remove channels begin_c+1 .. end_c (freeing their planes).
    for (image.channels.items[begin_c + 1 .. end_c + 1]) |*ch| ch.deinit();
    _ = try image.channels.replaceRange(image.gpa, begin_c + 1, end_c - begin_c, &.{});
    const pch = try Channel.create(image.gpa, nb_colors + nb_deltas, nb, -1, -1);
    try image.channels.insert(image.gpa, 0, pch);
}

/// Port of MetaSqueeze: splits channels into half-size + residual channels.
/// An empty parameter list selects the default squeeze scheme (which is
/// materialized into `t.squeezes`, mirroring libjxl's in-place mutation of
/// the transform, so that the inverse can replay it).
pub fn metaSqueeze(image: *Image, t: *Transform) !void {
    if (t.squeezes.items.len == 0) {
        try defaultSqueezeParametersInto(image, &t.squeezes);
    }
    for (t.squeezes.items) |parameter| {
        try checkMetaSqueezeParams(parameter, image.channels.items.len);
        const horizontal = parameter.horizontal;
        const in_place = parameter.in_place;
        const beginc = parameter.begin_c;
        const endc = parameter.begin_c + parameter.num_c - 1;

        var offset: usize = 0;
        if (beginc < image.nb_meta_channels) {
            if (endc >= image.nb_meta_channels) return error.InvalidSqueeze;
            if (!in_place) return error.InvalidSqueeze;
            image.nb_meta_channels += parameter.num_c;
        }
        if (in_place) {
            offset = endc + 1;
        } else {
            offset = image.channels.items.len;
        }
        var c = beginc;
        while (c <= endc) : (c += 1) {
            const ch = &image.channels.items[c];
            if (ch.hshift > 30 or ch.vshift > 30) return error.TooManySqueezes;
            var w = ch.w;
            var v = ch.h;
            if (w == 0 or v == 0) return error.SqueezingEmptyChannel;
            if (horizontal) {
                ch.w = (w + 1) / 2;
                if (ch.hshift >= 0) ch.hshift += 1;
                w = w - (w + 1) / 2;
            } else {
                ch.h = (v + 1) / 2;
                if (ch.vshift >= 0) ch.vshift += 1;
                v = v - (v + 1) / 2;
            }
            try ch.shrink();
            var placeholder = try Channel.create(image.gpa, w, v, ch.hshift, ch.vshift);
            placeholder.component = ch.component;
            try image.channels.insert(image.gpa, offset + (c - beginc), placeholder);
        }
    }
}

fn checkMetaSqueezeParams(parameter: SqueezeParams, num_channels: usize) !void {
    const c1 = parameter.begin_c;
    const c2 = parameter.begin_c + parameter.num_c - 1;
    if (c1 >= num_channels or c2 >= num_channels or c2 < c1) return error.InvalidChannelRange;
}

const kMaxFirstPreviewSize: usize = 8;

pub fn defaultSqueezeParametersInto(image: *Image, parameters: *std.ArrayList(SqueezeParams)) !void {
    const nb_channels = image.channels.items.len - image.nb_meta_channels;
    parameters.clearRetainingCapacity();
    var w = image.channels.items[image.nb_meta_channels].w;
    var v = image.channels.items[image.nb_meta_channels].h;
    const wide = w > v;

    if (nb_channels > 2 and image.channels.items[image.nb_meta_channels + 1].w == w and
        image.channels.items[image.nb_meta_channels + 1].h == v)
    {
        try parameters.append(image.gpa, .{ .horizontal = true, .in_place = false, .begin_c = @intCast(image.nb_meta_channels + 1), .num_c = 2 });
        try parameters.append(image.gpa, .{ .horizontal = false, .in_place = false, .begin_c = @intCast(image.nb_meta_channels + 1), .num_c = 2 });
    }
    var params = SqueezeParams{ .begin_c = @intCast(image.nb_meta_channels), .num_c = @intCast(nb_channels), .in_place = true };
    if (!wide) {
        if (v > kMaxFirstPreviewSize) {
            params.horizontal = false;
            try parameters.append(image.gpa, params);
            v = (v + 1) / 2;
        }
    }
    while (w > kMaxFirstPreviewSize or v > kMaxFirstPreviewSize) {
        if (w > kMaxFirstPreviewSize) {
            params.horizontal = true;
            try parameters.append(image.gpa, params);
            w = (w + 1) / 2;
        }
        if (v > kMaxFirstPreviewSize) {
            params.horizontal = false;
            try parameters.append(image.gpa, params);
            v = (v + 1) / 2;
        }
    }
}

/// Applies the channel-layout part of a transform (libjxl MetaApply).
pub fn metaApply(image: *Image, t: *Transform) !void {
    switch (t.id) {
        .rct => try checkEqualChannels(image, t.begin_c, t.begin_c + 2),
        .squeeze => try metaSqueeze(image, t),
        .palette => try metaPalette(image, t.begin_c, t.begin_c + t.num_c - 1, t.nb_colors, t.nb_deltas),
        .invalid => return error.InvalidTransform,
    }
}

// ---------------------------------------------------------------------------
// RCT inverse
// ---------------------------------------------------------------------------

/// Permutation of the three input channels into output positions.
fn rctOutSlot(permutation: u32, which: u32) u32 {
    return switch (which) {
        0 => permutation % 3,
        1 => (permutation + 1 + permutation / 3) % 3,
        else => (permutation + 2 - permutation / 3) % 3,
    };
}

fn invRctRow(comptime transform_type: u32, in0: []const pixel_type, in1: []const pixel_type, in2: []const pixel_type, out0: []pixel_type, out1: []pixel_type, out2: []pixel_type) void {
    const second = transform_type >> 1;
    const third = transform_type & 1;
    for (in0, 0..) |_, x| {
        if (transform_type == 6) {
            // YCoCg: in0=Y, in1=Co, in2=Cg.
            const y_in = in0[x];
            const co = in1[x];
            const cg = in2[x];
            const tmp = PixelAdd(y_in, -(cg >> 1));
            const g = PixelAdd(cg, tmp);
            const b = PixelAdd(tmp, -(co >> 1));
            const r = PixelAdd(b, co);
            out0[x] = r;
            out1[x] = g;
            out2[x] = b;
        } else {
            const first = in0[x];
            var sec = in1[x];
            var third_v = in2[x];
            if (third != 0) third_v = PixelAdd(third_v, first);
            if (second == 1) {
                sec = PixelAdd(sec, first);
            } else if (second == 2) {
                sec = PixelAdd(sec, PixelAdd(first, third_v) >> 1);
            }
            out0[x] = first;
            out1[x] = sec;
            out2[x] = third_v;
        }
    }
}

/// Port of InvRCT. Requires three equal-size channels starting at begin_c.
pub fn inverseRCT(image: *Image, begin_c: usize, rct_type: u32) !void {
    try checkEqualChannels(image, begin_c, begin_c + 2);
    const bc = begin_c;
    if (rct_type == 0) return; // noop
    const permutation = rct_type / 7;
    const custom = rct_type % 7;

    const channels = image.channels.items;
    const rows = channels[bc].h;

    if (custom == 0) {
        // Permute-only: swap whole channels around.
        const moved: [3]Channel = .{
            channels[bc + 0],
            channels[bc + 1],
            channels[bc + 2],
        };
        channels[bc + rctOutSlot(permutation, 0)] = moved[0];
        channels[bc + rctOutSlot(permutation, 1)] = moved[1];
        channels[bc + rctOutSlot(permutation, 2)] = moved[2];
        return;
    }

    var y: usize = 0;
    while (y < rows) : (y += 1) {
        const in0: []const pixel_type = channels[bc].row(y);
        const in1: []const pixel_type = channels[bc + 1].row(y);
        const in2: []const pixel_type = channels[bc + 2].row(y);
        const out0: []pixel_type = channels[bc + rctOutSlot(permutation, 0)].row(y);
        const out1: []pixel_type = channels[bc + rctOutSlot(permutation, 1)].row(y);
        const out2: []pixel_type = channels[bc + rctOutSlot(permutation, 2)].row(y);
        switch (custom) {
            1 => invRctRow(1, in0, in1, in2, out0, out1, out2),
            2 => invRctRow(2, in0, in1, in2, out0, out1, out2),
            3 => invRctRow(3, in0, in1, in2, out0, out1, out2),
            4 => invRctRow(4, in0, in1, in2, out0, out1, out2),
            5 => invRctRow(5, in0, in1, in2, out0, out1, out2),
            6 => invRctRow(6, in0, in1, in2, out0, out1, out2),
            else => unreachable,
        }
    }
}

// ---------------------------------------------------------------------------
// Palette inverse
// ---------------------------------------------------------------------------

const kRgbChannels: usize = 3;
const kLargeCube: i32 = 5;
const kSmallCube: i32 = 4;
const kSmallCubeBits: u5 = 2;
const kLargeCubeOffset: i32 = kSmallCube * kSmallCube * kSmallCube;

/// Delta palette for negative indices (libjxl palette.h kDeltaPalette),
/// flattened rows of 3 channels.
const delta_palette = [72][3]pixel_type{
    .{ 0, 0, 0 },       .{ 4, 4, 4 },       .{ 11, 0, 0 },
    .{ 0, 0, -13 },     .{ 0, -12, 0 },     .{ -10, -10, -10 },
    .{ -18, -18, -18 }, .{ -27, -27, -27 }, .{ -18, -18, 0 },
    .{ 0, 0, -32 },     .{ -32, 0, 0 },     .{ -37, -37, -37 },
    .{ 0, -32, -32 },   .{ 24, 24, 45 },    .{ 50, 50, 50 },
    .{ -45, -24, -24 }, .{ -24, -45, -45 }, .{ 0, -24, -24 },
    .{ -34, -34, 0 },   .{ -24, 0, -24 },   .{ -45, -45, -24 },
    .{ 64, 64, 64 },    .{ -32, 0, -32 },   .{ 0, -32, 0 },
    .{ -32, 0, 32 },    .{ -24, -45, -24 }, .{ 45, 24, 45 },
    .{ 24, -24, -45 },  .{ -45, -24, 24 },  .{ 80, 80, 80 },
    .{ 64, 0, 0 },      .{ 0, 0, -64 },     .{ 0, -64, -64 },
    .{ -24, -24, 45 },  .{ 96, 96, 96 },    .{ 64, 64, 0 },
    .{ 45, -24, -24 },  .{ 34, -34, 0 },    .{ 112, 112, 112 },
    .{ 24, -45, -45 },  .{ 45, 45, -24 },   .{ 0, -32, 32 },
    .{ 24, -24, 45 },   .{ 0, 96, 96 },     .{ 45, -24, 24 },
    .{ 24, -45, -24 },  .{ -24, -45, 24 },  .{ 0, -64, 0 },
    .{ 96, 0, 0 },      .{ 128, 128, 128 }, .{ 64, 0, 64 },
    .{ 144, 144, 144 }, .{ 96, 96, 0 },     .{ -36, -36, 36 },
    .{ 45, -24, -45 },  .{ 45, -45, -24 },  .{ 0, 0, -96 },
    .{ 0, 128, 128 },   .{ 0, 96, 0 },      .{ 45, 24, -45 },
    .{ -128, 0, 0 },    .{ 24, -45, 24 },   .{ -45, 24, -45 },
    .{ 64, 0, -64 },    .{ 64, -64, -64 },  .{ 96, 0, 96 },
    .{ 45, -45, 24 },   .{ 24, 45, -45 },   .{ 64, 64, -64 },
    .{ 128, 128, 0 },   .{ 0, 0, -128 },    .{ -24, 45, -45 },
};

fn paletteScale(value: i64, bit_depth: u32) pixel_type {
    // libjxl Scale<4>: (value * ((1 << bit_depth) - 1)) >> 2, computed in
    // uint64 and truncated to the 32-bit pixel_type (wraps for
    // bit_depth == 32, matching the C++ implicit conversion).
    const scaled = @as(u64, @intCast(value)) *% ((@as(u64, 1) << @intCast(@min(bit_depth, 63))) -% 1);
    return @bitCast(@as(u32, @truncate(scaled >> 2)));
}

/// Port of palette_internal::GetPaletteValue. `pal_ch` is the palette
/// channel: row `c`, column `index` holds the palette entry for component
/// `c` (the port stores planes densely, so the lookup goes through the
/// channel's own rows, matching libjxl's `palette[c * onerow + index]`
/// stride arithmetic).
fn getPaletteValue(pal_ch: *const Channel, index_in: i64, c: usize, palette_size: i64, bit_depth: u32) pixel_type {
    var index = index_in;
    if (index < 0) {
        if (c >= kRgbChannels) return 0;
        index = -(index + 1);
        index = @mod(index, 1 + 2 * (delta_palette.len - 1));
        const multiplier: i64 = if (index & 1 == 0) -1 else 1;
        var result: i64 = @as(i64, delta_palette[@intCast((index + 1) >> 1)][c]) * multiplier;
        if (bit_depth > 8) {
            // (C++ signed overflow wraps in practice; *% matches it for
            // bit_depth == 32.)
            result *%= @as(i64, 1) << @intCast(bit_depth - 8);
        }
        return @bitCast(@as(u32, @truncate(@as(u64, @bitCast(result)))));
    } else if (palette_size <= index and index < palette_size + kLargeCubeOffset) {
        if (c >= kRgbChannels) return 0;
        var idx = index - palette_size;
        idx >>= @intCast(c * kSmallCubeBits);
        const scale_part: i64 = paletteScale(@mod(idx, kSmallCube), bit_depth);
        const base: i64 = @as(i64, 1) << @intCast(@min(30, @max(0, @as(i32, @intCast(bit_depth)) - 3)));
        return @bitCast(@as(u32, @truncate(@as(u64, @bitCast(scale_part +% base)))));
    } else if (palette_size + kLargeCubeOffset <= index) {
        if (c >= kRgbChannels) return 0;
        var idx = index - (palette_size + kLargeCubeOffset);
        switch (c) {
            1 => idx = @divTrunc(idx, kLargeCube),
            2 => idx = @divTrunc(idx, kLargeCube * kLargeCube),
            else => {},
        }
        // libjxl: `Scale<kLargeCube - 1>(index % kLargeCube, bit_depth)` —
        // the cube coordinate is index modulo kLargeCube (5); the -1 only
        // selects the Scale<4> denominator.
        return paletteScale(@mod(idx, kLargeCube), bit_depth);
    }
    // Explicit palette entry: row c (component), column index.
    if (index < 0 or index >= pal_ch.w or c >= pal_ch.h) return 0;
    return pal_ch.row(c)[@intCast(index)];
}

/// Port of InvPalette (delta-palette path included; Weighted deltas use the
/// caller-provided wp header).
pub fn inversePalette(image: *Image, begin_c: u32, nb_colors: u32, nb_deltas: u32, predictor: u32, wp_header: *const context_predict.weighted.Header, bit_depth: u32) !void {
    _ = nb_colors;
    if (image.nb_meta_channels < 1) return error.PaletteWithoutMetaChannel;
    const nb: usize = @intCast(image.channels.items[0].h);
    const c0: usize = begin_c + 1;
    if (c0 >= image.channels.items.len) return error.InvalidChannelRange;
    const w = image.channels.items[c0].w;
    const v = image.channels.items[c0].h;
    if (nb < 1) return error.InvalidTransform;
    for (1..nb) |_| {
        const extra = try Channel.create(image.gpa, w, v, image.channels.items[c0].hshift, image.channels.items[c0].vshift);
        try image.channels.insert(image.gpa, c0 + 1, extra);
    }

    // Re-fetch the palette channel: the inserts above may have reallocated
    // the channel list, invalidating any pointer captured earlier (the
    // palette channel itself stays at index 0).
    const pal_ch = &image.channels.items[0];
    const palette_w: i64 = @intCast(pal_ch.w);

    if (w == 0) {
        // nothing to do
    } else if (nb_deltas == 0 and predictor == 0) {
        // Predictor::Zero, plain palette lookup
        if (nb == 1) {
            for (0..v) |y| {
                const p = image.channels.items[c0].row(y);
                for (0..w) |x| {
                    const index = std.math.clamp(p[x], 0, @as(pixel_type, @intCast(palette_w - 1)));
                    p[x] = getPaletteValue(pal_ch, index, 0, palette_w, bit_depth);
                }
            }
        } else {
            for (0..v) |y| {
                const index_row = image.channels.items[c0].row(y);
                for (0..w) |x| {
                    const index: i64 = index_row[x];
                    for (0..nb) |c| {
                        image.channels.items[c0 + c].row(y)[x] = getPaletteValue(pal_ch, index, c, palette_w, bit_depth);
                    }
                }
            }
        }
    } else {
        // Delta palette: save indices, then re-predict per channel.
        var indices = try Channel.create(image.gpa, w, v, 0, 0);
        defer indices.deinit();
        for (0..v) |y| {
            @memcpy(indices.row(y), image.channels.items[c0].row(y));
        }
        for (0..nb) |c| {
            const channel = &image.channels.items[c0 + c];
            var wp_state = try context_predict.weighted.State.create(image.gpa, wp_header, channel.w, channel.h);
            defer wp_state.deinit(image.gpa);
            for (0..channel.h) |y| {
                const p = channel.row(y);
                const idx_row = indices.row(y);
                for (0..channel.w) |x| {
                    const index: i64 = idx_row[x];
                    const palette_entry = getPaletteValue(pal_ch, index, c, palette_w, bit_depth);
                    var val: i64 = 0;
                    if (index < nb_deltas) {
                        const pred = if (context_predict.predictorHasWeighted(@enumFromInt(@as(u8, @intCast(predictor)))))
                            context_predict.predictNoTreeWP(channel.w, &channel.plane, x, y, @enumFromInt(@as(u8, @intCast(predictor))), &wp_state)
                        else
                            context_predict.predictNoTreeNoWP(channel.w, &channel.plane, x, y, @enumFromInt(@as(u8, @intCast(predictor))));
                        val = pred.guess + palette_entry;
                    } else {
                        val = palette_entry;
                    }
                    p[x] = @intCast(val);
                    if (context_predict.predictorHasWeighted(@enumFromInt(@as(u8, @intCast(predictor))))) {
                        wp_state.updateErrors(p[x], x, y, channel.w);
                    }
                }
            }
        }
    }

    // Remove the palette channel (signed arithmetic mirrors libjxl's
    // `nb_meta_channels -= 2 - nb`, which can be negative).
    if (c0 >= image.nb_meta_channels) {
        image.nb_meta_channels -= 1;
    } else {
        const delta: isize = 2 - @as(isize, @intCast(nb));
        if (delta >= 0) {
            image.nb_meta_channels -= @intCast(delta);
        } else {
            image.nb_meta_channels += @intCast(-delta);
        }
    }
    // Free and drop the palette channel itself.
    image.channels.items[0].deinit();
    _ = try image.channels.replaceRange(image.gpa, 0, 1, &.{});
}

// ---------------------------------------------------------------------------
// Squeeze inverse (libjxl squeeze.cc InvHSqueeze/InvVSqueeze/InvSqueeze)
// ---------------------------------------------------------------------------

/// Port of `SmoothTendency` (squeeze.h): the reversible "tendency" added to
/// residuals so that smooth areas produce zero residuals. Shared by the
/// inverse (this module) and the forward squeeze (enc_transforms.zig).
pub fn smoothTendency(B: pixel_type_w, a: pixel_type_w, n: pixel_type_w) pixel_type_w {
    var diff: pixel_type_w = 0;
    if (B >= a and a >= n) {
        diff = @divTrunc(4 * B - 3 * n - a + 6, 12);
        //      2C = a<<1 + diff - diff&1 <= 2B  so diff - diff&1 <= 2B - 2a
        //      2D = a<<1 - diff - diff&1 >= 2n  so diff + diff&1 <= 2a - 2n
        if (diff - (diff & 1) > 2 * (B - a)) diff = 2 * (B - a) + 1;
        if (diff + (diff & 1) > 2 * (a - n)) diff = 2 * (a - n);
    } else if (B <= a and a <= n) {
        diff = @divTrunc(4 * B - 3 * n - a - 6, 12);
        //      2C = a<<1 + diff + diff&1 >= 2B  so diff + diff&1 >= 2B - 2a
        //      2D = a<<1 - diff + diff&1 <= 2n  so diff - diff&1 >= 2a - 2n
        if (diff + (diff & 1) < 2 * (B - a)) diff = 2 * (B - a) - 1;
        if (diff - (diff & 1) < 2 * (a - n)) diff = 2 * (a - n);
    }
    return diff;
}

/// Port of `InvHSqueeze` for channel `c` with residuals in channel `rc`:
/// averages are `c`, residuals `rc`; the output channel replaces `c`.
fn invHSqueeze(image: *Image, c: usize, rc: usize) !void {
    std.debug.assert(c < image.channels.items.len);
    std.debug.assert(rc < image.channels.items.len);
    // Move out the average channel (the residual stays in place).
    var chin = image.channels.items[c];
    const chin_residual = &image.channels.items[rc];
    // These must be valid since we ran MetaApply already.
    if (chin.w != @divTrunc(chin.w + chin_residual.w + 1, 2)) return error.CorruptedSqueeze;
    if (chin.h != chin_residual.h) return error.CorruptedSqueeze;

    if (chin_residual.w == 0) {
        // Short-circuit: output channel has same dimensions as input; the
        // plane stays in place, only the shift changes.
        image.channels.items[c].hshift -= 1;
        return;
    }
    var chout = try Channel.create(image.gpa, chin.w + chin_residual.w, chin.h, chin.hshift - 1, chin.vshift);

    var y: usize = 0;
    while (y < chin.h) : (y += 1) {
        const p_residual = chin_residual.row(y);
        const p_avg = chin.row(y);
        const p_out = chout.row(y);
        var x: usize = 0;
        while (x < chin_residual.w) : (x += 1) {
            const diff_minus_tendency: pixel_type_w = p_residual[x];
            const avg: pixel_type_w = p_avg[x];
            const next_avg: pixel_type_w = if (x + 1 < chin.w) p_avg[x + 1] else avg;
            const left: pixel_type_w = if (x != 0) p_out[(x << 1) - 1] else avg;
            const tendency = smoothTendency(left, avg, next_avg);
            const diff = diff_minus_tendency + tendency;
            const A: pixel_type_w = avg + @divTrunc(diff, 2);
            p_out[x << 1] = @truncate(A);
            const Bv: pixel_type_w = A - diff;
            p_out[(x << 1) + 1] = @truncate(Bv);
        }
        if (chout.w & 1 != 0) p_out[chout.w - 1] = p_avg[chin.w - 1];
    }
    chin.deinit();
    image.channels.items[c] = chout;
}

/// Port of `InvVSqueeze` for channel `c` with residuals in channel `rc`.
fn invVSqueeze(image: *Image, c: usize, rc: usize) !void {
    std.debug.assert(c < image.channels.items.len);
    std.debug.assert(rc < image.channels.items.len);
    var chin = image.channels.items[c];
    const chin_residual = &image.channels.items[rc];
    if (chin.h != @divTrunc(chin.h + chin_residual.h + 1, 2)) return error.CorruptedSqueeze;
    if (chin.w != chin_residual.w) return error.CorruptedSqueeze;

    if (chin_residual.h == 0) {
        // Short-circuit: output channel has same dimensions as input; the
        // plane stays in place, only the shift changes.
        image.channels.items[c].vshift -= 1;
        return;
    }
    // Note: chin.h >= chin_residual.h and at most 1 different.
    var chout = try Channel.create(image.gpa, chin.w, chin.h + chin_residual.h, chin.hshift, chin.vshift - 1);

    if (chin_residual.w != 0) {
        // We only iterate up to chin_residual.h which is always <= chin.h.
        var y: usize = 0;
        while (y < chin_residual.h) : (y += 1) {
            const p_residual = chin_residual.row(y);
            const p_avg = chin.row(y);
            const p_navg = chin.row(if (y + 1 < chin.h) y + 1 else y);
            const p_out = chout.row(y << 1);
            const p_nout = chout.row((y << 1) + 1);
            const p_pout = if (y > 0) chout.row((y << 1) - 1) else p_avg;
            for (0..chin.w) |x| {
                const avg: pixel_type_w = p_avg[x];
                const next_avg: pixel_type_w = p_navg[x];
                const top: pixel_type_w = p_pout[x];
                const tendency = smoothTendency(top, avg, next_avg);
                const diff_minus_tendency: pixel_type_w = p_residual[x];
                const diff = diff_minus_tendency + tendency;
                const out: pixel_type_w = avg + @divTrunc(diff, 2);
                p_out[x] = @truncate(out);
                // If the chin_residual.h == chin.h, the output has an even
                // number of rows so the next line is fine. Otherwise, this
                // loop won't write to the last output row which is handled
                // separately.
                p_nout[x] = @truncate(out - diff);
            }
        }
    }

    if (chout.h & 1 != 0) {
        const y = chin.h - 1;
        const p_avg = chin.row(y);
        const p_out = chout.row(y << 1);
        for (0..chin.w) |x| {
            p_out[x] = p_avg[x];
        }
    }
    chin.deinit();
    image.channels.items[c] = chout;
}

/// Port of `InvSqueeze`: undo all squeezes (in reverse order).
pub fn inverseSqueeze(image: *Image, parameters: []const SqueezeParams) !void {
    if (parameters.len == 0) return; // default scheme handled at meta time
    var i: usize = parameters.len;
    while (i > 0) {
        i -= 1;
        try checkMetaSqueezeParams(parameters[i], image.channels.items.len);
        const horizontal = parameters[i].horizontal;
        const in_place = parameters[i].in_place;
        const beginc = parameters[i].begin_c;
        const endc = parameters[i].begin_c + parameters[i].num_c - 1;
        var offset: usize = undefined;
        if (in_place) {
            offset = endc + 1;
        } else {
            offset = image.channels.items.len + beginc - endc - 1;
        }
        if (beginc < image.nb_meta_channels) {
            // This is checked in MetaSqueeze.
            if (image.nb_meta_channels <= parameters[i].num_c) return error.InvalidSqueeze;
            image.nb_meta_channels -= parameters[i].num_c;
        }
        var c = beginc;
        while (c <= endc) : (c += 1) {
            const rc = offset + c - beginc;
            // MetaApply should imply that `rc` is within range.
            if (rc >= image.channels.items.len) return error.InvalidSqueeze;
            if (image.channels.items[c].w < image.channels.items[rc].w or
                image.channels.items[c].h < image.channels.items[rc].h)
            {
                return error.CorruptedSqueeze;
            }
            if (horizontal) {
                try invHSqueeze(image, c, rc);
            } else {
                try invVSqueeze(image, c, rc);
            }
        }
        // Free and drop the residual channels.
        for (image.channels.items[offset .. offset + endc - beginc + 1]) |*ch| ch.deinit();
        _ = try image.channels.replaceRange(image.gpa, offset, endc - beginc + 1, &.{});
    }
}

/// Inverse transform dispatch.
pub fn inverse(image: *Image, t: *const Transform, wp_header: *const context_predict.weighted.Header, bit_depth: u32) !void {
    switch (t.id) {
        .rct => try inverseRCT(image, t.begin_c, t.rct_type),
        .palette => try inversePalette(image, t.begin_c, t.nb_colors, t.nb_deltas, t.predictor, wp_header, bit_depth),
        .squeeze => try inverseSqueeze(image, t.squeezes.items),
        .invalid => return error.InvalidTransform,
    }
}

test "transform syntax roundtrip" {
    const alloc = std.testing.allocator;
    var t = Transform.init(alloc);
    defer t.deinit();
    t.id = .squeeze;
    try t.squeezes.append(alloc, .{ .horizontal = true, .in_place = true, .begin_c = 0, .num_c = 3 });
    try t.squeezes.append(alloc, .{ .horizontal = false, .in_place = false, .begin_c = 1, .num_c = 2 });

    var w = BitWriter.init(alloc);
    defer w.deinit();
    try t.write(&w);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    var got = Transform.init(alloc);
    defer got.deinit();
    try got.read(&br);
    try br.close();
    try std.testing.expectEqual(TransformId.squeeze, got.id);
    try std.testing.expectEqual(@as(usize, 2), got.squeezes.items.len);
    try std.testing.expectEqual(true, got.squeezes.items[0].horizontal);
    try std.testing.expectEqual(false, got.squeezes.items[1].in_place);
}
