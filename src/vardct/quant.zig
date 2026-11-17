//! VarDCT quantization data: QuantizerParams (`quantizer.cc`), the DC
//! dequant values (`quant_weights.cc` DequantMatrices::DecodeDC /
//! `enc_quant_weights.cc` DequantMatricesEncodeDC) and the full
//! DequantMatrices table encoding with the RAW mode used by JPEG
//! transcoding (`QuantEncoding::kQuantModeRAW`: F16 denominator + a
//! modular stream holding the table values).

const std = @import("std");
const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const fields_ext = @import("../header/fields_ext.zig");

const stream = @import("../modular/stream.zig");
const image_mod = @import("../modular/image.zig");
const ma = @import("../modular/ma.zig");
const context_predict = @import("../modular/context_predict.zig");
const enc_ans = @import("../entropy/enc_ans.zig");
const dec_ans = @import("../entropy/dec_ans.zig");
const ma_encode = @import("../modular/ma_encode.zig");

const BitReader = bit_io.BitReader;
const BitWriter = bit_io.BitWriter;
const U32Coder = fields.U32Coder;
const U32Enc = fields.U32Enc;
const Image = image_mod.Image;
const Channel = image_mod.Channel;
const Token = ma_encode.Token;
const FrameDim = stream.FrameDim;
const ModularStreamId = stream.ModularStreamId;
const EntropyCode = stream.EntropyCode;

const block_context = @import("block_context.zig");

/// libjxl `DequantMatrices::kNumQuantTables` (QuantTable enum size).
pub const kNumQuantTables: usize = 17;

/// required_size_x/y of libjxl DequantMatrices (17 tables).
pub const required_size_x = [kNumQuantTables]usize{
    1, 1, 1, 1, 2, 4, 1, 1, 2, 1, 1, 8, 4, 16, 8, 32, 16,
};
pub const required_size_y = [kNumQuantTables]usize{
    1, 1, 1, 1, 2, 4, 2, 4, 4, 1, 1, 8, 8, 16, 16, 32, 32,
};

/// QuantEncoding modes (libjxl QuantEncoding::Mode).
pub const QuantMode = enum(u3) {
    library = 0,
    id = 1,
    dct2 = 2,
    dct4 = 3,
    dct4x8 = 4,
    afv = 5,
    dct = 6,
    raw = 7,
};

/// libjxl `kDCQuant` / `kInvDCQuant` defaults (1/4096, 1/512, 1/256).
pub const kInvDCQuant = [3]f32{ 4096.0, 512.0, 256.0 };

/// QuantizerParams, port of `quantizer.cc QuantizerParams::VisitFields`.
pub const QuantizerParams = struct {
    global_scale: u32 = 1,
    quant_dc: u32 = 16,

    pub const global_scale_enc = U32Enc.init(
        fields.bitsOffset(11, 1),
        fields.bitsOffset(11, 2049),
        fields.bitsOffset(12, 4097),
        fields.bitsOffset(16, 8193),
    );
    pub const quant_dc_enc = U32Enc.init(
        fields.val(16),
        fields.bitsOffset(5, 1),
        fields.bitsOffset(8, 1),
        fields.bitsOffset(16, 1),
    );

    pub fn read(self: *QuantizerParams, br: *BitReader) !void {
        self.global_scale = U32Coder.read(global_scale_enc, br);
        self.quant_dc = U32Coder.read(quant_dc_enc, br);
    }

    pub fn write(self: *const QuantizerParams, writer: *BitWriter) !void {
        try U32Coder.write(
            global_scale_enc,
            self.global_scale,
            writer,
        );
        try U32Coder.write(
            quant_dc_enc,
            self.quant_dc,
            writer,
        );
    }
};

/// DC dequant values (DequantMatrices::DecodeDC / EncodeDC).
/// The JPEG transcode path always writes custom values
/// (dc_quant[c] = quant_dc_value / (8*255)).
pub const DequantDC = struct {
    /// Decoded values; all-default corresponds to kDCQuant.
    dc_quant: [3]f32 = .{
        1.0 / kInvDCQuant[0],
        1.0 / kInvDCQuant[1],
        1.0 / kInvDCQuant[2],
    },
    all_default: bool = false,

    pub fn read(self: *DequantDC, br: *BitReader) !void {
        self.all_default = br.readBits(1) != 0;
        if (!self.all_default) {
            for (&self.dc_quant) |*q| {
                q.* = try fields_ext.readF16(br);
                q.* *= 1.0 / 128.0;
                if (q.* < kAlmostZero) return error.InvalidDCQuant;
            }
        } else {
            for (&self.dc_quant, 0..) |*q, c| {
                q.* = 1.0 / kInvDCQuant[c];
            }
        }
    }

    pub fn write(self: *const DequantDC, writer: *BitWriter) !void {
        var all_default = true;
        for (&self.dc_quant, 0..) |q, c| {
            if (q != 1.0 / kInvDCQuant[c]) all_default = false;
        }
        writer.write(1, @intFromBool(all_default));
        if (!all_default) {
            for (&self.dc_quant) |q| {
                try fields_ext.writeF16(q * 128.0, writer);
            }
        }
    }
};

pub const kAlmostZero: f32 = 1.0 / @as(f32, 1 << 16);

/// The single RAW table used by JPEG transcoding: 3 x 64 values (the
/// transposed JPEG quant tables of channels X, Y, B) plus the F16
/// denominator (1 / (8 * 255)).
pub const RawQuantTable = struct {
    /// qtable[c * 64 + y * 8 + x]: channel c value at image row y, col x.
    values: [3 * 64]i32 = @splat(0),
    /// Serialized denominator; must stay within 1e-8 of 1/(8*255) after the
    /// F16 roundtrip for djxl to accept the file as a JPEG transcode.
    den: f32 = 1.0 / (8.0 * 255.0),

    pub fn read(self: *RawQuantTable, br: *BitReader) !void {
        self.den = try fields_ext.readF16(br);
        if (self.den < kAlmostZero) return error.InvalidQuantTable;
    }
};

/// Computes the DC quant values that libjxl's
/// `DequantMatricesSetCustomDC(dcquantization)` yields for a JPEG:
/// dcquantization[c] = 255*8/quant_dc[c] and the serialized value is its
/// reciprocal times 128.
pub fn jpegDCQuantSerialized(quant_dc_value: i32) f32 {
    const q: f32 = @floatFromInt(quant_dc_value);
    return q * 128.0 / (255.0 * 8.0);
}

/// The modular image of a RAW quant table stream: 3 channels of
/// size_x*8 x size_y*8 values (8x8 per channel for the DCT table).
pub fn buildQuantTableImage(
    allocator: std.mem.Allocator,
    raw: *const RawQuantTable,
    table_index: usize,
) !Image {
    const size_x = required_size_x[table_index] * 8;
    const size_y = required_size_y[table_index] * 8;
    var img = Image.create(allocator);
    errdefer img.deinit();
    for (0..3) |c| {
        var ch = try Channel.create(
            allocator,
            size_x,
            size_y,
            0,
            0,
        );
        for (0..size_y) |y| {
            for (0..size_x) |x| {
                ch.row(y)[x] = raw.values[c * size_x * size_y + y * size_x + x];
            }
        }
        try img.channels.append(allocator, ch);
    }
    return img;
}

/// Reads the RAW table values back from a decoded modular image
/// (`ModularFrameDecoder::DecodeQuantTable` value collection).
pub fn readQuantTableImage(
    img: *const Image,
    table_index: usize,
    out: *RawQuantTable,
) !void {
    const size_x = required_size_x[table_index] * 8;
    const size_y = required_size_y[table_index] * 8;
    if (img.channels.items.len != 3) return error.InvalidQuantTable;
    for (0..3) |c| {
        const ch = &img.channels.items[c];
        if (ch.w != size_x or ch.h != size_y) return error.InvalidQuantTable;
        for (0..size_y) |y| {
            for (0..size_x) |x| {
                const v = ch.row(y)[x];
                if (v <= 0 or v >= 65536) return error.InvalidQuantTable;
                out.values[c * size_x * size_y + y * size_x + x] = v;
            }
        }
    }
}

test "quantizer params + dequant dc roundtrip" {
    const alloc = std.testing.allocator;
    var w = BitWriter.init(alloc);
    defer w.deinit();
    var ddc = DequantDC{};
    ddc.dc_quant[0] = 0.188;
    ddc.dc_quant[1] = 0.188;
    ddc.dc_quant[2] = 0.094;
    try ddc.write(&w);
    try (QuantizerParams{
        .global_scale = 65536,
        .quant_dc = 1,
    }).write(&w);
    w.zeroPadToByte();

    var br = BitReader.init(w.getSpan());
    var ddc2 = DequantDC{};
    try ddc2.read(&br);
    var qp2 = QuantizerParams{};
    try qp2.read(&br);
    try std.testing.expectEqual(
        QuantizerParams{
            .global_scale = 65536,
            .quant_dc = 1,
        },
        qp2,
    );
    try std.testing.expect(@abs(ddc2.dc_quant[0] - 0.188) < 1e-3);
    try std.testing.expect(@abs(ddc2.dc_quant[2] - 0.094) < 1e-3);
}
