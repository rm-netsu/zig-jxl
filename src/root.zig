//! JPEG XL lossless codec in Zig.
//!
//! Layout mirrors libjxl's source tree so the mapping stays obvious:
//!   base/     - bit I/O, field coders (vendored)
//!   entropy/  - ANS + prefix-coding stack (vendored)
//!   modular/  - MA trees, predictors, stream encoding (vendored + glue)
//!   header/   - codestream headers: SizeHeader .. TOC, container
//!   codec/    - pixel buffers, frame encoder/decoder
//!   pnm.zig   - PNM (P5/P6) image input/output
//!   main.zig  - CLI front-end

const std = @import("std");

pub const base = @import("base/bit_io.zig");
pub const bits = @import("base/bits.zig");
pub const fields = @import("base/fields.zig");
pub const float_convert = @import("base/float_convert.zig");
pub const nlee = @import("base/nlee.zig");
pub const entropy = struct {
    pub const ans_common = @import("entropy/ans_common.zig");
    pub const enc_ans = @import("entropy/enc_ans.zig");
    pub const dec_ans = @import("entropy/dec_ans.zig");
    pub const rans = @import("entropy/rans.zig");
    pub const dec_huffman = @import("entropy/dec_huffman.zig");
};
pub const modular = struct {
    pub const common = @import("modular/common.zig");
    pub const options = @import("modular/options.zig");
    pub const image = @import("modular/image.zig");
    pub const ma = @import("modular/ma.zig");
    pub const ma_encode = @import("modular/ma_encode.zig");
    pub const context_predict = @import("modular/context_predict.zig");
    pub const stream = @import("modular/stream.zig");
    pub const transforms = @import("modular/transforms.zig");
    pub const enc_transforms = @import("modular/enc_transforms.zig");
};
pub const header = struct {
    pub const fields_ext = @import("header/fields_ext.zig");
    pub const size = @import("header/size.zig");
    pub const color = @import("header/color.zig");
    pub const metadata = @import("header/metadata.zig");
    pub const frame = @import("header/frame.zig");
    pub const toc = @import("header/toc.zig");
    pub const container = @import("header/container.zig");
    pub const icc = @import("header/icc.zig");
    pub const enc_icc = @import("header/enc_icc.zig");
};
pub const codec = struct {
    pub const pixel = @import("codec/pixel.zig");
    pub const encoder = @import("codec/encoder.zig");
    pub const decoder = @import("codec/decoder.zig");
    pub const animation = @import("codec/animation.zig");
};
pub const patches = struct {
    pub const dictionary = @import("patches/dictionary.zig");
    pub const enc_patches = @import("patches/enc_patches.zig");
};
pub const jpeg = struct {
    pub const jpeg_data = @import("jpeg/jpeg_data.zig");
    pub const reader = @import("jpeg/reader.zig");
    pub const writer = @import("jpeg/writer.zig");
    pub const jbrd = @import("jpeg/jbrd.zig");
    pub const brotli = @import("jpeg/brotli.zig");
    pub const transcode = @import("jpeg/transcode.zig");
};
pub const vardct = struct {
    pub const block_context = @import("vardct/block_context.zig");
    pub const coeff_order = @import("vardct/coeff_order.zig");
    pub const quant = @import("vardct/quant.zig");
    pub const ac_coefficient = @import("vardct/ac_coefficient.zig");
    pub const frame_encode = @import("vardct/frame_encode.zig");
    pub const frame_decode = @import("vardct/frame_decode.zig");
    pub const opsin = @import("vardct/opsin.zig");
    pub const transform_lossy = @import("vardct/transform_lossy.zig");
    pub const lossy = @import("vardct/lossy.zig");
};
pub const pnm = @import("pnm.zig");

test {
    std.testing.refAllDecls(@This());
    _ = @import("base/bits.zig");
    _ = @import("base/bit_io.zig");
    _ = @import("base/fields.zig");
    _ = @import("base/fast_math.zig");
    _ = @import("entropy/ans_common.zig");
    _ = @import("entropy/enc_ans.zig");
    _ = @import("entropy/dec_ans.zig");
    _ = @import("entropy/rans.zig");
    _ = @import("entropy/enc_cluster.zig");
    _ = @import("entropy/enc_context_map.zig");
    _ = @import("entropy/enc_huffman.zig");
    _ = @import("entropy/dec_huffman.zig");
    _ = @import("entropy/enc_lz77.zig");
    _ = @import("modular/common.zig");
    _ = @import("modular/hybrid_uint.zig");
    _ = @import("modular/prefix_code.zig");
    _ = @import("modular/symbol_reader.zig");
    _ = @import("modular/options.zig");
    _ = @import("modular/image.zig");
    _ = @import("modular/rng.zig");
    _ = @import("modular/context_predict.zig");
    _ = @import("modular/ma.zig");
    _ = @import("modular/ma_encode.zig");
    _ = @import("modular/stream.zig");
    _ = @import("modular/transforms.zig");
    _ = @import("base/float_convert.zig");
    _ = @import("base/dither.zig");
    _ = @import("base/nlee.zig");
    _ = @import("modular/enc_transforms.zig");
    _ = @import("modular/tests_group_palette.zig");
    _ = @import("header/fields_ext.zig");
    _ = @import("header/size.zig");
    _ = @import("header/color.zig");
    _ = @import("header/metadata.zig");
    _ = @import("header/frame.zig");
    _ = @import("header/toc.zig");
    _ = @import("header/container.zig");
    _ = @import("header/icc.zig");
    _ = @import("header/enc_icc.zig");
    _ = @import("header/tests_icc.zig");
    _ = @import("codec/pixel.zig");
    _ = @import("codec/tests_roundtrip.zig");
    _ = @import("codec/tests_ans_stress.zig");
    _ = @import("codec/tests_animation.zig");
    _ = @import("patches/dictionary.zig");
    _ = @import("patches/enc_patches.zig");
    _ = @import("patches/tests_patches.zig");
    _ = @import("jpeg/jpeg_data.zig");
    _ = @import("jpeg/reader.zig");
    _ = @import("jpeg/writer.zig");
    _ = @import("jpeg/jbrd.zig");
    _ = @import("jpeg/brotli.zig");
    _ = @import("jpeg/transcode.zig");
    _ = @import("vardct/block_context.zig");
    _ = @import("vardct/coeff_order.zig");
    _ = @import("vardct/quant.zig");
    _ = @import("vardct/ac_coefficient.zig");
    _ = @import("vardct/opsin.zig");
    _ = @import("vardct/transform_lossy.zig");
    _ = @import("vardct/lossy.zig");
    _ = @import("codec/tests_lossy.zig");
    _ = @import("jpeg/tests_roundtrip.zig");
    _ = @import("pnm.zig");
}
