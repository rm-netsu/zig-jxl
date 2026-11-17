//! rANS/ANS entropy coding from libjxl, ported to Zig 0.16.
//!
//! Root module re-exporting the full stack. See README.md for the mapping
//! to the original C++ sources.

pub const ans_common = @import("ans_common.zig");
pub const bits = @import("../base/bits.zig");
pub const bit_io = @import("../base/bit_io.zig");
pub const fields = @import("../base/fields.zig");
pub const fast_math = @import("../base/fast_math.zig");
pub const dec_ans = @import("dec_ans.zig");
pub const dec_huffman = @import("dec_huffman.zig");
pub const enc_ans = @import("enc_ans.zig");
pub const enc_cluster = @import("enc_cluster.zig");
pub const enc_context_map = @import("enc_context_map.zig");
pub const enc_huffman = @import("enc_huffman.zig");
pub const enc_lz77 = @import("enc_lz77.zig");

pub const BitReader = bit_io.BitReader;
pub const BitWriter = bit_io.BitWriter;
pub const HybridUintConfig = ans_common.HybridUintConfig;
pub const ANSCode = dec_ans.ANSCode;
pub const ANSSymbolReader = dec_ans.ANSSymbolReader;
pub const LZ77Params = fields.LZ77Params;
pub const Token = enc_ans.Token;
pub const Histogram = enc_ans.Histogram;
pub const HistogramParams = enc_ans.HistogramParams;
pub const EntropyEncodingData = enc_ans.EntropyEncodingData;

test {
    _ = ans_common;
    _ = bits;
    _ = bit_io;
    _ = fields;
    _ = fast_math;
    _ = dec_ans;
    _ = dec_huffman;
    _ = enc_ans;
    _ = enc_cluster;
    _ = enc_context_map;
    _ = enc_huffman;
    _ = enc_lz77;
    // Integration tests (encode -> decode roundtrips).
    // integration tests live in codec/tests_roundtrip.zig
}
