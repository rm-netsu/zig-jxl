//! ICC profile codec (encode side), port of libjxl `enc_icc_codec.cc`.
//!
//! `predictICC` transforms a raw ICC profile into a commands+data stream
//! that compresses well (tag keyword and offset prediction, per-tag-type
//! linear prediction of multi-byte values, byte interleaving);
//! `writeICC` entropy-codes the transformed bytes (prefix codes, LZ77)
//! into the codestream header position, mirroring libjxl's `WriteICC`.

const std = @import("std");

const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const enc_ans = @import("../entropy/enc_ans.zig");
const ans_common = @import("../entropy/ans_common.zig");
const icc = @import("icc.zig");

const BitWriter = bit_io.BitWriter;
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;

pub const Error = error{
    IccTooLarge,
    IccInvalidStride,
    IccTooLargeStride,
    IccEmpty,
} || icc.Error || ans_common.EncodeError || Allocator.Error;

/// 1GB should be enough for any purpose (fuzzer expectation).
const kSizeLimit: u64 = std.math.maxInt(u32) >> 2;

// ---------------------------------------------------------------------------
// Varint / unshuffle / predict helpers
// ---------------------------------------------------------------------------

fn encodeVarInt(value_in: u64, data: *ArrayList(u8), allocator: Allocator) !void {
    var value = value_in;
    try data.ensureUnusedCapacity(allocator, 10);
    // While more than 7 bits of data are left, store 7 bits and set the
    // next byte flag.
    while (value > 127) {
        data.appendAssumeCapacity(@as(u8, @truncate(value & 127)) | 128);
        value >>= 7;
    }
    data.appendAssumeCapacity(@truncate(value & 127));
}

/// Unshuffles (de-interleaves) bytes: with width 2, turns "AaBbCcDd" into
/// "ABCDabcd". Transposes a matrix of `width` columns and
/// `ceil(size / width)` rows.
fn unshuffle(allocator: Allocator, data: []u8, width: usize) !void {
    const size = data.len;
    const height = (size + width - 1) / width; // amount of rows of input
    const result = try allocator.alloc(u8, size);
    defer allocator.free(result);

    // i = input index, j output index
    var s: usize = 0;
    var j: usize = 0;
    var i: usize = 0;
    while (i < size) : (i += 1) {
        result[j] = data[i];
        j += height;
        if (j >= size) {
            s += 1;
            j = s;
        }
    }
    @memcpy(data, result);
}

/// Predicts and shuffles `num` bytes starting at `pos` of `data` (the ICC
/// profile), appending the transformed bytes to `result`.
fn predictAndShuffle(
    allocator: Allocator,
    stride: usize,
    width: usize,
    order: u2,
    num: usize,
    data: []const u8,
    pos: *usize,
    result: *ArrayList(u8),
) Error!void {
    if (pos.* + num > data.len) return Error.IccOutOfBounds;
    // Required by the specification, see the decoder. stride * 4 must be
    // < pos.
    if (pos.* == 0 or ((pos.* - 1) >> 2) < stride) {
        return Error.IccInvalidStride;
    }
    if (pos.* < stride * 4) return Error.IccTooLargeStride;
    const start = result.items.len;
    try result.ensureUnusedCapacity(allocator, num);
    var i: usize = 0;
    while (i < num) : (i += 1) {
        const predicted = icc.linearPredictICCValue(data, pos.*, i, stride, width, order);
        result.appendAssumeCapacity(data[pos.* + i] -% predicted);
    }
    pos.* += num;
    if (width > 1) {
        try unshuffle(allocator, result.items[start..], width);
    }
}

// ---------------------------------------------------------------------------
// PredictICC
// ---------------------------------------------------------------------------

const TagEntry = struct {
    tag: icc.Tag,
    start: usize,
    size: usize,
};

/// Outputs a transformed form of the given ICC profile. The result itself
/// is not particularly smaller than the input data in bytes, but it will
/// be in a form that is easier to compress (more zeroes, ...) and will
/// compress better with the entropy coder.
pub fn predictICC(allocator: Allocator, icc_bytes: []const u8, result: *ArrayList(u8)) Error!void {
    var commands: ArrayList(u8) = .empty;
    defer commands.deinit(allocator);
    var data: ArrayList(u8) = .empty;
    defer data.deinit(allocator);

    const size = icc_bytes.len;
    if (size > kSizeLimit) return Error.IccTooLarge;

    try encodeVarInt(size, result, allocator);

    // Header
    var header = icc.iccInitialHeaderPrediction(@truncate(size));
    {
        var i: usize = 0;
        while (i < icc.kICCHeaderSize and i < size) : (i += 1) {
            icc.iccPredictHeader(icc_bytes, &header, i);
            try data.append(allocator, icc_bytes[i] -% header[i]);
        }
    }
    if (size <= icc.kICCHeaderSize) {
        try encodeVarInt(0, result, allocator); // 0 commands
        try result.appendSlice(allocator, data.items);
        return;
    }

    var tags: ArrayList(TagEntry) = .empty;
    defer tags.deinit(allocator);
    var tagmap = std.AutoHashMap(usize, usize).init(allocator);
    defer tagmap.deinit();

    // Tag list
    var pos: usize = icc.kICCHeaderSize;
    if (pos + 4 <= size) {
        const numtags: u64 = icc.decodeUint32(icc_bytes, pos);
        pos += 4;
        try encodeVarInt(numtags + 1, &commands, allocator);
        var prevtagstart: u64 = icc.kICCHeaderSize + numtags * 12;
        var prevtagsize: u32 = 0;
        var i: usize = 0;
        while (i < numtags) : (i += 1) {
            if (pos + 12 > size) break;

            const tag = icc.decodeKeyword(icc_bytes, pos + 0);
            const tagstart = icc.decodeUint32(icc_bytes, pos + 4);
            const tagsize = icc.decodeUint32(icc_bytes, pos + 8);
            pos += 12;

            try tags.append(allocator, .{ .tag = tag, .start = tagstart, .size = tagsize });
            try tagmap.put(tagstart, tags.items.len - 1);

            var tagcode: u8 = icc.kCommandTagUnknown;
            for (icc.kTagStrings, 0..) |ts, j| {
                if (std.mem.eql(u8, &tag, &ts)) {
                    tagcode = @as(u8, @intCast(j)) + icc.kCommandTagStringFirst;
                    break;
                }
            }

            if (std.mem.eql(u8, &tag, &icc.kRtrcTag) and pos + 24 < size) {
                var ok = true;
                ok = ok and std.mem.eql(u8, &icc.decodeKeyword(icc_bytes, pos + 0), &icc.kGtrcTag);
                ok = ok and std.mem.eql(u8, &icc.decodeKeyword(icc_bytes, pos + 12), &icc.kBtrcTag);
                if (ok) {
                    var kk: usize = 0;
                    while (kk < 8) : (kk += 1) {
                        if (icc_bytes[pos - 8 + kk] != icc_bytes[pos + 4 + kk]) ok = false;
                        if (icc_bytes[pos - 8 + kk] != icc_bytes[pos + 16 + kk]) ok = false;
                    }
                }
                if (ok) {
                    tagcode = icc.kCommandTagTRC;
                    pos += 24;
                    i += 2;
                }
            }

            if (std.mem.eql(u8, &tag, &icc.kRxyzTag) and pos + 24 < size) {
                var ok = true;
                ok = ok and std.mem.eql(u8, &icc.decodeKeyword(icc_bytes, pos + 0), &icc.kGxyzTag);
                ok = ok and std.mem.eql(u8, &icc.decodeKeyword(icc_bytes, pos + 12), &icc.kBxyzTag);
                const offsetr = tagstart;
                const offsetg = icc.decodeUint32(icc_bytes, pos + 4);
                const offsetb = icc.decodeUint32(icc_bytes, pos + 16);
                const sizer = tagsize;
                const sizeg = icc.decodeUint32(icc_bytes, pos + 8);
                const sizeb = icc.decodeUint32(icc_bytes, pos + 20);
                ok = ok and sizer == 20;
                ok = ok and sizeg == 20;
                ok = ok and sizeb == 20;
                ok = ok and (offsetg == offsetr + 20);
                ok = ok and (offsetb == offsetr + 40);
                if (ok) {
                    tagcode = icc.kCommandTagXYZ;
                    pos += 24;
                    i += 2;
                }
            }

            var command = tagcode;
            const predicted_tagstart = prevtagstart + prevtagsize;
            if (predicted_tagstart != tagstart) command |= icc.kFlagBitOffset;
            var predicted_tagsize: usize = prevtagsize;
            if (std.mem.eql(u8, &tag, &icc.kRxyzTag) or std.mem.eql(u8, &tag, &icc.kGxyzTag) or
                std.mem.eql(u8, &tag, &icc.kBxyzTag) or std.mem.eql(u8, &tag, &icc.kKxyzTag) or
                std.mem.eql(u8, &tag, &icc.kWtptTag) or std.mem.eql(u8, &tag, &icc.kBkptTag) or
                std.mem.eql(u8, &tag, &icc.kLumiTag))
            {
                predicted_tagsize = 20;
            }
            if (predicted_tagsize != tagsize) command |= icc.kFlagBitSize;
            try commands.append(allocator, command);
            if (tagcode == 1) {
                try icc.appendKeyword(tag, &data, allocator);
            }
            if (command & icc.kFlagBitOffset != 0) {
                try encodeVarInt(tagstart, &commands, allocator);
            }
            if (command & icc.kFlagBitSize != 0) {
                try encodeVarInt(tagsize, &commands, allocator);
            }

            prevtagstart = tagstart;
            prevtagsize = tagsize;
        }
    }
    // Indicate end of tag list or varint indicating there's none.
    try commands.append(allocator, 0);

    // Main content. The main content in a valid ICC profile contains tagged
    // elements, with the tag types (4 letter names) given by the tag list
    // above. It is allowed for tagged elements to overlap.
    var tag: icc.Tag = .{ 0, 0, 0, 0 };
    var tagstart: usize = 0;
    var tagsize: usize = 0;
    var clutstart: usize = 0;

    var last0 = pos;
    while (pos <= size) {
        var last1 = pos;
        var commands_add: ArrayList(u8) = .empty;
        defer commands_add.deinit(allocator);
        var data_add: ArrayList(u8) = .empty;
        defer data_add.deinit(allocator);

        // This means the loop brought the position beyond the tag end.
        // If tagsize is nonsensical, any pos looks "ok-ish".
        if (pos > tagstart + tagsize and tagsize < kSizeLimit) {
            tag = .{ 0, 0, 0, 0 }; // nonsensical value
        }

        if (commands_add.items.len == 0 and data_add.items.len == 0 and
            tagmap.contains(pos) and pos + 4 <= size)
        {
            const index = tagmap.get(pos).?;
            tag = icc.decodeKeyword(icc_bytes, pos);
            tagstart = tags.items[index].start;
            tagsize = tags.items[index].size;

            if (std.mem.eql(u8, &tag, &icc.kMlucTag) and tagsize > 8 and
                tagsize < kSizeLimit and pos + tagsize <= size and
                icc_bytes[pos + 4] == 0 and icc_bytes[pos + 5] == 0 and
                icc_bytes[pos + 6] == 0 and icc_bytes[pos + 7] == 0)
            {
                const num = tagsize - 8;
                try commands_add.append(allocator, icc.kCommandTypeStartFirst + 3);
                pos += 8;
                try commands_add.append(allocator, icc.kCommandShuffle2);
                try encodeVarInt(num, &commands_add, allocator);
                const start = data_add.items.len;
                try data_add.appendSlice(allocator, icc_bytes[pos .. pos + num]);
                pos += num;
                try unshuffle(allocator, data_add.items[start..], 2);
            }

            if (std.mem.eql(u8, &tag, &icc.kCurvTag) and tagsize > 8 and
                tagsize < kSizeLimit and pos + tagsize <= size and
                icc_bytes[pos + 4] == 0 and icc_bytes[pos + 5] == 0 and
                icc_bytes[pos + 6] == 0 and icc_bytes[pos + 7] == 0)
            {
                const num = tagsize - 8;
                if (num > 16 and num < (1 << 28) and pos + num <= size and pos > 0) {
                    try commands_add.append(allocator, icc.kCommandTypeStartFirst + 5);
                    pos += 8;
                    try commands_add.append(allocator, icc.kCommandPredict);
                    const order: u2 = 1;
                    const width = 2;
                    const stride = width;
                    try commands_add.append(allocator, (@as(u8, order) << 2) | (width - 1));
                    try encodeVarInt(num, &commands_add, allocator);
                    try predictAndShuffle(allocator, stride, width, order, num, icc_bytes, &pos, &data_add);
                }
            }
        }

        if (std.mem.eql(u8, &tag, &icc.kMab_Tag) or std.mem.eql(u8, &tag, &icc.kMba_Tag)) {
            const sub_tag = icc.decodeKeyword(icc_bytes, pos);
            if (pos + 12 < size and
                (std.mem.eql(u8, &sub_tag, &icc.kCurvTag) or std.mem.eql(u8, &sub_tag, &icc.kVcgtTag)) and
                icc.decodeUint32(icc_bytes, pos + 4) == 0)
            {
                const num: usize = @as(usize, icc.decodeUint32(icc_bytes, pos + 8)) * 2;
                if (num > 16 and num < (1 << 28) and pos + 12 + num <= size) {
                    pos += 12;
                    last1 = pos;
                    try commands_add.append(allocator, icc.kCommandPredict);
                    const order: u2 = 1;
                    const width = 2;
                    const stride = width;
                    try commands_add.append(allocator, (@as(u8, order) << 2) | (width - 1));
                    try encodeVarInt(num, &commands_add, allocator);
                    try predictAndShuffle(allocator, stride, width, order, num, icc_bytes, &pos, &data_add);
                }
            }

            if (pos == tagstart + 24 and pos + 4 < size) {
                // Note that this value can be remembered for next iterations
                // of the loop, so the "pos == clutstart" if below can
                // trigger during a later iteration.
                clutstart = tagstart + icc.decodeUint32(icc_bytes, pos);
            }

            if (pos == clutstart and clutstart + 16 < size and tagstart + 10 <= size) {
                const numi: usize = icc_bytes[tagstart + 8];
                const numo: usize = icc_bytes[tagstart + 9];
                const width: usize = icc_bytes[clutstart + 16];
                const stride = width * numo;
                var num: usize = width * numo;
                var i: usize = 0;
                while (i < numi and clutstart + i < size) : (i += 1) {
                    num = num *% icc_bytes[clutstart + i];
                }
                if ((width == 1 or width == 2) and num > 64 and num < (1 << 28) and
                    pos + num <= size and pos > stride * 4)
                {
                    try commands_add.append(allocator, icc.kCommandPredict);
                    const order: u2 = 1;
                    const flags: u8 = (@as(u8, order) << 2) | @as(u8, @intCast(width - 1)) |
                        (if (stride == width) @as(u8, 0) else @as(u8, 16));
                    try commands_add.append(allocator, flags);
                    if (flags & 16 != 0) {
                        try encodeVarInt(stride, &commands_add, allocator);
                    }
                    try encodeVarInt(num, &commands_add, allocator);
                    try predictAndShuffle(allocator, stride, width, order, num, icc_bytes, &pos, &data_add);
                }
            }
        }

        if (commands_add.items.len == 0 and data_add.items.len == 0 and
            std.mem.eql(u8, &tag, &icc.kGbd_Tag) and
            tagsize > 8 and tagsize < kSizeLimit and pos == tagstart + 8 and
            pos + tagsize - 8 <= size and pos > 16)
        {
            const width: usize = 4;
            const order: u2 = 0;
            const stride = width;
            const num = tagsize - 8;
            const flags: u8 = (@as(u8, order) << 2) | @as(u8, @intCast(width - 1)) |
                (if (stride == width) @as(u8, 0) else @as(u8, 16));
            try commands_add.append(allocator, icc.kCommandPredict);
            try commands_add.append(allocator, flags);
            if (flags & 16 != 0) {
                try encodeVarInt(stride, &commands_add, allocator);
            }
            try encodeVarInt(num, &commands_add, allocator);
            try predictAndShuffle(allocator, stride, width, order, num, icc_bytes, &pos, &data_add);
        }

        if (commands_add.items.len == 0 and data_add.items.len == 0 and pos + 20 <= size) {
            const sub_tag = icc.decodeKeyword(icc_bytes, pos);
            if (std.mem.eql(u8, &sub_tag, &icc.kXyz_Tag) and icc.decodeUint32(icc_bytes, pos + 4) == 0) {
                try commands_add.append(allocator, icc.kCommandXYZ);
                pos += 8;
                var j: usize = 0;
                while (j < 12) : (j += 1) {
                    try data_add.append(allocator, icc_bytes[pos]);
                    pos += 1;
                }
            }
        }

        if (commands_add.items.len == 0 and data_add.items.len == 0 and pos + 8 <= size) {
            if (icc.decodeUint32(icc_bytes, pos + 4) == 0) {
                const sub_tag = icc.decodeKeyword(icc_bytes, pos);
                for (icc.kTypeStrings, 0..) |ts, i| {
                    if (std.mem.eql(u8, &sub_tag, &ts)) {
                        try commands_add.append(allocator, icc.kCommandTypeStartFirst + @as(u8, @intCast(i)));
                        pos += 8;
                        break;
                    }
                }
            }
        }

        if (!(commands_add.items.len == 0 and data_add.items.len == 0) or pos == size) {
            if (last0 < last1) {
                try commands.append(allocator, icc.kCommandInsert);
                try encodeVarInt(last1 - last0, &commands, allocator);
                while (last0 < last1) {
                    try data.append(allocator, icc_bytes[last0]);
                    last0 += 1;
                }
            }
            try commands.appendSlice(allocator, commands_add.items);
            try data.appendSlice(allocator, data_add.items);
            last0 = pos;
        }
        if (commands_add.items.len == 0 and data_add.items.len == 0) {
            pos += 1;
        }
    }

    try encodeVarInt(commands.items.len, result, allocator);
    try result.appendSlice(allocator, commands.items);
    try result.appendSlice(allocator, data.items);
}

// ---------------------------------------------------------------------------
// WriteICC
// ---------------------------------------------------------------------------

/// Test-only exposure of the private helpers.
pub const encodeVarIntForTest = encodeVarInt;
pub const unshuffleForTest = unshuffle;

/// Writes the compressed ICC stream (U64 size, histograms, entropy-coded
/// transformed profile) to `writer`, exactly at the position libjxl's
/// `WriteICC` occupies in the codestream headers.
pub fn writeICC(allocator: Allocator, icc_bytes: []const u8, writer: *BitWriter) Error!void {
    if (icc_bytes.len == 0) return Error.IccEmpty;
    var enc: ArrayList(u8) = .empty;
    defer enc.deinit(allocator);
    try predictICC(allocator, icc_bytes, &enc);

    fields.U64Coder.write(enc.items.len, writer);

    var tokens: ArrayList(enc_ans.Token) = .empty;
    try tokens.ensureTotalCapacity(allocator, enc.items.len);
    var i: usize = 0;
    while (i < enc.items.len) : (i += 1) {
        const ctx: u32 = @intCast(icc.iccANSContext(
            i,
            if (i > 0) enc.items[i - 1] else 0,
            if (i > 1) enc.items[i - 2] else 0,
        ));
        tokens.appendAssumeCapacity(.{ .context = ctx, .value = enc.items[i] });
    }
    var params = enc_ans.HistogramParams{};
    params.lz77_method = if (enc.items.len < 16384) .kOptimal else .kLZ77;
    params.force_huffman = true;
    var code = enc_ans.EntropyEncodingData.init(allocator);
    defer code.deinit();
    // buildAndEncodeHistograms takes ownership of the token streams: when
    // LZ77 is enabled they are deinitialized and replaced by the
    // LZ77-tokenized versions in place (libjxl `tokens =
    // std::move(tokens_lz77)`), so the tokens to write must come from the
    // same storage afterwards. token_streams[0] is the sole owner.
    var token_streams = [1]ArrayList(enc_ans.Token){tokens};
    defer token_streams[0].deinit(allocator);
    _ = try enc_ans.buildAndEncodeHistograms(allocator, &params, icc.kNumICCContexts, token_streams[0..], &code, writer);
    _ = try enc_ans.writeTokens(allocator, token_streams[0].items, &code, 0, writer);
}
