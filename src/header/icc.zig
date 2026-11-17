//! ICC profile codec (decode side), port of libjxl `icc_codec_common.cc`
//! and `icc_codec.cc`.
//!
//! A JXL codestream that sets `ColorEncoding.want_icc` carries a compressed
//! ICC profile right after the header bundles (SizeHeader, ImageMetadata,
//! CustomTransformData): a U64 size prefix, ANS histograms over 41
//! contexts and the entropy-coded `PredictICC` transform of the profile.
//! `unpredictICC` reverses the transform; the result must be byte-equal to
//! the original profile.

const std = @import("std");

const bit_io = @import("../base/bit_io.zig");
const fields = @import("../base/fields.zig");
const dec_ans = @import("../entropy/dec_ans.zig");
const ans_common = @import("../entropy/ans_common.zig");

const BitReader = bit_io.BitReader;
const Allocator = std.mem.Allocator;

pub const Error = error{
    IccTooLarge,
    IccMalformed,
    IccCorrupt,
    IccNotAllCommandsUsed,
    IccNotAllDataUsed,
    IccInvalidResultSize,
    IccOutOfBounds,
    IccInvalidStride,
    IccInvalidWidth,
    IccInvalidOrder,
    IccUnknownTagcode,
    IccUnknownCommand,
    IccInvalidNumTags,
} || Allocator.Error;

// ---------------------------------------------------------------------------
// Common constants (icc_codec_common.h)
// ---------------------------------------------------------------------------

pub const kICCHeaderSize: usize = 128;

pub const Tag = [4]u8;

pub const kAcspTag = Tag{ 'a', 'c', 's', 'p' };
pub const kBkptTag = Tag{ 'b', 'k', 'p', 't' };
pub const kBtrcTag = Tag{ 'b', 'T', 'R', 'C' };
pub const kBxyzTag = Tag{ 'b', 'X', 'Y', 'Z' };
pub const kChadTag = Tag{ 'c', 'h', 'a', 'd' };
pub const kChrmTag = Tag{ 'c', 'h', 'r', 'm' };
pub const kCprtTag = Tag{ 'c', 'p', 'r', 't' };
pub const kCurvTag = Tag{ 'c', 'u', 'r', 'v' };
pub const kDescTag = Tag{ 'd', 'e', 's', 'c' };
pub const kDmddTag = Tag{ 'd', 'm', 'd', 'd' };
pub const kDmndTag = Tag{ 'd', 'm', 'n', 'd' };
pub const kGbd_Tag = Tag{ 'g', 'b', 'd', ' ' };
pub const kGtrcTag = Tag{ 'g', 'T', 'R', 'C' };
pub const kGxyzTag = Tag{ 'g', 'X', 'Y', 'Z' };
pub const kKtrcTag = Tag{ 'k', 'T', 'R', 'C' };
pub const kKxyzTag = Tag{ 'k', 'X', 'Y', 'Z' };
pub const kLumiTag = Tag{ 'l', 'u', 'm', 'i' };
pub const kMab_Tag = Tag{ 'm', 'A', 'B', ' ' };
pub const kMba_Tag = Tag{ 'm', 'B', 'A', ' ' };
pub const kMlucTag = Tag{ 'm', 'l', 'u', 'c' };
pub const kMntrTag = Tag{ 'm', 'n', 't', 'r' };
pub const kParaTag = Tag{ 'p', 'a', 'r', 'a' };
pub const kRgb_Tag = Tag{ 'R', 'G', 'B', ' ' };
pub const kRtrcTag = Tag{ 'r', 'T', 'R', 'C' };
pub const kRxyzTag = Tag{ 'r', 'X', 'Y', 'Z' };
pub const kSf32Tag = Tag{ 's', 'f', '3', '2' };
pub const kTextTag = Tag{ 't', 'e', 'x', 't' };
pub const kVcgtTag = Tag{ 'v', 'c', 'g', 't' };
pub const kWtptTag = Tag{ 'w', 't', 'p', 't' };
pub const kXyz_Tag = Tag{ 'X', 'Y', 'Z', ' ' };

/// Tag names focused on RGB and GRAY monitor profiles.
pub const kNumTagStrings: usize = 17;
pub const kTagStrings = [kNumTagStrings]Tag{
    kCprtTag, kWtptTag, kBkptTag, kRxyzTag, kGxyzTag, kBxyzTag,
    kKxyzTag, kRtrcTag, kGtrcTag, kBtrcTag, kKtrcTag, kChadTag,
    kDescTag, kChrmTag, kDmndTag, kDmddTag, kLumiTag,
};

pub const kCommandTagUnknown: u8 = 1;
pub const kCommandTagTRC: u8 = 2;
pub const kCommandTagXYZ: u8 = 3;
pub const kCommandTagStringFirst: u8 = 4;

/// Tag types focused on RGB and GRAY monitor profiles.
pub const kNumTypeStrings: usize = 8;
pub const kTypeStrings = [kNumTypeStrings]Tag{
    kXyz_Tag, kDescTag, kTextTag, kMlucTag,
    kParaTag, kCurvTag, kSf32Tag, kGbd_Tag,
};

pub const kCommandInsert: u8 = 1;
pub const kCommandShuffle2: u8 = 2;
pub const kCommandShuffle4: u8 = 3;
pub const kCommandPredict: u8 = 4;
pub const kCommandXYZ: u8 = 10;
pub const kCommandTypeStartFirst: u8 = 16;

pub const kFlagBitOffset: u8 = 64;
pub const kFlagBitSize: u8 = 128;

pub const kNumICCContexts: usize = 41;

/// Two base-128 varints at up to 10 bytes each.
pub const kPreambleSize: usize = 20;

// ---------------------------------------------------------------------------
// Small helpers (icc_codec_common.cc)
// ---------------------------------------------------------------------------

pub fn decodeUint32(data: []const u8, pos: usize) u32 {
    if (pos + 4 > data.len) return 0;
    return std.mem.readInt(u32, data[pos..][0..4], .big);
}

pub fn appendUint32(value: u32, out: *std.ArrayList(u8), allocator: Allocator) !void {
    var buf: [4]u8 = undefined;
    std.mem.writeInt(u32, &buf, value, .big);
    try out.appendSlice(allocator, &buf);
}

pub fn decodeKeyword(data: []const u8, pos: usize) Tag {
    if (pos + 4 > data.len) return Tag{ ' ', ' ', ' ', ' ' };
    return .{ data[pos], data[pos + 1], data[pos + 2], data[pos + 3] };
}

pub fn appendKeyword(keyword: Tag, out: *std.ArrayList(u8), allocator: Allocator) !void {
    try out.appendSlice(allocator, &keyword);
}

/// Checks if `a + b` overflows or exceeds `size`.
pub fn checkOutOfBounds(a: u64, b: u64, size: u64) Error!void {
    const pos = a + b;
    if (pos > size) return Error.IccOutOfBounds;
    if (pos < a) return Error.IccOutOfBounds; // overflow happened
}

pub fn checkIs32Bit(v: u64) Error!void {
    const k_upper32: u64 = ~@as(u64, 0xFFFFFFFF);
    if ((v & k_upper32) != 0) return Error.IccOutOfBounds;
}

const kIccInitialHeaderPrediction = [kICCHeaderSize]u8{
    0,   0,   0,   0,   0,   0,   0,   0,   4,   0,   0,   0,   'm', 'n', 't', 'r',
    'R', 'G', 'B', ' ', 'X', 'Y', 'Z', ' ', 0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   'a', 'c', 's', 'p', 0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   246, 214, 0,   1,   0,   0,   0,   0,   211, 45,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
};

test "initial header prediction table matches libjxl" {
    // The non-trivial bytes of libjxl's kIccInitialHeaderPrediction
    // (icc_codec_common.cc), pinned position by position: a single wrong
    // byte makes every ICC stream this port writes undecodable by libjxl
    // while self-roundtrips still pass.
    try std.testing.expectEqual(@as(u8, 4), kIccInitialHeaderPrediction[8]);
    try std.testing.expectEqualSlices(u8, "mntr", kIccInitialHeaderPrediction[12..16]);
    try std.testing.expectEqualSlices(u8, "RGB XYZ ", kIccInitialHeaderPrediction[16..24]);
    try std.testing.expectEqualSlices(u8, "acsp", kIccInitialHeaderPrediction[36..40]);
    try std.testing.expectEqual(@as(u8, 246), kIccInitialHeaderPrediction[70]);
    try std.testing.expectEqual(@as(u8, 214), kIccInitialHeaderPrediction[71]);
    try std.testing.expectEqual(@as(u8, 1), kIccInitialHeaderPrediction[73]);
    try std.testing.expectEqual(@as(u8, 211), kIccInitialHeaderPrediction[78]);
    try std.testing.expectEqual(@as(u8, 45), kIccInitialHeaderPrediction[79]);
    // Every other byte is zero.
    for (kIccInitialHeaderPrediction, 0..) |b, i| {
        const interesting = (i >= 12 and i < 24) or (i >= 36 and i < 40) or
            i == 8 or i == 70 or i == 71 or i == 73 or i == 78 or i == 79;
        if (!interesting) try std.testing.expectEqual(@as(u8, 0), b);
    }
}

pub fn iccInitialHeaderPrediction(size: u32) [kICCHeaderSize]u8 {
    var copy = kIccInitialHeaderPrediction;
    std.mem.writeInt(u32, copy[0..4], size, .big);
    return copy;
}

pub fn iccPredictHeader(icc: []const u8, header: []u8, pos: usize) void {
    if (pos == 8 and icc.len >= 8) {
        header[80] = icc[4];
        header[81] = icc[5];
        header[82] = icc[6];
        header[83] = icc[7];
    }
    if (pos == 41 and icc.len >= 41) {
        if (icc[40] == 'A') {
            header[41] = 'P';
            header[42] = 'P';
            header[43] = 'L';
        }
        if (icc[40] == 'M') {
            header[41] = 'S';
            header[42] = 'F';
            header[43] = 'T';
        }
    }
    if (pos == 42 and icc.len >= 42) {
        if (icc[40] == 'S' and icc[41] == 'G') {
            header[42] = 'I';
            header[43] = ' ';
        }
        if (icc[40] == 'S' and icc[41] == 'U') {
            header[42] = 'N';
            header[43] = 'W';
        }
    }
}

fn predictValue(comptime T: type, p1: T, p2: T, p3: T, order: u2) T {
    // C++ computes in int and truncates on return; unsigned wrapping
    // arithmetic in T is the same modulo 2^bits.
    switch (order) {
        0 => return p1,
        1 => return (p1 *% 2) -% p2,
        2 => return (p1 *% 3) -% (p2 *% 3) +% p3,
        3 => return 0,
    }
}

/// Predicts a value with linear prediction of the given order (0-2), for
/// integers with `width` bytes and the given stride in bytes between
/// values. The start position is at `start + i`; the modulus of `i` selects
/// the byte of the multi-byte integer. `start + i` must be at least
/// `stride * 4` (the caller checks).
pub fn linearPredictICCValue(
    data: []const u8,
    start: usize,
    i: usize,
    stride: usize,
    width: usize,
    order: u2,
) u8 {
    const pos = start + i;
    if (width == 1) {
        const p1 = data[pos - stride];
        const p2 = data[pos - stride * 2];
        const p3 = data[pos - stride * 3];
        return predictValue(u8, p1, p2, p3, order);
    } else if (width == 2) {
        const p = start + (i & ~@as(usize, 1));
        const p1 = (@as(u16, data[p - stride]) << 8) + data[p - stride + 1];
        const p2 = (@as(u16, data[p - stride * 2]) << 8) + data[p - stride * 2 + 1];
        const p3 = (@as(u16, data[p - stride * 3]) << 8) + data[p - stride * 3 + 1];
        const pred = predictValue(u16, p1, p2, p3, order);
        return if (i & 1 != 0) @as(u8, @truncate(pred & 255)) else @as(u8, @truncate((pred >> 8) & 255));
    } else {
        const p = start + (i & ~@as(usize, 3));
        const p1 = decodeUint32(data, p - stride);
        const p2 = decodeUint32(data, p - stride * 2);
        const p3 = decodeUint32(data, p - stride * 3);
        const pred = predictValue(u32, p1, p2, p3, order);
        const shiftbytes = 3 - (i & 3);
        return @truncate((pred >> @intCast(shiftbytes * 8)) & 255);
    }
}

fn byteKind1(b: u8) u8 {
    if ('a' <= b and b <= 'z') return 0;
    if ('A' <= b and b <= 'Z') return 0;
    if ('0' <= b and b <= '9') return 1;
    if (b == '.' or b == ',') return 1;
    if (b == 0) return 2;
    if (b == 1) return 3;
    if (b < 16) return 4;
    if (b == 255) return 6;
    if (b > 240) return 5;
    return 7;
}

fn byteKind2(b: u8) u8 {
    if ('a' <= b and b <= 'z') return 0;
    if ('A' <= b and b <= 'Z') return 0;
    if ('0' <= b and b <= '9') return 1;
    if (b == '.' or b == ',') return 1;
    if (b < 16) return 2;
    if (b > 240) return 3;
    return 4;
}

pub fn iccANSContext(i: usize, b1: usize, b2: usize) usize {
    if (i <= 128) return 0;
    return 1 + byteKind1(@truncate(b1)) + byteKind2(@truncate(b2)) * 8;
}

// ---------------------------------------------------------------------------
// Varints
// ---------------------------------------------------------------------------

/// Decodes a base-128 unsigned varint into `out`, advancing `pos` by the
/// exact number of bytes consumed.
pub fn decodeVarInt(input: []const u8, pos: *usize) Error!u64 {
    var ret: u64 = 0;
    // 9 bytes cover bits 0..62; the 10th byte may only contribute bit 63.
    var i: usize = 0;
    while (i < 9) : (i += 1) {
        if (pos.* >= input.len) return Error.IccOutOfBounds;
        const byte = input[pos.*];
        pos.* += 1;
        ret |= @as(u64, byte & 0x7F) << @intCast(7 * i);
        if (byte & 0x80 == 0) return ret;
    }
    if (pos.* >= input.len) return Error.IccOutOfBounds;
    const byte = input[pos.*];
    pos.* += 1;
    if (byte & 0x80 != 0) return Error.IccMalformed; // varint exceeds 10 bytes
    if (byte & 0x7E != 0) return Error.IccMalformed; // value exceeds 2^64 - 1
    ret |= @as(u64, byte & 0x01) << 63;
    return ret;
}

// ---------------------------------------------------------------------------
// Shuffle (icc_codec.cc)
// ---------------------------------------------------------------------------

/// Shuffles (interleaves) bytes: with width 2, turns "ABCDabcd" into
/// "AaBbCcDd". Transposes a matrix of `ceil(size / width)` columns and
/// `width` rows; missing elements are transposed along with the filled
/// spots.
pub fn shuffle(allocator: Allocator, data: []u8, width: usize) Error!void {
    const size = data.len;
    const height = (size + width - 1) / width; // amount of rows of output
    const result = try allocator.alloc(u8, size);
    defer allocator.free(result);
    // i = output index, j input index
    var s: usize = 0;
    var j: usize = 0;
    var i: usize = 0;
    while (i < size) : (i += 1) {
        result[i] = data[j];
        j += height;
        if (j >= size) {
            s += 1;
            j = s;
        }
    }
    @memcpy(data, result);
}

// ---------------------------------------------------------------------------
// UnpredictICC (icc_codec.cc)
// ---------------------------------------------------------------------------

/// Mimics the beginning of `unpredictICC` for a quick validity check.
/// `data` should hold at least `kPreambleSize` bytes.
pub fn checkPreamble(data: []const u8, enc_size: usize) Error!void {
    var pos: usize = 0;
    const osize = try decodeVarInt(data, &pos);
    try checkIs32Bit(osize);
    const csize = try decodeVarInt(data, &pos);
    try checkIs32Bit(csize);
    try checkOutOfBounds(pos, csize, data.len);
    // We expect that unpredictICC inflates input, not the other way round.
    if (osize + 65536 < enc_size) return Error.IccMalformed;

    // NB(eustas): 64 MiB ICC should be enough for everything!?
    const output_limit: u64 = 1 << 28;
    if (osize > output_limit) return Error.IccTooLarge;
}

/// Decodes the result of `predictICC` back to a valid ICC profile.
pub fn unpredictICC(allocator: Allocator, enc: []const u8) Error![]u8 {
    const size = enc.len;
    var pos: usize = 0;
    const osize = try decodeVarInt(enc, &pos); // Output size
    try checkIs32Bit(osize);
    const csize = try decodeVarInt(enc, &pos); // Commands size
    // Every command is translated to at least one byte.
    try checkIs32Bit(csize);
    var cpos: usize = pos; // pos in commands stream
    try checkOutOfBounds(pos, csize, size);
    const commands_end: usize = cpos + @as(usize, @intCast(csize));
    pos = commands_end; // pos in data stream

    var result: std.ArrayList(u8) = .empty;
    errdefer result.deinit(allocator);

    // Header
    var header = iccInitialHeaderPrediction(@truncate(osize));
    var i: usize = 0;
    while (i <= kICCHeaderSize) : (i += 1) {
        if (result.items.len == osize) {
            if (cpos != commands_end) return Error.IccNotAllCommandsUsed;
            if (pos != size) return Error.IccNotAllDataUsed;
            return result.toOwnedSlice(allocator);
        }
        if (i == kICCHeaderSize) break; // Done
        iccPredictHeader(result.items, &header, i);
        if (pos >= size) return Error.IccOutOfBounds;
        try result.append(allocator, enc[pos] +% header[i]);
        pos += 1;
    }
    if (cpos >= commands_end) return Error.IccOutOfBounds;

    // Tag list
    var numtags = try decodeVarInt(enc[0..commands_end], &cpos);

    if (numtags != 0) {
        numtags -= 1;
        try checkIs32Bit(numtags);
        try appendUint32(@truncate(numtags), &result, allocator);
        var prevtagstart: u64 = kICCHeaderSize + numtags * 12;
        var prevtagsize: u64 = 0;
        while (true) {
            if (result.items.len > osize) return Error.IccInvalidResultSize;
            if (cpos > commands_end) return Error.IccOutOfBounds;
            if (cpos == commands_end) break; // Valid end
            const command = enc[cpos];
            cpos += 1;
            const tagcode = command & 63;
            var tag: Tag = undefined;
            if (tagcode == 0) {
                break;
            } else if (tagcode == kCommandTagUnknown) {
                try checkOutOfBounds(pos, 4, size);
                tag = decodeKeyword(enc, pos);
                pos += 4;
            } else if (tagcode == kCommandTagTRC) {
                tag = kRtrcTag;
            } else if (tagcode == kCommandTagXYZ) {
                tag = kRxyzTag;
            } else {
                if (tagcode - kCommandTagStringFirst >= kNumTagStrings) {
                    return Error.IccUnknownTagcode;
                }
                tag = kTagStrings[tagcode - kCommandTagStringFirst];
            }
            try appendKeyword(tag, &result, allocator);

            var tagstart: u64 = undefined;
            var tagsize: u64 = prevtagsize;
            if (std.mem.eql(u8, &tag, &kRxyzTag) or std.mem.eql(u8, &tag, &kGxyzTag) or
                std.mem.eql(u8, &tag, &kBxyzTag) or std.mem.eql(u8, &tag, &kKxyzTag) or
                std.mem.eql(u8, &tag, &kWtptTag) or std.mem.eql(u8, &tag, &kBkptTag) or
                std.mem.eql(u8, &tag, &kLumiTag))
            {
                tagsize = 20;
            }

            if (command & kFlagBitOffset != 0) {
                tagstart = try decodeVarInt(enc[0..commands_end], &cpos);
            } else {
                try checkIs32Bit(prevtagstart);
                tagstart = prevtagstart + prevtagsize;
            }
            try checkIs32Bit(tagstart);
            try appendUint32(@truncate(tagstart), &result, allocator);
            if (command & kFlagBitSize != 0) {
                tagsize = try decodeVarInt(enc[0..commands_end], &cpos);
            }
            try checkIs32Bit(tagsize);
            try appendUint32(@truncate(tagsize), &result, allocator);
            prevtagstart = tagstart;
            prevtagsize = tagsize;

            if (tagcode == kCommandTagTRC) {
                try appendKeyword(kGtrcTag, &result, allocator);
                try appendUint32(@truncate(tagstart), &result, allocator);
                try appendUint32(@truncate(tagsize), &result, allocator);
                try appendKeyword(kBtrcTag, &result, allocator);
                try appendUint32(@truncate(tagstart), &result, allocator);
                try appendUint32(@truncate(tagsize), &result, allocator);
            }

            if (tagcode == kCommandTagXYZ) {
                try checkIs32Bit(tagstart + tagsize * 2);
                try appendKeyword(kGxyzTag, &result, allocator);
                try appendUint32(@truncate(tagstart + tagsize), &result, allocator);
                try appendUint32(@truncate(tagsize), &result, allocator);
                try appendKeyword(kBxyzTag, &result, allocator);
                try appendUint32(@truncate(tagstart + tagsize * 2), &result, allocator);
                try appendUint32(@truncate(tagsize), &result, allocator);
            }
        }
    }

    // Main Content
    while (true) {
        if (result.items.len > osize) return Error.IccInvalidResultSize;
        if (cpos > commands_end) return Error.IccOutOfBounds;
        if (cpos == commands_end) break; // Valid end
        const command = enc[cpos];
        cpos += 1;
        if (command == kCommandInsert) {
            const num = try decodeVarInt(enc[0..commands_end], &cpos);
            try checkOutOfBounds(pos, num, size);
            var k: usize = 0;
            while (k < num) : (k += 1) {
                try result.append(allocator, enc[pos]);
                pos += 1;
            }
        } else if (command == kCommandShuffle2 or command == kCommandShuffle4) {
            const num = try decodeVarInt(enc[0..commands_end], &cpos);
            try checkOutOfBounds(pos, num, size);
            const shuffled = try allocator.alloc(u8, @intCast(num));
            defer allocator.free(shuffled);
            @memcpy(shuffled, enc[pos .. pos + @as(usize, @intCast(num))]);
            if (command == kCommandShuffle2) {
                try shuffle(allocator, shuffled, 2);
            } else {
                try shuffle(allocator, shuffled, 4);
            }
            var k: usize = 0;
            while (k < num) : (k += 1) {
                try result.append(allocator, shuffled[k]);
                pos += 1;
            }
        } else if (command == kCommandPredict) {
            try checkOutOfBounds(cpos, 2, commands_end);
            const flags = enc[cpos];
            cpos += 1;

            const width: usize = @as(usize, flags & 3) + 1;
            if (width == 3) return Error.IccInvalidWidth;

            const order: u2 = @truncate((flags & 12) >> 2);
            if (order == 3) return Error.IccInvalidOrder;

            var stride: u64 = width;
            if (flags & 16 != 0) {
                stride = try decodeVarInt(enc[0..commands_end], &cpos);
                if (stride < width) return Error.IccInvalidStride;
            }
            // Equivalent to "stride * 4 >= result->size()" without overflow.
            if (result.items.len == 0 or ((result.items.len - 1) >> 2) < stride) {
                return Error.IccInvalidStride;
            }

            const num = try decodeVarInt(enc[0..commands_end], &cpos); // in bytes
            try checkOutOfBounds(pos, num, size);

            const shuffled = try allocator.alloc(u8, @intCast(num));
            defer allocator.free(shuffled);
            @memcpy(shuffled, enc[pos .. pos + @as(usize, @intCast(num))]);

            if (width > 1) {
                try shuffle(allocator, shuffled, width);
            }

            const start = result.items.len;
            var k: usize = 0;
            while (k < num) : (k += 1) {
                const predicted = linearPredictICCValue(result.items, start, k, @intCast(stride), width, order);
                try result.append(allocator, predicted +% shuffled[k]);
            }
            pos += @intCast(num);
        } else if (command == kCommandXYZ) {
            try appendKeyword(kXyz_Tag, &result, allocator);
            var k: usize = 0;
            while (k < 4) : (k += 1) {
                try result.append(allocator, 0);
            }
            try checkOutOfBounds(pos, 12, size);
            k = 0;
            while (k < 12) : (k += 1) {
                try result.append(allocator, enc[pos]);
                pos += 1;
            }
        } else if (command >= kCommandTypeStartFirst and
            command < kCommandTypeStartFirst + kNumTypeStrings)
        {
            try appendKeyword(kTypeStrings[command - kCommandTypeStartFirst], &result, allocator);
            var k: usize = 0;
            while (k < 4) : (k += 1) {
                try result.append(allocator, 0);
            }
        } else {
            return Error.IccUnknownCommand;
        }
    }

    if (pos != size) return Error.IccNotAllDataUsed;
    if (result.items.len != osize) return Error.IccInvalidResultSize;

    return result.toOwnedSlice(allocator);
}

// ---------------------------------------------------------------------------
// ICC stream reader (non-streaming ICCReader)
// ---------------------------------------------------------------------------

/// Cheap header sanity check mirroring what libjxl's `SetFieldsFromICC`
/// (built-in CMS parse) effectively guarantees before a profile is embedded
/// into the codestream headers: a full, non-truncated RGB or GRAY profile
/// ("the value of the profile size field shall be the exact length of the
/// profile", ICC.1). Truncated or padded profiles and exotic color spaces
/// fall back to the sRGB-headers-plus-jbrd-payload path, where libjxl
/// would reject the JPEG outright.
pub fn looksLikeValidProfile(icc_bytes: []const u8) bool {
    if (icc_bytes.len < kICCHeaderSize) return false;
    const size_field = decodeUint32(icc_bytes, 0);
    if (size_field < kICCHeaderSize) return false;
    if (size_field > icc_bytes.len) return false; // truncated profile
    if (!std.mem.eql(u8, icc_bytes[36..40], "acsp")) return false;
    const cs = decodeKeyword(icc_bytes, 16);
    return std.mem.eql(u8, &cs, "RGB ") or std.mem.eql(u8, &cs, "GRAY");
}

/// Reads a complete ICC stream from `br` and returns the raw ICC profile.
/// Port of `ICCReader::Init` + `Process` for whole-buffer decoding:
/// U64 size prefix, histograms over `kNumICCContexts` contexts, entropy
/// code of the PredictICC-transformed profile, then `unpredictICC`.
pub fn readICC(allocator: Allocator, br: *BitReader) (Error || ans_common.DecodeError)![]u8 {
    const enc_size = fields.U64Coder.read(br);
    if (enc_size > 268435456) {
        // Avoid too large memory allocation for invalid file.
        return Error.IccTooLarge;
    }

    var code = dec_ans.ANSCode{};
    defer code.deinit(allocator);
    var context_map: std.ArrayList(u8) = .empty;
    defer context_map.deinit(allocator);
    try dec_ans.decodeHistograms(allocator, br, kNumICCContexts, &code, &context_map, false);

    // distance_multiplier 0 (libjxl `Create` default): the ICC encoder
    // leaves HistogramParams::image_widths empty, so the decoder must keep
    // the special-distance table disabled for distances to match.
    var reader = try dec_ans.ANSSymbolReader.create(allocator, &code, br, 0);
    defer reader.deinit();

    const enc = try allocator.alloc(u8, @intCast(enc_size));
    defer allocator.free(enc);
    var i: usize = 0;
    while (i < enc_size) : (i += 1) {
        const p1: usize = if (i > 0) enc[i - 1] else 0;
        const p2: usize = if (i > 1) enc[i - 2] else 0;
        enc[i] = @truncate(reader.readHybridUint(iccANSContext(i, p1, p2), br, context_map.items));
    }
    if (!reader.checkANSFinalState()) return Error.IccCorrupt;

    if (enc_size > kPreambleSize) {
        try checkPreamble(enc, @intCast(enc_size));
    }
    return unpredictICC(allocator, enc);
}
