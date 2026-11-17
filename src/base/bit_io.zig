//! Bit-level I/O, bit-exact ports of libjxl `dec_bit_reader.h` and
//! `enc_bit_writer.h`.
//!
//! Both sides are little-endian and LSB-first within a byte, exactly as in
//! the JPEG XL codestream specification.

const std = @import("std");

pub const kMaxBitsPerCall: usize = 56;
pub const kBitsPerByte: usize = 8;

/// Port of `jxl::BitReader`.
///
/// Reads bits written by `BitWriter`. Refills a 64-bit buffer with
/// little-endian loads; reading past the end of the stream returns zeros
/// and is flagged by `close()` / `allReadsWithinBounds()`, matching libjxl.
pub const BitReader = struct {
    data: []const u8,
    next: usize = 0, // index of the next byte to load
    buf: u64 = 0,
    bits_in_buf: usize = 0, // [0, 64)
    overread: u64 = 0, // bytes past the end assumed zero
    checked_out_of_bounds_bits: usize = 0,
    close_called: bool = false,

    /// `bytes` need not be aligned nor padded. Calls refill() like libjxl.
    pub fn init(bytes: []const u8) BitReader {
        var br = BitReader{ .data = bytes };
        br.refill();
        return br;
    }

    /// Based on variant 4 (plus bounds-checking), see
    /// fgiesen.wordpress.com/2018/02/20/reading-bits-in-far-too-many-ways-part-2/
    pub fn refill(self: *BitReader) void {
        if (self.next + 8 > self.data.len) {
            self.boundsCheckedRefill();
        } else {
            // Safe to load 64 bits; insert valid (possibly nonzero) bits above
            // bits_in_buf. The shift requires bits_in_buf < 64.
            const le64 = std.mem.readInt(
                u64,
                self.data[self.next..][0..8],
                .little,
            );
            self.buf |= le64 << @intCast(self.bits_in_buf);

            // Advance by bytes fully absorbed into the buffer.
            self.next += (63 - self.bits_in_buf) >> 3;

            // We absorbed a multiple of 8 bits, so the lower 3 bits of
            // bits_in_buf must remain unchanged. Set the three upper bits so
            // the result >= 56.
            self.bits_in_buf |= 56;
            std.debug.assert(56 <= self.bits_in_buf and self.bits_in_buf < 64);
        }
    }

    fn boundsCheckedRefill(self: *BitReader) void {
        // Read whole bytes until we have [56, 64) bits.
        while (self.bits_in_buf < 64 - kBitsPerByte) : (self.bits_in_buf += kBitsPerByte) {
            if (self.next >= self.data.len) break;
            self.buf |= @as(u64, self.data[self.next]) << @intCast(self.bits_in_buf);
            self.next += 1;
        }
        std.debug.assert(self.bits_in_buf < 64);

        // Add extra bytes as 0 at the end of the stream. If these bits are
        // read, close() will report a failure.
        const extra_bytes = (63 - self.bits_in_buf) / kBitsPerByte;
        self.overread += extra_bytes;
        self.bits_in_buf += extra_bytes * kBitsPerByte;

        std.debug.assert(self.bits_in_buf < 64);
        std.debug.assert(self.bits_in_buf >= 56);
    }

    /// Returns the bits that would be returned by read() without consuming.
    /// It is legal to peek at more bits than present; those bits are zero.
    pub fn peekFixedBits(self: *const BitReader, comptime N: usize) u64 {
        comptime std.debug.assert(N <= kMaxBitsPerCall);
        return self.buf & ((@as(u64, 1) << @intCast(N)) - 1);
    }

    pub fn peekBits(self: *const BitReader, nbits: usize) u64 {
        std.debug.assert(nbits <= kMaxBitsPerCall);
        const mask = (@as(u64, 1) << @intCast(nbits)) - 1;
        return self.buf & mask;
    }

    /// Removes bits from the buffer. The buffer must contain at least
    /// num_bits.
    pub fn consume(self: *BitReader, num_bits: usize) void {
        std.debug.assert(self.bits_in_buf >= num_bits);
        self.bits_in_buf -= num_bits;
        self.buf >>= @intCast(num_bits);
    }

    pub fn readBits(self: *BitReader, nbits: usize) u64 {
        self.refill();
        const bits = self.peekBits(nbits);
        self.consume(nbits);
        return bits;
    }

    pub fn readFixedBits(self: *BitReader, comptime N: usize) u64 {
        self.refill();
        const bits = self.peekFixedBits(N);
        self.consume(N);
        return bits;
    }

    /// Equivalent to calling readFixedBits(1) `skip0` times, but much faster.
    pub fn skipBits(self: *BitReader, skip0: usize) void {
        var skip = skip0;
        // Buffer is large enough - don't zero buf_ below.
        if (skip <= self.bits_in_buf) {
            self.consume(skip);
            return;
        }

        // First deduct what we can satisfy from the buffer.
        skip -= self.bits_in_buf;
        self.bits_in_buf = 0;
        self.buf = 0;

        // Skip whole bytes.
        const whole_bytes = skip / kBitsPerByte;
        skip %= kBitsPerByte;
        const avail = self.data.len - @min(self.next, self.data.len);
        if (whole_bytes > avail) {
            // Overflow condition; consume bits in overread_bytes_.
            self.next = self.data.len;
            skip += kBitsPerByte;
        } else {
            self.next += whole_bytes;
        }

        self.refill();
        self.consume(skip);
    }

    pub fn totalBitsConsumed(self: *const BitReader) usize {
        const bytes_read = self.next + @as(usize, @intCast(self.overread));
        return bytes_read * kBitsPerByte - self.bits_in_buf;
    }

    pub fn totalBytes(self: *const BitReader) usize {
        return self.data.len;
    }

    pub fn jumpToByteBoundary(self: *BitReader) !void {
        const remainder = self.totalBitsConsumed() % kBitsPerByte;
        if (remainder == 0) return;
        if (self.readBits(kBitsPerByte - remainder) != 0) {
            return error.NonZeroPaddingBits;
        }
    }

    /// Forces the reader into an out-of-bounds state, like
    /// `BitReader::MarkUnhealthy`.
    pub fn markUnhealthy(self: *BitReader) void {
        self.next = self.data.len;
        self.overread += 8;
    }

    /// Returns whether all bits read so far were within the input bounds.
    /// Marks the current position as checked (see `BitReader::Close`).
    pub fn allReadsWithinBounds(self: *BitReader) bool {
        self.checked_out_of_bounds_bits = self.totalBitsConsumed();
        return self.totalBitsConsumed() <= self.totalBytes() * kBitsPerByte;
    }

    /// Must be called once, after all reads; mirrors `BitReader::Close()`.
    pub fn close(self: *BitReader) !void {
        std.debug.assert(!self.close_called);
        self.close_called = true;
        if (self.totalBitsConsumed() > self.checked_out_of_bounds_bits and
            self.totalBitsConsumed() > self.totalBytes() * kBitsPerByte)
        {
            return error.ReadMoreBitsThanAvailable;
        }
    }
};

/// Port of `jxl::BitWriter`.
///
/// Writes bits into bytes in increasing addresses, least-significant-bit
/// first within a byte. Unlike libjxl (which uses an explicit Allotment
/// mechanism), this implementation simply grows a zero-padded byte buffer.
pub const BitWriter = struct {
    bits_written: usize = 0,
    storage: std.ArrayList(u8) = .empty,
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) BitWriter {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *BitWriter) void {
        self.storage.deinit(self.allocator);
    }

    pub fn bitsWritten(self: *const BitWriter) usize {
        return self.bits_written;
    }

    /// Byte-aligned view of the written data.
    /// Caller must ensure byte alignment (zeroPadToByte).
    pub fn getSpan(self: *const BitWriter) []const u8 {
        std.debug.assert(self.bits_written % kBitsPerByte == 0);
        return self.storage.items[0 .. self.bits_written / kBitsPerByte];
    }

    pub fn takeBytes(self: *BitWriter) ![]u8 {
        std.debug.assert(self.bits_written % kBitsPerByte == 0);
        const len = self.bits_written / kBitsPerByte;
        self.storage.shrinkRetainingCapacity(len);
        return self.storage.toOwnedSlice(self.allocator);
    }

    /// Writes bits; can write up to 56 bits in one go.
    pub fn write(self: *BitWriter, n_bits: usize, bits: u64) void {
        std.debug.assert(n_bits <= kMaxBitsPerCall);
        std.debug.assert((bits >> @intCast(n_bits)) == 0);

        const bytes_written = self.bits_written / kBitsPerByte;
        const bits_in_first_byte = self.bits_written % kBitsPerByte;
        const shifted = bits << @intCast(bits_in_first_byte);

        // Ensure the next 8 bytes exist and are zero above written bits.
        const needed = bytes_written + 8;
        if (self.storage.items.len < needed) {
            self.storage.appendNTimes(
                self.allocator,
                0,
                needed - self.storage.items.len,
            ) catch @panic("OOM");
        }

        const p = self.storage.items[bytes_written..][0..8];
        const old = std.mem.readInt(u64, p, .little);
        // Last (partial) or next byte to write must be zero-initialized!
        // write/append maintain this invariant.
        std.debug.assert((old >> @intCast(bits_in_first_byte)) == 0);
        std.mem.writeInt(u64, p, old | shifted, .little);

        self.bits_written += n_bits;
    }

    pub fn zeroPadToByte(self: *BitWriter) void {
        const remainder_bits = ((self.bits_written + 7) / 8) * 8 - self.bits_written;
        if (remainder_bits == 0) return;
        self.write(remainder_bits, 0);
        std.debug.assert(self.bits_written % kBitsPerByte == 0);
    }

    /// Concatenates another writer's bits without aligning first.
    pub fn appendUnaligned(self: *BitWriter, other: *const BitWriter) void {
        const full_bytes = other.bits_written / kBitsPerByte;
        const remaining_bits = other.bits_written % kBitsPerByte;
        for (0..full_bytes) |i| {
            self.write(8, other.storage.items[i]);
        }
        if (remaining_bits > 0) {
            const mask = (@as(u64, 1) << @intCast(remaining_bits)) - 1;
            self.write(
                remaining_bits,
                other.storage.items[full_bytes] & mask,
            );
        }
    }
};

test "bit writer / bit reader roundtrip" {
    const alloc = std.testing.allocator;
    var w = BitWriter.init(alloc);
    defer w.deinit();

    w.write(3, 0b101);
    w.write(13, 0x1BEE);
    w.write(32, 0xDEADBEEF);
    w.write(56, 0x0123456789ABCD);
    w.zeroPadToByte();

    const span = w.getSpan();
    var br = BitReader.init(span);
    try std.testing.expectEqual(@as(u64, 0b101), br.readFixedBits(3));
    try std.testing.expectEqual(@as(u64, 0x1BEE), br.readFixedBits(13));
    try std.testing.expectEqual(@as(u64, 0xDEADBEEF), br.readFixedBits(32));
    try std.testing.expectEqual(@as(u64, 0x0123456789ABCD), br.readFixedBits(56));
    try br.close();
}

test "bit reader detects overread" {
    const alloc = std.testing.allocator;
    var w = BitWriter.init(alloc);
    defer w.deinit();
    w.write(4, 0xA);
    w.zeroPadToByte();
    var br = BitReader.init(w.getSpan());
    try std.testing.expectEqual(
        @as(u64, 0xA),
        br.readFixedBits(4),
    );
    _ = br.readBits(56); // past the end -> zeros
    _ = br.readBits(44); // more zeros
    // AllReadsWithinBounds reports the failure and marks it as checked.
    try std.testing.expect(!br.allReadsWithinBounds());
    // Since the out-of-bounds read was checked, close() succeeds
    // (matching libjxl BitReader::Close semantics).
    try br.close();

    // Without checking, close() reports the failure.
    var br2 = BitReader.init(w.getSpan());
    _ = br2.readBits(9);
    try std.testing.expectError(
        error.ReadMoreBitsThanAvailable,
        br2.close(),
    );
}

test "bit reader skip bits" {
    const alloc = std.testing.allocator;
    var w = BitWriter.init(alloc);
    defer w.deinit();
    var prng = std.Random.DefaultPrng.init(12345);
    const rand = prng.random();
    var expected: [200]u64 = undefined;
    var bits_at: [200]usize = undefined;
    var total: usize = 0;
    for (0..200) |i| {
        const nbits = 1 + rand.uintLessThan(usize, 56);
        const val = rand.int(u64) & ((@as(u64, 1) << @intCast(nbits)) - 1);
        w.write(nbits, val);
        expected[i] = val;
        bits_at[i] = total;
        total += nbits;
    }
    w.zeroPadToByte();

    // Randomly skip over a run of values, then verify the value at the
    // landing position by reading it.
    var br = BitReader.init(w.getSpan());
    var idx: usize = 0;
    while (idx < 200) {
        if (rand.boolean() and idx + 1 < 200) {
            const target = idx + 1 + rand.uintLessThan(
                usize,
                @min(50, 200 - idx - 1),
            );
            br.skipBits(bits_at[target] - br.totalBitsConsumed());
            idx = target;
        } else {
            // Recompute the width from the recorded offsets.
            const end = if (idx + 1 < 200) bits_at[idx + 1] else total;
            try std.testing.expectEqual(
                expected[idx],
                br.readBits(end - bits_at[idx]),
            );
            idx += 1;
        }
    }
}

test "bit writer append unaligned" {
    const alloc = std.testing.allocator;
    var a = BitWriter.init(alloc);
    defer a.deinit();
    var b = BitWriter.init(alloc);
    defer b.deinit();
    a.write(5, 0b10101);
    b.write(7, 0b1010101);
    a.appendUnaligned(&b);
    a.zeroPadToByte();

    var br = BitReader.init(a.getSpan());
    try std.testing.expectEqual(@as(u64, 0b10101), br.readFixedBits(5));
    try std.testing.expectEqual(@as(u64, 0b1010101), br.readFixedBits(7));
    try br.close();
}
