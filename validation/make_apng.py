#!/usr/bin/env python3
"""Builds a 2-frame 8-bit RGB APNG from two P6 PPMs (no PIL quirks:
deterministic chunk layout, per-row filter 0).

Usage: make_apng.py <a.ppm> <b.ppm> <out.png> [delay_ms]
"""
import struct
import sys
import zlib


def read_ppm(path):
    d = open(path, "rb").read()
    pos = 2
    vals = []
    while len(vals) < 3:
        while d[pos : pos + 1].isspace():
            pos += 1
        if d[pos : pos + 1] == b"#":
            while d[pos : pos + 1] not in (b"\n", b"\r"):
                pos += 1
            continue
        start = pos
        while not d[pos : pos + 1].isspace():
            pos += 1
        vals.append(int(d[start:pos]))
    w, h, maxval = vals
    raster = d[pos + 1 :]
    assert maxval == 255, "8-bit RGB only"
    expect = w * h * 3
    assert len(raster) >= expect
    return w, h, raster[:expect]


def chunk(typ, data):
    return struct.pack(">I", len(data)) + typ + data + struct.pack(
        ">I", zlib.crc32(typ + data) & 0xFFFFFFFF
    )


def fcTL(seq, w, h, delay_ms):
    delay_num = int(round(delay_ms))
    delay_den = 1000
    # width height x y delay_num delay_den dispose(0=none) blend(0=source)
    return chunk(
        b"fcTL",
        struct.pack(">IIIIIHHBB", seq, w, h, 0, 0, delay_num, delay_den, 0, 0),
    )


def main():
    a, b, out = sys.argv[1], sys.argv[2], sys.argv[3]
    delay_ms = int(sys.argv[4]) if len(sys.argv) > 4 else 200
    wa, ha, ra = read_ppm(a)
    wb, hb, rb = read_ppm(b)
    assert (wa, ha) == (wb, hb), "frames must be the same size"

    def rows(raster):
        out = bytearray()
        for y in range(ha):
            out.append(0)  # filter: None
            out += raster[y * wa * 3 : (y + 1) * wa * 3]
        return bytes(out)

    # Per-frame independent zlib streams (the de-facto layout read by
    # libpng-based APNG readers: IDAT is a complete stream for frame 0 and
    # every fdAT is a complete stream for its frame, after its 4-byte
    # sequence number; this is what Pillow emits and what cjxl's libpng
    # accepts).
    frame0_stream = zlib.compress(rows(ra), 9)
    frame1_stream = zlib.compress(rows(rb), 9)

    ihdr = chunk(b"IHDR", struct.pack(">IIBBBBB", wa, ha, 8, 2, 0, 0, 0))
    actl = chunk(b"acTL", struct.pack(">II", 2, 0))  # 2 frames, infinite loop
    fctl0 = fcTL(0, wa, ha, delay_ms)
    idat = chunk(b"IDAT", frame0_stream)
    fctl1 = fcTL(1, wa, ha, delay_ms)
    fdat = chunk(b"fdAT", struct.pack(">I", 2) + frame1_stream)  # seq 2
    iend = chunk(b"IEND", b"")

    with open(out, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(ihdr)
        f.write(actl)
        f.write(fctl0)
        f.write(idat)
        f.write(fctl1)
        f.write(fdat)
        f.write(iend)
    print(f"apng: {out} {wa}x{ha} 2 frames")


if __name__ == "__main__":
    main()
