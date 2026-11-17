#!/usr/bin/env python3
"""Compares two PNM-family images (P5/P6/P7, 8 or 16 bit) for exact-raster
equality. Header words may differ (djxl writes different comments); only
the decoded pixel samples are compared. Exit 0 = identical."""
import sys


def read_pnm(path):
    with open(path, "rb") as f:
        data = f.read()
    # Parse header: magic, then whitespace/comments until the 4-tuple is
    # complete (width, height, maxval, [TUPLTYPE ...] ENDHDR for P7).
    magic = data[0:2]
    if magic == b"P7":
        # PAM: key-value header until ENDHDR
        end = data.index(b"ENDHDR\n") + len(b"ENDHDR\n")
        hdr = data[:end].decode("latin-1")
        fields = {}
        for line in hdr.splitlines()[1:]:
            parts = line.split()
            if parts:
                fields[parts[0]] = parts[1:]
        w, h = int(fields["WIDTH"][0]), int(fields["HEIGHT"][0])
        depth = int(fields["DEPTH"][0])
        maxval = int(fields["MAXVAL"][0])
        raster = data[end:]
        return w, h, depth, maxval, raster
    pos = 2
    vals = []
    while len(vals) < 3:  # width, height, maxval
        while pos < len(data):
            c = data[pos : pos + 1]
            if c == b"#":
                while data[pos : pos + 1] not in (b"\n", b"\r"):
                    pos += 1
            elif c.isspace():
                pos += 1
            else:
                break
        start = pos
        while pos < len(data) and not data[pos : pos + 1].isspace():
            pos += 1
        if data[start:pos].startswith(b"#"):
            continue
        vals.append(int(data[start:pos]))
    w, h, maxval = vals
    depth = 3 if magic == b"P6" else 1
    raster = data[pos + 1 :]
    return w, h, depth, maxval, raster


def main():
    a, b = sys.argv[1], sys.argv[2]
    wa, ha, da, ma, ra = read_pnm(a)
    wb, hb, db, mb, rb = read_pnm(b)
    if (wa, ha) != (wb, hb):
        print(f"SIZE MISMATCH: {a} {wa}x{ha} vs {b} {wb}x{hb}")
        return 1
    if da != db:
        print(f"DEPTH MISMATCH: {a} {da} vs {b} {db}")
        return 1
    expected = wa * ha * da * (2 if ma > 255 else 1)
    if len(ra) < expected or len(rb) < expected:
        print(f"RASTER SHORT: {a}={len(ra)} {b}={len(rb)} expected {expected}")
        return 1
    if ra[:expected] != rb[:expected]:
        n = sum(1 for i in range(expected) if ra[i] != rb[i])
        first = next(i for i in range(expected) if ra[i] != rb[i])
        print(f"RASTER MISMATCH: {a} vs {b}: {n}/{expected} bytes differ, first at {first}")
        return 1
    print(f"IDENTICAL {wa}x{ha} depth={da} ({expected} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
