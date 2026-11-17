#!/usr/bin/env python3
"""Exact float-sample comparer for PFM files (values must match bit-exactly).
Handles the PFM bottom-up row order and either endianness; RGB (PF) and
gray (Pf). Also compares against the dyadic generators' expected rasters.

Usage: compare_pfm.py <a.pfm> <b.pfm> [--gray]
       compare_pfm.py <a.pfm> --src <src.pfm>   (same as two-file form)
Prints IDENTICAL or the first mismatch, exits 0/1.
"""
import struct
import sys


def read_pfm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic not in (b'PF', b'Pf'):
            raise SystemExit('not a PFM: %s' % path)
        w, h = map(int, f.readline().split())
        scale = float(f.readline().strip())
        nch = 3 if magic == b'PF' else 1
        n = w * h * nch
        fmt = ('>' if scale > 0 else '<') + str(n) + 'f'
        vals = struct.unpack(fmt, f.read(4 * n))
    # file rows are bottom-up; return top-down row-major order
    stride = w * nch
    rows = [vals[y * stride:(y + 1) * stride] for y in range(h)]
    rows.reverse()
    out = []
    for r in rows:
        out.extend(r)
    return w, h, nch, out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    a = read_pfm(args[0])
    b = read_pfm(args[1])
    if a[:3] != b[:3]:
        print('DIFFER: geometry %s vs %s' % (a[:3], b[:3]))
        sys.exit(1)
    for i, (x, y) in enumerate(zip(a[3], b[3])):
        if x != y:
            print('DIFFER: sample %d: %r != %r (%d/%d)' % (i, x, y, i + 1, len(a[3])))
            sys.exit(1)
    print('IDENTICAL (%d samples)' % len(a[3]))
    sys.exit(0)


if __name__ == '__main__':
    main()
