#!/usr/bin/env python3
"""Generate fp16-exact float test images (PFM, correct bottom-up raster).

All sample values are dyadic multiples of 2^-8 (v/256 with v in 0..255),
exactly representable in binary16 AND binary32, so the fp16 codec paths
(lossless `--float16`) accept them. Mode "hostile" uses bg 2^-13 / fg 1.0:
every value is fp16-exact, but the glyph/background difference
(1 - 2^-13 = 8191 * 2^-13) needs 13 significant bits and is NOT
representable in binary16 — the case where libjxl's patch-reference-frame
encode fails (float_to_int "losing precision") and this port fails the
same way.

Usage: gen_fp16_patch_images.py <outdir> [--big]
Writes: text_fp16.pfm, text_fp16_gray.pfm (Pf), hostile_fp16.pfm and
optionally big_fp16.pfm (640x400, multi-group).
"""
import os
import struct
import sys
import random


def write_pfm(path, w, h, vals, gray=False):
    """vals: top-down row-major samples (1 per pixel for gray, 3 for RGB).
    PFM stores rows bottom-up."""
    nch = 1 if gray else 3
    assert len(vals) == w * h * nch
    rows = [vals[y * w * nch:(y + 1) * w * nch] for y in range(h)]
    rows.reverse()
    flat = []
    for r in rows:
        flat.extend(r)
    with open(path, 'wb') as f:
        f.write(b'Pf\n%d %d\n-1.0\n' % (w, h) if gray else b'PF\n%d %d\n-1.0\n' % (w, h))
        f.write(struct.pack('<%df' % len(flat), *flat))


def glyph_grid(w, h, seed=42, glyph=None):
    """Text-like grid of repeated 5x5 glyphs on a flat background."""
    if glyph is None:
        glyph = [[0, 32, 64, 32, 0],
                 [32, 96, 128, 96, 32],
                 [64, 128, 255, 128, 64],
                 [32, 96, 128, 96, 32],
                 [0, 32, 64, 32, 0]]
    random.seed(seed)
    px = [[0] * w for _ in range(h)]
    for gy in range(4, h - 8, 9):
        for gx in range(4, w - 8, 9):
            if random.random() < 0.75:
                for y in range(5):
                    for x in range(5):
                        px[gy + y][gx + x] = glyph[y][x]
    return px


def main():
    outdir = sys.argv[1]
    big = '--big' in sys.argv
    os.makedirs(outdir, exist_ok=True)

    # RGB text-like (matches gen_patch_images.py's text_like.ppm pattern)
    w = h = 256
    px = glyph_grid(w, h)
    vals = []
    for row in px:
        for v in row:
            # 3 channels with different weights, all dyadic
            vals.extend([v / 256.0, (v // 2) / 256.0, (v // 4) / 256.0])
    write_pfm(os.path.join(outdir, 'text_fp16.pfm'), w, h, vals)

    # gray text-like
    gvals = []
    for row in px:
        for v in row:
            gvals.append(v / 256.0)
    write_pfm(os.path.join(outdir, 'text_fp16_gray.pfm'), w, h, gvals, gray=True)

    # hostile: bg 2^-13, fg 1.0 in solid 6x8 blocks (the difference
    # 1 - 2^-13 needs 13 significant bits — not representable in binary16)
    random.seed(7)
    hpx = [[0] * w for _ in range(h)]
    for gy in range(4, h - 10, 12):
        for gx in range(4, w - 10, 12):
            if random.random() < 0.8:
                for y in range(8):
                    for x in range(6):
                        hpx[gy + y][gx + x] = 1
    hvals = []
    for row in hpx:
        for v in row:
            f = 1.0 if v else 2.0 ** -13
            hvals.extend([f, f, f])
    write_pfm(os.path.join(outdir, 'hostile_fp16.pfm'), w, h, hvals)

    if big:
        W, H = 640, 400
        bigpx = glyph_grid(W, H)
        bvals = []
        for row in bigpx:
            for v in row:
                bvals.extend([v / 256.0, (v // 2) / 256.0, (v // 4) / 256.0])
        write_pfm(os.path.join(outdir, 'big_fp16.pfm'), W, H, bvals)

    # PPM twins for the libjxl-API reference encoder (make_fp16_patches.c):
    # mode 0 maps every PPM byte v -> v * 2^-8, so the RGB twin carries the
    # same samples as text_fp16.pfm; mode 1 maps v >= 128 -> 1.0 and
    # v < 128 -> 2^-13, so the hostile twin (255/0 bytes on the same block
    # pattern as hostile_fp16.pfm) produces the same hostile raster.
    with open(os.path.join(outdir, 'text_fp16.ppm'), 'wb') as f:
        f.write(b'P6\n%d %d\n255\n' % (w, h))
        f.write(bytes(b for row in px for v in row
                      for b in (v, v // 2, v // 4)))
    with open(os.path.join(outdir, 'hostile_fp16.ppm'), 'wb') as f:
        f.write(b'P6\n%d %d\n255\n' % (w, h))
        f.write(bytes(255 if v else 0 for row in hpx for v in row
                      for _ in range(3)))

    print('OK')


if __name__ == '__main__':
    main()
