#!/usr/bin/env python3
"""Generates patch-friendly ("screenshot/text-like") test images.

Shapes follow what libjxl FindTextLikePatches detects: flat background
areas (4x4 seeds with >= 8 equal-color neighbors) + small connected
components of repeated identical glyphs (each < 32x32, quantized peaks
>= 2, >= 2 occurrences, >= 20 pixels per patch).
"""
import sys


def write_ppm(path, w, h, get):
    with open(path, "wb") as f:
        f.write(f"P6\n{w} {h}\n255\n".encode())
        for y in range(h):
            row = bytearray()
            for x in range(w):
                r, g, b = get(x, y)
                row += bytes([r, g, b])
            f.write(row)


def write_ppm16(path, w, h, get):
    with open(path, "wb") as f:
        f.write(f"P6\n{w} {h}\n65535\n".encode())
        for y in range(h):
            row = bytearray()
            for x in range(w):
                r, g, b = get(x, y)
                row += (r).to_bytes(2, "big") + (g).to_bytes(2, "big") + (b).to_bytes(2, "big")
            f.write(row)


def write_pam_alpha(path, w, h, get):
    with open(path, "wb") as f:
        f.write(b"P7\n")
        f.write(f"WIDTH {w}\n".encode())
        f.write(f"HEIGHT {h}\n".encode())
        f.write(b"DEPTH 4\nMAXVAL 255\nTUPLTYPE RGB_ALPHA\nENDHDR\n")
        for y in range(h):
            row = bytearray()
            for x in range(w):
                r, g, b, a = get(x, y)
                row += bytes([r, g, b, a])
            f.write(row)


# 6x8 "glyph" bitmaps: 0 = ink, 1 = paper. Four distinct shapes; each
# repeats many times per line so every patch has >= 2 occurrences.
GLYPHS = [
    [
        "011110",
        "110011",
        "000011",
        "011111",
        "110011",
        "110011",
        "110011",
        "000000",
    ],
    [
        "011110",
        "110011",
        "111111",
        "111110",
        "110000",
        "110011",
        "011110",
        "000000",
    ],
    [
        "111111",
        "001100",
        "001100",
        "001100",
        "001100",
        "001100",
        "001100",
        "000000",
    ],
    [
        "011111",
        "110000",
        "110000",
        "011110",
        "000011",
        "000011",
        "011110",
        "000000",
    ],
]


def glyph_color(gi):
    # dark ink, slightly different per glyph so patches differ
    return (30 + 7 * gi, 25 + 3 * gi, 35 + 11 * gi)


def text_image(w, h, seed):
    """White page with `text lines`: rows of repeated glyphs.

    Layout: lines at y0 in {40, 88, 136, 184} (16px tall bands); each line
    carries 15 glyph cells of 16px (glyph 6x8 at cell offset (5,4)).
    Everything else is flat white (4x4 seeds, flood-fill background).
    """
    lines = [40, 88, 136, 184] if h >= 200 else [24, 64, 104, 144]
    n_cells = 15 if w >= 240 else 11
    cell = 16

    def get(x, y):
        for li, ly in enumerate(lines):
            if not (ly <= y < ly + cell):
                continue
            ci = (x - 8) // cell
            if 0 <= ci < n_cells:
                lx = (x - 8) % cell
                ly2 = y - ly
                gi = (ci * 3 + li * 7 + seed) % 4
                pat = GLYPHS[gi]
                if 4 <= ly2 < 12 and 5 <= lx < 11:
                    if pat[ly2 - 4][lx - 5] == "0":
                        return glyph_color(gi)
        return (255, 255, 255)

    return get


def text_image_alpha(w, h, seed):
    getc = text_image(w, h, seed)

    def get(x, y):
        r, g, b = getc(x, y)
        # nontrivial alpha: 255 paper, 128 ink. Patches must not touch it
        # (the reference frame carries a zero alpha plane; blend mode on
        # the extra channel is kNone).
        a = 255 if (r, g, b) == (255, 255, 255) else 128
        return (r, g, b, a)

    return get


def upscale8(c):
    return c * 257


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    write_ppm(f"{out}/text_like.ppm", 256, 256, text_image(256, 256, 1))
    write_ppm(f"{out}/text_like2.ppm", 320, 192, text_image(320, 192, 2))
    # same-size variant for animation tests (256x256, different seed)
    write_ppm(f"{out}/text_like_b.ppm", 256, 256, text_image(256, 256, 5))
    write_pam_alpha(f"{out}/text_like_alpha.pam", 256, 256, text_image_alpha(256, 256, 3))
    write_ppm16(
        f"{out}/text_like_16b.ppm",
        256,
        256,
        lambda x, y: tuple(upscale8(c) for c in text_image(256, 256, 4)(x, y)),
    )
    print("OK")


if __name__ == "__main__":
    main()
