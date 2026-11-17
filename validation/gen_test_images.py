#!/usr/bin/env python3
"""Generates PNM/PGM test images for cross-validation."""
import sys, os

def clamp8(v):
    return max(0, min(255, int(v)))

def make_rgb(w, h, path):
    rows = []
    for y in range(h):
        row = bytearray()
        for x in range(w):
            r = (x * 255) // max(1, w - 1)
            g = (y * 255) // max(1, h - 1)
            b = clamp8(((x * x + y * y) // max(1, (w + h) // 4)) % 256)
            row += bytes([r, g, b])
        rows.append(bytes(row))
    with open(path, 'wb') as f:
        f.write(f"P6\n{w} {h}\n255\n".encode())
        for r in rows:
            f.write(r)

def make_gray(w, h, path):
    with open(path, 'wb') as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        for y in range(h):
            row = bytearray()
            for x in range(w):
                row.append(clamp8((x * 3 + y * 5) % 256))
            f.write(bytes(row))

def make_rgb16(w, h, path):
    import struct
    with open(path, 'wb') as f:
        f.write(f"P6\n{w} {h}\n65535\n".encode())
        for y in range(h):
            row = bytearray()
            for x in range(w):
                r = (x * 65535) // max(1, w - 1)
                g = (y * 65535) // max(1, h - 1)
                b = ((x + y) * 997) % 65536
                row += struct.pack('>HHH', r, g, b)
            f.write(bytes(row))

if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/xval'
    os.makedirs(out, exist_ok=True)
    make_rgb(77, 50, f'{out}/rgb8.pnm')
    make_gray(64, 64, f'{out}/gray8.pnm')
    make_rgb16(40, 30, f'{out}/rgb16.pnm')
    make_rgb(300, 200, f'{out}/rgb_big.pnm')
    print("images written to", out)
