#!/usr/bin/env python3
"""Compares port decode-anim output (PAMs + frames.txt) against a djxl APNG."""
import sys, glob, struct
from PIL import Image

def read_pam(path):
    d = open(path, 'rb').read()
    if d.startswith(b'P7'):
        pos = 2
        fields = {}
        while True:
            nl = d.index(b'\n', pos)
            line = d[pos:nl]
            pos = nl + 1
            if line == b'ENDHDR': break
            if not line or line.startswith(b'#'): continue
            parts = line.split()
            fields[parts[0]] = parts[1:]
        w = int(fields[b'WIDTH'][0]); h = int(fields[b'HEIGHT'][0])
        depth = int(fields[b'DEPTH'][0]); maxval = int(fields[b'MAXVAL'][0])
        data = d[pos:]
        return w, h, depth, maxval, data
    # P5 (gray) / P6 (rgb)
    magic = d[:2]
    parts = d.split(b'\n', 3)
    dims = parts[1].split()
    w, h = int(dims[0]), int(dims[1])
    maxval = int(parts[2])
    depth = 1 if magic == b'P5' else 3
    data = parts[3]
    return w, h, depth, maxval, data

def main():
    tag = sys.argv[1]
    ref = Image.open(f'{tag}_ref.apng')
    n_ref = ref.n_frames
    port_files = sorted(glob.glob(f'{tag}_*.pam'))
    if len(port_files) != n_ref:
        print(f'FRAME COUNT MISMATCH: port={len(port_files)} ref={n_ref}')
        return 1
    durations = [float(x) for x in open(f'{tag}.frames.txt').read().split()]
    for i in range(n_ref):
        ref.seek(i)
        fr = ref.convert('RGBA')
        w, h, depth, maxval, data = read_pam(port_files[i])
        if (w, h) != fr.size:
            print(f'frame {i}: SIZE MISMATCH port={w}x{h} ref={fr.size}')
            return 1
        if depth not in (3, 4):
            print(f'frame {i}: unexpected depth {depth}')
            return 1
        for y in range(h):
            for x in range(w):
                pr = data[(y*w+x)*depth:(y*w+x)*depth+depth]
                rgba = (pr[0], pr[1], pr[2], pr[3] if depth == 4 else 255)
                rr = fr.getpixel((x, y))
                if rgba != rr:
                    print(f'PIXEL MISMATCH frame {i} at ({x},{y}): port={rgba} ref={rr}')
                    return 1
        ref_dur = ref.info.get('duration', 0)
        if abs(ref_dur - durations[i]) > 1.0:
            print(f'DURATION MISMATCH frame {i}: ref={ref_dur} port={durations[i]}')
            return 1
    print(f'OK {n_ref} frames')
    return 0

if __name__ == '__main__':
    sys.exit(main())
