#!/bin/bash
# Animation ENCODE validation: port encode-anim -> djxl APNG -> compare with
# source frames (pixels + durations).
set -u
cd /tmp/xval
export PATH=/home/z/my-project/work/ref/libjxl-0.12.0/build/tools:$PATH
PORT=/home/z/my-project/work/jxl-zig/zig-out/bin/jxltool
pass=0; fail=0
ok()  { pass=$((pass+1)); }
bad() { fail=$((fail+1)); echo "FAIL: $1"; }

encode_and_check() {
  local tag="$1"; shift
  local durms="$1"; shift
  rm -f ${tag}_port.jxl ${tag}_port.apng
  $PORT encode-anim ${tag}_port.jxl "$@" --duration-ms "$durms" >/dev/null 2>&1 || { bad "encode-anim $tag"; return 1; }
  djxl ${tag}_port.jxl ${tag}_port.apng >/dev/null 2>&1 || { bad "djxl decode $tag"; return 1; }
  python3 - "$tag" "$durms" <<'EOF' && ok || bad "compare $tag"
import sys, glob
from PIL import Image
tag, dur = sys.argv[1], float(sys.argv[2])
ref = Image.open(f'{tag}_port.apng')
srcs = sorted(glob.glob(f'{tag}_f*.pam')) or sorted(glob.glob(f'{tag}_f*.pnm'))
if ref.n_frames != len(srcs):
    print(f'frame count: apng={ref.n_frames} src={len(srcs)}'); sys.exit(1)
for i in range(ref.n_frames):
    ref.seek(i)
    fr = ref.convert('RGBA')
    sd = open(srcs[i],'rb').read()
    # parse P5/P6/P7
    if sd.startswith(b'P7'):
        pos = sd.index(b'ENDHDR\n') + 7
        hdr = {}
        for line in sd[:pos].split(b'\n'):
            p = line.split()
            if len(p) >= 2: hdr[p[0]] = p[1]
        w, h, depth = int(hdr[b'WIDTH']), int(hdr[b'HEIGHT']), int(hdr[b'DEPTH'])
        data = sd[pos:]
    else:
        parts = sd.split(b'\n', 3)
        w, h = map(int, parts[1].split())
        depth = 1 if sd[:2] == b'P5' else 3
        data = parts[3]
    for y in range(h):
        for x in range(w):
            pr = tuple(data[(y*w+x)*depth:(y*w+x)*depth+depth])
            rgba = (pr[0], pr[1], pr[2], pr[3] if depth == 4 else 255)
            if fr.getpixel((x,y)) != rgba:
                print(f'frame {i} pixel ({x},{y}): apng={fr.getpixel((x,y))} src={rgba}')
                sys.exit(1)
    if abs(ref.info.get('duration', 0) - dur) > 1.0:
        print(f'frame {i} duration {ref.info.get("duration")} != {dur}')
        sys.exit(1)
print(f'OK {ref.n_frames} frames')
EOF
}

encode_and_check enc_rgb 100 enc_rgb_f0.pnm enc_rgb_f1.pnm enc_rgb_f2.pnm
encode_and_check enc_alpha 120 enc_alpha_f0.pam enc_alpha_f1.pam enc_alpha_f2.pam enc_alpha_f3.pam
echo "=========================="
echo "encode-anim validation: pass=$pass fail=$fail"
