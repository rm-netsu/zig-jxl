#!/bin/bash
# Lossy VarDCT patches: cross-validation against libjxl v0.12.0.
#
#  A. port -d D --patches=1 (text-like RGB, 8/16-bit, multi-group)
#       -> port decode == djxl decode (linear PFM, float noise level)
#  B. port -d D --patches=0 (the no-patch negative case)
#       -> port decode == djxl decode
#  C. patch effectiveness: --patches=1 files shrink vs --patches=0
#  D. cjxl -d D -e 1 --epf 0 [--gaborish 0] --patches=1 (the DCT-8-only
#     cjxl subset) -> port decode == djxl decode
#  E. 8-bit PPM outputs agree within +-1 (djxl dithers)
#  F. default cjxl lossy (varblocks + EPF) is rejected with
#     UnsupportedFrameFeatures (documented restriction)
set -u
cd "$(dirname "$0")/.."
PORT=./zig-out/bin/jxltool
CJXL=${CJXL:-/tmp/libjxl-build/tools/cjxl}
DJXL=${DJXL:-/tmp/libjxl-build/tools/djxl}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
PASS=0; FAIL=0
ok()   { PASS=$((PASS+1)); echo "ok   - $1"; }
bad()  { FAIL=$((FAIL+1)); echo "FAIL - $1"; }

compare_pfm() { # <a.pfm> <b.pfm> <tolerance> <label>
  local _a="$1" _b="$2" _tol="$3"
  python3 - "$_a" "$_b" "$_tol" <<'EOF'
import struct, sys
def read_pfm(path):
    with open(path, 'rb') as f:
        data = f.read()
    parts = data.split(b'\n', 3)
    w, h = map(int, parts[1].split())
    vals = struct.unpack('>' + 'f' * (w*h*3), parts[3])
    rows = [vals[y*w*3:(y+1)*w*3] for y in range(h)]
    rows.reverse()
    return w, h, [v for row in rows for v in row]
wa, ha, a = read_pfm(sys.argv[1])
wb, hb, b = read_pfm(sys.argv[2])
tol = float(sys.argv[3])
assert (wa, ha) == (wb, hb), "dimension mismatch"
maxd = max((abs(x - y) for x, y in zip(a, b)), default=0.0)
sys.exit(0 if maxd <= tol else 1)
EOF
}
compare_ppm() { # <a.ppm> <b.ppm>  (max +-1 like djxl's dither)
  local _a="$1" _b="$2"
  python3 - "$_a" "$_b" <<'EOF'
import sys
def read_ppm(path):
    with open(path, 'rb') as f:
        return f.read().split(b'\n', 3)[3]
a, b = read_ppm(sys.argv[1]), read_ppm(sys.argv[2])
assert len(a) == len(b)
sys.exit(0 if max((abs(x - y) for x, y in zip(a, b)), default=0) <= 1 else 1)
EOF
}

# ---------------------------------------------------------------- corpus
python3 validation/gen_patch_images.py >/dev/null 2>&1
cp /tmp/text_like.ppm "$TMP/" 2>/dev/null || true
[ -f "$TMP/text_like.ppm" ] || cp text_like.ppm "$TMP/" 2>/dev/null || \
  python3 - <<'EOF'
# fallback: a small text-like image
glyph = [[1,1,1,1,1,1]] + [[0,0,1,1,0,0]] * 7
W, H = 256, 256
with open('/tmp/text_like.ppm', 'wb') as f:
    f.write(f"P6\n{W} {H}\n255\n".encode())
    f.write(bytes(200 for _ in range(W*H*3)))
with open('/tmp/text_like.ppm', 'r+b') as f:
    f.seek(len(f"P6\n{W} {H}\n255\n"))
    data = bytearray(f.read())
    for gy in range(1, 12):
        for gx in range(1, 10):
            for j, row in enumerate(glyph):
                for i, v in enumerate(row):
                    if v:
                        off = ((gy*20 + j)*W + gx*24 + i)*3
                        data[off:off+3] = bytes([30, 25, 45])
    f.seek(len(f"P6\n{W} {H}\n255\n"))
    f.write(data)
EOF
cp /tmp/text_like.ppm "$TMP/"
cp /tmp/text_like_16b.ppm "$TMP/" 2>/dev/null || true
# A multi-group image (777x403, two AC groups).
python3 - "$TMP/text_big.ppm" <<'EOF'
import sys
glyph = [[1,1,1,1,1,1]] + [[0,0,1,1,0,0]] * 7
W, H = 777, 403
data = bytearray(b'\xc8' * (W*H*3))
for gy in range(1, 20):
    for gx in range(1, 32):
        for j, row in enumerate(glyph):
            for i, v in enumerate(row):
                if v:
                    x, y = gx*24 + i, gy*16 + j
                    if x < W and y < H:
                        off = (y*W + x)*3
                        data[off:off+3] = bytes([20, 30, 60])
with open(sys.argv[1], 'wb') as f:
    f.write(f"P6\n{W} {H}\n255\n".encode() + bytes(data))
EOF

# ------------------------------------------------------- A/B/C: port files
for spec in "text_like.ppm 1" "text_like.ppm 0.5" "text_big.ppm 1" "text_big.ppm 3"; do
  set -- $spec; img=$1; d=$2
  base="${img%.ppm}_d${d}"
  $PORT encode "$TMP/$img" "$TMP/${base}_p.jxl" -d $d --patches=1 >/dev/null 2>&1 \
    && ok "encode $img -d $d --patches=1" || bad "encode $img -d $d --patches=1"
  $PORT encode "$TMP/$img" "$TMP/${base}_n.jxl" -d $d --patches=0 >/dev/null 2>&1 \
    || bad "encode $img -d $d --patches=0"
  # Self decode (integer sRGB output).
  $PORT decode "$TMP/${base}_p.jxl" "$TMP/${base}_own.ppm" >/dev/null 2>&1 \
    && ok "self decode ${base}_p" || bad "self decode ${base}_p"
  # Linear agreement with djxl.
  $PORT decode "$TMP/${base}_p.jxl" "$TMP/${base}_own.pfm" --linear >/dev/null 2>&1
  $PORT decode "$TMP/${base}_n.jxl" "$TMP/${base}_n_own.pfm" --linear >/dev/null 2>&1
  $DJXL "$TMP/${base}_p.jxl" "$TMP/${base}_p_djxl.pfm" --color_space=RGB_D65_SRG_Rel_Lin >/dev/null 2>&1
  $DJXL "$TMP/${base}_n.jxl" "$TMP/${base}_n_djxl.pfm" --color_space=RGB_D65_SRG_Rel_Lin >/dev/null 2>&1
  compare_pfm "$TMP/${base}_own.pfm" "$TMP/${base}_p_djxl.pfm" 5e-5 \
    && ok "port == djxl (linear, patches)  $img d=$d" \
    || bad "port == djxl (linear, patches)  $img d=$d"
  compare_pfm "$TMP/${base}_n_own.pfm" "$TMP/${base}_n_djxl.pfm" 5e-5 \
    && ok "port == djxl (linear, no patch) $img d=$d" \
    || bad "port == djxl (linear, no patch) $img d=$d"
  # 8-bit PPM within +-1 (djxl dithers).
  $DJXL "$TMP/${base}_p.jxl" "$TMP/${base}_p_djxl.ppm" >/dev/null 2>&1
  compare_ppm "$TMP/${base}_own.ppm" "$TMP/${base}_p_djxl.ppm" \
    && ok "port PPM == djxl PPM (+-1)      $img d=$d" \
    || bad "port PPM == djxl PPM (+-1)      $img d=$d"
  # Effectiveness.
  sp=$(stat -c%s "$TMP/${base}_p.jxl"); sn=$(stat -c%s "$TMP/${base}_n.jxl")
  if [ "$sp" -lt "$sn" ]; then
    ok "patches shrink $img d=$d ($sp < $sn)"
  else
    bad "patches shrink $img d=$d ($sp >= $sn)"
  fi
done

# 16-bit input.
if [ -f "$TMP/text_like_16b.ppm" ]; then
  $PORT encode "$TMP/text_like_16b.ppm" "$TMP/b16.jxl" -d 1 --patches=1 >/dev/null 2>&1 \
    && ok "encode 16-bit -d 1 --patches=1" || bad "encode 16-bit -d 1 --patches=1"
  $PORT decode "$TMP/b16.jxl" "$TMP/b16_own.pfm" --linear >/dev/null 2>&1
  $DJXL "$TMP/b16.jxl" "$TMP/b16_djxl.pfm" --color_space=RGB_D65_SRG_Rel_Lin >/dev/null 2>&1
  compare_pfm "$TMP/b16_own.pfm" "$TMP/b16_djxl.pfm" 5e-5 \
    && ok "port == djxl (16-bit lossy patches)" \
    || bad "port == djxl (16-bit lossy patches)"
fi

# ------------------------------------------------------------- D: cjxl files
for spec in "--patches=1" "--patches=1 --gaborish 0" "--patches=0"; do
  $CJXL "$TMP/text_like.ppm" "$TMP/cjxl$RANDOM.jxl" -d 1 -e 1 --epf 0 $spec >/dev/null 2>&1
  f=$(ls -t "$TMP"/cjxl*.jxl | head -1)
  if $PORT decode "$f" "$TMP/cjxl_own.pfm" --linear >/dev/null 2>&1; then
    $DJXL "$f" "$TMP/cjxl_djxl.pfm" --color_space=RGB_D65_SRG_Rel_Lin >/dev/null 2>&1
    compare_pfm "$TMP/cjxl_own.pfm" "$TMP/cjxl_djxl.pfm" 5e-5 \
      && ok "cjxl file (-e 1 --epf 0 $spec) decodes == djxl" \
      || bad "cjxl file (-e 1 --epf 0 $spec) decodes == djxl"
  else
    bad "cjxl file (-e 1 --epf 0 $spec) decode"
  fi
done

# --------------------------------------------------- F: default cjxl reject
$CJXL "$TMP/text_like.ppm" "$TMP/cjxl_default.jxl" -d 1 >/dev/null 2>&1
if $PORT decode "$TMP/cjxl_default.jxl" "$TMP/x.ppm" >/dev/null 2>&1; then
  bad "default cjxl lossy (varblocks+EPF) should be rejected"
else
  ok "default cjxl lossy (varblocks+EPF) rejected (documented restriction)"
fi

echo "----------------------------------------"
echo "PASS=$PASS FAIL=$FAIL"
[ "$FAIL" -eq 0 ]
