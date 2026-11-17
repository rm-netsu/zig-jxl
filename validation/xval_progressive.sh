#!/bin/bash
# Progressive-lossless (-p) and TOC-permutation (--group_order=1) validation,
# both directions, against the reference cjxl/djxl (v0.12.0, or any local
# build on PATH).
set -u
mkdir -p /tmp/xval_prog
cd /tmp/xval_prog
PORT=/home/z/my-project/jxl-zig/zig-out/bin/jxltool
pass=0; fail=0
ok()  { pass=$((pass+1)); echo "PASS: $1"; }
bad() { fail=$((fail+1)); echo "FAIL: $1"; }

# --- inputs ---
python3 - <<'EOF'
import random
def gen(path, w, h, kind, seed):
    random.seed(seed)
    px = bytearray()
    for y in range(h):
        for x in range(w):
            if kind == 'grad':
                px += bytes([(x*2)%256, (y*2)%256, (x+y)%256])
            else:
                px += bytes(random.randrange(256) for _ in range(3))
    open(path, 'wb').write(f'P6\n{w} {h}\n255\n'.encode() + px)
gen('small.pnm', 256, 256, 'grad', 1)
gen('odd.pnm', 257, 129, 'grad', 2)
gen('noise.pnm', 512, 384, 'noise', 3)
gen('big.pnm', 1024, 768, 'grad', 4)
gen('huge.pnm', 2200, 1500, 'noise', 5)
EOF

echo "--- 1. cjxl -d 0 -p (progressive lossless) -> port decode == djxl ---"
for img in small.pnm odd.pnm noise.pnm big.pnm; do
  for e in 1 2 5 7; do
    n="${img%.pnm}_e$e"
    cjxl $img $n.jxl -d 0 -e $e -p >/dev/null 2>&1 || { echo "skip $n (cjxl)"; continue; }
    djxl $n.jxl ${n}_ref.pnm >/dev/null 2>&1 || { bad "djxl $n"; continue; }
    $PORT decode $n.jxl ${n}_port.pnm >/dev/null 2>&1 || { bad "port decode $n"; continue; }
    cmp -s ${n}_ref.pnm ${n}_port.pnm && ok "$n" || bad "pixels $n"
  done
done

echo "--- 2. cjxl --group_order=1 (TOC permutation) -> port decode == djxl ---"
for img in big.pnm; do
  for e in 5 7; do
    n="${img%.pnm}_go_e$e"
    cjxl $img $n.jxl -d 0 -e $e --group_order=1 >/dev/null 2>&1 || { echo "skip $n"; continue; }
    djxl $n.jxl ${n}_ref.pnm >/dev/null 2>&1 || { bad "djxl $n"; continue; }
    $PORT decode $n.jxl ${n}_port.pnm >/dev/null 2>&1 || { bad "port decode $n"; continue; }
    cmp -s ${n}_ref.pnm ${n}_port.pnm && ok "$n" || bad "pixels $n"
  done
done

echo "--- 3. cjxl -p + --group_order=1 combined -> port decode == djxl ---"
for e in 5 9; do
  n="big_pgo_e$e"
  cjxl big.pnm $n.jxl -d 0 -e $e -p --group_order=1 >/dev/null 2>&1 || { echo "skip $n"; continue; }
  djxl $n.jxl ${n}_ref.pnm >/dev/null 2>&1 || { bad "djxl $n"; continue; }
  $PORT decode $n.jxl ${n}_port.pnm >/dev/null 2>&1 || { bad "port decode $n"; continue; }
  cmp -s ${n}_ref.pnm ${n}_port.pnm && ok "$n" || bad "pixels $n"
done

echo "--- 4. port encode --progressive -> djxl == source, port == source ---"
for img in small.pnm odd.pnm big.pnm; do
  for gss in 1 0 3; do
    n="${img%.pnm}_prog_gss$gss"
    $PORT encode $img $n.jxl --progressive --group-size-shift $gss >/dev/null 2>&1 || { bad "encode $n"; continue; }
    djxl $n.jxl ${n}_ref.pnm >/dev/null 2>&1 || { bad "djxl $n"; continue; }
    cmp -s ${n}_ref.pnm $img && ok "$n djxl" || bad "$n djxl pixels"
    $PORT decode $n.jxl ${n}_port.pnm >/dev/null 2>&1 || { bad "port decode $n"; continue; }
    cmp -s ${n}_port.pnm $img && ok "$n self" || bad "$n self pixels"
  done
done

echo "--- 5. regression: plain and -R 1 stills unaffected ---"
for img in small.pnm big.pnm; do
  for opts in "" "--responsive 1"; do
    n="${img%.pnm}_regr$(echo $opts | tr -d ' -')"
    $PORT encode $img $n.jxl $opts >/dev/null 2>&1 || { bad "encode $n"; continue; }
    djxl $n.jxl ${n}_ref.pnm >/dev/null 2>&1 || { bad "djxl $n"; continue; }
    cmp -s ${n}_ref.pnm $img && ok "$n" || bad "$n pixels"
  done
done

echo ""
echo "pass=$pass fail=$fail"
