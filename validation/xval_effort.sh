#!/bin/bash
# Round 8: encoder completion — the effort ladder (--effort 1..10, the cjxl
# -e equivalents incl. the kKitten/kTortoise/kGlacier cost-based heuristics)
# and encoder-side TOC permutations (--group-order 1, cjxl --group_order=1),
# both validated against the reference cjxl/djxl (v0.12.0 or any local
# build on PATH).
set -u
mkdir -p /tmp/xval_effort
cd /tmp/xval_effort
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PORT=$ROOT/zig-out/bin/jxltool
DBG=$ROOT/zig-out/bin/jxldbg
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
            elif kind == 'ui':
                if x < w//3: px += bytes([128, 64, 32])
                elif x < 2*w//3: px += bytes([(x*255)//w, (y*255)//h, 100])
                else: px += bytes([random.randrange(256) for _ in range(3)])
            else:
                px += bytes(random.randrange(256) for _ in range(3))
    open(path, 'wb').write(f'P6\n{w} {h}\n255\n'.encode() + px)
gen('grad.pnm', 700, 500, 'grad', 1)
gen('noise.pnm', 512, 512, 'noise', 2)
gen('ui.pnm', 800, 600, 'ui', 3)
gen('odd.pnm', 999, 777, 'grad', 4)
EOF

echo "--- 1. port --effort 1..10 -> djxl decode pixel-exact ---"
for img in grad.pnm noise.pnm ui.pnm; do
  for e in 1 2 3 4 5 6 7 8 9 10; do
    n="${img%.pnm}_e$e"
    $PORT encode $img $n.jxl --effort $e >/dev/null 2>&1 || { bad "port encode $n"; continue; }
    djxl $n.jxl ${n}_ref.ppm --quiet >/dev/null 2>&1 || { bad "djxl $n"; continue; }
    cmp -s $img ${n}_ref.ppm && ok "$img e$e djxl" || bad "pixels $img e$e"
    $PORT decode $n.jxl ${n}_self.pnm >/dev/null 2>&1 || { bad "self decode $n"; continue; }
    cmp -s $img ${n}_self.pnm && ok "$img e$e self" || bad "self pixels $img e$e"
  done
done

echo "--- 2. port --effort 8-10 option combos -> djxl pixel-exact ---"
for opts in "--effort 8 -R 1" "--effort 9 -p" "--effort 9 --group-order 1" \
            "--effort 10 --group-size-shift 0" "--effort 8 --palette-colors 0"; do
  $PORT encode grad.pnm c.jxl $opts >/dev/null 2>&1 || { bad "port encode [$opts]"; continue; }
  djxl c.jxl c_ref.ppm --quiet >/dev/null 2>&1 || { bad "djxl [$opts]"; continue; }
  cmp -s grad.pnm c_ref.ppm && ok "[$opts] djxl" || bad "pixels [$opts]"
done

echo "--- 3. cjxl -d 0 -e 8..10 -> port decode (regression) ---"
for img in grad.pnm ui.pnm; do
  for e in 8 9 10; do
    n="${img%.pnm}_c$e"
    cjxl $img $n.jxl -d 0 -e $e --quiet >/dev/null 2>&1 || { echo "skip $n (cjxl)"; continue; }
    $PORT decode $n.jxl ${n}_port.pnm >/dev/null 2>&1 || { bad "port decode $n"; continue; }
    cmp -s $img ${n}_port.pnm && ok "$n port" || bad "pixels $n"
  done
done

echo "--- 4. port --group-order 1 -> djxl pixel-exact + self decode ---"
for img in grad.pnm noise.pnm odd.pnm; do
  for extra in "" "-p" "--effort 9" "-R 1"; do
    $PORT encode $img g.jxl --group-order 1 $extra >/dev/null 2>&1 || { bad "port encode [$img $extra]"; continue; }
    djxl g.jxl g_ref.ppm --quiet >/dev/null 2>&1 || { bad "djxl [$img $extra]"; continue; }
    cmp -s $img g_ref.ppm && ok "[$img go: $extra] djxl" || bad "pixels [$img go: $extra]"
    $PORT decode g.jxl g_self.pnm >/dev/null 2>&1 || { bad "self [$img $extra]"; continue; }
    cmp -s $img g_self.pnm && ok "[$img go: $extra] self" || bad "self pixels [$img $extra]"
  done
done

echo "--- 5. permutation parity: port --group-order 1 == cjxl --group_order=1 ---"
while read -r W H SH; do
  python3 - "$W" "$H" <<'EOF'
import random, sys
random.seed(1)
w,h=int(sys.argv[1]),int(sys.argv[2])
open('tmp.pnm','wb').write(f'P6\n{w} {h}\n255\n'.encode() +
    bytes(random.randrange(256) for _ in range(w*h*3)))
EOF
  $PORT encode tmp.pnm tmp_p.jxl --group-order 1 --group-size-shift "$SH" >/dev/null 2>&1
  cjxl tmp.pnm tmp_c.jxl --group_order=1 --modular_group_size="$SH" -d 0 --quiet >/dev/null 2>&1
  P1=$($DBG --toc tmp_p.jxl 2>&1 | grep '^permutation')
  P2=$($DBG --toc tmp_c.jxl 2>&1 | grep '^permutation')
  [ "$P1" = "$P2" ] && [ -n "$P1" ] && ok "perm parity ${W}x${H} s$SH" || bad "perm parity ${W}x${H} s$SH"
done <<'SPECS'
512 512 1
1024 768 1
1280 256 1
256 1280 1
768 768 1
640 480 2
1024 768 0
999 777 1
SPECS

echo "--- 6. progressive 2-pass permutation parity ---"
$PORT encode odd.pnm pp_p.jxl -p --group-order 1 >/dev/null 2>&1
cjxl odd.pnm pp_c.jxl --group_order=1 -d 0 -p --quiet >/dev/null 2>&1
P1=$($DBG --toc pp_p.jxl 2>&1 | grep '^permutation')
P2=$($DBG --toc pp_c.jxl 2>&1 | grep '^permutation')
[ "$P1" = "$P2" ] && ok "progressive perm parity" || bad "progressive perm parity"

echo "--- 7. JPEG transcode with --group-order: byte-exact both directions ---"
TD=$ROOT/src/jpeg/testdata
tdcount=0; tdfail=0
for jpg in "$TD"/*.jpg; do
  $PORT jpeg-transcode "$jpg" tj.jxl --group-order 1 >/dev/null 2>&1 || { tdfail=$((tdfail+1)); continue; }
  djxl tj.jxl tj.jpg --bits_per_sample 8 --quiet >/dev/null 2>&1 || { tdfail=$((tdfail+1)); continue; }
  if cmp -s "$jpg" tj.jpg; then tdcount=$((tdcount+1)); else tdfail=$((tdfail+1)); echo "  transcode FAIL: $jpg"; fi
done
[ "$tdfail" -eq 0 ] && ok "transcode corpus ($tdcount files byte-exact)" || bad "transcode corpus ($tdfail failed)"

echo
echo "== effort/group-order round: $pass passed, $fail failed =="
[ "$fail" -eq 0 ]
