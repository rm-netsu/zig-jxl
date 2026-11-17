#!/bin/bash
# Float-sample (PFM fp32/fp16) cross-validation vs libjxl 0.12.0:
# - port encode (fp32/fp16, options matrix) -> djxl PFM (values exact)
# - resampled files (lossy box+NLEE): port decode == djxl decode (exact)
# - cjxl fp32 (efforts 3..9, -R 1; cjxl itself fails on PFM below e3)
#   -> port decode == djxl decode (values exact)
# - port fp16 -> djxl PFM (values exact)
# - regression: int stills + jpeg transcode quick sweep
set -u
mkdir -p /tmp/xval_float && cd /tmp/xval_float
export PATH=/home/z/my-project/work/ref/libjxl-0.12.0/build/tools:$PATH
PORT=/home/z/my-project/work/jxl-zig/zig-out/bin/jxltool
PY=python3
pass=0; fail=0; skip=0
ok()   { pass=$((pass+1)); }
bad()  { fail=$((fail+1)); echo "FAIL: $1"; }
skipped() { skip=$((skip+1)); }

# PFM value comparison (header scale/endianness may differ; rows bottom-up).
cat > pfncmp.py <<'EOF'
import struct, sys
def read_pfm(p):
    d = open(p,'rb').read()
    nl1 = d.index(b'\n'); nl2 = d.index(b'\n', nl1+1); nl3 = d.index(b'\n', nl2+1)
    magic = d[:nl1]; dims = d[nl1+1:nl2]; scale = d[nl2+1:nl3]
    w, h = map(int, dims.split())
    be = float(scale) > 0
    nch = 3 if magic == b'PF' else 1
    n = w*h*nch
    fmt = ('>' if be else '<') + str(n) + 'f'
    vals = list(struct.unpack(fmt, d[nl3+1:nl3+1+4*n]))
    rows = [vals[r*w*nch:(r+1)*w*nch] for r in range(h)]
    rows.reverse()
    return magic, w, h, [v for row in rows for v in row]
a = read_pfm(sys.argv[1]); b = read_pfm(sys.argv[2])
sys.exit(0 if a == b else 1)
EOF

echo "=== generating PFM corpus ==="
$PY - <<'EOF'
import struct
def write_pfm(path, w, h, fn):
    hdr = b'PF\n%d %d\n-1.0\n' % (w, h)
    rows = []
    for y in range(h):
        row = b''
        for x in range(w):
            for c in range(3):
                row += struct.pack('<f', fn(x, y, c))
        rows.append(row)
    open(path,'wb').write(hdr + b''.join(reversed(rows)))
def write_pfm_gray(path, w, h, fn):
    hdr = b'Pf\n%d %d\n-1.0\n' % (w, h)
    rows = []
    for y in range(h):
        row = b''
        for x in range(w):
            row += struct.pack('<f', fn(x, y, 0))
        rows.append(row)
    open(path,'wb').write(hdr + b''.join(reversed(rows)))
write_pfm('hdr.pfm', 60, 40, lambda x,y,c: (x*0.05 - 1.5) + (y*0.02 if c==1 else -y*0.01) + c*0.5)
write_pfm_gray('gray.pfm', 60, 40, lambda x,y,c: -2.0 + 4.0*(x*0.5+y*0.3)/(30+12))
write_pfm('big.pfm', 600, 300, lambda x,y,c: (x*0.01 - 3.0)*(1.0 if c==0 else -0.5) + y*0.005 + c)
write_pfm('smooth.pfm', 200, 150, lambda x,y,c: (x+y)/349.0 - 0.5 + c*0.25)
write_pfm('extreme.pfm', 33, 21, lambda x,y,c: [1e-30, -1e-30, 1e-15, -1.0, 0.0, -0.0, 65504.0, -65504.0, 1e30, -1e30, 3.4e38, -3.4e38, 0.5][(x*3+y+c) % 13])
f16vals = [-3.5, -2.0, -0.75, -0.5, -0.25, 0.0, 0.125, 0.25, 0.5, 1.0, 2.5, 4.0, 8.0]
write_pfm('f16grid.pfm', 60, 40, lambda x,y,c: f16vals[(x*3+y*2+c) % len(f16vals)])
write_pfm('f16big.pfm', 520, 300, lambda x,y,c: f16vals[(x+y*3+c) % len(f16vals)])
EOF

echo "=== 1. port fp32 encode -> djxl PFM (options matrix) ==="
for img in hdr.pfm gray.pfm big.pfm smooth.pfm extreme.pfm; do
  name=${img%.pfm}
  for opts in "" "--group-size-shift 0" "--responsive 1" "--colorspace 5"; do
    tag=$(echo "$opts" | tr -d ' -')
    $PORT encode $img p_${name}${tag}.jxl $opts >/dev/null 2>&1 || { bad "encode $name $opts"; continue; }
    djxl p_${name}${tag}.jxl p_${name}${tag}.pfm >/dev/null 2>&1 || { bad "djxl $name $opts"; continue; }
    $PY pfncmp.py $img p_${name}${tag}.pfm && ok || bad "values $name $opts"
  done
  # Lossy resampling: port decode must equal djxl decode.
  $PORT encode $img rs_${name}.jxl --resampling 2 >/dev/null 2>&1 || { bad "encode rs $name"; continue; }
  djxl rs_${name}.jxl rs_${name}_d.pfm >/dev/null 2>&1 || { bad "djxl rs $name"; continue; }
  $PORT decode rs_${name}.jxl rs_${name}_p.pfm >/dev/null 2>&1 || { bad "port dec rs $name"; continue; }
  $PY pfncmp.py rs_${name}_d.pfm rs_${name}_p.pfm && ok || bad "rs port-vs-djxl $name"
done

echo "=== 2. port fp16 encode -> djxl PFM ==="
for img in f16grid.pfm f16big.pfm; do
  name=${img%.pfm}
  for opts in "" "--group-size-shift 0" "--responsive 1"; do
    tag=$(echo "$opts" | tr -d ' -')
    $PORT encode $img h_${name}${tag}.jxl --float16 $opts >/dev/null 2>&1 || { bad "encode16 $name $opts"; continue; }
    djxl h_${name}${tag}.jxl h_${name}${tag}.pfm >/dev/null 2>&1 || { bad "djxl16 $name $opts"; continue; }
    $PY pfncmp.py $img h_${name}${tag}.pfm && ok || bad "values16 $name $opts"
  done
done

echo "=== 3. cjxl fp32 -> port decode == djxl decode ==="
# cjxl itself fails on PFM input at efforts 1-2 (and intermittently at 3-7
# for some content); those cases are skipped, not counted as failures.
for img in hdr.pfm gray.pfm big.pfm smooth.pfm extreme.pfm; do
  name=${img%.pfm}
  for e in 3 4 5 7 9; do
    for R in "" "-R 1"; do
      tag="e${e}$(echo $R | tr -d ' -')"
      cjxl $img c_${name}${tag}.jxl -d 0 -e $e $R >/dev/null 2>&1 || { skipped; continue; }
      djxl c_${name}${tag}.jxl c_${name}${tag}_ref.pfm >/dev/null 2>&1 || { bad "djxl ref $name $tag"; continue; }
      $PORT decode c_${name}${tag}.jxl c_${name}${tag}_port.pfm >/dev/null 2>&1 || { bad "port decode $name $tag"; continue; }
      $PY pfncmp.py c_${name}${tag}_ref.pfm c_${name}${tag}_port.pfm && ok || bad "values $name $tag"
    done
  done
done

echo "=== 4. port self roundtrip (fp32 + fp16, exact paths) ==="
for img in hdr.pfm big.pfm extreme.pfm f16grid.pfm f16big.pfm; do
  name=${img%.pfm}
  f16=""
  [ "$name" != "${name#f16}" ] && f16="--float16"
  for opts in "" "--responsive 1"; do
    tag=$(echo "$opts" | tr -d ' -')
    $PORT encode $img s_${name}${tag}.jxl $f16 $opts >/dev/null 2>&1 || { bad "self enc $name $tag"; continue; }
    $PORT decode s_${name}${tag}.jxl s_${name}${tag}.pfm >/dev/null 2>&1 || { bad "self dec $name $tag"; continue; }
    $PY pfncmp.py $img s_${name}${tag}.pfm && ok || bad "self values $name $tag"
  done
done

echo "=== 5. regression: int stills ==="
$PY /home/z/my-project/work/jxl-zig/validation/gen_test_images.py /tmp/xval_float >/dev/null 2>&1 || true
for img in rgb8.pnm gray8.pnm rgb_big.pnm; do
  [ -f "$img" ] || continue
  for opts in "" "--responsive 1" "--group-size-shift 0"; do
    tag=$(echo "$opts" | tr -d ' -')
    $PORT encode $img r_${img%.pnm}${tag}.jxl $opts >/dev/null 2>&1 || { bad "reg enc $img $opts"; continue; }
    djxl r_${img%.pnm}${tag}.jxl r_${img%.pnm}${tag}.pnm >/dev/null 2>&1 || { bad "reg djxl $img $opts"; continue; }
    $PORT decode r_${img%.pnm}${tag}.jxl r_${img%.pnm}${tag}_p.pnm >/dev/null 2>&1 || { bad "reg dec $img $opts"; continue; }
    cmp -s r_${img%.pnm}${tag}.pnm r_${img%.pnm}${tag}_p.pnm && ok || bad "reg pixels $img $opts"
  done
done

echo "=== 6. regression: jpeg transcode (djxl 0.12 auto-reconstruction) ==="
cd /home/z/my-project/work/jxl-zig/src/jpeg/testdata
for jpg in baseline_444.jpg progressive_420.jpg gray_baseline.jpg big_444.jpg two_dqt_444.jpg; do
  n=${jpg%.jpg}
  $PORT jpeg-transcode $jpg /tmp/xval_float/j_$n.jxl >/dev/null 2>&1 || { bad "jt enc $jpg"; continue; }
  djxl /tmp/xval_float/j_$n.jxl /tmp/xval_float/j_$n.jpg >/dev/null 2>&1 || { bad "jt djxl $jpg"; continue; }
  cmp -s $jpg /tmp/xval_float/j_$n.jpg && ok || bad "jt bytes $jpg"
done

echo
echo "PASS=$pass FAIL=$fail SKIP(cjxl cannot encode: $skip)"
