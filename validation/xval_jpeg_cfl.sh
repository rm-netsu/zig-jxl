#!/bin/bash
# Round-5 validation: encoder-side chroma-from-luma + custom coefficient
# orders for the JPEG transcode path (vs. libjxl v0.12.0 cjxl/djxl).
#
# 1. port jpeg-transcode (cfl + coeff orders ON by default, the cjxl
#    default-effort shape) -> djxl --jpeg_reconstruction byte-exact.
# 2. same files -> djxl pixel decode (PNG) sanity.
# 3. cjxl transcodes (default + effort sweep) -> port jpeg-untranscode
#    byte-exact (decoder regression: cjxl's own CfL/orders streams).
# 4. size table: port old shape (--no-cfl --no-coeff-reorder) vs port new
#    default vs cjxl default.
set -u
cd /tmp/xval
export PATH=/home/z/my-project/work/ref/libjxl-0.12.0/build/tools:$PATH
PORT=/home/z/my-project/work/jxl-zig/zig-out/bin/jxltool
pass=0; fail=0
ok()  { pass=$((pass+1)); }
bad() { fail=$((fail+1)); echo "FAIL: $1"; }

CORPUS="/tmp/jpegcorpus/*.jpg /tmp/xval/testdata/*.jpg"
shopt -s nullglob

echo "--- 1. port transcode (cfl+orders) -> djxl byte-exact JPEG ---"
for jpg in $CORPUS; do
  name=$(basename "$jpg" .jpg)
  $PORT jpeg-transcode "$jpg" n_$name.jxl >/dev/null 2>&1 || { bad "transcode $name"; continue; }
  rm -f n_$name.rec.jpg
  djxl n_$name.jxl n_$name.rec.jpg >/dev/null 2>&1
  if [ -f n_$name.rec.jpg ]; then
    cmp -s "$jpg" n_$name.rec.jpg && ok || bad "djxl jpeg bytes $name"
  else
    bad "djxl no jpeg $name"
  fi
  # pixel decode sanity
  djxl n_$name.jxl n_$name.png >/dev/null 2>&1 && ok || bad "djxl pixels $name"
done

echo "--- 2. cjxl transcodes -> port jpeg-untranscode (regression) ---"
for jpg in $CORPUS; do
  name=$(basename "$jpg" .jpg)
  for e in 7 9; do
    cjxl "$jpg" c_${name}_e$e.jxl -d 0 -e $e >/dev/null 2>&1 || { ok; continue; }  # cjxl may reject some inputs
    $PORT jpeg-untranscode c_${name}_e$e.jxl c_${name}_e$e.rec.jpg >/dev/null 2>&1 || { bad "untranscode $name e$e"; continue; }
    cmp -s "$jpg" c_${name}_e$e.rec.jpg && ok || bad "port jpeg bytes $name e$e"
  done
done

echo "--- 3. sizes: port old vs port new (cfl+orders) vs cjxl ---"
printf "%-32s %10s %10s %10s %10s\n" "file" "jpeg" "port-old" "port-new" "cjxl"
for jpg in $CORPUS; do
  name=$(basename "$jpg" .jpg)
  $PORT jpeg-transcode "$jpg" o_$name.jxl --no-cfl --no-coeff-reorder >/dev/null 2>&1 || continue
  cjxl "$jpg" r_$name.jxl -d 0 >/dev/null 2>&1 || continue
  printf "%-32s %10d %10d %10d %10d\n" "$name" \
    "$(stat -c%s "$jpg")" "$(stat -c%s o_$name.jxl)" \
    "$(stat -c%s n_$name.jxl)" "$(stat -c%s r_$name.jxl)"
done

echo "================================"
echo "TOTAL: pass=$pass fail=$fail"
