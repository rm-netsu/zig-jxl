#!/bin/bash
# Animation cross-validation: port decode-anim vs djxl APNG (libjxl v0.12.0).
set -u
cd /tmp/xval
export PATH=/home/z/my-project/work/ref/libjxl-0.12.0/build/tools:$PATH
PORT=/home/z/my-project/work/jxl-zig/zig-out/bin/jxltool
pass=0; fail=0
ok()  { pass=$((pass+1)); }
bad() { fail=$((fail+1)); echo "FAIL: $1"; }

decode_and_compare() {
  local jxl="$1" tag="$2"
  rm -f ${tag}_*.pam ${tag}.frames.txt ${tag}_ref.apng
  $PORT decode-anim "$jxl" "$tag" >/dev/null 2>&1 || { bad "decode-anim $tag"; return 1; }
  djxl "$jxl" ${tag}_ref.apng >/dev/null 2>&1 || { bad "djxl $tag"; return 1; }
  if python3 /home/z/my-project/work/jxl-zig/validation/compare_anim.py "$tag"; then ok; else bad "compare $tag"; fi
}

for t in anim_ll anim_alpha_ll anim_gray_ll anim_disp anim_e9 anim_R; do
  [ -f ${t}.jxl ] || continue
  decode_and_compare ${t}.jxl ${t}_x || true
done
echo "=========================="
echo "decode-anim validation: pass=$pass fail=$fail"
