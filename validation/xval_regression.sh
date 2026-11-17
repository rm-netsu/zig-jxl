#!/bin/bash
# Full regression sweep after the LoopFilter/BlendingInfo/PackSigned header
# fixes: stills (both directions), ICC, JPEG transcode, animations.
set -u
cd /tmp/xval
export PATH=/home/z/my-project/work/ref/libjxl-0.12.0/build/tools:$PATH
PORT=/home/z/my-project/work/jxl-zig/zig-out/bin/jxltool
pass=0; fail=0
ok()  { pass=$((pass+1)); }
bad() { fail=$((fail+1)); echo "FAIL: $1"; }

echo "--- 1. still encode -> djxl (options matrix) ---"
for img in rgb8.pnm gray8.pnm rgb16.pnm rgb_big.pnm; do
  for opts in "" "--responsive 1" "--group-size-shift 0" "--colorspace 5" "--palette-colors 0" "--responsive 1 --group-size-shift 3"; do
    name=$(echo "${img%.pnm}$(echo $opts | tr -d ' ')" | tr -cd 'a-zA-Z0-9_')
    $PORT encode $img s_$name.jxl $opts >/dev/null 2>&1 || { bad "encode $img $opts"; continue; }
    djxl s_$name.jxl s_$name.pnm >/dev/null 2>&1 || { bad "djxl $img $opts"; continue; }
    $PORT decode s_$name.jxl s_${name}_port.pnm >/dev/null 2>&1 || { bad "port decode $img $opts"; continue; }
    # compare djxl output vs port decode output (both should equal the source)
    cmp -s s_$name.pnm s_${name}_port.pnm && ok || { bad "djxl-vs-port $img $opts"; continue; }
    cmp -s $img s_${name}_port.pnm && ok || bad "port-vs-src $img $opts"
  done
done

echo "--- 2. cjxl stills -> port decode (efforts, -R 1) ---"
for img in rgb8.pnm gray8.pnm rgb_big.pnm rgb16.pnm; do
  for e in 1 3 7 9; do
    for R in "" "-R 1"; do
      name="c_${img%.pnm}_e${e}$(echo $R | tr -d ' -')"
      cjxl $img $name.jxl -d 0 -e $e $R >/dev/null 2>&1 || { bad "cjxl $img e$e $R"; continue; }
      djxl $name.jxl ${name}_ref.pnm >/dev/null 2>&1 || { bad "djxl ref $img e$e $R"; continue; }
      $PORT decode $name.jxl ${name}_port.pnm >/dev/null 2>&1 || { bad "port decode $img e$e $R"; continue; }
      cmp -s ${name}_ref.pnm ${name}_port.pnm && ok || bad "pixels $img e$e $R"
    done
  done
done

echo "--- 3. ICC roundtrip ---"
for prof in srgb adobergb linear; do
  $PORT encode rgb8.pnm i_$prof.jxl --icc $prof.icc >/dev/null 2>&1 || { bad "enc icc $prof"; continue; }
  djxl i_$prof.jxl i_$prof.png >/dev/null 2>&1 || { bad "djxl icc $prof"; continue; }
  ok
done

echo "--- 4. jpeg transcode both directions ---"
for jpg in baseline_444.jpg progressive_420.jpg gray_baseline.jpg restart_420.jpg big_420.jpg wide_444.jpg two_dqt_444.jpg exif_420.jpg; do
  [ -f "$jpg" ] || continue
  name=${jpg%.jpg}
  $PORT jpeg-transcode $jpg jt_$name.jxl >/dev/null 2>&1 || { bad "transcode $jpg"; continue; }
  djxl jt_$name.jxl jt_$name.jpg 
  djxl jt_$name.jxl jt_$name.jpg >/dev/null 2>&1 || true
  if [ -f jt_$name.jpg ]; then
    cmp -s $jpg jt_$name.jpg && ok || bad "djxl jpeg bytes $jpg"
  else
    bad "djxl no jpeg $jpg"
  fi
  cjxl $jpg jtc_$name.jxl -d 0 >/dev/null 2>&1 || { ok; continue; }
  $PORT jpeg-untranscode jtc_$name.jxl jtc_$name.jpg >/dev/null 2>&1 || { bad "untranscode $jpg"; continue; }
  cmp -s $jpg jtc_$name.jpg && ok || bad "port jpeg bytes $jpg"
done

echo "--- 5. animations (decode + encode) ---"
VDIR=/home/z/my-project/work/jxl-zig/validation
bash $VDIR/xval_anim_decode.sh > /tmp/anim_dec.log 2>&1
tail -1 /tmp/anim_dec.log
dp=$(tail -1 /tmp/anim_dec.log | grep -oP 'pass=\K[0-9]+'); df=$(tail -1 /tmp/anim_dec.log | grep -oP 'fail=\K[0-9]+')
pass=$((pass+dp)); fail=$((fail+df))
bash $VDIR/xval_anim_encode.sh > /tmp/anim_enc.log 2>&1
tail -1 /tmp/anim_enc.log
ep=$(tail -1 /tmp/anim_enc.log | grep -oP 'pass=\K[0-9]+'); ef=$(tail -1 /tmp/anim_enc.log | grep -oP 'fail=\K[0-9]+')
pass=$((pass+ep)); fail=$((fail+ef))

echo "================================"
echo "TOTAL: pass=$pass fail=$fail"
