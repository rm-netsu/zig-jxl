#!/bin/bash
# Cross-validation of the FLOAT-SAMPLE (binary16) patch dictionary against
# official libjxl v0.12.0. libjxl's CLI cannot write fp16 samples (no
# --float16 flag), but its encoder API can (JxlEncoderAddImageFrame with
# JXL_TYPE_FLOAT16 + exponent_bits_per_sample=5) and its enc_modular gate
# enables the patch search for bits_per_sample <= 16 — binary16 included.
# The reference files are produced with make_fp16_patches.c against the
# locally built libjxl shared library.
#
#  A) ours --float16 --patches=1 -> ours decode  == input (value-exact)
#  B) ours --float16 --patches=1 -> djxl         == input (bitstream compat)
#  C) ours --float16 default effort ladder: patches on at >= 7 (sizes drop)
#  D) libjxl API fp16+patches -> ours decode == djxl decode (decode compat)
#  E) libjxl API fp16+patches file is lossless (djxl == input)
#  F) hostile fp16 (unrepresentable differences):
#     ours fails like libjxl (encode error), ours --patches=0 succeeds
#  G) gray (Pf) fp16 patches, multi-group frames, progressive, group sizes
#  H) fp32 still excluded from the patch search (libjxl gate 32 > 16)
set -u
cd "$(dirname "$0")/.."
JXLTOOL=${JXLTOOL:-zig-out/bin/jxltool}
CJXL=${CJXL:-/tmp/libjxl-build/tools/cjxl}
DJXL=${DJXL:-/tmp/libjxl-build/tools/djxl}
MK16=${MK16:-/tmp/patchgap/mk16}
WORK=${WORK:-$(mktemp -d)}
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$WORK/img"
python3 validation/gen_fp16_patch_images.py "$WORK/img" --big || exit 1

PASS=0
FAIL=0

report() { # name rc detail
    if [ "$2" = "0" ]; then
        PASS=$((PASS + 1)); echo "  ok   $1"
    else
        FAIL=$((FAIL + 1)); echo "  FAIL $1"; echo "       $3"
    fi
}

cmp_pfm() { # a b -> IDENTICAL/...; rc 0/1
    python3 validation/compare_pfm.py "$1" "$2"
}

SRC="$WORK/img/text_fp16.pfm"

# --- A/B: ours encode with patches, both decoders --------------------------
for img in text_fp16.pfm text_fp16_gray.pfm big_fp16.pfm; do
    B="${img%%.*}"
    echo "== $img =="
    "$JXLTOOL" encode "$WORK/img/$img" "$WORK/$B.ours.jxl" --float16 --patches=1 2>"$WORK/a.err"
    rc=$?
    if [ $rc -ne 0 ]; then
        report "A/B encode" 1 "$(tail -2 "$WORK/a.err")"
        continue
    fi
    sz=$(stat -c%s "$WORK/$B.ours.jxl")
    # no-patches control for the size assertion
    "$JXLTOOL" encode "$WORK/img/$img" "$WORK/$B.nopat.jxl" --float16 --patches=0 2>/dev/null
    szn=$(stat -c%s "$WORK/$B.nopat.jxl")
    "$JXLTOOL" decode "$WORK/$B.ours.jxl" "$WORK/$B.self.pfm" 2>/dev/null
    cmp_pfm "$WORK/img/$img" "$WORK/$B.self.pfm" >/dev/null 2>&1
    report "A ours->ours ($sz B vs $szn no-patches)" $? "$(cmp_pfm "$WORK/img/$img" "$WORK/$B.self.pfm" 2>&1 | tail -1)"
    "$DJXL" "$WORK/$B.ours.jxl" "$WORK/$B.djxl.pfm" >/dev/null 2>&1
    cmp_pfm "$WORK/img/$img" "$WORK/$B.djxl.pfm" >/dev/null 2>&1
    report "B ours->djxl" $? "$(cmp_pfm "$WORK/img/$img" "$WORK/$B.djxl.pfm" 2>&1 | tail -1)"
    if [ "$sz" -lt "$szn" ]; then
        report "patches shrink file" 0 ""
    else
        report "patches shrink file" 1 "$sz >= $szn"
    fi
done

# --- C: effort ladder, patches default (on at >= 7, off below) -------------
img=text_fp16.pfm; B=text_fp16
for eff in 5 6 7 9; do
    "$JXLTOOL" encode "$WORK/img/$img" "$WORK/$B.e$eff.jxl" --float16 -e $eff 2>/dev/null
    "$DJXL" "$WORK/$B.e$eff.jxl" "$WORK/$B.e$eff.pfm" >/dev/null 2>&1
    cmp_pfm "$WORK/img/$img" "$WORK/$B.e$eff.pfm" >/dev/null 2>&1
    report "C effort $eff -> djxl" $? ""
done
s5=$(stat -c%s "$WORK/$B.e5.jxl"); s7=$(stat -c%s "$WORK/$B.e7.jxl")
if [ "$s7" -lt "$s5" ]; then report "C patches on at e7 shrink ($s7 < $s5)" 0 ""; else report "C patches on at e7 shrink ($s7 vs $s5)" 1 ""; fi

# --- G: option matrix on fp16 patches ---------------------------------------
for opts in "--group-size-shift 0" "--group-size-shift 3" "-p" "-p --group-size-shift 0" "--colorspace 6" "-e 10"; do
    tag=$(echo "$opts" | tr -d ' =-')
    "$JXLTOOL" encode "$WORK/img/big_fp16.pfm" "$WORK/g_$tag.jxl" --float16 --patches=1 $opts 2>/dev/null
    rc=$?
    if [ $rc -ne 0 ]; then report "G $opts" 1 "encode failed"; continue; fi
    "$JXLTOOL" decode "$WORK/g_$tag.jxl" "$WORK/g_$tag.self.pfm" 2>/dev/null
    "$DJXL" "$WORK/g_$tag.jxl" "$WORK/g_$tag.djxl.pfm" >/dev/null 2>&1
    cmp_pfm "$WORK/img/big_fp16.pfm" "$WORK/g_$tag.self.pfm" >/dev/null 2>&1
    r1=$?
    cmp_pfm "$WORK/img/big_fp16.pfm" "$WORK/g_$tag.djxl.pfm" >/dev/null 2>&1
    r2=$?
    rc=$([ $r1 -eq 0 ] && [ $r2 -eq 0 ] && echo 0 || echo 1)
    report "G $opts (self+djxl)" $rc ""
done

# --- D/E: libjxl-produced fp16 patch files ----------------------------------
if [ -x "$MK16" ]; then
    # mk16 <in.ppm> <out.jxl> <patches 0|1|-1> [effort] [mode] ...
    # mode 0: v -> v * 2^-8 (the text_fp16.pfm twin)
    for spec in "lib_p1 1 7 0" "lib_p0 0 7 0" "lib_p1e9 1 9 0" "lib_p1e5 1 5 0"; do
        set -- $spec; f=$1; p=$2; e=$3; m=$4
        "$MK16" "$WORK/img/text_fp16.ppm" "$WORK/$f.jxl" $p $e $m 2>/dev/null
        [ -s "$WORK/$f.jxl" ] || { report "D $f: mk16 failed" 1 ""; continue; }
        "$DJXL" "$WORK/$f.jxl" "$WORK/$f.djxl.pfm" >/dev/null 2>&1
        "$JXLTOOL" decode "$WORK/$f.jxl" "$WORK/$f.ours.pfm" 2>/dev/null
        cmp_pfm "$WORK/$f.djxl.pfm" "$WORK/$f.ours.pfm" >/dev/null 2>&1
        report "D $f: ours == djxl ($(stat -c%s "$WORK/$f.jxl") B)" $? ""
        # E: the libjxl file itself must be lossless vs the dyadic source
        cmp_pfm "$WORK/img/text_fp16.pfm" "$WORK/$f.djxl.pfm" >/dev/null 2>&1
        report "E $f: libjxl file lossless" $? ""
    done
else
    echo "  (skip D/E: mk16 not built)"
fi

# --- F: hostile fp16 (unrepresentable difference) ---------------------------
img=hostile_fp16.pfm; B=hostile_fp16
"$JXLTOOL" encode "$WORK/img/$img" "$WORK/$B.p1.jxl" --float16 --patches=1 >/dev/null 2>"$WORK/f1.err"
rc1=$?
"$JXLTOOL" encode "$WORK/img/$img" "$WORK/$B.def.jxl" --float16 >/dev/null 2>"$WORK/f2.err"
rc2=$?
"$JXLTOOL" encode "$WORK/img/$img" "$WORK/$B.p0.jxl" --float16 --patches=0 >/dev/null 2>"$WORK/f3.err"
rc3=$?
if [ $rc1 -ne 0 ] && [ $rc2 -ne 0 ] && [ $rc3 -eq 0 ]; then
    report "F hostile: patches=1 fails, default fails, patches=0 ok" 0 ""
else
    report "F hostile" 1 "rcs: $rc1 $rc2 $rc3 (want !=0 !=0 0); err: $(grep -m1 error "$WORK/f1.err")"
fi
if [ -x "$MK16" ]; then
    "$MK16" "$WORK/img/hostile_fp16.ppm" "$WORK/$B.lib.jxl" 1 7 1 >/dev/null 2>&1
    rc=$?
    [ $rc -ne 0 ] && [ ! -s "$WORK/$B.lib.jxl" ]
    report "F hostile: libjxl patches=1 fails too" $? "mk16 rc=$rc"
    "$MK16" "$WORK/img/hostile_fp16.ppm" "$WORK/$B.lib0.jxl" 0 7 1 >/dev/null 2>&1
    report "F hostile: libjxl patches=0 succeeds" $? ""
fi

# --- H: fp32 excluded from the patch search (libjxl gate bits > 16) ---------
python3 - "$WORK/fp32.pfm" <<'EOF'
import sys, struct
sys.path.insert(0, 'validation')
from gen_fp16_patch_images import glyph_grid
w = h = 256
px = glyph_grid(w, h)
vals = []
for row in px:
    for v in row:
        vals.extend([v / 256.0, (v // 2) / 256.0, (v // 4) / 256.0])
rows = [vals[y*w*3:(y+1)*w*3] for y in range(h)]
rows.reverse()
flat = [x for r in rows for x in r]
with open(sys.argv[1], 'wb') as f:
    f.write(b'PF\n%d %d\n-1.0\n' % (w, h))
    f.write(struct.pack('<%df' % len(flat), *flat))
EOF
"$JXLTOOL" encode "$WORK/fp32.pfm" "$WORK/fp32.jxl" --patches=1 2>/dev/null
"$JXLTOOL" decode "$WORK/fp32.jxl" "$WORK/fp32.self.pfm" 2>/dev/null
cmp_pfm "$WORK/fp32.pfm" "$WORK/fp32.self.pfm" >/dev/null 2>&1
report "H fp32 --patches=1 roundtrip (search skipped)" $?

echo
echo "PASS=$PASS FAIL=$FAIL"
[ $FAIL -eq 0 ]
