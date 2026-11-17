#!/bin/bash
# Cross-validation of the patch dictionary (both directions) against
# libjxl v0.12.0 (cjxl/djxl).
#
#  A) ours --patches=1  -> ours decode   == input (exact raster)
#  B) ours --patches=1  -> djxl          == input (bitstream compatibility)
#  C) cjxl --patches=1  -> ours decode   == djxl decode (decode compat)
#  D) cjxl effort sweep (7..10 default, 5..6 forced) -> ours decode == djxl
#  E) animations with patches, both directions
#  F) ours default effort ladder: patches on at >= 7, off below
set -u
cd "$(dirname "$0")/.."
JXLTOOL=${JXLTOOL:-zig-out/bin/jxltool}
CJXL=${CJXL:-/tmp/libjxl-build/tools/cjxl}
DJXL=${DJXL:-/tmp/libjxl-build/tools/djxl}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/img"
python3 validation/gen_patch_images.py "$TMP/img" || exit 1

PASS=0
FAIL=0

report() { # name ok-detail|fail-detail
    if [ "${2%%}" = "0" ]; then
        PASS=$((PASS + 1)); echo "  ok   $1"
    else
        FAIL=$((FAIL + 1)); echo "  FAIL $1"; echo "       $3"
    fi
}

cmp_ppm() { # a b -> prints IDENTICAL/... ; returns 0/1
    python3 validation/compare_ppm.py "$1" "$2"
}

for img in text_like.ppm text_like2.ppm text_like_16b.ppm text_like_alpha.pam; do
    echo "== $img =="
    B="${img%%.*}"

    # A) ours -> ours
    "$JXLTOOL" encode "$TMP/img/$img" "$TMP/$B.ours.jxl" --patches=1 2>"$TMP/a.err" \
        && "$JXLTOOL" decode "$TMP/$B.ours.jxl" "$TMP/$B.ours.out.pam" 2>>"$TMP/a.err"
    rc=$?
    if [ $rc -ne 0 ]; then report "A ours->ours" 1 "encode/decode failed: $(cat "$TMP/a.err" | tail -2)"; else
        cmp_ppm "$TMP/img/$img" "$TMP/$B.ours.out.pam" >/dev/null 2>&1
        report "A ours->ours" $? "$(cmp_ppm "$TMP/img/$img" "$TMP/$B.ours.out.pam" 2>&1 | tail -1)"
    fi

    # B) ours -> djxl
    if [ -f "$TMP/$B.ours.jxl" ]; then
        "$DJXL" "$TMP/$B.ours.jxl" "$TMP/$B.djxl.pam" >/dev/null 2>"$TMP/b.err" \
            && cmp_ppm "$TMP/img/$img" "$TMP/$B.djxl.pam" >/dev/null 2>&1
        report "B ours->djxl" $? "djxl: $(tail -1 "$TMP/b.err"); $(cmp_ppm "$TMP/img/$img" "$TMP/$B.djxl.pam" 2>&1 | tail -1)"
    else
        report "B ours->djxl" 1 "no ours file"
    fi

    # C/D) cjxl -> djxl vs cjxl -> ours
    for e in 7 8 9 10 "5 --patches=1" "6 --patches=1"; do
        set -- $e; eff=$1; shift; extra="$*"
        tag="${B}_e${eff}_${extra// /}"
        if "$CJXL" "$TMP/img/$img" "$TMP/$tag.jxl" -d 0 -e $eff $extra --quiet >/dev/null 2>"$TMP/c.err"; then
            "$DJXL" "$TMP/$tag.jxl" "$TMP/$tag.djxl.pam" >/dev/null 2>&1
            "$JXLTOOL" decode "$TMP/$tag.jxl" "$TMP/$tag.ours.pam" >/dev/null 2>"$TMP/c.err"
            cmp_ppm "$TMP/$tag.djxl.pam" "$TMP/$tag.ours.pam" >/dev/null 2>&1
            report "C cjxl e$eff $extra" $? "$(cmp_ppm "$TMP/$tag.djxl.pam" "$TMP/$tag.ours.pam" 2>&1 | tail -1)"
        else
            report "C cjxl e$eff $extra" 1 "cjxl failed: $(tail -1 "$TMP/c.err")"
        fi
    done

    # F) ours effort ladder: sizes shrink or grow is not asserted; just
    # make sure every effort level roundtrips through djxl.
    for eff in 5 7; do
        po=0; [ $eff -ge 7 ] && po=-1   # default: on at >= 7
        "$JXLTOOL" encode "$TMP/img/$img" "$TMP/${B}_eff$eff.jxl" -e $eff --patches $po 2>/dev/null
        "$DJXL" "$TMP/${B}_eff$eff.jxl" "$TMP/${B}_eff$eff.pam" >/dev/null 2>&1
        cmp_ppm "$TMP/img/$img" "$TMP/${B}_eff$eff.pam" >/dev/null 2>&1
        report "F ours -e $eff (patches $po)" $? ""
    done
done

# E) animations with patches (two same-sized frames of text)
echo "== animation =="
"$JXLTOOL" encode-anim "$TMP/anim.ours.jxl" "$TMP/img/text_like.ppm" "$TMP/img/text_like_b.ppm" --patches=1 2>"$TMP/e.err"
rc=$?
if [ $rc -ne 0 ]; then report "E anim ours encode" 1 "$(tail -2 "$TMP/e.err")"; else
    "$JXLTOOL" decode-anim "$TMP/anim.ours.jxl" "$TMP/anim.ours" >/dev/null 2>&1
    ok=0
    python3 - "$TMP" <<'EOF' || ok=1
import sys, os
sys.path.insert(0, "validation")
from compare_ppm import read_pnm
tmp = sys.argv[1]
# frame order preserved; each frame must equal its source
pairs = [
    (f"{tmp}/anim.ours_0000.pam", f"{tmp}/img/text_like.ppm"),
    (f"{tmp}/anim.ours_0001.pam", f"{tmp}/img/text_like_b.ppm"),
]
for a, b in pairs:
    wa, ha, da, ma, ra = read_pnm(a)
    wb, hb, db, mb, rb = read_pnm(b)
    exp = wa * ha * da * (2 if ma > 255 else 1)
    if (wa, ha) != (wb, hb) or ra[:exp] != rb[:exp]:
        print(f"frame mismatch {a}")
        sys.exit(1)
sys.exit(0)
EOF
    report "E anim ours roundtrip" $ok ""
    # djxl decodes the same file to an APNG; every frame must match
    "$DJXL" "$TMP/anim.ours.jxl" "$TMP/anim.ours.djxl.apng" >/dev/null 2>&1
    ok=0
    python3 - "$TMP" <<'EOF' || ok=1
import sys
from PIL import Image
sys.path.insert(0, "validation")
from compare_ppm import read_pnm
tmp = sys.argv[1]
ref = Image.open(f"{tmp}/anim.ours.djxl.apng")
srcs = [f"{tmp}/img/text_like.ppm", f"{tmp}/img/text_like_b.ppm"]
if ref.n_frames != len(srcs):
    print(f"frame count {ref.n_frames} != {len(srcs)}"); sys.exit(1)
for i in range(ref.n_frames):
    ref.seek(i)
    px = ref.convert("RGB").tobytes()
    wb, hb, db, mb, rb = read_pnm(srcs[i])
    if px != rb[:wb * hb * 3]:
        print(f"frame {i} mismatch"); sys.exit(1)
sys.exit(0)
EOF
    report "E anim ours->djxl" $ok ""
fi

# cjxl animation with patches -> ours (APNG container input)
python3 validation/make_apng.py "$TMP/img/text_like.ppm" "$TMP/img/text_like_b.ppm" "$TMP/anim_src.png" 200
if "$CJXL" "$TMP/anim_src.png" "$TMP/anim.cjxl.jxl" -d 0 --patches=1 --quiet >/dev/null 2>"$TMP/e2.err"; then
    "$DJXL" "$TMP/anim.cjxl.jxl" "$TMP/anim.cjxl.djxl.apng" >/dev/null 2>&1
    rm -rf "$TMP/anims"; mkdir -p "$TMP/anims"
    "$JXLTOOL" decode-anim "$TMP/anim.cjxl.jxl" "$TMP/anims/a" >/dev/null 2>"$TMP/e2.err"
    ok=0
    python3 - "$TMP" <<'EOF' || ok=1
import sys, os
sys.path.insert(0, "validation")
from compare_ppm import read_pnm
tmp = sys.argv[1]
pairs = [
    (f"{tmp}/anims/a_0000.pam", f"{tmp}/img/text_like.ppm"),
    (f"{tmp}/anims/a_0001.pam", f"{tmp}/img/text_like_b.ppm"),
]
for a, b in pairs:
    if not os.path.exists(a):
        print(f"missing {a}"); sys.exit(1)
    wa, ha, da, ma, ra = read_pnm(a)
    wb, hb, db, mb, rb = read_pnm(b)
    exp = wa * ha * da * (2 if ma > 255 else 1)
    if (wa, ha) != (wb, hb) or ra[:exp] != rb[:exp]:
        print(f"frame mismatch {a}")
        sys.exit(1)
sys.exit(0)
EOF
    report "E anim cjxl->ours" $ok ""
else
    report "E anim cjxl->ours" 1 "cjxl anim failed: $(tail -1 "$TMP/e2.err")"
fi

echo
echo "PASS=$PASS FAIL=$FAIL"
[ $FAIL -eq 0 ] && echo "ALL OK" || exit 1
