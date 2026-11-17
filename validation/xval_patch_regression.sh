#!/bin/bash
# Post-patch regression: the still/animation paths this port already had
# (rounds 1-33 of CHANGES.md) must behave identically after the patch
# integration. Self-contained (no /tmp/xval leftovers): regenerates the
# corpus, then checks
#   R1) ours encode (options matrix, patches untouched) -> djxl == source
#   R2) cjxl (default, no patch flags) -> ours decode == djxl decode
#   R3) ours encode --patches=0 / default on non-text images -> djxl == src
#   R4) progressive / group_order / resampling tails through djxl
set -u
cd "$(dirname "$0")/.."
JXLTOOL=${JXLTOOL:-zig-out/bin/jxltool}
CJXL=${CJXL:-/tmp/libjxl-build/tools/cjxl}
DJXL=${DJXL:-/tmp/libjxl-build/tools/djxl}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
python3 validation/gen_test_images.py "$TMP" >/dev/null
python3 validation/gen_patch_images.py "$TMP" >/dev/null

PASS=0
FAIL=0
note() { echo "  $1"; }
ok() { PASS=$((PASS + 1)); }
bad() { FAIL=$((FAIL + 1)); echo "  FAIL $1"; }

check_pair() { # ours-or-djxl output vs source; $1 label $2 produced $3 source
    if python3 validation/compare_ppm.py "$2" "$3" >/dev/null 2>&1; then
        ok; note "  ok   $1"
    else
        bad "$1 ($(python3 validation/compare_ppm.py "$2" "$3" 2>&1 | tail -1))"
    fi
}

echo "== R1: ours -> djxl, options matrix =="
for img in rgb8.pnm gray8.pnm rgb16.pnm rgb_big.pnm text_like.ppm; do
    for opts in "" "--responsive 1" "--group-size-shift 0" "--colorspace 5" \
                "--palette-colors 0" "--progressive" "--group-order 1" \
                "-e 9 --responsive 1"; do
        tag="$(basename "$img")[$opts]"
        if ! "$JXLTOOL" encode "$TMP/$img" "$TMP/o.jxl" $opts 2>/dev/null; then
            bad "$tag encode"; continue
        fi
        if ! "$DJXL" "$TMP/o.jxl" "$TMP/o.pam" >/dev/null 2>&1; then
            bad "$tag djxl"; continue
        fi
        check_pair "$tag" "$TMP/o.pam" "$TMP/$img"
    done
    # resampling changes the pixel grid: ours-decode must equal djxl-decode
    if ! "$JXLTOOL" encode "$TMP/$img" "$TMP/o.jxl" --resampling 2 2>/dev/null; then
        bad "resampling encode"
    else
        "$DJXL" "$TMP/o.jxl" "$TMP/o.pam" >/dev/null 2>&1
        if ! "$JXLTOOL" decode "$TMP/o.jxl" "$TMP/o.ours.pam" >/dev/null 2>&1; then
            bad "resampling ours-decode"
        else
            check_pair "resampling 2" "$TMP/o.ours.pam" "$TMP/o.pam"
        fi
    fi
done

echo "== R2: cjxl (default) -> ours == djxl =="
for img in rgb8.pnm gray8.pnm rgb16.pnm rgb_big.pnm text_like.ppm text_like_16b.ppm; do
    for opts in "-e 7" "-e 9 -R 1" "-p" "--group_order=1" "-e 7 --resampling=2"; do
        tag="$(basename "$img")[$opts]"
        if ! "$CJXL" "$TMP/$img" "$TMP/c.jxl" -d 0 $opts --quiet >/dev/null 2>&1; then
            bad "$tag cjxl"; continue
        fi
        "$DJXL" "$TMP/c.jxl" "$TMP/c.djxl.pam" >/dev/null 2>&1
        if ! "$JXLTOOL" decode "$TMP/c.jxl" "$TMP/c.ours.pam" >/dev/null 2>&1; then
            bad "$tag ours-decode"; continue
        fi
        check_pair "$tag ours-vs-djxl" "$TMP/c.ours.pam" "$TMP/c.djxl.pam"
    done
done

echo "== R3: patches disabled paths on non-text images =="
for img in rgb8.pnm rgb_big.pnm; do
    for opts in "--patches 0" "" "--patches 1"; do
        tag="$(basename "$img")[$opts]"
        if ! "$JXLTOOL" encode "$TMP/$img" "$TMP/o.jxl" $opts 2>/dev/null; then
            bad "$tag encode"; continue
        fi
        "$DJXL" "$TMP/o.jxl" "$TMP/o.pam" >/dev/null 2>&1
        check_pair "$tag" "$TMP/o.pam" "$TMP/$img"
    done
done

echo "== R4: animation tail (no patches) =="
if "$JXLTOOL" encode-anim "$TMP/a.jxl" "$TMP/rgb8.pnm" "$TMP/rgb8.pnm" --patches 0 >/dev/null 2>&1; then
    "$DJXL" "$TMP/a.jxl" "$TMP/a.apng" >/dev/null 2>&1
    ok=0
    python3 - "$TMP" <<'EOF' || ok=1
import sys
from PIL import Image
tmp = sys.argv[1]
ref = Image.open(f"{tmp}/a.apng")
if ref.n_frames != 2:
    sys.exit(1)
for i in range(2):
    ref.seek(i)
    sys.path.insert(0, "validation")
    from compare_ppm import read_pnm
    _, _, _, _, src = read_pnm(f"{tmp}/rgb8.pnm")
    if ref.convert("RGB").tobytes() != src[:77 * 50 * 3]:
        sys.exit(1)
sys.exit(0)
EOF
    if [ $ok -eq 0 ]; then ok; note "  ok   anim roundtrip via djxl"; else bad "anim roundtrip"; fi
else
    bad "encode-anim"
fi

echo
echo "PASS=$PASS FAIL=$FAIL"
[ $FAIL -eq 0 ] && echo "ALL OK" || exit 1
