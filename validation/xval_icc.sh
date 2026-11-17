#!/bin/bash
# Cross-validation of the ICC round: port encode --icc / decode --dump-icc
# against libjxl v0.12.0 (djxl PNG output preserves the ICC in iCCP).
set -u
cd /tmp/xval
export PATH=/home/z/my-project/work/ref/libjxl-0.12.0/build/tools:$PATH
PORT=./jxltool
pass=0; fail=0
ok()   { pass=$((pass+1)); }
bad()  { fail=$((fail+1)); echo "FAIL: $1"; }

# ---- 1. port encode --icc -> port decode --dump-icc (self roundtrip) ----
for prof in srgb gray adobergb linear; do
  img=rgb8.pnm; [ $prof = gray ] && img=gray8.pnm
  $PORT encode $img t_$prof.jxl --icc $prof.icc 2>/dev/null || { bad "encode $prof"; continue; }
  $PORT decode t_$prof.jxl t_$prof.pnm --dump-icc t_$prof.out.icc 2>/dev/null || { bad "decode $prof"; continue; }
  cmp -s $prof.icc t_$prof.out.icc && ok || bad "icc bytes differ: $prof"
  cmp -s $img t_$prof.pnm && ok || bad "pixels differ: $prof"
done

# ---- 2. port encode --icc -> djxl decodes (PNG), iCCP matches ----
for prof in srgb gray adobergb linear; do
  djxl t_$prof.jxl t_$prof.png 2>/dev/null || { bad "djxl decode $prof"; continue; }
  python3 - <<EOF || bad "djxl icc mismatch: $prof"
import struct, zlib, sys
def iccp(path):
    d = open(path,'rb').read()
    pos = 8
    while pos < len(d):
        ln, = struct.unpack('>I', d[pos:pos+4]); typ = d[pos+4:pos+8]
        if typ == b'iCCP':
            body = d[pos+8:pos+8+ln]
            z = body.index(b'\0'); z += 1  # name\0
            comp = body[z]; z += 1
            assert comp == 0, 'unexpected deflate'
            return zlib.decompress(body[z:])
        pos += 8 + ln + 4
    return None
want = open('$prof.icc','rb').read()
got = iccp('t_$prof.png')
sys.exit(0 if got == want else 1)
EOF
done

# ---- 3. djxl pixel fidelity of ICC files (decode to PPM, compare to source) ----
for prof in srgb adobergb linear; do
  djxl t_$prof.jxl t_${prof}_djxl.pnm 2>/dev/null || { bad "djxl pnm $prof"; continue; }
  cmp -s rgb8.pnm t_${prof}_djxl.pnm && ok || bad "djxl pixels differ: $prof"
done

# ---- 4/5. valid ICC jpegs (lcms2/PIL) through both directions ----
for name in srgb adobergb; do
  [ -f goodicc_$name.jpg ] || continue
  $PORT jpeg-transcode goodicc_$name.jpg port_$name.jxl 2>/dev/null || { bad "transcode $name"; continue; }
  rm -f port_${name}_rec.jpg
  djxl port_$name.jxl port_${name}_rec.jpg >/dev/null 2>&1
  if [ -f port_${name}_rec.jpg ]; then
    cmp -s goodicc_$name.jpg port_${name}_rec.jpg && ok || bad "port->djxl jpeg bytes $name"
  else
    bad "djxl no jpeg $name"
  fi
  cjxl goodicc_$name.jpg cjxl_$name.jpg.jxl -d 0 >/dev/null 2>&1 || { bad "cjxl transcode $name"; continue; }
  $PORT jpeg-untranscode cjxl_$name.jpg.jxl cjxl_${name}_rec.jpg 2>/dev/null || { bad "port untranscode $name"; continue; }
  cmp -s goodicc_$name.jpg cjxl_${name}_rec.jpg && ok || bad "cjxl->port jpeg bytes $name"
done

# ---- 6. cjxl with embedded ICC (via PNG iCCP) -> port decode + dump icc ----
# Reuse the djxl-written PNG (iCCP preserved); cjxl embeds it into the JXL.
for prof in srgb adobergb linear; do
  cjxl t_$prof.png cjxl_$prof.jxl -d 0 2>/dev/null || { bad "cjxl encode png $prof"; continue; }
  $PORT decode cjxl_$prof.jxl cjxl_$prof.pnm --dump-icc cjxl_$prof.out.icc 2>/dev/null || { bad "port decode cjxl $prof"; continue; }
  cmp -s $prof.icc cjxl_$prof.out.icc && ok || bad "cjxl icc bytes differ: $prof"
done

echo "=================================="
echo "ICC cross-validation: pass=$pass fail=$fail"
