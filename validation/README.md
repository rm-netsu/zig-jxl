# Cross-validation harness

Scripts used to validate this port against the official libjxl
(built from source with `build_libjxl.sh`, tested against v0.12.0).
They expect `cjxl`, `djxl` (and `jxlinfo`) on PATH and a working Zig
toolchain; `jxltool` is taken from `../zig-out/bin/`.

- `build_libjxl.sh`    - clone + build the reference cjxl/djxl
- `gen_test_images.py` - generate the PNM/PAM test corpus
- `xval_icc.sh`        - ICC round: encode --icc / decode --dump-icc /
                         JPEG transcode with ICC, both directions
- `xval_anim_decode.sh`- cjxl animations -> port decode-anim vs djxl APNG
- `xval_anim_encode.sh`- port encode-anim -> djxl APNG vs source frames
- `xval_jpeg_cfl.sh`   - round 5: encoder CfL + custom coefficient orders
                         (port transcode -> djxl byte-exact JPEG + pixels,
                         cjxl -> port untranscode regression, size table
                         vs cjxl; corpus: /tmp/jpegcorpus + testdata)
- `xval_float.sh`      - round 6: float samples (PFM). Port fp32/fp16
                         encode -> djxl PFM (values exact, options matrix
                         incl. odd sizes and --resampling 2/4 where the
                         check is port-decode == djxl-decode); cjxl PFM
                         (efforts 3-9 x default/-R 1; cjxl itself fails
                         below e3) -> port decode == djxl decode; fp16
                         incl. palettes on the bit patterns; int-stills +
                         JPEG-transcode regression tail
- `xval_progressive.sh`- round 7: progressive lossless + TOC permutation.
                         cjxl -d 0 -p (efforts 1-7) -> port decode ==
                         djxl decode; cjxl --group_order=1 and -p +
                         --group_order=1 combined -> port decode;
                         port --progressive (group sizes 0/1/3) -> djxl +
                         self decode; plain / -R 1 regression tail
                         (validated with libjxl 0.11.2 tools: the header
                         and permutation field layouts this round needs
                         are identical in v0.11.2 and v0.12.0)
- `xval_regression.sh` - full sweep: stills (both directions, options and
                         effort matrices), ICC, JPEG transcode, animations
- `compare_anim.py`    - per-frame pixel/duration comparator (PAM vs APNG)
- `gen_icc.c`          - lcms2 ICC profile generator for the ICC tests

Working directory for the scripts: any dir with the test corpus
(`gen_test_images.py` output + the JPEGs from `src/jpeg/testdata/`).
- `xval_effort.sh`     - round 8: encoder effort ladder + TOC permutation
                         encoding. Port `--effort 1..10` -> djxl/self
                         pixel-exact; cjxl `-e 8..10` -> port decode;
                         port `--group-order 1` -> djxl/self; permutation
                         arrays identical to cjxl `--group_order=1` over
                         many geometries (via `jxldbg --toc`); JPEG
                         transcode with `--group-order` byte-exact both
                         directions.
- `xval_patches.sh`        - round 8: reference patches. Port `--patches=1`
                            (4 text-like images) -> self + djxl; cjxl
                            `--patches=1` (efforts 5-10) -> port == djxl;
                            patch-detection parity (identical dictionaries);
                            effort ladder default; animations both
                            directions (djxl APNG, cjxl APNG input via
                            make_apng.py)
- `xval_patch_regression.sh`- post-patch regression of everything earlier:
                            options matrix ours -> djxl, cjxl matrix ->
                            ours == djxl, patches-off paths, resampling
                            (ours == djxl), animation tail
- `gen_patch_images.py`    - text-like corpus for the patch round
- `compare_ppm.py`         - PNM/PAM exact-raster comparer (headers may
                            differ; samples must not)
- `make_apng.py`           - deterministic 2-frame 8-bit RGB APNG builder
                            (per-frame independent zlib streams, the
                            layout libpng-based readers accept)
- `xval_patches_float.sh`   - round 10: binary16 (fp16) reference patches.
                            Port `--float16 --patches=1` (RGB/gray/
                            multi-group) -> self + djxl value-exact;
                            effort ladder + options matrix; libjxl-API
                            files (make_fp16_patches.c) -> port == djxl
                            and lossless vs source; hostile fp16
                            (unrepresentable differences) fails the
                            encode in both codecs; fp32 skips the search
- `gen_fp16_patch_images.py` - fp16-exact dyadic corpus (PFM + PPM twins
                            for the libjxl-API reference encoder)
- `make_fp16_patches.c`     - libjxl-API fp16 sample producer (cjxl's CLI
                            has no --float16; build against the local
                            libjxl: gcc -o mk16 make_fp16_patches.c
                            -I<include> -L<lib> -ljxl)
- `compare_pfm.py`          - exact float-sample comparer (PFM bottom-up
                            row order, either endianness, PF/Pf)

- `xval_lossy_patches.sh`- round 11: lossy VarDCT patches. Port `-d D
  --patches=1|0` files (256x256 + 777x403 multi-group, 8/16-bit,
  d=0.5/1/3) decode within 1.6e-5 of djxl on linear PFM
  (`--color_space=RGB_D65_SRG_Rel_Lin`), 8-bit PPMs agree within +-1
  (djxl dithers), patches shrink the corpus 76-91%, the DCT-8-only cjxl
  subset (`-d 1 -e 1 --epf 0` x gaborish x patches) decodes
  pixel-exact, and default cjxl lossy (varblocks+EPF) is rejected.
