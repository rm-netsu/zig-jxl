# zig-jxl

A minimal **lossless JPEG XL (ISO/IEC 18181-1) codec in Zig 0.16**, built from
two bit-exact ports of libjxl v0.12.0 building blocks (the full ANS/prefix
entropy stack and the MA-tree/predictor stack) plus hand-written header,
container, framing and pipeline code.

Scope: the **modular lossless roundtrip path with encoder-side transforms**
(the full cjxl effort ladder `--effort 1..10`: channel/global/group palettes,
the local RCT search, the Squeeze transform with `-R 1`, and the slower-tier
cost-based heuristics — palette reverts, the weighted-predictor mode search
and wide property sets of kKitten/kTortoise/kGlacier), **progressive
lossless codestreams** (multi-pass modular frames, both directions — the
cjxl `-p` shape), **TOC permutations** (`--group_order=1`, both directions),
**float samples** (PFM fp32/fp16 HDR input and output), **embedded ICC
profiles**, **animations** (multi-frame modular codestreams: crops,
reference frames, all blend modes, progressive frames included), and
**reference patches** (`--patches`, spec §6.10.3: repeated fragments coded
once in a `kReferenceOnly` frame saved into slot 3 before the color
transform and blended back at their occurrences, encoder and decoder,
stills — integer samples and **binary16 float** (`--float16`, the
libjxl-API shape cjxl's CLI cannot write), animations for integer
samples — the cjxl `--patches=0|1` feature), **lossy VarDCT with
patches** (`-d D`, the XYB patch subset: patch search on the opsin
image, reference frame, gaborish, DCT-8, adaptive DC smoothing —
encoder and pixel decoder, djxl-validated to float noise), plus LZ77/RLE
in the entropy streams, and
the **lossless JPEG transcode path (VarDCT)**. The VarDCT implementation is the
JPEG-recompression subset: kVarDCT frames carrying JPEG DCT coefficients with
RAW quant tables, exactly the codestream libjxl's `ComputeJPEGTranscodingData`
produces — no general-purpose lossy VarDCT, no noise/splines. The architecture
mirrors libjxl's module layout so the remaining features have obvious extension
points.

## Layout

```
src/
  base/      bit-level I/O, field coders (vendored port of libjxl base)
  entropy/   ANS + prefix-code stack: histograms, clustering, context map,
             LZ77 (encode: RLE/LZ77/optimal, decode: window + special
             distances) — vendored port of enc_ans/dec_ans/dec_huffman/...
  modular/   MA trees, predictors, modular stream encode/decode,
             transforms: RCT/Palette/Squeeze forward AND inverse
             (enc_transforms.zig ports enc_rct/enc_palette/enc_squeeze.cc
             and the enc_modular transform-selection heuristics)
  header/    SizeHeader, ImageMetadata (incl. animation), ColorEncoding,
             FrameHeader (incl. YCbCrChromaSubsampling, blending info,
             animation frames, LoopFilter), TOC, container,
             ICC profile codec (decode + encode: predictICC/writeICC)
  vardct/    JPEG-transcode VarDCT frames: quantizer params, RAW quant tables,
             block context map, coefficient orders (decode of custom
             Lehmer-coded orders + the encoder-side ComputeCoeffOrder
             heuristic and serialization), chroma-from-luma (decode-side
             restore + the encoder-side FindCorrelation search and QCR
             subtraction, `force_cfl_jpeg_recompression`),
             AC coefficient coding, frame encoder/decoder (ports of
             quant_weights, ac_context, coeff_order, enc_entropy_coder/
             dec_group, enc_frame/dec_frame JPEG paths);
             lossy.zig (the general lossy VarDCT subset: XYB patch
             frames end-to-end), opsin.zig (XYB <-> linear sRGB +
             the sRGB EOTF), transform_lossy.zig (scaled DCT-8,
             gaborish pair, quant bias, Libshroom quant weights with
             libjxl's FastPowf)
  codec/     pixel buffers (int + float), modular frame encoder, frame
             decoder, animation encode/decode (frame loop, reference slots,
             blending.cc math), roundtrip tests
  jpeg/      JPEG stage 1: JPEGData, bitstream parser, writer, jbrd
             (bit-exact JPEGData::VisitFields port), brotli wrapper (real
             Brotli compression via zig-brotli at cjxl's default effort 9;
             stored meta-blocks kept as the quality-0 fallback), transcode
  pnm.zig    PNM/PAM image I/O (P5/P6 8/16-bit, P7 for alpha) and PFM
             float I/O (PF/Pf 32-bit IEEE-754, either endianness)
  main.zig   `jxltool` CLI front-end
```

## Build

```
zig build            # library + jxltool
zig build test       # 278 tests: headers, entropy, modular, roundtrips,
                     # ICC codec, animations, ANS stress, JPEG parse/write,
                     # JPEG transcode (incl. CfL + coeff-order variants,
                     # Lehmer/ order roundtrips), float samples (PFM IO,
                     # fp32/fp16 encode/decode, custom-float conversions),
                     # patches (integer + fp16)
zig build -Doptimize=ReleaseSafe
```

## CLI

```
jxltool encode <input.pnm> <output.jxl> [--group-size-shift 0..3]
                  [--responsive 0|1] [--progressive|-p] [--colorspace -1..41]
                  [--palette-colors N] [--channel-colors-global-pct F]
                  [--channel-colors-group-pct F] [--icc <file.icc>]
                  [--resampling 1|2|4|8] [--effort 1..10] [--group-order 0|1]
                  [--patches -1|0|1]   # reference patches (cjxl --patches)
                  [-d/--distance D]    # lossy VarDCT (RGB input, D > 0);
                                       # patches default on at effort >= 7
jxltool encode <input.pfm> <output.jxl> [--float16] [encode options]
jxltool decode <input.jxl> <output.pnm> [--dump-icc <file.icc>] [--linear]
                                        # --linear: PFM output (lossy frames
                                        # decode to linear-sRGB floats)
jxltool decode-anim <input.jxl> <out_prefix>      # JXL -> per-frame PAMs
jxltool encode-anim <out.jxl> <f1.pnm> [f2.pnm ...] [--duration-ms N]
                  [--loops N] [--group-size-shift N] [--responsive 0|1]
                  [--progressive|-p] [--effort 1..10] [--group-order 0|1]
                  [--patches -1|0|1]
jxltool jpeg-roundtrip <input.jpg> <output.jpg>     # parse -> write, verify
jxltool jpeg-transcode <input.jpg> <output.jxl>     # JPEG -> JXL (VarDCT)
                  [--brotli-effort 0..11] [--no-cfl] [--no-coeff-reorder]
                  [--group-order 0|1]
jxltool jpeg-untranscode <input.jxl> <output.jpg>   # JXL -> original JPEG
```

PNM images are read/written as P5 (gray), P6 (RGB), 8 or 16 bits per sample;
alpha is carried via P7 (PAM) with `TUPLTYPE GRAYSCALE_ALPHA` / `RGB_ALPHA`.
**PFM images (PF/Pf)** carry 32-bit float samples (HDR, values outside
[0,1] and negative included): they encode as float-sample codestreams
(fp32 by default, `--float16` as binary16 patterns — the encoder refuses
values not exactly representable in the target format) and decode back to
PFM with values bit-identical to `djxl`'s PFM output. `--resampling 2|4|8`
downsamples at encode time (box filter in float space for PFM) and the
decoder applies the NLEE upsampler. `--group-size-shift` selects the modular group size (128 << shift).
`--effort 1..10` mirrors cjxl `-e` (7 = the default kSquirrel heuristics):
efforts 8-10 are the slower libjxl tiers (kKitten/kTortoise/kGlacier) with
cost-based palette reverts (`maybe_do_transform`), wider tree-training
property sets, more RCT candidates (9/19), the weighted-predictor mode
search (`EstimateWPCost`, per-stream WP headers) and the matching entropy
parameters (kBest clustering, kLZ77/kOptimal); efforts 1-6 are the faster
tiers (fixed trees at 1-3, the global YCoCg at 1-4, tightened property
sets). `--group-order 1` writes the TOC with the center-first section
permutation (cjxl `--group_order=1`): the Lehmer-coded permutation over the
kPermutationContexts histograms, sections emitted in physical order — the
written permutation values are identical to cjxl's for the same geometry
(checked over many layouts, incl. 2-pass progressive frames).
`--responsive 1` enables the Squeeze transform (cjxl `-R 1`),
`--progressive` / `-p` writes a 2-pass progressive codestream (cjxl `-p`'s
lossless qprogressive shape: Squeeze residuals split across passes by
their downsampling bracket) and the decoder accepts any multi-pass
modular frame, including cjxl `-p` animations. `--colorspace`
selects a fixed RCT or the per-group RCT search (-1, the default), and the
palette options mirror cjxl's `--modular_palette_colors` /
`--modular_channel_colors_global_percent` /
`--modular_channel_colors_group_percent`.

## What works

- Full codestream headers (SizeHeader, ImageMetadata, ColorEncoding incl. ICC
  flag parsing, FrameHeader modular subset, TOC incl. permutations) and the
  ISO/IEC 18181-2
  container (raw codestream output; container-aware input).
- **Progressive lossless (multi-pass modular frames, both directions)**:
  decoding of every `cjxl -d 0 -p` codestream — the pass/downsample
  brackets (`GetDownsamplingBracket`), per-pass AC group sections
  (`AcGroupIndex(pass, group)` + `ModularStreamId::ModularAC(group,
  pass)`), DC-group coarse channels, single-group layouts with empty pass
  sections — and the encoder-side `--progressive`/`-p` (the qprogressive
  2-pass (2,0) header + bracket-partitioned squeeze residuals), for
  stills, PFM floats and animations. Cross-validated pixel-exactly
  against `djxl` in both directions.
- **TOC permutations (both directions)**: decoding of `cjxl
  --group_order=1` center-first files — the Lehmer-coded section
  permutation with its kPermutationContexts ANS histograms and the
  physical-vs-logical section mapping (`ReadToc`/`ReadGroupOffsets`
  semantics) for modular and VarDCT-transcode frames alike — and the
  encoder-side `--group-order 1` (`PermuteGroups`: the center-first
  Chebyshev-distance/angle group order with the exact `std::remainder`
  tie-to-even semantics, written via `WriteTocPermutation`), for stills,
  floats, animations, progressive 2-pass frames and the JPEG transcode;
  the emitted permutation arrays are identical to cjxl's on the same
  geometry (validated across aspect ratios, group sizes and multi-pass
  layouts).
- Modular frames: global tree + per-group trees, all 15 MA-tree predictors,
  trainable trees (collect → TreeSamples → computeBestTree), ANS histograms
  with kFast clustering, per-cluster hybrid-uint configs, prefix (Huffman)
  codes.
- **Modular transforms (decode)**: RCT (all 42 custom types plus the
  permutation-only forms), Palette (incl. negative-index delta palettes, the
  small/large cube lookups and predictor-coded deltas) and Squeeze (the
  default scheme as well as explicit squeeze lists; `InvHSqueeze`/
  `InvVSqueeze` with `SmoothTendency`). Global-stream transforms are recorded
  even when the global stream itself decodes no channels and are undone after
  all groups are decoded (mirroring libjxl's `image.transform` lifetime);
  group-stream transforms are undone right after the group.
- **LZ77 in entropy streams (decode)**: the full
  `ReadHybridUintClusteredInlined<uses_lz77>` port — 64k circular window,
  width-derived special distances, the length uint-config, wrapped-length
  corruption guards — behind the same `ANSSymbolReader` the tree/AC paths
  use, so modular streams, trees and VarDCT AC histograms all accept
  LZ77-enabled streams.
- **Encoder-side transforms** (ports of `enc_rct.cc`, `enc_palette.cc`,
  `enc_squeeze.cc` and the `enc_modular.cc` selection heuristics at the
  cjxl-default `kSquirrel` effort): global + per-group channel palettes and
  the all-channel palette with the implicit-cube extension and luma
  ordering, the lossless local RCT search (EstimateCost over the 7
  squirrel candidates), and the default Squeeze scheme with the libjxl
  shift-cap truncation — `-R 1` streams, including multi-group frames
  where high-shift squeeze channels travel in the modular **DC group**
  streams.
- Encode: PNM → JXL, with `kRLE` LZ77 in the entropy streams (the cjxl
  default at `SpeedTier::kSquirrel`); the output is decoded pixel-exactly by
  libjxl (`djxl`), with the per-stream LZ77 distance multipliers
  (`HistogramParams::image_widths`) matching the decoder's
  special-distance table.
- **ICC profiles** (encode + decode): `--icc` embeds a raw profile into the
  codestream headers through the full `enc_icc_codec.cc` port
  (predictICC + writeICC); `--dump-icc` extracts it back byte-exactly;
  djxl decodes the port's files with a byte-identical `iCCP` chunk. The
  JPEG transcode path carries ICC JPEGs through the headers (real
  `want_icc`) instead of forcing sRGB, restores APP2 chunks on
  untranscode, and handles cjxl's `brob` metadata boxes.
- **Animations** (encode + decode): multi-frame modular codestreams - the
  cjxl `-d 0` GIF/APNG shape. Decode walks the frame loop with reference
  slots 0-3, crop placement (`PackSigned` origins), and the complete
  `blending.cc` math (kReplace/kAdd/kBlend/kAlphaWeightedAdd/kMul,
  premultiplied and clamped variants, padding rows from the reference);
  frames snapshot the canvas (duration-0 frames stay invisible). Encode
  writes coalesced full-frame animations with per-frame durations,
  `save_as_reference` and loops. Cross-validated against djxl APNG output:
  pixel- and duration-exact both directions, including cjxl files with
  cropped/blended frames.
- Decode: JXL → PNM; validated pixel-exact against `cjxl` output for the
  full default-settings lossless matrix: efforts 1–9, default and `-R 1`
  (squeeze) settings, fixed-tree and local-tree streams, single- and
  multi-group modular frames, transform-bearing streams
  (palette/squeeze/RCT — global and per-group, including multi-group
  squeeze frames whose high-shift channels are carried in the modular DC
  group streams and whose group rects must be shift-scaled), 8/16-bit,
  gray/RGB, alpha.
- 8/16-bit, gray/RGB, optional alpha, multi-group frames up to any size.
- JPEG stage 1: full `ReadJpeg` (baseline + progressive, restart markers,
  APP/COM/ICC/Exif handling) and `WriteJpeg` bit-exact roundtrips; `jbrd`
  serialization bit-exact against libjxl, wrapped in a real Brotli stream via
  zig-brotli: the writer compresses at `brotli_effort` quality (cjxl default
  9) like libjxl's `EncodeJPEGData`, and the reader decodes any valid Brotli
  stream, so cjxl/libjxl-produced `jbrd` boxes are accepted (raw
  stored-meta-block streams remain supported as the quality-0 shape).
- **JPEG transcode (VarDCT, stage 2)**: JPEG → JXL → JPEG lossless recompression
  through the standard kVarDCT codestream — DCT coefficients in
  per-channel/subsampled AC group sections, DC in modular VarDCTDC streams,
  RAW quant tables riding a modular stream in AC global, JPEG-tuned block
  context map, zig-zag coefficient orders, `kSkipAdaptiveDCSmoothing`. The
  container carries `jbrd` so `djxl` reconstructs the original JPEG bytes
  **byte-for-byte** — cross-validated with libjxl v0.12.0 `djxl` on a 38-file
  corpus (444/422/420, gray, baseline/progressive, restart markers, Exif/ICC,
  two-DQT, 1x1 up to 2100px wide/multi-DC-group frames): 38/38 byte-exact
  reconstructions and pixel decoding by libjxl of every file. Gray, YCbCr and
  Adobe-RGB (kNone) color transforms are supported. In the reverse direction,
  cjxl-produced transcoded files (efforts 1–9) reconstruct byte-exactly via
  `jpeg-untranscode` (25/25 sweep; 19/20 of the default corpus — the one
  remaining file has an ICC profile that cjxl 0.11.2 itself rejects).
- **Encoder-side chroma-from-luma for JPEG recompression** (libjxl
  `force_cfl_jpeg_recompression`, cjxl default on): the 444 3-component path
  runs the `FindCorrelation` search (d_num_zeros interval voting,
  `FindAvgIndexOfSumMaximum`, kZeroBias zero-threshold) per 8x8-block tile,
  subtracts the fixed-point `cfl_factor` from the chroma AC coefficients
  (QCR = QChroma − cfl_factor(Y)) and carries the maps in the ACMetadata
  modular streams — exactly the codestream shape cjxl writes at its default
  effort; the decoder has restored CfL since the earlier rounds.
- **Encoder-side custom coefficient orders** (libjxl `ComputeUsedOrders`/
  `ComputeCoeffOrder` at kSquirrel): per-position zero statistics over the
  50% fixed-seed xorshift128+ block sample (including libjxl's shared
  `ac_offset` bookkeeping), count-quantized stable ordering, and the
  Lehmer-coded permutation serialization (Fenwick-tree `ComputeLehmerCode`,
  trailing-zero trimming, kPermutationContexts histograms with default
  HistogramParams) — `used_orders = 1` files like cjxl's, falling back to
  natural zig-zag for images below 40x40 pixels (both dimensions) or when
  the learned order equals the natural one.
- **Float samples** (PFM fp32/fp16, the `cjxl -d 0` PFM path, both
  directions): `readFloat`/`writeFloat` PFM I/O (either endianness, output
  byte-identical to djxl's), `encodeFloat` converts samples to the
  custom-float bit patterns (`float_to_int`; fp32 default, fp16 with
  `--float16`, refusing not-exactly-representable values), with libjxl's
  transform guards (fp32 patterns get no RCT/no palette — `max_bitdepth +
  1 > 32` forces colorspace 0; fp16 patterns keep the full default
  pipeline) and `--resampling` box-downsampling in float space; decode
  applies `int_to_float` and the NLEE upsampler for upsampled frames and
  returns raw f32 values (`DecodedImage`: `.int` or `.float`). Cross-
  validated value-exactly against `djxl` in both directions (fp32, fp16,
  multi-group, odd sizes, resampling 2/4; cjxl efforts 3–9 x default/`-R 1`)
  — and the port encodes PFM inputs where `cjxl` 0.12.0 itself fails at
  efforts 1–2 (its fixed low-effort trees do not support 32-bit float
  patterns).

## Known limitations

- The encoder's transform heuristics cover the full cjxl effort ladder
  (`--effort 1..10`, see above) including the slower-tier cost-based
  reverts; the remaining gap is the lossy-palette machinery
  (`lossy_palette`, dithering, frequent deltas) — modular streams are
  always lossless — and `nb_repeats` subsampling (the port's tree training
  always samples every pixel, which is slower but never worse).
- No streaming API; whole-image buffers only.
- Float samples: fp32 and fp16 only (no other custom float bit widths);
  no float extra channels (a float alpha would need a float surface next
  to the PFM-style output); animations are integer-only.
- Animation restrictions: modular frames with color_transform none,
  upsampling 1, no loop filter, at most one alpha extra
  channel (multi-pass progressive animation frames ARE supported, both
  directions); VarDCT/lossy animations, kDCFrame progressive files and
  preview images are rejected (cjxl lossy GIF output uses VarDCT XYB).
- Progressive mode covers the modular lossless shape (cjxl `-p`: 2-pass
  qprogressive); the encoder always uses exactly 2 passes (the decoder
  accepts any), and VarDCT-side progressive (kDCFrame, `--progressive_dc`)
  is not implemented.
- Reference patches (spec §6.10.3) are implemented end-to-end for both
  domains (encoder `--patches -1|0|1`, decoder, animations for the
  modular path; see CHANGES.md §34-36): repeated fragments are coded
  once in a `kReferenceOnly` frame saved into slot 3 before the color
  transform and blended back with kAdd. Integer samples of any bit
  depth <= 16 **and fp16 float samples** (byte-compatible with libjxl
  in both directions; hostile fp16 inputs fail the encode exactly like
  libjxl) on the modular side, and the **lossy VarDCT path** (`-d D`:
  the search runs on the raw XYB opsin image with the is_xyb=true
  weights, the reference frame is a modular XYB frame quantized onto
  the distance-scaled enc_factors grid, and the subtraction happens in
  float before gaborish — the libjxl `LossyFrameHeuristics` flow).
  Restrictions: fp32 excluded from the search (32 > 16), resampling 1,
  no recursive patch dictionaries.
- Lossy VarDCT (`-d D`) is the DCT-8 XYB subset: RGB input only (no
  gray/alpha), EPF off, no noise/splines, single pass, constant quant
  field (libjxl's `q = 0.79/d` branch — files are valid and
  djxl-decodable, just larger than cjxl's Butteraugli-driven output).
  The decoder additionally accepts the matching cjxl subset
  (`cjxl -d D -e 1 --epf 0`, with or without gaborish/patches —
  validated pixel-exact to float noise against djxl); default cjxl
  lossy files (varblocks > 8x8, EPF, adaptive quantization) and the
  remaining VarDCT shapes (JPEG transcode keeps its own path) are
  rejected.
- VarDCT support beyond that remains the JPEG-transcode subset: no
  varblocks > 8x8, no chroma-from-luma for lossy (non-JPEG) encoding,
  and custom coefficient orders / CfL are written only in the shapes
  the JPEG path produces (DCT-8 orders bucket 0, 444 CfL maps).
  Files that use other shapes are rejected.
- 4-component (CMYK/YCCK) JPEGs and sampling patterns outside the
  444/422/420/440 grid (e.g. 4x1) are rejected — the same files libjxl
  v0.12.0 itself rejects (SetColorTransformFromJpegData /
  YCbCrChromaSubsampling::Set).

## Testing

`zig build test` runs 292 tests: unit tests per module plus image
roundtrips (solid/gradient/noise × 8/16-bit × gray/RGB/alpha ×
single/multi-group), float roundtrips (fp32/fp16, HDR-valued, gray/RGB,
multi-group, responsive), ANS stress roundtrips (random token streams
with 15–200 contexts under kFast clustering), JPEG parse→write bit-exact
roundtrips over a 20-file corpus, and JPEG→JXL→JPEG transcode
roundtrips (byte-exact) over the same corpus plus
multi-group/multi-DC-group/odd-size files, progressive roundtrips
(single/multi-group, noise, odd sizes, 16-bit, alpha, multi-DC-group;
progressive animations; bracket and TOC-permutation unit tests), patch
roundtrips (text-like RGB/gray/alpha/16-bit, responsive+progressive
combinations, codestream-shape checks: a `kReferenceOnly` slot-3
pre-transform frame + a last `kPatches` regular frame, no-patch
single-frame negative case, patch effectiveness), **fp16 patch tests**
(RGB/gray roundtrips, default-effort ladder, progressive+responsive,
codestream shape, effectiveness, the hostile unrepresentable-difference
failure, the fp32 search skip) and **lossy VarDCT tests** (gradient
quality, patch roundtrip exactness — glyph pixels survive within a few
LSBs, codestream shape, patch effectiveness, gray/alpha/distance
rejections, distance monotonicity, linear float output).

External validation (libjxl `cjxl`/`djxl`, see `validation/`): the
effort ladder and encoder TOC permutations are covered by
`xval_effort.sh` (efforts 1-10 in both directions, permutation parity with
cjxl over many geometries, the transcode corpus with `--group-order`);
encoder outputs decode pixel-exactly in `djxl` — stills (options matrix),
float samples value-exactly (fp32/fp16, multi-group, odd sizes, resampling;
`xval_float.sh`), progressive codestreams (stills, floats, animations;
`xval_progressive.sh`), ICC, animations and the JPEG transcode (byte-exact JPEG
reconstruction both directions), and the lossy VarDCT patch round
(`xval_lossy_patches.sh`: port `-d` files decode to within 1.6e-5 of
djxl on linear PFM, patches shrink text-like images 76-91%, the
DCT-8-only cjxl subset decodes, default cjxl lossy rejected);
decoder inputs cover the full cjxl `-d 0`
matrix — efforts 1–9 with transforms and LZ77 enabled for integer samples,
progressive (`-p`) files at efforts 1–9 and `--group_order=1` center-first
TOC permutations,
efforts 3–9 (default and `-R 1`) for PFM input (cjxl 0.12.0 itself fails to
encode PFM at efforts 1–2: its fixed low-effort trees do not support 32-bit
float patterns).
