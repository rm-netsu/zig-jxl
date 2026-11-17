# zig-jxl: gap-closure report (vs. official libjxl v0.12.0)

This file documents the functionality that exists in the official libjxl and
was missing from this Zig port, and how it was implemented.

## 1. Real Brotli for the `jbrd` payload (was: stored meta-blocks only)

**libjxl**: `EncodeJPEGData` compresses the APP/COM/inter-marker/tail blob with
`BrotliEncoderCompressStream` at `cparams.brotli_effort` (cjxl default 9);
the decoder runs the full Brotli decoder on it.

**Port before**: `jbrd` was written as *stored* (uncompressed) Brotli
meta-blocks and the reader only accepted that exact shape — every
cjxl/libjxl-produced `jbrd` box (real Brotli) was rejected.

**Implemented** (`src/jpeg/brotli.zig`, `src/jpeg/jbrd.zig`,
`src/jpeg/transcode.zig`):
- `brotli.zig` is now a thin wrapper over the zig-brotli dependency:
  `compress(quality 0..11, lgwin)` / `decompress` with libjxl-matching
  semantics (trailing bytes rejected like libjxl's "Unused data after brotli
  stream").
- `jbrd.encode` compresses the blob at `brotli_effort` (default 9 = cjxl
  default); quality 0 keeps the stored-meta-block shape for tests.
- `jbrd.decode` accepts **any** valid Brotli stream, so both cjxl-produced
  and self-produced boxes decode.

**Validation**: 20/20 byte-exact JPEG reconstructions of this codec's output
by libjxl `djxl`; cjxl-transcoded files (efforts 1–9, 25/25 sweep) reconstruct
byte-exactly via `jpeg-untranscode` (the one default-corpus failure is an
invalid ICC profile that cjxl 0.11.2 itself rejects before writing anything).

## 2. Modular transforms: RCT, Palette, Squeeze (was: rejected)

**libjxl**: `modular/transform/{rct,palette,squeeze}.cc` — channels are
restructured by `MetaApply`, entropy-coded in the transformed domain, and
undone by `InvRCT`/`InvPalette`/`InvSqueeze` at the end of the stream (or of
the frame for global transforms). cjxl's *default* lossless settings emit
palette and/or squeeze/RCT transforms, so without them the port could not
decode default cjxl output at all.

**Port before**: transforms were parsed syntactically and rejected with
`error.UnsupportedFrameFeatures`.

**Implemented** (`src/modular/transforms.zig`, `src/modular/stream.zig`,
`src/codec/decoder.zig`):
- `inverseRCT`: all 42 `custom` types + the 6 permutation-only forms
  (whole-channel swap), bit-exact against `InvRCT` including the wraparound
  arithmetic (`PixelAdd`).
- `inversePalette`: `GetPaletteValue` port with the negative-index delta
  palette (72 entries), the small/large cube lookups, bit-depth scaling, and
  the predictor-coded delta path (Weighted-predictor state when the palette's
  predictor uses it); `nb_meta_channels` bookkeeping with the signed update
  from libjxl.
- `inverseSqueeze` (+ `invHSqueeze`/`invVSqueeze`): `SmoothTendency`, odd
  width/height tail handling, `hshift/vshift` repair, residual channel
  removal; the default squeeze scheme is materialized into the transform at
  meta-apply time (as libjxl mutates the transform) so the inverse can replay
  it.
- `decodeStream` integration: `MetaApply` before channel decode, inverse
  after; **global-stream transforms are exported even when the global stream
  decodes zero channels** (the `num_chans == 0` early return previously
  dropped them — this was also the root cause of the "residual decoder gap"
  with `cjxl -e 3/4` multi-group files, whose global stream carries an RCT
  while all pixel data lives in the group streams); the frame decoder undoes
  them after all groups, exactly like libjxl's `image.transform` +
  `FinalizeDecoding`.
- Memory: channels dropped by `metaPalette`/`inverseSqueeze` are now freed
  (two DebugAllocator leaks fixed).

**Validation**: 54/54 pixel-exact decodes of cjxl `-d 0` files across efforts
1–9 (palette shapes, squeeze shapes, global and per-group RCT), 10/10
default-effort files incl. 16-bit/gray/alpha, all against `djxl` output.

## 3. LZ77 in entropy streams (was: disallowed)

**libjxl**: the modular/VarDCT entropy coding carries an `LZ77Params` bundle
per histogram set; the `ANSSymbolReader` transparently replays length/
distance commands over a 64k window with width-derived special distances.
cjxl default (kSquirrel, modular) writes `kRLE` LZ77.

**Port before**: every `decodeHistograms` call passed `disallow_lz77=true`,
rejecting LZ77-enabled streams, and the encoder forced `kNone`.

**Implemented**:
- The reader side was already ported inside `dec_ans.zig`
  (`readHybridUintClusteredInlined<uses_lz77>` — window, special distances,
  wrapped-length guards); the policy flags at every call site were flipped to
  `false` to match libjxl (`stream.zig`, `codec/decoder.zig`,
  `vardct/frame_decode.zig`; the context-map internal call keeps the libjxl
  rule `context_map.size() <= 2`).
- The encoder now writes `kRLE` (`codec/encoder.zig`, `vardct/
  frame_encode.zig`) — the cjxl default — exercising both sides in every
  roundtrip.
- Fixed the half-finished `checkFinalState` guard in `decodeStream`
  (`error.ANSFinalState`, as libjxl's "ANS decode final state failed").

**Validation**: included in the sweeps above (every cjxl default file is
LZ77-enabled); port-encoded output with kRLE decodes pixel-exactly in `djxl`
(9/9).

## 4. Test-suite / build integration

- zig-brotli is vendored at `vendor/zig-brotli` and wired as a Zig
  dependency (`build.zig.zon`, `build.zig`; `lib_mod`/`exe_mod` import the
  `brotli` module). No C brotli anywhere.
- `zig build test`: **151 tests pass** (was 147; the new brotli-wrapper tests
  are included).
- External cross-validation summary (libjxl 0.11.2 `cjxl`/`djxl`):

| Direction | Files | Result |
| --- | --- | --- |
| cjxl `-d 0` → port decode (efforts 1–9) | 54 | 54 pixel-exact vs djxl |
| cjxl `-d 0` → port decode (default, incl. 16-bit/gray/alpha) | 10 | 10 pixel-exact vs djxl |
| port encode → djxl decode | 9 | 9 pixel-exact |
| port jpeg-transcode → djxl JPEG reconstruction | 20-file corpus | 20 byte-exact |
| cjxl jpeg-transcode (efforts 1–9) → port jpeg-untranscode | 25 | 25 byte-exact |
| cjxl jpeg-transcode (default) → port jpeg-untranscode | 20 | 19 byte-exact (1 fails inside cjxl itself: invalid ICC) |

## What is still out of scope (unchanged; patches were closed in §34)

- Slower-effort transform heuristics (cost-based reverts below
  `kSquirrel`), lossy palettes and general lossy VarDCT / noise /
  splines — see README "Known limitations". (Lossless reference patches
  are implemented since §34.)

---

# Round 2: encoder-side transforms (vs. libjxl v0.12.0)

This round closes the biggest remaining gap: the encoder wrote
**transform-free** modular streams while the decoder accepted any transform
shape. It now produces the codestream shapes cjxl itself writes, plus a set
of latent decoder bugs this work uncovered (all found by cross-validation
against libjxl 0.12.0 `cjxl`/`djxl` built from source).

## 5. Encoder-side transforms: RCT, Palette, Squeeze (was: not written)

**libjxl**: `modular/transform/{enc_rct,enc_palette,enc_squeeze,enc_transform}.cc`
apply the forward transforms; `enc_modular.cc` decides which to use
(`try_palettes`, the local RCT search, the responsive squeeze pipeline with
the shift-cap truncation).

**Port before**: streams were written transform-free (README limitation #1).

**Implemented** (`src/modular/enc_transforms.zig`, `src/codec/encoder.zig`):
- `fwdRct`: all 42 custom types + permutations, bit-exact against
  `FwdRct` (permutation swap for type%7==0, scratch-channel forwarding for
  the aliased in-place cases).
- `fwdPalette`: the lossless paths of `enc_palette.cc` — the `nb==1`
  channel palette (lookup-table and sorted-set variants), the multi-channel
  palette with the **implicit cube** (189 small/large-cube colors promoted
  when frequent, indices `nb_colors+k`), luma ordering with the
  frequent-colors-first key, and the exact channel-list surgery of
  `MetaPalette` so encoder and decoder layouts stay identical.
- `fwdHSqueeze`/`fwdVSqueeze`/`fwdSqueeze` + `truncateDefaultSqueeze`
  (the `shift_cap = 7 + group_size_shift` script truncation).
- `estimateCost` (scalar port of `enc_modular_simd.cc`): gradient residuals
  in 17 local-activity contexts, driving the RCT search and the palette
  color-count heuristic.
- The selection drivers at cjxl's default effort (kSquirrel): global
  `try_palettes` at the pre-transform percentage, the all-minus-one-channel
  fallback, per-channel palettes, the 7-candidate local RCT search, the
  responsive YCoCg-then-squeeze pipeline, and the single-group global
  stream getting the local pass (`PrepareStreamParams` for
  `ModularStreamId::Global()`).
- Stream plumbing: per-stream `GroupHeader`s carry their transform lists;
  AC group images are built with libjxl's unclamped group rect, the [0,2]
  shift bracket and shift-scaled channel rects; squeeze frames additionally
  populate the **modular DC group streams** (shift bracket [3,1000], DC
  group rects), which the port previously rejected outright.

**Validation**: 176/176 encode→djxl and encode→self pixel-exact roundtrips
(22 images x 8 option combinations: default, `--responsive 1`,
`--palette-colors 0`, group shifts 0/3, fixed colorspaces, combinations);
sizes match or beat cjxl 0.12.0 on the corpus (e.g. 1000x1000 gradient
3.6 KB vs 36 KB, flat-region image 117 B vs 119 B, noise 786 KB vs 832 KB).

## 6. Decoder: multi-group squeeze + DC group streams (was: broken)

- **Squeeze default scheme export**: `decodeStream` copied the global
  stream's transforms to the frame level *before* `MetaApply` ran, so a
  default-scheme Squeeze (empty parameter list, materialized only inside
  `MetaSqueeze`) reached the frame-level inverse with no parameters and the
  undo was a no-op → `InvalidChannelRange`. The export now happens after
  `MetaApply` (single-group squeeze files decode pixel-exact).
- **Group channel routing**: AC group images now use libjxl's unclamped
  group rect (the clamped rect loses coverage of shifted channels at the
  right/bottom frame edge — e.g. a 513px-wide image's last column), the
  [0,2] single-pass min-shift bracket, and per-channel shift-scaled rects.
- **Modular DC group streams** (new): channels with min-shift >= 3 over the
  2048px DC group rects — required for squeeze frames larger than 2048px;
  the port previously rejected non-empty DC group sections.
- **`FrameDim.modularDcGroupRect`**: the pixel-based DC group rect (the
  existing `dcGroupRect` is the block-based VarDCT one).

**Validation**: 130/130 base + 90/90 alpha-corpus cjxl→port decodes
pixel-exact vs `djxl` (efforts 1-9 x default/`-R 1`, incl. 3000x1200
DC-group squeeze frames and 513px odd-size edge cases).

## 7. Latent bugs found by the new paths (both directions)

- **`getPaletteValue` large-cube modulo** (`transforms.zig`): the large-cube
  branch computed `index % (kLargeCube - 1)` (4) instead of libjxl's
  `index % kLargeCube` (5) — every implicit-cube palette index whose cube
  coordinate depended on the difference decoded to a wrong color. cjxl
  streams never hit it (its encoder promotes frequent implicit colors to
  the explicit palette), the port's new palette encoder did.
- **LZ77 RLE distance symbols** (`enc_lz77.zig` callers): the RLE encoder
  wrote distance symbol 0 whenever `HistogramParams::image_widths` was
  empty, but the decoder always activates the special-distance table
  (multiplier = max stream channel width), where symbol 0 means "one row
  up" and **symbol 1** is distance 1. The modular and VarDCT encoders now
  fill `image_widths` with the per-stream max channel widths exactly as
  libjxl does (the tree token stream keeps multiplier 0, matching
  `EncodeGlobalInfo`); without this, any RLE run in a palette channel
  corrupted the stream on decode.
- **`inversePalette` dangling palette pointer** (`transforms.zig`): the
  palette channel pointer was captured before the `nb-1` channel inserts;
  an ArrayList growth invalidated it and the inverse read a garbage
  palette width (a cjxl effort-1 gray+alpha file crashed the decoder).
  The pointer is re-fetched after the inserts.

## 8. Tests

- `zig build test`: **172 tests pass** (was 151): +7 enc_transforms unit
  tests (all 42 RCT forward/inverse roundtrips, palette roundtrips incl.
  the implicit cube and the sorted-set channel palette, default-scheme
  squeeze roundtrips with layout checks, the squeeze truncation, EstimateCost
  sanity, the full try_palettes+RCT-search pipeline), +6 group-palette
  regression tests driving the real stream encode/decode path (the shapes
  that exposed the cube-mod and distance-symbol bugs), +8 codec roundtrips
  (few-color palettes, fixed colorspaces, responsive single/multi group,
  odd sizes, no-palette).
- External cross-validation vs libjxl **v0.12.0** (this round's reference
  build):

| Direction | Files | Result |
| --- | --- | --- |
| cjxl `-d 0` (efforts 1-9, default + `-R 1`) → port decode | 130 | 130 pixel-exact vs djxl |
| cjxl alpha/gray/16-bit corpus (same matrix) → port decode | 90 | 90 pixel-exact vs djxl |
| port encode (22 images x 8 option combos) → djxl + port decode | 176 | 176 pixel-exact |
| port jpeg-transcode → djxl byte-exact JPEG | 20 | 20 byte-exact |
| cjxl jpeg-transcode (e3/e7/e9) → port jpeg-untranscode | 57 | 57 byte-exact |

---

# Round 3: ICC profiles in the codestream headers (vs. libjxl v0.12.0)

The port could parse `want_icc` files (the full `icc_codec.cc` decoder was
already in place) but could not *write* them: the encoder always serialized
an enum ColorEncoding, and the JPEG transcode path forced sRGB headers for
ICC-carrying JPEGs, keeping the profile only inside the `jbrd` blob.

## 9. ICC encoder: `predictICC` + `writeICC` (was: decode-only)

**libjxl**: `enc_icc_codec.cc` transforms a raw ICC profile into a
commands+data stream (tag keyword/offset prediction, per-tag-type linear
prediction of multi-byte values, byte interleaving) and `WriteICC`
entropy-codes it (prefix codes, LZ77) into the header position after
CustomTransformData.

**Implemented** (`src/header/enc_icc.zig`, port of `enc_icc_codec.cc`):
- `predictICC`: all 23 ICC commands (`kColorEncoding`, `kTrc`, `kChad`...),
  the tag-table prediction (`PredictTag`/`PredictSize` byte games), the
  per-tag-type predictors (XYZ, `curv`, `parag`, `para`-family, text,
  `mluc`, `mft1/2`, `sf32`), unshuffle of both commands and data, and the
  41-context stream layout; `writeICC` builds histograms (kFast clustering,
  kRLE LZ77) and writes the U64 size prefix + entropy-coded stream.
- The decoder side (`icc.zig`) gained the `shuffle`/`unpredict` paths that
  the encoder exercises (roundtrip-tested over random inputs and over
  plausible synthetic profiles, incl. `mluc`/`desc`/`text` tag shapes).
- `--icc <file.icc>` on `jxltool encode` embeds a real profile
  (`ColorEncoding.wantIcc`); `--dump-icc` extracts it back byte-exactly.

## 10. JPEG transcode with ICC (was: forced sRGB headers)

`jpeg-transcode` now detects a canonically chunked APP2 ICC profile
(`DetectIccProfile`), extracts it, and when it is a complete profile rides
it through the codestream headers (`SetFieldsFromICC` semantics: `want_icc`
+ the compressed ICC stream) instead of an sRGB enum; the APP2 payloads are
then no longer stored in `jbrd`. `jpeg-untranscode` rebuilds the APP2
chunk sequence from the codestream profile (`SetJPEGDataFromICC`), and
cjxl-produced containers are handled in both directions: Exif/XMP payloads
that cjxl moves into `brob`/`Exif`/`xml ` container boxes are decompressed
and restored into their typed APP1 markers (Brotli via zig-brotli).

**Validation** (libjxl v0.12.0): 26/26 - encode `--icc` x 4 profiles
(sRGB/gray/AdobeRGB/linear, lcms2-generated) decode in djxl with
byte-identical ICC in the PNG `iCCP` chunk and identical pixels; the
20-file JPEG corpus still reconstructs byte-exactly through djxl (incl.
the ICC JPEG, which cjxl itself rejects); fresh valid ICC JPEGs
(PIL/lcms2) round-trip byte-exactly in both directions (port->djxl and
cjxl->port).

---

# Round 4: animations + FrameHeader fidelity (vs. libjxl v0.12.0)

Multi-frame codestreams were rejected outright (`!frame.is_last`). This
round adds full still-image-animation support for the modular lossless
path - the codestream shape cjxl writes for `-d 0` GIF/APNG input - in
both directions, and fixes a set of latent FrameHeader bugs the work
uncovered (all found by bit-level cross-validation against libjxl).

## 11. Latent FrameHeader bugs (affect every cjxl-produced file)

- **LoopFilter extensions** (`frame.zig`): libjxl's `LoopFilter` bundle
  ends with its own `BeginExtensions/EndExtensions` U64 field; the port
  neither read nor wrote it. Every frame header was 2 bits short. The TOC
  byte-alignment silently absorbed the error for ~3/4 of header alignments
  (which is why 130+ still files decoded pixel-exactly), but whenever the
  header ended at bit%8 in {6,7} the TOC landed on the wrong byte and the
  frame misparsed - always fatal for multi-frame files at some frame
  boundary, e.g. the 5th frame of a cjxl alpha animation.
- **BlendingInfo nonserialized wipe** (`frame.zig`): `BlendingInfo.read`
  began with `self.* = .{}`, erasing the `nonserialized_is_partial_frame`/
  `nonserialized_num_extra_channels` values the caller sets - so
  partial-frame replace-mode sources and blend-mode alpha channels were
  never read (impossible to parse cropped/blend animation frames).
- **Frame origin packing** (`frame.zig`): origins were serialized with a
  raw `@bitCast` instead of libjxl `PackSigned/UnpackSigned`
  (non-negative X -> 2X), so every cropped frame decoded at doubled
  offsets.
- **Animation duration** (`frame.zig`): read/written as U64 instead of
  `U32(Val(0), Val(1), Bits(8), Bits(32))`; the 32-bit `timecode` field
  (when `have_timecodes`) was missing entirely.
- **save_before_color_transform**: gated on `save_as_reference != 0`
  instead of `CanBeReferenced()` (a duration-0 frame with slot 0 is
  referenceable too), and never read for `kReferenceOnly` frames (libjxl
  reads it unconditionally there).

## 12. Animation decode (new: `src/codec/animation.zig`)

Port of the decode.cc/dec_frame.cc frame loop and blending.cc for the
modular lossless path:
- Frame loop: FrameHeader + TOC + sections per frame until `is_last`;
  reference frame slots 0-3 with libjxl semantics (`CanBeReferenced`,
  pre-transform saves keep the frame-sized image at the frame origin,
  post-transform saves are coalesced full-canvas images at (0,0) - the
  render pipeline's `SwitchToImageDimensions` behavior of the
  BlendingStage).
- Compositing: a float [0,1] canvas; frames that need blending (crop or
  any non-replace mode) are blended over their background references
  inside the frame rect and copy the reference outside it
  (ProcessPaddingRow); full-frame replace frames copy through without any
  float math (exact lossless roundtrips). Blending math is the
  PerformAlphaBlending/PerformAlphaWeightedAdd/PerformMulBlending port
  (premultiplied vs non-premultiplied alpha, clamp flags, the kBlend
  alpha-output rewrite, extra channels blended first with pre-blend
  alpha); int<->float conversion matches dec_modular
  (`f32(1.0/(2^bits-1))` scaling, round-half-even output via the
  NearestInt port).
- Frames with duration > 0 (plus the last frame) snapshot the canvas;
  duration-0 and reference-only frames update state silently. tps/loops/
  timecodes come from ImageMetadata.animation.
- Restrictions: modular encoding, color_transform none, upsampling 1,
  single pass, no loop filter, extra channels limited to one alpha
  (matches the pixel-buffer surface); DC frames, patches/noise/splines
  are rejected as before.

## 13. Animation encode (new: `encodeAnimation` + CLI)

`encodeAnimation` writes the codestream headers once (with
`have_animation`, tps 1000/1, loops) and then one full-canvas modular
frame per input: the FrameEncoder pipeline (transforms, trees, entropy
coding) is reused per frame via the extracted `prepareFrame`/
`writeCodestreamHeaders`/`writeFrame` helpers; each frame carries its
duration, `is_last` on the final one, and `save_as_reference=1` otherwise
(the cjxl coalesced-animation shape). CLI: `jxltool encode-anim <out.jxl>
<f1.pnm> [f2...] [--duration-ms N] [--loops N] [encode options]` and
`jxltool decode-anim <in.jxl> <prefix>` (per-frame PAM/PNM +
`<prefix>.frames.txt` duration sidecar).

## 14. Validation (libjxl v0.12.0 cjxl/djxl)

| Direction | Result |
| --- | --- |
| cjxl animations (`-d 0`: RGB, RGB+alpha, gray; efforts 7/9; `-R 1`) -> port `decode-anim` vs djxl APNG | 6/6 files, all frames pixel- and duration-exact |
| cjxl cropped+blended animation (50x30 crops, disposal-2 GIF) -> port vs djxl APNG | pixel- and duration-exact |
| port `encode-anim` (RGB 3-frame, RGBA 4-frame) -> djxl APNG vs sources | pixel- and duration-exact |
| full regression: stills (24 encode + 32 decode combos), ICC (3), JPEG transcode (17) | 107/107 |

`zig build test`: **195 tests pass** (was 187): +8 animation tests
(RGB/alpha/gray roundtrips incl. per-frame durations and loops,
single-frame == still equivalence, multi-group animations, round-half-even
checks, mismatched-frame rejection).

---

# Round 5: encoder-side CfL + custom coefficient orders for JPEG recompression (vs. libjxl v0.12.0)

The JPEG transcode path could *decode* cjxl's default-effort files (which
always carry chroma-from-luma maps and custom coefficient orders), but the
port's own encoder wrote zero CfL maps and natural zig-zag orders — the two
libjxl encoder features that are on by default
(`cparams.force_cfl_jpeg_recompression = true`; `ComputeUsedOrders`
customizes orders at every speed tier slower than kFalcon). This round
closes both gaps, so the port's default output now has the same feature
shape as cjxl's and compresses measurably better.

## 15. Encoder-side chroma-from-luma (was: zero maps)

**libjxl**: `ComputeJPEGTranscodingData` (enc_frame.cc) runs the
FindCorrelation tile loop for 444 3-component frames — per 8x8-block tile
it votes, per chroma coefficient, the interval of CfL factors that would
zero the residual (`d_num_zeros` + `FindAvgIndexOfSumMaximum<256>`, with
the `kZeroBiasDefault` zero-threshold and the XYB=false base of 127), then
subtracts the fixed-point `cfl_factor` from every chroma coefficient
(`QCR = QChroma - cfl_factor(Y)` via `RatioJPEG` and the
`scaled_qtable` quant-ratio table) and ships the maps in the ACMetadata
modular streams (ConvertPlaneAndClamp of `cmap.ytox_map`/`ytob_map`).

**Implemented** (`src/vardct/frame_encode.zig`, `TranscodeData`):
- `scaled_qtable`: the transposed (X-table / c-table) 2^11 fixed-point
  ratios, built alongside the existing range check (identical
  construction to the decoder's, so encode/decode share the exact
  semantics).
- `findCorrelation`: the full tile search in f32 — same expression order
  as the C++ (`scaled_m`, `scaled_s`, the [from, to] zero-threshold
  interval, ceil/floor bucket votes, `best_sum > offset_sum + 1`
  activation rule, int8 wraparound for `best == 255`).
- QCR subtraction inside the coefficient extraction loop (integer
  fixed-point, i64 where C++ uses int — identical values wherever the C++
  is defined), per-tile map lookup, and the ACMetadata stream channels 0/1
  now carry the real map tiles over each DC group's tile rect.
- The CfL DC section and the ACMetadata maps the decoder reads were
  already in place (validated against cjxl output in earlier rounds).

## 16. Encoder-side custom coefficient orders (was: used_orders = 0)

**libjxl**: `ComputeUsedOrders` marks the DCT-8 order bucket for
customization (unless the image is < 5x5 blocks in both dimensions);
`ComputeCoeffOrder` gathers per-position zero counts over a 50% block
sample drawn by a fixed-seed xorshift128+ stream
(`{0x94D049BB133111EB, 0xBF58476D1CE4E5B9}`), quantizes them
(`num_zeros / sqrt(64) + 0.1`), stable-sorts positions by
`(count << 16) | index`, and `EncodeCoeffOrders` serializes each channel's
order as a Lehmer-coded permutation (Fenwick-tree `ComputeLehmerCode`,
trailing-zero digit trimming, `kPermutationContexts` histograms built
with a default `HistogramParams`). When the learned order equals the
natural one the bit is cleared again (`is_nondefault`).

**Implemented** (`src/vardct/coeff_order.zig` + `frame_encode.zig`):
- `computeLehmerCode` (lehmer_code.h port), `tokenizePermutation`,
  `encodeCoeffOrders` — the histogram section uses the vendored entropy
  stack with libjxl's default `HistogramParams` (kBest clustering, kRLE
  LZ77, kPrecise).
- `computeCoeffOrder`: the xorshift128+ sampler (reference values
  unit-tested), per-group contiguous channel storage materialized in
  full-grid scan order, and — verbatim — libjxl's shared `ac_offset`
  bookkeeping (it advances only for sampled blocks and indexes every
  channel's storage uniformly; for subsampled layouts this reads beyond
  the channel's written blocks, where libjxl reads whatever the ACImage
  plane holds — this port reads deterministic zeros there, which yields
  equally valid but not bit-identical orders; the orders are transmitted
  either way).
- `TokenizeCoefficients` now takes per-channel scan orders; the AC global
  section writes the computed `used_orders` bitfield followed by the
  permutations; the orders are learned from the *post-CfL* coefficient
  planes exactly like libjxl (CfL runs first, then
  `ComputeAllCoeffOrders`).

## 17. Latent encoder bug fixed (found by the new paths)

`buildAndEncodeHistograms` implements libjxl's
`tokens = std::move(tokens_lz77)` semantics: when LZ77 is adopted the
input token lists are *replaced in place*. The VarDCT transcode path
converted the original lists again for `writeTokens`
(`writeModularStream`) — harmless for every previously-produced file
(LZ77 adoption never triggered for those streams) but a silent
corruption (dangling buffer / plain tokens under LZ77 histograms) once
the new permutation streams triggered adoption. All call sites now pass
and write the same list storage, matching `encoder.zig`'s modular-image
path and libjxl.

## 18. CLI

`jxltool jpeg-transcode` gains `--brotli-effort 0..11` (libjxl
`cparams.brotli_effort`), `--no-cfl` and `--no-coeff-reorder` (A/B
switches for the two new features; both default on, like cjxl).

## 19. Validation (libjxl v0.12.0 cjxl/djxl)

| Direction | Result |
| --- | --- |
| port jpeg-transcode (cfl+orders on) -> djxl JPEG reconstruction + pixel decode | 37/37 files byte-exact, 74/74 checks (fresh 17-file corpus: 444/422/420/440 samplings, gray, progressive, restart markers, 400x300..2100x1400; plus the 20-file unit corpus, incl. two-DQT/Exif/ICC/multi-group) |
| port self roundtrip, all 4 option combinations | 37/37 byte-exact |
| cjxl transcode (efforts 7/9) -> port jpeg-untranscode | 74/74 byte-exact (regression: cjxl's own CfL/orders streams) |
| sizes | port default now matches or beats cjxl default on nearly every file (text_800x600 135315 vs 143058, big_gray 14305 vs 16235, photo_2100x1400_q92_444 959720 vs 963925); vs the old port shape: two_dqt_444 67864 -> 55528 (-18%), text_800x600 143042 -> 135315 (-5%), photo_1024x768_q88_444 157767 -> 156484; on tiny images both cjxl and the new default pay a few bytes of permutation overhead (e.g. baseline_420 1109 -> 1112 vs cjxl 1142) |

Full regression of the unaffected paths (libjxl v0.12.0): stills
encode->djxl 48/48, cjxl stills (efforts 1/9, default/`-R 1`) -> port
decode 32/32 pixel-exact, ICC 3/3.

`zig build test`: **204 tests pass** (was 195): +5 coeff_order unit tests
(Lehmer encode/decode inverse, permutation write/read roundtrip through
the real entropy stack, used_orders=0 shape, xorshift128+ reference
sequence, computeCoeffOrder small/zero/asymmetric cases), +2 transcode
option-matrix tests (all four combinations byte-exact, CfL engages and
helps on a 444 file; subsampled/gray stay correct with every vector),
+2 frame_encode tests (maps engage for 444 and stay empty otherwise;
QCR subtraction is the exact inverse of the decoder's applyCfl).

---

# Round 6: float samples (PFM, fp32/fp16) — vs. libjxl v0.12.0

The port was integer-only: PFM (HDR) input was rejected and float-sample
codestreams (bit_depth.floating_point_sample) failed to decode. This
round adds the full libjxl float-sample path (the `cjxl -d 0` PFM shape)
in both directions: `decode` returns integer samples as `.int` and float
samples as `.float` (a `DecodedImage` union now).

## 20. PFM front-end (`src/pnm.zig`)

`readFloat` parses PF/Pf (32-bit IEEE-754, either endianness, rows
bottom-up) into a `FloatPixelBuffer`; `writeFloat` serializes it back
byte-identically to djxl's PFM output (big-endian samples, +1.0 scale).
`isPfm` auto-detects the format, so `jxltool encode in.pfm out.jxl`
needs no switch.

## 21. Float encode (`encoder.encodeFloat`, `float_convert`)

- Every sample is converted to the custom-float bit pattern
  (`enc_modular`'s `float_to_int`): fp32 patterns (bits 32 / exp 8) by
  default, **fp16** (bits 16 / exp 5) with `--float16` — the encoder
  refuses values that are not exactly representable
  (`FloatLosesPrecision`), unlike libjxl which silently quantizes.
- `--resampling N` works on float input: the image is box-downsampled in
  float space exactly like libjxl's encoder (average of the source
  floats, not of the bit patterns) and the frame carries `upsampling=N`.
- Transform guards exactly as `enc_modular.cc` computes them
  (`max_bitdepth = bits + (fp ? 0 : 1)`, level cap 32): **fp32 patterns
  get no RCT and no palette** (libjxl forces `colorspace = 0` —
  `max_bitdepth + 1 > level_max_bitdepth` — and `ForwardPalette` returns
  false for `bitdepth >= 32`); fp16 patterns (bitdepth 16) keep the full
  default pipeline: global/local palettes, the 7-candidate local RCT
  search and responsive squeeze.
- `fwdRctRow` now uses int32 wraparound arithmetic (`+%`/`-%`,
  `PixelAdd`), the exact semantics the C++ compiles to on real targets
  — float bit patterns span the full int32 range and the intermediates
  can overflow.

## 22. Float decode (`decoder.assembleFloatFast` / `assembleUpsampled`)

- `int_to_float` per sample (dec_modular's `IntToF`), PFM-style output:
  no alpha (float extra channels are rejected with
  `UnsupportedFloatSample`, matching the PFM output surface).
- The upsampled pipeline (frame or extra-channel upsampling > 1):
  int→float, then the NLEE upsampler with the CustomTransformData
  weight sets, then raw float output — matching libjxl's dec_modular →
  UpsamplingStage → float output pipeline (integer samples keep the
  dithered/NearestInt conversion).

## 23. CLI

`jxltool encode <in.pfm> <out.jxl> [--float16] [--resampling 2|4|8]
[encode options]` and `jxltool decode <in.jxl> <out.pfm>` (float files
auto-write PFM; integer files still write PNM/PAM).

## 24. Validation (libjxl v0.12.0 cjxl/djxl; `validation/xval_float.sh`)

| Direction | Result |
| --- | --- |
| port fp32 encode (5 images x options matrix: default, gss 0/3, responsive, colorspace 5, odd sizes) -> djxl PFM | values exact (all) |
| port fp16 encode (incl. 520x300 multi-group + palette) -> djxl PFM | values exact (all) |
| port `--resampling 2/4` (lossy box+NLEE) -> port decode == djxl decode | values exact (incl. odd sizes) |
| cjxl fp32 (efforts 3-9, default/`-R 1`) -> port decode == djxl decode | values exact (41 files) |
| port self roundtrips (fp32/fp16, default/responsive) | values exact |
| regression: stills (options matrix), ICC, JPEG transcode (djxl auto-reconstruction), animations | 91/91 |
| `zig build test` | **222 pass** (was 204): +4 PFM IO tests, +5 float_convert tests, +4 float roundtrips, +4 float pixel-buffer/decoder tests |

Notable: **cjxl v0.12.0 itself fails to encode PFM input at efforts 1-2**
(`JxlEncoderProcessOutput failed`, gray or color, any content) — the
low-effort path builds a *fixed* global tree (kThunder→kGradientFixedDC,
kFalcon→kWPFixedDC) that does not support 32-bit float bit patterns — and
it fails intermittently at efforts 3-7 on some content (e.g. a smooth
200x150 float gradient at e7). Those encoder-effort cells are skipped in
the matrix (9 skips); the port encodes every one of those inputs cleanly
(learned trees, no fixed-tree shortcut) and djxl decodes the port's files
value-exactly. On the cells where cjxl does run, the port matches or
beats its size (60x40 HDR gradient: port 9059 B vs cjxl 9231 B at e9).

---

# Round 7: progressive lossless + TOC permutation — vs. libjxl v0.12.0

Two decode-side gaps remained where cjxl-produced lossless files were
rejected outright, plus the matching encoder feature:

## 25. Progressive (multi-pass) modular frames (was: rejected)

**libjxl**: `cjxl -d 0 -p` (and every `-p` lossless encode) writes a
*progressive* codestream: `Passes` in the FrameHeader carries
`num_passes > 1` with `(downsample, last_pass)` pairs (the lossless shape is
the qprogressive mode `progressive_passes_dc_quant_ac_full_ac`: 2 passes with
(2, 0)), the responsive Squeeze transform supplies channels at several
resolutions, and each AC group section is emitted **per pass** — the pass's
channels are those whose min(h,v)-shift falls inside the pass's downsampling
bracket (`Passes::GetDownsamplingBracket`). Decoding walks
`AcGroupIndex(pass, group)` (`2 + num_dc_groups + pass * num_groups + group`)
with the stream id `ModularStreamId::ModularAC(group, pass)`.

**Port before**: `frame.passes.num_passes != 1` returned
`UnsupportedFrameFeatures` — every `-p` file failed.

**Implemented** (`src/header/frame.zig`, `src/codec/decoder.zig`,
`src/codec/animation.zig`):
- `Passes.getDownsamplingBracket` rewritten to the exact libjxl algorithm
  (the previous hand-rolled variant produced wrong brackets; unit tests
  pin the semantics for 1/2/3/4-pass layouts, including the (8,0) case
  where pass 0 has an empty AC bracket because the coarse channels travel
  in the DC group sections).
- The frame decoder loops over passes, computes each pass's bracket and
  decodes every AC group section `acGroupIndex(pass, g, ...)`
  (decode-side `ProcessACGroup`); `decodeACGroup` takes the pass and
  bracket; the modular stream id is `modularAC(g, pass)`. The single-section
  special case now applies only when `num_groups == 1 && num_passes == 1`
  (progressive single-group frames carry 2 + ndc + np sections, the pass
  ones empty — everything decodes in the global stream).
- The animation decoder's frame gate no longer rejects multi-pass frames
  (`decodeFrameImage` shares the pass loop), so `cjxl -d 0 -p` animations
  (APNG/GIF input) decode frame- and duration-exactly.

## 26. TOC permutation (was: rejected)

**libjxl**: `cjxl --group_order=1` (center-first group order) writes the
frame TOC with the permutation bit set: a standalone Lehmer-coded
permutation (its own `kPermutationContexts` ANS histograms —
`DecodePermutation`) reorders the AC group sections, and the size list
order is *physical* while `permutation[i]` names the slot holding logical
section `i` (`ReadToc` + `ReadGroupOffsets`: logical i lives at
`prefix_sum(sizes)[permutation[i]]`).

**Port before**: `readToc` rejected any permuted TOC.

**Implemented** (`src/vardct/coeff_order.zig`, `src/header/toc.zig`):
- `coeff_order.decodePermutation` / `encodePermutation`: standalone
  ports of libjxl `DecodePermutation`/`EncodePermutation` (the same
  kPermutationContexts histograms + Lehmer/Fenwick machinery the
  coefficient orders use).
- `toc.readTocWithPermutation` returns a `Toc` (physical-order sizes +
  optional permutation, validated as a bijection); `Toc.logicalLayout`
  computes per-logical-section byte ranges (`starts[i]`/`sizes[i]`).
  The still decoder, the animation decoder and the VarDCT JPEG-transcode
  decoder all consume the layout (the VarDCT path itself never *writes*
  permutations, but cjxl `--group_order=1` files now decode).

## 27. Encoder: `--progressive` (`cjxl -p` equivalent)

`EncodeOptions.progressive` (CLI `--progressive` / `-p`, also on
`encode-anim`): forces responsive (libjxl "Modular has to be squeezed to
show progressive HF passes"), writes the qprogressive Passes header
(np=2, (downsample,last_pass)=(2,0), shift[0]=1), and lays out the AC
group sections per pass with the bracket-partitioned channels
(enc_modular.cc's per-pass `AddModularGroupImage`); the TOC carries
num_groups * num_passes AC entries. Float samples
(`encodeFloat --progressive`) and animations
(`encodeAnimation` with `.progressive = true`) go through the same path.

## 28. Validation (libjxl 0.11.2 cjxl/djxl — the field layouts this round
depends on are byte-identical in v0.11.2 and v0.12.0: SizeHeader,
ImageMetadata, CustomTransformData, Passes/GetDownsamplingBracket and the
toc/coeff_order permutation code all diff clean between the two versions;
`validation/xval_progressive.sh`)

| Direction | Result |
| --- | --- |
| cjxl `-d 0 -p` (4 images x efforts 1/2/5/7, incl. odd sizes, noise, multi-group) -> port decode == djxl | 16/16 pixel-exact |
| cjxl `--group_order=1` (multi-group) -> port decode == djxl | 2/2 pixel-exact |
| cjxl `-p --group_order=1` combined -> port decode == djxl | 2/2 pixel-exact |
| port `--progressive` (3 images x gss 0/1/3) -> djxl and self decode | 18/18 pixel-exact |
| cjxl `-p` APNG animation -> port `decode-anim` vs djxl APNG frames | 4/4 frames pixel-exact |
| port `encode-anim --progressive` -> djxl APNG + self | 4/4 frames pixel-exact (both) |
| port `--progressive` on PFM float input -> djxl PFM + self | values exact (both) |
| regression: plain / `-R 1` stills, JPEG transcode, animations, float samples | all pass |
| `zig build test` | **234 pass** (was 222): +1 bracket unit test (1/2/3/4-pass layouts), +1 Passes read/write roundtrip, +2 TOC permutation tests, +7 progressive roundtrips (single/multi-group, noise, odd, 16-bit gray, rgba, multi-DC-group), +1 progressive animation roundtrip |

---

# Round 8: encoder completion — the effort ladder and TOC permutation encoding

This round closes the two remaining encoder-side feature gaps against
libjxl v0.12.0: the cjxl effort ladder (`-e 1..10` — the slower tiers'
cost-based heuristics that were explicitly listed as unsupported, plus the
faster tiers' fixed trees) and the *encoder-side* TOC permutation
(`--group_order=1`, previously decode-only). Both are wired into every
encode path (stills, floats, animations, progressive frames, the JPEG
transcode).

## 29. Encoder TOC permutation: `--group-order 1` (was: decode-only)

**libjxl**: `cparams.centerfirst` (cjxl `--group_order=1`) makes
`PermuteGroups` (enc_frame.cc) reorder the AC-group sections
center-first — groups sorted by (Chebyshev distance from the central
group, rotated angle), the permutation Lehler-coded into the TOC by
`WriteTocPermutation` + `EncodePermutation` over the shared
kPermutationContexts histograms.

**Implemented** (`src/header/toc.zig`: `permuteGroups`,
`writeTocWithPermutation`):
- `permuteGroups` is a line-faithful port of `PermuteGroups`: the image
  center (cjxl defaults, no `--center_x/y`), the central group's center,
  the `side` rotation via `fmod(-atan2(...) + 5pi/4, 2pi) * 2 / pi`, and
  the per-group key (max(|dx|,|dy|), `remainder(atan2(dy,dx) + pi/4 +
  side*pi/2, 2pi)`), stable-sorted; the globals/DC head stays identity and
  every pass block carries the same inverse center-first order.
  One subtlety mattered for bit-parity: C `std::remainder` rounds the
  quotient **ties-to-even** — with `@round` (half-away) the angle of
  exactly ±pi flips sign and the ring rotates. `roundTiesToEven` fixes
  that; with it the emitted permutation arrays are **identical to cjxl's**
  on every tested geometry (see validation).
- `writeTocWithPermutation` ports `WriteTocPermutation`+`WriteTocSizes`:
  permutation-present bit, `encodePermutation` (already in
  coeff_order.zig for the JPEG custom orders — the same
  kPermutationContexts machinery), byte alignment, sizes in physical
  order; the section writers are permuted exactly like libjxl moves its
  `group_codes`.
- Wired into `encoder.writeFrame` (stills/floats/animations/progressive —
  multi-pass frames write per-pass blocks), `frame_encode.zig`
  (JPEG transcode) and the CLI (`encode`, `encode-anim`,
  `jpeg-transcode`; also `jxldbg --toc <file>` dumps the permutation for
  parity testing).

## 30. Encoder effort ladder: `--effort 1..10` (was: kSquirrel only)

**libjxl**: `JxlEncoderFrameSettingsSetOption(EFFORT)` maps effort e to
`SpeedTier(10-e)`; the tier changes the modular lossless heuristics in
enc_modular.cc (property sets, palette gates, the RCT candidate count,
`maybe_do_transform` reverts, the WP-mode search, the fixed trees of
ComputeTree) and `HistogramParams::ForModular` in enc_ans.cc.

**Implemented** (`encoder.zig`: `SpeedTier`/`TierParams`/`tierParams`;
`enc_transforms.zig`: `estimateWPCost`, `searchWpMode`,
`doTransformMaybe`; `encoder.zig`: `makeFixedTree`):
- Efforts 8/9/10 (kKitten/kTortoise/kGlacier): `maybe_do_transform` —
  palettes whose post-transform `EstimateCost` exceeds the pre-transform
  cost are undone (`doTransformMaybe` runs the recorded inverse and pops
  the transform, exactly libjxl's `t.Inverse(image)` + pop); the RCT
  search tries 9/19 candidates (the full 19-entry libjxl list);
  per-stream weighted-predictor mode search — `EstimateWPCost` (the 33
  cutoff contexts over the WP error property, hybrid-uint extra bits, the
  "unhealthy" overflow guard) over the five `PredictorMode` presets, the
  winning header serialized per group and used consistently in tree
  training (`TrainStream.wp_header`), tokenization and the GroupHeader;
  the 10/16-property training sets (128/256 quantized values, the libjxl
  node thresholds 75+14*tier, group-id property kept at tortoise/glacier)
  and the kBest/kLZ77/kOptimal histogram parameters.
- Efforts 1-6 (lightning..cheetah): the `MakeFixedTree` ports — kWPFixedDC
  (falcon/e3), kGradientFixedDC (thunder/e2), the single gradient leaf
  (lightning/e1) with the small-image height reduction and the
  >11-bit cutoff scaling; the global YCoCg replaces the local RCT search
  (the corrected `colorspace < 0 && (responsive || tier > kHare)` gate),
  no local palettes at cheetah and faster, the tightened property sets of
  hare/wombat, and the arbitrary-bpp palette cost estimate of the fast
  tiers.
- The default effort 7 keeps the kSquirrel heuristics; its property set
  now matches libjxl exactly (7 properties of `prop_order`, node
  threshold 117) and the pre-transform palette gate follows libjxl's
  `speed_tier <= kThunder || bits > 8` (the channel-palette pass now keys
  on `channel_colors_percent` alone, like `try_palettes`).

## 31. Latent bugs found by the new paths

- **kBest-clustering double free** (`enc_cluster.zig`): the merge
  compaction moved live histograms down with a shallow struct copy and
  left the moved-from slot aliasing the same `counts` buffer — the tail
  free then released it and `histogramReindex` freed it a second time.
  Only reachable with `clustering = kBest` (first enabled by efforts
  8-10); the compaction now empties the moved-from slots.
- **`quantizeAbsPixelPropCopy`/`quantizeAbsDiffPropCopy` leaks**
  (`ma_encode.zig`): the side-effect calls to the `...PropCopy` helpers
  discarded their returned threshold ArrayLists. Only reachable with
  properties 4-8 in the training set (the 16-property tortoise set).
- **`decode-anim` sidecar leak** (`main.zig`): the per-frame
  `allocPrint` duration lines were appended without freeing.

## 32. Tests

- `zig build test`: **250 pass** (was 234): +3 permuteGroups tests (the
  2x2 center-first order, the no-permutation single-group shape, and the
  bijection/identity-head/center-rank/per-pass-block invariants),
  +4 WP-mode tests (estimateWPCost sanity, searchWpMode argmin fidelity,
  the non-default-mode usefulness probe, the maybe-do-transform revert on
  a harmful palette, the 0-candidate RCT no-op), +8 roundtrips (the
  effort ladder over noise/gradient/16-bit-alpha, effort 8-10 responsive
  and progressive, group-order multi-group/multi-DC/progressive/effort-9
  combinations).

## 33. Validation (`validation/xval_effort.sh`, libjxl v0.12.0 cjxl/djxl,
105/105)

| Direction | Result |
| --- | --- |
| port `--effort 1..10` (3 images x 10 efforts) -> djxl + self decode | 60/60 pixel-exact |
| port `--effort 8-10` combos (-R 1, -p, --group-order, gss 0, palette 0) -> djxl | 5/5 pixel-exact |
| cjxl `-d 0 -e 8/9/10` -> port decode | 6/6 pixel-exact |
| port `--group-order 1` (3 images x 4 option combos) -> djxl + self | 24/24 pixel-exact |
| permutation parity: port `--group-order 1` vs cjxl `--group_order=1` | 8/8 geometries identical (incl. odd sizes, gss 0/1/2) |
| progressive 2-pass permutation parity vs cjxl | identical |
| port `jpeg-transcode --group-order 1` (20-file corpus) -> djxl byte-exact JPEG | 20/20 |
| cjxl `--group_order=1` transcode -> port `jpeg-untranscode` | 19/19 byte-exact (the 20th is the ICC file cjxl itself rejects) |
| float samples at efforts 8-10 -> djxl + self | values exact (both) |
| animation `--effort 9` / `--group-order` / combined -> self + djxl | frames pixel-exact |
| regression: default matrix (plain/-R 1/-p/palette 0/colorspace/gss 0+3, gray+RGB+alpha) | all pixel-exact |

## 34. Patches (reference patches, spec 18181-1 §6.10.3; was: missing)

The last big functional gap this port had versus the official libjxl:
**the patch dictionary**. Repeated image fragments (text, UI elements,
logos) are coded once in a patch reference frame and blended back at
their occurrences, which cjxl uses by default at effort >= 7
(`cjxl --patches`, libjxl `FindTextLikePatches` /
`FindBestPatchDictionary`). Before this round the port rejected
everything patch-shaped: `kPatches` flag files failed, no
`kReferenceOnly` frames existed, `--patches` did not exist.

**Implemented** (`src/patches/dictionary.zig`, `src/patches/enc_patches.zig`,
both written from scratch in the previous round but never wired in;
integration + encoder reference-frame path this round):

- *Decoder*: `decode()` runs a frame loop; `kReferenceOnly` frames are
  decoded and stored (float [0,1] planes, per-channel
  SingleFromSingle bit depths) into the 4-slot reference store with the
  `save_before_color_transform` flag; a last regular frame carrying
  `kPatches` reads its patch dictionary from the head of the DC global
  section (libjxl `ProcessDCGlobal` order: patches first, before the
  dequant DC bit) and blends it into the decoded channel planes with
  `PerformBlending` (kAdd/kReplace/kNone/mul/alpha-weighted/clamp math)
  after all transforms are undone and before the output assembly.
  `decodeFrameImage` gained the same support, so animations decode
  patch frames too (per-frame reference stores, pre-transform saves).
- *Encoder*: `EncodeOptions.patches` (`PatchOverride`, cjxl
  `--patches=-1|0|1`; default: search at effort >= 7 like
  FindTextLikePatches, `.on` forces it at effort >= 5, `.off` disables)
  with the libjxl enc_modular gate (integer input, <= 16 bits,
  resampling 1). The search (`findTextLikePatches`, the non-XYB
  branch: 4x4 screenshot-like seeds, background flood-fill, connected
  components, quantized fg-bg difference) and bin-packing
  (`packPatches`, grow-by-1.05 until it fits) run between
  `buildChannels` and the global transforms; the packed reference image
  is encoded as its own `kReferenceOnly` frame — signed modular planes
  (`InputImage.planes`, new: the patch content is a *difference* image,
  negative where a fragment is darker than its background, which no
  displayable `PixelBuffer` can carry) — saved into slot 3 before the
  color transform, appended byte-aligned just ahead of its main frame
  (libjxl `AppendByteAligned(special_frames)`); the dictionary
  positions are written to the DC global head; the content is
  subtracted from the main frame channels in the exact integer domain
  (libjxl's float subtraction + float_to_int is bit-equal there), so
  the decoder's kAdd blend reconstructs the input bit-exactly.
- *CLI*: `encode`/`encode-anim` `--patches -1|0|1` (both `--patches N`
  and `--patches=N` forms, like cjxl).

**Latent bugs found and fixed in the previously-unwired modules**
(the `SubtractFrom` port had never executed once):
- `PatchDictionaryEncoder::SubtractFrom`: read the patch content from
  the reference store (libjxl `reference_frames_`), not from the frame
  being encoded; one blending info per color group
  (`blendings[blending_idx]` for all color channels), like libjxl; no
  `clamp01`/range clamps on the reconstruction — the patch content is
  negative for dark-on-light fragments and must stay signed.

**Validation** (`validation/xval_patches.sh` +
`validation/xval_patch_regression.sh`, libjxl v0.12.0 cjxl/djxl):

| Direction | Result |
| --- | --- |
| port `--patches=1` (4 text-like images: RGB/RGB-alpha/16-bit) -> port decode | pixel-exact |
| port `--patches=1` -> djxl | pixel-exact |
| cjxl `-d 0 --patches=1` (efforts 7-10 default, 5-6 forced) -> port decode == djxl | 24/24 pixel-exact (identical patch sets: 195 positions, 11 references) |
| patch detection parity: port vs cjxl on the same image | identical dictionaries |
| sizes: port 357 B vs cjxl 362 B on the 256x256 text corpus | port smaller |
| port `-e 5/7` ladder with patches default (on at >= 7) -> djxl | pixel-exact |
| animations: port `encode-anim --patches=1` (2 frames) -> self + djxl APNG | frames pixel-exact |
| cjxl APNG `--patches=1` -> port `decode-anim` == djxl | frames pixel-exact |
| post-patch regression: 82 checks (options matrix, cjxl matrix, progressive/group-order/resampling, animation tail) | all pixel-exact |
| `zig build test`: 269 tests incl. 9 new patch tests (roundtrips, codestream shape: [kReferenceOnly slot 3 pre-transform] + [regular kPatches last], no-patch single-frame, effectiveness) | all pass |

New validation assets: `gen_patch_images.py` (text-like corpus),
`compare_ppm.py` (PNM/PAM exact-raster comparer), `make_apng.py`
(deterministic 2-frame APNG builder for cjxl animation input;
per-frame independent zlib streams, the layout libpng accepts).

---

# Round 10: float-sample (binary16) patches (vs. libjxl v0.12.0)

The last sample-format gap of the patch dictionary: libjxl's
`enc_modular.cc` gate for the patch search is
`metadata.bit_depth.bits_per_sample <= 16` — which does not mean
"integer up to 16 bits" but *any* format up to 16 bits per sample,
**including 16-bit float (binary16)**. cjxl's CLI cannot even write fp16
samples (no `--float16` flag), but the libjxl encoder API can
(`JxlEncoderAddImageFrame` with `JXL_TYPE_FLOAT16` +
`exponent_bits_per_sample = 5`), and its patch search runs there. The
port rejected the whole float domain before this round: the decoder
failed with `UnsupportedFloatSample` and the encoder gate returned
early on `.float` input.

## 35. fp16 reference patches, encoder and decoder

**Implemented** (on top of the §34 integer patch path):

- *Shared sample format* (`patches/dictionary.zig`): `SampleFormat`
  (bits / exp_bits / floating) plus the two conversions every patch
  site needs — `sampleToFloat` (integers scale by 1/maxval like
  `ConvertFromExternal`; float patterns go through the bit-exact
  `int_to_float` port) and `floatToSample` (integers: clamp01, x maxval,
  round-to-nearest-even, clamp; float patterns: `float_to_int`, which
  errors on values not exactly representable — the same failure libjxl
  reports from `enc_modular.cc`).
- *Decoder* (`codec/decoder.zig`): `storePatchReference` converts the
  reference frame's planes per channel — fp16 patch frames carry bit
  patterns, now turned into floats via `int_to_float` instead of the
  integer `v/maxval` scaling. `applyPatches` dropped the
  `UnsupportedFloatSample` rejection: the affected rows convert
  pattern→float, blend with the same `PerformBlending` math, and the
  write-back converts with `float_to_int` — exact for lossless files,
  since residual + content reproduces the original pattern bit-for-bit.
  Per-channel formats come from the metadata bit depths (color channels
  and extra channels each carry their own).
- *Encoder* (`codec/encoder.zig`): the `tryPatches` gate now follows
  libjxl exactly — `bits_per_sample <= 16` whatever the format, so
  binary16 (16/5) passes and fp32 (32 > 16) stays excluded, like
  libjxl. The search runs on the float view of the pattern planes (the
  actual sample values, exactly libjxl's float `Image3F` domain); the
  patch reference frame is written with the pattern width as the
  modular bit depth and inherits the codestream's float metadata;
  `subtractFrom` does the subtraction in float space and converts the
  residual back with `float_to_int`.
- *Search* (`patches/enc_patches.zig`): `findTextLikePatches` /
  `packPatches` take the `SampleFormat`. Float patches keep the raw f32
  difference bits in `fpixels`; `packPatches` converts them to the
  [bits]/[exp_bits] patterns of the reference image — the exact point
  where libjxl's patch-frame `float_to_int` fails on unrepresentable
  differences, so hostile inputs error out identically (after dedup and
  the size filters, matching libjxl's failure stage).
- *Animation decoder* (`codec/animation.zig`): float-sample animations
  are still out of scope (integer-only pipeline), but they are now
  rejected with `UnsupportedFloatSample` up front instead of silently
  scaling bit patterns into garbage.

**Behavior parity with libjxl v0.12.0** (all verified against a
reference build):

- fp16 text-like encode with patches is lossless and the file decodes
  in djxl value-exactly; on the 256x256 corpus the port's files are
  smaller than the API encoder's (384 B vs 430 B).
- Files produced by the libjxl API (patches on/off, efforts 5-9)
  decode identically in this port and in djxl.
- Hostile fp16 input (every sample fp16-exact but the glyph/background
  difference, 1 - 2^-13 = 8191 * 2^-13, needs 13 significant bits):
  both encoders fail the whole encode at default settings (libjxl:
  `float_to_int` "losing precision" inside the patch-reference-frame
  encode; the port: `FloatLosesPrecision` from `packPatches`), and both
  succeed with patches disabled.
- fp32 input never enters the patch search (32 > 16), same as libjxl.

**Validation** (`validation/xval_patches_float.sh` +
`gen_fp16_patch_images.py` + `compare_pfm.py`, 32/32 checks; reference
files produced with `scripts/make_fp16_patches.c` against the local
libjxl shared library — cjxl cannot write these files):

| Direction | Result |
| --- | --- |
| port `--float16 --patches=1` (RGB / gray / 640x400 multi-group) -> self + djxl | value-exact, patches shrink 694->384 B (RGB), 621->384 (gray), 2457->1141 (big) |
| port `--float16` effort ladder (5/6/7/9) -> djxl | value-exact; patches auto-on at >= 7 (384 < 1691 B at e5) |
| port `--float16 --patches=1` x options (group shifts 0/3, `-p`, `-p`+gss0, colorspace, `-e 10`) -> self + djxl | value-exact |
| libjxl API fp16 (patches 1/0, e5/e7/e9) -> port decode == djxl decode | identical output; the libjxl files are lossless (checked vs source) |
| hostile fp16: port `--patches=1` / default | encode fails (FloatLosesPrecision) — same as libjxl; `--patches=0` succeeds (both) |
| fp32 `--patches=1` | search skipped, single frame, roundtrip exact |
| regression: `xval_patches.sh` 43/43, `xval_patch_regression.sh` 82/82 | all pixel-exact |
| `zig build test` | 278 tests pass (was 269; +9 fp16 patch tests: roundtrips RGB/gray/default-effort/progressive+responsive, codestream shape, effectiveness, hostile failure, fp32 skip) |

---

# Round 11: lossy VarDCT patches (vs. libjxl v0.12.0)

The VarDCT half of the patch dictionary — the last piece of the
`--patches` feature this port did not implement. In libjxl the lossy
patch path lives in `enc_heuristics.cc` (`LossyFrameHeuristics`:
`FindBestPatchDictionary(opsin, ..., is_xyb=true)` on the raw XYB opsin
image, then `PatchDictionaryEncoder::SubtractFrom` before gaborish) and
on the decoder side in the render pipeline (`dec_cache.cc` order:
gaborish -> EPF -> **patches** -> upsampling -> XYB-to-linear). This
round adds the whole carrier path around it — the port had no
general-purpose lossy VarDCT at all (only the JPEG-transcode subset):

## 36. Lossy VarDCT frames with patches (encoder + decoder)

**New modules**:

- *`vardct/opsin.zig`* — the XYB color space: `linearRgbToXyb`
  (`enc_xyb.cc` OpsinAbsorbance + cube roots + the `(m0-m1)/2` mixing)
  and `xybToLinearRgb` (`dec_xyb-inl.h` XybToRgb with the spec inverse
  matrix, FMA chains preserved), the exact sRGB EOTF pair, the frozen
  matrices/biases of `cms/opsin_params.h` and `kDefaultQuantBias`.
- *`vardct/transform_lossy.zig`* — the spec "scaled" DCT-8 pair
  (`dct_for_test.h` convention: DC = block mean, blocks in the JXL
  transposed layout `block[8u+v]`), the gaborish filters (decoder: the
  normalized 3x3 kernel of `stage_gaborish.cc` with mirror borders;
  encoder: the 5x5 `GaborishInverse` kernel of `enc_gaborish.cc` with
  the WeightsSymmetric5 tap layout of `convolve_symmetric5.cc`),
  `AdjustQuantBias` (`quantizer-inl.h`), and the default Libshroom
  DCT-8 quant weights (`DequantMatricesLibraryDef::DCT` bands through
  `GetQuantWeights` — with libjxl's `FastPowf` ported
  operation-for-operation so the computed table matches djxl's).
- *`vardct/lossy.zig`* — the frame encoder and pixel decoder:

**Encoder** (`encoder.encodeLossy`, CLI `encode -d D`, cjxl `-d`):
sRGB integer RGB input -> linear -> XYB opsin planes -> patch search
(`findTextLikePatchesXyb`: the is_xyb=true branch — XYB similarity
weights {30,3,1}, quantization steps {0.01615, 0.08875, 0.1922}) ->
`packPatchesXyb` (bin-packing + the `enc_modular.cc` XYB modular
quantization: the distance-scaled `enc_factors {65536/(1+23d),
4096/(1+14d), 4096/(1+14d)}` F16-round-tripped like the DequantDC
field, the [Y, X, B-Y] channel order, `trunc(v*factor + 0.5)` rounding)
-> float kAdd subtraction on the opsin planes -> gaborish compensation
-> per-block DCT-8 with DC split off (`AddVarDCTDC` quantizer:
`InitialQuantDC`, `Quantizer::ComputeGlobalScaleAndQuant` +
`SetQuant`'s constant quant field q = 0.79/d — libjxl's simple branch,
not Butteraugli) -> AC quantization against the Libshroom weights,
serialized as a **RAW quant table** (den 2^-16, exactly representable in
F16). The patch reference frame is a `kReferenceOnly` modular XYB frame
(slot 3, saved before the color transform, custom DequantDC) carrying
the [Y,X,B-Y] planes; the main frame is kVarDCT/XYB with `kPatches`,
gaborish on, EPF off, adaptive DC smoothing on, zero-base CfL DC header
(no chroma-from-luma — legal, simpler, and the file stays djxl-valid).
Coefficients reuse the transcode machinery (orders, contexts, ANS).

**Decoder** (`lossy.decodeLossyFrame`, wired into `codec/decoder.zig`):
DC global (patch dictionary first) -> DC groups (VarDCTDC with the
CfL-DC-aware `DequantDC` of `compressed_dc.cc`, ACMetadata with the
CfL maps / ACS / quant field / EPF sharpness) -> `AdaptiveDCSmoothing`
-> AC global (RAW or all-default library tables, custom orders,
histograms) -> AC groups (8x8 `DecodeACVarBlock` reuse) -> dequant
(`DequantBlock`: bias correction, per-channel qm multipliers
`pow(1/1.25, qm_scale-2)` of `dec_cache.h`, CfL restore from the
dequantized Y block) -> DC -> IDCT-8 -> gaborish -> **patches**
(`PatchDictionary.applyToFloatImage`, the render-pipeline stage order)
-> XYB-to-linear. Output: sRGB-encoded PNM (exact inverse EOTF) or
`--linear` PFM. The frame-loop now also accepts the XYB modular patch
reference frames ahead of the VarDCT frame (`storePatchReference`
reconstructs X = ch1*dcq, Y = ch0*dcq, B = (ch2+ch0)*dcq from the
frame's own DequantDC — `ModularImageToDecodedRect`).

**Bug fixes in shared code found by the new paths**:
- `LoopFilter.read` hardcoded `nonserialized_is_modular = true`: VarDCT
  frame headers misparsed (the `epf_sharp_custom` bit and the modular
  sigma F16). Now threaded through like libjxl.
- The modular DC global now parses custom DequantDC values (needed by
  the XYB patch reference frames; everything else wrote all-default).

**Restrictions** (documented): RGB input only (no gray/alpha lossy),
DCT-8 only, EPF off, no noise/splines/upsampling, single pass. Default
cjxl lossy output (varblocks > 8x8, EPF, adaptive quantization) is
rejected with `UnsupportedFrameFeatures`; the DCT-8-only cjxl subset
(`cjxl -d D -e 1 --epf 0 [--gaborish 0] [--patches=1]`) decodes.

**Validation** (`validation/xval_lossy_patches.sh`, 30/30; libjxl
v0.12.0 cjxl/djxl, linear PFM compared with
`djxl --color_space=RGB_D65_SRG_Rel_Lin`):

| Direction | Result |
| --- | --- |
| port `-d 0.5/1/3 --patches=1` (256x256, 777x403 multi-group) -> port decode == djxl | max abs diff <= 1.6e-5 (float noise; identical pipeline) |
| port `--patches=0` (no-patch negative) -> djxl | same agreement |
| port 16-bit `-d 1 --patches=1` -> djxl | same agreement |
| port patches effectiveness | 1348 vs 14396 B (256x256, -91%), 1749 vs 16471 B (multi-group, -90%), 1278 vs 5249 (d=3, -76%) |
| 8-bit PPM port vs djxl (djxl dithers) | max +-1 LSB, 0 pixels beyond |
| cjxl `-d 1 -e 1 --epf 0` files (--patches=0/1, --gaborish 0/1) -> port decode == djxl | max abs diff <= 4.9e-6 |
| default cjxl lossy (varblocks+EPF) | rejected (documented restriction) |
| regression: `xval_patches.sh` 43/43, `xval_patch_regression.sh` 82/82, `xval_effort.sh` (62 checks run) all pass | no regressions |
| `zig build test` | 292 tests pass (was 285; +7 lossy tests: gradient quality, patch roundtrip exactness, codestream shape, effectiveness, gray/alpha/distance rejections, distance monotonicity, linear output) |
