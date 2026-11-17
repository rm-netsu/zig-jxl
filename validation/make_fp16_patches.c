/* Produces an fp16-sample JXL with reference patches via the libjxl C API
 * (the shape cjxl's CLI cannot write: no --float16 flag exists there, but
 * the encoder gate in enc_modular.cc allows patches for bits_per_sample
 * <= 16, which includes 16-bit float).  Reads a P6 PPM, converts samples
 * to exact dyadic values (v * 2^-8 — exactly representable in binary16),
 * encodes lossless modular with patches forced on/off.
 *
 * Usage: make_fp16_patches <in.ppm> <out.jxl> <patches 0|1> [effort] [mode]
 * mode 0 (default): exact dyadic samples v * 2^-8
 * mode 1 (hostile): bg 2^-11, fg 1.0 — the fg-bg difference
 *                   (0.99951171875) is NOT representable in binary16.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "jxl/encode.h"
#include "jxl/types.h"

static uint16_t f32_to_f16_bits(float v) {
  /* round-to-nearest-even f32 -> f16 bit pattern */
  uint32_t x; memcpy(&x, &v, 4);
  uint32_t sign = (x >> 16) & 0x8000;
  int32_t exp = (int32_t)((x >> 23) & 0xff) - 127;
  uint32_t man = x & 0x7fffff;
  if (((x >> 23) & 0xff) == 0xff) return (uint16_t)(sign | 0x7c00); /* inf/nan */
  if (exp > 15) return (uint16_t)(sign | 0x7bff); /* overflow -> max (caller avoids) */
  if (exp >= -14) { /* normal */
    uint32_t m = man >> 13;
    uint32_t rest = man & 0x1fff;
    uint32_t half = 0x0fff;
    uint16_t bits = (uint16_t)(sign | ((uint32_t)(exp + 15) << 10) | m);
    if (rest > half || (rest == half && (m & 1))) bits++;
    return bits;
  }
  if (exp < -25) return (uint16_t)sign; /* underflow to zero (caller avoids) */
  { /* subnormal */
    uint32_t full = man | 0x800000;
    int shift = -exp - 14 + 13;
    uint32_t m = full >> shift;
    uint32_t rest = full & ((1u << shift) - 1);
    uint32_t halfway = 1u << (shift - 1);
    uint16_t bits = (uint16_t)(sign | m);
    if (rest > halfway || (rest == halfway && (m & 1))) bits++;
    return bits;
  }
}

static float f16_bits_to_f32(uint16_t b) {
  uint32_t sign = (b & 0x8000) << 16, e = (b >> 10) & 0x1f, m = b & 0x3ff;
  if (e == 0) { if (m == 0) { float z = 0.0f; if (sign) z = -0.0f; return z; }
    float v = (float)m * 5.960464477539063e-08f; /* m * 2^-24 */
    return sign ? -v : v; }
  if (e == 31) { float inf = 1e30f * 1e30f; return sign ? -inf : inf; }
  uint32_t bits = sign | ((e - 15 + 127) << 23) | (m << 13);
  float out; memcpy(&out, &bits, 4); return out;
}

int main(int argc, char** argv) {
  if (argc < 4) { fprintf(stderr, "usage: %s in.ppm out.jxl patches01 [effort]\n", argv[0]); return 2; }
  int want_patches = atoi(argv[3]); /* -1 = default, 0 = off, 1 = on */
  int effort = argc > 4 ? atoi(argv[4]) : 7;

  FILE* f = fopen(argv[1], "rb");
  if (!f) { perror("open ppm"); return 1; }
  char magic[3] = {0,0,0};
  if (fscanf(f, "%2s", magic) != 1 || strcmp(magic, "P6")) { fprintf(stderr, "need P6\n"); return 1; }
  int w, h, maxv;
  if (fscanf(f, "%d %d %d", &w, &h, &maxv) != 3) { fprintf(stderr, "bad ppm hdr\n"); return 1; }
  fgetc(f);
  if (maxv != 255) { fprintf(stderr, "need maxv 255\n"); return 1; }
  size_t n = (size_t)w * h * 3;
  uint8_t* rgb = malloc(n);
  if (fread(rgb, 1, n, f) != n) { fprintf(stderr, "short ppm\n"); return 1; }
  fclose(f);

  /* exact dyadic samples: v * 2^-8 (mode 0) or hostile bg/fg (mode 1) */
  const int mode = argc > 5 ? atoi(argv[5]) : 0;
  const int use_f32 = argc > 9 ? atoi(argv[9]) : 0;
  float* f32 = malloc(n * 4);
  for (size_t i = 0; i < n; i++)
    f32[i] = mode ? (rgb[i] >= 128 ? 1.0f : 0.0001220703125f) /* 2^-13: 1-2^-13 needs 13 sig bits */
                  : (float)rgb[i] * 0.00390625f;
  uint16_t* f16 = malloc(n * 2);
  for (size_t i = 0; i < n; i++) {
    f16[i] = f32_to_f16_bits(f32[i]);
    if (f16_bits_to_f32(f16[i]) != f32[i]) { fprintf(stderr, "not fp16-exact at %zu\n", i); return 1; }
  }

  JxlEncoder* enc = JxlEncoderCreate(NULL);
  JxlBasicInfo info;
  JxlEncoderInitBasicInfo(&info);
  info.xsize = (uint32_t)w;
  info.ysize = (uint32_t)h;
  info.bits_per_sample = use_f32 ? 32 : 16;
  info.exponent_bits_per_sample = use_f32 ? 8 : 5;
  info.uses_original_profile = JXL_TRUE;
  info.num_color_channels = 3;
  if (JxlEncoderSetBasicInfo(enc, &info) != JXL_ENC_SUCCESS) { fprintf(stderr, "basicinfo\n"); return 1; }
  JxlColorEncoding color;
  JxlColorEncodingSetToSRGB(&color, JXL_FALSE);
  if (JxlEncoderSetColorEncoding(enc, &color) != JXL_ENC_SUCCESS) { fprintf(stderr, "color\n"); return 1; }

  const int skip = argc > 10 ? atoi(argv[10]) : 0;
  JxlEncoderFrameSettings* fs = JxlEncoderFrameSettingsCreate(enc, NULL);
  JxlEncoderSetFrameLossless(fs, JXL_TRUE);
  if (!(skip & 1)) JxlEncoderSetFrameDistance(fs, 0.0f);
  JxlEncoderFrameSettingsSetOption(fs, JXL_ENC_FRAME_SETTING_EFFORT, effort);
  if (!(skip & 2)) JxlEncoderFrameSettingsSetOption(fs, JXL_ENC_FRAME_SETTING_MODULAR, 1);
  JxlEncoderFrameSettingsSetOption(fs, JXL_ENC_FRAME_SETTING_PATCHES,
                                   want_patches < 0 ? -1 : want_patches);
  if (argc > 6) JxlEncoderFrameSettingsSetOption(fs, JXL_ENC_FRAME_SETTING_PALETTE_COLORS, atoi(argv[6]));
  if (argc > 7) JxlEncoderFrameSettingsSetOption(fs, JXL_ENC_FRAME_SETTING_CHANNEL_COLORS_GLOBAL_PERCENT, atoi(argv[7]));
  if (!(skip & 4)) JxlEncoderFrameSettingsSetOption(fs, JXL_ENC_FRAME_SETTING_RESPONSIVE, 0);

  JxlPixelFormat fmt = { 3, use_f32 ? JXL_TYPE_FLOAT : JXL_TYPE_FLOAT16, JXL_NATIVE_ENDIAN, 0 };
  const void* buf = use_f32 ? (const void*)f32 : (const void*)f16;
  size_t buf_size = use_f32 ? n * 4 : n * 2;
  if (JxlEncoderAddImageFrame(fs, &fmt, buf, buf_size) != JXL_ENC_SUCCESS) {
    fprintf(stderr, "add frame failed\n"); return 1;
  }
  JxlEncoderCloseInput(enc);

  FILE* out = fopen(argv[2], "wb");
  if (!out) { perror("open out"); return 1; }
  uint8_t outbuf[65536];
  JxlEncoderStatus st;
  do {
    uint8_t* next_out = outbuf;
    size_t avail_out = sizeof outbuf;
    st = JxlEncoderProcessOutput(enc, &next_out, &avail_out);
    if (st == JXL_ENC_ERROR) { fprintf(stderr, "process output error\n"); return 1; }
    size_t used = sizeof outbuf - avail_out;
    if (used && fwrite(outbuf, 1, used, out) != used) { perror("write"); return 1; }
  } while (st == JXL_ENC_NEED_MORE_OUTPUT);
  fclose(out);
  JxlEncoderDestroy(enc);
  fprintf(stderr, "wrote %s (patches=%d effort=%d)\n", argv[2], want_patches, effort);
  return 0;
}
