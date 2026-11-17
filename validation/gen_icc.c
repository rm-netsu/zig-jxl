// Generates real ICC profiles with lcms2 for cross-validation.
#include <stdio.h>
#include <stdlib.h>
#include "lcms2.h"
#include <string.h>

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: %s <srgb|gray|adobergb|linear> <out.icc>\n", argv[0]);
    return 2;
  }
  cmsHPROFILE p = NULL;
  if (strcmp(argv[1], "srgb") == 0) {
    p = cmsCreate_sRGBProfile();
  } else if (strcmp(argv[1], "gray") == 0) {
    // Gray with sRGB-ish gamma TRC.
    cmsToneCurve* gamma = cmsBuildGamma(NULL, 2.2);
    cmsCIExyY wp = {0.3127, 0.3290, 1.0};
    p = cmsCreateGrayProfile(&wp, gamma);
    cmsFreeToneCurve(gamma);
  } else if (strcmp(argv[1], "adobergb") == 0) {
    cmsCIExyYTRIPLE prim = {
      {0.6400, 0.3300, 1.0},
      {0.2100, 0.7100, 1.0},
      {0.1500, 0.0600, 1.0},
    };
    cmsCIExyY wp = {0.3127, 0.3290, 1.0};
    cmsToneCurve* trc[3];
    for (int i = 0; i < 3; i++) trc[i] = cmsBuildParametricToneCurve(NULL, 4, (const cmsFloat64Number[5]){}); // gamma 1.0 + ... use simple gamma
    (void)trc;
    cmsToneCurve* g = cmsBuildGamma(NULL, 2.19921875); // AdobeRGB gamma
    p = cmsCreateRGBProfile(&wp, &prim, (cmsToneCurve*[3]){g, g, g});
    cmsFreeToneCurve(g);
  } else if (strcmp(argv[1], "linear") == 0) {
    cmsCIExyYTRIPLE prim = {
      {0.6400, 0.3300, 1.0},
      {0.3000, 0.6000, 1.0},
      {0.1500, 0.0600, 1.0},
    };
    cmsCIExyY wp = {0.3127, 0.3290, 1.0};
    cmsToneCurve* g = cmsBuildGamma(NULL, 1.0);
    p = cmsCreateRGBProfile(&wp, &prim, (cmsToneCurve*[3]){g, g, g});
    cmsFreeToneCurve(g);
  } else {
    fprintf(stderr, "unknown kind\n");
    return 2;
  }
  if (!p) { fprintf(stderr, "profile create failed\n"); return 1; }
  if (!cmsSaveProfileToFile(p, argv[2])) { fprintf(stderr, "save failed\n"); return 1; }
  cmsCloseProfile(p);
  printf("written %s\n", argv[2]);
  return 0;
}
