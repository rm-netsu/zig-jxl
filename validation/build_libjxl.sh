#!/bin/bash
# Clone and build libjxl v0.12.0 (cjxl + djxl) for cross-validation.
set -e
cd /tmp
rm -rf libjxl-clone
export GIT_LFS_SKIP_SMUDGE=1
git clone --depth 1 --branch v0.12.0 --recurse-submodules --shallow-submodules \
  --jobs 4 https://github.com/libjxl/libjxl libjxl-clone 2>&1 | tail -5
cd libjxl-clone
NPROC=$(nproc)
cmake -B build -DCMAKE_BUILD_TYPE=Release \
  -DJPEGXL_ENABLE_BENCHMARK=OFF \
  -DJPEGXL_ENABLE_DEVTOOLS=OFF \
  -DJPEGXL_ENABLE_EXAMPLES=OFF \
  -DJPEGXL_ENABLE_JNI=OFF \
  -DJPEGXL_ENABLE_MANPAGES=OFF \
  -DJPEGXL_ENABLE_OPENEXR=OFF \
  -DJPEGXL_ENABLE_PLUGINS=OFF \
  -DJPEGXL_ENABLE_SJPEG=OFF \
  -DJPEGXL_ENABLE_TCMALLOC=OFF \
  -DJPEGXL_ENABLE_DOXYGEN=OFF \
  -DJPEGXL_ENABLE_FUZZERS=OFF \
  -DJPEGXL_ENABLE_JPEGLI=OFF \
  -DBUILD_TESTING=OFF \
  -DJPEGXL_ENABLE_TOOLS=ON > /tmp/cmake_config.log 2>&1
cmake --build build --target cjxl djxl -j"$NPROC" > /tmp/cmake_build.log 2>&1
echo BUILD_OK
ls -la build/tools/cjxl build/tools/djxl
