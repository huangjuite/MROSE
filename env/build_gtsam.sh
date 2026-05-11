#!/usr/bin/env bash
set -e

SRC="$PIXI_PROJECT_ROOT/.pixi/third_party/gtsam"
BUILD="$PIXI_PROJECT_ROOT/.pixi/third_party/gtsam-build"

if [ ! -d "$SRC" ]; then
  git clone --branch 4.3a0 --depth 1 https://github.com/borglab/gtsam.git "$SRC"
fi

cmake -S "$SRC" -B "$BUILD" -G Ninja \
  -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_WITH_TBB=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_POSE3_EXPMAP=ON \
  -DGTSAM_ROT3_EXPMAP=ON

ninja -C "$BUILD" -j"$(nproc)"
ninja -C "$BUILD" install
