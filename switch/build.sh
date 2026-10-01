#!/usr/bin/env bash
set -euo pipefail

# Builds switch/build/WatermelonDS.nro (standalone test) and switch/build/tico-watermelonds.nro (tico).
# Uses a local devkitPro install when there is one, otherwise the switch-dev Docker image.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${WATERMELON_SWITCH_BUILD_DIR:-${ROOT}/switch/build}"
JOBS="${BUILD_JOBS:-8}"
IMAGE="${SWITCH_DEV_IMAGE:-}"
DEVKITPRO="${DEVKITPRO:-/opt/devkitpro}"

if [[ ! -f "${DEVKITPRO}/cmake/Switch.cmake" ]]; then
  if [[ -z "${IMAGE}" ]]; then
    # the switch-dev image is retagged by date, so take the newest one that is present
    IMAGE="$(docker images --format '{{.Repository}}:{{.Tag}}' ghcr.io/autorunhq/switch-dev | sort -r | head -n 1)"
  fi
  if [[ -z "${IMAGE}" ]]; then
    echo "No devkitPro install at ${DEVKITPRO} and no ghcr.io/autorunhq/switch-dev image found." >&2
    exit 1
  fi
  exec docker run --rm -v "${ROOT}:${ROOT}" -w "${ROOT}" \
    -e BUILD_JOBS="${JOBS}" -e DEVKITPRO=/opt/devkitpro \
    "${IMAGE}" bash "${ROOT}/switch/build.sh"
fi

export DEVKITPRO
export PATH="${DEVKITPRO}/tools/bin:${DEVKITPRO}/devkitA64/bin:${PATH}"

cmake -S "${ROOT}/switch" -B "${BUILD_DIR}" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="${DEVKITPRO}/cmake/Switch.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG -fno-omit-frame-pointer" \
  -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG -fno-omit-frame-pointer"

cmake --build "${BUILD_DIR}" --parallel "${JOBS}"

echo "Built ${BUILD_DIR}/WatermelonDS.nro and ${BUILD_DIR}/tico-watermelonds.nro"
