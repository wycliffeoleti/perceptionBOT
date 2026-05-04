#!/usr/bin/env bash
# Install ONNX Runtime (GPU build) to /opt/onnxruntime so the perception
# package can find onnxruntime_cxx_api.h + libonnxruntime.so.
#
# Usage:
#   bash scripts/install_onnxruntime.sh [version]
# Default version: 1.20.1
#
# Requires sudo for the extraction step.
set -euo pipefail

VERSION="${1:-1.20.1}"
TARGET="/opt/onnxruntime"
TARBALL_URL="https://github.com/microsoft/onnxruntime/releases/download/v${VERSION}/onnxruntime-linux-x64-gpu-${VERSION}.tgz"
TMPDIR=$(mktemp -d)

trap 'rm -rf "${TMPDIR}"' EXIT

if [[ -f "${TARGET}/lib/libonnxruntime.so" ]]; then
  echo "ONNX Runtime already installed at ${TARGET}; nothing to do."
  exit 0
fi

echo "Downloading ONNX Runtime ${VERSION} (GPU build)..."
curl -fL "${TARBALL_URL}" -o "${TMPDIR}/onnxruntime.tgz"

echo "Extracting to ${TARGET}..."
sudo mkdir -p "${TARGET}"
sudo tar -xzf "${TMPDIR}/onnxruntime.tgz" -C "${TMPDIR}"
sudo cp -r "${TMPDIR}"/onnxruntime-linux-x64-gpu-${VERSION}/* "${TARGET}/"

echo "Verifying..."
test -f "${TARGET}/include/onnxruntime_cxx_api.h"
test -f "${TARGET}/lib/libonnxruntime.so"

cat <<EOF

ONNX Runtime ${VERSION} installed at ${TARGET}.
Add to your shell rc file (or current shell) so the runtime loader can find it:

  export LD_LIBRARY_PATH=${TARGET}/lib:\$LD_LIBRARY_PATH

Then rebuild the workspace:

  colcon build --packages-select perception_bot_perception --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

EOF
