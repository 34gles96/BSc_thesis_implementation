#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_FILE="${SCRIPT_DIR}/generate_stereo_yaml.cpp"
BIN_FILE="${SCRIPT_DIR}/generate_stereo_yaml"

echo "[1/2] Compiling ${SRC_FILE}..."
if pkg-config --exists opencv4; then
    g++ -std=c++17 -O2 "${SRC_FILE}" -o "${BIN_FILE}" $(pkg-config --cflags --libs opencv4)
else
    g++ -std=c++17 -O2 "${SRC_FILE}" -o "${BIN_FILE}" -I/usr/include/opencv4 -lopencv_core
fi

echo "[2/2] Running ${BIN_FILE}..."
"${BIN_FILE}" "$@"