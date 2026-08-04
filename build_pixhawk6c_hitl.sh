#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${script_dir}/build/pixhawk6c_hitl"

if [[ ! -f "${script_dir}/boards/pixhawk6c/CMakeLists.txt" ]]; then
    echo "[ERROR] The in-tree Pixhawk 6C NuttX board port is not implemented." >&2
    echo "        See boards/pixhawk6c/README.md for the required board files." >&2
    exit 2
fi

for command_name in cmake ninja arm-none-eabi-g++; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "[ERROR] ${command_name} was not found in PATH." >&2
        exit 2
    fi
done

cmake \
    -S "${script_dir}" \
    -B "${build_dir}" \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="${script_dir}/cmake/toolchains/arm_none_eabi_gcc.cmake" \
    -DHYDROX_TARGET=PIXHAWK6C_HITL

cmake --build "${build_dir}" --target hydrox_pixhawk6c_hitl --parallel

firmware="${build_dir}/firmware/hydrox_pixhawk6c_hitl.bin"
if [[ ! -f "${firmware}" ]]; then
    echo "[ERROR] Expected firmware was not generated: ${firmware}" >&2
    exit 1
fi

echo "[OK] HydroX Pixhawk 6C HITL firmware: ${firmware}"
