#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${script_dir}/build/fmu_v6c_hitl"
nuttx="${script_dir}/third_party/nuttx"
nuttx_apps="${script_dir}/third_party/nuttx-apps"
external_link="${nuttx_apps}/external"

for command_name in cmake ninja arm-none-eabi-g++ olddefconfig python3; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "[ERROR] ${command_name} was not found in PATH." >&2
        exit 2
    fi
done

if [[ ! -f "${nuttx}/CMakeLists.txt" || ! -f "${nuttx_apps}/CMakeLists.txt" ]]; then
    echo "[ERROR] Pinned NuttX trees are missing; run tools/fetch_nuttx.sh." >&2
    exit 2
fi

if [[ -e "${external_link}" && ! -L "${external_link}" ]]; then
    echo "[ERROR] Refusing to replace non-link NuttX apps/external." >&2
    exit 2
fi
if [[ ! -e "${external_link}" ]]; then
    ln -s "${script_dir}/nuttx_apps/external" "${external_link}"
fi

cmake \
    -S "${nuttx}" \
    -B "${build_dir}" \
    -G Ninja \
    -DBOARD_CONFIG="${script_dir}/boards/fmu_v6c/configs/hydrox_hitl" \
    -DNUTTX_APPS_DIR="${nuttx_apps}" \
    -DHYDROX_SOURCE_DIR="${script_dir}" \
    -DPython3_EXECUTABLE="$(command -v python3)"

cmake --build "${build_dir}" --target hydrox_pixhawk6cmini_hitl --parallel

firmware="${build_dir}/firmware/hydrox_pixhawk6cmini_hitl.bin"
if [[ ! -f "${firmware}" ]]; then
    echo "[ERROR] Expected firmware was not generated: ${firmware}" >&2
    exit 1
fi

python3 "${script_dir}/tools/verify_fmuv6c_firmware.py" \
    --repository-root "${script_dir}" \
    --build-dir "${build_dir}" \
    --firmware-dir "${build_dir}/firmware" \
    --toolchain-bin "$(dirname "$(command -v arm-none-eabi-gcc)")" \
    --manifest "${build_dir}/firmware/hydrox_pixhawk6cmini_hitl.manifest.json"

echo "[OK] HydroX FMUv6C HITL flash candidate: ${firmware}"
