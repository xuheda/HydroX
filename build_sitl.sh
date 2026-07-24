#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root_dir="$(cd -- "${script_dir}/.." && pwd)"
build_dir="${script_dir}/build_sitl_linux"
runtime_dir="${root_dir}/engine/Binaries/ThirdParty/HydroX"
build_target="${1:-hydrox_sitl}"

echo "[INFO] Configuring HydroX (${build_target}, Release)"
cmake \
    -S "${script_dir}" \
    -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release

echo "[INFO] Building HydroX target: ${build_target}"
cmake --build "${build_dir}" --target "${build_target}" --parallel

if [[ "${build_target}" == "hydrox_sitl" ]]; then
    built_runtime="${build_dir}/hydrox_sitl"
    if [[ ! -x "${built_runtime}" ]]; then
        echo "[ERROR] Built HydroX runtime not found: ${built_runtime}" >&2
        exit 1
    fi

    install -D -m 0755 "${built_runtime}" "${runtime_dir}/hydrox_sitl"
    echo "[OK] HydroX runtime: ${runtime_dir}/hydrox_sitl"
else
    echo "[OK] HydroX target built: ${build_target}"
fi
