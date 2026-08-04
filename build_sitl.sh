#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${script_dir}/build/sitl"
build_target="${1:-hydrox_sitl}"
runtime_dir="${2:-}"

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

    if [[ -n "${runtime_dir}" ]]; then
        install -D -m 0755 "${built_runtime}" "${runtime_dir}/hydrox_sitl"
        echo "[OK] HydroX runtime deployed: ${runtime_dir}/hydrox_sitl"
    else
        echo "[OK] HydroX runtime: ${built_runtime}"
    fi
else
    echo "[OK] HydroX target built: ${build_target}"
fi
