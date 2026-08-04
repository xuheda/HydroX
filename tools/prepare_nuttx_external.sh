#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "${1:?repository root is required}" && pwd)"
apps_dir="${repo_root}/third_party/nuttx-apps"
source_dir="${repo_root}/nuttx_apps/external"
link_path="${apps_dir}/external"

[[ -f "${apps_dir}/CMakeLists.txt" ]] || {
  echo "[ERROR] Pinned NuttX apps tree is missing: ${apps_dir}" >&2
  exit 2
}
[[ -f "${source_dir}/CMakeLists.txt" ]] || {
  echo "[ERROR] HydroX external NuttX applications are missing: ${source_dir}" >&2
  exit 2
}

if [[ -L "${link_path}" ]]; then
  [[ "$(cd -- "$(dirname -- "$(readlink -f -- "${link_path}")")" && pwd)/$(basename -- "$(readlink -f -- "${link_path}")")" == "${source_dir}" ]] || {
    echo "[ERROR] Existing NuttX external link points elsewhere: ${link_path}" >&2
    exit 2
  }
  exit 0
fi
if [[ -e "${link_path}" ]]; then
  echo "[ERROR] Refusing to replace existing non-link path: ${link_path}" >&2
  exit 2
fi

ln -s ../../nuttx_apps/external "${link_path}"
echo "[OK] Linked NuttX apps external -> ${source_dir}"