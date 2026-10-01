#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
version="13.0.0"

temp_dir="$(mktemp -d -t hydrox-nuttx-XXXXXXXX)"
cleanup() {
    case "${temp_dir}" in
        "${TMPDIR:-/tmp}"/hydrox-nuttx-*) rm -rf -- "${temp_dir}" ;;
        *) echo "[WARN] Refusing to remove unexpected temporary path: ${temp_dir}" >&2 ;;
    esac
}
trap cleanup EXIT

fetch_package() {
    local name="$1"
    local url="$2"
    local expected_sha512="$3"
    local archive_root="$4"
    local destination="$5"
    local archive_path="${temp_dir}/${name}.tar.gz"
    local extract_path="${temp_dir}/extract-${name}"

    if [[ -e "${destination}" ]]; then
        echo "[ERROR] Destination already exists; remove it explicitly before retrying: ${destination}" >&2
        return 2
    fi

    mkdir -p -- "${extract_path}"
    echo "[INFO] Downloading ${name} ${version}"
    curl --fail --location --output "${archive_path}" "${url}"
    printf '%s  %s\n' "${expected_sha512}" "${archive_path}" | sha512sum --check --status
    tar -xzf "${archive_path}" -C "${extract_path}"

    local extracted_root="${extract_path}/${archive_root}"
    if [[ ! -d "${extracted_root}" ]]; then
        local roots=()
        while IFS= read -r -d '' root; do
            roots+=("${root}")
        done < <(find "${extract_path}" -mindepth 1 -maxdepth 1 -type d -print0)
        if [[ ${#roots[@]} -ne 1 ]]; then
            echo "[ERROR] Expected archive root was not found and archive root is ambiguous: ${archive_root}" >&2
            return 2
        fi
        extracted_root="${roots[0]}"
        echo "[WARN] Archive root is '$(basename -- "${extracted_root}")', not '${archive_root}'; using the only extracted root." >&2
    fi

    mkdir -p -- "$(dirname -- "${destination}")"
    mv -- "${extracted_root}" "${destination}"
    echo "[OK] ${name} -> ${destination}"
}

fetch_package \
    "nuttx" \
    "https://downloads.apache.org/nuttx/13.0.0/apache-nuttx-13.0.0.tar.gz" \
    "104263f050810455b6c14ad92f045e367ad07718840d5f7464e8dddcc7304dd077f3c8847f4ab21a34ed2c971c4d9b1a51413a8e1f1683d144439e0aa40279f0" \
    "apache-nuttx-13.0.0" \
    "${project_root}/third_party/nuttx"

fetch_package \
    "nuttx-apps" \
    "https://downloads.apache.org/nuttx/13.0.0/apache-nuttx-apps-13.0.0.tar.gz" \
    "d1bcb7cd1e9b769e73bdf7dcacfcc23c5108c9b5a8c9712b2bc5a3123f7bd501aeef782fbdab06f65e0a8290a2a9dfb86c5a97759e93fad471a71fd25ef6366e" \
    "apache-nuttx-apps-13.0.0" \
    "${project_root}/third_party/nuttx-apps"
