#!/bin/bash
#
# Update blob library references and regenerate module.yml
#
# Downloads blob files via HTTP and computes SHA256 hashes.
# Reads submodule SHAs from ESP-IDF to determine blob revisions.
#
# Usage:
#   ./hal-blobs.sh              # Update blobs
#   ./hal-blobs.sh --dry-run    # Show changes without writing

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAL_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ESPIDF_DIR="${SCRIPT_DIR}/.work/esp-idf"
MODULE_FILE="${HAL_ROOT}/zephyr/module.yml"
BLOB_SHAS_FILE="${SCRIPT_DIR}/.work/blob_shas"

source "${SCRIPT_DIR}/hal-parser.sh"
parse_config

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

die() {
    echo -e "${RED}[error]${NC} $1" >&2
    exit 1
}

# repo_path|github_url|socs|category|has_soc_prefix
BLOB_REPOS=(
    "components/bt/controller/lib_esp32c5/esp32c5-bt-lib|https://github.com/espressif/esp32c5-bt-lib|esp32c5|bt|no"
    "components/bt/controller/lib_esp32c6/esp32c6-bt-lib|https://github.com/espressif/esp32c6-bt-lib|esp32c6|bt|yes"
    "components/bt/controller/lib_esp32c2/esp32c2-bt-lib|https://github.com/espressif/esp32c2-bt-lib|esp32c2|bt|no"
    "components/bt/controller/lib_esp32h2/esp32h2-bt-lib|https://github.com/espressif/esp32h2-bt-lib|esp32h2|bt|no"
    "components/bt/controller/lib_esp32c3_family|https://github.com/espressif/esp32c3-bt-lib|esp32c3,esp32s3|bt|yes"
    "components/bt/controller/lib_esp32|https://github.com/espressif/esp32-bt-lib|esp32|bt|yes"
    "components/esp_wifi/lib|https://github.com/espressif/esp32-wifi-lib|esp32,esp32c2,esp32c3,esp32c5,esp32c6,esp32s2,esp32s3|wifi|yes"
    "components/esp_phy/lib|https://github.com/espressif/esp-phy-lib|esp32,esp32c2,esp32c3,esp32c5,esp32c6,esp32h2,esp32s2,esp32s3|phy|yes"
    "components/esp_coex/lib|https://github.com/espressif/esp-coex-lib|esp32,esp32c2,esp32c3,esp32c5,esp32c6,esp32h2,esp32s2,esp32s3|coex|yes"
)

lib_category() {
    case "$1" in
        ble_app|btdm_app) echo "bt" ;;
        btbb|phy|rtc)     echo "phy" ;;
        core|mesh|net80211|pp) echo "wifi" ;;
        coexist)          echo "coex" ;;
        *)                echo "" ;;
    esac
}

get_filters_for_soc() {
    get_blob_filters "$1"
}

get_submodule_sha() {
    local espidf_path="$1"
    local line
    line=$(git -C "${ESPIDF_DIR}" submodule status -- "${espidf_path}" 2>/dev/null | head -1)
    if [ -z "${line}" ]; then
        return 1
    fi
    echo "${line}" | awk '{print $1}' | sed 's/^[-+]//'
}

download_sha256() {
    local url="$1"
    local tmpfile
    tmpfile=$(mktemp)

    if curl -sSfL -o "${tmpfile}" -H "User-Agent: hal-espressif-blob-updater" "${url}" 2>/dev/null; then
        sha256sum "${tmpfile}" | awk '{print $1}'
        rm -f "${tmpfile}"
        return 0
    else
        rm -f "${tmpfile}"
        return 1
    fi
}

main() {
    local dry_run=false

    while [ $# -gt 0 ]; do
        case "$1" in
            --dry-run) dry_run=true; shift ;;
            --help|-h)
                echo "Usage: $0 [--dry-run]"
                echo ""
                echo "Update blob library references and regenerate module.yml."
                echo "Requires ESP-IDF clone at .work/esp-idf/ (run hal-sync.sh first)."
                exit 0
                ;;
            *) die "Unknown option: $1" ;;
        esac
    done

    if [ ! -d "${ESPIDF_DIR}" ]; then
        die "ESP-IDF not found at ${ESPIDF_DIR}. Run hal-sync.sh first."
    fi

    echo "============================================================"
    echo "Blob Update Tool"
    echo "============================================================"

    echo ""
    echo "[Step 1] Reading submodule SHAs from ESP-IDF..."

    local blob_count=0
    local error_count=0
    local module_blobs=""
    local blob_sha_lines=""

    echo ""
    echo "[Step 2] Downloading blobs and computing SHA256..."

    for entry in "${BLOB_REPOS[@]}"; do
        IFS='|' read -r repo_path github_url socs_csv category has_soc_prefix <<< "${entry}"

        local sha
        sha=$(get_submodule_sha "${repo_path}") || {
            echo -e "  ${YELLOW}[warning]${NC} Not found: ${repo_path}"
            continue
        }
        echo "  ${repo_path}: ${sha:0:12}"

        local repo_name
        repo_name=$(basename "${github_url}")
        blob_sha_lines="${blob_sha_lines}${repo_name}: ${sha}
"

        IFS=',' read -ra socs <<< "${socs_csv}"
        for soc in "${socs[@]}"; do
            local libs
            libs=$(get_filters_for_soc "${soc}")

            for lib_name in ${libs}; do
                local lib_cat
                lib_cat=$(lib_category "${lib_name}")
                [ "${lib_cat}" = "${category}" ] || continue

                local filename="lib${lib_name}.a"
                local url_path
                if [ "${has_soc_prefix}" = "yes" ]; then
                    url_path="${soc}/${filename}"
                else
                    url_path="${filename}"
                fi
                local raw_url="${github_url}/raw/${sha}/${url_path}"

                printf "  %s/%s... " "${soc}" "${filename}"

                local file_sha256
                if file_sha256=$(download_sha256 "${raw_url}"); then
                    echo "${file_sha256:0:12}"
                    blob_count=$((blob_count + 1))
                    module_blobs="${module_blobs}
  - path: lib/${soc}/${filename}
    sha256: ${file_sha256}
    type: lib
    version: '1.0'
    license-path: zephyr/blobs/license.txt
    url: ${github_url}/raw/${sha}/${url_path}
    description: \"Binary libraries supporting the ESP32 series RF subsystems\"
    doc-url: ${github_url}"
                else
                    echo -e "${RED}[error] HTTP download failed${NC}"
                    error_count=$((error_count + 1))
                fi
            done
        done
    done

    if [ "${error_count}" -gt 0 ]; then
        echo ""
        echo -e "${YELLOW}[warning]${NC} ${error_count} file(s) failed to download"
    fi

    if [ "${dry_run}" = false ] && [ -n "${blob_sha_lines}" ]; then
        printf "%s" "${blob_sha_lines}" | sort -u > "${BLOB_SHAS_FILE}"
    fi

    local module_yaml="name: hal_espressif
package-managers:
  pip:
    requirement-files:
      - zephyr/requirements.txt
build:
  cmake: zephyr
  kconfig: zephyr/Kconfig
  settings:
    dts_root: .
blobs:${module_blobs}
"

    if [ "${dry_run}" = true ]; then
        echo ""
        echo "[dry-run] Would write module.yml with ${blob_count} blobs"
    else
        echo "${module_yaml}" > "${MODULE_FILE}"
        echo ""
        echo "[info] Generated ${MODULE_FILE} with ${blob_count} blobs"
    fi

    echo ""
    echo "============================================================"
    echo "Done!"
    echo "============================================================"

    [ "${error_count}" -eq 0 ] || exit 1
}

main "$@"
