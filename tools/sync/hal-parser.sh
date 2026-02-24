#!/bin/bash
#
# YAML parser for hal-config.yml
#
# Parses the sync configuration file and populates shell variables.
# Source this file from hal-sync.sh, hal-patches.sh, or hal-blobs.sh.
#
# After sourcing and calling parse_config:
#   ESP_IDF_SHA      - ESP-IDF revision last synced from
#   COMPONENTS[]     - array of component names
#   EXCLUDES[]       - array of exclude paths
#   RENAME_SRC[]     - array of rename source paths
#   RENAME_DST[]     - array of rename destination paths
#   BLOB_FILTER_*    - space-separated lib names per SoC (e.g. BLOB_FILTER_esp32)
#   BLOB_FILTER_SOCS - space-separated list of SoC names that have filters

_CONFIG_PARSE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${_CONFIG_PARSE_DIR}/hal-config.yml"

ESP_IDF_SHA=""
COMPONENTS=()
EXCLUDES=()
RENAME_SRC=()
RENAME_DST=()
BLOB_FILTER_SOCS=""

parse_config() {
    if [ ! -f "${CONFIG_FILE}" ]; then
        echo "[error] Config not found: ${CONFIG_FILE}" >&2
        return 1
    fi

    local section=""
    local subsection=""
    local rename_src=""

    while IFS= read -r line || [ -n "${line}" ]; do
        # Skip comments and empty lines
        [[ "${line}" =~ ^[[:space:]]*# ]] && continue
        [[ "${line}" =~ ^[[:space:]]*$ ]] && continue

        # Top-level scalar (key: value on same line)
        if [[ "${line}" =~ ^([a-z_]+):[[:space:]]+(.+)$ ]]; then
            case "${BASH_REMATCH[1]}" in
                esp_idf_sha) ESP_IDF_SHA="${BASH_REMATCH[2]}" ;;
            esac
            continue
        fi

        # Top-level section headers (key: with no value)
        if [[ "${line}" =~ ^([a-z_]+):$ ]]; then
            section="${BASH_REMATCH[1]}"
            subsection=""
            continue
        fi

        # Nested key (blob_filters SoC name)
        if [[ "${line}" =~ ^[[:space:]]{2}([a-z0-9_]+):$ ]]; then
            if [ "${section}" = "blob_filters" ]; then
                subsection="${BASH_REMATCH[1]}"
                BLOB_FILTER_SOCS="${BLOB_FILTER_SOCS} ${subsection}"
            fi
            continue
        fi

        # Rename entries (src/dst keys) — must be checked before generic
        # depth-2 list items since "  - src: ..." also matches that pattern
        if [[ "${line}" =~ ^[[:space:]]+-[[:space:]]+src:[[:space:]]+(.+)$ ]]; then
            rename_src="${BASH_REMATCH[1]}"
            continue
        fi
        if [[ "${line}" =~ ^[[:space:]]+dst:[[:space:]]+(.+)$ ]]; then
            if [ -n "${rename_src}" ]; then
                RENAME_SRC+=("${rename_src}")
                RENAME_DST+=("${BASH_REMATCH[1]}")
                rename_src=""
            fi
            continue
        fi

        # List items at depth 4 (    - value) for blob_filters
        if [[ "${line}" =~ ^[[:space:]]{4}-[[:space:]]+(.+)$ ]]; then
            local value="${BASH_REMATCH[1]}"
            if [ "${section}" = "blob_filters" ] && [ -n "${subsection}" ]; then
                eval "BLOB_FILTER_${subsection}=\"\${BLOB_FILTER_${subsection}} ${value}\""
            fi
            continue
        fi

        # List items at depth 2 (  - value)
        if [[ "${line}" =~ ^[[:space:]]{2}-[[:space:]]+(.+)$ ]]; then
            local value="${BASH_REMATCH[1]}"
            case "${section}" in
                components) COMPONENTS+=("${value}") ;;
                exclude) EXCLUDES+=("${value}") ;;
            esac
            continue
        fi
    done < "${CONFIG_FILE}"
}

get_blob_filters() {
    local soc="$1"
    eval "echo \${BLOB_FILTER_${soc}}"
}

update_esp_idf_sha() {
    local new_sha="$1"
    sed -i "s/^esp_idf_sha: .*/esp_idf_sha: ${new_sha}/" "${CONFIG_FILE}"
    ESP_IDF_SHA="${new_sha}"
}
