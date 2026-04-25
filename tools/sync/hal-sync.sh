#!/bin/bash
#
# Sync hal_espressif components/ from ESP-IDF master
#
# Usage:
#   ./hal-sync.sh                  # Full sync: fetch, copy, patch, update blobs
#   ./hal-sync.sh --rev v5.4       # Sync from specific ESP-IDF tag/branch/commit
#   ./hal-sync.sh --dry-run        # Preview what would change without modifying anything
#   ./hal-sync.sh --commit         # Commit: pristine sync, patches, blobs (up to 3 commits)
#   ./hal-sync.sh --continue       # Continue after manual patch fix
#   ./hal-sync.sh --patches-only   # Only apply patches (skip ESP-IDF fetch/copy)
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAL_ESPRESSIF_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PATCHES_DIR="${SCRIPT_DIR}/patches"

source "${SCRIPT_DIR}/hal-parser.sh"
parse_config
ESP_IDF_URL="https://github.com/espressif/esp-idf.git"
ESP_IDF_BRANCH="master"
ESP_IDF_REV=""
WORK_DIR="${SCRIPT_DIR}/.work"
STATE_FILE="${WORK_DIR}/sync_state"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_notice() {
    echo -e "${CYAN}[NOTICE]${NC} $1"
}

die() {
    log_error "$1"
    exit 1
}

read_components() {
    echo "${COMPONENTS[*]}"
}

fetch_espidf() {
    local ref="${ESP_IDF_REV:-${ESP_IDF_BRANCH}}"
    log_info "Fetching ESP-IDF ${ref}..."

    mkdir -p "${WORK_DIR}"

    if [ -d "${WORK_DIR}/esp-idf" ]; then
        log_info "Updating existing ESP-IDF clone..."
        cd "${WORK_DIR}/esp-idf"
        if [ -n "${ESP_IDF_REV}" ]; then
            git fetch --unshallow origin 2>/dev/null || git fetch origin
            git checkout "${ESP_IDF_REV}"
        else
            git fetch --depth 1 origin "${ESP_IDF_BRANCH}"
            git reset --hard FETCH_HEAD
        fi
    else
        if [ -n "${ESP_IDF_REV}" ]; then
            log_info "Cloning ESP-IDF..."
            git clone --single-branch --branch "${ESP_IDF_BRANCH}" \
                "${ESP_IDF_URL}" "${WORK_DIR}/esp-idf"
            cd "${WORK_DIR}/esp-idf"
            git checkout "${ESP_IDF_REV}"
        else
            log_info "Cloning ESP-IDF (shallow)..."
            git clone --depth 1 --single-branch --branch "${ESP_IDF_BRANCH}" \
                "${ESP_IDF_URL}" "${WORK_DIR}/esp-idf"
            cd "${WORK_DIR}/esp-idf"
        fi
    fi

    ESP_IDF_SHA=$(git rev-parse --short HEAD)
    ESP_IDF_SHA_FULL=$(git rev-parse HEAD)
    log_info "ESP-IDF is at: ${ESP_IDF_SHA_FULL}"

    echo "${ESP_IDF_SHA_FULL}" > "${STATE_FILE}"
    update_esp_idf_sha "${ESP_IDF_SHA_FULL}"
}

check_new_hal_components() {
    log_info "Checking for new esp_hal_* components in ESP-IDF..."

    local esp_idf_components="${WORK_DIR}/esp-idf/components"
    local our_components=$(read_components)
    local new_components=""

    # Find all esp_hal_* directories in ESP-IDF
    for hal_dir in "${esp_idf_components}"/esp_hal_*; do
        if [ -d "${hal_dir}" ]; then
            local hal_name=$(basename "${hal_dir}")
            # Check if we have it in our list
            if ! echo "${our_components}" | grep -q "${hal_name}"; then
                new_components="${new_components}  - ${hal_name}\n"
            fi
        fi
    done

    if [ -n "${new_components}" ]; then
        echo ""
        log_notice "New esp_hal_* components found in ESP-IDF that are NOT in hal-config.yml:"
        echo -e "${new_components}"
        log_notice "Consider adding them to tools/sync/hal-config.yml if needed."
        echo ""
    fi
}

copy_components() {
    log_info "Copying components to hal_espressif..."

    cd "${HAL_ESPRESSIF_DIR}"

    # Remove old components
    if [ -d "components" ]; then
        rm -rf components
    fi

    mkdir -p components

    local components=$(read_components)
    local esp_idf_components="${WORK_DIR}/esp-idf/components"

    for comp in ${components}; do
        if [ -d "${esp_idf_components}/${comp}" ]; then
            log_info "  Copying ${comp}..."
            cp -r "${esp_idf_components}/${comp}" "components/"
        else
            log_warn "  Component ${comp} not found in ESP-IDF, skipping"
        fi
    done

    # Copy LICENSE file
    if [ -f "${WORK_DIR}/esp-idf/LICENSE" ]; then
        cp "${WORK_DIR}/esp-idf/LICENSE" "components/"
    fi

    # Cleanup: remove test_apps folders and .a files
    cleanup_components

    # Note: renames are applied AFTER patches, see main()

    # Check for new HAL components
    check_new_hal_components
}

cleanup_components() {
    log_info "Cleaning up unnecessary files..."

    cd "${HAL_ESPRESSIF_DIR}"

    # Remove all test_apps folders
    local test_apps_count=$(find components -type d -name "test_apps" 2>/dev/null | wc -l)
    if [ "${test_apps_count}" -gt 0 ]; then
        find components -type d -name "test_apps" -exec rm -rf {} + 2>/dev/null || true
        log_warn "Removed ${test_apps_count} test_apps folder(s)"
    fi

    # Remove all .a (static library) files
    local lib_count=$(find components -type f -name "*.a" 2>/dev/null | wc -l)
    if [ "${lib_count}" -gt 0 ]; then
        find components -type f -name "*.a" -delete 2>/dev/null || true
        log_warn "Removed ${lib_count} .a file(s)"
    fi

    # Remove all .bin files (unused on Zephyr; no partition/embed path wired up)
    local bin_count=$(find components -type f -name "*.bin" 2>/dev/null | wc -l)
    if [ "${bin_count}" -gt 0 ]; then
        find components -type f -name "*.bin" -delete 2>/dev/null || true
        log_warn "Removed ${bin_count} .bin file(s)"
    fi

    # Remove excluded folders from config
    local excluded_count=0
    for exclude_path in "${EXCLUDES[@]}"; do
        local full_path="components/${exclude_path}"
        if [ -d "${full_path}" ]; then
            rm -rf "${full_path}"
            excluded_count=$((excluded_count + 1))
            log_info "  Excluded: ${exclude_path}"
        fi
    done

    if [ "${excluded_count}" -gt 0 ]; then
        log_warn "Removed ${excluded_count} excluded folder(s)"
    fi
}

apply_renames() {
    if [ "${#RENAME_SRC[@]}" -eq 0 ]; then
        return 0
    fi

    log_info "Applying file/folder renames..."

    cd "${HAL_ESPRESSIF_DIR}"

    local renamed_count=0
    for (( i=0; i<${#RENAME_SRC[@]}; i++ )); do
        local src_path="${RENAME_SRC[$i]}"
        local dst_path="${RENAME_DST[$i]}"
        local full_src="components/${src_path}"
        local full_dst="components/${dst_path}"

        if [ -e "${full_src}" ]; then
            mkdir -p "$(dirname "${full_dst}")"
            if [ -e "${full_dst}" ]; then
                rm -rf "${full_dst}"
            fi
            mv "${full_src}" "${full_dst}"
            log_info "  Renamed: ${src_path} -> ${dst_path}"
            renamed_count=$((renamed_count + 1))
        else
            log_warn "  Source not found: ${src_path}"
        fi
    done

    if [ "${renamed_count}" -gt 0 ]; then
        log_info "Applied ${renamed_count} rename(s)"
    fi
}

commit_renames() {
    if [ "${#RENAME_SRC[@]}" -eq 0 ]; then
        return 0
    fi

    cd "${HAL_ESPRESSIF_DIR}"

    if git diff --quiet && git diff --cached --quiet; then
        log_info "No renames to stage"
        return 0
    fi

    git add -A components/
    log_info "Renames staged"
}

commit_pristine_sync() {
    cd "${HAL_ESPRESSIF_DIR}"

    git add components/
    log_info "Pristine ESP-IDF sync staged"
    log_info "ESP-IDF commit: $(cat "${STATE_FILE}")"

    local tree_hash
    tree_hash=$(git write-tree)
    echo "${tree_hash}" > "${WORK_DIR}/pristine_tree"
    log_info "Pristine tree saved: ${tree_hash}"
}

apply_patches() {
    log_info "Applying Zephyr adaptation patches..."

    cd "${HAL_ESPRESSIF_DIR}"

    if [ ! -d "${PATCHES_DIR}" ] || [ -z "$(ls -A ${PATCHES_DIR}/*.patch 2>/dev/null)" ]; then
        log_warn "No patches found in ${PATCHES_DIR}"
        return 0
    fi

    local failed_patches=""
    local applied_patches=""
    local skipped_patches=""
    local failed_count=0

    for patch in "${PATCHES_DIR}"/*.patch; do
        local patch_name=$(basename "${patch}")

        # Check if patch applies cleanly
        if git apply --check "${patch}" 2>/dev/null; then
            log_info "  ${GREEN}✓${NC} ${patch_name}"
            git apply "${patch}"
            applied_patches="${applied_patches}\n  - ${patch_name}"
        # Check if patch is already applied (reverse applies cleanly)
        elif git apply --check --reverse "${patch}" 2>/dev/null; then
            log_info "  ${YELLOW}≡${NC} ${patch_name} (already applied, skipping)"
            skipped_patches="${skipped_patches}\n  - ${patch_name}"
        else
            # Apply with --reject to generate .rej files and continue
            log_error "  ${RED}✗${NC} ${patch_name} FAILED (generating .rej files)"
            git apply --reject "${patch}" 2>/dev/null || true
            failed_patches="${failed_patches}\n  - ${patch_name}"
            failed_count=$((failed_count + 1))
        fi
    done

    if [ -n "${applied_patches}" ]; then
        git add -A components/
        log_info "Zephyr adaptations staged"
    fi

    if [ "${failed_count}" -gt 0 ]; then
        echo ""
        log_error "${failed_count} patch(es) failed to apply cleanly:${failed_patches}"
        echo ""
        echo "Rejected hunks saved as .rej files. To find them:"
        echo "  find components -name '*.rej'"
        echo ""
        echo "To fix:"
        echo "  1. Manually resolve conflicts in .c files based on .rej files"
        echo "  2. Delete .rej and .orig files: find components -name '*.rej' -o -name '*.orig' | xargs rm -f"
        echo "  3. Run: ./tools/sync/hal-patches.sh"
        echo "  4. Run: $0 --continue"
        exit 1
    fi
}

update_blobs() {
    log_info "Updating blob references..."

    cd "${HAL_ESPRESSIF_DIR}"

    "${SCRIPT_DIR}/hal-blobs.sh"

    if ! git diff --quiet -- zephyr/module.yml 2>/dev/null; then
        git add zephyr/module.yml
        log_info "Blob updates staged"
    else
        log_info "No blob changes"
    fi
}

report_git_changes() {
    local base_ref="$1"

    cd "${HAL_ESPRESSIF_DIR}"

    echo ""
    echo "=============================================="
    echo "  GIT CHANGES: ${base_ref}..HEAD"
    echo "=============================================="
    echo ""

    local renamed=$(git diff --diff-filter=R -M --name-status "${base_ref}"..HEAD -- components/)
    local added=$(git diff --diff-filter=A --name-only "${base_ref}"..HEAD -- components/)
    local deleted=$(git diff --diff-filter=D --name-only "${base_ref}"..HEAD -- components/)
    local modified=$(git diff --diff-filter=M --name-only "${base_ref}"..HEAD -- components/)

    if [ -n "${renamed}" ]; then
        local rename_count=$(echo "${renamed}" | wc -l)
        echo -e "RENAMED/MOVED (${rename_count}):"
        echo "${renamed}" | while IFS=$'\t' read -r status old_path new_path; do
            local similarity=$(echo "${status}" | sed 's/R//')
            echo "    ${old_path} -> ${new_path} (${similarity}% similar)"
        done
        echo ""
    fi

    if [ -n "${added}" ]; then
        local add_count=$(echo "${added}" | wc -l)
        echo -e "ADDED (${add_count}):"
        echo "${added}" | sed 's/^/    /'
        echo ""
    fi

    if [ -n "${deleted}" ]; then
        local del_count=$(echo "${deleted}" | wc -l)
        echo -e "DELETED (${del_count}):"
        echo "${deleted}" | sed 's/^/    /'
        echo ""
    fi

    if [ -n "${modified}" ]; then
        local mod_count=$(echo "${modified}" | wc -l)
        echo "MODIFIED (${mod_count})"
        echo ""
    fi

    if [ -z "${renamed}" ] && [ -z "${added}" ] && [ -z "${deleted}" ] && [ -z "${modified}" ]; then
        echo "  No changes in components/ since ${base_ref}"
        echo ""
    fi

    echo "=============================================="
    echo ""
}

generate_report() {
    local base_ref="${1:-}"

    log_info "Generating sync report..."

    cd "${HAL_ESPRESSIF_DIR}"

    if [ -n "${base_ref}" ]; then
        git rev-parse "${base_ref}" >/dev/null 2>&1 || die "Invalid git ref: ${base_ref}"
        report_git_changes "${base_ref}"
        return 0
    fi

    if [ ! -d "${WORK_DIR}/esp-idf" ]; then
        die "ESP-IDF not found. Run full sync first or use --dry-run."
    fi

    echo ""
    echo "=============================================="
    echo "  ESP-IDF SYNC REPORT"
    echo "=============================================="
    echo ""

    # Get ESP-IDF commit info
    local esp_idf_sha=$(cd "${WORK_DIR}/esp-idf" && git rev-parse HEAD)
    local esp_idf_date=$(cd "${WORK_DIR}/esp-idf" && git log -1 --format=%ci)
    echo "ESP-IDF Reference:"
    echo "  Commit: ${esp_idf_sha}"
    echo "  Date:   ${esp_idf_date}"
    echo "  Branch: ${ESP_IDF_BRANCH}"
    echo ""

    # Compare file structures
    local tmp_dir=$(mktemp -d)
    local components=$(read_components)

    # Build ESP-IDF file list (only components we sync, excluding test_apps)
    > "${tmp_dir}/espidf_files_unsorted.txt"
    for comp in ${components}; do
        if [ -d "${WORK_DIR}/esp-idf/components/${comp}" ]; then
            find "${WORK_DIR}/esp-idf/components/${comp}" -type f \( -name "*.c" -o -name "*.h" \) 2>/dev/null | \
                grep -v "/test_apps/" | \
                sed "s|${WORK_DIR}/esp-idf/||" >> "${tmp_dir}/espidf_files_unsorted.txt"
        fi
    done

    # Filter out excluded paths
    if [ "${#EXCLUDES[@]}" -gt 0 ]; then
        cp "${tmp_dir}/espidf_files_unsorted.txt" "${tmp_dir}/espidf_files_filtered.txt"
        for exclude_path in "${EXCLUDES[@]}"; do
            grep -v "components/${exclude_path}" "${tmp_dir}/espidf_files_filtered.txt" > "${tmp_dir}/espidf_files_tmp.txt" 2>/dev/null || true
            mv "${tmp_dir}/espidf_files_tmp.txt" "${tmp_dir}/espidf_files_filtered.txt"
        done
        mv "${tmp_dir}/espidf_files_filtered.txt" "${tmp_dir}/espidf_files_unsorted.txt"
    fi

    # Apply renames to ESP-IDF file list so paths match hal_espressif
    for (( i=0; i<${#RENAME_SRC[@]}; i++ )); do
        sed -i "s|components/${RENAME_SRC[$i]}|components/${RENAME_DST[$i]}|g" \
            "${tmp_dir}/espidf_files_unsorted.txt"
    done

    sort "${tmp_dir}/espidf_files_unsorted.txt" > "${tmp_dir}/espidf_files.txt"

    # Build current hal_espressif file list (only components we sync)
    > "${tmp_dir}/hal_files_unsorted.txt"
    for comp in ${components}; do
        if [ -d "components/${comp}" ]; then
            find "components/${comp}" -type f \( -name "*.c" -o -name "*.h" \) 2>/dev/null | \
                grep -v "/test_apps/" >> "${tmp_dir}/hal_files_unsorted.txt"
        fi
    done

    # Filter out excluded paths from hal_espressif list as well
    if [ "${#EXCLUDES[@]}" -gt 0 ]; then
        cp "${tmp_dir}/hal_files_unsorted.txt" "${tmp_dir}/hal_files_filtered.txt"
        for exclude_path in "${EXCLUDES[@]}"; do
            grep -v "components/${exclude_path}" "${tmp_dir}/hal_files_filtered.txt" > "${tmp_dir}/hal_files_tmp.txt" 2>/dev/null || true
            mv "${tmp_dir}/hal_files_tmp.txt" "${tmp_dir}/hal_files_filtered.txt"
        done
        mv "${tmp_dir}/hal_files_filtered.txt" "${tmp_dir}/hal_files_unsorted.txt"
    fi

    sort "${tmp_dir}/hal_files_unsorted.txt" > "${tmp_dir}/hal_files.txt"

    # Find new files (in ESP-IDF but not in hal)
    echo "NEW FILES (in ESP-IDF, not in hal_espressif):"
    comm -23 "${tmp_dir}/espidf_files.txt" "${tmp_dir}/hal_files.txt" > "${tmp_dir}/new_files.txt"
    local new_count=$(wc -l < "${tmp_dir}/new_files.txt")
    if [ "${new_count}" -gt 0 ]; then
        echo "  Count: ${new_count}"
        sed 's/^/    /' "${tmp_dir}/new_files.txt"
    else
        echo "  None"
    fi
    echo ""

    # Find deleted files (in hal but not in ESP-IDF)
    echo "DELETED FILES (in hal_espressif, not in ESP-IDF):"
    comm -13 "${tmp_dir}/espidf_files.txt" "${tmp_dir}/hal_files.txt" > "${tmp_dir}/deleted_files.txt"
    local deleted_count=$(wc -l < "${tmp_dir}/deleted_files.txt")
    if [ "${deleted_count}" -gt 0 ]; then
        echo "  Count: ${deleted_count}"
        sed 's/^/    /' "${tmp_dir}/deleted_files.txt"
    else
        echo "  None"
    fi
    echo ""

    # Find potentially renamed files (same basename, different path)
    echo "POTENTIALLY RENAMED/MOVED FILES:"
    local renamed_count=0
    while IFS= read -r new_file; do
        local basename=$(basename "${new_file}")
        local old_match=$(grep "/${basename}$" "${tmp_dir}/deleted_files.txt" 2>/dev/null | head -1)
        if [ -n "${old_match}" ]; then
            echo "    ${old_match} -> ${new_file}"
            renamed_count=$((renamed_count + 1))
            if [ "${renamed_count}" -ge 20 ]; then
                echo "    ... (showing first 20)"
                break
            fi
        fi
    done < "${tmp_dir}/new_files.txt"
    if [ "${renamed_count}" -eq 0 ]; then
        echo "  None detected"
    fi
    echo ""

    # Cleanup
    rm -rf "${tmp_dir}"

    echo "=============================================="
    echo ""
}

dry_run_cleanup_preview() {
    local esp_idf_components="${WORK_DIR}/esp-idf/components"
    local components=$(read_components)
    local found=""
    local found_count=0

    for comp in ${components}; do
        local comp_dir="${esp_idf_components}/${comp}"
        [ -d "${comp_dir}" ] || continue

        # Check for test_apps
        local test_dirs=$(find "${comp_dir}" -type d -name "test_apps" 2>/dev/null)
        for d in ${test_dirs}; do
            local rel="${d#${esp_idf_components}/}"
            found="${found}  ${YELLOW}${rel}${NC}\n"
            found_count=$((found_count + 1))
        done

        # Check for linux folders
        local linux_dirs=$(find "${comp_dir}" -type d -name "linux" 2>/dev/null)
        for d in ${linux_dirs}; do
            local rel="${d#${esp_idf_components}/}"
            found="${found}  ${YELLOW}${rel}${NC}\n"
            found_count=$((found_count + 1))
        done

        # Check for .a files
        local lib_files=$(find "${comp_dir}" -type f -name "*.a" 2>/dev/null)
        for f in ${lib_files}; do
            local rel="${f#${esp_idf_components}/}"
            found="${found}  ${YELLOW}${rel}${NC}\n"
            found_count=$((found_count + 1))
        done

        # Check for .bin files
        local bin_files=$(find "${comp_dir}" -type f -name "*.bin" 2>/dev/null)
        for f in ${bin_files}; do
            local rel="${f#${esp_idf_components}/}"
            found="${found}  ${YELLOW}${rel}${NC}\n"
            found_count=$((found_count + 1))
        done
    done

    # Check excluded paths from config
    local exclude_found=""
    local exclude_count=0
    for exclude_path in "${EXCLUDES[@]}"; do
        if [ -d "${esp_idf_components}/${exclude_path}" ]; then
            exclude_count=$((exclude_count + 1))
        else
            exclude_found="${exclude_found}  ${RED}${exclude_path}${NC} (not found in ESP-IDF)\n"
        fi
    done

    if [ "${found_count}" -gt 0 ] || [ -n "${exclude_found}" ]; then
        echo "CLEANUP PREVIEW:"
        if [ "${found_count}" -gt 0 ]; then
            echo "  ${found_count} path(s) will be removed during cleanup:"
            echo -e "${found}"
        fi
        if [ -n "${exclude_found}" ]; then
            echo "  Stale entries in hal-config.yml exclude:"
            echo -e "${exclude_found}"
        fi
    fi
}

dry_run_sync() {
    log_info "DRY RUN: Simulating sync without making changes..."

    # Fetch ESP-IDF if not present
    if [ ! -d "${WORK_DIR}/esp-idf" ]; then
        fetch_espidf
    else
        log_info "Using existing ESP-IDF at ${WORK_DIR}/esp-idf"
        cd "${WORK_DIR}/esp-idf"
        git fetch origin "${ESP_IDF_BRANCH}"
        log_info "ESP-IDF HEAD: $(git rev-parse HEAD)"
    fi

    # Show component-level changes based on hal-config.yml
    dry_run_component_summary

    # Preview cleanup: scan ESP-IDF components for paths that would be removed
    dry_run_cleanup_preview

    # Generate report
    generate_report

    # Check patches against a temp copy of ESP-IDF components
    echo "PATCH STATUS:"
    cd "${HAL_ESPRESSIF_DIR}"
    if [ -d "${PATCHES_DIR}" ] && [ -n "$(ls -A ${PATCHES_DIR}/*.patch 2>/dev/null)" ]; then
        local patch_tmp=$(mktemp -d)
        mkdir -p "${patch_tmp}/components"

        # Copy ESP-IDF components to temp dir
        local components=$(read_components)
        for comp in ${components}; do
            if [ -d "${WORK_DIR}/esp-idf/components/${comp}" ]; then
                cp -r "${WORK_DIR}/esp-idf/components/${comp}" "${patch_tmp}/components/"
            fi
        done

        # Init temp dir as git repo so git apply works
        # Note: patches are tested BEFORE renames, matching the real sync order
        git -C "${patch_tmp}" init --quiet 2>/dev/null

        # Test each patch
        local failed_count=0
        for patch in "${PATCHES_DIR}"/*.patch; do
            local patch_name=$(basename "${patch}")
            local check_err
            if check_err=$(git -C "${patch_tmp}" apply --check "${patch}" 2>&1); then
                echo -e "  ${GREEN}✓${NC} ${patch_name}"
            elif git -C "${patch_tmp}" apply --check --reverse "${patch}" 2>/dev/null; then
                echo -e "  ${YELLOW}≡${NC} ${patch_name} (already applied)"
            else
                local total_hunks=$(grep -c '^@@' "${patch}" || true)
                local patch_out
                patch_out=$(patch -d "${patch_tmp}" --dry-run -p1 < "${patch}" 2>&1 || true)
                local failed_hunks=$(echo "${patch_out}" | grep -c 'FAILED' || true)
                local ok_hunks=$((total_hunks - failed_hunks))
                echo -e "  ${RED}✗${NC} ${patch_name} (${ok_hunks}/${total_hunks} hunks OK)"
                echo "${check_err}" | grep 'patch failed:' | sed 's/^.*patch failed:/      /'
                failed_count=$((failed_count + 1))
            fi
        done

        rm -rf "${patch_tmp}"

        if [ "${failed_count}" -gt 0 ]; then
            echo ""
            log_warn "${failed_count} patch(es) will need manual resolution after sync."
        fi
    else
        echo "  No patches found"
    fi
    echo ""

    log_info "Dry run complete. Run without --dry-run to apply changes."
}

make_diffstat_bar() {
    local ins=$1
    local del=$2
    local max_total=$3
    local max_width=60
    local total=$((ins + del))

    if [ "${total}" -eq 0 ] || [ "${max_total}" -eq 0 ]; then
        echo ""
        return
    fi

    local bar_width=$(( (total * max_width + max_total - 1) / max_total ))
    if [ "${bar_width}" -lt 1 ]; then
        bar_width=1
    fi

    local plus_width=$(( (ins * bar_width + total - 1) / total ))
    local minus_width=$(( bar_width - plus_width ))

    local bar=""
    local i
    for (( i=0; i<plus_width; i++ )); do bar="${bar}+"; done
    for (( i=0; i<minus_width; i++ )); do bar="${bar}-"; done

    echo "${GREEN}${bar:0:${plus_width}}${RED}${bar:${plus_width}}${NC}"
}

dry_run_component_summary() {
    cd "${HAL_ESPRESSIF_DIR}"

    local esp_idf_components="${WORK_DIR}/esp-idf/components"
    local requested_components=$(read_components)

    local added=""
    local removed=""
    local changed=""
    local unchanged=""
    local missing=""
    local added_count=0
    local removed_count=0
    local changed_count=0
    local unchanged_count=0
    local missing_count=0

    # First pass: collect data and find max label width for alignment
    local changed_labels=()
    local changed_stats=()
    local max_label_len=0

    for comp in ${requested_components}; do
        if [ ! -d "${esp_idf_components}/${comp}" ]; then
            missing="${missing}    ${RED}- ${comp}${NC} (not found in ESP-IDF)\n"
            missing_count=$((missing_count + 1))
        elif [ ! -d "components/${comp}" ]; then
            added="${added}    ${GREEN}+ ${comp}${NC}\n"
            added_count=$((added_count + 1))
        else
            local diff_count=$(diff -rq \
                "${esp_idf_components}/${comp}" "components/${comp}" \
                --exclude="test_apps" --exclude="*.a" \
                2>/dev/null | wc -l)
            if [ "${diff_count}" -gt 0 ]; then
                local diffstat
                diffstat=$(diff -r \
                    "${esp_idf_components}/${comp}" "components/${comp}" \
                    --exclude="test_apps" --exclude="*.a" \
                    2>/dev/null || true)
                local ins=0 del=0
                ins=$(echo "${diffstat}" | grep -c '^>' || true)
                del=$(echo "${diffstat}" | grep -c '^<' || true)
                local total=$((ins + del))
                local label="${comp} (${diff_count} file(s))"
                local label_len=${#label}
                if [ "${label_len}" -gt "${max_label_len}" ]; then
                    max_label_len=${label_len}
                fi
                changed_labels+=("${label}")
                changed_stats+=("${ins}:${del}:${total}")
                changed_count=$((changed_count + 1))
            else
                unchanged="${unchanged}    ${comp}\n"
                unchanged_count=$((unchanged_count + 1))
            fi
        fi
    done

    # Find max total for proportional bars
    local max_total=0
    for (( idx=0; idx<${#changed_stats[@]}; idx++ )); do
        local t=${changed_stats[$idx]##*:}
        if [ "${t}" -gt "${max_total}" ]; then
            max_total=${t}
        fi
    done

    # Second pass: format changed entries with aligned columns
    for (( idx=0; idx<${#changed_labels[@]}; idx++ )); do
        local label="${changed_labels[$idx]}"
        local stats="${changed_stats[$idx]}"
        local ins=${stats%%:*}
        local rest=${stats#*:}
        local del=${rest%%:*}
        local total=${rest##*:}
        local bar=$(make_diffstat_bar "${ins}" "${del}" "${max_total}")
        local padding=$((max_label_len - ${#label}))
        local pad=""
        for (( p=0; p<padding; p++ )); do pad="${pad} "; done
        changed="${changed}  ${YELLOW}${label}${NC}${pad} | ${total} ${bar}\n"
    done

    # Check for components in hal_espressif/components/ not in hal-config.yml
    if [ -d "components" ]; then
        for dir in components/*/; do
            [ -d "${dir}" ] || continue
            local comp_name=$(basename "${dir}")
            if ! echo "${requested_components}" | grep -qw "${comp_name}"; then
                removed="${removed}    ${RED}- ${comp_name}${NC} (not in hal-config.yml, will be removed)\n"
                removed_count=$((removed_count + 1))
            fi
        done
    fi

    echo ""
    echo "=============================================="
    echo "  COMPONENT SYNC SUMMARY"
    echo "=============================================="
    echo ""

    if [ "${added_count}" -gt 0 ]; then
        echo -e "ADDED (${added_count}) - new components to be synced:"
        echo -e "${added}"
    fi

    if [ "${removed_count}" -gt 0 ]; then
        echo -e "REMOVED (${removed_count}) - existing components not in hal-config.yml:"
        echo -e "${removed}"
    fi

    if [ "${changed_count}" -gt 0 ]; then
        echo -e "CHANGED (${changed_count}) - components with file differences:"
        echo -e "${changed}"
    fi

    if [ "${unchanged_count}" -gt 0 ]; then
        echo "UNCHANGED (${unchanged_count}) - components already in sync"
    fi

    if [ "${missing_count}" -gt 0 ]; then
        echo -e "\nWARNING (${missing_count}) - listed in hal-config.yml but not in ESP-IDF:"
        echo -e "${missing}"
    fi

    # Check for esp_hal_* components in ESP-IDF not tracked by hal-config.yml
    local untracked=""
    local untracked_count=0
    for hal_dir in "${esp_idf_components}"/esp_hal_*; do
        if [ -d "${hal_dir}" ]; then
            local hal_name=$(basename "${hal_dir}")
            if ! echo "${requested_components}" | grep -qw "${hal_name}"; then
                untracked="${untracked}    ${CYAN}? ${hal_name}${NC}\n"
                untracked_count=$((untracked_count + 1))
            fi
        fi
    done

    if [ "${untracked_count}" -gt 0 ]; then
        echo -e "UNTRACKED (${untracked_count}) - esp_hal_* in ESP-IDF not in hal-config.yml:"
        echo -e "${untracked}"
        log_notice "Consider adding them to tools/sync/hal-config.yml if needed."
        echo ""
    fi

    echo "=============================================="
    echo ""
}

show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  (none)              Full sync: fetch ESP-IDF, copy components, apply patches"
    echo "  --cleanup-only      Only run cleanup (remove test_apps, excluded paths, .a files)"
    echo "  --cleanup-rej       Remove all .rej and .orig files from components/"
    echo "  --commit            Commit pristine sync, patches, and blobs (up to 3 commits)"
    echo "  --continue          Apply renames after manual patch fix (patches already applied)"
    echo "  --dry-run           Preview what would change without modifying anything"
    echo "  --help              Show this help"
    echo "  --no-fetch          Full sync using existing ESP-IDF clone (skip fetch)"
    echo "  --patches-only      Only apply patches (skip ESP-IDF fetch/copy)"
    echo "  --report [REF]      Show report. With REF (e.g. HEAD~1), uses git rename detection"
    echo "  --rev REV           Use specific ESP-IDF revision (branch, tag, or commit SHA)"
    echo ""
    echo "Config:"
    echo "  hal-config.yml      Components, excludes, renames, blob filters"
    echo "  patches/*.patch     Zephyr adaptation patches"
    echo ""
    echo "Workflow:"
    echo "  1. Run --dry-run to preview changes"
    echo "  2. Run without options to sync from ESP-IDF master"
    echo "     (fetches ESP-IDF, copies components, applies patches, updates blobs)"
    echo "  3. If patches fail, fix manually and run --continue"
    echo "  4. Run --report to see new/deleted/moved files"
    echo "  5. Run hal-patches.sh to regenerate patches (if components were edited)"
    echo "  6. Review staged changes and run --commit"
}

commit_pristine() {
    cd "${HAL_ESPRESSIF_DIR}"

    local pristine_tree_file="${WORK_DIR}/pristine_tree"

    if [ ! -f "${pristine_tree_file}" ]; then
        die "Pristine tree not found. Run a full sync first."
    fi

    local pristine_tree
    pristine_tree=$(cat "${pristine_tree_file}")

    local head_tree
    head_tree=$(git rev-parse HEAD^{tree})

    if [ "${pristine_tree}" = "${head_tree}" ]; then
        log_info "No pristine changes to commit"
        return 0
    fi

    local esp_idf_sha=""
    if [ -f "${STATE_FILE}" ]; then
        esp_idf_sha=$(cat "${STATE_FILE}")
    fi

    local head_commit
    head_commit=$(git rev-parse HEAD)

    local commit_msg="hal: sync components from esp-idf"
    if [ -n "${esp_idf_sha}" ]; then
        commit_msg="${commit_msg}

ESP-IDF commit: ${esp_idf_sha}"
    fi

    local pristine_commit
    pristine_commit=$(git commit-tree "${pristine_tree}" -p "${head_commit}" -m "${commit_msg}")
    git reset --soft "${pristine_commit}"
    log_info "Pristine ESP-IDF sync committed: $(git rev-parse --short HEAD)"
}

commit_patches() {
    cd "${HAL_ESPRESSIF_DIR}"

    git add -A components/

    if git diff --cached --quiet; then
        log_info "No patch changes to commit"
        return 0
    fi

    local esp_idf_sha=""
    if [ -f "${STATE_FILE}" ]; then
        esp_idf_sha=$(cat "${STATE_FILE}")
    fi

    log_info "Committing adaptation patches..."
    local commit_msg="hal: apply zephyr adaptation patches"
    if [ -n "${esp_idf_sha}" ]; then
        commit_msg="${commit_msg}

ESP-IDF commit: ${esp_idf_sha}"
    fi

    git commit -m "${commit_msg}"
    log_info "Adaptation patches committed: $(git rev-parse --short HEAD)"
}

commit_blobs() {
    cd "${HAL_ESPRESSIF_DIR}"

    git add -A zephyr/module.yml

    if git diff --cached --quiet; then
        log_info "No blob changes to commit"
        return 0
    fi

    local esp_idf_sha=""
    if [ -f "${STATE_FILE}" ]; then
        esp_idf_sha=$(cat "${STATE_FILE}")
    fi

    log_info "Committing blob updates..."
    local commit_msg="hal: update blob references"
    if [ -n "${esp_idf_sha}" ]; then
        commit_msg="${commit_msg}

ESP-IDF commit: ${esp_idf_sha}"
    fi

    local blob_shas_file="${WORK_DIR}/blob_shas"
    if [ -f "${blob_shas_file}" ]; then
        commit_msg="${commit_msg}

Blob repositories:
$(sed 's/^/  /' "${blob_shas_file}")"
    fi

    git commit -m "${commit_msg}"
    log_info "Blob updates committed: $(git rev-parse --short HEAD)"
}

commit_components() {
    commit_pristine
    commit_patches
    commit_blobs
}

main() {
    # Parse --rev before other options
    local args=()
    while [ $# -gt 0 ]; do
        case "$1" in
            --rev)
                [ -z "${2:-}" ] && die "--rev requires a branch, tag, or commit SHA"
                ESP_IDF_REV="$2"
                shift 2
                ;;
            *)
                args+=("$1")
                shift
                ;;
        esac
    done
    set -- "${args[@]+"${args[@]}"}"

    case "${1:-}" in
        --cleanup-only)
            cleanup_components
            log_info "Cleanup complete! Review with: git status"
            ;;
        --cleanup-rej)
            log_info "Cleaning up .rej and .orig files..."
            cd "${HAL_ESPRESSIF_DIR}"
            find components -name '*.rej' -o -name '*.orig' | xargs rm -f 2>/dev/null || true
            log_info "Cleanup complete!"
            ;;
        --commit)
            commit_components
            ;;
        --continue)
            apply_renames
            commit_renames
            log_info "Sync complete! Review with: git status && git diff --staged"
            ;;
        --dry-run)
            dry_run_sync
            ;;
        --help|-h)
            show_usage
            exit 0
            ;;
        --no-fetch)
            if [ ! -d "${WORK_DIR}/esp-idf" ]; then
                die "ESP-IDF not found at ${WORK_DIR}/esp-idf. Run sync first."
            fi
            log_info "Using existing ESP-IDF clone (skipping fetch)..."
            cd "${WORK_DIR}/esp-idf"
            ESP_IDF_SHA_FULL=$(git rev-parse HEAD)
            echo "${ESP_IDF_SHA_FULL}" > "${STATE_FILE}"
            update_esp_idf_sha "${ESP_IDF_SHA_FULL}"
            log_info "ESP-IDF is at: ${ESP_IDF_SHA_FULL}"
            copy_components
            commit_pristine_sync
            apply_patches
            update_blobs
            apply_renames
            commit_renames
            log_info "Sync complete! Review with: git status && git diff --staged"
            ;;
        --patches-only)
            apply_patches
            apply_renames
            commit_renames
            log_info "Sync complete! Review with: git status && git diff --staged"
            ;;
        --report)
            generate_report "${2:-}"
            ;;
        "")
            fetch_espidf
            copy_components
            commit_pristine_sync
            apply_patches
            update_blobs
            apply_renames
            commit_renames
            log_info "Sync complete! Review with: git status && git diff --staged"
            ;;
        *)
            die "Unknown option: $1"
            ;;
    esac
}

main "$@"
