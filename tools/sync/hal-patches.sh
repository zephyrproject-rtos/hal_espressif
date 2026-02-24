#!/bin/bash
#
# Regenerate Zephyr adaptation patches from current component changes
#
# Compares files in components/ against the ESP-IDF reference in
# tools/sync/.work/esp-idf/ to generate one patch per component.
#
# Patches are saved to patches/<component_name>.patch

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCHES_DIR="$SCRIPT_DIR/patches"
HAL_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ESP_IDF_REF="$SCRIPT_DIR/.work/esp-idf"

source "${SCRIPT_DIR}/hal-parser.sh"
parse_config

cd "$HAL_ROOT"

echo "Working in: $HAL_ROOT"
echo "Patches dir: $PATCHES_DIR"
echo "ESP-IDF reference: $ESP_IDF_REF"
echo ""

if [ ! -d "$ESP_IDF_REF" ]; then
    echo "ERROR: ESP-IDF reference not found at $ESP_IDF_REF"
    echo "Run hal-sync.sh first to fetch ESP-IDF"
    exit 1
fi

mkdir -p "$PATCHES_DIR"

should_exclude() {
    local path="$1"
    for pattern in "${EXCLUDES[@]}"; do
        if [[ "$path" == *"$pattern"* ]]; then
            return 0
        fi
    done
    return 1
}

updated=0
unchanged=0
created=0
no_diff=0
removed=0

resolve_renamed_path() {
    local result="$1"
    for (( i=0; i<${#RENAME_SRC[@]}; i++ )); do
        local src="${RENAME_SRC[$i]}"
        local dst="${RENAME_DST[$i]}"
        if [[ "$result" == "$src"* ]]; then
            result="${dst}${result#$src}"
        fi
    done
    echo "$result"
}

generate_component_patch() {
    local component_name="$1"
    local esp_idf_component="$ESP_IDF_REF/components/$component_name"
    local zephyr_component="$HAL_ROOT/components/$component_name"
    local patch_content=""

    if [ ! -d "$esp_idf_component" ]; then
        return 1
    fi

    if [ ! -d "$zephyr_component" ]; then
        return 1
    fi

    while IFS= read -r -d '' esp_file; do
        local rel_path="${esp_file#$esp_idf_component/}"
        local full_rel="$component_name/$rel_path"
        local patch_path="components/$full_rel"

        if should_exclude "$full_rel"; then
            continue
        fi

        # Resolve renames: map ESP-IDF path to hal path
        local resolved_rel=$(resolve_renamed_path "$full_rel")
        local zephyr_file="$HAL_ROOT/components/$resolved_rel"

        if [ -f "$zephyr_file" ]; then
            local file_diff=$(diff -u "$esp_file" "$zephyr_file" 2>/dev/null | \
                sed "s|^--- $esp_file.*|--- a/$patch_path|" | \
                sed "s|^+++ $zephyr_file.*|+++ b/$patch_path|")

            if [ -n "$file_diff" ]; then
                patch_content+="$file_diff"$'\n'
            fi
        fi
    done < <(find "$esp_idf_component" -type f \( -name "*.c" -o -name "*.h" -o -name "*.S" -o -name "*.ld" \) -print0 2>/dev/null | sort -z)

    echo "$patch_content"
}

echo "Processing components..."
echo ""

for component_name in "${COMPONENTS[@]}"; do
    patch_file="${component_name}.patch"
    patch_path="$PATCHES_DIR/$patch_file"

    if [ ! -d "$ESP_IDF_REF/components/$component_name" ]; then
        echo "  [S] $component_name: not in ESP-IDF reference (skipped)"
        continue
    fi

    if [ ! -d "$HAL_ROOT/components/$component_name" ]; then
        echo "  [S] $component_name: not in hal_espressif (skipped)"
        continue
    fi

    echo -n "  Checking $component_name... "

    new_patch=$(generate_component_patch "$component_name")

    if [ -z "$new_patch" ]; then
        echo "no differences"
        if [ -f "$patch_path" ]; then
            echo "    Removing obsolete patch: $patch_file"
            rm "$patch_path"
            ((removed++))
        fi
        ((no_diff++))
    elif [ -f "$patch_path" ]; then
        old_content=$(cat "$patch_path" 2>/dev/null || echo "")
        if [ "$old_content" = "$new_patch" ]; then
            echo "unchanged"
            ((unchanged++))
        else
            echo "UPDATED"
            echo "$new_patch" > "$patch_path"
            ((updated++))
        fi
    else
        echo "CREATED -> $patch_file"
        echo "$new_patch" > "$patch_path"
        ((created++))
    fi
done

echo ""
echo "Summary: $updated updated, $created created, $unchanged unchanged, $removed removed, $no_diff with no differences"
