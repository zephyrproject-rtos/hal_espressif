# ESP-IDF Sync Tools

Tools for synchronizing hal_espressif components from ESP-IDF master.

## Quick Start

```bash
cd tools/sync

# Full sync from ESP-IDF master (default)
./hal-sync.sh

# Sync from a specific tag or commit
./hal-sync.sh --rev v5.4
./hal-sync.sh --rev abc123def

# Preview what would change without modifying anything
./hal-sync.sh --dry-run

# Regenerate patches after modifying components
./hal-patches.sh
```

## hal-sync.sh

Main synchronization script. Fetches ESP-IDF components and applies
Zephyr adaptation patches.

### Options

| Option | Description |
|--------|-------------|
| (none) | Full sync: fetch, copy, patch, rename (stages only) |
| `--cleanup-only` | Remove test_apps, excluded paths, .a files |
| `--cleanup-rej` | Remove .rej and .orig files from components/ |
| `--commit` | Commit pristine sync + adaptation patches (two commits) |
| `--continue` | Apply renames after manual patch fix |
| `--dry-run` | Preview what would change without modifying anything |
| `--no-fetch` | Full sync using existing ESP-IDF clone |
| `--patches-only` | Only apply patches and renames |
| `--report [REF]` | Show sync report (with optional git ref for rename detection) |
| `--rev REV` | Use specific ESP-IDF revision (branch, tag, or commit SHA) |

### Full Sync Workflow

When run without options (or with `--rev`):

1. Fetch ESP-IDF (shallow clone to `.work/esp-idf/`, or specific rev with `--rev`)
2. Copy components listed in `hal-config.yml`
3. Cleanup: remove test_apps, .a files, and excluded folders
4. Apply patches from `patches/`
5. Apply file/folder renames from `hal-config.yml`
6. Stage final state

The script stages changes but does not commit. Use `--commit` to
create two separate commits (pristine sync and adaptation patches),
or review with `git diff --staged`.

### Handling Patch Failures

If a patch fails, the script generates `.rej` files and stops:

1. Resolve conflicts using the `.rej` files as reference
2. Clean up: `./hal-sync.sh --cleanup-rej`
3. Regenerate the failed patch: `./hal-patches.sh`
4. Continue: `./hal-sync.sh --continue`

## hal-patches.sh

Regenerates patch files by comparing `components/` against the
ESP-IDF reference in `.work/esp-idf/`. One patch per component,
named `<component>.patch`.

Handles renamed files/folders (from `hal-config.yml`) by diffing
against the original ESP-IDF paths.

## hal-blobs.sh

Downloads blob binary libraries via HTTP, computes SHA256 hashes,
and regenerates `zephyr/module.yml`. Reads blob filters from
`hal-config.yml` to determine which libraries each SoC needs.

## Configuration

### hal-config.yml

Single configuration file containing all sync settings:

- **components** — ESP-IDF components to sync
- **exclude** — Subdirectories to remove after copying (relative to `components/`)
- **renames** — File/folder renames applied after patching (src/dst pairs)
- **blob_filters** — Binary library filters per SoC (used by `hal-blobs.sh`)

### hal-parser.sh

Shared YAML parser sourced by all scripts. Populates shell arrays
(`COMPONENTS`, `EXCLUDES`, `RENAME_SRC`, `RENAME_DST`, `BLOB_FILTER_*`)
from `hal-config.yml`.

### patches/

Zephyr adaptation patches, one per component, applied alphabetically:

```
patches/
  bt.patch
  esp_hw_support.patch
  hal.patch
  ...
```

## Directory Layout

```
tools/sync/
  hal-sync.sh        Main sync script
  hal-patches.sh     Regenerate patches from current state
  hal-blobs.sh       Update blob references and module.yml
  hal-config.yml     Sync configuration (components, excludes, renames, blob filters)
  hal-parser.sh      Shared YAML config parser
  patches/           Zephyr adaptation patches
  .work/             Working directory (git-ignored)
    esp-idf/         Shallow ESP-IDF clone
```

## Typical Workflow

### Updating from ESP-IDF master

```bash
# 1. Preview changes
./hal-sync.sh --dry-run

# 2. Run sync
./hal-sync.sh

# 3. Fix any patch failures, then continue
./hal-sync.sh --continue

# 4. Update blobs if needed
./hal-blobs.sh

# 5. Regenerate patches
./hal-patches.sh

# 6. Review and commit
./hal-sync.sh --commit

# 7. Build and test
west build -p -b esp32s3_devkitc/esp32s3/procpu samples/hello_world
```

### After modifying a component

```bash
# Regenerate affected patches
./hal-patches.sh

# Verify patches still apply cleanly
./hal-sync.sh --patches-only
```
