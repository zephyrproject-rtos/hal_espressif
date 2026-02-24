# hal_espressif

Zephyr HAL module for Espressif SoCs. Contains ESP-IDF components adapted
for Zephyr RTOS, including low-level drivers, Wi-Fi, Bluetooth, and
system libraries.

## Supported SoCs

- ESP32
- ESP32-S2
- ESP32-S3
- ESP32-C2
- ESP32-C3
- ESP32-C6
- ESP32-H2

## Repository Structure

```
components/           ESP-IDF components (synced from ESP-IDF master)
tools/
  sync/               ESP-IDF sync tooling and adaptation patches
    patches/          Zephyr-specific patches applied on top of components
    hal-sync.sh    Main sync script
    hal-patches.sh
    components.txt    List of ESP-IDF components to sync
    exclude.txt       Folders to exclude after sync
    renames.txt       File/folder renames applied after patching
  ci/                 CI validation scripts
  flasher_stub/       Esptool flasher stub sources and linker scripts
  idf_monitor/        Serial monitor utility
west/                 West extension commands (serial monitor)
zephyr/
  CMakeLists.txt      Main build file
  Kconfig             Kconfig options
  module.yml          Zephyr module definition and blob manifest
  common/             Shared Zephyr integration code
  esp32*/             Per-SoC build files, linker scripts, and adaptations
  port/               Zephyr porting layer (boot, Wi-Fi, BLE, stubs)
  blobs/              Binary library manifests (Wi-Fi, BLE, PHY, coexistence)
  scripts/            Blob update and code generation tools
```

## ESP-IDF Sync Workflow

Components in `components/` are synced from ESP-IDF master using
`tools/sync/hal-sync.sh`. The sync process:

1. Clones/updates ESP-IDF master into `.work/esp-idf/`
2. Copies listed components and removes excluded folders
3. Applies Zephyr adaptation patches from `tools/sync/patches/`
4. On `--commit`, creates two separate commits: pristine sync and patches

After modifying components, regenerate patches with:

```bash
cd tools/sync
./hal-patches.sh
```

See [tools/sync/README.md](tools/sync/README.md) for full details.

## Binary Blobs

Wi-Fi, Bluetooth, PHY, and coexistence libraries are distributed as
binary blobs. Fetch them with:

```bash
west blobs fetch hal_espressif
```

Blob references are maintained in `zephyr/module.yml` and can be
updated with:

```bash
cd tools/sync
./hal-blobs.sh
```

## Getting Started

This module is fetched automatically by west as part of the Zephyr
build system. No manual setup is required.

For Zephyr development with Espressif devices, refer to the
[Zephyr documentation](https://docs.zephyrproject.org/).

## License

Apache-2.0. See [LICENSE](LICENSE) for details.

## Support

For issues and feature requests, use the Zephyr
[main repository](https://github.com/zephyrproject-rtos/zephyr).
