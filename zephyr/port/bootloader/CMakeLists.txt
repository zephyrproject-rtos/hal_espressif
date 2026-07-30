# SPDX-License-Identifier: Apache-2.0

# Wrapper forces NON_OS_BUILD so bootloader_flash_* use ROM/MMU semantics.
# Object name remains bootloader_flash.c for mcuboot.ld loader-section placement.
zephyr_sources(${CMAKE_CURRENT_LIST_DIR}/bootloader_flash.c)
