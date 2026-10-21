# SPDX-License-Identifier: Apache-2.0

# Wrapper selects the upstream backend:
# - CONFIG_MCUBOOT: NON_OS_BUILD (ROM/MMU bootloader_flash_*)
# - else: OS backend (esp_flash_*/spi_flash_mmap), plus esp_rom_flash_read()
#   for Simple Boot early header reads before flash stack is ready.
# Object name remains bootloader_flash.c for mcuboot.ld / default.ld loader
# section placement.
zephyr_sources(${CMAKE_CURRENT_LIST_DIR}/bootloader_flash.c)
