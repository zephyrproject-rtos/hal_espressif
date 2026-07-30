# Copyright (c) 2026 Espressif Systems (Shanghai) Co., Ltd.
# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_SECURE_FLASH_ENC_ENABLED
  ${CMAKE_CURRENT_LIST_DIR}/mcuboot_boot_hooks.c
  ${CMAKE_CURRENT_LIST_DIR}/mcuboot_flash_encrypt.c
  ${CMAKE_CURRENT_LIST_DIR}/mcuboot_secure_boot_hooks.c
  ../../../components/bootloader_support/src/${CONFIG_SOC_SERIES}/flash_encryption_secure_features.c
  ../../../components/bootloader_support/src/bootloader_random.c
  ../../../components/bootloader_support/src/bootloader_random_${CONFIG_SOC_SERIES}.c
)
