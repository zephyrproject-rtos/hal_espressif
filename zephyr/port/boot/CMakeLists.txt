# Copyright (c) 2026 Espressif Systems (Shanghai) Co., Ltd.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SECURE_FLASH_ENC_ENABLED)
  zephyr_sources(
    ${CMAKE_CURRENT_LIST_DIR}/mcuboot_boot_hooks.c
    ${CMAKE_CURRENT_LIST_DIR}/mcuboot_flash_encrypt.c
    ${CMAKE_CURRENT_LIST_DIR}/mcuboot_secure_boot_hooks.c
    ../../../components/bootloader_support/src/${CONFIG_SOC_SERIES}/flash_encryption_secure_features.c
    ../../../components/bootloader_support/src/bootloader_random.c
    ../../../components/bootloader_support/src/bootloader_random_${CONFIG_SOC_SERIES}.c
  )

  if(CONFIG_SOC_KEY_MANAGER_FE_KEY_DEPLOY)
    zephyr_sources(
      ${CMAKE_CURRENT_LIST_DIR}/esp_crypto_periph_clk.c
      ../../../components/esp_hal_security/key_mgr_hal.c
    )
  endif()
endif()
