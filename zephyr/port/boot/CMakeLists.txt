# Copyright (c) 2026 Espressif Systems (Shanghai) Co., Ltd.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_FAMILY_ESPRESSIF_FLASH_ENCRYPTION OR CONFIG_SOC_FAMILY_ESPRESSIF_SECURE_BOOT)
  zephyr_sources(
    ${CMAKE_CURRENT_LIST_DIR}/esp_mcuboot_boot_hooks.c
    ${CMAKE_CURRENT_LIST_DIR}/esp_mcuboot_security.c
  )
endif()

if(CONFIG_SOC_FAMILY_ESPRESSIF_FLASH_ENCRYPTION)
  zephyr_sources(
    ${CMAKE_CURRENT_LIST_DIR}/esp_mcuboot_flash_encrypt.c
    ../../../components/bootloader_support/src/${CONFIG_SOC_SERIES}/flash_encryption_secure_features.c
    ../../../components/bootloader_support/src/bootloader_random.c
  )

  # Weak Secure Boot hooks. The strong implementation replaces them when
  # Secure Boot v2 is enabled, including the combined Secure Boot + Flash
  # Encryption build.
  if(NOT CONFIG_SOC_FAMILY_ESPRESSIF_SECURE_BOOT)
    zephyr_sources(${CMAKE_CURRENT_LIST_DIR}/esp_mcuboot_secure_boot_hooks.c)
  endif()

  if(CONFIG_SOC_KEY_MANAGER_FE_KEY_DEPLOY)
    zephyr_sources(
      ${CMAKE_CURRENT_LIST_DIR}/esp_crypto_periph_clk.c
      ../../../components/esp_hal_security/key_mgr_hal.c
    )
  endif()
endif()

if(CONFIG_SOC_FAMILY_ESPRESSIF_SECURE_BOOT)
  zephyr_include_directories(
    ../../../components/bootloader_support/src/secure_boot_v2
  )
  zephyr_sources(
    ${CMAKE_CURRENT_LIST_DIR}/esp_mcuboot_secure_boot.c
    ${CMAKE_CURRENT_LIST_DIR}/bootloader_sha_zephyr.c
    ../../../components/bootloader_support/src/secure_boot_v2/secure_boot_signatures_bootloader.c
    ../../../components/bootloader_support/src/${CONFIG_SOC_SERIES}/secure_boot_secure_features.c
  )
endif()
