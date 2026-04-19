# SPDX-License-Identifier: Apache-2.0

if (CONFIG_HAS_ESPRESSIF_HAL)

if(CONFIG_MCUBOOT_ESPRESSIF)
  return()
endif()

zephyr_compile_definitions_ifdef(CONFIG_MCUBOOT BOOTLOADER_BUILD)
zephyr_compile_definitions_ifdef(CONFIG_ESP_PLATFORM ESP_PLATFORM)

# SOC_MMU_PAGE_SIZE is used by ESP-IDF code without CONFIG_ prefix
# Derive from Zephyr's MMU_PAGE_SIZE when available, otherwise default to 64KB
if(CONFIG_MMU_PAGE_SIZE)
  zephyr_compile_definitions(SOC_MMU_PAGE_SIZE=${CONFIG_MMU_PAGE_SIZE})
else()
  zephyr_compile_definitions(SOC_MMU_PAGE_SIZE=0x10000)
endif()

# ESP-IDF LL headers use (void)__DECLARE_RCC_ATOMIC_ENV as a compile-time
# guard. Zephyr handles atomicity in periph_rcc_enter/exit() and the
# PERIPH_RCC_* macros in periph_ctrl.h are patched to remove these tokens.
# Define as empty so (void)__DECLARE_RCC_ATOMIC_ENV becomes (void).
zephyr_compile_definitions(__DECLARE_RCC_ATOMIC_ENV)
zephyr_compile_definitions(__DECLARE_RCC_RC_ATOMIC_ENV)

# Include common Zephyr compatibility header
zephyr_compile_options(-include ${CMAKE_CURRENT_SOURCE_DIR}/port/include/zephyr_compat.h)

# ESP-IDF esp_perip_clk_init() disables USB Serial JTAG clock and PHY pad
# unless CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG_ENABLED is set.
if(CONFIG_ESP_USB_SERIAL_JTAG_ENABLED OR CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
  zephyr_compile_definitions(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG_ENABLED=1)
endif()

zephyr_sources_ifdef(
  CONFIG_CRYPTO_ESP32
  ../components/esp_hal_security/aes_hal.c
  ../components/esp_hal_security/sha_hal.c
  ../components/soc/dport_access_common.c
  )

add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32 esp32)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32C2 esp32c2)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32C3 esp32c3)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32C5 esp32c5)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32C6 esp32c6)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32H2 esp32h2)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32S2 esp32s2)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32S3 esp32s3)

endif() # CONFIG_HAS_ESPRESSIF_HAL
