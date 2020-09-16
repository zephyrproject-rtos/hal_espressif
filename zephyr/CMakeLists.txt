# At the moment there is only one supported ESP32 SoC, and it is not
# supported to omit the esp-dif HAL library, so we don't check for a
# KConfig option to enable the HAL
#
# In the future it might look like this:
# if(CONFIG_ESP_DIF_LIBRARY)
# to allow users to disable the HAL

if(CONFIG_SOC_ESP32)
  zephyr_include_directories(
    adapter/include
    adapter/include/crypto
    adapter/include/wifi
    ../components/hal/include
    ../components/hal/esp32/include
    ../components/esp_hw_support/include
    ../components/esp_hw_support/port/esp32
    ../components/esp_hw_support/port/esp32/private_include
    ../components/esp_common/include
    ../components/esp_rom/include
    ../components/esp32/include
    ../components/soc/esp32/include
    ../components/xtensa/include
    ../components/xtensa/esp32/include
    ../components/soc/soc/include
    ../components/soc/soc/esp32/include
    ../components/soc/include
    ../components/soc/src/esp32/include
    ../components/driver/include
    ../components/soc/soc/esp32
    ../components/esp_wifi/include
    ../components/efuse/esp32/include
    ../components/efuse/include
    ../components/esp_system/include
    ../components/esp_wifi/esp32/include
    ../components/esp_timer/include
    ../components/esp_timer/private_include
    ../components/esp_netif/include
    ../components/esp_event/include
    ../components/log/include
    ../components/wpa_supplicant/include
    ../components/wpa_supplicant/port/include
    ../components/wpa_supplicant/src
    ../components/wpa_supplicant/include/esp_supplicant
    ../components/wpa_supplicant/src/crypto
    ../components/wpa_supplicant/src
    ../components/spi_flash/include
    ../components/spi_flash/private_include
    ../components/bootloader_support/include
    adapter/include
    )

  zephyr_link_libraries(
    xt_hal
    -L${CMAKE_CURRENT_SOURCE_DIR}/../components/xtensa/esp32
    )

  zephyr_sources_ifdef(
    CONFIG_SOC_FLASH_ESP32
    ../components/spi_flash/flash_mmap.c
    ../components/spi_flash/esp32/spi_flash_rom_patch.c
    ../components/spi_flash/flash_ops.c
    )

endif()
