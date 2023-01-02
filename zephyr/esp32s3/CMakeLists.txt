# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_ESP32S3)

  zephyr_compile_options(-Wno-unused-variable -Wno-maybe-uninitialized)

  zephyr_include_directories(
    include
    include/bt
    include/crypto
    ../esp_shared/include
    ../esp_shared/include/wifi
    ../esp_shared/components/include
    ../../components/hal/include
    ../../components/hal/esp32s3/include
    ../../components/hal/platform_port/include
    ../../components/esp_hw_support/include/soc
    ../../components/esp_hw_support/port/esp32s3
    ../../components/esp_hw_support/port/esp32s3/private_include
    ../../components/esp_hw_support/include
    ../../components/esp_common/include
    ../../components/esp_pm/include
    ../../components/esp_rom/include
    ../../components/esp_rom/esp32/ld
    ../../components/esp32s3/include
    ../../components/soc/esp32s3/include
    ../../components/xtensa/include
    ../../components/xtensa/esp32s3/include
    ../../components/soc/include/soc
    ../../components/soc/include
    ../../components/soc/src/esp32s3/include
    ../../components/driver/include
    ../../components/driver/esp32s3/include
    ../../components/esp_phy/include
    ../../components/esp_phy/include/esp32
    ../../components/esp_phy/esp32s3/include
    ../../components/esp_wifi/include
    ../../components/efuse/include
    ../../components/efuse/private_include
    ../../components/efuse/esp32s3/include
    ../../components/efuse/esp32s3/private_include
    ../../components/esp_system/include
    ../../components/esp_system/port/include
    ../../components/esp_system/port/public_compat
    ../../components/esp_wifi/esp32s3/include
    ../../components/esp_timer/include
    ../../components/esp_timer/private_include
    ../../components/esp_netif/include
    ../../components/esp_event/include
    ../../components/log/include
    ../../components/wpa_supplicant/esp_supplicant/src
    ../../components/wpa_supplicant/esp_supplicant/include
    ../../components/wpa_supplicant/include
    ../../components/wpa_supplicant/include/esp_supplicant
    ../../components/wpa_supplicant/port/include
    ../../components/wpa_supplicant/src
    ../../components/wpa_supplicant/src/crypto
    ../../components/wpa_supplicant/src/utils
    ../../components/wpa_supplicant/src/eap_peer
    ../../components/spi_flash/include
    ../../components/spi_flash/include/spi_flash
    ../../components/spi_flash/private_include
    ../../components/bootloader_support/include
    ../../components/bootloader_support/include_bootloader
    ../../components/heap/include
    ../../components/mbedtls/port/include
    )

  zephyr_link_libraries(
    gcc
      -T${CMAKE_CURRENT_SOURCE_DIR}/src/linker/esp32s3.rom.alias.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32s3/ld/esp32s3.rom.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32s3/ld/esp32s3.rom.api.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32s3/ld/esp32s3.rom.libgcc.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/soc/esp32s3/ld/esp32s3.peripherals.ld
    )

  zephyr_compile_definitions(ESP_PLATFORM)

  zephyr_link_libraries_ifdef(
    CONFIG_HEAP_MEM_POOL_SIZE
    gcc
    "-Wl,--wrap=k_calloc"
    "-Wl,--wrap=k_malloc"
    )

  zephyr_sources_ifdef(
    CONFIG_UART_ESP32
    ../../components/hal/uart_hal.c
    ../../components/hal/uart_hal_iram.c
  )

  zephyr_sources(
    ../../components/soc/esp32s3/gpio_periph.c
    ../../components/soc/esp32s3/rtc_io_periph.c
    ../../components/spi_flash/spi_flash_timing_tuning.c
    ../../components/spi_flash/esp32s3/spi_timing_config.c
    ../../components/esp_hw_support/regi2c_ctrl.c
    ../../components/esp_hw_support/cpu_util.c
    ../../components/esp_hw_support/esp_clk.c
    ../../components/esp_hw_support/port/esp32s3/dport_access.c
    ../../components/esp_hw_support/esp_clk.c
    ../../components/esp_hw_support/port/esp32s3/rtc_clk.c
    ../../components/esp_hw_support/port/esp32s3/rtc_init.c
    ../../components/esp_hw_support/port/esp32s3/rtc_time.c
    ../../components/esp_hw_support/port/esp32s3/rtc_clk_init.c
    ../../components/esp_hw_support/port/esp32s3/rtc_sleep.c
    ../../components/esp_hw_support/mac_addr.c
    ../../components/esp_rom/patches/esp_rom_uart.c
    ../../components/esp_system/port/soc/esp32s3/clk.c
    ../../components/driver/periph_ctrl.c
    ../../components/hal/cpu_hal.c
    ../../components/hal/interrupt_controller_hal.c
    ../../components/hal/esp32s3/interrupt_descriptor_table.c
    ../../components/esp_timer/src/esp_timer_impl_systimer.c
    ../../components/esp_timer/src/ets_timer_legacy.c
    ../../components/esp_timer/src/esp_timer.c
    ../../components/driver/periph_ctrl.c
    ../../components/log/log_noos.c
    ../../components/log/log.c
    ../../components/efuse/esp32s3/esp_efuse_fields.c
    ../../components/efuse/esp32s3/esp_efuse_table.c
    ../../components/efuse/esp32s3/esp_efuse_utility.c
    ../../components/efuse/src/esp_efuse_api.c
    ../../components/efuse/src/esp_efuse_api_key_esp32xx.c
    ../../components/efuse/src/esp_efuse_utility.c
    ../esp_shared/src/common/esp_system_api.c
    src/heap_caps.c
    src/stubs.c
    )

  zephyr_link_libraries_ifdef(CONFIG_NEWLIB_LIBC c)

endif()
