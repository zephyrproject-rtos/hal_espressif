# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_ESP32C3)
  zephyr_include_directories(
    adapter/include

    ../../components/esp_common/include
    ../../components/esp_rom/include/esp32c3
    ../../components/esp32c3/include
    
    ../../components/riscv/include
    ../../components/hal/esp32c3/include
  
    ../../components/hal/include 
    
    ../../components/soc/esp32c3/include
    ../../components/soc/src/esp32c3/include
    ../../components/soc/include

    ../../components/driver/include

    ../../components/efuse/include
    ../../components/esp_system/include
    ../../components/esp_timer/include
    ../../components/esp_timer/private_include

    ../../components/log/include
    ../../components/spi_flash/include
    ../../components/spi_flash/private_include
    ../../components/bootloader_support/include
    )

  zephyr_compile_definitions(
    __ets__
    ESP_PLATFORM
    ESPRESSIF_USE
    )

  zephyr_sources(
    ../../components/esp_timer/src/esp_timer_impl_systimer.c
    )
endif()
