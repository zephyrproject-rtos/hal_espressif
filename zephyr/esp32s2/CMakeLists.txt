# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_ESP32S2)
  zephyr_include_directories(
    include
    ../../components/hal/include
    ../../components/hal/esp32s2/include
    ../../components/esp_common/include
    ../../components/esp_rom/include
    ../../components/esp_rom/esp32s2/ld
    ../../components/esp32s2/include
    ../../components/soc/esp32s2/include
    ../../components/xtensa/include
    ../../components/xtensa/esp32s2/include
    ../../components/soc/include
    ../../components/esp_system/include
    ../../components/bootloader_support/include
    )

    zephyr_link_libraries(
      gcc
        -T${CMAKE_CURRENT_SOURCE_DIR}/src/linker/esp32s2.rom.alias.ld
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32s2/ld/esp32s2.rom.ld
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32s2/ld/esp32s2.rom.api.ld
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32s2/ld/esp32s2.rom.libgcc.ld
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp32s2/ld/esp32s2.peripherals.ld
      )

  zephyr_sources(
    ../../components/soc/esp32s2/gpio_periph.c
    ../../components/hal/interrupt_controller_hal.c
    ../../components/hal/esp32s2/interrupt_descriptor_table.c
    )

endif()
