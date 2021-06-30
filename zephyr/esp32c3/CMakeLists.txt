# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_ESP32C3)
  zephyr_include_directories(
    include
    include/bt
    include/wifi

    ../../components/esp_common/include
    ../../components/esp_rom/include
    ../../components/esp_rom/include/esp32c3
    ../../components/esp32c3/include
    
    ../../components/esp_hw_support/port/esp32c3/private_include
    ../../components/esp_hw_support/include

    ../../components/riscv/include
    ../../components/hal/esp32c3/include
  
    ../../components/hal/include 
    
    ../../components/soc/esp32c3/include
    ../../components/soc/src/esp32c3/include
    ../../components/soc/include

    ../../components/driver/include

    ../../components/efuse/include
    ../../components/efuse/esp32c3/include
    ../../components/efuse/private_include/esp32c3

    ../../components/esp_system/include
    ../../components/esp_event/include
    ../../components/esp_timer/include
    ../../components/esp_timer/private_include
    ../../components/esp_netif/include
    ../../components/log/include
    ../../components/spi_flash/include
    ../../components/spi_flash/private_include
    ../../components/bootloader_support/include

    ../../components/esp_wifi/include/
    ../../components/esp_wifi/esp32c3/include
	)

  zephyr_link_libraries(
    gcc
    -T${CMAKE_CURRENT_SOURCE_DIR}/src/linker/esp32c3.rom.alias.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32c3/ld/esp32c3.rom.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32c3/ld/esp32c3.rom.api.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32c3/ld/esp32c3.rom.libgcc.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp32c3/ld/esp32c3.peripherals.ld
    )

  zephyr_compile_definitions(
    __ets__
    ESP_PLATFORM
    ESPRESSIF_USE
    )

  zephyr_sources(
    ../../components/esp_timer/src/esp_timer_impl_systimer.c
    ../../components/esp_hw_support/port/esp32c3/rtc_clk.c
    ../../components/driver/periph_ctrl.c
    ../../components/log/log_noos.c
    ../../components/log/log.c
    )

  ## shared WIFI/BT resources
  if (CONFIG_BT OR CONFIG_WIFI_ESP32)
    zephyr_sources( 
      ../../components/esp_wifi/src/phy_init.c
      )

    zephyr_link_libraries(
      net80211
      core
      pp
      coexist
      phy
      ## esp-idf wifi libs refer gcc libs symbols, so linked in libgcc
      gcc
        -L${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp32c3
        -L${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_wifi/lib/esp32c3
        -Wl,--defsym,phy_printf=printf
        -Wl,--defsym,net80211_printf=printf
        -Wl,--defsym,coexist_printf=printf
      )

  endif()

  ## BT definitions
  if (CONFIG_BT)
    zephyr_sources(
      src/bt/esp_bt_adapter.c
      )

    zephyr_compile_definitions(
      CONFIG_BT_ENABLED
      )

    zephyr_link_libraries(
      ## ble
      btbb
      btdm_app
      -L${CMAKE_CURRENT_SOURCE_DIR}/../../components/bt/controller/esp32c3-bt-lib/esp32c3
      coexist
      -L${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_wifi/lib/esp32c3
      )

  endif()

endif()