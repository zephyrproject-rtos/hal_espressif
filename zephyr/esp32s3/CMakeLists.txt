# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_SERIES_ESP32S3)

  zephyr_compile_options(-fstrict-volatile-bitfields)
  zephyr_compile_definitions_ifndef(CONFIG_MCUBOOT CONFIG_APP_BUILD_USE_FLASH_SECTIONS)

  if(CONFIG_MCUBOOT)
    zephyr_compile_options(-fdump-rtl-expand)
  endif()

  zephyr_include_directories(
    include
    include/bt
    ../esp_shared/include
    ../esp_shared/components/include
    ../port/include
    ../common/include

    ../../components/efuse/include
    ../../components/efuse/private_include
    ../../components/efuse/${CONFIG_SOC_SERIES}/include
    ../../components/efuse/${CONFIG_SOC_SERIES}/private_include

    ../../components/esp_common/include
    ../../components/esp_hw_support/dma
    ../../components/esp_hw_support/include
    ../../components/esp_hw_support/include/esp_private
    ../../components/esp_hw_support/include/hal
    ../../components/esp_hw_support/include/soc
    ../../components/esp_hw_support/include/soc/${CONFIG_SOC_SERIES}
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}
    ../../components/esp_hw_support/port/include

    ../../components/esp_rom/${CONFIG_SOC_SERIES}
    ../../components/esp_rom/${CONFIG_SOC_SERIES}/ld
    ../../components/esp_rom/include/
    ../../components/esp_rom/include/${CONFIG_SOC_SERIES}

    ../../components/esp_system/include
    ../../components/esp_system/include/esp_private
    ../../components/esp_system/port/include
    ../../components/esp_system/port/include/private

    ../../components/hal/${CONFIG_SOC_SERIES}/include
    ../../components/hal/include
    ../../components/hal/include/hal
    ../../components/hal/platform_port/include

    ../../components/log/include

    ../../components/soc/${CONFIG_SOC_SERIES}/include
    ../../components/soc/${CONFIG_SOC_SERIES}/ld
    ../../components/soc/include

    ../../components/xtensa/${CONFIG_SOC_SERIES}/include
    ../../components/xtensa/include
    ../../components/xtensa/include/esp_private
    ../../components/xtensa/include/xtensa

    ../../components/esp_timer/include
    ../../components/esp_timer/private_include

    ../../components/driver/include
    ../../components/driver/deprecated
    ../../components/driver/gpio/include
    ../../components/driver/uart/include
    ../../components/driver/touch_sensor/include
    ../../components/driver/touch_sensor/${CONFIG_SOC_SERIES}/include
    ../../components/driver/spi/include

    ../../components/spi_flash
    ../../components/spi_flash/include
    ../../components/spi_flash/include/spi_flash

    ../../components/esp_pm/include

    ../../components/bootloader_support/bootloader_flash/include
    ../../components/bootloader_support/private_include
    ../../components/bootloader_support/include

    ../../components/heap/include

    ../../components/esp_psram/include

    ../../components/esp_mm/include

    ../../components/esp_netif/include

    ../../components/esp_coex/include
    ../../components/esp_phy/include
    ../../components/esp_phy/include/esp_private
    ../../components/esp_phy/${CONFIG_SOC_SERIES}/include
    ../../components/esp_wifi/include
    ../../components/esp_event/include
    ../../components/wpa_supplicant/esp_supplicant/src
    ../../components/wpa_supplicant/esp_supplicant/include
    ../../components/wpa_supplicant/include
    ../../components/wpa_supplicant/include/esp_supplicant
    ../../components/wpa_supplicant/port/include
    ../../components/wpa_supplicant/src
    ../../components/wpa_supplicant/src/crypto
    ../../components/wpa_supplicant/src/utils
    ../../components/wpa_supplicant/src/eap_peer
    ../../components/mbedtls/port/include

    ../port/include/boot
    )

  zephyr_link_libraries(
    gcc
      -T${CMAKE_CURRENT_SOURCE_DIR}/src/linker/${CONFIG_SOC_SERIES}.rom.alias.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.api.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.libgcc.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.newlib.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.version.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/soc/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.peripherals.ld
    )

  zephyr_compile_definitions(ESP_PLATFORM)

  zephyr_sources(
    ../../components/efuse/src/esp_efuse_api.c
    ../../components/efuse/src/esp_efuse_utility.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_table.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_utility.c
    ../../components/efuse/src/efuse_controller/keys/with_key_purposes/esp_efuse_api_key.c
  )

  if(CONFIG_SOC_FLASH_ESP32 OR NOT CONFIG_BOOTLOADER_MCUBOOT)
    zephyr_sources(
    ../../components/bootloader_support/src/flash_encrypt.c
    ../../components/esp_hw_support/esp_gpio_reserve.c
    ../../components/hal/spi_flash_encrypt_hal_iram.c
    ../../components/hal/spi_flash_hal.c
    ../../components/hal/spi_flash_hal_iram.c
    ../../components/hal/spi_flash_hal_gpspi.c
    ../../components/spi_flash/${CONFIG_SOC_SERIES}/spi_flash_oct_flash_init.c
    ../../components/spi_flash/esp_flash_api.c
    ../../components/spi_flash/esp_flash_spi_init.c
    ../../components/spi_flash/flash_mmap.c
    ../../components/spi_flash/flash_ops.c
    ../../components/spi_flash/memspi_host_driver.c
    ../../components/spi_flash/spi_flash_chip_boya.c
    ../../components/spi_flash/spi_flash_chip_drivers.c
    ../../components/spi_flash/spi_flash_chip_gd.c
    ../../components/spi_flash/spi_flash_chip_generic.c
    ../../components/spi_flash/spi_flash_chip_issi.c
    ../../components/spi_flash/spi_flash_chip_mxic.c
    ../../components/spi_flash/spi_flash_chip_mxic_opi.c
    ../../components/spi_flash/spi_flash_chip_th.c
    ../../components/spi_flash/spi_flash_chip_winbond.c
    ../../components/spi_flash/spi_flash_hpm_enable.c
    ../../components/spi_flash/spi_flash_os_func_noos.c
    ../../components/spi_flash/spi_flash_os_func_app.c
    ../../components/spi_flash/spi_flash_wrap.c
    )
  endif()

  zephyr_sources_ifdef(
    CONFIG_UART_ESP32
    ../../components/hal/uart_hal.c
    ../../components/hal/uart_hal_iram.c
    )

  zephyr_sources_ifdef(
    CONFIG_ESP32_SPIM
    ../../components/hal/spi_hal.c
    ../../components/hal/spi_hal_iram.c
    ../../components/soc/lldesc.c
    ../../components/hal/gdma_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_COUNTER_TMR_ESP32
    ../../components/hal/timer_hal.c
  )

  zephyr_sources_ifdef(
    CONFIG_I2C_ESP32
    ../../components/hal/i2c_hal_iram.c
    ../../components/hal/i2c_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_I2S_ESP32
    ../../components/hal/i2s_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_PWM_LED_ESP32
    ../../components/hal/ledc_hal_iram.c
    ../../components/hal/ledc_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_PCNT_ESP32
    ../../components/hal/pcnt_hal.c
    )

  if (CONFIG_ADC_ESP32)
    zephyr_include_directories(
    ../../components/esp_adc/${CONFIG_SOC_SERIES}/include
    ../../components/esp_adc/include
    ../../components/esp_adc/interface
    )

    zephyr_sources(
    ../../components/hal/adc_hal.c
    ../../components/hal/adc_hal_common.c
    ../../components/hal/adc_oneshot_hal.c
    ../../components/esp_hw_support/adc_share_hw_ctrl.c
    ../../components/hal/rtc_io_hal.c
    ../../components/driver/gpio/rtc_io.c
    ../../components/esp_adc/adc_cali.c
    ../../components/esp_adc/adc_cali_curve_fitting.c
    ../../components/esp_adc/${CONFIG_SOC_SERIES}/curve_fitting_coefficients.c
    ../../components/soc/${CONFIG_SOC_SERIES}/adc_periph.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_rtc_calib.c
    )
  endif()

  if (CONFIG_UDC_DWC2)
    zephyr_include_directories(
    ../../components/usb/include
    )

    zephyr_sources(
    ../../components/driver/gpio/gpio.c
    ../../components/driver/gpio/rtc_io.c
    ../../components/hal/usb_wrap_hal.c
    )
  endif()

  zephyr_sources_ifdef(
    CONFIG_DMA_ESP32
    ../../components/soc/lldesc.c
    ../../components/hal/gdma_hal.c
    )

  zephyr_link_libraries("-Wl,--wrap=longjmp")

  if (CONFIG_ESP_SPIRAM)
    zephyr_compile_definitions(CONFIG_SPIRAM)
    zephyr_sources(
      ../../components/esp_psram/esp_psram.c
      ../../components/esp_hw_support/esp_memory_utils.c
      ../../components/esp_hw_support/esp_gpio_reserve.c
      )

    zephyr_sources_ifdef(CONFIG_SPIRAM_MODE_QUAD
      ../../components/esp_psram/${CONFIG_SOC_SERIES}/esp_psram_impl_quad.c
      )

    zephyr_sources_ifdef(CONFIG_SPIRAM_MODE_OCT
      ../../components/esp_psram/${CONFIG_SOC_SERIES}/esp_psram_impl_octal.c
      )

    if (CONFIG_SPIRAM_FETCH_INSTRUCTIONS OR CONFIG_SPIRAM_RODATA)
      zephyr_sources(
        ../../components/esp_psram/mmu_psram_flash.c
        )
    endif()
  endif()

  zephyr_sources_ifdef(
    CONFIG_VIDEO_ESP32
    ../../components/hal/cam_hal.c
    )

  if(CONFIG_PM OR CONFIG_POWEROFF)
    zephyr_sources(
      ../../components/driver/gpio/gpio.c
      ../../components/esp_hw_support/sleep_gpio.c
      ../../components/esp_hw_support/sleep_event.c
      ../../components/esp_hw_support/sleep_console.c
      ../../components/driver/gpio/rtc_io.c
      ../../components/esp_hw_support/sleep_modem.c
      ../../components/hal/rtc_io_hal.c
      ../../components/hal/esp32s3/touch_sensor_hal.c
      ../../components/esp_hw_support/sleep_cpu.c
      ../../components/hal/${CONFIG_SOC_SERIES}/rtc_cntl_hal.c
    )
  endif()

  zephyr_sources_ifdef(
    CONFIG_INPUT_ESP32_TOUCH_SENSOR
    ../../components/soc/esp32s3/touch_sensor_periph.c
    ../../components/driver/gpio/rtc_io.c
    ../../components/driver/touch_sensor/touch_sensor_common.c
    ../../components/hal/rtc_io_hal.c
    ../../components/hal/touch_sensor_hal.c
    ../../components/hal/esp32s3/touch_sensor_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_ESP32_TEMP
    ../../components/driver/deprecated/rtc_temperature_legacy.c
    ../../components/esp_hw_support/sar_periph_ctrl_common.c
    ../../components/soc/${CONFIG_SOC_SERIES}/temperature_sensor_periph.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_rtc_calib.c
    ../../components/esp_hw_support/adc_share_hw_ctrl.c
    )

  if (CONFIG_MCUBOOT OR NOT CONFIG_BOOTLOADER_MCUBOOT)
    zephyr_include_directories(
      ../../components/esp_rom/${CONFIG_SOC_SERIES}
      )

    zephyr_sources(
      ../../components/bootloader_support/src/bootloader_clock_init.c
      ../../components/hal/mpu_hal.c
      ../../components/bootloader_support/bootloader_flash/src/flash_qio_mode.c
      ../../components/bootloader_support/bootloader_flash/src/bootloader_flash_config_${CONFIG_SOC_SERIES}.c
      ../common/console_init.c
      ../common/soc_init.c
      )

    zephyr_sources_ifdef(
      CONFIG_MCUBOOT
      ../port/boot/esp_image_loader.c
      )
  endif()

  zephyr_sources(
    ../../components/soc/${CONFIG_SOC_SERIES}/gpio_periph.c
    ../../components/soc/${CONFIG_SOC_SERIES}/rtc_io_periph.c

    ../../components/esp_hw_support/cpu.c
    ../../components/esp_hw_support/clk_ctrl_os.c
    ../../components/esp_hw_support/esp_clk.c
    ../../components/esp_hw_support/regi2c_ctrl.c
    ../../components/esp_hw_support/mac_addr.c
    ../../components/esp_hw_support/periph_ctrl.c
    ../../components/esp_hw_support/mspi_timing_tuning.c
    ../../components/esp_hw_support/rtc_module.c
    ../../components/esp_hw_support/hw_random.c
    ../../components/esp_hw_support/sleep_modes.c
    ../../components/esp_hw_support/port/esp_clk_tree_common.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/cpu_region_protect.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/esp_cpu_intr.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/esp_clk_tree.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/esp_cpu_intr.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_clk.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_clk_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_time.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_sleep.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/sar_periph_ctrl.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/systimer.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/mspi_timing_config.c

    ../../components/esp_mm/esp_mmu_map.c
    ../../components/esp_mm/port/${CONFIG_SOC_SERIES}/ext_mem_layout.c

    ../../components/esp_rom/patches/esp_rom_crc.c
    ../../components/esp_rom/patches/esp_rom_sys.c
    ../../components/esp_rom/patches/esp_rom_uart.c
    ../../components/esp_rom/patches/esp_rom_spiflash.c
    ../../components/esp_rom/patches/esp_rom_efuse.c
    ../../components/esp_rom/patches/esp_rom_gpio.c
    ../../components/esp_rom/patches/esp_rom_systimer.c
    ../../components/esp_rom/patches/esp_rom_wdt.c
    ../../components/esp_rom/patches/esp_rom_longjmp.S
    ../../components/esp_rom/patches/esp_rom_cache_esp32s2_esp32s3.c

    ../../components/esp_system/esp_err.c
    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/clk.c
    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/system_internal.c
    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/reset_reason.c

    ../../components/esp_timer/src/ets_timer_legacy.c
    ../../components/esp_timer/src/esp_timer.c
    ../../components/esp_timer/src/esp_timer_impl_common.c
    ../../components/esp_timer/src/esp_timer_impl_systimer.c

    ../../components/hal/efuse_hal.c
    ../../components/hal/cache_hal.c
    ../../components/hal/mmu_hal.c
    ../../components/hal/systimer_hal.c
    ../../components/hal/wdt_hal_iram.c

    ../../components/hal/${CONFIG_SOC_SERIES}/clk_tree_hal.c
    ../../components/hal/${CONFIG_SOC_SERIES}/efuse_hal.c

    ../../components/hal/xt_wdt_hal.c

    ../../components/log/log_noos.c
    ../../components/log/log.c

    ../port/host_flash/cache_utils.c
    ../port/heap/heap_caps_zephyr.c
    ../port/bootloader/bootloader_flash.c

    ../common/flash_init.c
    ../common/esp_restart.c

    src/stubs.c
    src/soc_random.c
    src/soc_init.c
    src/soc_flash_init.c
  )

  ## shared WIFI/BT resources
  if (CONFIG_BT OR CONFIG_WIFI_ESP32)
    zephyr_compile_definitions(CONFIG_ESP_PHY_ENABLE_USB)
    zephyr_sources(
      ../../components/esp_phy/src/phy_init.c
      ../../components/esp_phy/src/lib_printf.c
      ../../components/esp_phy/src/phy_common.c
      )

    if (CONFIG_ESP32_SW_COEXIST_ENABLE)
      if (CONFIG_BUILD_ONLY_NO_BLOBS)
        zephyr_sources(
          ../port/coex/coex_stubs.c
          )
      endif()

      zephyr_link_libraries(
        coexist
          -L${CMAKE_CURRENT_SOURCE_DIR}/../blobs/lib/esp32s3
        )
    endif()

    zephyr_link_libraries_ifndef(
      CONFIG_BUILD_ONLY_NO_BLOBS
      net80211
      core
      pp
      phy
      ## esp-idf wifi libs refer gcc libs symbols, so linked in libgcc
      gcc
        -L${CMAKE_CURRENT_SOURCE_DIR}/../blobs/lib/esp32s3
      )
  endif()

  ## BT definitions
  if (CONFIG_BT)

    zephyr_sources(src/bt/esp_bt_adapter.c)
    zephyr_compile_definitions(CONFIG_BT_ENABLED)

    zephyr_sources_ifdef(
      CONFIG_BUILD_ONLY_NO_BLOBS
      ../port/bluetooth/bt_stubs.c
      ../port/phy/phy_stubs.c
      )

    zephyr_link_libraries(
      gcc
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.bt_funcs.ld
      )

    zephyr_link_libraries_ifndef(
      CONFIG_BUILD_ONLY_NO_BLOBS
      ## ble
      btbb
      btdm_app
        -L${CMAKE_CURRENT_SOURCE_DIR}/../blobs/lib/esp32s3
      )
  endif()

    ## WIFI definitions
  if (CONFIG_WIFI_ESP32)

    zephyr_sources_ifdef(
      CONFIG_BUILD_ONLY_NO_BLOBS
      ../port/wifi/wifi_stubs.c
      ../port/phy/phy_stubs.c
      )

    set(WPA_SUPPLICANT_COMPONENT_DIR "../../components/wpa_supplicant")
    #TODO: Additional WPA supplicant feature like Enterprise mode etc. are yet to be supported.
    set(WPA_SUPPLICANT_SRCS "../port/wifi/wpa_supplicant/os_xtensa.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/port/eloop.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/ap/ap_config.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/ap/ieee802_1x.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/ap/wpa_auth.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/ap/wpa_auth_ie.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/ap/pmksa_cache_auth.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/ap/sta_info.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/ap/ieee802_11.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/ap/comeback_token.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/common/sae.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/common/dragonfly.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/common/wpa_common.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/utils/bitfield.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-siv.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha256-kdf.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/ccmp.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-gcm.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/crypto_ops.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/dh_group5.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/dh_groups.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/ms_funcs.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha1-tlsprf.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha256-tlsprf.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha384-tlsprf.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha256-prf.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha1-prf.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha384-prf.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/md4-internal.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha1-tprf.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_common/eap_wsc_common.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/common/ieee802_11_common.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/chap.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/eap.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/eap_common.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/mschapv2.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/rsn_supp/pmksa_cache.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/rsn_supp/wpa.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/rsn_supp/wpa_ie.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/utils/common.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/utils/ext_password.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/utils/uuid.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/utils/wpabuf.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/utils/wpa_debug.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/utils/json.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/wps/wps.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/wps/wps_attr_build.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/wps/wps_attr_parse.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/wps/wps_attr_process.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/wps/wps_common.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/wps/wps_dev_attr.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/wps/wps_enrollee.c"
      )

    if(CONFIG_ESP32_WIFI_ENABLE_SAE_PK)
      set(WPA_SUPPLICANT_SRCS ${WPA_SUPPLICANT_SRCS}
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/common/sae_pk.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/common/bss.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/common/scan.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_scan.c"
        )
    endif()

    set(ESP_SUPPLICANT_SRCS "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_eap_client.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_wpa2_api_port.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_wpa_main.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_wpas_glue.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_common.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_wps.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_wpa3.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_owe.c"
      )

    if(CONFIG_ESP32_WIFI_SOFTAP_SUPPORT)
        set(ESP_SUPPLICANT_SRCS ${ESP_SUPPLICANT_SRCS} "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_hostap.c")
    endif()

    set(TLS_SRCS
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/asn1.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/bignum.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/pkcs1.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/pkcs5.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/pkcs8.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/rsa.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/tls_internal.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/tlsv1_client.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/tlsv1_client_read.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/tlsv1_client_write.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/tlsv1_common.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/tlsv1_cred.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/tlsv1_record.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/tlsv1_client_ocsp.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/x509v3.c"
      )

    if(CONFIG_ESP32_WIFI_MBEDTLS_CRYPTO)
      set(CRYPTO_SRCS
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/crypto/fastpbkdf2.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/crypto/crypto_mbedtls.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/crypto/crypto_mbedtls-bignum.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/crypto/crypto_mbedtls-rsa.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/crypto/crypto_mbedtls-ec.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/crypto/fastpsk.c"
         # Add internal RC4 as RC4 has been removed from mbedtls
         "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/rc4.c"
        )
        if(NOT CONFIG_MBEDTLS_CIPHER_DES_ENABLED)
          set(CRYPTO_SRCS ${CRYPTO_SRCS} "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/des-internal.c")
        endif()
        # Enabling this only for WiFi is probably not a good idea since MbedTLS
        # uses generic crypto init/update functions for this. That causes
        # binary size increment since all the other enabled module
        # functions will also linked in. Even after not using direct MbedTLS APIs
        # for these, these API are still faster since these all will be using
        # AES APIs which is using hardware AES blocks.
        if(NOT CONFIG_MBEDTLS_MAC_CMAC_ENABLED)
            set(CRYPTO_SRCS ${CRYPTO_SRCS} "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-omac1.c")
        endif()
        if(NOT CONFIG_MBEDTLS_NIST_KW_C)
            set(CRYPTO_SRCS ${CRYPTO_SRCS}
            "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-wrap.c"
            "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-unwrap.c")
        endif()
        if(NOT CONFIG_MBEDTLS_NIST_KW_C OR NOT CONFIG_MBEDTLS_CMAC_C OR NOT CONFIG_MBEDTLS_CCM_C)
            set(CRYPTO_SRCS ${CRYPTO_SRCS} "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-ccm.c")
        endif()
    else()
      set(CRYPTO_SRCS
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/rc4.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-ctr.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-cbc.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-ccm.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-internal-dec.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-internal-enc.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-internal.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-omac1.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-unwrap.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-wrap.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/crypto_internal-cipher.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/crypto_internal-modexp.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/crypto_internal-rsa.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/crypto_internal.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/des-internal.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/md4-internal.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/md5-internal.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/md5.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha1-internal.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha1-pbkdf2.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha1.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha256-internal.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha256.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha384-internal.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha512-internal.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha256.c"
        )
    endif()

    zephyr_sources(
      src/wifi/esp_wifi_adapter.c
      src/coex/esp_coex_adapter.c
      ../port/wifi/wifi_init.c
      ${WPA_SUPPLICANT_SRCS}
      ${ESP_SUPPLICANT_SRCS}
      ${TLS_SRCS}
      ${CRYPTO_SRCS}
      )

    zephyr_compile_definitions(
      __ets__
      ESP_SUPPLICANT
      IEEE8021X_EAPOL
      EAP_PEER_METHOD
      EAP_MSCHAPv2
      EAP_TTLS
      EAP_TLS
      EAP_PEAP
      USE_WPA2_TASK
      CONFIG_WPS
      USE_WPS_TASK
      CONFIG_ECC
      CONFIG_IEEE80211W
      CONFIG_SHA256
      CONFIG_NO_RADIUS
      CONFIG_CRYPTO_INTERNAL
      )

    zephyr_compile_definitions(CONFIG_ESP_WIFI_ENABLED)
    zephyr_compile_definitions(CONFIG_SOC_WIFI_SUPPORTED)
    zephyr_compile_definitions_ifdef(CONFIG_ESP32_WIFI_MBEDTLS_CRYPTO CONFIG_CRYPTO_MBEDTLS)
    zephyr_compile_definitions_ifdef(CONFIG_ESP32_WIFI_ENABLE_WPA3_SAE CONFIG_WPA3_SAE)
    zephyr_compile_definitions_ifdef(CONFIG_ESP32_WIFI_ENABLE_WPA3_OWE_STA CONFIG_OWE_STA)
    zephyr_compile_definitions_ifdef(CONFIG_ESP32_WIFI_ENABLE_SAE_PK CONFIG_SAE_PK)
    zephyr_compile_definitions_ifdef(CONFIG_ESP32_WIFI_SOFTAP_SAE_SUPPORT CONFIG_SAE)

    zephyr_link_libraries(mbedTLS)

  endif()

  zephyr_link_libraries_ifdef(CONFIG_NEWLIB_LIBC c)

endif()
