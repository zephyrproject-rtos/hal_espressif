# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_SERIES_ESP32C3)

  zephyr_compile_options(-Wno-unused-variable -Wno-maybe-uninitialized)

  zephyr_include_directories(
    include
    include/bt
    include/wifi
    include/crypto
    ../esp_shared/include
    ../esp_shared/include/wifi
    ../esp_shared/components/include
    ../port/include

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
    ../../components/hal/platform_port/include

    ../../components/log/include

    ../../components/soc/${CONFIG_SOC_SERIES}/include
    ../../components/soc/${CONFIG_SOC_SERIES}/ld
    ../../components/soc/include

    ../../components/riscv/${CONFIG_SOC_SERIES}/include
    ../../components/riscv/include
    ../../components/riscv/include/esp_private
    ../../components/riscv/include/xtensa

    ../../components/esp_timer/include
    ../../components/esp_timer/private_include

    ../../components/driver/include
    ../../components/driver/deprecated
    ../../components/driver/gpio/include
    ../../components/driver/uart/include
    ../../components/driver/touch_sensor/include
    ../../components/driver/touch_sensor/${CONFIG_SOC_SERIES}/include
    ../../components/driver/spi/include

    ../../components/spi_flash/include
    ../../components/spi_flash/include/spi_flash

    ../../components/esp_pm/include

    ../../components/bootloader_support/include
    ../../components/bootloader_support/bootloader_flash/include

    ../../components/heap/include

    ../../components/esp_mm/include

    # ../../components/esp_phy/include
    # ../../components/esp_phy/include
    # ../../components/esp_phy/esp32s3/include
    # ../../components/esp_wifi/include
    # ../../components/esp_event/include
    # ../../components/wpa_supplicant/esp_supplicant/src
    # ../../components/wpa_supplicant/esp_supplicant/include
    # ../../components/wpa_supplicant/include
    # ../../components/wpa_supplicant/include/esp_supplicant
    # ../../components/wpa_supplicant/port/include
    # ../../components/wpa_supplicant/src
    # ../../components/wpa_supplicant/src/crypto
    # ../../components/wpa_supplicant/src/utils
    # ../../components/wpa_supplicant/src/eap_peer
    # ../../components/spi_flash/include
    # ../../components/spi_flash/include/spi_flash
    # ../../components/spi_flash/private_include
    # ../../components/bootloader_support/include
    # ../../components/bootloader_support/include_bootloader
    # ../../components/mbedtls/port/include
  )

  zephyr_link_libraries(
    gcc
    -T${CMAKE_CURRENT_SOURCE_DIR}/src/linker/${CONFIG_SOC_SERIES}.rom.alias.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.eco3.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.api.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.libgcc.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/soc/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.peripherals.ld
  )

  zephyr_compile_definitions(ESP_PLATFORM)

  zephyr_sources_ifdef(
    CONFIG_ESP32_SPIM
    ../../components/hal/spi_hal.c
    ../../components/hal/spi_hal_iram.c
    ../../components/soc/lldesc.c
    ../../components/soc/${CONFIG_SOC_SERIES}/spi_periph.c
    ../../components/hal/gdma_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_SOC_FLASH_ESP32
    ../../components/spi_flash/flash_ops.c
    ../../components/spi_flash/esp_flash_api.c
    ../esp_shared/src/host_flash/cache_utils.c
    ../../components/spi_flash/flash_mmap.c
    ../../components/esp_mm/esp_mmu_map.c
    ../../components/bootloader_support/src/flash_encrypt.c
    ../../components/hal/mmu_hal.c
    ../../components/hal/cache_hal.c
    ../../components/spi_flash/esp_flash_spi_init.c
    ../../components/spi_flash/memspi_host_driver.c
    ../../components/hal/spi_flash_hal.c
    ../../components/hal/spi_flash_hal_iram.c
    ../../components/hal/spi_flash_hal_gpspi.c
    ../../components/spi_flash/spi_flash_os_func_noos.c
    ../../components/spi_flash/spi_flash_chip_drivers.c
    ../../components/spi_flash/spi_flash_chip_generic.c
    ../../components/hal/spi_flash_encrypt_hal_iram.c
    ../../components/esp_hw_support/esp_gpio_reserve.c
    )

  zephyr_sources_ifdef(
    CONFIG_I2C_ESP32
    ../../components/hal/i2c_hal_iram.c
    ../../components/hal/i2c_hal.c
    )

  zephyr_library_sources_ifdef(CONFIG_COUNTER_TMR_ESP32 ../../components/hal/timer_hal.c)

  zephyr_sources_ifdef(
    CONFIG_UART_ESP32
    ../../components/hal/uart_hal.c
    ../../components/hal/uart_hal_iram.c
    )

  zephyr_sources_ifdef(
    CONFIG_PWM_LED_ESP32
    ../../components/soc/esp32c3/ledc_periph.c
    ../../components/hal/ledc_hal_iram.c
    ../../components/hal/ledc_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_PM
    ../../components/esp_hw_support/sleep_gpio.c
    ../../components/driver/gpio/rtc_io.c
    ../../components/esp_hw_support/sleep_modes.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_sleep.c
    ../../components/esp_hw_support/sleep_modem.c
    ../../components/hal/rtc_io_hal.c
    ../../components/driver/gpio/gpio.c
    ../../components/esp_hw_support/sleep_cpu.c
    ../../components/hal/${CONFIG_SOC_SERIES}/rtc_cntl_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_ESP32_TEMP
    ../../components/driver/deprecated/rtc_temperature_legacy.c
    ../../components/soc//${CONFIG_SOC_SERIES}/temperature_sensor_periph.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_rtc_calib.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_table.c
    ../../components/efuse/src/esp_efuse_api.c
    ../../components/efuse/src/esp_efuse_utility.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_utility.c
    )

  zephyr_sources_ifdef(
    CONFIG_ADC_ESP32
    ../../components/hal/adc_hal.c
    ../../components/hal/adc_hal_common.c
    ../../components/driver/deprecated/adc_legacy.c
    ../../components/esp_hw_support/adc_share_hw_ctrl.c
    ../../components/hal/rtc_io_hal.c
    ../../components/driver/gpio/rtc_io.c
    src/esp_adc_cal/esp_adc_cal.c
    ../esp_shared/components/esp_adc_cal/esp_adc_cal_common.c
    ../../components/efuse/src/esp_efuse_api.c
    ../../components/efuse/src/esp_efuse_utility.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_rtc_calib.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_table.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_utility.c
  )

  zephyr_sources_ifdef(
    CONFIG_DMA_ESP32
    ../../components/soc/lldesc.c
    ../../components/hal/gdma_hal.c
    )

  if (CONFIG_MCUBOOT)
    zephyr_include_directories(
      ../../components/esp_rom/esp32c3
      ../esp_shared/include/boot
      )

    zephyr_sources(
      ../../components/bootloader_support/src/esp32c3/bootloader_soc.c
      ../../components/bootloader_support/src/bootloader_efuse_esp32c3.c
      ../../components/bootloader_support/src/bootloader_flash.c
      ../../components/bootloader_support/src/bootloader_flash_config_esp32c3.c
      ../../components/bootloader_support/src/bootloader_mem.c
      ../../components/bootloader_support/src/bootloader_clock_init.c
      ../../components/bootloader_support/src/bootloader_console.c
      ../../components/bootloader_support/src/bootloader_random.c
      ../../components/bootloader_support/src/bootloader_random_esp32c3.c
      ../../components/esp_hw_support/cpu_util.c
      ../../components/esp_hw_support/port/esp32c3/cpu_util_esp32c3.c
      ../../components/hal/mpu_hal.c
      ../../components/esp_rom/patches/esp_rom_uart.c
      ../../components/esp_rom/patches/esp_rom_sys.c
      ../esp_shared/src/boot/esp_loader.c
      ../esp_shared/src/boot/bootloader_wdt.c
      ../esp_shared/src/boot/bootloader_init_common.c
      src/boot/bootloader_init.c
      )
  endif()

  zephyr_sources(
    ../../components/soc/${CONFIG_SOC_SERIES}/adc_periph.c
    ../../components/soc/${CONFIG_SOC_SERIES}/i2s_periph.c
    ../../components/soc/${CONFIG_SOC_SERIES}/gpio_periph.c

    ../../components/esp_hw_support/cpu.c
    ../../components/esp_hw_support/esp_clk.c
    ../../components/esp_hw_support/mac_addr.c
    ../../components/esp_hw_support/periph_ctrl.c
    ../../components/esp_hw_support/regi2c_ctrl.c
    ../../components/esp_hw_support/sar_periph_ctrl_common.c

    ../../components/esp_hw_support/port/esp_clk_tree_common.c

    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/esp_clk_tree.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_clk.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_clk_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_time.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_sleep.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/sar_periph_ctrl.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/systimer.c

    ../../components/esp_rom/patches/esp_rom_crc.c
    ../../components/esp_rom/patches/esp_rom_sys.c
    ../../components/esp_rom/patches/esp_rom_uart.c
    ../../components/esp_rom/patches/esp_rom_spiflash.c
    ../../components/esp_rom/patches/esp_rom_efuse.c
    ../../components/esp_rom/patches/esp_rom_systimer.c

    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/clk.c
    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/system_internal.c
    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/reset_reason.c
    ../../components/esp_system/esp_err.c

    ../../components/esp_timer/src/ets_timer_legacy.c
    ../../components/esp_timer/src/esp_timer.c
    ../../components/esp_timer/src/esp_timer_impl_systimer.c
    ../../components/esp_timer/src/system_time.c

    ../../components/hal/efuse_hal.c
    ../../components/hal/cache_hal.c
    ../../components/hal/mmu_hal.c
    ../../components/hal/systimer_hal.c
    ../../components/hal/wdt_hal_iram.c

    ../../components/hal/${CONFIG_SOC_SERIES}/clk_tree_hal.c
    ../../components/hal/${CONFIG_SOC_SERIES}/efuse_hal.c

    ../../components/log/log_noos.c
    ../../components/log/log.c

    ../../components/esp_mm/port/${CONFIG_SOC_SERIES}/ext_mem_layout.c

    ../esp_shared/src/common/esp_system_api.c

    src/stubs.c

    ../esp_shared/src/common/heap_caps.c
    ../port/heap/heap_caps_zephyr.c
  )

  ## shared WIFI/BT resources
  if (CONFIG_BT OR CONFIG_WIFI_ESP32)
    zephyr_sources(
      ../../components/esp_phy/src/phy_init.c
      ../../components/esp_phy/src/lib_printf.c
      )

    zephyr_link_libraries(
      net80211
      core
      pp
      coexist
      phy
      mesh
      ## esp-idf wifi libs refer gcc libs symbols, so linked in libgcc
      gcc
        -L${CMAKE_CURRENT_SOURCE_DIR}/../blobs/lib/esp32c3
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
      coexist
        -L${CMAKE_CURRENT_SOURCE_DIR}/../blobs/lib/esp32c3
      )

  endif()

  ## WIFI definitions
  if (CONFIG_WIFI_ESP32)

    #TODO: Additional WPA supplicant feature like Enterprise mode etc. are yet to be supported.
    set(WPA_SUPPLICANT_SRCS "src/wpa_supplicant/port/os_riscv.c"
                       "../../components/wpa_supplicant/src/ap/ap_config.c"
                       "../../components/wpa_supplicant/src/ap/ieee802_1x.c"
                       "../../components/wpa_supplicant/src/ap/wpa_auth.c"
                       "../../components/wpa_supplicant/src/ap/wpa_auth_ie.c"
                       "../../components/wpa_supplicant/src/common/sae.c"
                       "../../components/wpa_supplicant/src/common/wpa_common.c"
                       "../../components/wpa_supplicant/src/crypto/aes-ctr.c"
                       "../../components/wpa_supplicant/src/crypto/aes-siv.c"
                       "../../components/wpa_supplicant/src/crypto/sha256-kdf.c"
                       "../../components/wpa_supplicant/src/crypto/sha256-prf.c"
                       "../../components/wpa_supplicant/src/crypto/aes-cbc.c"
                       "../../components/wpa_supplicant/src/crypto/aes-ccm.c"
                       "../../components/wpa_supplicant/src/crypto/aes-internal-dec.c"
                       "../../components/wpa_supplicant/src/crypto/aes-internal-enc.c"
                       "../../components/wpa_supplicant/src/crypto/aes-internal.c"
                       "../../components/wpa_supplicant/src/crypto/aes-omac1.c"
                       "../../components/wpa_supplicant/src/crypto/aes-unwrap.c"
                       "../../components/wpa_supplicant/src/crypto/aes-wrap.c"
                       "../../components/wpa_supplicant/src/crypto/aes-omac1.c"
                       "../../components/wpa_supplicant/src/crypto/sha256-tlsprf.c"
                       "../../components/wpa_supplicant/src/tls/bignum.c"
                       "../../components/wpa_supplicant/src/crypto/ccmp.c"
                       "../../components/wpa_supplicant/src/crypto/crypto_mbedtls-bignum.c"
                       "../../components/wpa_supplicant/src/crypto/crypto_mbedtls-ec.c"
                       "../../components/wpa_supplicant/src/crypto/crypto_ops.c"
                       "../../components/wpa_supplicant/src/crypto/crypto_internal-cipher.c"
                       "../../components/wpa_supplicant/src/crypto/crypto_internal-modexp.c"
                       "../../components/wpa_supplicant/src/crypto/crypto_internal-rsa.c"
                       "../../components/wpa_supplicant/src/crypto/crypto_internal.c"
                       "../../components/wpa_supplicant/src/crypto/des-internal.c"
                       "../../components/wpa_supplicant/src/crypto/dh_group5.c"
                       "../../components/wpa_supplicant/src/crypto/dh_groups.c"
                       "../../components/wpa_supplicant/src/crypto/md4-internal.c"
                       "../../components/wpa_supplicant/src/crypto/md5-internal.c"
                       "../../components/wpa_supplicant/src/crypto/md5.c"
                       "../../components/wpa_supplicant/src/crypto/ms_funcs.c"
                       "../../components/wpa_supplicant/src/crypto/rc4.c"
                       "../../components/wpa_supplicant/src/crypto/sha1-internal.c"
                       "../../components/wpa_supplicant/src/crypto/sha1-pbkdf2.c"
                       "../../components/wpa_supplicant/src/crypto/sha1-prf.c"
                       "../../components/wpa_supplicant/src/crypto/sha1.c"
                       "../../components/wpa_supplicant/src/crypto/sha256-internal.c"
                       "../../components/wpa_supplicant/src/crypto/sha256.c"
                       "../../components/wpa_supplicant/src/eap_peer/eap.c"
                       "../../components/wpa_supplicant/src/eap_peer/eap_common.c"
                       "../../components/wpa_supplicant/esp_supplicant/src/esp_hostap.c"
                       "../../components/wpa_supplicant/esp_supplicant/src/esp_wpa_main.c"
                       "../../components/wpa_supplicant/esp_supplicant/src/esp_wpas_glue.c"
                       "../../components/wpa_supplicant/esp_supplicant/src/esp_wpa3.c"
                       "../../components/wpa_supplicant/src/rsn_supp/pmksa_cache.c"
                       "../../components/wpa_supplicant/src/rsn_supp/wpa.c"
                       "../../components/wpa_supplicant/src/rsn_supp/wpa_ie.c"
                       "../../components/wpa_supplicant/src/utils/common.c"
                       "../../components/wpa_supplicant/src/utils/ext_password.c"
                       "../../components/wpa_supplicant/src/utils/uuid.c"
                       "../../components/wpa_supplicant/src/utils/wpabuf.c"
                       "../../components/wpa_supplicant/src/utils/wpa_debug.c"
                       "../../components/wpa_supplicant/src/utils/json.c"
                       )

    zephyr_sources(
      src/wifi/esp_wifi_adapter.c
      ${WPA_SUPPLICANT_SRCS}
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
      EAP_FAST
      USE_WPA2_TASK
      CONFIG_WPS2
      CONFIG_WPS_PIN
      USE_WPS_TASK
      ESPRESSIF_USE
      ESP32_WORKAROUND
      CONFIG_ECC
      CONFIG_IEEE80211W
      CONFIG_SHA256
      CONFIG_DPP
      CONFIG_WNM
      )

    zephyr_link_libraries(
      ## wifi
      mbedTLS
      )

  endif()

  zephyr_link_libraries_ifdef(CONFIG_NEWLIB_LIBC c)

endif()
