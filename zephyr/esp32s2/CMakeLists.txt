# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_SERIES_ESP32S2)

  zephyr_compile_options(-Wno-unused-variable  -Wno-maybe-uninitialized)

  zephyr_include_directories(
    include
    ../esp_shared/include
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

    ../../components/spi_flash/include
    ../../components/spi_flash/include/spi_flash

    ../../components/esp_pm/include

    ../../components/bootloader_support/include
    ../../components/bootloader_support/bootloader_flash/include
    ../../components/bootloader_support/private_include

    ../../components/heap/include

    ../../components/esp_mm/include

    ../../components/esp_psram/include

    ../../components/esp_netif/include

    ../../components/esp_phy/include
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
    )

  zephyr_link_libraries(
    gcc
      -T${CMAKE_CURRENT_SOURCE_DIR}/src/linker/${CONFIG_SOC_SERIES}.rom.alias.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.newlib-funcs.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.newlib-data.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.api.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.libgcc.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.spiflash.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/soc/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.peripherals.ld
    )

  zephyr_compile_definitions(ESP_PLATFORM)

  zephyr_link_libraries_ifdef(
    CONFIG_ESP_SPIRAM
    gcc
    "-Wl,--wrap=k_calloc"
    "-Wl,--wrap=k_malloc"
    )

  zephyr_sources_ifdef(
    CONFIG_SOC_FLASH_ESP32
    ../../components/spi_flash/flash_ops.c
    ../../components/spi_flash/esp_flash_api.c
    ../../components/bootloader_support/src/flash_encrypt.c
    ../../components/spi_flash/flash_mmap.c
    ../esp_shared/src/host_flash/cache_utils.c
    ../../components/hal/mmu_hal.c
    ../../components/spi_flash/esp_flash_spi_init.c
    ../../components/hal/cache_hal.c
    ../../components/spi_flash/spi_flash_os_func_noos.c
    ../../components/spi_flash/memspi_host_driver.c
    ../../components/hal/spi_flash_hal.c
    ../../components/hal/spi_flash_hal_iram.c
    ../../components/hal/spi_flash_hal_gpspi.c
    ../../components/spi_flash/spi_flash_chip_drivers.c
    ../../components/spi_flash/spi_flash_chip_generic.c
    ../../components/hal/spi_flash_encrypt_hal_iram.c
    ../../components/esp_hw_support/esp_gpio_reserve.c
    ../../components/bootloader_support/bootloader_flash/src/bootloader_flash.c
    ../../components/bootloader_support/bootloader_flash/src/bootloader_flash_config_esp32s2.c
    ../../components/bootloader_support/src/bootloader_efuse.c
    )

  zephyr_library_sources_ifdef(CONFIG_COUNTER_TMR_ESP32 ../../components/hal/timer_hal.c)

  zephyr_sources_ifdef(
    CONFIG_ESP32_SPIM
    ../../components/hal/spi_hal.c
    ../../components/hal/spi_hal_iram.c
    ../../components/soc/lldesc.c
    ../../components/soc/${CONFIG_SOC_SERIES}/spi_periph.c
    )

  zephyr_sources_ifdef(
    CONFIG_ESP_SPIRAM
    ../../components/esp_psram/esp_psram.c
    ../../components/esp_hw_support/esp_memory_utils.c
    ../../components/esp_psram/${CONFIG_SOC_SERIES}/esp_psram_impl_quad.c
    ../../components/esp_hw_support/esp_gpio_reserve.c
    ../../components/hal/mmu_hal.c
    ../../components/spi_flash/flash_ops.c
    ../esp_shared/src/host_flash/cache_utils.c
    )

  zephyr_sources_ifdef(
    CONFIG_I2C_ESP32
    ../../components/hal/i2c_hal_iram.c
    ../../components/hal/i2c_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_UART_ESP32
    ../../components/hal/uart_hal.c
    ../../components/hal/uart_hal_iram.c
    )

  zephyr_sources_ifdef(
    CONFIG_PWM_LED_ESP32
    ../../components/soc/esp32s2/ledc_periph.c
    ../../components/hal/ledc_hal_iram.c
    ../../components/hal/ledc_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_PCNT_ESP32
    ../../components/hal/pcnt_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_DAC_ESP32
    ../../components/soc/${CONFIG_SOC_SERIES}/dac_periph.c
    ../../components/driver/deprecated/dac_common_legacy.c
    ../../components/hal/rtc_io_hal.c
    ../../components/driver/gpio/rtc_io.c
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
    ../../components/efuse/src/esp_efuse_api.c
    ../../components/efuse/src/esp_efuse_utility.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_rtc_calib.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_rtc_table.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_utility.c
    )

  zephyr_sources_ifdef(
    CONFIG_PM
    ../../components/esp_hw_support/sleep_gpio.c
    ../../components/driver/gpio/rtc_io.c
    ../../components/esp_hw_support/sleep_modes.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_sleep.c
    ../../components/esp_hw_support/sleep_modem.c
    ../../components/hal/rtc_io_hal.c
    ../../components/esp_system/port/brownout.c
    ../../components/hal/brownout_hal.c
    ../../components/hal/esp32s3/touch_sensor_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_ESP32_TEMP
    ../../components/driver/deprecated/rtc_temperature_legacy.c
    ../../components/soc//${CONFIG_SOC_SERIES}/temperature_sensor_periph.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_rtc_calib.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_rtc_table.c
    ../../components/efuse/src/esp_efuse_api.c
    ../../components/efuse/src/esp_efuse_utility.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_utility.c
    )

  zephyr_link_libraries("-Wl,--wrap=longjmp")

  if (CONFIG_MCUBOOT)
    zephyr_include_directories(
      ../../components/esp_rom/esp32s2
      ../port/include/boot
      )

    zephyr_sources(
      ../../components/bootloader_support/src/${CONFIG_SOC_SERIES}/bootloader_soc.c
      ../../components/bootloader_support/src/bootloader_mem.c
      ../../components/bootloader_support/src/bootloader_clock_init.c
      ../../components/bootloader_support/src/bootloader_console.c
      ../../components/bootloader_support/src/bootloader_random.c
      ../../components/esp_rom/patches/esp_rom_sys.c
      ../../components/hal/mpu_hal.c
      ../../components/bootloader_support/src/${CONFIG_SOC_SERIES}/bootloader_${CONFIG_SOC_SERIES}.c
      ../../components/bootloader_support/src/bootloader_init.c
      ../../components/bootloader_support/src/bootloader_random_${CONFIG_SOC_SERIES}.c
      ../../components/bootloader_support/src/bootloader_common.c
      ../../components/bootloader_support/src/bootloader_common_loader.c

      ../port/boot/esp_loader.c

      )
  endif()

  zephyr_sources(
    ../../components/soc/${CONFIG_SOC_SERIES}/adc_periph.c
    ../../components/soc/${CONFIG_SOC_SERIES}/i2s_periph.c
    ../../components/soc/${CONFIG_SOC_SERIES}/gpio_periph.c
    ../../components/soc/${CONFIG_SOC_SERIES}/rtc_io_periph.c

    ../../components/esp_hw_support/cpu.c
    ../../components/esp_hw_support/clk_ctrl_os.c
    ../../components/esp_hw_support/esp_clk.c
    ../../components/esp_hw_support/mac_addr.c
    ../../components/esp_hw_support/periph_ctrl.c
    ../../components/esp_hw_support/regi2c_ctrl.c
    ../../components/esp_hw_support/rtc_module.c
    ../../components/esp_hw_support/sar_periph_ctrl_common.c

    ../../components/esp_hw_support/port/esp_clk_tree_common.c

    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/esp_clk_tree.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_clk.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_clk_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_time.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_sleep.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/sar_periph_ctrl.c
    ../../components/esp_rom/patches/esp_rom_crc.c
    ../../components/esp_rom/patches/esp_rom_sys.c
    ../../components/esp_rom/patches/esp_rom_uart.c
    ../../components/esp_rom/patches/esp_rom_spiflash.c
    ../../components/esp_rom/patches/esp_rom_efuse.c
    ../../components/esp_rom/patches/esp_rom_systimer.c
    ../../components/esp_rom/patches/esp_rom_wdt.c
    ../../components/esp_rom/patches/esp_rom_longjmp.S
    ../../components/esp_rom/patches/esp_rom_regi2c_esp32s2.c
    ../../components/esp_rom/patches/esp_rom_cache_esp32s2_esp32s3.c


    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/clk.c
    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/system_internal.c
    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/reset_reason.c

    ../../components/esp_timer/src/ets_timer_legacy.c
    ../../components/esp_timer/src/esp_timer.c
    ../../components/esp_timer/src/esp_timer_impl_lac.c
    ../../components/esp_timer/src/system_time.c

    ../../components/hal/efuse_hal.c
    ../../components/hal/cache_hal.c
    ../../components/hal/mmu_hal.c
    ../../components/hal/wdt_hal_iram.c

    ../../components/hal/${CONFIG_SOC_SERIES}/clk_tree_hal.c
    ../../components/hal/${CONFIG_SOC_SERIES}/efuse_hal.c

    ../../components/log/log_noos.c
    ../../components/log/log.c

    ../../components/esp_mm/port/${CONFIG_SOC_SERIES}/ext_mem_layout.c
    ../../components/esp_mm/esp_mmu_map.c

    ../esp_shared/src/common/esp_system_api.c

    src/stubs.c

    ../port/heap/heap_caps_zephyr.c
  )

  ## WIFI definitions
  if (CONFIG_WIFI_ESP32)
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
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/utils/base64.c"
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

    set(ESP_SUPPLICANT_SRCS "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_wpa2.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_wpa_main.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_wpas_glue.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_common.c"
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
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/x509v3.c")

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
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/sha256.c")

    zephyr_sources(
      ../../components/esp_system/esp_err.c
      ../esp_shared/src/host_flash/cache_utils.c
      ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_table.c
      ../../components/efuse/src/esp_efuse_api.c
      ../../components/efuse/src/esp_efuse_utility.c
      ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_utility.c
      ../../components/esp_hw_support/sleep_modes.c

      src/wifi/esp_wifi_adapter.c
      ../../components/esp_phy/src/phy_init.c
      ../../components/esp_phy/src/lib_printf.c
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
    zephyr_link_libraries(
      net80211
      core
      pp
      phy
      # mbedTLSCrypto
      mbedTLS
      ## esp-idf wifi libs refer gcc libs symbols, so linked in libgcc
      gcc
        -L${CMAKE_CURRENT_SOURCE_DIR}/../blobs/lib/esp32s2
    )

  endif() ## CONFIG_WIFI_ESP32

  zephyr_link_libraries_ifdef(CONFIG_NEWLIB_LIBC c)

endif()
