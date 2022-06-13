# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_ESP32S2)

  zephyr_compile_options(-Wno-unused-variable)

  zephyr_include_directories(
    include
    include/crypto
    include/wifi
    ../esp_shared/include
    ../esp_shared/include/wifi
    ../../components/hal/include
    ../../components/hal/esp32s2/include
    ../../components/esp_hw_support/port/esp32s2
    ../../components/esp_hw_support/port/esp32s2/private_include
    ../../components/esp_hw_support/include
    ../../components/esp_common/include
    ../../components/esp_rom/include
    ../../components/esp_rom/esp32s2/ld
    ../../components/esp32s2/include
    ../../components/driver/include
    ../../components/soc/esp32s2/include
    ../../components/xtensa/include
    ../../components/xtensa/esp32s2/include
    ../../components/soc/include
    ../../components/driver/include
    ../../components/esp_system/include
    ../../components/esp_wifi/esp32s2/include
    ../../components/esp_wifi/include
    ../../components/esp_timer/include
    ../../components/esp_timer/private_include
    ../../components/esp_netif/include
    ../../components/esp_event/include
    ../../components/log/include
    ../../components/wpa_supplicant/include
    ../../components/wpa_supplicant/port/include
    ../../components/wpa_supplicant/src
    ../../components/wpa_supplicant/include/esp_supplicant
    ../../components/spi_flash/include
    ../../components/spi_flash/private_include
    ../../components/bootloader_support/include
    ../../components/efuse/include
    ../../components/efuse/include/esp32s2
    ../../components/efuse/esp32s2/include
    ../../components/bootloader_support/include_bootloader
    )

    zephyr_link_libraries(
      gcc
        -T${CMAKE_CURRENT_SOURCE_DIR}/src/linker/esp32s2.rom.alias.ld
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32s2/ld/esp32s2.rom.ld
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32s2/ld/esp32s2.rom.api.ld
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32s2/ld/esp32s2.rom.libgcc.ld
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp32s2/ld/esp32s2.peripherals.ld
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32s2/ld/esp32s2.rom.spiflash.ld
      )

  zephyr_link_libraries_ifdef(
    CONFIG_HEAP_MEM_POOL_SIZE
    gcc
    "-Wl,--wrap=k_calloc"
    "-Wl,--wrap=k_malloc"
    )

  zephyr_sources_ifdef(
    CONFIG_SOC_FLASH_ESP32
    ../esp_shared/src/host_flash/cache_utils.c
    ../../components/spi_flash/flash_mmap.c
    ../../components/spi_flash/esp32s2/spi_flash_rom_patch.c
    ../../components/spi_flash/flash_ops.c
    ../../components/spi_flash/esp32s2/flash_ops_esp32s2.c
    )

  zephyr_compile_definitions(
    CONFIG_SPI_FLASH_ROM_DRIVER_PATCH
    CONFIG_SPI_FLASH_USE_LEGACY_IMPL
    )

  zephyr_library_sources_ifdef(CONFIG_COUNTER_ESP32     ../../components/hal/timer_hal.c)

  zephyr_sources_ifdef(
    CONFIG_ESP32_SPIM
    ../../components/hal/spi_hal.c
    ../../components/hal/spi_hal_iram.c
    ../../components/soc/lldesc.c
    ../../components/soc/esp32s2/spi_periph.c
    )

  zephyr_sources_ifdef(
    CONFIG_ESP_SPIRAM
    ../esp_shared/src/host_flash/cache_utils.c
    ../../components/spi_flash/flash_ops.c
    ../../components/esp32s2/spiram.c
    ../../components/esp32s2/spiram_psram.c
    ../../components/esp_hw_support/port/esp32s2/rtc_init.c
    ../../components/soc/esp32s2/gpio_periph.c
    ../../components/bootloader_support/src/bootloader_common.c
    )

  zephyr_sources_ifdef(
    CONFIG_I2C_ESP32
    ../../components/hal/i2c_hal_iram.c
    ../../components/hal/i2c_hal.c
    )

  zephyr_sources_ifdef(
    CONFIG_WDT_ESP32
    ../../components/hal/wdt_hal_iram.c
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

  zephyr_sources(
    ../../components/soc/esp32s2/gpio_periph.c
    ../../components/soc/esp32s2/rtc_io_periph.c
    ../../components/esp_hw_support/port/esp32s2/rtc_clk.c
    ../../components/esp_hw_support/port/esp32s2/rtc_init.c
    ../../components/esp_hw_support/port/esp32s2/rtc_time.c
    ../../components/esp_hw_support/port/esp32s2/rtc_clk_init.c
    ../../components/esp32s2/clk.c
    ../../components/esp_hw_support/port/esp32s2/regi2c_ctrl.c
    ../../components/driver/periph_ctrl.c
    ../../components/hal/interrupt_controller_hal.c
    ../../components/hal/esp32s2/interrupt_descriptor_table.c
    ../../components/esp_timer/src/ets_timer_legacy.c
    ../../components/esp_timer/src/esp_timer_impl_lac.c
    ../../components/esp_timer/src/esp_timer.c
    ../../components/driver/periph_ctrl.c
    ../../components/log/log_noos.c
    ../../components/log/log.c
    ../esp_shared/src/common/esp_system_api.c
    src/heap_caps.c
    )

  ## WIFI definitions
  if (CONFIG_WIFI_ESP32)

    #TODO: Additional WPA supplicant feature like Enterprise mode etc. are yet to be supported.
    set(WPA_SUPPLICANT_SRCS "src/wpa_supplicant/port/os_xtensa.c"
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
                        "../../components/wpa_supplicant/src/crypto/bignum.c"
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
                        "../../components/wpa_supplicant/src/crypto/sha1.c"
                        "../../components/wpa_supplicant/src/crypto/sha256-internal.c"
                        "../../components/wpa_supplicant/src/crypto/sha256.c"
                        "../../components/wpa_supplicant/src/esp_supplicant/esp_hostap.c"
                        "../../components/wpa_supplicant/src/esp_supplicant/esp_wpa_main.c"
                        "../../components/wpa_supplicant/src/esp_supplicant/esp_wpas_glue.c"
                        "../../components/wpa_supplicant/src/esp_supplicant/esp_wpa3.c"
                        "../../components/wpa_supplicant/src/rsn_supp/pmksa_cache.c"
                        "../../components/wpa_supplicant/src/rsn_supp/wpa.c"
                        "../../components/wpa_supplicant/src/rsn_supp/wpa_ie.c"
                        "../../components/wpa_supplicant/src/utils/base64.c"
                        "../../components/wpa_supplicant/src/utils/common.c"
                        "../../components/wpa_supplicant/src/utils/ext_password.c"
                        "../../components/wpa_supplicant/src/utils/uuid.c"
                        "../../components/wpa_supplicant/src/utils/wpabuf.c"
                        "../../components/wpa_supplicant/src/utils/wpa_debug.c"
                        "../../components/wpa_supplicant/src/utils/json.c"
                        )

    zephyr_sources(
      src/wifi/esp_wifi_adapter.c
      ../../components/esp_wifi/src/phy_init.c
      ${WPA_SUPPLICANT_SRCS}
      )

    zephyr_compile_definitions(
      __ets__
      ESP_PLATFORM
      ESP_SUPPLICANT
      IEEE8021X_EAPOL
      EAP_PEER_METHOD
      EAP_MSCHAPv2
      EAP_PEAP
      USE_WPA2_TASK
      ESPRESSIF_USE
      ESP32_WORKAROUND
      CONFIG_ECC
      CONFIG_IEEE80211W
      CONFIG_WPA3_SAE
      CONFIG_SHA256
      CONFIG_DPP
      )

    zephyr_link_libraries(
      net80211
      core
      pp
      coexist
      phy
      rtc
      mbedTLS
      ## esp-idf wifi libs refer gcc libs symbols, so linked in libgcc
      gcc
        -L${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp32s2
        -L${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_wifi/lib/esp32s2
        -Wl,--defsym,phy_printf=printk
        -Wl,--defsym,net80211_printf=printk
        -Wl,--defsym,coexist_printf=printk
      )

  endif() ## CONFIG_WIFI_ESP32

  zephyr_link_libraries_ifdef(CONFIG_NEWLIB_LIBC c)

endif()
