# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_ESP32 OR CONFIG_SOC_ESP32_NET)

  zephyr_compile_options(-Wno-unused-variable -Wno-maybe-uninitialized)

  zephyr_link_libraries_ifdef(CONFIG_NEWLIB_LIBC c)

  zephyr_include_directories(
    include
    include/bt
    include/crypto
    ../esp_shared/include
    ../esp_shared/include/wifi
    ../../components/hal/include
    ../../components/hal/esp32/include
    ../../components/esp_hw_support/include
    ../../components/esp_hw_support/port/esp32
    ../../components/esp_hw_support/port/esp32/private_include
    ../../components/esp_common/include
    ../../components/esp_rom/include
    ../../components/esp_rom/esp32/ld
    ../../components/esp32/include
    ../../components/soc/esp32/include
    ../../components/xtensa/include
    ../../components/xtensa/esp32/include
    ../../components/soc/include/soc
    ../../components/soc/include
    ../../components/soc/src/esp32/include
    ../../components/driver/include
    ../../components/esp_wifi/include
    ../../components/efuse/esp32/include
    ../../components/efuse/include
    ../../components/esp_system/include
    ../../components/esp_wifi/esp32/include
    ../../components/esp_timer/include
    ../../components/esp_timer/private_include
    ../../components/esp_netif/include
    ../../components/esp_event/include
    ../../components/log/include
    ../../components/wpa_supplicant/include
    ../../components/wpa_supplicant/port/include
    ../../components/wpa_supplicant/src
    ../../components/wpa_supplicant/include/esp_supplicant
    ../../components/wpa_supplicant/src/crypto
    ../../components/wpa_supplicant/src
    ../../components/spi_flash/include
    ../../components/spi_flash/private_include
    ../../components/bootloader_support/include
    ../../components/bootloader_support/include_bootloader
    )

  zephyr_link_libraries(
    gcc
      -T${CMAKE_CURRENT_SOURCE_DIR}/src/linker/esp32.rom.alias.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32/ld/esp32.rom.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32/ld/esp32.rom.api.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32/ld/esp32.rom.libgcc.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/esp32/ld/esp32.rom.redefined.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp32/ld/esp32.peripherals.ld
    )

  zephyr_library_sources(
    src/hal/windowspill_asm.S
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
    ../../components/spi_flash/esp32/spi_flash_rom_patch.c
    ../../components/spi_flash/flash_ops.c
    )

  zephyr_library_sources_ifdef(CONFIG_COUNTER_ESP32     ../../components/hal/timer_hal.c)

  zephyr_sources_ifdef(
    CONFIG_ESP32_SPIM
    ../../components/hal/spi_hal.c
    ../../components/hal/spi_hal_iram.c
    ../../components/soc/lldesc.c
    ../../components/soc/esp32/spi_periph.c
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
    CONFIG_ESP_SPIRAM
    ../esp_shared/src/host_flash/cache_utils.c
    ../../components/spi_flash/flash_ops.c
    ../../components/esp32/spiram.c
    ../../components/esp32/spiram_psram.c
    ../../components/esp32/cache_sram_mmu.c
    ../../components/bootloader_support/src/bootloader_common.c
    )

  zephyr_sources_ifdef(
    CONFIG_UART_ESP32
    ../../components/soc/esp32/uart_periph.c
    ../../components/hal/uart_hal.c
    ../../components/hal/uart_hal_iram.c
    )

  zephyr_sources(
    ../../components/soc/esp32/gpio_periph.c
    ../../components/soc/esp32/rtc_io_periph.c
    ../../components/esp_hw_support/port/esp32/rtc_clk.c
    ../../components/esp_hw_support/port/esp32/rtc_init.c
    ../../components/esp_hw_support/port/esp32/rtc_time.c
    ../../components/esp_hw_support/port/esp32/rtc_clk_init.c
    ../../components/esp32/clk.c
    ../../components/esp_timer/src/ets_timer_legacy.c
    ../../components/esp_timer/src/esp_timer.c
    ../../components/esp_timer/src/esp_timer_impl_lac.c
    ../../components/driver/periph_ctrl.c
    ../../components/log/log_noos.c
    ../../components/log/log.c
    ../../components/hal/interrupt_controller_hal.c
    ../../components/hal/esp32/interrupt_descriptor_table.c
    ../esp_shared/src/common/esp_system_api.c
    src/common/dport_access.c
    src/stubs.c
    src/heap_caps.c
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
      rtc
      ## esp-idf wifi libs refer gcc libs symbols, so linked in libgcc
      gcc
        -L${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp32
        -L${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_wifi/lib/esp32
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
      btdm_app
        -L${CMAKE_CURRENT_SOURCE_DIR}/../../components/bt/controller/lib_esp32/esp32
      coexist
        -L${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_wifi/lib/esp32

      )

  endif()

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
      ## wifi
      mbedTLS
      )

  endif()

endif()
