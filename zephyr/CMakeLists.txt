# At the moment there is only one supported ESP32 SoC, and it is not
# supported to omit the esp-dif HAL library, so we don't check for a
# KConfig option to enable the HAL
#
# In the future it might look like this:
# if(CONFIG_ESP_DIF_LIBRARY)
# to allow users to disable the HAL

if(CONFIG_SOC_ESP32)
  zephyr_include_directories(
    ../components/esp_common/include
    ../components/esp_rom/include
    ../components/esp32/include
    ../components/soc/esp32/include
    ../components/xtensa/include
    ../components/xtensa/esp32/include
    ../components/soc/soc/include
    ../components/soc/soc/esp32/include
    ../components/soc/soc/esp32/private_include
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
    adapter/include/crypto
    )

  zephyr_link_libraries(
    hal
    -L${CMAKE_CURRENT_SOURCE_DIR}/../components/xtensa/esp32
    )

  zephyr_sources_ifdef(
    CONFIG_SOC_FLASH_ESP32
    ../components/spi_flash/flash_mmap.c
    ../components/spi_flash/esp32/spi_flash_rom_patch.c
    ../components/spi_flash/flash_ops.c
    )

  zephyr_library_sources_ifdef(CONFIG_TIMER_ESP32   ../components/soc/src/hal/timer_hal.c) 
  
  zephyr_sources_ifdef(
    CONFIG_ESP32_SPIM   
    ../components/soc/src/hal/spi_hal.c
    ../components/soc/src/hal/spi_hal_iram.c
    ../components/soc/src/lldesc.c
    ../components/soc/soc/esp32/spi_periph.c
    )

  zephyr_sources(
    ../components/soc/src/esp32/rtc_clk.c
    ../components/esp_timer/src/ets_timer_legacy.c
    ../components/esp_timer/src/esp_timer.c
    ../components/esp_timer/src/esp_timer_impl_frc_legacy.c
    ../components/driver/periph_ctrl.c
    ../components/log/log_noos.c
    ../components/log/log.c
    adapter/src/stubs.c
    )

  if (CONFIG_WIFI_ESP32)

    #TODO: Additional WPA supplicant feature like Enterprise mode etc. are yet to be supported.
    set(WPA_SUPPLICANT_SRCS "../components/wpa_supplicant/port/os_xtensa.c"
                        "../components/wpa_supplicant/src/ap/ap_config.c"
                        "../components/wpa_supplicant/src/ap/ieee802_1x.c"
                        "../components/wpa_supplicant/src/ap/wpa_auth.c"
                        "../components/wpa_supplicant/src/ap/wpa_auth_ie.c"
                        "../components/wpa_supplicant/src/common/sae.c"
                        "../components/wpa_supplicant/src/common/wpa_common.c"
                        "../components/wpa_supplicant/src/crypto/aes-ctr.c"
                        "../components/wpa_supplicant/src/crypto/aes-siv.c"
                        "../components/wpa_supplicant/src/crypto/sha256-kdf.c"
                        "../components/wpa_supplicant/src/crypto/sha256-prf.c"
                        "../components/wpa_supplicant/src/crypto/aes-cbc.c"
                        "../components/wpa_supplicant/src/crypto/aes-ccm.c"
                        "../components/wpa_supplicant/src/crypto/aes-internal-dec.c"
                        "../components/wpa_supplicant/src/crypto/aes-internal-enc.c"
                        "../components/wpa_supplicant/src/crypto/aes-internal.c"
                        "../components/wpa_supplicant/src/crypto/aes-omac1.c"
                        "../components/wpa_supplicant/src/crypto/aes-unwrap.c"
                        "../components/wpa_supplicant/src/crypto/aes-wrap.c"
                        "../components/wpa_supplicant/src/crypto/aes-omac1.c"
                        "../components/wpa_supplicant/src/crypto/sha256-tlsprf.c"
                        "../components/wpa_supplicant/src/crypto/bignum.c"
                        "../components/wpa_supplicant/src/crypto/ccmp.c"
                        "../components/wpa_supplicant/src/crypto/crypto_mbedtls-bignum.c"
                        "../components/wpa_supplicant/src/crypto/crypto_mbedtls-ec.c"
                        "../components/wpa_supplicant/src/crypto/crypto_ops.c"
                        "../components/wpa_supplicant/src/crypto/crypto_internal-cipher.c"
                        "../components/wpa_supplicant/src/crypto/crypto_internal-modexp.c"
                        "../components/wpa_supplicant/src/crypto/crypto_internal-rsa.c"
                        "../components/wpa_supplicant/src/crypto/crypto_internal.c"
                        "../components/wpa_supplicant/src/crypto/des-internal.c"
                        "../components/wpa_supplicant/src/crypto/dh_group5.c"
                        "../components/wpa_supplicant/src/crypto/dh_groups.c"
                        "../components/wpa_supplicant/src/crypto/md4-internal.c"
                        "../components/wpa_supplicant/src/crypto/md5-internal.c"
                        "../components/wpa_supplicant/src/crypto/md5.c"
                        "../components/wpa_supplicant/src/crypto/ms_funcs.c"
                        "../components/wpa_supplicant/src/crypto/rc4.c"
                        "../components/wpa_supplicant/src/crypto/sha1-internal.c"
                        "../components/wpa_supplicant/src/crypto/sha1-pbkdf2.c"
                        "../components/wpa_supplicant/src/crypto/sha1.c"
                        "../components/wpa_supplicant/src/crypto/sha256-internal.c"
                        "../components/wpa_supplicant/src/crypto/sha256.c"
                        "../components/wpa_supplicant/src/esp_supplicant/esp_hostap.c"
                        "../components/wpa_supplicant/src/esp_supplicant/esp_wpa_main.c"
                        "../components/wpa_supplicant/src/esp_supplicant/esp_wpas_glue.c"
                        "../components/wpa_supplicant/src/esp_supplicant/esp_wpa3.c"
                        "../components/wpa_supplicant/src/rsn_supp/pmksa_cache.c"
                        "../components/wpa_supplicant/src/rsn_supp/wpa.c"
                        "../components/wpa_supplicant/src/rsn_supp/wpa_ie.c"
                        "../components/wpa_supplicant/src/utils/base64.c"
                        "../components/wpa_supplicant/src/utils/common.c"
                        "../components/wpa_supplicant/src/utils/ext_password.c"
                        "../components/wpa_supplicant/src/utils/uuid.c"
                        "../components/wpa_supplicant/src/utils/wpabuf.c"
                        "../components/wpa_supplicant/src/utils/wpa_debug.c"
                        "../components/wpa_supplicant/src/utils/json.c"
                        )

    zephyr_sources(
      ../components/esp_wifi/src/phy_init.c
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
      net80211
      core
      net80211
      pp
      coexist
      phy
      rtc
      mesh
      mbedTLS

      ## esp-idf wifi libs refer gcc libs symbols, so linked in libgcc
      gcc
        -L${CMAKE_CURRENT_SOURCE_DIR}/../components/esp32
        -L${CMAKE_CURRENT_SOURCE_DIR}/../components/esp_wifi/lib/esp32
        -L${CMAKE_CURRENT_SOURCE_DIR}/../components/wpa_supplicant
        -Wl,--defsym,phy_printf=printf
        -Wl,--defsym,net80211_printf=printf
        -Wl,--defsym,coexist_printf=printf
      )

  endif()

endif()
