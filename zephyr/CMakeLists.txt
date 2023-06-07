# SPDX-License-Identifier: Apache-2.0

# check for RF libraries in case ESP32 family is used and BT or WIFI are enabled
if(CONFIG_BT_ESP32 OR CONFIG_WIFI_ESP32)
  list(APPEND LIBRARIES "core" "net80211" "phy" "pp" "espnow" "mesh" "wapi")

  if(CONFIG_SOC_SERIES_ESP32)
    list(APPEND LIBRARIES "btdm_app" "rtc")
  endif()

  if(CONFIG_SOC_SERIES_ESP32C3 OR CONFIG_SOC_SERIES_ESP32S3)
    list(APPEND LIBRARIES "btbb" "btdm_app" "bttestmode" "rftest" "smartconfig")
  endif()

  foreach(lib ${LIBRARIES})
    find_library(${lib}_FOUND ${lib} HINTS
      "${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/zephyr/blobs/lib/${CONFIG_SOC_SERIES}")

    if(NOT ${lib}_FOUND)
      message(FATAL_ERROR
        "ESP32 RF libraries not found.\n"
        "Run the command below to download the necessary files:\n"
        "> west blobs fetch hal_espressif")
      break()
    endif()
  endforeach()
endif()

add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32 esp32)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32C3 esp32c3)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32C6 esp32c6)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32S2 esp32s2)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32S3 esp32s3)
