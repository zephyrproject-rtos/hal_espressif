# SPDX-License-Identifier: Apache-2.0
if(CONFIG_HAS_ESPRESSIF_HAL)
  list(APPEND LIBRARIES "coexist" "core" "net80211" "phy" "pp")

  if(CONFIG_SOC_ESP32)
    list(APPEND LIBRARIES "btdm_app" "rtc")
  endif()

  if(CONFIG_SOC_ESP32C3 OR CONFIG_SOC_ESP32S3)
    list(APPEND LIBRARIES "btbb" "btdm_app")
  endif()

  foreach(lib ${LIBRARIES})
    find_library(${lib}_FOUND ${lib} HINTS "${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/zephyr/blobs/lib/${CONFIG_SOC}")

    if(NOT ${lib}_FOUND)
      message(FATAL_ERROR
        "ESP32 RF libraries not found.\n"
        "Run the command below to download the necessary files:\n"
        "> west blobs fetch hal_espressif")
      break()
    endif()
  endforeach()

  add_subdirectory_ifdef(CONFIG_SOC_ESP32 esp32)
  add_subdirectory_ifdef(CONFIG_SOC_ESP32_NET esp32)
  add_subdirectory_ifdef(CONFIG_SOC_ESP32C3 esp32c3)
  add_subdirectory_ifdef(CONFIG_SOC_ESP32S2 esp32s2)
  add_subdirectory_ifdef(CONFIG_SOC_ESP32S3 esp32s3)

endif()
