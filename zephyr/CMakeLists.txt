# SPDX-License-Identifier: Apache-2.0

if(CONFIG_MCUBOOT_ESPRESSIF)
  return()
endif()

add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32 esp32)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32C2 esp32c2)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32C3 esp32c3)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32C6 esp32c6)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32H2 esp32h2)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32S2 esp32s2)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32S3 esp32s3)
