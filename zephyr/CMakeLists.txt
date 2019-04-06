# At the moment there is only one supported ESP32 SoC, and it is not
# supported to omit the esp-dif HAL library, so we don't check for a
# KConfig option to enable the HAL
#
# In the future it might look like this:
# if(CONFIG_ESP_DIF_LIBRARY)
# to allow users to disable the HAL
if(CONFIG_SOC_ESP32)
  zephyr_include_directories(
    ../components/esp32/include
    ../components/soc/esp32/include
    )

  zephyr_link_libraries(
    hal
    -L${CMAKE_CURRENT_SOURCE_DIR}/../components/esp32
    )
endif()
