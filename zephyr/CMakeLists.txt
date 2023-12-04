# SPDX-License-Identifier: Apache-2.0

set(LIBS_BLOBS "")

# Function to check the status of the blobs
function(check_status blobs_output )
  set(NOT_PRESENT_FILES "")
  set(OUTDATED_FILES "")

  foreach(blob ${blobs_output})
    # Split each blob into words
    string(REPLACE " " ";" words ${blob})

    # Get the status and the path
    list(GET words 1 status)
    list(GET words 2 blob_path)

    # Check if the file is not present or outdated
    if(status STREQUAL "D")
      list(APPEND NOT_PRESENT_FILES ${blob_path})
    elseif(status STREQUAL "M")
      list(APPEND OUTDATED_FILES ${blob_path})
    endif()
  endforeach()

  # Print warning if necessary
  if(NOT "${NOT_PRESENT_FILES}" STREQUAL "")
    string(REPLACE ";" "\n" NOT_PRESENT_FILES "${NOT_PRESENT_FILES}")
    message(WARNING
      "The following files are not present in the blobs repository:\n"
      "${NOT_PRESENT_FILES}\n"
      "Run the command below to download the necessary files:\n"
      "> west blobs fetch hal_espressif")
  endif()

  if(NOT "${OUTDATED_FILES}" STREQUAL "")
    string(REPLACE ";" "\n" OUTDATED_FILES "${OUTDATED_FILES}")
    message(WARNING
      "The following files are outdated in the blobs repository:\n"
      "${OUTDATED_FILES}"
      "Run the command below to download the necessary files:\n"
      "> west blobs fetch hal_espressif")
  endif()
endfunction()

# Get the status of the blobs
execute_process(
  COMMAND west blobs list hal_espressif
  RESULT_VARIABLE CMD_RESULT
  OUTPUT_VARIABLE CMD_OUTPUT
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(CMD_RESULT EQUAL 0)
  # Split the output into lines
  string(REPLACE "\n" ";" BLOBS_STATUS_LIST ${CMD_OUTPUT})

  foreach(blob_status ${BLOBS_STATUS_LIST})
    # Split each line into words
    string(REPLACE " " ";" item ${blob_status})

    list(GET item 2 blob_path)

    # Check if the file is a library
    if(blob_path MATCHES "lib/${CONFIG_SOC_SERIES}")
      list(APPEND LIBS_BLOBS ${blob_status})
    endif()
  endforeach()
else()
  message(FATAL_ERROR "Failed to run command: west blobs list hal_espressif")
endif()

# check for RF libraries in case ESP32 family is used and BT or WIFI are enabled
if(CONFIG_BT_ESP32 OR CONFIG_WIFI_ESP32)
  check_status("${LIBS_BLOBS}")
endif()

add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32 esp32)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32_NET esp32)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32C3 esp32c3)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32S2 esp32s2)
add_subdirectory_ifdef(CONFIG_SOC_SERIES_ESP32S3 esp32s3)
