idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    set(ldfragments)
else()
    set(ldfragments common.lf soc.lf)
endif()

set(srcs "src/esp_err_to_name.c")

idf_component_register(SRCS "${srcs}"
                        INCLUDE_DIRS include
                        LDFRAGMENTS ${ldfragments})

# Note: LINK_INTERFACE_MULTIPLICITY is needed to break circular static-library
# dependencies in the component graph. The core cycle is:
#   esp_system -> bootloader_support -> efuse -> esp_system
# (efuse registers an ESP_SYSTEM_INIT_FN and calls esp_restart(), both of which
# pull in esp_system).  Several more cycles exist through esp_partition, esp_mm,
# and mbedtls.  All cycle-causing deps are already PRIV_REQUIRES, so the
# multiplicity property on this universal dependency is the established fix.
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY LINK_INTERFACE_MULTIPLICITY 4)

# List of components needed for the error codes list
set(optional_reqs ulp
                    efuse
                    esp_http_client
                    esp_http_server
                    bootloader_support
                    nvs_flash
                    esp_wifi
                    app_update
                    lwip
                    spi_flash
                    wpa_supplicant
                    esp_serial_slave_link
                    esp_netif
                    soc
                    esp-tls
                    esp_https_ota
                    esp_hal_mspi
                    esp_hw_support)

if(IDF_BUILD_V2)
    idf_component_optional_requires(PRIVATE ${optional_reqs})
else()
    idf_build_get_property(build_components BUILD_COMPONENTS)
    foreach(req ${optional_reqs})
        if(req IN_LIST build_components)
            idf_component_get_property(req_lib ${req} COMPONENT_LIB)
            target_link_libraries(${COMPONENT_LIB} PRIVATE ${req_lib})
        endif()
    endforeach()
endif()
