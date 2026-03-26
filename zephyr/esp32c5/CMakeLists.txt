# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_SERIES_ESP32C5)

  zephyr_compile_options(-fstrict-volatile-bitfields)
  zephyr_compile_definitions_ifndef(CONFIG_MCUBOOT CONFIG_APP_BUILD_USE_FLASH_SECTIONS)
  zephyr_compile_definitions_ifndef(asm asm=__asm__)
  zephyr_compile_definitions_ifndef(typeof typeof=__typeof__)

  if(CONFIG_MCUBOOT)
    zephyr_compile_options(-fdump-rtl-expand)
  endif()

  zephyr_include_directories(
    include
    include/bt
    ../common/include
    ../port/include
    ../port/include/boot
    ../../components/bootloader_support/bootloader_flash/include
    ../../components/bootloader_support/include
    ../../components/bootloader_support/private_include
    ../../components/bt/controller/${CONFIG_SOC_SERIES}
    ../../components/bt/include/${CONFIG_SOC_SERIES}/include
    ../../components/bt/porting/include
    ../../components/bt/porting/npl/zephyr/include
    ../../components/bt/porting/npl/zephyr/include/nimble
    ../../components/bt/porting/transport/include
    ../../components/driver/deprecated
    ../../components/driver/gpio/include
    ../../components/driver/include
    ../../components/driver/spi/include
    ../../components/driver/uart/include
    ../../components/efuse/include
    ../../components/efuse/private_include
    ../../components/efuse/${CONFIG_SOC_SERIES}/include
    ../../components/efuse/${CONFIG_SOC_SERIES}/private_include
    ../../components/esp_coex/include
    ../../components/esp_common/include
    ../../components/esp_driver_gpio/include
    ../../components/esp_event/include
    ../../components/esp_hal_ana_conv/include
    ../../components/esp_hal_ana_conv/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_clock/include
    ../../components/esp_hal_clock/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_dma/include
    ../../components/esp_hal_dma/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_gpio/include
    ../../components/esp_hal_gpio/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_gpspi/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_gpspi/include
    ../../components/esp_hal_i2c/include
    ../../components/esp_hal_i2c/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_i2s/include
    ../../components/esp_hal_i2s/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_ieee802154/include
    ../../components/esp_hal_ieee802154/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_ledc/include
    ../../components/esp_hal_ledc/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_mcpwm/include
    ../../components/esp_hal_mcpwm/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_mspi/include
    ../../components/esp_hal_mspi/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_pcnt/include
    ../../components/esp_hal_pcnt/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_pmu/include
    ../../components/esp_hal_pmu/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_rtc_timer/include
    ../../components/esp_hal_rtc_timer/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_sd/include
    ../../components/esp_hal_sd/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_security/include
    ../../components/esp_hal_security/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_systimer/include
    ../../components/esp_hal_systimer/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_timg/include
    ../../components/esp_hal_timg/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_twai/include
    ../../components/esp_hal_twai/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_uart/include
    ../../components/esp_hal_uart/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_usb/include
    ../../components/esp_hal_usb/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_wdt/include
    ../../components/esp_hal_wdt/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hw_support/dma
    ../../components/esp_hw_support/etm/include
    ../../components/esp_hw_support/include
    ../../components/esp_hw_support/include/esp_private
    ../../components/esp_hw_support/include/hal
    ../../components/esp_hw_support/include/soc
    ../../components/esp_hw_support/include/soc/${CONFIG_SOC_SERIES}
    ../../components/esp_hw_support/ldo/include
    ../../components/esp_hw_support/modem/include
    ../../components/esp_hw_support/mspi/mspi_intr/include
    ../../components/esp_hw_support/mspi/mspi_timing_tuning/include
    ../../components/esp_hw_support/mspi/mspi_timing_tuning/port/${CONFIG_SOC_SERIES}
    ../../components/esp_hw_support/mspi/mspi_timing_tuning/tuning_scheme_impl/include
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/private_include
    ../../components/esp_hw_support/port/include
    ../../components/esp_hw_support/power_supply/include
    ../../components/esp_mm/include
    ../../components/esp_netif/include
    ../../components/esp_phy/include
    ../../components/esp_phy/${CONFIG_SOC_SERIES}/include
    ../../components/esp_phy/include/esp_private
    ../../components/esp_pm/include
    ../../components/esp_psram/device/include
    ../../components/esp_psram/include
    ../../components/esp_psram/xip_impl/include
    ../../components/esp_rom/${CONFIG_SOC_SERIES}
    ../../components/esp_rom/${CONFIG_SOC_SERIES}/include
    ../../components/esp_rom/include
    ../../components/esp_rom/${CONFIG_SOC_SERIES}/include/${CONFIG_SOC_SERIES}
    ../../components/esp_rom/${CONFIG_SOC_SERIES}/ld
    ../../components/esp_system/include
    ../../components/esp_system/include/esp_private
    ../../components/esp_system/port/include
    ../../components/esp_system/port/include/private
    ../../components/esp_system/port/include/private/esp_private
    ../../components/esp_timer/include
    ../../components/esp_timer/private_include
    ../../components/esp_wifi/include
    ../../components/esp_wifi/include/local
    ../../components/hal/${CONFIG_SOC_SERIES}/include
    ../../components/hal/include
    ../../components/hal/platform_port/include
    ../../components/heap/include
    ../../components/ieee802154/include
    ../../components/ieee802154/private_include
    ../../components/mbedtls/port/include
    ../../components/mbedtls/port/psa_driver/include
    ../../components/riscv/${CONFIG_SOC_SERIES}/include
    ../../components/riscv/include
    ../../components/riscv/include/esp_private
    ../../components/riscv/include/xtensa
    ../../components/soc/${CONFIG_SOC_SERIES}/include
    ../../components/soc/include
    ../../components/soc/${CONFIG_SOC_SERIES}/ld
    ../../components/soc/${CONFIG_SOC_SERIES}/register
    ../../components/spi_flash/include
    ../../components/spi_flash/include/esp_private
    ../../components/ulp/lp_core/include
    ../../components/ulp/lp_core/lp_core/include
    ../../components/ulp/lp_core/shared/include
    ../../components/ulp/ulp_common/include
    ../../components/wpa_supplicant/esp_supplicant/include
    ../../components/wpa_supplicant/esp_supplicant/src
    ../../components/wpa_supplicant/include
    ../../components/wpa_supplicant/include/esp_supplicant
    ../../components/wpa_supplicant/port/include
    ../../components/wpa_supplicant/src
    ../../components/wpa_supplicant/src/crypto
    ../../components/wpa_supplicant/src/eap_peer
    ../../components/wpa_supplicant/src/utils
  )

  zephyr_link_libraries(
    gcc
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/soc/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.peripherals.ld
  )

  if(NOT CONFIG_SOC_ESP32C5_LPCORE)
    zephyr_link_libraries(
      -T${CMAKE_CURRENT_SOURCE_DIR}/src/linker/${CONFIG_SOC_SERIES}.rom.alias.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.api.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.rvfp.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.newlib.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.version.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.libc.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.libc-suboptimal_for_misaligned_mem.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.libgcc.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_hal_wdt/${CONFIG_SOC_SERIES}/rom.wdt.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/riscv/ld/rom.api.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../tools/flasher_stub/ld/rom_32c5.ld
    )
  endif()

  zephyr_compile_definitions(ESP_PLATFORM)

  zephyr_sources(
    ../../components/efuse/src/esp_efuse_api.c
    ../../components/efuse/src/esp_efuse_fields.c
    ../../components/efuse/src/esp_efuse_utility.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_table.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_fields.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_utility.c
    ../../components/efuse/src/efuse_controller/keys/with_key_purposes/esp_efuse_api_key.c
  )

  zephyr_sources_ifdef(CONFIG_ESP32_SPIM
    ../../components/esp_hal_gpspi/spi_hal.c
    ../../components/esp_hal_gpspi/spi_hal_iram.c
    ../../components/soc/lldesc.c
    ../../components/esp_hal_dma/gdma_hal_top.c
    )

  if(CONFIG_SOC_FLASH_ESP32 OR NOT CONFIG_BOOTLOADER_MCUBOOT)
    zephyr_sources(
    ../../components/esp_mm/esp_mmu_map.c
    ../../components/esp_mm/esp_cache_utils.c
    ../../components/esp_mm/port/${CONFIG_SOC_SERIES}/ext_mem_layout.c
    ../../components/bootloader_support/src/flash_encrypt.c
    ../../components/esp_hw_support/esp_gpio_reserve.c
    ../../components/hal/cache_hal.c
    ../../components/hal/mmu_hal.c
    ../../components/esp_hal_mspi/spi_flash_encrypt_hal_iram.c
    ../../components/esp_hal_mspi/spi_flash_hal.c
    ../../components/esp_hal_mspi/spi_flash_hal_iram.c
    ../../components/esp_hal_mspi/spi_flash_hal_gpspi.c
    ../../components/spi_flash/esp_flash_api.c
    ../../components/spi_flash/esp_flash_spi_init.c
    ../../components/spi_flash/flash_mmap.c
    ../../components/spi_flash/flash_ops.c
    ../../components/spi_flash/memspi_host_driver.c
    ../../components/spi_flash/spi_flash_chip_boya.c
    ../../components/spi_flash/spi_flash_chip_drivers.c
    ../../components/spi_flash/spi_flash_chip_gd.c
    ../../components/spi_flash/spi_flash_chip_generic.c
    ../../components/spi_flash/spi_flash_chip_issi.c
    ../../components/spi_flash/spi_flash_chip_mxic.c
    ../../components/spi_flash/spi_flash_chip_mxic_opi.c
    ../../components/spi_flash/spi_flash_chip_th.c
    ../../components/spi_flash/spi_flash_chip_winbond.c
    ../../components/spi_flash/spi_flash_os_func_noos.c
    ../../components/spi_flash/spi_flash_os_func_app.c
    ../../components/esp_hw_support/mspi/mspi_timing_tuning/mspi_timing_tuning.c
    ../../components/esp_hw_support/mspi/mspi_timing_tuning/port/${CONFIG_SOC_SERIES}/mspi_timing_config.c
    ../../components/esp_hw_support/mspi/mspi_timing_tuning/tuning_scheme_impl/mspi_timing_by_mspi_delay.c
    )
  endif()

  if(CONFIG_PM OR CONFIG_POWEROFF)
  zephyr_sources(
    ../../components/esp_driver_gpio/src/gpio.c
    ../../components/esp_driver_gpio/src/rtc_io.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_sleep.c
    ../../components/esp_hw_support/port/pau_regdma.c
    ../../components/esp_hw_support/lowpower/port/${CONFIG_SOC_SERIES}/sleep_clock.c
    ../../components/esp_hw_support/lowpower/port/${CONFIG_SOC_SERIES}/sleep_cpu.c
    ../../components/esp_hw_support/lowpower/port/${CONFIG_SOC_SERIES}/sleep_cpu_asm.S
    ../../components/esp_hw_support/sleep_gpio.c
    ../../components/esp_hw_support/sleep_event.c
    ../../components/esp_hw_support/sleep_console.c
    ../../components/esp_hw_support/sleep_modem.c
    ../../components/esp_hw_support/sleep_modes.c
    ../../components/esp_hw_support/sleep_retention.c
    ../../components/esp_hw_support/sleep_system_peripheral.c
    ../../components/esp_hal_pmu/${CONFIG_SOC_SERIES}/pau_hal.c
    ../../components/esp_hal_pmu/${CONFIG_SOC_SERIES}/pmu_hal.c
    ../../components/esp_hal_gpio/rtc_io_hal.c
    ../../components/esp_hw_support/sleep_uart.c
    ../../components/esp_hw_support/clk_utils.c
    ../../components/esp_hw_support/esp_memory_utils.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/peripheral_domain_pd.c
    ../common/pm_impl_zephyr.c
    )
  endif()

  zephyr_sources_ifdef(CONFIG_DMA_ESP32
    ../../components/soc/lldesc.c
    ../../components/esp_hal_dma/gdma_hal_top.c
    ../../components/esp_hal_dma/gdma_hal_ahb_v2.c
    )

  if (CONFIG_ESP_SPIRAM)
    zephyr_compile_definitions(CONFIG_SPIRAM)
    zephyr_sources(
      ../../components/esp_psram/system_layer/esp_psram.c
      ../../components/esp_hw_support/esp_memory_utils.c
      ../../components/esp_hw_support/esp_gpio_reserve.c
      )

    zephyr_sources_ifdef(CONFIG_SPIRAM_MODE_QUAD
      ../../components/esp_psram/device/esp_psram_impl_ap_quad.c
      )
  endif()

  if (CONFIG_MCUBOOT)
    zephyr_sources(
        ../port/boot/esp_image_loader.c
        )
  endif()

  if (CONFIG_MCUBOOT OR NOT CONFIG_BOOTLOADER_MCUBOOT)
    zephyr_include_directories(
      ../../components/esp_rom/${CONFIG_SOC_SERIES}
      )

    zephyr_sources(
      ../../components/bootloader_support/src/bootloader_clock_init.c
      ../../components/bootloader_support/bootloader_flash/src/flash_qio_mode.c
      ../common/console_init.c
      ../common/soc_init.c
      )
  endif()

  zephyr_sources_ifdef(CONFIG_SOC_ESP32C5_HPCORE
    ../../components/esp_driver_gpio/src/rtc_io.c
    ../../components/esp_hal_ana_conv/${CONFIG_SOC_SERIES}/temperature_sensor_periph.c
    ../../components/esp_hal_ana_conv/temperature_sensor_hal.c
    ../../components/esp_hal_clock/${CONFIG_SOC_SERIES}/clk_tree_hal.c
    ../../components/esp_hal_gpio/${CONFIG_SOC_SERIES}/rtc_io_periph.c
    ../../components/esp_hal_gpio/rtc_io_hal.c
    ../../components/esp_hal_systimer/systimer_hal.c
    ../../components/esp_hal_wdt/wdt_hal_iram.c
    ../../components/esp_hw_support/adc_share_hw_ctrl.c
    ../../components/esp_hw_support/clk_ctrl_os.c
    ../../components/esp_hw_support/cpu.c
    ../../components/esp_hw_support/esp_clk.c
    ../../components/esp_hw_support/hw_random.c
    ../../components/esp_hw_support/mac_addr.c
    ../../components/esp_hw_support/modem/modem_clock.c
    ../../components/esp_hw_support/modem/port/${CONFIG_SOC_SERIES}/modem_clock_impl.c
    ../../components/esp_hw_support/periph_ctrl.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/cpu_region_protect.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/esp_clk_tree.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/esp_cpu_intr.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/io_mux.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/ocode_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_param.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_pvt.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_sleep.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_clk.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_clk_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_time.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/sar_periph_ctrl.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/systimer.c
    ../../components/esp_hw_support/port/esp_clk_tree_common.c
    ../../components/esp_hw_support/port/regdma_link.c
    ../../components/esp_hw_support/regi2c_ctrl.c
    ../../components/esp_hw_support/rtc_module.c
    ../../components/esp_hw_support/sar_tsens_ctrl.c
    ../../components/esp_hw_support/sleep_modes.c
    ../../components/esp_hw_support/sleep_retention.c
    ../../components/esp_mm/esp_cache_msync.c
    ../../components/esp_rom/patches/esp_rom_crc.c
    ../../components/esp_rom/patches/esp_rom_efuse.c
    ../../components/esp_rom/patches/esp_rom_gpio.c
    ../../components/esp_rom/patches/esp_rom_hp_regi2c_esp32c5.c
    ../../components/esp_rom/patches/esp_rom_print.c
    ../../components/esp_rom/patches/esp_rom_serial_output.c
    ../../components/esp_rom/patches/esp_rom_spiflash.c
    ../../components/esp_rom/patches/esp_rom_sys.c
    ../../components/esp_system/esp_err.c
    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/clk.c
    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/reset_reason.c
    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/system_internal.c
    ../../components/esp_timer/src/esp_timer.c
    ../../components/esp_timer/src/esp_timer_etm.c
    ../../components/esp_timer/src/esp_timer_impl_common.c
    ../../components/esp_timer/src/esp_timer_impl_systimer.c
    ../../components/esp_timer/src/esp_timer_init.c
    ../../components/esp_timer/src/ets_timer_legacy.c
    ../../components/esp_timer/src/system_time.c
    ../../components/hal/${CONFIG_SOC_SERIES}/efuse_hal.c
    ../../components/hal/${CONFIG_SOC_SERIES}/modem_clock_hal.c
    ../../components/hal/efuse_hal.c
    ../../components/riscv/instruction_decode.c
    ../../components/riscv/interrupt.c
    ../../components/riscv/interrupt_clic.c
    ../../components/riscv/rv_utils.c
    ../../components/soc/${CONFIG_SOC_SERIES}/gpio_periph.c
    ../../components/spi_flash/cache_utils.c
    ../common/esp_restart.c
    ../common/flash_init.c
    ../port/bootloader/bootloader_flash.c
    ../port/heap/heap_caps_zephyr.c
    src/soc_flash_init.c
    src/soc_init.c
    src/soc_random.c
  )

  zephyr_sources_ifdef(CONFIG_I2C_ESP32
    ../../components/esp_hal_i2c/i2c_hal_iram.c
    ../../components/esp_hal_i2c/i2c_hal.c
    )

  zephyr_sources_ifdef(CONFIG_I2S_ESP32
    ../../components/esp_hal_i2s/i2s_hal.c
    ../../components/hal/hal_utils.c
    )

  zephyr_sources_ifdef(CONFIG_COUNTER_TMR_ESP32
    ../../components/esp_hal_timg/timer_hal.c
    )

  zephyr_sources_ifdef(CONFIG_UART_ESP32
    ../../components/esp_hal_uart/uart_hal.c
    ../../components/esp_hal_uart/uart_hal_iram.c
    ../../components/esp_hal_uart/${CONFIG_SOC_SERIES}/uart_periph.c
    )

  zephyr_sources_ifdef(CONFIG_PWM_LED_ESP32
    ../../components/esp_hal_ledc/${CONFIG_SOC_SERIES}/ledc_periph.c
    ../../components/esp_hal_ledc/ledc_hal_iram.c
    ../../components/esp_hal_ledc/ledc_hal.c
    )

  zephyr_sources_ifdef(CONFIG_MCPWM_ESP32
    ../../components/esp_hal_mcpwm/mcpwm_hal.c
    )

  zephyr_sources_ifdef(CONFIG_PCNT_ESP32
    ../../components/esp_hal_pcnt/pcnt_hal.c
    )

  if (CONFIG_ESP32_TEMP)
    zephyr_include_directories(
    ../../components/esp_driver_tsens/include
    )

    zephyr_sources(
    ../../components/esp_driver_tsens/src/temperature_sensor.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_rtc_calib.c
    )
  endif()

  if (CONFIG_ADC_ESP32)
    zephyr_include_directories(
    ../../components/esp_adc/${CONFIG_SOC_SERIES}/include
    ../../components/esp_adc/include
    ../../components/esp_adc/interface
    )

    zephyr_sources(
    ../../components/esp_hal_ana_conv/adc_hal.c
    ../../components/esp_hal_ana_conv/adc_hal_common.c
    ../../components/esp_hal_ana_conv/adc_oneshot_hal.c
    ../../components/esp_adc/adc_cali.c
    ../../components/esp_adc/adc_cali_curve_fitting.c
    ../../components/esp_adc/${CONFIG_SOC_SERIES}/curve_fitting_coefficients.c
    ../../components/esp_hal_ana_conv/${CONFIG_SOC_SERIES}/adc_periph.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_rtc_calib.c
    )
  endif()

  if(CONFIG_SOC_ESP32C5_LPCORE)
    zephyr_compile_definitions(IS_ULP_COCPU)
    zephyr_ld_options("-nostartfiles")
    zephyr_ld_options("-Wl,--no-warn-rwx-segments")
    zephyr_ld_options("-Wl,--gc-sections")
  endif()

  if(CONFIG_ESP32_ULP_COPROC_ENABLED)
    zephyr_sources_ifdef(CONFIG_SOC_ESP32C5_HPCORE
      ../../components/ulp/lp_core/lp_core.c
      ../../components/ulp/lp_core/shared/ulp_lp_core_lp_timer_shared.c
      ../../components/ulp/lp_core/shared/ulp_lp_core_memory_shared.c
      ../../components/esp_hal_gpio/rtc_io_hal.c
      )

    zephyr_sources_ifdef(CONFIG_SOC_ESP32C5_LPCORE
      ../../components/ulp/lp_core/lp_core/lp_core_interrupt.c
      ../../components/ulp/lp_core/lp_core/lp_core_utils.c
      ../../components/ulp/lp_core/lp_core/lp_core_print.c
      ../../components/ulp/lp_core/lp_core/lp_core_uart.c
      ../../components/ulp/lp_core/shared/ulp_lp_core_lp_timer_shared.c
      ../../components/ulp/lp_core/shared/ulp_lp_core_memory_shared.c
      ../../components/esp_hal_gpio/rtc_io_hal.c
      ../../components/esp_hal_gpio/${CONFIG_SOC_SERIES}/rtc_io_periph.c
      ../../components/esp_hal_uart/uart_hal_iram.c
      ../../components/esp_hal_uart/uart_hal.c
      )
  endif()

  ## shared WIFI/BT/IEEE802154 resources
  if (CONFIG_BT OR CONFIG_WIFI_ESP32 OR CONFIG_IEEE802154)
    zephyr_compile_definitions(CONFIG_ESP_COEX_ENABLED)

    zephyr_sources(
      ../../components/esp_phy/src/phy_init.c
      ../../components/esp_phy/src/lib_printf.c
      ../../components/esp_phy/src/phy_common.c
      ../../components/esp_phy/src/phy_override.c
      ../../components/esp_phy/${CONFIG_SOC_SERIES}/phy_init_data.c
      ../../components/esp_coex/${CONFIG_SOC_SERIES}/esp_coex_adapter.c
      )

    if(CONFIG_ESP32_SW_COEXIST_ENABLE)
      zephyr_sources(../../components/esp_coex/src/lib_printf.c)
      zephyr_sources_ifdef(CONFIG_BUILD_ONLY_NO_BLOBS ../port/stubs/coex_stubs.c)

      zephyr_link_libraries_ifndef(CONFIG_BUILD_ONLY_NO_BLOBS
        coexist
          -L${CMAKE_CURRENT_SOURCE_DIR}/../blobs/lib/${CONFIG_SOC_SERIES}
          -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.coexist.ld
        )
    endif()

    zephyr_link_libraries_ifndef(CONFIG_BUILD_ONLY_NO_BLOBS
      net80211
      core
      pp
      phy
      mesh
      ## esp-idf wifi libs refer gcc libs symbols, so linked in libgcc
      gcc
        -L${CMAKE_CURRENT_SOURCE_DIR}/../blobs/lib/${CONFIG_SOC_SERIES}
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.pp.ld
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.phy.ld
        -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.net80211.ld
      )
  endif()

  ## BT definitions
  if (CONFIG_BT)

    zephyr_sources(
      ../../components/esp_phy/src/btbb_init.c
      ../../components/bt/porting/mem/bt_osi_mem.c
      ../../components/bt/porting/mem/os_msys_init.c
      ../../components/bt/porting/npl/zephyr/src/npl_os_zephyr.c
      ../../components/bt/porting/transport/driver/vhci/hci_driver_standard.c
      ../../components/bt/porting/transport/src/hci_transport.c
      ../../components/bt/controller/${CONFIG_SOC_SERIES}/ble.c
      ../../components/bt/controller/${CONFIG_SOC_SERIES}/bt.c
      )

    zephyr_compile_definitions(CONFIG_BT_ENABLED=1)
    zephyr_compile_definitions(CONFIG_BT_CONTROLLER_ENABLED=1)

    zephyr_sources_ifdef(CONFIG_BUILD_ONLY_NO_BLOBS
      ../port/stubs/bt_stubs.c
      ../port/stubs/phy_stubs.c
      )

    zephyr_link_libraries_ifndef(CONFIG_BUILD_ONLY_NO_BLOBS
      ## ble
      ble_app
      btbb
        -L${CMAKE_CURRENT_SOURCE_DIR}/../blobs/lib/${CONFIG_SOC_SERIES}
      )

  endif()

  ## WIFI definitions
  if (CONFIG_WIFI_ESP32)

    set(WPA_SUPPLICANT_COMPONENT_DIR "../../components/wpa_supplicant")
    #TODO: Additional WPA supplicant feature like Enterprise mode etc. are yet to be supported.
    set(WPA_SUPPLICANT_SRCS "${WPA_SUPPLICANT_COMPONENT_DIR}/port/os_xtensa.c"
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
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/eap_mschapv2.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/eap_peap.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/eap_peap_common.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/eap_tls.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/eap_tls_common.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/eap_ttls.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/mschapv2.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/eap_fast.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/eap_peer/eap_fast_common.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/rsn_supp/pmksa_cache.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/rsn_supp/wpa.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/rsn_supp/wpa_ie.c"
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

    if(CONFIG_ESP32_WIFI_ENABLE_SAE_PK)
      set(WPA_SUPPLICANT_SRCS ${WPA_SUPPLICANT_SRCS}
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/common/sae_pk.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/common/bss.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/src/common/scan.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_scan.c"
        )
    endif()

    set(ESP_SUPPLICANT_SRCS "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_eap_client.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_wpa2_api_port.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_wpa_main.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_wpas_glue.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_common.c"
      "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/esp_wps.c"
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
      "${WPA_SUPPLICANT_COMPONENT_DIR}/src/tls/x509v3.c"
      )

    if(CONFIG_ESP32_WIFI_MBEDTLS_CRYPTO)
      set(CRYPTO_SRCS
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/crypto/crypto_mbedtls.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/crypto/crypto_mbedtls-bignum.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/crypto/crypto_mbedtls-rsa.c"
        "${WPA_SUPPLICANT_COMPONENT_DIR}/esp_supplicant/src/crypto/crypto_mbedtls-ec.c"
         # Add internal RC4 as RC4 has been removed from mbedtls
         "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/rc4.c"
        )
        if(NOT CONFIG_MBEDTLS_CIPHER_DES_ENABLED)
          set(CRYPTO_SRCS ${CRYPTO_SRCS} "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/des-internal.c")
        endif()
        if(NOT CONFIG_MBEDTLS_MAC_CMAC_ENABLED AND NOT CONFIG_MBEDTLS_CMAC AND NOT CONFIG_PSA_WANT_ALG_CMAC)
            set(CRYPTO_SRCS ${CRYPTO_SRCS} "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-omac1.c")
        endif()
        if(NOT CONFIG_MBEDTLS_NIST_KW_C)
            set(CRYPTO_SRCS ${CRYPTO_SRCS}
            "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-wrap.c"
            "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-unwrap.c")
        endif()
        if(NOT CONFIG_MBEDTLS_NIST_KW_C OR NOT CONFIG_MBEDTLS_CMAC_C OR NOT CONFIG_MBEDTLS_CCM_C)
            set(CRYPTO_SRCS ${CRYPTO_SRCS} "${WPA_SUPPLICANT_COMPONENT_DIR}/src/crypto/aes-ccm.c")
        endif()
    else()
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
        )
    endif()

    zephyr_sources_ifdef(CONFIG_BUILD_ONLY_NO_BLOBS
      ../port/stubs/wifi_stubs.c
      ../port/stubs/phy_stubs.c
      )

    zephyr_sources(
      ../../components/esp_wifi/${CONFIG_SOC_SERIES}/esp_adapter.c
      ../../components/esp_wifi/src/wifi_init.c
      ../../components/esp_wifi/src/lib_printf.c
      ../port/stubs/wifi_compat_stubs.c
      ${WPA_SUPPLICANT_SRCS}
      ${ESP_SUPPLICANT_SRCS}
      ${TLS_SRCS}
      ${CRYPTO_SRCS}
      )

    zephyr_compile_definitions(
      __ets__
      ESP_SUPPLICANT
      ESPRESSIF_USE
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

    zephyr_compile_definitions(CONFIG_ESP_WIFI_ENABLED)
    zephyr_compile_definitions(CONFIG_SOC_WIFI_SUPPORTED)
    zephyr_compile_definitions_ifdef(CONFIG_ESP32_WIFI_MBEDTLS_CRYPTO CONFIG_CRYPTO_MBEDTLS)
    zephyr_compile_definitions_ifdef(CONFIG_ESP32_WIFI_ENABLE_WPA3_SAE CONFIG_WPA3_SAE)
    zephyr_compile_definitions_ifdef(CONFIG_ESP32_WIFI_ENABLE_WPA3_OWE_STA CONFIG_OWE_STA)
    zephyr_compile_definitions_ifdef(CONFIG_ESP32_WIFI_ENABLE_SAE_H2E CONFIG_SAE_H2E)
    zephyr_compile_definitions_ifdef(CONFIG_ESP32_WIFI_ENABLE_SAE_PK CONFIG_SAE_PK)
    zephyr_compile_definitions_ifdef(CONFIG_ESP32_WIFI_SOFTAP_SAE_SUPPORT CONFIG_SAE)
    zephyr_compile_definitions_ifdef(CONFIG_ESP32_WIFI_WPA3_COMPATIBLE CONFIG_WPA3_COMPAT)

  endif()

  ## IEEE 802.15.4 definitions
  if (CONFIG_IEEE802154)
    zephyr_sources(
      ../../components/esp_phy/src/btbb_init.c
      ../../components/esp_hal_ieee802154/${CONFIG_SOC_SERIES}/ieee802154_periph.c
      ../../components/ieee802154/esp_ieee802154.c
      ../../components/ieee802154/driver/esp_ieee802154_ack.c
      ../../components/ieee802154/driver/esp_ieee802154_dev.c
      ../../components/ieee802154/driver/esp_ieee802154_event.c
      ../../components/ieee802154/driver/esp_ieee802154_frame.c
      ../../components/ieee802154/driver/esp_ieee802154_pib.c
      ../../components/ieee802154/driver/esp_ieee802154_util.c
      ../../components/ieee802154/driver/esp_ieee802154_sec.c
      ../../components/ieee802154/driver/esp_ieee802154_timer.c
      )

    zephyr_sources_ifdef(CONFIG_BUILD_ONLY_NO_BLOBS
      ../port/stubs/bt_stubs.c
      ../port/stubs/ieee802154_stubs.c
      ../port/stubs/phy_stubs.c
      )

    zephyr_compile_definitions(CONFIG_IEEE802154_ENABLED)
    zephyr_compile_definitions(CONFIG_IEEE802154_RX_BUFFER_SIZE=${CONFIG_IEEE802154_ESP32_RX_BUFFER_SIZE})
    zephyr_compile_definitions(CONFIG_IEEE802154_CCA_MODE=${CONFIG_IEEE802154_ESP32_CCA_MODE})
    zephyr_compile_definitions(CONFIG_IEEE802154_CCA_THRESHOLD=${CONFIG_IEEE802154_ESP32_CCA_THRESHOLD})
    zephyr_compile_definitions(CONFIG_IEEE802154_PENDING_TABLE_SIZE=${CONFIG_IEEE802154_ESP32_PENDING_TABLE_SIZE})
    zephyr_compile_definitions(CONFIG_IEEE802154_INTERFACE_NUM=1)

    zephyr_link_libraries_ifndef(CONFIG_BUILD_ONLY_NO_BLOBS
      btbb
        -L${CMAKE_CURRENT_SOURCE_DIR}/../blobs/lib/${CONFIG_SOC_SERIES}
      )
  endif()

  zephyr_link_libraries_ifdef(CONFIG_NEWLIB_LIBC c)

endif()
