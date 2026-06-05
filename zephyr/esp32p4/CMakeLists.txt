# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_SERIES_ESP32P4)

  zephyr_compile_options(-fstrict-volatile-bitfields)
  zephyr_compile_definitions_ifndef(CONFIG_MCUBOOT CONFIG_APP_BUILD_USE_FLASH_SECTIONS)
  zephyr_compile_definitions_ifndef(asm asm=__asm__)
  zephyr_compile_definitions_ifndef(typeof typeof=__typeof__)
  zephyr_compile_definitions(CONFIG_ESP_REV_MIN_FULL=500)

  if(CONFIG_MCUBOOT)
    zephyr_compile_options(-fdump-rtl-expand)
  endif()

  zephyr_include_directories(
    include
    ../common/include
    ../port/include
    ../port/include/boot
    ../../components/bootloader_support/bootloader_flash/include
    ../../components/bootloader_support/include
    ../../components/bootloader_support/private_include
    ../../components/driver/deprecated
    ../../components/driver/gpio/include
    ../../components/driver/include
    ../../components/driver/spi/include
    ../../components/driver/uart/include
    ../../components/efuse/include
    ../../components/efuse/private_include
    ../../components/efuse/${CONFIG_SOC_SERIES}/include
    ../../components/efuse/${CONFIG_SOC_SERIES}/private_include
    ../../components/esp_common/include
    ../../components/esp_driver_gpio/include
    ../../components/esp_event/include
    ../../components/esp_hal_ana_conv/include
    ../../components/esp_hal_ana_conv/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_clock/include
    ../../components/esp_hal_clock/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_cam/include
    ../../components/esp_hal_cam/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_dma/include
    ../../components/esp_hal_dma/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_emac/include
    ../../components/esp_hal_emac/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_gpio/include
    ../../components/esp_hal_gpio/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_gpspi/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_gpspi/include
    ../../components/esp_hal_i2c/include
    ../../components/esp_hal_i2c/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_i2s/include
    ../../components/esp_hal_i2s/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_lcd/include
    ../../components/esp_hal_lcd/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_ledc/include
    ../../components/esp_hal_ledc/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_mcpwm/include
    ../../components/esp_hal_mcpwm/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_mspi/include
    ../../components/esp_hal_mspi/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_pcnt/include
    ../../components/esp_hal_pcnt/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_parlio/include
    ../../components/esp_hal_parlio/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_pmu/include
    ../../components/esp_hal_pmu/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_regi2c/include
    ../../components/esp_hal_regi2c/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_rmt/include
    ../../components/esp_hal_rmt/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_rtc_timer/include
    ../../components/esp_hal_rtc_timer/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_sd/include
    ../../components/esp_hal_sd/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_security/include
    ../../components/esp_hal_security/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_systimer/include
    ../../components/esp_hal_systimer/${CONFIG_SOC_SERIES}/include
    ../../components/esp_hal_touch_sens/include
    ../../components/esp_hal_touch_sens/${CONFIG_SOC_SERIES}/include
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
    ../../components/hal/${CONFIG_SOC_SERIES}/include
    ../../components/hal/include
    ../../components/hal/platform_port/include
    ../../components/heap/include
    ../../components/mbedtls/port/include
    ../../components/riscv/include
    ../../components/riscv/include/esp_private
    ../../components/riscv/include/xtensa
    ../../components/soc/${CONFIG_SOC_SERIES}/include
    ../../components/soc/include
    ../../components/soc/${CONFIG_SOC_SERIES}/ld
    ../../components/soc/${CONFIG_SOC_SERIES}/register
    ../../components/soc/${CONFIG_SOC_SERIES}/register/hw_ver3
    ../../components/spi_flash/include
    ../../components/spi_flash/include/esp_private
    ../../components/ulp/lp_core/include
    ../../components/ulp/lp_core/lp_core/include
    ../../components/ulp/lp_core/shared/include
    ../../components/ulp/ulp_common/include
  )

  zephyr_link_libraries(
    gcc
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/soc/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.peripherals.ld
  )

  if(NOT CONFIG_SOC_ESP32P4_LPCORE)
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
    )
  endif()

  if(CONFIG_SOC_ESP32P4_LPCORE)
    zephyr_link_libraries(
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}lp.rom.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}lp.rom.api.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}lp.rom.newlib.ld
      -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}lp.rom.version.ld
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
    ../../components/esp_hal_gpspi/${CONFIG_SOC_SERIES}/spi_periph.c
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
    ../../components/esp_hw_support/mspi/mspi_timing_tuning/tuning_scheme_impl/mspi_timing_by_dqs.c
    ../../components/esp_hw_support/mspi/mspi_timing_tuning/tuning_scheme_impl/mspi_timing_by_flash_delay.c
    )
  endif()

  zephyr_sources_ifdef(CONFIG_ESPRESSIF_RMT
    ../../components/esp_hal_rmt/rmt_hal.c
    ../../components/esp_hal_gpio/gpio_hal.c
    ../../components/esp_driver_gpio/src/gpio.c
    ../../components/esp_driver_gpio/src/rtc_io.c
    ../../components/esp_hal_rmt/${CONFIG_SOC_SERIES}/rmt_periph.c
    )

  if(CONFIG_PM OR CONFIG_POWEROFF)
    zephyr_sources(
      ../../components/esp_driver_gpio/src/gpio.c
      ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_sleep.c
      ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_sleep_clock_icg.c
      ../../components/esp_hw_support/port/pau_regdma.c
      ../../components/esp_hw_support/lowpower/port/${CONFIG_SOC_SERIES}/sleep_clock.c
      ../../components/esp_hw_support/sleep_gpio.c
      ../../components/esp_hw_support/sleep_event.c
      ../../components/esp_hw_support/sleep_console.c
      ../../components/esp_hw_support/sleep_modem.c
      ../../components/esp_hw_support/sleep_system_peripheral.c
      ../../components/esp_hw_support/sleep_usb.c
      ../../components/soc/${CONFIG_SOC_SERIES}/system_retention_periph.c
      ../../components/esp_hal_pmu/${CONFIG_SOC_SERIES}/pau_hal.c
      ../../components/esp_hal_pmu/${CONFIG_SOC_SERIES}/pmu_hal.c
      ../../components/esp_hw_support/sleep_uart.c
      ../../components/esp_hw_support/clk_utils.c
      ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/peripheral_domain_pd.c
      ../common/pm_impl_zephyr.c
      )
    if(CONFIG_ESP32_PM_POWER_DOWN_CPU_IN_LIGHT_SLEEP)
      zephyr_sources(
        ../../components/esp_hw_support/lowpower/port/${CONFIG_SOC_SERIES}/sleep_cpu.c
        ../../components/esp_hw_support/lowpower/port/${CONFIG_SOC_SERIES}/sleep_cpu_asm.S
        ../../components/esp_hw_support/lowpower/port/${CONFIG_SOC_SERIES}/sleep_fpu_asm.S
        )
      zephyr_sources_ifdef(CONFIG_ESP32_PM_CPU_RETENTION_DYNAMIC
        ../../components/esp_hw_support/lowpower/port/${CONFIG_SOC_SERIES}/sleep_cpu_dynamic.c
        )
      zephyr_sources_ifdef(CONFIG_ESP32_PM_CPU_RETENTION_STATIC
        ../../components/esp_hw_support/lowpower/port/${CONFIG_SOC_SERIES}/sleep_cpu_static.c
        )
    endif()
  endif()

  if(CONFIG_DMA_ESP32)
    zephyr_sources(
      ../../components/soc/lldesc.c
      ../../components/esp_hal_dma/gdma_hal_top.c
      ../../components/esp_hal_dma/gdma_hal_ahb_v2.c
      ../../components/esp_hal_dma/gdma_hal_axi.c
      ../../components/esp_hal_dma/gdma_hal_crc_gen.c
      )

    if(CONFIG_ESP32_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP)
      zephyr_sources(
        ../../components/esp_hal_dma/${CONFIG_SOC_SERIES}/gdma_periph.c
        )
    endif()
  endif()

  if(CONFIG_WDT_ESP32 AND CONFIG_ESP32_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP)
    zephyr_sources(
      ../../components/esp_hal_wdt/${CONFIG_SOC_SERIES}/mwdt_periph.c
      )
  endif()

  if (CONFIG_ESP_SPIRAM)
    zephyr_compile_definitions(CONFIG_SPIRAM)
    zephyr_sources(
      ../../components/esp_psram/system_layer/esp_psram.c
      ../../components/esp_hw_support/esp_gpio_reserve.c
      )

    zephyr_sources_ifdef(CONFIG_SPIRAM_MODE_HEX
      ../../components/esp_psram/device/esp_psram_impl_ap_hex.c
      )

    if (CONFIG_SPIRAM_FETCH_INSTRUCTIONS OR CONFIG_SPIRAM_RODATA)
      zephyr_sources(
        ../../components/esp_psram/xip_impl/mmu_psram_flash.c
        )
    endif()
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

  zephyr_sources_ifdef(CONFIG_SOC_ESP32P4_HPCORE
    ../../components/bootloader_support/bootloader_flash/src/bootloader_flash_config_${CONFIG_SOC_SERIES}.c
    ../../components/esp_driver_gpio/src/rtc_io.c
    ../../components/esp_hal_ana_conv/${CONFIG_SOC_SERIES}/temperature_sensor_periph.c
    ../../components/esp_hal_ana_conv/temperature_sensor_hal.c
    ../../components/esp_hal_clock/${CONFIG_SOC_SERIES}/clk_tree_hal.c
    ../../components/esp_hal_gpio/${CONFIG_SOC_SERIES}/rtc_io_periph.c
    ../../components/esp_hal_gpio/gpio_hal.c
    ../../components/esp_hal_gpio/rtc_io_hal.c
    ../../components/esp_hal_systimer/systimer_hal.c
    ../../components/esp_hal_touch_sens/touch_sens_hal.c
    ../../components/esp_hal_wdt/wdt_hal_iram.c
    ../../components/esp_hw_support/adc_share_hw_ctrl.c
    ../../components/esp_hw_support/clk_ctrl_os.c
    ../../components/esp_hw_support/cpu.c
    ../../components/esp_hw_support/esp_clk.c
    ../../components/esp_hw_support/hw_random.c
    ../../components/esp_hw_support/mac_addr.c
    ../../components/esp_hw_support/periph_ctrl.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/cpu_region_protect.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/esp_clk_tree.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/esp_cpu_intr.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/io_mux.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_param.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_clk.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_clk_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_time.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/sar_periph_ctrl.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/systimer.c
    ../../components/esp_hw_support/port/esp_clk_tree_common.c
    ../../components/esp_hw_support/port/regdma_link.c
    ../../components/esp_hw_support/regi2c_ctrl.c
    ../../components/esp_hal_ana_conv/adc_hal_common.c
    ../../components/esp_hw_support/sar_periph_ctrl_common.c
    ../../components/esp_hw_support/rtc_module.c
    ../../components/esp_hw_support/sar_tsens_ctrl.c
    ../../components/esp_hw_support/sleep_modes.c
    ../../components/esp_hw_support/sleep_cache.c
    ../../components/esp_hw_support/sleep_retention.c
    ../../components/esp_hw_support/esp_memory_utils.c
    ../../components/esp_mm/esp_cache_msync.c
    ../../components/esp_rom/patches/esp_rom_crc.c
    ../../components/esp_rom/patches/esp_rom_efuse.c
    ../../components/esp_rom/patches/esp_rom_gpio.c
    ../../components/esp_hal_regi2c/${CONFIG_SOC_SERIES}/regi2c_impl.c
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

  zephyr_sources_ifdef(CONFIG_ESP_ENABLE_PVT
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_pvt.c
    )

  zephyr_sources_ifdef(CONFIG_I2C_ESP32
    ../../components/esp_hal_i2c/i2c_hal_iram.c
    ../../components/esp_hal_i2c/i2c_hal.c
    )

  zephyr_sources_ifdef(CONFIG_I2S_ESP32
    ../../components/esp_hal_i2s/i2s_hal.c
    ../../components/hal/hal_utils.c
    )

  if(CONFIG_COUNTER_TMR_ESP32)
    zephyr_sources(
      ../../components/esp_hal_timg/timer_hal.c
      )
    if(CONFIG_ESP32_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP)
      zephyr_sources(
        ../../components/esp_hal_timg/${CONFIG_SOC_SERIES}/timer_periph.c
        )
    endif()
  endif()

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

  if(CONFIG_MCPWM_ESP32)
    zephyr_sources(
      ../../components/esp_hal_mcpwm/mcpwm_hal.c
      )
    if(CONFIG_ESP32_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP)
      zephyr_sources(
        ../../components/esp_hal_mcpwm/${CONFIG_SOC_SERIES}/mcpwm_periph.c
        )
    endif()
  endif()

  zephyr_sources_ifdef(CONFIG_PCNT_ESP32
    ../../components/esp_hal_pcnt/pcnt_hal.c
    )

  zephyr_sources_ifdef(CONFIG_SDHC_ESP32
    ../../components/esp_hal_sd/sdmmc_hal.c
    ../../components/esp_hal_sd/${CONFIG_SOC_SERIES}/sdmmc_periph.c
    )

  if (CONFIG_UDC_DWC2 OR CONFIG_UHC_DWC2)
    zephyr_sources(
    ../../components/esp_hal_usb/usb_wrap_hal.c
    ../../components/esp_hal_usb/usb_utmi_hal.c
    )
  endif()

  zephyr_sources_ifdef(CONFIG_VIDEO_ESP32
    ../../components/esp_hal_cam/cam_hal.c
    )

  zephyr_sources_ifdef(CONFIG_MIPI_DBI_ESP32
    ../../components/esp_hal_lcd/lcd_hal.c
    )

  zephyr_sources_ifdef(CONFIG_MIPI_DSI_ESP32
    ../../components/esp_hal_lcd/mipi_dsi_hal.c
    ../../components/esp_hal_lcd/${CONFIG_SOC_SERIES}/mipi_dsi_periph.c
    )

  zephyr_sources_ifdef(CONFIG_INPUT_ESP32_TOUCH_SENSOR
    ../../components/esp_hal_touch_sens/esp32p4/touch_sensor_periph.c
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
    ../../components/esp_hal_ana_conv/adc_oneshot_hal.c
    ../../components/esp_adc/adc_cali.c
    ../../components/esp_adc/adc_cali_curve_fitting.c
    ../../components/esp_adc/${CONFIG_SOC_SERIES}/curve_fitting_coefficients.c
    ../../components/esp_hal_ana_conv/${CONFIG_SOC_SERIES}/adc_periph.c
    ../../components/efuse/${CONFIG_SOC_SERIES}/esp_efuse_rtc_calib.c
    )
  endif()

  if(CONFIG_SOC_ESP32P4_LPCORE)
    zephyr_compile_definitions(IS_ULP_COCPU)
    zephyr_ld_options("-nostartfiles")
    zephyr_ld_options("-Wl,--no-warn-rwx-segments")
    zephyr_ld_options("-Wl,--gc-sections")
  endif()

  if(CONFIG_ESP32_ULP_COPROC_ENABLED)
    zephyr_sources_ifdef(CONFIG_SOC_ESP32P4_HPCORE
      ../../components/ulp/lp_core/lp_core.c
      ../../components/ulp/lp_core/shared/ulp_lp_core_lp_timer_shared.c
      ../../components/ulp/lp_core/shared/ulp_lp_core_memory_shared.c
      )

    zephyr_sources_ifdef(CONFIG_SOC_ESP32P4_LPCORE
      ../../components/ulp/lp_core/lp_core/lp_core_interrupt.c
      ../../components/ulp/lp_core/lp_core/lp_core_utils.c
      ../../components/ulp/lp_core/lp_core/lp_core_print.c
      ../../components/ulp/lp_core/lp_core/lp_core_uart.c
      ../../components/ulp/lp_core/shared/ulp_lp_core_lp_timer_shared.c
      ../../components/ulp/lp_core/shared/ulp_lp_core_memory_shared.c
      ../../components/esp_hal_uart/uart_hal.c
      ../../components/esp_hal_uart/uart_hal_iram.c
      ../../components/esp_hal_gpio/rtc_io_hal.c
      ../../components/esp_hal_gpio/${CONFIG_SOC_SERIES}/rtc_io_periph.c
      )
  endif()

  zephyr_sources_ifdef(CONFIG_ESP32P4_EMAC
    ../../components/esp_hal_emac/emac_hal.c
    ../../components/esp_hal_emac/${CONFIG_SOC_SERIES}/emac_periph.c
    )

  zephyr_link_libraries_ifdef(CONFIG_NEWLIB_LIBC c)

endif()
