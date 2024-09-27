# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_SERIES_ESP32C6)

  zephyr_compile_options(-fstrict-volatile-bitfields)
  zephyr_compile_definitions_ifndef(CONFIG_MCUBOOT CONFIG_APP_BUILD_USE_FLASH_SECTIONS)
  zephyr_compile_definitions_ifndef(asm asm=__asm__)

  zephyr_compile_definitions(CONFIG_IDF_TARGET_ARCH_RISCV)

  if(CONFIG_ESP_SIMPLE_BOOT)
    zephyr_compile_definitions_ifdef(CONFIG_ESPTOOLPY_FLASHMODE_QIO BOOTLOADER_BUILD)
    zephyr_compile_definitions_ifdef(CONFIG_ESPTOOLPY_FLASHMODE_QOUT BOOTLOADER_BUILD)
  endif()

  if(CONFIG_MCUBOOT)
    zephyr_compile_options(-fdump-rtl-expand)
  endif()

  zephyr_include_directories(
    include
    ../esp_shared/include
    ../esp_shared/components/include
    ../port/include

    ../../components/efuse/include
    ../../components/efuse/private_include
    ../../components/efuse/${CONFIG_SOC_SERIES}/include
    ../../components/efuse/${CONFIG_SOC_SERIES}/private_include

    ../../components/esp_common/include

    ../../components/esp_hw_support/dma
    ../../components/esp_hw_support/include
    ../../components/esp_hw_support/include/esp_private
    ../../components/esp_hw_support/include/hal
    ../../components/esp_hw_support/include/soc
    ../../components/esp_hw_support/include/soc/${CONFIG_SOC_SERIES}
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/private_include
    ../../components/esp_hw_support/port/include
    ../../components/esp_rom/${CONFIG_SOC_SERIES}
    ../../components/esp_rom/${CONFIG_SOC_SERIES}/ld
    ../../components/esp_rom/include/
    ../../components/esp_rom/include/${CONFIG_SOC_SERIES}

    ../../components/esp_system/include
    ../../components/esp_system/include/esp_private
    ../../components/esp_system/port/include
    ../../components/esp_system/port/include/private/esp_private
    ../../components/esp_system/port/include/private

    ../../components/hal/${CONFIG_SOC_SERIES}/include
    ../../components/hal/include
    ../../components/hal/platform_port/include

    ../../components/log/include

    ../../components/soc/${CONFIG_SOC_SERIES}/include
    ../../components/soc/${CONFIG_SOC_SERIES}/ld
    ../../components/soc/include

    ../../components/riscv/${CONFIG_SOC_SERIES}/include
    ../../components/riscv/include
    ../../components/riscv/include/esp_private
    ../../components/riscv/include/xtensa

    ../../components/esp_timer/include
    ../../components/esp_timer/private_include

    ../../components/driver/include
    ../../components/driver/deprecated
    ../../components/driver/gpio/include
    ../../components/driver/uart/include
    ../../components/driver/touch_sensor/include
    ../../components/driver/touch_sensor/${CONFIG_SOC_SERIES}/include
    ../../components/driver/spi/include

    ../../components/spi_flash/include
    ../../components/spi_flash/include/spi_flash

    ../../components/esp_pm/include

    ../../components/bootloader_support/include
    ../../components/bootloader_support/private_include
    ../../components/bootloader_support/bootloader_flash/include

    ../../components/heap/include

    ../../components/esp_mm/include
  )

  zephyr_link_libraries(
    gcc
    -T${CMAKE_CURRENT_SOURCE_DIR}/src/linker/${CONFIG_SOC_SERIES}.rom.alias.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.api.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp_rom/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.rom.libgcc.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../components/soc/${CONFIG_SOC_SERIES}/ld/${CONFIG_SOC_SERIES}.peripherals.ld
    -T${CMAKE_CURRENT_SOURCE_DIR}/../../tools/esptool_py/flasher_stub/ld/rom_32c6.ld
  )

  zephyr_compile_definitions(ESP_PLATFORM)

  zephyr_sources_ifdef(
    CONFIG_ESP32_SPIM
    ../../components/hal/spi_hal.c
    ../../components/hal/spi_hal_iram.c
    ../../components/soc/lldesc.c
    ../../components/hal/gdma_hal.c
    )

  if(CONFIG_SOC_FLASH_ESP32 OR NOT CONFIG_BOOTLOADER_MCUBOOT)
    zephyr_sources(
    ../../components/esp_mm/esp_mmu_map.c
    ../../components/esp_mm/port/${CONFIG_SOC_SERIES}/ext_mem_layout.c
    ../../components/bootloader_support/src/flash_encrypt.c
    ../../components/esp_hw_support/esp_gpio_reserve.c
    ../../components/hal/cache_hal.c
    ../../components/hal/mmu_hal.c
    ../../components/hal/spi_flash_encrypt_hal_iram.c
    ../../components/hal/spi_flash_hal.c
    ../../components/hal/spi_flash_hal_iram.c
    ../../components/hal/spi_flash_hal_gpspi.c
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
    )
  endif()

  if(CONFIG_PM OR CONFIG_POWEROFF)
  zephyr_sources(
    ../../components/driver/gpio/gpio.c
    ../../components/driver/gpio/rtc_io.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_sleep.c
    ../../components/esp_hw_support/sleep_clock.c
    ../../components/esp_hw_support/sleep_cpu.c
    ../../components/esp_hw_support/sleep_cpu_asm.S
    ../../components/esp_hw_support/sleep_gpio.c
    ../../components/esp_hw_support/sleep_event.c
    ../../components/esp_hw_support/sleep_console.c
    ../../components/esp_hw_support/sleep_modem.c
    ../../components/esp_hw_support/sleep_modes.c
    ../../components/esp_hw_support/sleep_retention.c
    ../../components/esp_hw_support/sleep_system_peripheral.c
    ../../components/esp_pm/pm_impl.c
    ../../components/hal/${CONFIG_SOC_SERIES}/pmu_hal.c
    ../../components/hal/rtc_io_hal.c
    )
  endif()

  zephyr_sources_ifdef(
    CONFIG_DMA_ESP32
    ../../components/soc/lldesc.c
    ../../components/hal/gdma_hal.c
    )

  if (CONFIG_MCUBOOT)
    zephyr_compile_definitions(BOOTLOADER_BUILD)
    zephyr_compile_definitions(NDEBUG)

    zephyr_sources(
        ../port/boot/esp_image_loader.c
        )
  endif()

  if (CONFIG_MCUBOOT OR NOT CONFIG_BOOTLOADER_MCUBOOT)
    zephyr_include_directories(
      ../../components/esp_rom/${CONFIG_SOC_SERIES}
      ../port/include/boot
      )

    zephyr_sources(
      ../../components/bootloader_support/src/bootloader_mem.c
      ../../components/bootloader_support/src/bootloader_clock_init.c
      ../../components/bootloader_support/src/bootloader_console.c
      ../../components/hal/mpu_hal.c
      ../../components/bootloader_support/src/${CONFIG_SOC_SERIES}/bootloader_${CONFIG_SOC_SERIES}.c
      ../../components/bootloader_support/src/bootloader_init.c
      ../../components/bootloader_support/src/bootloader_common_loader.c
      ../../components/bootloader_support/bootloader_flash/src/flash_qio_mode.c
      ../../components/bootloader_support/bootloader_flash/src/bootloader_flash_config_${CONFIG_SOC_SERIES}.c
      )
  endif()

  zephyr_sources(

    ../../components/soc/${CONFIG_SOC_SERIES}/gpio_periph.c
    ../../components/soc/${CONFIG_SOC_SERIES}/rtc_io_periph.c
    ../../components/soc/${CONFIG_SOC_SERIES}/temperature_sensor_periph.c

    ../../components/bootloader_support/bootloader_flash/src/bootloader_flash.c
    ../../components/bootloader_support/src/bootloader_random_${CONFIG_SOC_SERIES}.c
    ../../components/bootloader_support/src/${CONFIG_SOC_SERIES}/bootloader_soc.c

    ../../components/esp_hw_support/adc_share_hw_ctrl.c
    ../../components/esp_hw_support/cpu.c
    ../../components/esp_hw_support/esp_clk.c
    ../../components/esp_hw_support/mac_addr.c
    ../../components/esp_hw_support/modem_clock.c
    ../../components/esp_hw_support/periph_ctrl.c
    ../../components/esp_hw_support/regi2c_ctrl.c
    ../../components/esp_hw_support/sar_periph_ctrl_common.c
    ../../components/esp_hw_support/sleep_modes.c
    ../../components/esp_hw_support/sleep_retention.c
    ../../components/esp_hw_support/port/esp_clk_tree_common.c
    ../../components/esp_hw_support/port/regdma_link.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/esp_cpu_intr.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/esp_clk_tree.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_clk.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_clk_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/rtc_time.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/sar_periph_ctrl.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/systimer.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/pmu_param.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/ocode_init.c
    ../../components/esp_hw_support/port/${CONFIG_SOC_SERIES}/cpu_region_protect.c

    ../../components/esp_rom/patches/esp_rom_crc.c
    ../../components/esp_rom/patches/esp_rom_sys.c
    ../../components/esp_rom/patches/esp_rom_uart.c
    ../../components/esp_rom/patches/esp_rom_spiflash.c
    ../../components/esp_rom/patches/esp_rom_efuse.c
    ../../components/esp_rom/patches/esp_rom_systimer.c
    ../../components/esp_rom/patches/esp_rom_hp_regi2c_esp32c6.c

    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/clk.c
    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/system_internal.c
    ../../components/esp_system/port/soc/${CONFIG_SOC_SERIES}/reset_reason.c
    ../../components/esp_system/esp_err.c

    ../../components/esp_timer/src/ets_timer_legacy.c
    ../../components/esp_timer/src/esp_timer.c
    ../../components/esp_timer/src/esp_timer_etm.c
    ../../components/esp_timer/src/esp_timer_impl_common.c
    ../../components/esp_timer/src/esp_timer_impl_systimer.c

    ../../components/hal/efuse_hal.c
    ../../components/hal/systimer_hal.c
    ../../components/hal/wdt_hal_iram.c
    ../../components/hal/${CONFIG_SOC_SERIES}/clk_tree_hal.c
    ../../components/hal/${CONFIG_SOC_SERIES}/efuse_hal.c
    ../../components/hal/${CONFIG_SOC_SERIES}/modem_clock_hal.c
    ../../components/hal/lp_timer_hal.c

    ../../components/log/log_noos.c
    ../../components/log/log.c

    ../esp_shared/src/common/esp_system_api.c

    ../port/heap/heap_caps_zephyr.c
    ../port/host_flash/cache_utils.c
    ../../components/riscv/interrupt.c

    src/stubs.c
  )

  
  zephyr_sources_ifdef(
    CONFIG_UART_ESP32
    ../../components/hal/uart_hal.c
    ../../components/hal/uart_hal_iram.c
    )
    
  zephyr_sources_ifdef(
    CONFIG_PWM_LED_ESP32
    ../../components/hal/ledc_hal_iram.c
    ../../components/hal/ledc_hal.c
    )

  zephyr_link_libraries_ifdef(CONFIG_NEWLIB_LIBC c)

endif()
