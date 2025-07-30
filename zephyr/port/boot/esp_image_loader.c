/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <bootutil/bootutil_log.h>

#include <bootutil/fault_injection_hardening.h>

#include "bootloader_flash_priv.h"
#include "esp_flash_encrypt.h"
#include "soc/soc_memory_layout.h"
#include "esp_log.h"

#if !defined(CONFIG_SOC_SERIES_ESP32C2) &&	\
	!defined(CONFIG_SOC_SERIES_ESP32C3) &&	\
	!defined(CONFIG_SOC_SERIES_ESP32C6) &&	\
	!defined(CONFIG_SOC_SERIES_ESP32H2)
#include "soc/dport_reg.h"
#endif

#include "esp_rom_sys.h"
#include "soc/gpio_periph.h"
#include "soc/rtc_periph.h"
#if !defined(CONFIG_SOC_SERIES_ESP32C6) && !defined(CONFIG_SOC_SERIES_ESP32H2)
#include "soc/rtc_cntl_reg.h"
#endif
#include "esp_cpu.h"

#if CONFIG_SOC_SERIES_ESP32
#include "esp32/rom/uart.h"
#include "esp32/rom/cache.h"
#define LP_RTC_PREFIX "RTC"
#elif CONFIG_SOC_SERIES_ESP32S2
#include "esp32s2/rom/uart.h"
#define LP_RTC_PREFIX "RTC"
#elif CONFIG_SOC_SERIES_ESP32S3
#include "esp32s3/rom/uart.h"
#include "esp32s3/rom/cache.h"
#define LP_RTC_PREFIX "RTC"
#elif CONFIG_SOC_SERIES_ESP32C2
#include "esp32c2/rom/uart.h"
#elif CONFIG_SOC_SERIES_ESP32C3
#include "esp32c3/rom/uart.h"
#define LP_RTC_PREFIX "RTC"
#elif CONFIG_SOC_SERIES_ESP32C6
#include "esp32c6/rom/uart.h"
#define LP_RTC_PREFIX "LP"
#elif CONFIG_SOC_SERIES_ESP32H2
#include "esp32h2/rom/uart.h"
#define LP_RTC_PREFIX "LP"
#endif

#include "esp_mcuboot_image.h"
#include "esp_image_loader.h"
#include "flash_map_backend/flash_map_backend.h"
#include <zephyr/drivers/timer/system_timer.h>

#ifdef CONFIG_ESP_MULTI_PROCESSOR_BOOT
#include "app_cpu_start.h"
#endif

static int load_segment(const struct flash_area *fap, uint32_t data_addr,
                        uint32_t data_len, uint32_t load_addr)
{
    const uint32_t *data = (const uint32_t *)esp_rom_flash_mmap((fap->fa_off + data_addr), data_len);
    if (!data) {
        BOOT_LOG_ERR("%s: Bootloader mmap failed", __func__);
        return -1;
    }
    memcpy((void *)load_addr, data, data_len);
    esp_rom_flash_unmmap(data);
    return 0;
}

void esp_app_image_load(int image_index, int slot,
                        unsigned int hdr_offset, unsigned int *entry_addr)
{
    const struct flash_area *fap;
    int area_id;
    int rc;

    area_id = flash_area_id_from_multi_image_slot(image_index, slot);
    rc = flash_area_open(area_id, &fap);
    if (rc != 0) {
        BOOT_LOG_ERR("%s: flash_area_open failed with %d", __func__, rc);
    }

    BOOT_LOG_INF("Loading image %d - slot %d from flash, area id: %d",
    image_index, slot, area_id);

    const uint32_t *data = (const uint32_t *)esp_rom_flash_mmap((fap->fa_off + hdr_offset),
                                                             sizeof(esp_image_load_header_t));
    esp_image_load_header_t load_header = {0};
    memcpy((void *)&load_header, data, sizeof(esp_image_load_header_t));
    esp_rom_flash_unmmap(data);

    if (load_header.header_magic != ESP_LOAD_HEADER_MAGIC) {
        BOOT_LOG_ERR("Load header magic verification failed. Aborting");
        FIH_PANIC;
    }

    if (!esp_ptr_in_iram((void *)load_header.iram_dest_addr) ||
        !esp_ptr_in_iram((void *)(load_header.iram_dest_addr + load_header.iram_size))) {
        BOOT_LOG_ERR("IRAM region in load header is not valid. Aborting");
        FIH_PANIC;
    }

    if (!esp_ptr_in_dram((void *)load_header.dram_dest_addr) ||
        !esp_ptr_in_dram((void *)(load_header.dram_dest_addr + load_header.dram_size))) {
        BOOT_LOG_ERR("DRAM region in load header is not valid. Aborting");
        FIH_PANIC;
    }

#if SOC_RTC_FAST_MEM_SUPPORTED
    if ((load_header.lp_rtc_iram_size) &&
	(!esp_ptr_in_rtc_iram_fast((void *)load_header.lp_rtc_iram_dest_addr) ||
        !esp_ptr_in_rtc_iram_fast((void *)(load_header.lp_rtc_iram_dest_addr + load_header.lp_rtc_iram_size)))) {
        BOOT_LOG_ERR("%s_IRAM region in load header is not valid. Aborting", LP_RTC_PREFIX);
        FIH_PANIC;
    }
#endif

#if SOC_RTC_SLOW_MEM_SUPPORTED
    if ((load_header.lp_rtc_dram_size) &&
	(!esp_ptr_in_rtc_slow((void *)load_header.lp_rtc_dram_dest_addr) ||
        !esp_ptr_in_rtc_slow((void *)(load_header.lp_rtc_dram_dest_addr + load_header.lp_rtc_dram_size)))) {
        BOOT_LOG_ERR("%s_RAM region in load header is not valid. Aborting %p", LP_RTC_PREFIX, load_header.lp_rtc_dram_dest_addr);
        FIH_PANIC;
    }
#endif

    if (!esp_ptr_in_iram((void *)load_header.entry_addr)) {
        BOOT_LOG_ERR("Application entry point (%xh) is not in IRAM. Aborting",
        load_header.entry_addr);
        FIH_PANIC;
    }

    BOOT_LOG_INF("Application start=%xh", load_header.entry_addr);

#if SOC_RTC_FAST_MEM_SUPPORTED || SOC_RTC_SLOW_MEM_SUPPORTED
    if (load_header.lp_rtc_dram_size > 0) {
        soc_reset_reason_t reset_reason = esp_rom_get_reset_reason(0);

        /* Unless waking from deep sleep (implying RTC memory is intact), load its segments */
        if (reset_reason != RESET_REASON_CORE_DEEP_SLEEP) {
            BOOT_LOG_INF("%s_RAM\t: lma=%08xh vma=%08xh size=%05xh (%6d) load", LP_RTC_PREFIX,
                         (fap->fa_off + load_header.lp_rtc_dram_flash_offset), load_header.lp_rtc_dram_dest_addr,
                         load_header.lp_rtc_dram_size, load_header.lp_rtc_dram_size);
            load_segment(fap, load_header.lp_rtc_dram_flash_offset,
                         load_header.lp_rtc_dram_size, load_header.lp_rtc_dram_dest_addr);
        } else {
            BOOT_LOG_INF("%s_RAM\t: lma=%08xh vma=%08xh size=%05xh (%6d) noload", LP_RTC_PREFIX,
                         load_header.lp_rtc_dram_flash_offset, load_header.lp_rtc_dram_dest_addr,
                         load_header.lp_rtc_dram_size, load_header.lp_rtc_dram_size);
        }
    }

    if (load_header.lp_rtc_iram_size > 0) {
        BOOT_LOG_INF("%s_IRAM\t: lma=%08xh vma=%08xh size=%05xh (%6d) load", LP_RTC_PREFIX,
                     (fap->fa_off + load_header.lp_rtc_iram_flash_offset), load_header.lp_rtc_iram_dest_addr,
                     load_header.lp_rtc_iram_size, load_header.lp_rtc_iram_size);
        load_segment(fap, load_header.lp_rtc_iram_flash_offset,
                     load_header.lp_rtc_iram_size, load_header.lp_rtc_iram_dest_addr);
    }
#endif

    BOOT_LOG_INF("DRAM\t: lma=%08xh vma=%08xh size=%05xh (%6d) load",
                 (fap->fa_off + load_header.dram_flash_offset), load_header.dram_dest_addr,
                 load_header.dram_size, load_header.dram_size);
    load_segment(fap, load_header.dram_flash_offset,
                 load_header.dram_size, load_header.dram_dest_addr);

    BOOT_LOG_INF("IRAM\t: lma=%08xh vma=%08xh size=%05xh (%6d) load",
                 (fap->fa_off + load_header.iram_flash_offset), load_header.iram_dest_addr,
                 load_header.iram_size, load_header.iram_size);
    load_segment(fap, load_header.iram_flash_offset,
                 load_header.iram_size, load_header.iram_dest_addr);

    uart_tx_wait_idle(0);

    assert(entry_addr != NULL);
    *entry_addr = load_header.entry_addr;
}

void start_cpu0_image(int image_index, int slot, unsigned int hdr_offset)
{
    unsigned int entry_addr;
    esp_app_image_load(image_index, slot, hdr_offset, &entry_addr);

    if (IS_ENABLED(CONFIG_SYSTEM_TIMER_HAS_DISABLE_SUPPORT)) {
        sys_clock_disable();
    }

    ((void (*)(void))entry_addr)(); /* Call to application entry address should not return */
    FIH_PANIC; /* It should not get here */
}

#ifdef CONFIG_ESP_MULTI_PROCESSOR_BOOT

void appcpu_start(uint32_t entry_addr)
{
    ESP_LOGI(TAG, "Starting APPCPU");

#if defined(CONFIG_SOC_SERIES_ESP32)
    Cache_Flush(1);
    Cache_Read_Enable(1);
#endif

    esp_cpu_unstall(1);

#if defined(CONFIG_SOC_SERIES_ESP32)
    DPORT_SET_PERI_REG_MASK(DPORT_APPCPU_CTRL_B_REG, DPORT_APPCPU_CLKGATE_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_APPCPU_CTRL_C_REG, DPORT_APPCPU_RUNSTALL);
    DPORT_SET_PERI_REG_MASK(DPORT_APPCPU_CTRL_A_REG, DPORT_APPCPU_RESETTING);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_APPCPU_CTRL_A_REG, DPORT_APPCPU_RESETTING);
#elif defined(CONFIG_SOC_SERIES_ESP32S3)
    // Enable clock and reset APP CPU. Note that OpenOCD may have already
    // enabled clock and taken APP CPU out of reset. In this case don't reset
    // APP CPU again, as that will clear the breakpoints which may have already
    // been set.
    if (!REG_GET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN)) {
        REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN);
        REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RUNSTALL);
        REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETTING);
        REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETTING);
    }
#endif
    ets_set_appcpu_boot_addr(entry_addr);
    ets_delay_us(10000);
    uart_tx_wait_idle(0);
    ESP_LOGI(TAG, "APPCPU start sequence complete");
}

void start_cpu1_image(int image_index, int slot, unsigned int hdr_offset)
{
    unsigned int entry_addr;
    esp_app_image_load(image_index, slot, hdr_offset, &entry_addr);
    appcpu_start(entry_addr);
}
#endif
