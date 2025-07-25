/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "bootloader_init.h"
#include "bootloader_flash_priv.h"
#include "bootloader_flash_config.h"
#include "bootloader_random.h"
#include "bootloader_clock.h"
#include "bootloader_common.h"
#include "esp_flash_encrypt.h"
#include "esp_cpu.h"
#include "soc/rtc.h"
#include "hal/wdt_hal.h"
#include "hal/efuse_hal.h"

static const char *TAG = "boot";

esp_image_header_t WORD_ALIGNED_ATTR bootloader_image_hdr;

void bootloader_clear_bss_section(void)
{
    memset(&_bss_start, 0, ((unsigned*)&_bss_end - (unsigned*)&_bss_start) * sizeof(&_bss_start));
}

esp_err_t bootloader_read_bootloader_header(void)
{
    /* load bootloader image header */
    if (bootloader_flash_read(ESP_BOOTLOADER_OFFSET, &bootloader_image_hdr, sizeof(esp_image_header_t), true) != ESP_OK) {
        ESP_EARLY_LOGE(TAG, "failed to load bootloader image header!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t bootloader_check_bootloader_validity(void)
{
    unsigned int chip_revision = efuse_hal_chip_revision();
    unsigned int chip_major_rev = chip_revision / 100;
    unsigned int chip_minor_rev = chip_revision % 100;
    ESP_EARLY_LOGI(TAG, "chip revision: v%d.%d", chip_major_rev, chip_minor_rev);
/* ESP32 doesn't have more memory and more efuse bits for block major version. */
#if !CONFIG_IDF_TARGET_ESP32
    unsigned int efuse_revision = efuse_hal_blk_version();
    unsigned int efuse_major_rev = efuse_revision / 100;
    unsigned int efuse_minor_rev = efuse_revision % 100;
    ESP_EARLY_LOGI(TAG, "efuse block revision: v%d.%d", efuse_major_rev, efuse_minor_rev);
#endif  // !CONFIG_IDF_TARGET_ESP32

#if defined(CONFIG_SOC_SERIES_ESP32)
    if (major < 3) {
        ESP_EARLY_LOGE(TAG, "You are using ESP32 chip revision (%d) that is unsupported. While it may work, it could cause unexpected behavior or issues.", major);
        ESP_EARLY_LOGE(TAG, "Proceeding with this ESP32 chip revision is not recommended unless you fully understand the potential risk and limitations.");
#if !defined(CONFIG_ESP32_USE_UNSUPPORTED_REVISION)
        ESP_EARLY_LOGE(TAG, "If you choose to continue, please enable the 'CONFIG_ESP32_USE_UNSUPPORTED_REVISION' in your project configuration.");
        return ESP_FAIL;
#endif
    }
#endif
    /* compare with the one set in bootloader image header */
    if (bootloader_common_check_chip_validity(&bootloader_image_hdr, ESP_IMAGE_BOOTLOADER) != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void bootloader_config_wdt(void)
{
    /*
     * At this point, the flashboot protection of RWDT and MWDT0 will have been
     * automatically enabled. We can disable flashboot protection as it's not
     * needed anymore. If configured to do so, we also initialize the RWDT to
     * protect the remainder of the bootloader process.
     */
    //Disable RWDT flashboot protection.
    wdt_hal_context_t rwdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
    wdt_hal_write_protect_disable(&rwdt_ctx);
    wdt_hal_set_flashboot_en(&rwdt_ctx, false);
    wdt_hal_write_protect_enable(&rwdt_ctx);

#ifdef CONFIG_BOOTLOADER_WDT_ENABLE
    //Initialize and start RWDT to protect the  for bootloader if configured to do so
    ESP_EARLY_LOGD(TAG, "Enabling RTCWDT(%d ms)", CONFIG_BOOTLOADER_WDT_TIME_MS);
    wdt_hal_init(&rwdt_ctx, WDT_RWDT, 0, false);
    uint32_t stage_timeout_ticks = (uint32_t)((uint64_t)CONFIG_BOOTLOADER_WDT_TIME_MS * rtc_clk_slow_freq_get_hz() / 1000);
    wdt_hal_write_protect_disable(&rwdt_ctx);
    wdt_hal_config_stage(&rwdt_ctx, WDT_STAGE0, stage_timeout_ticks, WDT_STAGE_ACTION_RESET_RTC);
    wdt_hal_enable(&rwdt_ctx);
    wdt_hal_write_protect_enable(&rwdt_ctx);
#endif

    //Disable MWDT0 flashboot protection. But only after we've enabled the RWDT first so that there's not gap in WDT protection.
    wdt_hal_context_t mwdt_ctx = {.inst = WDT_MWDT0, .mwdt_dev = &TIMERG0};
    wdt_hal_write_protect_disable(&mwdt_ctx);
    wdt_hal_set_flashboot_en(&mwdt_ctx, false);
    wdt_hal_write_protect_enable(&mwdt_ctx);
}

void bootloader_enable_random(void)
{
    ESP_EARLY_LOGI(TAG, "Enabling RNG early entropy source...");
    bootloader_random_enable();
}

void bootloader_print_banner(void)
{
#ifdef CONFIG_MCUBOOT
    ESP_EARLY_LOGI(TAG, "MCUboot 2nd stage bootloader");
#else
#ifdef CONFIG_BOOTLOADER_MCUBOOT
    ESP_EARLY_LOGI(TAG, "MCUboot Application image");
#else /* ifdef CONFIG_ESP_SIMPLE_BOOT */
    ESP_EARLY_LOGI(TAG, "ESP Simple boot");
#endif
#endif

#ifndef CONFIG_APP_REPRODUCIBLE_BUILD
    ESP_EARLY_LOGI(TAG, "compile time " __DATE__ " " __TIME__);
#endif

#if CONFIG_FREERTOS_UNICORE
#if (SOC_CPU_CORES_NUM > 1)
    ESP_EARLY_LOGW(TAG, "Unicore bootloader");
#endif
#else
    ESP_EARLY_LOGI(TAG, "Multicore bootloader");
#endif
}
