/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdarg.h>
#include "sdkconfig.h"
#include "esp_flash.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "rom/cache.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#include "soc/soc_caps.h"

static IRAM_ATTR esp_err_t start(void *arg)
{
#if CONFIG_IDF_TARGET_ESP32
    Cache_Read_Disable(0);
#if CONFIG_SMP
    Cache_Read_Disable(1);
#endif
#else
    cache_hal_suspend(CACHE_TYPE_ALL);
#endif

    return ESP_OK;
}

static IRAM_ATTR esp_err_t end(void *arg)
{
#if CONFIG_IDF_TARGET_ESP32
    Cache_Read_Enable(0);
#if CONFIG_SMP
    Cache_Read_Enable(1);
#endif
#else
    cache_hal_resume(CACHE_TYPE_ALL);
#endif

    return ESP_OK;
}

static IRAM_ATTR esp_err_t delay_us(void *arg, uint32_t us)
{
    esp_rom_delay_us(us);
    return ESP_OK;
}

// Currently when the os is not up yet, the caller is supposed to call esp_flash APIs with proper
// buffers.
IRAM_ATTR void* get_temp_buffer_not_supported(void* arg, size_t reqest_size, size_t* out_size)
{
    return NULL;
}

const DRAM_ATTR esp_flash_os_functions_t esp_flash_noos_functions = {
    .start = start,
    .end = end,
    .delay_us = delay_us,
    .region_protected = NULL,
    /* the caller is supposed to call esp_flash_read/esp_flash_write APIs with buffers in DRAM */
    .get_temp_buffer = NULL,
    .release_temp_buffer = NULL,
    .yield = NULL,
};

esp_err_t IRAM_ATTR esp_flash_app_disable_os_functions(esp_flash_t* chip)
{
    chip->os_func = &esp_flash_noos_functions;

    return ESP_OK;
}
