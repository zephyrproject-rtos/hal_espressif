/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include "rom/cache.h"

#if CONFIG_IDF_TARGET_ESP32
#include "soc/dport_reg.h"
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C6
#include "soc/extmem_reg.h"
#endif

#include "soc/ext_mem_defs.h"
#include "esp_rom_spiflash.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#include <soc/soc.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_memory_utils.h"
#include "esp_intr_alloc.h"
#include "esp_cpu.h"
#include "esp_private/esp_cache_private.h"
#include "esp_private/cache_utils.h"
#include "esp_private/spi_flash_os.h"
#include "esp_log.h"

ESP_LOG_ATTR_TAG(TAG, "cache");

static unsigned int s_intr_saved_state;

// Used only on ROM impl. in idf, this param unused, cache status hold by hal
static uint32_t s_flash_op_cache_state[2];

#ifndef CONFIG_MCUBOOT
K_MUTEX_DEFINE(s_flash_op_mutex);

void spi_flash_init_lock(void)
{
}

void spi_flash_op_lock(void)
{
    k_mutex_lock(&s_flash_op_mutex, K_FOREVER);
}

void spi_flash_op_unlock(void)
{
    k_mutex_unlock(&s_flash_op_mutex);
}
#endif /* !CONFIG_MCUBOOT */

void IRAM_ATTR spi_flash_disable_interrupts_caches_and_other_cpu(void)
{
    s_intr_saved_state = irq_lock();
    esp_intr_noniram_disable();
    spi_flash_disable_cache(0, &s_flash_op_cache_state[0]);
}

void IRAM_ATTR spi_flash_enable_interrupts_caches_and_other_cpu(void)
{
    spi_flash_restore_cache(0, s_flash_op_cache_state[0]);
    esp_intr_noniram_enable();
    irq_unlock(s_intr_saved_state);
}

void IRAM_ATTR spi_flash_disable_interrupts_caches_and_other_cpu_no_os(void)
{
    esp_intr_noniram_disable();
    spi_flash_disable_cache(0, &s_flash_op_cache_state[0]);
}

void IRAM_ATTR spi_flash_enable_interrupts_caches_no_os(void)
{
    spi_flash_restore_cache(0, s_flash_op_cache_state[0]);
    esp_intr_noniram_enable();
}


void IRAM_ATTR spi_flash_enable_cache(uint32_t cpuid)
{
#if CONFIG_IDF_TARGET_ESP32
    uint32_t cache_value = cache_ll_l1_get_enabled_bus(cpuid);

    // Re-enable cache on this CPU
    spi_flash_restore_cache(cpuid, cache_value);
#else
    spi_flash_restore_cache(0, 0); // TODO cache_value should be non-zero
#endif
}

void IRAM_ATTR spi_flash_disable_cache(uint32_t cpuid, uint32_t *saved_state)
{
#if SOC_BRANCH_PREDICTOR_SUPPORTED
    //branch predictor will start cache request as well
    esp_cpu_branch_prediction_disable();
#endif
    esp_cache_suspend_ext_mem_cache();
}

void IRAM_ATTR spi_flash_restore_cache(uint32_t cpuid, uint32_t saved_state)
{
    esp_cache_resume_ext_mem_cache();
#if SOC_BRANCH_PREDICTOR_SUPPORTED
    esp_cpu_branch_prediction_enable();
#endif
}

bool IRAM_ATTR spi_flash_cache_enabled(void)
{
    return cache_hal_is_cache_enabled(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_ALL);
}

#if CONFIG_IDF_TARGET_ESP32S2
IRAM_ATTR void esp_config_instruction_cache_mode(void)
{
    cache_size_t cache_size;
    cache_ways_t cache_ways;
    cache_line_size_t cache_line_size;

#if CONFIG_ESP32S2_INSTRUCTION_CACHE_8KB
    Cache_Allocate_SRAM(CACHE_MEMORY_ICACHE_LOW, CACHE_MEMORY_INVALID, CACHE_MEMORY_INVALID, CACHE_MEMORY_INVALID);
    cache_size = CACHE_SIZE_8KB;
#else
    Cache_Allocate_SRAM(CACHE_MEMORY_ICACHE_LOW, CACHE_MEMORY_ICACHE_HIGH, CACHE_MEMORY_INVALID, CACHE_MEMORY_INVALID);
    cache_size = CACHE_SIZE_16KB;
#endif
    cache_ways = CACHE_4WAYS_ASSOC;
#if CONFIG_ESP32S2_INSTRUCTION_CACHE_LINE_16B
    cache_line_size = CACHE_LINE_SIZE_16B;
#else
    cache_line_size = CACHE_LINE_SIZE_32B;
#endif
    ESP_EARLY_LOGI(TAG, "Instruction cache \t: size %dKB, %dWays, cache line size %dByte", cache_size == CACHE_SIZE_8KB ? 8 : 16, 4, cache_line_size == CACHE_LINE_SIZE_16B ? 16 : 32);
    Cache_Suspend_ICache();
    Cache_Set_ICache_Mode(cache_size, cache_ways, cache_line_size);
    Cache_Invalidate_ICache_All();
    Cache_Resume_ICache(0);
}

IRAM_ATTR void esp_config_data_cache_mode(void)
{
#define CACHE_SIZE_0KB  99  //If Cache set to 0 KB, cache is bypassed, the cache size doesn't take into effect. Set this macro to a unique value for log

    cache_size_t cache_size;
    cache_ways_t cache_ways;
    cache_line_size_t cache_line_size;

#if CONFIG_ESP32S2_INSTRUCTION_CACHE_8KB
#if CONFIG_ESP32S2_DATA_CACHE_0KB
    Cache_Allocate_SRAM(CACHE_MEMORY_ICACHE_LOW, CACHE_MEMORY_INVALID, CACHE_MEMORY_INVALID, CACHE_MEMORY_INVALID);
    cache_size = CACHE_SIZE_0KB;
#elif CONFIG_ESP32S2_DATA_CACHE_8KB
    Cache_Allocate_SRAM(CACHE_MEMORY_ICACHE_LOW, CACHE_MEMORY_DCACHE_LOW, CACHE_MEMORY_INVALID, CACHE_MEMORY_INVALID);
    cache_size = CACHE_SIZE_8KB;
#else
    Cache_Allocate_SRAM(CACHE_MEMORY_ICACHE_LOW, CACHE_MEMORY_DCACHE_LOW, CACHE_MEMORY_DCACHE_HIGH, CACHE_MEMORY_INVALID);
    cache_size = CACHE_SIZE_16KB;
#endif
#else
#if CONFIG_ESP32S2_DATA_CACHE_0KB
    Cache_Allocate_SRAM(CACHE_MEMORY_ICACHE_LOW, CACHE_MEMORY_ICACHE_HIGH, CACHE_MEMORY_INVALID, CACHE_MEMORY_INVALID);
    cache_size = CACHE_SIZE_0KB;
#elif CONFIG_ESP32S2_DATA_CACHE_8KB
    Cache_Allocate_SRAM(CACHE_MEMORY_ICACHE_LOW, CACHE_MEMORY_ICACHE_HIGH, CACHE_MEMORY_DCACHE_LOW, CACHE_MEMORY_INVALID);
    cache_size = CACHE_SIZE_8KB;
#else
    Cache_Allocate_SRAM(CACHE_MEMORY_ICACHE_LOW, CACHE_MEMORY_ICACHE_HIGH, CACHE_MEMORY_DCACHE_LOW, CACHE_MEMORY_DCACHE_HIGH);
    cache_size = CACHE_SIZE_16KB;
#endif
#endif

    cache_ways = CACHE_4WAYS_ASSOC;
#if CONFIG_ESP32S2_DATA_CACHE_LINE_16B
    cache_line_size = CACHE_LINE_SIZE_16B;
#else
    cache_line_size = CACHE_LINE_SIZE_32B;
#endif
    ESP_EARLY_LOGI(TAG, "Data cache \t\t: size %dKB, %dWays, cache line size %dByte", (cache_size == CACHE_SIZE_0KB) ? 0 : ((cache_size == CACHE_SIZE_8KB) ? 8 : 16), 4, cache_line_size == CACHE_LINE_SIZE_16B ? 16 : 32);
    Cache_Set_DCache_Mode(cache_size, cache_ways, cache_line_size);
    Cache_Invalidate_DCache_All();
}

static IRAM_ATTR void esp_enable_cache_flash_wrap(bool icache, bool dcache)
{
    uint32_t i_autoload, d_autoload;
    if (icache) {
        i_autoload = Cache_Suspend_ICache();
    }
    if (dcache) {
        d_autoload = Cache_Suspend_DCache();
    }
    REG_SET_BIT(EXTMEM_PRO_CACHE_WRAP_AROUND_CTRL_REG, EXTMEM_PRO_CACHE_FLASH_WRAP_AROUND);
    if (icache) {
        Cache_Resume_ICache(i_autoload);
    }
    if (dcache) {
        Cache_Resume_DCache(d_autoload);
    }
}

#if (CONFIG_IDF_TARGET_ESP32S2 && CONFIG_SPIRAM)
static IRAM_ATTR void esp_enable_cache_spiram_wrap(bool icache, bool dcache)
{
    uint32_t i_autoload, d_autoload;
    if (icache) {
        i_autoload = Cache_Suspend_ICache();
    }
    if (dcache) {
        d_autoload = Cache_Suspend_DCache();
    }
    REG_SET_BIT(EXTMEM_PRO_CACHE_WRAP_AROUND_CTRL_REG, EXTMEM_PRO_CACHE_SRAM_RD_WRAP_AROUND);
    if (icache) {
        Cache_Resume_ICache(i_autoload);
    }
    if (dcache) {
        Cache_Resume_DCache(d_autoload);
    }
}
#endif

esp_err_t esp_enable_cache_wrap(bool icache_wrap_enable, bool dcache_wrap_enable)
{
    int icache_wrap_size = 0, dcache_wrap_size = 0;
    int flash_wrap_sizes[2] = {-1, -1}, spiram_wrap_sizes[2] = {-1, -1};
    int flash_wrap_size = 0, spiram_wrap_size = 0;
    int flash_count = 0, spiram_count = 0;
    int i;
    bool flash_spiram_wrap_together, flash_support_wrap = true, spiram_support_wrap = true;
    uint32_t drom0_in_icache = 1;//always 1 in esp32s2
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32P4 || CONFIG_IDF_TARGET_ESP32C61 //TODO: IDF-4307
    drom0_in_icache = 0;
#endif

    if (icache_wrap_enable) {
#if CONFIG_ESP32S2_INSTRUCTION_CACHE_LINE_16B || CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_16B
        icache_wrap_size = FLASH_WRAP_SIZE_16B;
#else
        icache_wrap_size = FLASH_WRAP_SIZE_32B;
#endif
    }
    if (dcache_wrap_enable) {
#if CONFIG_ESP32S2_DATA_CACHE_LINE_16B || CONFIG_ESP32S3_DATA_CACHE_LINE_16B
        dcache_wrap_size = FLASH_WRAP_SIZE_16B;
#else
        dcache_wrap_size = FLASH_WRAP_SIZE_32B;
#endif
    }

    uint32_t instruction_use_spiram = 0;
    uint32_t rodata_use_spiram = 0;
#if CONFIG_SPIRAM_FETCH_INSTRUCTIONS
    extern uint32_t esp_spiram_instruction_access_enabled(void);
    instruction_use_spiram = esp_spiram_instruction_access_enabled();
#endif
#if CONFIG_SPIRAM_RODATA
    extern uint32_t esp_spiram_rodata_access_enabled(void);
    rodata_use_spiram = esp_spiram_rodata_access_enabled();
#endif

    if (instruction_use_spiram) {
        spiram_wrap_sizes[0] = icache_wrap_size;
    } else {
        flash_wrap_sizes[0] = icache_wrap_size;
    }
    if (rodata_use_spiram) {
        if (drom0_in_icache) {
            spiram_wrap_sizes[0] = icache_wrap_size;
        } else {
            spiram_wrap_sizes[1] = dcache_wrap_size;
            flash_wrap_sizes[1] = dcache_wrap_size;
        }
    } else {
        if (drom0_in_icache) {
            flash_wrap_sizes[0] = icache_wrap_size;
        } else {
            flash_wrap_sizes[1] = dcache_wrap_size;
        }
    }
#if (CONFIG_IDF_TARGET_ESP32S2 && CONFIG_SPIRAM)
    spiram_wrap_sizes[1] = dcache_wrap_size;
#endif
    for (i = 0; i < 2; i++) {
        if (flash_wrap_sizes[i] != -1) {
            flash_count++;
            flash_wrap_size = flash_wrap_sizes[i];
        }
    }
    for (i = 0; i < 2; i++) {
        if (spiram_wrap_sizes[i] != -1) {
            spiram_count++;
            spiram_wrap_size = spiram_wrap_sizes[i];
        }
    }
    if (flash_count + spiram_count <= 2) {
        flash_spiram_wrap_together = false;
    } else {
        flash_spiram_wrap_together = true;
    }
    ESP_EARLY_LOGI(TAG, "flash_count=%d, size=%d, spiram_count=%d, size=%d,together=%d", flash_count, flash_wrap_size, spiram_count, spiram_wrap_size, flash_spiram_wrap_together);
    if (flash_count > 1 && flash_wrap_sizes[0] != flash_wrap_sizes[1]) {
        ESP_EARLY_LOGW(TAG, "Flash wrap with different length %d and %d, abort wrap.", flash_wrap_sizes[0], flash_wrap_sizes[1]);
        if (spiram_wrap_size == 0) {
            return ESP_FAIL;
        }
        if (flash_spiram_wrap_together) {
            ESP_EARLY_LOGE(TAG, "Abort spiram wrap because flash wrap length not fixed.");
            return ESP_FAIL;
        }
    }
    if (spiram_count > 1 && spiram_wrap_sizes[0] != spiram_wrap_sizes[1]) {
        ESP_EARLY_LOGW(TAG, "SPIRAM wrap with different length %d and %d, abort wrap.", spiram_wrap_sizes[0], spiram_wrap_sizes[1]);
        if (flash_wrap_size == 0) {
            return ESP_FAIL;
        }
        if (flash_spiram_wrap_together) {
            ESP_EARLY_LOGW(TAG, "Abort flash wrap because spiram wrap length not fixed.");
            return ESP_FAIL;
        }
    }

    if (flash_spiram_wrap_together && flash_wrap_size != spiram_wrap_size) {
        ESP_EARLY_LOGW(TAG, "SPIRAM has different wrap length with flash, %d and %d, abort wrap.", spiram_wrap_size, flash_wrap_size);
        return ESP_FAIL;
    }

#ifdef CONFIG_ESPTOOLPY_FLASHMODE_QIO
    flash_support_wrap = true;
    spi_flash_wrap_probe();
    if (!spi_flash_support_wrap_size(flash_wrap_size)) {
        flash_support_wrap = false;
        ESP_EARLY_LOGW(TAG, "Flash do not support wrap size %d.", flash_wrap_size);
    }
#else
    ESP_EARLY_LOGW(TAG, "Flash is not in QIO mode, do not support wrap.");
#endif

#if (CONFIG_IDF_TARGET_ESP32S2 && CONFIG_SPIRAM)
    extern bool psram_support_wrap_size(uint32_t wrap_size);
    if (!psram_support_wrap_size(spiram_wrap_size)) {
        spiram_support_wrap = false;
        ESP_EARLY_LOGW(TAG, "SPIRAM do not support wrap size %d.", spiram_wrap_size);
    }
#endif

    if (flash_spiram_wrap_together && !(flash_support_wrap && spiram_support_wrap)) {
        ESP_EARLY_LOGW(TAG, "Flash and SPIRAM should support wrap together.");
        return ESP_FAIL;
    }

    if (flash_support_wrap && flash_wrap_size > 0) {
        ESP_EARLY_LOGI(TAG, "Flash wrap enabled, size = %d.", flash_wrap_size);
        spi_flash_wrap_enable(flash_wrap_size);
        esp_enable_cache_flash_wrap((flash_wrap_sizes[0] > 0), (flash_wrap_sizes[1] > 0));
    }
#if (CONFIG_IDF_TARGET_ESP32S2 && CONFIG_SPIRAM)
    extern esp_err_t psram_enable_wrap(uint32_t wrap_size);
    if (spiram_support_wrap && spiram_wrap_size > 0) {
        ESP_EARLY_LOGI(TAG, "SPIRAM wrap enabled, size = %d.", spiram_wrap_size);
        psram_enable_wrap(spiram_wrap_size);
        esp_enable_cache_spiram_wrap((spiram_wrap_sizes[0] > 0), (spiram_wrap_sizes[1] > 0));
    }
#endif

    return ESP_OK;

}
#endif
#if CONFIG_IDF_TARGET_ESP32S3
IRAM_ATTR void esp_config_instruction_cache_mode(void)
{
    cache_size_t cache_size;
    cache_ways_t cache_ways;
    cache_line_size_t cache_line_size;

#if CONFIG_ESP32S3_INSTRUCTION_CACHE_16KB
    Cache_Occupy_ICache_MEMORY(CACHE_MEMORY_IBANK0, CACHE_MEMORY_INVALID);
    cache_size = CACHE_SIZE_HALF;
#else
    Cache_Occupy_ICache_MEMORY(CACHE_MEMORY_IBANK0, CACHE_MEMORY_IBANK1);
    cache_size = CACHE_SIZE_FULL;
#endif
#if CONFIG_ESP32S3_INSTRUCTION_CACHE_4WAYS
    cache_ways = CACHE_4WAYS_ASSOC;
#else
    cache_ways = CACHE_8WAYS_ASSOC;
#endif
#if CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_16B
    cache_line_size = CACHE_LINE_SIZE_16B;
#elif CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_32B
    cache_line_size = CACHE_LINE_SIZE_32B;
#else
    cache_line_size = CACHE_LINE_SIZE_64B;
#endif
    ESP_EARLY_LOGI(TAG, "Instruction cache: size %dKB, %dWays, cache line size %dByte", cache_size == CACHE_SIZE_HALF ? 16 : 32, cache_ways == CACHE_4WAYS_ASSOC ? 4 : 8, cache_line_size == CACHE_LINE_SIZE_16B ? 16 : (cache_line_size == CACHE_LINE_SIZE_32B ? 32 : 64));
    Cache_Set_ICache_Mode(cache_size, cache_ways, cache_line_size);
    Cache_Invalidate_ICache_All();
    extern void Cache_Enable_ICache(uint32_t autoload);
    Cache_Enable_ICache(0);
}

IRAM_ATTR void esp_config_data_cache_mode(void)
{
    cache_size_t cache_size;
    cache_ways_t cache_ways;
    cache_line_size_t cache_line_size;

#if CONFIG_ESP32S3_DATA_CACHE_32KB
    Cache_Occupy_DCache_MEMORY(CACHE_MEMORY_DBANK1, CACHE_MEMORY_INVALID);
    cache_size = CACHE_SIZE_HALF;
#else
    Cache_Occupy_DCache_MEMORY(CACHE_MEMORY_DBANK0, CACHE_MEMORY_DBANK1);
    cache_size = CACHE_SIZE_FULL;
#endif
#if CONFIG_ESP32S3_DATA_CACHE_4WAYS
    cache_ways = CACHE_4WAYS_ASSOC;
#else
    cache_ways = CACHE_8WAYS_ASSOC;
#endif
#if CONFIG_ESP32S3_DATA_CACHE_LINE_16B
    cache_line_size = CACHE_LINE_SIZE_16B;
#elif CONFIG_ESP32S3_DATA_CACHE_LINE_32B
    cache_line_size = CACHE_LINE_SIZE_32B;
#else
    cache_line_size = CACHE_LINE_SIZE_64B;
#endif
    Cache_Set_DCache_Mode(cache_size, cache_ways, cache_line_size);
    Cache_Invalidate_DCache_All();
}

static IRAM_ATTR void esp_enable_cache_flash_wrap(bool icache, bool dcache)
{
    uint32_t i_autoload, d_autoload;
    if (icache) {
        i_autoload = Cache_Suspend_ICache();
    }
    if (dcache) {
        d_autoload = Cache_Suspend_DCache();
    }
    REG_SET_BIT(EXTMEM_CACHE_WRAP_AROUND_CTRL_REG, EXTMEM_CACHE_FLASH_WRAP_AROUND);
    if (icache) {
        Cache_Resume_ICache(i_autoload);
    }
    if (dcache) {
        Cache_Resume_DCache(d_autoload);
    }
}

#if (CONFIG_IDF_TARGET_ESP32S3 && CONFIG_SPIRAM)
static IRAM_ATTR void esp_enable_cache_spiram_wrap(bool icache, bool dcache)
{
    uint32_t i_autoload, d_autoload;
    if (icache) {
        i_autoload = Cache_Suspend_ICache();
    }
    if (dcache) {
        d_autoload = Cache_Suspend_DCache();
    }
    REG_SET_BIT(EXTMEM_CACHE_WRAP_AROUND_CTRL_REG, EXTMEM_CACHE_SRAM_RD_WRAP_AROUND);
    if (icache) {
        Cache_Resume_ICache(i_autoload);
    }
    if (dcache) {
        Cache_Resume_DCache(d_autoload);
    }
}
#endif

esp_err_t esp_enable_cache_wrap(bool icache_wrap_enable, bool dcache_wrap_enable)
{
    int icache_wrap_size = 0, dcache_wrap_size = 0;
    int flash_wrap_sizes[2] = {-1, -1}, spiram_wrap_sizes[2] = {-1, -1};
    int flash_wrap_size = 0, spiram_wrap_size = 0;
    int flash_count = 0, spiram_count = 0;
    int i;
    bool flash_spiram_wrap_together, flash_support_wrap = false, spiram_support_wrap = true;
    uint32_t drom0_in_icache = 0;//always 0 in chip7.2.4

    if (icache_wrap_enable) {
#if CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_16B
        icache_wrap_size = FLASH_WRAP_SIZE_16B;
#elif CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_32B
        icache_wrap_size = FLASH_WRAP_SIZE_32B;
#else
        icache_wrap_size = FLASH_WRAP_SIZE_64B;
#endif
    }
    if (dcache_wrap_enable) {
#if CONFIG_ESP32S3_DATA_CACHE_LINE_16B
        dcache_wrap_size = FLASH_WRAP_SIZE_16B;
#elif CONFIG_ESP32S3_DATA_CACHE_LINE_32B
        dcache_wrap_size = FLASH_WRAP_SIZE_32B;
#else
        dcache_wrap_size = FLASH_WRAP_SIZE_64B;
#endif
    }

    uint32_t instruction_use_spiram = 0;
    uint32_t rodata_use_spiram = 0;
#if CONFIG_SPIRAM_FETCH_INSTRUCTIONS
    extern uint32_t esp_spiram_instruction_access_enabled(void);
    instruction_use_spiram = esp_spiram_instruction_access_enabled();
#endif
#if CONFIG_SPIRAM_RODATA
    extern uint32_t esp_spiram_rodata_access_enabled(void);
    rodata_use_spiram = esp_spiram_rodata_access_enabled();
#endif

    if (instruction_use_spiram) {
        spiram_wrap_sizes[0] = icache_wrap_size;
    } else {
        flash_wrap_sizes[0] = icache_wrap_size;
    }
    if (rodata_use_spiram) {
        if (drom0_in_icache) {
            spiram_wrap_sizes[0] = icache_wrap_size;
        } else {
            spiram_wrap_sizes[1] = dcache_wrap_size;
        }
    } else {
        if (drom0_in_icache) {
            flash_wrap_sizes[0] = icache_wrap_size;
        } else {
            flash_wrap_sizes[1] = dcache_wrap_size;
        }
    }
#if (CONFIG_IDF_TARGET_ESP32S3 && CONFIG_SPIRAM)
    spiram_wrap_sizes[1] = dcache_wrap_size;
#endif
    for (i = 0; i < 2; i++) {
        if (flash_wrap_sizes[i] != -1) {
            flash_count++;
            flash_wrap_size = flash_wrap_sizes[i];
        }
    }
    for (i = 0; i < 2; i++) {
        if (spiram_wrap_sizes[i] != -1) {
            spiram_count++;
            spiram_wrap_size = spiram_wrap_sizes[i];
        }
    }
    if (flash_count + spiram_count <= 2) {
        flash_spiram_wrap_together = false;
    } else {
        flash_spiram_wrap_together = true;
    }
    if (flash_count > 1 && flash_wrap_sizes[0] != flash_wrap_sizes[1]) {
        ESP_EARLY_LOGW(TAG, "Flash wrap with different length %d and %d, abort wrap.", flash_wrap_sizes[0], flash_wrap_sizes[1]);
        if (spiram_wrap_size == 0) {
            return ESP_FAIL;
        }
        if (flash_spiram_wrap_together) {
            ESP_EARLY_LOGE(TAG, "Abort spiram wrap because flash wrap length not fixed.");
            return ESP_FAIL;
        }
    }
    if (spiram_count > 1 && spiram_wrap_sizes[0] != spiram_wrap_sizes[1]) {
        ESP_EARLY_LOGW(TAG, "SPIRAM wrap with different length %d and %d, abort wrap.", spiram_wrap_sizes[0], spiram_wrap_sizes[1]);
        if (flash_wrap_size == 0) {
            return ESP_FAIL;
        }
        if (flash_spiram_wrap_together) {
            ESP_EARLY_LOGW(TAG, "Abort flash wrap because spiram wrap length not fixed.");
            return ESP_FAIL;
        }
    }

    if (flash_spiram_wrap_together && flash_wrap_size != spiram_wrap_size) {
        ESP_EARLY_LOGW(TAG, "SPIRAM has different wrap length with flash, %d and %d, abort wrap.", spiram_wrap_size, flash_wrap_size);
        return ESP_FAIL;
    }

#ifdef CONFIG_ESPTOOLPY_FLASHMODE_QIO
    flash_support_wrap = true;
    spi_flash_wrap_probe();
    if (!spi_flash_support_wrap_size(flash_wrap_size)) {
        flash_support_wrap = false;
        ESP_EARLY_LOGW(TAG, "Flash do not support wrap size %d.", flash_wrap_size);
    }
#else
    ESP_EARLY_LOGW(TAG, "Flash is not in QIO mode, do not support wrap.");
#endif


#if (CONFIG_IDF_TARGET_ESP32S3 && CONFIG_SPIRAM)
    extern bool psram_support_wrap_size(uint32_t wrap_size);
    if (!psram_support_wrap_size(spiram_wrap_size)) {
        spiram_support_wrap = false;
        ESP_EARLY_LOGW(TAG, "SPIRAM do not support wrap size %d.", spiram_wrap_size);
    }
#endif

    if (flash_spiram_wrap_together && !(flash_support_wrap && spiram_support_wrap)) {
        ESP_EARLY_LOGW(TAG, "Flash and SPIRAM should support wrap together.");
        return ESP_FAIL;
    }

    if (flash_support_wrap && flash_wrap_size > 0) {
        ESP_EARLY_LOGI(TAG, "Flash wrap enabled, size = %d.", flash_wrap_size);
        spi_flash_wrap_enable(flash_wrap_size);
        esp_enable_cache_flash_wrap((flash_wrap_sizes[0] > 0), (flash_wrap_sizes[1] > 0));
    }
#if (CONFIG_IDF_TARGET_ESP32S3 && CONFIG_SPIRAM)
    extern esp_err_t psram_enable_wrap(uint32_t wrap_size);
    if (spiram_support_wrap && spiram_wrap_size > 0) {
        ESP_EARLY_LOGI(TAG, "SPIRAM wrap enabled, size = %d.", spiram_wrap_size);
        psram_enable_wrap(spiram_wrap_size);
        esp_enable_cache_spiram_wrap((spiram_wrap_sizes[0] > 0), (spiram_wrap_sizes[1] > 0));
    }
#endif

    return ESP_OK;

}
#endif

#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2

static IRAM_ATTR void esp_enable_cache_flash_wrap(bool icache)
{
    uint32_t i_autoload;
    if (icache) {
        i_autoload = Cache_Suspend_ICache();
    }
    REG_SET_BIT(EXTMEM_CACHE_WRAP_AROUND_CTRL_REG, EXTMEM_CACHE_FLASH_WRAP_AROUND);
    if (icache) {
        Cache_Resume_ICache(i_autoload);
    }
}

esp_err_t esp_enable_cache_wrap(bool icache_wrap_enable)
{
    int flash_wrap_size = 0;
    bool flash_support_wrap = false;

    if (icache_wrap_enable) {
        flash_wrap_size = 32;
    }

#ifdef CONFIG_ESPTOOLPY_FLASHMODE_QIO
    flash_support_wrap = true;
    spi_flash_wrap_probe();
    if (!spi_flash_support_wrap_size(flash_wrap_size)) {
        flash_support_wrap = false;
        ESP_EARLY_LOGW(TAG, "Flash do not support wrap size %d.", flash_wrap_size);
    }
#else
    ESP_EARLY_LOGW(TAG, "Flash is not in QIO mode, do not support wrap.");
#endif // CONFIG_ESPTOOLPY_FLASHMODE_QIO

    if (flash_support_wrap && flash_wrap_size > 0) {
        ESP_EARLY_LOGI(TAG, "Flash wrap enabled, size = %d.", flash_wrap_size);
        spi_flash_wrap_enable(flash_wrap_size);
        esp_enable_cache_flash_wrap((flash_wrap_size > 0));
    }
    return ESP_OK;
}
#endif // CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2
