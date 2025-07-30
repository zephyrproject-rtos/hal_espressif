/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#if CONFIG_SOC_SERIES_ESP32S2
#include "esp32s2/rom/cache.h"
#include <soc/extmem_reg.h>
#include "soc/ext_mem_defs.h"
#elif CONFIG_SOC_SERIES_ESP32S3
#include "esp32s3/rom/cache.h"
#include "soc/extmem_reg.h"
#include "soc/ext_mem_defs.h"
#elif CONFIG_SOC_SERIES_ESP32C2
#include "esp32c2/rom/cache.h"
#include "soc/extmem_reg.h"
#include "soc/ext_mem_defs.h"
#elif CONFIG_SOC_SERIES_ESP32C3
#include "esp32c3/rom/cache.h"
#include "soc/extmem_reg.h"
#include "soc/ext_mem_defs.h"
#elif CONFIG_SOC_SERIES_ESP32C6
#include "esp32c6/rom/cache.h"
#include "soc/extmem_reg.h"
#include "soc/ext_mem_defs.h"
#elif CONFIG_SOC_SERIES_ESP32H2
#include "esp32h2/rom/cache.h"
#include "soc/extmem_reg.h"
#include "soc/ext_mem_defs.h"
#endif

#include "esp_private/spi_flash_os.h"
#include "spi_flash_override.h"
#include "esp_rom_spiflash.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#include "hal/cpu_ll.h"
#include <soc/soc.h>
#include "esp_log.h"

static __attribute__((unused)) const char *TAG = "cache";

static void spi_flash_disable_cache(uint32_t cpuid, uint32_t *saved_state);
static void spi_flash_restore_cache(uint32_t cpuid, uint32_t saved_state);

static uint32_t s_flash_op_cache_state[2];
static unsigned int s_intr_saved_state;

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

#endif

void IRAM_ATTR spi_flash_disable_interrupts_caches_and_other_cpu(void)
{
	s_intr_saved_state = irq_lock();
#if !defined(CONFIG_SOC_SERIES_ESP32C2) && !defined(CONFIG_SOC_SERIES_ESP32C3) &&                  \
	!defined(CONFIG_SOC_SERIES_ESP32C6) && !defined(CONFIG_SOC_SERIES_ESP32H2)
	esp_intr_noniram_disable();
#endif

#if !defined(CONFIG_SOC_SERIES_ESP32C2) && !defined(CONFIG_SOC_SERIES_ESP32C3) &&                  \
	!defined(CONFIG_SOC_SERIES_ESP32C6) && !defined(CONFIG_SOC_SERIES_ESP32H2)
	int cpu_id = esp_cpu_get_core_id();
#else
	int cpu_id = PRO_CPU_NUM;
#endif
	spi_flash_disable_cache(cpu_id, &s_flash_op_cache_state[cpu_id]);

#ifdef CONFIG_SMP
	int other_cpu = (cpu_id == PRO_CPU_NUM) ? APP_CPU_NUM : PRO_CPU_NUM;
	spi_flash_disable_cache(other_cpu, &s_flash_op_cache_state[other_cpu]);
#endif
}

void IRAM_ATTR spi_flash_enable_interrupts_caches_and_other_cpu(void)
{
#if !defined(CONFIG_SOC_SERIES_ESP32C2) && !defined(CONFIG_SOC_SERIES_ESP32C3) &&                  \
	!defined(CONFIG_SOC_SERIES_ESP32C6) && !defined(CONFIG_SOC_SERIES_ESP32H2)
	int cpu_id = esp_cpu_get_core_id();
#else
	int cpu_id = PRO_CPU_NUM;
#endif
	spi_flash_restore_cache(cpu_id, s_flash_op_cache_state[cpu_id]);

#if defined(CONFIG_SMP) && SOC_IDCACHE_PER_CORE
	int other_cpu = (cpu_id == PRO_CPU_NUM) ? APP_CPU_NUM : PRO_CPU_NUM;
	spi_flash_restore_cache(other_cpu, s_flash_op_cache_state[other_cpu]);
#endif

#if !defined(CONFIG_SOC_SERIES_ESP32C2) && !defined(CONFIG_SOC_SERIES_ESP32C3) &&                  \
	!defined(CONFIG_SOC_SERIES_ESP32C6) && !defined(CONFIG_SOC_SERIES_ESP32H2)
	esp_intr_noniram_enable();
#endif
	irq_unlock(s_intr_saved_state);
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

/**
 * The following two functions are replacements for Cache_Read_Disable and Cache_Read_Enable
 * function in ROM. They are used to work around a bug where Cache_Read_Disable requires a call to
 * Cache_Flush before Cache_Read_Enable, even if cached data was not modified.
 */
static void IRAM_ATTR spi_flash_disable_cache(uint32_t cpuid, uint32_t *saved_state)
{
	cache_hal_suspend(CACHE_TYPE_ALL);
}

static void IRAM_ATTR spi_flash_restore_cache(uint32_t cpuid, uint32_t saved_state)
{
	cache_hal_resume(CACHE_TYPE_ALL);
}

bool IRAM_ATTR spi_flash_cache_enabled(void)
{
	return cache_hal_is_cache_enabled(CACHE_TYPE_ALL);
}

#if CONFIG_IDF_TARGET_ESP32S2
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
	uint32_t drom0_in_icache = 1; // always 1 in esp32s2
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2 ||         \
	CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32H2
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
	ESP_EARLY_LOGI(TAG, "flash_count=%d, size=%d, spiram_count=%d, size=%d,together=%d",
		       flash_count, flash_wrap_size, spiram_count, spiram_wrap_size,
		       flash_spiram_wrap_together);
	if (flash_count > 1 && flash_wrap_sizes[0] != flash_wrap_sizes[1]) {
		ESP_EARLY_LOGW(TAG, "Flash wrap with different length %d and %d, abort wrap.",
			       flash_wrap_sizes[0], flash_wrap_sizes[1]);
		if (spiram_wrap_size == 0) {
			return ESP_FAIL;
		}
		if (flash_spiram_wrap_together) {
			ESP_EARLY_LOGE(TAG,
				       "Abort spiram wrap because flash wrap length not fixed.");
			return ESP_FAIL;
		}
	}
	if (spiram_count > 1 && spiram_wrap_sizes[0] != spiram_wrap_sizes[1]) {
		ESP_EARLY_LOGW(TAG, "SPIRAM wrap with different length %d and %d, abort wrap.",
			       spiram_wrap_sizes[0], spiram_wrap_sizes[1]);
		if (flash_wrap_size == 0) {
			return ESP_FAIL;
		}
		if (flash_spiram_wrap_together) {
			ESP_EARLY_LOGW(TAG,
				       "Abort flash wrap because spiram wrap length not fixed.");
			return ESP_FAIL;
		}
	}

	if (flash_spiram_wrap_together && flash_wrap_size != spiram_wrap_size) {
		ESP_EARLY_LOGW(
			TAG, "SPIRAM has different wrap length with flash, %d and %d, abort wrap.",
			spiram_wrap_size, flash_wrap_size);
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
		esp_enable_cache_spiram_wrap((spiram_wrap_sizes[0] > 0),
					     (spiram_wrap_sizes[1] > 0));
	}
#endif

	return ESP_OK;
}
#endif
#if CONFIG_IDF_TARGET_ESP32S3
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
