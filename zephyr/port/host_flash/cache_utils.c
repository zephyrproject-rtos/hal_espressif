/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <host_flash/cache_utils.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#if CONFIG_SOC_SERIES_ESP32S2
#include "esp32s2/rom/cache.h"
#include <soc/extmem_reg.h>
#include "soc/ext_mem_defs.h"
#elif CONFIG_SOC_SERIES_ESP32S3
#include "esp32s3/rom/cache.h"
#include "soc/extmem_reg.h"
#include "soc/ext_mem_defs.h"
#elif CONFIG_SOC_SERIES_ESP32C3
#include "esp32c3/rom/cache.h"
#include "soc/extmem_reg.h"
#include "soc/ext_mem_defs.h"
#endif

#define DPORT_CACHE_BIT(cpuid, regid) DPORT_ ## cpuid ## regid

#define DPORT_CACHE_MASK(cpuid) (DPORT_CACHE_BIT(cpuid, _CACHE_MASK_OPSDRAM) | DPORT_CACHE_BIT(cpuid, _CACHE_MASK_DROM0) | \
                                DPORT_CACHE_BIT(cpuid, _CACHE_MASK_DRAM1) | DPORT_CACHE_BIT(cpuid, _CACHE_MASK_IROM0) | \
                                DPORT_CACHE_BIT(cpuid, _CACHE_MASK_IRAM1) | DPORT_CACHE_BIT(cpuid, _CACHE_MASK_IRAM0) )

#define DPORT_CACHE_VAL(cpuid) (~(DPORT_CACHE_BIT(cpuid, _CACHE_MASK_DROM0) | \
                                        DPORT_CACHE_BIT(cpuid, _CACHE_MASK_DRAM1) | \
                                        DPORT_CACHE_BIT(cpuid, _CACHE_MASK_IRAM0)))

#define DPORT_CACHE_GET_VAL(cpuid) (cpuid == 0) ? DPORT_CACHE_VAL(PRO) : DPORT_CACHE_VAL(APP)
#define DPORT_CACHE_GET_MASK(cpuid) (cpuid == 0) ? DPORT_CACHE_MASK(PRO) : DPORT_CACHE_MASK(APP)

static void spi_flash_disable_cache(uint32_t cpuid, uint32_t *saved_state);
static void spi_flash_restore_cache(uint32_t cpuid, uint32_t saved_state);


static uint32_t s_cache_ops_saved_state[2];
static unsigned int s_intr_saved_state;

static void IRAM_ATTR spi_flash_disable_cache(uint32_t cpuid, uint32_t *saved_state)
{
#if CONFIG_SOC_SERIES_ESP32
	uint32_t ret = 0;
	const uint32_t cache_mask = DPORT_CACHE_GET_MASK(cpuid);
	if (cpuid == 0) {
		ret |= DPORT_GET_PERI_REG_BITS2(DPORT_PRO_CACHE_CTRL1_REG, cache_mask, 0);
		while (DPORT_GET_PERI_REG_BITS2(DPORT_PRO_DCACHE_DBUG0_REG, DPORT_PRO_CACHE_STATE, DPORT_PRO_CACHE_STATE_S) != 1) {
			;
		}
		DPORT_SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL_REG, 1, 0, DPORT_PRO_CACHE_ENABLE_S);
	}
#if CONFIG_SMP
	else {
		ret |= DPORT_GET_PERI_REG_BITS2(DPORT_APP_CACHE_CTRL1_REG, cache_mask, 0);
		while (DPORT_GET_PERI_REG_BITS2(DPORT_APP_DCACHE_DBUG0_REG, DPORT_APP_CACHE_STATE, DPORT_APP_CACHE_STATE_S) != 1) {
			;
		}
		DPORT_SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL_REG, 1, 0, DPORT_APP_CACHE_ENABLE_S);
	}
#endif
	*saved_state = ret;
#elif CONFIG_SOC_SERIES_ESP32S2
	*saved_state = Cache_Suspend_ICache();
#elif CONFIG_SOC_SERIES_ESP32S3
	uint32_t icache_state, dcache_state;
	icache_state = Cache_Suspend_ICache() << 16;
	dcache_state = Cache_Suspend_DCache();
	*saved_state = icache_state | dcache_state;
#elif CONFIG_SOC_SERIES_ESP32C3
	uint32_t icache_state;
	icache_state = Cache_Suspend_ICache() << 16;
	*saved_state = icache_state;
#endif /* CONFIG_SOC_SERIES_ESP32xx */
}

static void IRAM_ATTR spi_flash_restore_cache(uint32_t cpuid, uint32_t saved_state)
{
#if CONFIG_SOC_SERIES_ESP32
	const uint32_t cache_mask = DPORT_CACHE_GET_MASK(cpuid);
	if (cpuid == 0) {
		DPORT_SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL_REG, 1, 1, DPORT_PRO_CACHE_ENABLE_S);
		DPORT_SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL1_REG, cache_mask, saved_state, 0);
	}
#if CONFIG_SMP
	else {
		DPORT_SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL_REG, 1, 1, DPORT_APP_CACHE_ENABLE_S);
		DPORT_SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL1_REG, cache_mask, saved_state, 0);
	}
#endif
#elif CONFIG_SOC_SERIES_ESP32S2
	Cache_Resume_ICache(saved_state);
#elif CONFIG_SOC_SERIES_ESP32S3
	Cache_Resume_DCache(saved_state & 0xffff);
	Cache_Resume_ICache(saved_state >> 16);
#elif CONFIG_SOC_SERIES_ESP32C3
	Cache_Resume_ICache(saved_state >> 16);
#endif /* CONFIG_SOC_SERIES_ESP32xx */
}

void IRAM_ATTR spi_flash_disable_interrupts_caches_and_other_cpu(void)
{
#ifdef CONFIG_MULTITHREADING
	if(!k_is_in_isr()) {
		k_sched_lock();
	}
#endif

	s_intr_saved_state = irq_lock();

#if !defined(CONFIG_SOC_SERIES_ESP32C3)
	esp_intr_noniram_disable();
#endif

#if !defined(CONFIG_SOC_SERIES_ESP32C3)
	int cpu_id = arch_curr_cpu()->id;
#else
	int cpu_id =  PRO_CPU_NUM;
#endif
	spi_flash_disable_cache(cpu_id, &s_cache_ops_saved_state[cpu_id]);

#ifdef CONFIG_SMP
	int other_cpu = (cpu_id == PRO_CPU_NUM) ? APP_CPU_NUM : PRO_CPU_NUM;
	spi_flash_disable_cache(other_cpu, &s_cache_ops_saved_state[other_cpu]);
#endif
}

void IRAM_ATTR spi_flash_enable_interrupts_caches_and_other_cpu(void)
{
#if !defined(CONFIG_SOC_SERIES_ESP32C3)
	int cpu_id = arch_curr_cpu()->id;
#else
	int cpu_id = PRO_CPU_NUM;
#endif
	spi_flash_restore_cache(cpu_id, s_cache_ops_saved_state[cpu_id]);

#ifdef CONFIG_SMP
	int other_cpu = (cpu_id == PRO_CPU_NUM) ? APP_CPU_NUM : PRO_CPU_NUM;
	spi_flash_restore_cache(other_cpu, s_cache_ops_saved_state[other_cpu]);
#endif

#if !defined(CONFIG_SOC_SERIES_ESP32C3)
	esp_intr_noniram_enable();
#endif

	irq_unlock(s_intr_saved_state);

#ifdef CONFIG_MULTITHREADING
	if(!k_is_in_isr()) {
		k_sched_unlock();
	}
#endif
}

K_MUTEX_DEFINE(s_flash_op_mutex);

void spi_flash_init_lock(void)
{
	return;
}

void spi_flash_op_lock(void)
{
    k_mutex_lock(&s_flash_op_mutex, K_FOREVER);
}

void spi_flash_op_unlock(void)
{
   k_mutex_unlock(&s_flash_op_mutex);
}

IRAM_ATTR bool spi_flash_cache_enabled(void)
{
#if CONFIG_SOC_SERIES_ESP32
    bool result = (DPORT_REG_GET_BIT(DPORT_PRO_CACHE_CTRL_REG, DPORT_PRO_CACHE_ENABLE) != 0);
#if portNUM_PROCESSORS == 2
    result = result && (DPORT_REG_GET_BIT(DPORT_APP_CACHE_CTRL_REG, DPORT_APP_CACHE_ENABLE) != 0);
#endif
#elif CONFIG_SOC_SERIES_ESP32S2
    bool result = (REG_GET_BIT(EXTMEM_PRO_ICACHE_CTRL_REG, EXTMEM_PRO_ICACHE_ENABLE) != 0);
#elif CONFIG_SOC_SERIES_ESP32S3 || CONFIG_SOC_SERIES_ESP32C3 || CONFIG_SOC_SERIES_ESP32C2
    bool result = (REG_GET_BIT(EXTMEM_ICACHE_CTRL_REG, EXTMEM_ICACHE_ENABLE) != 0);
#elif CONFIG_SOC_SERIES_ESP32C6 || CONFIG_SOC_SERIES_ESP32H2
    bool result = s_cache_enabled;
#endif
    return result;
}
