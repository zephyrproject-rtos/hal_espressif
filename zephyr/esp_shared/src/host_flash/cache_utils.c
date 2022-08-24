/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "host_flash/cache_utils.h"
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>

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
#if CONFIG_SOC_ESP32
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
#elif CONFIG_SOC_ESP32S2
	*saved_state = Cache_Suspend_ICache();
#elif CONFIG_SOC_ESP32S3
	uint32_t icache_state, dcache_state;
	icache_state = Cache_Suspend_ICache() << 16;
	dcache_state = Cache_Suspend_DCache();
	*saved_state = icache_state | dcache_state;
#elif CONFIG_SOC_ESP32C3
	uint32_t icache_state;
	icache_state = Cache_Suspend_ICache() << 16;
	*saved_state = icache_state;
#endif
}

static void IRAM_ATTR spi_flash_restore_cache(uint32_t cpuid, uint32_t saved_state)
{
#if CONFIG_SOC_ESP32
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
#elif CONFIG_IDF_TARGET_ESP32S2
	Cache_Resume_ICache(saved_state);
#elif CONFIG_IDF_TARGET_ESP32S3
	Cache_Resume_DCache(saved_state & 0xffff);
	Cache_Resume_ICache(saved_state >> 16);
#elif CONFIG_IDF_TARGET_ESP32C3
	Cache_Resume_ICache(saved_state >> 16);
#endif
}

void IRAM_ATTR spi_flash_disable_interrupts_caches_and_other_cpu(void)
{
	k_sched_lock();

	s_intr_saved_state = irq_lock();

#if !defined(CONFIG_SOC_ESP32C3)
	esp_intr_noniram_disable();
#endif

#if !defined(CONFIG_SOC_ESP32C3)
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
#if !defined(CONFIG_SOC_ESP32C3)
	int cpu_id = arch_curr_cpu()->id;
#else
	int cpu_id = PRO_CPU_NUM;
#endif
	spi_flash_restore_cache(cpu_id, s_cache_ops_saved_state[cpu_id]);

#ifdef CONFIG_SMP
	int other_cpu = (cpu_id == PRO_CPU_NUM) ? APP_CPU_NUM : PRO_CPU_NUM;
	spi_flash_restore_cache(other_cpu, s_cache_ops_saved_state[other_cpu]);
#endif

#if !defined(CONFIG_SOC_ESP32C3)
	esp_intr_noniram_enable();
#endif

	irq_unlock(s_intr_saved_state);

	k_sched_unlock();
}

