/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <zephyr.h>

static uint32_t s_cache_ops_saved_state[2];
static unsigned int s_intr_saved_state;

static void IRAM_ATTR esp32_disable_cache(uint32_t cpuid, uint32_t *saved_state)
{
	/*
	 * TODO: this function should implement cache disabling
	 * functionality as per IDF's spi_flash_disable_cache
	 * function, however, Cache_Suspend_ICache hangs forever
	 * and the root cause was not found yet.
	 * The original implementation was left below as reference.
	 */
#if 0
	uint32_t icache_state = 0;

	icache_state = Cache_Suspend_ICache() << 16;
	*saved_state = icache_state;
#endif
}

static void IRAM_ATTR esp32_restore_cache(uint32_t cpuid, uint32_t saved_state)
{
	/*
	 * TODO: this function shall be activated once
	 * esp32_disable_cache is also made available
	 */
#if 0
	Cache_Resume_ICache(saved_state >> 16);
#endif
}

void IRAM_ATTR esp32_spiflash_start(void)
{
	k_sched_lock();

	s_intr_saved_state = irq_lock();

	int cpu_id = PRO_CPU_NUM;

	esp32_disable_cache(cpu_id, &s_cache_ops_saved_state[cpu_id]);
}

void IRAM_ATTR esp32_spiflash_end(void)
{
	int cpu_id = PRO_CPU_NUM;

	esp32_restore_cache(cpu_id, s_cache_ops_saved_state[cpu_id]);

	irq_unlock(s_intr_saved_state);

	k_sched_unlock();
}

