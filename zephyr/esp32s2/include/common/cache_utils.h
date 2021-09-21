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
	*saved_state = Cache_Suspend_ICache();
}

static void IRAM_ATTR esp32_restore_cache(uint32_t cpuid, uint32_t saved_state)
{
	Cache_Resume_ICache(saved_state);
}

void IRAM_ATTR esp32_spiflash_start(void)
{
	k_sched_lock();

	s_intr_saved_state = irq_lock();

	int cpu_id = arch_curr_cpu()->id;

	esp32_disable_cache(cpu_id, &s_cache_ops_saved_state[cpu_id]);
}

void IRAM_ATTR esp32_spiflash_end(void)
{
	int cpu_id = arch_curr_cpu()->id;

	esp32_restore_cache(cpu_id, s_cache_ops_saved_state[cpu_id]);

	irq_unlock(s_intr_saved_state);

	k_sched_unlock();
}

