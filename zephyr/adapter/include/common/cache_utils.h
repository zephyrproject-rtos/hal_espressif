/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <zephyr.h>
#include "soc/dport_reg.h"

#define DPORT_CACHE_MASK 0x3f

static uint32_t s_cache_ops_saved_state[2];
static unsigned int s_intr_saved_state;

static void IRAM_ATTR esp32_disable_cache(uint32_t cpuid, uint32_t *saved_state)
{
	uint32_t ret = 0;

	if (cpuid == PRO_CPU_NUM) {
		ret |= DPORT_GET_PERI_REG_BITS2(DPORT_PRO_CACHE_CTRL1_REG, DPORT_CACHE_MASK, 0);
		while (DPORT_GET_PERI_REG_BITS2(DPORT_PRO_DCACHE_DBUG0_REG, DPORT_PRO_CACHE_STATE, DPORT_PRO_CACHE_STATE_S) != 1) {
			;
		}
		DPORT_SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL_REG, 1, 0, DPORT_PRO_CACHE_ENABLE_S);
	} else   {
		ret |= DPORT_GET_PERI_REG_BITS2(DPORT_APP_CACHE_CTRL1_REG, DPORT_CACHE_MASK, 0);
		while (DPORT_GET_PERI_REG_BITS2(DPORT_APP_DCACHE_DBUG0_REG, DPORT_APP_CACHE_STATE, DPORT_APP_CACHE_STATE_S) != 1) {
			;
		}
		DPORT_SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL_REG, 1, 0, DPORT_APP_CACHE_ENABLE_S);
	}
	*saved_state = ret;
}

static void IRAM_ATTR esp32_restore_cache(uint32_t cpuid, uint32_t saved_state)
{
	if (cpuid == PRO_CPU_NUM) {
		DPORT_SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL_REG, 1, 1, DPORT_PRO_CACHE_ENABLE_S);
		DPORT_SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL1_REG, DPORT_CACHE_MASK, saved_state, 0);
	} else   {
		DPORT_SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL_REG, 1, 1, DPORT_APP_CACHE_ENABLE_S);
		DPORT_SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL1_REG, DPORT_CACHE_MASK, saved_state, 0);
	}
}

void IRAM_ATTR esp32_spiflash_start(void)
{
	k_sched_lock();

	s_intr_saved_state = irq_lock();

	int cpu_id = arch_curr_cpu()->id;
	esp32_disable_cache(cpu_id, &s_cache_ops_saved_state[cpu_id]);

#ifdef CONFIG_SMP
	int other_cpu = (cpu_id == PRO_CPU_NUM) ? APP_CPU_NUM : PRO_CPU_NUM;
	esp32_disable_cache(other_cpu, &s_cache_ops_saved_state[other_cpu]);
#endif
}

void IRAM_ATTR esp32_spiflash_end(void)
{
	int cpu_id = arch_curr_cpu()->id;

	esp32_restore_cache(cpu_id, s_cache_ops_saved_state[cpu_id]);

#ifdef CONFIG_SMP
	int other_cpu = (cpu_id == PRO_CPU_NUM) ? APP_CPU_NUM : PRO_CPU_NUM;
	esp32_restore_cache(other_cpu, s_cache_ops_saved_state[other_cpu]);
#endif
	irq_unlock(s_intr_saved_state);

	k_sched_unlock();
}

