/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"
#include "soc/dport_access.h"

uint32_t IRAM_ATTR esp_dport_access_reg_read(uint32_t reg)
{
	uint32_t apb;
	unsigned int intLvl;

	__asm__ __volatile__ (							\
		"rsil %[LVL], "XTSTR (CONFIG_ESP32_DPORT_DIS_INTERRUPT_LVL)"\n"	\
		"movi %[APB], "XTSTR (0x3ff40078) "\n"				\
		"l32i %[APB], %[APB], 0\n"					\
		"l32i %[REG], %[REG], 0\n"					\
		"wsr  %[LVL], "XTSTR (PS)"\n"					\
		"rsync\n"							\
		: [APB] "=a" (apb), [REG] "+a" (reg), [LVL] "=a" (intLvl)	\
		:								\
		: "memory"							\
		);
	return reg;
}
