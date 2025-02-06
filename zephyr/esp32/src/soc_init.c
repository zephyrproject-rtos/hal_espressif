/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc_init.h"
#include "esp_log.h"
#include "rom/cache.h"
#include "soc/dport_reg.h"
#include <stdint.h>

static const char *TAG = "soc_init";

void wdt_reset_info_dump(int cpu)
{
	uint32_t inst = 0, pid = 0, stat = 0, data = 0, pc = 0, lsstat = 0, lsaddr = 0, lsdata = 0,
		 dstat = 0;
	const char *cpu_name = cpu ? "APP" : "PRO";

	stat = 0xdeadbeef;
	pid = 0;

	if (cpu == 0) {
		stat = DPORT_REG_READ(DPORT_PRO_CPU_RECORD_STATUS_REG);
		pid = DPORT_REG_READ(DPORT_PRO_CPU_RECORD_PID_REG);
		inst = DPORT_REG_READ(DPORT_PRO_CPU_RECORD_PDEBUGINST_REG);
		dstat = DPORT_REG_READ(DPORT_PRO_CPU_RECORD_PDEBUGSTATUS_REG);
		data = DPORT_REG_READ(DPORT_PRO_CPU_RECORD_PDEBUGDATA_REG);
		pc = DPORT_REG_READ(DPORT_PRO_CPU_RECORD_PDEBUGPC_REG);
		lsstat = DPORT_REG_READ(DPORT_PRO_CPU_RECORD_PDEBUGLS0STAT_REG);
		lsaddr = DPORT_REG_READ(DPORT_PRO_CPU_RECORD_PDEBUGLS0ADDR_REG);
		lsdata = DPORT_REG_READ(DPORT_PRO_CPU_RECORD_PDEBUGLS0DATA_REG);
	} else {
		ESP_EARLY_LOGE(TAG, "WDT reset info: %s CPU not support!\n", cpu_name);
		return;
	}

	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU STATUS        0x%08" PRIx32, cpu_name, stat);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PID           0x%08" PRIx32, cpu_name, pid);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGINST    0x%08" PRIx32, cpu_name, inst);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGSTATUS  0x%08" PRIx32, cpu_name, dstat);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGDATA    0x%08" PRIx32, cpu_name, data);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGPC      0x%08" PRIx32, cpu_name, pc);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGLS0STAT 0x%08" PRIx32, cpu_name, lsstat);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGLS0ADDR 0x%08" PRIx32, cpu_name, lsaddr);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGLS0DATA 0x%08" PRIx32, cpu_name, lsdata);
}

void wdt_reset_cpu0_info_enable(void)
{
	DPORT_REG_SET_BIT(DPORT_PRO_CPU_RECORD_CTRL_REG,
			  DPORT_PRO_CPU_PDEBUG_ENABLE | DPORT_PRO_CPU_RECORD_ENABLE);
	DPORT_REG_CLR_BIT(DPORT_PRO_CPU_RECORD_CTRL_REG, DPORT_PRO_CPU_RECORD_ENABLE);
}

void check_wdt_reset(void)
{
	int wdt_rst = 0;
	soc_reset_reason_t rst_reas;

	rst_reas = esp_rom_get_reset_reason(0);
	if (rst_reas == RESET_REASON_CORE_RTC_WDT || rst_reas == RESET_REASON_CORE_MWDT0 ||
	    rst_reas == RESET_REASON_CORE_MWDT1 || rst_reas == RESET_REASON_CPU0_RTC_WDT) {
		ESP_EARLY_LOGW(TAG, "PRO CPU has been reset by WDT.");
		wdt_rst = 1;
	}

	if (wdt_rst) {
		wdt_reset_info_dump(0);
	}

	wdt_reset_cpu0_info_enable();
}

void reset_mmu(void)
{
	/* completely reset MMU in case serial bootloader was running */
	Cache_Read_Disable(0);
	Cache_Flush(0);
	mmu_init(0);

	/* normal ROM boot exits with DROM0 cache unmasked,
	 * but serial bootloader exits with it masked.
	 */
	DPORT_REG_CLR_BIT(DPORT_PRO_CACHE_CTRL1_REG, DPORT_PRO_CACHE_MASK_DROM0);
}

/* Not supported but common bootloader calls the function. Do nothing */
void ana_clock_glitch_reset_config(bool enable)
{
	(void)enable;
}
