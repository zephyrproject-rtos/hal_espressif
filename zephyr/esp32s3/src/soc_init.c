/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <inttypes.h>
#include "soc_init.h"
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include "soc/system_reg.h"
#include "soc/assist_debug_reg.h"
#include "soc/reset_reasons.h"
#include "esp_log.h"

const static char *TAG = "soc_init";

void ana_super_wdt_reset_config(bool enable)
{
	REG_CLR_BIT(RTC_CNTL_FIB_SEL_REG, RTC_CNTL_FIB_SUPER_WDT_RST);

	if (enable) {
		REG_CLR_BIT(RTC_CNTL_SWD_CONF_REG, RTC_CNTL_SWD_BYPASS_RST);
	} else {
		REG_SET_BIT(RTC_CNTL_SWD_CONF_REG, RTC_CNTL_SWD_BYPASS_RST);
	}
}

void ana_bod_reset_config(bool enable)
{
	REG_CLR_BIT(RTC_CNTL_FIB_SEL_REG, RTC_CNTL_FIB_BOD_RST);

	if (enable) {
		REG_SET_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ANA_RST_EN);
	} else {
		REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ANA_RST_EN);
	}
}

void ana_clock_glitch_reset_config(bool enable)
{
	REG_CLR_BIT(RTC_CNTL_FIB_SEL_REG, RTC_CNTL_FIB_GLITCH_RST);

	if (enable) {
		REG_SET_BIT(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_GLITCH_RST_EN);
	} else {
		REG_CLR_BIT(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_GLITCH_RST_EN);
	}
}

void ana_reset_config(void)
{
	ana_super_wdt_reset_config(true);
	ana_bod_reset_config(true);
	ana_clock_glitch_reset_config(true);
}

void super_wdt_auto_feed(void)
{
	REG_WRITE(RTC_CNTL_SWD_WPROTECT_REG, RTC_CNTL_SWD_WKEY_VALUE);
	REG_SET_BIT(RTC_CNTL_SWD_CONF_REG, RTC_CNTL_SWD_AUTO_FEED_EN);
	REG_WRITE(RTC_CNTL_SWD_WPROTECT_REG, 0);
}

void wdt_reset_info_dump(int cpu)
{
	uint32_t inst = 0, pid = 0, stat = 0, data = 0, pc = 0, lsstat = 0, lsaddr = 0, lsdata = 0,
		 dstat = 0;
	const char *cpu_name = cpu ? "APP" : "PRO";

	stat = 0xdeadbeef;
	pid = 0;

	if (cpu == 0) {
		inst = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGINST_REG);
		dstat = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGSTATUS_REG);
		data = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGDATA_REG);
		pc = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC_REG);
		lsstat = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGLS0STAT_REG);
		lsaddr = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGLS0ADDR_REG);
		lsdata = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGLS0DATA_REG);
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
	REG_SET_BIT(SYSTEM_CPU_PERI_CLK_EN_REG, SYSTEM_CLK_EN_ASSIST_DEBUG);
	REG_CLR_BIT(SYSTEM_CPU_PERI_RST_EN_REG, SYSTEM_RST_EN_ASSIST_DEBUG);
	REG_WRITE(ASSIST_DEBUG_CORE_0_RCD_PDEBUGENABLE_REG, 1);
	REG_WRITE(ASSIST_DEBUG_CORE_0_RCD_RECORDING_REG, 1);
}

void check_wdt_reset(void)
{
	int wdt_rst = 0;
	soc_reset_reason_t rst_reas;

	rst_reas = esp_rom_get_reset_reason(0);
	if (rst_reas == RESET_REASON_CORE_RTC_WDT || rst_reas == RESET_REASON_CORE_MWDT0 ||
	    rst_reas == RESET_REASON_CORE_MWDT1 || rst_reas == RESET_REASON_CPU0_MWDT0 ||
	    rst_reas == RESET_REASON_CPU0_RTC_WDT) {
		ESP_EARLY_LOGW(TAG, "PRO CPU has been reset by WDT.");
		wdt_rst = 1;
	}

	if (wdt_rst) {
		wdt_reset_info_dump(0);
	}

	wdt_reset_cpu0_info_enable();
}
