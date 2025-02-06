
/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include "soc_init.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/system_reg.h"
#include "soc/assist_debug_reg.h"
#include "soc/reset_reasons.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "soc_init";

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

void ana_reset_config(void)
{
	ana_super_wdt_reset_config(true);
	ana_bod_reset_config(true);
}

void super_wdt_auto_feed(void)
{
	REG_WRITE(RTC_CNTL_SWD_WPROTECT_REG, RTC_CNTL_SWD_WKEY_VALUE);
	REG_SET_BIT(RTC_CNTL_SWD_CONF_REG, RTC_CNTL_SWD_AUTO_FEED_EN);
	REG_WRITE(RTC_CNTL_SWD_WPROTECT_REG, 0);
}

void wdt_reset_cpu0_info_enable(void)
{
	REG_SET_BIT(SYSTEM_CPU_PERI_CLK_EN_REG, SYSTEM_CLK_EN_ASSIST_DEBUG);
	REG_CLR_BIT(SYSTEM_CPU_PERI_RST_EN_REG, SYSTEM_RST_EN_ASSIST_DEBUG);
	REG_WRITE(ASSIST_DEBUG_CORE_0_RCD_EN_REG,
		  ASSIST_DEBUG_CORE_0_RCD_PDEBUGEN | ASSIST_DEBUG_CORE_0_RCD_RECORDEN);
}

void check_wdt_reset(void)
{
	int wdt_rst = 0;
	soc_reset_reason_t rst_reas;

	rst_reas = esp_rom_get_reset_reason(0);
	if (rst_reas == RESET_REASON_CORE_RTC_WDT || rst_reas == RESET_REASON_CORE_MWDT0 ||
	    rst_reas == RESET_REASON_CPU0_MWDT0 || rst_reas == RESET_REASON_CPU0_RTC_WDT) {
		ESP_EARLY_LOGW(TAG, "PRO CPU has been reset by WDT.");
		wdt_rst = 1;
	}

	wdt_reset_cpu0_info_enable();
}

/* Not supported but common bootloader calls the function. Do nothing */
void ana_clock_glitch_reset_config(bool enable)
{
	(void)enable;
}
