/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include "soc_init.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include "soc/system_reg.h"
#include "soc/assist_debug_reg.h"
#include "soc/reset_reasons.h"
#include "soc/dport_access.h"
#include "esp_log.h"

const static char *TAG = "soc_init";

void super_wdt_auto_feed(void)
{
	REG_SET_BIT(RTC_CNTL_SWD_CONF_REG, RTC_CNTL_SWD_AUTO_FEED_EN);
}

void wdt_reset_cpu0_info_enable(void)
{
	DPORT_REG_SET_BIT(DPORT_PERI_CLK_EN_REG, DPORT_PERI_EN_ASSIST_DEBUG);
	DPORT_REG_CLR_BIT(DPORT_PERI_RST_EN_REG, DPORT_PERI_EN_ASSIST_DEBUG);
	REG_WRITE(ASSIST_DEBUG_PRO_PDEBUGENABLE, 1);
	REG_WRITE(ASSIST_DEBUG_PRO_RCD_RECORDING, 1);
}

void check_wdt_reset(void)
{
	int wdt_rst = 0;
	soc_reset_reason_t rst_reas;

	rst_reas = esp_rom_get_reset_reason(0);
	if (rst_reas == RESET_REASON_CORE_RTC_WDT || rst_reas == RESET_REASON_CORE_MWDT0 ||
	    rst_reas == RESET_REASON_CORE_MWDT1 || rst_reas == RESET_REASON_CPU0_MWDT0 ||
	    rst_reas == RESET_REASON_CPU0_MWDT1 || rst_reas == RESET_REASON_CPU0_RTC_WDT) {
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
