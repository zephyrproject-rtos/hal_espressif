/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <assert.h>
#include "soc_init.h"
#include <soc/soc.h>
#include "soc/lp_analog_peri_reg.h"
#include "hal/clk_tree_ll.h"
#include "soc/system_reg.h"
#include "soc/assist_debug_reg.h"
#include "esp_private/regi2c_ctrl.h"
#include "soc/regi2c_lp_bias.h"
#include "soc/regi2c_bias.h"
#include "modem/modem_lpcon_reg.h"
#include "soc/pcr_reg.h"
#include "soc/lp_wdt_reg.h"
#include "esp_log.h"

const static char *TAG = "soc_init";

void soc_hw_init(void)
{
	/* Enable analog i2c master clock */
	SET_PERI_REG_MASK(MODEM_LPCON_CLK_CONF_REG, MODEM_LPCON_CLK_I2C_MST_EN);
	/* Fix low temp issue, need to increase this internal voltage */
	REGI2C_WRITE_MASK(I2C_BIAS, I2C_BIAS_DREG_0P8, 8);
}

void ana_super_wdt_reset_config(bool enable)
{
	/* H2 doesn't support bypass super WDT reset */
	assert(enable);
	REG_CLR_BIT(LP_ANALOG_PERI_LP_ANA_FIB_ENABLE_REG, LP_ANALOG_PERI_LP_ANA_FIB_SUPER_WDT_RST);
}

void ana_bod_reset_config(bool enable)
{
	REG_CLR_BIT(LP_ANALOG_PERI_LP_ANA_FIB_ENABLE_REG, LP_ANALOG_PERI_LP_ANA_FIB_BOD_RST);

	if (enable) {
		REG_SET_BIT(LP_ANALOG_PERI_LP_ANA_BOD_MODE1_CNTL_REG,
				LP_ANALOG_PERI_LP_ANA_BOD_MODE1_RESET_ENA);
	} else {
		REG_CLR_BIT(LP_ANALOG_PERI_LP_ANA_BOD_MODE1_CNTL_REG,
				LP_ANALOG_PERI_LP_ANA_BOD_MODE1_RESET_ENA);
	}
}

void ana_reset_config(void)
{
	ana_super_wdt_reset_config(true);
	ana_bod_reset_config(true);
}

void super_wdt_auto_feed(void)
{
	REG_WRITE(LP_WDT_SWD_WPROTECT_REG, LP_WDT_SWD_WKEY_VALUE);
	REG_SET_BIT(LP_WDT_SWD_CONFIG_REG, LP_WDT_SWD_AUTO_FEED_EN);
	REG_WRITE(LP_WDT_SWD_WPROTECT_REG, 0);
}

void wdt_reset_cpu0_info_enable(void)
{
	REG_SET_BIT(PCR_ASSIST_CONF_REG, PCR_ASSIST_CLK_EN);
	REG_CLR_BIT(PCR_ASSIST_CONF_REG, PCR_ASSIST_RST_EN);
	REG_WRITE(ASSIST_DEBUG_CORE_0_RCD_EN_REG,
		  ASSIST_DEBUG_CORE_0_RCD_PDEBUGEN | ASSIST_DEBUG_CORE_0_RCD_RECORDEN);
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
