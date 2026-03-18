
/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include "soc_init.h"
#include "esp32c2/rom/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/system_reg.h"
#include "soc/assist_debug_reg.h"
#include "soc/reset_reasons.h"
#include "hal/rwdt_ll.h"
#include "hal/brownout_ll.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "soc_init";

void soc_hw_init(void)
{
#if CONFIG_ESP_HAL_EARLY_LOG_LEVEL == 0
	rtc_suppress_rom_log();
#endif
}

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
	brownout_ll_ana_reset_enable(enable);
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

#if defined(CONFIG_ESP_SIMPLE_BOOT)
#include "soc/soc.h"
#include "soc/rtc.h"
#include "esp_rom_serial_output.h"
#include "hal/clk_tree_ll.h"
#include "hal/regi2c_ctrl_ll.h"

/*
 * Custom bootloader_clock_configure() for ESP32-C2 simple boot mode.
 *
 * In simple boot mode, we need to configure the PLL for 480MHz and switch
 * CPU to run from PLL. The ROM bootloader leaves CPU running on XTAL.
 * ESP32-C2 boards typically use 26MHz XTAL (some use 40MHz).
 */
void bootloader_clock_configure(void)
{
	esp_rom_output_tx_wait_idle(0);

	if (clk_ll_cpu_get_src() == SOC_CPU_CLK_SRC_PLL) {
		clk_ll_cpu_set_src(SOC_CPU_CLK_SRC_XTAL);
	}

	/* Reset I2C master for BBPLL configuration */
	regi2c_ctrl_ll_i2c_reset();
	regi2c_ctrl_ll_i2c_bbpll_enable();

	/* Get the XTAL frequency from config and update RTC */
	soc_xtal_freq_t xtal_freq = CONFIG_XTAL_FREQ;
	rtc_clk_xtal_freq_update(xtal_freq);
	rtc_clk_apb_freq_update(xtal_freq * MHZ(1));

	/* Enable and configure BBPLL for 480MHz */
	clk_ll_bbpll_enable();
	clk_ll_bbpll_set_freq_mhz(CLK_LL_PLL_480M_FREQ_MHZ);

	/* Start PLL calibration */
	regi2c_ctrl_ll_bbpll_calibration_start();
	clk_ll_bbpll_set_config(CLK_LL_PLL_480M_FREQ_MHZ, xtal_freq);

	/* Wait for calibration to complete */
	while (!regi2c_ctrl_ll_bbpll_calibration_is_done()) {
		;
	}
	esp_rom_delay_us(10);
	regi2c_ctrl_ll_bbpll_calibration_stop();

	/* Set CPU frequency to 120MHz (480MHz / 4) */
	clk_ll_cpu_set_freq_mhz_from_pll(CLK_LL_PLL_120M_FREQ_MHZ);
	clk_ll_cpu_set_src(SOC_CPU_CLK_SRC_PLL);
	esp_rom_set_cpu_ticks_per_us(CLK_LL_PLL_120M_FREQ_MHZ);

	/* Clear pending RTC/WDT interrupts */
	REG_WRITE(RTC_CNTL_INT_ENA_REG, 0);
	REG_WRITE(RTC_CNTL_INT_CLR_REG, UINT32_MAX);
}
#endif /* CONFIG_ESP_SIMPLE_BOOT */

