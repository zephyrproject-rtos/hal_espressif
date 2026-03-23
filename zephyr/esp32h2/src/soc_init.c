/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <assert.h>
#include "soc_init.h"
#include "esp32h2/rom/rtc.h"
#include <soc/soc.h>
#include "soc/lp_analog_peri_reg.h"
#include "hal/clk_tree_ll.h"
#include "hal/brownout_ll.h"
#include "soc/system_reg.h"
#include "soc/assist_debug_reg.h"
#include "esp_private/regi2c_ctrl.h"
#include "soc/regi2c_lp_bias.h"
#include "soc/regi2c_bias.h"
#include "modem/modem_lpcon_reg.h"
#include "soc/pcr_reg.h"
#include "soc/lp_wdt_reg.h"
#include "soc/pmu_reg.h"
#include "hal/lpwdt_ll.h"
#include "hal/regi2c_ctrl_ll.h"
#include "esp_log.h"

const static char *TAG = "soc_init";

void soc_hw_init(void)
{
#if CONFIG_ESP_HAL_EARLY_LOG_LEVEL == 0
	rtc_suppress_rom_log();
#endif

	/* Disable RF pll by default */
	CLEAR_PERI_REG_MASK(PMU_RF_PWC_REG, PMU_XPD_RFPLL);
	SET_PERI_REG_MASK(PMU_RF_PWC_REG, PMU_XPD_FORCE_RFPLL);

	/* Enable analog i2c master clock */
	_regi2c_ctrl_ll_master_enable_clock(true);
	regi2c_ctrl_ll_master_configure_clock();

	/*
	 * Fix low temp issue, need to increase this internal voltage.
	 * Use ROM function directly instead of REGI2C_WRITE_MASK macro
	 * which requires kernel services (irq_lock) not available during early boot.
	 */
	esp_rom_regi2c_write_mask(I2C_BIAS, I2C_BIAS_HOSTID, I2C_BIAS_DREG_0P8,
				  I2C_BIAS_DREG_0P8_MSB, I2C_BIAS_DREG_0P8_LSB, 8);
}

void ana_super_wdt_reset_config(bool enable)
{
	/* H2 doesn't support bypass super WDT reset */
	assert(enable);
	REG_CLR_BIT(LP_ANALOG_PERI_LP_ANA_FIB_ENABLE_REG, LP_ANALOG_PERI_LP_ANA_FIB_SUPER_WDT_RST);
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

#if defined(CONFIG_ESP_SIMPLE_BOOT)
#include "esp_rom_serial_output.h"
#include "modem/modem_lpcon_struct.h"

void bootloader_clock_configure(void)
{
	esp_rom_output_tx_wait_idle(0);

	MODEM_LPCON.rst_conf.rst_i2c_mst = 1;
	MODEM_LPCON.rst_conf.rst_i2c_mst = 0;

	clk_ll_bbpll_enable();

	regi2c_ctrl_ll_master_force_enable_clock(true);
	regi2c_ctrl_ll_bbpll_calibration_start();

	clk_ll_bbpll_set_config(CLK_LL_PLL_96M_FREQ_MHZ, SOC_XTAL_FREQ_32M);

	while (!regi2c_ctrl_ll_bbpll_calibration_is_done()) {
		;
	}
	esp_rom_delay_us(10);

	regi2c_ctrl_ll_bbpll_calibration_stop();
	regi2c_ctrl_ll_master_force_enable_clock(false);

	clk_ll_cpu_set_divider(1);
	clk_ll_ahb_set_divider(3);

	clk_ll_cpu_set_src(SOC_CPU_CLK_SRC_PLL);
	clk_ll_bus_update();
	esp_rom_set_cpu_ticks_per_us(CLK_LL_PLL_96M_FREQ_MHZ);

	clk_ll_xtal_store_freq_mhz(SOC_XTAL_FREQ_32M);

	CLEAR_PERI_REG_MASK(LP_WDT_INT_ENA_REG, LP_WDT_SUPER_WDT_INT_ENA);
	CLEAR_PERI_REG_MASK(LP_ANALOG_PERI_LP_ANA_LP_INT_ENA_REG,
			    LP_ANALOG_PERI_LP_ANA_BOD_MODE0_LP_INT_ENA);
	CLEAR_PERI_REG_MASK(LP_WDT_INT_ENA_REG, LP_WDT_LP_WDT_INT_ENA);
	CLEAR_PERI_REG_MASK(PMU_HP_INT_ENA_REG, PMU_SOC_WAKEUP_INT_ENA);
	CLEAR_PERI_REG_MASK(PMU_HP_INT_ENA_REG, PMU_SOC_SLEEP_REJECT_INT_ENA);

	SET_PERI_REG_MASK(LP_WDT_INT_CLR_REG, LP_WDT_SUPER_WDT_INT_CLR);
	SET_PERI_REG_MASK(LP_ANALOG_PERI_LP_ANA_LP_INT_CLR_REG,
			  LP_ANALOG_PERI_LP_ANA_BOD_MODE0_LP_INT_CLR);
	SET_PERI_REG_MASK(LP_WDT_INT_CLR_REG, LP_WDT_LP_WDT_INT_CLR);
	SET_PERI_REG_MASK(PMU_HP_INT_CLR_REG, PMU_SOC_WAKEUP_INT_CLR);
	SET_PERI_REG_MASK(PMU_HP_INT_CLR_REG, PMU_SOC_SLEEP_REJECT_INT_CLR);
}
#endif /* CONFIG_ESP_SIMPLE_BOOT */
