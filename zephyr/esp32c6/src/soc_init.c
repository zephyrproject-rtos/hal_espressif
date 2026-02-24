/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <assert.h>
#include "soc_init.h"
#include <soc/soc.h>
#include "soc/lp_analog_peri_reg.h"
#include "hal/clk_tree_ll.h"
#include "hal/brownout_ll.h"
#include "hal/regi2c_ctrl_ll.h"
#include "hal/pmu_ll.h"
#include "hal/modem_syscon_ll.h"
#include "hal/modem_lpcon_ll.h"
#include "esp_private/esp_pmu.h"
#include "esp32c6/rom/spi_flash.h"
#include "modem/modem_lpcon_reg.h"
#include "soc/system_reg.h"
#include "soc/assist_debug_reg.h"
#include "soc/hp_apm_reg.h"
#include "soc/lp_apm_reg.h"
#include "soc/lp_apm0_reg.h"
#include "soc/pcr_reg.h"
#include "soc/lp_wdt_reg.h"
#include "hal/lpwdt_ll.h"
#include "esp_log.h"

const static char *TAG = "soc_init";

void soc_hw_init(void)
{
	/* In 80MHz flash mode, ROM sets the mspi module clk divider to 2, fix it here */
#if CONFIG_ESPTOOLPY_FLASHFREQ_80M
	clk_ll_mspi_fast_set_hs_divider(6);
	esp_rom_spiflash_config_clk(1, 0);
	esp_rom_spiflash_config_clk(1, 1);
	esp_rom_spiflash_fix_dummylen(0, 1);
	esp_rom_spiflash_fix_dummylen(1, 1);
#endif

	/* Enable analog I2C master clock */
	_regi2c_ctrl_ll_master_enable_clock(true);
	regi2c_ctrl_ll_master_configure_clock();

	/*
	 * Configure modem ICG code in PMU_ACTIVE state so the I2C master
	 * clock is not gated. Required before any REGI2C operations.
	 */
	pmu_ll_hp_set_icg_modem(&PMU, PMU_MODE_HP_ACTIVE, PMU_HP_ICG_MODEM_CODE_ACTIVE);
	modem_syscon_ll_set_modem_apb_icg_bitmap(&MODEM_SYSCON, BIT(PMU_HP_ICG_MODEM_CODE_ACTIVE));
	modem_lpcon_ll_set_i2c_master_icg_bitmap(&MODEM_LPCON, BIT(PMU_HP_ICG_MODEM_CODE_ACTIVE));
	modem_lpcon_ll_set_lp_apb_icg_bitmap(&MODEM_LPCON, BIT(PMU_HP_ICG_MODEM_CODE_ACTIVE));
	pmu_ll_imm_update_dig_icg_modem_code(&PMU, true);
	pmu_ll_imm_update_dig_icg_switch(&PMU, true);
}

void ana_super_wdt_reset_config(bool enable)
{
	/* C6 doesn't support bypass super WDT reset */
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
#include "soc/pmu_reg.h"
#include "esp_rom_serial_output.h"
#include "modem/modem_lpcon_struct.h"
#include "hal/clk_tree_ll.h"
#include "hal/regi2c_ctrl_ll.h"
#include "esp_private/regi2c_ctrl.h"
#include "esp_private/esp_pmu.h"

/*
 * Custom bootloader_clock_configure() for ESP32-C6 simple boot mode.
 *
 * In simple boot mode, we need to enable PLL and switch CPU clock source
 * so that flash can run at 80MHz (requires 480MHz PLL with divider 6).
 * The ROM bootloader leaves CPU running on XTAL (40MHz).
 */
void bootloader_clock_configure(void)
{
	/* Wait for ROM output to finish */
	esp_rom_output_tx_wait_idle(0);

	/* Reset I2C master */
	MODEM_LPCON.rst_conf.rst_i2c_mst = 1;
	MODEM_LPCON.rst_conf.rst_i2c_mst = 0;

	/* Set MSPI divider for 80MHz flash (PLL 480MHz / 6 = 80MHz) */
	clk_ll_mspi_fast_set_hs_divider(6);

	/* Enable BBPLL */
	clk_ll_bbpll_enable();

	/*
	 * Configure BBPLL for 480MHz with 40MHz XTAL.
	 * Use direct clock enable instead of ANALOG_CLOCK_ENABLE() macro
	 * which requires kernel services (irq_lock) not available during early boot.
	 */
	regi2c_ctrl_ll_master_force_enable_clock(true);
	regi2c_ctrl_ll_bbpll_calibration_start();
	clk_ll_bbpll_set_config(CLK_LL_PLL_480M_FREQ_MHZ, SOC_XTAL_FREQ_40M);
	while (!regi2c_ctrl_ll_bbpll_calibration_is_done()) {
		;
	}
	esp_rom_delay_us(10);
	regi2c_ctrl_ll_bbpll_calibration_stop();
	regi2c_ctrl_ll_master_force_enable_clock(false);

	/* Switch CPU to PLL at 160MHz (480MHz / 3) */
	clk_ll_cpu_set_hs_divider(CLK_LL_PLL_480M_FREQ_MHZ / 160);
	clk_ll_cpu_set_src(SOC_CPU_CLK_SRC_PLL);
	esp_rom_set_cpu_ticks_per_us(160);

	/* Store XTAL frequency for later use */
	clk_ll_xtal_store_freq_mhz(SOC_XTAL_FREQ_40M);

	/* Clear any pending LP/RTC interrupts */
	CLEAR_PERI_REG_MASK(LP_WDT_INT_ENA_REG, LP_WDT_SUPER_WDT_INT_ENA);
	CLEAR_PERI_REG_MASK(LP_ANALOG_PERI_LP_ANA_LP_INT_ENA_REG, LP_ANALOG_PERI_LP_ANA_BOD_MODE0_LP_INT_ENA);
	CLEAR_PERI_REG_MASK(LP_WDT_INT_ENA_REG, LP_WDT_LP_WDT_INT_ENA);
	CLEAR_PERI_REG_MASK(PMU_HP_INT_ENA_REG, PMU_SOC_WAKEUP_INT_ENA);
	CLEAR_PERI_REG_MASK(PMU_HP_INT_ENA_REG, PMU_SOC_SLEEP_REJECT_INT_ENA);

	SET_PERI_REG_MASK(LP_WDT_INT_CLR_REG, LP_WDT_SUPER_WDT_INT_CLR);
	SET_PERI_REG_MASK(LP_ANALOG_PERI_LP_ANA_LP_INT_CLR_REG, LP_ANALOG_PERI_LP_ANA_BOD_MODE0_LP_INT_CLR);
	SET_PERI_REG_MASK(LP_WDT_INT_CLR_REG, LP_WDT_LP_WDT_INT_CLR);
}
#endif /* CONFIG_ESP_SIMPLE_BOOT */
