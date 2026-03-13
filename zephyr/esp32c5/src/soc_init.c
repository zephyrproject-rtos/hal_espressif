/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
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
#include "hal/mspi_ll.h"
#include "esp_private/esp_pmu.h"
#include "esp32c5/rom/spi_flash.h"
#include "modem/modem_lpcon_reg.h"
#include "soc/system_reg.h"
#include "soc/assist_debug_reg.h"
#include "soc/hp_apm_reg.h"
#include "soc/lp_apm_reg.h"
#include "soc/lp_apm0_reg.h"
#include "soc/pcr_reg.h"
#include "soc/lp_wdt_reg.h"
#include "hal/lpwdt_ll.h"
#include "hal/axi_icm_ll.h"
#include "esp_log.h"

const static char *TAG = "soc_init";

void soc_hw_init(void)
{
	/*
	 * Ensure the system bus resets properly during a core reset (WDT).
	 * Prevents bus freezing caused by an incorrect MSPI core reset in ROM.
	 */
	axi_icm_ll_reset_with_core_reset(true);

	/* In 80MHz flash mode, ROM sets the mspi module clk divider to 2, fix it here */
#if CONFIG_ESPTOOLPY_FLASHFREQ_80M
	mspi_timing_ll_set_core_clock(0, 80);
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
	(void)enable;
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

	(void)wdt_rst;
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
#include "esp_rom_regi2c.h"
#include "soc/regi2c_bbpll.h"

/*
 * Custom bootloader_clock_configure() for ESP32-C5 simple boot mode.
 *
 * In simple boot mode, we enable PLL and switch CPU clock source so that
 * flash can run at higher speeds. The ROM bootloader leaves CPU on XTAL.
 *
 * Cannot use rtc_clk_init() or REGI2C_WRITE/REGI2C_WRITE_MASK here
 * because without BOOTLOADER_BUILD they go through
 * ANALOG_CLOCK_ENABLE() -> PERIPH_RCC_ACQUIRE_ATOMIC -> irq_lock
 * which requires kernel infrastructure not available during early boot.
 *
 * Instead, use esp_rom_regi2c_write/write_mask directly for BBPLL
 * register configuration, and LL functions for calibration control
 * and clock source switching.
 */
void bootloader_clock_configure(void)
{
	uint32_t xtal_freq;
	uint8_t div_ref, div7_0, dr1, dr3;
	uint8_t dchgp = 5, dbias = 3, href = 3, lref = 1;

	esp_rom_output_tx_wait_idle(0);

	xtal_freq = clk_ll_xtal_get_freq_mhz();

	/* Reset I2C master in case it was left in a bad state (e.g. after WDT reset) */
	_regi2c_ctrl_ll_master_reset();

	/* Force-enable I2C master clock for REGI2C operations */
	regi2c_ctrl_ll_master_force_enable_clock(true);

	/* Enable BBPLL power */
	clk_ll_bbpll_enable();

	/* Start BBPLL self-calibration */
	regi2c_ctrl_ll_bbpll_calibration_start();

	/*
	 * Configure BBPLL for 480MHz using direct ROM regi2c calls.
	 * This bypasses REGI2C_WRITE/REGI2C_WRITE_MASK macros which
	 * would call ANALOG_CLOCK_ENABLE() -> irq_lock during early boot.
	 */
	switch (xtal_freq) {
	case SOC_XTAL_FREQ_48M:
		div_ref = 1;
		div7_0 = 10;
		dr1 = 1;
		dr3 = 1;
		break;
	case SOC_XTAL_FREQ_40M:
	default:
		div_ref = 1;
		div7_0 = 12;
		dr1 = 0;
		dr3 = 0;
		break;
	}

	uint8_t i2c_bbpll_lref = (dchgp << I2C_BBPLL_OC_DCHGP_LSB) | div_ref;
	esp_rom_regi2c_write(I2C_BBPLL, I2C_BBPLL_HOSTID,
			     I2C_BBPLL_OC_REF_DIV, i2c_bbpll_lref);
	esp_rom_regi2c_write(I2C_BBPLL, I2C_BBPLL_HOSTID,
			     I2C_BBPLL_OC_DIV_7_0, div7_0);
	esp_rom_regi2c_write_mask(I2C_BBPLL, I2C_BBPLL_HOSTID,
				  I2C_BBPLL_OC_DR1,
				  I2C_BBPLL_OC_DR1_MSB,
				  I2C_BBPLL_OC_DR1_LSB, dr1);
	esp_rom_regi2c_write_mask(I2C_BBPLL, I2C_BBPLL_HOSTID,
				  I2C_BBPLL_OC_DR3,
				  I2C_BBPLL_OC_DR3_MSB,
				  I2C_BBPLL_OC_DR3_LSB, dr3);
	esp_rom_regi2c_write_mask(I2C_BBPLL, I2C_BBPLL_HOSTID,
				  I2C_BBPLL_OC_DLREF_SEL,
				  I2C_BBPLL_OC_DLREF_SEL_MSB,
				  I2C_BBPLL_OC_DLREF_SEL_LSB, lref);
	esp_rom_regi2c_write_mask(I2C_BBPLL, I2C_BBPLL_HOSTID,
				  I2C_BBPLL_OC_DHREF_SEL,
				  I2C_BBPLL_OC_DHREF_SEL_MSB,
				  I2C_BBPLL_OC_DHREF_SEL_LSB, href);
	esp_rom_regi2c_write_mask(I2C_BBPLL, I2C_BBPLL_HOSTID,
				  I2C_BBPLL_OC_VCO_DBIAS,
				  I2C_BBPLL_OC_VCO_DBIAS_MSB,
				  I2C_BBPLL_OC_VCO_DBIAS_LSB, dbias);

	/* Wait for calibration to complete */
	while (!regi2c_ctrl_ll_bbpll_calibration_is_done()) {
		;
	}
	esp_rom_delay_us(10);
	regi2c_ctrl_ll_bbpll_calibration_stop();
	regi2c_ctrl_ll_master_force_enable_clock(false);

	/* Switch CPU to PLL_F160M at 160MHz */
	clk_ll_cpu_set_divider(1);
	clk_ll_ahb_set_divider(4);
	clk_ll_cpu_set_src(SOC_CPU_CLK_SRC_PLL_F160M);
	clk_ll_bus_update();
	esp_rom_set_cpu_ticks_per_us(160);

	/* Clear any pending LP/RTC interrupts */
	CLEAR_PERI_REG_MASK(LP_WDT_INT_ENA_REG, LP_WDT_SUPER_WDT_INT_ENA);
	CLEAR_PERI_REG_MASK(LP_ANA_LP_INT_ENA_REG,
			    LP_ANA_BOD_MODE0_LP_INT_ENA);
	CLEAR_PERI_REG_MASK(LP_WDT_INT_ENA_REG, LP_WDT_LP_WDT_INT_ENA);
	CLEAR_PERI_REG_MASK(PMU_HP_INT_ENA_REG, PMU_SOC_WAKEUP_INT_ENA);
	CLEAR_PERI_REG_MASK(PMU_HP_INT_ENA_REG, PMU_SOC_SLEEP_REJECT_INT_ENA);

	SET_PERI_REG_MASK(LP_WDT_INT_CLR_REG, LP_WDT_SUPER_WDT_INT_CLR);
	SET_PERI_REG_MASK(LP_ANA_LP_INT_CLR_REG,
			  LP_ANA_BOD_MODE0_LP_INT_CLR);
	SET_PERI_REG_MASK(LP_WDT_INT_CLR_REG, LP_WDT_LP_WDT_INT_CLR);
	SET_PERI_REG_MASK(PMU_HP_INT_CLR_REG, PMU_SOC_WAKEUP_INT_CLR);
	SET_PERI_REG_MASK(PMU_HP_INT_CLR_REG, PMU_SOC_SLEEP_REJECT_INT_CLR);
}
#endif /* CONFIG_ESP_SIMPLE_BOOT */
