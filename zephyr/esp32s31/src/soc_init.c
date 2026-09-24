/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <assert.h>
#include "soc_init.h"
#include <soc/soc.h>
#include <soc/rtc.h>
#include "soc/pmu_reg.h"
#include "soc/lp_analog_peri_reg.h"
#include "soc/regi2c_saradc.h"
#include "soc/regi2c_dig_reg.h"
#include "hal/clk_tree_ll.h"
#include "hal/brownout_ll.h"
#include "hal/regi2c_ctrl_ll.h"
#include "regi2c_ctrl.h"
#include "hal/pmu_ll.h"
#include "hal/mspi_ll.h"
#include "hal/assist_debug_ll.h"
#include "esp32s31/rom/spi_flash.h"
#include "soc/assist_debug_reg.h"
#include "soc/rtc_wdt_reg.h"
#include "hal/rwdt_ll.h"
#include "soc/regi2c_bias.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_rom_regi2c.h"

const static char *TAG = "soc_init";

void soc_hw_init(void)
{
	_regi2c_ctrl_ll_master_enable_clock(true);
	regi2c_ctrl_ll_master_configure_clock();

	REGI2C_WRITE_MASK(I2C_BIAS, I2C_BIAS_DREG_1P1, 10);
	REGI2C_WRITE_MASK(I2C_BIAS, I2C_BIAS_DREG_1P1_PVT, 10);
}

void ana_super_wdt_reset_config(bool enable)
{
	(void)enable;
}

void ana_bod_reset_config(bool enable)
{
	brownout_ll_ana_reset_enable(enable);
}

void ana_power_glitch_reset_config(bool enable)
{
	/* Only the VDDPST power glitch is detected */
	SET_PERI_REG_MASK(PMU_ANA_PERI_PWR_CTRL_REG, PMU_RSTB_PERIF_I2C);
	SET_PERI_REG_MASK(PMU_ANA_PERI_PWR_CTRL_REG, PMU_XPD_PERIF_I2C);
	REGI2C_WRITE_MASK(I2C_SARADC, POWER_GLITCH_XPD_VDET_PERIF, 0);
	REGI2C_WRITE_MASK(I2C_SARADC, POWER_GLITCH_XPD_VDET_PLLBB, 0);
	REGI2C_WRITE_MASK(I2C_SARADC, POWER_GLITCH_XPD_VDET_PLL, 0);

	REG_SET_FIELD(LP_ANA_FIB_ENABLE_REG, LP_ANA_ANA_FIB_PWR_GLITCH_ENA, 0);
	if (enable) {
		REG_SET_FIELD(LP_ANA_PG_GLITCH_CNTL_REG,
			      LP_ANA_POWER_GLITCH_RESET_ENA, 0xf);
	} else {
		REG_SET_FIELD(LP_ANA_PG_GLITCH_CNTL_REG,
			      LP_ANA_POWER_GLITCH_RESET_ENA, 0);
	}
}

void ana_reset_config(void)
{
	ana_super_wdt_reset_config(true);
	ana_bod_reset_config(true);
	ana_power_glitch_reset_config(true);
}

void super_wdt_auto_feed(void)
{
	REG_WRITE(RTC_WDT_SWD_WPROTECT_REG, RTC_WDT_SWD_WKEY_VALUE);
	REG_SET_BIT(RTC_WDT_SWD_CONFIG_REG, RTC_WDT_SWD_AUTO_FEED_EN);
	REG_WRITE(RTC_WDT_SWD_WPROTECT_REG, 0);
}

void wdt_reset_cpu0_info_enable(void)
{
	assist_debug_ll_enable_bus_clock(0, true);
	assist_debug_ll_enable_pc_recording(0, true);
}

void check_wdt_reset(void)
{
	int wdt_rst = 0;
	soc_reset_reason_t rst_reas;

	rst_reas = esp_rom_get_reset_reason(0);
	if (rst_reas == RESET_REASON_CPU_MWDT || rst_reas == RESET_REASON_CPU_RWDT ||
	    rst_reas == RESET_REASON_CORE_MWDT0 || rst_reas == RESET_REASON_CORE_MWDT1 ||
	    rst_reas == RESET_REASON_CORE_RWDT || rst_reas == RESET_REASON_SYS_RWDT) {
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

#if defined(CONFIG_ESP_SIMPLE_BOOT) || defined(CONFIG_MCUBOOT)
#include "esp_rom_serial_output.h"

/* Custom bootloader_clock_configure() for the ESP32-S31 bootloader stage
 * (simple boot or MCUboot).
 *
 * The ROM bootloader leaves the CPU on XTAL. Bring it up to
 * CONFIG_BOOTLOADER_CPU_CLK_FREQ_MHZ through the hal CPU frequency
 * configuration, which selects the CPLL (fixed 320 MHz, self calibrated
 * on power up) for the 80/160/320 MHz options and PLL_F240M for 240 MHz.
 * The application stage then only changes the dividers to reach the
 * devicetree CPU frequency, as the CPLL is already running.
 */
void bootloader_clock_configure(void)
{
	rtc_cpu_freq_config_t cpu_cfg;

	esp_rom_output_tx_wait_idle(0);

	/* Set the RC_SLOW tuning value and hand the RTC and DIG regulators
	 * over to the PMU, following the vendor clock init sequence.
	 */
	REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_SCK_DCAP, RTC_CNTL_SCK_DCAP_DEFAULT);
	REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_ENIF_RTC_DREG, 1);
	REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_ENIF_DIG_DREG, 1);
	REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_XPD_RTC_REG, 0);
	REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_XPD_DIG_REG, 0);

#if defined(CONFIG_ESP_SIMPLE_BOOT)
	/* Simple boot runs the bootloader stage inside the application
	 * image, where rtc_clk_cpu_freq_set_config() takes the application
	 * path and expects a separate bootloader to have powered up the
	 * CPLL already. Power it up here; esp_clk_tree_initialize() takes
	 * the reference once the CPU runs from it.
	 */
	clk_ll_cpll_enable();
#endif

	if (rtc_clk_cpu_freq_mhz_to_config(CONFIG_BOOTLOADER_CPU_CLK_FREQ_MHZ, &cpu_cfg)) {
		rtc_clk_cpu_freq_set_config(&cpu_cfg);
	}

	/* The ROM leaves the MSPI flash clock on XTAL. Set the 80 MHz core
	 * clock the image header divider works from and point the flash
	 * clock at the 480 MHz BBPLL for every flash mode, as the vendor
	 * bootloader does once the pll is up. Only the divider fix is
	 * specific to the 80 MHz option.
	 */
	_mspi_timing_ll_set_flash_core_clock(MSPI_TIMING_LL_MSPI_ID_0, 80);
	_mspi_timing_ll_set_flash_clk_src(MSPI_TIMING_LL_MSPI_ID_0, FLASH_CLK_SRC_BBPLL);
#if CONFIG_ESPTOOLPY_FLASHFREQ_80M
	/* In 80MHz flash mode, ROM sets the mspi module clk divider to 2 */
	esp_rom_spiflash_config_clk(1, 0);
	esp_rom_spiflash_config_clk(1, 1);
	esp_rom_spiflash_fix_dummylen(0, 1);
	esp_rom_spiflash_fix_dummylen(1, 1);
#endif

	/* Keep RC_FAST running so RNG has an entropy source during boot */
	rtc_clk_8m_enable(true);
	rtc_clk_fast_src_set(SOC_RTC_FAST_CLK_SRC_RC_FAST);

	/* Clear any pending LP/RTC interrupts */
	CLEAR_PERI_REG_MASK(RTC_WDT_INT_ENA_REG, RTC_WDT_SUPER_WDT_INT_ENA);
	CLEAR_PERI_REG_MASK(LP_ANA_LP_INT_ENA_REG, LP_ANA_BOD_MODE0_LP_INT_ENA);
	CLEAR_PERI_REG_MASK(RTC_WDT_INT_ENA_REG, RTC_WDT_LP_WDT_INT_ENA);
	CLEAR_PERI_REG_MASK(PMU_HP_INT_ENA_REG, PMU_SOC_WAKEUP_INT_ENA);
	CLEAR_PERI_REG_MASK(PMU_HP_INT_ENA_REG, PMU_SOC_SLEEP_REJECT_INT_ENA);

	SET_PERI_REG_MASK(RTC_WDT_INT_CLR_REG, RTC_WDT_SUPER_WDT_INT_CLR);
	SET_PERI_REG_MASK(LP_ANA_LP_INT_CLR_REG, LP_ANA_BOD_MODE0_LP_INT_CLR);
	SET_PERI_REG_MASK(RTC_WDT_INT_CLR_REG, RTC_WDT_LP_WDT_INT_CLR);
}
#endif /* CONFIG_ESP_SIMPLE_BOOT || CONFIG_MCUBOOT */

#include "esp_sleep.h"
__attribute__((weak)) esp_err_t esp_sleep_pd_config(esp_sleep_pd_domain_t domain,
						    esp_sleep_pd_option_t option)
{
	(void)domain;
	(void)option;
	return 0;
}
