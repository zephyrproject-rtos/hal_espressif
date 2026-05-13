/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <assert.h>
#include "soc_init.h"
#include <soc/soc.h>
#include "hal/clk_tree_ll.h"
#include "hal/brownout_ll.h"
#include "hal/regi2c_ctrl_ll.h"
#include "regi2c_ctrl.h"
#include "hal/pmu_ll.h"
#include "hal/mspi_ll.h"
#include "hal/assist_debug_ll.h"
#include "hal/lpwdt_ll.h"
#include "esp32p4/rom/spi_flash.h"
#include "soc/assist_debug_reg.h"
#include "soc/lp_wdt_reg.h"
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
	_assist_debug_ll_enable_bus_clock(0, true);
	assist_debug_ll_enable_pc_recording(0, true);
}

void check_wdt_reset(void)
{
	int wdt_rst = 0;
	soc_reset_reason_t rst_reas;

	rst_reas = esp_rom_get_reset_reason(0);
	if (rst_reas == RESET_REASON_CPU_MWDT || rst_reas == RESET_REASON_CPU_RWDT ||
	    rst_reas == RESET_REASON_CORE_MWDT || rst_reas == RESET_REASON_CORE_RWDT ||
	    rst_reas == RESET_REASON_SYS_RWDT) {
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
#include "soc/pmu_reg.h"
#include "soc/lp_system_reg.h"
#include "soc/chip_revision.h"
#include "esp_rom_serial_output.h"
#include "esp_rom_regi2c.h"
#include "soc/regi2c_cpll.h"
#include "hal/mspi_ll.h"
#include "hal/efuse_hal.h"
#include "pmu_param.h"

/*
 * Custom bootloader_clock_configure() for ESP32-P4 simple boot mode.
 *
 * In simple boot mode, we enable PLL and switch CPU clock source so that
 * flash can run at higher speeds. The ROM bootloader leaves CPU on XTAL.
 */
void bootloader_clock_configure(void)
{
	esp_rom_output_tx_wait_idle(0);

	clk_ll_xtal_store_freq_mhz(SOC_XTAL_FREQ_40M);

	clk_ll_cpll_enable();

	clk_ll_cpll_calibration_start();
	while (!clk_ll_cpll_calibration_is_done()) {
		;
	}
	esp_rom_delay_us(10);
	clk_ll_cpll_calibration_stop();

	/*
	 * Voltage regulator + DCDC handoff, mirroring ESP-IDF rtc_clk_init().
	 * The ROM leaves the chip on LDO with default dbias. This sequence
	 * calibrates dbias, enables the DCDC switch, hands DIG dbias control
	 * to PMU, and finally turns the LDO off so DCDC drives HP. Required
	 * before deep sleep, because pmu_sleep_shutdown_ldo() expects DCDC
	 * to be the active regulator on wake.
	 */
	uint32_t hp_cali_dbias = get_act_hp_dbias();
	uint32_t lp_cali_dbias = get_act_lp_dbias();
	unsigned chip_version = efuse_hal_chip_revision();

	pmu_ll_hp_set_regulator_xpd(&PMU, PMU_MODE_HP_ACTIVE, true);
	pmu_ll_hp_set_regulator_dbias(&PMU, PMU_MODE_HP_ACTIVE, hp_cali_dbias);
	pmu_ll_lp_set_regulator_dbias(&PMU, PMU_MODE_LP_ACTIVE, lp_cali_dbias);

	uint32_t pvt_hp_dcmvset =
		GET_PERI_REG_BITS2(PMU_HP_ACTIVE_HP_REGULATOR0_REG,
				   PMU_HP_DBIAS_VOL_V, PMU_HP_DBIAS_VOL_S);
	uint32_t hp_dcmvset = HP_CALI_ACTIVE_DCM_VSET_DEFAULT;

	if (pvt_hp_dcmvset > hp_dcmvset) {
		hp_dcmvset = pvt_hp_dcmvset;
	}

	if (ESP_CHIP_REV_ABOVE(chip_version, 301)) {
		SET_PERI_REG_MASK(PMU_DCM_CTRL_REG, PMU_DCDC_FB_RES_FORCE_PD);
	}
	pmu_ll_set_dcdc_en(&PMU, true);
	pmu_ll_set_dcdc_switch_force_power_down(&PMU, false);
	pmu_ll_hp_set_dcm_vset(&PMU, PMU_MODE_HP_ACTIVE, hp_dcmvset);
	SET_PERI_REG_MASK(PMU_HP_ACTIVE_HP_REGULATOR0_REG,
			  PMU_DIG_REGULATOR0_DBIAS_SEL);
	esp_rom_delay_us(1000);
	if (ESP_CHIP_REV_ABOVE(chip_version, 301)) {
		REG_SET_FIELD(LP_SYSTEM_REG_SYS_CTRL_REG,
			      LP_SYSTEM_REG_LP_FIB_SEL, 0xEF);
		CLEAR_PERI_REG_MASK(PMU_DCM_CTRL_REG, PMU_DCDC_FB_RES_FORCE_PD);
		esp_rom_delay_us(10);
	}
	pmu_ll_hp_set_regulator_xpd(&PMU, PMU_MODE_HP_ACTIVE, false);

	/* Upscale CPU to CONFIG_BOOTLOADER_CPU_CLK_FREQ_MHZ (100 MHz on
	 * ECO >= 3.0), matching ESP-IDF rtc_clk_cpu_freq_to_cpll_mhz():
	 *   CPLL = 400 MHz, cpu_div = 4 -> CPU = 100 MHz
	 *   mem_div = 1, sys_div = 1, apb_div = 1
	 * Upscaling order: APB -> SYS -> MEM -> CPU -> src switch.
	 */
	clk_ll_apb_set_divider(1);
	clk_ll_bus_update();
	clk_ll_sys_set_divider(1);
	clk_ll_bus_update();
	clk_ll_mem_set_divider(1);
	clk_ll_bus_update();
	clk_ll_cpu_set_divider(4, 0, 0);
	clk_ll_bus_update();
	clk_ll_cpu_set_src(SOC_CPU_CLK_SRC_CPLL);
	esp_rom_set_cpu_ticks_per_us(CONFIG_BOOTLOADER_CPU_CLK_FREQ_MHZ);

	/* MSPI flash: SPLL (480MHz) with core clock at 80MHz */
	_mspi_timing_ll_set_flash_clk_src(0, FLASH_CLK_SRC_SPLL);
	_mspi_timing_ll_set_flash_core_clock(0, 80);

	CLEAR_PERI_REG_MASK(LP_WDT_INT_ENA_REG, LP_WDT_SUPER_WDT_INT_ENA);
	CLEAR_PERI_REG_MASK(LP_WDT_INT_ENA_REG, LP_WDT_LP_WDT_INT_ENA);
	CLEAR_PERI_REG_MASK(PMU_HP_INT_ENA_REG, PMU_SOC_WAKEUP_INT_ENA);
	CLEAR_PERI_REG_MASK(PMU_HP_INT_ENA_REG, PMU_SOC_SLEEP_REJECT_INT_ENA);

	SET_PERI_REG_MASK(LP_WDT_INT_CLR_REG, LP_WDT_SUPER_WDT_INT_CLR);
	SET_PERI_REG_MASK(LP_WDT_INT_CLR_REG, LP_WDT_LP_WDT_INT_CLR);
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
