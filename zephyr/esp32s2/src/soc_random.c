/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include "soc_random.h"
#include <esp_private/regi2c_ctrl.h>
#include <esp_private/sar_periph_ctrl.h>
#include <hal/adc_ll.h>
#include <hal/clk_tree_ll.h>
#include <hal/sar_ctrl_ll.h>
#include <soc/system_reg.h>

void soc_random_enable(void)
{
	/* Enable Wi-Fi clock for RNG module (direct register access, no kernel needed) */
	SET_PERI_REG_MASK(DPORT_WIFI_CLK_EN_REG, DPORT_WIFI_CLK_RNG_EN);

	/*
	 * Enable 8M clock source for RNG (this is actually enough to produce strong
	 * random results, but enabling the SAR ADC as well adds some insurance.)
	 */
	clk_ll_rc_fast_digi_enable();

	/* Enable SAR ADC to read a disconnected input for additional entropy */
	_adc_ll_reset_register();
	_adc_ll_enable_bus_clock(true);

	adc_ll_digi_clk_sel(ADC_DIGI_CLK_SRC_APB);
	adc_ll_calibration_prepare(ADC_UNIT_1, false);
	adc_ll_calibration_prepare(ADC_UNIT_2, false);
	adc_ll_calibration_init(ADC_UNIT_1);
	adc_ll_calibration_init(ADC_UNIT_2);

	regi2c_saradc_enable();

	/* Enable analog I2C master clock for RNG runtime */
	ANALOG_CLOCK_ENABLE();

	adc_ll_regi2c_init();
	sar_ctrl_ll_set_power_mode(SAR_CTRL_LL_POWER_ON);

	/* Use reserved channel 10 to get internal voltage */
	adc_digi_pattern_config_t pattern_config = {};

	pattern_config.unit = ADC_UNIT_1;
	pattern_config.atten = ADC_ATTEN_DB_12;
	pattern_config.channel = ADC_CHANNEL_10;
	adc_ll_digi_set_pattern_table(ADC_UNIT_1, 0, pattern_config);

	pattern_config.unit = ADC_UNIT_2;
	pattern_config.atten = ADC_ATTEN_DB_12;
	pattern_config.channel = ADC_CHANNEL_10;
	adc_ll_digi_set_pattern_table(ADC_UNIT_2, 0, pattern_config);

	adc_ll_digi_set_pattern_table_len(ADC_UNIT_1, 1);
	adc_ll_digi_set_pattern_table_len(ADC_UNIT_2, 1);

	adc_ll_digi_set_convert_mode(ADC_LL_DIGI_CONV_BOTH_UNIT);

	adc_ll_set_controller(ADC_UNIT_1, ADC_LL_CTRL_DIG);

	adc_ll_digi_set_clk_div(4);
	adc_ll_digi_set_trigger_interval(100);
	adc_ll_digi_trigger_enable();
}

void soc_random_disable(void)
{
	adc_ll_calibration_clear();
	_adc_ll_enable_bus_clock(false);
	adc_ll_digi_trigger_disable();
	adc_ll_digi_reset_pattern_table();

	adc_ll_regi2c_adc_deinit();
	regi2c_saradc_disable();

	ANALOG_CLOCK_DISABLE();

	/*
	 * Note: the 8M CLK entropy source continues running even after this function
	 * is called, but as mentioned above it's better to enable Wi-Fi or BT or call
	 * soc_random_enable() in order to get a secondary entropy source.
	 */
}
