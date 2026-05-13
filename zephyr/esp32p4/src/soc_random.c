/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include "soc_random.h"
#include <esp_private/regi2c_ctrl.h>
#include <hal/adc_ll.h>
#include <hal/adc_types.h>
#include <hal/regi2c_ctrl_ll.h>

#define I2C_SAR_ADC_INIT_CODE_VAL       2166
#define ADC_RNG_CLKM_DIV_NUM            0
#define ADC_RNG_CLKM_DIV_B              0
#define ADC_RNG_CLKM_DIV_A              0

void soc_random_enable(void)
{
	_adc_ll_reset_register();
	_adc_ll_enable_bus_clock(true);

	adc_ll_digi_clk_sel(ADC_DIGI_CLK_SRC_XTAL);
	adc_ll_digi_controller_clk_div(ADC_RNG_CLKM_DIV_NUM, ADC_RNG_CLKM_DIV_B, ADC_RNG_CLKM_DIV_A);

	/* Some ADC sensor registers are in power group PERIF_I2C and need to be enabled via PMU */
	regi2c_ctrl_ll_i2c_sar_periph_enable();

	/* Enable analog I2C master clock for RNG runtime */
	ANALOG_CLOCK_ENABLE();

	adc_ll_regi2c_init();
	adc_ll_set_calibration_param(ADC_UNIT_1, I2C_SAR_ADC_INIT_CODE_VAL);

	adc_digi_pattern_config_t pattern_config = {};

	pattern_config.unit = ADC_UNIT_1;
	pattern_config.atten = ADC_ATTEN_DB_12;
	pattern_config.channel = ADC_CHANNEL_10;
	adc_ll_digi_set_pattern_table(ADC_UNIT_1, 0, pattern_config);
	adc_ll_digi_set_pattern_table(ADC_UNIT_1, 1, pattern_config);
	adc_ll_digi_set_pattern_table(ADC_UNIT_1, 2, pattern_config);
	adc_ll_digi_set_pattern_table(ADC_UNIT_1, 3, pattern_config);
	adc_ll_digi_set_pattern_table_len(ADC_UNIT_1, 1);

	adc_ll_set_controller(ADC_UNIT_1, ADC_LL_CTRL_DIG);
	adc_ll_digi_set_power_manage(ADC_UNIT_1, ADC_LL_POWER_SW_ON);

	adc_ll_digi_set_clk_div(15);
	adc_ll_digi_set_trigger_interval(100);
	adc_ll_digi_trigger_enable();
}

void soc_random_disable(void)
{
	adc_ll_digi_trigger_disable();
	adc_ll_digi_reset_pattern_table();
	adc_ll_set_calibration_param(ADC_UNIT_1, 0x0);
	adc_ll_set_calibration_param(ADC_UNIT_2, 0x0);

	adc_ll_regi2c_adc_deinit();

	/* Disable analog I2C master clock */
	ANALOG_CLOCK_DISABLE();
	adc_ll_digi_controller_clk_div(4, 0, 0);
	adc_ll_digi_clk_sel(ADC_DIGI_CLK_SRC_XTAL);

	adc_ll_set_controller(ADC_UNIT_1, ADC_LL_CTRL_ULP);
}
