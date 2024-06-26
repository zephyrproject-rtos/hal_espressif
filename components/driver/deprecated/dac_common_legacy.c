/*
 * SPDX-FileCopyrightText: 2019-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/kernel.h>

#include <string.h>
#include "esp_check.h"
#include "driver/rtc_io.h"
#include "driver/dac_types_legacy.h"
#include "soc/dac_periph.h"
#include "hal/gpio_types.h"
#include "hal/dac_ll.h"
#include "clk_ctrl_os.h"

extern int rtc_spinlock;

#define RTC_ENTER_CRITICAL()    do { rtc_spinlock = irq_lock(); } while(0)
#define RTC_EXIT_CRITICAL()    irq_unlock(rtc_spinlock);

static __attribute__((unused)) const char *TAG = "DAC";

/*---------------------------------------------------------------
                    DAC
---------------------------------------------------------------*/
esp_err_t dac_pad_get_io_num(dac_channel_t channel, gpio_num_t *gpio_num)
{
    ESP_RETURN_ON_FALSE(channel < SOC_DAC_CHAN_NUM, ESP_ERR_INVALID_ARG, TAG, "DAC channel error");

    *gpio_num = (gpio_num_t)dac_periph_signal.dac_channel_io_num[channel];

    return ESP_OK;
}

static esp_err_t dac_rtc_pad_init(dac_channel_t channel)
{
    ESP_RETURN_ON_FALSE(channel < SOC_DAC_CHAN_NUM, ESP_ERR_INVALID_ARG, TAG, "DAC channel error");

    gpio_num_t gpio_num = 0;
    dac_pad_get_io_num(channel, &gpio_num);
    rtc_gpio_init(gpio_num);
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_DISABLED);
    rtc_gpio_pullup_dis(gpio_num);
    rtc_gpio_pulldown_dis(gpio_num);

    return ESP_OK;
}

esp_err_t dac_output_enable(dac_channel_t channel)
{
    ESP_RETURN_ON_FALSE(channel < SOC_DAC_CHAN_NUM, ESP_ERR_INVALID_ARG, TAG, "DAC channel error");

    dac_rtc_pad_init(channel);
    RTC_ENTER_CRITICAL();
    dac_ll_power_on(channel);
    dac_ll_rtc_sync_by_adc(false);
    RTC_EXIT_CRITICAL();

    return ESP_OK;
}

esp_err_t dac_output_disable(dac_channel_t channel)
{
    ESP_RETURN_ON_FALSE(channel < SOC_DAC_CHAN_NUM, ESP_ERR_INVALID_ARG, TAG, "DAC channel error");

    RTC_ENTER_CRITICAL();
    dac_ll_power_down(channel);
    RTC_EXIT_CRITICAL();

    return ESP_OK;
}

esp_err_t dac_output_voltage(dac_channel_t channel, uint8_t dac_value)
{
    ESP_RETURN_ON_FALSE(channel < SOC_DAC_CHAN_NUM, ESP_ERR_INVALID_ARG, TAG, "DAC channel error");

    RTC_ENTER_CRITICAL();
    dac_ll_update_output_value(channel, dac_value);
    RTC_EXIT_CRITICAL();

    return ESP_OK;
}

esp_err_t dac_out_voltage(dac_channel_t channel, uint8_t dac_value)
{
    ESP_RETURN_ON_FALSE(channel < SOC_DAC_CHAN_NUM, ESP_ERR_INVALID_ARG, TAG, "DAC channel error");

    RTC_ENTER_CRITICAL();
    dac_ll_update_output_value(channel, dac_value);
    RTC_EXIT_CRITICAL();

    return ESP_OK;
}

esp_err_t dac_cw_generator_enable(void)
{
    RTC_ENTER_CRITICAL();
    periph_rtc_dig_clk8m_enable();
    dac_ll_cw_generator_enable();
    RTC_EXIT_CRITICAL();

    return ESP_OK;
}

esp_err_t dac_cw_generator_disable(void)
{
    RTC_ENTER_CRITICAL();
    dac_ll_cw_generator_disable();
    periph_rtc_dig_clk8m_disable();
    RTC_EXIT_CRITICAL();

    return ESP_OK;
}

esp_err_t dac_cw_generator_config(dac_cw_config_t *cw)
{
    ESP_RETURN_ON_FALSE(cw, ESP_ERR_INVALID_ARG, TAG, "invalid clock configuration");
    RTC_ENTER_CRITICAL();
    /* Enable the rtc8m clock temporary to get the correct frequency */
    periph_rtc_dig_clk8m_enable();
    uint32_t rtc_freq = periph_rtc_dig_clk8m_get_freq();
    periph_rtc_dig_clk8m_disable();
    dac_ll_cw_set_freq(cw->freq, rtc_freq);
    dac_ll_cw_set_atten(cw->en_ch, (dac_cosine_atten_t)cw->scale);
    dac_ll_cw_set_phase(cw->en_ch, (dac_cosine_phase_t)cw->phase);
    dac_ll_cw_set_dc_offset(cw->en_ch, cw->offset);
    dac_ll_cw_enable_channel(cw->en_ch, true);
    RTC_EXIT_CRITICAL();

    return ESP_OK;
}
