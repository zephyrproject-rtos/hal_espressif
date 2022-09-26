/*
 * SPDX-FileCopyrightText: 2019-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <esp_types.h>
#include "esp_err.h"
#include "esp_check.h"
#include "driver/rtc_io.h"
#include "driver/dac_common.h"
#include "soc/dac_periph.h"
#include "hal/dac_hal.h"

struct dac_context {
    unsigned int spinlock;
};

static struct dac_context dac_context = {
    .spinlock = 0
};

static const char *TAG = "DAC";

/*---------------------------------------------------------------
                    DAC
---------------------------------------------------------------*/
esp_err_t dac_pad_get_io_num(dac_channel_t channel, gpio_num_t *gpio_num)
{
    ESP_RETURN_ON_FALSE(channel < DAC_CHANNEL_MAX,
            ESP_ERR_INVALID_ARG, TAG, "DAC channel error");

    *gpio_num = (gpio_num_t) dac_periph_signal.dac_channel_io_num[channel];

    return ESP_OK;
}

static esp_err_t dac_rtc_pad_init(dac_channel_t channel)
{
    ESP_RETURN_ON_FALSE(channel < DAC_CHANNEL_MAX,
            ESP_ERR_INVALID_ARG, TAG, "DAC channel error");

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
    ESP_RETURN_ON_FALSE(channel < DAC_CHANNEL_MAX,
            ESP_ERR_INVALID_ARG, TAG, "DAC channel error");

    dac_rtc_pad_init(channel);
    dac_context.spinlock = irq_lock();
    dac_hal_power_on(channel);
    dac_hal_rtc_sync_by_adc(false);
    irq_unlock(dac_context.spinlock);

    return ESP_OK;
}

esp_err_t dac_output_disable(dac_channel_t channel)
{
    ESP_RETURN_ON_FALSE(channel < DAC_CHANNEL_MAX,
            ESP_ERR_INVALID_ARG, TAG, "DAC channel error");

    dac_context.spinlock = irq_lock();
    dac_hal_power_down(channel);
    irq_unlock(dac_context.spinlock);

    return ESP_OK;
}

esp_err_t dac_output_voltage(dac_channel_t channel, uint8_t dac_value)
{
    ESP_RETURN_ON_FALSE(channel < DAC_CHANNEL_MAX,
            ESP_ERR_INVALID_ARG, TAG, "DAC channel error");

    dac_context.spinlock = irq_lock();
    dac_hal_update_output_value(channel, dac_value);
    irq_unlock(dac_context.spinlock);

    return ESP_OK;
}

esp_err_t dac_out_voltage(dac_channel_t channel, uint8_t dac_value)
{
    ESP_RETURN_ON_FALSE(channel < DAC_CHANNEL_MAX,
            ESP_ERR_INVALID_ARG, TAG, "DAC channel error");

    dac_context.spinlock = irq_lock();
    dac_hal_update_output_value(channel, dac_value);
    irq_unlock(dac_context.spinlock);

    return ESP_OK;
}

esp_err_t dac_cw_generator_enable(void)
{
    dac_context.spinlock = irq_lock();
    dac_hal_cw_generator_enable();
    irq_unlock(dac_context.spinlock);

    return ESP_OK;
}

esp_err_t dac_cw_generator_disable(void)
{
    dac_context.spinlock = irq_lock();
    dac_hal_cw_generator_disable();
    irq_unlock(dac_context.spinlock);

    return ESP_OK;
}

esp_err_t dac_cw_generator_config(dac_cw_config_t *cw)
{
    ESP_RETURN_ON_FALSE(cw, ESP_ERR_INVALID_ARG, TAG, "invalid clock config");

    dac_context.spinlock = irq_lock();
    dac_hal_cw_generator_config(cw);
    irq_unlock(dac_context.spinlock);

    return ESP_OK;
}
