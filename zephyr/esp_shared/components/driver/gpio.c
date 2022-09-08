/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <esp_types.h>
#include "esp_err.h"
#include <zephyr/kernel.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "soc/soc.h"
#include "soc/periph_defs.h"
#include "soc/soc_caps.h"
#include "soc/gpio_periph.h"
#include "esp_log.h"
#include "esp_check.h"
#include "hal/gpio_hal.h"
#include "esp_rom_gpio.h"

static const char *GPIO_TAG = "gpio";
#define GPIO_CHECK(a, str, ret_val) ESP_RETURN_ON_FALSE(a, ret_val, GPIO_TAG, "%s", str)

#define GPIO_ISR_CORE_ID_UNINIT    (3)

//default value for SOC_GPIO_SUPPORT_RTC_INDEPENDENT is 0
#ifndef SOC_GPIO_SUPPORT_RTC_INDEPENDENT
#define SOC_GPIO_SUPPORT_RTC_INDEPENDENT 0
#endif

typedef struct {
    gpio_isr_t fn;   /*!< isr function */
    void *args;      /*!< isr function args */
} gpio_isr_func_t;

typedef struct {
    gpio_hal_context_t *gpio_hal;
    unsigned int gpio_spinlock;
    uint32_t isr_core_id;
    gpio_isr_func_t *gpio_isr_func;
    gpio_isr_handle_t gpio_isr_handle;
} gpio_context_t;

static gpio_hal_context_t _gpio_hal = {
    .dev = GPIO_HAL_GET_HW(GPIO_PORT_0)
};

static gpio_context_t gpio_context = {
    .gpio_hal = &_gpio_hal,
    .gpio_spinlock = 0,
    .isr_core_id = GPIO_ISR_CORE_ID_UNINIT,
    .gpio_isr_func = NULL,
};

esp_err_t gpio_pullup_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
	gpio_context.gpio_spinlock = irq_lock();
        gpio_hal_pullup_en(gpio_context.gpio_hal, gpio_num);
	irq_unlock(gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pullup_en(gpio_num);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occured
#endif
    }

    return ESP_OK;
}

esp_err_t gpio_pullup_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
	gpio_context.gpio_spinlock = irq_lock();
        gpio_hal_pullup_dis(gpio_context.gpio_hal, gpio_num);
	irq_unlock(gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pullup_dis(gpio_num);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occured
#endif
    }

    return ESP_OK;
}

esp_err_t gpio_pulldown_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
	gpio_context.gpio_spinlock = irq_lock();
        gpio_hal_pulldown_en(gpio_context.gpio_hal, gpio_num);
	irq_unlock(gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pulldown_en(gpio_num);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occured
#endif
    }

    return ESP_OK;
}

esp_err_t gpio_pulldown_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
	gpio_context.gpio_spinlock = irq_lock();
        gpio_hal_pulldown_dis(gpio_context.gpio_hal, gpio_num);
	irq_unlock(gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pulldown_dis(gpio_num);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occured
#endif
    }

    return ESP_OK;
}

esp_err_t gpio_wakeup_enable(gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    if ((intr_type == GPIO_INTR_LOW_LEVEL) || (intr_type == GPIO_INTR_HIGH_LEVEL)) {
#if SOC_RTCIO_WAKE_SUPPORTED
        if (rtc_gpio_is_valid_gpio(gpio_num)) {
            ret = rtc_gpio_wakeup_enable(gpio_num, intr_type);
        }
#endif
        gpio_context.gpio_spinlock = irq_lock();
        gpio_hal_wakeup_enable(gpio_context.gpio_hal, gpio_num, intr_type);
#if SOC_GPIO_SUPPORT_SLP_SWITCH && CONFIG_ESP32C3_LIGHTSLEEP_GPIO_RESET_WORKAROUND
        gpio_hal_sleep_sel_dis(gpio_context.gpio_hal, gpio_num);
#endif
        irq_unlock(gpio_context.gpio_spinlock);
    } else {
        ESP_LOGE(GPIO_TAG, "GPIO wakeup only supports level mode, but edge mode set. gpio_num:%u", gpio_num);
        ret = ESP_ERR_INVALID_ARG;
    }

    return ret;
}

#if defined(__ZEPHYR__) && defined(CONFIG_PM)
esp_err_t esp_gpio_wakeup_enable(gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    return gpio_wakeup_enable(gpio_num, intr_type);
}
#endif

esp_err_t gpio_wakeup_disable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;
#if SOC_RTCIO_WAKE_SUPPORTED
    if (rtc_gpio_is_valid_gpio(gpio_num)) {
        ret = rtc_gpio_wakeup_disable(gpio_num);
    }
#endif
    gpio_context.gpio_spinlock = irq_lock();
    gpio_hal_wakeup_disable(gpio_context.gpio_hal, gpio_num);
#if SOC_GPIO_SUPPORT_SLP_SWITCH && CONFIG_ESP32C3_LIGHTSLEEP_GPIO_RESET_WORKAROUND
    gpio_hal_sleep_sel_en(gpio_context.gpio_hal, gpio_num);
#endif
    irq_unlock(gpio_context.gpio_spinlock);
    return ret;
}

esp_err_t gpio_hold_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "Only output-capable GPIO support this function", ESP_ERR_NOT_SUPPORTED);
    int ret = ESP_OK;

    if (rtc_gpio_is_valid_gpio(gpio_num)) {
#if SOC_RTCIO_HOLD_SUPPORTED
        ret = rtc_gpio_hold_en(gpio_num);
#endif
    } else if (GPIO_HOLD_MASK[gpio_num]) {
	gpio_context.gpio_spinlock = irq_lock();
        gpio_hal_hold_en(gpio_context.gpio_hal, gpio_num);
	irq_unlock(gpio_context.gpio_spinlock);
    } else {
        ret = ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

esp_err_t gpio_hold_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "Only output-capable GPIO support this function", ESP_ERR_NOT_SUPPORTED);
    int ret = ESP_OK;

    if (rtc_gpio_is_valid_gpio(gpio_num)) {
#if SOC_RTCIO_HOLD_SUPPORTED
        ret = rtc_gpio_hold_dis(gpio_num);
#endif
    }else if (GPIO_HOLD_MASK[gpio_num]) {
        gpio_context.gpio_spinlock = irq_lock();
        gpio_hal_hold_dis(gpio_context.gpio_hal, gpio_num);
        irq_unlock(gpio_context.gpio_spinlock);
    } else {
        ret = ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

void gpio_deep_sleep_hold_en(void)
{
    gpio_context.gpio_spinlock = irq_lock();
    gpio_hal_deep_sleep_hold_en(gpio_context.gpio_hal);
    irq_unlock(gpio_context.gpio_spinlock);
}

void gpio_deep_sleep_hold_dis(void)
{
    gpio_context.gpio_spinlock = irq_lock();
    gpio_hal_deep_sleep_hold_dis(gpio_context.gpio_hal);
    irq_unlock(gpio_context.gpio_spinlock);
}

#if SOC_GPIO_SUPPORT_FORCE_HOLD

esp_err_t gpio_force_hold_all()
{
#if SOC_RTCIO_HOLD_SUPPORTED
    rtc_gpio_force_hold_all();
#endif
    gpio_context.gpio_spinlock = irq_lock();
    gpio_hal_force_hold_all(gpio_context.gpio_hal);
    irq_unlock(gpio_context.gpio_spinlock);
    return ESP_OK;
}

esp_err_t gpio_force_unhold_all()
{
#if SOC_RTCIO_HOLD_SUPPORTED
    rtc_gpio_force_hold_dis_all();
#endif
    gpio_context.gpio_spinlock = irq_lock();
    gpio_hal_force_unhold_all();
    irq_unlock(gpio_context.gpio_spinlock);
    return ESP_OK;
}
#endif

void gpio_iomux_in(uint32_t gpio, uint32_t signal_idx)
{
    gpio_hal_iomux_in(gpio_context.gpio_hal, gpio, signal_idx);
}

void gpio_iomux_out(uint8_t gpio_num, int func, bool oen_inv)
{
    gpio_hal_iomux_out(gpio_context.gpio_hal, gpio_num, func, (uint32_t)oen_inv);
}

#if SOC_GPIO_SUPPORT_SLP_SWITCH
esp_err_t gpio_sleep_sel_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    gpio_context.gpio_spinlock = irq_lock();
    gpio_hal_sleep_sel_en(gpio_context.gpio_hal, gpio_num);
    irq_unlock(gpio_context.gpio_spinlock);

    return ESP_OK;
}

esp_err_t gpio_sleep_sel_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    gpio_context.gpio_spinlock = irq_lock();
    gpio_hal_sleep_sel_dis(gpio_context.gpio_hal, gpio_num);
    irq_unlock(gpio_context.gpio_spinlock);

    return ESP_OK;
}

#if CONFIG_GPIO_ESP32_SUPPORT_SWITCH_SLP_PULL
esp_err_t gpio_sleep_pupd_config_apply(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_pupd_config_apply(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}

esp_err_t gpio_sleep_pupd_config_unapply(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_pupd_config_unapply(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}
#endif // CONFIG_GPIO_ESP32_SUPPORT_SWITCH_SLP_PULL
#endif // SOC_GPIO_SUPPORT_SLP_SWITCH

#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
esp_err_t gpio_deep_sleep_wakeup_enable(gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    if (!gpio_hal_is_valid_deepsleep_wakeup_gpio(gpio_num)) {
        ESP_LOGE(GPIO_TAG, "GPIO %d does not support deep sleep wakeup", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    if ((intr_type != GPIO_INTR_LOW_LEVEL) && (intr_type != GPIO_INTR_HIGH_LEVEL)) {
        ESP_LOGE(GPIO_TAG, "GPIO wakeup only supports level mode, but edge mode set. gpio_num:%u", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    gpio_context.gpio_spinlock = irq_lock();
    gpio_hal_deepsleep_wakeup_enable(gpio_context.gpio_hal, gpio_num, intr_type);
#if SOC_GPIO_SUPPORT_SLP_SWITCH && CONFIG_ESP32C3_LIGHTSLEEP_GPIO_RESET_WORKAROUND
    gpio_hal_sleep_sel_dis(gpio_context.gpio_hal, gpio_num);
#endif
    irq_unlock(gpio_context.gpio_spinlock);
    return ESP_OK;
}

esp_err_t gpio_deep_sleep_wakeup_disable(gpio_num_t gpio_num)
{
    if (!gpio_hal_is_valid_deepsleep_wakeup_gpio(gpio_num)) {
        ESP_LOGE(GPIO_TAG, "GPIO %d does not support deep sleep wakeup", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    gpio_context.gpio_spinlock = irq_lock();
    gpio_hal_deepsleep_wakeup_disable(gpio_context.gpio_hal, gpio_num);
#if SOC_GPIO_SUPPORT_SLP_SWITCH && CONFIG_ESP32C3_LIGHTSLEEP_GPIO_RESET_WORKAROUND
    gpio_hal_sleep_sel_en(gpio_context.gpio_hal, gpio_num);
#endif
    irq_unlock(gpio_context.gpio_spinlock);
    return ESP_OK;
}
#endif // SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
