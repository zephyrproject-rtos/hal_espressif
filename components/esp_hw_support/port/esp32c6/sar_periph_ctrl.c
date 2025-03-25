/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * SAR related peripherals are interdependent. This file
 * provides a united control to these registers, as multiple
 * components require these controls.
 *
 * Related peripherals are:
 * - ADC
 * - PWDET
 */

#include <zephyr/kernel.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_private/sar_periph_ctrl.h"
#include "esp_private/esp_modem_clock.h"
#include "hal/sar_ctrl_ll.h"

static const char *TAG = "sar_periph_ctrl";

void sar_periph_ctrl_init(void)
{
    sar_ctrl_ll_force_power_ctrl_from_pwdet(true);

    //Add other periph power control initialisation here
}

void sar_periph_ctrl_power_enable(void)
{
    unsigned int key = irq_lock();
    sar_ctrl_ll_force_power_ctrl_from_pwdet(true);
    irq_unlock(key);
}

void sar_periph_ctrl_power_disable(void)
{
    unsigned int key = irq_lock();
    sar_ctrl_ll_force_power_ctrl_from_pwdet(false);
    irq_unlock(key);
}

/**
 * This gets incremented when s_sar_power_acquire() is called,
 * and decremented when s_sar_power_release() is called.
 * PWDET is powered down when the value reaches zero.
 * Should be modified within critical section.
 */
static int s_pwdet_power_on_cnt;

static void s_sar_power_acquire(void)
{
    modem_clock_module_enable(PERIPH_MODEM_ADC_COMMON_FE_MODULE);
    unsigned int key = irq_lock();
    s_pwdet_power_on_cnt++;
    if (s_pwdet_power_on_cnt == 1) {
        sar_ctrl_ll_set_power_mode_from_pwdet(SAR_CTRL_LL_POWER_ON);
    }
    irq_unlock(key);
}

static void s_sar_power_release(void)
{
    unsigned int key = irq_lock();
    s_pwdet_power_on_cnt--;
    if (s_pwdet_power_on_cnt < 0) {
        irq_unlock(key);
        ESP_LOGE(TAG, "%s called, but s_pwdet_power_on_cnt == 0", __func__);
        abort();
    } else if (s_pwdet_power_on_cnt == 0) {
        sar_ctrl_ll_set_power_mode_from_pwdet(SAR_CTRL_LL_POWER_FSM);
    }
    irq_unlock(key);
    modem_clock_module_disable(PERIPH_MODEM_ADC_COMMON_FE_MODULE);
}


/*------------------------------------------------------------------------------
* PWDET Power
*----------------------------------------------------------------------------*/
void sar_periph_ctrl_pwdet_power_acquire(void)
{
    s_sar_power_acquire();
}

void sar_periph_ctrl_pwdet_power_release(void)
{
    s_sar_power_release();
}


/*------------------------------------------------------------------------------
* ADC Power
*----------------------------------------------------------------------------*/
void sar_periph_ctrl_adc_oneshot_power_acquire(void)
{
    s_sar_power_acquire();
}

void sar_periph_ctrl_adc_oneshot_power_release(void)
{
    s_sar_power_release();
}

void sar_periph_ctrl_adc_continuous_power_acquire(void)
{
    s_sar_power_acquire();
}

void sar_periph_ctrl_adc_continuous_power_release(void)
{
    s_sar_power_release();
}
