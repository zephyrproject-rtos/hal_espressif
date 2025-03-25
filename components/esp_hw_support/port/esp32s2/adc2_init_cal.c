/*
 * SPDX-FileCopyrightText: 2016-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* This file is used to get `adc2_init_code_calibration` executed before the APP when the ADC2 is used by Wi-Fi or other drivers.
The linker will link constructor (adc2_init_code_calibration) only when any sections inside the same file (adc2_cal_include) is used.
Don't put any other code into this file. */

#include <zephyr/kernel.h>

#include "hal/adc_types.h"
#include "hal/adc_hal_common.h"
#include "esp_private/adc_share_hw_ctrl.h"

/**
 * @brief Set initial code to ADC2 after calibration. ADC2 RTC and ADC2 PWDET controller share the initial code.
 *        This API be called in before `app_main()`.
 */
void adc2_init_code_calibration(void)
{
    adc_hal_calibration_init(ADC_UNIT_2);
    adc_calc_hw_calibration_code(ADC_UNIT_2, ADC_ATTEN_DB_12);
    unsigned int key = irq_lock();
    adc_set_hw_calibration_code(ADC_UNIT_2, ADC_ATTEN_DB_12);
    irq_unlock(key);
}

/** Don't call `adc2_cal_include` in user code. */
void adc2_cal_include(void)
{
    /* When this empty function is called, the `adc2_init_code_calibration` constructor will be linked and executed before the app.*/
}
