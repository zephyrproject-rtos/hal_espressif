/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Minimal PM implementation stubs for Zephyr.
 * These stubs are needed when pm_impl.c is not compiled (it requires FreeRTOS).
 */

#include "esp_private/pm_impl.h"
#include "soc/rtc.h"
#include "soc/soc_caps.h"

int esp_pm_impl_get_cpu_freq(pm_mode_t mode)
{
    (void)mode;
    rtc_cpu_freq_config_t config;
    rtc_clk_cpu_freq_get_config(&config);
    return config.freq_mhz;
}

#if SOC_VBAT_SUPPORTED
#include "esp_vbat.h"

esp_vbat_state_t esp_vbat_get_battery_state(void)
{
    return ESP_VBAT_STATE_NORMAL;
}
#endif
