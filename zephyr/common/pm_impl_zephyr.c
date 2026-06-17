/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Minimal PM implementation stubs for Zephyr.
 * These stubs are needed when pm_impl.c is not compiled (it requires FreeRTOS).
 */

#include <zephyr/pm/policy.h>
#include "esp_pm.h"
#include "esp_private/pm_impl.h"
#include "soc/rtc.h"
#include "soc/soc_caps.h"

esp_err_t esp_pm_get_configuration(void *vconfig)
{
    if (vconfig == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_pm_config_t *config = (esp_pm_config_t *)vconfig;
    rtc_cpu_freq_config_t freq_config;

    rtc_clk_cpu_freq_get_config(&freq_config);

    config->max_freq_mhz = freq_config.freq_mhz;
    config->min_freq_mhz = freq_config.freq_mhz;
#if CONFIG_PM
    config->light_sleep_enable = true;
#else
    config->light_sleep_enable = false;
#endif

    return ESP_OK;
}

int esp_pm_impl_get_cpu_freq(pm_mode_t mode)
{
    (void)mode;
    rtc_cpu_freq_config_t config;
    rtc_clk_cpu_freq_get_config(&config);
    return config.freq_mhz;
}

pm_mode_t esp_pm_impl_get_mode(esp_pm_lock_type_t type, int arg)
{
    (void)arg;

    switch (type) {
    case ESP_PM_CPU_FREQ_MAX:
        return PM_MODE_CPU_MAX;
    case ESP_PM_APB_FREQ_MAX:
        return PM_MODE_APB_MAX;
    case ESP_PM_NO_LIGHT_SLEEP:
        return PM_MODE_LIGHT_SLEEP;
    default:
        return PM_MODE_COUNT;
    }
}

void esp_pm_impl_switch_mode(pm_mode_t mode, pm_mode_switch_t lock_or_unlock, pm_time_t now)
{
    (void)mode;
    (void)now;

    /*
     * This function is kept in this file (not pm_locks_zephyr.c) to allow PM
     * policy to expand when we port IDF pm_impl: track locks per power mode,
     * pick the lowest allowed mode, then change CPU/APB clocks or sleep
     * policy — WiFi/BLE keep calling esp_pm_lock_* only.
     */
    if (lock_or_unlock == MODE_LOCK) {
        pm_policy_state_all_lock_get();
    } else {
        pm_policy_state_all_lock_put();
    }
}

#if SOC_VBAT_SUPPORTED
#include "esp_vbat.h"

esp_vbat_state_t esp_vbat_get_battery_state(void)
{
    return ESP_VBAT_STATE_NORMAL;
}
#endif
