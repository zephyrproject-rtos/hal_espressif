/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>

#include "hal/clk_gate_ll.h"
#include "esp_attr.h"
#include "esp_private/periph_ctrl.h"
#include "soc/soc_caps.h"

#if SOC_MODEM_CLOCK_IS_INDEPENDENT
#include "esp_private/esp_modem_clock.h"
#endif

static int periph_spinlock;

#define ENTER_CRITICAL_SECTION()    do { periph_spinlock = irq_lock(); } while(0)
#define LEAVE_CRITICAL_SECTION()    irq_unlock(periph_spinlock);

static uint8_t ref_counts[PERIPH_MODULE_MAX] = {0};

void periph_module_enable(periph_module_t periph)
{
    assert(periph < PERIPH_MODULE_MAX);
    ENTER_CRITICAL_SECTION();
    if (ref_counts[periph] == 0) {
        periph_ll_enable_clk_clear_rst(periph);
    }
    ref_counts[periph]++;
    LEAVE_CRITICAL_SECTION();
}

void periph_module_disable(periph_module_t periph)
{
    assert(periph < PERIPH_MODULE_MAX);
    ENTER_CRITICAL_SECTION();
    ref_counts[periph]--;
    if (ref_counts[periph] == 0) {
        periph_ll_disable_clk_set_rst(periph);
    }
    LEAVE_CRITICAL_SECTION();
}

void periph_module_reset(periph_module_t periph)
{
    assert(periph < PERIPH_MODULE_MAX);
    ENTER_CRITICAL_SECTION();
    periph_ll_reset(periph);
    LEAVE_CRITICAL_SECTION();
}

#if !SOC_IEEE802154_BLE_ONLY
IRAM_ATTR void wifi_bt_common_module_enable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_enable(PERIPH_PHY_MODULE);
#else
    ENTER_CRITICAL_SECTION();
    if (ref_counts[PERIPH_WIFI_BT_COMMON_MODULE] == 0) {
        periph_ll_wifi_bt_module_enable_clk();
    }
    ref_counts[PERIPH_WIFI_BT_COMMON_MODULE]++;
    LEAVE_CRITICAL_SECTION();
#endif
}

IRAM_ATTR void wifi_bt_common_module_disable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_disable(PERIPH_PHY_MODULE);
#else
    ENTER_CRITICAL_SECTION();
    ref_counts[PERIPH_WIFI_BT_COMMON_MODULE]--;
    if (ref_counts[PERIPH_WIFI_BT_COMMON_MODULE] == 0) {
        periph_ll_wifi_bt_module_disable_clk();
    }
    LEAVE_CRITICAL_SECTION();
#endif
}
#endif

#if CONFIG_WIFI_ESP32
void wifi_module_enable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_enable(PERIPH_WIFI_MODULE);
#else
    ENTER_CRITICAL_SECTION();
    periph_ll_wifi_module_enable_clk_clear_rst();
    LEAVE_CRITICAL_SECTION();
#endif
}

void wifi_module_disable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_disable(PERIPH_WIFI_MODULE);
#else
    ENTER_CRITICAL_SECTION();
    periph_ll_wifi_module_disable_clk_set_rst();
    LEAVE_CRITICAL_SECTION();
#endif
}
#endif // CONFIG_WIFI_ESP32
