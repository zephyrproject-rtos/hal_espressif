/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/__assert.h>

#include "hal/clk_gate_ll.h"
#include "esp_attr.h"
#include "esp_private/periph_ctrl.h"
#include "soc/soc_caps.h"

#if SOC_MODEM_CLOCK_IS_INDEPENDENT
#include "esp_private/esp_modem_clock.h"
#endif

unsigned int rcc_lock;
static atomic_t rcc_lock_counter;

static uint8_t ref_counts[PERIPH_MODULE_MAX] = {0};

IRAM_ATTR void periph_rcc_enter(void)
{
    if (atomic_inc(&rcc_lock_counter) == 0) {
        rcc_lock = irq_lock();
    }
}

IRAM_ATTR void periph_rcc_exit(void)
{
    __ASSERT_NO_MSG(atomic_get(&rcc_lock_counter) > 0);
    if (atomic_dec(&rcc_lock_counter) == 1) {
        irq_unlock(rcc_lock);
    }
}

IRAM_ATTR uint8_t periph_rcc_acquire_enter(periph_module_t periph)
{
    periph_rcc_enter();
    return ref_counts[periph];
}

IRAM_ATTR void periph_rcc_acquire_exit(periph_module_t periph, uint8_t ref_count)
{
    ref_counts[periph] = ++ref_count;
    periph_rcc_exit();
}

IRAM_ATTR uint8_t periph_rcc_release_enter(periph_module_t periph)
{
    periph_rcc_enter();
    return ref_counts[periph] - 1;
}

IRAM_ATTR void periph_rcc_release_exit(periph_module_t periph, uint8_t ref_count)
{
    ref_counts[periph] = ref_count;
    periph_rcc_exit();
}

void periph_module_enable(periph_module_t periph)
{
    assert(periph < PERIPH_MODULE_MAX);
    unsigned int key = irq_lock();
    if (ref_counts[periph] == 0) {
        periph_ll_enable_clk_clear_rst(periph);
    }
    ref_counts[periph]++;
    irq_unlock(key);
}

void periph_module_disable(periph_module_t periph)
{
    assert(periph < PERIPH_MODULE_MAX);
    unsigned int key = irq_lock();
    ref_counts[periph]--;
    if (ref_counts[periph] == 0) {
        periph_ll_disable_clk_set_rst(periph);
    }
    irq_unlock(key);
}

void periph_module_reset(periph_module_t periph)
{
    assert(periph < PERIPH_MODULE_MAX);
    unsigned int key = irq_lock();
    periph_ll_reset(periph);
    irq_unlock(key);
}

#if !SOC_IEEE802154_BLE_ONLY
IRAM_ATTR void wifi_bt_common_module_enable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_enable(PERIPH_PHY_MODULE);
#else
    unsigned int key = irq_lock();
    if (ref_counts[PERIPH_WIFI_BT_COMMON_MODULE] == 0) {
        periph_ll_wifi_bt_module_enable_clk();
    }
    ref_counts[PERIPH_WIFI_BT_COMMON_MODULE]++;
    irq_unlock(key);
#endif
}

IRAM_ATTR void wifi_bt_common_module_disable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_disable(PERIPH_PHY_MODULE);
#else
    unsigned int key = irq_lock();
    ref_counts[PERIPH_WIFI_BT_COMMON_MODULE]--;
    if (ref_counts[PERIPH_WIFI_BT_COMMON_MODULE] == 0) {
        periph_ll_wifi_bt_module_disable_clk();
    }
    irq_unlock(key);
#endif
}
#endif

#if CONFIG_WIFI_ESP32
void wifi_module_enable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_enable(PERIPH_WIFI_MODULE);
#else
    unsigned int key = irq_lock();
    periph_ll_wifi_module_enable_clk_clear_rst();
    irq_unlock(key);
#endif
}

void wifi_module_disable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_disable(PERIPH_WIFI_MODULE);
#else
    unsigned int key = irq_lock();
    periph_ll_wifi_module_disable_clk_set_rst();
    irq_unlock(key);
#endif
}
#endif // CONFIG_WIFI_ESP32
