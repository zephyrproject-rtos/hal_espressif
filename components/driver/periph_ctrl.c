// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <zephyr.h>
#include "freertos/FreeRTOS.h"
#include "hal/clk_gate_ll.h"
#include "driver/periph_ctrl.h"

static unsigned int periph_spinlock;

static uint8_t ref_counts[PERIPH_MODULE_MAX] = {0};

void periph_module_enable(periph_module_t periph)
{
    assert(periph < PERIPH_MODULE_MAX);
    periph_spinlock = irq_lock();
    if (ref_counts[periph] == 0) {
        periph_ll_enable_clk_clear_rst(periph);
    }
    ref_counts[periph]++;
    irq_unlock(periph_spinlock);
}

void periph_module_disable(periph_module_t periph)
{
    assert(periph < PERIPH_MODULE_MAX);
    periph_spinlock = irq_lock();
    ref_counts[periph]--;
    if (ref_counts[periph] == 0) {
        periph_ll_disable_clk_set_rst(periph);
    }
    irq_unlock(periph_spinlock);
}

void periph_module_reset(periph_module_t periph)
{
    assert(periph < PERIPH_MODULE_MAX);
    periph_spinlock = irq_lock();
    periph_ll_reset(periph);
    irq_unlock(periph_spinlock);
}

IRAM_ATTR void wifi_bt_common_module_enable(void)
{
    periph_spinlock = irq_lock();
    if (ref_counts[PERIPH_WIFI_BT_COMMON_MODULE] == 0) {
        periph_ll_wifi_bt_module_enable_clk_clear_rst();
    }
    ref_counts[PERIPH_WIFI_BT_COMMON_MODULE]++;
    irq_unlock(periph_spinlock);
}

IRAM_ATTR void wifi_bt_common_module_disable(void)
{
    periph_spinlock = irq_lock();
    ref_counts[PERIPH_WIFI_BT_COMMON_MODULE]--;
    if (ref_counts[PERIPH_WIFI_BT_COMMON_MODULE] == 0) {
        periph_ll_wifi_bt_module_disable_clk_set_rst();
    }
    irq_unlock(periph_spinlock);
} 