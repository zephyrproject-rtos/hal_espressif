// Copyright 2020 Espressif Systems (Shanghai) PTE LTD
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

#include <assert.h>
#include "esp_log_private.h"

#ifdef CONFIG_IDF_TARGET_ESP32C3
#include "hal/cpu_hal.h"  // for cpu_hal_get_cycle_count()
#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#endif
#else 
#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#endif
#include "hal/cpu_hal.h"  // for cpu_hal_get_cycle_count()
#endif


static int s_lock = 0;

void esp_log_impl_lock(void)
{
    assert(s_lock == 0);
    s_lock = 1;
}

bool esp_log_impl_lock_timeout(void)
{
    esp_log_impl_lock();
    return true;
}

void esp_log_impl_unlock(void)
{
    assert(s_lock == 1);
    s_lock = 0;
}

/* FIXME: define an API for getting the timestamp in soc/hal IDF-2351 */
uint32_t esp_log_early_timestamp(void)
{
#ifdef __ZEPHYR__
        return k_uptime_get_32();
#endif

    extern uint32_t ets_get_cpu_frequency(void);
    return cpu_hal_get_cycle_count() / (ets_get_cpu_frequency() * 1000);
}

uint32_t esp_log_timestamp(void) __attribute__((alias("esp_log_early_timestamp")));
