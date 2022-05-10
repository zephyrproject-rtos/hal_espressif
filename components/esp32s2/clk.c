// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
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

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#endif
#include <stdint.h>
#include <sys/param.h>

#include "esp_attr.h"
#include "soc/rtc.h"
#include "esp32s2/clk.h"

#ifndef __ZEPHYR__
#define MHZ (1000000)
#endif

// g_ticks_us defined in ROMs
extern uint32_t g_ticks_per_us_pro;

int IRAM_ATTR esp_clk_cpu_freq(void)
{
#ifdef __ZEPHYR__
    return MHZ(g_ticks_per_us_pro);
#else
    return g_ticks_per_us_pro * MHZ;
#endif
}

int IRAM_ATTR esp_clk_apb_freq(void)
{
#ifdef __ZEPHYR__
    return MHZ(MIN(g_ticks_per_us_pro, 80));
#else
    return MIN(g_ticks_per_us_pro, 80) * MHZ;
#endif
}

int IRAM_ATTR esp_clk_xtal_freq(void)
{
#ifdef __ZEPHYR__
    return MHZ(rtc_clk_xtal_freq_get());
#else
    return rtc_clk_xtal_freq_get() * MHZ;
#endif
}

void IRAM_ATTR ets_update_cpu_frequency(uint32_t ticks_per_us)
{
    /* Update scale factors used by esp_rom_delay_us */
    g_ticks_per_us_pro = ticks_per_us;
}
