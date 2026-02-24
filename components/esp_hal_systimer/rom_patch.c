/*
 * SPDX-FileCopyrightText: 2022-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief ROM patch for systimer HAL
 *
 * Some chips have systimer HAL implementations in ROM that require patches.
 * This file provides the necessary patches when ROM implementation is used.
 *
 * For chips with ESP_ROM_SYSTIMER_INIT_PATCH defined (e.g., ESP32-C5, ESP32-C6,
 * ESP32-H2, ESP32-P4), the ROM systimer_hal_init/deinit functions do not
 * enable ETM, so we need to patch them here.
 */

#include <stddef.h>
#include "esp_rom_caps.h"
#include "hal/systimer_hal.h"
#include "hal/systimer_ll.h"

#if ESP_ROM_SYSTIMER_INIT_PATCH
void systimer_hal_init(systimer_hal_context_t *hal)
{
    hal->dev = &SYSTIMER;
    systimer_ll_enable_clock(hal->dev, true);
    systimer_ll_enable_etm(&SYSTIMER, true);
}

void systimer_hal_deinit(systimer_hal_context_t *hal)
{
    systimer_ll_enable_etm(&SYSTIMER, false);
    systimer_ll_enable_clock(hal->dev, false);
    hal->dev = NULL;
}
#endif // ESP_ROM_SYSTIMER_INIT_PATCH
