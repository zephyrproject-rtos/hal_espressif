/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ulp_lp_core_memory_shared.h"

#include "sdkconfig.h"
#include "soc/soc.h"
#include "esp_rom_caps.h"
#include "esp_assert.h"
#include <zephyr/devicetree.h>

#define ULP_SHARED_MEM DT_REG_SIZE(DT_NODELABEL(ulp_shm))

/* The ulp_shm DTS node reserves memory for a shared cfg struct
   between the HP core and LP core. */

#if IS_ULP_COCPU
static ulp_lp_core_memory_shared_cfg_t __attribute__((section(".shared_mem"))) s_shared_mem = {};
ESP_STATIC_ASSERT(DT_REG_SIZE(DT_NODELABEL(ulp_shm)) >= sizeof(ulp_lp_core_memory_shared_cfg_t));
#endif

ulp_lp_core_memory_shared_cfg_t* ulp_lp_core_memory_shared_cfg_get(void)
{
#if IS_ULP_COCPU
    return &s_shared_mem;
#else
    return (ulp_lp_core_memory_shared_cfg_t *)DT_REG_ADDR(DT_NODELABEL(ulp_shm));
#endif
}
