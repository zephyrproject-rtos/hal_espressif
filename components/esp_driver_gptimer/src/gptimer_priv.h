/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "soc/soc_caps.h"
#include "hal/timer_ll.h"
#include "esp_private/sleep_retention.h"
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#if SOC_PAU_SUPPORTED && SOC_TIMER_SUPPORT_SLEEP_RETENTION
#include "soc/retention_periph_defs.h"

typedef struct {
    const periph_retention_module_t module;
    const regdma_entries_config_t *regdma_entry_array;
    const size_t array_size;
} gptimer_retention_desc_t;

extern const gptimer_retention_desc_t gptimer_retention_infos[TIMG_LL_GET(INST_NUM)][TIMG_LL_GET(GPTIMERS_PER_INST)];
#endif /* SOC_PAU_SUPPORTED && SOC_TIMER_SUPPORT_SLEEP_RETENTION */

#ifdef __cplusplus
}
#endif
