/*
 * SPDX-FileCopyrightText: 2022-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "soc/soc_caps.h"
#include "hal/mcpwm_ll.h"
#include "esp_private/sleep_retention.h"
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#if SOC_MCPWM_SUPPORT_SLEEP_RETENTION
typedef struct {
    const regdma_entries_config_t *regdma_entry_array;
    const size_t array_size;
    const periph_retention_module_t retention_module;
} mcpwm_retention_desc_t;

extern const mcpwm_retention_desc_t mcpwm_retention_infos[MCPWM_LL_GET(GROUP_NUM)];
#endif /* SOC_MCPWM_SUPPORT_SLEEP_RETENTION */

#ifdef __cplusplus
}
#endif
