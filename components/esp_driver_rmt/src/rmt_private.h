/*
 * SPDX-FileCopyrightText: 2022-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "soc/soc_caps.h"
#include "hal/rmt_ll.h"
#include "esp_private/sleep_retention.h"
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#if SOC_RMT_SUPPORT_SLEEP_RETENTION
typedef struct {
    periph_retention_module_t module;
    const regdma_entries_config_t *regdma_entry_array;
    uint32_t array_size;
} rmt_retention_desc_t;

extern const rmt_retention_desc_t rmt_retention_infos[RMT_LL_GET(INST_NUM)];
#endif /* SOC_RMT_SUPPORT_SLEEP_RETENTION */

#ifdef __cplusplus
}
#endif
