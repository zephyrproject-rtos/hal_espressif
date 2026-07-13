/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include "soc/soc_caps.h"
#include "hal/gdma_ll.h"
#include "esp_private/sleep_retention.h"
#include <zephyr/sys/util.h>

#if SOC_PHY_SUPPORTED
#define GDMA_RETENTION_ENTRY    (ENTRY(0) | ENTRY(2))
#else
#define GDMA_RETENTION_ENTRY    (ENTRY(0))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if SOC_GDMA_SUPPORT_SLEEP_RETENTION && SOC_PAU_SUPPORTED
typedef struct {
    const regdma_entries_config_t *link_list;
    uint32_t link_num;
    const periph_retention_module_t module_id;
} gdma_retention_desc_t;

extern const gdma_retention_desc_t gdma_retention_infos[GDMA_LL_GET(INST_NUM)][GDMA_LL_GET(PAIRS_PER_INST)];

#endif // SOC_GDMA_SUPPORT_SLEEP_RETENTION && SOC_PAU_SUPPORTED

#ifdef __cplusplus
}
#endif
