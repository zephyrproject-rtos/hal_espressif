/*
 * SPDX-FileCopyrightText: 2019-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc/soc_caps.h"
#include "esp_private/esp_system_attr.h"
#include "esp32s3/rom/apb_backup_dma.h"
#include <zephyr/spinlock.h>

static struct k_spinlock s_apb_backup_dma_lock;
static k_spinlock_key_t s_apb_backup_dma_key;

static void ESP_SYSTEM_IRAM_ATTR apb_backup_dma_lock(void)
{
    s_apb_backup_dma_key = k_spin_lock(&s_apb_backup_dma_lock);
}

static void ESP_SYSTEM_IRAM_ATTR apb_backup_dma_unlock(void)
{
    k_spin_unlock(&s_apb_backup_dma_lock, s_apb_backup_dma_key);
}

void esp_apb_backup_dma_lock_init(void)
{
    ets_apb_backup_init_lock_func(apb_backup_dma_lock, apb_backup_dma_unlock);
}
