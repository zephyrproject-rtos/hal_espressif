/*
 * SPDX-FileCopyrightText: 2019-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc/soc_caps.h"
#include "esp_private/esp_system_attr.h"
#include "esp32s3/rom/apb_backup_dma.h"
#include "esp_private/critical_section.h"

static esp_os_spinlock_t s_apb_backup_dma_lock = ESP_OS_SPINLOCK_INIT;

static void ESP_SYSTEM_IRAM_ATTR apb_backup_dma_lock(void)
{
    esp_os_enter_critical_safe(&s_apb_backup_dma_lock);
}

static void ESP_SYSTEM_IRAM_ATTR apb_backup_dma_unlock(void)
{
    esp_os_exit_critical_safe(&s_apb_backup_dma_lock);
}

void esp_apb_backup_dma_lock_init(void)
{
    ets_apb_backup_init_lock_func(apb_backup_dma_lock, apb_backup_dma_unlock);
}
