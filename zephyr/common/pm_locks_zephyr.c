/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/kernel.h>

#include "esp_pm.h"
#include "esp_attr.h"
#include "esp_private/pm_impl.h"


typedef struct esp_pm_lock {
    esp_pm_lock_type_t type;
    int arg;
    pm_mode_t mode;
    const char *name;
    size_t count;
    struct k_spinlock spinlock;
} esp_pm_lock_t;


esp_err_t esp_pm_lock_create(esp_pm_lock_type_t lock_type, int arg,
                             const char *name, esp_pm_lock_handle_t *out_handle)
{
#ifndef CONFIG_PM
    return ESP_ERR_NOT_SUPPORTED;
#endif

    if (out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_pm_lock_t *new_lock = (esp_pm_lock_t *)calloc(1, sizeof(*new_lock));

    if (!new_lock) {
        return ESP_ERR_NO_MEM;
    }

    new_lock->type = lock_type;
    new_lock->arg = arg;
    new_lock->mode = esp_pm_impl_get_mode(lock_type, arg);
    new_lock->name = name;
    *out_handle = new_lock;

    return ESP_OK;
}

esp_err_t esp_pm_lock_delete(esp_pm_lock_handle_t handle)
{
#ifndef CONFIG_PM
    return ESP_ERR_NOT_SUPPORTED;
#endif

    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->count > 0) {
        return ESP_ERR_INVALID_STATE;
    }

    free(handle);

    return ESP_OK;
}

esp_err_t IRAM_ATTR esp_pm_lock_acquire(esp_pm_lock_handle_t handle)
{
#ifndef CONFIG_PM
    return ESP_ERR_NOT_SUPPORTED;
#endif

    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    k_spinlock_key_t key = k_spin_lock(&handle->spinlock);

    if (handle->count++ == 0) {
        esp_pm_impl_switch_mode(handle->mode, MODE_LOCK, 0);
    }

    k_spin_unlock(&handle->spinlock, key);

    return ESP_OK;
}

esp_err_t IRAM_ATTR esp_pm_lock_release(esp_pm_lock_handle_t handle)
{
#ifndef CONFIG_PM
    return ESP_ERR_NOT_SUPPORTED;
#endif

    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    k_spinlock_key_t key = k_spin_lock(&handle->spinlock);

    if (handle->count == 0) {
        k_spin_unlock(&handle->spinlock, key);
        return ESP_ERR_INVALID_STATE;
    }

    if (--handle->count == 0) {
        esp_pm_impl_switch_mode(handle->mode, MODE_UNLOCK, 0);
    }

    k_spin_unlock(&handle->spinlock, key);

    return ESP_OK;
}
