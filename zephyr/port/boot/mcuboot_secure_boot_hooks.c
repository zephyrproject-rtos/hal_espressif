/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mcuboot_secure_boot_hooks.h"

__attribute__((weak)) esp_err_t esp_mcuboot_secure_boot_prepare(void)
{
	return ESP_OK;
}

__attribute__((weak)) void esp_mcuboot_secure_boot_cancel(void)
{
}

__attribute__((weak)) esp_err_t esp_mcuboot_secure_boot_finalize(void)
{
	return ESP_OK;
}
