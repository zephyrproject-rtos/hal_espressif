/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>

#include "boot/esp_mcuboot_secure_boot_hooks.h"

__attribute__((weak)) esp_err_t esp_mcuboot_secure_boot_prepare(esp_mcuboot_security_ctx_t *ctx)
{
	ARG_UNUSED(ctx);
	return ESP_OK;
}

__attribute__((weak)) void esp_mcuboot_secure_boot_cancel(esp_mcuboot_security_ctx_t *ctx)
{
	ARG_UNUSED(ctx);
}

__attribute__((weak)) esp_err_t esp_mcuboot_secure_boot_finalize(esp_mcuboot_security_ctx_t *ctx)
{
	ARG_UNUSED(ctx);
	return ESP_OK;
}
