/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"

#include "boot/esp_mcuboot_security.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Secure Boot preparation (Phase 1). Weak default is a no-op.
 * Full Secure Boot V2 can override these symbols and update @p ctx.
 */
esp_err_t esp_mcuboot_secure_boot_prepare(esp_mcuboot_security_ctx_t *ctx);
void esp_mcuboot_secure_boot_cancel(esp_mcuboot_security_ctx_t *ctx);
esp_err_t esp_mcuboot_secure_boot_finalize(esp_mcuboot_security_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
