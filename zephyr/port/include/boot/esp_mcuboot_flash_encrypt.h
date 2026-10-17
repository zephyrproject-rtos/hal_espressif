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
 * Flash Encryption prepare (Phase 1). Probes FE enablement into @p ctx.
 */
esp_err_t esp_mcuboot_flash_encryption_prepare(esp_mcuboot_security_ctx_t *ctx);

/** Cancel pending Flash Encryption work after a failed boot_go(). */
void esp_mcuboot_flash_encryption_cancel(esp_mcuboot_security_ctx_t *ctx);

/**
 * Flash Encryption finalize (Phase 3): encrypt flash regions and enable FE.
 * Called by esp_mcuboot_security only when FE was not already enabled.
 */
esp_err_t esp_mcuboot_flash_encryption_finalize(esp_mcuboot_security_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
