/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Secure Boot preparation (Phase 1). Weak default is a no-op.
 * Full Secure Boot V2 can override these symbols.
 */
esp_err_t esp_mcuboot_secure_boot_prepare(void);
void esp_mcuboot_secure_boot_cancel(void);
esp_err_t esp_mcuboot_secure_boot_finalize(void);

#ifdef __cplusplus
}
#endif
