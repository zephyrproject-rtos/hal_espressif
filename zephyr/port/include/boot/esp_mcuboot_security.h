/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Shared security context owned by esp_mcuboot_security.c.
 * Secure Boot / Flash Encryption prepare hooks fill their fields;
 * finalize uses them for orchestration (including a future
 * CONFIG_SECURE_BOOT_FLASH_ENC_KEYS_BURN_TOGETHER path).
 */
typedef struct {
	bool flash_encryption_already_enabled;
	bool secure_boot_already_enabled;
} esp_mcuboot_security_ctx_t;

/**
 * Phase 1: virtual eFuse init (if enabled), then Secure Boot and Flash
 * Encryption prepare hooks.
 */
esp_err_t esp_mcuboot_security_prepare(void);

/** Cancel pending Secure Boot / Flash Encryption work after a failed boot_go(). */
void esp_mcuboot_security_cancel(void);

/**
 * Phase 3: finalize Secure Boot, then Flash Encryption.
 * May reset the chip (does not return) after enabling Flash Encryption.
 */
esp_err_t esp_mcuboot_security_finalize(void);

#ifdef __cplusplus
}
#endif
