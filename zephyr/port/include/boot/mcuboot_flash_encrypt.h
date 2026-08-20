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

/** Returned from finalize when the device must reset after enabling FE */
#define ESP_ERR_FLASH_ENCRYPT_RESET_NEEDED 0x11001

/**
 * Phase 1: probe eFuse state, virtual eFuse init, begin batched writes,
 * and secure-boot key preparation.
 */
esp_err_t esp_mcuboot_security_prepare(void);

/** Cancel pending eFuse batch writes after a failed boot_go(). */
void esp_mcuboot_security_cancel(void);

/**
 * Phase 3: burn eFuses, encrypt flash regions, enable hardware FE.
 * Returns ESP_ERR_FLASH_ENCRYPT_RESET_NEEDED if a reset is required.
 */
esp_err_t esp_mcuboot_security_finalize(void);

#ifdef __cplusplus
}
#endif
