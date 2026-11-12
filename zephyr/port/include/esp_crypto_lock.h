/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "soc/soc_caps.h"

#ifdef __cplusplus
extern "C" {
#endif

void esp_crypto_ecc_lock_acquire(void);

void esp_crypto_ecc_lock_release(void);

#ifdef __cplusplus
}
#endif
