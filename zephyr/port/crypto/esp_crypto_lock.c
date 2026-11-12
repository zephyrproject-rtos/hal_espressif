/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include "esp_crypto_lock.h"

static K_MUTEX_DEFINE(s_crypto_ecc_lock);

void esp_crypto_ecc_lock_acquire(void)
{
	(void)k_mutex_lock(&s_crypto_ecc_lock, K_FOREVER);
}

void esp_crypto_ecc_lock_release(void)
{
	(void)k_mutex_unlock(&s_crypto_ecc_lock);
}
