/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>

#include "esp_crypto_periph_clk.h"
#include "esp_private/periph_ctrl.h"
#include "hal/ecc_ll.h"

void esp_crypto_ecc_enable_periph_clk(bool enable)
{
	PERIPH_RCC_ATOMIC() {
		ecc_ll_enable_bus_clock(enable);
		if (enable) {
			ecc_ll_reset_register();
		}
	}
}
