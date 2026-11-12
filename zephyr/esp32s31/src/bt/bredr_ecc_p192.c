/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "esp_private/periph_ctrl.h"
#include "esp_random.h"
#include "hal/ecc_hal.h"
#include "hal/ecc_ll.h"

#include "bredr_ecc_p192.h"

#define P192_LEN 24U

/* secp192r1 generator and order, little endian. */
static const uint8_t p192_gx[P192_LEN] = {
	0x12, 0x10, 0xff, 0x82, 0xfd, 0x0a, 0xff, 0xf4,
	0x00, 0x88, 0xa1, 0x43, 0xeb, 0x20, 0xbf, 0x7c,
	0xf6, 0x90, 0x30, 0xb0, 0x0e, 0xa8, 0x8d, 0x18,
};
static const uint8_t p192_gy[P192_LEN] = {
	0x11, 0x48, 0x79, 0x1e, 0xa1, 0x77, 0xf9, 0x73,
	0xd5, 0xcd, 0x24, 0x6b, 0xed, 0x11, 0x10, 0x63,
	0x78, 0xda, 0xc8, 0xff, 0x95, 0x2b, 0x19, 0x07,
};
static const uint8_t p192_n[P192_LEN] = {
	0x31, 0x28, 0xd2, 0xb4, 0xb1, 0xc9, 0x6b, 0x14,
	0x36, 0xf8, 0xde, 0x99, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

static K_MUTEX_DEFINE(p192_lock);

static bool p192_scalar_is_valid(const uint8_t *k)
{
	bool nonzero = false;
	int i;

	for (i = 0; i < (int)P192_LEN; i++) {
		if (k[i] != 0U) {
			nonzero = true;
			break;
		}
	}

	if (!nonzero) {
		return false;
	}

	for (i = (int)P192_LEN - 1; i >= 0; i--) {
		if (k[i] != p192_n[i]) {
			return k[i] < p192_n[i];
		}
	}

	return false;
}

static int p192_point_mul(const uint8_t *k, const uint8_t *px, const uint8_t *py, uint8_t *rx,
			  uint8_t *ry, bool verify_first)
{
	int ret;

	k_mutex_lock(&p192_lock, K_FOREVER);

	PERIPH_RCC_ATOMIC() {
		ecc_ll_enable_bus_clock(true);
		ecc_ll_reset_register();
	}

	ecc_hal_write_mul_param(k, px, py, P192_LEN);
	ecc_hal_set_mode(verify_first ? ECC_MODE_VERIFY_THEN_POINT_MUL : ECC_MODE_POINT_MUL);
	ecc_hal_enable_constant_time_point_mul(true);
	ecc_hal_start_calc();

	while (ecc_hal_is_calc_finished() == 0) {
	}

	ret = ecc_hal_read_mul_result(rx, ry, P192_LEN);

	PERIPH_RCC_ATOMIC() {
		ecc_ll_enable_bus_clock(false);
	}

	k_mutex_unlock(&p192_lock);

	return ret;
}

int bredr_ecc_p192_gen_keypair(uint8_t *public_key, uint8_t *private_key)
{
	do {
		esp_fill_random(private_key, P192_LEN);
	} while (!p192_scalar_is_valid(private_key));

	return p192_point_mul(private_key, p192_gx, p192_gy, public_key, &public_key[P192_LEN],
			      false);
}

int bredr_ecc_p192_gen_dhkey(const uint8_t *peer_x, const uint8_t *peer_y,
			     const uint8_t *private_key, uint8_t *dhkey)
{
	uint8_t ry[P192_LEN];
	int ret;

	if (!p192_scalar_is_valid(private_key)) {
		return -1;
	}

	ret = p192_point_mul(private_key, peer_x, peer_y, dhkey, ry, true);
	memset(ry, 0, sizeof(ry));

	return ret;
}
