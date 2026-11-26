/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BREDR_ECC_P192_H_
#define BREDR_ECC_P192_H_

#include <stdint.h>

/* secp192r1 on the ECC accelerator. Coordinates, keys and the DH key are
 * little endian, 24 bytes each. The public key is X followed by Y.
 */
int bredr_ecc_p192_gen_keypair(uint8_t *public_key, uint8_t *private_key);

int bredr_ecc_p192_gen_dhkey(const uint8_t *peer_x, const uint8_t *peer_y,
			     const uint8_t *private_key, uint8_t *dhkey);

#endif /* BREDR_ECC_P192_H_ */
