/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/toolchain.h>

#include "btdm_endian.h"

#define BLE_SM_KEY_ERR  0x17
#define BLE_SM_KEY_LEN  32
#define BLE_SM_AES_LEN  16

int ble_sm_alg_gen_key_pair(uint8_t *pub, uint8_t *priv);
int ble_sm_alg_gen_dhkey(const uint8_t *peer_pub_key_x, const uint8_t *peer_pub_key_y,
			 const uint8_t *our_priv_key, uint8_t *out_dhkey);
int ble_sm_ecc_aes_cmac(const uint8_t *key, const uint8_t *in, size_t len, uint8_t *out);

#if defined(CONFIG_ESP32_BT_LE_CRYPTO_STACK_MBEDTLS)

#include <mbedtls/platform_util.h>
#include <psa/crypto.h>

/* Uncompressed P-256 point: 0x04 prefix, then X and Y */
#define BLE_SM_PUB_POINT_LEN (1 + (2 * BLE_SM_KEY_LEN))

#if defined(CONFIG_ESP32_BT_LE_SM_SC_DEBUG_KEYS)
/* Core Specification Vol 3, Part H, 2.3.5.6.1, big-endian */
static const uint8_t ble_sm_alg_dbg_pub_key[2 * BLE_SM_KEY_LEN] = {
	0x20, 0xb0, 0x03, 0xd2, 0xf2, 0x97, 0xbe, 0x2c, 0x5e, 0x2c, 0x83, 0xa7, 0xe9, 0xf9, 0xa5, 0xb9,
	0xef, 0xf4, 0x91, 0x11, 0xac, 0xf4, 0xfd, 0xdb, 0xcc, 0x03, 0x01, 0x48, 0x0e, 0x35, 0x9d, 0xe6,
	0xdc, 0x80, 0x9c, 0x49, 0x65, 0x2a, 0xeb, 0x6d, 0x63, 0x32, 0x9a, 0xbf, 0x5a, 0x52, 0x15, 0x5c,
	0x76, 0x63, 0x45, 0xc2, 0x8f, 0xed, 0x30, 0x24, 0x74, 0x1c, 0x8e, 0xd0, 0x15, 0x89, 0xd2, 0x8b,
};
#endif

static const uint8_t ble_sm_alg_dbg_priv_key[BLE_SM_KEY_LEN] = {
	0x3f, 0x49, 0xf6, 0xd4, 0xa3, 0xc5, 0x5f, 0x38, 0x74, 0xc9, 0xb3, 0xe3, 0xd2, 0x10, 0x3f, 0x50,
	0x4a, 0xff, 0x60, 0x7b, 0xeb, 0x40, 0xb7, 0x99, 0x58, 0x99, 0xb8, 0xa6, 0xcd, 0x3c, 0x1a, 0xbd,
};

int ble_sm_alg_gen_dhkey(const uint8_t *peer_pub_key_x, const uint8_t *peer_pub_key_y,
			 const uint8_t *our_priv_key, uint8_t *out_dhkey)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	uint8_t pk[BLE_SM_PUB_POINT_LEN];
	uint8_t priv[BLE_SM_KEY_LEN];
	uint8_t dh[BLE_SM_KEY_LEN];
	psa_key_id_t key_id = PSA_KEY_ID_NULL;
	psa_status_t status;
	size_t olen = 0;

	pk[0] = 0x04;
	btdm_swap_buf(&pk[1], peer_pub_key_x, BLE_SM_KEY_LEN);
	btdm_swap_buf(&pk[1 + BLE_SM_KEY_LEN], peer_pub_key_y, BLE_SM_KEY_LEN);
	btdm_swap_buf(priv, our_priv_key, BLE_SM_KEY_LEN);

	psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
	psa_set_key_bits(&attr, 256);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDH);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE);

	status = psa_import_key(&attr, priv, sizeof(priv), &key_id);
	psa_reset_key_attributes(&attr);
	mbedtls_platform_zeroize(priv, sizeof(priv));
	if (status != PSA_SUCCESS) {
		return BLE_SM_KEY_ERR;
	}

	status = psa_raw_key_agreement(PSA_ALG_ECDH, key_id, pk, sizeof(pk), dh, sizeof(dh),
				       &olen);
	(void)psa_destroy_key(key_id);
	if ((status != PSA_SUCCESS) || (olen != sizeof(dh))) {
		mbedtls_platform_zeroize(dh, sizeof(dh));
		return BLE_SM_KEY_ERR;
	}

	btdm_swap_buf(out_dhkey, dh, BLE_SM_KEY_LEN);
	mbedtls_platform_zeroize(dh, sizeof(dh));

	return 0;
}

#if !defined(CONFIG_ESP32_BT_LE_SM_SC_DEBUG_KEYS)
static int ble_sm_alg_psa_gen_key_pair(uint8_t *pub_point, uint8_t *priv)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t key_id = PSA_KEY_ID_NULL;
	psa_status_t status;
	size_t olen = 0;
	int rc = BLE_SM_KEY_ERR;

	psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
	psa_set_key_bits(&attr, 256);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDH);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE | PSA_KEY_USAGE_EXPORT);

	status = psa_generate_key(&attr, &key_id);
	psa_reset_key_attributes(&attr);
	if (status != PSA_SUCCESS) {
		return BLE_SM_KEY_ERR;
	}

	status = psa_export_public_key(key_id, pub_point, BLE_SM_PUB_POINT_LEN, &olen);
	if ((status != PSA_SUCCESS) || (olen != BLE_SM_PUB_POINT_LEN)) {
		goto exit;
	}

	status = psa_export_key(key_id, priv, BLE_SM_KEY_LEN, &olen);
	if ((status != PSA_SUCCESS) || (olen != BLE_SM_KEY_LEN)) {
		goto exit;
	}

	rc = 0;

exit:
	(void)psa_destroy_key(key_id);

	return rc;
}
#endif

int ble_sm_alg_gen_key_pair(uint8_t *pub, uint8_t *priv)
{
#if defined(CONFIG_ESP32_BT_LE_SM_SC_DEBUG_KEYS)
	btdm_swap_buf(pub, ble_sm_alg_dbg_pub_key, BLE_SM_KEY_LEN);
	btdm_swap_buf(&pub[BLE_SM_KEY_LEN], &ble_sm_alg_dbg_pub_key[BLE_SM_KEY_LEN],
		      BLE_SM_KEY_LEN);
	btdm_swap_buf(priv, ble_sm_alg_dbg_priv_key, BLE_SM_KEY_LEN);
#else
	uint8_t pk[BLE_SM_PUB_POINT_LEN];

	do {
		if (ble_sm_alg_psa_gen_key_pair(pk, priv) != 0) {
			return BLE_SM_KEY_ERR;
		}
	} while (memcmp(priv, ble_sm_alg_dbg_priv_key, BLE_SM_KEY_LEN) == 0);

	btdm_swap_buf(pub, &pk[1], BLE_SM_KEY_LEN);
	btdm_swap_buf(&pub[BLE_SM_KEY_LEN], &pk[1 + BLE_SM_KEY_LEN], BLE_SM_KEY_LEN);
	btdm_swap_in_place(priv, BLE_SM_KEY_LEN);
#endif

	return 0;
}

int ble_sm_ecc_aes_cmac(const uint8_t *key, const uint8_t *in, size_t len, uint8_t *out)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t key_id = PSA_KEY_ID_NULL;
	psa_status_t status;
	size_t olen = 0;

	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attr, 128);
	psa_set_key_algorithm(&attr, PSA_ALG_CMAC);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE);

	status = psa_import_key(&attr, key, BLE_SM_AES_LEN, &key_id);
	psa_reset_key_attributes(&attr);
	if (status != PSA_SUCCESS) {
		return -1;
	}

	status = psa_mac_compute(key_id, PSA_ALG_CMAC, in, len, out, BLE_SM_AES_LEN, &olen);
	(void)psa_destroy_key(key_id);
	if ((status != PSA_SUCCESS) || (olen != BLE_SM_AES_LEN)) {
		return -1;
	}

	return 0;
}

#else /* CONFIG_ESP32_BT_LE_CRYPTO_STACK_MBEDTLS */

int ble_sm_alg_gen_key_pair(uint8_t *pub, uint8_t *priv)
{
	ARG_UNUSED(pub);
	ARG_UNUSED(priv);

	return BLE_SM_KEY_ERR;
}

int ble_sm_alg_gen_dhkey(const uint8_t *peer_pub_key_x, const uint8_t *peer_pub_key_y,
			 const uint8_t *our_priv_key, uint8_t *out_dhkey)
{
	ARG_UNUSED(peer_pub_key_x);
	ARG_UNUSED(peer_pub_key_y);
	ARG_UNUSED(our_priv_key);
	ARG_UNUSED(out_dhkey);

	return BLE_SM_KEY_ERR;
}

int ble_sm_ecc_aes_cmac(const uint8_t *key, const uint8_t *in, size_t len, uint8_t *out)
{
	ARG_UNUSED(key);
	ARG_UNUSED(in);
	ARG_UNUSED(len);
	ARG_UNUSED(out);

	return -1;
}

#endif /* CONFIG_ESP32_BT_LE_CRYPTO_STACK_MBEDTLS */
