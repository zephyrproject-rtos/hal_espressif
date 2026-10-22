/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Transparent PSA Crypto hash driver for the ESP32 SHA accelerator.
 *
 * Entry points of the TF-PSA-Crypto driver interface, called from the dispatch
 * in dispatch/psa_crypto_driver_wrappers.h. Algorithms: SHA-1, SHA-224 (where
 * the SoC has it) and SHA-256; anything else is PSA_ERROR_NOT_SUPPORTED and
 * falls through to the built-in implementation.
 */

#ifndef ESP_PSA_SHA_H_
#define ESP_PSA_SHA_H_

#include <stddef.h>
#include <stdint.h>

#include <psa/crypto.h>

#include "esp_psa_sha_ctx.h"

#ifdef __cplusplus
extern "C" {
#endif

psa_status_t esp_psa_sha_hash_setup(esp_psa_sha_hash_operation_t *op, psa_algorithm_t alg);

psa_status_t esp_psa_sha_hash_update(esp_psa_sha_hash_operation_t *op, const uint8_t *input,
				     size_t input_length);

psa_status_t esp_psa_sha_hash_finish(esp_psa_sha_hash_operation_t *op, uint8_t *hash,
				     size_t hash_size, size_t *hash_length);

psa_status_t esp_psa_sha_hash_abort(esp_psa_sha_hash_operation_t *op);

psa_status_t esp_psa_sha_hash_clone(const esp_psa_sha_hash_operation_t *source,
				    esp_psa_sha_hash_operation_t *target);

psa_status_t esp_psa_sha_hash_compute(psa_algorithm_t alg, const uint8_t *input,
				      size_t input_length, uint8_t *hash, size_t hash_size,
				      size_t *hash_length);

#ifdef __cplusplus
}
#endif

#endif /* ESP_PSA_SHA_H_ */
