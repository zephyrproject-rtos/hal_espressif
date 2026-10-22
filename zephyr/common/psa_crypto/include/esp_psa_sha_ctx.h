/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Operation context of the ESP32 SHA PSA driver, as it sits in the hash union
 * of the PSA dispatch (psa_driver_hash_context_t). Opaque on purpose: this
 * header is reached from <psa/crypto.h> in every user of PSA, C++ included, so
 * it pulls in nothing from the HAL. esp_psa_sha.c keeps the ESP-IDF SHA context
 * (esp_sha1_context / esp_sha256_context) in `storage` and checks its size at
 * build time.
 */

#ifndef ESP_PSA_SHA_CTX_H_
#define ESP_PSA_SHA_CTX_H_

#include <stdint.h>

/* Large enough for esp_sha256_context (116 bytes on 32- and 64-bit hosts). */
#define ESP_PSA_SHA_CTX_WORDS 30

typedef struct {
	uint8_t kind; /* enum esp_psa_sha_kind, private to esp_psa_sha.c */
	uint32_t storage[ESP_PSA_SHA_CTX_WORDS];
} esp_psa_sha_hash_operation_t;

#endif /* ESP_PSA_SHA_CTX_H_ */
