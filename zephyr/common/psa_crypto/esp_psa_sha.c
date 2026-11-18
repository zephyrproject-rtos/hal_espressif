/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Transparent PSA Crypto hash driver for the ESP32 SHA accelerator.
 *
 * The hashing itself is ESP-IDF's: the SHA-1 and SHA-256/224 cores in
 * components/mbedtls/port/psa_driver/esp_sha/core/ (buffering, length,
 * padding, and the engine calls through esp_sha_port.c). This file is the PSA
 * side of them, kept apart from ESP-IDF's own wrapper
 * (psa_driver/esp_sha/psa_crypto_driver_esp_sha.c) because, on Zephyr, that
 * one allocates a context per operation with heap_caps_malloc() (k_malloc()
 * here) and releases it with libc free(). The context lives in the operation
 * instead, and the output size is checked before anything is written: the PSA
 * core passes hash_size to the driver unchecked.
 */

/* First, as in the ESP-IDF cores: the configuration they are built with. */
#include "mbedtls/esp_config.h"

#include <stdint.h>
#include <string.h>

#include <zephyr/toolchain.h>

#include <psa/crypto.h>
#include <mbedtls/platform_util.h>

#include "esp_psa_sha.h"
#include "hal/sha_types.h"
#include "psa_crypto_driver_esp_sha.h"
#include "psa_crypto_driver_esp_sha1.h"
#include "psa_crypto_driver_esp_sha256.h"
#include "soc/soc_caps.h"

#include "esp_sha_port.h"

enum esp_psa_sha_kind {
	ESP_PSA_SHA_KIND_NONE = 0,
	ESP_PSA_SHA_KIND_SHA1,
	ESP_PSA_SHA_KIND_SHA224,
	ESP_PSA_SHA_KIND_SHA256,
};

#define OP_STORAGE_SIZE sizeof(((esp_psa_sha_hash_operation_t *)0)->storage)

BUILD_ASSERT(sizeof(esp_sha1_context) <= OP_STORAGE_SIZE,
	     "esp_psa_sha_hash_operation_t storage too small for esp_sha1_context");
BUILD_ASSERT(sizeof(esp_sha256_context) <= OP_STORAGE_SIZE,
	     "esp_psa_sha_hash_operation_t storage too small for esp_sha256_context");
BUILD_ASSERT(__alignof__(esp_sha1_context) <= __alignof__(uint32_t) &&
		     __alignof__(esp_sha256_context) <= __alignof__(uint32_t),
	     "ESP-IDF SHA context needs more than word alignment");

/*
 * Algorithms the engine does and the PSA configuration asks for; everything
 * else is declined, as the built-in implementation declines what it was not
 * built with.
 */
static enum esp_psa_sha_kind kind_of(psa_algorithm_t alg)
{
	switch (alg) {
#if defined(PSA_WANT_ALG_SHA_1)
	case PSA_ALG_SHA_1:
		return ESP_PSA_SHA_KIND_SHA1;
#endif
#if SOC_SHA_SUPPORT_SHA224 && defined(PSA_WANT_ALG_SHA_224)
	case PSA_ALG_SHA_224:
		return ESP_PSA_SHA_KIND_SHA224;
#endif
#if defined(PSA_WANT_ALG_SHA_256)
	case PSA_ALG_SHA_256:
		return ESP_PSA_SHA_KIND_SHA256;
#endif
	default:
		return ESP_PSA_SHA_KIND_NONE;
	}
}

static size_t digest_length(enum esp_psa_sha_kind kind)
{
	switch (kind) {
	case ESP_PSA_SHA_KIND_SHA1:
		return 20;
	case ESP_PSA_SHA_KIND_SHA224:
		return 28;
	case ESP_PSA_SHA_KIND_SHA256:
		return 32;
	default:
		return 0;
	}
}

static esp_sha1_context *sha1_ctx(esp_psa_sha_hash_operation_t *op)
{
	return (esp_sha1_context *)op->storage;
}

static esp_sha256_context *sha256_ctx(esp_psa_sha_hash_operation_t *op)
{
	return (esp_sha256_context *)op->storage;
}

static void wipe(esp_psa_sha_hash_operation_t *op)
{
	mbedtls_platform_zeroize(op, sizeof(*op));
}

psa_status_t esp_psa_sha_hash_setup(esp_psa_sha_hash_operation_t *op, psa_algorithm_t alg)
{
	enum esp_psa_sha_kind kind = kind_of(alg);
	psa_status_t status;

	if (op == NULL) {
		return PSA_ERROR_INVALID_ARGUMENT;
	}
	if (kind == ESP_PSA_SHA_KIND_NONE || esp_sha_port_ready() != 0) {
		return PSA_ERROR_NOT_SUPPORTED;
	}

	wipe(op);
	if (kind == ESP_PSA_SHA_KIND_SHA1) {
		status = (esp_sha1_starts(sha1_ctx(op)) == 0) ? PSA_SUCCESS
							      : PSA_ERROR_HARDWARE_FAILURE;
	} else {
		/* the header calls the parameter is224; the core stores it as the engine mode */
		status = esp_sha256_starts(sha256_ctx(op), (kind == ESP_PSA_SHA_KIND_SHA224)
								   ? SHA2_224
								   : SHA2_256);
	}
	if (status != PSA_SUCCESS) {
		wipe(op);
		return status;
	}
	op->kind = (uint8_t)kind;

	return PSA_SUCCESS;
}

psa_status_t esp_psa_sha_hash_update(esp_psa_sha_hash_operation_t *op, const uint8_t *input,
				     size_t input_length)
{
	if (op == NULL || op->kind == ESP_PSA_SHA_KIND_NONE) {
		return PSA_ERROR_BAD_STATE;
	}
	if (input_length == 0) {
		return PSA_SUCCESS;
	}
	if (input == NULL) {
		return PSA_ERROR_INVALID_ARGUMENT;
	}

	if (op->kind == ESP_PSA_SHA_KIND_SHA1) {
		return esp_sha1_driver_update(sha1_ctx(op), input, input_length);
	}
	return esp_sha256_driver_update(sha256_ctx(op), input, input_length);
}

psa_status_t esp_psa_sha_hash_finish(esp_psa_sha_hash_operation_t *op, uint8_t *hash,
				     size_t hash_size, size_t *hash_length)
{
	psa_status_t status;
	size_t length;

	if (op == NULL || op->kind == ESP_PSA_SHA_KIND_NONE) {
		return PSA_ERROR_BAD_STATE;
	}
	if (hash == NULL || hash_length == NULL) {
		return PSA_ERROR_INVALID_ARGUMENT;
	}
	length = digest_length((enum esp_psa_sha_kind)op->kind);
	if (hash_size < length) {
		/* nothing written: the cores would write a full digest first */
		return PSA_ERROR_BUFFER_TOO_SMALL;
	}

	if (op->kind == ESP_PSA_SHA_KIND_SHA1) {
		status = esp_sha1_driver_finish(sha1_ctx(op), hash, hash_size, &length);
	} else {
		status = esp_sha256_driver_finish(sha256_ctx(op), hash, hash_size, &length,
						  (op->kind == ESP_PSA_SHA_KIND_SHA224)
							  ? ESP_SHA_OPERATION_TYPE_SHA224
							  : ESP_SHA_OPERATION_TYPE_SHA256);
	}
	wipe(op);
	if (status == PSA_SUCCESS) {
		*hash_length = length;
	}

	return status;
}

psa_status_t esp_psa_sha_hash_abort(esp_psa_sha_hash_operation_t *op)
{
	if (op != NULL) {
		wipe(op);
	}
	return PSA_SUCCESS;
}

psa_status_t esp_psa_sha_hash_clone(const esp_psa_sha_hash_operation_t *source,
				    esp_psa_sha_hash_operation_t *target)
{
	if (source == NULL || target == NULL) {
		return PSA_ERROR_INVALID_ARGUMENT;
	}
	if (source->kind == ESP_PSA_SHA_KIND_NONE) {
		return PSA_ERROR_BAD_STATE;
	}
	/* The digest lives in the context, not in the engine: a copy is a fork. */
	memcpy(target, source, sizeof(*target));

	return PSA_SUCCESS;
}

psa_status_t esp_psa_sha_hash_compute(psa_algorithm_t alg, const uint8_t *input,
				      size_t input_length, uint8_t *hash, size_t hash_size,
				      size_t *hash_length)
{
	enum esp_psa_sha_kind kind = kind_of(alg);
	esp_psa_sha_hash_operation_t op;
	psa_status_t status;

	if (kind == ESP_PSA_SHA_KIND_NONE) {
		return PSA_ERROR_NOT_SUPPORTED;
	}
	if (hash == NULL || hash_length == NULL) {
		return PSA_ERROR_INVALID_ARGUMENT;
	}
	if (hash_size < digest_length(kind)) {
		return PSA_ERROR_BUFFER_TOO_SMALL;
	}

	status = esp_psa_sha_hash_setup(&op, alg);
	if (status != PSA_SUCCESS) {
		return status;
	}
	status = esp_psa_sha_hash_update(&op, input, input_length);
	if (status == PSA_SUCCESS) {
		status = esp_psa_sha_hash_finish(&op, hash, hash_size, hash_length);
	}
	wipe(&op);

	return status;
}
