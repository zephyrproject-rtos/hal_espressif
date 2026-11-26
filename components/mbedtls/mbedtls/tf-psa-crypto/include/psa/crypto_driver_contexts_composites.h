/**
 * \file crypto_driver_contexts_composites.h
 *
 * \brief Declaration of context structures for use with the PSA driver wrapper
 *  interface. This file contains the context structures for 'composite'
 *  operations, i.e. those operations which need to make use of other operations
 *  from the primitives (crypto_driver_contexts_primitives.h)
 *
 * \warning This file will be auto-generated in the future.
 *
 * \note This file may not be included directly. Applications must
 * include psa/crypto.h.
 *
 * \note This header and its content are not part of the Mbed TLS API and
 * applications must not depend on it. Its main purpose is to define the
 * multi-part state objects of the PSA drivers included in the cryptographic
 * library. The definitions of these objects are then used by crypto_struct.h
 * to define the implementation-defined types of PSA multi-part state objects.
 */
/*  Copyright The Mbed TLS Contributors
 *  SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later
 */

#ifndef PSA_CRYPTO_DRIVER_CONTEXTS_COMPOSITES_H
#define PSA_CRYPTO_DRIVER_CONTEXTS_COMPOSITES_H

#include "psa/crypto_driver_common.h"

/* Include the context structure definitions for the Mbed TLS software drivers */
#include "mbedtls/private/crypto_builtin_composites.h"

/* Include the context structure definitions for those drivers that were
 * declared during the autogeneration process. */

#if defined(ESP_AES_DRIVER_ENABLED)
#include "psa_crypto_driver_esp_aes_contexts.h"
#endif /* ESP_AES_DRIVER_ENABLED */

#if defined(ESP_ECDSA_DRIVER_ENABLED)
#include "psa_crypto_driver_esp_ecdsa_contexts.h"
#endif /* ESP_ECDSA_DRIVER_ENABLED */

#if defined(ESP_RSA_DS_DRIVER_ENABLED)
#include "psa_crypto_driver_esp_rsa_ds_contexts.h"
#endif /* ESP_RSA_DS_DRIVER_ENABLED */

#if defined(ESP_CMAC_DRIVER_ENABLED)
#include "psa_crypto_driver_esp_cmac_contexts.h"
#endif

#if defined(ESP_HMAC_TRANSPARENT_DRIVER_ENABLED)
#include "psa_crypto_driver_esp_hmac_transparent_contexts.h"
#endif

#if defined(ESP_HMAC_OPAQUE_DRIVER_ENABLED)
#include "psa_crypto_driver_esp_hmac_opaque_contexts.h"
#endif

#if defined(MBEDTLS_TEST_LIBTESTDRIVER1)
#if defined(TF_PSA_CRYPTO_TEST_LIBTESTDRIVER1)
#include "mbedtls/private/libtestdriver1-crypto_builtin_composites.h"
#else
#include <libtestdriver1/tf-psa-crypto/include/psa/crypto.h>
#endif
#endif

#if defined(PSA_CRYPTO_DRIVER_TEST)
#if defined(MBEDTLS_TEST_LIBTESTDRIVER1) && \
    defined(LIBTESTDRIVER1_MBEDTLS_PSA_BUILTIN_MAC)
typedef libtestdriver1_mbedtls_psa_mac_operation_t
    mbedtls_transparent_test_driver_mac_operation_t;
typedef libtestdriver1_mbedtls_psa_mac_operation_t
    mbedtls_opaque_test_driver_mac_operation_t;

#define MBEDTLS_TRANSPARENT_TEST_DRIVER_MAC_OPERATION_INIT \
    LIBTESTDRIVER1_MBEDTLS_PSA_MAC_OPERATION_INIT
#define MBEDTLS_OPAQUE_TEST_DRIVER_MAC_OPERATION_INIT \
    LIBTESTDRIVER1_MBEDTLS_PSA_MAC_OPERATION_INIT

#else
typedef mbedtls_psa_mac_operation_t
    mbedtls_transparent_test_driver_mac_operation_t;
typedef mbedtls_psa_mac_operation_t
    mbedtls_opaque_test_driver_mac_operation_t;

#define MBEDTLS_TRANSPARENT_TEST_DRIVER_MAC_OPERATION_INIT \
    MBEDTLS_PSA_MAC_OPERATION_INIT
#define MBEDTLS_OPAQUE_TEST_DRIVER_MAC_OPERATION_INIT \
    MBEDTLS_PSA_MAC_OPERATION_INIT

#endif /* MBEDTLS_TEST_LIBTESTDRIVER1 && LIBTESTDRIVER1_MBEDTLS_PSA_BUILTIN_MAC */

#if defined(MBEDTLS_TEST_LIBTESTDRIVER1) && \
    defined(LIBTESTDRIVER1_MBEDTLS_PSA_BUILTIN_AEAD)
typedef libtestdriver1_mbedtls_psa_aead_operation_t
    mbedtls_transparent_test_driver_aead_operation_t;

#define MBEDTLS_TRANSPARENT_TEST_DRIVER_AEAD_OPERATION_INIT \
    LIBTESTDRIVER1_MBEDTLS_PSA_AEAD_OPERATION_INIT
#else
typedef mbedtls_psa_aead_operation_t
    mbedtls_transparent_test_driver_aead_operation_t;

#define MBEDTLS_TRANSPARENT_TEST_DRIVER_AEAD_OPERATION_INIT \
    MBEDTLS_PSA_AEAD_OPERATION_INIT

#endif /* MBEDTLS_TEST_LIBTESTDRIVER1 && LIBTESTDRIVER1_MBEDTLS_PSA_BUILTIN_AEAD */

#if defined(MBEDTLS_TEST_LIBTESTDRIVER1) && \
    defined(LIBTESTDRIVER1_MBEDTLS_PSA_BUILTIN_PAKE)

typedef libtestdriver1_mbedtls_psa_pake_operation_t
    mbedtls_transparent_test_driver_pake_operation_t;
typedef libtestdriver1_mbedtls_psa_pake_operation_t
    mbedtls_opaque_test_driver_pake_operation_t;

#define MBEDTLS_TRANSPARENT_TEST_DRIVER_PAKE_OPERATION_INIT \
    LIBTESTDRIVER1_MBEDTLS_PSA_PAKE_OPERATION_INIT
#define MBEDTLS_OPAQUE_TEST_DRIVER_PAKE_OPERATION_INIT \
    LIBTESTDRIVER1_MBEDTLS_PSA_PAKE_OPERATION_INIT

#else
typedef mbedtls_psa_pake_operation_t
    mbedtls_transparent_test_driver_pake_operation_t;
typedef mbedtls_psa_pake_operation_t
    mbedtls_opaque_test_driver_pake_operation_t;

#define MBEDTLS_TRANSPARENT_TEST_DRIVER_PAKE_OPERATION_INIT \
    MBEDTLS_PSA_PAKE_OPERATION_INIT
#define MBEDTLS_OPAQUE_TEST_DRIVER_PAKE_OPERATION_INIT \
    MBEDTLS_PSA_PAKE_OPERATION_INIT

#endif /* MBEDTLS_TEST_LIBTESTDRIVER1 && LIBTESTDRIVER1_MBEDTLS_PSA_BUILTIN_PAKE */

#endif /* PSA_CRYPTO_DRIVER_TEST */

/* Define the context to be used for an operation that is executed through the
 * PSA Driver wrapper layer as the union of all possible driver's contexts.
 *
 * The union members are the driver's context structures, and the member names
 * are formatted as `'drivername'_ctx`. This allows for procedural generation
 * of both this file and the content of psa_crypto_driver_wrappers.h */

typedef union {
    unsigned dummy; /* Make sure this union is always non-empty */
    mbedtls_psa_mac_operation_t mbedtls_ctx;
#if defined(PSA_CRYPTO_DRIVER_TEST)
    mbedtls_transparent_test_driver_mac_operation_t transparent_test_driver_ctx;
    mbedtls_opaque_test_driver_mac_operation_t opaque_test_driver_ctx;
#endif
#if defined(ESP_CMAC_DRIVER_ENABLED)
    esp_cmac_operation_t esp_cmac_ctx;
#endif
#if defined(ESP_HMAC_TRANSPARENT_DRIVER_ENABLED)
    esp_hmac_transparent_operation_t esp_hmac_transparent_ctx;
#endif
#if defined(ESP_HMAC_OPAQUE_DRIVER_ENABLED)
    esp_hmac_opaque_operation_t esp_hmac_opaque_ctx;
#endif
} psa_driver_mac_context_t;

typedef union {
    unsigned dummy; /* Make sure this union is always non-empty */
    mbedtls_psa_aead_operation_t mbedtls_ctx;
#if defined(PSA_CRYPTO_DRIVER_TEST)
    mbedtls_transparent_test_driver_aead_operation_t transparent_test_driver_ctx;
#endif
#if defined(ESP_AES_DRIVER_ENABLED)
    esp_aes_gcm_operation_t esp_aes_gcm_ctx;
#endif
} psa_driver_aead_context_t;

typedef union {
    unsigned dummy; /* Make sure this union is always non-empty */
    mbedtls_psa_sign_hash_interruptible_operation_t mbedtls_ctx;
#if defined(ESP_ECDSA_DRIVER_ENABLED) && defined(ESP_ECDSA_SIGN_DRIVER_ENABLED)
    esp_ecdsa_opaque_sign_hash_operation_t esp_ecdsa_opaque_sign_hash_ctx;
#endif
#if defined(ESP_RSA_DS_DRIVER_ENABLED)
    esp_rsa_ds_opaque_sign_hash_operation_t esp_rsa_ds_opaque_sign_hash_ctx;
#endif
} psa_driver_sign_hash_interruptible_context_t;

typedef union {
    unsigned dummy; /* Make sure this union is always non-empty */
    mbedtls_psa_verify_hash_interruptible_operation_t mbedtls_ctx;
#if defined(ESP_ECDSA_DRIVER_ENABLED) && defined(ESP_ECDSA_VERIFY_DRIVER_ENABLED)
    esp_ecdsa_transparent_verify_hash_operation_t esp_ecdsa_transparent_verify_hash_ctx;
#endif
} psa_driver_verify_hash_interruptible_context_t;

typedef union {
    unsigned dummy; /* Make sure this union is always non-empty */
    mbedtls_psa_pake_operation_t mbedtls_ctx;
#if defined(PSA_CRYPTO_DRIVER_TEST)
    mbedtls_transparent_test_driver_pake_operation_t transparent_test_driver_ctx;
    mbedtls_opaque_test_driver_pake_operation_t opaque_test_driver_ctx;
#endif
} psa_driver_pake_context_t;

#endif /* PSA_CRYPTO_DRIVER_CONTEXTS_COMPOSITES_H */
/* End of automatically generated file. */
