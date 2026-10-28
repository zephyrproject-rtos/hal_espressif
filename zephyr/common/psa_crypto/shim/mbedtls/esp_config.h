/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Stand-in for ESP-IDF's mbedtls/esp_config.h, seen only by the ESP-IDF SHA
 * cores (components/mbedtls/port/psa_driver/esp_sha/core/). It replaces the
 * whole ESP-IDF Mbed TLS configuration with what the cores need: the macro
 * that selects the accelerator path of the SHA-1 core (the SHA-256 core has no
 * software path) and the one that exposes the cores' prototypes. The driver's
 * wrapper includes it first too, so it is built like the cores.
 *
 * <psa/crypto.h> comes first, so the TF-PSA-Crypto configuration is already
 * adjusted when the macro appears: it reaches the cores' own #ifdef only and
 * the PSA structures keep the layout every other translation unit sees.
 */

#ifndef ESP_PSA_SHIM_ESP_CONFIG_H_
#define ESP_PSA_SHIM_ESP_CONFIG_H_

#include <psa/crypto.h>

/*
 * From the CONFIG_MBEDTLS_HARDWARE_SHA block of ESP-IDF's esp_config.h, the
 * part the cores and their headers test. The built-in implementation is NOT
 * removed (no #undef MBEDTLS_PSA_BUILTIN_*): it stays the fallback.
 */
#define ESP_SHA_DRIVER_ENABLED
#define MBEDTLS_PSA_ACCEL_ALG_SHA_1

#endif /* ESP_PSA_SHIM_ESP_CONFIG_H_ */
