/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * CONFIG_TF_PSA_CRYPTO_DRIVER_CONFIG_FILE for the ESP32 PSA drivers: included
 * at the end of the TF-PSA-Crypto configuration, so the dispatch in
 * dispatch/ and every user of <psa/crypto.h> see the same set of drivers.
 */

#ifndef ESP_PSA_DRIVER_CONFIG_H_
#define ESP_PSA_DRIVER_CONFIG_H_

#define ESP_PSA_SHA_DRIVER_ENABLED

#endif /* ESP_PSA_DRIVER_CONFIG_H_ */
