/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Stub header for esp_crypto_lock.h
 * ESP32-S2 crypto DMA locking is not required in Zephyr context
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

static inline void esp_crypto_dma_lock_acquire(void) {}
static inline void esp_crypto_dma_lock_release(void) {}
static inline void esp_crypto_sha_aes_lock_acquire(void) {}
static inline void esp_crypto_sha_aes_lock_release(void) {}
static inline void esp_crypto_mpi_lock_acquire(void) {}
static inline void esp_crypto_mpi_lock_release(void) {}

#ifdef __cplusplus
}
#endif
