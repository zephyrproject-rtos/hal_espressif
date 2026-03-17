/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize PSRAM interface/hardware.
 * Initializes the PSRAM hardware and load the XIP segments or maps the PSRAM memory
 *
 * @return
 *        - ESP_OK:                On success
 *        - ESP_FAIL:              PSRAM isn't initialized successfully, potential reason would be: wrong VDDSDIO, invalid chip ID, etc.
 *        - ESP_ERR_INVALID_STATE: PSRAM is initialized already
 */
esp_err_t esp_psram_init(void);

/**
 * @brief If PSRAM has been initialized
 *
 * @return
 *          - true:  PSRAM has been initialized successfully
 *          - false: PSRAM hasn't been initialized or initialized failed
 */
bool esp_psram_is_initialized(void);

/**
 * @brief Get the available size of the attached PSRAM chip
 *
 * @return Size in bytes, or 0 if PSRAM isn't successfully initialized
 */
size_t esp_psram_get_size(void);

/**
 * @brief Get the mapped PSRAM region address and size (Zephyr only)
 *
 * Returns the virtual address and size of the PSRAM region mapped by
 * esp_psram_init(). Used by Zephyr's shared_multi_heap to register
 * the PSRAM heap at the correct runtime-determined address.
 *
 * @param[out] out_vaddr  Mapped virtual start address
 * @param[out] out_size   Mapped region size in bytes
 *
 * @return
 *        - ESP_OK:                On success
 *        - ESP_ERR_INVALID_STATE: PSRAM not initialized
 */
esp_err_t esp_psram_get_mapped_region(intptr_t *out_vaddr, size_t *out_size);

#ifdef __cplusplus
}
#endif
