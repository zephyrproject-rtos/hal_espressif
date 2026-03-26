/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file sleep_sys_periph.h
 *
 * This file contains declarations of digital peripheral retention related functions in light sleep mode.
 */

/**
 * @brief Whether to allow the TOP power domain to be powered off.
 *
 * In light sleep mode, only when the system can provide enough memory
 * for digital peripheral retention, the TOP power domain can be powered off.
 *
 * @return True to allow power off
 */
bool peripheral_domain_pd_allowed(void);

/**
 * @brief Initialize sleep retention for system peripherals.
 *
 * Registers and allocates REGDMA retention entries for system
 * peripherals (Interrupt Matrix, HP System, IO MUX, SPI MEM,
 * SysTimer, etc.) to allow the TOP power domain to be powered
 * off during light sleep.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NO_MEM not enough memory for peripheral retention
 *      - ESP_ERR_INVALID_ARG if either of the arguments is out of range
 */
esp_err_t sleep_sys_periph_startup_init(void);

#ifdef __cplusplus
}
#endif
