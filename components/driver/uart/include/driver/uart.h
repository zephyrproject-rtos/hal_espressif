/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "soc/soc_caps.h"
#include "hal/uart_types.h"

// Valid UART port number
#define UART_NUM_0             (0) /*!< UART port 0 */
#define UART_NUM_1             (1) /*!< UART port 1 */
#if SOC_UART_NUM > 2
#define UART_NUM_2             (2) /*!< UART port 2 */
#endif
#define UART_NUM_MAX           (SOC_UART_NUM) /*!< UART port max */

/* @brief When calling `uart_set_pin`, instead of GPIO number, `UART_PIN_NO_CHANGE`
 *        can be provided to keep the currently allocated pin.
 */
#define UART_PIN_NO_CHANGE      (-1)

#define UART_FIFO_LEN           SOC_UART_FIFO_LEN       ///< Length of the UART HW FIFO
#define UART_BITRATE_MAX        SOC_UART_BITRATE_MAX    ///< Maximum configurable bitrate

#ifdef __cplusplus
}
#endif
