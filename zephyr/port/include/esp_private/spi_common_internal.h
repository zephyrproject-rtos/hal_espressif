/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Minimal stub for Zephyr to satisfy esp_psram_impl_quad.c include.
 * The full ESP-IDF SPI driver is not used in Zephyr.
 * When CONFIG_SPIRAM_OCCUPY_NO_HOST is defined, spicommon_periph_claim()
 * is not called, so we only need the type definitions.
 */

#pragma once

#include "hal/spi_types.h"
#include "soc/spi_reg.h"
#include "soc/spi_pins.h"
