/*
 * Copyright (c) 2025 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Compatibility stubs for Wi-Fi blobs.
 */

#include <zephyr/kernel.h>

/*
 * Regulatory domain data required by the Wi-Fi blob. Provided as weak NULL
 * fallbacks so a soc that compiles the real esp_wifi_regulatory.c tables
 * overrides them with valid data.
 */
const void *__attribute__((weak)) regdomain_table;
const void *__attribute__((weak)) regulatory_data;

void __attribute__((weak)) pm_beacon_offset_funcs_empty_init(void)
{
}

