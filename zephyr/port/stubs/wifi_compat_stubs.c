/*
 * Copyright (c) 2025 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Compatibility stubs for Wi-Fi blobs.
 */

#include <zephyr/kernel.h>

/* Regulatory domain data - required by blob */
const void *regdomain_table = NULL;
const void *regulatory_data = NULL;

void __attribute__((weak)) pm_beacon_offset_funcs_empty_init(void)
{
}

