/*
 * SPDX-FileCopyrightText: 2019-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SUPPLICANT_OPT_H
#define _SUPPLICANT_OPT_H

#include "sdkconfig.h"

#if CONFIG_ESP32_WIFI_DEBUG_PRINT
#define DEBUG_PRINT
#define ELOOP_DEBUG
#endif

#if CONFIG_ESP_WIFI_SCAN_CACHE
#define SCAN_CACHE_SUPPORTED
#endif

#endif /* _SUPPLICANT_OPT_H */
