/*
 * SPDX-FileCopyrightText: 2016-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lib_printf.c
 *
 * This file contains library-specific printf functions
 * used by WiFi libraries in the `lib` directory.
 * These function are used to catch any output which gets printed
 * by libraries, and redirect it to ESP_LOG macros.
 *
 * Eventually WiFi libraries will use ESP_LOG functions internally
 * and these definitions will be removed.
 */

#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"

#define VPRINTF_STACK_BUFFER_SIZE 80

int IRAM_ATTR phy_printf(const char* format, ...)
{
    va_list arg;
    va_start(arg, format);
    esp_rom_printf("PHY: ");
    esp_rom_vprintf(format, arg);
    va_end(arg);
    return 0;
}


int IRAM_ATTR rtc_printf(const char* format, ...)
{
    va_list arg;
    va_start(arg, format);
    esp_rom_printf("RTC: ");
    esp_rom_vprintf(format, arg);
    va_end(arg);
    return 0;
}
