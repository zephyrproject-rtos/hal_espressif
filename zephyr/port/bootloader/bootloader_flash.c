/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Build upstream bootloader_flash.c with the ROM (NON_OS_BUILD) backend for
 * Simple Boot, and MCUboot.
 * Application runtime flash I/O uses esp_flash_* via flash_esp32.c.
 *
 */
#define NON_OS_BUILD 1
#include "../../../components/bootloader_support/bootloader_flash/src/bootloader_flash.c"
