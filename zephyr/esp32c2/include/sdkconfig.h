/*
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define CONFIG_IDF_FIRMWARE_CHIP_ID 0x000C
#define CONFIG_LOG_TIMESTAMP_SOURCE_RTOS 1
#define CONFIG_BOOTLOADER_LOG_LEVEL 3

#define CONFIG_IDF_TARGET_ESP32C2 1

#undef asm
#define asm __asm__

#undef typeof
#define typeof __typeof__
