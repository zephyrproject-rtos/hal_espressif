/*
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_ESP_SIMPLE_BOOT
#define CONFIG_BOOTLOADER_FLASH_XMC_SUPPORT 1
#endif /* !CONFIG_BOOTLOADER_MCUBOOT */

#define CONFIG_IDF_FIRMWARE_CHIP_ID 0x000D
#define CONFIG_LOG_TIMESTAMP_SOURCE_RTOS 1
#define CONFIG_BOOTLOADER_LOG_LEVEL 3

#define CONFIG_ESPTOOLPY_FLASHMODE_DIO 1

#undef asm
#define asm __asm__

#undef typeof
#define typeof __typeof__
