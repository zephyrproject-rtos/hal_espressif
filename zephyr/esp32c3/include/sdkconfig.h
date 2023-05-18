/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_MCUBOOT

#define BOOTLOADER_BUILD 1
#define NDEBUG 1

#define CONFIG_BOOTLOADER_FLASH_XMC_SUPPORT 1
#define CONFIG_SPI_FLASH_ROM_DRIVER_PATCH 1
#define CONFIG_EFUSE_VIRTUAL_OFFSET 0x250000
#define CONFIG_EFUSE_VIRTUAL_SIZE 0x2000
#define CONFIG_EFUSE_MAX_BLK_LEN 256

#endif /* CONFIG_MCUBOOT*/

#define CONFIG_IDF_FIRMWARE_CHIP_ID 0x0005
#define CONFIG_LOG_TIMESTAMP_SOURCE_RTOS 1
#define CONFIG_BOOTLOADER_LOG_LEVEL 3

#define CONFIG_BT_CTRL_ESP32C3 1
#define CONFIG_BT_SOC_SUPPORT_5_0 1
#define BTDM_CTRL_HCI_MODE_VHCI 1

#define CONFIG_ESPTOOLPY_FLASHMODE_DIO 1

#undef asm
#define asm __asm__

#undef typeof
#define typeof __typeof__
