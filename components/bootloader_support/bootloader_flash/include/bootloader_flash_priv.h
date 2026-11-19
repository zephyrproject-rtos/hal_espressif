/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include "sdkconfig.h"
#include "bootloader_flash.h"
#include "bootloader_flash_override.h"

/*
 * SPI commands
 * These are deprecated, use the ones in spi_flash_defs.h instead.
 */
#define CMD_RDID       0x9F
#define CMD_WRSR       0x01
#define CMD_WRSR2      0x31 /* Not all SPI flash uses this command */
#define CMD_WRSR3      0x11 /* Not all SPI flash uses this command */
#define CMD_WREN       0x06
#define CMD_WRENVSR    0x50 /* Flash write enable for volatile SR bits */
#define CMD_WRDI       0x04
#define CMD_RDSR       0x05
#define CMD_RDSR2      0x35 /* Not all SPI flash uses this command */
#define CMD_RDSR3      0x15 /* Not all SPI flash uses this command */
#define CMD_OTPEN      0x3A /* Enable OTP mode, not all SPI flash uses this command */
#define CMD_RDSFDP     0x5A /* Read the SFDP of the flash */
#define CMD_RESUME     0x7A /* Resume command to clear flash suspend bit */
#define CMD_RESETEN    0x66
#define CMD_RESET      0x99
#define CMD_FASTRD_QIO_4B   0xEC
#define CMD_FASTRD_QUAD_4B  0x6C
#define CMD_FASTRD_DIO_4B   0xBC
#define CMD_FASTRD_DUAL_4B  0x3C
#define CMD_FASTRD_4B       0x0C
#define CMD_SLOWRD_4B       0x13

#ifdef __ZEPHYR__
#include "esp_private/bootloader_flash_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Zephyr-specific flash ROM function wrappers.
 * These are implemented in zephyr/port/bootloader/bootloader_flash.c
 */
esp_err_t esp_rom_flash_read(size_t src_addr, void *dest, size_t size, bool allow_decrypt);
esp_err_t esp_rom_flash_write(size_t dest_addr, void *src, size_t size, bool write_encrypted);
esp_err_t esp_rom_flash_erase_sector(size_t sector);
esp_err_t esp_rom_flash_erase_range(uint32_t start_addr, uint32_t size);
const void *esp_rom_flash_mmap(uint32_t src_paddr, uint32_t size);
void esp_rom_flash_unmmap(const void *mapping);

#ifdef __cplusplus
}
#endif
#endif /* __ZEPHYR__ */
