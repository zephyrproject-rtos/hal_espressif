/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Build upstream bootloader_flash.c with the ROM (NON_OS_BUILD) backend for
 * MCUboot.
 * Simple Boot and MCUboot-chained apps compile the OS backend
 * (esp_flash_* / spi_flash_mmap).
 * Early Simple Boot still needs ROM-backend reads before flash stack is
 * ready; those are provided as esp_rom_flash_read() below.
 */
#ifdef CONFIG_MCUBOOT
#define NON_OS_BUILD 1
#endif

#include "../../../components/bootloader_support/bootloader_flash/src/bootloader_flash.c"

#if defined(CONFIG_ESP_SIMPLE_BOOT)

#include <assert.h>
#include <inttypes.h>

#include "esp_rom_spiflash.h"
#include "hal/mmu_hal.h"
#include "hal/mmu_ll.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/cache.h"
#endif

#if CONFIG_IDF_TARGET_ESP32
/* Use first 50 blocks in MMU for mapping; 50th block for decrypted reads */
#define MMU_BLOCK0_VADDR  SOC_DROM_LOW
#define MMU_TOTAL_SIZE    (0x320000)
#define MMU_BLOCK50_VADDR (MMU_BLOCK0_VADDR + MMU_TOTAL_SIZE)
#define FLASH_READ_VADDR  MMU_BLOCK50_VADDR
#else
#define MMU_BLOCK0_VADDR  SOC_DROM_LOW
#if CONFIG_IDF_TARGET_ESP32S2
#define MMU_TOTAL_SIZE    (SOC_DRAM0_CACHE_ADDRESS_HIGH - SOC_DRAM0_CACHE_ADDRESS_LOW)
#else
#define MMU_TOTAL_SIZE    (SOC_DRAM_FLASH_ADDRESS_HIGH - SOC_DRAM_FLASH_ADDRESS_LOW)
#endif
#define MMU_END_VADDR     (MMU_BLOCK0_VADDR + MMU_TOTAL_SIZE)
#define FLASH_READ_VADDR  (MMU_END_VADDR - CONFIG_MMU_PAGE_SIZE)
#endif

/* Current MMU window used by decrypted esp_rom_flash_read() */
static uint32_t current_read_mapping = UINT32_MAX;

static esp_err_t esp_rom_spi_to_esp_err(esp_rom_spiflash_result_t r)
{
	switch (r) {
	case ESP_ROM_SPIFLASH_RESULT_OK:
		return ESP_OK;
	case ESP_ROM_SPIFLASH_RESULT_ERR:
		return ESP_ERR_FLASH_OP_FAIL;
	case ESP_ROM_SPIFLASH_RESULT_TIMEOUT:
		return ESP_ERR_FLASH_OP_TIMEOUT;
	default:
		return ESP_FAIL;
	}
}

static esp_err_t esp_rom_flash_read_no_decrypt(size_t src_addr, void *dest, size_t size)
{
#if CONFIG_IDF_TARGET_ESP32
	Cache_Read_Disable(0);
	Cache_Flush(0);
#else
	cache_hal_disable(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_ALL);
#endif

	esp_rom_spiflash_result_t r = esp_rom_spiflash_read(src_addr, dest, size);

#if CONFIG_IDF_TARGET_ESP32
	Cache_Read_Enable(0);
#else
	cache_hal_enable(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_ALL);
#endif

	return esp_rom_spi_to_esp_err(r);
}

static esp_err_t esp_rom_flash_read_allow_decrypt(size_t src_addr, void *dest, size_t size)
{
	uint32_t *dest_words = (uint32_t *)dest;

	for (size_t word = 0; word < size / 4; word++) {
		uint32_t word_src = src_addr + word * 4;
		uint32_t map_at = word_src & MMU_FLASH_MASK;
		uint32_t *map_ptr;

		if (map_at != current_read_mapping) {
#if CONFIG_IDF_TARGET_ESP32
			Cache_Read_Disable(0);
			Cache_Flush(0);
#else
			cache_hal_disable(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_ALL);
#endif

			ESP_EARLY_LOGD(TAG, "mmu set block paddr=0x%08" PRIx32
				       " (was 0x%08" PRIx32 ")",
				       map_at, current_read_mapping);
#if CONFIG_IDF_TARGET_ESP32
			int e __attribute__((unused)) =
				cache_flash_mmu_set(0, 0, FLASH_READ_VADDR, map_at, 64, 1);
			assert(e == 0);
#else
			uint32_t actual_mapped_len = 0;

			mmu_hal_map_region(0, MMU_TARGET_FLASH0, FLASH_READ_VADDR, map_at,
					   SPI_FLASH_MMU_PAGE_SIZE - 1, &actual_mapped_len);
#endif
			current_read_mapping = map_at;

#if CONFIG_IDF_TARGET_ESP32
			Cache_Read_Enable(0);
#else
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
			cache_ll_invalidate_addr(CACHE_LL_LEVEL_ALL, CACHE_TYPE_ALL,
						 CACHE_LL_ID_ALL, FLASH_READ_VADDR,
						 actual_mapped_len);
#endif
			cache_hal_enable(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_ALL);
#endif
		}
		map_ptr = (uint32_t *)(FLASH_READ_VADDR + (word_src - map_at));
		dest_words[word] = *map_ptr;
	}
	current_read_mapping = UINT32_MAX;
	return ESP_OK;
}

esp_err_t esp_rom_flash_read(size_t src_addr, void *dest, size_t size, bool allow_decrypt)
{
	if ((src_addr & 3) || (size & 3) || ((intptr_t)dest & 3)) {
		ESP_EARLY_LOGE(TAG,
			       "esp_rom_flash_read src_addr 0x%x, size 0x%x or dest 0x%x not "
			       "4-byte aligned",
			       src_addr, size, (intptr_t)dest);
		return ESP_FAIL;
	}

	if (allow_decrypt) {
		return esp_rom_flash_read_allow_decrypt(src_addr, dest, size);
	}

	return esp_rom_flash_read_no_decrypt(src_addr, dest, size);
}

#endif /* CONFIG_ESP_SIMPLE_BOOT */
