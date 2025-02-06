/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "flash_init.h"

#include <stdbool.h>
#include <bootloader_flash_priv.h>
#include <esp_flash_internal.h>
#include <esp_private/mspi_timing_tuning.h>
#include <esp_private/esp_mmu_map_private.h>
#include <hal/efuse_ll.h>
#include <hal/efuse_hal.h>
#include "esp_private/spi_flash_os.h"
#include "esp_log.h"
#if CONFIG_SOC_SERIES_ESP32S3
#include <esp32s3/opi_flash_private.h>
#endif

static const char *TAG = "flash_init";

bool flash_is_octal_mode_enabled(void)
{
#if SOC_SPI_MEM_SUPPORT_OPI_MODE
	return efuse_ll_get_flash_type();
#else
	return false;
#endif
}

int spi_flash_init_chip_state(void)
{
#if SOC_SPI_MEM_SUPPORT_OPI_MODE
	if (flash_is_octal_mode_enabled()) {
		return esp_opiflash_init(rom_spiflash_legacy_data->chip.device_id);
	}
#endif
#if CONFIG_SPI_FLASH_HPM_ON
	/* Currently, only esp32s3 allows high performance mode. */
	return spi_flash_enable_high_performance_mode();
#else
	return 0;
#endif /* CONFIG_SOC_SERIES_ESP32S3 */
}

void esp_flash_config(void)
{
	esp_err_t ret;

	spi_flash_init_chip_state();

	esp_flash_app_init();

	ret = esp_flash_init_default_chip();
	if (ret != ESP_OK) {
		ESP_EARLY_LOGE(TAG, "Failed to init flash chip: %d ", ret);
		abort();
	}

	esp_mspi_pin_init();

#if SOC_MEMSPI_SRC_FREQ_120M
	mspi_timing_flash_tuning();
#endif

	esp_mmu_map_init();
}
