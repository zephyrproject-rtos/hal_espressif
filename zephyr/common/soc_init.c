/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <errno.h>
#include "esp_log.h"
#include "bootloader_flash_priv.h"
#include <zephyr/storage/flash_map.h>
#include "esp_app_format.h"
#include "bootloader_common.h"
#include "hal/efuse_ll.h"
#include "hal/efuse_hal.h"
#include "soc/chip_revision.h"
#include <hal/wdt_hal.h>
#include "soc_init.h"

#define IS_MAX_REV_SET(max_chip_rev_full)                                                          \
	(((max_chip_rev_full) != 65535) && ((max_chip_rev_full) != 0))

static const char *TAG = "soc_init";

extern esp_image_header_t WORD_ALIGNED_ATTR bootloader_image_hdr;

void print_banner(void)
{
#ifdef CONFIG_MCUBOOT
	ESP_EARLY_LOGI(TAG, "MCUboot 2nd stage bootloader");
#elif CONFIG_BOOTLOADER_MCUBOOT
	ESP_EARLY_LOGI(TAG, "MCUboot Application image");
#else /* CONFIG_ESP_SIMPLE_BOOT */
	ESP_EARLY_LOGI(TAG, "ESP Simple boot");
#endif
	ESP_EARLY_LOGI(TAG, "compile time " __DATE__ " " __TIME__);
#ifndef CONFIG_SMP
#if (SOC_CPU_CORES_NUM > 1)
	ESP_EARLY_LOGW(TAG, "Unicore bootloader");
#endif
#else
	ESP_EARLY_LOGI(TAG, "Multicore bootloader");
#endif
}

int read_bootloader_header(void)
{
	/* load bootloader image header */
	if (esp_rom_flash_read(FIXED_PARTITION_OFFSET(boot_partition), &bootloader_image_hdr,
				      sizeof(esp_image_header_t), true) != 0) {
		ESP_EARLY_LOGE(TAG, "failed to load bootloader image header!");
		return -EIO;
	}

	return 0;
}

static int check_chip_validity(const esp_image_header_t *img_hdr, esp_image_type type)
{
	int err = 0;
	esp_chip_id_t chip_id = CONFIG_IDF_FIRMWARE_CHIP_ID;

	if (chip_id != img_hdr->chip_id) {
		ESP_EARLY_LOGE(TAG, "mismatch chip ID, expected %d, found %d", chip_id,
			       img_hdr->chip_id);
		err = -EIO;
	} else {
		unsigned int revision = efuse_hal_chip_revision();
		unsigned int major_rev = revision / 100;
		unsigned int minor_rev = revision % 100;
		unsigned int min_rev = img_hdr->min_chip_rev_full;

		if (type == ESP_IMAGE_BOOTLOADER || type == ESP_IMAGE_APPLICATION) {
			if (!ESP_CHIP_REV_ABOVE(revision, min_rev)) {
				ESP_EARLY_LOGE(
					TAG,
					"Image requires chip rev >= v%d.%d, but chip is v%d.%d",
					min_rev / 100, min_rev % 100, major_rev, minor_rev);
				err = ESP_FAIL;
			}
		}
		if (type == ESP_IMAGE_APPLICATION) {
			unsigned int max_rev = img_hdr->max_chip_rev_full;

			if ((IS_MAX_REV_SET(max_rev) && (revision > max_rev) &&
			     !efuse_ll_get_disable_wafer_version_major())) {
				ESP_EARLY_LOGE(
					TAG,
					"Image requires chip rev <= v%d.%d, but chip is v%d.%d",
					max_rev / 100, max_rev % 100, major_rev, minor_rev);
				err = ESP_FAIL;
			}
		}
	}

	return err;
}

void config_wdt(void)
{
	/*
	 * At this point, the flashboot protection of RWDT and MWDT0 will have been
	 * automatically enabled. We can disable flashboot protection as it's not
	 * needed anymore. If configured to do so, we also initialize the RWDT to
	 * protect the remainder of the bootloader process.
	 */
	wdt_hal_context_t rwdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();

	wdt_hal_write_protect_disable(&rwdt_ctx);
	wdt_hal_set_flashboot_en(&rwdt_ctx, false);
	wdt_hal_write_protect_enable(&rwdt_ctx);

	/* Disable MWDT0 flashboot protection. But only after we've enabled the RWDT first so that
	 * there's not gap in WDT protection.
	 */
	wdt_hal_context_t mwdt_ctx = {.inst = WDT_MWDT0, .mwdt_dev = &TIMERG0};

	wdt_hal_write_protect_disable(&mwdt_ctx);
	wdt_hal_set_flashboot_en(&mwdt_ctx, false);
	wdt_hal_write_protect_enable(&mwdt_ctx);
}

int check_bootloader_validity(void)
{
	unsigned int revision = efuse_hal_chip_revision();
	unsigned int major = revision / 100;
	unsigned int minor = revision % 100;

	ESP_EARLY_LOGI(TAG, "chip revision: v%d.%d", major, minor);

#ifndef CONFIG_MCUBOOT
#if defined(CONFIG_SOC_SERIES_ESP32)
	if (major < 3) {
		ESP_EARLY_LOGE(TAG,
			       "You are using ESP32 chip revision (%d) that is unsupported. While "
			       "it may work, it could cause unexpected behavior or issues.",
			       major);
		ESP_EARLY_LOGE(TAG,
			       "Proceeding with this ESP32 chip revision is not recommended unless "
			       "you fully understand the potential risk and limitations.");
#if !defined(CONFIG_ESP32_USE_UNSUPPORTED_REVISION)
		ESP_EARLY_LOGE(
			TAG,
			"If you choose to continue, please enable "
			"'CONFIG_ESP32_USE_UNSUPPORTED_REVISION=y' in your project configuration.");
		config_wdt(); // disable watchdog to avoid reset
		abort();
#endif /* !CONFIG_ESP32_USE_UNSUPPORTED_REVISION */
	}
#endif /* CONFIG_SOC_SERIES_ESP32 */
#endif /* CONFIG_MCUBOOT */

	/* compare with the one set in bootloader image header */
	if (check_chip_validity(&bootloader_image_hdr, ESP_IMAGE_BOOTLOADER) != 0) {
		return ESP_FAIL;
	}
	return 0;
}
