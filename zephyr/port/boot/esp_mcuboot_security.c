/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "esp_efuse.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "boot/esp_mcuboot_flash_encrypt.h"
#include "boot/esp_mcuboot_secure_boot_hooks.h"
#include "boot/esp_mcuboot_security.h"

static const char *TAG = "mcuboot_security";

/*
 * Shared security context across Secure Boot and Flash Encryption phases.
 * CONFIG_SECURE_BOOT_FLASH_ENC_KEYS_BURN_TOGETHER can later extend the
 * struct / flow (e.g. combined eFuse batch) using the same prepare/finalize.
 */
static esp_mcuboot_security_ctx_t security_ctx;

static void mcuboot_bootloader_reset(void)
{
	esp_rom_delay_us(1000);
	esp_rom_software_reset_system();
	while (true) {
	}
}

esp_err_t esp_mcuboot_security_prepare(void)
{
	esp_err_t err;

	memset(&security_ctx, 0, sizeof(security_ctx));

#ifdef CONFIG_EFUSE_VIRTUAL_KEEP_IN_FLASH
	ESP_LOGW(TAG,
		 "eFuse virtual mode is enabled. If Secure boot or Flash encryption is enabled "
		 "then it does not provide any security. FOR TESTING ONLY!");
	esp_efuse_init_virtual_mode_in_flash(CONFIG_EFUSE_VIRTUAL_OFFSET,
					     CONFIG_EFUSE_VIRTUAL_SIZE);
#endif

	err = esp_mcuboot_secure_boot_prepare(&security_ctx);
	if (err != ESP_OK) {
		esp_mcuboot_security_cancel();
		return err;
	}

	err = esp_mcuboot_flash_encryption_prepare(&security_ctx);
	if (err != ESP_OK) {
		esp_mcuboot_security_cancel();
		return err;
	}

	return ESP_OK;
}

void esp_mcuboot_security_cancel(void)
{
	esp_mcuboot_secure_boot_cancel(&security_ctx);
	esp_mcuboot_flash_encryption_cancel(&security_ctx);
}

esp_err_t esp_mcuboot_security_finalize(void)
{
	esp_err_t err;

	err = esp_mcuboot_secure_boot_finalize(&security_ctx);
	if (err != ESP_OK) {
		esp_mcuboot_security_cancel();
		return err;
	}

	if (!security_ctx.flash_encryption_already_enabled) {
		err = esp_mcuboot_flash_encryption_finalize(&security_ctx);
		if (err != ESP_OK) {
			esp_mcuboot_security_cancel();
			return err;
		}

		if (esp_efuse_is_flash_encryption_enabled()) {
			ESP_LOGI(TAG, "Resetting with flash encryption enabled...");
			mcuboot_bootloader_reset();
		}
	}

	return ESP_OK;
}
