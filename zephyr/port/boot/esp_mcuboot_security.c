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
#include "esp_rom_serial_output.h"

#include "boot/esp_mcuboot_flash_encrypt.h"
#include "boot/esp_mcuboot_secure_boot_hooks.h"
#include "boot/esp_mcuboot_security.h"

static const char *TAG = "mcuboot_security";

/*
 * Shared security context across Secure Boot and Flash Encryption phases.
 * CONFIG_SECURE_BOOT_FLASH_ENC_KEYS_BURN_TOGETHER, planned for a future
 * Secure Boot pull request, can later extend the struct / flow
 * (e.g. combined eFuse batch) using the same prepare/finalize.
 */
static esp_mcuboot_security_ctx_t security_ctx;

static void mcuboot_bootloader_reset(void)
{
	esp_rom_output_tx_wait_idle(0);
	esp_rom_software_reset_system();
	while (true) {
	}
}

esp_err_t esp_mcuboot_security_prepare(void)
{
	esp_err_t err;

	memset(&security_ctx, 0, sizeof(security_ctx));

#ifdef CONFIG_EFUSE_VIRTUAL_KEEP_IN_FLASH
	ESP_EARLY_LOGW(TAG,
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
	}

#ifdef CONFIG_SECURE_ENABLE_SECURE_ROM_DL_MODE
		ESP_EARLY_LOGI(TAG, "Enabling Security download mode...");
		err = esp_efuse_enable_rom_secure_download_mode();
		if (err != ESP_OK) {
			ESP_EARLY_LOGE(TAG, "Could not enable Security download mode...");
			esp_mcuboot_security_cancel();
			return err;
		}
#elif CONFIG_SECURE_DISABLE_ROM_DL_MODE
		ESP_EARLY_LOGI(TAG, "Disable ROM Download mode...");
		err = esp_efuse_disable_rom_download_mode();
		if (err == ESP_ERR_NOT_SUPPORTED) {
			ESP_EARLY_LOGW(TAG, "ROM Download mode cannot be disabled on this chip revision");
		} else if (err != ESP_OK) {
			ESP_EARLY_LOGE(TAG, "Could not disable ROM Download mode...");
			esp_mcuboot_security_cancel();
			return err;
		}
#else
		ESP_EARLY_LOGW(TAG, "UART ROM Download mode kept enabled - SECURITY COMPROMISED");
#endif

	if (!security_ctx.flash_encryption_already_enabled &&
		esp_efuse_is_flash_encryption_enabled()) {
		ESP_EARLY_LOGI(TAG, "Resetting with flash encryption enabled...");
		mcuboot_bootloader_reset();
	}

	return ESP_OK;
}
