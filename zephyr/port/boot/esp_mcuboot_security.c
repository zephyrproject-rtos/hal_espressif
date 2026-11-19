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
#include "esp_secure_boot.h"

#ifdef CONFIG_SECURE_FLASH_ENC_ENABLED
#include "esp_flash_encrypt.h"
#include "boot/esp_mcuboot_flash_encrypt.h"
#endif
#include "boot/esp_mcuboot_secure_boot_hooks.h"
#include "boot/esp_mcuboot_security.h"

static const char *TAG = "mcuboot_security";

/* Shared security context across Secure Boot and Flash Encryption phases. */
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

#ifdef CONFIG_SECURE_BOOT_FLASH_ENC_KEYS_BURN_TOGETHER
	/*
	 * ESP32-C2 has one key block. Secure Boot and Flash Encryption must be
	 * burned together, and the first boot refuses a half-enabled device.
	 */
	if (esp_secure_boot_enabled() ^ esp_flash_encrypt_initialized_once()) {
		ESP_LOGE(TAG,
			 "Secure Boot and Flash Encryption cannot be enabled separately, "
			 "only together (their keys go into one eFuse key block)");
		return ESP_FAIL;
	}
	if (!esp_secure_boot_enabled() || !esp_flash_encrypt_initialized_once()) {
		err = esp_efuse_batch_write_begin();
		if (err != ESP_OK) {
			return err;
		}
		security_ctx.combined_batch_open = true;
	}
#endif

	err = esp_mcuboot_secure_boot_prepare(&security_ctx);
	if (err != ESP_OK) {
		esp_mcuboot_security_cancel();
		return err;
	}

#ifdef CONFIG_SECURE_FLASH_ENC_ENABLED
	err = esp_mcuboot_flash_encryption_prepare(&security_ctx);
	if (err != ESP_OK) {
		esp_mcuboot_security_cancel();
		return err;
	}
#else
	security_ctx.flash_encryption_already_enabled = true;
#endif

	return ESP_OK;
}

void esp_mcuboot_security_cancel(void)
{
	esp_mcuboot_secure_boot_cancel(&security_ctx);
#ifdef CONFIG_SECURE_FLASH_ENC_ENABLED
	esp_mcuboot_flash_encryption_cancel(&security_ctx);
#endif
	if (security_ctx.combined_batch_open) {
		esp_efuse_batch_write_cancel();
		security_ctx.combined_batch_open = false;
	}
}

esp_err_t esp_mcuboot_security_finalize(void)
{
	esp_err_t err;

	err = esp_mcuboot_secure_boot_finalize(&security_ctx);
	if (err != ESP_OK) {
		esp_mcuboot_security_cancel();
		return err;
	}

#ifdef CONFIG_SECURE_FLASH_ENC_ENABLED
	if (!security_ctx.flash_encryption_already_enabled) {
		err = esp_mcuboot_flash_encryption_finalize(&security_ctx);
		if (err != ESP_OK) {
			esp_mcuboot_security_cancel();
			return err;
		}
	}
#endif

#ifdef CONFIG_SECURE_ENABLE_SECURE_ROM_DL_MODE
		ESP_LOGI(TAG, "Enabling Security download mode...");
		err = esp_efuse_enable_rom_secure_download_mode();
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Could not enable Security download mode...");
			esp_mcuboot_security_cancel();
			return err;
		}
#elif CONFIG_SECURE_DISABLE_ROM_DL_MODE
		ESP_LOGI(TAG, "Disable ROM Download mode...");
		err = esp_efuse_disable_rom_download_mode();
		if (err == ESP_ERR_NOT_SUPPORTED) {
			ESP_LOGW(TAG, "ROM Download mode cannot be disabled on this chip revision");
		} else if (err != ESP_OK) {
			ESP_LOGE(TAG, "Could not disable ROM Download mode...");
			esp_mcuboot_security_cancel();
			return err;
		}
#else
		ESP_LOGW(TAG, "UART ROM Download mode kept enabled - SECURITY COMPROMISED");
#endif

	if (security_ctx.combined_batch_open) {
		err = esp_efuse_batch_write_commit();
		security_ctx.combined_batch_open = false;
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Error programming eFuses (err=0x%x).", err);
			esp_mcuboot_security_cancel();
			return err;
		}
#ifdef CONFIG_SECURE_BOOT_FLASH_ENC_KEYS_BURN_TOGETHER
		if (!esp_secure_boot_enabled()) {
			ESP_LOGE(TAG, "Secure boot was not enabled after the combined eFuse burn");
			return ESP_FAIL;
		}
		ESP_LOGI(TAG, "Secure boot permanently enabled");
#endif
	}

	if (!security_ctx.flash_encryption_already_enabled &&
		esp_efuse_is_flash_encryption_enabled()) {
		ESP_LOGI(TAG, "Resetting with flash encryption enabled...");
		mcuboot_bootloader_reset();
	}

	return ESP_OK;
}
