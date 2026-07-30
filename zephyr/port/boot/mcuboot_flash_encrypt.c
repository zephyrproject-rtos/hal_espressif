/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <strings.h>

#include <zephyr/devicetree.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>

#include "bootloader_flash_priv.h"
#include "bootloader_random.h"
#include "esp_app_format.h"
#include "esp_flash_encrypt.h"
#include "esp_secure_boot.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_log.h"
#include "hal/wdt_hal.h"
#include "hal/efuse_hal.h"
#include "soc/soc_caps.h"

#ifdef CONFIG_SOC_EFUSE_CONSISTS_OF_ONE_KEY_BLOCK
#include "soc/sensitive_reg.h"
#endif

#include "boot/esp_mcuboot_image.h"
#include "boot/mcuboot_flash_encrypt.h"
#include "boot/mcuboot_secure_boot_hooks.h"

#if CONFIG_IDF_TARGET_ESP32
#define CRYPT_CNT ESP_EFUSE_FLASH_CRYPT_CNT
#define WR_DIS_CRYPT_CNT ESP_EFUSE_WR_DIS_FLASH_CRYPT_CNT
#else
#define CRYPT_CNT ESP_EFUSE_SPI_BOOT_CRYPT_CNT
#define WR_DIS_CRYPT_CNT ESP_EFUSE_WR_DIS_SPI_BOOT_CRYPT_CNT
#endif

#define FLASH_ENC_CNT_MAX (CRYPT_CNT[0]->bit_count)

#define BOOTLOADER_ADDR   DT_REG_ADDR(DT_NODELABEL(boot_partition))
#define BOOTLOADER_SIZE   DT_REG_SIZE(DT_NODELABEL(boot_partition))

#define MCUBOOT_HEADER_SIZE 0x20

static const char *TAG = "flash_encrypt";

static struct {
	bool flash_encryption_was_enabled;
	bool combined_batch_open;
} security_state;

static esp_err_t encrypt_bootloader(void);
static size_t get_flash_encrypt_cnt_value(void);

static size_t get_flash_encrypt_cnt_value(void)
{
	size_t flash_crypt_cnt = 0;

	esp_efuse_read_field_cnt(CRYPT_CNT, &flash_crypt_cnt);
	return flash_crypt_cnt;
}

bool esp_flash_encrypt_initialized_once(void)
{
	return get_flash_encrypt_cnt_value() != 0;
}

bool esp_flash_encrypt_is_write_protected(bool print_error)
{
	if (esp_efuse_read_field_bit(WR_DIS_CRYPT_CNT)) {
		if (print_error) {
			ESP_LOGE(TAG,
				 "Flash Encryption cannot be enabled (CRYPT_CNT (%d) is write protected)",
				 get_flash_encrypt_cnt_value());
		}
		return true;
	}
	return false;
}

bool esp_flash_encrypt_state(void)
{
	size_t flash_crypt_cnt = get_flash_encrypt_cnt_value();
	bool flash_crypt_wr_dis = esp_flash_encrypt_is_write_protected(false);

	ESP_LOGV(TAG, "CRYPT_CNT %d, write protection %d", flash_crypt_cnt, flash_crypt_wr_dis);

	if (flash_crypt_cnt % 2 == 1) {
		int left = (FLASH_ENC_CNT_MAX - flash_crypt_cnt) / 2;

		if (flash_crypt_wr_dis) {
			left = 0;
		}
		ESP_LOGI(TAG, "flash encryption is enabled (%d plaintext flashes left)", left);
		return true;
	}
	return false;
}

static esp_err_t check_and_generate_encryption_keys(void)
{
	size_t key_size = 32;

#ifdef CONFIG_IDF_TARGET_ESP32
	enum { BLOCKS_NEEDED = 1 };
	esp_efuse_purpose_t purposes[BLOCKS_NEEDED] = {
		ESP_EFUSE_KEY_PURPOSE_FLASH_ENCRYPTION,
	};
	esp_efuse_coding_scheme_t coding_scheme =
		esp_efuse_get_coding_scheme(EFUSE_BLK_ENCRYPT_FLASH);

	if (coding_scheme != EFUSE_CODING_SCHEME_NONE &&
	    coding_scheme != EFUSE_CODING_SCHEME_3_4) {
		ESP_LOGE(TAG, "Unknown/unsupported CODING_SCHEME value 0x%x", coding_scheme);
		return ESP_ERR_NOT_SUPPORTED;
	}
	if (coding_scheme == EFUSE_CODING_SCHEME_3_4) {
		key_size = 24;
	}
#else
#ifdef CONFIG_SECURE_FLASH_ENCRYPTION_AES256
	enum { BLOCKS_NEEDED = 2 };
	esp_efuse_purpose_t purposes[BLOCKS_NEEDED] = {
		ESP_EFUSE_KEY_PURPOSE_XTS_AES_256_KEY_1,
		ESP_EFUSE_KEY_PURPOSE_XTS_AES_256_KEY_2,
	};

	if (esp_efuse_find_purpose(ESP_EFUSE_KEY_PURPOSE_XTS_AES_128_KEY, NULL)) {
		ESP_LOGE(TAG,
			 "XTS_AES_128_KEY is already in use, XTS_AES_256_KEY_1/2 can not be used");
		return ESP_ERR_INVALID_STATE;
	}
#else
#ifdef CONFIG_SECURE_FLASH_ENCRYPTION_AES128_DERIVED
	enum { BLOCKS_NEEDED = 1 };
	esp_efuse_purpose_t purposes[BLOCKS_NEEDED] = {
		ESP_EFUSE_KEY_PURPOSE_XTS_AES_128_KEY_DERIVED_FROM_128_EFUSE_BITS,
	};

	key_size = 16;
#else
	enum { BLOCKS_NEEDED = 1 };
	esp_efuse_purpose_t purposes[BLOCKS_NEEDED] = {
		ESP_EFUSE_KEY_PURPOSE_XTS_AES_128_KEY,
	};
#endif
#endif
#endif

	esp_efuse_block_t blocks[BLOCKS_NEEDED] = {[0 ... BLOCKS_NEEDED - 1] = EFUSE_BLK_KEY_MAX};
	bool has_key = true;

	for (unsigned i = 0; i < BLOCKS_NEEDED; i++) {
		bool tmp_has_key = esp_efuse_find_purpose(purposes[i], &blocks[i]);

		if (tmp_has_key) {
			tmp_has_key &= !esp_efuse_key_block_unused(blocks[i]);
		}
		if (i == 1 && tmp_has_key != has_key) {
			ESP_LOGE(TAG, "Invalid efuse key blocks: Both AES-256 key blocks must be set.");
			return ESP_ERR_INVALID_STATE;
		}
		has_key &= tmp_has_key;
	}

	if (!has_key) {
		uint8_t keys[BLOCKS_NEEDED][32] = { 0 };

		ESP_LOGI(TAG, "Generating new flash encryption key...");
		for (unsigned i = 0; i < BLOCKS_NEEDED; ++i) {
			bootloader_fill_random(keys[i], key_size);
		}

		esp_err_t err = esp_efuse_write_keys(purposes, keys, BLOCKS_NEEDED);

		if (err != ESP_OK) {
			if (err == ESP_ERR_NOT_ENOUGH_UNUSED_KEY_BLOCKS) {
				ESP_LOGE(TAG, "Not enough free efuse key blocks (need %d) to continue",
					 BLOCKS_NEEDED);
			} else {
				ESP_LOGE(TAG,
					 "Failed to write efuse block with purpose (err=0x%x). Can't continue.",
					 err);
			}
			return err;
		}
	} else {
		for (unsigned i = 0; i < BLOCKS_NEEDED; i++) {
			if (!esp_efuse_get_key_dis_write(blocks[i]) ||
			    !esp_efuse_get_key_dis_read(blocks[i]) ||
			    !esp_efuse_get_keypurpose_dis_write(blocks[i])) {
				ESP_LOGE(TAG,
					 "Invalid key state, check read&write protection for key and keypurpose(if exists)");
				return ESP_ERR_INVALID_STATE;
			}
		}
		ESP_LOGI(TAG, "Using pre-loaded flash encryption key in efuse");
	}
	return ESP_OK;
}

esp_err_t esp_flash_encrypt_init(void)
{
	if (esp_efuse_is_flash_encryption_enabled() || esp_flash_encrypt_initialized_once()) {
		return ESP_OK;
	}

	esp_efuse_batch_write_begin();

	esp_err_t err = check_and_generate_encryption_keys();

	if (err != ESP_OK) {
		esp_efuse_batch_write_cancel();
		return err;
	}

	err = esp_flash_encryption_enable_secure_features();
	if (err != ESP_OK) {
		esp_efuse_batch_write_cancel();
		return err;
	}

	err = esp_efuse_batch_write_commit();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error programming security eFuses (err=0x%x).", err);
		return err;
	}

	return ESP_OK;
}

static esp_err_t encrypt_bootloader(void)
{
	esp_image_header_t img_hdr;
	esp_err_t err;

	err = bootloader_flash_read(BOOTLOADER_ADDR, &img_hdr, sizeof(img_hdr), false);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to read bootloader image header");
		return err;
	}

	if (img_hdr.magic != ESP_IMAGE_HEADER_MAGIC) {
		ESP_LOGW(TAG, "No valid bootloader was found");
		return ESP_ERR_NOT_FOUND;
	}

	ESP_LOGI(TAG, "Encrypting bootloader...");
	err = esp_flash_encrypt_region(BOOTLOADER_ADDR, BOOTLOADER_SIZE);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to encrypt bootloader in place: 0x%x", err);
		return err;
	}

	ESP_LOGI(TAG, "Bootloader encrypted successfully");
	return ESP_OK;
}

static esp_err_t encrypt_other_partition(uint32_t addr, size_t size, const char *label)
{
	esp_err_t err;

	/* boot_partition is handled specially above. */
	if (addr == BOOTLOADER_ADDR) {
		return ESP_OK;
	}

	if (size == 0) {
		return ESP_OK;
	}

	ESP_LOGI(TAG, "Encrypting partition '%s' at 0x%x (%u bytes)...", label, addr,
		 (unsigned int)size);
	err = esp_flash_encrypt_region(addr, size);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to encrypt partition '%s' in place: 0x%x", label, err);
		return err;
	}
	ESP_LOGI(TAG, "Partition '%s' encrypted successfully", label);
	return ESP_OK;
}

#define ENCRYPT_DT_PARTITION(node_id)                                                          \
	{                                                                                      \
		esp_err_t _err = encrypt_other_partition(DT_REG_ADDR(node_id),                 \
							 DT_REG_SIZE(node_id),                 \
							 DT_PROP(node_id, label));             \
		if (_err != ESP_OK) {                                                          \
			return _err;                                                           \
		}                                                                              \
	}

esp_err_t esp_flash_encrypt_contents(void)
{
	esp_err_t err;

#ifdef CONFIG_SOC_EFUSE_CONSISTS_OF_ONE_KEY_BLOCK
	REG_WRITE(SENSITIVE_XTS_AES_KEY_UPDATE_REG, 1);
#endif

	err = encrypt_bootloader();
	if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
		return err;
	}

	/* Encrypt remaining DT flash partitions (slots, scratch, storage, …). */
	DT_FOREACH_STATUS_OKAY(zephyr_mapped_partition, ENCRYPT_DT_PARTITION);

	ESP_LOGI(TAG, "Flash encryption completed");
	return ESP_OK;
}

esp_err_t esp_flash_encrypt_enable(void)
{
	esp_err_t err = ESP_OK;

	if (!esp_efuse_is_flash_encryption_enabled()) {
		if (esp_flash_encrypt_is_write_protected(true)) {
			return ESP_FAIL;
		}

		size_t flash_crypt_cnt = get_flash_encrypt_cnt_value();

#ifdef CONFIG_SECURE_FLASH_ENCRYPTION_MODE_RELEASE
		ESP_LOGI(TAG, "Setting CRYPT_CNT for permanent encryption");
		size_t new_flash_crypt_cnt = FLASH_ENC_CNT_MAX - flash_crypt_cnt;
#else
		size_t new_flash_crypt_cnt = 1;
#endif
		ESP_LOGD(TAG, "CRYPT_CNT %d -> %d", flash_crypt_cnt, new_flash_crypt_cnt);
		err = esp_efuse_write_field_cnt(CRYPT_CNT, new_flash_crypt_cnt);

#if defined(CONFIG_SECURE_FLASH_ENCRYPTION_MODE_RELEASE) && \
	defined(CONFIG_SOC_FLASH_ENCRYPTION_XTS_AES_128_DERIVED)
		esp_efuse_write_field_bit(WR_DIS_CRYPT_CNT);
#endif
	}

#ifdef CONFIG_EFUSE_VIRTUAL
	ESP_LOGW(TAG, "Flash encryption not really completed. Must disable virtual efuses");
#endif

	return err;
}

esp_err_t esp_flash_encrypt_region(uint32_t src_addr, size_t data_length)
{
	esp_err_t err;
	uint32_t buf[FLASH_SECTOR_SIZE / sizeof(uint32_t)];

	if (src_addr % FLASH_SECTOR_SIZE != 0) {
		ESP_LOGE(TAG, "esp_flash_encrypt_region bad src_addr 0x%x", src_addr);
		return ESP_FAIL;
	}

	wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();

	for (size_t i = 0; i < data_length; i += FLASH_SECTOR_SIZE) {
		wdt_hal_write_protect_disable(&rtc_wdt_ctx);
		wdt_hal_feed(&rtc_wdt_ctx);
		wdt_hal_write_protect_enable(&rtc_wdt_ctx);
		uint32_t sec_start = i + src_addr;

		err = bootloader_flash_read(sec_start, buf, FLASH_SECTOR_SIZE, false);
		if (err != ESP_OK) {
			goto flash_failed;
		}
		err = bootloader_flash_erase_sector(sec_start / FLASH_SECTOR_SIZE);
		if (err != ESP_OK) {
			goto flash_failed;
		}
		err = bootloader_flash_write(sec_start, buf, FLASH_SECTOR_SIZE, true);
		if (err != ESP_OK) {
			goto flash_failed;
		}
	}
	return ESP_OK;

flash_failed:
	ESP_LOGE(TAG, "flash operation failed: 0x%x", err);
	return err;
}

esp_err_t esp_mcuboot_security_prepare(void)
{
	esp_err_t err;

	memset(&security_state, 0, sizeof(security_state));

#ifdef CONFIG_EFUSE_VIRTUAL_KEEP_IN_FLASH
	ESP_LOGW(TAG,
		 "eFuse virtual mode is enabled. If Secure boot or Flash encryption is enabled "
		 "then it does not provide any security. FOR TESTING ONLY!");
	esp_efuse_init_virtual_mode_in_flash(CONFIG_EFUSE_VIRTUAL_OFFSET,
					     CONFIG_EFUSE_VIRTUAL_SIZE);
#endif

#ifdef CONFIG_SECURE_BOOT_FLASH_ENC_KEYS_BURN_TOGETHER
	if (esp_secure_boot_enabled() ^ esp_flash_encrypt_initialized_once()) {
		ESP_LOGE(TAG,
			 "Secure Boot and Flash Encryption cannot be enabled separately, only together "
			 "(their keys go into one eFuse key block)");
		return ESP_FAIL;
	}

	if (!esp_secure_boot_enabled() || !esp_efuse_is_flash_encryption_enabled()) {
		esp_efuse_batch_write_begin();
		security_state.combined_batch_open = true;
	}
#endif

	err = esp_mcuboot_secure_boot_prepare();
	if (err != ESP_OK) {
		esp_mcuboot_security_cancel();
		return err;
	}

	security_state.flash_encryption_was_enabled = esp_flash_encrypt_state();

	return ESP_OK;
}

void esp_mcuboot_security_cancel(void)
{
	esp_mcuboot_secure_boot_cancel();
	esp_efuse_batch_write_cancel();
	security_state.combined_batch_open = false;
}

esp_err_t esp_mcuboot_security_finalize(void)
{
	esp_err_t err;

	err = esp_mcuboot_secure_boot_finalize();
	if (err != ESP_OK) {
		return err;
	}

	if (!security_state.flash_encryption_was_enabled) {
#ifdef CONFIG_SECURE_FLASH_REQUIRE_ALREADY_ENABLED
		ESP_LOGE(TAG,
			 "flash encryption is not enabled, and SECURE_FLASH_REQUIRE_ALREADY_ENABLED "
			 "is set, refusing to boot.");
		return ESP_ERR_INVALID_STATE;
#endif

		if (esp_flash_encrypt_is_write_protected(true)) {
			return ESP_FAIL;
		}

		ESP_LOGI(TAG, "Checking flash encryption...");

		err = esp_flash_encrypt_init();
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Initialization of Flash Encryption key failed (%d)", err);
			return err;
		}

		err = esp_flash_encrypt_contents();
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Encryption flash contents failed (%d)", err);
			return err;
		}

		err = esp_flash_encrypt_enable();
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Enabling of Flash encryption failed (%d)", err);
			return err;
		}
	}

#ifdef CONFIG_SECURE_BOOT_FLASH_ENC_KEYS_BURN_TOGETHER
	if (security_state.combined_batch_open &&
	    (!esp_secure_boot_enabled() || !security_state.flash_encryption_was_enabled)) {
		err = esp_efuse_batch_write_commit();
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Error programming eFuses (err=0x%x).", err);
			return err;
		}
		security_state.combined_batch_open = false;
	}
#endif

	if (!security_state.flash_encryption_was_enabled && esp_efuse_is_flash_encryption_enabled()) {
		ESP_LOGI(TAG, "Resetting with flash encryption enabled...");
		return ESP_ERR_FLASH_ENCRYPT_RESET_NEEDED;
	}

	return ESP_OK;
}
