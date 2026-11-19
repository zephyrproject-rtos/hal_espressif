/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Secure Boot v2 for the Zephyr MCUboot image.
 *
 * Port of check_and_generate_secure_boot_keys() from the native MCUboot
 * Espressif port. ROM verifies this bootloader. Application images stay
 * imgtool-signed and are not Secure Boot v2 signed.
 */

#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#include "bootloader_flash_priv.h"
#include "bootloader_sha.h"
#include "esp_app_format.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_image_format.h"
#include "esp_log.h"
#include "esp_secure_boot.h"
#include "soc/soc_caps.h"

#if CONFIG_SECURE_SIGNED_APPS_RSA_SCHEME
#include "rom/rsa_pss.h"
#elif CONFIG_SECURE_SIGNED_APPS_ECDSA_V2_SCHEME
#include "rom/ecdsa.h"
#endif

#include "boot/esp_mcuboot_secure_boot_hooks.h"

#ifdef CONFIG_SECURE_BOOT_V2_ENABLED

#define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))

static const char *TAG = "secure_boot_v2";

#define BOOTLOADER_ADDR DT_REG_ADDR(DT_NODELABEL(boot_partition))

/* True while the Secure Boot eFuse batch level is open. */
static bool sb_batch_open;

/*
 * Hash flash contents without bootloader_mmap. The Zephyr MCUboot image
 * does not link bootloader_utility.c, which is where ESP-IDF defines this.
 */
esp_err_t bootloader_sha256_flash_contents(uint32_t flash_offset, uint32_t len, uint8_t *digest)
{
	uint8_t buf[128];
	bootloader_sha256_handle_t handle;

	if (digest == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	handle = bootloader_sha256_start();
	if (handle == NULL) {
		return ESP_ERR_NO_MEM;
	}

	while (len > 0) {
		size_t chunk = MIN(len, sizeof(buf));
		esp_err_t err = bootloader_flash_read(flash_offset, buf, chunk, false);

		if (err != ESP_OK) {
			bootloader_sha256_finish(handle, NULL);
			return err;
		}
		bootloader_sha256_data(handle, buf, chunk);
		flash_offset += chunk;
		len -= chunk;
	}

	bootloader_sha256_finish(handle, digest);
	return ESP_OK;
}

/* Content length up to and including the simple hash, excluding the 4 KiB pad and signature. */
static esp_err_t read_bootloader_content_length(uint32_t start, uint32_t *content_len)
{
	esp_image_header_t hdr;
	esp_err_t err;
	uint32_t image_len;
	uint32_t padded;

	err = bootloader_flash_read(start, &hdr, sizeof(hdr), false);
	if (err != ESP_OK) {
		return err;
	}
	if (hdr.magic != ESP_IMAGE_HEADER_MAGIC || hdr.segment_count == 0 ||
	    hdr.segment_count > ESP_IMAGE_MAX_SEGMENTS) {
		ESP_LOGE(TAG, "bootloader image header is invalid");
		return ESP_ERR_IMAGE_INVALID;
	}

	image_len = sizeof(hdr);
	for (uint8_t i = 0; i < hdr.segment_count; i++) {
		esp_image_segment_header_t seg;

		err = bootloader_flash_read(start + image_len, &seg, sizeof(seg), false);
		if (err != ESP_OK) {
			return err;
		}
		if (image_len + sizeof(seg) + seg.data_len < image_len) {
			return ESP_ERR_IMAGE_INVALID;
		}
		image_len += sizeof(seg) + seg.data_len;
	}

	/* Checksum byte, then pad so (content + checksum) is 16-byte aligned. */
	padded = image_len + 1;
	padded = (padded + 15U) & ~15U;
	image_len += padded - image_len;

	if (hdr.hash_appended) {
		image_len += ESP_IMAGE_HASH_LEN;
	}

	*content_len = image_len;
	return ESP_OK;
}

static esp_err_t validate_signature_block(const ets_secure_boot_sig_block_t *block, int block_num,
					  const uint8_t *image_digest)
{
	if (block->magic_byte != ETS_SECURE_BOOT_V2_SIGNATURE_MAGIC) {
		ESP_LOGD(TAG, "Signature block(%d) invalid/absent.", block_num);
		return ESP_FAIL;
	}
	if (block->block_crc != esp_rom_crc32_le(0, (uint8_t *)block, CRC_SIGN_BLOCK_LEN)) {
		ESP_LOGE(TAG, "Magic byte correct but incorrect crc.");
		return ESP_FAIL;
	}
	if (memcmp(image_digest, block->image_digest, ESP_SECURE_BOOT_DIGEST_LEN) != 0) {
		ESP_LOGE(TAG, "Magic byte & CRC correct but incorrect image digest.");
		return ESP_FAIL;
	}
	ESP_LOGD(TAG, "valid signature block(%d) found", block_num);
	return ESP_OK;
}

static esp_err_t s_calculate_image_public_key_digests(uint32_t flash_offset, uint32_t flash_size,
						      esp_image_sig_public_key_digests_t *public_key_digests)
{
	esp_err_t ret;
	uint8_t image_digest[ESP_SECURE_BOOT_DIGEST_LEN];
	__aligned(4) uint8_t key_digest[ESP_SECURE_BOOT_DIGEST_LEN];
	size_t sig_block_addr = flash_offset + ALIGN_UP(flash_size, FLASH_SECTOR_SIZE);
	static ets_secure_boot_signature_t signatures;

	memset(image_digest, 0, sizeof(image_digest));
	memset(public_key_digests, 0, sizeof(*public_key_digests));

	ESP_LOGD(TAG, "calculating public key digests for sig blocks of image offset 0x%x "
		      "(sig block offset 0x%x)",
		 flash_offset, sig_block_addr);

	ret = bootloader_sha256_flash_contents(flash_offset, sig_block_addr - flash_offset, image_digest);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "error generating image digest, %d", ret);
		return ret;
	}

	ESP_LOGD(TAG, "reading signature(s)");
	ret = bootloader_flash_read(sig_block_addr, &signatures, sizeof(signatures), false);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "failed to read signature sector at 0x%x", sig_block_addr);
		return ret;
	}

	for (unsigned int i = 0; i < SECURE_BOOT_NUM_BLOCKS; i++) {
		const ets_secure_boot_sig_block_t *block = &signatures.block[i];
		bootloader_sha256_handle_t sig_block_sha;
		uint8_t temp_verified_digest[ESP_SECURE_BOOT_DIGEST_LEN];
		bool verified;

		ret = validate_signature_block(block, i, image_digest);
		if (ret != ESP_OK) {
			ret = ESP_OK;
			break;
		}

		sig_block_sha = bootloader_sha256_start();
#if CONFIG_SECURE_SIGNED_APPS_RSA_SCHEME
		bootloader_sha256_data(sig_block_sha, &block->key, sizeof(block->key));
#elif CONFIG_SECURE_SIGNED_APPS_ECDSA_V2_SCHEME
		bootloader_sha256_data(sig_block_sha, &block->ecdsa.key, sizeof(block->ecdsa.key));
#endif
		bootloader_sha256_finish(sig_block_sha, key_digest);

#if CONFIG_SECURE_SIGNED_APPS_RSA_SCHEME
		verified = ets_rsa_pss_verify(&block->key, block->signature, image_digest,
					      temp_verified_digest);
#elif CONFIG_SECURE_SIGNED_APPS_ECDSA_V2_SCHEME
		verified = ets_ecdsa_verify(&block->ecdsa.key.point[0], block->ecdsa.signature,
					    block->ecdsa.key.curve_id, image_digest,
					    temp_verified_digest);
#else
		verified = false;
#endif
		if (!verified) {
			ret = ESP_FAIL;
			ESP_LOGE(TAG, "Secure boot key (%d) verification failed.", i);
			break;
		}
		ESP_LOGD(TAG, "Signature block (%d) is verified", i);
		memcpy(public_key_digests->key_digests[i], key_digest, ESP_SECURE_BOOT_DIGEST_LEN);
		public_key_digests->num_digests++;
	}

	if (ret == ESP_OK && public_key_digests->num_digests > 0) {
		ESP_LOGI(TAG, "Digests successfully calculated, %d valid signatures (image offset 0x%x)",
			 public_key_digests->num_digests, flash_offset);
	}
	return ret;
}

static esp_err_t check_and_generate_secure_boot_keys(void)
{
	esp_err_t ret;
#ifdef CONFIG_IDF_TARGET_ESP32
	esp_efuse_coding_scheme_t coding_scheme = esp_efuse_get_coding_scheme(EFUSE_BLK_SECURE_BOOT);

	if (coding_scheme != EFUSE_CODING_SCHEME_NONE) {
		ESP_LOGE(TAG, "No coding schemes are supported in secure boot v2.(Detected scheme: 0x%x)",
			 coding_scheme);
		return ESP_ERR_NOT_SUPPORTED;
	}
#endif

	esp_efuse_purpose_t secure_boot_key_purpose[SECURE_BOOT_NUM_BLOCKS] = {
#if SECURE_BOOT_NUM_BLOCKS == 1
		ESP_EFUSE_KEY_PURPOSE_SECURE_BOOT_V2,
#else
		ESP_EFUSE_KEY_PURPOSE_SECURE_BOOT_DIGEST0,
		ESP_EFUSE_KEY_PURPOSE_SECURE_BOOT_DIGEST1,
		ESP_EFUSE_KEY_PURPOSE_SECURE_BOOT_DIGEST2,
#endif
	};

	uint32_t content_len = 0;

	ret = read_bootloader_content_length(BOOTLOADER_ADDR, &content_len);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "bootloader image appears invalid! error %d", ret);
		return ret;
	}

	esp_efuse_block_t blocks[SECURE_BOOT_NUM_BLOCKS];
	bool has_secure_boot_digest = false;

	for (unsigned int i = 0; i < SECURE_BOOT_NUM_BLOCKS; i++) {
		blocks[i] = EFUSE_BLK_KEY_MAX;
	}

	for (unsigned int i = 0; i < SECURE_BOOT_NUM_BLOCKS; i++) {
		bool tmp_has_key = esp_efuse_find_purpose(secure_boot_key_purpose[i], &blocks[i]);

		if (tmp_has_key) {
			tmp_has_key &= !esp_efuse_key_block_unused(blocks[i]);
		}
		has_secure_boot_digest |= tmp_has_key;
	}

	esp_image_sig_public_key_digests_t boot_key_digests = {0};

	ESP_LOGI(TAG, "Secure boot digests %s",
		 has_secure_boot_digest ? "already present" : "absent, generating..");

	if (!has_secure_boot_digest) {
		ret = s_calculate_image_public_key_digests(BOOTLOADER_ADDR, content_len,
							   &boot_key_digests);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Bootloader signature block is invalid");
			return ret;
		}
		if (boot_key_digests.num_digests == 0) {
			ESP_LOGE(TAG, "No valid bootloader signature blocks found.");
			return ESP_FAIL;
		}
		ESP_LOGI(TAG, "%d signature block(s) found appended to the bootloader.",
			 boot_key_digests.num_digests);

		ESP_LOGI(TAG, "Burning public key hash to eFuse");
		ret = esp_efuse_write_keys(secure_boot_key_purpose, boot_key_digests.key_digests,
					   boot_key_digests.num_digests);
		if (ret != ESP_OK) {
			if (ret == ESP_ERR_NOT_ENOUGH_UNUSED_KEY_BLOCKS) {
				ESP_LOGE(TAG, "Bootloader signatures(%d) more than available key slots.",
					 boot_key_digests.num_digests);
			} else {
				ESP_LOGE(TAG, "Failed to write efuse block with purpose (err=0x%x). Can't continue.",
					 ret);
			}
			return ret;
		}
	} else {
		for (unsigned int i = 0; i < SECURE_BOOT_NUM_BLOCKS; i++) {
			if (blocks[i] == EFUSE_BLK_KEY_MAX) {
				ESP_LOGD(TAG, "SECURE_BOOT_DIGEST%d slot is not used", i);
				continue;
			}
#if SOC_EFUSE_REVOKE_BOOT_KEY_DIGESTS
			if (esp_efuse_get_digest_revoke(i)) {
				continue;
			}
#endif
#ifndef CONFIG_SOC_EFUSE_CONSISTS_OF_ONE_KEY_BLOCK
			if (esp_efuse_get_key_dis_read(blocks[i])) {
				ESP_LOGE(TAG, "Key digest (BLK%d) read protected, aborting...", blocks[i]);
				return ESP_FAIL;
			}
#endif
			if (esp_efuse_block_is_empty(blocks[i])) {
				ESP_LOGE(TAG, "%d eFuse block is empty, aborting...", blocks[i]);
				return ESP_FAIL;
			}
			esp_efuse_set_key_dis_write(blocks[i]);
#ifdef CONFIG_SOC_EFUSE_CONSISTS_OF_ONE_KEY_BLOCK
			size_t offset = 128;
#else
			size_t offset = 0;
#endif
			ret = esp_efuse_read_block(blocks[i],
						   boot_key_digests.key_digests[boot_key_digests.num_digests],
						   offset, ESP_SECURE_BOOT_KEY_DIGEST_LEN * 8);
			if (ret != ESP_OK) {
				ESP_LOGE(TAG, "Error during reading %d eFuse block (err=0x%x)", blocks[i], ret);
				return ret;
			}
			boot_key_digests.num_digests++;
		}
		if (boot_key_digests.num_digests == 0) {
			ESP_LOGE(TAG, "No valid pre-loaded public key digest in eFuse");
			return ESP_FAIL;
		}
		ESP_LOGW(TAG, "Using pre-loaded public key digest in eFuse");
	}

#if SOC_EFUSE_REVOKE_BOOT_KEY_DIGESTS
	if (boot_key_digests.num_digests < SECURE_BOOT_NUM_BLOCKS) {
		for (unsigned int i = boot_key_digests.num_digests; i < SECURE_BOOT_NUM_BLOCKS; i++) {
			ESP_LOGI(TAG, "Revoking empty key digest slot (%d)...", i);
			esp_efuse_set_digest_revoke(i);
		}
	}
#endif
	return ESP_OK;
}

static void secure_boot_batch_cancel(void)
{
	if (!sb_batch_open) {
		return;
	}
	esp_efuse_batch_write_cancel();
	sb_batch_open = false;
}

esp_err_t esp_mcuboot_secure_boot_prepare(esp_mcuboot_security_ctx_t *ctx)
{
	esp_err_t err;

	if (ctx == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	ctx->secure_boot_already_enabled = esp_secure_boot_enabled();
	if (ctx->secure_boot_already_enabled) {
		ESP_LOGI(TAG, "secure boot v2 is already enabled, continuing..");
		return ESP_OK;
	}

	ESP_LOGI(TAG, "enabling secure boot v2...");
	err = esp_efuse_batch_write_begin();
	if (err != ESP_OK) {
		return err;
	}
	sb_batch_open = true;

	err = check_and_generate_secure_boot_keys();
	if (err != ESP_OK) {
		secure_boot_batch_cancel();
		return err;
	}
	return ESP_OK;
}

void esp_mcuboot_secure_boot_cancel(esp_mcuboot_security_ctx_t *ctx)
{
	ARG_UNUSED(ctx);
	secure_boot_batch_cancel();
}

esp_err_t esp_mcuboot_secure_boot_finalize(esp_mcuboot_security_ctx_t *ctx)
{
	esp_err_t err;

	if (ctx == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	if (ctx->secure_boot_already_enabled) {
		return ESP_OK;
	}

	ESP_LOGI(TAG, "blowing secure boot efuse...");
	err = esp_secure_boot_enable_secure_features();
	if (err != ESP_OK) {
		return err;
	}

	err = esp_efuse_batch_write_commit();
	sb_batch_open = false;
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error programming security eFuses (err=0x%x).", err);
		return err;
	}

#ifndef CONFIG_SECURE_BOOT_FLASH_ENC_KEYS_BURN_TOGETHER
	if (!esp_secure_boot_enabled()) {
		ESP_LOGE(TAG, "Secure boot was not enabled after commit");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG, "Secure boot permanently enabled");
#endif
	return ESP_OK;
}

#endif /* CONFIG_SECURE_BOOT_V2_ENABLED */
