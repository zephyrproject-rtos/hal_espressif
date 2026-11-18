/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Zephyr port of the engine primitives the ESP-IDF SHA cores call
 * (components/mbedtls/port/include/sha/sha_core.h). Mirrors ESP-IDF's
 * components/mbedtls/port/sha/core/sha.c in block mode; that file is not used
 * because it brings FreeRTOS semaphores, esp_cache, the shared GDMA and
 * esp_crypto_lock with it.
 *
 * The cores call acquire -> set_mode -> [write digest] -> blocks -> read digest
 * -> release for every update that has at least one full block, and keep the
 * digest in their own context in between, so operations from different threads
 * interleave on the single engine. The lock is a k_mutex (priority inheritance).
 * Blocks are started only on an idle engine, see esp_sha_block().
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "hal/sha_hal.h"
#include "hal/sha_ll.h"
#include "sha/sha_core.h"

#include "esp_sha_port.h"

LOG_MODULE_REGISTER(esp_psa_sha, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * The engine's clock comes from the `sha` devicetree node by label, not by
 * compatible: "espressif,esp32-sha" is also the compatible the Zephyr crypto
 * API driver (drivers/crypto/crypto_esp32_sha.c) binds to, and the two drivers
 * are mutually exclusive in Kconfig. Same hardware, one owner.
 */
#define SHA_NODE DT_NODELABEL(sha)

/* SHA-1, SHA-224 and SHA-256 blocks; the port does not do SHA-384/512. */
#define BLOCK_BYTES 64U
#define BLOCK_WORDS (BLOCK_BYTES / sizeof(uint32_t))

static K_MUTEX_DEFINE(sha_lock);

/*
 * 0: clock not on yet, 1: engine usable, < 0: the clock could not be turned on
 * (latched until reboot). Moves away from 0 once, so a lock-free read that sees
 * a non-zero value is final and every operation after the first skips the
 * lock, which another thread may be holding through a long update.
 */
static atomic_t engine_state = ATOMIC_INIT(0);

int esp_sha_port_ready(void)
{
	atomic_val_t state = atomic_get(&engine_state);

	if (state != 0) {
		return (state > 0) ? 0 : (int)state;
	}

	k_mutex_lock(&sha_lock, K_FOREVER);
	state = atomic_get(&engine_state);
	if (state == 0) {
#if DT_NODE_HAS_STATUS(SHA_NODE, okay)
		const struct device *clk = DEVICE_DT_GET(DT_CLOCKS_CTLR(SHA_NODE));

		if (!device_is_ready(clk)) {
			/* too early in boot: decline this operation, try again next time */
			k_mutex_unlock(&sha_lock);
			return -EAGAIN;
		}

		int rc = clock_control_on(clk,
					  (clock_control_subsys_t)DT_CLOCKS_CELL(SHA_NODE, offset));

		if (rc == 0 || rc == -EALREADY) {
			state = 1;
		} else {
			state = (rc < 0) ? rc : -EIO;
			LOG_ERR("SHA engine clock failed (%d): hashing stays in software", rc);
		}
#else
		state = -ENODEV;
#endif
		atomic_set(&engine_state, state);
	}
	k_mutex_unlock(&sha_lock);

	return (state > 0) ? 0 : (int)state;
}

static size_t block_length(esp_sha_type sha_type)
{
	switch (sha_type) {
	case SHA1:
	case SHA2_224:
	case SHA2_256:
		return BLOCK_BYTES;
	default:
		return 0;
	}
}

void esp_sha_acquire_hardware(void)
{
	k_mutex_lock(&sha_lock, K_FOREVER);
}

void esp_sha_release_hardware(void)
{
	k_mutex_unlock(&sha_lock);
}

void esp_sha_set_mode(esp_sha_type sha_type)
{
	sha_hal_wait_idle();
	sha_hal_set_mode(sha_type);
}

/*
 * One block, in the order the TRM gives (ESP32-P4 TRM, SHA Accelerator, "Hash
 * Operation Process", step 4): the message words may be written while the
 * previous block is still being compressed, but START/CONTINUE must wait for
 * BUSY to clear. sha_hal_hash_block() issues CONTINUE right after the write;
 * on the ESP32-P4 a CONTINUE issued while the engine is busy gives a wrong
 * digest, and the HAL order holds only because writing 16 words takes longer
 * than one compression. Overlapping the write with the compression is also
 * faster. The engine takes the block as words and the cores hand over the
 * caller's buffer as is, so an unaligned block goes through a copy.
 */
void esp_sha_block(esp_sha_type sha_type, const void *data_block, bool is_first_block)
{
	size_t words = block_length(sha_type) / sizeof(uint32_t);
	uint32_t aligned[BLOCK_WORDS];

	if (((uintptr_t)data_block % sizeof(uint32_t)) != 0U) {
		memcpy(aligned, data_block, words * sizeof(uint32_t));
		data_block = aligned;
	}
	sha_ll_fill_text_block(data_block, words);
	sha_hal_wait_idle();
	if (is_first_block) {
		sha_ll_start_block(sha_type);
	} else {
		sha_ll_continue_block(sha_type);
	}
}

void esp_sha_read_digest_state(esp_sha_type sha_type, void *digest_state)
{
	sha_hal_read_digest(sha_type, digest_state);
}

void esp_sha_write_digest_state(esp_sha_type sha_type, void *digest_state)
{
	sha_hal_write_digest(sha_type, digest_state);
}

#if SOC_SHA_SUPPORT_DMA
/*
 * The cores hand updates longer than SHA_DMA_MODE_THRESHOLD to this function.
 * No DMA here yet: the same blocks go through the CPU, the leftover block
 * `buf` first, as ESP-IDF's esp_sha_block_mode_fallback() does.
 */
int esp_sha_dma(esp_sha_type sha_type, const void *input, uint32_t ilen, const void *buf,
		uint32_t buf_len, bool is_first_block)
{
	size_t blk_len = block_length(sha_type);
	const uint8_t *in = input;

	if (blk_len == 0) {
		return -EINVAL;
	}
	if (buf_len != 0) {
		esp_sha_block(sha_type, buf, is_first_block);
		is_first_block = false;
	}
	for (uint32_t i = 0; i < ilen / blk_len; i++) {
		esp_sha_block(sha_type, in + blk_len * i, is_first_block);
		is_first_block = false;
	}
	return 0;
}
#endif /* SOC_SHA_SUPPORT_DMA */
