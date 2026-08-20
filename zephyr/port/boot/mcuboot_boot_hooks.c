/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bootutil/boot_hooks.h"
#include "bootutil/bootutil.h"
#include "bootutil/fault_injection_hardening.h"

#include "bootloader_random.h"
#include "esp_rom_sys.h"

#include "boot/mcuboot_flash_encrypt.h"

static void mcuboot_bootloader_reset(void)
{
	esp_rom_delay_us(1000);
	esp_rom_software_reset_system();
	while (true) {
	}
}

fih_ret boot_go_hook(struct boot_rsp *rsp)
{
	FIH_DECLARE(fih_rc, FIH_FAILURE);
	esp_err_t err;

	err = esp_mcuboot_security_prepare();
	if (err != ESP_OK) {
		FIH_RET(FIH_FAILURE);
	}

	FIH_CALL(boot_go, fih_rc, rsp);
	if (FIH_NOT_EQ(fih_rc, FIH_SUCCESS)) {
		esp_mcuboot_security_cancel();
		FIH_RET(fih_rc);
	}

	err = esp_mcuboot_security_finalize();
	if (err == ESP_ERR_FLASH_ENCRYPT_RESET_NEEDED) {
		mcuboot_bootloader_reset();
	}
	if (err != ESP_OK) {
		FIH_RET(FIH_FAILURE);
	}

	bootloader_random_disable();
	FIH_RET(FIH_SUCCESS);
}
