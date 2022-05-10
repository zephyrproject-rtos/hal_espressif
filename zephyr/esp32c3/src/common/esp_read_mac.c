/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_system.h"
#include <rom/efuse.h>
#include <zephyr/kernel.h>

esp_err_t esp_read_mac(uint8_t *mac, esp_mac_type_t type)
{
	ARG_UNUSED(type);

	/* obtains the factory mac address by reading corresponding
	 * efuse
	 */
	ets_efuse_read();
	uint32_t mac_word_0;
	uint32_t mac_word_1;
	volatile uint32_t *efuse_register;

	efuse_register = (volatile uint32_t *)ets_efuse_get_read_register_address(ETS_EFUSE_MAC_SPI_SYS_0);
	mac_word_0 = (uint32_t)*efuse_register;
	mac_word_1 = (uint32_t)*(efuse_register + 4);

	mac[5] = (mac_word_0 & 0xFF);
	mac[4] = (mac_word_0 & 0xFF00) >> 8;
	mac[3] = (mac_word_0 & 0xFF000) >> 16;
	mac[2] = (mac_word_0 & 0xFF000000) >> 24;
	mac[1] = (mac_word_1 & 0xFF);
	mac[0] = (mac_word_1 & 0xFF00) >> 8;

	return ESP_OK;
}
