/*
 * Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include "string.h"
#include "esp_wifi_system.h"
#include "soc/efuse_reg.h"
#include "esp_log.h"

static unsigned int lock_key;

static const char *TAG = "system_api";

esp_err_t esp_read_mac(uint8_t *mac, esp_mac_type_t type)
{
	uint8_t efuse_mac[6] = { 0 };

	if (mac == NULL) {
		ESP_LOGE(TAG, "mac address param is NULL");
		return ESP_ERR_INVALID_ARG;
	}

	if (type < ESP_MAC_WIFI_STA || type > ESP_MAC_ETH) {
		ESP_LOGE(TAG, "mac type is incorrect");
		return ESP_ERR_INVALID_ARG;
	}

	lock_key = irq_lock();
	uint32_t high = REG_READ(EFUSE_BLK0_RDATA2_REG);
	uint32_t low = REG_READ(EFUSE_BLK0_RDATA1_REG);

	irq_unlock(lock_key);

	for (int i = 4; i > 0; i--) {
		efuse_mac[2 + (4 - i)] = (low >> ((i - 1) * 8)) & 0xff;
	}

	efuse_mac[1] = high & 0xff;
	efuse_mac[0] = (high >> 8) & 0xff;

	switch (type) {
	case ESP_MAC_WIFI_STA:
		memcpy(mac, efuse_mac, 6);
		break;
	case ESP_MAC_WIFI_SOFTAP:
		if (UNIVERSAL_MAC_ADDR_NUM == FOUR_UNIVERSAL_MAC_ADDR) {
			memcpy(mac, efuse_mac, 6);
			mac[5] += 1;
		}
		break;
	default:
		ESP_LOGW(TAG, "incorrect mac type");
		break;
	}

	return ESP_OK;
}
