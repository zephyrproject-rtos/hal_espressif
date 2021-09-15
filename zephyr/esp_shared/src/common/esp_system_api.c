/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <logging/log.h>
#include "string.h"
#include "soc/efuse_reg.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_private/system_internal.h"
#include "soc/cpu.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_rom_uart.h"

static unsigned int lock_key;

static const char *TAG = "system_api";

#define SHUTDOWN_HANDLERS_NO 4
static shutdown_handler_t shutdown_handlers[SHUTDOWN_HANDLERS_NO];

esp_err_t esp_register_shutdown_handler(shutdown_handler_t handler)
{
	for (int i = 0; i < SHUTDOWN_HANDLERS_NO; i++) {
		if (shutdown_handlers[i] == handler) {
			return ESP_ERR_INVALID_STATE;
		} else if (shutdown_handlers[i] == NULL) {
			shutdown_handlers[i] = handler;
			return ESP_OK;
		}
	}
	return ESP_ERR_NO_MEM;
}

esp_err_t esp_unregister_shutdown_handler(shutdown_handler_t handler)
{
	for (int i = 0; i < SHUTDOWN_HANDLERS_NO; i++) {
		if (shutdown_handlers[i] == handler) {
			shutdown_handlers[i] = NULL;
			return ESP_OK;
		}
	}
	return ESP_ERR_INVALID_STATE;
}

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
#if CONFIG_IDF_TARGET_ESP32
	uint32_t high = REG_READ(EFUSE_BLK0_RDATA2_REG);
	uint32_t low = REG_READ(EFUSE_BLK0_RDATA1_REG);
#elif CONFIG_IDF_TARGET_ESP32S2
	uint32_t high = REG_READ(EFUSE_RD_MAC_SPI_SYS_1_REG);
	uint32_t low = REG_READ(EFUSE_RD_MAC_SPI_SYS_0_REG);
#endif

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
	case ESP_MAC_BT:
#if CONFIG_ESP_MAC_ADDR_UNIVERSE_BT
		memcpy(mac, efuse_mac, 6);
		mac[5] += CONFIG_ESP_MAC_ADDR_UNIVERSE_BT_OFFSET;
#endif
		break;
	default:
		ESP_LOGW(TAG, "incorrect mac type");
		break;
	}

	return ESP_OK;
}
