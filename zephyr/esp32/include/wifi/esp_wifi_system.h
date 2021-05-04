/*
 * Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"

#define TWO_UNIVERSAL_MAC_ADDR 2
#define FOUR_UNIVERSAL_MAC_ADDR 4
#define UNIVERSAL_MAC_ADDR_NUM 4

typedef enum {
	ESP_MAC_WIFI_STA,
	ESP_MAC_WIFI_SOFTAP,
	ESP_MAC_BT,
	ESP_MAC_ETH,
} esp_mac_type_t;

esp_err_t esp_read_mac(uint8_t *mac, esp_mac_type_t type);
