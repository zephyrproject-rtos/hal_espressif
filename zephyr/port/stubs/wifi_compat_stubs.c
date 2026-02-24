/*
 * Copyright (c) 2025 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Compatibility stubs for ESP-IDF master Wi-Fi blobs.
 */

#include <zephyr/kernel.h>
#include "esp_wifi.h"
#include "esp_event.h"

/* ESP-NOW OUI - required by blob */
const uint8_t g_espnow_user_oui[3] = {0x18, 0xFE, 0x34};

/* Regulatory domain data - required by blob */
const void *regdomain_table = NULL;
const void *regulatory_data = NULL;

void __attribute__((weak)) pm_beacon_offset_funcs_empty_init(void)
{
}

/* Route esp_event_post to Zephyr Wi-Fi event handler */
esp_err_t esp_event_post(esp_event_base_t event_base, int32_t event_id,
			 const void *event_data, size_t event_data_size,
			 uint32_t ticks_to_wait)
{
	extern void esp_wifi_event_handler(esp_event_base_t event_base,
					   int32_t event_id,
					   void *event_data,
					   size_t event_data_size,
					   uint32_t ticks_to_wait);
	esp_wifi_event_handler(event_base, event_id, (void *)event_data,
			       event_data_size, ticks_to_wait);
	return ESP_OK;
}
