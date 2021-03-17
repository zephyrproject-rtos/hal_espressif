/*
 * Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include "esp_event.h"
#include "esp_private/wifi.h"
#include "esp_networking_priv.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(esp_event, LOG_LEVEL_INF);

K_THREAD_STACK_DEFINE(event_task_stack, CONFIG_ESP32_WIFI_EVENT_TASK_STACK_SIZE);
static struct k_thread event_task_handle;
static struct k_msgq event_queue;
static void *event_msgq_buffer;

esp_err_t esp_event_send_internal(esp_event_base_t event_base,
				  int32_t event_id,
				  void *event_data,
				  size_t event_data_size,
				  TickType_t ticks_to_wait)
{
	k_msgq_put(&event_queue, (int32_t *)&event_id, K_FOREVER);
	return ESP_OK;
}

void event_task(void *arg)
{
	int32_t event_id;

	while (1) {
		k_msgq_get(&event_queue, &event_id, K_FOREVER);

		switch (event_id) {
		case WIFI_EVENT_STA_START:
			LOG_INF("WIFI_EVENT_STA_START");
			esp_wifi_set_net_state(true);
			break;
		case WIFI_EVENT_STA_STOP:
			LOG_INF("WIFI_EVENT_STA_STOP");
			esp_wifi_set_net_state(false);
			break;
		case WIFI_EVENT_STA_CONNECTED:
			LOG_INF("WIFI_EVENT_STA_CONNECTED");
			break;
		case WIFI_EVENT_STA_DISCONNECTED:
			LOG_INF("WIFI_EVENT_STA_DISCONNECTED");

			if (IS_ENABLED(CONFIG_ESP32_WIFI_STA_RECONNECT)) {
				esp_wifi_connect();
			}
			break;
		default:
			break;
		}
	}
}

esp_err_t esp_event_init(void)
{
	event_msgq_buffer = k_malloc(sizeof(system_event_t) * 10);

	if (event_msgq_buffer == NULL) {
		LOG_ERR("memory allocation failed");
		return ESP_ERR_NO_MEM;
	}

	k_msgq_init(&event_queue, event_msgq_buffer, sizeof(system_event_t), 10);
	k_thread_create(&event_task_handle, event_task_stack, CONFIG_ESP32_WIFI_EVENT_TASK_STACK_SIZE,
			(k_thread_entry_t)event_task, NULL, NULL, NULL,
			CONFIG_ESP32_WIFI_EVENT_TASK_PRIO, K_INHERIT_PERMS, K_NO_WAIT);

	return ESP_OK;
}
