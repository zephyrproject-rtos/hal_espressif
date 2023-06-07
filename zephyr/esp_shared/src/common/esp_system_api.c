/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "string.h"
#include "soc/efuse_reg.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_private/system_internal.h"
#include "esp_cpu.h"
#include "soc/rtc.h"
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
