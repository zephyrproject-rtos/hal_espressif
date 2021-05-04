/*
 * Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_err.h"
#include "esp_event_legacy.h"

esp_err_t esp_event_send_internal(esp_event_base_t event_base,
				  int32_t event_id,
				  void *event_data,
				  size_t event_data_size,
				  TickType_t ticks_to_wait);

esp_err_t esp_event_init(void);
