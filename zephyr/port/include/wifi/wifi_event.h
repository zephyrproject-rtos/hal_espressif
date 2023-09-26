/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

void esp_wifi_event_handler(const char *event_base, int32_t event_id, void *event_data,
			    size_t event_data_size, uint32_t ticks_to_wait);
