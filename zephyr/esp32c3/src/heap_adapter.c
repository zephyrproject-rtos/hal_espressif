/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <esp_heap_caps_adapter.h>
#include "esp_log.h"
#include "esp_attr.h"

IRAM_ATTR void *heap_caps_malloc(size_t size, uint32_t caps)
{
    return k_malloc(size);
}
