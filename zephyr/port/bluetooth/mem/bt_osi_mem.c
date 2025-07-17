/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_attr.h"
#include "esp_heap_adapter.h"
#include "sdkconfig.h"

IRAM_ATTR void *bt_osi_mem_malloc(size_t size)
{
    return esp_bt_malloc_func(size);
}

IRAM_ATTR void *bt_osi_mem_calloc(size_t n, size_t size)
{
    return esp_bt_calloc_func(n, size);
}

IRAM_ATTR void *bt_osi_mem_malloc_internal(size_t size)
{
    return esp_bt_malloc_func(size);
}

IRAM_ATTR void *bt_osi_mem_calloc_internal(size_t n, size_t size)
{
    return esp_bt_calloc_func(n, size);
}

IRAM_ATTR void bt_osi_mem_free(void *ptr)
{
    esp_bt_free_func(ptr);
}
