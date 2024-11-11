/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/multi_heap/shared_multi_heap.h>

/* Select heap to be used for WiFi adapter and WPA supplicant */
#if defined(CONFIG_ESP_WIFI_HEAP_SYSTEM)

#define esp_wifi_malloc_func(_size)         k_malloc(_size)
#define esp_wifi_calloc_func(_nmemb, _size) k_calloc(_nmemb, _size)
#define esp_wifi_free_func(_mem)            k_free(_mem)

#define os_wpa_malloc_func(_size)         k_malloc(_size)
#define os_wpa_realloc_func(_ptr, _size)  k_realloc(_ptr, _size)
#define os_wpa_calloc_func(_nmemb, _size) k_calloc(_nmemb, _size)
#define os_wpa_free_func(_mem)            k_free(_mem)

#elif defined(CONFIG_ESP_WIFI_HEAP_SPIRAM)

#include <esp_memory_utils.h>

static inline void* esp_wifi_malloc_func(size_t _size)
{
	return shared_multi_heap_aligned_alloc(SMH_REG_ATTR_EXTERNAL, 16, _size);
}

static inline void* esp_wifi_calloc_func(size_t _nmemb, size_t _size)
{
	void *p = shared_multi_heap_aligned_alloc(SMH_REG_ATTR_EXTERNAL, 16, _nmemb * _size);
	if (p) {
		memset(p, 0, _nmemb * _size);
	}
	return p;
}

static inline void esp_wifi_free_func(void *_mem)
{
	if (esp_ptr_in_dram(_mem)) {
		k_free(_mem);
	} else {
		shared_multi_heap_free(_mem);
	}
}

static inline void* os_wpa_malloc_func(size_t _size)
{
	return shared_multi_heap_aligned_alloc(SMH_REG_ATTR_EXTERNAL, 16, _size);
}

static inline void* os_wpa_realloc_func(void *_ptr, size_t _size)
{
	if (_ptr) {
		esp_wifi_free_func(_ptr);
	}
	return os_wpa_malloc_func(_size);
}

static inline void* os_wpa_calloc_func(size_t _nmemb, size_t _size)
{
	void *p = os_wpa_malloc_func(_nmemb * _size);
	if (p) {
		memset(p, 0, _nmemb * _size);
	}
	return p;
}
static inline void os_wpa_free_func(void *_mem)
{
	esp_wifi_free_func(_mem);
}

#else

#define esp_wifi_malloc_func(_size)         malloc(_size)
#define esp_wifi_calloc_func(_nmemb, _size) calloc(_nmemb, _size)
#define esp_wifi_free_func(_mem)            free(_mem)

#define os_wpa_malloc_func(_size)         malloc(_size)
#define os_wpa_realloc_func(_ptr, _size)  realloc(_ptr, _size)
#define os_wpa_calloc_func(_nmemb, _size) calloc(_nmemb, _size)
#define os_wpa_free_func(_mem)            free(_mem)

#endif

/* Select heap to be used for BLE adapter */
#if defined(CONFIG_ESP_BT_HEAP_SYSTEM)

#define esp_bt_malloc_func(_size) k_malloc(_size)
#define esp_bt_calloc_func(_nmemb, _size) k_calloc(_nmemb, _size)
#define esp_bt_free_func(_mem)    k_free(_mem)

#elif defined(CONFIG_ESP_BT_HEAP_SPIRAM)

static inline void* esp_bt_malloc_func(size_t _size)
{
	return shared_multi_heap_aligned_alloc(SMH_REG_ATTR_EXTERNAL, 16, _size);
}

static inline void* esp_bt_calloc_func(size_t _nmemb, size_t _size)
{
	void *p = shared_multi_heap_aligned_alloc(SMH_REG_ATTR_EXTERNAL, 16, _nmemb * _size);
	if (p) {
		memset(p, 0, _nmemb * _size);
	}
	return p;
}

static inline void esp_bt_free_func(_mem)
{
	shared_multi_heap_free(_mem);
}

#else

#define esp_bt_malloc_func(_size) malloc(_size)
#define esp_bt_calloc_func(_size) calloc(_size)
#define esp_bt_free_func(_mem)    free(_mem)

#endif
