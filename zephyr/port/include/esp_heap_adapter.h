/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

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
#define esp_bt_free_func(_mem)    k_free(_mem)

#else

#define esp_bt_malloc_func(_size) malloc(_size)
#define esp_bt_free_func(_mem)    free(_mem)

#endif
