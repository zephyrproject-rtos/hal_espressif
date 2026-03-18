/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>
#include <esp_rom_sys.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESP_LOG_NONE    = 0,
    ESP_LOG_ERROR   = 1,
    ESP_LOG_WARN    = 2,
    ESP_LOG_INFO    = 3,
    ESP_LOG_DEBUG   = 4,
    ESP_LOG_VERBOSE = 5,
    ESP_LOG_MAX     = 6,
} esp_log_level_t;

#ifdef CONFIG_ESP_HAL_LOG_LEVEL
#define ESP_HAL_LOG_LEVEL  CONFIG_ESP_HAL_LOG_LEVEL
#else
#define ESP_HAL_LOG_LEVEL  0
#endif

#ifdef CONFIG_ESP_HAL_EARLY_LOG_LEVEL
#define ESP_HAL_EARLY_LOG_LEVEL  CONFIG_ESP_HAL_EARLY_LOG_LEVEL
#else
#define ESP_HAL_EARLY_LOG_LEVEL  3
#endif

#define _ESP_LOG_NOOP(tag, fmt, ...) \
    do { if (0) { esp_rom_printf("%s" fmt, tag, ##__VA_ARGS__); } } while (0)

#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL  ESP_HAL_LOG_LEVEL
#endif

#define ESP_LOG_ENABLED(configs)  (LOG_LOCAL_LEVEL >= (configs))

static inline void esp_log_level_set(const char *tag, esp_log_level_t level)
{
    (void)tag;
    (void)level;
}

/* Normal logging - printk, gated by ESP_HAL_LOG_LEVEL */
#if ESP_HAL_LOG_LEVEL >= 1
#define ESP_LOGE(tag, fmt, ...) printk("E (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#else
#define ESP_LOGE(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

#if ESP_HAL_LOG_LEVEL >= 2
#define ESP_LOGW(tag, fmt, ...) printk("W (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#else
#define ESP_LOGW(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

#if ESP_HAL_LOG_LEVEL >= 3
#define ESP_LOGI(tag, fmt, ...) printk("I (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#else
#define ESP_LOGI(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

#if ESP_HAL_LOG_LEVEL >= 4
#define ESP_LOGD(tag, fmt, ...) printk("D (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#else
#define ESP_LOGD(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

#if ESP_HAL_LOG_LEVEL >= 5
#define ESP_LOGV(tag, fmt, ...) printk("V (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#else
#define ESP_LOGV(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

/* Early boot logging - esp_rom_printf, gated by ESP_HAL_EARLY_LOG_LEVEL */
#if ESP_HAL_EARLY_LOG_LEVEL >= 1
#define ESP_EARLY_LOGE(tag, fmt, ...) esp_rom_printf("E (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#else
#define ESP_EARLY_LOGE(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

#if ESP_HAL_EARLY_LOG_LEVEL >= 2
#define ESP_EARLY_LOGW(tag, fmt, ...) esp_rom_printf("W (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#else
#define ESP_EARLY_LOGW(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

#if ESP_HAL_EARLY_LOG_LEVEL >= 3
#define ESP_EARLY_LOGI(tag, fmt, ...) esp_rom_printf("I (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#else
#define ESP_EARLY_LOGI(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

#if ESP_HAL_EARLY_LOG_LEVEL >= 4
#define ESP_EARLY_LOGD(tag, fmt, ...) esp_rom_printf("D (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#else
#define ESP_EARLY_LOGD(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

#if ESP_HAL_EARLY_LOG_LEVEL >= 5
#define ESP_EARLY_LOGV(tag, fmt, ...) esp_rom_printf("V (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#else
#define ESP_EARLY_LOGV(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

/* DRAM logging - esp_rom_printf, gated by ESP_HAL_EARLY_LOG_LEVEL */
#if ESP_HAL_EARLY_LOG_LEVEL >= 1
#define ESP_DRAM_LOGE(tag, fmt, ...) esp_rom_printf("E: " fmt "\n", ##__VA_ARGS__)
#else
#define ESP_DRAM_LOGE(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

#if ESP_HAL_EARLY_LOG_LEVEL >= 2
#define ESP_DRAM_LOGW(tag, fmt, ...) esp_rom_printf("W: " fmt "\n", ##__VA_ARGS__)
#else
#define ESP_DRAM_LOGW(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

#if ESP_HAL_EARLY_LOG_LEVEL >= 3
#define ESP_DRAM_LOGI(tag, fmt, ...) esp_rom_printf("I: " fmt "\n", ##__VA_ARGS__)
#else
#define ESP_DRAM_LOGI(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

#if ESP_HAL_EARLY_LOG_LEVEL >= 4
#define ESP_DRAM_LOGD(tag, fmt, ...) esp_rom_printf("D: " fmt "\n", ##__VA_ARGS__)
#else
#define ESP_DRAM_LOGD(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

#if ESP_HAL_EARLY_LOG_LEVEL >= 5
#define ESP_DRAM_LOGV(tag, fmt, ...) esp_rom_printf("V: " fmt "\n", ##__VA_ARGS__)
#else
#define ESP_DRAM_LOGV(tag, fmt, ...) _ESP_LOG_NOOP(tag, fmt, ##__VA_ARGS__)
#endif

/* Dynamic level selection */
#define ESP_LOG_LEVEL_LOCAL(level, tag, fmt, ...) \
    do { if (ESP_LOG_ENABLED(level)) { \
        if ((level) == ESP_LOG_ERROR)        { ESP_LOGE(tag, fmt, ##__VA_ARGS__); } \
        else if ((level) == ESP_LOG_WARN)    { ESP_LOGW(tag, fmt, ##__VA_ARGS__); } \
        else if ((level) == ESP_LOG_INFO)    { ESP_LOGI(tag, fmt, ##__VA_ARGS__); } \
        else if ((level) == ESP_LOG_DEBUG)   { ESP_LOGD(tag, fmt, ##__VA_ARGS__); } \
        else if ((level) == ESP_LOG_VERBOSE) { ESP_LOGV(tag, fmt, ##__VA_ARGS__); } \
    } } while (0)

/* Buffer logging stubs */
#define ESP_LOG_BUFFER_HEXDUMP(tag, buffer, buff_len, level) do { } while (0)
#define ESP_LOG_BUFFER_HEX(tag, buffer, buff_len)            do { } while (0)
#define ESP_LOG_BUFFER_CHAR(tag, buffer, buff_len)           do { } while (0)

#undef DRAM_STR
#define DRAM_STR(str) (str)

#define ESP_LOG_ATTR_TAG(tag, str)      static __attribute__((unused)) const char *tag = str
#define ESP_LOG_ATTR_TAG_DRAM(tag, str) static __attribute__((unused)) const char tag[] = str
#define ESP_LOG_ATTR
#define ESP_LOG_ATTR_STR(str)           (str)
#define ESP_LOG_ATTR_DRAM_STR(str)      (str)

static inline uint32_t esp_log_timestamp(void)
{
    return 0;
}

#define LOG_COLOR_I  ""
#define LOG_RESET_COLOR ""

#ifdef __cplusplus
}
#endif
