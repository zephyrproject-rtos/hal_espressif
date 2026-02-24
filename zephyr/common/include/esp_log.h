/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Zephyr shim for ESP-IDF logging macros.
 * Maps ESP_LOGx macros to Zephyr's LOG_xxx macros.
 */

#pragma once

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Log level enum - kept for API compatibility with ESP-IDF components
 */
typedef enum {
    ESP_LOG_NONE    = 0,
    ESP_LOG_ERROR   = 1,
    ESP_LOG_WARN    = 2,
    ESP_LOG_INFO    = 3,
    ESP_LOG_DEBUG   = 4,
    ESP_LOG_VERBOSE = 5,
    ESP_LOG_MAX     = 6,
} esp_log_level_t;

#define ESP_LOG_LEVEL_LEN   (3)
#define ESP_LOG_LEVEL_MASK  ((1 << ESP_LOG_LEVEL_LEN) - 1)
#define ESP_LOG_GET_LEVEL(config)  ((config) & ESP_LOG_LEVEL_MASK)

/*
 * Compile-time log level control
 */
#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#endif

#define ESP_LOG_ENABLED(configs) (LOG_LOCAL_LEVEL >= ESP_LOG_GET_LEVEL(configs))
#define _ESP_LOG_ENABLED(log_level) ESP_LOG_ENABLED(log_level)
#define _ESP_LOG_EARLY_ENABLED(log_level) ESP_LOG_ENABLED(log_level)

/*
 * Runtime log level functions - stubs for API compatibility
 */
static inline esp_log_level_t esp_log_get_default_level(void)
{
    return ESP_LOG_INFO;
}

static inline void esp_log_level_set(const char *tag, esp_log_level_t level)
{
    (void)tag;
    (void)level;
    /* Zephyr uses compile-time log filtering via Kconfig */
}

static inline esp_log_level_t esp_log_level_get(const char *tag)
{
    (void)tag;
    return ESP_LOG_INFO;
}

/*
 * Main logging macros - use printk since ESP-IDF files don't have LOG_MODULE_REGISTER
 * Note: tag parameter is printed for context
 */
#define ESP_LOGE(tag, fmt, ...) esp_rom_printf("E (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) esp_rom_printf("W (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) esp_rom_printf("I (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) do { } while (0)
#define ESP_LOGV(tag, fmt, ...) do { } while (0)

/*
 * Early logging macros - use esp_rom_printf for early boot (before flash MMU is set up)
 * printk uses vfprintf which is in flash and will crash during early init.
 */
#define ESP_EARLY_LOGE(tag, fmt, ...) esp_rom_printf("E (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#define ESP_EARLY_LOGW(tag, fmt, ...) esp_rom_printf("W (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#define ESP_EARLY_LOGI(tag, fmt, ...) esp_rom_printf("I (%s): " fmt "\n", (tag), ##__VA_ARGS__)
#define ESP_EARLY_LOGD(tag, fmt, ...) do { } while (0)
#define ESP_EARLY_LOGV(tag, fmt, ...) do { } while (0)

/*
 * DRAM logging macros - for use when cache is disabled
 * Use esp_rom_printf since printk uses vfprintf which is in flash.
 */
#define ESP_DRAM_LOGE(tag, fmt, ...) esp_rom_printf("E: " fmt "\n", ##__VA_ARGS__)
#define ESP_DRAM_LOGW(tag, fmt, ...) esp_rom_printf("W: " fmt "\n", ##__VA_ARGS__)
#define ESP_DRAM_LOGI(tag, fmt, ...) esp_rom_printf("I: " fmt "\n", ##__VA_ARGS__)
#define ESP_DRAM_LOGD(tag, fmt, ...) esp_rom_printf("D: " fmt "\n", ##__VA_ARGS__)
#define ESP_DRAM_LOGV(tag, fmt, ...) esp_rom_printf("V: " fmt "\n", ##__VA_ARGS__)

/*
 * Log level macros - for dynamic level selection
 */
#define ESP_LOG_LEVEL(level, tag, fmt, ...) do { \
    if ((level) == ESP_LOG_ERROR)        { ESP_LOGE(tag, fmt, ##__VA_ARGS__); } \
    else if ((level) == ESP_LOG_WARN)    { ESP_LOGW(tag, fmt, ##__VA_ARGS__); } \
    else if ((level) == ESP_LOG_INFO)    { ESP_LOGI(tag, fmt, ##__VA_ARGS__); } \
    else if ((level) == ESP_LOG_DEBUG)   { ESP_LOGD(tag, fmt, ##__VA_ARGS__); } \
    else if ((level) == ESP_LOG_VERBOSE) { ESP_LOGV(tag, fmt, ##__VA_ARGS__); } \
} while(0)

#define ESP_LOG_LEVEL_LOCAL(level, tag, fmt, ...) \
    do { if (ESP_LOG_ENABLED(level)) ESP_LOG_LEVEL(level, tag, fmt, ##__VA_ARGS__); } while(0)

/*
 * Buffer logging - hex dump utilities (simplified)
 */
#define ESP_LOG_BUFFER_HEX_LEVEL(tag, buffer, buff_len, level) do { } while (0)
#define ESP_LOG_BUFFER_CHAR_LEVEL(tag, buffer, buff_len, level) do { } while (0)
#define ESP_LOG_BUFFER_HEXDUMP(tag, buffer, buff_len, level) do { } while (0)
#define ESP_LOG_BUFFER_HEX(tag, buffer, buff_len) do { } while (0)
#define ESP_LOG_BUFFER_CHAR(tag, buffer, buff_len) do { } while (0)

/*
 * Timestamp function - return Zephyr uptime in ms
 */
static inline uint32_t esp_log_timestamp(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static inline char *esp_log_system_timestamp(void)
{
    static char timestamp_str[16];
    snprintf(timestamp_str, sizeof(timestamp_str), "%u", esp_log_timestamp());
    return timestamp_str;
}

/*
 * DRAM string macro - not needed in Zephyr but kept for compatibility
 */
#undef DRAM_STR
#define DRAM_STR(str) (str)

/*
 * Log tag macros - ESP-IDF master uses ESP_LOG_ATTR_TAG for tag definitions
 */
#define ESP_LOG_ATTR_TAG(tag, str)      static __attribute__((unused)) const char *tag = str
#define ESP_LOG_ATTR_TAG_DRAM(tag, str) static __attribute__((unused)) const char tag[] = str

/*
 * Additional log attribute macros
 */
#define ESP_LOG_ATTR
#define ESP_LOG_ATTR_STR(str)      (str)
#define ESP_LOG_ATTR_DRAM_STR(str) (str)

/*
 * Log color macros - no color support in Zephyr by default
 */
#define LOG_COLOR_E ""
#define LOG_COLOR_W ""
#define LOG_COLOR_I ""
#define LOG_COLOR_D ""
#define LOG_COLOR_V ""
#define LOG_RESET_COLOR ""

#ifdef __cplusplus
}
#endif
