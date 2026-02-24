/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Stub header for Zephyr - esp_newlib is not used.
 * Zephyr uses picolibc instead of newlib.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

static inline void esp_newlib_time_init(void) {}

static inline void esp_set_time_from_rtc(void) {}

static inline void esp_sync_timekeeping_timers(void) {}

#ifdef __cplusplus
}
#endif
