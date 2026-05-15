/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ESP_WIFI_ZEPHYR_TASK_H
#define ESP_WIFI_ZEPHYR_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t esp_wifi_zephyr_task_create(void *task_func, const char *name,
				    uint32_t stack_depth, void *param,
				    uint32_t prio, void *task_handle);

void esp_wifi_zephyr_task_delete(void *handle);

#ifdef __cplusplus
}
#endif

#endif /* ESP_WIFI_ZEPHYR_TASK_H */
