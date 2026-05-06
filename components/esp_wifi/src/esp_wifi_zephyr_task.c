/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_wifi_zephyr_task.h"

#include <errno.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(esp_wifi_zephyr_task);

#ifndef CONFIG_ESP32_WIFI_ADAPTER_TASK_COUNT
#define CONFIG_ESP32_WIFI_ADAPTER_TASK_COUNT 2
#endif

#define WIFI_ADAPTER_TASK_SLOT_COUNT CONFIG_ESP32_WIFI_ADAPTER_TASK_COUNT

static void wifi_task_cleanup(struct k_work *work);

static struct k_thread wifi_task_handles[WIFI_ADAPTER_TASK_SLOT_COUNT];
static k_thread_stack_t *wifi_task_stacks[WIFI_ADAPTER_TASK_SLOT_COUNT];
static bool wifi_task_slots_used[WIFI_ADAPTER_TASK_SLOT_COUNT];
static bool wifi_task_cleanup_pending[WIFI_ADAPTER_TASK_SLOT_COUNT];
static K_MUTEX_DEFINE(wifi_task_slots_lock);
static K_WORK_DELAYABLE_DEFINE(wifi_task_cleanup_work, wifi_task_cleanup);

static int wifi_task_slot_alloc(void)
{
	int slot = -1;

	k_mutex_lock(&wifi_task_slots_lock, K_FOREVER);
	for (int i = 0; i < WIFI_ADAPTER_TASK_SLOT_COUNT; i++) {
		if (!wifi_task_slots_used[i]) {
			wifi_task_slots_used[i] = true;
			slot = i;
			break;
		}
	}
	k_mutex_unlock(&wifi_task_slots_lock);

	return slot;
}

static int wifi_task_slot_find(k_tid_t tid)
{
	int slot = -1;

	k_mutex_lock(&wifi_task_slots_lock, K_FOREVER);
	for (int i = 0; i < WIFI_ADAPTER_TASK_SLOT_COUNT; i++) {
		if (tid == &wifi_task_handles[i]) {
			slot = i;
			break;
		}
	}
	k_mutex_unlock(&wifi_task_slots_lock);

	return slot;
}

static void wifi_task_slot_free(int slot)
{
	k_mutex_lock(&wifi_task_slots_lock, K_FOREVER);
	wifi_task_cleanup_pending[slot] = false;
	wifi_task_slots_used[slot] = false;
	k_mutex_unlock(&wifi_task_slots_lock);
}

static void wifi_task_cleanup(struct k_work *work)
{
	bool reschedule = false;

	ARG_UNUSED(work);

	for (int i = 0; i < WIFI_ADAPTER_TASK_SLOT_COUNT; i++) {
		k_thread_stack_t *stack;
		int ret;

		k_mutex_lock(&wifi_task_slots_lock, K_FOREVER);
		if (!wifi_task_cleanup_pending[i] || wifi_task_stacks[i] == NULL) {
			k_mutex_unlock(&wifi_task_slots_lock);
			continue;
		}
		stack = wifi_task_stacks[i];
		k_mutex_unlock(&wifi_task_slots_lock);

		ret = k_thread_stack_free(stack);
		if (ret == 0) {
			k_mutex_lock(&wifi_task_slots_lock, K_FOREVER);
			if (wifi_task_stacks[i] == stack) {
				wifi_task_stacks[i] = NULL;
				wifi_task_cleanup_pending[i] = false;
				wifi_task_slots_used[i] = false;
			}
			k_mutex_unlock(&wifi_task_slots_lock);
		} else if (ret == -EBUSY) {
			reschedule = true;
		} else {
			LOG_ERR("failed to free Wi-Fi task stack %d (%d)", i, ret);
			reschedule = true;
		}
	}

	if (reschedule) {
		(void)k_work_reschedule(&wifi_task_cleanup_work, K_MSEC(100));
	}
}

int32_t esp_wifi_zephyr_task_create(void *task_func, const char *name,
				    uint32_t stack_depth, void *param,
				    uint32_t prio, void *task_handle)
{
	int slot = wifi_task_slot_alloc();

	if (slot < 0) {
		LOG_ERR("no Wi-Fi adapter task slots available for %s", name);
		return 0;
	}

	k_thread_stack_t *wifi_stack = k_thread_stack_alloc(stack_depth,
					IS_ENABLED(CONFIG_USERSPACE) ? K_USER : 0);
	if (wifi_stack == NULL) {
		LOG_ERR("failed to allocate stack for Wi-Fi task %s (%u bytes)",
			name, stack_depth);
		wifi_task_slot_free(slot);
		return 0;
	}

	wifi_task_stacks[slot] = wifi_stack;
	wifi_task_cleanup_pending[slot] = false;

	k_tid_t tid = k_thread_create(&wifi_task_handles[slot], wifi_stack,
				      stack_depth, (k_thread_entry_t)task_func,
				      param, NULL, NULL, prio, K_INHERIT_PERMS,
				      K_NO_WAIT);

	k_thread_name_set(tid, name);

	if (task_handle != NULL) {
		*(void **)task_handle = tid;
	}

	return 1;
}

void esp_wifi_zephyr_task_delete(void *handle)
{
	k_tid_t tid = handle != NULL ? (k_tid_t)handle : k_current_get();
	int slot = wifi_task_slot_find(tid);

	if (slot >= 0) {
		k_mutex_lock(&wifi_task_slots_lock, K_FOREVER);
		wifi_task_cleanup_pending[slot] = true;
		k_mutex_unlock(&wifi_task_slots_lock);
		(void)k_work_reschedule(&wifi_task_cleanup_work, K_MSEC(10));
	}

	k_thread_abort(tid);
}
