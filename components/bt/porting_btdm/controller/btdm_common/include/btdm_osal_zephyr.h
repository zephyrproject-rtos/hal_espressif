/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _BTDM_OSAL_ZEPHYR_H_
#define _BTDM_OSAL_ZEPHYR_H_

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>

#include "btdm_osal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Backing objects for the controller handles, allocated from btdm mempools. */

struct btdm_osal_event_zephyr {
	bool queued;
	void (*fn)(struct btdm_osal_event *ev);
	void *arg;
};

struct btdm_osal_callout_zephyr {
	struct k_timer timer;
	struct k_work work;
	struct btdm_osal_eventq *evq;
	struct btdm_osal_event ev;
};

struct btdm_osal_mutex_zephyr {
	struct k_mutex handle;
};

struct btdm_osal_sem_zephyr {
	struct k_sem handle;
};

#ifdef __cplusplus
}
#endif

#endif /* _BTDM_OSAL_ZEPHYR_H_ */
