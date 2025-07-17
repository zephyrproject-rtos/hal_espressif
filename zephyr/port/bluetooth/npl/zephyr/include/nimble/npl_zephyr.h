/*
 * SPDX-FileCopyrightText: 2015-2022 The Apache Software Foundation (ASF)
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPDX-FileContributor: 2019-2022 Espressif Systems (Shanghai) CO LTD
 */

#ifndef _NPL_ZEPHYR_H_
#define _NPL_ZEPHYR_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <zephyr/kernel.h>
#include <esp_timer.h>
#include "sdkconfig.h"

#define BLE_NPL_USE_ESP_TIMER (1)

typedef struct {
	uint16_t evt_count;
	uint16_t evtq_count;
	uint16_t co_count;
	uint16_t sem_count;
	uint16_t mutex_count;
} ble_npl_count_info_t;

typedef void ble_npl_event_fn(struct ble_npl_event *ev);

struct ble_npl_event_zephyr {
	bool queued;
	ble_npl_event_fn *fn;
	void *arg;
};

struct ble_npl_eventq_zephyr {
	struct k_msgq q;
};

struct ble_npl_callout_zephyr {
	esp_timer_handle_t handle;
	struct ble_npl_eventq *evq;
	struct ble_npl_event ev;
};

struct ble_npl_mutex_zephyr {
	struct k_mutex handle;
};

struct ble_npl_sem_zephyr {
	struct k_sem handle;
};

typedef void ble_npl_event_fn_zephyr(struct ble_npl_event_zephyr *ev);

struct ble_npl_eventq *npl_zephyr_eventq_dflt_get(void);

struct ble_npl_event *npl_zephyr_eventq_get(struct ble_npl_eventq *evq, ble_npl_time_t tmo);

void npl_zephyr_eventq_put(struct ble_npl_eventq *evq, struct ble_npl_event *ev);

void npl_zephyr_eventq_remove(struct ble_npl_eventq *evq, struct ble_npl_event *ev);

ble_npl_error_t npl_zephyr_mutex_init(struct ble_npl_mutex *mu);
ble_npl_error_t npl_zephyr_mutex_deinit(struct ble_npl_mutex *mu);

ble_npl_error_t npl_zephyr_mutex_pend(struct ble_npl_mutex *mu, ble_npl_time_t timeout);

ble_npl_error_t npl_zephyr_mutex_release(struct ble_npl_mutex *mu);

ble_npl_error_t npl_zephyr_sem_init(struct ble_npl_sem *sem, uint16_t tokens);
ble_npl_error_t npl_zephyr_sem_deinit(struct ble_npl_sem *sem);

ble_npl_error_t npl_zephyr_sem_pend(struct ble_npl_sem *sem, ble_npl_time_t timeout);

ble_npl_error_t npl_zephyr_sem_release(struct ble_npl_sem *sem);

int npl_zephyr_callout_init(struct ble_npl_callout *co, struct ble_npl_eventq *evq,
			    ble_npl_event_fn *ev_cb, void *ev_arg);

void npl_zephyr_callout_deinit(struct ble_npl_callout *co);

void npl_zephyr_callout_stop(struct ble_npl_callout *co);

bool npl_zephyr_callout_is_active(struct ble_npl_callout *co);

ble_npl_time_t npl_zephyr_callout_get_ticks(struct ble_npl_callout *co);

ble_npl_error_t npl_zephyr_callout_reset(struct ble_npl_callout *co, ble_npl_time_t ticks);

ble_npl_time_t npl_zephyr_callout_remaining_ticks(struct ble_npl_callout *co, ble_npl_time_t now);

ble_npl_error_t npl_zephyr_time_ms_to_ticks(uint32_t ms, ble_npl_time_t *out_ticks);

ble_npl_error_t npl_zephyr_time_ticks_to_ms(ble_npl_time_t ticks, uint32_t *out_ms);

uint32_t npl_zephyr_hw_enter_critical(void);

void npl_zephyr_hw_exit_critical(uint32_t ctx);

#ifdef __cplusplus
}
#endif

#endif /* _NPL_ZEPHYR_H_ */
