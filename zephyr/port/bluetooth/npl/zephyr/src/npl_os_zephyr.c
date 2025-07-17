/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <stddef.h>
#include <string.h>

#include "nimble/nimble_npl.h"
#include "nimble/npl_zephyr.h"
#include <zephyr/kernel.h>
#include "os/os_mempool.h"
#include "soc/soc_caps.h"
#include "esp_bt.h"
#include "bt_osi_mem.h"

static unsigned int ble_port_lock_key;

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(esp32_ble_npl, CONFIG_LOG_DEFAULT_LEVEL);

#define BLE_HOST_CO_COUNT    (0)
#define BLE_HOST_EV_COUNT    (0)
#define BLE_HOST_EVQ_COUNT   (0)
#define BLE_HOST_SEM_COUNT   (0)
#define BLE_HOST_MUTEX_COUNT (0)

static struct ble_npl_event **evq_buffer;

struct os_mempool ble_zephyr_ev_pool;
static os_membuf_t *ble_zephyr_ev_buf = NULL;

struct os_mempool ble_zephyr_evq_pool;
static os_membuf_t *ble_zephyr_evq_buf = NULL;

struct os_mempool ble_zephyr_co_pool;
static os_membuf_t *ble_zephyr_co_buf = NULL;

struct os_mempool ble_zephyr_sem_pool;
static os_membuf_t *ble_zephyr_sem_buf = NULL;

struct os_mempool ble_zephyr_mutex_pool;
static os_membuf_t *ble_zephyr_mutex_buf = NULL;

static uint16_t ble_zephyr_total_event_cnt = 0;
static ble_npl_count_info_t g_ctrl_npl_info = {
	.co_count = 0,
	.evt_count = 0,
	.evtq_count = 0,
	.mutex_count = 0,
	.sem_count = 0,
};

bool IRAM_ATTR npl_zephyr_os_started(void)
{
	return (k_is_pre_kernel() == false);
}

void *IRAM_ATTR npl_zephyr_get_current_task_id(void)
{
	return k_current_get();
}

void IRAM_ATTR npl_zephyr_event_init(struct ble_npl_event *ev, ble_npl_event_fn *fn, void *arg)
{
	struct ble_npl_event_zephyr *event = NULL;
	if (!ev->event) {
		ev->event = bt_osi_mem_malloc_internal(sizeof(struct ble_npl_event_zephyr));
	}
	event = (struct ble_npl_event_zephyr *)ev->event;
	BLE_LL_ASSERT(event);

	memset(event, 0, sizeof(*event));
	event->fn = fn;
	event->arg = arg;
}

void IRAM_ATTR npl_zephyr_event_deinit(struct ble_npl_event *ev)
{
	BLE_LL_ASSERT(ev->event);
	bt_osi_mem_free(ev->event);
	ev->event = NULL;
}

void IRAM_ATTR npl_zephyr_event_reset(struct ble_npl_event *ev)
{
	struct ble_npl_event_zephyr *event = (struct ble_npl_event_zephyr *)ev->event;
	BLE_LL_ASSERT(event);
	event->queued = 0;
}

void npl_zephyr_eventq_init(struct ble_npl_eventq *evq)
{
	struct ble_npl_eventq_zephyr *eventq = NULL;
	if (!evq->eventq) {
		evq->eventq = bt_osi_mem_malloc_internal(sizeof(struct ble_npl_eventq_zephyr));
		eventq = (struct ble_npl_eventq_zephyr *)evq->eventq;
		BLE_LL_ASSERT(eventq);
		memset(eventq, 0, sizeof(*eventq));
		evq_buffer = bt_osi_mem_malloc_internal(
			ble_zephyr_total_event_cnt * sizeof(struct ble_npl_event *));
		BLE_LL_ASSERT(evq_buffer);
		k_msgq_init(&eventq->q, (char *)evq_buffer, sizeof(struct ble_npl_event *),
			    ble_zephyr_total_event_cnt);
	} else {
		eventq = (struct ble_npl_eventq_zephyr *)evq->eventq;
		k_msgq_purge(&eventq->q);
	}
}

void npl_zephyr_eventq_deinit(struct ble_npl_eventq *evq)
{
	struct ble_npl_eventq_zephyr *eventq = (struct ble_npl_eventq_zephyr *)evq->eventq;

	if (!eventq) {
		return;
	}
	k_free(eventq);
	k_free(evq_buffer);
	evq->eventq = NULL;
}

void IRAM_ATTR npl_zephyr_callout_mem_reset(struct ble_npl_callout *co)
{
	struct ble_npl_callout_zephyr *callout = (struct ble_npl_callout_zephyr *)co->co;

	BLE_LL_ASSERT(callout);
	BLE_LL_ASSERT(callout->handle);

	ble_npl_event_reset(&callout->ev);
}

static inline bool IRAM_ATTR in_isr(void)
{
	/* XXX hw specific! */
	return k_is_in_isr();
}

struct ble_npl_event *IRAM_ATTR npl_zephyr_eventq_get(struct ble_npl_eventq *evq,
						      ble_npl_time_t tmo)
{
	struct ble_npl_event *ev = NULL;
	struct ble_npl_eventq_zephyr *eventq = (struct ble_npl_eventq_zephyr *)evq->eventq;
	int rc;

	if (in_isr()) {
		BLE_LL_ASSERT(tmo == 0);
		rc = k_msgq_get(&eventq->q, &ev, K_NO_WAIT);
	} else {
		rc = k_msgq_get(&eventq->q, &ev, K_MSEC(tmo));
	}
	BLE_LL_ASSERT(rc == 0 || rc == -ENOMSG);

	if (ev) {
		struct ble_npl_event_zephyr *event = (struct ble_npl_event_zephyr *)ev->event;
		if (event) {
			event->queued = false;
		}
	}

	return ev;
}

void IRAM_ATTR npl_zephyr_eventq_put(struct ble_npl_eventq *evq, struct ble_npl_event *ev)
{
	struct ble_npl_eventq_zephyr *eventq = (struct ble_npl_eventq_zephyr *)evq->eventq;
	struct ble_npl_event_zephyr *event = (struct ble_npl_event_zephyr *)ev->event;
	int ret;

	if (event->queued) {
		return;
	}

	event->queued = true;

	if (in_isr()) {
		ret = k_msgq_put(&eventq->q, &ev, K_NO_WAIT);
	} else {
		ret = k_msgq_put(&eventq->q, &ev, K_FOREVER);
	}

	BLE_LL_ASSERT(ret == 0);
}

void IRAM_ATTR npl_zephyr_eventq_put_to_front(struct ble_npl_eventq *evq, struct ble_npl_event *ev)
{
	struct ble_npl_eventq_zephyr *eventq = (struct ble_npl_eventq_zephyr *)evq->eventq;
	struct ble_npl_event_zephyr *event = (struct ble_npl_event_zephyr *)ev->event;
	int ret;

	if (event->queued) {
		return;
	}

	event->queued = true;

	if (in_isr()) {
		ret = k_msgq_put(&eventq->q, &ev, K_NO_WAIT);
	} else {
		ret = k_msgq_put(&eventq->q, &ev, K_FOREVER);
	}

	BLE_LL_ASSERT(ret == 0);
}

void IRAM_ATTR npl_zephyr_eventq_remove(struct ble_npl_eventq *evq, struct ble_npl_event *ev)
{
	struct ble_npl_eventq_zephyr *eventq = evq->eventq;
	struct ble_npl_event_zephyr *e = ev->event;
	struct ble_npl_event *tmp;
	struct ble_npl_event **buf;
	uint32_t total;
	uint32_t kept = 0;
	uint32_t i;

	/* If it was never queued, nothing to do */
	if (!e->queued) {
		return;
	}

	/* Must not be called from ISR */
	BLE_LL_ASSERT(!in_isr());

	/* How many messages are currently in the queue */
	total = k_msgq_num_used_get(&eventq->q);
	if (total == 0) {
		e->queued = false;
		return;
	}

	/* Allocate a temporary buffer to hold pointers */
	buf = bt_osi_mem_malloc_internal(total * sizeof(*buf));
	BLE_LL_ASSERT(buf);

	/* Drain everything out */
	for (i = 0; i < total; i++) {
		/* non-blocking: we know there are 'total' items */
		(void)k_msgq_get(&eventq->q, &tmp, K_NO_WAIT);
		if (tmp != ev) {
			buf[kept++] = tmp;
		}
	}

	/* Re-queue only the kept ones */
	for (i = 0; i < kept; i++) {
		/* this cannot fail unless out of mem, which shouldn't happen */
		(void)k_msgq_put(&eventq->q, &buf[i], K_FOREVER);
	}

	/* Done — free temp storage and clear the flag */
	bt_osi_mem_free(buf);
	e->queued = false;
}

ble_npl_error_t npl_zephyr_mutex_init(struct ble_npl_mutex *mu)
{
	struct ble_npl_mutex_zephyr *mutex = mu->mutex;

	if (!mutex) {
		mu->mutex = bt_osi_mem_malloc_internal(sizeof(struct ble_npl_mutex_zephyr));
		mutex = mu->mutex;
		BLE_LL_ASSERT(mutex);
		k_mutex_init(&mutex->handle);
	}

	return BLE_NPL_OK;
}

ble_npl_error_t npl_zephyr_mutex_deinit(struct ble_npl_mutex *mu)
{
	struct ble_npl_mutex_zephyr *mutex = (struct ble_npl_mutex_zephyr *)mu->mutex;

	if (!mutex) {
		return BLE_NPL_INVALID_PARAM;
	}

	bt_osi_mem_free((void *)mutex);
	mu->mutex = NULL;

	return BLE_NPL_OK;
}

void IRAM_ATTR npl_zephyr_event_run(struct ble_npl_event *ev)
{
	struct ble_npl_event_zephyr *event = (struct ble_npl_event_zephyr *)ev->event;
	event->fn(ev);
}

bool IRAM_ATTR npl_zephyr_eventq_is_empty(struct ble_npl_eventq *evq)
{
	struct ble_npl_eventq_zephyr *eventq = evq->eventq;
	return (k_msgq_num_used_get(&eventq->q) == 0);
}

bool IRAM_ATTR npl_zephyr_event_is_queued(struct ble_npl_event *ev)
{
	struct ble_npl_event_zephyr *event = (struct ble_npl_event_zephyr *)ev->event;
	return event->queued;
}

void *IRAM_ATTR npl_zephyr_event_get_arg(struct ble_npl_event *ev)
{
	struct ble_npl_event_zephyr *event = (struct ble_npl_event_zephyr *)ev->event;
	return event->arg;
}

void IRAM_ATTR npl_zephyr_event_set_arg(struct ble_npl_event *ev, void *arg)
{
	struct ble_npl_event_zephyr *event = (struct ble_npl_event_zephyr *)ev->event;
	event->arg = arg;
}

ble_npl_error_t IRAM_ATTR npl_zephyr_mutex_pend(struct ble_npl_mutex *mu, ble_npl_time_t timeout)
{
	struct ble_npl_mutex_zephyr *mutex = mu->mutex;
	int rc;

	if (!mutex) {
		return BLE_NPL_INVALID_PARAM;
	}

	if (in_isr()) {
		BLE_LL_ASSERT(0);
	}

	rc = k_mutex_lock(&mutex->handle, K_MSEC(timeout));

	return rc == 0 ? BLE_NPL_OK : BLE_NPL_TIMEOUT;
}

ble_npl_error_t IRAM_ATTR npl_zephyr_mutex_release(struct ble_npl_mutex *mu)
{
	struct ble_npl_mutex_zephyr *mutex = mu->mutex;

	if (!mutex) {
		return BLE_NPL_INVALID_PARAM;
	}

	if (in_isr()) {
		BLE_LL_ASSERT(0);
	}

	k_mutex_unlock(&mutex->handle);

	return BLE_NPL_OK;
}

ble_npl_error_t npl_zephyr_sem_init(struct ble_npl_sem *sem, uint16_t tokens)
{
	struct ble_npl_sem_zephyr *semaphore = sem->sem;

	if (!semaphore) {
		sem->sem = bt_osi_mem_malloc_internal(sizeof(struct ble_npl_sem_zephyr));
		semaphore = sem->sem;
		BLE_LL_ASSERT(semaphore);
		k_sem_init(&semaphore->handle, tokens, 128);
	}

	return BLE_NPL_OK;
}

ble_npl_error_t npl_zephyr_sem_deinit(struct ble_npl_sem *sem)
{
	struct ble_npl_sem_zephyr *semaphore = (struct ble_npl_sem_zephyr *)sem->sem;

	if (!semaphore) {
		return BLE_NPL_INVALID_PARAM;
	}

	bt_osi_mem_free((void *)semaphore);

	sem->sem = NULL;

	return BLE_NPL_OK;
}

ble_npl_error_t IRAM_ATTR npl_zephyr_sem_pend(struct ble_npl_sem *sem, ble_npl_time_t timeout)
{
	struct ble_npl_sem_zephyr *semaphore = sem->sem;
	int rc;

	if (!semaphore) {
		return BLE_NPL_INVALID_PARAM;
	}

	if (in_isr()) {
		BLE_LL_ASSERT(timeout == 0);
		rc = k_sem_take(&semaphore->handle, K_NO_WAIT);
	} else {
		rc = k_sem_take(&semaphore->handle, K_MSEC(timeout));
	}

	return rc == 0 ? BLE_NPL_OK : BLE_NPL_TIMEOUT;
}

ble_npl_error_t IRAM_ATTR npl_zephyr_sem_release(struct ble_npl_sem *sem)
{
	struct ble_npl_sem_zephyr *semaphore = sem->sem;

	if (!semaphore) {
		return BLE_NPL_INVALID_PARAM;
	}

	k_sem_give(&semaphore->handle);

	return BLE_NPL_OK;
}

static void IRAM_ATTR ble_npl_event_fn_wrapper(void *arg)
{
	struct ble_npl_callout_zephyr *callout = (struct ble_npl_callout_zephyr *)arg;
	BLE_LL_ASSERT(callout);

	if (callout->evq) {
		ble_npl_eventq_put(callout->evq, &callout->ev);
	} else {
		struct ble_npl_event_zephyr *event =
			(struct ble_npl_event_zephyr *)callout->ev.event;
		event->fn(&callout->ev);
	}
}

static IRAM_ATTR ble_npl_error_t esp_err_to_npl_error(esp_err_t err)
{
	switch (err) {
	case ESP_ERR_INVALID_ARG:
		return BLE_NPL_INVALID_PARAM;

	case ESP_ERR_INVALID_STATE:
		return BLE_NPL_EINVAL;

	case ESP_OK:
		return BLE_NPL_OK;

	default:
		return BLE_NPL_ERROR;
	}
}

int npl_zephyr_callout_init(struct ble_npl_callout *co, struct ble_npl_eventq *evq,
			    ble_npl_event_fn *ev_cb, void *ev_arg)
{
	struct ble_npl_callout_zephyr *callout = NULL;

	if (!co->co) {
		co->co = bt_osi_mem_malloc_internal(sizeof(struct ble_npl_callout_zephyr));
		callout = (struct ble_npl_callout_zephyr *)co->co;
		if (!callout) {
			return -1;
		}

		memset(callout, 0, sizeof(*callout));
		ble_npl_event_init(&callout->ev, ev_cb, ev_arg);

		callout->evq = evq;

		esp_timer_create_args_t create_args = {.callback = ble_npl_event_fn_wrapper,
						       .arg = callout,
						       .name = "ble_timer"};

		if (esp_timer_create(&create_args, &callout->handle) != ESP_OK) {
			ble_npl_event_deinit(&callout->ev);
			bt_osi_mem_free((void *)callout);
			co->co = NULL;
			return -1;
		}
	} else {
		callout = (struct ble_npl_callout_zephyr *)co->co;
		BLE_LL_ASSERT(callout);
		callout->evq = evq;
		ble_npl_event_init(&callout->ev, ev_cb, ev_arg);
	}

	return 0;
}

void npl_zephyr_callout_deinit(struct ble_npl_callout *co)
{
	struct ble_npl_callout_zephyr *callout = (struct ble_npl_callout_zephyr *)co->co;

	/* Since we dynamically deinit timers, function can be called for NULL timers. Return for
	 * such scenarios */
	if (!callout) {
		return;
	}

	if (!callout->handle) {
		return;
	}

	ble_npl_event_deinit(&callout->ev);
	esp_err_t err = esp_timer_stop(callout->handle);
	if (err != ESP_OK) {
		if (err != ESP_ERR_INVALID_STATE) {
			LOG_DBG("Timer not stopped");
		}
	}
	err = esp_timer_delete(callout->handle);
	if (err != ESP_OK) {
		LOG_WRN("Timer not deleted");
	}

	bt_osi_mem_free((void *)callout);

	co->co = NULL;
	memset(co, 0, sizeof(struct ble_npl_callout));
}

uint16_t IRAM_ATTR npl_zephyr_sem_get_count(struct ble_npl_sem *sem)
{
	struct ble_npl_sem_zephyr *semaphore = sem->sem;
	return k_sem_count_get(&semaphore->handle);
}

ble_npl_error_t IRAM_ATTR npl_zephyr_callout_reset(struct ble_npl_callout *co, ble_npl_time_t ticks)
{
	struct ble_npl_callout_zephyr *callout = (struct ble_npl_callout_zephyr *)co->co;
	esp_timer_stop(callout->handle);

	return esp_err_to_npl_error(esp_timer_start_once(callout->handle, ticks * 1000));
}

void IRAM_ATTR npl_zephyr_callout_stop(struct ble_npl_callout *co)
{
	struct ble_npl_callout_zephyr *callout = (struct ble_npl_callout_zephyr *)co->co;

	if (!callout) {
		return;
	}

	esp_timer_stop(callout->handle);
}

bool IRAM_ATTR npl_zephyr_callout_is_active(struct ble_npl_callout *co)
{
	struct ble_npl_callout_zephyr *callout = (struct ble_npl_callout_zephyr *)co->co;
	return esp_timer_is_active(callout->handle);
}

ble_npl_time_t IRAM_ATTR npl_zephyr_callout_get_ticks(struct ble_npl_callout *co)
{
	uint32_t exp = 0;

	uint64_t expiry = 0;
	esp_err_t err;

	struct ble_npl_callout_zephyr *callout = (struct ble_npl_callout_zephyr *)co->co;

	// Fetch expiry time in microseconds
	err = esp_timer_get_expiry_time((esp_timer_handle_t)(callout->handle), &expiry);
	if (err != ESP_OK) {
		// Error. Could not fetch the expiry time
		return 0;
	}

	// Convert microseconds to ticks
	npl_zephyr_time_ms_to_ticks((uint32_t)(expiry / 1000), &exp);

	return exp;
}

ble_npl_time_t IRAM_ATTR npl_zephyr_callout_remaining_ticks(struct ble_npl_callout *co,
							    ble_npl_time_t now)
{
	ble_npl_time_t rt;
	uint32_t exp = 0;

	struct ble_npl_callout_zephyr *callout = (struct ble_npl_callout_zephyr *)co->co;

	uint64_t expiry = 0;
	esp_err_t err;

	// Fetch expiry time in microseconds
	err = esp_timer_get_expiry_time((esp_timer_handle_t)(callout->handle), &expiry);
	if (err != ESP_OK) {
		// Error. Could not fetch the expiry time
		return 0;
	}

	// Convert microseconds to ticks
	npl_zephyr_time_ms_to_ticks((uint32_t)(expiry / 1000), &exp);

	if (exp > now) {
		rt = exp - now;
	} else {
		rt = 0;
	}

	return rt;
}

void IRAM_ATTR npl_zephyr_callout_set_arg(struct ble_npl_callout *co, void *arg)
{
	struct ble_npl_callout_zephyr *callout = (struct ble_npl_callout_zephyr *)co->co;
	struct ble_npl_event_zephyr *event = (struct ble_npl_event_zephyr *)callout->ev.event;
	event->arg = arg;
}

uint32_t IRAM_ATTR npl_zephyr_time_get(void)
{
	return esp_timer_get_time() / 1000;
}

ble_npl_error_t IRAM_ATTR npl_zephyr_time_ms_to_ticks(uint32_t ms, ble_npl_time_t *out_ticks)
{
	uint64_t ticks;

	ticks = (uint64_t)ms;

	if (ticks > UINT32_MAX) {
		return BLE_NPL_EINVAL;
	}

	*out_ticks = ticks;

	return 0;
}

ble_npl_error_t IRAM_ATTR npl_zephyr_time_ticks_to_ms(ble_npl_time_t ticks, uint32_t *out_ms)
{
	uint64_t ms;
	ms = ((uint64_t)ticks);

	if (ms > UINT32_MAX) {
		return BLE_NPL_EINVAL;
	}

	*out_ms = ms;

	return 0;
}

ble_npl_time_t IRAM_ATTR npl_zephyr_time_ms_to_ticks32(uint32_t ms)
{
	return ms;
}

uint32_t IRAM_ATTR npl_zephyr_time_ticks_to_ms32(ble_npl_time_t ticks)
{
	return ticks;
}

void IRAM_ATTR npl_zephyr_time_delay(ble_npl_time_t ticks)
{
	k_sleep(K_TICKS(ticks));
}

uint8_t hw_critical_state_status = 0;

uint32_t IRAM_ATTR npl_zephyr_hw_enter_critical(void)
{
	if (hw_critical_state_status == 0) {
		ble_port_lock_key = irq_lock();
	}

	if (hw_critical_state_status < 0xFF) {
		hw_critical_state_status++;
	}

	return 0;
}

uint8_t IRAM_ATTR npl_zephyr_hw_is_in_critical(void)
{
	return hw_critical_state_status;
}

void IRAM_ATTR npl_zephyr_hw_exit_critical(uint32_t ctx)
{
	if (hw_critical_state_status > 0) {
		hw_critical_state_status--;
	}

	if (hw_critical_state_status == 0) {
		irq_unlock(ble_port_lock_key);
	}
}

uint32_t IRAM_ATTR npl_zephyr_get_time_forever(void)
{
	return 0xFFFFFFFF;
}

const struct npl_funcs_t npl_funcs_ro = {
	.p_ble_npl_os_started = npl_zephyr_os_started,
	.p_ble_npl_get_current_task_id = npl_zephyr_get_current_task_id,
	.p_ble_npl_eventq_init = npl_zephyr_eventq_init,
	.p_ble_npl_eventq_deinit = npl_zephyr_eventq_deinit,
	.p_ble_npl_eventq_get = npl_zephyr_eventq_get,
	.p_ble_npl_eventq_put = npl_zephyr_eventq_put,
	.p_ble_npl_eventq_put_to_front = npl_zephyr_eventq_put_to_front,
	.p_ble_npl_eventq_remove = npl_zephyr_eventq_remove,
	.p_ble_npl_event_run = npl_zephyr_event_run,
	.p_ble_npl_eventq_is_empty = npl_zephyr_eventq_is_empty,
	.p_ble_npl_event_init = npl_zephyr_event_init,
	.p_ble_npl_event_deinit = npl_zephyr_event_deinit,
	.p_ble_npl_event_reset = npl_zephyr_event_reset,
	.p_ble_npl_event_is_queued = npl_zephyr_event_is_queued,
	.p_ble_npl_event_get_arg = npl_zephyr_event_get_arg,
	.p_ble_npl_event_set_arg = npl_zephyr_event_set_arg,
	.p_ble_npl_mutex_init = npl_zephyr_mutex_init,
	.p_ble_npl_mutex_deinit = npl_zephyr_mutex_deinit,
	.p_ble_npl_mutex_pend = npl_zephyr_mutex_pend,
	.p_ble_npl_mutex_release = npl_zephyr_mutex_release,
	.p_ble_npl_sem_init = npl_zephyr_sem_init,
	.p_ble_npl_sem_deinit = npl_zephyr_sem_deinit,
	.p_ble_npl_sem_pend = npl_zephyr_sem_pend,
	.p_ble_npl_sem_release = npl_zephyr_sem_release,
	.p_ble_npl_sem_get_count = npl_zephyr_sem_get_count,
	.p_ble_npl_callout_init = npl_zephyr_callout_init,
	.p_ble_npl_callout_reset = npl_zephyr_callout_reset,
	.p_ble_npl_callout_stop = npl_zephyr_callout_stop,
	.p_ble_npl_callout_deinit = npl_zephyr_callout_deinit,
	.p_ble_npl_callout_mem_reset = npl_zephyr_callout_mem_reset,
	.p_ble_npl_callout_is_active = npl_zephyr_callout_is_active,
	.p_ble_npl_callout_get_ticks = npl_zephyr_callout_get_ticks,
	.p_ble_npl_callout_remaining_ticks = npl_zephyr_callout_remaining_ticks,
	.p_ble_npl_callout_set_arg = npl_zephyr_callout_set_arg,
	.p_ble_npl_time_get = npl_zephyr_time_get,
	.p_ble_npl_time_ms_to_ticks = npl_zephyr_time_ms_to_ticks,
	.p_ble_npl_time_ticks_to_ms = npl_zephyr_time_ticks_to_ms,
	.p_ble_npl_time_ms_to_ticks32 = npl_zephyr_time_ms_to_ticks32,
	.p_ble_npl_time_ticks_to_ms32 = npl_zephyr_time_ticks_to_ms32,
	.p_ble_npl_time_delay = npl_zephyr_time_delay,
#if NIMBLE_CFG_CONTROLLER || CONFIG_NIMBLE_CONTROLLER_MODE
	.p_ble_npl_hw_set_isr = NULL,
#endif
	.p_ble_npl_hw_enter_critical = npl_zephyr_hw_enter_critical,
	.p_ble_npl_hw_exit_critical = npl_zephyr_hw_exit_critical,
	.p_ble_npl_get_time_forever = npl_zephyr_get_time_forever,
	.p_ble_npl_hw_is_in_critical = npl_zephyr_hw_is_in_critical
};

struct npl_funcs_t *npl_funcs = NULL;

struct npl_funcs_t *npl_zephyr_funcs_get(void)
{
	return npl_funcs;
}

void npl_zephyr_funcs_init(void)
{
	npl_funcs = (struct npl_funcs_t *)bt_osi_mem_malloc_internal(sizeof(struct npl_funcs_t));
	if (!npl_funcs) {
		LOG_INF("npl funcs init failed");
		assert(0);
	}
	memcpy(npl_funcs, &npl_funcs_ro, sizeof(struct npl_funcs_t));
}

int npl_zephyr_set_controller_npl_info(ble_npl_count_info_t *ctrl_npl_info)
{
	if (!ctrl_npl_info) {
		return -1;
	}

	memcpy(&g_ctrl_npl_info, ctrl_npl_info, sizeof(ble_npl_count_info_t));
	return 0;
}

int npl_zephyr_mempool_init(void)
{
	int rc = -1;
	uint16_t ble_total_evt_count = 0;
	uint16_t ble_total_co_count = 0;
	uint16_t ble_total_evtq_count = 0;
	uint16_t ble_total_sem_count = 0;
	uint16_t ble_total_mutex_count = 0;
	ble_total_evt_count = g_ctrl_npl_info.evt_count + BLE_HOST_EV_COUNT;
	ble_total_evtq_count = g_ctrl_npl_info.evtq_count + BLE_HOST_EVQ_COUNT;
	ble_total_co_count = g_ctrl_npl_info.co_count + BLE_HOST_CO_COUNT;
	ble_total_sem_count = g_ctrl_npl_info.sem_count + BLE_HOST_SEM_COUNT;
	ble_total_mutex_count = g_ctrl_npl_info.mutex_count + BLE_HOST_MUTEX_COUNT;
	ble_zephyr_total_event_cnt = ble_total_evt_count;

	if (ble_total_evt_count) {
		ble_zephyr_ev_buf = bt_osi_mem_malloc_internal(
			OS_MEMPOOL_SIZE(ble_total_evt_count, sizeof(struct ble_npl_event_zephyr)) *
			sizeof(os_membuf_t));
		if (!ble_zephyr_ev_buf) {
			goto _error;
		}
		rc = os_mempool_init(&ble_zephyr_ev_pool, ble_total_evt_count,
				     sizeof(struct ble_npl_event_zephyr), ble_zephyr_ev_buf,
				     "ble_zephyr_ev_pool");
		if (rc) {
			goto _error;
		}
	}

	if (ble_total_evtq_count) {
		ble_zephyr_evq_buf =
			bt_osi_mem_malloc_internal(OS_MEMPOOL_SIZE(ble_total_evtq_count,
						 sizeof(struct ble_npl_eventq_zephyr)) *
				 sizeof(os_membuf_t));
		if (!ble_zephyr_evq_buf) {
			goto _error;
		}
		rc = os_mempool_init(&ble_zephyr_evq_pool, ble_total_evtq_count,
				     sizeof(struct ble_npl_eventq_zephyr), ble_zephyr_evq_buf,
				     "ble_zephyr_evq_pool");
		if (rc) {
			goto _error;
		}
	}

	if (ble_total_co_count) {
		ble_zephyr_co_buf = bt_osi_mem_malloc_internal(
			OS_MEMPOOL_SIZE(ble_total_co_count, sizeof(struct ble_npl_callout_zephyr)) *
			sizeof(os_membuf_t));
		if (!ble_zephyr_co_buf) {
			goto _error;
		}
		rc = os_mempool_init(&ble_zephyr_co_pool, ble_total_co_count,
				     sizeof(struct ble_npl_callout_zephyr), ble_zephyr_co_buf,
				     "ble_zephyr_co_pool");
		if (rc) {
			goto _error;
		}
	}

	if (ble_total_sem_count) {
		ble_zephyr_sem_buf = bt_osi_mem_malloc_internal(
			OS_MEMPOOL_SIZE(ble_total_sem_count, sizeof(struct ble_npl_sem_zephyr)) *
			sizeof(os_membuf_t));
		if (!ble_zephyr_sem_buf) {
			goto _error;
		}
		rc = os_mempool_init(&ble_zephyr_sem_pool, ble_total_sem_count,
				     sizeof(struct ble_npl_sem_zephyr), ble_zephyr_sem_buf,
				     "ble_zephyr_sem_pool");
		if (rc) {
			goto _error;
		}
	}

	if (ble_total_mutex_count) {
		ble_zephyr_mutex_buf =
			bt_osi_mem_malloc_internal(OS_MEMPOOL_SIZE(ble_total_mutex_count,
						 sizeof(struct ble_npl_mutex_zephyr)) *
				 sizeof(os_membuf_t));
		if (!ble_zephyr_mutex_buf) {
			goto _error;
		}
		rc = os_mempool_init(&ble_zephyr_mutex_pool, ble_total_mutex_count,
				     sizeof(struct ble_npl_mutex_zephyr), ble_zephyr_mutex_buf,
				     "ble_zephyr_mutex_pool");
		if (rc) {
			goto _error;
		}
	}

	return 0;
_error:
	if (ble_zephyr_ev_buf) {
		bt_osi_mem_free(ble_zephyr_ev_buf);
		ble_zephyr_ev_buf = NULL;
	}

	if (ble_zephyr_evq_buf) {
		bt_osi_mem_free(ble_zephyr_evq_buf);
		ble_zephyr_evq_buf = NULL;
	}

	if (ble_zephyr_co_buf) {
		bt_osi_mem_free(ble_zephyr_co_buf);
		ble_zephyr_co_buf = NULL;
	}

	if (ble_zephyr_sem_buf) {
		bt_osi_mem_free(ble_zephyr_sem_buf);
		ble_zephyr_sem_buf = NULL;
	}

	if (ble_zephyr_mutex_buf) {
		bt_osi_mem_free(ble_zephyr_mutex_buf);
		ble_zephyr_mutex_buf = NULL;
	}
	return -1;
}

void npl_zephyr_mempool_deinit(void)
{
	if (ble_zephyr_ev_buf) {
		bt_osi_mem_free(ble_zephyr_ev_buf);
		ble_zephyr_ev_buf = NULL;
	}
	if (ble_zephyr_evq_buf) {
		bt_osi_mem_free(ble_zephyr_evq_buf);
		ble_zephyr_evq_buf = NULL;
	}
	if (ble_zephyr_co_buf) {
		bt_osi_mem_free(ble_zephyr_co_buf);
		ble_zephyr_co_buf = NULL;
	}
	if (ble_zephyr_sem_buf) {
		bt_osi_mem_free(ble_zephyr_sem_buf);
		ble_zephyr_sem_buf = NULL;
	}
	if (ble_zephyr_mutex_buf) {
		bt_osi_mem_free(ble_zephyr_mutex_buf);
		ble_zephyr_mutex_buf = NULL;
	}
}

void npl_zephyr_funcs_deinit(void)
{
	if (npl_funcs) {
		bt_osi_mem_free(npl_funcs);
	}
	npl_funcs = NULL;
}
