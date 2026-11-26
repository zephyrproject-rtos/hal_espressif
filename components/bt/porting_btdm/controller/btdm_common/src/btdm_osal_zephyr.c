/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/dlist.h>
#include <string.h>

#include "btdm_osal_zephyr.h"
#include "btdm_user_cfg.h"
#include "btdm_mempool.h"

#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_intr_alloc.h"
#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "esp_private/critical_section.h"

#define BTDM_OSAL_ASSERT(con)                                                                      \
	do {                                                                                       \
		if (!(con)) {                                                                      \
			esp_rom_printf("assertion:%s\n", #con);                                    \
			esp_rom_printf("line:%d,function:%s\n", __LINE__, __func__);               \
			k_panic();                                                                 \
		}                                                                                  \
	} while (0)

#define BTDM_OSAL_SEM_MAX_COUNT 128

static struct btdm_mempool s_btdm_osal_ev_pool;
static btdm_membuf_t *s_btdm_osal_ev_buf;

static struct btdm_mempool s_btdm_osal_evq_pool;
static btdm_membuf_t *s_btdm_osal_evq_buf;

static struct btdm_mempool s_btdm_osal_co_pool;
static btdm_membuf_t *s_btdm_osal_co_buf;

static struct btdm_mempool s_btdm_osal_sem_pool;
static btdm_membuf_t *s_btdm_osal_sem_buf;

static struct btdm_mempool s_btdm_osal_mutex_pool;
static btdm_membuf_t *s_btdm_osal_mutex_buf;

/* Protects the event queue lists and the queued flags. */
static struct k_spinlock s_btdm_osal_evq_lock;

/* Controller critical section, recursive. */
static esp_os_spinlock_t s_btdm_osal_crit_lock = ESP_OS_SPINLOCK_INIT;

struct btdm_osal_ev_node {
	sys_dnode_t node;
	struct btdm_osal_event *ev;
};

/* One list node per controller event. */
static struct k_mem_slab s_btdm_osal_node_slab;
static void *s_btdm_osal_node_buf;

struct btdm_osal_evq_impl {
	sys_dlist_t list;
	struct k_sem items;
};

static inline bool btdm_osal_in_isr(void)
{
	return k_is_in_isr();
}

static inline k_timeout_t btdm_osal_ticks_to_timeout(btdm_osal_time_t ticks)
{
	if (ticks == UINT32_MAX) {
		return K_FOREVER;
	}

	return K_TICKS(ticks);
}

/* BTDM OSAL event queue */

static void btdm_osal_eventq_drain(struct btdm_osal_evq_impl *eventq)
{
	struct btdm_osal_event_zephyr *event;
	struct btdm_osal_ev_node *n;
	sys_dnode_t *dn, *dns;
	k_spinlock_key_t key;

	key = k_spin_lock(&s_btdm_osal_evq_lock);
	SYS_DLIST_FOR_EACH_NODE_SAFE(&eventq->list, dn, dns) {
		sys_dlist_remove(dn);
		n = CONTAINER_OF(dn, struct btdm_osal_ev_node, node);
		event = (struct btdm_osal_event_zephyr *)n->ev->event;
		if (event != NULL) {
			event->queued = false;
		}
		k_mem_slab_free(&s_btdm_osal_node_slab, (void *)n);
	}
	k_sem_reset(&eventq->items);
	k_spin_unlock(&s_btdm_osal_evq_lock, key);
}

void wr_btdm_osal_eventq_init(struct btdm_osal_eventq *evq)
{
	struct btdm_osal_evq_impl *eventq;

	if (btdm_memblock_from(&s_btdm_osal_evq_pool, evq->eventq) == 0) {
		evq->eventq = btdm_memblock_get(&s_btdm_osal_evq_pool);
		eventq = (struct btdm_osal_evq_impl *)evq->eventq;
		BTDM_OSAL_ASSERT(eventq != NULL);
		memset(eventq, 0, sizeof(*eventq));
		sys_dlist_init(&eventq->list);
		k_sem_init(&eventq->items, 0, K_SEM_MAX_LIMIT);
	} else {
		eventq = (struct btdm_osal_evq_impl *)evq->eventq;
		btdm_osal_eventq_drain(eventq);
	}
}

void wr_btdm_osal_eventq_deinit(struct btdm_osal_eventq *evq)
{
	struct btdm_osal_evq_impl *eventq = (struct btdm_osal_evq_impl *)evq->eventq;

	BTDM_OSAL_ASSERT(eventq != NULL);
	btdm_osal_eventq_drain(eventq);
	btdm_memblock_put(&s_btdm_osal_evq_pool, eventq);
	evq->eventq = NULL;
}

struct btdm_osal_event *IRAM_ATTR wr_btdm_osal_eventq_get(struct btdm_osal_eventq *evq,
							  btdm_osal_time_t tmo)
{
	struct btdm_osal_evq_impl *eventq = (struct btdm_osal_evq_impl *)evq->eventq;
	struct btdm_osal_event_zephyr *event;
	struct btdm_osal_event *ev = NULL;
	struct btdm_osal_ev_node *n;
	bool in_isr = btdm_osal_in_isr();
	k_timeout_t timeout;
	k_spinlock_key_t key;
	sys_dnode_t *dn;

	if (in_isr) {
		BTDM_OSAL_ASSERT(tmo == 0);
		timeout = K_NO_WAIT;
	} else {
		timeout = btdm_osal_ticks_to_timeout(tmo);
	}

	do {
		if (k_sem_take(&eventq->items, timeout) != 0) {
			if (!in_isr && (tmo == UINT32_MAX)) {
				continue;
			}
			return NULL;
		}

		key = k_spin_lock(&s_btdm_osal_evq_lock);
		dn = sys_dlist_get(&eventq->list);
		if (dn != NULL) {
			n = CONTAINER_OF(dn, struct btdm_osal_ev_node, node);
			ev = n->ev;
			event = (struct btdm_osal_event_zephyr *)ev->event;
			if (event != NULL) {
				event->queued = false;
			}
			k_mem_slab_free(&s_btdm_osal_node_slab, (void *)n);
		}
		k_spin_unlock(&s_btdm_osal_evq_lock, key);
	} while ((ev == NULL) && !in_isr && (tmo == UINT32_MAX));

	return ev;
}

static void IRAM_ATTR btdm_osal_eventq_put_common(struct btdm_osal_eventq *evq,
						  struct btdm_osal_event *ev, bool to_front)
{
	struct btdm_osal_evq_impl *eventq = (struct btdm_osal_evq_impl *)evq->eventq;
	struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;
	struct btdm_osal_ev_node *n;
	k_spinlock_key_t key;

	key = k_spin_lock(&s_btdm_osal_evq_lock);
	if (event->queued) {
		k_spin_unlock(&s_btdm_osal_evq_lock, key);
		return;
	}

	if (k_mem_slab_alloc(&s_btdm_osal_node_slab, (void **)&n, K_NO_WAIT) != 0) {
		k_spin_unlock(&s_btdm_osal_evq_lock, key);
		BTDM_OSAL_ASSERT(false);
		return;
	}

	n->ev = ev;
	event->queued = true;

	if (to_front) {
		sys_dlist_prepend(&eventq->list, &n->node);
	} else {
		sys_dlist_append(&eventq->list, &n->node);
	}
	k_spin_unlock(&s_btdm_osal_evq_lock, key);

	k_sem_give(&eventq->items);
}

void IRAM_ATTR wr_btdm_osal_eventq_put(struct btdm_osal_eventq *evq, struct btdm_osal_event *ev)
{
	btdm_osal_eventq_put_common(evq, ev, false);
}

void IRAM_ATTR wr_btdm_osal_eventq_put_to_front(struct btdm_osal_eventq *evq,
						struct btdm_osal_event *ev)
{
	btdm_osal_eventq_put_common(evq, ev, true);
}

void IRAM_ATTR wr_btdm_osal_eventq_remove(struct btdm_osal_eventq *evq, struct btdm_osal_event *ev)
{
	struct btdm_osal_evq_impl *eventq = (struct btdm_osal_evq_impl *)evq->eventq;
	struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;
	struct btdm_osal_ev_node *n;
	sys_dnode_t *dn, *dns;
	bool removed = false;
	k_spinlock_key_t key;

	key = k_spin_lock(&s_btdm_osal_evq_lock);
	if (!event->queued) {
		k_spin_unlock(&s_btdm_osal_evq_lock, key);
		return;
	}

	SYS_DLIST_FOR_EACH_NODE_SAFE(&eventq->list, dn, dns) {
		n = CONTAINER_OF(dn, struct btdm_osal_ev_node, node);
		if (n->ev == ev) {
			sys_dlist_remove(dn);
			k_mem_slab_free(&s_btdm_osal_node_slab, (void *)n);
			removed = true;
			break;
		}
	}

	event->queued = false;
	k_spin_unlock(&s_btdm_osal_evq_lock, key);

	if (removed) {
		(void)k_sem_take(&eventq->items, K_NO_WAIT);
	}
}

bool IRAM_ATTR wr_btdm_osal_eventq_is_empty(struct btdm_osal_eventq *evq)
{
	struct btdm_osal_evq_impl *eventq = (struct btdm_osal_evq_impl *)evq->eventq;
	k_spinlock_key_t key;
	bool empty;

	key = k_spin_lock(&s_btdm_osal_evq_lock);
	empty = sys_dlist_is_empty(&eventq->list);
	k_spin_unlock(&s_btdm_osal_evq_lock, key);

	return empty;
}

/* BTDM OSAL event */

void IRAM_ATTR wr_btdm_osal_event_run(struct btdm_osal_event *ev)
{
	struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;

	event->fn(ev);
}

void IRAM_ATTR wr_btdm_osal_event_init(struct btdm_osal_event *ev, btdm_osal_event_fn *fn,
				       void *arg)
{
	struct btdm_osal_event_zephyr *event;

	if (btdm_memblock_from(&s_btdm_osal_ev_pool, ev->event) == 0) {
		ev->event = btdm_memblock_get(&s_btdm_osal_ev_pool);
	}

	event = (struct btdm_osal_event_zephyr *)ev->event;
	BTDM_OSAL_ASSERT(event != NULL);

	memset(event, 0, sizeof(*event));
	event->fn = fn;
	event->arg = arg;
}

void IRAM_ATTR wr_btdm_osal_event_deinit(struct btdm_osal_event *ev)
{
	struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;

	if (event == NULL) {
		return;
	}

	btdm_memblock_put(&s_btdm_osal_ev_pool, event);
	ev->event = NULL;
}

void IRAM_ATTR wr_btdm_osal_event_reset(struct btdm_osal_event *ev)
{
	struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;

	BTDM_OSAL_ASSERT(event != NULL);
	event->queued = false;
}

bool IRAM_ATTR wr_btdm_osal_event_is_queued(struct btdm_osal_event *ev)
{
	struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;

	return event->queued;
}

void *IRAM_ATTR wr_btdm_osal_event_get_arg(struct btdm_osal_event *ev)
{
	struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;

	return event->arg;
}

void IRAM_ATTR wr_btdm_osal_event_set_arg(struct btdm_osal_event *ev, void *arg)
{
	struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)ev->event;

	event->arg = arg;
}

/* BTDM OSAL mutex */

btdm_osal_error_t wr_btdm_osal_mutex_init(struct btdm_osal_mutex *mu)
{
	struct btdm_osal_mutex_zephyr *mutex;

	if (mu == NULL) {
		return BTDM_OSAL_INVALID_PARM;
	}

	if (btdm_memblock_from(&s_btdm_osal_mutex_pool, mu->mutex) == 0) {
		mu->mutex = btdm_memblock_get(&s_btdm_osal_mutex_pool);
		mutex = (struct btdm_osal_mutex_zephyr *)mu->mutex;
		if (mutex == NULL) {
			return BTDM_OSAL_INVALID_PARM;
		}

		memset(mutex, 0, sizeof(*mutex));
		k_mutex_init(&mutex->handle);
	}

	return BTDM_OSAL_OK;
}

btdm_osal_error_t wr_btdm_osal_mutex_deinit(struct btdm_osal_mutex *mu)
{
	struct btdm_osal_mutex_zephyr *mutex = (struct btdm_osal_mutex_zephyr *)mu->mutex;

	if (mutex == NULL) {
		return BTDM_OSAL_INVALID_PARM;
	}

	btdm_memblock_put(&s_btdm_osal_mutex_pool, mutex);
	mu->mutex = NULL;

	return BTDM_OSAL_OK;
}

btdm_osal_error_t IRAM_ATTR wr_btdm_osal_mutex_pend(struct btdm_osal_mutex *mu,
						    btdm_osal_time_t timeout)
{
	struct btdm_osal_mutex_zephyr *mutex = (struct btdm_osal_mutex_zephyr *)mu->mutex;

	if (mutex == NULL) {
		return BTDM_OSAL_INVALID_PARM;
	}

	if (btdm_osal_in_isr()) {
		BTDM_OSAL_ASSERT(false);
		return BTDM_OSAL_ERR_IN_ISR;
	}

	if (k_mutex_lock(&mutex->handle, btdm_osal_ticks_to_timeout(timeout)) != 0) {
		return BTDM_OSAL_TIMEOUT;
	}

	return BTDM_OSAL_OK;
}

btdm_osal_error_t IRAM_ATTR wr_btdm_osal_mutex_release(struct btdm_osal_mutex *mu)
{
	struct btdm_osal_mutex_zephyr *mutex = (struct btdm_osal_mutex_zephyr *)mu->mutex;

	if (mutex == NULL) {
		return BTDM_OSAL_INVALID_PARM;
	}

	if (btdm_osal_in_isr()) {
		BTDM_OSAL_ASSERT(false);
		return BTDM_OSAL_ERR_IN_ISR;
	}

	if (k_mutex_unlock(&mutex->handle) != 0) {
		return BTDM_OSAL_BAD_MUTEX;
	}

	return BTDM_OSAL_OK;
}

/* BTDM OSAL semaphore */

btdm_osal_error_t wr_btdm_osal_sem_init(struct btdm_osal_sem *sem, uint16_t tokens)
{
	struct btdm_osal_sem_zephyr *semaphore;

	if (btdm_memblock_from(&s_btdm_osal_sem_pool, sem->sem) == 0) {
		sem->sem = btdm_memblock_get(&s_btdm_osal_sem_pool);
		semaphore = (struct btdm_osal_sem_zephyr *)sem->sem;
		if (semaphore == NULL) {
			return BTDM_OSAL_INVALID_PARM;
		}

		memset(semaphore, 0, sizeof(*semaphore));
		k_sem_init(&semaphore->handle, tokens, BTDM_OSAL_SEM_MAX_COUNT);
	}

	return BTDM_OSAL_OK;
}

btdm_osal_error_t wr_btdm_osal_sem_deinit(struct btdm_osal_sem *sem)
{
	struct btdm_osal_sem_zephyr *semaphore = (struct btdm_osal_sem_zephyr *)sem->sem;

	if (semaphore == NULL) {
		return BTDM_OSAL_INVALID_PARM;
	}

	btdm_memblock_put(&s_btdm_osal_sem_pool, semaphore);
	sem->sem = NULL;

	return BTDM_OSAL_OK;
}

btdm_osal_error_t IRAM_ATTR wr_btdm_osal_sem_pend(struct btdm_osal_sem *sem,
						  btdm_osal_time_t timeout)
{
	struct btdm_osal_sem_zephyr *semaphore = (struct btdm_osal_sem_zephyr *)sem->sem;

	if (semaphore == NULL) {
		return BTDM_OSAL_INVALID_PARM;
	}

	if (btdm_osal_in_isr()) {
		BTDM_OSAL_ASSERT(timeout == 0);
		if (k_sem_take(&semaphore->handle, K_NO_WAIT) != 0) {
			return BTDM_OSAL_TIMEOUT;
		}
		return BTDM_OSAL_OK;
	}

	if (k_sem_take(&semaphore->handle, btdm_osal_ticks_to_timeout(timeout)) != 0) {
		return BTDM_OSAL_TIMEOUT;
	}

	return BTDM_OSAL_OK;
}

btdm_osal_error_t IRAM_ATTR wr_btdm_osal_sem_release(struct btdm_osal_sem *sem)
{
	struct btdm_osal_sem_zephyr *semaphore = (struct btdm_osal_sem_zephyr *)sem->sem;

	if (semaphore == NULL) {
		return BTDM_OSAL_INVALID_PARM;
	}

	k_sem_give(&semaphore->handle);

	return BTDM_OSAL_OK;
}

uint16_t IRAM_ATTR wr_btdm_osal_sem_get_count(struct btdm_osal_sem *sem)
{
	struct btdm_osal_sem_zephyr *semaphore = (struct btdm_osal_sem_zephyr *)sem->sem;

	return (uint16_t)k_sem_count_get(&semaphore->handle);
}

/* BTDM OSAL callout */

static void btdm_osal_callout_work_handler(struct k_work *work)
{
	struct btdm_osal_callout_zephyr *callout =
		CONTAINER_OF(work, struct btdm_osal_callout_zephyr, work);
	struct btdm_osal_event_zephyr *event = (struct btdm_osal_event_zephyr *)callout->ev.event;

	if (event != NULL) {
		event->fn(&callout->ev);
	}
}

static void IRAM_ATTR btdm_osal_callout_expiry(struct k_timer *timer)
{
	struct btdm_osal_callout_zephyr *callout =
		CONTAINER_OF(timer, struct btdm_osal_callout_zephyr, timer);

	if (callout->evq != NULL) {
		btdm_osal_eventq_put(callout->evq, &callout->ev);
	} else {
		(void)k_work_submit(&callout->work);
	}
}

int wr_btdm_osal_callout_init(struct btdm_osal_callout *co, struct btdm_osal_eventq *evq,
			      btdm_osal_event_fn *ev_cb, void *ev_arg)
{
	struct btdm_osal_callout_zephyr *callout;

	if (btdm_memblock_from(&s_btdm_osal_co_pool, co->co) == 0) {
		co->co = btdm_memblock_get(&s_btdm_osal_co_pool);
		callout = (struct btdm_osal_callout_zephyr *)co->co;
		BTDM_OSAL_ASSERT(callout != NULL);

		memset(callout, 0, sizeof(*callout));
		btdm_osal_event_init(&callout->ev, ev_cb, ev_arg);
		callout->evq = evq;
		k_timer_init(&callout->timer, btdm_osal_callout_expiry, NULL);
		k_work_init(&callout->work, btdm_osal_callout_work_handler);
	} else {
		callout = (struct btdm_osal_callout_zephyr *)co->co;
		k_timer_stop(&callout->timer);
		(void)k_work_cancel(&callout->work);
		btdm_osal_event_init(&callout->ev, ev_cb, ev_arg);
		callout->evq = evq;
	}

	return 0;
}

void wr_btdm_osal_callout_deinit(struct btdm_osal_callout *co)
{
	struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;
	struct k_work_sync sync;

	if (callout == NULL) {
		return;
	}

	k_timer_stop(&callout->timer);
	if (btdm_osal_in_isr() || (k_current_get() == k_work_queue_thread_get(&k_sys_work_q))) {
		(void)k_work_cancel(&callout->work);
	} else {
		(void)k_work_cancel_sync(&callout->work, &sync);
	}

	btdm_osal_event_deinit(&callout->ev);
	btdm_memblock_put(&s_btdm_osal_co_pool, callout);
	co->co = NULL;
}

btdm_osal_error_t IRAM_ATTR wr_btdm_osal_callout_reset(struct btdm_osal_callout *co,
						       btdm_osal_time_t ticks)
{
	struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;

	if (callout == NULL) {
		return BTDM_OSAL_INVALID_PARM;
	}

	k_timer_stop(&callout->timer);
	(void)k_work_cancel(&callout->work);
	k_timer_start(&callout->timer, btdm_osal_ticks_to_timeout(ticks), K_NO_WAIT);

	return BTDM_OSAL_OK;
}

void wr_btdm_osal_callout_mem_reset(struct btdm_osal_callout *co)
{
	struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;

	BTDM_OSAL_ASSERT(callout != NULL);
	wr_btdm_osal_event_reset(&callout->ev);
}

void IRAM_ATTR wr_btdm_osal_callout_stop(struct btdm_osal_callout *co)
{
	struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;

	if (callout == NULL) {
		return;
	}

	k_timer_stop(&callout->timer);
	(void)k_work_cancel(&callout->work);
}

bool IRAM_ATTR wr_btdm_osal_callout_is_active(struct btdm_osal_callout *co)
{
	struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;

	return k_timer_remaining_ticks(&callout->timer) != 0;
}

btdm_osal_time_t wr_btdm_osal_callout_get_ticks(struct btdm_osal_callout *co)
{
	struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;

	return (btdm_osal_time_t)k_timer_expires_ticks(&callout->timer);
}

btdm_osal_time_t wr_btdm_osal_callout_remaining_ticks(struct btdm_osal_callout *co,
						      btdm_osal_time_t time)
{
	struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;
	btdm_osal_time_t exp = (btdm_osal_time_t)k_timer_expires_ticks(&callout->timer);

	if (exp > time) {
		return exp - time;
	}

	return 0;
}

void wr_btdm_osal_callout_set_arg(struct btdm_osal_callout *co, void *arg)
{
	struct btdm_osal_callout_zephyr *callout = (struct btdm_osal_callout_zephyr *)co->co;

	wr_btdm_osal_event_set_arg(&callout->ev, arg);
}

/* BTDM OSAL time, in kernel ticks */

btdm_osal_time_t IRAM_ATTR wr_btdm_osal_time_get(void)
{
	return (btdm_osal_time_t)k_uptime_ticks();
}

btdm_osal_error_t wr_btdm_osal_time_ms_to_ticks(uint32_t ms, btdm_osal_time_t *out_ticks)
{
	*out_ticks = (btdm_osal_time_t)k_ms_to_ticks_ceil32(ms);

	return BTDM_OSAL_OK;
}

btdm_osal_error_t wr_btdm_osal_time_ticks_to_ms(btdm_osal_time_t ticks, uint32_t *out_ms)
{
	*out_ms = k_ticks_to_ms_floor32(ticks);

	return BTDM_OSAL_OK;
}

btdm_osal_time_t IRAM_ATTR wr_btdm_osal_time_ms_to_ticks32(uint32_t ms)
{
	return (btdm_osal_time_t)k_ms_to_ticks_ceil32(ms);
}

uint32_t IRAM_ATTR wr_btdm_osal_time_ticks_to_ms32(btdm_osal_time_t ticks)
{
	return k_ticks_to_ms_floor32(ticks);
}

uint32_t IRAM_ATTR wr_btdm_osal_get_time_forever(void)
{
	return UINT32_MAX;
}

/* BTDM OSAL critical section */

uint32_t IRAM_ATTR wr_btdm_osal_hw_enter_critical(void)
{
	esp_os_enter_critical(&s_btdm_osal_crit_lock);

	return 0;
}

uint8_t IRAM_ATTR wr_btdm_osal_hw_is_in_critical(void)
{
	uint32_t depth;

	if (IS_ENABLED(CONFIG_SMP) && (atomic_get(&s_btdm_osal_crit_lock.owner_cpu) !=
				       (atomic_val_t)esp_cpu_get_core_id())) {
		return 0;
	}

	depth = s_btdm_osal_crit_lock.count;

	return (depth >= 0xFFU) ? 0xFFU : (uint8_t)depth;
}

void IRAM_ATTR wr_btdm_osal_hw_exit_critical(uint32_t ctx)
{
	ARG_UNUSED(ctx);

	esp_os_exit_critical(&s_btdm_osal_crit_lock);
}

/* BTDM OSAL task */

#define BTDM_OSAL_MAX_TASKS 1

static K_KERNEL_STACK_ARRAY_DEFINE(s_btdm_osal_stacks, BTDM_OSAL_MAX_TASKS,
				   CONFIG_ESP32_BT_CONTROLLER_STACK_SIZE);

static struct btdm_osal_task {
	struct k_thread thread;
	bool used;
} s_btdm_osal_tasks[BTDM_OSAL_MAX_TASKS];

int wr_btdm_osal_task_create(btdm_osal_task_fn *fn, const char *name, uint32_t stack_size,
			     void *arg, uint32_t priority, btdm_osal_task_handle_t *task_handle,
			     uint32_t core_id)
{
	struct btdm_osal_task *task = NULL;
	k_tid_t tid;
	int i;

	ARG_UNUSED(priority);
	ARG_UNUSED(core_id);

	for (i = 0; i < BTDM_OSAL_MAX_TASKS; i++) {
		if (!s_btdm_osal_tasks[i].used) {
			task = &s_btdm_osal_tasks[i];
			break;
		}
	}

	if (task == NULL) {
		return BTDM_OSAL_ENOMEM;
	}

	if (stack_size > K_KERNEL_STACK_SIZEOF(s_btdm_osal_stacks[i])) {
		esp_rom_printf("btdm task %s needs %u bytes of stack\n", name, stack_size);
		return BTDM_OSAL_ENOMEM;
	}

	task->used = true;

	tid = k_thread_create(&task->thread, s_btdm_osal_stacks[i],
			      K_KERNEL_STACK_SIZEOF(s_btdm_osal_stacks[i]), (k_thread_entry_t)fn,
			      arg, NULL, NULL, K_PRIO_COOP(CONFIG_ESP32_BT_CONTROLLER_TASK_PRIO), 0,
			      K_NO_WAIT);
	if (tid == NULL) {
		task->used = false;
		return BTDM_OSAL_ERROR;
	}

	k_thread_name_set(tid, name);

	if (task_handle != NULL) {
		*task_handle = (btdm_osal_task_handle_t)tid;
	}

	return BTDM_OSAL_OK;
}

void wr_btdm_osal_task_delete(btdm_osal_task_handle_t task_handle)
{
	k_tid_t tid = (k_tid_t)task_handle;
	int i;

	if (tid == NULL) {
		return;
	}

	for (i = 0; i < BTDM_OSAL_MAX_TASKS; i++) {
		if (s_btdm_osal_tasks[i].used && &s_btdm_osal_tasks[i].thread == tid) {
			s_btdm_osal_tasks[i].used = false;
			break;
		}
	}

	k_thread_abort(tid);
}

/* BTDM OSAL interrupt */

int wr_btdm_osal_intr_alloc(int src, int flags, btdm_osal_intr_fn *fn, void *arg,
			    btdm_osal_intr_handle_t *intr_handle)
{
	int alloc_flags = ESP_INTR_FLAG_LEVEL3;

	if (flags == 0) {
		alloc_flags |= ESP_INTR_FLAG_IRAM;
	}

	return esp_intr_alloc(src, alloc_flags, (intr_handler_t)fn, arg,
			      (intr_handle_t *)intr_handle);
}

int wr_btdm_osal_intr_free(btdm_osal_intr_handle_t intr_handle)
{
	return esp_intr_free((intr_handle_t)intr_handle);
}

/* BTDM OSAL memory */

void *IRAM_ATTR wr_btdm_osal_malloc(uint32_t size, btdm_osal_malloc_flag_t flags)
{
	ARG_UNUSED(flags);

	return k_malloc(size);
}

void IRAM_ATTR wr_btdm_osal_free(void *ptr)
{
	k_free(ptr);
}

/* BTDM OSAL misc */

int wr_btdm_osal_read_efuse_mac(uint8_t *mac)
{
	return esp_read_mac(mac, ESP_MAC_BT);
}

void wr_btdm_osal_srand(uint32_t seed)
{
	ARG_UNUSED(seed);
}

int wr_btdm_osal_rand(void)
{
	return (int)sys_rand32_get();
}

/* BTDM OSAL element mempools */

static int btdm_osal_pool_alloc(struct btdm_mempool *pool, btdm_membuf_t **buf, uint16_t count,
				uint32_t block_size, const char *name)
{
	if (count == 0) {
		return 0;
	}

	*buf = btdm_osal_malloc(BTDM_MEMPOOL_SIZE(count, block_size) * sizeof(btdm_membuf_t),
				BTDM_OSAL_MALLOC_F_INTERNAL);
	if (*buf == NULL) {
		return -1;
	}

	return btdm_mempool_init(pool, count, block_size, *buf, name);
}

int btdm_osal_elem_mempool_init(btdm_osal_elem_num_t *elem_num)
{
	if (btdm_osal_pool_alloc(&s_btdm_osal_ev_pool, &s_btdm_osal_ev_buf, elem_num->evt_count,
				 sizeof(struct btdm_osal_event_zephyr), "s_btdm_osal_ev_pool") != 0) {
		return -1;
	}

	if (elem_num->evt_count > 0) {
		s_btdm_osal_node_buf = btdm_osal_malloc(elem_num->evt_count *
							sizeof(struct btdm_osal_ev_node),
							BTDM_OSAL_MALLOC_F_INTERNAL);
		if (s_btdm_osal_node_buf == NULL) {
			return -2;
		}

		if (k_mem_slab_init(&s_btdm_osal_node_slab, s_btdm_osal_node_buf,
				    sizeof(struct btdm_osal_ev_node), elem_num->evt_count) != 0) {
			return -2;
		}
	}

	if (btdm_osal_pool_alloc(&s_btdm_osal_evq_pool, &s_btdm_osal_evq_buf, elem_num->evtq_count,
				 sizeof(struct btdm_osal_evq_impl), "s_btdm_osal_evq_pool") != 0) {
		return -3;
	}

	if (btdm_osal_pool_alloc(&s_btdm_osal_co_pool, &s_btdm_osal_co_buf, elem_num->co_count,
				 sizeof(struct btdm_osal_callout_zephyr), "s_btdm_osal_co_pool") != 0) {
		return -5;
	}

	if (btdm_osal_pool_alloc(&s_btdm_osal_sem_pool, &s_btdm_osal_sem_buf, elem_num->sem_count,
				 sizeof(struct btdm_osal_sem_zephyr), "s_btdm_osal_sem_pool") != 0) {
		return -7;
	}

	if (btdm_osal_pool_alloc(&s_btdm_osal_mutex_pool, &s_btdm_osal_mutex_buf,
				 elem_num->mutex_count, sizeof(struct btdm_osal_mutex_zephyr),
				 "s_btdm_osal_mutex_pool") != 0) {
		return -9;
	}

	return 0;
}

void btdm_osal_elem_mempool_deinit(void)
{
	btdm_osal_free(s_btdm_osal_ev_buf);
	s_btdm_osal_ev_buf = NULL;

	btdm_osal_free(s_btdm_osal_node_buf);
	s_btdm_osal_node_buf = NULL;

	BTDM_OSAL_ASSERT(s_btdm_osal_evq_pool.mp_num_free == s_btdm_osal_evq_pool.mp_num_blocks);
	btdm_osal_free(s_btdm_osal_evq_buf);
	s_btdm_osal_evq_buf = NULL;

	BTDM_OSAL_ASSERT(s_btdm_osal_co_pool.mp_num_free == s_btdm_osal_co_pool.mp_num_blocks);
	btdm_osal_free(s_btdm_osal_co_buf);
	s_btdm_osal_co_buf = NULL;

	btdm_osal_free(s_btdm_osal_sem_buf);
	s_btdm_osal_sem_buf = NULL;

	btdm_osal_free(s_btdm_osal_mutex_buf);
	s_btdm_osal_mutex_buf = NULL;
}
