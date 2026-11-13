/*
 * SPDX-FileCopyrightText: 2017-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <stdarg.h>
#include <string.h>
#include "soc/soc.h"
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_timer_impl.h"
#include "esp_compiler.h"
#include "esp_private/startup_internal.h"
#include "esp_private/esp_timer_private.h"
#include "esp_private/system_internal.h"
#include "esp_private/critical_section.h"
#include "sdkconfig.h"

#include <zephyr/drivers/interrupt_controller/intc_esp32.h>

#ifdef CONFIG_ESP_TIMER_PROFILING
#define WITH_PROFILING 1
#endif

#ifndef NDEBUG
// Enable built-in checks in queue.h in debug builds
#define INVARIANTS
#endif
#include "sys/queue.h"

#define EVENT_ID_DELETE_TIMER   0xF0DE1E1E

typedef enum {
    FL_ISR_DISPATCH_METHOD   = (1 << 0),  //!< 0=Callback is called from timer task, 1=Callback is called from timer ISR
    FL_SKIP_UNHANDLED_EVENTS = (1 << 1),  //!< 0=NOT skip unhandled events for periodic timers, 1=Skip unhandled events for periodic timers
    FL_CALLBACK_IS_RUNNING   = (1 << 2),  //!< 0=Callback is NOT running, 1=Callback is running
} flags_t;

struct esp_timer {
    uint64_t alarm;
    uint64_t period: 56;
    volatile flags_t flags: 8;
    union {
        esp_timer_cb_t callback;
        uint32_t event_id;
    };
    void* arg;
#if WITH_PROFILING
    const char* name;
    size_t times_triggered;
    size_t times_armed;
    size_t times_skipped;
    uint64_t total_callback_run_time;
#endif // WITH_PROFILING
    LIST_ENTRY(esp_timer) list_entry;
};

static inline bool is_initialized(void);
static esp_err_t timer_insert(esp_timer_handle_t timer, bool without_update_alarm);
static void timer_remove(esp_timer_handle_t timer);
static bool timer_armed(esp_timer_handle_t timer);
static void timer_list_lock(esp_timer_dispatch_t timer_type);
static void timer_list_unlock(esp_timer_dispatch_t timer_type);
static esp_err_t timer_restart(esp_timer_handle_t timer, uint64_t timeout_us, uint64_t alarm_us);

#if WITH_PROFILING
static void timer_insert_inactive(esp_timer_handle_t timer);
static void timer_remove_inactive(esp_timer_handle_t timer);
#endif // WITH_PROFILING

ESP_LOG_ATTR_TAG(TAG, "esp_timer");

// Armed timers sorted by alarm for each dispatch method, followed by a suffix
// of EVENT_ID_DELETE_TIMER entries ready to be freed by the timer task.
static LIST_HEAD(esp_timer_list, esp_timer) s_timers[ESP_TIMER_MAX] = {
    [0 ...(ESP_TIMER_MAX - 1)] = LIST_HEAD_INITIALIZER(s_timers)
};
#if WITH_PROFILING
// lists of unarmed timers for two dispatch methods: ISR and TASK,
// used only to be able to dump statistics about all the timers
static LIST_HEAD(esp_inactive_timer_list, esp_timer) s_inactive_timers[ESP_TIMER_MAX] = {
    [0 ...(ESP_TIMER_MAX - 1)] = LIST_HEAD_INITIALIZER(s_timers)
};
#endif

K_KERNEL_STACK_MEMBER(timer_task_stack, CONFIG_ESP32_TIMER_TASK_STACK_SIZE);
// task used to dispatch timer callbacks
static struct k_thread s_timer_task;
/* counting semaphore used to notify the timer task from ISR */
static struct k_sem s_timer_semaphore;
static bool s_timer_task_created = false;

/* lock key for critical sections */
static esp_os_spinlock_t s_timer_lock_key[ESP_TIMER_MAX] = {
    [0 ... ESP_TIMER_MAX - 1] = ESP_OS_SPINLOCK_INIT,
};

// Callback execution is serialized within each dispatch method.
// Completion sequences are protected by the corresponding dispatch list lock.
static uint32_t s_callback_completed[ESP_TIMER_MAX];

// Accepted deletions, including callbacks still running and batches being freed.
// Counter updates are protected by the corresponding dispatch list lock.
static size_t s_pending_delete_count[ESP_TIMER_MAX];

#ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
// For ISR dispatch method, a callback function of the timer may require a context switch
static volatile bool s_isr_dispatch_need_yield = false;
#endif // CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD

esp_err_t esp_timer_create(const esp_timer_create_args_t* args,
                           esp_timer_handle_t* out_handle)
{
    if (!is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (args == NULL || args->callback == NULL || out_handle == NULL ||
            args->dispatch_method < 0 || args->dispatch_method >= ESP_TIMER_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_timer_handle_t result = (esp_timer_handle_t) k_calloc(1, sizeof(*result));
    if (result == NULL) {
        return ESP_ERR_NO_MEM;
    }
    result->callback = args->callback;
    result->arg = args->arg;
    result->flags = (args->dispatch_method ? FL_ISR_DISPATCH_METHOD : 0) |
                    (args->skip_unhandled_events ? FL_SKIP_UNHANDLED_EVENTS : 0);
#if WITH_PROFILING
    result->name = args->name;
    esp_timer_dispatch_t dispatch_method = result->flags & FL_ISR_DISPATCH_METHOD;
    timer_list_lock(dispatch_method);
    timer_insert_inactive(result);
    timer_list_unlock(dispatch_method);
#endif
    *out_handle = result;
    return ESP_OK;
}

/*
 * We have placed this function in IRAM to ensure consistency with the esp_timer API.
 * esp_timer_start_once, esp_timer_start_periodic and esp_timer_stop are in IRAM.
 * But actually in IDF esp_timer_restart is used only in one place, which requires keeping
 * in IRAM when PM_SLP_IRAM_OPT = y and ESP_TASK_WDT USE ESP_TIMER = y.
*/
esp_err_t ESP_TIMER_IRAM_ATTR esp_timer_restart(esp_timer_handle_t timer, uint64_t timeout_us)
{
    return timer_restart(timer, timeout_us, 0);
}

esp_err_t ESP_TIMER_IRAM_ATTR esp_timer_restart_at(esp_timer_handle_t timer, uint64_t period_us, uint64_t first_alarm_us)
{
    const uint64_t min_overhead_us = esp_timer_impl_get_min_period_us();
    if (first_alarm_us + min_overhead_us < esp_timer_impl_get_time()) {
        return ESP_ERR_INVALID_ARG;
    }
    return timer_restart(timer, period_us, first_alarm_us);
}

static esp_err_t ESP_TIMER_IRAM_ATTR timer_restart(esp_timer_handle_t timer, uint64_t timeout_us, uint64_t first_alarm_us)
{
    esp_err_t ret = ESP_OK;

    if (timer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_timer_dispatch_t dispatch_method = timer->flags & FL_ISR_DISPATCH_METHOD;
    timer_list_lock(dispatch_method);

    /* The timer may expire while this task is waiting for the list lock. */
    if (!timer_armed(timer)) {
        timer_list_unlock(dispatch_method);
        return ESP_ERR_INVALID_STATE;
    }

    const int64_t now = esp_timer_impl_get_time();
    const uint64_t period = timer->period;

    /* We need to remove the timer to the list of timers and reinsert it at
     * the right position. In fact, the timers are sorted by their alarm value
     * (earliest first) */
    timer_remove(timer);

    /* Two cases here:
     * - if the alarm was a periodic one, i.e. `period` is not 0, the given timeout_us becomes the new period
     * - if the alarm was a one-shot one, i.e. `period` is 0, it remains non-periodic. */
    if (period != 0) {
        /* Remove function got rid of the alarm and period fields, restore them */
        const uint64_t min_period = esp_timer_impl_get_min_period_us();
        const uint64_t new_period = MAX(timeout_us, min_period);
        timer->alarm = (first_alarm_us != 0) ? first_alarm_us : now + new_period;
        timer->period = new_period;
    } else {
        /* The new one-shot alarm shall be triggered timeout_us after the current time */
        timer->alarm = (first_alarm_us != 0) ? first_alarm_us : now + timeout_us;
        timer->period = 0;
    }
    ret = timer_insert(timer, false);

    timer_list_unlock(dispatch_method);

    return ret;
}

static esp_err_t ESP_TIMER_IRAM_ATTR timer_init(esp_timer_handle_t timer, uint64_t period_us, uint64_t first_alarm_us)
{
    if (timer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_timer_dispatch_t dispatch_method = timer->flags & FL_ISR_DISPATCH_METHOD;
    esp_err_t err;

    timer_list_lock(dispatch_method);

    /* Check if the timer is armed once the list is locked.
     * Otherwise another task may arm the timer between the checks
     * and us locking the list, resulting in us inserting the
     * timer to s_timers a second time. This will create a loop
     * in s_timers. */
    if (timer_armed(timer) || (timer->event_id == EVENT_ID_DELETE_TIMER)) {
        err = ESP_ERR_INVALID_STATE;
    } else {
        timer->alarm = first_alarm_us;
        timer->period = period_us;
#if WITH_PROFILING
        timer->times_armed++;
        timer->times_skipped = 0;
#endif
        err = timer_insert(timer, false);
    }
    timer_list_unlock(dispatch_method);
    return err;
}

esp_err_t ESP_TIMER_IRAM_ATTR esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us)
{
    return timer_init(timer, 0, esp_timer_get_time() + timeout_us);
}

esp_err_t ESP_TIMER_IRAM_ATTR esp_timer_start_once_at(esp_timer_handle_t timer, uint64_t alarm_us)
{
    const uint64_t min_overhead_us = esp_timer_impl_get_min_period_us();
    if (alarm_us + min_overhead_us < esp_timer_impl_get_time()) {
        return ESP_ERR_INVALID_ARG;
    }
    return timer_init(timer, 0, alarm_us);
}

esp_err_t ESP_TIMER_IRAM_ATTR esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period_us)
{
    uint64_t min_period = esp_timer_impl_get_min_period_us();
    period_us = MAX(period_us, min_period);
    return timer_init(timer, period_us, esp_timer_get_time() + period_us);
}

esp_err_t ESP_TIMER_IRAM_ATTR esp_timer_start_periodic_at(esp_timer_handle_t timer, uint64_t period_us, uint64_t first_alarm_us)
{
    const uint64_t min_overhead_us = esp_timer_impl_get_min_period_us();
    if (first_alarm_us + min_overhead_us < esp_timer_impl_get_time()) {
        return ESP_ERR_INVALID_ARG;
    }
    period_us = MAX(period_us, min_overhead_us);
    return timer_init(timer, period_us, first_alarm_us);
}

esp_err_t ESP_TIMER_IRAM_ATTR esp_timer_stop(esp_timer_handle_t timer)
{
    if (timer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_timer_dispatch_t dispatch_method = timer->flags & FL_ISR_DISPATCH_METHOD;
    esp_err_t err = ESP_OK;

    timer_list_lock(dispatch_method);

    /* Check if the timer is armed once the list is locked to avoid a data race */
    if (!timer_armed(timer)) {
        err = ESP_ERR_INVALID_STATE;
    } else {
        timer_remove(timer);
    }
    timer_list_unlock(dispatch_method);
    return err;
}

static inline bool callback_has_completed(esp_timer_dispatch_t dispatch_method, uint32_t completed_seq)
{
    timer_list_lock(dispatch_method);
    bool completed = s_callback_completed[dispatch_method] != completed_seq;
    timer_list_unlock(dispatch_method);
    return completed;
}

esp_err_t esp_timer_stop_blocking(esp_timer_handle_t timer, uint32_t timeout_ticks)
{
    if (timer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_timer_dispatch_t dispatch_method = timer->flags & FL_ISR_DISPATCH_METHOD;

    timer_list_lock(dispatch_method);

    bool callback_running = (timer->flags & FL_CALLBACK_IS_RUNNING) != 0;
    /* Check if the timer is armed once the list is locked to avoid a data race */
    if (timer_armed(timer)) {
        timer_remove(timer);
    }

    // The timer may be deleted while we wait for its callback to complete,
    // so do not access the timer structure after releasing this lock.
    uint32_t completed_seq = s_callback_completed[dispatch_method];
    timer_list_unlock(dispatch_method);

    if (callback_running) {
        // timer_process_alarm() releases the timer list lock while executing the callback.
        // So it is possible that timer is disarmed but its callback is still running.
        // To guarantee that the callback will not run after esp_timer_stop_blocking(),
        // we need to wait for the callback to complete here.

        // In ISR context: do not wait to avoid blocking
        if (k_is_in_isr()) {
            return ESP_ERR_NOT_FINISHED;
        }

        if (k_current_get() == &s_timer_task) {
            // Called from the esp_timer task context (i.e., the callback owner is a TASK-dispatch timer).
            // Concurrency model:
            // - TASK-dispatch callbacks are executed by a single esp_timer task and are strictly serialized.
            //   Therefore, only one TASK callback can be running at any time.
            // - If CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD is enabled,
            //   ISR-dispatch callbacks and TASK-dispatch callbacks may be running at the same time.
            if (dispatch_method == ESP_TIMER_TASK) {
                // Only one ESP_TIMER_TASK callback can be running at any time.
                // So we are stopping the timer from its own callback context:
                // the callback will complete naturally after this function returns.
                return ESP_OK;
            }

            // Stopping a running ISR-dispatch timer from a foreign callback:
            // we cannot wait for ISR context to complete, and blocking the esp_timer task would
            // stall TASK-dispatch callbacks. Report that the timer callback is still running.
            return ESP_ERR_NOT_FINISHED;
        }

        int64_t start_time = k_uptime_get();
        while (!callback_has_completed(dispatch_method, completed_seq)) {
            if (timeout_ticks != (uint32_t) K_FOREVER.ticks) {
                int64_t elapsed = k_uptime_get() - start_time;
                if (elapsed >= k_ticks_to_ms_floor64(timeout_ticks)) {
                    return ESP_ERR_TIMEOUT;
                }
            }
            k_sleep(K_MSEC(1));
        }
    }

    return ESP_OK;
}

// Called with the original dispatch list locked, after the dispatcher has
// finished using the timer, including updating its profiling fields.
static void ESP_TIMER_IRAM_ATTR timer_schedule_delete(esp_timer_handle_t timer)
{
    // Reuse the original list without scheduling an alarm. timer_insert() moves
    // the inactive profiling entry, if any, into the ready-to-delete suffix.
    timer->alarm = UINT64_MAX;
    timer->period = 0;
    timer_insert(timer, true);
    // The task can free the timer once this dispatch list lock is released.
}

esp_err_t esp_timer_delete(esp_timer_handle_t timer)
{
    if (timer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_timer_dispatch_t dispatch_method = timer->flags & FL_ISR_DISPATCH_METHOD;
    esp_err_t err = ESP_OK;
    bool notify_task = false;
    timer_list_lock(dispatch_method);

    if (timer_armed(timer) || (timer->event_id == EVENT_ID_DELETE_TIMER)) {
        err = ESP_ERR_INVALID_STATE;
    } else {
        ++s_pending_delete_count[dispatch_method];
        // A running callback already has a local copy of the function pointer.
        // Keep list ownership until the callback epilogue, including profiling, finishes.
        timer->event_id = EVENT_ID_DELETE_TIMER;
        if (!(timer->flags & FL_CALLBACK_IS_RUNNING)) {
            timer_schedule_delete(timer);
            notify_task = true;
        }
    }
    timer_list_unlock(dispatch_method);

    if (notify_task) {
        k_sem_give(&s_timer_semaphore);
    }
    return err;
}

static ESP_TIMER_IRAM_ATTR esp_err_t timer_insert(esp_timer_handle_t timer, bool without_update_alarm)
{
#if WITH_PROFILING
    timer_remove_inactive(timer);
#endif
    esp_timer_handle_t it, last = NULL;
    esp_timer_dispatch_t dispatch_method = timer->flags & FL_ISR_DISPATCH_METHOD;
    if (LIST_FIRST(&s_timers[dispatch_method]) == NULL) {
        LIST_INSERT_HEAD(&s_timers[dispatch_method], timer, list_entry);
    } else {
        LIST_FOREACH(it, &s_timers[dispatch_method], list_entry) {
            // Keep deletion records last, even if a normal timer's alarm is UINT64_MAX.
            if (it->event_id == EVENT_ID_DELETE_TIMER || timer->alarm < it->alarm) {
                LIST_INSERT_BEFORE(it, timer, list_entry);
                break;
            }
            last = it;
        }
        if (it == NULL) {
            assert(last);
            LIST_INSERT_AFTER(last, timer, list_entry);
        }
    }
    if (without_update_alarm == false && timer == LIST_FIRST(&s_timers[dispatch_method])) {
        esp_timer_impl_set_alarm_id(timer->alarm, dispatch_method);
    }
    return ESP_OK;
}

// It should be always called with the timer list locked
static ESP_TIMER_IRAM_ATTR void timer_remove(esp_timer_handle_t timer)
{
    esp_timer_dispatch_t dispatch_method = timer->flags & FL_ISR_DISPATCH_METHOD;
    esp_timer_handle_t first_timer = LIST_FIRST(&s_timers[dispatch_method]);
    LIST_REMOVE(timer, list_entry);
    timer->alarm = 0;
    timer->period = 0;
    if (timer == first_timer) { // if this timer was the first in the list.
        uint64_t next_timestamp = UINT64_MAX;
        first_timer = LIST_FIRST(&s_timers[dispatch_method]);
        if (first_timer) { // if after removing the timer from the list, this list is not empty.
            next_timestamp = first_timer->alarm;
        }
        esp_timer_impl_set_alarm_id(next_timestamp, dispatch_method);
    }
#if WITH_PROFILING
    timer_insert_inactive(timer);
#endif
}

#if WITH_PROFILING

static ESP_TIMER_IRAM_ATTR void timer_insert_inactive(esp_timer_handle_t timer)
{
    /* May be locked or not, depending on where this is called from.
     * Lock recursively.
     */
    esp_timer_dispatch_t dispatch_method = timer->flags & FL_ISR_DISPATCH_METHOD;
    esp_timer_handle_t head = LIST_FIRST(&s_inactive_timers[dispatch_method]);
    if (head == NULL) {
        LIST_INSERT_HEAD(&s_inactive_timers[dispatch_method], timer, list_entry);
    } else {
        /* Insert as head element as this is the fastest thing to do.
         * Removal is O(1) anyway.
         */
        LIST_INSERT_BEFORE(head, timer, list_entry);
    }
}

static ESP_TIMER_IRAM_ATTR void timer_remove_inactive(esp_timer_handle_t timer)
{
    LIST_REMOVE(timer, list_entry);
}

#endif // WITH_PROFILING

static ESP_TIMER_IRAM_ATTR bool timer_armed(esp_timer_handle_t timer)
{
    return timer->alarm > 0 && timer->event_id != EVENT_ID_DELETE_TIMER;
}

static ESP_TIMER_IRAM_ATTR void timer_list_lock(esp_timer_dispatch_t timer_type)
{
    esp_os_enter_critical_safe(&s_timer_lock_key[timer_type]);
}

static ESP_TIMER_IRAM_ATTR void timer_list_unlock(esp_timer_dispatch_t timer_type)
{
    esp_os_exit_critical_safe(&s_timer_lock_key[timer_type]);
}

#ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
ESP_TIMER_IRAM_ATTR
#endif
// Keep this as a standalone symbol because ldgen maps it by name.
// Return whether the timer task needs to process newly ready deletions.
static bool __attribute__((noinline, noclone)) timer_process_alarm(esp_timer_dispatch_t dispatch_method)
{
    bool timers_to_delete = false;
    timer_list_lock(dispatch_method);
    esp_timer_handle_t it;
    while (1) {
        it = LIST_FIRST(&s_timers[dispatch_method]);
        int64_t now = esp_timer_impl_get_time();
        // Ready-to-delete entries have alarm == UINT64_MAX and never become due.
        if (it == NULL || it->alarm > now) {
            break;
        }
        LIST_REMOVE(it, list_entry);
        it->flags |= FL_CALLBACK_IS_RUNNING;
        if (it->period > 0) {
            int skipped = (now - it->alarm) / it->period;
            if ((it->flags & FL_SKIP_UNHANDLED_EVENTS) && (skipped > 1)) {
                it->alarm = now + it->period;
#if WITH_PROFILING
                it->times_skipped += skipped;
#endif
            } else {
                it->alarm += it->period;
            }
            timer_insert(it, true);
        } else {
            it->alarm = 0;
#if WITH_PROFILING
            timer_insert_inactive(it);
#endif
        }
#if WITH_PROFILING
        uint64_t callback_start = now;
#endif
        esp_timer_cb_t callback = it->callback;
        void* arg = it->arg;
        timer_list_unlock(dispatch_method);
        (*callback)(arg);
        timer_list_lock(dispatch_method);
        it->flags &= ~FL_CALLBACK_IS_RUNNING;
#if WITH_PROFILING
        it->times_triggered++;
        it->total_callback_run_time += esp_timer_impl_get_time() - callback_start;
#endif
        // Publish completion after the callback and its profiling updates.
        ++s_callback_completed[dispatch_method];
        if (it->event_id == EVENT_ID_DELETE_TIMER) {
            timer_schedule_delete(it);
            timers_to_delete = true;
        }
    } // while(1)
    uint64_t next_alarm = (it != NULL) ? it->alarm : UINT64_MAX;
    esp_timer_impl_set_alarm_id(next_alarm, dispatch_method);
    timer_list_unlock(dispatch_method);
    return timers_to_delete;
}

// Keep this as a standalone symbol because ldgen maps it by name.
static void __attribute__((noinline, noclone)) timer_process_deletes(void)
{
    for (esp_timer_dispatch_t dispatch_method = ESP_TIMER_TASK; dispatch_method < ESP_TIMER_MAX; ++dispatch_method) {
        timer_list_lock(dispatch_method);
        if (s_pending_delete_count[dispatch_method] == 0) {
            timer_list_unlock(dispatch_method);
            continue;
        }

        esp_timer_handle_t timer;
        LIST_FOREACH(timer, &s_timers[dispatch_method], list_entry) {
            if (timer->event_id == EVENT_ID_DELETE_TIMER) {
                break;
            }
        }
        if (timer != NULL) {
            // All remaining entries are ready for deletion. Detach the suffix
            // in one step; removing it does not change the next hardware alarm.
            *timer->list_entry.le_prev = NULL;
        }
        timer_list_unlock(dispatch_method);

        size_t deleted_count = 0;
        while (timer != NULL) {
            // Only use forward links: the detached nodes' back-links are no longer valid.
            esp_timer_handle_t next = LIST_NEXT(timer, list_entry);
            k_free(timer);
            timer = next;
            ++deleted_count;
        }

        if (deleted_count != 0) {
            // Include detached timers in the count until every free() completes,
            // so deinit cannot stop the task in the middle of this batch.
            timer_list_lock(dispatch_method);
            s_pending_delete_count[dispatch_method] -= deleted_count;
            timer_list_unlock(dispatch_method);
        }
    }
}

static void timer_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        k_sem_take(&s_timer_semaphore, K_FOREVER);
        // all deferred events are processed at a time
        timer_process_alarm(ESP_TIMER_TASK);
        timer_process_deletes();
    }
}

#ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
ESP_TIMER_IRAM_ATTR void esp_timer_isr_dispatch_need_yield(void)
{
    __ASSERT(k_is_in_isr(), "esp_timer_isr_dispatch_need_yield must be called from ISR context");
    s_isr_dispatch_need_yield = true;
}

#endif

static void ESP_TIMER_IRAM_ATTR timer_alarm_handler(void *arg)
{
    ARG_UNUSED(arg);

#ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
    uint32_t due_alarm_mask = esp_timer_impl_claim_due_alarms();
    bool timers_to_delete = false;

    if (due_alarm_mask & (1U << ESP_TIMER_ISR)) {
        timers_to_delete = timer_process_alarm(ESP_TIMER_ISR);
    }

    /* The kernel reschedules on ISR exit, no explicit yield is needed. */
    s_isr_dispatch_need_yield = false;

    if (due_alarm_mask & (1U << ESP_TIMER_TASK) || timers_to_delete) {
        k_sem_give(&s_timer_semaphore);
    }
#else
    k_sem_give(&s_timer_semaphore);
#endif
}

static ESP_TIMER_IRAM_ATTR inline bool is_initialized(void)
{
    return s_timer_task_created;
}

static esp_err_t init_timer_task(void)
{
    esp_err_t err = ESP_OK;
    if (is_initialized()) {
        ESP_EARLY_LOGE(TAG, "Task is already initialized");
        err = ESP_ERR_INVALID_STATE;
    } else {
        k_sem_init(&s_timer_semaphore, 0, 1);

        /* Pin to the configured core (create suspended, pin, then start) to keep
         * the timer task on the same core as the Wi-Fi/BLE stacks it serves.
         */
        k_tid_t tid = k_thread_create(&s_timer_task, timer_task_stack,
                        CONFIG_ESP32_TIMER_TASK_STACK_SIZE,
                        timer_task, NULL, NULL, NULL,
                        K_PRIO_PREEMPT(CONFIG_ESP32_TIMER_TASK_PRIO), 0,
                        IS_ENABLED(CONFIG_SMP) ? K_FOREVER : K_NO_WAIT);
        k_thread_name_set(&s_timer_task, "esp_timer");

#if defined(CONFIG_SMP)
#if defined(CONFIG_SCHED_CPU_MASK) && defined(CONFIG_SOC_ESP32_TIMER_TASK_CORE_ID)
        k_thread_cpu_pin(tid, CONFIG_SOC_ESP32_TIMER_TASK_CORE_ID);
#endif
        k_thread_start(tid);
#else
        ARG_UNUSED(tid);
#endif
        s_timer_task_created = true;
    }
    return err;
}

static void deinit_timer_task(void)
{
    if (s_timer_task_created) {
        k_thread_abort(&s_timer_task);
        s_timer_task_created = false;
    }
}

esp_err_t esp_timer_init(void)
{
    esp_err_t err = ESP_OK;

    err = init_timer_task();
    if (err == ESP_OK) {
        err = esp_timer_impl_init(&timer_alarm_handler);
        if (err != ESP_OK) {
            ESP_EARLY_LOGE(TAG, "ISR init failed");
            deinit_timer_task();
        }
    }
    return err;
}

#if defined(CONFIG_WIFI_ESP32) || defined(CONFIG_BT_ESP32) || defined(CONFIG_IEEE802154_ESP32)
SYS_INIT(esp_timer_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif

esp_err_t esp_timer_deinit(void)
{
    if (!is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Check if there are any active timers and pending deletions, including detached batches. */
    for (esp_timer_dispatch_t dispatch_method = ESP_TIMER_TASK; dispatch_method < ESP_TIMER_MAX; ++dispatch_method) {
        if (!LIST_EMPTY(&s_timers[dispatch_method]) || s_pending_delete_count[dispatch_method] != 0) {
            return ESP_ERR_INVALID_STATE;
        }
    }

    /* We can only check if there are any timers which are not deleted if
     * profiling is enabled.
     */
#if WITH_PROFILING
    for (esp_timer_dispatch_t dispatch_method = ESP_TIMER_TASK; dispatch_method < ESP_TIMER_MAX; ++dispatch_method) {
        if (!LIST_EMPTY(&s_inactive_timers[dispatch_method])) {
            return ESP_ERR_INVALID_STATE;
        }
    }
#endif

    esp_timer_impl_deinit();
    deinit_timer_task();
    return ESP_OK;
}

static void append_to_buffer(char** dst, size_t* dst_size, const char* format, ...)
{
    /* Defensive: avoid underflow in the truncation path below if no space remains. */
    if (*dst_size == 0) {
        return;
    }

    va_list args;
    va_start(args, format);
    int cb = vsnprintf(*dst, *dst_size, format, args);
    va_end(args);

    if (cb < 0) {
        return;
    }

    /* On truncation, snprintf returns the full would-be length. Keep the
     * cursor inside the buffer and preserve the terminating NUL byte.
     */
    if ((size_t) cb >= *dst_size) {
        *dst += *dst_size - 1;
        *dst_size = 1;
        return;
    }

    *dst += cb;
    *dst_size -= cb;
}

static void print_timer_info(esp_timer_handle_t t, char** dst, size_t* dst_size)
{
#if WITH_PROFILING
    // name is optional, might be missed.
    if (t->name) {
        append_to_buffer(dst, dst_size, "%-20.20s  ", t->name);
    } else {
        append_to_buffer(dst, dst_size, "timer@%-10p  ", t);
    }

    append_to_buffer(dst, dst_size, "%-10" PRIu64"  %-12" PRIu64"  %-12zu  %-12zu  %-12zu  %-12" PRIu64"\n",
                     (uint64_t)t->period, t->alarm, t->times_armed,
                     t->times_triggered, t->times_skipped, t->total_callback_run_time);
    /* keep this in sync with the format string, used in esp_timer_dump */
#define TIMER_INFO_LINE_LEN 103
#else
    append_to_buffer(dst, dst_size, "timer@%-14p  %-10" PRIu64"  %-12" PRIu64"\n", t, (uint64_t)t->period, t->alarm);
#define TIMER_INFO_LINE_LEN 47
#endif
}

esp_err_t esp_timer_dump(FILE* stream)
{
    /* Since timer lock is a critical section, we don't want to print directly
     * to stdout, since that may cause a deadlock if stdout is interrupt-driven
     * (via the UART driver). Allocate sufficiently large chunk of memory first,
     * print to it, then dump this memory to stdout.
     */

    esp_timer_handle_t it;

    /* First count the number of timers */
    size_t timer_count = 0;
    for (esp_timer_dispatch_t dispatch_method = ESP_TIMER_TASK; dispatch_method < ESP_TIMER_MAX; ++dispatch_method) {
        timer_list_lock(dispatch_method);
        LIST_FOREACH(it, &s_timers[dispatch_method], list_entry) {
            if (it->event_id == EVENT_ID_DELETE_TIMER) {
                break;
            }
            ++timer_count;
        }
#if WITH_PROFILING
        LIST_FOREACH(it, &s_inactive_timers[dispatch_method], list_entry) {
            if (it->event_id != EVENT_ID_DELETE_TIMER) {
                ++timer_count;
            }
        }
#endif
        timer_list_unlock(dispatch_method);
    }

    /* Allocate the memory for this number of timers. Since we have unlocked,
     * we may find that there are more timers. There's no bulletproof solution
     * for this (can't allocate from a critical section), but we allocate
     * slightly more and the output will be truncated if that is not enough.
     */
    size_t buf_size = TIMER_INFO_LINE_LEN * (timer_count + 3);
    /* buf_size is the snprintf size, including NUL; keep one extra byte as slack. */
    char* print_buf = k_calloc(1, buf_size + 1);
    if (print_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Print to the buffer */
    char* pos = print_buf;
    for (esp_timer_dispatch_t dispatch_method = ESP_TIMER_TASK; dispatch_method < ESP_TIMER_MAX; ++dispatch_method) {
        timer_list_lock(dispatch_method);
        LIST_FOREACH(it, &s_timers[dispatch_method], list_entry) {
            if (it->event_id == EVENT_ID_DELETE_TIMER) {
                break;
            }
            print_timer_info(it, &pos, &buf_size);
        }
#if WITH_PROFILING
        LIST_FOREACH(it, &s_inactive_timers[dispatch_method], list_entry) {
            if (it->event_id != EVENT_ID_DELETE_TIMER) {
                print_timer_info(it, &pos, &buf_size);
            }
        }
#endif
        timer_list_unlock(dispatch_method);
    }

    if (stream != NULL) {
        fprintf(stream, "Timer stats:\n");
#if WITH_PROFILING
        fprintf(stream, "%-20s  %-10s  %-12s  %-12s  %-12s  %-12s  %-12s\n",
                "Name", "Period", "Alarm", "Times_armed", "Times_trigg", "Times_skip", "Cb_exec_time");
#else
        fprintf(stream, "%-20s  %-10s  %-12s\n", "Name", "Period", "Alarm");
#endif

        /* Print the buffer */
        fputs(print_buf, stream);
    }

    k_free(print_buf);
    return ESP_OK;
}

int64_t ESP_TIMER_IRAM_ATTR esp_timer_get_next_alarm(void)
{
    int64_t next_alarm = INT64_MAX;
    for (esp_timer_dispatch_t dispatch_method = ESP_TIMER_TASK; dispatch_method < ESP_TIMER_MAX; ++dispatch_method) {
        timer_list_lock(dispatch_method);
        esp_timer_handle_t it = LIST_FIRST(&s_timers[dispatch_method]);
        if (it) {
            if (next_alarm > it->alarm) {
                next_alarm = it->alarm;
            }
        }
        timer_list_unlock(dispatch_method);
    }
    return next_alarm;
}

int64_t ESP_TIMER_IRAM_ATTR esp_timer_get_next_alarm_for_wake_up(void)
{
    int64_t next_alarm = INT64_MAX;
    for (esp_timer_dispatch_t dispatch_method = ESP_TIMER_TASK; dispatch_method < ESP_TIMER_MAX; ++dispatch_method) {
        timer_list_lock(dispatch_method);
        esp_timer_handle_t it = NULL;
        LIST_FOREACH(it, &s_timers[dispatch_method], list_entry) {
            // timers with the SKIP_UNHANDLED_EVENTS flag do not want to wake up CPU from a sleep mode.
            if ((it->flags & FL_SKIP_UNHANDLED_EVENTS) == 0) {
                if (next_alarm > it->alarm) {
                    next_alarm = it->alarm;
                }
                break;
            }
        }
        timer_list_unlock(dispatch_method);
    }
    return next_alarm;
}

esp_err_t ESP_TIMER_IRAM_ATTR esp_timer_get_period(esp_timer_handle_t timer, uint64_t *period)
{
    if (timer == NULL || period == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_timer_dispatch_t dispatch_method = timer->flags & FL_ISR_DISPATCH_METHOD;

    timer_list_lock(dispatch_method);
    *period = timer->period;
    timer_list_unlock(dispatch_method);

    return ESP_OK;
}

esp_err_t ESP_TIMER_IRAM_ATTR esp_timer_get_expiry_time(esp_timer_handle_t timer, uint64_t *expiry)
{
    if (timer == NULL || expiry == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (timer->period > 0) {
        /* Return error for periodic timers */
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_timer_dispatch_t dispatch_method = timer->flags & FL_ISR_DISPATCH_METHOD;

    timer_list_lock(dispatch_method);
    *expiry = timer->alarm;
    timer_list_unlock(dispatch_method);

    return ESP_OK;
}

bool ESP_TIMER_IRAM_ATTR esp_timer_is_active(esp_timer_handle_t timer)
{
    if (timer == NULL) {
        return false;
    }
    esp_timer_dispatch_t dispatch_method = timer->flags & FL_ISR_DISPATCH_METHOD;

    timer_list_lock(dispatch_method);
    // Timer is active if it is armed or its callback is currently running
    // After esp_timer_stop() timer is disarmed, but its callback may still be running
    bool active = timer_armed(timer) || ((timer->flags & FL_CALLBACK_IS_RUNNING) != 0);
    timer_list_unlock(dispatch_method);

    return active;
}
