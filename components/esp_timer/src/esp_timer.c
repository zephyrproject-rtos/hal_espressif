// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <zephyr.h>
#include <sys/param.h>
#include <string.h>
#include "soc/soc.h"
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp32/rom/ets_sys.h"
#include "esp_task.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
// #include "soc/spinlock.h"
#include "esp_timer.h"
#include "esp_timer_impl.h"
#include "sdkconfig.h"

#include "esp_private/startup_internal.h"
#include "esp_private/esp_timer_private.h"
#include "esp_private/system_internal.h"
#define LOG_MODULE_NAME esp_timer
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);


#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rtc.h"
#endif

#ifdef CONFIG_ESP_TIMER_PROFILING
#define WITH_PROFILING 1
#endif

#ifndef NDEBUG
// Enable built-in checks in queue.h in debug builds
#define INVARIANTS
#endif
#include "sys/queue.h"

#define EVENT_ID_DELETE_TIMER   0xF0DE1E1E

#define TIMER_EVENT_QUEUE_SIZE      16
typedef enum {
    FL_DISPATCH_METHOD       = (1 << 0),  //!< 0=Callback is called from timer task, 1=Callback is called from timer ISR
    FL_SKIP_UNHANDLED_EVENTS = (1 << 1),  //!< 0=NOT skip unhandled events for periodic timers, 1=Skip unhandled events for periodic timers
} flags_t;

struct esp_timer {
    uint64_t alarm;
    uint64_t period:56;
    flags_t flags:8;
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

K_KERNEL_STACK_MEMBER(timer_task_stack, 4096);
static bool init_status = false;

static bool is_initialized(void);
static esp_err_t timer_insert(esp_timer_handle_t timer);
static esp_err_t timer_remove(esp_timer_handle_t timer);
static bool timer_armed(esp_timer_handle_t timer);
static void timer_list_lock(void);
static void timer_list_unlock(void);

#if WITH_PROFILING
static void timer_insert_inactive(esp_timer_handle_t timer);
static void timer_remove_inactive(esp_timer_handle_t timer);
#endif // WITH_PROFILING

// list of currently armed timers
static LIST_HEAD(esp_timer_list, esp_timer) s_timers =
        LIST_HEAD_INITIALIZER(s_timers);
#if WITH_PROFILING
// list of unarmed timers, used only to be able to dump statistics about
// all the timers
static LIST_HEAD(esp_inactive_timer_list, esp_timer) s_inactive_timers =
        LIST_HEAD_INITIALIZER(s_timers);
#endif
// task used to dispatch timer callbacks
static struct k_thread s_timer_task;
// counting semaphore used to notify the timer task from ISR
static struct k_sem s_timer_semaphore;

#if CONFIG_SPIRAM_USE_MALLOC
// memory for s_timer_semaphore
static StaticQueue_t s_timer_semaphore_memory;
#endif

// lock protecting s_timers, s_inactive_timers
static unsigned int s_timer_lock;

esp_err_t esp_timer_create(const esp_timer_create_args_t* args,
                           esp_timer_handle_t* out_handle)
{
    if (!is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (args == NULL || args->callback == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_timer_handle_t result = (esp_timer_handle_t) k_calloc(1, sizeof(*result));
    if (result == NULL) {
        return ESP_ERR_NO_MEM;
    }
    result->callback = args->callback;
    result->arg = args->arg;
    result->flags = (args->dispatch_method ? FL_DISPATCH_METHOD : 0) |
                    (args->skip_unhandled_events ? FL_SKIP_UNHANDLED_EVENTS : 0);
#if WITH_PROFILING
    result->name = args->name;
    timer_insert_inactive(result);
#endif
    *out_handle = result;
    return ESP_OK;
}

esp_err_t IRAM_ATTR esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us)
{
    if (timer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!is_initialized() || timer_armed(timer)) {
        return ESP_ERR_INVALID_STATE;
    }
    timer_list_lock();
    timer->alarm = esp_timer_get_time() + timeout_us;
    timer->period = 0;
#if WITH_PROFILING
    timer->times_armed++;
#endif
    esp_err_t err = timer_insert(timer);
    timer_list_unlock();
    return err;
}

esp_err_t IRAM_ATTR esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period_us)
{
    if (timer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!is_initialized() || timer_armed(timer)) {
        return ESP_ERR_INVALID_STATE;
    }
    timer_list_lock();
    period_us = MAX(period_us, esp_timer_impl_get_min_period_us());
    timer->alarm = esp_timer_get_time() + period_us;
    timer->period = period_us;
#if WITH_PROFILING
    timer->times_armed++;
    timer->times_skipped = 0;
#endif
    esp_err_t err = timer_insert(timer);
    timer_list_unlock();
    return err;
}

esp_err_t IRAM_ATTR esp_timer_stop(esp_timer_handle_t timer)
{
    if (timer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!is_initialized() || !timer_armed(timer)) {
        return ESP_ERR_INVALID_STATE;
    }
    return timer_remove(timer);
}

esp_err_t esp_timer_delete(esp_timer_handle_t timer)
{
    if (timer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (timer_armed(timer)) {
        return ESP_ERR_INVALID_STATE;
    }
    timer_list_lock();
    timer->event_id = EVENT_ID_DELETE_TIMER;
    timer->alarm = esp_timer_get_time();
    timer->period = 0;
    timer_insert(timer);
    timer_list_unlock();
    return ESP_OK;
}

static IRAM_ATTR esp_err_t timer_insert(esp_timer_handle_t timer)
{
#if WITH_PROFILING
    timer_remove_inactive(timer);
#endif
    esp_timer_handle_t it, last = NULL;
    if (LIST_FIRST(&s_timers) == NULL) {
        LIST_INSERT_HEAD(&s_timers, timer, list_entry);
    } else {
        LIST_FOREACH(it, &s_timers, list_entry) {
            if (timer->alarm < it->alarm) {
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
    if (timer == LIST_FIRST(&s_timers)) {
        esp_timer_impl_set_alarm(timer->alarm);
    }
    return ESP_OK;
}

static IRAM_ATTR esp_err_t timer_remove(esp_timer_handle_t timer)
{
    timer_list_lock();
    LIST_REMOVE(timer, list_entry);
    timer->alarm = 0;
    timer->period = 0;
#if WITH_PROFILING
    timer_insert_inactive(timer);
#endif
    timer_list_unlock();
    return ESP_OK;
}

#if WITH_PROFILING

static IRAM_ATTR void timer_insert_inactive(esp_timer_handle_t timer)
{
    /* May be locked or not, depending on where this is called from.
     * Lock recursively.
     */
    timer_list_lock();
    esp_timer_handle_t head = LIST_FIRST(&s_inactive_timers);
    if (head == NULL) {
        LIST_INSERT_HEAD(&s_inactive_timers, timer, list_entry);
    } else {
        /* Insert as head element as this is the fastest thing to do.
         * Removal is O(1) anyway.
         */
        LIST_INSERT_BEFORE(head, timer, list_entry);
    }
    timer_list_unlock();
}

static IRAM_ATTR void timer_remove_inactive(esp_timer_handle_t timer)
{
    timer_list_lock();
    LIST_REMOVE(timer, list_entry);
    timer_list_unlock();
}

#endif // WITH_PROFILING

static IRAM_ATTR bool timer_armed(esp_timer_handle_t timer)
{
    return timer->alarm > 0;
}

static IRAM_ATTR void timer_list_lock(void)
{
    s_timer_lock = irq_lock();
}

static IRAM_ATTR void timer_list_unlock(void)
{
    irq_unlock(s_timer_lock);
}

static void timer_process_alarm(esp_timer_dispatch_t dispatch_method)
{
    /* unused, provision to allow running callbacks from ISR */
    (void) dispatch_method;

    timer_list_lock();
    esp_timer_handle_t it;
    while (1) {
        it = LIST_FIRST(&s_timers);
        int64_t now = esp_timer_impl_get_time();
        if (it == NULL || it->alarm > now) {
            break;
        }
        LIST_REMOVE(it, list_entry);
        if (it->event_id == EVENT_ID_DELETE_TIMER) {
            k_free(it);
            it = NULL;
        } else {
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
                timer_insert(it);
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
            timer_list_unlock();
            (*callback)(arg);
            timer_list_lock();
#if WITH_PROFILING
            it->times_triggered++;
            it->total_callback_run_time += esp_timer_impl_get_time() - callback_start;
#endif
        }
    }
    if (it) {
        esp_timer_impl_set_alarm(it->alarm);
    }
    timer_list_unlock();
}

static void timer_task(void* arg)
{
    while (true){
        k_sem_take(&s_timer_semaphore, K_FOREVER);
        // all deferred events are processed at a time
        timer_process_alarm(ESP_TIMER_TASK);
    }
}

static void IRAM_ATTR timer_alarm_handler(void* arg)
{
    k_sem_give(&s_timer_semaphore);
}

static IRAM_ATTR inline bool is_initialized(void)
{
    return init_status;
}

esp_err_t esp_timer_init(void)
{
    esp_err_t err;
    if (is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    int ret = k_sem_init(&s_timer_semaphore, 0, TIMER_EVENT_QUEUE_SIZE);
    if (ret != 0)
    {
        goto out;
    }

    k_tid_t tid = k_thread_create(&s_timer_task, timer_task_stack,
                4096, (k_thread_entry_t)timer_task, NULL, NULL, NULL,
                3, K_INHERIT_PERMS, K_NO_WAIT);

    if (!tid)
    {
        goto out;
    }

    k_thread_name_set(tid, "esp_timer");

    err = esp_timer_impl_init(&timer_alarm_handler);
    if (err != ESP_OK) {
        goto out;
    }

#if CONFIG_ESP_TIME_FUNCS_USE_ESP_TIMER
    // [refactor-todo] this logic, "esp_rtc_get_time_us() - g_startup_time", is also
    // the weak definition of esp_system_get_time; find a way to remove this duplication.
    esp_timer_private_advance(esp_rtc_get_time_us() - g_startup_time);
#endif

	init_status = true;

	return ESP_OK;

out:
	LOG_ERR("could not start esp timer");
	k_free(&s_timer_task);
	init_status = false;

    return ESP_ERR_NO_MEM;
}

esp_err_t esp_timer_deinit(void)
{
    if (!is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Check if there are any active timers */
    if (!LIST_EMPTY(&s_timers)) {
        return ESP_ERR_INVALID_STATE;
    }

    /* We can only check if there are any timers which are not deleted if
     * profiling is enabled.
     */
#if WITH_PROFILING
    if (!LIST_EMPTY(&s_inactive_timers)) {
        return ESP_ERR_INVALID_STATE;
    }
#endif

    esp_timer_impl_deinit();

    k_free(&s_timer_task);
    init_status = false;
    return ESP_OK;
}

static void print_timer_info(esp_timer_handle_t t, char** dst, size_t* dst_size)
{
#if WITH_PROFILING
    size_t cb;
    // name is optional, might be missed.
    if (t->name) {
        cb = snprintf(*dst, *dst_size, "%-12s  ", t->name);
    } else {
        cb = snprintf(*dst, *dst_size, "timer@%p  ", t);
    }
    cb += snprintf(*dst + cb, *dst_size + cb, "%12lld  %12lld  %9d  %9d  %6d  %12lld\n",
                    (uint64_t)t->period, t->alarm, t->times_armed,
                    t->times_triggered, t->times_skipped, t->total_callback_run_time);
    /* keep this in sync with the format string, used in esp_timer_dump */
#define TIMER_INFO_LINE_LEN 90
#else
    size_t cb = snprintf(*dst, *dst_size, "timer@%p  %12lld  %12lld\n", t, (uint64_t)t->period, t->alarm);
#define TIMER_INFO_LINE_LEN 46
#endif
    *dst += cb;
    *dst_size -= cb;
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
    timer_list_lock();
    LIST_FOREACH(it, &s_timers, list_entry) {
        ++timer_count;
    }
#if WITH_PROFILING
    LIST_FOREACH(it, &s_inactive_timers, list_entry) {
        ++timer_count;
    }
#endif
    timer_list_unlock();

    /* Allocate the memory for this number of timers. Since we have unlocked,
     * we may find that there are more timers. There's no bulletproof solution
     * for this (can't allocate from a critical section), but we allocate
     * slightly more and the output will be truncated if that is not enough.
     */
    size_t buf_size = TIMER_INFO_LINE_LEN * (timer_count + 3);
    char* print_buf = k_calloc(1, buf_size + 1);
    if (print_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Print to the buffer */
    timer_list_lock();
    char* pos = print_buf;
    LIST_FOREACH(it, &s_timers, list_entry) {
        print_timer_info(it, &pos, &buf_size);
    }
#if WITH_PROFILING
    LIST_FOREACH(it, &s_inactive_timers, list_entry) {
        print_timer_info(it, &pos, &buf_size);
    }
#endif
    timer_list_unlock();

    /* Print the buffer */
    fputs(print_buf, stream);

    k_free(print_buf);
    return ESP_OK;
}

int64_t IRAM_ATTR esp_timer_get_next_alarm(void)
{
    int64_t next_alarm = INT64_MAX;
    timer_list_lock();
    esp_timer_handle_t it = LIST_FIRST(&s_timers);
    if (it) {
        next_alarm = it->alarm;
    }
    timer_list_unlock();
    return next_alarm;
}

// Provides strong definition for system time functions relied upon
// by core components.
#if CONFIG_ESP_TIME_FUNCS_USE_ESP_TIMER
int64_t IRAM_ATTR esp_system_get_time(void)
{
    return esp_timer_get_time();
}

uint32_t IRAM_ATTR esp_system_get_time_resolution(void)
{
    return 1000;
}
#endif
