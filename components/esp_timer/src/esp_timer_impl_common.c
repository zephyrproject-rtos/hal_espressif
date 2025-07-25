/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "esp_timer_impl.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_task.h"
#include "esp_attr.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/__assert.h>

/* Spinlock used to protect access to the hardware registers. */
unsigned int s_time_update_lock;
static atomic_t s_timer_lock_counter;

/* Alarm values to generate interrupt on match
 * [0] - for ESP_TIMER_TASK alarms,
 * [1] - for ESP_TIMER_ISR alarms.
*/
uint64_t timestamp_id[2] = { UINT64_MAX, UINT64_MAX };

void esp_timer_impl_lock(void)
{
    if (atomic_inc(&s_timer_lock_counter) == 0) {
        s_time_update_lock = irq_lock();
    }
}

void esp_timer_impl_unlock(void)
{
    __ASSERT_NO_MSG(atomic_get(&s_timer_lock_counter) > 0);
    if (atomic_dec(&s_timer_lock_counter) == 1) {
        irq_unlock(s_time_update_lock);
    }
}

void esp_timer_private_lock(void) __attribute__((alias("esp_timer_impl_lock")));
void esp_timer_private_unlock(void) __attribute__((alias("esp_timer_impl_unlock")));

void IRAM_ATTR esp_timer_impl_set_alarm(uint64_t timestamp)
{
    esp_timer_impl_set_alarm_id(timestamp, 0);
}

#ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
void IRAM_ATTR esp_timer_impl_try_to_set_next_alarm(void) {
    esp_timer_impl_lock();
    unsigned now_alarm_idx;  // ISR is called due to this current alarm
    unsigned next_alarm_idx; // The following alarm after now_alarm_idx
    if (timestamp_id[0] < timestamp_id[1]) {
        now_alarm_idx = 0;
        next_alarm_idx = 1;
    } else {
        now_alarm_idx = 1;
        next_alarm_idx = 0;
    }

    if (timestamp_id[next_alarm_idx] != UINT64_MAX) {
        // The following alarm is valid and can be used.
        // Remove the current alarm from consideration.
        esp_timer_impl_set_alarm_id(UINT64_MAX, now_alarm_idx);
    } else {
        // There is no the following alarm.
        // Remove the current alarm from consideration as well.
        timestamp_id[now_alarm_idx] = UINT64_MAX;
    }
    esp_timer_impl_unlock();
}
#endif

/* FIXME: This value is safe for 80MHz APB frequency, should be modified to depend on clock frequency. */
uint64_t IRAM_ATTR esp_timer_impl_get_min_period_us(void)
{
    return 50;
}
