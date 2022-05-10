// Copyright 2017-2020 Espressif Systems (Shanghai) PTE LTD
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

#include "esp_timer_impl.h"
#include "esp_err.h"
#include "esp_timer.h"
#ifndef CONFIG_SOC_ESP32C3
#include "esp_intr_alloc.h"
#endif
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/periph_defs.h"
#include "freertos/FreeRTOS.h"
#include "hal/systimer_ll.h"
#include "hal/systimer_types.h"
#include "hal/systimer_hal.h"
#include <zephyr/zephyr.h>

#ifdef CONFIG_SOC_ESP32C3
#include <zephyr/drivers/interrupt_controller/intc_esp32c3.h>
#endif

/**
 * @file esp_timer_systimer.c
 * @brief Implementation of esp_timer using systimer.
 *
 * This timer is a 64-bit up-counting timer, with a programmable compare value (called 'alarm' hereafter).
 * When the timer reaches compare value, interrupt is raised.
 * The timer can be configured to produce an edge interrupt.
 *
 * @note systimer counter0 and alarm2 are adopted to implemented esp_timer
 */

/* Function from the upper layer to be called when the interrupt happens.
 * Registered in esp_timer_impl_init.
 */
static intr_handler_t s_alarm_handler = NULL;

/* Spinlock used to protect access to the hardware registers. */
static unsigned int s_time_update_lock;

#define SYS_TIMER_ESP_IRQ  17

void esp_timer_impl_lock(void)
{
    s_time_update_lock = irq_lock();
}

void esp_timer_impl_unlock(void)
{
    irq_unlock(s_time_update_lock);
}

uint64_t IRAM_ATTR esp_timer_impl_get_counter_reg(void)
{
    return systimer_hal_get_counter_value(SYSTIMER_COUNTER_0);
}

int64_t IRAM_ATTR esp_timer_impl_get_time(void)
{
    if (s_alarm_handler == NULL) {
        return 0;
    }
    return systimer_hal_get_time(SYSTIMER_COUNTER_0);
}

int64_t esp_timer_get_time(void) __attribute__((alias("esp_timer_impl_get_time")));

void IRAM_ATTR esp_timer_impl_set_alarm(uint64_t timestamp)
{
    systimer_hal_select_alarm_mode(SYSTIMER_ALARM_2, SYSTIMER_ALARM_MODE_ONESHOT);
    systimer_hal_set_alarm_target(SYSTIMER_ALARM_2, timestamp);
    systimer_hal_enable_alarm_int(SYSTIMER_ALARM_2);
}

static void IRAM_ATTR esp_timer_alarm_isr(void *arg)
{
    // clear the interrupt
    systimer_ll_clear_alarm_int(SYSTIMER_ALARM_2);
    /* Call the upper layer handler */
    (*s_alarm_handler)(arg);
}

void IRAM_ATTR esp_timer_impl_update_apb_freq(uint32_t apb_ticks_per_us)
{
    systimer_hal_on_apb_freq_update(apb_ticks_per_us);
}

void esp_timer_impl_advance(int64_t time_us)
{
    esp_timer_impl_lock();
    systimer_hal_counter_value_advance(SYSTIMER_COUNTER_0, time_us);
    esp_timer_impl_unlock();
}

esp_err_t esp_timer_impl_init(intr_handler_t alarm_handler)
{
    s_alarm_handler = alarm_handler;

    esp_intr_alloc(ETS_SYSTIMER_TARGET2_EDGE_INTR_SOURCE,
        0,
        (isr_handler_t)esp_timer_alarm_isr,
        NULL,
        NULL);

    systimer_hal_connect_alarm_counter(SYSTIMER_ALARM_2, SYSTIMER_COUNTER_0);
    systimer_hal_enable_counter(SYSTIMER_COUNTER_0);
    systimer_hal_select_alarm_mode(SYSTIMER_ALARM_2, SYSTIMER_ALARM_MODE_ONESHOT);
    systimer_hal_counter_can_stall_by_cpu(SYSTIMER_COUNTER_0, 0, true);

    /* TODO: if SYSTIMER is used for anything else, access to SYSTIMER_INT_ENA_REG has to be
    * protected by a shared spinlock. Since this code runs as part of early startup, this
    * is practically not an issue.
    */
    systimer_hal_enable_alarm_int(SYSTIMER_ALARM_2);
    irq_enable(SYS_TIMER_ESP_IRQ);

    return ESP_OK;
}

void esp_timer_impl_deinit(void)
{
    systimer_ll_disable_alarm(SYSTIMER_ALARM_2);
    /* TODO: may need a spinlock, see the note related to SYSTIMER_INT_ENA_REG in systimer_hal_init */
    systimer_ll_disable_alarm_int(SYSTIMER_ALARM_2);
    s_alarm_handler = NULL;
}

uint64_t IRAM_ATTR esp_timer_impl_get_min_period_us(void)
{
    return 50;
}

uint64_t esp_timer_impl_get_alarm_reg(void)
{
    esp_timer_impl_lock();
    uint64_t val = systimer_hal_get_alarm_value(SYSTIMER_ALARM_2);
    esp_timer_impl_unlock();
    return val;
}

void esp_timer_private_update_apb_freq(uint32_t apb_ticks_per_us) __attribute__((alias("esp_timer_impl_update_apb_freq")));
void esp_timer_private_advance(int64_t time_us) __attribute__((alias("esp_timer_impl_advance")));
void esp_timer_private_lock(void) __attribute__((alias("esp_timer_impl_lock")));
void esp_timer_private_unlock(void) __attribute__((alias("esp_timer_impl_unlock")));
