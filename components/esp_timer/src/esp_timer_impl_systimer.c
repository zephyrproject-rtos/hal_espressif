/*
 * SPDX-FileCopyrightText: 2017-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <sys/param.h>
#include "sdkconfig.h"
#include "esp_timer_impl.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_compiler.h"
#include "soc/periph_defs.h"
#include "soc/soc_caps.h"
#include "esp_private/esp_clk.h"
#include "esp_private/systimer.h"
#include "esp_private/periph_ctrl.h"
#include "hal/systimer_ll.h"
#include "hal/systimer_types.h"
#include "hal/systimer_hal.h"

#include <zephyr/drivers/interrupt_controller/intc_esp32.h>

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

static const char *TAG = "esp_timer_systimer";

#define NOT_USED 0xBAD00FAD

/* Function from the upper layer to be called when the interrupt happens.
 * Registered in esp_timer_impl_init.
 */
static intr_handler_t s_alarm_handler = NULL;

/* Systimer HAL layer object */
static systimer_hal_context_t systimer_hal;

/* Alarm values to generate interrupt on match */
extern uint64_t timestamp_id[2];

uint64_t IRAM_ATTR esp_timer_impl_get_counter_reg(void)
{
    return systimer_hal_get_counter_value(&systimer_hal, SYSTIMER_COUNTER_ESPTIMER);
}

int64_t IRAM_ATTR esp_timer_impl_get_time(void)
{
    // we hope the execution time of this function won't > 1us
    // thus, to save one function call, we didn't use the existing `systimer_hal_get_time`
    return systimer_hal.ticks_to_us(systimer_hal_get_counter_value(&systimer_hal, SYSTIMER_COUNTER_ESPTIMER));
}

int64_t esp_timer_get_time(void) __attribute__((alias("esp_timer_impl_get_time")));

void IRAM_ATTR esp_timer_impl_set_alarm_id(uint64_t timestamp, unsigned alarm_id)
{
    esp_timer_impl_lock();
    timestamp_id[alarm_id] = timestamp;
    timestamp = MIN(timestamp_id[0], timestamp_id[1]);
    systimer_hal_set_alarm_target(&systimer_hal, SYSTIMER_ALARM_ESPTIMER, timestamp);
    esp_timer_impl_unlock();
}

static void IRAM_ATTR timer_alarm_isr(void *arg)
{
    // clear the interrupt
    systimer_ll_clear_alarm_int(systimer_hal.dev, SYSTIMER_ALARM_ESPTIMER);
    /* Call the upper layer handler */
    (*s_alarm_handler)(arg);
}

void esp_timer_impl_set(uint64_t new_us)
{
    esp_timer_impl_lock();
    systimer_counter_value_t new_count = {
        .val = systimer_hal.us_to_ticks(new_us),
    };
    systimer_ll_set_counter_value(systimer_hal.dev, SYSTIMER_COUNTER_ESPTIMER, new_count.val);
    systimer_ll_apply_counter_value(systimer_hal.dev, SYSTIMER_COUNTER_ESPTIMER);
    esp_timer_impl_unlock();
}

void esp_timer_impl_advance(int64_t time_diff_us)
{
    esp_timer_impl_lock();
    systimer_hal_counter_value_advance(&systimer_hal, SYSTIMER_COUNTER_ESPTIMER, time_diff_us);
    esp_timer_impl_unlock();
}

esp_err_t esp_timer_impl_early_init(void)
{
    periph_module_enable(PERIPH_SYSTIMER_MODULE);
    systimer_hal_tick_rate_ops_t ops = {
        .ticks_to_us = systimer_ticks_to_us,
        .us_to_ticks = systimer_us_to_ticks,
    };
    systimer_hal_init(&systimer_hal);
    systimer_hal_set_tick_rate_ops(&systimer_hal, &ops);

#if !SOC_SYSTIMER_FIXED_DIVIDER
    assert(esp_clk_xtal_freq() == (40 * 1000000) &&
           "update the step for xtal to support other XTAL:APB frequency ratios");
    systimer_hal_set_steps_per_tick(&systimer_hal, 0, 2); // for xtal
    systimer_hal_set_steps_per_tick(&systimer_hal, 1, 1); // for pll
#endif

    systimer_hal_enable_counter(&systimer_hal, SYSTIMER_COUNTER_ESPTIMER);
    systimer_hal_select_alarm_mode(&systimer_hal, SYSTIMER_ALARM_ESPTIMER, SYSTIMER_ALARM_MODE_ONESHOT);
    systimer_hal_connect_alarm_counter(&systimer_hal, SYSTIMER_ALARM_ESPTIMER, SYSTIMER_COUNTER_ESPTIMER);

    for (unsigned cpuid = 0; cpuid < SOC_CPU_CORES_NUM; ++cpuid) {
        systimer_hal_counter_can_stall_by_cpu(&systimer_hal, SYSTIMER_COUNTER_ESPTIMER, cpuid, (cpuid < CONFIG_MP_MAX_NUM_CPUS) ? true : false);
    }

    return ESP_OK;
}

esp_err_t esp_timer_impl_init(intr_handler_t alarm_handler)
{
    int isr_flags = 0  /* ZEP-795 (GH #74368): esp_timer ISR priority relaxed to avoid
                        * IRQ not being allocated when several peripherals are enabled
                        */
#if !SOC_SYSTIMER_INT_LEVEL
                    | ESP_INTR_FLAG_EDGE
#endif
                    | ESP_INTR_FLAG_IRAM;

	esp_err_t err = esp_intr_alloc(ETS_SYSTIMER_TARGET2_EDGE_INTR_SOURCE, isr_flags,
								   (intr_handler_t)timer_alarm_isr, NULL, NULL);
	if (err != ESP_OK) {
        ESP_EARLY_LOGE(TAG, "esp_intr_alloc failed (0x%x)", err);
        return err;
    }

    if (s_alarm_handler == NULL) {
        s_alarm_handler = alarm_handler;
        /* TODO: if SYSTIMER is used for anything else, access to SYSTIMER_INT_ENA_REG has to be
        * protected by a shared spinlock. Since this code runs as part of early startup, this
        * is practically not an issue.
        */
        systimer_hal_enable_alarm_int(&systimer_hal, SYSTIMER_ALARM_ESPTIMER);
    }

    return err;
}

void esp_timer_impl_deinit(void)
{
    systimer_ll_enable_alarm(systimer_hal.dev, SYSTIMER_ALARM_ESPTIMER, false);
    /* TODO: may need a spinlock, see the note related to SYSTIMER_INT_ENA_REG in systimer_hal_init */
    systimer_ll_enable_alarm_int(systimer_hal.dev, SYSTIMER_ALARM_ESPTIMER, false);
    s_alarm_handler = NULL;
}

uint64_t esp_timer_impl_get_alarm_reg(void)
{
    esp_timer_impl_lock();
    uint64_t val = systimer_hal_get_alarm_value(&systimer_hal, SYSTIMER_ALARM_ESPTIMER);
    esp_timer_impl_unlock();
    return val;
}

void esp_timer_private_set(uint64_t new_us) __attribute__((alias("esp_timer_impl_set")));
void esp_timer_private_advance(int64_t time_diff_us) __attribute__((alias("esp_timer_impl_advance")));
