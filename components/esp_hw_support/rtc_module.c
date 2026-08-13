/*
 * SPDX-FileCopyrightText: 2016-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <ctype.h>
#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_log.h"
#include <zephyr/kernel.h>
#include "soc/rtc_periph.h"
#include "soc/rtc.h"
#include "soc/periph_defs.h"
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/dt-bindings/interrupt-controller/espressif-esp32-l3-intc.h>
#include <zephyr/irq.h>
#include <zephyr/irq_multilevel.h>
#include <zephyr/devicetree.h>
#include "sys/lock.h"
#include "esp_private/rtc_ctrl.h"
#include "esp_private/critical_section.h"
#include "esp_private/periph_ctrl.h"
#include "esp_attr.h"
#include "esp_cpu.h"

#if SOC_RTC_CNTL_NEEDS_ATOMIC_ACCESS
#define RTC_CNTL_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#define RTC_CNTL_ATOMIC()
#endif


#ifndef NDEBUG
// Enable built-in checks in queue.h in debug builds
#define INVARIANTS
#endif
#include "sys/queue.h"

#if !SOC_LP_PERIPH_SHARE_INTERRUPT // TODO: IDF-8008
ESP_LOG_ATTR_TAG(TAG, "rtc_module");
#endif

// rtc_spinlock is used by other peripheral drivers
esp_os_spinlock_t rtc_spinlock = ESP_OS_SPINLOCK_INIT;

#if SOC_LP_PERIPH_SHARE_INTERRUPT // TODO: IDF-8008

#define NOT_REGISTERED      (-1)

// Disable the interrupt which cannot work without cache.
static DRAM_ATTR uint32_t rtc_intr_cache;
static DRAM_ATTR uint32_t rtc_intr_enabled;
static DRAM_ATTR int rtc_isr_cpu = NOT_REGISTERED;  // Unused number
static void s_rtc_isr_noniram_hook(uint32_t rtc_intr_mask);
static void s_rtc_isr_noniram_hook_relieve(uint32_t rtc_intr_mask);

/*---------------------------------------------------------------
                        INTERRUPT HANDLER
---------------------------------------------------------------*/


typedef struct rtc_isr_handler_ {
    uint32_t mask;
    intr_handler_t handler;
    void* handler_arg;
    uint32_t flags;
    SLIST_ENTRY(rtc_isr_handler_) next;
} rtc_isr_handler_t;

static DRAM_ATTR SLIST_HEAD(rtc_isr_handler_list_, rtc_isr_handler_) s_rtc_isr_handler_list =
        SLIST_HEAD_INITIALIZER(s_rtc_isr_handler_list);
static DRAM_ATTR esp_os_spinlock_t s_rtc_isr_handler_list_lock = ESP_OS_SPINLOCK_INIT;

/*
 * Where the RTC_CNTL bits have their own consumers, RTC_CORE_INTR_SOURCE feeds
 * the rtc_l3_intc aggregator and rtc_isr() runs from its last input, a
 * catch-all after the inputs that own a bit. Otherwise rtc_isr() takes the
 * source as a level-2 IRQ.
 */
#if DT_NODE_EXISTS(DT_NODELABEL(rtc_l3_intc))
#define RTC_ISR_NODE DT_NODELABEL(rtc_l3_intc)
BUILD_ASSERT(DT_PROP_LAST(RTC_ISR_NODE, bits) ==
             ESP_L3_CATCH_ALL, "the last rtc_l3_intc input must be ESP_L3_CATCH_ALL");
#define RTC_ISR_IRQ \
    (irq_to_level_3(DT_PROP_LEN(RTC_ISR_NODE, bits) - 1) | DT_IRQN(RTC_ISR_NODE))
#else
#define RTC_ISR_IRQ \
    (irq_to_level_2(ETS_RTC_CORE_INTR_SOURCE) | DT_IRQ(DT_NODELABEL(intmux), irq))
#endif

IRAM_ATTR static void rtc_isr(void* arg)
{
    uint32_t status = REG_READ(RTC_CNTL_INT_ST_REG);
    rtc_isr_handler_t* it;
    esp_os_enter_critical_isr(&s_rtc_isr_handler_list_lock);
    SLIST_FOREACH(it, &s_rtc_isr_handler_list, next) {
        if (it->mask & status) {
            esp_os_exit_critical_isr(&s_rtc_isr_handler_list_lock);
            (*it->handler)(it->handler_arg);
            esp_os_enter_critical_isr(&s_rtc_isr_handler_list_lock);
        }
    }
    esp_os_exit_critical_isr(&s_rtc_isr_handler_list_lock);
    REG_WRITE(RTC_CNTL_INT_CLR_REG, status);
}

static esp_err_t rtc_isr_ensure_installed(void)
{
    esp_err_t err = ESP_OK;
    esp_os_enter_critical(&s_rtc_isr_handler_list_lock);
    if (rtc_isr_cpu != NOT_REGISTERED) {
        goto out;
    }

    /*
     * RTC_CNTL_INT_ENA_REG is left alone: drivers on the other rtc_l3_intc
     * inputs own some of its bits. rtc_isr_register() callers enable their own.
     */
    irq_connect_dynamic(RTC_ISR_IRQ, 1, (void (*)(const void *))rtc_isr, NULL, ESP_INTR_FLAG_IRAM);
    irq_enable(RTC_ISR_IRQ);

    rtc_isr_cpu = esp_cpu_get_core_id();
out:
    esp_os_exit_critical(&s_rtc_isr_handler_list_lock);
    return err;
}
#endif // SOC_LP_PERIPH_SHARE_INTERRUPT TODO: IDF-8008

esp_err_t rtc_isr_register(intr_handler_t handler, void* handler_arg, uint32_t rtc_intr_mask, uint32_t flags)
{
#if !SOC_LP_PERIPH_SHARE_INTERRUPT // TODO: IDF-8008
    ESP_EARLY_LOGW(TAG, "rtc_isr_register() has not been implemented yet");
    return ESP_OK;
#else
    esp_err_t err = rtc_isr_ensure_installed();
    if (err != ESP_OK) {
        return err;
    }

    rtc_isr_handler_t* item = k_malloc(sizeof(*item));
    if (item == NULL) {
        return ESP_ERR_NO_MEM;
    }
    item->handler = handler;
    item->handler_arg = handler_arg;
    item->mask = rtc_intr_mask;
    item->flags = flags;
    esp_os_enter_critical(&s_rtc_isr_handler_list_lock);
    if (flags & RTC_INTR_FLAG_IRAM) {
        s_rtc_isr_noniram_hook(rtc_intr_mask);
    } else {
        s_rtc_isr_noniram_hook_relieve(rtc_intr_mask);
    }
    SLIST_INSERT_HEAD(&s_rtc_isr_handler_list, item, next);
    esp_os_exit_critical(&s_rtc_isr_handler_list_lock);
    return ESP_OK;
#endif
}

esp_err_t rtc_isr_deregister(intr_handler_t handler, void* handler_arg)
{
#if !SOC_LP_PERIPH_SHARE_INTERRUPT // TODO: IDF-8008
    ESP_EARLY_LOGW(TAG, "rtc_isr_deregister() has not been implemented yet");
    return ESP_OK;
#else
    rtc_isr_handler_t* it;
    rtc_isr_handler_t* prev = NULL;
    bool found = false;
    esp_os_enter_critical(&s_rtc_isr_handler_list_lock);
    SLIST_FOREACH(it, &s_rtc_isr_handler_list, next) {
        if (it->handler == handler && it->handler_arg == handler_arg) {
            if (it == SLIST_FIRST(&s_rtc_isr_handler_list)) {
                SLIST_REMOVE_HEAD(&s_rtc_isr_handler_list, next);
            } else {
                SLIST_REMOVE_AFTER(prev, next);
            }
            found = true;
            if (it->flags & RTC_INTR_FLAG_IRAM) {
                s_rtc_isr_noniram_hook_relieve(it->mask);
            }
            k_free(it);
            break;
        }
        prev = it;
    }
    esp_os_exit_critical(&s_rtc_isr_handler_list_lock);
    return found ? ESP_OK : ESP_ERR_INVALID_STATE;
#endif
}

#if SOC_LP_PERIPH_SHARE_INTERRUPT // TODO: IDF-8008
/**
 * @brief This helper function can be used to avoid the interrupt to be triggered with cache disabled.
 *        There are lots of different signals on RTC module (i.e. sleep_wakeup, wdt, brownout_detect, etc.)
 *        We might want some of them can be triggered with cache disabled, some are not. Therefore, this function
 *        is created to avoid those which do not want to be triggered with cache disabled.
 *
 * @param rtc_intr_mask the mask of the rtc interrupt.
 */
static void s_rtc_isr_noniram_hook(uint32_t rtc_intr_mask)
{
    rtc_intr_cache |= rtc_intr_mask;
}

static void s_rtc_isr_noniram_hook_relieve(uint32_t rtc_intr_mask)
{
    rtc_intr_cache &= ~rtc_intr_mask;
}
#endif


IRAM_ATTR void rtc_isr_noniram_disable(uint32_t cpu)
{
#if SOC_LP_PERIPH_SHARE_INTERRUPT // TODO: IDF-8008
    if (rtc_isr_cpu == cpu) {
        RTC_CNTL_ATOMIC() {
            rtc_intr_enabled |= RTCCNTL.int_ena.val;
            RTCCNTL.int_ena.val &= rtc_intr_cache;
        }
    }
#endif
}

IRAM_ATTR void rtc_isr_noniram_enable(uint32_t cpu)
{
#if SOC_LP_PERIPH_SHARE_INTERRUPT // TODO: IDF-8008
    if (rtc_isr_cpu == cpu) {
        RTC_CNTL_ATOMIC() {
            RTCCNTL.int_ena.val = rtc_intr_enabled;
            rtc_intr_enabled = 0;
        }
    }
#endif
}
