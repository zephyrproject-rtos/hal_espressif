/*
 * SPDX-FileCopyrightText: 2016-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <esp_heap_caps.h>

#include <stdlib.h>
#include <ctype.h>
#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_log.h"
#include "soc/rtc_periph.h"
#include "soc/syscon_periph.h"
#include "soc/rtc.h"
#include "soc/periph_defs.h"
#include "esp_intr_alloc.h"
#include "sys/lock.h"
#include "esp_private/rtc_ctrl.h"
#include "esp_attr.h"

#ifndef NDEBUG
// Enable built-in checks in queue.h in debug builds
#define INVARIANTS
#endif
#include "sys/queue.h"

#if CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32H2 // TODO: IDF-5645
static const char *TAG = "rtc_module";
#endif

#if !CONFIG_IDF_TARGET_ESP32C6 && !CONFIG_IDF_TARGET_ESP32H2 // TODO: IDF-5645

#define NOT_REGISTERED      (-1)

static DRAM_ATTR int s_rtc_isr_handler_list_lock;
#define RTC_ISR_HANDLER_ENTER_CRITICAL()    do { s_rtc_isr_handler_list_lock = irq_lock(); } while(0)
#define RTC_ISR_HANDLER_EXIT_CRITICAL()    irq_unlock(s_rtc_isr_handler_list_lock);

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
static intr_handle_t s_rtc_isr_handle;

IRAM_ATTR static void rtc_isr(void* arg)
{
    uint32_t status = REG_READ(RTC_CNTL_INT_ST_REG);
    rtc_isr_handler_t* it;
    RTC_ISR_HANDLER_ENTER_CRITICAL();
    SLIST_FOREACH(it, &s_rtc_isr_handler_list, next) {
        if (it->mask & status) {
            RTC_ISR_HANDLER_EXIT_CRITICAL();
            (*it->handler)(it->handler_arg);
            RTC_ISR_HANDLER_ENTER_CRITICAL();
        }
    }
    RTC_ISR_HANDLER_EXIT_CRITICAL();
    REG_WRITE(RTC_CNTL_INT_CLR_REG, status);
}

static esp_err_t rtc_isr_ensure_installed(void)
{
    esp_err_t err = ESP_OK;
    RTC_ISR_HANDLER_ENTER_CRITICAL();
    if (s_rtc_isr_handle) {
        goto out;
    }

    REG_WRITE(RTC_CNTL_INT_ENA_REG, 0);
    REG_WRITE(RTC_CNTL_INT_CLR_REG, UINT32_MAX);
    err = esp_intr_alloc(ETS_RTC_CORE_INTR_SOURCE, ESP_INTR_FLAG_IRAM, &rtc_isr, NULL, &s_rtc_isr_handle);
    if (err != ESP_OK) {
        goto out;
    }
    rtc_isr_cpu = esp_intr_get_cpu(s_rtc_isr_handle);
out:
    RTC_ISR_HANDLER_EXIT_CRITICAL();
    return err;
}
#endif // !CONFIG_IDF_TARGET_ESP32C6 TODO: IDF-5645

esp_err_t rtc_isr_register(intr_handler_t handler, void* handler_arg, uint32_t rtc_intr_mask, uint32_t flags)
{
#if CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32H2 // TODO: IDF-5645
    ESP_EARLY_LOGW(TAG, "rtc_isr_register() has not been implemented yet");
    return ESP_OK;
#else
    esp_err_t err = rtc_isr_ensure_installed();
    if (err != ESP_OK) {
        return err;
    }

    rtc_isr_handler_t* item = heap_caps_malloc(sizeof(*item), MALLOC_CAP_INTERNAL);
    if (item == NULL) {
        return ESP_ERR_NO_MEM;
    }
    item->handler = handler;
    item->handler_arg = handler_arg;
    item->mask = rtc_intr_mask;
    item->flags = flags;
    RTC_ISR_HANDLER_ENTER_CRITICAL();
    if (flags & RTC_INTR_FLAG_IRAM) {
        s_rtc_isr_noniram_hook(rtc_intr_mask);
    } else {
        s_rtc_isr_noniram_hook_relieve(rtc_intr_mask);
    }
    SLIST_INSERT_HEAD(&s_rtc_isr_handler_list, item, next);
    RTC_ISR_HANDLER_EXIT_CRITICAL();
    return ESP_OK;
#endif
}

esp_err_t rtc_isr_deregister(intr_handler_t handler, void* handler_arg)
{
#if CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32H2 // TODO: IDF-5645
    ESP_EARLY_LOGW(TAG, "rtc_isr_deregister() has not been implemented yet");
    return ESP_OK;
#else
    rtc_isr_handler_t* it;
    rtc_isr_handler_t* prev = NULL;
    bool found = false;
    RTC_ISR_HANDLER_ENTER_CRITICAL();
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
            free(it);
            break;
        }
        prev = it;
    }
    RTC_ISR_HANDLER_EXIT_CRITICAL();
    return found ? ESP_OK : ESP_ERR_INVALID_STATE;
#endif
}

#if !CONFIG_IDF_TARGET_ESP32C6 && !CONFIG_IDF_TARGET_ESP32H2 // TODO: IDF-5645
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
#if !CONFIG_IDF_TARGET_ESP32C6 && !CONFIG_IDF_TARGET_ESP32H2 // TODO: IDF-5645
    if (rtc_isr_cpu == cpu) {
        rtc_intr_enabled |= RTCCNTL.int_ena.val;
        RTCCNTL.int_ena.val &= rtc_intr_cache;
    }
#endif
}

IRAM_ATTR void rtc_isr_noniram_enable(uint32_t cpu)
{
#if !CONFIG_IDF_TARGET_ESP32C6 && !CONFIG_IDF_TARGET_ESP32H2 // TODO: IDF-5645
    if (rtc_isr_cpu == cpu) {
        RTCCNTL.int_ena.val = rtc_intr_enabled;
        rtc_intr_enabled = 0;
    }
#endif
}
