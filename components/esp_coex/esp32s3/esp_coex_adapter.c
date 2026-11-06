/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/init.h>
#include <zephyr/spinlock.h>
#include <zephyr/logging/log.h>

#include "esp_attr.h"
#include "esp_timer.h"
#include "soc/rtc.h"
#include "soc/soc_caps.h"
#include "esp_private/esp_clk.h"
#include "private/esp_coexist_adapter.h"
#include "private/esp_coexist_internal.h"
#include "esp32s3/rom/ets_sys.h"
#include "soc/system_reg.h"
#include "private/esp_coexist_debug.h"
#include "esp_heap_adapter.h"

LOG_MODULE_REGISTER(esp32_coex_adapter, CONFIG_WIFI_LOG_LEVEL);

#define TAG "esp_coex_adapter"

#define OSI_FUNCS_TIME_BLOCKING  0xffffffff

typedef struct {
    void *handle;   /* k_sem pointer */
    void *storage;  /* unused in Zephyr */
} modem_static_queue_t;

static void esp_wifi_free(void *mem)
{
    esp_wifi_free_func(mem);
}

bool IRAM_ATTR esp_coex_common_env_is_chip_wrapper(void)
{
#ifdef CONFIG_IDF_ENV_FPGA
    return false;
#else
    return true;
#endif
}

/*
 * The blob's critical sections nest: the same wifi_int_mux can legitimately
 * be entered more than once by the same core before it is fully unwound
 * (this mirrors real ESP-IDF, whose portMUX_TYPE tracks an owner core plus a
 * nesting count for exactly this reason). A plain struct k_spinlock has no
 * notion of an owner, so a same-core re-entry would spin forever waiting for
 * itself to release it. Track the owning core and a nesting count instead: a
 * core already holding the lock just increments the count, and only the
 * outermost release (count reaching 0) actually gives it up.
 */
struct esp_wifi_int_mux {
    atomic_t owner;  /* 0 = free, otherwise (cpu id + 1) of the owner */
    uint32_t count;  /* nesting depth; only ever touched by the owner */
};

void * esp_coex_common_spin_lock_create_wrapper(void)
{
    struct esp_wifi_int_mux *mux =
        (struct esp_wifi_int_mux *)k_malloc(sizeof(struct esp_wifi_int_mux));

    if (mux == NULL) {
        LOG_ERR("spin_lock_create_wrapper allocation failed");
        return NULL;
    }

    mux->owner = ATOMIC_INIT(0);
    mux->count = 0;

    return (void *)mux;
}

/*
 * These guard the wifi blob's (libpp.a/libcore.a/libnet80211.a) internal
 * critical sections, which may be entered from either the wifi task or an
 * ISR. A plain irq_lock()/irq_unlock() pair only masks interrupts on the
 * calling core, which is safe under single-core builds but not under SMP:
 * the other core can still be mid-way through the same protected state.
 * irq_lock() still runs on every call (nests safely on its own, each call
 * gets back its own key to restore), and on top of it a core-owned, nesting-
 * aware spin loop gives real cross-core exclusion without the self-deadlock
 * a plain, non-recursive k_spinlock would hit on same-core re-entry.
 */
uint32_t IRAM_ATTR esp_coex_common_int_disable_wrapper(void *wifi_int_mux)
{
    struct esp_wifi_int_mux *mux = (struct esp_wifi_int_mux *)wifi_int_mux;
    unsigned int key = irq_lock();
    atomic_val_t me = (atomic_val_t)(arch_curr_cpu()->id + 1);

    if (atomic_get(&mux->owner) != me) {
        while (!atomic_cas(&mux->owner, 0, me)) {
            arch_nop();
        }
    }
    mux->count++;

    return (uint32_t)key;
}

void IRAM_ATTR esp_coex_common_int_restore_wrapper(void *wifi_int_mux, uint32_t tmp)
{
    struct esp_wifi_int_mux *mux = (struct esp_wifi_int_mux *)wifi_int_mux;

    if (--mux->count == 0) {
        atomic_set(&mux->owner, 0);
    }
    irq_unlock((unsigned int)tmp);
}

void IRAM_ATTR esp_coex_common_task_yield_from_isr_wrapper(void)
{
    k_yield();
}

void * esp_coex_common_semphr_create_wrapper(uint32_t max, uint32_t init)
{
    struct k_sem *sem = (struct k_sem *)k_malloc(sizeof(struct k_sem));

    if (sem == NULL) {
        LOG_ERR("semphr_create_wrapper allocation failed");
        return NULL;
    }

    k_sem_init(sem, init, max);

    return (void *)sem;
}

void esp_coex_common_semphr_delete_wrapper(void *semphr)
{
    esp_wifi_free(semphr);
}

int32_t esp_coex_common_semphr_take_wrapper(void *semphr, uint32_t block_time_tick)
{
    if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
        int ret = k_sem_take((struct k_sem *)semphr, K_FOREVER);
        if (ret == 0) {
            return 1;
        }
    } else {
        int ret = k_sem_take((struct k_sem *)semphr, K_TICKS(block_time_tick));
        if (ret == 0) {
            return 1;
        }
    }
    return 0;
}

int32_t esp_coex_common_semphr_give_wrapper(void *semphr)
{
    k_sem_give((struct k_sem *)semphr);
    return 1;
}

void IRAM_ATTR esp_coex_common_timer_disarm_wrapper(void *timer)
{
    ets_timer_disarm(timer);
}

void esp_coex_common_timer_done_wrapper(void *ptimer)
{
    ets_timer_done(ptimer);
}

void esp_coex_common_timer_setfn_wrapper(void *ptimer, void *pfunction, void *parg)
{
    ets_timer_setfn(ptimer, pfunction, parg);
}

void IRAM_ATTR esp_coex_common_timer_arm_us_wrapper(void *ptimer, uint32_t us, bool repeat)
{
    ets_timer_arm_us(ptimer, us, repeat);
}

void * IRAM_ATTR esp_coex_common_malloc_internal_wrapper(size_t size)
{
    return k_malloc(size);
}

uint32_t esp_coex_common_clk_slowclk_cal_get_wrapper(void)
{
    /* The bit width of WiFi light sleep clock calibration is 12 while the one of
     * system is 19. It should shift 19 - 12 = 7.
    */
    if (GET_PERI_REG_MASK(SYSTEM_BT_LPCK_DIV_FRAC_REG, SYSTEM_LPCLK_SEL_XTAL)) {
        uint64_t time_per_us = 1000000ULL;
        return (((time_per_us << RTC_CLK_CAL_FRACT) / (MHZ(1))) >> (RTC_CLK_CAL_FRACT - SOC_WIFI_LIGHT_SLEEP_CLK_WIDTH));
    } else {
        return (esp_clk_slowclk_cal_get() >> (RTC_CLK_CAL_FRACT - SOC_WIFI_LIGHT_SLEEP_CLK_WIDTH));
    }
}

/* static wrapper */

static int IRAM_ATTR esp_coex_is_in_isr_wrapper(void)
{
    return k_is_in_isr();
}

static void *esp_coex_internal_semphr_create_wrapper(uint32_t max, uint32_t init)
{
    modem_static_queue_t *semphr = k_calloc(1, sizeof(modem_static_queue_t));
    if (!semphr) {
        return NULL;
    }

    struct k_sem *sem = k_malloc(sizeof(struct k_sem));
    if (!sem) {
        k_free(semphr);
        return NULL;
    }

    k_sem_init(sem, init, max);
    semphr->handle = sem;

    return (void *)semphr;
}

static void esp_coex_internal_semphr_delete_wrapper(void *semphr)
{
    modem_static_queue_t *semphr_item = (modem_static_queue_t *)semphr;
    if (semphr_item) {
        if (semphr_item->handle) {
            k_free(semphr_item->handle);
        }
        k_free(semphr_item);
    }
}

static int32_t IRAM_ATTR esp_coex_internal_semphr_take_from_isr_wrapper(void *semphr, void *hptw)
{
    int *hpt = (int *)hptw;
    int ret = k_sem_take((struct k_sem *)((modem_static_queue_t *)semphr)->handle, K_NO_WAIT);

    if (ret == 0) {
        if (hpt) {
            *hpt = 0;
        }
        return 1;
    }

    if (hpt) {
        *hpt = 0;
    }
    return 0;
}

static int32_t IRAM_ATTR esp_coex_internal_semphr_give_from_isr_wrapper(void *semphr, void *hptw)
{
    int *hpt = (int *)hptw;

    k_sem_give((struct k_sem *)((modem_static_queue_t *)semphr)->handle);

    if (hpt) {
        *hpt = 0;
    }
    return 1;
}

static int32_t esp_coex_internal_semphr_take_wrapper(void *semphr, uint32_t block_time_tick)
{
    if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
        int ret = k_sem_take((struct k_sem *)((modem_static_queue_t *)semphr)->handle, K_FOREVER);
        if (ret == 0) {
            return 1;
        }
    } else {
        int ret = k_sem_take((struct k_sem *)((modem_static_queue_t *)semphr)->handle, K_TICKS(block_time_tick));
        if (ret == 0) {
            return 1;
        }
    }
    return 0;
}

static int32_t esp_coex_internal_semphr_give_wrapper(void *semphr)
{
    k_sem_give((struct k_sem *)((modem_static_queue_t *)semphr)->handle);
    return 1;
}

static int esp_coexist_debug_matrix_init_wrapper(int evt, int sig, bool rev)
{
#if CONFIG_ESP_COEX_GPIO_DEBUG
    return esp_coexist_debug_matrix_init(evt, sig, rev);
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

coex_adapter_funcs_t g_coex_adapter_funcs = {
    ._version = COEX_ADAPTER_VERSION,
    ._task_yield_from_isr = esp_coex_common_task_yield_from_isr_wrapper,
    ._semphr_create = esp_coex_internal_semphr_create_wrapper,
    ._semphr_delete = esp_coex_internal_semphr_delete_wrapper,
    ._semphr_take_from_isr = esp_coex_internal_semphr_take_from_isr_wrapper,
    ._semphr_give_from_isr = esp_coex_internal_semphr_give_from_isr_wrapper,
    ._semphr_take = esp_coex_internal_semphr_take_wrapper,
    ._semphr_give = esp_coex_internal_semphr_give_wrapper,
    ._is_in_isr = esp_coex_is_in_isr_wrapper,
    ._malloc_internal =  esp_coex_common_malloc_internal_wrapper,
    ._free = esp_wifi_free,
    ._esp_timer_get_time = esp_timer_get_time,
    ._env_is_chip = esp_coex_common_env_is_chip_wrapper,
    ._timer_disarm = esp_coex_common_timer_disarm_wrapper,
    ._timer_done = esp_coex_common_timer_done_wrapper,
    ._timer_setfn = esp_coex_common_timer_setfn_wrapper,
    ._timer_arm_us = esp_coex_common_timer_arm_us_wrapper,
    ._debug_matrix_init = esp_coexist_debug_matrix_init_wrapper,
    ._xtal_freq_get = esp_clk_xtal_freq,
    ._magic = COEX_ADAPTER_MAGIC,
};

#if CONFIG_ESP32_SW_COEXIST_ENABLE
/*
 * Register coex adapter functions early in boot, before Wi-Fi blob uses them.
 * ESP-IDF does this in init_coexist() at priority 204.
 */
static int esp_coex_adapter_init(void)
{
    esp_coex_adapter_register(&g_coex_adapter_funcs);
    coex_pre_init();
    return 0;
}

SYS_INIT(esp_coex_adapter_init, POST_KERNEL, 50);
#endif
