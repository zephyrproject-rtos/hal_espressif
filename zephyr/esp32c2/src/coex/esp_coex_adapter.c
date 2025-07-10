/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_private/wifi.h"
#include "soc/soc_caps.h"
#include "esp_attr.h"
#include "esp32c2/rom/ets_sys.h"
#include "esp_heap_adapter.h"
#include "esp_timer.h"
#include "soc/rtc.h"
#include "esp_private/esp_clk.h"
#include "private/esp_coexist_adapter.h"
#include "soc/system_reg.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(esp32_coex_adapter, CONFIG_WIFI_LOG_LEVEL);

#define OSI_FUNCS_TIME_BLOCKING 0xffffffff

void esp_wifi_free(void *mem)
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

void *esp_coex_common_spin_lock_create_wrapper(void)
{
	unsigned int *wifi_spin_lock = (unsigned int *)wifi_malloc(sizeof(unsigned int));
	if (wifi_spin_lock == NULL) {
		LOG_ERR("spin_lock_create_wrapper allocation failed");
	}

	return (void *)wifi_spin_lock;
}

uint32_t IRAM_ATTR esp_coex_common_int_disable_wrapper(void *wifi_int_mux)
{
	unsigned int *int_mux = (unsigned int *)wifi_int_mux;

	*int_mux = irq_lock();
	return 0;
}

void IRAM_ATTR esp_coex_common_int_restore_wrapper(void *wifi_int_mux, uint32_t tmp)
{
	unsigned int *key = (unsigned int *)wifi_int_mux;

	irq_unlock(*key);
}

void IRAM_ATTR esp_coex_common_task_yield_from_isr_wrapper(void)
{
	k_yield();
}

void *esp_coex_common_semphr_create_wrapper(uint32_t max, uint32_t init)
{
	struct k_sem *sem = (struct k_sem *)wifi_malloc(sizeof(struct k_sem));

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

void *IRAM_ATTR esp_coex_common_malloc_internal_wrapper(size_t size)
{
	return k_malloc(size);
}

uint32_t esp_coex_common_clk_slowclk_cal_get_wrapper(void)
{
	/* The bit width of WiFi light sleep clock calibration is 12 while the one of
	 * system is 19. It should shift 19 - 12 = 7.
	 */
	return (esp_clk_slowclk_cal_get() >> (RTC_CLK_CAL_FRACT - SOC_WIFI_LIGHT_SLEEP_CLK_WIDTH));
}

/* static wrapper */

static int32_t IRAM_ATTR esp_coex_semphr_take_from_isr_wrapper(void *semphr, void *hptw)
{
	int *hpt = (int *)hptw;

	int ret = k_sem_take((struct k_sem *)semphr, K_NO_WAIT);

	if (ret == 0) {
		return 1;
	}

	*hpt = 0;
	return 0;
}

static int32_t IRAM_ATTR esp_coex_semphr_give_from_isr_wrapper(void *semphr, void *hptw)
{
	int *hpt = (int *)hptw;

	k_sem_give((struct k_sem *)semphr);

	*hpt = 0;
	return 0;
}

static int esp_coexist_debug_matrix_init_wrapper(int evt, int sig, bool rev)
{
#if CONFIG_ESP_COEX_GPIO_DEBUG
	return esp_coexist_debug_matrix_init(evt, sig, rev);
#else
	return ESP_ERR_NOT_SUPPORTED;
#endif
}

static IRAM_ATTR int esp_coex_common_xtal_freq_get_wrapper(void)
{
	return rtc_clk_xtal_freq_get();
}

int32_t IRAM_ATTR coex_is_in_isr_wrapper(void)
{
	return k_is_in_isr();
}

coex_adapter_funcs_t g_coex_adapter_funcs = {
	._version = COEX_ADAPTER_VERSION,
	._task_yield_from_isr = esp_coex_common_task_yield_from_isr_wrapper,
	._semphr_create = esp_coex_common_semphr_create_wrapper,
	._semphr_delete = esp_coex_common_semphr_delete_wrapper,
	._semphr_take_from_isr = esp_coex_semphr_take_from_isr_wrapper,
	._semphr_give_from_isr = esp_coex_semphr_give_from_isr_wrapper,
	._semphr_take = esp_coex_common_semphr_take_wrapper,
	._semphr_give = esp_coex_common_semphr_give_wrapper,
	._is_in_isr = coex_is_in_isr_wrapper,
	._malloc_internal = esp_coex_common_malloc_internal_wrapper,
	._free = esp_wifi_free,
	._esp_timer_get_time = esp_timer_get_time,
	._env_is_chip = esp_coex_common_env_is_chip_wrapper,
	._slowclk_cal_get = esp_coex_common_clk_slowclk_cal_get_wrapper,
	._timer_disarm = esp_coex_common_timer_disarm_wrapper,
	._timer_done = esp_coex_common_timer_done_wrapper,
	._timer_setfn = esp_coex_common_timer_setfn_wrapper,
	._timer_arm_us = esp_coex_common_timer_arm_us_wrapper,
	._debug_matrix_init = esp_coexist_debug_matrix_init_wrapper,
	._xtal_freq_get = esp_coex_common_xtal_freq_get_wrapper,
	._magic = COEX_ADAPTER_MAGIC,
};
