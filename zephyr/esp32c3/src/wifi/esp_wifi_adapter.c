/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define CONFIG_POSIX_FS

#include "esp_wifi.h"
#include "stdlib.h"
#include "string.h"
#include "esp_private/wifi.h"
#include "soc/soc.h"
#include "soc/dport_access.h"
#include "soc/wdev_reg.h"
#include "soc/soc_caps.h"
#include "soc/rtc.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_private/wifi_os_adapter.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "os.h"
#include "esp_event.h"
#include "esp_wpa.h"
#include "driver/periph_ctrl.h"
#include "esp_phy_init.h"
#include <soc/syscon_reg.h>
#include <rom/efuse.h>
#include <rom/rtc.h>
#include <rom/ets_sys.h>
#include <riscv/interrupt.h>

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/random/rand32.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32c3.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(esp32_wifi_adapter, CONFIG_WIFI_LOG_LEVEL);

K_THREAD_STACK_DEFINE(wifi_stack, 4096);

ESP_EVENT_DEFINE_BASE(WIFI_EVENT);

static void esp_wifi_free(void *mem);

static void *wifi_msgq_buffer;

static struct k_thread wifi_task_handle;

#if CONFIG_IDF_TARGET_ESP32
extern wifi_mac_time_update_cb_t s_wifi_mac_time_update_cb;
#endif

uint64_t g_wifi_feature_caps =
#if CONFIG_ESP32_WIFI_ENABLE_WPA3_SAE
    CONFIG_FEATURE_WPA3_SAE_BIT |
#endif
#if (CONFIG_ESP32_SPIRAM_SUPPORT || CONFIG_ESP32S2_SPIRAM_SUPPORT || CONFIG_ESP32S3_SPIRAM_SUPPORT)
    CONFIG_FEATURE_CACHE_TX_BUF_BIT |
#endif
#if CONFIG_ESP_WIFI_FTM_INITIATOR_SUPPORT
    CONFIG_FEATURE_FTM_INITIATOR_BIT |
#endif
#if CONFIG_ESP_WIFI_FTM_RESPONDER_SUPPORT
    CONFIG_FEATURE_FTM_RESPONDER_BIT |
#endif
0;

IRAM_ATTR void *wifi_malloc(size_t size)
{
	void *ptr = k_malloc(size);

	if (ptr == NULL) {
		LOG_ERR("memory allocation failed");
	}

	return ptr;
}

IRAM_ATTR void *wifi_realloc(void *ptr, size_t size)
{
	ARG_UNUSED(ptr);
	ARG_UNUSED(size);

	LOG_ERR("%s not yet supported", __func__);
	return NULL;
}

IRAM_ATTR void *wifi_calloc(size_t n, size_t size)
{
	void *ptr = k_calloc(n, size);

	if (ptr == NULL) {
		LOG_ERR("memory allocation failed");
	}

	return ptr;
}

static void *IRAM_ATTR wifi_zalloc_wrapper(size_t size)
{
	return wifi_calloc(1, size);
}

wifi_static_queue_t *wifi_create_queue(int queue_len, int item_size)
{
	wifi_static_queue_t *queue = NULL;

	queue = (wifi_static_queue_t *) wifi_malloc(sizeof(wifi_static_queue_t));
	if (!queue) {
		LOG_ERR("msg buffer allocation failed");
		return NULL;
	}

	wifi_msgq_buffer = wifi_malloc(queue_len * item_size);
	if (wifi_msgq_buffer == NULL) {
		LOG_ERR("msg buffer allocation failed");
		return NULL;
	}

	queue->handle = wifi_malloc(sizeof(struct k_msgq));
	if (queue->handle == NULL) {
		esp_wifi_free(wifi_msgq_buffer);
		LOG_ERR("queue handle allocation failed");
		return NULL;
	}

	k_msgq_init((struct k_msgq *)queue->handle, wifi_msgq_buffer, item_size, queue_len);

	return queue;
}

void wifi_delete_queue(wifi_static_queue_t *queue)
{
	if (queue) {
		esp_wifi_free(queue->handle);
		esp_wifi_free(queue);
	}
}

static void *wifi_create_queue_wrapper(int queue_len, int item_size)
{
	return wifi_create_queue(queue_len, item_size);
}

static void wifi_delete_queue_wrapper(void *queue)
{
	wifi_delete_queue(queue);
}

static bool IRAM_ATTR env_is_chip_wrapper(void)
{
#ifdef CONFIG_IDF_ENV_FPGA
	return false;
#else
	return true;
#endif
}

static void *spin_lock_create_wrapper(void)
{
	unsigned int *wifi_spin_lock = (unsigned int *) wifi_malloc(sizeof(unsigned int));
	if (wifi_spin_lock == NULL) {
		LOG_ERR("spin_lock_create_wrapper allocation failed");
	}

	return (void *)wifi_spin_lock;
}

static uint32_t IRAM_ATTR wifi_int_disable_wrapper(void *wifi_int_mux)
{
	unsigned int *int_mux = (unsigned int *) wifi_int_mux;

	*int_mux = irq_lock();
	return 0;
}

static void IRAM_ATTR wifi_int_restore_wrapper(void *wifi_int_mux, uint32_t tmp)
{
	unsigned int *key = (unsigned int *) wifi_int_mux;

	irq_unlock(*key);
}

static void IRAM_ATTR task_yield_from_isr_wrapper(void)
{
	k_yield();
}

static void *semphr_create_wrapper(uint32_t max, uint32_t init)
{
	struct k_sem *sem = (struct k_sem *) wifi_malloc(sizeof(struct k_sem));

	if (sem == NULL) {
		LOG_ERR("semphr_create_wrapper allocation failed");
	}

	k_sem_init(sem, init, max);
	return (void *) sem;
}

static void semphr_delete_wrapper(void *semphr)
{
	esp_wifi_free(semphr);
}

static void *wifi_thread_semphr_get_wrapper(void)
{
	struct k_sem *sem = NULL;

	sem = k_thread_custom_data_get();
	if (!sem) {
		sem = (struct k_sem *) wifi_malloc(sizeof(struct k_sem));
		if (sem == NULL) {
			LOG_ERR("wifi_thread_semphr_get_wrapper allocation failed");
		}
		k_sem_init(sem, 0, 1);
		if (sem) {
			k_thread_custom_data_set(sem);
		}
	}
	return (void *)sem;
}

static int32_t semphr_take_wrapper(void *semphr, uint32_t block_time_tick)
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

static int32_t semphr_give_wrapper(void *semphr)
{
	k_sem_give((struct k_sem *) semphr);
	return 1;
}

static void *recursive_mutex_create_wrapper(void)
{
	struct k_mutex *my_mutex = (struct k_mutex *) wifi_malloc(sizeof(struct k_mutex));

	if (my_mutex == NULL) {
		LOG_ERR("recursive_mutex_create_wrapper allocation failed");
	}

	k_mutex_init(my_mutex);

	return (void *)my_mutex;
}

static void *mutex_create_wrapper(void)
{
	struct k_mutex *my_mutex = (struct k_mutex *) wifi_malloc(sizeof(struct k_mutex));

	if (my_mutex == NULL) {
		LOG_ERR("mutex_create_wrapper allocation failed");
	}

	k_mutex_init(my_mutex);

	return (void *)my_mutex;
}

static void mutex_delete_wrapper(void *mutex)
{
	esp_wifi_free(mutex);
}

static int32_t IRAM_ATTR mutex_lock_wrapper(void *mutex)
{
	struct k_mutex *my_mutex = (struct k_mutex *) mutex;

	k_mutex_lock(my_mutex, K_FOREVER);
	return 0;
}

static int32_t IRAM_ATTR mutex_unlock_wrapper(void *mutex)
{
	struct k_mutex *my_mutex = (struct k_mutex *) mutex;

	k_mutex_unlock(my_mutex);
	return 0;
}

static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size)
{
	struct k_queue *queue = (struct k_queue *) wifi_malloc(sizeof(struct k_queue));

	if (queue == NULL) {
		LOG_ERR("queue malloc failed");
		return NULL;
	}

	k_msgq_init((struct k_msgq *)queue, wifi_msgq_buffer, item_size, queue_len);

	return (void *)queue;
}

static void delete_wrapper(void *handle)
{
	esp_wifi_free(handle);
}

static int32_t queue_send_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
	if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
		k_msgq_put((struct k_msgq *)queue, item, K_FOREVER);
	} else {
		k_msgq_put((struct k_msgq *)queue, item, K_TICKS(block_time_tick));
	}
	return 1;
}

static int32_t IRAM_ATTR queue_send_from_isr_wrapper(void *queue, void *item, void *hptw)
{
	int *hpt = (int *) hptw;

	k_msgq_put((struct k_msgq *)queue, item, K_NO_WAIT);
	*hpt = 0;
	return 1;
}

int32_t queue_send_to_back_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
	ARG_UNUSED(queue);
	ARG_UNUSED(item);
	ARG_UNUSED(block_time_tick);

	return 0;
}

int32_t queue_send_to_front_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
	ARG_UNUSED(queue);
	ARG_UNUSED(item);
	ARG_UNUSED(block_time_tick);

	return 0;
}

static int32_t queue_recv_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
	if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
		k_msgq_get((struct k_msgq *)queue, item, K_FOREVER);
	} else {
		k_msgq_get((struct k_msgq *)queue, item, K_MSEC(block_time_tick));
	}
	return 1;
}

static uint32_t event_group_wait_bits_wrapper(void *event, uint32_t bits_to_wait_for, int clear_on_exit, int wait_for_all_bits, uint32_t block_time_tick)
{
	ARG_UNUSED(event);
	ARG_UNUSED(bits_to_wait_for);
	ARG_UNUSED(clear_on_exit);
	ARG_UNUSED(wait_for_all_bits);
	ARG_UNUSED(block_time_tick);

	return 0;
}

static int32_t task_create_pinned_to_core_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle, uint32_t core_id)
{
	k_tid_t tid = k_thread_create(&wifi_task_handle, wifi_stack, stack_depth,
				      (k_thread_entry_t)task_func, param, NULL, NULL,
				      prio, K_INHERIT_PERMS, K_NO_WAIT);

	k_thread_name_set(tid, name);

	*(int32_t *)task_handle = (int32_t) tid;
	return 1;
}

static int32_t task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle)
{
	k_tid_t tid = k_thread_create(&wifi_task_handle, wifi_stack, stack_depth,
				      (k_thread_entry_t)task_func, param, NULL, NULL,
				      prio, K_INHERIT_PERMS, K_NO_WAIT);

	k_thread_name_set(tid, name);

	*(int32_t *)task_handle = (int32_t) tid;
	return 1;
}

static int32_t IRAM_ATTR task_ms_to_tick_wrapper(uint32_t ms)
{
	return (int32_t)(k_ms_to_ticks_ceil32(ms));
}

static int32_t task_get_max_priority_wrapper(void)
{
	return (int32_t)(4);
}

static int32_t esp_event_post_wrapper(const char* event_base, int32_t event_id, void* event_data, size_t event_data_size, uint32_t ticks_to_wait)
{
	ARG_UNUSED(event_base);
	ARG_UNUSED(event_id);
	ARG_UNUSED(event_data);
	ARG_UNUSED(event_data_size);
	ARG_UNUSED(ticks_to_wait);

	LOG_ERR("%s not yet supported", __func__);
	return 0;
}

static void IRAM_ATTR wifi_apb80m_request_wrapper(void)
{
#ifdef CONFIG_PM_ENABLE
	wifi_apb80m_request();
#endif
}

static void IRAM_ATTR wifi_apb80m_release_wrapper(void)
{
#ifdef CONFIG_PM_ENABLE
	wifi_apb80m_release();
#endif
}

static void IRAM_ATTR timer_arm_wrapper(void *timer, uint32_t tmout, bool repeat)
{
	ets_timer_arm(timer, tmout, repeat);
}

static void IRAM_ATTR timer_disarm_wrapper(void *timer)
{
	ets_timer_disarm(timer);
}

static void timer_done_wrapper(void *ptimer)
{
	ets_timer_done(ptimer);
}

static void timer_setfn_wrapper(void *ptimer, void *pfunction, void *parg)
{
	ets_timer_setfn(ptimer, pfunction, parg);
}

static void IRAM_ATTR timer_arm_us_wrapper(void *ptimer, uint32_t us, bool repeat)
{
	ets_timer_arm_us(ptimer, us, repeat);
}

static int get_time_wrapper(void *t)
{
	return os_get_time(t);
}

static void *IRAM_ATTR malloc_internal_wrapper(size_t size)
{
	return wifi_malloc(size);
}

static void *IRAM_ATTR realloc_internal_wrapper(void *ptr, size_t size)
{
	ARG_UNUSED(ptr);
	ARG_UNUSED(size);

	LOG_ERR("%s not yet supported", __func__);
	return NULL;
}

static void *IRAM_ATTR calloc_internal_wrapper(size_t n, size_t size)
{
	return wifi_calloc(n, size);
}

static void *IRAM_ATTR zalloc_internal_wrapper(size_t size)
{
	return wifi_calloc(1, size);
}

uint32_t uxQueueMessagesWaiting(void *queue)
{
	ARG_UNUSED(queue);

	return 0;
}

void *xEventGroupCreate(void)
{
	LOG_ERR("EventGroup not supported!");
	return NULL;
}

void vEventGroupDelete(void *grp)
{
	ARG_UNUSED(grp);
}

uint32_t xEventGroupSetBits(void *ptr, uint32_t data)
{
	ARG_UNUSED(ptr);
	ARG_UNUSED(data);

	return 0;
}

uint32_t xEventGroupClearBits(void *ptr, uint32_t data)
{
	ARG_UNUSED(ptr);
	ARG_UNUSED(data);

	return 0;
}

void *xTaskGetCurrentTaskHandle(void)
{
	return (void *)k_current_get();
}

void task_delay(uint32_t ticks)
{
	k_sleep(K_TICKS(ticks));
}

static void set_intr_wrapper(int32_t cpu_no, uint32_t intr_source, uint32_t intr_num, int32_t intr_prio)
{
	ARG_UNUSED(cpu_no);

    REG_WRITE(DR_REG_INTERRUPT_BASE + 4 * intr_source, intr_num);
    esprv_intc_int_set_priority(intr_num, intr_prio);
    esprv_intc_int_set_type(intr_num, INTR_TYPE_LEVEL);
}

static void clear_intr_wrapper(uint32_t intr_source, uint32_t intr_num)
{
	ARG_UNUSED(intr_source);
	ARG_UNUSED(intr_num);
}

static void set_isr_wrapper(int32_t n, void *f, void *arg)
{
	ARG_UNUSED(n);

	/* workaround to force allocating same handler for wifi interrupts */
	esp_intr_alloc(0, 0, (isr_handler_t)f, arg, NULL);
	esp_intr_alloc(2, 0, (isr_handler_t)f, arg,	NULL);
}

static void intr_on(unsigned int mask)
{
	esp_intr_enable(mask);
}

static void intr_off(unsigned int mask)
{
	esp_intr_disable(mask);
}

uint32_t esp_get_free_heap_size(void)
{
	/* FIXME: API to get free heap size is not available in Zephyr. */
	/* It is only used by ESP-MESH feature (not supported yet) */
	return 10000;
}

static unsigned long random(void)
{
	return sys_rand32_get();
}

static void wifi_clock_enable_wrapper(void)
{
	wifi_module_enable();
}

static void wifi_clock_disable_wrapper(void)
{
	wifi_module_disable();
}

static void wifi_reset_mac_wrapper(void)
{
	SET_PERI_REG_MASK(SYSCON_WIFI_RST_EN_REG, SYSTEM_MAC_RST);
	CLEAR_PERI_REG_MASK(SYSCON_WIFI_RST_EN_REG, SYSTEM_MAC_RST);
}

int32_t nvs_set_i8(uint32_t handle, const char *key, int8_t value)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(key);
	ARG_UNUSED(value);

	return 0;
}

int32_t nvs_get_i8(uint32_t handle, const char *key, int8_t *out_value)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(key);
	ARG_UNUSED(out_value);

	return 0;
}

int32_t nvs_set_u8(uint32_t handle, const char *key, uint8_t value)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(key);
	ARG_UNUSED(value);

	return 0;
}

int32_t nvs_get_u8(uint32_t handle, const char *key, uint8_t *out_value)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(key);
	ARG_UNUSED(out_value);

	return 0;
}

int32_t nvs_set_u16(uint32_t handle, const char *key, uint16_t value)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(key);
	ARG_UNUSED(value);

	return 0;
}

int32_t nvs_get_u16(uint32_t handle, const char *key, uint16_t *out_value)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(key);
	ARG_UNUSED(out_value);

	return 0;
}

int32_t nvs_open(const char *name, uint32_t open_mode, uint32_t *out_handle)
{
	ARG_UNUSED(name);
	ARG_UNUSED(open_mode);
	ARG_UNUSED(out_handle);

	return 0;
}

void nvs_close(uint32_t handle)
{
	ARG_UNUSED(handle);

	return;
}

int32_t nvs_commit(uint32_t handle)
{
	ARG_UNUSED(handle);

	return 0;
}

int32_t nvs_set_blob(uint32_t handle, const char *key, const void *value,
		     size_t length)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(key);
	ARG_UNUSED(value);
	ARG_UNUSED(length);

	return 0;
}

int32_t nvs_get_blob(uint32_t handle, const char *key, void *out_value,
		     size_t *length)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(key);
	ARG_UNUSED(out_value);
	ARG_UNUSED(length);

	return 0;
}

int32_t nvs_erase_key(uint32_t handle, const char *key)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(key);

	return 0;
}

static int coex_init_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_init();
#else
	return 0;
#endif
}

static void coex_deinit_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	coex_deinit();
#endif
}

static int coex_enable_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_enable();
#else
	return 0;
#endif
}

static void coex_disable_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	coex_disable();
#endif
}

static IRAM_ATTR uint32_t coex_status_get_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_status_get();
#else
	return 0;
#endif
}

static void coex_condition_set_wrapper(uint32_t type, bool dissatisfy)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	coex_condition_set(type, dissatisfy);
#endif
}

static int coex_wifi_request_wrapper(uint32_t event, uint32_t latency, uint32_t duration)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_wifi_request(event, latency, duration);
#else
	return 0;
#endif
}

static IRAM_ATTR int coex_wifi_release_wrapper(uint32_t event)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_wifi_release(event);
#else
	return 0;
#endif
}

static int coex_wifi_channel_set_wrapper(uint8_t primary, uint8_t secondary)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_wifi_channel_set(primary, secondary);
#else
	return 0;
#endif
}

static IRAM_ATTR int coex_event_duration_get_wrapper(uint32_t event, uint32_t *duration)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_event_duration_get(event, duration);
#else
	return 0;
#endif
}

static int coex_pti_get_wrapper(uint32_t event, uint8_t *pti)
{
	ARG_UNUSED(event);
	ARG_UNUSED(pti);

	return 0;
}

static void coex_schm_status_bit_clear_wrapper(uint32_t type, uint32_t status)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	coex_schm_status_bit_clear(type, status);
#endif
}

static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	coex_schm_status_bit_set(type, status);
#endif
}

static IRAM_ATTR int coex_schm_interval_set_wrapper(uint32_t interval)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_schm_interval_set(interval);
#else
	return 0;
#endif
}

static uint32_t coex_schm_interval_get_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_schm_interval_get();
#else
	return 0;
#endif
}

static uint8_t coex_schm_curr_period_get_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_schm_curr_period_get();
#else
	return 0;
#endif
}

static void * coex_schm_curr_phase_get_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_schm_curr_phase_get();
#else
	return NULL;
#endif
}

static int coex_schm_curr_phase_idx_set_wrapper(int idx)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_schm_curr_phase_idx_set(idx);
#else
	return 0;
#endif
}

static int coex_schm_curr_phase_idx_get_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_schm_curr_phase_idx_get();
#else
	return 0;
#endif
}

static void IRAM_ATTR esp_empty_wrapper(void)
{

}

int32_t IRAM_ATTR coex_is_in_isr_wrapper(void)
{
	return !k_is_in_isr();
}

static uint32_t esp_clk_slowclk_cal_get_wrapper(void)
{
	/* The bit width of WiFi light sleep clock calibration is 12 while the one of
	 * system is 19. It should shift 19 - 12 = 7.
	 */
	return (REG_READ(RTC_SLOW_CLK_CAL_REG) >> (RTC_CLK_CAL_FRACT - SOC_WIFI_LIGHT_SLEEP_CLK_WIDTH));
}

static void esp_log_writev_wrapper(uint32_t level, const char *tag, const char *format, va_list args)
{
#if CONFIG_WIFI_LOG_LEVEL >= LOG_LEVEL_DBG
	esp_log_writev((esp_log_level_t)level,tag,format,args);
#endif
}

static void esp_log_write_wrapper(uint32_t level,const char *tag,const char *format, ...)
{
#if CONFIG_WIFI_LOG_LEVEL >= LOG_LEVEL_DBG
	va_list list;
	va_start(list, format);
	esp_log_writev((esp_log_level_t)level, tag, format, list);
	va_end(list);
#endif
}

wifi_osi_funcs_t g_wifi_osi_funcs = {
	._version = ESP_WIFI_OS_ADAPTER_VERSION,
	._env_is_chip = env_is_chip_wrapper,
	._set_intr = set_intr_wrapper,
	._clear_intr = clear_intr_wrapper,
	._set_isr = set_isr_wrapper,
	._ints_on = intr_on,
	._ints_off = intr_off,
	._is_from_isr = k_is_in_isr,
	._spin_lock_create = spin_lock_create_wrapper,
	._spin_lock_delete = esp_wifi_free,
	._wifi_int_disable = wifi_int_disable_wrapper,
	._wifi_int_restore = wifi_int_restore_wrapper,
	._task_yield_from_isr = task_yield_from_isr_wrapper,
	._semphr_create = semphr_create_wrapper,
	._semphr_delete = semphr_delete_wrapper,
	._semphr_take = semphr_take_wrapper,
	._semphr_give = semphr_give_wrapper,
	._wifi_thread_semphr_get = wifi_thread_semphr_get_wrapper,
	._mutex_create = mutex_create_wrapper,
	._recursive_mutex_create = recursive_mutex_create_wrapper,
	._mutex_delete = mutex_delete_wrapper,
	._mutex_lock = mutex_lock_wrapper,
	._mutex_unlock = mutex_unlock_wrapper,
	._queue_create = queue_create_wrapper,
	._queue_delete = delete_wrapper,
	._queue_send = queue_send_wrapper,
	._queue_send_from_isr = queue_send_from_isr_wrapper,
	._queue_send_to_back = queue_send_to_back_wrapper,
	._queue_send_to_front = queue_send_to_front_wrapper,
	._queue_recv = queue_recv_wrapper,
	._queue_msg_waiting = uxQueueMessagesWaiting,
	._event_group_create = (void *(*)(void))xEventGroupCreate,
	._event_group_delete = (void (*)(void *))vEventGroupDelete,
	._event_group_set_bits = xEventGroupSetBits,
	._event_group_clear_bits = xEventGroupClearBits,
	._event_group_wait_bits = event_group_wait_bits_wrapper,
	._task_create_pinned_to_core = task_create_pinned_to_core_wrapper,
	._task_create = task_create_wrapper,
	._task_delete = delete_wrapper,
	._task_delay = task_delay,
	._task_ms_to_tick = task_ms_to_tick_wrapper,
	._task_get_current_task = (void *(*)(void))k_current_get,
	._task_get_max_priority = task_get_max_priority_wrapper,
	._malloc = wifi_malloc,
	._free = esp_wifi_free,
	._event_post = esp_event_post_wrapper,
	._get_free_heap_size = esp_get_free_heap_size,
	._rand = sys_rand32_get,
	._dport_access_stall_other_cpu_start_wrap = esp_empty_wrapper,
	._dport_access_stall_other_cpu_end_wrap = esp_empty_wrapper,
	._wifi_apb80m_request = wifi_apb80m_request_wrapper,
	._wifi_apb80m_release = wifi_apb80m_release_wrapper,
	._phy_disable = esp_phy_disable,
	._phy_enable = esp_phy_enable,
	._phy_update_country_info = esp_phy_update_country_info,
	._read_mac = esp_read_mac,
	._timer_arm = timer_arm_wrapper,
	._timer_disarm = timer_disarm_wrapper,
	._timer_done = timer_done_wrapper,
	._timer_setfn = timer_setfn_wrapper,
	._timer_arm_us = timer_arm_us_wrapper,
	._wifi_reset_mac = wifi_reset_mac_wrapper,
	._wifi_clock_enable = wifi_clock_enable_wrapper,
	._wifi_clock_disable = wifi_clock_disable_wrapper,
	._wifi_rtc_enable_iso = esp_empty_wrapper,
	._wifi_rtc_disable_iso = esp_empty_wrapper,
	._esp_timer_get_time = esp_timer_get_time,
	._nvs_set_i8 = nvs_set_i8,
	._nvs_get_i8 = nvs_get_i8,
	._nvs_set_u8 = nvs_set_u8,
	._nvs_get_u8 = nvs_get_u8,
	._nvs_set_u16 = nvs_set_u16,
	._nvs_get_u16 = nvs_get_u16,
	._nvs_open = nvs_open,
	._nvs_close = nvs_close,
	._nvs_commit = nvs_commit,
	._nvs_set_blob = nvs_set_blob,
	._nvs_get_blob = nvs_get_blob,
	._nvs_erase_key = nvs_erase_key,
	._get_random = os_get_random,
	._get_time = get_time_wrapper,
	._random = random,
	._slowclk_cal_get = esp_clk_slowclk_cal_get_wrapper,
	._log_write = esp_log_write_wrapper,
	._log_writev = esp_log_writev_wrapper,
	._log_timestamp = k_uptime_get_32,
	._malloc_internal =  malloc_internal_wrapper,
	._realloc_internal = realloc_internal_wrapper,
	._calloc_internal = calloc_internal_wrapper,
	._zalloc_internal = zalloc_internal_wrapper,
	._wifi_malloc = wifi_malloc,
	._wifi_realloc = wifi_realloc,
	._wifi_calloc = wifi_calloc,
	._wifi_zalloc = wifi_zalloc_wrapper,
	._wifi_create_queue = wifi_create_queue_wrapper,
	._wifi_delete_queue = wifi_delete_queue_wrapper,
	._coex_init = coex_init_wrapper,
	._coex_deinit = coex_deinit_wrapper,
	._coex_enable = coex_enable_wrapper,
	._coex_disable = coex_disable_wrapper,
	._coex_status_get = coex_status_get_wrapper,
	._coex_condition_set = coex_condition_set_wrapper,
	._coex_wifi_request = coex_wifi_request_wrapper,
	._coex_wifi_release = coex_wifi_release_wrapper,
	._coex_wifi_channel_set = coex_wifi_channel_set_wrapper,
	._coex_event_duration_get = coex_event_duration_get_wrapper,
	._coex_pti_get = coex_pti_get_wrapper,
	._coex_schm_status_bit_clear = coex_schm_status_bit_clear_wrapper,
	._coex_schm_status_bit_set = coex_schm_status_bit_set_wrapper,
	._coex_schm_interval_set = coex_schm_interval_set_wrapper,
	._coex_schm_interval_get = coex_schm_interval_get_wrapper,
	._coex_schm_curr_period_get = coex_schm_curr_period_get_wrapper,
	._coex_schm_curr_phase_get = coex_schm_curr_phase_get_wrapper,
	._coex_schm_curr_phase_idx_set = coex_schm_curr_phase_idx_set_wrapper,
	._coex_schm_curr_phase_idx_get = coex_schm_curr_phase_idx_get_wrapper,
	._magic = ESP_WIFI_OS_ADAPTER_MAGIC,
};

esp_err_t esp_wifi_deinit(void)
{
	esp_err_t err = ESP_OK;

	esp_supplicant_deinit();
	err = esp_wifi_deinit_internal();

	return err;
}

esp_err_t esp_wifi_init(const wifi_init_config_t *config)
{
	esp_wifi_power_domain_on();

#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	coex_init();
#endif

	esp_err_t result = esp_wifi_init_internal(config);
	if (result == ESP_OK) {
		result = esp_supplicant_init();
		if (result != ESP_OK) {
		LOG_ERR("Failed to init supplicant (0x%x)", result);
			esp_wifi_deinit();
		}
	}

	return result;
}

static void esp_wifi_free(void *mem)
{
	k_free(mem);
}
