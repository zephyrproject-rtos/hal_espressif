/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/random/random.h>

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "xtensa/core-macros.h"
#include "esp_types.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_attr.h"
#include "esp_phy_init.h"
#include "esp_bt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "esp_private/periph_ctrl.h"
#include "soc/rtc.h"
#include "soc/soc_memory_layout.h"
#include "soc/dport_reg.h"
#include "private/esp_coexist_internal.h"
#include "esp_heap_adapter.h"
#include "esp_system.h"
#include "esp_rom_sys.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(esp32_bt_adapter, CONFIG_LOG_DEFAULT_LEVEL);

#define BTDM_INIT_PERIOD                    (5000)    /* ms */

/* Bluetooth system and controller config */
#define BTDM_CFG_BT_DATA_RELEASE            (1 << 0)
#define BTDM_CFG_HCI_UART                   (1 << 1)
#define BTDM_CFG_CONTROLLER_RUN_APP_CPU     (1 << 2)
#define BTDM_CFG_SCAN_DUPLICATE_OPTIONS     (1 << 3)
#define BTDM_CFG_SEND_ADV_RESERVED_SIZE     (1 << 4)
#define BTDM_CFG_BLE_FULL_SCAN_SUPPORTED    (1 << 5)

/* Sleep mode */
#define BTDM_MODEM_SLEEP_MODE_NONE          (0)
#define BTDM_MODEM_SLEEP_MODE_ORIG          (1)
/* sleep mode for BLE controller, used only for internal test. */
#define BTDM_MODEM_SLEEP_MODE_EVED          (2)

/* Low Power Clock Selection */
#define BTDM_LPCLK_SEL_XTAL      (0)
#define BTDM_LPCLK_SEL_XTAL32K   (1)
#define BTDM_LPCLK_SEL_RTC_SLOW  (2)
#define BTDM_LPCLK_SEL_8M        (3)

/* Sleep and wakeup interval control */
/* threshold of interval in slots to allow to fall into modem sleep */
#define BTDM_MIN_SLEEP_DURATION          (12)
/* delay in slots of modem wake up procedure, including re-enable PHY/RF */
#define BTDM_MODEM_WAKE_UP_DELAY         (4)

#define OSI_FUNCS_TIME_BLOCKING  0xffffffff
#define OSI_VERSION              0x00010005
#define OSI_MAGIC_VALUE          0xFADEBEAD

/* VHCI function interface */
typedef struct vhci_host_callback {
	/* callback used to notify that the host can send packet to controller */
	void (*notify_host_send_available)(void);
	/* callback used to notify that the controller has a packet to send to the host */
	int (*notify_host_recv)(uint8_t *data, uint16_t len);
} vhci_host_callback_t;

/* Dram region */
typedef struct {
	esp_bt_mode_t mode;
	intptr_t start;
	intptr_t end;
} btdm_dram_available_region_t;

/* bt queue */
struct bt_queue_t {
	struct k_msgq queue;
	void *pool;
};

/* OSI function */
struct osi_funcs_t {
	uint32_t _version;
	void (*_set_isr)(int n, void *f, void *arg);
	void (*_ints_on)(unsigned int mask);
	void (*_interrupt_disable)(void);
	void (*_interrupt_restore)(void);
	void (*_task_yield)(void);
	void (*_task_yield_from_isr)(void);
	void *(*_semphr_create)(uint32_t max, uint32_t init);
	void (*_semphr_delete)(void *semphr);
	int32_t (*_semphr_take_from_isr)(void *semphr, void *hptw);
	int32_t (*_semphr_give_from_isr)(void *semphr, void *hptw);
	int32_t (*_semphr_take)(void *semphr, uint32_t block_time_ms);
	int32_t (*_semphr_give)(void *semphr);
	void *(*_mutex_create)(void);
	void (*_mutex_delete)(void *mutex);
	int32_t (*_mutex_lock)(void *mutex);
	int32_t (*_mutex_unlock)(void *mutex);
	void *(*_queue_create)(uint32_t queue_len, uint32_t item_size);
	void (*_queue_delete)(void *queue);
	int32_t (*_queue_send)(void *queue, void *item, uint32_t block_time_ms);
	int32_t (*_queue_send_from_isr)(void *queue, void *item, void *hptw);
	int32_t (*_queue_recv)(void *queue, void *item, uint32_t block_time_ms);
	int32_t (*_queue_recv_from_isr)(void *queue, void *item, void *hptw);
	int32_t (*_task_create)(void *task_func, const char *name, uint32_t stack_depth, void *param,
							uint32_t prio, void *task_handle, uint32_t core_id);
	void (*_task_delete)(void *task_handle);
	bool (*_is_in_isr)(void);
	int (*_cause_sw_intr_to_core)(int core_id, int intr_no);
	void *(*_malloc)(uint32_t size);
	void *(*_malloc_internal)(uint32_t size);
	void (*_free)(void *p);
	int32_t (*_read_efuse_mac)(uint8_t mac[6]);
	void (*_srand)(unsigned int seed);
	int (*_rand)(void);
	uint32_t (*_btdm_lpcycles_2_us)(uint32_t cycles);
	uint32_t (*_btdm_us_2_lpcycles)(uint32_t us);
	bool (*_btdm_sleep_check_duration)(uint32_t *slot_cnt);
	void (*_btdm_sleep_enter_phase1)(uint32_t lpcycles);    /* called when interrupt is disabled */
	void (*_btdm_sleep_enter_phase2)(void);
	void (*_btdm_sleep_exit_phase1)(void);                  /* called from ISR */
	void (*_btdm_sleep_exit_phase2)(void);                  /* called from ISR */
	void (*_btdm_sleep_exit_phase3)(void);                  /* called from task */
	bool (*_coex_bt_wakeup_request)(void);
	void (*_coex_bt_wakeup_request_end)(void);
	int (*_coex_bt_request)(uint32_t event, uint32_t latency, uint32_t duration);
	int (*_coex_bt_release)(uint32_t event);
	int (*_coex_register_bt_cb)(coex_func_cb_t cb);
	uint32_t (*_coex_bb_reset_lock)(void);
	void (*_coex_bb_reset_unlock)(uint32_t restore);
	int (* _coex_schm_register_btdm_callback)(void *callback);
	void (* _coex_schm_status_bit_clear)(uint32_t type, uint32_t status);
	void (* _coex_schm_status_bit_set)(uint32_t type, uint32_t status);
	uint32_t (* _coex_schm_interval_get)(void);
	uint8_t (* _coex_schm_curr_period_get)(void);
	void *(* _coex_schm_curr_phase_get)(void);
	int (* _coex_wifi_channel_get)(uint8_t *primary, uint8_t *secondary);
	int (* _coex_register_wifi_channel_change_callback)(void *cb);
	void (*_set_isr_l3)(int n, void *f, void *arg);
	void (*_interrupt_l3_disable)(void);
	void (*_interrupt_l3_restore)(void);
	void *(* _customer_queue_create)(uint32_t queue_len, uint32_t item_size);
	int (* _coex_version_get)(unsigned int *major, unsigned int *minor, unsigned int *patch);
	void (* _patch_apply)(void);
	uint32_t _magic;
};

/* OSI */
extern int btdm_osi_funcs_register(void *osi_funcs);
/* Initialise and De-initialise */
extern int btdm_controller_init(uint32_t config_mask, esp_bt_controller_config_t *config_opts);
extern void btdm_controller_deinit(void);
extern int btdm_controller_enable(esp_bt_mode_t mode);
extern void btdm_controller_disable(void);
extern uint8_t btdm_controller_get_mode(void);
extern const char *btdm_controller_get_compile_version(void);
extern void btdm_rf_bb_init_phase2(void); /* shall be called after PHY/RF is enabled */
extern int btdm_dispatch_work_to_controller(workitem_handler_t callback, void *arg, bool blocking);
/* Sleep */
extern void btdm_controller_enable_sleep(bool enable);
extern void btdm_controller_set_sleep_mode(uint8_t mode);
extern uint8_t btdm_controller_get_sleep_mode(void);
extern bool btdm_power_state_active(void);
extern void btdm_wakeup_request(void);
extern void btdm_in_wakeup_requesting_set(bool in_wakeup_requesting);
/* Low Power Clock */
extern bool btdm_lpclk_select_src(uint32_t sel);
extern bool btdm_lpclk_set_div(uint32_t div);
/* VHCI */
extern bool API_vhci_host_check_send_available(void);
extern void API_vhci_host_send_packet(uint8_t *data, uint16_t len);
extern int API_vhci_host_register_callback(const vhci_host_callback_t *callback);
/* TX power */
extern int ble_txpwr_set(int power_type, int power_level);
extern int ble_txpwr_get(int power_type);
extern int bredr_txpwr_set(int min_power_level, int max_power_level);
extern int bredr_txpwr_get(int *min_power_level, int *max_power_level);
extern void bredr_sco_datapath_set(uint8_t data_path);
extern void btdm_controller_scan_duplicate_list_clear(void);
/* Coexistence */
extern int coex_bt_request(uint32_t event, uint32_t latency, uint32_t duration);
extern int coex_bt_release(uint32_t event);
extern int coex_register_bt_cb(coex_func_cb_t cb);
extern uint32_t coex_bb_reset_lock(void);
extern void coex_bb_reset_unlock(uint32_t restore);
extern int coex_schm_register_btdm_callback(void *callback);
extern void coex_schm_status_bit_clear(uint32_t type, uint32_t status);
extern void coex_schm_status_bit_set(uint32_t type, uint32_t status);
extern uint32_t coex_schm_interval_get(void);
extern uint8_t coex_schm_curr_period_get(void);
extern void * coex_schm_curr_phase_get(void);
extern int coex_wifi_channel_get(uint8_t *primary, uint8_t *secondary);
extern void coex_ble_adv_priority_high_set(bool high);
/* Shutdown */
extern void esp_bt_controller_shutdown(void);
extern void sdk_config_set_bt_pll_track_enable(bool enable);
extern void sdk_config_set_uart_flow_ctrl_enable(bool enable);

extern char _data_start_btdm[];
extern char _data_end_btdm[];
extern uint32_t _data_start_btdm_rom;
extern uint32_t _data_end_btdm_rom;

extern void config_bt_funcs_reset(void);
extern void config_ble_funcs_reset(void);
extern void config_btdm_funcs_reset(void);

extern void bt_stack_enableSecCtrlVsCmd(bool en);
extern void bt_stack_enableCoexVsCmd(bool en);
extern void scan_stack_enableAdvFlowCtrlVsCmd(bool en);
extern void adv_stack_enableClearLegacyAdvVsCmd(bool en);
extern void advFilter_stack_enableDupExcListVsCmd(bool en);

static void task_yield_from_isr(void);
static void *semphr_create_wrapper(uint32_t max, uint32_t init);
static void semphr_delete_wrapper(void *semphr);
static int32_t semphr_take_from_isr_wrapper(void *semphr, void *hptw);
static int32_t semphr_give_from_isr_wrapper(void *semphr, void *hptw);
static int32_t semphr_take_wrapper(void *semphr, uint32_t block_time_ms);
static int32_t semphr_give_wrapper(void *semphr);
static void *mutex_create_wrapper(void);
static void mutex_delete_wrapper(void *mutex);
static int32_t mutex_lock_wrapper(void *mutex);
static int32_t mutex_unlock_wrapper(void *mutex);
static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size);
static void queue_delete_wrapper(void *queue);
static int32_t queue_send_wrapper(void *queue, void *item, uint32_t block_time_ms);
static int32_t queue_send_from_isr_wrapper(void *queue, void *item, void *hptw);
static int32_t queue_recv_wrapper(void *queue, void *item, uint32_t block_time_ms);
static int32_t queue_recv_from_isr_wrapper(void *queue, void *item, void *hptw);
static int32_t task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle, uint32_t core_id);
static void task_delete_wrapper(void *task_handle);
static int cause_sw_intr_to_core_wrapper(int core_id, int intr_no);
static void *malloc_internal_wrapper(size_t size);
static int32_t read_mac_wrapper(uint8_t mac[6]);
static void srand_wrapper(unsigned int seed);
static int rand_wrapper(void);
static uint32_t btdm_lpcycles_2_us(uint32_t cycles);
static uint32_t btdm_us_2_lpcycles(uint32_t us);
static bool btdm_sleep_check_duration(uint32_t *slot_cnt);
static void btdm_sleep_enter_phase1_wrapper(uint32_t lpcycles);
static void btdm_sleep_enter_phase2_wrapper(void);
static void btdm_sleep_exit_phase3_wrapper(void);
static bool coex_bt_wakeup_request(void);
static void coex_bt_wakeup_request_end(void);
static int coex_bt_request_wrapper(uint32_t event, uint32_t latency, uint32_t duration);
static int coex_bt_release_wrapper(uint32_t event);
static int coex_register_bt_cb_wrapper(coex_func_cb_t cb);
static uint32_t coex_bb_reset_lock_wrapper(void);
static void coex_bb_reset_unlock_wrapper(uint32_t restore);
static int coex_schm_register_btdm_callback_wrapper(void *callback);
static void coex_schm_status_bit_clear_wrapper(uint32_t type, uint32_t status);
static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status);
static uint32_t coex_schm_interval_get_wrapper(void);
static uint8_t coex_schm_curr_period_get_wrapper(void);
static void * coex_schm_curr_phase_get_wrapper(void);
static int coex_wifi_channel_get_wrapper(uint8_t *primary, uint8_t *secondary);
static int coex_register_wifi_channel_change_callback_wrapper(void *cb);
static int coex_version_get_wrapper(unsigned int *major, unsigned int *minor, unsigned int *patch);
static void set_isr_wrapper(int32_t n, void *f, void *arg);
static void intr_on(unsigned int mask);
static void interrupt_l3_disable(void);
static void interrupt_l3_restore(void);
static void bt_controller_deinit_internal(void);
static void patch_apply(void);
static void esp_bt_free(void *mem);

/* Local variable definition
 ***************************************************************************
 */
/* OSI funcs */
static const struct osi_funcs_t osi_funcs_ro = {
	._version = OSI_VERSION,
	._set_isr = set_isr_wrapper,
	._ints_on = intr_on,
	._interrupt_disable = interrupt_l3_disable,
	._interrupt_restore = interrupt_l3_restore,
	._task_yield = task_yield_from_isr,
	._task_yield_from_isr = task_yield_from_isr,
	._semphr_create = semphr_create_wrapper,
	._semphr_delete = semphr_delete_wrapper,
	._semphr_take_from_isr = semphr_take_from_isr_wrapper,
	._semphr_give_from_isr = semphr_give_from_isr_wrapper,
	._semphr_take = semphr_take_wrapper,
	._semphr_give = semphr_give_wrapper,
	._mutex_create = mutex_create_wrapper,
	._mutex_delete = mutex_delete_wrapper,
	._mutex_lock = mutex_lock_wrapper,
	._mutex_unlock = mutex_unlock_wrapper,
	._queue_create = queue_create_wrapper,
	._queue_delete = queue_delete_wrapper,
	._queue_send = queue_send_wrapper,
	._queue_send_from_isr = queue_send_from_isr_wrapper,
	._queue_recv = queue_recv_wrapper,
	._queue_recv_from_isr = queue_recv_from_isr_wrapper,
	._task_create = task_create_wrapper,
	._task_delete = task_delete_wrapper,
	._is_in_isr = k_is_in_isr,
	._cause_sw_intr_to_core = cause_sw_intr_to_core_wrapper,
	._malloc = malloc_internal_wrapper,
	._malloc_internal = malloc_internal_wrapper,
	._free = esp_bt_free,
	._read_efuse_mac = read_mac_wrapper,
	._srand = srand_wrapper,
	._rand = rand_wrapper,
	._btdm_lpcycles_2_us = btdm_lpcycles_2_us,
	._btdm_us_2_lpcycles = btdm_us_2_lpcycles,
	._btdm_sleep_check_duration = btdm_sleep_check_duration,
	._btdm_sleep_enter_phase1 = btdm_sleep_enter_phase1_wrapper,
	._btdm_sleep_enter_phase2 = btdm_sleep_enter_phase2_wrapper,
	._btdm_sleep_exit_phase1 = NULL,
	._btdm_sleep_exit_phase2 = NULL,
	._btdm_sleep_exit_phase3 = btdm_sleep_exit_phase3_wrapper,
	._coex_bt_wakeup_request = coex_bt_wakeup_request,
	._coex_bt_wakeup_request_end = coex_bt_wakeup_request_end,
	._coex_bt_request = coex_bt_request_wrapper,
	._coex_bt_release = coex_bt_release_wrapper,
	._coex_register_bt_cb = coex_register_bt_cb_wrapper,
	._coex_bb_reset_lock = coex_bb_reset_lock_wrapper,
	._coex_bb_reset_unlock = coex_bb_reset_unlock_wrapper,
	._coex_schm_register_btdm_callback = coex_schm_register_btdm_callback_wrapper,
	._coex_schm_status_bit_clear = coex_schm_status_bit_clear_wrapper,
	._coex_schm_status_bit_set = coex_schm_status_bit_set_wrapper,
	._coex_schm_interval_get = coex_schm_interval_get_wrapper,
	._coex_schm_curr_period_get = coex_schm_curr_period_get_wrapper,
	._coex_schm_curr_phase_get = coex_schm_curr_phase_get_wrapper,
	._coex_wifi_channel_get = coex_wifi_channel_get_wrapper,
	._coex_register_wifi_channel_change_callback = coex_register_wifi_channel_change_callback_wrapper,
	._set_isr_l3 = set_isr_wrapper,
	._interrupt_l3_disable = interrupt_l3_disable,
	._interrupt_l3_restore = interrupt_l3_restore,
	._customer_queue_create = NULL,
	._coex_version_get = coex_version_get_wrapper,
	._patch_apply = patch_apply,
	._magic = OSI_MAGIC_VALUE,
};

/* the mode column will be modified by release function to indicate the available region */
static btdm_dram_available_region_t btdm_dram_available_region[] = {
	/* following is .data */
	{ ESP_BT_MODE_BTDM,          SOC_MEM_BT_DATA_START,      SOC_MEM_BT_DATA_END          },
	/* following is memory which HW will use */
	{ ESP_BT_MODE_BTDM,          SOC_MEM_BT_EM_BTDM0_START,  SOC_MEM_BT_EM_BTDM0_END      },
	{ ESP_BT_MODE_BLE,           SOC_MEM_BT_EM_BLE_START,    SOC_MEM_BT_EM_BLE_END        },
	{ ESP_BT_MODE_BTDM,          SOC_MEM_BT_EM_BTDM1_START,  SOC_MEM_BT_EM_BTDM1_END      },
	{ ESP_BT_MODE_CLASSIC_BT,    SOC_MEM_BT_EM_BREDR_START,  SOC_MEM_BT_EM_BREDR_REAL_END },
	/* following is .bss */
	{ ESP_BT_MODE_BTDM,          SOC_MEM_BT_BSS_START,       SOC_MEM_BT_BSS_END           },
	{ ESP_BT_MODE_BTDM,          SOC_MEM_BT_MISC_START,      SOC_MEM_BT_MISC_END          },
};

static DRAM_ATTR struct osi_funcs_t *osi_funcs_p;

/* Static variable declare */
/* timestamp when PHY/RF was switched on */
static DRAM_ATTR int64_t s_time_phy_rf_just_enabled = 0;
static DRAM_ATTR esp_bt_controller_status_t btdm_controller_status = ESP_BT_CONTROLLER_STATUS_IDLE;

static unsigned int global_int_lock;
static unsigned int global_nested_counter = 0;

/* BT library uses a single task */
K_THREAD_STACK_DEFINE(bt_stack, CONFIG_ESP32_BT_CONTROLLER_STACK_SIZE);
static struct k_thread bt_task_handle;

/* measured average low power clock period in micro seconds */
static DRAM_ATTR uint32_t btdm_lpcycle_us = 0;
/* number of fractional bit for btdm_lpcycle_us */
static DRAM_ATTR uint8_t btdm_lpcycle_us_frac = 0;

#if CONFIG_ESP32_BT_CTLR_MODEM_SLEEP_MODE_ORIG
// used low power clock
#if CONFIG_ESP32_BT_CTLR_LPCLK_SEL_EXT_32K_XTAL
static DRAM_ATTR uint8_t btdm_lpclk_sel = ESP_BT_SLEEP_CLOCK_EXT_32K_XTAL;
#else
static DRAM_ATTR uint8_t btdm_lpclk_sel = ESP_BT_SLEEP_CLOCK_MAIN_XTAL;
#endif /* CONFIG_ESP32_BT_CTLR_LPCLK_SEL_EXT_32K_XTAL */
#endif /* #ifdef CONFIG_ESP32_BT_CTLR_MODEM_SLEEP_MODE_ORIG */

static DRAM_ATTR struct k_sem *s_wakeup_req_sem = NULL;

static inline void esp_bt_power_domain_on(void)
{
	/* Bluetooth module power up */
	esp_wifi_bt_power_domain_on();
}

static inline void esp_bt_power_domain_off(void)
{
	/* Bluetooth module power down */
	esp_wifi_bt_power_domain_off();
}

static inline void btdm_check_and_init_bb(void)
{
	/* init BT-BB if PHY/RF has been switched off since last BT-BB init */
	int64_t latest_ts = esp_phy_rf_get_on_ts();

	if (latest_ts != s_time_phy_rf_just_enabled ||
	    s_time_phy_rf_just_enabled == 0) {
		btdm_rf_bb_init_phase2();
		s_time_phy_rf_just_enabled = latest_ts;
	}
}

static void IRAM_ATTR interrupt_l3_disable(void)
{
	if (global_nested_counter == 0) {
		global_int_lock = irq_lock();
	}

	if (global_nested_counter < 0xFFFFFFFF) {
		global_nested_counter++;
	}
}

static void IRAM_ATTR interrupt_l3_restore(void)
{
	if (global_nested_counter > 0) {
		global_nested_counter--;
	}

	if (global_nested_counter == 0) {
		irq_unlock(global_int_lock);
	}
}

static void set_isr_wrapper(int32_t n, void *f, void *arg)
{
	irq_disable(n);
	irq_connect_dynamic(n, Xthal_intlevel[n], f, arg, 0);
}

static void intr_on(unsigned int mask)
{
	unsigned int pos = 0, i = 1;

	while (!(i & mask)) {
		i = i << 1;
		++pos;
	}

	irq_enable(pos);
}

static void IRAM_ATTR task_yield_from_isr(void)
{
	k_yield();
}

static void *semphr_create_wrapper(uint32_t max, uint32_t init)
{
	struct k_sem *sem = (struct k_sem *) esp_bt_malloc_func(sizeof(struct k_sem));

	if (sem == NULL) {
		LOG_ERR("semaphore malloc failed");
		return NULL;
	}

	k_sem_init(sem, init, max);
	return sem;
}

static void semphr_delete_wrapper(void *semphr)
{
	esp_bt_free(semphr);
}

static int32_t IRAM_ATTR semphr_take_from_isr_wrapper(void *semphr, void *hptw)
{
	int *hpt = (int *) hptw;
	int ret = k_sem_take((struct k_sem *)semphr, K_NO_WAIT);

	*hpt = 0;

	if (ret == 0) {
		return 1;
	}

	return 0;
}

static int32_t IRAM_ATTR semphr_give_from_isr_wrapper(void *semphr, void *hptw)
{
	int *hpt = (int *) hptw;

	k_sem_give((struct k_sem *) semphr);

	*hpt = 0;
	return 1;
}

static int32_t semphr_take_wrapper(void *semphr, uint32_t block_time_ms)
{
	int ret = 0;

	if (block_time_ms == OSI_FUNCS_TIME_BLOCKING) {
		ret = k_sem_take((struct k_sem *)semphr, K_FOREVER);
	} else {
		ret = k_sem_take((struct k_sem *)semphr, K_MSEC(block_time_ms));
	}

	if (ret == 0) {
		return 1;
	}

	return 0;
}

static int32_t semphr_give_wrapper(void *semphr)
{
	k_sem_give((struct k_sem *) semphr);
	return 1;
}

static void *mutex_create_wrapper(void)
{
	struct k_mutex *my_mutex = (struct k_mutex *) esp_bt_malloc_func(sizeof(struct k_mutex));

	if (my_mutex == NULL) {
		LOG_ERR("mutex malloc failed");
		return NULL;
	}

	k_mutex_init(my_mutex);

	return my_mutex;
}

static void mutex_delete_wrapper(void *mutex)
{
	esp_bt_free(mutex);
}

static int32_t mutex_lock_wrapper(void *mutex)
{
	struct k_mutex *my_mutex = (struct k_mutex *) mutex;

	k_mutex_lock(my_mutex, K_FOREVER);
	return 0;
}

static int32_t mutex_unlock_wrapper(void *mutex)
{
	struct k_mutex *my_mutex = (struct k_mutex *) mutex;

	k_mutex_unlock(my_mutex);
	return 0;
}

static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size)
{
	struct bt_queue_t *queue = esp_bt_malloc_func(sizeof(struct bt_queue_t));

	if (queue == NULL) {
		LOG_ERR("queue malloc failed");
		return NULL;
	}

	queue->pool = (uint8_t *)esp_bt_malloc_func(queue_len * item_size * sizeof(uint8_t));

	if (queue->pool == NULL) {
		LOG_ERR("queue pool malloc failed");
		esp_bt_free(queue);
		return NULL;
	}

	k_msgq_init(&queue->queue, queue->pool, item_size, queue_len);
	return queue;
}

static void queue_delete_wrapper(void *queue)
{
	struct bt_queue_t *q = (struct bt_queue_t *) queue;

	if (q != NULL) {
		esp_bt_free(q->pool);
		esp_bt_free(q);
	}
}

static int32_t queue_send_wrapper(void *queue, void *item, uint32_t block_time_ms)
{
	int res = 0;

	if (block_time_ms == OSI_FUNCS_TIME_BLOCKING) {
		res = k_msgq_put(queue, item, K_FOREVER);
	} else {
		res = k_msgq_put(queue, item, K_MSEC(block_time_ms));
	}

	if (res == 0) {
		return 1;
	}

	return 0;
}

static int32_t IRAM_ATTR queue_send_from_isr_wrapper(void *queue, void *item, void *hptw)
{
	int *hpt = (int *) hptw;
	int ret = k_msgq_put(queue, item, K_NO_WAIT);

	*hpt = 0;

	if (ret == 0) {
		return 1;
	}

	return 0;
}

static int32_t queue_recv_wrapper(void *queue, void *item, uint32_t block_time_ms)
{
	int ret = 0;

	if (block_time_ms == OSI_FUNCS_TIME_BLOCKING) {
		ret = k_msgq_get(queue, item, K_FOREVER);
	} else {
		ret = k_msgq_get(queue, item, K_MSEC(block_time_ms));
	}

	if (ret == 0) {
		return 1;
	}

	return 0;
}

static int32_t IRAM_ATTR queue_recv_from_isr_wrapper(void *queue, void *item, void *hptw)
{
	int *hpt = (int *) hptw;
	int ret = k_msgq_get(queue, item, K_NO_WAIT);

	*hpt = 0;

	if (ret == 0) {
		return 1;
	}

	return 0;
}

static int32_t task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle, uint32_t core_id)
{
	k_tid_t tid = k_thread_create(&bt_task_handle, bt_stack, stack_depth,
				      (k_thread_entry_t)task_func, param, NULL, NULL,
				      K_PRIO_COOP(prio), K_INHERIT_PERMS, K_NO_WAIT);

	k_thread_name_set(tid, name);

	*(int32_t *)task_handle = (int32_t) tid;

	return 1;
}

static void task_delete_wrapper(void *task_handle)
{
	if (task_handle != NULL) {
		k_thread_abort((k_tid_t)task_handle);
	}

	k_object_release(&bt_task_handle);
}

static int IRAM_ATTR cause_sw_intr_to_core_wrapper(int core_id, int intr_no)
{
	esp_err_t err = ESP_OK;

#ifndef CONFIG_SMP
	XTHAL_SET_INTSET((1 << intr_no));
#else
	/* this should be handled once SMP support gets in place */
	err = ESP_ERR_NOT_SUPPORTED;
#endif

	return err;
}

static void *malloc_internal_wrapper(size_t size)
{
	return esp_bt_malloc_func(sizeof(uint8_t) * size);
}

static int32_t IRAM_ATTR read_mac_wrapper(uint8_t mac[6])
{
	return esp_read_mac(mac, ESP_MAC_BT);
}

static int IRAM_ATTR rand_wrapper(void)
{
	return (int)esp_random();
}

static void IRAM_ATTR srand_wrapper(unsigned int seed)
{
	/* empty function */
	ARG_UNUSED(seed);
}

static uint32_t IRAM_ATTR btdm_lpcycles_2_us(uint32_t cycles)
{
	/* The number of lp cycles should not lead to overflow. Thrs: 100s */
	/* clock measurement is conducted */
	uint64_t us = (uint64_t)btdm_lpcycle_us * cycles;

	us = (us + (1 << (btdm_lpcycle_us_frac - 1))) >> btdm_lpcycle_us_frac;
	return (uint32_t)us;
}

/*
 * @brief Converts a duration in slots into a number of low power clock cycles.
 */
static uint32_t IRAM_ATTR btdm_us_2_lpcycles(uint32_t us)
{
	/* The number of sleep duration(us) should not lead to overflow. Thrs: 100s */
	/* Compute the sleep duration in us to low power clock cycles, with calibration result applied */
	/* clock measurement is conducted */
	uint64_t cycles = ((uint64_t)(us) << btdm_lpcycle_us_frac) / btdm_lpcycle_us;

	return (uint32_t)cycles;
}

static bool IRAM_ATTR btdm_sleep_check_duration(uint32_t *slot_cnt)
{
	if (*slot_cnt < BTDM_MIN_SLEEP_DURATION) {
		return false;
	}
	/* wake up in advance considering the delay in enabling PHY/RF */
	*slot_cnt -= BTDM_MODEM_WAKE_UP_DELAY;
	return true;
}

static void btdm_sleep_enter_phase1_wrapper(uint32_t lpcycles)
{
	ARG_UNUSED(lpcycles);
}

static void btdm_sleep_enter_phase2_wrapper(void)
{
	if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG) {
		esp_phy_disable(PHY_MODEM_BT);
	} else if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_EVED) {
		esp_phy_disable(PHY_MODEM_BT);
		/* pause bluetooth baseband */
		periph_module_disable(PERIPH_BT_BASEBAND_MODULE);
	}
}

static void btdm_sleep_exit_phase3_wrapper(void)
{
	if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG) {
		esp_phy_enable(PHY_MODEM_BT);
		btdm_check_and_init_bb();
	} else if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_EVED) {
		/* resume bluetooth baseband */
		periph_module_enable(PERIPH_BT_BASEBAND_MODULE);
		esp_phy_enable(PHY_MODEM_BT);
	}
}

#define BTDM_ASYNC_WAKEUP_REQ_HCI       0
#define BTDM_ASYNC_WAKEUP_REQ_COEX      1
#define BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA 2
#define BTDM_ASYNC_WAKEUP_REQMAX        3

static void btdm_wakeup_request_callback(void *arg)
{
	(void)(arg);

	btdm_wakeup_request();

	semphr_give_wrapper(s_wakeup_req_sem);
}

static bool async_wakeup_request(int event)
{
	bool do_wakeup_request = false;

	switch (event) {
	case BTDM_ASYNC_WAKEUP_REQ_HCI:
	case BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA:
		btdm_in_wakeup_requesting_set(true);
		if (!btdm_power_state_active()) {
			do_wakeup_request = true;

			btdm_dispatch_work_to_controller(btdm_wakeup_request_callback, NULL, true);
			semphr_take_wrapper(s_wakeup_req_sem, OSI_FUNCS_TIME_BLOCKING);
		}
		break;
	case BTDM_ASYNC_WAKEUP_REQ_COEX:
		if (!btdm_power_state_active()) {
			do_wakeup_request = true;
			btdm_wakeup_request();
		}
		break;
	default:
		return false;
	}

	return do_wakeup_request;
}

static void async_wakeup_request_end(int event)
{
	bool request_lock = false;

	switch (event) {
	case BTDM_ASYNC_WAKEUP_REQ_HCI:
	case BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA:
		request_lock = true;
		break;
	case BTDM_ASYNC_WAKEUP_REQ_COEX:
		request_lock = false;
		break;
	default:
		return;
	}

	if (request_lock) {
		btdm_in_wakeup_requesting_set(false);
	}

	return;
}

static bool coex_bt_wakeup_request(void)
{
	return async_wakeup_request(BTDM_ASYNC_WAKEUP_REQ_COEX);
}

static void coex_bt_wakeup_request_end(void)
{
	async_wakeup_request_end(BTDM_ASYNC_WAKEUP_REQ_COEX);
	return;
}

static int IRAM_ATTR coex_bt_request_wrapper(uint32_t event, uint32_t latency, uint32_t duration)
{
#if CONFIG_SW_COEXIST_ENABLE
	return coex_bt_request(event, latency, duration);
#else
	return 0;
#endif
}

static int IRAM_ATTR coex_bt_release_wrapper(uint32_t event)
{
#if CONFIG_SW_COEXIST_ENABLE
	return coex_bt_release(event);
#else
	return 0;
#endif
}

static int coex_register_bt_cb_wrapper(coex_func_cb_t cb)
{
#if CONFIG_SW_COEXIST_ENABLE
	return coex_register_bt_cb(cb);
#else
	return 0;
#endif
}

static uint32_t IRAM_ATTR coex_bb_reset_lock_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE
	return coex_bb_reset_lock();
#else
	return 0;
#endif
}

static void IRAM_ATTR coex_bb_reset_unlock_wrapper(uint32_t restore)
{
#if CONFIG_SW_COEXIST_ENABLE
	coex_bb_reset_unlock(restore);
#endif
}

static int coex_schm_register_btdm_callback_wrapper(void *callback)
{
#if CONFIG_SW_COEXIST_ENABLE
	return coex_schm_register_callback(COEX_SCHM_CALLBACK_TYPE_BT, callback);
#else
	return 0;
#endif
}

static void coex_schm_status_bit_clear_wrapper(uint32_t type, uint32_t status)
{
#if CONFIG_SW_COEXIST_ENABLE
	coex_schm_status_bit_clear(type, status);
#endif
}

static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status)
{
#if CONFIG_SW_COEXIST_ENABLE
	coex_schm_status_bit_set(type, status);
#endif
}

static uint32_t coex_schm_interval_get_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE
	return coex_schm_interval_get();
#else
	return 0;
#endif
}

static uint8_t coex_schm_curr_period_get_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE
	return coex_schm_curr_period_get();
#else
	return 1;
#endif
}

static void * coex_schm_curr_phase_get_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE
	return coex_schm_curr_phase_get();
#else
	return NULL;
#endif
}

static int coex_wifi_channel_get_wrapper(uint8_t *primary, uint8_t *secondary)
{
#if CONFIG_SW_COEXIST_ENABLE
	return coex_wifi_channel_get(primary, secondary);
#else
	return -1;
#endif
}

static int coex_register_wifi_channel_change_callback_wrapper(void *cb)
{
#if CONFIG_SW_COEXIST_ENABLE
	return coex_register_wifi_channel_change_callback(cb);
#else
	return -1;
#endif
}

static int coex_version_get_wrapper(unsigned int *major, unsigned int *minor, unsigned int *patch)
{
#if CONFIG_SW_COEXIST_ENABLE
	coex_version_t version;
	coex_version_get_value(&version);
	*major = (unsigned int)version.major;
	*minor = (unsigned int)version.minor;
	*patch = (unsigned int)version.patch;
	return 0;
#endif
	return -1;
}

bool esp_vhci_host_check_send_available(void)
{
	return API_vhci_host_check_send_available();
}

void esp_vhci_host_send_packet(uint8_t *data, uint16_t len)
{
	async_wakeup_request(BTDM_ASYNC_WAKEUP_REQ_HCI);

	API_vhci_host_send_packet(data, len);

	async_wakeup_request_end(BTDM_ASYNC_WAKEUP_REQ_HCI);
}

esp_err_t esp_vhci_host_register_callback(const esp_vhci_host_callback_t *callback)
{
	return API_vhci_host_register_callback((const vhci_host_callback_t *)callback) == 0 ? ESP_OK : ESP_FAIL;
}

static uint32_t btdm_config_mask_load(void)
{
	uint32_t mask = 0x0;

#if CONFIG_ESP32_BT_CTLR_HCI_MODE_UART_H4
	mask |= BTDM_CFG_HCI_UART;
#endif
#if CONFIG_ESP32_BT_CTLR_PINNED_TO_CORE == 1
	mask |= BTDM_CFG_CONTROLLER_RUN_APP_CPU;
#endif
#if CONFIG_ESP32_BT_CTLR_FULL_SCAN_SUPPORTED
	mask |= BTDM_CFG_BLE_FULL_SCAN_SUPPORTED;
#endif /* CONFIG_ESP32_BT_CTLR_FULL_SCAN_SUPPORTED */
	mask |= BTDM_CFG_SCAN_DUPLICATE_OPTIONS;

	mask |= BTDM_CFG_SEND_ADV_RESERVED_SIZE;

	return mask;
}

static void btdm_controller_mem_init(void)
{
	/* initialise .data section */
	memcpy(_data_start_btdm, (void *)_data_start_btdm_rom, _data_end_btdm - _data_start_btdm);
	LOG_DBG(".data initialise [0x%08x] <== [0x%08x]", (uint32_t)&_data_start_btdm, (uint32_t)_data_start_btdm_rom);

	/* initial em, .bss section */
	for (int i = 1; i < sizeof(btdm_dram_available_region) / sizeof(btdm_dram_available_region_t); i++) {
		if (btdm_dram_available_region[i].mode != ESP_BT_MODE_IDLE) {
			memset((void *)btdm_dram_available_region[i].start, 0x0, btdm_dram_available_region[i].end - btdm_dram_available_region[i].start);
			LOG_DBG(".bss initialise [0x%08x] - [0x%08x]", (uint32_t)btdm_dram_available_region[i].start, (uint32_t)btdm_dram_available_region[i].end);
		}
	}
}

esp_err_t esp_bt_controller_mem_release(esp_bt_mode_t mode)
{
	/* not implemented */
	return ESP_OK;
}

esp_err_t esp_bt_mem_release(esp_bt_mode_t mode)
{
	/* not implemented */
	return ESP_OK;
}

// init low-power control resources
static esp_err_t btdm_low_power_mode_init(void)
{
	esp_err_t err = ESP_OK;

	// set default sleep clock cycle and its fractional bits
	btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
	btdm_lpcycle_us = 2 << (btdm_lpcycle_us_frac);

#if CONFIG_ESP32_BT_CTLR_MODEM_SLEEP_MODE_ORIG
	if (btdm_lpclk_sel == ESP_BT_SLEEP_CLOCK_EXT_32K_XTAL) {
		// check whether or not EXT_CRYS is working
		if (rtc_clk_slow_src_get() == SOC_RTC_SLOW_CLK_SRC_XTAL32K) {
			// do nothing
		} else {
			LOG_WRN("32.768kHz XTAL not detected, fall back to main "
					       "XTAL as Bluetooth sleep clock\n"
					       "light sleep mode will not be able to apply when "
					       "bluetooth is enabled");
			btdm_lpclk_sel = ESP_BT_SLEEP_CLOCK_MAIN_XTAL; // set default value
		}
	} else if (btdm_lpclk_sel != ESP_BT_SLEEP_CLOCK_MAIN_XTAL) {
		assert(0);
	}

	bool select_src_ret __attribute__((unused));
	bool set_div_ret __attribute__((unused));
	if (btdm_lpclk_sel == ESP_BT_SLEEP_CLOCK_MAIN_XTAL) {
		select_src_ret = btdm_lpclk_select_src(BTDM_LPCLK_SEL_XTAL);
		set_div_ret = btdm_lpclk_set_div(esp_clk_xtal_freq() * 2 / MHZ - 1);
		assert(select_src_ret && set_div_ret);
		btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
		btdm_lpcycle_us = 2 << (btdm_lpcycle_us_frac);
	} else { // btdm_lpclk_sel == BTDM_LPCLK_SEL_XTAL32K
		select_src_ret = btdm_lpclk_select_src(BTDM_LPCLK_SEL_XTAL32K);
		set_div_ret = btdm_lpclk_set_div(0);
		assert(select_src_ret && set_div_ret);
		btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
		btdm_lpcycle_us = (RTC_CLK_CAL_FRACT > 15) ? (1000000 << (RTC_CLK_CAL_FRACT - 15))
							   : (1000000 >> (15 - RTC_CLK_CAL_FRACT));
		assert(btdm_lpcycle_us != 0);
	}
	btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_ORIG);

#elif CONFIG_ESP32_BT_CTLR_MODEM_SLEEP_MODE_EVED
	btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_EVED);
#else
	btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_NONE);
#endif

	return err;
}

esp_bt_sleep_clock_t esp_bt_get_lpclk_src(void)
{
#if CONFIG_ESP32_BT_CTLR_MODEM_SLEEP_MODE_ORIG
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_INITED &&
	    btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return ESP_BT_SLEEP_CLOCK_NONE;
	}
	return btdm_lpclk_sel;
#else
	return ESP_BT_SLEEP_CLOCK_NONE;
#endif
}

esp_err_t esp_bt_set_lpclk_src(esp_bt_sleep_clock_t lpclk)
{
#if CONFIG_ESP32_BT_CTLR_MODEM_SLEEP_MODE_ORIG
	if (lpclk < ESP_BT_SLEEP_CLOCK_MAIN_XTAL || lpclk > ESP_BT_SLEEP_CLOCK_EXT_32K_XTAL) {
		return ESP_ERR_INVALID_ARG;
	}

	if (btdm_controller_status == ESP_BT_CONTROLLER_STATUS_INITED ||
	    btdm_controller_status == ESP_BT_CONTROLLER_STATUS_ENABLED) {
		ESP_LOGW(BTDM_LOG_TAG, "Please set the Bluetooth sleep clock source before "
				       "Bluetooth initialization");
		return ESP_ERR_INVALID_STATE;
	}

	btdm_lpclk_sel = lpclk;
	return ESP_OK;
#else
	return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t esp_bt_controller_init(esp_bt_controller_config_t *cfg)
{
	esp_err_t err;
	uint32_t btdm_cfg_mask = 0;

	osi_funcs_p = (struct osi_funcs_t *)malloc_internal_wrapper(sizeof(struct osi_funcs_t));
	if (osi_funcs_p == NULL) {
		return ESP_ERR_NO_MEM;
	}

	memcpy(osi_funcs_p, &osi_funcs_ro, sizeof(struct osi_funcs_t));
	if (btdm_osi_funcs_register(osi_funcs_p) != 0) {
		return ESP_ERR_INVALID_ARG;
	}

	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_IDLE) {
		return ESP_ERR_INVALID_STATE;
	}

	if (cfg == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	if (cfg->controller_task_prio != CONFIG_ESP32_BT_CONTROLLER_TASK_PRIO
	    || cfg->controller_task_stack_size < CONFIG_ESP32_BT_CONTROLLER_STACK_SIZE) {
		return ESP_ERR_INVALID_ARG;
	}

	/* overwrite some parameters */
	cfg->bt_max_sync_conn = BTDM_CTRL_BR_EDR_MAX_SYNC_CONN;
	cfg->magic = ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL;

	if (((cfg->mode & ESP_BT_MODE_BLE) && (cfg->ble_max_conn <= 0 || cfg->ble_max_conn > BTDM_CONTROLLER_BLE_MAX_CONN_LIMIT))
	    || ((cfg->mode & ESP_BT_MODE_CLASSIC_BT) && (cfg->bt_max_acl_conn <= 0 || cfg->bt_max_acl_conn > BTDM_CONTROLLER_BR_EDR_MAX_ACL_CONN_LIMIT))
	    || ((cfg->mode & ESP_BT_MODE_CLASSIC_BT) && (cfg->bt_max_sync_conn > BTDM_CONTROLLER_BR_EDR_MAX_SYNC_CONN_LIMIT))) {
		return ESP_ERR_INVALID_ARG;
	}

	LOG_INF("BT controller compile version [%s]", btdm_controller_get_compile_version());

	s_wakeup_req_sem = semphr_create_wrapper(1, 0);
	if (s_wakeup_req_sem == NULL) {
		err = ESP_ERR_NO_MEM;
		goto error;
	}

	esp_phy_modem_init();

	esp_bt_power_domain_on();

	btdm_controller_mem_init();

	periph_module_enable(PERIPH_BT_MODULE);
	periph_module_reset(PERIPH_BT_MODULE);

#if CONFIG_ESP32_BT_CTLR_HCI_UART_FLOW_CTRL_EN
	sdk_config_set_uart_flow_ctrl_enable(true);
#else
	sdk_config_set_uart_flow_ctrl_enable(false);
#endif

	if (btdm_low_power_mode_init() != 0) {
		err = ESP_ERR_NO_MEM;
		goto error;
	}

#if CONFIG_SW_COEXIST_ENABLE
	coex_init();
#endif

	btdm_cfg_mask = btdm_config_mask_load();

	err = btdm_controller_init(btdm_cfg_mask, cfg);

	if (err != 0) {
		LOG_ERR("%s %d", __func__, err);
		err = ESP_ERR_NO_MEM;
		goto error;
	}

	bt_stack_enableSecCtrlVsCmd(true);
	bt_stack_enableCoexVsCmd(true);
	scan_stack_enableAdvFlowCtrlVsCmd(true);
	adv_stack_enableClearLegacyAdvVsCmd(true);
	advFilter_stack_enableDupExcListVsCmd(true);

	btdm_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

	return ESP_OK;

error:

	bt_controller_deinit_internal();

	return err;
}

esp_err_t esp_bt_controller_deinit(void)
{
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_INITED) {
		return ESP_ERR_INVALID_STATE;
	}

	btdm_controller_deinit();

	bt_controller_deinit_internal();

	bt_stack_enableSecCtrlVsCmd(false);
	bt_stack_enableCoexVsCmd(false);
	scan_stack_enableAdvFlowCtrlVsCmd(false);
	adv_stack_enableClearLegacyAdvVsCmd(false);
	advFilter_stack_enableDupExcListVsCmd(false);

	return ESP_OK;
}

// deinit low power control resources
static void btdm_low_power_mode_deinit(void)
{
	btdm_lpcycle_us = 0;
	btdm_controller_set_sleep_mode(BTDM_MODEM_SLEEP_MODE_NONE);
}

static void bt_controller_deinit_internal(void)
{
	periph_module_disable(PERIPH_BT_MODULE);

	btdm_low_power_mode_deinit();

	if (s_wakeup_req_sem) {
		semphr_delete_wrapper(s_wakeup_req_sem);
		s_wakeup_req_sem = NULL;
	}

	if (osi_funcs_p) {
		esp_bt_free(osi_funcs_p);
		osi_funcs_p = NULL;
	}

	btdm_controller_status = ESP_BT_CONTROLLER_STATUS_IDLE;

	esp_bt_power_domain_off();

	esp_phy_modem_deinit();
}

static void bt_shutdown(void)
{
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return;
	}

	esp_bt_controller_shutdown();
	esp_phy_disable(PHY_MODEM_BT);
	return;
}

static void patch_apply(void)
{
	config_btdm_funcs_reset();

#ifdef CONFIG_BT_CLASSIC
	config_bt_funcs_reset();
#endif

	/* Zephyr: BLE mode enabled by default */
	config_ble_funcs_reset();
}

esp_err_t esp_bt_controller_enable(esp_bt_mode_t mode)
{
	int ret;

	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_INITED) {
		return ESP_ERR_INVALID_STATE;
	}

	/* As the history reason, mode should be equal to the mode which set in esp_bt_controller_init() */
	if (mode != btdm_controller_get_mode()) {
		return ESP_ERR_INVALID_ARG;
	}

	esp_phy_enable(PHY_MODEM_BT);

#if CONFIG_SW_COEXIST_ENABLE
	coex_enable();
#endif

	if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG) {
		btdm_controller_enable_sleep(true);
	}

	sdk_config_set_bt_pll_track_enable(false);

	/* inititalize bluetooth baseband */
	btdm_check_and_init_bb();

	ret = btdm_controller_enable(mode);
	if (ret != 0) {
#if CONFIG_SW_COEXIST_ENABLE
		coex_disable();
#endif
		esp_phy_disable(PHY_MODEM_BT);
		return ESP_ERR_INVALID_STATE;
	}

	btdm_controller_status = ESP_BT_CONTROLLER_STATUS_ENABLED;
	ret = esp_register_shutdown_handler(bt_shutdown);
	if (ret != ESP_OK) {
		LOG_WRN("Register shutdown handler failed, ret = 0x%x", ret);
	}

	/* set default TX power level */
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP32_RADIO_TXP_DEFAULT);

	return ESP_OK;
}

esp_err_t esp_bt_controller_disable(void)
{
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return ESP_ERR_INVALID_STATE;
	}

	/* disable modem sleep and wake up from sleep mode */
	if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG) {
		btdm_controller_enable_sleep(false);
		async_wakeup_request(BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA);
		while (!btdm_power_state_active()) {
			esp_rom_delay_us(1000);
		}
		async_wakeup_request_end(BTDM_ASYNC_WAKEUP_REQ_CTRL_DISA);
	}

	btdm_controller_disable();

#if CONFIG_SW_COEXIST_ENABLE
	coex_disable();
#endif

	esp_phy_disable(PHY_MODEM_BT);
	btdm_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;
	esp_unregister_shutdown_handler(bt_shutdown);

	return ESP_OK;
}

esp_bt_controller_status_t esp_bt_controller_get_status(void)
{
	return btdm_controller_status;
}

/* extra functions */
esp_err_t esp_ble_tx_power_set(esp_ble_power_type_t power_type, esp_power_level_t power_level)
{
	if (ble_txpwr_set(power_type, power_level) != 0) {
		return ESP_ERR_INVALID_ARG;
	}

	return ESP_OK;
}

esp_power_level_t esp_ble_tx_power_get(esp_ble_power_type_t power_type)
{
	return (esp_power_level_t)ble_txpwr_get(power_type);
}

esp_err_t esp_bredr_tx_power_set(esp_power_level_t min_power_level, esp_power_level_t max_power_level)
{
	esp_err_t err;
	int ret;

	ret = bredr_txpwr_set(min_power_level, max_power_level);

	if (ret == 0) {
		err = ESP_OK;
	} else if (ret == -1) {
		err = ESP_ERR_INVALID_ARG;
	} else {
		err = ESP_ERR_INVALID_STATE;
	}

	return err;
}

esp_err_t esp_bredr_tx_power_get(esp_power_level_t *min_power_level, esp_power_level_t *max_power_level)
{
	if (bredr_txpwr_get((int *)min_power_level, (int *)max_power_level) != 0) {
		return ESP_ERR_INVALID_ARG;
	}

	return ESP_OK;
}

esp_err_t esp_bt_sleep_enable(void)
{
	esp_err_t status;

	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return ESP_ERR_INVALID_STATE;
	}
	if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG ||
		btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_EVED) {
		btdm_controller_enable_sleep(true);
		status = ESP_OK;
	} else {
		status = ESP_ERR_NOT_SUPPORTED;
	}

	return status;
}

esp_err_t esp_bt_sleep_disable(void)
{
	esp_err_t status;

	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return ESP_ERR_INVALID_STATE;
	}
	if (btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_ORIG ||
		btdm_controller_get_sleep_mode() == BTDM_MODEM_SLEEP_MODE_EVED) {
		btdm_controller_enable_sleep(false);
		status = ESP_OK;
	} else {
		status = ESP_ERR_NOT_SUPPORTED;
	}

	return status;
}

esp_err_t esp_bredr_sco_datapath_set(esp_sco_data_path_t data_path)
{
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return ESP_ERR_INVALID_STATE;
	}
	bredr_sco_datapath_set(data_path);
	return ESP_OK;
}

esp_err_t esp_ble_scan_duplicate_list_flush(void)
{
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return ESP_ERR_INVALID_STATE;
	}
	btdm_controller_scan_duplicate_list_clear();
	return ESP_OK;
}

esp_err_t esp_ble_scan_dupilcate_list_flush(void)
{
	return esp_ble_scan_duplicate_list_flush();
}

static void esp_bt_free(void *mem)
{
	esp_bt_free_func(mem);
}
