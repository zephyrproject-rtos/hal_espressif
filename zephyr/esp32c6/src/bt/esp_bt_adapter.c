/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_attr.h"
#include "esp_phy_init.h"
#include "esp_bt.h"
#include "esp_err.h"
#include "esp_cpu.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/esp_clk.h"
#include "soc/soc_caps.h"
#include "soc/rtc.h"
#include "soc/soc_memory_layout.h"
#include "private/esp_coexist_internal.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_rom_sys.h"
#include "esp_private/phy.h"
#include "esp_heap_runtime.h"
#include "hal/efuse_hal.h"
#include "soc/rtc.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/random/random.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32c3.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(esp32_bt_adapter, CONFIG_LOG_DEFAULT_LEVEL);

#define OSI_COEX_VERSION			  0x00010006
#define OSI_COEX_MAGIC_VALUE		  0xFADEBEAD

#define EXT_FUNC_VERSION			 0x20240422
#define EXT_FUNC_MAGIC_VALUE		 0xA5A5A5A5

#define BT_ASSERT_PRINT			  ets_printf

/* Select heap to be used for WiFi adapter */
#if defined(CONFIG_ESP_BT_HEAP_RUNTIME)

#define esp_bt_malloc_func(_size) esp_heap_runtime_malloc(_size)
#define esp_bt_free_func(_mem) esp_heap_runtime_free(_mem)

#else

#define esp_bt_malloc_func(_size) k_malloc(_size)
#define esp_bt_free_func(_mem) k_free(_mem)

#endif /* CONFIG_ESP_BLUETOOTH_HEAP_RUNTIME */

struct osi_coex_funcs_t {
	uint32_t _magic;
	uint32_t _version;
	void (* _coex_wifi_sleep_set)(bool sleep);
	int (* _coex_core_ble_conn_dyn_prio_get)(bool *low, bool *high);
	void (* _coex_schm_status_bit_set)(uint32_t type, uint32_t status);
	void (* _coex_schm_status_bit_clear)(uint32_t type, uint32_t status);
};

struct ext_funcs_t {
	uint32_t ext_version;
	int (*_esp_intr_alloc)(int source, int flags, intr_handler_t handler, void *arg, void **ret_handle);
	int (*_esp_intr_free)(void **ret_handle);
	void *(* _malloc)(size_t size);
	void (*_free)(void *p);
	int (* _task_create)(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle, uint32_t core_id);
	void (* _task_delete)(void *task_handle);
	void (*_osi_assert)(const uint32_t ln, const char *fn, uint32_t param1, uint32_t param2);
	uint32_t (* _os_random)(void);
	int (* _ecc_gen_key_pair)(uint8_t *public, uint8_t *priv);
	int (* _ecc_gen_dh_key)(const uint8_t *remote_pub_key_x, const uint8_t *remote_pub_key_y, const uint8_t *local_priv_key, uint8_t *dhkey);
	void (* _esp_reset_rpa_moudle)(void);
	uint32_t magic;
};

/* External functions or variables
 ************************************************************************
 */
extern int ble_osi_coex_funcs_register(struct osi_coex_funcs_t *coex_funcs);
extern int r_ble_controller_init(esp_bt_controller_config_t *cfg);
#if CONFIG_BT_LE_CONTROLLER_LOG_ENABLED
extern int r_ble_log_init_async(interface_func_t bt_controller_log_interface, bool task_create, uint8_t buffers, uint32_t *bufs_size);
extern int r_ble_log_deinit_async(void);
extern void r_ble_log_async_select_dump_buffers(uint8_t buffers);
extern void r_ble_log_async_output_dump_all(bool output);
extern void esp_panic_handler_reconfigure_wdts(uint32_t timeout_ms);
#endif // CONFIG_BT_LE_CONTROLLER_LOG_ENABLED
extern int r_ble_controller_deinit(void);
extern int r_ble_controller_enable(uint8_t mode);
extern int r_ble_controller_disable(void);
extern int esp_register_ext_funcs (struct ext_funcs_t *);
extern void esp_unregister_ext_funcs (void);
extern int r_esp_ble_ll_set_public_addr(const uint8_t *addr);
extern int esp_register_npl_funcs (struct npl_funcs_t *p_npl_func);
extern void esp_unregister_npl_funcs (void);
extern void npl_freertos_mempool_deinit(void);
extern uint32_t r_os_cputime_get32(void);
extern uint32_t r_os_cputime_ticks_to_usecs(uint32_t ticks);
extern void r_ble_lll_rfmgmt_set_sleep_cb(void *s_cb, void *w_cb, void *s_arg,
										  void *w_arg, uint32_t us_to_enabled);
extern void r_ble_rtc_wake_up_state_clr(void);
extern int os_msys_init(void);
extern void os_msys_deinit(void);
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
extern const sleep_retention_entries_config_t *esp_ble_mac_retention_link_get(uint8_t *size, uint8_t extra);
extern void r_esp_ble_set_wakeup_overhead(uint32_t overhead);
#endif /* CONFIG_FREERTOS_USE_TICKLESS_IDLE */
extern void r_esp_ble_change_rtc_freq(uint32_t freq);
extern int ble_sm_alg_gen_dhkey(const uint8_t *peer_pub_key_x,
								const uint8_t *peer_pub_key_y,
								const uint8_t *our_priv_key, uint8_t *out_dhkey);
extern int ble_sm_alg_gen_key_pair(uint8_t *pub, uint8_t *priv);
extern int r_ble_txpwr_set(esp_ble_enhanced_power_type_t power_type, uint16_t handle, int power_level);
extern int r_ble_txpwr_get(esp_ble_enhanced_power_type_t power_type, uint16_t handle);
extern int r_ble_get_npl_element_info(esp_bt_controller_config_t *cfg, ble_npl_count_info_t * npl_info);
extern char *ble_controller_get_compile_version(void);
extern int esp_ble_register_bb_funcs(void);
extern void esp_ble_unregister_bb_funcs(void);
extern uint32_t _bt_bss_start;
extern uint32_t _bt_bss_end;
extern uint32_t _bt_controller_bss_start;
extern uint32_t _bt_controller_bss_end;
extern uint32_t _bt_data_start;
extern uint32_t _bt_data_end;
extern uint32_t _bt_controller_data_start;
extern uint32_t _bt_controller_data_end;

/* Local Function Declaration
 *********************************************************************
 */
static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status);
static void coex_schm_status_bit_clear_wrapper(uint32_t type, uint32_t status);
static int task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth,
							   void *param, uint32_t prio, void *task_handle, uint32_t core_id);
static void task_delete_wrapper(void *task_handle);
static int esp_intr_alloc_wrapper(int source, int flags, intr_handler_t handler,
								  void *arg, void **ret_handle_in);
static int esp_intr_free_wrapper(void **ret_handle);
static void osi_assert_wrapper(const uint32_t ln, const char *fn, uint32_t param1, uint32_t param2);
static uint32_t osi_random_wrapper(void);
static void esp_reset_rpa_moudle(void);
static int esp_ecc_gen_key_pair(uint8_t *pub, uint8_t *priv);
static int esp_ecc_gen_dh_key(const uint8_t *peer_pub_key_x, const uint8_t *peer_pub_key_y,
							  const uint8_t *our_priv_key, uint8_t *out_dhkey);

/* Static variable declare */
static DRAM_ATTR esp_bt_controller_status_t ble_controller_status = ESP_BT_CONTROLLER_STATUS_IDLE;

/* This variable tells if BLE is running */
static bool s_ble_active = false;
#ifdef CONFIG_PM_ENABLE
static DRAM_ATTR esp_pm_lock_handle_t s_pm_lock = NULL;
#define BTDM_MIN_TIMER_UNCERTAINTY_US	  (200)
#endif // CONFIG_PM_ENABLE

#define BLE_RTC_DELAY_US_LIGHT_SLEEP		(2500)
#define BLE_RTC_DELAY_US_MODEM_SLEEP		(500)

static const struct osi_coex_funcs_t s_osi_coex_funcs_ro = {
	._magic = OSI_COEX_MAGIC_VALUE,
	._version = OSI_COEX_VERSION,
	._coex_wifi_sleep_set = NULL,
	._coex_core_ble_conn_dyn_prio_get = NULL,
	._coex_schm_status_bit_set = coex_schm_status_bit_set_wrapper,
	._coex_schm_status_bit_clear = coex_schm_status_bit_clear_wrapper,
};

struct ext_funcs_t ext_funcs_ro = {
	.ext_version = EXT_FUNC_VERSION,
	._esp_intr_alloc = esp_intr_alloc_wrapper,
	._esp_intr_free = esp_intr_free_wrapper,
	._malloc = esp_bt_malloc_func,
	._free = esp_bt_free,
	._task_create = task_create_wrapper,
	._task_delete = task_delete_wrapper,
	._osi_assert = osi_assert_wrapper,
	._os_random = osi_random_wrapper,
	._ecc_gen_key_pair = esp_ecc_gen_key_pair,
	._ecc_gen_dh_key = esp_ecc_gen_dh_key,
	._esp_reset_rpa_moudle = esp_reset_rpa_moudle,
	.magic = EXT_FUNC_MAGIC_VALUE,
};

static void esp_bt_free(void *mem)
{
	esp_bt_free_func(mem);
}

static void esp_bt_malloc(void *mem)
{
	esp_bt_free_func(mem);
}

static void IRAM_ATTR esp_reset_rpa_moudle(void)
{

}

static void IRAM_ATTR osi_assert_wrapper(const uint32_t ln, const char *fn,
										 uint32_t param1, uint32_t param2)
{
	BT_ASSERT_PRINT("BLE assert: line %d in function %s, param: 0x%x, 0x%x", ln, fn, param1, param2);
	assert(0);
}

static uint32_t IRAM_ATTR osi_random_wrapper(void)
{
	return esp_random();
}

static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status)
{
#if CONFIG_SW_COEXIST_ENABLE
	coex_schm_status_bit_set(type, status);
#endif // CONFIG_SW_COEXIST_ENABLE
}

static void coex_schm_status_bit_clear_wrapper(uint32_t type, uint32_t status)
{
#if CONFIG_SW_COEXIST_ENABLE
	coex_schm_status_bit_clear(type, status);
#endif // CONFIG_SW_COEXIST_ENABLE
}

static int task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth,
								void *param, uint32_t prio, void *task_handle, uint32_t core_id)
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

static int esp_ecc_gen_key_pair(uint8_t *pub, uint8_t *priv)
{
	int rc = -1;
#if CONFIG_BT_LE_SM_LEGACY || CONFIG_BT_LE_SM_SC
	rc = ble_sm_alg_gen_key_pair(pub, priv);
#endif // CONFIG_BT_LE_SM_LEGACY || CONFIG_BT_LE_SM_SC
	return rc;
}

static int esp_ecc_gen_dh_key(const uint8_t *peer_pub_key_x, const uint8_t *peer_pub_key_y,
							  const uint8_t *our_priv_key, uint8_t *out_dhkey)
{
	int rc = -1;
#if CONFIG_BT_LE_SM_LEGACY || CONFIG_BT_LE_SM_SC
	rc = ble_sm_alg_gen_dhkey(peer_pub_key_x, peer_pub_key_y, our_priv_key, out_dhkey);
#endif // CONFIG_BT_LE_SM_LEGACY || CONFIG_BT_LE_SM_SC
	return rc;
}

static int esp_intr_alloc_wrapper(int source, int flags, intr_handler_t handler,
								  void *arg, void **ret_handle_in)
{
	int rc = esp_intr_alloc(source, flags | ESP_INTR_FLAG_IRAM, handler,
							arg, (intr_handle_t *)ret_handle_in);
	return rc;
}

static int esp_intr_free_wrapper(void **ret_handle)
{
	int rc = 0;
	// rc = esp_intr_free((intr_handle_t) * ret_handle);
	// *ret_handle = NULL;
	return rc;
}

void esp_bt_rtc_slow_clk_select(uint8_t slow_clk_src)
{
	/* Select slow clock source for BT momdule */
	switch (slow_clk_src) {
		case MODEM_CLOCK_LPCLK_SRC_MAIN_XTAL:
			LOG_INF("Using main XTAL as clock source");
			uint32_t chip_version = efuse_hal_chip_revision();
			if (chip_version == 0) {
				modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (400 - 1));
			} else{
				modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (5 - 1));
			}
			break;
		case MODEM_CLOCK_LPCLK_SRC_RC_SLOW:
			LOG_INF("Using 136 kHz RC as clock source, can only run legacy ADV or SCAN due to low clock accuracy!");
			modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (5 - 1));
			break;
		case MODEM_CLOCK_LPCLK_SRC_XTAL32K:
			LOG_INF("Using external 32.768 kHz XTAL as clock source");
			modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (1 - 1));
			break;
		case MODEM_CLOCK_LPCLK_SRC_RC32K:
			LOG_INF("Using 32 kHz RC as clock source, can only run legacy ADV or SCAN due to low clock accuracy!");
			modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (1 - 1));
			break;
		case MODEM_CLOCK_LPCLK_SRC_EXT32K:
			LOG_INF("Using 32 kHz oscillator as clock source, can only run legacy ADV or SCAN due to low clock accuracy!");
			modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (1 - 1));
			break;
		default:
	}
}

IRAM_ATTR void controller_sleep_cb(uint32_t enable_tick, void *arg)
{
	if (!s_ble_active) {
		return;
	}
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
	r_ble_rtc_wake_up_state_clr();
#endif /* CONFIG_FREERTOS_USE_TICKLESS_IDLE */
	esp_phy_disable(PHY_MODEM_BT);
#ifdef CONFIG_PM_ENABLE
	esp_pm_lock_release(s_pm_lock);
#endif // CONFIG_PM_ENABLE
	s_ble_active = false;
}

IRAM_ATTR void controller_wakeup_cb(void *arg)
{
	if (s_ble_active) {
		return;
	}
#ifdef CONFIG_PM_ENABLE
	esp_pm_lock_acquire(s_pm_lock);
	r_ble_rtc_wake_up_state_clr();
#endif //CONFIG_PM_ENABLE
	esp_phy_enable(PHY_MODEM_BT);
	s_ble_active = true;
}

#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
static esp_err_t sleep_modem_ble_mac_retention_init(void *arg)
{
	uint8_t size;
	int extra = *(int *)arg;
	const sleep_retention_entries_config_t *ble_mac_modem_config = esp_ble_mac_retention_link_get(&size, extra);
	esp_err_t err = sleep_retention_entries_create(ble_mac_modem_config, size, REGDMA_LINK_PRI_5, SLEEP_RETENTION_MODULE_BLE_MAC);
	if (err == ESP_OK) {
		LOG_INF("Modem BLE MAC retention initialization");
	}
	return err;
}

static esp_err_t sleep_modem_ble_mac_modem_state_init(uint8_t extra)
{
	int retention_args = extra;
	sleep_retention_module_init_param_t init_param = {
		.cbs	 = { .create = { .handle = sleep_modem_ble_mac_retention_init, .arg = &retention_args } },
		.depends = BIT(SLEEP_RETENTION_MODULE_BT_BB)
	};
	esp_err_t err = sleep_retention_module_init(SLEEP_RETENTION_MODULE_BLE_MAC, &init_param);
	if (err == ESP_OK) {
		err = sleep_retention_module_allocate(SLEEP_RETENTION_MODULE_BLE_MAC);
	}
	return err;
}

static void sleep_modem_ble_mac_modem_state_deinit(void)
{
	esp_err_t err = sleep_retention_module_free(SLEEP_RETENTION_MODULE_BLE_MAC);
	if (err == ESP_OK) {
		err = sleep_retention_module_deinit(SLEEP_RETENTION_MODULE_BLE_MAC);
		assert(err == ESP_OK);
	}
}

void sleep_modem_light_sleep_overhead_set(uint32_t overhead)
{
	r_esp_ble_set_wakeup_overhead(overhead);
}
#endif /* CONFIG_FREERTOS_USE_TICKLESS_IDLE */


esp_err_t controller_sleep_init(void)
{
	esp_err_t rc = 0;

#ifdef CONFIG_BT_LE_SLEEP_ENABLE
	LOG_WRN("BLE modem sleep is enabled");
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
	r_ble_lll_rfmgmt_set_sleep_cb(controller_sleep_cb, controller_wakeup_cb, 0, 0,
								BLE_RTC_DELAY_US_LIGHT_SLEEP);
#else
	r_ble_lll_rfmgmt_set_sleep_cb(controller_sleep_cb, controller_wakeup_cb, 0, 0,
								BLE_RTC_DELAY_US_MODEM_SLEEP);
#endif /* FREERTOS_USE_TICKLESS_IDLE */
#endif // CONFIG_BT_LE_SLEEP_ENABLE

#ifdef CONFIG_PM_ENABLE
	rc = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "bt", &s_pm_lock);
	if (rc != ESP_OK) {
		goto error;
	}
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
#if CONFIG_BT_LE_SLEEP_ENABLE && !CONFIG_MAC_BB_PD
#error "CONFIG_MAC_BB_PD required for BLE light sleep to run properly"
#endif // CONFIG_BT_LE_SLEEP_ENABLE && !CONFIG_MAC_BB_PD
	/* Create a new regdma link for BLE related register restoration */
	rc = sleep_modem_ble_mac_modem_state_init(1);
	assert(rc == 0);
	esp_sleep_enable_bt_wakeup();
	LOG_WRN("Enable light sleep, the wake up source is BLE timer");

	rc = esp_pm_register_inform_out_light_sleep_overhead_callback(sleep_modem_light_sleep_overhead_set);
	if (rc != ESP_OK) {
		goto error;
	}

#if SOC_PM_RETENTION_HAS_CLOCK_BUG && CONFIG_MAC_BB_PD
	sleep_modem_register_mac_bb_module_prepare_callback(sleep_modem_mac_bb_power_down_prepare,
												   sleep_modem_mac_bb_power_up_prepare);
#endif // SOC_PM_RETENTION_HAS_CLOCK_BUG && CONFIG_MAC_BB_PD
#endif /* CONFIG_FREERTOS_USE_TICKLESS_IDLE */
	return rc;

error:

#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
#if SOC_PM_RETENTION_HAS_CLOCK_BUG && CONFIG_MAC_BB_PD
	sleep_modem_unregister_mac_bb_module_prepare_callback(sleep_modem_mac_bb_power_down_prepare,
													 sleep_modem_mac_bb_power_up_prepare);
#endif // SOC_PM_RETENTION_HAS_CLOCK_BUG && CONFIG_MAC_BB_PD
	esp_sleep_disable_bt_wakeup();
	esp_pm_unregister_inform_out_light_sleep_overhead_callback(sleep_modem_light_sleep_overhead_set);
#endif /* CONFIG_FREERTOS_USE_TICKLESS_IDLE */
	/*lock should release first and then delete*/
	if (s_pm_lock != NULL) {
		esp_pm_lock_delete(s_pm_lock);
		s_pm_lock = NULL;
	}
#endif // CONFIG_PM_ENABLE

	return rc;
}

void controller_sleep_deinit(void)
{
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
#if SOC_PM_RETENTION_HAS_CLOCK_BUG && CONFIG_MAC_BB_PD
	sleep_modem_unregister_mac_bb_module_prepare_callback(sleep_modem_mac_bb_power_down_prepare,
													 sleep_modem_mac_bb_power_up_prepare);
#endif // SOC_PM_RETENTION_HAS_CLOCK_BUG && CONFIG_MAC_BB_PD
	r_ble_rtc_wake_up_state_clr();
	esp_sleep_disable_bt_wakeup();
	sleep_modem_ble_mac_modem_state_deinit();
	esp_pm_unregister_inform_out_light_sleep_overhead_callback(sleep_modem_light_sleep_overhead_set);
#endif /* CONFIG_FREERTOS_USE_TICKLESS_IDLE */
#ifdef CONFIG_PM_ENABLE
	/* lock should be released first */
	esp_pm_lock_delete(s_pm_lock);
	s_pm_lock = NULL;
#endif //CONFIG_PM_ENABLE
}

typedef enum {
	FILTER_DUPLICATE_PDUTYPE = BIT(0),
	FILTER_DUPLICATE_LENGTH  = BIT(1),
	FILTER_DUPLICATE_ADDRESS = BIT(2),
	FILTER_DUPLICATE_ADVDATA = BIT(3),
	FILTER_DUPLICATE_DEFAULT = FILTER_DUPLICATE_PDUTYPE | FILTER_DUPLICATE_ADDRESS,
	FILTER_DUPLICATE_PDU_ALL = 0xF,
	FILTER_DUPLICATE_EXCEPTION_FOR_MESH = BIT(4),
	FILTER_DUPLICATE_AD_TYPE = BIT(5),
}disc_duplicate_mode_t;


extern void r_filter_duplicate_mode_enable(disc_duplicate_mode_t mode);
extern void r_filter_duplicate_mode_disable(disc_duplicate_mode_t mode);
extern void r_filter_duplicate_set_ring_list_max_num(uint32_t max_num);
extern void r_scan_duplicate_cache_refresh_set_time(uint32_t period_time);

int
ble_vhci_disc_duplicate_mode_enable(int mode)
{
	// TODO: use vendor hci to update
	r_filter_duplicate_mode_enable(mode);
	return true;
}

int
ble_vhci_disc_duplicate_mode_disable(int mode)
{
	// TODO: use vendor hci to update
	r_filter_duplicate_mode_disable(mode);
	return true;
}

int ble_vhci_disc_duplicate_set_max_cache_size(int max_cache_size){
	// TODO: use vendor hci to update
	r_filter_duplicate_set_ring_list_max_num(max_cache_size);
	return true;
}

int ble_vhci_disc_duplicate_set_period_refresh_time(int refresh_period_time){
	// TODO: use vendor hci to update
	r_scan_duplicate_cache_refresh_set_time(refresh_period_time);
	return true;
}

/**
 * @brief Config scan duplicate option mode from menuconfig (Adapt to the old configuration method.)
 */
void ble_controller_scan_duplicate_config(void)
{
	uint32_t duplicate_mode = FILTER_DUPLICATE_DEFAULT;
	uint32_t cache_size = 100;
#if CONFIG_BT_LE_SCAN_DUPL == true
	cache_size = CONFIG_BT_LE_LL_DUP_SCAN_LIST_COUNT;
	if (CONFIG_BT_LE_SCAN_DUPL_TYPE == 0) {
		duplicate_mode = FILTER_DUPLICATE_ADDRESS | FILTER_DUPLICATE_PDUTYPE;
	} else if (CONFIG_BT_LE_SCAN_DUPL_TYPE == 1) {
		duplicate_mode = FILTER_DUPLICATE_ADVDATA;
	} else if (CONFIG_BT_LE_SCAN_DUPL_TYPE == 2) {
		duplicate_mode = FILTER_DUPLICATE_ADDRESS | FILTER_DUPLICATE_ADVDATA;
	}
	duplicate_mode |= FILTER_DUPLICATE_EXCEPTION_FOR_MESH;

	ble_vhci_disc_duplicate_set_period_refresh_time(CONFIG_BT_LE_SCAN_DUPL_CACHE_REFRESH_PERIOD);
#endif

	ble_vhci_disc_duplicate_mode_disable(0xFFFFFFFF);
	ble_vhci_disc_duplicate_mode_enable(duplicate_mode);
	ble_vhci_disc_duplicate_set_max_cache_size(cache_size);
}

esp_err_t esp_bt_controller_init(esp_bt_controller_config_t *cfg)
{
	uint8_t mac[6];
	esp_err_t ret = ESP_OK;
	ble_npl_count_info_t npl_info;
	uint32_t slow_clk_freq = 0;
	uint8_t hci_transport_mode;

	memset(&npl_info, 0, sizeof(ble_npl_count_info_t));
	if (ble_controller_status != ESP_BT_CONTROLLER_STATUS_IDLE) {
		LOG_WRN("invalid controller state");
		return ESP_ERR_INVALID_STATE;
	}

	if (!cfg) {
		LOG_WRN("cfg is NULL");
		return ESP_ERR_INVALID_ARG;
	}

	ret = esp_register_ext_funcs(&ext_funcs_ro);
	if (ret != ESP_OK) {
		LOG_WRN("register extend functions failed");
		return ret;
	}

	/* Initialize the function pointers for OS porting */
	npl_freertos_funcs_init();
	struct npl_funcs_t *p_npl_funcs = npl_freertos_funcs_get();
	if (!p_npl_funcs) {
		LOG_WRN("npl functions get failed");
		return ESP_ERR_INVALID_ARG;
	}

	ret = esp_register_npl_funcs(p_npl_funcs);
	if (ret != ESP_OK) {
		LOG_WRN("npl functions register failed");
		goto free_mem;
	}

	r_ble_get_npl_element_info(cfg, &npl_info);
	npl_freertos_set_controller_npl_info(&npl_info);
	if (npl_freertos_mempool_init() != 0) {
		LOG_WRN("npl mempool init failed");
		ret = ESP_ERR_INVALID_ARG;
		goto free_mem;
	}

	/* Enable BT-related clocks */
	modem_clock_module_enable(PERIPH_BT_MODULE);
	modem_clock_module_mac_reset(PERIPH_BT_MODULE);
	/* Select slow clock source for BT momdule */
#if CONFIG_BT_LE_LP_CLK_SRC_MAIN_XTAL
   esp_bt_rtc_slow_clk_select(MODEM_CLOCK_LPCLK_SRC_MAIN_XTAL);
   slow_clk_freq = 100000;
#else
#if CONFIG_RTC_CLK_SRC_INT_RC
	esp_bt_rtc_slow_clk_select(MODEM_CLOCK_LPCLK_SRC_RC_SLOW);
	slow_clk_freq = 30000;
#elif CONFIG_RTC_CLK_SRC_EXT_CRYS
	if (rtc_clk_slow_src_get() == SOC_RTC_SLOW_CLK_SRC_XTAL32K) {
		esp_bt_rtc_slow_clk_select(MODEM_CLOCK_LPCLK_SRC_XTAL32K);
		slow_clk_freq = 32768;
	} else {
		LOG_WRN("32.768kHz XTAL not detected, fall back to main XTAL as Bluetooth sleep clock");
		esp_bt_rtc_slow_clk_select(MODEM_CLOCK_LPCLK_SRC_MAIN_XTAL);
		slow_clk_freq = 100000;
	}
#elif CONFIG_RTC_CLK_SRC_INT_RC32K
	esp_bt_rtc_slow_clk_select(MODEM_CLOCK_LPCLK_SRC_RC32K);
	slow_clk_freq = 32000;
#elif CONFIG_RTC_CLK_SRC_EXT_OSC
	esp_bt_rtc_slow_clk_select(MODEM_CLOCK_LPCLK_SRC_EXT32K);
	slow_clk_freq = 32000;
#else
	LOG_ERR("Unsupported clock source");
	assert(0);
#endif
#endif /* CONFIG_BT_LE_LP_CLK_SRC_MAIN_XTAL */
	esp_phy_modem_init();

	if (ble_osi_coex_funcs_register((struct osi_coex_funcs_t *)&s_osi_coex_funcs_ro) != 0) {
		LOG_WRN("osi coex funcs reg failed");
		ret = ESP_ERR_INVALID_ARG;
		goto modem_deint;
	}

#if CONFIG_SW_COEXIST_ENABLE
	coex_init();
#endif // CONFIG_SW_COEXIST_ENABLE

	ret = esp_ble_register_bb_funcs();
	if (ret != ESP_OK) {
		LOG_WRN("esp_ble_register_bb_funcs failed %d", ret);
		goto modem_deint;
	}

	ret = r_ble_controller_init(cfg);
	if (ret != ESP_OK) {
		LOG_WRN("r_ble_controller_init failed %d", ret);
		goto modem_deint;
	}

	LOG_INF("ble controller commit:[%s]", ble_controller_get_compile_version());
	r_esp_ble_change_rtc_freq(slow_clk_freq);

	ble_controller_scan_duplicate_config();

	ret = os_msys_init();
	if (ret != ESP_OK) {
		LOG_WRN("msys_init failed %d", ret);
		goto free_controller;
	}

	ret = controller_sleep_init();
	if (ret != ESP_OK) {
		LOG_WRN("controller_sleep_init failed %d", ret);
		goto free_controller;
	}

	ESP_ERROR_CHECK(esp_read_mac((uint8_t *)mac, ESP_MAC_BT));
	LOG_INF("Bluetooth MAC: %02x:%02x:%02x:%02x:%02x:%02x",
			 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	swap_in_place(mac, 6);
	r_esp_ble_ll_set_public_addr(mac);

	ble_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

	hci_transport_mode = HCI_TRANSPORT_VHCI;

	ret = hci_transport_init(hci_transport_mode);
	if (ret) {
		LOG_WRN("hci transport init failed %d", ret);
		goto free_controller;
	}

	return ESP_OK;
free_controller:
	hci_transport_deinit();
	controller_sleep_deinit();
	os_msys_deinit();
	r_ble_controller_deinit();
modem_deint:
	esp_ble_unregister_bb_funcs();
	esp_phy_modem_deinit();
	modem_clock_deselect_lp_clock_source(PERIPH_BT_MODULE);
	modem_clock_module_disable(PERIPH_BT_MODULE);
free_mem:
	npl_freertos_mempool_deinit();
	esp_unregister_npl_funcs();
	npl_freertos_funcs_deinit();
	esp_unregister_ext_funcs();
	return ret;
}

esp_err_t esp_bt_controller_deinit(void)
{
	if ((ble_controller_status < ESP_BT_CONTROLLER_STATUS_INITED) ||
		(ble_controller_status >= ESP_BT_CONTROLLER_STATUS_ENABLED)) {
		LOG_WRN("invalid controller state");
		return ESP_FAIL;
	}

	hci_transport_deinit();
	controller_sleep_deinit();

	os_msys_deinit();

	esp_phy_modem_deinit();
	modem_clock_deselect_lp_clock_source(PERIPH_BT_MODULE);
	modem_clock_module_disable(PERIPH_BT_MODULE);

	r_ble_controller_deinit();
	esp_ble_unregister_bb_funcs();

	esp_unregister_npl_funcs();

	esp_unregister_ext_funcs();

	/* De-initialize npl functions */
	npl_freertos_funcs_deinit();

	npl_freertos_mempool_deinit();

	ble_controller_status = ESP_BT_CONTROLLER_STATUS_IDLE;

	return ESP_OK;
}

esp_err_t esp_bt_controller_enable(esp_bt_mode_t mode)
{
	esp_err_t ret = ESP_OK;

	if (mode != ESP_BT_MODE_BLE) {
		LOG_WRN("invalid controller mode");
		return ESP_FAIL;
	}
	if (ble_controller_status != ESP_BT_CONTROLLER_STATUS_INITED) {
		LOG_WRN("invalid controller state");
		return ESP_FAIL;
	}
	if (!s_ble_active) {
#if CONFIG_PM_ENABLE
		esp_pm_lock_acquire(s_pm_lock);
#endif  // CONFIG_PM_ENABLE
		esp_phy_enable(PHY_MODEM_BT);
		s_ble_active = true;
	}
	esp_btbb_enable();
#if CONFIG_SW_COEXIST_ENABLE
	coex_enable();
#endif // CONFIG_SW_COEXIST_ENABLE

	if (r_ble_controller_enable(mode) != 0) {
		ret = ESP_FAIL;
		goto error;
	}
	ble_controller_status = ESP_BT_CONTROLLER_STATUS_ENABLED;
	return ESP_OK;

error:
#if CONFIG_SW_COEXIST_ENABLE
	coex_disable();
#endif
	esp_btbb_disable();
	if (s_ble_active) {
		esp_phy_disable(PHY_MODEM_BT);
#if CONFIG_PM_ENABLE
		esp_pm_lock_release(s_pm_lock);
#endif  // CONFIG_PM_ENABLE
		s_ble_active = false;
	}
	return ret;
}

esp_err_t esp_bt_controller_disable(void)
{
	if (ble_controller_status < ESP_BT_CONTROLLER_STATUS_ENABLED) {
		LOG_WRN("invalid controller state");
		return ESP_FAIL;
	}
	if (r_ble_controller_disable() != 0) {
		return ESP_FAIL;
	}
#if CONFIG_SW_COEXIST_ENABLE
	coex_disable();
#endif
	esp_btbb_disable();
	if (s_ble_active) {
		esp_phy_disable(PHY_MODEM_BT);
#if CONFIG_PM_ENABLE
		esp_pm_lock_release(s_pm_lock);
#endif  // CONFIG_PM_ENABLE
		s_ble_active = false;
	}
	ble_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;
	return ESP_OK;
}

esp_err_t esp_bt_controller_mem_release(esp_bt_mode_t mode)
{
	LOG_WRN("%s not implemented, return OK", __func__);
	return ESP_OK;
}

esp_err_t esp_bt_mem_release(esp_bt_mode_t mode)
{
	LOG_WRN("%s not implemented, return OK", __func__);
	return ESP_OK;
}

esp_bt_controller_status_t esp_bt_controller_get_status(void)
{
	return ble_controller_status;
}

esp_err_t esp_ble_tx_power_set(esp_ble_power_type_t power_type, esp_power_level_t power_level)
{
	esp_err_t stat = ESP_FAIL;

	switch (power_type) {
	case ESP_BLE_PWR_TYPE_DEFAULT:
	case ESP_BLE_PWR_TYPE_ADV:
	case ESP_BLE_PWR_TYPE_SCAN:
		if (r_ble_txpwr_set(ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT, 0, power_level) == 0) {
			stat = ESP_OK;
		}
		break;
	case ESP_BLE_PWR_TYPE_CONN_HDL0:
	case ESP_BLE_PWR_TYPE_CONN_HDL1:
	case ESP_BLE_PWR_TYPE_CONN_HDL2:
	case ESP_BLE_PWR_TYPE_CONN_HDL3:
	case ESP_BLE_PWR_TYPE_CONN_HDL4:
	case ESP_BLE_PWR_TYPE_CONN_HDL5:
	case ESP_BLE_PWR_TYPE_CONN_HDL6:
	case ESP_BLE_PWR_TYPE_CONN_HDL7:
	case ESP_BLE_PWR_TYPE_CONN_HDL8:
		if (r_ble_txpwr_set(ESP_BLE_ENHANCED_PWR_TYPE_CONN, power_type, power_level) == 0) {
			stat = ESP_OK;
		}
		break;
	default:
		stat = ESP_ERR_NOT_SUPPORTED;
		break;
	}

	return stat;
}

esp_err_t esp_ble_tx_power_set_enhanced(esp_ble_enhanced_power_type_t power_type, uint16_t handle,
										esp_power_level_t power_level)
{
	esp_err_t stat = ESP_FAIL;
	switch (power_type) {
	case ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT:
	case ESP_BLE_ENHANCED_PWR_TYPE_SCAN:
	case ESP_BLE_ENHANCED_PWR_TYPE_INIT:
		if (r_ble_txpwr_set(ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT, 0, power_level) == 0) {
			stat = ESP_OK;
		}
		break;
	case ESP_BLE_ENHANCED_PWR_TYPE_ADV:
	case ESP_BLE_ENHANCED_PWR_TYPE_CONN:
		if (r_ble_txpwr_set(power_type, handle, power_level) == 0) {
			stat = ESP_OK;
		}
		break;
	default:
		stat = ESP_ERR_NOT_SUPPORTED;
		break;
	}

	return stat;
}

esp_power_level_t esp_ble_tx_power_get(esp_ble_power_type_t power_type)
{
	int tx_level = 0;

	switch (power_type) {
	case ESP_BLE_PWR_TYPE_ADV:
	case ESP_BLE_PWR_TYPE_SCAN:
	case ESP_BLE_PWR_TYPE_DEFAULT:
		tx_level = r_ble_txpwr_get(ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT, 0);
		break;
	case ESP_BLE_PWR_TYPE_CONN_HDL0:
	case ESP_BLE_PWR_TYPE_CONN_HDL1:
	case ESP_BLE_PWR_TYPE_CONN_HDL2:
	case ESP_BLE_PWR_TYPE_CONN_HDL3:
	case ESP_BLE_PWR_TYPE_CONN_HDL4:
	case ESP_BLE_PWR_TYPE_CONN_HDL5:
	case ESP_BLE_PWR_TYPE_CONN_HDL6:
	case ESP_BLE_PWR_TYPE_CONN_HDL7:
	case ESP_BLE_PWR_TYPE_CONN_HDL8:
		tx_level = r_ble_txpwr_get(ESP_BLE_ENHANCED_PWR_TYPE_CONN, power_type);
		break;
	default:
		return ESP_PWR_LVL_INVALID;
	}

	if (tx_level < 0) {
		return ESP_PWR_LVL_INVALID;
	}

	return (esp_power_level_t)tx_level;
}

esp_power_level_t esp_ble_tx_power_get_enhanced(esp_ble_enhanced_power_type_t power_type,
												uint16_t handle)
{
	int tx_level = 0;

	switch (power_type) {
	case ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT:
	case ESP_BLE_ENHANCED_PWR_TYPE_SCAN:
	case ESP_BLE_ENHANCED_PWR_TYPE_INIT:
		tx_level = r_ble_txpwr_get(ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT, 0);
		break;
	case ESP_BLE_ENHANCED_PWR_TYPE_ADV:
	case ESP_BLE_ENHANCED_PWR_TYPE_CONN:
		tx_level = r_ble_txpwr_get(power_type, handle);
		break;
	default:
		return ESP_PWR_LVL_INVALID;
	}

	if (tx_level < 0) {
	   return ESP_PWR_LVL_INVALID;
	}

	return (esp_power_level_t)tx_level;
}

#if CONFIG_BT_LE_SM_LEGACY || CONFIG_BT_LE_SM_SC
#define BLE_SM_KEY_ERR 0x17
#if CONFIG_BT_LE_CRYPTO_STACK_MBEDTLS
#include "mbedtls/aes.h"
#if CONFIG_BT_LE_SM_SC
#include "mbedtls/cipher.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/cmac.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"

static mbedtls_ecp_keypair keypair;
#endif // CONFIG_BT_LE_SM_SC

#else
#include "tinycrypt/aes.h"
#include "tinycrypt/constants.h"
#include "tinycrypt/utils.h"

#if CONFIG_BT_LE_SM_SC
#include "tinycrypt/cmac_mode.h"
#include "tinycrypt/ecc_dh.h"
#endif // CONFIG_BT_LE_SM_SC
#endif // CONFIG_BT_LE_CRYPTO_STACK_MBEDTLS

/* Based on Core Specification 4.2 Vol 3. Part H 2.3.5.6.1 */
static const uint8_t ble_sm_alg_dbg_priv_key[32] = {
	0x3f, 0x49, 0xf6, 0xd4, 0xa3, 0xc5, 0x5f, 0x38, 0x74, 0xc9, 0xb3, 0xe3,
	0xd2, 0x10, 0x3f, 0x50, 0x4a, 0xff, 0x60, 0x7b, 0xeb, 0x40, 0xb7, 0x99,
	0x58, 0x99, 0xb8, 0xa6, 0xcd, 0x3c, 0x1a, 0xbd
};

int ble_sm_alg_gen_dhkey(const uint8_t *peer_pub_key_x, const uint8_t *peer_pub_key_y,
						 const uint8_t *our_priv_key, uint8_t *out_dhkey)
{
	uint8_t dh[32];
	uint8_t pk[64];
	uint8_t priv[32];
	int rc = BLE_SM_KEY_ERR;

	swap_buf(pk, peer_pub_key_x, 32);
	swap_buf(&pk[32], peer_pub_key_y, 32);
	swap_buf(priv, our_priv_key, 32);

#if CONFIG_BT_LE_CRYPTO_STACK_MBEDTLS
	struct mbedtls_ecp_point pt = {0}, Q = {0};
	mbedtls_mpi z = {0}, d = {0};
	mbedtls_ctr_drbg_context ctr_drbg = {0};
	mbedtls_entropy_context entropy = {0};

	uint8_t pub[65] = {0};
	/* Hardcoded first byte of pub key for MBEDTLS_ECP_PF_UNCOMPRESSED */
	pub[0] = 0x04;
	memcpy(&pub[1], pk, 64);

	/* Initialize the required structures here */
	mbedtls_ecp_point_init(&pt);
	mbedtls_ecp_point_init(&Q);
	mbedtls_ctr_drbg_init(&ctr_drbg);
	mbedtls_entropy_init(&entropy);
	mbedtls_mpi_init(&d);
	mbedtls_mpi_init(&z);

	/* Below 3 steps are to validate public key on curve secp256r1 */
	if (mbedtls_ecp_group_load(&keypair.MBEDTLS_PRIVATE(grp), MBEDTLS_ECP_DP_SECP256R1) != 0) {
		goto exit;
	}

	if (mbedtls_ecp_point_read_binary(&keypair.MBEDTLS_PRIVATE(grp), &pt, pub, 65) != 0) {
		goto exit;
	}

	if (mbedtls_ecp_check_pubkey(&keypair.MBEDTLS_PRIVATE(grp), &pt) != 0) {
		goto exit;
	}

	/* Set PRNG */
	if ((rc = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy, NULL, 0)) != 0) {
		goto exit;
	}

	/* Prepare point Q from pub key */
	if (mbedtls_ecp_point_read_binary(&keypair.MBEDTLS_PRIVATE(grp), &Q, pub, 65) != 0) {
		goto exit;
	}

	if (mbedtls_mpi_read_binary(&d, priv, 32) != 0) {
		goto exit;
	}

	rc = mbedtls_ecdh_compute_shared(&keypair.MBEDTLS_PRIVATE(grp), &z, &Q, &d,
									 mbedtls_ctr_drbg_random, &ctr_drbg);
	if (rc != 0) {
		goto exit;
	}

	rc = mbedtls_mpi_write_binary(&z, dh, 32);
	if (rc != 0) {
		goto exit;
	}

exit:
	mbedtls_ecp_point_free(&pt);
	mbedtls_mpi_free(&z);
	mbedtls_mpi_free(&d);
	mbedtls_ecp_point_free(&Q);
	mbedtls_entropy_free(&entropy);
	mbedtls_ctr_drbg_free(&ctr_drbg);
	if (rc != 0) {
		return BLE_SM_KEY_ERR;
	}

#else
	if (uECC_valid_public_key(pk, &curve_secp256r1) < 0) {
		return BLE_SM_KEY_ERR;
	}

	rc = uECC_shared_secret(pk, priv, dh, &curve_secp256r1);
	if (rc == TC_CRYPTO_FAIL) {
		return BLE_SM_KEY_ERR;
	}
#endif // CONFIG_BT_LE_CRYPTO_STACK_MBEDTLS

	swap_buf(out_dhkey, dh, 32);
	return 0;
}

#if CONFIG_BT_LE_CRYPTO_STACK_MBEDTLS
static int mbedtls_gen_keypair(uint8_t *public_key, uint8_t *private_key)
{
	int rc = BLE_SM_KEY_ERR;
	mbedtls_entropy_context entropy = {0};
	mbedtls_ctr_drbg_context ctr_drbg = {0};

	mbedtls_entropy_init(&entropy);
	mbedtls_ctr_drbg_init(&ctr_drbg);
	mbedtls_ecp_keypair_init(&keypair);

	if ((rc = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
									NULL, 0)) != 0) {
		goto exit;
	}

	if ((rc = mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, &keypair,
								  mbedtls_ctr_drbg_random, &ctr_drbg)) != 0) {
		goto exit;
	}

	if ((rc = mbedtls_mpi_write_binary(&keypair.MBEDTLS_PRIVATE(d), private_key, 32)) != 0) {
		goto exit;
	}

	size_t olen = 0;
	uint8_t pub[65] = {0};

	if ((rc = mbedtls_ecp_point_write_binary(&keypair.MBEDTLS_PRIVATE(grp), &keypair.MBEDTLS_PRIVATE(Q), MBEDTLS_ECP_PF_UNCOMPRESSED,
			  &olen, pub, 65)) != 0) {
		goto exit;
	}

	memcpy(public_key, &pub[1], 64);

exit:
	mbedtls_ctr_drbg_free(&ctr_drbg);
	mbedtls_entropy_free(&entropy);
	if (rc != 0) {
		mbedtls_ecp_keypair_free(&keypair);
		return BLE_SM_KEY_ERR;
	}

	return 0;
}
#endif  // CONFIG_BT_LE_CRYPTO_STACK_MBEDTLS

/**
 * pub: 64 bytes
 * priv: 32 bytes
 */
int ble_sm_alg_gen_key_pair(uint8_t *pub, uint8_t *priv)
{
#if CONFIG_BT_LE_SM_SC_DEBUG_KEYS
	swap_buf(pub, ble_sm_alg_dbg_pub_key, 32);
	swap_buf(&pub[32], &ble_sm_alg_dbg_pub_key[32], 32);
	swap_buf(priv, ble_sm_alg_dbg_priv_key, 32);
#else
	uint8_t pk[64];

	do {
#if CONFIG_BT_LE_CRYPTO_STACK_MBEDTLS
		if (mbedtls_gen_keypair(pk, priv) != 0) {
			return BLE_SM_KEY_ERR;
		}
#else
		if (uECC_make_key(pk, priv, &curve_secp256r1) != TC_CRYPTO_SUCCESS) {
			return BLE_SM_KEY_ERR;
		}
#endif  // CONFIG_BT_LE_CRYPTO_STACK_MBEDTLS
		/* Make sure generated key isn't debug key. */
	} while (memcmp(priv, ble_sm_alg_dbg_priv_key, 32) == 0);

	swap_buf(pub, pk, 32);
	swap_buf(&pub[32], &pk[32], 32);
	swap_in_place(priv, 32);
#endif // CONFIG_BT_LE_SM_SC_DEBUG_KEYS
	return 0;
}

#endif // CONFIG_BT_LE_SM_LEGACY || CONFIG_BT_LE_SM_SC






























static void esp_bt_free(void *mem);

/* Local variable definition
 ***************************************************************************
 */

static const struct osi_funcs_t osi_funcs_ro = {
	._magic = OSI_MAGIC_VALUE,
	._version = OSI_VERSION,
	._interrupt_alloc = interrupt_alloc_wrapper,
	._interrupt_free = interrupt_free_wrapper,
	._interrupt_handler_set_rsv = NULL,
	._global_intr_disable = global_interrupt_disable,
	._global_intr_restore = global_interrupt_restore,
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
	._is_in_isr = is_in_isr_wrapper,
	._cause_sw_intr_to_core = NULL,
	._malloc = malloc_internal_wrapper,
	._malloc_internal = malloc_internal_wrapper,
	._free = esp_bt_free,
	._read_efuse_mac = read_mac_wrapper,
	._srand = srand_wrapper,
	._rand = rand_wrapper,
	._btdm_lpcycles_2_hus = btdm_lpcycles_2_hus,
	._btdm_hus_2_lpcycles = btdm_hus_2_lpcycles,
	._btdm_sleep_check_duration = btdm_sleep_check_duration,
	._btdm_sleep_enter_phase1 = btdm_sleep_enter_phase1_wrapper,
	._btdm_sleep_enter_phase2 = btdm_sleep_enter_phase2_wrapper,
	._btdm_sleep_exit_phase1 = NULL,
	._btdm_sleep_exit_phase2 = NULL,
	._btdm_sleep_exit_phase3 = btdm_sleep_exit_phase3_wrapper,
	._coex_wifi_sleep_set = coex_wifi_sleep_set_hook,
	._coex_core_ble_conn_dyn_prio_get = NULL,
	._coex_schm_register_btdm_callback = coex_schm_register_btdm_callback_wrapper,
	._coex_schm_status_bit_set = coex_schm_status_bit_set_wrapper,
	._coex_schm_status_bit_clear = coex_schm_status_bit_clear_wrapper,
	._coex_schm_interval_get = coex_schm_interval_get_wrapper,
	._coex_schm_curr_period_get = coex_schm_curr_period_get_wrapper,
	._coex_schm_curr_phase_get = coex_schm_curr_phase_get_wrapper,
	._interrupt_enable = interrupt_enable_wrapper,
	._interrupt_disable = interrupt_disable_wrapper,
	._esp_hw_power_down = btdm_hw_mac_power_down_wrapper,
	._esp_hw_power_up = btdm_hw_mac_power_up_wrapper,
	._ets_backup_dma_copy = btdm_backup_dma_copy_wrapper,
	._ets_delay_us = esp_rom_delay_us,
	._btdm_rom_table_ready = btdm_funcs_table_ready_wrapper,
	._coex_bt_wakeup_request = coex_bt_wakeup_request,
	._coex_bt_wakeup_request_end = coex_bt_wakeup_request_end,
};

static DRAM_ATTR struct osi_funcs_t *osi_funcs_p;

/* Static variable declare */
static DRAM_ATTR esp_bt_controller_status_t btdm_controller_status = ESP_BT_CONTROLLER_STATUS_IDLE;

// low power control struct
static DRAM_ATTR btdm_lpcntl_t s_lp_cntl;
// low power status struct
static DRAM_ATTR btdm_lpstat_t s_lp_stat;
// measured average low power clock period in micro seconds
static DRAM_ATTR uint32_t btdm_lpcycle_us = 0;
// number of fractional bit for btdm_lpcycle_us
static DRAM_ATTR uint8_t btdm_lpcycle_us_frac = 0;
// wakeup timer
static DRAM_ATTR esp_timer_handle_t s_btdm_slp_tmr;

static unsigned int global_int_mux;
static unsigned int global_nested_counter = 0;

K_THREAD_STACK_DEFINE(bt_stack, CONFIG_ESP32_BT_CONTROLLER_STACK_SIZE);
static struct k_thread bt_task_handle;

static DRAM_ATTR struct k_sem *s_wakeup_req_sem = NULL;

void IRAM_ATTR btdm_hw_mac_power_down_wrapper(void)
{
#if CONFIG_MAC_BB_PD
#if SOC_PM_SUPPORT_BT_PD
	// le module power down
	SET_PERI_REG_MASK(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_BT_FORCE_ISO);
	SET_PERI_REG_MASK(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_BT_FORCE_PD);
#endif
	esp_mac_bb_power_down();
#endif
}

void IRAM_ATTR btdm_hw_mac_power_up_wrapper(void)
{
#if CONFIG_MAC_BB_PD
#if SOC_PM_SUPPORT_BT_PD
	// le module power up
	CLEAR_PERI_REG_MASK(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_BT_FORCE_PD);
	CLEAR_PERI_REG_MASK(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_BT_FORCE_ISO);
#endif
	esp_mac_bb_power_up();
#endif
}

void IRAM_ATTR btdm_backup_dma_copy_wrapper(uint32_t reg, uint32_t mem_addr, uint32_t num,  bool to_mem)
{
#if CONFIG_MAC_BB_PD
	ets_backup_dma_copy(reg, mem_addr, num, to_mem);
#endif
}

static inline void esp_bt_power_domain_on(void)
{
	// Bluetooth module power up
#if SOC_PM_SUPPORT_BT_PD
	CLEAR_PERI_REG_MASK(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_BT_FORCE_PD);
	CLEAR_PERI_REG_MASK(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_BT_FORCE_ISO);
#endif
	esp_wifi_bt_power_domain_on();
}

static inline void esp_bt_power_domain_off(void)
{
	// Bluetooth module power down
#if SOC_PM_SUPPORT_BT_PD
	SET_PERI_REG_MASK(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_BT_FORCE_ISO);
	SET_PERI_REG_MASK(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_BT_FORCE_PD);
#endif
	esp_wifi_bt_power_domain_off();
}

static void btdm_intr_alloc(void *arg)
{
	btdm_isr_alloc_t *p = arg;
	p->ret = esp_intr_alloc(p->source,	p->flags, (isr_handler_t)p->fn, p->arg, NULL);
}

static int interrupt_alloc_wrapper(int cpu_id, int source, isr_handler_t handler, void *arg, void **ret_handle)
{
	btdm_isr_alloc_t p;
	p.source = source;
	p.flags = ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM;
	p.fn = (void *)handler;
	p.arg = arg;
	p.handle = (isr_handler_t *)ret_handle;
	btdm_intr_alloc(&p);
	return p.ret;
}

static int interrupt_free_wrapper(void *handle)
{
	/* TODO: implement esp_intr_free() for ESP32-C3 */
	return ESP_OK;
}

static int interrupt_enable_wrapper(void *handle)
{
	ARG_UNUSED(handle);

	return ESP_OK;
}

static int interrupt_disable_wrapper(void *handle)
{
	ARG_UNUSED(handle);

	return ESP_OK;
}

static void IRAM_ATTR global_interrupt_disable(void)
{
	if (global_nested_counter == 0) {
		global_int_mux = irq_lock();
	}

	if (global_nested_counter < 0xFFFFFFFF) {
		global_nested_counter++;
	}
}

static void IRAM_ATTR global_interrupt_restore(void)
{
	if (global_nested_counter > 0) {
		global_nested_counter--;
	}

	if (global_nested_counter == 0) {
		irq_unlock(global_int_mux);
	}
}

static void IRAM_ATTR task_yield_from_isr(void)
{
	k_yield();
}

static bool IRAM_ATTR is_in_isr_wrapper(void)
{
	return (bool)k_is_in_isr();
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

static uint32_t IRAM_ATTR btdm_lpcycles_2_hus(uint32_t cycles, uint32_t *error_corr)
{
	uint64_t local_error_corr = (error_corr == NULL) ? 0 : (uint64_t)(*error_corr);
	uint64_t res = (uint64_t)btdm_lpcycle_us * cycles * 2;
	local_error_corr += res;
	res = (local_error_corr >> btdm_lpcycle_us_frac);
	local_error_corr -= (res << btdm_lpcycle_us_frac);
	if (error_corr) {
			*error_corr = (uint32_t) local_error_corr;
	}
	return (uint32_t)res;
}

/*
 * @brief Converts a duration in half us into a number of low power clock cycles.
 */
static uint32_t IRAM_ATTR btdm_hus_2_lpcycles(uint32_t hus)
{
	/* The number of sleep duration(us) should not lead to overflow. Thrs: 100s
	 * Compute the sleep duration in us to low power clock cycles, with calibration result applied
	 * clock measurement is conducted
	 */
	uint64_t cycles = ((uint64_t)(hus) << btdm_lpcycle_us_frac) / btdm_lpcycle_us;
	cycles >>= 1;

	return (uint32_t)cycles;
}

static bool IRAM_ATTR btdm_sleep_check_duration(int32_t *half_slot_cnt)
{
	if (*half_slot_cnt < BTDM_MIN_SLEEP_DURATION) {
		return false;
	}
	/* wake up in advance considering the delay in enabling PHY/RF */
	*half_slot_cnt -= BTDM_MODEM_WAKE_UP_DELAY;
	return true;
}

static void btdm_sleep_enter_phase1_wrapper(uint32_t lpcycles)
{
	if (s_lp_cntl.wakeup_timer_required == 0) {
		return;
	}

	// start a timer to wake up and acquire the pm_lock before modem_sleep awakes
	uint32_t us_to_sleep = btdm_lpcycles_2_hus(lpcycles, NULL) >> 1;

#define BTDM_MIN_TIMER_UNCERTAINTY_US	  (1800)
	assert(us_to_sleep > BTDM_MIN_TIMER_UNCERTAINTY_US);
	// allow a maximum time uncertainty to be about 488ppm(1/2048) at least as clock drift
	// and set the timer in advance
	uint32_t uncertainty = (us_to_sleep >> 11);
	if (uncertainty < BTDM_MIN_TIMER_UNCERTAINTY_US) {
		uncertainty = BTDM_MIN_TIMER_UNCERTAINTY_US;
	}

	assert (s_lp_stat.wakeup_timer_started == 0);
	if (esp_timer_start_once(s_btdm_slp_tmr, us_to_sleep - uncertainty) == ESP_OK) {
		s_lp_stat.wakeup_timer_started = 1;
	} else {
		LOG_ERR("timer start failed");
		assert(0);
	}
}

static void btdm_sleep_enter_phase2_wrapper(void)
{
	if (btdm_controller_get_sleep_mode() == ESP_BT_SLEEP_MODE_1) {
		if (s_lp_stat.phy_enabled) {
			esp_phy_disable(PHY_MODEM_BT);
			s_lp_stat.phy_enabled = 0;
		} else {
			assert(0);
		}

		if (s_lp_stat.pm_lock_released == 0) {
			s_lp_stat.pm_lock_released = 1;
		}
	}
}

static void btdm_sleep_exit_phase3_wrapper(void)
{
	if (btdm_controller_get_sleep_mode() == ESP_BT_SLEEP_MODE_1) {
		if (s_lp_stat.phy_enabled == 0) {
			esp_phy_enable(PHY_MODEM_BT);
			s_lp_stat.phy_enabled = 1;
		}
	}

	// If BT wakeup before esp timer coming due to timer task have no chance to run.
	// Then we will not run into `btdm_sleep_exit_phase0` and stop esp timer,
	// Do it again here to fix this issue.
	if (s_lp_cntl.wakeup_timer_required && s_lp_stat.wakeup_timer_started) {
		esp_timer_stop(s_btdm_slp_tmr);
		s_lp_stat.wakeup_timer_started = 0;
	}

	// wait for the sleep state to change
	// the procedure duration is at micro-second level or less
	while (btdm_sleep_clock_sync()) {
		;
	}
}

static void IRAM_ATTR btdm_sleep_exit_phase0(void *param)
{
	assert(s_lp_cntl.enable == 1);

	int event = (int) param;
	if (event == BTDM_ASYNC_WAKEUP_SRC_VHCI || event == BTDM_ASYNC_WAKEUP_SRC_DISA) {
		btdm_wakeup_request();
	}

	if (s_lp_cntl.wakeup_timer_required && s_lp_stat.wakeup_timer_started) {
		esp_timer_stop(s_btdm_slp_tmr);
		s_lp_stat.wakeup_timer_started = 0;
	}

	if (event == BTDM_ASYNC_WAKEUP_SRC_VHCI || event == BTDM_ASYNC_WAKEUP_SRC_DISA) {
		semphr_give_wrapper(s_wakeup_req_sem);
	}
}

static void IRAM_ATTR btdm_slp_tmr_callback(void *arg)
{
}

static bool async_wakeup_request(int event)
{
	if (s_lp_cntl.enable == 0) {
		return false;
	}

	bool do_wakeup_request = false;
	switch (event) {
		case BTDM_ASYNC_WAKEUP_SRC_VHCI:
		case BTDM_ASYNC_WAKEUP_SRC_DISA:
			btdm_in_wakeup_requesting_set(true);
			if (!btdm_power_state_active()) {
				r_btdm_vnd_offload_post(BTDM_VND_OL_SIG_WAKEUP_TMR, (void *)event);
				do_wakeup_request = true;
				semphr_take_wrapper(s_wakeup_req_sem, OSI_FUNCS_TIME_BLOCKING);
			}
			break;
		default:
			break;
	}

	return do_wakeup_request;
}

static void async_wakeup_request_end(int event)
{
	if (s_lp_cntl.enable == 0) {
		return;
	}

	bool allow_to_sleep;
	switch (event) {
		case BTDM_ASYNC_WAKEUP_SRC_VHCI:
		case BTDM_ASYNC_WAKEUP_SRC_DISA:
			allow_to_sleep = true;
			break;
		default:
			allow_to_sleep = true;
			break;
	}

	if (allow_to_sleep) {
		btdm_in_wakeup_requesting_set(false);
	}

	return;
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

static void btdm_funcs_table_ready_wrapper(void)
{
#if BT_BLE_CCA_MODE == 2
	btdm_cca_feature_enable();
#endif
}

bool bt_async_wakeup_request(void)
{
	return async_wakeup_request(BTDM_ASYNC_WAKEUP_SRC_VHCI);
}

void bt_wakeup_request_end(void)
{
	async_wakeup_request_end(BTDM_ASYNC_WAKEUP_SRC_VHCI);
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

bool esp_vhci_host_check_send_available(void)
{
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return false;
	}
	return API_vhci_host_check_send_available();
}

void esp_vhci_host_send_packet(uint8_t *data, uint16_t len)
{
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return;
	}
	async_wakeup_request(BTDM_ASYNC_WAKEUP_SRC_VHCI);

	API_vhci_host_send_packet(data, len);

	async_wakeup_request_end(BTDM_ASYNC_WAKEUP_SRC_VHCI);
}

esp_err_t esp_vhci_host_register_callback(const esp_vhci_host_callback_t *callback)
{
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return ESP_FAIL;
	}

	return API_vhci_host_register_callback((const vhci_host_callback_t *)callback) == 0 ? ESP_OK : ESP_FAIL;
}


static void btdm_controller_mem_init(void)
{
	extern void btdm_controller_rom_data_init(void );
	btdm_controller_rom_data_init();
}

esp_err_t esp_bt_controller_mem_release(esp_bt_mode_t mode)
{
	LOG_WRN("%s not implemented, return OK", __func__);
	return ESP_OK;
}

esp_err_t esp_bt_mem_release(esp_bt_mode_t mode)
{
	LOG_WRN("%s not implemented, return OK", __func__);
	return ESP_OK;
}

#if CONFIG_MAC_BB_PD
static void IRAM_ATTR btdm_mac_bb_power_down_cb(void)
{
	if (s_lp_cntl.mac_bb_pd && s_lp_stat.mac_bb_pd == 0) {
		btdm_ble_power_down_dma_copy(true);
		s_lp_stat.mac_bb_pd = 1;
	}
}

static void IRAM_ATTR btdm_mac_bb_power_up_cb(void)
{
	if (s_lp_cntl.mac_bb_pd && s_lp_stat.mac_bb_pd) {
		btdm_ble_power_down_dma_copy(false);
		s_lp_stat.mac_bb_pd = 0;
	}
}
#endif

esp_err_t esp_bt_controller_init(esp_bt_controller_config_t *cfg)
{
	esp_err_t err = ESP_FAIL;

	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_IDLE) {
		return ESP_ERR_INVALID_STATE;
	}

	if (cfg == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	if (cfg->controller_task_prio != CONFIG_ESP32_BT_CONTROLLER_TASK_PRIO
		|| cfg->controller_task_stack_size < CONFIG_ESP32_BT_CONTROLLER_STACK_SIZE) {
		LOG_ERR("Invalid controller task prioriy or stack size");
		return ESP_ERR_INVALID_ARG;
	}

	if (cfg->bluetooth_mode != ESP_BT_MODE_BLE) {
		LOG_ERR("%s controller only support BLE only mode", __func__);
		return ESP_ERR_NOT_SUPPORTED;
	}

	if (cfg->bluetooth_mode & ESP_BT_MODE_BLE) {
		if ((cfg->ble_max_act <= 0) || (cfg->ble_max_act > BT_CTRL_BLE_MAX_ACT_LIMIT)) {
			LOG_ERR("Invalid value of ble_max_act");
			return ESP_ERR_INVALID_ARG;
		}
	}

	if (cfg->sleep_mode == ESP_BT_SLEEP_MODE_1) {
		if (cfg->sleep_clock == ESP_BT_SLEEP_CLOCK_NONE) {
			LOG_ERR("SLEEP_MODE_1 enabled but sleep clock not configured");
			return ESP_ERR_INVALID_ARG;
		}
	}

	// overwrite some parameters
	cfg->magic = ESP_BT_CTRL_CONFIG_MAGIC_VAL;

#if CONFIG_MAC_BB_PD
	esp_mac_bb_pd_mem_init();
#endif
	esp_phy_modem_init();
	esp_bt_power_domain_on();

	btdm_controller_mem_init();

#if CONFIG_MAC_BB_PD
	if (esp_register_mac_bb_pd_callback(btdm_mac_bb_power_down_cb) != 0) {
		err = ESP_ERR_INVALID_ARG;
		goto error;
	}

	if (esp_register_mac_bb_pu_callback(btdm_mac_bb_power_up_cb) != 0) {
		err = ESP_ERR_INVALID_ARG;
		goto error;
	}
#endif

	osi_funcs_p = (struct osi_funcs_t *)malloc_internal_wrapper(sizeof(struct osi_funcs_t));
	if (osi_funcs_p == NULL) {
		return ESP_ERR_NO_MEM;
	}

	memcpy(osi_funcs_p, &osi_funcs_ro, sizeof(struct osi_funcs_t));
	if (btdm_osi_funcs_register(osi_funcs_p) != 0) {
		return ESP_ERR_INVALID_ARG;
	}

	LOG_INF("BT controller compile version [%s]", btdm_controller_get_compile_version());

	// init low-power control resources
	do {
		// set default values for global states or resources
		s_lp_stat.val = 0;
		s_lp_cntl.val = 0;
		s_lp_cntl.main_xtal_pu = 0;
		s_wakeup_req_sem = NULL;
		s_btdm_slp_tmr = NULL;

		// configure and initialize resources
		s_lp_cntl.enable = (cfg->sleep_mode == ESP_BT_SLEEP_MODE_1) ? 1 : 0;
		s_lp_cntl.no_light_sleep = 0;

		if (s_lp_cntl.enable) {
#if (CONFIG_MAC_BB_PD)
			if (!btdm_deep_sleep_mem_init()) {
				err = ESP_ERR_NO_MEM;
				goto error;
			}
			s_lp_cntl.mac_bb_pd = 1;
#endif
			// async wakeup semaphore for VHCI
			s_wakeup_req_sem = semphr_create_wrapper(1, 0);
			if (s_wakeup_req_sem == NULL) {
				err = ESP_ERR_NO_MEM;
				goto error;
			}
			btdm_vnd_offload_task_register(BTDM_VND_OL_SIG_WAKEUP_TMR, btdm_sleep_exit_phase0);
		}

		if (s_lp_cntl.wakeup_timer_required) {
			esp_timer_create_args_t create_args = {
				.callback = btdm_slp_tmr_callback,
				.arg = NULL,
				.name = "btSlp",
			};
			if ((err = esp_timer_create(&create_args, &s_btdm_slp_tmr)) != ESP_OK) {
				goto error;
			}
		}

		// set default bluetooth sleep clock cycle and its fractional bits
		btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
		btdm_lpcycle_us = 2 << (btdm_lpcycle_us_frac);

		// set default bluetooth sleep clock source
		s_lp_cntl.lpclk_sel = BTDM_LPCLK_SEL_XTAL;  // set default value
#if CONFIG_BT_CTRL_LPCLK_SEL_EXT_32K_XTAL
		// check whether or not EXT_CRYS is working
		if (rtc_clk_slow_freq_get() == SOC_RTC_SLOW_CLK_SRC_XTAL32K) {
			s_lp_cntl.lpclk_sel = BTDM_LPCLK_SEL_XTAL32K; // External 32 kHz XTAL
		} else {
			LOG_WRN("32.768kHz XTAL not detected, fall back to main XTAL as Bluetooth sleep clock\n"
				 "light sleep mode will not be able to apply when bluetooth is enabled");
#if !CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP
		s_lp_cntl.no_light_sleep = 1;
#endif
		}
#elif (CONFIG_BT_CTRL_LPCLK_SEL_MAIN_XTAL)
		LOG_WRN("Bluetooth will use main XTAL as Bluetooth sleep clock.");
#if !CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP
		s_lp_cntl.no_light_sleep = 1;
#endif
#elif (CONFIG_BT_CTRL_LPCLK_SEL_RTC_SLOW)
		// check whether or not EXT_CRYS is working
		if (rtc_clk_slow_freq_get() == SOC_RTC_SLOW_CLK_SRC_RC_SLOW) {
			s_lp_cntl.lpclk_sel = BTDM_LPCLK_SEL_RTC_SLOW; // Internal 150 kHz RC oscillator
			LOG_WRN("Internal 150kHz RC osciallator. The accuracy of this clock is a lot larger than 500ppm which is "
				 "required in Bluetooth communication, so don't select this option in scenarios such as BLE connection state.");
		} else {
			LOG_WRN("Internal 150kHz RC oscillator not detected.");
			assert(0);
		}
#endif

		bool select_src_ret __attribute__((unused));
		bool set_div_ret __attribute__((unused));
		if (s_lp_cntl.lpclk_sel == BTDM_LPCLK_SEL_XTAL) {
#ifdef CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP
			esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON));
			s_lp_cntl.main_xtal_pu = 1;
#endif
			select_src_ret = btdm_lpclk_select_src(BTDM_LPCLK_SEL_XTAL);
			set_div_ret = btdm_lpclk_set_div(rtc_clk_xtal_freq_get() / MHZ(1));
			assert(select_src_ret && set_div_ret);
			btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
			btdm_lpcycle_us = 1 << (btdm_lpcycle_us_frac);
		} else if (s_lp_cntl.lpclk_sel == BTDM_LPCLK_SEL_XTAL32K) {
			select_src_ret = btdm_lpclk_select_src(BTDM_LPCLK_SEL_XTAL32K);
			set_div_ret = btdm_lpclk_set_div(0);
			assert(select_src_ret && set_div_ret);
			btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
			btdm_lpcycle_us = (RTC_CLK_CAL_FRACT > 15) ? (1000000 << (RTC_CLK_CAL_FRACT - 15)) :
				(1000000 >> (15 - RTC_CLK_CAL_FRACT));
			assert(btdm_lpcycle_us != 0);
		} else if (s_lp_cntl.lpclk_sel == BTDM_LPCLK_SEL_RTC_SLOW) {
			select_src_ret = btdm_lpclk_select_src(BTDM_LPCLK_SEL_RTC_SLOW);
			set_div_ret = btdm_lpclk_set_div(0);
			assert(select_src_ret && set_div_ret);
			btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
			btdm_lpcycle_us = esp_clk_slowclk_cal_get();
		} else {
			err = ESP_ERR_INVALID_ARG;
			goto error;
		}
#if CONFIG_SW_COEXIST_ENABLE
		coex_update_lpclk_interval();
#endif
	} while (0);

#if CONFIG_SW_COEXIST_ENABLE
	coex_init();
#endif

	periph_module_enable(PERIPH_BT_MODULE);
	periph_module_reset(PERIPH_BT_MODULE);

	esp_phy_enable(PHY_MODEM_BT);
	s_lp_stat.phy_enabled = 1;

	if (btdm_controller_init(cfg) != 0) {
		err = ESP_ERR_NO_MEM;
		goto error;
	}
	coex_pti_v2();

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

	return ESP_OK;
}

static void bt_controller_deinit_internal(void)
{
	periph_module_disable(PERIPH_BT_MODULE);

	if (s_lp_stat.phy_enabled) {
		esp_phy_disable(PHY_MODEM_BT);
		s_lp_stat.phy_enabled = 0;
	}

	// deinit low power control resources
	do {

#if CONFIG_MAC_BB_PD
		if (s_lp_cntl.mac_bb_pd) {
			btdm_deep_sleep_mem_deinit();
			s_lp_cntl.mac_bb_pd = 0;
		}
#endif

		if (s_lp_cntl.wakeup_timer_required) {
			if (s_lp_stat.wakeup_timer_started) {
				esp_timer_stop(s_btdm_slp_tmr);
			}
			s_lp_stat.wakeup_timer_started = 0;
			esp_timer_delete(s_btdm_slp_tmr);
			s_btdm_slp_tmr = NULL;
		}

		if (s_lp_cntl.enable) {
			btdm_vnd_offload_task_deregister(BTDM_VND_OL_SIG_WAKEUP_TMR);
			if (s_wakeup_req_sem != NULL) {
				semphr_delete_wrapper(s_wakeup_req_sem);
				s_wakeup_req_sem = NULL;
			}
		}

		if (s_lp_cntl.lpclk_sel == BTDM_LPCLK_SEL_XTAL) {
#ifdef CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP
			if (s_lp_cntl.main_xtal_pu) {
				esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
				s_lp_cntl.main_xtal_pu = 0;
			}
#endif
			btdm_lpclk_select_src(BTDM_LPCLK_SEL_RTC_SLOW);
			btdm_lpclk_set_div(0);
#if CONFIG_SW_COEXIST_ENABLE
			coex_update_lpclk_interval();
#endif
		}

		btdm_lpcycle_us = 0;
	} while (0);

#if CONFIG_MAC_BB_PD
	esp_unregister_mac_bb_pd_callback(btdm_mac_bb_power_down_cb);
	esp_unregister_mac_bb_pu_callback(btdm_mac_bb_power_up_cb);
#endif

	esp_bt_power_domain_off();
#if CONFIG_MAC_BB_PD
	esp_mac_bb_pd_mem_deinit();
#endif
	esp_phy_modem_deinit();

	if (osi_funcs_p != NULL) {
		esp_bt_free(osi_funcs_p);
		osi_funcs_p = NULL;
	}

	btdm_controller_status = ESP_BT_CONTROLLER_STATUS_IDLE;
}

esp_err_t esp_bt_controller_enable(esp_bt_mode_t mode)
{
	int ret = ESP_OK;

	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_INITED) {
		return ESP_ERR_INVALID_STATE;
	}

	/* As the history reason, mode should be equal to the mode which set in esp_bt_controller_init() */
	if (mode != btdm_controller_get_mode()) {
		return ESP_ERR_INVALID_ARG;
	}

#if CONFIG_SW_COEXIST_ENABLE
	coex_enable();
#endif

	// enable low power mode
	do {
		if (s_lp_cntl.enable) {
			btdm_controller_enable_sleep(true);
		}
	} while (0);

	if (btdm_controller_enable(mode) != 0) {
		ret = ESP_ERR_INVALID_STATE;
		goto error;
	}

	btdm_controller_status = ESP_BT_CONTROLLER_STATUS_ENABLED;

	return ret;

error:
	// disable low power mode
	do {
		btdm_controller_enable_sleep(false);
	} while (0);

#if CONFIG_SW_COEXIST_ENABLE
	coex_disable();
#endif

	return ret;
}

esp_err_t esp_bt_controller_disable(void)
{
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return ESP_ERR_INVALID_STATE;
	}

	async_wakeup_request(BTDM_ASYNC_WAKEUP_SRC_DISA);
	while (!btdm_power_state_active()){}
	btdm_controller_disable();

	async_wakeup_request_end(BTDM_ASYNC_WAKEUP_SRC_DISA);

#if CONFIG_SW_COEXIST_ENABLE
	coex_disable();
#endif

	btdm_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

	return ESP_OK;
}

esp_bt_controller_status_t esp_bt_controller_get_status(void)
{
	return btdm_controller_status;
}

/* extra functions */
esp_err_t esp_ble_tx_power_set(esp_ble_power_type_t power_type, esp_power_level_t power_level)
{
	esp_err_t stat = ESP_FAIL;

	switch (power_type) {
	case ESP_BLE_PWR_TYPE_ADV:
	case ESP_BLE_PWR_TYPE_SCAN:
	case ESP_BLE_PWR_TYPE_DEFAULT:
		if (ble_txpwr_set(power_type, power_level) == 0) {
			stat = ESP_OK;
		}
		break;
	default:
		stat = ESP_ERR_NOT_SUPPORTED;
		break;
	}

	return stat;
}

esp_power_level_t esp_ble_tx_power_get(esp_ble_power_type_t power_type)
{
	esp_power_level_t lvl;

	switch (power_type) {
	case ESP_BLE_PWR_TYPE_ADV:
	case ESP_BLE_PWR_TYPE_SCAN:
		lvl = (esp_power_level_t)ble_txpwr_get(power_type);
		break;
	case ESP_BLE_PWR_TYPE_CONN_HDL0:
	case ESP_BLE_PWR_TYPE_CONN_HDL1:
	case ESP_BLE_PWR_TYPE_CONN_HDL2:
	case ESP_BLE_PWR_TYPE_CONN_HDL3:
	case ESP_BLE_PWR_TYPE_CONN_HDL4:
	case ESP_BLE_PWR_TYPE_CONN_HDL5:
	case ESP_BLE_PWR_TYPE_CONN_HDL6:
	case ESP_BLE_PWR_TYPE_CONN_HDL7:
	case ESP_BLE_PWR_TYPE_CONN_HDL8:
	case ESP_BLE_PWR_TYPE_DEFAULT:
		lvl = (esp_power_level_t)ble_txpwr_get(ESP_BLE_PWR_TYPE_DEFAULT);
		break;
	default:
		lvl = ESP_PWR_LVL_INVALID;
		break;
	}

	return lvl;
}

esp_err_t esp_bt_sleep_enable (void)
{
	esp_err_t status;
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return ESP_ERR_INVALID_STATE;
	}
	if (btdm_controller_get_sleep_mode() == ESP_BT_SLEEP_MODE_1) {
		btdm_controller_enable_sleep (true);
		status = ESP_OK;
	} else {
		status = ESP_ERR_NOT_SUPPORTED;
	}

	return status;
}

esp_err_t esp_bt_sleep_disable (void)
{
	esp_err_t status;
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
		return ESP_ERR_INVALID_STATE;
	}
	if (btdm_controller_get_sleep_mode() == ESP_BT_SLEEP_MODE_1) {
		btdm_controller_enable_sleep (false);
		status = ESP_OK;
	} else {
		status = ESP_ERR_NOT_SUPPORTED;
	}

	return status;
}

bool esp_bt_controller_is_sleeping(void)
{
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED ||
			btdm_controller_get_sleep_mode() != ESP_BT_SLEEP_MODE_1) {
			return false;
	}

	return !btdm_power_state_active();
}

void esp_bt_controller_wakeup_request(void)
{
	if (btdm_controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED ||
			btdm_controller_get_sleep_mode() != ESP_BT_SLEEP_MODE_1) {
			return;
	}

	btdm_wakeup_request();
}

int IRAM_ATTR esp_bt_h4tl_eif_io_event_notify(int event)
{
	return btdm_hci_tl_io_event_post(event);
}

uint16_t esp_bt_get_tx_buf_num(void)
{
	return l2c_ble_link_get_tx_buf_num();
}

static void coex_wifi_sleep_set_hook(bool sleep)
{

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