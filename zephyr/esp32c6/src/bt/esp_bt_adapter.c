/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
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
#include "bt_osi_mem.h"
#include "esp_clk_tree.h"
#include "esp_private/esp_clk.h"
#include "esp_private/esp_clk_tree_common.h"
#include "soc/soc_caps.h"
#include "soc/rtc.h"
#include "soc/soc_memory_layout.h"
#include "private/esp_coexist_internal.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_rom_sys.h"
#include "esp_private/phy.h"
#include "hal/efuse_hal.h"
#include "soc/rtc.h"
#include "esp_heap_adapter.h"
#include "nimble/nimble_port_zephyr.h"
#include "esp_private/esp_modem_clock.h"
#include "nimble/nimble_npl_os.h"
#include "esp_hci_transport.h"
#include "os/endian.h"
#include "ble_priv.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/random/random.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(esp32_bt_adapter, CONFIG_LOG_DEFAULT_LEVEL);

#define OSI_COEX_VERSION     0x00010006
#define OSI_COEX_MAGIC_VALUE 0xFADEBEAD

#define EXT_FUNC_VERSION     0x20250415
#define EXT_FUNC_MAGIC_VALUE 0xA5A5A5A5

#define BT_ASSERT_PRINT ets_printf

struct osi_coex_funcs_t {
	uint32_t _magic;
	uint32_t _version;
	void (*_coex_wifi_sleep_set)(bool sleep);
	int (*_coex_core_ble_conn_dyn_prio_get)(bool *low, bool *high);
	void (*_coex_schm_status_bit_set)(uint32_t type, uint32_t status);
	void (*_coex_schm_status_bit_clear)(uint32_t type, uint32_t status);
};

struct ext_funcs_t {
	uint32_t ext_version;
	int (*_esp_intr_alloc)(int source, int flags, intr_handler_t handler, void *arg,
			       void **ret_handle);
	int (*_esp_intr_free)(void **ret_handle);
	void *(*_malloc)(size_t size);
	void (*_free)(void *p);
	int (*_task_create)(void *task_func, const char *name, uint32_t stack_depth, void *param,
			    uint32_t prio, void *task_handle, uint32_t core_id);
	void (*_task_delete)(void *task_handle);
	void (*_osi_assert)(const uint32_t ln, const char *fn, uint32_t param1, uint32_t param2);
	uint32_t (*_os_random)(void);
	int (*_ecc_gen_key_pair)(uint8_t *public, uint8_t *priv);
	int (*_ecc_gen_dh_key)(const uint8_t *remote_pub_key_x, const uint8_t *remote_pub_key_y,
			       const uint8_t *local_priv_key, uint8_t *dhkey);
#if CONFIG_IDF_TARGET_ESP32C6
	void (*_esp_reset_modem)(uint8_t mdl_opts, uint8_t start);
#endif /* CONFIG_IDF_TARGET_ESP32C6 */
	uint32_t magic;
};

/* External functions or variables
 ************************************************************************
 */
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
extern void coex_hw_timer_set(uint8_t idx,uint8_t src, uint8_t pti,uint32_t latency, uint32_t perioidc);
extern void coex_hw_timer_enable(uint8_t idx);
extern void coex_hw_timer_disable(uint8_t idx);
#endif /* CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE */
extern int ble_osi_coex_funcs_register(struct osi_coex_funcs_t *coex_funcs);
extern int r_ble_controller_init(esp_bt_controller_config_t *cfg);
extern void esp_ble_controller_info_capture(uint32_t cycle_times);
extern int r_ble_controller_deinit(void);
extern int r_ble_controller_enable(uint8_t mode);
extern int r_ble_controller_disable(void);
extern int esp_register_ext_funcs (struct ext_funcs_t *);
extern void esp_unregister_ext_funcs (void);
extern int r_esp_ble_ll_set_public_addr(const uint8_t *addr);
extern int esp_register_npl_funcs (struct npl_funcs_t *p_npl_func);
extern void esp_unregister_npl_funcs (void);
extern void npl_zephyr_mempool_deinit(void);
extern uint32_t r_os_cputime_get32(void);
extern uint32_t r_os_cputime_ticks_to_usecs(uint32_t ticks);
extern void r_ble_lll_rfmgmt_set_sleep_cb(void *s_cb, void *w_cb, void *s_arg,
                                          void *w_arg, uint32_t us_to_enabled);
extern void r_ble_rtc_wake_up_state_clr(void);
extern int os_msys_init(void);
extern void os_msys_deinit(void);
#if CONFIG_ESP32_BT_LE_LL_PEER_SCA_SET_ENABLE
extern void r_ble_ll_customize_peer_sca_set(uint16_t peer_sca);
#endif  /* CONFIG_ESP32_BT_LE_LL_PEER_SCA_SET_ENABLE */
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
#if CONFIG_IDF_TARGET_ESP32C6
static void esp_reset_modem(uint8_t mdl_opts,uint8_t start);
#endif /* CONFIG_IDF_TARGET_ESP32C6 */
static int esp_ecc_gen_key_pair(uint8_t *pub, uint8_t *priv);
static int esp_ecc_gen_dh_key(const uint8_t *peer_pub_key_x, const uint8_t *peer_pub_key_y,
                              const uint8_t *our_priv_key, uint8_t *out_dhkey);

/* Static variable declare */
static DRAM_ATTR esp_bt_controller_status_t ble_controller_status = ESP_BT_CONTROLLER_STATUS_IDLE;

/* This variable tells if BLE is running */
static bool s_ble_active = false;
#define MAIN_XTAL_FREQ_HZ                 (40000000)
#define MAIN_XTAL_FREQ_HZ_WORKROUND       (500000)
static DRAM_ATTR modem_clock_lpclk_src_t s_bt_lpclk_src = MODEM_CLOCK_LPCLK_SRC_INVALID;
static DRAM_ATTR uint32_t s_bt_lpclk_freq = 100000;

#define BLE_RTC_DELAY_US_LIGHT_SLEEP        (3200)
#define BLE_RTC_DELAY_US_MODEM_SLEEP        (500)

static struct k_thread bt_task_handle;

K_THREAD_STACK_DEFINE(bt_stack, CONFIG_ESP32_BT_CONTROLLER_STACK_SIZE);

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
	._malloc = bt_osi_mem_malloc_internal,
	._free = bt_osi_mem_free,
	._task_create = task_create_wrapper,
	._task_delete = task_delete_wrapper,
	._osi_assert = osi_assert_wrapper,
	._os_random = osi_random_wrapper,
	._ecc_gen_key_pair = esp_ecc_gen_key_pair,
	._ecc_gen_dh_key = esp_ecc_gen_dh_key,
#if CONFIG_IDF_TARGET_ESP32C6
	._esp_reset_modem = esp_reset_modem,
#endif /* CONFIG_IDF_TARGET_ESP32C6 */
	.magic = EXT_FUNC_MAGIC_VALUE,
};

#if CONFIG_IDF_TARGET_ESP32C6
static void IRAM_ATTR esp_reset_modem(uint8_t mdl_opts, uint8_t start)
{
	if (mdl_opts == 0x05) {
		if (start) {
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
			coex_hw_timer_set(0x04, 0x02, 15, 0, 5000);
			coex_hw_timer_enable(0x04);
#endif /* CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE */
			MODEM_SYSCON.modem_rst_conf.val |= (BIT(16) | BIT(18));
			MODEM_SYSCON.modem_rst_conf.val &= ~(BIT(16) | BIT(18));
		} else {
#if CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE
			coex_hw_timer_disable(0x04);
#endif /* CONFIG_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE */
		}
	}
}

#endif /* CONFIG_IDF_TARGET_ESP32C6 */

static void IRAM_ATTR osi_assert_wrapper(const uint32_t ln, const char *fn, uint32_t param1,
					 uint32_t param2)
{
	BT_ASSERT_PRINT("BLE assert: line %d in function %s, param: 0x%x, 0x%x", ln, fn, param1,
			param2);
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
#endif /* CONFIG_SW_COEXIST_ENABLE */
}

static void coex_schm_status_bit_clear_wrapper(uint32_t type, uint32_t status)
{
#if CONFIG_SW_COEXIST_ENABLE
	coex_schm_status_bit_clear(type, status);
#endif /* CONFIG_SW_COEXIST_ENABLE */
}

static int task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param,
			       uint32_t prio, void *task_handle, uint32_t core_id)
{
	k_tid_t tid =
		k_thread_create(&bt_task_handle, bt_stack, stack_depth, (k_thread_entry_t)task_func,
				param, NULL, NULL, K_PRIO_COOP(prio), K_INHERIT_PERMS, K_NO_WAIT);

	k_thread_name_set(tid, name);

	*(int32_t *)task_handle = (int32_t)tid;

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
#if CONFIG_ESP32_BT_LE_SM_LEGACY || CONFIG_ESP32_BT_LE_SM_SC
	rc = ble_sm_alg_gen_key_pair(pub, priv);
#endif /* CONFIG_ESP32_BT_LE_SM_LEGACY || CONFIG_ESP32_BT_LE_SM_SC */
	return rc;
}

static int esp_ecc_gen_dh_key(const uint8_t *peer_pub_key_x, const uint8_t *peer_pub_key_y,
			      const uint8_t *our_priv_key, uint8_t *out_dhkey)
{
	int rc = -1;
#if CONFIG_ESP32_BT_LE_SM_LEGACY || CONFIG_ESP32_BT_LE_SM_SC
	rc = ble_sm_alg_gen_dhkey(peer_pub_key_x, peer_pub_key_y, our_priv_key, out_dhkey);
#endif /* CONFIG_ESP32_BT_LE_SM_LEGACY || CONFIG_ESP32_BT_LE_SM_SC */
	return rc;
}

static int esp_intr_alloc_wrapper(int source, int flags, intr_handler_t handler, void *arg,
				  void **ret_handle_in)
{
	int rc = esp_intr_alloc(source, flags | ESP_INTR_FLAG_IRAM, (void *)handler, arg,
							(intr_handle_t *)ret_handle_in);
	return rc;
}

static int esp_intr_free_wrapper(void **ret_handle)
{
	int rc = 0;

	rc = esp_intr_free((intr_handle_t)*ret_handle);
	*ret_handle = NULL;

	return rc;
}

void esp_bt_rtc_slow_clk_select(uint8_t slow_clk_src)
{
	/* Select slow clock source for BT module */
	switch (slow_clk_src) {
	case MODEM_CLOCK_LPCLK_SRC_MAIN_XTAL:
#if SOC_BLE_USE_WIFI_PWR_CLK_WORKAROUND
		uint32_t chip_version = efuse_hal_chip_revision();
		LOG_INF("Using main XTAL as clock source, chip ver: %d", chip_version);
		if (chip_version == 0) {
			modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (MAIN_XTAL_FREQ_HZ/s_bt_lpclk_freq - 1));
		} else{
			modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (MAIN_XTAL_FREQ_HZ_WORKROUND/s_bt_lpclk_freq - 1));
		}
#else
		LOG_INF("Using main XTAL as clock source");
		modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (MAIN_XTAL_FREQ_HZ/s_bt_lpclk_freq - 1));
#endif /* SOC_BLE_USE_WIFI_PWR_CLK_WORKAROUND */
		break;
	case MODEM_CLOCK_LPCLK_SRC_RC_SLOW:
		LOG_INF("Using 136 kHz RC as clock source, can only run legacy ADV or SCAN due to "
			"low clock accuracy!");
		modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (5 - 1));
		break;
	case MODEM_CLOCK_LPCLK_SRC_XTAL32K:
		LOG_INF("Using external 32.768 kHz XTAL as clock source");
		modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (1 - 1));
		break;
	case MODEM_CLOCK_LPCLK_SRC_RC32K:
		LOG_INF("Using 32 kHz RC as clock source, can only run legacy ADV or SCAN due to "
			"low clock accuracy!");
		modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (1 - 1));
		break;
	case MODEM_CLOCK_LPCLK_SRC_EXT32K:
		LOG_INF("Using 32 kHz oscillator as clock source, can only run legacy ADV or SCAN "
			"due to low clock accuracy!");
		modem_clock_select_lp_clock_source(PERIPH_BT_MODULE, slow_clk_src, (1 - 1));
		break;
	default:
	}
}

modem_clock_lpclk_src_t esp_bt_get_lpclk_src(void)
{
	return s_bt_lpclk_src;
}

void esp_bt_set_lpclk_src(modem_clock_lpclk_src_t clk_src)
{
	if (ble_controller_status != ESP_BT_CONTROLLER_STATUS_IDLE) {
		return;
	}

	if (clk_src >= MODEM_CLOCK_LPCLK_SRC_MAX) {
		return;
	}

	s_bt_lpclk_src = clk_src;
}

uint32_t esp_bt_get_lpclk_freq(void)
{
	return s_bt_lpclk_freq;
}

void esp_bt_set_lpclk_freq(uint32_t clk_freq)
{
	uint32_t xtal_freq;

	if (ble_controller_status != ESP_BT_CONTROLLER_STATUS_IDLE) {
		return;
	}

	if (!clk_freq) {
		return;
	}

	xtal_freq = MAIN_XTAL_FREQ_HZ;
#if SOC_BLE_USE_WIFI_PWR_CLK_WORKAROUND
	uint32_t chip_version = efuse_hal_chip_revision();
	if (chip_version == 1) {
		xtal_freq = MAIN_XTAL_FREQ_HZ_WORKROUND;
	}
#endif /* SOC_BLE_USE_WIFI_PWR_CLK_WORKAROUND */

	if (xtal_freq % clk_freq) {
		return;
	}

	s_bt_lpclk_freq = clk_freq;
}

IRAM_ATTR void controller_sleep_cb(uint32_t enable_tick, void *arg)
{
	if (!s_ble_active) {
		return;
	}
	esp_phy_disable(PHY_MODEM_BT);

	s_ble_active = false;
}

IRAM_ATTR void controller_wakeup_cb(void *arg)
{
	if (s_ble_active) {
		return;
	}

	esp_phy_enable(PHY_MODEM_BT);
	if (s_bt_lpclk_src == MODEM_CLOCK_LPCLK_SRC_RC_SLOW) {
		uint32_t *clk_freq = (uint32_t *)arg;
		*clk_freq = esp_clk_tree_lp_slow_get_freq_hz(ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED) /	5;
	}
	s_ble_active = true;
}

esp_err_t controller_sleep_init(void)
{
	esp_err_t rc = 0;

#ifdef CONFIG_ESP32_BT_LE_SLEEP_ENABLE
	LOG_WRN("BLE modem sleep is enabled");
	r_ble_lll_rfmgmt_set_sleep_cb(controller_sleep_cb, controller_wakeup_cb, 0, 0,
				      BLE_RTC_DELAY_US_MODEM_SLEEP);
#endif /* CONFIG_ESP32_BT_LE_SLEEP_ENABLE */

	return rc;
}

void controller_sleep_deinit(void)
{

}

typedef enum {
	FILTER_DUPLICATE_PDUTYPE = BIT(0),
	FILTER_DUPLICATE_LENGTH = BIT(1),
	FILTER_DUPLICATE_ADDRESS = BIT(2),
	FILTER_DUPLICATE_ADVDATA = BIT(3),
	FILTER_DUPLICATE_DEFAULT = FILTER_DUPLICATE_PDUTYPE | FILTER_DUPLICATE_ADDRESS,
	FILTER_DUPLICATE_PDU_ALL = 0xF,
	FILTER_DUPLICATE_EXCEPTION_FOR_MESH = BIT(4),
	FILTER_DUPLICATE_AD_TYPE = BIT(5),
} disc_duplicate_mode_t;

extern void r_filter_duplicate_mode_enable(disc_duplicate_mode_t mode);
extern void r_filter_duplicate_mode_disable(disc_duplicate_mode_t mode);
extern void r_filter_duplicate_set_ring_list_max_num(uint32_t max_num);
extern void r_scan_duplicate_cache_refresh_set_time(uint32_t period_time);

int ble_vhci_disc_duplicate_mode_enable(int mode)
{
	/* TODO: use vendor hci to update */
	r_filter_duplicate_mode_enable(mode);
	return true;
}

int ble_vhci_disc_duplicate_mode_disable(int mode)
{
	/* TODO: use vendor hci to update */
	r_filter_duplicate_mode_disable(mode);
	return true;
}

int ble_vhci_disc_duplicate_set_max_cache_size(int max_cache_size)
{
	/* TODO: use vendor hci to update */
	r_filter_duplicate_set_ring_list_max_num(max_cache_size);
	return true;
}

int ble_vhci_disc_duplicate_set_period_refresh_time(int refresh_period_time)
{
	/* TODO: use vendor hci to update */
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
#if CONFIG_ESP32_BT_LE_SCAN_DUPL == true
	cache_size = CONFIG_ESP32_BT_LE_LL_DUP_SCAN_LIST_COUNT;
	if (CONFIG_ESP32_BT_LE_SCAN_DUPL_TYPE == 0) {
		duplicate_mode = FILTER_DUPLICATE_ADDRESS | FILTER_DUPLICATE_PDUTYPE;
	} else if (CONFIG_ESP32_BT_LE_SCAN_DUPL_TYPE == 1) {
		duplicate_mode = FILTER_DUPLICATE_ADVDATA;
	} else if (CONFIG_ESP32_BT_LE_SCAN_DUPL_TYPE == 2) {
		duplicate_mode = FILTER_DUPLICATE_ADDRESS | FILTER_DUPLICATE_ADVDATA;
	}
	duplicate_mode |= FILTER_DUPLICATE_EXCEPTION_FOR_MESH;

	ble_vhci_disc_duplicate_set_period_refresh_time(CONFIG_ESP32_BT_LE_SCAN_DUPL_CACHE_REFRESH_PERIOD);
#endif

	ble_vhci_disc_duplicate_mode_disable(0xFFFFFFFF);
	ble_vhci_disc_duplicate_mode_enable(duplicate_mode);
	ble_vhci_disc_duplicate_set_max_cache_size(cache_size);
}

static void ble_rtc_clk_init(esp_bt_controller_config_t *cfg)
{
	if (s_bt_lpclk_src == MODEM_CLOCK_LPCLK_SRC_INVALID) {
#if CONFIG_ESP32_BT_LE_LP_CLK_SRC_MAIN_XTAL
		s_bt_lpclk_src = MODEM_CLOCK_LPCLK_SRC_MAIN_XTAL;
#else
#if CONFIG_RTC_CLK_SRC_INT_RC
		s_bt_lpclk_src = MODEM_CLOCK_LPCLK_SRC_RC_SLOW;
#elif CONFIG_RTC_CLK_SRC_EXT_CRYS
		if (rtc_clk_slow_src_get() == SOC_RTC_SLOW_CLK_SRC_XTAL32K) {
			s_bt_lpclk_src = MODEM_CLOCK_LPCLK_SRC_XTAL32K;
		} else {
			LOG_WRN("32.768kHz XTAL not detected, fall back to "
				"main XTAL as Bluetooth sleep clock");
			s_bt_lpclk_src = MODEM_CLOCK_LPCLK_SRC_MAIN_XTAL;
		}
#elif CONFIG_RTC_CLK_SRC_INT_RC32K
		s_bt_lpclk_src = MODEM_CLOCK_LPCLK_SRC_RC32K;
#elif CONFIG_RTC_CLK_SRC_EXT_OSC
		s_bt_lpclk_src = MODEM_CLOCK_LPCLK_SRC_EXT32K;
#else
		LOG_ERR("Unsupported clock source");
		assert(0);
#endif
#endif /* CONFIG_ESP32_BT_LE_LP_CLK_SRC_MAIN_XTAL */
	}

	if (s_bt_lpclk_src == MODEM_CLOCK_LPCLK_SRC_MAIN_XTAL) {
		cfg->rtc_freq = s_bt_lpclk_freq;
	} else if (s_bt_lpclk_src == MODEM_CLOCK_LPCLK_SRC_XTAL32K) {
		cfg->rtc_freq = 32768;
	} else if (s_bt_lpclk_src == MODEM_CLOCK_LPCLK_SRC_RC_SLOW) {
		cfg->rtc_freq =	esp_clk_tree_lp_slow_get_freq_hz(ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED) / 5;
		cfg->ble_ll_sca = 3000;
	} else if (s_bt_lpclk_src == MODEM_CLOCK_LPCLK_SRC_RC32K) {
		cfg->rtc_freq = 32000;
	} else if (s_bt_lpclk_src == MODEM_CLOCK_LPCLK_SRC_EXT32K) {
		cfg->rtc_freq = 32000;
	}
	esp_bt_rtc_slow_clk_select(s_bt_lpclk_src);
}

esp_err_t esp_bt_controller_init(esp_bt_controller_config_t *cfg)
{
	uint8_t mac[6];
	esp_err_t ret = ESP_OK;
	ble_npl_count_info_t npl_info;
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
	npl_zephyr_funcs_init();
	struct npl_funcs_t *p_npl_funcs = npl_zephyr_funcs_get();
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
	npl_zephyr_set_controller_npl_info(&npl_info);
	if (npl_zephyr_mempool_init() != 0) {
		LOG_WRN("npl mempool init failed");
		ret = ESP_ERR_INVALID_ARG;
		goto free_mem;
	}

	/* Enable BT-related clocks */
	modem_clock_module_enable(PERIPH_BT_MODULE);
	modem_clock_module_mac_reset(PERIPH_BT_MODULE);
	/* Select slow clock source for BT module */
	ble_rtc_clk_init(cfg);
	esp_phy_modem_init();

	if (ble_osi_coex_funcs_register((struct osi_coex_funcs_t *)&s_osi_coex_funcs_ro) != 0) {
		LOG_WRN("osi coex funcs reg failed");
		ret = ESP_ERR_INVALID_ARG;
		goto modem_deint;
	}

#if CONFIG_SW_COEXIST_ENABLE
	coex_init();
#endif /* CONFIG_SW_COEXIST_ENABLE */

	ret = esp_ble_register_bb_funcs();
	if (ret != ESP_OK) {
		LOG_WRN("esp_ble_register_bb_funcs failed %d", ret);
		goto modem_deint;
	}

	LOG_INF("ble controller commit:[%s]", ble_controller_get_compile_version());

	ret = r_ble_controller_init(cfg);
	if (ret != ESP_OK) {
		LOG_WRN("r_ble_controller_init failed %d", ret);
		goto modem_deint;
	}

#if CONFIG_ESP32_BT_LE_LL_PEER_SCA_SET_ENABLE
	r_ble_ll_customize_peer_sca_set(CONFIG_ESP32_BT_LE_LL_PEER_SCA);
#endif

	ret = ble_stack_initEnv();
	if (ret != ESP_OK) {
		LOG_WRN("ble_stack_initEnv failed %d", ret);
		goto free_controller;
	}

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

	esp_read_mac((uint8_t *)mac, ESP_MAC_BT);
	LOG_INF("Bluetooth MAC: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3],
		mac[4], mac[5]);
	swap_in_place(mac, 6);
	r_esp_ble_ll_set_public_addr(mac);

	ble_controller_status = ESP_BT_CONTROLLER_STATUS_INITED;

#if CONFIG_ESP32_BT_LE_HCI_INTERFACE_USE_RAM
	hci_transport_mode = HCI_TRANSPORT_VHCI;
#endif /* CONFIG_ESP32_BT_LE_HCI_INTERFACE_USE_RAM */
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
	ble_stack_deinitEnv();
	r_ble_controller_deinit();
modem_deint:
	esp_ble_unregister_bb_funcs();
	esp_phy_modem_deinit();
	modem_clock_deselect_lp_clock_source(PERIPH_BT_MODULE);
	modem_clock_module_disable(PERIPH_BT_MODULE);
free_mem:
	npl_zephyr_mempool_deinit();
	esp_unregister_npl_funcs();
	npl_zephyr_funcs_deinit();
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

	ble_stack_deinitEnv();
	r_ble_controller_deinit();
	esp_ble_unregister_bb_funcs();

	esp_unregister_npl_funcs();

	esp_unregister_ext_funcs();

	/* De-initialize npl functions */
	npl_zephyr_funcs_deinit();

	npl_zephyr_mempool_deinit();

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
		esp_phy_enable(PHY_MODEM_BT);
		s_ble_active = true;
	}
	esp_btbb_enable();
#if CONFIG_SW_COEXIST_ENABLE
	coex_enable();
#endif /* CONFIG_SW_COEXIST_ENABLE */

	if (ble_stack_enable() != 0) {
		ret = ESP_FAIL;
		goto error;
	}

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
		if (r_ble_txpwr_set(ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT, 0, power_level) == 0) {
			stat = ESP_OK;
		}
		break;
	case ESP_BLE_PWR_TYPE_ADV:
		if (r_ble_txpwr_set(ESP_BLE_ENHANCED_PWR_TYPE_ADV, 0xFF, power_level) == 0) {
			stat = ESP_OK;
		}
		break;
	case ESP_BLE_PWR_TYPE_SCAN:
		if (r_ble_txpwr_set(ESP_BLE_ENHANCED_PWR_TYPE_SCAN, 0, power_level) == 0) {
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
		if (r_ble_txpwr_set(ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT, 0, power_level) == 0) {
			stat = ESP_OK;
		}
		break;
	case ESP_BLE_ENHANCED_PWR_TYPE_SCAN:
	case ESP_BLE_ENHANCED_PWR_TYPE_INIT:
		if (r_ble_txpwr_set(ESP_BLE_ENHANCED_PWR_TYPE_SCAN, 0, power_level) == 0) {
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
	case ESP_BLE_PWR_TYPE_DEFAULT:
		tx_level = r_ble_txpwr_get(ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT, 0);
		break;
	case ESP_BLE_PWR_TYPE_ADV:
		tx_level = r_ble_txpwr_get(ESP_BLE_ENHANCED_PWR_TYPE_ADV, 0);
		break;
	case ESP_BLE_PWR_TYPE_SCAN:
		tx_level = r_ble_txpwr_get(ESP_BLE_ENHANCED_PWR_TYPE_SCAN, 0);
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
		tx_level = r_ble_txpwr_get(ESP_BLE_ENHANCED_PWR_TYPE_DEFAULT, 0);
		break;
	case ESP_BLE_ENHANCED_PWR_TYPE_SCAN:
	case ESP_BLE_ENHANCED_PWR_TYPE_INIT:
		tx_level = r_ble_txpwr_get(ESP_BLE_ENHANCED_PWR_TYPE_SCAN, 0);
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

#if CONFIG_ESP32_BT_LE_SM_LEGACY || CONFIG_ESP32_BT_LE_SM_SC
#define BLE_SM_KEY_ERR 0x17
#if CONFIG_ESP32_BT_LE_CRYPTO_STACK_MBEDTLS
#include "mbedtls/aes.h"
#if CONFIG_ESP32_BT_LE_SM_SC
#include "mbedtls/cipher.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/cmac.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"

static mbedtls_ecp_keypair keypair;
#endif /* CONFIG_ESP32_BT_LE_SM_SC */

#endif /* CONFIG_ESP32_BT_LE_CRYPTO_STACK_MBEDTLS */

/* Based on Core Specification 4.2 Vol 3. Part H 2.3.5.6.1 */
static const uint8_t ble_sm_alg_dbg_priv_key[32] = {0x3f, 0x49, 0xf6, 0xd4, 0xa3, 0xc5, 0x5f, 0x38,
						    0x74, 0xc9, 0xb3, 0xe3, 0xd2, 0x10, 0x3f, 0x50,
						    0x4a, 0xff, 0x60, 0x7b, 0xeb, 0x40, 0xb7, 0x99,
						    0x58, 0x99, 0xb8, 0xa6, 0xcd, 0x3c, 0x1a, 0xbd};

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

#if CONFIG_ESP32_BT_LE_CRYPTO_STACK_MBEDTLS
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

#endif /* CONFIG_ESP32_BT_LE_CRYPTO_STACK_MBEDTLS */

	swap_buf(out_dhkey, dh, 32);
	return 0;
}

#if CONFIG_ESP32_BT_LE_CRYPTO_STACK_MBEDTLS
static int mbedtls_gen_keypair(uint8_t *public_key, uint8_t *private_key)
{
	int rc = BLE_SM_KEY_ERR;
	mbedtls_entropy_context entropy = {0};
	mbedtls_ctr_drbg_context ctr_drbg = {0};

	mbedtls_entropy_init(&entropy);
	mbedtls_ctr_drbg_init(&ctr_drbg);
	mbedtls_ecp_keypair_init(&keypair);

	if ((rc = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy, NULL, 0)) != 0) {
		goto exit;
	}

	if ((rc = mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, &keypair, mbedtls_ctr_drbg_random,
				      &ctr_drbg)) != 0) {
		goto exit;
	}

	if ((rc = mbedtls_mpi_write_binary(&keypair.MBEDTLS_PRIVATE(d), private_key, 32)) != 0) {
		goto exit;
	}

	size_t olen = 0;
	uint8_t pub[65] = {0};

	if ((rc = mbedtls_ecp_point_write_binary(
		     &keypair.MBEDTLS_PRIVATE(grp), &keypair.MBEDTLS_PRIVATE(Q),
		     MBEDTLS_ECP_PF_UNCOMPRESSED, &olen, pub, 65)) != 0) {
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
#endif /* CONFIG_ESP32_BT_LE_CRYPTO_STACK_MBEDTLS */

/**
 * pub: 64 bytes
 * priv: 32 bytes
 */
int ble_sm_alg_gen_key_pair(uint8_t *pub, uint8_t *priv)
{
#if CONFIG_ESP32_BT_LE_SM_SC_DEBUG_KEYS
	swap_buf(pub, ble_sm_alg_dbg_pub_key, 32);
	swap_buf(&pub[32], &ble_sm_alg_dbg_pub_key[32], 32);
	swap_buf(priv, ble_sm_alg_dbg_priv_key, 32);
#else
	uint8_t pk[64];

	do {
#if CONFIG_ESP32_BT_LE_CRYPTO_STACK_MBEDTLS
		if (mbedtls_gen_keypair(pk, priv) != 0) {
			return BLE_SM_KEY_ERR;
		}
#endif /* CONFIG_ESP32_BT_LE_CRYPTO_STACK_MBEDTLS */
		/* Make sure generated key isn't debug key. */
	} while (memcmp(priv, ble_sm_alg_dbg_priv_key, 32) == 0);

	swap_buf(pub, pk, 32);
	swap_buf(&pub[32], &pk[32], 32);
	swap_in_place(priv, 32);
#endif /* CONFIG_ESP32_BT_LE_SM_SC_DEBUG_KEYS */
	return 0;
}

#endif /* CONFIG_ESP32_BT_LE_SM_LEGACY || CONFIG_ESP32_BT_LE_SM_SC */

int IRAM_ATTR ble_capture_info_user_handler(uint8_t type, uint32_t reason, uint32_t param1,
					    uint32_t param2)
{
	int i;

	switch (type) {
	case 0:
		for (i = 0; i < 2; i++) {
			esp_ble_controller_info_capture(0x010101);
		}
#if CONFIG_ESP32_BT_LE_DEBUG_REMAIN_SCENE_ENABLED
		uintptr_t sp;
		__asm__ volatile("mv %0, sp" : "=r"(sp));
		esp_gdbstub_panic_handler(&sp);
#endif /* CONFIG_ESP32_BT_LE_DEBUG_REMAIN_SCENE_ENABLED */
		break;
#if CONFIG_ESP32_BT_LE_ASSERT_WHEN_ABNORMAL_DISCONN_ENABLED
	case 1:
		if ((reason == 0x08) || (reason == 0x3d) || (reason == 0x28)) {
			osi_assert_wrapper(__LINE__, __func__, type, reason);
		}
		break;
#endif /* CONFIG_ESP32_BT_LE_ASSERT_WHEN_ABNORMAL_DISCONN_ENABLED */
	default:
		break;
	}
	return 0;
}
