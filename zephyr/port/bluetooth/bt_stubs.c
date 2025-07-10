#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_bt.h"

int btdm_osi_funcs_register(void *osi_funcs)
{
	(void)osi_funcs;
	return ESP_OK;
}

int btdm_controller_init(uint32_t config_mask, esp_bt_controller_config_t *config_opts)
{
	(void)config_mask;
	(void)config_opts;
	return ESP_OK;
}

void btdm_controller_deinit(void)
{
}

int btdm_controller_enable(esp_bt_mode_t mode)
{
	(void)mode;
	return ESP_OK;
}

void btdm_controller_disable(void)
{
}

uint8_t btdm_controller_get_mode(void)
{
	return 0;
}

const char *btdm_controller_get_compile_version(void)
{
	return "dummy_version";
}

void btdm_rf_bb_init_phase2(void)
{
}

#ifdef CONFIG_SOC_SERIES_ESP32
int btdm_dispatch_work_to_controller(workitem_handler_t callback, void *arg, bool blocking)
{
	(void)callback;
	(void)arg;
	(void)blocking;
	return ESP_OK;
}
#endif

void btdm_controller_enable_sleep(bool enable)
{
	(void)enable;
}

void btdm_controller_set_sleep_mode(uint8_t mode)
{
	(void)mode;
}

uint8_t btdm_controller_get_sleep_mode(void)
{
	return 0;
}

bool btdm_power_state_active(void)
{
	return true;
}

void btdm_wakeup_request(void)
{
}

void btdm_in_wakeup_requesting_set(bool in_wakeup_requesting)
{
	(void)in_wakeup_requesting;
}

#if defined(CONFIG_SOC_SERIES_ESP32C3) || defined(CONFIG_SOC_SERIES_ESP32S3)
int btdm_vnd_offload_task_register(btdm_vnd_ol_sig_t sig, btdm_vnd_ol_task_func_t func)
{
	(void)sig;
	(void)func;
	return ESP_OK;
}

int btdm_vnd_offload_task_deregister(btdm_vnd_ol_sig_t sig)
{
	(void)sig;
	return ESP_OK;
}

int r_btdm_vnd_offload_post(btdm_vnd_ol_sig_t sig, void *param)
{
	(void)sig;
	(void)param;
	return ESP_OK;
}
#endif

uint8_t btdm_sleep_clock_sync(void)
{
	return 0;
}

void sdk_config_extend_set_pll_track(bool enable)
{
	(void)enable;
}

bool btdm_lpclk_select_src(uint32_t sel)
{
	(void)sel;
	return true;
}

bool btdm_lpclk_set_div(uint32_t div)
{
	(void)div;
	return true;
}

int btdm_hci_tl_io_event_post(int event)
{
	(void)event;
	return ESP_OK;
}

bool API_vhci_host_check_send_available(void)
{
	return true;
}

void API_vhci_host_send_packet(uint8_t *data, uint16_t len)
{
	(void)data;
	(void)len;
}

int API_vhci_host_register_callback(const esp_vhci_host_callback_t *callback)
{
	(void)callback;
	return ESP_OK;
}

int ble_txpwr_set(int power_type, int power_level)
{
	(void)power_type;
	(void)power_level;
	return ESP_OK;
}

int ble_txpwr_get(int power_type)
{
	(void)power_type;
	return ESP_OK;
}

int bredr_txpwr_set(int min_power_level, int max_power_level)
{
	(void)min_power_level;
	(void)max_power_level;
	return ESP_OK;
}

int bredr_txpwr_get(int *min_power_level, int *max_power_level)
{
	(void)min_power_level;
	(void)max_power_level;
	return ESP_OK;
}

void bredr_sco_datapath_set(uint8_t data_path)
{
	(void)data_path;
}

void btdm_controller_scan_duplicate_list_clear(void)
{
}

void coex_pti_v2(void)
{
}

void esp_bt_controller_shutdown(void)
{
}

void sdk_config_set_bt_pll_track_enable(bool enable)
{
	(void)enable;
}

void sdk_config_set_uart_flow_ctrl_enable(bool enable)
{
	(void)enable;
}

void config_bt_funcs_reset(void)
{
}

void config_ble_funcs_reset(void)
{
}

void config_btdm_funcs_reset(void)
{
}

void bt_stack_enableSecCtrlVsCmd(bool en)
{
	(void)en;
}

void bt_stack_enableCoexVsCmd(bool en)
{
	(void)en;
}

void scan_stack_enableAdvFlowCtrlVsCmd(bool en)
{
	(void)en;
}

void adv_stack_enableClearLegacyAdvVsCmd(bool en)
{
	(void)en;
}

void advFilter_stack_enableDupExcListVsCmd(bool en)
{
	(void)en;
}

void chanSel_stack_enableSetCsaVsCmd(bool en)
{
	(void)en;
}

void ble_dtm_funcs_reset(void)
{
}

void ble_scan_funcs_reset(void)
{
}

void ble_42_adv_funcs_reset(void)
{
}

void ble_init_funcs_reset(void)
{
}

void ble_con_funcs_reset(void)
{
}

void ble_cca_funcs_reset(void)
{
}

void ble_ext_adv_funcs_reset(void)
{
}

void ble_ext_scan_funcs_reset(void)
{
}

void ble_base_funcs_reset(void)
{
}

void ble_enc_funcs_reset(void)
{
}

#if defined(CONFIG_SOC_SERIES_ESP32C2) || defined(CONFIG_SOC_SERIES_ESP32C6)

/* Forward declarations of opaque types */
struct osi_coex_funcs_t;
struct os_mempool;
struct os_mempool_ext;
struct os_mbuf;
struct ext_funcs_t;
struct npl_funcs_t;

enum os_error {
	OS_OK = 0
};
typedef enum os_error os_error_t;

/* Function pointer types */
typedef void (*workitem_handler_t)(void *);
typedef struct esp_vhci_host_callback esp_vhci_host_callback_t;
typedef int (*esp_hci_internal_rx_cmd_fn)(uint8_t *cmd, void *arg);
typedef int (*esp_hci_internal_rx_acl_fn)(struct os_mbuf *om, void *arg);
typedef int (*os_mempool_put_fn)(struct os_mempool_ext *ome, void *data, void *arg);
struct ble_npl_count_info_t;

typedef enum {
	EMPTY_ENUM = 0,
} disc_duplicate_mode_t;

void r_filter_duplicate_mode_enable(disc_duplicate_mode_t mode)
{
	(void)mode;
}

void r_filter_duplicate_mode_disable(disc_duplicate_mode_t mode)
{
	(void)mode;
}

void r_filter_duplicate_set_ring_list_max_num(uint32_t max_num)
{
	(void)max_num;
}

void r_scan_duplicate_cache_refresh_set_time(uint32_t period_time)
{
	(void)period_time;
}

int r_ble_controller_deinit(void)
{
	return 0;
}

int r_ble_controller_enable(uint8_t mode)
{
	(void)mode;
	return 0;
}

int r_ble_controller_disable(void)
{
	return 0;
}

int esp_register_ext_funcs(struct ext_funcs_t *funcs)
{
	(void)funcs;
	return 0;
}

void esp_unregister_ext_funcs(void)
{
}

int r_esp_ble_ll_set_public_addr(const uint8_t *addr)
{
	(void)addr;
	return 0;
}

int esp_register_npl_funcs(struct npl_funcs_t *p_npl_func)
{
	(void)p_npl_func;
	return 0;
}

void esp_unregister_npl_funcs(void)
{
}

uint32_t r_os_cputime_get32(void)
{
	return 0;
}

uint32_t r_os_cputime_ticks_to_usecs(uint32_t ticks)
{
	(void)ticks;
	return 0;
}

void r_ble_lll_rfmgmt_set_sleep_cb(void *s_cb, void *w_cb, void *s_arg, void *w_arg,
				   uint32_t us_to_enabled)
{
	(void)s_cb;
	(void)w_cb;
	(void)s_arg;
	(void)w_arg;
	(void)us_to_enabled;
}

void r_ble_rtc_wake_up_state_clr(void)
{
}

void r_ble_ll_customize_peer_sca_set(uint16_t peer_sca)
{
	(void)peer_sca;
}

int r_ble_txpwr_set(esp_ble_enhanced_power_type_t power_type, uint16_t handle, int power_level)
{
	(void)power_type;
	(void)handle;
	(void)power_level;
	return 0;
}

int r_ble_txpwr_get(esp_ble_enhanced_power_type_t power_type, uint16_t handle)
{
	(void)power_type;
	(void)handle;
	return 0;
}

int r_ble_get_npl_element_info(esp_bt_controller_config_t *cfg,
			       struct ble_npl_count_info_t *npl_info)
{
	(void)cfg;
	(void)npl_info;
	return 0;
}

int base_stack_initEnv(void)
{
	return 0;
}

void base_stack_deinitEnv(void)
{
}

int base_stack_enable(void)
{
	return 0;
}

void base_stack_disable(void)
{
}

int conn_stack_initEnv(void)
{
	return 0;
}

void conn_stack_deinitEnv(void)
{
}

int conn_stack_enable(void)
{
	return 0;
}

void conn_stack_disable(void)
{
}

void mmgmt_enableRxbufOptFeature(void)
{
}

void r_ble_hci_trans_cfg_hs(esp_hci_internal_rx_cmd_fn *evt_cb, void *evt_arg,
			    esp_hci_internal_rx_acl_fn *acl_cb, void *acl_arg)
{
	(void)evt_cb;
	(void)evt_arg;
	(void)acl_cb;
	(void)acl_arg;
}

int r_ble_hci_trans_hs_acl_tx(struct os_mbuf *om)
{
	(void)om;
	return 0;
}

int r_ble_hci_trans_hs_cmd_tx(uint8_t *cmd)
{
	(void)cmd;
	return 0;
}

uint8_t *r_ble_hci_trans_buf_alloc(int type)
{
	(void)type;
	return NULL;
}

void r_ble_hci_trans_buf_free(uint8_t *buf)
{
	(void)buf;
}

int r_ble_hci_trans_set_acl_free_cb(os_mempool_put_fn *cb, void *arg)
{
	(void)cb;
	(void)arg;
	return 0;
}

int r_ble_ll_hci_ev_hw_err(uint8_t hw_err)
{
	(void)hw_err;
	return 0;
}

int r_ble_hci_trans_reset(void)
{
	return 0;
}

void esp_ble_hci_trans_init(uint8_t param)
{
	(void)param;
}

void bt_bb_v2_init_cmplx(int print_version)
{
	(void)print_version;
}

int r_esp_ble_msys_init(uint16_t msys_size1, uint16_t msys_size2, uint16_t msys_cnt1,
			uint16_t msys_cnt2, uint8_t from_heap)
{
	(void)msys_size1;
	(void)msys_size2;
	(void)msys_cnt1;
	(void)msys_cnt2;
	(void)from_heap;
	return 0;
}

void r_esp_ble_msys_deinit(void)
{
}
int ble_osi_coex_funcs_register(struct osi_coex_funcs_t *coex_funcs)
{
	(void)coex_funcs;
	return 0;
}
void esp_ble_unregister_bb_funcs(void)
{
}
int esp_ble_register_bb_funcs(void)
{
	return 0;
}
char *ble_controller_get_compile_version(void)
{
	return "dummy_compile_version";
}
int r_ble_controller_init(esp_bt_controller_config_t *cfg)
{
	(void)cfg;
	return 0;
}
void r_swap_in_place(void *buf, int len)
{
	(void)buf;
	(void)len;
}
void r_swap_buf(uint8_t *dst, const uint8_t *src, int len)
{
	(void)dst;
	(void)src;
	(void)len;
}
os_error_t r_os_mempool_init(struct os_mempool *mp, uint16_t blocks, uint32_t block_size,
			     void *membuf, const char *name)
{
	(void)mp;
	(void)blocks;
	(void)block_size;
	(void)membuf;
	(void)name;
	return OS_OK;
}
struct os_mbuf *r_os_msys_get_pkthdr(uint16_t dsize, uint16_t user_hdr_len)
{
	(void)dsize;
	(void)user_hdr_len;
	return NULL;
}
int r_os_mbuf_append(struct os_mbuf *m, const void *, uint16_t)
{
	(void)m;
	(void)NULL;
	(void)0;
	return 0;
}
int r_os_mbuf_copydata(const struct os_mbuf *m, int off, int len, void *dst)
{
	(void)m;
	(void)off;
	(void)len;
	(void)dst;
	return 0;
}
int r_os_mbuf_free_chain(struct os_mbuf *om)
{
	(void)om;
	return 0;
}
#endif /* CONFIG_SOC_SERIES_ESP32C2 || CONFIG_SOC_SERIES_ESP32C6 */

#if defined(CONFIG_SOC_SERIES_ESP32C2)

int esp_ble_rom_func_ptr_init_all(void)
{
	return 0;
}

int esp_ble_ll_set_public_addr(const uint8_t *addr)
{
	(void)addr;
	return 0;
}

int ble_get_npl_element_info(esp_bt_controller_config_t *cfg,
			       struct ble_npl_count_info_t *npl_info)
{
	(void)cfg;
	(void)npl_info;
	return 0;
}

int ble_controller_init(esp_bt_controller_config_t *cfg)
{
	(void)cfg;
	return 0;
}

int ble_controller_deinit(void)
{
	return 0;
}

int ble_controller_enable(uint8_t mode)
{
	(void)mode;
	return 0;
}

int ble_controller_disable(void)
{
	return 0;
}

#endif /* CONFIG_SOC_SERIES_ESP32C2 */
