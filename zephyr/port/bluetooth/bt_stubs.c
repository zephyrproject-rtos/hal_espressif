
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_bt.h"

int btdm_osi_funcs_register(void *osi_funcs)
{
	return ESP_OK;
}

int btdm_controller_init(uint32_t config_mask, esp_bt_controller_config_t *config_opts)
{
	return ESP_OK;
}

void btdm_controller_deinit(void)
{
	// No-op
}

int btdm_controller_enable(esp_bt_mode_t mode)
{
	return ESP_OK;
}

void btdm_controller_disable(void)
{
	// No-op
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
	// No-op
}

#ifdef CONFIG_SOC_SERIES_ESP32
int btdm_dispatch_work_to_controller(workitem_handler_t callback, void *arg, bool blocking)
{
	return ESP_OK;
}
#endif

void btdm_controller_enable_sleep(bool enable)
{
	// No-op
}

void btdm_controller_set_sleep_mode(uint8_t mode)
{
	// No-op
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
	// No-op
}

void btdm_in_wakeup_requesting_set(bool in_wakeup_requesting)
{
	// No-op
}

#ifndef CONFIG_SOC_SERIES_ESP32

int btdm_vnd_offload_task_register(btdm_vnd_ol_sig_t sig, btdm_vnd_ol_task_func_t func)
{
	return ESP_OK;
}

int btdm_vnd_offload_task_deregister(btdm_vnd_ol_sig_t sig)
{
	return ESP_OK;
}

int r_btdm_vnd_offload_post(btdm_vnd_ol_sig_t sig, void *param)
{
	return ESP_OK;
}

#endif /* CONFIG_SOC_SERIES_ESP32 */
uint8_t btdm_sleep_clock_sync(void)
{
	return 0;
}

bool btdm_lpclk_select_src(uint32_t sel)
{
	return true;
}

bool btdm_lpclk_set_div(uint32_t div)
{
	return true;
}

int btdm_hci_tl_io_event_post(int event)
{
	return ESP_OK;
}

bool API_vhci_host_check_send_available(void)
{
	return true;
}

void API_vhci_host_send_packet(uint8_t *data, uint16_t len)
{
	// No-op
}

int API_vhci_host_register_callback(const esp_vhci_host_callback_t *callback)
{
	return ESP_OK;
}

int ble_txpwr_set(int power_type, int power_level)
{
	return ESP_OK;
}

int ble_txpwr_get(int power_type)
{
	return ESP_OK;
}

int bredr_txpwr_set(int min_power_level, int max_power_level)
{
	return ESP_OK;
}

int bredr_txpwr_get(int *min_power_level, int *max_power_level)
{
	return ESP_OK;
}

void bredr_sco_datapath_set(uint8_t data_path)
{
	// No-op
}

void btdm_controller_scan_duplicate_list_clear(void)
{
	// No-op
}

void coex_pti_v2(void)
{
	// No-op
}

void sdk_config_set_bt_pll_track_enable(bool enable)
{
	// No-op
}

void sdk_config_set_uart_flow_ctrl_enable(bool enable)
{
	// No-op
}

void config_bt_funcs_reset(void)
{
	// No-op
}

void config_ble_funcs_reset(void)
{
	// No-op
}

void config_btdm_funcs_reset(void)
{
	// No-op
}
