#include "utils/includes.h"
#include "utils/common.h"

#include "esp_private/wifi.h"
#include "esp_wifi_driver.h"
#include "esp_wifi.h"

uint8_t *esp_wifi_ap_get_prof_pmk_internal(void)
{
	return NULL;
}

struct wifi_ssid *esp_wifi_ap_get_prof_ap_ssid_internal(void)
{
	return NULL;
}

uint8_t esp_wifi_ap_get_prof_authmode_internal(void)
{
	return 0;
}

uint8_t esp_wifi_sta_get_prof_authmode_internal(void)
{
	return 0;
}

uint8_t *esp_wifi_ap_get_prof_password_internal(void)
{
	return NULL;
}

struct wifi_ssid *esp_wifi_sta_get_prof_ssid_internal(void)
{
	return NULL;
}

uint8_t esp_wifi_sta_get_reset_param_internal(void)
{
	return 0;
}

uint8_t esp_wifi_sta_get_pairwise_cipher_internal(void)
{
	return 0;
}

uint8_t esp_wifi_sta_get_group_cipher_internal(void)
{
	return 0;
}

bool esp_wifi_sta_prof_is_wpa_internal(void)
{
	return false;
}

int esp_wifi_get_macaddr_internal(uint8_t if_index, uint8_t *macaddr)
{
	ARG_UNUSED(if_index);
	ARG_UNUSED(macaddr);
	return ESP_OK;
}

int esp_wifi_set_appie_internal(uint8_t type, uint8_t *ie, uint16_t len, uint8_t flag)
{
	ARG_UNUSED(type);
	ARG_UNUSED(ie);
	ARG_UNUSED(len);
	ARG_UNUSED(flag);
	return ESP_OK;
}

int esp_wifi_unset_appie_internal(uint8_t type)
{
	ARG_UNUSED(type);
	return ESP_OK;
}

struct wifi_appie *esp_wifi_get_appie_internal(uint8_t type)
{
	ARG_UNUSED(type);
	return NULL;
}

void *esp_wifi_get_hostap_private_internal(void)
{
	return NULL;
}

uint8_t *esp_wifi_sta_get_prof_password_internal(void)
{
	return NULL;
}

void esp_wifi_deauthenticate_internal(u8 reason_code)
{
	ARG_UNUSED(reason_code);
}

uint16_t esp_wifi_get_spp_attrubute_internal(uint8_t ifx)
{
	ARG_UNUSED(ifx);
	return 0;
}

bool esp_wifi_sta_is_running_internal(void)
{
	return false;
}

bool esp_wifi_auth_done_internal(void)
{
	return false;
}

int esp_wifi_set_ap_key_internal(int alg, const u8 *addr, int idx, u8 *key, size_t key_len)
{
	ARG_UNUSED(alg);
	ARG_UNUSED(addr);
	ARG_UNUSED(idx);
	ARG_UNUSED(key);
	ARG_UNUSED(key_len);
	return ESP_OK;
}

int esp_wifi_set_sta_key_internal(int alg, u8 *addr, int key_idx, int set_tx, u8 *seq,
				  size_t seq_len, u8 *key, size_t key_len, enum key_flag key_flag)
{
	ARG_UNUSED(alg);
	ARG_UNUSED(addr);
	ARG_UNUSED(key_idx);
	ARG_UNUSED(set_tx);
	ARG_UNUSED(seq);
	ARG_UNUSED(seq_len);
	ARG_UNUSED(key);
	ARG_UNUSED(key_len);
	ARG_UNUSED(key_flag);
	return ESP_OK;
}

int esp_wifi_get_sta_key_internal(uint8_t *ifx, int *alg, u8 *addr, int *key_idx, u8 *key,
				  size_t key_len, enum key_flag key_flag)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(alg);
	ARG_UNUSED(addr);
	ARG_UNUSED(key_idx);
	ARG_UNUSED(key);
	ARG_UNUSED(key_len);
	ARG_UNUSED(key_flag);
	return ESP_OK;
}

bool esp_wifi_wpa_ptk_init_done_internal(uint8_t *mac)
{
	ARG_UNUSED(mac);
	return false;
}

uint8_t esp_wifi_sta_set_reset_param_internal(uint8_t reset_flag)
{
	ARG_UNUSED(reset_flag);
	return 0;
}

uint8_t esp_wifi_get_sta_gtk_index_internal(void)
{
	return 0;
}

int esp_wifi_register_tx_cb_internal(wifi_tx_cb_t fn, u8 id)
{
	ARG_UNUSED(fn);
	ARG_UNUSED(id);
	return ESP_OK;
}

int esp_wifi_register_eapol_txdonecb_internal(eapol_txcb_t fn)
{
	ARG_UNUSED(fn);
	return ESP_OK;
}

int esp_wifi_register_wpa_cb_internal(struct wpa_funcs *cb)
{
	ARG_UNUSED(cb);
	return ESP_OK;
}

int esp_wifi_unregister_wpa_cb_internal(void)
{
	return ESP_OK;
}

int esp_wifi_get_assoc_bssid_internal(uint8_t *bssid)
{
	ARG_UNUSED(bssid);
	return ESP_OK;
}

bool esp_wifi_sta_is_ap_notify_completed_rsne_internal(void)
{
	return false;
}

int esp_wifi_ap_deauth_internal(uint8_t *mac, uint32_t reason)
{
	ARG_UNUSED(mac);
	ARG_UNUSED(reason);
	return ESP_OK;
}

int esp_wifi_ipc_internal(wifi_ipc_config_t *cfg, bool sync)
{
	ARG_UNUSED(cfg);
	ARG_UNUSED(sync);
	return ESP_OK;
}

int esp_wifi_register_wpa2_cb_internal(struct wpa2_funcs *cb)
{
	ARG_UNUSED(cb);
	return ESP_OK;
}

int esp_wifi_unregister_wpa2_cb_internal(void)
{
	return ESP_OK;
}

bool esp_wifi_sta_prof_is_wpa2_internal(void)
{
	return false;
}

bool esp_wifi_sta_prof_is_rsn_internal(void)
{
	return false;
}

bool esp_wifi_sta_prof_is_wapi_internal(void)
{
	return false;
}

esp_err_t esp_wifi_sta_wpa2_ent_disable_internal(wifi_wpa2_param_t *param)
{
	ARG_UNUSED(param);
	return ESP_OK;
}

esp_err_t esp_wifi_sta_wpa2_ent_enable_internal(wifi_wpa2_param_t *param)
{
	ARG_UNUSED(param);
	return ESP_OK;
}

esp_err_t esp_wifi_set_wpa2_ent_state_internal(wpa2_ent_eap_state_t state)
{
	ARG_UNUSED(state);
	return ESP_OK;
}

int esp_wifi_get_wps_type_internal(void)
{
	return 0;
}

int esp_wifi_set_wps_type_internal(uint32_t type)
{
	ARG_UNUSED(type);
	return ESP_OK;
}

int esp_wifi_get_wps_status_internal(void)
{
	return 0;
}

int esp_wifi_set_wps_status_internal(uint32_t status)
{
	ARG_UNUSED(status);
	return ESP_OK;
}

int esp_wifi_disarm_sta_connection_timer_internal(void)
{
	return ESP_OK;
}

bool esp_wifi_get_sniffer_internal(void)
{
	return false;
}

int esp_wifi_set_wps_cb_internal(struct wps_funcs *wps_cb)
{
	ARG_UNUSED(wps_cb);
	return ESP_OK;
}

bool esp_wifi_enable_sta_privacy_internal(void)
{
	return false;
}

uint8_t esp_wifi_get_user_init_flag_internal(void)
{
	return 0;
}

esp_err_t esp_wifi_internal_supplicant_header_md5_check(const char *md5)
{
	ARG_UNUSED(md5);
	return ESP_OK;
}

int esp_wifi_sta_update_ap_info_internal(void)
{
	return ESP_OK;
}

uint8_t *esp_wifi_sta_get_ap_info_prof_pmk_internal(void)
{
	return NULL;
}

esp_err_t esp_wifi_set_wps_start_flag_internal(bool start)
{
	ARG_UNUSED(start);
	return ESP_OK;
}

uint16_t esp_wifi_sta_pmf_enabled(void)
{
	return 0;
}

wifi_cipher_type_t esp_wifi_sta_get_mgmt_group_cipher(void)
{
	return 0;
}

int esp_wifi_set_igtk_internal(uint8_t if_index, const wifi_wpa_igtk_t *igtk)
{
	ARG_UNUSED(if_index);
	ARG_UNUSED(igtk);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_issue_disconnect(uint8_t reason_code)
{
	ARG_UNUSED(reason_code);
	return ESP_OK;
}

bool esp_wifi_skip_supp_pmkcaching(void)
{
	return false;
}

bool esp_wifi_is_rm_enabled_internal(uint8_t if_index)
{
	ARG_UNUSED(if_index);
	return false;
}

bool esp_wifi_is_btm_enabled_internal(uint8_t if_index)
{
	ARG_UNUSED(if_index);
	return false;
}

esp_err_t esp_wifi_register_mgmt_frame_internal(uint32_t type, uint32_t subtype)
{
	ARG_UNUSED(type);
	ARG_UNUSED(subtype);
	return ESP_OK;
}

esp_err_t esp_wifi_send_mgmt_frm_internal(const wifi_mgmt_frm_req_t *req)
{
	ARG_UNUSED(req);
	return ESP_OK;
}

uint8_t esp_wifi_ap_get_prof_pairwise_cipher_internal(void)
{
	return 0;
}

esp_err_t esp_wifi_action_tx_req(uint8_t type, uint8_t channel, uint32_t wait_time_ms,
				 const wifi_action_tx_req_t *req)
{
	ARG_UNUSED(type);
	ARG_UNUSED(channel);
	ARG_UNUSED(wait_time_ms);
	ARG_UNUSED(req);
	return ESP_OK;
}

esp_err_t esp_wifi_remain_on_channel(uint8_t ifx, uint8_t type, uint8_t channel,
				     uint32_t wait_time_ms, wifi_action_rx_cb_t rx_cb)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(type);
	ARG_UNUSED(channel);
	ARG_UNUSED(wait_time_ms);
	ARG_UNUSED(rx_cb);
	return ESP_OK;
}

bool esp_wifi_is_mbo_enabled_internal(uint8_t if_index)
{
	ARG_UNUSED(if_index);
	return false;
}

void esp_wifi_get_pmf_config_internal(wifi_pmf_config_t *pmf_cfg, uint8_t ifx)
{
	ARG_UNUSED(pmf_cfg);
	ARG_UNUSED(ifx);
}

bool esp_wifi_is_ft_enabled_internal(uint8_t if_index)
{
	ARG_UNUSED(if_index);
	return false;
}

uint8_t esp_wifi_sta_get_config_sae_pk_internal(void)
{
	return 0;
}

void esp_wifi_sta_disable_sae_pk_internal(void)
{
}

void esp_wifi_sta_disable_wpa2_authmode_internal(void)
{
}

void esp_wifi_sta_disable_owe_trans_internal(void)
{
}

uint8_t esp_wifi_ap_get_max_sta_conn(void)
{
	return 0;
}

uint8_t esp_wifi_get_config_sae_pwe_h2e_internal(uint8_t ifx)
{
	ARG_UNUSED(ifx);
	return 0;
}

bool esp_wifi_ap_notify_node_sae_auth_done(uint8_t *mac)
{
	ARG_UNUSED(mac);
	return false;
}

bool esp_wifi_ap_is_sta_sae_reauth_node(uint8_t *mac)
{
	ARG_UNUSED(mac);
	return false;
}

uint8_t *esp_wifi_sta_get_sae_identifier_internal(void)
{
	return NULL;
}

bool esp_wifi_eb_tx_status_success_internal(void *eb)
{
	ARG_UNUSED(eb);
	return false;
}

uint8_t *esp_wifi_sta_get_rsnxe(u8 *bssid)
{
	ARG_UNUSED(bssid);
	return NULL;
}

esp_err_t esp_wifi_sta_connect_internal(const uint8_t *bssid)
{
	ARG_UNUSED(bssid);
	return ESP_OK;
}

void esp_wifi_enable_sae_pk_only_mode_internal(void)
{
}

esp_err_t esp_wifi_init_internal(const wifi_init_config_t *config)
{
	ARG_UNUSED(config);
	return ESP_OK;
}

esp_err_t esp_wifi_deinit_internal(void)
{
	return ESP_OK;
}

void esp_wifi_internal_free_rx_buffer(void *buffer)
{
	ARG_UNUSED(buffer);
}

int esp_wifi_internal_tx(wifi_interface_t wifi_if, void *buffer, uint16_t len)
{
	ARG_UNUSED(wifi_if);
	ARG_UNUSED(buffer);
	ARG_UNUSED(len);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_tx_by_ref(wifi_interface_t ifx, void *buffer, size_t len,
				      void *netstack_buf)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(buffer);
	ARG_UNUSED(len);
	ARG_UNUSED(netstack_buf);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_wapi_init(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_internal_wapi_deinit(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_internal_reg_netstack_buf_cb(wifi_netstack_buf_ref_cb_t ref,
						wifi_netstack_buf_free_cb_t free)
{
	ARG_UNUSED(ref);
	ARG_UNUSED(free);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_reg_rxcb(wifi_interface_t ifx, wifi_rxcb_t fn)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(fn);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_set_sta_ip(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_internal_set_fix_rate(wifi_interface_t ifx, bool en, wifi_phy_rate_t rate)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(en);
	ARG_UNUSED(rate);
	return ESP_OK;
}

esp_err_t esp_smartconfig_internal_start(const smartconfig_start_config_t *config)
{
	ARG_UNUSED(config);
	return ESP_OK;
}

esp_err_t esp_smartconfig_internal_stop(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_internal_osi_funcs_md5_check(const char *md5)
{
	ARG_UNUSED(md5);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_crypto_funcs_md5_check(const char *md5)
{
	ARG_UNUSED(md5);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_wifi_type_md5_check(const char *md5)
{
	ARG_UNUSED(md5);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_wifi_he_type_md5_check(const char *md5)
{
	ARG_UNUSED(md5);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_esp_wifi_md5_check(const char *md5)
{
	ARG_UNUSED(md5);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_esp_wifi_he_md5_check(const char *md5)
{
	ARG_UNUSED(md5);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_update_mac_time(uint32_t time_delta)
{
	ARG_UNUSED(time_delta);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_set_log_level(wifi_log_level_t level)
{
	ARG_UNUSED(level);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_set_log_mod(wifi_log_module_t module, uint32_t submodule, bool enable)
{
	ARG_UNUSED(module);
	ARG_UNUSED(submodule);
	ARG_UNUSED(enable);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_get_log(wifi_log_level_t *log_level, uint32_t *log_mod)
{
	ARG_UNUSED(log_level);
	ARG_UNUSED(log_mod);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_ioctl(int cmd, wifi_ioctl_config_t *cfg)
{
	ARG_UNUSED(cmd);
	ARG_UNUSED(cfg);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_get_config_channel(wifi_interface_t ifx, uint8_t *primary,
					       uint8_t *second)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(primary);
	ARG_UNUSED(second);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_get_negotiated_channel(wifi_interface_t ifx, uint8_t aid,
						   uint8_t *primary, uint8_t *second)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(aid);
	ARG_UNUSED(primary);
	ARG_UNUSED(second);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_get_negotiated_bandwidth(wifi_interface_t ifx, uint8_t aid, uint8_t *bw)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(aid);
	ARG_UNUSED(bw);
	return ESP_OK;
}

#if SOC_WIFI_HW_TSF
bool esp_wifi_internal_is_tsf_active(void)
{
	return false;
}

void esp_wifi_internal_update_light_sleep_wake_ahead_time(uint32_t time)
{
	ARG_UNUSED(time);
}

esp_err_t esp_wifi_update_tsf_tick_interval(void)
{
	return ESP_OK;
}
#endif

#if (CONFIG_FREERTOS_USE_TICKLESS_IDLE && SOC_PM_MODEM_RETENTION_BY_REGDMA)
void *esp_wifi_internal_mac_retention_context_get(int *config_size)
{
	ARG_UNUSED(config_size);
	return NULL;
}
#endif

#if CONFIG_MAC_BB_PD
esp_err_t esp_wifi_internal_set_mac_sleep(bool enable)
{
	ARG_UNUSED(enable);
	return ESP_OK;
}

void pm_mac_sleep(void)
{
}

void pm_mac_wakeup(void)
{
}
#endif

esp_err_t esp_wifi_set_tx_done_cb(wifi_tx_done_cb_t cb)
{
	ARG_UNUSED(cb);
	return ESP_OK;
}

esp_err_t esp_wifi_internal_set_spp_amsdu(wifi_interface_t ifidx, bool spp_cap, bool spp_req)
{
	ARG_UNUSED(ifidx);
	ARG_UNUSED(spp_cap);
	ARG_UNUSED(spp_req);
	return ESP_OK;
}

void esp_wifi_internal_update_light_sleep_default_params(int min_freq_mhz, int max_freq_mhz)
{
	ARG_UNUSED(min_freq_mhz);
	ARG_UNUSED(max_freq_mhz);
}

void esp_wifi_set_sleep_min_active_time(uint32_t min_active_time)
{
	ARG_UNUSED(min_active_time);
}

void esp_wifi_set_keep_alive_time(uint32_t keep_alive_time)
{
	ARG_UNUSED(keep_alive_time);
}

void esp_wifi_set_sleep_wait_broadcast_data_time(uint32_t time)
{
	ARG_UNUSED(time);
}

void esp_wifi_beacon_monitor_configure(wifi_beacon_monitor_config_t *config)
{
	ARG_UNUSED(config);
}

void esp_wifi_internal_modem_state_configure(bool require_modem_state)
{
	ARG_UNUSED(require_modem_state);
}

void esp_wifi_internal_light_sleep_configure(bool light_sleep_enable)
{
	ARG_UNUSED(light_sleep_enable);
}

esp_err_t esp_nan_internal_publish_service(const wifi_nan_publish_cfg_t *publish_cfg, uint8_t *id,
					   bool cancel)
{
	ARG_UNUSED(publish_cfg);
	ARG_UNUSED(id);
	ARG_UNUSED(cancel);
	return ESP_OK;
}

esp_err_t esp_nan_internal_subscribe_service(const wifi_nan_subscribe_cfg_t *subscribe_cfg,
					     uint8_t *id, bool cancel)
{
	ARG_UNUSED(subscribe_cfg);
	ARG_UNUSED(id);
	ARG_UNUSED(cancel);
	return ESP_OK;
}

esp_err_t esp_nan_internal_send_followup(const wifi_nan_followup_params_t *fup_params)
{
	ARG_UNUSED(fup_params);
	return ESP_OK;
}

esp_err_t esp_nan_internal_datapath_req(wifi_nan_datapath_req_t *req, uint8_t *ndp_id)
{
	ARG_UNUSED(req);
	ARG_UNUSED(ndp_id);
	return ESP_OK;
}

esp_err_t esp_nan_internal_datapath_resp(wifi_nan_datapath_resp_t *resp)
{
	ARG_UNUSED(resp);
	return ESP_OK;
}

esp_err_t esp_nan_internal_datapath_end(wifi_nan_datapath_end_req_t *req)
{
	ARG_UNUSED(req);
	return ESP_OK;
}

esp_err_t esp_wifi_set_mode(wifi_mode_t mode)
{
	ARG_UNUSED(mode);
	return ESP_OK;
}

esp_err_t esp_wifi_get_mode(wifi_mode_t *mode)
{
	ARG_UNUSED(mode);
	return ESP_OK;
}

esp_err_t esp_wifi_start(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_stop(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_restore(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_connect(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_disconnect(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_clear_fast_connect(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_deauth_sta(uint16_t aid)
{
	ARG_UNUSED(aid);
	return ESP_OK;
}

esp_err_t esp_wifi_scan_start(const wifi_scan_config_t *config, bool block)
{
	ARG_UNUSED(config);
	ARG_UNUSED(block);
	return ESP_OK;
}

esp_err_t esp_wifi_scan_stop(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_scan_get_ap_num(uint16_t *number)
{
	ARG_UNUSED(number);
	return ESP_OK;
}

esp_err_t esp_wifi_scan_get_ap_records(uint16_t *number, wifi_ap_record_t *ap_records)
{
	ARG_UNUSED(number);
	ARG_UNUSED(ap_records);
	return ESP_OK;
}

esp_err_t esp_wifi_scan_get_ap_record(wifi_ap_record_t *ap_record)
{
	ARG_UNUSED(ap_record);
	return ESP_OK;
}

esp_err_t esp_wifi_clear_ap_list(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t *ap_info)
{
	ARG_UNUSED(ap_info);
	return ESP_OK;
}

esp_err_t esp_wifi_set_ps(wifi_ps_type_t type)
{
	ARG_UNUSED(type);
	return ESP_OK;
}

esp_err_t esp_wifi_get_ps(wifi_ps_type_t *type)
{
	ARG_UNUSED(type);
	return ESP_OK;
}

esp_err_t esp_wifi_set_protocol(wifi_interface_t ifx, uint8_t protocol_bitmap)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(protocol_bitmap);
	return ESP_OK;
}

esp_err_t esp_wifi_get_protocol(wifi_interface_t ifx, uint8_t *protocol_bitmap)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(protocol_bitmap);
	return ESP_OK;
}

esp_err_t esp_wifi_set_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t bw)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(bw);
	return ESP_OK;
}

esp_err_t esp_wifi_get_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t *bw)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(bw);
	return ESP_OK;
}

esp_err_t esp_wifi_set_channel(uint8_t primary, wifi_second_chan_t second)
{
	ARG_UNUSED(primary);
	ARG_UNUSED(second);
	return ESP_OK;
}

esp_err_t esp_wifi_get_channel(uint8_t *primary, wifi_second_chan_t *second)
{
	ARG_UNUSED(primary);
	ARG_UNUSED(second);
	return ESP_OK;
}

esp_err_t esp_wifi_set_country(const wifi_country_t *country)
{
	ARG_UNUSED(country);
	return ESP_OK;
}

esp_err_t esp_wifi_get_country(wifi_country_t *country)
{
	ARG_UNUSED(country);
	return ESP_OK;
}

esp_err_t esp_wifi_set_mac(wifi_interface_t ifx, const uint8_t mac[6])
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(mac);
	return ESP_OK;
}

esp_err_t esp_wifi_get_mac(wifi_interface_t ifx, uint8_t mac[6])
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(mac);
	return ESP_OK;
}

esp_err_t esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_cb_t cb)
{
	ARG_UNUSED(cb);
	return ESP_OK;
}

esp_err_t esp_wifi_set_promiscuous(bool en)
{
	ARG_UNUSED(en);
	return ESP_OK;
}

esp_err_t esp_wifi_get_promiscuous(bool *en)
{
	ARG_UNUSED(en);
	return ESP_OK;
}

esp_err_t esp_wifi_set_promiscuous_filter(const wifi_promiscuous_filter_t *filter)
{
	ARG_UNUSED(filter);
	return ESP_OK;
}

esp_err_t esp_wifi_get_promiscuous_filter(wifi_promiscuous_filter_t *filter)
{
	ARG_UNUSED(filter);
	return ESP_OK;
}

esp_err_t esp_wifi_set_promiscuous_ctrl_filter(const wifi_promiscuous_filter_t *filter)
{
	ARG_UNUSED(filter);
	return ESP_OK;
}

esp_err_t esp_wifi_get_promiscuous_ctrl_filter(wifi_promiscuous_filter_t *filter)
{
	ARG_UNUSED(filter);
	return ESP_OK;
}

esp_err_t esp_wifi_set_config(wifi_interface_t interface, wifi_config_t *conf)
{
	ARG_UNUSED(interface);
	ARG_UNUSED(conf);
	return ESP_OK;
}

esp_err_t esp_wifi_get_config(wifi_interface_t interface, wifi_config_t *conf)
{
	ARG_UNUSED(interface);
	ARG_UNUSED(conf);
	return ESP_OK;
}

esp_err_t esp_wifi_ap_get_sta_list(wifi_sta_list_t *sta)
{
	ARG_UNUSED(sta);
	return ESP_OK;
}

esp_err_t esp_wifi_ap_get_sta_aid(const uint8_t mac[6], uint16_t *aid)
{
	ARG_UNUSED(mac);
	ARG_UNUSED(aid);
	return ESP_OK;
}

esp_err_t esp_wifi_set_storage(wifi_storage_t storage)
{
	ARG_UNUSED(storage);
	return ESP_OK;
}

esp_err_t esp_wifi_set_vendor_ie(bool enable, wifi_vendor_ie_type_t type, wifi_vendor_ie_id_t idx,
				 const void *vnd_ie)
{
	ARG_UNUSED(enable);
	ARG_UNUSED(type);
	ARG_UNUSED(idx);
	ARG_UNUSED(vnd_ie);
	return ESP_OK;
}

esp_err_t esp_wifi_set_vendor_ie_cb(esp_vendor_ie_cb_t cb, void *ctx)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(ctx);
	return ESP_OK;
}

esp_err_t esp_wifi_set_max_tx_power(int8_t power)
{
	ARG_UNUSED(power);
	return ESP_OK;
}

esp_err_t esp_wifi_get_max_tx_power(int8_t *power)
{
	ARG_UNUSED(power);
	return ESP_OK;
}

esp_err_t esp_wifi_set_event_mask(uint32_t mask)
{
	ARG_UNUSED(mask);
	return ESP_OK;
}

esp_err_t esp_wifi_get_event_mask(uint32_t *mask)
{
	ARG_UNUSED(mask);
	return ESP_OK;
}

esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(buffer);
	ARG_UNUSED(len);
	ARG_UNUSED(en_sys_seq);
	return ESP_OK;
}

esp_err_t esp_wifi_set_csi_rx_cb(wifi_csi_cb_t cb, void *ctx)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(ctx);
	return ESP_OK;
}

esp_err_t esp_wifi_set_csi_config(const wifi_csi_config_t *config)
{
	ARG_UNUSED(config);
	return ESP_OK;
}

esp_err_t esp_wifi_set_csi(bool en)
{
	ARG_UNUSED(en);
	return ESP_OK;
}

esp_err_t esp_wifi_set_ant_gpio(const wifi_ant_gpio_config_t *config)
{
	ARG_UNUSED(config);
	return ESP_OK;
}

esp_err_t esp_wifi_get_ant_gpio(wifi_ant_gpio_config_t *config)
{
	ARG_UNUSED(config);
	return ESP_OK;
}

esp_err_t esp_wifi_set_ant(const wifi_ant_config_t *config)
{
	ARG_UNUSED(config);
	return ESP_OK;
}

esp_err_t esp_wifi_get_ant(wifi_ant_config_t *config)
{
	ARG_UNUSED(config);
	return ESP_OK;
}

int64_t esp_wifi_get_tsf_time(wifi_interface_t interface)
{
	ARG_UNUSED(interface);
	return 0;
}

esp_err_t esp_wifi_set_inactive_time(wifi_interface_t ifx, uint16_t sec)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(sec);
	return ESP_OK;
}

esp_err_t esp_wifi_get_inactive_time(wifi_interface_t ifx, uint16_t *sec)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(sec);
	return ESP_OK;
}

esp_err_t esp_wifi_statis_dump(uint32_t modules)
{
	ARG_UNUSED(modules);
	return ESP_OK;
}

esp_err_t esp_wifi_set_rssi_threshold(int32_t rssi)
{
	ARG_UNUSED(rssi);
	return ESP_OK;
}

esp_err_t esp_wifi_ftm_initiate_session(wifi_ftm_initiator_cfg_t *cfg)
{
	ARG_UNUSED(cfg);
	return ESP_OK;
}

esp_err_t esp_wifi_ftm_end_session(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_ftm_resp_set_offset(int16_t offset_cm)
{
	ARG_UNUSED(offset_cm);
	return ESP_OK;
}

esp_err_t esp_wifi_ftm_get_report(wifi_ftm_report_entry_t *report, uint8_t num_entries)
{
	ARG_UNUSED(report);
	ARG_UNUSED(num_entries);
	return ESP_OK;
}

esp_err_t esp_wifi_config_11b_rate(wifi_interface_t ifx, bool disable)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(disable);
	return ESP_OK;
}

esp_err_t esp_wifi_connectionless_module_set_wake_interval(uint16_t wake_interval)
{
	ARG_UNUSED(wake_interval);
	return ESP_OK;
}

esp_err_t esp_wifi_force_wakeup_acquire(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_force_wakeup_release(void)
{
	return ESP_OK;
}

esp_err_t esp_wifi_set_country_code(const char *country, bool ieee80211d_enabled)
{
	ARG_UNUSED(country);
	ARG_UNUSED(ieee80211d_enabled);
	return ESP_OK;
}

esp_err_t esp_wifi_get_country_code(char *country)
{
	ARG_UNUSED(country);
	return ESP_OK;
}

esp_err_t esp_wifi_config_80211_tx_rate(wifi_interface_t ifx, wifi_phy_rate_t rate)
{
	ARG_UNUSED(ifx);
	ARG_UNUSED(rate);
	return ESP_OK;
}

esp_err_t esp_wifi_disable_pmf_config(wifi_interface_t ifx)
{
	ARG_UNUSED(ifx);
	return ESP_OK;
}

esp_err_t esp_wifi_sta_get_aid(uint16_t *aid)
{
	ARG_UNUSED(aid);
	return ESP_OK;
}

esp_err_t esp_wifi_sta_get_negotiated_phymode(wifi_phy_mode_t *phymode)
{
	ARG_UNUSED(phymode);
	return ESP_OK;
}

esp_err_t esp_wifi_set_dynamic_cs(bool enabled)
{
	ARG_UNUSED(enabled);
	return ESP_OK;
}

esp_err_t esp_wifi_sta_get_rssi(int *rssi)
{
	ARG_UNUSED(rssi);
	return ESP_OK;
}

#if CONFIG_ESP_COEX_POWER_MANAGEMENT
esp_err_t esp_wifi_coex_pwr_configure(bool enabled)
{
	ARG_UNUSED(enabled);
	return ESP_OK;
}
#endif

uint8_t esp_wifi_ap_get_transition_disable_internal(void)
{
	return 0;
}

uint8_t esp_wifi_sta_get_reset_nvs_pmk_internal(void)
{
	return 0;
}

uint8_t esp_wifi_sta_set_reset_nvs_pmk_internal(uint8_t reset_flag)
{
	return 0;
}
