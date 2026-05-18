#include "esp_now.h"

esp_err_t esp_now_init(void)
{
	return ESP_OK;
}

esp_err_t esp_now_deinit(void)
{
	return ESP_OK;
}

esp_err_t esp_now_get_version(uint32_t *version)
{
	ARG_UNUSED(version);
	return ESP_OK;
}

esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb)
{
	ARG_UNUSED(cb);
	return ESP_OK;
}

esp_err_t esp_now_unregister_recv_cb(void)
{
	return ESP_OK;
}

esp_err_t esp_now_register_send_cb(esp_now_send_cb_t cb)
{
	ARG_UNUSED(cb);
	return ESP_OK;
}

esp_err_t esp_now_unregister_send_cb(void)
{
	return ESP_OK;
}

esp_err_t esp_now_send(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
	ARG_UNUSED(peer_addr);
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	return ESP_OK;
}

esp_err_t esp_now_add_peer(const esp_now_peer_info_t *peer)
{
	ARG_UNUSED(peer);
	return ESP_OK;
}

esp_err_t esp_now_del_peer(const uint8_t *peer_addr)
{
	ARG_UNUSED(peer_addr);
	return ESP_OK;
}

esp_err_t esp_now_mod_peer(const esp_now_peer_info_t *peer)
{
	ARG_UNUSED(peer);
	return ESP_OK;
}

esp_err_t esp_now_set_peer_rate_config(const uint8_t *peer_addr, esp_now_rate_config_t *config)
{
	ARG_UNUSED(peer_addr);
	ARG_UNUSED(config);
	return ESP_OK;
}

esp_err_t esp_now_get_peer(const uint8_t *peer_addr, esp_now_peer_info_t *peer)
{
	ARG_UNUSED(peer_addr);
	ARG_UNUSED(peer);
	return ESP_OK;
}

esp_err_t esp_now_fetch_peer(bool from_head, esp_now_peer_info_t *peer)
{
	ARG_UNUSED(from_head);
	ARG_UNUSED(peer);
	return ESP_OK;
}

bool esp_now_is_peer_exist(const uint8_t *peer_addr)
{
	ARG_UNUSED(peer_addr);
	return false;
}

esp_err_t esp_now_get_peer_num(esp_now_peer_num_t *num)
{
	ARG_UNUSED(num);
	return ESP_OK;
}

esp_err_t esp_now_set_pmk(const uint8_t *pmk)
{
	ARG_UNUSED(pmk);
	return ESP_OK;
}

esp_err_t esp_now_set_wake_window(uint16_t window)
{
	ARG_UNUSED(window);
	return ESP_OK;
}

esp_err_t esp_now_set_user_oui(uint8_t *oui)
{
	ARG_UNUSED(oui);
	return ESP_OK;
}

esp_err_t esp_now_get_user_oui(uint8_t *oui)
{
	ARG_UNUSED(oui);
	return ESP_OK;
}

esp_err_t esp_now_switch_channel_tx(esp_now_switch_channel_t *config)
{
	ARG_UNUSED(config);
	return ESP_OK;
}

esp_err_t esp_now_remain_on_channel(esp_now_remain_on_channel_t *config)
{
	ARG_UNUSED(config);
	return ESP_OK;
}
