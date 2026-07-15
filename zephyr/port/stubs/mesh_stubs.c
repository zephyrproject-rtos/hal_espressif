#include "esp_mesh.h"

esp_err_t esp_mesh_init(void)
{
	return ESP_OK;
}

esp_err_t esp_mesh_start(void)
{
	return ESP_OK;
}

esp_err_t esp_mesh_set_config(const mesh_cfg_t *config)
{
	ARG_UNUSED(config);
	return ESP_OK;
}

esp_err_t esp_mesh_set_topology(esp_mesh_topology_t topo)
{
	ARG_UNUSED(topo);
	return ESP_OK;
}

esp_err_t esp_mesh_set_max_layer(int max_layer)
{
	ARG_UNUSED(max_layer);
	return ESP_OK;
}

esp_err_t esp_mesh_set_vote_percentage(float percentage)
{
	ARG_UNUSED(percentage);
	return ESP_OK;
}

esp_err_t esp_mesh_set_xon_qsize(int qsize)
{
	ARG_UNUSED(qsize);
	return ESP_OK;
}

esp_err_t esp_mesh_disable_ps(void)
{
	return ESP_OK;
}

esp_err_t esp_mesh_set_ap_assoc_expire(int seconds)
{
	ARG_UNUSED(seconds);
	return ESP_OK;
}

esp_err_t esp_mesh_set_ap_authmode(wifi_auth_mode_t authmode)
{
	ARG_UNUSED(authmode);
	return ESP_OK;
}

esp_err_t esp_mesh_fix_root(bool enable)
{
	ARG_UNUSED(enable);
	return ESP_OK;
}

esp_err_t esp_mesh_set_type(mesh_type_t type)
{
	ARG_UNUSED(type);
	return ESP_OK;
}

esp_err_t esp_mesh_set_self_organized(bool enable, bool select_parent)
{
	ARG_UNUSED(enable);
	ARG_UNUSED(select_parent);
	return ESP_OK;
}

esp_err_t esp_mesh_send_block_time(uint32_t time_ms)
{
	ARG_UNUSED(time_ms);
	return ESP_OK;
}

esp_err_t esp_mesh_post_toDS_state(bool reachable)
{
	ARG_UNUSED(reachable);
	return ESP_OK;
}

bool esp_mesh_is_root(void)
{
	return false;
}

int esp_mesh_get_layer(void)
{
	return -1;
}

int esp_mesh_get_routing_table_size(void)
{
	return 0;
}

esp_err_t esp_mesh_get_routing_table(mesh_addr_t *mac, int len, int *size)
{
	ARG_UNUSED(mac);
	ARG_UNUSED(len);

	if (size != NULL) {
		*size = 0;
	}

	return ESP_OK;
}

esp_err_t esp_mesh_send(const mesh_addr_t *to, const mesh_data_t *data, int flag,
			const mesh_opt_t opt[], int opt_count)
{
	ARG_UNUSED(to);
	ARG_UNUSED(data);
	ARG_UNUSED(flag);
	ARG_UNUSED(opt);
	ARG_UNUSED(opt_count);
	return ESP_OK;
}

esp_err_t esp_mesh_recv(mesh_addr_t *from, mesh_data_t *data, int timeout_ms, int *flag,
			mesh_opt_t opt[], int opt_count)
{
	ARG_UNUSED(from);
	ARG_UNUSED(data);
	ARG_UNUSED(timeout_ms);
	ARG_UNUSED(flag);
	ARG_UNUSED(opt);
	ARG_UNUSED(opt_count);
	return ESP_FAIL;
}

esp_err_t esp_mesh_recv_toDS(mesh_addr_t *from, mesh_addr_t *to, mesh_data_t *data, int timeout_ms,
			     int *flag, mesh_opt_t opt[], int opt_count)
{
	ARG_UNUSED(from);
	ARG_UNUSED(to);
	ARG_UNUSED(data);
	ARG_UNUSED(timeout_ms);
	ARG_UNUSED(flag);
	ARG_UNUSED(opt);
	ARG_UNUSED(opt_count);
	return ESP_FAIL;
}
