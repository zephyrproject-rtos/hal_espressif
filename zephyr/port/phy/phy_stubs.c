#include "utils/includes.h"
#include "utils/common.h"

#include "esp_private/phy.h"
#include "esp_phy_init.h"

void phy_param_track_tot(bool en_wifi, bool en_ble_154)
{
}

void phy_get_romfunc_addr(void)
{
	return;
}

int register_chipv7_phy(const esp_phy_init_data_t *init_data, esp_phy_calibration_data_t *cal_data,
			esp_phy_calibration_mode_t cal_mode)
{
	ARG_UNUSED(init_data);
	ARG_UNUSED(cal_data);
	ARG_UNUSED(cal_mode);
	return ESP_CAL_DATA_CHECK_FAIL;
}

uint32_t phy_get_rf_cal_version(void)
{
	return 0;
}

void phy_set_wifi_mode_only(bool wifi_only)
{
	ARG_UNUSED(wifi_only);
}

void coex_bt_high_prio(void)
{
	return;
}

void phy_wakeup_init(void)
{
	return;
}

void phy_close_rf(void)
{
	return;
}

#if !CONFIG_IDF_TARGET_ESP32
void phy_xpd_tsens(void)
{
	return;
}
#endif

#if CONFIG_IDF_TARGET_ESP32C3
void phy_init_flag(void)
{
	return;
}
#endif

#if CONFIG_IDF_TARGET_ESP32C6
void phy_i2c_master_mem_cfg(phy_i2c_master_command_attribute_t *attr)
{
	ARG_UNUSED(attr);
}
#endif

uint8_t phy_dig_reg_backup(bool backup_en, uint32_t *mem_addr)
{
	ARG_UNUSED(backup_en);
	ARG_UNUSED(mem_addr);
	return 0;
}

#if CONFIG_MAC_BB_PD
void phy_freq_mem_backup(bool backup_en, uint32_t *mem)
{
	ARG_UNUSED(backup_en);
	ARG_UNUSED(mem);
}
#endif

#if CONFIG_ESP_PHY_ENABLE_USB
void phy_bbpll_en_usb(bool en)
{
	ARG_UNUSED(en);
}
#endif

#if CONFIG_IDF_TARGET_ESP32S2
void phy_eco_version_sel(uint8_t chip_ver)
{
	ARG_UNUSED(chip_ver);
}
#endif

#if CONFIG_ESP_PHY_IMPROVE_RX_11B
void phy_improve_rx_special(bool enable)
{
	ARG_UNUSED(enable);
}
#endif

esp_err_t esp_phy_load_cal_data_from_nvs(esp_phy_calibration_data_t *out_cal_data)
{
	ARG_UNUSED(out_cal_data);
	return ESP_OK;
}

esp_err_t esp_phy_store_cal_data_to_nvs(const esp_phy_calibration_data_t *cal_data)
{
	ARG_UNUSED(cal_data);
	return ESP_OK;
}

esp_err_t esp_phy_erase_cal_data_in_nvs(void)
{
	return ESP_OK;
}

#if CONFIG_MAC_BB_PD
void esp_mac_bb_pd_mem_init(void)
{
	return;
}

void esp_mac_bb_pd_mem_deinit(void)
{
	return;
}

void esp_mac_bb_power_up(void)
{
	return;
}

void esp_mac_bb_power_down(void)
{
	return;
}
#endif

#if CONFIG_ESP_PHY_MULTIPLE_INIT_DATA_BIN
esp_err_t esp_phy_apply_phy_init_data(uint8_t *init_data)
{
	ARG_UNUSED(init_data);
	return ESP_OK;
}
#endif

char *get_phy_version_str(void)
{
	return "";
}

void phy_init_param_set(uint8_t param)
{
	ARG_UNUSED(param);
}

void phy_wifi_enable_set(uint8_t enable)
{
	ARG_UNUSED(enable);
}
