
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "private/esp_coexist_internal.h"

/**
 * @brief Pre-Init software coexist
 *
 * @return ESP_OK on success.
 */
esp_err_t coex_pre_init(void)
{
    return ESP_OK;
}

/**
 * @brief Init software coexist
 *
 * @return ESP_OK on success.
 */
esp_err_t coex_init(void)
{
    return ESP_OK;
}

/**
 * @brief De-init software coexist
 */
void coex_deinit(void)
{
}

/**
 * @brief Enable software coexist
 *
 * @return ESP_OK on success.
 */
esp_err_t coex_enable(void)
{
    return ESP_OK;
}

/**
 * @brief Disable software coexist
 */
void coex_disable(void)
{
}

/**
 * @brief Get software coexist version string
 *
 * @return Pointer to version string.
 */
const char *coex_version_get(void)
{
    return "stub";
}

/**
 * @brief Get software coexist version value.
 *
 * @param ptr_version Pointer to version structure.
 * @return ESP_OK on success.
 */
esp_err_t coex_version_get_value(coex_version_t* ptr_version)
{
    (void)ptr_version;
    return ESP_OK;
}

/**
 * @brief Set coexist performance preference.
 *
 * @param prefer Preference value.
 * @return ESP_OK on success.
 */
esp_err_t coex_preference_set(coex_prefer_t prefer)
{
    (void)prefer;
    return ESP_OK;
}

/**
 * @brief Get software coexist status.
 *
 * @param bitmap Bitmap of the module getting status.
 * @return Status (stub returns 0).
 */
uint32_t coex_status_get(uint8_t bitmap)
{
    (void)bitmap;
    return 0;
}

/**
 * @brief WiFi requests coexistence.
 *
 * @param event WiFi event.
 * @param latency Latency.
 * @param duration Duration.
 * @return 0 on success.
 */
int coex_wifi_request(uint32_t event, uint32_t latency, uint32_t duration)
{
    (void)event;
    (void)latency;
    (void)duration;
    return 0;
}

/**
 * @brief WiFi release coexistence.
 *
 * @param event WiFi event.
 * @return 0 on success.
 */
int coex_wifi_release(uint32_t event)
{
    (void)event;
    return 0;
}

/**
 * @brief Set WiFi channel to coexistence module.
 *
 * @param primary WiFi primary channel.
 * @param secondary WiFi secondary channel.
 * @return 0 on success.
 */
int coex_wifi_channel_set(uint8_t primary, uint8_t secondary)
{
    (void)primary;
    (void)secondary;
    return 0;
}

/**
 * @brief Get WiFi channel from coexistence module.
 *
 * @param primary Pointer to WiFi primary channel.
 * @param secondary Pointer to WiFi secondary channel.
 * @return 0 on success.
 */
int coex_wifi_channel_get(uint8_t *primary, uint8_t *secondary)
{
    if (primary) {
        *primary = 0;
    }
    if (secondary) {
        *secondary = 0;
    }
    return 0;
}

/**
 * @brief Register application callback function to update low power clock.
 *
 * @param callback Callback function.
 */
void coex_wifi_register_update_lpclk_callback(coex_set_lpclk_source_callback_t callback)
{
    (void)callback;
}

/**
 * @brief Bluetooth requests coexistence.
 *
 * @param event Bluetooth event.
 * @param latency Latency.
 * @param duration Duration.
 * @return 0 on success.
 */
int coex_bt_request(uint32_t event, uint32_t latency, uint32_t duration)
{
    (void)event;
    (void)latency;
    (void)duration;
    return 0;
}

/**
 * @brief Bluetooth release coexistence.
 *
 * @param event Bluetooth event.
 * @return 0 on success.
 */
int coex_bt_release(uint32_t event)
{
    (void)event;
    return 0;
}

#if defined(CONFIG_IDF_TARGET_ESP32)
/**
 * @brief Bluetooth registers callback function.
 *
 * @param callback Callback function.
 * @return 0 on success.
 */
int coex_register_bt_cb(coex_func_cb_t callback)
{
    (void)callback;
    return 0;
}

/**
 * @brief Acquire the spin-lock used in resetting Bluetooth baseband.
 *
 * @return Lock value (stub returns 0).
 */
uint32_t coex_bb_reset_lock(void)
{
    return 0;
}

/**
 * @brief Release the spin-lock used in resetting Bluetooth baseband.
 *
 * @param restore Value returned from coex_bb_reset_lock.
 */
void coex_bb_reset_unlock(uint32_t restore)
{
    (void)restore;
}
#endif /* CONFIG_IDF_TARGET_ESP32 */

/**
 * @brief Bluetooth registers callback for Wi-Fi channel change notification.
 *
 * @param callback Callback function.
 * @return 0 on success.
 */
int coex_register_wifi_channel_change_callback(coex_wifi_channel_change_cb_t callback)
{
    (void)callback;
    return 0;
}

/**
 * @brief Update low power clock interval.
 */
void coex_update_lpclk_interval(void)
{
}

/**
 * @brief Get coexistence event duration.
 *
 * @param event Coexistence event.
 * @param duration Pointer to store duration.
 * @return 0 on success.
 */
int coex_event_duration_get(uint32_t event, uint32_t *duration)
{
    (void)event;
    if (duration) {
        *duration = 0;
    }
    return 0;
}

#if defined(SOC_COEX_HW_PTI)
/**
 * @brief Get coexistence event priority.
 *
 * @param event Coexistence event.
 * @param pti Pointer to store priority.
 * @return 0 on success.
 */
int coex_pti_get(uint32_t event, uint8_t *pti)
{
    (void)event;
    if (pti) {
        *pti = 0;
    }
    return 0;
}
#endif

/**
 * @brief Clear coexistence status.
 *
 * @param type Coexistence status type.
 * @param status Coexistence status.
 */
void coex_schm_status_bit_clear(uint32_t type, uint32_t status)
{
    (void)type;
    (void)status;
}

/**
 * @brief Set coexistence status.
 *
 * @param type Coexistence status type.
 * @param status Coexistence status.
 */
void coex_schm_status_bit_set(uint32_t type, uint32_t status)
{
    (void)type;
    (void)status;
}

/**
 * @brief Set coexistence scheme interval.
 *
 * @param interval Coexistence scheme interval.
 * @return 0 on success.
 */
int coex_schm_interval_set(uint32_t interval)
{
    (void)interval;
    return 0;
}

/**
 * @brief Get coexistence scheme interval.
 *
 * @return Coexistence scheme interval (stub returns 0).
 */
uint32_t coex_schm_interval_get(void)
{
    return 0;
}

/**
 * @brief Get current coexistence scheme period.
 *
 * @return Coexistence scheme period (stub returns 0).
 */
uint8_t coex_schm_curr_period_get(void)
{
    return 0;
}

/**
 * @brief Get current coexistence scheme phase.
 *
 * @return Pointer to current phase (stub returns NULL).
 */
void * coex_schm_curr_phase_get(void)
{
    return NULL;
}

/**
 * @brief Set current coexistence scheme phase index.
 *
 * @param idx Coexistence scheme phase index.
 * @return 0 on success.
 */
int coex_schm_curr_phase_idx_set(int idx)
{
    (void)idx;
    return 0;
}

/**
 * @brief Get current coexistence scheme phase index.
 *
 * @return Coexistence scheme phase index (stub returns 0).
 */
int coex_schm_curr_phase_idx_get(void)
{
    return 0;
}

/**
 * @brief Register WiFi callback for coexistence starts.
 *
 * @param cb Callback function.
 * @return 0 on success.
 */
int coex_register_start_cb(int (* cb)(void))
{
    (void)cb;
    return 0;
}

/**
 * @brief Restart current coexistence scheme.
 *
 * @return 0 on success.
 */
int coex_schm_process_restart(void)
{
    return 0;
}

/**
 * @brief Register callback for coexistence scheme.
 *
 * @param type Callback type.
 * @param callback Callback pointer.
 * @return 0 on success.
 */
int coex_schm_register_callback(coex_schm_callback_type_t type, void *callback)
{
    (void)type;
    (void)callback;
    return 0;
}

/**
 * @brief Register coexistence adapter functions.
 *
 * @param funcs Pointer to adapter functions.
 * @return ESP_OK on success.
 */
esp_err_t esp_coex_adapter_register(coex_adapter_funcs_t *funcs)
{
    (void)funcs;
    return ESP_OK;
}

#if defined(CONFIG_EXTERNAL_COEX_ENABLE)
/**
 * @brief Set external coexistence advanced information.
 *
 * @param coex_info Advanced coexistence information.
 * @param out_pti1 Deprecated parameter.
 * @param out_pti2 Deprecated parameter.
 * @return ESP_OK on success.
 */
esp_err_t esp_coex_external_params(esp_external_coex_advance_t coex_info, uint32_t out_pti1, uint32_t out_pti2)
{
    (void)coex_info;
    (void)out_pti1;
    (void)out_pti2;
    return ESP_OK;
}

/**
 * @brief Set external coexistence pti levels and enable it.
 *
 * @param level1 External coex low pti.
 * @param level2 External coex mid pti.
 * @param level3 External coex high pti.
 * @return ESP_OK on success.
 */
esp_err_t esp_coex_external_set(esp_coex_pti_level_t level1,
                                esp_coex_pti_level_t level2,
                                esp_coex_pti_level_t level3)
{
    (void)level1;
    (void)level2;
    (void)level3;
    return ESP_OK;
}

/**
 * @brief Disable external coexist.
 */
void esp_coex_external_stop(void)
{
}

/**
 * @brief Set external coexistence wire type.
 *
 * @param wire_type Wire type.
 */
void esp_coex_external_set_wire_type(external_coex_wire_t wire_type)
{
    (void)wire_type;
}

#if defined(SOC_EXTERNAL_COEX_LEADER_TX_LINE)
/**
 * @brief Enable external coexist TX line.
 *
 * @param en Enable flag.
 */
void esp_coex_external_set_txline(bool en)
{
    (void)en;
}
#endif /* SOC_EXTERNAL_COEX_LEADER_TX_LINE */
#endif /* CONFIG_EXTERNAL_COEX_ENABLE */

#if defined(CONFIG_ESP_COEX_POWER_MANAGEMENT)
/**
 * @brief Set coexist scheme flexible period.
 *
 * @param period Flexible period.
 * @return 0 on success.
 */
int coex_schm_flexible_period_set(uint8_t period)
{
    (void)period;
    return 0;
}

/**
 * @brief Get coexist scheme flexible period.
 *
 * @return Flexible period (stub returns 0).
 */
uint8_t coex_schm_flexible_period_get(void)
{
    return 0;
}
#endif /* CONFIG_ESP_COEX_POWER_MANAGEMENT */

/**
 * @brief Check the MD5 values of the coexistence adapter header files.
 *
 * @param md5 MD5 string.
 * @return ESP_OK on success.
 */
esp_err_t esp_coex_adapter_funcs_md5_check(const char *md5)
{
    (void)md5;
    return ESP_OK;
}
