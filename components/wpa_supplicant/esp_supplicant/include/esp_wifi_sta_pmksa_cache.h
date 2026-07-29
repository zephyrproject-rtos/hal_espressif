/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ESP_WIFI_STA_PMKSA_CACHE_H
#define ESP_WIFI_STA_PMKSA_CACHE_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_WIFI_STA_PMKSA_MAC_LEN   6U
#define ESP_WIFI_STA_PMKSA_PMKID_LEN 16U
#define ESP_WIFI_STA_PMKSA_PMK_LEN   32U

/* Policy limit for imported PMKSA lifetimes. */
#define ESP_WIFI_STA_PMKSA_MAX_LIFETIME_S 43200U

#define ESP_WIFI_STA_PMKSA_AKM_802_1X        0x000FAC01U /* 00-0F-AC:1 */
#define ESP_WIFI_STA_PMKSA_AKM_802_1X_SHA256 0x000FAC05U /* 00-0F-AC:5 */

/**
 * @brief RSN PMKSA record.
 *
 * @a pmk is secret. Protect persisted records and wipe caller-owned copies.
 * This structure is not a stable serialized storage format.
 */
typedef struct {
    uint8_t bssid[ESP_WIFI_STA_PMKSA_MAC_LEN];    /**< Authenticator address. */
    uint8_t sta_addr[ESP_WIFI_STA_PMKSA_MAC_LEN]; /**< Station address. */
    uint8_t pmkid[ESP_WIFI_STA_PMKSA_PMKID_LEN];
    uint8_t pmk[ESP_WIFI_STA_PMKSA_PMK_LEN];
    uint8_t pmk_len;                 /**< Must be ESP_WIFI_STA_PMKSA_PMK_LEN. */
    uint32_t akm_suite;              /**< IEEE 802.11 AKM suite selector. */
    uint32_t expiration_remaining_s; /**< Non-zero, at most the max above. */
    uint32_t reauth_remaining_s;     /**< At most expiration_remaining_s. */
} esp_wifi_sta_pmksa_cache_entry_t;

/*
 * Not synchronized against the Wi-Fi task: stage before connecting, export
 * while associated, clear while disconnected.
 */

/**
 * @brief Export the PMKSA in use, lifetimes relative to now and capped.
 *
 * @param[out] entry Always zeroed; populated only on success.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_NOT_FOUND, ESP_ERR_NOT_SUPPORTED
 *         for an out-of-scope AKM or PMK length, or ESP_ERR_INVALID_STATE.
 */
esp_err_t esp_wifi_sta_pmksa_cache_export(esp_wifi_sta_pmksa_cache_entry_t *entry);

/**
 * @brief Stage a record for the next association to its BSSID.
 *
 * Discarded if the station address or AKM mismatch. A non-expired record is
 * kept if another BSSID is attempted. Replaces any previous record and evicts
 * the entry it installed.
 * The caller must deduct time elapsed while the record was stored from the
 * remaining lifetimes before staging. A matching-BSSID attempt consumes the
 * staged record except when entry allocation fails, which leaves it for retry.
 *
 * @param[in] entry Record to stage.
 * @return ESP_OK or ESP_ERR_INVALID_ARG if the record is malformed.
 */
esp_err_t esp_wifi_sta_pmksa_cache_stage(const esp_wifi_sta_pmksa_cache_entry_t *entry);

/** @brief Discard the staged record and flush every station PMKSA. @return ESP_OK */
esp_err_t esp_wifi_sta_pmksa_cache_clear(void);

/**
 * @brief Discard staged and externally installed station PMKSA state.
 *
 * Supplicant-learned entries are preserved. An externally installed entry that
 * is in use is also preserved; call this again once disconnected to remove it.
 *
 * @return ESP_OK
 */
esp_err_t esp_wifi_sta_pmksa_cache_clear_external(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP_WIFI_STA_PMKSA_CACHE_H */
