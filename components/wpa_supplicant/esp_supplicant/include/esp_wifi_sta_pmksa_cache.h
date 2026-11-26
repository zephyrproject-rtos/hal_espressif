/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ESP_WIFI_STA_PMKSA_CACHE_H
#define ESP_WIFI_STA_PMKSA_CACHE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_WIFI_STA_PMKSA_MAC_LEN   6U
#define ESP_WIFI_STA_PMKSA_PMKID_LEN 16U
#define ESP_WIFI_STA_PMKSA_PMK_LEN   32U

/* Maximum number of distinct BSSIDs that can be staged. */
#define ESP_WIFI_STA_PMKSA_MAX_ENTRIES 10U

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
 * These functions run in the Wi-Fi task and block until it has handled the
 * request. Call them between esp_wifi_init() and esp_wifi_deinit(), and not
 * from a task the Wi-Fi task can wait on, such as an event handler.
 */

/**
 * @brief Replace imported PMKSA state with records for the next associations.
 *
 * Removes idle entries imported earlier; the entry in use is kept. Malformed
 * records are skipped, a later record for a BSSID replaces an earlier one, and
 * records for BSSIDs beyond ESP_WIFI_STA_PMKSA_MAX_ENTRIES are ignored.
 * Records are copied, so the caller may wipe them on return. Each is installed
 * at the next association attempt and discarded if its station address does
 * not match. The caller must deduct the time a record was stored from its
 * remaining lifetimes.
 *
 * @param[in] entries Records to stage, or NULL if @p count is zero.
 * @param[in] count Number of records.
 * @param[out] staged_count Number of distinct records staged.
 * @return ESP_OK, ESP_ERR_INVALID_ARG if @p staged_count is NULL or @p entries
 *         is NULL with a non-zero @p count, or ESP_FAIL if the Wi-Fi task
 *         could not run the request, in which case nothing changed.
 */
esp_err_t esp_wifi_sta_pmksa_cache_stage(const esp_wifi_sta_pmksa_cache_entry_t *entries,
                                         size_t count, size_t *staged_count);

/**
 * @brief Export one PMKSA of the current station profile.
 *
 * Lifetimes are relative to now and capped. Entries are counted in cache
 * order, which can change between calls.
 *
 * @param[in] index Index among the exportable entries.
 * @param[out] entry Always zeroed; populated only on success.
 * @param[out] entry_count Number of exportable entries, set on ESP_OK and
 *             ESP_ERR_NOT_FOUND and zeroed otherwise.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_NOT_FOUND if @p index is out of
 *         range, ESP_ERR_INVALID_STATE without a station profile, or ESP_FAIL
 *         if the Wi-Fi task could not run the request.
 */
esp_err_t esp_wifi_sta_pmksa_cache_export(size_t index, esp_wifi_sta_pmksa_cache_entry_t *entry,
                                          size_t *entry_count);

/**
 * @brief Discard staged records and cached station PMKSA entries.
 *
 * @param[in] external_only If true, remove only idle imported entries and keep
 *            the entry in use. If false, flush every entry, including learned
 *            ones; the station must be disconnected.
 * @return ESP_OK, or ESP_FAIL if the Wi-Fi task could not run the request.
 */
esp_err_t esp_wifi_sta_pmksa_cache_clear(bool external_only);

#ifdef __cplusplus
}
#endif

#endif /* ESP_WIFI_STA_PMKSA_CACHE_H */
