/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/includes.h"
#include "utils/common.h"
#include "common/defs.h"

#include "esp_wifi_sta_pmksa_cache.h"
#include "esp_wifi_sta_pmksa_cache_i.h"
#include "rsn_supp/wpa.h"
#include "rsn_supp/wpa_i.h"
#include "rsn_supp/pmksa_cache.h"

/* Record awaiting installation on the next association attempt. */
static esp_wifi_sta_pmksa_cache_entry_t s_staged;
static bool s_staged_valid;
static os_time_t s_staged_expiration;
static os_time_t s_staged_reauth_time;

/* Lets a retry of the same association reuse the entry already installed. */
static struct {
    u8 bssid[ETH_ALEN];
    u8 spa[ETH_ALEN];
    u8 pmkid[PMKID_LEN];
    unsigned int akmp;
    bool valid;
} s_installed;

static void pmksa_staged_clear(void)
{
    forced_memzero(&s_staged, sizeof(s_staged));
    s_staged_valid = false;
    s_staged_expiration = 0;
    s_staged_reauth_time = 0;
}

/*
 * Drop the entry installed by this module, if it is still cached. Returns false
 * if that entry is in use, leaving it installed and still tracked.
 */
static bool pmksa_installed_try_remove(void)
{
    struct wpa_sm *sm = &gWpaSm;
    struct rsn_pmksa_cache_entry *entry;

    if (s_installed.valid && sm->pmksa != NULL) {
        /* pmksa_cache_get() cannot match on AKM, so check it here. */
        entry = pmksa_cache_get(sm->pmksa, s_installed.bssid, s_installed.spa,
                                s_installed.pmkid, NULL);
        if (entry != NULL && entry->akmp == (int)s_installed.akmp) {
            if (pmksa_cache_get_current(sm) == entry) {
                return false;
            }
            pmksa_cache_remove(sm->pmksa, entry);
        }
    }

    os_memset(&s_installed, 0, sizeof(s_installed));

    return true;
}

/* An individual, non-zero MAC address. */
static bool pmksa_addr_is_valid(const u8 *addr)
{
    static const u8 zero[ETH_ALEN] = { 0 };

    return (addr[0] & 0x01) == 0 && os_memcmp(addr, zero, ETH_ALEN) != 0;
}

/* Returns the matching WPA_KEY_MGMT_* value, or 0 if out of scope here. */
static unsigned int pmksa_akm_suite_to_akmp(uint32_t akm_suite)
{
    switch (akm_suite) {
    case ESP_WIFI_STA_PMKSA_AKM_802_1X:
        return WPA_KEY_MGMT_IEEE8021X;
    case ESP_WIFI_STA_PMKSA_AKM_802_1X_SHA256:
        return WPA_KEY_MGMT_IEEE8021X_SHA256;
    default:
        return 0;
    }
}

static uint32_t pmksa_akmp_to_akm_suite(int akmp)
{
    switch (akmp) {
    case WPA_KEY_MGMT_IEEE8021X:
        return ESP_WIFI_STA_PMKSA_AKM_802_1X;
    case WPA_KEY_MGMT_IEEE8021X_SHA256:
        return ESP_WIFI_STA_PMKSA_AKM_802_1X_SHA256;
    default:
        return 0;
    }
}

/* Seconds from now until t, clamped to [0, cap]. */
static uint32_t pmksa_remaining_s(os_time_t t, os_time_t now, uint32_t cap)
{
    os_time_t delta;

    if (t <= now) {
        return 0;
    }

    delta = t - now;

    return (delta > (os_time_t)cap) ? cap : (uint32_t)delta;
}

esp_err_t esp_wifi_sta_pmksa_cache_export(esp_wifi_sta_pmksa_cache_entry_t *entry)
{
    struct wpa_sm *sm = &gWpaSm;
    struct rsn_pmksa_cache_entry *current;
    struct os_reltime now;
    uint32_t akm_suite;
    uint32_t expiration_s;

    if (entry == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    os_memset(entry, 0, sizeof(*entry));

    current = pmksa_cache_get_current(sm);
    if (current == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    if (current->pmk_len != ESP_WIFI_STA_PMKSA_PMK_LEN) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    akm_suite = pmksa_akmp_to_akm_suite(current->akmp);
    if (akm_suite == 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Without both addresses the PMKID cannot be re-derived or re-bound. */
    if (!pmksa_addr_is_valid(current->aa) || !pmksa_addr_is_valid(current->spa)) {
        return ESP_ERR_INVALID_STATE;
    }

    os_get_reltime(&now);

    expiration_s = pmksa_remaining_s(current->expiration, now.sec,
                                     ESP_WIFI_STA_PMKSA_MAX_LIFETIME_S);
    if (expiration_s == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    os_memcpy(entry->bssid, current->aa, ETH_ALEN);
    os_memcpy(entry->sta_addr, current->spa, ETH_ALEN);
    os_memcpy(entry->pmkid, current->pmkid, PMKID_LEN);
    os_memcpy(entry->pmk, current->pmk, ESP_WIFI_STA_PMKSA_PMK_LEN);
    entry->pmk_len = ESP_WIFI_STA_PMKSA_PMK_LEN;
    entry->akm_suite = akm_suite;
    entry->expiration_remaining_s = expiration_s;
    entry->reauth_remaining_s = pmksa_remaining_s(current->reauth_time, now.sec,
                                                 expiration_s);

    return ESP_OK;
}

esp_err_t esp_wifi_sta_pmksa_cache_stage(const esp_wifi_sta_pmksa_cache_entry_t *entry)
{
    struct os_reltime now;

    if (entry == NULL ||
        entry->pmk_len != ESP_WIFI_STA_PMKSA_PMK_LEN ||
        pmksa_akm_suite_to_akmp(entry->akm_suite) == 0 ||
        !pmksa_addr_is_valid(entry->bssid) ||
        !pmksa_addr_is_valid(entry->sta_addr) ||
        entry->expiration_remaining_s == 0 ||
        entry->expiration_remaining_s > ESP_WIFI_STA_PMKSA_MAX_LIFETIME_S ||
        entry->reauth_remaining_s > entry->expiration_remaining_s) {
        return ESP_ERR_INVALID_ARG;
    }

    os_get_reltime(&now);

    pmksa_staged_clear();
    os_memcpy(&s_staged, entry, sizeof(s_staged));
    s_staged_expiration = now.sec + entry->expiration_remaining_s;
    s_staged_reauth_time = now.sec + entry->reauth_remaining_s;
    s_staged_valid = true;

    /*
     * Evict now rather than leaving it live until the replacement installs, and
     * stop tracking it either way so it cannot shadow the record just staged.
     */
    if (!pmksa_installed_try_remove()) {
        os_memset(&s_installed, 0, sizeof(s_installed));
    }

    return ESP_OK;
}

esp_err_t esp_wifi_sta_pmksa_cache_clear_external(void)
{
    pmksa_staged_clear();
    /* Keep tracking an in-use entry so a later call can still remove it. */
    (void)pmksa_installed_try_remove();

    return ESP_OK;
}

esp_err_t esp_wifi_sta_pmksa_cache_clear(void)
{
    struct wpa_sm *sm = &gWpaSm;

    pmksa_staged_clear();
    os_memset(&s_installed, 0, sizeof(s_installed));

    /* Flush all, not just ours: credentials may have changed. */
    if (sm->pmksa != NULL) {
        pmksa_cache_flush(sm->pmksa, NULL, NULL, 0);
        pmksa_cache_clear_current(sm);
    }

    return ESP_OK;
}

void esp_wifi_sta_pmksa_cache_deinit(void)
{
    /* The cache is being torn down, so only drop state held here. */
    pmksa_staged_clear();
    os_memset(&s_installed, 0, sizeof(s_installed));
}

bool esp_wifi_sta_pmksa_cache_install(struct wpa_sm *sm, const u8 *bssid)
{
    struct rsn_pmksa_cache_entry *entry;
    struct rsn_pmksa_cache_entry *existing;
    struct os_reltime now;
    unsigned int akmp;

    if (sm == NULL || bssid == NULL || sm->pmksa == NULL) {
        return false;
    }

    /* Retry of an association this module already installed an entry for. */
    if (s_installed.valid &&
        os_memcmp(s_installed.bssid, bssid, ETH_ALEN) == 0 &&
        os_memcmp(s_installed.spa, sm->own_addr, ETH_ALEN) == 0 &&
        s_installed.akmp == sm->key_mgmt) {
        /* Match on PMKID and AKM so only our installed entry is reused. */
        entry = pmksa_cache_get(sm->pmksa, bssid, sm->own_addr,
                                s_installed.pmkid, NULL);
        if (entry != NULL && entry->akmp == (int)s_installed.akmp) {
            return true;
        }
        os_memset(&s_installed, 0, sizeof(s_installed));
    }

    if (!s_staged_valid) {
        return false;
    }

    os_get_reltime(&now);

    if (s_staged_expiration <= now.sec) {
        pmksa_staged_clear();
        return false;
    }

    /* Another AP: keep it staged for the BSSID it was meant for. */
    if (os_memcmp(s_staged.bssid, bssid, ETH_ALEN) != 0) {
        return false;
    }

    /* A station or AKM mismatch is permanent, so drop the record. */
    akmp = pmksa_akm_suite_to_akmp(s_staged.akm_suite);
    if (akmp != sm->key_mgmt ||
        os_memcmp(s_staged.sta_addr, sm->own_addr, ETH_ALEN) != 0) {
        wpa_printf(MSG_DEBUG, "RSN: staged PMKSA does not match this station");
        pmksa_staged_clear();
        return false;
    }

    entry = os_zalloc(sizeof(*entry));
    if (entry == NULL) {
        return false;
    }

    os_memcpy(entry->aa, s_staged.bssid, ETH_ALEN);
    os_memcpy(entry->spa, sm->own_addr, ETH_ALEN);
    os_memcpy(entry->pmkid, s_staged.pmkid, PMKID_LEN);
    os_memcpy(entry->pmk, s_staged.pmk, ESP_WIFI_STA_PMKSA_PMK_LEN);
    entry->pmk_len = ESP_WIFI_STA_PMKSA_PMK_LEN;
    entry->akmp = (int)akmp;
    entry->network_ctx = sm->network_ctx;
    entry->expiration = s_staged_expiration;
    entry->reauth_time = s_staged_reauth_time;

    /*
     * pmksa_cache_add_entry() reuses an identical entry without updating its
     * lifetime or metadata. Remove only that case before adding this record.
     */
    existing = pmksa_cache_get(sm->pmksa, bssid, NULL, NULL, NULL);
    if (existing != NULL &&
        existing->pmk_len == entry->pmk_len &&
        os_memcmp_const(existing->pmk, entry->pmk, entry->pmk_len) == 0 &&
        os_memcmp_const(existing->pmkid, entry->pmkid, PMKID_LEN) == 0) {
        if (pmksa_cache_get_current(sm) == existing) {
            pmksa_cache_clear_current(sm);
        }
        pmksa_cache_remove(sm->pmksa, existing);
    }

    /* Takes ownership of entry. */
    if (pmksa_cache_add_entry(sm->pmksa, entry) == NULL) {
        pmksa_staged_clear();
        return false;
    }

    os_memcpy(s_installed.bssid, s_staged.bssid, ETH_ALEN);
    os_memcpy(s_installed.spa, sm->own_addr, ETH_ALEN);
    os_memcpy(s_installed.pmkid, s_staged.pmkid, PMKID_LEN);
    s_installed.akmp = akmp;
    s_installed.valid = true;

    pmksa_staged_clear();

    return true;
}
