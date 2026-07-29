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
#include "utils/eloop.h"

extern bool current_task_is_wifi_task(void);

struct pmksa_staged_entry {
    esp_wifi_sta_pmksa_cache_entry_t record;
    os_time_t expiration;
    os_time_t reauth_time;
};

/* Records awaiting installation. Only accessed from the Wi-Fi task. */
static struct pmksa_staged_entry s_staged[ESP_WIFI_STA_PMKSA_MAX_ENTRIES];
static size_t s_staged_count;

static void pmksa_staged_clear(void)
{
    forced_memzero(s_staged, sizeof(s_staged));
    s_staged_count = 0;
}

static void pmksa_staged_remove(size_t index)
{
    os_memmove(&s_staged[index], &s_staged[index + 1],
               (s_staged_count - index - 1) * sizeof(s_staged[0]));
    s_staged_count--;
    forced_memzero(&s_staged[s_staged_count], sizeof(s_staged[0]));
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

static bool pmksa_record_is_valid(const esp_wifi_sta_pmksa_cache_entry_t *record)
{
    return record->pmk_len == ESP_WIFI_STA_PMKSA_PMK_LEN &&
           pmksa_akm_suite_to_akmp(record->akm_suite) != 0 &&
           pmksa_addr_is_valid(record->bssid) &&
           pmksa_addr_is_valid(record->sta_addr) &&
           record->expiration_remaining_s != 0 &&
           record->expiration_remaining_s <= ESP_WIFI_STA_PMKSA_MAX_LIFETIME_S &&
           record->reauth_remaining_s <= record->expiration_remaining_s;
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

/* Remove idle imported entries; removing the one in use would deauthenticate. */
static void pmksa_flush_idle_external(struct wpa_sm *sm)
{
    struct rsn_pmksa_cache_entry *entry;
    struct rsn_pmksa_cache_entry *next;

    for (entry = pmksa_cache_head(sm->pmksa); entry != NULL; entry = next) {
        next = entry->next;
        if (entry->external && entry != pmksa_cache_get_current(sm)) {
            pmksa_cache_remove(sm->pmksa, entry);
        }
    }
}

struct pmksa_call {
    esp_err_t (*op)(void *arg);
    void *arg;
    esp_err_t result;
};

static int pmksa_call_handler(void *eloop_ctx, void *user_ctx)
{
    struct pmksa_call *call = user_ctx;

    (void)eloop_ctx;
    call->result = call->op(call->arg);
    return 0;
}

/* Run op in the Wi-Fi task, which owns the cache. */
static esp_err_t pmksa_run(esp_err_t (*op)(void *arg), void *arg)
{
    struct pmksa_call call = {
        .op = op,
        .arg = arg,
        .result = ESP_FAIL,
    };

    if (current_task_is_wifi_task()) {
        return op(arg);
    }

    if (eloop_register_timeout_blocking(pmksa_call_handler, NULL, &call) < 0) {
        return ESP_FAIL;
    }

    return call.result;
}

struct pmksa_stage_args {
    const esp_wifi_sta_pmksa_cache_entry_t *entries;
    size_t count;
    size_t staged_count;
};

static void pmksa_stage_record(const esp_wifi_sta_pmksa_cache_entry_t *record, os_time_t now)
{
    struct pmksa_staged_entry *slot = NULL;
    size_t i;

    for (i = 0; i < s_staged_count; i++) {
        if (os_memcmp(s_staged[i].record.bssid, record->bssid, ETH_ALEN) == 0) {
            slot = &s_staged[i];
            break;
        }
    }

    if (slot == NULL) {
        if (s_staged_count == ESP_WIFI_STA_PMKSA_MAX_ENTRIES) {
            return;
        }
        slot = &s_staged[s_staged_count++];
    }

    os_memcpy(&slot->record, record, sizeof(*record));
    slot->expiration = now + record->expiration_remaining_s;
    slot->reauth_time = now + record->reauth_remaining_s;
}

static esp_err_t pmksa_stage_op(void *arg)
{
    struct pmksa_stage_args *args = arg;
    struct wpa_sm *sm = &gWpaSm;
    struct os_reltime now;
    size_t i;

    os_get_reltime(&now);

    pmksa_staged_clear();
    if (sm->pmksa != NULL) {
        pmksa_flush_idle_external(sm);
    }

    for (i = 0; i < args->count; i++) {
        if (pmksa_record_is_valid(&args->entries[i])) {
            pmksa_stage_record(&args->entries[i], now.sec);
        }
    }
    args->staged_count = s_staged_count;

    return ESP_OK;
}

esp_err_t esp_wifi_sta_pmksa_cache_stage(const esp_wifi_sta_pmksa_cache_entry_t *entries,
                                         size_t count, size_t *staged_count)
{
    struct pmksa_stage_args args = {
        .entries = entries,
        .count = count,
    };
    esp_err_t err;

    if (staged_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *staged_count = 0;

    if (entries == NULL && count != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    err = pmksa_run(pmksa_stage_op, &args);
    if (err == ESP_OK) {
        *staged_count = args.staged_count;
    }

    return err;
}

struct pmksa_export_args {
    size_t index;
    esp_wifi_sta_pmksa_cache_entry_t *entry;
    size_t entry_count;
};

static bool pmksa_entry_is_exportable(const struct wpa_sm *sm,
                                      const struct rsn_pmksa_cache_entry *entry,
                                      os_time_t now)
{
    return entry->network_ctx == sm->network_ctx &&
           os_memcmp(entry->spa, sm->own_addr, ETH_ALEN) == 0 &&
           entry->pmk_len == ESP_WIFI_STA_PMKSA_PMK_LEN &&
           pmksa_akmp_to_akm_suite(entry->akmp) != 0 &&
           pmksa_addr_is_valid(entry->aa) &&
           pmksa_addr_is_valid(entry->spa) &&
           entry->expiration > now;
}

static esp_err_t pmksa_export_op(void *arg)
{
    struct pmksa_export_args *args = arg;
    struct wpa_sm *sm = &gWpaSm;
    const struct rsn_pmksa_cache_entry *entry;
    esp_wifi_sta_pmksa_cache_entry_t *out = args->entry;
    struct os_reltime now;
    size_t count = 0;

    if (sm->pmksa == NULL || sm->network_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    os_get_reltime(&now);

    for (entry = pmksa_cache_head(sm->pmksa); entry != NULL; entry = entry->next) {
        if (!pmksa_entry_is_exportable(sm, entry, now.sec)) {
            continue;
        }

        if (count == args->index) {
            os_memcpy(out->bssid, entry->aa, ETH_ALEN);
            os_memcpy(out->sta_addr, entry->spa, ETH_ALEN);
            os_memcpy(out->pmkid, entry->pmkid, PMKID_LEN);
            os_memcpy(out->pmk, entry->pmk, ESP_WIFI_STA_PMKSA_PMK_LEN);
            out->pmk_len = ESP_WIFI_STA_PMKSA_PMK_LEN;
            out->akm_suite = pmksa_akmp_to_akm_suite(entry->akmp);
            out->expiration_remaining_s =
                pmksa_remaining_s(entry->expiration, now.sec, ESP_WIFI_STA_PMKSA_MAX_LIFETIME_S);
            out->reauth_remaining_s =
                pmksa_remaining_s(entry->reauth_time, now.sec, out->expiration_remaining_s);
        }
        count++;
    }
    args->entry_count = count;

    return args->index < count ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t esp_wifi_sta_pmksa_cache_export(size_t index, esp_wifi_sta_pmksa_cache_entry_t *entry,
                                          size_t *entry_count)
{
    struct pmksa_export_args args = {
        .index = index,
        .entry = entry,
    };
    esp_err_t err;

    if (entry == NULL || entry_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    os_memset(entry, 0, sizeof(*entry));
    *entry_count = 0;

    err = pmksa_run(pmksa_export_op, &args);
    if (err == ESP_OK || err == ESP_ERR_NOT_FOUND) {
        *entry_count = args.entry_count;
    } else {
        forced_memzero(entry, sizeof(*entry));
    }

    return err;
}

static esp_err_t pmksa_clear_op(void *arg)
{
    bool external_only = *(bool *)arg;
    struct wpa_sm *sm = &gWpaSm;

    pmksa_staged_clear();
    if (sm->pmksa == NULL) {
        return ESP_OK;
    }

    if (external_only) {
        pmksa_flush_idle_external(sm);
    } else {
        pmksa_cache_flush(sm->pmksa, NULL, NULL, 0);
        pmksa_cache_clear_current(sm);
    }

    return ESP_OK;
}

esp_err_t esp_wifi_sta_pmksa_cache_clear(bool external_only)
{
    return pmksa_run(pmksa_clear_op, &external_only);
}

void esp_wifi_sta_pmksa_cache_deinit(void)
{
    pmksa_staged_clear();
}

/*
 * Replace any idle entry for the record's BSSID with the import. Returns false
 * if the existing entry is in use or allocation fails; the record stays staged.
 */
static bool pmksa_install_record(struct wpa_sm *sm, const struct pmksa_staged_entry *staged)
{
    struct rsn_pmksa_cache_entry *existing;
    struct rsn_pmksa_cache_entry *entry;

    existing = pmksa_cache_get(sm->pmksa, staged->record.bssid, NULL, NULL, NULL);
    if (existing != NULL && existing == pmksa_cache_get_current(sm)) {
        return false;
    }

    entry = os_zalloc(sizeof(*entry));
    if (entry == NULL) {
        return false;
    }

    os_memcpy(entry->aa, staged->record.bssid, ETH_ALEN);
    os_memcpy(entry->spa, sm->own_addr, ETH_ALEN);
    os_memcpy(entry->pmkid, staged->record.pmkid, PMKID_LEN);
    os_memcpy(entry->pmk, staged->record.pmk, ESP_WIFI_STA_PMKSA_PMK_LEN);
    entry->pmk_len = ESP_WIFI_STA_PMKSA_PMK_LEN;
    entry->akmp = (int)pmksa_akm_suite_to_akmp(staged->record.akm_suite);
    entry->network_ctx = sm->network_ctx;
    entry->expiration = staged->expiration;
    entry->reauth_time = staged->reauth_time;
    entry->external = true;

    /*
     * pmksa_cache_add_entry() would keep an identical entry without its new
     * metadata, or flush other entries sharing the replaced PMK.
     */
    if (existing != NULL) {
        pmksa_cache_remove(sm->pmksa, existing);
    }

    /* Takes ownership of entry. */
    pmksa_cache_add_entry(sm->pmksa, entry);

    return true;
}

bool esp_wifi_sta_pmksa_cache_install(struct wpa_sm *sm, const u8 *bssid)
{
    struct rsn_pmksa_cache_entry *entry;
    struct os_reltime now;
    unsigned int pass;
    size_t i;

    if (sm == NULL || bssid == NULL || sm->pmksa == NULL) {
        return false;
    }

    os_get_reltime(&now);

    /* Install the target last so evicting the oldest entry cannot drop it. */
    for (pass = 0; pass < 2; pass++) {
        for (i = 0; i < s_staged_count;) {
            struct pmksa_staged_entry *staged = &s_staged[i];
            bool target = os_memcmp(staged->record.bssid, bssid, ETH_ALEN) == 0;

            if (target != (pass == 1)) {
                i++;
                continue;
            }

            if (staged->expiration <= now.sec ||
                os_memcmp(staged->record.sta_addr, sm->own_addr, ETH_ALEN) != 0) {
                pmksa_staged_remove(i);
                continue;
            }

            if (!pmksa_install_record(sm, staged)) {
                i++;
                continue;
            }

            pmksa_staged_remove(i);
        }
    }

    entry = pmksa_cache_get(sm->pmksa, bssid, sm->own_addr, NULL, sm->network_ctx);

    return entry != NULL && entry->external && entry->akmp == (int)sm->key_mgmt &&
           entry->expiration > now.sec;
}
