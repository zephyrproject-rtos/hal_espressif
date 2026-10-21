/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ESP_WIFI_STA_PMKSA_CACHE_I_H
#define ESP_WIFI_STA_PMKSA_CACHE_I_H

#include <stdbool.h>
#include <stdint.h>

struct wpa_sm;

/*
 * Install staged records, the one for @p bssid last. Called from wpa_set_bss();
 * true enables PMKSA cache selection because an imported entry matches this
 * association.
 */
bool esp_wifi_sta_pmksa_cache_install(struct wpa_sm *sm, const uint8_t *bssid);

/*
 * Wipe staged records. Called from wpa_sm_deinit() so no key material outlives
 * the supplicant; native teardown frees the cache entries.
 */
void esp_wifi_sta_pmksa_cache_deinit(void);

#endif /* ESP_WIFI_STA_PMKSA_CACHE_I_H */
