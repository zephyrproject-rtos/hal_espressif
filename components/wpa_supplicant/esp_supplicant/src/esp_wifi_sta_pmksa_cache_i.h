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
 * Install a staged PMKSA, or report one installed on an earlier attempt. Called
 * from wpa_set_bss(); true enables PMKSA cache selection for this association.
 */
bool esp_wifi_sta_pmksa_cache_install(struct wpa_sm *sm, const uint8_t *bssid);

/*
 * Wipe the staged record and drop tracking. Called from wpa_sm_deinit() so no
 * key material outlives the supplicant.
 */
void esp_wifi_sta_pmksa_cache_deinit(void);

#endif /* ESP_WIFI_STA_PMKSA_CACHE_I_H */
