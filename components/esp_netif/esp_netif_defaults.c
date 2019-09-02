// Copyright 2015-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "esp_netif.h"
#include "esp_wifi_default.h"
#include "esp_eth.h"

//
// Purpose of this module is to provide
//  - general esp-netif definitions of default objects for STA, AP, ETH
//  - default init / create functions for basic default interfaces
//

#define IP4TOUINT32(a,b,c,d) (((uint32_t)((a) & 0xffU) << 24) | \
                               ((uint32_t)((b) & 0xffU) << 16) | \
                               ((uint32_t)((c) & 0xffU) << 8)  | \
                                (uint32_t)((d) & 0xffU))

#define IP4TOADDR(a,b,c,d) esp_netif_htonl(IP4TOUINT32(a, b, c, d))


//
// Default configuration of common interfaces, such as STA, AP, ETH
//
const esp_netif_inherent_config_t _g_esp_netif_inherent_sta_config = {
        .flags = ESP_NETIF_DHCPC | ESP_NETIF_FLAG_GARP | ESP_NETIF_FLAG_EVENT_IP_MODIFIED,
        .lost_ip_event = IP_EVENT_STA_LOST_IP,
        .get_ip_event = IP_EVENT_STA_GOT_IP,
        .if_key = "WIFI_STA_DEF",
        .if_type = ESP_NETIF_TYPE_STA,
        .route_prio = 100
};

static const esp_netif_ip_info_t soft_ap_ip = {
        .ip = { .addr = IP4TOADDR( 192, 168, 4, 1) },
        .gw = { .addr = IP4TOADDR( 192, 168, 4, 1) },
        .netmask = { .addr = IP4TOADDR( 255, 255, 255, 0) },

};

const esp_netif_inherent_config_t _g_esp_netif_inherent_ap_config = {
        .flags = ESP_NETIF_DHCPS | ESP_NETIF_FLAG_AUTOUP,
        .ip_info = (esp_netif_ip_info_t*)&soft_ap_ip,
        .if_key = "WIFI_AP_DEF",
        .if_type = ESP_NETIF_TYPE_AP,
        .route_prio = 10
};

const esp_netif_inherent_config_t _g_esp_netif_inherent_eth_config = {
        .get_ip_event = IP_EVENT_ETH_GOT_IP,
        .lost_ip_event = 0,
        .flags = ESP_NETIF_DHCPC | ESP_NETIF_FLAG_GARP | ESP_NETIF_FLAG_EVENT_IP_MODIFIED,
        .if_key = "ETH_DEF",
        .if_type = ESP_NETIF_TYPE_ETH,
        .route_prio = 50
};

esp_netif_t* esp_netif_create_default_wifi_ap(void)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_WIFI_AP();
    esp_netif_t *netif = esp_netif_new(&cfg);
    assert(netif);
    esp_wifi_set_default_wifi_ap_handlers(netif);
    return netif;
}

esp_netif_t* esp_netif_create_default_wifi_sta(void)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_WIFI_STA();
    esp_netif_t *netif = esp_netif_new(&cfg);
    assert(netif);
    esp_wifi_set_default_wifi_sta_handlers(netif);
    return netif;
}

esp_netif_t* esp_netif_create_default_eth(void * eth_driver)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t* eth_netif = esp_netif_new(&cfg);
    assert(eth_netif);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, eth_driver));
    return eth_netif;
}