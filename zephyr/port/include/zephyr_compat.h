/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Compatibility header for translating Zephyr Kconfig options to ESP-IDF Kconfig options.
 */

#ifndef ZEPHYR_COMPAT_H
#define ZEPHYR_COMPAT_H


#ifdef CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM
#define CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM
#endif

#ifdef CONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM
#define CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM CONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM
#endif

#ifdef CONFIG_ESP32_WIFI_TX_BUFFER
#define CONFIG_ESP_WIFI_TX_BUFFER CONFIG_ESP32_WIFI_TX_BUFFER
#endif

#ifdef CONFIG_ESP32_WIFI_TX_BUFFER_TYPE
#define CONFIG_ESP_WIFI_TX_BUFFER_TYPE CONFIG_ESP32_WIFI_TX_BUFFER_TYPE
#endif

#ifdef CONFIG_ESP32_WIFI_STATIC_TX_BUFFER_NUM
#define CONFIG_ESP_WIFI_STATIC_TX_BUFFER_NUM CONFIG_ESP32_WIFI_STATIC_TX_BUFFER_NUM
#endif

#ifdef CONFIG_ESP32_WIFI_CACHE_TX_BUFFER_NUM
#define CONFIG_ESP_WIFI_CACHE_TX_BUFFER_NUM CONFIG_ESP32_WIFI_CACHE_TX_BUFFER_NUM
#endif

#ifdef CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM
#define CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM
#endif

#ifdef CONFIG_ESP32_WIFI_CSI_ENABLED
#define CONFIG_ESP_WIFI_CSI_ENABLED CONFIG_ESP32_WIFI_CSI_ENABLED
#endif

#ifdef CONFIG_ESP32_WIFI_AMPDU_TX_ENABLED
#define CONFIG_ESP_WIFI_AMPDU_TX_ENABLED CONFIG_ESP32_WIFI_AMPDU_TX_ENABLED
#endif

#ifdef CONFIG_ESP32_WIFI_TX_BA_WIN
#define CONFIG_ESP_WIFI_TX_BA_WIN CONFIG_ESP32_WIFI_TX_BA_WIN
#endif

#ifdef CONFIG_ESP32_WIFI_AMPDU_RX_ENABLED
#define CONFIG_ESP_WIFI_AMPDU_RX_ENABLED CONFIG_ESP32_WIFI_AMPDU_RX_ENABLED
#endif

#ifdef CONFIG_ESP32_WIFI_RX_BA_WIN
#define CONFIG_ESP_WIFI_RX_BA_WIN CONFIG_ESP32_WIFI_RX_BA_WIN
#endif

#ifdef CONFIG_ESP32_WIFI_AMSDU_TX_ENABLED
#define CONFIG_ESP_WIFI_AMSDU_TX_ENABLED CONFIG_ESP32_WIFI_AMSDU_TX_ENABLED
#endif

#ifdef CONFIG_ESP32_WIFI_IRAM_OPT
#define CONFIG_ESP_WIFI_IRAM_OPT CONFIG_ESP32_WIFI_IRAM_OPT
#endif

#ifdef CONFIG_ESP32_WIFI_RX_IRAM_OPT
#define CONFIG_ESP_WIFI_RX_IRAM_OPT CONFIG_ESP32_WIFI_RX_IRAM_OPT
#endif

#ifdef CONFIG_ESP32_WIFI_FTM_ENABLE
#define CONFIG_ESP_WIFI_FTM_ENABLE CONFIG_ESP32_WIFI_FTM_ENABLE
#endif

#ifdef CONFIG_ESP32_WIFI_FTM_INITIATOR_SUPPORT
#define CONFIG_ESP_WIFI_FTM_INITIATOR_SUPPORT CONFIG_ESP32_WIFI_FTM_INITIATOR_SUPPORT
#endif

#ifdef CONFIG_ESP32_WIFI_FTM_RESPONDER_SUPPORT
#define CONFIG_ESP_WIFI_FTM_RESPONDER_SUPPORT CONFIG_ESP32_WIFI_FTM_RESPONDER_SUPPORT
#endif

#ifdef CONFIG_ESP32_WIFI_SOFTAP_SUPPORT
#define CONFIG_ESP_WIFI_SOFTAP_SUPPORT CONFIG_ESP32_WIFI_SOFTAP_SUPPORT
#endif

#ifdef CONFIG_ESP32_WIFI_ENABLE_WPA3_SAE
#define CONFIG_ESP_WIFI_ENABLE_WPA3_SAE CONFIG_ESP32_WIFI_ENABLE_WPA3_SAE
#endif

#ifdef CONFIG_ESP32_PHY_MAX_TX_POWER
#define CONFIG_ESP_PHY_MAX_TX_POWER CONFIG_ESP32_PHY_MAX_TX_POWER
#endif

#ifdef CONFIG_ESP32_WIFI_SLP_DEFAULT_MIN_ACTIVE_TIME
#define CONFIG_ESP_WIFI_SLP_DEFAULT_MIN_ACTIVE_TIME CONFIG_ESP32_WIFI_SLP_DEFAULT_MIN_ACTIVE_TIME
#endif

#ifdef CONFIG_ESP32_WIFI_SLP_DEFAULT_MAX_ACTIVE_TIME
#define CONFIG_ESP_WIFI_SLP_DEFAULT_MAX_ACTIVE_TIME CONFIG_ESP32_WIFI_SLP_DEFAULT_MAX_ACTIVE_TIME
#endif

#ifdef CONFIG_ESP32_WIFI_SLP_DEFAULT_WAIT_BROADCAST_DATA_TIME
#define CONFIG_ESP_WIFI_SLP_DEFAULT_WAIT_BROADCAST_DATA_TIME CONFIG_ESP32_WIFI_SLP_DEFAULT_WAIT_BROADCAST_DATA_TIME
#endif

#ifdef CONFIG_ESP32_WIFI_DYNAMIC_RX_MGMT_BUF
#define CONFIG_ESP_WIFI_DYNAMIC_RX_MGMT_BUF CONFIG_ESP32_WIFI_DYNAMIC_RX_MGMT_BUF
#endif

#ifdef CONFIG_ESP32_WIFI_RX_MGMT_BUF_NUM_DEF
#define CONFIG_ESP_WIFI_RX_MGMT_BUF_NUM_DEF CONFIG_ESP32_WIFI_RX_MGMT_BUF_NUM_DEF
#endif

#ifdef CONFIG_ESP32_WIFI_MGMT_SBUF_NUM
#define CONFIG_ESP_WIFI_MGMT_SBUF_NUM CONFIG_ESP32_WIFI_MGMT_SBUF_NUM
#endif

#ifdef CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
#define CONFIG_SW_COEXIST_ENABLE CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
#endif

#endif /* ZEPHYR_COMPAT_H */
