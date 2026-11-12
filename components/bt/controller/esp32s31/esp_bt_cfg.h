/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ESP_BT_CFG_H__
#define __ESP_BT_CFG_H__

#ifdef __cplusplus
extern "C" {
#endif

#define DEFAULT_BT_CTRL_HCI_CMD_NUM         (5)

#define DEFAULT_BT_CTRL_BLE_ENABLE          (1)

#if defined(CONFIG_BT_CLASSIC)
#define DEFAULT_BT_CTRL_BREDR_ENABLE        (1)
#define DEFAULT_BT_CTRL_MULTI_LINK_ENABLED  (1)

#define DEFAULT_BT_CTRL_BR_EDR_ACLU_TX_BUF_NB           (4)
#define DEFAULT_BT_CTRL_BR_EDR_ACLU_RX_BUF_NB           (6)

#if CONFIG_ESP32_BT_CTLR_BR_EDR_MAX_SYNC_CONN == 1
#define DEFAULT_BT_CTRL_BR_EDR_SYNC_TX_BUF_NB           (4)
#define DEFAULT_BT_CTRL_BR_EDR_SYNC_RX_BUF_NB           (4)
#elif CONFIG_ESP32_BT_CTLR_BR_EDR_MAX_SYNC_CONN == 2
#define DEFAULT_BT_CTRL_BR_EDR_SYNC_TX_BUF_NB           (6)
#define DEFAULT_BT_CTRL_BR_EDR_SYNC_RX_BUF_NB           (8)
#elif CONFIG_ESP32_BT_CTLR_BR_EDR_MAX_SYNC_CONN == 3
#define DEFAULT_BT_CTRL_BR_EDR_SYNC_TX_BUF_NB           (8)
#define DEFAULT_BT_CTRL_BR_EDR_SYNC_RX_BUF_NB           (12)
#endif

#define DEFAULT_BT_CTRL_BR_EDR_ESCO_EV3_2_SUPP          (1)
#define DEFAULT_BT_CTRL_BR_EDR_CCA_RSSI_THRESH          (20)
#define DEFAULT_BT_CTRL_BR_EDR_TX_PWR_ACL_MIN           (-13)
#define DEFAULT_BT_CTRL_BR_EDR_TX_PWR_ACL_MAX           (4)
#define DEFAULT_BT_CTRL_BR_EDR_TX_PWR_PAGE              (4)
#define DEFAULT_BT_CTRL_BR_EDR_TX_PWR_PSCAN             (4)
#define DEFAULT_BT_CTRL_BR_EDR_TX_PWR_ISCAN             (4)
#define DEFAULT_BT_CTRL_BR_EDR_LEGACY_AUTH_VENDOR_EVT   (1)
#else
#define DEFAULT_BT_CTRL_BREDR_ENABLE        (0)
#define DEFAULT_BT_CTRL_MULTI_LINK_ENABLED  (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ESP_BT_CFG_H__ */
