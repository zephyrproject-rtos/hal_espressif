/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/*******************************************************************************
 *
 *  Filename:      btc_av.h
 *
 *  Description:   Main API header file for all BTC AV functions accessed
 *                 from internal stack.
 *
 *******************************************************************************/

#ifndef __BTC_AV_H__
#define __BTC_AV_H__

#include "common/bt_target.h"
#include "esp_a2dp_api.h"
#include "btc/btc_task.h"
#include "btc/btc_common.h"
#include "btc/btc_sm.h"
#include "bta/bta_av_api.h"

#if (BTC_AV_INCLUDED == TRUE)

// global variable to indicate avrc is initialized with a2dp
extern bool g_av_with_rc;
// global variable to indicate a2dp is initialized
extern bool g_a2dp_on_init;
// global variable to indicate a2dp is deinitialized
extern bool g_a2dp_on_deinit;
// global variable to indicate a2dp source deinitialization is ongoing
extern bool g_a2dp_source_ongoing_deinit;
// global variable to indicate a2dp sink deinitialization is ongoing
extern bool g_a2dp_sink_ongoing_deinit;

/*******************************************************************************
**  Type definitions for callback functions
********************************************************************************/

enum {
    BTC_AV_DATAPATH_OPEN_EVT, // original UIPC_OPEN_EVT for data channel in bluedroid
    BTC_AV_DATAPATH_MAX_EVT,
};

typedef enum {
    BTC_AV_CONNECT_REQ_EVT = BTA_AV_MAX_EVT,
    BTC_AV_DISCONNECT_REQ_EVT,
    BTC_AV_START_STREAM_REQ_EVT,
    BTC_AV_STOP_STREAM_REQ_EVT,
    BTC_AV_SUSPEND_STREAM_REQ_EVT,
    BTC_AV_SINK_CONFIG_REQ_EVT,
} btc_av_sm_event_t;

typedef enum {
#if BTC_AV_SINK_INCLUDED
    BTC_AV_SINK_API_INIT_EVT = 0,
    BTC_AV_SINK_API_DEINIT_EVT,
    BTC_AV_SINK_API_CONNECT_EVT,
    BTC_AV_SINK_API_DISCONNECT_EVT,
    BTC_AV_SINK_API_REG_DATA_CB_EVT,
    BTC_AV_SINK_API_SET_DELAY_VALUE_EVT,
    BTC_AV_SINK_API_GET_DELAY_VALUE_EVT,
#endif  /* BTC_AV_SINK_INCLUDED */
#if BTC_AV_SRC_INCLUDED
    BTC_AV_SRC_API_INIT_EVT,
    BTC_AV_SRC_API_DEINIT_EVT,
    BTC_AV_SRC_API_CONNECT_EVT,
    BTC_AV_SRC_API_DISCONNECT_EVT,
    BTC_AV_SRC_API_REG_DATA_CB_EVT,
#endif  /* BTC_AV_SRC_INCLUDED */
    BTC_AV_API_MEDIA_CTRL_EVT,
} btc_av_act_t;

/* btc_av_args_t */
typedef union {
#if BTC_AV_SINK_INCLUDED
    // BTC_AV_SINK_CONFIG_REQ_EVT -- internal event
    esp_a2d_mcc_t mcc;
    // BTC_AV_SINK_API_CONNECT_EVT
    bt_bdaddr_t connect;
    // BTC_AV_SINK_API_DISCONNECT_EVT
    bt_bdaddr_t disconn;
    // BTC_AV_SINK_API_REG_DATA_CB_EVT
    esp_a2d_sink_data_cb_t data_cb;
    // BTC_AV_SINK_API_SET_DELAY_VALUE_EVT
    uint16_t delay_value;
#endif  /* BTC_AV_SINK_INCLUDED */
#if BTC_AV_SRC_INCLUDED
    // BTC_AV_SRC_API_REG_DATA_CB_EVT
    esp_a2d_source_data_cb_t src_data_cb;
    // BTC_AV_SRC_API_CONNECT
    bt_bdaddr_t src_connect;
    // BTC_AV_SRC_API_DISCONNECT_EVT
    bt_bdaddr_t src_disconn;
#endif /* BTC_AV_SRC_INCLUDED */
    // BTC_AV_API_MEDIA_CTRL_EVT
    esp_a2d_media_ctrl_t ctrl;
} btc_av_args_t;

/*******************************************************************************
**  BTC AV API
********************************************************************************/

void btc_a2dp_call_handler(btc_msg_t *msg);

void btc_a2dp_cb_handler(btc_msg_t *msg);

void btc_a2dp_get_profile_status(esp_a2d_profile_status_t *param);

void btc_a2dp_sink_reg_data_cb(esp_a2d_sink_data_cb_t callback);

void btc_a2dp_src_reg_data_cb(esp_a2d_source_data_cb_t callback);
/*******************************************************************************
**
** Function         btc_av_get_sm_handle
**
** Description      Fetches current av SM handle
**
** Returns          None
**
*******************************************************************************/

btc_sm_handle_t btc_av_get_sm_handle(void);

/*******************************************************************************
**
** Function         btc_av_stream_ready
**
** Description      Checks whether AV is ready for starting a stream
**
** Returns          None
**
*******************************************************************************/

BOOLEAN btc_av_stream_ready(void);

/*******************************************************************************
**
** Function         btc_av_stream_started_ready
**
** Description      Checks whether AV ready for media start in streaming state
**
** Returns          None
**
*******************************************************************************/

BOOLEAN btc_av_stream_started_ready(void);

/*******************************************************************************
**
** Function         btc_dispatch_sm_event
**
** Description      Send event to AV statemachine
**
** Returns          None
**
*******************************************************************************/

/* used to pass events to AV statemachine from other tasks */
void btc_dispatch_sm_event(btc_av_sm_event_t event, void *p_data, int len);

/*******************************************************************************
**
** Function         btc_av_is_connected
**
** Description      Checks if av has a connected sink
**
** Returns          BOOLEAN
**
*******************************************************************************/

BOOLEAN btc_av_is_connected(void);


/*******************************************************************************
 *
 * Function         btc_av_get_peer_sep
 *
 * Description      Get the stream endpoint type.
 *
 * Returns          The stream endpoint type: either AVDT_TSEP_SRC or
 *                  AVDT_TSEP_SNK.
 *
 ******************************************************************************/

uint8_t btc_av_get_peer_sep(void);

/*******************************************************************************
**
** Function         btc_av_is_peer_edr
**
** Description      Check if the connected a2dp device supports
**                  EDR or not. Only when connected this function
**                  will accurately provide a true capability of
**                  remote peer. If not connected it will always be false.
**
** Returns          TRUE if remote device is capable of EDR
**
*******************************************************************************/

BOOLEAN btc_av_is_peer_edr(void);

/******************************************************************************
**
** Function         btc_av_clear_remote_suspend_flag
**
** Description      Clears remote suspended flag
**
** Returns          Void
********************************************************************************/
void btc_av_clear_remote_suspend_flag(void);

/*******************************************************************************
 *
 * Function         btc_av_get_service_id
 *
 * Description      Get the current AV service ID.
 *
 * Returns          The stream endpoint type: either BTA_A2DP_SOURCE_SERVICE_ID or
 *                  BTA_A2DP_SINK_SERVICE_ID.
 *
 ******************************************************************************/
uint8_t btc_av_get_service_id(void);

#endif  ///BTC_AV_INCLUDED == TRUE

#endif /* __BTC_AV_H__ */
