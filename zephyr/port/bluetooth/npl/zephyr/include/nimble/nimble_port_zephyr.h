/*
 * SPDX-FileCopyrightText: 2015-2022 The Apache Software Foundation (ASF)
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPDX-FileContributor: 2019-2022 Espressif Systems (Shanghai) CO LTD
 */

#ifndef _NIMBLE_PORT_ZEPHYR_H
#define _NIMBLE_PORT_ZEPHYR_H

#include "nimble/nimble_npl.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief esp_nimble_enable - Initialize the NimBLE host task
 *
 * @param host_task
 * @return esp_err_t
 */
esp_err_t esp_nimble_enable(void *host_task);

/**
 * @brief esp_nimble_disable - Disable the NimBLE host task
 *
 * @return esp_err_t
 */
esp_err_t esp_nimble_disable(void);

void nimble_port_zephyr_init(struct k_thread);
void nimble_port_zephyr_deinit(void);
void npl_zephyr_funcs_init(void);
void npl_zephyr_funcs_deinit(void);
int npl_zephyr_mempool_init(void);
struct npl_funcs_t * npl_zephyr_funcs_get(void);
int npl_zephyr_set_controller_npl_info(ble_npl_count_info_t *ctrl_npl_info);
#ifdef __cplusplus
}
#endif

#endif /* _NIMBLE_PORT_FREERTOS_H */
