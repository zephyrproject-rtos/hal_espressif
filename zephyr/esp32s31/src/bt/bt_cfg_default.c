/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_bt.h"

void esp_bt_controller_get_default_config(esp_bt_controller_config_t *cfg)
{
	*cfg = (esp_bt_controller_config_t)BT_CONTROLLER_INIT_CONFIG_DEFAULT();
}
