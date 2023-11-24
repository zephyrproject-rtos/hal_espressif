/*
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mcuboot_config/mcuboot_logging.h>
#include <esp_rom_sys.h>

#define SOC_LOGE(tag, fmt, ...) MCUBOOT_LOG_ERR("[%s] " fmt, tag, ##__VA_ARGS__)
#define SOC_LOGW(tag, fmt, ...) MCUBOOT_LOG_WRN("[%s] " fmt, tag, ##__VA_ARGS__)
#define SOC_LOGI(tag, fmt, ...) MCUBOOT_LOG_INF("[%s] " fmt, tag, ##__VA_ARGS__)
#define SOC_LOGD(tag, fmt, ...) MCUBOOT_LOG_DBG("[%s] " fmt, tag, ##__VA_ARGS__)
