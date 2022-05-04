/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#if defined(CONFIG_SOC_ESP32)
#include "soc/dport_reg.h"
#elif defined(CONFIG_SOC_ESP32S2)
#include "esp_attr.h"
#include "esp32s2/rom/cache.h"
#elif defined(CONFIG_SOC_ESP32C3)
#include "soc/soc.h"
#include "esp32c3/rom/cache.h"
#endif

#include <zephyr.h>

void IRAM_ATTR esp32_spiflash_start(void);

void IRAM_ATTR esp32_spiflash_end(void);

