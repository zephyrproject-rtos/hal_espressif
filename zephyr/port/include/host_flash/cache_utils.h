/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#if defined(CONFIG_SOC_SERIES_ESP32)
#include "soc/dport_reg.h"
#include "esp_attr.h"
#elif defined(CONFIG_SOC_SERIES_ESP32S2)
#include "esp_attr.h"
#include "esp32s2/rom/cache.h"
#elif defined(CONFIG_SOC_SERIES_ESP32S3)
#include "esp_attr.h"
#include "esp32s3/rom/cache.h"
#elif defined(CONFIG_SOC_SERIES_ESP32C3)
#include "soc/soc.h"
#include "esp32c3/rom/cache.h"
#endif

#include <zephyr/kernel.h>
#include <soc.h>
#include "esp_err.h"

void IRAM_ATTR esp32_spiflash_start(void);

void IRAM_ATTR esp32_spiflash_end(void);

//config cache mode
#if !CONFIG_IDF_TARGET_ESP32
//config instrcutin cache size and cache block size by menuconfig
void esp_config_instruction_cache_mode(void);
//config data cache size and cache block size by menuconfig
void esp_config_data_cache_mode(void);
//enable cache wrap mode for instruction cache and data cache
esp_err_t esp_enable_cache_wrap(bool icache_wrap_enable, bool dcache_wrap_enable);
#endif
