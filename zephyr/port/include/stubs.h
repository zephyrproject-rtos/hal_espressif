/*
 * Copyright (c) 2021-2025 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STUBS_H_
#define _STUBS_H_

/* Required for C99 compilation (required for GCC-8.x version,
 * where typeof is used instead of __typeof__)
 */
#ifndef typeof
#define typeof  __typeof__
#endif

#include <zephyr/devicetree.h>
#include <esp_rom_caps.h>

#if defined(CONFIG_SOC_SERIES_ESP32)
#define DT_CPU_COMPAT espressif_xtensa_lx6
#elif defined(CONFIG_SOC_SERIES_ESP32S2) || defined(CONFIG_SOC_SERIES_ESP32S3)
#define DT_CPU_COMPAT espressif_xtensa_lx7
#elif defined(CONFIG_SOC_SERIES_ESP32C2) || defined(CONFIG_SOC_SERIES_ESP32C3) || \
	  defined(CONFIG_SOC_SERIES_ESP32C6) || defined(CONFIG_SOC_SERIES_ESP32H2)
#define DT_CPU_COMPAT espressif_riscv
#endif

#define CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ ((DT_PROP(DT_INST(0, DT_CPU_COMPAT), clock_frequency)) / 1000000)
#define CONFIG_XTAL_FREQ (DT_PROP(DT_INST(0, DT_CPU_COMPAT), xtal_freq) / 1000000)

#if defined(CONFIG_SOC_SERIES_ESP32)
#define CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ ESP_SOC_DEFAULT_CPU_FREQ_MHZ
#elif defined(CONFIG_SOC_SERIES_ESP32S2)
#define CONFIG_ESP32S2_DEFAULT_CPU_FREQ_MHZ ESP_SOC_DEFAULT_CPU_FREQ_MHZ
#elif defined(CONFIG_SOC_SERIES_ESP32S3)
#define CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ ESP_SOC_DEFAULT_CPU_FREQ_MHZ
#elif defined(CONFIG_SOC_SERIES_ESP32C2)
#define CONFIG_ESP32C2_DEFAULT_CPU_FREQ_MHZ ESP_SOC_DEFAULT_CPU_FREQ_MHZ
#elif defined(CONFIG_SOC_SERIES_ESP32C3)
#define CONFIG_ESP32C3_DEFAULT_CPU_FREQ_MHZ ESP_SOC_DEFAULT_CPU_FREQ_MHZ
#elif defined(CONFIG_SOC_SERIES_ESP32C6)
#define CONFIG_ESP32C6_DEFAULT_CPU_FREQ_MHZ ESP_SOC_DEFAULT_CPU_FREQ_MHZ
#elif defined(CONFIG_SOC_SERIES_ESP32H2)
#define CONFIG_ESP32H2_DEFAULT_CPU_FREQ_MHZ ESP_SOC_DEFAULT_CPU_FREQ_MHZ
#endif

#endif /* _STUBS_H_ */
