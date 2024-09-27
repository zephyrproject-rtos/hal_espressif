/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>

void print_banner(void);
int read_bootloader_header(void);
int check_bootloader_validity(void);
void config_wdt(void);


#if defined(CONFIG_SOC_SERIES_ESP32) || defined (CONFIG_SOC_SERIES_ESP32S3)
void wdt_reset_info_dump(int cpu);
#endif

void wdt_reset_cpu0_info_enable(void);

#if defined(CONFIG_SOC_SERIES_ESP32)
void reset_mmu(void);
#endif

#if !defined(CONFIG_SOC_SERIES_ESP32)
void super_wdt_auto_feed(void);
#endif


void check_wdt_reset(void);

void soc_hw_init(void);

void ana_super_wdt_reset_config(bool enable);

void ana_bod_reset_config(bool enable);

void ana_clock_glitch_reset_config(bool enable);

void ana_reset_config(void);
