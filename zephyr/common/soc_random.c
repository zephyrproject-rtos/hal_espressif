/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Build upstream bootloader_random_<soc>.c with the bootloader LL path
 * (SAR driver not initialized yet) for the application, Simple Boot,
 * and MCUboot.
 */
#ifndef BOOTLOADER_BUILD
#define BOOTLOADER_BUILD 1
#endif

#if defined(CONFIG_SOC_SERIES_ESP32)
#include "../../components/bootloader_support/src/bootloader_random_esp32.c"
#elif defined(CONFIG_SOC_SERIES_ESP32C2)
#include "../../components/bootloader_support/src/bootloader_random_esp32c2.c"
#elif defined(CONFIG_SOC_SERIES_ESP32C3)
#include "../../components/bootloader_support/src/bootloader_random_esp32c3.c"
#elif defined(CONFIG_SOC_SERIES_ESP32C5)
#include "../../components/bootloader_support/src/bootloader_random_esp32c5.c"
#elif defined(CONFIG_SOC_SERIES_ESP32C6)
#include "../../components/bootloader_support/src/bootloader_random_esp32c6.c"
#elif defined(CONFIG_SOC_SERIES_ESP32C61)
#include "../../components/bootloader_support/src/bootloader_random_esp32c61.c"
#elif defined(CONFIG_SOC_SERIES_ESP32H2)
#include "../../components/bootloader_support/src/bootloader_random_esp32h2.c"
#elif defined(CONFIG_SOC_SERIES_ESP32P4)
#include "../../components/bootloader_support/src/bootloader_random_esp32p4.c"
#elif defined(CONFIG_SOC_SERIES_ESP32S2)
#include "../../components/bootloader_support/src/bootloader_random_esp32s2.c"
#elif defined(CONFIG_SOC_SERIES_ESP32S3)
#include "../../components/bootloader_support/src/bootloader_random_esp32s3.c"
#else
#error "No bootloader_random source for this SoC series"
#endif
