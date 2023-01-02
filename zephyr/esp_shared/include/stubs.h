/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
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

#include <zephyr/device.h>

#define ESP_SOC_DEFAULT_CPU_FREQ_MHZ (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000)
#define ESP_SOC_DEFAULT_RTC_CLK_CAL_CYCLES 1024

/* Extract configuration from the devicetree */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay) &&		\
	DT_PROP_BY_IDX(DT_NODELABEL(uart0), reg, 0) ==		\
	DT_PROP_BY_IDX(DT_CHOSEN(zephyr_console), reg, 0)
#define CONFIG_ESP_CONSOLE_UART 1
#define CONFIG_ESP_CONSOLE_UART_NUM 0
#define CONFIG_ESP_CONSOLE_UART_BAUDRATE DT_PROP(DT_NODELABEL(uart0), current_speed)

#elif DT_NODE_HAS_STATUS(DT_NODELABEL(uart1), okay) &&		\
	DT_PROP_BY_IDX(DT_NODELABEL(uart1), reg, 0) ==		\
	DT_PROP_BY_IDX(DT_CHOSEN(zephyr_console), reg, 0)
#define CONFIG_ESP_CONSOLE_UART 1
#define CONFIG_ESP_CONSOLE_UART_NUM 1
#define CONFIG_ESP_CONSOLE_UART_BAUDRATE DT_PROP(DT_NODELABEL(uart1), current_speed)

#elif DT_NODE_HAS_STATUS(DT_NODELABEL(uart2), okay) &&		\
	DT_PROP_BY_IDX(DT_NODELABEL(uart2), reg, 0) ==		\
	DT_PROP_BY_IDX(DT_CHOSEN(zephyr_console), reg, 0)
#define CONFIG_ESP_CONSOLE_UART 1
#define CONFIG_ESP_CONSOLE_UART_NUM 2
#define CONFIG_ESP_CONSOLE_UART_BAUDRATE DT_PROP(DT_NODELABEL(uart2), current_speed)

#else

#define CONFIG_ESP_CONSOLE_UART_NUM -1

#endif

#endif /* _STUBS_H_ */
