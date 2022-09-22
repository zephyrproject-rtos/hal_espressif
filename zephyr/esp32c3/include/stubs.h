/*
 * Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
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

#define ESP_SOC_DEFAULT_CPU_FREQ_MHZ (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000)

#if !defined(CONFIG_ESP_CONSOLE_UART_BAUDRATE) && defined(CONFIG_ESP_CONSOLE_UART)
#define CONFIG_ESP_CONSOLE_UART_BAUDRATE                \
        ((CONFIG_ESP_CONSOLE_UART_NUM)?                 \
         DT_PROP(DT_NODELABEL(uart1), current_speed) :  \
         DT_PROP(DT_NODELABEL(uart0), current_speed))
#endif

#endif /* _STUBS_H_ */
