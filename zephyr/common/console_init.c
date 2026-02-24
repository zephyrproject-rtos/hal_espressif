/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "console_init.h"
#include "hal/uart_periph.h"
#include "hal/uart_ll.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_periph.h"
#include "soc/gpio_sig_map.h"
#include "hal/clk_gate_ll.h"
#include "hal/gpio_hal.h"
#include "soc/rtc.h"
#if CONFIG_SOC_SERIES_ESP32S2
#include "esp32s2/rom/usb/cdc_acm.h"
#include "esp32s2/rom/usb/usb_common.h"
#include "esp32s2/rom/usb/usb_persist.h"
#endif
#include "esp_rom_gpio.h"
#include "esp_rom_serial_output.h"
#include "esp_rom_sys.h"
#include "esp_rom_caps.h"

void esp_console_deinit(void)
{
#ifdef CONFIG_ESP_CONSOLE_UART
	/* Ensure any buffered log output is displayed */
	esp_rom_output_flush_tx(CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM);
#endif /* CONFIG_ESP_CONSOLE_UART */
}

#ifdef CONFIG_ESP_CONSOLE_UART
void esp_console_init(void)
{
	esp_rom_install_uart_printf();
	esp_rom_output_set_as_console(CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM);

	/* Set configured UART console baud rate
	 * Use rtc_clk_apb_freq_get() instead of esp_clk_apb_freq() because during
	 * early boot the g_ticks_per_us_pro ROM variable may not be set correctly yet.
	 */
	uint32_t clock_hz = rtc_clk_apb_freq_get();
#if ESP_ROM_UART_CLK_IS_XTAL
	/* From ESP32-S3 on, UART clk source is selected to XTAL in ROM */
	clock_hz = (uint32_t)rtc_clk_xtal_freq_get() * 1000000UL;
#endif
	esp_rom_output_tx_wait_idle(CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM);
	_uart_ll_set_baudrate(UART_LL_GET_HW(CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM),
			      CONFIG_ESP_CONSOLE_UART_BAUDRATE, clock_hz);
}
#endif /* CONFIG_ESP_CONSOLE_UART */

#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
void esp_console_init(void)
{
	/* Wait for any pending UART output to complete before switching */
	esp_rom_output_tx_wait_idle(0);

	/* Switch ROM buffer to USB Serial JTAG */
	esp_rom_output_switch_buffer(CONFIG_ESP_ROM_USB_SERIAL_DEVICE_NUM);

	/* Configure ROM printf to use USB Serial JTAG for output */
	esp_rom_output_set_as_console(CONFIG_ESP_ROM_USB_SERIAL_DEVICE_NUM);

	/* Disable ROM channel 2 to avoid double output (channel 2 also goes to USB) */
	esp_rom_install_channel_putc(2, NULL);
}
#endif /* CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG */
