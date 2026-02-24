/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_system.h"
#include "esp_private/system_internal.h"
#include <stdint.h>
#include "esp_cpu.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "esp_private/rtc_clk.h"
#include "esp_private/panic_internal.h"
#include "esp_private/spi_flash_os.h"
#include "esp_rom_serial_output.h"
#include "esp_rom_sys.h"
#include "sdkconfig.h"

#define SHUTDOWN_HANDLERS_NO 5

static shutdown_handler_t shutdown_handlers[SHUTDOWN_HANDLERS_NO];

esp_err_t esp_register_shutdown_handler(shutdown_handler_t handler)
{
	for (int i = 0; i < SHUTDOWN_HANDLERS_NO; i++) {
		if (shutdown_handlers[i] == handler) {
			return ESP_ERR_INVALID_STATE;
		} else if (shutdown_handlers[i] == NULL) {
			shutdown_handlers[i] = handler;
			return ESP_OK;
		}
	}
	return ESP_ERR_NO_MEM;
}

esp_err_t esp_unregister_shutdown_handler(shutdown_handler_t handler)
{
	for (int i = 0; i < SHUTDOWN_HANDLERS_NO; i++) {
		if (shutdown_handlers[i] == handler) {
			shutdown_handlers[i] = NULL;
			return ESP_OK;
		}
	}
	return ESP_ERR_INVALID_STATE;
}

/*
 * esp_restart_noos_dig() is used only by ESP32 panic handler.
 * It generates a digital domain reset which does not reset RTC domain registers.
 */
#ifdef CONFIG_SOC_SERIES_ESP32
void IRAM_ATTR esp_restart_noos_dig(void)
{
	/*
	 * In case any of the calls below results in re-enabling of interrupts
	 * (for example, by entering a critical section), disable all the
	 * interrupts (e.g. from watchdogs) here.
	 */
	xt_ints_off(0xFFFFFFFF);

	/* Make sure all the panic handler output is sent from UART FIFO */
	if (CONFIG_ESP_CONSOLE_UART_NUM >= 0) {
		esp_rom_output_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
	}

#if SOC_MEMSPI_CLOCK_IS_INDEPENDENT
	spi_flash_set_clock_src(MSPI_CLK_SRC_ROM_DEFAULT);
#endif

	/* Switch to XTAL (otherwise we will keep running from the PLL) */
	rtc_clk_cpu_set_to_default_config();

	/*
	 * esp_restart_noos_dig() generates a core reset, which does not reset the
	 * registers of the RTC domain, so the CPU's stall state remains after the reset,
	 * we need to release them here.
	 */
#if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
	int core_id = esp_cpu_get_core_id();

	for (uint32_t i = 0; i < SOC_CPU_CORES_NUM; i++) {
		if (i != core_id) {
			esp_cpu_unstall(i);
		}
	}
#endif
	esp_rom_software_reset_system();

	while (true) {
		;
	}
}
#endif /* CONFIG_SOC_SERIES_ESP32 */

void esp_restart(void)
{
	for (int i = SHUTDOWN_HANDLERS_NO - 1; i >= 0; i--) {
		if (shutdown_handlers[i]) {
			shutdown_handlers[i]();
		}
	}

	esp_restart_noos();
}
