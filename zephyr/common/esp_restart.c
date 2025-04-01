/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "esp_system.h"
#include "esp_private/system_internal.h"
#include <stdint.h>
#include "esp_cpu.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"
#include "esp_private/rtc_clk.h"
#include "esp_private/panic_internal.h"
#include "esp_private/system_internal.h"
#include "esp_private/spi_flash_os.h"
#include "esp_rom_uart.h"
#include "esp_rom_sys.h"
#include "sdkconfig.h"

#if CONFIG_ESP_SYSTEM_MEMPROT_FEATURE
#if CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/memprot.h"
#else
#include "esp_memprot.h"
#endif
#endif

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

void IRAM_ATTR esp_restart_noos_dig(void)
{
	/* In case any of the calls below results in re-enabling of interrupts
	 * (for example, by entering a critical section), disable all the
	 * interrupts (e.g. from watchdogs) here.
	 */
#ifdef CONFIG_IDF_TARGET_ARCH_RISCV
	rv_utils_intr_global_disable();
#else
	xt_ints_off(0xFFFFFFFF);
#endif

	/* make sure all the panic handler output is sent from UART FIFO */
	if (CONFIG_ESP_CONSOLE_UART_NUM >= 0) {
		esp_rom_uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
	}

#if SOC_MEMSPI_CLOCK_IS_INDEPENDENT
	spi_flash_set_clock_src(MSPI_CLK_SRC_ROM_DEFAULT);
#endif

	/* switch to XTAL (otherwise we will keep running from the PLL) */
	rtc_clk_cpu_set_to_default_config();

	/* esp_restart_noos_dig() will generates a core reset, which does not reset the
	 * registers of the RTC domain, so the CPU's stall state remains after the reset,
	 * we need to release them here
	 */
#if !CONFIG_FREERTOS_UNICORE
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

void esp_restart(void)
{
	for (int i = SHUTDOWN_HANDLERS_NO - 1; i >= 0; i--) {
		if (shutdown_handlers[i]) {
			shutdown_handlers[i]();
		}
	}

	k_sched_lock();

	bool digital_reset_needed = false;
#if CONFIG_ESP_SYSTEM_MEMPROT_FEATURE
#if CONFIG_IDF_TARGET_ESP32S2
	if (esp_memprot_is_intr_ena_any() || esp_memprot_is_locked_any()) {
		digital_reset_needed = true;
	}
#else
	bool is_on = false;
	if (esp_mprot_is_intr_ena_any(&is_on) != ESP_OK || is_on) {
		digital_reset_needed = true;
	} else if (esp_mprot_is_conf_locked_any(&is_on) != ESP_OK || is_on) {
		digital_reset_needed = true;
	}
#endif
#endif
	if (digital_reset_needed) {
		esp_restart_noos_dig();
	}
	esp_restart_noos();
}
