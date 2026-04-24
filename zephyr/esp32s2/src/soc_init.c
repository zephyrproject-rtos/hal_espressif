/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <inttypes.h>
#include "soc_init.h"
#include "esp32s2/rom/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include "soc/dport_reg.h"
#include "soc/assist_debug_reg.h"
#include "soc/reset_reasons.h"
#include "soc/dport_access.h"
#include "esp_log.h"

static const char *TAG = "soc_init";

void soc_hw_init(void)
{
#if CONFIG_ESP_HAL_EARLY_LOG_LEVEL == 0
	rtc_suppress_rom_log();
#endif
}

void ana_reset_config(void)
{
	/* ESP32-S2 does not have FIB_SEL register, analog reset config not supported */
}

void super_wdt_auto_feed(void)
{
	REG_SET_BIT(RTC_CNTL_SWD_CONF_REG, RTC_CNTL_SWD_AUTO_FEED_EN);
}

static void wdt_reset_info_dump(int cpu)
{
	uint32_t inst = 0, pid = 0, stat = 0, data = 0, pc = 0,
		 lsstat = 0, lsaddr = 0, lsdata = 0, dstat = 0;
	const char *cpu_name = cpu ? "APP" : "PRO";

	stat = 0xdeadbeef;
	pid = 0;
	inst = REG_READ(ASSIST_DEBUG_PRO_RCD_PDEBUGINST);
	dstat = REG_READ(ASSIST_DEBUG_PRO_RCD_PDEBUGSTATUS);
	data = REG_READ(ASSIST_DEBUG_PRO_RCD_PDEBUGDATA);
	pc = REG_READ(ASSIST_DEBUG_PRO_RCD_PDEBUGPC);
	lsstat = REG_READ(ASSIST_DEBUG_PRO_RCD_PDEBUGLS0STAT);
	lsaddr = REG_READ(ASSIST_DEBUG_PRO_RCD_PDEBUGLS0ADDR);
	lsdata = REG_READ(ASSIST_DEBUG_PRO_RCD_PDEBUGLS0DATA);

	if (DPORT_RECORD_PDEBUGINST_SZ(inst) == 0 &&
	    DPORT_RECORD_PDEBUGSTATUS_BBCAUSE(dstat) == DPORT_RECORD_PDEBUGSTATUS_BBCAUSE_WAITI) {
		ESP_EARLY_LOGW(TAG, "WDT reset info: %s CPU PC=0x%08" PRIx32 " (waiti mode)",
			       cpu_name, pc);
	} else {
		ESP_EARLY_LOGW(TAG, "WDT reset info: %s CPU PC=0x%08" PRIx32, cpu_name, pc);
	}
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU STATUS        0x%08" PRIx32, cpu_name, stat);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PID           0x%08" PRIx32, cpu_name, pid);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGINST    0x%08" PRIx32, cpu_name, inst);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGSTATUS  0x%08" PRIx32, cpu_name, dstat);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGDATA    0x%08" PRIx32, cpu_name, data);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGPC      0x%08" PRIx32, cpu_name, pc);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGLS0STAT 0x%08" PRIx32, cpu_name, lsstat);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGLS0ADDR 0x%08" PRIx32, cpu_name, lsaddr);
	ESP_EARLY_LOGD(TAG, "WDT reset info: %s CPU PDEBUGLS0DATA 0x%08" PRIx32, cpu_name, lsdata);
}

void wdt_reset_cpu0_info_enable(void)
{
	DPORT_REG_SET_BIT(DPORT_PERI_CLK_EN_REG, DPORT_PERI_EN_ASSIST_DEBUG);
	DPORT_REG_CLR_BIT(DPORT_PERI_RST_EN_REG, DPORT_PERI_EN_ASSIST_DEBUG);
	REG_WRITE(ASSIST_DEBUG_PRO_PDEBUGENABLE, 1);
	REG_WRITE(ASSIST_DEBUG_PRO_RCD_RECORDING, 1);
}

void check_wdt_reset(void)
{
	int wdt_rst = 0;
	soc_reset_reason_t rst_reas;

	rst_reas = esp_rom_get_reset_reason(0);
	if (rst_reas == RESET_REASON_CORE_RTC_WDT || rst_reas == RESET_REASON_CORE_MWDT0 ||
	    rst_reas == RESET_REASON_CORE_MWDT1 || rst_reas == RESET_REASON_CPU0_MWDT0 ||
	    rst_reas == RESET_REASON_CPU0_MWDT1 || rst_reas == RESET_REASON_CPU0_RTC_WDT) {
		ESP_EARLY_LOGW(TAG, "PRO CPU has been reset by WDT.");
		wdt_rst = 1;
	}

	if (wdt_rst) {
		wdt_reset_info_dump(0);
	}

	wdt_reset_cpu0_info_enable();
}

/* Not supported but common bootloader calls the function. Do nothing */
void ana_clock_glitch_reset_config(bool enable)
{
	(void)enable;
}

#if defined(CONFIG_ESP_SIMPLE_BOOT)
#include "soc/rtc.h"
#include "hal/regi2c_ctrl_ll.h"
#include "hal/clk_tree_ll.h"
#include "esp_rom_sys.h"
#include "esp_rom_serial_output.h"

/*
 * Custom bootloader_clock_configure() for ESP32-S2 simple boot mode.
 * In simple boot mode, the ROM bootloader has already configured clocks
 * appropriately for flash access. We only initialize the essential REGI2C
 * blocks needed for later peripheral initialization without changing CPU
 * frequency, which would affect flash timing and cause crashes.
 */
void bootloader_clock_configure(void)
{
	esp_rom_output_tx_wait_idle(0);

	/*
	 * After a WDT reset, the CPU may still be running from PLL.
	 * Switch to XTAL so the actual APB frequency matches the value
	 * used by esp_console_init() for UART baud rate computation.
	 */
	if (clk_ll_cpu_get_src() == SOC_CPU_CLK_SRC_PLL) {
		clk_ll_cpu_set_src(SOC_CPU_CLK_SRC_XTAL);
	}

	/* Set tuning parameters for RC_SLOW and RC_FAST clocks (matches ESP-IDF rtc_clk_init) */
	REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_SCK_DCAP, RTC_CNTL_SCK_DCAP_DEFAULT);
	REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_DFREQ, RTC_CNTL_CK8M_DFREQ_DEFAULT);

	regi2c_ctrl_ll_i2c_reset();
	regi2c_ctrl_ll_i2c_bbpll_enable();
	regi2c_ctrl_ll_i2c_apll_enable();

	/* Keep RC_FAST running so RNG has an entropy source during boot */
	rtc_clk_8m_enable(true, false);
	rtc_clk_fast_src_set(SOC_RTC_FAST_CLK_SRC_RC_FAST);

	REG_WRITE(RTC_CNTL_INT_ENA_REG, 0);
	REG_WRITE(RTC_CNTL_INT_CLR_REG, UINT32_MAX);
}
#endif /* CONFIG_ESP_SIMPLE_BOOT */
