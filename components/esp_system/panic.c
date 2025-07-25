/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_attr.h"

#include "esp_private/system_internal.h"
#include "esp_private/usb_console.h"

#include "esp_cpu.h"
#include "soc/rtc.h"
#include "hal/timer_hal.h"
#include "hal/wdt_types.h"
#include "hal/wdt_hal.h"
#include "hal/mwdt_ll.h"
#include "esp_private/esp_int_wdt.h"

#include "esp_private/panic_internal.h"
#include "port/panic_funcs.h"
#include "esp_rom_sys.h"

#include "sdkconfig.h"

#if __has_include("esp_app_desc.h")
#define WITH_ELF_SHA256
#include "esp_app_desc.h"
#endif

#if CONFIG_ESP_COREDUMP_ENABLE
#include "esp_core_dump.h"
#endif

#if CONFIG_APPTRACE_ENABLE
#include "esp_app_trace.h"
#if CONFIG_APPTRACE_SV_ENABLE
#include "SEGGER_RTT.h"
#endif

#if CONFIG_APPTRACE_ONPANIC_HOST_FLUSH_TMO == -1
#define APPTRACE_ONPANIC_HOST_FLUSH_TMO   ESP_APPTRACE_TMO_INFINITE
#else
#define APPTRACE_ONPANIC_HOST_FLUSH_TMO   (1000*CONFIG_APPTRACE_ONPANIC_HOST_FLUSH_TMO)
#endif
#endif // CONFIG_APPTRACE_ENABLE

#if !CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT
#include "hal/uart_hal.h"
#endif

#if CONFIG_ESP_SYSTEM_PANIC_GDBSTUB
#include "esp_gdbstub.h"
#endif

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
#include "hal/usb_serial_jtag_ll.h"
#endif

#ifdef __XTENSA__
#include "xtensa/semihosting.h"
#elif __riscv
#include "riscv/semihosting.h"
#endif

#define ESP_SEMIHOSTING_SYS_PANIC_REASON	0x116

#define MWDT_DEFAULT_TICKS_PER_US       500

#define PANIC_ENTRY_COUNT_MAX 2 // We allow at least 2 panic entries to let the panic handler process Double Exceptions

bool g_panic_abort = false;
char *g_panic_abort_details = NULL;

static wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();

static uint32_t DRAM_ATTR g_panic_entry_count[portNUM_PROCESSORS] = {0}; // Number of times panic handler has been entered per core since multiple cores can enter the panic handler simultaneously

#if !CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT

/********************** Panic print functions **********************/

#if CONFIG_ESP_CONSOLE_UART
static uart_hal_context_t s_panic_uart = { .dev = CONFIG_ESP_CONSOLE_UART_NUM == 0 ? &UART0 :&UART1 };

static void panic_print_char_uart(const char c)
{
    uint32_t sz = 0;
    while (!uart_hal_get_txfifo_len(&s_panic_uart));
    uart_hal_write_txfifo(&s_panic_uart, (uint8_t *) &c, 1, &sz);
}
#endif // CONFIG_ESP_CONSOLE_UART


#if CONFIG_ESP_CONSOLE_USB_CDC
static void panic_print_char_usb_cdc(const char c)
{
    esp_usb_console_write_buf(&c, 1);
    /* result ignored */
}
#endif // CONFIG_ESP_CONSOLE_USB_CDC

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
//Timeout; if there's no host listening, the txfifo won't ever
//be writable after the first packet.

#define USBSERIAL_TIMEOUT_MAX_US 50000
static int s_usbserial_timeout = 0;

static void panic_print_char_usb_serial_jtag(const char c)
{
    while (!usb_serial_jtag_ll_txfifo_writable() && s_usbserial_timeout < (USBSERIAL_TIMEOUT_MAX_US / 100)) {
        esp_rom_delay_us(100);
        s_usbserial_timeout++;
    }
    if (usb_serial_jtag_ll_txfifo_writable()) {
        usb_serial_jtag_ll_write_txfifo((const uint8_t *)&c, 1);
        s_usbserial_timeout = 0;
    }
}
#endif //CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG


void panic_print_char(const char c)
{
#if CONFIG_ESP_CONSOLE_UART
    panic_print_char_uart(c);
#endif
#if CONFIG_ESP_CONSOLE_USB_CDC
    panic_print_char_usb_cdc(c);
#endif
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
    panic_print_char_usb_serial_jtag(c);
#endif
}

void panic_print_str(const char *str)
{
    for (int i = 0; str[i] != 0; i++) {
        panic_print_char(str[i]);
    }
}

void panic_print_hex(int h)
{
    int x;
    int c;
    // Does not print '0x', only the digits (8 digits to print)
    for (x = 0; x < 8; x++) {
        c = (h >> 28) & 0xf; // extract the leftmost byte
        if (c < 10) {
            panic_print_char('0' + c);
        } else {
            panic_print_char('a' + c - 10);
        }
        h <<= 4; // move the 2nd leftmost byte to the left, to be extracted next
    }
}

void panic_print_dec(int d)
{
    // can print at most 2 digits!
    int n1, n2;
    n1 = d % 10; // extract ones digit
    n2 = d / 10; // extract tens digit
    if (n2 == 0) {
        panic_print_char(' ');
    } else {
        panic_print_char(n2 + '0');
    }
    panic_print_char(n1 + '0');
}
#endif  // CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT

static void print_abort_details(const void *f)
{
    panic_print_str(g_panic_abort_details);
}

/********************** Panic handler watchdog timer functions **********************/

/* This function disables the Timer Group WDTs */
void esp_panic_handler_disable_timg_wdts(void)
{
    wdt_hal_context_t wdt0_context = {.inst = WDT_MWDT0, .mwdt_dev = &TIMERG0};
    wdt_hal_write_protect_disable(&wdt0_context);
    wdt_hal_disable(&wdt0_context);
    wdt_hal_write_protect_enable(&wdt0_context);

#if SOC_TIMER_GROUPS >= 2
    wdt_hal_context_t wdt1_context = {.inst = WDT_MWDT1, .mwdt_dev = &TIMERG1};
    wdt_hal_write_protect_disable(&wdt1_context);
    wdt_hal_disable(&wdt1_context);
    wdt_hal_write_protect_enable(&wdt1_context);
#endif /* SOC_TIMER_GROUPS >= 2 */
}

/* This function enables the RTC WDT with the given timeout in milliseconds */
void esp_panic_handler_enable_rtc_wdt(uint32_t timeout_ms)
{
    wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT(); // Use a local context variable to avoid race conditions when both cores enter the panic handler
    wdt_hal_init(&rtc_wdt_ctx, WDT_RWDT, 0, false);
    uint32_t stage_timeout_ticks = (uint32_t)(timeout_ms * rtc_clk_slow_freq_get_hz() / 1000ULL);
    wdt_hal_write_protect_disable(&rtc_wdt_ctx);
    wdt_hal_config_stage(&rtc_wdt_ctx, WDT_STAGE0, stage_timeout_ticks, WDT_STAGE_ACTION_RESET_RTC);
    wdt_hal_enable(&rtc_wdt_ctx);
    wdt_hal_write_protect_enable(&rtc_wdt_ctx);
}

/* Feed the watchdogs if they are enabled and if we are not already in the panic handler */
void esp_panic_handler_feed_wdts(void)
{
    /* If we have already entered the panic handler multiple times,
     * we should not feed the WDTs. This is because we need an
     * alternate mechanism to reset the system if we happen to be stuck
     * in a panic loop.
     */
    if (g_panic_entry_count[esp_cpu_get_core_id()] > PANIC_ENTRY_COUNT_MAX) {
        return;
    }

    // Feed Timer Group 0 WDT
    wdt_hal_context_t wdt0_context = {.inst = WDT_MWDT0, .mwdt_dev = &TIMERG0};
    if (wdt_hal_is_enabled(&wdt0_context)) {
        wdt_hal_write_protect_disable(&wdt0_context);
        wdt_hal_feed(&wdt0_context);
        wdt_hal_write_protect_enable(&wdt0_context);
    }

#if SOC_TIMER_GROUPS >= 2
    // Feed Timer Group 1 WDT
    wdt_hal_context_t wdt1_context = {.inst = WDT_MWDT1, .mwdt_dev = &TIMERG1};
    if (wdt_hal_is_enabled(&wdt1_context)) {
        wdt_hal_write_protect_disable(&wdt1_context);
        wdt_hal_feed(&wdt1_context);
        wdt_hal_write_protect_enable(&wdt1_context);
    }
#endif /* SOC_TIMER_GROUPS >= 2 */

    // Feed RTC WDT
    if (wdt_hal_is_enabled(&rtc_wdt_ctx)) {
        wdt_hal_write_protect_disable(&rtc_wdt_ctx);
        wdt_hal_feed(&rtc_wdt_ctx);
        wdt_hal_write_protect_enable(&rtc_wdt_ctx);
    }
}

/* This function disables all the watchdogs */
static inline void disable_all_wdts(void)
{
    //Disable Timer Group WDTs
    esp_panic_handler_disable_timg_wdts();

    //Disable RTC WDT
    wdt_hal_write_protect_disable(&rtc_wdt_ctx);
    wdt_hal_disable(&rtc_wdt_ctx);
    wdt_hal_write_protect_enable(&rtc_wdt_ctx);
}

/********************** Panic handler functions **********************/

/* This function is called from the panic handler entry point to increment the panic entry count */
void esp_panic_handler_increment_entry_count(void)
{
    int core_id = esp_cpu_get_core_id();

    g_panic_entry_count[core_id]++;
    if (g_panic_entry_count[core_id] > PANIC_ENTRY_COUNT_MAX) {
        /* If we have already panicked multiple times, chances are
         * that the panic handler itself is broken. In this case, we
         * should just reset the system.
         */
        panic_print_str("Panic handler entered multiple times. Abort panic handling. Rebooting ...\r\n");
        panic_restart();
    }
}

// Control arrives from chip-specific panic handler, environment prepared for
// the 'main' logic of panic handling. This means that chip-specific stuff have
// already been done, and panic_info_t has been filled.
void esp_panic_handler(panic_info_t *info)
{
    // The port-level panic handler has already called this, but call it again
    // to reset the RTC WDT period
    esp_panic_handler_feed_wdts();

    // If the exception was due to an abort, override some of the panic info
    if (g_panic_abort) {
        info->description = NULL;
        info->details = g_panic_abort_details ? print_abort_details : NULL;
        info->reason = NULL;
        info->exception = PANIC_EXCEPTION_ABORT;
    }

    /*
     * For any supported chip, the panic handler prints the contents of panic_info_t in the following format:
     *
     *
     * Guru Meditation Error: Core <core> (<exception>). <description>
     * <details>
     *
     * <state>
     *
     * <elf_info>
     *
     *
     * ----------------------------------------------------------------------------------------
     * core - core where exception was triggered
     * exception - what kind of exception occurred
     * description - a short description regarding the exception that occurred
     * details - more details about the exception
     * state - processor state like register contents, and backtrace
     * elf_info - details about the image currently running
     *
     * NULL fields in panic_info_t are not printed.
     *
     */
    if (info->reason) {
        panic_print_str("Guru Meditation Error: Core ");
        panic_print_dec(info->core);
        panic_print_str(" panic'ed (");
        panic_print_str(info->reason);
        panic_print_str("). ");
    }

    if (info->description) {
        panic_print_str(info->description);
    }

    panic_print_str("\r\n");

    PANIC_INFO_DUMP(info, details);

    panic_print_str("\r\n");

    // If on-chip-debugger is attached, and system is configured to be aware of this,
    // then only print up to details. Users should be able to probe for the other information
    // in debug mode.
#if CONFIG_ESP_DEBUG_OCDAWARE
    if (esp_cpu_dbgr_is_attached()) {
        char *panic_reason_str = NULL;
        if (info->pseudo_excause) {
            panic_reason_str = (char *)info->reason;
        } else if (g_panic_abort) {
            panic_reason_str = g_panic_abort_details;
        }
        if (panic_reason_str) {
            /* OpenOCD will print the halt cause when target is stopped at the below breakpoint (info->addr) */
            long args[] = {(long)panic_reason_str, strlen(panic_reason_str)};
            semihosting_call_noerrno(ESP_SEMIHOSTING_SYS_PANIC_REASON, args);
        }
        panic_print_str("Setting breakpoint at 0x");
        panic_print_hex((uint32_t)info->addr);
        panic_print_str(" and returning...\r\n");
#if CONFIG_APPTRACE_ENABLE
#if CONFIG_APPTRACE_SV_ENABLE
        SEGGER_RTT_ESP_FlushNoLock(CONFIG_APPTRACE_POSTMORTEM_FLUSH_THRESH, APPTRACE_ONPANIC_HOST_FLUSH_TMO);
#else
        esp_apptrace_flush_nolock(ESP_APPTRACE_DEST_TRAX, CONFIG_APPTRACE_POSTMORTEM_FLUSH_THRESH,
                                  APPTRACE_ONPANIC_HOST_FLUSH_TMO);
#endif
#endif

        disable_all_wdts();
        esp_cpu_set_breakpoint(0, info->addr); // use breakpoint 0
        return;
    }
#endif //CONFIG_ESP_DEBUG_OCDAWARE

    /* Feed the WDTs here. This is done to fascilitate a "slow" UART
     * which might take a longer time to print the state of the processor.
     */
    esp_panic_handler_feed_wdts();

    PANIC_INFO_DUMP(info, state);
    panic_print_str("\r\n");

    /* No matter if we come here from abort or an exception, this variable must be reset.
     * Else, any exception/error occurring during the current panic handler would considered
     * an abort. Do this after PANIC_INFO_DUMP(info, state) as it also checks this variable.
     * For example, if coredump triggers a stack overflow and this variable is not reset,
     * the second panic would be still be marked as the result of an abort, even the previous
     * message reason would be kept. */
    g_panic_abort = false;

#ifdef WITH_ELF_SHA256
    panic_print_str("\r\nELF file SHA256: ");
    char sha256_buf[65];
    esp_app_get_elf_sha256(sha256_buf, sizeof(sha256_buf));
    panic_print_str(sha256_buf);
    panic_print_str("\r\n");
#endif

    panic_print_str("\r\n");

#if CONFIG_APPTRACE_ENABLE
    esp_panic_handler_feed_wdts();
#if CONFIG_APPTRACE_SV_ENABLE
    SEGGER_RTT_ESP_FlushNoLock(CONFIG_APPTRACE_POSTMORTEM_FLUSH_THRESH, APPTRACE_ONPANIC_HOST_FLUSH_TMO);
#else
    esp_apptrace_flush_nolock(ESP_APPTRACE_DEST_TRAX, CONFIG_APPTRACE_POSTMORTEM_FLUSH_THRESH,
                              APPTRACE_ONPANIC_HOST_FLUSH_TMO);
#endif
#endif // CONFIG_APPTRACE_ENABLE

#if CONFIG_ESP_COREDUMP_ENABLE
    esp_panic_handler_feed_wdts();
    static bool s_dumping_core = false;
    if (s_dumping_core) {
        panic_print_str("Re-entered core dump! Exception happened during core dump!\r\n");
    } else {
        s_dumping_core = true;
        esp_core_dump_write(info);
        s_dumping_core = false;
    }
#endif /* CONFIG_ESP_COREDUMP_ENABLE */

#if CONFIG_ESP_SYSTEM_PANIC_GDBSTUB
    panic_print_str("Entering gdb stub now.\r\n");
    disable_all_wdts();
    esp_gdbstub_panic_handler((void *)info->frame);
#else
#if CONFIG_ESP_SYSTEM_PANIC_REBOOT_DELAY_SECONDS
    esp_panic_handler_feed_wdts();

    panic_print_str("Rebooting in ");
    panic_print_dec(CONFIG_ESP_SYSTEM_PANIC_REBOOT_DELAY_SECONDS);
    panic_print_str(" seconds...\r\n");

    esp_rom_delay_us(CONFIG_ESP_SYSTEM_PANIC_REBOOT_DELAY_SECONDS * 1000000);
#endif /* CONFIG_ESP_SYSTEM_PANIC_REBOOT_DELAY_SECONDS */

#if CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT || CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT

    esp_panic_handler_feed_wdts();
    if (esp_reset_reason_get_hint() == ESP_RST_UNKNOWN) {
        switch (info->exception) {
        case PANIC_EXCEPTION_IWDT:
            esp_reset_reason_set_hint(ESP_RST_INT_WDT);
            break;
        case PANIC_EXCEPTION_TWDT:
            esp_reset_reason_set_hint(ESP_RST_TASK_WDT);
            break;
        case PANIC_EXCEPTION_ABORT:
        case PANIC_EXCEPTION_FAULT:
        default:
            esp_reset_reason_set_hint(ESP_RST_PANIC);
            break; // do not touch the previously set reset reason hint
        }
    }

    panic_print_str("Rebooting...\r\n");
    panic_restart();
#else /* CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT || CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT */
    panic_print_str("CPU halted.\r\n");
    esp_system_reset_modules_on_exit();
    disable_all_wdts();
    while (1);
#endif /* CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT || CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT */
#endif /* CONFIG_ESP_SYSTEM_PANIC_GDBSTUB */
}


void IRAM_ATTR __attribute__((noreturn, no_sanitize_undefined)) panic_abort(const char *details)
{
    g_panic_abort = true;
    g_panic_abort_details = (char *) details;

#if CONFIG_APPTRACE_ENABLE
#if CONFIG_APPTRACE_SV_ENABLE
    SEGGER_RTT_ESP_FlushNoLock(CONFIG_APPTRACE_POSTMORTEM_FLUSH_THRESH, APPTRACE_ONPANIC_HOST_FLUSH_TMO);
#else
    esp_apptrace_flush_nolock(ESP_APPTRACE_DEST_TRAX, CONFIG_APPTRACE_POSTMORTEM_FLUSH_THRESH,
                              APPTRACE_ONPANIC_HOST_FLUSH_TMO);
#endif
#endif

    *((volatile int *) 0) = 0; // NOLINT(clang-analyzer-core.NullDereference) should be an invalid operation on targets
    while (1);
}

/* Weak versions of reset reason hint functions.
 * If these weren't provided, reset reason code would be linked into the app
 * even if the app never called esp_reset_reason().
 */
void IRAM_ATTR __attribute__((weak)) esp_reset_reason_set_hint(esp_reset_reason_t hint)
{
}

esp_reset_reason_t IRAM_ATTR  __attribute__((weak)) esp_reset_reason_get_hint(void)
{
    return ESP_RST_UNKNOWN;
}
