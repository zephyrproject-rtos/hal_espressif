/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "hal/lp_core_ll.h"
#include "riscv/rv_utils.h"
#include "riscv/rvruntime-frames.h"
#include "ulp_lp_core_utils.h"

#if SOC_LP_CORE_SINGLE_INTERRUPT_VECTOR
/* Enable interrupt 30, which all external interrupts are routed to*/
#define MIE_ALL_INTS_MASK (1 << 30)
#else
/* Enable all external interrupts routed to CPU, expect HP_INTR,
   as this would trigger an LP core interrupt for every single interrupt
   that triggers on HP Core.
 */
#define MIE_ALL_INTS_MASK 0x3FFF0888
#endif

void ulp_lp_core_intr_enable(void)
{
    /* Enable interrupt globally */
    RV_SET_CSR(mstatus, MSTATUS_MIE);
    RV_SET_CSR(mie, MIE_ALL_INTS_MASK);

}

void ulp_lp_core_intr_disable(void)
{
    RV_CLEAR_CSR(mie, MIE_ALL_INTS_MASK);
    /* Disable interrupts globally */
    RV_CLEAR_CSR(mstatus, MSTATUS_MIE);
}

void  __attribute__((weak)) ulp_lp_core_panic_handler(RvExcFrame *frame, int exccause)
{
    ulp_lp_core_abort();
}

static void ulp_lp_core_default_intr_handler(void)
{
    ulp_lp_core_abort();
}

/* Default ISR handlers, intended to be overwritten by users */
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_lp_io_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_lp_i2c_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_lp_uart_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_lp_timer_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_lp_pmu_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_lp_spi_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_trng_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_lp_adc_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_lp_touch_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_tsens_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_efuse_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_lp_sysreg_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_lp_ana_peri_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_mailbox_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_lp_wdt_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_lp_rtc_intr_handler(void);
void __attribute__((weak, alias("ulp_lp_core_default_intr_handler"))) ulp_lp_core_sw_intr_handler(void);

#if SOC_LP_CORE_SINGLE_INTERRUPT_VECTOR

typedef struct {
    void (*handler)(void *arg);
    void *arg;
} ulp_lp_core_intr_handler_t;

static ulp_lp_core_intr_handler_t s_intr_handlers[] = {
    {(void (*)(void *))ulp_lp_core_lp_io_intr_handler, NULL},
    {(void (*)(void *))ulp_lp_core_lp_i2c_intr_handler, NULL},
    {(void (*)(void *))ulp_lp_core_lp_uart_intr_handler, NULL},
    {(void (*)(void *))ulp_lp_core_lp_timer_intr_handler, NULL},
    {NULL, NULL}, // Reserved / Unused
    {(void (*)(void *))ulp_lp_core_lp_pmu_intr_handler, NULL},
};

int ulp_lp_core_intr_set_handler(int intr_source, void (*handler)(void *arg), void *arg)
{
    if (intr_source < 0 || intr_source >= sizeof(s_intr_handlers) / sizeof(s_intr_handlers[0])) {
        return -1;
    }

    s_intr_handlers[intr_source].handler = handler;
    s_intr_handlers[intr_source].arg = arg;

    return 0;
}

void __attribute__((weak)) ulp_lp_core_intr_handler(void)
{
    uint8_t intr_source = lp_core_ll_get_triggered_interrupt_srcs();
    for (int i = 0; i < sizeof(s_intr_handlers) / sizeof(s_intr_handlers[0]); i++) {
        if (intr_source & (1 << i)) {
            ulp_lp_core_intr_handler_t *handler_entry = &s_intr_handlers[i];
            if (handler_entry->handler) {
                handler_entry->handler(handler_entry->arg);
            }
        }
    }
}

#endif //SOC_LP_CORE_SINGLE_INTERRUPT_VECTOR
