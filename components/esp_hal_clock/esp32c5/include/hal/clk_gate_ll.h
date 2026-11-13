/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "soc/soc.h"
#include "soc/reset_reasons.h"
#include "soc/periph_defs.h"
#include "soc/pcr_reg.h"
#include "soc/pcr_struct.h"
#include "soc/ahb_dma_struct.h"
#include "soc/lpperi_struct.h"
#include "soc/uart_struct.h"
#include "soc/tee_struct.h"
#include "soc/lp_gpio_struct.h"
#include "soc/lp_tee_struct.h"
#include "soc/lp_apm_struct.h"
#include "soc/lp_apm0_struct.h"
#include "soc/hp_apm_struct.h"
#include "soc/cpu_apm_struct.h"
#include "soc/usb_serial_jtag_struct.h"
#include "soc/lp_clkrst_struct.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline uint32_t periph_ll_get_clk_en_mask(shared_periph_module_t periph)
{
    switch (periph) {
    case PERIPH_TIMG0_MODULE:
        return PCR_TG0_CLK_EN;
    case PERIPH_TIMG1_MODULE:
        return PCR_TG1_CLK_EN;
    case PERIPH_UHCI0_MODULE:
        return PCR_UHCI_CLK_EN;
    case PERIPH_SYSTIMER_MODULE:
        return PCR_SYSTIMER_CLK_EN;
    default:
        return 0;
    }
}

static inline uint32_t periph_ll_get_rst_en_mask(shared_periph_module_t periph, bool enable)
{
    (void)enable;

    switch (periph) {
    case PERIPH_TIMG0_MODULE:
        return PCR_TG0_RST_EN;
    case PERIPH_TIMG1_MODULE:
        return PCR_TG1_RST_EN;
    case PERIPH_UHCI0_MODULE:
        return PCR_UHCI_RST_EN;
    case PERIPH_SYSTIMER_MODULE:
        return PCR_SYSTIMER_RST_EN;
    default:
        return 0;
    }
}

static inline uint32_t periph_ll_get_clk_en_reg(shared_periph_module_t periph)
{
    switch (periph) {
    case PERIPH_TIMG0_MODULE:
        return PCR_TIMERGROUP0_CONF_REG;
    case PERIPH_TIMG1_MODULE:
        return PCR_TIMERGROUP1_CONF_REG;
    case PERIPH_UHCI0_MODULE:
        return PCR_UHCI_CONF_REG;
    case PERIPH_SYSTIMER_MODULE:
        return PCR_SYSTIMER_CONF_REG;
    default:
        return 0;
    }
}

static inline uint32_t periph_ll_get_rst_en_reg(shared_periph_module_t periph)
{
    switch (periph) {
    case PERIPH_TIMG0_MODULE:
        return PCR_TIMERGROUP0_CONF_REG;
    case PERIPH_TIMG1_MODULE:
        return PCR_TIMERGROUP1_CONF_REG;
    case PERIPH_UHCI0_MODULE:
        return PCR_UHCI_CONF_REG;
    case PERIPH_SYSTIMER_MODULE:
        return PCR_SYSTIMER_CONF_REG;
    default:
        return 0;
    }
}

static inline void periph_ll_enable_clk_clear_rst(shared_periph_module_t periph)
{
    SET_PERI_REG_MASK(periph_ll_get_clk_en_reg(periph), periph_ll_get_clk_en_mask(periph));
    CLEAR_PERI_REG_MASK(periph_ll_get_rst_en_reg(periph), periph_ll_get_rst_en_mask(periph, true));
}

static inline void periph_ll_disable_clk_set_rst(shared_periph_module_t periph)
{
    CLEAR_PERI_REG_MASK(periph_ll_get_clk_en_reg(periph), periph_ll_get_clk_en_mask(periph));
    SET_PERI_REG_MASK(periph_ll_get_rst_en_reg(periph), periph_ll_get_rst_en_mask(periph, false));
}

static inline void periph_ll_reset(shared_periph_module_t periph)
{
    SET_PERI_REG_MASK(periph_ll_get_rst_en_reg(periph), periph_ll_get_rst_en_mask(periph, false));
    CLEAR_PERI_REG_MASK(periph_ll_get_rst_en_reg(periph), periph_ll_get_rst_en_mask(periph, false));
}

static inline bool IRAM_ATTR periph_ll_periph_enabled(shared_periph_module_t periph)
{
    return REG_GET_BIT(periph_ll_get_rst_en_reg(periph), periph_ll_get_rst_en_mask(periph, false)) == 0 &&
           REG_GET_BIT(periph_ll_get_clk_en_reg(periph), periph_ll_get_clk_en_mask(periph)) != 0;
}

/**
 * Enable or disable the clock gate for ref_12m.
 * @param  enable Enable / disable
 */
FORCE_INLINE_ATTR void _clk_gate_ll_ref_12m_clk_en(bool enable)
{
    PCR.pll_div_clk_en.pll_12m_clk_en = enable;
}
/// use a macro to wrap the function, force the caller to use it in a critical section
/// the critical section needs to declare the __DECLARE_RCC_ATOMIC_ENV variable in advance
#define clk_gate_ll_ref_12m_clk_en(...) (void)__DECLARE_RCC_ATOMIC_ENV; _clk_gate_ll_ref_12m_clk_en(__VA_ARGS__)

/**
 * Enable or disable the clock gate for ref_20m.
 * @param  enable Enable / disable
 */
FORCE_INLINE_ATTR void _clk_gate_ll_ref_20m_clk_en(bool enable)
{
    PCR.pll_div_clk_en.pll_20m_clk_en = enable;
}
/// use a macro to wrap the function, force the caller to use it in a critical section
/// the critical section needs to declare the __DECLARE_RCC_ATOMIC_ENV variable in advance
#define clk_gate_ll_ref_20m_clk_en(...) (void)__DECLARE_RCC_ATOMIC_ENV; _clk_gate_ll_ref_20m_clk_en(__VA_ARGS__)

/**
 * Enable or disable the clock gate for ref_40m.
 * @param  enable Enable / disable
 */
FORCE_INLINE_ATTR void _clk_gate_ll_ref_40m_clk_en(bool enable)
{
    PCR.pll_div_clk_en.pll_40m_clk_en = enable;
}
/// use a macro to wrap the function, force the caller to use it in a critical section
/// the critical section needs to declare the __DECLARE_RCC_ATOMIC_ENV variable in advance
#define clk_gate_ll_ref_40m_clk_en(...) (void)__DECLARE_RCC_ATOMIC_ENV; _clk_gate_ll_ref_40m_clk_en(__VA_ARGS__)

/**
 * Enable or disable the clock gate for ref_48m.
 * @param  enable Enable / disable
 */
FORCE_INLINE_ATTR void _clk_gate_ll_ref_48m_clk_en(bool enable)
{
    PCR.pll_div_clk_en.pll_48m_clk_en = enable;
}
/// use a macro to wrap the function, force the caller to use it in a critical section
/// the critical section needs to declare the __DECLARE_RCC_ATOMIC_ENV variable in advance
#define clk_gate_ll_ref_48m_clk_en(...) (void)__DECLARE_RCC_ATOMIC_ENV; _clk_gate_ll_ref_48m_clk_en(__VA_ARGS__)

/**
 * Enable or disable the clock gate for ref_60m.
 * @param  enable Enable / disable
 */
FORCE_INLINE_ATTR void _clk_gate_ll_ref_60m_clk_en(bool enable)
{
    PCR.pll_div_clk_en.pll_60m_clk_en = enable;
}
/// use a macro to wrap the function, force the caller to use it in a critical section
/// the critical section needs to declare the __DECLARE_RCC_ATOMIC_ENV variable in advance
#define clk_gate_ll_ref_60m_clk_en(...) (void)__DECLARE_RCC_ATOMIC_ENV; _clk_gate_ll_ref_60m_clk_en(__VA_ARGS__)

/**
 * Enable or disable the clock gate for ref_80m.
 * @param  enable Enable / disable
 */
FORCE_INLINE_ATTR void _clk_gate_ll_ref_80m_clk_en(bool enable)
{
    PCR.pll_div_clk_en.pll_80m_clk_en = enable;
}
/// use a macro to wrap the function, force the caller to use it in a critical section
/// the critical section needs to declare the __DECLARE_RCC_ATOMIC_ENV variable in advance
#define clk_gate_ll_ref_80m_clk_en(...) (void)__DECLARE_RCC_ATOMIC_ENV; _clk_gate_ll_ref_80m_clk_en(__VA_ARGS__)

/**
 * Enable or disable the clock gate for ref_120m.
 * @param  enable Enable / disable
 */
FORCE_INLINE_ATTR void _clk_gate_ll_ref_120m_clk_en(bool enable)
{
    PCR.pll_div_clk_en.pll_120m_clk_en = enable;
}
/// use a macro to wrap the function, force the caller to use it in a critical section
/// the critical section needs to declare the __DECLARE_RCC_ATOMIC_ENV variable in advance
#define clk_gate_ll_ref_120m_clk_en(...) (void)__DECLARE_RCC_ATOMIC_ENV; _clk_gate_ll_ref_120m_clk_en(__VA_ARGS__)

/**
 * Enable or disable the clock gate for ref_160m.
 * @param  enable Enable / disable
 */
FORCE_INLINE_ATTR void _clk_gate_ll_ref_160m_clk_en(bool enable)
{
    PCR.pll_div_clk_en.pll_160m_clk_en = enable;
}
/// use a macro to wrap the function, force the caller to use it in a critical section
/// the critical section needs to declare the __DECLARE_RCC_ATOMIC_ENV variable in advance
#define clk_gate_ll_ref_160m_clk_en(...) (void)__DECLARE_RCC_ATOMIC_ENV; _clk_gate_ll_ref_160m_clk_en(__VA_ARGS__)

/**
 * Enable or disable the clock gate for ref_240m.
 * @param  enable Enable / disable
 */
FORCE_INLINE_ATTR void _clk_gate_ll_ref_240m_clk_en(bool enable)
{
    PCR.pll_div_clk_en.pll_240m_clk_en = enable;
}
/// use a macro to wrap the function, force the caller to use it in a critical section
/// the critical section needs to declare the __DECLARE_RCC_ATOMIC_ENV variable in advance
#define clk_gate_ll_ref_240m_clk_en(...) (void)__DECLARE_RCC_ATOMIC_ENV; _clk_gate_ll_ref_240m_clk_en(__VA_ARGS__)

/**
 * Enable or disable the clock gate for rtc_fast to lp periph
 * @param  enable Enable / disable
 */
FORCE_INLINE_ATTR void _clk_gate_ll_rtc_fast_to_lp_periph_en(bool enable)
{
    LP_CLKRST.lp_clk_en.fast_ori_gate = enable;
}
/// use a macro to wrap the function, force the caller to use it in a critical section
/// the critical section needs to declare the __DECLARE_RCC_ATOMIC_ENV variable in advance
#define clk_gate_ll_rtc_fast_to_lp_periph_en(...) (void)__DECLARE_RCC_ATOMIC_ENV; _clk_gate_ll_rtc_fast_to_lp_periph_en(__VA_ARGS__)

/**
 * @brief Configuration structure for peripheral clock gate settings
 */
typedef struct {
    bool disable_uart0_clk;          ///< Disable UART0 clock (when UART0 is not console)
    bool disable_uart1_clk;          ///< Disable UART1 clock (when UART1 is not console)
    bool disable_mspi_flash_clk;     ///< Disable MSPI flash clock (for PURE_RAM_APP)
    bool disable_crypto_periph_clk;  ///< Disable crypto peripherals clock when TEE is not enabled
    bool disable_usb_serial_jtag;    ///< Disable USB-Serial-JTAG clock and pad (when not enabled)
    bool disable_pvt_clk;            ///< Disable PVT clock
} periph_ll_clk_gate_config_t;

/**
 * @brief Set the default clock gate configuration
 * @param config The configuration structure
 */
static inline void periph_ll_clk_gate_set_default(soc_reset_reason_t rst_reason, const periph_ll_clk_gate_config_t *config)
{
    if ((rst_reason != RESET_REASON_CPU0_MWDT0) && (rst_reason != RESET_REASON_CPU0_MWDT1)          \
            && (rst_reason != RESET_REASON_CPU0_SW) && (rst_reason != RESET_REASON_CPU0_RTC_WDT)    \
            && (rst_reason != RESET_REASON_CPU0_JTAG) && (rst_reason != RESET_REASON_CPU0_LOCKUP)) {
        if (config->disable_uart0_clk) {
            PCR.uart0_conf.uart0_clk_en = 0;
            PCR.uart0_sclk_conf.uart0_sclk_en = 0;
        }

        else if (config->disable_uart1_clk) {
            PCR.uart1_sclk_conf.uart1_sclk_en = 0;
            PCR.uart1_conf.uart1_clk_en = 0;
        }

        PCR.timergroup_xtal_conf.tg0_xtal_clk_en = 0;
        PCR.timergroup0_timer_clk_conf.tg0_timer_clk_en = 0;
        PCR.timergroup1_timer_clk_conf.tg1_timer_clk_en = 0;
        PCR.timergroup0_conf.tg0_clk_en = 0;
        PCR.timergroup1_conf.tg1_clk_en = 0;
        PCR.gdma_conf.gdma_clk_en = 0;

        if (config->disable_mspi_flash_clk) {
            PCR.mspi_conf.mspi_clk_en = 0;
        }

        PCR.spi2_conf.spi2_clk_en = 0;

        if (config->disable_crypto_periph_clk) {
            TEE.clock_gate.clk_en = 0;
            LP_TEE.clock_gate.clk_en = 0;
            HP_APM.clock_gate.clk_en = 0;
            CPU_APM.clock_gate.clk_en = 0;
        }

        PCR.uhci_conf.uhci_clk_en = 0;
        PCR.tcm_mem_monitor_conf.tcm_mem_monitor_clk_en = 0;
        PCR.psram_mem_monitor_conf.psram_mem_monitor_clk_en = 0;
        if (config->disable_pvt_clk) {
            PCR.pvt_monitor_conf.pvt_monitor_clk_en = 0;
            PCR.pvt_monitor_func_clk_conf.pvt_monitor_func_clk_en = 0;
        }
        PCR.ctrl_clk_out_en.val = 0;

        if (config->disable_usb_serial_jtag) {
            // Disable USB-Serial-JTAG clock and it's pad if not used
            USB_SERIAL_JTAG.conf0.usb_pad_enable = 0;
            PCR.usb_device_conf.usb_device_clk_en = 0;
            USB_SERIAL_JTAG.mem_conf.usb_mem_clk_en = 0;
            USB_SERIAL_JTAG.mem_conf.usb_mem_pd = 1;
        }
    }

    if ((rst_reason == RESET_REASON_CHIP_POWER_ON) || (rst_reason == RESET_REASON_CHIP_BROWN_OUT)       \
            || (rst_reason == RESET_REASON_SYS_RTC_WDT) || (rst_reason == RESET_REASON_SYS_SUPER_WDT)   \
            || (rst_reason == RESET_REASON_CORE_PWR_GLITCH)) {
        LP_GPIO.clock_gate.clk_en = 0;
        LP_UART.clk_conf.val = 0;
        LPPERI.clk_en.val = 0;
        LPPERI.clk_en.efuse_ck_en = 1;  // keep efuse clock enabled

        if (config->disable_crypto_periph_clk) {
            LP_APM.clock_gate.clk_en = 0;
            LP_APM0.clock_gate.clk_en = 0;
        }

        LP_CLKRST.lp_clk_po_en.val = 0;
    }
}

#ifdef __cplusplus
}
#endif
