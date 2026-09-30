/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "hal/mspi_ll.h"
#include "hal/mspi_periph.h"
#include "esp_private/startup_internal.h"
#include "esp_private/mspi_intr.h"

#if MSPI_LL_INTR_EVENT_SUPPORTED && MSPI_LL_INTR_SHARED

#if CONFIG_ESP_PANIC_HANDLER_IRAM
#define MSPI_ISR_ATTR  IRAM_ATTR
#define MSPI_ISR_FLAGS ESP_INTR_FLAG_IRAM
#else
#define MSPI_ISR_ATTR
#define MSPI_ISR_FLAGS 0
#endif

/*
 * The MSPI error interrupt belongs to the flash controller node, which carries
 * it for both the flash and the PSRAM halves of the controller. The source and
 * the handler are both known at build time, so this is a plain static connect
 * and the interrupt shows up in build/zephyr/isr_intlist.txt.
 *
 * The guard covers SoCs whose devicetree has not been moved onto the
 * multi-level interrupt model yet (esp32c61); there the interrupt is simply not
 * available, rather than silently landing on the wrong slot.
 */
#define MSPI_NODE DT_NODELABEL(flash)
#define MSPI_HAS_IRQ DT_IRQ_HAS_IDX(MSPI_NODE, 0)

ESP_LOG_ATTR_TAG_DRAM(TAG, "mspi_intr");

#if MSPI_HAS_IRQ
static bool s_intr_installed;
static volatile mspi_isr_t s_isr = {
    NULL,
    NULL,
};

static void MSPI_ISR_ATTR mspi_isr_handler(const void *arg)
{
    uint32_t intr_events = mspi_ll_get_intr_raw(MSPI_TIMING_LL_MSPI_ID_0);
    mspi_ll_clear_intr(MSPI_TIMING_LL_MSPI_ID_0, intr_events);

    ESP_DRAM_LOGE(TAG, "MSPI error");
    ESP_DRAM_LOGD(TAG, "intr_events: 0x%" PRIx32, intr_events);

    bool is_ecc_error = false;

#if MSPI_LL_ECC_INT_SUPPORTED
    if (intr_events & MSPI_LL_EVENT_ECC_ERR) {
        ESP_DRAM_LOGE(TAG, "ecc error");
        is_ecc_error = true;
    }
#endif
#if MSPI_LL_PMS_INT_SUPPORTED
    if (intr_events & MSPI_LL_EVENT_PMS_REJECT) {
        ESP_DRAM_LOGE(TAG, "pms reject");
    }
#endif
#if MSPI_LL_ADDR_INT_SUPPORTED
    if (intr_events & MSPI_LL_EVENT_AXI_RADDR_ERR) {
        ESP_DRAM_LOGE(TAG, "read address invalid or misaligned");
    }
    if (intr_events & MSPI_LL_EVENT_AXI_WADDR_ERR) {
        ESP_DRAM_LOGE(TAG, "write addr error");
    }
    if (intr_events & MSPI_LL_EVENT_AXI_WR_FLASH_ERR) {
        ESP_DRAM_LOGE(TAG, "write flash error");
    }
#endif
#if MSPI_LL_THRESH_INT_SUPPORTED
    if (intr_events & MSPI_LL_EVENT_RX_TRANS_OVF) {
        ESP_DRAM_LOGE(TAG, "rx trans overflow");
    }
    if (intr_events & MSPI_LL_EVENT_TX_TRANS_UDF) {
        ESP_DRAM_LOGE(TAG, "tx trans underflow");
    }
#endif

    if (s_isr.psram_isr) {
        s_isr.psram_isr((void *)arg, intr_events);
    }

    if (s_isr.flash_isr) {
        s_isr.flash_isr((void *)arg, intr_events);
    }

    // For ecc error, will handle in the flash/psram isr
    if (!is_ecc_error) {
        abort();
    }

    //no yield for now
}
#endif /* MSPI_HAS_IRQ */

esp_err_t esp_mspi_register_isr(mspi_isr_t *isr)
{
#if !MSPI_HAS_IRQ
    ARG_UNUSED(isr);
    ESP_EARLY_LOGE(TAG, "no MSPI interrupt in the devicetree");
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (isr && isr->psram_isr) {
        s_isr.psram_isr = isr->psram_isr;
    }

    if (isr && isr->flash_isr) {
        s_isr.flash_isr = isr->flash_isr;
    }

    if (!s_intr_installed) {
        IRQ_CONNECT(DT_IRQN(MSPI_NODE), IRQ_DEFAULT_PRIORITY, mspi_isr_handler, NULL,
                    MSPI_ISR_FLAGS);
        irq_enable(DT_IRQN(MSPI_NODE));
        s_intr_installed = true;

        mspi_ll_clear_intr(MSPI_TIMING_LL_MSPI_ID_0, MSPI_LL_EVENT_MASK);
        mspi_ll_enable_intr(MSPI_TIMING_LL_MSPI_ID_0, MSPI_LL_EVENT_MASK, true);
    }

    return ESP_OK;
#endif
}

esp_err_t esp_mspi_unregister_isr(void)
{
#if !MSPI_HAS_IRQ
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!s_intr_installed) {
        ESP_EARLY_LOGE(TAG, "MSPI interrupt not registered");
        return ESP_ERR_INVALID_STATE;
    }

    /* The table slot stays claimed by IRQ_CONNECT; masking the line is all that
     * can be undone, and all that unregistering needs.
     */
    irq_disable(DT_IRQN(MSPI_NODE));
    s_intr_installed = false;

    s_isr.psram_isr = NULL;
    s_isr.flash_isr = NULL;

    return ESP_OK;
#endif
}
#endif  //#if MSPI_LL_INTR_EVENT_SUPPORTED && MSPI_LL_INTR_SHARED
