/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "soc/pcr_struct.h"
#include "hal/ahb_dma_ll.h"

#define GDMA_LL_GET(_attr)          GDMA_LL_ ## _attr

#define GDMA_LL_INST_NUM            1
#define GDMA_LL_PAIRS_PER_INST      GDMA_LL_AHB_PAIRS_PER_GROUP

#define GDMA_LL_AHB_PSRAM_CAPABLE         1

#define GDMA_LL_AHB_BURST_SIZE_ADJUSTABLE 1  // AHB GDMA supports adjustable burst size
#define GDMA_LL_MAX_BURST_SIZE_PSRAM      32 // PSRAM controller doesn't support burst access with size > 32 bytes

/*
 * Compatibility aliases: map gdma_ll_* names to the ahb_dma_ll_* API
 * so that Zephyr drivers written against the v1 GDMA interface compile on
 * AHB-GDMA-v2 SoCs without source changes.
 */
#define GDMA_LL_M2M_FREE_PERIPH_ID_MASK          AHB_DMA_LL_M2M_FREE_PERIPH_ID_MASK
#define gdma_ll_force_enable_reg_clock            ahb_dma_ll_force_enable_reg_clock
#define gdma_ll_rx_clear_interrupt_status          ahb_dma_ll_rx_clear_interrupt_status
#define gdma_ll_rx_connect_to_mem                  ahb_dma_ll_rx_connect_to_mem
#define gdma_ll_rx_connect_to_periph               ahb_dma_ll_rx_connect_to_periph
#define gdma_ll_rx_enable_data_burst               ahb_dma_ll_rx_enable_data_burst
#define gdma_ll_rx_enable_descriptor_burst         ahb_dma_ll_rx_enable_descriptor_burst
#define gdma_ll_rx_enable_interrupt                ahb_dma_ll_rx_enable_interrupt
#define gdma_ll_rx_enable_owner_check              ahb_dma_ll_rx_enable_owner_check
#define gdma_ll_rx_get_interrupt_status            ahb_dma_ll_rx_get_interrupt_status
#define gdma_ll_rx_get_interrupt_status_reg        ahb_dma_ll_rx_get_interrupt_status_reg
#define gdma_ll_rx_get_prefetched_desc_addr        ahb_dma_ll_rx_get_prefetched_desc_addr
#define gdma_ll_rx_get_success_eof_desc_addr       ahb_dma_ll_rx_get_success_eof_desc_addr
#define gdma_ll_rx_is_desc_fsm_idle                ahb_dma_ll_rx_is_desc_fsm_idle
#define gdma_ll_rx_reset_channel                   ahb_dma_ll_rx_reset_channel
#define gdma_ll_rx_set_desc_addr                   ahb_dma_ll_rx_set_desc_addr
#define gdma_ll_rx_start                           ahb_dma_ll_rx_start
#define gdma_ll_rx_stop                            ahb_dma_ll_rx_stop
#define gdma_ll_tx_clear_interrupt_status          ahb_dma_ll_tx_clear_interrupt_status
#define gdma_ll_tx_connect_to_mem                  ahb_dma_ll_tx_connect_to_mem
#define gdma_ll_tx_connect_to_periph               ahb_dma_ll_tx_connect_to_periph
#define gdma_ll_tx_enable_data_burst               ahb_dma_ll_tx_enable_data_burst
#define gdma_ll_tx_enable_descriptor_burst         ahb_dma_ll_tx_enable_descriptor_burst
#define gdma_ll_tx_enable_interrupt                ahb_dma_ll_tx_enable_interrupt
#define gdma_ll_tx_get_interrupt_status            ahb_dma_ll_tx_get_interrupt_status
#define gdma_ll_tx_get_interrupt_status_reg        ahb_dma_ll_tx_get_interrupt_status_reg
#define gdma_ll_tx_get_prefetched_desc_addr        ahb_dma_ll_tx_get_prefetched_desc_addr
#define gdma_ll_tx_is_desc_fsm_idle                ahb_dma_ll_tx_is_desc_fsm_idle
#define gdma_ll_tx_reset_channel                   ahb_dma_ll_tx_reset_channel
#define gdma_ll_tx_set_desc_addr                   ahb_dma_ll_tx_set_desc_addr
#define gdma_ll_tx_start                           ahb_dma_ll_tx_start
#define gdma_ll_tx_stop                            ahb_dma_ll_tx_stop

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enable the bus clock for the DMA module
 */
static inline void _gdma_ll_enable_bus_clock(int group_id, bool enable)
{
    (void)group_id;
    PCR.gdma_conf.gdma_clk_en = enable;
}

#define gdma_ll_enable_bus_clock(...) _gdma_ll_enable_bus_clock(__VA_ARGS__)

/**
 * @brief Reset the DMA module
 */
static inline void _gdma_ll_reset_register(int group_id)
{
    (void)group_id;
    PCR.gdma_conf.gdma_rst_en = 1;
    PCR.gdma_conf.gdma_rst_en = 0;
}

#define gdma_ll_reset_register(...) _gdma_ll_reset_register(__VA_ARGS__)

#ifdef __cplusplus
}
#endif
