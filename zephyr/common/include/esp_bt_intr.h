/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ESP_BT_INTR_H_
#define ESP_BT_INTR_H_

#include <zephyr/drivers/interrupt_controller/intc_esp32.h>

/*
 * Interrupts of the Bluetooth controller. The controller picks its interrupt
 * matrix sources at run time, so they have no devicetree consumer. They share
 * one CPU line that the devicetree lists in the intc "reserved-lines"; these
 * helpers route each source to that line and dispatch it from one ISR.
 */

/**
 * @brief Route a source to the Bluetooth line and record its handler.
 *
 * The source stays disabled until esp_bt_intr_enable().
 *
 * @param source Interrupt matrix source
 * @param handler Handler of the source
 * @param arg Handler argument
 * @param handle Returns the handle for the other calls
 *
 * @retval 0 Success
 * @retval -ENOMEM No free slot
 * @retval <0 The line could not be connected
 */
int esp_bt_intr_alloc(int source, intr_handler_t handler, void *arg, void **handle);

/**
 * @brief Unroute a source and forget its handler.
 *
 * @param handle Handle from esp_bt_intr_alloc()
 */
void esp_bt_intr_free(void *handle);

/**
 * @brief Let the source call its handler.
 *
 * @param handle Handle from esp_bt_intr_alloc()
 */
void esp_bt_intr_enable(void *handle);

/**
 * @brief Stop the source from calling its handler.
 *
 * @param handle Handle from esp_bt_intr_alloc()
 */
void esp_bt_intr_disable(void *handle);

#endif /* ESP_BT_INTR_H_ */
