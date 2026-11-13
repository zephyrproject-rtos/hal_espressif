/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <soc/soc_caps.h>
#include <esp_attr.h>
#include <esp_cpu.h>
#include <esp_rom_sys.h>

#include "esp_bt_intr.h"

#define BT_INTC_NODE DT_NODELABEL(intc)

/* CPU line of the Bluetooth controller; level 3 on Xtensa */
#if defined(CONFIG_SOC_SERIES_ESP32S3)
#define BT_INTR_LINE 23
#else
#define BT_INTR_LINE 22
#endif

/* Priority of the line on SoCs that set it per line, as the controller asks for level 3 */
#define BT_INTR_PRIORITY 3

/* Line a freed source is routed to: the intmux key line, which never takes a source */
#define BT_INTR_UNROUTED DT_IRQ(DT_NODELABEL(intmux), irq)

#define BT_RESERVED_LINE_BIT(node_id, prop, idx) BIT(DT_PROP_BY_IDX(node_id, prop, idx)) |
BUILD_ASSERT(((DT_FOREACH_PROP_ELEM(BT_INTC_NODE, reserved_lines, BT_RESERVED_LINE_BIT) 0U) &
	      BIT(BT_INTR_LINE)) != 0U,
	     "the Bluetooth CPU line must be listed in the intc reserved-lines");

#define BT_INTR_MAX_SOURCES 4

struct bt_intr {
	intr_handler_t handler;
	void *arg;
	int source;
	bool enabled;
};

/* In DRAM: the ISR reads it with the flash cache possibly off */
static DRAM_ATTR struct bt_intr bt_intr[BT_INTR_MAX_SOURCES];
static bool bt_intr_connected;

static inline uint32_t bt_intr_status_base(void)
{
#if DT_REG_HAS_IDX(BT_INTC_NODE, 3)
	if (esp_cpu_get_core_id() != 0) {
		return DT_REG_ADDR_BY_IDX(BT_INTC_NODE, 3);
	}
#endif
	return DT_REG_ADDR_BY_IDX(BT_INTC_NODE, 1);
}

static void IRAM_ATTR bt_intr_isr(const void *unused)
{
	uint32_t base = bt_intr_status_base();

	ARG_UNUSED(unused);

	for (size_t i = 0; i < ARRAY_SIZE(bt_intr); i++) {
		const struct bt_intr *e = &bt_intr[i];

		if ((e->handler == NULL) || !e->enabled) {
			continue;
		}
		if ((sys_read32(base + ((uint32_t)e->source / 32U) * 4U) &
		     BIT((uint32_t)e->source % 32U)) != 0U) {
			e->handler(e->arg);
		}
	}
}

int esp_bt_intr_alloc(int source, intr_handler_t handler, void *arg, void **handle)
{
	unsigned int key = irq_lock();
	struct bt_intr *e = NULL;

	for (size_t i = 0; i < ARRAY_SIZE(bt_intr); i++) {
		if (bt_intr[i].handler == NULL) {
			e = &bt_intr[i];
			break;
		}
	}
	if (e == NULL) {
		irq_unlock(key);
		return -ENOMEM;
	}

	if (!bt_intr_connected) {
		int rc;

#if SOC_CPU_HAS_FLEXIBLE_INTC
		esp_cpu_intr_set_priority(BT_INTR_LINE, BT_INTR_PRIORITY);
		esp_cpu_intr_set_type(BT_INTR_LINE, ESP_CPU_INTR_TYPE_LEVEL);
#endif
		rc = irq_connect_dynamic(BT_INTR_LINE, IRQ_DEFAULT_PRIORITY, bt_intr_isr, NULL,
					 ESP_INTR_FLAG_IRAM);
		if (rc < 0) {
			irq_unlock(key);
			return rc;
		}
		bt_intr_connected = true;
	}

	e->source = source;
	e->arg = arg;
	e->enabled = false;
	e->handler = handler;
	esp_rom_route_intr_matrix(esp_cpu_get_core_id(), source, BT_INTR_LINE);
	irq_unlock(key);

	*handle = e;

	return 0;
}

void esp_bt_intr_free(void *handle)
{
	struct bt_intr *e = handle;
	unsigned int key;

	if (e == NULL) {
		return;
	}

	esp_bt_intr_disable(e);

	key = irq_lock();
	esp_rom_route_intr_matrix(esp_cpu_get_core_id(), e->source, BT_INTR_UNROUTED);
	e->handler = NULL;
	irq_unlock(key);
}

void IRAM_ATTR esp_bt_intr_enable(void *handle)
{
	struct bt_intr *e = handle;

	e->enabled = true;
	irq_enable(BT_INTR_LINE);
}

void IRAM_ATTR esp_bt_intr_disable(void *handle)
{
	struct bt_intr *e = handle;
	bool any = false;
	unsigned int key = irq_lock();

	e->enabled = false;
	for (size_t i = 0; i < ARRAY_SIZE(bt_intr); i++) {
		any = any || ((bt_intr[i].handler != NULL) && bt_intr[i].enabled);
	}
	if (!any) {
		irq_disable(BT_INTR_LINE);
	}
	irq_unlock(key);
}
