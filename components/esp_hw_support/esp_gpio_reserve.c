/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc/soc_caps.h"
#include "esp_types.h"
#include "esp_bit_defs.h"
#include "esp_private/esp_gpio_reserve.h"

#ifdef __ZEPHYR__
#include <zephyr/irq.h>
#include <zephyr/arch/cpu.h>

/*
 * Use arch_irq_lock instead of irq_lock because this code runs during early
 * boot before the kernel threading model is up (e.g., PSRAM init). Under SMP,
 * irq_lock() resolves to z_smp_global_lock() which dereferences _current and
 * is unsafe pre-kernel.
 */
static uint64_t s_reserved_pin_mask = ~(SOC_GPIO_VALID_GPIO_MASK);

uint64_t esp_gpio_reserve(uint64_t gpio_mask)
{
    unsigned int key = arch_irq_lock();
    uint64_t prev = s_reserved_pin_mask;
    s_reserved_pin_mask |= gpio_mask;
    arch_irq_unlock(key);
    return prev;
}

uint64_t esp_gpio_revoke(uint64_t gpio_mask)
{
    unsigned int key = arch_irq_lock();
    uint64_t prev = s_reserved_pin_mask;
    s_reserved_pin_mask &= ~gpio_mask;
    arch_irq_unlock(key);
    return prev;
}

bool esp_gpio_is_reserved(uint64_t gpio_mask)
{
    return s_reserved_pin_mask & gpio_mask;
}

#else /* ESP-IDF */

#include <stdatomic.h>

static _Atomic uint64_t s_reserved_pin_mask = ATOMIC_VAR_INIT(~(SOC_GPIO_VALID_GPIO_MASK));

uint64_t esp_gpio_reserve(uint64_t gpio_mask)
{
    return atomic_fetch_or(&s_reserved_pin_mask, gpio_mask);
}

uint64_t esp_gpio_revoke(uint64_t gpio_mask)
{
    return atomic_fetch_and(&s_reserved_pin_mask, ~gpio_mask);
}

bool esp_gpio_is_reserved(uint64_t gpio_mask)
{
    return atomic_load(&s_reserved_pin_mask) & gpio_mask;
}

#endif /* __ZEPHYR__ */

// TODO: IDF-6968 reserve the pins that not fanned out regarding the SiP version
