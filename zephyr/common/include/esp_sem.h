/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Zephyr-port helper for locks that may be taken from an interrupt or
 * with interrupts locked. A Zephyr mutex cannot be used in those
 * contexts; a binary semaphore can. k_sem_give() is already legal from
 * any context, so only the take needs a helper.
 */

#pragma once

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Take a semaphore from thread, interrupt, or interrupt-locked context.
 *
 * Wait only when the caller is allowed to block. Otherwise take without
 * waiting, and panic if the semaphore is already held: the caller cannot
 * wait there, and continuing without the lock would corrupt the state it
 * protects.
 */
static inline void esp_sem_take_safe(struct k_sem *sem)
{
    bool can_block = false;

    if (!k_is_in_isr() && !k_is_pre_kernel()) {
        unsigned int key = arch_irq_lock();

        can_block = arch_irq_unlocked(key);
        arch_irq_unlock(key);
    }

    if (k_sem_take(sem, can_block ? K_FOREVER : K_NO_WAIT) != 0) {
        k_panic();
    }
}

#ifdef __cplusplus
}
#endif
