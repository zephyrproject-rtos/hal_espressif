/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Zephyr-port implementation of the hal_espressif critical-section
 * primitive. Included by components/esp_system/include/esp_private/
 * critical_section.h, which keeps the pristine ESP-IDF API surface.
 *
 * Design
 * ------
 * Single-core: IRQ save/restore with a recursion counter. Outermost
 *   enter captures the IRQ key; outermost exit restores it.
 *
 * SMP: recursive CAS-based spinlock over `owner_cpu`, modelled after
 *   pristine ESP-IDF spinlock_acquire():
 *   - Lock state lives in `owner_cpu` (ESP_CPU_UNOWNED when free).
 *   - Outermost enter: disable IRQs, CAS(owner_cpu, UNOWNED, me), spin
 *     on the CAS until it succeeds, save IRQ key.
 *   - Recursive enter on the owning CPU: bump counter under local IRQ
 *     lock; no CAS. The atomic load of owner_cpu uses SEQ_CST
 *     ordering, so we cannot miss an ownership transfer done by
 *     another CPU.
 *   - Outermost exit: atomic SEQ_CST store of ESP_CPU_UNOWNED,
 *     restore IRQs.
 *
 *   This does not use k_spin_lock() because Zephyr spinlocks are
 *   non-recursive and their key type is opaque; modelling recursion
 *   on top is racy without an atomic ownership field. Using
 *   atomic_cas on owner_cpu with proper memory ordering gives a
 *   correct recursive lock in one primitive.
 *
 * Safety
 * ------
 * Safe to call from task, ISR, and pre-kernel contexts. The same
 * primitive is used for all contexts and across the pre/post-kernel
 * boundary, so an enter before k_start_kernel() pairs correctly with
 * an exit after it.
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/atomic.h>
#include "esp_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Sentinel value for "no CPU owns this lock". Must not collide with any
 * valid esp_cpu_get_core_id() return (which is [0 .. SOC_CPU_CORES_NUM-1]).
 */
#define ESP_CRITICAL_SECTION_UNOWNED   ((atomic_val_t)-1)

typedef struct {
    atomic_t          owner_cpu;      /* ESP_CRITICAL_SECTION_UNOWNED or owning core id */
    unsigned int      saved_irq_key;  /* IRQ key saved at outermost enter */
    volatile uint32_t count;          /* recursion depth, 0 when free */
} esp_critical_section_t;

#define ESP_CRITICAL_SECTION_INIT { \
    .owner_cpu = ATOMIC_INIT(ESP_CRITICAL_SECTION_UNOWNED), \
    .saved_irq_key = 0, \
    .count = 0, \
}

static ALWAYS_INLINE void esp_critical_section_enter(esp_critical_section_t *l)
{
    unsigned int flags = arch_irq_lock();

#ifdef CONFIG_SMP
    atomic_val_t me = (atomic_val_t)esp_cpu_get_core_id();

    /*
     * Recursive fast path: if this CPU already owns the lock, bump
     * the counter. IRQs are disabled locally so a concurrent writer
     * on this CPU (an ISR) cannot race us on count. Cross-CPU
     * staleness is not a concern here because owner_cpu can only
     * equal `me` if we set it, and we haven't released yet (count
     * has been >= 1 the whole time).
     */
    if (atomic_get(&l->owner_cpu) == me) {
        l->count++;
        return;
    }

    /*
     * Outermost path: spin on CAS until we win ownership. Pristine
     * ESP-IDF's spinlock_acquire uses the same pattern; Zephyr's
     * atomic_cas provides __ATOMIC_SEQ_CST ordering, which is
     * sufficient (and stronger than required) for correct cross-CPU
     * handoff.
     */
    while (!atomic_cas(&l->owner_cpu, ESP_CRITICAL_SECTION_UNOWNED, me)) {
        /* Busy-wait. Interrupts are off on this CPU; the holding
         * CPU will release soon. arch_spin_relax() is a no-op on
         * current hal_espressif targets, so it is omitted here.
         */
    }

    /* We now exclusively own the lock. count must be 0 (invariant). */
    __ASSERT_NO_MSG(l->count == 0);
    l->count = 1;
    l->saved_irq_key = flags;
#else
    /* Single-core: IRQ lock alone is sufficient for mutual exclusion. */
    if (l->count++ == 0) {
        l->saved_irq_key = flags;
    }
#endif
}

static ALWAYS_INLINE void esp_critical_section_exit(esp_critical_section_t *l)
{
    /*
     * Release-build safety net: an unpaired exit would underflow the
     * count and leave the lock held forever. Debug builds assert;
     * release builds panic to surface the bug instead of deadlocking
     * silently.
     */
    if (l->count == 0) {
        __ASSERT(false, "esp_critical_section_exit: unpaired exit on %p", l);
        k_panic();
        return;
    }

#ifdef CONFIG_SMP
    __ASSERT_NO_MSG(atomic_get(&l->owner_cpu) == (atomic_val_t)esp_cpu_get_core_id());

    if (--l->count == 0) {
        unsigned int flags = l->saved_irq_key;

        /* SEQ_CST store: the next acquirer's atomic_cas sees all
         * writes we made while holding the lock.
         */
        atomic_set(&l->owner_cpu, ESP_CRITICAL_SECTION_UNOWNED);
        arch_irq_unlock(flags);
    }
#else
    if (--l->count == 0) {
        arch_irq_unlock(l->saved_irq_key);
    }
#endif
}

#ifdef __cplusplus
}
#endif
