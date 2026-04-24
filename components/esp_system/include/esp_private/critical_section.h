/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * This file provides an abstract OS API for entering and exiting critical sections.
 * It furthermore provides macros to define and initialize an optional spinlock
 * if the used chip is a multi-core chip. If a single-core chip is used, just disabling interrupts
 * is sufficient to guarantee consecutive, non-interrupted execution of a critical section.
 * Hence, the spinlock is unnecessary and will be automatically omitted by the macros.
 */
#pragma once

#if !NON_OS_BUILD
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * For Zephyr, we use irq_lock/irq_unlock instead of FreeRTOS spinlocks.
 *
 * NOTE: All six enter/exit variants (normal, _isr, _safe) map to the same
 * irq_lock/irq_unlock pair. On single-core Zephyr this collapses correctly —
 * once irq_lock() disables interrupts globally, no ISR can preempt before
 * irq_unlock(), so the ISR/task distinction is not needed.
 *
 * The `lock` parameter points to an unsigned int (defined by
 * DEFINE_CRIT_SECTION_LOCK_STATIC/LOCK) that stores the IRQ key between
 * enter and exit. This storage is SHARED across all callers of the same lock.
 * This is safe on single-core because the key-writing enter() cannot be
 * preempted by another enter() on the same lock (interrupts are disabled
 * during the critical section). Nested critical sections on the same lock
 * within a single call path MUST NOT be used — the inner enter() would
 * overwrite the outer's saved key and the outer exit() would restore the
 * wrong IRQ state. If nesting is required, use a stack-local key pattern
 * instead (see components/esp_driver_dac/dac_priv_common.h for an example).
 */
#define OS_SPINLOCK 0

/**
 * Define and initialize a static (internal linking) lock for entering critical sections.
 *
 * Use this when all the critical sections are local inside a file.
 * The lock will only be defined if built for a multi-core system, otherwise it is unnecessary.
 *
 * @note When using this macro, the critical section macros esp_os_enter_critical* and esp_os_exit_critical*
 * MUST be used, otherwise normal functions would be passed an undefined variable when build for single-core systems.
 *
 * @param lock_name Variable name of the lock. This will later be used to reference the declared lock.
 * @param optional_qualifiers Qualifiers such as DRAM_ATTR and other attributes. Can be omitted if no qualifiers are
 *        required.
 *
 * Example usage:
 * @code{c}
 * ...
 * #include "os/critical_section.h"
 * ...
 * DEFINE_CRIT_SECTION_LOCK_STATIC(my_lock); // will have internal linking (static)
 * ...
 * esp_os_enter_critical(&my_lock);
 * ...
 * esp_os_exit_critical(&my_lock);
 * @endcode
 */
#if OS_SPINLOCK == 1
#define DEFINE_CRIT_SECTION_LOCK_STATIC(lock_name, optional_qualifiers...) static optional_qualifiers esp_os_spinlock_t lock_name = SPINLOCK_INITIALIZER
#else
/* For Zephyr single-core, still define the lock variable to store irq_lock state */
#define DEFINE_CRIT_SECTION_LOCK_STATIC(lock_name, optional_qualifiers...) static optional_qualifiers unsigned int lock_name = 0
#endif

/**
 * Define and initialize a non-static (external linking) lock for entering critical sections.
 *
 * Locks defined by this macro can be linked among object files but this rather exceptional.
 * Prefer the static lock definition whenever possible.
 * The lock will only be defined if built for a multi-core system, otherwise it is unnecessary.
 *
 * @note When using this macro, the critical section macros esp_os_enter_critical* and esp_os_exit_critical*
 * MUST be used, otherwise normal functions would be passed an undefined variable when build for single-core systems.
 *
 * @param lock_name Variable name of the lock. This will later be used to reference the declared lock.
 * @param optional_qualifiers Qualifiers such as DRAM_ATTR and other attributes. Can be omitted if no qualifiers are
 *        required.
 *
 * Example usage:
 * @code{c}
 * ...
 * #include "os/critical_section.h"
 * ...
 * DEFINE_CRIT_SECTION_LOCK(my_lock); // will have external linking (non-static)
 * ...
 * esp_os_enter_critical(&my_lock);
 * ...
 * esp_os_exit_critical(&my_lock);
 * @endcode
 */
#if OS_SPINLOCK == 1
#define DEFINE_CRIT_SECTION_LOCK(lock_name, optional_qualifiers...) optional_qualifiers esp_os_spinlock_t lock_name = SPINLOCK_INITIALIZER
#else
/* For Zephyr single-core, still define the lock variable to store irq_lock state */
#define DEFINE_CRIT_SECTION_LOCK(lock_name, optional_qualifiers...) optional_qualifiers unsigned int lock_name = 0
#endif

/**
 * @brief This macro initializes a critical section lock at runtime.
 *
 * This macro basically creates a member of the initialization list, including the trailing comma.
 * If the lock is unnecessary because the architecture is single-core, this macro will not do anything.
 * This is incompatible with a lock created by DEFINE_CRIT_SECTION_LOCK_STATIC from above.
 *
 * @param lock_name Pointer to the lock.
 *
 * @note When using this macro, the critical section macros esp_os_enter_critical* and esp_os_exit_critical*
 *       MUST be used, otherwise normal functions would be passed an undefined variable when build for single-core
 *       systems.
 *
 * Example usage:
 * @code{c}
 * ...
 * #include "os/critical_section.h"
 * ...
 * typedef struct protected_struct_t {
 *     int member1;
 *     DECLARE_CRIT_SECTION_LOCK_IN_STRUCT(my_lock)
 *     int another_member;
 * };
 * ...
 * protected_struct_t my_protected;
 * INIT_CRIT_SECTION_LOCK_IN_STRUCT(&(my_protected.my_lock));
 * };
 * @endcode
 */
#if OS_SPINLOCK == 1
#define INIT_CRIT_SECTION_LOCK_RUNTIME(lock_name) spinlock_initialize(lock_name)
#else
#define INIT_CRIT_SECTION_LOCK_RUNTIME(lock_name)
#endif

/**
 * @brief This macro declares a critical section lock as a member of a struct.
 *
 * The critical section lock member is only declared if built for multi-core systems, otherwise it is omitted.
 *
 * @note When using this macro, the critical section macros esp_os_enter_critical* and esp_os_exit_critical*
 *       MUST be used, otherwise normal functions would be passed an undefined variable when build for single-core
 *       systems.
 * @note Do NOT add any semicolon after declaring the member with this macro.
 *       The trailing semicolon is included in the macro, otherwise -Wpedantic would complain about
 *       superfluous ";" if OS_SPINLOCK == 0.
 *
 * Example usage:
 * @code{c}
 * ...
 * #include "os/critical_section.h"
 * ...
 * typedef struct protected_struct_t {
 *     int member1;
 *     DECLARE_CRIT_SECTION_LOCK_IN_STRUCT(my_lock) // no semicolon!
 *     int another_member;
 * };
 * @endcode
 */
#if OS_SPINLOCK == 1
#define DECLARE_CRIT_SECTION_LOCK_IN_STRUCT(lock_name) esp_os_spinlock_t lock_name;
#else
#define DECLARE_CRIT_SECTION_LOCK_IN_STRUCT(lock_name)
#endif

/**
 * @brief This macro initializes a critical section lock as a member of a struct when using an list initialization.
 *        It has to be used together with \c DECLARE_CRIT_SECTION_LOCK_IN_STRUCT() to work.
 *
 * This macro basically creates a member of the initialization list, including the trailing comma.
 * If the lock is unnecessary because the architecture is single-core, this macro will not do anything.
 * This means that if \c lock_name is still a member of the struct, \c lock_name will be uninitialized.
 * Hence, this macro has to be used together with \c DECLARE_CRIT_SECTION_LOCK_IN_STRUCT() to correctly to declare
 * or omit the struct member \c lock_name.
 *
 * @param lock_name The field name of the lock inside the struct.
 *
 * @note When using this macro, the critical section macros esp_os_enter_critical* and esp_os_exit_critical*
 *       MUST be used, otherwise normal functions would be passed an undefined variable when build for single-core
 *       systems.
 * @note Do NOT add any comma in the initializer list after using this macro.
 *
 * Example usage:
 * @code{c}
 * ...
 * #include "os/critical_section.h"
 * ...
 * typedef struct protected_struct_t {
 *     int member1;
 *     DECLARE_CRIT_SECTION_LOCK_IN_STRUCT(my_lock)
 *     int another_member;
 * };
 * ...
 * protected_struct_t my_protected = {
 *     .member1 = 0,
 *     INIT_CRIT_SECTION_LOCK_IN_STRUCT(my_lock) // no comma!
 *     another_member = 47,
 * };
 * @endcode
 */
#if OS_SPINLOCK == 1
#define INIT_CRIT_SECTION_LOCK_IN_STRUCT(lock_name) .lock_name = portMUX_INITIALIZER_UNLOCKED,
#else
#define INIT_CRIT_SECTION_LOCK_IN_STRUCT(lock_name)
#endif

/**
 * @brief Enter a critical section, i.e., a section that will not be interrupted by any other task or interrupt.
 *
 * On multi-core systems, this will disable interrupts and take the spinlock \c lock. On single core systems, a
 * spinlock is unnecessary, hence \c lock is ignored and interrupts are disabled only.
 *
 * @note This macro MUST be used together with any of the initialization macros, e.g.
 *       DEFINE_CRIT_SECTION_LOCK_STATIC. If not, there may be unused variables.
 *
 * @param lock Pointer to the critical section lock. Ignored if build for single core system.
 *
 * Example usage with static locks:
 * @code{c}
 * ...
 * #include "os/critical_section.h"
 * ...
 * DEFINE_CRIT_SECTION_LOCK_STATIC(my_lock); // will have internal linking (static)
 * ...
 * esp_os_enter_critical(&my_lock);
 * // code inside critical section
 * esp_os_exit_critical(&my_lock);
 * @endcode
 */
#define esp_os_enter_critical(lock)         do { unsigned int *_k = (unsigned int *)(lock); *_k = irq_lock(); } while(0)

/**
 * @brief Exit a critical section.
 *
 * On multi-core systems, this will enable interrupts and release the spinlock \c lock. On single core systems, a
 * spinlock is unnecessary, hence \c lock is ignored and interrupts are enabled only.
 *
 * @note This macro MUST be used together with any of the initialization macros, e.g.
 *       DEFINE_CRIT_SECTION_LOCK_STATIC. If not, there may be unused variables.
 *
 * @param lock Pointer to the critical section lock. Ignored if build for single core system.
 *
 * Example usage with static locks:
 * @code{c}
 * ...
 * #include "os/critical_section.h"
 * ...
 * DEFINE_CRIT_SECTION_LOCK_STATIC(my_lock); // will have internal linking (static)
 * ...
 * esp_os_enter_critical(&my_lock);
 * // code inside critical section
 * esp_os_exit_critical(&my_lock);
 * @endcode
 */
#define esp_os_exit_critical(lock)          irq_unlock(*(unsigned int *)(lock))

/**
 * @brief Enter a critical section while from ISR.
 *
 * On multi-core systems, this will disable interrupts and take the spinlock \c lock. On single core systems, a
 * spinlock is unnecessary, hence \c lock is ignored and interrupts are disabled only.
 *
 * @note This macro MUST be used together with any of the initialization macros, e.g.
 *       DEFINE_CRIT_SECTION_LOCK_STATIC. If not, there may be unused variables.
 *
 * @param lock Pointer to the critical section lock. Ignored if build for single core system.
 *
 * Example usage with static locks:
 * @code{c}
 * ...
 * #include "os/critical_section.h"
 * ...
 * DEFINE_CRIT_SECTION_LOCK_STATIC(my_lock); // will have internal linking (static)
 * ...
 * esp_os_enter_critical(&my_lock);
 * // code inside critical section
 * esp_os_exit_critical(&my_lock);
 * @endcode
 */
#define esp_os_enter_critical_isr(lock)     do { unsigned int *_k = (unsigned int *)(lock); *_k = irq_lock(); } while(0)

/**
 * @brief Exit a critical section after entering from ISR.
 *
 * On multi-core systems, this will enable interrupts and release the spinlock \c lock. On single core systems, a
 * spinlock is unnecessary, hence \c lock is ignored and interrupts are enabled only.
 *
 * @note This macro MUST be used together with any of the initialization macros, e.g.
 *       DEFINE_CRIT_SECTION_LOCK_STATIC. If not, there may be unused variables.
 *
 * @param lock Pointer to the critical section lock. Ignored if build for single core system.
 *
 * Example usage with static locks:
 * @code{c}
 * ...
 * #include "os/critical_section.h"
 * ...
 * DEFINE_CRIT_SECTION_LOCK_STATIC(my_lock); // will have internal linking (static)
 * ...
 * esp_os_enter_critical(&my_lock);
 * // code inside critical section
 * esp_os_exit_critical(&my_lock);
 * @endcode
 */
#define esp_os_exit_critical_isr(lock)      irq_unlock(*(unsigned int *)(lock))

/**
 * @brief Enter a critical section from normal task or ISR. This macro will check if the current CPU is processing
 *        an ISR or not and enter the critical section accordingly.
 *
 * On multi-core systems, this will disable interrupts and take the spinlock \c lock. On single core systems, a
 * spinlock is unnecessary, hence \c lock is ignored and interrupts are disabled only.
 *
 * @note This macro MUST be used together with any of the initialization macros, e.g.
 *       DEFINE_CRIT_SECTION_LOCK_STATIC. If not, there may be unused variables.
 *
 * @param lock Pointer to the critical section lock. Ignored if build for single core system.
 *
 * Example usage with static locks:
 * @code{c}
 * ...
 * #include "os/critical_section.h"
 * ...
 * DEFINE_CRIT_SECTION_LOCK_STATIC(my_lock); // will have internal linking (static)
 * ...
 * esp_os_enter_critical(&my_lock);
 * // code inside critical section
 * esp_os_exit_critical(&my_lock);
 * @endcode
 */
#define esp_os_enter_critical_safe(lock)    do { unsigned int *_k = (unsigned int *)(lock); *_k = irq_lock(); } while(0)

/**
 * @brief Exit a critical section after entering via esp_os_enter_critical_safe.
 *
 * On multi-core systems, this will enable interrupts and release the spinlock \c lock. On single core systems, a
 * spinlock is unnecessary, hence \c lock is ignored and interrupts are enabled only.
 *
 * @note This macro MUST be used together with any of the initialization macros, e.g.
 *       DEFINE_CRIT_SECTION_LOCK_STATIC. If not, there may be unused variables.
 *
 * @param lock Pointer to the critical section lock. Ignored if build for single core system.
 *
 * Example usage with static locks:
 * @code{c}
 * ...
 * #include "os/critical_section.h"
 * ...
 * DEFINE_CRIT_SECTION_LOCK_STATIC(my_lock); // will have internal linking (static)
 * ...
 * esp_os_enter_critical(&my_lock);
 * // code inside critical section
 * esp_os_exit_critical(&my_lock);
 * @endcode
 */
#define esp_os_exit_critical_safe(lock)     irq_unlock(*(unsigned int *)(lock))

#ifdef __cplusplus
}
#endif
