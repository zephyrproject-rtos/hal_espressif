#include <stdint.h>
#include <stdbool.h>
#include <zephyr/irq.h>
#include <esp_attr.h>

#define ATOMIC_LOAD(size, type) \
    IRAM_ATTR __attribute__((weak)) type __atomic_load_##size(const volatile void *ptr, const int memorder) { \
        (void)memorder; \
        return *(const volatile type *)ptr; \
    }

#define ATOMIC_STORE(size, type) \
    IRAM_ATTR __attribute__((weak)) void __atomic_store_##size(volatile void *ptr, const type val, const int memorder) { \
        (void)memorder; \
        *(volatile type *)ptr = val; \
    }

#define ATOMIC_EXCHANGE(size, type) \
    IRAM_ATTR __attribute__((weak)) type __atomic_exchange_##size(volatile void *ptr, const type val, const int memorder) { \
        (void)memorder; \
        const unsigned int key = irq_lock(); \
        const type old = *(volatile type *)ptr; \
        *(volatile type *)ptr = val; \
        irq_unlock(key); \
        return old; \
    }

#define ATOMIC_COMPARE_EXCHANGE(size, type) \
    IRAM_ATTR __attribute__((weak)) bool __atomic_compare_exchange_##size(volatile void *ptr, void *expected, const type desired, const bool weak, const int success_memorder, const int failure_memorder) { \
        (void)weak; (void)success_memorder; (void)failure_memorder; \
        const unsigned int key = irq_lock(); \
        const type actual = *(volatile type *)ptr; \
        const type exp = *(type *)expected; \
        if (actual == exp) { \
            *(volatile type *)ptr = desired; \
            irq_unlock(key); \
            return true; \
        } else { \
            *(type *)expected = actual; \
            irq_unlock(key); \
            return false; \
        } \
    }

#define ATOMIC_FETCH_OP(op, name, size, type) \
    IRAM_ATTR __attribute__((weak)) type __atomic_fetch_##name##_##size(volatile void *ptr, const type val, const int memorder) { \
        (void)memorder; \
        const unsigned int key = irq_lock(); \
        const type old = *(volatile type *)ptr; \
        *(volatile type *)ptr = old op val; \
        irq_unlock(key); \
        return old; \
    }

// 1-byte atomics
ATOMIC_LOAD(1, uint8_t)
ATOMIC_STORE(1, uint8_t)
ATOMIC_EXCHANGE(1, uint8_t)
ATOMIC_COMPARE_EXCHANGE(1, uint8_t)
ATOMIC_FETCH_OP(+, add, 1, uint8_t)
ATOMIC_FETCH_OP(-, sub, 1, uint8_t)
ATOMIC_FETCH_OP(&, and, 1, uint8_t)
ATOMIC_FETCH_OP(|, or, 1, uint8_t)
ATOMIC_FETCH_OP(^, xor, 1, uint8_t)

// 2-byte atomics
ATOMIC_LOAD(2, uint16_t)
ATOMIC_STORE(2, uint16_t)
ATOMIC_EXCHANGE(2, uint16_t)
ATOMIC_COMPARE_EXCHANGE(2, uint16_t)
ATOMIC_FETCH_OP(+, add, 2, uint16_t)
ATOMIC_FETCH_OP(-, sub, 2, uint16_t)
ATOMIC_FETCH_OP(&, and, 2, uint16_t)
ATOMIC_FETCH_OP(|, or, 2, uint16_t)
ATOMIC_FETCH_OP(^, xor, 2, uint16_t)

// 4-byte atomics
ATOMIC_LOAD(4, uint32_t)
ATOMIC_STORE(4, uint32_t)
ATOMIC_EXCHANGE(4, uint32_t)
ATOMIC_COMPARE_EXCHANGE(4, uint32_t)
ATOMIC_FETCH_OP(+, add, 4, uint32_t)
ATOMIC_FETCH_OP(-, sub, 4, uint32_t)
ATOMIC_FETCH_OP(&, and, 4, uint32_t)
ATOMIC_FETCH_OP(|, or, 4, uint32_t)
ATOMIC_FETCH_OP(^, xor, 4, uint32_t)

// 8-byte atomics
ATOMIC_LOAD(8, uint64_t)
ATOMIC_STORE(8, uint64_t)
ATOMIC_EXCHANGE(8, uint64_t)
ATOMIC_COMPARE_EXCHANGE(8, uint64_t)
ATOMIC_FETCH_OP(+, add, 8, uint64_t)
ATOMIC_FETCH_OP(-, sub, 8, uint64_t)
ATOMIC_FETCH_OP(&, and, 8, uint64_t)
ATOMIC_FETCH_OP(|, or, 8, uint64_t)
ATOMIC_FETCH_OP(^, xor, 8, uint64_t)
