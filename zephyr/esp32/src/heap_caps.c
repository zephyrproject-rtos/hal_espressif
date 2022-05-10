/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/sys/math_extras.h>
#include <esp32/spiram.h>
#include <esp_attr.h>

#if (CONFIG_ESP_SPIRAM || (CONFIG_HEAP_MEM_POOL_SIZE > 0) || (CONFIG_ESP_HEAP_MEM_POOL_REGION_1_SIZE > 0))
#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
void *__real_k_malloc(size_t size);
void *__real_k_calloc(size_t nmemb, size_t size);
#endif

#if (CONFIG_ESP_HEAP_MEM_POOL_REGION_1_SIZE > 0)
char __aligned(sizeof(void *)) __NOINIT_ATTR dram0_seg_1_heap[CONFIG_ESP_HEAP_MEM_POOL_REGION_1_SIZE];
STRUCT_SECTION_ITERABLE(k_heap, _internal_heap_1) = {
    .heap = {
        .init_mem = dram0_seg_1_heap,
        .init_bytes = CONFIG_ESP_HEAP_MEM_POOL_REGION_1_SIZE,
    }
};
#endif

#if defined(CONFIG_ESP_SPIRAM)
EXT_RAM_ATTR int _spiram_data_start;
STRUCT_SECTION_ITERABLE(k_heap, _spiram_heap) = {
    .heap = {
        .init_mem = &_spiram_data_start,
#if (CONFIG_ESP_SPIRAM_SIZE <= 0x400000)
        .init_bytes = CONFIG_ESP_SPIRAM_SIZE,
#else
        .init_bytes = 0x400000,
#endif
    },
};
#endif

#if (CONFIG_ESP_SPIRAM || (CONFIG_ESP_HEAP_MEM_POOL_REGION_1_SIZE > 0))
static void *z_esp_aligned_alloc(struct k_heap *heap, size_t align, size_t size)
{
    void *mem;
    struct k_heap **heap_ref;
    size_t __align;

    /*
     * Adjust the size to make room for our heap reference.
     * Merge a rewind bit with align value (see sys_heap_aligned_alloc()).
     * This allows for storing the heap pointer right below the aligned
     * boundary without wasting any memory.
     */
    if (size_add_overflow(size, sizeof(heap_ref), &size)) {
        return NULL;
    }
    __align = align | sizeof(heap_ref);

    mem = k_heap_aligned_alloc(heap, __align, size, K_NO_WAIT);
    if (mem == NULL) {
        return NULL;
    }

    heap_ref = mem;
    *heap_ref = heap;
    mem = ++heap_ref;
    __ASSERT(align == 0 || ((uintptr_t)mem & (align - 1)) == 0,
             "misaligned memory at %p (align = %zu)", mem, align);

    return mem;
}

static void *z_esp_aligned_calloc(struct k_heap *heap, size_t nmemb, size_t size)
{
    void *ret;
    size_t bounds;
    if (size_mul_overflow(nmemb, size, &bounds)) {
        return NULL;
    }
    ret = z_esp_aligned_alloc(heap, sizeof(void *), bounds);
    if (ret != NULL) {
        (void)memset(ret, 0, bounds);
    }
    return ret;
}
#endif

static void *z_esp_alloc_internal(size_t align, size_t size)
{
    void *ptr = NULL;
#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
    ptr = __real_k_malloc(size);
#endif
#if (CONFIG_ESP_HEAP_MEM_POOL_REGION_1_SIZE > 0)
    if (ptr == NULL) {
        ptr = z_esp_aligned_alloc(&_internal_heap_1, align, size);
    }
#endif
    return ptr;
}

static void *z_esp_calloc_internal(size_t nmemb, size_t size)
{
    void *ptr = NULL;
#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
    ptr = __real_k_calloc(nmemb, size);
#endif
#if (CONFIG_ESP_HEAP_MEM_POOL_REGION_1_SIZE > 0)
    if (ptr == NULL) {
        ptr = z_esp_aligned_calloc(&_internal_heap_1, nmemb, size);
    }
#endif
    return ptr;
}

#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
void *__wrap_k_malloc(size_t size)
#else
void *k_malloc(size_t size)
#endif
{
    void *ptr = NULL;
#if defined(CONFIG_ESP_SPIRAM)
    if (size < CONFIG_ESP_HEAP_MIN_EXTRAM_THRESHOLD) {
#endif
       ptr = z_esp_alloc_internal(sizeof(void *), size);
#if defined(CONFIG_ESP_HEAP_SEARCH_ALL_REGIONS)
        if (ptr == NULL) {
            ptr = z_esp_aligned_alloc(&_spiram_heap, sizeof(void *), size);
        }
#endif
#if defined(CONFIG_ESP_SPIRAM)
    } else {
        ptr = z_esp_aligned_alloc(&_spiram_heap, sizeof(void *), size);
#if defined(CONFIG_ESP_HEAP_SEARCH_ALL_REGIONS)
        if (ptr == NULL) {
            ptr = z_esp_alloc_internal(sizeof(void *), size);
        }
#endif
    }
#endif
    return ptr;
}

#if (CONFIG_HEAP_MEM_POOL_SIZE > 0)
void *__wrap_k_calloc(size_t nmemb, size_t size)
#else
void *k_calloc(size_t nmemb, size_t size)
#endif
{
    void *ptr = NULL;
#if defined(CONFIG_ESP_SPIRAM)
    if (size < CONFIG_ESP_HEAP_MIN_EXTRAM_THRESHOLD) {
#endif
        ptr = z_esp_calloc_internal(nmemb, size);
#if defined(CONFIG_ESP_HEAP_SEARCH_ALL_REGIONS)
        if (ptr == NULL) {
            ptr = z_esp_aligned_calloc(&_spiram_heap, nmemb, size);
        }
#endif
#if defined(CONFIG_ESP_SPIRAM)
    } else {
        ptr = z_esp_aligned_calloc(&_spiram_heap, nmemb, size);
#if defined(CONFIG_ESP_HEAP_SEARCH_ALL_REGIONS)
        if (ptr == NULL) {
            ptr = z_esp_calloc_internal(nmemb, size);
        }
#endif
    }
#endif
    return ptr;
}
#endif
