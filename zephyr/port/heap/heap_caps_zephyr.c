/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/math_extras.h>
#include <esp_attr.h>
#include <esp_heap_caps.h>

#if (CONFIG_ESP_HEAP_MEM_POOL_REGION_1_SIZE > 0)
char __aligned(sizeof(void *)) __NOINIT_ATTR dram0_seg_1_heap[CONFIG_ESP_HEAP_MEM_POOL_REGION_1_SIZE];
STRUCT_SECTION_ITERABLE(k_heap, _internal_heap_1) = {
    .heap = {
        .init_mem = dram0_seg_1_heap,
        .init_bytes = CONFIG_ESP_HEAP_MEM_POOL_REGION_1_SIZE,
    }
};
#endif /* CONFIG_ESP_HEAP_MEM_POOL_REGION_1_SIZE > 0 */

static esp_alloc_failed_hook_t alloc_failed_callback;

esp_err_t heap_caps_register_failed_alloc_callback(esp_alloc_failed_hook_t callback)
{
    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    alloc_failed_callback = callback;

    return ESP_OK;
}

static void heap_caps_alloc_failed(size_t requested_size, uint32_t caps, const char *function_name)
{
    if (alloc_failed_callback) {
        alloc_failed_callback(requested_size, caps, function_name);
    }

#ifdef CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS
    esp_system_abort("Memory allocation failed");
#endif
}

/*
Routine to allocate a bit of memory with certain capabilities. caps is a bitfield of MALLOC_CAP_* bits.
*/
static void *heap_caps_malloc_base( size_t size, uint32_t caps)
{

    void *ptr = k_malloc(size);

    if (!ptr && size > 0) {
        heap_caps_alloc_failed(size, caps, __func__);
    }

    return ptr;
}

void *heap_caps_malloc( size_t size, uint32_t caps)
{
    return heap_caps_malloc_base(size, caps);
}

void heap_caps_malloc_extmem_enable(size_t limit)
{
    (void)limit;
    // Not implemented
}

void *heap_caps_malloc_default( size_t size )
{
    return heap_caps_malloc_base(size, MALLOC_CAP_DEFAULT);
}

void *heap_caps_malloc_prefer( size_t size, size_t num, ... )
{
    return heap_caps_malloc(size, MALLOC_CAP_DEFAULT);
}


static void *heap_caps_realloc_base( void *ptr, size_t size, uint32_t caps)
{
    ptr = k_realloc(ptr, size);

    if (ptr == NULL && size > 0) {
        heap_caps_alloc_failed(size, caps, __func__);
    }

    return ptr;
}

void *heap_caps_realloc( void *ptr, size_t size, uint32_t caps)
{
    return heap_caps_realloc_base(ptr, size, caps);
}

void *heap_caps_realloc_default( void *ptr, size_t size )
{
    return heap_caps_realloc_base(ptr, size, MALLOC_CAP_DEFAULT);
}

void *heap_caps_realloc_prefer( void *ptr, size_t size, size_t num, ... )
{
    return heap_caps_realloc_base(ptr, size, MALLOC_CAP_DEFAULT);
}

void heap_caps_free( void *ptr)
{
    k_free(ptr);
}

static void *heap_caps_calloc_base( size_t n, size_t size, uint32_t caps)
{
    size_t size_bytes;

    if (__builtin_mul_overflow(n, size, &size_bytes)) {
        return NULL;
    }

    return k_calloc(n, size);
}

void *heap_caps_calloc( size_t n, size_t size, uint32_t caps)
{
    void *ptr = heap_caps_calloc_base(n, size, caps);

    if (!ptr && size > 0) {
        heap_caps_alloc_failed(size, caps, __func__);
    }

    return ptr;
}


void *heap_caps_calloc_prefer( size_t n, size_t size, size_t num, ... )
{
    return heap_caps_calloc_base(n, size, MALLOC_CAP_DEFAULT);
}

size_t heap_caps_get_total_size(uint32_t caps)
{
    return K_HEAP_MEM_POOL_SIZE;
}

size_t heap_caps_get_free_size( uint32_t caps )
{
    // Not implemented
    return 0;
}

size_t heap_caps_get_minimum_free_size( uint32_t caps )
{
    // Not implemented
    return 0;
}

size_t heap_caps_get_largest_free_block( uint32_t caps )
{
    // Not implemented
    return 0;
}

void heap_caps_get_info( multi_heap_info_t *info, uint32_t caps )
{
    memset(info, 0, sizeof(multi_heap_info_t));
}

void heap_caps_print_heap_info( uint32_t caps )
{
    printf("No heap summary available when building for the zephyr target");
}

bool heap_caps_check_integrity(uint32_t caps, bool print_errors)
{
    return true;
}

bool heap_caps_check_integrity_all(bool print_errors)
{
    return heap_caps_check_integrity(MALLOC_CAP_INVALID, print_errors);
}

bool heap_caps_check_integrity_addr(intptr_t addr, bool print_errors)
{
    return true;
}

void heap_caps_dump(uint32_t caps)
{
    // Not implemented

}

void heap_caps_dump_all(void)
{
    heap_caps_dump(MALLOC_CAP_INVALID);
}

size_t heap_caps_get_allocated_size( void *ptr )
{
    return 0;
}

void *heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t caps)
{
    void *ptr = k_aligned_alloc(alignment, size);

    if (!ptr && size > 0) {
        heap_caps_alloc_failed(size, caps, __func__);
    }

    return ptr;
}

void heap_caps_aligned_free(void *ptr)
{
    heap_caps_free(ptr);
}

void *heap_caps_aligned_calloc(size_t alignment, size_t n, size_t size, uint32_t caps)
{
    size_t size_bytes;
    if (__builtin_mul_overflow(n, size, &size_bytes)) {
        return NULL;
    }

    void *ptr = heap_caps_aligned_alloc(alignment, size_bytes, caps);
    if (ptr != NULL) {
        memset(ptr, 0, size_bytes);
    }

    return ptr;
}
