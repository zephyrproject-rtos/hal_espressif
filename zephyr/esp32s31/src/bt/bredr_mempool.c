/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/sys_heap.h>
#include <zephyr/sys/util.h>

#include "esp_bt.h"

/* Pool backend of the BR/EDR controller mempool, on top of sys_heap. */

orca_mempool_t tlsf_create_with_pool(void *mem, size_t pool_bytes, size_t max_bytes)
{
	uintptr_t start = ROUND_UP((uintptr_t)mem, __alignof__(struct sys_heap));
	uintptr_t end = (uintptr_t)mem + pool_bytes;
	struct sys_heap *heap = (struct sys_heap *)start;
	uintptr_t heap_mem = start + sizeof(*heap);

	ARG_UNUSED(max_bytes);

	if ((mem == NULL) || (heap_mem >= end)) {
		return NULL;
	}

	sys_heap_init(heap, (void *)heap_mem, end - heap_mem);

	return heap;
}

void *tlsf_malloc(orca_mempool_t pool, size_t size)
{
	return sys_heap_alloc(pool, size);
}

void tlsf_free(orca_mempool_t pool, void *ptr)
{
	sys_heap_free(pool, ptr);
}
