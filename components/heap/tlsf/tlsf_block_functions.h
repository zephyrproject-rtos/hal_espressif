/*
 * SPDX-FileCopyrightText: 2006-2016 Matthew Conte
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

/*
** Constants definition for poisoning.
** These defines are used as 3rd argument of tlsf_poison_fill_region() for readability purposes.
*/
#define POISONING_AFTER_FREE true
#define POISONING_AFTER_MALLOC !POISONING_AFTER_FREE

/* A type used for casting when doing pointer arithmetic. */
typedef ptrdiff_t tlsfptr_t;

/*
** Cast and min/max macros.
*/
#if !defined (tlsf_cast)
#define tlsf_cast(t, exp)	((t) (exp))
#endif
#if !defined (tlsf_min)
#define tlsf_min(a, b)		((a) < (b) ? (a) : (b))
#endif
#if !defined (tlsf_max)
#define tlsf_max(a, b)		((a) > (b) ? (a) : (b))
#endif

/*
** Set assert macro, if it has not been provided by the user.
*/
#if !defined (tlsf_assert)
#define tlsf_assert assert
#endif

typedef struct block_header_t
{
	/* Points to the previous physical block. */
	struct block_header_t* prev_phys_block;

	/* The size of this block, excluding the block header. */
	size_t size;

	/* Next and previous free blocks. */
	struct block_header_t* next_free;
	struct block_header_t* prev_free;
} block_header_t;

/* User data starts directly after the size field in a used block. */
#define block_start_offset (offsetof(block_header_t, size) + sizeof(size_t))

/*
** A free block must be large enough to store its header minus the size of
** the prev_phys_block field, and no larger than the number of addressable
** bits for FL_INDEX.
*/
#define block_size_min (sizeof(block_header_t) - sizeof(block_header_t*))

/*
** Since block sizes are always at least a multiple of 4, the two least
** significant bits of the size field are used to store the block status:
** - bit 0: whether block is busy or free
** - bit 1: whether previous block is busy or free
*/
#define block_header_free_bit (1UL << 0)
#define block_header_prev_free_bit (1UL << 1)

/*
** The size of the block header exposed to used blocks is the size field.
** The prev_phys_block field is stored *inside* the previous free block.
*/
#define block_header_overhead (sizeof(size_t))

/*
** block_header_t member functions.
*/

static inline __attribute__((always_inline)) size_t block_size(const block_header_t* block)
{
	return block->size & ~(block_header_free_bit | block_header_prev_free_bit);
}

static inline __attribute__((always_inline)) void block_set_size(block_header_t* block, size_t size)
{
	const size_t oldsize = block->size;
	block->size = size | (oldsize & (block_header_free_bit | block_header_prev_free_bit));
}

static inline __attribute__((always_inline)) int block_is_last(const block_header_t* block)
{
	return block_size(block) == 0;
}

static inline __attribute__((always_inline)) int block_is_free(const block_header_t* block)
{
	return tlsf_cast(int, block->size & block_header_free_bit);
}

static inline __attribute__((always_inline)) void block_set_free(block_header_t* block)
{
	block->size |= block_header_free_bit;
}

static inline __attribute__((always_inline)) void block_set_used(block_header_t* block)
{
	block->size &= ~block_header_free_bit;
}

static inline __attribute__((always_inline)) int block_is_prev_free(const block_header_t* block)
{
	return tlsf_cast(int, block->size & block_header_prev_free_bit);
}

static inline __attribute__((always_inline)) void block_set_prev_free(block_header_t* block)
{
	block->size |= block_header_prev_free_bit;
}

static inline __attribute__((always_inline)) void block_set_prev_used(block_header_t* block)
{
	block->size &= ~block_header_prev_free_bit;
}

static inline __attribute__((always_inline)) block_header_t* block_from_ptr(const void* ptr)
{
	return tlsf_cast(block_header_t*,
		tlsf_cast(unsigned char*, ptr) - block_start_offset);
}

static inline __attribute__((always_inline)) void* block_to_ptr(const block_header_t* block)
{
	return tlsf_cast(void*,
		tlsf_cast(unsigned char*, block) + block_start_offset);
}

/* Return location of next block after block of given size. */
static inline __attribute__((always_inline)) block_header_t* offset_to_block(const void* ptr, size_t size)
{
	return tlsf_cast(block_header_t*, tlsf_cast(tlsfptr_t, ptr) + size);
}

/* Return location of previous block. */
static inline __attribute__((always_inline)) block_header_t* block_prev(const block_header_t* block)
{
	tlsf_assert(block_is_prev_free(block) && "previous block must be free");
	return block->prev_phys_block;
}

/* Return location of next existing block. */
static inline __attribute__((always_inline)) block_header_t* block_next(const block_header_t* block)
{
	block_header_t* next = offset_to_block(block_to_ptr(block),
		block_size(block) - block_header_overhead);
	tlsf_assert(!block_is_last(block));
	return next;
}

/* Link a new block with its physical neighbor, return the neighbor. */
static inline __attribute__((always_inline)) block_header_t* block_link_next(block_header_t* block)
{
	block_header_t* next = block_next(block);
	next->prev_phys_block = block;
	return next;
}

static inline __attribute__((always_inline)) void block_mark_as_free(block_header_t* block)
{
	/* Link the block to the next block, first. */
	block_header_t* next = block_link_next(block);
	block_set_prev_free(next);
	block_set_free(block);
}

static inline __attribute__((always_inline)) void block_mark_as_used(block_header_t* block)
{
	block_header_t* next = block_next(block);
	block_set_prev_used(next);
	block_set_used(block);
}

#if defined(__cplusplus)
};
#endif
