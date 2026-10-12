/*
 * Copyright (c) 2026, ArtInChip Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* RT-Thread system heap backed by the in-tree TLSF allocator.
 *
 * Enabled by RT_USING_USERHEAP: these strong symbols override the RT_WEAK
 * fallbacks in kernel/rt-thread/src/kservice.c, so rt_malloc()/rt_free() and
 * friends allocate from a single TLSF pool covering the region that
 * rt_hw_board_init() hands to rt_system_heap_init() (the PSRAM_SW region on
 * boards that keep the QuickJS/PocketJS workload in the system heap).
 *
 * The stock memheap best-fit search degrades under the bursty alloc/free
 * churn of a JS engine; TLSF answers alloc and free in bounded time and
 * keeps fragmentation local, which is the same reason the baremetal
 * bootloader uses this allocator. Only drv_bare/tlsf_heap/tlsf.c is reused
 * here; the baremetal region setup in aic_tlsf.c does not apply.
 */
#include <rtconfig.h>
#include <rtthread.h>
#include <rthw.h>

#include "tlsf.h"

#ifdef RT_USING_USERHEAP

static tlsf_t system_tlsf;
static rt_size_t system_tlsf_total;
static rt_size_t system_tlsf_used;
static rt_size_t system_tlsf_max_used;

#if defined(RT_USING_HEAP_ISR)
#elif defined(RT_USING_MUTEX)
static struct rt_mutex _tlsf_lock;
#endif

#ifdef RT_USING_HOOK
static void (*rt_malloc_hook)(void *ptr, rt_size_t size);
static void (*rt_free_hook)(void *ptr);

void rt_malloc_sethook(void (*hook)(void *ptr, rt_size_t size))
{
    rt_malloc_hook = hook;
}
RTM_EXPORT(rt_malloc_sethook);

void rt_free_sethook(void (*hook)(void *ptr))
{
    rt_free_hook = hook;
}
RTM_EXPORT(rt_free_sethook);
#endif /* RT_USING_HOOK */

rt_inline void _heap_lock_init(void)
{
#if defined(RT_USING_HEAP_ISR)
#elif defined(RT_USING_MUTEX)
    rt_mutex_init(&_tlsf_lock, "heap", RT_IPC_FLAG_PRIO);
#endif
}

rt_inline rt_base_t _heap_lock(void)
{
#if defined(RT_USING_HEAP_ISR)
    return rt_hw_interrupt_disable();
#elif defined(RT_USING_MUTEX)
    if (rt_thread_self())
        return rt_mutex_take(&_tlsf_lock, RT_WAITING_FOREVER);
    else
        return RT_EOK;
#else
    rt_enter_critical();
    return RT_EOK;
#endif
}

rt_inline void _heap_unlock(rt_base_t level)
{
#if defined(RT_USING_HEAP_ISR)
    rt_hw_interrupt_enable(level);
#elif defined(RT_USING_MUTEX)
    RT_ASSERT(level == RT_EOK);
    if (rt_thread_self())
        rt_mutex_release(&_tlsf_lock);
#else
    rt_exit_critical();
#endif
}

static void _account_alloc(void *ptr)
{
    if (ptr != RT_NULL)
    {
        system_tlsf_used += tlsf_block_size(ptr);
        if (system_tlsf_used > system_tlsf_max_used)
            system_tlsf_max_used = system_tlsf_used;
    }
}

static void _account_free(void *ptr)
{
    if (ptr != RT_NULL)
        system_tlsf_used -= tlsf_block_size(ptr);
}

/**
 * @brief This function will init system heap.
 *
 * @param begin_addr the beginning address of system page.
 *
 * @param end_addr the end address of system page.
 */
void rt_system_heap_init(void *begin_addr, void *end_addr)
{
    rt_ubase_t begin_align = RT_ALIGN((rt_ubase_t)begin_addr, RT_ALIGN_SIZE);
    rt_ubase_t end_align   = RT_ALIGN_DOWN((rt_ubase_t)end_addr, RT_ALIGN_SIZE);

    RT_ASSERT(end_align > begin_align);

    system_tlsf_total = end_align - begin_align;
    system_tlsf_used = 0;
    system_tlsf_max_used = 0;
    system_tlsf = tlsf_create_with_pool((void *)begin_align, system_tlsf_total);
    RT_ASSERT(system_tlsf != RT_NULL);
    _heap_lock_init();
}
RTM_EXPORT(rt_system_heap_init);

/**
 * @brief Allocate a block of memory with a minimum of 'size' bytes.
 *
 * @param size is the minimum size of the requested block in bytes.
 *
 * @return the pointer to allocated memory or NULL if no free memory was found.
 */
void *rt_malloc(rt_size_t size)
{
    rt_base_t level;
    void *ptr;

    /* Enter critical zone */
    level = _heap_lock();
    /* allocate memory block from system heap */
    ptr = tlsf_malloc(system_tlsf, size);
    _account_alloc(ptr);
    /* Exit critical zone */
    _heap_unlock(level);
    /* call 'rt_malloc' hook */
    RT_OBJECT_HOOK_CALL(rt_malloc_hook, (ptr, size));
    return ptr;
}
RTM_EXPORT(rt_malloc);

/**
 * @brief This function will change the size of previously allocated memory block.
 *
 * @param rmem is the pointer to memory allocated by rt_malloc.
 *
 * @param newsize is the required new size.
 *
 * @return the changed memory block address.
 */
void *rt_realloc(void *rmem, rt_size_t newsize)
{
    rt_base_t level;
    void *nptr;

    /* Enter critical zone */
    level = _heap_lock();
    /* Change the size of previously allocated memory block */
    if (rmem != RT_NULL)
        _account_free(rmem);
    nptr = tlsf_realloc(system_tlsf, rmem, newsize);
    _account_alloc(nptr);
    /* Exit critical zone */
    _heap_unlock(level);
    return nptr;
}
RTM_EXPORT(rt_realloc);

/**
 * @brief  This function will contiguously allocate enough space for count
 *         objects that are size bytes of memory each and returns a pointer
 *         to the allocated memory.
 *
 * @note   The allocated memory is filled with bytes of value zero.
 *
 * @return pointer to allocated memory / NULL pointer if there is an error.
 */
void *rt_calloc(rt_size_t count, rt_size_t size)
{
    void *p;

    /* allocate 'count' objects of size 'size' */
    p = rt_malloc(count * size);
    /* zero the memory */
    if (p)
        rt_memset(p, 0, count * size);
    return p;
}
RTM_EXPORT(rt_calloc);

/**
 * @brief This function will release the previously allocated memory block by
 *        rt_malloc. The released memory block is taken back to system heap.
 *
 * @param rmem the address of memory which will be released.
 */
void rt_free(void *rmem)
{
    rt_base_t level;

    /* call 'rt_free' hook */
    RT_OBJECT_HOOK_CALL(rt_free_hook, (rmem));
    /* NULL check */
    if (rmem == RT_NULL) return;
    /* Enter critical zone */
    level = _heap_lock();
    _account_free(rmem);
    tlsf_free(system_tlsf, rmem);
    /* Exit critical zone */
    _heap_unlock(level);
}
RTM_EXPORT(rt_free);

/**
 * @brief Allocate a block of memory with a longer alignment starting at the
 *        beginning of the block.
 *
 * @param size is the minimum size of the requested block in bytes.
 *
 * @param align is the alignment size; rounded up to a power of two as
 *        required by tlsf_memalign().
 *
 * @return the pointer to allocated memory or NULL if no free memory was found.
 */
void *rt_malloc_align(rt_size_t size, rt_size_t align)
{
    rt_base_t level;
    void *ptr;

    if (align < RT_ALIGN_SIZE)
        align = RT_ALIGN_SIZE;
    else if (align & (align - 1))
    {
        rt_size_t up = RT_ALIGN_SIZE;
        while (up < align)
            up <<= 1;
        align = up;
    }

    /* Enter critical zone */
    level = _heap_lock();
    ptr = tlsf_memalign(system_tlsf, align, size);
    _account_alloc(ptr);
    /* Exit critical zone */
    _heap_unlock(level);
    return ptr;
}
RTM_EXPORT(rt_malloc_align);

/**
 * @brief This function will release the memory block, which is allocated by
 *        rt_malloc_align. tlsf_memalign() blocks are freed with the plain
 *        tlsf_free(), so this is the same as rt_free().
 *
 * @param ptr is the memory block pointer.
 */
void rt_free_align(void *ptr)
{
    rt_free(ptr);
}
RTM_EXPORT(rt_free_align);

/**
 * @brief This function will caculate the total memory, the used memory, and
 *        the max used memory.
 *
 * @param total is a pointer to get the total size of the memory.
 *
 * @param used is a pointer to get the size of memory used.
 *
 * @param max_used is a pointer to get the maximum memory used.
 */
void rt_memory_info(rt_size_t *total,
                    rt_size_t *used,
                    rt_size_t *max_used)
{
    rt_base_t level;

    /* Enter critical zone */
    level = _heap_lock();
    if (total)
        *total = system_tlsf_total;
    if (used)
        *used = system_tlsf_used;
    if (max_used)
        *max_used = system_tlsf_max_used;
    /* Exit critical zone */
    _heap_unlock(level);
}
RTM_EXPORT(rt_memory_info);

struct tlsf_walk_summary {
    rt_size_t free_total;
    rt_size_t largest_free;
    rt_size_t free_blocks;
};

static void tlsf_walk_cb(void *ptr, size_t size, int used, void *user)
{
    struct tlsf_walk_summary *summary = user;
    (void)ptr;
    if (!used)
    {
        summary->free_total += size;
        summary->free_blocks++;
        if (size > summary->largest_free)
            summary->largest_free = size;
    }
}

/**
 * @brief Walk the TLSF pool and summarize its free blocks: the total free
 *        bytes, the largest contiguous free block and the free block count.
 *
 * Sizes are the allocator's physical block sizes (header included), so the
 * largest request that can actually succeed is slightly below largest_free.
 * Meant for diagnostics at safe sample points only: it takes the heap lock
 * and walks every block.
 *
 * @param free_total is a pointer to get the sum of free block sizes.
 * @param largest_free is a pointer to get the biggest contiguous free block.
 * @param free_blocks is a pointer to get the number of free blocks.
 */
void rt_memory_walk(rt_size_t *free_total,
                    rt_size_t *largest_free,
                    rt_size_t *free_blocks)
{
    struct tlsf_walk_summary summary = {0, 0, 0};
    rt_base_t level;

    if (system_tlsf == RT_NULL)
        return;

    /* Enter critical zone */
    level = _heap_lock();
    tlsf_walk_pool(tlsf_get_pool(system_tlsf), tlsf_walk_cb, &summary);
    /* Exit critical zone */
    _heap_unlock(level);
    if (free_total)
        *free_total = summary.free_total;
    if (largest_free)
        *largest_free = summary.largest_free;
    if (free_blocks)
        *free_blocks = summary.free_blocks;
}
RTM_EXPORT(rt_memory_walk);

#endif /* RT_USING_USERHEAP */
