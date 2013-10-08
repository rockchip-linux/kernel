/*
 * Utils for keeping track of which areas of RAM change during suspend code.
 *
 * Copyright (C) 2012 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>

#include <linux/addr_overlap.h>
#include <linux/io.h>
#include <linux/pm.h>

static LIST_HEAD(volatile_chunks);
static DEFINE_SPINLOCK(volatile_chunks_lock);

/**
 * Register a range of memory as volatile.
 *
 * @chunk: The chunk to register; memory for this structure must remain
 *	allocated until the chunk is unregistered.
 */
void pm_register_suspend_volatile(struct pm_suspend_volatile_chunk *chunk)
{
	unsigned long flags;

	spin_lock_irqsave(&volatile_chunks_lock, flags);
	list_add_tail(&chunk->list, &volatile_chunks);
	spin_unlock_irqrestore(&volatile_chunks_lock, flags);
}

/**
 * Unregister a range of memory as volatile.
 *
 * @chunk: The chunk to unregister.  Should be exact pointer passed to register.
 */
void pm_unregister_suspend_volatile(struct pm_suspend_volatile_chunk *chunk)
{
	unsigned long flags;

	spin_lock_irqsave(&volatile_chunks_lock, flags);
	list_del(&chunk->list);
	spin_unlock_irqrestore(&volatile_chunks_lock, flags);
}

/**
 * Return true if the chunk overlaps memory that is touched by suspend process.
 *
 * Checks both memory that was registered with pm_register_suspend_volatile()
 * and memory that was tagged with __suspend_volatile.
 *
 * As the list is not proected with locks, this code must only be called
 * from an uninterruptible context.
 *
 * @start: Physical start address of chunk to check.
 * @num_bytes: Size of the chunk to check in bytes.
 */
bool pm_does_overlap_suspend_volatile(phys_addr_t start, size_t num_bytes)
{
	struct pm_suspend_volatile_chunk *chunk;
	bool is_volatile = false;

	/* Bits marked at compile time as suspend volatile */
	if (phys_addrs_overlap(start, num_bytes,
			       virt_to_phys(__start_suspend_volatile_bss),
			       __stop_suspend_volatile_bss -
			       __start_suspend_volatile_bss))
		return true;

	/* Bits marked at compile time as suspend volatile */
	if (phys_addrs_overlap(start, num_bytes,
			       virt_to_phys(__start_suspend_volatile_data),
			       __stop_suspend_volatile_data -
			       __start_suspend_volatile_data))
		return true;

	/*
	 * Bits registered dynamically
	 *
	 * Normally a call to spin_lock_irqsave(&volatile_chunks_lock, flags);
	 * would go here.  However this code is only called with scheduling
	 * disabled so there is no need to lock the list.  It is also called
	 * often enough that the overhead involved is signficant (~100 ms)
	 */
	list_for_each_entry(chunk, &volatile_chunks, list) {
		if (phys_addrs_overlap(start, num_bytes,
				       chunk->start, chunk->num_bytes)) {
			is_volatile = true;
			break;
		}
	}
	return is_volatile;
}
