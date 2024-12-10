/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2019-2022 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#ifndef _KBASE_MEM_POOL_GROUP_H_
#define _KBASE_MEM_POOL_GROUP_H_

#include <mali_kbase_defs.h>

/**
 * kbase_mem_pool_group_select() - Select the memory pool to use.
 *
 * @kbdev:         Device pointer.
 * @mem_group_id:  Physical memory group ID to use.
 * @is_small_page: Flag used to select between the small and
 *                 large memory pool.
 *
 * Return: A pointer to the selected memory pool.
 */
static inline struct kbase_mem_pool *kbase_mem_pool_group_select(
	struct kbase_device *kbdev, u32 mem_group_id, bool is_small_page)
{
	if (WARN_ON(unlikely(kbdev == NULL)))
		return NULL;

	WARN_ON(mem_group_id > BASE_MEM_GROUP_COUNT);

	if (is_small_page)
		return &kbdev->mem_pools.small[mem_group_id];

	return &kbdev->mem_pools.large[mem_group_id];
}

/**
 * kbase_mem_pool_group_config_set_max_size - Set the initial configuration for
 * a set of memory pools
 *
 * @configs:  Initial configuration for the set of memory pools
 * @max_size: Maximum number of free 4 KiB pages each pool can hold
 *
 * This function sets the initial configuration for every memory pool so that
 * the maximum amount of free memory that each pool can hold is identical.
 * The equivalent number of 2 MiB pages is calculated automatically for the
 * purpose of configuring the large page pools.
 */
void kbase_mem_pool_group_config_set_max_size(
	struct kbase_mem_pool_group_config *configs, size_t max_size);

/**
 * kbase_mem_pool_group_init - Initialize a set of memory pools
 *
 * @mem_pools:  Set of memory pools to initialize
 * @kbdev:      Kbase device where memory is used
 * @configs:    Initial configuration for the set of memory pools
 * @next_pools: Set of memory pools from which to allocate memory if there
 *              is no free memory in one of the @mem_pools
 *
 * Initializes a complete set of physical memory pools. Memory pools are used to
 * allow efficient reallocation of previously-freed physical pages. A pair of
 * memory pools is initialized for each physical memory group: one for 4 KiB
 * pages and one for 2 MiB pages.
 *
 * If @next_pools is not NULL then a request to allocate memory from an
 * empty pool in @mem_pools will attempt to allocate from the equivalent pool
 * in @next_pools before going to the memory group manager. Similarly
 * pages can spill over to the equivalent pool in @next_pools when a pool
 * is full in @mem_pools. Pages are zeroed before they spill over to another
 * pool, to prevent leaking information between applications.
 *
 * Return: 0 on success, otherwise a negative error code
 */
int kbase_mem_pool_group_init(struct kbase_mem_pool_group *mem_pools, struct kbase_device *kbdev,
			      const struct kbase_mem_pool_group_config *configs,
			      struct kbase_mem_pool_group *next_pools);

/**
 * kbase_mem_pool_group_mark_dying - Mark a set of memory pools as dying
 *
 * @mem_pools: Set of memory pools to mark
 *
 * Marks a complete set of physical memory pools previously initialized by
 * @kbase_mem_pool_group_init as dying. This will cause any ongoing allocation
 * operations (eg growing on page fault) to be terminated.
 */
void kbase_mem_pool_group_mark_dying(struct kbase_mem_pool_group *mem_pools);

/**
 * kbase_mem_pool_group_term - Terminate a set of memory pools
 *
 * @mem_pools: Set of memory pools to terminate
 *
 * Terminates a complete set of physical memory pools previously initialized by
 * @kbase_mem_pool_group_init.
 */
void kbase_mem_pool_group_term(struct kbase_mem_pool_group *mem_pools);

#endif /* _KBASE_MEM_POOL_GROUP_H_ */
