// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2010-2014, 2016-2022 ARM Limited. All rights reserved.
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

#include "mali_kbase.h"
#include <gpu/mali_kbase_gpu_regmap.h>
#include "mali_kbase_defs.h"
#include <mmu/mali_kbase_mmu.h>
#include <mmu/mali_kbase_mmu_internal.h>

#define ENTRY_TYPE_MASK     3ULL
/* For valid ATEs bit 1 = ((level == 3) ? 1 : 0).
 * Valid ATE entries at level 3 are flagged with the value 3.
 * Valid ATE entries at level 0-2 are flagged with the value 1.
 */
#define ENTRY_IS_ATE_L3		3ULL
#define ENTRY_IS_ATE_L02	1ULL
#define ENTRY_IS_INVAL		2ULL
#define ENTRY_IS_PTE		3ULL

#define ENTRY_ACCESS_RW (1ULL << 6)     /* bits 6:7 */
#define ENTRY_ACCESS_RO (3ULL << 6)
#define ENTRY_ACCESS_BIT (1ULL << 10)
#define ENTRY_NX_BIT (1ULL << 54)

#define UNUSED_BIT_POSITION_IN_PAGE_DESCRIPTOR (55)
#define VALID_ENTRY_MASK ((u64)0xF << UNUSED_BIT_POSITION_IN_PAGE_DESCRIPTOR)

/* Helper Function to perform assignment of page table entries, to
 * ensure the use of strd, which is required on LPAE systems.
 */
static inline void page_table_entry_set(u64 *pte, u64 phy)
{
	WRITE_ONCE(*pte, phy);
}

static void mmu_update(struct kbase_device *kbdev, struct kbase_mmu_table *mmut,
		int as_nr)
{
	struct kbase_as *as;
	struct kbase_mmu_setup *current_setup;

	if (WARN_ON(as_nr == KBASEP_AS_NR_INVALID))
		return;

	as = &kbdev->as[as_nr];
	current_setup = &as->current_setup;

	kbase_mmu_get_as_setup(mmut, current_setup);

	/* Apply the address space setting */
	kbase_mmu_hw_configure(kbdev, as);
}

static void mmu_disable_as(struct kbase_device *kbdev, int as_nr)
{
	struct kbase_as * const as = &kbdev->as[as_nr];
	struct kbase_mmu_setup * const current_setup = &as->current_setup;

	current_setup->transtab = 0ULL;
	current_setup->transcfg = AS_TRANSCFG_ADRMODE_UNMAPPED;

	/* Apply the address space setting */
	kbase_mmu_hw_configure(kbdev, as);
}

static phys_addr_t pte_to_phy_addr(u64 entry)
{
	if (!(entry & 1))
		return 0;

	entry &= ~VALID_ENTRY_MASK;
	return entry & ~0xFFF;
}

static int ate_is_valid(u64 ate, int const level)
{
	if (level == MIDGARD_MMU_BOTTOMLEVEL)
		return ((ate & ENTRY_TYPE_MASK) == ENTRY_IS_ATE_L3);
	else
		return ((ate & ENTRY_TYPE_MASK) == ENTRY_IS_ATE_L02);
}

static int pte_is_valid(u64 pte, int const level)
{
	/* PTEs cannot exist at the bottom level */
	if (level == MIDGARD_MMU_BOTTOMLEVEL)
		return false;
	return ((pte & ENTRY_TYPE_MASK) == ENTRY_IS_PTE);
}

/*
 * Map KBASE_REG flags to MMU flags
 */
static u64 get_mmu_flags(unsigned long flags)
{
	u64 mmu_flags;

	/* store mem_attr index as 4:2 (macro called ensures 3 bits already) */
	mmu_flags = KBASE_REG_MEMATTR_VALUE(flags) << 2;

	/* Set access flags - note that AArch64 stage 1 does not support
	 * write-only access, so we use read/write instead
	 */
	if (flags & KBASE_REG_GPU_WR)
		mmu_flags |= ENTRY_ACCESS_RW;
	else if (flags & KBASE_REG_GPU_RD)
		mmu_flags |= ENTRY_ACCESS_RO;

	/* nx if requested */
	mmu_flags |= (flags & KBASE_REG_GPU_NX) ? ENTRY_NX_BIT : 0;

	if (flags & KBASE_REG_SHARE_BOTH) {
		/* inner and outer shareable */
		mmu_flags |= SHARE_BOTH_BITS;
	} else if (flags & KBASE_REG_SHARE_IN) {
		/* inner shareable coherency */
		mmu_flags |= SHARE_INNER_BITS;
	}

	return mmu_flags;
}

static void entry_set_ate(u64 *entry,
		struct tagged_addr phy,
		unsigned long flags,
		int const level)
{
	if (level == MIDGARD_MMU_BOTTOMLEVEL)
		page_table_entry_set(entry, as_phys_addr_t(phy) |
				get_mmu_flags(flags) |
				ENTRY_ACCESS_BIT | ENTRY_IS_ATE_L3);
	else
		page_table_entry_set(entry, as_phys_addr_t(phy) |
				get_mmu_flags(flags) |
				ENTRY_ACCESS_BIT | ENTRY_IS_ATE_L02);
}

static unsigned int get_num_valid_entries(u64 *pgd)
{
	register unsigned int num_of_valid_entries;

	num_of_valid_entries =
		(unsigned int)((pgd[2] & VALID_ENTRY_MASK) >>
			       (UNUSED_BIT_POSITION_IN_PAGE_DESCRIPTOR - 8));
	num_of_valid_entries |=
		(unsigned int)((pgd[1] & VALID_ENTRY_MASK) >>
			       (UNUSED_BIT_POSITION_IN_PAGE_DESCRIPTOR - 4));
	num_of_valid_entries |=
		(unsigned int)((pgd[0] & VALID_ENTRY_MASK) >>
			       (UNUSED_BIT_POSITION_IN_PAGE_DESCRIPTOR));

	return num_of_valid_entries;
}

static void set_num_valid_entries(u64 *pgd, unsigned int num_of_valid_entries)
{
	WARN_ON_ONCE(num_of_valid_entries > KBASE_MMU_PAGE_ENTRIES);

	pgd[0] &= ~VALID_ENTRY_MASK;
	pgd[0] |= ((u64)(num_of_valid_entries & 0xF)
		   << UNUSED_BIT_POSITION_IN_PAGE_DESCRIPTOR);

	pgd[1] &= ~VALID_ENTRY_MASK;
	pgd[1] |= ((u64)((num_of_valid_entries >> 4) & 0xF)
		   << UNUSED_BIT_POSITION_IN_PAGE_DESCRIPTOR);

	pgd[2] &= ~VALID_ENTRY_MASK;
	pgd[2] |= ((u64)((num_of_valid_entries >> 8) & 0xF)
		   << UNUSED_BIT_POSITION_IN_PAGE_DESCRIPTOR);
}

static void entry_set_pte(u64 *entry, phys_addr_t phy)
{
	page_table_entry_set(entry, (phy & PAGE_MASK) | ENTRY_ACCESS_BIT | ENTRY_IS_PTE);
}

static void entries_invalidate(u64 *entry, u32 count)
{
	u32 i;

	for (i = 0; i < count; i++)
		page_table_entry_set(entry + i, ENTRY_IS_INVAL);
}

static const struct kbase_mmu_mode aarch64_mode = { .update = mmu_update,
						    .get_as_setup = kbase_mmu_get_as_setup,
						    .disable_as = mmu_disable_as,
						    .pte_to_phy_addr = pte_to_phy_addr,
						    .ate_is_valid = ate_is_valid,
						    .pte_is_valid = pte_is_valid,
						    .entry_set_ate = entry_set_ate,
						    .entry_set_pte = entry_set_pte,
						    .entries_invalidate = entries_invalidate,
						    .get_num_valid_entries = get_num_valid_entries,
						    .set_num_valid_entries = set_num_valid_entries,
						    .flags = KBASE_MMU_MODE_HAS_NON_CACHEABLE };

struct kbase_mmu_mode const *kbase_mmu_mode_get_aarch64(void)
{
	return &aarch64_mode;
}
