/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2015 Synopsys, Inc. (www.synopsys.com)
 */

#ifndef _ASM_HIGHMEM_H
#define _ASM_HIGHMEM_H

#ifdef CONFIG_HIGHMEM

#include <uapi/asm/page.h>
#include <asm/kmap_types.h>

/* start after vmalloc area */
#define FIXMAP_BASE		(PAGE_OFFSET - FIXMAP_SIZE - PKMAP_SIZE)
#define FIXMAP_SIZE		PGDIR_SIZE	/* only 1 PGD worth */
#define KM_TYPE_NR		((FIXMAP_SIZE >> PAGE_SHIFT)/NR_CPUS)

#define FIX_KMAP_BEGIN		(0)
#define FIX_KMAP_END		((FIXMAP_SIZE >> PAGE_SHIFT) - 1)
#define FIXADDR_TOP		(FIXMAP_BASE + FIXMAP_SIZE - PAGE_SIZE)

/* start after fixmap area */
#define PKMAP_BASE		(FIXMAP_BASE + FIXMAP_SIZE)
#define PKMAP_SIZE		PGDIR_SIZE
#define LAST_PKMAP		(PKMAP_SIZE >> PAGE_SHIFT)
#define LAST_PKMAP_MASK		(LAST_PKMAP - 1)
#define PKMAP_ADDR(nr)		(PKMAP_BASE + ((nr) << PAGE_SHIFT))
#define PKMAP_NR(virt)		(((virt) - PKMAP_BASE) >> PAGE_SHIFT)

#include <asm/cacheflush.h>

extern void kmap_init(void);

#define arch_kmap_local_post_unmap(vaddr)			\
	local_flush_tlb_kernel_range(vaddr, vaddr + PAGE_SIZE)

static inline void flush_cache_kmaps(void)
{
	flush_cache_all();
}

#endif

#endif
