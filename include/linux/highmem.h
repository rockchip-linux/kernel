/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_HIGHMEM_H
#define _LINUX_HIGHMEM_H

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/hardirq.h>

#include <asm/cacheflush.h>

#ifndef ARCH_HAS_FLUSH_ANON_PAGE
static inline void flush_anon_page(struct vm_area_struct *vma, struct page *page, unsigned long vmaddr)
{
}
#endif

#ifndef ARCH_HAS_FLUSH_KERNEL_DCACHE_PAGE
static inline void flush_kernel_dcache_page(struct page *page)
{
}
static inline void flush_kernel_vmap_range(void *vaddr, int size)
{
}
static inline void invalidate_kernel_vmap_range(void *vaddr, int size)
{
}
#endif

#include <asm/kmap_types.h>

/*
 * Outside of CONFIG_HIGHMEM to support X86 32bit iomap_atomic() cruft.
 */
#ifdef CONFIG_KMAP_LOCAL
void *__kmap_local_pfn_prot(unsigned long pfn, pgprot_t prot);
void *__kmap_local_page_prot(struct page *page, pgprot_t prot);
void kunmap_local_indexed(void *vaddr);
void kmap_local_fork(struct task_struct *tsk);
void __kmap_local_sched_out(void);
void __kmap_local_sched_in(void);
static inline void kmap_assert_nomap(void)
{
	DEBUG_LOCKS_WARN_ON(current->kmap_ctrl.idx);
}
#else
static inline void kmap_local_fork(struct task_struct *tsk) { }
static inline void kmap_assert_nomap(void) { }
#endif

#ifdef CONFIG_HIGHMEM
#include <asm/highmem.h>

#ifndef ARCH_HAS_KMAP_FLUSH_TLB
static inline void kmap_flush_tlb(unsigned long addr) { }
#endif

#ifndef kmap_prot
#define kmap_prot PAGE_KERNEL
#endif

void *kmap_high(struct page *page);
static inline void *kmap(struct page *page)
{
	void *addr;

	might_sleep();
	if (!PageHighMem(page))
		addr = page_address(page);
	else
		addr = kmap_high(page);
	kmap_flush_tlb((unsigned long)addr);
	return addr;
}

void kunmap_high(struct page *page);

static inline void kunmap(struct page *page)
{
	might_sleep();
	if (!PageHighMem(page))
		return;
	kunmap_high(page);
}

/*
 * For highmem systems it is required to temporarily map pages
 * which reside in the portion of memory which is not covered
 * by the permanent kernel mapping.
 *
 * This comes in three flavors:
 *
 * 1) kmap/kunmap:
 *
 *    An interface to acquire longer term mappings with no restrictions
 *    on preemption and migration. This comes with an overhead as the
 *    mapping space is restricted and protected by a global lock. It
 *    also requires global TLB invalidation when the kmap pool wraps.
 *
 *    kmap() might block when the mapping space is fully utilized until a
 *    slot becomes available. Only callable from preemptible thread
 *    context.
 *
 * 2) kmap_local.*()/kunmap_local.*()
 *
 *    An interface to acquire short term mappings. Can be invoked from any
 *    context including interrupts. The mapping is per thread, CPU local
 *    and not globaly visible. It can only be used in the context which
 *    acquried the mapping. Nesting kmap_local.*() and kmap_atomic.*()
 *    mappings is allowed to a certain extent (up to KMAP_TYPE_NR).
 *
 *    Nested kmap_local.*() and kunmap_local.*() invocations have to be
 *    strictly ordered because the map implementation is stack based.
 *
 *    kmap_local.*() disables migration, but keeps preemption enabled. It's
 *    valid to take pagefaults in a kmap_local region unless the context in
 *    which the local kmap is acquired does not allow it for other reasons.
 *
 *    If a task holding local kmaps is preempted, the maps are removed on
 *    context switch and restored when the task comes back on the CPU. As
 *    the maps are strictly CPU local it is guaranteed that the task stays
 *    on the CPU and the CPU cannot be unplugged until the local kmaps are
 *    released.
 *
 * 3) kmap_atomic.*()/kunmap_atomic.*()
 *
 *    Based on the same mechanism as kmap local. Atomic kmap disables
 *    preemption and pagefaults. Only use if absolutely required, use
 *    the corresponding kmap_local variant if possible.
 *
 * Local and atomic kmaps are faster than kmap/kunmap, but impose
 * restrictions. Only use them when required.
 *
 * For !HIGHMEM enabled systems the kmap flavours are not doing any mapping
 * operation and kmap() won't sleep, but the kmap local and atomic variants
 * still disable migration resp. pagefaults and preemption.
 */
static inline void *kmap_atomic_prot(struct page *page, pgprot_t prot)
{
	if (IS_ENABLED(CONFIG_PREEMPT_RT))
		migrate_disable();
	else
		preempt_disable();
	pagefault_disable();
	return __kmap_local_page_prot(page, prot);
}

static inline void *kmap_atomic(struct page *page)
{
	return kmap_atomic_prot(page, kmap_prot);
}

static inline void *kmap_atomic_pfn(unsigned long pfn)
{
	if (IS_ENABLED(CONFIG_PREEMPT_RT))
		migrate_disable();
	else
		preempt_disable();
	pagefault_disable();
	return __kmap_local_pfn_prot(pfn, kmap_prot);
}

static inline void __kunmap_atomic(void *addr)
{
	kunmap_local_indexed(addr);
}

static inline void *kmap_local_page_prot(struct page *page, pgprot_t prot)
{
	migrate_disable();
	return __kmap_local_page_prot(page, prot);
}

static inline void *kmap_local_page(struct page *page)
{
	return kmap_local_page_prot(page, kmap_prot);
}

static inline void *kmap_local_pfn(unsigned long pfn)
{
	migrate_disable();
	return __kmap_local_pfn_prot(pfn, kmap_prot);
}

static inline void __kunmap_local(void *vaddr)
{
	kunmap_local_indexed(vaddr);
}

/* declarations for linux/mm/highmem.c */
unsigned int nr_free_highpages(void);
extern atomic_long_t _totalhigh_pages;
static inline unsigned long totalhigh_pages(void)
{
	return (unsigned long)atomic_long_read(&_totalhigh_pages);
}

static inline void totalhigh_pages_inc(void)
{
	atomic_long_inc(&_totalhigh_pages);
}

static inline void totalhigh_pages_dec(void)
{
	atomic_long_dec(&_totalhigh_pages);
}

static inline void totalhigh_pages_add(long count)
{
	atomic_long_add(count, &_totalhigh_pages);
}

static inline void totalhigh_pages_set(long val)
{
	atomic_long_set(&_totalhigh_pages, val);
}

void kmap_flush_unused(void);

struct page *kmap_to_page(void *addr);

#else /* CONFIG_HIGHMEM */

static inline unsigned int nr_free_highpages(void) { return 0; }

static inline struct page *kmap_to_page(void *addr)
{
	return virt_to_page(addr);
}

static inline unsigned long totalhigh_pages(void) { return 0UL; }

static inline void *kmap(struct page *page)
{
	might_sleep();
	return page_address(page);
}

static inline void kunmap_high(struct page *page)
{
}

static inline void kunmap(struct page *page)
{
#ifdef ARCH_HAS_FLUSH_ON_KUNMAP
	kunmap_flush_on_unmap(page_address(page));
#endif
}

static inline void *kmap_atomic(struct page *page)
{
	if (IS_ENABLED(CONFIG_PREEMPT_RT))
		migrate_disable();
	else
		preempt_disable();
	pagefault_disable();
	return page_address(page);
}

static inline void *kmap_atomic_prot(struct page *page, pgprot_t prot)
{
	return kmap_atomic(page);
}

static inline void *kmap_atomic_pfn(unsigned long pfn)
{
	return kmap_atomic(pfn_to_page(pfn));
}

static inline void __kunmap_local(void *addr)
{
#ifdef ARCH_HAS_FLUSH_ON_KUNMAP
	kunmap_flush_on_unmap(addr);
#endif
}

static inline void __kunmap_atomic(void *addr)
{
	__kunmap_local(addr);
}

static inline void *kmap_local_page(struct page *page)
{
	migrate_disable();
	return page_address(page);
}

static inline void *kmap_local_page_prot(struct page *page, pgprot_t prot)
{
	return kmap_local_page(page);
}

static inline void *kmap_local_pfn(unsigned long pfn)
{
	return kmap_local_page(pfn_to_page(pfn));
}

#define kmap_flush_unused()	do {} while(0)

#endif /* CONFIG_HIGHMEM */

/*
 * Prevent people trying to call kunmap_atomic() as if it were kunmap()
 * kunmap_atomic() should get the return value of kmap_atomic, not the page.
 */
#define kunmap_atomic(__addr)					\
do {								\
	BUILD_BUG_ON(__same_type((__addr), struct page *));	\
	__kunmap_atomic(__addr);				\
	pagefault_enable();					\
	if (IS_ENABLED(CONFIG_PREEMPT_RT))			\
		migrate_enable();				\
	else							\
		preempt_enable();				\
} while (0)

#define kunmap_local(__addr)					\
do {								\
	BUILD_BUG_ON(__same_type((__addr), struct page *));	\
	__kunmap_local(__addr);					\
	migrate_enable();					\
} while (0)

/* when CONFIG_HIGHMEM is not set these will be plain clear/copy_page */
#ifndef clear_user_highpage
static inline void clear_user_highpage(struct page *page, unsigned long vaddr)
{
	void *addr = kmap_atomic(page);
	clear_user_page(addr, vaddr, page);
	kunmap_atomic(addr);
}
#endif

#ifndef __HAVE_ARCH_ALLOC_ZEROED_USER_HIGHPAGE
/**
 * __alloc_zeroed_user_highpage - Allocate a zeroed HIGHMEM page for a VMA with caller-specified movable GFP flags
 * @movableflags: The GFP flags related to the pages future ability to move like __GFP_MOVABLE
 * @vma: The VMA the page is to be allocated for
 * @vaddr: The virtual address the page will be inserted into
 *
 * This function will allocate a page for a VMA but the caller is expected
 * to specify via movableflags whether the page will be movable in the
 * future or not
 *
 * An architecture may override this function by defining
 * __HAVE_ARCH_ALLOC_ZEROED_USER_HIGHPAGE and providing their own
 * implementation.
 */
static inline struct page *
__alloc_zeroed_user_highpage(gfp_t movableflags,
			struct vm_area_struct *vma,
			unsigned long vaddr)
{
	struct page *page = alloc_page_vma(GFP_HIGHUSER | movableflags,
			vma, vaddr);

	if (page)
		clear_user_highpage(page, vaddr);

	return page;
}
#endif

/**
 * alloc_zeroed_user_highpage_movable - Allocate a zeroed HIGHMEM page for a VMA that the caller knows can move
 * @vma: The VMA the page is to be allocated for
 * @vaddr: The virtual address the page will be inserted into
 *
 * This function will allocate a page for a VMA that the caller knows will
 * be able to migrate in the future using move_pages() or reclaimed
 */
static inline struct page *
alloc_zeroed_user_highpage_movable(struct vm_area_struct *vma,
					unsigned long vaddr)
{
	return __alloc_zeroed_user_highpage(__GFP_MOVABLE, vma, vaddr);
}

static inline void clear_highpage(struct page *page)
{
	void *kaddr = kmap_atomic(page);
	clear_page(kaddr);
	kunmap_atomic(kaddr);
}

static inline void zero_user_segments(struct page *page,
	unsigned start1, unsigned end1,
	unsigned start2, unsigned end2)
{
	void *kaddr = kmap_atomic(page);

	BUG_ON(end1 > PAGE_SIZE || end2 > PAGE_SIZE);

	if (end1 > start1)
		memset(kaddr + start1, 0, end1 - start1);

	if (end2 > start2)
		memset(kaddr + start2, 0, end2 - start2);

	kunmap_atomic(kaddr);
	flush_dcache_page(page);
}

static inline void zero_user_segment(struct page *page,
	unsigned start, unsigned end)
{
	zero_user_segments(page, start, end, 0, 0);
}

static inline void zero_user(struct page *page,
	unsigned start, unsigned size)
{
	zero_user_segments(page, start, start + size, 0, 0);
}

#ifndef __HAVE_ARCH_COPY_USER_HIGHPAGE

static inline void copy_user_highpage(struct page *to, struct page *from,
	unsigned long vaddr, struct vm_area_struct *vma)
{
	char *vfrom, *vto;

	vfrom = kmap_atomic(from);
	vto = kmap_atomic(to);
	copy_user_page(vto, vfrom, vaddr, to);
	kunmap_atomic(vto);
	kunmap_atomic(vfrom);
}

#endif

#ifndef __HAVE_ARCH_COPY_HIGHPAGE

static inline void copy_highpage(struct page *to, struct page *from)
{
	char *vfrom, *vto;

	vfrom = kmap_atomic(from);
	vto = kmap_atomic(to);
	copy_page(vto, vfrom);
	kunmap_atomic(vto);
	kunmap_atomic(vfrom);
}

#endif

#endif /* _LINUX_HIGHMEM_H */
