// SPDX-License-Identifier: GPL-2.0-only
/*
 * Dynamic DMA mapping support.
 *
 * This implementation is a fallback for platforms that do not support
 * I/O TLBs (aka DMA address translation hardware).
 * Copyright (C) 2000 Asit Mallick <Asit.K.Mallick@intel.com>
 * Copyright (C) 2000 Goutham Rao <goutham.rao@intel.com>
 * Copyright (C) 2000, 2003 Hewlett-Packard Co
 *	David Mosberger-Tang <davidm@hpl.hp.com>
 *
 * 03/05/07 davidm	Switch from PCI-DMA to generic device DMA API.
 * 00/12/13 davidm	Rename to swiotlb.c and add mark_clean() to avoid
 *			unnecessary i-cache flushing.
 * 04/07/.. ak		Better overflow handling. Assorted fixes.
 * 05/09/10 linville	Add support for syncing ranges, support syncing for
 *			DMA_BIDIRECTIONAL mappings, miscellaneous cleanup.
 * 08/12/11 beckyb	Add highmem support
 */

#define pr_fmt(fmt) "software IO TLB: " fmt

#include <linux/cache.h>
#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/mm.h>
#include <linux/export.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/swiotlb.h>
#include <linux/pfn.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/highmem.h>
#include <linux/gfp.h>
#include <linux/scatterlist.h>
#include <linux/mem_encrypt.h>
#include <linux/set_memory.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#include <asm/io.h>
#include <asm/dma.h>

#include <linux/init.h>
#include <linux/memblock.h>
#include <linux/iommu-helper.h>

#define CREATE_TRACE_POINTS
#include <trace/events/swiotlb.h>

#define SLABS_PER_PAGE (1 << (PAGE_SHIFT - IO_TLB_SHIFT))

/*
 * Minimum IO TLB size to bother booting with.  Systems with mainly
 * 64bit capable cards will only lightly use the swiotlb.  If we can't
 * allocate a contiguous 1MB, we're probably in trouble anyway.
 */
#define IO_TLB_MIN_SLABS ((1<<20) >> IO_TLB_SHIFT)

enum swiotlb_force swiotlb_force;

/*
 * Used to do a quick range check in swiotlb_tbl_unmap_single and
 * swiotlb_tbl_sync_single_*, to see if the memory was in fact allocated by this
 * API.
 */
phys_addr_t io_tlb_start, io_tlb_end;

/*
 * The number of IO TLB blocks (in groups of 64) between io_tlb_start and
 * io_tlb_end.  This is command line adjustable via setup_io_tlb_npages.
 */
static unsigned long io_tlb_nslabs;

/*
 * The number of used IO TLB block
 */
static unsigned long io_tlb_used;

/*
 * This is a free list describing the number of free entries available from
 * each index
 */
static unsigned int *io_tlb_list;
static unsigned int io_tlb_index;

/*
 * Max segment that we can provide which (if pages are contingous) will
 * not be bounced (unless SWIOTLB_FORCE is set).
 */
static unsigned int max_segment;

/*
 * We need to save away the original address corresponding to a mapped entry
 * for the sync operations.
 */
#define INVALID_PHYS_ADDR (~(phys_addr_t)0)
static phys_addr_t *io_tlb_orig_addr;

/*
 * Protect the above data structures in the map and unmap calls
 */
static DEFINE_SPINLOCK(io_tlb_lock);

static int late_alloc;

static int __init
setup_io_tlb_npages(char *str)
{
	if (isdigit(*str)) {
		io_tlb_nslabs = simple_strtoul(str, &str, 0);
		/* avoid tail segment of size < IO_TLB_SEGSIZE */
		io_tlb_nslabs = ALIGN(io_tlb_nslabs, IO_TLB_SEGSIZE);
	}
	if (*str == ',')
		++str;
	if (!strcmp(str, "force")) {
		swiotlb_force = SWIOTLB_FORCE;
	} else if (!strcmp(str, "noforce")) {
		swiotlb_force = SWIOTLB_NO_FORCE;
		io_tlb_nslabs = 1;
	}

	return 0;
}
early_param("swiotlb", setup_io_tlb_npages);

static bool no_iotlb_memory;

unsigned long swiotlb_nr_tbl(void)
{
	return unlikely(no_iotlb_memory) ? 0 : io_tlb_nslabs;
}
EXPORT_SYMBOL_GPL(swiotlb_nr_tbl);

unsigned int swiotlb_max_segment(void)
{
	return unlikely(no_iotlb_memory) ? 0 : max_segment;
}
EXPORT_SYMBOL_GPL(swiotlb_max_segment);

void swiotlb_set_max_segment(unsigned int val)
{
	if (swiotlb_force == SWIOTLB_FORCE)
		max_segment = 1;
	else
		max_segment = rounddown(val, PAGE_SIZE);
}

/* default to 64MB */
#define IO_TLB_DEFAULT_SIZE (64UL<<20)
unsigned long swiotlb_size_or_default(void)
{
	unsigned long size;

	size = io_tlb_nslabs << IO_TLB_SHIFT;

	return size ? size : (IO_TLB_DEFAULT_SIZE);
}

void swiotlb_print_info(void)
{
	unsigned long bytes = io_tlb_nslabs << IO_TLB_SHIFT;

	if (no_iotlb_memory) {
		pr_warn("No low mem\n");
		return;
	}

	pr_info("mapped [mem %pa-%pa] (%luMB)\n", &io_tlb_start, &io_tlb_end,
	       bytes >> 20);
}

static inline unsigned long io_tlb_offset(unsigned long val)
{
	return val & (IO_TLB_SEGSIZE - 1);
}

static inline unsigned long nr_slots(u64 val)
{
	return DIV_ROUND_UP(val, IO_TLB_SIZE);
}

/*
 * Early SWIOTLB allocation may be too early to allow an architecture to
 * perform the desired operations.  This function allows the architecture to
 * call SWIOTLB when the operations are possible.  It needs to be called
 * before the SWIOTLB memory is used.
 */
void __init swiotlb_update_mem_attributes(void)
{
	void *vaddr;
	unsigned long bytes;

	if (no_iotlb_memory || late_alloc)
		return;

	vaddr = phys_to_virt(io_tlb_start);
	bytes = PAGE_ALIGN(io_tlb_nslabs << IO_TLB_SHIFT);
	set_memory_decrypted((unsigned long)vaddr, bytes >> PAGE_SHIFT);
	memset(vaddr, 0, bytes);
}

int __init swiotlb_init_with_tbl(char *tlb, unsigned long nslabs, int verbose)
{
	unsigned long i, bytes;
	size_t alloc_size;

	bytes = nslabs << IO_TLB_SHIFT;

	io_tlb_nslabs = nslabs;
	io_tlb_start = __pa(tlb);
	io_tlb_end = io_tlb_start + bytes;

	/*
	 * Allocate and initialize the free list array.  This array is used
	 * to find contiguous free memory regions of size up to IO_TLB_SEGSIZE
	 * between io_tlb_start and io_tlb_end.
	 */
	alloc_size = PAGE_ALIGN(io_tlb_nslabs * sizeof(int));
	io_tlb_list = memblock_alloc(alloc_size, PAGE_SIZE);
	if (!io_tlb_list)
		panic("%s: Failed to allocate %zu bytes align=0x%lx\n",
		      __func__, alloc_size, PAGE_SIZE);

	alloc_size = PAGE_ALIGN(io_tlb_nslabs * sizeof(phys_addr_t));
	io_tlb_orig_addr = memblock_alloc(alloc_size, PAGE_SIZE);
	if (!io_tlb_orig_addr)
		panic("%s: Failed to allocate %zu bytes align=0x%lx\n",
		      __func__, alloc_size, PAGE_SIZE);

	for (i = 0; i < io_tlb_nslabs; i++) {
		io_tlb_list[i] = IO_TLB_SEGSIZE - io_tlb_offset(i);
		io_tlb_orig_addr[i] = INVALID_PHYS_ADDR;
	}
	io_tlb_index = 0;
	no_iotlb_memory = false;

	if (verbose)
		swiotlb_print_info();

	swiotlb_set_max_segment(io_tlb_nslabs << IO_TLB_SHIFT);
	return 0;
}

/*
 * Statically reserve bounce buffer space and initialize bounce buffer data
 * structures for the software IO TLB used to implement the DMA API.
 */
void  __init
swiotlb_init(int verbose)
{
	size_t default_size = IO_TLB_DEFAULT_SIZE;
	unsigned char *vstart;
	unsigned long bytes;

	if (!io_tlb_nslabs) {
		io_tlb_nslabs = (default_size >> IO_TLB_SHIFT);
		io_tlb_nslabs = ALIGN(io_tlb_nslabs, IO_TLB_SEGSIZE);
	}

	bytes = io_tlb_nslabs << IO_TLB_SHIFT;

	/* Get IO TLB memory from the low pages */
	vstart = memblock_alloc_low(PAGE_ALIGN(bytes), PAGE_SIZE);
	if (vstart && !swiotlb_init_with_tbl(vstart, io_tlb_nslabs, verbose))
		return;

	if (io_tlb_start) {
		memblock_free_early(io_tlb_start,
				    PAGE_ALIGN(io_tlb_nslabs << IO_TLB_SHIFT));
		io_tlb_start = 0;
	}
	pr_warn("Cannot allocate buffer");
	no_iotlb_memory = true;
}

/*
 * Systems with larger DMA zones (those that don't support ISA) can
 * initialize the swiotlb later using the slab allocator if needed.
 * This should be just like above, but with some error catching.
 */
int
swiotlb_late_init_with_default_size(size_t default_size)
{
	unsigned long bytes, req_nslabs = io_tlb_nslabs;
	unsigned char *vstart = NULL;
	unsigned int order;
	int rc = 0;

	if (!io_tlb_nslabs) {
		io_tlb_nslabs = (default_size >> IO_TLB_SHIFT);
		io_tlb_nslabs = ALIGN(io_tlb_nslabs, IO_TLB_SEGSIZE);
	}

	/*
	 * Get IO TLB memory from the low pages
	 */
	order = get_order(io_tlb_nslabs << IO_TLB_SHIFT);
	io_tlb_nslabs = SLABS_PER_PAGE << order;
	bytes = io_tlb_nslabs << IO_TLB_SHIFT;

	while ((SLABS_PER_PAGE << order) > IO_TLB_MIN_SLABS) {
		vstart = (void *)__get_free_pages(GFP_DMA | __GFP_NOWARN,
						  order);
		if (vstart)
			break;
		order--;
	}

	if (!vstart) {
		io_tlb_nslabs = req_nslabs;
		return -ENOMEM;
	}
	if (order != get_order(bytes)) {
		pr_warn("only able to allocate %ld MB\n",
			(PAGE_SIZE << order) >> 20);
		io_tlb_nslabs = SLABS_PER_PAGE << order;
	}
	rc = swiotlb_late_init_with_tbl(vstart, io_tlb_nslabs);
	if (rc)
		free_pages((unsigned long)vstart, order);

	return rc;
}

static void swiotlb_cleanup(void)
{
	io_tlb_end = 0;
	io_tlb_start = 0;
	io_tlb_nslabs = 0;
	max_segment = 0;
}

int
swiotlb_late_init_with_tbl(char *tlb, unsigned long nslabs)
{
	unsigned long i, bytes;

	bytes = nslabs << IO_TLB_SHIFT;

	io_tlb_nslabs = nslabs;
	io_tlb_start = virt_to_phys(tlb);
	io_tlb_end = io_tlb_start + bytes;

	set_memory_decrypted((unsigned long)tlb, bytes >> PAGE_SHIFT);
	memset(tlb, 0, bytes);

	/*
	 * Allocate and initialize the free list array.  This array is used
	 * to find contiguous free memory regions of size up to IO_TLB_SEGSIZE
	 * between io_tlb_start and io_tlb_end.
	 */
	io_tlb_list = (unsigned int *)__get_free_pages(GFP_KERNEL,
	                              get_order(io_tlb_nslabs * sizeof(int)));
	if (!io_tlb_list)
		goto cleanup3;

	io_tlb_orig_addr = (phys_addr_t *)
		__get_free_pages(GFP_KERNEL,
				 get_order(io_tlb_nslabs *
					   sizeof(phys_addr_t)));
	if (!io_tlb_orig_addr)
		goto cleanup4;

	for (i = 0; i < io_tlb_nslabs; i++) {
		io_tlb_list[i] = IO_TLB_SEGSIZE - io_tlb_offset(i);
		io_tlb_orig_addr[i] = INVALID_PHYS_ADDR;
	}
	io_tlb_index = 0;
	no_iotlb_memory = false;

	swiotlb_print_info();

	late_alloc = 1;

	swiotlb_set_max_segment(io_tlb_nslabs << IO_TLB_SHIFT);

	return 0;

cleanup4:
	free_pages((unsigned long)io_tlb_list, get_order(io_tlb_nslabs *
	                                                 sizeof(int)));
	io_tlb_list = NULL;
cleanup3:
	swiotlb_cleanup();
	return -ENOMEM;
}

void __init swiotlb_exit(void)
{
	if (!io_tlb_orig_addr)
		return;

	if (late_alloc) {
		free_pages((unsigned long)io_tlb_orig_addr,
			   get_order(io_tlb_nslabs * sizeof(phys_addr_t)));
		free_pages((unsigned long)io_tlb_list, get_order(io_tlb_nslabs *
								 sizeof(int)));
		free_pages((unsigned long)phys_to_virt(io_tlb_start),
			   get_order(io_tlb_nslabs << IO_TLB_SHIFT));
	} else {
		memblock_free_late(__pa(io_tlb_orig_addr),
				   PAGE_ALIGN(io_tlb_nslabs * sizeof(phys_addr_t)));
		memblock_free_late(__pa(io_tlb_list),
				   PAGE_ALIGN(io_tlb_nslabs * sizeof(int)));
		memblock_free_late(io_tlb_start,
				   PAGE_ALIGN(io_tlb_nslabs << IO_TLB_SHIFT));
	}
	swiotlb_cleanup();
}

/*
 * Bounce: copy the swiotlb buffer from or back to the original dma location
 */
static void swiotlb_bounce(phys_addr_t orig_addr, phys_addr_t tlb_addr,
			   size_t size, enum dma_data_direction dir)
{
	unsigned long pfn = PFN_DOWN(orig_addr);
	unsigned char *vaddr = phys_to_virt(tlb_addr);

	if (PageHighMem(pfn_to_page(pfn))) {
		/* The buffer does not have a mapping.  Map it in and copy */
		unsigned int offset = orig_addr & ~PAGE_MASK;
		char *buffer;
		unsigned int sz = 0;
		unsigned long flags;

		while (size) {
			sz = min_t(size_t, PAGE_SIZE - offset, size);

			local_irq_save(flags);
			buffer = kmap_atomic(pfn_to_page(pfn));
			if (dir == DMA_TO_DEVICE)
				memcpy(vaddr, buffer + offset, sz);
			else
				memcpy(buffer + offset, vaddr, sz);
			kunmap_atomic(buffer);
			local_irq_restore(flags);

			size -= sz;
			pfn++;
			vaddr += sz;
			offset = 0;
		}
	} else if (dir == DMA_TO_DEVICE) {
		memcpy(vaddr, phys_to_virt(orig_addr), size);
	} else {
		memcpy(phys_to_virt(orig_addr), vaddr, size);
	}
}

static inline phys_addr_t slot_addr(phys_addr_t start, phys_addr_t idx)
{
	return start + (idx << IO_TLB_SHIFT);
}

/*
 * Return the offset into a iotlb slot required to keep the device happy.
 */
static unsigned int swiotlb_align_offset(struct device *dev, u64 addr)
{
	return addr & dma_get_min_align_mask(dev) & (IO_TLB_SIZE - 1);
}

/*
 * Carefully handle integer overflow which can occur when boundary_mask == ~0UL.
 */
static inline unsigned long get_max_slots(unsigned long boundary_mask)
{
	if (boundary_mask == ~0UL)
		return 1UL << (BITS_PER_LONG - IO_TLB_SHIFT);
	return nr_slots(boundary_mask + 1);
}

static unsigned int wrap_index(unsigned int index)
{
	if (index >= io_tlb_nslabs)
		return 0;
	return index;
}

/*
 * Find a suitable number of IO TLB entries size that will fit this request and
 * allocate a buffer from that IO TLB pool.
 */
static int find_slots(struct device *dev, phys_addr_t orig_addr,
		size_t alloc_size)
{
	unsigned long boundary_mask = dma_get_seg_boundary(dev);
	dma_addr_t tbl_dma_addr =
		phys_to_dma_unencrypted(dev, io_tlb_start) & boundary_mask;
	unsigned long max_slots = get_max_slots(boundary_mask);
	unsigned int iotlb_align_mask =
		dma_get_min_align_mask(dev) & ~(IO_TLB_SIZE - 1);
	unsigned int nslots = nr_slots(alloc_size), stride;
	unsigned int index, wrap, count = 0, i;
	unsigned long flags;

	BUG_ON(!nslots);

	/*
	 * For mappings with an alignment requirement don't bother looping to
	 * unaligned slots once we found an aligned one.  For allocations of
	 * PAGE_SIZE or larger only look for page aligned allocations.
	 */
	stride = (iotlb_align_mask >> IO_TLB_SHIFT) + 1;
	if (alloc_size >= PAGE_SIZE)
		stride = max(stride, stride << (PAGE_SHIFT - IO_TLB_SHIFT));

	spin_lock_irqsave(&io_tlb_lock, flags);
	if (unlikely(nslots > io_tlb_nslabs - io_tlb_used))
		goto not_found;

	index = wrap = wrap_index(ALIGN(io_tlb_index, stride));
	do {
		if ((slot_addr(tbl_dma_addr, index) & iotlb_align_mask) !=
		    (orig_addr & iotlb_align_mask)) {
			index = wrap_index(index + 1);
			continue;
		}

		/*
		 * If we find a slot that indicates we have 'nslots' number of
		 * contiguous buffers, we allocate the buffers from that slot
		 * and mark the entries as '0' indicating unavailable.
		 */
		if (!iommu_is_span_boundary(index, nslots,
					    nr_slots(tbl_dma_addr),
					    max_slots)) {
			if (io_tlb_list[index] >= nslots)
				goto found;
		}
		index = wrap_index(index + stride);
	} while (index != wrap);

not_found:
	spin_unlock_irqrestore(&io_tlb_lock, flags);
	return -1;

found:
	for (i = index; i < index + nslots; i++)
		io_tlb_list[i] = 0;
	for (i = index - 1;
	     io_tlb_offset(i) != IO_TLB_SEGSIZE - 1 &&
	     io_tlb_list[i]; i--)
		io_tlb_list[i] = ++count;

	/*
	 * Update the indices to avoid searching in the next round.
	 */
	if (index + nslots < io_tlb_nslabs)
		io_tlb_index = index + nslots;
	else
		io_tlb_index = 0;
	io_tlb_used += nslots;

	spin_unlock_irqrestore(&io_tlb_lock, flags);
	return index;
}

phys_addr_t swiotlb_tbl_map_single(struct device *dev, phys_addr_t orig_addr,
		size_t mapping_size, size_t alloc_size,
		enum dma_data_direction dir, unsigned long attrs)
{
	unsigned int offset = swiotlb_align_offset(dev, orig_addr);
	unsigned int i;
	int index;
	phys_addr_t tlb_addr;

	if (no_iotlb_memory)
		panic("Can not allocate SWIOTLB buffer earlier and can't now provide you with the DMA bounce buffer");

	if (mem_encrypt_active())
		pr_warn_once("Memory encryption is active and system is using DMA bounce buffers\n");

	if (mapping_size > alloc_size) {
		dev_warn_once(dev, "Invalid sizes (mapping: %zd bytes, alloc: %zd bytes)",
			      mapping_size, alloc_size);
		return (phys_addr_t)DMA_MAPPING_ERROR;
	}

	index = find_slots(dev, orig_addr, alloc_size + offset);
	if (index == -1) {
		if (!(attrs & DMA_ATTR_NO_WARN))
			dev_warn_ratelimited(dev,
	"swiotlb buffer is full (sz: %zd bytes), total %lu (slots), used %lu (slots)\n",
				 alloc_size, io_tlb_nslabs, io_tlb_used);
		return (phys_addr_t)DMA_MAPPING_ERROR;
	}

	/*
	 * Save away the mapping from the original address to the DMA address.
	 * This is needed when we sync the memory.  Then we sync the buffer if
	 * needed.
	 */
	for (i = 0; i < nr_slots(alloc_size + offset); i++)
		io_tlb_orig_addr[index + i] = slot_addr(orig_addr, i);

	tlb_addr = slot_addr(io_tlb_start, index) + offset;
	/*
	 * When dir == DMA_FROM_DEVICE we could omit the copy from the orig
	 * to the tlb buffer, if we knew for sure the device will
	 * overwirte the entire current content. But we don't. Thus
	 * unconditional bounce may prevent leaking swiotlb content (i.e.
	 * kernel memory) to user-space.
	 */
	swiotlb_bounce(orig_addr, tlb_addr, mapping_size, DMA_TO_DEVICE);
	return tlb_addr;
}

/*
 * tlb_addr is the physical address of the bounce buffer to unmap.
 */
void swiotlb_tbl_unmap_single(struct device *hwdev, phys_addr_t tlb_addr,
			      size_t mapping_size, size_t alloc_size,
			      enum dma_data_direction dir, unsigned long attrs)
{
	unsigned long flags;
	unsigned int offset = swiotlb_align_offset(hwdev, tlb_addr);
	int i, count, nslots = nr_slots(alloc_size + offset);
	int index = (tlb_addr - offset - io_tlb_start) >> IO_TLB_SHIFT;
	phys_addr_t orig_addr = io_tlb_orig_addr[index];

	/*
	 * First, sync the memory before unmapping the entry
	 */
	if (orig_addr != INVALID_PHYS_ADDR &&
	    !(attrs & DMA_ATTR_SKIP_CPU_SYNC) &&
	    ((dir == DMA_FROM_DEVICE) || (dir == DMA_BIDIRECTIONAL)))
		swiotlb_bounce(orig_addr, tlb_addr, mapping_size, DMA_FROM_DEVICE);

	/*
	 * Return the buffer to the free list by setting the corresponding
	 * entries to indicate the number of contiguous entries available.
	 * While returning the entries to the free list, we merge the entries
	 * with slots below and above the pool being returned.
	 */
	spin_lock_irqsave(&io_tlb_lock, flags);
	if (index + nslots < ALIGN(index + 1, IO_TLB_SEGSIZE))
		count = io_tlb_list[index + nslots];
	else
		count = 0;

	/*
	 * Step 1: return the slots to the free list, merging the slots with
	 * superceeding slots
	 */
	for (i = index + nslots - 1; i >= index; i--) {
		io_tlb_list[i] = ++count;
		io_tlb_orig_addr[i] = INVALID_PHYS_ADDR;
	}

	/*
	 * Step 2: merge the returned slots with the preceding slots, if
	 * available (non zero)
	 */
	for (i = index - 1;
	     io_tlb_offset(i) != IO_TLB_SEGSIZE - 1 && io_tlb_list[i];
	     i--)
		io_tlb_list[i] = ++count;
	io_tlb_used -= nslots;
	spin_unlock_irqrestore(&io_tlb_lock, flags);
}

void swiotlb_tbl_sync_single(struct device *hwdev, phys_addr_t tlb_addr,
			     size_t size, enum dma_data_direction dir,
			     enum dma_sync_target target)
{
	int index = (tlb_addr - io_tlb_start) >> IO_TLB_SHIFT;
	phys_addr_t orig_addr = io_tlb_orig_addr[index];

	if (orig_addr == INVALID_PHYS_ADDR)
		return;

	orig_addr += (tlb_addr & (IO_TLB_SIZE - 1)) -
		swiotlb_align_offset(hwdev, orig_addr);

	switch (target) {
	case SYNC_FOR_CPU:
		if (likely(dir == DMA_FROM_DEVICE || dir == DMA_BIDIRECTIONAL))
			swiotlb_bounce(orig_addr, tlb_addr,
				       size, DMA_FROM_DEVICE);
		else
			BUG_ON(dir != DMA_TO_DEVICE);
		break;
	case SYNC_FOR_DEVICE:
		if (likely(dir == DMA_TO_DEVICE || dir == DMA_BIDIRECTIONAL))
			swiotlb_bounce(orig_addr, tlb_addr,
				       size, DMA_TO_DEVICE);
		else
			BUG_ON(dir != DMA_FROM_DEVICE);
		break;
	default:
		BUG();
	}
}

/*
 * Create a swiotlb mapping for the buffer at @paddr, and in case of DMAing
 * to the device copy the data into it as well.
 */
dma_addr_t swiotlb_map(struct device *dev, phys_addr_t paddr, size_t size,
		enum dma_data_direction dir, unsigned long attrs)
{
	phys_addr_t swiotlb_addr;
	dma_addr_t dma_addr;

	trace_swiotlb_bounced(dev, phys_to_dma(dev, paddr), size,
			      swiotlb_force);

	swiotlb_addr = swiotlb_tbl_map_single(dev, paddr, size, size, dir,
			attrs);
	if (swiotlb_addr == (phys_addr_t)DMA_MAPPING_ERROR)
		return DMA_MAPPING_ERROR;

	/* Ensure that the address returned is DMA'ble */
	dma_addr = phys_to_dma_unencrypted(dev, swiotlb_addr);
	if (unlikely(!dma_capable(dev, dma_addr, size, true))) {
		swiotlb_tbl_unmap_single(dev, swiotlb_addr, size, size, dir,
			attrs | DMA_ATTR_SKIP_CPU_SYNC);
		dev_WARN_ONCE(dev, 1,
			"swiotlb addr %pad+%zu overflow (mask %llx, bus limit %llx).\n",
			&dma_addr, size, *dev->dma_mask, dev->bus_dma_limit);
		return DMA_MAPPING_ERROR;
	}

	if (!dev_is_dma_coherent(dev) && !(attrs & DMA_ATTR_SKIP_CPU_SYNC))
		arch_sync_dma_for_device(swiotlb_addr, size, dir);
	return dma_addr;
}

size_t swiotlb_max_mapping_size(struct device *dev)
{
	int min_align_mask = dma_get_min_align_mask(dev);
	int min_align = 0;

	/*
	 * swiotlb_find_slots() skips slots according to
	 * min align mask. This affects max mapping size.
	 * Take it into acount here.
	 */
	if (min_align_mask)
		min_align = roundup(min_align_mask, IO_TLB_SIZE);

	return ((size_t)IO_TLB_SIZE) * IO_TLB_SEGSIZE - min_align;
}

bool is_swiotlb_active(void)
{
	/*
	 * When SWIOTLB is initialized, even if io_tlb_start points to physical
	 * address zero, io_tlb_end surely doesn't.
	 */
	return io_tlb_end != 0;
}

#ifdef CONFIG_DEBUG_FS

static int __init swiotlb_create_debugfs(void)
{
	struct dentry *root;

	root = debugfs_create_dir("swiotlb", NULL);
	debugfs_create_ulong("io_tlb_nslabs", 0400, root, &io_tlb_nslabs);
	debugfs_create_ulong("io_tlb_used", 0400, root, &io_tlb_used);
	return 0;
}

late_initcall(swiotlb_create_debugfs);

#endif
