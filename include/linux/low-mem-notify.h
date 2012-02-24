#ifndef _LINUX_LOW_MEM_NOTIFY_H
#define _LINUX_LOW_MEM_NOTIFY_H

#include <linux/stddef.h>
#include <linux/mm.h>

extern unsigned low_mem_margin_percent;
extern unsigned long low_mem_minfree;
void low_mem_notify(void);
extern const struct file_operations low_mem_notify_fops;

/*
 * Return TRUE if we are in a low memory state.
 */
static inline bool is_low_mem_situation(void)
{
	const int lru_base = NR_LRU_BASE - LRU_BASE;
	/*
	 * We declare a low-memory condition when free memory plus easily
	 * reclaimable memory is low.
	 */
	unsigned long free_mem = global_page_state(NR_FREE_PAGES);
	unsigned long file_mem =
			global_page_state(lru_base + LRU_ACTIVE_FILE) +
			global_page_state(lru_base + LRU_INACTIVE_FILE);
	unsigned long dirty_mem = global_page_state(NR_FILE_DIRTY);
	unsigned long min_file_mem =
			min_filelist_kbytes >> (PAGE_SHIFT - 10);
	/*
	 * free_mem is completely unallocated; clean file-backed memory
	 * (file_mem - dirty_mem) is easy to reclaim, except for the last
	 * min_filelist_kbytes.
	 */
	unsigned long available_mem =
			free_mem - (file_mem - dirty_mem - min_file_mem);
	bool is_low_mem = available_mem < low_mem_minfree;

#ifdef CONFIG_LOW_MEM_NOTIFY_DEBUG
	{
		static bool was_low_mem;
		if (is_low_mem && !was_low_mem)
			printk(KERN_INFO "entering low_mem: avail=%lu MB\n",
			       available_mem >> (20 - PAGE_SHIFT));
		else if (!is_low_mem && was_low_mem)
			printk(KERN_INFO "exiting low_mem: avail=%lu MB\n",
			       available_mem >> (20 - PAGE_SHIFT));
		was_low_mem = is_low_mem;
	}
#endif

	return is_low_mem;
}

#endif
