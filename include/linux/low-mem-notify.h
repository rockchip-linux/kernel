#ifndef _LINUX_LOW_MEM_NOTIFY_H
#define _LINUX_LOW_MEM_NOTIFY_H

#include <linux/stddef.h>
#include <linux/mm.h>

extern unsigned low_mem_margin_percent;
extern unsigned long low_mem_minfree;
void low_mem_notify(void);
extern const struct file_operations low_mem_notify_fops;
extern bool low_mem_margin_enabled;
extern unsigned long low_mem_lowest_seen_anon_mem;
extern const unsigned long low_mem_anon_mem_delta;

/*
 * Return TRUE if we are in a low memory state.
 */
static inline bool _is_low_mem_situation(void)
{
	const int lru_base = NR_LRU_BASE - LRU_BASE;
	static bool was_low_mem;	/* style guide prohibits "= false" */
	/*
	 * We declare a low-memory condition when free memory plus easily
	 * reclaimable memory is low.
	 *
	 * free_mem is completely unallocated; clean file-backed memory
	 * (file_mem - dirty_mem) is easy to reclaim, except for the last
	 * min_filelist_kbytes.
	 */
	unsigned long free_mem = global_page_state(NR_FREE_PAGES);
	unsigned long file_mem =
			global_page_state(lru_base + LRU_ACTIVE_FILE) +
			global_page_state(lru_base + LRU_INACTIVE_FILE);
	unsigned long dirty_mem = global_page_state(NR_FILE_DIRTY);
	unsigned long min_file_mem = min_filelist_kbytes >> (PAGE_SHIFT - 10);
	unsigned long available_file_mem = file_mem - dirty_mem - min_file_mem;
	unsigned long available_mem = free_mem + available_file_mem;
	bool is_low_mem = available_mem < low_mem_minfree;

	if (unlikely(is_low_mem && !was_low_mem)) {
		unsigned long anon_mem =
			global_page_state(lru_base + LRU_ACTIVE_ANON) +
			global_page_state(lru_base + LRU_INACTIVE_ANON);
		if (anon_mem < low_mem_lowest_seen_anon_mem) {
			printk(KERN_INFO "entering low_mem (avail = %lu kB) "
			       "with lowest seen anon mem: %lu kB\n",
			       available_mem * PAGE_SIZE / 1024,
			       anon_mem * PAGE_SIZE / 1024);
			low_mem_lowest_seen_anon_mem = anon_mem -
				low_mem_anon_mem_delta;
		}
	}
	was_low_mem = is_low_mem;

	return is_low_mem;
}

static inline bool is_low_mem_situation(void)
{
	return low_mem_margin_enabled ? _is_low_mem_situation() : false;
}

#endif
