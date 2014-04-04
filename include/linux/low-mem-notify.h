#ifndef _LINUX_LOW_MEM_NOTIFY_H
#define _LINUX_LOW_MEM_NOTIFY_H

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
	const int file_mem_multiplier = 2;  /* between 1 and 2 seems right */
	/*
	 * We declare a low-memory condition when free memory is low and there
	 * isn't much reclaimable file memory.
	 */
	unsigned long free_mem = global_page_state(NR_FREE_PAGES);
	unsigned long file_mem =
			global_page_state(lru_base + LRU_ACTIVE_FILE) +
			global_page_state(lru_base + LRU_INACTIVE_FILE);
	unsigned long min_file_mem = file_mem_multiplier *
			(min_filelist_kbytes >> (PAGE_SHIFT - 10));
	bool is_low_mem = free_mem < low_mem_minfree && file_mem < min_file_mem;

#ifdef CONFIG_LOW_MEM_NOTIFY_DEBUG
	{
		static bool was_low_mem;
		if (is_low_mem && !was_low_mem)
			printk(KERN_INFO "entering low_mem: %lu\n", in_use);
		else if (!is_low_mem && was_low_mem)
			printk(KERN_INFO "exiting low_mem: %lu\n", in_use);
		was_low_mem = is_low_mem;
	}
#endif

	return is_low_mem;
}

#endif
