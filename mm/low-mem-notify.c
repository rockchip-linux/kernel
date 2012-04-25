/*
 * mm/low-mem-notify.c
 *
 * Sends low-memory notifications to processes via /dev/low-mem.
 *
 * Copyright (C) 2012 The Chromium OS Authors
 * This program is free software, released under the GPL.
 * Based on a proposal by Minchan Kim
 *
 * A process that polls /dev/low-mem is notified of a low-memory situation.
 * The intent is to allow the process to free some memory before the OOM killer
 * is invoked.
 *
 * A low-memory condition is estimated by subtracting anonymous memory
 * (i.e. process data segments), kernel memory, and a fixed amount of
 * file-backed memory from total memory.  This is just a heuristic, as in
 * general we don't know how much memory can be reclaimed before we try to
 * reclaim it, and that's too expensive or too late.
 *
 * This is tailored to Chromium OS, where a single program (the browser)
 * controls most of the memory, and (currently) no swap space is used.
 */


#include <linux/low-mem-notify.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/mm.h>

static DECLARE_WAIT_QUEUE_HEAD(low_mem_wait);
static atomic_t low_mem_state = ATOMIC_INIT(0);
unsigned low_mem_margin_mb = 50;
bool low_mem_margin_enabled = true;
unsigned long low_mem_minfree;
/*
 * We're interested in worst-case anon memory usage when the low-memory
 * notification fires.  To contain logging, we limit our interest to
 * non-trivial steps.
 */
unsigned long low_mem_lowest_seen_anon_mem;
const unsigned long low_mem_anon_mem_delta = 10 * 1024 * 1024 / PAGE_SIZE;

struct low_mem_notify_file_info {
	unsigned long unused;
};

void low_mem_notify(void)
{
	atomic_set(&low_mem_state, true);
	wake_up(&low_mem_wait);
}

static int low_mem_notify_open(struct inode *inode, struct file *file)
{
	struct low_mem_notify_file_info *info;
	int err = 0;

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto out;
	}

	file->private_data = info;
out:
	return err;
}

static int low_mem_notify_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	return 0;
}

static unsigned int low_mem_notify_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;

	/* Update state to reflect any recent freeing. */
	atomic_set(&low_mem_state, is_low_mem_situation());

	poll_wait(file, &low_mem_wait, wait);

	if (low_mem_margin_enabled && atomic_read(&low_mem_state) != 0)
		ret = POLLIN;

	return ret;
}

const struct file_operations low_mem_notify_fops = {
	.open = low_mem_notify_open,
	.release = low_mem_notify_release,
	.poll = low_mem_notify_poll,
};
EXPORT_SYMBOL(low_mem_notify_fops);

#ifdef CONFIG_SYSFS

#define LOW_MEM_ATTR(_name)				      \
	static struct kobj_attribute low_mem_##_name##_attr = \
		__ATTR(_name, 0644, low_mem_##_name##_show,   \
		       low_mem_##_name##_store)

static ssize_t low_mem_margin_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	if (low_mem_margin_enabled)
		return sprintf(buf, "%u\n", low_mem_margin_mb);
	else
		return sprintf(buf, "off\n");
}

static unsigned low_mem_margin_to_minfree(unsigned margin_mb)
{
	return margin_mb * (1024 * 1024 / PAGE_SIZE);
}

static ssize_t low_mem_margin_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	int err;
	unsigned long margin;
	/*
	 * Even though the API does not say anything about this, the string in
	 * buf is zero-terminated (as long as count < PAGE_SIZE) because buf is
	 * a newly allocated zero-filled page.  Most other sysfs handlers rely
	 * on this too.
	 */
	if (strncmp("off", buf, 3) == 0) {
		printk(KERN_INFO "low_mem: disabling notifier\n");
		low_mem_margin_enabled = false;
		return count;
	}
	if (strncmp("on", buf, 2) == 0) {
		printk(KERN_INFO "low_mem: enabling notifier\n");
		low_mem_margin_enabled = true;
		return count;
	}

	err = strict_strtoul(buf, 10, &margin);
	if (err)
		return -EINVAL;
	if (margin * ((1024 * 1024) / PAGE_SIZE) > totalram_pages)
		return -EINVAL;
	/* Notify when the "free" memory is below margin megabytes. */
	low_mem_margin_enabled = true;
	low_mem_margin_mb = (unsigned int) margin;
	/* Convert to pages outside the allocator fast path. */
	low_mem_minfree = low_mem_margin_to_minfree(low_mem_margin_mb);
	printk(KERN_INFO "low_mem: setting minfree to %lu kB\n",
	       low_mem_minfree * (PAGE_SIZE / 1024));
	return count;
}
LOW_MEM_ATTR(margin);

static struct attribute *low_mem_attrs[] = {
	&low_mem_margin_attr.attr,
	NULL,
};

static struct attribute_group low_mem_attr_group = {
	.attrs = low_mem_attrs,
	.name = "chromeos-low_mem",
};

static int __init low_mem_init(void)
{
	int err = sysfs_create_group(mm_kobj, &low_mem_attr_group);
	if (err)
		printk(KERN_ERR "low_mem: register sysfs failed\n");
	low_mem_minfree = low_mem_margin_to_minfree(low_mem_margin_mb);
	low_mem_lowest_seen_anon_mem = totalram_pages;
	return err;
}
module_init(low_mem_init)

#endif
