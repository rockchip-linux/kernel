/*
 * Copyright 2012 Google, Inc.
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
/*
 * The boot cache device mapper reads a set of contiguously stored sectors.
 * These sectors are copies of the sectors read during an earlier boot. Only
 * small reads (less than some number of sectors) are selected for the cache,
 * since this results in the highest benefit.
 *
 * The data for the boot cache consists of three sections:
 * a header, the sector trace and the cache sectors.
 * These are stored after the file system in the same partition.
 *
 * The boot cache is created by separate user process that reads a
 * sector trace created if the boot cache is invalid.
 */
#include <linux/async.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/device-mapper.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include "dm.h"

#include "dm-bootcache.h"

#define DM_MSG_PREFIX "bootcache"

#define DEFAULT_MAX_PAGES	50000
#define DEFAULT_SIZE_LIMIT	128
#define DEFAULT_MAX_TRACE	(1 << 13)
#define MAX_TRACE		(1 << 20)
#define DEV_MODE		FMODE_READ
#define SECTOR_SIZE		(1 << SECTOR_SHIFT)
#define SECTORS_PER_PAGE	(PAGE_SIZE / SECTOR_SIZE)
#define MAX_DEVICE_NAME		(1 << 8)
#define FRACTION_OF_TOTAL_PAGES	10


enum bc_state {
	BC_INIT = 1,
	BC_TRACING,
	BC_FILLING,
	BC_FILLED,
	BC_BYPASS
};

struct bootcache_waiter {
	struct completion completion;
	int error;
};

struct bootcache_args {
	/* Device being cached. The boot cache also stores its cache here. */
	char device[MAX_DEVICE_NAME];

	/* Identifies the data on the device. eg root hex digest from verity */
	char signature[MAX_SIGNATURE];

	/* Sector start of cache on device */
	u64 cache_start;

	/* Max num of pages to cache */
	u64 max_pages;

	/* Reads this size or larger will not be cached */
	u64 size_limit;

	/* Maximum number of trace records to collect */
	u64 max_trace;
};

struct bootcache_stats {
	unsigned num_requests;     /* Read requests */
	unsigned num_hits;         /* Number of hits */
	unsigned overlapped;       /* Blocks used while reading rest */
};

struct bootcache_page {
	struct bootcache_page *next;
	struct page *page;
	u64 sector;	/* first sector in set of sectors in this page */
	bool is_filled;
};

struct bootcache_sector_map {
	u32	num_buckets;	/* Number of buckets for hash */
	u32	num_pages;	/* Number of pages of sectors */
	struct bootcache_page *pages;	/* Cache of pages of sectors */
	struct bootcache_page *nextpage;/* Next page entry to add */
	struct bootcache_page **bucket;	/* Hash buckets */
};

struct bootcache {
	const char	*name;		/* Taken from device being cached */
	struct bootcache_stats	stats;
	struct bootcache_args	args;
	sector_t	begin;	/* Beginning sector of underlying device */
	sector_t	len;	/* Length in sectors of underlying device */
	atomic_t	state;	/* Cache state - needs atomic read */
	spinlock_t	trace_lock;	/* Spin lock for trace table */
	struct bootcache_trace	*trace;	/* Trace of blocks read during boot */
	u32		trace_next;	/* Next element to fill for tracing */
	u32		max_io;		/* Max pages we can read/write */
	bool		is_valid;	/* The cache is valid */
	bool		is_free;	/* The cache data has been freed */
	struct kref	kref;		/* Protects in-flight operations */
	struct dm_target	*ti;	/* Device in device mapper */
	struct bio_set	*bio_set;	/* Set of bios for reading blocks */
	struct dm_dev	*dev;		/* Device for both cache and data */
	struct delayed_work	work;	/* Work that needs a thread */
	struct mutex	cache_lock;	/* Locks everything in cache struct */
	struct completion	init_complete;	/* Wait for initialization */
	struct bootcache_sector_map sectors;	/* Table of pages of sectors */
	/* Sysfs files for managing the block cache */
	struct bin_attribute valid;	/* 1 -> valid 0 -> build cache */
	struct bin_attribute free;	/* Write '1' to free cache */
	struct bin_attribute header;	/* Content for bootcache header */
	struct bin_attribute blocktrace;/* Trace of blocks accessed */
	/* Computed hdr to be compared with on disk header. */
	struct bootcache_hdr hdr;
};

static inline u64 bytes_to_pages(u64 bytes)
{
	return (bytes + PAGE_SIZE - 1) >> PAGE_SHIFT;
}

static inline u64 sectors_to_pages(u64 sectors)
{
	return sectors >> (PAGE_SHIFT - SECTOR_SHIFT);
}

static inline u64 pages_to_sectors(u64 pages)
{
	return pages << (PAGE_SHIFT - SECTOR_SHIFT);
}

static inline struct bootcache_page **bootcache_hash(
					struct bootcache_sector_map *map,
					u64 sector)
{
	return &map->bucket[(u32)sector % map->num_buckets];
}

static struct bootcache_page *bootcache_get_chunk(
					struct bootcache_sector_map *map,
					u64 sector)
{
	struct bootcache_page *next;

	next = *bootcache_hash(map, sector);
	while (next) {
		if (sector == next->sector) {
			if (next->is_filled)
				return next;
			else
				return NULL;
		}
		next = next->next;
	}
	return next;
}

struct bootcache_page *bootcache_new_chunk(struct bootcache_sector_map *map,
					u64 sector)
{
	struct bootcache_page **bucket = bootcache_hash(map, sector);
	struct bootcache_page *p;

	if (map->nextpage == &map->pages[map->num_pages]) {
		DMWARN("block cache full");
		return NULL;
	}
	p = map->nextpage++;
	p->page = alloc_page(GFP_KERNEL);
	p->sector = sector;
	p->next = *bucket;
	*bucket = p;
	return p;
}

static int build_sector_map(struct bootcache_sector_map *map, u32 num_pages)
{
	map->num_pages = num_pages;
	map->num_buckets = num_pages * 3 / 2;
	map->bucket = kzalloc(map->num_buckets * sizeof(*map->bucket),
				GFP_KERNEL);
	if (!map->bucket) {
		DMERR("build_sector_maps kzalloc buckets");
		return -ENOMEM;
	}
	map->pages = kzalloc(num_pages * sizeof(*map->pages), GFP_KERNEL);
	if (!map->pages) {
		kfree(map->bucket);
		DMERR("build_sector_maps kzalloc pages");
		return -ENOMEM;
	}
	map->nextpage = map->pages;
	return 0;
}

static void bootcache_free_sector_map(struct bootcache_sector_map *map)
{
	struct bootcache_page *p;

	for (p = map->pages; p < map->nextpage; p++)
		if (p->page)
			__free_pages(p->page, 0);
	kfree(map->pages);
	kfree(map->bucket);
	map->pages = NULL;
	map->bucket = NULL;
	map->nextpage = 0;
}

static int bootcache_create_bin_file(struct bootcache *cache,
	struct bin_attribute *attr, char *name, ssize_t size,
	ssize_t (*read)(struct file *, struct kobject *,
			struct bin_attribute *, char *, loff_t, size_t),
	ssize_t (*write)(struct file *, struct kobject *,
			struct bin_attribute *, char *, loff_t, size_t))
{
	int rc = 0;

	if (attr->attr.name)
		return -EEXIST;
	attr->attr.name = name;
	attr->attr.mode = write ? 0644 : 0444;
	attr->size = size;
	attr->read = read;
	attr->write = write;

	rc = sysfs_create_bin_file(dm_kobject(dm_table_get_md(
					cache->ti->table)), attr);
	if (rc)
		DMERR("sysfs_create_bin_file %s: %d", name, rc);
	return rc;
}

/*
 * bootcache_remove_bin_file uses the file name as flag
 * to determine if the sysfs file has been created.
 */
static void bootcache_remove_bin_file(struct bootcache *cache,
	struct bin_attribute *attr)
{
	if (attr->attr.name) {
		sysfs_remove_bin_file(dm_kobject(dm_table_get_md(
					cache->ti->table)), attr);
		attr->attr.name = NULL;
	}
}

/*
 * bootcache_remove_all_files removes all the sysfs files
 * that have been created and only the ones that have been
 * craeted.
 */
static void bootcache_remove_all_files(struct bootcache *cache)
{
	bootcache_remove_bin_file(cache,  &cache->blocktrace);
	bootcache_remove_bin_file(cache,  &cache->header);
	bootcache_remove_bin_file(cache,  &cache->free);
	bootcache_remove_bin_file(cache,  &cache->valid);
}

static void bootcache_free_resources(struct kref *kref)
{
	struct bootcache *cache = container_of(kref, struct bootcache,
						kref);
	/* Will hang if we try to remove cache->free here */
	bootcache_remove_bin_file(cache,  &cache->blocktrace);
	bootcache_remove_bin_file(cache,  &cache->header);
	bootcache_remove_bin_file(cache,  &cache->valid);
	bootcache_free_sector_map(&cache->sectors);
	kfree(cache->trace);
	cache->trace  = NULL;
}

/*
 * bootcache_get_ino returns the inode number of the bio if it has one.
 * If not, it returns 0, an illegal inode number.
 * When the bio is sent down for I/O, these fields don't change
 * while the I/O is pending.
 */
static unsigned long bootcache_get_ino(struct bio *bio)
{
	if (!bio)
		return 0;
	if (!bio->bi_io_vec)
		return 0;
	if (!bio->bi_io_vec->bv_page)
		return 0;
	if (!bio->bi_io_vec->bv_page->mapping)
		return 0;
	if (!bio->bi_io_vec->bv_page->mapping->host)
		return 0;
	return bio->bi_io_vec->bv_page->mapping->host->i_ino;
}

static void bootcache_record(struct bootcache *cache, struct bio *bio)
{
	u64 sector = bio->bi_iter.bi_sector;
	u64 count = to_sector(bio->bi_iter.bi_size);
	struct bootcache_trace *tr;

	if (!cache->trace)
		return;
	spin_lock(&cache->trace_lock);
	if (cache->trace_next < cache->args.max_trace) {
		tr = &cache->trace[cache->trace_next];
		tr->sector = sector;
		tr->count = count;
		tr->ino = bootcache_get_ino(bio);
		++cache->trace_next;
	}
	spin_unlock(&cache->trace_lock);
}

static bool is_in_cache(struct bootcache *cache, struct bio *bio)
{
	u64 sector = bio->bi_iter.bi_sector;
	u32 count = bytes_to_pages(bio->bi_iter.bi_size);
	u32 i;

	for (i = 0; i < count; i++, sector += SECTORS_PER_PAGE) {
		if (!bootcache_get_chunk(&cache->sectors, sector))
			return 0;
	}
	++cache->stats.num_hits;
	return 1;
}

static void bootcache_read_from_cache(struct bootcache *cache, struct bio *bio)
{
	struct bootcache_page *bp;
	u64 sector = bio->bi_iter.bi_sector;
	u32 count = bytes_to_pages(bio->bi_iter.bi_size);
	u8 *dst;
	u8 *src;
	u32 i;

	for (i = 0; i < count; i++, sector += SECTORS_PER_PAGE) {
		bp = bootcache_get_chunk(&cache->sectors, sector);
		if (!bp) {
			/*
			 * Should have found it because we just
			 * looked for it before calling this code
			 */
			DMCRIT("Didn't find block %llx", sector);
			BUG();
		}
		dst = kmap_atomic(bio->bi_io_vec[i].bv_page);
		src = kmap_atomic(bp->page);
		memcpy(dst, src, PAGE_SIZE);
		kunmap_atomic(src);
		kunmap_atomic(dst);
	}
	set_bit(BIO_UPTODATE, &bio->bi_flags);
	bio->bi_end_io(bio, 0);
}

static void bootcache_read(struct bootcache *cache, struct bio *bio)
{
	int state;

	bio->bi_bdev = cache->dev->bdev;
	/* Only record reads below the given size */
	if ((atomic_read(&cache->state) == BC_BYPASS) ||
		(bio_sectors(bio) > cache->args.size_limit)) {
		generic_make_request(bio);
		return;
	}
	kref_get(&cache->kref);
try_again:
	state = atomic_read(&cache->state);
	switch (state) {
	case BC_INIT:
		wait_for_completion(&cache->init_complete);
		goto try_again;
	case BC_TRACING:
		bootcache_record(cache, bio);
		generic_make_request(bio);
		break;
	case BC_FILLING:
		++cache->stats.overlapped;
		/* FALLTHRU */
	case BC_FILLED:
		if (is_in_cache(cache, bio))
			bootcache_read_from_cache(cache, bio);
		else
			generic_make_request(bio);
		break;
	case BC_BYPASS:
		generic_make_request(bio);
		break;
	default:
		DMCRIT("unknown state %d", state);
		BUG();
		break;
	}
	++cache->stats.num_requests;
	if (cache->stats.num_requests % 1000 == 0) {
		DMINFO("hits = %u / %u",
			cache->stats.num_hits,
			cache->stats.num_requests);
	}
	kref_put(&cache->kref, bootcache_free_resources);
}

static ssize_t valid_read(struct file *file, struct kobject *kobp,
				struct bin_attribute *bin_attr, char *buf,
				loff_t pos, size_t count)
{
	struct bootcache *cache = container_of(bin_attr, struct bootcache,
						valid);

	if (pos > 0 || count == 0)
		return 0;
	buf[0] = cache->is_valid ? '1' : '0';
	return 1;
}

static ssize_t free_read(struct file *file, struct kobject *kobp,
				struct bin_attribute *bin_attr, char *buf,
				loff_t pos, size_t count)
{
	struct bootcache *cache = container_of(bin_attr, struct bootcache,
						free);

	if (pos > 0 || count == 0)
		return 0;
	buf[0] = cache->is_free ? '1' : '0';
	return 1;
}

static ssize_t free_write(struct file *file, struct kobject *kobp,
				struct bin_attribute *bin_attr, char *buf,
				loff_t pos, size_t count)
{
	struct bootcache *cache = container_of(bin_attr, struct bootcache,
						free);
	ssize_t err = 0;

	mutex_lock(&cache->cache_lock);
	if (cache->is_free) {
		err = 0;
		goto exit;
	}
	atomic_set(&cache->state, BC_BYPASS);
	/*
	 * Once BC_BYPASS is set, the system
	 * should drain quickly.
	 */
	kref_put(&cache->kref, bootcache_free_resources);
	cache->is_free = 1;
	/* Tell caller we wrote everything */
	err = count;
exit:
	mutex_unlock(&cache->cache_lock);
	return err;
}

static ssize_t header_read(struct file *file, struct kobject *kobp,
				struct bin_attribute *bin_attr, char *buf,
				loff_t pos, size_t count)
{
	struct bootcache *cache = container_of(bin_attr, struct bootcache,
						header);

	return memory_read_from_buffer(buf, count, &pos, &cache->hdr,
					sizeof(cache->hdr));
}

static ssize_t blocktrace_read(struct file *file, struct kobject *kobp,
				struct bin_attribute *bin_attr, char *buf,
				loff_t pos, size_t count)
{
	struct bootcache *cache = container_of(bin_attr, struct bootcache,
						blocktrace);
	char *data;
	size_t next, size;
	ssize_t err = 0;

	kref_get(&cache->kref);
	if (atomic_read(&cache->state) != BC_TRACING) {
		err = -ENODEV;
		goto exit;
	}
	data = (char *)cache->trace;

	spin_lock(&cache->trace_lock);
	next = cache->trace_next;
	spin_unlock(&cache->trace_lock);

	size = next * sizeof(struct bootcache_trace);

	err = memory_read_from_buffer(buf, count, &pos, data, size);
exit:
	kref_put(&cache->kref, bootcache_free_resources);
	return err;
}

static int bootcache_init_sysfs(struct bootcache *cache, struct dm_target *ti)
{
	int rc;

	rc = bootcache_create_bin_file(cache, &cache->valid, "valid",
					3, valid_read, NULL);
	if (rc)
		goto error;
	rc = bootcache_create_bin_file(cache, &cache->free, "free",
					3, free_read, free_write);
	if (rc)
		goto error;
	rc = bootcache_create_bin_file(cache, &cache->header, "header",
					sizeof(cache->hdr), header_read, NULL);
	if (rc)
		goto error;
	rc = bootcache_create_bin_file(cache, &cache->blocktrace, "blocktrace",
			cache->args.max_trace * sizeof(struct bootcache_trace),
			blocktrace_read, NULL);
	if (rc)
		goto error;
	return rc;
error:
	bootcache_remove_all_files(cache);
	return rc;
}

static void bootcache_read_sectors_end(struct bio *bio, int error)
{
	struct bootcache_waiter	*waiter = bio->bi_private;

	if (unlikely(error)) {
		waiter->error = error;
		DMERR("Error occurred in bootcache_read_sectors:"
			" %d (%llx, %x)",
			error, (u64)bio->bi_iter.bi_sector,
			bio->bi_iter.bi_size);
	}
	complete(&waiter->completion);
}

static int bootcache_read_sectors(struct bootcache *cache)
{
	struct bootcache_waiter waiter;
	struct bio *bio;
	struct bootcache_page *p;
	struct bootcache_page *start_page;
	struct bio_vec *bvec;
	sector_t sector = cache->args.cache_start + cache->hdr.sectors_meta +
				SECTORS_PER_PAGE;
	u32 max_io = cache->max_io;
	u32 numpages = cache->sectors.num_pages;
	u32 chunks_to_read = (numpages + max_io - 1) / max_io;
	int i;
	int j;
	int rc = 0;

	p = cache->sectors.pages;
	for (i = 0; i < chunks_to_read; i++) {
		bio = bio_alloc_bioset(GFP_KERNEL, max_io, cache->bio_set);
		if (unlikely(!bio)) {
			DMERR("Out of memory bio_alloc_bioset");
			return -ENOMEM;
		}
		bio->bi_private = &waiter;
		bio->bi_iter.bi_idx = 0;
		bio->bi_iter.bi_bvec_done = 0;
		bio->bi_bdev = cache->dev->bdev;
		bio->bi_end_io = bootcache_read_sectors_end;
		bio->bi_rw = 0;
		bio->bi_iter.bi_sector = sector;
		bvec = bio->bi_io_vec;
		start_page = p;
		for (j = 0; j < max_io; j++, bvec++, p++) {
			if (p == cache->sectors.nextpage)
				break;
			bvec->bv_page = p->page;
			bvec->bv_offset = 0;
			bvec->bv_len = PAGE_SIZE;
		}
		bio->bi_iter.bi_size = j * PAGE_SIZE;
		bio->bi_vcnt = j;

		init_completion(&waiter.completion);
		waiter.error = 0;
		generic_make_request(bio);
		wait_for_completion(&waiter.completion);
		if (waiter.error) {
			rc = waiter.error;
			bio->bi_private = cache;
			bio_put(bio);
			break;
		}
		p = start_page;
		for (j = 0; j < max_io; j++, p++) {
			if (p == cache->sectors.nextpage)
				break;
			p->is_filled = 1;
		}
		sector += pages_to_sectors(j);
		bio->bi_private = cache;
		bio_put(bio);
	}
	atomic_set(&cache->state, BC_FILLED);
	return rc;
}

static void bootcache_dev_read_end(struct bio *bio, int error)
{
	struct bootcache_waiter *waiter = bio->bi_private;

	if (unlikely(error)) {
		waiter->error = error;
		DMERR("Error occurred in bootcache_dev_read: %d (%llx, %x)",
		      error, (u64)bio->bi_iter.bi_sector, bio->bi_iter.bi_size);
	}
	complete(&waiter->completion);
}

static int bootcache_dev_read(struct bootcache *cache, void *data,
				int len, u64 sector)
{
	struct bootcache_waiter waiter;
	struct bio *bio;
	struct bio_vec *bvec;
	int pages_to_read = (len + PAGE_SIZE - 1) >> PAGE_SHIFT;
	int max_io = cache->max_io;
	int bytes_to_copy;
	int i;
	int rc = 0;
	int pages_read;
	u8 *dst = data;
	u8 *src;

	pages_read = 0;
	while (len) {
		if (pages_to_read < max_io)
			max_io = pages_to_read;
		bio = bio_alloc_bioset(GFP_KERNEL, max_io, cache->bio_set);
		if (unlikely(!bio)) {
			DMERR("Out of memory bio_alloc_bioset");
			return -ENOMEM;
		}
		bvec = bio->bi_io_vec;
		for (i = 0; i < max_io; i++, bvec++)
			bvec->bv_page = alloc_page(GFP_KERNEL);
		bio->bi_private = &waiter;
		bio->bi_iter.bi_idx = 0;
		bio->bi_iter.bi_bvec_done = 0;
		bio->bi_bdev = cache->dev->bdev;
		bio->bi_end_io = bootcache_dev_read_end;
		bio->bi_rw = 0;
		bio->bi_iter.bi_sector = sector;
		bvec = bio->bi_io_vec;
		for (i = 0; i < max_io; i++, bvec++) {
			bvec->bv_offset = 0;
			bvec->bv_len = PAGE_SIZE;
		}
		pages_to_read -= max_io;
		bio->bi_iter.bi_size = max_io * PAGE_SIZE;
		bio->bi_vcnt = max_io;

		init_completion(&waiter.completion);
		waiter.error = 0;
		generic_make_request(bio);
		wait_for_completion(&waiter.completion);
		if (waiter.error) {
			rc = waiter.error;
			goto error;
		}
		for (i = 0; i < max_io; i++) {
			bytes_to_copy = min(len, (int)PAGE_SIZE);
			src = kmap_atomic(bio->bi_io_vec[i].bv_page);
			memcpy(dst, src, bytes_to_copy);
			kunmap_atomic(src);
			len -= bytes_to_copy;
			if (!len)
				break;
			dst += bytes_to_copy;
		}
		sector += pages_to_sectors(max_io);
		bvec = bio->bi_io_vec;
		for (i = 0; i < max_io; i++, bvec++)
			__free_pages(bvec->bv_page, 0);
		bio->bi_private = cache;
		bio_put(bio);
	}
	return rc;
error:
	bvec = bio->bi_io_vec;
	for (i = 0; i < max_io; i++, bvec++)
		__free_pages(bvec->bv_page, 0);
	bio->bi_private = cache;
	bio_put(bio);
	return rc;
}

static int is_valid_hdr(struct bootcache *cache, struct bootcache_hdr *hdr)
{
	u64 max_sectors;
	u64 max_meta_sectors;
	u64 max_pages;

	if (hdr->magic != BOOTCACHE_MAGIC)
		return 0;
	if (hdr->version != BOOTCACHE_VERSION)
		return 0;
	if (hdr->max_sectors != cache->hdr.max_sectors)
		return 0;
	if (hdr->max_hw_sectors != cache->hdr.max_hw_sectors)
		return 0;
	if (strncmp(hdr->timestamp, utsname()->version, sizeof(hdr->timestamp)))
		return 0;
	if (strncmp(hdr->signature, cache->hdr.signature,
			sizeof(hdr->signature)) != 0)
		return 0;
	/*
	 * Check sanity:
	 * Can't have any more meta sectors than it takes to map
	 * the remaining parition space for bootcache.
	 */
	max_sectors = to_sector(i_size_read(cache->dev->bdev->bd_inode))
			- cache->args.cache_start;
	max_pages = sectors_to_pages(max_sectors);
	max_pages = min(max_pages, (u64)INT_MAX / sizeof(*cache->trace));
	max_pages = min(max_pages,
			(u64)totalram_pages / FRACTION_OF_TOTAL_PAGES);
	if (hdr->num_trace_recs > max_pages) {
		DMERR("too many trace records %lld", (u64)hdr->num_trace_recs);
		return 0;
	}
	max_meta_sectors = to_sector(round_up(max_pages * sizeof(u64),
						SECTOR_SIZE));
	if (hdr->sectors_meta > max_meta_sectors) {
		DMERR("too many meta sectors %lld", (u64)hdr->sectors_meta);
		return 0;
	}
	if (hdr->sectors_data > max_sectors - hdr->sectors_meta - 1) {
		DMERR("bootcache too big %lld", (u64)hdr->sectors_data);
		return 0;
	}
	return 1;
}

static int read_trace(struct bootcache *cache)
{
	u64 size_trace;
	u64 i;
	u64 j;
	int rc;
	int sum = 0;

	size_trace = sizeof(*cache->trace) * cache->hdr.num_trace_recs;
	cache->trace = kzalloc(size_trace, GFP_KERNEL);
	if (!cache->trace) {
		DMERR("read_trace out of memory");
		return -ENOMEM;
	}
	rc = bootcache_dev_read(cache, cache->trace, size_trace,
			cache->hdr.sector + SECTORS_PER_PAGE);
	if (rc) {
		DMERR("bootcache_dev_read trace %d", rc);
		return rc;
	}
	for (i = 0; i < cache->hdr.num_trace_recs; i++) {
		struct bootcache_trace *tr;
		tr = &cache->trace[i];
		for (j = 0; j < tr->count; j += SECTORS_PER_PAGE) {
			bootcache_new_chunk(&cache->sectors, tr->sector + j);
			++sum;
		}
	}
	return 0;
}

/**
 * bootcache_start:
 *
 * Reads the bootcache header from disk, checks if it is valid
 * if valid:
 *   read the sector trace from disk
 *   build hash table for sector trace on page boundaries
 *   begin reading in sectors to be cached
 * else:
 *   setup to capture trace of sectors
 *
 * on error: by pass boot cache
 */
static void bootcache_start(struct work_struct *work)
{
	struct bootcache *cache = container_of(work, struct bootcache,
						work.work);
	struct bootcache_hdr hdr;
	int rc;

	rc = bootcache_dev_read(cache, &hdr, sizeof(hdr), cache->hdr.sector);
	if (rc) {
		DMERR("bootcache_dev_read hdr %d", rc);
		goto error;
	}
	if (is_valid_hdr(cache, &hdr)) {
		cache->is_valid = 1;
		memcpy(&cache->hdr, &hdr, sizeof(cache->hdr));
		rc = build_sector_map(&cache->sectors,
				sectors_to_pages(cache->hdr.sectors_data));
		if (rc)
			goto error;
		rc = read_trace(cache);
		if (rc)
			goto error;
		atomic_set(&cache->state, BC_FILLING);
		rc = bootcache_read_sectors(cache);
		if (rc)
			goto error;
	} else {
		atomic_set(&cache->state, BC_TRACING);
		cache->trace = kzalloc(sizeof(*cache->trace) *
					cache->args.max_trace, GFP_KERNEL);
		if (!cache->trace) {
			DMERR("cache->trace out of memory");
			goto error;
		}
	}
exit:
	complete_all(&cache->init_complete);
	return;
error:
	DMERR("error occured starting bootcache, setting to by pass mode");
	atomic_set(&cache->state, BC_BYPASS);
	cache->is_valid = 0;
	goto exit;
}

/**
 * bootcache_max_io determines the maximum number of pages that can
 * be passed in one read request to the underlying device.
 * @cache:           the max_sectors and max_hw_sectors must
 *                   be filled in.
 * @proposed_max_io: maxium number of pages the caller wants
 *                   to read at a time.
 *
 * Returns maximum number of pages that can be read but
 * no more than proposed_max_io
 */
static u32 bootcache_max_io(struct bootcache *cache, u32 proposed_max_io)
{
	u32 max_sectors;
	u32 max_pages;

	max_sectors = min(cache->hdr.max_sectors, cache->hdr.max_hw_sectors);
	max_pages = sectors_to_pages(max_sectors);
	if (proposed_max_io < max_pages)
		max_pages = proposed_max_io;
	return max_pages;
}

static void bootcache_init_hdr(struct bootcache_hdr *hdr, u64 cache_start,
			struct block_device *bdev, const char *signature)
{
	hdr->sector = cache_start;
	hdr->magic = BOOTCACHE_MAGIC;
	hdr->version = BOOTCACHE_VERSION;
	hdr->state = BC_INIT;
	hdr->alignment = PAGE_SIZE;
	hdr->max_hw_sectors = queue_max_hw_sectors(bdev_get_queue(bdev));
	hdr->max_sectors = queue_max_sectors(bdev_get_queue(bdev));
	strncpy(hdr->timestamp, utsname()->version, sizeof(hdr->timestamp));
	strncpy(hdr->signature, signature, sizeof(hdr->signature));
}

static int bootcache_get_device(
	struct dm_target *ti,
	char *devname,
	sector_t dev_start,
	sector_t dev_len,
	struct dm_dev **dm_dev)
{
	do {
		/* Try the normal path first since if everything is ready, it
		 * will be the fastest.
		 */
		if (!dm_get_device(ti, devname,
				   dm_table_get_mode(ti->table), dm_dev))
			return 0;

		/* No need to be too aggressive since this is a slow path. */
		msleep(500);
	} while (driver_probe_done() != 0 || *dm_dev == NULL);
	async_synchronize_full();
	return -1;
}

/**
 * bootcache_ctr - Construct a boot cache
 * @ti:   Target being created
 * @argc: Number of elements in argv
 * @argv: Vector of arguments - All arguments are positional, this
 *		means that to set a particular argument, all of its
 *		predecessors must be present.
 *
 * Accepts the folowing parametes [defaults in brackets]:
 * @device:      Device being cached. The boot cache is alsoe stored here.
 * @cache_start: Sector start on the device for the boot cache.
 * @signature:   Signature to determine if cache is valid.
 * @size_limit:  In sectors, max size reads to include in cache [128]
 * @max_trace:   Number of entries in block trace made during boot [8192]
 * @max_pages:   Maximum number of pages to cache in memory [50000]
 *
 * Argument list:
 * [<dev> [<cache_start> [<sig> [<size_limit> [<max_trace> [<max_limit>]]]]]]
 *
 * Example:
 * PARTUUID=0f5dbd05-c063-a848-a296-b8b8c2c24b28/PARTNROFF=1 1741200
 *    10e8...78 80 64000 60000
 */
static int bootcache_ctr(struct dm_target *ti, unsigned argc, char **argv)
{
	struct bootcache *cache = NULL;
	const char *signature = NULL;
	char *device = NULL;
	u64 cache_start = 0;
	u64 max_pages = DEFAULT_MAX_PAGES;
	u64 size_limit = DEFAULT_SIZE_LIMIT;
	u64 max_trace  = DEFAULT_MAX_TRACE;
	int rc = 0;

	if (argc > 0)
		device = argv[0];
	if (argc > 1)
		if (strict_strtoull(argv[1], 10, &cache_start)) {
			ti->error = "Invalid cache_start";
			return -EINVAL;
		}
	if (argc > 2)
		signature = argv[2];
	if (argc > 3)
		if (strict_strtoull(argv[3], 10, &size_limit)) {
			ti->error = "Invalid size_limit";
			return -EINVAL;
		}
	if (argc > 4)
		if (strict_strtoull(argv[4], 10, &max_trace)) {
			ti->error = "Invalid max_trace";
			return -EINVAL;
		}
	if (argc > 5)
		if (strict_strtoull(argv[5], 10, &max_pages)) {
			ti->error = "Invalid max_pages";
			return -EINVAL;
		}

#define NEEDARG(n) \
	if (!(n)) { \
		ti->error = "Missing argument: " #n; \
		return -EINVAL; \
	}

	NEEDARG(device);
	NEEDARG(signature);
	NEEDARG(cache_start);

#undef NEEDARG
	if ((dm_table_get_mode(ti->table) & DEV_MODE) != DEV_MODE) {
		ti->error = "Must be created read only.";
		return -EINVAL;
	}

	cache = kzalloc(sizeof(*cache), GFP_KERNEL);
	if (!cache)
		goto bad_cache;
	init_completion(&cache->init_complete);
	cache->ti = ti;

	strlcpy(cache->args.device, device, sizeof(cache->args.device));
	strlcpy(cache->args.signature, signature,
		sizeof(cache->args.signature));
	cache->args.cache_start = cache_start;
	cache->args.max_pages = max_pages;
	cache->args.size_limit = size_limit;
	if (max_trace > MAX_TRACE) {
		DMWARN("max_trace too large %llu, setting to %d\n",
			max_trace, MAX_TRACE);
		max_trace = MAX_TRACE;
	}
	cache->args.max_trace = max_trace;

	cache->begin = ti->begin;
	cache->len   = ti->len;

	atomic_set(&cache->state, BC_INIT);
	kref_init(&cache->kref);
	mutex_init(&cache->cache_lock);
	spin_lock_init(&cache->trace_lock);

	/* For the name, use the device default with / changed to _ */
	cache->name = dm_disk(dm_table_get_md(ti->table))->disk_name;

	if (bootcache_init_sysfs(cache, ti))
		goto bad_sysfs;

	rc = bootcache_get_device(ti, device,
		ti->begin, ti->len, &cache->dev);
	if (rc) {
		DMERR("Failed to acquire device '%s': %d", device, rc);
		ti->error = "Device lookup failed";
		goto bad_dev;
	}

	bootcache_init_hdr(&cache->hdr, cache_start,
				cache->dev->bdev, signature);
	cache->max_io = bootcache_max_io(cache, BIO_MAX_PAGES);

	/* Allocate the bioset used for request padding */
	cache->bio_set = bioset_create(cache->max_io * 4, 0);
	if (!cache->bio_set) {
		ti->error = "Cannot allocate verity bioset";
		goto bad_bio_set;
	}

	ti->num_flush_bios = 1;
	ti->private = cache;

	{
		char vdev[BDEVNAME_SIZE];
		bdevname(cache->dev->bdev, vdev);
		DMINFO("dev:%s", vdev);
	}
	INIT_WORK(&cache->work.work, bootcache_start);
	schedule_work(&cache->work.work);

	DMINFO("cache:%p", cache);
	return 0;

bad_bio_set:
	dm_put_device(ti, cache->dev);
bad_dev:
	bootcache_remove_all_files(cache);
bad_sysfs:
	kfree(cache);   /* hash is not secret so no need to zero */
bad_cache:
	return -EINVAL;
}

static void bootcache_status(struct dm_target *ti, status_type_t type,
			    unsigned status_flags, char *result, uint maxlen)
{
	struct bootcache *cache = (struct bootcache *) ti->private;
	uint sz = 0;
	char vdev[BDEVNAME_SIZE];

	switch (type) {
	case STATUSTYPE_INFO:
		DMEMIT("%u %u %u",
		       cache->stats.num_requests,
		       cache->stats.num_hits,
		       cache->stats.overlapped);
		break;

	case STATUSTYPE_TABLE:
		bdevname(cache->dev->bdev, vdev);
		DMEMIT("/dev/%s signature=%s cache_start=%llu max_pages=%llu"
			" size_limit=%llu max_trace=%llu\n",
			vdev,
			cache->args.signature,
			cache->args.cache_start,
			cache->args.max_pages,
			cache->args.size_limit,
			cache->args.max_trace);
		break;
	}
}

static void bootcache_dtr(struct dm_target *ti)
{
	/*
	 * Doesn't have to clean-up the meta files in sysfs
	 * because the device mapper has already done it.
	 */
	struct bootcache *cache = (struct bootcache *)ti->private;

	DMDEBUG("Destroying bio set");
	bioset_free(cache->bio_set);

	DMDEBUG("Putting dev");
	dm_put_device(ti, cache->dev);

	DMDEBUG("Destroying config");
	kfree(cache);
}

static int bootcache_map(struct dm_target *ti, struct bio *bio)
{
	bootcache_read(ti->private, bio);
	return DM_MAPIO_SUBMITTED;
}

static int bootcache_merge(struct dm_target *ti, struct bvec_merge_data *bvm,
				struct bio_vec *biovec, int max_size)
{
	struct bootcache *cache = ti->private;
	struct request_queue *q = bdev_get_queue(cache->dev->bdev);

	if (!q->merge_bvec_fn)
		return max_size;

	bvm->bi_bdev = cache->dev->bdev;
	bvm->bi_sector = cache->begin +
				bvm->bi_sector - ti->begin;

	/* Optionally, this could just return 0 to stick to single pages. */
	return min(max_size, q->merge_bvec_fn(q, bvm, biovec));
}

static int bootcache_iterate_devices(struct dm_target *ti,
				iterate_devices_callout_fn fn, void *data)
{
	struct bootcache *cache = ti->private;

	return fn(ti, cache->dev, cache->begin, ti->len, data);
}

static void bootcache_io_hints(struct dm_target *ti,
			    struct queue_limits *limits)
{
	limits->logical_block_size = PAGE_SIZE;
	limits->physical_block_size = PAGE_SIZE;
	blk_limits_io_min(limits, PAGE_SIZE);
}

static struct target_type bootcache_target = {
	.name   = "bootcache",
	.version = {0, 1, 0},
	.module = THIS_MODULE,
	.ctr    = bootcache_ctr,
	.dtr    = bootcache_dtr,
	.map    = bootcache_map,
	.merge  = bootcache_merge,
	.status = bootcache_status,
	.iterate_devices = bootcache_iterate_devices,
	.io_hints = bootcache_io_hints,
};

static int __init dm_bootcache_init(void)
{
	int rc = -ENOMEM;

	rc = dm_register_target(&bootcache_target);
	if (rc < 0) {
		DMERR("register failed %d", rc);
		goto register_failed;
	}

	DMINFO("version %u.%u.%u loaded", bootcache_target.version[0],
	       bootcache_target.version[1], bootcache_target.version[2]);

	return rc;

register_failed:
	return rc;
}

static void __exit dm_bootcache_exit(void)
{
	dm_unregister_target(&bootcache_target);
}

module_init(dm_bootcache_init);
module_exit(dm_bootcache_exit);

MODULE_AUTHOR("Paul Taysom <taysom@chromium.org>");
MODULE_DESCRIPTION(DM_NAME "read cache");
MODULE_LICENSE("GPL");
