/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_MBCACHE_H
#define _LINUX_MBCACHE_H

#include <linux/hash.h>
#include <linux/list_bl.h>
#include <linux/list.h>
#include <linux/atomic.h>
#include <linux/fs.h>

struct mb_cache;

/* Cache entry flags */
enum {
	MBE_REFERENCED_B = 0,
	MBE_REUSABLE_B
};

struct mb_cache_entry {
	/* List of entries in cache - protected by cache->c_list_lock */
	struct list_head	e_list;
	/*
	 * Hash table list - protected by hash chain bitlock. The entry is
	 * guaranteed to be hashed while e_refcnt > 0.
	 */
	struct hlist_bl_node	e_hash_list;
	/*
	 * Entry refcount. Once it reaches zero, entry is unhashed and freed.
	 * While refcount > 0, the entry is guaranteed to stay in the hash and
	 * e.g. mb_cache_entry_try_delete() will fail.
	 */
	atomic_t		e_refcnt;
	/* Key in hash - stable during lifetime of the entry */
	u32			e_key;
	unsigned long		e_flags;
	/* User provided value - stable during lifetime of the entry */
	u64			e_value;
};

struct mb_cache *mb_cache_create(int bucket_bits);
void mb_cache_destroy(struct mb_cache *cache);

int mb_cache_entry_create(struct mb_cache *cache, gfp_t mask, u32 key,
			  u64 value, bool reusable);
void __mb_cache_entry_free(struct mb_cache *cache,
			   struct mb_cache_entry *entry);
void mb_cache_entry_wait_unused(struct mb_cache_entry *entry);
static inline void mb_cache_entry_put(struct mb_cache *cache,
				      struct mb_cache_entry *entry)
{
	unsigned int cnt = atomic_dec_return(&entry->e_refcnt);

	if (cnt > 0) {
		if (cnt <= 2)
			wake_up_var(&entry->e_refcnt);
		return;
	}
	__mb_cache_entry_free(cache, entry);
}

struct mb_cache_entry *mb_cache_entry_delete_or_get(struct mb_cache *cache,
						    u32 key, u64 value);
void mb_cache_entry_delete(struct mb_cache *cache, u32 key, u64 value);
struct mb_cache_entry *mb_cache_entry_get(struct mb_cache *cache, u32 key,
					  u64 value);
struct mb_cache_entry *mb_cache_entry_find_first(struct mb_cache *cache,
						 u32 key);
struct mb_cache_entry *mb_cache_entry_find_next(struct mb_cache *cache,
						struct mb_cache_entry *entry);
void mb_cache_entry_touch(struct mb_cache *cache,
			  struct mb_cache_entry *entry);

#endif	/* _LINUX_MBCACHE_H */
