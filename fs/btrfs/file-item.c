// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2007 Oracle.  All rights reserved.
 */

#include <linux/bio.h>
#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/highmem.h>
#include <linux/sched/mm.h>
#include <crypto/hash.h>
#include "ctree.h"
#include "disk-io.h"
#include "transaction.h"
#include "volumes.h"
#include "print-tree.h"
#include "compression.h"

#define __MAX_CSUM_ITEMS(r, size) ((unsigned long)(((BTRFS_LEAF_DATA_SIZE(r) - \
				   sizeof(struct btrfs_item) * 2) / \
				  size) - 1))

#define MAX_CSUM_ITEMS(r, size) (min_t(u32, __MAX_CSUM_ITEMS(r, size), \
				       PAGE_SIZE))

/**
 * @inode - the inode we want to update the disk_i_size for
 * @new_i_size - the i_size we want to set to, 0 if we use i_size
 *
 * With NO_HOLES set this simply sets the disk_is_size to whatever i_size_read()
 * returns as it is perfectly fine with a file that has holes without hole file
 * extent items.
 *
 * However without NO_HOLES we need to only return the area that is contiguous
 * from the 0 offset of the file.  Otherwise we could end up adjust i_size up
 * to an extent that has a gap in between.
 *
 * Finally new_i_size should only be set in the case of truncate where we're not
 * ready to use i_size_read() as the limiter yet.
 */
void btrfs_inode_safe_disk_i_size_write(struct inode *inode, u64 new_i_size)
{
	struct btrfs_fs_info *fs_info = BTRFS_I(inode)->root->fs_info;
	u64 start, end, i_size;
	int ret;

	i_size = new_i_size ?: i_size_read(inode);
	if (btrfs_fs_incompat(fs_info, NO_HOLES)) {
		BTRFS_I(inode)->disk_i_size = i_size;
		return;
	}

	spin_lock(&BTRFS_I(inode)->lock);
	ret = find_contiguous_extent_bit(&BTRFS_I(inode)->file_extent_tree, 0,
					 &start, &end, EXTENT_DIRTY);
	if (!ret && start == 0)
		i_size = min(i_size, end + 1);
	else
		i_size = 0;
	BTRFS_I(inode)->disk_i_size = i_size;
	spin_unlock(&BTRFS_I(inode)->lock);
}

/**
 * @inode - the inode we're modifying
 * @start - the start file offset of the file extent we've inserted
 * @len - the logical length of the file extent item
 *
 * Call when we are inserting a new file extent where there was none before.
 * Does not need to call this in the case where we're replacing an existing file
 * extent, however if not sure it's fine to call this multiple times.
 *
 * The start and len must match the file extent item, so thus must be sectorsize
 * aligned.
 */
int btrfs_inode_set_file_extent_range(struct btrfs_inode *inode, u64 start,
				      u64 len)
{
	if (len == 0)
		return 0;

	ASSERT(IS_ALIGNED(start + len, inode->root->fs_info->sectorsize));

	if (btrfs_fs_incompat(inode->root->fs_info, NO_HOLES))
		return 0;
	return set_extent_bits(&inode->file_extent_tree, start, start + len - 1,
			       EXTENT_DIRTY);
}

/**
 * @inode - the inode we're modifying
 * @start - the start file offset of the file extent we've inserted
 * @len - the logical length of the file extent item
 *
 * Called when we drop a file extent, for example when we truncate.  Doesn't
 * need to be called for cases where we're replacing a file extent, like when
 * we've COWed a file extent.
 *
 * The start and len must match the file extent item, so thus must be sectorsize
 * aligned.
 */
int btrfs_inode_clear_file_extent_range(struct btrfs_inode *inode, u64 start,
					u64 len)
{
	if (len == 0)
		return 0;

	ASSERT(IS_ALIGNED(start + len, inode->root->fs_info->sectorsize) ||
	       len == (u64)-1);

	if (btrfs_fs_incompat(inode->root->fs_info, NO_HOLES))
		return 0;
	return clear_extent_bit(&inode->file_extent_tree, start,
				start + len - 1, EXTENT_DIRTY, 0, 0, NULL);
}

static inline u32 max_ordered_sum_bytes(struct btrfs_fs_info *fs_info,
					u16 csum_size)
{
	u32 ncsums = (PAGE_SIZE - sizeof(struct btrfs_ordered_sum)) / csum_size;

	return ncsums * fs_info->sectorsize;
}

int btrfs_insert_file_extent(struct btrfs_trans_handle *trans,
			     struct btrfs_root *root,
			     u64 objectid, u64 pos,
			     u64 disk_offset, u64 disk_num_bytes,
			     u64 num_bytes, u64 offset, u64 ram_bytes,
			     u8 compression, u8 encryption, u16 other_encoding)
{
	int ret = 0;
	struct btrfs_file_extent_item *item;
	struct btrfs_key file_key;
	struct btrfs_path *path;
	struct extent_buffer *leaf;

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;
	file_key.objectid = objectid;
	file_key.offset = pos;
	file_key.type = BTRFS_EXTENT_DATA_KEY;

	path->leave_spinning = 1;
	ret = btrfs_insert_empty_item(trans, root, path, &file_key,
				      sizeof(*item));
	if (ret < 0)
		goto out;
	BUG_ON(ret); /* Can't happen */
	leaf = path->nodes[0];
	item = btrfs_item_ptr(leaf, path->slots[0],
			      struct btrfs_file_extent_item);
	btrfs_set_file_extent_disk_bytenr(leaf, item, disk_offset);
	btrfs_set_file_extent_disk_num_bytes(leaf, item, disk_num_bytes);
	btrfs_set_file_extent_offset(leaf, item, offset);
	btrfs_set_file_extent_num_bytes(leaf, item, num_bytes);
	btrfs_set_file_extent_ram_bytes(leaf, item, ram_bytes);
	btrfs_set_file_extent_generation(leaf, item, trans->transid);
	btrfs_set_file_extent_type(leaf, item, BTRFS_FILE_EXTENT_REG);
	btrfs_set_file_extent_compression(leaf, item, compression);
	btrfs_set_file_extent_encryption(leaf, item, encryption);
	btrfs_set_file_extent_other_encoding(leaf, item, other_encoding);

	btrfs_mark_buffer_dirty(leaf);
out:
	btrfs_free_path(path);
	return ret;
}

static struct btrfs_csum_item *
btrfs_lookup_csum(struct btrfs_trans_handle *trans,
		  struct btrfs_root *root,
		  struct btrfs_path *path,
		  u64 bytenr, int cow)
{
	struct btrfs_fs_info *fs_info = root->fs_info;
	int ret;
	struct btrfs_key file_key;
	struct btrfs_key found_key;
	struct btrfs_csum_item *item;
	struct extent_buffer *leaf;
	u64 csum_offset = 0;
	u16 csum_size = btrfs_super_csum_size(fs_info->super_copy);
	int csums_in_item;

	file_key.objectid = BTRFS_EXTENT_CSUM_OBJECTID;
	file_key.offset = bytenr;
	file_key.type = BTRFS_EXTENT_CSUM_KEY;
	ret = btrfs_search_slot(trans, root, &file_key, path, 0, cow);
	if (ret < 0)
		goto fail;
	leaf = path->nodes[0];
	if (ret > 0) {
		ret = 1;
		if (path->slots[0] == 0)
			goto fail;
		path->slots[0]--;
		btrfs_item_key_to_cpu(leaf, &found_key, path->slots[0]);
		if (found_key.type != BTRFS_EXTENT_CSUM_KEY)
			goto fail;

		csum_offset = (bytenr - found_key.offset) >>
				fs_info->sb->s_blocksize_bits;
		csums_in_item = btrfs_item_size_nr(leaf, path->slots[0]);
		csums_in_item /= csum_size;

		if (csum_offset == csums_in_item) {
			ret = -EFBIG;
			goto fail;
		} else if (csum_offset > csums_in_item) {
			goto fail;
		}
	}
	item = btrfs_item_ptr(leaf, path->slots[0], struct btrfs_csum_item);
	item = (struct btrfs_csum_item *)((unsigned char *)item +
					  csum_offset * csum_size);
	return item;
fail:
	if (ret > 0)
		ret = -ENOENT;
	return ERR_PTR(ret);
}

int btrfs_lookup_file_extent(struct btrfs_trans_handle *trans,
			     struct btrfs_root *root,
			     struct btrfs_path *path, u64 objectid,
			     u64 offset, int mod)
{
	int ret;
	struct btrfs_key file_key;
	int ins_len = mod < 0 ? -1 : 0;
	int cow = mod != 0;

	file_key.objectid = objectid;
	file_key.offset = offset;
	file_key.type = BTRFS_EXTENT_DATA_KEY;
	ret = btrfs_search_slot(trans, root, &file_key, path, ins_len, cow);
	return ret;
}

/**
 * btrfs_lookup_bio_sums - Look up checksums for a bio.
 * @inode: inode that the bio is for.
 * @bio: bio to look up.
 * @offset: Unless (u64)-1, look up checksums for this offset in the file.
 *          If (u64)-1, use the page offsets from the bio instead.
 * @dst: Buffer of size nblocks * btrfs_super_csum_size() used to return
 *       checksum (nblocks = bio->bi_iter.bi_size / fs_info->sectorsize). If
 *       NULL, the checksum buffer is allocated and returned in
 *       btrfs_io_bio(bio)->csum instead.
 *
 * Return: BLK_STS_RESOURCE if allocating memory fails, BLK_STS_OK otherwise.
 */
blk_status_t btrfs_lookup_bio_sums(struct inode *inode, struct bio *bio,
				   u64 offset, u8 *dst)
{
	struct btrfs_fs_info *fs_info = btrfs_sb(inode->i_sb);
	struct bio_vec bvec;
	struct bvec_iter iter;
	struct btrfs_csum_item *item = NULL;
	struct extent_io_tree *io_tree = &BTRFS_I(inode)->io_tree;
	struct btrfs_path *path;
	const bool page_offsets = (offset == (u64)-1);
	u8 *csum;
	u64 item_start_offset = 0;
	u64 item_last_offset = 0;
	u64 disk_bytenr;
	u64 page_bytes_left;
	u32 diff;
	int nblocks;
	int count = 0;
	u16 csum_size = btrfs_super_csum_size(fs_info->super_copy);

	path = btrfs_alloc_path();
	if (!path)
		return BLK_STS_RESOURCE;

	nblocks = bio->bi_iter.bi_size >> inode->i_sb->s_blocksize_bits;
	if (!dst) {
		struct btrfs_io_bio *btrfs_bio = btrfs_io_bio(bio);

		if (nblocks * csum_size > BTRFS_BIO_INLINE_CSUM_SIZE) {
			btrfs_bio->csum = kmalloc_array(nblocks, csum_size,
							GFP_NOFS);
			if (!btrfs_bio->csum) {
				btrfs_free_path(path);
				return BLK_STS_RESOURCE;
			}
		} else {
			btrfs_bio->csum = btrfs_bio->csum_inline;
		}
		csum = btrfs_bio->csum;
	} else {
		csum = dst;
	}

	if (bio->bi_iter.bi_size > PAGE_SIZE * 8)
		path->reada = READA_FORWARD;

	/*
	 * the free space stuff is only read when it hasn't been
	 * updated in the current transaction.  So, we can safely
	 * read from the commit root and sidestep a nasty deadlock
	 * between reading the free space cache and updating the csum tree.
	 */
	if (btrfs_is_free_space_inode(BTRFS_I(inode))) {
		path->search_commit_root = 1;
		path->skip_locking = 1;
	}

	disk_bytenr = (u64)bio->bi_iter.bi_sector << 9;

	bio_for_each_segment(bvec, bio, iter) {
		page_bytes_left = bvec.bv_len;
		if (count)
			goto next;

		if (page_offsets)
			offset = page_offset(bvec.bv_page) + bvec.bv_offset;
		count = btrfs_find_ordered_sum(BTRFS_I(inode), offset,
					       disk_bytenr, csum, nblocks);
		if (count)
			goto found;

		if (!item || disk_bytenr < item_start_offset ||
		    disk_bytenr >= item_last_offset) {
			struct btrfs_key found_key;
			u32 item_size;

			if (item)
				btrfs_release_path(path);
			item = btrfs_lookup_csum(NULL, fs_info->csum_root,
						 path, disk_bytenr, 0);
			if (IS_ERR(item)) {
				count = 1;
				memset(csum, 0, csum_size);
				if (BTRFS_I(inode)->root->root_key.objectid ==
				    BTRFS_DATA_RELOC_TREE_OBJECTID) {
					set_extent_bits(io_tree, offset,
						offset + fs_info->sectorsize - 1,
						EXTENT_NODATASUM);
				} else {
					btrfs_info_rl(fs_info,
						   "no csum found for inode %llu start %llu",
					       btrfs_ino(BTRFS_I(inode)), offset);
				}
				item = NULL;
				btrfs_release_path(path);
				goto found;
			}
			btrfs_item_key_to_cpu(path->nodes[0], &found_key,
					      path->slots[0]);

			item_start_offset = found_key.offset;
			item_size = btrfs_item_size_nr(path->nodes[0],
						       path->slots[0]);
			item_last_offset = item_start_offset +
				(item_size / csum_size) *
				fs_info->sectorsize;
			item = btrfs_item_ptr(path->nodes[0], path->slots[0],
					      struct btrfs_csum_item);
		}
		/*
		 * this byte range must be able to fit inside
		 * a single leaf so it will also fit inside a u32
		 */
		diff = disk_bytenr - item_start_offset;
		diff = diff / fs_info->sectorsize;
		diff = diff * csum_size;
		count = min_t(int, nblocks, (item_last_offset - disk_bytenr) >>
					    inode->i_sb->s_blocksize_bits);
		read_extent_buffer(path->nodes[0], csum,
				   ((unsigned long)item) + diff,
				   csum_size * count);
found:
		csum += count * csum_size;
		nblocks -= count;
next:
		while (count > 0) {
			count--;
			disk_bytenr += fs_info->sectorsize;
			offset += fs_info->sectorsize;
			page_bytes_left -= fs_info->sectorsize;
			if (!page_bytes_left)
				break; /* move to next bio */
		}
	}

	WARN_ON_ONCE(count);
	btrfs_free_path(path);
	return BLK_STS_OK;
}

int btrfs_lookup_csums_range(struct btrfs_root *root, u64 start, u64 end,
			     struct list_head *list, int search_commit)
{
	struct btrfs_fs_info *fs_info = root->fs_info;
	struct btrfs_key key;
	struct btrfs_path *path;
	struct extent_buffer *leaf;
	struct btrfs_ordered_sum *sums;
	struct btrfs_csum_item *item;
	LIST_HEAD(tmplist);
	unsigned long offset;
	int ret;
	size_t size;
	u64 csum_end;
	u16 csum_size = btrfs_super_csum_size(fs_info->super_copy);

	ASSERT(IS_ALIGNED(start, fs_info->sectorsize) &&
	       IS_ALIGNED(end + 1, fs_info->sectorsize));

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	if (search_commit) {
		path->skip_locking = 1;
		path->reada = READA_FORWARD;
		path->search_commit_root = 1;
	}

	key.objectid = BTRFS_EXTENT_CSUM_OBJECTID;
	key.offset = start;
	key.type = BTRFS_EXTENT_CSUM_KEY;

	ret = btrfs_search_slot(NULL, root, &key, path, 0, 0);
	if (ret < 0)
		goto fail;
	if (ret > 0 && path->slots[0] > 0) {
		leaf = path->nodes[0];
		btrfs_item_key_to_cpu(leaf, &key, path->slots[0] - 1);
		if (key.objectid == BTRFS_EXTENT_CSUM_OBJECTID &&
		    key.type == BTRFS_EXTENT_CSUM_KEY) {
			offset = (start - key.offset) >>
				 fs_info->sb->s_blocksize_bits;
			if (offset * csum_size <
			    btrfs_item_size_nr(leaf, path->slots[0] - 1))
				path->slots[0]--;
		}
	}

	while (start <= end) {
		leaf = path->nodes[0];
		if (path->slots[0] >= btrfs_header_nritems(leaf)) {
			ret = btrfs_next_leaf(root, path);
			if (ret < 0)
				goto fail;
			if (ret > 0)
				break;
			leaf = path->nodes[0];
		}

		btrfs_item_key_to_cpu(leaf, &key, path->slots[0]);
		if (key.objectid != BTRFS_EXTENT_CSUM_OBJECTID ||
		    key.type != BTRFS_EXTENT_CSUM_KEY ||
		    key.offset > end)
			break;

		if (key.offset > start)
			start = key.offset;

		size = btrfs_item_size_nr(leaf, path->slots[0]);
		csum_end = key.offset + (size / csum_size) * fs_info->sectorsize;
		if (csum_end <= start) {
			path->slots[0]++;
			continue;
		}

		csum_end = min(csum_end, end + 1);
		item = btrfs_item_ptr(path->nodes[0], path->slots[0],
				      struct btrfs_csum_item);
		while (start < csum_end) {
			size = min_t(size_t, csum_end - start,
				     max_ordered_sum_bytes(fs_info, csum_size));
			sums = kzalloc(btrfs_ordered_sum_size(fs_info, size),
				       GFP_NOFS);
			if (!sums) {
				ret = -ENOMEM;
				goto fail;
			}

			sums->bytenr = start;
			sums->len = (int)size;

			offset = (start - key.offset) >>
				fs_info->sb->s_blocksize_bits;
			offset *= csum_size;
			size >>= fs_info->sb->s_blocksize_bits;

			read_extent_buffer(path->nodes[0],
					   sums->sums,
					   ((unsigned long)item) + offset,
					   csum_size * size);

			start += fs_info->sectorsize * size;
			list_add_tail(&sums->list, &tmplist);
		}
		path->slots[0]++;
	}
	ret = 0;
fail:
	while (ret < 0 && !list_empty(&tmplist)) {
		sums = list_entry(tmplist.next, struct btrfs_ordered_sum, list);
		list_del(&sums->list);
		kfree(sums);
	}
	list_splice_tail(&tmplist, list);

	btrfs_free_path(path);
	return ret;
}

/*
 * btrfs_csum_one_bio - Calculates checksums of the data contained inside a bio
 * @inode:	 Owner of the data inside the bio
 * @bio:	 Contains the data to be checksummed
 * @file_start:  offset in file this bio begins to describe
 * @contig:	 Boolean. If true/1 means all bio vecs in this bio are
 *		 contiguous and they begin at @file_start in the file. False/0
 *		 means this bio can contains potentially discontigous bio vecs
 *		 so the logical offset of each should be calculated separately.
 */
blk_status_t btrfs_csum_one_bio(struct btrfs_inode *inode, struct bio *bio,
		       u64 file_start, int contig)
{
	struct btrfs_fs_info *fs_info = inode->root->fs_info;
	SHASH_DESC_ON_STACK(shash, fs_info->csum_shash);
	struct btrfs_ordered_sum *sums;
	struct btrfs_ordered_extent *ordered = NULL;
	char *data;
	struct bvec_iter iter;
	struct bio_vec bvec;
	int index;
	int nr_sectors;
	unsigned long total_bytes = 0;
	unsigned long this_sum_bytes = 0;
	int i;
	u64 offset;
	unsigned nofs_flag;
	const u16 csum_size = btrfs_super_csum_size(fs_info->super_copy);

	nofs_flag = memalloc_nofs_save();
	sums = kvzalloc(btrfs_ordered_sum_size(fs_info, bio->bi_iter.bi_size),
		       GFP_KERNEL);
	memalloc_nofs_restore(nofs_flag);

	if (!sums)
		return BLK_STS_RESOURCE;

	sums->len = bio->bi_iter.bi_size;
	INIT_LIST_HEAD(&sums->list);

	if (contig)
		offset = file_start;
	else
		offset = 0; /* shut up gcc */

	sums->bytenr = (u64)bio->bi_iter.bi_sector << 9;
	index = 0;

	shash->tfm = fs_info->csum_shash;

	bio_for_each_segment(bvec, bio, iter) {
		if (!contig)
			offset = page_offset(bvec.bv_page) + bvec.bv_offset;

		if (!ordered) {
			ordered = btrfs_lookup_ordered_extent(inode, offset);
			/*
			 * The bio range is not covered by any ordered extent,
			 * must be a code logic error.
			 */
			if (unlikely(!ordered)) {
				WARN(1, KERN_WARNING
			"no ordered extent for root %llu ino %llu offset %llu\n",
				     inode->root->root_key.objectid,
				     btrfs_ino(inode), offset);
				kvfree(sums);
				return BLK_STS_IOERR;
			}
		}

		nr_sectors = BTRFS_BYTES_TO_BLKS(fs_info,
						 bvec.bv_len + fs_info->sectorsize
						 - 1);

		for (i = 0; i < nr_sectors; i++) {
			if (offset >= ordered->file_offset + ordered->num_bytes ||
			    offset < ordered->file_offset) {
				unsigned long bytes_left;

				sums->len = this_sum_bytes;
				this_sum_bytes = 0;
				btrfs_add_ordered_sum(ordered, sums);
				btrfs_put_ordered_extent(ordered);

				bytes_left = bio->bi_iter.bi_size - total_bytes;

				nofs_flag = memalloc_nofs_save();
				sums = kvzalloc(btrfs_ordered_sum_size(fs_info,
						      bytes_left), GFP_KERNEL);
				memalloc_nofs_restore(nofs_flag);
				if (!sums)
					return BLK_STS_RESOURCE;

				sums->len = bytes_left;
				ordered = btrfs_lookup_ordered_extent(inode,
								offset);
				ASSERT(ordered); /* Logic error */
				sums->bytenr = ((u64)bio->bi_iter.bi_sector << 9)
					+ total_bytes;
				index = 0;
			}

			data = kmap_atomic(bvec.bv_page);
			crypto_shash_digest(shash, data + bvec.bv_offset
					    + (i * fs_info->sectorsize),
					    fs_info->sectorsize,
					    sums->sums + index);
			kunmap_atomic(data);
			index += csum_size;
			offset += fs_info->sectorsize;
			this_sum_bytes += fs_info->sectorsize;
			total_bytes += fs_info->sectorsize;
		}

	}
	this_sum_bytes = 0;
	btrfs_add_ordered_sum(ordered, sums);
	btrfs_put_ordered_extent(ordered);
	return 0;
}

/*
 * helper function for csum removal, this expects the
 * key to describe the csum pointed to by the path, and it expects
 * the csum to overlap the range [bytenr, len]
 *
 * The csum should not be entirely contained in the range and the
 * range should not be entirely contained in the csum.
 *
 * This calls btrfs_truncate_item with the correct args based on the
 * overlap, and fixes up the key as required.
 */
static noinline void truncate_one_csum(struct btrfs_fs_info *fs_info,
				       struct btrfs_path *path,
				       struct btrfs_key *key,
				       u64 bytenr, u64 len)
{
	struct extent_buffer *leaf;
	u16 csum_size = btrfs_super_csum_size(fs_info->super_copy);
	u64 csum_end;
	u64 end_byte = bytenr + len;
	u32 blocksize_bits = fs_info->sb->s_blocksize_bits;

	leaf = path->nodes[0];
	csum_end = btrfs_item_size_nr(leaf, path->slots[0]) / csum_size;
	csum_end <<= fs_info->sb->s_blocksize_bits;
	csum_end += key->offset;

	if (key->offset < bytenr && csum_end <= end_byte) {
		/*
		 *         [ bytenr - len ]
		 *         [   ]
		 *   [csum     ]
		 *   A simple truncate off the end of the item
		 */
		u32 new_size = (bytenr - key->offset) >> blocksize_bits;
		new_size *= csum_size;
		btrfs_truncate_item(path, new_size, 1);
	} else if (key->offset >= bytenr && csum_end > end_byte &&
		   end_byte > key->offset) {
		/*
		 *         [ bytenr - len ]
		 *                 [ ]
		 *                 [csum     ]
		 * we need to truncate from the beginning of the csum
		 */
		u32 new_size = (csum_end - end_byte) >> blocksize_bits;
		new_size *= csum_size;

		btrfs_truncate_item(path, new_size, 0);

		key->offset = end_byte;
		btrfs_set_item_key_safe(fs_info, path, key);
	} else {
		BUG();
	}
}

/*
 * deletes the csum items from the csum tree for a given
 * range of bytes.
 */
int btrfs_del_csums(struct btrfs_trans_handle *trans,
		    struct btrfs_root *root, u64 bytenr, u64 len)
{
	struct btrfs_fs_info *fs_info = trans->fs_info;
	struct btrfs_path *path;
	struct btrfs_key key;
	u64 end_byte = bytenr + len;
	u64 csum_end;
	struct extent_buffer *leaf;
	int ret = 0;
	u16 csum_size = btrfs_super_csum_size(fs_info->super_copy);
	int blocksize_bits = fs_info->sb->s_blocksize_bits;

	ASSERT(root == fs_info->csum_root ||
	       root->root_key.objectid == BTRFS_TREE_LOG_OBJECTID);

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;

	while (1) {
		key.objectid = BTRFS_EXTENT_CSUM_OBJECTID;
		key.offset = end_byte - 1;
		key.type = BTRFS_EXTENT_CSUM_KEY;

		path->leave_spinning = 1;
		ret = btrfs_search_slot(trans, root, &key, path, -1, 1);
		if (ret > 0) {
			ret = 0;
			if (path->slots[0] == 0)
				break;
			path->slots[0]--;
		} else if (ret < 0) {
			break;
		}

		leaf = path->nodes[0];
		btrfs_item_key_to_cpu(leaf, &key, path->slots[0]);

		if (key.objectid != BTRFS_EXTENT_CSUM_OBJECTID ||
		    key.type != BTRFS_EXTENT_CSUM_KEY) {
			break;
		}

		if (key.offset >= end_byte)
			break;

		csum_end = btrfs_item_size_nr(leaf, path->slots[0]) / csum_size;
		csum_end <<= blocksize_bits;
		csum_end += key.offset;

		/* this csum ends before we start, we're done */
		if (csum_end <= bytenr)
			break;

		/* delete the entire item, it is inside our range */
		if (key.offset >= bytenr && csum_end <= end_byte) {
			int del_nr = 1;

			/*
			 * Check how many csum items preceding this one in this
			 * leaf correspond to our range and then delete them all
			 * at once.
			 */
			if (key.offset > bytenr && path->slots[0] > 0) {
				int slot = path->slots[0] - 1;

				while (slot >= 0) {
					struct btrfs_key pk;

					btrfs_item_key_to_cpu(leaf, &pk, slot);
					if (pk.offset < bytenr ||
					    pk.type != BTRFS_EXTENT_CSUM_KEY ||
					    pk.objectid !=
					    BTRFS_EXTENT_CSUM_OBJECTID)
						break;
					path->slots[0] = slot;
					del_nr++;
					key.offset = pk.offset;
					slot--;
				}
			}
			ret = btrfs_del_items(trans, root, path,
					      path->slots[0], del_nr);
			if (ret)
				break;
			if (key.offset == bytenr)
				break;
		} else if (key.offset < bytenr && csum_end > end_byte) {
			unsigned long offset;
			unsigned long shift_len;
			unsigned long item_offset;
			/*
			 *        [ bytenr - len ]
			 *     [csum                ]
			 *
			 * Our bytes are in the middle of the csum,
			 * we need to split this item and insert a new one.
			 *
			 * But we can't drop the path because the
			 * csum could change, get removed, extended etc.
			 *
			 * The trick here is the max size of a csum item leaves
			 * enough room in the tree block for a single
			 * item header.  So, we split the item in place,
			 * adding a new header pointing to the existing
			 * bytes.  Then we loop around again and we have
			 * a nicely formed csum item that we can neatly
			 * truncate.
			 */
			offset = (bytenr - key.offset) >> blocksize_bits;
			offset *= csum_size;

			shift_len = (len >> blocksize_bits) * csum_size;

			item_offset = btrfs_item_ptr_offset(leaf,
							    path->slots[0]);

			memzero_extent_buffer(leaf, item_offset + offset,
					     shift_len);
			key.offset = bytenr;

			/*
			 * btrfs_split_item returns -EAGAIN when the
			 * item changed size or key
			 */
			ret = btrfs_split_item(trans, root, path, &key, offset);
			if (ret && ret != -EAGAIN) {
				btrfs_abort_transaction(trans, ret);
				break;
			}
			ret = 0;

			key.offset = end_byte - 1;
		} else {
			truncate_one_csum(fs_info, path, &key, bytenr, len);
			if (key.offset < bytenr)
				break;
		}
		btrfs_release_path(path);
	}
	btrfs_free_path(path);
	return ret;
}

int btrfs_csum_file_blocks(struct btrfs_trans_handle *trans,
			   struct btrfs_root *root,
			   struct btrfs_ordered_sum *sums)
{
	struct btrfs_fs_info *fs_info = root->fs_info;
	struct btrfs_key file_key;
	struct btrfs_key found_key;
	struct btrfs_path *path;
	struct btrfs_csum_item *item;
	struct btrfs_csum_item *item_end;
	struct extent_buffer *leaf = NULL;
	u64 next_offset;
	u64 total_bytes = 0;
	u64 csum_offset;
	u64 bytenr;
	u32 nritems;
	u32 ins_size;
	int index = 0;
	int found_next;
	int ret;
	u16 csum_size = btrfs_super_csum_size(fs_info->super_copy);

	path = btrfs_alloc_path();
	if (!path)
		return -ENOMEM;
again:
	next_offset = (u64)-1;
	found_next = 0;
	bytenr = sums->bytenr + total_bytes;
	file_key.objectid = BTRFS_EXTENT_CSUM_OBJECTID;
	file_key.offset = bytenr;
	file_key.type = BTRFS_EXTENT_CSUM_KEY;

	item = btrfs_lookup_csum(trans, root, path, bytenr, 1);
	if (!IS_ERR(item)) {
		ret = 0;
		leaf = path->nodes[0];
		item_end = btrfs_item_ptr(leaf, path->slots[0],
					  struct btrfs_csum_item);
		item_end = (struct btrfs_csum_item *)((char *)item_end +
			   btrfs_item_size_nr(leaf, path->slots[0]));
		goto found;
	}
	ret = PTR_ERR(item);
	if (ret != -EFBIG && ret != -ENOENT)
		goto out;

	if (ret == -EFBIG) {
		u32 item_size;
		/* we found one, but it isn't big enough yet */
		leaf = path->nodes[0];
		item_size = btrfs_item_size_nr(leaf, path->slots[0]);
		if ((item_size / csum_size) >=
		    MAX_CSUM_ITEMS(fs_info, csum_size)) {
			/* already at max size, make a new one */
			goto insert;
		}
	} else {
		int slot = path->slots[0] + 1;
		/* we didn't find a csum item, insert one */
		nritems = btrfs_header_nritems(path->nodes[0]);
		if (!nritems || (path->slots[0] >= nritems - 1)) {
			ret = btrfs_next_leaf(root, path);
			if (ret < 0) {
				goto out;
			} else if (ret > 0) {
				found_next = 1;
				goto insert;
			}
			slot = path->slots[0];
		}
		btrfs_item_key_to_cpu(path->nodes[0], &found_key, slot);
		if (found_key.objectid != BTRFS_EXTENT_CSUM_OBJECTID ||
		    found_key.type != BTRFS_EXTENT_CSUM_KEY) {
			found_next = 1;
			goto insert;
		}
		next_offset = found_key.offset;
		found_next = 1;
		goto insert;
	}

	/*
	 * At this point, we know the tree has a checksum item that ends at an
	 * offset matching the start of the checksum range we want to insert.
	 * We try to extend that item as much as possible and then add as many
	 * checksums to it as they fit.
	 *
	 * First check if the leaf has enough free space for at least one
	 * checksum. If it has go directly to the item extension code, otherwise
	 * release the path and do a search for insertion before the extension.
	 */
	if (btrfs_leaf_free_space(leaf) >= csum_size) {
		btrfs_item_key_to_cpu(leaf, &found_key, path->slots[0]);
		csum_offset = (bytenr - found_key.offset) >>
			fs_info->sb->s_blocksize_bits;
		goto extend_csum;
	}

	btrfs_release_path(path);
	ret = btrfs_search_slot(trans, root, &file_key, path,
				csum_size, 1);
	if (ret < 0)
		goto out;

	if (ret > 0) {
		if (path->slots[0] == 0)
			goto insert;
		path->slots[0]--;
	}

	leaf = path->nodes[0];
	btrfs_item_key_to_cpu(leaf, &found_key, path->slots[0]);
	csum_offset = (bytenr - found_key.offset) >>
			fs_info->sb->s_blocksize_bits;

	if (found_key.type != BTRFS_EXTENT_CSUM_KEY ||
	    found_key.objectid != BTRFS_EXTENT_CSUM_OBJECTID ||
	    csum_offset >= MAX_CSUM_ITEMS(fs_info, csum_size)) {
		goto insert;
	}

extend_csum:
	if (csum_offset == btrfs_item_size_nr(leaf, path->slots[0]) /
	    csum_size) {
		int extend_nr;
		u64 tmp;
		u32 diff;

		tmp = sums->len - total_bytes;
		tmp >>= fs_info->sb->s_blocksize_bits;
		WARN_ON(tmp < 1);

		extend_nr = max_t(int, 1, (int)tmp);
		diff = (csum_offset + extend_nr) * csum_size;
		diff = min(diff,
			   MAX_CSUM_ITEMS(fs_info, csum_size) * csum_size);

		diff = diff - btrfs_item_size_nr(leaf, path->slots[0]);
		diff = min_t(u32, btrfs_leaf_free_space(leaf), diff);
		diff /= csum_size;
		diff *= csum_size;

		btrfs_extend_item(path, diff);
		ret = 0;
		goto csum;
	}

insert:
	btrfs_release_path(path);
	csum_offset = 0;
	if (found_next) {
		u64 tmp;

		tmp = sums->len - total_bytes;
		tmp >>= fs_info->sb->s_blocksize_bits;
		tmp = min(tmp, (next_offset - file_key.offset) >>
					 fs_info->sb->s_blocksize_bits);

		tmp = max_t(u64, 1, tmp);
		tmp = min_t(u64, tmp, MAX_CSUM_ITEMS(fs_info, csum_size));
		ins_size = csum_size * tmp;
	} else {
		ins_size = csum_size;
	}
	path->leave_spinning = 1;
	ret = btrfs_insert_empty_item(trans, root, path, &file_key,
				      ins_size);
	path->leave_spinning = 0;
	if (ret < 0)
		goto out;
	if (WARN_ON(ret != 0))
		goto out;
	leaf = path->nodes[0];
csum:
	item = btrfs_item_ptr(leaf, path->slots[0], struct btrfs_csum_item);
	item_end = (struct btrfs_csum_item *)((unsigned char *)item +
				      btrfs_item_size_nr(leaf, path->slots[0]));
	item = (struct btrfs_csum_item *)((unsigned char *)item +
					  csum_offset * csum_size);
found:
	ins_size = (u32)(sums->len - total_bytes) >>
		   fs_info->sb->s_blocksize_bits;
	ins_size *= csum_size;
	ins_size = min_t(u32, (unsigned long)item_end - (unsigned long)item,
			      ins_size);
	write_extent_buffer(leaf, sums->sums + index, (unsigned long)item,
			    ins_size);

	index += ins_size;
	ins_size /= csum_size;
	total_bytes += ins_size * fs_info->sectorsize;

	btrfs_mark_buffer_dirty(path->nodes[0]);
	if (total_bytes < sums->len) {
		btrfs_release_path(path);
		cond_resched();
		goto again;
	}
out:
	btrfs_free_path(path);
	return ret;
}

void btrfs_extent_item_to_extent_map(struct btrfs_inode *inode,
				     const struct btrfs_path *path,
				     struct btrfs_file_extent_item *fi,
				     const bool new_inline,
				     struct extent_map *em)
{
	struct btrfs_fs_info *fs_info = inode->root->fs_info;
	struct btrfs_root *root = inode->root;
	struct extent_buffer *leaf = path->nodes[0];
	const int slot = path->slots[0];
	struct btrfs_key key;
	u64 extent_start, extent_end;
	u64 bytenr;
	u8 type = btrfs_file_extent_type(leaf, fi);
	int compress_type = btrfs_file_extent_compression(leaf, fi);

	btrfs_item_key_to_cpu(leaf, &key, slot);
	extent_start = key.offset;
	extent_end = btrfs_file_extent_end(path);
	em->ram_bytes = btrfs_file_extent_ram_bytes(leaf, fi);
	if (type == BTRFS_FILE_EXTENT_REG ||
	    type == BTRFS_FILE_EXTENT_PREALLOC) {
		em->start = extent_start;
		em->len = extent_end - extent_start;
		em->orig_start = extent_start -
			btrfs_file_extent_offset(leaf, fi);
		em->orig_block_len = btrfs_file_extent_disk_num_bytes(leaf, fi);
		bytenr = btrfs_file_extent_disk_bytenr(leaf, fi);
		if (bytenr == 0) {
			em->block_start = EXTENT_MAP_HOLE;
			return;
		}
		if (compress_type != BTRFS_COMPRESS_NONE) {
			set_bit(EXTENT_FLAG_COMPRESSED, &em->flags);
			em->compress_type = compress_type;
			em->block_start = bytenr;
			em->block_len = em->orig_block_len;
		} else {
			bytenr += btrfs_file_extent_offset(leaf, fi);
			em->block_start = bytenr;
			em->block_len = em->len;
			if (type == BTRFS_FILE_EXTENT_PREALLOC)
				set_bit(EXTENT_FLAG_PREALLOC, &em->flags);
		}
	} else if (type == BTRFS_FILE_EXTENT_INLINE) {
		em->block_start = EXTENT_MAP_INLINE;
		em->start = extent_start;
		em->len = extent_end - extent_start;
		/*
		 * Initialize orig_start and block_len with the same values
		 * as in inode.c:btrfs_get_extent().
		 */
		em->orig_start = EXTENT_MAP_HOLE;
		em->block_len = (u64)-1;
		if (!new_inline && compress_type != BTRFS_COMPRESS_NONE) {
			set_bit(EXTENT_FLAG_COMPRESSED, &em->flags);
			em->compress_type = compress_type;
		}
	} else {
		btrfs_err(fs_info,
			  "unknown file extent item type %d, inode %llu, offset %llu, "
			  "root %llu", type, btrfs_ino(inode), extent_start,
			  root->root_key.objectid);
	}
}

/*
 * Returns the end offset (non inclusive) of the file extent item the given path
 * points to. If it points to an inline extent, the returned offset is rounded
 * up to the sector size.
 */
u64 btrfs_file_extent_end(const struct btrfs_path *path)
{
	const struct extent_buffer *leaf = path->nodes[0];
	const int slot = path->slots[0];
	struct btrfs_file_extent_item *fi;
	struct btrfs_key key;
	u64 end;

	btrfs_item_key_to_cpu(leaf, &key, slot);
	ASSERT(key.type == BTRFS_EXTENT_DATA_KEY);
	fi = btrfs_item_ptr(leaf, slot, struct btrfs_file_extent_item);

	if (btrfs_file_extent_type(leaf, fi) == BTRFS_FILE_EXTENT_INLINE) {
		end = btrfs_file_extent_ram_bytes(leaf, fi);
		end = ALIGN(key.offset + end, leaf->fs_info->sectorsize);
	} else {
		end = key.offset + btrfs_file_extent_num_bytes(leaf, fi);
	}

	return end;
}
