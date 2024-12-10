/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019 Google LLC
 */
#ifndef _INCFS_DATA_MGMT_H
#define _INCFS_DATA_MGMT_H

#include <linux/cred.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <linux/zstd.h>
#include <crypto/hash.h>
#include <linux/rwsem.h>

#include <uapi/linux/incrementalfs.h>

#include "internal.h"
#include "pseudo_files.h"

#define SEGMENTS_PER_FILE 3

enum LOG_RECORD_TYPE {
	FULL,
	SAME_FILE,
	SAME_FILE_CLOSE_BLOCK,
	SAME_FILE_CLOSE_BLOCK_SHORT,
	SAME_FILE_NEXT_BLOCK,
	SAME_FILE_NEXT_BLOCK_SHORT,
};

struct full_record {
	enum LOG_RECORD_TYPE type : 3; /* FULL */
	u32 block_index : 29;
	incfs_uuid_t file_id;
	u64 absolute_ts_us;
	uid_t uid;
} __packed; /* 32 bytes */

struct same_file {
	enum LOG_RECORD_TYPE type : 3; /* SAME_FILE */
	u32 block_index : 29;
	uid_t uid;
	u16 relative_ts_us; /* max 2^16 us ~= 64 ms */
} __packed; /* 10 bytes */

struct same_file_close_block {
	enum LOG_RECORD_TYPE type : 3; /* SAME_FILE_CLOSE_BLOCK */
	u16 relative_ts_us : 13; /* max 2^13 us ~= 8 ms */
	s16 block_index_delta;
} __packed; /* 4 bytes */

struct same_file_close_block_short {
	enum LOG_RECORD_TYPE type : 3; /* SAME_FILE_CLOSE_BLOCK_SHORT */
	u8 relative_ts_tens_us : 5; /* max 2^5*10 us ~= 320 us */
	s8 block_index_delta;
} __packed; /* 2 bytes */

struct same_file_next_block {
	enum LOG_RECORD_TYPE type : 3; /* SAME_FILE_NEXT_BLOCK */
	u16 relative_ts_us : 13; /* max 2^13 us ~= 8 ms */
} __packed; /* 2 bytes */

struct same_file_next_block_short {
	enum LOG_RECORD_TYPE type : 3; /* SAME_FILE_NEXT_BLOCK_SHORT */
	u8 relative_ts_tens_us : 5; /* max 2^5*10 us ~= 320 us */
} __packed; /* 1 byte */

union log_record {
	struct full_record full_record;
	struct same_file same_file;
	struct same_file_close_block same_file_close_block;
	struct same_file_close_block_short same_file_close_block_short;
	struct same_file_next_block same_file_next_block;
	struct same_file_next_block_short same_file_next_block_short;
};

struct read_log_state {
	/* Log buffer generation id, incremented on configuration changes */
	u32 generation_id;

	/* Offset in rl_ring_buf to write into. */
	u32 next_offset;

	/* Current number of writer passes over rl_ring_buf */
	u32 current_pass_no;

	/* Current full_record to diff against */
	struct full_record base_record;

	/* Current record number counting from configuration change */
	u64 current_record_no;
};

/* A ring buffer to save records about data blocks which were recently read. */
struct read_log {
	void *rl_ring_buf;

	int rl_size;

	struct read_log_state rl_head;

	struct read_log_state rl_tail;

	/* A lock to protect the above fields */
	spinlock_t rl_lock;

	/* A queue of waiters who want to be notified about reads */
	wait_queue_head_t ml_notif_wq;

	/* A work item to wake up those waiters without slowing down readers */
	struct delayed_work ml_wakeup_work;
};

struct mount_options {
	unsigned int read_timeout_ms;
	unsigned int readahead_pages;
	unsigned int read_log_pages;
	unsigned int read_log_wakeup_count;
	bool report_uid;
	char *sysfs_name;
};

struct mount_info {
	struct super_block *mi_sb;

	struct path mi_backing_dir_path;

	struct dentry *mi_index_dir;
	/* For stacking mounts, if true, this indicates if the index dir needs
	 * to be freed for this SB otherwise it was created by lower level SB */
	bool mi_index_free;

	struct dentry *mi_incomplete_dir;
	/* For stacking mounts, if true, this indicates if the incomplete dir
	 * needs to be freed for this SB. Similar to mi_index_free */
	bool mi_incomplete_free;

	const struct cred *mi_owner;

	struct mount_options mi_options;

	/* This mutex is to be taken before create, rename, delete */
	struct mutex mi_dir_struct_mutex;

	/*
	 * A queue of waiters who want to be notified about new pending reads.
	 */
	wait_queue_head_t mi_pending_reads_notif_wq;

	/*
	 * Protects - RCU safe:
	 *  - reads_list_head
	 *  - mi_pending_reads_count
	 *  - mi_last_pending_read_number
	 *  - data_file_segment.reads_list_head
	 */
	spinlock_t pending_read_lock;

	/* List of active pending_read objects */
	struct list_head mi_reads_list_head;

	/* Total number of items in reads_list_head */
	int mi_pending_reads_count;

	/*
	 * Last serial number that was assigned to a pending read.
	 * 0 means no pending reads have been seen yet.
	 */
	int mi_last_pending_read_number;

	/* Temporary buffer for read logger. */
	struct read_log mi_log;

	/* SELinux needs special xattrs on our pseudo files */
	struct mem_range pseudo_file_xattr[PSEUDO_FILE_COUNT];

	/* A queue of waiters who want to be notified about blocks_written */
	wait_queue_head_t mi_blocks_written_notif_wq;

	/* Number of blocks written since mount */
	atomic_t mi_blocks_written;

	/* Per UID read timeouts */
	spinlock_t mi_per_uid_read_timeouts_lock;
	struct incfs_per_uid_read_timeouts *mi_per_uid_read_timeouts;
	int mi_per_uid_read_timeouts_size;

	/* zstd workspace */
	struct mutex mi_zstd_workspace_mutex;
	void *mi_zstd_workspace;
	ZSTD_DStream *mi_zstd_stream;
	struct delayed_work mi_zstd_cleanup_work;

	/* sysfs node */
	struct incfs_sysfs_node *mi_sysfs_node;

	/* Last error information */
	struct mutex	mi_le_mutex;
	incfs_uuid_t	mi_le_file_id;
	u64		mi_le_time_us;
	u32		mi_le_page;
	u32		mi_le_errno;
	uid_t		mi_le_uid;

	/* Number of reads timed out */
	u32 mi_reads_failed_timed_out;

	/* Number of reads failed because hash verification failed */
	u32 mi_reads_failed_hash_verification;

	/* Number of reads failed for another reason */
	u32 mi_reads_failed_other;

	/* Number of reads delayed because page had to be fetched */
	u32 mi_reads_delayed_pending;

	/* Total time waiting for pages to be fetched */
	u64 mi_reads_delayed_pending_us;

	/*
	 * Number of reads delayed because of per-uid min_time_us or
	 * min_pending_time_us settings
	 */
	u32 mi_reads_delayed_min;

	/* Total time waiting because of per-uid min_time_us or
	 * min_pending_time_us settings.
	 *
	 * Note that if a read is initially delayed because we have to wait for
	 * the page, then further delayed because of min_pending_time_us
	 * setting, this counter gets incremented by only the further delay
	 * time.
	 */
	u64 mi_reads_delayed_min_us;
};

struct data_file_block {
	loff_t db_backing_file_data_offset;

	size_t db_stored_size;

	enum incfs_compression_alg db_comp_alg;
};

struct pending_read {
	incfs_uuid_t file_id;

	s64 timestamp_us;

	atomic_t done;

	int block_index;

	int serial_number;

	uid_t uid;

	struct list_head mi_reads_list;

	struct list_head segment_reads_list;

	struct rcu_head rcu;
};

struct data_file_segment {
	wait_queue_head_t new_data_arrival_wq;

	/* Protects reads and writes from the blockmap */
	struct rw_semaphore rwsem;

	/* List of active pending_read objects belonging to this segment */
	/* Protected by mount_info.pending_reads_mutex */
	struct list_head reads_list_head;
};

/*
 * Extra info associated with a file. Just a few bytes set by a user.
 */
struct file_attr {
	loff_t fa_value_offset;

	size_t fa_value_size;

	u32 fa_crc;
};


struct data_file {
	struct backing_file_context *df_backing_file_context;

	struct mount_info *df_mount_info;

	incfs_uuid_t df_id;

	/*
	 * Array of segments used to reduce lock contention for the file.
	 * Segment is chosen for a block depends on the block's index.
	 */
	struct data_file_segment df_segments[SEGMENTS_PER_FILE];

	/* Base offset of the first metadata record. */
	loff_t df_metadata_off;

	/* Base offset of the block map. */
	loff_t df_blockmap_off;

	/* File size in bytes */
	loff_t df_size;

	/* File header flags */
	u32 df_header_flags;

	/* File size in DATA_FILE_BLOCK_SIZE blocks */
	int df_data_block_count;

	/* Total number of blocks, data + hash */
	int df_total_block_count;

	/* For mapped files, the offset into the actual file */
	loff_t df_mapped_offset;

	/* Number of data blocks written to file */
	atomic_t df_data_blocks_written;

	/* Number of data blocks in the status block */
	u32 df_initial_data_blocks_written;

	/* Number of hash blocks written to file */
	atomic_t df_hash_blocks_written;

	/* Number of hash blocks in the status block */
	u32 df_initial_hash_blocks_written;

	/* Offset to status metadata header */
	loff_t df_status_offset;

	/*
	 * Mutex acquired while enabling verity. Note that df_hash_tree is set
	 * by enable verity.
	 *
	 * The backing file mutex bc_mutex  may be taken while this mutex is
	 * held.
	 */
	struct mutex df_enable_verity;

	/*
	 * Set either at construction time or during enabling verity. In the
	 * latter case, set via smp_store_release, so use smp_load_acquire to
	 * read it.
	 */
	struct mtree *df_hash_tree;

	/* Guaranteed set if df_hash_tree is set. */
	struct incfs_df_signature *df_signature;

	/*
	 * The verity file digest, set when verity is enabled and the file has
	 * been opened
	 */
	struct mem_range df_verity_file_digest;

	struct incfs_df_verity_signature *df_verity_signature;
};

struct dir_file {
	struct mount_info *mount_info;

	struct file *backing_dir;
};

struct inode_info {
	struct mount_info *n_mount_info; /* A mount, this file belongs to */

	struct inode *n_backing_inode;

	struct data_file *n_file;

	struct inode n_vfs_inode;
};

struct dentry_info {
	struct path backing_path;
};

enum FILL_PERMISSION {
	CANT_FILL = 0,
	CAN_FILL = 1,
};

struct incfs_file_data {
	/* Does this file handle have INCFS_IOC_FILL_BLOCKS permission */
	enum FILL_PERMISSION fd_fill_permission;

	/* If INCFS_IOC_GET_FILLED_BLOCKS has been called, where are we */
	int fd_get_block_pos;

	/* And how many filled blocks are there up to that point */
	int fd_filled_data_blocks;
	int fd_filled_hash_blocks;
};

struct mount_info *incfs_alloc_mount_info(struct super_block *sb,
					  struct mount_options *options,
					  struct path *backing_dir_path);

int incfs_realloc_mount_info(struct mount_info *mi,
			     struct mount_options *options);

void incfs_free_mount_info(struct mount_info *mi);

char *file_id_to_str(incfs_uuid_t id);
struct dentry *incfs_lookup_dentry(struct dentry *parent, const char *name);
struct data_file *incfs_open_data_file(struct mount_info *mi, struct file *bf);
void incfs_free_data_file(struct data_file *df);

struct dir_file *incfs_open_dir_file(struct mount_info *mi, struct file *bf);
void incfs_free_dir_file(struct dir_file *dir);

struct incfs_read_data_file_timeouts {
	u32 min_time_us;
	u32 min_pending_time_us;
	u32 max_pending_time_us;
};

ssize_t incfs_read_data_file_block(struct mem_range dst, struct file *f,
			int index, struct mem_range tmp,
			struct incfs_read_data_file_timeouts *timeouts,
			unsigned int *delayed_min_us);

ssize_t incfs_read_merkle_tree_blocks(struct mem_range dst,
				      struct data_file *df, size_t offset);

int incfs_get_filled_blocks(struct data_file *df,
			    struct incfs_file_data *fd,
			    struct incfs_get_filled_blocks_args *arg);

int incfs_read_file_signature(struct data_file *df, struct mem_range dst);

int incfs_process_new_data_block(struct data_file *df,
				 struct incfs_fill_block *block, u8 *data,
				 bool *complete);

int incfs_process_new_hash_block(struct data_file *df,
				 struct incfs_fill_block *block, u8 *data);

bool incfs_fresh_pending_reads_exist(struct mount_info *mi, int last_number);

/*
 * Collects pending reads and saves them into the array (reads/reads_size).
 * Only reads with serial_number > sn_lowerbound are reported.
 * Returns how many reads were saved into the array.
 */
int incfs_collect_pending_reads(struct mount_info *mi, int sn_lowerbound,
				struct incfs_pending_read_info *reads,
				struct incfs_pending_read_info2 *reads2,
				int reads_size, int *new_max_sn);

int incfs_collect_logged_reads(struct mount_info *mi,
			       struct read_log_state *start_state,
			       struct incfs_pending_read_info *reads,
			       struct incfs_pending_read_info2 *reads2,
			       int reads_size);
struct read_log_state incfs_get_log_state(struct mount_info *mi);
int incfs_get_uncollected_logs_count(struct mount_info *mi,
				     const struct read_log_state *state);

static inline struct inode_info *get_incfs_node(struct inode *inode)
{
	if (!inode)
		return NULL;

	if (inode->i_sb->s_magic != INCFS_MAGIC_NUMBER) {
		/* This inode doesn't belong to us. */
		pr_warn_once("incfs: %s on an alien inode.", __func__);
		return NULL;
	}

	return container_of(inode, struct inode_info, n_vfs_inode);
}

static inline struct data_file *get_incfs_data_file(struct file *f)
{
	struct inode_info *node = NULL;

	if (!f)
		return NULL;

	if (!S_ISREG(f->f_inode->i_mode))
		return NULL;

	node = get_incfs_node(f->f_inode);
	if (!node)
		return NULL;

	return node->n_file;
}

static inline struct dir_file *get_incfs_dir_file(struct file *f)
{
	if (!f)
		return NULL;

	if (!S_ISDIR(f->f_inode->i_mode))
		return NULL;

	return (struct dir_file *)f->private_data;
}

/*
 * Make sure that inode_info.n_file is initialized and inode can be used
 * for reading and writing data from/to the backing file.
 */
int make_inode_ready_for_data_ops(struct mount_info *mi,
				struct inode *inode,
				struct file *backing_file);

static inline struct dentry_info *get_incfs_dentry(const struct dentry *d)
{
	if (!d)
		return NULL;

	return (struct dentry_info *)d->d_fsdata;
}

static inline void get_incfs_backing_path(const struct dentry *d,
					  struct path *path)
{
	struct dentry_info *di = get_incfs_dentry(d);

	if (!di) {
		*path = (struct path) {};
		return;
	}

	*path = di->backing_path;
	path_get(path);
}

static inline int get_blocks_count_for_size(u64 size)
{
	if (size == 0)
		return 0;
	return 1 + (size - 1) / INCFS_DATA_FILE_BLOCK_SIZE;
}

#endif /* _INCFS_DATA_MGMT_H */
