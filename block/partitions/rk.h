/*
 *  block/partitions/rk.h
 */

/* error message prefix */
#define ERRP "rkpart: "

/* debug macro */
#define RKPART_DEBUG 0
#if RKPART_DEBUG
#define dbg(x)  do {    \
printk("DEBUG-CMDLINE-PART: ");	\
printk x;       \
} while (0)
#else
#define dbg(x)
#endif

/* At least 1GB disk support*/
#define SECTOR_1G	0x200000

/* Default partition table offet got from loader: 4MB*/
#define FROM_OFFSET	0x2000

/* special size referring to all the remaining space in a partition */
#define SIZE_REMAINING UINT_MAX
#define OFFSET_CONTINUOUS UINT_MAX

struct rk_partition {
	char *name;
	sector_t from;
	sector_t size;
};
struct cmdline_rk_partition {
	struct cmdline_rk_partition *next;
	char *rk_id;
	int num_parts;
	struct rk_partition *parts;
};

#define RK_PARTITION_TAG 0x50464B52

struct data_time {
	unsigned short year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char min;
	unsigned char sec;
	unsigned char reserve;
} __packed;

enum e_partition_type {
	PART_VENDOR = 1 << 0,
	PART_IDBLOCK = 1 << 1,
	PART_KERNEL = 1 << 2,
	PART_BOOT = 1 << 3,
	PART_USER = 1 << 31
};

struct rk_fw_header {
	unsigned int ui_fw_tag;	/* "RKFP" */
	struct data_time t_release_data_time;
	unsigned int ui_fw_ver;
	unsigned int ui_size;	/* size of sturct,unit of u8 */
	unsigned int ui_part_entry_offset;	/* unit of sector */
	unsigned int ui_backup_part_entry_offset;
	unsigned int ui_part_entry_size;	/* unit of u8 */
	unsigned int ui_part_entry_count;
	unsigned int ui_fw_size;	/* unit of u8 */
	unsigned char reserved[464];
	unsigned int ui_part_entry_crc;
	unsigned int ui_header_crc;
};

struct rk_part_entey {
	unsigned char name[32];
	enum e_partition_type em_part_type;
	unsigned int offset;	/* unit of sector */
	unsigned int size;	/* unit of sector */
	unsigned int data_length;	/* unit of u8 */
	unsigned int part_property;
	unsigned char reserved[76];
};

struct rk_part_info {
	struct rk_fw_header hdr;	/* 0.5KB */
	struct rk_part_entey part[28];	/* 3.5KB */
} __packed;

int rkpart_partition(struct parsed_partitions *state);
int rkpart_new_partition(struct parsed_partitions *state);
