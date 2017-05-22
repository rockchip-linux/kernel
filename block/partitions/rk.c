#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/bootmem.h>
#include "check.h"
#include "rk.h"

/* rkpart_setup() parses into here */
static struct cmdline_rk_partition *partitions;
/* the command line passed to mtdpart_setupd() */
static char *cmdline;
static int cmdline_parsed;

/*
 * Parse one partition definition for an rkpart. Since there can be many
 * comma separated partition definitions, this function calls itself
 * recursively until no more partition definitions are found. Nice side
 * effect: the memory to keep the rk_partition structs and the names
 * is allocated upon the last definition being found. At that point the
 * syntax has been verified ok.
 */
static struct rk_partition *newpart(char *s,
                                        char **retptr,
                                        int *num_parts,
                                        int this_part,
                                        unsigned char **extra_mem_ptr,
                                        int extra_mem_size)
{
	struct rk_partition *parts;
	sector_t size;
	sector_t from = OFFSET_CONTINUOUS;
	char *name;
	int name_len;
	unsigned char *extra_mem;
	char delim;

	/* fetch the partition size */
	if (*s == '-')
	{	/* assign all remaining space to this partition */
		size = SIZE_REMAINING;
		s++;
	}
	else
	{
		size = memparse(s, &s);
		/* No sense support partition less than 8B */
		if (size < ((PAGE_SIZE) >> 9))
		{
			printk(KERN_ERR ERRP "partition size too small (%llx)\n", (u64)size);
			return NULL;
		}
	}

	/* fetch partition name */
	delim = 0;
        /* check for from */
        if (*s == '@')
	{
                s++;
                from = memparse(s, &s);
        }
        /* now look for name */
	if (*s == '(')
	{
		delim = ')';
	}

	if (delim)
	{
		char *p;

	    	name = ++s;
		p = strchr(name, delim);
		if (!p)
		{
			printk(KERN_ERR ERRP "no closing %c found in partition name\n", delim);
			return NULL;
		}
		name_len = p - name;
		s = p + 1;
	}
	else
	{
	    	name = NULL;
		name_len = 13; /* Partition_000 */
	}

	/* record name length for memory allocation later */
	extra_mem_size += name_len + 1;

	/* test if more partitions are following */
	if (*s == ',')
	{
		if (size == SIZE_REMAINING)
		{
			printk(KERN_ERR ERRP "no partitions allowed after a fill-up partition\n");
			return NULL;
		}
		/* more partitions follow, parse them */
		parts = newpart(s + 1, &s, num_parts, this_part + 1,
				&extra_mem, extra_mem_size);
		if (!parts)
			return NULL;
	}
	else
	{	/* this is the last partition: allocate space for all */
		int alloc_size;

		*num_parts = this_part + 1;
		alloc_size = *num_parts * sizeof(struct rk_partition) +
			     extra_mem_size;
		parts = kzalloc(alloc_size, GFP_KERNEL);
		if (!parts)
		{
			printk(KERN_ERR ERRP "out of memory\n");
			return NULL;
		}
		extra_mem = (unsigned char *)(parts + *num_parts);
	}
	/* enter this partition (from will be calculated later if it is zero at this point) */
	parts[this_part].size = size;
	parts[this_part].from = from;
	if (name)
	{
		strlcpy(extra_mem, name, name_len + 1);
	}
	else
	{
		sprintf(extra_mem, "Partition_%03d", this_part);
	}
	parts[this_part].name = extra_mem;
	extra_mem += name_len + 1;

	dbg(("partition %d: name <%s>, from %llx, size %llx\n",
	     this_part,
	     parts[this_part].name,
	     parts[this_part].from,
	     parts[this_part].size));

	/* return (updated) pointer to extra_mem memory */
	if (extra_mem_ptr)
	  *extra_mem_ptr = extra_mem;

	/* return (updated) pointer command line string */
	*retptr = s;

	/* return partition table */
	return parts;
}

/*
 * Parse the command line.
 */
static int rkpart_setup_real(char *s)
{
	cmdline_parsed = 1;

	for( ; s != NULL; )
	{
		struct cmdline_rk_partition *this_rk;
		struct rk_partition *parts;
	    	int rk_id_len;
		int num_parts;
		char *p, *rk_id;

	    	rk_id = s;
		/* fetch <rk-id> */
		if (!(p = strchr(s, ':')))
		{
			dbg(( "no rk-id\n"));
			return 0;
		}
		rk_id_len = p - rk_id;

		dbg(("parsing <%s>\n", p + 1));

		/*
		 * parse one mtd. have it reserve memory for the
		 * struct cmdline_mtd_partition and the mtd-id string.
		 */
		parts = newpart(p + 1,		/* cmdline */
				&s,		/* out: updated cmdline ptr */
				&num_parts,	/* out: number of parts */
				0,		/* first partition */
				(unsigned char**)&this_rk, /* out: extra mem */
				rk_id_len + 1 + sizeof(*this_rk) +
				sizeof(void*)-1 /*alignment*/);
		if(!parts)
		{
			/*
			 * An error occurred. We're either:
			 * a) out of memory, or
			 * b) in the middle of the partition spec
			 * Either way, this mtd is hosed and we're
			 * unlikely to succeed in parsing any more
			 */
			 return 0;
		 }

		/* align this_rk */
		this_rk = (struct cmdline_rk_partition *)
			ALIGN((unsigned long)this_rk, sizeof(void*));
		/* enter results */
		this_rk->parts = parts;
		this_rk->num_parts = num_parts;
		this_rk->rk_id = (char*)(this_rk + 1);
		strlcpy(this_rk->rk_id, rk_id, rk_id_len + 1);

		/* link into chain */
		this_rk->next = partitions;
		partitions = this_rk;

		dbg(("rkid=<%s> num_parts=<%d>\n",
		     this_rk->mtd_id, this_rk->num_parts));

		/* EOS - we're done */
		if (*s == 0)
			break;
		s++;
	}
	return 1;
}

/*
 * Main function to be called from the MTD mapping driver/device to
 * obtain the partitioning information. At this point the command line
 * arguments will actually be parsed and turned to struct mtd_partition
 * information. It returns partitions for the requested mtd device, or
 * the first one in the chain if a NULL mtd_id is passed in.
 */
static int parse_cmdline_partitions(sector_t n,
                             	    struct rk_partition **pparts,
                             	    unsigned long origin)
{
	unsigned long from;
	int i;
	struct cmdline_rk_partition *part;
	/* Fixme: parameter should be coherence with part table id */
	const char *rk_id = "rk29xxnand";

	/* parse command line */
	if (!cmdline_parsed)
		rkpart_setup_real(cmdline);

	for(part = partitions; part; part = part->next)
	{
		if ((!rk_id) || (!strcmp(part->rk_id, rk_id)))
		{
			for(i = 0, from = 0; i < part->num_parts; i++)
			{
				if (part->parts[i].from == OFFSET_CONTINUOUS)
				  part->parts[i].from = from;
				else
				  from = part->parts[i].from;
				if (part->parts[i].size == SIZE_REMAINING)
				  part->parts[i].size = n - from - FROM_OFFSET;
				if (from + part->parts[i].size > n)
				{
					printk(KERN_WARNING ERRP
					       "%s: partitioning exceeds flash size, truncating\n",
					       part->rk_id);
					part->parts[i].size = n - from;
					part->num_parts = i;
				}
				from += part->parts[i].size;
			}
			*pparts = kmemdup(part->parts,
					sizeof(*part->parts) * part->num_parts,
					GFP_KERNEL);
			if (!*pparts)
				return -ENOMEM;
			return part->num_parts;
		}
	}
	return 0;
}

static void rkpart_bootmode_fixup(void)
{
	const char mode[] = " androidboot.mode=emmc";
	const char charger[] = " androidboot.charger.emmc=1";
	char *new_command_line;
	size_t saved_command_line_len = strlen(saved_command_line);

	if (strstr(saved_command_line, "androidboot.mode=charger")) {
		new_command_line = kzalloc(saved_command_line_len + strlen(charger) + 1, GFP_KERNEL);
		sprintf(new_command_line, "%s%s", saved_command_line, charger);
	} else {
		new_command_line = kzalloc(saved_command_line_len + strlen(mode) + 1, GFP_KERNEL);
		sprintf(new_command_line, "%s%s", saved_command_line, mode);
	}
	saved_command_line = new_command_line;
}

/*
 * read_lba(): Read bytes from disk, starting at given LBA
 * @state
 * @lba
 * @buffer
 * @size_t
 *
 * Description: Reads @count bytes from @state->bdev into @buffer.
 * Returns number of bytes read on success, 0 on error.
 */
static size_t read_lba(struct parsed_partitions *state,
		       u64 lba, u8 *buffer, size_t count)
{
	size_t totalreadcount = 0;
	struct block_device *bdev = state->bdev;
	sector_t n = lba * (bdev_logical_block_size(bdev) / 512);

	while (count) {
		int copied = 512;
		Sector sect;
		unsigned char *data = read_part_sector(state, n++, &sect);

		if (!data)
			break;
		if (copied > count)
			copied = count;
		memcpy(buffer, data, copied);
		put_dev_sector(sect);
		buffer += copied;
		totalreadcount += copied;
		count -= copied;
	}
	return totalreadcount;
}

static int rk_new_part_dectet(sector_t n, struct parsed_partitions *state)
{
	int i, part_num = 0;
	struct rk_part_info *g_part; /* size 2KB */
	struct rk_part_entey *parts;
	int size;

	BUILD_BUG_ON_MSG(sizeof(struct rk_part_info) != 4096,
			 "struct rk_part_info size error");

	size = sizeof(*g_part);
	g_part = kmalloc(size, GFP_KERNEL | GFP_DMA);
	if (!g_part)
		return 0;

	if (read_lba(state, 0, (u8 *)g_part, size) < size) {
		kfree(g_part);
		return 0;
	}

	if (g_part->hdr.ui_fw_tag == RK_PARTITION_TAG) {
		part_num = g_part->hdr.ui_part_entry_count;
		parts = g_part->part;

		for (i = 0; i < part_num; i++) {
			struct partition_meta_info *info;
			unsigned int label_max;

			if (parts[i].size == SIZE_REMAINING)
				parts[i].size = n - parts[i].offset;

			info = &state->parts[i + 1].info;
			put_partition(state, i + 1,
				      parts[i].offset,
				      parts[i].size);
			label_max = min(sizeof(info->volname) - 1,
					sizeof(parts[i].name));
			strncpy(info->volname, parts[i].name, label_max);
			pr_info("%10s: 0x%09llx -- 0x%09llx (%llu MB)\n",
				parts[i].name,
				(u64)parts[i].offset * 512,
				(u64)(parts[i].offset + parts[i].size) * 512,
				(u64)parts[i].size / 2048);
		}
	}

	kfree(g_part);
	return (part_num > 0);
}

int rkpart_partition(struct parsed_partitions *state)
{
	int num_parts = 0, i;
	sector_t n = get_capacity(state->bdev->bd_disk);
	struct rk_partition *parts = NULL;

	if (n < SECTOR_1G)
		return 0;

	/* ONLY be used by eMMC-disk */
	if (1 != state->bdev->bd_disk->emmc_disk)
		return 0;

	/* Fixme: parameter should be coherence with part table */
	cmdline = strstr(saved_command_line, "mtdparts=") + 9;
	cmdline_parsed = 0;

	num_parts = parse_cmdline_partitions(n, &parts, 0);
	if (num_parts < 0)
		return num_parts;

	for (i = 0; i < num_parts; i++) {
		struct partition_meta_info *info;
		unsigned int label_max;

		info = &state->parts[i + 1].info;
		put_partition(state,
			      i + 1,
			      parts[i].from + FROM_OFFSET,
			      parts[i].size);
		label_max = sizeof(info->volname) - 1;
		strncpy(info->volname, parts[i].name, label_max);
		pr_info("%10s: 0x%09llx -- 0x%09llx (%llu MB)\n",
			parts[i].name,
			(u64)parts[i].from * 512,
			(u64)(parts[i].from + parts[i].size) * 512,
			(u64)parts[i].size / 2048);
	}

	rkpart_bootmode_fixup();

	return 1;
}

int rkpart_new_partition(struct parsed_partitions *state)
{
	sector_t n = get_capacity(state->bdev->bd_disk);

	if (n < SECTOR_1G)
		return 0;

	/* ONLY be used by eMMC-disk */
	if (state->bdev->bd_disk->emmc_disk != 1)
		return 0;

	if (rk_new_part_dectet(n, state) == 1)
		return 1;
	return 0;
}
