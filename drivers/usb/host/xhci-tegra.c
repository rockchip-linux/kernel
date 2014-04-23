/*
 * xhci-tegra.c - Nvidia xHCI host controller driver
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/circ_buf.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/firmware.h>
#include <linux/of_device.h>
#include <linux/platform_data/tegra_emc.h>
#include <linux/regulator/consumer.h>
#include <linux/tegra-powergate.h>
#include <linux/tegra-soc.h>
#include <linux/uaccess.h>
#include <linux/usb/phy.h>
#include <linux/usb/tegra_xusb_phy.h>
#include <linux/usb/tegra_xusb.h>

#include "xhci-tegra.h"
#include "xhci.h"

/* macros */
#define FW_IOCTL_LOG_DEQUEUE_LOW	4
#define FW_IOCTL_LOG_DEQUEUE_HIGH	5
#define FW_IOCTL_DATA_SHIFT		0
#define FW_IOCTL_DATA_MASK		0x00ffffff
#define FW_IOCTL_TYPE_SHIFT		24
#define FW_IOCTL_TYPE_MASK		0xff000000
#define FW_LOG_SIZE			sizeof(struct log_entry)
#define FW_LOG_COUNT			4096
#define FW_LOG_RING_SIZE		(FW_LOG_SIZE * FW_LOG_COUNT)
#define FW_LOG_PAYLOAD_SIZE		27
#define FW_LOG_OWNER_DRIVER		0x01
#define FW_LOG_CIRC_BUF_SIZE		(4 * (1 << 20)) /* 4MB */
#define FW_LOG_THREAD_RELAX		msecs_to_jiffies(100)

/* tegra_xhci_firmware_log.flags bits */
#define FW_LOG_CONTEXT_VALID		0
#define FW_LOG_FILE_OPENED		1

#define PAGE_SELECT_MASK		0xFFFFFE00
#define PAGE_SELECT_SHIFT		9
#define PAGE_OFFSET_MASK		0x000001FF
#define CSB_PAGE_SELECT(_addr)						\
	({								\
		typecheck(u32, _addr);					\
		((_addr & PAGE_SELECT_MASK) >> PAGE_SELECT_SHIFT);	\
	})
#define CSB_PAGE_OFFSET(_addr)						\
	({								\
		typecheck(u32, _addr);					\
		(_addr & PAGE_OFFSET_MASK);				\
	})

#define reg_dump(_dev, _base, _reg)					\
	dev_dbg(_dev, "%s: %s @%x = 0x%x\n", __func__, #_reg,		\
		_reg, readl(_base + _reg))

/* Usb3 Firmware Cfg Table */
struct cfgtbl {
	u32 boot_loadaddr_in_imem;
	u32 boot_codedfi_offset;
	u32 boot_codetag;
	u32 boot_codesize;

	/* Physical memory reserved by Bootloader/BIOS */
	u32 phys_memaddr;
	u16 reqphys_memsize;
	u16 alloc_phys_memsize;

	/* .rodata section */
	u32 rodata_img_offset;
	u32 rodata_section_start;
	u32 rodata_section_end;
	u32 main_fnaddr;

	u32 fwimg_cksum;
	u32 fwimg_created_time;

	/*
	 * Fields that get filled by linker during linking phase
	 * or initialized in the FW code.
	 */
	u32 imem_resident_start;
	u32 imem_resident_end;
	u32 idirect_start;
	u32 idirect_end;
	u32 l2_imem_start;
	u32 l2_imem_end;
	u32 version_id;
	u8 init_ddirect;
	u8 reserved[3];
	u32 phys_addr_log_buffer;
	u32 total_log_entries;
	u32 dequeue_ptr;

	/*
	 * Below two dummy variables are used to replace
	 * L2IMemSymTabOffsetInDFI and L2IMemSymTabSize in order to retain
	 * the size of struct _CFG_TBL used by other AP/Module.
	 */
	u32 dummy_var1;
	u32 dummy_var2;

	/* fwimg_len */
	u32 fwimg_len;
	u8 magic[8];
	u32 ss_low_power_entry_timeout;
	u8 num_hsic_port;
	u8 padding[139]; /* padding bytes to makeup 256-bytes cfgtbl */
};

struct xusb_save_regs {
	u32 msi_bar_sz;
	u32 msi_axi_barst;
	u32 msi_fpci_barst;
	u32 msi_vec0;
	u32 msi_en_vec0;
	u32 fpci_error_masks;
	u32 intr_mask;
	u32 ipfs_intr_enable;
	u32 ufpci_config;
	u32 clkgate_hysteresis;
	u32 xusb_host_mccif_fifo_cntrl;
	/* PG does not mention below */
	u32 hs_pls;
	u32 fs_pls;
	u32 hs_fs_speed;
	u32 hs_fs_pp;
	u32 cfg_aru;
	u32 cfg_order;
	u32 cfg_fladj;
	u32 cfg_sid;
};

struct tegra_xhci_firmware {
	void *data; /* kernel virtual address */
	size_t size; /* firmware size */
	dma_addr_t dma; /* dma address for controller */
};

struct log_entry {
	u32 sequence_no;
	u8 data[FW_LOG_PAYLOAD_SIZE];
	u8 owner;
};

struct tegra_xhci_firmware_log {
	dma_addr_t phys_addr;		/* dma-able address */
	void *virt_addr;		/* kernel va of the shared log buffer*/
	struct log_entry *dequeue;	/* current dequeue pointer (va) */
	struct circ_buf circ;		/* big circular buffer */
	u32 seq;			/* log sequence number */

	struct task_struct *thread;	/* a thread to consume log */
	struct mutex mutex;
	wait_queue_head_t read_wait;
	wait_queue_head_t write_wait;
	wait_queue_head_t intr_wait;
	struct dentry *path;
	struct dentry *log_file;
	unsigned long flags;
};

struct tegra_xusb_soc_config {
	const char *firmware_file;
	bool use_hs_src_clk2;
};

struct tegra_xhci_hcd {
	struct platform_device *pdev;
	struct platform_device *xhci_pdev;
	u16 device_id;

	struct mutex sync_lock;

	int mbox_irq;
	struct mutex mbox_lock;
	struct raw_notifier_head mbox_notifiers;
	struct notifier_block mbox_nb;

	bool hc_in_elpg;
	phys_addr_t host_phys_base;
	void __iomem *fpci_base;
	void __iomem *ipfs_base;

	const struct tegra_xusb_soc_config *soc_config;

	struct regulator *xusb_s1p05v_reg;
	struct regulator *xusb_s3p3v_reg;
	struct regulator *xusb_s1p8v_reg;

	/* XUSB host clock */
	struct clk *host_clk;
	/* XUSB Falcon SuperSpeed clock */
	struct clk *falc_clk;
	/* EMC clock */
	struct clk *emc_clk;
	/* refPLLE clock */
	struct clk *pll_re_vco_clk;

	/* XUSB PHY */
	struct usb_phy *phy;
	struct notifier_block phy_nb;

	/*
	 * XUSB/IPFS specific registers these need to be saved/restored in
	 * addition to spec defined registers
	 */
	struct xusb_save_regs sregs;

	/* Firmware loading related */
	struct tegra_xhci_firmware firmware;

#ifdef CONFIG_USB_XHCI_TEGRA_FW_LOG
	struct tegra_xhci_firmware_log log;
#endif

	bool init_done;
	struct notifier_block bus_nb;
};

static inline struct usb_hcd *tegra_to_hcd(struct tegra_xhci_hcd *tegra)
{
	return dev_get_drvdata(&tegra->xhci_pdev->dev);
}

static inline struct xhci_hcd *tegra_to_xhci(struct tegra_xhci_hcd *tegra)
{
	return hcd_to_xhci(tegra_to_hcd(tegra));
}

static int tegra_xhci_probe2(struct tegra_xhci_hcd *tegra);
static int tegra_xhci_mbox_send(struct tegra_xhci_hcd *tegra,
				enum tegra_xusb_mbox_cmd cmd, u32 data);

static u32 tegra_xhci_read_portsc(struct tegra_xhci_hcd *tegra,
				  unsigned int port)
{
	struct xhci_hcd *xhci = tegra_to_xhci(tegra);

	return readl(&xhci->op_regs->port_status_base +
		     NUM_PORT_REGS * port);
}

enum usb_device_speed tegra_xhci_port_speed(struct tegra_xhci_hcd *tegra,
					    unsigned int port)
{
	u32 portsc = tegra_xhci_read_portsc(tegra, port);

	if (DEV_FULLSPEED(portsc))
		return USB_SPEED_FULL;
	else if (DEV_HIGHSPEED(portsc))
		return USB_SPEED_HIGH;
	else if (DEV_LOWSPEED(portsc))
		return USB_SPEED_LOW;
	else if (DEV_SUPERSPEED(portsc))
		return USB_SPEED_SUPER;
	else
		return USB_SPEED_UNKNOWN;
}
EXPORT_SYMBOL(tegra_xhci_port_speed);

bool tegra_xhci_port_connected(struct tegra_xhci_hcd *tegra, unsigned int port)
{
	u32 portsc = tegra_xhci_read_portsc(tegra, port);

	return portsc & PORT_CONNECT;
}
EXPORT_SYMBOL(tegra_xhci_port_connected);

bool tegra_xhci_port_may_wakeup(struct tegra_xhci_hcd *tegra, unsigned int port)
{
	struct usb_hcd *hcd = tegra_to_hcd(tegra);
	struct usb_device *rhdev = hcd_to_bus(hcd)->root_hub;

	/* Note: this only applies to ports on the USB2.0 root hub. */
	return usb_port_may_wakeup(rhdev, port + 1);
}
EXPORT_SYMBOL(tegra_xhci_port_may_wakeup);

static u32 csb_read(struct tegra_xhci_hcd *tegra, u32 addr)
{
	struct device *dev = &tegra->pdev->dev;
	void __iomem *fpci_base = tegra->fpci_base;
	u32 input_addr;
	u32 data;
	u32 csb_page_select;

	/* to select the appropriate CSB page to write to */
	csb_page_select = CSB_PAGE_SELECT(addr);

	dev_dbg(dev, "csb_read: csb_page_select= 0x%08x\n", csb_page_select);

	writel(csb_page_select, fpci_base + XUSB_CFG_ARU_C11_CSBRANGE);

	/* selects the appropriate offset in the page to read from */
	input_addr = CSB_PAGE_OFFSET(addr);
	data = readl(fpci_base + XUSB_CFG_CSB_BASE_ADDR + input_addr);

	dev_dbg(dev, "csb_read: input_addr = 0x%08x data = 0x%08x\n",
		input_addr, data);

	return data;
}

static void csb_write(struct tegra_xhci_hcd *tegra, u32 addr, u32 data)
{
	struct device *dev = &tegra->pdev->dev;
	void __iomem *fpci_base = tegra->fpci_base;
	u32 input_addr;
	u32 csb_page_select;

	/* to select the appropriate CSB page to write to */
	csb_page_select = CSB_PAGE_SELECT(addr);

	dev_dbg(dev, "csb_write:csb_page_selectx = 0x%08x\n", csb_page_select);

	writel(csb_page_select, fpci_base + XUSB_CFG_ARU_C11_CSBRANGE);

	/* selects the appropriate offset in the page to write to */
	input_addr = CSB_PAGE_OFFSET(addr);
	writel(data, fpci_base + XUSB_CFG_CSB_BASE_ADDR + input_addr);

	dev_dbg(dev, "csb_write: input_addr = 0x%08x data = %0x08x\n",
		input_addr, data);
}

#ifdef CONFIG_USB_XHCI_TEGRA_FW_LOG
/**
 * fw_log_next - find next log entry in a tegra_xhci_firmware_log context.
 *	This function takes care of wrapping. That means when current log entry
 *	is the last one, it returns with the first one.
 *
 * @param log	The tegra_xhci_firmware_log context.
 * @param this	The current log entry.
 * @return	The log entry which is next to the current one.
 */
static inline struct log_entry *
fw_log_next(struct tegra_xhci_firmware_log *log, struct log_entry *this)
{
	struct log_entry *first = (struct log_entry *) log->virt_addr;
	struct log_entry *last = first + FW_LOG_COUNT - 1;

	WARN((this < first) || (this > last), "%s: invalid input\n", __func__);

	return (this == last) ? first : (this + 1);
}

/**
 * fw_log_update_dequeue_pointer - update dequeue pointer to both firmware and
 *	tegra_xhci_firmware_log.dequeue.
 *
 * @param log	The tegra_xhci_firmware_log context.
 * @param n	Counts of log entries to fast-forward.
 */
static inline void
fw_log_update_deq_pointer(struct tegra_xhci_firmware_log *log, int n)
{
	struct tegra_xhci_hcd *tegra =
			container_of(log, struct tegra_xhci_hcd, log);
	struct device *dev = &tegra->pdev->dev;
	struct log_entry *deq = tegra->log.dequeue;
	dma_addr_t physical_addr;
	u32 reg;

	dev_dbg(dev, "curr 0x%p fast-forward %d entries\n", deq, n);
	while (n-- > 0)
		deq = fw_log_next(log, deq);

	tegra->log.dequeue = deq;
	physical_addr = tegra->log.phys_addr +
			((u8 *)deq - (u8 *)tegra->log.virt_addr);

	/* update dequeue pointer to firmware */
	reg = (FW_IOCTL_LOG_DEQUEUE_LOW << FW_IOCTL_TYPE_SHIFT);
	reg |= (physical_addr & 0xffff); /* lower 16-bits */
	writel(reg, tegra->fpci_base + XUSB_CFG_ARU_FW_SCRATCH);

	reg = (FW_IOCTL_LOG_DEQUEUE_HIGH << FW_IOCTL_TYPE_SHIFT);
	reg |= ((physical_addr >> 16) & 0xffff); /* higher 16-bits */
	writel(reg, tegra->fpci_base + XUSB_CFG_ARU_FW_SCRATCH);

	dev_dbg(dev, "new 0x%p physical addr 0x%x\n", deq, (u32)physical_addr);
}

static inline bool circ_buffer_full(struct circ_buf *circ)
{
	int space = CIRC_SPACE(circ->head, circ->tail, FW_LOG_CIRC_BUF_SIZE);

	return (space <= FW_LOG_SIZE);
}

static inline bool fw_log_available(struct tegra_xhci_hcd *tegra)
{
	return (tegra->log.dequeue->owner == FW_LOG_OWNER_DRIVER);
}

/**
 * fw_log_wait_empty_timeout - wait firmware log thread to clean up shared
 *	log buffer.
 * @param tegra:	tegra_xhci_hcd context
 * @param msec:		timeout value in millisecond
 * @return true:	shared log buffer is empty,
 *	   false:	shared log buffer isn't empty.
 */
static inline bool
fw_log_wait_empty_timeout(struct tegra_xhci_hcd *tegra, unsigned timeout)
{
	unsigned long target = jiffies + msecs_to_jiffies(timeout);
	bool ret;

	mutex_lock(&tegra->log.mutex);

	while (fw_log_available(tegra) && time_is_after_jiffies(target)) {
		mutex_unlock(&tegra->log.mutex);
		usleep_range(1000, 2000);
		mutex_lock(&tegra->log.mutex);
	}

	ret = fw_log_available(tegra);
	mutex_unlock(&tegra->log.mutex);

	return ret;
}

/**
 * fw_log_copy - copy firmware log from device's buffer to driver's circular
 *	buffer.
 * @param tegra	tegra_xhci_hcd context
 * @return true,	We still have firmware log in device's buffer to copy.
 *			This function returned due the driver's circular buffer
 *			is full. Caller should invoke this function again as
 *			soon as there is space in driver's circular buffer.
 *	   false,	Device's buffer is empty.
 */
static inline bool fw_log_copy(struct tegra_xhci_hcd *tegra)
{
	struct device *dev = &tegra->pdev->dev;
	struct circ_buf *circ = &tegra->log.circ;
	int head, tail;
	int buffer_len, copy_len;
	struct log_entry *entry;
	struct log_entry *first = tegra->log.virt_addr;

	while (fw_log_available(tegra)) {
		/* calculate maximum contiguous driver buffer length */
		head = circ->head;
		tail = ACCESS_ONCE(circ->tail);
		buffer_len = CIRC_SPACE_TO_END(head, tail,
			FW_LOG_CIRC_BUF_SIZE);
		/* round down to FW_LOG_SIZE */
		buffer_len -= (buffer_len % FW_LOG_SIZE);
		if (!buffer_len)
			return true; /* log available but no space left */

		/* calculate maximum contiguous log copy length */
		entry = tegra->log.dequeue;
		copy_len = 0;
		do {
			if (tegra->log.seq != entry->sequence_no) {
				dev_warn(dev,
				"%s: discontinuous seq no, expect %u get %u\n",
				__func__, tegra->log.seq, entry->sequence_no);
			}
			tegra->log.seq = entry->sequence_no + 1;

			copy_len += FW_LOG_SIZE;
			buffer_len -= FW_LOG_SIZE;
			if (!buffer_len)
				break; /* no space left */
			entry = fw_log_next(&tegra->log, entry);
		} while ((entry->owner == FW_LOG_OWNER_DRIVER) &&
			 (entry != first));

		memcpy(&circ->buf[head], tegra->log.dequeue, copy_len);
		memset(tegra->log.dequeue, 0, copy_len);
		circ->head = (circ->head + copy_len) &
			(FW_LOG_CIRC_BUF_SIZE - 1);

		mb();

		fw_log_update_deq_pointer(&tegra->log, copy_len/FW_LOG_SIZE);

		dev_dbg(dev, "copied %d entries, new dequeue 0x%p\n",
				copy_len/FW_LOG_SIZE, tegra->log.dequeue);
		wake_up_interruptible(&tegra->log.read_wait);
	}

	return false;
}

static int fw_log_thread(void *data)
{
	struct tegra_xhci_hcd *tegra = data;
	struct device *dev = &tegra->pdev->dev;
	struct circ_buf *circ = &tegra->log.circ;
	bool logs_left;

	dev_dbg(dev, "start firmware log thread\n");

	do {
		mutex_lock(&tegra->log.mutex);
		if (circ_buffer_full(circ)) {
			mutex_unlock(&tegra->log.mutex);
			dev_info(dev, "%s: circ buffer full\n", __func__);
			wait_event_interruptible(tegra->log.write_wait,
			    kthread_should_stop() || !circ_buffer_full(circ));
			mutex_lock(&tegra->log.mutex);
		}

		logs_left = fw_log_copy(tegra);
		mutex_unlock(&tegra->log.mutex);

		/* relax if no logs left  */
		if (!logs_left)
			wait_event_interruptible_timeout(tegra->log.intr_wait,
				fw_log_available(tegra), FW_LOG_THREAD_RELAX);
	} while (!kthread_should_stop());

	dev_dbg(dev, "stop firmware log thread\n");

	return 0;
}

static inline bool circ_buffer_empty(struct circ_buf *circ)
{
	return (CIRC_CNT(circ->head, circ->tail, FW_LOG_CIRC_BUF_SIZE) == 0);
}

static ssize_t fw_log_file_read(struct file *file, char __user *buf,
				size_t count, loff_t *offp)
{
	struct tegra_xhci_hcd *tegra = file->private_data;
	struct circ_buf *circ = &tegra->log.circ;
	struct device *dev = &tegra->pdev->dev;
	int head, tail;
	size_t n = 0;
	int s;

	mutex_lock(&tegra->log.mutex);

	while (circ_buffer_empty(circ)) {
		mutex_unlock(&tegra->log.mutex);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN; /* non-blocking read */

		dev_dbg(dev, "%s: nothing to read\n", __func__);

		if (wait_event_interruptible(tegra->log.read_wait,
				!circ_buffer_empty(circ)))
			return -ERESTARTSYS;

		if (mutex_lock_interruptible(&tegra->log.mutex))
			return -ERESTARTSYS;
	}

	while (count > 0) {
		head = ACCESS_ONCE(circ->head);
		tail = circ->tail;
		s = min_t(int, count,
			CIRC_CNT_TO_END(head, tail, FW_LOG_CIRC_BUF_SIZE));

		if (s > 0) {
			if (copy_to_user(&buf[n], &circ->buf[tail], s)) {
				dev_warn(dev, "copy_to_user failed\n");
				mutex_unlock(&tegra->log.mutex);
				return -EFAULT;
			}
			circ->tail = (circ->tail + s) &
				(FW_LOG_CIRC_BUF_SIZE - 1);

			count -= s;
			n += s;
		} else
			break;
	}

	mutex_unlock(&tegra->log.mutex);

	wake_up_interruptible(&tegra->log.write_wait);

	dev_dbg(dev, "%s: %d bytes\n", __func__, n);

	return n;
}

static int fw_log_file_open(struct inode *inode, struct file *file)
{
	struct tegra_xhci_hcd *tegra;

	file->private_data = inode->i_private;
	tegra = file->private_data;

	if (test_and_set_bit(FW_LOG_FILE_OPENED, &tegra->log.flags)) {
		dev_info(&tegra->pdev->dev, "%s: already opened\n", __func__);
		return -EBUSY;
	}

	return 0;
}

static int fw_log_file_close(struct inode *inode, struct file *file)
{
	struct tegra_xhci_hcd *tegra = file->private_data;

	clear_bit(FW_LOG_FILE_OPENED, &tegra->log.flags);

	return 0;
}

static const struct file_operations firmware_log_fops = {
	.open		= fw_log_file_open,
	.release	= fw_log_file_close,
	.read		= fw_log_file_read,
	.owner		= THIS_MODULE,
};

static int fw_log_init(struct tegra_xhci_hcd *tegra)
{
	struct device *dev = &tegra->pdev->dev;
	int rc = 0;

	/* Allocate buffer to be shared between driver and firmware */
	tegra->log.virt_addr = dma_alloc_writecombine(dev, FW_LOG_RING_SIZE,
						      &tegra->log.phys_addr,
						      GFP_KERNEL);
	if (!tegra->log.virt_addr) {
		dev_err(dev, "dma_alloc_writecombine() size %d failed\n",
			FW_LOG_RING_SIZE);
		return -ENOMEM;
	}

	dev_info(dev, "%d bytes log buffer physical 0x%u virtual 0x%p\n",
		 FW_LOG_RING_SIZE, (u32)tegra->log.phys_addr,
		 tegra->log.virt_addr);

	memset(tegra->log.virt_addr, 0, FW_LOG_RING_SIZE);
	tegra->log.dequeue = tegra->log.virt_addr;

	tegra->log.circ.buf = vmalloc(FW_LOG_CIRC_BUF_SIZE);
	if (!tegra->log.circ.buf) {
		dev_err(dev, "vmalloc size %d failed\n",
			FW_LOG_CIRC_BUF_SIZE);
		rc = -ENOMEM;
		goto error_free_dma;
	}

	tegra->log.circ.head = 0;
	tegra->log.circ.tail = 0;

	init_waitqueue_head(&tegra->log.read_wait);
	init_waitqueue_head(&tegra->log.write_wait);
	init_waitqueue_head(&tegra->log.intr_wait);

	mutex_init(&tegra->log.mutex);

	tegra->log.path = debugfs_create_dir("tegra_xhci", NULL);
	if (IS_ERR_OR_NULL(tegra->log.path)) {
		dev_warn(dev, "debugfs_create_dir() failed\n");
		rc = -ENOMEM;
		goto error_free_mem;
	}

	tegra->log.log_file = debugfs_create_file("firmware_log",
			S_IRUGO, tegra->log.path, tegra, &firmware_log_fops);
	if (IS_ERR_OR_NULL(tegra->log.log_file)) {
		dev_warn(dev, "debugfs_create_file() failed\n");
		rc = -ENOMEM;
		goto error_remove_debugfs_path;
	}

	tegra->log.thread = kthread_run(fw_log_thread, tegra, "xusb-fw-log");
	if (IS_ERR(tegra->log.thread)) {
		dev_warn(dev, "kthread_run() failed\n");
		rc = -ENOMEM;
		goto error_remove_debugfs_file;
	}

	set_bit(FW_LOG_CONTEXT_VALID, &tegra->log.flags);
	return rc;

error_remove_debugfs_file:
	debugfs_remove(tegra->log.log_file);
error_remove_debugfs_path:
	debugfs_remove(tegra->log.path);
error_free_mem:
	vfree(tegra->log.circ.buf);
error_free_dma:
	dma_free_writecombine(dev, FW_LOG_RING_SIZE,
			      tegra->log.virt_addr, tegra->log.phys_addr);
	memset(&tegra->log, 0, sizeof(tegra->log));
	return rc;
}

static void fw_log_deinit(struct tegra_xhci_hcd *tegra)
{
	struct device *dev = &tegra->pdev->dev;

	if (test_and_clear_bit(FW_LOG_CONTEXT_VALID, &tegra->log.flags)) {
		debugfs_remove(tegra->log.log_file);
		debugfs_remove(tegra->log.path);

		wake_up_interruptible(&tegra->log.read_wait);
		wake_up_interruptible(&tegra->log.write_wait);
		kthread_stop(tegra->log.thread);

		mutex_lock(&tegra->log.mutex);
		dma_free_writecombine(dev, FW_LOG_RING_SIZE,
				      tegra->log.virt_addr,
				      tegra->log.phys_addr);
		vfree(tegra->log.circ.buf);
		tegra->log.circ.head = tegra->log.circ.tail = 0;
		mutex_unlock(&tegra->log.mutex);

		mutex_destroy(&tegra->log.mutex);
	}
}

static void fw_log_suspend(struct tegra_xhci_hcd *tegra)
{
	struct device *dev = &tegra->pdev->dev;

	/* In ELPG, firmware log context is gone. Rewind shared log buffer. */
	if (fw_log_wait_empty_timeout(tegra, 100))
		dev_warn(dev, "fw still has logs\n");
	tegra->log.dequeue = tegra->log.virt_addr;
	tegra->log.seq = 0;
}

static void fw_log_init_cfgtbl(struct tegra_xhci_hcd *tegra)
{
	struct cfgtbl *cfg_tbl = (struct cfgtbl *)tegra->firmware.data;

	/* Update the phys_log_buffer and total_entries */
	cfg_tbl->phys_addr_log_buffer = tegra->log.phys_addr;
	cfg_tbl->total_log_entries = FW_LOG_COUNT;
}
#else
static inline int fw_log_init(struct tegra_xhci_hcd *tegra)
{
	return 0;
}
static inline void fw_log_deinit(struct tegra_xhci_hcd *tegra)
{
}
static inline void fw_log_suspend(struct tegra_xhci_hcd *tegra)
{
}
static inline void fw_log_init_cfgtbl(struct tegra_xhci_hcd *tegra)
{
}
#endif

static int tegra_xhci_phy_notifier(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	struct tegra_xhci_hcd *tegra = container_of(nb, struct tegra_xhci_hcd,
						    phy_nb);

	if (action != USB_EVENT_VBUS)
		return NOTIFY_DONE;

	pm_request_resume(&tegra->pdev->dev);

	return NOTIFY_OK;
}

static void tegra_xhci_cfg(struct tegra_xhci_hcd *tegra)
{
	u32 reg;

	reg = readl(tegra->ipfs_base + IPFS_XUSB_HOST_CONFIGURATION_0);
	reg |= IPFS_EN_FPCI;
	writel(reg, tegra->ipfs_base + IPFS_XUSB_HOST_CONFIGURATION_0);
	udelay(10);

	/* Program Bar0 Space */
	reg = readl(tegra->fpci_base + XUSB_CFG_4);
	reg |= tegra->host_phys_base;
	writel(reg, tegra->fpci_base + XUSB_CFG_4);
	usleep_range(100, 200);

	/* Enable Bus Master */
	reg = readl(tegra->fpci_base + XUSB_CFG_1);
	reg |= 0x7;
	writel(reg, tegra->fpci_base + XUSB_CFG_1);

	/* Set intr mask to enable intr assertion */
	reg = readl(tegra->ipfs_base + IPFS_XUSB_HOST_INTR_MASK_0);
	reg |= IPFS_IP_INT_MASK;
	writel(reg, tegra->ipfs_base + IPFS_XUSB_HOST_INTR_MASK_0);

	/* Set hysteris to 0x80 */
	writel(0x80, tegra->ipfs_base + IPFS_XUSB_HOST_CLKGATE_HYSTERESIS_0);
}

static int tegra_xhci_mbox_send(struct tegra_xhci_hcd *tegra,
				enum tegra_xusb_mbox_cmd type, u32 data)
{
	struct device *dev = &tegra->pdev->dev;
	void __iomem *base = tegra->fpci_base;
	unsigned long target;
	u32 reg;

	dev_dbg(dev, "MBOX send message 0x%x:0x%x\n", type, data);
	mutex_lock(&tegra->mbox_lock);

	target = jiffies + msecs_to_jiffies(20);
	/* Wait for mailbox to become idle */
	while (((reg = readl(base + XUSB_CFG_ARU_MBOX_OWNER)) != 0) &&
		time_is_after_jiffies(target)) {
		mutex_unlock(&tegra->mbox_lock);
		usleep_range(100, 200);
		mutex_lock(&tegra->mbox_lock);
	}

	if (reg != 0) {
		dev_err(dev, "Mailbox is still busy\n");
		goto timeout;
	}

	target = jiffies + msecs_to_jiffies(10);
	/* Acquire mailbox */
	writel(MBOX_OWNER_SW, base + XUSB_CFG_ARU_MBOX_OWNER);
	while (((reg = readl(base + XUSB_CFG_ARU_MBOX_OWNER)) != MBOX_OWNER_SW)
		&& time_is_after_jiffies(target)) {
		mutex_unlock(&tegra->mbox_lock);
		usleep_range(100, 200);
		mutex_lock(&tegra->mbox_lock);
		writel(MBOX_OWNER_SW, base + XUSB_CFG_ARU_MBOX_OWNER);
	}

	if (reg != MBOX_OWNER_SW) {
		dev_err(dev, "Acquire mailbox timeout\n");
		goto timeout;
	}

	reg = CMD_TYPE(type) | CMD_DATA(data);
	writel(reg, base + XUSB_CFG_ARU_MBOX_DATA_IN);

	reg = readl(tegra->fpci_base + XUSB_CFG_ARU_MBOX_CMD);
	reg |= MBOX_INT_EN | MBOX_FALC_INT_EN;
	writel(reg, tegra->fpci_base + XUSB_CFG_ARU_MBOX_CMD);

	mutex_unlock(&tegra->mbox_lock);

	return 0;

timeout:
	reg_dump(dev, base, XUSB_CFG_ARU_MBOX_CMD);
	reg_dump(dev, base, XUSB_CFG_ARU_MBOX_DATA_IN);
	reg_dump(dev, base, XUSB_CFG_ARU_MBOX_DATA_OUT);
	reg_dump(dev, base, XUSB_CFG_ARU_MBOX_OWNER);
	mutex_unlock(&tegra->mbox_lock);
	return -ETIMEDOUT;
}

static irqreturn_t tegra_xhci_mbox_irq(int irq, void *ptrdev)
{
	struct tegra_xhci_hcd *tegra = (struct tegra_xhci_hcd *)ptrdev;
	struct device *dev = &tegra->pdev->dev;
	u32 resp = 0, cmd_type, cmd_data, reg;

	pm_runtime_get_sync(dev);
	mutex_lock(&tegra->mbox_lock);
	/*
	 * Clear the mbox interrupt. Other bits are W1C bits, so just write
	 * to SMI bit.
	 */
	reg = readl(tegra->fpci_base + XUSB_CFG_ARU_SMI_INTR);
	if (reg & MBOX_SMI_INTR_FW_HANG)
		dev_err(dev, "Hang up inside firmware\n");
	writel(reg, tegra->fpci_base + XUSB_CFG_ARU_SMI_INTR);

	/* Get the mbox message from firmware */
	reg = readl(tegra->fpci_base + XUSB_CFG_ARU_MBOX_DATA_OUT);
	cmd_type = MBOX_MSG_CMD(reg);
	cmd_data = MBOX_MSG_DATA(reg);

	/* Decode the message and call the notifiers */
	dev_dbg(dev, "MBOX receive message 0x%x:0x%x\n", cmd_type, cmd_data);
	if (cmd_type < MBOX_CMD_MAX) {
		struct mbox_notifier_data mbox_info;

		mbox_info.msg_data = cmd_data;
		mbox_info.resp_cmd = 0;
		mbox_info.resp_data = 0;
		raw_notifier_call_chain(&tegra->mbox_notifiers, cmd_type,
					&mbox_info);
		if (mbox_info.resp_cmd)
			resp = MBOX_PACK_MSG(mbox_info.resp_cmd,
					     mbox_info.resp_data);
	} else if (cmd_type == MBOX_CMD_ACK) {
		dev_dbg(dev, "Firmware responds with ACK\n");
	} else if (cmd_type == MBOX_CMD_NACK) {
		dev_err(dev, "Firmware responds with NACK\n");
	} else {
		dev_err(dev, "Invalid command %x\n", cmd_type);
	}

	if (resp) {
		/* Send ACK/NACK to firmware */
		writel(resp, tegra->fpci_base + XUSB_CFG_ARU_MBOX_DATA_IN);
		reg = readl(tegra->fpci_base + XUSB_CFG_ARU_MBOX_CMD);
		reg |= MBOX_INT_EN | MBOX_FALC_INT_EN;
		writel(reg, tegra->fpci_base + XUSB_CFG_ARU_MBOX_CMD);
	} else {
		/* Clear MBOX_SMI_INT_EN bit */
		reg = readl(tegra->fpci_base + XUSB_CFG_ARU_MBOX_CMD);
		reg &= ~MBOX_SMI_INT_EN;
		writel(reg, tegra->fpci_base + XUSB_CFG_ARU_MBOX_CMD);
		/* Clear mailbox ownership */
		writel(0, tegra->fpci_base + XUSB_CFG_ARU_MBOX_OWNER);
	}

	mutex_unlock(&tegra->mbox_lock);
	pm_runtime_put(dev);

	return IRQ_HANDLED;
}

static int tegra_xhci_default_mbox_notifier(struct notifier_block *nb,
					    unsigned long event, void *p)
{
	struct tegra_xhci_hcd *tegra = container_of(nb, struct tegra_xhci_hcd,
						    mbox_nb);
	struct mbox_notifier_data *data = (struct mbox_notifier_data *)p;
	unsigned long freq;

	switch (event) {
	case MBOX_CMD_INC_FALC_CLOCK:
	case MBOX_CMD_DEC_FALC_CLOCK:
		data->resp_data = clk_get_rate(tegra->falc_clk) / 1000;
		if (data->resp_data != data->msg_data)
			data->resp_cmd = MBOX_CMD_NACK;
		else
			data->resp_cmd = MBOX_CMD_ACK;
		return NOTIFY_STOP;
	case MBOX_CMD_SET_BW:
		freq = tegra_emc_bw_to_freq_req(data->msg_data << 10) * 1000;
		clk_set_rate(tegra->emc_clk, freq);
		return NOTIFY_STOP;
	default:
		return NOTIFY_DONE;
	}
}

int tegra_xhci_register_mbox_notifier(struct tegra_xhci_hcd *tegra,
				      struct notifier_block *nb)
{
	int ret;

	mutex_lock(&tegra->mbox_lock);
	ret = raw_notifier_chain_register(&tegra->mbox_notifiers, nb);
	mutex_unlock(&tegra->mbox_lock);

	return ret;
}
EXPORT_SYMBOL(tegra_xhci_register_mbox_notifier);

void tegra_xhci_unregister_mbox_notifier(struct tegra_xhci_hcd *tegra,
					 struct notifier_block *nb)
{
	mutex_lock(&tegra->mbox_lock);
	raw_notifier_chain_unregister(&tegra->mbox_notifiers, nb);
	mutex_unlock(&tegra->mbox_lock);
}
EXPORT_SYMBOL(tegra_xhci_unregister_mbox_notifier);

static int load_firmware(struct tegra_xhci_hcd *tegra)
{
	struct device *dev = &tegra->pdev->dev;
	struct cfgtbl *cfg_tbl = (struct cfgtbl *) tegra->firmware.data;
	u32 phys_addr_lo;
	u32 hw_reg;
	u16 nblocks;
	time_t fw_time;
	struct tm fw_tm;

	if (csb_read(tegra, XUSB_CSB_MP_ILOAD_BASE_LO) != 0) {
		dev_info(dev, "Firmware already loaded, Falcon state 0x%x\n",
			 csb_read(tegra, XUSB_FALC_CPUCTL));
		return 0;
	}

	fw_log_init_cfgtbl(tegra);

	phys_addr_lo = tegra->firmware.dma;
	phys_addr_lo += sizeof(struct cfgtbl);

	/* Program the size of DFI into ILOAD_ATTR */
	csb_write(tegra, XUSB_CSB_MP_ILOAD_ATTR, tegra->firmware.size);

	/*
	 * Boot code of the firmware reads the ILOAD_BASE_LO register
	 * to get to the start of the dfi in system memory.
	 */
	csb_write(tegra, XUSB_CSB_MP_ILOAD_BASE_LO, phys_addr_lo);

	/* Program the ILOAD_BASE_HI with a value of MSB 32 bits */
	csb_write(tegra, XUSB_CSB_MP_ILOAD_BASE_HI, 0);

	/* Set BOOTPATH to 1 in APMAP Register. Bit 31 is APMAP_BOOTMAP */
	csb_write(tegra, XUSB_CSB_MP_APMAP, APMAP_BOOTPATH);

	/* Invalidate L2IMEM. */
	csb_write(tegra, XUSB_CSB_MP_L2IMEMOP_TRIG, L2IMEM_INVALIDATE_ALL);

	/*
	 * Initiate fetch of Bootcode from system memory into L2IMEM.
	 * Program BootCode location and size in system memory.
	 */
	hw_reg = (DIV_ROUND_UP(cfg_tbl->boot_codetag, IMEM_BLOCK_SIZE) &
			L2IMEMOP_SIZE_SRC_OFFSET_MASK)
			<< L2IMEMOP_SIZE_SRC_OFFSET_SHIFT;
	hw_reg |= (DIV_ROUND_UP(cfg_tbl->boot_codesize, IMEM_BLOCK_SIZE) &
			L2IMEMOP_SIZE_SRC_COUNT_MASK)
			<< L2IMEMOP_SIZE_SRC_COUNT_SHIFT;
	csb_write(tegra, XUSB_CSB_MP_L2IMEMOP_SIZE, hw_reg);

	/* Trigger L2IMEM Load operation. */
	csb_write(tegra, XUSB_CSB_MP_L2IMEMOP_TRIG, L2IMEM_LOAD_LOCKED_RESULT);

	/* Setup Falcon Auto-fill */
	nblocks = DIV_ROUND_UP(cfg_tbl->boot_codesize, IMEM_BLOCK_SIZE);
	csb_write(tegra, XUSB_FALC_IMFILLCTL, nblocks);

	hw_reg = DIV_ROUND_UP(cfg_tbl->boot_codetag, IMEM_BLOCK_SIZE) &
			IMFILLRNG_TAG_MASK;
	hw_reg |= DIV_ROUND_UP(cfg_tbl->boot_codetag + cfg_tbl->boot_codesize,
			IMEM_BLOCK_SIZE) << IMFILLRNG1_TAG_HI_SHIFT;
	csb_write(tegra, XUSB_FALC_IMFILLRNG1, hw_reg);

	csb_write(tegra, XUSB_FALC_DMACTL, 0);
	msleep(50);

	csb_write(tegra, XUSB_FALC_BOOTVEC, cfg_tbl->boot_codetag);

	/* Start Falcon CPU */
	csb_write(tegra, XUSB_FALC_CPUCTL, CPUCTL_STARTCPU);
	usleep_range(1000, 2000);

	fw_time = cfg_tbl->fwimg_created_time;
	time_to_tm(fw_time, 0, &fw_tm);
	dev_info(dev,
		 "Firmware timestamp: %ld-%02d-%02d %02d:%02d:%02d UTC, "
		 "Falcon state 0x%x\n", fw_tm.tm_year + 1900,
		 fw_tm.tm_mon + 1, fw_tm.tm_mday, fw_tm.tm_hour,
		 fw_tm.tm_min, fw_tm.tm_sec,
		 csb_read(tegra, XUSB_FALC_CPUCTL));

	/* Bail out if Falcon CPU is not in a good state */
	if (csb_read(tegra, XUSB_FALC_CPUCTL) == XUSB_FALC_STATE_HALTED)
		return -EFAULT;

	return 0;
}

static void init_firmware_done(const struct firmware *fw, void *context)
{
	struct tegra_xhci_hcd *tegra = context;
	struct device *dev = &tegra->pdev->dev;
	struct cfgtbl *fw_cfgtbl;
	size_t fw_size;
	void *fw_data;
	dma_addr_t fw_dma;
	int ret;

	mutex_lock(&tegra->sync_lock);

	if (fw == NULL) {
		dev_err(dev, "failed to init firmware from filesystem: %s\n",
			tegra->soc_config->firmware_file);
		goto err_firmware_done;
	}

	fw_cfgtbl = (struct cfgtbl *)fw->data;
	fw_size = fw_cfgtbl->fwimg_len;
	dev_info(dev, "Firmware File: %s (%d Bytes)\n",
		 tegra->soc_config->firmware_file, fw_size);

	fw_data = dma_alloc_coherent(dev, fw_size, &fw_dma, GFP_KERNEL);
	if (!fw_data) {
		dev_err(dev, "dma_alloc_coherent failed\n");
		goto err_firmware_done;
	}

	memcpy(fw_data, fw->data, fw_size);
	dev_info(dev,
		 "Firmware DMA Memory: dma 0x%llx mapped 0x%p (%d Bytes)\n",
		 (u64)fw_dma, fw_data, fw_size);

	/* all set and ready to go */
	tegra->firmware.data = fw_data;
	tegra->firmware.dma = fw_dma;
	tegra->firmware.size = fw_size;

	ret = tegra_xhci_probe2(tegra);
	if (ret < 0) {
		dev_err(dev, "failed to probe: %d\n", ret);
		goto err_firmware_done;
	}

	release_firmware(fw);
	mutex_unlock(&tegra->sync_lock);
	return;

err_firmware_done:
	release_firmware(fw);
	mutex_unlock(&tegra->sync_lock);
	platform_device_unregister(tegra->pdev);
}

static int init_firmware(struct tegra_xhci_hcd *tegra)
{
	int ret;

	ret = tegra_xhci_register_mbox_notifier(tegra, &tegra->mbox_nb);
	if (ret < 0) {
		dev_err(&tegra->pdev->dev, "register mbox notifier failed\n");
		return ret;
	}

	ret = request_firmware_nowait(THIS_MODULE, true,
		tegra->soc_config->firmware_file, &tegra->pdev->dev, GFP_KERNEL,
		tegra, init_firmware_done);
	if (ret < 0) {
		dev_err(&tegra->pdev->dev, "request_firmware failed %d\n", ret);
		return ret;
	}

	return ret;
}

static void deinit_firmware(struct tegra_xhci_hcd *tegra)
{
	if (tegra->firmware.data)
		dma_free_coherent(&tegra->pdev->dev, tegra->firmware.size,
			tegra->firmware.data, tegra->firmware.dma);

	tegra_xhci_unregister_mbox_notifier(tegra, &tegra->mbox_nb);
}

static int tegra_xusb_regulator_init(struct tegra_xhci_hcd *tegra)
{
	struct device *dev = &tegra->pdev->dev;
	int err = 0;

	tegra->xusb_s3p3v_reg = devm_regulator_get(dev, "s3p3v");
	if (IS_ERR(tegra->xusb_s3p3v_reg)) {
		dev_err(dev, "s3p3v regulator not found: %ld.",
			PTR_ERR(tegra->xusb_s3p3v_reg));
		return PTR_ERR(tegra->xusb_s3p3v_reg);
	}
	err = regulator_enable(tegra->xusb_s3p3v_reg);
	if (err < 0) {
		dev_err(dev, "s3p3v regulator enable failed:%d\n", err);
		return err;
	}

	tegra->xusb_s1p8v_reg = devm_regulator_get(dev, "s1p8v");
	if (IS_ERR(tegra->xusb_s1p8v_reg)) {
		dev_err(dev, "s1p8v regulator not found: %ld.",
			PTR_ERR(tegra->xusb_s1p8v_reg));
		err = PTR_ERR(tegra->xusb_s1p8v_reg);
		goto err_put_s3p3v_reg;
	} else {
		err = regulator_enable(tegra->xusb_s1p8v_reg);
		if (err < 0) {
			dev_err(dev, "s1p8v regulator enable failed:%d\n", err);
			goto err_put_s3p3v_reg;
		}
	}

	tegra->xusb_s1p05v_reg = devm_regulator_get(dev, "s1p05v");
	if (IS_ERR(tegra->xusb_s1p05v_reg)) {
		dev_err(dev, "s1p05v regulator not found: %ld.",
			PTR_ERR(tegra->xusb_s1p05v_reg));
		err = PTR_ERR(tegra->xusb_s1p05v_reg);
		goto err_put_s1p8v_reg;
	} else {
		err = regulator_enable(tegra->xusb_s1p05v_reg);
		if (err < 0) {
			dev_err(dev,
				"s1p05v regulator enable failed:%d\n", err);
			goto err_put_s1p8v_reg;
		}
	}

	return 0;

err_put_s1p8v_reg:
	regulator_disable(tegra->xusb_s1p8v_reg);
err_put_s3p3v_reg:
	regulator_disable(tegra->xusb_s3p3v_reg);
	return err;
}

static void tegra_xusb_regulator_deinit(struct tegra_xhci_hcd *tegra)
{
	regulator_disable(tegra->xusb_s1p05v_reg);
	regulator_disable(tegra->xusb_s1p8v_reg);
	regulator_disable(tegra->xusb_s3p3v_reg);
}

static int tegra_xusb_clk_init(struct tegra_xhci_hcd *tegra)
{
	struct device *dev = &tegra->pdev->dev;
	int err = 0;

	if (tegra->soc_config->use_hs_src_clk2) {
		tegra->pll_re_vco_clk = devm_clk_get(dev, "pll_re_vco");
		if (IS_ERR(tegra->pll_re_vco_clk)) {
			dev_err(dev, "Failed to get refPLLE clock\n");
			err = PTR_ERR(tegra->pll_re_vco_clk);
			return err;
		}
	}

	tegra->emc_clk = devm_clk_get(dev, "xusb.emc");
	if (IS_ERR(tegra->emc_clk)) {
		dev_err(dev, "Failed to get xusb.emc clock\n");
		return PTR_ERR(tegra->emc_clk);
	}

	tegra->host_clk = devm_clk_get(dev, "xusb_host");
	if (IS_ERR(tegra->host_clk)) {
		dev_err(dev, "Failed to get host partition clk\n");
		err = PTR_ERR(tegra->host_clk);
		return err;
	}

	tegra->falc_clk = devm_clk_get(dev, "xusb_falcon_src");
	if (IS_ERR(tegra->falc_clk)) {
		dev_err(dev, "Failed to get xusb_falcon_src clk\n");
		err = PTR_ERR(tegra->falc_clk);
		return err;
	}

	if (tegra->soc_config->use_hs_src_clk2) {
		err = clk_prepare_enable(tegra->pll_re_vco_clk);
		if (err) {
			dev_err(dev, "Failed to enable refPLLE clk\n");
			return err;
		}
	}

	err = clk_prepare_enable(tegra->host_clk);
	if (err) {
		dev_err(dev, "Failed to enable host partition clk\n");
		goto enable_host_clk_failed;
	}

	err = clk_prepare_enable(tegra->emc_clk);
	if (err) {
		dev_err(dev, "Failed to enable xusb.emc clk\n");
		goto enable_emc_clk_failed;
	}

	err = clk_prepare_enable(tegra->falc_clk);
	if (err) {
		dev_err(dev, "Failed to enable xusb_falcon_src clk\n");
		goto enable_falc_clk_failed;
	}

	return 0;

enable_falc_clk_failed:
	clk_disable_unprepare(tegra->emc_clk);
enable_emc_clk_failed:
	clk_disable_unprepare(tegra->host_clk);
enable_host_clk_failed:
	if (tegra->soc_config->use_hs_src_clk2)
		clk_disable_unprepare(tegra->pll_re_vco_clk);

	return err;
}

static void tegra_xusb_clk_deinit(struct tegra_xhci_hcd *tegra)
{
	clk_disable_unprepare(tegra->host_clk);
	clk_disable_unprepare(tegra->falc_clk);
	if (tegra->soc_config->use_hs_src_clk2)
		clk_disable_unprepare(tegra->pll_re_vco_clk);
}

static int tegra_xhci_bus_notifier(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	struct tegra_xhci_hcd *tegra = container_of(nb, struct tegra_xhci_hcd,
						    bus_nb);
	struct device *dev = data;

	if (tegra->xhci_pdev && dev == &tegra->xhci_pdev->dev &&
	    action == BUS_NOTIFY_BOUND_DRIVER)
		tegra->init_done = true;

	return NOTIFY_DONE;
}

static const struct tegra_xusb_soc_config tegra114_soc_config = {
	.use_hs_src_clk2 = true,
	.firmware_file = "tegra11x/tegra_xusb_firmware",
};

static const struct tegra_xusb_soc_config tegra124_soc_config = {
	.use_hs_src_clk2 = false,
	.firmware_file = "tegra12x/tegra_xusb_firmware",
};

static struct of_device_id tegra_xhci_of_match[] = {
	{ .compatible = "nvidia,tegra114-xhci", .data = &tegra114_soc_config },
	{ .compatible = "nvidia,tegra124-xhci", .data = &tegra124_soc_config },
	{ },
};

static int tegra_xhci_probe(struct platform_device *pdev)
{
	struct tegra_xhci_hcd *tegra;
	struct resource	*res;
	u32 val;
	int ret;
	const struct of_device_id *match;

	BUILD_BUG_ON(sizeof(struct cfgtbl) != 256);

	tegra = devm_kzalloc(&pdev->dev, sizeof(*tegra), GFP_KERNEL);
	if (!tegra) {
		dev_err(&pdev->dev, "memory alloc failed\n");
		return -ENOMEM;
	}
	mutex_init(&tegra->sync_lock);
	mutex_init(&tegra->mbox_lock);
	RAW_INIT_NOTIFIER_HEAD(&tegra->mbox_notifiers);
	tegra->mbox_nb.notifier_call = tegra_xhci_default_mbox_notifier;

	match = of_match_device(tegra_xhci_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "No device match found\n");
		return -ENODEV;
	}
	tegra->pdev = pdev;
	tegra->soc_config = match->data;

	/* Get XUSB PHY device */
	tegra->phy = devm_usb_get_phy_by_phandle(&pdev->dev, "nvidia,xusb-phy",
						 0);
	if (IS_ERR(tegra->phy)) {
		dev_err(&pdev->dev, "Failed to get PHY\n");
		return PTR_ERR(tegra->phy);
	}

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64));

	/* Map XHCI registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "mem resource host doesn't exist\n");
		return -ENODEV;
	}
	tegra->host_phys_base = res->start;

	/* Map FPCI registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "mem resource fpci doesn't exist\n");
		return -ENODEV;
	}
	tegra->fpci_base = devm_ioremap(&pdev->dev, res->start,
					resource_size(res));
	if (ret) {
		dev_err(&pdev->dev, "failed to map fpci\n");
		return ret;
	}

	/* Map IPFS registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dev_err(&pdev->dev, "mem resource ipfs doesn't exist\n");
		return -ENODEV;
	}
	tegra->ipfs_base = devm_ioremap(&pdev->dev, res->start,
					resource_size(res));
	if (ret) {
		dev_err(&pdev->dev, "failed to map ipfs\n");
		return ret;
	}

	ret = tegra_xusb_clk_init(tegra);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize xusb clocks\n");
		return ret;
	}

	ret = tegra_xusb_regulator_init(tegra);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize xusb regulator\n");
		goto err_deinit_xusb_partition_clk;
	}

	/* Unpowergate SS partition */
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_XUSBA);
	if (ret)
		dev_err(&pdev->dev, "could not unpowergate xusba partition\n");

	/* Unpowergate host partition */
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_XUSBC);
	if (ret)
		dev_err(&pdev->dev, "could not unpowergate xusbc partition\n");

	/* Powergate device partition when device mode is not used */
	ret = tegra_powergate_partition(TEGRA_POWERGATE_XUSBB);
	if (ret)
		dev_err(&pdev->dev, "could not powergate xusbb partition\n");

	/* Setup IPFS access and BAR0 space */
	tegra_xhci_cfg(tegra);

	val = readl(tegra->fpci_base + XUSB_CFG_0);
	tegra->device_id = (val >> 16) & 0xffff;
	dev_info(&pdev->dev, "XUSB device id = 0x%x\n", tegra->device_id);

	/* Initialize the PHY */
	tegra_xusb_phy_bind_xhci_dev(tegra->phy, tegra);
	ret = usb_phy_init(tegra->phy);
	if (ret < 0) {
		dev_err(&pdev->dev, "PHY initialization failed\n");
		goto err_deinit_regulator;
	}

	/* Register a VBUS notifier which will be used for runtime wakeup */
	tegra->phy_nb.notifier_call = tegra_xhci_phy_notifier;
	ret = usb_register_notifier(tegra->phy, &tegra->phy_nb);
	if (ret < 0) {
		dev_err(&pdev->dev, "USB notifier registration failed\n");
		goto err_shutdown_phy;
	}

	/* Deassert reset to XUSB host */
	tegra_periph_reset_deassert(tegra->host_clk);

	platform_set_drvdata(pdev, tegra);
	fw_log_init(tegra);

	ret = init_firmware(tegra);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to init firmware\n");
		ret = -ENODEV;
		goto err_deinit_firmware_log;
	}

	return 0;

err_deinit_firmware_log:
	fw_log_deinit(tegra);
	usb_unregister_notifier(tegra->phy, &tegra->phy_nb);
err_shutdown_phy:
	usb_phy_shutdown(tegra->phy);
err_deinit_regulator:
	tegra_xusb_regulator_deinit(tegra);
err_deinit_xusb_partition_clk:
	tegra_xusb_clk_deinit(tegra);

	return ret;
}

static int tegra_xhci_probe2(struct tegra_xhci_hcd *tegra)
{
	struct platform_device *pdev = tegra->pdev;
	struct platform_device *xhci;
	int ret;
	struct resource xhci_resources[2];
	struct resource	*res;

	ret = load_firmware(tegra);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to load firmware\n");
		return -ENODEV;
	}

	device_init_wakeup(&pdev->dev, 1);

	tegra->bus_nb.notifier_call = tegra_xhci_bus_notifier;
	bus_register_notifier(&platform_bus_type, &tegra->bus_nb);

	memset(xhci_resources, 0, sizeof(xhci_resources));
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "missing XHCI IRQ\n");
		return -ENODEV;
	}
	xhci_resources[0].start = res->start;
	xhci_resources[0].end = res->end;
	xhci_resources[0].flags = res->flags;
	xhci_resources[0].name = res->name;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "missing XHCI registers\n");
		return -ENODEV;
	}
	xhci_resources[1].start = res->start;
	xhci_resources[1].end = res->end;
	xhci_resources[1].flags = res->flags;
	xhci_resources[1].name = res->name;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!res) {
		dev_err(&pdev->dev, "missing MBOX IRQ\n");
		return -ENODEV;
	}
	tegra->mbox_irq = res->start;
	ret = devm_request_threaded_irq(&pdev->dev, tegra->mbox_irq, NULL,
					tegra_xhci_mbox_irq, IRQF_ONESHOT,
					"tegra_xhci_mbox_irq", tegra);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to request MBOX IRQ: %d\n", ret);
		return ret;
	}

	xhci = platform_device_alloc("xhci-hcd", PLATFORM_DEVID_AUTO);
	if (!xhci)
		return -ENOMEM;
	dma_set_coherent_mask(&xhci->dev, pdev->dev.coherent_dma_mask);
	xhci->dev.parent = &pdev->dev;
	xhci->dev.dma_mask = pdev->dev.dma_mask;
	xhci->dev.dma_parms = pdev->dev.dma_parms;
	tegra->xhci_pdev = xhci;

	ret = platform_device_add_resources(xhci, xhci_resources,
					    ARRAY_SIZE(xhci_resources));
	if (ret) {
		dev_err(&pdev->dev, "failed to add XHCI resources\n");
		goto err;
	}

	ret = platform_device_add(xhci);
	if (ret) {
		dev_err(&pdev->dev, "failed to register xHCI device\n");
		goto err;
	}

	/* Enable firmware message */
	tegra_xhci_mbox_send(tegra, MBOX_CMD_MSG_ENABLED, 0);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	device_enable_async_suspend(&pdev->dev);

	return 0;
err:
	platform_device_put(xhci);
	return ret;
}

static int tegra_xhci_remove(struct platform_device *pdev)
{
	struct tegra_xhci_hcd *tegra = platform_get_drvdata(pdev);

	mutex_lock(&tegra->sync_lock);

	deinit_firmware(tegra);
	fw_log_deinit(tegra);

	usb_unregister_notifier(tegra->phy, &tegra->phy_nb);
	usb_phy_shutdown(tegra->phy);

	tegra_xusb_regulator_deinit(tegra);
	tegra_xusb_clk_deinit(tegra);

	mutex_unlock(&tegra->sync_lock);

	return 0;
}

#ifdef CONFIG_PM
static void tegra_xhci_save_xusb_ctx(struct tegra_xhci_hcd *tegra)
{
	/* Save IPFS registers */
	tegra->sregs.msi_bar_sz =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_MSI_BAR_SZ_0);

	tegra->sregs.msi_axi_barst =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_MSI_AXI_BAR_ST_0);

	tegra->sregs.msi_fpci_barst =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_FPCI_BAR_ST_0);

	tegra->sregs.msi_vec0 =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_MSI_VEC0_0);

	tegra->sregs.msi_en_vec0 =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_MSI_EN_VEC0_0);

	tegra->sregs.fpci_error_masks =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_FPCI_ERROR_MASKS_0);

	tegra->sregs.intr_mask =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_INTR_MASK_0);

	tegra->sregs.ipfs_intr_enable =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_IPFS_INTR_ENABLE_0);

	tegra->sregs.ufpci_config =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_UFPCI_CONFIG_0);

	tegra->sregs.clkgate_hysteresis =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_CLKGATE_HYSTERESIS_0);

	tegra->sregs.xusb_host_mccif_fifo_cntrl =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_MCCIF_FIFOCTRL_0);

	/* Save CFG registers */
	tegra->sregs.hs_pls =
		readl(tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_HS_PLS);

	tegra->sregs.fs_pls =
		readl(tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_FS_PLS);

	tegra->sregs.hs_fs_speed =
		readl(tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_HSFS_SPEED);

	tegra->sregs.hs_fs_pp =
		readl(tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_HSFS_PP);

	tegra->sregs.cfg_aru =
		readl(tegra->fpci_base + XUSB_CFG_ARU_CONTEXT);

	tegra->sregs.cfg_order =
		readl(tegra->fpci_base + XUSB_CFG_FPCICFG);

	tegra->sregs.cfg_fladj =
		readl(tegra->fpci_base + XUSB_CFG_24);

	tegra->sregs.cfg_sid =
		readl(tegra->fpci_base + XUSB_CFG_16);
}

static void tegra_xhci_restore_xusb_ctx(struct tegra_xhci_hcd *tegra)
{
	/* Restore CFG registers */
	writel(tegra->sregs.hs_pls,
		tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_HS_PLS);

	writel(tegra->sregs.fs_pls,
		tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_FS_PLS);

	writel(tegra->sregs.hs_fs_speed,
		tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_HSFS_SPEED);

	writel(tegra->sregs.hs_fs_pp,
		tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_HSFS_PP);

	writel(tegra->sregs.cfg_aru,
		tegra->fpci_base + XUSB_CFG_ARU_CONTEXT);

	writel(tegra->sregs.cfg_order,
		tegra->fpci_base + XUSB_CFG_FPCICFG);

	writel(tegra->sregs.cfg_fladj,
		tegra->fpci_base + XUSB_CFG_24);

	writel(tegra->sregs.cfg_sid,
		tegra->fpci_base + XUSB_CFG_16);

	/* Restore IPFS registers */
	writel(tegra->sregs.msi_bar_sz,
		tegra->ipfs_base + IPFS_XUSB_HOST_MSI_BAR_SZ_0);

	writel(tegra->sregs.msi_axi_barst,
		tegra->ipfs_base + IPFS_XUSB_HOST_MSI_AXI_BAR_ST_0);

	writel(tegra->sregs.msi_fpci_barst,
		tegra->ipfs_base + IPFS_XUSB_HOST_FPCI_BAR_ST_0);

	writel(tegra->sregs.msi_vec0,
		tegra->ipfs_base + IPFS_XUSB_HOST_MSI_VEC0_0);

	writel(tegra->sregs.msi_en_vec0,
		tegra->ipfs_base + IPFS_XUSB_HOST_MSI_EN_VEC0_0);

	writel(tegra->sregs.fpci_error_masks,
		tegra->ipfs_base + IPFS_XUSB_HOST_FPCI_ERROR_MASKS_0);

	writel(tegra->sregs.intr_mask,
		tegra->ipfs_base + IPFS_XUSB_HOST_INTR_MASK_0);

	writel(tegra->sregs.ipfs_intr_enable,
		tegra->ipfs_base + IPFS_XUSB_HOST_IPFS_INTR_ENABLE_0);

	writel(tegra->sregs.ufpci_config,
		tegra->fpci_base + IPFS_XUSB_HOST_UFPCI_CONFIG_0);

	writel(tegra->sregs.clkgate_hysteresis,
		tegra->ipfs_base + IPFS_XUSB_HOST_CLKGATE_HYSTERESIS_0);

	writel(tegra->sregs.xusb_host_mccif_fifo_cntrl,
		tegra->ipfs_base + IPFS_XUSB_HOST_MCCIF_FIFOCTRL_0);
}

static int tegra_xhci_elpg_enter(struct tegra_xhci_hcd *tegra)
{
	struct device *dev = &tegra->pdev->dev;
	int ret;

	dev_dbg(dev, "ELPG enter\n");

	tegra_xusb_phy_presuspend(tegra->phy);

	/* Powergate SS partition */
	ret = tegra_powergate_partition(TEGRA_POWERGATE_XUSBA);
	if (ret) {
		dev_err(dev, "could not powergate xusba partition\n");
		return ret;
	}

	usb_phy_set_suspend(tegra->phy, true);

	/* Save host context */
	tegra_xhci_save_xusb_ctx(tegra);

	/* Powergate host partition */
	ret = tegra_powergate_partition(TEGRA_POWERGATE_XUSBC);
	if (ret) {
		dev_err(dev, "could not unpowergate xusbc partition %d\n",
			ret);
		return ret;
	}
	clk_disable_unprepare(tegra->host_clk);
	clk_disable_unprepare(tegra->falc_clk);

	if (tegra->soc_config->use_hs_src_clk2)
		clk_disable_unprepare(tegra->pll_re_vco_clk);
	clk_disable_unprepare(tegra->emc_clk);

	tegra_xusb_phy_postsuspend(tegra->phy);

	fw_log_suspend(tegra);

	tegra->hc_in_elpg = true;

	return ret;
}

static int tegra_xhci_elpg_exit(struct tegra_xhci_hcd *tegra)
{
	struct device *dev = &tegra->pdev->dev;
	int ret = 0;

	dev_dbg(dev, "ELPG exit\n");

	clk_prepare_enable(tegra->emc_clk);
	if (tegra->soc_config->use_hs_src_clk2)
		clk_prepare_enable(tegra->pll_re_vco_clk);

	tegra_xusb_phy_preresume(tegra->phy);

	/* Unpowergate host partition */
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_XUSBC);
	if (ret) {
		dev_err(dev, "could not unpowergate xusbc partition %d\n",
			ret);
		return ret;
	}
	clk_prepare_enable(tegra->falc_clk);
	clk_prepare_enable(tegra->host_clk);

	/* Pull host partition out of reset */
	tegra_periph_reset_deassert(tegra->host_clk);

	/* Restore controller context */
	tegra_xhci_cfg(tegra);
	tegra_xhci_restore_xusb_ctx(tegra);

	/* Unpowergate SS partition */
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_XUSBA);
	if (ret) {
		dev_err(dev, "could not unpowergate xusba partition %d\n",
			ret);
		return ret;
	}

	usb_phy_set_suspend(tegra->phy, false);

	/* Load firmware */
	dev_dbg(dev, "elpg_exit: loading firmware from pmc.\n"
		"ss (p1=0x%x, p2=0x%x, p3=0x%x), "
		"hs (p1=0x%x, p2=0x%x, p3=0x%x),\n"
		"fs (p1=0x%x, p2=0x%x, p3=0x%x)\n",
		csb_read(tegra, XUSB_FALC_SS_PVTPORTSC1),
		csb_read(tegra, XUSB_FALC_SS_PVTPORTSC2),
		csb_read(tegra, XUSB_FALC_SS_PVTPORTSC3),
		csb_read(tegra, XUSB_FALC_HS_PVTPORTSC1),
		csb_read(tegra, XUSB_FALC_HS_PVTPORTSC2),
		csb_read(tegra, XUSB_FALC_HS_PVTPORTSC3),
		csb_read(tegra, XUSB_FALC_FS_PVTPORTSC1),
		csb_read(tegra, XUSB_FALC_FS_PVTPORTSC2),
		csb_read(tegra, XUSB_FALC_FS_PVTPORTSC3));

	ret = load_firmware(tegra);
	if (ret < 0) {
		dev_err(dev, "failed to load firmware %d\n", ret);
		return ret;
	}

	tegra_xusb_phy_postresume(tegra->phy);

	tegra->hc_in_elpg = false;

	return ret;
}

static int tegra_xhci_suspend(struct device *dev)
{
	struct tegra_xhci_hcd *tegra = dev_get_drvdata(dev);
	struct usb_hcd *hcd;
	int ret;

	pm_runtime_get_sync(dev);

	mutex_lock(&tegra->sync_lock);
	if (!tegra->init_done) {
		dev_warn(dev, "%s: xhci probe not done\n", __func__);
		mutex_unlock(&tegra->sync_lock);
		return -EBUSY;
	}
	WARN_ON(tegra->hc_in_elpg);

	/* Synchronize wake-enable of host with PHY */
	hcd = tegra_to_hcd(tegra);
	device_init_wakeup(tegra->phy->dev,
			   device_may_wakeup(&hcd->self.root_hub->dev));

	ret = tegra_xhci_elpg_enter(tegra);
	if (ret) {
		dev_err(dev, "unable to perform elpg entry %d\n", ret);
		goto out;
	}

	regulator_disable(tegra->xusb_s1p8v_reg);
	regulator_disable(tegra->xusb_s1p05v_reg);

out:
	mutex_unlock(&tegra->sync_lock);
	pm_runtime_put(dev);

	return ret;
}

static int tegra_xhci_resume(struct device *dev)
{
	struct tegra_xhci_hcd *tegra = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&tegra->sync_lock);
	WARN_ON(!tegra->hc_in_elpg);

	ret = regulator_enable(tegra->xusb_s1p05v_reg);
	if (ret < 0) {
		dev_err(dev, "failed to enable xusb_s1p05v: %d\n", ret);
		goto err;
	}
	ret = regulator_enable(tegra->xusb_s1p8v_reg);
	if (ret < 0) {
		dev_err(dev, "failed to enable xusb_s1p8v: %d\n", ret);
		goto err;
	}

	ret = tegra_xhci_elpg_exit(tegra);
	if (ret) {
		dev_err(dev, "unable to perform elpg exit %d\n", ret);
		goto err;
	}

	mutex_unlock(&tegra->sync_lock);

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;
err:
	mutex_unlock(&tegra->sync_lock);
	return ret;
}

#ifdef CONFIG_PM_RUNTIME
static int tegra_xhci_runtime_suspend(struct device *dev)
{
	struct tegra_xhci_hcd *tegra = dev_get_drvdata(dev);
	struct xhci_hcd *xhci = tegra_to_xhci(tegra);
	int err = 0;

	/* Wait for port to enter U3 state */
	usleep_range(10000, 11000);

	mutex_lock(&tegra->sync_lock);
	WARN_ON(tegra->hc_in_elpg);

	err = xhci_suspend(xhci);
	if (err) {
		dev_err(dev, "xhci_suspend failed: %d\n", err);
		goto out;
	}

	/*
	 * Always enable wakeup for the PHY so that hotplug events may
	 * bring the host out of runtime suspend.
	 */
	device_init_wakeup(tegra->phy->dev, true);

	err = tegra_xhci_elpg_enter(tegra);
	if (err)
		dev_err(dev, "unable to perform elpg entry %d\n", err);

out:
	mutex_unlock(&tegra->sync_lock);
	return err;
}

static int tegra_xhci_runtime_resume(struct device *dev)
{
	struct tegra_xhci_hcd *tegra = dev_get_drvdata(dev);
	struct xhci_hcd *xhci = tegra_to_xhci(tegra);
	int err = 0;

	mutex_lock(&tegra->sync_lock);
	WARN_ON(!tegra->hc_in_elpg);
	err = tegra_xhci_elpg_exit(tegra);
	if (err) {
		dev_err(dev, "unable to perform elpg exit %d\n", err);
		goto out;
	}

	err = xhci_resume(xhci, 0);
	if (err)
		dev_err(dev, "xhci_resume failed %d\n", err);

out:
	mutex_unlock(&tegra->sync_lock);
	return err;
}
#endif /* CONFIG_PM_RUNTIME */
#endif /* CONFIG_PM */

static const struct dev_pm_ops tegra_xhci_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_xhci_suspend, tegra_xhci_resume)
	SET_RUNTIME_PM_OPS(tegra_xhci_runtime_suspend,
			   tegra_xhci_runtime_resume, NULL)
};

static struct platform_driver tegra_xhci_driver = {
	.probe	= tegra_xhci_probe,
	.remove	= tegra_xhci_remove,
	.driver	= {
		.name = "tegra-xhci",
		.of_match_table = of_match_ptr(tegra_xhci_of_match),
		.pm	= &tegra_xhci_dev_pm_ops,
	},
};
module_platform_driver(tegra_xhci_driver);
MODULE_ALIAS("platform:tegra-xhci");
