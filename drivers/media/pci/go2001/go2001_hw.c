/*
 *  go2001 - GO2001 codec driver.
 *
 *  Author : Pawel Osciak <posciak@chromium.org>
 *
 *  Copyright (C) 2015 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/delay.h>
#include <linux/firmware.h>

#include "go2001.h"
#include "go2001_hw.h"
#include "go2001_proto.h"

static int go2001_load_fw_image(struct go2001_dev *gdev,
				const struct firmware *fw,
				void __iomem *ctrl_addr, size_t ctrl_size,
				void __iomem *load_addr, size_t max_size,
				u32 entry_addr)
{
	struct go2001_boot_hdr hdr;
	u8 *chksum_ptr;
	size_t chksum_size;
	unsigned long flags;
	u32 reg;
	int i;

	if (fw->size > GO2001_FW_MAX_SIZE || fw->size > max_size) {
		go2001_err(gdev, "Firmware image too large (%zu)\n", fw->size);
		return -EINVAL;
	}

	if (ctrl_size < GO2001_FW_HDR_OFF + sizeof(hdr)
			|| ctrl_size < GO2001_FW_STOP + sizeof(u32)) {
		go2001_err(gdev, "PCI BAR sizes invalid\n");
		return -ENODEV;
	}

	hdr.signature = GO2001_FW_SIGNATURE;
	hdr.entry_addr = entry_addr;
	hdr.size = 16;
	hdr.checksum = 0;

	/*
	 * Calculate a checksum of the header (excluding checksum field itself).
	 */
	chksum_ptr = (u8 *)&hdr;
	chksum_size = offsetof(struct go2001_boot_hdr, checksum);
	for (i = 0; i < chksum_size; ++i)
		hdr.checksum += chksum_ptr[i];
	hdr.checksum &= 0xff;

	/* Stop the firmware via a soft interrupt. */
	writeb('p', ctrl_addr + GO2001_FW_HDR_OFF);
	reg = readl(ctrl_addr + GO2001_FW_STOP);
	reg |= GO2001_FW_STOP_BIT;
	writel(reg, ctrl_addr + GO2001_FW_STOP);

	/*
	 * The next interrupt after resuming will be the "firmware loaded"
	 * interrupt, clear the flag so we handle this on next IRQ.
	 */
	spin_lock_irqsave(&gdev->irqlock, flags);
	gdev->fw_loaded = false;
	spin_unlock_irqrestore(&gdev->irqlock, flags);

	/* Copy firmware and the header */
	memcpy_toio(load_addr, fw->data, fw->size);
	writel(hdr.entry_addr, ctrl_addr + GO2001_FW_HDR_OFF
			+ offsetof(struct go2001_boot_hdr, entry_addr));
	writel(hdr.size, ctrl_addr + GO2001_FW_HDR_OFF
			+ offsetof(struct go2001_boot_hdr, size));
	writel(hdr.checksum, ctrl_addr + GO2001_FW_HDR_OFF
			+ offsetof(struct go2001_boot_hdr, checksum));
	/*
	 * Write signature last, as this is what the device is spinning on
	 * while we are loading the firmware
	 */
	writel(hdr.signature, ctrl_addr + GO2001_FW_HDR_OFF
			+ offsetof(struct go2001_boot_hdr, signature));

	/* Clear the interrupt. */
	reg &= ~GO2001_FW_STOP_BIT;
	writel(reg, ctrl_addr + GO2001_FW_STOP);

	return 0;
}

static int go2001_load_firmware(struct go2001_dev *gdev)
{
	resource_size_t ctrl_start, fw_start;
	const struct firmware *boot_fw = NULL;
	const struct firmware *fw = NULL;
	void __iomem *fw_iomem;
	void __iomem *ctrl_iomem;
	size_t fw_iomem_size, ctrl_iomem_size;
	int time_spent = 0;
	int ret;

	init_completion(&gdev->fw_completion);

	ctrl_start = pci_resource_start(gdev->pdev, 2);
	ctrl_iomem_size = pci_resource_len(gdev->pdev, 2);
	if (!ctrl_start || !ctrl_iomem_size) {
		go2001_err(gdev, "PCI BAR 2 invalid\n");
		return -ENODEV;
	}

	fw_start = pci_resource_start(gdev->pdev, 0);
	fw_iomem_size = pci_resource_len(gdev->pdev, 0);
	if (!fw_start || !fw_iomem_size) {
		go2001_err(gdev, "PCI BAR 0 invalid\n");
		return -ENODEV;
	}

	if (ctrl_iomem_size < GO2001_BOOT_FW_OFF
				|| fw_iomem_size < GO2001_FW_OFF) {
		go2001_err(gdev, "Invalid PCI BAR sizes\n");
		return -EINVAL;
	}

	ctrl_iomem = ioremap_nocache(ctrl_start, ctrl_iomem_size);
	if (!ctrl_iomem) {
		go2001_err(gdev, "Failed mapping PCI BAR 2\n");
		return -ENOMEM;
	}

	fw_iomem = ioremap_nocache(fw_start, fw_iomem_size);
	if (!fw_iomem) {
		go2001_err(gdev, "Failed mapping PCI BAR 0\n");
		ret = -ENOMEM;
		goto out_unmap_ctrl_iomem;
	}

	ret = request_firmware(&boot_fw, GO2001_BOOT_FW_NAME, &gdev->pdev->dev);
	if (ret) {
		go2001_err(gdev, "Unable to open firmware %s\n",
				GO2001_BOOT_FW_NAME);
		goto out_unmap_fw_iomem;
	}

	ret = request_firmware(&fw, GO2001_FW_NAME, &gdev->pdev->dev);
	if (ret) {
		go2001_err(gdev, "Unable to open firmware %s\n",
				GO2001_FW_NAME);
		goto out_release_boot_fw;
	}

	ret = go2001_load_fw_image(gdev, boot_fw,
				ctrl_iomem, ctrl_iomem_size,
				ctrl_iomem + GO2001_BOOT_FW_OFF,
				ctrl_iomem_size - GO2001_BOOT_FW_OFF,
				GO2001_BOOT_FW_ENTRY_BASE + GO2001_BOOT_FW_OFF);
	if (ret) {
		go2001_err(gdev, "Failed loading firmware %s\n",
				GO2001_BOOT_FW_NAME);
		goto out_release_fw;
	}

	/*
	 * Boot firmware will change GO2001_FW_SIGNATURE to
	 * GO2001_FW_DONE_SIGNATURE when done.
	 */
	while (readl(ctrl_iomem + GO2001_FW_HDR_OFF) != GO2001_FW_DONE_SIGNATURE
			&& time_spent < GO2001_REPLY_TIMEOUT_MS) {
		mdelay(100);
		time_spent += 100;
	}

	if (time_spent >= GO2001_REPLY_TIMEOUT_MS) {
		go2001_err(gdev, "Timed out waiting for firmware to start\n");
		ret = -ENODEV;
		goto out_release_fw;
	}

	go2001_dbg(gdev, 1, "Firmware %s loaded\n", GO2001_BOOT_FW_NAME);

	ret = go2001_load_fw_image(gdev, fw,
				ctrl_iomem, ctrl_iomem_size,
				fw_iomem + GO2001_FW_OFF,
				fw_iomem_size - GO2001_FW_OFF,
				GO2001_FW_ENTRY_BASE + GO2001_FW_OFF);
	if (ret) {
		go2001_err(gdev, "Failed loading firmware %s\n",
				GO2001_FW_NAME);
		goto out_release_fw;
	}

	if (wait_for_completion_timeout(&gdev->fw_completion,
			msecs_to_jiffies(GO2001_REPLY_TIMEOUT_MS)) == 0) {
		go2001_err(gdev, "Timed out waiting for firmware\n");
		ret = -ENODEV;
		goto out_release_fw;
	}

	go2001_dbg(gdev, 1, "Firmware %s loaded\n", GO2001_FW_NAME);
out_release_fw:
	release_firmware(fw);
out_release_boot_fw:
	release_firmware(boot_fw);
out_unmap_fw_iomem:
	iounmap(fw_iomem);
out_unmap_ctrl_iomem:
	iounmap(ctrl_iomem);

	return ret;
}

int go2001_map_iomem(struct go2001_dev *gdev)
{
	resource_size_t start = pci_resource_start(gdev->pdev, 0);
	resource_size_t len = pci_resource_len(gdev->pdev, 0);

	if (!start || !len)
		return -ENODEV;
	if (len < GO2001_MSG_RING_MEM_OFFSET + GO2001_MSG_RING_MEM_SIZE)
		return -ENODEV;

	gdev->iomem_size = GO2001_MSG_RING_MEM_SIZE;
	gdev->iomem = ioremap(start + GO2001_MSG_RING_MEM_OFFSET,
				gdev->iomem_size);
	if (!gdev->iomem)
		return -ENOMEM;

	return 0;
}

void go2001_unmap_iomem(struct go2001_dev *gdev)
{
	iounmap(gdev->iomem);
}

void go2001_init_hw_inst(struct go2001_hw_inst *inst, u32 inst_id)
{
	inst->session_id = inst_id;
	inst->sequence_id = 0;
	INIT_LIST_HEAD(&inst->pending_list);
	INIT_LIST_HEAD(&inst->inst_entry);
}

static void go2001_release_hw_inst(struct go2001_dev *gdev,
				struct go2001_hw_inst *inst)
{
	unsigned long flags;

	spin_lock_irqsave(&gdev->irqlock, flags);
	WARN_ON(!list_empty(&inst->pending_list));
	go2001_cancel_hw_inst_locked(gdev, inst);
	spin_unlock_irqrestore(&gdev->irqlock, flags);
}

static int go2001_init_ring(struct go2001_dev *gdev,
			    struct go2001_msg_ring *ring,
			    off_t ring_desc_offset)
{
	struct go2001_msg_ring_desc *desc = &ring->desc;

	spin_lock_init(&ring->lock);

	ring->desc_iomem = gdev->iomem + ring_desc_offset;
	memcpy_fromio(desc, ring->desc_iomem, sizeof(*desc));
	go2001_dbg(gdev, 1, "Ring %p: [0x%x-0x%x], msg size: %d, capacity: %d,"
			  " wr_off: 0x%x, rd_off: 0x%x\n",
			ring->desc_iomem, desc->start_off,
			desc->end_off, desc->msg_size,
			(desc->end_off - desc->start_off) / desc->msg_size,
			desc->wr_off, desc->rd_off);

	if (WARN_ON(desc->msg_size == 0 || desc->end_off <= desc->start_off
			|| desc->end_off > gdev->iomem_size
			|| desc->wr_off != desc->rd_off)) {
		go2001_err(gdev, "Invalid ring descriptor\n");
		return -EIO;
	}

	if (WARN_ON(desc->msg_size > sizeof(struct go2001_msg))) {
		go2001_err(gdev, "Ring message size over limit\n");
		return -EIO;
	}

	return 0;
}

static int go2001_init_messaging(struct go2001_dev *gdev)
{
	int ret = 0;

	go2001_dbg(gdev, 1, "Initializing TX ring\n");
	ret = go2001_init_ring(gdev, &gdev->tx_ring,
				GO2001_TX_RING_DESC_OFFSET);
	if (ret)
		return ret;

	go2001_dbg(gdev, 1, "Initializing RX ring\n");
	ret = go2001_init_ring(gdev, &gdev->rx_ring,
				GO2001_RX_RING_DESC_OFFSET);
	if (ret)
		return ret;

	return 0;
}

static void go2001_update_ring_locked(struct go2001_msg_ring *r)
{
	r->desc.wr_off = ioread32(r->desc_iomem +
				offsetof(struct go2001_msg_ring_desc, wr_off));
	r->desc.rd_off = ioread32(r->desc_iomem +
				offsetof(struct go2001_msg_ring_desc, rd_off));
}

static inline bool go2001_ring_is_full_locked(struct go2001_msg_ring *r)
{
	u32 next_off;

	go2001_update_ring_locked(r);
	next_off = r->desc.wr_off + r->desc.msg_size;
	return (next_off == r->desc.rd_off
		|| (r->desc.rd_off == r->desc.start_off
			&& next_off >= r->desc.end_off));
}

static inline bool go2001_ring_is_empty_locked(struct go2001_msg_ring *r)
{
	go2001_update_ring_locked(r);
	return r->desc.rd_off == r->desc.wr_off;
}

static struct go2001_msg *prepare_msg(struct go2001_dev *gdev,
				enum go2001_msg_type type, size_t param_size)
{
	struct go2001_msg *msg;
	struct go2001_msg_hdr *hdr;

	msg = kmem_cache_zalloc(gdev->msg_cache, GFP_KERNEL);
	if (!msg) {
		go2001_err(gdev, "Failed allocating msg\n");
		return NULL;
	}

	INIT_LIST_HEAD(&msg->list_entry);
	hdr = msg_to_hdr(msg);
	hdr->type = type;
	hdr->size = go2001_calc_payload_size(param_size);
	return msg;
}

static void free_msg(struct go2001_dev *gdev, struct go2001_msg *msg)
{
	WARN_ON(!list_empty(&msg->list_entry));
	kmem_cache_free(gdev->msg_cache, msg);
}

static struct go2001_hw_inst *get_next_ready_hw_inst(struct go2001_dev *gdev)
{
	struct go2001_hw_inst *hw_inst;

	go2001_trace(gdev);

	if (list_empty(&gdev->inst_list))
		return NULL;

	if (!gdev->curr_hw_inst) {
		gdev->curr_hw_inst = list_first_entry(&gdev->inst_list,
					struct go2001_hw_inst, inst_entry);
	}

	hw_inst = gdev->curr_hw_inst;
	list_for_each_entry_continue(hw_inst, &gdev->inst_list, inst_entry) {
		if (!list_empty(&hw_inst->pending_list))
			return hw_inst;
	}

	list_for_each_entry(hw_inst, &gdev->inst_list, inst_entry) {
		if (!list_empty(&hw_inst->pending_list))
			return hw_inst;

		if (hw_inst == gdev->curr_hw_inst)
			break;
	}

	return NULL;
}

static struct go2001_msg *go2001_get_pending_msg_locked(struct go2001_dev *gdev)
{
	struct go2001_hw_inst *hw_inst;
	struct go2001_msg_hdr *hdr;
	struct go2001_msg *msg;

	assert_spin_locked(&gdev->irqlock);

	hw_inst = get_next_ready_hw_inst(gdev);
	if (!hw_inst) {
		go2001_dbg(gdev, 5, "No pending messages\n");
		return NULL;
	}

	gdev->curr_hw_inst = hw_inst;
	msg = list_first_entry(&hw_inst->pending_list, struct go2001_msg,
				list_entry);
	hdr = msg_to_hdr(msg);
	go2001_dbg(gdev, 5, "Found pending message for instance %d, seqid %d\n",
			hw_inst->session_id, hdr->sequence_id);
	list_del_init(&msg->list_entry);

	return msg;
}

static void go2001_drop_pending_for_hw_inst_locked(struct go2001_dev *gdev,
						struct go2001_hw_inst *hw_inst)
{
	struct go2001_msg *msg, *tmp_msg;
	assert_spin_locked(&gdev->irqlock);

	list_for_each_entry_safe(msg, tmp_msg, &hw_inst->pending_list,
					list_entry) {
		list_del_init(&msg->list_entry);
		free_msg(gdev, msg);
	}
	INIT_LIST_HEAD(&hw_inst->pending_list);
}

void go2001_cancel_hw_inst_locked(struct go2001_dev *gdev,
					struct go2001_hw_inst *hw_inst)
{
	assert_spin_locked(&gdev->irqlock);

	go2001_drop_pending_for_hw_inst_locked(gdev, hw_inst);
	list_del_init(&hw_inst->inst_entry);
	if (gdev->curr_hw_inst == hw_inst)
		gdev->curr_hw_inst = NULL;
}

void go2001_cancel_all_hw_inst_locked(struct go2001_dev *gdev)
{
	struct go2001_hw_inst *hw_inst;
	assert_spin_locked(&gdev->irqlock);

	list_for_each_entry(hw_inst, &gdev->inst_list, inst_entry)
		go2001_drop_pending_for_hw_inst_locked(gdev, hw_inst);
	INIT_LIST_HEAD(&gdev->inst_list);

	list_for_each_entry(hw_inst, &gdev->new_inst_list, inst_entry)
		go2001_drop_pending_for_hw_inst_locked(gdev, hw_inst);
	INIT_LIST_HEAD(&gdev->new_inst_list);

	gdev->curr_hw_inst = NULL;
}

static void go2001_print_msg(struct go2001_dev *gdev, struct go2001_msg *msg,
			const char *prefix)
{
	struct go2001_msg_hdr *hdr = msg_to_hdr(msg);
	go2001_dbg(gdev, 4,
			"%s: MSG: [%d/%d], type=0x%x, size=%d, status=0x%x\n",
			prefix, hdr->session_id, hdr->sequence_id, hdr->type,
			hdr->size, hdr->status);
	if (go2001_debug_level >= 5) {
		print_hex_dump(KERN_DEBUG, prefix, DUMP_PREFIX_NONE, 16, 1,
			msg_to_param(msg),
			hdr->size - sizeof(struct go2001_msg_hdr), false);
	}
}

static void go2001_print_mmap_list(struct go2001_dev *gdev,
				struct go2001_dma_desc *dma_desc)
{
	struct go2001_mmap_list_entry *mmap_list = dma_desc->mmap_list;
	int i;

	for (i = 0; i < dma_desc->num_entries; ++i) {
		go2001_dbg(gdev, 4, "entry %02d: addr: 0x%08llx, size: %d\n",
				i, mmap_list[i].dma_addr, mmap_list[i].size);
	}
}

static void go2001_send_pending_locked(struct go2001_dev *gdev)
{
	struct go2001_msg_ring *r = &gdev->tx_ring;
	struct go2001_msg *msg;
	struct go2001_msg_hdr *hdr;
	unsigned long flags;
	struct go2001_msg_ring_desc *desc = &r->desc;

	spin_lock_irqsave(&r->lock, flags);
	while (1) {
		if (go2001_ring_is_full_locked(r)) {
			go2001_dbg(gdev, 5, "TX ring full, not sending.\n");
			break;
		}

		msg = go2001_get_pending_msg_locked(gdev);
		if (!msg)
			break;
		hdr = msg_to_hdr(msg);

		memcpy_toio(r->desc_iomem + desc->wr_off, hdr, hdr->size);
		go2001_print_msg(gdev, msg, "TX: ");
		desc->wr_off += desc->msg_size;
		if (desc->wr_off >= desc->end_off)
			desc->wr_off = desc->start_off;

		iowrite32(desc->wr_off, r->desc_iomem
			  + offsetof(struct go2001_msg_ring_desc, wr_off));

		if (gdev->msgs_in_flight == 0) {
			schedule_delayed_work(&gdev->watchdog_work,
				msecs_to_jiffies(GO2001_WATCHDOG_TIMEOUT_MS));
		}
		++gdev->msgs_in_flight;
		go2001_dbg(gdev, 5, "Messages in flight: %d\n",
				gdev->msgs_in_flight);

		free_msg(gdev, msg);
	}
	spin_unlock_irqrestore(&r->lock, flags);
}

void go2001_send_pending(struct go2001_dev *gdev)
{
	unsigned long flags;

	spin_lock_irqsave(&gdev->irqlock, flags);
	go2001_send_pending_locked(gdev);
	spin_unlock_irqrestore(&gdev->irqlock, flags);
}

/*
 * NOTE: msg must not be used by the caller after this functions returns;
 * either we'll free it after sending it to HW, or it will be cleaned up by the
 * watchdog on timeout (if still on the pending_list at that time).
 */
static u32 go2001_queue_msg_locked(struct go2001_dev *gdev,
			struct go2001_hw_inst *hw_inst, struct go2001_msg *msg)
{
	struct go2001_msg_hdr *hdr = msg_to_hdr(msg);
	u32 seqid;

	assert_spin_locked(&gdev->irqlock);
	hdr->session_id = hw_inst->session_id;
	seqid = ++hw_inst->sequence_id;
	hdr->sequence_id = seqid;
	list_add_tail(&msg->list_entry, &hw_inst->pending_list);
	return seqid;
}

static u32 go2001_queue_msg(struct go2001_ctx *ctx, struct go2001_msg *msg)
{
	struct go2001_dev *gdev = ctx->gdev;
	unsigned long flags;
	u32 type = msg_to_hdr(msg)->type;
	u32 seqid;

	spin_lock_irqsave(&gdev->irqlock, flags);
	seqid = go2001_queue_msg_locked(gdev, &ctx->hw_inst, msg);
	spin_unlock_irqrestore(&gdev->irqlock, flags);
	go2001_dbg(gdev, 5, "Queued message type 0x%x for inst %d, seqid: %d\n",
			type, ctx->hw_inst.session_id, seqid);

	/* The message is freed after being sent to hardware or on error. */
	go2001_send_pending(gdev);
	return seqid;
}

static struct go2001_hw_inst *find_hw_inst_by_id_locked(struct go2001_dev *gdev,
							u32 session_id)
{
	struct go2001_hw_inst *hw_inst;

	list_for_each_entry(hw_inst, &gdev->inst_list, inst_entry) {
		if (hw_inst->session_id == session_id)
			return hw_inst;
	}

	return NULL;
}

static bool go2001_sequence_passed(struct go2001_dev *gdev,
				   u32 session_id, u32 seq_id)
{
	struct go2001_hw_inst *hw_inst;
	unsigned long flags;
	bool ret = true;

	spin_lock_irqsave(&gdev->irqlock, flags);
	hw_inst = find_hw_inst_by_id_locked(gdev, session_id);
	if (hw_inst)
		ret = (hw_inst->last_reply_seq_id >= seq_id);
	spin_unlock_irqrestore(&gdev->irqlock, flags);
	return ret;
}

static int go2001_wait_for_reply(struct go2001_dev *gdev, u32 session_id,
					u32 sequence_id)
{
	int ret;

	ret = wait_event_timeout(gdev->reply_wq,
			go2001_sequence_passed(gdev, session_id, sequence_id),
			msecs_to_jiffies(GO2001_REPLY_TIMEOUT_MS));
	if (ret == 0) {
		go2001_err(gdev, "Timeout waiting for reply to %d/%d\n",
				session_id, sequence_id);
		return -ETIMEDOUT;
	}

	go2001_dbg(gdev, 5, "Finished waiting for %d/%d\n",
			session_id, sequence_id);

	return 0;
}

static inline bool go2001_ctx_idle(struct go2001_ctx *ctx)
{
	unsigned long flags1, flags2;
	bool idle;

	spin_lock_irqsave(&ctx->gdev->irqlock, flags1);
	spin_lock_irqsave(&ctx->qlock, flags2);
	idle = list_empty(&ctx->hw_inst.pending_list) && !ctx->job.src_buf;
	spin_unlock_irqrestore(&ctx->qlock, flags2);
	spin_unlock_irqrestore(&ctx->gdev->irqlock, flags1);

	return idle;
}

void go2001_wait_for_ctx_done(struct go2001_ctx *ctx)
{
	/* No timeout, the watchdog will wake us up if needed. */
	wait_event(ctx->gdev->reply_wq, go2001_ctx_idle(ctx));
	go2001_wait_for_reply(ctx->gdev, ctx->hw_inst.session_id,
				ctx->hw_inst.sequence_id);
}

static int go2001_queue_msg_and_wait(struct go2001_ctx *ctx,
					struct go2001_msg *msg)
{
	u32 session_id = ctx->hw_inst.session_id;
	u32 seqid;

	/* msg becomes invalid after the call to go2001_queue_msg. */
	seqid = go2001_queue_msg(ctx, msg);

	return go2001_wait_for_reply(ctx->gdev, session_id, seqid);
}

static int go2001_queue_ctrl_msg_and_wait(struct go2001_dev *gdev,
						struct go2001_msg *msg)
{
	unsigned long flags;
	u32 seqid;

	spin_lock_irqsave(&gdev->irqlock, flags);
	seqid = go2001_queue_msg_locked(gdev, &gdev->ctrl_inst, msg);
	spin_unlock_irqrestore(&gdev->irqlock, flags);
	go2001_dbg(gdev, 5, "Queued control msg seqid: %d\n", seqid);

	go2001_send_pending(gdev);
	return go2001_wait_for_reply(gdev, 0, seqid);
}

static int go2001_queue_init_msg_and_wait(struct go2001_ctx *ctx,
						struct go2001_msg *msg)
{
	struct go2001_dev *gdev = ctx->gdev;
	unsigned long flags;
	u32 seqid;

	spin_lock_irqsave(&gdev->irqlock, flags);
	seqid = go2001_queue_msg_locked(gdev, &gdev->ctrl_inst, msg);
	if (ctx && !WARN_ON(!list_empty(&ctx->hw_inst.inst_entry))) {
		go2001_dbg(gdev, 5, "New inst seq %d\n", seqid);
		list_add_tail(&ctx->hw_inst.inst_entry, &gdev->new_inst_list);
		/* Will use sequence_id to check if we got the
		 * reply for the correct new ctx in IRQ.
		 */
		ctx->hw_inst.sequence_id = seqid;
	}
	spin_unlock_irqrestore(&gdev->irqlock, flags);

	go2001_send_pending(gdev);
	return go2001_wait_for_reply(gdev, 0, seqid);
}

int go2001_get_reply(struct go2001_dev *gdev, struct go2001_msg *msg)
{
	struct go2001_msg_ring *r = &gdev->rx_ring;
	struct go2001_msg_hdr *hdr = msg_to_hdr(msg);
	struct go2001_msg_ring_desc *desc = &r->desc;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&r->lock, flags);
	if (go2001_ring_is_empty_locked(r)) {
		go2001_dbg(gdev, 5, "Ring empty\n");
		ret = -EAGAIN;
		goto out;
	}

	memcpy_fromio(hdr, r->desc_iomem + desc->rd_off, desc->msg_size);
	go2001_print_msg(gdev, msg, "RX: ");

	desc->rd_off += desc->msg_size;
	if (desc->rd_off >= desc->end_off)
		desc->rd_off = desc->start_off;

	iowrite32(desc->rd_off, r->desc_iomem
			+ offsetof(struct go2001_msg_ring_desc, rd_off));
out:
	spin_unlock_irqrestore(&r->lock, flags);
	return ret;
}

static int go2001_init_decoder(struct go2001_ctx *ctx)
{
	struct go2001_dev *gdev = ctx->gdev;
	struct go2001_init_decoder_param *param;
	struct go2001_msg *msg;
	unsigned long flags;
	int ret;

	if (!ctx->src_fmt) {
		go2001_err(gdev, "Input format not set\n");
		return -EINVAL;
	}

	go2001_dbg(gdev, 2, "Initializing decoder for format %s\n",
			ctx->src_fmt->desc);

	msg = prepare_msg(gdev, GO2001_VM_INIT_DECODER, sizeof(*param));
	if (!msg)
		return -ENOMEM;

	param = msg_to_param(msg);
	param->coded_fmt = ctx->src_fmt->hw_format;
	param->concealment = 0;

	ret = go2001_queue_init_msg_and_wait(ctx, msg);
	spin_lock_irqsave(&ctx->qlock, flags);
	if (ret || ctx->state == ERROR) {
		go2001_err(gdev, "Failed initializing decoder\n");
		ret = ret ? ret : -EIO;
	}
	spin_unlock_irqrestore(&ctx->qlock, flags);

	return ret;
}

static int go2001_s_ctrl(struct go2001_ctx *ctx, enum go2001_hw_ctrl_type type,
				union go2001_hw_ctrl *ctrl)
{
	struct go2001_set_ctrl_param *param;
	struct go2001_msg *msg;

	msg = prepare_msg(ctx->gdev, GO2001_VM_SET_CTRL, sizeof(*param));
	if (!msg)
		return -ENOMEM;

	param = msg_to_param(msg);
	param->type = type;

	memcpy(&param->ctrl, ctrl, sizeof(*ctrl));

	return go2001_queue_msg_and_wait(ctx, msg);
}

static int go2001_set_def_encoder_ctrls(struct go2001_ctx *ctx)
{
	union go2001_hw_ctrl hw_ctrl;
	struct go2001_enc_coding_ctrl *ctrl = &hw_ctrl.coding_ctrl;

	memset(&hw_ctrl, 0, sizeof(hw_ctrl));
	ctrl->interp_filter_type = GO2001_CODING_CTRL_INTERP_FILTER_BICUBIC;
	ctrl->deblock_filter_type = GO2001_CODING_CTRL_DEBLOCK_FILTER_NORMAL;
	ctrl->deblock_filter_level = 64;
	ctrl->deblock_filter_sharpness = 8;
	ctrl->num_dct_parts = 1;
	ctrl->error_resilient = 1;
	ctrl->split_mv = GO2001_CODING_CTRL_MV_ADAPTIVE;
	ctrl->quarter_pixel_mv = GO2001_CODING_CTRL_MV_ADAPTIVE;
	ctrl->deadzone_enabled = 1;
	ctrl->max_num_passes = 1;
	ctrl->quality_metric = GO2001_CODING_CTRL_QM_SSIM;

	return go2001_s_ctrl(ctx, GO2001_HW_CTRL_TYPE_CODING, &hw_ctrl);
}

static int go2001_init_encoder(struct go2001_ctx *ctx)
{
	struct go2001_dev *gdev = ctx->gdev;
	struct go2001_init_encoder_param *param;
	struct go2001_frame_info *finfo;
	struct go2001_msg *msg;
	int ret;

	if (WARN_ON(!ctx->src_fmt || !ctx->dst_fmt
			|| !go2001_has_frame_info(ctx))) {
		go2001_dbg(ctx->gdev, 2, "Formats not set\n");
		return -EINVAL;
	}

	go2001_dbg(gdev, 2,
			"Initializing encoder for formats %s->%s at %dx%d\n",
			ctx->src_fmt->desc, ctx->dst_fmt->desc,
			ctx->finfo.width, ctx->finfo.height);

	msg = prepare_msg(gdev, GO2001_VM_INIT_ENCODER, sizeof(*param));
	if (!msg)
		return -ENOMEM;

	param = msg_to_param(msg);
	finfo = &ctx->finfo;

	param->session_id = 0;
	param->num_ref_frames = ctx->enc_params.multi_ref_frame_mode ? 1 : 3;
	param->width = finfo->width;
	param->height = finfo->height;
	param->orig_width = finfo->width;
	param->orig_height = finfo->height;
	param->framerate_num = ctx->enc_params.framerate_num;
	param->framerate_denom = ctx->enc_params.framerate_denom;
	param->raw_fmt = ctx->src_fmt->hw_format;

	ret = go2001_queue_init_msg_and_wait(ctx, msg);
	if (ret || ctx->state == ERROR) {
		go2001_err(gdev, "Failed initializing encoder\n");
		return ret ? ret : -EIO;
	}

	return go2001_set_def_encoder_ctrls(ctx);
}

void go2001_release_codec(struct go2001_ctx *ctx)
{
	struct go2001_dev *gdev = ctx->gdev;
	struct go2001_msg *msg;

	go2001_dbg(gdev, 2, "Releasing session id %d, codec state %d\n",
			ctx->hw_inst.session_id, ctx->state);
	if (ctx->hw_inst.session_id != 0) {
		msg = prepare_msg(gdev, GO2001_VM_RELEASE, 0);
		if (msg)
			go2001_queue_msg_and_wait(ctx, msg);
	}
	/* Regardless of whether we sent and/or got a reply or not, clean up. */
	go2001_release_hw_inst(gdev, &ctx->hw_inst);
}

int go2001_map_buffer(struct go2001_ctx *ctx, struct go2001_buffer *buf)
{
	struct go2001_dev *gdev = ctx->gdev;
	struct device *dev = &gdev->pdev->dev;
	struct go2001_set_mmap_param *param;
	struct vb2_buffer *vb = &buf->vb;
	struct go2001_dma_desc *dma_desc;
	struct go2001_msg *msg;
	struct go2001_mmap_list_desc *list_desc;
	unsigned int i;

	if (buf->mapped) {
		go2001_err(gdev, "Buffer already mapped\n");
		return -EINVAL;
	}

	if (vb->num_planes > GO2001_MMAP_MAX_ENTRIES) {
		go2001_err(gdev, "Too many planes to map\n");
		return -EIO;
	}

	msg = prepare_msg(gdev, GO2001_VM_SET_MMAP, sizeof(*param));
	if (!msg)
		return -ENOMEM;
	param = msg_to_param(msg);
	param->dir = V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)
		   ? GO2001_VSM_DIR_IN : GO2001_VSM_DIR_OUT;
	param->count = vb->num_planes;
	for (i = 0; i < vb->num_planes; ++i) {
		dma_desc = &buf->dma_desc[i];
		list_desc = &param->mmap_list_desc[i];

		dma_desc->map_addr = dma_desc->mmap_list[0].dma_addr;

		list_desc->first_entry_dma_addr = dma_desc->map_addr;
		list_desc->entry_count = dma_desc->num_entries;
		list_desc->mmap_list_addr = dma_desc->dma_addr;

		go2001_dbg(gdev, 4,
			"Mapping plane %u: 0x%08llx, chunks: %d in HW\n",
			i, dma_desc->map_addr, dma_desc->num_entries);
		go2001_print_mmap_list(gdev, dma_desc);
		dma_cache_sync(dev, dma_desc->mmap_list, dma_desc->list_size,
				DMA_TO_DEVICE);
	}

	go2001_queue_msg(ctx, msg);

	buf->mapped = true;
	return 0;
}

void go2001_unmap_buffer(struct go2001_ctx *ctx, struct go2001_buffer *buf)
{
	struct go2001_release_mmap_param *param;
	struct go2001_dev *gdev = ctx->gdev;
	struct vb2_buffer *vb = &buf->vb;
	struct go2001_msg *msg;
	unsigned int i;

	if (!buf->mapped) {
		go2001_dbg(gdev, 1, "Buffer not mapped\n");
		return;
	}

	if (vb->num_planes > GO2001_MMAP_MAX_ENTRIES) {
		go2001_dbg(gdev, 1, "Too many planes\n");
		return;
	}

	go2001_dbg(gdev, 3, "Unmapping buffer %p from HW\n", buf);

	msg = prepare_msg(gdev, GO2001_VM_RELEASE_MMAP, sizeof(*param));
	if (!msg)
		return;
	param = msg_to_param(msg);
	param->dir = V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)
		   ? GO2001_VSM_DIR_IN : GO2001_VSM_DIR_OUT;
	param->count = vb->num_planes;
	for (i = 0; i < vb->num_planes; ++i) {
		param->addr[i] = buf->dma_desc[i].map_addr;
		go2001_dbg(gdev, 4, "Unmapping plane %u, 0x%08llx from HW\n",
				i, buf->dma_desc[i].map_addr);
	}

	go2001_queue_msg_and_wait(ctx, msg);

	buf->mapped = false;
}

int go2001_unmap_buffers(struct go2001_ctx *ctx, bool unmap_src, bool unmap_dst)
{
	struct go2001_release_mmap_param *param;
	struct go2001_msg *msg;

	if (WARN_ON(!unmap_src && !unmap_dst))
		return 0;

	msg = prepare_msg(ctx->gdev, GO2001_VM_RELEASE_MMAP, sizeof(*param));
	if (!msg)
		return -ENOMEM;
	param = msg_to_param(msg);
	param->dir |= (unmap_src ? GO2001_VSM_DIR_IN : 0);
	param->dir |= (unmap_dst ? GO2001_VSM_DIR_OUT : 0);

	return go2001_queue_msg_and_wait(ctx, msg);
}

static int go2001_build_dec_msg(struct go2001_ctx *ctx, struct go2001_msg *msg,
		struct go2001_buffer *src_buf, struct go2001_buffer *dst_buf)
{
	struct go2001_dev *gdev = ctx->gdev;
	struct go2001_empty_buffer_dec_param *param;
	int i;

	assert_spin_locked(&ctx->qlock);

	if (!src_buf->mapped || (dst_buf && !dst_buf->mapped)) {
		go2001_err(gdev, "Buffer(s) not mapped!\n");
		return -EINVAL;
	}

	param = msg_to_param(msg);

	param->in_addr = src_buf->dma_desc[0].map_addr;
	param->payload_size = vb2_get_plane_payload(&src_buf->vb, 0);
	if (dst_buf) {
		if (WARN_ON(dst_buf->vb.num_planes >
					ARRAY_SIZE(param->out_addr)))
			return -EINVAL;

		for (i = 0; i < dst_buf->vb.num_planes; ++i) {
			param->out_addr[i] = dst_buf->dma_desc[i].map_addr;
			WARN_ON(!IS_ALIGNED(param->out_addr[i], 16));
		}

		if (!ctx->dst_fmt) {
			go2001_err(gdev, "Destination format not set\n");
			return -EINVAL;
		}
		param->raw_fmt = ctx->dst_fmt->hw_format;
	}

	param->flags = ctx->need_resume ?
			G02001_EMPTY_BUF_DEC_FLAG_RES_CHANGE_DONE : 0;
	ctx->need_resume = false;

	return 0;
}

static void go2001_update_enc_params(struct go2001_ctx *ctx,
			struct go2001_runtime_enc_params *rt_enc_params) {
	unsigned long *changed_mask = rt_enc_params->changed_mask;
	struct go2001_enc_params *new_params = &rt_enc_params->enc_params;

	if (changed_mask) {
		if (test_and_clear_bit(GO2001_BITRATE_CHANGE, changed_mask))
			ctx->enc_params.bitrate = new_params->bitrate;

		if (test_and_clear_bit(GO2001_FRAMERATE_CHANGE, changed_mask)) {
			ctx->enc_params.framerate_num =
				new_params->framerate_num;
			ctx->enc_params.framerate_denom =
				new_params->framerate_denom;
		}

		if (test_and_clear_bit(GO2001_KEYFRAME_REQUESTED,
					changed_mask)) {
			ctx->enc_params.request_keyframe =
				new_params->request_keyframe;
		}
	}
}

static int go2001_build_enc_msg(struct go2001_ctx *ctx, struct go2001_msg *msg,
		struct go2001_buffer *src_buf, struct go2001_buffer *dst_buf)
{
	struct go2001_empty_buffer_enc_param *param;
	struct go2001_dev *gdev = ctx->gdev;
	int i;

	BUG_ON(!src_buf || !dst_buf);
	assert_spin_locked(&ctx->qlock);

	if (!src_buf->mapped || !dst_buf->mapped) {
		go2001_err(gdev, "Buffer(s) not mapped!\n");
		return -EINVAL;
	}

	param = msg_to_param(msg);

	if (WARN_ON(src_buf->vb.num_planes > ARRAY_SIZE(param->in_addr)))
		return -EINVAL;

	for (i = 0; i < src_buf->vb.num_planes; ++i) {
		param->in_addr[i] = src_buf->dma_desc[i].map_addr;
		WARN_ON(!IS_ALIGNED(param->in_addr[i], 16));
	}
	param->out_addr = dst_buf->dma_desc[0].map_addr;
	param->out_size = vb2_plane_size(&dst_buf->vb, 0);

	go2001_update_enc_params(ctx, &src_buf->rt_enc_params);
	if (ctx->enc_params.request_keyframe) {
		param->frame_type = GO2001_EMPTY_BUF_ENC_FRAME_KEYFRAME;
		ctx->enc_params.frames_since_intra = 0;
		ctx->enc_params.request_keyframe = false;
	} else {
		param->frame_type = GO2001_EMPTY_BUF_ENC_FRAME_PRED;
	}

	if (WARN_ON(ctx->enc_params.framerate_num == 0))
		return -EINVAL;

	param->time_increment = GO2001_MAX_FPS / ctx->enc_params.framerate_num;
	param->bits_per_sec = ctx->enc_params.bitrate;
	ctx->enc_params.bitrate = 0;

	if (!ctx->enc_params.multi_ref_frame_mode ||
	    ctx->enc_params.frames_since_intra % 4 == 0) {
		/* Frame controls for temporal layer 0. */
		param->ipf_frame_ctrl = GO2001_FRM_CTRL_REFERENCE_AND_REFRESH;
		param->grf_frame_ctrl = GO2001_FRM_CTRL_NO_REFRESH;
		param->arf_frame_ctrl = GO2001_FRM_CTRL_NO_REFRESH;
	} else if (ctx->enc_params.frames_since_intra % 2 == 0) {
		/* Frame controls for temporal layer 1. */
		param->ipf_frame_ctrl = GO2001_FRM_CTRL_REFERENCE;
		param->grf_frame_ctrl = GO2001_FRM_CTRL_REFERENCE_AND_REFRESH;
		param->arf_frame_ctrl = GO2001_FRM_CTRL_NO_REFRESH;
	} else {
		/* Frame controls for temporal layer 2. */
		param->ipf_frame_ctrl = GO2001_FRM_CTRL_REFERENCE;
		param->grf_frame_ctrl = GO2001_FRM_CTRL_REFERENCE;
		param->arf_frame_ctrl = GO2001_FRM_CTRL_REFERENCE_AND_REFRESH;
	}
	ctx->enc_params.frames_since_intra++;

	return 0;
}

static void go2001_flush(struct go2001_ctx *ctx, struct go2001_buffer *src_buf)
{
	assert_spin_locked(&ctx->qlock);

	go2001_dbg(ctx->gdev, 4, "Flushing at src_buf ts=%ld.%06ld\n",
			src_buf->vb.v4l2_buf.timestamp.tv_sec,
			src_buf->vb.v4l2_buf.timestamp.tv_usec);

	/*
	 * Since VPX has no frame reordering and buffers are returned as
	 * soon as they are decoded, there is no need trigger anything on the
	 * destination queue. Just return the flush buffer to userspace.
	 */
	list_del(&src_buf->list);
	vb2_buffer_done(&src_buf->vb, VB2_BUF_STATE_DONE);
}

int go2001_prepare_gbuf(struct go2001_ctx *ctx, struct go2001_buffer *gbuf,
			bool is_src)
{
	struct go2001_msg *msg;
	size_t size;

	if (!is_src)
		return 0;

	if (WARN_ON(gbuf->msg))
		return -EINVAL;

	if (ctx->codec_mode == CODEC_MODE_DECODER)
		size = sizeof(struct go2001_empty_buffer_dec_param);
	else
		size = sizeof(struct go2001_empty_buffer_enc_param);

	msg = prepare_msg(ctx->gdev, GO2001_VM_EMPTY_BUFFER, size);
	if (!msg)
		return -ENOMEM;

	gbuf->msg = msg;
	return 0;
}

void go2001_finish_gbuf(struct go2001_ctx *ctx, struct go2001_buffer *gbuf)
{
	if (gbuf->msg) {
		free_msg(ctx->gdev, gbuf->msg);
		gbuf->msg = NULL;
	}
}

int go2001_schedule_frames(struct go2001_ctx *ctx)
{
	struct go2001_dev *gdev = ctx->gdev;
	struct go2001_job *job = &ctx->job;
	struct go2001_msg *msg = NULL;
	struct go2001_buffer *src_buf;
	struct go2001_buffer *dst_buf;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&ctx->qlock, flags);
	if (job->src_buf) {
		go2001_dbg(gdev, 5, "Job already running\n");
		goto out;
	}
	BUG_ON(job->dst_buf);

	go2001_dbg(gdev, 5, "State: %d\n", ctx->state);
again:
	src_buf = NULL;
	dst_buf = NULL;

	switch (ctx->state) {
	case NEED_HEADER_INFO:
		BUG_ON(ctx->codec_mode == CODEC_MODE_ENCODER);
		if (!list_empty(&ctx->src_buf_q)) {
			src_buf = list_first_entry(&ctx->src_buf_q,
						struct go2001_buffer, list);
		}
		break;

	case RUNNING:
		if (!list_empty(&ctx->src_buf_q)
				&& !list_empty(&ctx->dst_buf_q)) {
			src_buf = list_first_entry(&ctx->src_buf_q,
						struct go2001_buffer, list);
			dst_buf = list_first_entry(&ctx->dst_buf_q,
						struct go2001_buffer, list);
		}
		break;

	default:
		break;
	}

	if (!src_buf)
		goto out;

	msg = src_buf->msg;
	if (WARN_ON(!msg))
		goto out;

	if (ctx->codec_mode == CODEC_MODE_DECODER) {
		if (vb2_get_plane_payload(&src_buf->vb, 0) == 0) {
			/* Flush buffer */
			go2001_flush(ctx, src_buf);
			goto again;
		}

		ret = go2001_build_dec_msg(ctx, msg, src_buf, dst_buf);
	} else {
		ret = go2001_build_enc_msg(ctx, msg, src_buf, dst_buf);
	}
	if (ret)
		goto out;

	job->src_buf = src_buf;
	if (dst_buf)
		job->dst_buf = dst_buf;

	src_buf->msg = NULL;
	spin_unlock_irqrestore(&ctx->qlock, flags);

	go2001_queue_msg(ctx, msg);
	return 0;
out:
	if (ret) {
		go2001_set_ctx_state(ctx, ERROR);
		go2001_err(gdev, "Error scheduling frames\n");
	}

	spin_unlock_irqrestore(&ctx->qlock, flags);
	return ret;
}

static int go2001_query_hw_version(struct go2001_dev *gdev)
{
	struct go2001_msg *msg;

	msg = prepare_msg(gdev, GO2001_VM_GET_VERSION, 0);
	if (!msg)
		return -ENOMEM;

	return go2001_queue_ctrl_msg_and_wait(gdev, msg);
}

static int go2001_set_log_level(struct go2001_dev *gdev, u32 level)
{
	struct go2001_set_log_level_param *param;
	struct go2001_msg *msg;

	if (level != GO2001_LOG_LEVEL_DISABLED && level > GO2001_LOG_LEVEL_MAX)
		level = GO2001_LOG_LEVEL_MAX;

	msg = prepare_msg(gdev, GO2001_VM_SET_LOG_LEVEL, sizeof(*param));
	if (!msg)
		return -ENOMEM;
	param = msg_to_param(msg);
	param->level = level;

	return go2001_queue_ctrl_msg_and_wait(gdev, msg);
}

int go2001_init_codec(struct go2001_ctx *ctx)
{
	int ret = 0;

	if (WARN_ON(ctx->hw_inst.session_id != 0))
		return -EINVAL;

	if (ctx->codec_mode == CODEC_MODE_DECODER)
		ret = go2001_init_decoder(ctx);
	else
		ret = go2001_init_encoder(ctx);

	return ret;
}

int go2001_init(struct go2001_dev *gdev)
{
	int ret;

	WARN_ON(!mutex_is_locked(&gdev->lock));

	ret = go2001_load_firmware(gdev);
	if (ret) {
		go2001_err(gdev, "Failed loading firmware\n");
		return ret;
	}

	ret = go2001_init_messaging(gdev);
	if (ret) {
		go2001_err(gdev, "Failed to init messaging\n");
		return ret;
	}

	ret = go2001_query_hw_version(gdev);
	if (ret) {
		go2001_err(gdev, "Failed querying HW version\n");
		return ret;
	}

	ret = go2001_set_log_level(gdev, go2001_fw_debug_level);
	if (ret) {
		go2001_err(gdev, "Failed setting log level\n");
		return ret;
	}

	return 0;
}
