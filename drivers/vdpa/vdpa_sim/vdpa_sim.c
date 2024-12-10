// SPDX-License-Identifier: GPL-2.0-only
/*
 * VDPA networking device simulator.
 *
 * Copyright (c) 2020, Red Hat Inc. All rights reserved.
 *     Author: Jason Wang <jasowang@redhat.com>
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uuid.h>
#include <linux/iommu.h>
#include <linux/dma-map-ops.h>
#include <linux/sysfs.h>
#include <linux/file.h>
#include <linux/etherdevice.h>
#include <linux/vringh.h>
#include <linux/vdpa.h>
#include <linux/virtio_byteorder.h>
#include <linux/vhost_iotlb.h>
#include <uapi/linux/virtio_config.h>
#include <uapi/linux/virtio_net.h>

#define DRV_VERSION  "0.1"
#define DRV_AUTHOR   "Jason Wang <jasowang@redhat.com>"
#define DRV_DESC     "vDPA Device Simulator"
#define DRV_LICENSE  "GPL v2"

static int batch_mapping = 1;
module_param(batch_mapping, int, 0444);
MODULE_PARM_DESC(batch_mapping, "Batched mapping 1 -Enable; 0 - Disable");

static char *macaddr;
module_param(macaddr, charp, 0);
MODULE_PARM_DESC(macaddr, "Ethernet MAC address");

u8 macaddr_buf[ETH_ALEN];

struct vdpasim_virtqueue {
	struct vringh vring;
	struct vringh_kiov iov;
	unsigned short head;
	bool ready;
	u64 desc_addr;
	u64 device_addr;
	u64 driver_addr;
	u32 num;
	void *private;
	irqreturn_t (*cb)(void *data);
};

#define VDPASIM_QUEUE_ALIGN PAGE_SIZE
#define VDPASIM_QUEUE_MAX 256
#define VDPASIM_DEVICE_ID 0x1
#define VDPASIM_VENDOR_ID 0
#define VDPASIM_VQ_NUM 0x2
#define VDPASIM_NAME "vdpasim-netdev"

static u64 vdpasim_features = (1ULL << VIRTIO_F_ANY_LAYOUT) |
			      (1ULL << VIRTIO_F_VERSION_1)  |
			      (1ULL << VIRTIO_F_ACCESS_PLATFORM) |
			      (1ULL << VIRTIO_NET_F_MAC);

struct vdpasim;

struct vdpasim_dev_attr {
	size_t config_size;
	int nvqs;
	void (*get_config)(struct vdpasim *vdpasim, void *config);
};

/* State of each vdpasim device */
struct vdpasim {
	struct vdpa_device vdpa;
	struct vdpasim_virtqueue *vqs;
	struct work_struct work;
	struct vdpasim_dev_attr dev_attr;
	/* spinlock to synchronize virtqueue state */
	spinlock_t lock;
	/* virtio config according to device type */
	void *config;
	struct vhost_iotlb *iommu;
	void *buffer;
	u32 status;
	u32 generation;
	u64 features;
	/* spinlock to synchronize iommu table */
	spinlock_t iommu_lock;
};

/* TODO: cross-endian support */
static inline bool vdpasim_is_little_endian(struct vdpasim *vdpasim)
{
	return virtio_legacy_is_little_endian() ||
		(vdpasim->features & (1ULL << VIRTIO_F_VERSION_1));
}

static inline u16 vdpasim16_to_cpu(struct vdpasim *vdpasim, __virtio16 val)
{
	return __virtio16_to_cpu(vdpasim_is_little_endian(vdpasim), val);
}

static inline __virtio16 cpu_to_vdpasim16(struct vdpasim *vdpasim, u16 val)
{
	return __cpu_to_virtio16(vdpasim_is_little_endian(vdpasim), val);
}

static struct vdpasim *vdpasim_dev;

static struct vdpasim *vdpa_to_sim(struct vdpa_device *vdpa)
{
	return container_of(vdpa, struct vdpasim, vdpa);
}

static struct vdpasim *dev_to_sim(struct device *dev)
{
	struct vdpa_device *vdpa = dev_to_vdpa(dev);

	return vdpa_to_sim(vdpa);
}

static void vdpasim_queue_ready(struct vdpasim *vdpasim, unsigned int idx)
{
	struct vdpasim_virtqueue *vq = &vdpasim->vqs[idx];

	vringh_init_iotlb(&vq->vring, vdpasim_features,
			  VDPASIM_QUEUE_MAX, false,
			  (struct vring_desc *)(uintptr_t)vq->desc_addr,
			  (struct vring_avail *)
			  (uintptr_t)vq->driver_addr,
			  (struct vring_used *)
			  (uintptr_t)vq->device_addr);
}

static void vdpasim_vq_reset(struct vdpasim_virtqueue *vq)
{
	vq->ready = false;
	vq->desc_addr = 0;
	vq->driver_addr = 0;
	vq->device_addr = 0;
	vq->cb = NULL;
	vq->private = NULL;
	vringh_init_iotlb(&vq->vring, vdpasim_features, VDPASIM_QUEUE_MAX,
			  false, NULL, NULL, NULL);
}

static void vdpasim_reset(struct vdpasim *vdpasim)
{
	int i;

	for (i = 0; i < vdpasim->dev_attr.nvqs; i++)
		vdpasim_vq_reset(&vdpasim->vqs[i]);

	spin_lock(&vdpasim->iommu_lock);
	vhost_iotlb_reset(vdpasim->iommu);
	spin_unlock(&vdpasim->iommu_lock);

	vdpasim->features = 0;
	vdpasim->status = 0;
	++vdpasim->generation;
}

static void vdpasim_work(struct work_struct *work)
{
	struct vdpasim *vdpasim = container_of(work, struct
						 vdpasim, work);
	struct vdpasim_virtqueue *txq = &vdpasim->vqs[1];
	struct vdpasim_virtqueue *rxq = &vdpasim->vqs[0];
	ssize_t read, write;
	size_t total_write;
	int pkts = 0;
	int err;

	spin_lock(&vdpasim->lock);

	if (!(vdpasim->status & VIRTIO_CONFIG_S_DRIVER_OK))
		goto out;

	if (!txq->ready || !rxq->ready)
		goto out;

	while (true) {
		total_write = 0;
		err = vringh_getdesc_iotlb(&txq->vring, &txq->iov, NULL,
					   &txq->head, GFP_ATOMIC);
		if (err <= 0)
			break;

		err = vringh_getdesc_iotlb(&rxq->vring, NULL, &rxq->iov,
					   &rxq->head, GFP_ATOMIC);
		if (err <= 0) {
			vringh_complete_iotlb(&txq->vring, txq->head, 0);
			break;
		}

		while (true) {
			read = vringh_iov_pull_iotlb(&txq->vring, &txq->iov,
						     vdpasim->buffer,
						     PAGE_SIZE);
			if (read <= 0)
				break;

			write = vringh_iov_push_iotlb(&rxq->vring, &rxq->iov,
						      vdpasim->buffer, read);
			if (write <= 0)
				break;

			total_write += write;
		}

		/* Make sure data is wrote before advancing index */
		smp_wmb();

		vringh_complete_iotlb(&txq->vring, txq->head, 0);
		vringh_complete_iotlb(&rxq->vring, rxq->head, total_write);

		/* Make sure used is visible before rasing the interrupt. */
		smp_wmb();

		local_bh_disable();
		if (txq->cb)
			txq->cb(txq->private);
		if (rxq->cb)
			rxq->cb(rxq->private);
		local_bh_enable();

		if (++pkts > 4) {
			schedule_work(&vdpasim->work);
			goto out;
		}
	}

out:
	spin_unlock(&vdpasim->lock);
}

static int dir_to_perm(enum dma_data_direction dir)
{
	int perm = -EFAULT;

	switch (dir) {
	case DMA_FROM_DEVICE:
		perm = VHOST_MAP_WO;
		break;
	case DMA_TO_DEVICE:
		perm = VHOST_MAP_RO;
		break;
	case DMA_BIDIRECTIONAL:
		perm = VHOST_MAP_RW;
		break;
	default:
		break;
	}

	return perm;
}

static dma_addr_t vdpasim_map_page(struct device *dev, struct page *page,
				   unsigned long offset, size_t size,
				   enum dma_data_direction dir,
				   unsigned long attrs)
{
	struct vdpasim *vdpasim = dev_to_sim(dev);
	struct vhost_iotlb *iommu = vdpasim->iommu;
	u64 pa = (page_to_pfn(page) << PAGE_SHIFT) + offset;
	int ret, perm = dir_to_perm(dir);

	if (perm < 0)
		return DMA_MAPPING_ERROR;

	/* For simplicity, use identical mapping to avoid e.g iova
	 * allocator.
	 */
	spin_lock(&vdpasim->iommu_lock);
	ret = vhost_iotlb_add_range(iommu, pa, pa + size - 1,
				    pa, dir_to_perm(dir));
	spin_unlock(&vdpasim->iommu_lock);
	if (ret)
		return DMA_MAPPING_ERROR;

	return (dma_addr_t)(pa);
}

static void vdpasim_unmap_page(struct device *dev, dma_addr_t dma_addr,
			       size_t size, enum dma_data_direction dir,
			       unsigned long attrs)
{
	struct vdpasim *vdpasim = dev_to_sim(dev);
	struct vhost_iotlb *iommu = vdpasim->iommu;

	spin_lock(&vdpasim->iommu_lock);
	vhost_iotlb_del_range(iommu, (u64)dma_addr,
			      (u64)dma_addr + size - 1);
	spin_unlock(&vdpasim->iommu_lock);
}

static void *vdpasim_alloc_coherent(struct device *dev, size_t size,
				    dma_addr_t *dma_addr, gfp_t flag,
				    unsigned long attrs)
{
	struct vdpasim *vdpasim = dev_to_sim(dev);
	struct vhost_iotlb *iommu = vdpasim->iommu;
	void *addr = kmalloc(size, flag);
	int ret;

	spin_lock(&vdpasim->iommu_lock);
	if (!addr) {
		*dma_addr = DMA_MAPPING_ERROR;
	} else {
		u64 pa = virt_to_phys(addr);

		ret = vhost_iotlb_add_range(iommu, (u64)pa,
					    (u64)pa + size - 1,
					    pa, VHOST_MAP_RW);
		if (ret) {
			*dma_addr = DMA_MAPPING_ERROR;
			kfree(addr);
			addr = NULL;
		} else
			*dma_addr = (dma_addr_t)pa;
	}
	spin_unlock(&vdpasim->iommu_lock);

	return addr;
}

static void vdpasim_free_coherent(struct device *dev, size_t size,
				  void *vaddr, dma_addr_t dma_addr,
				  unsigned long attrs)
{
	struct vdpasim *vdpasim = dev_to_sim(dev);
	struct vhost_iotlb *iommu = vdpasim->iommu;

	spin_lock(&vdpasim->iommu_lock);
	vhost_iotlb_del_range(iommu, (u64)dma_addr,
			      (u64)dma_addr + size - 1);
	spin_unlock(&vdpasim->iommu_lock);

	kfree(phys_to_virt((uintptr_t)dma_addr));
}

static const struct dma_map_ops vdpasim_dma_ops = {
	.map_page = vdpasim_map_page,
	.unmap_page = vdpasim_unmap_page,
	.alloc = vdpasim_alloc_coherent,
	.free = vdpasim_free_coherent,
};

static const struct vdpa_config_ops vdpasim_net_config_ops;
static const struct vdpa_config_ops vdpasim_net_batch_config_ops;

static struct vdpasim *vdpasim_create(struct vdpasim_dev_attr *dev_attr)
{
	const struct vdpa_config_ops *ops;
	struct vdpasim *vdpasim;
	struct device *dev;
	int i, ret = -ENOMEM;

	if (batch_mapping)
		ops = &vdpasim_net_batch_config_ops;
	else
		ops = &vdpasim_net_config_ops;

	vdpasim = vdpa_alloc_device(struct vdpasim, vdpa, NULL, ops,
				    dev_attr->nvqs);
	if (!vdpasim)
		goto err_alloc;

	vdpasim->dev_attr = *dev_attr;
	INIT_WORK(&vdpasim->work, vdpasim_work);
	spin_lock_init(&vdpasim->lock);
	spin_lock_init(&vdpasim->iommu_lock);

	dev = &vdpasim->vdpa.dev;
	dev->dma_mask = &dev->coherent_dma_mask;
	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64)))
		goto err_iommu;
	set_dma_ops(dev, &vdpasim_dma_ops);

	vdpasim->config = kzalloc(dev_attr->config_size, GFP_KERNEL);
	if (!vdpasim->config)
		goto err_iommu;

	vdpasim->vqs = kcalloc(dev_attr->nvqs, sizeof(struct vdpasim_virtqueue),
			       GFP_KERNEL);
	if (!vdpasim->vqs)
		goto err_iommu;

	vdpasim->iommu = vhost_iotlb_alloc(2048, 0);
	if (!vdpasim->iommu)
		goto err_iommu;

	vdpasim->buffer = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!vdpasim->buffer)
		goto err_iommu;

	if (macaddr) {
		mac_pton(macaddr, macaddr_buf);
		if (!is_valid_ether_addr(macaddr_buf)) {
			ret = -EADDRNOTAVAIL;
			goto err_iommu;
		}
	} else {
		eth_random_addr(macaddr_buf);
	}

	for (i = 0; i < dev_attr->nvqs; i++)
		vringh_set_iotlb(&vdpasim->vqs[i].vring, vdpasim->iommu);

	vdpasim->vdpa.dma_dev = dev;
	ret = vdpa_register_device(&vdpasim->vdpa);
	if (ret)
		goto err_iommu;

	return vdpasim;

err_iommu:
	put_device(dev);
err_alloc:
	return ERR_PTR(ret);
}

static int vdpasim_set_vq_address(struct vdpa_device *vdpa, u16 idx,
				  u64 desc_area, u64 driver_area,
				  u64 device_area)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);
	struct vdpasim_virtqueue *vq = &vdpasim->vqs[idx];

	vq->desc_addr = desc_area;
	vq->driver_addr = driver_area;
	vq->device_addr = device_area;

	return 0;
}

static void vdpasim_set_vq_num(struct vdpa_device *vdpa, u16 idx, u32 num)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);
	struct vdpasim_virtqueue *vq = &vdpasim->vqs[idx];

	vq->num = num;
}

static void vdpasim_kick_vq(struct vdpa_device *vdpa, u16 idx)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);
	struct vdpasim_virtqueue *vq = &vdpasim->vqs[idx];

	if (vq->ready)
		schedule_work(&vdpasim->work);
}

static void vdpasim_set_vq_cb(struct vdpa_device *vdpa, u16 idx,
			      struct vdpa_callback *cb)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);
	struct vdpasim_virtqueue *vq = &vdpasim->vqs[idx];

	vq->cb = cb->callback;
	vq->private = cb->private;
}

static void vdpasim_set_vq_ready(struct vdpa_device *vdpa, u16 idx, bool ready)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);
	struct vdpasim_virtqueue *vq = &vdpasim->vqs[idx];
	bool old_ready;

	spin_lock(&vdpasim->lock);
	old_ready = vq->ready;
	vq->ready = ready;
	if (vq->ready && !old_ready) {
		vdpasim_queue_ready(vdpasim, idx);
	}
	spin_unlock(&vdpasim->lock);
}

static bool vdpasim_get_vq_ready(struct vdpa_device *vdpa, u16 idx)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);
	struct vdpasim_virtqueue *vq = &vdpasim->vqs[idx];

	return vq->ready;
}

static int vdpasim_set_vq_state(struct vdpa_device *vdpa, u16 idx,
				const struct vdpa_vq_state *state)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);
	struct vdpasim_virtqueue *vq = &vdpasim->vqs[idx];
	struct vringh *vrh = &vq->vring;

	spin_lock(&vdpasim->lock);
	vrh->last_avail_idx = state->avail_index;
	spin_unlock(&vdpasim->lock);

	return 0;
}

static int vdpasim_get_vq_state(struct vdpa_device *vdpa, u16 idx,
				struct vdpa_vq_state *state)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);
	struct vdpasim_virtqueue *vq = &vdpasim->vqs[idx];
	struct vringh *vrh = &vq->vring;

	state->avail_index = vrh->last_avail_idx;
	return 0;
}

static u32 vdpasim_get_vq_align(struct vdpa_device *vdpa)
{
	return VDPASIM_QUEUE_ALIGN;
}

static u64 vdpasim_get_features(struct vdpa_device *vdpa)
{
	return vdpasim_features;
}

static int vdpasim_set_features(struct vdpa_device *vdpa, u64 features)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);

	/* DMA mapping must be done by driver */
	if (!(features & (1ULL << VIRTIO_F_ACCESS_PLATFORM)))
		return -EINVAL;

	vdpasim->features = features & vdpasim_features;

	return 0;
}

static void vdpasim_set_config_cb(struct vdpa_device *vdpa,
				  struct vdpa_callback *cb)
{
	/* We don't support config interrupt */
}

static u16 vdpasim_get_vq_num_max(struct vdpa_device *vdpa)
{
	return VDPASIM_QUEUE_MAX;
}

static u32 vdpasim_get_device_id(struct vdpa_device *vdpa)
{
	return VDPASIM_DEVICE_ID;
}

static u32 vdpasim_get_vendor_id(struct vdpa_device *vdpa)
{
	return VDPASIM_VENDOR_ID;
}

static u8 vdpasim_get_status(struct vdpa_device *vdpa)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);
	u8 status;

	spin_lock(&vdpasim->lock);
	status = vdpasim->status;
	spin_unlock(&vdpasim->lock);

	return status;
}

static void vdpasim_set_status(struct vdpa_device *vdpa, u8 status)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);

	spin_lock(&vdpasim->lock);
	vdpasim->status = status;
	if (status == 0)
		vdpasim_reset(vdpasim);
	spin_unlock(&vdpasim->lock);
}

static void vdpasim_get_config(struct vdpa_device *vdpa, unsigned int offset,
			     void *buf, unsigned int len)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);

	if (offset + len > vdpasim->dev_attr.config_size)
		return;

	if (vdpasim->dev_attr.get_config)
		vdpasim->dev_attr.get_config(vdpasim, vdpasim->config);

	memcpy(buf, vdpasim->config + offset, len);
}

static void vdpasim_set_config(struct vdpa_device *vdpa, unsigned int offset,
			     const void *buf, unsigned int len)
{
	/* No writable config supportted by vdpasim */
}

static u32 vdpasim_get_generation(struct vdpa_device *vdpa)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);

	return vdpasim->generation;
}

static struct vdpa_iova_range vdpasim_get_iova_range(struct vdpa_device *vdpa)
{
	struct vdpa_iova_range range = {
		.first = 0ULL,
		.last = ULLONG_MAX,
	};

	return range;
}

static int vdpasim_set_map(struct vdpa_device *vdpa,
			   struct vhost_iotlb *iotlb)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);
	struct vhost_iotlb_map *map;
	u64 start = 0ULL, last = 0ULL - 1;
	int ret;

	spin_lock(&vdpasim->iommu_lock);
	vhost_iotlb_reset(vdpasim->iommu);

	for (map = vhost_iotlb_itree_first(iotlb, start, last); map;
	     map = vhost_iotlb_itree_next(map, start, last)) {
		ret = vhost_iotlb_add_range(vdpasim->iommu, map->start,
					    map->last, map->addr, map->perm);
		if (ret)
			goto err;
	}
	spin_unlock(&vdpasim->iommu_lock);
	return 0;

err:
	vhost_iotlb_reset(vdpasim->iommu);
	spin_unlock(&vdpasim->iommu_lock);
	return ret;
}

static int vdpasim_dma_map(struct vdpa_device *vdpa, u64 iova, u64 size,
			   u64 pa, u32 perm)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);
	int ret;

	spin_lock(&vdpasim->iommu_lock);
	ret = vhost_iotlb_add_range(vdpasim->iommu, iova, iova + size - 1, pa,
				    perm);
	spin_unlock(&vdpasim->iommu_lock);

	return ret;
}

static int vdpasim_dma_unmap(struct vdpa_device *vdpa, u64 iova, u64 size)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);

	spin_lock(&vdpasim->iommu_lock);
	vhost_iotlb_del_range(vdpasim->iommu, iova, iova + size - 1);
	spin_unlock(&vdpasim->iommu_lock);

	return 0;
}

static void vdpasim_free(struct vdpa_device *vdpa)
{
	struct vdpasim *vdpasim = vdpa_to_sim(vdpa);

	cancel_work_sync(&vdpasim->work);
	kfree(vdpasim->buffer);
	if (vdpasim->iommu)
		vhost_iotlb_free(vdpasim->iommu);
	kfree(vdpasim->vqs);
	kfree(vdpasim->config);
}

static const struct vdpa_config_ops vdpasim_net_config_ops = {
	.set_vq_address         = vdpasim_set_vq_address,
	.set_vq_num             = vdpasim_set_vq_num,
	.kick_vq                = vdpasim_kick_vq,
	.set_vq_cb              = vdpasim_set_vq_cb,
	.set_vq_ready           = vdpasim_set_vq_ready,
	.get_vq_ready           = vdpasim_get_vq_ready,
	.set_vq_state           = vdpasim_set_vq_state,
	.get_vq_state           = vdpasim_get_vq_state,
	.get_vq_align           = vdpasim_get_vq_align,
	.get_features           = vdpasim_get_features,
	.set_features           = vdpasim_set_features,
	.set_config_cb          = vdpasim_set_config_cb,
	.get_vq_num_max         = vdpasim_get_vq_num_max,
	.get_device_id          = vdpasim_get_device_id,
	.get_vendor_id          = vdpasim_get_vendor_id,
	.get_status             = vdpasim_get_status,
	.set_status             = vdpasim_set_status,
	.get_config             = vdpasim_get_config,
	.set_config             = vdpasim_set_config,
	.get_generation         = vdpasim_get_generation,
	.get_iova_range         = vdpasim_get_iova_range,
	.dma_map                = vdpasim_dma_map,
	.dma_unmap              = vdpasim_dma_unmap,
	.free                   = vdpasim_free,
};

static const struct vdpa_config_ops vdpasim_net_batch_config_ops = {
	.set_vq_address         = vdpasim_set_vq_address,
	.set_vq_num             = vdpasim_set_vq_num,
	.kick_vq                = vdpasim_kick_vq,
	.set_vq_cb              = vdpasim_set_vq_cb,
	.set_vq_ready           = vdpasim_set_vq_ready,
	.get_vq_ready           = vdpasim_get_vq_ready,
	.set_vq_state           = vdpasim_set_vq_state,
	.get_vq_state           = vdpasim_get_vq_state,
	.get_vq_align           = vdpasim_get_vq_align,
	.get_features           = vdpasim_get_features,
	.set_features           = vdpasim_set_features,
	.set_config_cb          = vdpasim_set_config_cb,
	.get_vq_num_max         = vdpasim_get_vq_num_max,
	.get_device_id          = vdpasim_get_device_id,
	.get_vendor_id          = vdpasim_get_vendor_id,
	.get_status             = vdpasim_get_status,
	.set_status             = vdpasim_set_status,
	.get_config             = vdpasim_get_config,
	.set_config             = vdpasim_set_config,
	.get_generation         = vdpasim_get_generation,
	.get_iova_range         = vdpasim_get_iova_range,
	.set_map                = vdpasim_set_map,
	.free                   = vdpasim_free,
};

static void vdpasim_net_get_config(struct vdpasim *vdpasim, void *config)
{
	struct virtio_net_config *net_config =
		(struct virtio_net_config *)config;

	net_config->mtu = cpu_to_vdpasim16(vdpasim, 1500);
	net_config->status = cpu_to_vdpasim16(vdpasim, VIRTIO_NET_S_LINK_UP);
	memcpy(net_config->mac, macaddr_buf, ETH_ALEN);
}

static int __init vdpasim_dev_init(void)
{
	struct vdpasim_dev_attr dev_attr = {};

	dev_attr.nvqs = VDPASIM_VQ_NUM;
	dev_attr.config_size = sizeof(struct virtio_net_config);
	dev_attr.get_config = vdpasim_net_get_config;

	vdpasim_dev = vdpasim_create(&dev_attr);

	if (!IS_ERR(vdpasim_dev))
		return 0;

	return PTR_ERR(vdpasim_dev);
}

static void __exit vdpasim_dev_exit(void)
{
	struct vdpa_device *vdpa = &vdpasim_dev->vdpa;

	vdpa_unregister_device(vdpa);
}

module_init(vdpasim_dev_init)
module_exit(vdpasim_dev_exit)

MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE(DRV_LICENSE);
MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
