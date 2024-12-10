/******************************************************************************
 * gntdev.c
 *
 * Device for accessing (in user-space) pages that have been granted by other
 * domains.
 *
 * Copyright (c) 2006-2007, D G Murray.
 *           (c) 2009 Gerd Hoffmann <kraxel@redhat.com>
 *           (c) 2018 Oleksandr Andrushchenko, EPAM Systems Inc.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#undef DEBUG

#define pr_fmt(fmt) "xen:" KBUILD_MODNAME ": " fmt

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/refcount.h>
#include <linux/workqueue.h>

#include <xen/xen.h>
#include <xen/grant_table.h>
#include <xen/balloon.h>
#include <xen/gntdev.h>
#include <xen/events.h>
#include <xen/page.h>
#include <asm/xen/hypervisor.h>
#include <asm/xen/hypercall.h>

#include "gntdev-common.h"
#ifdef CONFIG_XEN_GNTDEV_DMABUF
#include "gntdev-dmabuf.h"
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Derek G. Murray <Derek.Murray@cl.cam.ac.uk>, "
	      "Gerd Hoffmann <kraxel@redhat.com>");
MODULE_DESCRIPTION("User-space granted page access driver");

static unsigned int limit = 64*1024;
module_param(limit, uint, 0644);
MODULE_PARM_DESC(limit,
	"Maximum number of grants that may be mapped by one mapping request");

/* True in PV mode, false otherwise */
static int use_ptemod;

static void unmap_grant_pages(struct gntdev_grant_map *map,
			      int offset, int pages);

static struct miscdevice gntdev_miscdev;

/* ------------------------------------------------------------------ */

bool gntdev_test_page_count(unsigned int count)
{
	return !count || count > limit;
}

static void gntdev_print_maps(struct gntdev_priv *priv,
			      char *text, int text_index)
{
#ifdef DEBUG
	struct gntdev_grant_map *map;

	pr_debug("%s: maps list (priv %p)\n", __func__, priv);
	list_for_each_entry(map, &priv->maps, next)
		pr_debug("  index %2d, count %2d %s\n",
		       map->index, map->count,
		       map->index == text_index && text ? text : "");
#endif
}

static void gntdev_free_map(struct gntdev_grant_map *map)
{
	if (map == NULL)
		return;

#ifdef CONFIG_XEN_GRANT_DMA_ALLOC
	if (map->dma_vaddr) {
		struct gnttab_dma_alloc_args args;

		args.dev = map->dma_dev;
		args.coherent = !!(map->dma_flags & GNTDEV_DMA_FLAG_COHERENT);
		args.nr_pages = map->count;
		args.pages = map->pages;
		args.frames = map->frames;
		args.vaddr = map->dma_vaddr;
		args.dev_bus_addr = map->dma_bus_addr;

		gnttab_dma_free_pages(&args);
	} else
#endif
	if (map->pages)
		gnttab_free_pages(map->count, map->pages);

#ifdef CONFIG_XEN_GRANT_DMA_ALLOC
	kvfree(map->frames);
#endif
	kvfree(map->pages);
	kvfree(map->grants);
	kvfree(map->map_ops);
	kvfree(map->unmap_ops);
	kvfree(map->kmap_ops);
	kvfree(map->kunmap_ops);
	kvfree(map->being_removed);
	kfree(map);
}

struct gntdev_grant_map *gntdev_alloc_map(struct gntdev_priv *priv, int count,
					  int dma_flags)
{
	struct gntdev_grant_map *add;
	int i;

	add = kzalloc(sizeof(*add), GFP_KERNEL);
	if (NULL == add)
		return NULL;

	add->grants    = kvcalloc(count, sizeof(add->grants[0]), GFP_KERNEL);
	add->map_ops   = kvcalloc(count, sizeof(add->map_ops[0]), GFP_KERNEL);
	add->unmap_ops = kvcalloc(count, sizeof(add->unmap_ops[0]), GFP_KERNEL);
	add->kmap_ops  = kvcalloc(count, sizeof(add->kmap_ops[0]), GFP_KERNEL);
	add->kunmap_ops = kvcalloc(count,
				   sizeof(add->kunmap_ops[0]), GFP_KERNEL);
	add->pages     = kvcalloc(count, sizeof(add->pages[0]), GFP_KERNEL);
	add->being_removed =
		kvcalloc(count, sizeof(add->being_removed[0]), GFP_KERNEL);
	if (NULL == add->grants    ||
	    NULL == add->map_ops   ||
	    NULL == add->unmap_ops ||
	    NULL == add->kmap_ops  ||
	    NULL == add->kunmap_ops ||
	    NULL == add->pages     ||
	    NULL == add->being_removed)
		goto err;

#ifdef CONFIG_XEN_GRANT_DMA_ALLOC
	add->dma_flags = dma_flags;

	/*
	 * Check if this mapping is requested to be backed
	 * by a DMA buffer.
	 */
	if (dma_flags & (GNTDEV_DMA_FLAG_WC | GNTDEV_DMA_FLAG_COHERENT)) {
		struct gnttab_dma_alloc_args args;

		add->frames = kvcalloc(count, sizeof(add->frames[0]),
				       GFP_KERNEL);
		if (!add->frames)
			goto err;

		/* Remember the device, so we can free DMA memory. */
		add->dma_dev = priv->dma_dev;

		args.dev = priv->dma_dev;
		args.coherent = !!(dma_flags & GNTDEV_DMA_FLAG_COHERENT);
		args.nr_pages = count;
		args.pages = add->pages;
		args.frames = add->frames;

		if (gnttab_dma_alloc_pages(&args))
			goto err;

		add->dma_vaddr = args.vaddr;
		add->dma_bus_addr = args.dev_bus_addr;
	} else
#endif
	if (gnttab_alloc_pages(count, add->pages))
		goto err;

	for (i = 0; i < count; i++) {
		add->map_ops[i].handle = -1;
		add->unmap_ops[i].handle = -1;
		add->kmap_ops[i].handle = -1;
		add->kunmap_ops[i].handle = -1;
	}

	add->index = 0;
	add->count = count;
	refcount_set(&add->users, 1);

	return add;

err:
	gntdev_free_map(add);
	return NULL;
}

void gntdev_add_map(struct gntdev_priv *priv, struct gntdev_grant_map *add)
{
	struct gntdev_grant_map *map;

	list_for_each_entry(map, &priv->maps, next) {
		if (add->index + add->count < map->index) {
			list_add_tail(&add->next, &map->next);
			goto done;
		}
		add->index = map->index + map->count;
	}
	list_add_tail(&add->next, &priv->maps);

done:
	gntdev_print_maps(priv, "[new]", add->index);
}

static struct gntdev_grant_map *gntdev_find_map_index(struct gntdev_priv *priv,
						      int index, int count)
{
	struct gntdev_grant_map *map;

	list_for_each_entry(map, &priv->maps, next) {
		if (map->index != index)
			continue;
		if (count && map->count != count)
			continue;
		return map;
	}
	return NULL;
}

void gntdev_put_map(struct gntdev_priv *priv, struct gntdev_grant_map *map)
{
	if (!map)
		return;

	if (!refcount_dec_and_test(&map->users))
		return;

	if (map->pages && !use_ptemod) {
		/*
		 * Increment the reference count.  This ensures that the
		 * subsequent call to unmap_grant_pages() will not wind up
		 * re-entering itself.  It *can* wind up calling
		 * gntdev_put_map() recursively, but such calls will be with a
		 * reference count greater than 1, so they will return before
		 * this code is reached.  The recursion depth is thus limited to
		 * 1.  Do NOT use refcount_inc() here, as it will detect that
		 * the reference count is zero and WARN().
		 */
		refcount_set(&map->users, 1);

		/*
		 * Unmap the grants.  This may or may not be asynchronous, so it
		 * is possible that the reference count is 1 on return, but it
		 * could also be greater than 1.
		 */
		unmap_grant_pages(map, 0, map->count);

		/* Check if the memory now needs to be freed */
		if (!refcount_dec_and_test(&map->users))
			return;

		/*
		 * All pages have been returned to the hypervisor, so free the
		 * map.
		 */
	}

	if (use_ptemod && map->notifier_init)
		mmu_interval_notifier_remove(&map->notifier);

	if (map->notify.flags & UNMAP_NOTIFY_SEND_EVENT) {
		notify_remote_via_evtchn(map->notify.event);
		evtchn_put(map->notify.event);
	}
	gntdev_free_map(map);
}

/* ------------------------------------------------------------------ */

static int find_grant_ptes(pte_t *pte, unsigned long addr, void *data)
{
	struct gntdev_grant_map *map = data;
	unsigned int pgnr = (addr - map->pages_vm_start) >> PAGE_SHIFT;
	int flags = map->flags | GNTMAP_application_map | GNTMAP_contains_pte |
		    (1 << _GNTMAP_guest_avail0);
	u64 pte_maddr;

	BUG_ON(pgnr >= map->count);
	pte_maddr = arbitrary_virt_to_machine(pte).maddr;

	gnttab_set_map_op(&map->map_ops[pgnr], pte_maddr, flags,
			  map->grants[pgnr].ref,
			  map->grants[pgnr].domid);
	gnttab_set_unmap_op(&map->unmap_ops[pgnr], pte_maddr, flags,
			    -1 /* handle */);
	return 0;
}

int gntdev_map_grant_pages(struct gntdev_grant_map *map)
{
	size_t alloced = 0;
	int i, err = 0;

	if (!use_ptemod) {
		/* Note: it could already be mapped */
		if (map->map_ops[0].handle != -1)
			return 0;
		for (i = 0; i < map->count; i++) {
			unsigned long addr = (unsigned long)
				pfn_to_kaddr(page_to_pfn(map->pages[i]));
			gnttab_set_map_op(&map->map_ops[i], addr, map->flags,
				map->grants[i].ref,
				map->grants[i].domid);
			gnttab_set_unmap_op(&map->unmap_ops[i], addr,
				map->flags, -1 /* handle */);
		}
	} else {
		/*
		 * Setup the map_ops corresponding to the pte entries pointing
		 * to the kernel linear addresses of the struct pages.
		 * These ptes are completely different from the user ptes dealt
		 * with find_grant_ptes.
		 * Note that GNTMAP_device_map isn't needed here: The
		 * dev_bus_addr output field gets consumed only from ->map_ops,
		 * and by not requesting it when mapping we also avoid needing
		 * to mirror dev_bus_addr into ->unmap_ops (and holding an extra
		 * reference to the page in the hypervisor).
		 */
		unsigned int flags = (map->flags & ~GNTMAP_device_map) |
				     GNTMAP_host_map;

		for (i = 0; i < map->count; i++) {
			unsigned long address = (unsigned long)
				pfn_to_kaddr(page_to_pfn(map->pages[i]));
			BUG_ON(PageHighMem(map->pages[i]));

			gnttab_set_map_op(&map->kmap_ops[i], address, flags,
				map->grants[i].ref,
				map->grants[i].domid);
			gnttab_set_unmap_op(&map->kunmap_ops[i], address,
				flags, -1);
		}
	}

	pr_debug("map %d+%d\n", map->index, map->count);
	err = gnttab_map_refs(map->map_ops, use_ptemod ? map->kmap_ops : NULL,
			map->pages, map->count);

	for (i = 0; i < map->count; i++) {
		if (map->map_ops[i].status == GNTST_okay) {
			map->unmap_ops[i].handle = map->map_ops[i].handle;
			alloced++;
		} else if (!err)
			err = -EINVAL;

		if (map->flags & GNTMAP_device_map)
			map->unmap_ops[i].dev_bus_addr = map->map_ops[i].dev_bus_addr;

		if (use_ptemod) {
			if (map->kmap_ops[i].status == GNTST_okay) {
				alloced++;
				map->kunmap_ops[i].handle = map->kmap_ops[i].handle;
			} else if (!err)
				err = -EINVAL;
		}
	}
	atomic_add(alloced, &map->live_grants);
	return err;
}

static void __unmap_grant_pages_done(int result,
		struct gntab_unmap_queue_data *data)
{
	unsigned int i;
	struct gntdev_grant_map *map = data->data;
	unsigned int offset = data->unmap_ops - map->unmap_ops;
	int successful_unmaps = 0;
	int live_grants;

	for (i = 0; i < data->count; i++) {
		if (map->unmap_ops[offset + i].status == GNTST_okay &&
		    map->unmap_ops[offset + i].handle != -1)
			successful_unmaps++;

		WARN_ON(map->unmap_ops[offset+i].status &&
			map->unmap_ops[offset+i].handle != -1);
		pr_debug("unmap handle=%d st=%d\n",
			map->unmap_ops[offset+i].handle,
			map->unmap_ops[offset+i].status);
		map->unmap_ops[offset+i].handle = -1;
		if (use_ptemod) {
			if (map->kunmap_ops[offset + i].status == GNTST_okay &&
			    map->kunmap_ops[offset + i].handle != -1)
				successful_unmaps++;

			WARN_ON(map->kunmap_ops[offset+i].status &&
				map->kunmap_ops[offset+i].handle != -1);
			pr_debug("kunmap handle=%u st=%d\n",
				 map->kunmap_ops[offset+i].handle,
				 map->kunmap_ops[offset+i].status);
			map->kunmap_ops[offset+i].handle = -1;
		}
	}

	/*
	 * Decrease the live-grant counter.  This must happen after the loop to
	 * prevent premature reuse of the grants by gnttab_mmap().
	 */
	live_grants = atomic_sub_return(successful_unmaps, &map->live_grants);
	if (WARN_ON(live_grants < 0))
		pr_err("%s: live_grants became negative (%d) after unmapping %d pages!\n",
		       __func__, live_grants, successful_unmaps);

	/* Release reference taken by __unmap_grant_pages */
	gntdev_put_map(NULL, map);
}

static void __unmap_grant_pages(struct gntdev_grant_map *map, int offset,
			       int pages)
{
	if (map->notify.flags & UNMAP_NOTIFY_CLEAR_BYTE) {
		int pgno = (map->notify.addr >> PAGE_SHIFT);

		if (pgno >= offset && pgno < offset + pages) {
			/* No need for kmap, pages are in lowmem */
			uint8_t *tmp = pfn_to_kaddr(page_to_pfn(map->pages[pgno]));

			tmp[map->notify.addr & (PAGE_SIZE-1)] = 0;
			map->notify.flags &= ~UNMAP_NOTIFY_CLEAR_BYTE;
		}
	}

	map->unmap_data.unmap_ops = map->unmap_ops + offset;
	map->unmap_data.kunmap_ops = use_ptemod ? map->kunmap_ops + offset : NULL;
	map->unmap_data.pages = map->pages + offset;
	map->unmap_data.count = pages;
	map->unmap_data.done = __unmap_grant_pages_done;
	map->unmap_data.data = map;
	refcount_inc(&map->users); /* to keep map alive during async call below */

	gnttab_unmap_refs_async(&map->unmap_data);
}

static void unmap_grant_pages(struct gntdev_grant_map *map, int offset,
			      int pages)
{
	int range;

	if (atomic_read(&map->live_grants) == 0)
		return; /* Nothing to do */

	pr_debug("unmap %d+%d [%d+%d]\n", map->index, map->count, offset, pages);

	/* It is possible the requested range will have a "hole" where we
	 * already unmapped some of the grants. Only unmap valid ranges.
	 */
	while (pages) {
		while (pages && map->being_removed[offset]) {
			offset++;
			pages--;
		}
		range = 0;
		while (range < pages) {
			if (map->being_removed[offset + range])
				break;
			map->being_removed[offset + range] = true;
			range++;
		}
		if (range)
			__unmap_grant_pages(map, offset, range);
		offset += range;
		pages -= range;
	}
}

/* ------------------------------------------------------------------ */

static void gntdev_vma_open(struct vm_area_struct *vma)
{
	struct gntdev_grant_map *map = vma->vm_private_data;

	pr_debug("gntdev_vma_open %p\n", vma);
	refcount_inc(&map->users);
}

static void gntdev_vma_close(struct vm_area_struct *vma)
{
	struct gntdev_grant_map *map = vma->vm_private_data;
	struct file *file = vma->vm_file;
	struct gntdev_priv *priv = file->private_data;

	pr_debug("gntdev_vma_close %p\n", vma);

	vma->vm_private_data = NULL;
	gntdev_put_map(priv, map);
}

static struct page *gntdev_vma_find_special_page(struct vm_area_struct *vma,
						 unsigned long addr)
{
	struct gntdev_grant_map *map = vma->vm_private_data;

	return map->pages[(addr - map->pages_vm_start) >> PAGE_SHIFT];
}

static const struct vm_operations_struct gntdev_vmops = {
	.open = gntdev_vma_open,
	.close = gntdev_vma_close,
	.find_special_page = gntdev_vma_find_special_page,
};

/* ------------------------------------------------------------------ */

static bool gntdev_invalidate(struct mmu_interval_notifier *mn,
			      const struct mmu_notifier_range *range,
			      unsigned long cur_seq)
{
	struct gntdev_grant_map *map =
		container_of(mn, struct gntdev_grant_map, notifier);
	unsigned long mstart, mend;
	unsigned long map_start, map_end;

	if (!mmu_notifier_range_blockable(range))
		return false;

	map_start = map->pages_vm_start;
	map_end = map->pages_vm_start + (map->count << PAGE_SHIFT);

	/*
	 * If the VMA is split or otherwise changed the notifier is not
	 * updated, but we don't want to process VA's outside the modified
	 * VMA. FIXME: It would be much more understandable to just prevent
	 * modifying the VMA in the first place.
	 */
	if (map_start >= range->end || map_end <= range->start)
		return true;

	mstart = max(range->start, map_start);
	mend = min(range->end, map_end);
	pr_debug("map %d+%d (%lx %lx), range %lx %lx, mrange %lx %lx\n",
		 map->index, map->count, map_start, map_end,
		 range->start, range->end, mstart, mend);
	unmap_grant_pages(map, (mstart - map_start) >> PAGE_SHIFT,
			  (mend - mstart) >> PAGE_SHIFT);

	return true;
}

static const struct mmu_interval_notifier_ops gntdev_mmu_ops = {
	.invalidate = gntdev_invalidate,
};

/* ------------------------------------------------------------------ */

static int gntdev_open(struct inode *inode, struct file *flip)
{
	struct gntdev_priv *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	INIT_LIST_HEAD(&priv->maps);
	mutex_init(&priv->lock);

#ifdef CONFIG_XEN_GNTDEV_DMABUF
	priv->dmabuf_priv = gntdev_dmabuf_init(flip);
	if (IS_ERR(priv->dmabuf_priv)) {
		int ret = PTR_ERR(priv->dmabuf_priv);

		kfree(priv);
		return ret;
	}
#endif

	flip->private_data = priv;
#ifdef CONFIG_XEN_GRANT_DMA_ALLOC
	priv->dma_dev = gntdev_miscdev.this_device;
	dma_coerce_mask_and_coherent(priv->dma_dev, DMA_BIT_MASK(64));
#endif
	pr_debug("priv %p\n", priv);

	return 0;
}

static int gntdev_release(struct inode *inode, struct file *flip)
{
	struct gntdev_priv *priv = flip->private_data;
	struct gntdev_grant_map *map;

	pr_debug("priv %p\n", priv);

	mutex_lock(&priv->lock);
	while (!list_empty(&priv->maps)) {
		map = list_entry(priv->maps.next,
				 struct gntdev_grant_map, next);
		list_del(&map->next);
		gntdev_put_map(NULL /* already removed */, map);
	}
	mutex_unlock(&priv->lock);

#ifdef CONFIG_XEN_GNTDEV_DMABUF
	gntdev_dmabuf_fini(priv->dmabuf_priv);
#endif

	kfree(priv);
	return 0;
}

static long gntdev_ioctl_map_grant_ref(struct gntdev_priv *priv,
				       struct ioctl_gntdev_map_grant_ref __user *u)
{
	struct ioctl_gntdev_map_grant_ref op;
	struct gntdev_grant_map *map;
	int err;

	if (copy_from_user(&op, u, sizeof(op)) != 0)
		return -EFAULT;
	pr_debug("priv %p, add %d\n", priv, op.count);
	if (unlikely(gntdev_test_page_count(op.count)))
		return -EINVAL;

	err = -ENOMEM;
	map = gntdev_alloc_map(priv, op.count, 0 /* This is not a dma-buf. */);
	if (!map)
		return err;

	if (copy_from_user(map->grants, &u->refs,
			   sizeof(map->grants[0]) * op.count) != 0) {
		gntdev_put_map(NULL, map);
		return -EFAULT;
	}

	mutex_lock(&priv->lock);
	gntdev_add_map(priv, map);
	op.index = map->index << PAGE_SHIFT;
	mutex_unlock(&priv->lock);

	if (copy_to_user(u, &op, sizeof(op)) != 0)
		return -EFAULT;

	return 0;
}

static long gntdev_ioctl_unmap_grant_ref(struct gntdev_priv *priv,
					 struct ioctl_gntdev_unmap_grant_ref __user *u)
{
	struct ioctl_gntdev_unmap_grant_ref op;
	struct gntdev_grant_map *map;
	int err = -ENOENT;

	if (copy_from_user(&op, u, sizeof(op)) != 0)
		return -EFAULT;
	pr_debug("priv %p, del %d+%d\n", priv, (int)op.index, (int)op.count);

	mutex_lock(&priv->lock);
	map = gntdev_find_map_index(priv, op.index >> PAGE_SHIFT, op.count);
	if (map) {
		list_del(&map->next);
		err = 0;
	}
	mutex_unlock(&priv->lock);
	if (map)
		gntdev_put_map(priv, map);
	return err;
}

static long gntdev_ioctl_get_offset_for_vaddr(struct gntdev_priv *priv,
					      struct ioctl_gntdev_get_offset_for_vaddr __user *u)
{
	struct ioctl_gntdev_get_offset_for_vaddr op;
	struct vm_area_struct *vma;
	struct gntdev_grant_map *map;
	int rv = -EINVAL;

	if (copy_from_user(&op, u, sizeof(op)) != 0)
		return -EFAULT;
	pr_debug("priv %p, offset for vaddr %lx\n", priv, (unsigned long)op.vaddr);

	mmap_read_lock(current->mm);
	vma = find_vma(current->mm, op.vaddr);
	if (!vma || vma->vm_ops != &gntdev_vmops)
		goto out_unlock;

	map = vma->vm_private_data;
	if (!map)
		goto out_unlock;

	op.offset = map->index << PAGE_SHIFT;
	op.count = map->count;
	rv = 0;

 out_unlock:
	mmap_read_unlock(current->mm);

	if (rv == 0 && copy_to_user(u, &op, sizeof(op)) != 0)
		return -EFAULT;
	return rv;
}

static long gntdev_ioctl_notify(struct gntdev_priv *priv, void __user *u)
{
	struct ioctl_gntdev_unmap_notify op;
	struct gntdev_grant_map *map;
	int rc;
	int out_flags;
	evtchn_port_t out_event;

	if (copy_from_user(&op, u, sizeof(op)))
		return -EFAULT;

	if (op.action & ~(UNMAP_NOTIFY_CLEAR_BYTE|UNMAP_NOTIFY_SEND_EVENT))
		return -EINVAL;

	/* We need to grab a reference to the event channel we are going to use
	 * to send the notify before releasing the reference we may already have
	 * (if someone has called this ioctl twice). This is required so that
	 * it is possible to change the clear_byte part of the notification
	 * without disturbing the event channel part, which may now be the last
	 * reference to that event channel.
	 */
	if (op.action & UNMAP_NOTIFY_SEND_EVENT) {
		if (evtchn_get(op.event_channel_port))
			return -EINVAL;
	}

	out_flags = op.action;
	out_event = op.event_channel_port;

	mutex_lock(&priv->lock);

	list_for_each_entry(map, &priv->maps, next) {
		uint64_t begin = map->index << PAGE_SHIFT;
		uint64_t end = (map->index + map->count) << PAGE_SHIFT;
		if (op.index >= begin && op.index < end)
			goto found;
	}
	rc = -ENOENT;
	goto unlock_out;

 found:
	if ((op.action & UNMAP_NOTIFY_CLEAR_BYTE) &&
			(map->flags & GNTMAP_readonly)) {
		rc = -EINVAL;
		goto unlock_out;
	}

	out_flags = map->notify.flags;
	out_event = map->notify.event;

	map->notify.flags = op.action;
	map->notify.addr = op.index - (map->index << PAGE_SHIFT);
	map->notify.event = op.event_channel_port;

	rc = 0;

 unlock_out:
	mutex_unlock(&priv->lock);

	/* Drop the reference to the event channel we did not save in the map */
	if (out_flags & UNMAP_NOTIFY_SEND_EVENT)
		evtchn_put(out_event);

	return rc;
}

#define GNTDEV_COPY_BATCH 16

struct gntdev_copy_batch {
	struct gnttab_copy ops[GNTDEV_COPY_BATCH];
	struct page *pages[GNTDEV_COPY_BATCH];
	s16 __user *status[GNTDEV_COPY_BATCH];
	unsigned int nr_ops;
	unsigned int nr_pages;
	bool writeable;
};

static int gntdev_get_page(struct gntdev_copy_batch *batch, void __user *virt,
				unsigned long *gfn)
{
	unsigned long addr = (unsigned long)virt;
	struct page *page;
	unsigned long xen_pfn;
	int ret;

	ret = pin_user_pages_fast(addr, 1, batch->writeable ? FOLL_WRITE : 0, &page);
	if (ret < 0)
		return ret;

	batch->pages[batch->nr_pages++] = page;

	xen_pfn = page_to_xen_pfn(page) + XEN_PFN_DOWN(addr & ~PAGE_MASK);
	*gfn = pfn_to_gfn(xen_pfn);

	return 0;
}

static void gntdev_put_pages(struct gntdev_copy_batch *batch)
{
	unpin_user_pages_dirty_lock(batch->pages, batch->nr_pages, batch->writeable);
	batch->nr_pages = 0;
	batch->writeable = false;
}

static int gntdev_copy(struct gntdev_copy_batch *batch)
{
	unsigned int i;

	gnttab_batch_copy(batch->ops, batch->nr_ops);
	gntdev_put_pages(batch);

	/*
	 * For each completed op, update the status if the op failed
	 * and all previous ops for the segment were successful.
	 */
	for (i = 0; i < batch->nr_ops; i++) {
		s16 status = batch->ops[i].status;
		s16 old_status;

		if (status == GNTST_okay)
			continue;

		if (__get_user(old_status, batch->status[i]))
			return -EFAULT;

		if (old_status != GNTST_okay)
			continue;

		if (__put_user(status, batch->status[i]))
			return -EFAULT;
	}

	batch->nr_ops = 0;
	return 0;
}

static int gntdev_grant_copy_seg(struct gntdev_copy_batch *batch,
				 struct gntdev_grant_copy_segment *seg,
				 s16 __user *status)
{
	uint16_t copied = 0;

	/*
	 * Disallow local -> local copies since there is only space in
	 * batch->pages for one page per-op and this would be a very
	 * expensive memcpy().
	 */
	if (!(seg->flags & (GNTCOPY_source_gref | GNTCOPY_dest_gref)))
		return -EINVAL;

	/* Can't cross page if source/dest is a grant ref. */
	if (seg->flags & GNTCOPY_source_gref) {
		if (seg->source.foreign.offset + seg->len > XEN_PAGE_SIZE)
			return -EINVAL;
	}
	if (seg->flags & GNTCOPY_dest_gref) {
		if (seg->dest.foreign.offset + seg->len > XEN_PAGE_SIZE)
			return -EINVAL;
	}

	if (put_user(GNTST_okay, status))
		return -EFAULT;

	while (copied < seg->len) {
		struct gnttab_copy *op;
		void __user *virt;
		size_t len, off;
		unsigned long gfn;
		int ret;

		if (batch->nr_ops >= GNTDEV_COPY_BATCH) {
			ret = gntdev_copy(batch);
			if (ret < 0)
				return ret;
		}

		len = seg->len - copied;

		op = &batch->ops[batch->nr_ops];
		op->flags = 0;

		if (seg->flags & GNTCOPY_source_gref) {
			op->source.u.ref = seg->source.foreign.ref;
			op->source.domid = seg->source.foreign.domid;
			op->source.offset = seg->source.foreign.offset + copied;
			op->flags |= GNTCOPY_source_gref;
		} else {
			virt = seg->source.virt + copied;
			off = (unsigned long)virt & ~XEN_PAGE_MASK;
			len = min(len, (size_t)XEN_PAGE_SIZE - off);
			batch->writeable = false;

			ret = gntdev_get_page(batch, virt, &gfn);
			if (ret < 0)
				return ret;

			op->source.u.gmfn = gfn;
			op->source.domid = DOMID_SELF;
			op->source.offset = off;
		}

		if (seg->flags & GNTCOPY_dest_gref) {
			op->dest.u.ref = seg->dest.foreign.ref;
			op->dest.domid = seg->dest.foreign.domid;
			op->dest.offset = seg->dest.foreign.offset + copied;
			op->flags |= GNTCOPY_dest_gref;
		} else {
			virt = seg->dest.virt + copied;
			off = (unsigned long)virt & ~XEN_PAGE_MASK;
			len = min(len, (size_t)XEN_PAGE_SIZE - off);
			batch->writeable = true;

			ret = gntdev_get_page(batch, virt, &gfn);
			if (ret < 0)
				return ret;

			op->dest.u.gmfn = gfn;
			op->dest.domid = DOMID_SELF;
			op->dest.offset = off;
		}

		op->len = len;
		copied += len;

		batch->status[batch->nr_ops] = status;
		batch->nr_ops++;
	}

	return 0;
}

static long gntdev_ioctl_grant_copy(struct gntdev_priv *priv, void __user *u)
{
	struct ioctl_gntdev_grant_copy copy;
	struct gntdev_copy_batch batch;
	unsigned int i;
	int ret = 0;

	if (copy_from_user(&copy, u, sizeof(copy)))
		return -EFAULT;

	batch.nr_ops = 0;
	batch.nr_pages = 0;

	for (i = 0; i < copy.count; i++) {
		struct gntdev_grant_copy_segment seg;

		if (copy_from_user(&seg, &copy.segments[i], sizeof(seg))) {
			ret = -EFAULT;
			goto out;
		}

		ret = gntdev_grant_copy_seg(&batch, &seg, &copy.segments[i].status);
		if (ret < 0)
			goto out;

		cond_resched();
	}
	if (batch.nr_ops)
		ret = gntdev_copy(&batch);
	return ret;

  out:
	gntdev_put_pages(&batch);
	return ret;
}

static long gntdev_ioctl(struct file *flip,
			 unsigned int cmd, unsigned long arg)
{
	struct gntdev_priv *priv = flip->private_data;
	void __user *ptr = (void __user *)arg;

	switch (cmd) {
	case IOCTL_GNTDEV_MAP_GRANT_REF:
		return gntdev_ioctl_map_grant_ref(priv, ptr);

	case IOCTL_GNTDEV_UNMAP_GRANT_REF:
		return gntdev_ioctl_unmap_grant_ref(priv, ptr);

	case IOCTL_GNTDEV_GET_OFFSET_FOR_VADDR:
		return gntdev_ioctl_get_offset_for_vaddr(priv, ptr);

	case IOCTL_GNTDEV_SET_UNMAP_NOTIFY:
		return gntdev_ioctl_notify(priv, ptr);

	case IOCTL_GNTDEV_GRANT_COPY:
		return gntdev_ioctl_grant_copy(priv, ptr);

#ifdef CONFIG_XEN_GNTDEV_DMABUF
	case IOCTL_GNTDEV_DMABUF_EXP_FROM_REFS:
		return gntdev_ioctl_dmabuf_exp_from_refs(priv, use_ptemod, ptr);

	case IOCTL_GNTDEV_DMABUF_EXP_WAIT_RELEASED:
		return gntdev_ioctl_dmabuf_exp_wait_released(priv, ptr);

	case IOCTL_GNTDEV_DMABUF_IMP_TO_REFS:
		return gntdev_ioctl_dmabuf_imp_to_refs(priv, ptr);

	case IOCTL_GNTDEV_DMABUF_IMP_RELEASE:
		return gntdev_ioctl_dmabuf_imp_release(priv, ptr);
#endif

	default:
		pr_debug("priv %p, unknown cmd %x\n", priv, cmd);
		return -ENOIOCTLCMD;
	}

	return 0;
}

static int gntdev_mmap(struct file *flip, struct vm_area_struct *vma)
{
	struct gntdev_priv *priv = flip->private_data;
	int index = vma->vm_pgoff;
	int count = vma_pages(vma);
	struct gntdev_grant_map *map;
	int err = -EINVAL;

	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED))
		return -EINVAL;

	pr_debug("map %d+%d at %lx (pgoff %lx)\n",
		 index, count, vma->vm_start, vma->vm_pgoff);

	mutex_lock(&priv->lock);
	map = gntdev_find_map_index(priv, index, count);
	if (!map)
		goto unlock_out;
	if (!atomic_add_unless(&map->in_use, 1, 1))
		goto unlock_out;

	refcount_inc(&map->users);

	vma->vm_ops = &gntdev_vmops;

	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP | VM_MIXEDMAP;

	if (use_ptemod)
		vma->vm_flags |= VM_DONTCOPY;

	vma->vm_private_data = map;
	if (map->flags) {
		if ((vma->vm_flags & VM_WRITE) &&
				(map->flags & GNTMAP_readonly))
			goto out_unlock_put;
	} else {
		map->flags = GNTMAP_host_map;
		if (!(vma->vm_flags & VM_WRITE))
			map->flags |= GNTMAP_readonly;
	}

	map->pages_vm_start = vma->vm_start;

	if (use_ptemod) {
		err = mmu_interval_notifier_insert_locked(
			&map->notifier, vma->vm_mm, vma->vm_start,
			vma->vm_end - vma->vm_start, &gntdev_mmu_ops);
		if (err)
			goto out_unlock_put;

		map->notifier_init = true;
	}
	mutex_unlock(&priv->lock);

	if (use_ptemod) {
		/*
		 * gntdev takes the address of the PTE in find_grant_ptes() and
		 * passes it to the hypervisor in gntdev_map_grant_pages(). The
		 * purpose of the notifier is to prevent the hypervisor pointer
		 * to the PTE from going stale.
		 *
		 * Since this vma's mappings can't be touched without the
		 * mmap_lock, and we are holding it now, there is no need for
		 * the notifier_range locking pattern.
		 */
		mmu_interval_read_begin(&map->notifier);

		err = apply_to_page_range(vma->vm_mm, vma->vm_start,
					  vma->vm_end - vma->vm_start,
					  find_grant_ptes, map);
		if (err) {
			pr_warn("find_grant_ptes() failure.\n");
			goto out_put_map;
		}
	}

	err = gntdev_map_grant_pages(map);
	if (err)
		goto out_put_map;

	if (!use_ptemod) {
		err = vm_map_pages_zero(vma, map->pages, map->count);
		if (err)
			goto out_put_map;
	}

	return 0;

unlock_out:
	mutex_unlock(&priv->lock);
	return err;

out_unlock_put:
	mutex_unlock(&priv->lock);
out_put_map:
	if (use_ptemod)
		unmap_grant_pages(map, 0, map->count);
	gntdev_put_map(priv, map);
	return err;
}

static const struct file_operations gntdev_fops = {
	.owner = THIS_MODULE,
	.open = gntdev_open,
	.release = gntdev_release,
	.mmap = gntdev_mmap,
	.unlocked_ioctl = gntdev_ioctl
};

static struct miscdevice gntdev_miscdev = {
	.minor        = MISC_DYNAMIC_MINOR,
	.name         = "xen/gntdev",
	.fops         = &gntdev_fops,
};

/* ------------------------------------------------------------------ */

static int __init gntdev_init(void)
{
	int err;

	if (!xen_domain())
		return -ENODEV;

	use_ptemod = !xen_feature(XENFEAT_auto_translated_physmap);

	err = misc_register(&gntdev_miscdev);
	if (err != 0) {
		pr_err("Could not register gntdev device\n");
		return err;
	}
	return 0;
}

static void __exit gntdev_exit(void)
{
	misc_deregister(&gntdev_miscdev);
}

module_init(gntdev_init);
module_exit(gntdev_exit);

/* ------------------------------------------------------------------ */
