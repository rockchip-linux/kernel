/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:Mark Yao <mark.yao@rock-chips.com>
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

#ifndef _ROCKCHIP_DRM_GEM_H
#define _ROCKCHIP_DRM_GEM_H

#define to_rockchip_gem_obj(x) container_of(x, struct rockchip_gem_object, base)
#define IS_NONCONTIG_BUFFER(f)          (f & ROCKCHIP_BO_NONCONTIG)

struct rockchip_gem_object {
	struct drm_gem_object base;
	unsigned int flags;

	void *kvaddr;
	dma_addr_t dma_addr;
	struct dma_attrs dma_attrs;
	struct ion_handle *handle;
};

struct sg_table *rockchip_gem_prime_get_sg_table(struct drm_gem_object *obj);
struct drm_gem_object *
rockchip_gem_prime_import_sg_table(struct drm_device *dev, size_t size,
				   struct sg_table *sgt);
void *rockchip_gem_prime_vmap(struct drm_gem_object *obj);
void rockchip_gem_prime_vunmap(struct drm_gem_object *obj, void *vaddr);

/* drm driver mmap file operations */
int rockchip_gem_mmap(struct file *filp, struct vm_area_struct *vma);

/* mmap a gem object to userspace. */
int rockchip_gem_mmap_buf(struct drm_gem_object *obj,
			  struct vm_area_struct *vma);

struct rockchip_gem_object *
	rockchip_gem_create_object(struct drm_device *drm, unsigned int size,
				   bool alloc_kmap);
void rockchip_drm_gem_destroy(struct rockchip_gem_object *rockchip_gem_obj);

void rockchip_gem_free_object(struct drm_gem_object *obj);

int rockchip_gem_dumb_create(struct drm_file *file_priv,
			     struct drm_device *dev,
			     struct drm_mode_create_dumb *args);
int rockchip_gem_dumb_map_offset(struct drm_file *file_priv,
				 struct drm_device *dev, uint32_t handle,
				 uint64_t *offset);
int rockchip_drm_gem_dumb_destroy(struct drm_file *file_priv,
				  struct drm_device *dev,
				  unsigned int handle);
int rockchip_drm_gem_object_mmap(struct drm_gem_object *obj,
				 struct vm_area_struct *vma);
struct drm_gem_object *rockchip_drm_gem_prime_import(struct drm_device *dev,
					    struct dma_buf *dma_buf, int prime_fd);
int rockchip_drm_gem_prime_fd_to_handle(struct drm_device *dev,
					struct drm_file *file_priv,
					int prime_fd, uint32_t *handle);
extern struct dma_buf_ops drm_gem_prime_dmabuf_ops;
extern int drm_prime_add_buf_handle(struct drm_prime_file_private *prime_fpriv,
				    struct dma_buf *dma_buf, uint32_t handle);
#endif /* _ROCKCHIP_DRM_GEM_H */
