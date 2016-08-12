/**
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 * author: ZhiChao Yu zhichao.yu@rock-chips.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/slab.h>
#include <linux/err.h>
#if defined(CONFIG_ION_ROCKCHIP)
#include <linux/rockchip_ion.h>
#endif
#include "dsp_dbg.h"
#include "dsp_loader.h"

/* 1MB memory region for dsp running code on DDR */
#define DSP_TEXT_MEM_SIZE          (0x100000)
#define DSP_TEXT_OFFSET_MASK       (0xfffff)

#define FIRMWARE_MAGIC_SIZE        16
#define FIRMWARE_VERSION_SIZE      16
#define FIRMWARE_RESERVE_SIZE      60
#define FIRMWARE_MAX_IMAGES        8

struct dsp_section_header {
	u32 type;
	u32 size;
	u32 load_address;
};

struct dsp_image_header {
	u32 id;
	char name[DSP_IMAGE_NAME_SIZE];
	u32 section_count;
};

struct dsp_firmware_header {
	char magic[FIRMWARE_MAGIC_SIZE];
	char version[FIRMWARE_VERSION_SIZE];
	u32 image_count;
	u32 image_size[FIRMWARE_MAX_IMAGES];
	u8 reserve[FIRMWARE_RESERVE_SIZE];
};

static int dsp_loader_get_image_by_id(struct dsp_loader *loader, u32 id,
				      struct dsp_image **image_out)
{
	struct list_head *pos, *n;

	list_for_each_safe(pos, n, &loader->images) {
		struct dsp_image *image = container_of(pos,
				struct dsp_image, list_node);

		if (image->id == id) {
			(*image_out) = image;
			return 0;
		}
	}

	(*image_out) = NULL;
	return -EEXIST;
}

static int dsp_loader_image_create(struct dsp_loader *loader, u32 id,
				   char *name, struct dsp_image **image_out)
{
	int ret = 0;
	struct dsp_image *image;

	dsp_debug_enter();

	image = kzalloc(sizeof(*image), GFP_KERNEL);
	if (!image) {
		ret = -ENOMEM;
		dsp_err("cannot alloc mem for dsp image\n");
		goto out;
	}

	image->id = id;
	memcpy(image->name, name, DSP_IMAGE_NAME_SIZE);
	(*image_out) = image;
out:
	dsp_debug_leave();
	return ret;
}

static int dsp_loader_image_destroy(struct dsp_image *image)
{
	int i;

	dsp_debug_enter();

	for (i = 0; i < DSP_IMAGE_MAX_SECTION; i++) {
		struct dsp_section *section = &image->sections[i];

		if (section->valid)
			kfree(section->src);
	}

	kfree(image);

	dsp_debug_leave();
	return 0;
}

static int dsp_loader_image_parse(struct dsp_loader *loader,
				  u8 *image_data, u32 image_size)
{
	int ret = 0;
	int idx;
	u32 offset = 0;
	struct dsp_image *image;
	struct dsp_image_header *image_hdr;

	dsp_debug_enter();

	image_hdr = (struct dsp_image_header *)image_data;
	offset += sizeof(*image_hdr);
	dsp_debug(DEBUG_LOADER,
		  "parse image, id=%d, name=%s, count=%d\n",
		  image_hdr->id, image_hdr->name, image_hdr->section_count);

	ret = dsp_loader_image_create(loader, image_hdr->id,
				      image_hdr->name, &image);
	if (ret)
		goto out;

	for (idx = 0; idx < image_hdr->section_count; idx++) {
		struct dsp_section_header *section_hdr;
		struct dsp_section *section = &image->sections[idx];

		section_hdr = (struct dsp_section_header *)
			      (image_data + offset);
		offset += sizeof(*section_hdr);

		section->type = section_hdr->type;
		section->size = section_hdr->size;
		section->dst = (void *)section_hdr->load_address;

		dsp_debug(DEBUG_LOADER,
			  "image section, type=%d, size=%d, load_addr=0x%08x\n",
			  section->type, section->size, (u32)section->dst);

		section->src = kzalloc(section->size, GFP_KERNEL);
		if (!section->src) {
			dsp_err("cannot alloc mem for section\n");
			ret = -ENOMEM;
			goto out;
		}

		memcpy(section->src, image_data + offset, section->size);
		offset += section->size;

		/* TODO change to phys address */

		section->valid = 1;
	}

	if (offset != image_size) {
		dsp_err("parse image failed, name=%s, offset=%d\n",
			image->name, offset);
		ret = -EFAULT;
		goto out;
	}

	list_add_tail(&image->list_node, &loader->images);

out:
	if (ret)
		dsp_loader_image_destroy(image);
	dsp_debug_leave();
	return ret;
}

/*
 * dsp_loader_prepare_image - DSP firmware is parsed here,
 * image information can be extract from firmware file.
 * A firmware file layout is as follows:
 *
 * +--------+----------+----------+-----------+----------+-----------+
 * | fw_hdr | img1_hdr | sec1_hdr | sec1_data | sec2_hdr | sec2_data +->
 * +--------+----------+----------+-----------+----------+-----------+
 * Continued
 *          +----------+----------+-----------+----------+-----------+
 * +--------> img2_hdr | sec3_hdr | sec3_data | sec4_hdr | sec4_data |
 *          +----------+----------+-----------+----------+-----------+
 *
 * @fw: firmware prt
 * @loader: loader ptr
 */
static int dsp_loader_prepare_image(const struct firmware *fw,
				    struct dsp_loader *loader)
{
	int i;
	int ret = 0;
	u32 offset = 0;
	struct dsp_firmware_header firmware_hdr;

	dsp_debug_enter();

	memset(&firmware_hdr, 0, sizeof(firmware_hdr));

	memcpy(&firmware_hdr, fw->data, sizeof(firmware_hdr));
	offset += sizeof(firmware_hdr);

	dsp_debug(DEBUG_LOADER,
		  "firmware: magic - %s, version - %s, count - %d\n",
		  firmware_hdr.magic, firmware_hdr.version,
		  firmware_hdr.image_count);

	for (i = 0; i < firmware_hdr.image_count; i++) {
		u32 image_size = firmware_hdr.image_size[i];
		u8 *image_data;

		image_data = kzalloc(image_size, GFP_KERNEL);
		if (!image_data) {
			dsp_err("cannot alloc mem for image data\n");
			ret = -ENOMEM;
			goto out;
		}

		memcpy(image_data, fw->data + offset, image_size);
		offset += image_size;

		dsp_loader_image_parse(loader, image_data, image_size);
		kfree(image_data);
	}
out:
	dsp_debug_leave();
	return ret;
}

/*
 * dsp_loader_load_image - loader DSP image by image id
 *
 * @loader: DSP loader ptr
 * @id: image id
 */
static int dsp_loader_load_image(struct dsp_loader *loader, u32 id)
{
	int ret = 0;
	int i;
	u32 offset = 0;
	struct dsp_image *image;

	dsp_debug_enter();

	ret = dsp_loader_get_image_by_id(loader, id, &image);
	if (!image) {
		dsp_err("cannot found image for dsp, id=%d\n", id);
		ret = -EEXIST;
		goto out;
	}

	for (i = 0; i < DSP_IMAGE_MAX_SECTION; i++) {
		struct dsp_section *section = &image->sections[i];

		if (!section->valid)
			continue;

		switch (section->type) {
		case DSP_IMAGE_CODE_INTERNAL:
			loader->dma->transfer_code(loader->dma, section->src,
						   section->dst, section->size);
			dsp_debug(DEBUG_LOADER,
				  "load section, src=0x%p, dst=0x%p\n",
				  section->src, section->dst);
			break;
		case DSP_IMAGE_CODE_EXTERNAL:
			offset = (u32)section->dst & DSP_TEXT_OFFSET_MASK;
			memcpy(loader->external_text + offset,
			       section->src, section->size);
			dsp_debug(DEBUG_LOADER,
				  "load section, src=0x%p, dst=0x%p\n",
				  section->src, section->dst);
			break;
		case DSP_IMAGE_DATA_INTERNAL:
			loader->dma->transfer_data(loader->dma, section->src,
						   section->dst, section->size);
			dsp_debug(DEBUG_LOADER,
				  "load section, src=0x%p, dst=0x%p\n",
				  section->src, section->dst);
			break;
		case DSP_IMAGE_DATA_EXTERNAL:
			offset = (u32)section->dst & DSP_TEXT_OFFSET_MASK;
			memcpy(loader->external_text + offset,
			       section->src, section->size);
			dsp_debug(DEBUG_LOADER,
				  "load section, src=0x%p, dst=0x%p\n",
				  section->src, section->dst);
			break;
		default:
			dsp_err("unknown section type\n");
			break;
		}
	}

	dsp_debug(DEBUG_LOADER, "dsp image loaded, name=%s\n", image->name);
out:
	dsp_debug_leave();
	return ret;
}

void dsp_loader_request_firmware(const struct firmware *fw, void *context)
{
	const char *magic = "#RKCPDSPFW#";
	struct dsp_loader *loader = context;

	dsp_debug_enter();

	/* Check firmware magic, must be #RKCPDSPFW# */
	if (memcmp(fw->data, magic, strlen(magic))) {
		dsp_err("invalid dsp firmware\n");
		goto out;
	}

	dsp_loader_prepare_image(fw, loader);
out:
	release_firmware(fw);
	dsp_debug_leave();
}

int dsp_loader_create(struct dsp_dma *dma, struct dsp_loader **loader_out)
{
	int ret = 0;
	struct ion_client *ion_client;
	struct ion_handle *hdl;
	struct dsp_loader *loader;

	dsp_debug_enter();

	loader = kzalloc(sizeof(*loader), GFP_KERNEL);
	if (!loader) {
		dsp_err("cannot alloc mem for loader\n");
		ret = -ENOMEM;
		goto out;
	}

	ion_client = rockchip_ion_client_create("dsp");
	if (IS_ERR(ion_client)) {
		ret = PTR_ERR(ion_client);
		dsp_err("cannot create ion client\n");
		goto out;
	}
	hdl = ion_alloc(ion_client, (size_t)DSP_TEXT_MEM_SIZE, 0,
			ION_HEAP(ION_CARVEOUT_HEAP_ID), 0);
	if (IS_ERR(hdl)) {
		dsp_err("cannnot alloc memory for dsp to run\n");
		ret = PTR_ERR(hdl);
		goto out;
	}
	loader->external_text = ion_map_kernel(ion_client, hdl);

	loader->load_image = dsp_loader_load_image;
	loader->dma = dma;
	INIT_LIST_HEAD(&loader->images);

	(*loader_out) = loader;
out:
	if (ret) {
		kfree(loader);
		(*loader_out) = NULL;
	}
	dsp_debug_leave();
	return ret;
}

int dsp_loader_destroy(struct dsp_loader *loader)
{
	struct list_head *pos, *n;

	if (!loader)
		goto out;

	dsp_debug_enter();
	list_for_each_safe(pos, n, &loader->images) {
		struct dsp_image *image = container_of(pos,
				struct dsp_image, list_node);

		dsp_loader_image_destroy(image);
	}

	kfree(loader);
out:
	dsp_debug_leave();
	return 0;
}
