/*
 * Intel SST Firmware Loader
 *
 * Copyright (C) 2013, Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define DEBUG
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/firmware.h>
#include <linux/export.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/pci.h>

#include <asm/page.h>
#include <asm/pgtable.h>

#include "sst-dsp.h"
#include "sst-dsp-priv.h"

#define SST_HSW_BLOCK_ANY	0xffffffff

static void sst_memcpy32(volatile void __iomem *dest, void *src, u32 bytes)
{
	u32 i;

	/* copy one 32 bit word at a time as 64 bit access is not supported */
	for (i = 0; i < bytes; i += 4)
		memcpy_toio(dest + i, src + i, 4);
}


/* remove module from memory - callers hold locks */
static void block_list_remove(struct sst_dsp *dsp,
	struct list_head *block_list)
{
	struct sst_mem_block *block, *tmp;
	int err;

	/* disable each block  */
	list_for_each_entry(block, block_list, module_list) {

		if (block->ops && block->ops->disable) {
			err = block->ops->disable(block);
			if (err < 0)
				dev_err(dsp->dev,
					"error: cant disable block %d:%d\n",
					block->type, block->index);
		}
	}

	/* mark each block as free */
	list_for_each_entry_safe(block, tmp, block_list, module_list) {
		list_del(&block->module_list);
		list_move(&block->list, &dsp->free_block_list);
		dev_dbg(dsp->dev, "block freed %d:%d at offset 0x%x\n",
			block->type, block->index, block->offset);
	}
}

/* prepare the memory block to receive data from host - callers hold locks */
static int block_list_prepare(struct sst_dsp *dsp,
	struct list_head *block_list)
{
	struct sst_mem_block *block;
	int ret = 0;

	/* enable each block so that's it'e ready for data */
	list_for_each_entry(block, block_list, module_list) {

		if (block->ops && block->ops->enable && !block->users) {
			ret = block->ops->enable(block);
			if (ret < 0) {
				dev_err(dsp->dev,
					"error: cant disable block %d:%d\n",
					block->type, block->index);
				goto err;
			}
		}
	}
	return ret;

err:
	list_for_each_entry(block, block_list, module_list) {
		if (block->ops && block->ops->disable)
			block->ops->disable(block);
	}
	return ret;
}

/* create new generic firmware object */
struct sst_fw *sst_fw_new(struct sst_dsp *dsp, 
	const struct firmware *fw, void *private)
{
	struct sst_fw *sst_fw;
	int err;

	if (!dsp->ops->parse_fw)
		return NULL;

	sst_fw = kzalloc(sizeof(*sst_fw), GFP_KERNEL);
	if (sst_fw == NULL)
		return NULL;

	sst_fw->dsp = dsp;
	sst_fw->private = private;
	sst_fw->size = fw->size;

	/* allocate DMA buffer to store FW data */
	sst_fw->dma_buf = dma_alloc_coherent(dsp->dma_dev, sst_fw->size,
				&sst_fw->dmable_fw_paddr, GFP_DMA | GFP_KERNEL);
	if (!sst_fw->dma_buf) {
		dev_err(dsp->dev, "error: DMA alloc failed\n");
		kfree(sst_fw);
		return NULL;
	}

	/* copy FW data to DMA-able memory */
	memcpy((void *)sst_fw->dma_buf, (void *)fw->data, fw->size);

	/* call core specific FW paser to load FW data into DSP */
	err = dsp->ops->parse_fw(sst_fw);
	if (err < 0) {
		dev_err(dsp->dev, "error: parse fw failed %d\n", err);
		goto parse_err;
	}

	mutex_lock(&dsp->mutex);
	list_add(&sst_fw->list, &dsp->fw_list);
	mutex_unlock(&dsp->mutex);

	return sst_fw;

parse_err:
	dma_free_coherent(dsp->dev, sst_fw->size,
				sst_fw->dma_buf,
				sst_fw->dmable_fw_paddr);
	kfree(sst_fw);
	return NULL;
}
EXPORT_SYMBOL_GPL(sst_fw_new);

int sst_fw_reload(struct sst_fw *sst_fw)
{
	struct sst_dsp *dsp = sst_fw->dsp;
	int ret;

	dev_dbg(dsp->dev, "reloading firmware\n");

	/* call core specific FW paser to load FW data into DSP */
	ret = dsp->ops->parse_fw(sst_fw);
	if (ret < 0)
		dev_err(dsp->dev, "error: parse fw failed %d\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(sst_fw_reload);

void sst_fw_unload(struct sst_fw *sst_fw)
{
        struct sst_dsp *dsp = sst_fw->dsp;
        struct sst_module *module, *tmp;

        dev_dbg(dsp->dev, "unloading firmware\n");

        mutex_lock(&dsp->mutex);
        list_for_each_entry_safe(module, tmp, &dsp->module_list, list) {
                if (module->sst_fw == sst_fw) {
                        block_module_remove(module);
                        list_del(&module->list);
                        kfree(module);
                }
        }

        mutex_unlock(&dsp->mutex);
}
EXPORT_SYMBOL_GPL(sst_fw_unload);

/* free single firmware object */
void sst_fw_free(struct sst_fw *sst_fw)
{
	struct sst_dsp *dsp = sst_fw->dsp;

	mutex_lock(&dsp->mutex);
	list_del(&sst_fw->list);
	mutex_unlock(&dsp->mutex);

	dma_free_coherent(dsp->dma_dev, sst_fw->size, sst_fw->dma_buf,
			sst_fw->dmable_fw_paddr);
	kfree(sst_fw);
}
EXPORT_SYMBOL_GPL(sst_fw_free);

/* free all firmware objects */
void sst_fw_free_all(struct sst_dsp *dsp)
{
	struct sst_fw *sst_fw, *t;

	mutex_lock(&dsp->mutex);
	list_for_each_entry_safe(sst_fw, t, &dsp->fw_list, list) {

		list_del(&sst_fw->list);
		dma_free_coherent(dsp->dev, sst_fw->size, sst_fw->dma_buf,
			sst_fw->dmable_fw_paddr);
		kfree(sst_fw);
	}
	mutex_unlock(&dsp->mutex);
}
EXPORT_SYMBOL_GPL(sst_fw_free_all);

/* create a new SST generic module from FW template */
struct sst_module *sst_module_new(struct sst_fw *sst_fw,
	struct sst_module_template *template, void *private)
{
	struct sst_dsp *dsp = sst_fw->dsp;
	struct sst_module *sst_module;

	sst_module = kzalloc(sizeof(*sst_module), GFP_KERNEL);
	if (sst_module == NULL)
		return NULL;

	sst_module->id = template->id;
	sst_module->dsp = dsp;
	sst_module->sst_fw = sst_fw;

	memcpy(&sst_module->s, &template->s, sizeof(struct sst_module_data));
	memcpy(&sst_module->p, &template->p, sizeof(struct sst_module_data));

	INIT_LIST_HEAD(&sst_module->block_list);

	mutex_lock(&dsp->mutex);
	list_add(&sst_module->list, &dsp->module_list);
	mutex_unlock(&dsp->mutex);

	return sst_module;
}
EXPORT_SYMBOL_GPL(sst_module_new);

/* free firmware module and remove from available list */
void sst_module_free(struct sst_module *sst_module)
{
	struct sst_dsp *dsp = sst_module->dsp;

	mutex_lock(&dsp->mutex);
	list_del(&sst_module->list);
	mutex_unlock(&dsp->mutex);

	kfree(sst_module);
}
EXPORT_SYMBOL_GPL(sst_module_free);

struct sst_module_runtime *sst_module_runtime_new(struct sst_module *module,
	int id, void *private)
{
	struct sst_dsp *dsp = module->dsp;
	struct sst_module_runtime *runtime;

	runtime = kzalloc(sizeof(*runtime), GFP_KERNEL);
	if (runtime == NULL)
		return NULL;

	runtime->id = id;
	runtime->dsp = dsp;
	runtime->module = module;
	INIT_LIST_HEAD(&runtime->block_list);

	mutex_lock(&dsp->mutex);
	list_add(&runtime->list, &module->runtime_list);
	mutex_unlock(&dsp->mutex);

	return runtime;
}
EXPORT_SYMBOL_GPL(sst_module_runtime_new);

void sst_module_runtime_free(struct sst_module_runtime *runtime)
{
	struct sst_dsp *dsp = runtime->dsp;

	mutex_lock(&dsp->mutex);
	list_del(&runtime->list);
	mutex_unlock(&dsp->mutex);

	kfree(runtime);
}
EXPORT_SYMBOL_GPL(sst_module_runtime_free);

static struct sst_mem_block *find_block(struct sst_dsp *dsp,
	struct sst_block_allocator *ba)
{
	struct sst_mem_block *block;

	list_for_each_entry(block, &dsp->free_block_list, list) {
		if (block->type == ba->type && block->offset == ba->offset)
			return block;
	}

	return NULL;
}

/* Block allocator must be on block boundary */
static int block_alloc_contiguous(struct sst_dsp *dsp,
	struct sst_block_allocator *ba, struct list_head *block_list)
{
	struct list_head tmp = LIST_HEAD_INIT(tmp);
	struct sst_mem_block *block;
	u32 block_start = SST_HSW_BLOCK_ANY;
	int size = ba->size, offset = ba->offset;

	while (ba->size > 0) {

		block = find_block(dsp, ba);
		if (!block) {
			list_splice(&tmp, &dsp->free_block_list);

			ba->size = size;
			ba->offset = offset;
			return -ENOMEM;
		}

		list_move_tail(&block->list, &tmp);
		ba->offset += block->size;
		ba->size -= block->size;
	}
	ba->size = size;
	ba->offset = offset;

	list_for_each_entry(block, &tmp, list) {

		if (block->offset < block_start)
			block_start = block->offset;

		list_add(&block->module_list, block_list);

		dev_dbg(dsp->dev, "block allocated %d:%d at offset 0x%x\n",
			block->type, block->index, block->offset);
	}

	list_splice(&tmp, &dsp->used_block_list);

	return 0;
}

/* allocate first free DSP blocks for data - callers hold locks */
static int block_alloc(struct sst_dsp *dsp, struct sst_block_allocator *ba,
	struct list_head *block_list)
{
	struct sst_mem_block *block, *tmp;
	int ret = 0;

	if (ba->size == 0)
		return 0;

	/* find first free whole blocks that can hold module */
	list_for_each_entry_safe(block, tmp, &dsp->free_block_list, list) {

		/* ignore blocks with wrong type */
		if (block->type != ba->type)
			continue;

		if (ba->size > block->size)
			continue;

		ba->offset = block->offset;
		block->bytes_used = ba->size % block->size;
		list_add(&block->module_list, block_list);
		list_move(&block->list, &dsp->used_block_list);
		dev_dbg(dsp->dev, "block allocated %d:%d at offset 0x%x\n",
			block->type, block->index, block->offset);
		return 0;
	}

	/* then find free multiple blocks that can hold module */
	list_for_each_entry_safe(block, tmp, &dsp->free_block_list, list) {

		/* ignore blocks with wrong type */
		if (block->type != ba->type)
			continue;

		/* do we span > 1 blocks */
		if (ba->size > block->size) {

			/* align ba to block boundary */
			ba->offset = block->offset;

			ret = block_alloc_contiguous(dsp, ba, block_list);
			if (ret == 0)
				return ret;

		}
	}

	/* not enough free block space */
	return -ENOMEM;
}

int sst_alloc_blocks(struct sst_dsp *dsp, struct sst_block_allocator *ba,
	struct list_head *block_list)
{
	int ret;

	dev_dbg(dsp->dev, "block request 0x%x bytes at offset 0x%x type %d\n",
		ba->size, ba->offset, ba->type);

	mutex_lock(&dsp->mutex);

	ret = block_alloc(dsp, ba, block_list);
	if (ret < 0) {
		dev_err(dsp->dev, "error: can't alloc blocks %d\n", ret);
		goto out;
	}

	/* prepare DSP blocks for module usage */
	ret = block_list_prepare(dsp, block_list);
	if (ret < 0)
		dev_err(dsp->dev, "error: prepare failed\n");

out:
	mutex_unlock(&dsp->mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(sst_alloc_blocks);

int sst_free_blocks(struct sst_dsp *dsp, struct list_head *block_list)
{
	mutex_lock(&dsp->mutex);
	block_list_remove(dsp, block_list);
	mutex_unlock(&dsp->mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(sst_free_blocks);

/* allocate memory blocks for static module addresses - callers hold locks */
static int block_alloc_fixed(struct sst_dsp *dsp, struct sst_block_allocator *ba,
	struct list_head *block_list)
{
	struct sst_mem_block *block, *tmp;
	u32 end = ba->offset + ba->size, block_end;
	int err;

	/* only IRAM/DRAM blocks are managed */
	if (ba->type != SST_MEM_IRAM && ba->type != SST_MEM_DRAM)
		return 0;

	/* are blocks already attached to this module */
	list_for_each_entry_safe(block, tmp, block_list, module_list) {

		/* ignore blocks with wrong type */
		if (block->type != ba->type)
			continue;

		block_end = block->offset + block->size;

		/* find block that holds section */
		if (ba->offset >= block->offset && end <= block_end)
			return 0;

		/* does block span more than 1 section */
		if (ba->offset >= block->offset && ba->offset < block_end) {

			/* align ba to block boundary */
			ba->size -= block_end - ba->offset;
			ba->offset = block_end;
			err = block_alloc_contiguous(dsp, ba, block_list);
			if (err < 0)
				return -ENOMEM;

			/* module already owns blocks */
			return 0;
		}
	}

	/* find first free blocks that can hold section in free list */
	list_for_each_entry_safe(block, tmp, &dsp->free_block_list, list) {
		block_end = block->offset + block->size;

		/* ignore blocks with wrong type */
		if (block->type != ba->type)
			continue;

		/* find block that holds section */
		if (ba->offset >= block->offset && end <= block_end) {

			/* add block */
			list_move(&block->list, &dsp->used_block_list);
			list_add(&block->module_list, block_list);
			dev_dbg(dsp->dev, "block allocated %d:%d at offset 0x%x\n",
				block->type, block->index, block->offset);
			return 0;
		}

		/* does block span more than 1 section */
		if (ba->offset >= block->offset && ba->offset < block_end) {

			/* align ba to block boundary */
			ba->offset = block->offset;

			err = block_alloc_contiguous(dsp, ba, block_list);
			if (err < 0)
				return -ENOMEM;
			return 0;
		}
	}

	return -ENOMEM;
}

/* Load fixed module data into DSP memory blocks */
int sst_module_alloc_blocks(struct sst_module *module)
{
	struct sst_dsp *dsp = module->dsp;
	struct sst_fw *sst_fw = module->sst_fw;
	struct sst_block_allocator ba;
	int ret;

	ba.size = module->size;
	ba.type = module->type;
	ba.offset = module->offset;

	dev_dbg(dsp->dev, "block request 0x%x bytes at offset 0x%x type %d\n",
		ba.size, ba.offset, ba.type);

	mutex_lock(&dsp->mutex);

	/* alloc blocks that includes this section */
	ret = block_alloc_fixed(dsp, &ba, &module->block_list);
	if (ret < 0) {
		dev_err(dsp->dev,
			"error: no free blocks for section at offset 0x%x size 0x%x\n",
			module->offset, module->size);
		mutex_unlock(&dsp->mutex);
		return -ENOMEM;
	}

	/* prepare DSP blocks for module copy */
	ret = block_list_prepare(dsp, &module->block_list);
	if (ret < 0) {
		dev_err(dsp->dev, "error: fw module prepare failed\n");
		goto err;
	}

	/* copy partial module data to blocks */
	sst_memcpy32(dsp->addr.lpe + data->offset, data->data, data->size);

	mutex_unlock(&dsp->mutex);
	return 0;

err:
	block_list_remove(dsp, &module->block_list);
	mutex_unlock(&dsp->mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(sst_module_alloc_blocks);

/* Unload entire module from DSP memory */
int sst_module_free_blocks(struct sst_module *module)
{
	struct sst_dsp *dsp = module->dsp;

	mutex_lock(&dsp->mutex);
	block_list_remove(dsp, &module->block_list);
	mutex_unlock(&dsp->mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(sst_module_free_blocks);
	mutex_unlock(&dsp->mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(sst_block_module_remove);

/* register a DSP memory block for use with FW based modules */
struct sst_mem_block *sst_mem_block_register(struct sst_dsp *dsp, u32 offset,
	u32 size, enum sst_mem_type type, struct sst_block_ops *ops, u32 index,
	void *private)
{
	struct sst_mem_block *block;

	block = kzalloc(sizeof(*block), GFP_KERNEL);
	if (block == NULL)
		return NULL;

	block->offset = offset;
	block->size = size;
	block->index = index;
	block->type = type;
	block->dsp = dsp;
	block->private = private;
	block->ops = ops;

	mutex_lock(&dsp->mutex);
	list_add(&block->list, &dsp->free_block_list);
	mutex_unlock(&dsp->mutex);

	return block;
}
EXPORT_SYMBOL_GPL(sst_mem_block_register);

/* unregister all DSP memory blocks */
void sst_mem_block_unregister_all(struct sst_dsp *dsp)
{
	struct sst_mem_block *block, *tmp;

	mutex_lock(&dsp->mutex);

	/* unregister used blocks */
	list_for_each_entry_safe(block, tmp, &dsp->used_block_list, list) {
		list_del(&block->list);
		kfree(block);
	}

	/* unregister free blocks */
	list_for_each_entry_safe(block, tmp, &dsp->free_block_list, list) {
		list_del(&block->list);
		kfree(block);
	}

	mutex_unlock(&dsp->mutex);
}
EXPORT_SYMBOL_GPL(sst_mem_block_unregister_all);

/* allocate scratch buffer blocks */
struct sst_module *sst_mem_block_alloc_scratch(struct sst_dsp *dsp)
{
	struct sst_module *sst_module, *scratch;
	struct sst_mem_block *block, *tmp;
	u32 block_size;
	int ret = 0;

	scratch = kzalloc(sizeof(struct sst_module), GFP_KERNEL);
	if (scratch == NULL)
		return NULL;

	mutex_lock(&dsp->mutex);

	/* calculate required scratch size */
	list_for_each_entry(sst_module, &dsp->module_list, list) {
		if (scratch->s.size < sst_module->s.size)
			scratch->s.size = sst_module->s.size;
	}

	dev_dbg(dsp->dev, "scratch buffer required is %d bytes\n",
		scratch->s.size);

	/* init scratch module */
	scratch->dsp = dsp;
	scratch->s.type = SST_MEM_DRAM;
	scratch->s.data_type = SST_DATA_S;
	INIT_LIST_HEAD(&scratch->block_list);

	/* check free blocks before looking at used blocks for space */
	if (!list_empty(&dsp->free_block_list))
		block = list_first_entry(&dsp->free_block_list,
			struct sst_mem_block, list);
	else
		block = list_first_entry(&dsp->used_block_list,
			struct sst_mem_block, list);
	block_size = block->size;

	/* allocate blocks for module scratch buffers */
	dev_dbg(dsp->dev, "allocating scratch blocks\n");
	ret = block_alloc(scratch, &scratch->s);
	if (ret < 0) {
		dev_err(dsp->dev, "error: can't alloc scratch blocks\n");
		goto err;
	}

	/* assign the same offset of scratch to each module */
	list_for_each_entry(sst_module, &dsp->module_list, list)
		sst_module->s.offset = scratch->s.offset;

	mutex_unlock(&dsp->mutex);
	return scratch;

err:
	list_for_each_entry_safe(block, tmp, &scratch->block_list, module_list)
		list_del(&block->module_list);
	mutex_unlock(&dsp->mutex);
	return NULL;
}
EXPORT_SYMBOL_GPL(sst_mem_block_alloc_scratch);

/* free all scratch blocks */
void sst_mem_block_free_scratch(struct sst_dsp *dsp,
	struct sst_module *scratch)
{
	struct sst_mem_block *block, *tmp;

	mutex_lock(&dsp->mutex);

	list_for_each_entry_safe(block, tmp, &scratch->block_list, module_list)
		list_del(&block->module_list);

	mutex_unlock(&dsp->mutex);
}
EXPORT_SYMBOL_GPL(sst_mem_block_free_scratch);

/* get a module from it's unique ID */
struct sst_module *sst_module_get_from_id(struct sst_dsp *dsp, u32 id)
{
	struct sst_module *module;

	mutex_lock(&dsp->mutex);

	list_for_each_entry(module, &dsp->module_list, list) {
		if (module->id == id) {
			mutex_unlock(&dsp->mutex);
			return module;
		}
	}

	mutex_unlock(&dsp->mutex);
	return NULL;
}
EXPORT_SYMBOL_GPL(sst_module_get_from_id);
