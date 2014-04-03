/*
 * OF helpers for IOMMU
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/export.h>
#include <linux/limits.h>
#include <linux/of.h>
#include <linux/of_iommu.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

static DEFINE_MUTEX(iommus_lock);
static LIST_HEAD(iommus_list);

void iommu_add(struct iommu *iommu)
{
	INIT_LIST_HEAD(&iommu->list);
	mutex_lock(&iommus_lock);
	list_add_tail(&iommu->list, &iommus_list);
	mutex_unlock(&iommus_lock);
}

void iommu_del(struct iommu *iommu)
{
	INIT_LIST_HEAD(&iommu->list);
	mutex_lock(&iommus_lock);
	list_del(&iommu->list);
	mutex_unlock(&iommus_lock);
}

static struct iommu *of_find_iommu_by_node(struct device_node *np)
{
	struct iommu *iommu;

	mutex_lock(&iommus_lock);
	list_for_each_entry(iommu, &iommus_list, list) {
		if (iommu->dev->of_node == np) {
			mutex_unlock(&iommus_lock);
			return iommu;
		}
	}
	mutex_unlock(&iommus_lock);

	return NULL;
}

/**
 * of_get_dma_window - Parse *dma-window property and returns 0 if found.
 *
 * @dn: device node
 * @prefix: prefix for property name if any
 * @index: index to start to parse
 * @busno: Returns busno if supported. Otherwise pass NULL
 * @addr: Returns address that DMA starts
 * @size: Returns the range that DMA can handle
 *
 * This supports different formats flexibly. "prefix" can be
 * configured if any. "busno" and "index" are optionally
 * specified. Set 0(or NULL) if not used.
 */
int of_get_dma_window(struct device_node *dn, const char *prefix, int index,
		      unsigned long *busno, dma_addr_t *addr, size_t *size)
{
	const __be32 *dma_window, *end;
	int bytes, cur_index = 0;
	char propname[NAME_MAX], addrname[NAME_MAX], sizename[NAME_MAX];

	if (!dn || !addr || !size)
		return -EINVAL;

	if (!prefix)
		prefix = "";

	snprintf(propname, sizeof(propname), "%sdma-window", prefix);
	snprintf(addrname, sizeof(addrname), "%s#dma-address-cells", prefix);
	snprintf(sizename, sizeof(sizename), "%s#dma-size-cells", prefix);

	dma_window = of_get_property(dn, propname, &bytes);
	if (!dma_window)
		return -ENODEV;
	end = dma_window + bytes / sizeof(*dma_window);

	while (dma_window < end) {
		u32 cells;
		const void *prop;

		/* busno is one cell if supported */
		if (busno)
			*busno = be32_to_cpup(dma_window++);

		prop = of_get_property(dn, addrname, NULL);
		if (!prop)
			prop = of_get_property(dn, "#address-cells", NULL);

		cells = prop ? be32_to_cpup(prop) : of_n_addr_cells(dn);
		if (!cells)
			return -EINVAL;
		*addr = of_read_number(dma_window, cells);
		dma_window += cells;

		prop = of_get_property(dn, sizename, NULL);
		cells = prop ? be32_to_cpup(prop) : of_n_size_cells(dn);
		if (!cells)
			return -EINVAL;
		*size = of_read_number(dma_window, cells);
		dma_window += cells;

		if (cur_index++ == index)
			break;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(of_get_dma_window);

int of_iommu_attach(struct device *dev)
{
	const __be32 *cur, *end;
	struct of_phandle_args args;

	of_property_for_each_phandle_with_args(dev->of_node, "iommus",
				       "#iommu-cells", 0, args, cur, end) {
		set_dma_ops(dev, ((struct dma_map_ops *)-ENXIO));
		if (!of_find_iommu_by_node(args.np))
			return -EPROBE_DEFER;
	}

	return 0;
}
