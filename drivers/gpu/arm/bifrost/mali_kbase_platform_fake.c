// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2011-2014, 2016-2017, 2020-2022 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/string.h>

/*
 * This file is included only for type definitions and functions belonging to
 * specific platform folders. Do not add dependencies with symbols that are
 * defined somewhere else.
 */
#include <mali_kbase_config.h>

#ifndef CONFIG_OF

#define PLATFORM_CONFIG_RESOURCE_COUNT 4

static struct platform_device *mali_device;

/**
 * kbasep_config_parse_io_resources - Convert data in struct kbase_io_resources
 * struct to Linux-specific resources
 * @io_resources:      Input IO resource data
 * @linux_resources:  Pointer to output array of Linux resource structures
 *
 * Function converts data in struct kbase_io_resources struct to an array of Linux resource structures. Note that function
 * assumes that size of linux_resource array is at least PLATFORM_CONFIG_RESOURCE_COUNT.
 * Resources are put in fixed order: I/O memory region, job IRQ, MMU IRQ, GPU IRQ.
 */
static void kbasep_config_parse_io_resources(const struct kbase_io_resources *io_resources, struct resource *const linux_resources)
{
	if (!io_resources || !linux_resources) {
		pr_err("%s: couldn't find proper resources\n", __func__);
		return;
	}

	memset(linux_resources, 0, PLATFORM_CONFIG_RESOURCE_COUNT * sizeof(struct resource));

	linux_resources[0].start = io_resources->io_memory_region.start;
	linux_resources[0].end   = io_resources->io_memory_region.end;
	linux_resources[0].flags = IORESOURCE_MEM;

	linux_resources[1].start = io_resources->job_irq_number;
	linux_resources[1].end   = io_resources->job_irq_number;
	linux_resources[1].flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL;

	linux_resources[2].start = io_resources->mmu_irq_number;
	linux_resources[2].end   = io_resources->mmu_irq_number;
	linux_resources[2].flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL;

	linux_resources[3].start = io_resources->gpu_irq_number;
	linux_resources[3].end   = io_resources->gpu_irq_number;
	linux_resources[3].flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL;
}

int kbase_platform_register(void)
{
	struct kbase_platform_config *config;
	struct resource resources[PLATFORM_CONFIG_RESOURCE_COUNT];
	int err;

	config = kbase_get_platform_config(); /* declared in midgard/mali_kbase_config.h but defined in platform folder */
	if (config == NULL) {
		pr_err("%s: couldn't get platform config\n", __func__);
		return -ENODEV;
	}

	mali_device = platform_device_alloc("mali", 0);
	if (mali_device == NULL)
		return -ENOMEM;

	kbasep_config_parse_io_resources(config->io_resources, resources);
	err = platform_device_add_resources(mali_device, resources, PLATFORM_CONFIG_RESOURCE_COUNT);
	if (err) {
		platform_device_put(mali_device);
		mali_device = NULL;
		return err;
	}

	err = platform_device_add(mali_device);
	if (err) {
		platform_device_unregister(mali_device);
		mali_device = NULL;
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(kbase_platform_register);

void kbase_platform_unregister(void)
{
	if (mali_device)
		platform_device_unregister(mali_device);
}
EXPORT_SYMBOL(kbase_platform_unregister);

#endif /* CONFIG_OF */
