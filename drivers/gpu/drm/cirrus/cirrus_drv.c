/*
 * Copyright 2012 Red Hat <mjg@redhat.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License version 2. See the file COPYING in the main
 * directory of this archive for more details.
 *
 * Authors: Matthew Garrett
 *          Dave Airlie
 */
#include <linux/module.h>
#include <linux/console.h>
#include <drm/drmP.h>

#include "cirrus_drv.h"

int cirrus_modeset = -1;

MODULE_PARM_DESC(modeset, "Disable/Enable modesetting");
module_param_named(modeset, cirrus_modeset, int, 0400);

/*
 * This is the generic driver code. This binds the driver to the drm core,
 * which then performs further device association and calls our graphics init
 * functions
 */

static struct drm_driver driver;

/* only bind to the cirrus chip in qemu */
static DEFINE_PCI_DEVICE_TABLE(pciidlist) = {
	{ PCI_VENDOR_ID_CIRRUS, PCI_DEVICE_ID_CIRRUS_5446, 0x1af4, 0x1100, 0,
	  0, 0 },
	{0,}
};


static int cirrus_kick_out_firmware_fb(struct pci_dev *pdev)
{
	struct apertures_struct *ap;
	bool primary = false;

	ap = alloc_apertures(1);
	if (!ap)
		return -ENOMEM;

	ap->ranges[0].base = pci_resource_start(pdev, 0);
	ap->ranges[0].size = pci_resource_len(pdev, 0);

#ifdef CONFIG_X86
	primary = pdev->resource[PCI_ROM_RESOURCE].flags & IORESOURCE_ROM_SHADOW;
#endif
	remove_conflicting_framebuffers(ap, "cirrusdrmfb", primary);
	kfree(ap);

	return 0;
}

static int cirrus_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *ent)
{
	int ret;

	ret = cirrus_kick_out_firmware_fb(pdev);
	if (ret)
		return ret;

	return drm_get_pci_dev(pdev, ent, &driver);
}

static void cirrus_pci_remove(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);

	drm_put_dev(dev);
}

static const struct file_operations cirrus_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.read = drm_read,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = cirrus_mmap,
	.poll = drm_poll,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
};
static struct drm_driver driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_PRIME,
	.load = cirrus_driver_load,
	.unload = cirrus_driver_unload,
	.fops = &cirrus_driver_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
	.gem_free_object = cirrus_gem_free_object,
	.dumb_create = cirrus_dumb_create,
	.dumb_map_offset = cirrus_dumb_mmap_offset,
	.dumb_destroy = drm_gem_dumb_destroy,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export = drm_gem_prime_export,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_pin = cirrus_gem_prime_pin,
	.gem_prime_get_sg_table = cirrus_gem_prime_get_sg_table,
	.gem_prime_import_sg_table = cirrus_gem_prime_import_sg_table,
	.gem_prime_vmap = cirrus_gem_prime_vmap,
	.gem_prime_vunmap = cirrus_gem_prime_vunmap,
	.gem_prime_mmap = cirrus_gem_prime_mmap,
	.atomic_begin     = drm_atomic_begin,
	.atomic_set_event = drm_atomic_set_event,
	.atomic_check     = drm_atomic_check,
	.atomic_commit    = drm_atomic_commit,
	.atomic_end       = drm_atomic_end,
	.atomic_funcs     = &drm_atomic_funcs,
};

static struct pci_driver cirrus_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = pciidlist,
	.probe = cirrus_pci_probe,
	.remove = cirrus_pci_remove,
};

static int __init cirrus_init(void)
{
#ifdef CONFIG_VGA_CONSOLE
	if (vgacon_text_force() && cirrus_modeset == -1)
		return -EINVAL;
#endif

	if (cirrus_modeset == 0)
		return -EINVAL;
	return drm_pci_init(&driver, &cirrus_pci_driver);
}

static void __exit cirrus_exit(void)
{
	drm_pci_exit(&driver, &cirrus_pci_driver);
}

module_init(cirrus_init);
module_exit(cirrus_exit);

MODULE_DEVICE_TABLE(pci, pciidlist);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
