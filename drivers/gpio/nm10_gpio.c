/*
 * Copyright (c) 2010, The Chromium OS Authors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * This driver supports GPIO controller of the NM10 chip.
 *
 * The NM10 has many GPIO pins, the exact number depends on the configuration,
 * as some of the pins can be used for other than GPIO purposes. The GPIO
 * controller has provision of managing of up to 64 bits. Each bit can be
 * configured as 'not available', (when for other than GPIO purposes), or a
 * GPIO input/output.
 *
 * Even though NM10 provides the ability to change GPIO bits' directions, this
 * driver does NOT allow to change the use of the bits. Whatever the system is
 * strapped and/or configured by BIOS for is used by the driver.
 *
 * This driver plugs in into the existing linux gpio infrastructure and allows
 * to instantiate all 64 bits (through writing into /sys/class/gpio/export)
 * even though not all bits can be used. This simplifies bit mapping between
 * hardware and software.
 *
 * Attempts to write into unsupported bits are silently ignored. Attempts to
 * read unsupported bits return value of zero.
 *
 * For the lower 32 GPIO bits the NM10 provides the ability to 'blink'
 * (alternate 1 and zero with 1Hz frequency at approximately 50% duty cycle)
 * on output and invert level on input. This driver does not provide access
 * these features.
 *
 * The NM10 GPIO controller is a part of the LPC PCI device, (PCI device ID
 * 0x27bc). The GPIO register file is mapped to the IO space. An earlier Intel
 * chip, the ICH7M south bridge, has a similar GPIO controller, it could be
 * also supported by this driver.
 *
 * "Intel NM10 Family Express Chipset" datasheet of Dec 2009 (document number
 * 322896-001) was used as a reference when writing this driver.
 *
 * This driver also supports the Intel 6 Series chipset. This chipset
 * supports 96 GPIOs instead of 64 that NM10 does. Datasheet here:
 * http://www.intel.com/content/www/us/en/chipsets/6-chipset-c200-chipset-datasheet.html
 *
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/gpio.h>
#include <linux/slab.h>

static char gpio_driver_name[] = "nm10_gpio";
static char gpio_driver_version[] = "0.04";

/* NM10 GPIO register file definitions, offsets in the IO space */
#define NM10_GPIO_USE_SEL 0
#define NM10_GPIO_IO_SEL 4
#define NM10_GPIO_LVL 0xc
#define NM10_GPIO_USE_SEL2 0x30
#define NM10_GPIO_IO_SEL2 0x34
#define NM10_GPIO_LVL2 0x38
#define NM10_GPIO_USE_SEL3 0x40
#define NM10_GPIO_IO_SEL3 0x44
#define NM10_GPIO_LVL3 0x48

#define NM10_GPIO_REG_FILE_SIZE 0x40

/* Structure describing one GPIO section in the nm10, accessing 32 GPIO bits. */
struct nm10_gpio_info {
	u_char use_select_offset;
	u_char io_select_offset;
	u_char io_level_offset;
};

/* This array describes two NM10 GPIO sections */
const struct nm10_gpio_info nm10_gpio_sections[] = {
	{NM10_GPIO_USE_SEL, NM10_GPIO_IO_SEL, NM10_GPIO_LVL},
	{NM10_GPIO_USE_SEL2, NM10_GPIO_IO_SEL2, NM10_GPIO_LVL2},
	{NM10_GPIO_USE_SEL3, NM10_GPIO_IO_SEL3, NM10_GPIO_LVL3},
};

#define NM10_GPIO_BITS_PER_SECTION 32
#define NM10_GPIO_SECTIONS ARRAY_SIZE(nm10_gpio_sections)

static u32 max_gpio_bits;
#define NM10_MAX_GPIO_BITS max_gpio_bits

/*
 * Structure representing a single NM10 GPIO driver instance.
 */
struct nm10_gpio {
	struct gpio_chip chip;
	u32 io_base;	/* base IO space address of the GPIO register file */

	/* cached contents of the GPIO bit selections, read during driver
	 * installation */
	u32 cached_select[NM10_GPIO_SECTIONS];
};

/**
 * nm10_get_parameters() - get value of a GPIO bit
 *
 * Inputs:
 * @chip: generic gpio chip handle associated with this module
 * @offset: zero based GPIO bit number (in this controller's scope).
 *
 * Outputs:
 * @psection - pointer to the NM10 section number containing bit offset
 * @pbit,- pointer to the offset's bit mask within the section
 * @pgpio - pointer to address of this nm10_gpio instance.
 *
 * Returns zero on errors or nonzero on success.
 */
static int nm10_get_parameters(struct gpio_chip *chip, unsigned offset,
			       u8* psection, u32* pbit,
			       struct nm10_gpio **pgpio)
{
	*pgpio = container_of(chip, struct nm10_gpio, chip);

	*psection = offset / NM10_GPIO_BITS_PER_SECTION;
	*pbit = BIT(offset % NM10_GPIO_BITS_PER_SECTION);

	if (*psection >= NM10_GPIO_SECTIONS) {
		printk(KERN_ERR "%s: bad offset %d\n",
		       gpio_driver_name, offset);
		return 0;
	}

	if (!(*pbit & (*pgpio)->cached_select[*psection])) {
		return 0;	/* this bit is not used for GPIO */
	}
	return ~0;
}

/**
 * nm10_gpio_get() - get value of a GPIO bit
 * @chip: generic gpio chip handle associated with this module
 * @offset: zero based GPIO bit number (in this controller's scope).
 *
 * Returns zero in cases when offset exceeds the chip's GPIO capacity, or the
 * passed in bit not used for GPIO. If the offset is of a valid bit - returns
 * a bitmask with the bit value matching the actual bit input state.
 */
static int nm10_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	u8 section;
	u32 bit;
	struct nm10_gpio *pgpio = container_of(chip, struct nm10_gpio, chip);

	if (!nm10_get_parameters(chip, offset, &section, &bit, &pgpio)) {
		return 0;
	}

	return inl(pgpio->io_base +
		   nm10_gpio_sections[section].io_level_offset) & bit;
}

/**
 * nm10_gpio_set() - set value of a GPIO bit
 * @chip: generic gpio chip handle associated with this module
 * @offset: zero based GPIO bit number (in this controller's scope).
 * @value: the value to set the output to
 *
 * If the offset is of a valid bit (used for GPIO output) - the bit state is
 * changed to reflect the value. All other in range offset values are ignored.
 */
static void nm10_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	u8 section;
	u32 bit;
	const struct nm10_gpio_info *pinfo;
	struct nm10_gpio *pgpio;
	u32 gpio_reg_value;

	if (!nm10_get_parameters(chip, offset, &section, &bit, &pgpio)) {
		return;
	}

	pinfo = nm10_gpio_sections + section;
	if (inl(pgpio->io_base + pinfo->io_select_offset) & bit) {
		return;		/* this is an input bit */
	}

	gpio_reg_value = inl(pgpio->io_base + pinfo->io_level_offset);

	if (value) {
		gpio_reg_value |= bit;
	} else {
		gpio_reg_value &= ~bit;
	}
	outl(gpio_reg_value, pgpio->io_base + pinfo->io_level_offset);
}

/**
 * nm10_gpio_direction_inp() configure signal "offset" as input, or return error
 * @chip: generic gpio chip handle associated with this module
 * @offset: zero based GPIO bit number (in this controller's scope).
 */
static int nm10_gpio_direction_inp(struct gpio_chip *chip,
				   unsigned offset)
{
	u8 section;
	u32 bit;
	struct nm10_gpio *pgpio = container_of(chip, struct nm10_gpio, chip);
	u32 io_select_offset;

	if (!nm10_get_parameters(chip, offset, &section, &bit, &pgpio)) {
		return -1;
	}

	io_select_offset = pgpio->io_base +
		nm10_gpio_sections[section].io_select_offset;
	outl(inl(io_select_offset) | bit, io_select_offset);
	return 0;
}

/**
 * nm10_gpio_direction_out() configure signal "offset" as output,
 * 			     or return error
 * @chip: generic gpio chip handle associated with this module
 * @offset: zero based GPIO bit number (in this controller's scope).
 * @value: the value to set the output to
 */
static int nm10_gpio_direction_out(struct gpio_chip *chip,
				   unsigned offset, int value)
{
	u8 section;
	u32 bit;
	struct nm10_gpio *pgpio = container_of(chip, struct nm10_gpio, chip);
	u32 io_select_offset;

	if (!nm10_get_parameters(chip, offset, &section, &bit, &pgpio)) {
		return -1;
	}

	io_select_offset = pgpio->io_base +
		nm10_gpio_sections[section].io_select_offset;
	outl(inl(io_select_offset) & ~bit, io_select_offset);
	nm10_gpio_set(chip, offset, value);
	return 0;
}

static int nm10_gpio_probe(struct pci_dev *pdev,
			   const struct pci_device_id *id)
{
	int retval, ii;
	u32 value;
	struct nm10_gpio *pgpio;

	retval = pci_enable_device(pdev);
	printk(KERN_INFO "%s version %s.\n", gpio_driver_name,
	       gpio_driver_version);

	if (retval) {
		goto done;
	}

	/* actual IO space offset of the GPIO block */
	retval = pci_read_config_dword(pdev, 0x48, &value);
	if (retval || !(value & 1)) {
		dev_err(&pdev->dev,
			"failed retrieving IO base addr, got %d(0x%x)\n",
			retval, value);
		goto err2;
	}

	value &= ~1; /* clear the IO space flag */
	if (!request_region(value, NM10_GPIO_REG_FILE_SIZE, gpio_driver_name)) {
		dev_err(&pdev->dev, "error requesting REGION\n");
		retval = -ENOMEM;
		goto err2;
	}

	pgpio = kzalloc(sizeof(struct nm10_gpio), GFP_KERNEL);
	if (!pgpio) {
		dev_err(&pdev->dev, "can't allocate nm10_gpio structure\n");
		retval = -ENOMEM;
		goto err3;
	}

	if (id->device == PCI_DEVICE_ID_INTEL_TGP_LPC)
		/* NM10 supports 64 GPIOs */
		max_gpio_bits = 64;
	else
		/* Cougarpoint supports 96 GPIOs */
		max_gpio_bits = 96;

	/* used to access GPIO bits on this chip */
	pgpio->io_base = value;

	pgpio->chip.base = -1;
	pgpio->chip.label = dev_name(&pdev->dev);
	pgpio->chip.get = nm10_gpio_get;
	pgpio->chip.set = nm10_gpio_set;
	pgpio->chip.direction_input = nm10_gpio_direction_inp;
	pgpio->chip.direction_output = nm10_gpio_direction_out;
	pgpio->chip.ngpio = NM10_MAX_GPIO_BITS;
	pgpio->chip.can_sleep = 0;
	pci_set_drvdata(pdev, pgpio);

	/* store GPIO configuration locally */
	for (ii = 0; ii < ARRAY_SIZE(pgpio->cached_select); ii++) {
		pgpio->cached_select[ii] = inl(pgpio->io_base +
					       nm10_gpio_sections[ii].
					       use_select_offset);
	}

	retval = gpiochip_add(&pgpio->chip);
	if (!retval) {
		goto done;
	}

	dev_err(&pdev->dev, "%s gpiochip_add error %d\n",
		gpio_driver_name, retval);

	kfree(pgpio);
      err3:
	release_region(value, NM10_GPIO_REG_FILE_SIZE);
      err2:
	pci_disable_device(pdev);
      done:
	if (retval) {
		printk(KERN_ERR "%s failed!\n", gpio_driver_name);

	}
	return retval;
}

/**
 * nm10_gpio_remove() - remove the NM10 gpio driver module.
 * @pdev: pci device associated with this driver module
 *
 */
static void nm10_gpio_remove(struct pci_dev *pdev)
{
	struct nm10_gpio *pgpio = pci_get_drvdata(pdev);
	int base = pgpio->chip.base;

	release_region(pgpio->io_base, NM10_GPIO_REG_FILE_SIZE);

	if (gpiochip_remove(&pgpio->chip)) {
		printk(KERN_ERR "%s: failed removing!\n", gpio_driver_name);
		return;
	}

	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	kfree(pgpio);
	printk(KERN_INFO "%s base %d removed\n", gpio_driver_name, base);
}

static struct pci_device_id nm10_gpio_ids[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_TGP_LPC)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_Z68)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_P67)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_UM67)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_HM65)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_H67)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_HM67)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_Q65)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_QS67)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_Q67)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_QM67)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_B65)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_C202)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_C204)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_C206)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_COUGARPOINT_LPC_H61)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_B75)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_C216)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_H77)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_HM70)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_HM75)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_HM76)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_HM77)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_MBL_SAMPLE)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_NM70)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_Q75)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_Q77)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_QM77)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_QS77)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_SFF_SAMPLE)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_UM77)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_Z75)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL,
		PCI_DEVICE_ID_INTEL_PANTHERPOINT_LPC_Z77)},
	{0,}
};

MODULE_DEVICE_TABLE(pci, nm10_gpio_ids);

static struct pci_driver nm10_gpio_pci_driver = {
	.name = gpio_driver_name,
	.id_table = nm10_gpio_ids,
	.probe = nm10_gpio_probe,
	.remove = nm10_gpio_remove
};

static int __init nm10_gpio_init(void)
{
	return pci_register_driver(&nm10_gpio_pci_driver);
}

static void __exit nm10_gpio_exit(void)
{
	pci_unregister_driver(&nm10_gpio_pci_driver);
}

module_init(nm10_gpio_init);
module_exit(nm10_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("The Chromium OS Authors");
MODULE_DESCRIPTION("NM10 GPIO driver");
