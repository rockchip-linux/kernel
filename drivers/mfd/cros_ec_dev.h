/*
 * Copyright (C) 2014 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _DRV_CROS_EC_DEV_H_
#define _DRV_CROS_EC_DEV_H_

struct cros_ec_device;


/* struct cros_ec_platform - ChromeOS EC platform information
 *
 * @ec_name: name of EC device (e.g. 'cros-ec', 'cros-pd', ...)
 * used in /dev/ and sysfs.
 * @cmd_offset: offset to apply for each command. Set when
 * registering a devicde behind another one.
 */
struct cros_ec_platform {
	const char *ec_name;
	u16 cmd_offset;
};

/*
 * struct cros_ec_dev - ChromeOS EC device entry point
 *
 * @class_dev: Device structure used in sysfs
 * @cdev: Character device structure in /dev
 * @ec_dev: cros_ec_device structure to talk to the physical device
 * @dev: pointer to the platform device
 * @cmd_offset: offset to apply for each command.
 */
struct cros_ec_dev {
	struct device class_dev;
	struct cdev cdev;
	struct cros_ec_device *ec_dev;
	struct device *dev;
	u16 cmd_offset;
};

/* sysfs stuff */
extern struct attribute_group cros_ec_attr_group;
extern struct attribute_group cros_ec_lightbar_attr_group;

#endif	/* _DRV_CROS_EC_DEV_H_ */
