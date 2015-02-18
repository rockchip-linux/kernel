/* Marvell wireless LAN device driver: sysfs
 *
 * Copyright (C) 2015, Marvell International Ltd.
 *
 * This software file (the "File") is distributed by Marvell International
 * Ltd. under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available on the worldwide web at
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#include "main.h"

bool mfg_mode;
char mfg_firmware[32];

static ssize_t
mwifiex_sysfs_get_mfg_mode(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mwifiex_private *priv = to_net_dev(dev)->ml_priv;

	return snprintf(buf, PAGE_SIZE, "%d\n", priv->adapter->mfg_mode);
}

static ssize_t
mwifiex_sysfs_set_mfg_mode(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	bool res;

	if (strtobool(buf, &res))
		return -EINVAL;

	mfg_mode = res;

	return count;
}

static DEVICE_ATTR(mfg_mode, S_IRUGO | S_IWUSR,
		   mwifiex_sysfs_get_mfg_mode,
		   mwifiex_sysfs_set_mfg_mode);

static ssize_t mwifiex_sysfs_show_mfg_firmware(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", mfg_firmware);
}

static ssize_t mwifiex_sysfs_store_mfg_firmware(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int len;

	len = strlen(buf);
	if (len > sizeof(mfg_firmware))
		return -EINVAL;

	strcpy(mfg_firmware, buf);
	mfg_firmware[len - 1] = '\0';

	return count;
}
static DEVICE_ATTR(mfg_firmware, S_IRUGO | S_IWUSR,
		   mwifiex_sysfs_show_mfg_firmware,
		   mwifiex_sysfs_store_mfg_firmware);

int mwifiex_sysfs_register(struct mwifiex_private *priv)
{
	int ret;

	/* Create sysfs file to control manufacturing mode feature*/
	ret = device_create_file(&priv->netdev->dev, &dev_attr_mfg_mode);
	if (ret)
		dev_err(priv->adapter->dev,
			"failed to create sysfs file mfg_mode\n");

	/* Create sysfs file for manufacturing firmware name */
	ret = device_create_file(&priv->netdev->dev, &dev_attr_mfg_firmware);
	if (ret)
		dev_err(priv->adapter->dev,
			"failed to create sysfs file mfg_firmware\n");

	return ret;
}

void mwifiex_sysfs_unregister(struct mwifiex_private *priv)
{
	device_remove_file(&priv->netdev->dev, &dev_attr_mfg_mode);
	device_remove_file(&priv->netdev->dev, &dev_attr_mfg_firmware);
}

