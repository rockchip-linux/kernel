
/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_SOC_QFPROM_H__
#define __LINUX_SOC_QFPROM_H__

struct device;

#ifdef CONFIG_QCOM_QFPROM
extern char *devm_qfprom_get_data_byname(struct device *dev,
				const char *name, int *len);
extern char *devm_qfprom_get_data(struct device *dev, int index, int *len);
extern char *qfprom_get_data_byname(struct device *dev,
	const char *name, int *len);
#else
static inline char *devm_qfprom_get_data_byname(struct device *dev,
				const char *name, int *len)
{
	return NULL;
}

static inline char *devm_qfprom_get_data(struct device *dev,
				int index, int *len)
{
	return NULL;
}

static inline char *qfprom_get_data_byname(struct device *dev,
				const char *name, int *len)
{
	return NULL;
}

static inline char *qfprom_get_data(struct device *dev,
				int index, int *len)
{
	return NULL;
}
#endif

#endif /* __LINUX_SOC_QFPROM_H__ */
