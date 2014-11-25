/*
 * Copyright (C) 2011-2014 Imagination Technologies Ltd.
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
 * AXD sysfs Support API
 */
#ifndef AXD_SYSFS_H_
#define AXD_SYSFS_H_

#include <linux/stat.h>

#define CMP_PARAM(str, param) \
	!strncmp((str), (param "="), sizeof(param "=")-1)

#define PARAM_VALUE(str, param) \
	PARAM_VALUE_WITH_END(str, param, NULL)

#define PARAM_VALUE_WITH_END(str, param, end) \
	simple_strtol((str)+sizeof(param "=")-1, end, 0)

#define PARAM_VALUE_ADV(str, param) \
	PARAM_VALUE_WITH_END(str, param, &str)

#define RD_PERMS	(S_IRUSR | S_IRGRP)
#define WR_PERMS	(S_IWUSR | S_IWGRP)

int axd_ctrl_sysfs_add(struct device *dev);
void axd_ctrl_sysfs_remove(struct device *dev);
int axd_input_sysfs_add(struct device *dev);
void axd_input_sysfs_remove(struct device *dev);
int axd_output_sysfs_add(struct device *dev);
void axd_output_sysfs_remove(struct device *dev);
#endif /* AXD_SYSFS_H_ */
