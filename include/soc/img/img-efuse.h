/*
 * Imagination Technologies Generic eFuse driver
 *
 * Copyright (C) 2014 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __IMG_EFUSE_H
#define __IMG_EFUSE_H

int img_efuse_readl(unsigned int offset, u32 *value);

#endif
