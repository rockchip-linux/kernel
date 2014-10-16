/*
 *  ffu.h
 *
 * Copyright (C) 2015 Google, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This program was created by SanDisk Corp
 */

#ifndef _FFU_H_
#define _FFU_H_

#include <linux/mmc/card.h>

/*
 * eMMC5.0 Field Firmware Update (FFU) opcodes
*/
#define MMC_FFU_INVOKE_OP 302

#define MMC_FFU_MODE_SET 0x1
#define MMC_FFU_MODE_NORMAL 0x0
#define MMC_FFU_INSTALL_SET 0x2

#ifdef CONFIG_MMC_FFU
#define MMC_FFU_FEATURES 0x1
#define FFU_FEATURES(ffu_features) (ffu_features & MMC_FFU_FEATURES)

int mmc_ffu_invoke(struct mmc_card *card, const char *name);

#else
static inline int mmc_ffu_invoke(struct mmc_card *card, const char *name)
{
	return -ENOSYS;
}
#endif
#endif /* FFU_H_ */


