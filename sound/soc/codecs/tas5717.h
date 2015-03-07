/*
 * TAS5717/TAS5719 amplifier audio driver
 *
 * Copyright (C) 2015 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _TAS5717_H
#define _TAS5717_H

#include <sound/pcm.h>

/* device registers */
#define TAS5717_SDI_REG			0x04
#define TAS5717_SDI_FMT_MASK		0x0f

#define TAS5717_SYS_CTRL_2_REG		0x05
#define TAS5717_SYS_CTRL_2_SDN_MASK	0x40

#define TAS5717_SOFT_MUTE_REG		0x06
#define TAS5717_SOFT_MUTE_CH_MASK	0x07

#define TAS5717_MVOL_REG		0x07
#define TAS5717_CH1_VOL_REG		0x08
#define TAS5717_CH2_VOL_REG		0x09

#define TAS5717_OSC_TRIM_REG		0x1b

#endif /* _TAS5717_H */
