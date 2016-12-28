/*
**************************************************************************
 * Rockchip driver for CVBSIN 1.0
 *
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
**************************************************************************
 */
#ifndef _CIF_CVBSIN_H
#define _CIF_CVBSIN_H
#include <linux/videodev2.h>

enum pltfrm_cvbsin_cfg_cmd {
	PLTFRM_CVBSIN_POWERON = 0,
	PLTFRM_CVBSIN_POWEROFF,

	PLTFRM_CVBSIN_RST,
	PLTFRM_CVBSIN_INIT
};

struct pltfrm_cvbsin_cfg_para {
	enum pltfrm_cvbsin_cfg_cmd cmd;
	void *cfg_para;
};

struct pltfrm_cvbsin_init_para {
	struct platform_device *pdev;
};

struct pltfrm_cvbsin_cfg {
	char name[32];

	int (*soc_cfg)(struct pltfrm_cvbsin_cfg_para *cfg);
};

int pltfrm_rv1108_cvbsin_cfg(
	struct pltfrm_cvbsin_cfg_para *cfg);

#endif
