
/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
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

#ifndef __QCOM_CLK_RPM_H
#define __QCOM_CLK_RPM_H

#include <linux/clk-provider.h>
#include <linux/mfd/qcom_rpm.h>

extern const struct clk_ops clk_ops_rpm;
extern const struct clk_ops clk_ops_rpm_branch;

struct rpm_clk {
	struct qcom_rpm *rpm;
	struct device *dev;
	struct clk_hw hw;
	int rpm_clk_id;
	u32 rate;
	bool active_only;
	bool enabled;
	bool branch; /* true: RPM only accepts 1 for ON and 0 for OFF */
};

#define to_rpm_clk(_hw) container_of(_hw, struct rpm_clk, hw)

#endif
