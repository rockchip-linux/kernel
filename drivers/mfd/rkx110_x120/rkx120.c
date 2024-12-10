// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Rockchip Electronics Co. Ltd.
 *
 * Author: Andy Yan <Andy.yan@rock-chips.com>
 */

#include <dt-bindings/mfd/rockchip-serdes.h>
#include <linux/debugfs.h>

#include "hal/cru_api.h"
#include "hal/pinctrl_api.h"
#include "rkx110_x120.h"
#include "rkx120_reg.h"
#include "serdes_combphy.h"

#if defined(CONFIG_DEBUG_FS)
static struct pattern_gen rkx120_pattern_gen[] = {
	{
		.name = "dsi",
		.base = RKX120_PATTERN_GEN_DSI_BASE,
		.link_src_reg = DES_GRF_SOC_CON2,
		.link_src_offset = 12,
		.type = RK_SERDES_DSI_TX0,
	}, {
		.name = "dual-lvds",
		.link_src_reg = DES_GRF_SOC_CON1,
		.link_src_offset = 14,
		.type = RK_SERDES_DUAL_LVDS_TX,
	}, {
		.name = "lvds0",
		.base = RKX120_PATTERN_GEN_LVDS0_BASE,
		.link_src_reg = DES_GRF_SOC_CON2,
		.link_src_offset = 13,
		.type = RK_SERDES_LVDS_TX0,
	}, {
		.name = "lvds1",
		.base = RKX120_PATTERN_GEN_LVDS1_BASE,
		.link_src_reg = DES_GRF_SOC_CON2,
		.link_src_offset = 14,
		.type = RK_SERDES_LVDS_TX1,
	},
	{ /* sentinel */ }
};

static const struct rk_serdes_reg rkx120_regs[] = {
	{
		.name = "grf",
		.reg_base = RKX120_DES_GRF_BASE,
		.reg_len = 0x410,
	},
	{
		.name = "cru",
		.reg_base = RKX120_DES_CRU_BASE,
		.reg_len = 0xF04,
	},
	{
		.name = "dvp",
		.reg_base = RKX120_DVP_TX_BASE,
		.reg_len = 0x1d0,
	},

	{
		.name = "grf_mipi0",
		.reg_base = RKX120_GRF_MIPI0_BASE,
		.reg_len = 0x600,
	},
	{
		.name = "grf_mipi1",
		.reg_base = RKX120_GRF_MIPI1_BASE,
		.reg_len = 0x600,
	},
	{
		.name = "mipi_lvds_phy0",
		.reg_base = RKX120_MIPI_LVDS_TX_PHY0_BASE,
		.reg_len = 0x70,
	},
	{
		.name = "mipi_lvds_phy1",
		.reg_base = RKX120_MIPI_LVDS_TX_PHY1_BASE,
		.reg_len = 0x70,
	},
	{
		.name = "dsihost",
		.reg_base = RKX120_DSI_TX_BASE,
		.reg_len = 0x190,
	},
	{
		.name = "gpio0",
		.reg_base = RKX120_GPIO0_TX_BASE,
		.reg_len = 0x80,
	},
	{
		.name = "gpio1",
		.reg_base = RKX120_GPIO1_TX_BASE,
		.reg_len = 0x80,
	},
	{
		.name = "csi_tx0",
		.reg_base = RKX120_CSI_TX0_BASE,
		.reg_len = 0x1D0,
	},
	{
		.name = "csi_tx1",
		.reg_base = RKX120_CSI_TX1_BASE,
		.reg_len = 0x1D0,
	},
	{
		.name = "rklink",
		.reg_base = RKX120_DES_RKLINK_BASE,
		.reg_len = 0xD4,
	},
	{
		.name = "pcs0",
		.reg_base = RKX120_DES_PCS0_BASE,
		.reg_len = 0x1c0,
	},
	{
		.name = "pcs1",
		.reg_base = RKX120_DES_PCS1_BASE,
		.reg_len = 0x1c0,
	},
	{
		.name = "pwm",
		.reg_base = RKX120_PWM_BASE,
		.reg_len = 0x100,
	},
	{
		.name = "pma0",
		.reg_base = RKX120_DES_PMA0_BASE,
		.reg_len = 0x100,
	},
	{
		.name = "pma1",
		.reg_base = RKX120_DES_PMA1_BASE,
		.reg_len = 0x100,
	},
	{
		.name = "dsi_pattern_gen",
		.reg_base = RKX120_PATTERN_GEN_DSI_BASE,
		.reg_len = 0x18,
	},
	{
		.name = "lvds0_pattern_gen",
		.reg_base = RKX120_PATTERN_GEN_LVDS0_BASE,
		.reg_len = 0x18,
	},
	{
		.name = "lvds1_pattern_gen",
		.reg_base = RKX120_PATTERN_GEN_LVDS1_BASE,
		.reg_len = 0x18,
	},
	{ /* sentinel */ }
};

static int rkx120_reg_show(struct seq_file *s, void *v)
{
	const struct rk_serdes_reg *regs = rkx120_regs;
	struct rk_serdes_chip *chip = s->private;
	struct rk_serdes *serdes = chip->serdes;
	struct i2c_client *client = chip->client;
	int i;
	u32 val = 0;

	seq_printf(s, "rkx120_%s:\n", file_dentry(s->file)->d_iname);

	while (regs->name) {
		if (!strcmp(regs->name, file_dentry(s->file)->d_iname))
			break;
		regs++;
	}

	if (!regs->name)
		return -ENODEV;

	for (i = 0; i <= regs->reg_len; i += 4) {
		serdes->i2c_read_reg(client, regs->reg_base + i, &val);

		if (i % 16 == 0)
			seq_printf(s, "\n0x%04x:", i);
		seq_printf(s, " %08x", val);
	}
	seq_puts(s, "\n");

	return 0;
}

static ssize_t rkx120_reg_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	const struct rk_serdes_reg *regs = rkx120_regs;
	struct rk_serdes_chip *chip = file->f_path.dentry->d_inode->i_private;
	struct rk_serdes *serdes = chip->serdes;
	struct i2c_client *client = chip->client;
	u32 addr;
	u32 val;
	char kbuf[25];
	int ret;

	if (count >= sizeof(kbuf))
		return -ENOSPC;

	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;

	kbuf[count] = '\0';

	ret = sscanf(kbuf, "%x%x", &addr, &val);
	if (ret != 2)
		return -EINVAL;

	while (regs->name) {
		if (!strcmp(regs->name, file_dentry(file)->d_iname))
			break;
		regs++;
	}

	if (!regs->name)
		return -ENODEV;

	addr += regs->reg_base;

	serdes->i2c_write_reg(client, addr, val);

	return count;
}

static int rkx120_reg_open(struct inode *inode, struct file *file)
{
	struct rk_serdes_chip *chip = inode->i_private;

	return single_open(file, rkx120_reg_show, chip);
}

static const struct file_operations rkx120_reg_fops = {
	.owner          = THIS_MODULE,
	.open           = rkx120_reg_open,
	.read           = seq_read,
	.write          = rkx120_reg_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

void rkx120_debugfs_init(struct rk_serdes_chip *chip, struct dentry *dentry)
{
	const struct rk_serdes_reg *regs = rkx120_regs;
	struct pattern_gen *pattern_gen;
	struct dentry *dir;

	pattern_gen = devm_kmemdup(chip->serdes->dev, &rkx120_pattern_gen,
				   sizeof(rkx120_pattern_gen), GFP_KERNEL);
	if (!pattern_gen)
		return;

	dir = debugfs_create_dir("registers", dentry);
	if (!IS_ERR(dir)) {
		while (regs->name) {
			debugfs_create_file(regs->name, 0600, dir, chip, &rkx120_reg_fops);
			regs++;
		}
	}

	dir = debugfs_create_dir("pattern_gen", dentry);
	if (!IS_ERR(dir)) {
		while (pattern_gen->name) {
			rkx110_x120_pattern_gen_debugfs_create_file(pattern_gen, chip, dir);
			pattern_gen++;
		}
	}
}
#endif

static int rkx120_rgb_tx_iomux_cfg(struct rk_serdes *serdes, struct rk_serdes_route *route,
				   u8 remote_id)
{
	struct i2c_client *client = serdes->chip[remote_id].client;
	uint32_t pins;

	pins = RK_SERDES_GPIO_PIN_C0 | RK_SERDES_GPIO_PIN_C1 | RK_SERDES_GPIO_PIN_C2 |
	       RK_SERDES_GPIO_PIN_C3 | RK_SERDES_GPIO_PIN_C4 | RK_SERDES_GPIO_PIN_C5 |
	       RK_SERDES_GPIO_PIN_C6 | RK_SERDES_GPIO_PIN_C7;
	serdes->set_hwpin(serdes, client, PIN_RKX120, RK_SERDES_DES_GPIO_BANK0, pins,
			  RK_SERDES_PIN_CONFIG_MUX_FUNC1);

	pins = RK_SERDES_GPIO_PIN_A0 | RK_SERDES_GPIO_PIN_A1 | RK_SERDES_GPIO_PIN_A2 |
	       RK_SERDES_GPIO_PIN_A3 | RK_SERDES_GPIO_PIN_A4 | RK_SERDES_GPIO_PIN_A5 |
	       RK_SERDES_GPIO_PIN_A6 | RK_SERDES_GPIO_PIN_A7 | RK_SERDES_GPIO_PIN_B0 |
	       RK_SERDES_GPIO_PIN_B1 | RK_SERDES_GPIO_PIN_B2 | RK_SERDES_GPIO_PIN_B3 |
	       RK_SERDES_GPIO_PIN_B4 | RK_SERDES_GPIO_PIN_B5 | RK_SERDES_GPIO_PIN_B6 |
	       RK_SERDES_GPIO_PIN_B7 | RK_SERDES_GPIO_PIN_C0 | RK_SERDES_GPIO_PIN_C1 |
	       RK_SERDES_GPIO_PIN_C2 | RK_SERDES_GPIO_PIN_C3;
	serdes->set_hwpin(serdes, client, PIN_RKX120, RK_SERDES_DES_GPIO_BANK1, pins,
			  RK_SERDES_PIN_CONFIG_MUX_FUNC1);

	return 0;
}

int rkx120_rgb_tx_enable(struct rk_serdes *serdes, struct rk_serdes_route *route,
			 u8 remote_id)
{
	rkx120_rgb_tx_iomux_cfg(serdes, route, remote_id);

	return 0;
}

int rkx120_lvds_tx_enable(struct rk_serdes *serdes, struct rk_serdes_route *route, u8 remote_id,
			  u8 phy_id)
{
	struct i2c_client *client = serdes->chip[remote_id].client;
	struct rk_serdes_panel *sd_panel = container_of(route, struct rk_serdes_panel, route);
	struct rkx120_combtxphy *combtxphy = &sd_panel->combtxphy;

	if (phy_id) {
		serdes->i2c_write_reg(client, DES_GRF_SOC_CON7, 0xfff00630);
		serdes->i2c_write_reg(client, DES_GRF_SOC_CON2, 0x00200020);
	} else {
		serdes->i2c_write_reg(client, DES_GRF_SOC_CON6, 0x0fff0063);
		serdes->i2c_write_reg(client, DES_GRF_SOC_CON2, 0x00040004);
	}

	rkx120_combtxphy_set_mode(combtxphy, COMBTX_PHY_MODE_VIDEO_LVDS);
	if (route->remote0_port0 & RK_SERDES_DUAL_LVDS_TX)
		rkx120_combtxphy_set_rate(combtxphy, route->vm.pixelclock * 7 / 2);
	else
		rkx120_combtxphy_set_rate(combtxphy, route->vm.pixelclock * 7);
	rkx120_combtxphy_power_on(serdes, combtxphy, remote_id, phy_id);

	return 0;
}
