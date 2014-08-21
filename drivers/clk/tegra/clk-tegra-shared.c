/*
 * Copyright (c) 2012, 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>

#include "clk.h"
#include "clk-id.h"

struct tegra_shared_clk {
	char *name;
	char *client;
	union {
		const char **parents;
		const char *parent;
	} p;
	int num_parents;
	enum shared_bus_users_mode mode;
	int flags;
	int clk_id;
};

#define SHARED_CLK(_name, _parent, _mode, _flags, _client, _id)\
	{\
		.name = _name,\
		.p.parent = _parent,\
		.num_parents = 1,\
		.mode = _mode, \
		.flags = _flags, \
		.client = _client,\
		.clk_id = _id,\
	}

static struct tegra_shared_clk shared_clks[] = {
	SHARED_CLK("cap.c2bus", "c2bus", SHARED_CEILING, 0, NULL, tegra_clk_cap_c2bus),
	SHARED_CLK("cap.throttle.c2bus", "c2bus", SHARED_CEILING, 0, NULL, tegra_clk_cap_throttle_c2bus),
	SHARED_CLK("floor.c2bus", "c2bus", 0, 0, NULL, tegra_clk_floor_c2bus),
	SHARED_CLK("override.c2bus", "c2bus", SHARED_OVERRIDE, 0, NULL, tegra_clk_override_c2bus),
	SHARED_CLK("edp.c2bus", "c2bus", SHARED_CEILING, 0, NULL, tegra_clk_edp_c2bus),
	SHARED_CLK("battery.c2bus", "c2bus", SHARED_CEILING, 0, NULL, tegra_clk_battery_c2bus),
	SHARED_CLK("cap.profile.c2bus", "c2bus", SHARED_CEILING, 0, NULL, tegra_clk_cap_profile_c2bus),
	SHARED_CLK("cap.c3bus", "c3bus", SHARED_CEILING, 0, NULL, tegra_clk_cap_c3bus),
	SHARED_CLK("cap.throttle.c3bus", "c3bus", SHARED_CEILING, 0, NULL, tegra_clk_cap_throttle_c3bus),
	SHARED_CLK("override.c3bus", "c3bus", SHARED_OVERRIDE, 0, NULL, tegra_clk_override_c3bus),
	SHARED_CLK("cap.sclk", "sbus", SHARED_CEILING, 0, NULL, tegra_clk_cap_sclk),
	SHARED_CLK("cap.throttle.sclk", "sbus", SHARED_CEILING, 0, NULL, tegra_clk_cap_throttle_sclk),
	SHARED_CLK("floor.sclk", "sbus", 0, 0, NULL, tegra_clk_floor_sclk),
	SHARED_CLK("override.sclk", "sbus", SHARED_OVERRIDE, 0, NULL, tegra_clk_override_sclk),
	SHARED_CLK("avp.sclk", "sbus", 0, 0, NULL, tegra_clk_avp_sclk),
	SHARED_CLK("bsea.sclk", "sbus", 0, 0, NULL, tegra_clk_bsea_sclk),
	SHARED_CLK("usbd.sclk", "sbus", 0, 0, NULL, tegra_clk_usbd_sclk),
	SHARED_CLK("usb1.sclk", "sbus", 0, 0, NULL, tegra_clk_usb1_sclk),
	SHARED_CLK("usb2.sclk", "sbus", 0, 0, NULL, tegra_clk_usb2_sclk),
	SHARED_CLK("usb3.sclk", "sbus", 0, 0, NULL, tegra_clk_usb3_sclk),
	SHARED_CLK("wake.sclk", "sbus", 0, 0, NULL, tegra_clk_wake_sclk),
	SHARED_CLK("sbc1.sclk", "sbus", 0, 0, NULL, tegra_clk_sbc1_sclk),
	SHARED_CLK("sbc2.sclk", "sbus", 0, 0, NULL, tegra_clk_sbc2_sclk),
	SHARED_CLK("sbc3.sclk", "sbus", 0, 0, NULL, tegra_clk_sbc3_sclk),
	SHARED_CLK("sbc4.sclk", "sbus", 0, 0, NULL, tegra_clk_sbc4_sclk),
	SHARED_CLK("sbc5.sclk", "sbus", 0, 0, NULL, tegra_clk_sbc5_sclk),
	SHARED_CLK("sbc6.sclk", "sbus", 0, 0, NULL, tegra_clk_sbc6_sclk),
	SHARED_CLK("mon.avp", "sbus", 0, 0, NULL, tegra_clk_mon_avp),
	SHARED_CLK("avp.emc", "emc_master", 0, 0, NULL, tegra_clk_avp_emc),
	SHARED_CLK("cpu.emc", "emc_master", 0, 0, NULL, tegra_clk_cpu_emc),
	SHARED_CLK("disp1.emc", "emc_master", SHARED_BW, 0, NULL, tegra_clk_disp1_emc),
	SHARED_CLK("disp2.emc", "emc_master", SHARED_BW, 0, NULL, tegra_clk_disp2_emc),
	SHARED_CLK("hdmi.emc", "emc_master", 0, 0, NULL, tegra_clk_hdmi_emc),
	SHARED_CLK("usbd.emc", "emc_master", 0, 0, NULL, tegra_clk_usbd_emc),
	SHARED_CLK("usb1.emc", "emc_master", 0, 0, NULL, tegra_clk_usb1_emc),
	SHARED_CLK("usb2.emc", "emc_master", 0, 0, NULL, tegra_clk_usb2_emc),
	SHARED_CLK("usb3.emc", "emc_master", 0, 0, NULL, tegra_clk_usb3_emc),
	SHARED_CLK("mon.emc", "emc_master", 0, 0, NULL, tegra_clk_mon_emc),
	SHARED_CLK("msenc.emc", "emc_master", SHARED_BW, 0, NULL, tegra_clk_msenc_emc),
	SHARED_CLK("tsec.emc", "emc_master", 0, 0, NULL, tegra_clk_tsec_emc),
	SHARED_CLK("sdmmc4.emc", "emc_master", 0, 0, NULL, tegra_clk_sdmmc4_emc),
	SHARED_CLK("camera.emc", "emc_master", SHARED_BW, 0, NULL, tegra_clk_camera_emc),
	SHARED_CLK("iso.emc", "emc_master", SHARED_BW, 0, NULL, tegra_clk_iso_emc),
	SHARED_CLK("cap.emc", "emc_master", SHARED_CEILING, 0, NULL, tegra_clk_cap_emc),
	SHARED_CLK("cap.throttle.emc", "emc_master", SHARED_CEILING, 0, NULL, tegra_clk_cap_throttle_emc),
	SHARED_CLK("floor.emc", "emc_master", 0, 0, NULL, tegra_clk_floor_emc),
	SHARED_CLK("override.emc", "emc_master", SHARED_OVERRIDE, 0, NULL, tegra_clk_override_emc),
	SHARED_CLK("edp.emc", "emc_master", SHARED_CEILING, 0, NULL, tegra_clk_edp_emc),
	SHARED_CLK("battery.emc", "emc_master", SHARED_CEILING, 0, NULL, tegra_clk_battery_emc),
	SHARED_CLK("gk20a.emc", "emc_master", 0, 0, NULL, tegra_clk_gk20a_emc),
	SHARED_CLK("vic03.emc", "emc_master", 0, 0, NULL, tegra_clk_vic03_emc),
	SHARED_CLK("ispa.emc", "emc_master", 0, 0, NULL, tegra_clk_ispa_emc),
	SHARED_CLK("ispb.emc", "emc_master", 0, 0, NULL, tegra_clk_ispb_emc),
	SHARED_CLK("xusb.emc", "emc_master", 0, 0, NULL, tegra_clk_xusb_emc),
};

void __init tegra_shared_clk_init(struct tegra_clk *tegra_clks)
{
	int i;
	const char **parents;
	struct tegra_shared_clk *data;
	struct clk *clk;
	struct clk **dt_clk;

	for (i = 0; i < ARRAY_SIZE(shared_clks); i++) {
		data = &shared_clks[i];
		if (data->num_parents == 1)
			parents = &data->p.parent;
		else
			parents = data->p.parents;

		dt_clk = tegra_lookup_dt_id(data->clk_id, tegra_clks);
		if (!dt_clk)
			continue;

		clk = tegra_clk_register_shared(data->name, parents,
				data->num_parents, data->flags, data->mode,
				data->client);
		*dt_clk = clk;
	}
}
