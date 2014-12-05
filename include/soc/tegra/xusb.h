/*
 * Copyright (C) 2014 NVIDIA Corporation
 * Copyright (C) 2014 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#ifndef __SOC_TEGRA_XUSB_H__
#define __SOC_TEGRA_XUSB_H__

#define TEGRA_XUSB_USB3_PHYS 2
#define TEGRA_XUSB_UTMI_PHYS 3
#define TEGRA_XUSB_HSIC_PHYS 2
#define TEGRA_XUSB_NUM_USB_PHYS (TEGRA_XUSB_USB3_PHYS + TEGRA_XUSB_UTMI_PHYS + \
				 TEGRA_XUSB_HSIC_PHYS)
#define TEGRA_XUSB_NUM_PHYS (TEGRA_XUSB_NUM_USB_PHYS + 2) /* + SATA & PCIe */

/* Two virtual channels: host + phy */
#define TEGRA_XUSB_MBOX_NUM_CHANS 2

/* Command requests from the firmware */
enum tegra_xusb_mbox_cmd {
	MBOX_CMD_MSG_ENABLED = 1,
	MBOX_CMD_INC_FALC_CLOCK,
	MBOX_CMD_DEC_FALC_CLOCK,
	MBOX_CMD_INC_SSPI_CLOCK,
	MBOX_CMD_DEC_SSPI_CLOCK,
	MBOX_CMD_SET_BW, /* no ACK/NAK required */
	MBOX_CMD_SET_SS_PWR_GATING,
	MBOX_CMD_SET_SS_PWR_UNGATING,
	MBOX_CMD_SAVE_DFE_CTLE_CTX,
	MBOX_CMD_AIRPLANE_MODE_ENABLED, /* unused */
	MBOX_CMD_AIRPLANE_MODE_DISABLED, /* unused */
	MBOX_CMD_START_HSIC_IDLE,
	MBOX_CMD_STOP_HSIC_IDLE,
	MBOX_CMD_DBC_WAKE_STACK, /* unused */
	MBOX_CMD_HSIC_PRETEND_CONNECT,

	MBOX_CMD_MAX,

	/* Response message to above commands */
	MBOX_CMD_ACK = 128,
	MBOX_CMD_NAK
};

struct tegra_xusb_mbox_msg {
	u32 cmd;
	u32 data;
};

struct phy;

extern int tegra_xusb_phy_wake_enable(struct phy *phy);
extern int tegra_xusb_phy_wake_disable(struct phy *phy);
extern int tegra_xusb_usb3_phy_vcore_down(struct phy *phy);
extern int tegra_xusb_utmi_phy_get_tctrl_rctrl(struct phy *phy, u32 *tctrl,
					       u32 *rctrl);

#endif /* __SOC_TEGRA_XUSB_H__ */
