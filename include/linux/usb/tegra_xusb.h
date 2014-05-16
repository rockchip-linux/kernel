/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Ajay Gupta <ajayg@nvidia.com>
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
 */
#ifndef _TEGRA_XUSB_H
#define _TEGRA_XUSB_H

#include <uapi/linux/usb/ch9.h>

/* Command requests from the firmware */
enum tegra_xusb_mbox_cmd {
	MBOX_CMD_MSG_ENABLED = 1,
	MBOX_CMD_INC_FALC_CLOCK,
	MBOX_CMD_DEC_FALC_CLOCK,
	MBOX_CMD_INC_SSPI_CLOCK,
	MBOX_CMD_DEC_SSPI_CLOCK,
	MBOX_CMD_SET_BW,
	MBOX_CMD_SET_SS_PWR_GATING,
	MBOX_CMD_SET_SS_PWR_UNGATING,
	MBOX_CMD_SAVE_DFE_CTLE_CTX,
	MBOX_CMD_AIRPLANE_MODE_ENABLED, /* unused */
	MBOX_CMD_AIRPLANE_MODE_DISABLED, /* unused */
	MBOX_CMD_STAR_HSIC_IDLE,
	MBOX_CMD_STOP_HSIC_IDLE,
	MBOX_CMD_DBC_WAKE_STACK, /* unused */
	MBOX_CMD_HSIC_PRETEND_CONNECT,

	MBOX_CMD_MAX,

	/* Response message to above commands */
	MBOX_CMD_ACK = 128,
	MBOX_CMD_NACK
};

#define MSG_DATA_SHIFT		0
#define MSG_DATA_MASK		0xffffff
#define MSG_CMD_SHIFT		24
#define MSG_CMD_MASK		0xff

#define MBOX_PACK_MSG(c, d)	((((c) & MSG_CMD_MASK) << MSG_CMD_SHIFT) | \
				 (((d) & MSG_DATA_MASK) << MSG_DATA_SHIFT))
#define MBOX_MSG_CMD(m)		(((m) >> MSG_CMD_SHIFT) & MSG_CMD_MASK)
#define MBOX_MSG_DATA(m)	(((m) >> MSG_DATA_SHIFT) & MSG_DATA_MASK)

struct tegra_xhci_hcd;
struct notifier_block;

/*
 * Tegra XUSB MBOX handler interface:
 *   - Drivers which may handle mbox messages should register a notifier.
 *   - The notifier event will be an mbox command (above) and the data will
 *     be a pointer to struct mbox_notifier_data.
 *   - If a notifier has handled the message, it should return NOTIFY_STOP
 *     and populate resp_{cmd,data} appropriately.
 *   - A resp_cmd of 0 indicates that no response should be sent.
 */
struct mbox_notifier_data {
	u32 msg_data; /* Inbound message data */
	u32 resp_cmd; /* Response message command */
	u32 resp_data; /* Response message data */
};

extern enum usb_device_speed tegra_xhci_port_speed(struct tegra_xhci_hcd *tegra,
						   unsigned int port);
extern bool tegra_xhci_port_connected(struct tegra_xhci_hcd *tegra,
				      unsigned int port);
extern bool tegra_xhci_port_may_wakeup(struct tegra_xhci_hcd *tegra,
				       unsigned int port);
extern int tegra_xhci_register_mbox_notifier(struct tegra_xhci_hcd *tegra,
					     struct notifier_block *nb);
extern void tegra_xhci_unregister_mbox_notifier(struct tegra_xhci_hcd *tegra,
						struct notifier_block *nb);

#endif /* _TEGRA_XUSB_H */
