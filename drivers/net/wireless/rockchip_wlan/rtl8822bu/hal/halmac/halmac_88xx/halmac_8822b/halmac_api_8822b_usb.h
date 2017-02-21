#ifndef _HALMAC_API_8822B_USB_H_
#define _HALMAC_API_8822B_USB_H_

extern HALMAC_INTF_PHY_PARA HALMAC_RTL8822B_USB2_PHY[];
extern HALMAC_INTF_PHY_PARA HALMAC_RTL8822B_USB3_PHY[];

#include "../../halmac_2_platform.h"
#include "../../halmac_type.h"

HALMAC_RET_STATUS
halmac_mac_power_switch_8822b_usb(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN HALMAC_MAC_POWER halmac_power
);

HALMAC_RET_STATUS
halmac_phy_cfg_8822b_usb(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN HALMAC_INTF_PHY_PLATFORM platform
);
#endif/* _HALMAC_API_8822B_USB_H_ */
