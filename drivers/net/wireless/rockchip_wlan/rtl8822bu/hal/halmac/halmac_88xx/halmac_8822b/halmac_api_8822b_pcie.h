#ifndef _HALMAC_API_8822B_PCIE_H_
#define _HALMAC_API_8822B_PCIE_H_

#include "../../halmac_2_platform.h"
#include "../../halmac_type.h"

extern HALMAC_INTF_PHY_PARA HALMAC_RTL8822B_PCIE_PHY_GEN1[];
extern HALMAC_INTF_PHY_PARA HALMAC_RTL8822B_PCIE_PHY_GEN2[];

HALMAC_RET_STATUS
halmac_mac_power_switch_8822b_pcie(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN HALMAC_MAC_POWER halmac_power
);

HALMAC_RET_STATUS
halmac_pcie_switch_8822b(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN HALMAC_PCIE_CFG	pcie_cfg
);

HALMAC_RET_STATUS
halmac_pcie_switch_8822b_nc(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN HALMAC_PCIE_CFG	pcie_cfg
);

HALMAC_RET_STATUS
halmac_phy_cfg_8822b_pcie(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN HALMAC_INTF_PHY_PLATFORM platform
);

#endif/* _HALMAC_API_8822B_PCIE_H_ */
