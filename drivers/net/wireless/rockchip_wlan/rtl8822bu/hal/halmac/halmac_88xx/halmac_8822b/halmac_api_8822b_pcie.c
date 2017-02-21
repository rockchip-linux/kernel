#include "../halmac_88xx_cfg.h"
#include "../halmac_api_88xx_pcie.h"
#include "halmac_8822b_cfg.h"

/**
 * halmac_mac_power_switch_8822b_pcie() - change mac power
 * @pHalmac_adapter
 * @halmac_power
 * Author : KaiYuan Chang
 * Return : HALMAC_RET_STATUS
 */
HALMAC_RET_STATUS
halmac_mac_power_switch_8822b_pcie(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN HALMAC_MAC_POWER	halmac_power
)
{
	u8 interface_mask;
	u8 value8;
	VOID *pDriver_adapter = NULL;
	PHALMAC_API pHalmac_api;

	if (HALMAC_RET_SUCCESS != halmac_adapter_validate(pHalmac_adapter))
		return HALMAC_RET_ADAPTER_INVALID;

	if (HALMAC_RET_SUCCESS != halmac_api_validate(pHalmac_adapter))
		return HALMAC_RET_API_INVALID;

	halmac_api_record_id_88xx(pHalmac_adapter, HALMAC_API_MAC_POWER_SWITCH);

	pDriver_adapter = pHalmac_adapter->pDriver_adapter;
	pHalmac_api = (PHALMAC_API)pHalmac_adapter->pHalmac_api;

	PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_TRACE, "halmac_mac_power_switch_88xx_pcie halmac_power =  %x ==========>\n", halmac_power);
	interface_mask = HALMAC_PWR_INTF_PCI_MSK;

	value8 = HALMAC_REG_READ_8(pHalmac_adapter, REG_CR);
	if (0xEA == value8)
		pHalmac_adapter->halmac_state.mac_power = HALMAC_MAC_POWER_OFF;
	else
		pHalmac_adapter->halmac_state.mac_power = HALMAC_MAC_POWER_ON;

	/* Check if power switch is needed */
	if (halmac_power == HALMAC_MAC_POWER_ON && pHalmac_adapter->halmac_state.mac_power == HALMAC_MAC_POWER_ON) {
		PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_WARN, "halmac_mac_power_switch power state unchange!\n");
		return HALMAC_RET_PWR_UNCHANGE;
	} else {
		if (HALMAC_MAC_POWER_OFF == halmac_power) {
			if (HALMAC_RET_SUCCESS != halmac_pwr_seq_parser_88xx(pHalmac_adapter, HALMAC_PWR_CUT_ALL_MSK, HALMAC_PWR_FAB_TSMC_MSK,
				    interface_mask, halmac_8822b_card_disable_flow)) {
				PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_ERR, "Handle power off cmd error\n");
				return HALMAC_RET_POWER_OFF_FAIL;
			}

			pHalmac_adapter->halmac_state.mac_power = HALMAC_MAC_POWER_OFF;
			pHalmac_adapter->halmac_state.ps_state = HALMAC_PS_STATE_UNDEFINE;
			pHalmac_adapter->halmac_state.dlfw_state = HALMAC_DLFW_NONE;
			halmac_init_adapter_dynamic_para_88xx(pHalmac_adapter);
		} else {
			if (HALMAC_RET_SUCCESS != halmac_pwr_seq_parser_88xx(pHalmac_adapter, HALMAC_PWR_CUT_ALL_MSK, HALMAC_PWR_FAB_TSMC_MSK,
				    interface_mask, halmac_8822b_card_enable_flow)) {
				PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_ERR, "Handle power on cmd error\n");
				return HALMAC_RET_POWER_ON_FAIL;
			}

			pHalmac_adapter->halmac_state.mac_power = HALMAC_MAC_POWER_ON;
			pHalmac_adapter->halmac_state.ps_state = HALMAC_PS_STATE_ACT;
		}
	}

	PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_TRACE, "halmac_mac_power_switch_88xx_pcie <==========\n");

	return HALMAC_RET_SUCCESS;
}

/**
 * halmac_pcie_switch_8822b() - change mac power
 * @pHalmac_adapter
 * @pcie_cfg
 * Author : KaiYuan Chang
 * Return : HALMAC_RET_STATUS
 */
HALMAC_RET_STATUS
halmac_pcie_switch_8822b(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN HALMAC_PCIE_CFG	pcie_cfg
)
{
	VOID *pDriver_adapter = NULL;
	PHALMAC_API pHalmac_api;
	u8 current_link_speed = 0;
	u32 count = 0;

	if (HALMAC_RET_SUCCESS != halmac_adapter_validate(pHalmac_adapter))
		return HALMAC_RET_ADAPTER_INVALID;

	if (HALMAC_RET_SUCCESS != halmac_api_validate(pHalmac_adapter))
		return HALMAC_RET_API_INVALID;

	halmac_api_record_id_88xx(pHalmac_adapter, HALMAC_API_PCIE_SWITCH);

	pDriver_adapter = pHalmac_adapter->pDriver_adapter;
	pHalmac_api = (PHALMAC_API)pHalmac_adapter->pHalmac_api;

	PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_TRACE, "halmac_pcie_switch_8822b ==========>\n");

	/**
	* Link Control 2 Register[3:0] Target Link Speed
	* Defined encodings are:
	* 0001b Target Link 2.5 GT/s
	* 0010b Target Link 5.0 GT/s
	* 0100b Target Link 8.0 GT/s
	*/

	if (HALMAC_PCIE_GEN1 == pcie_cfg) {

		/* cfg 0xA0[3:0]=4'b0001 */
		halmac_dbi_write8_88xx(pHalmac_adapter, LINK_CTRL2_REG_OFFSET, (halmac_dbi_read8_88xx(pHalmac_adapter, LINK_CTRL2_REG_OFFSET) & 0xF0) | BIT(0));

		/* cfg 0x80C[17]=1 //PCIe DesignWave */
		halmac_dbi_write32_88xx(pHalmac_adapter, GEN2_CTRL_OFFSET, halmac_dbi_read32_88xx(pHalmac_adapter, GEN2_CTRL_OFFSET) | BIT(17));

		/* check link speed if GEN1 */
		/* cfg 0x82[3:0]=4'b0001 */
		current_link_speed = halmac_dbi_read8_88xx(pHalmac_adapter, LINK_STATUS_REG_OFFSET) & 0x0F ;
		count = 2000;

		while ((current_link_speed != GEN1_SPEED) && (count != 0)) {
			PLATFORM_RTL_DELAY_US(pDriver_adapter, 50);
			current_link_speed = halmac_dbi_read8_88xx(pHalmac_adapter, LINK_STATUS_REG_OFFSET) & 0x0F ;
			count--;
		}

		if (current_link_speed != GEN1_SPEED) {
			PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_ERR, "Speed change to GEN1 fail !\n");
			return HALMAC_RET_FAIL;
		}

	} else if (HALMAC_PCIE_GEN2 == pcie_cfg) {

		/* cfg 0xA0[3:0]=4'b0010 */
		halmac_dbi_write8_88xx(pHalmac_adapter, LINK_CTRL2_REG_OFFSET, (halmac_dbi_read8_88xx(pHalmac_adapter, LINK_CTRL2_REG_OFFSET) & 0xF0) | BIT(1));

		/* cfg 0x80C[17]=1 //PCIe DesignWave */
		halmac_dbi_write32_88xx(pHalmac_adapter, GEN2_CTRL_OFFSET, halmac_dbi_read32_88xx(pHalmac_adapter, GEN2_CTRL_OFFSET) | BIT(17));

		/* check link speed if GEN2 */
		/* cfg 0x82[3:0]=4'b0010 */
		current_link_speed = halmac_dbi_read8_88xx(pHalmac_adapter, LINK_STATUS_REG_OFFSET) & 0x0F ;
		count = 2000;

		while ((current_link_speed != GEN2_SPEED) && (count != 0)) {
			PLATFORM_RTL_DELAY_US(pDriver_adapter, 50);
			current_link_speed = halmac_dbi_read8_88xx(pHalmac_adapter, LINK_STATUS_REG_OFFSET) & 0x0F ;
			count++;
		}

		if (current_link_speed != GEN2_SPEED) {
			PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_ERR, "Speed change to GEN1 fail !\n");
			return HALMAC_RET_FAIL;
		}

	} else {
		PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_ERR, "Error Speed !\n");
		return HALMAC_RET_FAIL;
	}

	PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_TRACE, "halmac_pcie_switch_8822b <==========\n");

	return HALMAC_RET_SUCCESS;
}

/**
 * halmac_pcie_switch_8822b_nc() - change mac power
 * @pHalmac_adapter
 * @pcie_cfg
 * Author : KaiYuan Chang
 * Return : HALMAC_RET_STATUS
 */
HALMAC_RET_STATUS
halmac_pcie_switch_8822b_nc(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN HALMAC_PCIE_CFG	pcie_cfg
)
{
	VOID *pDriver_adapter = NULL;
	PHALMAC_API pHalmac_api;

	if (HALMAC_RET_SUCCESS != halmac_adapter_validate(pHalmac_adapter))
		return HALMAC_RET_ADAPTER_INVALID;

	if (HALMAC_RET_SUCCESS != halmac_api_validate(pHalmac_adapter))
		return HALMAC_RET_API_INVALID;

	halmac_api_record_id_88xx(pHalmac_adapter, HALMAC_API_PCIE_SWITCH);

	pDriver_adapter = pHalmac_adapter->pDriver_adapter;
	pHalmac_api = (PHALMAC_API)pHalmac_adapter->pHalmac_api;

	PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_TRACE, "halmac_pcie_switch_8822b_nc ==========>\n");

	PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_TRACE, "halmac_pcie_switch_8822b_nc <==========\n");

	return HALMAC_RET_SUCCESS;
}

/**
 * halmac_phy_cfg_8822b_pcie() - phy config
 * @pHalmac_adapter
 * Author : KaiYuan Chang
 * Return : HALMAC_RET_STATUS
 */
HALMAC_RET_STATUS
halmac_phy_cfg_8822b_pcie(
	IN PHALMAC_ADAPTER pHalmac_adapter,
	IN HALMAC_INTF_PHY_PLATFORM platform
)
{
	VOID *pDriver_adapter = NULL;
	HALMAC_RET_STATUS status = HALMAC_RET_SUCCESS;
	PHALMAC_API pHalmac_api;

	if (HALMAC_RET_SUCCESS != halmac_adapter_validate(pHalmac_adapter))
		return HALMAC_RET_ADAPTER_INVALID;

	if (HALMAC_RET_SUCCESS != halmac_api_validate(pHalmac_adapter))
		return HALMAC_RET_API_INVALID;

	halmac_api_record_id_88xx(pHalmac_adapter, HALMAC_API_PHY_CFG);

	pDriver_adapter = pHalmac_adapter->pDriver_adapter;
	pHalmac_api = (PHALMAC_API)pHalmac_adapter->pHalmac_api;

	PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_TRACE, "halmac_phy_cfg ==========>\n");

	status = halmac_parse_intf_phy_88xx(pHalmac_adapter, HALMAC_RTL8822B_PCIE_PHY_GEN1, platform, HAL_INTF_PHY_PCIE_GEN1);

	if (HALMAC_RET_SUCCESS != status)
		return status;

	status = halmac_parse_intf_phy_88xx(pHalmac_adapter, HALMAC_RTL8822B_PCIE_PHY_GEN2, platform, HAL_INTF_PHY_PCIE_GEN2);

	if (HALMAC_RET_SUCCESS != status)
		return status;

	PLATFORM_MSG_PRINT(pDriver_adapter, HALMAC_MSG_PWR, HALMAC_DBG_TRACE, "halmac_phy_cfg <==========\n");

	return HALMAC_RET_SUCCESS;
}
