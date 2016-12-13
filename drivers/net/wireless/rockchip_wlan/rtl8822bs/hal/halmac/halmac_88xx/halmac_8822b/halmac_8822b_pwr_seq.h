#ifndef HALMAC_POWER_SEQUENCE_8822B
#define HALMAC_POWER_SEQUENCE_8822B

#include "../../halmac_pwr_seq_cmd.h"

#define HALMAC_8822B_PWR_SEQ_VER  "V14"
extern PHALMAC_WLAN_PWR_CFG halmac_8822b_card_disable_flow[];
extern PHALMAC_WLAN_PWR_CFG halmac_8822b_card_enable_flow[];
extern PHALMAC_WLAN_PWR_CFG halmac_8822b_suspend_flow[];
extern PHALMAC_WLAN_PWR_CFG halmac_8822b_resume_flow[];
extern PHALMAC_WLAN_PWR_CFG halmac_8822b_hwpdn_flow[];
extern PHALMAC_WLAN_PWR_CFG halmac_8822b_enter_lps_flow[];
extern PHALMAC_WLAN_PWR_CFG halmac_8822b_enter_deep_lps_flow[];
extern PHALMAC_WLAN_PWR_CFG halmac_8822b_leave_lps_flow[];

#endif
