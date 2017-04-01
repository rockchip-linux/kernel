
#ifdef CONFIG_RTL8822B

#ifndef _FW_HEADER_8822B_H
#define _FW_HEADER_8822B_H

#ifdef LOAD_FW_HEADER_FROM_DRIVER
#if (defined(CONFIG_AP_WOWLAN) || (DM_ODM_SUPPORT_TYPE & (ODM_AP)))
extern u8 array_mp_8822b_fw_ap[76464];
extern u32 array_length_mp_8822b_fw_ap;
#endif

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN)) || (DM_ODM_SUPPORT_TYPE & (ODM_CE))
extern u8 array_mp_8822b_fw_nic[119880];
extern u32 array_length_mp_8822b_fw_nic;
extern u8 array_mp_8822b_fw_wowlan[66024];
extern u32 array_length_mp_8822b_fw_wowlan;
#endif
#endif /* end of LOAD_FW_HEADER_FROM_DRIVER */

#endif

#endif

