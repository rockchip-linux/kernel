#ifndef _FW_HEADER_8188E_S_H
#define _FW_HEADER_8188E_S_H
#ifdef CONFIG_SFW_SUPPORTED
#ifdef LOAD_FW_HEADER_FROM_DRIVER
#if (defined(CONFIG_AP_WOWLAN) || (DM_ODM_SUPPORT_TYPE & (ODM_AP)))
extern u8 array_mp_8188e_s_fw_ap[15618];
extern u32 array_length_mp_8188e_s_fw_ap;
#endif

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN)) || (DM_ODM_SUPPORT_TYPE & (ODM_CE))
extern u8 array_mp_8188e_s_fw_nic[19114];
extern u32 array_length_mp_8188e_s_fw_nic;
extern u8 array_mp_8188e_s_fw_wowlan[22042];
extern u32 array_length_mp_8188e_s_fw_wowlan;
#endif
#endif /* end of LOAD_FW_HEADER_FROM_DRIVER */

#endif
#endif /* end of CONFIG_SFW_SUPPORTED */
