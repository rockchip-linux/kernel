
#ifndef _FW_HEADER_8188E_T_H
#define _FW_HEADER_8188E_T_H

#ifdef LOAD_FW_HEADER_FROM_DRIVER
#if (defined(CONFIG_AP_WOWLAN) || (DM_ODM_SUPPORT_TYPE & (ODM_AP)))
extern u8 array_mp_8188e_t_fw_ap[15098];
extern u32 array_length_mp_8188e_t_fw_ap;
#endif

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN)) || (DM_ODM_SUPPORT_TYPE & (ODM_CE))
extern u8 array_mp_8188e_t_fw_nic[15096];
extern u32 array_length_mp_8188e_t_fw_nic;
extern u8 array_mp_8188e_t_fw_nic_89em[14144];
extern u32 array_length_mp_8188e_t_fw_nic_89em;
extern u8 array_mp_8188e_t_fw_wowlan[16090];
extern u32 array_length_mp_8188e_t_fw_wowlan;
#endif
#endif /* end of LOAD_FW_HEADER_FROM_DRIVER */

#endif
