
#ifdef CONFIG_RTL8723D
#ifndef _FW_HEADER_8723D_H
#define _FW_HEADER_8723D_H

#ifdef LOAD_FW_HEADER_FROM_DRIVER
#if (defined(CONFIG_AP_WOWLAN) || (DM_ODM_SUPPORT_TYPE & (ODM_AP)))
extern u1Byte Array_MP_8723D_FW_AP[20958];
extern u4Byte ArrayLength_MP_8723D_FW_AP;
#endif

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN)) || (DM_ODM_SUPPORT_TYPE & (ODM_CE))
extern u1Byte Array_MP_8723D_FW_NIC[23228];
extern u4Byte ArrayLength_MP_8723D_FW_NIC;

extern u1Byte Array_MP_8723D_FW_WoWLAN[28450];
extern u4Byte ArrayLength_MP_8723D_FW_WoWLAN;
#endif
#endif /* end of LOAD_FW_HEADER_FROM_DRIVER*/

#endif
#endif /* end of HWIMG_SUPPORT*/


