/*
 * DHD debugability header file
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * Portions of this code are copyright (c) 2022 Cypress Semiconductor Corporation
 *
 * Copyright (C) 1999-2017, Broadcom Corporation
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: dhd_mschdbg.h 571265 2015-07-14 20:50:18Z $
 */

#ifndef _dhd_mschdbg_h_
#define _dhd_mschdbg_h_

#ifdef SHOW_LOGTRACE
extern void wl_mschdbg_event_handler(dhd_pub_t *dhdp, void *raw_event_ptr, int type,
	void *data, int len);
extern void wl_mschdbg_verboselog_handler(dhd_pub_t *dhdp, void *raw_event_ptr,
	prcd_event_log_hdr_t *plog_hdr, uint32 *log_ptr);
#endif /* SHOW_LOGTRACE */

#endif /* _dhd_mschdbg_h_ */
