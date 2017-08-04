/*
**************************************************************************
 * Rockchip driver for CIF ISP 1.1
 * (Based on Intel driver for sofiaxxx)
 *
 * Copyright (C) 2015 Intel Mobile Communications GmbH
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
**************************************************************************
 */
 #ifndef _CIF_ISP11_RK_VERSION_H_
#define _CIF_ISP11_RK_VERSION_H_
#include <linux/version.h>

/*
*       CIF DRIVER VERSION NOTE
*
*v0.1.0:
*1. New mi register update mode is invalidate in raw/jpeg for rv1108,
* All path used old mode for rv1108;
*v0.1.1:
*1. Modify CIF stop sequence for fix isp bus may dead when switch isp:
*Original stop sequence: Stop ISP(mipi) -> Stop ISP(isp) ->wait for ISP isp off -> Stop ISP(mi)
*Current stop sequence: ISP(mi) stop in mi frame end -> Stop ISP(mipi) -> Stop ISP(isp) ->wait for ISP isp off;
* Current stop sequence is only match sensor stream v-blanking >= 1.5ms;
*
*v0.1.2:
*1. Disable CIF_MIPI_ERR_DPHY interrupt here temporary for
*isp bus may be dead when switch isp;
*2. Cancel hw restart isp operation in mipi isr, only notice error log;
*
*v0.1.3:
*1. fix camerahal query exp info failed from cifisp_stat_buffer, because
*wake_up buffer before cif_isp11_sensor_mode_data_sync;
*
*v0.1.4:
*1. Disable DPHY errctrl interrupt, because this dphy erctrl signal
*is assert and until the next changes in line state. This time is may
*be too long and cpu is hold in this interrupt. Enable DPHY errctrl
*interrupt again, if mipi have receive the whole frame without any error.
*2. Modify mipi_dphy_cfg follow vendor recommended process in
*document.
*3. Select the limit dphy setting if sensor mipi datarate is overflow,
*and print warning information to user.
*
*v0.1.5:
*Exposure list must be queue operation, not stack. list_add switch to
*list_add_tail in cif_isp11_s_exp;
*
*v0.1.6:
*Add isp output size in struct isp_supplemental_sensor_mode_data.
*
*v0.1.7:
*1. Direct config isp lsc table size in cifisp_lsc_config. Because active_lsc_width
*is not same with isp register after isp reset.
*2. Support separate config sensor gain and shutter time for some sensor which
*gain and shutter isn't valid at the same time. Ex. ov2710.
*
*v0.1.8:
*1. Register v4l2 subdev for support subdev api; ex s_frame_interval;
*
*v0.1.9:
*1. Isp device video node is dynamic;
*2. Isp dma device add querycap ops;
*
*v0.1.a:
*1. fix struct cifisp_hist_stat .hist_bins unsigned short to unsigned int;
*
*v0.1.b:
*1. add wdr config in cifisp_isp_isr_other_config;
*
*v0.1.c:
*1. sync other_configs/meas_config info in readout work;
*2. meta data must sync in every mi frame end isr,
*3. isp configs info may be async.
*
*v0.1.d:
*1. add cif_isp11_v4l2_g_frame_interval / cif_isp11_v4l2_s_frame_interval;
*
*v0.1.e
*1. support lsensor;
*
*v0.1.f:
*1. support Y8/Y10 format.
*
*v0.2.0:
*1. Resume isp internal reset by VI_IRCL CIF_IRCL_CIF_SW_RST bit in cif_isp11_config_cif;
*Isp internal reset is cancel in v0.1.1.
*
*v0.2.1:
*1. disable CIF_ISP11_PIX_FMT_Y_AS_BAYER macro defaultly
*
*v0.2.2:
*1. Fix goc update bit isn't been clear in cifisp_isp_isr_other_config;
*
*v0.2.3:
*1. Fix module enable isn't match current value, if new module parameters set,
*but enable bit isn't switch;
*
*v0.2.4:
*1. Isp metadata is independent;
*
*v0.2.5:
*1. Fix cifisp_configure_isp is invalidate if isp parameter not update,
*this is trigger when two path enable;
*
*v0.2.6:
*1. Fix cif_isp11_start check stream on before access stream list;,
*2. Add control colorspace by v4l2_format.fmt.pix.colorspace;
*
*v0.2.7:
*1. Delete aec measurement window size check in cifisp_aec_param;
*
*v0.2.8:
*1. output->quantization filled error by sp.output.quantization in
*cif_isp11_config_isp when mp is ready;
*2. Cifisp stream off, if cif output is raw.  Because check isp_config.out.pix
*may be error in cifisp_streamon, if isp dev streamon before path dev;
*
*v0.2.9:
*1. fix dpf enable may be set when dpf strength config;
*
*v0.2.a:
*1. Support Y8 for AEC;
*
*v0.2.b:
*1. Add error judgement and spinlock to fix kernel crash in long time test;
*Change the call of drain_workqueue;
*/

#define CONFIG_CIFISP11_DRIVER_VERSION KERNEL_VERSION(0, 2, 0xb)


#endif
