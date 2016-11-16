/*
 * drivers/video/rockchip/lcdc/rk_vop_full.h
 *
 * Copyright (C) 2016 ROCKCHIP, Inc.
 * Author: hjc <hjc@rock-chips.com>
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef RK_VOP_FULL_H_
#define RK_VOP_FULL_H_
#include<linux/rk_fb.h>
#include<linux/io.h>
#include<linux/clk.h>
#include<linux/mfd/syscon.h>
#include<linux/regmap.h>

#define VOP_INPUT_MAX_WIDTH 4096
/*
 * Registers in this file
 * REG_CFG_DONE: Register config done flag
 * VERSION_INFO: Version for vop
 * SYS_CTRL: System control register0
 * SYS_CTRL1: System control register1
 * DSP_CTRL0: Display control register0
 * DSP_CTRL1: Display control register1
 * DSP_BG: Background color
 * MCU_CTRL: MCU mode control register
 * WB_CTRL0: write back ctrl0
 * WB_CTRL1: write back ctrl1
 * WB_YRGB_MST: write back yrgb mst
 * WB_CBR_MST: write back cbr mst
 * WIN0_CTRL0: Win0 ctrl register0
 * WIN0_CTRL1: Win0 ctrl register1
 * WIN0_COLOR_KEY: Win0 color key register
 * WIN0_VIR: Win0 virtual stride
 * WIN0_YRGB_MST: Win0 YRGB memory start address
 * WIN0_CBR_MST: Win0 Cbr memory start address
 * WIN0_ACT_INFO: Win0 active window width/height
 * WIN0_DSP_INFO: Win0 display width/height on panel
 * WIN0_DSP_ST: Win0 display start point on panel
 * WIN0_SCL_FACTOR_YRGB: Win0 YRGB scaling factor
 * WIN0_SCL_FACTOR_CBR: Win0 Cbr scaling factor
 * WIN0_SCL_OFFSET: Win0 scaling start point offset
 * WIN0_SRC_ALPHA_CTRL: Win0 alpha source control register
 * WIN0_DST_ALPHA_CTRL: Win0 alpha destination control register
 * WIN0_FADING_CTRL: Win0 fading contrl register
 * WIN0_CTRL2: Win0 ctrl register2
 * WIN1_CTRL0: Win1 ctrl register0
 * WIN1_CTRL1: Win1 ctrl register1
 * WIN1_COLOR_KEY: Win1 color key register
 * WIN1_VIR: win1 virtual stride
 * WIN1_YRGB_MST: Win1 YRGB memory start address
 * WIN1_CBR_MST: Win1 Cbr memory start address
 * WIN1_ACT_INFO: Win1 active window width/height
 * WIN1_DSP_INFO: Win1 display width/height on panel
 * WIN1_DSP_ST: Win1 display start point on panel
 * WIN1_SCL_FACTOR_YRGB: Win1 YRGB scaling factor
 * WIN1_SCL_FACTOR_CBR: Win1 Cbr scaling factor
 * WIN1_SCL_OFFSET: Win1 scaling start point offset
 * WIN1_SRC_ALPHA_CTRL: Win1 alpha source control register
 * WIN1_DST_ALPHA_CTRL: Win1 alpha destination control register
 * WIN1_FADING_CTRL: Win1 fading contrl register
 * WIN1_CTRL2: Win1 ctrl register2
 * WIN2_CTRL0: win2 ctrl register0
 * WIN2_CTRL1: win2 ctrl register1
 * WIN2_VIR0_1: Win2 virtual stride0 and virtual stride1
 * WIN2_VIR2_3: Win2 virtual stride2 and virtual stride3
 * WIN2_MST0: Win2 memory start address0
 * WIN2_DSP_INFO0: Win2 display width0/height0 on panel
 * WIN2_DSP_ST0: Win2 display start point0 on panel
 * WIN2_COLOR_KEY: Win2 color key register
 * WIN2_MST1: Win2 memory start address1
 * WIN2_DSP_INFO1: Win2 display width1/height1 on panel
 * WIN2_DSP_ST1: Win2 display start point1 on panel
 * WIN2_SRC_ALPHA_CTRL: Win2 alpha source control register
 * WIN2_MST2: Win2 memory start address2
 * WIN2_DSP_INFO2:  Win2 display width2/height2 on panel
 * WIN2_DSP_ST2: Win2 display start point2 on panel
 * WIN2_DST_ALPHA_CTRL: Win2 alpha destination control register
 * WIN2_MST3: Win2 memory start address3
 * WIN2_DSP_INFO3:  Win2 display width3/height3 on panel
 * WIN2_DSP_ST3: Win2 display start point3 on panel
 * WIN2_FADING_CTRL: Win2 fading contrl register
 * WIN3_CTRL0: Win3 ctrl register0
 * WIN3_CTRL1: Win3 ctrl register1
 * WIN3_VIR0_1: Win3 virtual stride0 and virtual stride1
 * WIN3_VIR2_3: Win3 virtual stride2 and virtual stride3
 * WIN3_MST0: Win3 memory start address0
 * WIN3_DSP_INFO0: Win3 display width0/height0 on panel
 * WIN3_DSP_ST0: Win3 display start point0 on panel
 * WIN3_COLOR_KEY: Win3 color key register
 * WIN3_MST1: Win3 memory start address1
 * WIN3_DSP_INFO1:  Win3 display width1/height1 on panel
 * WIN3_DSP_ST1: Win3 display start point1 on panel
 * WIN3_SRC_ALPHA_CTRL: Win3 alpha source control register
 * WIN3_MST2: Win3 memory start address2
 * WIN3_DSP_INFO2:  Win3 display width2/height2 on panel
 * WIN3_DSP_ST2: Win3 display start point2 on panel
 * WIN3_DST_ALPHA_CTRL: Win3 alpha destination control register
 * WIN3_MST3: Win3 memory start address3
 * WIN3_DSP_INFO3:  Win3 display width3/height3 on panel
 * WIN3_DSP_ST3: Win3 display start point3 on panel
 * WIN3_FADING_CTRL: Win3 fading contrl register
 * HWC_CTRL0: Hwc ctrl register0
 * HWC_CTRL1: Hwc ctrl register1
 * HWC_MST: Hwc memory start address
 * HWC_DSP_ST: Hwc display start point on panel
 * HWC_SRC_ALPHA_CTRL: Hwc alpha source control register
 * HWC_DST_ALPHA_CTRL: Hwc alpha destination control register
 * HWC_FADING_CTRL: Hwc fading contrl register
 * HWC_RESERVED1: Hwc reserved
 * POST_DSP_HACT_INFO: Post scaler down horizontal start and end
 * POST_DSP_VACT_INFO: Panel active horizontal scanning start point
 *                     and end point
 * POST_SCL_FACTOR_YRGB: Post yrgb scaling factor
 * POST_RESERVED: Post reserved
 * POST_SCL_CTRL: Post scaling start point offset
 * POST_DSP_VACT_INFO_F1: Panel active horizontal scanning start point
 *                        and end point F1
 * DSP_HTOTAL_HS_END: Panel scanning horizontal width and hsync pulse end point
 * DSP_HACT_ST_END: Panel active horizontal scanning start point and end point
 * DSP_VTOTAL_VS_END: Panel scanning vertical height and vsync pulse end point
 * DSP_VACT_ST_END: Panel active vertical scanning start point and end point
 * DSP_VS_ST_END_F1: Vertical scanning start point and vsync pulse end point
 *                   of even filed in interlace mode
 * DSP_VACT_ST_END_F1: Vertical scanning active start point and end point of
 *                     even filed in interlace mode
 * PWM_CTRL: PWM Control Register
 * PWM_PERIOD_HPR: PWM Period Register/High Polarity Capture Register
 * PWM_DUTY_LPR: PWM Duty Register/Low Polarity Capture Register
 * PWM_CNT: PWM Counter Register
 * BCSH_COLOR_BAR: Color bar config register
 * BCSH_BCS: Brightness contrast saturation*contrast config register
 * BCSH_H: Sin hue and cos hue config register
 * BCSH_CTRL: BCSH contrl register
 * CABC_CTRL0: Content Adaptive Backlight Control register0
 * CABC_CTRL1: Content Adaptive Backlight Control register1
 * CABC_CTRL2: Content Adaptive Backlight Control register2
 * CABC_CTRL3: Content Adaptive Backlight Control register3
 * CABC_GAUSS_LINE0_0: CABC gauss line config register00
 * CABC_GAUSS_LINE0_1: CABC gauss line config register01
 * CABC_GAUSS_LINE1_0: CABC gauss line config register10
 * CABC_GAUSS_LINE1_1: CABC gauss line config register11
 * CABC_GAUSS_LINE2_0: CABC gauss line config register20
 * CABC_GAUSS_LINE2_1: CABC gauss line config register21
 * FRC_LOWER01_0: FRC lookup table config register010
 * FRC_LOWER01_1: FRC lookup table config register011
 * FRC_LOWER10_0: FRC lookup table config register100
 * FRC_LOWER10_1: FRC lookup table config register101
 * FRC_LOWER11_0: FRC lookup table config register110
 * FRC_LOWER11_1: FRC lookup table config register111
 * AFBCD0_CTRL:
 * AFBCD0_HDR_PTR:
 * AFBCD0_PIC_SIZE:
 * AFBCD0_STATUS:
 * AFBCD1_CTRL:
 * AFBCD1_HDR_PTR:
 * AFBCD1_PIC_SIZE:
 * AFBCD1_STATUS:
 * AFBCD2_CTRL:
 * AFBCD2_HDR_PTR:
 * AFBCD2_PIC_SIZE:
 * AFBCD2_STATUS:
 * AFBCD3_CTRL:
 * AFBCD3_HDR_PTR:
 * AFBCD3_PIC_SIZE:
 * AFBCD3_STATUS:
 * INTR_EN0: Interrupt enable register
 * INTR_CLEAR0: Interrupt clear register
 * INTR_STATUS0: interrupt  status
 * INTR_RAW_STATUS0: raw interrupt status
 * INTR_EN1: Interrupt enable register
 * INTR_CLEAR1: Interrupt clear register
 * INTR_STATUS1: interrupt  status
 * INTR_RAW_STATUS1: raw interrupt status
 * LINE_FLAG: Line flag config register
 * VOP_STATUS: vop status register
 * BLANKING_VALUE: Register0000 Abstract
 * WIN0_DSP_BG: Win0 layer background color
 * WIN1_DSP_BG: Win1 layer background color
 * WIN2_DSP_BG: Win2 layer background color
 * WIN3_DSP_BG: Win3 layer background color
 * DBG_PERF_LATENCY_CTRL0: Axi performance latency module contrl register0
 * DBG_PERF_RD_MAX_LATENCY_NUM0: Read max latency number
 * DBG_PERF_RD_LATENCY_THR_NUM0: The number of bigger than configed
 *                               threshold value
 * DBG_PERF_RD_LATENCY_SAMP_NUM0: Total sample number
 * DBG_CABC0: CABC debug register0
 * DBG_CABC1: CABC debug register1
 * DBG_CABC2: CABC debug register2
 * DBG_CABC3: CABC debug register3
 * DBG_WIN0_REG0: Vop debug win0 register0
 * DBG_WIN0_REG1: Vop debug win0 register1
 * DBG_WIN0_REG2: Vop debug win0 register2
 * DBG_WIN0_RESERVED: Vop debug win0 register3 reserved
 * DBG_WIN1_REG0: Vop debug win1 register0
 * DBG_WIN1_REG1: Vop debug win1 register1
 * DBG_WIN1_REG2: Vop debug win1 register2
 * DBG_WIN1_RESERVED: Vop debug win1 register3 reserved
 * DBG_WIN2_REG0: Vop debug win2 register0
 * DBG_WIN2_REG1: Vop debug win2 register1
 * DBG_WIN2_REG2: Vop debug win2 register2
 * DBG_WIN2_REG3: Vop debug win2 register3
 * DBG_WIN3_REG0: Vop debug win3 register0
 * DBG_WIN3_REG1: Vop debug win3 register1
 * DBG_WIN3_REG2: Vop debug win3 register2
 * DBG_WIN3_REG3: Vop debug win3 register3
 * DBG_PRE_REG0: Vop debug pre register0
 * DBG_PRE_RESERVED: Vop debug pre register1 reserved
 * DBG_POST_REG0: Vop debug post register0
 * DBG_POST_RESERVED: Vop debug post register1 reserved
 * DBG_DATAO: debug data output path
 * DBG_DATAO_2: debug data output path 2
 * WIN2_LUT_ADDR: Win2 lut base address
 * WIN3_LUT_ADDR: Win3 lut base address
 * HWC_LUT_ADDR: Hwc lut base address
 * GAMMA0_LUT_ADDR: GAMMA lut base address
 * GAMMA1_LUT_ADDR: GAMMA lut base address
 * CABC_GAMMA_LUT_ADDR: CABC GAMMA lut base address
 * MCU_BYPASS_WPORT:
 * MCU_BYPASS_RPORT:
 */

static inline u64 val_mask(int val, u64 msk, int shift)
{
	return (msk << (shift + 32)) | ((msk & val) << shift);
}

#define VAL_MASK(x, width, shift) val_mask(x, (1 << (width)) - 1, shift)

#define MASK(x) (V_##x(0) >> 32)

#define REG_CFG_DONE                    0x00000000
#define  V_REG_LOAD_EN(x)                       VAL_MASK(x, 1, 0)
#define  V_REG_LOAD_WIN0_EN(x)                  VAL_MASK(x, 1, 1)
#define  V_REG_LOAD_WIN1_EN(x)                  VAL_MASK(x, 1, 2)
#define  V_REG_LOAD_WIN2_EN(x)                  VAL_MASK(x, 1, 3)
#define  V_REG_LOAD_WIN3_EN(x)                  VAL_MASK(x, 1, 4)
#define  V_REG_LOAD_HWC_EN(x)                   VAL_MASK(x, 1, 5)
#define  V_REG_LOAD_IEP_EN(x)                   VAL_MASK(x, 1, 6)
#define  V_REG_LOAD_FBDC_EN(x)                  VAL_MASK(x, 1, 7)
#define  V_REG_LOAD_SYS_EN(x)                   VAL_MASK(x, 1, 8)
#define  V_WRITE_MASK(x)                        VAL_MASK(x, 16, 16)
#define VERSION_INFO                    0x00000004
#define  V_SVNBUILD(x)                          VAL_MASK(x, 16, 0)
#define  V_MINOR(x)                             VAL_MASK(x, 8, 16)
#define  V_MAJOR(x)                             VAL_MASK(x, 8, 24)
#define SYS_CTRL                        0x00000008
#define  V_DIRECT_PATH_EN(x)                    VAL_MASK(x, 1, 0)
#define  V_DIRECT_PATH_LAYER_SEL(x)             VAL_MASK(x, 2, 1)
#define  V_EDPI_HALT_EN(x)                      VAL_MASK(x, 1, 8)
#define  V_EDPI_WMS_MODE(x)                     VAL_MASK(x, 1, 9)
#define  V_EDPI_WMS_FS(x)                       VAL_MASK(x, 1, 10)
#define  V_GLOBAL_REGDONE_EN(x)                 VAL_MASK(x, 1, 11)
#define  V_RGB_OUT_EN(x)                        VAL_MASK(x, 1, 12)
#define  V_HDMI_OUT_EN(x)                       VAL_MASK(x, 1, 13)
#define  V_EDP_OUT_EN(x)                        VAL_MASK(x, 1, 14)
#define  V_MIPI_OUT_EN(x)                       VAL_MASK(x, 1, 15)
#define  V_OVERLAY_MODE(x)                      VAL_MASK(x, 1, 16)
#define  V_FS_SAME_ADDR_MASK_EN(x)              VAL_MASK(x, 1, 17)
#define  V_POST_LB_MODE(x)                      VAL_MASK(x, 1, 18)
#define  V_WIN23_PRI_OPT_MODE(x)                VAL_MASK(x, 1, 19)
#define  V_VOP_MMU_EN(x)                        VAL_MASK(x, 1, 20)
#define  V_VOP_DMA_STOP(x)                      VAL_MASK(x, 1, 21)
#define  V_VOP_STANDBY_EN(x)                    VAL_MASK(x, 1, 22)
#define  V_AUTO_GATING_EN(x)                    VAL_MASK(x, 1, 23)
#define  V_SW_IMD_TVE_DCLK_EN(x)                VAL_MASK(x, 1, 24)
#define  V_SW_IMD_TVE_DCLK_POL(x)               VAL_MASK(x, 1, 25)
#define  V_SW_TVE_MODE(x)                       VAL_MASK(x, 1, 26)
#define  V_SW_UV_OFFSET_EN(x)                   VAL_MASK(x, 1, 27)
#define  V_SW_GENLOCK(x)                        VAL_MASK(x, 1, 28)
#define  V_SW_DAC_SEL(x)                        VAL_MASK(x, 1, 29)
#define SYS_CTRL1                       0x0000000c
#define  V_NOC_HURRY_EN(x)                      VAL_MASK(x, 1, 0)
#define  V_NOC_HURRY_VALUE(x)                   VAL_MASK(x, 2, 1)
#define  V_NOC_HURRY_THRESHOLD(x)               VAL_MASK(x, 6, 3)
#define  V_NOC_QOS_EN(x)                        VAL_MASK(x, 1, 9)
#define  V_NOC_WIN_QOS(x)                       VAL_MASK(x, 2, 10)
#define  V_AXI_MAX_OUTSTANDING_EN(x)            VAL_MASK(x, 1, 12)
#define  V_AXI_OUTSTANDING_MAX_NUM(x)           VAL_MASK(x, 5, 13)
#define  V_NOC_HURRY_W_MODE(x)                  VAL_MASK(x, 2, 20)
#define  V_NOC_HURRY_W_VALUE(x)                 VAL_MASK(x, 2, 22)
#define  V_REG_DONE_FRM(x)                      VAL_MASK(x, 1, 24)
#define  V_ALPHA_HARD_CALC(x)                   VAL_MASK(x, 1, 27)
#define  V_LEVEL2_OVERLAY_EN(x)                 VAL_MASK(x, 1, 28)
#define  V_POST_BANDWIDTH_CTRL(x)               VAL_MASK(x, 1, 29)
#define  V_DCF_IDLE_EN(x)                       VAL_MASK(x, 1, 30)
#define  V_FP_STANDBY_EN(x)                     VAL_MASK(x, 1, 31)
#define DSP_CTRL0                       0x00000010
#define  V_DSP_OUT_MODE(x)                      VAL_MASK(x, 4, 0)
#define  V_SW_CORE_DCLK_SEL(x)                  VAL_MASK(x, 1, 4)
#define  V_SW_P2I_EN(x)                         VAL_MASK(x, 1, 5)
#define  V_DSP_DCLK_DDR(x)                      VAL_MASK(x, 1, 8)
#define  V_DSP_DDR_PHASE(x)                     VAL_MASK(x, 1, 9)
#define  V_DSP_INTERLACE(x)                     VAL_MASK(x, 1, 10)
#define  V_DSP_FIELD_POL(x)                     VAL_MASK(x, 1, 11)
#define  V_DSP_BG_SWAP(x)                       VAL_MASK(x, 1, 12)
#define  V_DSP_RB_SWAP(x)                       VAL_MASK(x, 1, 13)
#define  V_DSP_RG_SWAP(x)                       VAL_MASK(x, 1, 14)
#define  V_DSP_DELTA_SWAP(x)                    VAL_MASK(x, 1, 15)
#define  V_DSP_DUMMY_SWAP(x)                    VAL_MASK(x, 1, 16)
#define  V_DSP_OUT_ZERO(x)                      VAL_MASK(x, 1, 17)
#define  V_DSP_BLANK_EN(x)                      VAL_MASK(x, 1, 18)
#define  V_DSP_BLACK_EN(x)                      VAL_MASK(x, 1, 19)
#define  V_DSP_CCIR656_AVG(x)                   VAL_MASK(x, 1, 20)
#define  V_DSP_YUV_CLIP(x)                      VAL_MASK(x, 1, 21)
#define  V_DSP_X_MIR_EN(x)                      VAL_MASK(x, 1, 22)
#define  V_DSP_Y_MIR_EN(x)                      VAL_MASK(x, 1, 23)
#define DSP_CTRL1                       0x00000014
#define  V_DSP_LUT_EN(x)                        VAL_MASK(x, 1, 0)
#define  V_PRE_DITHER_DOWN_EN(x)                VAL_MASK(x, 1, 1)
#define  V_DITHER_DOWN_EN(x)                    VAL_MASK(x, 1, 2)
#define  V_DITHER_DOWN_MODE(x)                  VAL_MASK(x, 1, 3)
#define  V_DITHER_DOWN_SEL(x)                   VAL_MASK(x, 1, 4)
#define  V_DITHER_UP_EN(x)                      VAL_MASK(x, 1, 6)
#define  V_DSP_LAYER0_SEL(x)                    VAL_MASK(x, 2, 8)
#define  V_DSP_LAYER1_SEL(x)                    VAL_MASK(x, 2, 10)
#define  V_DSP_LAYER2_SEL(x)                    VAL_MASK(x, 2, 12)
#define  V_DSP_LAYER3_SEL(x)                    VAL_MASK(x, 2, 14)
#define  V_RGB_LVDS_HSYNC_POL(x)                VAL_MASK(x, 1, 16)
#define  V_RGB_LVDS_VSYNC_POL(x)                VAL_MASK(x, 1, 17)
#define  V_RGB_LVDS_DEN_POL(x)                  VAL_MASK(x, 1, 18)
#define  V_RGB_LVDS_DCLK_POL(x)                 VAL_MASK(x, 1, 19)
#define  V_HDMI_HSYNC_POL(x)                    VAL_MASK(x, 1, 20)
#define  V_HDMI_VSYNC_POL(x)                    VAL_MASK(x, 1, 21)
#define  V_HDMI_DEN_POL(x)                      VAL_MASK(x, 1, 22)
#define  V_HDMI_DCLK_POL(x)                     VAL_MASK(x, 1, 23)
#define  V_EDP_HSYNC_POL(x)                     VAL_MASK(x, 1, 24)
#define  V_EDP_VSYNC_POL(x)                     VAL_MASK(x, 1, 25)
#define  V_EDP_DEN_POL(x)                       VAL_MASK(x, 1, 26)
#define  V_EDP_DCLK_POL(x)                      VAL_MASK(x, 1, 27)
#define  V_MIPI_HSYNC_POL(x)                    VAL_MASK(x, 1, 28)
#define  V_MIPI_VSYNC_POL(x)                    VAL_MASK(x, 1, 29)
#define  V_MIPI_DEN_POL(x)                      VAL_MASK(x, 1, 30)
#define  V_MIPI_DCLK_POL(x)                     VAL_MASK(x, 1, 31)
#define DSP_BG                          0x00000018
#define  V_DSP_BG_BLUE(x)                       VAL_MASK(x, 10, 0)
#define  V_DSP_BG_GREEN(x)                      VAL_MASK(x, 10, 10)
#define  V_DSP_BG_RED(x)                        VAL_MASK(x, 10, 20)
#define AUTO_GATING_EN                  0x0000003c
#define  V_WIN0_ACLK_GATING_EN(x)               VAL_MASK(x, 1, 0)
#define  V_WIN1_ACLK_GATING_EN(x)               VAL_MASK(x, 1, 1)
#define  V_WIN2_ACLK_GATING_EN(x)               VAL_MASK(x, 1, 2)
#define  V_HWC_ACLK_GATING_EN(x)                VAL_MASK(x, 1, 4)
#define  V_OVERLAY_ACLK_GATING_EN(x)            VAL_MASK(x, 1, 5)
#define  V_GAMMA_ACLK_GATING_EN(x)              VAL_MASK(x, 1, 6)
#define  V_FIELD00000(x)                         VAL_MASK(x, 1, 7)
#define LINE_FLAG                       0x00000040
#define  V_DSP_LINE_FLAG_NUM_0(x)               VAL_MASK(x, 13, 0)
#define  V_DSP_LINE_FLAG_NUM_1(x)               VAL_MASK(x, 13, 16)
#define VOP_STATUS                      0x00000044
#define  V_DSP_VCNT(x)                          VAL_MASK(x, 13, 0)
#define  V_VOP_MMU_IDLE(x)                      VAL_MASK(x, 1, 16)
#define  V_DMA_STOP_VALID(x)                    VAL_MASK(x, 1, 17)
#define BLANKING_VALUE                  0x00000048
#define  V_BLANKING_VALUE(x)                    VAL_MASK(x, 24, 0)
#define  V_BLANKING_VALUE_CONFIG_EN(x)          VAL_MASK(x, 1, 24)
#define WIN0_DSP_BG                     0x00000050
#define  V_WIN0_DSP_BG_BLUE(x)                  VAL_MASK(x, 8, 0)
#define  V_WIN0_DSP_BG_GREEN(x)                 VAL_MASK(x, 8, 8)
#define  V_WIN0_DSP_BG_RED(x)                   VAL_MASK(x, 8, 16)
#define  V_WIN0_BG_EN(x)                        VAL_MASK(x, 1, 31)
#define WIN1_DSP_BG                     0x00000054
#define  V_WIN1_DSP_BG_BLUE(x)                  VAL_MASK(x, 8, 0)
#define  V_WIN1_DSP_BG_GREEN(x)                 VAL_MASK(x, 8, 8)
#define  V_WIN1_DSP_BG_RED(x)                   VAL_MASK(x, 8, 16)
#define  V_WIN1_BG_EN(x)                        VAL_MASK(x, 1, 31)
#define DBG_PERF_LATENCY_CTRL0          0x000000c0
#define  V_RD_LATENCY_EN(x)                     VAL_MASK(x, 1, 0)
#define  V_HAND_LATENCY_CLR(x)                  VAL_MASK(x, 1, 1)
#define  V_RD_LATENCY_MODE(x)                   VAL_MASK(x, 1, 2)
#define  V_RD_LATENCY_ID0(x)                    VAL_MASK(x, 4, 4)
#define  V_RD_LATENCY_THR(x)                    VAL_MASK(x, 12, 8)
#define  V_RD_LATENCY_ST_NUM(x)                 VAL_MASK(x, 5, 20)
#define DBG_PERF_RD_MAX_LATENCY_NUM0    0x000000c4
#define  V_RD_MAX_LATENCY_NUM_CH0(x)            VAL_MASK(x, 12, 0)
#define  V_RD_LATENCY_OVERFLOW_CH0(x)           VAL_MASK(x, 1, 16)
#define DBG_PERF_RD_LATENCY_THR_NUM0    0x000000c8
#define  V_RD_LATENCY_THR_NUM_CH0(x)            VAL_MASK(x, 24, 0)
#define DBG_PERF_RD_LATENCY_SAMP_NUM0   0x000000cc
#define  V_RD_LATENCY_SAMP_NUM_CH0(x)           VAL_MASK(x, 24, 0)
#define INTR_EN0                        0x000000e0
#define  V_INTR_EN_FS(x)                        VAL_MASK(x, 1, 0)
#define  V_INTR_EN_FS_NEW(x)                    VAL_MASK(x, 1, 1)
#define  V_INTR_EN_ADDR_SAME(x)                 VAL_MASK(x, 1, 2)
#define  V_INTR_EN_LINE_FLAG0(x)                VAL_MASK(x, 1, 3)
#define  V_INTR_EN_LINE_FLAG1(x)                VAL_MASK(x, 1, 4)
#define  V_INTR_EN_BUS_ERROR(x)                 VAL_MASK(x, 1, 5)
#define  V_INTR_EN_WIN0_EMPTY(x)                VAL_MASK(x, 1, 6)
#define  V_INTR_EN_WIN1_EMPTY(x)                VAL_MASK(x, 1, 7)
#define  V_INTR_EN_WIN2_EMPTY(x)                VAL_MASK(x, 1, 8)
#define  V_INTR_EN_WIN3_EMPTY(x)                VAL_MASK(x, 1, 9)
#define  V_INTR_EN_HWC_EMPTY(x)                 VAL_MASK(x, 1, 10)
#define  V_INTR_EN_POST_BUF_EMPTY(x)            VAL_MASK(x, 1, 11)
#define  V_INTR_EN_PWM_GEN(x)                   VAL_MASK(x, 1, 12)
#define  V_INTR_EN_DSP_HOLD_VALID(x)            VAL_MASK(x, 1, 13)
#define  V_INTR_EN_MMU(x)                       VAL_MASK(x, 1, 14)
#define  V_INTR_EN_DMA_FINISH(x)                VAL_MASK(x, 1, 15)
#define  V_WRITE_MASK(x)                        VAL_MASK(x, 16, 16)
#define INTR_CLEAR0                     0x000000e4
#define  V_INT_CLR_FS(x)                        VAL_MASK(x, 1, 0)
#define  V_INT_CLR_FS_NEW(x)                    VAL_MASK(x, 1, 1)
#define  V_INT_CLR_ADDR_SAME(x)                 VAL_MASK(x, 1, 2)
#define  V_INT_CLR_LINE_FLAG0(x)                VAL_MASK(x, 1, 3)
#define  V_INT_CLR_LINE_FLAG1(x)                VAL_MASK(x, 1, 4)
#define  V_INT_CLR_BUS_ERROR(x)                 VAL_MASK(x, 1, 5)
#define  V_INT_CLR_WIN0_EMPTY(x)                VAL_MASK(x, 1, 6)
#define  V_INT_CLR_WIN1_EMPTY(x)                VAL_MASK(x, 1, 7)
#define  V_INT_CLR_WIN2_EMPTY(x)                VAL_MASK(x, 1, 8)
#define  V_INT_CLR_WIN3_EMPTY(x)                VAL_MASK(x, 1, 9)
#define  V_INT_CLR_HWC_EMPTY(x)                 VAL_MASK(x, 1, 10)
#define  V_INT_CLR_POST_BUF_EMPTY(x)            VAL_MASK(x, 1, 11)
#define  V_INT_CLR_PWM_GEN(x)                   VAL_MASK(x, 1, 12)
#define  V_INT_CLR_DSP_HOLD_VALID(x)            VAL_MASK(x, 1, 13)
#define  V_INT_CLR_MMU(x)                       VAL_MASK(x, 1, 14)
#define  V_INT_CLR_DMA_FINISH(x)                VAL_MASK(x, 1, 15)
#define  V_WRITE_MASK(x)                        VAL_MASK(x, 16, 16)
#define INTR_STATUS0                    0x000000e8
#define  V_INT_STATUS_FS(x)                     VAL_MASK(x, 1, 0)
#define  V_INT_STATUS_FS_NEW(x)                 VAL_MASK(x, 1, 1)
#define  V_INT_STATUS_ADDR_SAME(x)              VAL_MASK(x, 1, 2)
#define  V_INT_STATUS_LINE_FLAG0(x)             VAL_MASK(x, 1, 3)
#define  V_INT_STATUS_LINE_FLAG1(x)             VAL_MASK(x, 1, 4)
#define  V_INT_STATUS_BUS_ERROR(x)              VAL_MASK(x, 1, 5)
#define  V_INT_STATUS_WIN0_EMPTY(x)             VAL_MASK(x, 1, 6)
#define  V_INT_STATUS_WIN1_EMPTY(x)             VAL_MASK(x, 1, 7)
#define  V_INT_STATUS_WIN2_EMPTY(x)             VAL_MASK(x, 1, 8)
#define  V_INT_STATUS_WIN3_EMPTY(x)             VAL_MASK(x, 1, 9)
#define  V_INT_STATUS_HWC_EMPTY(x)              VAL_MASK(x, 1, 10)
#define  V_INT_STATUS_POST_BUF_EMPTY(x)         VAL_MASK(x, 1, 11)
#define  V_INT_STATUS_PWM_GEN(x)                VAL_MASK(x, 1, 12)
#define  V_INT_STATUS_DSP_HOLD_VALID(x)         VAL_MASK(x, 1, 13)
#define  V_INT_STATUS_MMU(x)                    VAL_MASK(x, 1, 14)
#define  V_INT_STATUS_DMA_FINISH(x)             VAL_MASK(x, 1, 15)
#define INTR_RAW_STATUS0                0x000000ec
#define  V_INT_RAW_STATUS_FS(x)                 VAL_MASK(x, 1, 0)
#define  V_INT_RAW_STATUS_FS_NEW(x)             VAL_MASK(x, 1, 1)
#define  V_INT_RAW_STATUS_ADDR_SAME(x)          VAL_MASK(x, 1, 2)
#define  V_INT_RAW_STATUS_LINE_FRAG0(x)         VAL_MASK(x, 1, 3)
#define  V_INT_RAW_STATUS_LINE_FRAG1(x)         VAL_MASK(x, 1, 4)
#define  V_INT_RAW_STATUS_BUS_ERROR(x)          VAL_MASK(x, 1, 5)
#define  V_INT_RAW_STATUS_WIN0_EMPTY(x)         VAL_MASK(x, 1, 6)
#define  V_INT_RAW_STATUS_WIN1_EMPTY(x)         VAL_MASK(x, 1, 7)
#define  V_INT_RAW_STATUS_WIN2_EMPTY(x)         VAL_MASK(x, 1, 8)
#define  V_INT_RAW_STATUS_WIN3_EMPTY(x)         VAL_MASK(x, 1, 9)
#define  V_INT_RAW_STATUS_HWC_EMPTY(x)          VAL_MASK(x, 1, 10)
#define  V_INT_RAW_STATUS_POST_BUF_EMPTY(x)     VAL_MASK(x, 1, 11)
#define  V_INT_RAW_STATUS_PWM_GEN(x)            VAL_MASK(x, 1, 12)
#define  V_INT_RAW_STATUS_DSP_HOLD_VALID(x)     VAL_MASK(x, 1, 13)
#define  V_INT_RAW_STATUS_MMU(x)                VAL_MASK(x, 1, 14)
#define  V_INT_RAW_STATUS_DMA_FINISH(x)         VAL_MASK(x, 1, 15)
#define INTR_EN1                        0x000000f0
#define  V_INT_EN_FBCD0(x)                      VAL_MASK(x, 1, 0)
#define  V_INT_EN_FBCD1(x)                      VAL_MASK(x, 1, 1)
#define  V_INT_EN_FBCD2(x)                      VAL_MASK(x, 1, 2)
#define  V_INT_EN_FBCD3(x)                      VAL_MASK(x, 1, 3)
#define  V_INT_EN_AFBCD0_HREG_DEC_RESP(x)       VAL_MASK(x, 1, 4)
#define  V_INT_EN_AFBCD0_HREG_AXI_RRESP(x)      VAL_MASK(x, 1, 5)
#define  V_INT_EN_AFBCD1_HREG_DEC_RESP(x)       VAL_MASK(x, 1, 6)
#define  V_INT_EN_AFBCD1_HREG_AXI_RRESP(x)      VAL_MASK(x, 1, 7)
#define  V_INT_EN_AFBCD2_HREG_DEC_RESP(x)       VAL_MASK(x, 1, 8)
#define  V_INT_EN_AFBCD2_HREG_AXI_RRESP(x)      VAL_MASK(x, 1, 9)
#define  V_INT_EN_AFBCD3_HREG_DEC_RESP(x)       VAL_MASK(x, 1, 10)
#define  V_INT_EN_AFBCD3_HREG_AXI_RRESP(x)      VAL_MASK(x, 1, 11)
#define  V_INT_EN_WB_YRGB_FIFO_FULL(x)          VAL_MASK(x, 1, 12)
#define  V_INT_EN_WB_UV_FIFO_FULL(x)            VAL_MASK(x, 1, 13)
#define  V_INT_EN_WB_FINISH(x)                  VAL_MASK(x, 1, 14)
#define  V_WRITE_MASK(x)                        VAL_MASK(x, 16, 16)
#define INTR_CLEAR1                     0x000000f4
#define  V_INT_CLR_FBCD0(x)                     VAL_MASK(x, 1, 0)
#define  V_INT_CLR_FBCD1(x)                     VAL_MASK(x, 1, 1)
#define  V_INT_CLR_FBCD2(x)                     VAL_MASK(x, 1, 2)
#define  V_INT_CLR_FBCD3(x)                     VAL_MASK(x, 1, 3)
#define  V_INT_CLR_AFBCD0_HREG_DEC_RESP(x)      VAL_MASK(x, 1, 4)
#define  V_INT_CLR_AFBCD0_HREG_AXI_RRESP(x)     VAL_MASK(x, 1, 5)
#define  V_INT_CLR_AFBCD1_HREG_DEC_RESP(x)      VAL_MASK(x, 1, 6)
#define  V_INT_CLR_AFBCD1_HREG_AXI_RRESP(x)     VAL_MASK(x, 1, 7)
#define  V_INT_CLR_AFBCD2_HREG_DEC_RESP(x)      VAL_MASK(x, 1, 8)
#define  V_INT_CLR_AFBCD2_HREG_AXI_RRESP(x)     VAL_MASK(x, 1, 9)
#define  V_INT_CLR_AFBCD3_HREG_DEC_RESP(x)      VAL_MASK(x, 1, 10)
#define  V_INT_CLR_AFBCD3_HREG_AXI_RRESP(x)     VAL_MASK(x, 1, 11)
#define INTR_STATUS1                    0x000000f8
#define  V_INT_STATUS_FBCD0(x)                  VAL_MASK(x, 1, 0)
#define  V_INT_STATUS_FBCD1(x)                  VAL_MASK(x, 1, 1)
#define  V_INT_STATUS_FBCD2(x)                  VAL_MASK(x, 1, 2)
#define  V_INT_STATUS_FBCD3(x)                  VAL_MASK(x, 1, 3)
#define  V_INT_STATUS_AFBCD0_HREG_DEC_RESP(x)   VAL_MASK(x, 1, 4)
#define  V_INT_STATUS_AFBCD0_HREG_AXI_RRESP(x)  VAL_MASK(x, 1, 5)
#define  V_INT_STATUS_AFBCD1_HREG_DEC_RESP(x)   VAL_MASK(x, 1, 6)
#define  V_INT_STATUS_AFBCD1_HREG_AXI_RRESP(x)  VAL_MASK(x, 1, 7)
#define  V_INT_STATUS_AFBCD2_HREG_DEC_RESP(x)   VAL_MASK(x, 1, 8)
#define  V_INT_STATUS_AFBCD2_HREG_AXI_RRESP(x)  VAL_MASK(x, 1, 9)
#define  V_INT_STATUS_AFBCD3_HREG_DEC_RESP(x)   VAL_MASK(x, 1, 10)
#define  V_INT_STATUS_AFBCD4_HREG_DEC_RESP(x)   VAL_MASK(x, 1, 11)
#define INTR_RAW_STATUS1                0x000000fc
#define  V_INT_RAW_STATUS_FBCD0(x)              VAL_MASK(x, 1, 0)
#define  V_INT_RAW_STATUS_FBCD1(x)              VAL_MASK(x, 1, 1)
#define  V_INT_RAW_STATUS_FBCD2(x)              VAL_MASK(x, 1, 2)
#define  V_INT_RAW_STATUS_FBCD3(x)              VAL_MASK(x, 1, 3)
#define  V_INT_RAW_STATUS_AFBCD0_HREG_DEC_RESP(x)       VAL_MASK(x, 1, 4)
#define  V_INT_RAW_STATUS_AFBCD0_HREG_AXI_RRESP(x)      VAL_MASK(x, 1, 5)
#define  V_INT_RAW_STATUS_AFBCD1_HREG_DEC_RESP(x)       VAL_MASK(x, 1, 6)
#define  V_INT_RAW_STATUS_AFBCD1_HREG_AXI_RRESP(x)      VAL_MASK(x, 1, 7)
#define  V_INT_RAW_STATUS_AFBCD2_HREG_DEC_RESP(x)       VAL_MASK(x, 1, 8)
#define  V_INT_RAW_STATUS_AFBCD2_HREG_AXI_RRESP(x)      VAL_MASK(x, 1, 9)
#define  V_INT_RAW_STATUS_AFBCD3_HREG_DEC_RESP(x)       VAL_MASK(x, 1, 10)
#define  V_INT_RAW_STATUS_AFBCD3_HREG_AXI_RRESP(x)      VAL_MASK(x, 1, 11)
#define WIN0_CTRL0                      0x00000100
#define  V_WIN0_EN(x)                           VAL_MASK(x, 1, 0)
#define  V_WIN0_DATA_FMT(x)                     VAL_MASK(x, 3, 1)
#define  V_WIN0_FMT_10(x)                       VAL_MASK(x, 1, 4)
#define  V_WIN0_LB_MODE(x)                      VAL_MASK(x, 3, 5)
#define  V_WIN0_INTERLACE_READ(x)               VAL_MASK(x, 1, 8)
#define  V_WIN0_NO_OUTSTANDING(x)               VAL_MASK(x, 1, 9)
#define  V_WIN0_CSC_MODE(x)                     VAL_MASK(x, 2, 10)
#define  V_WIN0_RB_SWAP(x)                      VAL_MASK(x, 1, 12)
#define  V_WIN0_ALPHA_SWAP(x)                   VAL_MASK(x, 1, 13)
#define  V_WIN0_MID_SWAP(x)                     VAL_MASK(x, 1, 14)
#define  V_WIN0_UV_SWAP(x)                      VAL_MASK(x, 1, 15)
#define  V_WIN0_HW_PRE_MUL_EN(x)                VAL_MASK(x, 1, 16)
#define  V_WIN0_YRGB_DEFLICK(x)                 VAL_MASK(x, 1, 18)
#define  V_WIN0_CBR_DEFLICK(x)                  VAL_MASK(x, 1, 19)
#define  V_WIN0_YUV_CLIP(x)                     VAL_MASK(x, 1, 20)
#define  V_WIN0_X_MIR_EN(x)                     VAL_MASK(x, 1, 21)
#define  V_WIN0_Y_MIR_EN(x)                     VAL_MASK(x, 1, 22)
#define  V_WIN0_AXI_MAX_OUTSTANDING_EN(x)       VAL_MASK(x, 1, 24)
#define  V_WIN0_AXI_OUTSTANDING_MAX_NUM(x)      VAL_MASK(x, 5, 25)
#define  V_WIN0_DMA_BURST_LENGTH(x)             VAL_MASK(x, 2, 30)
#define WIN0_CTRL1                      0x00000104
#define  V_WIN0_YRGB_AXI_GATHER_EN(x)           VAL_MASK(x, 1, 0)
#define  V_WIN0_CBR_AXI_GATHER_EN(x)            VAL_MASK(x, 1, 1)
#define  V_WIN0_BIC_COE_SEL(x)                  VAL_MASK(x, 2, 2)
#define  V_WIN0_VSD_YRGB_GT4(x)                 VAL_MASK(x, 1, 4)
#define  V_WIN0_VSD_YRGB_GT2(x)                 VAL_MASK(x, 1, 5)
#define  V_WIN0_VSD_CBR_GT4(x)                  VAL_MASK(x, 1, 6)
#define  V_WIN0_VSD_CBR_GT2(x)                  VAL_MASK(x, 1, 7)
#define  V_WIN0_YRGB_AXI_GATHER_NUM(x)          VAL_MASK(x, 4, 8)
#define  V_WIN0_CBR_AXI_GATHER_NUM(x)           VAL_MASK(x, 3, 12)
#define  V_WIN0_LINE_LOAD_MODE(x)               VAL_MASK(x, 1, 15)
#define  V_WIN0_YRGB_HOR_SCL_MODE(x)            VAL_MASK(x, 2, 16)
#define  V_WIN0_YRGB_VER_SCL_MODE(x)            VAL_MASK(x, 2, 18)
#define  V_WIN0_YRGB_HSD_MODE(x)                VAL_MASK(x, 2, 20)
#define  V_WIN0_YRGB_VSU_MODE(x)                VAL_MASK(x, 1, 22)
#define  V_WIN0_YRGB_VSD_MODE(x)                VAL_MASK(x, 1, 23)
#define  V_WIN0_CBR_HOR_SCL_MODE(x)             VAL_MASK(x, 2, 24)
#define  V_WIN0_CBR_VER_SCL_MODE(x)             VAL_MASK(x, 2, 26)
#define  V_WIN0_CBR_HSD_MODE(x)                 VAL_MASK(x, 2, 28)
#define  V_WIN0_CBR_VSU_MODE(x)                 VAL_MASK(x, 1, 30)
#define  V_WIN0_CBR_VSD_MODE(x)                 VAL_MASK(x, 1, 31)
#define WIN0_COLOR_KEY                  0x00000108
#define  V_WIN0_KEY_COLOR(x)                    VAL_MASK(x, 24, 0)
#define  V_WIN0_KEY_EN(x)                       VAL_MASK(x, 1, 31)
#define WIN0_VIR                        0x0000010c
#define  V_WIN0_VIR_STRIDE(x)                   VAL_MASK(x, 16, 0)
#define  V_WIN0_VIR_STRIDE_UV(x)                VAL_MASK(x, 16, 16)
#define WIN0_YRGB_MST                   0x00000110
#define  V_WIN0_YRGB_MST(x)                     VAL_MASK(x, 32, 0)
#define WIN0_CBR_MST                    0x00000114
#define  V_WIN0_CBR_MST(x)                      VAL_MASK(x, 32, 0)
#define WIN0_ACT_INFO                   0x00000118
#define  V_WIN0_ACT_WIDTH(x)                    VAL_MASK(x, 13, 0)
#define  V_WIN0_ACT_HEIGHT(x)                   VAL_MASK(x, 13, 16)
#define WIN0_DSP_INFO                   0x0000011c
#define  V_WIN0_DSP_WIDTH(x)                    VAL_MASK(x, 12, 0)
#define  V_WIN0_DSP_HEIGHT(x)                   VAL_MASK(x, 12, 16)
#define WIN0_DSP_ST                     0x00000120
#define  V_WIN0_DSP_XST(x)                      VAL_MASK(x, 13, 0)
#define  V_WIN0_DSP_YST(x)                      VAL_MASK(x, 13, 16)
#define WIN0_SCL_FACTOR_YRGB            0x00000124
#define  V_WIN0_HS_FACTOR_YRGB(x)               VAL_MASK(x, 16, 0)
#define  V_WIN0_VS_FACTOR_YRGB(x)               VAL_MASK(x, 16, 16)
#define WIN0_SCL_FACTOR_CBR             0x00000128
#define  V_WIN0_HS_FACTOR_CBR(x)                VAL_MASK(x, 16, 0)
#define  V_WIN0_VS_FACTOR_CBR(x)                VAL_MASK(x, 16, 16)
#define WIN0_SCL_OFFSET                 0x0000012c
#define  V_WIN0_HS_OFFSET_YRGB(x)               VAL_MASK(x, 8, 0)
#define  V_WIN0_HS_OFFSET_CBR(x)                VAL_MASK(x, 8, 8)
#define  V_WIN0_VS_OFFSET_YRGB(x)               VAL_MASK(x, 8, 16)
#define  V_WIN0_VS_OFFSET_CBR(x)                VAL_MASK(x, 8, 24)
#define WIN0_SRC_ALPHA_CTRL             0x00000130
#define  V_WIN0_SRC_ALPHA_EN(x)                 VAL_MASK(x, 1, 0)
#define  V_WIN0_SRC_COLOR_MODE(x)               VAL_MASK(x, 1, 1)
#define  V_WIN0_SRC_ALPHA_MODE(x)               VAL_MASK(x, 1, 2)
#define  V_WIN0_SRC_BLEND_MODE(x)               VAL_MASK(x, 2, 3)
#define  V_WIN0_SRC_ALPHA_CAL_MODE(x)           VAL_MASK(x, 1, 5)
#define  V_WIN0_SRC_FACTOR_MODE(x)              VAL_MASK(x, 3, 6)
#define  V_WIN0_SRC_GLOBAL_ALPHA(x)             VAL_MASK(x, 8, 16)
#define  V_WIN0_FADING_VALUE(x)                 VAL_MASK(x, 8, 24)
#define WIN0_DST_ALPHA_CTRL             0x00000134
#define  V_WIN0_DST_M0_RESERVED(x)              VAL_MASK(x, 6, 0)
#define  V_WIN0_DST_FACTOR_MODE(x)              VAL_MASK(x, 3, 6)
#define WIN0_FADING_CTRL                0x00000138
#define  V_LAYER0_FADING_OFFSET_R(x)            VAL_MASK(x, 8, 0)
#define  V_LAYER0_FADING_OFFSET_G(x)            VAL_MASK(x, 8, 8)
#define  V_LAYER0_FADING_OFFSET_B(x)            VAL_MASK(x, 8, 16)
#define  V_LAYER0_FADING_EN(x)                  VAL_MASK(x, 1, 24)
#define WIN0_CTRL2                      0x0000013c
#define  V_WIN_RID_WIN0_YRGB(x)                 VAL_MASK(x, 4, 0)
#define  V_WIN_RID_WIN0_CBR(x)                  VAL_MASK(x, 4, 4)
#define DBG_WIN0_REG0                   0x000001f0
#define DBG_WIN0_REG1                   0x000001f4
#define DBG_WIN0_REG2                   0x000001f8
#define  V_DBG_WIN0_YRGB_CMD_LINE_CNT(x)        VAL_MASK(x, 13, 16)
#define DBG_WIN0_RESERVED               0x000001fc
#define  V_RESERVED(x)                          VAL_MASK(x, 1, 0)
#define WIN1_CTRL0                      0x00000200
#define  V_WIN1_EN(x)                           VAL_MASK(x, 1, 0)
#define  V_WIN1_DATA_FMT(x)                     VAL_MASK(x, 3, 1)
#define  V_WIN1_FMT_10(x)                       VAL_MASK(x, 1, 4)
#define  V_WIN1_LB_MODE(x)                      VAL_MASK(x, 3, 5)
#define  V_WIN1_INTERLACE_READ(x)               VAL_MASK(x, 1, 8)
#define  V_WIN1_NO_OUTSTANDING(x)               VAL_MASK(x, 1, 9)
#define  V_WIN1_CSC_MODE(x)                     VAL_MASK(x, 2, 10)
#define  V_WIN1_RB_SWAP(x)                      VAL_MASK(x, 1, 12)
#define  V_WIN1_ALPHA_SWAP(x)                   VAL_MASK(x, 1, 13)
#define  V_WIN1_MID_SWAP(x)                     VAL_MASK(x, 1, 14)
#define  V_WIN1_UV_SWAP(x)                      VAL_MASK(x, 1, 15)
#define  V_WIN1_HW_PRE_MUL_EN(x)                VAL_MASK(x, 1, 16)
#define  V_WIN1_YRGB_DEFLICK(x)                 VAL_MASK(x, 1, 18)
#define  V_WIN1_CBR_DEFLICK(x)                  VAL_MASK(x, 1, 19)
#define  V_WIN1_YUV_CLIP(x)                     VAL_MASK(x, 1, 20)
#define  V_WIN1_X_MIR_EN(x)                     VAL_MASK(x, 1, 21)
#define  V_WIN1_Y_MIR_EN(x)                     VAL_MASK(x, 1, 22)
#define  V_WIN1_AXI_MAX_OUTSTANDING_EN(x)       VAL_MASK(x, 1, 24)
#define  V_WIN1_AXI_MAX_OUTSTANDING_NUM(x)      VAL_MASK(x, 5, 25)
#define  V_WIN1_DMA_BURST_LENGTH(x)             VAL_MASK(x, 2, 30)
#define WIN1_CTRL1                      0x00000204
#define  V_WIN1_YRGB_AXI_GATHER_EN(x)           VAL_MASK(x, 1, 0)
#define  V_WIN1_CBR_AXI_GATHER_EN(x)            VAL_MASK(x, 1, 1)
#define  V_WIN1_BIC_COE_SEL(x)                  VAL_MASK(x, 2, 2)
#define  V_WIN1_VSD_YRGB_GT4(x)                 VAL_MASK(x, 1, 4)
#define  V_WIN1_VSD_YRGB_GT2(x)                 VAL_MASK(x, 1, 5)
#define  V_WIN1_VSD_CBR_GT4(x)                  VAL_MASK(x, 1, 6)
#define  V_WIN1_VSD_CBR_GT2(x)                  VAL_MASK(x, 1, 7)
#define  V_WIN1_YRGB_AXI_GATHER_NUM(x)          VAL_MASK(x, 4, 8)
#define  V_WIN1_CBR_AXI_GATHER_NUM(x)           VAL_MASK(x, 3, 12)
#define  V_WIN1_LINE_LOAD_MODE(x)               VAL_MASK(x, 1, 15)
#define  V_WIN1_YRGB_HOR_SCL_MODE(x)            VAL_MASK(x, 2, 16)
#define  V_WIN1_YRGB_VER_SCL_MODE(x)            VAL_MASK(x, 2, 18)
#define  V_WIN1_YRGB_HSD_MODE(x)                VAL_MASK(x, 2, 20)
#define  V_WIN1_YRGB_VSU_MODE(x)                VAL_MASK(x, 1, 22)
#define  V_WIN1_YRGB_VSD_MODE(x)                VAL_MASK(x, 1, 23)
#define  V_WIN1_CBR_HOR_SCL_MODE(x)             VAL_MASK(x, 2, 24)
#define  V_WIN1_CBR_VER_SCL_MODE(x)             VAL_MASK(x, 2, 26)
#define  V_WIN1_CBR_HSD_MODE(x)                 VAL_MASK(x, 2, 28)
#define  V_WIN1_CBR_VSU_MODE(x)                 VAL_MASK(x, 1, 30)
#define  V_WIN1_CBR_VSD_MODE(x)                 VAL_MASK(x, 1, 31)
#define WIN1_COLOR_KEY                  0x00000208
#define  V_WIN1_KEY_COLOR(x)                    VAL_MASK(x, 24, 0)
#define  V_WIN1_KEY_EN(x)                       VAL_MASK(x, 1, 31)
#define WIN1_VIR                        0x0000020c
#define  V_WIN1_VIR_STRIDE(x)                   VAL_MASK(x, 16, 0)
#define  V_WIN1_VIR_STRIDE_UV(x)                VAL_MASK(x, 16, 16)
#define WIN1_YRGB_MST                   0x00000210
#define  V_WIN1_YRGB_MST(x)                     VAL_MASK(x, 32, 0)
#define WIN1_CBR_MST                    0x00000214
#define  V_WIN1_CBR_MST(x)                      VAL_MASK(x, 32, 0)
#define WIN1_ACT_INFO                   0x00000218
#define  V_WIN1_ACT_WIDTH(x)                    VAL_MASK(x, 13, 0)
#define  V_WIN1_ACT_HEIGHT(x)                   VAL_MASK(x, 13, 16)
#define WIN1_DSP_INFO                   0x0000021c
#define  V_WIN1_DSP_WIDTH(x)                    VAL_MASK(x, 12, 0)
#define  V_WIN1_DSP_HEIGHT(x)                   VAL_MASK(x, 12, 16)
#define WIN1_DSP_ST                     0x00000220
#define  V_WIN1_DSP_XST(x)                      VAL_MASK(x, 13, 0)
#define  V_WIN1_DSP_YST(x)                      VAL_MASK(x, 13, 16)
#define WIN1_SCL_FACTOR_YRGB            0x00000224
#define  V_WIN1_HS_FACTOR_YRGB(x)               VAL_MASK(x, 16, 0)
#define  V_WIN1_VS_FACTOR_YRGB(x)               VAL_MASK(x, 16, 16)
#define WIN1_SCL_FACTOR_CBR             0x00000228
#define  V_WIN1_HS_FACTOR_CBR(x)                VAL_MASK(x, 16, 0)
#define  V_WIN1_VS_FACTOR_CBR(x)                VAL_MASK(x, 16, 16)
#define WIN1_SCL_OFFSET                 0x0000022c
#define  V_WIN1_HS_OFFSET_YRGB(x)               VAL_MASK(x, 8, 0)
#define  V_WIN1_HS_OFFSET_CBR(x)                VAL_MASK(x, 8, 8)
#define  V_WIN1_VS_OFFSET_YRGB(x)               VAL_MASK(x, 8, 16)
#define  V_WIN1_VS_OFFSET_CBR(x)                VAL_MASK(x, 8, 24)
#define WIN1_SRC_ALPHA_CTRL             0x00000230
#define  V_WIN1_SRC_ALPHA_EN(x)                 VAL_MASK(x, 1, 0)
#define  V_WIN1_SRC_COLOR_MODE(x)               VAL_MASK(x, 1, 1)
#define  V_WIN1_SRC_ALPHA_MODE(x)               VAL_MASK(x, 1, 2)
#define  V_WIN1_SRC_BLEND_MODE(x)               VAL_MASK(x, 2, 3)
#define  V_WIN1_SRC_ALPHA_CAL_MODE(x)           VAL_MASK(x, 1, 5)
#define  V_WIN1_SRC_FACTOR_MODE(x)              VAL_MASK(x, 3, 6)
#define  V_WIN1_SRC_GLOBAL_ALPHA(x)             VAL_MASK(x, 8, 16)
#define  V_WIN1_FADING_VALUE(x)                 VAL_MASK(x, 8, 24)
#define WIN1_DST_ALPHA_CTRL             0x00000234
#define  V_WIN1_DSP_M0_RESERVED(x)              VAL_MASK(x, 6, 0)
#define  V_WIN1_DST_FACTOR_M0(x)                VAL_MASK(x, 3, 6)
#define WIN1_FADING_CTRL                0x00000238
#define  V_WIN1_FADING_OFFSET_R(x)              VAL_MASK(x, 8, 0)
#define  V_WIN1_FADING_OFFSET_G(x)              VAL_MASK(x, 8, 8)
#define  V_WIN1_FADING_OFFSET_B(x)              VAL_MASK(x, 8, 16)
#define  V_WIN1_FADING_EN(x)                    VAL_MASK(x, 1, 24)
#define WIN1_CTRL2                      0x0000023c
#define  V_WIN_RID_WIN1_YRGB(x)                 VAL_MASK(x, 4, 0)
#define  V_WIN_RID_WIN1_CBR(x)                  VAL_MASK(x, 4, 4)
#define DBG_WIN1_REG0                   0x000002f0
#define DBG_WIN1_REG1                   0x000002f4
#define DBG_WIN1_REG2                   0x000002f8
#define DBG_WIN1_RESERVED               0x000002fc
#define  V_RESERVED(x)                          VAL_MASK(x, 1, 0)
#define WIN2_CTRL0                      0x00000300
#define  V_WIN2_EN(x)                           VAL_MASK(x, 1, 0)
#define  V_WIN2_DATA_FMT(x)                     VAL_MASK(x, 3, 1)
#define  V_WIN2_FMT_10(x)                       VAL_MASK(x, 1, 4)
#define  V_WIN2_LB_MODE(x)                      VAL_MASK(x, 3, 5)
#define  V_WIN2_INTERLACE_READ(x)               VAL_MASK(x, 1, 8)
#define  V_WIN2_NO_OUTSTANDING(x)               VAL_MASK(x, 1, 9)
#define  V_WIN2_CSC_MODE(x)                     VAL_MASK(x, 2, 10)
#define  V_WIN2_RB_SWAP(x)                      VAL_MASK(x, 1, 12)
#define  V_WIN2_ALPHA_SWAP(x)                   VAL_MASK(x, 1, 13)
#define  V_WIN2_MID_SWAP(x)                     VAL_MASK(x, 1, 14)
#define  V_WIN2_UV_SWAP(x)                      VAL_MASK(x, 1, 15)
#define  V_WIN2_HW_PRE_MUL_EN(x)                VAL_MASK(x, 1, 16)
#define  V_WIN2_YRGB_DEFLICK(x)                 VAL_MASK(x, 1, 18)
#define  V_WIN2_CBR_DEFLICK(x)                  VAL_MASK(x, 1, 19)
#define  V_WIN2_YUV_CLIP(x)                     VAL_MASK(x, 1, 20)
#define  V_WIN2_X_MIR_EN(x)                     VAL_MASK(x, 1, 21)
#define  V_WIN2_Y_MIR_EN(x)                     VAL_MASK(x, 1, 22)
#define  V_WIN2_AXI_MAX_OUTSTANDING_EN(x)       VAL_MASK(x, 1, 24)
#define  V_WIN2_AXI_MAX_OUTSTANDING_NUM(x)      VAL_MASK(x, 5, 25)
#define  V_WIN2_DMA_BURST_LENGTH(x)             VAL_MASK(x, 2, 30)
#define WIN2_CTRL1                      0x00000304
#define  V_WIN2_YRGB_AXI_GATHER_EN(x)           VAL_MASK(x, 1, 0)
#define  V_WIN2_CBR_AXI_GATHER_EN(x)            VAL_MASK(x, 1, 1)
#define  V_WIN2_BIC_COE_SEL(x)                  VAL_MASK(x, 2, 2)
#define  V_WIN2_VSD_YRGB_GT4(x)                 VAL_MASK(x, 1, 4)
#define  V_WIN2_VSD_YRGB_GT2(x)                 VAL_MASK(x, 1, 5)
#define  V_WIN2_VSD_CBR_GT4(x)                  VAL_MASK(x, 1, 6)
#define  V_WIN2_VSD_CBR_GT2(x)                  VAL_MASK(x, 1, 7)
#define  V_WIN2_YRGB_AXI_GATHER_NUM(x)          VAL_MASK(x, 4, 8)
#define  V_WIN2_CBR_AXI_GATHER_NUM(x)           VAL_MASK(x, 3, 12)
#define  V_WIN2_LINE_LOAD_MODE(x)               VAL_MASK(x, 1, 15)
#define  V_WIN2_YRGB_HOR_SCL_MODE(x)            VAL_MASK(x, 2, 16)
#define  V_WIN2_YRGB_VER_SCL_MODE(x)            VAL_MASK(x, 2, 18)
#define  V_WIN2_YRGB_HSD_MODE(x)                VAL_MASK(x, 2, 20)
#define  V_WIN2_YRGB_VSU_MODE(x)                VAL_MASK(x, 1, 22)
#define  V_WIN2_YRGB_VSD_MODE(x)                VAL_MASK(x, 1, 23)
#define  V_WIN2_CBR_HOR_SCL_MODE(x)             VAL_MASK(x, 2, 24)
#define  V_WIN2_CBR_VER_SCL_MODE(x)             VAL_MASK(x, 2, 26)
#define  V_WIN2_CBR_HSD_MODE(x)                 VAL_MASK(x, 2, 28)
#define  V_WIN2_CBR_VSU_MODE(x)                 VAL_MASK(x, 1, 30)
#define  V_WIN2_CBR_VSD_MODE(x)                 VAL_MASK(x, 1, 31)
#define WIN2_COLOR_KEY                  0x00000308
#define  V_WIN2_KEY_COLOR(x)                    VAL_MASK(x, 24, 0)
#define  V_WIN2_KEY_EN(x)                       VAL_MASK(x, 1, 31)
#define WIN2_VIR                        0x0000030c
#define  V_WIN2_VIR_STRIDE(x)                   VAL_MASK(x, 16, 0)
#define  V_WIN2_VIR_STRIDE_UV(x)                VAL_MASK(x, 16, 16)
#define WIN2_YRGB_MST                   0x00000310
#define  V_WIN2_YRGB_MST(x)                     VAL_MASK(x, 32, 0)
#define WIN2_CBR_MST                    0x00000314
#define  V_WIN2_CBR_MST(x)                      VAL_MASK(x, 32, 0)
#define WIN2_ACT_INFO                   0x00000318
#define  V_WIN2_ACT_WIDTH(x)                    VAL_MASK(x, 13, 0)
#define  V_WIN2_ACT_HEIGHT(x)                   VAL_MASK(x, 13, 16)
#define WIN2_DSP_INFO                   0x0000031c
#define  V_WIN2_DSP_WIDTH(x)                    VAL_MASK(x, 12, 0)
#define  V_WIN2_DSP_HEIGHT(x)                   VAL_MASK(x, 12, 16)
#define WIN2_DSP_ST                     0x00000320
#define  V_WIN2_DSP_XST(x)                      VAL_MASK(x, 13, 0)
#define  V_WIN2_DSP_YST(x)                      VAL_MASK(x, 13, 16)
#define WIN2_SCL_FACTOR_YRGB            0x00000324
#define  V_WIN2_HS_FACTOR_YRGB(x)               VAL_MASK(x, 16, 0)
#define  V_WIN2_VS_FACTOR_YRGB(x)               VAL_MASK(x, 16, 16)
#define WIN2_SCL_FACTOR_CBR             0x00000328
#define  V_WIN2_HS_FACTOR_CBR(x)                VAL_MASK(x, 16, 0)
#define  V_WIN2_VS_FACTOR_CBR(x)                VAL_MASK(x, 16, 16)
#define WIN2_SCL_OFFSET                 0x0000032c
#define  V_WIN2_HS_OFFSET_YRGB(x)               VAL_MASK(x, 8, 0)
#define  V_WIN2_HS_OFFSET_CBR(x)                VAL_MASK(x, 8, 8)
#define  V_WIN2_VS_OFFSET_YRGB(x)               VAL_MASK(x, 8, 16)
#define  V_WIN2_VS_OFFSET_CBR(x)                VAL_MASK(x, 8, 24)
#define WIN2_SRC_ALPHA_CTRL             0x00000330
#define  V_WIN2_SRC_ALPHA_EN(x)                 VAL_MASK(x, 1, 0)
#define  V_WIN2_SRC_COLOR_MODE(x)               VAL_MASK(x, 1, 1)
#define  V_WIN2_SRC_ALPHA_MODE(x)               VAL_MASK(x, 1, 2)
#define  V_WIN2_SRC_BLEND_MODE(x)               VAL_MASK(x, 2, 3)
#define  V_WIN2_SRC_ALPHA_CAL_MODE(x)           VAL_MASK(x, 1, 5)
#define  V_WIN2_SRC_FACTOR_MODE(x)              VAL_MASK(x, 3, 6)
#define  V_WIN2_SRC_GLOBAL_ALPHA(x)             VAL_MASK(x, 8, 16)
#define  V_WIN2_FADING_VALUE(x)                 VAL_MASK(x, 8, 24)
#define WIN2_DST_ALPHA_CTRL             0x00000334
#define  V_WIN2_DSP_M0_RESERVED(x)              VAL_MASK(x, 6, 0)
#define  V_WIN2_DST_FACTOR_M0(x)                VAL_MASK(x, 3, 6)
#define WIN2_FADING_CTRL                0x00000338
#define  V_WIN2_FADING_OFFSET_R(x)              VAL_MASK(x, 8, 0)
#define  V_WIN2_FADING_OFFSET_G(x)              VAL_MASK(x, 8, 8)
#define  V_WIN2_FADING_OFFSET_B(x)              VAL_MASK(x, 8, 16)
#define  V_WIN2_FADING_EN(x)                    VAL_MASK(x, 1, 24)
#define WIN2_CTRL2                      0x0000033c
#define  V_WIN_RID_WIN2_YRGB(x)                 VAL_MASK(x, 4, 0)
#define  V_WIN_RID_WIN2_CBR(x)                  VAL_MASK(x, 4, 4)
#define DBG_WIN2_REG0                   0x000003f0
#define DBG_WIN2_REG1                   0x000003f4
#define DBG_WIN2_REG2                   0x000003f8
#define DBG_WIN2_RESERVED               0x000003fc
#define  V_RESERVED(x)                          VAL_MASK(x, 1, 0)
#define WIN3_CTRL0                      0x00000400
#define  V_WIN3_EN(x)                           VAL_MASK(x, 1, 0)
#define  V_WIN3_DATA_FMT(x)                     VAL_MASK(x, 3, 1)
#define  V_WIN3_FMT_10(x)                       VAL_MASK(x, 1, 4)
#define  V_WIN3_LB_MODE(x)                      VAL_MASK(x, 3, 5)
#define  V_WIN3_INTERLACE_READ(x)               VAL_MASK(x, 1, 8)
#define  V_WIN3_NO_OUTSTANDING(x)               VAL_MASK(x, 1, 9)
#define  V_WIN3_CSC_MODE(x)                     VAL_MASK(x, 2, 10)
#define  V_WIN3_RB_SWAP(x)                      VAL_MASK(x, 1, 12)
#define  V_WIN3_ALPHA_SWAP(x)                   VAL_MASK(x, 1, 13)
#define  V_WIN3_MID_SWAP(x)                     VAL_MASK(x, 1, 14)
#define  V_WIN3_UV_SWAP(x)                      VAL_MASK(x, 1, 15)
#define  V_WIN3_HW_PRE_MUL_EN(x)                VAL_MASK(x, 1, 16)
#define  V_WIN3_YRGB_DEFLICK(x)                 VAL_MASK(x, 1, 18)
#define  V_WIN3_CBR_DEFLICK(x)                  VAL_MASK(x, 1, 19)
#define  V_WIN3_YUV_CLIP(x)                     VAL_MASK(x, 1, 20)
#define  V_WIN3_X_MIR_EN(x)                     VAL_MASK(x, 1, 21)
#define  V_WIN3_Y_MIR_EN(x)                     VAL_MASK(x, 1, 22)
#define  V_WIN3_AXI_MAX_OUTSTANDING_EN(x)       VAL_MASK(x, 1, 24)
#define  V_WIN3_AXI_MAX_OUTSTANDING_NUM(x)      VAL_MASK(x, 5, 25)
#define  V_WIN3_DMA_BURST_LENGTH(x)             VAL_MASK(x, 2, 30)
#define WIN3_CTRL1                      0x00000404
#define  V_WIN3_YRGB_AXI_GATHER_EN(x)           VAL_MASK(x, 1, 0)
#define  V_WIN3_CBR_AXI_GATHER_EN(x)            VAL_MASK(x, 1, 1)
#define  V_WIN3_BIC_COE_SEL(x)                  VAL_MASK(x, 2, 2)
#define  V_WIN3_VSD_YRGB_GT4(x)                 VAL_MASK(x, 1, 4)
#define  V_WIN3_VSD_YRGB_GT2(x)                 VAL_MASK(x, 1, 5)
#define  V_WIN3_VSD_CBR_GT4(x)                  VAL_MASK(x, 1, 6)
#define  V_WIN3_VSD_CBR_GT2(x)                  VAL_MASK(x, 1, 7)
#define  V_WIN3_YRGB_AXI_GATHER_NUM(x)          VAL_MASK(x, 4, 8)
#define  V_WIN3_CBR_AXI_GATHER_NUM(x)           VAL_MASK(x, 3, 12)
#define  V_WIN3_LINE_LOAD_MODE(x)               VAL_MASK(x, 1, 15)
#define  V_WIN3_YRGB_HOR_SCL_MODE(x)            VAL_MASK(x, 2, 16)
#define  V_WIN3_YRGB_VER_SCL_MODE(x)            VAL_MASK(x, 2, 18)
#define  V_WIN3_YRGB_HSD_MODE(x)                VAL_MASK(x, 2, 20)
#define  V_WIN3_YRGB_VSU_MODE(x)                VAL_MASK(x, 1, 22)
#define  V_WIN3_YRGB_VSD_MODE(x)                VAL_MASK(x, 1, 23)
#define  V_WIN3_CBR_HOR_SCL_MODE(x)             VAL_MASK(x, 2, 24)
#define  V_WIN3_CBR_VER_SCL_MODE(x)             VAL_MASK(x, 2, 26)
#define  V_WIN3_CBR_HSD_MODE(x)                 VAL_MASK(x, 2, 28)
#define  V_WIN3_CBR_VSU_MODE(x)                 VAL_MASK(x, 1, 30)
#define  V_WIN3_CBR_VSD_MODE(x)                 VAL_MASK(x, 1, 31)
#define WIN3_COLOR_KEY                  0x00000408
#define  V_WIN3_KEY_COLOR(x)                    VAL_MASK(x, 24, 0)
#define  V_WIN3_KEY_EN(x)                       VAL_MASK(x, 1, 31)
#define WIN3_VIR                        0x0000040c
#define  V_WIN3_VIR_STRIDE(x)                   VAL_MASK(x, 16, 0)
#define  V_WIN3_VIR_STRIDE_UV(x)                VAL_MASK(x, 16, 16)
#define WIN3_YRGB_MST                   0x00000410
#define  V_WIN3_YRGB_MST(x)                     VAL_MASK(x, 32, 0)
#define WIN3_CBR_MST                    0x00000414
#define  V_WIN3_CBR_MST(x)                      VAL_MASK(x, 32, 0)
#define WIN3_ACT_INFO                   0x00000418
#define  V_WIN3_ACT_WIDTH(x)                    VAL_MASK(x, 13, 0)
#define  V_WIN3_ACT_HEIGHT(x)                   VAL_MASK(x, 13, 16)
#define WIN3_DSP_INFO                   0x0000041c
#define  V_WIN3_DSP_WIDTH(x)                    VAL_MASK(x, 12, 0)
#define  V_WIN3_DSP_HEIGHT(x)                   VAL_MASK(x, 12, 16)
#define WIN3_DSP_ST                     0x00000420
#define  V_WIN3_DSP_XST(x)                      VAL_MASK(x, 13, 0)
#define  V_WIN3_DSP_YST(x)                      VAL_MASK(x, 13, 16)
#define WIN3_SCL_FACTOR_YRGB            0x00000424
#define  V_WIN3_HS_FACTOR_YRGB(x)               VAL_MASK(x, 16, 0)
#define  V_WIN3_VS_FACTOR_YRGB(x)               VAL_MASK(x, 16, 16)
#define WIN3_SCL_FACTOR_CBR             0x00000428
#define  V_WIN3_HS_FACTOR_CBR(x)                VAL_MASK(x, 16, 0)
#define  V_WIN3_VS_FACTOR_CBR(x)                VAL_MASK(x, 16, 16)
#define WIN3_SCL_OFFSET                 0x0000042c
#define  V_WIN3_HS_OFFSET_YRGB(x)               VAL_MASK(x, 8, 0)
#define  V_WIN3_HS_OFFSET_CBR(x)                VAL_MASK(x, 8, 8)
#define  V_WIN3_VS_OFFSET_YRGB(x)               VAL_MASK(x, 8, 16)
#define  V_WIN3_VS_OFFSET_CBR(x)                VAL_MASK(x, 8, 24)
#define WIN3_SRC_ALPHA_CTRL             0x00000430
#define  V_WIN3_SRC_ALPHA_EN(x)                 VAL_MASK(x, 1, 0)
#define  V_WIN3_SRC_COLOR_MODE(x)               VAL_MASK(x, 1, 1)
#define  V_WIN3_SRC_ALPHA_MODE(x)               VAL_MASK(x, 1, 2)
#define  V_WIN3_SRC_BLEND_MODE(x)               VAL_MASK(x, 2, 3)
#define  V_WIN3_SRC_ALPHA_CAL_MODE(x)           VAL_MASK(x, 1, 5)
#define  V_WIN3_SRC_FACTOR_MODE(x)              VAL_MASK(x, 3, 6)
#define  V_WIN3_SRC_GLOBAL_ALPHA(x)             VAL_MASK(x, 8, 16)
#define  V_WIN3_FADING_VALUE(x)                 VAL_MASK(x, 8, 24)
#define WIN3_DST_ALPHA_CTRL             0x00000434
#define  V_WIN3_DSP_M0_RESERVED(x)              VAL_MASK(x, 6, 0)
#define  V_WIN3_DST_FACTOR_M0(x)                VAL_MASK(x, 3, 6)
#define WIN3_FADING_CTRL                0x00000438
#define  V_WIN3_FADING_OFFSET_R(x)              VAL_MASK(x, 8, 0)
#define  V_WIN3_FADING_OFFSET_G(x)              VAL_MASK(x, 8, 8)
#define  V_WIN3_FADING_OFFSET_B(x)              VAL_MASK(x, 8, 16)
#define  V_WIN3_FADING_EN(x)                    VAL_MASK(x, 1, 24)
#define WIN3_CTRL2                      0x0000043c
#define  V_WIN_RID_WIN3_YRGB(x)                 VAL_MASK(x, 4, 0)
#define  V_WIN_RID_WIN3_CBR(x)                  VAL_MASK(x, 4, 4)
#define DBG_WIN3_REG0                   0x000004f0
#define DBG_WIN3_REG1                   0x000004f4
#define DBG_WIN3_REG2                   0x000004f8
#define DBG_WIN3_RESERVED               0x000004fc
#define  V_RESERVED(x)                          VAL_MASK(x, 1, 0)

#define HWC_CTRL0                       0x00000500
#define  V_HWC_EN(x)                            VAL_MASK(x, 1, 0)
#define  V_HWC_DATA_FMT(x)                      VAL_MASK(x, 3, 1)
#define  V_HWC_MODE(x)                          VAL_MASK(x, 1, 4)
#define  V_HWC_SIZE(x)                          VAL_MASK(x, 2, 5)
#define  V_HWC_INTERLACE_READ(x)                VAL_MASK(x, 1, 8)
#define  V_HWC_CSC_MODE(x)                      VAL_MASK(x, 1, 10)
#define  V_HWC_RB_SWAP(x)                       VAL_MASK(x, 1, 12)
#define  V_HWC_ALPHA_SWAP(x)                    VAL_MASK(x, 1, 13)
#define  V_HWC_ENDIAN_SWAP(x)                   VAL_MASK(x, 1, 14)
#define HWC_CTRL1                       0x00000504
#define  V_HWC_AXI_GATHER_EN(x)                 VAL_MASK(x, 1, 0)
#define  V_HWC_AXI_MAX_OUTSTANDING_EN(x)        VAL_MASK(x, 1, 1)
#define  V_HWC_DMA_BURST_LENGTH(x)              VAL_MASK(x, 2, 2)
#define  V_HWC_AXI_GATHER_NUM(x)                VAL_MASK(x, 3, 4)
#define  V_HWC_AXI_MAX_OUTSTANDING_NUM(x)       VAL_MASK(x, 5, 8)
#define  V_HWC_RGB2YUV_EN(x)                    VAL_MASK(x, 1, 13)
#define  V_HWC_NO_OUTSTANDING(x)                VAL_MASK(x, 1, 14)
#define  V_HWC_Y_MIR_EN(x)                      VAL_MASK(x, 1, 15)
#define  V_HWC_LUT_EN(x)                        VAL_MASK(x, 1, 16)
#define  V_WIN_RID_HWC(x)                       VAL_MASK(x, 4, 20)
#define HWC_MST                         0x00000508
#define  V_HWC_MST(x)                           VAL_MASK(x, 32, 0)
#define HWC_DSP_ST                      0x0000050c
#define  V_HWC_DSP_XST(x)                       VAL_MASK(x, 13, 0)
#define  V_HWC_DSP_YST(x)                       VAL_MASK(x, 13, 16)
#define HWC_SRC_ALPHA_CTRL              0x00000510
#define  V_HWC_SRC_ALPHA_EN(x)                  VAL_MASK(x, 1, 0)
#define  V_HWC_SRC_COLOR_MODE(x)                VAL_MASK(x, 1, 1)
#define  V_HWC_SRC_ALPHA_MODE(x)                VAL_MASK(x, 1, 2)
#define  V_HWC_SRC_BLEND_MODE(x)                VAL_MASK(x, 2, 3)
#define  V_HWC_SRC_ALPHA_CAL_MODE(x)            VAL_MASK(x, 1, 5)
#define  V_HWC_SRC_FACTOR_MODE(x)               VAL_MASK(x, 3, 6)
#define  V_HWC_SRC_GLOBAL_ALPHA(x)              VAL_MASK(x, 8, 16)
#define  V_HWC_FADING_VALUE(x)                  VAL_MASK(x, 8, 24)
#define HWC_DST_ALPHA_CTRL              0x00000514
#define  V_HWC_DST_M0_RESERVED(x)               VAL_MASK(x, 6, 0)
#define  V_HWC_DST_FACTOR_MODE(x)               VAL_MASK(x, 3, 6)
#define HWC_FADING_CTRL                 0x00000518
#define  V_HWC_FADING_OFFSET_R(x)               VAL_MASK(x, 8, 0)
#define  V_HWC_FADING_OFFSET_G(x)               VAL_MASK(x, 8, 8)
#define  V_HWC_FADING_OFFSET_B(x)               VAL_MASK(x, 8, 16)
#define  V_HWC_FADING_EN(x)                     VAL_MASK(x, 1, 24)
#define HWC_RESERVED1                   0x0000051c
#define POST_DSP_HACT_INFO              0x00000600
#define  V_DSP_HACT_END_POST(x)                 VAL_MASK(x, 13, 0)
#define  V_DSP_HACT_ST_POST(x)                  VAL_MASK(x, 13, 16)
#define POST_DSP_VACT_INFO              0x00000604
#define  V_DSP_VACT_END_POST(x)                 VAL_MASK(x, 13, 0)
#define  V_DSP_VACT_ST_POST(x)                  VAL_MASK(x, 13, 16)
#define POST_SCL_FACTOR_YRGB            0x00000608
#define  V_POST_HS_FACTOR_YRGB(x)               VAL_MASK(x, 16, 0)
#define  V_POST_VS_FACTOR_YRGB(x)               VAL_MASK(x, 16, 16)
#define POST_RESERVED                   0x0000060c
#define POST_SCL_CTRL                   0x00000610
#define  V_POST_HOR_SD_EN(x)                    VAL_MASK(x, 1, 0)
#define  V_POST_VER_SD_EN(x)                    VAL_MASK(x, 1, 1)
#define  V_DSP_OUT_RGB_YUV(x)			VAL_MASK(x, 1, 2)
#define POST_DSP_VACT_INFO_F1           0x00000614
#define  V_DSP_VACT_END_POST(x)                 VAL_MASK(x, 13, 0)
#define  V_DSP_VACT_ST_POST(x)                  VAL_MASK(x, 13, 16)
#define DSP_HTOTAL_HS_END               0x00000618
#define  V_DSP_HS_END(x)                        VAL_MASK(x, 13, 0)
#define  V_DSP_HTOTAL(x)                        VAL_MASK(x, 13, 16)
#define DSP_HACT_ST_END                 0x0000061c
#define  V_DSP_HACT_END(x)                      VAL_MASK(x, 13, 0)
#define  V_DSP_HACT_ST(x)                       VAL_MASK(x, 13, 16)
#define DSP_VTOTAL_VS_END               0x00000620
#define  V_DSP_VS_END(x)                        VAL_MASK(x, 13, 0)
#define  V_SW_DSP_VTOTAL_IMD(x)                 VAL_MASK(x, 1, 15)
#define  V_DSP_VTOTAL(x)                        VAL_MASK(x, 13, 16)
#define DSP_VACT_ST_END                 0x00000624
#define  V_DSP_VACT_END(x)                      VAL_MASK(x, 13, 0)
#define  V_DSP_VACT_ST(x)                       VAL_MASK(x, 13, 16)
#define DSP_VS_ST_END_F1                0x00000628
#define  V_DSP_VS_END_F1(x)                     VAL_MASK(x, 13, 0)
#define  V_DSP_VS_ST_F1(x)                      VAL_MASK(x, 13, 16)
#define DSP_VACT_ST_END_F1              0x0000062c
#define  V_DSP_VACT_END_F1(x)                   VAL_MASK(x, 13, 0)
#define  V_DSP_VACT_ST_F1(x)                    VAL_MASK(x, 13, 16)
#define BCSH_COLOR_BAR                  0x00000640
#define  V_BCSH_EN(x)                           VAL_MASK(x, 1, 0)
#define  V_COLOR_BAR_Y(x)                       VAL_MASK(x, 8, 8)
#define  V_COLOR_BAR_U(x)                       VAL_MASK(x, 8, 16)
#define  V_COLOR_BAR_V(x)                       VAL_MASK(x, 8, 24)
#define BCSH_BCS                        0x00000644
#define  V_BRIGHTNESS(x)                        VAL_MASK(x, 8, 0)
#define  V_CONTRAST(x)                          VAL_MASK(x, 9, 8)
#define  V_SAT_CON(x)                           VAL_MASK(x, 10, 20)
#define  V_OUT_MODE(x)                          VAL_MASK(x, 2, 30)
#define BCSH_H                          0x00000648
#define  V_SIN_HUE(x)                           VAL_MASK(x, 9, 0)
#define  V_COS_HUE(x)                           VAL_MASK(x, 9, 16)
#define BCSH_CTRL                       0x0000064c
#define  V_BCSH_Y2R_EN(x)                       VAL_MASK(x, 1, 0)
#define  V_BCSH_Y2R_CSC_MODE(x)                 VAL_MASK(x, 2, 2)
#define  V_BCSH_R2Y_EN(x)                       VAL_MASK(x, 1, 4)
#define  V_BCSH_R2Y_CSC_MODE(x)                 VAL_MASK(x, 2, 6)
#define FRC_LOWER01_0                   0x00000678
#define  V_LOWER01_FRM0(x)                      VAL_MASK(x, 16, 0)
#define  V_LOWER01_FRM1(x)                      VAL_MASK(x, 16, 16)
#define FRC_LOWER01_1                   0x0000067c
#define  V_LOWER01_FRM2(x)                      VAL_MASK(x, 16, 0)
#define  V_LOWER01_FRM3(x)                      VAL_MASK(x, 16, 16)
#define FRC_LOWER10_0                   0x00000680
#define  V_LOWER10_FRM0(x)                      VAL_MASK(x, 16, 0)
#define  V_LOWER10_FRM1(x)                      VAL_MASK(x, 16, 16)
#define FRC_LOWER10_1                   0x00000684
#define  V_LOWER10_FRM2(x)                      VAL_MASK(x, 16, 0)
#define  V_LOWER10_FRM3(x)                      VAL_MASK(x, 16, 16)
#define FRC_LOWER11_0                   0x00000688
#define  V_LOWER11_FRM0(x)                      VAL_MASK(x, 16, 0)
#define  V_LOWER11_FRM1(x)                      VAL_MASK(x, 16, 16)
#define FRC_LOWER11_1                   0x0000068c
#define  V_LOWER11_FRM2(x)                      VAL_MASK(x, 16, 0)
#define  V_LOWER11_FRM3(x)                      VAL_MASK(x, 16, 16)
#define DBG_POST_REG0                   0x000006e8
#define DBG_POST_RESERVED               0x000006ec
#define  V_RESERVED(x)                          VAL_MASK(x, 1, 0)
#define DBG_DATAO                       0x000006f0
#define  V_SW_DATAO_SEL(x)                      VAL_MASK(x, 2, 30)
#define DBG_DATAO_2                     0x000006f4
#define  V_VOP_DATA_O_2(x)                      VAL_MASK(x, 30, 0)
#define  V_SW_DATAO_SEL_2(x)                    VAL_MASK(x, 2, 30)

/* sdr to hdr */
#define SDR2HDR_CTRL                    0x00000700
#define  V_BT1886EOTF_PRE_CONV_EN(x)            VAL_MASK(x, 1, 0)
#define  V_RGB2RGB_PRE_CONV_EN(x)               VAL_MASK(x, 1, 1)
#define  V_RGB2RGB_PRE_CONV_MODE(x)             VAL_MASK(x, 1, 2)
#define  V_ST2084OETF_PRE_CONV_EN(x)            VAL_MASK(x, 1, 3)
#define  V_BT1886EOTF_POST_CONV_EN(x)           VAL_MASK(x, 1, 4)
#define  V_RGB2RGB_POST_CONV_EN(x)              VAL_MASK(x, 1, 5)
#define  V_RGB2RGB_POST_CONV_MODE(x)            VAL_MASK(x, 1, 6)
#define  V_ST2084OETF_POST_CONV_EN(x)           VAL_MASK(x, 1, 7)
#define  V_WIN0_R2Y_EN(x)                       VAL_MASK(x, 1, 8)
#define  V_WIN0_Y2R_EN(x)                       VAL_MASK(x, 1, 9)
#define  V_WIN1_R2Y_EN(x)                       VAL_MASK(x, 1, 10)
#define  V_WIN1_Y2R_EN(x)                       VAL_MASK(x, 1, 11)
#define  V_WIN2_R2Y_EN(x)                       VAL_MASK(x, 1, 12)
#define  V_WIN2_Y2R_EN(x)                       VAL_MASK(x, 1, 13)
#define  V_WIN3_R2Y_EN(x)                       VAL_MASK(x, 1, 14)
#define  V_WIN3_Y2R_EN(x)                       VAL_MASK(x, 1, 15)
#define  V_HWC_R2Y_EN(x)                        VAL_MASK(x, 1, 16)
#define  V_HWC_Y2R_EN(x)                        VAL_MASK(x, 1, 17)
#define  V_WIN_CSC_MODE_SEL(x)                  VAL_MASK(x, 1, 31)
#define EOTF_OETF_Y0                    0x00000704
#define RESERVED0001                    0x00000708
#define RESERVED0002                    0x0000070c
#define EOTF_OETF_Y1                    0x00000710
#define EOTF_OETF_Y64                   0x0000080c
#define OETF_DX_DXPOW1                  0x00000810
#define OETF_DX_DXPOW64                 0x0000090c
#define OETF_XN1                        0x00000910
#define OETF_XN63                       0x00000a08

/* hdr to sdr */
#define HDR2SDR_CTRL                    0x00000a10
#define  V_HDR2SDR_EN(x)                        VAL_MASK(x, 1, 0)
#define HDR2SDR_SRC_RANGE               0x00000a14
#define  V_SRC_MIN(x)                           VAL_MASK(x, 14, 0)
#define  V_SRC_MAX(x)                           VAL_MASK(x, 14, 16)
#define HDR2SDR_NORMFACEETF             0x00000a18
#define  V_NORMFACEETF(x)                       VAL_MASK(x, 11, 0)
#define RESERVED0003                    0x00000a1c
#define  V_FIELD00003(x)                         VAL_MASK(x, 1, 0)
#define HDR2SDR_DST_RANGE               0x00000a20
#define  V_DST_MIN(x)                           VAL_MASK(x, 16, 0)
#define  V_DST_MAX(x)                           VAL_MASK(x, 16, 16)
#define HDR2SDR_NORMFACCGAMMA           0x00000a24
#define  V_NORMFACCGAMMA(x)                     VAL_MASK(x, 16, 0)
#define EETF_OETF_Y0                    0x00000a28
#define SAT_Y0                          0x00000a2c
#define EETF_OETF_Y1                    0x00000a30
#define SAT_Y1				0x00000ab0
#define SAT_Y8                          0x00000acc

#define HWC_LUT_ADDR                    0x00000c00
#define  V_HWC_LUT_ADDR(x)                      VAL_MASK(x, 32, 0)
#define MMU_DTE_ADDR                    0x00003f00
#define  V_MMU_DTE_ADDR(x)                      VAL_MASK(x, 32, 0)
#define MMU_STATUS                      0x00003f04
#define  V_PAGING_ENABLED(x)                    VAL_MASK(x, 1, 0)
#define  V_PAGE_FAULT_ACTIVE(x)                 VAL_MASK(x, 1, 1)
#define  V_STAIL_ACTIVE(x)                      VAL_MASK(x, 1, 2)
#define  V_MMU_IDLE(x)                          VAL_MASK(x, 1, 3)
#define  V_REPLAY_BUFFER_EMPTY(x)               VAL_MASK(x, 1, 4)
#define  V_PAGE_FAULT_IS_WRITE(x)               VAL_MASK(x, 1, 5)
#define  V_PAGE_FAULT_BUS_ID(x)                 VAL_MASK(x, 5, 6)
#define MMU_COMMAND                     0x00003f08
#define  V_MMU_CMD(x)                           VAL_MASK(x, 3, 0)
#define MMU_PAGE_FAULT_ADDR             0x00003f0c
#define  V_PAGE_FAULT_ADDR(x)                   VAL_MASK(x, 32, 0)
#define MMU_ZAP_ONE_LINE                0x00003f10
#define  V_MMU_ZAP_ONE_LINE(x)                  VAL_MASK(x, 32, 0)
#define MMU_INT_RAWSTAT                 0x00003f14
#define  V_PAGE_FAULT(x)                        VAL_MASK(x, 1, 0)
#define  V_READ_BUS_ERROR(x)                    VAL_MASK(x, 1, 1)
#define MMU_INT_CLEAR                   0x00003f18
#define  V_PAGE_FAULT(x)                        VAL_MASK(x, 1, 0)
#define  V_READ_BUS_ERROR(x)                    VAL_MASK(x, 1, 1)
#define MMU_INT_MASK                    0x00003f1c
#define  V_PAGE_FAULT(x)                        VAL_MASK(x, 1, 0)
#define  V_READ_BUS_ERROR(x)                    VAL_MASK(x, 1, 1)
#define MMU_INT_STATUS                  0x00003f20
#define  V_PAGE_FAULT(x)                        VAL_MASK(x, 1, 0)
#define  V_READ_BUS_ERROR(x)                    VAL_MASK(x, 1, 1)
#define MMU_AUTO_GATING                 0x00003f24
#define  V_MMU_AUTO_GATING(x)                   VAL_MASK(x, 1, 0)

#define  INTR_FS			BIT(0)
#define  INTR_FS_NEW			BIT(1)
#define  INTR_ADDR_SAME			BIT(2)
#define  INTR_LINE_FLAG0		BIT(3)
#define  INTR_LINE_FLAG1		BIT(4)
#define  INTR_BUS_ERROR			BIT(5)
#define  INTR_WIN0_EMPTY		BIT(6)
#define  INTR_WIN1_EMPTY		BIT(7)
#define  INTR_WIN2_EMPTY		BIT(8)
#define  INTR_WIN3_EMPTY		BIT(9)
#define  INTR_HWC_EMPTY			BIT(10)
#define  INTR_POST_BUF_EMPTY		BIT(11)
#define  INTR_PWM_GEN			BIT(12)
#define  INTR_DSP_HOLD_VALID		BIT(13)
#define  INTR_MMU			BIT(14)
#define  INTR_DMA_FINISH		BIT(15)
#define INTR_MASK (INTR_FS | INTR_FS_NEW | INTR_ADDR_SAME | INTR_LINE_FLAG0 | \
			INTR_LINE_FLAG1 | INTR_BUS_ERROR | INTR_WIN0_EMPTY | \
			INTR_WIN1_EMPTY | INTR_WIN2_EMPTY | INTR_WIN3_EMPTY | \
			INTR_HWC_EMPTY | INTR_POST_BUF_EMPTY | INTR_PWM_GEN | \
			INTR_DSP_HOLD_VALID | INTR_MMU | INTR_DMA_FINISH)

#define OUT_CCIR656_MODE_0              5
#define OUT_CCIR656_MODE_1              6
#define OUT_CCIR656_MODE_2              7

enum {
	VOP_RK322XH,
};

enum {
	SDR2HDR_FOR_BT2020,
	SDR2HDR_FOR_HDR,
};

struct vop_data {
	int chip_type;
	struct rk_lcdc_win *win;
	int n_wins;
};

struct vop_device {
	int id;
	const struct vop_data *data;
	struct rk_lcdc_driver driver;
	struct device *dev;
	struct rk_screen *screen;

	void __iomem *regs;
	void *regsbak;
	u32 reg_phy_base;
	u32 len;

	/* one time only one process allowed to config the register */
	spinlock_t reg_lock;

	int prop;	/* used for primary or extended display device */
	bool pre_init;
	bool pwr18;		/* if lcdc use 1.8v power supply */
	/* if aclk or hclk is closed ,access to register is not allowed */
	bool clk_on;
	/* active layer counter,when  atv_layer_cnt = 0,disable lcdc */
	u8 atv_layer_cnt;
	bool pre_sdr2hdr;
	bool post_sdr2hdr;
	bool post_hdr2sdr;

	unsigned int		irq;

	struct clk		*hclk;		/* lcdc AHP clk*/
	struct clk		*dclk;		/* lcdc dclk*/
	struct clk		*aclk;		/* lcdc share memory freq */
	struct clk		*hclk_noc;
	struct clk		*aclk_noc;
	u32 pixclock;

	u32 standby;				/* 1:standby, 0:work */
	u32 iommu_status;
	struct backlight_device *backlight;
	struct clk		*pll_sclk;

	/* lock vop irq reg */
	spinlock_t irq_lock;
};

static inline void vop_writel(struct vop_device *vop_dev, u32 offset, u32 v)
{
	u32 *_pv = (u32 *)vop_dev->regsbak;

	_pv += (offset >> 2);
	*_pv = v;
	writel_relaxed(v, vop_dev->regs + offset);
}

static inline u32 vop_readl(struct vop_device *vop_dev, u32 offset)
{
	u32 v;

	v = readl_relaxed(vop_dev->regs + offset);
	return v;
}

static inline u32 vop_readl_backup(struct vop_device *vop_dev, u32 offset)
{
	u32 v;
	u32 *_pv = (u32 *)vop_dev->regsbak;

	_pv += (offset >> 2);
	v = readl_relaxed(vop_dev->regs + offset);
	*_pv = v;
	return v;
}

static inline u32 vop_read_bit(struct vop_device *vop_dev, u32 offset, u64 v)
{
	u32 _v = readl_relaxed(vop_dev->regs + offset);

	_v &= v >> 32;
	v = (_v ? 1 : 0);
	return v;
}

static inline void vop_set_bit(struct vop_device *vop_dev, u32 offset, u64 v)
{
	u32 *_pv = (u32 *)vop_dev->regsbak;

	_pv += (offset >> 2);
	(*_pv) |= v >> 32;
	writel_relaxed(*_pv, vop_dev->regs + offset);
}

static inline void vop_clr_bit(struct vop_device *vop_dev, u32 offset, u64 v)
{
	u32 *_pv = (u32 *)vop_dev->regsbak;

	_pv += (offset >> 2);
	(*_pv) &= (~(v >> 32));
	writel_relaxed(*_pv, vop_dev->regs + offset);
}

static inline void  vop_msk_reg(struct vop_device *vop_dev, u32 offset, u64 v)
{
	u32 *_pv = (u32 *)vop_dev->regsbak;

	_pv += (offset >> 2);
	(*_pv) &= (~(v >> 32));
	(*_pv) |= (u32)v;
	writel_relaxed(*_pv, vop_dev->regs + offset);
}

static inline void vop_mask_writel(struct vop_device *vop_dev, u32 offset,
				   u32 mask, u32 v)
{
	v = mask << 16 | v;
	writel_relaxed(v, vop_dev->regs + offset);
}

static inline void vop_cfg_done(struct vop_device *vop_dev)
{
	writel_relaxed(0x001f001f, vop_dev->regs + REG_CFG_DONE);
	dsb(sy);
}

static inline int vop_grf_writel(struct regmap *base, u32 offset, u32 val)
{
	regmap_write(base, offset, val);
	dsb(sy);

	return 0;
}

static inline int vop_cru_writel(struct regmap *base, u32 offset, u32 val)
{
	regmap_write(base, offset, val);
	dsb(sy);

	return 0;
}

static inline int vop_cru_readl(struct regmap *base, u32 offset)
{
	u32 v;

	regmap_read(base, offset, &v);

	return v;
}

enum lb_mode {
	LB_YUV_3840X5 = 0x0,
	LB_YUV_2560X8 = 0x1,
	LB_RGB_3840X2 = 0x2,
	LB_RGB_2560X4 = 0x3,
	LB_RGB_1920X5 = 0x4,
	LB_RGB_1280X8 = 0x5
};

enum sacle_up_mode {
	SCALE_UP_BIL = 0x0,
	SCALE_UP_BIC = 0x1
};

enum scale_down_mode {
	SCALE_DOWN_BIL = 0x0,
	SCALE_DOWN_AVG = 0x1
};

/* ALPHA BLENDING MODE */
enum alpha_mode {               /*  Fs		Fd */
	AB_USER_DEFINE     = 0x0,
	AB_CLEAR	   = 0x1,/*  0		0 */
	AB_SRC		   = 0x2,/*  1		0 */
	AB_DST		   = 0x3,/*  0		1  */
	AB_SRC_OVER	   = 0x4,/*  1		1-As'' */
	AB_DST_OVER	   = 0x5,/*  1-Ad''	1 */
	AB_SRC_IN	   = 0x6,
	AB_DST_IN	   = 0x7,
	AB_SRC_OUT	   = 0x8,
	AB_DST_OUT	   = 0x9,
	AB_SRC_ATOP        = 0xa,
	AB_DST_ATOP	   = 0xb,
	XOR                = 0xc,
	AB_SRC_OVER_GLOBAL = 0xd
};

enum src_alpha_mode {
	AA_STRAIGHT	   = 0x0,
	AA_INVERSE         = 0x1
};/* src_alpha_mode */

enum global_alpha_mode {
	AA_GLOBAL	  = 0x0,
	AA_PER_PIX        = 0x1,
	AA_PER_PIX_GLOBAL = 0x2
};/* src_global_alpha_mode */

enum src_alpha_sel {
	AA_SAT		= 0x0,
	AA_NO_SAT	= 0x1
};/* src_alpha_sel */

enum src_color_mode {
	AA_SRC_PRE_MUL	       = 0x0,
	AA_SRC_NO_PRE_MUL      = 0x1
};/* src_color_mode */

enum factor_mode {
	AA_ZERO			= 0x0,
	AA_ONE			= 0x1,
	AA_SRC			= 0x2,
	AA_SRC_INVERSE          = 0x3,
	AA_SRC_GLOBAL           = 0x4
};/* src_factor_mode  &&  dst_factor_mode */

enum _vop_csc_mode {
	VOP_CSC_BT601L = 0,
	VOP_CSC_BT709L,
	VOP_CSC_BT601F,
	VOP_CSC_BT2020
};

enum _vop_format {
	VOP_FORMAT_ARGB888 = 0,
	VOP_FORMAT_RGB888,
	VOP_FORMAT_RGB565,
	VOP_FORMAT_YCBCR420 = 4,
	VOP_FORMAT_YCBCR422,
	VOP_FORMAT_YCBCR444
};

#define IS_YUV(x) ((x) >= VOP_FORMAT_YCBCR420)

enum _vop_overlay_mode {
	VOP_RGB_DOMAIN,
	VOP_YUV_DOMAIN
};

struct alpha_config {
	enum src_alpha_mode src_alpha_mode;	/* win0_src_alpha_m0 */
	u32 src_global_alpha_val;		/* win0_src_global_alpha */
	enum global_alpha_mode src_global_alpha_mode;	/* win0_src_blend_m0 */
	enum src_alpha_sel src_alpha_cal_m0;	/* win0_src_alpha_cal_m0 */
	enum src_color_mode src_color_mode;	/* win0_src_color_m0 */
	enum factor_mode src_factor_mode;	/* win0_src_factor_m0 */
	enum factor_mode dst_factor_mode;	/* win0_dst_factor_m0 */
};

struct lcdc_cabc_mode {
	u32 pixel_num;		/* pixel precent number */
	u16 stage_up;		/* up stride */
	u16 stage_down;		/* down stride */
	u16 global_su;
};

#define CUBIC_PRECISE  0
#define CUBIC_SPLINE   1
#define CUBIC_CATROM   2
#define CUBIC_MITCHELL 3

#define FBDC_FMT_RGB565		0x5
#define FBDC_FMT_U8U8U8U8	0xc	/* ARGB888 */
#define FBDC_FMT_U8U8U8		0x3a	/* RGBP888 */

#define CUBIC_MODE_SELETION      CUBIC_PRECISE

/*************************************************************/
#define SCALE_FACTOR_BILI_DN_FIXPOINT_SHIFT   12   /* 4.12 */
#define SCALE_FACTOR_BILI_DN_FIXPOINT(x)      \
	((INT32)((x) * (1 << SCALE_FACTOR_BILI_DN_FIXPOINT_SHIFT)))

#define SCALE_FACTOR_BILI_UP_FIXPOINT_SHIFT   16   /* 0.16 */

#define SCALE_FACTOR_AVRG_FIXPOINT_SHIFT   16   /* 0.16 */
#define SCALE_FACTOR_AVRG_FIXPOINT(x)      \
	((INT32)((x) * (1 << SCALE_FACTOR_AVRG_FIXPOINT_SHIFT)))

#define SCALE_FACTOR_BIC_FIXPOINT_SHIFT    16   /* 0.16 */
#define SCALE_FACTOR_BIC_FIXPOINT(x)       \
	((INT32)((x) * (1 << SCALE_FACTOR_BIC_FIXPOINT_SHIFT)))

#define SCALE_FACTOR_DEFAULT_FIXPOINT_SHIFT    12  /* NONE SCALE,vsd_bil */
#define SCALE_FACTOR_VSDBIL_FIXPOINT_SHIFT     12  /* VER SCALE DOWN BIL */

/*********************************************************/

/* #define GET_SCALE_FACTOR_BILI(src, dst)  \
 *	((((src) - 1) << SCALE_FACTOR_BILI_FIXPOINT_SHIFT) / ((dst) - 1))
 * #define GET_SCALE_FACTOR_BIC(src, dst)   \
 *	((((src) - 1) << SCALE_FACTOR_BIC_FIXPOINT_SHIFT) / ((dst) - 1))
 * modified by hpz
 */
#define GET_SCALE_FACTOR_BILI_DN(src, dst)  \
	((((src) * 2 - 3) << (SCALE_FACTOR_BILI_DN_FIXPOINT_SHIFT - 1)) \
	/ ((dst) - 1))
#define GET_SCALE_FACTOR_BILI_UP(src, dst)  \
	((((src) * 2 - 3) << (SCALE_FACTOR_BILI_UP_FIXPOINT_SHIFT - 1)) \
	/ ((dst) - 1))
#define GET_SCALE_FACTOR_BIC(src, dst)      \
	((((src) * 2 - 3) << (SCALE_FACTOR_BIC_FIXPOINT_SHIFT - 1)) \
	/ ((dst) - 1))

/*********************************************************/
/* NOTE: hardware in order to save resource , srch first to get interlace line
 * (srch+vscalednmult-1)/vscalednmult; and do scale
 */
#define GET_SCALE_DN_ACT_HEIGHT(srch, vscalednmult) \
	(((srch) + (vscalednmult) - 1) / (vscalednmult))

/* #define VSKIP_MORE_PRECISE */

#ifdef VSKIP_MORE_PRECISE
#define MIN_SCALE_FACTOR_AFTER_VSKIP        1.5f
#define GET_SCALE_FACTOR_BILI_DN_VSKIP(srch, dsth, vscalednmult) \
	(GET_SCALE_FACTOR_BILI_DN(GET_SCALE_DN_ACT_HEIGHT((srch),\
	(vscalednmult)), (dsth)))
#else
#define MIN_SCALE_FACTOR_AFTER_VSKIP        1
#define GET_SCALE_FACTOR_BILI_DN_VSKIP(srch, dsth, vscalednmult) \
	((GET_SCALE_DN_ACT_HEIGHT((srch), (vscalednmult)) == (dsth)) \
	? (GET_SCALE_FACTOR_BILI_DN((srch), (dsth)) / (vscalednmult)) \
	: (GET_SCALE_DN_ACT_HEIGHT((srch), (vscalednmult)) == ((dsth) * 2)) \
	?  GET_SCALE_FACTOR_BILI_DN(GET_SCALE_DN_ACT_HEIGHT(((srch) - 1),\
	(vscalednmult)), (dsth)) : \
	GET_SCALE_FACTOR_BILI_DN(GET_SCALE_DN_ACT_HEIGHT((srch),\
	(vscalednmult)), (dsth)))

#endif
/*****************************************************************/

/* scalefactor must >= dst/src, or pixels at end of line may be unused
 * scalefactor must < dst/(src-1), or dst buffer may overflow
 * avrg old code: ((((dst) << SCALE_FACTOR_AVRG_FIXPOINT_SHIFT))\
	/((src) - 1)) hxx_chgsrc
 * modified by hpz:
 */
#define GET_SCALE_FACTOR_AVRG(src, dst)  ((((dst) << \
	(SCALE_FACTOR_AVRG_FIXPOINT_SHIFT + 1))) / (2 * (src) - 1))

/*************************************************************************/
/*Scale Coordinate Accumulate, x.16*/
#define SCALE_COOR_ACC_FIXPOINT_SHIFT     16
#define SCALE_COOR_ACC_FIXPOINT_ONE	BIT(SCALE_COOR_ACC_FIXPOINT_SHIFT)
#define SCALE_COOR_ACC_FIXPOINT(x) \
	((INT32)((x) * (1 << SCALE_COOR_ACC_FIXPOINT_SHIFT)))
#define SCALE_COOR_ACC_FIXPOINT_REVERT(x) \
	((((x) >> (SCALE_COOR_ACC_FIXPOINT_SHIFT - 1)) + 1) >> 1)

#define SCALE_GET_COOR_ACC_FIXPOINT(scalefactor, factorfixpointshift)  \
	((scalefactor) << \
	(SCALE_COOR_ACC_FIXPOINT_SHIFT - (factorfixpointshift)))

/************************************************************************/
/* CoarsePart of Scale Coordinate Accumulate, for pixel mult-add factor 0.8 */
#define SCALE_FILTER_FACTOR_FIXPOINT_SHIFT     8
#define SCALE_FILTER_FACTOR_FIXPOINT_ONE       \
	(1 << SCALE_FILTER_FACTOR_FIXPOINT_SHIFT)
#define SCALE_FILTER_FACTOR_FIXPOINT(x)        \
	((INT32)((x) * (1 << SCALE_FILTER_FACTOR_FIXPOINT_SHIFT)))
#define SCALE_FILTER_FACTOR_FIXPOINT_REVERT(x) \
	((((x) >> (SCALE_FILTER_FACTOR_FIXPOINT_SHIFT - 1)) + 1) >> 1)

#define SCALE_GET_FILTER_FACTOR_FIXPOINT(cooraccumulate, \
	cooraccfixpointshift) \
	(((cooraccumulate) >> \
	((cooraccfixpointshift) - SCALE_FILTER_FACTOR_FIXPOINT_SHIFT)) & \
	(SCALE_FILTER_FACTOR_FIXPOINT_ONE - 1))

#define SCALE_OFFSET_FIXPOINT_SHIFT            8
#define SCALE_OFFSET_FIXPOINT(x)              \
	((INT32)((x) * (1 << SCALE_OFFSET_FIXPOINT_SHIFT)))

static inline u32 vop_get_hard_ware_vskiplines(u32 srch, u32 dsth)
{
	u32 vscalednmult;

	if (srch >= (u32)(4 * dsth * MIN_SCALE_FACTOR_AFTER_VSKIP))
		vscalednmult = 4;
	else if (srch >= (u32)(2 * dsth * MIN_SCALE_FACTOR_AFTER_VSKIP))
		vscalednmult = 2;
	else
		vscalednmult = 1;

	return vscalednmult;
}

#endif
