/*
 * rk1108_codec.h  --  rk1108 ALSA Soc Audio driver
 *
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __RK1108_CODEC_H__
#define __RK1108_CODEC_H__

#define RK1108_RESET				(0x00) /* REG 0x00 */
#define RK1108_ADC_INT_CTL1			(0x08) /* REG 0x02 */
#define RK1108_ADC_INT_CTL2			(0x0c) /* REG 0x03 */
#define RK1108_DAC_INT_CTL1			(0x10) /* REG 0x04 */
#define RK1108_DAC_INT_CTL2			(0x14) /* REG 0x05 */
#define RK1108_BIST_CTL				(0x1c) /* REG 0x07 */
#define RK1108_SELECT_CURRENT			(0x88) /* REG 0x22 */
#define RK1108_BIAS_CTL				(0x8c) /* REG 0x23 */
#define RK1108_ADC_CTL				(0x90) /* REG 0x24 */
#define RK1108_BST_CTL				(0x94) /* REG 0x25 */
#define RK1108_ALC_MUNIN_CTL			(0x98) /* REG 0x26 */
#define RK1108_ALCL_GAIN_CTL			(0x9c) /* REG 0x27 */
#define RK1108_ALCR_GAIN_CTL			(0xa0) /* REG 0x28 */
#define RK1108_ADC_ENABLE			(0xa4) /* REG 0x29 */
#define RK1108_DAC_CTL				(0xa8) /* REG 0x2a */
#define RK1108_DAC_ENABLE			(0xac) /* REG 0x2b */
#define RK1108_LOUT_CTL				(0xb8) /* REG 0x2e */
#define RK1108_LOUTL_GAIN			(0xbc) /* REG 0x2f */
#define RK1108_LOUTR_GAIN			(0xc0) /* REG 0x30 */
#define RK1108_PGA_AGC_CTL1			(0x100) /* REG 0x40 */
#define RK1108_PGA_AGC_CTL2			(0x104) /* REG 0x41 */
#define RK1108_PGA_AGC_CTL3			(0x108) /* REG 0x42 */
#define RK1108_PGA_AGC_CTL4			(0x10c) /* REG 0x43 */
#define RK1108_PGA_ASR_CTL			(0x110) /* REG 0x44 */
#define RK1108_PGA_AGC_MAX_H			(0x114) /* REG 0x45 */
#define RK1108_PGA_AGC_MAX_L			(0x118) /* REG 0x46 */
#define RK1108_PGA_AGC_MIN_H			(0x11c) /* REG 0x47 */
#define RK1108_PGA_AGC_MIN_L			(0x120) /* REG 0x48 */
#define RK1108_PGA_AGC_CTL5			(0x124) /* REG 0x49 */

/* ADC INTERFACE CONTROL 1 (REG 0X02) */
#define RK1108_ALRCK_POL_MASK			(0x1 << 7)
#define RK1108_ALRCK_POL_SFT			7
#define RK1108_ALRCK_POL_EN			(0x1 << 7)
#define RK1108_ALRCK_POL_DIS			(0x0 << 7)
#define RK1108_ADC_VWL_MASK			(0x3 << 5)
#define RK1108_ADC_VWL_SFT			5
#define RK1108_ADC_VWL_32			(0x3 << 5)
#define RK1108_ADC_VWL_24			(0x2 << 5)
#define RK1108_ADC_VWL_20			(0x1 << 5)
#define RK1108_ADC_VWL_16			(0x0 << 5)
#define RK1108_ADC_DF_MASK			(0x3 << 3)
#define RK1108_ADC_DF_SFT			3
#define RK1108_ADC_DF_PCM			(0x3 << 3)
#define RK1108_ADC_DF_I2S			(0x2 << 3)
#define RK1108_ADC_DF_LJ			(0x1 << 3)
#define RK1108_ADC_DF_RJ			(0x0 << 3)
#define RK1108_ADC_SWAP_MASK			(0x1 << 1)
#define RK1108_ADC_SWAP_SFT			1
#define RK1108_ADC_SWAP_EN			(0x1 << 1)
#define RK1108_ADC_SWAP_DIS			(0x0 << 1)
#define RK1108_ADC_TYPE_MASK			(0x1 << 0)
#define RK1108_ADC_TYPE_SFT			0
#define RK1108_ADC_TYPE_MONO			(0x1 << 0)
#define RK1108_ADC_TYPE_STEREO			(0x0 << 0)

/* ADC INTERFACE CONTROL 2 (REG 0X03) */
#define RK1108_I2S_MODE_MASK			(0x1 << 4)
#define RK1108_I2S_MODE_SFT			(4)
#define RK1108_I2S_MODE_MST			(0x1 << 4)
#define RK1108_I2S_MODE_SLV			(0x0 << 4)
#define RK1108_ADC_WL_MASK			(0x3 << 2)
#define RK1108_ADC_WL_SFT			(2)
#define RK1108_ADC_WL_32			(0x3 << 2)
#define RK1108_ADC_WL_24			(0x2 << 2)
#define RK1108_ADC_WL_20			(0x1 << 2)
#define RK1108_ADC_WL_16			(0x0 << 2)
#define RK1108_ADC_RST_MASK			(0x1 << 1)
#define RK1108_ADC_RST_SFT			1
#define RK1108_ADC_RST_DIS			(0x1 << 1)
#define RK1108_ADC_RST_EN			(0x0 << 1)
#define RK1108_ABCLK_POL_MASK			(0x1 << 0)
#define RK1108_ABCLK_POL_SFT			0
#define RK1108_ABCLK_POL_EN			(0x1 << 0)
#define RK1108_ABCLK_POL_DIS			(0x0 << 0)

/* DAC INTERFACE CONTROL 1 (REG 0X04) */
#define RK1108_DLRCK_POL_MASK			(0x1 << 7)
#define RK1108_DLRCK_POL_SFT			7
#define RK1108_DLRCK_POL_EN			(0x1 << 7)
#define RK1108_DLRCK_POL_DIS			(0x0 << 7)
#define RK1108_DAC_VWL_MASK			(0x3 << 5)
#define RK1108_DAC_VWL_SFT			5
#define RK1108_DAC_VWL_32			(0x3 << 5)
#define RK1108_DAC_VWL_24			(0x2 << 5)
#define RK1108_DAC_VWL_20			(0x1 << 5)
#define RK1108_DAC_VWL_16			(0x0 << 5)
#define RK1108_DAC_DF_MASK			(0x3 << 3)
#define RK1108_DAC_DF_SFT			3
#define RK1108_DAC_DF_PCM			(0x3 << 3)
#define RK1108_DAC_DF_I2S			(0x2 << 3)
#define RK1108_DAC_DF_LJ			(0x1 << 3)
#define RK1108_DAC_DF_RJ			(0x0 << 3)
#define RK1108_DAC_SWAP_MASK			(0x1 << 2)
#define RK1108_DAC_SWAP_SFT			2
#define RK1108_DAC_SWAP_EN			(0x1 << 2)
#define RK1108_DAC_SWAP_DIS			(0x0 << 2)

/* DAC INTERFACE CONTROL 2 (REG 0X05) */
#define RK1108_DAC_WL_MASK			(0x3 << 2)
#define RK1108_DAC_WL_SFT			2
#define RK1108_DAC_WL_32			(0x3 << 2)
#define RK1108_DAC_WL_24			(0x2 << 2)
#define RK1108_DAC_WL_20			(0x1 << 2)
#define RK1108_DAC_WL_16			(0x0 << 2)
#define RK1108_DAC_RST_MASK			(0x1 << 1)
#define RK1108_DAC_RST_SFT			1
#define RK1108_DAC_RST_DIS			(0x1 << 1)
#define RK1108_DAC_RST_EN			(0x0 << 1)
#define RK1108_DBCLK_POL_MASK			(0x1 << 0)
#define RK1108_DBCLK_POL_SFT			0
#define RK1108_DBCLK_POL_EN			(0x1 << 0)
#define RK1108_DBCLK_POL_DIS			(0x0 << 0)

/* SELECT CURR PRECHAGRGE/DISCHARGE (REG 0X22) */
#define RK1108_XCHARGE_MASK			(0x1 << 7)
#define RK1108_PRECHARGE_HPOUT			(0x1 << 7)
#define RK1108_DISCHARGE_HPOUT			(0x0 << 7)
#define RK1108_CHARGE_CURRENT_ALL_MASK		(0x7f << 0)
#define RK1108_CHARGE_CURRENT_ALL_ON		(0x7f << 0)
#define RK1108_CHARGE_CURRENT_ALL_OFF		(0x0 << 0)

/*  MICBIAS (REG 0X23) */
#define RK1108_MICBIAS_VOL_EN_MASK		(0x1 << 3)
#define RK1108_MICBIAS_VOL_EN			(0x1 << 3)
#define RK1108_MICBIAS_VOL_DIS			(0x0 << 3)
#define RK1108_MICBIAS_VOL_MSK			(0x7 << 0)
#define RK1108_MICBIAS_VOL_MIN			(0x0 << 0)
#define RK1108_MICBIAS_VOL_MAX			(0x7 << 0)

/* ADC CONTROL (REG 0X24) */
#define RK1108_ADC_CURRENT_MASK			(0x1 << 6)
#define RK1108_ADC_CURRENT_EN			(0x1 << 6)
#define RK1108_ADC_CURRENT_DIS			(0x0 << 6)
#define RK1108_ADCL_REF_VOL_EN_MASK		(0x1 << 5)
#define RK1108_ADCL_REF_VOL_EN			(0x1 << 5)
#define RK1108_ADCL_REF_VOL_DIS			(0x0 << 5)
#define RK1108_ADCL_ZERO_DET_EN_MASK		(0x1 << 4)
#define RK1108_ADCL_ZERO_DET_EN			(0x1 << 4)
#define RK1108_ADCL_ZERO_DET_DIS		(0x0 << 4)
#define RK1108_ADCR_REF_VOL_EN_MASK		(0x1 << 1)
#define RK1108_ADCR_REF_VOL_EN			(0x1 << 1)
#define RK1108_ADCR_REF_VOL_DIS			(0x0 << 1)
#define RK1108_ADCR_ZERO_DET_EN_MASK		(0x1 << 0)
#define RK1108_ADCR_ZERO_DET_EN			(0x1 << 0)
#define RK1108_ADCR_ZERO_DET_DIS		(0x0 << 0)

/* BST_L  BST_R  CONTROL (REG 0X25)  */
#define RK1108_BSTL_PWRD_MASK			(0x1 << 7)
#define RK1108_BSTL_EN				(0x1 << 7)
#define RK1108_BSTL_DIS				(0x0 << 7)
#define RK1108_BSTL_GAIN_MASK			(0x1 << 6)
#define RK1108_BSTL_GAIN_20			(0x1 << 6)
#define RK1108_BSTL_GAIN_0			(0x0 << 6)
#define RK1108_BSTL_MUTE_MASK			(0x1 << 5)
#define RK1108_BSTL_MUTE			(0x0 << 5)
#define RK1108_BSTL_UNMUTE			(0x1 << 5)
#define RK1108_BSTR_PWRD_MASK			(0x1 << 3)
#define RK1108_BSTR_EN				(0x1 << 3)
#define RK1108_BSTR_DIS				(0x0 << 3)
#define RK1108_BSTR_GAIN_MASK			(0x1 << 2)
#define RK1108_BSTR_GAIN_20			(0x1 << 2)
#define RK1108_BSTR_GAIN_0			(0x0 << 2)
#define RK1108_BSTR_MUTE_MASK			(0x1 << 1)
#define RK1108_BSTR_MUTE			(0x0 << 1)
#define RK1108_BSTR_UNMUTE			(0x1 << 1)

/* MUXINL ALCL MUXINR ALCR  (REG 0X26)  */
#define RK1108_ALCL_PWR_MASK			(0x1 << 5)
#define RK1108_ALCL_EN				(0x1 << 5)
#define RK1108_ALCL_DIS				(0x0 << 5)
#define RK1108_ALCL_MUTE_MASK			(0x1 << 4)
#define RK1108_ALCL_MUTE			(0x0 << 4)
#define RK1108_ALCL_UNMUTE			(0x1 << 4)
#define RK1108_ALCR_PWR_MASK			(0x1 << 1)
#define RK1108_ALCR_EN				(0x1 << 1)
#define RK1108_ALCR_DIS				(0x0 << 1)
#define RK1108_ALCR_MUTE_MASK			(0x1 << 0)
#define RK1108_ALCR_MUTE			(0x0 << 0)
#define RK1108_ALCR_UNMUTE			(0x1 << 0)

/* ALC_L GAIN (REG 0X27) */

#define RK1108_ALCL_GAIN_SHT			(0)
#define RK1108_ALCL_GAIN_MSK			(0x1f)

/* ALC_R GAIN (REG 0X28) */
#define RK1108_ALCR_GAIN_SHT			(0)
#define RK1108_ALCR_GAIN_MSK			(0x1f)

/* ADC ENABLE (REG 0X29) */
#define RK1108_ADCL_CLK_EN_MASK			(0x1 << 6)
#define RK1108_ADCL_CLK_EN			(0x1 << 6)
#define RK1108_ADCL_CLK_DIS			(0x0 << 6)
#define RK1108_ADCL_AMP_EN_MASK			(0x1 << 5)
#define RK1108_ADCL_AMP_EN			(0x1 << 5)
#define RK1108_ADCL_AMP_DIS			(0x0 << 5)
#define RK1108_ADCL_RST_MASK			(0x1 << 4)
#define RK1108_ADCL_RST_EN			(0x1 << 4)
#define RK1108_ADCL_RST_DIS			(0x0 << 4)
#define RK1108_ADCR_CLK_EN_MASK			(0x1 << 2)
#define RK1108_ADCR_CLK_EN			(0x1 << 2)
#define RK1108_ADCR_CLK_DIS			(0x0 << 2)
#define RK1108_ADCR_AMP_EN_MASK			(0x1 << 1)
#define RK1108_ADCR_AMP_EN			(0x1 << 1)
#define RK1108_ADCR_AMP_DIS			(0x0 << 1)
#define RK1108_ADCR_RST_MASK			(0x1 << 0)
#define RK1108_ADCR_RST_EN			(0x1 << 0)
#define RK1108_ADCR_RST_DIS			(0x0 << 0)

/* DAC & VOUT CONTROL (REG 0X2A)  */
#define RK1108_CURRENT_MASK			(0x1 << 6)
#define RK1108_CURRENT_EN			(0x1 << 6)
#define RK1108_CURRENT_DIS			(0x0 << 6)
#define RK1108_REF_VOL_DACL_MASK		(0x1 << 5)
#define RK1108_REF_VOL_DACL_EN			(0x1 << 5)
#define RK1108_REF_VOL_DACL_DIS			(0x0 << 5)
#define RK1108_REF_VOL_DACR_MASK		(0x1 << 1)
#define RK1108_REF_VOL_DACR_EN			(0x1 << 1)
#define RK1108_REF_VOL_DACR_DIS			(0x0 << 1)

/* DAC CONTROL (REG 0X2B) */
#define RK1108_DACL_REF_VOL_MASK		(0x1 << 7)
#define RK1108_DACL_REF_VOL_EN			(0x1 << 7)
#define RK1108_DACL_REF_VOL_DIS			(0x0 << 7)
#define RK1108_DACL_CLK_MASK			(0x1 << 6)
#define RK1108_DACL_CLK_EN			(0x1 << 6)
#define RK1108_DACL_CLK_DIS			(0x0 << 6)
#define RK1108_DACL_EN_MASK			(0x1 << 5)
#define RK1108_DACL_EN				(0x1 << 5)
#define RK1108_DACL_DIS				(0x0 << 5)
#define RK1108_DACL_INIT_MASK			(0x1 << 4)
#define RK1108_DACL_INIT			(0x0 << 4)
#define RK1108_DACL_WORK			(0x1 << 4)
#define RK1108_DACR_REF_VOL_MASK		(0x1 << 3)
#define RK1108_DACR_REF_VOL_EN			(0x1 << 3)
#define RK1108_DACR_REF_VOL_DIS			(0x0 << 3)
#define RK1108_DACR_CLK_MASK			(0x1 << 2)
#define RK1108_DACR_CLK_EN			(0x1 << 2)
#define RK1108_DACR_CLK_DIS			(0x0 << 2)
#define RK1108_DACR_EN_MASK			(0x1 << 1)
#define RK1108_DACR_EN				(0x1 << 1)
#define RK1108_DACR_DIS				(0x0 << 1)
#define RK1108_DACR_INIT_MASK			(0x1 << 0)
#define RK1108_DACR_INIT			(0x0 << 0)
#define RK1108_DACR_WORK			(0x1 << 0)

/* LOUT CONTROL  (REG 0X2E) */
#define RK1108_LOUTL_PWR_SHT			(7)
#define RK1108_LOUTL_MSK			(0x1 << 7)
#define RK1108_LOUTL_EN				(0x1 << 7)
#define RK1108_LOUTL_DIS			(0x0 << 7)
#define RK1108_LOUTL_MUTE_SHT			(5)
#define RK1108_LOUTL_MUTE_MSK			(0x1 << 5)
#define RK1108_LOUTL_MUTE			(0x0 << 5)
#define RK1108_LOUTL_UNMUTE			(0x1 << 5)
#define RK1108_LOUTR_PWR_SHT			(4)
#define RK1108_LOUTR_MSK			(0x1 << 4)
#define RK1108_LOUTR_EN				(0x1 << 4)
#define RK1108_LOUTR_DIS			(0x0 << 4)
#define RK1108_LOUTR_MUTE_SHT			(2)
#define RK1108_LOUTR_MUTE_MSK			(0x1 << 2)
#define RK1108_LOUTR_MUTE			(0x0 << 2)
#define RK1108_LOUTR_UNMUTE			(0x1 << 2)

/* LOUT GAIN (REG 0X2F, 0X30) */
#define RK1108_LOUT_GAIN_MASK			(0X1f << 0)

/* PGA AGC CONTROL 1 (REG 0X40) */
#define RK1108_PGA_AGC_WAY_MASK			(0x1 << 6)
#define RK1108_PGA_AGC_WAY_SFT			6
#define RK1108_PGA_AGC_WAY_JACK			(0x1 << 6)
#define RK1108_PGA_AGC_WAY_NOR			(0x0 << 6)
#define RK1108_PGA_AGC_BK_WAY_SFT		4
#define RK1108_PGA_AGC_BK_WAY_JACK1		(0x1 << 4)
#define RK1108_PGA_AGC_BK_WAY_NOR		(0x0 << 4)
#define RK1108_PGA_AGC_BK_WAY_JACK2		(0x2 << 4)
#define RK1108_PGA_AGC_BK_WAY_JACK3		(0x3 << 4)
#define RK1108_PGA_AGC_HOLD_T_MASK		(0xf << 0)
#define RK1108_PGA_AGC_HOLD_T_SFT		0
#define RK1108_PGA_AGC_HOLD_T_1024		(0xa << 0)
#define RK1108_PGA_AGC_HOLD_T_512		(0x9 << 0)
#define RK1108_PGA_AGC_HOLD_T_256		(0x8 << 0)
#define RK1108_PGA_AGC_HOLD_T_128		(0x7 << 0)
#define RK1108_PGA_AGC_HOLD_T_64		(0x6 << 0)
#define RK1108_PGA_AGC_HOLD_T_32		(0x5 << 0)
#define RK1108_PGA_AGC_HOLD_T_16		(0x4 << 0)
#define RK1108_PGA_AGC_HOLD_T_8			(0x3 << 0)
#define RK1108_PGA_AGC_HOLD_T_4			(0x2 << 0)
#define RK1108_PGA_AGC_HOLD_T_2			(0x1 << 0)
#define RK1108_PGA_AGC_HOLD_T_0			(0x0 << 0)

/* PGA AGC CONTROL 2 (REG 0X41) */
#define RK1108_PGA_AGC_GRU_T_MASK		(0xf << 4)
#define RK1108_PGA_AGC_GRU_T_SFT			4
#define RK1108_PGA_AGC_GRU_T_512		(0xa << 4)
#define RK1108_PGA_AGC_GRU_T_256		(0x9 << 4)
#define RK1108_PGA_AGC_GRU_T_128		(0x8 << 4)
#define RK1108_PGA_AGC_GRU_T_64			(0x7 << 4)
#define RK1108_PGA_AGC_GRU_T_32			(0x6 << 4)
#define RK1108_PGA_AGC_GRU_T_16			(0x5 << 4)
#define RK1108_PGA_AGC_GRU_T_8			(0x4 << 4)
#define RK1108_PGA_AGC_GRU_T_4			(0x3 << 4)
#define RK1108_PGA_AGC_GRU_T_2			(0x2 << 4)
#define RK1108_PGA_AGC_GRU_T_1			(0x1 << 4)
#define RK1108_PGA_AGC_GRU_T_0_5		(0x0 << 4)
#define RK1108_PGA_AGC_GRD_T_MASK		(0xf << 0)
#define RK1108_PGA_AGC_GRD_T_SFT		0
#define RK1108_PGA_AGC_GRD_T_128_32		(0xa << 0)
#define RK1108_PGA_AGC_GRD_T_64_16		(0x9 << 0)
#define RK1108_PGA_AGC_GRD_T_32_8		(0x8 << 0)
#define RK1108_PGA_AGC_GRD_T_16_4		(0x7 << 0)
#define RK1108_PGA_AGC_GRD_T_8_2		(0x6 << 0)
#define RK1108_PGA_AGC_GRD_T_4_1		(0x5 << 0)
#define RK1108_PGA_AGC_GRD_T_2_0_512		(0x4 << 0)
#define RK1108_PGA_AGC_GRD_T_1_0_256		(0x3 << 0)
#define RK1108_PGA_AGC_GRD_T_0_500_128		(0x2 << 0)
#define RK1108_PGA_AGC_GRD_T_0_250_64		(0x1 << 0)
#define RK1108_PGA_AGC_GRD_T_0_125_32		(0x0 << 0)

/* PGA AGC CONTROL 3 (REG 0X42) */
#define RK1108_PGA_AGC_MODE_MASK		(0x1 << 7)
#define RK1108_PGA_AGC_MODE_SFT			7
#define RK1108_PGA_AGC_MODE_LIMIT		(0x1 << 7)
#define RK1108_PGA_AGC_MODE_NOR			(0x0 << 7)
#define RK1108_PGA_AGC_ZO_MASK			(0x1 << 6)
#define RK1108_PGA_AGC_ZO_SFT			6
#define RK1108_PGA_AGC_ZO_EN			(0x1 << 6)
#define RK1108_PGA_AGC_ZO_DIS			(0x0 << 6)
#define RK1108_PGA_AGC_REC_MODE_MASK		(0x1 << 5)
#define RK1108_PGA_AGC_REC_MODE_SFT		5
#define RK1108_PGA_AGC_REC_MODE_AC		(0x1 << 5)
#define RK1108_PGA_AGC_REC_MODE_RN		(0x0 << 5)
#define RK1108_PGA_AGC_FAST_D_MASK		(0x1 << 4)
#define RK1108_PGA_AGC_FAST_D_SFT		4
#define RK1108_PGA_AGC_FAST_D_EN		(0x1 << 4)
#define RK1108_PGA_AGC_FAST_D_DIS		(0x0 << 4)
#define RK1108_PGA_AGC_NG_MASK			(0x1 << 3)
#define RK1108_PGA_AGC_NG_SFT			3
#define RK1108_PGA_AGC_NG_EN			(0x1 << 3)
#define RK1108_PGA_AGC_NG_DIS			(0x0 << 3)
#define RK1108_PGA_AGC_NG_THR_MASK		(0x7 << 0)
#define RK1108_PGA_AGC_NG_THR_SFT		0
#define RK1108_PGA_AGC_NG_THR_N81DB		(0x7 << 0)
#define RK1108_PGA_AGC_NG_THR_N75DB		(0x6 << 0)
#define RK1108_PGA_AGC_NG_THR_N69DB		(0x5 << 0)
#define RK1108_PGA_AGC_NG_THR_N63DB		(0x4 << 0)
#define RK1108_PGA_AGC_NG_THR_N57DB		(0x3 << 0)
#define RK1108_PGA_AGC_NG_THR_N51DB		(0x2 << 0)
#define RK1108_PGA_AGC_NG_THR_N45DB		(0x1 << 0)
#define RK1108_PGA_AGC_NG_THR_N39DB		(0x0 << 0)

/* PGA AGC CONTROL 4 (REG 0X43) */
#define RK1108_PGA_AGC_ZO_MODE_MASK		(0x1 << 5)
#define RK1108_PGA_AGC_ZO_MODE_SFT		5
#define RK1108_PGA_AGC_ZO_MODE_UWRC		(0x1 << 5)
#define RK1108_PGA_AGC_ZO_MODE_UARC		(0x0 << 5)
#define RK1108_PGA_AGC_VOL_MASK			0x1f
#define RK1108_PGA_AGC_VOL_SFT			0

/* PGA ASR CONTROL (REG 0X44) */
#define RK1108_PGA_SLOW_CLK_MASK		(0x1 << 3)
#define RK1108_PGA_SLOW_CLK_SFT			3
#define RK1108_PGA_SLOW_CLK_EN			(0x1 << 3)
#define RK1108_PGA_SLOW_CLK_DIS			(0x0 << 3)
#define RK1108_PGA_ASR_MASK			(0x7 << 0)
#define RK1108_PGA_ASR_SFT			0
#define RK1108_PGA_ASR_8KHZ			(0x7 << 0)
#define RK1108_PGA_ASR_12KHZ			(0x6 << 0)
#define RK1108_PGA_ASR_16KHZ			(0x5 << 0)
#define RK1108_PGA_ASR_24KHZ			(0x4 << 0)
#define RK1108_PGA_ASR_32KHZ			(0x3 << 0)
#define RK1108_PGA_ASR_441KHZ			(0x2 << 0)
#define RK1108_PGA_ASR_48KHZ			(0x1 << 0)
#define RK1108_PGA_ASR_96KHZ			(0x0 << 0)

/* PGA AGC CONTROL 5 (REG 0X49) */
#define RK1108_PGA_AGC_MASK			(0x1 << 6)
#define RK1108_PGA_AGC_SFT			6
#define RK1108_PGA_AGC_EN			(0x1 << 6)
#define RK1108_PGA_AGC_DIS			(0x0 << 6)
#define RK1108_PGA_AGC_MAX_G_MASK		(0x7 << 3)
#define RK1108_PGA_AGC_MAX_G_SFT		3
#define RK1108_PGA_AGC_MAX_G_28_5DB		(0x7 << 3)
#define RK1108_PGA_AGC_MAX_G_22_5DB		(0x6 << 3)
#define RK1108_PGA_AGC_MAX_G_16_5DB		(0x5 << 3)
#define RK1108_PGA_AGC_MAX_G_10_5DB		(0x4 << 3)
#define RK1108_PGA_AGC_MAX_G_4_5DB		(0x3 << 3)
#define RK1108_PGA_AGC_MAX_G_N1_5DB		(0x2 << 3)
#define RK1108_PGA_AGC_MAX_G_N7_5DB		(0x1 << 3)
#define RK1108_PGA_AGC_MAX_G_N13_5DB		(0x0 << 3)
#define RK1108_PGA_AGC_MIN_G_MASK		(0x7 << 0)
#define RK1108_PGA_AGC_MIN_G_SFT		0
#define RK1108_PGA_AGC_MIN_G_24DB		(0x7 << 0)
#define RK1108_PGA_AGC_MIN_G_18DB		(0x6 << 0)
#define RK1108_PGA_AGC_MIN_G_12DB		(0x5 << 0)
#define RK1108_PGA_AGC_MIN_G_6DB		(0x4 << 0)
#define RK1108_PGA_AGC_MIN_G_0DB		(0x3 << 0)
#define RK1108_PGA_AGC_MIN_G_N6DB		(0x2 << 0)
#define RK1108_PGA_AGC_MIN_G_N12DB		(0x1 << 0)
#define RK1108_PGA_AGC_MIN_G_N18DB		(0x0 << 0)

#define RK1108_HIFI				(0x0)

struct rk1108_reg_msk_val {
	unsigned int reg;
	unsigned int msk;
	unsigned int val;
};

#endif
