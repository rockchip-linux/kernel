/*
**************************************************************************
 * Rockchip driver for NVP6124b
 * (Based on NEXTCHIP driver for nvp)
 *
 * Copyright 2011 by NEXTCHIP Co., Ltd.
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
**************************************************************************
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include "nvp6124_reg.h"
#include "video.h"
#include "coax_protocol.h"

#define chip_id NVP6124B_R0_ID

/* nvp6124 1080P 色彩推荐配置 */
#define BRI_CENTER_VAL_NTSC 0xF4
#define BRI_CENTER_VAL_PAL  0xF4
#define CON_CENTER_VAL_NTSC 0x90
#define CON_CENTER_VAL_PAL  0x90
#define SAT_CENTER_VAL_NTSC 0x80
#define SAT_CENTER_VAL_PAL  0x80
#define HUE_CENTER_VAL_NTSC 0x00
#define HUE_CENTER_VAL_PAL  0x00


/* nvp6124 720P 色彩推荐配置 */
#define BRI_CENTER_VAL_NTSC_720P 0x08
#define BRI_CENTER_VAL_PAL_720P  0x08
#define CON_CENTER_VAL_NTSC_720P 0x88
#define CON_CENTER_VAL_PAL_720P  0x88
#define SAT_CENTER_VAL_NTSC_720P 0x90
#define SAT_CENTER_VAL_PAL_720P  0x90
#define HUE_CENTER_VAL_NTSC_720P 0xFD
#define HUE_CENTER_VAL_PAL_720P  0x00

/* nvp6124 960H 色彩推荐配置 */
#define BRI_CENTER_VAL_NTSC_960H 0xFF
#define BRI_CENTER_VAL_PAL_960H  0x00
#define CON_CENTER_VAL_NTSC_960H 0x92
#define CON_CENTER_VAL_PAL_960H  0x90
#define SAT_CENTER_VAL_NTSC_960H 0x88
#define SAT_CENTER_VAL_PAL_960H  0x88
#define HUE_CENTER_VAL_NTSC_960H 0x01
#define HUE_CENTER_VAL_PAL_960H  0x00

#define WRITE_I2C_SPEED (200*1000)
#define I2C_SPEED  (200*1000)

static unsigned int nvp6124_con_tbl[2]  = {
	CON_CENTER_VAL_NTSC,
	CON_CENTER_VAL_PAL
};
static unsigned int nvp6124_hue_tbl[2]  = {
	HUE_CENTER_VAL_NTSC,
	HUE_CENTER_VAL_PAL
};
static unsigned int nvp6124_sat_tbl[2]  = {
	SAT_CENTER_VAL_NTSC,
	SAT_CENTER_VAL_PAL
};
static unsigned int nvp6124_bri_tbl[2]  = {
	BRI_CENTER_VAL_NTSC,
	BRI_CENTER_VAL_PAL
};
static unsigned int nvp6124_con_tbl_720P[2]  = {
	CON_CENTER_VAL_NTSC_720P,
	CON_CENTER_VAL_PAL_720P
};
static unsigned int nvp6124_hue_tbl_720P[2]  = {
	HUE_CENTER_VAL_NTSC_720P,
	HUE_CENTER_VAL_PAL_720P
};
static unsigned int nvp6124_sat_tbl_720P[2]  = {
	SAT_CENTER_VAL_NTSC_720P,
	SAT_CENTER_VAL_PAL_720P
};

static unsigned int nvp6124_bri_tbl_720P[2]  = {
	BRI_CENTER_VAL_NTSC_720P,
	BRI_CENTER_VAL_PAL_720P
};
static unsigned int nvp6124_con_tbl_960H[2]  = {
	CON_CENTER_VAL_NTSC_960H,
	CON_CENTER_VAL_PAL_960H
};
static unsigned int nvp6124_hue_tbl_960H[2]  = {
	HUE_CENTER_VAL_NTSC_960H,
	HUE_CENTER_VAL_PAL_960H
};
static unsigned int nvp6124_sat_tbl_960H[2]  = {
	SAT_CENTER_VAL_NTSC_960H,
	SAT_CENTER_VAL_PAL_960H
};
static unsigned int nvp6124_bri_tbl_960H[2]  = {
	BRI_CENTER_VAL_NTSC_960H,
	BRI_CENTER_VAL_PAL_960H
};

static unsigned char ch_mode_status = 0xff;
static unsigned char ch_vfmt_status = 0xff;

static struct i2c_client *client;

static int i2c_master_normal_send(
	const struct i2c_client *client,
	const char *buf,
	int count,
	int scl_rate)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = count;
	msg.buf = (char *)buf;
	msg.scl_rate = scl_rate;
	ret = i2c_transfer(adap, &msg, 1);
	return (ret == 1) ? count : ret;
}

/*
static char i2c_master_normal_recv(
	const struct i2c_client *client,
	char *buf,
	int count,
	int scl_rate)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret, val;

	msg.addr = client->addr;
	msg.flags = client->flags | I2C_M_RD;
	msg.len = count;
	msg.buf = (char *)buf;
	msg.scl_rate = scl_rate;

	ret = i2c_transfer(adap, &msg, 1);

	val = *(msg.buf);
	return (*(msg.buf));
}
*/

static int nvp6124_write_table(
	u8 addr,
	const u8 *pdata,
	int datalen)
{
	int ret = 0;
	u8 tmp_buf[256];
	unsigned int bytelen;

	bytelen = 0;

	if (datalen > 256) {
		pr_info("%s too big datalen = %d!\n",
			__func__,
			datalen);
		return -1;
	}

	tmp_buf[0] = addr;
	bytelen++;
	if (datalen != 0 && pdata != NULL) {
		memcpy(&tmp_buf[bytelen], pdata, datalen);
		bytelen += datalen;
	}
	ret = i2c_master_normal_send(client,
				     tmp_buf,
				     bytelen,
				     I2C_SPEED);
	return ret;
}

static unsigned char nvp_i2c_read(
	unsigned char subaddr)
{
	u8 buf[1];
	struct i2c_msg msg = {client->addr, 0, 1, buf};
	int ret;

	buf[0] = subaddr;
	pr_info("nvp_i2c_read: %x, %x\n", client->addr, subaddr);
	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) {
		pr_info("i2c transfer error\n");
		return 0;
	}

	msg.flags = I2C_M_RD;
	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;

	return buf[0];
}

static u32 nvp_i2c_write(u8 reg, u8 data)
{
	struct i2c_msg xfer_msg[1];
	int num = 1;
	char buf[2];
	int ret;

	buf[0] = reg;
	buf[1] = data;

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags;
	xfer_msg[0].buf = buf;
	xfer_msg[0].scl_rate = WRITE_I2C_SPEED;

	ret = i2c_transfer(client->adapter, xfer_msg, 1);
	return ret;
}

int nvp6124b_check_id(struct nvp *nvp)
{
	int ret = 0;
	u8 id = 0;

	client = nvp->client;
	nvp_i2c_write(0xFF, 0x00);   /* choose bank 0 */
	id = nvp_i2c_read(0xf4);    /* read id */
	if (id != chip_id) {
		ret = -1;
		pr_err("nvp i2c id read fail! exp %#x real %x\n",
		       chip_id,
		       id);
	}
	return ret;
}

static void nvp6124b_reg_check(u8 regaddr, u8 data)
{
	u8 tmp1 = 0;
	u8 tmp2 = 0;
	int ret = 0;

	ret = nvp_i2c_write(0xFF, 0x00);   /* choose bank 0 */
	if (ret < 0) {
		dev_err(&client->dev,
			"%s: i2c write error:%d\n",
			__func__,
			ret);
	}
	tmp1 = nvp_i2c_read(regaddr);
	ret = nvp_i2c_write(regaddr, data);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s: i2c write error:%d\n",
			__func__,
			ret);
	}
	tmp2 = nvp_i2c_read(regaddr);
	pr_info("NVP6124B reg_0x%x value: init=0x%x,exp=0x%x,real=0x%x\n",
		regaddr,
		tmp1,
		data,
		tmp2);

	if (tmp2 == data)
		pr_info("NVP6124B I2C TEST PASS!\n");
	else
		pr_err("NVP6124B I2C TEST FAIL!\n");
}

void nvp6124b_common_init(struct nvp *nvp)
{
	unsigned char tmpclk1 = 0, tmpclk2 = 0;

	client = nvp->client;

	nvp6124b_reg_check(0x0d, 0x08);
	nvp_i2c_write(0xFF, 0x00);
	nvp6124_write_table(0x00, NVP6124_B0_Buf, 254);
	nvp_i2c_write(0xFF, 0x01);
	nvp6124_write_table(0x00, NVP6124_B1_Buf, 254);
	nvp_i2c_write(0xFF, 0x02);
	nvp6124_write_table(0x00, NVP6124_B2_Buf, 254);
	nvp_i2c_write(0xFF, 0x03);
	nvp6124_write_table(0x00, NVP6124_B3_Buf, 254);
	nvp_i2c_write(0xFF, 0x04);
	nvp6124_write_table(0x00, NVP6124_B4_Buf, 254);
	nvp_i2c_write(0xFF, 0x09);
	nvp6124_write_table(0x00, NVP6124_B9_Buf, 254);
	nvp_i2c_write(0xFF, 0x0a);
	nvp6124_write_table(0x00, NVP6124_BA_Buf, 254);
	nvp_i2c_write(0xFF, 0x0b);
	nvp6124_write_table(0x00, NVP6124_BB_Buf, 254);

	/* system_init */
	nvp_i2c_write(0xFF, 0x01);
	nvp_i2c_write(0x82, 0x12);
	nvp_i2c_write(0x83, 0x2C);
	nvp_i2c_write(0x3e, 0x10);
	nvp_i2c_write(0x80, 0x60);
	nvp_i2c_write(0x80, 0x61);
	mdelay(1000);
	nvp_i2c_write(0x80, 0x40);
	nvp_i2c_write(0x81, 0x02);
	nvp_i2c_write(0x97, 0x00);
	mdelay(10);
	nvp_i2c_write(0x80, 0x60);
	nvp_i2c_write(0x81, 0x00);
	mdelay(10);
	nvp_i2c_write(0x97, 0x0F);
	nvp_i2c_write(0x38, 0x18);
	nvp_i2c_write(0x38, 0x08);
	mdelay(10);
	mdelay(100);
	nvp_i2c_write(0xCA, 0xAE);

	/* for test clock start */
	tmpclk1 = nvp_i2c_read(0xCD);
	tmpclk2 = nvp_i2c_read(0xCF);
	pr_info("before test clk1: 0x%x, clk2: 0x%x\n",
		tmpclk1,
		tmpclk2);
	nvp_i2c_write(0xFF, 0x01);
	nvp_i2c_write(0xCD, 0x40);
	nvp_i2c_write(0xCF, 0x40);
	tmpclk1 = nvp_i2c_read(0xCD);
	tmpclk2 = nvp_i2c_read(0xCF);
	pr_info("test clk1: 0x%x, clk2: 0x%x\n",
		tmpclk1,
		tmpclk2);
}

static void nvp6124_set_fpc(
	unsigned char ch,
	unsigned char vformat)
{
	unsigned char tmp;

	nvp_i2c_write(0xFF, 0x00);
	tmp = nvp_i2c_read(0x54);
	if (vformat == PAL)
		tmp &= ~(0x10 << ch);
	else
		tmp |= (0x10 << ch);
	nvp_i2c_write(0x54, tmp);
}

static void nvp6124b_syncchange(
	unsigned char ch,
	unsigned char vformat,
	unsigned char chmode)
{
	if (vformat == NTSC) {
		if (chmode == NVP6124_VI_1080P_2530) {
			nvp_i2c_write(0xff, 0x00);
			nvp_i2c_write(0x23 + ch * 4, 0x43);
			nvp_i2c_write(0x58 + ch, 0x4E);
			nvp_i2c_write(0x8E + ch, 0x09);

			nvp_i2c_write(0xff, 0x05 + ch);
			nvp_i2c_write(0x24, 0x1A);
			nvp_i2c_write(0x47, 0xEE);
			nvp_i2c_write(0x50, 0xC6);
			nvp_i2c_write(0xBB, 0x04);
		} else if (chmode  == NVP6124_VI_720P_5060) {
			nvp_i2c_write(0xff, 0x00);
			nvp_i2c_write(0x23 + ch * 4, 0x43);
			nvp_i2c_write(0x58 + ch, 0xa8);
			nvp_i2c_write(0x8E + ch, 0x09);

			nvp_i2c_write(0xff, 0x05 + ch);
			nvp_i2c_write(0x24, 0x2A);
			nvp_i2c_write(0x47, 0xEE);
			nvp_i2c_write(0x50, 0xC6);
			nvp_i2c_write(0xBB, 0x04);
		} else {
			nvp_i2c_write(0xff, 0x00);
			nvp_i2c_write(0x23 + ch * 4, 0x43);
			nvp_i2c_write(0xff, 0x05 + ch);
			nvp_i2c_write(0x47, 0x04);
		}
	} else {
		nvp_i2c_write(0xff, 0x00);
		nvp_i2c_write(0x23 + ch * 4, 0x43);
		nvp_i2c_write(0xff, 0x05 + ch);
		nvp_i2c_write(0x47, 0x04);
	}
}

void acp_reg_rx_clear(unsigned char ch)
{
	nvp_i2c_write(0xFF, 0x03 + ch / 2);
	nvp_i2c_write(ACP_CLR+((ch % 2) * 0x80), 0x01);
	udelay(10);
	nvp_i2c_write(ACP_CLR+((ch % 2) * 0x80), 0x00);
	udelay(100);
}

void init_acp(unsigned char ch)
{
	nvp_i2c_write(0xFF, 0x01);
	/* Decoder TX Setting */
	if (chip_id == NVP6124_R0_ID) {
		nvp_i2c_write(0xBD, 0xD0);
		nvp_i2c_write(0xBE, 0xDD);
		nvp_i2c_write(0xBF, 0x0D);
	} else if (chip_id == NVP6114A_R0_ID) {
		nvp_i2c_write(0xBC, 0xDD);
		nvp_i2c_write(0xBD, 0xED);
	} else if (chip_id == NVP6124B_R0_ID) {
		nvp_i2c_write(0xBC, 0xF7);
		nvp_i2c_write(0xBD, 0xF7);
		nvp_i2c_write(0xA8, 0xA5);
	}

	acp_reg_rx_clear(ch);
}

void acp_each_setting(unsigned char ch)
{
	init_acp(ch);

	nvp_i2c_write(0xFF, 0x03 + ch / 2);
	if (ch_mode_status == NVP6124_VI_720P_2530 ||
	    ch_mode_status == NVP6124_VI_HDEX) {
		nvp_i2c_write(AHD2_FHD_BAUD + ((ch % 2) * 0x80), 0x0E);
		nvp_i2c_write(AHD2_FHD_LINE + ((ch % 2) * 0x80),
			      ch_vfmt_status == PAL ? 0x0D : 0x0E);
	} else if (ch_mode_status ==
		  NVP6124_VI_1080P_2530) {
		nvp_i2c_write(AHD2_FHD_BAUD+((ch%2)*0x80), 0x0E);
		nvp_i2c_write(AHD2_FHD_LINE+((ch%2)*0x80), 0x0E);
	}
	nvp_i2c_write(AHD2_PEL_SYNC + ((ch % 2) * 0x80), 0x14);
	nvp_i2c_write(AHD2_PEL_SYNC + ((ch % 2) * 0x80) + 1, 0x00);
	nvp_i2c_write(AHD2_FHD_LINES + ((ch % 2) * 0x80), 0x07);
	nvp_i2c_write(AHD2_FHD_BYTE + ((ch % 2) * 0x80), 0x03);
	nvp_i2c_write(AHD2_FHD_MODE + ((ch % 2) * 0x80), 0x10);
	nvp_i2c_write(AHD2_PEL_EVEN + ((ch % 2) * 0x80), 0x00);

	/* Decoder RX Setting */
	nvp_i2c_write(0xFF, 0x05 + ch);
	nvp_i2c_write(0x30, 0x00);
	nvp_i2c_write(0x31, 0x01);
	nvp_i2c_write(0x32, 0x64);
	nvp_i2c_write(0x7C, 0x11);
	if (ch_mode_status == NVP6124_VI_720P_2530 ||
	    ch_mode_status == NVP6124_VI_HDEX)
		nvp_i2c_write(0x7D, 0x80);
	else if (ch_mode_status == NVP6124_VI_1080P_2530)
		nvp_i2c_write(0x7D, 0x80);
	nvp_i2c_write(0xFF, 0x03 + ch / 2);

	if (ch_mode_status == NVP6124_VI_720P_2530 ||
	    ch_mode_status == NVP6124_VI_HDEX) {
		/* Camera TX DATA Check */
		nvp_i2c_write(0x62 + ((ch % 2) * 0x80),
			      ch_vfmt_status == PAL ?
			      0x05 : 0x06);
		/* RX size */
		nvp_i2c_write(0x68 + ((ch % 2) * 0x80), 0x40);
	} else if (ch_mode_status == NVP6124_VI_1080P_2530) {
		/* Camera TX DATA Check */
		nvp_i2c_write(0x62+((ch % 2) * 0x80), 0x06);
		/* RX size */
		nvp_i2c_write(0x68+((ch % 2) * 0x80), 0x70);
	}

	nvp_i2c_write(0x60 + ((ch % 2) * 0x80), 0x55);
	nvp_i2c_write(0x63 + ((ch % 2) * 0x80), 0x01);
	nvp_i2c_write(0x66 + ((ch % 2) * 0x80), 0x80);
	nvp_i2c_write(0x67 + ((ch % 2) * 0x80), 0x01);
}

void nvp6124_each_mode_setting(
	struct nvp *nvp,
	unsigned char ch,
	unsigned char vformat,
	enum nvp_mode chmode)
{
	unsigned char tmp;
	unsigned char pn_value_sd_nt_comet[4] = {
		0x4D, 0x0E, 0x88, 0x6C
	};
	unsigned char pn_value_720p_30[4] = {
		0xEE, 0x00, 0xE5, 0x4E};
	unsigned char pn_value_720p_60[4] = {
		0x2C, 0xF9, 0xC5, 0x52
	};
	unsigned char pn_value_fhd_nt[4] = {
		0x2C, 0xF0, 0xCA, 0x52
	};
	unsigned char pn_value_sd_pal_comet[4] = {
		0x75, 0x35, 0xB4, 0x6C
	};
	unsigned char pn_value_720p_25[4] = {
		0x46, 0x08, 0x10, 0x4F
	};
	unsigned char pn_value_720p_50[4] = {
		0x2C, 0xE7, 0xCF, 0x52
	};
	unsigned char pn_value_fhd_pal[4] = {
		0xC8, 0x7D, 0xC3, 0x52
	};

	if (chmode < NVP6124_VI_BUTT) {
		switch (chmode) {
		case NVP6124_VI_SD:
		case NVP6124_VI_1920H:
		case NVP6124_VI_720H:
		case NVP6124_VI_1280H:
		case NVP6124_VI_1440H:
		case NVP6124_VI_960H2EX:
			nvp_i2c_write(0xFF, 0x00);
			nvp_i2c_write(0x08 + ch,
				      vformat == PAL ? 0xDD : 0xA0);
			nvp_i2c_write(0x0c + ch,
				      nvp6124_bri_tbl_960H[vformat % 2]);
			nvp_i2c_write(0x10 + ch,
				      nvp6124_con_tbl_960H[vformat % 2]);
			nvp_i2c_write(0x14 + ch,
				      vformat == PAL ? 0x80 : 0x80);
			nvp_i2c_write(0x18 + ch,
				      vformat == PAL ? 0x18 : 0x18);
			nvp_i2c_write(0x21 + 4 * ch,
				      vformat == PAL ? 0x02 : 0x82);
			nvp_i2c_write(0x22 + 4 * ch, 0x0B);
			nvp_i2c_write(0x23 + 4 * ch, 0x43);
			nvp_i2c_write(0x30 + ch,
				      vformat == PAL ? 0x12 : 0x11);
			nvp_i2c_write(0x3c + ch,
				      nvp6124_sat_tbl_960H[vformat % 2]);
			nvp_i2c_write(0x40 + ch,
				      nvp6124_hue_tbl_960H[vformat % 2]);
			nvp_i2c_write(0x44 + ch,
				      vformat == PAL ? 0x00 : 0x00);
			nvp_i2c_write(0x48 + ch,
				      vformat == PAL ? 0x00 : 0x00);
			nvp_i2c_write(0x4c + ch,
				      vformat == PAL ? 0x04 : 0x00);
			nvp_i2c_write(0x50 + ch,
				      vformat == PAL ? 0x04 : 0x00);
			nvp6124_set_fpc(ch, vformat);
			if (chmode == NVP6124_VI_SD) {
				if (chip_id == NVP6124B_R0_ID)
					nvp_i2c_write(0x58 + ch,
						      vformat == PAL ?
						      0x60 : 0xC0);
				else
					nvp_i2c_write(0x58 + ch,
						      vformat == PAL ?
						      0x80 : 0x90);
			} else if (chmode == NVP6124_VI_960H2EX) {
				nvp_i2c_write(0x58 + ch,
					      vformat == PAL ? 0x80 : 0x90);
			} else if (chmode == NVP6124_VI_720H) {
				if (chip_id == NVP6124B_R0_ID)
					nvp_i2c_write(0x58 + ch, 0x70);
				else
					nvp_i2c_write(0x58 + ch,
						      vformat == PAL ?
						      0xB0 : 0x40);
			} else if (chmode == NVP6124_VI_1280H) {
				if (chip_id == NVP6124B_R0_ID)
					nvp_i2c_write(0x58 + ch, 0x50);
				else
					nvp_i2c_write(0x58 + ch,
						      vformat == PAL ?
						      0x80 : 0x90);
			} else if (chmode == NVP6124_VI_1440H) {
				nvp_i2c_write(0x58 + ch,
					      vformat == PAL ? 0x90 : 0xA0);
			} else	{
				if (chip_id == NVP6124B_R0_ID)
					nvp_i2c_write(0x58 + ch,
						      vformat == PAL ?
						      0x88 : 0x98);
				else
					nvp_i2c_write(0x58 + ch,
						      vformat == PAL ?
						      0x3B : 0x4B);
			}
			nvp_i2c_write(0x5c + ch, 0x1e);
			nvp_i2c_write(0x64 + ch,
				      vformat == PAL ? 0x0d : 0x08);
			if (chip_id == NVP6124B_R0_ID) {
				if (chmode == NVP6124_VI_SD ||
				    chmode == NVP6124_VI_720H) {
					nvp_i2c_write(0x81 + ch, 0x00);
					nvp_i2c_write(0x85 + ch, 0x11);
					if (chmode == NVP6124_VI_720H) {
						pr_info("ch %d setted to 720H %s\n",
							ch,
							vformat == PAL ?
							"PAL" : "NTSC");
						nvp_i2c_write(0x89 + ch, 0x11);
						nvp_i2c_write(0x8E + ch, 0x00);
					} else {
						pr_info("ch %d setted to 960H %s\n",
							ch,
							vformat == PAL ?
							"PAL" : "NTSC");
						nvp_i2c_write(0x89 + ch,
							      vformat == PAL ?
							      0x10 : 0x00);
						nvp_i2c_write(0x8E + ch,
							      vformat == PAL ?
							      0x08 : 0x07);
					}
				} else if (chmode == NVP6124_VI_1280H) {
					pr_info("ch %d setted to 1280H %s\n",
						ch,
						vformat == PAL ?
						"PAL" : "NTSC");
					nvp_i2c_write(0x81 + ch, 0x20);
					nvp_i2c_write(0x85 + ch, 0x00);
					nvp_i2c_write(0x89 + ch, 0x10);
					nvp_i2c_write(0x8E + ch, 0x20);
				} else if (chmode == NVP6124_VI_1440H) {
					pr_info("ch %d setted to 1440H %s\n",
						ch,
						vformat == PAL ?
						"PAL" : "NTSC");
					nvp_i2c_write(0x81 + ch, 0x30);
					nvp_i2c_write(0x85 + ch, 0x00);
					nvp_i2c_write(0x89 + ch, 0x10);
					nvp_i2c_write(0x8E + ch, 0x10);
				} else if (chmode == NVP6124_VI_960H2EX) {
					pr_info("ch %d setted to 3840H %s\n",
						ch,
						vformat == PAL ?
						"PAL" : "NTSC");
					nvp_i2c_write(0x81 + ch, 0x00);
					nvp_i2c_write(0x85 + ch, 0x11);
					nvp_i2c_write(0x89 + ch, 0x10);
					nvp_i2c_write(0x8E + ch, 0x22);
				} else {
					pr_info("ch %d setted to 1920H %s\n",
						ch,
						vformat == PAL ?
						"PAL" : "NTSC");
					nvp_i2c_write(0x81 + ch, 0x40);
					nvp_i2c_write(0x85 + ch, 0x11);
					nvp_i2c_write(0x89 + ch, 0x00);
					nvp_i2c_write(0x8E + ch, 0x10);
				}
			} else {
				if (chmode == NVP6124_VI_SD ||
				    chmode == NVP6124_VI_720H) {
					nvp_i2c_write(0x81 + ch, 0x00);
					nvp_i2c_write(0x85 + ch, 0x11);
					if (chmode == NVP6124_VI_720H) {
						pr_info("ch %d setted to 720H %s\n",
							ch,
							vformat == PAL ?
							"PAL" : "NTSC");
						nvp_i2c_write(0x89 + ch, 0x11);
						nvp_i2c_write(0x8E + ch,
							      vformat == PAL ?
							      0x20 : 0x00);
					} else {
						pr_info("ch %d setted to 960H %s\n",
							ch,
							vformat == PAL ?
							"PAL" : "NTSC");
						nvp_i2c_write(0x89 + ch,
							      vformat == PAL ?
							      0x10 : 0x00);
						nvp_i2c_write(0x8E + ch,
							      vformat == PAL ?
							      0x08 : 0x07);
					}
				} else if (chmode == NVP6124_VI_1280H) {
					pr_info("ch %d setted to 1280H %s\n",
						ch,
						vformat == PAL ?
						"PAL" : "NTSC");
					nvp_i2c_write(0x81 + ch, 0x20);
					nvp_i2c_write(0x85 + ch, 0x00);
					nvp_i2c_write(0x89 + ch, 0x10);
					nvp_i2c_write(0x8E + ch, 0x17);
				} else if (chmode == NVP6124_VI_1440H) {
					pr_info("ch %d setted to 1440H %s\n",
						ch,
						vformat == PAL ?
						"PAL" : "NTSC");
					nvp_i2c_write(0x81 + ch, 0x30);
					nvp_i2c_write(0x85 + ch, 0x00);
					nvp_i2c_write(0x89 + ch, 0x10);
					nvp_i2c_write(0x8E + ch, 0x0B);
				} else {
					pr_info("ch %d setted to 1920H %s\n",
						ch,
						vformat == PAL ?
						"PAL" : "NTSC");
					nvp_i2c_write(0x81 + ch, 0x40);
					nvp_i2c_write(0x85 + ch, 0x00);
					nvp_i2c_write(0x89 + ch, 0x00);
					nvp_i2c_write(0x8E + ch, 0x10);
				}
			}
			nvp_i2c_write(0x93 + ch, 0x00);
			nvp_i2c_write(0x98 + ch,
				      vformat == PAL ? 0x07 : 0x04);
			if (chip_id == NVP6124B_R0_ID) {
				if (chmode == NVP6124_VI_960H2EX)
					nvp_i2c_write(0xa0 + ch,
						      vformat == PAL ?
						      0x16 : 0x15);
				else
					nvp_i2c_write(0xa0 + ch, 0x15);
				nvp_i2c_write(0xa4 + ch,
					      vformat == PAL ? 0x05 : 0x07);
			} else {
				nvp_i2c_write(0xa0 + ch,
					      vformat == PAL ? 0x00 : 0x10);
				nvp_i2c_write(0xa4 + ch,
					      vformat == PAL ? 0x00 : 0x01);
			}
			nvp_i2c_write(0xFF, 0x01);
			if (chip_id == NVP6124B_R0_ID) {
				if (chmode == NVP6124_VI_SD ||
				    chmode == NVP6124_VI_720H) {
					nvp_i2c_write(0x88 + ch, 0x06);
					nvp_i2c_write(0x8c + ch, 0xa6);
					nvp_i2c_write(0xd7,
						      nvp_i2c_read(0xd7) &
						      (~(1 << ch)));
				} else if (chmode == NVP6124_VI_1280H) {
					nvp_i2c_write(0x88 + ch, 0x06);
					nvp_i2c_write(0x8c + ch, 0x26);
					nvp_i2c_write(0xd7,
						      nvp_i2c_read(0xd7) |
						      (1 << ch));
				} else if (chmode == NVP6124_VI_1440H) {
					nvp_i2c_write(0x88 + ch, 0x06);
					nvp_i2c_write(0x8c + ch, 0x26);
					nvp_i2c_write(0xd7,
						      nvp_i2c_read(0xd7) |
						      (1 << ch));
				} else if (chmode == NVP6124_VI_960H2EX) {
					nvp_i2c_write(0x88 + ch, 0x06);
					nvp_i2c_write(0x8c + ch, 0x46);
					nvp_i2c_write(0xd7,
						      nvp_i2c_read(0xd7) |
						      (1 << ch));
				} else {
					nvp_i2c_write(0x88 + ch, 0x06);
					nvp_i2c_write(0x8c + ch, 0x06);
					nvp_i2c_write(0xd7,
						      nvp_i2c_read(0xd7) |
						      (1 << ch));
				}
			} else {
				if (chmode == NVP6124_VI_SD ||
				    chmode == NVP6124_VI_720H) {
					nvp_i2c_write(0x88 + ch, 0x7e);
					nvp_i2c_write(0x8c + ch, 0x26);
					nvp_i2c_write(0xd7,
						      nvp_i2c_read(0xd7) &
						      (~(1 << ch)));
				} else if (chmode == NVP6124_VI_1280H) {
					nvp_i2c_write(0x88 + ch, 0x7e);
					nvp_i2c_write(0x8c + ch, 0x56);
					nvp_i2c_write(0xd7,
						      nvp_i2c_read(0xd7) |
						      (1 << ch));
				} else if (chmode == NVP6124_VI_1440H) {
					nvp_i2c_write(0x88 + ch, 0x7e);
					nvp_i2c_write(0x8c + ch, 0x56);
					nvp_i2c_write(0xd7,
						      nvp_i2c_read(0xd7) |
						      (1 << ch));
				} else {
					/* if(chmode==NVP6124_VI_1920H) */
					nvp_i2c_write(0x88 + ch, 0x46);
					nvp_i2c_write(0x8c + ch, 0x47);
					nvp_i2c_write(0xd7,
						      nvp_i2c_read(0xd7) |
						      (1 << ch));
				}
			}
			nvp_i2c_write(0xFF, 0x02);
			tmp = nvp_i2c_read(0x16 + ch / 2);
			nvp_i2c_write(0x16 + ch/2,
				      (tmp & (ch % 2 == 0 ? 0xF0 : 0x0F)) |
				      (0x00 << ((ch % 2) * 4)));
			nvp_i2c_write(0xFF, 0x05 + ch);
			nvp6124_write_table(0x00,
					    NVP6124_B5678_SD_Buf,
					    254);
			nvp_i2c_write(0x06, 0x40);
			nvp_i2c_write(0x0F,
				      vformat == PAL ? 0x13 : 0x00);
			nvp_i2c_write(0x1B, 0x08);
			nvp_i2c_write(0x20, 0x88);
			nvp_i2c_write(0x1E,
				      vformat == PAL ? 0x00 : 0x01);
			nvp_i2c_write(0x2C,
				      vformat == PAL ? 0x08 : 0x0C);
			nvp_i2c_write(0x35,
				      vformat == PAL ? 0x17 : 0x15);
			if (chip_id == NVP6124B_R0_ID)
				nvp_i2c_write(0x62, 0x00);
			else
				nvp_i2c_write(0x62,
					      vformat == PAL ? 0x20 : 0x00);
			nvp_i2c_write(0xA1,
				      vformat == PAL ? 0x10 : 0x30);
			nvp_i2c_write(0xA2,
				      vformat == PAL ? 0x0E : 0x0C);
			nvp_i2c_write(0xA3,
				      vformat == PAL ? 0x70 : 0x50);
			nvp_i2c_write(0xA8,
				      vformat == PAL ? 0x40 : 0x20);
			nvp_i2c_write(0xAC,
				      vformat == PAL ? 0x10 : 0x20);
			nvp_i2c_write(0xAD,
				      vformat == PAL ? 0x08 : 0x20);
			nvp_i2c_write(0xAE,
				      vformat == PAL ? 0x04 : 0x14);
			nvp_i2c_write(0xC0,
				      vformat == PAL ? 0x0D : 0x00);
			nvp_i2c_write(0x25,
				      vformat == PAL ? 0xCA : 0xDA);
			if (chmode == NVP6124_VI_1280H ||
			    chmode == NVP6124_VI_1440H) {
				nvp_i2c_write(0x62, 0x20);
				nvp_i2c_write(0x64, 0x0D);
			} else if (chmode == NVP6124_VI_960H2EX) {
				nvp_i2c_write(0x6B, 0x10);
			}
			nvp_i2c_write(0xFF, 0x09);
			nvp_i2c_write(0x40 + ch, 0x60);
			nvp_i2c_write(0x44,
				      nvp_i2c_read(0x44) & (~(1 << ch)));
			nvp_i2c_write(0x50 + ch * 4,
				      vformat == PAL ?
				      pn_value_sd_pal_comet[0] :
				      pn_value_sd_nt_comet[0]);
			nvp_i2c_write(0x51 + ch * 4,
				      vformat == PAL ?
				      pn_value_sd_pal_comet[1] :
				      pn_value_sd_nt_comet[1]);
			nvp_i2c_write(0x52 + ch * 4,
				      vformat == PAL ?
				      pn_value_sd_pal_comet[2] :
				      pn_value_sd_nt_comet[2]);
			nvp_i2c_write(0x53 + ch * 4,
				      vformat == PAL ?
				      pn_value_sd_pal_comet[3] :
				      pn_value_sd_nt_comet[3]);
			break;
		case NVP6124_VI_720P_2530:
		case NVP6124_VI_HDEX:
			nvp_i2c_write(0xFF, 0x00);
			nvp_i2c_write(0x08 + ch,
				      vformat == PAL ? 0x60 : 0x60);
			nvp_i2c_write(0x0c + ch,
				      nvp6124_bri_tbl_720P[vformat % 2]);
			nvp_i2c_write(0x10 + ch,
				      nvp6124_con_tbl_720P[vformat % 2]);
			nvp_i2c_write(0x14 + ch,
				      vformat == PAL ? 0x90 : 0x90);
			nvp_i2c_write(0x18 + ch,
				      vformat == PAL ? 0x30 : 0x30);
			nvp_i2c_write(0x21 + ch * 4, 0x92);
			nvp_i2c_write(0x22 + ch * 4, 0x0A);
			nvp_i2c_write(0x23 + ch * 4, 0x43);
			if (chip_id == NVP6124B_R0_ID)
				nvp_i2c_write(0x30 + ch, 0x15);
			else
				nvp_i2c_write(0x30 + ch, 0x12);
			nvp_i2c_write(0x3c + ch,
				      nvp6124_sat_tbl_720P[vformat % 2]);
			nvp_i2c_write(0x40 + ch,
				      nvp6124_hue_tbl_720P[vformat % 2]);
			nvp_i2c_write(0x44 + ch,
				      vformat == PAL ? 0x30 : 0x30);
			nvp_i2c_write(0x48 + ch,
				      vformat == PAL ? 0x30 : 0x30);
			nvp_i2c_write(0x4c + ch,
				      vformat == PAL ? 0x04 : 0x04);
			nvp_i2c_write(0x50 + ch,
				      vformat == PAL ? 0x04 : 0x04);
			nvp6124_set_fpc(ch, vformat);
			nvp_i2c_write(0x58 + ch,
				      vformat == PAL ? 0x80 : 0x90);
			nvp_i2c_write(0x5c + ch,
				      vformat == PAL ? 0x9e : 0x9e);
			nvp_i2c_write(0x64 + ch,
				      vformat == PAL ? 0xb1 : 0xb2);
			nvp_i2c_write(0x78, 0x10);
			nvp_i2c_write(0x79, 0x32);
			nvp_i2c_write(0x81 + ch,
				      vformat == PAL ? 0x07 : 0x06);
			nvp_i2c_write(0x85 + ch,
				      vformat == PAL ? 0x00 : 0x00);
			nvp_i2c_write(0x89 + ch,
				      vformat == PAL ? 0x10 : 0x10);
			if (chip_id == NVP6124B_R0_ID)
				if (chmode == NVP6124_VI_HDEX)
					nvp_i2c_write(0x8E + ch, 0x07);
				else
					nvp_i2c_write(0x8E + ch, 0x0d);
			else
				nvp_i2c_write(0x8E + ch,
					      vformat == PAL ? 0x0d : 0x0d);
			nvp_i2c_write(0x93 + ch, 0x00);
			nvp_i2c_write(0x98 + ch,
				      vformat == PAL ? 0x07 : 0x04);
			if (chip_id == NVP6124B_R0_ID)
				nvp_i2c_write(0xa0 + ch, 0x15);
			else
				nvp_i2c_write(0xa0 + ch, 0x00);
			nvp_i2c_write(0xa4 + ch,
				      vformat == PAL ? 0x00 : 0x01);
			nvp_i2c_write(0xFF, 0x01);
			if (chip_id == NVP6124B_R0_ID) {
				nvp_i2c_write(0x88 + ch,
					      vformat == PAL ? 0x0A : 0x0A);

				if (chmode == NVP6124_VI_HDEX) {
					pr_info("ch %d setted to HDEX(2560x720) %s\n",
						ch,
						vformat == PAL ?
						"PAL" : "NTSC");
					nvp_i2c_write(0x8c + ch,
						      vformat == PAL ?
						      0x4A : 0x5A);
				} else {
					pr_info("ch %d setted to 720P %s\n",
						ch,
						vformat == PAL ?
						"PAL" : "NTSC");
					nvp_i2c_write(0x8c + ch, 0x0A);
				}

				nvp_i2c_write(0x9E, 0x55);
			} else {
				nvp_i2c_write(0x88 + ch, 0x5C);
				nvp_i2c_write(0x8c + ch, 0x40);
			}
			nvp_i2c_write(0xd7,
				      nvp_i2c_read(0xd7) | (1 << ch));
			nvp_i2c_write(0xFF, 0x02);
			tmp = nvp_i2c_read(0x16 + ch / 2);
			nvp_i2c_write(0x16 + ch / 2,
				      (tmp & (ch % 2 == 0 ? 0xF0 : 0x0F)) |
				      (0x05 << ((ch % 2) * 4)));
			nvp_i2c_write(0xFF, 0x05 + ch);
			nvp6124_write_table(0x00, NVP6124_B5678_Buf, 254);
			nvp_i2c_write(0x01, 0x0D);
			nvp_i2c_write(0x06, 0x40);
			nvp_i2c_write(0x1E,
				      vformat == PAL ? 0x00 : 0x01);
			nvp_i2c_write(0x35,
				      vformat == PAL ? 0x17 : 0x15);
			nvp_i2c_write(0x7A,
				      vformat == PAL ? 0x00 : 0x01);
			nvp_i2c_write(0x7B,
				      vformat == PAL ? 0x00 : 0x81);
			nvp_i2c_write(0xA1,
				      vformat == PAL ? 0x10 : 0x30);
			nvp_i2c_write(0xA2,
				      vformat == PAL ? 0x0E : 0x0C);
			nvp_i2c_write(0xA3,
				      vformat == PAL ? 0x70 : 0x50);
			nvp_i2c_write(0xA8,
				      vformat == PAL ? 0x40 : 0x20);
			nvp_i2c_write(0xAC,
				      vformat == PAL ? 0x10 : 0x20);
			nvp_i2c_write(0xAD,
				      vformat == PAL ? 0x08 : 0x20);
			nvp_i2c_write(0xAE,
				      vformat == PAL ? 0x04 : 0x14);
			nvp_i2c_write(0x25, 0xDB);
			nvp_i2c_write(0x2B, 0x78);
			nvp_i2c_write(0x59, 0x00);
			nvp_i2c_write(0x58, 0x13);
			if (chmode == NVP6124_VI_HDEX) {
				nvp_i2c_write(0x54, 0x20);
				nvp_i2c_write(0x55, 0x11);
				nvp_i2c_write(0x6B, 0x01);
			}
			nvp_i2c_write(0xC0, 0x16);
			if (chip_id == NVP6124B_R0_ID) {
				nvp_i2c_write(0xC1, 0x13);
				nvp_i2c_write(0xC8, 0x04);
			} else {
				nvp_i2c_write(0xC1, 0x14);
			}
			nvp_i2c_write(0xD8, 0x0C);
			nvp_i2c_write(0xD9, 0x0E);
			nvp_i2c_write(0xDA, 0x12);
			nvp_i2c_write(0xDB, 0x14);
			nvp_i2c_write(0xDC, 0x1C);
			nvp_i2c_write(0xDD, 0x2C);
			nvp_i2c_write(0xDE, 0x34);

			nvp_i2c_write(0xFF, 0x05 + ch % 4);
			nvp_i2c_write(0xFF, 0x09);
			nvp_i2c_write(0x40 + ch, 0x00);
			if (chip_id == NVP6124B_R0_ID)
				nvp_i2c_write(0x44, 0x00);
			else
				nvp_i2c_write(0x44,
					      nvp_i2c_read(0x44) | (1 << ch));
			nvp_i2c_write(0x50 + ch * 4,
				      vformat == PAL ? pn_value_720p_25[0] :
						       pn_value_720p_30[0]);
			nvp_i2c_write(0x51 + ch * 4,
				      vformat == PAL ? pn_value_720p_25[1] :
						       pn_value_720p_30[1]);
			nvp_i2c_write(0x52 + ch * 4,
				      vformat == PAL ? pn_value_720p_25[2] :
						       pn_value_720p_30[2]);
			nvp_i2c_write(0x53 + ch * 4,
				      vformat == PAL ? pn_value_720p_25[3] :
						       pn_value_720p_30[3]);
		break;
		case NVP6124_VI_720P_5060:
			nvp_i2c_write(0xFF, 0x00);
			nvp_i2c_write(0x08 + ch, 0x60);
			nvp_i2c_write(0x0c + ch,
				      nvp6124_bri_tbl_720P[vformat % 2]);
			nvp_i2c_write(0x10 + ch,
				      nvp6124_con_tbl_720P[vformat % 2]);
			nvp_i2c_write(0x14 + ch, 0x90);
			nvp_i2c_write(0x18 + ch, 0x00);
			nvp_i2c_write(0x21 + ch * 4, 0x92);
			nvp_i2c_write(0x22 + ch * 4, 0x0A);
			nvp_i2c_write(0x23 + ch * 4, 0x43);
			nvp_i2c_write(0x30 + ch, 0x12);
			nvp_i2c_write(0x3c + ch,
				      nvp6124_sat_tbl_720P[vformat % 2]);
			nvp_i2c_write(0x40 + ch,
				      nvp6124_hue_tbl_720P[vformat % 2]);
			nvp_i2c_write(0x44 + ch, 0x30);
			nvp_i2c_write(0x48 + ch, 0x30);
			nvp_i2c_write(0x4c + ch, 0x04);
			nvp_i2c_write(0x50 + ch, 0x04);
			nvp6124_set_fpc(ch, vformat);
			nvp_i2c_write(0x58 + ch,
				      vformat == PAL ? 0xc0 : 0xb0);
			nvp_i2c_write(0x5c + ch, 0x9e);
			nvp_i2c_write(0x64 + ch,
				      vformat == PAL ? 0xb1 : 0xb2);
			nvp_i2c_write(0x81 + ch,
				      vformat == PAL ? 0x05 : 0x04);
			nvp_i2c_write(0x85 + ch, 0x00);
			nvp_i2c_write(0x89 + ch, 0x10);
			nvp_i2c_write(0x8E + ch,
				      vformat == PAL ? 0x0b : 0x09);
			nvp_i2c_write(0x93 + ch, 0x00);
			nvp_i2c_write(0x98 + ch,
				      vformat == PAL ? 0x07 : 0x04);
			nvp_i2c_write(0xa0 + ch, 0x00);
			nvp_i2c_write(0xa4 + ch,
				      vformat == PAL ? 0x00 : 0x01);
			nvp_i2c_write(0xFF, 0x01);
			if (chip_id == NVP6124B_R0_ID) {
				nvp_i2c_write(0x88 + ch, 0x00);
				nvp_i2c_write(0x8c + ch, 0x42);
			} else {
				nvp_i2c_write(0x88 + ch, 0x4d);
				nvp_i2c_write(0x8c + ch, 0x84);
			}
			nvp_i2c_write(0xd7,
				      nvp_i2c_read(0xd7) | (1 << ch));
			nvp_i2c_write(0xFF, 0x02);
			tmp = nvp_i2c_read(0x16 + ch / 2);
			nvp_i2c_write(0x16 + ch / 2,
				      (tmp & ((ch % 2 == 0) ? 0xF0 : 0x0F)) |
				      (0x05 << ((ch % 2) * 4)));
			nvp_i2c_write(0xFF, 0x05 + ch);
			nvp6124_write_table(0x00,
					    NVP6124_B5678_Buf,
					    254);
			nvp_i2c_write(0x01, 0x0C);
			nvp_i2c_write(0x06, 0x40);
			nvp_i2c_write(0x1E,
				      vformat == PAL ? 0x00 : 0x01);
			nvp_i2c_write(0x35,
				      vformat == PAL ? 0x17 : 0x15);
			nvp_i2c_write(0x7A,
				      vformat == PAL ? 0x00 : 0x01);
			nvp_i2c_write(0x7B,
				      vformat == PAL ? 0x00 : 0x81);
			nvp_i2c_write(0xA1,
				      vformat == PAL ? 0x10 : 0x30);
			nvp_i2c_write(0xA2,
				      vformat == PAL ? 0x0E : 0x0C);
			nvp_i2c_write(0xA3,
				      vformat == PAL ? 0x70 : 0x50);
			nvp_i2c_write(0xA8,
				      vformat == PAL ? 0x40 : 0x20);
			nvp_i2c_write(0xAC,
				      vformat == PAL ? 0x10 : 0x20);
			nvp_i2c_write(0xAD,
				      vformat == PAL ? 0x08 : 0x20);
			nvp_i2c_write(0xAE,
				      vformat == PAL ? 0x04 : 0x14);
			nvp_i2c_write(0x2B, 0x78);
			nvp_i2c_write(0x58, 0x13);
			nvp_i2c_write(0x59, 0x01);
			nvp_i2c_write(0x24,
				      vformat == PAL ? 0x2A : 0x1A);
			nvp_i2c_write(0x50,
				      vformat == PAL ? 0x84 : 0x86);
			nvp_i2c_write(0xBB,
				      vformat == PAL ? 0x00 : 0xE4);
			nvp_i2c_write(0xD8, 0x0C);
			nvp_i2c_write(0xD9, 0x0E);
			nvp_i2c_write(0xDA, 0x12);
			nvp_i2c_write(0xDB, 0x14);
			nvp_i2c_write(0xDC, 0x1C);
			nvp_i2c_write(0xDD, 0x2C);
			nvp_i2c_write(0xDE, 0x34);
			nvp_i2c_write(0xFF, 0x09);
			nvp_i2c_write(0x40 + ch, 0x00);
			if (chip_id == NVP6124B_R0_ID)
				nvp_i2c_write(0x44, 0x00);
			else
				nvp_i2c_write(0x44,
					      nvp_i2c_read(0x44) | (1 << ch));
			nvp_i2c_write(0x50 + ch * 4,
				      vformat == PAL ? pn_value_720p_50[0] :
						       pn_value_720p_60[0]);
			nvp_i2c_write(0x51 + ch * 4,
				      vformat == PAL ? pn_value_720p_50[1] :
						       pn_value_720p_60[1]);
			nvp_i2c_write(0x52 + ch * 4,
				      vformat == PAL ? pn_value_720p_50[2] :
						       pn_value_720p_60[2]);
			nvp_i2c_write(0x53 + ch * 4,
				      vformat == PAL ? pn_value_720p_50[3] :
						       pn_value_720p_60[3]);
			pr_info("ch %d setted to 720P@RT %s\n",
				ch,
				vformat == PAL ? "PAL" : "NTSC");
			break;
		case NVP6124_VI_1080P_2530:
			nvp_i2c_write(0xFF, 0x00);
			nvp_i2c_write(0x08 + ch, 0x60);
			nvp_i2c_write(0x0c + ch,
				      nvp6124_bri_tbl[vformat % 2]);
			nvp_i2c_write(0x10 + ch,
				      nvp6124_con_tbl[vformat % 2]);
			nvp_i2c_write(0x14 + ch, 0x90);
			nvp_i2c_write(0x18 + ch, 0x00);
			nvp_i2c_write(0x21 + ch * 4, 0x92);
			if (chip_id == NVP6124B_R0_ID)
				nvp_i2c_write(0x22 + ch * 4, 0x0B);
			else
				nvp_i2c_write(0x22 + ch * 4, 0x0A);
			nvp_i2c_write(0x23 + ch * 4, 0x43);
			if (chip_id == NVP6124B_R0_ID)
				nvp_i2c_write(0x30 + ch, 0x15);
			else
				nvp_i2c_write(0x30 + ch, 0x12);
			nvp_i2c_write(0x3c + ch,
				      nvp6124_sat_tbl[vformat % 2]);
			nvp_i2c_write(0x40 + ch,
				      nvp6124_hue_tbl[vformat % 2]);
			nvp_i2c_write(0x44 + ch, 0x00);
			nvp_i2c_write(0x48 + ch, 0x00);
			nvp_i2c_write(0x4c + ch, 0x00);
			nvp_i2c_write(0x50 + ch, 0x00);
			nvp6124_set_fpc(ch, vformat);
			if (chip_id == NVP6124B_R0_ID)
				nvp_i2c_write(0x58 + ch,
					      vformat == PAL ? 0x78 : 0x57);
			else
				nvp_i2c_write(0x58 + ch,
					      vformat == PAL ? 0x6a : 0x49);
			nvp_i2c_write(0x5c + ch,
				      vformat == PAL ? 0x9e : 0x9e);
			nvp_i2c_write(0x64 + ch,
				      vformat == PAL ? 0xbf : 0x8d);
			nvp_i2c_write(0x78, 0x0e);
			nvp_i2c_write(0x79, 0x32);
			nvp_i2c_write(0x81 + ch,
				      vformat == PAL ? 0x03 : 0x02);
			nvp_i2c_write(0x85 + ch,
				      vformat == PAL ? 0x00 : 0x00);
			nvp_i2c_write(0x89 + ch,
				      vformat == PAL ? 0x10 : 0x10);
			nvp_i2c_write(0x8E + ch,
				      vformat == PAL ? 0x0a : 0x09);
			nvp_i2c_write(0x93 + ch, 0x00);
			nvp_i2c_write(0x98 + ch,
				      vformat == PAL ? 0x07 : 0x04);
			if (chip_id == NVP6124B_R0_ID)
				nvp_i2c_write(0xa0 + ch, 0x15);
			else
				nvp_i2c_write(0xa0 + ch, 0x00);
			nvp_i2c_write(0xa4 + ch,
				      vformat == PAL ? 0x00 : 0x01);
			nvp_i2c_write(0xFF, 0x01);

			if (chip_id == NVP6124B_R0_ID) {
				nvp_i2c_write(0x88 + ch,
					      vformat == PAL ? 0x00 : 0x00);
				nvp_i2c_write(0x8c + ch,
					      vformat == PAL ? 0x42 : 0x42);
			} else {
				nvp_i2c_write(0x88 + ch,
					      vformat == PAL ? 0x4c : 0x4c);
				nvp_i2c_write(0x8c + ch,
					      vformat == PAL ? 0x84 : 0x84);
			}
			nvp_i2c_write(0xd7, nvp_i2c_read(0xd7) | (1 << ch));
			nvp_i2c_write(0xFF, 0x02);
			tmp = nvp_i2c_read(0x16 + ch / 2);
			nvp_i2c_write(0x16 + ch / 2,
				      (tmp & (ch % 2 == 0 ? 0xF0 : 0x0F)) |
				      (0x05 << ((ch % 2) * 4)));
			nvp_i2c_write(0xFF, 0x05 + ch);
			nvp6124_write_table(0x00,
					    NVP6124_B5678_Buf,
					    254);
			nvp_i2c_write(0x01, 0x0C);
			nvp_i2c_write(0x06, 0x40);
			nvp_i2c_write(0x1E,
				      vformat == PAL ? 0x00 : 0x01);
			nvp_i2c_write(0x35,
				      vformat == PAL ? 0x17 : 0x15);
			nvp_i2c_write(0x7A,
				      vformat == PAL ? 0x00 : 0x01);
			nvp_i2c_write(0x7B,
				      vformat == PAL ? 0x00 : 0x81);
			nvp_i2c_write(0xA1,
				      vformat == PAL ? 0x10 : 0x30);
			nvp_i2c_write(0xA2,
				      vformat == PAL ? 0x0E : 0x0C);
			nvp_i2c_write(0xA3,
				      vformat == PAL ? 0x70 : 0x50);
			nvp_i2c_write(0xA8,
				      vformat == PAL ? 0x40 : 0x20);
			nvp_i2c_write(0xAC,
				      vformat == PAL ? 0x10 : 0x20);
			nvp_i2c_write(0xAD,
				      vformat == PAL ? 0x08 : 0x20);
			nvp_i2c_write(0xAE,
				      vformat == PAL ? 0x04 : 0x14);
			nvp_i2c_write(0x2A, 0x72);
			nvp_i2c_write(0x2B, 0xA8);
			nvp_i2c_write(0x58, 0x13);
			nvp_i2c_write(0x59, 0x01);
			if (chip_id == NVP6124B_R0_ID) {
				nvp_i2c_write(0xC0, 0x16);
				nvp_i2c_write(0xC1, 0x13);
				nvp_i2c_write(0xC8, 0x04);
				nvp_i2c_write(0x47, 0xEE);
				nvp_i2c_write(0x50, 0xC4);
			} else {
				nvp_i2c_write(0xC0, 0x17);
				nvp_i2c_write(0xC1, 0x14);
			}
			nvp_i2c_write(0xD8, 0x10);
			nvp_i2c_write(0xD9, 0x1F);
			nvp_i2c_write(0xDA, 0x2B);
			nvp_i2c_write(0xDB, 0x7F);
			nvp_i2c_write(0xDC, 0xFF);
			nvp_i2c_write(0xDD, 0xFF);
			nvp_i2c_write(0xDE, 0xFF);

			nvp_i2c_write(0x24,
				      vformat == PAL ? 0x2A : 0x1A);
			nvp_i2c_write(0x50,
				      vformat == PAL ? 0x84 : 0x86);
			nvp_i2c_write(0xBB,
				      vformat == PAL ? 0x00 : 0xE4);
			nvp_i2c_write(0xFF, 0x09);
			nvp_i2c_write(0x40 + ch, 0x00);
			if (chip_id == NVP6124B_R0_ID)
				nvp_i2c_write(0x44, 0x00);
			else
				nvp_i2c_write(0x44,
					      nvp_i2c_read(0x44) | (1 << ch));
			nvp_i2c_write(0x50 + ch * 4,
				      vformat == PAL ? pn_value_fhd_pal[0] :
						       pn_value_fhd_nt[0]);
			nvp_i2c_write(0x51 + ch * 4,
				      vformat == PAL ? pn_value_fhd_pal[1] :
						       pn_value_fhd_nt[1]);
			nvp_i2c_write(0x52 + ch * 4,
				      vformat == PAL ? pn_value_fhd_pal[2] :
						       pn_value_fhd_nt[2]);
			nvp_i2c_write(0x53 + ch * 4,
				      vformat == PAL ? pn_value_fhd_pal[3] :
						       pn_value_fhd_nt[3]);
			pr_info("ch %d setted to 1080P %s\n",
				ch,
				vformat == PAL ? "PAL" : "NTSC");
		break;
		default:
			pr_info("ch%d wrong mode detected!!!\n", ch);
			break;
		}
		ch_mode_status = chmode;
		ch_vfmt_status = vformat;
		if (chip_id == NVP6124B_R0_ID) {
			nvp6124b_syncchange(ch, vformat, chmode);
			nvp_i2c_write(0xFF, 0x09);
			nvp_i2c_write(0x64, 0x18);
			nvp_i2c_write(0x65, 0xC2);
			nvp_i2c_write(0x66, 0x01);
			nvp_i2c_write(0x67, 0x1E);
			nvp_i2c_write(0x68, 0x02);
			nvp_i2c_write(0x69, 0x64);
			nvp_i2c_write(0x6A, 0x60);
			nvp_i2c_write(0x6B, 0x3C);
		}
		acp_each_setting(ch);
	}
}


/*
portsel: port select[0,1];
portmode: port mode select[1mux,2mux,4mux]
chid:  channel id, 1mux[0,1,2,3], 2mux[0,1], 4mux[0]
NOTE:
portsel == 0,对应芯片硬件VDO1,寄存器描述VPORT_2;
portsel == 1,对应芯片硬件VDO2,寄存器描述VPORT_1;
*/
void nvp6124b_set_portmode(
	struct nvp *nvp,
	unsigned char portsel,
	unsigned char portmode,
	unsigned char chid)
{
	unsigned char tmp = 0, reg1 = 0, reg2 = 0;
	unsigned char tmpclk1 = 0, tmpclk2 = 0;

	if (portsel > 1)
		pr_info("nvp portsel[%d] error!\n", portsel);

	switch (portmode) {
	case NVP6124_OUTMODE_1MUX_SD:
		/* 输出720H或者960H单通道,数据37.125MHz,
		   时钟37.125MHz,单沿采样. */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x10);
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x03);
		nvp_i2c_write(0x92 + portsel, 0x03);
		nvp_i2c_write(0xA0 + chid, 0x00);
		nvp_i2c_write(0xC0 + portsel * 2,
			      (chid << 4) | chid);
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xC8 + portsel, 0x00);
		nvp_i2c_write(0xCF - (portsel << 1), 0x86);
	break;
	case NVP6124_OUTMODE_1MUX_HD:
		/* 输出720P或者1280H或者1440H单通道,
		   数据74.25MHz,时钟74.25MHz,单沿采样. */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x10);
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x01);
		nvp_i2c_write(0x92 + portsel, 0x01);
		nvp_i2c_write(0xA0 + chid, 0x00);
		nvp_i2c_write(0xC0 + portsel * 2,
			      (chid << 4) | chid);
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xCF - (portsel << 1), 0x16);
	break;
	case NVP6124_OUTMODE_1MUX_HD5060:
	case NVP6124_OUTMODE_1MUX_FHD:
		/* 输出720P@5060或者1080P单通道,数据148.5MHz,
		   时钟148.5MHz,单沿采样. */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x10);
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x01);
		nvp_i2c_write(0x92 + portsel, 0x01);
		nvp_i2c_write(0xA0 + chid, 0x01);
		nvp_i2c_write(0xC0 + portsel * 2,
			      (chid << 4) | chid);
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xCF - (portsel << 1), 0x43);
		break;
	case NVP6124_OUTMODE_2MUX_SD:
		/*输出720H或者960H 2通道,数据74.25MHz,
		  时钟74.25MHz,单沿采样.*/
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x10);
		reg1 = nvp_i2c_read(0xFD);
		reg2 = nvp_i2c_read(0xFE);
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x02);
		nvp_i2c_write(0x92 + portsel, 0x02);
		nvp_i2c_write(0xA0 + chid * 2, 0x00);
		nvp_i2c_write(0xA1 + chid * 2, 0x00);
		nvp_i2c_write(0xC0 + portsel * 2,
			      chid == 0 ? 0x10 : 0x32);
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		tmp |= (portsel % 2) ? 0x20 : 0x02;
		nvp_i2c_write(0xC8, tmp);
		if (portsel == 0)
			nvp_i2c_write(0xCF, 0x1A);
		else
			nvp_i2c_write(0xCD, 0x16);
		break;
	case NVP6124_OUTMODE_2MUX_HD_X:
		/* 输出HD-X 2通道,数据74.25MHz,
		   时钟74.25MHz,单沿采样. */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x10);
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x00);
		nvp_i2c_write(0x92 + portsel, 0x00);
		nvp_i2c_write(0xA0 + chid * 2, 0x00);
		nvp_i2c_write(0xA1 + chid * 2, 0x00);
		nvp_i2c_write(0xC0 + portsel * 2,
			      chid == 0 ? 0x98 : 0xBA);
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		tmp |= (portsel % 2) ? 0x20 : 0x02;
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xCF - (portsel << 1), 0x0);
		break;
	case NVP6124_OUTMODE_2MUX_FHD_X:
		/* 输出FHD-X 2通道,数据148.5MHz,
		   时钟148.5MHz,单沿采样. */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x10);
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x01);
		nvp_i2c_write(0x92 + portsel, 0x01);
		nvp_i2c_write(0xA0 + chid * 2, 0x01);
		nvp_i2c_write(0xA1 + chid * 2, 0x01);
		nvp_i2c_write(0xC0 + portsel * 2,
			      chid == 0 ? 0x10 : 0x32);
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		tmp |= (portsel % 2) ? 0x10 : 0x01;
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xCF - (portsel << 1), 0x43);
		break;
	case NVP6124_OUTMODE_2MUX_HD:
		/* 输出HD 2通道,数据148.5MHz
		   时钟148.5MHz,单沿采样. */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x10);
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x03);
		nvp_i2c_write(0x92 + portsel, 0x03);
		nvp_i2c_write(0xA0 + chid * 2, 0x03);
		nvp_i2c_write(0xA1 + chid * 2, 0x03);
		nvp_i2c_write(0xC0 + portsel * 2,
			      chid == 0 ? 0x10 : 0x32);
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		tmp |= (portsel % 2) ? 0x20 : 0x02;
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xCF - (portsel << 1), 0x43);
		break;
	case NVP6124_OUTMODE_4MUX_SD:
		/* 输出720H或者960H 4通道,
		   数据148.5MHz,时钟148.5MHz,单沿采样. */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x32);
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x03);
		nvp_i2c_write(0x92 + portsel, 0x03);
		nvp_i2c_write(0xA0, 0x03);
		nvp_i2c_write(0xA1, 0x03);
		nvp_i2c_write(0xA2, 0x03);
		nvp_i2c_write(0xA3, 0x03);
		nvp_i2c_write(0xC0 + portsel * 2, 0x10);
		nvp_i2c_write(0xC1 + portsel * 2, 0x32);
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		tmp |= (portsel % 2) ? 0x80 : 0x08;
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xCF - (portsel << 1), 0x43);

		/* for test clock start */
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0xCD, 0x4f);
		nvp_i2c_write(0xCF, 0x4f);
		tmpclk1 = nvp_i2c_read(0xCD);
		tmpclk2 = nvp_i2c_read(0xCF);
		pr_info("test clk1: 0x%x, clk2: 0x%x\n",
			tmpclk1, tmpclk2);
		break;
	case NVP6124_OUTMODE_4MUX_HD:
		/* 输出720P 4通道,数据297MHz,
		 时钟148.5MHz,双沿采样. */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x32);
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x00);
		nvp_i2c_write(0x92 + portsel, 0x00);
		nvp_i2c_write(0xA0, 0x00);
		nvp_i2c_write(0xA1, 0x00);
		nvp_i2c_write(0xA2, 0x00);
		nvp_i2c_write(0xA3, 0x00);
		nvp_i2c_write(0xC0 + portsel * 2, 0x10);
		nvp_i2c_write(0xC1 + portsel * 2, 0x32);
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		tmp |= (portsel % 2) ? 0x80 : 0x08;
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xCF - (portsel << 1), 0x43);
		break;
	case NVP6124_OUTMODE_4MUX_HD_X:
		/* 输出HD-X 4通道,数据148.5MHz,
		   时钟148.5MHz,单沿采样. */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x32);
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x00);
		nvp_i2c_write(0x92 + portsel, 0x00);
		nvp_i2c_write(0xA0, 0x00);
		nvp_i2c_write(0xA1, 0x00);
		nvp_i2c_write(0xA2, 0x00);
		nvp_i2c_write(0xA3, 0x00);
		nvp_i2c_write(0xC0 + portsel * 2, 0x98);
		nvp_i2c_write(0xC1 + portsel * 2, 0xBA);
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		tmp |= (portsel % 2) ? 0x80 : 0x08;
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xCF - (portsel << 1), 0x43);
		break;
	case NVP6124_OUTMODE_2MUX_FHD:
		/* FHD,3840H,HDEX 2mux任意混合,
		   数据297MHz,时钟148.5MHz,双沿采样.
		   SOC VI端通过丢点，实现3840H->960H,
		   HDEX->720P */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x10);
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x01);
		nvp_i2c_write(0x92 + portsel, 0x01);
		nvp_i2c_write(0xA0 + chid * 2, 0x01);
		nvp_i2c_write(0xA1 + chid * 2, 0x01);
		nvp_i2c_write(0xC0 + portsel * 2,
			      chid == 0 ? 0x10 : 0x32);
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		tmp |= (portsel % 2) ? 0x20 : 0x02;
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xCF - (portsel << 1), 0x43);
		break;
	case NVP6124_OUTMODE_1MUX_HD_X:
	case NVP6124_OUTMODE_1MUX_FHD_X:
		break;
	case NVP6124_OUTMODE_4MUX_FHD_X:
		/* 输出FHD-X 4通道,数据297MHz,
		   时钟148.5MHz,单沿采样. */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x32);
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x01);
		nvp_i2c_write(0x92 + portsel, 0x01);
		nvp_i2c_write(0xA0, 0x01);
		nvp_i2c_write(0xA1, 0x01);
		nvp_i2c_write(0xA2, 0x01);
		nvp_i2c_write(0xA3, 0x01);
		nvp_i2c_write(0xC0 + portsel * 2, 0x98);
		nvp_i2c_write(0xC1 + portsel * 2, 0xBA);
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		tmp |= (portsel % 2) ? 0x80 : 0x08;
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xCF - (portsel << 1), 0x43);
		break;
	case NVP6124_OUTMODE_4MUX_MIX:
		/* HD,1920H,FHD-X 4mux任意混合,
		数据297MHz,时钟148.5MHz,双沿采样
		SOC VI端通过丢点，实现1920H->960H */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x32);
		tmp = nvp_i2c_read(0xFD);
		if ((tmp & 0x0F) == 0x02 ||
		    (tmp & 0x0F) == 0x03)
			reg1 |= 0x08;
		else
			reg1 &= 0xF0;
		if (((tmp >> 4) & 0x0F) == 0x02 ||
		    ((tmp >> 4) & 0x0F) == 0x03)
			reg1 |= 0x80;
		else
			reg1 &= 0x0F;
		tmp = nvp_i2c_read(0xFE);
		if ((tmp & 0x0F) == 0x02 ||
		    (tmp & 0x0F) == 0x03)
			reg2 |= 0x08;
		else
			reg2 &= 0xF0;
		if (((tmp >> 4) & 0x0F) == 0x02 ||
		    ((tmp >> 4) & 0x0F) == 0x03)
			reg2 |= 0x80;
		else
			reg2 &= 0x0F;
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x01);
		nvp_i2c_write(0x92 + portsel, 0x01);
		nvp_i2c_write(0xA0, 0x01);
		nvp_i2c_write(0xA1, 0x01);
		nvp_i2c_write(0xA2, 0x01);
		nvp_i2c_write(0xA3, 0x01);
		nvp_i2c_write(0xC0 + portsel * 2, 0x10 | reg1);
		nvp_i2c_write(0xC1 + portsel * 2, 0x32 | reg2);
		tmp = nvp_i2c_read(0xC8) & ((portsel % 2) ? 0x0F : 0xF0);
		tmp |= (portsel % 2) ? 0x80 : 0x0;
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xCF - (portsel << 1), 0x43);
		break;
	case NVP6124_OUTMODE_2MUX_MIX:
		/* HD,1920H,FHD-X 2MUX任意混合,
		数据148.5MHz,时钟148.5MHz,单沿采样
		SOC VI端通过丢点，实现1920H->960H */
		nvp_i2c_write(0xFF, 0x00);
		nvp_i2c_write(0x56, 0x10);
		tmp = nvp_i2c_read(0xFD);
		if ((tmp & 0x0F) == 0x02 ||
		    (tmp & 0x0F) == 0x03)
			reg1 |= 0x08;
		else
			reg1 &= 0xF0;
		if (((tmp >> 4) & 0x0F) == 0x02 ||
		    ((tmp >> 4) & 0x0F) == 0x03)
			reg1 |= 0x80;
		else
			reg1 &= 0x0F;
		tmp = nvp_i2c_read(0xFE);
		if ((tmp & 0x0F) == 0x02 ||
		    (tmp & 0x0F) == 0x03)
			reg2 |= 0x08;
		else
			reg2 &= 0xF0;
		if (((tmp >> 4) & 0x0F) == 0x02 ||
		    ((tmp >> 4) & 0x0F) == 0x03)
			reg2 |= 0x80;
		else
			reg2 &= 0x0F;
		nvp_i2c_write(0xFF, 0x01);
		nvp_i2c_write(0x90 + portsel, 0x03);
		nvp_i2c_write(0x92 + portsel, 0x03);
		nvp_i2c_write(0xA0 + chid * 2, 0x03);
		nvp_i2c_write(0xA1 + chid * 2, 0x03);
		nvp_i2c_write(0xC0 + portsel * 2,
			      chid == 0 ? (0x10 | reg1) :
					  (0x32 | reg2));
		tmp = nvp_i2c_read(0xC8) &
		      ((portsel % 2) ? 0x0F : 0xF0);
		tmp |= (portsel % 2) ? 0x20 : 0x02;
		nvp_i2c_write(0xC8, tmp);
		nvp_i2c_write(0xCF - (portsel << 1), 0x4f);
		break;
	default:
		pr_info("portmode %d not supported yet\n",
			portmode);
		break;
	}
	pr_info("nvp portsel %d portmode %d setting\n",
		portsel,
		portmode);
}

