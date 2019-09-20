/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Tusson <dusong@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "ap-i2c.h"
#include "linux/i2c.h"

int preisp_ap_i2c_do_xfer(struct spi_device *spi, msg_do_i2c_t *i2c_msg)
{
	struct i2c_adapter *i2c;
	struct i2c_msg msg[2];
	int err = 0;
	int i;

	MSG(msg_do_i2c_ret_t, ret_msg);
	i2c = i2c_get_adapter(i2c_msg->head.nr);
	if (!i2c) {
		dev_err(&spi->dev, "i2c get adapter(%d) error!", i2c_msg->head.nr);
		return -1;
	}

	if (i2c_msg->head.num_msg > AP_I2C_ONCE_MAX_NUM) {
		i2c_msg->head.num_msg = AP_I2C_ONCE_MAX_NUM;
		dev_warn(&spi->dev, "ap i2c once msg num too large %d > %d",
				i2c_msg->head.num_msg, AP_I2C_ONCE_MAX_NUM);
	}

	for (i = 0; i < i2c_msg->head.num_msg; i++) {
		int msg_num = 1;

		msg[0].addr = i2c_msg->msg[i].addr;
		msg[0].len = i2c_msg->msg[i].len;
		msg[0].buf = i2c_msg->msg[i].buf;
		msg[0].flags = i2c_msg->msg[i].flags;

		if (i < (i2c_msg->head.num_msg - 1) &&
			(i2c_msg->msg[i + 1].flags & I2C_M_RD)) {
			msg_num = 2;
			i++;
			msg[1].addr = i2c_msg->msg[i].addr;
			msg[1].len = i2c_msg->msg[i].len;
			msg[1].buf = i2c_msg->msg[i].buf;
			msg[1].flags = i2c_msg->msg[i].flags;
		}

		err = i2c_transfer(i2c, msg, msg_num);

		if (err < 0) {
			dev_err(&spi->dev, "i2c transfer(addr:0x%x,len:%d, msg_num:%d) error(%d)!",
					msg[0].addr, msg[0].len, msg_num, err);
		} else if (msg_num == 2) {
			ret_msg.nr = i2c_msg->head.nr;
			ret_msg.addr = i2c_msg->msg[i].addr;
			memcpy(ret_msg.buf, i2c_msg->msg[i].buf, sizeof(i2c_msg->msg[i].len));

			dsp_msq_send_msg(spi, (msg_t *)&ret_msg);
		}
	}

	i2c_put_adapter(i2c);

	return err;
}
