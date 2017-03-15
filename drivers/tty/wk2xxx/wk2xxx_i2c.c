/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Wang Jianhui <wjh@rock-chips.com>
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/console.h>
#include <linux/device.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/spi/spi.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/tty_flip.h>
#include <asm/irq.h>
#include <linux/io.h>

#include "wk2xxx_i2c.h"

#define WK2XXX_PAGE1        1
#define WK2XXX_PAGE0        0

#define WK2XXX_STATUS_PE    1
#define WK2XXX_STATUS_FE    2
#define WK2XXX_STATUS_BRK   4
#define WK2XXX_STATUS_OE    8

static DEFINE_MUTEX(wk2xxxs_lock);
static DEFINE_MUTEX(wk2xxxs_reg);
static DEFINE_MUTEX(wk2xxxs_work_lock);

struct wk2xxx_port {
	struct uart_port port;
	struct i2c_client *client;
	spinlock_t conf_lock;	/* shared data */
	struct workqueue_struct *workqueue;
	struct work_struct work;
	int suspending;

	void (*wk2xxx_hw_suspend)(int suspend);

	int tx_done;

	int force_end_work;
	int irq;
	int minor;
	int tx_empty;
	int tx_empty_flag;

	int start_tx_flag;
	int stop_tx_flag;
	int stop_rx_flag;
	int irq_flag;
	int conf_flag;

	int tx_empty_fail;
	int start_tx_fail;
	int stop_tx_fail;
	int stop_rx_fail;
	int irq_fail;
	int conf_fail;

	u8 new_lcr;
	u8 new_scr;
	/*set baud 0f register*/
	u8 new_baud1;
	u8 new_baud0;
	u8 new_pres;
};

static struct wk2xxx_port wk2xxxs[NR_PORTS]; /* the chips */

static int wk2xxx_read_reg(struct i2c_client *client,
			   u8 port, u8 reg, u8 *dat)
{
	struct i2c_msg msg[2];
	u8 wk_addr = (0xE0 | ((port - 1) << 2)) >> 1;
	u8 ret, count = 0, status = 0;
	u8 wk_buf[2];

	mutex_lock(&wk2xxxs_reg);

err_read:
	wk_buf[0] = reg;
	msg[0].addr = wk_addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &wk_buf[0];
	if (i2c_transfer(client->adapter, &msg[0], 1) < 0) {
		if (count <= 2) {
			count++;
			goto err_read;
		} else {
			dev_err(&client->dev, "wk2xxx_read_reg w_error!\n");
			status = 1;
		}
	}

	msg[1].addr = wk_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &wk_buf[1];
	ret = i2c_transfer(client->adapter, &msg[1], 1);
	if (ret == 1) {
		*dat = wk_buf[1];
	} else {
		if (count <= 2) {
			count++;
			goto err_read;
		} else {
			dev_err(&client->dev, "wk2xxx_read_reg r_error!\n");
			*dat = 0x0;
			status = 1;
		}
	}

	udelay(5);
	mutex_unlock(&wk2xxxs_reg);

	return status;
}

static int wk2xxx_write_reg(struct i2c_client *client,
	u8 port, u8 reg, u8 dat)
{
	struct i2c_msg msg;
	u8 wk_addr = (0xE0 | ((port - 1) << 2)) >> 1;
	u8 wk_buf[2];
	u8 count1 = 0, status = 0;

	mutex_lock(&wk2xxxs_reg);
err_write:

	wk_buf[0] = reg;
	wk_buf[1] = dat;
	msg.addr = wk_addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = wk_buf;
	if (i2c_transfer(client->adapter, &msg, 1) < 0) {
		if (count1 == 0) {
			count1++;
			goto err_write;
		} else {
			dev_err(&client->dev, "wk2xxx_write_reg error!\n");
			status = 1;
		}
	}

	udelay(5);
	mutex_unlock(&wk2xxxs_reg);
	return status;
}

#define MAX_RFCOUNT_SIZE 256
static int wk2xxx_read_fifo(struct i2c_client *client,
	u8 port, u8 fifolen, u8 *dat)
{
	struct i2c_msg msg;
	u8 wk_addr = (0xE0 | ((port - 1) << 2) | 0x03) >> 1;
	int i, status = 0;
	u8 fifo_data[256] = {0};

	msg.addr = wk_addr;
	msg.flags = I2C_M_RD;
	msg.len = fifolen;
	msg.buf = fifo_data;
	if (i2c_transfer(client->adapter, &msg, 1) < 0) {
		dev_err(&client->dev, "wk2xxx_write_reg error!\n");
		status = 1;
	} else {
		for (i = 0; i < fifolen; i++)
			*(dat + i) = fifo_data[i];
	}
	return status;
}

static int wk2xxx_write_fifo(struct i2c_client *client,
	u8 port, u8 fifolen, u8 *dat)
{
	struct i2c_msg msg;
	u8 wk_addr = (0xE0 | ((port - 1) << 2) | 0x02) >> 1;
	int i, status = 0;
	u8 fifo_data[256] = {0};

	for (i = 0; i < fifolen; i++)
		fifo_data[i + 1] = *(dat + i);

	msg.addr = wk_addr;
	msg.flags = 0;
	msg.len = fifolen;
	msg.buf = dat;

	if (i2c_transfer(client->adapter, &msg, 1) < 0) {
		dev_err(&client->dev, "wk2xxx_write_reg error!\n");
		status = 1;
	}
	return status;
}

static void wk2xxxirq_app(struct uart_port *port);
static void conf_wk2xxx_subport(struct uart_port *port);
static void wk2xxx_work(struct work_struct *w);
static void wk2xxx_stop_tx(struct uart_port *port);
static u32 wk2xxx_tx_empty(struct uart_port *port);

static int wk2xxx_dowork(struct wk2xxx_port *s)
{
	if (!s->force_end_work && !work_pending(&s->work) &&
	    !freezing(current) && !s->suspending) {
		queue_work(s->workqueue, &s->work);
		return 1;
	} else {
		return 0;
	}
}

static void wk2xxx_work(struct work_struct *w)
{
	u8 rx;
	struct wk2xxx_port *s = container_of(w, struct wk2xxx_port, work);
	struct i2c_client *wk2xxx_i2c_client = s->client;
	int work_start_tx_flag;
	int work_stop_rx_flag;

	int work_irq_flag;
	int work_conf_flag;

	do {
		mutex_lock(&wk2xxxs_work_lock);

		work_start_tx_flag = s->start_tx_flag;

		if (work_start_tx_flag)
			s->start_tx_flag = 0;
		work_stop_rx_flag = s->stop_rx_flag;

		if (work_stop_rx_flag)
			s->stop_rx_flag = 0;
		work_conf_flag = s->conf_flag;
		work_irq_flag = s->irq_flag;

		if (work_irq_flag)
			s->irq_flag = 0;

		mutex_unlock(&wk2xxxs_work_lock);

		if (work_start_tx_flag) {
			wk2xxx_read_reg(wk2xxx_i2c_client,
					s->port.iobase, WK2XXX_SIER, &rx);
			rx |= WK2XXX_TFTRIG_IEN;
			wk2xxx_write_reg(wk2xxx_i2c_client,
					 s->port.iobase, WK2XXX_SIER, rx);
		}

		if (work_stop_rx_flag) {
			wk2xxx_read_reg(wk2xxx_i2c_client,
					s->port.iobase, WK2XXX_SIER, &rx);
			rx &= ~WK2XXX_RFTRIG_IEN;
			rx &= ~WK2XXX_RXOUT_IEN;
			wk2xxx_write_reg(wk2xxx_i2c_client,
					 s->port.iobase, WK2XXX_SIER, rx);
			wk2xxx_read_reg(wk2xxx_i2c_client,
					s->port.iobase, WK2XXX_SIFR, &rx);
			rx &= ~WK2XXX_RXOVT_INT;
			rx &= ~WK2XXX_RFTRIG_INT;
			wk2xxx_write_reg(wk2xxx_i2c_client,
					 s->port.iobase, WK2XXX_SIFR, rx);
		}

		if (work_irq_flag) {
			wk2xxxirq_app(&s->port);
			s->irq_fail = 1;
		}
	} while (!s->force_end_work && !freezing(current) &&
		 (work_irq_flag || work_stop_rx_flag));

	if (s->start_tx_fail) {
		wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase,
				WK2XXX_SIER, &rx);
		rx |= WK2XXX_TFTRIG_IEN;
		wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
				 WK2XXX_SIER, rx);
		s->start_tx_fail = 0;
	}

	if (s->stop_rx_fail) {
		wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase,
				WK2XXX_SIER, &rx);
		rx &= ~WK2XXX_RFTRIG_IEN;
		rx &= ~WK2XXX_RXOUT_IEN;
		wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
				 WK2XXX_SIER, rx);

		wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase,
				WK2XXX_SIFR, &rx);
		rx &= ~WK2XXX_RFTRIG_INT;
		rx &= ~WK2XXX_RXOVT_INT;
		wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
				 WK2XXX_SIFR, rx);
		s->stop_rx_fail = 0;
	}

	if (s->irq_fail) {
		s->irq_fail = 0;
		enable_irq(s->port.irq);
	}
}

static void wk2xxx_rx_chars(struct uart_port *port)
{
	int rfcnt = 0, rx_num = 0;
	u8 fsr, lsr, dat[1], rx_dat[256] = {0};
	u32 ch, flg, sifr, ignored = 0, status = 0, rx_count = 0;
	struct wk2xxx_port *s = container_of(port, struct wk2xxx_port, port);
	struct i2c_client *wk2xxx_i2c_client = s->client;

	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
			 WK2XXX_SPAGE, WK2XXX_PAGE0);
	wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_FSR, dat);
	fsr = dat[0];
	wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_LSR, dat);
	lsr = dat[0];
	wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_SIFR, dat);
	sifr = dat[0];

	if (!(sifr & 0x80)) {
		flg = TTY_NORMAL;
		if (fsr & WK2XXX_RDAT) {
			wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase,
					 WK2XXX_RFCNT, dat);
			rfcnt = dat[0];
			if (rfcnt == 0)
				rfcnt = 255;

			wk2xxx_read_fifo(wk2xxx_i2c_client, s->port.iobase,
					 rfcnt, rx_dat);
			s->port.icount.rx += rfcnt;
			for (rx_num = 0; rx_num < rfcnt; rx_num++) {
				if (uart_handle_sysrq_char(&s->port,
							   rx_dat[rx_num]))
					break;

				uart_insert_char(&s->port, status,
						 WK2XXX_STATUS_OE,
						 rx_dat[rx_num], flg);
				rx_count++;

				if ((rx_count >= 64) &&
				    s->port.state) {
					tty_flip_buffer_push(&s->port.state->port);
					rx_count = 0;
				}
			}

			if ((rx_count > 0) && s->port.state) {
				tty_flip_buffer_push(&s->port.state->port);
				rx_count = 0;
			}
		}
	} else {
		while (fsr & WK2XXX_RDAT) {
			wk2xxx_read_reg(wk2xxx_i2c_client,
					s->port.iobase, WK2XXX_FDAT, dat);
			ch = (int)dat[0];

			s->port.icount.rx++;
			flg = TTY_NORMAL;
			if (lsr & (WK2XXX_OE | WK2XXX_FE
			    | WK2XXX_PE | WK2XXX_BI)) {
				if (lsr & WK2XXX_PE) {
					s->port.icount.parity++;
					status |= WK2XXX_STATUS_PE;
					flg = TTY_PARITY;
				}
				if (lsr & WK2XXX_FE) {
					s->port.icount.frame++;
					status |= WK2XXX_STATUS_FE;
					flg = TTY_FRAME;
				}
				if (lsr & WK2XXX_OE) {
					s->port.icount.overrun++;
					status |= WK2XXX_STATUS_OE;
					flg = TTY_OVERRUN;
				}
				if (lsr & fsr & WK2XXX_BI) {
					s->port.icount.brk++;
					status |= WK2XXX_STATUS_BRK;
					flg = TTY_BREAK;
				}
				if (++ignored > 100)
					goto out;

				goto ignore_char;
			}

error_return:
			if (uart_handle_sysrq_char(&s->port, ch))
				goto ignore_char;

			uart_insert_char(&s->port, status,
					 WK2XXX_STATUS_OE, ch, flg);
			rx_count++;

			if ((rx_count >= 64) && s->port.state) {
				tty_flip_buffer_push(&s->port.state->port);
				rx_count = 0;
			}

ignore_char:
			wk2xxx_read_reg(wk2xxx_i2c_client,
					s->port.iobase, WK2XXX_FSR, dat);
			fsr = dat[0];
			wk2xxx_read_reg(wk2xxx_i2c_client,
					s->port.iobase, WK2XXX_LSR, dat);
			lsr = dat[0];
	}

out:
		if ((rx_count > 0) && s->port.state) {
			tty_flip_buffer_push(&s->port.state->port);
			rx_count = 0;
		}
	}

	return;

#ifdef SUPPORT_SYSRQ
	s->port.state->sysrq = 0;
#endif
	goto error_return;
}

static void wk2xxx_tx_chars(struct uart_port *port)
{
	int count, tx_count, i;
	u8 fsr, tfcnt, dat[1], txbuf[255] = {0};
	struct wk2xxx_port *s = container_of(port, struct wk2xxx_port, port);
	struct i2c_client *wk2xxx_i2c_client = s->client;

	wk2xxx_write_reg(wk2xxx_i2c_client,
			 s->port.iobase, WK2XXX_SPAGE, WK2XXX_PAGE0);
	if (s->port.x_char) {
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 s->port.iobase, WK2XXX_FDAT, s->port.x_char);
		s->port.icount.tx++;
		s->port.x_char = 0;
		goto out;
	}

	if (uart_circ_empty(&s->port.state->xmit) ||
	    uart_tx_stopped(&s->port)) {
		goto out;
	}

	/*
	 * Tried using FIFO (not checking TNF) for fifo fill:
	 * still had the '1 bytes repeated' problem.
	 */
	wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_FSR, dat);
	fsr = dat[0];
	wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_TFCNT, dat);
	tfcnt = dat[0];

	if (tfcnt == 0) {
		if (fsr & WK2XXX_TFULL) {
			tfcnt = 255;
			tx_count = 0;
		} else {
			tfcnt = 0;
			tx_count = 255;
		}
	} else {
		tx_count = 255 - tfcnt;
	}

	count = tx_count;
	i = 0;

	do {
		if (uart_circ_empty(&s->port.state->xmit))
			break;
		txbuf[i] = s->port.state->xmit.buf[s->port.state->xmit.tail];
		s->port.state->xmit.tail = (s->port.state->xmit.tail + 1)
					   & (UART_XMIT_SIZE - 1);
		s->port.icount.tx++;
		i++;
	} while (--count > 0);

	wk2xxx_write_fifo(wk2xxx_i2c_client, s->port.iobase, i, txbuf);
out:
	wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_FSR, dat);
	fsr = dat[0];
	if (((fsr & WK2XXX_TDAT) == 0) && ((fsr & WK2XXX_TBUSY) == 0)) {
		if (uart_circ_chars_pending(&s->port.state->xmit)
		    < WAKEUP_CHARS)
			uart_write_wakeup(&s->port);

		if (uart_circ_empty(&s->port.state->xmit))
			wk2xxx_stop_tx(&s->port);
	}
}

static irqreturn_t wk2xxx_irq(int irq, void *dev_id)
{
	struct wk2xxx_port *s = dev_id;

	disable_irq_nosync(s->port.irq);
	s->irq_flag = 1;

	if (!wk2xxx_dowork(s)) {
		s->irq_flag = 0;
		s->irq_fail = 1;
	}

	return IRQ_HANDLED;
}

static void wk2xxxirq_app(struct uart_port *port)
{
	u32 pass_counter = 0;
	u8 sifr, gifr, sier, dat[1];
	struct wk2xxx_port *s = container_of(port, struct wk2xxx_port, port);
	struct i2c_client *wk2xxx_i2c_client = s->client;

	wk2xxx_read_reg(wk2xxx_i2c_client, 2, WK2XXX_GIFR, dat);
	gifr = dat[0];

	switch (s->port.iobase) {
	case 1:
		if (!(gifr & WK2XXX_UT1INT))
			return;
		break;
	case 2:
		if (!(gifr & WK2XXX_UT2INT))
			return;
		break;
	case 3:
		if (!(gifr & WK2XXX_UT3INT))
			return;
		break;
	case 4:
		if (!(gifr & WK2XXX_UT4INT))
			return;
		break;
	default:
		break;
	}

	wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_SIFR, dat);
	sifr = dat[0];
	wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_SIER, dat);
	sier = dat[0];

	do {
		if ((sifr & WK2XXX_RFTRIG_INT) | (sifr & WK2XXX_RXOVT_INT))
			wk2xxx_rx_chars(&s->port);

		if ((sifr & WK2XXX_TFTRIG_INT) &&
		    (sier & WK2XXX_TFTRIG_IEN)) {
			wk2xxx_tx_chars(&s->port);
			return;
		}

		if (pass_counter++ > WK2XXX_ISR_PASS_LIMIT)
			break;
		wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase,
				WK2XXX_SIFR, dat);
		sifr = dat[0];
		wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase,
				WK2XXX_SIER, dat);
		sier = dat[0];
	} while ((sifr & WK2XXX_RXOVT_INT) || (sifr & WK2XXX_RFTRIG_INT) ||
		 ((sifr & WK2XXX_TFTRIG_INT) && (sier & WK2XXX_TFTRIG_IEN)));
}

/*
 *   Return TIOCSER_TEMT when transmitter is not busy.
 */
static u_int wk2xxx_tx_empty(struct uart_port *port)
{
	u8 rx;
	struct wk2xxx_port *s = container_of(port, struct wk2xxx_port, port);
	struct i2c_client *wk2xxx_i2c_client = s->client;

	mutex_lock(&wk2xxxs_lock);

	if (!(s->tx_empty_flag || s->tx_empty_fail)) {
		wk2xxx_read_reg(wk2xxx_i2c_client,
				s->port.iobase, WK2XXX_FSR, &rx);

		while ((rx & WK2XXX_TDAT) | (rx & WK2XXX_TBUSY)) {
			wk2xxx_read_reg(wk2xxx_i2c_client,
					s->port.iobase, WK2XXX_FSR, &rx);
		}
		s->tx_empty = ((rx & WK2XXX_TDAT) | (rx & WK2XXX_TBUSY)) <= 0;

		if (s->tx_empty) {
			s->tx_empty_flag = 0;
			s->tx_empty_fail = 0;
		} else {
			s->tx_empty_fail = 0;
			s->tx_empty_flag = 0;
		}
	}

	mutex_unlock(&wk2xxxs_lock);
	return s->tx_empty;
}

static void wk2xxx_set_mctrl(struct uart_port *port, u_int mctrl)
{
#ifdef _DEBUG_WK2XXX
	pr_info("wk2xxx_set_mctrl---------exit---\n");
#endif
}

static u32 wk2xxx_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

/*
 *  interrupts disabled on entry
 */
static void wk2xxx_stop_tx(struct uart_port *port)
{
	u8 dat[1], sier, sifr;
	struct wk2xxx_port *s = container_of(port, struct wk2xxx_port, port);
	struct i2c_client *wk2xxx_i2c_client = s->client;

	mutex_lock(&wk2xxxs_lock);

	if (!(s->stop_tx_flag || s->stop_tx_fail)) {
		wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase,
				WK2XXX_SIER, dat);
		sier = dat[0];
		s->stop_tx_fail = (sier & WK2XXX_TFTRIG_IEN) > 0;
		if (s->stop_tx_fail) {
			wk2xxx_read_reg(wk2xxx_i2c_client,
					s->port.iobase, WK2XXX_SIER, dat);
			sier = dat[0];
			sier &= ~WK2XXX_TFTRIG_IEN;
			wk2xxx_write_reg(wk2xxx_i2c_client,
					 s->port.iobase, WK2XXX_SIER, sier);
			wk2xxx_read_reg(wk2xxx_i2c_client,
					s->port.iobase, WK2XXX_SIFR, dat);
			sifr = dat[0];
			sifr &= ~WK2XXX_TFTRIG_INT;
			wk2xxx_write_reg(wk2xxx_i2c_client,
					 s->port.iobase, WK2XXX_SIFR, sifr);
			s->stop_tx_fail = 0;
			s->stop_tx_flag = 0;
		} else {
			s->stop_tx_fail = 0;
			s->stop_tx_flag = 0;
		}
	}
	mutex_unlock(&wk2xxxs_lock);
}

/*
 *  * interrupts may not be disabled on entry
 */
static void wk2xxx_start_tx(struct uart_port *port)
{
	struct wk2xxx_port *s = container_of(port, struct wk2xxx_port, port);

	if (!(s->start_tx_flag || s->start_tx_fail)) {
		s->start_tx_flag = 1;
		if (!wk2xxx_dowork(s)) {
			s->start_tx_fail = 1;
			s->start_tx_flag = 0;
		}
	}
}

/*
 *  * Interrupts enabled
 */

static void wk2xxx_stop_rx(struct uart_port *port)
{
	struct wk2xxx_port *s = container_of(port, struct wk2xxx_port, port);

	if (!(s->stop_rx_flag || s->stop_rx_fail)) {
		s->stop_rx_flag = 1;
		if (!wk2xxx_dowork(s)) {
			s->stop_rx_flag = 0;
			s->stop_rx_fail = 1;
		}
	}
}

/*
 *  * No modem control lines
 *
 */
static void wk2xxx_enable_ms(struct uart_port *port)
{
#ifdef _DEBUG_WK2XXX
	pr_info("wk2xxx_enable_ms------exit---\n");
#endif
}

/*
 *  * Interrupts always disabled.
 */
static void wk2xxx_break_ctl(struct uart_port *port, int break_state)
{
#ifdef _DEBUG_WK2XXX
	pr_info("wk2xxx_break_ctl------exit---\n");
#endif
}

static int wk2xxx_startup(struct uart_port *port)
{
	u8 gena, grst, gier, sier, scr, dat[1];
	char b[12];
	struct wk2xxx_port *s = container_of(port, struct wk2xxx_port, port);
	struct i2c_client *wk2xxx_i2c_client = s->client;

	if (s->suspending)
		return 0;

	s->force_end_work = 0;
	sprintf(b, "wk2xxx-%d", (u8)s->port.iobase);
	s->workqueue = create_singlethread_workqueue(b);

	if (!s->workqueue) {
		dev_warn(&wk2xxx_i2c_client->dev,
			 "cannot create workqueue\n");
		return -EBUSY;
	}

	INIT_WORK(&s->work, wk2xxx_work);

	if (s->wk2xxx_hw_suspend)
		s->wk2xxx_hw_suspend(0);

	wk2xxx_read_reg(wk2xxx_i2c_client, 1, WK2XXX_GENA, dat);
	gena = dat[0];

	switch (s->port.iobase) {
	case 1:
		gena |= WK2XXX_UT1EN;
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 1, WK2XXX_GENA, gena);
		break;
	case 2:
		gena |= WK2XXX_UT2EN;
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 1, WK2XXX_GENA, gena);
		break;
	case 3:
		gena |= WK2XXX_UT3EN;
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 1, WK2XXX_GENA, gena);
		break;
	case 4:
		gena |= WK2XXX_UT4EN;
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 1, WK2XXX_GENA, gena);
		break;
	default:
		pr_info("con_wk2xxx_subport bad iobase %d\n",
			(u8)s->port.iobase);
		break;
	}

	wk2xxx_read_reg(wk2xxx_i2c_client, 1, WK2XXX_GRST, dat);
	grst = dat[0];
	switch (s->port.iobase) {
	case 1:
		grst |= WK2XXX_UT1RST;
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 1, WK2XXX_GRST, grst);
		break;
	case 2:
		grst |= WK2XXX_UT2RST;
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 1, WK2XXX_GRST, grst);
		break;
	case 3:
		grst |= WK2XXX_UT3RST;
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 1, WK2XXX_GRST, grst);
		break;
	case 4:
		grst |= WK2XXX_UT4RST;
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 1, WK2XXX_GRST, grst);
		break;
	default:
		pr_info("con_wk2xxx_subport bad iobase %x\n",
			(u8)s->port.iobase);
		break;
	}

	wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_SIER, dat);
	sier = dat[0];
	sier &= ~WK2XXX_TFTRIG_IEN;
	sier |= WK2XXX_RFTRIG_IEN;
	sier |= WK2XXX_RXOUT_IEN;
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_SIER, sier);
	wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_SCR, dat);
	scr = dat[0] | WK2XXX_TXEN | WK2XXX_RXEN;
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_SCR, scr);

	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_FCR, 0x0f);
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_FCR, 0x0c);
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_SPAGE, 1);
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_RFTL, 0x80);
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_TFTL, 0x20);
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_SPAGE, 0);

	wk2xxx_read_reg(wk2xxx_i2c_client, 2, WK2XXX_GIER, dat);
	gier = dat[0];

	switch (s->port.iobase) {
	case 1:
		gier |= WK2XXX_UT1IE;
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 2, WK2XXX_GIER, gier);
		break;
	case 2:
		gier |= WK2XXX_UT2IE;
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 2, WK2XXX_GIER, gier);
		break;
	case 3:
		gier |= WK2XXX_UT3IE;
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 2, WK2XXX_GIER, gier);
		break;
	case 4:
		gier |= WK2XXX_UT4IE;
		wk2xxx_write_reg(wk2xxx_i2c_client,
				 2, WK2XXX_GIER, gier);
		break;
	default:
		pr_info(": bad iobase %x\n", (u8)s->port.iobase);
		break;
	}

	if (s->wk2xxx_hw_suspend)
		s->wk2xxx_hw_suspend(0);
	msleep(50);

	uart_circ_clear(&s->port.state->xmit);
	wk2xxx_enable_ms(&s->port);
	if (request_irq(s->port.irq, wk2xxx_irq,
			IRQF_SHARED | IRQF_TRIGGER_LOW,
			"wk2xxx_i2c", s) < 0) {
		dev_warn(&wk2xxx_i2c_client->dev,
			 "cannot allocate irq %d\n", s->irq);
		s->port.irq = 0;
		destroy_workqueue(s->workqueue);
		s->workqueue = NULL;
		return -EBUSY;
	}
	udelay(200);

	return 0;
}

static void wk2xxx_shutdown(struct uart_port *port)
{
	u8 gena, dat[1];
	struct wk2xxx_port *s = container_of(port, struct wk2xxx_port, port);
	struct i2c_client *wk2xxx_i2c_client = s->client;

	if (s->suspending)
		return;

	s->force_end_work = 1;

	if (s->workqueue) {
		flush_workqueue(s->workqueue);
		destroy_workqueue(s->workqueue);
		s->workqueue = NULL;
	}

	if (s->port.irq)
		free_irq(s->port.irq, s);

	wk2xxx_read_reg(wk2xxx_i2c_client, 1, WK2XXX_GENA, dat);
	gena = dat[0];
	switch (s->port.iobase) {
	case 1:
		gena &= ~WK2XXX_UT1EN;
		wk2xxx_write_reg(wk2xxx_i2c_client, 1, WK2XXX_GENA, gena);
		break;
	case 2:
		gena &= ~WK2XXX_UT2EN;
		wk2xxx_write_reg(wk2xxx_i2c_client, 1, WK2XXX_GENA, gena);
		break;
	case 3:
		gena &= ~WK2XXX_UT3EN;
		wk2xxx_write_reg(wk2xxx_i2c_client, 1, WK2XXX_GENA, gena);
		break;
	case 4:
		gena &= ~WK2XXX_UT4EN;
		wk2xxx_write_reg(wk2xxx_i2c_client, 1, WK2XXX_GENA, gena);
		break;
	default:
		pr_info(":wk2xxx_shutdown   bad iobase %x\n",
			(u8)s->port.iobase);
		break;
	}
}

static void conf_wk2xxx_subport(struct uart_port *port)
{
	u8 old_sier, lcr, scr, scr_ss, dat[1], baud0_ss, baud1_ss, pres_ss;
	struct wk2xxx_port *s = container_of(port, struct wk2xxx_port, port);
	struct i2c_client *wk2xxx_i2c_client = s->client;

	lcr = s->new_lcr;
	scr_ss = s->new_scr;
	baud0_ss = s->new_baud0;
	baud1_ss = s->new_baud1;
	pres_ss = s->new_pres;
	wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_SIER, dat);
	old_sier = dat[0];
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_SIER,
			 old_sier & (~(WK2XXX_TFTRIG_IEN |
			 WK2XXX_RFTRIG_IEN | WK2XXX_RXOUT_IEN)));
	do {
		wk2xxx_read_reg(wk2xxx_i2c_client,
				s->port.iobase, WK2XXX_FSR, dat);
	} while (dat[0] & WK2XXX_TBUSY);

	wk2xxx_read_reg(wk2xxx_i2c_client, s->port.iobase, WK2XXX_SCR, dat);
	scr = dat[0];
	scr &= 0x0f;
	scr |= scr_ss;

	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
			 WK2XXX_SCR, scr & (~(WK2XXX_RXEN | WK2XXX_TXEN)));
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
			 WK2XXX_LCR, lcr);
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
			 WK2XXX_SIER, old_sier);
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
			 WK2XXX_SPAGE, 1);
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
			 WK2XXX_BAUD0, baud0_ss);
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
			 WK2XXX_BAUD1, baud1_ss);
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
			 WK2XXX_PRES, pres_ss);

	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
			 WK2XXX_SPAGE, 0);
	wk2xxx_write_reg(wk2xxx_i2c_client, s->port.iobase,
			 WK2XXX_SCR, scr | (WK2XXX_RXEN | WK2XXX_TXEN));
}

static void wk2xxx_termios(struct uart_port *port, struct ktermios *termios,
			   struct ktermios *old)
{
	int baud = 0;
	u8 lcr, baud1 = 0, baud0 = 0, pres = 0;
	u16 cflag;
	u16 lflag;
	struct wk2xxx_port *s = container_of(port, struct wk2xxx_port, port);

	cflag = termios->c_cflag;
	lflag = termios->c_lflag;

	baud = tty_termios_baud_rate(termios);

	switch (baud) {
	case 600:
		baud1 = 0x4;
		baud0 = 0x7f;
		pres = 0;
		break;
	case 1200:
		baud1 = 0x2;
		baud0 = 0x3F;
		pres = 0;
		break;
	case 2400:
		baud1 = 0x1;
		baud0 = 0x1f;
		pres = 0;
		break;
	case 4800:
		baud1 = 0x00;
		baud0 = 0x8f;
		pres = 0;
		break;
	case 9600:
		baud1 = 0x00;
		baud0 = 0x47;
		pres = 0;
		break;
	case 19200:
		baud1 = 0x00;
		baud0 = 0x23;
		pres = 0;
		break;
	case 38400:
		baud1 = 0x00;
		baud0 = 0x11;
		pres = 0;
		break;
	case 76800:
		baud1 = 0x00;
		baud0 = 0x08;
		pres = 0;
		break;
	case 1800:
		baud1 = 0x01;
		baud0 = 0x7f;
		pres = 0;
		break;
	case 3600:
		baud1 = 0x00;
		baud0 = 0xbf;
		pres = 0;
		break;
	case 7200:
		baud1 = 0x00;
		baud0 = 0x5f;
		pres = 0;
		break;
	case 14400:
		baud1 = 0x00;
		baud0 = 0x2f;
		pres = 0;
		break;
	case 28800:
		baud1 = 0x00;
		baud0 = 0x17;
		pres = 0;
		break;
	case 57600:
		baud1 = 0x00;
		baud0 = 0x0b;
		pres = 0;
		break;
	case 115200:
		baud1 = 0x00;
		baud0 = 0x05;
		pres = 0;
		break;
	case 230400:
		baud1 = 0x00;
		baud0 = 0x02;
		pres = 0;
		break;
	default:
		baud1 = 0x00;
		baud0 = 0x00;
		pres = 0;
	}
	tty_termios_encode_baud_rate(termios, baud, baud);

	/* we are sending char from a workqueue so enable */
	lcr = 0;
	if (cflag & CSTOPB)
		lcr |= WK2XXX_STPL;
	else
		lcr &= ~WK2XXX_STPL;

	if (cflag & PARENB) {
		if (!(cflag & PARODD)) {
			lcr |= WK2XXX_PAM1;
			lcr &= ~WK2XXX_PAM0;
		} else {
			lcr |= WK2XXX_PAM0;
			lcr &= ~WK2XXX_PAM1;
		}
	} else {
		lcr &= ~WK2XXX_PAEN;
	}

	s->new_baud1 = baud1;
	s->new_baud0 = baud0;
	s->new_pres = pres;
	s->new_lcr = lcr;

	conf_wk2xxx_subport(&s->port);
}

static const char *wk2xxx_type(struct uart_port *port)
{
	return port->type == PORT_WK2XXX ? "wk2xxx" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void wk2xxx_release_port(struct uart_port *port)
{
	pr_info("wk2xxx_release_port\n");
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int wk2xxx_request_port(struct uart_port *port)
{
	return 0;
}

/*
 * Configure/autoconfigure the port
 */
static void wk2xxx_config_port(struct uart_port *port, int flags)
{
	struct wk2xxx_port *s = container_of(port, struct wk2xxx_port, port);

	if (flags & UART_CONFIG_TYPE && wk2xxx_request_port(port) == 0)
		s->port.type = PORT_WK2XXX;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_vk32xx and PORT_UNKNOWN
 */
static int wk2xxx_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_WK2XXX)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != SERIAL_IO_PORT)
		ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

static const struct uart_ops wk2xxx_pops = {
	.tx_empty		= wk2xxx_tx_empty,
	.set_mctrl		= wk2xxx_set_mctrl,
	.get_mctrl		= wk2xxx_get_mctrl,
	.stop_tx		= wk2xxx_stop_tx,
	.start_tx		= wk2xxx_start_tx,
	.stop_rx		= wk2xxx_stop_rx,
	.enable_ms		= wk2xxx_enable_ms,
	.break_ctl		= wk2xxx_break_ctl,
	.startup		= wk2xxx_startup,
	.shutdown		= wk2xxx_shutdown,
	.set_termios		= wk2xxx_termios,
	.type			= wk2xxx_type,
	.release_port		= wk2xxx_release_port,
	.request_port		= wk2xxx_request_port,
	.config_port		= wk2xxx_config_port,
	.verify_port		= wk2xxx_verify_port,
};

static struct uart_driver wk2xxx_uart_driver = {
	.owner				= THIS_MODULE,
	.major				= SERIAL_WK2XXX_MAJOR,
	.driver_name			= "ttySWK",
	.dev_name			= "ttysWK",
	.minor				= MINOR_START,
	.nr				= NR_PORTS,
	.cons				= NULL,
};

static int uart_driver_registered;
static int wk2xxx_probe(struct i2c_client *client,
			const struct i2c_device_id *dev_id)
{
	u8 i;
	int status;

	mutex_lock(&wk2xxxs_lock);
	if (!uart_driver_registered) {
		uart_driver_registered = 1;
		status = uart_register_driver(&wk2xxx_uart_driver);
		if (status) {
			dev_err(&client->dev,
				"Couldn't register wk2xxx uart driver\n");
			mutex_unlock(&wk2xxxs_lock);
			return status;
		}
	}

	if (client->irq <= 0) {
		dev_err(&client->dev,
			"wk2xxx has not been provided an Int IRQ\n");
		mutex_unlock(&wk2xxxs_lock);
		return -EINVAL;
	}

	for (i = 0; i < NR_PORTS; i++) {
		struct wk2xxx_port *s	= &wk2xxxs[i];

		s->tx_done		= 0;
		s->client		= client;
		s->port.line		= i;
		s->port.ops		= &wk2xxx_pops;
		s->port.uartclk		= WK_CRASTAL_CLK;
		s->port.fifosize	= 64;
		s->port.iobase		= i + 1;
		s->port.irq		= client->irq;
		s->port.iotype		= SERIAL_IO_PORT;
		s->port.flags		= ASYNC_BOOT_AUTOCONF;

		status = uart_add_one_port(&wk2xxx_uart_driver, &s->port);
		if (status < 0)
			pr_info("%s failed i:= %d with error %d\n",
				__func__, i, status);
	}

	mutex_unlock(&wk2xxxs_lock);

	return 0;
}

static int wk2xxx_remove(struct i2c_client *i2c_client)
{
	int i;

	mutex_lock(&wk2xxxs_lock);

	for (i = 0; i < NR_PORTS; i++) {
		struct wk2xxx_port *s = &wk2xxxs[i];

		uart_remove_one_port(&wk2xxx_uart_driver, &s->port);
	}

	uart_unregister_driver(&wk2xxx_uart_driver);
	mutex_unlock(&wk2xxxs_lock);

	return 0;
}

static const struct i2c_device_id wk2xxx_i2c_id_table[] = {
	{"wk2xxx_i2c", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, wk2xxx_i2c_id_table);

static struct of_device_id wk2xxx_i2c_dt_ids[] = {
	{.compatible = "wkmic,wk2xxx_i2c", },
	{}
};

MODULE_DEVICE_TABLE(of, wk2xxx_i2c_dt_ids);

static struct i2c_driver wk2xxx_i2c_driver = {
	.driver = {
		.name       = "wk2xxx_i2c",
		.owner      = THIS_MODULE,
		.of_match_table = of_match_ptr(wk2xxx_i2c_dt_ids),
	},

	.probe          = wk2xxx_probe,
	.remove         = wk2xxx_remove,
	.id_table       = wk2xxx_i2c_id_table,
};

module_i2c_driver(wk2xxx_i2c_driver);

MODULE_AUTHOR("WangJianhui <wjh@rock-chips.com>");
MODULE_DESCRIPTION("wk2xxx generic serial port Driver");
MODULE_LICENSE("GPL");
