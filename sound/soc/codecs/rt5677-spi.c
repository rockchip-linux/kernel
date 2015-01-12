/*
 * rt5677-spi.c  --  RT5677 ALSA SoC audio codec driver
 *
 * Copyright 2013 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_qos.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/acpi.h>

#include "rt5677-spi.h"

#define SPI_BURST_LEN		240
#define SPI_HEADER		5
#define SPI_READ_FREQ		1000000

#define RT5677_SPI_WRITE_BURST	0x5
#define RT5677_SPI_READ_BURST	0x4
#define RT5677_SPI_WRITE_32	0x3
#define RT5677_SPI_READ_32	0x2
#define RT5677_SPI_WRITE_16	0x1
#define RT5677_SPI_READ_16	0x0

static struct spi_device *g_spi;
static DEFINE_MUTEX(spi_mutex);

/* Read DSP memory using SPI. Addr and len have to be multiples of 16-bits. */
int rt5677_spi_read(u32 addr, void *rxbuf, size_t len)
{
	unsigned int i, end, offset = 0;
	int status = 0;
	struct spi_transfer t[2];
	struct spi_message m;
	u8 *rx_buf;
	u8 buf[SPI_BURST_LEN + SPI_HEADER + 4];
	u8 spi_cmd;
	u8 *rx_data = rxbuf;

	if (!g_spi)
		return -ENODEV;

	rx_buf = buf + SPI_HEADER + 4;
	memset(t, 0, sizeof(t));
	t[0].tx_buf = buf;
	t[0].len = SPI_HEADER + 4;
	t[0].speed_hz = SPI_READ_FREQ;
	t[1].rx_buf = rx_buf;
	t[1].speed_hz = SPI_READ_FREQ;
	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	while (offset < len) {
		/* TODO benzh: Clean up the read length selection logic */
		switch (min(len - offset, (size_t)(addr + offset) & 0x7)) {
		case 4:
			spi_cmd = RT5677_SPI_READ_32;
			end = 4;
			break;
		case 2:
		case 6:
			spi_cmd = RT5677_SPI_READ_16;
			end = 2;
			break;
		case 0:
			if (offset + SPI_BURST_LEN <= len) {
				spi_cmd = RT5677_SPI_READ_BURST;
				end = SPI_BURST_LEN;
				break;
			} else if (offset + 4 <= len) {
				spi_cmd = RT5677_SPI_READ_32;
				end = 4;
				break;
			} else if (offset + 2 <= len) {
				spi_cmd = RT5677_SPI_READ_16;
				end = 2;
				break;
			}
			/* Fall through to default error case */
		default:
			pr_err("Bad section alignment\n");
			return -EACCES;
		}

		buf[0] = spi_cmd;
		buf[1] = ((addr + offset) & 0xff000000) >> 24;
		buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		t[1].len = end;

		mutex_lock(&spi_mutex);
		status |= spi_sync(g_spi, &m);
		mutex_unlock(&spi_mutex);

		if (spi_cmd == RT5677_SPI_READ_BURST) {
			for (i = 0; i < end; i += 8) {
				rx_data[offset + i + 0] = rx_buf[i + 7];
				rx_data[offset + i + 1] = rx_buf[i + 6];
				rx_data[offset + i + 2] = rx_buf[i + 5];
				rx_data[offset + i + 3] = rx_buf[i + 4];
				rx_data[offset + i + 4] = rx_buf[i + 3];
				rx_data[offset + i + 5] = rx_buf[i + 2];
				rx_data[offset + i + 6] = rx_buf[i + 1];
				rx_data[offset + i + 7] = rx_buf[i + 0];
			}
		} else {
			for (i = 0; i < end; i++)
				rx_data[offset + i] = rx_buf[end - i - 1];
		}

		offset += end;
	}

	return status;
}
EXPORT_SYMBOL_GPL(rt5677_spi_read);

int rt5677_spi_write(u32 addr, const u8 *txbuf, size_t len)
{
	unsigned int i, end, offset = 0;
	int status = 0;
	u8 write_buf[SPI_BURST_LEN + SPI_HEADER + 1];
	u8 spi_cmd;

	if (!g_spi)
		return -ENODEV;

	while (offset < len) {
		switch ((addr + offset) & 0x7) {
		case 4:
			spi_cmd = RT5677_SPI_WRITE_32;
			end = 4;
			break;
		case 2:
		case 6:
			spi_cmd = RT5677_SPI_WRITE_16;
			end = 2;
			break;
		case 0:
			spi_cmd = RT5677_SPI_WRITE_BURST;
			if (offset + SPI_BURST_LEN <= len)
				end = SPI_BURST_LEN;
			else {
				end = len - offset;
				end = (((end - 1) >> 3) + 1) << 3;
			}
			break;
		default:
			pr_err("Bad section alignment\n");
			return -EACCES;
		}

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		if (spi_cmd == RT5677_SPI_WRITE_BURST) {
			for (i = 0; i < end; i += 8) {
				write_buf[i + 12] = txbuf[offset + i + 0];
				write_buf[i + 11] = txbuf[offset + i + 1];
				write_buf[i + 10] = txbuf[offset + i + 2];
				write_buf[i +  9] = txbuf[offset + i + 3];
				write_buf[i +  8] = txbuf[offset + i + 4];
				write_buf[i +  7] = txbuf[offset + i + 5];
				write_buf[i +  6] = txbuf[offset + i + 6];
				write_buf[i +  5] = txbuf[offset + i + 7];
			}
		} else {
			unsigned int j = end + (SPI_HEADER - 1);
			for (i = 0; i < end; i++, j--) {
				if (offset + i < len)
					write_buf[j] = txbuf[offset + i];
				else
					write_buf[j] = 0;
			}
		}
		write_buf[end + SPI_HEADER] = spi_cmd;

		mutex_lock(&spi_mutex);
		status |= spi_write(g_spi, write_buf, end + SPI_HEADER + 1);
		mutex_unlock(&spi_mutex);

		offset += end;
	}

	return status;
}
EXPORT_SYMBOL_GPL(rt5677_spi_write);

static int rt5677_spi_probe(struct spi_device *spi)
{
	g_spi = spi;
	return 0;
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id rt5677_spi_acpi_id[] = {
	{ "RT5677AA", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, rt5677_spi_acpi_id);
#endif

static struct spi_driver rt5677_spi_driver = {
	.driver = {
		.name = "rt5677spi",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(rt5677_spi_acpi_id),
	},
	.probe = rt5677_spi_probe,
};
module_spi_driver(rt5677_spi_driver);

MODULE_DESCRIPTION("ASoC RT5677 SPI driver");
MODULE_AUTHOR("Oder Chiou <oder_chiou@realtek.com>");
MODULE_LICENSE("GPL v2");
