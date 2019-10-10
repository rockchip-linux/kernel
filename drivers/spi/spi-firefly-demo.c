/*
 * Driver for pwm demo on Firefly board.
 *
 * Copyright (C) 2016, Zhongshan T-chip Intelligent Technology Co.,ltd.
 * Copyright 2006  Sam Chan
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#define FIREFLY_SPI_READ_ID_CMD 0x9F

#define FIREFLY_SPI_PRINT_ID(rbuf) \
        do { \
                if (status == 0) \
                        dev_dbg(&spi->dev, "%s: ID = %02x %02x %02x %02x %02x\n", __FUNCTION__, \
                                rbuf[0], rbuf[1], rbuf[2], rbuf[3], rbuf[4]); \
                else \
                        dev_err(&spi->dev, "%s: read ID error\n", __FUNCTION__); \
        }while(0)

static int firefly_spi_read_w25x_id_0(struct spi_device *spi)
{       
        int     status;
        char tbuf[]={FIREFLY_SPI_READ_ID_CMD};
        char rbuf[5];

        struct spi_transfer     t = {
                .tx_buf         = tbuf,
                .len            = sizeof(tbuf),
        };

        struct spi_transfer     r = {
                .rx_buf         = rbuf,
                .len            = sizeof(rbuf),
        };
        struct spi_message      m;

        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        spi_message_add_tail(&r, &m);
        status = spi_sync(spi, &m);

        printk("%s ID = %02x %02x %02x %02x %02x\n", __FUNCTION__, rbuf[0], rbuf[1], rbuf[2], rbuf[3], rbuf[4]);
        return status;
}


static int firefly_spi_read_w25x_id_1(struct spi_device *spi)
{
        int     status;
        char tbuf[] = {FIREFLY_SPI_READ_ID_CMD};
        char rbuf[5];

        status = spi_write_then_read(spi, tbuf, sizeof(tbuf), rbuf, sizeof(rbuf));
        printk("%s ID = %02x %02x %02x %02x %02x\n", __FUNCTION__, rbuf[0], rbuf[1], rbuf[2], rbuf[3], rbuf[4]);
        return status;
}

static int firefly_spi_probe(struct spi_device *spi)
{
    int ret = 0;
    struct device_node __maybe_unused *np = spi->dev.of_node;

    dev_dbg(&spi->dev, "Firefly SPI demo program\n");
    printk("firefly spi demo\r\n");
        if(!spi)        
                return -ENOMEM;

        dev_dbg(&spi->dev, "firefly_spi_probe: setup mode %d, %s%s%s%s%u bits/w, %u Hz max\n",
                        (int) (spi->mode & (SPI_CPOL | SPI_CPHA)),
                        (spi->mode & SPI_CS_HIGH) ? "cs_high, " : "",
                        (spi->mode & SPI_LSB_FIRST) ? "lsb, " : "",
                        (spi->mode & SPI_3WIRE) ? "3wire, " : "",
                        (spi->mode & SPI_LOOP) ? "loopback, " : "",
                        spi->bits_per_word, spi->max_speed_hz);

        firefly_spi_read_w25x_id_0(spi);
        firefly_spi_read_w25x_id_1(spi);
        
    return ret;
}

static int firefly_spi_remove(struct spi_device *spi)
{
    return 0;
}


static struct of_device_id firefly_match_table[] = {
        { .compatible = "firefly,rk3399-spi",},
        {},
};

static struct spi_driver firefly_spi_driver = {
        .driver = {
                .name = "firefly-spi",
                .of_match_table = of_match_ptr(firefly_match_table),
        },
        .probe = firefly_spi_probe,
        .remove = firefly_spi_remove,
};

static int firefly_spi_init(void)
{
        int retval;
        retval = spi_register_driver(&firefly_spi_driver);
        printk(KERN_ALERT "register firefly_spi_init spi return v = :%d\n",retval);
        return retval;
}

module_init(firefly_spi_init);

static void firefly_spi_exit(void)
{
        spi_unregister_driver(&firefly_spi_driver);
}
module_exit(firefly_spi_exit);

MODULE_AUTHOR("zhansb <service@t-firefly.com>");
MODULE_DESCRIPTION("Firefly SPI demo driver");
MODULE_ALIAS("platform:firefly-spi");
MODULE_LICENSE("GPL");
