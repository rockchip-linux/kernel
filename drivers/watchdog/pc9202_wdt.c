/*
 * drivers/input/touchscreen/gslX680.c
 *
 * Copyright (c) 2012 Shanghai Basewin
 *	Guan Yuwei<guanyuwei@basewin.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>

#include <linux/version.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

#include <linux/blkdev.h>

#define PC9202_I2C_NAME 			"pc9202-wdt"
#define SW2001_REG_DECRYPT_CTRL 	0x00
#define SW2001_REG_WDT_CTRL 		0x01
#define WDT_KICK_S_0_64				0xc8
#define WDT_KICK_S_2_56				0xd8
#define WDT_KICK_S_10_24			0xe8
#define WDT_KICK_S_40_96			0xf8

static unsigned char demoBuffer[16];

struct sw2001 {
	struct i2c_client	*client;
	struct mutex		lock;
	//struct platform_device	*sw2001_device;
};

static struct sw2001 *the_sw2001;
static int major;
static struct class *cls;
static struct device *dev;

/*-------------------------------------------------------------------------*/
/* sw2001_write parameter:
 * reg:  
 * data: 
 */
int sw2001_write(unsigned char reg, unsigned char *buf, int len)
{
	int	 status=0;

	if (!the_sw2001)
		return -ENODEV;

	mutex_lock(&the_sw2001->lock);

	//status = i2c_smbus_write_byte_data(the_sw2001->client, reg, data);
	status = i2c_smbus_write_i2c_block_data(the_sw2001->client, reg, len, buf);

	mutex_unlock(&the_sw2001->lock);
	
	printk("%s: sw2001_write(0x%x,0x%x)\n", "pc9202", reg, *buf);

	if(status)
		printk("error.. status=0x%x\n",status);
		
	if(status>=len) return 0;
	else return status;
}

//********************************************************************************
// function    : iWriteByte(uint8_t addr, uint8_t data)
// description : write data to byte addr(internal register address in device)
// parameters  :
//                addr : register address in device
//                data:  data 
// return        : 
//               TRUE : success
//               FALSE : fail
//*********************************************************************************
bool iWriteByte(uint8_t addr, uint8_t data)
{
	//sw2001_write(addr, &data, 1);
	return sw2001_write(addr, &data, 1);
}

/* sw2001_read parameter:
 * reg:  
 * data: 
 */
int sw2001_read(unsigned char reg, unsigned char *buf, int len)
{
	int	 status=0;

	if (!the_sw2001)
		return -ENODEV;
	mutex_lock(&the_sw2001->lock);
	//status = i2c_smbus_read_byte_data(the_sw2001->client, reg);
	status = i2c_smbus_read_i2c_block_data(the_sw2001->client, reg, len, buf);
	mutex_unlock(&the_sw2001->lock);
	printk("%s: sw2001_read(0x%x) return 0x%x\n", "pc9202", reg, status);
	
	printk("status=0x%x\n",status);	
	
	if(status>=len) return 0;
	else return status;	
}
EXPORT_SYMBOL(sw2001_read);

//*********************************************************************************
// function    : iReadByte(uint8_t addr, uint8_t data)
// description :  read data from addr(internal register address in device)
// parameters  :
//                  addr : register address in device
//                  data:  data 
// return        : 
//               TRUE : success
//               FALSE : fail
//*********************************************************************************
bool iReadByte(uint8_t addr, uint8_t *data)
{
	//sw2001_read(addr, data, 1);
	return sw2001_read(addr, data, 1);
}

static int pc9202_wdt_suspend(struct device *dev)
{
    return 0;
}

static int pc9202_wdt_resume(struct device *dev)
{
    return 0;
}

int wdt_open(struct inode *inode, struct file *filp)
{
	//printk("wdt_open \r\n");
    return 0;
}

int wdt_release(struct inode *inode, struct file *filp)
{  
	return 0;
}  
ssize_t wdt_read(struct file *filp, char __user *buf, size_t count,loff_t *f_pos)
{
#if 0
	uint8_t reg_value;
	iReadByte(SW2001_REG_WDT_CTRL, &reg_value);
	/* 把数据复制到应用程序空间 */
    if (copy_to_user(buf,&reg_value,1))
    {
		count=-EFAULT;
	}
#endif
	//printk("wdt_read \r\n");
	return count;
}

ssize_t wdt_write(struct file *filp, const char __user *buf, size_t count,loff_t *f_pos)
{
	/* 把数据复制到内核空间 */
	uint8_t len = (int)count;
	if(len>2)
	{
		printk("wdt:inviald value too long\r\n");
		return count;
	}
	if (copy_from_user(demoBuffer+*f_pos, buf, count))
	{
		count = -EFAULT;
    }
	switch(demoBuffer[0])
	{
		case '0':iWriteByte(SW2001_REG_WDT_CTRL,WDT_KICK_S_0_64);break;//0.64
		case '1':iWriteByte(SW2001_REG_WDT_CTRL,WDT_KICK_S_2_56);break;//2.56
		case '2':iWriteByte(SW2001_REG_WDT_CTRL,WDT_KICK_S_10_24);break;//10.24
		case '3':iWriteByte(SW2001_REG_WDT_CTRL,WDT_KICK_S_40_96);break;//40.96
		default:printk("wdt:inviald value \r\n");break;
	}
	//printk("wdt_weite %c\r\n", demoBuffer[0]);
	return count;
}

struct file_operations wdt_fops = {
	.owner = THIS_MODULE,
	.read =	wdt_read,
	.write = wdt_write,
	.open = wdt_open,
	.release = wdt_release,
};

static int pc9202_wdt_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    //struct device_node *np = client->dev.of_node;
    //struct sw2001 *ts;
	struct sw2001	*pEnc;
	uint8_t reg_value; 

    if (!of_device_is_available(client->dev.of_node)) {
		return 0;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C functionality not supported\n");
        return -ENODEV;
    }

	pEnc = kzalloc(sizeof(struct sw2001), GFP_KERNEL); 
	if (!pEnc)
		return -ENOMEM;

	mutex_init(&pEnc->lock);
	pEnc->client = client;
	i2c_set_clientdata(client,pEnc);
	the_sw2001 = pEnc;
	
	if(0 != iReadByte(SW2001_REG_WDT_CTRL, &reg_value))
	{
		printk("====== i2c detect failed watchdog init err==%x=====\r\n", reg_value);
		goto err;
	}
	else
	{
		printk("====== i2c detect success watchdog init========\r\n");
	}

	major = register_chrdev(0, "wdt_crl", &wdt_fops);
    if (major < 0)
    {
		printk("Unable to register wdt character device !\n");
		goto err;
    }
	// 创建类
	cls = class_create(THIS_MODULE, "wdt_crl");
	// 创建设备节点
	dev = device_create(cls, NULL, MKDEV(major, 0), NULL, "wdt_crl");

    return 0;  	
err:
	kfree(pEnc);
	i2c_set_clientdata(client, NULL);
	the_sw2001 = NULL;
	return 0;
}

static int pc9202_wdt_remove(struct i2c_client *client)
{
	struct sw2001		*axp = i2c_get_clientdata(client);

	//PR_DEBUG("%s\n",__func__);

	kfree(axp);
	i2c_set_clientdata(client, NULL);
	the_sw2001 = NULL;
    return 0;
}

static struct of_device_id pc9202_wdt_ids[] = {
    {.compatible = "firefly,pc9202"},
    {}
};

static const struct i2c_device_id pc9202_wdt_id[] = {
    {PC9202_I2C_NAME, 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, pc9202_wdt_id);

static const struct dev_pm_ops pc9202_wdt_ops = {
	//.write		= pc9202_wdt_write,
	.suspend    = pc9202_wdt_suspend,
	.resume     = pc9202_wdt_resume,
};

static struct i2c_driver pc9202_wdt_driver = {
    .driver = {
        .name = PC9202_I2C_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(pc9202_wdt_ids),
        .pm = &pc9202_wdt_ops,
    },
    .probe		= pc9202_wdt_probe,
    .remove		= pc9202_wdt_remove,
    .id_table	= pc9202_wdt_id,
};

static int __init pc9202_wdt_init(void)
{
    int ret;
    ret = i2c_add_driver(&pc9202_wdt_driver);
    return ret;
}
static void __exit pc9202_wdt_exit(void)
{
    i2c_del_driver(&pc9202_wdt_driver);
    return;
}

module_init(pc9202_wdt_init);
module_exit(pc9202_wdt_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PC9202 watchdog driver");
MODULE_AUTHOR("zjy, zhengjunye123@126.com");
MODULE_ALIAS("platform:rk3399");
