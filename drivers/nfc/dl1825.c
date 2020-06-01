/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
//arm64 has no these header files
//#include <mach/gpio.h>
//#include <mach/irqs.h>
//#include <mach/board.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/input.h>

#define DL1825_NAME 		"dl1825"
#define MAX_BUFFER_SIZE		512
#define NFC_DATA_HEAD 		0x91
#define NFC_DATA_TAIL 		0x0d
#define NFC_DATA_MAX_LEN 	20
#define NFC_NAME			"firefly_nfc"
#define KEY_NFC_IN			0x2f0
#define NFC_DATA_REG		0x91
#define KEY_PRESS			1
#define KEY_RELEASE			0

struct input_dev *nfc_input_temp = NULL;
static int nfc_value;

struct dl1825_dev
{
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	dl1825_device;

	unsigned int		irq_gpio;
};

typedef struct nfc_data_struct
{
	unsigned char head;
	unsigned char len;
	unsigned char type;
	unsigned char *data;
	unsigned char crc8;
	unsigned char tail;
}nfc_data_struct_t;

unsigned char firefly_cal_crc8(unsigned char *setence, unsigned char len)
{
    unsigned char checksum = 0;
	int i;
    for(i=0; i<len; i++)
    {
        checksum ^= (unsigned char)* setence++;
    }

    return checksum;
}

static irqreturn_t dl1825_dev_irq_handler(int irq, void *dev_id)
{
	//printk("zjy:dl1825_dev_irq_handler \r\n");
	input_report_key(nfc_input_temp, KEY_NFC_IN, KEY_PRESS);
	input_report_key(nfc_input_temp, KEY_NFC_IN, KEY_RELEASE);
	input_sync(nfc_input_temp);

	return IRQ_HANDLED;
}

static ssize_t dl1825_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	struct dl1825_dev *dl1825_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	unsigned char data_len, crc8;
	int ret;
	struct device * dev = dl1825_dev->dl1825_device.this_device ;
	nfc_data_struct_t cur_nfc_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	//printk("%s : zjy reading %zu bytes.\n", __func__, count);

	mutex_lock(&dl1825_dev->read_mutex);
#if 1
	/* Read data */
	ret = i2c_smbus_read_i2c_block_data(dl1825_dev->client, NFC_DATA_REG, NFC_DATA_MAX_LEN, tmp);
	mutex_unlock(&dl1825_dev->read_mutex);
#if 0
	for(i = 0; i < 20; i++){
		printk(" %02X", tmp[i]);
	}
	printk("\n");
#endif
	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}

	memcpy(&cur_nfc_data, tmp, sizeof(cur_nfc_data.head)+sizeof(cur_nfc_data.len)+sizeof(cur_nfc_data.type));
	if(NFC_DATA_HEAD != cur_nfc_data.head
			|| cur_nfc_data.len > NFC_DATA_MAX_LEN)
	{
		dev_warn(dev, "%s: i2c_master_recv data err[head] \n", __func__);
		return 0;
	}

	data_len = cur_nfc_data.len - sizeof(cur_nfc_data.len) - sizeof(cur_nfc_data.type) - sizeof(cur_nfc_data.crc8);
	crc8 = firefly_cal_crc8(&tmp[0], sizeof(cur_nfc_data.head)+sizeof(cur_nfc_data.len)+sizeof(cur_nfc_data.type)+data_len);
	cur_nfc_data.crc8 = tmp[sizeof(cur_nfc_data.head)+sizeof(cur_nfc_data.len)+sizeof(cur_nfc_data.type)+data_len];

	if(crc8 != cur_nfc_data.crc8)
	{
		pr_err("%s: i2c_master_recv data err[crc8] %x %x \n", __func__, crc8, cur_nfc_data.crc8);
		return -EIO;
	}
	cur_nfc_data.tail = tmp[sizeof(cur_nfc_data.head)+sizeof(cur_nfc_data.len)+sizeof(cur_nfc_data.type)+data_len+sizeof(cur_nfc_data.crc8)];
	if(NFC_DATA_TAIL != cur_nfc_data.tail)
	{
		pr_err("%s: i2c_master_recv data err[tail] \n", __func__);
		return -EIO;
	}

	cur_nfc_data.data = (unsigned char *)kzalloc(data_len, GFP_KERNEL);
	if(NULL == cur_nfc_data.data)
	{
		pr_err("%s : failed to malloc mem\n", __func__);
		return -EIO;
	}
	memcpy(cur_nfc_data.data, &tmp[sizeof(cur_nfc_data.head)+sizeof(cur_nfc_data.len)+sizeof(cur_nfc_data.type)], data_len);
#if 1
	if (copy_to_user(buf, cur_nfc_data.data, data_len)) {
		pr_err("%s : failed to copy to user space\n", __func__);
	}
#endif

#if 0
	printk("zjy len = %d type = %d IFD->PC:", data_len, cur_nfc_data.type);
	for(i = 0; i < data_len; i++){
		printk(" %02X", cur_nfc_data.data[i]);
	}
	printk("\n");
#endif

#endif

	kfree(cur_nfc_data.data);
	return data_len;

//fail:
	//mutex_unlock(&dl1825_dev->read_mutex);
	//return ret;
}

static ssize_t dl1825_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
#if 0
	struct dl1825_dev  *dl1825_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	dl1825_dev = filp->private_data;
	printk("%s : zjy write %zu bytes.\n", __func__, count);
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("%s : writing %zu bytes.\n", __func__, count);
	/* Write data */
	ret = i2c_master_send(dl1825_dev->client, tmp, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
		goto exit;
	}
	pr_debug("PC->IFD:");
	for(i = 0; i < count; i++){
		pr_debug(" %02X", tmp[i]);
	}
	pr_debug("\n");

exit:
	return ret;
#endif
	return 0;
}

static int dl1825_dev_open(struct inode *inode, struct file *filp)
{
#if 1
	struct dl1825_dev *dl1825_dev = container_of(filp->private_data, struct dl1825_dev, dl1825_device);

	filp->private_data = dl1825_dev;

	dev_dbg (&dl1825_dev->client->dev, "%s: %d,%d\n", __func__, imajor(inode), iminor(inode));
#endif
	return 0;
}

static const struct file_operations dl1825_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= dl1825_dev_read,
	.write	= dl1825_dev_write,
	.open	= dl1825_dev_open,
};

//extern bool firefly_hwversion_in_range(const struct device_node *device);

static int dl1825_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	int irqn = 0,timer =3;
	int error;
	struct dl1825_dev *dl1825_dev;
	char buffer = 0;
	struct device_node *np = client->dev.of_node;
	unsigned long irq_flags;
	unsigned int irq_gpio;
	struct input_dev *nfc_input = NULL;


	// if (!firefly_hwversion_in_range(np))
	// 	return -EINVAL;

	while (timer > 0) {
		ret = i2c_master_recv(client, &buffer, 1);
		if (ret >= 0)
			break;
		timer--;
		msleep(100);
	}

	if (ret < 0)
	{
		printk("dl1825 not exist\r\n");
		return -EINVAL;
	}

	nfc_input = input_allocate_device();
	if (!nfc_input) {
		printk("input_allocate_device failed!\n");
		return -ENOMEM;
	}

	nfc_input->name = NFC_NAME;
	nfc_input->id.bustype = BUS_HOST;

	nfc_input->phys = "gpio-keys/nfc";
	nfc_input->id.vendor = 0x0001;
	nfc_input->id.product = 0x0001;
	nfc_input->id.version = 0x0100;

	nfc_input->dev.parent = &client->dev;
	input_set_drvdata(nfc_input, &nfc_value);
	nfc_input_temp = nfc_input;

	input_set_capability(nfc_input_temp, EV_KEY, KEY_OK);
	input_set_capability(nfc_input_temp, EV_KEY, KEY_NFC_IN);
	error = input_register_device(nfc_input);
	if (error)
		pr_err("nfc: register input device err, error: %d\n", error);

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	printk("zjy dl1825_probe \r\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	dl1825_dev = kzalloc(sizeof(*dl1825_dev), GFP_KERNEL);
	if (dl1825_dev == NULL) {
		dev_err(&client->dev, "%s: failed to allocate memory for pn548_dev\n", __func__);
		return -ENOMEM;
	}

	dl1825_dev->client = client;

	irq_gpio = of_get_named_gpio_flags(np, "nfc-irq-gpio", 0,(enum of_gpio_flags *)&irq_flags);
	if (!gpio_is_valid(irq_gpio))
	{
		return -EINVAL;
	}

	/*irq_gpio*/
	if (gpio_is_valid(irq_gpio))
	{
		 ret = gpio_request(irq_gpio, "nfc_int");
		 if (ret)
		 {
			   dev_err(&client->dev, "%s: unable to request gpio [%d]\n", __func__, irq_gpio);
			   goto err_irq;
		 }
		 ret = gpio_direction_input(irq_gpio);
		 if (ret) {
			   dev_err(&client->dev, "%s: unable to set direction for gpio [%d]\n", __func__, irq_gpio);
			   goto err_irq;
		 }

		 irqn = gpio_to_irq(irq_gpio);
		 if (irqn < 0){
			   ret = irqn;
			   goto err_irq;
		 }

		 client->irq = irqn;

	}else{
		 dev_err(&client->dev, "%s: irq_gpio not provided\n", __func__);
		 goto err_irq;
	}

	dl1825_dev->irq_gpio = irq_gpio;

	/* init mutex */
	mutex_init(&dl1825_dev->read_mutex);

	dl1825_dev->dl1825_device.minor = MISC_DYNAMIC_MINOR;
	dl1825_dev->dl1825_device.name = DL1825_NAME;
	dl1825_dev->dl1825_device.fops = &dl1825_dev_fops;

	ret = misc_register(&dl1825_dev->dl1825_device);
	if (ret) {
		dev_err(&client->dev, "%s : misc_register failed\n", __func__);
		goto err_misc_register;
	}

	/* NFC_INT IRQ */
	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	ret = request_irq(client->irq, dl1825_dev_irq_handler, irq_flags, client->name, dl1825_dev);
	if (ret) {
              dev_err(&client->dev, "%s: request_irq failed\n", __func__);
              goto err_request_irq_failed;
	}

	i2c_set_clientdata(client, dl1825_dev);
	printk("zjy dl1825 exited successfully\n");
	return 0;

err_request_irq_failed:
	misc_deregister(&dl1825_dev->dl1825_device);
err_misc_register:
	mutex_destroy(&dl1825_dev->read_mutex);
err_irq:
	gpio_free(irq_gpio);
    kfree(dl1825_dev);

	return ret;
}

static int dl1825_remove(struct i2c_client *client)
{
	struct dl1825_dev *dl1825_dev;

	dl1825_dev = i2c_get_clientdata(client);

	free_irq(client->irq, dl1825_dev);
	misc_deregister(&dl1825_dev->dl1825_device);
	mutex_destroy(&dl1825_dev->read_mutex);

	gpio_free(dl1825_dev->irq_gpio);
	kfree(dl1825_dev);

	return 0;
}

static const struct i2c_device_id dl1825_id[] = {
	{ DL1825_NAME, 0 },
	{ }
};
static struct of_device_id dl1825_match_table[] = {
	{ .compatible = "firefly,dl1825"},
	{ },
};

static struct i2c_driver dl1825_driver = {
	.id_table	= dl1825_id,
	.probe		= dl1825_probe,
	.remove		= dl1825_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DL1825_NAME,
		.of_match_table = dl1825_match_table,
	},
};

/*
 * module load/unload record keeping
 */
static int __init dl1825_dev_init(void)
{
	pr_info("Loading dl1825 driver\n");
	return i2c_add_driver(&dl1825_driver);
}
module_init(dl1825_dev_init);

static void __exit dl1825_dev_exit(void)
{
	pr_info("Unloading dl1825 driver\n");
	i2c_del_driver(&dl1825_driver);
}
module_exit(dl1825_dev_exit);

MODULE_AUTHOR("Ernest Li");
MODULE_DESCRIPTION("NFC DL1825 driver");
MODULE_LICENSE("GPL");
