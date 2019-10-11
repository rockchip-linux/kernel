#include <linux/module.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/device.h>
#include <linux/semaphore.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>

//#define Delay_50us(x) usleep(x*50)
#define	Delay_1ms(x)	msleep(x)
#define	Delay_50us(x)	usleep_range(50*x,50*x+20)

//#define	D0_GPIO_NO	41	//gpio2_a7
//#define	D1_GPIO_NO	42	//gpio2_b0


//static struct workqueue_struct *queue = NULL;
static struct work_struct work;
static u32 level_effect=0;
static unsigned char wg_level_high = 1;
static unsigned char wg_level_low = 0;
static int even =0;
static int odd =0;

struct Wiegand26{
	bool flag;
	char data[24];
};

struct Wiegand34{
	bool flag;
	char data[32];
};

struct Wiegand26 wiegand26;
struct Wiegand34 wiegand34;
static unsigned int D0;
static unsigned int D1;
static unsigned int mode_switch_gpio;

//static int major;

//static struct class *cls;
//static struct device *dev;

#if 0
static int wiegandOut_open(struct inode *inode, struct file *file){
	return 0;
}

static ssize_t wiegandOut_write(struct file *file, const char __user *buf,
	size_t count,loff_t *ppos){

	return 0;
}
#endif
#if 0
static struct file_operations wiegandOut_fops = {
	.owner = THIS_MODULE,
	.open = wiegandOut_open,
	.write = wiegandOut_write,
};
#endif

int Wiegand26_IntToChar(int data)
{
	int i = 0;
	odd=0;
	even=0;
	for(i = 0; i < 24; i++)
	{
		if ((1 << i) & data)
		{
			wiegand26.data[23-i]='1';
			if(i<12)
				odd++;
			else
				even++;
		}
		else
			wiegand26.data[23-i] = '0';
	}
	printk("in str %s \n", wiegand26.data);
	printk("check even=%d,odd=%d \n",even,odd);
	return 0;
}

int Wiegand34_IntToChar(int data)
{
	int i =0;
	odd=0;
	even=0;
	for (i = 0; i < 32; i++)
	{
		if ((1 << i) & data)
		{
			wiegand34.data[31-i] = '1';
			if(i<16)
				odd++;
			else
				even++;
		}
		else
			wiegand34.data[31-i] = '0';
	}
	printk("in str %s \n", wiegand34.data);
	printk("check even=%d,odd=%d \n",even,odd);
	return 0;
}

void gpio_reset(void)
{
	gpio_direction_output(D0,wg_level_high);
	gpio_direction_output(D1,wg_level_high);
}

void Output_DATA0(void)
{
	gpio_set_value(D0,wg_level_low);
	Delay_50us(8);
	gpio_set_value(D0,wg_level_high);
}

void Output_DATA1(void)
{
	gpio_set_value(D1,wg_level_low);
	Delay_50us(8);
	gpio_set_value(D1,wg_level_high);
}

void Send_Wiegand34(unsigned char *str)
{
	//unsigned char one_num = 0;
	int i =0;
	// WG_DATA0 = 1;
	// WG_DATA1 = 1;
	gpio_reset();
	Delay_1ms(2);
	if(even % 2)
	{
		//  WG_DATA1 = 0;                    /*偶校验位为1*/
		printk("1");
		//   Delay_50us(8);
		//  WG_DATA1 = 1;
		Output_DATA1();
	}
	else
	{
		Output_DATA0();
		// WG_DATA0 = 0;                   /*偶校验位为0*/
		printk("0");
		//Delay_50us(8);
		// WG_DATA0 = 1;
	}

	Delay_1ms(2);/*延时2ms*/
	for(i = 0;i < 32;i++)
	{
		//   WG_DATA0 = 1;
		//   WG_DATA1 = 1;
		if(str[i] & 0x01)
		{
			// WG_DATA1 = 0;
			// Delay_50us(8);
			// WG_DATA1 = 1;
			printk("1");
			Output_DATA1();
		}
		else
		{
			// WG_DATA0 = 0;
			// Delay_50us(8);
			// WG_DATA0 = 1;
			printk("0");
			Output_DATA0();
		}
		//(*(long*)&str[0]) <<= 1;
		Delay_1ms(2);/*延时2ms*/
	}
	//  WG_DATA0 = 1;
	//  WG_DATA1 = 1;
	if(odd % 2)
	{
	//    WG_DATA0 = 0;
	//    Delay_50us(8);
	//   WG_DATA0 = 1;
		printk("0");
		Output_DATA0();
	}
	else
	{
		//  WG_DATA1 = 0;
		//  Delay_50us(8);
		//  WG_DATA1 = 1;
		printk("1");
		Output_DATA1();
	}
	Delay_1ms(2);
}
/*
韦根26发送函数
*/
void Send_Wiegand26(unsigned char *str)
{
	//unsigned char one_num = 0;
	int i =0;
	printk("\nSend_Wiegand26 start \n");
	//  WG_DATA0 = 1;
	//  WG_DATA1 = 1;
	gpio_reset();
	Delay_1ms(2);
	if(even % 2)
	{
	//  WG_DATA1 = 0;                    /*偶校验位为1*/
	//  Delay_50us(8);
	//  WG_DATA1 = 1;
		printk("1");
		Output_DATA1();
	}
	else
	{
	//  WG_DATA0 = 0;                   /*偶校验位为0*/
	//  Delay_50us(8);
	//  WG_DATA0 = 1;
		Output_DATA0();
		printk("0");
	}

	Delay_1ms(2);/*延时2ms*/
	for(i = 0;i < 24;i++)
	{
	//  WG_DATA0 = 1;
	//  WG_DATA1 = 1;
		if(str[i] & 0x01)
		{
		//   WG_DATA1 = 0;
		//   Delay_50us(8);
		//   WG_DATA1 = 1;
			printk("1");
			Output_DATA1();
		}
		else
		{
			//   WG_DATA0 = 0;
			//Delay_50us(8);
			//WG_DATA0 = 1;
			printk("0");
			Output_DATA0();
		}
		//  (*(long*)&str[0]) <<= 1;
		Delay_1ms(2);/*延时2ms*/
	}
	// WG_DATA0 = 1;
	// WG_DATA1 = 1;
	if(odd % 2)
	{
	//  WG_DATA0 = 0;
	//  Delay_50us(8);
	//  WG_DATA0 = 1;
		printk("0");
		Output_DATA0();
	}
	else
	{
		//  WG_DATA1 = 0;
			//  Delay_50us(8);
		//  WG_DATA1 = 1;
		printk("1");
		Output_DATA1();
	}
	Delay_1ms(2);
	printk("\n Send_Weigand26 end \n");
}

static void work_handler(struct work_struct *data)
{
	printk(KERN_ALERT"work handler function.\n");
	if(wiegand26.flag)
	{
		Send_Wiegand26(wiegand26.data);
		wiegand26.flag =false;
	}
	else if(wiegand34.flag)
	{
		Send_Wiegand34(wiegand34.data);
		wiegand34.flag =false;
	}
	else
	{
		printk(KERN_ALERT"work handler function error.\n");
	}
	printk(KERN_ALERT"work handler end.\n");
}

static ssize_t gpiod0_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int value;
	value = gpio_get_value(D0);
	if(level_effect)
	{
		value = !value;
	}
	return sprintf(buf,"%d\n", value);
}

static ssize_t gpiod0_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	gpio_direction_output(D0,wg_level_high);
	if('0' == buf[0])
	{
		gpio_set_value(D0,wg_level_low);
	}
	else
	{
		gpio_set_value(D0,wg_level_high);
	}
	return count;
}

static ssize_t gpiod1_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	
	unsigned int value;
	value = gpio_get_value(D1);
	if(level_effect)
	{
		value = !value;
	}
	return sprintf(buf,"%d\n", value);
}

static ssize_t gpiod1_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	gpio_direction_output(D1,wg_level_high);
	if('0' == buf[0])
	{
		gpio_set_value(D1,wg_level_low);
	}
	else
	{
		gpio_set_value(D1,wg_level_high);
	}
	return count;
}

static ssize_t mode_switch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	
	return sprintf(buf,"%d\n", gpio_get_value(mode_switch_gpio));
}

static ssize_t mode_switch_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	gpio_direction_output(mode_switch_gpio,1);
	if('0' == buf[0])
	{
		gpio_set_value(mode_switch_gpio,0);
	}
	else
	{
		gpio_set_value(mode_switch_gpio,1);
	}
	return count;
}

static ssize_t Wiegand26_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%s\n", wiegand26.data);
}

static ssize_t Wiegand26_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	Wiegand26_IntToChar(val);
	wiegand26.flag = true;
	INIT_WORK(&work,work_handler);
	schedule_work(&work);
	return count;
}

static ssize_t Wiegand34_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	//printk("zjy : Wiegand34_show\r\n");
	return sprintf(buf,"%s\n", wiegand34.data);
}

static ssize_t Wiegand34_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	//printk("zjy : Wiegand34_storer\n");
	Wiegand34_IntToChar(val);
	wiegand34.flag = true;
	INIT_WORK(&work,work_handler);
	schedule_work(&work);
	return count;
}

static DEVICE_ATTR(wiegand26,0664,Wiegand26_show, Wiegand26_store);
static DEVICE_ATTR(wiegand34,0664,Wiegand34_show, Wiegand34_store);
static DEVICE_ATTR(D0, 0644, gpiod0_show, gpiod0_store);
static DEVICE_ATTR(D1, 0644, gpiod1_show, gpiod1_store);
static DEVICE_ATTR(mode_switch, 0664, mode_switch_show, mode_switch_store);

static struct attribute *wiegandOut_attributes[] = {
	&dev_attr_wiegand26.attr,
	&dev_attr_wiegand34.attr,
	&dev_attr_D0.attr,
	&dev_attr_D1.attr,
	&dev_attr_mode_switch.attr,
	NULL
};

static struct attribute_group wiegandOut_attribute_group = {
	.attrs = wiegandOut_attributes
};

//static void wiegandOut_release(struct device *dev){
//}

static struct of_device_id wiegandOut_of_match[] = {
	{ .compatible = "firefly,wiegandout" },
	{ }
};
MODULE_DEVICE_TABLE(of, wiegandOut_of_match);

static int wiegandOut_probe(struct platform_device *pdev){
	int ret =0;
	printk("wiegandOut_probe\n");
	// 注册设备驱动 创建设备节点
	//major = register_chrdev(0, "wiegandOut", &wiegandOut_fops);
	// 创建类
	//cls = class_create(THIS_MODULE, "wiegandOut");
	// 创建设备节点
	//dev = device_create(cls, NULL, MKDEV(major, 0), NULL, "wiegandOut");

	ret = of_get_named_gpio(pdev->dev.of_node, "gpio_d0", 0);                                                                                       
	if (ret) {
		D0 = ret;
		printk("wiegandOut_probe gpiod0 %d \r\n", D0);
	}
	else
	{
		if (ret != -EPROBE_DEFER)
			printk("%s() Can not read property gpio_d0\n", __FUNCTION__);
		goto err_dt;
	}
	ret = of_get_named_gpio(pdev->dev.of_node, "gpio_d1", 0);                                                                                       
	if (ret) {
		D1 = ret;
		printk("wiegandOut_probe gpiod1 %d \r\n", D1);
	}
	else
	{
		if (ret != -EPROBE_DEFER)
			printk("%s() Can not read property gpio_d1\n", __FUNCTION__);
		goto err_dt;
	}

	ret = of_get_named_gpio(pdev->dev.of_node, "gpio_mode_switch", 0);
	if (ret) {
		mode_switch_gpio = ret;
		printk("wiegandOut_probe gpio_mode_switch %d \r\n", mode_switch_gpio);
	}
	else
	{
		if (ret != -EPROBE_DEFER)
			printk("%s() Can not read property gpio_d1\n", __FUNCTION__);
		goto err_dt;
	}
	
	if (!of_property_read_u32(pdev->dev.of_node, "level_effect", &level_effect))
		printk("wiegandOut_probe level-effect %d\r\n", level_effect);
	if(level_effect)
	{
		wg_level_high = 0;
		wg_level_low = 1;
	}else
	{
		wg_level_high = 1;
		wg_level_low = 0;
	}
	
	ret = sysfs_create_group(&pdev->dev.kobj, &wiegandOut_attribute_group);
	wiegand26.flag=false;
	wiegand34.flag=false;
	printk("wiegandOut_probe, end ret=%d \n",ret);

err_dt:
	return ret;
}

static int wiegandOut_remove(struct platform_device *pdev){
	printk("wiegandOut_remove, \n");
	// 删除设备节点
	//device_unregister(dev);
	// 销毁类
	//class_destroy(cls);
	// 取消注册设备驱动
	//unregister_chrdev(major, "wiegandOut");
	// 取消内存映射
	gpio_reset();
	return 0;
}

struct platform_driver wiegandOut_drv = {
	.probe	= wiegandOut_probe,//匹配到dev之后调用probe
	.remove = wiegandOut_remove,
	.driver = {
		.name = "wiegandOut",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(wiegandOut_of_match),
	},
};

static int __init wiegandOut_init(void)
{
	int ret =0;
	printk(KERN_ALERT"ace test_init function.\n");
	//queue=create_singlethread_workqueue("weigandSend");/*创建一个单线程的工作队列*/
	//if(!queue)
		//goto err;
	// INIT_WORK(&work,work_handler);
	//  schedule_work(&work);
	ret=platform_driver_register(&wiegandOut_drv);
	printk(KERN_ALERT"ace test_init end.ret=%d \n",ret);
	return 0;
//err:
	//return -1;
}

static void __exit wiegandOut_exit(void)
{
	platform_driver_unregister(&wiegandOut_drv);
	//destroy_workqueue(queue);
}
MODULE_LICENSE("GPL");
module_init(wiegandOut_init);
module_exit(wiegandOut_exit);
