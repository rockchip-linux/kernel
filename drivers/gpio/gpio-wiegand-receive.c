#include <linux/module.h>
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
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/fcntl.h>

#define WIEGAND_MAJOR 243        
//delay 33ms to convert data
#define TIMER_DELAY HZ/4
#define DEVICE_NAME "wiegand" 

#define WG_IOC_MAGIC 'k'

#define WG_IOCGETFUN 	_IOR(WG_IOC_MAGIC, 1, int)  //
#define WG_IOC_FUN_IN 	_IOR(WG_IOC_MAGIC, 2, int)  //设置为普通输入IO
#define WG_IOC_FUN_WG 	_IOR(WG_IOC_MAGIC, 3, int)  //设置为韦根输入IO
#define WG_IOCGETD0		_IOR(WG_IOC_MAGIC, 4, int)  //获取普通IO数据
#define WG_IOCGETD1		_IOR(WG_IOC_MAGIC, 5, int)  //
#define WG_IOCGETWG		_IOR(WG_IOC_MAGIC, 6, int)  //获取韦根数据

#define WG_IOC_MAXNR 6

 
bool TIMEER_FLAG = false;   
bool RF_OPEN_FLAG = false;
static struct class *cdev_class; 
//dev_t dev = 0;


struct wiegand_dev {
	char wiegand[34];    //Wiegand-26
	int hid;
	int pid;
	int count;    //Global Counter
	struct cdev cdev;
	struct semaphore sem;
	struct completion receive_completion;
	struct timer_list wiegand_timer;
	struct work_struct pen_event_work;
	struct workqueue_struct *ts_workqueue;

	int gpio_d0;
	int gpio_d1;
	int d0_irq;
	int d1_irq;
	
};

enum Wiegand_FLAG {
	WG26=0,
	WG34,
	MAX_NUM
}g_flag;

enum FUN_FLAG {
	DEF_NULL =0,
	IO_FUN,
	WG_FUN,
}fun_flag;

static struct wiegand_dev *rf_card;

typedef struct _WiegandIn {
	unsigned int cardno;
	enum Wiegand_FLAG flag;  //0:WG26 ,  1:WG34
}WiegandIn;

WiegandIn wiegandin;

//应该除了初始化外，只能有这一个地方能够将rf_card->count置为0，其余地方置为0皆不合法
static char convert_data(void) {
	int i,even,odd;
    int cardno ; 

	//偶校验
    even = 0;
    for(i = 1; i < 13; i++) {
	    if(rf_card->wiegand[i] == 1)	
	        even = (~even) & 0x01; 
    }
    if(even != rf_card->wiegand[0]) {
	    rf_card->count = 0;
	    goto error;
    }

	//奇校验  
   	odd = 1;
    for(i = 13; i < 25; i++) {
        if(rf_card->wiegand[i] == 1)			 
	        odd = (~odd) & 0x01;	      
    }
    if(odd != rf_card->wiegand[25]) {
        rf_card->count = 0;
            goto error;
    }

	//奇偶校验通过
	rf_card->hid = 0;
	for(i = 1 ;i <= 8; i++)//hid转换
		rf_card->hid =  rf_card->hid << 1 | rf_card->wiegand[i];	

	rf_card->pid = 0;
	for(i = 9 ; i <= 24; i++)//pid转换
		rf_card->pid =  rf_card->pid << 1 | rf_card->wiegand[i];	
	  
    cardno = rf_card->hid << 16 | rf_card->pid;
    //rf_card->count = 0;
	g_flag=WG26;
    printk("%s,%s,%d,cardno=(%x,%d)\n",__FILE__,__func__,__LINE__,cardno,cardno);
	return 0;
	
error:
	printk("Parity Efficacy Error!\n");
	return -1;
}


static char convert_data34(void) {
	int i, even, odd;
    unsigned int cardno ; 

	//偶校验
    even = 0;
    for(i = 1; i < 17; i++) {
	    if(rf_card->wiegand[i] == 1)	
	    even = (~even) & 0x01; 
    }
    if(even != rf_card->wiegand[0]) {
	    rf_card->count = 0;
	    goto error;
    }	

	//奇校验  
   	odd = 1;
    for(i = 17; i < 33; i++) {
	    if(rf_card->wiegand[i] == 1)			 
		    odd = (~odd) & 0x01;	      
    }
    if(odd != rf_card->wiegand[33]) {
	    rf_card->count = 0;
	        goto error;
    }

	//奇偶校验通过
	rf_card->hid = 0;
	for(i = 1 ; i <= 16; i++)//hid转换
		rf_card->hid =  rf_card->hid << 1 |rf_card->wiegand[i];	

	rf_card->pid = 0;
	for(i = 17 ; i <= 32; i++)//pid转换
		rf_card->pid = rf_card->pid << 1 |rf_card->wiegand[i];	
	  
    cardno = rf_card->hid << 16 | rf_card->pid;
    //rf_card->count = 0;
	g_flag = WG34;
	printk("%s,%s,%d,cardno=(%x,%u),hid=%d pid=%d \n",__FILE__,__func__,__LINE__,cardno,cardno,rf_card->hid,rf_card->pid); 
	return 0;
	
error:
	printk("Parity Efficacy Error!\n");
	return -1;
}


//static void wiegand_pen_irq_work(struct work_struct *work)

static void wiegand_do_timer(unsigned long arg) {
	//wait_for_completion(&(rf_card->receive_completion));//等待维根数据传送完毕才进行转换

    printk("%s,%s,%d,rf_card->count=%d arg=%lu\n",__FILE__,__func__,__LINE__,rf_card->count,arg);
	g_flag = MAX_NUM;
	disable_irq(rf_card->d0_irq);
	disable_irq(rf_card->d1_irq);//防止wieg_data在转换期间发生变化
	
	if(rf_card->count == 26) {
		convert_data();
	} else if(rf_card->count == 34) {
		convert_data34();
	}
	up(&rf_card->sem);
	memset(rf_card->wiegand, 0x00, 34);	
	rf_card->count = 0 ;
	//TIMEER_FLAG = false;
	enable_irq(rf_card->d0_irq);
	enable_irq(rf_card->d1_irq);
	TIMEER_FLAG = false;
}

static void enable_irqs(void) {	
	enable_irq(rf_card->d0_irq);
	enable_irq(rf_card->d1_irq);
	printk("%s:the irq is setup!\n",__func__);
}

static void disable_irqs(void) {	
	disable_irq(rf_card->d0_irq);
	disable_irq(rf_card->d1_irq);
	printk("%s:the irq is setup!\n",__func__);
}

static irqreturn_t wiegand_irq0(int irq, void *dev_id) {
	if(fun_flag==IO_FUN)
		return IRQ_HANDLED;
	//printk("%s:the irq0 ! start\n",__func__);
	disable_irq_nosync(rf_card->d0_irq);
	udelay(5);

    rf_card->wiegand[rf_card->count] = 0;
    rf_card->count ++;
    udelay(100);
	
	enable_irq(rf_card->d0_irq);

   	if(TIMEER_FLAG == false) {
		rf_card->wiegand_timer.expires = jiffies + TIMER_DELAY;
		add_timer(&rf_card->wiegand_timer);
		TIMEER_FLAG = true;
	}
	return IRQ_HANDLED;
}

static irqreturn_t wiegand_irq1(int irq, void *dev_id) {
	if(fun_flag == IO_FUN)
		return IRQ_HANDLED;
	disable_irq_nosync(rf_card->d1_irq);//必须用这个函数
   
	udelay(5);

	rf_card->wiegand[rf_card->count] = 1;
	rf_card->count ++;
	udelay(100);
	
	enable_irq(rf_card->d1_irq);

 	if(TIMEER_FLAG== false) {
		rf_card->wiegand_timer.expires = jiffies + TIMER_DELAY;
		add_timer(&rf_card->wiegand_timer);
		TIMEER_FLAG = true;
	}
	return IRQ_HANDLED;
}

static ssize_t rfcd_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos) {    	
    //int max_count ;  
    //char cardno[12];
	WiegandIn data;

	if(TIMEER_FLAG)   //中断处理中，禁止读数据
		return -1 ;
	
	disable_irq(rf_card->d0_irq);
	disable_irq(rf_card->d1_irq);//防止wieg_data在转换期间发生变化

	data.cardno=rf_card->hid << 16 | rf_card->pid;

	data.flag=g_flag;
    //*ppos += size;
	memset(rf_card->wiegand, 0x00, 34);
	rf_card->hid = 0;
	rf_card->pid = 0;
	rf_card->count = 0;
	g_flag = MAX_NUM;
	enable_irq(rf_card->d0_irq);
	enable_irq(rf_card->d1_irq);
	if ( copy_to_user((WiegandIn __user *)buf, &data, sizeof(WiegandIn))) {
        return -EFAULT;
    }
	return size;
}

static ssize_t rfcd_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos) {
	return 0;
}

static int rfcd_open(struct inode *inode, struct file *filp) {
	return 0;
}

static void free_irqs(void) {
	disable_irq(rf_card->d0_irq);
	disable_irq(rf_card->d1_irq);	
	free_irq(rf_card->d0_irq, rf_card);
	free_irq(rf_card->d1_irq, rf_card);
}

int rfcd_release(struct inode *inode, struct file *filp) {
//	disable_irq(rf_card->d0_irq);
//	disable_irq(rf_card->d1_irq);

	RF_OPEN_FLAG = false;
	return 0; 
}

long rfcd_ioctl( struct file *filp, unsigned int cmd, unsigned long arg) {
	int err = 0;
	int ret = 0;
	//int ioarg =0;
    //char cardno[12];
    WiegandIn data;
	if(_IOC_TYPE(cmd) != WG_IOC_MAGIC)
		return -EINVAL;

	if(_IOC_NR(cmd) > WG_IOC_MAXNR)
		return -EINVAL;

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,(void *)arg, _IOC_SIZE(cmd));

	if(err)
		return -EFAULT;

	switch(cmd) {
        case WG_IOCGETFUN:
        	//printk("<---------CMD WG_IOCGETFUN DONE-------> %d\n\n",fun_flag);
        	ret = __put_user(fun_flag, (int *)arg);
        	break;
        case WG_IOC_FUN_IN:
        	if(fun_flag == WG_FUN) {
        		fun_flag = IO_FUN;
        		disable_irqs();
        	}
        	//printk("<---------CMD WG_IOC_FUN_IN DONE-------> %d\n\n",fun_flag);
        	break;
        case WG_IOC_FUN_WG:
        	if(fun_flag == IO_FUN) {
        		enable_irqs();
        		fun_flag = WG_FUN;
        	}	
        	//printk("<---------CMD WG_IOC_FUN_WG DONE-------> %d\n\n",fun_flag);
        	break;		
        case WG_IOCGETD0:
        	if(fun_flag == WG_FUN) {
        		printk("<---------io work as WG------->\n\n");
        		return -EINVAL;
        	}
        	//ioarg = 1101;
        	ret = gpio_get_value(rf_card->gpio_d0);
        	ret = __put_user(ret, (int *)arg);
        	break;
        case WG_IOCGETD1:
        	//ioarg = 1101;
        	if(fun_flag == WG_FUN) {
        		printk("<---------io work as WG------->\n\n");
        		return -EINVAL;
        	}
        	ret = gpio_get_value(rf_card->gpio_d1);
        	ret = __put_user(ret, (int *)arg);
        	break;
        case WG_IOCGETWG:
        	if(fun_flag == IO_FUN) {
        		printk("<---------io work as IO------->\n\n");
        		return -EINVAL;
        	}
        
        	if(TIMEER_FLAG)   //中断处理中，禁止读数据
        		return -1 ;

        	disable_irq(rf_card->d0_irq);
        	disable_irq(rf_card->d1_irq);//防止wieg_data在转换期间发生变化
        	data.cardno = rf_card->hid << 16 | rf_card->pid;
        	data.flag = g_flag;
        	memset(rf_card->wiegand, 0x00, 34);
        	rf_card->hid = 0;
        	rf_card->pid = 0;
        	rf_card->count = 0;
        	g_flag=MAX_NUM;
        	enable_irq(rf_card->d0_irq);
        	enable_irq(rf_card->d1_irq);
        	if ( copy_to_user((WiegandIn __user *)arg, &data, sizeof(WiegandIn))){
                 return -EFAULT;
        	}
        	ret = sizeof(WiegandIn);
        	break;	
        /*		
        	case MEMDEV_IOCSETDATA:
        		ret = __get_user(ioarg, (int *)arg);
        		printk("<--------In kernel MEMDEV_IOCSETDATA ioarg =%d --------->\n\n",ioarg);
        		break;
        */
        default:
            return -EINVAL;
    }
	
	return ret;
}

static struct file_operations rfcd_fops =  {
	.owner          = THIS_MODULE,
	.read           = rfcd_read,
	.write          = rfcd_write,
	.open           = rfcd_open,
	.release        = rfcd_release,
	.unlocked_ioctl = rfcd_ioctl,
};

//extern bool firefly_hwversion_in_range(const struct device_node *device);

static int wiegand_rec_probe(struct platform_device *pdev) {
	int err, result;
    int ret; 
    dev_t devno;
    struct device_node *np = pdev->dev.of_node;

//	if (!firefly_hwversion_in_range(np))
//		return -EINVAL;

	devno = MKDEV(WIEGAND_MAJOR, 1);
	
	printk("%s: start \n", __func__);
	if(1)
		result = register_chrdev_region(devno, 1, DEVICE_NAME);   
	else
		result = alloc_chrdev_region(&devno, 0, 1, DEVICE_NAME);
	
	if(result < 0) {
		printk("lengzhg %s:register_chrdev_region error\n",__func__);
		return result;
	}

	rf_card = kmalloc(sizeof(struct wiegand_dev), GFP_KERNEL);
	if (!rf_card) {
		result = -ENOMEM;
		goto fail_malloc;
	}

	memset(rf_card, 0, sizeof(struct wiegand_dev));

	rf_card->count = 0;

	cdev_init(&(rf_card->cdev), &rfcd_fops);
     
	rf_card->cdev.owner = THIS_MODULE;

	err = cdev_add(&rf_card->cdev, devno, 1);

	if(err) {
		printk("lengzhg adding err\r\n");
		unregister_chrdev_region(devno, 1);
		kfree(rf_card);
		free_irqs();
		return err;
	}

	cdev_class = class_create(THIS_MODULE, DEVICE_NAME);//
	if(IS_ERR(cdev_class)) { 
		printk("ERR:cannot create a cdev_class\n");
		unregister_chrdev_region(devno, 1);
        return -1;
	}
	device_create(cdev_class, NULL, devno, 0, DEVICE_NAME);
/* 
	ret = sysfs_create_group(&pdev->dev.kobj, &wiegandIn_attribute_group); 
*/	 
	init_completion(&(rf_card->receive_completion));
	sema_init(&rf_card->sem, 0);

	fun_flag = WG_FUN;

	rf_card->gpio_d0 = of_get_named_gpio(np, "wiegandin_d0", 0);
    if (!gpio_is_valid(rf_card->gpio_d0)) {
        dev_err(&pdev->dev, "invalid wiegandin_d0 gpio%d\n", rf_card->gpio_d0);
        return -ENODEV; 
    } else {
        ret = gpio_request(rf_card->gpio_d0, "wiegandin_d0");
	    if(ret) {
	    	printk("Failed to get gpio data0.\n");
            return -1;
	    }
	    gpio_direction_input(rf_card->gpio_d0);
	} 

	rf_card->gpio_d1 = of_get_named_gpio(np, "wiegandin_d1", 0);
    if (!gpio_is_valid(rf_card->gpio_d1)) {
        dev_err(&pdev->dev, "invalid wiegandin_d1 gpio%d\n", rf_card->gpio_d1);
		return -ENODEV;
    } else {
        ret = gpio_request(rf_card->gpio_d1, "wiegandin_d1");
        if(ret) {
            printk("Failed to get gpio data1.\n" );
            return -1;
        }
	    gpio_direction_input(rf_card->gpio_d1);
    }

	setup_timer(&rf_card->wiegand_timer, wiegand_do_timer, 0);
	rf_card->wiegand_timer.expires = jiffies + TIMER_DELAY;

    rf_card->d0_irq = gpio_to_irq(rf_card->gpio_d0);
    rf_card->d1_irq = gpio_to_irq(rf_card->gpio_d1);

    printk("%s:d0_irq=%d,d1_irq=%d\n",__func__, rf_card->d0_irq , rf_card->d1_irq );

    ret = request_irq(rf_card->d0_irq, wiegand_irq0, IRQF_SHARED | IRQF_TRIGGER_RISING,"wiegand_irq0", rf_card);

    if(ret) {
        printk("%s:request rf_card->d0_irq):%d,ret:%d failed!\n",__func__,rf_card->d0_irq,ret);
        return -1;
    }
    ret = request_irq(rf_card->d1_irq, wiegand_irq1, IRQF_SHARED | IRQF_TRIGGER_RISING, "wiegand_irq1", rf_card);

    if(ret) {
        printk("%s:request rf_card->d1_irq:%d,ret:%d failed!\n",__func__,rf_card->d1_irq,ret);
        return -1;
    }

	if(fun_flag == IO_FUN)	
        disable_irqs();

	printk (KERN_INFO "%s initialized\n", DEVICE_NAME);
	return 0;

fail_malloc: 
	unregister_chrdev_region(devno, 1);
	return result;
};

static int wiegand_rec_remove(struct platform_device *dev) {
	dev_t devno = MKDEV(WIEGAND_MAJOR, 1);
	gpio_free(rf_card->gpio_d0);
	gpio_free(rf_card->gpio_d1);
	free_irqs();
	del_timer_sync(&rf_card->wiegand_timer);
	cdev_del(&rf_card->cdev);
	device_destroy(cdev_class, devno);
	class_destroy(cdev_class);	
	kfree(rf_card);
	unregister_chrdev_region(MKDEV(WIEGAND_MAJOR, 1), 1);
	printk (KERN_INFO"%s removed\n", DEVICE_NAME);
    return 0;
};

static const struct of_device_id of_wiegand_rec_match[] = {
    { .compatible = "firefly,wiegand-receive" },
};

static struct platform_driver wiegand_rec_driver = {
    .probe      = wiegand_rec_probe,
    .remove     = wiegand_rec_remove,
    .driver     = {
        .owner  = THIS_MODULE,
        .of_match_table = of_wiegand_rec_match,
        .name   = "wiegand-receive",
    },
};

static int __init wiegand_init(void) {
    return platform_driver_register(&wiegand_rec_driver);
}

static void __exit wiegand_exit(void) {
    platform_driver_unregister(&wiegand_rec_driver);
}

module_init(wiegand_init);
module_exit(wiegand_exit);

MODULE_AUTHOR("firefly-lizs");
MODULE_LICENSE("GPL");
