/*
*   Face-future Ltd.
*   By  ach Tech
*   DEMO Version :1.1 Data:2017-05-09
*
*   1. compiler warnings all changes
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/spi/spi.h>
#include<linux/timer.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/io.h>
#include "spi-wk2xxx.h"
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/serial.h>
#include <linux/io.h>

MODULE_LICENSE("Dual BSD/GPL");

#define SPI_BUFSIZ      max(32,SMP_CACHE_BYTES)
//#define _DEBUG_WK2XXX
//#define _DEBUG_WK2XXX1
//#define _DEBUG_WK2XXX2
//#define _DEBUG_WK2XXX4
//#define _DEBUG_WK2XXX5
#define CONFIG_DEVFS_FS

#define WK2XXX_PAGE1        1
#define WK2XXX_PAGE0        0

#define WK2XXX_STATUS_PE    1
#define WK2XXX_STATUS_FE    2
#define WK2XXX_STATUS_BRK   4
#define WK2XXX_STATUS_OE    8


//#define WK2XX_DEBUG(x)      printk(KERN_ALERT "ttysWK%d: pass_counter = %d\n",s->port.iobase, pass_counter);


#define WK2XX_DEBUG(format, ...) printk (#format, ## __VA_ARGS__)

static DEFINE_MUTEX(wk2xxxs_lock);                /* race on probe */
static DEFINE_MUTEX(wk2xxxs_reg_lock);
static DEFINE_MUTEX(wk2xxs_work_lock);                /* work on probe */
static DEFINE_MUTEX(wk2xxs_global_register_lock);                /* work on wk2xxx_startup read or write global register */

//extern void tty_flip_buffer_push(struct tty_port *port);

#define WK2XXX_SPI_MAX_SPEED_HZ	10000000

struct wk2xxx_port
{
	//struct timer_list mytimer;

	struct uart_port port;//[NR_PORTS];
	struct spi_device *spi_wk;
	spinlock_t conf_lock;   /* shared data */
	struct workqueue_struct *workqueue;
	struct work_struct work;
	int suspending;
	void (*wk2xxx_hw_suspend) (int suspend);
	int tx_done;

	int force_end_work;
	int irq;
	int minor;      /* minor number */
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

	uint8_t new_lcr;
	uint8_t new_scr;

	/*set baud 0f register*/
	uint8_t new_baud1;
	uint8_t new_baud0;
	uint8_t new_pres;
	unsigned	pwr;
	unsigned	rst;
	unsigned	cs;
	unsigned	irq_gpio;
};

static struct wk2xxx_port wk2xxxs[NR_PORTS]; /* the chips */

struct wk2xxx_port *wk2124_dev = NULL;

void wk2124_cs_gpio(int on)
{
	gpio_set_value(wk2124_dev->cs, !!on);
}

static int wk2xxx_read_reg(struct spi_device *spi,uint8_t port,uint8_t reg,uint8_t *dat)
{
	struct spi_message msg;
	uint8_t buf_wdat[2];
	uint8_t buf_rdat[2];
	int status;
	struct spi_transfer index_xfer = {
		.len            = 2,
		.cs_change      = 1,
	};

	mutex_lock(&wk2xxxs_reg_lock);
	status =0;
	wk2124_cs_gpio(0);
	spi_message_init(&msg);
	buf_wdat[0] = 0x40|(((port-1)<<4)|reg);
	buf_wdat[1] = 0x00;
	buf_rdat[0] = 0x00;
	buf_rdat[1] = 0x00;
	index_xfer.tx_buf = buf_wdat;
	index_xfer.rx_buf =(void *) buf_rdat;
	spi_message_add_tail(&index_xfer, &msg);
	status = spi_sync(spi, &msg);
	udelay(3);
	wk2124_cs_gpio(1);
	mutex_unlock(&wk2xxxs_reg_lock);

	if (status)
	{
		return status;
	}
	*dat = buf_rdat[1];

	return 0;
}

static int wk2xxx_write_reg(struct spi_device *spi,uint8_t port,uint8_t reg,uint8_t dat)
{
        struct spi_message msg;
        uint8_t buf_reg[2];
        int status;
        struct spi_transfer index_xfer = {
                .len            = 2,
                .cs_change      = 1,
        };

        mutex_lock(&wk2xxxs_reg_lock);
		wk2124_cs_gpio(0);
        spi_message_init(&msg);
        /* register index */
        buf_reg[0] = ((port-1)<<4)|reg;
        buf_reg[1] = dat;
        index_xfer.tx_buf = buf_reg;
        spi_message_add_tail(&index_xfer, &msg);
        status = spi_sync(spi, &msg);
        udelay(3);
		wk2124_cs_gpio(1);
        mutex_unlock(&wk2xxxs_reg_lock);

        if(status)
        {
         return status;
        }

        return status;
}

#define MAX_RFCOUNT_SIZE 256
static int wk2xxx_read_fifo(struct spi_device *spi,uint8_t port,uint8_t fifolen,uint8_t *dat)
{
        struct spi_message msg;
        int status,i;

        uint8_t recive_fifo_data[MAX_RFCOUNT_SIZE+1]={0};
        uint8_t transmit_fifo_data[MAX_RFCOUNT_SIZE+1]={0};
        struct spi_transfer index_xfer = {
                .len            = fifolen+1,
                .cs_change      = 1,
        };

        mutex_lock(&wk2xxxs_reg_lock);
		wk2124_cs_gpio(0);
        spi_message_init(&msg);
        /* register index */
        transmit_fifo_data[0] = ((port-1)<<4)|0xc0;
        index_xfer.tx_buf = transmit_fifo_data;
        index_xfer.rx_buf =(void *) recive_fifo_data;
        spi_message_add_tail(&index_xfer, &msg);

        status = spi_sync(spi, &msg);
        udelay(3);
        for(i=0;i<fifolen;i++)
            *(dat+i)=recive_fifo_data[i+1];
		wk2124_cs_gpio(1);
        mutex_unlock(&wk2xxxs_reg_lock);

        return status;
}

static int wk2xxx_write_fifo(struct spi_device *spi,uint8_t port,uint8_t fifolen,uint8_t *dat)
{
        struct spi_message msg;
        int status,i;
        uint8_t recive_fifo_data[MAX_RFCOUNT_SIZE+1]={0};
        uint8_t transmit_fifo_data[MAX_RFCOUNT_SIZE+1]={0};

        struct spi_transfer index_xfer = {
                .len            = fifolen+1,
                .cs_change      = 1,
        };

        mutex_lock(&wk2xxxs_reg_lock);
		wk2124_cs_gpio(0);
        spi_message_init(&msg);
        /* register index */
        transmit_fifo_data[0] = ((port-1)<<4)|0x80;

         for(i=0;i<fifolen;i++)
            {
            transmit_fifo_data[i+1]=*(dat+i);
            }
        index_xfer.tx_buf = transmit_fifo_data;
        index_xfer.rx_buf =(void *) recive_fifo_data;
        spi_message_add_tail(&index_xfer, &msg);

        status = spi_sync(spi, &msg);
        udelay(3);
		wk2124_cs_gpio(1);
        mutex_unlock(&wk2xxxs_reg_lock);

        return status;
}

static void wk2xxxirq_app(struct uart_port *port);
static void conf_wk2xxx_subport(struct uart_port *port);
static void wk2xxx_work(struct work_struct *w);
static void wk2xxx_stop_tx(struct uart_port *port);
static u_int wk2xxx_tx_empty(struct uart_port *port);// or query the tx fifo is not empty?

static int wk2xxx_dowork(struct wk2xxx_port *s)
{
#ifdef _DEBUG_WK2XXX
    printk("--wk2xxx_dowork---in---\n");
#endif

    if (!s->force_end_work && !work_pending(&s->work) && !freezing(current) && !s->suspending)
    {
        queue_work(s->workqueue, &s->work);//
        #ifdef _DEBUG_WK2XXX
        printk("--queue_work---ok---\n");
         printk("--wk2xxx_dowork---exit---\n");
      #endif

        return 1;
    }
    else
    {
      #ifdef _DEBUG_WK2XXX
        printk("--queue_work---error---\n");
         printk("--wk2xxx_dowork---exit---\n");
      #endif

        return 0;
    }
}

static void wk2xxx_work(struct work_struct *w)
{
#ifdef _DEBUG_WK2XXX
    //printk("--wk2xxx_work---in---\n");
#endif

	struct wk2xxx_port *s = container_of(w, struct wk2xxx_port, work);
	uint8_t rx;
	int work_start_tx_flag;
	int work_stop_rx_flag;
	int work_irq_flag;
	int work_conf_flag;

	do {
	    mutex_lock(&wk2xxs_work_lock);
	
    	work_start_tx_flag = s->start_tx_flag;
		if(work_start_tx_flag)
			s->start_tx_flag = 0;

	    work_stop_rx_flag = s->stop_rx_flag;
		if(work_stop_rx_flag)
			s->stop_rx_flag = 0;

		work_conf_flag = s->conf_flag;

	    work_irq_flag = s->irq_flag;
        if(work_irq_flag)
			s->irq_flag = 0;

	    mutex_unlock(&wk2xxs_work_lock);

	//printk("%s : ----- work_start_tx_flag = %d \r\n",__func__,work_start_tx_flag);

		if(work_start_tx_flag)  //使能 发送FIFO触点中断
		{
			wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,&rx);
			rx |= WK2XXX_TFTRIG_IEN;
			wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,rx);
		}

        if(work_stop_rx_flag)
		{
            //禁用接收超时 和 触点终端
			wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,&rx);
			rx &=~WK2XXX_RFTRIG_IEN;
			rx &=~WK2XXX_RXOUT_IEN;
			wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,rx);

            // 清接收超时和触点终端标志
			wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIFR,&rx);
			rx &= ~WK2XXX_RFTRIG_INT;
			rx &= ~WK2XXX_RXOVT_INT;
			wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SIFR,rx);
		}

		if(work_irq_flag)   //如果有硬件中断
		{
			wk2xxxirq_app(&s->port);
			s->irq_fail = 1;
		}
	}while (!s->force_end_work && \
                    !freezing(current) && \
	                (work_irq_flag || work_stop_rx_flag ) \
                     );
                     
	if(s->start_tx_fail)
	{
        wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,&rx);
        rx |= WK2XXX_TFTRIG_IEN;
		wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,rx);
		s->start_tx_fail =0;
	}

	if(s->stop_rx_fail)
	{
        wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,&rx);
        rx &=~WK2XXX_RFTRIG_IEN;
        rx &=~WK2XXX_RXOUT_IEN;
        wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,rx);

        wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIFR,&rx);
        rx &= ~WK2XXX_RFTRIG_INT;
        rx &= ~WK2XXX_RXOVT_INT;
        wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SIFR,rx);
        s->stop_rx_fail =0;
    }

	if(s->irq_fail)
	{
        s->irq_fail = 0;
        enable_irq(s->port.irq);
	}

#ifdef _DEBUG_WK2XXX
    printk("--wk2xxx_work---exit---\n");
#endif
}


static void wk2xxx_rx_chars(struct uart_port *port)//vk32xx_port *port)
{
    struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
    uint8_t fsr,lsr,dat[1],rx_dat[256]={0};
    unsigned int ch,flg,sifr, ignored=0,status = 0,rx_count=0;
    int rfcnt=0,rx_num=0;

    wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
    wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_FSR,dat);
    fsr = dat[0];

    wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_LSR,dat);
    lsr = dat[0];

    wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIFR,dat);
    sifr=dat[0];

	if(!(sifr&0x80))//no error
	{
		flg = TTY_NORMAL;
        if (fsr& WK2XXX_RDAT)   //接收FIFO未空
        {
            //子串口接收的FIFO个数
            wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_RFCNT,dat);
            rfcnt = dat[0];
            if(rfcnt == 0)
            {
                rfcnt=255;
            }

            wk2xxx_read_fifo(s->spi_wk,s->port.iobase, rfcnt,rx_dat);
            s->port.icount.rx += rfcnt;     //加计数

            //没64字节提交一次buff
            for(rx_num=0; rx_num < rfcnt; rx_num++)
            {
                if (uart_handle_sysrq_char(&s->port,rx_dat[rx_num]))//.state, ch))
                    break;//

                uart_insert_char(&s->port, status, WK2XXX_STATUS_OE, rx_dat[rx_num], flg);
                rx_count++;

                if ((rx_count >= 64 ) && (s->port.state->port.tty != NULL))
                {
                    tty_flip_buffer_push(&s->port. state->port);
                    rx_count = 0;
                }
           }

           if((rx_count > 0)&&(s->port.state->port.tty != NULL))
            {
                tty_flip_buffer_push(&s->port.state->port);
                rx_count = 0;
            }
        }
    }
    else//error
    {
        while (fsr& WK2XXX_RDAT)/**/ //循环校验错误标志位
        {
            wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_FDAT,dat);
            ch = (int)dat[0];

            s->port.icount.rx++;

            flg = TTY_NORMAL;
            if (lsr&(WK2XXX_OE |WK2XXX_FE|WK2XXX_PE|WK2XXX_BI))
            {
                printk(KERN_ALERT "wk2xxx_rx_chars()----port:%lx error,lsr:%x!!!!!!!!!!!!!!!!!\n",s->port.iobase,lsr);
                if (lsr & WK2XXX_PE)
                {
                    s->port.icount.parity++;
                    status |= WK2XXX_STATUS_PE;
                    flg = TTY_PARITY;
                }
                if (lsr & WK2XXX_FE)
                {
                    s->port.icount.frame++;
                    status |= WK2XXX_STATUS_FE;
                    flg = TTY_FRAME;
                }
                if (lsr & WK2XXX_OE)
                {
                    s->port.icount.overrun++;
                    status |= WK2XXX_STATUS_OE;
                    flg = TTY_OVERRUN;
                }
                if(lsr&fsr & WK2XXX_BI)
                {
                    s->port.icount.brk++;
                    status |= WK2XXX_STATUS_BRK;
                    flg = TTY_BREAK;
                }

                if (++ignored > 100)
                {
                    goto out;
                }
                goto ignore_char;
            }

error_return:
            if (uart_handle_sysrq_char(&s->port,ch))//.state, ch))
                goto ignore_char;

            uart_insert_char(&s->port, status, WK2XXX_STATUS_OE, ch, flg);
            rx_count++;

            if ((rx_count >= 64 ) && (s->port.state->port.tty != NULL))
            {
                tty_flip_buffer_push(&s->port.state->port);
                rx_count = 0;
            }

#ifdef _DEBUG_WK2XXX1
            printk(KERN_ALERT  " s->port.icount.rx = 0x%X char = 0x%X flg = 0x%X port = %d rx_count = %d\n",s->port.icount.rx,ch,flg,s->port.iobase,rx_count);
#endif

ignore_char:
            wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_FSR,dat);
            fsr = dat[0];
            wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_LSR,dat);
            lsr = dat[0];
        }

    out:
    if((rx_count > 0)&&(s->port.state->port.tty != NULL))
    {
#ifdef _DEBUG_WK2XXX1
       printk(KERN_ALERT  "push buffer tty flip port = :%d count = :%d\n",s->port.iobase,rx_count);
#endif
        tty_flip_buffer_push(&s->port.state->port);
        rx_count = 0;
    }

    }//if()else

 #if 0
   printk(KERN_ALERT  " rx_num = :%d\n",s->port.icount.rx);
 #endif

#ifdef _DEBUG_WK2XXX
                 printk(KERN_ALERT "wk2xxx_rx_chars()---------out---\n");
#endif

      return;
#ifdef SUPPORT_SYSRQ
        s->port.state->sysrq = 0;
#endif
        goto error_return;

#ifdef _DEBUG_WK2XXX
    printk("--wk2xxx_rx_chars---exit---\n");
#endif

}

static void wk2xxx_tx_chars(struct uart_port *port)//
{
#ifdef _DEBUG_WK2XXX
    printk("--wk2xxx_tx_chars---in---\n");
#endif
    struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
    uint8_t fsr,tfcnt,dat[1],txbuf[255]={0};
    int count,tx_count,i;

    wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SPAGE,WK2XXX_PAGE0);//set register in page0
        if (s->port.x_char)
        {
        #ifdef _DEBUG_WK2XXX
        printk(KERN_ALERT "wk2xxx_tx_chars   s->port.x_char:%x,port = %d\n",s->port.x_char,s->port.iobase);
       #endif
            wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_FDAT,s->port.x_char);
            s->port.icount.tx++;
            s->port.x_char = 0;
            goto out;
        }

        if(uart_circ_empty(&s->port.state->xmit) || uart_tx_stopped(&s->port))
        {
            goto out;
        }
    /*
     * Tried using FIFO (not checking TNF) for fifo fill:
     * still had the '1 bytes repeated' problem.
    */
    wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_FSR,dat);
    fsr = dat[0];

    wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_TFCNT,dat);
    tfcnt= dat[0];
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "wk2xxx_tx_chars   fsr:0x%x,rfcnt:0x%x,port = %x\n",fsr,tfcnt,s->port.iobase);
#endif

	if(tfcnt==0)
	{
		if(fsr & WK2XXX_TFULL)
		{
			tfcnt=255;
			tx_count=0;
		}
		else
		{
			tfcnt=0;
			tx_count=255;
		}
		}
		else
		{
			tx_count=255-tfcnt;
#ifdef _DEBUG_WK2XXX
		printk(KERN_ALERT "wk2xxx_tx_chars2   tx_count:%x,port = %x\n",tx_count,s->port.iobase);
#endif
	}

#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "fsr:%x\n",fsr);
#endif

count = tx_count;
i=0;
do
{
  if(uart_circ_empty(&s->port.state->xmit))
     break;
   txbuf[i]=s->port.state->xmit.buf[s->port.state->xmit.tail];
   s->port.state->xmit.tail = (s->port.state->xmit.tail + 1) & (UART_XMIT_SIZE - 1);
   s->port.icount.tx++;
   i++;
#ifdef _DEBUG_WK2XXX

        printk(KERN_ALERT "tx_chars:0x%x--\n",txbuf[i-1]);
#endif

}while(--count>0);

#ifdef _DEBUG_WK2XXX5
        printk(KERN_ALERT "tx_chars I:0x%x--\n",i);
#endif

	wk2xxx_write_fifo(s->spi_wk,s->port.iobase,i,txbuf);
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "s->port.icount.tx,xmit.head1:%d,xmit.tail:%d,UART_XMIT_SIZE::%d,char:%d,fsr:0x%X,port = %d\n",s->port.icount.tx,s->port.state->xmit.head,s->port.state->xmit.tail,UART_XMIT_SIZE,s->port.state->xmit.buf[s->port.state->xmit.tail],fsr,s->port.iobase);
#endif

out:
	wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_FSR,dat);
	fsr = dat[0];
	if(((fsr&WK2XXX_TDAT)==0)&&((fsr&WK2XXX_TBUSY)==0))
	{
		if (uart_circ_chars_pending(&s->port.state->xmit) < WAKEUP_CHARS)
			uart_write_wakeup(&s->port);

           if (uart_circ_empty(&s->port.state->xmit))
            {
               wk2xxx_stop_tx(&s->port);
            }
        }
#ifdef _DEBUG_WK2XXX
    printk("--wk2xxx_tx_chars---exit---\n");
#endif


}

static irqreturn_t wk2xxx_irq(int irq, void *dev_id)//
{
#ifdef _DEBUG_WK2XXX
    printk("--wk2xxx_irq---in---\n");
#endif
    struct wk2xxx_port *s = dev_id;
    disable_irq_nosync(s->port.irq);

        s->irq_flag = 1;
    if(wk2xxx_dowork(s))
    {

        //s->irq_flag = 1;

    }
    else
    {
        s->irq_flag = 0;
        s->irq_fail = 1;
    }
#ifdef _DEBUG_WK2XXX
    printk("--wk2xxx_irq---exit---\n");
#endif

    return IRQ_HANDLED;
}

static void wk2xxxirq_app(struct uart_port *port)//
{
    struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "wk2xxxirq_app()------port:%d--------------\n",s->port.iobase);
#endif
    unsigned int  pass_counter = 0;
    uint8_t sifr,gifr,sier,dat[1];

    wk2xxx_read_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GIFR ,dat);
    gifr = dat[0];

    switch(s->port.iobase)
    {
        case 1 :
            if(!(gifr & WK2XXX_UT1INT))
            {
                return;
            }
            break;
        case 2 :
            if(!(gifr & WK2XXX_UT2INT))
            {
                return;
            }
            break;
        case 3 :
            if(!(gifr & WK2XXX_UT3INT))
            {
                return;
            }
            break;
        case 4 :
            if(!(gifr & WK2XXX_UT4INT))
            {
                return;
            }
            break;
        default:
            break;
    }

    wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIFR,dat);
    sifr = dat[0];
    wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,dat);
    sier = dat[0];

    do {
        //有接收FIFO触点中断和中断超时标志
        if ((sifr&WK2XXX_RFTRIG_INT)||(sifr&WK2XXX_RXOVT_INT))
        {
            wk2xxx_rx_chars(&s->port);
        }

        //有发送FIFO触点中断和中断超时标志
        if ((sifr & WK2XXX_TFTRIG_INT)&&(sier & WK2XXX_TFTRIG_IEN ))
        {
            wk2xxx_tx_chars(&s->port);
           return;
        }

        if (pass_counter++ > WK2XXX_ISR_PASS_LIMIT)
            break;

        wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIFR,dat);
        sifr = dat[0];
        wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,dat);
        sier = dat[0];
    } while ((sifr&WK2XXX_RXOVT_INT) ||  \
                    (sifr & WK2XXX_RFTRIG_INT) || \
                    ((sifr & WK2XXX_TFTRIG_INT) && (sier & WK2XXX_TFTRIG_IEN)));
}


/*
 *   Return TIOCSER_TEMT when transmitter is not busy.
 */

static u_int wk2xxx_tx_empty(struct uart_port *port)// or query the tx fifo is not empty?
{
    uint8_t rx;
//       mutex_lock(&wk2xxxs_lock);
    struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
#ifdef _DEBUG_WK2XXX
        printk(KERN_ALERT "wk2xxx_tx_empty()---------in---\n");
#endif

mutex_lock(&wk2xxxs_lock);

if(!(s->tx_empty_flag || s->tx_empty_fail))
{
    wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_FSR,&rx);
    while((rx & WK2XXX_TDAT)|(rx&WK2XXX_TBUSY))
	{
		wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_FSR,&rx);
	}
    s->tx_empty = ((rx & WK2XXX_TDAT)|(rx&WK2XXX_TBUSY))<=0;

    if(s->tx_empty)
    {
    s->tx_empty_flag =0;
    s->tx_empty_fail=0;
    }
    else
    {
    s->tx_empty_fail=0;
    s->tx_empty_flag =0;
    }
}
mutex_unlock(&wk2xxxs_lock);

  #ifdef _DEBUG_WK2XXX5
              printk(KERN_ALERT "s->tx_empty_fail----FSR:%d--s->tx_empty:%d--\n",rx,s->tx_empty);
   #endif

#ifdef _DEBUG_WK2XXX
        printk(KERN_ALERT "wk2xxx_tx_empty----------exit---\n");
#endif
    return s->tx_empty;

}

static void wk2xxx_set_mctrl(struct uart_port *port, u_int mctrl)//nothing
{
#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "-wk2xxx_set_mctrl---------exit---\n");
#endif

}
static u_int wk2xxx_get_mctrl(struct uart_port *port)// since no modem control line
{
#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "-wk2xxx_get_mctrl---------exit---\n");
#endif

        return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}


/*
 *  interrupts disabled on entry
 */

static void wk2xxx_stop_tx(struct uart_port *port)//
{

#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "-wk2xxx_stop_tx------in---\n");
#endif
    uint8_t dat[1],sier,sifr;
    struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);

    mutex_lock(&wk2xxxs_lock);

    if(!(s->stop_tx_flag||s->stop_tx_fail))
      {
        wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,dat);
        sier=dat[0];
        s->stop_tx_fail=(sier&WK2XXX_TFTRIG_IEN)>0;
        if(s->stop_tx_fail)
          {
                  wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,dat);
                  sier=dat[0];
                  sier&=~WK2XXX_TFTRIG_IEN;
                  wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,sier);
                  wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIFR,dat);
                  sifr=dat[0];
                  sifr&= ~WK2XXX_TFTRIG_INT;
                  wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SIFR,sifr);
                  s->stop_tx_fail =0;
                  s->stop_tx_flag=0;
          }
		else
		{
			  s->stop_tx_fail =0;
			  s->stop_tx_flag=0;


		}
    }
mutex_unlock(&wk2xxxs_lock);
#ifdef _DEBUG_WK2XXX4
    printk(KERN_ALERT "-wk2xxx_stop_tx------exit---\n");
#endif
}

/*
 *  * interrupts may not be disabled on entry
*/
static void wk2xxx_start_tx(struct uart_port *port)
{
#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "-wk2xxx_start_tx------in---\n");
#endif

	struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
	if(!(s->start_tx_flag||s->start_tx_fail))
	{
		s->start_tx_flag = 1;
		if(wk2xxx_dowork(s))
		{
			;
		}
		else
		{
			s->start_tx_fail = 1;
			s->start_tx_flag = 0;
		}
	}

#ifdef _DEBUG_WK2XXX
        printk(KERN_ALERT "-wk2xxx_start_tx------exit---\n");
#endif
}

/*
 *  * Interrupts enabled
*/

static void wk2xxx_stop_rx(struct uart_port *port)
{
#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "-wk2xxx_stop_rx------in---\n");
#endif

        struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
       if(!(s->stop_rx_flag ||s->stop_rx_fail ))
        {
            s->stop_rx_flag = 1;
            if(wk2xxx_dowork(s))
            {
                ;
            }
            else
            {
                s->stop_rx_flag = 0;
                s->stop_rx_fail = 1;
            }
        }


#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "-wk2xxx_stop_rx------exit---\n");
#endif


}


/*
 *  * No modem control lines
 *   */
static void wk2xxx_enable_ms(struct uart_port *port)    //nothing
{
#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "-wk2xxx_enable_ms------exit---\n");
#endif

}
/*
 *  * Interrupts always disabled.
*/
static void wk2xxx_break_ctl(struct uart_port *port, int break_state)
{
#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "-wk2xxx_break_ctl------exit---\n");
#endif

    //break operation, but there  seems no such operation in vk32
}


/*
 *  * enable uart port clock.
 * @jincheng
*/
void wk2xxx_enable_uart_port_clock(struct wk2xxx_port *s)
{
    uint8_t gena;

    mutex_lock(&wk2xxs_global_register_lock);
    wk2xxx_read_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GENA, &gena);
    switch (s->port.iobase)
    {
    case 1:
            gena|=WK2XXX_UT1EN;
            wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GENA,gena);
            break;
    case 2:
            gena|=WK2XXX_UT2EN;
            wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GENA,gena);
            break;
    case 3:
            gena|=WK2XXX_UT3EN;
            wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GENA,gena);
            break;
    case 4:
            gena|=WK2XXX_UT4EN;
            wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GENA,gena);
            break;
    default:
        printk(KERN_ALERT ":con_wk2xxx_subport bad iobase %d\n", (uint8_t)s->port.iobase);
            break;
    }
    mutex_unlock(&wk2xxs_global_register_lock);
}

/*
 *  * disable uart port clock.
 * @jincheng
*/
void wk2xxx_disable_uart_port_clock(struct wk2xxx_port *s)
{
    uint8_t gena;

    mutex_lock(&wk2xxs_global_register_lock);
    wk2xxx_read_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GENA, &gena);
    switch (s->port.iobase)
    {
        case 1:
            gena&=~WK2XXX_UT1EN;
            wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GENA,gena);
           break;
        case 2:
            gena&=~WK2XXX_UT2EN;
            wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GENA,gena);
            break;
        case 3:
            gena&=~WK2XXX_UT3EN;
            wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GENA,gena);
            break;
        case 4:
            gena&=~WK2XXX_UT4EN;
            wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GENA,gena);
            break;
        default:
            dev_warn(&s->spi_wk->dev, ":con_wk2xxx_subport bad iobase %d\n", (uint8_t)s->port.iobase);
        break;
    }
    mutex_unlock(&wk2xxs_global_register_lock);
}



void wk2xxx_reset_uart_port(struct wk2xxx_port *s)
{
    uint8_t grst;

    mutex_lock(&wk2xxs_global_register_lock);
    wk2xxx_read_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GRST, &grst);
    switch (s->port.iobase)
    {
        case 1:
                grst|=WK2XXX_UT1RST;
                wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GRST,grst);
                break;
        case 2:
                grst|=WK2XXX_UT2RST;
                wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GRST,grst);
                break;
        case 3:
                grst|=WK2XXX_UT3RST;
                wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GRST,grst);
                break;
        case 4:
                grst|=WK2XXX_UT4RST;
                wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GRST,grst);
                break;
        default:
                printk(KERN_ALERT ":con_wk2xxx_subport bad iobase %d\n", (uint8_t)s->port.iobase);
                break;
    }
    mutex_unlock(&wk2xxs_global_register_lock);
}

//enable the sub port interrupt
void wk2xxx_enabel_sub_port_int(struct wk2xxx_port *s)
{
    uint8_t gier;

    mutex_lock(&wk2xxs_global_register_lock);
    wk2xxx_read_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GIER,  &gier);

    switch (s->port.iobase){
        case 1:
                gier|=WK2XXX_UT1IE;
                wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GIER,gier);
                break;
        case 2:
                gier|=WK2XXX_UT2IE;
                wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GIER,gier);
                break;
        case 3:
                gier|=WK2XXX_UT3IE;
                wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GIER,gier);
                break;
        case 4:
                gier|=WK2XXX_UT4IE;
                wk2xxx_write_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GIER,gier);
                break;
        default:
                printk(KERN_ALERT ": bad iobase %d\n", (uint8_t)s->port.iobase);
                break;
    } 
    mutex_unlock(&wk2xxs_global_register_lock);
}



void wk2xxx_init_uart_port_FIFO(struct wk2xxx_port *s)
{
    uint8_t sier;

    wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER, &sier);

    sier &= ~WK2XXX_TFTRIG_IEN; //禁用FIFO发送触发终端
    sier |= WK2XXX_RFTRIG_IEN;   //接收FIFO触发中断
    sier |= WK2XXX_RXOUT_IEN;      //接收FIFO超时中断

    wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER,sier);
}


static int wk2xxx_startup(struct uart_port *port)//i
{
#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "-wk2xxx_startup------in---\n");
#endif
    uint8_t scr,dat[1];
    struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
    char b[12];

    if (s->suspending)
        return 0;

    s->force_end_work = 0;  //clear force_end_work

    //init workqueue
    sprintf(b, "wk2xxx-%d", (uint8_t)s->port.iobase);
    s->workqueue = create_workqueue(b);
    if (!s->workqueue)
    {
        dev_warn(&s->spi_wk->dev, "cannot create workqueue\n");
        return -EBUSY;
    }
    INIT_WORK(&s->work, wk2xxx_work);

    if (s->wk2xxx_hw_suspend)
        s->wk2xxx_hw_suspend(0);

    wk2xxx_enable_uart_port_clock(s);//使能串口时钟 
    wk2xxx_reset_uart_port(s);//复位串口
    wk2xxx_init_uart_port_FIFO(s);   //初始化FIFO中断

    //使能串口发送和接收位
    wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SCR,dat);
    scr = dat[0] | WK2XXX_TXEN|WK2XXX_RXEN;
    wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SCR,scr);
//initiate the fifos
    wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_FCR,0xff);//initiate the fifos
    wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_FCR,0xfc);
//set rx/tx interrupt
    wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SPAGE,1);
    wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_RFTL,0X40);
    wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_TFTL,0X40);
    wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SPAGE,0);
//enable the sub port interrupt
    wk2xxx_enabel_sub_port_int(s);

    if (s->wk2xxx_hw_suspend)
    {
        s->wk2xxx_hw_suspend(0);
        msleep(50);
    }
     
    uart_circ_clear(&s->port.state->xmit);
    wk2xxx_enable_ms(&s->port);

    if(request_irq(s->port.irq, wk2xxx_irq,IRQF_SHARED | IRQF_TRIGGER_LOW,"wk2xxxspi", s) < 0)
    {
            dev_warn(&s->spi_wk->dev, "cannot allocate irq %d\n", s->irq);
            s->port.irq = 0;
            destroy_workqueue(s->workqueue);
            s->workqueue = NULL;
            return -EBUSY;
    }       

#ifdef _DEBUG_WK2XXX
        printk(KERN_ALERT "-wk2xxx_startup------exit---\n", );
#endif
       return 0;
}

//* Power down all displays on reboot, poweroff or halt *

static void wk2xxx_shutdown(struct uart_port *port)//
{
#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "-wk2xxx_shutdown------in---\n");
#endif

    struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);

    if (s->suspending)
       return;

	s->force_end_work = 1;
    if (s->workqueue)
    {
        flush_workqueue(s->workqueue);
        destroy_workqueue(s->workqueue);
        s->workqueue = NULL;
    }

    if (s->port.irq)
    {
          free_irq(s->port.irq,s);
    }

    //disabel uart port clock
    wk2xxx_disable_uart_port_clock(s);


#ifdef _DEBUG_WK2XXX5
        wk2xxx_read_reg(s->spi_wk,WK2XXX_GPORT,WK2XXX_GENA,dat);
      gena=dat[0];
        printk(KERN_ALERT "-wk2xxx_shutdown-----port:%d--gena:%x-\n",(uint8_t)s->port.iobase,gena);
#endif
#ifdef _DEBUG_WK2XXX
        printk(KERN_ALERT "-wk2xxx_shutdown-----exit---\n");
#endif
        return;
}

static void conf_wk2xxx_subport(struct uart_port *port)//i
{
#ifdef _DEBUG_WK2XXX
            printk(KERN_ALERT "-conf_wk2xxx_subport------in---\n");
#endif

    struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
    uint8_t old_sier,lcr,scr,scr_ss,dat[1],baud0_ss,baud1_ss,pres_ss;

    lcr = s->new_lcr;
    scr_ss = s->new_scr;
    baud0_ss=s->new_baud0;
    baud1_ss=s->new_baud1;
    pres_ss=s->new_pres;
    wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER ,dat);
    old_sier = dat[0];
    wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER ,old_sier&(~(WK2XXX_TFTRIG_IEN | WK2XXX_RFTRIG_IEN | WK2XXX_RXOUT_IEN)));
    //local_irq_restore(flags);
    do{
        wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_FSR,dat);
        //ssr = dat[0];
       } while (dat[0] & WK2XXX_TBUSY);
        // then, disable everything
        wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_SCR,dat);
        scr = dat[0];

        wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SCR ,scr&(~(WK2XXX_RXEN|WK2XXX_TXEN)));
        // set the parity, stop bits and data size //
        wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_LCR ,lcr);
        // set the baud rate //
        wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SIER ,old_sier);
        // set the baud rate //
        wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SPAGE ,1);
        wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_BAUD0 ,baud0_ss);
        wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_BAUD1 ,baud1_ss);
        wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_PRES ,pres_ss);
#ifdef _DEBUG_WK2XXX2
        wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_BAUD0,dat);
        printk(KERN_ALERT ":WK2XXX_BAUD0=0x%X\n", dat[0]);
        wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_BAUD1,dat);
        printk(KERN_ALERT ":WK2XXX_BAUD1=0x%X\n", dat[0]);
        wk2xxx_read_reg(s->spi_wk,s->port.iobase,WK2XXX_PRES,dat);
        printk(KERN_ALERT ":WK2XXX_PRES=0x%X\n", dat[0]);
#endif
        wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SPAGE ,0);
        wk2xxx_write_reg(s->spi_wk,s->port.iobase,WK2XXX_SCR ,scr|(WK2XXX_RXEN|WK2XXX_TXEN)  );
#ifdef _DEBUG_WK2XXX
        printk(KERN_ALERT "-conf_wk2xxx_subport------exit---\n");
#endif

}


// change speed
static void wk2xxx_termios( struct uart_port *port, struct ktermios *termios,
            struct ktermios *old)
{
#ifdef _DEBUG_WK2XXX
       printk(KERN_ALERT "-wk2xxx_termios------in---\n");
#endif

    struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
    int baud = 0;
    uint8_t lcr,baud1,baud0,pres;
    unsigned short cflag;
    unsigned short lflag;

    cflag = termios->c_cflag;
    lflag = termios->c_lflag;
#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "cflag := 0x%X  lflag : = 0x%X\n",cflag,lflag);
#endif
    baud1=0;
    baud0=0;
    pres=0;
    baud = tty_termios_baud_rate(termios);//用户空间传递到内核空间

	switch (baud) {
	case 600:
		baud1=0x4;
		baud0=0x7f;
		pres=0;
		break;
	case 1200:
		baud1=0x2;
		baud0=0x3F;
		pres=0;
		break;
	case 2400:
		baud1=0x1;
		baud0=0x1f;
		pres=0;
		break;
	case 4800:
		baud1=0x00;
		baud0=0x8f;
		pres=0;
		break;
	case 9600:
		baud1=0x00;
		baud0=0x47;
		pres=0;
		break;
	case 19200:
		baud1=0x00;
		baud0=0x23;
		pres=0;
		break;
	case 38400:
		baud1=0x00;
		baud0=0x11;
		pres=0;
		break;
	case 76800:
		baud1=0x00;
		baud0=0x08;//0x08
		pres=0;
		break;
	case 1800:
		baud1=0x01;
		baud0=0x7f;
		pres=0;
		break;
	case 3600:
		baud1=0x00;
		baud0=0xbf;
		pres=0;
		break;
	case 7200:
		baud1=0x00;
		baud0=0x5f;
		pres=0;
		break;
	case 14400:
		baud1=0x00;
		baud0=0x2f;
		pres=0;
		break;
	case 28800:
		baud1=0x00;
		baud0=0x17;
		pres=0;
		break;
	case 57600:
		baud1=0x00;
		baud0=0x0b;
		pres=0;
		break;
	case 115200:
		baud1=0x00;
		baud0=0x05;
		pres=0;
		break;
	case 230400:
		baud1=0x00;
		baud0=0x02;
		pres=0;
		break;
	case 460800:
		baud1 = 0x00;
		baud0 = 0x00;
		pres = 0x08;
		break;
	case 500000:
		baud1 = 0x00;
		baud0 = 0x00;
		pres = 0x06;
		break;

	default:
		baud1=0x00;
		baud0=0x00;
		pres=0;
	}
    tty_termios_encode_baud_rate(termios, baud, baud);

    /* we are sending char from a workqueue so enable */

    lcr =0;

	//设置停止位
        if (cflag & CSTOPB)
			lcr |= WK2XXX_STPL;		//two  stop_bits
        else
			lcr &= ~WK2XXX_STPL;		//one  stop_bits

        if (cflag & PARENB) 	/* 是否是偶校验*/
		{
                lcr|=WK2XXX_PAEN;		//enbale spa
                if (!(cflag & PARODD)){

                        lcr |= WK2XXX_PAM1;
                        lcr &= ~WK2XXX_PAM0;
                }
                else{
                        lcr |= WK2XXX_PAM0;//PAM0=1
                        lcr &= ~WK2XXX_PAM1;//PAM1=0
                }
        }
        else{
                lcr&=~WK2XXX_PAEN;	/*奇校验*/
        }

#ifdef _DEBUG_WK2XXX
	 printk(KERN_ALERT "wk2xxx_termios()----port:=%d -- lcr:=0x%x -- cflag:=0x%x -- [CSTOPB:0x%x,PARENB:0x%x,PARODD:0x%x--]\n",s->port.iobase,lcr,cflag,CSTOPB,PARENB,PARODD);
#endif

		s->new_baud1=baud1;
		s->new_baud0=baud0;
		s->new_pres=pres;
		s->new_lcr = lcr;

		conf_wk2xxx_subport(&s->port);
#ifdef _DEBUG_WK2XXX
      printk(KERN_ALERT "-wk2xxx_termios------exit---\n");
#endif

}


static const char *wk2xxx_type(struct uart_port *port)
{
#ifdef _DEBUG_WK2XXX
	printk(KERN_ALERT "wk2xxx_type-------------out-------- \n");
#endif
	return port->type == PORT_WK2XXX ? "wk2xxx" : NULL;//this is defined in serial_core.h
}

/*
* Release the memory region(s) being used by 'port'.
*/
static void wk2xxx_release_port(struct uart_port *port)
{
    printk(KERN_ALERT "wk2xxx_release_port\n");
}

/*
* Request the memory region(s) being used by 'port'.
*/
static int wk2xxx_request_port(struct uart_port *port)//no such memory region needed for vk32
{
        printk(KERN_ALERT "wk2xxx_request_port\n");

        return 0;
}

/*
* Configure/autoconfigure the port*/
static void wk2xxx_config_port(struct uart_port *port, int flags)
{
        struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);

#ifdef _DEBUG_WK2XXX
        printk(KERN_ALERT "wk2xxx_config_port \n");
#endif

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

#ifdef _DEBUG_WK2XXX
        printk(KERN_ALERT "wk2xxx_verify_port \n");
#endif
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

static struct uart_ops wk2xxx_pops = {
        tx_empty	:       wk2xxx_tx_empty,
        set_mctrl	:       wk2xxx_set_mctrl,
        get_mctrl	:       wk2xxx_get_mctrl,
        stop_tx		:       wk2xxx_stop_tx,
        start_tx	:       wk2xxx_start_tx,
        stop_rx		:       wk2xxx_stop_rx,
        enable_ms	:       wk2xxx_enable_ms,
        break_ctl	:       wk2xxx_break_ctl,
        startup		:       wk2xxx_startup,
        shutdown	:       wk2xxx_shutdown,
        set_termios:       wk2xxx_termios,  //串口波特率和校验位接口
        type		:       wk2xxx_type,
        release_port:     wk2xxx_release_port,
        request_port	:      wk2xxx_request_port,
        config_port	:      wk2xxx_config_port,
        verify_port	:      wk2xxx_verify_port,

};
static struct uart_driver wk2xxx_uart_driver = {
        owner	:               THIS_MODULE,
        major	:               SERIAL_WK2XXX_MAJOR,
#ifdef CONFIG_DEVFS_FS
        driver_name	: "ttySWK",
        dev_name		: "ttysWK",
#else
        driver_name	:  "ttySWK",
        dev_name		:  "ttysWK",
#endif
        minor			:  MINOR_START,
        nr				:  NR_PORTS,
        cons			:  NULL//WK2Xxx_CONSOLE,
};

static int uart_driver_registered;
static struct spi_driver wk2xxx_driver;

static int wk2xxx_probe(struct spi_device *spi)
{
#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "-wk2xxx_probe()------in---\n");
#endif
	struct device_node *node = spi->dev.of_node;
	struct wk2xxx_port *wk2xxx_dev;
	uint8_t i;
	int status;
	uint8_t dat[1];
	int ret;
	int err;
#if 1
	printk("11wk2xxx_probe: setup mode %d, %s%s%s%s%u bits/w, %u Hz max\r\n", (int) (spi->mode & (SPI_CPOL | SPI_CPHA)),
			(spi->mode & SPI_CS_HIGH) ? "cs_high, " : "",
			(spi->mode & SPI_LSB_FIRST) ? "lsb, " : "",
			(spi->mode & SPI_3WIRE) ? "3wire, " : "",
			(spi->mode & SPI_LOOP) ? "loopback, " : "",
			spi->bits_per_word, spi->max_speed_hz);
#endif
	wk2xxx_dev = kzalloc(sizeof(struct wk2xxx_port), GFP_KERNEL);
	if (!wk2xxx_dev) {
		dev_err(&spi->dev, "spi device allocation failed");
		return -ENOMEM;
	}


	ret = of_get_named_gpio_flags(node, "power-gpio", 0, NULL);
	if (ret < 0) {
		printk("%s() Can not read property power-gpio\n", __FUNCTION__);
		goto err;
	} else {
		wk2xxx_dev->pwr = ret;
		ret = devm_gpio_request(&spi->dev, wk2xxx_dev->pwr, "power-gpio");

		if(ret < 0){
			printk("%s() devm_gpio_request pwr_gpio request ERROR\n", __FUNCTION__);
			goto err;
		}

		ret = gpio_direction_output(wk2xxx_dev->pwr,1);
		if(ret < 0){
			printk("%s() gpio_direction_output power-gpio set ERROR\n", __FUNCTION__);
			goto err;
		}
    }

	ret = of_get_named_gpio_flags(node, "reset-gpio", 0, NULL);
	if (ret < 0) {
		printk("%s() Can not read property reset-gpio\n", __FUNCTION__);
		goto err;
	} else {
		wk2xxx_dev->rst = ret;
		ret = devm_gpio_request(&spi->dev, wk2xxx_dev->rst, "reset-gpio");

		if(ret < 0){
			printk("%s() devm_gpio_request rst_gpio request ERROR\n", __FUNCTION__);
			goto err;
		}

		ret = gpio_direction_output(wk2xxx_dev->rst,1);
		if(ret < 0){
			printk("%s() gpio_direction_input reset-gpio set ERROR\n", __FUNCTION__);
			goto err;
		}
		mdelay(50);
		gpio_set_value(wk2xxx_dev->rst, 0);
		mdelay(100);
		gpio_set_value(wk2xxx_dev->rst, 1);
		mdelay(20);
	}

	ret = of_get_named_gpio_flags(node, "irq-gpio", 0, NULL);
	if (ret < 0) {
		printk("%s() Can not read property irq-gpio\n", __FUNCTION__);
		goto err;
	} else {
		wk2xxx_dev->irq_gpio= ret;
		ret = devm_gpio_request(&spi->dev, wk2xxx_dev->irq_gpio, "irq-gpio");
		//dump_stack();
		if(ret < 0){
			printk("%s() devm_gpio_request irq_gpio request ERROR\n", __FUNCTION__);
			goto err;
		}

		ret = gpio_direction_input(wk2xxx_dev->irq_gpio);
		if(ret < 0){
			printk("%s() gpio_direction_input irq_gpio set ERROR\n", __FUNCTION__);
			goto err;
		}

		if (wk2xxx_dev->irq_gpio) {
			spi->irq = gpio_to_irq(wk2xxx_dev->irq_gpio);
		}
	}

	ret = of_get_named_gpio_flags(node, "cs-gpio", 0, NULL);
	if (ret < 0) {
		printk("%s() Can not read property cs-gpio\n", __FUNCTION__);
		goto err;
	} else {
		wk2xxx_dev->cs = ret;
		ret = devm_gpio_request(&spi->dev, wk2xxx_dev->cs, "cs-gpio");
		if(ret < 0){
			printk("%s() devm_gpio_request cs_gpio request ERROR\n", __FUNCTION__);
			goto err;
		}

		ret = gpio_direction_output(wk2xxx_dev->cs,0);
		if(ret < 0){
			printk("%s() gpio_direction_input cs-gpio set ERROR\n", __FUNCTION__);
			goto err;
		}
	}

	wk2124_dev = wk2xxx_dev;

	spi->mode = SPI_MODE_0;
	if (!spi->max_speed_hz)
		spi->max_speed_hz = WK2XXX_SPI_MAX_SPEED_HZ;

	err = spi_setup(spi);
	if (err)
		return err;

	//read it twice for the first one which the clk always in low level before cs pull down.
	wk2xxx_read_reg(spi,WK2XXX_GPORT,WK2XXX_GENA,dat);
	wk2xxx_read_reg(spi,WK2XXX_GPORT,WK2XXX_GENA,dat);

     if((dat[0]&0xf0)!=0x30)
     {
          printk(KERN_ALERT "wk2xxx_probe()  GENA = 0x%X\n",dat[0]);
          printk(KERN_ERR "spi driver  error!!!!\n");
          return 1;
     }

	mutex_lock(&wk2xxxs_lock);

	/* 注册串口设备*/
	if(!uart_driver_registered)
	{
		uart_driver_registered = 1;
		status=uart_register_driver(&wk2xxx_uart_driver);
		if (status)
		{
			printk(KERN_ERR "Couldn't register wk2xxx uart driver\n");
			mutex_unlock(&wk2xxxs_lock);
			return status;
		}
	}
	printk(KERN_ALERT "wk2xxx_serial_init()\n");

    /*串口设备初始化*/
    for(i =0;i<NR_PORTS;i++)
     {
        struct wk2xxx_port *s = &wk2xxxs[i];//container_of(port,struct wk2xxx_port,port);
        s->tx_done       =0;
        s->spi_wk        = spi;
        s->port.line     = i;
        s->port.ops      = &wk2xxx_pops;
        s->port.uartclk  = WK_CRASTAL_CLK;
        s->port.fifosize = 64;
        s->port.iobase   = i+1;
        s->port.irq      = spi->irq;
        s->port.iotype   = SERIAL_IO_PORT;
        s->port.flags    = ASYNC_BOOT_AUTOCONF;
        //s->minor       = i;
        status = uart_add_one_port(&wk2xxx_uart_driver, &s->port);
        if(status<0)
        {
            printk(KERN_ALERT "uart_add_one_port failed for line i:= %d with error %d\n",i,status);
        }else
            printk(KERN_INFO "uart_add_one_port success for line i:= %d with right %d\n",i,status);

        //=============================================================
        //spi_set_drvdata(spi, s);
        //=============================================================
    }

		printk(KERN_ALERT "uart_add_one_port = 0x%d\n",status);
		mutex_unlock(&wk2xxxs_lock);

		return 0;
err:
	return ret;
}

static int wk2xxx_remove(struct spi_device *spi)
{

    int i;
#ifdef _DEBUG_WK2XXX
        printk(KERN_ALERT "-wk2xxx_remove()------in---\n");
#endif

    mutex_lock(&wk2xxxs_lock);
    for(i =0;i<NR_PORTS;i++)
   {
            struct wk2xxx_port *s = &wk2xxxs[i];
            uart_remove_one_port(&wk2xxx_uart_driver, &s->port);
    }
    printk(KERN_ALERT "removing wk2xxx driver\n");
    uart_unregister_driver(&wk2xxx_uart_driver);
    mutex_unlock(&wk2xxxs_lock);
#ifdef _DEBUG_WK2XXX
    printk(KERN_ALERT "-wk2xxx_remove()------exit---\n");
#endif

    return 0;
}

static const struct spi_device_id wk2xxx_id[] = {
	{ "wk2xxx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, wk2xxx_id);

#ifdef CONFIG_OF
static const struct of_device_id wk2xxx_of_match[] = {
	{ .compatible = "firefly,spi-wk2xxx" },
	{}
};
MODULE_DEVICE_TABLE(of, wk2xxx_of_match);
#endif

static struct spi_driver wk2xxx_driver = {
        .driver = {
                .name           = "wk2xxxspi",
                .bus            = &spi_bus_type,
                .owner          = THIS_MODULE,
		  .of_match_table = of_match_ptr(wk2xxx_of_match),
        },
        .probe          = wk2xxx_probe,
        .remove         = wk2xxx_remove,
	.id_table		= wk2xxx_id,
};

static int __init wk2xxx_init(void)
{
	int retval;
	retval = spi_register_driver(&wk2xxx_driver);
	printk(KERN_ALERT "register spi return v = :%d\n",retval);
	return retval;
}

static void __exit wk2xxx_exit(void)
{
        spi_unregister_driver(&wk2xxx_driver);
        printk("TEST_REG:quit ");
}

module_init(wk2xxx_init);
module_exit(wk2xxx_exit);

MODULE_AUTHOR("WKIC Ltd");
MODULE_DESCRIPTION("wk2xxx generic serial port driver");
MODULE_LICENSE("GPL");

