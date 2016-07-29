/**
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 * author: ZhiChao Yu zhichao.yu@rock-chips.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dmapool.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include "dsp_ioctl.h"
#include "dsp_dbg.h"
#include "dsp_dev.h"
#include "dsp_work.h"

/* dma pool buffer size 1K */
#define DSP_DMA_POOL_BUFFER_SIZE    (1 * 1024)

#define DSP_WORK_TIMEOUT_MS  500
#define DSP_SESSION_ID_START 1000

/*
 * User can write debug mask value by
 * echo <mask> > /sys/kernel/debug/dsp/debug
 */
u32 dsp_debug_mask;

/*
 * dsp_session
 * TODO
 */
struct dsp_session {
	u32 id;

	wait_queue_head_t wait;
	struct list_head done;

	struct list_head list_node;
	void *owner;
};

/*
 * dsp_service
 * TODO
 */
struct dsp_service {
	struct dsp_dev *dev;
	struct dsp_dev_client dev_client;
	struct dma_pool *dma_pool;

	wait_queue_head_t wait;
	struct task_struct *work_consumer;

	struct list_head sessions;
	struct list_head running;
	struct list_head pending;

	/* use counter to generate session id */
	u32 counter;
	/* reference count */
	atomic_t ref;
	/* lock work lists */
	struct mutex lock;
};

/*
 * rk_dsp - Informations and context of DSP
 * TODO
 */
struct rk_dsp {
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	struct dsp_service service;
};

#ifdef CONFIG_DEBUG_FS
static struct dentry *dsp_debugfs_root;

static int dsp_debugfs_init(void)
{
	dsp_debugfs_root = debugfs_create_dir("dsp", NULL);
	if (!dsp_debugfs_root)
		return -EFAULT;

	debugfs_create_u32("debug", 0644, dsp_debugfs_root, &dsp_debug_mask);
	return 0;
}

static void dsp_debugfs_exit(void)
{
	debugfs_remove(dsp_debugfs_root);
	dsp_debugfs_root = NULL;
}
#endif

/*
 * dsp_queue_work - queue a work to pending list which will be done
 * by DSP.
 *
 * @session: presents a connection between user process and kernel driver
 * @work: work to be queued
 */
static int dsp_queue_work(struct dsp_session *session, struct dsp_work *work)
{
	int ret = 0;
	struct dsp_service *service = session->owner;

	dsp_debug_enter();
	dsp_debug(DEBUG_SERVICE, "queue work id=0x%08x\n", work->id);

	mutex_lock(&service->lock);
	list_add_tail(&work->list_node, &service->pending);
	mutex_unlock(&service->lock);

	wake_up(&service->wait);
	dsp_debug_leave();
	return ret;
}

/*
 * dsp_dequeue_work - dequeue a work from session's done list, if the done
 * list is empty, then the process will be blocked until a work with this
 * session is done.
 *
 * @session: Presents a connection between user process and kernel driver
 * @work_out: output work dequeued
 */
static int dsp_dequeue_work(struct dsp_session *session,
			    struct dsp_work **work_out)
{
	int ret = 0;
	int timeout = 0;
	struct dsp_work *work;

	dsp_debug_enter();

	timeout = wait_event_timeout(session->wait, !list_empty(&session->done),
				     DSP_WORK_TIMEOUT_MS);
	if (unlikely(timeout <= 0)) {
		dsp_err("dequeue work timeout\n");
		ret = -EBUSY;
		goto out;
	}

	work = list_first_entry(&session->done, struct dsp_work, list_node);
	list_del(&work->list_node);
	dsp_debug(DEBUG_SERVICE, "dequeue work id=0x%08x\n", work->id);
	(*work_out) = work;
out:
	if (ret)
		(*work_out) = NULL;
	dsp_debug_leave();
	return ret;
}

/*
 * dsp_session_create - create a session when a user process open
 * DSP device file. A session presents a connection between user
 * process and kernel driver.
 *
 * @service: DSP service
 * @session_out: return the created session
 */
static int dsp_session_create(struct dsp_service *service,
			      struct dsp_session **session_out)
{
	int ret = 0;
	struct dsp_session *session;

	dsp_debug_enter();

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session) {
		dsp_err("cannot alloc mem for session\n");
		ret = -ENOMEM;
		goto out;
	}

	INIT_LIST_HEAD(&session->done);
	init_waitqueue_head(&session->wait);
	session->id = service->counter++;
	session->owner = service;

	mutex_lock(&service->lock);
	list_add_tail(&session->list_node, &service->sessions);
	mutex_unlock(&service->lock);
	atomic_add(1, &service->ref);

	(*session_out) = session;

out:
	if (ret)
		(*session_out) = NULL;
	dsp_debug_leave();
	return ret;
}

static int dsp_session_destroy(struct dsp_session *session)
{
	struct dsp_service *service = session->owner;

	dsp_debug_enter();

	/* TODO must to check works of this session has been done */
	mutex_lock(&service->lock);
	list_del(&session->list_node);
	mutex_unlock(&service->lock);
	atomic_sub(1, &service->ref);
	kfree(session);

	dsp_debug_leave();

	return 0;
}

/*
 * dsp_service_clean_pending_works - clean works in service pending list
 * for a specific session. These cleaned works will be add to the sessions
 * done list with a EABADON result. This function should be called in
 * dsp_release for destroy a session carefully purpose.
 *
 * @service: service ptr
 * @session: session prt
 */
static int dsp_service_clean_pending_works(struct dsp_service *service,
					   struct dsp_session *session)
{
	int ret = 0;
	struct list_head *pos, *n;

	dsp_debug_enter();

	mutex_lock(&service->lock);
	list_for_each_safe(pos, n, &service->pending) {
		struct dsp_work *work = list_entry(pos, struct dsp_work,
			list_node);

		if (work->session == (u32)session) {
			work->result = DSP_WORK_EABANDON;
			list_del(&work->list_node);
			list_add_tail(&work->list_node, &session->done);
		}
	}
	mutex_unlock(&service->lock);

	dsp_debug_leave();
	return ret;
}

/*
 * dsp_work_consume - work consumer thread function, consumer thread will
 * be waked up when pending list is not empty.
 *
 * @data: service ptr
 */
static int dsp_work_consume(void *data)
{
	int ret = 0;
	struct dsp_service *service = data;
	struct dsp_work *work;

	dsp_debug_enter();

	service->dev->config(service->dev);

	while (!kthread_should_stop()) {
		if (!wait_event_timeout(service->wait,
					!list_empty(&service->pending), HZ))
			continue;

		mutex_lock(&service->lock);
		work = list_first_entry(&service->pending,
					struct dsp_work, list_node);
		list_del(&work->list_node);
		list_add_tail(&work->list_node, &service->running);
		mutex_unlock(&service->lock);

		dsp_debug(DEBUG_SERVICE, "consume a work=0x%08x\n", work->id);

		ret = service->dev->work(service->dev, work);
		if (ret)
			dsp_err("fatal error, dsp work failed\n");
	}

	dsp_debug_leave();
	return 0;
}

/*
 * dsp_work_done - a callback function will be called when
 * a work has been done by DSP core.
 *
 * @client: DSP device client prt
 * @work: work ptr
 */
static int dsp_work_done(struct dsp_dev_client *client, struct dsp_work *work)
{
	int ret = 0;
	struct dsp_session *session = (struct dsp_session *)work->session;
	struct dsp_service *service = session->owner;

	dsp_debug_enter();

	mutex_lock(&service->lock);
	list_del(&work->list_node);
	list_add_tail(&work->list_node, &session->done);
	mutex_unlock(&service->lock);

	dsp_debug(DEBUG_SERVICE, "work done id=0x%08x\n", work->id);
	dsp_debug_leave();

	wake_up(&session->wait);
	return ret;
}

/*
 * dsp_device_ready - a callback function will be called when
 * DSP core is ready to work.
 *
 * @client: DSP device client ptr
 */
static int dsp_device_ready(struct dsp_dev_client *client)
{
	int ret = 0;
	struct dsp_service *service = client->data;

	dsp_debug_enter();
	service->work_consumer = kthread_run(dsp_work_consume, service,
				"dsp_work_consumer");
	dsp_debug_leave();
	return ret;
}

/*
 * dsp_service_prepare - prepare DSP service
 *
 * @pdev: platform device
 * @service: service prt to be prepared
 */
static int dsp_service_prepare(struct platform_device *pdev,
			       struct dsp_service *service)
{
	int ret = 0;

	dsp_debug_enter();

	mutex_init(&service->lock);
	init_waitqueue_head(&service->wait);

	INIT_LIST_HEAD(&service->running);
	INIT_LIST_HEAD(&service->pending);
	INIT_LIST_HEAD(&service->sessions);

	service->dev_client.data = service;
	service->dev_client.device_ready = dsp_device_ready;
	service->dev_client.work_done = dsp_work_done;

	atomic_set(&service->ref, 0);
	service->counter = DSP_SESSION_ID_START;

	service->dma_pool = dma_pool_create("dsp_dma", &pdev->dev,
			DSP_DMA_POOL_BUFFER_SIZE, 32, 0);
	if (!service->dma_pool) {
		dsp_err("cannot alloc dma pool for dsp\n");
		ret = -ENOMEM;
		goto out;
	}

	ret = dsp_dev_create(pdev, service->dma_pool, &service->dev);
	if (ret) {
		dsp_err("cannot open dsp device\n");
		goto out;
	}
	dsp_dev_register_client(service->dev, &service->dev_client);
out:
	if (ret) {
		if (service->dma_pool)
			dma_pool_destroy(service->dma_pool);
	}
	dsp_debug_leave();
	return ret;
}

/*
 * dsp_service_release - release service, if there any reamin sessions,
 * the return value is fail.
 *
 * @service: service ptr
 */
static int dsp_service_release(struct dsp_service *service)
{
	int ret = 0;

	dsp_debug_enter();

	if (!list_empty(&service->sessions)) {
		ret = -EFAULT;
		dsp_err("cannot release service with remaining sessions\n");
		goto out;
	}

	if (!list_empty(&service->pending) || !list_empty(&service->running)) {
		ret = -EFAULT;
		dsp_err("Unclaimed works in pending list\n");
		goto out;
	}

	kthread_stop(service->work_consumer);

	dma_pool_destroy(service->dma_pool);
	ret = dsp_dev_destroy(service->dev);
out:
	dsp_debug_leave();
	return ret;
}

static long dsp_ioctl(struct file *filp, unsigned int cmd,
		      unsigned long arg)
{
	int ret = 0;
	struct dsp_session *session = filp->private_data;
	struct dsp_service *service = session->owner;

	if (!session)
		return -EINVAL;

	dsp_debug_enter();

	switch (cmd) {
	case DSP_IOC_QUEUE_WORK: {
		struct dsp_work *work;

		dsp_debug(DEBUG_SERVICE, "queue work\n");

		ret = dsp_work_create(service->dma_pool, session, &work);
		if (!ret) {
			dsp_work_copy_from_user(service->dma_pool, work,
						(void *)arg);
			ret = dsp_queue_work(session, work);
			if (ret) {
				dsp_err("queue work failed\n");
				dsp_work_destroy(service->dma_pool, work);
			}
		}
	} break;

	case DSP_IOC_DEQUEUE_WORK: {
		struct dsp_work *work;

		dsp_debug(DEBUG_SERVICE, "dequeue work\n");

		ret = dsp_dequeue_work(session, &work);
		if (work) {
			dsp_work_copy_to_user(work, (void *)arg);
			dsp_work_destroy(service->dma_pool, work);
		} else {
			struct dsp_user_work user_work;

			if (ret == -EBUSY) {
				user_work.hdl = 0;
				user_work.result = DSP_WORK_ETIMEOUT;
			}
			if (copy_to_user((void __user *)arg,
					 &user_work, sizeof(user_work)))
				dsp_err("dequeue work copy failed\n");
		}

	} break;
	}

	dsp_debug_leave();
	return 0;
}

static int dsp_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct rk_dsp *dsp;
	struct dsp_session *session;
	struct dsp_service *service;

	dsp_debug_enter();

	dsp = container_of(inode->i_cdev, struct rk_dsp, cdev);
	if (!dsp) {
		dsp_err("invalid dsp handle\n");
		ret = -EINVAL;
		goto out;
	}
	service = &dsp->service;

	ret = dsp_session_create(service, &session);
	if (ret) {
		dsp_err("cannot create a session\n");
		goto out;
	}

	/* Power on DSP if service has more than one session */
	if (atomic_read(&service->ref))
		service->dev->on(service->dev);

	filp->private_data = session;
out:
	dsp_debug_leave();
	return ret;
}

static int dsp_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct dsp_session *session = filp->private_data;
	struct dsp_service *service = session->owner;

	dsp_debug_enter();

	/* waiting running done */
	while (!list_empty(&service->running))
		msleep(20);

	dsp_service_clean_pending_works(service, session);
	ret = dsp_session_destroy(session);

	/* Power off DSP if service has not sessions anymore */
	if (!atomic_read(&service->ref))
		service->dev->off(service->dev);

	dsp_debug_leave();
	return ret;
}

static const struct file_operations dsp_fops = {
	.unlocked_ioctl = dsp_ioctl,
	.open           = dsp_open,
	.release        = dsp_release,
#ifdef CONFIG_COMPAT
	/* TODO compat_ioctl */
#endif
};

static int dsp_cdev_create(struct device *dev, struct rk_dsp *dsp)
{
	int ret = 0;
	struct device_node *np = dev->of_node;
	char *name = (char *)dev_name(dev);

	dsp_debug_enter();

	of_property_read_string(np, "name", (const char **)&name);

	ret = alloc_chrdev_region(&dsp->devt, 0, 1, name);
	if (ret) {
		dsp_err("alloc devt failed\n");
		goto out;
	}

	cdev_init(&dsp->cdev, &dsp_fops);
	dsp->cdev.owner = THIS_MODULE;
	dsp->cdev.ops = &dsp_fops;
	ret = cdev_add(&dsp->cdev, dsp->devt, 1);
	if (ret) {
		dev_err(dev, "add cdev failed\n");
		goto out;
	}

	dsp->class = class_create(THIS_MODULE, name);
	if (IS_ERR(dsp->class)) {
		ret = PTR_ERR(dsp->class);
		dsp_err("create class failed\n");
		goto out;
	}

	device_create(dsp->class, dev, dsp->devt, NULL, name);

out:
	if (ret)
		unregister_chrdev_region(dsp->devt, 1);
	dsp_debug_leave();
	return ret;
}

static int dsp_cdev_destroy(struct rk_dsp *dsp)
{
	dsp_debug_enter();

	device_destroy(dsp->class, dsp->devt);
	class_destroy(dsp->class);
	cdev_del(&dsp->cdev);
	unregister_chrdev_region(dsp->devt, 1);

	dsp_debug_leave();
	return 0;
}

static int dsp_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct rk_dsp *dsp;

	dsp_debug_enter();

	if (DSP_DMA_POOL_BUFFER_SIZE < sizeof(struct dsp_work)) {
		dsp_err("dma buffer size is smaller than sizeof work\n");
		ret = -EFAULT;
		goto out;
	}

	dsp = devm_kzalloc(dev, sizeof(*dsp), GFP_KERNEL);
	if (!dsp) {
		dsp_err("cannot alloc mem for rk_dsp struct\n");
		ret = -ENOMEM;
		goto out;
	}

	ret = dsp_cdev_create(dev, dsp);
	if (ret) {
		dsp_err("create char deivce failed\n");
		goto out;
	}

	ret = dsp_service_prepare(pdev, &dsp->service);
	if (ret) {
		dsp_err("prepare dsp service failed\n");
		goto out;
	}

	platform_set_drvdata(pdev, dsp);

out:
	if (ret)
		dsp_cdev_destroy(dsp);

	dsp_debug_leave();
	return ret;
}

static int dsp_remove(struct platform_device *pdev)
{
	struct rk_dsp *dsp = platform_get_drvdata(pdev);

	if (!dsp)
		goto out;

	dsp_debug_enter();
	dsp_service_release(&dsp->service);
	dsp_cdev_destroy(dsp);
	dsp_debug_leave();

out:
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id dsp_dt_ids[] = {
	{.compatible = "rockchip,dsp",},
	{},
};
#endif

static struct platform_driver dsp_driver = {
	.probe = dsp_probe,
	.remove = dsp_remove,
	.driver = {
		.name = "dsp",
		.owner = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(dsp_dt_ids),
#endif
	},
};

static int __init dsp_init(void)
{
	int ret = platform_driver_register(&dsp_driver);

#ifdef CONFIG_DEBUG_FS
	dsp_debugfs_init();
#endif

	return ret;
}

static void __exit dsp_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	dsp_debugfs_exit();
#endif

	platform_driver_unregister(&dsp_driver);
}

module_init(dsp_init);
module_exit(dsp_exit);
MODULE_LICENSE("GPL");
