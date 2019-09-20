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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>

#include "spi-rkpreisp.h"
#include "spi2apb.h"
#include "msg-queue.h"
#include "isp-fw.h"
#include "ap-i2c.h"

#define ENABLE_CACHE_FIRMWARE 0

#define DEBUG_DUMP_ALL_SEND_RECV_MSG 0

#define DEBUG_MSG_LOOP_TEST 0

#define INVALID_ID -1

#define PREISP_MCLK_RATE (24 * 1000 * 1000ul)

#define PREISP_WAKEUP_TIMEOUT_MS 100
#define PREISP_REQUEST_SLEEP_TIMEOUT_MS 100

#define PREISP_PDVDD_VOL_DEFAULT 1100000

#define PREISP_VDD_CORE_VOL_DEFAULT 1150000

typedef struct {
	int8_t id;
	struct msg_queue q;
	struct list_head list;
	wait_queue_head_t wait;
	void *private_data;
} preisp_client;

typedef struct {
	struct mutex mutex;/* mutex */
	struct list_head list;
} preisp_client_list;

struct spi_rk_preisp_data {
	struct miscdevice misc;
	struct spi_device *spi;
	struct device *dev;
	preisp_client_list clients;
	int reset_gpio;
	int reset_active;
	int irq_gpio;
	int irq;
	int sleepst_gpio;
	int sleepst_irq;
	int sleep_mode;
	int wakeup_gpio;
	int wakeup_active;
	struct clk *mclk;
	atomic_t power_on_cnt;
	atomic_t wake_sleep_cnt;
	struct mutex send_msg_lock;/* mutex */
	struct mutex power_lock;/* mutex */
	struct wake_lock resume_wake_lock;
	uint32_t max_speed_hz;
	uint32_t min_speed_hz;
	uint32_t fw_nowait_mode;
	struct regulator *pdvdd_regu;
	struct regulator *vdd_core_regu;
	int log_level;
	int sleep_state_flag;
	bool is_resume_processing;
};

enum {
	AUTO_ARG_TYPE_STR,
	AUTO_ARG_TYPE_INT32,
};

struct auto_arg {
	int type;
	union {
		int32_t m_int32;
		const char *m_str;
	};
};

struct auto_args {
	int argc;
	struct auto_arg *argv;
};

static void preisp_set_spi_speed(struct spi_rk_preisp_data *pdata, uint32_t hz)
{
	pdata->spi->max_speed_hz = hz;
}

static int preisp_set_regu_vol(struct regulator *regu, int uV)
{
	int ret = 0;
	unsigned int step;
	int min_uV;

	if (IS_ERR(regu))
		return -1;

	step = regulator_get_linear_step(regu);
	min_uV = uV - step + 1;

	ret = regulator_set_voltage(regu, min_uV, uV);

	return ret;
}

static void preisp_client_list_init(preisp_client_list *s)
{
	mutex_init(&s->mutex);
	INIT_LIST_HEAD(&s->list);
}

static preisp_client *preisp_client_new(void)
{
	preisp_client *c =
		(preisp_client *)kzalloc(sizeof(preisp_client), GFP_KERNEL);
	c->id = INVALID_ID;
	INIT_LIST_HEAD(&c->list);
	msq_init(&c->q, MSG_QUEUE_DEFAULT_SIZE);
	init_waitqueue_head(&c->wait);
	return c;
}

static void preisp_client_release(preisp_client *c)
{
	msq_release(&c->q);
	kfree(c);
}

static preisp_client *preisp_client_find(preisp_client_list *s, preisp_client *c)
{
	preisp_client *client = NULL;

	list_for_each_entry(client, &s->list, list) {
		if (c == client)
			return c;
	}
	return NULL;
}

static int preisp_client_connect(struct spi_rk_preisp_data *pdata, preisp_client *c)
{
	preisp_client_list *s = &pdata->clients;

	mutex_lock(&s->mutex);
	if (preisp_client_find(s, c)) {
		mutex_unlock(&s->mutex);
		return -1;
	}

	list_add_tail(&c->list, &s->list);
	mutex_unlock(&s->mutex);

	return 0;
}

static void preisp_client_disconnect(struct spi_rk_preisp_data *pdata, preisp_client *c)
{
	preisp_client_list *s = &pdata->clients;

	mutex_lock(&s->mutex);
	if (preisp_client_find(s, c))
		list_del_init(&c->list);

	mutex_unlock(&s->mutex);
}

static void spi_cs_set_value(struct spi_rk_preisp_data *pdata, int value)
{
	int8_t null_cmd = 0;
	struct spi_transfer null_cmd_packet = {
		.tx_buf = &null_cmd,
		.len	= sizeof(null_cmd),
		.cs_change = !value,
	};
	struct spi_message  m;

	spi_message_init(&m);
	spi_message_add_tail(&null_cmd_packet, &m);
	spi_sync(pdata->spi, &m);
}

static void rkpreisp_hw_init(struct spi_device *spi)
{
	spi2apb_safe_w32(spi, 0x12008098, 0xff004000);
}

static int preisp_send_msg_to_dsp(struct spi_rk_preisp_data *pdata, const struct msg *m)
{
	int ret = -1;

	mutex_lock(&pdata->power_lock);
	if (atomic_read(&pdata->power_on_cnt) > 0 &&
		atomic_read(&pdata->wake_sleep_cnt) > 0) {
		mutex_lock(&pdata->send_msg_lock);
		ret = dsp_msq_send_msg(pdata->spi, m);
		mutex_unlock(&pdata->send_msg_lock);
	}
	mutex_unlock(&pdata->power_lock);

	return ret;
}

static int rkpreisp_set_log_level(struct spi_rk_preisp_data *pdata, int level)
{
	int ret = 0;

	MSG(msg_set_log_level_t, m);
	m.log_level = level;
	ret = preisp_send_msg_to_dsp(pdata, (struct msg *)&m);
	return ret;
}

static int rkpreisp_request_sleep_nolock(struct spi_rk_preisp_data *pdata, int32_t mode)
{
	int ret;
	int try = 0;

	MSG(msg_set_sys_mode_standby_t, m);

	if (atomic_dec_return(&pdata->wake_sleep_cnt) == 0) {
		if (mode >= PREISP_SLEEP_MODE_MAX || mode < 0) {
			dev_warn(pdata->dev, "Unknown sleep mode %d\n", mode);
			return -1;
		}
		dev_info(pdata->dev, "request sleep\n");

		if (mode == PREISP_SLEEP_MODE_BYPASS) {
			m.type = id_msg_set_sys_mode_bypass_t;
			pdata->sleep_mode = PREISP_SLEEP_MODE_BYPASS;
		} else {
			pdata->sleep_mode = PREISP_SLEEP_MODE_STANDBY;
		}

		if (pdata->wakeup_gpio > 0)
			gpio_set_value(pdata->wakeup_gpio, !pdata->wakeup_active);

		mutex_lock(&pdata->send_msg_lock);
		ret = dsp_msq_send_msg(pdata->spi, (struct msg *)&m);
		mutex_unlock(&pdata->send_msg_lock);

		do {
			if (pdata->sleep_state_flag) {
				ret = 0;
				pdata->sleep_state_flag = 0;
				mdelay(5);
				break;
			}
			if (try++ == PREISP_REQUEST_SLEEP_TIMEOUT_MS) {
				ret = -1;
				dev_err(pdata->dev, "request sleep timeout\n");
				break;
			}
			mdelay(1);
		} while (1);

		dev_info(pdata->dev, "request dsp enter %s mode. ret:%d",
				mode == PREISP_SLEEP_MODE_BYPASS ? "bypass" : "sleep", ret);
	} else if (atomic_read(&pdata->wake_sleep_cnt) < 0) {
		atomic_set(&pdata->wake_sleep_cnt, 0);
	}

	return ret;
}

static int rkpreisp_request_sleep(struct spi_rk_preisp_data *pdata, int32_t mode)
{
	int ret;

	mutex_lock(&pdata->power_lock);
	ret = rkpreisp_request_sleep_nolock(pdata, mode);
	mutex_unlock(&pdata->power_lock);

	return ret;
}

static int rkpreisp_download_fw(struct spi_rk_preisp_data *pdata, char *fw_name)
{
	int ret;
	/* request rkpreisp enter slave mode */
	spi_cs_set_value(pdata, 0);
	if (pdata->wakeup_gpio > 0)
		gpio_set_value(pdata->wakeup_gpio, pdata->wakeup_active);

	mdelay(3);
	if (pdata->reset_gpio > 0)
		gpio_set_value(pdata->reset_gpio, pdata->reset_active);

	mdelay(5);
	spi_cs_set_value(pdata, 1);
	preisp_set_spi_speed(pdata, pdata->min_speed_hz);
	spi2apb_switch_to_msb(pdata->spi);
	rkpreisp_hw_init(pdata->spi);

	preisp_set_spi_speed(pdata, pdata->max_speed_hz);
	/* download system firmware */
	ret = preisp_spi_download_fw(pdata->spi, fw_name);
	if (ret)
		dev_err(pdata->dev, "download firmware failed!");
	else
		dev_info(pdata->dev, "download firmware success!");

#if ENABLE_CACHE_FIRMWARE
	uncache_firmware(RKL_DEFAULT_FW_NAME);
#endif

	enable_irq(pdata->irq);
	if (pdata->sleepst_irq > 0)
		enable_irq(pdata->sleepst_irq);

	return ret;
}

static int rkpreisp_reset(struct spi_rk_preisp_data *pdata, char *fw_name)
{
	int ret = 0;

	disable_irq(pdata->irq);
	if (pdata->sleepst_irq > 0)
		disable_irq(pdata->sleepst_irq);

	if (pdata->reset_gpio > 0)
		gpio_set_value(pdata->reset_gpio, !pdata->reset_active);

	mdelay(3);
	ret = rkpreisp_download_fw(pdata, fw_name);

	return ret;
}

static int rkpreisp_wakeup_nolock(struct spi_rk_preisp_data *pdata)
{
	int32_t reg = 0;
	int try = 0, ret = 0;

	if (atomic_inc_return(&pdata->wake_sleep_cnt) == 1) {
		/* enable vdd core regulator */
		if (!IS_ERR(pdata->vdd_core_regu)) {
			if (regulator_is_enabled(pdata->vdd_core_regu) <= 0)
				ret = regulator_enable(pdata->vdd_core_regu);
		}
		if (pdata->wakeup_gpio > 0)
			gpio_set_value(pdata->wakeup_gpio, pdata->wakeup_active);
		else
			dev_info(pdata->dev, "please config wakeup gpio first!");

		/* waiting for dsp wakeup */
		mdelay(10);

		do {
			ret = spi2apb_safe_r32(pdata->spi, DSP_PMU_SYS_REG0, &reg);

			if (!ret && ((reg & DSP_MSG_QUEUE_OK_MASK) == DSP_MSG_QUEUE_OK_TAG)) {
				dev_info(pdata->dev, "wakeup dsp.");
				break;
			}

			if (try++ == PREISP_WAKEUP_TIMEOUT_MS) {
				dev_err(pdata->dev, "wakeup timeout, restart preisp\n");
				ret = rkpreisp_reset(pdata, RKL_DEFAULT_FW_NAME);
				break;
			}
			mdelay(1);
		} while (1);
	}

	return ret;
}

static int rkpreisp_wakeup(struct spi_rk_preisp_data *pdata)
{
	int ret = 0;

	mutex_lock(&pdata->power_lock);
	ret = rkpreisp_wakeup_nolock(pdata);
	mutex_unlock(&pdata->power_lock);

	return ret;
}

static int rkpreisp_power_on(struct spi_rk_preisp_data *pdata)
{
	int ret = 0;
	mutex_lock(&pdata->power_lock);

	if (atomic_inc_return(&pdata->power_on_cnt) == 1) {
		dev_info(pdata->dev, "dsp power on!");
		/* do power/clk on */
		if (pdata->mclk) {
			clk_prepare_enable(pdata->mclk);
			clk_set_rate(pdata->mclk, PREISP_MCLK_RATE);
		}

		/* enable pvdd-regulator */
		if (!IS_ERR(pdata->pdvdd_regu)) {
			if (regulator_is_enabled(pdata->pdvdd_regu) <= 0)
				ret = regulator_enable(pdata->pdvdd_regu);
		}
		/* enable vdd-core-regulator */
		if (!IS_ERR(pdata->vdd_core_regu)) {
			if (regulator_is_enabled(pdata->vdd_core_regu) <= 0)
				ret = regulator_enable(pdata->vdd_core_regu);
		}
		pdata->sleep_mode = 0;
		/* download fw and start run */
		ret = rkpreisp_download_fw(pdata, RKL_DEFAULT_FW_NAME);

		pdata->sleep_state_flag = 0;
		atomic_set(&pdata->wake_sleep_cnt, 1);
	} else {
		ret = rkpreisp_wakeup_nolock(pdata);
	}
	mutex_unlock(&pdata->power_lock);

	if (!ret)
		rkpreisp_set_log_level(pdata, pdata->log_level);

	return ret;
}

static int rkpreisp_power_off(struct spi_rk_preisp_data *pdata)
{
	int ret = 0;
	mutex_lock(&pdata->power_lock);

	if (atomic_dec_return(&pdata->power_on_cnt) == 0) {
#if ENABLE_CACHE_FIRMWARE
		cache_firmware(RKL_DEFAULT_FW_NAME);
#endif
		/* do power/clk off */
		dev_info(pdata->dev, "dsp power off!");
		mutex_unlock(&pdata->power_lock);
		disable_irq(pdata->irq);
		if (pdata->sleepst_irq > 0)
			disable_irq(pdata->sleepst_irq);

		mutex_lock(&pdata->power_lock);

		/* disable vdd-core-regulator */
		if (!IS_ERR(pdata->vdd_core_regu)) {
			if (regulator_is_enabled(pdata->vdd_core_regu) > 0)
				regulator_disable(pdata->vdd_core_regu);
		}
		/* disable pdvdd-regulator */
		if (!IS_ERR(pdata->pdvdd_regu)) {
			if (regulator_is_enabled(pdata->pdvdd_regu) > 0)
				regulator_disable(pdata->pdvdd_regu);
		}
		if (pdata->wakeup_gpio > 0)
			gpio_set_value(pdata->wakeup_gpio, !pdata->wakeup_active);

		if (pdata->reset_gpio > 0)
			gpio_set_value(pdata->reset_gpio, !pdata->reset_active);

		spi_cs_set_value(pdata, 0);
		if (pdata->mclk)
			clk_disable_unprepare(pdata->mclk);

		atomic_set(&pdata->wake_sleep_cnt, 0);
	} else if (atomic_read(&pdata->power_on_cnt) < 0) {
		atomic_set(&pdata->power_on_cnt, 0);
	} else {
		ret = rkpreisp_request_sleep_nolock(pdata, PREISP_SLEEP_MODE_STANDBY);
	}
	mutex_unlock(&pdata->power_lock);

	return ret;
}

static void fw_nowait_power_on(const struct firmware *fw, void *context)
{
	int ret = 0;
	struct spi_rk_preisp_data *pdata = context;

	ret = rkpreisp_power_on(pdata);
	if (!ret) {
		mdelay(10); /*delay for dsp boot*/
		rkpreisp_request_sleep(pdata, PREISP_SLEEP_MODE_STANDBY);
	}
	if (fw)
		release_firmware(fw);

	if (pdata->is_resume_processing) {
		dev_info(pdata->dev, "resume done!\n");
		wake_unlock(&pdata->resume_wake_lock);
		pdata->is_resume_processing = false;
	}
}

static int parse_arg(const char *s, struct auto_arg *arg)
{
	if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
		long v;

		v = simple_strtol(s, NULL, 16);
		arg->type = AUTO_ARG_TYPE_INT32;
		arg->m_int32 = v;
	} else if (isdigit(s[0])) {
		long v;

		v = simple_strtol(s, NULL, 10);
		arg->type = AUTO_ARG_TYPE_INT32;
		arg->m_int32 = v;
	} else {
		arg->type = AUTO_ARG_TYPE_STR;
		arg->m_str = s;
	}

	return 0;
}

static int parse_auto_args(char *s, struct auto_args *args)
{
	int i = 0;
	char c = 0;
	int last_is_arg_flag = 0;
	const char *last_arg;

	args->argc = 0;

	i = -1;
	do {
		c = s[++i];
		if (c == ' ' || c == ',' || c == '\n' || c == '\r' || c == 0) {
			if (last_is_arg_flag) {
				args->argc++;
			}
			last_is_arg_flag = 0;
		} else {
			last_is_arg_flag = 1;
		}
	} while (c != 0 && c != '\n' && c != '\r');

	args->argv = (struct auto_arg *)kmalloc(args->argc * sizeof(struct auto_arg), GFP_KERNEL);
	if (!args->argv)
		return -ENOMEM;

	i = -1;
	last_is_arg_flag = 0;
	last_arg = s;
	args->argc = 0;
	do {
		c = s[++i];
		if (c == ' ' || c == ',' || c == '\n' || c == '\r' || c == 0) {
			if (last_is_arg_flag) {
				parse_arg(last_arg, args->argv + args->argc++);
				s[i] = 0;
			}
			last_is_arg_flag = 0;
		} else {
			if (last_is_arg_flag == 0) {
				last_arg = s + i;
			}
			last_is_arg_flag = 1;
		}
	} while (c != 0 && c != '\n' && c != '\r');

	return c == 0 ? i : i + 1;
}

static void free_auto_args(struct auto_args *args)
{
	kfree(args->argv);
	args->argc = 0;
}

static void int32_hexdump(const char *prefix, int32_t *data, int len)
{
	pr_err("%s\n", prefix);
	print_hex_dump(KERN_ERR, "offset ", DUMP_PREFIX_OFFSET,
			16, 4, data, len, false);
	pr_err("\n");
}

static int do_cmd_write(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	int32_t addr;
	int32_t len = (args->argc - 2) * sizeof(int32_t);
	int32_t *data;
	int i;

	if (args->argc < 3 || args->argv[1].type != AUTO_ARG_TYPE_INT32) {
		dev_err(pdata->dev, "Mis or unknown args!");
		return -1;
	}

	len = MIN(len, APB_MAX_OP_BYTES);

	addr = args->argv[1].m_int32;
	data = (int32_t *)kmalloc(len, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	for (i = 0; i < len / 4; i++) {
		if (args->argv[i + 2].type != AUTO_ARG_TYPE_INT32) {
			dev_err(pdata->dev, "Unknown args!");
			kfree(data);
			return -1;
		}

		data[i] = args->argv[i + 2].m_int32;
	}

	spi2apb_write(pdata->spi, addr, data, len);

	kfree(data);

	dev_info(pdata->dev, "write addr: 0x%x, len: %d bytes", addr, len);
	return 0;
}

static int do_cmd_read(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	int32_t addr;
	int32_t len;
	int32_t *data;

	if (args->argc < 3 || args->argv[1].type != AUTO_ARG_TYPE_INT32) {
		dev_err(pdata->dev, "Mis or unknown args!");
		return -1;
	}

	addr = args->argv[1].m_int32;
	if (args->argc == 2) {
		len = 32;
	} else {
		if (args->argv[2].type != AUTO_ARG_TYPE_INT32) {
			dev_err(pdata->dev, "Unknown args!");
			return -1;
		}
		len = args->argv[2].m_int32 * sizeof(int32_t);
		len = MIN(len, APB_MAX_OP_BYTES);
	}

	data = (int32_t *)kmalloc(len, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev_info(pdata->dev, "\nread addr: %x, len: %d bytes", addr, len);
	spi2apb_read(pdata->spi, addr, data, len);
	int32_hexdump("read data:", data, len);
	kfree(data);

	return 0;
}

static int do_cmd_set_spi_rate(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	if (args->argc < 2 || args->argv[1].type != AUTO_ARG_TYPE_INT32) {
		dev_err(pdata->dev, "Mis or unknown args!");
		return -1;
	}

	pdata->max_speed_hz = args->argv[1].m_int32;
	dev_info(pdata->dev, "set spi max speed to %d!", pdata->max_speed_hz);

	if (args->argc == 3 && args->argv[2].type == AUTO_ARG_TYPE_INT32) {
		pdata->min_speed_hz = args->argv[2].m_int32;
		dev_info(pdata->dev, "set spi min speed to %d!", pdata->min_speed_hz);
	}

	return 0;
}

static int do_cmd_query(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	int32_t state;

	spi2apb_operation_query(pdata->spi, &state);
	dev_info(pdata->dev, "state %x", state);
	return 0;
}

static int do_cmd_download_fw(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	int ret = 0;
	const char *fw_name = NULL;

	if (args->argc == 2 && args->argv[1].type == AUTO_ARG_TYPE_STR)
		fw_name = args->argv[1].m_str;

	ret = preisp_spi_download_fw(pdata->spi, fw_name);
	if (ret)
		dev_err(pdata->dev, "download firmware failed!");
	else
		dev_info(pdata->dev, "download firmware success!");
	return 0;
}

static int do_cmd_fast_write(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	int ret = 0;
	int32_t reg;

	if (args->argc != 2 || args->argv[1].type != AUTO_ARG_TYPE_INT32) {
		dev_err(pdata->dev, "Mis or unknown args!");
		return -1;
	}

	reg = args->argv[1].m_int32;

	ret = spi2apb_interrupt_request(pdata->spi, reg);
	dev_info(pdata->dev, "interrupt request reg1:%x ret:%x", reg, ret);

	return 0;
}

static int do_cmd_fast_read(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	int32_t state;

	spi2apb_state_query(pdata->spi, &state);
	dev_info(pdata->dev, "dsp state %x", state);

	return 0;
}

static int do_cmd_queue_init(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	int state = 0;

	state = dsp_msq_init(pdata->spi);
	dev_info(pdata->dev, "message queue init state: %d", state);

	return 0;
}

static int do_cmd_send_msg(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	struct msg *m;
	int ret = 0;
	int msg_len;
	unsigned int i = 0;

	if (args->argc < 2) {
		dev_err(pdata->dev, "need more args");
		return -1;
	}

	msg_len = args->argc * sizeof(uint32_t);

	m = (struct msg *)kmalloc(msg_len, GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->size = msg_len / 4;
	for (i = 1; i < m->size; i++) {
		if (args->argv[i].type != AUTO_ARG_TYPE_INT32) {
			dev_err(pdata->dev, "Unknown args!");
			kfree(m);
			return -1;
		}

		*((int32_t *)m + i) = args->argv[i].m_int32;
	}

	ret = preisp_send_msg_to_dsp(pdata, m);

	dev_info(pdata->dev, "send msg len: %d, ret: %x",
			m->size, ret);

	kfree(m);

	return 0;
}

static int do_cmd_recv_msg(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	struct msg *m;
	char buf[256] = "";
	int ret = 0;

	ret = dsp_msq_recv_msg(pdata->spi, &m);
	if (ret || !m)
		return 0;

	dev_info(pdata->dev, "\nrecv msg len: %d, ret: %x",
			m->size, ret);
	int32_hexdump("recv msg:", (int32_t *)m, m->size * 4);

	dev_info(pdata->dev, buf);

	dsp_msq_free_received_msg(m);

	return 0;
}

static int do_cmd_power_on(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	int ret;

	ret = rkpreisp_power_on(pdata);
	dev_info(pdata->dev, "do cmd power on, count++");
	return ret;
}

static int do_cmd_power_off(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	int ret;

	ret = rkpreisp_power_off(pdata);
	dev_info(pdata->dev, "do cmd power off, count--");
	return ret;
}

static int do_cmd_set_dsp_log_level(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	int ret;

	if (args->argc != 2 || args->argv[1].type != AUTO_ARG_TYPE_INT32) {
		dev_err(pdata->dev, "Mis or unknown args!");
		return -1;
	}

	pdata->log_level = args->argv[1].m_int32;
	ret = rkpreisp_set_log_level(pdata, pdata->log_level);

	dev_info(pdata->dev, "set dsp log level %d, ret: %d",
			pdata->log_level, ret);

	return ret;
}

static int do_cmd_request_bypass(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	atomic_set(&pdata->wake_sleep_cnt, 1);
	return rkpreisp_request_sleep(pdata, PREISP_SLEEP_MODE_BYPASS);
}

static int do_cmd_request_sleep(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	atomic_set(&pdata->wake_sleep_cnt, 1);
	return rkpreisp_request_sleep(pdata, PREISP_SLEEP_MODE_STANDBY);
}

static int do_cmd_version(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	dev_info(pdata->dev, "driver version: %s", RKPREISP_VERSION);
	return 0;
}

static int do_cmd_wakeup(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args)
{
	return rkpreisp_wakeup(pdata);
}

static int do_cmd_set_regu_vol(struct spi_rk_preisp_data *pdata,
		const struct auto_args *args, struct regulator *regu)
{
	int ret = 0;
	int uV;

	if (IS_ERR(regu))
		return -1;

	if (args->argc == 1) {
		unsigned int step = regulator_get_linear_step(regu);

		uV = regulator_get_voltage(regu);
		dev_info(pdata->dev, "%s get voltage:%duV, step:%duV!\n",
				args->argv[0].m_str, uV, step);
		return 0;
	} else if (args->argc != 2 || args->argv[1].type != AUTO_ARG_TYPE_INT32) {
		dev_err(pdata->dev, "Mis or unknown args!");
		return -1;
	}

	uV = args->argv[1].m_int32;
	ret = preisp_set_regu_vol(pdata->vdd_core_regu, uV);
	dev_info(pdata->dev, "%s set voltage %d (ret=%d), cur voltage:%duV!\n",
			args->argv[0].m_str, uV, ret, regulator_get_voltage(regu));

	return ret;
}

static int do_cmd_set_dsi_stream(struct spi_rk_preisp_data *pdata,
		 const struct auto_args *args)
{
	int ret = 0;
	int on = 0;

	dev_info(pdata->dev, "set dsi stream %d, ret: %d", on, ret);
	return ret;
}

static int do_cmd_help(struct spi_rk_preisp_data *pdata)
{
	dev_info(pdata->dev, "\n"
		"support debug commands:\n"
		"bypass	   -- request bypass sleep.\n"
		"sleep		-- request standby sleep.\n"
		"f [fw_name]  -- download fw.\n"
		"log level	-- set preisp log level.\n"
		"on		   -- power count + 1.\n"
		"off		  -- power count - 1.\n"
		"q			-- query operation status.\n"
		"r addr [length=32] -- read addr\n"
		"w addr value,...   -- write addr\n"
		"core_vdd [value]   -- get/set core_vdd voltage\n"
		"pd_vdd [value]	 -- get/set pd_vdd voltage\n\n");
	return 0;
}

static int do_cmd(struct spi_rk_preisp_data *pdata,
	const struct auto_args *args)
{
	const char *s;

	if (args->argv->type != AUTO_ARG_TYPE_STR)
		return 0;

	s = args->argv->m_str;
	if (!strcmp(s, "bypass"))
		return do_cmd_request_bypass(pdata, args);
	if (!strcmp(s, "c"))
		return do_cmd_recv_msg(pdata, args);
	if (!strcmp(s, "f"))
		return do_cmd_download_fw(pdata, args);
	if (!strcmp(s, "fw"))
		return do_cmd_fast_write(pdata, args);
	if (!strcmp(s, "fr"))
		return do_cmd_fast_read(pdata, args);
	if (!strcmp(s, "i"))
		return do_cmd_queue_init(pdata, args);
	if (!strcmp(s, "log"))
		return do_cmd_set_dsp_log_level(pdata, args);
	if (!strcmp(s, "on"))
		return do_cmd_power_on(pdata, args);
	if (!strcmp(s, "off"))
		return do_cmd_power_off(pdata, args);
	if (!strcmp(s, "q"))
		return do_cmd_query(pdata, args);
	if (!strcmp(s, "r"))
		return do_cmd_read(pdata, args);
	if (!strcmp(s, "rate"))
		return do_cmd_set_spi_rate(pdata, args);
	if (!strcmp(s, "s"))
		return do_cmd_send_msg(pdata, args);
	if (!strcmp(s, "sleep"))
		return do_cmd_request_sleep(pdata, args);
	if (!strcmp(s, "v"))
		return do_cmd_version(pdata, args);
	if (!strcmp(s, "w"))
		return do_cmd_write(pdata, args);
	if (!strcmp(s, "wake"))
		return do_cmd_wakeup(pdata, args);
	if (!strcmp(s, "core_vdd"))
		return do_cmd_set_regu_vol(pdata, args, pdata->vdd_core_regu);
	if (!strcmp(s, "pd_vdd"))
		return do_cmd_set_regu_vol(pdata, args, pdata->pdvdd_regu);
	if (!strcmp(s, "dsi"))
		return do_cmd_set_dsi_stream(pdata, args);
	dev_err(pdata->dev, "unknown commands:%s", s);
	do_cmd_help(pdata);

	return 0;
}

static int rkpreisp_open(struct inode *inode, struct file *file)
{
	struct spi_rk_preisp_data *pdata =
		container_of(file->private_data, struct spi_rk_preisp_data, misc);

	preisp_client *client = preisp_client_new();

	client->private_data = pdata;
	file->private_data = client;
	rkpreisp_power_on(pdata);
	return 0;
}

static int rkpreisp_release(struct inode *inode, struct file *file)
{
	preisp_client *client = file->private_data;
	struct spi_rk_preisp_data *pdata = client->private_data;

	preisp_client_disconnect(pdata, client);
	preisp_client_release(client);
	rkpreisp_power_off(pdata);
	return 0;
}

static ssize_t rkpreisp_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char *buf;
	struct auto_args args;
	int i;
	preisp_client *client = file->private_data;
	struct spi_rk_preisp_data *pdata = client->private_data;

	buf = (char *)kmalloc(count + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;
	buf[count] = 0;

	i = 0;
	while (buf[i] != 0) {
		int ret = parse_auto_args(buf + i, &args);

		if (ret < 0)
			break;

		i += ret;
		if (args.argc == 0)
			continue;

		do_cmd(pdata, &args);
		free_auto_args(&args);
	}

	kfree(buf);

	return count;
}

static void print_dsp_log(struct spi_rk_preisp_data *pdata, msg_dsp_log_t *log)
{
	char *str = (char *)(log);

	str[log->size * sizeof(int32_t) - 1] = 0;
	str += sizeof(msg_dsp_log_t);

	dev_info(pdata->dev, "DSP%d: %s", log->core_id, str);
}

static void dispatch_received_msg(struct spi_rk_preisp_data *pdata,
		struct msg *msg)
{
	preisp_client *client;

#if DEBUG_DUMP_ALL_SEND_RECV_MSG == 1
	int32_hexdump("recv msg:", (int32_t *)msg, msg->size * 4);
#endif

	if (msg->type == id_msg_dsp_log_t) {
		print_dsp_log(pdata, (msg_dsp_log_t *)msg);
	} else if (msg->type == id_msg_do_i2c_t) {
		preisp_ap_i2c_do_xfer(pdata->spi, (msg_do_i2c_t *)msg);
	} else {
		mutex_lock(&pdata->clients.mutex);
		list_for_each_entry(client, &pdata->clients.list, list) {
			if (client->id == msg->camera_id) {
				msq_send_msg(&client->q, msg);
				wake_up_interruptible(&client->wait);
			}
		}
		mutex_unlock(&pdata->clients.mutex);
	}
}

static long rkpreisp_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	void __user *ubuf = (void __user *)arg;
	preisp_client *client = file->private_data;
	struct spi_rk_preisp_data *pdata = client->private_data;

	switch (cmd) {
	case PREISP_POWER_ON:
		ret = rkpreisp_power_on(pdata);
		break;
	case PREISP_POWER_OFF:
		ret = rkpreisp_power_off(pdata);
		break;
	case PREISP_REQUEST_SLEEP: {
		int sleep_mode = arg;

		ret = rkpreisp_request_sleep(pdata, sleep_mode);
		break;
	}
	case PREISP_WAKEUP:
		ret = rkpreisp_wakeup(pdata);
		break;
	case PREISP_DOWNLOAD_FW: {
		char fw_name[PREISP_FW_NAME_LEN];

		if (strncpy_from_user(fw_name, ubuf, PREISP_FW_NAME_LEN) <= 0) {
			ret = -EINVAL;
			break;
		}
		dev_info(pdata->dev, "download fw:%s", fw_name);
		ret = preisp_spi_download_fw(pdata->spi, fw_name);
		break;
	}
	case PREISP_RESET_FW: {
		char fw_name[PREISP_FW_NAME_LEN];

		if (strncpy_from_user(fw_name, ubuf, PREISP_FW_NAME_LEN) <= 0) {
			ret = -EINVAL;
			break;
		}
		dev_info(pdata->dev, "reset fw:%s", fw_name);
		ret = rkpreisp_reset(pdata, fw_name);
		break;
	}
	case PREISP_APB_WRITE: {
		struct preisp_apb_pkt pkt;
		int32_t *data;

		if (copy_from_user(&pkt, ubuf, sizeof(pkt))) {
			ret = -EINVAL;
			break;
		}
		pkt.data_len = MIN(pkt.data_len, APB_MAX_OP_BYTES);
		data = kmalloc(pkt.data_len, GFP_KERNEL);
		if (!data) {
			ret = -ENOMEM;
			break;
		}

		if (copy_from_user(data, (void __user *)pkt.un.data, pkt.data_len)) {
			kfree(data);
			ret = -EINVAL;
			break;
		}
		ret = spi2apb_safe_write(pdata->spi, pkt.addr, data, pkt.data_len);
		kfree(data);
		break;
	}
	case PREISP_APB_READ: {
		struct preisp_apb_pkt pkt;
		int32_t *data;

		if (copy_from_user(&pkt, ubuf, sizeof(pkt))) {
			ret = -EINVAL;
			break;
		}
		pkt.data_len = MIN(pkt.data_len, APB_MAX_OP_BYTES);
		data = kmalloc(pkt.data_len, GFP_KERNEL);
		if (!data) {
			ret = -ENOMEM;
			break;
		}

		ret = spi2apb_safe_read(pdata->spi, pkt.addr, data, pkt.data_len);
		if (ret) {
			pr_err("spi2apb_safe_read fail, ret=%d\n", ret);
			kfree(data);
			break;
		}
		ret = copy_to_user((void __user *)pkt.un.data, data, pkt.data_len);
		if (ret)
			pr_err("copy_to_user fail, uncopy left %d bytes\n", ret);

		kfree(data);
		break;
	}
	case PREISP_ST_QUERY: {
		int32_t state;

		ret = spi2apb_state_query(pdata->spi, &state);
		if (ret)
			break;

		ret = put_user(state, (int32_t __user *)ubuf);
		break;
	}
	case PREISP_IRQ_REQUEST: {
		int int_num = arg;

		ret = spi2apb_interrupt_request(pdata->spi, int_num);
		break;
	}
	case PREISP_SEND_MSG: {
		struct msg *msg;
		uint32_t len;

		if (get_user(len, (uint32_t __user *)ubuf)) {
			ret = -EINVAL;
			break;
		}
		len = len * sizeof(int32_t);
		msg = kmalloc(len, GFP_KERNEL);
		if (!msg) {
			ret = -ENOMEM;
			break;
		}

		if (copy_from_user(msg, ubuf, len)) {
			kfree(msg);
			ret = -EINVAL;
			break;
		}
#if DEBUG_DUMP_ALL_SEND_RECV_MSG == 1
		int32_hexdump("send msg:", (int32_t *)msg, len);
#endif

#if DEBUG_MSG_LOOP_TEST == 0
		ret = preisp_send_msg_to_dsp(pdata, msg);
#else
		dispatch_received_msg(pdata, msg);
#endif
		kfree(msg);
		break;
	}
	case PREISP_QUERY_MSG: {
		struct msg *msg;

		ret = msq_recv_msg(&client->q, &msg);
		if (ret)
			break;

		ret = put_user(msg->size, (uint32_t __user *)ubuf);
		break;
	}
	case PREISP_RECV_MSG: {
		struct msg *msg;

		ret = msq_recv_msg(&client->q, &msg);
		if (ret)
			break;
		ret = copy_to_user(ubuf, msg, msg->size * sizeof(uint32_t));
		msq_free_received_msg(&client->q, msg);
		break;
	}
	case PREISP_CLIENT_CONNECT:
	{
		int id = arg;

		client->id = id;
		ret = preisp_client_connect(pdata, client);
		break;
	}
	case PREISP_CLIENT_DISCONNECT:
	{
		preisp_client_disconnect(pdata, client);
		client->id = INVALID_ID;
		break;
	}
	case PREISP_CORE_VDD_SET:
	{
		int uV = arg;

		ret = preisp_set_regu_vol(pdata->vdd_core_regu, uV);
		break;
	}
	case PREISP_CORE_VDD_GET:
	{
		int32_t uV = PREISP_VDD_CORE_VOL_DEFAULT;

		if (!IS_ERR(pdata->vdd_core_regu))
			uV = regulator_get_voltage(pdata->vdd_core_regu);
		ret = put_user(uV, (int32_t __user *)ubuf);
		break;
	}
	case PREISP_AP_DSI_STREAM:
	{
		uint32_t dsi_stream;

		dsi_stream = arg;

		dev_err(pdata->dev, "dsi_stream = %d\n", dsi_stream);
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static unsigned int rkpreisp_poll(struct file *file, poll_table *wait)
{
	preisp_client *client = file->private_data;
	unsigned int mask = 0;

	poll_wait(file, &client->wait, wait);

	if (!msq_is_empty(&client->q))
		mask |= POLLIN;

	return mask;
}

static const struct file_operations rkpreisp_fops = {
	.owner = THIS_MODULE,
	.open = rkpreisp_open,
	.release = rkpreisp_release,
	.write = rkpreisp_write,
	.poll = rkpreisp_poll,
	.unlocked_ioctl = rkpreisp_ioctl,
	.compat_ioctl = rkpreisp_ioctl,
};

static irqreturn_t rkpreisp_threaded_isr(int irq, void *dev_id)
{
	struct spi_rk_preisp_data *pdata = dev_id;
	struct msg *msg;

	BUG_ON(irq != pdata->irq);

	mutex_lock(&pdata->power_lock);
	if (atomic_read(&pdata->power_on_cnt) > 0 &&
		atomic_read(&pdata->wake_sleep_cnt) > 0) {
		while (!dsp_msq_recv_msg(pdata->spi, &msg) && msg) {
			dispatch_received_msg(pdata, msg);
			dsp_msq_free_received_msg(msg);
		}
	}
	mutex_unlock(&pdata->power_lock);

	return IRQ_HANDLED;
}

static irqreturn_t rkpreisp_sleep_threaded_isr(int irq, void *dev_id)
{
	struct spi_rk_preisp_data *pdata = dev_id;

	BUG_ON(irq != pdata->sleepst_irq);
	pr_info("dsp enter sleep done!");

	/* disable vdd-core-regulator */
	if (!IS_ERR(pdata->vdd_core_regu) && pdata->sleep_mode == PREISP_SLEEP_MODE_STANDBY) {
		if (regulator_is_enabled(pdata->vdd_core_regu) > 0)
			regulator_disable(pdata->vdd_core_regu);
	}

	pdata->sleep_state_flag = 1;

	return IRQ_HANDLED;
}

static int rkpreisp_parse_dt_property(struct device *dev,
				  struct spi_rk_preisp_data *pdata)
{
	int ret = 0;
	struct device_node *node = dev->of_node;
	enum of_gpio_flags flags;
	const char *regu_name;
	int uV;

	if (!node)
		return -ENODEV;

	of_property_read_u32(node, "spi-max-frequency",
			&pdata->max_speed_hz);

	ret = of_property_read_u32(node, "spi-min-frequency",
			&pdata->min_speed_hz);
	if (ret) {
		dev_warn(dev, "can not get spi-min-frequency!");
		pdata->min_speed_hz = pdata->max_speed_hz / 2;
	}

	pdata->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(pdata->mclk)) {
		dev_warn(dev, "can not get mclk, error %ld\n", PTR_ERR(pdata->mclk));
		pdata->mclk = NULL;
	}

	ret = of_get_named_gpio_flags(node, "reset-gpio", 0, &flags);
	if (ret <= 0)
		dev_warn(dev, "can not find property reset-gpio, error %d\n", ret);

	pdata->reset_gpio = ret;
	pdata->reset_active = 1;
	if (flags == OF_GPIO_ACTIVE_LOW)
		pdata->reset_active = 0;

	if (pdata->reset_gpio > 0) {
		ret = devm_gpio_request(dev, pdata->reset_gpio, "preisp-reset");
		if (ret) {
			dev_err(dev, "gpio %d request error %d\n", pdata->reset_gpio, ret);
			return ret;
		}

		ret = gpio_direction_output(pdata->reset_gpio, !pdata->reset_active);
		if (ret) {
			dev_err(dev, "gpio %d direction output error %d\n",
					pdata->reset_gpio, ret);
			return ret;
		}
	}

	ret = of_get_named_gpio_flags(node, "irq-gpio", 0, NULL);
	if (ret <= 0) {
		dev_warn(dev, "can not find property irq-gpio, error %d\n", ret);
		return ret;
	}

	pdata->irq_gpio = ret;

	ret = devm_gpio_request(dev, pdata->irq_gpio, "preisp-irq");
	if (ret) {
		dev_err(dev, "gpio %d request error %d\n", pdata->irq_gpio, ret);
		return ret;
	}

	ret = gpio_direction_input(pdata->irq_gpio);
	if (ret) {
		dev_err(dev, "gpio %d direction input error %d\n",
				pdata->irq_gpio, ret);
		return ret;
	}

	ret = gpio_to_irq(pdata->irq_gpio);
	if (ret < 0) {
		dev_err(dev, "Unable to get irq number for GPIO %d, error %d\n",
			pdata->irq_gpio, ret);
		return ret;
	}
	pdata->irq = ret;
	ret = request_threaded_irq(pdata->irq, NULL, rkpreisp_threaded_isr,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "preisp-irq", pdata);
	if (ret) {
		dev_err(dev, "cannot request thread irq: %d\n", ret);
		return ret;
	}

	disable_irq(pdata->irq);

	/* for pdvdd-regulator */
	ret = of_property_read_string(node, "pdvdd-regulator", &regu_name);
	if (!ret) {
		ret = of_property_read_u32(node, "pdvdd-microvolt", &uV);
		if (ret) {
			uV = PREISP_PDVDD_VOL_DEFAULT;
			dev_warn(dev, "cannot get pdvdd-microvolt, set default:%dmV\n", uV);
		}

		pdata->pdvdd_regu = regulator_get(dev, regu_name);
		if (!IS_ERR(pdata->pdvdd_regu)) {
			ret = preisp_set_regu_vol(pdata->pdvdd_regu, uV);
			dev_info(pdata->dev, "pdvdd set voltage %d (ret=%d), cur voltage:%duV!\n",
					uV, ret, regulator_get_voltage(pdata->pdvdd_regu));
			/* disable regulator */
			if (regulator_is_enabled(pdata->pdvdd_regu) > 0)
				regulator_disable(pdata->pdvdd_regu);

		} else {
			dev_warn(dev, "get regulator: %s fail\n", regu_name);
		}
	} else {
		pdata->pdvdd_regu = ERR_PTR(-EPERM);
		dev_warn(dev, "no pdvdd-regulator found\n");
	}

	/* for vdd-core-regulator */
	ret = of_property_read_string(node, "vdd-core-regulator", &regu_name);
	if (!ret) {
		ret = of_property_read_u32(node, "vdd-core-microvolt", &uV);
		if (ret) {
			uV = PREISP_VDD_CORE_VOL_DEFAULT;
			dev_warn(dev, "cannot get vdd-core-microvolt, set default:%dmV\n", uV);
		}

		pdata->vdd_core_regu = regulator_get(dev, regu_name);
		if (!IS_ERR(pdata->vdd_core_regu)) {
			ret = preisp_set_regu_vol(pdata->vdd_core_regu, uV);
			dev_info(pdata->dev, "vdd-core set voltage %d (ret=%d), cur voltage:%duV!\n",
					uV, ret, regulator_get_voltage(pdata->vdd_core_regu));
			/* disable regulator */
			if (regulator_is_enabled(pdata->vdd_core_regu) > 0)
				regulator_disable(pdata->vdd_core_regu);

		} else {
			dev_warn(dev, "get regulator: %s fail\n", regu_name);
		}
	} else {
		pdata->vdd_core_regu = ERR_PTR(-EPERM);
		dev_warn(dev, "no vdd-core-regulator found\n");
	}

	pdata->sleepst_gpio = -1;
	pdata->sleepst_irq = -1;
	pdata->wakeup_gpio = -1;

	ret = of_get_named_gpio_flags(node, "sleepst-gpio", 0, NULL);
	if (ret <= 0) {
		dev_warn(dev, "can not find property sleepst-gpio, error %d\n", ret);
		return 0;
	}

	pdata->sleepst_gpio = ret;

	ret = devm_gpio_request(dev, pdata->sleepst_gpio, "preisp-sleep-irq");
	if (ret) {
		dev_err(dev, "gpio %d request error %d\n", pdata->sleepst_gpio, ret);
		return 0;
	}

	ret = gpio_direction_input(pdata->sleepst_gpio);
	if (ret) {
		dev_err(dev, "gpio %d direction input error %d\n",
				pdata->sleepst_gpio, ret);
		return ret;
	}

	ret = gpio_to_irq(pdata->sleepst_gpio);
	if (ret < 0) {
		dev_err(dev, "Unable to get irq number for GPIO %d, error %d\n",
			pdata->sleepst_gpio, ret);
		return ret;
	}
	pdata->sleepst_irq = ret;
	ret = request_threaded_irq(pdata->sleepst_irq, NULL, rkpreisp_sleep_threaded_isr,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "preisp-sleep-irq", pdata);
	disable_irq(pdata->sleepst_irq);

	ret = of_get_named_gpio_flags(node, "wakeup-gpio", 0, &flags);
	if (ret <= 0) {
		dev_warn(dev, "can not find property wakeup-gpio, error %d\n", ret);
	}

	pdata->wakeup_gpio = ret;
	pdata->wakeup_active = 1;
	if (flags == OF_GPIO_ACTIVE_LOW) {
		pdata->wakeup_active = 0;
	}

	if (pdata->wakeup_gpio > 0) {
		ret = devm_gpio_request(dev, pdata->wakeup_gpio, "preisp-wakeup");
		if (ret) {
			dev_err(dev, "gpio %d request error %d\n", pdata->wakeup_gpio, ret);
			return ret;
		}

		ret = gpio_direction_output(pdata->wakeup_gpio, !pdata->wakeup_active);
		if (ret) {
			dev_err(dev, "gpio %d direction output error %d\n",
					pdata->wakeup_gpio, ret);
			return ret;
		}
	}

	ret = of_property_read_u32(node, "firmware-nowait-mode", &pdata->fw_nowait_mode);
	if (ret) {
		dev_warn(dev, "can not get firmware-nowait-mode!");
		pdata->fw_nowait_mode = 0;
	}

	return ret;
}

static int spi_rk_preisp_probe(struct spi_device *spi)
{
	struct spi_rk_preisp_data *data;
	int err;

	dev_info(&spi->dev, "rk preisp spi probe start, version:%s",
			RKPREISP_VERSION);
	data = devm_kzalloc(&spi->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	atomic_set(&data->power_on_cnt, 0);
	atomic_set(&data->wake_sleep_cnt, 0);
	preisp_client_list_init(&data->clients);
	mutex_init(&data->send_msg_lock);
	mutex_init(&data->power_lock);
	wake_lock_init(&data->resume_wake_lock, WAKE_LOCK_SUSPEND, "Preisp_Wake_Lock");

	data->spi = spi;
	data->dev = &spi->dev;
	rkpreisp_parse_dt_property(data->dev, data);

	spi_set_drvdata(spi, data);
	data->log_level = LOG_INFO;

	data->misc.minor = MISC_DYNAMIC_MINOR;
	data->misc.name = "rk_preisp";
	data->misc.fops = &rkpreisp_fops;

	err = misc_register(&data->misc);
	if (err < 0)
		dev_err(data->dev, "Error: misc_register returned %d", err);

#if ENABLE_CACHE_FIRMWARE
	cache_firmware(RKL_DEFAULT_FW_NAME);
#endif
	if (!data->fw_nowait_mode)
		return 0;

	err = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			RKL_DEFAULT_FW_NAME, data->dev, GFP_KERNEL, data, fw_nowait_power_on);
	if (err)
		dev_err(data->dev, "request firmware nowait failed!");

	return 0;
}

static int spi_rk_preisp_remove(struct spi_device *spi)
{
	struct spi_rk_preisp_data *data = spi_get_drvdata(spi);

#if ENABLE_CACHE_FIRMWARE
	uncache_firmware(RKL_DEFAULT_FW_NAME);
#endif
	spi_set_drvdata(spi, NULL);
	misc_deregister(&data->misc);
	return 0;
}

static int spi_rk_preisp_suspend(struct device *dev, pm_message_t mesg)
{
	struct spi_rk_preisp_data *pdata = dev_get_drvdata(dev);

	if (!pdata->fw_nowait_mode)
		return 0;

	rkpreisp_power_off(pdata);

	return 0;
}

static int spi_rk_preisp_resume(struct device *dev)
{
	struct spi_rk_preisp_data *pdata = dev_get_drvdata(dev);
	int ret = 0;

	if (pdata->wakeup_gpio > 0)
		gpio_direction_output(pdata->wakeup_gpio, !pdata->wakeup_active);

	if (pdata->reset_gpio > 0)
		gpio_direction_output(pdata->reset_gpio, !pdata->reset_active);

	if (!pdata->fw_nowait_mode)
		return 0;

	wake_lock(&pdata->resume_wake_lock);
	pdata->is_resume_processing = true;
	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			RKL_DEFAULT_FW_NAME, pdata->dev, GFP_KERNEL, pdata, fw_nowait_power_on);
	if (ret) {
		dev_err(pdata->dev, "request firmware nowait failed!");
		wake_unlock(&pdata->resume_wake_lock);
	}
	return ret;
}

static const struct of_device_id spi_rk_preisp_dt_match[] = {
	{ .compatible = "rockchip,spi_rk_preisp", },
	{},
};

MODULE_DEVICE_TABLE(of, spi_rk_preisp_dt_match);

static struct spi_driver spi_rk_preisp_driver = {
	.driver = {
		.name   = "spi_rk_preisp",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(spi_rk_preisp_dt_match),
		.suspend = spi_rk_preisp_suspend,
		.resume  = spi_rk_preisp_resume,
	},
	.probe	  = spi_rk_preisp_probe,
	.remove	 = spi_rk_preisp_remove,
};

module_spi_driver(spi_rk_preisp_driver);

MODULE_AUTHOR("Tusson <dusong@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip spi interface for PreIsp");
MODULE_LICENSE("GPL");
