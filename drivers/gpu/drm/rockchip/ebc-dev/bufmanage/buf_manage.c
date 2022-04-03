// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co. Ltd.
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/wait.h>

#include "../ebc_dev.h"
#include "buf_manage.h"
#include "buf_list.h"

struct buf_info_s {
	int buf_total_num;
	unsigned long phy_mem_base;
	char *virt_mem_base;

	struct buf_list_s *buf_list; /* buffer list. */
	int use_buf_is_empty;

	struct buf_list_s *dsp_buf_list; /* dispplay buffer list. */
	int dsp_buf_list_status;
	struct ebc_buf_s *osd_buf;
	struct buf_list_s *osd_buf_list; /* dispplay buffer list. */
	struct mutex osd_buf_lock;
	struct mutex dsp_buf_lock;
	struct mutex ebc_buf_lock;
};

static struct buf_info_s ebc_buf_info;
static DECLARE_WAIT_QUEUE_HEAD(ebc_buf_wq);

int ebc_buf_release(struct ebc_buf_s  *release_buf)
{
	struct ebc_buf_s *temp_buf = release_buf;

	if (temp_buf) {
		if (temp_buf->status == buf_osd) {
			kfree(temp_buf);
		} else {
			temp_buf->status = buf_idle;
			if (1 == ebc_buf_info.use_buf_is_empty) {
				ebc_buf_info.use_buf_is_empty = 0;
				wake_up_interruptible_sync(&ebc_buf_wq);
			}
		}
	}

	return BUF_SUCCESS;
}

static void do_dsp_buf_list(struct ebc_buf_s *dsp_buf)
{
	struct ebc_buf_s *temp_buf;
	int temp_pos;

	if (ebc_buf_info.dsp_buf_list->nb_elt > 0) {
		temp_pos = ebc_buf_info.dsp_buf_list->nb_elt;
		while (temp_pos) {
			temp_pos--;
			temp_buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.dsp_buf_list, temp_pos);
			if (temp_buf->needpic) {
				continue;
			} else {
				buf_list_remove(ebc_buf_info.dsp_buf_list, temp_pos);
				ebc_buf_release(temp_buf);
			}
		}
	}
}

int ebc_add_to_dsp_buf_list(struct ebc_buf_s *dsp_buf)
{
	mutex_lock(&ebc_buf_info.dsp_buf_lock);
	if (ebc_buf_info.dsp_buf_list) {
		do_dsp_buf_list(dsp_buf);

		if (-1 == buf_list_add(ebc_buf_info.dsp_buf_list, (int *)dsp_buf, -1)) {
			ebc_buf_release(dsp_buf);
			mutex_unlock(&ebc_buf_info.dsp_buf_lock);
			return BUF_ERROR;
		}

		if (dsp_buf->status != buf_osd)
			dsp_buf->status = buf_dsp;
	}
	mutex_unlock(&ebc_buf_info.dsp_buf_lock);

	return BUF_SUCCESS;
}

int ebc_add_to_osd_buf_list(struct ebc_buf_s *dsp_buf)
{
	int ret = BUF_SUCCESS;

	mutex_lock(&ebc_buf_info.osd_buf_lock);
	if (ebc_buf_info.osd_buf_list) {
		if (-1 == buf_list_add(ebc_buf_info.osd_buf_list, (int *)dsp_buf, -1)) {
			ebc_buf_release(dsp_buf);
			ret = BUF_ERROR;
		}
	}
	mutex_unlock(&ebc_buf_info.osd_buf_lock);
	return ret;
}

struct ebc_buf_s *ebc_osd_buf_get(void)
{
	struct ebc_buf_s *buf = NULL;

	mutex_lock(&ebc_buf_info.osd_buf_lock);
	if (ebc_buf_info.osd_buf_list && (ebc_buf_info.osd_buf_list->nb_elt > 0)) {
		buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.osd_buf_list, 0);
		buf_list_remove(ebc_buf_info.osd_buf_list, 0);
	}
	mutex_unlock(&ebc_buf_info.osd_buf_lock);

	return buf;
}

int ebc_get_dsp_list_enum_num(void)
{
	return ebc_buf_info.dsp_buf_list->nb_elt;
}

int ebc_get_osd_list_enum_num(void)
{
	return ebc_buf_info.osd_buf_list->nb_elt;
}

struct ebc_buf_s *ebc_find_buf_by_phy_addr(unsigned long phy_addr)
{
	struct ebc_buf_s *temp_buf;
	int temp_pos;

	if (ebc_buf_info.buf_list) {
		temp_pos = 0;
		while (temp_pos < ebc_buf_info.buf_list->nb_elt) {
			temp_buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.buf_list, temp_pos++);
			if (temp_buf && (temp_buf->phy_addr == phy_addr))
				return temp_buf;
		}
	}

	return NULL;
}

struct ebc_buf_s *ebc_dsp_buf_get(void)
{
	struct ebc_buf_s *buf = NULL;

	mutex_lock(&ebc_buf_info.dsp_buf_lock);
	if (ebc_buf_info.dsp_buf_list && (ebc_buf_info.dsp_buf_list->nb_elt > 0)) {
		buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.dsp_buf_list, 0);
		buf_list_remove(ebc_buf_info.dsp_buf_list, 0);
	}
	mutex_unlock(&ebc_buf_info.dsp_buf_lock);

	return buf;
}

struct ebc_buf_s *ebc_empty_osd_buf_get(void)
{
	if (ebc_buf_info.osd_buf)
		return ebc_buf_info.osd_buf;
	return NULL;
}

struct ebc_buf_s *ebc_osd_buf_clone(void)
{
	struct ebc_buf_s *temp_buf;

	temp_buf = kzalloc(sizeof(*temp_buf), GFP_KERNEL);
	if (NULL == temp_buf)
		return NULL;

	temp_buf->virt_addr = ebc_buf_info.osd_buf->virt_addr;
	temp_buf->phy_addr = ebc_buf_info.osd_buf->phy_addr;
	temp_buf->status = buf_osd;

	return temp_buf;
}

struct ebc_buf_s *ebc_empty_buf_get(const char *tid_name)
{
	struct ebc_buf_s *temp_buf = NULL;
	int temp_pos;

	mutex_lock(&ebc_buf_info.ebc_buf_lock);
	while (ebc_buf_info.buf_list) {
		temp_pos = 0;

		while (temp_pos < ebc_buf_info.buf_list->nb_elt) {
			temp_buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.buf_list, temp_pos++);
			if (temp_buf) {
				if (temp_buf->status == buf_idle) {
					temp_buf->status = buf_user;
					memcpy(temp_buf->tid_name, tid_name, TASK_COMM_LEN - 1); //store user thread name
					goto OUT;
				}
				// one tid only can get one buf at one time
				else if ((temp_buf->status == buf_user) && (!strncmp(temp_buf->tid_name, tid_name, TASK_COMM_LEN - 1))) {
					printk("[%s]: one tid only can get one buf at one time\n", tid_name);
					goto OUT;
				}
			}
		}
		ebc_buf_info.use_buf_is_empty = 1;
		wait_event_interruptible(ebc_buf_wq, ebc_buf_info.use_buf_is_empty != 1);
	}

OUT:
	mutex_unlock(&ebc_buf_info.ebc_buf_lock);
	return temp_buf;
}

unsigned long ebc_phy_buf_base_get(void)
{
	return ebc_buf_info.phy_mem_base;
}

char *ebc_virt_buf_base_get(void)
{
	return ebc_buf_info.virt_mem_base;
}

int ebc_buf_state_show(char *buf)
{
	int i;
	int ret = 0;
	struct ebc_buf_s *temp_buf;

	ret += sprintf(buf, "dsp_buf num = %d\n", ebc_buf_info.dsp_buf_list->nb_elt);
	if (ebc_buf_info.buf_list) {
		for (i = 0; i < ebc_buf_info.buf_list->nb_elt; i++) {
			temp_buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.buf_list, i);
			ret += sprintf(buf + ret, "ebc_buf[%d]: s = %d, m = %d, tid = %s\n", i, temp_buf->status, temp_buf->buf_mode, temp_buf->tid_name);
		}
	}

	return ret;
}

int ebc_buf_uninit(void)
{
	struct ebc_buf_s *temp_buf;
	int pos;

	ebc_buf_info.buf_total_num = 0;
	if (ebc_buf_info.buf_list) {
		pos = ebc_buf_info.buf_list->nb_elt - 1;
		while (pos >= 0) {
			temp_buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.buf_list, pos);
			if (temp_buf)
				kfree(temp_buf);
			buf_list_remove(ebc_buf_info.buf_list, pos);
			pos--;
		}
	}

	return BUF_SUCCESS;
}

int ebc_buf_init(unsigned long phy_start, char *mem_start, int men_len, int dest_buf_len, int max_buf_num)
{
	int res;
	int use_len;
	char *temp_addr;
	struct ebc_buf_s *temp_buf;

	if (max_buf_num < 0)
		return BUF_ERROR;

	if (NULL == mem_start)
		return BUF_ERROR;

	mutex_init(&ebc_buf_info.dsp_buf_lock);
	mutex_init(&ebc_buf_info.ebc_buf_lock);
	mutex_init(&ebc_buf_info.osd_buf_lock);

	if (buf_list_init(&ebc_buf_info.buf_list, BUF_LIST_MAX_NUMBER))
		return BUF_ERROR;

	if (buf_list_init(&ebc_buf_info.dsp_buf_list, BUF_LIST_MAX_NUMBER)) {
		res = BUF_ERROR;
		goto dsp_list_err;
	}

	if (buf_list_init(&ebc_buf_info.osd_buf_list, BUF_LIST_MAX_NUMBER)) {
		res = BUF_ERROR;
		goto osd_list_err;
	}

	ebc_buf_info.buf_total_num = 0;
	use_len = 0;

	temp_addr = mem_start;
	ebc_buf_info.virt_mem_base = mem_start;
	ebc_buf_info.phy_mem_base = phy_start;
	use_len += dest_buf_len;
	while (use_len <= men_len) {
		temp_buf = kzalloc(sizeof(*temp_buf), GFP_KERNEL);
		if (NULL == temp_buf) {
			res = BUF_ERROR;
			goto exit;
		}
		temp_buf->virt_addr = temp_addr;
		temp_buf->phy_addr = phy_start;
		temp_buf->len = dest_buf_len;
		temp_buf->status = buf_idle;

		if (-1 == buf_list_add(ebc_buf_info.buf_list, (int *)temp_buf, -1)) {
			res = BUF_ERROR;
			goto exit;
		}
		ebc_buf_info.use_buf_is_empty = 0;

		temp_addr += dest_buf_len;
		phy_start += dest_buf_len;
		use_len += dest_buf_len;

		if (ebc_buf_info.buf_list->nb_elt == max_buf_num)
			break;
	}

	ebc_buf_info.buf_total_num = ebc_buf_info.buf_list->nb_elt;
	if (use_len <= men_len) {
		temp_buf = kzalloc(sizeof(*temp_buf), GFP_KERNEL);
		if (NULL == temp_buf) {
			res = BUF_ERROR;
			goto exit;
		}
		temp_buf->virt_addr = temp_addr;
		temp_buf->phy_addr = phy_start;
		temp_buf->len = dest_buf_len;
		temp_buf->status = buf_osd;
		ebc_buf_info.osd_buf = temp_buf;
	}

	return BUF_SUCCESS;

exit:
	ebc_buf_uninit();
	buf_list_uninit(ebc_buf_info.osd_buf_list);
osd_list_err:
	buf_list_uninit(ebc_buf_info.dsp_buf_list);
dsp_list_err:
	buf_list_uninit(ebc_buf_info.buf_list);

	return res;
}
