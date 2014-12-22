/*
 * Copyright (C) 2014 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/mutex.h>
#include <soc/rockchip/dmc-sync.h>

static RAW_NOTIFIER_HEAD(sync_chain);
static RAW_NOTIFIER_HEAD(en_chain);
static DEFINE_MUTEX(sync_lock);
static DEFINE_MUTEX(en_lock);
static int num_wait;
static bool enable;

/**
 * rockchip_dmc_lock - Lock the sync notifiers and call sync notifiers with
 * SYNC_LOCK action.
 */
void rockchip_dmc_lock(void)
{
	mutex_lock(&sync_lock);
}
EXPORT_SYMBOL_GPL(rockchip_dmc_lock);

/**
 * rockchip_dmc_wait - Call sync notifiers with SYNC_WAIT action.
 */
void rockchip_dmc_wait(void)
{
	raw_notifier_call_chain(&sync_chain, 0, NULL);
}
EXPORT_SYMBOL_GPL(rockchip_dmc_wait);

/**
 * rockchip_dmc_unlock - Unlock the sync notifiers and call sync notifiers with
 * SYNC_UNLOCK action.
 */
void rockchip_dmc_unlock(void)
{
	mutex_unlock(&sync_lock);
}
EXPORT_SYMBOL_GPL(rockchip_dmc_unlock);

/**
 * rockchip_dmc_enable - Enable dmc frequency scaling. Will only enable
 * frequency scaling if there are 1 or fewer notifiers that will block on
 * SYNC_WAIT. Call to undo rockchip_dmc_disable.
 */
void rockchip_dmc_enable(void)
{
	mutex_lock(&en_lock);
	if (!enable && num_wait <= 1)
		raw_notifier_call_chain(&en_chain, DMC_ENABLE, NULL);
	enable = true;
	mutex_unlock(&en_lock);
}
EXPORT_SYMBOL_GPL(rockchip_dmc_enable);

/**
 * rockchip_dmc_disable - Disable dmc frequency scaling. Call when something
 * cannot coincide with dmc frequency scaling.
 */
void rockchip_dmc_disable(void)
{
	mutex_lock(&en_lock);
	if (enable && num_wait <= 1)
		raw_notifier_call_chain(&en_chain, DMC_DISABLE, NULL);
	enable = false;
	mutex_unlock(&en_lock);
}
EXPORT_SYMBOL_GPL(rockchip_dmc_disable);

/**
 * rockchip_dmc_inc_wait - Register the notifier block for the sync chain.
 *
 * We can't sync with more than one notifier. By the time we sync with the
 * second notifier, the window of time for the first notifier for which we can
 * change the dmc freq for the first may have passed. So if the number of things
 * waiting during rockchip_dmc_wait is greater than one, call the enable call
 * chain with the disable message (this will likely disable ddr freq).
 * @nb The sync notifier block to register
 */
int rockchip_dmc_get(struct notifier_block *nb)
{
	int ret;

	if (!nb)
		return -EINVAL;

	mutex_lock(&en_lock);
	/* This may call rockchip_dmc_lock/wait/unlock. */
	if (num_wait == 1 && enable)
		raw_notifier_call_chain(&en_chain, DMC_DISABLE, NULL);

	mutex_lock(&sync_lock);
	ret = raw_notifier_chain_register(&sync_chain, nb);
	mutex_unlock(&sync_lock);
	/*
	 * We need to call the enable chain before adding the notifier, which is
	 * why errors are handled this way.
	 */
	if (!ret)
		num_wait++;
	else if (num_wait == 1 && enable)
		raw_notifier_call_chain(&en_chain, DMC_ENABLE, NULL);

	mutex_unlock(&en_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rockchip_dmc_get);

/**
 * rockchip_dmc_dev_wait - Remove the notifier block from the sync chain.
 *
 * Call when registered notifier will no longer block. Increments the number of
 * things waiting during rockchip_dmc_wait. If that number is now 1, call the
 * enable call chain (likely enabling ddr freq).
 * @nb The sync notifier block to unregister
 */
int rockchip_dmc_put(struct notifier_block *nb)
{
	int ret;

	if (!nb)
		return -EINVAL;

	mutex_lock(&en_lock);
	mutex_lock(&sync_lock);
	ret = raw_notifier_chain_unregister(&sync_chain, nb);
	if (!ret)
		num_wait--;
	mutex_unlock(&sync_lock);
	/* This may call rockchip_dmc_lock/wait/unlock. */
	if (num_wait == 1 && enable && !ret)
		raw_notifier_call_chain(&en_chain, DMC_ENABLE, NULL);
	mutex_unlock(&en_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rockchip_dmc_put);

/**
 * rockchip_dmc_register_enable_notifier - Add notifier to enable notifiers.
 *
 * Enable notifiers are called when we enable/disable dmc. This can be done
 * through rockchip_dmc_enable/disable or when there is more than one sync
 * notifier that blocks for the SYNC_WAIT message.
 * @nb The notifier to add
 */
int rockchip_dmc_register_enable_notifier(struct notifier_block *nb)
{
	int ret = 0;

	if (!nb)
		return -EINVAL;

	mutex_lock(&en_lock);
	ret = raw_notifier_chain_register(&en_chain, nb);
	mutex_unlock(&en_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(rockchip_dmc_register_enable_notifier);

/**
 * rockchip_dmc_unregister_enable_notifier - Remove notifier from enable
 * notifiers.
 * @nb The notifier to remove.
 */
int rockchip_dmc_unregister_enable_notifier(struct notifier_block *nb)
{
	int ret = 0;

	if (!nb)
		return -EINVAL;

	mutex_lock(&en_lock);
	ret = raw_notifier_chain_unregister(&en_chain, nb);
	mutex_unlock(&en_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(rockchip_dmc_unregister_enable_notifier);
