/*
 * drivers/cpufreq/cpufreq_interactive.c
 *
 * Copyright (C) 2010 Google, Inc.
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
 * Author: Mike Chan (mike@android.com)
 *
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <asm/cputime.h>

#include "cpufreq_governor.h"

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_interactive.h>

static atomic_t active_count = ATOMIC_INIT(0);

struct cpufreq_interactive_cpuinfo {
	struct timer_list cpu_timer;
	int timer_idlecancel;
	u64 time_in_idle;
	u64 idle_exit_time;
	u64 target_set_time;
	u64 target_set_time_in_idle;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;
	unsigned int target_freq;
	unsigned int floor_freq;
	u64 floor_validate_time;
	u64 hispeed_validate_time;
	struct rw_semaphore enable_sem;
	int governor_enabled;
};

static DEFINE_PER_CPU(struct cpufreq_interactive_cpuinfo, cpuinfo);

/* A realtime thread handles frequency scaling */
static struct task_struct *updown_task;
static cpumask_t updown_cpumask;
static spinlock_t updown_state_lock;

/*
 * Mapping from loads to CPU frequencies to jump to.  When we exceed a
 * certain load we will immediately jump to the corresponding frequency.
 * Default: 85% -> max frequency.
 */
struct hispeed_freq_level {
	unsigned int load;
	unsigned int freq;
};
#define DEFAULT_GO_HISPEED_LOAD 85
static struct hispeed_freq_level *hispeed_freqs;
static int nhispeed_freqs;
static spinlock_t hispeed_freqs_lock;

/*
 * The minimum amount of time to spend at a frequency before we can ramp down.
 */
#define DEFAULT_MIN_SAMPLE_TIME (80 * USEC_PER_MSEC)
static unsigned long min_sample_time;

/*
 * The sample rate of the timer used to increase frequency
 */
#define DEFAULT_TIMER_RATE (20 * USEC_PER_MSEC)
static unsigned long timer_rate;

/*
 * Wait this long before raising speed above hispeed, by default a single
 * timer interval.
 */
#define DEFAULT_ABOVE_HISPEED_DELAY DEFAULT_TIMER_RATE
static unsigned int default_above_hispeed_delay[] = {
	DEFAULT_ABOVE_HISPEED_DELAY };
static spinlock_t above_hispeed_delay_lock;
static unsigned int *above_hispeed_delay = default_above_hispeed_delay;
static int nabove_hispeed_delay = ARRAY_SIZE(default_above_hispeed_delay);

/*
 * Boost pulse to hispeed on touchscreen input.
 */

static int input_boost_val;

struct cpufreq_interactive_inputopen {
	struct work_struct inputopen_work;
	struct input_handle *handle;
};

/*
 * Non-zero means longer-term speed boost active.
 */

static int boost_val;

static int cpufreq_governor_interactive(struct cpufreq_policy *policy,
		unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE
static
#endif
struct cpufreq_governor cpufreq_gov_interactive = {
	.name = "interactive",
	.governor = cpufreq_governor_interactive,
	.max_transition_latency = 10000000,
	.owner = THIS_MODULE,
};

static void rearm_idle_timer(struct cpufreq_interactive_cpuinfo *pcpu)
{
	pcpu->time_in_idle = get_cpu_idle_time(smp_processor_id(),
					       &pcpu->idle_exit_time, 1);
	mod_timer_pinned(&pcpu->cpu_timer,
		jiffies + usecs_to_jiffies(timer_rate));
}

static void arm_idle_timer(struct cpufreq_interactive_cpuinfo *pcpu)
{
	pcpu->timer_idlecancel = 0;
	rearm_idle_timer(pcpu);
}

static void del_idle_timer(struct cpufreq_interactive_cpuinfo *pcpu)
{
	del_timer(&pcpu->cpu_timer);
	pcpu->timer_idlecancel = 0;
}

static unsigned int freq_to_above_hispeed_delay(unsigned int freq)
{
	int i;
	unsigned int ret;
	unsigned long flags;

	spin_lock_irqsave(&above_hispeed_delay_lock, flags);

	for (i = 0; i < nabove_hispeed_delay - 1 &&
			freq >= above_hispeed_delay[i+1]; i += 2)
		;

	ret = above_hispeed_delay[i];
	spin_unlock_irqrestore(&above_hispeed_delay_lock, flags);
	return ret;
}

static unsigned int next_hispeed_freq(struct cpufreq_interactive_cpuinfo *pcpu)
{
	unsigned int ret = pcpu->policy->max;
	unsigned long flags;
	int i;

	BUG_ON(hispeed_freqs == NULL);

	spin_lock_irqsave(&hispeed_freqs_lock, flags);
	for (i = 0; i < nhispeed_freqs; i++) {
		if (hispeed_freqs[i].freq > pcpu->target_freq) {
			ret = hispeed_freqs[i].freq;
			break;
		}
	}
	spin_unlock_irqrestore(&hispeed_freqs_lock, flags);

	return ret;
}

static unsigned int load_to_hispeed_freq(unsigned int load)
{
	unsigned int ret;
	unsigned long flags;
	int i;

	BUG_ON(hispeed_freqs == NULL);

	spin_lock_irqsave(&hispeed_freqs_lock, flags);
	ret = hispeed_freqs[nhispeed_freqs - 1].freq;
	for (i = 1; i < nhispeed_freqs; i++) {
		if (load < hispeed_freqs[i].load) {
			ret = hispeed_freqs[i - 1].freq;
			break;
		}
	}
	spin_unlock_irqrestore(&hispeed_freqs_lock, flags);

	return ret;
}

static void cpufreq_interactive_timer(unsigned long data)
{
	u64 now;
	unsigned int delta_idle;
	unsigned int delta_time;
	int cpu_load;
	int load_since_change;
	int need_wakeup;
	u64 time_in_idle;
	u64 idle_exit_time;
	struct cpufreq_interactive_cpuinfo *pcpu =
		&per_cpu(cpuinfo, data);
	u64 now_idle;
	unsigned int hispeed_freq;
	unsigned int new_freq;
	unsigned int index;
	unsigned long flags;

	if (!down_read_trylock(&pcpu->enable_sem))
		return;
	if (!pcpu->governor_enabled)
		goto exit;

	time_in_idle = pcpu->time_in_idle;
	idle_exit_time = pcpu->idle_exit_time;
	now_idle = get_cpu_idle_time(data, &now, 1);
	delta_idle = (unsigned int)(now_idle - time_in_idle);
	delta_time = (unsigned int)(now - idle_exit_time);

	/*
	 * If timer ran less than 1ms after short-term sample started, retry.
	 */
	if (delta_time < 1000)
		goto rearm;

	if (delta_idle > delta_time)
		cpu_load = 0;
	else
		cpu_load = 100 * (delta_time - delta_idle) / delta_time;

	delta_idle = (unsigned int)(now_idle - pcpu->target_set_time_in_idle);
	delta_time = (unsigned int)(now - pcpu->target_set_time);

	if ((delta_time == 0) || (delta_idle > delta_time))
		load_since_change = 0;
	else
		load_since_change =
			100 * (delta_time - delta_idle) / delta_time;

	/*
	 * Choose greater of short-term load (since last idle timer
	 * started or timer function re-armed itself) or long-term load
	 * (since last frequency change).
	 */
	if (load_since_change > cpu_load)
		cpu_load = load_since_change;

	/*
	 * The first hispeed_freq level has the lowest load.  Only boost if
	 * we excced that value.
	 */
	if (cpu_load >= hispeed_freqs[0].load || boost_val) {
		hispeed_freq = load_to_hispeed_freq(cpu_load);
		if (pcpu->target_freq < hispeed_freq) {
			new_freq = hispeed_freq;
		} else {
			new_freq = next_hispeed_freq(pcpu) * cpu_load / 100;

			if (new_freq < hispeed_freq)
				new_freq = hispeed_freq;
		}
	} else {
		hispeed_freq = next_hispeed_freq(pcpu);
		new_freq = hispeed_freq * cpu_load / 100;
	}

	if (pcpu->target_freq >= hispeed_freqs[0].freq &&
	    new_freq > pcpu->target_freq &&
	    now - pcpu->hispeed_validate_time <
	    freq_to_above_hispeed_delay(pcpu->target_freq)) {
		trace_cpufreq_interactive_notyet(data, cpu_load,
						pcpu->target_freq, new_freq);
		goto rearm;
	}

	pcpu->hispeed_validate_time = now;

	if (cpufreq_frequency_table_target(pcpu->policy, pcpu->freq_table,
					   new_freq, CPUFREQ_RELATION_H,
					   &index)) {
		pr_warn_once("timer %d: cpufreq_frequency_table_target error\n",
			     (int) data);
		goto rearm;
	}

	new_freq = pcpu->freq_table[index].frequency;

	/*
	 * Do not scale below floor_freq unless we have been at or above the
	 * floor frequency for the minimum sample time since last validated.
	 */
	if (new_freq < pcpu->floor_freq) {
		if (now - pcpu->floor_validate_time < min_sample_time) {
			trace_cpufreq_interactive_notyet(data, cpu_load,
							 pcpu->target_freq,
							 new_freq);
			goto rearm;
		}
	}

	spin_lock_irqsave(&updown_state_lock, flags);
	if (pcpu->target_freq != new_freq) {
		trace_cpufreq_interactive_target(data, cpu_load,
						 pcpu->target_freq, new_freq);
		pcpu->target_set_time_in_idle = now_idle;
		pcpu->target_freq = new_freq;
		pcpu->target_set_time = now;
		cpumask_set_cpu(data, &updown_cpumask);
		need_wakeup = 1;
	} else {
		trace_cpufreq_interactive_already(data, cpu_load,
						  pcpu->target_freq, new_freq);
		need_wakeup = 0;
	}
	pcpu->floor_freq = new_freq;
	pcpu->floor_validate_time = now;
	spin_unlock_irqrestore(&updown_state_lock, flags);

	if (need_wakeup)
		wake_up_process(updown_task);
	/*
	 * Already set max speed and don't see a need to change that,
	 * wait until next idle to re-evaluate, don't need timer.
	 */
	if (pcpu->target_freq == pcpu->policy->max)
		goto exit;

rearm:
	if (!timer_pending(&pcpu->cpu_timer)) {
		/*
		 * If already at min, cancel the timer if that CPU goes idle.
		 * We don't need to re-evaluate speed until the next idle exit.
		 */
		if (pcpu->target_freq == pcpu->policy->min)
			pcpu->timer_idlecancel = 1;
		rearm_idle_timer(pcpu);
	}

exit:
	up_read(&pcpu->enable_sem);
	return;
}

static void cpufreq_interactive_idle_start(void)
{
	struct cpufreq_interactive_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());
	int pending;

	if (!down_read_trylock(&pcpu->enable_sem))
		return;
	if (!pcpu->governor_enabled) {
		up_read(&pcpu->enable_sem);
		return;
	}

	pending = timer_pending(&pcpu->cpu_timer);

	if (pcpu->target_freq != pcpu->policy->min) {
#ifdef CONFIG_SMP
		/*
		 * Entering idle while not at lowest speed.  On some
		 * platforms this can hold the other CPU(s) at that speed
		 * even though the CPU is idle. Set a timer to re-evaluate
		 * speed so this idle CPU doesn't hold the other CPUs above
		 * min indefinitely.  This should probably be a quirk of
		 * the CPUFreq driver.
		 */
		if (!pending)
			arm_idle_timer(pcpu);
#endif
	} else {
		/*
		 * If at min speed and entering idle after load has
		 * already been evaluated, and a timer has been set just in
		 * case the CPU suddenly goes busy, cancel that timer.  The
		 * CPU didn't go busy; we'll recheck things upon idle exit.
		 */
		if (pending && pcpu->timer_idlecancel)
			del_idle_timer(pcpu);
	}

	up_read(&pcpu->enable_sem);
}

static void cpufreq_interactive_idle_end(void)
{
	struct cpufreq_interactive_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());

	if (!down_read_trylock(&pcpu->enable_sem))
		return;
	if (!pcpu->governor_enabled) {
		up_read(&pcpu->enable_sem);
		return;
	}

	/* Arm the timer for 1-2 ticks later if not already. */
	if (!timer_pending(&pcpu->cpu_timer))
		arm_idle_timer(pcpu);


	up_read(&pcpu->enable_sem);
}

static int cpufreq_interactive_updown_task(void *data)
{
	unsigned int cpu;
	cpumask_t tmp_mask;
	unsigned long flags;
	struct cpufreq_interactive_cpuinfo *pcpu;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&updown_state_lock, flags);

		if (cpumask_empty(&updown_cpumask)) {
			spin_unlock_irqrestore(&updown_state_lock, flags);
			schedule();

			if (kthread_should_stop())
				break;

			spin_lock_irqsave(&updown_state_lock, flags);
		}

		set_current_state(TASK_RUNNING);
		tmp_mask = updown_cpumask;
		cpumask_clear(&updown_cpumask);
		spin_unlock_irqrestore(&updown_state_lock, flags);

		for_each_cpu(cpu, &tmp_mask) {
			unsigned int j;
			unsigned int max_freq, cur_freq;

			pcpu = &per_cpu(cpuinfo, cpu);
			if (!down_read_trylock(&pcpu->enable_sem))
				continue;
			if (!pcpu->governor_enabled) {
				up_read(&pcpu->enable_sem);
				continue;
			}

			/*
			 * Calculate the max frequency over all affected cpu's
			 * and use that to set the target frequency.  This
			 * handles the case where setting the frequency of one
			 * cpu causes multiple to change.  In that case we
			 * never want to down-clock related cpu's just because
			 * one cpu found itself idle and requested a change.
			 * When up-clocking we want that request to go through
			 * and related cpu's will be dragged along.
			 *
			 * NB: this calculation is racey because target_freq is
			 * set under the updown_state_lock (and not held here)
			 */
			max_freq = 0;
			for_each_cpu(j, pcpu->policy->cpus) {
				struct cpufreq_interactive_cpuinfo *pjcpu =
					&per_cpu(cpuinfo, j);

				if (pjcpu->target_freq > max_freq)
					max_freq = pjcpu->target_freq;
			}

			cur_freq = pcpu->policy->cur;
			if (max_freq == 0 || max_freq == cur_freq) {
				up_read(&pcpu->enable_sem);
				continue;
			}

			/* NB: trace before call as it may block for a while */
			if (max_freq < cur_freq)
				trace_cpufreq_interactive_down(cpu,
						max_freq, cur_freq);
			else
				trace_cpufreq_interactive_up(cpu,
						max_freq, cur_freq);
			__cpufreq_driver_target(pcpu->policy, max_freq,
						CPUFREQ_RELATION_H);

			up_read(&pcpu->enable_sem);
		}
	}

	return 0;
}

static void cpufreq_interactive_boost(void)
{
	int i;
	int anyboost = 0;
	unsigned long flags;
	unsigned int hispeed_freq;
	struct cpufreq_interactive_cpuinfo *pcpu;

	spin_lock_irqsave(&updown_state_lock, flags);

	for_each_online_cpu(i) {
		pcpu = &per_cpu(cpuinfo, i);

		hispeed_freq = next_hispeed_freq(pcpu);
		if (pcpu->target_freq < hispeed_freq) {
			pcpu->target_freq = hispeed_freq;
			cpumask_set_cpu(i, &updown_cpumask);
			pcpu->target_set_time_in_idle =
				get_cpu_idle_time(i, &pcpu->target_set_time, 1);
			pcpu->hispeed_validate_time = pcpu->target_set_time;
			anyboost = 1;
		}

		/*
		 * Set floor freq and (re)start timer for when last
		 * validated.
		 */

		pcpu->floor_freq = hispeed_freq;
		pcpu->floor_validate_time = ktime_to_us(ktime_get());
	}

	spin_unlock_irqrestore(&updown_state_lock, flags);

	if (anyboost)
		wake_up_process(updown_task);
}

/*
 * Pulsed boost on input event raises CPUs to hispeed_freq and lets
 * usual algorithm of min_sample_time  decide when to allow speed
 * to drop.
 */

static void cpufreq_interactive_input_event(struct input_handle *handle,
					    unsigned int type,
					    unsigned int code, int value)
{
	if (input_boost_val && type == EV_SYN && code == SYN_REPORT) {
		trace_cpufreq_interactive_boost("input");
		cpufreq_interactive_boost();
	}
}

static void cpufreq_interactive_input_open(struct work_struct *w)
{
	struct cpufreq_interactive_inputopen *io =
		container_of(w, struct cpufreq_interactive_inputopen,
			     inputopen_work);
	struct input_handle *handle = io->handle;
	int error;

	error = input_open_device(handle);
	if (error) {
		pr_warn("%s: open(%s) failed, error %d\n", __func__,
		    handle->dev->name, error);
		input_unregister_handle(handle);
	} else
		pr_info("%s: monitoring input on %s\n",
		    handle->name, handle->dev->name);
	kfree(io);
}

static int cpufreq_interactive_input_connect(struct input_handler *handler,
					     struct input_dev *dev,
					     const struct input_device_id *id)
{
	struct input_handle *handle;
	struct cpufreq_interactive_inputopen *inputopen;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle) {
		pr_warn("%s: no memory to register %s\n", __func__, dev->name);
		return -ENOMEM;
	}

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq_interactive";

	error = input_register_handle(handle);
	if (error) {
		pr_warn("%s: failed to register %s, error %d\n", __func__,
		    dev->name, error);
		goto err;
	}

	inputopen = kzalloc(sizeof(struct cpufreq_interactive_inputopen),
	    GFP_KERNEL);
	if (!inputopen) {
		pr_warn("%s: failed to setup %s, no space for workq item\n",
		    __func__, dev->name);
		input_unregister_handle(handle);
		goto err;
	}
	INIT_WORK(&inputopen->inputopen_work, cpufreq_interactive_input_open);
	inputopen->handle = handle;
	schedule_work(&inputopen->inputopen_work);
	return 0;
err:
	kfree(handle);
	return error;
}

static void cpufreq_interactive_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpufreq_interactive_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			    BIT_MASK(ABS_MT_POSITION_X) |
			    BIT_MASK(ABS_MT_POSITION_Y) },
	}, /* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(BTN_LEFT)] = BIT_MASK(BTN_LEFT) },
	}, /* pointer (e.g. trackpad, mouse) */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(KEY_ESC)] = BIT_MASK(KEY_ESC) },
	}, /* keyboard */
	{ },
};

static struct input_handler cpufreq_interactive_input_handler = {
	.event          = cpufreq_interactive_input_event,
	.connect        = cpufreq_interactive_input_connect,
	.disconnect     = cpufreq_interactive_input_disconnect,
	.name           = "cpufreq_interactive",
	.id_table       = cpufreq_interactive_ids,
};

static unsigned int *get_tokenized_data(const char *buf, int *num_tokens)
{
	const char *cp;
	int i;
	int ntokens = 1;
	unsigned int *tokenized_data;
	int err = -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	tokenized_data = kmalloc(ntokens * sizeof(unsigned int), GFP_KERNEL);
	if (!tokenized_data) {
		err = -ENOMEM;
		goto err;
	}

	cp = buf;
	i = 0;
	while (i < ntokens) {
		if (sscanf(cp, "%u", &tokenized_data[i++]) != 1)
			goto err_kfree;

		cp = strpbrk(cp, " :");
		if (!cp)
			break;
		cp++;
	}

	if (i != ntokens)
		goto err_kfree;

	*num_tokens = ntokens;
	return tokenized_data;

err_kfree:
	kfree(tokenized_data);
err:
	return ERR_PTR(err);
}

static ssize_t show_above_hispeed_delay(struct kobject *kobj,
					struct attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&above_hispeed_delay_lock, flags);

	for (i = 0; i < nabove_hispeed_delay; i++)
		ret += sprintf(buf + ret, "%u%s", above_hispeed_delay[i],
			       i & 0x1 ? ":" : " ");

	ret += sprintf(buf + ret, "\n");
	spin_unlock_irqrestore(&above_hispeed_delay_lock, flags);
	return ret;
}

static ssize_t store_above_hispeed_delay(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ntokens, i;
	unsigned int *new_above_hispeed_delay = NULL;
	unsigned long flags;

	new_above_hispeed_delay = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(new_above_hispeed_delay))
		return PTR_RET(new_above_hispeed_delay);
	if (ntokens % 2 != 1) {
		kfree(new_above_hispeed_delay);
		return -EINVAL;
	}

	/* Make sure frequencies are in ascending order. */
	for (i = 3; i < ntokens; i += 2) {
		if (new_above_hispeed_delay[i] <=
		    new_above_hispeed_delay[i - 2]) {
			kfree(new_above_hispeed_delay);
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&above_hispeed_delay_lock, flags);
	if (above_hispeed_delay != default_above_hispeed_delay)
		kfree(above_hispeed_delay);
	above_hispeed_delay = new_above_hispeed_delay;
	nabove_hispeed_delay = ntokens;
	spin_unlock_irqrestore(&above_hispeed_delay_lock, flags);
	return count;

}

static struct global_attr above_hispeed_delay_attr =
	__ATTR(above_hispeed_delay, S_IRUGO | S_IWUSR,
		show_above_hispeed_delay, store_above_hispeed_delay);

static ssize_t show_hispeed_freq(struct kobject *kobj,
				 struct attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&hispeed_freqs_lock, flags);
	for (i = 0; i < nhispeed_freqs; i++) {
		ret += sprintf(buf + ret, "%s%u:%u", i > 0 ? " " : "",
				hispeed_freqs[i].freq, hispeed_freqs[i].load);
	}
	ret += sprintf(buf + ret, "\n");
	spin_unlock_irqrestore(&hispeed_freqs_lock, flags);

	return ret;
}

static ssize_t store_hispeed_freq(struct kobject *kobj,
				  struct attribute *attr, const char *buf,
				  size_t count)
{
	int ntokens, i, ret = count;
	unsigned int *tokens;
	unsigned long flags;
	struct hispeed_freq_level *new_hispeed_freqs;

	tokens = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(tokens))
		return PTR_RET(tokens);
	if (ntokens % 2 != 0) {
		ret = -EINVAL;
		goto out;
	}

	new_hispeed_freqs = kzalloc(sizeof(*new_hispeed_freqs) * ntokens / 2,
				    GFP_KERNEL);
	if (!new_hispeed_freqs) {
		ret = -ENOMEM;
		goto out;
	}
	for (i = 0; i < ntokens / 2; i++) {
		new_hispeed_freqs[i].freq = tokens[2 * i];
		new_hispeed_freqs[i].load = tokens[2 * i + 1];
		if (new_hispeed_freqs[i].load > 100) {
			kfree(new_hispeed_freqs);
			ret = -EINVAL;
			goto out;
		}
		if (i > 0 && (new_hispeed_freqs[i].freq <=
			      new_hispeed_freqs[i - 1].freq ||
			      new_hispeed_freqs[i].load <=
			      new_hispeed_freqs[i - 1].load)) {
			kfree(new_hispeed_freqs);
			ret = -EINVAL;
			goto out;
		}
	}

	spin_lock_irqsave(&hispeed_freqs_lock, flags);
	kfree(hispeed_freqs);
	hispeed_freqs = new_hispeed_freqs;
	nhispeed_freqs = ntokens / 2;
	spin_unlock_irqrestore(&hispeed_freqs_lock, flags);
out:
	kfree(tokens);
	return ret;
}

static struct global_attr hispeed_freq_attr = __ATTR(hispeed_freq, 0644,
		show_hispeed_freq, store_hispeed_freq);

static ssize_t show_min_sample_time(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", min_sample_time);
}

static ssize_t store_min_sample_time(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	min_sample_time = val;
	return count;
}

static struct global_attr min_sample_time_attr = __ATTR(min_sample_time, 0644,
		show_min_sample_time, store_min_sample_time);

static ssize_t show_timer_rate(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", timer_rate);
}

static ssize_t store_timer_rate(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	timer_rate = val;
	return count;
}

static struct global_attr timer_rate_attr = __ATTR(timer_rate, 0644,
		show_timer_rate, store_timer_rate);

static ssize_t show_input_boost(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	return sprintf(buf, "%u\n", input_boost_val);
}

static ssize_t store_input_boost(struct kobject *kobj, struct attribute *attr,
				 const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	input_boost_val = val;
	return count;
}

define_one_global_rw(input_boost);

static ssize_t show_boost(struct kobject *kobj, struct attribute *attr,
			  char *buf)
{
	return sprintf(buf, "%d\n", boost_val);
}

static ssize_t store_boost(struct kobject *kobj, struct attribute *attr,
			   const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	boost_val = val;

	if (boost_val) {
		trace_cpufreq_interactive_boost("on");
		cpufreq_interactive_boost();
	} else {
		trace_cpufreq_interactive_unboost("off");
	}

	return count;
}

define_one_global_rw(boost);

static ssize_t store_boostpulse(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	trace_cpufreq_interactive_boost("pulse");
	cpufreq_interactive_boost();
	return count;
}

static struct global_attr boostpulse =
	__ATTR(boostpulse, 0200, NULL, store_boostpulse);

static struct attribute *interactive_attributes[] = {
	&above_hispeed_delay_attr.attr,
	&hispeed_freq_attr.attr,
	&min_sample_time_attr.attr,
	&timer_rate_attr.attr,
	&input_boost.attr,
	&boost.attr,
	&boostpulse.attr,
	NULL,
};

static struct attribute_group interactive_attr_group = {
	.attrs = interactive_attributes,
	.name = "interactive",
};

static int cpufreq_interactive_idle_notifier(struct notifier_block *nb,
					     unsigned long val,
					     void *data)
{
	switch (val) {
	case IDLE_START:
		cpufreq_interactive_idle_start();
		break;
	case IDLE_END:
		cpufreq_interactive_idle_end();
		break;
	}

	return 0;
}

static struct notifier_block cpufreq_interactive_idle_nb = {
	.notifier_call = cpufreq_interactive_idle_notifier,
};

static int cpufreq_governor_interactive(struct cpufreq_policy *policy,
		unsigned int event)
{
	int rc;
	unsigned int j;
	struct cpufreq_interactive_cpuinfo *pcpu;
	struct cpufreq_frequency_table *freq_table;

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(policy->cpu))
			return -EINVAL;

		freq_table =
			cpufreq_frequency_get_table(policy->cpu);

		if (!hispeed_freqs) {
			hispeed_freqs = kzalloc(sizeof(*hispeed_freqs),
						GFP_KERNEL);
			if (!hispeed_freqs)
				return -ENOMEM;
			nhispeed_freqs = 1;
			hispeed_freqs[0].load = DEFAULT_GO_HISPEED_LOAD;
			hispeed_freqs[0].freq = policy->max;
		}

		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			pcpu->policy = policy;
			pcpu->target_freq = policy->cur;
			pcpu->freq_table = freq_table;
			pcpu->target_set_time_in_idle =
				get_cpu_idle_time(j, &pcpu->target_set_time, 1);
			pcpu->floor_freq = pcpu->target_freq;
			pcpu->floor_validate_time =
				pcpu->target_set_time;
			pcpu->hispeed_validate_time =
				pcpu->target_set_time;
			down_write(&pcpu->enable_sem);
			del_timer_sync(&pcpu->cpu_timer);
			pcpu->cpu_timer.expires =
				jiffies + usecs_to_jiffies(timer_rate);
			add_timer_on(&pcpu->cpu_timer, j);
			pcpu->governor_enabled = 1;
			up_write(&pcpu->enable_sem);
		}

		/*
		 * Do not register the idle hook and create sysfs
		 * entries if we have already done so.
		 */
		if (atomic_inc_return(&active_count) > 1)
			return 0;

		rc = sysfs_create_group(cpufreq_global_kobject,
				&interactive_attr_group);
		if (rc)
			return rc;

		rc = input_register_handler(&cpufreq_interactive_input_handler);
		if (rc)
			pr_warn("%s: failed to register input handler\n",
				__func__);

		idle_notifier_register(&cpufreq_interactive_idle_nb);
		break;

	case CPUFREQ_GOV_STOP:
		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			down_write(&pcpu->enable_sem);
			pcpu->governor_enabled = 0;
			del_timer_sync(&pcpu->cpu_timer);
			up_write(&pcpu->enable_sem);
		}

		if (atomic_dec_return(&active_count) > 0)
			return 0;

		idle_notifier_unregister(&cpufreq_interactive_idle_nb);
		input_unregister_handler(&cpufreq_interactive_input_handler);
		sysfs_remove_group(cpufreq_global_kobject,
				&interactive_attr_group);

		break;

	case CPUFREQ_GOV_LIMITS:
		if (policy->max < policy->cur)
			__cpufreq_driver_target(policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > policy->cur)
			__cpufreq_driver_target(policy,
					policy->min, CPUFREQ_RELATION_L);
		break;
	}
	return 0;
}

static int __init cpufreq_interactive_init(void)
{
	unsigned int i;
	struct cpufreq_interactive_cpuinfo *pcpu;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };

	min_sample_time = DEFAULT_MIN_SAMPLE_TIME;
	timer_rate = DEFAULT_TIMER_RATE;

	/* Initalize per-cpu timers */
	for_each_possible_cpu(i) {
		pcpu = &per_cpu(cpuinfo, i);
		init_timer(&pcpu->cpu_timer);
		pcpu->cpu_timer.function = cpufreq_interactive_timer;
		pcpu->cpu_timer.data = i;
		init_rwsem(&pcpu->enable_sem);
	}

	spin_lock_init(&hispeed_freqs_lock);
	spin_lock_init(&above_hispeed_delay_lock);
	spin_lock_init(&updown_state_lock);

	updown_task = kthread_create(cpufreq_interactive_updown_task, NULL,
				 "kinteractive");
	if (IS_ERR(updown_task))
		return PTR_ERR(updown_task);

	sched_setscheduler_nocheck(updown_task, SCHED_FIFO, &param);
	get_task_struct(updown_task);

	/* NB: wake up so the thread does not look hung to the freezer */
	wake_up_process(updown_task);

	return cpufreq_register_governor(&cpufreq_gov_interactive);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_INTERACTIVE
fs_initcall(cpufreq_interactive_init);
#else
module_init(cpufreq_interactive_init);
#endif

static void __exit cpufreq_interactive_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_interactive);
	kthread_stop(updown_task);
	put_task_struct(updown_task);
	if (above_hispeed_delay != default_above_hispeed_delay)
		kfree(above_hispeed_delay);
	kfree(hispeed_freqs);
	/* TODO(sleffler) cancel inputopen wq request? */
}

module_exit(cpufreq_interactive_exit);

MODULE_AUTHOR("Mike Chan <mike@android.com>");
MODULE_DESCRIPTION("'cpufreq_interactive' - A cpufreq governor for "
	"Latency sensitive workloads");
MODULE_LICENSE("GPL");
