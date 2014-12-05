/*
 * drivers/soc/tegra/pmc.c
 *
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/reset.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <dt-bindings/soc/tegra-pmc.h>

#include <soc/tegra/common.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/pmc.h>
#include <asm/system_misc.h>

#define PMC_CNTRL			0x0
#define  PMC_CNTRL_LATCH_WAKEUPS	(1 << 5)
#define  PMC_CNTRL_PWRREQ_POLARITY	(1 << 8)   /* core power req polarity */
#define  PMC_CNTRL_PWRREQ_OE		(1 << 9)   /* core power req enable */
#define  PMC_CNTRL_SYSCLK_POLARITY	(1 << 10)  /* sys clk polarity */
#define  PMC_CNTRL_SYSCLK_OE		(1 << 11)  /* system clock enable */
#define  PMC_CNTRL_SIDE_EFFECT_LP0	(1 << 14)  /* LP0 when CPU pwr gated */
#define  PMC_CNTRL_CPU_PWRREQ_POLARITY	(1 << 15)  /* CPU pwr req polarity */
#define  PMC_CNTRL_CPU_PWRREQ_OE	(1 << 16)  /* CPU pwr req enable */
#define  PMC_CNTRL_INTR_POLARITY	(1 << 17)  /* inverts INTR polarity */

#define PMC_WAKE_MASK			0xc
#define PMC_WAKE_LEVEL			0x10
#define PMC_WAKE_STATUS			0x14
#define PMC_SW_WAKE_STATUS		0x18

#define DPD_SAMPLE			0x020
#define  DPD_SAMPLE_ENABLE		(1 << 0)
#define  DPD_SAMPLE_DISABLE		(0 << 0)

#define PMC_DPD_ENABLE			0x24
#define PMC_DPD_ENABLE_ON		(1 << 0)
#define PMC_DPD_ENABLE_TSC_MULT_ENABLE	(1 << 1)

#define PWRGATE_TOGGLE			0x30
#define  PWRGATE_TOGGLE_START		(1 << 8)

#define REMOVE_CLAMPING			0x34

#define PWRGATE_STATUS			0x38

#define PMC_COREPWRGOOD_TIMER		0x3c

#define PMC_SCRATCH0			0x50
#define  PMC_SCRATCH0_MODE_RECOVERY	(1 << 31)
#define  PMC_SCRATCH0_MODE_BOOTLOADER	(1 << 30)
#define  PMC_SCRATCH0_MODE_RCM		(1 << 1)
#define  PMC_SCRATCH0_MODE_MASK		(PMC_SCRATCH0_MODE_RECOVERY | \
					 PMC_SCRATCH0_MODE_BOOTLOADER | \
					 PMC_SCRATCH0_MODE_RCM)

#define PMC_SCRATCH1			0x54

#define PMC_CPUPWRGOOD_TIMER		0xc8
#define PMC_CPUPWROFF_TIMER		0xcc

#define PMC_WAKE_DELAY			0xe0
#define PMC_COREPWROFF_TIMER		PMC_WAKE_DELAY

#define PMC_SCRATCH41			0x140

#define PMC_WAKE2_MASK			0x160
#define PMC_WAKE2_LEVEL			0x164
#define PMC_WAKE2_STATUS		0x168
#define PMC_SW_WAKE2_STATUS		0x16c

#define IO_DPD_REQ			0x1b8
#define  IO_DPD_CSIA			(1 << 0)
#define  IO_DPD_CSIB			(1 << 1)
#define  IO_DPD_DSI			(1 << 2)
#define  IO_DPD_MIPI_BIAS		(1 << 3)
#define  IO_DPD_PEX_BIAS		(1 << 4)
#define  IO_DPD_PEX_CLK1		(1 << 5)
#define  IO_DPD_PEX_CLK2		(1 << 6)
#define  IO_DPD_PEX_CLK3		(1 << 7)
#define  IO_DPD_DAC			(1 << 8)
#define  IO_DPD_USB0			(1 << 9)
#define  IO_DPD_USB1			(1 << 10)
#define  IO_DPD_USB2			(1 << 11)
#define  IO_DPD_USB_BIAS		(1 << 12)
#define  IO_DPD_NAND			(1 << 13)
#define  IO_DPD_UART			(1 << 14)
#define  IO_DPD_BB			(1 << 15)
#define  IO_DPD_VI			(1 << 16)
#define  IO_DPD_AUDIO			(1 << 17)
#define  IO_DPD_LCD			(1 << 18)
#define  IO_DPD_HSIC			(1 << 19)
#define  IO_DPD_REQ_CODE_IDLE		(0 << 30)
#define  IO_DPD_REQ_CODE_OFF		(1 << 30)
#define  IO_DPD_REQ_CODE_ON		(2 << 30)
#define  IO_DPD_REQ_CODE_MASK		(3 << 30)

#define IO_DPD_STATUS			0x1bc

#define IO_DPD2_REQ			0x1c0
#define  IO_DPD2_PEX_CNTRL		(1 << 0)
#define  IO_DPD2_SDMMC1			(1 << 1)
#define  IO_DPD2_SDMMC3			(1 << 2)
#define  IO_DPD2_SDMMC4			(1 << 3)
#define  IO_DPD2_CAM			(1 << 4)
#define  IO_DPD2_RES_RAIL		(1 << 5)
#define  IO_DPD2_HV			(1 << 6)
#define  IO_DPD2_DSIB			(1 << 7)
#define  IO_DPD2_DSIC			(1 << 8)
#define  IO_DPD2_DSID			(1 << 9)
#define  IO_DPD2_CSIC			(1 << 10)
#define  IO_DPD2_CSID			(1 << 11)
#define  IO_DPD2_CSIE			(1 << 12)

#define IO_DPD2_STATUS			0x1c4
#define SEL_DPD_TIM			0x1c8

#define DPD_STATE_CHANGE_DELAY		700

#define GPU_RG_CNTRL			0x2d4

#define PMC_SENSOR_CTRL			0x1b0
#define PMC_SENSOR_CTRL_SCRATCH_WRITE	(1 << 2)
#define PMC_SENSOR_CTRL_ENABLE_RST	(1 << 1)

#define PMC_SCRATCH54			0x258
#define PMC_SCRATCH54_DATA_SHIFT	8
#define PMC_SCRATCH54_ADDR_SHIFT	0

#define PMC_SCRATCH55			0x25c
#define PMC_SCRATCH55_RESET_TEGRA	(1 << 31)
#define PMC_SCRATCH55_CNTRL_ID_SHIFT	27
#define PMC_SCRATCH55_PINMUX_SHIFT	24
#define PMC_SCRATCH55_16BITOP		(1 << 15)
#define PMC_SCRATCH55_CHECKSUM_SHIFT	16
#define PMC_SCRATCH55_I2CSLV1_SHIFT	0

struct tegra_pmc_soc {
	unsigned int num_powergates;
	const char *const *powergates;
	unsigned int num_cpu_powergates;
	const u8 *cpu_powergates;
	bool has_tsense_reset;
};

/**
 * struct tegra_pmc - NVIDIA Tegra PMC
 * @dev: pointer to struct device
 * @base: pointer to I/O remapped register region
 * @clk: pointer to pclk clock
 * @rate: currently configured rate of pclk
 * @suspend_mode: lowest suspend mode available
 * @cpu_good_time: CPU power good time (in microseconds)
 * @cpu_off_time: CPU power off time (in microsecends)
 * @core_osc_time: core power good OSC time (in microseconds)
 * @core_pmu_time: core power good PMU time (in microseconds)
 * @core_off_time: core power off time (in microseconds)
 * @corereq_high: core power request is active-high
 * @sysclkreq_high: system clock request is active-high
 * @combined_req: combined power request for CPU & core
 * @cpu_pwr_good_en: CPU power good signal is enabled
 * @lp0_vec_phys: physical base address of the LP0 warm boot code
 * @lp0_vec_size: size of the LP0 warm boot code
 * @powergates_lock: mutex for power gate register access
 * @suspend_notifier: PM notifier for suspend events
 */
struct tegra_pmc {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;

	const struct tegra_pmc_soc *soc;

	unsigned long rate;

	enum tegra_suspend_mode suspend_mode;
	u32 cpu_good_time;
	u32 cpu_off_time;
	u32 core_osc_time;
	u32 core_pmu_time;
	u32 core_off_time;
	bool corereq_high;
	bool sysclkreq_high;
	bool combined_req;
	bool cpu_pwr_good_en;
	u32 lp0_vec_phys;
	u32 lp0_vec_size;

	struct mutex powergates_lock;
	struct notifier_block suspend_notifier;
};

#ifdef CONFIG_PM_SLEEP
#define PMC_WAKE_TYPE_INDEX	0
#define PMC_WAKE_MASK_INDEX	1
#define PMC_TRIGGER_TYPE_INDEX	2
#define PMC_OF_ARGS_COUNT	3
struct pmc_wakeup {
	u32 wake_type;
	u32 wake_mask_offset;
	u32 irq_num;
	struct list_head list;
};

struct pmc_lp0_wakeup {
	struct device_node *of_node;
	u64 enable;
	u64 level;
	u64 level_any;
	struct list_head wake_list;
};
static struct pmc_lp0_wakeup tegra_lp0_wakeup;
static u32 io_dpd_reg, io_dpd2_reg;
#endif

static struct tegra_pmc *pmc = &(struct tegra_pmc) {
	.base = NULL,
	.suspend_mode = TEGRA_SUSPEND_NONE,
};

static u32 tegra_pmc_readl(unsigned long offset)
{
	return readl(pmc->base + offset);
}

static void tegra_pmc_writel(u32 value, unsigned long offset)
{
	writel(value, pmc->base + offset);
}

/**
 * tegra_powergate_set() - set the state of a partition
 * @id: partition ID
 * @new_state: new state of the partition
 */
static int tegra_powergate_set(int id, bool new_state)
{
	bool status;

	mutex_lock(&pmc->powergates_lock);

	status = tegra_pmc_readl(PWRGATE_STATUS) & (1 << id);

	if (status == new_state) {
		mutex_unlock(&pmc->powergates_lock);
		return 0;
	}

	tegra_pmc_writel(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);

	mutex_unlock(&pmc->powergates_lock);

	return 0;
}

/**
 * tegra_powergate_power_on() - power on partition
 * @id: partition ID
 */
int tegra_powergate_power_on(int id)
{
	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	return tegra_powergate_set(id, true);
}

/**
 * tegra_powergate_power_off() - power off partition
 * @id: partition ID
 */
int tegra_powergate_power_off(int id)
{
	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	return tegra_powergate_set(id, false);
}
EXPORT_SYMBOL(tegra_powergate_power_off);

/**
 * tegra_powergate_is_powered() - check if partition is powered
 * @id: partition ID
 */
int tegra_powergate_is_powered(int id)
{
	u32 status;

	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	status = tegra_pmc_readl(PWRGATE_STATUS) & (1 << id);
	return !!status;
}

/**
 * tegra_powergate_remove_clamping() - remove power clamps for partition
 * @id: partition ID
 */
int tegra_powergate_remove_clamping(int id)
{
	u32 mask;

	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	/*
	 * The Tegra124 GPU has a separate register (with different semantics)
	 * to remove clamps.
	 */
	if (tegra_get_chip_id() == TEGRA124) {
		if (id == TEGRA_POWERGATE_3D) {
			tegra_pmc_writel(0, GPU_RG_CNTRL);
			return 0;
		}
	}

	/*
	 * Tegra 2 has a bug where PCIE and VDE clamping masks are
	 * swapped relatively to the partition ids
	 */
	if (id == TEGRA_POWERGATE_VDEC)
		mask = (1 << TEGRA_POWERGATE_PCIE);
	else if (id == TEGRA_POWERGATE_PCIE)
		mask = (1 << TEGRA_POWERGATE_VDEC);
	else
		mask = (1 << id);

	tegra_pmc_writel(mask, REMOVE_CLAMPING);

	return 0;
}
EXPORT_SYMBOL(tegra_powergate_remove_clamping);

/**
 * tegra_powergate_sequence_power_up() - power up partition
 * @id: partition ID
 * @clk: clock for partition
 * @rst: reset for partition
 *
 * Must be called with clk disabled, and returns with clk enabled.
 */
int tegra_powergate_sequence_power_up(int id, struct clk *clk,
				      struct reset_control *rst)
{
	int ret;

	reset_control_assert(rst);

	ret = tegra_powergate_power_on(id);
	if (ret)
		goto err_power;

	ret = clk_prepare_enable(clk);
	if (ret)
		goto err_clk;

	usleep_range(10, 20);

	ret = tegra_powergate_remove_clamping(id);
	if (ret)
		goto err_clamp;

	usleep_range(10, 20);
	reset_control_deassert(rst);

	return 0;

err_clamp:
	clk_disable_unprepare(clk);
err_clk:
	tegra_powergate_power_off(id);
err_power:
	return ret;
}
EXPORT_SYMBOL(tegra_powergate_sequence_power_up);

/**
 * tegra_powergate_sequence_power_down() - power down partition
 * @id: partition ID
 * @clk: clock for partition
 * @rst: reset for partition
 *
 * Must be called with clk enabled, and returns with clk disabled.
 */
int tegra_powergate_sequence_power_down(int id, struct clk *clk,
					struct reset_control *rst)
{
	int ret;

	ret = reset_control_assert(rst);
	if (ret)
		return ret;
	usleep_range(10, 20);

	clk_disable_unprepare(clk);
	usleep_range(10, 20);

	ret = tegra_powergate_power_off(id);
	if (ret) {
		clk_prepare_enable(clk);
		reset_control_deassert(rst);
	}

	return ret;
}
EXPORT_SYMBOL(tegra_powergate_sequence_power_down);

#ifdef CONFIG_SMP
/**
 * tegra_get_cpu_powergate_id() - convert from CPU ID to partition ID
 * @cpuid: CPU partition ID
 *
 * Returns the partition ID corresponding to the CPU partition ID or a
 * negative error code on failure.
 */
static int tegra_get_cpu_powergate_id(int cpuid)
{
	if (pmc->soc && cpuid > 0 && cpuid < pmc->soc->num_cpu_powergates)
		return pmc->soc->cpu_powergates[cpuid];

	return -EINVAL;
}

/**
 * tegra_pmc_cpu_is_powered() - check if CPU partition is powered
 * @cpuid: CPU partition ID
 */
bool tegra_pmc_cpu_is_powered(int cpuid)
{
	int id;

	id = tegra_get_cpu_powergate_id(cpuid);
	if (id < 0)
		return false;

	return tegra_powergate_is_powered(id);
}

/**
 * tegra_pmc_cpu_power_on() - power on CPU partition
 * @cpuid: CPU partition ID
 */
int tegra_pmc_cpu_power_on(int cpuid)
{
	int id;

	id = tegra_get_cpu_powergate_id(cpuid);
	if (id < 0)
		return id;

	return tegra_powergate_set(id, true);
}

/**
 * tegra_pmc_cpu_remove_clamping() - remove power clamps for CPU partition
 * @cpuid: CPU partition ID
 */
int tegra_pmc_cpu_remove_clamping(int cpuid)
{
	int id;

	id = tegra_get_cpu_powergate_id(cpuid);
	if (id < 0)
		return id;

	return tegra_powergate_remove_clamping(id);
}
#endif /* CONFIG_SMP */

/**
 * tegra_pmc_restart() - reboot the system
 * @mode: which mode to reboot in
 * @cmd: reboot command
 */
static int tegra_pmc_restart_notify(struct notifier_block *this,
			unsigned long mode, void *cmd)
{
	u32 value;
	const char *cmd_str = (const char *) cmd;

	value = tegra_pmc_readl(PMC_SCRATCH0);
	value &= ~PMC_SCRATCH0_MODE_MASK;

	if (cmd_str) {
		if (strcmp(cmd_str, "recovery") == 0)
			value |= PMC_SCRATCH0_MODE_RECOVERY;

		if (strcmp(cmd_str, "bootloader") == 0)
			value |= PMC_SCRATCH0_MODE_BOOTLOADER;

		if (strcmp(cmd_str, "forced-recovery") == 0)
			value |= PMC_SCRATCH0_MODE_RCM;
	}

	tegra_pmc_writel(value, PMC_SCRATCH0);

	value = tegra_pmc_readl(0);
	value |= 0x10;
	tegra_pmc_writel(value, 0);

	return NOTIFY_DONE;
}

static struct notifier_block tegra_pmc_restart_handler = {
	.notifier_call = tegra_pmc_restart_notify,
	.priority = 128,
};

static int powergate_show(struct seq_file *s, void *data)
{
	unsigned int i;

	seq_printf(s, " powergate powered\n");
	seq_printf(s, "------------------\n");

	for (i = 0; i < pmc->soc->num_powergates; i++) {
		if (!pmc->soc->powergates[i])
			continue;

		seq_printf(s, " %9s %7s\n", pmc->soc->powergates[i],
			   tegra_powergate_is_powered(i) ? "yes" : "no");
	}

	return 0;
}

static int powergate_open(struct inode *inode, struct file *file)
{
	return single_open(file, powergate_show, inode->i_private);
}

static const struct file_operations powergate_fops = {
	.open = powergate_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int tegra_powergate_debugfs_init(void)
{
	struct dentry *d;

	d = debugfs_create_file("powergate", S_IRUGO, NULL, NULL,
				&powergate_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}

static int tegra_io_rail_prepare(int id, unsigned long *request,
				 unsigned long *status, unsigned int *bit)
{
	unsigned long rate, value;
	struct clk *clk;

	*bit = id % 32;

	/*
	 * There are two sets of 30 bits to select IO rails, but bits 30 and
	 * 31 are control bits rather than IO rail selection bits.
	 */
	if (id > 63 || *bit == 30 || *bit == 31)
		return -EINVAL;

	if (id < 32) {
		*status = IO_DPD_STATUS;
		*request = IO_DPD_REQ;
	} else {
		*status = IO_DPD2_STATUS;
		*request = IO_DPD2_REQ;
	}

	clk = clk_get_sys(NULL, "pclk");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	rate = clk_get_rate(clk);
	clk_put(clk);

	tegra_pmc_writel(DPD_SAMPLE_ENABLE, DPD_SAMPLE);

	/* must be at least 200 ns, in APB (PCLK) clock cycles */
	value = DIV_ROUND_UP(1000000000, rate);
	value = DIV_ROUND_UP(200, value);
	tegra_pmc_writel(value, SEL_DPD_TIM);

	return 0;
}

static int tegra_io_rail_poll(unsigned long offset, unsigned long mask,
			      unsigned long val, unsigned long timeout)
{
	unsigned long value;

	timeout = jiffies + msecs_to_jiffies(timeout);

	while (time_after(timeout, jiffies)) {
		value = tegra_pmc_readl(offset);
		if ((value & mask) == val)
			return 0;

		usleep_range(250, 1000);
	}

	return -ETIMEDOUT;
}

static void tegra_io_rail_unprepare(void)
{
	tegra_pmc_writel(DPD_SAMPLE_DISABLE, DPD_SAMPLE);
}

int tegra_io_rail_power_on(int id)
{
	unsigned long request, status, value;
	unsigned int bit, mask;
	int err;

	err = tegra_io_rail_prepare(id, &request, &status, &bit);
	if (err < 0)
		return err;

	mask = 1 << bit;

	value = tegra_pmc_readl(request);
	value |= mask;
	value &= ~IO_DPD_REQ_CODE_MASK;
	value |= IO_DPD_REQ_CODE_OFF;
	tegra_pmc_writel(value, request);

	err = tegra_io_rail_poll(status, mask, 0, 250);
	if (err < 0)
		return err;

	tegra_io_rail_unprepare();

	return 0;
}
EXPORT_SYMBOL(tegra_io_rail_power_on);

int tegra_io_rail_power_off(int id)
{
	unsigned long request, status, value;
	unsigned int bit, mask;
	int err;

	err = tegra_io_rail_prepare(id, &request, &status, &bit);
	if (err < 0)
		return err;

	mask = 1 << bit;

	value = tegra_pmc_readl(request);
	value |= mask;
	value &= ~IO_DPD_REQ_CODE_MASK;
	value |= IO_DPD_REQ_CODE_ON;
	tegra_pmc_writel(value, request);

	err = tegra_io_rail_poll(status, mask, mask, 250);
	if (err < 0)
		return err;

	tegra_io_rail_unprepare();

	return 0;
}
EXPORT_SYMBOL(tegra_io_rail_power_off);

#ifdef CONFIG_PM_SLEEP
void tegra_tsc_suspend(void)
{
	if (IS_ENABLED(CONFIG_ARM_ARCH_TIMER)) {
		u32 reg;

		reg = tegra_pmc_readl(PMC_DPD_ENABLE);
		reg |= PMC_DPD_ENABLE_TSC_MULT_ENABLE;
		tegra_pmc_writel(reg, PMC_DPD_ENABLE);
	}
}

void tegra_tsc_resume(void)
{
	if (IS_ENABLED(CONFIG_ARM_ARCH_TIMER)) {
		u32 reg;

		reg = tegra_pmc_readl(PMC_DPD_ENABLE);
		reg &= ~PMC_DPD_ENABLE_TSC_MULT_ENABLE;
		switch (tegra_get_chip_id()) {
		case TEGRA124:
		case TEGRA132:
			/* WAR to avoid PMC wake status getting cleared */
			reg &= ~PMC_DPD_ENABLE_ON;
			break;
		default:
			break;
		}
		tegra_pmc_writel(reg, PMC_DPD_ENABLE);
	}
}

static void tegra_pmc_remove_dpd_req(void)
{
	/* Clear DPD req */
	tegra_pmc_writel(io_dpd_reg | IO_DPD_REQ_CODE_OFF, IO_DPD_REQ);
	tegra_pmc_readl(IO_DPD_REQ); /* unblock posted write */
	/* delay apb_clk * (SEL_DPD_TIM*5) */
	udelay(DPD_STATE_CHANGE_DELAY);

	tegra_pmc_writel(io_dpd2_reg | IO_DPD_REQ_CODE_OFF, IO_DPD2_REQ);
	tegra_pmc_readl(IO_DPD2_REQ); /* unblock posted write */
	udelay(DPD_STATE_CHANGE_DELAY);
}

void tegra_pmc_lp0_resume(void)
{
	tegra_pmc_remove_dpd_req();
}

static void tegra_pmc_clear_dpd_sample(void)
{
	/* Clear DPD sample */
	tegra_pmc_writel(0x0, DPD_SAMPLE);
}

static void tegra_pmc_add_wakeup_event(struct of_phandle_args *ph_args,
				       struct device *dev,
				       struct device_node *np)
{
	struct platform_device *pdev;
	struct pmc_wakeup *pmc_wake_source;
	struct irq_desc *irqd;
	struct irq_data *irq_data;
	int pmc_wake_type, wake;
	int irq, pmc_trigger_type;

	if (ph_args->np != tegra_lp0_wakeup.of_node)
		return;
	if (ph_args->args_count != PMC_OF_ARGS_COUNT)
		return;

	pdev = to_platform_device(dev);
	irq = platform_get_irq(pdev, 0);
	pmc_wake_type = ph_args->args[PMC_WAKE_TYPE_INDEX];

	switch (pmc_wake_type) {
	case PMC_WAKE_TYPE_GPIO:
		if (irq < 0) {
			int gpio;

			gpio = of_get_named_gpio(np, "gpios", 0);
			irq = gpio_to_irq(gpio);
			if (WARN_ON(irq < 0))
				return;
		}
		irqd = irq_to_desc(irq);
		irq_data = &irqd->irq_data;
		pmc_trigger_type = irqd_get_trigger_type(irq_data);
		break;
	case PMC_WAKE_TYPE_EVENT:
		pmc_trigger_type = ph_args->args[PMC_TRIGGER_TYPE_INDEX];
		break;
	default:
		return;
	}

	pmc_wake_source = kzalloc(sizeof(*pmc_wake_source), GFP_KERNEL);
	if (!pmc_wake_source)
		return;

	pmc_wake_source->wake_type = pmc_wake_type;
	pmc_wake_source->irq_num = irq;
	pmc_wake_source->wake_mask_offset = ph_args->args[PMC_WAKE_MASK_INDEX];
	wake = pmc_wake_source->wake_mask_offset;

	list_add_tail(&pmc_wake_source->list, &tegra_lp0_wakeup.wake_list);

	tegra_lp0_wakeup.enable |= 1ULL << wake;
	switch (pmc_trigger_type) {
	case IRQF_TRIGGER_FALLING:
	case IRQF_TRIGGER_LOW:
		tegra_lp0_wakeup.level &= ~(1ULL << wake);
		tegra_lp0_wakeup.level_any &= ~(1ULL << wake);
		break;
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_RISING:
		tegra_lp0_wakeup.level |= (1ULL << wake);
		tegra_lp0_wakeup.level_any &= ~(1ULL << wake);
		break;
	case IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING:
		tegra_lp0_wakeup.level_any |= (1ULL << wake);
		break;
	default:
		break;
	}
}

static void tegra_of_device_add_pmc_wake(struct device *dev)
{
	struct of_phandle_args ph_args;
	struct device_node *np = NULL;
	int child_node_num;

	child_node_num = of_get_child_count(dev->of_node);
	if (child_node_num == 0) {
		if (!of_parse_phandle_with_args(dev->of_node,
					       "nvidia,pmc-wakeup",
					       "#wake-cells", 0, &ph_args))
			tegra_pmc_add_wakeup_event(&ph_args, dev, dev->of_node);
	} else {
		for_each_child_of_node(dev->of_node, np)
			if (!of_parse_phandle_with_args(np,
						"nvidia,pmc-wakeup",
						"#wake-cells", 0, &ph_args))
				tegra_pmc_add_wakeup_event(&ph_args, dev, np);
	}

	of_node_put(ph_args.np);
}

static int tegra_pmc_wake_notifier_call(struct notifier_block *nb,
				      unsigned long event, void *data)
{
	struct device *dev = data;

	switch (event) {
	case BUS_NOTIFY_BOUND_DRIVER:
		if (dev->of_node)
			tegra_of_device_add_pmc_wake(dev);
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block tegra_pmc_wake_notifier = {
	.notifier_call = tegra_pmc_wake_notifier_call,
};

static int __init tegra_pmc_lp0_wakeup_init(void)
{
	if (!soc_is_tegra())
		return 0;

	bus_register_notifier(&platform_bus_type, &tegra_pmc_wake_notifier);
	return 0;
}
arch_initcall(tegra_pmc_lp0_wakeup_init);

static inline void write_pmc_wake_mask(u64 value)
{
	pr_info("PMC wake enable = 0x%llx\n", value);
	tegra_pmc_writel((u32)value, PMC_WAKE_MASK);
	if (tegra_get_chip_id() != TEGRA20)
		tegra_pmc_writel((u32)(value >> 32), PMC_WAKE2_MASK);
}

static inline u64 read_pmc_wake_level(void)
{
	u64 reg;

	reg = tegra_pmc_readl(PMC_WAKE_LEVEL);
	if (tegra_get_chip_id() != TEGRA20)
		reg |= ((u64)tegra_pmc_readl(PMC_WAKE2_LEVEL)) << 32;

	return reg;
}

static inline void write_pmc_wake_level(u64 value)
{
	pr_info("PMC wake level = 0x%llx\n", value);
	tegra_pmc_writel((u32)value, PMC_WAKE_LEVEL);
	if (tegra_get_chip_id() != TEGRA20)
		tegra_pmc_writel((u32)(value >> 32), PMC_WAKE2_LEVEL);
}

static inline u64 read_pmc_wake_status(void)
{
	u64 reg;

	reg = tegra_pmc_readl(PMC_WAKE_STATUS);
	if (tegra_get_chip_id() != TEGRA20)
		reg |= ((u64)tegra_pmc_readl(PMC_WAKE2_STATUS)) << 32;

	return reg;
}

static inline void clear_pmc_wake_status(void)
{
	u32 reg;

	reg = tegra_pmc_readl(PMC_WAKE_STATUS);
	if (reg)
		tegra_pmc_writel(reg, PMC_WAKE_STATUS);
	if (tegra_get_chip_id() != TEGRA20) {
		reg = tegra_pmc_readl(PMC_WAKE2_STATUS);
		if (reg)
			tegra_pmc_writel(reg, PMC_WAKE2_STATUS);
	}
}

static inline u64 read_pmc_sw_wake_status(void)
{
	u64 reg;

	reg = tegra_pmc_readl(PMC_SW_WAKE_STATUS);
	if (tegra_get_chip_id() != TEGRA20)
		reg |= ((u64)tegra_pmc_readl(PMC_SW_WAKE2_STATUS)) << 32;

	return reg;
}

static inline void clear_pmc_sw_wake_status(void)
{
	tegra_pmc_writel(0, PMC_SW_WAKE_STATUS);
	if (tegra_get_chip_id() != TEGRA20)
		tegra_pmc_writel(0, PMC_SW_WAKE2_STATUS);
}

/* translate lp0 wake sources back into irqs to catch edge triggered wakeups */
static void tegra_pmc_wake_syscore_resume(void)
{
	struct pmc_wakeup *wake;
	struct irq_desc *desc;
	u64 wake_status = read_pmc_wake_status();

	pr_info("PMC wake status = 0x%llx\n", wake_status);

	list_for_each_entry(wake, &tegra_lp0_wakeup.wake_list, list) {
		if (!(wake_status & BIT(wake->wake_mask_offset)))
			continue;

		if (wake->irq_num <= 0) {
			pr_info("Resume caused by PMC WAKE%d\n",
				wake->wake_mask_offset);
			continue;
		}

		desc = irq_to_desc(wake->irq_num);
		if (!desc || !desc->action || !desc->action->name) {
			pr_info("Resume caused by PMC WAKE%d, irq %d\n",
				wake->wake_mask_offset, wake->irq_num);
			continue;
		}

		pr_info("Resume caused by PMC WAKE%d, %s\n",
			wake->wake_mask_offset, desc->action->name);
		generic_handle_irq(wake->irq_num);
	}
}

static int tegra_pmc_wake_syscore_suspend(void)
{
	u32 reg;
	u64 status;
	u64 lvl;
	u64 wake_level;
	u64 wake_enb;

	clear_pmc_sw_wake_status();

	/* enable PMC wake */
	reg = tegra_pmc_readl(PMC_CNTRL);
	reg |= PMC_CNTRL_LATCH_WAKEUPS;
	tegra_pmc_writel(reg, PMC_CNTRL);
	udelay(120);

	reg &= ~PMC_CNTRL_LATCH_WAKEUPS;
	tegra_pmc_writel(reg, PMC_CNTRL);
	udelay(120);

	status = read_pmc_sw_wake_status();

	lvl = read_pmc_wake_level();

	/*
	 * flip the wakeup trigger for any-edge triggered pads
	 * which are currently asserting as wakeups
	 */
	lvl ^= status;

	lvl &= tegra_lp0_wakeup.level_any;

	wake_level = lvl | tegra_lp0_wakeup.level;
	wake_enb = tegra_lp0_wakeup.enable;

	/* Clear PMC Wake Status registers while going to suspend */
	clear_pmc_wake_status();
	write_pmc_wake_level(wake_level);
	write_pmc_wake_mask(wake_enb);

	return 0;
}

static struct syscore_ops tegra_pmc_wake_syscore_ops = {
	.suspend = tegra_pmc_wake_syscore_suspend,
	.resume = tegra_pmc_wake_syscore_resume,
};

static void tegra_pmc_wake_syscore_init(void)
{
	register_syscore_ops(&tegra_pmc_wake_syscore_ops);
}

static int tegra_pmc_suspend(void)
{
	tegra_pmc_writel(virt_to_phys(tegra_resume), PMC_SCRATCH41);

	return 0;
}

static void tegra_pmc_resume(void)
{
	tegra_pmc_clear_dpd_sample();
	/* Clear DPD Enable */
	switch (tegra_get_chip_id()) {
	case TEGRA20:
	case TEGRA30:
	case TEGRA114:
		break;
	default:
		tegra_pmc_writel(0x0, PMC_DPD_ENABLE);
		break;
	}

	tegra_pmc_writel(0x0, PMC_SCRATCH41);
}

static struct syscore_ops tegra_pmc_syscore_ops = {
	.suspend = tegra_pmc_suspend,
	.resume = tegra_pmc_resume,
};

static void tegra_pmc_syscore_init(void)
{
	register_syscore_ops(&tegra_pmc_syscore_ops);
}

static void set_core_power_timers(void)
{
	unsigned long osc, pmu, off;

	osc = DIV_ROUND_UP_ULL(pmc->core_osc_time * 32768, 1000000);
	pmu = DIV_ROUND_UP_ULL(pmc->core_pmu_time * 32768, 1000000);
	off = DIV_ROUND_UP_ULL(pmc->core_off_time * 32768, 1000000);

	tegra_pmc_writel(((osc << 8) & 0xff00) | (pmu & 0xff),
			 PMC_COREPWRGOOD_TIMER);
	tegra_pmc_writel(off, PMC_COREPWROFF_TIMER);
}

enum tegra_suspend_mode tegra_pmc_get_suspend_mode(void)
{
	return pmc->suspend_mode;
}

void tegra_pmc_set_suspend_mode(enum tegra_suspend_mode mode)
{
	if (mode < TEGRA_SUSPEND_NONE || mode >= TEGRA_MAX_SUSPEND_MODE)
		return;

	pmc->suspend_mode = mode;
}

void tegra_pmc_enter_suspend_mode(enum tegra_suspend_mode mode)
{
	unsigned long long rate = 0;
	u32 boot_flag, cntrl_value;

	cntrl_value = tegra_pmc_readl(PMC_CNTRL);
	cntrl_value &= ~PMC_CNTRL_SIDE_EFFECT_LP0;
	if (pmc->combined_req)
		cntrl_value &= ~PMC_CNTRL_PWRREQ_OE;
	else
		cntrl_value |= PMC_CNTRL_PWRREQ_OE;
	cntrl_value |= PMC_CNTRL_CPU_PWRREQ_OE;

	switch (mode) {
	case TEGRA_SUSPEND_LP0:
		/*
		 * Enable DPD sample to trigger sampling pads data and direction
		 * in which pad will be driven during LP0 mode.
		 */
		tegra_pmc_writel(0x1, DPD_SAMPLE);

		/*
		 * Power down IO logic
		 */
		switch (tegra_get_chip_id()) {
		case TEGRA114:
		case TEGRA124:
		case TEGRA132:
			io_dpd_reg = IO_DPD_CSIA | IO_DPD_CSIB | IO_DPD_DSI |
				IO_DPD_MIPI_BIAS | IO_DPD_PEX_BIAS |
				IO_DPD_PEX_CLK1 | IO_DPD_PEX_CLK2 |
				IO_DPD_PEX_CLK3 | IO_DPD_DAC | IO_DPD_USB0 |
				IO_DPD_USB1 | IO_DPD_USB2 | IO_DPD_USB_BIAS |
				IO_DPD_UART | IO_DPD_BB | IO_DPD_VI |
				IO_DPD_AUDIO | IO_DPD_LCD | IO_DPD_HSIC;
			io_dpd2_reg = IO_DPD2_PEX_CNTRL | IO_DPD2_SDMMC1 |
				IO_DPD2_SDMMC3 | IO_DPD2_SDMMC4 | IO_DPD2_CAM |
				IO_DPD2_RES_RAIL | IO_DPD2_HV | IO_DPD2_DSIB |
				IO_DPD2_DSIC | IO_DPD2_DSID | IO_DPD2_CSIC |
				IO_DPD2_CSID | IO_DPD2_CSIE;
			break;
		default:
			break;
		}
		tegra_pmc_writel(io_dpd_reg | IO_DPD_REQ_CODE_ON, IO_DPD_REQ);
		tegra_pmc_readl(IO_DPD_REQ); /* unblock posted write */

		/* delay apb_clk * (SEL_DPD_TIM*5) */
		udelay(DPD_STATE_CHANGE_DELAY);

		tegra_pmc_writel(io_dpd2_reg | IO_DPD_REQ_CODE_ON, IO_DPD2_REQ);
		tegra_pmc_readl(IO_DPD2_REQ); /* unblock posted write */
		udelay(DPD_STATE_CHANGE_DELAY);

		/* Set warmboot flag */
		boot_flag = tegra_pmc_readl(PMC_SCRATCH0);
		tegra_pmc_writel(boot_flag | 1, PMC_SCRATCH0);

		tegra_pmc_writel(pmc->lp0_vec_phys, PMC_SCRATCH1);
		cntrl_value |= PMC_CNTRL_SIDE_EFFECT_LP0;
	case TEGRA_SUSPEND_LP1:
		rate = 32768;
		break;

	case TEGRA_SUSPEND_LP2:
		rate = clk_get_rate(pmc->clk);
		break;

	default:
		break;
	}

	if (WARN_ON_ONCE(rate == 0))
		rate = 100000000;

	if (rate != pmc->rate) {
		u64 ticks;

		ticks = pmc->cpu_good_time * rate + USEC_PER_SEC - 1;
		do_div(ticks, USEC_PER_SEC);
		tegra_pmc_writel(ticks, PMC_CPUPWRGOOD_TIMER);

		ticks = pmc->cpu_off_time * rate + USEC_PER_SEC - 1;
		do_div(ticks, USEC_PER_SEC);
		tegra_pmc_writel(ticks, PMC_CPUPWROFF_TIMER);

		wmb();

		pmc->rate = rate;
	}

	tegra_pmc_writel(cntrl_value, PMC_CNTRL);
}

/*
 * When starting to enter LP0 without LP0 boot code, try to request the
 * code with request_firmware, if it can't be loaded, switch to LP1.
 */
static int tegra_pmc_suspend_notifier(struct notifier_block *nb,
				      unsigned long event,
				      void *ptr)
{
	const struct firmware *fw;
	int ret;
	void *fw_buff;
	const char fw_name[] = "tegra_lp0_resume.fw";
	char fw_path[32];

	if (event != PM_SUSPEND_PREPARE)
		return 0;

	if (pmc->suspend_mode != TEGRA_SUSPEND_LP0 || pmc->lp0_vec_size)
		return 0;

	switch (tegra_get_chip_id()) {
	case TEGRA124:
		sprintf(fw_path, "tegra12x/%s", fw_name);
		break;
	case TEGRA132:
		sprintf(fw_path, "tegra13x/%s", fw_name);
		break;
	default:
		break;
	}

	ret = request_firmware(&fw, fw_path, pmc->dev);
	if (ret) {
		dev_info(pmc->dev, "Disabling LP0, no resume code found\n");
		pmc->suspend_mode = TEGRA_SUSPEND_LP1;
		return 0;
	}

	fw_buff = (void *)__get_dma_pages(GFP_DMA32, get_order(fw->size));
	if (!fw_buff) {
		pmc->suspend_mode = TEGRA_SUSPEND_LP1;
		goto suspend_check_done;
	}
	dev_info(pmc->dev, "Loaded LP0 firmware with request_firmware.\n");

	memcpy(fw_buff, fw->data, fw->size);
	pmc->lp0_vec_phys = virt_to_phys(fw_buff);
	pmc->lp0_vec_size = fw->size;
suspend_check_done:
	release_firmware(fw);

	return 0;
}
#endif

static const char * const tegra20_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "cpu",
	[TEGRA_POWERGATE_3D] = "3d",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_VDEC] = "vdec",
	[TEGRA_POWERGATE_PCIE] = "pcie",
	[TEGRA_POWERGATE_L2] = "l2",
	[TEGRA_POWERGATE_MPE] = "mpe",
};

static const struct tegra_pmc_soc tegra20_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra20_powergates),
	.powergates = tegra20_powergates,
	.num_cpu_powergates = 0,
	.cpu_powergates = NULL,
};

static const char * const tegra30_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "cpu0",
	[TEGRA_POWERGATE_3D] = "3d0",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_VDEC] = "vdec",
	[TEGRA_POWERGATE_PCIE] = "pcie",
	[TEGRA_POWERGATE_L2] = "l2",
	[TEGRA_POWERGATE_MPE] = "mpe",
	[TEGRA_POWERGATE_HEG] = "heg",
	[TEGRA_POWERGATE_SATA] = "sata",
	[TEGRA_POWERGATE_CPU1] = "cpu1",
	[TEGRA_POWERGATE_CPU2] = "cpu2",
	[TEGRA_POWERGATE_CPU3] = "cpu3",
	[TEGRA_POWERGATE_CELP] = "celp",
	[TEGRA_POWERGATE_3D1] = "3d1",
};

static const u8 tegra30_cpu_powergates[] = {
	TEGRA_POWERGATE_CPU,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static const struct tegra_pmc_soc tegra30_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra30_powergates),
	.powergates = tegra30_powergates,
	.num_cpu_powergates = ARRAY_SIZE(tegra30_cpu_powergates),
	.cpu_powergates = tegra30_cpu_powergates,
	.has_tsense_reset = true,
};

static const char * const tegra114_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "crail",
	[TEGRA_POWERGATE_3D] = "3d",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_VDEC] = "vdec",
	[TEGRA_POWERGATE_MPE] = "mpe",
	[TEGRA_POWERGATE_HEG] = "heg",
	[TEGRA_POWERGATE_CPU1] = "cpu1",
	[TEGRA_POWERGATE_CPU2] = "cpu2",
	[TEGRA_POWERGATE_CPU3] = "cpu3",
	[TEGRA_POWERGATE_CELP] = "celp",
	[TEGRA_POWERGATE_CPU0] = "cpu0",
	[TEGRA_POWERGATE_C0NC] = "c0nc",
	[TEGRA_POWERGATE_C1NC] = "c1nc",
	[TEGRA_POWERGATE_DIS] = "dis",
	[TEGRA_POWERGATE_DISB] = "disb",
	[TEGRA_POWERGATE_XUSBA] = "xusba",
	[TEGRA_POWERGATE_XUSBB] = "xusbb",
	[TEGRA_POWERGATE_XUSBC] = "xusbc",
};

static const u8 tegra114_cpu_powergates[] = {
	TEGRA_POWERGATE_CPU0,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static const struct tegra_pmc_soc tegra114_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra114_powergates),
	.powergates = tegra114_powergates,
	.num_cpu_powergates = ARRAY_SIZE(tegra114_cpu_powergates),
	.cpu_powergates = tegra114_cpu_powergates,
	.has_tsense_reset = true,
};

static const char * const tegra124_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "crail",
	[TEGRA_POWERGATE_3D] = "3d",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_PCIE] = "pcie",
	[TEGRA_POWERGATE_VDEC] = "vdec",
	[TEGRA_POWERGATE_L2] = "l2",
	[TEGRA_POWERGATE_MPE] = "mpe",
	[TEGRA_POWERGATE_HEG] = "heg",
	[TEGRA_POWERGATE_SATA] = "sata",
	[TEGRA_POWERGATE_CPU1] = "cpu1",
	[TEGRA_POWERGATE_CPU2] = "cpu2",
	[TEGRA_POWERGATE_CPU3] = "cpu3",
	[TEGRA_POWERGATE_CELP] = "celp",
	[TEGRA_POWERGATE_CPU0] = "cpu0",
	[TEGRA_POWERGATE_C0NC] = "c0nc",
	[TEGRA_POWERGATE_C1NC] = "c1nc",
	[TEGRA_POWERGATE_SOR] = "sor",
	[TEGRA_POWERGATE_DIS] = "dis",
	[TEGRA_POWERGATE_DISB] = "disb",
	[TEGRA_POWERGATE_XUSBA] = "xusba",
	[TEGRA_POWERGATE_XUSBB] = "xusbb",
	[TEGRA_POWERGATE_XUSBC] = "xusbc",
	[TEGRA_POWERGATE_VIC] = "vic",
	[TEGRA_POWERGATE_IRAM] = "iram",
};

static const u8 tegra124_cpu_powergates[] = {
	TEGRA_POWERGATE_CPU0,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static const struct tegra_pmc_soc tegra124_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra124_powergates),
	.powergates = tegra124_powergates,
	.num_cpu_powergates = ARRAY_SIZE(tegra124_cpu_powergates),
	.cpu_powergates = tegra124_cpu_powergates,
	.has_tsense_reset = true,
};

static const struct of_device_id tegra_pmc_match[] = {
	{ .compatible = "nvidia,tegra124-pmc", .data = &tegra124_pmc_soc },
	{ .compatible = "nvidia,tegra114-pmc", .data = &tegra114_pmc_soc },
	{ .compatible = "nvidia,tegra30-pmc", .data = &tegra30_pmc_soc },
	{ .compatible = "nvidia,tegra20-pmc", .data = &tegra20_pmc_soc },
	{ }
};

void tegra_pmc_init_tsense_reset(struct device *dev)
{
	u32 pmu_i2c_addr, i2c_ctrl_id, reg_addr, reg_data, pinmux;
	u32 value, checksum;
	struct device_node *np = dev->of_node;
	struct device_node *i2c_np, *tt_np;
	const struct of_device_id *match = of_match_node(tegra_pmc_match, np);
	const struct tegra_pmc_soc *data = match->data;

	if (!data->has_tsense_reset)
		return;

	tt_np = of_find_node_by_name(np, "i2c-thermtrip");
	if (!tt_np) {
		dev_warn(dev, "no i2c-thermtrip node found, disabling emergency thermal reset\n");
		return;
	}

	i2c_np = of_parse_phandle(tt_np, "nvidia,pmu", 0);
	if (!i2c_np) {
		dev_err(dev, "PMU reference missing, disabling emergency thermal reset\n");
		goto put_tt;
	}

	if (of_property_read_u32(i2c_np, "reg", &pmu_i2c_addr)) {
		dev_err(dev, "PMU address missing, disabling emergency thermal reset\n");
		goto put_i2c;
	}

	if (of_property_read_u32(i2c_np->parent, "nvidia,controller-id", &i2c_ctrl_id)) {
		dev_err(dev, "PMU controller id missing, disabling emergency thermal reset\n");
		goto put_i2c;
	}

	of_node_put(i2c_np);

	if (of_property_read_u32(tt_np, "nvidia,reg-addr", &reg_addr)) {
		dev_err(dev, "nvidia,reg-addr missing, disabling emergency thermal reset\n");
		goto put_tt;
	}

	if (of_property_read_u32(tt_np, "nvidia,reg-data", &reg_data)) {
		dev_err(dev, "nvidia,reg-data missing, disabling emergency thermal reset\n");
		goto put_tt;
	}

	if (of_property_read_u32(tt_np, "nvidia,pinmux-id", &pinmux))
		pinmux = 0;

	of_node_put(tt_np);

	value = tegra_pmc_readl(PMC_SENSOR_CTRL);
	value |= PMC_SENSOR_CTRL_SCRATCH_WRITE;
	tegra_pmc_writel(value, PMC_SENSOR_CTRL);

	value = (reg_data << PMC_SCRATCH54_DATA_SHIFT) |
	      (reg_addr << PMC_SCRATCH54_ADDR_SHIFT);
	tegra_pmc_writel(value, PMC_SCRATCH54);

	value = 0;
	value |= PMC_SCRATCH55_RESET_TEGRA;
	value |= i2c_ctrl_id << PMC_SCRATCH55_CNTRL_ID_SHIFT;
	value |= pinmux << PMC_SCRATCH55_PINMUX_SHIFT;
	value |= pmu_i2c_addr << PMC_SCRATCH55_I2CSLV1_SHIFT;

	/* Calculate checksum of SCRATCH54, SCRATCH55 fields.
	 * Bits 23:16 will contain the checksum and are currently zero,
	 * so they are not added.
	 */
	checksum = reg_addr + reg_data + (value & 0xff) + ((value >> 8) & 0xff) +
		((value >> 24) & 0xff);
	checksum &= 0xff;
	checksum = 0x100 - checksum;

	value |= checksum << PMC_SCRATCH55_CHECKSUM_SHIFT;

	tegra_pmc_writel(value, PMC_SCRATCH55);

	value = tegra_pmc_readl(PMC_SENSOR_CTRL);
	value |= PMC_SENSOR_CTRL_ENABLE_RST;
	tegra_pmc_writel(value, PMC_SENSOR_CTRL);

	dev_info(dev, "PMC emergency thermal reset enabled\n");

	return;

put_i2c:
	of_node_put(i2c_np);

put_tt:
	of_node_put(tt_np);
}

static int tegra_pmc_parse_dt(struct tegra_pmc *pmc, struct device_node *np)
{
	u32 value, values[2];

	if (of_property_read_u32(np, "nvidia,suspend-mode", &value)) {
	} else {
		switch (value) {
		case 0:
			pmc->suspend_mode = TEGRA_SUSPEND_LP0;
			break;

		case 1:
			pmc->suspend_mode = TEGRA_SUSPEND_LP1;
			break;

		case 2:
			pmc->suspend_mode = TEGRA_SUSPEND_LP2;
			break;

		default:
			pmc->suspend_mode = TEGRA_SUSPEND_NONE;
			break;
		}
	}

	pmc->suspend_mode = tegra_pm_validate_suspend_mode(pmc->suspend_mode);

	if (of_property_read_u32(np, "nvidia,cpu-pwr-good-time", &value))
		pmc->suspend_mode = TEGRA_SUSPEND_NONE;

	pmc->cpu_good_time = value;

	if (of_property_read_u32(np, "nvidia,cpu-pwr-off-time", &value))
		pmc->suspend_mode = TEGRA_SUSPEND_NONE;

	pmc->cpu_off_time = value;

	if (of_property_read_u32_array(np, "nvidia,core-pwr-good-time",
				       values, ARRAY_SIZE(values)))
		pmc->suspend_mode = TEGRA_SUSPEND_NONE;

	pmc->core_osc_time = values[0];
	pmc->core_pmu_time = values[1];

	if (of_property_read_u32(np, "nvidia,core-pwr-off-time", &value))
		pmc->suspend_mode = TEGRA_SUSPEND_NONE;

	pmc->core_off_time = value;

	pmc->corereq_high = of_property_read_bool(np,
				"nvidia,core-power-req-active-high");

	pmc->sysclkreq_high = of_property_read_bool(np,
				"nvidia,sys-clock-req-active-high");

	pmc->combined_req = of_property_read_bool(np,
				"nvidia,combined-power-req");

	pmc->cpu_pwr_good_en = of_property_read_bool(np,
				"nvidia,cpu-pwr-good-en");

	return 0;
}

static void tegra_pmc_init(struct tegra_pmc *pmc)
{
	u32 value;

	/* Always enable CPU power request */
	value = tegra_pmc_readl(PMC_CNTRL);
	value |= PMC_CNTRL_CPU_PWRREQ_OE;
	tegra_pmc_writel(value, PMC_CNTRL);

	value = tegra_pmc_readl(PMC_CNTRL);

	if (pmc->sysclkreq_high)
		value &= ~PMC_CNTRL_SYSCLK_POLARITY;
	else
		value |= PMC_CNTRL_SYSCLK_POLARITY;

	if (!pmc->corereq_high)
		value |= PMC_CNTRL_PWRREQ_POLARITY;
	else
		value &= ~PMC_CNTRL_PWRREQ_POLARITY;

	/* configure the output polarity while the request is tristated */
	tegra_pmc_writel(value, PMC_CNTRL);

	/* now enable the request */
	value = tegra_pmc_readl(PMC_CNTRL);
	value |= PMC_CNTRL_SYSCLK_OE;
	tegra_pmc_writel(value, PMC_CNTRL);

#ifdef CONFIG_PM_SLEEP
	set_core_power_timers();
	tegra_pmc_syscore_init();
	tegra_pmc_wake_syscore_init();
#endif
}

static int tegra_pmc_probe(struct platform_device *pdev)
{
	void __iomem *base = pmc->base;
	struct resource *res;
	int err;

	err = tegra_pmc_parse_dt(pmc, pdev->dev.of_node);
	if (err < 0)
		return err;

	/* take over the memory region from the early initialization */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pmc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pmc->base))
		return PTR_ERR(pmc->base);

	iounmap(base);

	pmc->clk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(pmc->clk)) {
		err = PTR_ERR(pmc->clk);
		dev_err(&pdev->dev, "failed to get pclk: %d\n", err);
		return err;
	}

	tegra_pmc_init(pmc);

	tegra_pmc_init_tsense_reset(&pdev->dev);

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		err = tegra_powergate_debugfs_init();
		if (err < 0)
			return err;
	}

	err = register_restart_handler(&tegra_pmc_restart_handler);
	if (err) {
		dev_err(&pdev->dev, "unable to register restart handler, %d\n",
				err);
		return err;
	}

#ifdef CONFIG_PM_SLEEP
	pmc->suspend_notifier.notifier_call = tegra_pmc_suspend_notifier;
	register_pm_notifier(&pmc->suspend_notifier);
#endif

	pmc->dev = &pdev->dev;

	return 0;
}

static struct platform_driver tegra_pmc_driver = {
	.driver = {
		.name = "tegra-pmc",
		.suppress_bind_attrs = true,
		.of_match_table = tegra_pmc_match,
	},
	.probe = tegra_pmc_probe,
};
module_platform_driver(tegra_pmc_driver);

/*
 * Early initialization to allow access to registers in the very early boot
 * process.
 */
static int __init tegra_pmc_early_init(void)
{
	const struct of_device_id *match;
	struct device_node *np;
	struct resource regs;
	bool invert;
	u32 value;

	if (!soc_is_tegra())
		return 0;

	np = of_find_matching_node_and_match(NULL, tegra_pmc_match, &match);
	if (!np) {
		pr_warn("PMC device node not found, disabling powergating\n");

		regs.start = 0x7000e400;
		regs.end = 0x7000e7ff;
		regs.flags = IORESOURCE_MEM;

		pr_warn("Using memory region %pR\n", &regs);
	} else {
		pmc->soc = match->data;
	}

	if (of_address_to_resource(np, 0, &regs) < 0) {
		pr_err("failed to get PMC registers\n");
		return -ENXIO;
	}

	pmc->base = ioremap_nocache(regs.start, resource_size(&regs));
	if (!pmc->base) {
		pr_err("failed to map PMC registers\n");
		return -ENXIO;
	}

	mutex_init(&pmc->powergates_lock);

	invert = of_property_read_bool(np, "nvidia,invert-interrupt");

	value = tegra_pmc_readl(PMC_CNTRL);

	if (invert)
		value |= PMC_CNTRL_INTR_POLARITY;
	else
		value &= ~PMC_CNTRL_INTR_POLARITY;

	tegra_pmc_writel(value, PMC_CNTRL);

#ifdef CONFIG_PM_SLEEP
	tegra_lp0_wakeup.of_node = np;
	INIT_LIST_HEAD(&tegra_lp0_wakeup.wake_list);
#endif

	return 0;
}
early_initcall(tegra_pmc_early_init);
