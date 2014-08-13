/*
 * Spin Table SMP initialisation
 *
 * Copyright (C) 2013 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/cpu_ops.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/smp_spin_table.h>

extern void secondary_holding_pen(void);
volatile unsigned long secondary_holding_pen_release = INVALID_HWID;

static phys_addr_t cpu_release_addr[NR_CPUS];

#ifdef CONFIG_HOTPLUG_CPU
static struct spin_table_soc_ops *soc_ops;
#endif
/*
 * Write secondary_holding_pen_release in a way that is guaranteed to be
 * visible to all observers, irrespective of whether they're taking part
 * in coherency or not.  This is necessary for the hotplug code to work
 * reliably.
 */
static void write_pen_release(u64 val)
{
	void *start = (void *)&secondary_holding_pen_release;
	unsigned long size = sizeof(secondary_holding_pen_release);

	secondary_holding_pen_release = val;
	__flush_dcache_area(start, size);
}


static int smp_spin_table_cpu_init(struct device_node *dn, unsigned int cpu)
{
	/*
	 * Determine the address from which the CPU is polling.
	 */
	if (of_property_read_u64(dn, "cpu-release-addr",
				 &cpu_release_addr[cpu])) {
		pr_err("CPU %d: missing or invalid cpu-release-addr property\n",
		       cpu);

		return -1;
	}

	return 0;
}

static int smp_spin_table_cpu_prepare(unsigned int cpu)
{
	void **release_addr;

	if (!cpu_release_addr[cpu])
		return -ENODEV;

	release_addr = __va(cpu_release_addr[cpu]);

	/*
	 * We write the release address as LE regardless of the native
	 * endianess of the kernel. Therefore, any boot-loaders that
	 * read this address need to convert this address to the
	 * boot-loader's endianess before jumping. This is mandated by
	 * the boot protocol.
	 */
	release_addr[0] = (void *) cpu_to_le64(__pa(secondary_holding_pen));

	__flush_dcache_area(release_addr, sizeof(release_addr[0]));

	/*
	 * Send an event to wake up the secondary CPU.
	 */
	sev();

	return 0;
}

static int smp_spin_table_cpu_boot(unsigned int cpu)
{
#ifdef CONFIG_HOTPLUG_CPU
	if (soc_ops && soc_ops->cpu_on)
		soc_ops->cpu_on(cpu);
#endif
	/*
	 * Update the pen release flag.
	 */
	write_pen_release(cpu_logical_map(cpu));

	/*
	 * Send an event, causing the secondaries to read pen_release.
	 */
	sev();

	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static int smp_spin_table_cpu_disable(unsigned int cpu)
{
	/* Fail early if we don't have CPU_OFF support */
	if (!soc_ops || !soc_ops->cpu_off)
		return -EOPNOTSUPP;
	return 0;
}

static void smp_spin_table_cpu_die(unsigned int cpu)
{
	soc_ops->cpu_off(cpu);
}
#endif

const struct cpu_operations smp_spin_table_ops = {
	.name		= "spin-table",
	.cpu_init	= smp_spin_table_cpu_init,
	.cpu_prepare	= smp_spin_table_cpu_prepare,
	.cpu_boot	= smp_spin_table_cpu_boot,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_disable	= smp_spin_table_cpu_disable,
	.cpu_die	= smp_spin_table_cpu_die,
#endif
};

#ifdef CONFIG_HOTPLUG_CPU
void smp_spin_table_set_soc_ops(struct spin_table_soc_ops *ops)
{
	if (ops)
		soc_ops = ops;
}
#endif
