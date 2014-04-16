/*
 * Copyright (C) 2014 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef MEMORY_TEGRA_MC_H
#define MEMORY_TEGRA_MC_H

#include <linux/types.h>

struct page;

struct latency_allowance {
	unsigned int reg;
	unsigned int shift;
	unsigned int mask;
	unsigned int def;
};

struct smmu_enable {
	unsigned int reg;
	unsigned int bit;
};

struct tegra_mc_client {
	unsigned int id;
	const char *name;
	unsigned int swgroup;

	unsigned int fifo_size;

	struct smmu_enable smmu;
	struct latency_allowance latency;
};

struct tegra_smmu_swgroup {
	unsigned int swgroup;
	unsigned int reg;
};

struct tegra_smmu_ops {
	void (*flush_dcache)(struct page *page, unsigned long offset,
			     size_t size);
};

struct tegra_smmu_soc {
	const struct tegra_mc_client *clients;
	unsigned int num_clients;

	bool supports_round_robin_arbitration;
	bool supports_request_limit;

	const struct tegra_smmu_swgroup *swgroups;
	unsigned int num_swgroups;

	unsigned int num_address_bits;
	unsigned int num_asids;

	const struct tegra_smmu_ops *ops;
};

struct tegra_mc_soc {
	const struct tegra_mc_client *clients;
	unsigned int num_clients;

	unsigned int atom_size;

	const struct tegra_smmu_soc *smmu;
};

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
extern const struct tegra_mc_soc tegra30_mc_soc;
#endif

#ifdef CONFIG_ARCH_TEGRA_114_SOC
extern const struct tegra_mc_soc tegra114_mc_soc;
#endif

#ifdef CONFIG_ARCH_TEGRA_124_SOC
extern const struct tegra_mc_soc tegra124_mc_soc;
#endif

#ifdef CONFIG_ARCH_TEGRA_132_SOC
extern const struct tegra_mc_soc tegra132_mc_soc;
#endif

#endif /* MEMORY_TEGRA_MC_H */
