/*
 * Qualcomm ARM SoC AHCI SATA platform driver
 *
 * based on the AHCI SATA platform driver by Jeff Garzik and Anton Vorontsov
 *
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/libata.h>
#include <linux/ahci_platform.h>
#include "ahci.h"

static const struct ata_port_info qcom_ahci_port_info = {
	.flags		= AHCI_FLAG_COMMON,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &ahci_platform_ops,
};

static int qcom_ahci_probe(struct platform_device *pdev)
{
	struct ahci_host_priv *hpriv;
	struct clk *rxoob_clk;
	int rc;

	hpriv = ahci_platform_get_resources(pdev);
	if (IS_ERR(hpriv))
		return PTR_ERR(hpriv);

	/* Try and set the rxoob clk to 100Mhz */
	rxoob_clk = of_clk_get_by_name(pdev->dev.of_node, "rxoob");
	if (IS_ERR(rxoob_clk))
		return PTR_ERR(rxoob_clk);

	rc = clk_set_rate(rxoob_clk, 100000000);
	if (rc)
		return rc;

	rc = ahci_platform_enable_resources(hpriv);
	if (rc)
		return rc;

	rc = ahci_platform_init_host(pdev, hpriv, &qcom_ahci_port_info, 0, 0);
	if (rc)
		goto disable_resources;

	return 0;
disable_resources:
	ahci_platform_disable_resources(hpriv);
	return rc;
}

static const struct of_device_id qcom_ahci_of_match[] = {
	{ .compatible = "qcom,msm-ahci", },
	{},
};
MODULE_DEVICE_TABLE(of, qcom_ahci_of_match);

static struct platform_driver qcom_ahci_driver = {
	.probe = qcom_ahci_probe,
	.remove = ata_platform_remove_one,
	.driver = {
		.name = "qcom_ahci_qcom",
		.owner = THIS_MODULE,
		.of_match_table = qcom_ahci_of_match,
	},
};
module_platform_driver(qcom_ahci_driver);

MODULE_DESCRIPTION("Qualcomm AHCI SATA platform driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("ahci:qcom");
