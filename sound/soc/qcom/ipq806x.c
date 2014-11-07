/*
 * Copyright (c) 2010-2011,2013-2014 The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <sound/soc.h>

#define DRV_NAME	"ipq806x-machine"

static struct snd_soc_dai_link ipq806x_machine_dai_link = {
	.name		= "IPQ806x Media1",
	.stream_name	= "MultiMedia1",
};

static int ipq806x_populate_dai_link_of_nodes(struct snd_soc_card *card)
{
	int ret;
	struct device *cdev = card->dev;
	struct snd_soc_dai_link *dai_link = card->dai_link;
	struct device_node *np;

	np = of_parse_phandle(cdev->of_node, "cpu", 0);
	if (!np) {
		dev_err(cdev, "%s: no phandle for cpu-dai driver\n", __func__);
		return -ENODEV;
	}
	dai_link->cpu_of_node = np;
	dai_link->platform_of_node = np;

	np = of_parse_phandle(cdev->of_node, "codec", 0);
	if (!np) {
		dev_err(cdev, "%s: no phandle for codec driver\n", __func__);
		return -ENODEV;
	}
	dai_link->codec_of_node = np;

	ret = of_property_read_string(cdev->of_node, "codec-dai",
			&dai_link->codec_dai_name);
	if (ret) {
		dev_err(cdev, "%s: no name for codec dai\n", __func__);
		return ret;
	}

	return 0;
}

static struct snd_soc_card ipq806x_machine_soc_card = {
	.name	= DRV_NAME,
	.dev	= NULL,
};

static int ipq806x_machine_platform_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &ipq806x_machine_soc_card;

	if (card->dev) {
		dev_err(&pdev->dev, "soundcard instance already created\n");
		return -ENODEV;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);

	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret) {
		dev_err(&pdev->dev, "parse card name failed, err:%d\n", ret);
		return ret;
	}

	card->dai_link	= &ipq806x_machine_dai_link;
	card->num_links	= 1;

	ret = ipq806x_populate_dai_link_of_nodes(card);
	if (ret) {
		dev_err(&pdev->dev, "could not resolve dai links, err:%d\n",
				ret);
		return ret;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret == -EPROBE_DEFER) {
		card->dev = NULL;
		return ret;
	} else if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id ipq806x_machine_dt_match[]  = {
	{ .compatible = "qcom,ipq806x-snd-card", },
	{},
};

static struct platform_driver ipq806x_machine_platform_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = ipq806x_machine_dt_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = ipq806x_machine_platform_probe,
};
module_platform_driver(ipq806x_machine_platform_driver);

MODULE_DESCRIPTION("QCOM IPQ806X MACHINE DRIVER");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, ipq806x_asoc_machine_of_match);
