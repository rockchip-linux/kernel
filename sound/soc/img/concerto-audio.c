/*
 * Concerto audio card driver
 *
 * Copyright (C) 2014 Imagination Technologies Ltd.
 * Copyright (C) 2015 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <sound/soc.h>

#define CONCERTO_MAX_LINKS		5
#define CONCERTO_MAX_CODECS		10

struct concerto_codec_config {
	unsigned int fmt;
	struct device_node *np;
	const char *name;
};

struct concerto_audio_card {
	struct snd_soc_card card;
	struct snd_soc_dai_link dai_links[CONCERTO_MAX_LINKS];
	struct concerto_codec_config *i2s_out_codecs;
	unsigned int num_i2s_out_codecs;
	unsigned int i2s_out_fmt;
	struct snd_soc_dai *i2s_out_dai;
	struct concerto_codec_config *i2s_in_codecs;
	unsigned int num_i2s_in_codecs;
	unsigned int i2s_in_fmt;
	bool loopback_i2s_clk;
	struct clk *audio_pll;
	struct clk *mclk;
	struct regmap *periph_regs;
	struct regmap *top_regs;
};

static int concerto_mclk_configure(struct concerto_audio_card *cc,
				   unsigned long rate)
{
	unsigned long pll_rate, mclk_rate;
	int ret;

	switch (rate) {
	case 192000:
	case 96000:
	case 64000:
	case 48000:
	case 32000:
	case 16000:
	case 8000:
		pll_rate = 147456000;
		break;
	default:
		dev_err(cc->card.dev, "Unsupported rate: %lu\n", rate);
		return -EINVAL;
	}
	mclk_rate = rate * 256;

	ret = clk_set_rate(cc->audio_pll, pll_rate);
	if (ret < 0) {
		dev_err(cc->card.dev, "Failed to set PLL rate to %lu: %d\n",
			pll_rate, ret);
		return ret;
	}

	ret = clk_set_rate(cc->mclk, mclk_rate);
	if (ret < 0) {
		dev_err(cc->card.dev, "Failed to set MCLK rate to %lu: %d\n",
			mclk_rate, ret);
		return ret;
	}

	return 0;
}

static int concerto_i2s_out_init(struct snd_soc_pcm_runtime *rtd)
{
	struct concerto_audio_card *cc = snd_soc_card_get_drvdata(rtd->card);
	int ret, i;

	/*
	 * If the I2S out clocks are looped back to I2S in, the I2S out
	 * must be the clock master and must supply a continuous clock.
	 */
	if (cc->loopback_i2s_clk) {
		if ((cc->i2s_out_fmt & SND_SOC_DAIFMT_MASTER_MASK) !=
		    SND_SOC_DAIFMT_CBS_CFS) {
			dev_err(cc->card.dev, "I2S out must be clock master");
			return -EINVAL;
		}
		cc->i2s_out_fmt |= SND_SOC_DAIFMT_CONT;
	}

	cc->i2s_out_dai = rtd->cpu_dai;
	ret = snd_soc_dai_set_fmt(rtd->cpu_dai, cc->i2s_out_fmt);
	if (ret < 0)
		return ret;

	for (i = 0; i < cc->num_i2s_out_codecs; i++) {
		struct snd_soc_dai *codec = rtd->codec_dais[i];
		unsigned long mclk = clk_get_rate(cc->mclk);

		ret = snd_soc_dai_set_fmt(codec, cc->i2s_out_codecs[i].fmt);
		if (ret)
			return ret;

		ret = snd_soc_dai_set_sysclk(codec, 0, mclk, 0);
		if (ret)
			return ret;
	}

	return 0;
}

static int concerto_i2s_out_hw_params(struct snd_pcm_substream *st,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = st->private_data;
	struct concerto_audio_card *cc = snd_soc_card_get_drvdata(rtd->card);
	unsigned int i;
	int ret;

	ret = concerto_mclk_configure(cc, params_rate(params));
	if (ret < 0)
		return ret;

	for (i = 0; i < cc->num_i2s_out_codecs; i++) {
		unsigned long mclk = clk_get_rate(cc->mclk);

		ret = snd_soc_dai_set_sysclk(rtd->codec_dais[i], 0, mclk, 0);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static const struct snd_soc_ops concerto_i2s_out_ops = {
	.hw_params = concerto_i2s_out_hw_params,
};

#define CR_I2S_CTRL				0x88
#define CR_I2S_CTRL_CLK_SRC_MASK		0x3
#define CR_I2S_CTRL_CLK_SRC_NO_LOOPBACK		0x0
#define CR_I2S_CTRL_CLK_SRC_MFIO_LOOPBACK	0x1
#define CR_I2S_CTRL_CLK_SRC_LOCAL_LOOPBACK	0x2

static int concerto_i2s_in_init(struct snd_soc_pcm_runtime *rtd)
{
	struct concerto_audio_card *cc = snd_soc_card_get_drvdata(rtd->card);
	int ret, i;

	if (cc->loopback_i2s_clk && !cc->i2s_out_dai) {
		dev_err(cc->card.dev, "No I2S out DAI registered\n");
		return -EINVAL;
	}

	ret = snd_soc_dai_set_fmt(rtd->cpu_dai, cc->i2s_in_fmt);
	if (ret < 0)
		return ret;

	for (i = 0; i < cc->num_i2s_in_codecs; i++) {
		struct snd_soc_dai *codec = rtd->codec_dais[i];
		unsigned long mclk = clk_get_rate(cc->mclk);

		ret = snd_soc_dai_set_fmt(codec, cc->i2s_in_codecs[i].fmt);
		if (ret)
			return ret;

		ret = snd_soc_dai_set_sysclk(codec, 0, mclk, 0);
		if (ret)
			return ret;
	}

	if (cc->loopback_i2s_clk) {
		regmap_update_bits(cc->periph_regs, CR_I2S_CTRL,
				   CR_I2S_CTRL_CLK_SRC_MASK,
				   CR_I2S_CTRL_CLK_SRC_LOCAL_LOOPBACK);
	} else {
		regmap_update_bits(cc->periph_regs, CR_I2S_CTRL,
				   CR_I2S_CTRL_CLK_SRC_MASK,
				   CR_I2S_CTRL_CLK_SRC_NO_LOOPBACK);
	}

	return 0;
}

static int concerto_i2s_in_startup(struct snd_pcm_substream *st)
{
	struct snd_soc_pcm_runtime *rtd = st->private_data;
	struct concerto_audio_card *cc = snd_soc_card_get_drvdata(rtd->card);

	if (cc->loopback_i2s_clk)
		return pm_runtime_get_sync(cc->i2s_out_dai->dev);

	return 0;
}

static void concerto_i2s_in_shutdown(struct snd_pcm_substream *st)
{
	struct snd_soc_pcm_runtime *rtd = st->private_data;
	struct concerto_audio_card *cc = snd_soc_card_get_drvdata(rtd->card);

	if (cc->loopback_i2s_clk)
		pm_runtime_put(cc->i2s_out_dai->dev);
}

static int concerto_i2s_in_hw_params(struct snd_pcm_substream *st,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = st->private_data;
	struct concerto_audio_card *cc = snd_soc_card_get_drvdata(rtd->card);
	unsigned int i;
	int ret;

	ret = concerto_mclk_configure(cc, params_rate(params));
	if (ret < 0)
		return ret;

	for (i = 0; i < cc->num_i2s_in_codecs; i++) {
		unsigned long mclk = clk_get_rate(cc->mclk);

		ret = snd_soc_dai_set_sysclk(rtd->codec_dais[i], 0, mclk, 0);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static const struct snd_soc_ops concerto_i2s_in_ops = {
	.startup = concerto_i2s_in_startup,
	.shutdown = concerto_i2s_in_shutdown,
	.hw_params = concerto_i2s_in_hw_params,
};

#define CR_AUDIO_DAC_CTRL			0x40
#define CR_AUDIO_DAC_CTRL_PWR			BIT(0)
#define CR_AUDIO_DAC_CTRL_PWR_SEL		BIT(1)
#define CR_AUDIO_DAC_CTRL_MUTE			BIT(2)

#define CR_AUDIO_DAC_RESET			0x44
#define CR_AUDIO_DAC_RESET_SR			BIT(0)

#define CR_AUDIO_DAC_GTI_CTRL			0x48
#define CR_AUDIO_DAC_GTI_CTRL_ADDR_SHIFT	0
#define CR_AUDIO_DAC_GTI_CTRL_ADDR_MASK		0xfff
#define CR_AUDIO_DAC_GTI_CTRL_WE		BIT(12)
#define CR_AUDIO_DAC_GTI_CTRL_WDATA_SHIFT	13
#define CR_AUDIO_DAC_GTI_CTRL_WDATA_MASK	0xff

#define CR_AUDIO_DAC_GTI_OUT			0x4c
#define CR_AUDIO_DAC_GTI_OUT_RDATA_SHIFT	0
#define CR_AUDIO_DAC_GTI_OUT_RDATA_MASK		0xff

static int concerto_parallel_out_init(struct snd_soc_pcm_runtime *rtd)
{
	struct concerto_audio_card *cc = snd_soc_card_get_drvdata(rtd->card);

	regmap_update_bits(cc->top_regs, CR_AUDIO_DAC_CTRL,
			   CR_AUDIO_DAC_CTRL_PWR, CR_AUDIO_DAC_CTRL_PWR);

	usleep_range(10000, 11000);

	regmap_update_bits(cc->top_regs, CR_AUDIO_DAC_GTI_CTRL,
			   CR_AUDIO_DAC_GTI_CTRL_ADDR_MASK <<
			   CR_AUDIO_DAC_GTI_CTRL_ADDR_SHIFT,
			   1 << CR_AUDIO_DAC_GTI_CTRL_ADDR_SHIFT);

	regmap_update_bits(cc->top_regs, CR_AUDIO_DAC_GTI_CTRL,
			   CR_AUDIO_DAC_GTI_CTRL_WDATA_MASK <<
			   CR_AUDIO_DAC_GTI_CTRL_WDATA_SHIFT,
			   1 << CR_AUDIO_DAC_GTI_CTRL_WDATA_SHIFT);

	regmap_update_bits(cc->top_regs, CR_AUDIO_DAC_GTI_CTRL,
			   CR_AUDIO_DAC_GTI_CTRL_WE, CR_AUDIO_DAC_GTI_CTRL_WE);

	regmap_update_bits(cc->top_regs, CR_AUDIO_DAC_GTI_CTRL,
			   CR_AUDIO_DAC_GTI_CTRL_WE, 0);

	regmap_update_bits(cc->top_regs, CR_AUDIO_DAC_CTRL,
			   CR_AUDIO_DAC_CTRL_PWR, 0);

	return 0;
}

static unsigned int concerto_count_of_codecs(struct device_node *node)
{
	unsigned int i;

	for (i = 0; i < CONCERTO_MAX_CODECS; i++) {
		char prop[sizeof("codec-N")];

		snprintf(prop, sizeof(prop), "codec-%d", i);
		if (!of_get_child_by_name(node, prop))
			break;
	}

	return i;
}

static int concerto_parse_of_codecs(struct concerto_audio_card *cc,
				    struct device_node *node,
				    struct snd_soc_dai_link *link,
				    struct concerto_codec_config **configs,
				    bool codec_master)
{
	unsigned int i, num_codecs;
	bool found = !codec_master;

	num_codecs = concerto_count_of_codecs(node);
	if (num_codecs == 0)
		return 0;

	link->codecs = devm_kcalloc(cc->card.dev, num_codecs,
				    sizeof(*link->codecs), GFP_KERNEL);
	if (!link->codecs)
		return -ENOMEM;
	*configs = devm_kcalloc(cc->card.dev, num_codecs, sizeof(**configs),
				GFP_KERNEL);
	if (!*configs)
		return -ENOMEM;
	link->num_codecs = num_codecs;

	for (i = 0; i < num_codecs; i++) {
		struct concerto_codec_config *conf = &(*configs)[i];
		struct device_node *codec, *codec_dai;
		char node_name[sizeof("codec-N")];
		int ret;

		snprintf(node_name, sizeof(node_name), "codec-%d", i);
		codec = of_get_child_by_name(node, node_name);
		if (!codec)
			return -EINVAL;
		codec_dai = of_parse_phandle(codec, "sound-dai", 0);
		if (!codec_dai)
			return -EINVAL;
		ret = snd_soc_of_get_dai_name(codec, &link->codecs[i].dai_name);
		if (ret < 0)
			return ret;
		link->codecs[i].of_node = codec_dai;
		conf->np = codec_dai;
		conf->fmt = snd_soc_of_parse_daifmt(codec, NULL, NULL, NULL);
		of_property_read_string(codec, "name", &conf->name);
		/* Ensure there is only one clock master. */
		if ((conf->fmt & SND_SOC_DAIFMT_MASTER_MASK) ==
		    SND_SOC_DAIFMT_CBM_CFM) {
			if (found) {
				dev_err(cc->card.dev,
					"Multiple clock masters specified\n");
				return -EINVAL;
			} else {
				found = true;
			}
		}
	}

	return 0;
}

static int concerto_parse_of_i2s_out(struct concerto_audio_card *cc,
				     struct device_node *node,
				     struct snd_soc_dai_link *link)
{
	struct device_node *cpu, *cpu_dai;
	bool codec_master;
	unsigned int fmt;
	int ret;

	link->name = link->stream_name = "pistachio-i2s-out";
	cpu = of_get_child_by_name(node, "cpu");
	if (!cpu)
		return -EINVAL;
	cpu_dai = of_parse_phandle(cpu, "sound-dai", 0);
	if (!cpu_dai)
		return -EINVAL;
	link->cpu_of_node = cpu_dai;
	link->platform_of_node = cpu_dai;
	/* Flip the polarity of the clock format for the CPU side. */
	fmt = snd_soc_of_parse_daifmt(cpu, NULL, NULL, NULL);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		cc->i2s_out_fmt = (fmt & ~SND_SOC_DAIFMT_MASTER_MASK) |
			SND_SOC_DAIFMT_CBS_CFS;
		codec_master = false;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		cc->i2s_out_fmt = (fmt & ~SND_SOC_DAIFMT_MASTER_MASK) |
			SND_SOC_DAIFMT_CBM_CFM;
		codec_master = true;
		break;
	default:
		dev_err(cc->card.dev, "Invalid i2s-out format: %x\n", fmt);
		return -EINVAL;
	}

	ret = concerto_parse_of_codecs(cc, node, link, &cc->i2s_out_codecs,
				       codec_master);
	if (ret < 0)
		return ret;
	if (link->num_codecs == 0) {
		link->codec_dai_name = "snd-soc-dummy-dai";
		link->codec_name = "snd-soc-dummy";
	}
	cc->num_i2s_out_codecs = link->num_codecs;

	link->init = concerto_i2s_out_init;
	link->ops = &concerto_i2s_out_ops;

	return 0;
}

static int concerto_parse_of_i2s_in(struct concerto_audio_card *cc,
				    struct device_node *node,
				    struct snd_soc_dai_link *link)
{
	struct device_node *cpu, *cpu_dai;
	unsigned int fmt;
	int ret;

	link->name = link->stream_name = "pistachio-i2s-in";
	cpu = of_get_child_by_name(node, "cpu");
	if (!cpu)
		return -EINVAL;
	cpu_dai = of_parse_phandle(cpu, "sound-dai", 0);
	if (!cpu_dai)
		return -EINVAL;
	link->cpu_of_node = cpu_dai;
	link->platform_of_node = cpu_dai;
	/* i2s-in is always the clock slave */
	fmt = snd_soc_of_parse_daifmt(cpu, NULL, NULL, NULL);
	cc->i2s_in_fmt = (fmt & ~SND_SOC_DAIFMT_MASTER_MASK) |
		SND_SOC_DAIFMT_CBM_CFM;
	cc->loopback_i2s_clk = of_property_read_bool(cpu,
						     "img,i2s-clock-loopback");

	ret = concerto_parse_of_codecs(cc, node, link, &cc->i2s_in_codecs,
				       !cc->loopback_i2s_clk);
	if (ret < 0)
		return ret;
	if (link->num_codecs == 0) {
		link->codec_dai_name = "snd-soc-dummy-dai";
		link->codec_name = "snd-soc-dummy";
	}
	cc->num_i2s_in_codecs = link->num_codecs;

	link->init = concerto_i2s_in_init;
	link->ops = &concerto_i2s_in_ops;

	return 0;
}

static int concerto_parse_of_spdif_out(struct concerto_audio_card *cc,
				       struct device_node *node,
				       struct snd_soc_dai_link *link)
{
	struct device_node *cpu, *cpu_dai;

	link->name = link->stream_name = "pistachio-spdif-out";
	cpu = of_get_child_by_name(node, "cpu");
	if (!cpu)
		return -EINVAL;
	cpu_dai = of_parse_phandle(cpu, "sound-dai", 0);
	if (!cpu_dai)
		return -EINVAL;
	link->cpu_of_node = cpu_dai;
	link->platform_of_node = cpu_dai;
	link->codec_dai_name = "snd-soc-dummy-dai";
	link->codec_name = "snd-soc-dummy";

	return 0;
}

static int concerto_parse_of_spdif_in(struct concerto_audio_card *cc,
				      struct device_node *node,
				      struct snd_soc_dai_link *link)
{
	struct device_node *cpu, *cpu_dai;

	link->name = link->stream_name = "pistachio-spdif-in";
	cpu = of_get_child_by_name(node, "cpu");
	if (!cpu)
		return -EINVAL;
	cpu_dai = of_parse_phandle(cpu, "sound-dai", 0);
	if (!cpu_dai)
		return -EINVAL;
	link->cpu_of_node = cpu_dai;
	link->platform_of_node = cpu_dai;
	link->codec_dai_name = "snd-soc-dummy-dai";
	link->codec_name = "snd-soc-dummy";

	return 0;
}

static int concerto_parse_of_parallel_out(struct concerto_audio_card *cc,
					  struct device_node *node,
					  struct snd_soc_dai_link *link)
{
	struct device_node *cpu, *cpu_dai;

	link->name = link->stream_name = "pistachio-parallel-out";
	cpu = of_get_child_by_name(node, "cpu");
	if (!cpu)
		return -EINVAL;
	cpu_dai = of_parse_phandle(cpu, "sound-dai", 0);
	if (!cpu)
		return -EINVAL;
	link->cpu_of_node = cpu_dai;
	link->platform_of_node = cpu_dai;
	link->codec_dai_name = "snd-soc-dummy-dai";
	link->codec_name = "snd-soc-dummy";
	link->init = concerto_parallel_out_init;

	return 0;
}

struct concerto_dai_link_type {
	const char *of_name;
	int (*of_parse)(struct concerto_audio_card *, struct device_node *,
			struct snd_soc_dai_link *);
};

static const struct concerto_dai_link_type concerto_dai_link_types[] = {
	{
		.of_name = "i2s-out",
		.of_parse = concerto_parse_of_i2s_out,
	},
	{
		.of_name = "i2s-in",
		.of_parse = concerto_parse_of_i2s_in,
	},
	{
		.of_name = "spdif-out",
		.of_parse = concerto_parse_of_spdif_out,
	},
	{
		.of_name = "spdif-in",
		.of_parse = concerto_parse_of_spdif_in,
	},
	{
		.of_name = "parallel-out",
		.of_parse = concerto_parse_of_parallel_out,
	},
};

static int concerto_parse_of(struct concerto_audio_card *cc,
			     struct device_node *node)
{
	const struct concerto_dai_link_type *t;
	struct snd_soc_codec_conf *conf;
	struct device_node *np;
	unsigned int i, link;
	int ret;

	ret = snd_soc_of_parse_card_name(&cc->card, "model");
	if (ret < 0)
		return ret;

	if (of_property_read_bool(node, "widgets")) {
		ret = snd_soc_of_parse_audio_simple_widgets(&cc->card,
							    "widgets");
		if (ret)
			return ret;
	}

	if (of_property_read_bool(node, "routing")) {
		ret = snd_soc_of_parse_audio_routing(&cc->card, "routing");
		if (ret)
			return ret;
	}

	for (i = 0, link = 0; i < ARRAY_SIZE(concerto_dai_link_types); i++) {
		t = &concerto_dai_link_types[i];
		np = of_get_child_by_name(node, t->of_name);
		if (!np)
			continue;
		ret = t->of_parse(cc, np, &cc->dai_links[link]);
		if (ret < 0)
			return ret;
		link++;
	}
	cc->card.dai_link = cc->dai_links;
	cc->card.num_links = link;

	cc->card.codec_conf = devm_kcalloc(cc->card.dev,
					   cc->num_i2s_out_codecs +
					   cc->num_i2s_in_codecs,
					   sizeof(*cc->card.codec_conf),
					   GFP_KERNEL);
	if (!cc->card.codec_conf)
		return -ENOMEM;
	cc->card.num_configs = cc->num_i2s_out_codecs + cc->num_i2s_in_codecs;

	conf = &cc->card.codec_conf[0];
	for (i = 0; i < cc->num_i2s_out_codecs; i++, conf++) {
		conf->of_node = cc->i2s_out_codecs[i].np;
		conf->name_prefix = cc->i2s_out_codecs[i].name;
	}
	for (i = 0; i < cc->num_i2s_in_codecs; i++, conf++) {
		conf->of_node = cc->i2s_in_codecs[i].np;
		conf->name_prefix = cc->i2s_in_codecs[i].name;
	}

	return 0;
}

static void concerto_unref_of(struct concerto_audio_card *cc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cc->dai_links); i++) {
		struct snd_soc_dai_link *link = &cc->dai_links[i];
		struct device_node *np;

		np = (struct device_node *)link->cpu_of_node;
		if (np)
			of_node_put(np);

		if (link->codecs) {
			unsigned int j;

			for (j = 0; j < link->num_codecs; j++) {
				np = (struct device_node *)
					link->codecs[j].of_node;
				if (np)
					of_node_put(np);
			}
		}
	}
}

static int concerto_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct concerto_audio_card *cc;
	int ret;

	cc = devm_kzalloc(dev, sizeof(*cc), GFP_KERNEL);
	if (!cc)
		return -ENOMEM;
	cc->card.owner = THIS_MODULE;
	cc->card.dev = dev;
	platform_set_drvdata(pdev, &cc->card);
	snd_soc_card_set_drvdata(&cc->card, cc);

	cc->audio_pll = devm_clk_get(dev, "audio_pll");
	if (IS_ERR(cc->audio_pll))
		return PTR_ERR(cc->audio_pll);

	cc->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(cc->mclk))
		return PTR_ERR(cc->mclk);
	ret = clk_prepare_enable(cc->mclk);
	if (ret < 0)
		return ret;

	cc->periph_regs = syscon_regmap_lookup_by_phandle(np, "img,cr-periph");
	if (IS_ERR(cc->periph_regs)) {
		ret = PTR_ERR(cc->periph_regs);
		goto disable_mclk;
	}

	cc->top_regs = syscon_regmap_lookup_by_phandle(np, "img,cr-top");
	if (IS_ERR(cc->top_regs)) {
		ret = PTR_ERR(cc->top_regs);
		goto disable_mclk;
	}

	ret = concerto_parse_of(cc, np);
	if (ret < 0)
		goto unref_of;

	ret = devm_snd_soc_register_card(dev, &cc->card);
	if (ret < 0)
		goto unref_of;

	return 0;

unref_of:
	concerto_unref_of(cc);
disable_mclk:
	clk_disable_unprepare(cc->mclk);

	return ret;
}

static int concerto_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct concerto_audio_card *cc = snd_soc_card_get_drvdata(card);

	concerto_unref_of(cc);
	clk_disable_unprepare(cc->mclk);

	return 0;
}

static const struct of_device_id concerto_audio_card_of_match[] = {
	{ .compatible = "google,concerto-audio", },
	{},
};
MODULE_DEVICE_TABLE(of, concerto_audio_card_of_match);

static struct platform_driver concerto_audio_card_driver = {
	.driver = {
		.name = "concerto-audio-card",
		.of_match_table = concerto_audio_card_of_match,
	},
	.probe = concerto_audio_probe,
	.remove = concerto_audio_remove,
};
module_platform_driver(concerto_audio_card_driver);

MODULE_DESCRIPTION("Concerto audio card driver");
MODULE_AUTHOR("Andrew Bresticker <abrestic@chromium.org>");
MODULE_LICENSE("GPL v2");
