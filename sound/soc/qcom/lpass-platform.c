/*
 * Copyright (c) 2010-2011,2013-2015 The Linux Foundation. All rights reserved.
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
#include <linux/io.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include "lpass-lpaif-ipq806x.h"
#include "lpass.h"

#define DRV_NAME			"lpass-platform"

#define LPASS_PLATFORM_CHANNELS_MIN	1
#define LPASS_PLATFORM_CHANNELS_MAX	8
#define LPASS_PLATFORM_PERIODS_MIN	2
#define LPASS_PLATFORM_PERIODS_MAX	2
#define LPASS_PLATFORM_RATE_MIN		8000
#define LPASS_PLATFORM_RATE_MAX		192000

static struct snd_pcm_hardware lpass_platform_hardware = {
	.info			=	SNDRV_PCM_INFO_INTERLEAVED |
					SNDRV_PCM_INFO_PAUSE |
					SNDRV_PCM_INFO_RESUME,
	.formats		=	SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S24 |
					SNDRV_PCM_FMTBIT_S32,
	.rates			=	SNDRV_PCM_RATE_8000_192000,
	.rate_min		=	LPASS_PLATFORM_RATE_MIN,
	.rate_max		=	LPASS_PLATFORM_RATE_MAX,
	.channels_min		=	LPASS_PLATFORM_CHANNELS_MIN,
	.channels_max		=	LPASS_PLATFORM_CHANNELS_MAX,
	.periods_min		=	LPASS_PLATFORM_PERIODS_MIN,
	.periods_max		=	LPASS_PLATFORM_PERIODS_MAX,
};

static int lpass_platform_pcmops_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct lpass_data *drvdata =
		snd_soc_platform_get_drvdata(soc_runtime->platform);
	int ret;

	lpass_platform_hardware.buffer_bytes_max = drvdata->lpm_size;
	lpass_platform_hardware.period_bytes_max =
		drvdata->lpm_size / LPASS_PLATFORM_PERIODS_MIN;
	lpass_platform_hardware.period_bytes_min =
		drvdata->lpm_size / LPASS_PLATFORM_PERIODS_MAX;
	snd_soc_set_runtime_hwparams(substream, &lpass_platform_hardware);

	runtime->dma_bytes = drvdata->lpm_size;

	ret = snd_pcm_hw_constraint_integer(runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		dev_err(soc_runtime->dev, "%s() setting constraints failed: %d\n",
				__func__, ret);
		return -EINVAL;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	return 0;
}

static int lpass_platform_pcmops_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct lpass_data *drvdata =
		snd_soc_platform_get_drvdata(soc_runtime->platform);
	snd_pcm_format_t format = params_format(params);
	unsigned int channels = params_channels(params);
	unsigned int regval;
	int bitwidth;
	int ret;

	bitwidth = snd_pcm_format_width(format);
	if (bitwidth < 0) {
		dev_err(soc_runtime->dev, "%s() invalid bit width given\n",
				__func__);
		return bitwidth;
	}

	regval = 0;
	regval |= LPAIF_RDMACTL_BURSTEN_INCR4;
	regval |= LPAIF_RDMACTL_AUDINTF_MI2S;
	regval |= LPAIF_RDMACTL_FIFOWM_8;

	switch (bitwidth) {
	case 16:
		switch (channels) {
		case 1:
		case 2:
			regval |= LPAIF_RDMACTL_WPSCNT_ONE;
			break;
		case 4:
			regval |= LPAIF_RDMACTL_WPSCNT_TWO;
			break;
		case 6:
			regval |= LPAIF_RDMACTL_WPSCNT_THREE;
			break;
		case 8:
			regval |= LPAIF_RDMACTL_WPSCNT_FOUR;
			break;
		default:
			dev_err(soc_runtime->dev, "%s() invalid PCM config given: bw=%u, ch=%u\n",
					__func__, bitwidth, channels);
			return -EINVAL;
		}
		break;
	case 24:
	case 32:
		switch (channels) {
		case 1:
			regval |= LPAIF_RDMACTL_WPSCNT_ONE;
			break;
		case 2:
			regval |= LPAIF_RDMACTL_WPSCNT_TWO;
			break;
		case 4:
			regval |= LPAIF_RDMACTL_WPSCNT_FOUR;
			break;
		case 6:
			regval |= LPAIF_RDMACTL_WPSCNT_SIX;
			break;
		case 8:
			regval |= LPAIF_RDMACTL_WPSCNT_EIGHT;
			break;
		default:
			dev_err(soc_runtime->dev, "%s() invalid PCM config given: bw=%u, ch=%u\n",
					__func__, bitwidth, channels);
			return -EINVAL;
		}
		break;
	default:
		dev_err(soc_runtime->dev, "%s() invalid PCM config given: bw=%u, ch=%u\n",
				__func__, bitwidth, channels);
		return -EINVAL;
	}

	ret = regmap_write(drvdata->lpaif_map,
			LPAIF_RDMACTL_REG(LPAIF_RDMA_CHAN_MI2S), regval);
	if (ret) {
		dev_err(soc_runtime->dev, "%s() error writing to rdmactl reg: %d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

static int lpass_platform_pcmops_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct lpass_data *drvdata =
		snd_soc_platform_get_drvdata(soc_runtime->platform);
	int ret;

	ret = regmap_write(drvdata->lpaif_map,
			LPAIF_RDMACTL_REG(LPAIF_RDMA_CHAN_MI2S), 0);
	if (ret) {
		dev_err(soc_runtime->dev, "%s() error writing to rdmactl reg: %d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

static int lpass_platform_pcmops_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct lpass_data *drvdata =
		snd_soc_platform_get_drvdata(soc_runtime->platform);
	unsigned int reg, mask, val;
	int ret;

	reg = LPAIF_RDMABASE_REG(LPAIF_RDMA_CHAN_MI2S);
	val = runtime->dma_addr;
	ret = regmap_write(drvdata->lpaif_map, reg, val);
	if (ret) {
		dev_err(soc_runtime->dev, "%s() error writing to rdmabase reg: %d\n",
				__func__, ret);
		return ret;
	}

	reg = LPAIF_RDMABUFF_REG(LPAIF_RDMA_CHAN_MI2S);
	val = (snd_pcm_lib_buffer_bytes(substream) >> 2) - 1;
	ret = regmap_write(drvdata->lpaif_map, reg, val);
	if (ret) {
		dev_err(soc_runtime->dev, "%s() error writing to rdmabuff reg: %d\n",
				__func__, ret);
		return ret;
	}

	reg = LPAIF_RDMAPER_REG(LPAIF_RDMA_CHAN_MI2S);
	val = (snd_pcm_lib_period_bytes(substream) >> 2) - 1;
	ret = regmap_write(drvdata->lpaif_map, reg, val);
	if (ret) {
		dev_err(soc_runtime->dev, "%s() error writing to rdmaper reg: %d\n",
				__func__, ret);
		return ret;
	}

	reg = LPAIF_RDMACTL_REG(LPAIF_RDMA_CHAN_MI2S);
	mask = LPAIF_RDMACTL_ENABLE_MASK;
	val = LPAIF_RDMACTL_ENABLE_ON;
	ret = regmap_update_bits(drvdata->lpaif_map, reg, mask, val);
	if (ret) {
		dev_err(soc_runtime->dev, "%s() error writing to rdmactl reg: %d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

static int lpass_platform_pcmops_trigger(struct snd_pcm_substream *substream,
		int cmd)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct lpass_data *drvdata =
		snd_soc_platform_get_drvdata(soc_runtime->platform);
	unsigned int reg, mask, val;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* clear status before enabling interrupts */
		reg = LPAIF_IRQCLEAR_REG(LPAIF_IRQ_PORT_HOST);
		val = LPAIF_IRQ_ALL(LPAIF_RDMA_CHAN_MI2S);
		ret = regmap_write(drvdata->lpaif_map, reg, val);
		if (ret) {
			dev_err(soc_runtime->dev, "%s() error writing to irqclear reg: %d\n",
					__func__, ret);
			return ret;
		}

		reg = LPAIF_IRQEN_REG(LPAIF_IRQ_PORT_HOST);
		mask = LPAIF_IRQ_ALL(LPAIF_RDMA_CHAN_MI2S);
		val = LPAIF_IRQ_ALL(LPAIF_RDMA_CHAN_MI2S);
		ret = regmap_update_bits(drvdata->lpaif_map, reg, mask, val);
		if (ret) {
			dev_err(soc_runtime->dev, "%s() error writing to irqen reg: %d\n",
					__func__, ret);
			return ret;
		}

		reg = LPAIF_RDMACTL_REG(LPAIF_RDMA_CHAN_MI2S);
		mask = LPAIF_RDMACTL_ENABLE_MASK;
		val = LPAIF_RDMACTL_ENABLE_ON;
		ret = regmap_update_bits(drvdata->lpaif_map, reg, mask, val);
		if (ret) {
			dev_err(soc_runtime->dev, "%s() error writing to rdmactl reg: %d\n",
					__func__, ret);
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		reg = LPAIF_RDMACTL_REG(LPAIF_RDMA_CHAN_MI2S);
		mask = LPAIF_RDMACTL_ENABLE_MASK;
		val = LPAIF_RDMACTL_ENABLE_OFF;
		ret = regmap_update_bits(drvdata->lpaif_map, reg, mask, val);
		if (ret) {
			dev_err(soc_runtime->dev, "%s() error writing to rdmactl reg: %d\n",
					__func__, ret);
			return ret;
		}

		reg = LPAIF_IRQEN_REG(LPAIF_IRQ_PORT_HOST);
		mask = LPAIF_IRQ_ALL(LPAIF_RDMA_CHAN_MI2S);
		val = 0;
		ret = regmap_update_bits(drvdata->lpaif_map, reg, mask, val);
		if (ret) {
			dev_err(soc_runtime->dev, "%s() error writing to irqen reg: %d\n",
					__func__, ret);
			return ret;
		}
		break;
	}

	return 0;
}

static snd_pcm_uframes_t lpass_platform_pcmops_pointer(
		struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct lpass_data *drvdata =
			snd_soc_platform_get_drvdata(soc_runtime->platform);
	unsigned int base_addr, curr_addr;
	int ret;

	ret = regmap_read(drvdata->lpaif_map,
			LPAIF_RDMABASE_REG(LPAIF_RDMA_CHAN_MI2S), &base_addr);
	if (ret) {
		dev_err(soc_runtime->dev, "%s() error reading from rdmabase reg: %d\n",
				__func__, ret);
		return ret;
	}

	ret = regmap_read(drvdata->lpaif_map,
			LPAIF_RDMACURR_REG(LPAIF_RDMA_CHAN_MI2S), &curr_addr);
	if (ret) {
		dev_err(soc_runtime->dev, "%s() error reading from rdmacurr reg: %d\n",
				__func__, ret);
		return ret;
	}

	return bytes_to_frames(substream->runtime, curr_addr - base_addr);
}

static struct snd_pcm_ops lpass_platform_pcm_ops = {
	.open		= lpass_platform_pcmops_open,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= lpass_platform_pcmops_hw_params,
	.hw_free	= lpass_platform_pcmops_hw_free,
	.prepare	= lpass_platform_pcmops_prepare,
	.trigger	= lpass_platform_pcmops_trigger,
	.pointer	= lpass_platform_pcmops_pointer,
};

static irqreturn_t lpass_platform_lpaif_irq(int irq, void *data)
{
	struct snd_pcm_substream *substream = data;
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct lpass_data *drvdata =
		snd_soc_platform_get_drvdata(soc_runtime->platform);
	const u32 status_offset = LPAIF_IRQSTAT_REG(LPAIF_IRQ_PORT_HOST);
	const u32 clear_offset = LPAIF_IRQCLEAR_REG(LPAIF_IRQ_PORT_HOST);
	const u32 period_bitmask = LPAIF_IRQ_PER(LPAIF_RDMA_CHAN_MI2S);
	const u32 xrun_bitmask = LPAIF_IRQ_XRUN(LPAIF_RDMA_CHAN_MI2S);
	const u32 error_bitmask = LPAIF_IRQ_ERR(LPAIF_RDMA_CHAN_MI2S);
	const u32 chan_bitmask = LPAIF_IRQ_ALL(LPAIF_RDMA_CHAN_MI2S);
	u32 interrupts;
	irqreturn_t ret = IRQ_NONE;

	interrupts = ioread32(drvdata->lpaif + status_offset) & chan_bitmask;

	if (likely(interrupts & period_bitmask)) {
		iowrite32(period_bitmask, drvdata->lpaif + clear_offset);
		snd_pcm_period_elapsed(substream);
		ret = IRQ_HANDLED;
	}

	if (unlikely(interrupts & xrun_bitmask)) {
		iowrite32(xrun_bitmask, drvdata->lpaif + clear_offset);
		dev_warn(soc_runtime->dev, "%s() xrun warning\n", __func__);
		snd_pcm_stop(substream, SNDRV_PCM_STATE_XRUN);
		ret = IRQ_HANDLED;
	}

	if (unlikely(interrupts & error_bitmask)) {
		iowrite32(error_bitmask, drvdata->lpaif + clear_offset);
		dev_err(soc_runtime->dev, "%s() bus access error\n", __func__);
		snd_pcm_stop(substream, SNDRV_PCM_STATE_DISCONNECTED);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static int lpass_platform_alloc_buffer(struct snd_pcm_substream *substream,
		struct snd_soc_pcm_runtime *soc_runtime)
{
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	struct lpass_data *drvdata =
		snd_soc_platform_get_drvdata(soc_runtime->platform);

	if (atomic_cmpxchg(&drvdata->lpm_lock, 0, 1)) {
		dev_err(soc_runtime->dev, "%s() buffer already in use\n",
				__func__);
		return -ENOMEM;
	}

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = soc_runtime->dev;
	buf->private_data = NULL;
	buf->area = (unsigned char __force *)drvdata->lpm;
	buf->addr = drvdata->lpm_phys;
	buf->bytes = drvdata->lpm_size;

	return 0;
}

static void lpass_platform_free_buffer(struct snd_pcm_substream *substream,
		struct snd_soc_pcm_runtime *soc_runtime)
{
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	struct lpass_data *drvdata =
		snd_soc_platform_get_drvdata(soc_runtime->platform);

	if (buf->area == (unsigned char __force *)drvdata->lpm) {
		buf->area = NULL;
		atomic_dec(&drvdata->lpm_lock);
	} else {
		dev_warn(soc_runtime->dev, "%s() attempting to free invalid buffer\n",
				__func__);
	}
}

static int lpass_platform_pcm_new(struct snd_soc_pcm_runtime *soc_runtime)
{
	struct snd_pcm *pcm = soc_runtime->pcm;
	struct snd_pcm_substream *substream =
		pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	struct lpass_data *drvdata =
		snd_soc_platform_get_drvdata(soc_runtime->platform);
	int ret;

	ret = lpass_platform_alloc_buffer(substream, soc_runtime);
	if (ret)
		return ret;

	ret = devm_request_irq(soc_runtime->dev, drvdata->lpaif_irq,
			lpass_platform_lpaif_irq, IRQF_TRIGGER_RISING,
			"lpass-irq-lpaif", substream);
	if (ret) {
		dev_err(soc_runtime->dev, "%s() irq request failed: %d\n",
				__func__, ret);
		goto err_buf;
	}

	/* ensure audio hardware is disabled */
	ret = regmap_write(drvdata->lpaif_map,
			LPAIF_IRQEN_REG(LPAIF_IRQ_PORT_HOST), 0);
	if (ret) {
		dev_err(soc_runtime->dev, "%s() error writing to irqen reg: %d\n",
				__func__, ret);
		return ret;
	}
	ret = regmap_write(drvdata->lpaif_map,
			LPAIF_RDMACTL_REG(LPAIF_RDMA_CHAN_MI2S), 0);
	if (ret) {
		dev_err(soc_runtime->dev, "%s() error writing to rdmactl reg: %d\n",
				__func__, ret);
		return ret;
	}

	return 0;

err_buf:
	lpass_platform_free_buffer(substream, soc_runtime);
	return ret;
}

static void lpass_platform_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream =
		pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;

	lpass_platform_free_buffer(substream, soc_runtime);
}

static struct snd_soc_platform_driver lpass_platform_driver = {
	.pcm_new	= lpass_platform_pcm_new,
	.pcm_free	= lpass_platform_pcm_free,
	.ops		= &lpass_platform_pcm_ops,
};

int asoc_qcom_lpass_platform_register(struct platform_device *pdev)
{
	struct lpass_data *drvdata = platform_get_drvdata(pdev);
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lpass-lpm");
	if (!res) {
		dev_err(&pdev->dev, "%s() error getting resource\n", __func__);
		return -ENODEV;
	}

	drvdata->lpm = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR((unsigned char __force *)drvdata->lpm)) {
		dev_err(&pdev->dev, "%s() error mapping lpm resource: %ld\n",
				__func__,
				PTR_ERR((unsigned char __force *)drvdata->lpm));
		return PTR_ERR((unsigned char __force *)drvdata->lpm);
	}
	drvdata->lpm_phys = res->start;
	drvdata->lpm_size = resource_size(res);
	atomic_set(&drvdata->lpm_lock, 0);

	drvdata->lpaif_irq = platform_get_irq_byname(pdev, "lpass-irq-lpaif");
	if (drvdata->lpaif_irq < 0) {
		dev_err(&pdev->dev, "%s() error getting irq handle: %d\n",
				__func__, drvdata->lpaif_irq);
		return -ENODEV;
	}

	return devm_snd_soc_register_platform(&pdev->dev,
			&lpass_platform_driver);
}
EXPORT_SYMBOL(asoc_qcom_lpass_platform_register);

MODULE_DESCRIPTION("QTi LPASS Platform Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
