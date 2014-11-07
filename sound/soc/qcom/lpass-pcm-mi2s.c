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
#include <linux/dma-mapping.h>
#include <sound/soc.h>
#include "lpass-lpaif.h"
#include "lpass-cpu-mi2s.h"

#define DRV_NAME			"lpass-pcm-mi2s"

/* MI2S HW params */
#define LPASS_MI2S_PERIOD_SIZE		(8064)
#define LPASS_MI2S_PERIODS_MIN		(2)
#define LPASS_MI2S_PERIODS_MAX		(4)
#define LPASS_MI2S_BUFF_SIZE_MIN	(LPASS_MI2S_PERIOD_SIZE * \
						LPASS_MI2S_PERIODS_MIN)
#define LPASS_MI2S_BUFF_SIZE_MAX	(LPASS_MI2S_PERIOD_SIZE * \
						LPASS_MI2S_PERIODS_MAX)

static struct snd_pcm_hardware lpass_pcm_mi2s_hardware_playback = {
	.info			=	SNDRV_PCM_INFO_MMAP |
					SNDRV_PCM_INFO_BLOCK_TRANSFER |
					SNDRV_PCM_INFO_MMAP_VALID |
					SNDRV_PCM_INFO_INTERLEAVED |
					SNDRV_PCM_INFO_PAUSE |
					SNDRV_PCM_INFO_RESUME,
	.formats		=	SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S24 |
					SNDRV_PCM_FMTBIT_S32,
	.rates			=	SNDRV_PCM_RATE_8000_192000,
	.rate_min		=	8000,
	.rate_max		=	192000,
	.channels_min		=	1,
	.channels_max		=	8,
	.buffer_bytes_max	=	LPASS_MI2S_BUFF_SIZE_MAX,
	.period_bytes_max	=	LPASS_MI2S_PERIOD_SIZE,
	.period_bytes_min	=	LPASS_MI2S_PERIOD_SIZE,
	.periods_min		=	LPASS_MI2S_PERIODS_MIN,
	.periods_max		=	LPASS_MI2S_PERIODS_MAX,
	.fifo_size		=	0,
};

static int lpass_lpaif_int_enable(struct lpass_cpu_mi2s_data *prtd)
{
	u32 intr_val;

	/* clear status before enabling interrupt */
	writel(LPAIF_DMAIRQ_ALL(LPAIF_DMA_RD_CH_MI2S),
			prtd->base + LPAIF_DMAIRQ_CLEAR(LPAIF_IRQ_RECV_HOST));

	intr_val = readl(prtd->base + LPAIF_DMAIRQ_EN(LPAIF_IRQ_RECV_HOST));
	intr_val |= LPAIF_DMAIRQ_ALL(LPAIF_DMA_RD_CH_MI2S);
	writel(intr_val, prtd->base + LPAIF_DMAIRQ_EN(LPAIF_IRQ_RECV_HOST));

	return 0;
}

static int lpass_lpaif_int_disable(struct lpass_cpu_mi2s_data *prtd)
{
	u32 intr_val;

	intr_val = readl(prtd->base + LPAIF_DMAIRQ_EN(LPAIF_IRQ_RECV_HOST));
	intr_val &= ~LPAIF_DMAIRQ_ALL(LPAIF_DMA_RD_CH_MI2S);
	writel(intr_val, prtd->base + LPAIF_DMAIRQ_EN(LPAIF_IRQ_RECV_HOST));

	return 0;
}

static int lpass_lpaif_cfg_dma(struct snd_soc_platform *platform,
		dma_addr_t src_start, int buffer_size, int period_size,
		int channels, int bitwidth)
{
	struct lpass_cpu_mi2s_data *prtd =
		snd_soc_platform_get_drvdata(platform);
	int ret = 0;
	u32 cfg;

	lpass_lpaif_int_enable(prtd);

	writel(src_start, prtd->base +
			LPAIF_DMA_BASEADDR(LPAIF_DMA_RD_CH_MI2S));
	writel((buffer_size >> 2) - 1, prtd->base +
			LPAIF_DMA_BUFFLEN(LPAIF_DMA_RD_CH_MI2S));
	writel((period_size >> 2) - 1, prtd->base +
			LPAIF_DMA_PERLEN(LPAIF_DMA_RD_CH_MI2S));

	cfg = 0;
	cfg |= LPAIF_DMACTL_BURST_EN;
	cfg |= LPAIF_DMACTL_AUDIO_INTF_MI2S;
	cfg |= LPAIF_DMACTL_FIFO_WM_8;
	cfg |= LPAIF_DMACTL_ENABLE;

	switch (bitwidth) {
	case 16:
		switch (channels) {
		case 1:
		case 2:
			cfg |= LPAIF_DMACTL_WPSCNT_SINGLE;
			break;
		case 4:
			cfg |= LPAIF_DMACTL_WPSCNT_DOUBLE;
			break;
		case 6:
			cfg |= LPAIF_DMACTL_WPSCNT_TRIPLE;
			break;
		case 8:
			cfg |= LPAIF_DMACTL_WPSCNT_QUAD;
			break;
		default:
			dev_err(platform->dev, "%s: invalid PCM config given: bw=%u, ch=%u\n",
					__func__, bitwidth, channels);
			ret = -EINVAL;
		}
		break;
	case 24:
	case 32:
		switch (channels) {
		case 1:
			cfg |= LPAIF_DMACTL_WPSCNT_SINGLE;
			break;
		case 2:
			cfg |= LPAIF_DMACTL_WPSCNT_DOUBLE;
			break;
		case 4:
			cfg |= LPAIF_DMACTL_WPSCNT_QUAD;
			break;
		case 6:
			cfg |= LPAIF_DMACTL_WPSCNT_SIXPACK;
			break;
		case 8:
			cfg |= LPAIF_DMACTL_WPSCNT_OCTAL;
			break;
		default:
			dev_err(platform->dev, "%s: invalid PCM config given: bw=%u, ch=%u\n",
					__func__, bitwidth, channels);
			ret = -EINVAL;
		}
		break;
	default:
		dev_err(platform->dev, "%s: invalid PCM config given: bw=%u, ch=%u\n",
				__func__, bitwidth, channels);
		ret = -EINVAL;
	}

	if (!ret)
		writel(cfg, prtd->base + LPAIF_DMA_CTL(LPAIF_DMA_RD_CH_MI2S));

	return ret;
}

static void lpass_lpaif_dma_stop_clear(struct lpass_cpu_mi2s_data *prtd)
{
	writel(0x0, prtd->base + LPAIF_DMA_CTL(LPAIF_DMA_RD_CH_MI2S));
}

static void lpass_lpaif_dma_stop(struct lpass_cpu_mi2s_data *prtd)
{
	u32 cfg;

	cfg = readl(prtd->base + LPAIF_DMA_CTL(LPAIF_DMA_RD_CH_MI2S));
	cfg &= ~LPAIF_DMACTL_ENABLE;
	writel(cfg, prtd->base + LPAIF_DMA_CTL(LPAIF_DMA_RD_CH_MI2S));
}

static int lpass_pcm_mi2s_alloc_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_soc_pcm_runtime *prtd = substream->private_data;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size;

	size = lpass_pcm_mi2s_hardware_playback.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev, size,
			&buf->addr, GFP_KERNEL);
	if (!buf->area) {
		dev_err(prtd->dev, "%s: Could not allocate DMA buffer\n",
				__func__);
		return -ENOMEM;
	}
	buf->bytes = size;

	return 0;
}

static void lpass_pcm_mi2s_free_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;

	substream = pcm->streams[stream].substream;
	buf = &substream->dma_buffer;
	if (buf->area) {
		dma_free_coherent(pcm->card->dev, buf->bytes, buf->area,
				buf->addr);
	}
	buf->area = NULL;
}

static irqreturn_t lpass_pcm_mi2s_irq(int irq, void *data)
{
	irqreturn_t ret = IRQ_NONE;
	u32 intrsrc;
	u32 has_xrun, pending;
	struct snd_pcm_substream *substream = data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct lpass_cpu_mi2s_data *prtd =
		snd_soc_platform_get_drvdata(soc_prtd->platform);

	intrsrc = readl(prtd->base + LPAIF_DMAIRQ_STAT(LPAIF_IRQ_RECV_HOST));
	intrsrc &= LPAIF_DMAIRQ_ALL(LPAIF_DMA_RD_CH_MI2S);
	writel(intrsrc, prtd->base + LPAIF_DMAIRQ_CLEAR(LPAIF_IRQ_RECV_HOST));

	pending = intrsrc & LPAIF_DMAIRQ_ALL(LPAIF_DMA_RD_CH_MI2S);
	has_xrun = pending & LPAIF_DMAIRQ_XRUN(LPAIF_DMA_RD_CH_MI2S);

	if (unlikely(has_xrun) && snd_pcm_running(substream)) {
		dev_warn(soc_prtd->dev, "%s: xrun warning\n", __func__);
		snd_pcm_stop(substream, SNDRV_PCM_STATE_XRUN);
		pending &= ~LPAIF_DMAIRQ_XRUN(LPAIF_DMA_RD_CH_MI2S);
		ret = IRQ_HANDLED;
	}

	if (pending & LPAIF_DMAIRQ_PER(LPAIF_DMA_RD_CH_MI2S)) {
		if (++prtd->period_index >= runtime->periods)
			prtd->period_index = 0;
		snd_pcm_period_elapsed(substream);
		pending &= ~LPAIF_DMAIRQ_PER(LPAIF_DMA_RD_CH_MI2S);
		ret = IRQ_HANDLED;
	}

	if (pending & LPAIF_DMAIRQ_XRUN(LPAIF_DMA_RD_CH_MI2S)) {
		snd_pcm_period_elapsed(substream);
		dev_warn(soc_prtd->dev, "%s: xrun warning\n", __func__);
		ret = IRQ_HANDLED;
	}

	if (pending & LPAIF_DMAIRQ_ERR(LPAIF_DMA_RD_CH_MI2S)) {
		dev_err(soc_prtd->dev, "%s: Bus access error\n", __func__);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static int lpass_pcm_mi2s_open(struct snd_pcm_substream *substream)
{
	int ret;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct lpass_cpu_mi2s_data *prtd =
		snd_soc_platform_get_drvdata(soc_prtd->platform);

	prtd->prepare_start = 0;
	prtd->period_index = 0;

	runtime->dma_bytes = lpass_pcm_mi2s_hardware_playback.buffer_bytes_max;
	snd_soc_set_runtime_hwparams(substream,
			&lpass_pcm_mi2s_hardware_playback);

	ret = snd_pcm_hw_constraint_integer(runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		dev_err(soc_prtd->dev,
				"%s: snd_pcm_hw_constraint_integer failed\n",
				__func__);
		return -EINVAL;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	return 0;
}

static int lpass_pcm_mi2s_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct lpass_cpu_mi2s_data *prtd =
		snd_soc_platform_get_drvdata(soc_prtd->platform);

	lpass_lpaif_dma_stop_clear(prtd);
	lpass_lpaif_int_disable(prtd);

	return 0;
}

static int lpass_pcm_mi2s_prepare(struct snd_pcm_substream *substream)
{
	int ret;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct lpass_cpu_mi2s_data *prtd =
		snd_soc_platform_get_drvdata(soc_prtd->platform);

	/* xrun recovery */
	if (prtd->prepare_start)
		return 0;

	lpass_lpaif_dma_stop(prtd);
	prtd->prepare_start = 1;

	ret = lpass_lpaif_cfg_dma(soc_prtd->platform, runtime->dma_addr,
			snd_pcm_lib_buffer_bytes(substream),
			snd_pcm_lib_period_bytes(substream), runtime->channels,
			runtime->sample_bits);
	if (ret) {
		dev_err(soc_prtd->dev, "%s: Error in configuring DMA\n",
				__func__);
		ret = -EINVAL;
		goto err;
	}

	return 0;

err:
	return ret;
}

static int lpass_pcm_mi2s_mmap(struct snd_pcm_substream *substream,
		struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_coherent(substream->pcm->card->dev, vma,
		runtime->dma_area, runtime->dma_addr, runtime->dma_bytes);
}

static snd_pcm_uframes_t lpass_pcm_mi2s_pointer(
		struct snd_pcm_substream *substream)
{
	snd_pcm_uframes_t offset;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct lpass_cpu_mi2s_data *prtd =
		snd_soc_platform_get_drvdata(soc_prtd->platform);

	offset = prtd->period_index * runtime->period_size;

	return offset >= (runtime->buffer_size) ? 0 : offset;
}

static struct snd_pcm_ops lpass_pcm_mi2s_soc_ops = {
	.open		= lpass_pcm_mi2s_open,
	.close		= lpass_pcm_mi2s_close,
	.prepare	= lpass_pcm_mi2s_prepare,
	.mmap		= lpass_pcm_mi2s_mmap,
	.pointer	= lpass_pcm_mi2s_pointer,
	.ioctl		= snd_pcm_lib_ioctl,
};

static int lpass_pcm_mi2s_soc_new(struct snd_soc_pcm_runtime *soc_prtd)
{
	int ret;
	struct snd_card *card = soc_prtd->card->snd_card;
	struct snd_pcm *pcm = soc_prtd->pcm;
	struct lpass_cpu_mi2s_data *prtd =
		snd_soc_platform_get_drvdata(soc_prtd->platform);
	struct snd_pcm_substream *substream =
		pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &card->dev->coherent_dma_mask;

	ret = lpass_pcm_mi2s_alloc_buffer(pcm, SNDRV_PCM_STREAM_PLAYBACK);
	if (ret)
		return ret;

	ret = request_irq(prtd->irqnum, lpass_pcm_mi2s_irq,
			IRQF_TRIGGER_RISING, "lpass-lpaif-intr", substream);
	if (ret) {
		dev_err(soc_prtd->dev, "%s: irq resource request failed\n",
				__func__);
		goto err;
	}
	prtd->irq_acquired = 1;

	return 0;

err:
	lpass_pcm_mi2s_free_buffer(pcm, SNDRV_PCM_STREAM_PLAYBACK);
	return ret;
}

static void lpass_pcm_mi2s_soc_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream =
		pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct lpass_cpu_mi2s_data *prtd =
		snd_soc_platform_get_drvdata(soc_prtd->platform);

	lpass_pcm_mi2s_free_buffer(pcm, SNDRV_PCM_STREAM_PLAYBACK);

	disable_irq(prtd->irqnum);
	if (prtd->irq_acquired)
		free_irq(prtd->irqnum, NULL);
	prtd->irq_acquired = 0;
}

static struct snd_soc_platform_driver lpass_pcm_mi2s_soc_driver = {
	.pcm_new	= lpass_pcm_mi2s_soc_new,
	.pcm_free	= lpass_pcm_mi2s_soc_free,
	.ops		= &lpass_pcm_mi2s_soc_ops,
};

int lpass_pcm_mi2s_platform_register(struct device *dev)
{
	return devm_snd_soc_register_platform(dev, &lpass_pcm_mi2s_soc_driver);
}

MODULE_DESCRIPTION("QCOM LPASS MI2S PLATFORM DRIVER");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
