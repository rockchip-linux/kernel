/*
 * rk_pcm.c  --  ALSA SoC ROCKCHIP PCM Audio Layer Platform driver
 *
 * Driver for rockchip pcm audio
 *
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include "rk_pcm.h"
/*
 * small buffer case: support up to 48k 8ch 32bit audio
 * period_max: 2048 frames, channel_max: 8, fmt_max: 4bytes
 * period_bytes_max = 2048 * 8 * 4 = 64 * 1024
 * buffer_bytes_max = 4 * period_bytes_max
 * normal buffer case: support up to 192k 8ch 32bit audio
 * period_max: 16384 frames, channel_max: 8, fmt_max: 4bytes
 * period_bytes_max = 16384 * 8 * 4 = 512 * 1024
 * buffer_bytes_max = 4 * period_bytes_max
 */
#ifdef CONFIG_SND_RK_PCM_SMALL_BUFFER
#define BUFFER_BYTES_MAX	(256 * 1024)
#define PERIOD_BYTES_MAX	(64 * 1024)
#else
#define BUFFER_BYTES_MAX	(2 * 1024 * 1024)
#define PERIOD_BYTES_MAX	(512 * 1024)
#endif

static const struct snd_pcm_hardware rockchip_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				    SNDRV_PCM_INFO_BLOCK_TRANSFER |
				    SNDRV_PCM_INFO_MMAP |
				    SNDRV_PCM_INFO_MMAP_VALID |
				    SNDRV_PCM_INFO_PAUSE |
				    SNDRV_PCM_INFO_RESUME,
	.formats		=   SNDRV_PCM_FMTBIT_S24_LE |
				    SNDRV_PCM_FMTBIT_S20_3LE |
				    SNDRV_PCM_FMTBIT_S16_LE,
	.channels_min		= 2,
	.channels_max		= 8,
	.buffer_bytes_max	= BUFFER_BYTES_MAX,
	.period_bytes_min	= 64,
	.period_bytes_max	= PERIOD_BYTES_MAX,
	.periods_min		= 3,
	.periods_max		= 128,
	.fifo_size		= 16,
};

static const struct snd_dmaengine_pcm_config rockchip_dmaengine_pcm_config = {
	.pcm_hardware = &rockchip_pcm_hardware,
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.compat_filter_fn = NULL,
	.prealloc_buffer_size = BUFFER_BYTES_MAX,
};

int rockchip_pcm_platform_register(struct device *dev)
{
	return snd_dmaengine_pcm_register(dev, &rockchip_dmaengine_pcm_config,
			SND_DMAENGINE_PCM_FLAG_COMPAT|
			SND_DMAENGINE_PCM_FLAG_NO_RESIDUE);
}
EXPORT_SYMBOL_GPL(rockchip_pcm_platform_register);

int rockchip_pcm_platform_unregister(struct device *dev)
{
	snd_dmaengine_pcm_unregister(dev);
	return 0;
}
EXPORT_SYMBOL_GPL(rockchip_pcm_platform_unregister);

/* Module information */
MODULE_AUTHOR("rockchip");
MODULE_DESCRIPTION("ROCKCHIP PCM ASoC Interface");
MODULE_LICENSE("GPL");
