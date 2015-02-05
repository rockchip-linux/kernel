/*
 * rt5677-spi.c  --  RT5677 ALSA SoC audio codec driver
 *
 * Copyright 2013 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_qos.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/acpi.h>

#include <sound/soc.h>

#include "rt5677-spi.h"

#define SPI_BURST_LEN		240
#define SPI_HEADER		5
#define SPI_READ_FREQ		1000000

#define RT5677_SPI_WRITE_BURST	0x5
#define RT5677_SPI_READ_BURST	0x4
#define RT5677_SPI_WRITE_32	0x3
#define RT5677_SPI_READ_32	0x2
#define RT5677_SPI_WRITE_16	0x1
#define RT5677_SPI_READ_16	0x0

#define RT5677_MIC_BUF_ADDR		0x60030000
#define RT5677_MODEL_ADDR		0x5FFC9800
#define RT5677_MIC_BUF_BYTES		(0x20000 - sizeof(u32))
#define RT5677_MIC_BUF_FIRST_READ_SIZE	0x10000

static struct spi_device *g_spi;
static DEFINE_MUTEX(spi_mutex);

struct rt5677_dsp {
	struct device *dev;
	struct delayed_work copy_work;
	struct mutex dma_lock;
	struct snd_pcm_substream *substream;
	size_t dma_offset;	/* zero-based offset into runtime->dma_area */
	size_t avail_bytes;	/* number of new bytes since last period */
	u32 mic_read_offset;	/* zero-based offset into DSP's mic buffer */
	bool new_hotword;	/* a new hotword is fired */
};

static const struct snd_pcm_hardware rt5677_spi_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_NO_PERIOD_WAKEUP,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= RT5677_MIC_BUF_BYTES / 2,
	.periods_min		= 2,
	.periods_max		= 2,
	.channels_min		= 1,
	.channels_max		= 1,
	.buffer_bytes_max	= RT5677_MIC_BUF_BYTES,
};

static int rt5677_spi_dai_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	return 0;
}

static struct snd_soc_dai_ops rt5677_spi_dai_ops = {
	.hw_params = rt5677_spi_dai_hw_params,
};

static struct snd_soc_dai_driver rt5677_spi_dai = {
	/* The DAI name "rt5677-dsp-cpu-dai" is not used. The actual DAI name
	 * registered with ASoC is the name of the device "spi-RT5677AA:00",
	 * because we only have one DAI. See snd_soc_register_dais().
	 */
	.name = "rt5677-dsp-cpu-dai",
	.id = 0,
	.capture = {
		.stream_name = "DSP Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &rt5677_spi_dai_ops,
};

/* PCM for streaming audio from the DSP buffer */
static int rt5677_spi_pcm_open(struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, &rt5677_spi_pcm_hardware);
	return 0;
}

static int rt5677_spi_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5677_dsp *rt5677_dsp =
			snd_soc_platform_get_drvdata(rtd->platform);

	cancel_delayed_work_sync(&rt5677_dsp->copy_work);
	return 0;
}

static int rt5677_spi_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5677_dsp *rt5677_dsp =
			snd_soc_platform_get_drvdata(rtd->platform);
	int ret;

	mutex_lock(&rt5677_dsp->dma_lock);
	ret = snd_pcm_lib_alloc_vmalloc_buffer(substream,
			params_buffer_bytes(hw_params));
	rt5677_dsp->substream = substream;
	mutex_unlock(&rt5677_dsp->dma_lock);

	return ret;
}

static int rt5677_spi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5677_dsp *rt5677_dsp =
			snd_soc_platform_get_drvdata(rtd->platform);

	mutex_lock(&rt5677_dsp->dma_lock);
	rt5677_dsp->substream = 0;
	mutex_unlock(&rt5677_dsp->dma_lock);

	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int rt5677_spi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5677_dsp *rt5677_dsp =
			snd_soc_platform_get_drvdata(rtd->platform);

	rt5677_dsp->dma_offset = 0;
	rt5677_dsp->avail_bytes = 0;
	return 0;
}

static snd_pcm_uframes_t rt5677_spi_pcm_pointer(
		struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5677_dsp *rt5677_dsp =
		snd_soc_platform_get_drvdata(rtd->platform);

	return bytes_to_frames(runtime, rt5677_dsp->dma_offset);
}

static int rt5677_spi_mic_write_offset(u32 *mic_write_offset)
{
	int ret;
	/* Grab the first 4 bytes that hold the write pointer on the
	 * dsp, and check to make sure that it points somewhere inside the
	 * buffer.
	 */
	ret = rt5677_spi_read(RT5677_MIC_BUF_ADDR, mic_write_offset,
			sizeof(u32));
	if (ret)
		return ret;
	/* Adjust the offset so that it's zero-based */
	*mic_write_offset = *mic_write_offset - sizeof(u32);
	return *mic_write_offset < RT5677_MIC_BUF_BYTES ? 0 : -EFAULT;
}

/*
 * Copy a block of audio samples from the DSP mic buffer to the dma_area of
 * the pcm runtime. The receiving buffer may wrap around.
 * @begin: start offset of the block to copy, in bytes.
 * @end:   offset of the first byte after the block to copy, must be greater
 *         than begin.
 *
 * Return: Zero if successful, or a negative error code on failure.
 */
static int rt5677_spi_append_data(struct rt5677_dsp *rt5677_dsp,
		u32 begin, u32 end)
{
	struct snd_pcm_runtime *runtime = rt5677_dsp->substream->runtime;
	size_t bytes_per_frame = frames_to_bytes(runtime, 1);
	size_t first_chunk_len, second_chunk_len;
	int ret;

	if (begin >= end || runtime->dma_bytes < 2 * bytes_per_frame) {
		dev_err(rt5677_dsp->dev,
			"Invalid copy from (%u, %u), dma_area size %zu\n",
			begin, end, runtime->dma_bytes);
		return -EINVAL;
	}

	/* If the incoming chunk is too big for the receiving buffer, only the
	 * last "receiving buffer size - one frame" bytes are copied.
	 */
	if (end - begin > runtime->dma_bytes - bytes_per_frame)
		begin = end - (runtime->dma_bytes - bytes_per_frame);

	/* May need to split to two chunks, calculate the size of each */
	first_chunk_len = end - begin;
	second_chunk_len = 0;
	if (rt5677_dsp->dma_offset + first_chunk_len > runtime->dma_bytes) {
		/* Receiving buffer wrapped around */
		second_chunk_len = first_chunk_len;
		first_chunk_len = runtime->dma_bytes - rt5677_dsp->dma_offset;
		second_chunk_len -= first_chunk_len;
	}

	/* Copy first chunk */
	ret = rt5677_spi_read(RT5677_MIC_BUF_ADDR + sizeof(u32) + begin,
			runtime->dma_area + rt5677_dsp->dma_offset,
			first_chunk_len);
	if (ret)
		return ret;
	rt5677_dsp->dma_offset += first_chunk_len;
	if (rt5677_dsp->dma_offset == runtime->dma_bytes)
		rt5677_dsp->dma_offset = 0;

	/* Copy second chunk */
	if (second_chunk_len) {
		ret = rt5677_spi_read(RT5677_MIC_BUF_ADDR + sizeof(u32) +
				begin + first_chunk_len, runtime->dma_area,
				second_chunk_len);
		if (!ret)
			rt5677_dsp->dma_offset = second_chunk_len;
	}
	return ret;
}

/*
 * A delayed work that streams audio samples from the DSP mic buffer to the
 * dma_area of the pcm runtime via SPI.
 */
static void rt5677_spi_copy_work(struct work_struct *work)
{
	struct rt5677_dsp *rt5677_dsp =
		container_of(work, struct rt5677_dsp, copy_work.work);
	struct snd_pcm_runtime *runtime;
	u32 mic_write_offset;
	size_t bytes_copied, period_bytes;
	int ret = 0;

	/* Ensure runtime->dma_area buffer does not go away while copying. */
	mutex_lock(&rt5677_dsp->dma_lock);
	if (!rt5677_dsp->substream) {
		dev_err(rt5677_dsp->dev, "No pcm substream\n");
		goto done;
	}

	runtime = rt5677_dsp->substream->runtime;

	if (rt5677_spi_mic_write_offset(&mic_write_offset)) {
		dev_err(rt5677_dsp->dev, "No mic_write_offset\n");
		goto done;
	}

	/* If this is the first time that we've asked for streaming data after
	 * a hotword is fired, we should start reading from the previous 2
	 * seconds of audio from wherever the mic_write_offset is currently.
	 */
	if (rt5677_dsp->new_hotword) {
		rt5677_dsp->new_hotword = false;
		/* See if buffer wraparound happens */
		if (mic_write_offset < RT5677_MIC_BUF_FIRST_READ_SIZE)
			rt5677_dsp->mic_read_offset = RT5677_MIC_BUF_BYTES -
					(RT5677_MIC_BUF_FIRST_READ_SIZE -
					mic_write_offset);
		else
			rt5677_dsp->mic_read_offset = mic_write_offset -
					RT5677_MIC_BUF_FIRST_READ_SIZE;
	}

	/* Copy all new samples from DSP's mic buffer to dma_area */
	if (rt5677_dsp->mic_read_offset < mic_write_offset) {
		/* One chunk in DSP's mic buffer */
		ret |= rt5677_spi_append_data(rt5677_dsp,
				rt5677_dsp->mic_read_offset, mic_write_offset);
		bytes_copied = mic_write_offset - rt5677_dsp->mic_read_offset;
	} else if (rt5677_dsp->mic_read_offset > mic_write_offset) {
		/* Wrap around, two chunks in DSP's mic buffer */
		ret |= rt5677_spi_append_data(rt5677_dsp,
				rt5677_dsp->mic_read_offset,
				RT5677_MIC_BUF_BYTES);
		ret |= rt5677_spi_append_data(rt5677_dsp, 0, mic_write_offset);
		bytes_copied = RT5677_MIC_BUF_BYTES -
				rt5677_dsp->mic_read_offset + mic_write_offset;
	}
	if (ret) {
		dev_err(rt5677_dsp->dev, "Copy failed %d\n", ret);
		goto done;
	}

	rt5677_dsp->mic_read_offset = mic_write_offset;
	rt5677_dsp->avail_bytes += bytes_copied;
	period_bytes = snd_pcm_lib_period_bytes(rt5677_dsp->substream);

	if (rt5677_dsp->avail_bytes >= period_bytes) {
		snd_pcm_period_elapsed(rt5677_dsp->substream);
		rt5677_dsp->avail_bytes = 0;
	}
	/* TODO benzh: use better delay time based on period_bytes */
	schedule_delayed_work(&rt5677_dsp->copy_work, msecs_to_jiffies(5));
done:
	mutex_unlock(&rt5677_dsp->dma_lock);
}

struct page *rt5677_spi_pcm_page(struct snd_pcm_substream *substream,
		unsigned long offset)
{
	return snd_pcm_lib_get_vmalloc_page(substream, offset);
}

static struct snd_pcm_ops rt5677_spi_pcm_ops = {
	.open		= rt5677_spi_pcm_open,
	.close		= rt5677_spi_pcm_close,
	.hw_params	= rt5677_spi_hw_params,
	.hw_free	= rt5677_spi_hw_free,
	.prepare	= rt5677_spi_prepare,
	.pointer	= rt5677_spi_pcm_pointer,
	.mmap		= snd_pcm_lib_mmap_vmalloc,
	.page		= rt5677_spi_pcm_page,
};

static int rt5677_spi_pcm_probe(struct snd_soc_platform *platform)
{
	struct rt5677_dsp *rt5677_dsp;

	rt5677_dsp = devm_kzalloc(platform->dev, sizeof(*rt5677_dsp),
			GFP_KERNEL);
	rt5677_dsp->dev = &g_spi->dev;
	mutex_init(&rt5677_dsp->dma_lock);
	INIT_DELAYED_WORK(&rt5677_dsp->copy_work, rt5677_spi_copy_work);

	snd_soc_platform_set_drvdata(platform, rt5677_dsp);
	return 0;
}

static struct snd_soc_platform_driver rt5677_spi_platform = {
	.probe = rt5677_spi_pcm_probe,
	.ops = &rt5677_spi_pcm_ops,
};

static const struct snd_soc_component_driver rt5677_spi_dai_component = {
	.name		= "rt5677-spi-dai",
};

/* Read DSP memory using SPI. Addr and len have to be multiples of 16-bits. */
int rt5677_spi_read(u32 addr, void *rxbuf, size_t len)
{
	unsigned int i, end, offset = 0;
	int status = 0;
	struct spi_transfer t[2];
	struct spi_message m;
	u8 *rx_buf;
	u8 buf[SPI_BURST_LEN + SPI_HEADER + 4];
	u8 spi_cmd;
	u8 *rx_data = rxbuf;

	if (!g_spi)
		return -ENODEV;

	rx_buf = buf + SPI_HEADER + 4;
	memset(t, 0, sizeof(t));
	t[0].tx_buf = buf;
	t[0].len = SPI_HEADER + 4;
	t[0].speed_hz = SPI_READ_FREQ;
	t[1].rx_buf = rx_buf;
	t[1].speed_hz = SPI_READ_FREQ;
	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	while (offset < len) {
		/* TODO benzh: Clean up the read length selection logic */
		switch (min(len - offset, (size_t)(addr + offset) & 0x7)) {
		case 4:
			spi_cmd = RT5677_SPI_READ_32;
			end = 4;
			break;
		case 2:
		case 6:
			spi_cmd = RT5677_SPI_READ_16;
			end = 2;
			break;
		case 0:
			if (offset + SPI_BURST_LEN <= len) {
				spi_cmd = RT5677_SPI_READ_BURST;
				end = SPI_BURST_LEN;
				break;
			} else if (offset + 4 <= len) {
				spi_cmd = RT5677_SPI_READ_32;
				end = 4;
				break;
			} else if (offset + 2 <= len) {
				spi_cmd = RT5677_SPI_READ_16;
				end = 2;
				break;
			}
			/* Fall through to default error case */
		default:
			pr_err("Bad section alignment\n");
			return -EACCES;
		}

		buf[0] = spi_cmd;
		buf[1] = ((addr + offset) & 0xff000000) >> 24;
		buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		t[1].len = end;

		mutex_lock(&spi_mutex);
		status |= spi_sync(g_spi, &m);
		mutex_unlock(&spi_mutex);

		if (spi_cmd == RT5677_SPI_READ_BURST) {
			for (i = 0; i < end; i += 8) {
				rx_data[offset + i + 0] = rx_buf[i + 7];
				rx_data[offset + i + 1] = rx_buf[i + 6];
				rx_data[offset + i + 2] = rx_buf[i + 5];
				rx_data[offset + i + 3] = rx_buf[i + 4];
				rx_data[offset + i + 4] = rx_buf[i + 3];
				rx_data[offset + i + 5] = rx_buf[i + 2];
				rx_data[offset + i + 6] = rx_buf[i + 1];
				rx_data[offset + i + 7] = rx_buf[i + 0];
			}
		} else {
			for (i = 0; i < end; i++)
				rx_data[offset + i] = rx_buf[end - i - 1];
		}

		offset += end;
	}

	return status;
}
EXPORT_SYMBOL_GPL(rt5677_spi_read);

int rt5677_spi_write(u32 addr, const u8 *txbuf, size_t len)
{
	unsigned int i, end, offset = 0;
	int status = 0;
	u8 write_buf[SPI_BURST_LEN + SPI_HEADER + 1];
	u8 spi_cmd;

	if (!g_spi)
		return -ENODEV;

	while (offset < len) {
		switch ((addr + offset) & 0x7) {
		case 4:
			spi_cmd = RT5677_SPI_WRITE_32;
			end = 4;
			break;
		case 2:
		case 6:
			spi_cmd = RT5677_SPI_WRITE_16;
			end = 2;
			break;
		case 0:
			spi_cmd = RT5677_SPI_WRITE_BURST;
			if (offset + SPI_BURST_LEN <= len)
				end = SPI_BURST_LEN;
			else {
				end = len - offset;
				end = (((end - 1) >> 3) + 1) << 3;
			}
			break;
		default:
			pr_err("Bad section alignment\n");
			return -EACCES;
		}

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		if (spi_cmd == RT5677_SPI_WRITE_BURST) {
			for (i = 0; i < end; i += 8) {
				write_buf[i + 12] = txbuf[offset + i + 0];
				write_buf[i + 11] = txbuf[offset + i + 1];
				write_buf[i + 10] = txbuf[offset + i + 2];
				write_buf[i +  9] = txbuf[offset + i + 3];
				write_buf[i +  8] = txbuf[offset + i + 4];
				write_buf[i +  7] = txbuf[offset + i + 5];
				write_buf[i +  6] = txbuf[offset + i + 6];
				write_buf[i +  5] = txbuf[offset + i + 7];
			}
		} else {
			unsigned int j = end + (SPI_HEADER - 1);
			for (i = 0; i < end; i++, j--) {
				if (offset + i < len)
					write_buf[j] = txbuf[offset + i];
				else
					write_buf[j] = 0;
			}
		}
		write_buf[end + SPI_HEADER] = spi_cmd;

		mutex_lock(&spi_mutex);
		status |= spi_write(g_spi, write_buf, end + SPI_HEADER + 1);
		mutex_unlock(&spi_mutex);

		offset += end;
	}

	return status;
}
EXPORT_SYMBOL_GPL(rt5677_spi_write);

void rt5677_spi_hotword_detected(void)
{
	struct snd_soc_platform *platform;
	struct rt5677_dsp *rt5677_dsp;

	if (!g_spi)
		return;
	platform = snd_soc_lookup_platform(&g_spi->dev);
	if (!platform) {
		dev_err(&g_spi->dev, "Can't get snd_soc_platform\n");
		return;
	}

	rt5677_dsp = snd_soc_platform_get_drvdata(platform);
	if (!rt5677_dsp) {
		dev_err(&g_spi->dev, "Can't get rt5677_dsp\n");
		return;
	}

	mutex_lock(&rt5677_dsp->dma_lock);
	dev_info(rt5677_dsp->dev, "Hotword detected\n");
	rt5677_dsp->new_hotword = true;
	mutex_unlock(&rt5677_dsp->dma_lock);

	schedule_delayed_work(&rt5677_dsp->copy_work, 0);
}
EXPORT_SYMBOL_GPL(rt5677_spi_hotword_detected);

static int rt5677_spi_probe(struct spi_device *spi)
{
	int ret;

	g_spi = spi;

	ret = snd_soc_register_platform(&spi->dev, &rt5677_spi_platform);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register platform.\n");
		goto err_plat;
	}

	ret = snd_soc_register_component(&spi->dev, &rt5677_spi_dai_component,
					 &rt5677_spi_dai, 1);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register component.\n");
		goto err_comp;
	}
	return 0;

err_comp:
	snd_soc_unregister_platform(&spi->dev);
err_plat:
	return ret;
}

static int rt5677_spi_remove(struct spi_device *spi)
{
	snd_soc_unregister_component(&spi->dev);
	snd_soc_unregister_platform(&spi->dev);
	return 0;
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id rt5677_spi_acpi_id[] = {
	{ "RT5677AA", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, rt5677_spi_acpi_id);
#endif

static struct spi_driver rt5677_spi_driver = {
	.driver = {
		.name = "rt5677spi",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(rt5677_spi_acpi_id),
	},
	.probe = rt5677_spi_probe,
	.remove = rt5677_spi_remove,
};
module_spi_driver(rt5677_spi_driver);

MODULE_DESCRIPTION("ASoC RT5677 SPI driver");
MODULE_AUTHOR("Oder Chiou <oder_chiou@realtek.com>");
MODULE_LICENSE("GPL v2");
