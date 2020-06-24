// SPDX-License-Identifier: GPL-2.0
/*
 * xc7022 driver
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>


#define XC7022_LINK_FREQ_300MHZ	300000000
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define XC7022_PIXEL_RATE		(XC7022_LINK_FREQ_300MHZ * 2 * 2 / 10)
#define XC7022_XVCLK_FREQ		24000000

#define CHECK_CHIP_ID_REG1		0xfffd
#define CHECK_CHIP_ID_REG2		0xfffe
#define CHECK_CHIP_ID_VAL		0x80
#define XC7022_CHIP_ID_REG1		0xfffb
#define XC7022_CHIP_ID_REG2		0xfffc
#define XC7022_CHIP_ID1			0x71
#define XC7022_CHIP_ID2			0x60
#define XC6130_CHIP_ID_REG1		0x0002
#define XC6130_CHIP_ID_REG2	0x0003
#define XC6130_CHIP_ID1			0x43
#define XC6130_CHIP_ID2			0x58

#define XC7022_REG_VALUE_08BIT		1
#define XC7022_REG_VALUE_16BIT		2
#define XC7022_REG_VALUE_24BIT		3

#define	XC7022_EXPOSURE_MIN		4
#define	XC7022_EXPOSURE_STEP		1
#define XC7022_VTS_MAX			0x7fff
#define XC7022_GAIN_MIN		0x10
#define XC7022_GAIN_MAX		0xf8
#define XC7022_GAIN_STEP		1
#define XC7022_GAIN_DEFAULT		0x10

#define REG_NULL			0xFFFF


static DEFINE_MUTEX(xc7022_power_mutex);
static int xc7022_power_count;


static const char * const xc7022_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define XC7022_NUM_SUPPLIES ARRAY_SIZE(xc7022_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct xc7022_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct xc7022 {
	struct i2c_client	*client;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[XC7022_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	const struct xc7022_mode *cur_mode;
};

static struct xc7022 *xc7022_master;

#define to_xc7022(sd) container_of(sd, struct xc7022, subdev)

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 600Mbps
 */
static const struct regval xc7022_1080p_regs[] = {
	{0xfffd, 0x80}, // AE_avg
	{0xfffe, 0x30}, // AE_avg
	{0xfffe, 0x30},   // AE_avg
	{0x1f04, 0x07},   // WIN width
	{0x1f05, 0x80},
	{0x1f06, 0x03},   // WIN height
	{0x1f07, 0x60},
	{0x1f08, 0x03},
	{0xfffe, 0x2d},
	{0x0003, 0x39},
	{0xfffe, 0x26},
	{0x8010, 0x05},
	{0x8012, 0x80},
	{0x8013, 0x07},
	{0x8016, 0x00},
	{0xfffe, 0x2c},
	{0x0001, 0x07},
	{0x0002, 0x80},
	{0x0004, 0x04},
	{0x0005, 0x38},
	{0x0048, 0x0E},
	{0x0049, 0xF0},
	{0xfffe, 0x26},
	{0x2019, 0x07},
	{0x201a, 0x80},
	{0x201b, 0x04},
	{0x201c, 0x38},
	{0xfffe, 0x30},
	{0x0001, 0x92},
	{0x005e, 0x7f},
	{0x005f, 0x07},
	{0x0060, 0x37},
	{0x0061, 0x04},
	{0x0064, 0x80},
	{0x0065, 0x07},
	{0x0066, 0x38},
	{0x0067, 0x04},
	{0x0006, 0x07},
	{0x0007, 0x80},
	{0x0008, 0x04},
	{0x0009, 0x38},
	{0x000a, 0x07},
	{0x000b, 0x80},
	{0x000c, 0x04},
	{0x000d, 0x38},
	{0xfffd, 0x80},
	{0xfffe, 0x50},
	{0x001a, 0x08},
	{0x001a, 0x00},
	{REG_NULL, 0x00},
};

static const struct regval xc7022_720p_regs[] = {
	{0xfffd, 0x80}, // AE_avg
	{0xfffe, 0x30},   // AE_avg
	{0x1f04, 0x07},   // WIN width
	{0x1f05, 0x80},
	{0x1f06, 0x03},   // WIN height
	{0x1f07, 0x60},
	{0x1f08, 0x03},
	{0xfffe, 0x2d},
	{0x0003, 0x39},
	{0xfffe, 0x26},
	{0x8010, 0x05},
	{0x8012, 0x80},
	{0x8013, 0x07},
	{0x8016, 0x00},
	{0xfffe, 0x2c},
	{0x0001, 0x05},
	{0x0002, 0x00},
	{0x0004, 0x02},
	{0x0005, 0xD0},
	{0x0048, 0x09},
	{0x0049, 0xF0},
	{0xfffe, 0x26},
	{0x2019, 0x05},
	{0x201a, 0x00},
	{0x201b, 0x02},
	{0x201c, 0xD0},
	{0xfffe, 0x30},
	{0x0001, 0x92},
	{0x005e, 0x7f},
	{0x005f, 0x07},
	{0x0060, 0x37},
	{0x0061, 0x04},
	{0x0064, 0x00},
	{0x0065, 0x05},
	{0x0066, 0xD0},
	{0x0067, 0x02},
	{0x0006, 0x07},
	{0x0007, 0x80},
	{0x0008, 0x04},
	{0x0009, 0x38},
	{0x000a, 0x05},
	{0x000b, 0x00},
	{0x000c, 0x02},
	{0x000d, 0xD0},
	{0xfffd, 0x80},
	{0xfffe, 0x50},
	{0x001a, 0x08},
	{0x001a, 0x00},
	{REG_NULL, 0x00},
};

static const struct regval xc7022_480p_regs[] = {
	{0xfffd, 0x80}, // AE_avg
	{0xfffe, 0x30}, // AE_avg
	{0x1f04, 0x05}, // WIN width
	{0x1f05, 0xa0},
	{0x1f06, 0x03}, // WIN height
	{0x1f07, 0x60},
	{0x1f08, 0x03},
	{0xfffe, 0x2d},
	{0x0003, 0x38},
	{0xfffe, 0x26},
	{0x8010, 0x05},
	{0x8012, 0xA0},
	{0x8013, 0x05},
	{0x8016, 0xf0},
	{0xfffe, 0x2c},
	{0x0001, 0x02},
	{0x0002, 0x80},
	{0x0004, 0x01},
	{0x0005, 0xE0},
	{0x0048, 0x04},
	{0x0049, 0xF0},
	{0xfffe, 0x26},
	{0x2019, 0x02},
	{0x201a, 0x80},
	{0x201b, 0x01},
	{0x201c, 0xE0},
	{0xfffe, 0x30},
	{0x0001, 0x92},
	{0x005e, 0x9F},
	{0x005f, 0x05},
	{0x0060, 0x37},
	{0x0061, 0x04},
	{0x0064, 0x80},
	{0x0065, 0x02},
	{0x0066, 0xE0},
	{0x0067, 0x01},
	{0x0006, 0x05},
	{0x0007, 0xA0},
	{0x0008, 0x04},
	{0x0009, 0x38},
	{0x000a, 0x02},
	{0x000b, 0x80},
	{0x000c, 0x01},
	{0x000d, 0xE0},
	{0xfffd, 0x80},
	{0xfffe, 0x50},
	{0x001a, 0x08},
	{0x001a, 0x00},
	{REG_NULL, 0x00},
};

static const struct regval xc7022_stream_on_regs[] = {
	{0xfffd, 0x80},
	{0xfffe, 0x26},
	{0x8010, 0x0d},
	{REG_NULL, 0x00},
};

static const struct regval xc7022_stream_off_regs[] = {
	{0xfffd, 0x80},
	{0xfffe, 0x26},
	{0x8010, 0x09},
	{REG_NULL, 0x00},
};


static const struct xc7022_mode supported_modes[] = {
	{
		.width = 1280,
		.height = 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x0600,
		.hts_def = 0x12c0,
		.vts_def = 0x0680,

		//.exp_def = 0x0450,
		//.hts_def = 0x0780,
		//.vts_def = 0x0680,
		.reg_list = xc7022_720p_regs,
	},
	{
		.width = 640,
		.height = 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0600,
		.hts_def = 0x12c0,
		.vts_def = 0x0680,
		.reg_list = xc7022_480p_regs,
	},
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0600,
		.hts_def = 0x12c0,
		.vts_def = 0x0680,
		.reg_list = xc7022_1080p_regs,
	},

};

static const s64 link_freq_menu_items[] = {
	XC7022_LINK_FREQ_300MHZ
};

static const char * const xc7022_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int xc7022_write_reg(struct i2c_client *client, u16 reg,
			     u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int xc7022_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = xc7022_write_reg(client, regs[i].addr,
					XC7022_REG_VALUE_08BIT,
					regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int xc7022_read_reg(struct i2c_client *client, u16 reg,
			    unsigned int len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int xc7022_get_reso_dist(const struct xc7022_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct xc7022_mode *
xc7022_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = xc7022_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	pr_info("========= set %d cur_best_fit\n", cur_best_fit);
	return &supported_modes[cur_best_fit];
}

static int xc7022_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct xc7022 *xc7022 = to_xc7022(sd);
	const struct xc7022_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&xc7022->mutex);

	mode = xc7022_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_YUYV10_2X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&xc7022->mutex);
		return -ENOTTY;
#endif
	} else {
		xc7022->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(xc7022->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(xc7022->vblank, vblank_def,
					 XC7022_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&xc7022->mutex);

	return 0;
}

static int xc7022_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct xc7022 *xc7022 = to_xc7022(sd);
	const struct xc7022_mode *mode = xc7022->cur_mode;

	mutex_lock(&xc7022->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&xc7022->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_YUYV10_2X10;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&xc7022->mutex);

	return 0;
}


static int xc7022_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct xc7022 *xc7022 = to_xc7022(sd);
	const struct xc7022_mode *mode = xc7022->cur_mode;

	mutex_lock(&xc7022->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&xc7022->mutex);

	return 0;
}

static int __xc7022_start_stream(struct xc7022 *xc7022)
{
	int ret;

	xc7022_write_array(xc7022->client, xc7022->cur_mode->reg_list);

	ret = xc7022_write_array(xc7022->client, xc7022_stream_on_regs);
	if(ret)
		printk("write stream on failed\n");

	/* In case these controls are set before streaming */
	mutex_unlock(&xc7022->mutex);
	ret = v4l2_ctrl_handler_setup(&xc7022->ctrl_handler);
	mutex_lock(&xc7022->mutex);
	if (ret)
		return ret;

	return 0;
}

static int __xc7022_stop_stream(struct xc7022 *xc7022)
{
	int ret;

	ret = xc7022_write_array(xc7022->client, xc7022_stream_off_regs);
	if(ret)
		printk("write stream off failed\n");

	return ret;
}

static int xc7022_s_stream(struct v4l2_subdev *sd, int on)
{
	struct xc7022 *xc7022 = to_xc7022(sd);
	struct i2c_client *client = xc7022->client;
	int ret = 0;

	mutex_lock(&xc7022->mutex);
	on = !!on;
	if (on == xc7022->streaming){
		goto unlock_and_return;
	}
	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __xc7022_start_stream(xc7022);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__xc7022_stop_stream(xc7022);
		pm_runtime_put(&client->dev);
	}

	xc7022->streaming = on;

unlock_and_return:
	mutex_unlock(&xc7022->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 xc7022_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, XC7022_XVCLK_FREQ / 1000 / 1000);
}

static int __xc7022_master_power_on(struct device *dev) {
	struct xc7022 *xc7022 = xc7022_master;
	int ret;

	if (!xc7022) {
		dev_err(dev, "no xc7022 master set\n");
		return -EINVAL;
	}

	xc7022_power_count++;
	if (xc7022_power_count > 1) {
		ret = 0;
		goto err_shortcut;
	}

	if (!IS_ERR_OR_NULL(xc7022->pins_default)) {
		ret = pinctrl_select_state(xc7022->pinctrl,
					   xc7022->pins_default);
		if (ret < 0) {
			dev_err(dev, "could not set pins\n");
			goto err_pins;
		}
	}

	if (!IS_ERR(xc7022->reset_gpio)){
		gpiod_set_value_cansleep(xc7022->reset_gpio, 1);
	} else{
		dev_err(dev, "could not get reset gpio\n");
		return -EINVAL;
	}

	return 0;

err_pins:
	xc7022_power_count--;
err_shortcut:
	return ret;
}

static void __xc7022_master_power_off(struct device *dev)
{
	struct xc7022 *xc7022 = xc7022_master;

	if (!xc7022) {
		dev_err(dev, "no xc7022 master set\n");
		return;
	}

	xc7022_power_count--;
	if (xc7022_power_count > 0) {
		return;
	}

	regulator_bulk_disable(XC7022_NUM_SUPPLIES, xc7022->supplies);

	return;
}

static int __xc7022_power_on(struct xc7022 *xc7022)
{
	int ret;
	struct device *dev = &xc7022->client->dev;

	mutex_lock(&xc7022_power_mutex);
	ret = __xc7022_master_power_on(dev);
	if (ret) {
		dev_err(dev, "could not power on, error %d\n", ret);
		goto err_power;
	}

	mutex_unlock(&xc7022_power_mutex);
	return 0;

err_power:
	mutex_unlock(&xc7022_power_mutex);
	return ret;
}

static void __xc7022_power_off(struct xc7022 *xc7022)
{
	struct device *dev = &xc7022->client->dev;

	mutex_lock(&xc7022_power_mutex);
	__xc7022_master_power_off(dev);
	mutex_unlock(&xc7022_power_mutex);
}

static int xc7022_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct xc7022 *xc7022 = to_xc7022(sd);

	return __xc7022_power_on(xc7022);
}

static int xc7022_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct xc7022 *xc7022 = to_xc7022(sd);

	__xc7022_power_off(xc7022);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int xc7022_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct xc7022 *xc7022 = to_xc7022(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct xc7022_mode *def_mode = &supported_modes[0];

	mutex_lock(&xc7022->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_YUYV10_2X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&xc7022->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static const struct dev_pm_ops xc7022_pm_ops = {
	SET_RUNTIME_PM_OPS(xc7022_runtime_suspend,
			   xc7022_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops xc7022_internal_ops = {
	.open = xc7022_open,
};
#endif

static const struct v4l2_subdev_video_ops xc7022_video_ops = {
	.s_stream = xc7022_s_stream,
	.g_frame_interval = xc7022_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops xc7022_pad_ops = {
	.get_fmt = xc7022_get_fmt,
	.set_fmt = xc7022_set_fmt,
};

static const struct v4l2_subdev_ops xc7022_subdev_ops = {
	.video	= &xc7022_video_ops,
	.pad	= &xc7022_pad_ops,
};

static int xc7022_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct xc7022 *xc7022 = container_of(ctrl->handler,
					     struct xc7022, ctrl_handler);
	s64 max;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = xc7022->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(xc7022->exposure,
					 xc7022->exposure->minimum, max,
					 xc7022->exposure->step,
					 xc7022->exposure->default_value);
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops xc7022_ctrl_ops = {
	.s_ctrl = xc7022_set_ctrl,
};

static int xc7022_initialize_controls(struct xc7022 *xc7022)
{
	const struct xc7022_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &xc7022->ctrl_handler;
	mode = xc7022->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &xc7022->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, XC7022_PIXEL_RATE, 1, XC7022_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	xc7022->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (xc7022->hblank)
		xc7022->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	xc7022->vblank = v4l2_ctrl_new_std(handler, &xc7022_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				XC7022_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	xc7022->exposure = v4l2_ctrl_new_std(handler, &xc7022_ctrl_ops,
				V4L2_CID_EXPOSURE, XC7022_EXPOSURE_MIN,
				exposure_max, XC7022_EXPOSURE_STEP,
				mode->exp_def);

	xc7022->anal_gain = v4l2_ctrl_new_std(handler, &xc7022_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, XC7022_GAIN_MIN,
				XC7022_GAIN_MAX, XC7022_GAIN_STEP,
				XC7022_GAIN_DEFAULT);

	xc7022->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&xc7022_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(xc7022_test_pattern_menu) - 1,
				0, 0, xc7022_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&xc7022->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	xc7022->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int xc7022_check_sensor_id(struct xc7022 *xc7022,
				   struct i2c_client *client)
{
	struct device *dev = &xc7022->client->dev;
	u32 id = 0;
	int ret;

	ret = xc7022_write_reg(client, CHECK_CHIP_ID_REG1,
							XC7022_REG_VALUE_08BIT,
							CHECK_CHIP_ID_VAL);
	if (ret){
		dev_err(dev, "write CHECK_CHIP_ID_REG1 failed\n");
		return ret;
	}

	ret = xc7022_write_reg(client, CHECK_CHIP_ID_REG2,
							XC7022_REG_VALUE_08BIT,
							CHECK_CHIP_ID_VAL);
	if (ret){
		dev_err(dev, "write CHECK_CHIP_ID_REG2 failed\n");
		return ret;
	}

	ret = xc7022_read_reg(client, XC7022_CHIP_ID_REG1,
			       XC7022_REG_VALUE_08BIT, &id);
	if (id == XC7022_CHIP_ID1) {
		dev_info(dev, "chip is xc7022\n");
		ret = xc7022_read_reg(client, XC7022_CHIP_ID_REG2,
		       XC7022_REG_VALUE_08BIT, &id);
		if (id != XC7022_CHIP_ID2) {
			dev_err(dev, "Unexpected sensor of XC7022_CHIP_ID_REG2, id(%06x), ret(%d)\n", id, ret);
			return ret;
		}
	} else {
		ret = xc7022_read_reg(client, XC6130_CHIP_ID_REG1,
			       XC7022_REG_VALUE_08BIT, &id);
		if (id == XC6130_CHIP_ID1)	{
			dev_info(dev, "chip is xc6130\n");
			ret = xc7022_read_reg(client, XC6130_CHIP_ID_REG2,
			       XC7022_REG_VALUE_08BIT, &id);
			if (id != XC6130_CHIP_ID2) {
				dev_err(dev, "Unexpected sensor of XC6130_CHIP_ID_REG2, id(%06x), ret(%d)\n", id, ret);
				return ret;
			}
		} else {
			dev_err(dev, "Check chip ID failed\n");
			return ret;
		}
	}

	return 0;

}

static int xc7022_configure_regulators(struct device *dev)
{
	struct xc7022 *xc7022 = xc7022_master;
	unsigned int i;

	if (!xc7022) {
		dev_err(dev, "no xc7022 master set\n");
		return -EINVAL;
	}
	for (i = 0; i < XC7022_NUM_SUPPLIES; i++)
		xc7022->supplies[i].supply = xc7022_supply_names[i];

	return devm_regulator_bulk_get(dev,
				       XC7022_NUM_SUPPLIES,
				       xc7022->supplies);
}

static void xc7022_detach_master(void *data)
{
	if (xc7022_master == data)
		xc7022_master = NULL;
}

static int xc7022_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct xc7022 *xc7022;
	struct v4l2_subdev *sd;
	int ret;

	xc7022 = devm_kzalloc(dev, sizeof(*xc7022), GFP_KERNEL);
	if (!xc7022)
		return -ENOMEM;

	xc7022->client = client;
	xc7022->cur_mode = &supported_modes[0];

	if (!xc7022_master) {
		xc7022_master = xc7022;
		devm_add_action(dev, xc7022_detach_master, xc7022);
	}
	if (xc7022_master == xc7022) {

		xc7022->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(xc7022->reset_gpio)){
			dev_warn(dev, "Failed to get reset-gpios\n");
			return -1;
		} else{
			gpiod_set_value_cansleep(xc7022->reset_gpio, 1);
			msleep(4500);
		}

		ret = xc7022_configure_regulators(dev);
		if (ret) {
			dev_err(dev, "Failed to get power regulators\n");
			return ret;
		}

		xc7022->pinctrl = devm_pinctrl_get(dev);
		if (!IS_ERR(xc7022->pinctrl)) {
			xc7022->pins_default =
				pinctrl_lookup_state(xc7022->pinctrl, "default");
			if (IS_ERR(xc7022->pins_default))
				dev_err(dev, "could not get default pinstate\n");

		}
	}

	mutex_init(&xc7022->mutex);

	sd = &xc7022->subdev;
	v4l2_i2c_subdev_init(sd, client, &xc7022_subdev_ops);
	ret = xc7022_initialize_controls(xc7022);

	if (ret)
		goto err_destroy_mutex;

	ret = __xc7022_power_on(xc7022);
	if (ret)
		goto err_free_handler;

	ret = xc7022_check_sensor_id(xc7022, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &xc7022_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	xc7022->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &xc7022->pad, 0);
	if (ret < 0)
		goto err_power_off;
#endif

	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	__xc7022_stop_stream(xc7022);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__xc7022_power_off(xc7022);
err_free_handler:
	v4l2_ctrl_handler_free(&xc7022->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&xc7022->mutex);

	return ret;
}

static int xc7022_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct xc7022 *xc7022 = to_xc7022(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&xc7022->ctrl_handler);
	mutex_destroy(&xc7022->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__xc7022_power_off(xc7022);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id xc7022_of_match[] = {
	{ .compatible = "firefly,xc7022" },
	{},
};
MODULE_DEVICE_TABLE(of, xc7022_of_match);
#endif

static const struct i2c_device_id xc7022_match_id[] = {
	{ "firefly,xc7022", 0 },
	{ },
};

static struct i2c_driver xc7022_i2c_driver = {
	.driver = {
		.name = "xc7022",
		.pm = &xc7022_pm_ops,
		.of_match_table = of_match_ptr(xc7022_of_match),
	},
	.probe		= &xc7022_probe,
	.remove		= &xc7022_remove,
	.id_table	= xc7022_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&xc7022_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&xc7022_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision xc7022 sensor driver");
MODULE_LICENSE("GPL v2");
