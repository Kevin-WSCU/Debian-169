/*
 * Driver for the MT9V024 camera sensor.
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015 By Tech Design S.L. All Rights Reserved.
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Based on:
 * - the MT9V024 driver from QC msm-3.10 kernel on codeaurora.org:
 *   https://us.codeaurora.org/cgit/quic/la/kernel/msm-3.10/tree/drivers/
 *       media/platform/msm/camera_v2/sensor/mt9v024.c?h=LA.BR.1.2.4_rb1.41
 * - the OV5640 driver posted on linux-media:
 *   https://www.mail-archive.com/linux-media%40vger.kernel.org/msg92671.html
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-of.h>
#include <media/v4l2-subdev.h>

static DEFINE_MUTEX(mt9v024_lock);

/* HACKs here! */

#include <../drivers/media/platform/msm/cci/msm_cci.h>

#ifdef dev_dbg
	#undef dev_dbg
	#define dev_dbg dev_err
#endif

#define MT9V024_VOLTAGE_ANALOG               2800000
#define MT9V024_VOLTAGE_DIGITAL_CORE         1500000
#define MT9V024_VOLTAGE_DIGITAL_IO           1800000


#define MT9V024_SYSTEM_CTRL0		0x0100
#define	MT9V024_SYSTEM_CTRL0_START	0x01
#define MT9V024_SYSTEM_CTRL0_STOP 0x00

#define MT9V024_CHIP_ID		0x00
#define	MT9V024_CHIP_ID_WORD	0x1324


#define TOSHIBA_BRG_ID_HIGH 0x44
#define TOSHIBA_BRG_ID_LOW 0x01


#define MT9V024_AWB_MANUAL_CONTROL	0x3406
#define		MT9V024_AWB_MANUAL_ENABLE	BIT(0)
#define MT9V024_AEC_PK_MANUAL		0x3503
#define		MT9V024_AEC_MANUAL_ENABLE	BIT(0)
#define		MT9V024_AGC_MANUAL_ENABLE	BIT(1)
#define MT9V024_TIMING_TC_REG20		0x3820
#define		MT9V024_SENSOR_VFLIP		BIT(1)
#define		MT9V024_ISP_VFLIP		BIT(2)
#define MT9V024_TIMING_TC_REG21		0x3821
#define		MT9V024_SENSOR_MIRROR		BIT(1)
#define MT9V024_PRE_ISP_TEST_SETTING_1	0x503d
#define		MT9V024_TEST_PATTERN_MASK	0x3
#define		MT9V024_SET_TEST_PATTERN(x)	((x) & MT9V024_TEST_PATTERN_MASK)
#define		MT9V024_TEST_PATTERN_ENABLE	BIT(7)
#define MT9V024_SDE_SAT_U		0x5583
#define MT9V024_SDE_SAT_V		0x5584

enum mt9v024_mode {
	MT9V024_MODE_MIN = 0,
	MT9V024_MODE_VGA = 0,
	MT9V024_MODE_MAX = 0
};

struct reg_value {
	u8 reg;
	u16 val;
};

struct mt9v024_mode_info {
	enum mt9v024_mode mode;
	u32 width;
	u32 height;
	struct reg_value *data;
	u32 data_size;
};

struct mt9v024 {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_of_endpoint ep;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;
	struct clk *xclk;
	/* External clock frequency currently supported is 23880000Hz */
	u32 xclk_freq;

	struct regulator *io_regulator;
	struct regulator *core_regulator;
	struct regulator *analog_regulator;

	enum mt9v024_mode current_mode;

	/* Cached control values */
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *autogain;
	struct v4l2_ctrl *autoexposure;
	struct v4l2_ctrl *awb;
	struct v4l2_ctrl *pattern;

	struct mutex power_lock; /* lock to protect power state */
	bool power;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *rst_gpio;

	struct v4l2_subdev *cci;
};

static inline struct mt9v024 *to_mt9v024(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9v024, sd);
}
/*Some controllers(MIPI receiver detect the LPxx transaction),so stay in standby after the initial setting*/
static struct reg_value MT9V024_VGA_60fps[] =
{

};



static struct mt9v024_mode_info mt9v024_mode_info_data[MT9V024_MODE_MAX + 1] = {
	{
		.mode = MT9V024_MODE_VGA,
		.width = 640,
		.height = 480,
		.data = MT9V024_VGA_60fps,
		.data_size = ARRAY_SIZE(MT9V024_VGA_60fps)
	},

};

static int mt9v024_regulators_enable(struct mt9v024 *mt9v024)
{
	int ret;

	ret = regulator_enable(mt9v024->io_regulator);
	if (ret < 0) {
		dev_err(mt9v024->dev, "set io voltage failed\n");
		return ret;
	}

	ret = regulator_enable(mt9v024->core_regulator);
	if (ret) {
		dev_err(mt9v024->dev, "set core voltage failed\n");
		goto err_disable_io;
	}

	ret = regulator_enable(mt9v024->analog_regulator);
	if (ret) {
		dev_err(mt9v024->dev, "set analog voltage failed\n");
		goto err_disable_core;
	}

	return 0;

err_disable_core:
	regulator_disable(mt9v024->core_regulator);
err_disable_io:
	regulator_disable(mt9v024->io_regulator);

	return ret;
}

static void mt9v024_regulators_disable(struct mt9v024 *mt9v024)
{
	int ret;

	ret = regulator_disable(mt9v024->analog_regulator);
	if (ret < 0)
		dev_err(mt9v024->dev, "analog regulator disable failed\n");

	ret = regulator_disable(mt9v024->core_regulator);
	if (ret < 0)
		dev_err(mt9v024->dev, "core regulator disable failed\n");

	ret = regulator_disable(mt9v024->io_regulator);
	if (ret < 0)
		dev_err(mt9v024->dev, "io regulator disable failed\n");
}

static int mt9v024_write_reg_to(struct mt9v024 *mt9v024, u16 reg, u8 val, u16 i2c_addr)
{
	int ret;

	ret = msm_cci_ctrl_write(i2c_addr, reg, &val, 1);
	if (ret < 0)
		dev_err(mt9v024->dev,
			"%s: write reg error %d on addr 0x%x: reg=0x%x, val=0x%x\n",
			__func__, ret, i2c_addr, reg, val);

	return ret;
}

static int mt9v024_write_reg(struct mt9v024 *mt9v024, u8 reg, u16 val)
{
	int ret;

	u16 i2c_addr = mt9v024->i2c_client->addr = 0x90;

	ret = msm_cci_ctrl_write(i2c_addr, reg, &val, 1);
	if (ret < 0)
		dev_err(mt9v024->dev,
			"%s: write reg error %d on addr 0x%x: reg=0x%x, val=0x%x\n",
			__func__, ret, i2c_addr, reg, val);

	return ret;
}

static int mt9v024_read_reg(struct mt9v024 *mt9v024, u16 reg, u8 *val)
{
	u8 tmpval;
	int ret;
	u16 i2c_addr = mt9v024->i2c_client->addr = 0x90;

	ret = msm_cci_ctrl_read(i2c_addr, reg, &tmpval, 1);
	if (ret < 0) {
		dev_err(mt9v024->dev,
			"%s: read reg error %d on addr 0x%x: reg=0x%x\n",
			__func__, ret, i2c_addr, reg);
		return ret;
	}

	*val = tmpval;

	return 0;
}

static int mt9v024_set_aec_mode(struct mt9v024 *mt9v024, u32 mode)
{
	u8 val;
	int ret;

	ret = mt9v024_read_reg(mt9v024, MT9V024_AEC_PK_MANUAL, &val);
	if (ret < 0)
		return ret;

	if (mode == V4L2_EXPOSURE_AUTO)
		val &= ~MT9V024_AEC_MANUAL_ENABLE;
	else /* V4L2_EXPOSURE_MANUAL */
		val |= MT9V024_AEC_MANUAL_ENABLE;

	return mt9v024_write_reg(mt9v024, MT9V024_AEC_PK_MANUAL, val);
}

static int mt9v024_set_agc_mode(struct mt9v024 *mt9v024, u32 enable)
{
	u8 val;
	int ret;

	ret = mt9v024_read_reg(mt9v024, MT9V024_AEC_PK_MANUAL, &val);
	if (ret < 0)
		return ret;

	if (enable)
		val &= ~MT9V024_AGC_MANUAL_ENABLE;
	else
		val |= MT9V024_AGC_MANUAL_ENABLE;

	return mt9v024_write_reg(mt9v024, MT9V024_AEC_PK_MANUAL, val);
}

static int mt9v024_set_register_array(struct mt9v024 *mt9v024,
				     struct reg_value *settings,
				     u32 num_settings)
{
	u16 reg;
	u8 val;
	u32 i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		reg = settings->reg;
		val = settings->val;

		ret = mt9v024_write_reg(mt9v024, reg, val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int mt9v024_init(struct mt9v024 *mt9v024)
{
	return 0;
}

static int mt9v024_change_mode(struct mt9v024 *mt9v024, enum mt9v024_mode mode)
{
	struct reg_value *settings;
	u32 num_settings;

	settings = mt9v024_mode_info_data[mode].data;
	num_settings = mt9v024_mode_info_data[mode].data_size;

	return mt9v024_set_register_array(mt9v024, settings, num_settings);
}

static int mt9v024_set_power_on(struct mt9v024 *mt9v024)
{
	int ret;

	clk_set_rate(mt9v024->xclk, mt9v024->xclk_freq);

	ret = clk_prepare_enable(mt9v024->xclk);
	if (ret < 0) {
		dev_err(mt9v024->dev, "clk prepare enable failed\n");
		return ret;
	}

	ret = mt9v024_regulators_enable(mt9v024);
	if (ret < 0) {
		clk_disable_unprepare(mt9v024->xclk);
		return ret;
	}

	usleep_range(5000, 15000);
	gpiod_set_value_cansleep(mt9v024->enable_gpio, 0);

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(mt9v024->rst_gpio, 0);

	msleep(20);

	return ret;
}

static void mt9v024_set_power_off(struct mt9v024 *mt9v024)
{
	gpiod_set_value_cansleep(mt9v024->rst_gpio, 1);
	gpiod_set_value_cansleep(mt9v024->enable_gpio, 1);
	mt9v024_regulators_disable(mt9v024);
	clk_disable_unprepare(mt9v024->xclk);
}

static int mt9v024_s_power(struct v4l2_subdev *sd, int on)
{
	struct mt9v024 *mt9v024 = to_mt9v024(sd);
	int ret = 0;

	mutex_lock(&mt9v024->power_lock);

	if (on) {
		ret = msm_cci_ctrl_init();
		if (ret < 0)
			goto exit;
	}

	if (mt9v024->power == !on) {
		/* Power state changes. */
		if (on) {
			mutex_lock(&mt9v024_lock);

			ret = mt9v024_set_power_on(mt9v024);
			if (ret < 0) {
				dev_err(mt9v024->dev, "could not set power %s\n",
					on ? "on" : "off");
				goto exit;
			}

//don't change sensor i2c address for this time

//			ret = mt9v024_write_reg_to(mt9v024, 0x0109,
//					       mt9v024->i2c_client->addr, 0xc0);


			if (ret < 0) {
				dev_err(mt9v024->dev,
					"could not change i2c address\n");
				mt9v024_set_power_off(mt9v024);
				mutex_unlock(&mt9v024_lock);
				goto exit;
			}

			mutex_unlock(&mt9v024_lock);
/*
			ret = mt9v024_init(mt9v024);
			if (ret < 0) {
				dev_err(mt9v024->dev,
					"could not set init registers\n");
				mt9v024_set_power_off(mt9v024);
				goto exit;
			}
*/
			ret = mt9v024_write_reg(mt9v024, MT9V024_SYSTEM_CTRL0,
					       MT9V024_SYSTEM_CTRL0_STOP);
			if (ret < 0) {
				mt9v024_set_power_off(mt9v024);
				goto exit;
			}
		} else {
			mt9v024_set_power_off(mt9v024);
		}

		/* Update the power state. */
		mt9v024->power = on ? true : false;
	}

exit:
	if (!on)
		msm_cci_ctrl_release();

	mutex_unlock(&mt9v024->power_lock);

	return ret;
}


static int mt9v024_set_saturation(struct mt9v024 *mt9v024, s32 value)
{
	u32 reg_value = (value * 0x10) + 0x40;
	int ret;

	ret = mt9v024_write_reg(mt9v024, MT9V024_SDE_SAT_U, reg_value);
	if (ret < 0)
		return ret;

	ret = mt9v024_write_reg(mt9v024, MT9V024_SDE_SAT_V, reg_value);

	return ret;
}

static int mt9v024_set_hflip(struct mt9v024 *mt9v024, s32 value)
{
	u8 val;
	int ret;

	ret = mt9v024_read_reg(mt9v024, MT9V024_TIMING_TC_REG21, &val);
	if (ret < 0)
		return ret;

	if (value == 0)
		val &= ~(MT9V024_SENSOR_MIRROR);
	else
		val |= (MT9V024_SENSOR_MIRROR);

	return mt9v024_write_reg(mt9v024, MT9V024_TIMING_TC_REG21, val);
}

static int mt9v024_set_vflip(struct mt9v024 *mt9v024, s32 value)
{
	u8 val;
	int ret;

	ret = mt9v024_read_reg(mt9v024, MT9V024_TIMING_TC_REG20, &val);
	if (ret < 0)
		return ret;

	if (value == 0)
		val |= (MT9V024_SENSOR_VFLIP | MT9V024_ISP_VFLIP);
	else
		val &= ~(MT9V024_SENSOR_VFLIP | MT9V024_ISP_VFLIP);

	return mt9v024_write_reg(mt9v024, MT9V024_TIMING_TC_REG20, val);
}

static int mt9v024_set_test_pattern(struct mt9v024 *mt9v024, s32 value)
{
	u8 val;
	int ret;

	ret = mt9v024_read_reg(mt9v024, MT9V024_PRE_ISP_TEST_SETTING_1, &val);
	if (ret < 0)
		return ret;

	if (value) {
		val &= ~MT9V024_SET_TEST_PATTERN(MT9V024_TEST_PATTERN_MASK);
		val |= MT9V024_SET_TEST_PATTERN(value - 1);
		val |= MT9V024_TEST_PATTERN_ENABLE;
	} else {
		val &= ~MT9V024_TEST_PATTERN_ENABLE;
	}

	return mt9v024_write_reg(mt9v024, MT9V024_PRE_ISP_TEST_SETTING_1, val);
}

static const char * const mt9v024_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bars",
	"Pseudo-Random Data",
	"Color Square",
	"Black Image",
};

static int mt9v024_set_awb(struct mt9v024 *mt9v024, s32 enable_auto)
{
	u8 val;
	int ret;

	ret = mt9v024_read_reg(mt9v024, MT9V024_AWB_MANUAL_CONTROL, &val);
	if (ret < 0)
		return ret;

	if (enable_auto)
		val &= ~MT9V024_AWB_MANUAL_ENABLE;
	else
		val |= MT9V024_AWB_MANUAL_ENABLE;

	return mt9v024_write_reg(mt9v024, MT9V024_AWB_MANUAL_CONTROL, val);
}

static int mt9v024_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9v024 *mt9v024 = container_of(ctrl->handler,
					     struct mt9v024, ctrls);

	int ret = -EINVAL;

	return 0;	
	mutex_lock(&mt9v024->power_lock);
	if (mt9v024->power == 0) {
		mutex_unlock(&mt9v024->power_lock);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_SATURATION:
		ret = mt9v024_set_saturation(mt9v024, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = mt9v024_set_awb(mt9v024, ctrl->val);
		break;
	case V4L2_CID_AUTOGAIN:
		ret = mt9v024_set_agc_mode(mt9v024, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = mt9v024_set_aec_mode(mt9v024, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = mt9v024_set_test_pattern(mt9v024, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = mt9v024_set_hflip(mt9v024, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = mt9v024_set_vflip(mt9v024, ctrl->val);
		break;
	}

	mutex_unlock(&mt9v024->power_lock);

	return ret;
}

static struct v4l2_ctrl_ops mt9v024_ctrl_ops = {
	.s_ctrl = mt9v024_s_ctrl,
};

static int mt9v024_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = { 0 };
	struct mt9v024 *mt9v024 = to_mt9v024(subdev);

	dev_err(mt9v024->dev, "%s: Enter\n", __func__);


	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 1920;
	fmt.format.height = 1080;

	v4l2_subdev_call(subdev, pad, set_fmt, cfg, &fmt);

	return 0;
}

static int mt9v024_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct mt9v024 *mt9v024 = to_mt9v024(sd);

	if (code->index > 0)
		return -EINVAL;

	code->code = mt9v024->fmt.code;

	return 0;
}

static int mt9v024_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > MT9V024_MODE_MAX)
		return -EINVAL;

	fse->min_width = mt9v024_mode_info_data[fse->index].width;
	fse->max_width = mt9v024_mode_info_data[fse->index].width;
	fse->min_height = mt9v024_mode_info_data[fse->index].height;
	fse->max_height = mt9v024_mode_info_data[fse->index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__mt9v024_get_pad_format(struct mt9v024 *mt9v024,
			struct v4l2_subdev_pad_config *cfg,
			unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&mt9v024->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mt9v024->fmt;
	default:
		return NULL;
	}
}

static int mt9v024_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct mt9v024 *mt9v024 = to_mt9v024(sd);

	format->format = *__mt9v024_get_pad_format(mt9v024, cfg, format->pad,
						  format->which);
	return 0;
}

static struct v4l2_rect *
__mt9v024_get_pad_crop(struct mt9v024 *mt9v024, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&mt9v024->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mt9v024->crop;
	default:
		return NULL;
	}
}

static enum mt9v024_mode mt9v024_find_nearest_mode(struct mt9v024 *mt9v024,
						 int width, int height)
{
	int i;

	for (i = MT9V024_MODE_MAX; i >= 0; i--) {
		if (mt9v024_mode_info_data[i].width <= width &&
		    mt9v024_mode_info_data[i].height <= height)
			break;
	}

	if (i < 0)
		i = 0;

	return (enum mt9v024_mode)i;
}

static int mt9v024_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct mt9v024 *mt9v024 = to_mt9v024(sd);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	enum mt9v024_mode new_mode;

	__crop = __mt9v024_get_pad_crop(mt9v024, cfg, format->pad,
			format->which);

	new_mode = mt9v024_find_nearest_mode(mt9v024,
			format->format.width, format->format.height);
	
	//printk("set format,new mode index:%d",new_mode);

	
	__crop->width = mt9v024_mode_info_data[new_mode].width;
	__crop->height = mt9v024_mode_info_data[new_mode].height;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		mt9v024->current_mode = new_mode;

	__format = __mt9v024_get_pad_format(mt9v024, cfg, format->pad,
			format->which);
	__format->width = __crop->width;
	__format->height = __crop->height;
	__format->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	__format->field = V4L2_FIELD_NONE;
	__format->colorspace = V4L2_COLORSPACE_SRGB;

	format->format = *__format;

	return 0;
}

static int mt9v024_get_selection(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_selection *sel)
{
	struct mt9v024 *mt9v024 = to_mt9v024(sd);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__mt9v024_get_pad_crop(mt9v024, cfg, sel->pad,
					sel->which);
	return 0;
}

static int mt9v024_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct mt9v024 *mt9v024 = to_mt9v024(subdev);
	int ret;

	if (enable) {
		ret = mt9v024_change_mode(mt9v024, mt9v024->current_mode);//Do the non-match test
		if (ret < 0) {
			dev_err(mt9v024->dev, "could not set mode %d\n",
				mt9v024->current_mode);
			return ret;
		}else
			{
			printk("new mode index:%d",mt9v024->current_mode);
		}

/*		
		ret = v4l2_ctrl_handler_setup(&mt9v024->ctrls);
		if (ret < 0) {
			dev_err(mt9v024->dev, "could not sync v4l2 controls\n");
			return ret;
		}
//		ret = mt9v024_write_reg(mt9v024, MT9V024_SYSTEM_CTRL0,
//				       MT9V024_SYSTEM_CTRL0_START);
*/
		ret = mt9v024_write_reg(mt9v024, MT9V024_SYSTEM_CTRL0,
				       MT9V024_SYSTEM_CTRL0_START);

		if (ret < 0)
			return ret;
	} else {
		ret = mt9v024_write_reg(mt9v024, MT9V024_SYSTEM_CTRL0,
				       MT9V024_SYSTEM_CTRL0_STOP);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static struct v4l2_subdev_core_ops mt9v024_core_ops = {
	.s_power = mt9v024_s_power,
};

static struct v4l2_subdev_video_ops mt9v024_video_ops = {
	.s_stream = mt9v024_s_stream,
};

static struct v4l2_subdev_pad_ops mt9v024_subdev_pad_ops = {
	.enum_mbus_code = mt9v024_enum_mbus_code,
	.enum_frame_size = mt9v024_enum_frame_size,
	.get_fmt = mt9v024_get_format,
	.set_fmt = mt9v024_set_format,
	.get_selection = mt9v024_get_selection,
};

static struct v4l2_subdev_ops mt9v024_subdev_ops = {
	.core = &mt9v024_core_ops,
	.video = &mt9v024_video_ops,
	.pad = &mt9v024_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops mt9v024_subdev_internal_ops = {
};

static int mt9v024_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *endpoint;
	struct mt9v024 *mt9v024;
	u16 chip_id;
	int ret;

	client->addr = 0x90;

	dev_dbg(dev, "%s: Enter, i2c addr = 0x%x\n", __func__, client->addr);
	
	mt9v024 = devm_kzalloc(dev, sizeof(struct mt9v024), GFP_KERNEL);
	if (!mt9v024)
		return -ENOMEM;

	mt9v024->i2c_client = client;
	mt9v024->dev = dev;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_of_parse_endpoint(endpoint, &mt9v024->ep);
	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}
	if (mt9v024->ep.bus_type != V4L2_MBUS_CSI2) {
		dev_err(dev, "invalid bus type, must be CSI2\n");
		of_node_put(endpoint);
		return -EINVAL;
	}
	of_node_put(endpoint);

	/* get system clock (xclk) */
	mt9v024->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(mt9v024->xclk)) {
		dev_err(dev, "could not get xclk");
		return PTR_ERR(mt9v024->xclk);
	}

	ret = of_property_read_u32(dev->of_node, "clock-frequency",
				    &mt9v024->xclk_freq);
	if (ret) {
		dev_err(dev, "could not get xclk frequency\n");
		return ret;
	}

	mt9v024->io_regulator = devm_regulator_get(dev, "vdddo");
	if (IS_ERR(mt9v024->io_regulator)) {
		dev_err(dev, "cannot get io regulator\n");
		return PTR_ERR(mt9v024->io_regulator);
	}

	ret = regulator_set_voltage(mt9v024->io_regulator,
				    MT9V024_VOLTAGE_DIGITAL_IO,
				    MT9V024_VOLTAGE_DIGITAL_IO);
	if (ret < 0) {
		dev_err(dev, "cannot set io voltage\n");
		return ret;
	}

	mt9v024->core_regulator = devm_regulator_get(dev, "vddd");
	if (IS_ERR(mt9v024->core_regulator)) {
		dev_err(dev, "cannot get core regulator\n");
		return PTR_ERR(mt9v024->core_regulator);
	}

	ret = regulator_set_voltage(mt9v024->core_regulator,
				    MT9V024_VOLTAGE_DIGITAL_CORE,
				    MT9V024_VOLTAGE_DIGITAL_CORE);
	if (ret < 0) {
		dev_err(dev, "cannot set core voltage\n");
		return ret;
	}

	mt9v024->analog_regulator = devm_regulator_get(dev, "vdda");
	if (IS_ERR(mt9v024->analog_regulator)) {
		dev_err(dev, "cannot get analog regulator\n");
		return PTR_ERR(mt9v024->analog_regulator);
	}

	ret = regulator_set_voltage(mt9v024->analog_regulator,
				    MT9V024_VOLTAGE_ANALOG,
				    MT9V024_VOLTAGE_ANALOG);
	if (ret < 0) {
		dev_err(dev, "cannot set analog voltage\n");
		return ret;
	}

	mt9v024->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(mt9v024->enable_gpio)) {
		dev_err(dev, "cannot get enable gpio\n");
		return PTR_ERR(mt9v024->enable_gpio);
	}

	mt9v024->rst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(mt9v024->rst_gpio)) {
		dev_err(dev, "cannot get reset gpio\n");
		return PTR_ERR(mt9v024->rst_gpio);
	}

	mutex_init(&mt9v024->power_lock);

	v4l2_ctrl_handler_init(&mt9v024->ctrls, 7);
	mt9v024->saturation = v4l2_ctrl_new_std(&mt9v024->ctrls, &mt9v024_ctrl_ops,
				V4L2_CID_SATURATION, -4, 4, 1, 0);
	mt9v024->hflip = v4l2_ctrl_new_std(&mt9v024->ctrls, &mt9v024_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);
	mt9v024->vflip = v4l2_ctrl_new_std(&mt9v024->ctrls, &mt9v024_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	mt9v024->autogain = v4l2_ctrl_new_std(&mt9v024->ctrls, &mt9v024_ctrl_ops,
				V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	mt9v024->autoexposure = v4l2_ctrl_new_std_menu(&mt9v024->ctrls,
				&mt9v024_ctrl_ops, V4L2_CID_EXPOSURE_AUTO,
				V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);
	mt9v024->awb = v4l2_ctrl_new_std(&mt9v024->ctrls, &mt9v024_ctrl_ops,
				V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);
	mt9v024->pattern = v4l2_ctrl_new_std_menu_items(&mt9v024->ctrls,
				&mt9v024_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(mt9v024_test_pattern_menu) - 1, 0, 0,
				mt9v024_test_pattern_menu);

	mt9v024->sd.ctrl_handler = &mt9v024->ctrls;

	if (mt9v024->ctrls.error) {
		dev_err(dev, "%s: control initialization error %d\n",
		       __func__, mt9v024->ctrls.error);
		ret = mt9v024->ctrls.error;
		goto free_ctrl;
	}

	v4l2_i2c_subdev_init(&mt9v024->sd, client, &mt9v024_subdev_ops);
	mt9v024->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	mt9v024->pad.flags = MEDIA_PAD_FL_SOURCE;
	mt9v024->sd.internal_ops = &mt9v024_subdev_internal_ops;

	ret = media_entity_init(&mt9v024->sd.entity, 1, &mt9v024->pad, 0);
	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		goto free_ctrl;
	}

	mt9v024->sd.dev = &client->dev;
	ret = v4l2_async_register_subdev(&mt9v024->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	ret = mt9v024_s_power(&mt9v024->sd, true);
	if (ret < 0) {
		dev_err(dev, "could not power up MT9V024\n");
		goto unregister_subdev;
	}

	ret = mt9v024_read_reg(mt9v024, MT9V024_CHIP_ID, &chip_id);
	if (ret < 0 || chip_id != MT9V024_CHIP_ID_WORD) {
		dev_err(dev, "could not read ID high,%x\n",chip_id);
		ret = -ENODEV;
		goto power_down;
	}

	dev_info(dev, "MT9V024 detected at address 0x%x,ID:0x%x\n", client->addr,chip_id);

	mt9v024_s_power(&mt9v024->sd, false);

	mt9v024_entity_init_cfg(&mt9v024->sd, NULL);

	return 0;

power_down:
	mt9v024_s_power(&mt9v024->sd, false);
unregister_subdev:
	v4l2_async_unregister_subdev(&mt9v024->sd);
free_entity:
	media_entity_cleanup(&mt9v024->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&mt9v024->ctrls);
	mutex_destroy(&mt9v024->power_lock);

	return ret;
}


static int mt9v024_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mt9v024 *mt9v024 = to_mt9v024(sd);

	v4l2_async_unregister_subdev(&mt9v024->sd);
	media_entity_cleanup(&mt9v024->sd.entity);
	v4l2_ctrl_handler_free(&mt9v024->ctrls);
	mutex_destroy(&mt9v024->power_lock);

	return 0;
}


static const struct i2c_device_id mt9v024_id[] = {
	{ "mt9v024", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, mt9v024_id);

static const struct of_device_id mt9v024_of_match[] = {
	{ .compatible = "aptina,mt9v024" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt9v024_of_match);

static struct i2c_driver mt9v024_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(mt9v024_of_match),
		.name  = "mt9v024",
	},
	.probe  = mt9v024_probe,
	.remove = mt9v024_remove,
	.id_table = mt9v024_id,
};

module_i2c_driver(mt9v024_i2c_driver);

MODULE_DESCRIPTION("Omnivision MT9V024 Camera Driver");
MODULE_AUTHOR("Todor Tomov <todor.tomov@linaro.org>");
MODULE_LICENSE("GPL v2");
