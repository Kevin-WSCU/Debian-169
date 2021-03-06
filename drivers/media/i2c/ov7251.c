/*
 * Driver for the OV7251 camera sensor.
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015 By Tech Design S.L. All Rights Reserved.
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Based on:
 * - the OV7251 driver from QC msm-3.10 kernel on codeaurora.org:
 *   https://us.codeaurora.org/cgit/quic/la/kernel/msm-3.10/tree/drivers/
 *       media/platform/msm/camera_v2/sensor/ov7251.c?h=LA.BR.1.2.4_rb1.41
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

static DEFINE_MUTEX(ov7251_lock);

/* HACKs here! */

#include <../drivers/media/platform/msm/cci/msm_cci.h>

#ifdef dev_dbg
	#undef dev_dbg
	#define dev_dbg dev_err
#endif

#define OV7251_VOLTAGE_ANALOG               2800000
#define OV7251_VOLTAGE_DIGITAL_CORE         1500000
#define OV7251_VOLTAGE_DIGITAL_IO           1800000


#define OV7251_SYSTEM_CTRL0		0x0100
#define	OV7251_SYSTEM_CTRL0_START	0x01
#define OV7251_SYSTEM_CTRL0_STOP 0x00

#define OV7251_CHIP_ID_HIGH		0x300A
#define		OV7251_CHIP_ID_HIGH_BYTE	0x77
#define OV7251_CHIP_ID_LOW		0x300B
#define		OV7251_CHIP_ID_LOW_BYTE		0x50


#define OV7251_AWB_MANUAL_CONTROL	0x3406
#define		OV7251_AWB_MANUAL_ENABLE	BIT(0)
#define OV7251_AEC_PK_MANUAL		0x3503
#define		OV7251_AEC_MANUAL_ENABLE	BIT(0)
#define		OV7251_AGC_MANUAL_ENABLE	BIT(1)
#define OV7251_TIMING_TC_REG20		0x3820
#define		OV7251_SENSOR_VFLIP		BIT(1)
#define		OV7251_ISP_VFLIP		BIT(2)
#define OV7251_TIMING_TC_REG21		0x3821
#define		OV7251_SENSOR_MIRROR		BIT(1)
#define OV7251_PRE_ISP_TEST_SETTING_1	0x503d
#define		OV7251_TEST_PATTERN_MASK	0x3
#define		OV7251_SET_TEST_PATTERN(x)	((x) & OV7251_TEST_PATTERN_MASK)
#define		OV7251_TEST_PATTERN_ENABLE	BIT(7)
#define OV7251_SDE_SAT_U		0x5583
#define OV7251_SDE_SAT_V		0x5584

enum ov7251_mode {
	OV7251_MODE_MIN = 0,
	OV7251_MODE_VGA = 0,
	OV7251_MODE_SXGA = 1,
	OV7251_MODE_1080P = 2,
	OV7251_MODE_FULL = 3,
	OV7251_MODE_MAX = 3
};

struct reg_value {
	u16 reg;
	u8 val;
};

struct ov7251_mode_info {
	enum ov7251_mode mode;
	u32 width;
	u32 height;
	struct reg_value *data;
	u32 data_size;
};

struct ov7251 {
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

	enum ov7251_mode current_mode;

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

static inline struct ov7251 *to_ov7251(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov7251, sd);
}
/*Some controllers(MIPI receiver detect the LPxx transaction),so stay in standby after the initial setting*/
static struct reg_value OV7251_VGA_100fps[] =
{

	{0x0103 ,0x01},
	{0x0100 ,0x00},
	{0x3005 ,0x00},
	{0x3012 ,0xc0},
	{0x3013 ,0xd2},
	{0x3014 ,0x04},
	{0x3016 ,0x10},
	{0x3017 ,0x00},
	{0x3018 ,0x00},
	{0x301a ,0x00},
	{0x301b ,0x00},
	{0x301c ,0x00},
	{0x3023 ,0x05},
	{0x3037 ,0xf0},
	{0x3098 ,0x04},
	{0x3099 ,0x28},
	{0x309a ,0x05},
	{0x309b ,0x04},
	{0x30b0 ,0x0a},
	{0x30b1 ,0x01},
	{0x30b3 ,0x64},
	{0x30b4 ,0x03},
	{0x30b5 ,0x05},
	{0x3106 ,0xda},	
	{0x3500 ,0x00},
	{0x3501 ,0x1f},
	{0x3502 ,0x40},
	{0x3503 ,0x07},	
	{0x3509 ,0x10},
	{0x350b ,0x10},
	{0x3600 ,0x1c},
	{0x3602 ,0x62},
	{0x3620 ,0xb7},
	{0x3622 ,0x04},
	{0x3626 ,0x21},
	{0x3627 ,0x30},
	{0x3630 ,0x44},
	{0x3631 ,0x35},
	{0x3634 ,0x60},
	{0x3636 ,0x00},
	{0x3662 ,0x01},
	{0x3663 ,0x70},
	{0x3664 ,0xf0},
	{0x3666 ,0x0a},
	{0x3669 ,0x1a},
	{0x366a ,0x00},
	{0x366b ,0x50},
	{0x3673 ,0x01},
	{0x3674 ,0xff},
	{0x3675 ,0x03},
	{0x3705 ,0xc1},
	{0x3709 ,0x40},
	{0x373c ,0x08},
	{0x3742 ,0x00},
	{0x3757 ,0xb3},
	{0x3788 ,0x00},
	{0x37a8 ,0x01},
	{0x37a9 ,0xc0},
	{0x3800 ,0x00},
	{0x3801 ,0x04},
	{0x3802 ,0x00},
	{0x3803 ,0x04},
	{0x3804 ,0x02},
	{0x3805 ,0x8b},
	{0x3806 ,0x01},
	{0x3807 ,0xeb},
	{0x3808 ,0x02},
	{0x3809 ,0x80},
	{0x380a ,0x01},
	{0x380b ,0xe0},
	{0x380c ,0x03},
	{0x380d ,0xa0},
	{0x380e ,0x02},//100fps
	{0x380f ,0x0a},	
	{0x3810 ,0x00},
	{0x3811 ,0x04},
	{0x3812 ,0x00},
	{0x3813 ,0x05},
	{0x3814 ,0x11},
	{0x3815 ,0x11},
	{0x3820 ,0x40},
	{0x3821 ,0x00},
	{0x382f ,0x0e},
	{0x3832 ,0x00},
	{0x3833 ,0x05},
	{0x3834 ,0x00},
	{0x3835 ,0x0c},
	{0x3837 ,0x00},
	{0x3b80 ,0x00},
	{0x3b81 ,0xa5},
	{0x3b82 ,0x10},
	{0x3b83 ,0x00},
	{0x3b84 ,0x08},
	{0x3b85 ,0x00},
	{0x3b86 ,0x01},
	{0x3b87 ,0x00},
	{0x3b88 ,0x00},
	{0x3b89 ,0x00},
	{0x3b8a ,0x00},
	{0x3b8b ,0x05},
	{0x3b8c ,0x00},
	{0x3b8d ,0x00},
	{0x3b8e ,0x00},
	{0x3b8f ,0x1a},
	{0x3b94 ,0x05},
	{0x3b95 ,0xf2},
	{0x3b96 ,0x40},
	{0x3c00 ,0x89},
	{0x3c01 ,0x63},
	{0x3c02 ,0x01},
	{0x3c03 ,0x00},
	{0x3c04 ,0x00},
	{0x3c05 ,0x03},
	{0x3c06 ,0x00},
	{0x3c07 ,0x06},
	{0x3c0c ,0x01},
	{0x3c0d ,0xd0},
	{0x3c0e ,0x02},
	{0x3c0f ,0x0a},
	{0x4001 ,0x42},
	{0x4004 ,0x04},
	{0x4005 ,0x00},
	{0x404e ,0x01},
	{0x4300 ,0xff},
	{0x4301 ,0x00},
	{0x4501 ,0x48},
	{0x4600 ,0x00},
	{0x4601 ,0x4e},
	{0x4801 ,0x0f},
	{0x4806 ,0x0f},
	{0x4819 ,0xaa},
	{0x4823 ,0x3e},
	{0x4837 ,0x19},
	{0x4a0d ,0x00},
	{0x4a47 ,0x7f},
	{0x4a49 ,0xf0},
	{0x4a4b ,0x30},
	{0x5000 ,0x85},
	{0x5001 ,0x80}
	//{0x0100 ,0x01},

};

static struct reg_value ov7251_setting_sxga[] = {
	{ 0x3612, 0xa9 },
	{ 0x3614, 0x50 },
	{ 0x3618, 0x00 },
	{ 0x3034, 0x18 },
	{ 0x3035, 0x21 },
	{ 0x3036, 0x70 },
	{ 0x3600, 0x09 },
	{ 0x3601, 0x43 },
	{ 0x3708, 0x66 },
	{ 0x370c, 0xc3 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x06 },
	{ 0x3804, 0x0a },
	{ 0x3805, 0x3f },
	{ 0x3806, 0x07 },
	{ 0x3807, 0x9d },
	{ 0x3808, 0x05 },
	{ 0x3809, 0x00 },
	{ 0x380a, 0x03 },
	{ 0x380b, 0xc0 },
	{ 0x380c, 0x07 },
	{ 0x380d, 0x68 },
	{ 0x380e, 0x03 },
	{ 0x380f, 0xd8 },
	{ 0x3813, 0x06 },
	{ 0x3814, 0x31 },
	{ 0x3815, 0x31 },
	{ 0x3820, 0x47 },
	{ 0x3a02, 0x03 },
	{ 0x3a03, 0xd8 },
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0xf8 },
	{ 0x3a0a, 0x01 },
	{ 0x3a0b, 0xa4 },
	{ 0x3a0e, 0x02 },
	{ 0x3a0d, 0x02 },
	{ 0x3a14, 0x03 },
	{ 0x3a15, 0xd8 },
	{ 0x3a18, 0x00 },
	{ 0x4004, 0x02 },
	{ 0x4005, 0x18 },
	{ 0x4300, 0x32 },
	{ 0x4202, 0x00 }
};

static struct reg_value ov7251_setting_1080p[] = {
	{ 0x3612, 0xab },
	{ 0x3614, 0x50 },
	{ 0x3618, 0x04 },
	{ 0x3034, 0x18 },
	{ 0x3035, 0x11 },
	{ 0x3036, 0x54 },
	{ 0x3600, 0x08 },
	{ 0x3601, 0x33 },
	{ 0x3708, 0x63 },
	{ 0x370c, 0xc0 },
	{ 0x3800, 0x01 },
	{ 0x3801, 0x50 },
	{ 0x3802, 0x01 },
	{ 0x3803, 0xb2 },
	{ 0x3804, 0x08 },
	{ 0x3805, 0xef },
	{ 0x3806, 0x05 },
	{ 0x3807, 0xf1 },
	{ 0x3808, 0x07 },
	{ 0x3809, 0x80 },
	{ 0x380a, 0x04 },
	{ 0x380b, 0x38 },
	{ 0x380c, 0x09 },
	{ 0x380d, 0xc4 },
	{ 0x380e, 0x04 },
	{ 0x380f, 0x60 },
	{ 0x3813, 0x04 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3820, 0x47 },
	{ 0x4514, 0x88 },
	{ 0x3a02, 0x04 },
	{ 0x3a03, 0x60 },
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0x50 },
	{ 0x3a0a, 0x01 },
	{ 0x3a0b, 0x18 },
	{ 0x3a0e, 0x03 },
	{ 0x3a0d, 0x04 },
	{ 0x3a14, 0x04 },
	{ 0x3a15, 0x60 },
	{ 0x3a18, 0x00 },
	{ 0x4004, 0x06 },
	{ 0x4005, 0x18 },
	{ 0x4300, 0x32 },
	{ 0x4202, 0x00 },
	{ 0x4837, 0x0b }
};

static struct reg_value ov7251_setting_full[] = {
	{ 0x3612, 0xab },
	{ 0x3614, 0x50 },
	{ 0x3618, 0x04 },
	{ 0x3034, 0x18 },
	{ 0x3035, 0x11 },
	{ 0x3036, 0x54 },
	{ 0x3600, 0x08 },
	{ 0x3601, 0x33 },
	{ 0x3708, 0x63 },
	{ 0x370c, 0xc0 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x00 },
	{ 0x3804, 0x0a },
	{ 0x3805, 0x3f },
	{ 0x3806, 0x07 },
	{ 0x3807, 0x9f },
	{ 0x3808, 0x0a },
	{ 0x3809, 0x20 },
	{ 0x380a, 0x07 },
	{ 0x380b, 0x98 },
	{ 0x380c, 0x0b },
	{ 0x380d, 0x1c },
	{ 0x380e, 0x07 },
	{ 0x380f, 0xb0 },
	{ 0x3813, 0x06 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3820, 0x47 },
	{ 0x4514, 0x88 },
	{ 0x3a02, 0x07 },
	{ 0x3a03, 0xb0 },
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0x27 },
	{ 0x3a0a, 0x00 },
	{ 0x3a0b, 0xf6 },
	{ 0x3a0e, 0x06 },
	{ 0x3a0d, 0x08 },
	{ 0x3a14, 0x07 },
	{ 0x3a15, 0xb0 },
	{ 0x3a18, 0x01 },
	{ 0x4004, 0x06 },
	{ 0x4005, 0x18 },
	{ 0x4300, 0x32 },
	{ 0x4837, 0x0b },
	{ 0x4202, 0x00 }
};

static struct ov7251_mode_info ov7251_mode_info_data[OV7251_MODE_MAX + 1] = {
	{
		.mode = OV7251_MODE_VGA,
		.width = 640,
		.height = 480,
		.data = OV7251_VGA_100fps,
		.data_size = ARRAY_SIZE(OV7251_VGA_100fps)
	},
	{
		.mode = OV7251_MODE_SXGA,
		.width = 1280,
		.height = 960,
		.data = ov7251_setting_sxga,
		.data_size = ARRAY_SIZE(ov7251_setting_sxga)
	},
	{
		.mode = OV7251_MODE_1080P,
		.width = 1920,
		.height = 1080,
		.data = ov7251_setting_1080p,
		.data_size = ARRAY_SIZE(ov7251_setting_1080p)
	},
	{
		.mode = OV7251_MODE_FULL,
		.width = 2592,
		.height = 1944,
		.data = ov7251_setting_full,
		.data_size = ARRAY_SIZE(ov7251_setting_full)
	},
};

static int ov7251_regulators_enable(struct ov7251 *ov7251)
{
	int ret;

	ret = regulator_enable(ov7251->io_regulator);
	if (ret < 0) {
		dev_err(ov7251->dev, "set io voltage failed\n");
		return ret;
	}

	ret = regulator_enable(ov7251->core_regulator);
	if (ret) {
		dev_err(ov7251->dev, "set core voltage failed\n");
		goto err_disable_io;
	}

	ret = regulator_enable(ov7251->analog_regulator);
	if (ret) {
		dev_err(ov7251->dev, "set analog voltage failed\n");
		goto err_disable_core;
	}

	return 0;

err_disable_core:
	regulator_disable(ov7251->core_regulator);
err_disable_io:
	regulator_disable(ov7251->io_regulator);

	return ret;
}

static void ov7251_regulators_disable(struct ov7251 *ov7251)
{
	int ret;

	ret = regulator_disable(ov7251->analog_regulator);
	if (ret < 0)
		dev_err(ov7251->dev, "analog regulator disable failed\n");

	ret = regulator_disable(ov7251->core_regulator);
	if (ret < 0)
		dev_err(ov7251->dev, "core regulator disable failed\n");

	ret = regulator_disable(ov7251->io_regulator);
	if (ret < 0)
		dev_err(ov7251->dev, "io regulator disable failed\n");
}

/*
static int ov7251_write_reg_to(struct ov7251 *ov7251, u16 reg, u8 val, u16 i2c_addr)
{
	int ret;

	ret = msm_cci_ctrl_write(i2c_addr, reg, &val, 1);
	if (ret < 0)
		dev_err(ov7251->dev,
			"%s: write reg error %d on addr 0x%x: reg=0x%x, val=0x%x\n",
			__func__, ret, i2c_addr, reg, val);

	return ret;
}
*/
static int ov7251_write_reg(struct ov7251 *ov7251, u16 reg, u8 val)
{
	int ret;
	u16 i2c_addr = 0xc0;//ov7251->i2c_client->addr;

	ret = msm_cci_ctrl_write(i2c_addr, reg, &val, 1);
	if (ret < 0)
		dev_err(ov7251->dev,
			"%s: write reg error %d on addr 0x%x: reg=0x%x, val=0x%x\n",
			__func__, ret, i2c_addr, reg, val);

	return ret;
}

static int ov7251_read_reg(struct ov7251 *ov7251, u16 reg, u8 *val)
{
	u8 tmpval;
	int ret;
	u16 i2c_addr = ov7251->i2c_client->addr;

	ret = msm_cci_ctrl_read(i2c_addr, reg, &tmpval, 1);
	if (ret < 0) {
		dev_err(ov7251->dev,
			"%s: read reg error %d on addr 0x%x: reg=0x%x\n",
			__func__, ret, i2c_addr, reg);
		return ret;
	}

	*val = tmpval;

	return 0;
}

static int ov7251_set_aec_mode(struct ov7251 *ov7251, u32 mode)
{
	u8 val;
	int ret;

	ret = ov7251_read_reg(ov7251, OV7251_AEC_PK_MANUAL, &val);
	if (ret < 0)
		return ret;

	if (mode == V4L2_EXPOSURE_AUTO)
		val &= ~OV7251_AEC_MANUAL_ENABLE;
	else /* V4L2_EXPOSURE_MANUAL */
		val |= OV7251_AEC_MANUAL_ENABLE;

	return ov7251_write_reg(ov7251, OV7251_AEC_PK_MANUAL, val);
}

static int ov7251_set_agc_mode(struct ov7251 *ov7251, u32 enable)
{
	u8 val;
	int ret;

	ret = ov7251_read_reg(ov7251, OV7251_AEC_PK_MANUAL, &val);
	if (ret < 0)
		return ret;

	if (enable)
		val &= ~OV7251_AGC_MANUAL_ENABLE;
	else
		val |= OV7251_AGC_MANUAL_ENABLE;

	return ov7251_write_reg(ov7251, OV7251_AEC_PK_MANUAL, val);
}

static int ov7251_set_register_array(struct ov7251 *ov7251,
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

		ret = ov7251_write_reg(ov7251, reg, val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/*
static int ov7251_init(struct ov7251 *ov7251)
{
	struct reg_value *settings;
	u32 num_settings;

	settings = ov7251_global_init_setting;
	num_settings = ARRAY_SIZE(ov7251_global_init_setting);

	return ov7251_set_register_array(ov7251, settings, num_settings);
}
*/

static int ov7251_change_mode(struct ov7251 *ov7251, enum ov7251_mode mode)
{
	struct reg_value *settings;
	u32 num_settings;

	settings = ov7251_mode_info_data[mode].data;
	num_settings = ov7251_mode_info_data[mode].data_size;

	return ov7251_set_register_array(ov7251, settings, num_settings);
}

static int ov7251_set_power_on(struct ov7251 *ov7251)
{
	int ret;

	clk_set_rate(ov7251->xclk, ov7251->xclk_freq);

	ret = clk_prepare_enable(ov7251->xclk);
	if (ret < 0) {
		dev_err(ov7251->dev, "clk prepare enable failed\n");
		return ret;
	}

	ret = ov7251_regulators_enable(ov7251);
	if (ret < 0) {
		clk_disable_unprepare(ov7251->xclk);
		return ret;
	}

	usleep_range(5000, 15000);
	gpiod_set_value_cansleep(ov7251->enable_gpio, 0);

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ov7251->rst_gpio, 0);

	msleep(20);

	return ret;
}

static void ov7251_set_power_off(struct ov7251 *ov7251)
{
	gpiod_set_value_cansleep(ov7251->rst_gpio, 1);
	gpiod_set_value_cansleep(ov7251->enable_gpio, 1);
	ov7251_regulators_disable(ov7251);
	clk_disable_unprepare(ov7251->xclk);
}

static int ov7251_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov7251 *ov7251 = to_ov7251(sd);
	int ret = 0;

	mutex_lock(&ov7251->power_lock);

	if (on) {
		ret = msm_cci_ctrl_init();
		if (ret < 0)
			goto exit;
	}

	if (ov7251->power == !on) {
		/* Power state changes. */
		if (on) {
			mutex_lock(&ov7251_lock);

			ret = ov7251_set_power_on(ov7251);
			if (ret < 0) {
				dev_err(ov7251->dev, "could not set power %s\n",
					on ? "on" : "off");
				goto exit;
			}

//don't change sensor i2c address for this time

//			ret = ov7251_write_reg_to(ov7251, 0x0109,
//					       ov7251->i2c_client->addr, 0xc0);


			if (ret < 0) {
				dev_err(ov7251->dev,
					"could not change i2c address\n");
				ov7251_set_power_off(ov7251);
				mutex_unlock(&ov7251_lock);
				goto exit;
			}

			mutex_unlock(&ov7251_lock);
/*
			ret = ov7251_init(ov7251);
			if (ret < 0) {
				dev_err(ov7251->dev,
					"could not set init registers\n");
				ov7251_set_power_off(ov7251);
				goto exit;
			}
*/
			ret = ov7251_write_reg(ov7251, OV7251_SYSTEM_CTRL0,
					       OV7251_SYSTEM_CTRL0_STOP);
			if (ret < 0) {
				ov7251_set_power_off(ov7251);
				goto exit;
			}
		} else {
			ov7251_set_power_off(ov7251);
		}

		/* Update the power state. */
		ov7251->power = on ? true : false;
	}

exit:
	if (!on)
		msm_cci_ctrl_release();

	mutex_unlock(&ov7251->power_lock);

	return ret;
}


static int ov7251_set_saturation(struct ov7251 *ov7251, s32 value)
{
	u32 reg_value = (value * 0x10) + 0x40;
	int ret;

	ret = ov7251_write_reg(ov7251, OV7251_SDE_SAT_U, reg_value);
	if (ret < 0)
		return ret;

	ret = ov7251_write_reg(ov7251, OV7251_SDE_SAT_V, reg_value);

	return ret;
}

static int ov7251_set_hflip(struct ov7251 *ov7251, s32 value)
{
	u8 val;
	int ret;

	ret = ov7251_read_reg(ov7251, OV7251_TIMING_TC_REG21, &val);
	if (ret < 0)
		return ret;

	if (value == 0)
		val &= ~(OV7251_SENSOR_MIRROR);
	else
		val |= (OV7251_SENSOR_MIRROR);

	return ov7251_write_reg(ov7251, OV7251_TIMING_TC_REG21, val);
}

static int ov7251_set_vflip(struct ov7251 *ov7251, s32 value)
{
	u8 val;
	int ret;

	ret = ov7251_read_reg(ov7251, OV7251_TIMING_TC_REG20, &val);
	if (ret < 0)
		return ret;

	if (value == 0)
		val |= (OV7251_SENSOR_VFLIP | OV7251_ISP_VFLIP);
	else
		val &= ~(OV7251_SENSOR_VFLIP | OV7251_ISP_VFLIP);

	return ov7251_write_reg(ov7251, OV7251_TIMING_TC_REG20, val);
}

static int ov7251_set_test_pattern(struct ov7251 *ov7251, s32 value)
{
	u8 val;
	int ret;

	ret = ov7251_read_reg(ov7251, OV7251_PRE_ISP_TEST_SETTING_1, &val);
	if (ret < 0)
		return ret;

	if (value) {
		val &= ~OV7251_SET_TEST_PATTERN(OV7251_TEST_PATTERN_MASK);
		val |= OV7251_SET_TEST_PATTERN(value - 1);
		val |= OV7251_TEST_PATTERN_ENABLE;
	} else {
		val &= ~OV7251_TEST_PATTERN_ENABLE;
	}

	return ov7251_write_reg(ov7251, OV7251_PRE_ISP_TEST_SETTING_1, val);
}

static const char * const ov7251_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bars",
	"Pseudo-Random Data",
	"Color Square",
	"Black Image",
};

static int ov7251_set_awb(struct ov7251 *ov7251, s32 enable_auto)
{
	u8 val;
	int ret;

	ret = ov7251_read_reg(ov7251, OV7251_AWB_MANUAL_CONTROL, &val);
	if (ret < 0)
		return ret;

	if (enable_auto)
		val &= ~OV7251_AWB_MANUAL_ENABLE;
	else
		val |= OV7251_AWB_MANUAL_ENABLE;

	return ov7251_write_reg(ov7251, OV7251_AWB_MANUAL_CONTROL, val);
}

static int ov7251_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov7251 *ov7251 = container_of(ctrl->handler,
					     struct ov7251, ctrls);

	int ret = -EINVAL;

	return 0;	
	mutex_lock(&ov7251->power_lock);
	if (ov7251->power == 0) {
		mutex_unlock(&ov7251->power_lock);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_SATURATION:
		ret = ov7251_set_saturation(ov7251, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = ov7251_set_awb(ov7251, ctrl->val);
		break;
	case V4L2_CID_AUTOGAIN:
		ret = ov7251_set_agc_mode(ov7251, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = ov7251_set_aec_mode(ov7251, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov7251_set_test_pattern(ov7251, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = ov7251_set_hflip(ov7251, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = ov7251_set_vflip(ov7251, ctrl->val);
		break;
	}

	mutex_unlock(&ov7251->power_lock);

	return ret;
}

static struct v4l2_ctrl_ops ov7251_ctrl_ops = {
	.s_ctrl = ov7251_s_ctrl,
};

static int ov7251_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = { 0 };
	struct ov7251 *ov7251 = to_ov7251(subdev);

	dev_err(ov7251->dev, "%s: Enter\n", __func__);


	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 1920;
	fmt.format.height = 1080;

	v4l2_subdev_call(subdev, pad, set_fmt, cfg, &fmt);

	return 0;
}

static int ov7251_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ov7251 *ov7251 = to_ov7251(sd);

	if (code->index > 0)
		return -EINVAL;

	code->code = ov7251->fmt.code;

	return 0;
}

static int ov7251_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > OV7251_MODE_MAX)
		return -EINVAL;

	fse->min_width = ov7251_mode_info_data[fse->index].width;
	fse->max_width = ov7251_mode_info_data[fse->index].width;
	fse->min_height = ov7251_mode_info_data[fse->index].height;
	fse->max_height = ov7251_mode_info_data[fse->index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ov7251_get_pad_format(struct ov7251 *ov7251,
			struct v4l2_subdev_pad_config *cfg,
			unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&ov7251->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov7251->fmt;
	default:
		return NULL;
	}
}

static int ov7251_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct ov7251 *ov7251 = to_ov7251(sd);

	format->format = *__ov7251_get_pad_format(ov7251, cfg, format->pad,
						  format->which);
	return 0;
}

static struct v4l2_rect *
__ov7251_get_pad_crop(struct ov7251 *ov7251, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&ov7251->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov7251->crop;
	default:
		return NULL;
	}
}

static enum ov7251_mode ov7251_find_nearest_mode(struct ov7251 *ov7251,
						 int width, int height)
{
	int i;

	for (i = OV7251_MODE_MAX; i >= 0; i--) {
		if (ov7251_mode_info_data[i].width <= width &&
		    ov7251_mode_info_data[i].height <= height)
			break;
	}

	if (i < 0)
		i = 0;

	return (enum ov7251_mode)i;
}

static int ov7251_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct ov7251 *ov7251 = to_ov7251(sd);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	enum ov7251_mode new_mode;

	__crop = __ov7251_get_pad_crop(ov7251, cfg, format->pad,
			format->which);

	new_mode = ov7251_find_nearest_mode(ov7251,
			format->format.width, format->format.height);
	
	//printk("set format,new mode index:%d",new_mode);

	
	__crop->width = ov7251_mode_info_data[new_mode].width;
	__crop->height = ov7251_mode_info_data[new_mode].height;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		ov7251->current_mode = new_mode;

	__format = __ov7251_get_pad_format(ov7251, cfg, format->pad,
			format->which);
	__format->width = __crop->width;
	__format->height = __crop->height;
	__format->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	__format->field = V4L2_FIELD_NONE;
	__format->colorspace = V4L2_COLORSPACE_SRGB;

	format->format = *__format;

	return 0;
}

static int ov7251_get_selection(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_selection *sel)
{
	struct ov7251 *ov7251 = to_ov7251(sd);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__ov7251_get_pad_crop(ov7251, cfg, sel->pad,
					sel->which);
	return 0;
}

static int ov7251_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ov7251 *ov7251 = to_ov7251(subdev);
	int ret;

	if (enable) {
		ret = ov7251_change_mode(ov7251, ov7251->current_mode);//Do the non-match test
		if (ret < 0) {
			dev_err(ov7251->dev, "could not set mode %d\n",
				ov7251->current_mode);
			return ret;
		}else
			{
			printk("new mode index:%d",ov7251->current_mode);
		}

/*		
		ret = v4l2_ctrl_handler_setup(&ov7251->ctrls);
		if (ret < 0) {
			dev_err(ov7251->dev, "could not sync v4l2 controls\n");
			return ret;
		}
//		ret = ov7251_write_reg(ov7251, OV7251_SYSTEM_CTRL0,
//				       OV7251_SYSTEM_CTRL0_START);
*/
		ret = ov7251_write_reg(ov7251, OV7251_SYSTEM_CTRL0,
				       OV7251_SYSTEM_CTRL0_START);

		if (ret < 0)
			return ret;
	} else {
		ret = ov7251_write_reg(ov7251, OV7251_SYSTEM_CTRL0,
				       OV7251_SYSTEM_CTRL0_STOP);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static struct v4l2_subdev_core_ops ov7251_core_ops = {
	.s_power = ov7251_s_power,
};

static struct v4l2_subdev_video_ops ov7251_video_ops = {
	.s_stream = ov7251_s_stream,
};

static struct v4l2_subdev_pad_ops ov7251_subdev_pad_ops = {
	.enum_mbus_code = ov7251_enum_mbus_code,
	.enum_frame_size = ov7251_enum_frame_size,
	.get_fmt = ov7251_get_format,
	.set_fmt = ov7251_set_format,
	.get_selection = ov7251_get_selection,
};

static struct v4l2_subdev_ops ov7251_subdev_ops = {
	.core = &ov7251_core_ops,
	.video = &ov7251_video_ops,
	.pad = &ov7251_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov7251_subdev_internal_ops = {
};

static int ov7251_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *endpoint;
	struct ov7251 *ov7251;
	u8 chip_id_high, chip_id_low;
	int ret;

	dev_dbg(dev, "%s: Enter, i2c addr = 0x%x\n", __func__, client->addr);

	client->addr = 0xc0;
	
	ov7251 = devm_kzalloc(dev, sizeof(struct ov7251), GFP_KERNEL);
	if (!ov7251)
		return -ENOMEM;

	ov7251->i2c_client = client;
	ov7251->dev = dev;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_of_parse_endpoint(endpoint, &ov7251->ep);
	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}
	if (ov7251->ep.bus_type != V4L2_MBUS_CSI2) {
		dev_err(dev, "invalid bus type, must be CSI2\n");
		of_node_put(endpoint);
		return -EINVAL;
	}
	of_node_put(endpoint);

	/* get system clock (xclk) */
	ov7251->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(ov7251->xclk)) {
		dev_err(dev, "could not get xclk");
		return PTR_ERR(ov7251->xclk);
	}

	ret = of_property_read_u32(dev->of_node, "clock-frequency",
				    &ov7251->xclk_freq);
	if (ret) {
		dev_err(dev, "could not get xclk frequency\n");
		return ret;
	}

	ov7251->io_regulator = devm_regulator_get(dev, "vdddo");
	if (IS_ERR(ov7251->io_regulator)) {
		dev_err(dev, "cannot get io regulator\n");
		return PTR_ERR(ov7251->io_regulator);
	}

	ret = regulator_set_voltage(ov7251->io_regulator,
				    OV7251_VOLTAGE_DIGITAL_IO,
				    OV7251_VOLTAGE_DIGITAL_IO);
	if (ret < 0) {
		dev_err(dev, "cannot set io voltage\n");
		return ret;
	}

	ov7251->core_regulator = devm_regulator_get(dev, "vddd");
	if (IS_ERR(ov7251->core_regulator)) {
		dev_err(dev, "cannot get core regulator\n");
		return PTR_ERR(ov7251->core_regulator);
	}

	ret = regulator_set_voltage(ov7251->core_regulator,
				    OV7251_VOLTAGE_DIGITAL_CORE,
				    OV7251_VOLTAGE_DIGITAL_CORE);
	if (ret < 0) {
		dev_err(dev, "cannot set core voltage\n");
		return ret;
	}

	ov7251->analog_regulator = devm_regulator_get(dev, "vdda");
	if (IS_ERR(ov7251->analog_regulator)) {
		dev_err(dev, "cannot get analog regulator\n");
		return PTR_ERR(ov7251->analog_regulator);
	}

	ret = regulator_set_voltage(ov7251->analog_regulator,
				    OV7251_VOLTAGE_ANALOG,
				    OV7251_VOLTAGE_ANALOG);
	if (ret < 0) {
		dev_err(dev, "cannot set analog voltage\n");
		return ret;
	}

	ov7251->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ov7251->enable_gpio)) {
		dev_err(dev, "cannot get enable gpio\n");
		return PTR_ERR(ov7251->enable_gpio);
	}

	ov7251->rst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ov7251->rst_gpio)) {
		dev_err(dev, "cannot get reset gpio\n");
		return PTR_ERR(ov7251->rst_gpio);
	}

	mutex_init(&ov7251->power_lock);

	v4l2_ctrl_handler_init(&ov7251->ctrls, 7);
	ov7251->saturation = v4l2_ctrl_new_std(&ov7251->ctrls, &ov7251_ctrl_ops,
				V4L2_CID_SATURATION, -4, 4, 1, 0);
	ov7251->hflip = v4l2_ctrl_new_std(&ov7251->ctrls, &ov7251_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);
	ov7251->vflip = v4l2_ctrl_new_std(&ov7251->ctrls, &ov7251_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	ov7251->autogain = v4l2_ctrl_new_std(&ov7251->ctrls, &ov7251_ctrl_ops,
				V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	ov7251->autoexposure = v4l2_ctrl_new_std_menu(&ov7251->ctrls,
				&ov7251_ctrl_ops, V4L2_CID_EXPOSURE_AUTO,
				V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);
	ov7251->awb = v4l2_ctrl_new_std(&ov7251->ctrls, &ov7251_ctrl_ops,
				V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);
	ov7251->pattern = v4l2_ctrl_new_std_menu_items(&ov7251->ctrls,
				&ov7251_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(ov7251_test_pattern_menu) - 1, 0, 0,
				ov7251_test_pattern_menu);

	ov7251->sd.ctrl_handler = &ov7251->ctrls;

	if (ov7251->ctrls.error) {
		dev_err(dev, "%s: control initialization error %d\n",
		       __func__, ov7251->ctrls.error);
		ret = ov7251->ctrls.error;
		goto free_ctrl;
	}

	v4l2_i2c_subdev_init(&ov7251->sd, client, &ov7251_subdev_ops);
	ov7251->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov7251->pad.flags = MEDIA_PAD_FL_SOURCE;
	ov7251->sd.internal_ops = &ov7251_subdev_internal_ops;

	ret = media_entity_init(&ov7251->sd.entity, 1, &ov7251->pad, 0);
	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		goto free_ctrl;
	}

	ov7251->sd.dev = &client->dev;
	ret = v4l2_async_register_subdev(&ov7251->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	ret = ov7251_s_power(&ov7251->sd, true);
	if (ret < 0) {
		dev_err(dev, "could not power up OV7251\n");
		goto unregister_subdev;
	}

	ret = ov7251_read_reg(ov7251, OV7251_CHIP_ID_HIGH, &chip_id_high);
	if (ret < 0 || chip_id_high != OV7251_CHIP_ID_HIGH_BYTE) {
		dev_err(dev, "could not read ID high\n");
		ret = -ENODEV;
		goto power_down;
	}
	ret = ov7251_read_reg(ov7251, OV7251_CHIP_ID_LOW, &chip_id_low);
	if (ret < 0 || chip_id_low != OV7251_CHIP_ID_LOW_BYTE) {
		dev_err(dev, "could not read ID low\n");
		ret = -ENODEV;
		goto power_down;
	}

	dev_info(dev, "OV7251 detected at address 0x%x,ID:0x%x\n", client->addr,chip_id_high<<8|chip_id_low);

	ov7251_s_power(&ov7251->sd, false);

	ov7251_entity_init_cfg(&ov7251->sd, NULL);

	return 0;

power_down:
	ov7251_s_power(&ov7251->sd, false);
unregister_subdev:
	v4l2_async_unregister_subdev(&ov7251->sd);
free_entity:
	media_entity_cleanup(&ov7251->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&ov7251->ctrls);
	mutex_destroy(&ov7251->power_lock);

	return ret;
}


static int ov7251_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov7251 *ov7251 = to_ov7251(sd);

	v4l2_async_unregister_subdev(&ov7251->sd);
	media_entity_cleanup(&ov7251->sd.entity);
	v4l2_ctrl_handler_free(&ov7251->ctrls);
	mutex_destroy(&ov7251->power_lock);

	return 0;
}


static const struct i2c_device_id ov7251_id[] = {
	{ "ov7251", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ov7251_id);

static const struct of_device_id ov7251_of_match[] = {
	{ .compatible = "ovti,ov7251" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov7251_of_match);

static struct i2c_driver ov7251_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ov7251_of_match),
		.name  = "ov7251",
	},
	.probe  = ov7251_probe,
	.remove = ov7251_remove,
	.id_table = ov7251_id,
};

module_i2c_driver(ov7251_i2c_driver);

MODULE_DESCRIPTION("Omnivision OV7251 Camera Driver");
MODULE_AUTHOR("Todor Tomov <todor.tomov@linaro.org>");
MODULE_LICENSE("GPL v2");
