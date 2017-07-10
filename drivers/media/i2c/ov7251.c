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


#define OV7251_SYSTEM_CTRL0		0x3000
#define	OV7251_SYSTEM_CTRL0_START	0x00
#define OV7251_SYSTEM_CTRL0_STOP 0x01 //Standby

#define OV7251_CHIP_ID_HIGH		0x3384
#define		OV7251_CHIP_ID_HIGH_BYTE	0x85
#define OV7251_CHIP_ID_LOW		0x3385
#define		OV7251_CHIP_ID_LOW_BYTE		0x01


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
	OV7251_MODE_1080P = 0,
	OV7251_MODE_MAX = 0
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

static struct reg_value ov7251_global_init_setting[] = {
	{ 0x3103, 0x11 },
	{ 0x3008, 0x82 },
	{ 0x3008, 0x42 },
	{ 0x3103, 0x03 },
	{ 0x3503, 0x07 },
	{ 0x3002, 0x1c },
	{ 0x3006, 0xc3 },
	{ 0x300e, 0x45 },
	{ 0x3017, 0x00 },
	{ 0x3018, 0x00 },
	{ 0x302e, 0x0b },
	{ 0x3037, 0x13 },
	{ 0x3108, 0x01 },
	{ 0x3611, 0x06 },
	{ 0x3500, 0x00 },
	{ 0x3501, 0x01 },
	{ 0x3502, 0x00 },
	{ 0x350a, 0x00 },
	{ 0x350b, 0x3f },
	{ 0x3620, 0x33 },
	{ 0x3621, 0xe0 },
	{ 0x3622, 0x01 },
	{ 0x3630, 0x2e },
	{ 0x3631, 0x00 },
	{ 0x3632, 0x32 },
	{ 0x3633, 0x52 },
	{ 0x3634, 0x70 },
	{ 0x3635, 0x13 },
	{ 0x3636, 0x03 },
	{ 0x3703, 0x5a },
	{ 0x3704, 0xa0 },
	{ 0x3705, 0x1a },
	{ 0x3709, 0x12 },
	{ 0x370b, 0x61 },
	{ 0x370f, 0x10 },
	{ 0x3715, 0x78 },
	{ 0x3717, 0x01 },
	{ 0x371b, 0x20 },
	{ 0x3731, 0x12 },
	{ 0x3901, 0x0a },
	{ 0x3905, 0x02 },
	{ 0x3906, 0x10 },
	{ 0x3719, 0x86 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x10 },
	{ 0x3812, 0x00 },
	{ 0x3821, 0x01 },
	{ 0x3824, 0x01 },
	{ 0x3826, 0x03 },
	{ 0x3828, 0x08 },
	{ 0x3a19, 0xf8 },
	{ 0x3c01, 0x34 },
	{ 0x3c04, 0x28 },
	{ 0x3c05, 0x98 },
	{ 0x3c07, 0x07 },
	{ 0x3c09, 0xc2 },
	{ 0x3c0a, 0x9c },
	{ 0x3c0b, 0x40 },
	{ 0x3c01, 0x34 },
	{ 0x4001, 0x02 },
	{ 0x4514, 0x00 },
	{ 0x4520, 0xb0 },
	{ 0x460b, 0x37 },
	{ 0x460c, 0x20 },
	{ 0x4818, 0x01 },
	{ 0x481d, 0xf0 },
	{ 0x481f, 0x50 },
	{ 0x4823, 0x70 },
	{ 0x4831, 0x14 },
	{ 0x5000, 0xa7 },
	{ 0x5001, 0x83 },
	{ 0x501d, 0x00 },
	{ 0x501f, 0x00 },
	{ 0x503d, 0x00 },
	{ 0x505c, 0x30 },
	{ 0x5181, 0x59 },
	{ 0x5183, 0x00 },
	{ 0x5191, 0xf0 },
	{ 0x5192, 0x03 },
	{ 0x5684, 0x10 },
	{ 0x5685, 0xa0 },
	{ 0x5686, 0x0c },
	{ 0x5687, 0x78 },
	{ 0x5a00, 0x08 },
	{ 0x5a21, 0x00 },
	{ 0x5a24, 0x00 },
	{ 0x3008, 0x02 },
	{ 0x3503, 0x00 },
	{ 0x5180, 0xff },
	{ 0x5181, 0xf2 },
	{ 0x5182, 0x00 },
	{ 0x5183, 0x14 },
	{ 0x5184, 0x25 },
	{ 0x5185, 0x24 },
	{ 0x5186, 0x09 },
	{ 0x5187, 0x09 },
	{ 0x5188, 0x0a },
	{ 0x5189, 0x75 },
	{ 0x518a, 0x52 },
	{ 0x518b, 0xea },
	{ 0x518c, 0xa8 },
	{ 0x518d, 0x42 },
	{ 0x518e, 0x38 },
	{ 0x518f, 0x56 },
	{ 0x5190, 0x42 },
	{ 0x5191, 0xf8 },
	{ 0x5192, 0x04 },
	{ 0x5193, 0x70 },
	{ 0x5194, 0xf0 },
	{ 0x5195, 0xf0 },
	{ 0x5196, 0x03 },
	{ 0x5197, 0x01 },
	{ 0x5198, 0x04 },
	{ 0x5199, 0x12 },
	{ 0x519a, 0x04 },
	{ 0x519b, 0x00 },
	{ 0x519c, 0x06 },
	{ 0x519d, 0x82 },
	{ 0x519e, 0x38 },
	{ 0x5381, 0x1e },
	{ 0x5382, 0x5b },
	{ 0x5383, 0x08 },
	{ 0x5384, 0x0a },
	{ 0x5385, 0x7e },
	{ 0x5386, 0x88 },
	{ 0x5387, 0x7c },
	{ 0x5388, 0x6c },
	{ 0x5389, 0x10 },
	{ 0x538a, 0x01 },
	{ 0x538b, 0x98 },
	{ 0x5300, 0x08 },
	{ 0x5301, 0x30 },
	{ 0x5302, 0x10 },
	{ 0x5303, 0x00 },
	{ 0x5304, 0x08 },
	{ 0x5305, 0x30 },
	{ 0x5306, 0x08 },
	{ 0x5307, 0x16 },
	{ 0x5309, 0x08 },
	{ 0x530a, 0x30 },
	{ 0x530b, 0x04 },
	{ 0x530c, 0x06 },
	{ 0x5480, 0x01 },
	{ 0x5481, 0x08 },
	{ 0x5482, 0x14 },
	{ 0x5483, 0x28 },
	{ 0x5484, 0x51 },
	{ 0x5485, 0x65 },
	{ 0x5486, 0x71 },
	{ 0x5487, 0x7d },
	{ 0x5488, 0x87 },
	{ 0x5489, 0x91 },
	{ 0x548a, 0x9a },
	{ 0x548b, 0xaa },
	{ 0x548c, 0xb8 },
	{ 0x548d, 0xcd },
	{ 0x548e, 0xdd },
	{ 0x548f, 0xea },
	{ 0x5490, 0x1d },
	{ 0x5580, 0x02 },
	{ 0x5583, 0x40 },
	{ 0x5584, 0x10 },
	{ 0x5589, 0x10 },
	{ 0x558a, 0x00 },
	{ 0x558b, 0xf8 },
	{ 0x5800, 0x3f },
	{ 0x5801, 0x16 },
	{ 0x5802, 0x0e },
	{ 0x5803, 0x0d },
	{ 0x5804, 0x17 },
	{ 0x5805, 0x3f },
	{ 0x5806, 0x0b },
	{ 0x5807, 0x06 },
	{ 0x5808, 0x04 },
	{ 0x5809, 0x04 },
	{ 0x580a, 0x06 },
	{ 0x580b, 0x0b },
	{ 0x580c, 0x09 },
	{ 0x580d, 0x03 },
	{ 0x580e, 0x00 },
	{ 0x580f, 0x00 },
	{ 0x5810, 0x03 },
	{ 0x5811, 0x08 },
	{ 0x5812, 0x0a },
	{ 0x5813, 0x03 },
	{ 0x5814, 0x00 },
	{ 0x5815, 0x00 },
	{ 0x5816, 0x04 },
	{ 0x5817, 0x09 },
	{ 0x5818, 0x0f },
	{ 0x5819, 0x08 },
	{ 0x581a, 0x06 },
	{ 0x581b, 0x06 },
	{ 0x581c, 0x08 },
	{ 0x581d, 0x0c },
	{ 0x581e, 0x3f },
	{ 0x581f, 0x1e },
	{ 0x5820, 0x12 },
	{ 0x5821, 0x13 },
	{ 0x5822, 0x21 },
	{ 0x5823, 0x3f },
	{ 0x5824, 0x68 },
	{ 0x5825, 0x28 },
	{ 0x5826, 0x2c },
	{ 0x5827, 0x28 },
	{ 0x5828, 0x08 },
	{ 0x5829, 0x48 },
	{ 0x582a, 0x64 },
	{ 0x582b, 0x62 },
	{ 0x582c, 0x64 },
	{ 0x582d, 0x28 },
	{ 0x582e, 0x46 },
	{ 0x582f, 0x62 },
	{ 0x5830, 0x60 },
	{ 0x5831, 0x62 },
	{ 0x5832, 0x26 },
	{ 0x5833, 0x48 },
	{ 0x5834, 0x66 },
	{ 0x5835, 0x44 },
	{ 0x5836, 0x64 },
	{ 0x5837, 0x28 },
	{ 0x5838, 0x66 },
	{ 0x5839, 0x48 },
	{ 0x583a, 0x2c },
	{ 0x583b, 0x28 },
	{ 0x583c, 0x26 },
	{ 0x583d, 0xae },
	{ 0x5025, 0x00 },
	{ 0x3a0f, 0x30 },
	{ 0x3a10, 0x28 },
	{ 0x3a1b, 0x30 },
	{ 0x3a1e, 0x26 },
	{ 0x3a11, 0x60 },
	{ 0x3a1f, 0x14 },
	{ 0x0601, 0x02 },
	{ 0x3008, 0x42 },
	{ 0x3008, 0x02 }
};


static struct reg_value ov7251_setting_1080p[] = {

	{0x3002, 0x01},
	{0x3005, 0x00},/*10BIT*/
	{0x3006, 0x00},
	{0x3007, 0x50},
	{0x3009, 0x01},
	{0x300a, 0x3c},/*10BIT*/
	{0x300f, 0x01},
	{0x3018, 0x65},
	{0x3019, 0x04},
	{0x301b, 0x4c},
	{0x301c, 0x04},
	{0x301d, 0x08},
	{0x301e, 0x02},

	{0x3036, 0x06},
	{0x3038, 0x08},
	{0x3039, 0x00},
	{0x303a, 0x40},
	{0x303b, 0x04},
	{0x303c, 0x0c},
	{0x303d, 0x00},
	{0x303e, 0x7c},
	{0x303f, 0x07},

	{0x3044, 0xe1},
	{0x3048, 0x33},

	{0x305C, 0x20},
	{0x305D, 0x00},
	{0x305E, 0x18},
	{0x305F, 0x00},
	{0x3063, 0x74},

	{0x3084, 0x0f},

	{0x3086, 0x10},
	{0x30A1, 0x44},
	{0x30cf, 0xe1},
	{0x30d0, 0x29},
	{0x30d2, 0x9b},
	{0x30d3, 0x01},

	{0x311d, 0x0a},
	{0x3123, 0x0f},
	{0x3126, 0xdf},
	{0x3147, 0x87},
	{0x31e0, 0x01},
	{0x31e1, 0x9e},
	{0x31e2, 0x01},
	{0x31e5, 0x05},
	{0x31e6, 0x05},
	{0x31e7, 0x3a},
	{0x31e8, 0x3a},

	{0x3203, 0xc8},
	{0x3207, 0x54},
	{0x3213, 0x16},
	{0x3215, 0xf6},
	{0x321a, 0x14},
	{0x321b, 0x51},
	{0x3229, 0xe7},
	{0x322a, 0xf0},
	{0x322b, 0x10},
	{0x3231, 0xe7},
	{0x3232, 0xf0},
	{0x3233, 0x10},
	{0x323c, 0xe8},
	{0x323d, 0x70},
	{0x3243, 0x08},
	{0x3244, 0xe1},
	{0x3245, 0x10},
	{0x3247, 0xe7},
	{0x3248, 0x60},
	{0x3249, 0x1e},
	{0x324b, 0x00},
	{0x324c, 0x41},
	{0x3250, 0x30},
	{0x3251, 0x0a},
	{0x3252, 0xff},
	{0x3253, 0xff},
	{0x3254, 0xff},
	{0x3255, 0x02},
	{0x3257, 0xf0},
	{0x325a, 0xa6},
	{0x325d, 0x14},
	{0x325e, 0x51},
	{0x3260, 0x00},
	{0x3261, 0x61},
	{0x3266, 0x30},
	{0x3267, 0x05},
	{0x3275, 0xe7},
	{0x3281, 0xea},
	{0x3282, 0x70},
	{0x3285, 0xff},
	{0x328a, 0xf0},
	{0x328d, 0xb6},
	{0x328e, 0x40},
	{0x3290, 0x42},
	{0x3291, 0x51},
	{0x3292, 0x1e},
	{0x3294, 0xc4},
	{0x3295, 0x20},
	{0x3297, 0x50},
	{0x3298, 0x31},
	{0x3299, 0x1f},
	{0x329b, 0xc0},
	{0x329c, 0x60},
	{0x329e, 0x4c},
	{0x329f, 0x71},
	{0x32a0, 0x1f},
	{0x32a2, 0xb6},
	{0x32a3, 0xc0},
	{0x32a4, 0x0b},
	{0x32a9, 0x24},
	{0x32aa, 0x41},
	{0x32b0, 0x25},
	{0x32b1, 0x51},
	{0x32b7, 0x1c},
	{0x32b8, 0xc1},
	{0x32b9, 0x12},
	{0x32be, 0x1d},
	{0x32bf, 0xd1},
	{0x32c0, 0x12},
	{0x32c2, 0xa8},
	{0x32c3, 0xc0},
	{0x32c4, 0x0a},
	{0x32c5, 0x1e},
	{0x32c6, 0x21},
	{0x32c9, 0xb0},
	{0x32ca, 0x40},
	{0x32cc, 0x26},
	{0x32cd, 0xa1},
	{0x32d0, 0xb6},
	{0x32d1, 0xc0},
	{0x32d2, 0x0b},
	{0x32d4, 0xe2},
	{0x32d5, 0x40},
	{0x32d8, 0x4e},
	{0x32d9, 0xa1},
	{0x32ec, 0xf0},

	{0x3303, 0x00},
	{0x3305, 0x03},
	{0x3314, 0x04},
	{0x3315, 0x01},
	{0x3316, 0x04},
	{0x3317, 0x04},
	{0x3318, 0x38},
	{0x3319, 0x04},
	{0x332c, 0x40},
	{0x332d, 0x20},
	{0x332e, 0x03},
	{0x333e, 0x0a},/*10BIT*/
	{0x333f, 0x0a},/*10BIT*/
	{0x3340, 0x03},
	{0x3341, 0x20},
	{0x3342, 0x25},
	{0x3343, 0x68},
	{0x3344, 0x20},
	{0x3345, 0x40},
	{0x3346, 0x28},
	{0x3347, 0x20},
	{0x3348, 0x18},
	{0x3349, 0x78},
	{0x334a, 0x28},
	{0x334e, 0xb4},
	{0x334f, 0x01},


};


static struct ov7251_mode_info ov7251_mode_info_data[OV7251_MODE_MAX + 1] = {
	{
		.mode = OV7251_MODE_1080P,
		.width = 1920,
		.height = 1080,
		.data = ov7251_setting_1080p,
		.data_size = ARRAY_SIZE(ov7251_setting_1080p)
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

static int ov7251_write_reg(struct ov7251 *ov7251, u16 reg, u8 val)
{
	int ret;
	u16 i2c_addr = 0x34;

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

static int ov7251_init(struct ov7251 *ov7251)
{
	struct reg_value *settings;
	u32 num_settings;

	settings = ov7251_global_init_setting;
	num_settings = ARRAY_SIZE(ov7251_global_init_setting);

	return ov7251_set_register_array(ov7251, settings, num_settings);
}

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

	client->addr = 0x34;
	
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

	ret = ov7251_read_reg(ov7251, OV7251_CHIP_ID_LOW, &chip_id_high);
	if (ret < 0 || chip_id_high != OV7251_CHIP_ID_LOW_BYTE) {
		dev_err(dev, "could not read ID high\n");
		ret = -ENODEV;
		goto power_down;
	}
	ret = ov7251_read_reg(ov7251, OV7251_CHIP_ID_HIGH, &chip_id_low);
	if (ret < 0 || chip_id_low != OV7251_CHIP_ID_HIGH_BYTE) {
		dev_err(dev, "could not read ID low\n");
		ret = -ENODEV;
		goto power_down;
	}

	dev_info(dev, "Sony IMX185 detected at address 0x%x,ID:0x%x\n", client->addr,chip_id_high<<8|chip_id_low);

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
