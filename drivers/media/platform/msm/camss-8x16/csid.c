/*
 * csid.c
 *
 * Qualcomm MSM Camera Subsystem - CSID Module
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2016 Linaro Ltd.
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
#define DEBUG

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "csid.h"
#include "camss.h"

#define MSM_CSID_NAME "msm_csid"

#define CAMSS_CSID_HW_VERSION		0x0
#define CAMSS_CSID_CORE_CTRL_0		0x004
#define CAMSS_CSID_CORE_CTRL_1		0x008
#define CAMSS_CSID_RST_CMD		0x00c
#define CAMSS_CSID_CID_LUT_VC_n(n)	(0x010 + 0x4 * (n))
#define CAMSS_CSID_CID_n_CFG(n)		(0x020 + 0x4 * (n))
#define CAMSS_CSID_IRQ_CLEAR_CMD	0x060
#define CAMSS_CSID_IRQ_MASK		0x064
#define CAMSS_CSID_IRQ_STATUS		0x068
#define CAMSS_CSID_TG_CTRL		0x0a0
#define CAMSS_CSID_TG_VC_CFG		0x0a4
#define CAMSS_CSID_TG_VC_CFG_H_BLANKING		0x3ff
#define CAMSS_CSID_TG_VC_CFG_V_BLANKING		0x7f
#define CAMSS_CSID_TG_DT_n_CGG_0(n)	(0x0ac + 0xc * (n))
#define CAMSS_CSID_TG_DT_n_CGG_1(n)	(0x0b0 + 0xc * (n))
#define CAMSS_CSID_TG_DT_n_CGG_2(n)	(0x0b4 + 0xc * (n))

#define DATA_TYPE_EMBEDDED_DATA_8BIT	0x12
#define DATA_TYPE_YUV422_8BIT		0x1e
#define DATA_TYPE_RAW_6BIT		0x28
#define DATA_TYPE_RAW_8BIT		0x2a
#define DATA_TYPE_RAW_10BIT		0x2b
#define DATA_TYPE_RAW_12BIT		0x2c

#define DECODE_FORMAT_UNCOMPRESSED_6_BIT	0x0
#define DECODE_FORMAT_UNCOMPRESSED_8_BIT	0x1
#define DECODE_FORMAT_UNCOMPRESSED_10_BIT	0x2
#define DECODE_FORMAT_UNCOMPRESSED_12_BIT	0x3
#define DECODE_FORMAT_DPCM_10_8_10		0x5

static const struct {
	u32 code;
	u32 uncompressed;
	u8 data_type;
	u8 decode_format;
	u8 uncompr_bpp;
} csid_input_fmts[] = {
	{
		MEDIA_BUS_FMT_UYVY8_2X8,
		MEDIA_BUS_FMT_UYVY8_2X8,
		DATA_TYPE_YUV422_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		16
	},
	{
		MEDIA_BUS_FMT_VYUY8_2X8,
		MEDIA_BUS_FMT_VYUY8_2X8,
		DATA_TYPE_YUV422_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		16
	},
	{
		MEDIA_BUS_FMT_YUYV8_2X8,
		MEDIA_BUS_FMT_YUYV8_2X8,
		DATA_TYPE_YUV422_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		16
	},
	{
		MEDIA_BUS_FMT_YVYU8_2X8,
		MEDIA_BUS_FMT_YVYU8_2X8,
		DATA_TYPE_YUV422_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		16
	},
	{
		MEDIA_BUS_FMT_SBGGR8_1X8,
		MEDIA_BUS_FMT_SBGGR8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8
	},
	{
		MEDIA_BUS_FMT_SGBRG8_1X8,
		MEDIA_BUS_FMT_SGBRG8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8
	},
	{
		MEDIA_BUS_FMT_SGRBG8_1X8,
		MEDIA_BUS_FMT_SGRBG8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8
	},
	{
		MEDIA_BUS_FMT_SRGGB8_1X8,
		MEDIA_BUS_FMT_SRGGB8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8
	},
	{
		MEDIA_BUS_FMT_SBGGR10_1X10,
		MEDIA_BUS_FMT_SBGGR10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10
	},
	{
		MEDIA_BUS_FMT_SGBRG10_1X10,
		MEDIA_BUS_FMT_SGBRG10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10
	},
	{
		MEDIA_BUS_FMT_SGRBG10_1X10,
		MEDIA_BUS_FMT_SGRBG10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10
	},
	{
		MEDIA_BUS_FMT_SRGGB10_1X10,
		MEDIA_BUS_FMT_SRGGB10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10
	},
	{
		MEDIA_BUS_FMT_SBGGR12_1X12,
		MEDIA_BUS_FMT_SBGGR12_1X12,
		DATA_TYPE_RAW_12BIT,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12
	},
	{
		MEDIA_BUS_FMT_SGBRG12_1X12,
		MEDIA_BUS_FMT_SGBRG12_1X12,
		DATA_TYPE_RAW_12BIT,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12
	},
	{
		MEDIA_BUS_FMT_SGRBG12_1X12,
		MEDIA_BUS_FMT_SGRBG12_1X12,
		DATA_TYPE_RAW_12BIT,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12
	},
	{
		MEDIA_BUS_FMT_SRGGB12_1X12,
		MEDIA_BUS_FMT_SRGGB12_1X12,
		DATA_TYPE_RAW_12BIT,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12
	},
	{
		MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8,
		MEDIA_BUS_FMT_SBGGR10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_DPCM_10_8_10,
		10
	},
	{
		MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8,
		MEDIA_BUS_FMT_SGBRG10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_DPCM_10_8_10,
		10
	},
	{
		MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8,
		MEDIA_BUS_FMT_SGRBG10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_DPCM_10_8_10,
		10
	},
	{
		MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8,
		MEDIA_BUS_FMT_SRGGB10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_DPCM_10_8_10,
		10
	}
};

/*
 * csid_isr - CSID module interrupt handler
 * @irq: Interrupt line
 * @dev: CSID device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t csid_isr(int irq, void *dev)
{
	struct csid_device *csid = dev;
	u32 value;

	value = readl_relaxed(csid->base + CAMSS_CSID_IRQ_STATUS);

	dev_dbg(to_device_index(csid, csid->id)," csid%d status = 0x%x\n",csid->id,value);
	
	writel_relaxed(value, csid->base + CAMSS_CSID_IRQ_CLEAR_CMD);

	if ((value >> 11) & 0x1)
		complete(&csid->reset_complete);

	return IRQ_HANDLED;
}

/*
 * csid_enable_clocks - Enable clocks for CSID module and
 * set clock rates where needed
 * @nclocks: Number of clocks in clock array
 * @clock: Clock array
 * @clock_rate: Clock rates array
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_enable_clocks(int nclocks, struct clk **clock, s32 *clock_rate)
{
	long clk_rate;
	int ret;
	int i;

	for (i = 0; i < nclocks; i++) {
		if (clock_rate[i]) {
			clk_rate = clk_round_rate(clock[i], clock_rate[i]);
			if (clk_rate < 0) {
				pr_err("clock round rate failed\n");
				ret = clk_rate;
				goto error;
			}
			ret = clk_set_rate(clock[i], clk_rate);
			if (ret < 0) {
				pr_err("clock set rate failed\n");
				goto error;
			}
		}
		ret = clk_prepare_enable(clock[i]);
		if (ret) {
			pr_err("clock enable failed\n");
			goto error;
		}
	}

	return 0;

error:
	for (i--; i >= 0; i--)
		clk_disable_unprepare(clock[i]);

	return ret;
}

/*
 * csid_disable_clocks - Disable clocks for CSID module
 * @nclocks: Number of clocks in clock array
 * @clock: Clock array
 */
static void csid_disable_clocks(int nclocks, struct clk **clock)
{
	int i;

	for (i = nclocks - 1; i >= 0; i--)
		clk_disable_unprepare(clock[i]);
}

/*
 * csid_set_power - Power on/off CSID module
 * @sd: CSID V4L2 subdevice
 * @on: Requested power state
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_set_power(struct v4l2_subdev *sd, int on)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	int ret;

	dev_dbg(to_device_index(csid, csid->id), "%s: Enter, csid%d on = %d\n",
		__func__, csid->id, on);

	if (on) {
		u32 hw_version;

		ret = regulator_enable(csid->vdda);
		if (ret < 0)
			return ret;

		ret = csid_enable_clocks(csid->nclocks, csid->clock,
					 csid->clock_rate);
		if (ret < 0)
			return ret;

		enable_irq(csid->irq);

		/* Reset */
		writel_relaxed(0x7fff, csid->base + CAMSS_CSID_RST_CMD);
		wait_for_completion(&csid->reset_complete);

		hw_version = readl_relaxed(csid->base + CAMSS_CSID_HW_VERSION);
		dev_dbg(to_device_index(csid, csid->id), "CSID HW Version = 0x%08x\n", hw_version);
	} else {
		disable_irq(csid->irq);

		csid_disable_clocks(csid->nclocks, csid->clock);

		ret = regulator_disable(csid->vdda);
		if (ret < 0)
			return ret;
	}

	dev_dbg(to_device_index(csid, csid->id), "%s: Exit, csid%d on = %d\n",
		__func__, csid->id, on);

	return 0;
}

/*
 * csid_get_uncompressed - map media bus format to uncompressed media bus format
 * @fmt media bus format code
 *
 * Return uncompressed media bus format
 */
static u32 csid_get_uncompressed(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(csid_input_fmts); i++)
		if (code == csid_input_fmts[i].code)
			break;

	return csid_input_fmts[i].uncompressed;
}

/*
 * csid_get_data_type - map media bus format to data type
 * @fmt media bus format code
 *
 * Return data type code
 */
static u8 csid_get_data_type(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(csid_input_fmts); i++)
		if (code == csid_input_fmts[i].code)
			break;

	return csid_input_fmts[i].data_type;
}

/*
 * csid_get_decode_format - map media but format to decode format
 * @fmt media bus format code
 *
 * Return decode format code
 */
static u8 csid_get_decode_format(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(csid_input_fmts); i++)
		if (code == csid_input_fmts[i].code)
			break;

	return csid_input_fmts[i].decode_format;
}

/*
 * csid_get_bpp - map media bus format to bits per pixel
 * @fmt media bus format code
 *
 * Return number of bits per pixel
 */
static u8 csid_get_bpp(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(csid_input_fmts); i++)
		if (code == csid_input_fmts[i].uncompressed)
			break;

	return csid_input_fmts[i].uncompr_bpp;
}


/*
 * csid_set_stream - Enable/disable streaming on CSID module
 * @sd: CSID V4L2 subdevice
 * @enable: Requested streaming state
 *
 * Main configuration of CSID module is also done here.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct csid_testgen_config *tg = &csid->testgen;

	dev_dbg(to_device_index(csid, csid->id), "%s: Enter, csid%d enable = %d\n",
		__func__, csid->id, enable);

	if (enable) {
		u8 vc = 0; /* TODO: How to get this from sensor? */
		u8 cid = vc * 4;
		u8 dt, dt_shift, df;
		u32 val;
		int ret;

		ret = v4l2_ctrl_handler_setup(&csid->ctrls);
		if (ret < 0) {
			dev_err(to_device_index(csid, csid->id),
				"could not sync v4l2 controls\n");
			return ret;
		}

		if (!tg->enabled &&
		    !media_entity_remote_pad(&csid->pads[MSM_CSID_PAD_SINK])) {
			return -ENOLINK;
		}

		dt = csid_get_data_type(csid->fmt[MSM_CSID_PAD_SRC].code);

		if (tg->enabled) {
			/* Config Test Generator */
			u8 bpp = csid_get_bpp(csid->fmt[MSM_CSID_PAD_SRC].code);
			u32 num_bytes_per_line =
					csid->fmt[MSM_CSID_PAD_SRC].width * bpp / 8;
			u32 num_lines = csid->fmt[MSM_CSID_PAD_SRC].height;

			/* 31:24 V blank, 23:13 H blank, 3:2 num of active DT */
			/* 1:0 VC */
			val = ((CAMSS_CSID_TG_VC_CFG_V_BLANKING & 0xff) << 24) |
			      ((CAMSS_CSID_TG_VC_CFG_H_BLANKING & 0x7ff) << 13);
			writel_relaxed(val, csid->base + CAMSS_CSID_TG_VC_CFG);

			/* 28:16 bytes per lines, 12:0 num of lines */
			val = ((num_bytes_per_line & 0x1fff) << 16) |
			      (num_lines & 0x1fff);
			writel_relaxed(val, csid->base +
				       CAMSS_CSID_TG_DT_n_CGG_0(0));

			/* 5:0 data type */
			val = dt;
			writel_relaxed(val, csid->base +
				       CAMSS_CSID_TG_DT_n_CGG_1(0));

			/* 2:0 output random */
			val = tg->payload_mode;
			writel_relaxed(val, csid->base +
				       CAMSS_CSID_TG_DT_n_CGG_2(0));
		} else {
			struct csid_phy_config *phy = &csid->phy;

			val = phy->lane_cnt - 1;
			val |= phy->lane_assign << 4;

			writel_relaxed(val,
				       csid->base + CAMSS_CSID_CORE_CTRL_0);

			val = phy->csiphy_id << 17;
			val |= 0x9;

			writel_relaxed(val,
				       csid->base + CAMSS_CSID_CORE_CTRL_1);
		}

		/* Config LUT */

		dt_shift = (cid % 4) * 8;
		df = csid_get_decode_format(csid->fmt[MSM_CSID_PAD_SINK].code);

		val = readl_relaxed(csid->base + CAMSS_CSID_CID_LUT_VC_n(vc));
		val &= ~(0xff << dt_shift);
		val |= dt << dt_shift;
		writel_relaxed(val, csid->base + CAMSS_CSID_CID_LUT_VC_n(vc));

		val = (df << 4) | 0x3;
		writel_relaxed(val, csid->base + CAMSS_CSID_CID_n_CFG(cid));

		if (tg->enabled) {
			val = 0x00a06437;
			writel_relaxed(val, csid->base + CAMSS_CSID_TG_CTRL);
		}
	} else {
		if (tg->enabled) {
			u32 val = 0x00a06436;
			writel_relaxed(val, csid->base + CAMSS_CSID_TG_CTRL);
		}
	}

	return 0;
}

/*
 * __csid_get_format - Get pointer to format structure
 * @csid: CSID device
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad from which format is requested
 * @which: TRY or ACTIVE format
 *
 * Return pointer to TRY or ACTIVE format structure
 */
static struct v4l2_mbus_framefmt *
__csid_get_format(struct csid_device *csid,
		  struct v4l2_subdev_pad_config *cfg,
		  unsigned int pad,
		  enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(&csid->subdev, cfg, pad);

	return &csid->fmt[pad];
}

/*
 * csid_try_format - Handle try format by pad subdev method
 * @csid: CSID device
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad on which format is requested
 * @fmt: pointer to v4l2 format structure
 * @which: wanted subdev format
 */
static void csid_try_format(struct csid_device *csid,
			    struct v4l2_subdev_pad_config *cfg,
			    unsigned int pad,
			    struct v4l2_mbus_framefmt *fmt,
			    enum v4l2_subdev_format_whence which)
{
	unsigned int i;

	switch (pad) {
	case MSM_CSID_PAD_SINK:
		/* Set format on sink pad */

		for (i = 0; i < ARRAY_SIZE(csid_input_fmts); i++)
			if (fmt->code == csid_input_fmts[i].code)
				break;

		/* If not found, use UYVY as default */
		if (i >= ARRAY_SIZE(csid_input_fmts))
			fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;

		fmt->width = clamp_t(u32, fmt->width, 1, 8191);
		fmt->height = clamp_t(u32, fmt->height, 1, 8191);

		if (fmt->field == V4L2_FIELD_ANY)
			fmt->field = V4L2_FIELD_NONE;

		break;

	case MSM_CSID_PAD_SRC:
		if (csid->testgen_mode->cur.val == 0) {
			/* Test generator is disabled, keep pad formats */
			/* in sync - set and return a format same as sink pad */
			struct v4l2_mbus_framefmt format;

			format = *__csid_get_format(csid, cfg,
						    MSM_CSID_PAD_SINK, which);
			format.code = csid_get_uncompressed(format.code);
			*fmt = format;
		} else {
			/* Test generator is enabled, set format on source pad */
			/* to allow test generator usage */

			for (i = 0; i < ARRAY_SIZE(csid_input_fmts); i++)
				if (fmt->code == csid_input_fmts[i].uncompressed)
					break;

			/* If not found, use UYVY as default */
			if (i >= ARRAY_SIZE(csid_input_fmts))
				fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;

			fmt->width = clamp_t(u32, fmt->width, 1, 8191);
			fmt->height = clamp_t(u32, fmt->height, 1, 8191);

			fmt->field = V4L2_FIELD_NONE;
		}
		break;
	}

	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

/*
 * csid_enum_mbus_code - Handle pixel format enumeration
 * @sd: CSID V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @code: pointer to v4l2_subdev_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int csid_enum_mbus_code(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_mbus_code_enum *code)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	if (code->pad == MSM_CSID_PAD_SINK) {
		if (code->index >= ARRAY_SIZE(csid_input_fmts))
			return -EINVAL;

		code->code = csid_input_fmts[code->index].code;
	} else {
		if (csid->testgen_mode->cur.val == 0) {
			if (code->index > 0)
				return -EINVAL;

			format = __csid_get_format(csid, cfg, MSM_CSID_PAD_SINK,
						   code->which);

			code->code = csid_get_uncompressed(format->code);
		} else {
			if (code->index >= ARRAY_SIZE(csid_input_fmts))
				return -EINVAL;

			code->code = csid_input_fmts[code->index].uncompressed;
		}
	}

	return 0;
}

/*
 * csid_enum_frame_size - Handle frame size enumeration
 * @sd: CSID V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fse: pointer to v4l2_subdev_frame_size_enum structure
 * return -EINVAL or zero on success
 */
static int csid_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt format;

	if (fse->index != 0)
		return -EINVAL;

	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	csid_try_format(csid, cfg, fse->pad, &format, fse->which);
	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code)
		return -EINVAL;

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	csid_try_format(csid, cfg, fse->pad, &format, fse->which);
	fse->max_width = format.width;
	fse->max_height = format.height;

	return 0;
}

/*
 * csid_get_format - Handle get format by pads subdev method
 * @sd: CSID V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int csid_get_format(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __csid_get_format(csid, cfg, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

/*
 * csid_set_format - Handle set format by pads subdev method
 * @sd: CSID V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int csid_set_format(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __csid_get_format(csid, cfg, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	csid_try_format(csid, cfg, fmt->pad, &fmt->format, fmt->which);
	*format = fmt->format;

	/* Propagate the format from sink to source */
	if (fmt->pad == MSM_CSID_PAD_SINK) {
		format = __csid_get_format(csid, cfg, MSM_CSID_PAD_SRC,
					   fmt->which);

		*format = fmt->format;
		csid_try_format(csid, cfg, MSM_CSID_PAD_SRC, format,
				fmt->which);
	}

	return 0;
}

/*
 * csid_init_formats - Initialize formats on all pads
 * @sd: CSID V4L2 subdevice
 *
 * Initialize all pad formats with default values.
 */
static int csid_init_formats(struct v4l2_subdev *sd)
{
	struct v4l2_subdev_format format;

	memset(&format, 0, sizeof(format));
	format.pad = MSM_CSID_PAD_SINK;
	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format.format.code = MEDIA_BUS_FMT_UYVY8_2X8;
	format.format.width = 1920;
	format.format.height = 1080;
	csid_set_format(sd, NULL, &format);

	return 0;
}

static const char * const csid_test_pattern_menu[] = {
	"Disabled",
	"Incrementing",
	"Alternating 55/AA",
	"All Zeros",
	"All Ones",
	"Random Data",
};

static int csid_set_test_pattern(struct csid_device *csid, s32 value)
{
	struct csid_testgen_config *tg = &csid->testgen;

	/* If CSID is linked to CSIPHY, do not allow to enable test generator */
	if (value && media_entity_remote_pad(&csid->pads[MSM_CSID_PAD_SINK]))
		return -EBUSY;

	tg->enabled = !!value;

	switch (value) {
	case 1:
		tg->payload_mode = CSID_PAYLOAD_MODE_INCREMENTING;
		break;
	case 2:
		tg->payload_mode = CSID_PAYLOAD_MODE_ALTERNATING_55_AA;
		break;
	case 3:
		tg->payload_mode = CSID_PAYLOAD_MODE_ALL_ZEROES;
		break;
	case 4:
		tg->payload_mode = CSID_PAYLOAD_MODE_ALL_ONES;
		break;
	case 5:
		tg->payload_mode = CSID_PAYLOAD_MODE_RANDOM;
		break;
	}

	return 0;
}

static int csid_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct csid_device *csid = container_of(ctrl->handler,
						struct csid_device, ctrls);
	int ret = -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		ret = csid_set_test_pattern(csid, ctrl->val);
		break;
	}

	return ret;
}

static struct v4l2_ctrl_ops csid_ctrl_ops = {
	.s_ctrl = csid_s_ctrl,
};

/*
 * msm_csid_subdev_init - Initialize CSID device structure and resources
 * @csid: CSID device
 * @camss: Camera sub-system structure
 * @res: CSID module resources table
 * @id: CSID module id
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_csid_subdev_init(struct csid_device *csid,
			 struct resources *res, u8 id)
{
	struct device *dev = to_device_index(csid, id);
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct resource *r;
	int i;
	int ret;

	csid->id = id;

	/* Memory */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[0]);
	csid->base = devm_ioremap_resource(dev, r);
	if (IS_ERR(csid->base)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(csid->base);
	}

	/* Interrupt */

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ, res->interrupt[0]);
	csid->irq = r->start;
	if (IS_ERR_VALUE(csid->irq))
		return csid->irq;

	ret = devm_request_irq(dev, csid->irq, csid_isr,
		IRQF_TRIGGER_RISING, dev_name(dev), csid);
	if (ret < 0) {
		dev_err(dev, "request_irq failed\n");
		return ret;
	}

	disable_irq(csid->irq);

	/* Clocks */

	csid->nclocks = 0;
	while (res->clock[csid->nclocks])
		csid->nclocks++;

	csid->clock = devm_kzalloc(dev, csid->nclocks * sizeof(*csid->clock),
				    GFP_KERNEL);
	if (!csid->clock) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	csid->clock_rate = devm_kzalloc(dev, csid->nclocks *
					sizeof(*csid->clock_rate), GFP_KERNEL);
	if (!csid->clock_rate) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < csid->nclocks; i++) {
		csid->clock[i] = devm_clk_get(dev, res->clock[i]);
		if (IS_ERR(csid->clock[i]))
			return PTR_ERR(csid->clock[i]);
		csid->clock_rate[i] = res->clock_rate[i];
	}

	/* Regulator */

	csid->vdda = devm_regulator_get(dev, res->regulator[0]);
	if (IS_ERR(csid->vdda)) {
		dev_err(dev, "could not get regulator\n");
		return PTR_ERR(csid->vdda);
	}

	init_completion(&csid->reset_complete);

	return 0;
}

void msm_csid_get_csid_id(struct media_entity *entity, u8 *id)
{
	struct v4l2_subdev *sd;
	struct csid_device *csid;

	sd = container_of(entity, struct v4l2_subdev, entity);
	csid = v4l2_get_subdevdata(sd);

	*id = csid->id;
}

/*
 * csid_get_lane_assign - Calculate CSI2 lane assign configuration parameter
 * @lane_cfg - CSI2 lane configuration
 *
 * Return lane assign
 */
static u32 csid_get_lane_assign(struct csiphy_lanes_cfg *lane_cfg)
{
	u32 lane_assign = 0;
	int i;

	for (i = 0; i < lane_cfg->num_data; i++)
		lane_assign |= lane_cfg->data[i].pos << (i * 4);

	return lane_assign;
}

/*
 * csid_link_setup - Setup CSID connections
 * @entity: Pointer to media entity structure
 * @local: Pointer to local pad
 * @remote: Pointer to remote pad
 * @flags: Link flags
 *
 * Rreturn 0 on success
 */
static int csid_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	if ((local->flags & MEDIA_PAD_FL_SINK) &&
	    (flags & MEDIA_LNK_FL_ENABLED)) {
		struct v4l2_subdev *sd;
		struct csid_device *csid;
		struct csiphy_device *csiphy;
		struct csiphy_lanes_cfg *lane_cfg;
		struct v4l2_subdev_format format;

		sd = container_of(entity, struct v4l2_subdev, entity);
		csid = v4l2_get_subdevdata(sd);

		/* If test generator is enabled
		 * do not allow a link from CSIPHY to CSID */
		if (csid->testgen_mode->cur.val != 0)
			return -EBUSY;

		sd = container_of(remote->entity, struct v4l2_subdev, entity);
		csiphy = v4l2_get_subdevdata(sd);

		/* If a sensor is not linked to CSIPHY
		 * do no allow a link from CSIPHY to CSID */
		if (!csiphy->cfg.csi2)
			return -EPERM;

		csid->phy.csiphy_id = csiphy->id;

		lane_cfg = &csiphy->cfg.csi2->lane_cfg;
		csid->phy.lane_cnt = lane_cfg->num_data;
		csid->phy.lane_assign = csid_get_lane_assign(lane_cfg);

		// Reset format on source pad to sink pad format
		memset(&format, 0, sizeof(format));
		format.pad = MSM_CSID_PAD_SRC;
		format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		csid_set_format(&csid->subdev, NULL, &format);
	}

	return 0;
}

static const struct v4l2_subdev_core_ops csid_core_ops = {
	.s_power = csid_set_power,
};

static const struct v4l2_subdev_video_ops csid_video_ops = {
	.s_stream = csid_set_stream,
};

static const struct v4l2_subdev_pad_ops csid_pad_ops = {
	.enum_mbus_code = csid_enum_mbus_code,
	.enum_frame_size = csid_enum_frame_size,
	.get_fmt = csid_get_format,
	.set_fmt = csid_set_format,
};

static const struct v4l2_subdev_ops csid_v4l2_ops = {
	.core = &csid_core_ops,
	.video = &csid_video_ops,
	.pad = &csid_pad_ops,
};

static const struct v4l2_subdev_internal_ops csid_v4l2_internal_ops;

static const struct media_entity_operations csid_media_ops = {
	.link_setup = csid_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

/*
 * msm_csid_register_entities - Register subdev node for CSID module
 * @csid: CSID device
 * @v4l2_dev: V4L2 device
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_csid_register_entities(struct csid_device *csid,
			       struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd = &csid->subdev;
	struct media_pad *pads = csid->pads;
	int ret;

	v4l2_subdev_init(sd, &csid_v4l2_ops);
	sd->internal_ops = &csid_v4l2_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, ARRAY_SIZE(sd->name), "%s%d",
		 MSM_CSID_NAME, csid->id);
	v4l2_set_subdevdata(sd, csid);

	v4l2_ctrl_handler_init(&csid->ctrls, 1);
	csid->testgen_mode = v4l2_ctrl_new_std_menu_items(&csid->ctrls,
				&csid_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(csid_test_pattern_menu) - 1, 0, 0,
				csid_test_pattern_menu);

	if (csid->ctrls.error) {
		dev_err(to_device_index(csid, csid->id), "failed to init ctrl: %d\n",
			csid->ctrls.error);
		ret = csid->ctrls.error;
		goto free_ctrl;
	}

	csid->subdev.ctrl_handler = &csid->ctrls;

	csid_init_formats(sd);

	pads[MSM_CSID_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[MSM_CSID_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

	sd->entity.ops = &csid_media_ops;
	ret = media_entity_init(&sd->entity, MSM_CSID_PADS_NUM, pads, 0);
	if (ret < 0) {
		dev_err(to_device_index(csid, csid->id), "failed to init media entity");
		goto free_ctrl;
	}

	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0) {
		dev_err(to_device_index(csid, csid->id), "failed to register subdev");
		goto media_cleanup;
	}

	return 0;

media_cleanup:
	media_entity_cleanup(&sd->entity);
free_ctrl:
	v4l2_ctrl_handler_free(&csid->ctrls);

	return ret;
}

/*
 * msm_csid_unregister_entities - Unregister CSID module subdev node
 * @csid: CSID device
 */
void msm_csid_unregister_entities(struct csid_device *csid)
{
	v4l2_device_unregister_subdev(&csid->subdev);
	v4l2_ctrl_handler_free(&csid->ctrls);
}
