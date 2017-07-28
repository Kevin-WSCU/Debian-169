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

#define MT9V024_CHIP_ID		    0x00
#define	MT9V024_CHIP_ID_WORD 	0x1324

#define TOSHIBA_BRG_ID          0x0000
#define TOSHIBA_BRG_ID_WORD     0x4401



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

struct tc358746_reg_struct {
	u8	size;
	u16	addr;
	u32	val;
};
static inline struct mt9v024 *to_mt9v024(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9v024, sd);
}
/*Some controllers(MIPI receiver detect the LPxx transaction),so stay in standby after the initial setting*/

struct tc358746_reg_struct reg_parallel_in_mipi_out[] = {
   {2, 0x0004, 0x0004},
   {2, 0x0002, 0x0001},
   {2, 0x0002, 0x0000},
   {2, 0x0016, 0x504D},
   {2, 0x0018, 0x0213},
   {2, 0x0006, 0x0064},
   {2, 0x0008, 0x0010},//RAW10
   {2, 0x0022, 0x0320},//0x0320
   {4, 0x0140, 0x00000000},
   {4, 0x0144, 0x00000000},
   {4, 0x0148, 0x00000000},
   {4, 0x014C, 0x00000001},
   {4, 0x0150, 0x00000001},
   
   {4, 0x0210, 0x00001200},
   {4, 0x0214, 0x00000001},
   {4, 0x0218, 0x00000701},
   {4, 0x021C, 0x00000000},
   {4, 0x0220, 0x00000002},
   {4, 0x0224, 0x00004988},
   {4, 0x0228, 0x00000005},
   {4, 0x022C, 0x00000000},
   {4, 0x0234, 0x00000007},
   {4, 0x0238, 0x00000001},
   {4, 0x0204, 0x00000001},//TX PPI starts
   {4, 0x0518, 0x00000001},
   {4, 0x0500, 0xA30080A3},
   {2, 0x0004, 0x0045},
};

struct tc358746_reg_struct reg_parallel_in_mipi_out_752[] = {
   {2, 0x0004, 0x0004},
   {2, 0x0002, 0x0001},
   {2, 0x0002, 0x0000},
   {2, 0x0016, 0x504D},
   {2, 0x0018, 0x0213},
   {2, 0x0006, 0x0078},
   {2, 0x0008, 0x0010},//RAW10
   {2, 0x0022, 0x03ac},//0x0320
   {4, 0x0140, 0x00000000},
   {4, 0x0144, 0x00000000},
   {4, 0x0148, 0x00000000},
   {4, 0x014C, 0x00000001},
   {4, 0x0150, 0x00000001},
   
   {4, 0x0210, 0x00001200},
   {4, 0x0214, 0x00000001},
   {4, 0x0218, 0x00000701},
   {4, 0x021C, 0x00000000},
   {4, 0x0220, 0x00000002},
   {4, 0x0224, 0x00004988},
   {4, 0x0228, 0x00000005},
   {4, 0x022C, 0x00000000},
   {4, 0x0234, 0x00000007},
   {4, 0x0238, 0x00000001},
   {4, 0x0204, 0x00000001},//TX PPI starts
   {4, 0x0518, 0x00000001},
   {4, 0x0500, 0xA30080A3},
   {2, 0x0004, 0x0045},
};

const u16 mt9v024_vga[]={

//  0x01, 0x0001,  // COL_WINDOW_START_CONTEXTA_REG
  0x01, 0x0038,  // COL_WINDOW_START_CONTEXTA_REG
  0x02, 0x0004,  // ROW_WINDOW_START_CONTEXTA_REG
  0x03, 0x01E0,  // ROW_WINDOW_SIZE_CONTEXTA_REG
//  0x04, 0x02F0,  // COL_WINDOW_SIZE_CONTEXTA_REG
  0x04, 0x0280,  // COL_WINDOW_SIZE_CONTEXTA_REG
  0x05, 0x005E,  // HORZ_BLANK_CONTEXTA_REG
  0x06, 0x0039,//0x0041,  // VERT_BLANK_CONTEXTA_REG//tweak to 59.995fps
  0x07, 0x0188,  // CONTROL_MODE_REG
  0x08, 0x0190,  // COARSE_SHUTTER_WIDTH_1_CONTEXTA
  0x09, 0x01BD,  // COARSE_SHUTTER_WIDTH_2_CONTEXTA
  0x0A, 0x0164,  // SHUTTER_WIDTH_CONTROL_CONTEXTA
  0x0B, 0x01C2,  // COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTA
  0x0C, 0x0000,  // RESET_REG
  0x0D, 0x0300,  // READ_MODE_REG
  0x0E, 0x0000,  // READ_MODE2_REG
  0x0F, 0x0100,  // PIXEL_OPERATION_MODE
  0x10, 0x0040,  // RESERVED_CORE_10
  0x11, 0x8042,  // RESERVED_CORE_11
  0x12, 0x0022,  // RESERVED_CORE_12
  0x13, 0x2D2E,  // RESERVED_CORE_13
  0x14, 0x0E02,  // RESERVED_CORE_14
  0x15, 0x0E32,  // RESERVED_CORE_15
  0x16, 0x2802,  // RESERVED_CORE_16
  0x17, 0x3E38,  // RESERVED_CORE_17
  0x18, 0x3E38,  // RESERVED_CORE_18
  0x19, 0x2802,  // RESERVED_CORE_19
  0x1A, 0x0428,  // RESERVED_CORE_1A
  0x1B, 0x0000,  // LED_OUT_CONTROL
  0x1C, 0x0302,  // DATA_COMPRESSION
  0x1D, 0x0040,  // RESERVED_CORE_1D
  0x1E, 0x0000,  // RESERVED_CORE_1E
  0x1F, 0x0000,  // RESERVED_CORE_1F
  0x20, 0x03C7,  // RESERVED_CORE_20
  0x21, 0x0020,  // RESERVED_CORE_21
  0x22, 0x0020,  // RESERVED_CORE_22
  0x23, 0x0010,  // RESERVED_CORE_23
  0x24, 0x001B,  // RESERVED_CORE_24
  0x25, 0x001A,  // RESERVED_CORE_25
  0x26, 0x0004,  // RESERVED_CORE_26
  0x27, 0x000C,  // RESERVED_CORE_27
  0x28, 0x0010,  // RESERVED_CORE_28
  0x29, 0x0010,  // RESERVED_CORE_29
  0x2A, 0x0020,  // RESERVED_CORE_2A
  0x2B, 0x0003,  // RESERVED_CORE_2B
  0x2C, 0x0004,  // VREF_ADC_CONTROL
  0x2D, 0x0004,  // RESERVED_CORE_2D
  0x2E, 0x0007,  // RESERVED_CORE_2E
  0x2F, 0x0003,  // RESERVED_CORE_2F
  0x30, 0x0003,  // RESERVED_CORE_30
  0x31, 0x001F,  // V1_CONTROL_CONTEXTA
  0x32, 0x001A,  // V2_CONTROL_CONTEXTA
  0x33, 0x0012,  // V3_CONTROL_CONTEXTA
  0x34, 0x0003,  // V4_CONTROL_CONTEXTA
  0x35, 0x0020,  // GLOBAL_GAIN_CONTEXTA_REG
  0x36, 0x0010,  // GLOBAL_GAIN_CONTEXTB_REG
  0x37, 0x0000,  // RESERVED_CORE_37
  0x38, 0x0000,  // RESERVED_CORE_38
  0x39, 0x0025,  // V1_CONTROL_CONTEXTB
  0x3A, 0x0020,  // V2_CONTROL_CONTEXTB
  0x3B, 0x0003,  // V3_CONTROL_CONTEXTB
  0x3C, 0x0003,  // V4_CONTROL_CONTEXTB
  0x46, 0x231D,  // DARK_AVG_THRESHOLDS
  0x47, 0x0080,  // CALIB_CONTROL_REG
  0x4C, 0x0002,  // STEP_SIZE_AVG_MODE
  0x70, 0x0000,  // ROW_NOISE_CONTROL
  0x71, 0x002A,  // NOISE_CONSTANT
  0x72, 0x0000,  // PIXCLK_CONTROL
  0x7F, 0x0000,  // TEST_DATA
  0x80, 0x04F4,  // TILE_X0_Y0
  0x81, 0x04F4,  // TILE_X1_Y0
  0x82, 0x04F4,  // TILE_X2_Y0
  0x83, 0x04F4,  // TILE_X3_Y0
  0x84, 0x04F4,  // TILE_X4_Y0
  0x85, 0x04F4,  // TILE_X0_Y1
  0x86, 0x04F4,  // TILE_X1_Y1
  0x87, 0x04F4,  // TILE_X2_Y1
  0x88, 0x04F4,  // TILE_X3_Y1
  0x89, 0x04F4,  // TILE_X4_Y1
  0x8A, 0x04F4,  // TILE_X0_Y2
  0x8B, 0x04F4,  // TILE_X1_Y2
  0x8C, 0x04F4,  // TILE_X2_Y2
  0x8D, 0x04F4,  // TILE_X3_Y2
  0x8E, 0x04F4,  // TILE_X4_Y2
  0x8F, 0x04F4,  // TILE_X0_Y3
  0x90, 0x04F4,  // TILE_X1_Y3
  0x91, 0x04F4,  // TILE_X2_Y3
  0x92, 0x04F4,  // TILE_X3_Y3
  0x93, 0x04F4,  // TILE_X4_Y3
  0x94, 0x04F4,  // TILE_X0_Y4
  0x95, 0x04F4,  // TILE_X1_Y4
  0x96, 0x04F4,  // TILE_X2_Y4
  0x97, 0x04F4,  // TILE_X3_Y4
  0x98, 0x04F4,  // TILE_X4_Y4
  0x99, 0x0000,  // X0_SLASH5
  0x9A, 0x0096,  // X1_SLASH5
  0x9B, 0x012C,  // X2_SLASH5
  0x9C, 0x01C2,  // X3_SLASH5
  0x9D, 0x0258,  // X4_SLASH5
  0x9E, 0x02F0,  // X5_SLASH5
  0x9F, 0x0000,  // Y0_SLASH5
  0xA0, 0x0060,  // Y1_SLASH5
  0xA1, 0x00C0,  // Y2_SLASH5
  0xA2, 0x0120,  // Y3_SLASH5
  0xA3, 0x0180,  // Y4_SLASH5
  0xA4, 0x01E0,  // Y5_SLASH5
  0xA5, 0x003A,  // DESIRED_BIN
  0xA6, 0x0002,  // EXP_SKIP_FRM_H
  0xA8, 0x0000,  // EXP_LPF
  0xA9, 0x0002,  // GAIN_SKIP_FRM
  0xAA, 0x0002,  // GAIN_LPF_H
  0xAB, 0x0040,  // MAX_GAIN
  0xAC, 0x0001,  // MIN_COARSE_EXPOSURE
  0xAD, 0x01E0,  // MAX_COARSE_EXPOSURE
  0xAE, 0x0014,  // BIN_DIFF_THRESHOLD
  0xAF, 0x0000,  // AUTO_BLOCK_CONTROL,disable AE and AG(both context A and context B)
  0xB0, 0xABE0,  // PIXEL_COUNT
  0xB1, 0x0002,  // LVDS_MASTER_CONTROL
  0xB2, 0x0010,  // LVDS_SHFT_CLK_CONTROL
  0xB3, 0x0010,  // LVDS_DATA_CONTROL
  0xB4, 0x0000,  // LVDS_DATA_STREAM_LATENCY
  0xB5, 0x0000,  // LVDS_INTERNAL_SYNC
  0xB6, 0x0000,  // LVDS_USE_10BIT_PIXELS
  0xB7, 0x0000,  // STEREO_ERROR_CONTROL
  0xBF, 0x0016,  // INTERLACE_FIELD_VBLANK
  0xC0, 0x000A,  // IMAGE_CAPTURE_NUM
  0xC2, 0x18D0,  // ANALOG_CONTROLS
  0xC3, 0x007F,  // RESERVED_CORE_C3
  0xC4, 0x007F,  // RESERVED_CORE_C4
  0xC5, 0x007F,  // RESERVED_CORE_C5
  0xC6, 0x0000,  // NTSC_FV_CONTROL
  0xC7, 0x4416,  // NTSC_HBLANK
  0xC8, 0x4421,  // NTSC_VBLANK
  0xC9, 0x0002,  // COL_WINDOW_START_CONTEXTB_REG
  0xCA, 0x0004,  // ROW_WINDOW_START_CONTEXTB_REG
  0xCB, 0x01E0,  // ROW_WINDOW_SIZE_CONTEXTB_REG
  0xCC, 0x02EE,  // COL_WINDOW_SIZE_CONTEXTB_REG
  0xCD, 0x0100,  // HORZ_BLANK_CONTEXTB_REG
  0xCE, 0x0100,  // VERT_BLANK_CONTEXTB_REG
  0xCF, 0x0190,  // COARSE_SHUTTER_WIDTH_1_CONTEXTB
  0xD0, 0x01BD,  // COARSE_SHUTTER_WIDTH_2_CONTEXTB
  0xD1, 0x0064,  // SHUTTER_WIDTH_CONTROL_CONTEXTB
  0xD2, 0x01C2,  // COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTB
  0xD3, 0x0000,  // FINE_SHUTTER_WIDTH_1_CONTEXTA
  0xD4, 0x0000,  // FINE_SHUTTER_WIDTH_2_CONTEXTA
  0xD5, 0x0000,  // FINE_SHUTTER_WIDTH_TOTAL_CONTEXTA
  0xD6, 0x0000,  // FINE_SHUTTER_WIDTH_1_CONTEXTB
  0xD7, 0x0000,  // FINE_SHUTTER_WIDTH_2_CONTEXTB
  0xD8, 0x0000,  // FINE_SHUTTER_WIDTH_TOTAL_CONTEXTB
  0xD9, 0x0000,  // MONITOR_MODE_CONTROL
  0xC2, 0x18D0,  // ANALOG_CONTROLS
  0x07, 0x0388    // CONTROL_MODE_REG

};

const u16 mt9v024_full[]=
{
		0x01, 0x0001, 	// COL_WINDOW_START_CONTEXTA_REG
//		0x01, 0x0038, 	// COL_WINDOW_START_CONTEXTA_REG
		0x02, 0x0004, 	// ROW_WINDOW_START_CONTEXTA_REG
		0x03, 0x01E0, 	// ROW_WINDOW_SIZE_CONTEXTA_REG
		0x04, 0x02F0, 	// COL_WINDOW_SIZE_CONTEXTA_REG
//		0x04, 0x0280, 	// COL_WINDOW_SIZE_CONTEXTA_REG
		0x05, 0x005D, 	// HORZ_BLANK_CONTEXTA_REG
		0x06, 0x0034,//0x0041, 	// VERT_BLANK_CONTEXTA_REG//tweak to 59.995fps
		0x07, 0x0188, 	// CONTROL_MODE_REG
		0x08, 0x0190, 	// COARSE_SHUTTER_WIDTH_1_CONTEXTA
		0x09, 0x01BD, 	// COARSE_SHUTTER_WIDTH_2_CONTEXTA
		0x0A, 0x0164, 	// SHUTTER_WIDTH_CONTROL_CONTEXTA
		0x0B, 0x0020, 	// COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTA
		0x0C, 0x0000, 	// RESET_REG
		0x0D, 0x0300, 	// READ_MODE_REG
		0x0E, 0x0000, 	// READ_MODE2_REG
		0x0F, 0x0100, 	// PIXEL_OPERATION_MODE
		0x10, 0x0040, 	// RESERVED_CORE_10
		0x11, 0x8042, 	// RESERVED_CORE_11
		0x12, 0x0022, 	// RESERVED_CORE_12
		0x13, 0x2D2E, 	// RESERVED_CORE_13
		0x14, 0x0E02, 	// RESERVED_CORE_14
		0x15, 0x0E32, 	// RESERVED_CORE_15
		0x16, 0x2802, 	// RESERVED_CORE_16
		0x17, 0x3E38, 	// RESERVED_CORE_17
		0x18, 0x3E38, 	// RESERVED_CORE_18
		0x19, 0x2802, 	// RESERVED_CORE_19
		0x1A, 0x0428, 	// RESERVED_CORE_1A
		0x1B, 0x0000, 	// LED_OUT_CONTROL
		0x1C, 0x0302, 	// DATA_COMPRESSION
		0x1D, 0x0040, 	// RESERVED_CORE_1D
		0x1E, 0x0000, 	// RESERVED_CORE_1E
		0x1F, 0x0000, 	// RESERVED_CORE_1F
		0x20, 0x03C7, 	// RESERVED_CORE_20
		0x21, 0x0020, 	// RESERVED_CORE_21
		0x22, 0x0020, 	// RESERVED_CORE_22
		0x23, 0x0010, 	// RESERVED_CORE_23
		0x24, 0x001B, 	// RESERVED_CORE_24
		0x25, 0x001A, 	// RESERVED_CORE_25
		0x26, 0x0004, 	// RESERVED_CORE_26
		0x27, 0x000C, 	// RESERVED_CORE_27
		0x28, 0x0010, 	// RESERVED_CORE_28
		0x29, 0x0010, 	// RESERVED_CORE_29
		0x2A, 0x0020, 	// RESERVED_CORE_2A
		0x2B, 0x0003, 	// RESERVED_CORE_2B
		0x2C, 0x0004, 	// VREF_ADC_CONTROL
		0x2D, 0x0004, 	// RESERVED_CORE_2D
		0x2E, 0x0007, 	// RESERVED_CORE_2E
		0x2F, 0x0003, 	// RESERVED_CORE_2F
		0x30, 0x0003, 	// RESERVED_CORE_30
		0x31, 0x001F, 	// V1_CONTROL_CONTEXTA
		0x32, 0x001A, 	// V2_CONTROL_CONTEXTA
		0x33, 0x0012, 	// V3_CONTROL_CONTEXTA
		0x34, 0x0003, 	// V4_CONTROL_CONTEXTA
		0x35, 0x0020, 	// GLOBAL_GAIN_CONTEXTA_REG
		0x36, 0x0010, 	// GLOBAL_GAIN_CONTEXTB_REG
		0x37, 0x0000, 	// RESERVED_CORE_37
		0x38, 0x0000, 	// RESERVED_CORE_38
		0x39, 0x0025, 	// V1_CONTROL_CONTEXTB
		0x3A, 0x0020, 	// V2_CONTROL_CONTEXTB
		0x3B, 0x0003, 	// V3_CONTROL_CONTEXTB
		0x3C, 0x0003, 	// V4_CONTROL_CONTEXTB
		0x46, 0x231D, 	// DARK_AVG_THRESHOLDS
		0x47, 0x0080, 	// CALIB_CONTROL_REG
		0x4C, 0x0002, 	// STEP_SIZE_AVG_MODE
		0x70, 0x0000, 	// ROW_NOISE_CONTROL
		0x71, 0x002A, 	// NOISE_CONSTANT
		0x72, 0x0000, 	// PIXCLK_CONTROL
		0x7F, 0x0000, 	// TEST_DATA
		0x80, 0x04F4, 	// TILE_X0_Y0
		0x81, 0x04F4, 	// TILE_X1_Y0
		0x82, 0x04F4, 	// TILE_X2_Y0
		0x83, 0x04F4, 	// TILE_X3_Y0
		0x84, 0x04F4, 	// TILE_X4_Y0
		0x85, 0x04F4, 	// TILE_X0_Y1
		0x86, 0x04F4, 	// TILE_X1_Y1
		0x87, 0x04F4, 	// TILE_X2_Y1
		0x88, 0x04F4, 	// TILE_X3_Y1
		0x89, 0x04F4, 	// TILE_X4_Y1
		0x8A, 0x04F4, 	// TILE_X0_Y2
		0x8B, 0x04F4, 	// TILE_X1_Y2
		0x8C, 0x04F4, 	// TILE_X2_Y2
		0x8D, 0x04F4, 	// TILE_X3_Y2
		0x8E, 0x04F4, 	// TILE_X4_Y2
		0x8F, 0x04F4, 	// TILE_X0_Y3
		0x90, 0x04F4, 	// TILE_X1_Y3
		0x91, 0x04F4, 	// TILE_X2_Y3
		0x92, 0x04F4, 	// TILE_X3_Y3
		0x93, 0x04F4, 	// TILE_X4_Y3
		0x94, 0x04F4, 	// TILE_X0_Y4
		0x95, 0x04F4, 	// TILE_X1_Y4
		0x96, 0x04F4, 	// TILE_X2_Y4
		0x97, 0x04F4, 	// TILE_X3_Y4
		0x98, 0x04F4, 	// TILE_X4_Y4
		0x99, 0x0000, 	// X0_SLASH5
		0x9A, 0x0096, 	// X1_SLASH5
		0x9B, 0x012C, 	// X2_SLASH5
		0x9C, 0x01C2, 	// X3_SLASH5
		0x9D, 0x0258, 	// X4_SLASH5
		0x9E, 0x02F0, 	// X5_SLASH5
		0x9F, 0x0000, 	// Y0_SLASH5
		0xA0, 0x0060, 	// Y1_SLASH5
		0xA1, 0x00C0, 	// Y2_SLASH5
		0xA2, 0x0120, 	// Y3_SLASH5
		0xA3, 0x0180, 	// Y4_SLASH5
		0xA4, 0x01E0, 	// Y5_SLASH5
		0xA5, 0x003A, 	// DESIRED_BIN
		0xA6, 0x0002, 	// EXP_SKIP_FRM_H
		0xA8, 0x0000, 	// EXP_LPF
		0xA9, 0x0002, 	// GAIN_SKIP_FRM
		0xAA, 0x0002, 	// GAIN_LPF_H
		0xAB, 0x0040, 	// MAX_GAIN
		0xAC, 0x0001, 	// MIN_COARSE_EXPOSURE
		0xAD, 0x01E0, 	// MAX_COARSE_EXPOSURE
		0xAE, 0x0014, 	// BIN_DIFF_THRESHOLD
		0xAF, 0x0000, 	// AUTO_BLOCK_CONTROL,disable AE and AG(both context A and context B)
		0xB0, 0xABE0, 	// PIXEL_COUNT
		0xB1, 0x0002, 	// LVDS_MASTER_CONTROL
		0xB2, 0x0010, 	// LVDS_SHFT_CLK_CONTROL
		0xB3, 0x0010, 	// LVDS_DATA_CONTROL
		0xB4, 0x0000, 	// LVDS_DATA_STREAM_LATENCY
		0xB5, 0x0000, 	// LVDS_INTERNAL_SYNC
		0xB6, 0x0000, 	// LVDS_USE_10BIT_PIXELS
		0xB7, 0x0000, 	// STEREO_ERROR_CONTROL
		0xBF, 0x0016, 	// INTERLACE_FIELD_VBLANK
		0xC0, 0x000A, 	// IMAGE_CAPTURE_NUM
		0xC2, 0x18D0, 	// ANALOG_CONTROLS
		0xC3, 0x007F, 	// RESERVED_CORE_C3
		0xC4, 0x007F, 	// RESERVED_CORE_C4
		0xC5, 0x007F, 	// RESERVED_CORE_C5
		0xC6, 0x0000, 	// NTSC_FV_CONTROL
		0xC7, 0x4416, 	// NTSC_HBLANK
		0xC8, 0x4421, 	// NTSC_VBLANK
		0xC9, 0x0002, 	// COL_WINDOW_START_CONTEXTB_REG
		0xCA, 0x0004, 	// ROW_WINDOW_START_CONTEXTB_REG
		0xCB, 0x01E0, 	// ROW_WINDOW_SIZE_CONTEXTB_REG
		0xCC, 0x02EE, 	// COL_WINDOW_SIZE_CONTEXTB_REG
		0xCD, 0x0100, 	// HORZ_BLANK_CONTEXTB_REG
		0xCE, 0x0100, 	// VERT_BLANK_CONTEXTB_REG
		0xCF, 0x0190, 	// COARSE_SHUTTER_WIDTH_1_CONTEXTB
		0xD0, 0x01BD, 	// COARSE_SHUTTER_WIDTH_2_CONTEXTB
		0xD1, 0x0064, 	// SHUTTER_WIDTH_CONTROL_CONTEXTB
		0xD2, 0x01C2, 	// COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTB
		0xD3, 0x0000, 	// FINE_SHUTTER_WIDTH_1_CONTEXTA
		0xD4, 0x0000, 	// FINE_SHUTTER_WIDTH_2_CONTEXTA
		0xD5, 0x0000, 	// FINE_SHUTTER_WIDTH_TOTAL_CONTEXTA
		0xD6, 0x0000, 	// FINE_SHUTTER_WIDTH_1_CONTEXTB
		0xD7, 0x0000, 	// FINE_SHUTTER_WIDTH_2_CONTEXTB
		0xD8, 0x0000, 	// FINE_SHUTTER_WIDTH_TOTAL_CONTEXTB
		0xD9, 0x0000, 	// MONITOR_MODE_CONTROL
		0xC2, 0x18D0, 	// ANALOG_CONTROLS
		0x07, 0x0388, 	// CONTROL_MODE_REG
 
};





static struct mt9v024_mode_info mt9v024_mode_info_data[MT9V024_MODE_MAX + 1] = {
	{
		.mode = MT9V024_MODE_VGA,
		.width = 640,
		.height = 480,
		.data = mt9v024_vga,
		.data_size = ARRAY_SIZE(mt9v024_vga)
	},

};

static u16 i2c_rd16(struct i2c_client *client, u16 reg)
{
	int err;
	u8 value[2];
	u8 buf[2] = { reg >> 8, reg & 0xff };
	
	client->addr = 0x0E;
	
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 2,
			.buf = value,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		printk("%s: reading register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);
	}

	//if (debug < 3)
	//	return;


	//printk("I2C read from address 0x%04X = 0x%02x%02x\n", reg, value[0], value[1]);

	return ((value[0]<<8)|(value[1]));

}

static u32 i2c_rd32(struct i2c_client *client, u16 reg)
{

	client->addr = 0x0E;
	
	int err;
	u8 value[4];
	u8 buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 4,
			.buf = value,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		printk("%s: reading register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);
	}

	//if (debug < 3)
	//	return;

	//printk("I2C read from address 0x%04X = 0x%02x%02x%02x%02x\n", reg, value[2], value[3], value[0], value[1]);

	return ((value[0]<<8)|(value[1])|(value[2]<<24)|(value[3]<<16));

}

static void i2c_wr16(struct i2c_client *client, u16 reg, u16 value)
{
	int err, i;
	struct i2c_msg msg;
	u8 data[4];
	
	client->addr = 0x0E;
	
	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 4;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;
	data[2] = value >> 8;
	data[3] = (value & 0xff);

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		printk("%s: writing register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);
		return;
	}

	//if (debug < 3)
	//	return;

	//printk("I2C write 0x%04X = 0x%02X%02X\n", reg, data[2], data[3]);
}

static void i2c_wr32(struct i2c_client *client, u16 reg, u32 value)
{
	int err, i;
	struct i2c_msg msg;
	u8 data[4];

    client->addr = 0x0E;
	
	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 6;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;
	data[2] = value >> 8;
	data[3] = (value & 0xff);
	data[4] = value >> 24;
	data[5] = value >> 16;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		printk("%s: writing register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);
		return;
	}

	//if (debug < 3)
	//	return;

	//printk("I2C write 0x%04X = 0x%02X%02X%02X%02X\n", reg, data[4], data[5], data[2], data[3]);
}

static int tc358746_reg_init(struct i2c_client *client)
{
	int size, i;
	
	size = sizeof(reg_parallel_in_mipi_out_752) / sizeof(struct tc358746_reg_struct);
	for(i = 0; i < size; i++) {
		if(reg_parallel_in_mipi_out_752[i].size == 2)
			i2c_wr16(client, reg_parallel_in_mipi_out_752[i].addr, reg_parallel_in_mipi_out_752[i].val);
		else
			i2c_wr32(client, reg_parallel_in_mipi_out_752[i].addr, reg_parallel_in_mipi_out_752[i].val);
	}


	return 0;
}
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


/*
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
*/

static int mt9v024_write_reg(struct mt9v024 *mt9v024, u8 reg, u16 val)
{
	int ret;

	u16 i2c_addr = mt9v024->i2c_client->addr = 0x90;

	ret = msm_cci_ctrl8_write16(i2c_addr, reg, &val, 2);
	if (ret < 0)
		dev_err(mt9v024->dev,
			"%s: write reg error %d on addr 0x%x: reg=0x%x, val=0x%x\n",
			__func__, ret, i2c_addr, reg, val);

	return ret;
}

static int mt9v024_read_reg(struct mt9v024 *mt9v024, u8 reg, u16 *val)
{
	u8 rev_buf[2];
	int ret;
	u16 i2c_addr = mt9v024->i2c_client->addr = 0x90;

	ret = msm_cci_ctrl8_read16(i2c_addr, reg, rev_buf, 2);
	if (ret < 0) {
		dev_err(mt9v024->dev,
			"%s: read reg error %d on addr 0x%x: reg=0x%x\n",
			__func__, ret, i2c_addr, reg);
		return ret;
	}

	*val = (rev_buf[0]<<8)|rev_buf[1];

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
	u16 chip_id,bridge_id;
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

	ret = mt9v024_read_reg(mt9v024, TOSHIBA_BRG_ID, &bridge_id);
	if (ret < 0 || chip_id != TOSHIBA_BRG_ID_WORD) {
		dev_err(dev, "could not read ID high,%x\n",bridge_id);
		ret = -ENODEV;
		goto power_down;
	}
	dev_info(dev, "Toshiba bridge detected at address 0x%x,ID:0x%x\n", client->addr,bridge_id);
	
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
