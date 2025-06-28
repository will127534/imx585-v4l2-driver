// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony imx585 cameras.
 *
 * Based on Sony imx477 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 * Modified by Will WHANG
 * Modified by sohonomura2020 in Soho Enterprise Ltd.
 */
#include <linux/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

// Support for rpi kernel pre git commit 314a685
#ifndef MEDIA_BUS_FMT_SENSOR_DATA
#define MEDIA_BUS_FMT_SENSOR_DATA       0x7002
#endif

#define V4L2_CID_IMX585_HDR_DATASEL_TH   (V4L2_CID_USER_ASPEED_BASE + 0)
#define V4L2_CID_IMX585_HDR_DATASEL_BK   (V4L2_CID_USER_ASPEED_BASE + 1)
#define V4L2_CID_IMX585_HDR_GRAD_TH      (V4L2_CID_USER_ASPEED_BASE + 2)
#define V4L2_CID_IMX585_HDR_GRAD_COMP_L  (V4L2_CID_USER_ASPEED_BASE + 3)
#define V4L2_CID_IMX585_HDR_GRAD_COMP_H  (V4L2_CID_USER_ASPEED_BASE + 4)
#define V4L2_CID_IMX585_HDR_GAIN         (V4L2_CID_USER_ASPEED_BASE + 5)
#define V4L2_CID_IMX585_HGC_GAIN         (V4L2_CID_USER_ASPEED_BASE + 6)

/* Standby or streaming mode */
#define IMX585_REG_MODE_SELECT          0x3000
#define IMX585_MODE_STANDBY             0x01
#define IMX585_MODE_STREAMING           0x00
#define IMX585_STREAM_DELAY_US          25000
#define IMX585_STREAM_DELAY_RANGE_US    1000

/* Leader mode and XVS/XHS direction */
#define IMX585_REG_XMSTA     0x3002
#define IMX585_REG_XXS_DRV   0x30A6
#define IMX585_REG_EXTMODE   0x30CE
#define IMX585_REG_XXS_OUTSEL 0x30A4

/*XVS pulse length, 2^n H with n=0~3*/
#define IMX585_REG_XVSLNG    0x30CC
/*XHS pulse length, 16*(2^n) Clock with n=0~3*/
#define IMX585_REG_XHSLNG    0x30CD


/* Clk selection */
#define IMX585_INCK_SEL                 0x3014

/* Link Speed */
#define IMX585_DATARATE_SEL             0x3015

/* BIN mode */
/* 2x2 Bin mode selection, 0x01 => Mono, 0x00 => Color */
#define IMX585_BIN_MODE                 0x3019

/* Lane Count */
#define IMX585_LANEMODE                 0x3040

/* VMAX internal VBLANK*/
#define IMX585_REG_VMAX                 0x3028
#define IMX585_VMAX_MAX                 0xfffff
#define IMX585_VMAX_DEFAULT             2250

/* HMAX internal HBLANK*/
#define IMX585_REG_HMAX                 0x302C
#define IMX585_HMAX_MAX                 0xffff

/* SHR internal */
#define IMX585_REG_SHR                  0x3050
#define IMX585_SHR_MIN                  8
#define IMX585_SHR_MIN_CLEARHDR         10
#define IMX585_SHR_MAX                  0xfffff

/* Exposure control */
#define IMX585_EXPOSURE_MIN             2
#define IMX585_EXPOSURE_STEP            1
#define IMX585_EXPOSURE_DEFAULT         1000
#define IMX585_EXPOSURE_MAX             49865

/* HDR threshold */
#define IMX585_REG_EXP_TH_H             0x36D0
#define IMX585_REG_EXP_TH_L             0x36D4
#define IMX585_REG_EXP_BK               0x36E2

/* Gradation compression control */
#define IMX595_REG_CCMP_EN              0x36EF
#define IMX585_REG_CCMP1_EXP            0x36E8
#define IMX585_REG_CCMP2_EXP            0x36E4
#define IMX585_REG_ACMP1_EXP            0x36EE
#define IMX585_REG_ACMP2_EXP            0x36EC

/* HDR Gain Adder */
#define IMX585_REG_EXP_GAIN             0x3081

/* Black level control */
#define IMX585_REG_BLKLEVEL             0x30DC
#define IMX585_BLKLEVEL_DEFAULT         50

/* Digital Clamp */
#define IMX585_REG_DIGITAL_CLAMP        0x3458

/* Analog gain control */
#define IMX678_REG_ANALOG_GAIN          0x3070 
#define IMX585_REG_ANALOG_GAIN          0x306C
#define IMX585_REG_FDG_SEL0             0x3030
#define IMX585_ANA_GAIN_MIN             0
#define IMX585_ANA_GAIN_MIN_HGC         34
#define IMX585_ANA_GAIN_MAX_HDR         80
#define IMX585_ANA_GAIN_MAX_NORMAL      240
#define IMX585_ANA_GAIN_STEP            1
#define IMX585_ANA_GAIN_DEFAULT         0


/* Flip */
#define IMX585_FLIP_WINMODEH            0x3020
#define IMX585_FLIP_WINMODEV            0x3021

/* Embedded metadata stream structure */
#define IMX585_EMBEDDED_LINE_WIDTH      16384
#define IMX585_NUM_EMBEDDED_LINES       1

#define IMX585_PIXEL_RATE               74250000

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

/* imx585 native and active pixel array size. */
#define IMX585_NATIVE_WIDTH         3856U
#define IMX585_NATIVE_HEIGHT        2180U
#define IMX585_PIXEL_ARRAY_LEFT     8U
#define IMX585_PIXEL_ARRAY_TOP      8U
#define IMX585_PIXEL_ARRAY_WIDTH    3840U
#define IMX585_PIXEL_ARRAY_HEIGHT   2160U

/* Link frequency setup */
enum {
	IMX585_LINK_FREQ_297MHZ, // 594Mbps/lane
	IMX585_LINK_FREQ_360MHZ, // 720Mbps/lane
	IMX585_LINK_FREQ_445MHZ, // 891Mbps/lane
	IMX585_LINK_FREQ_594MHZ, // 1188Mbps/lane
	IMX585_LINK_FREQ_720MHZ, // 1440Mbps/lane
	IMX585_LINK_FREQ_891MHZ, // 1782Mbps/lane
	IMX585_LINK_FREQ_1039MHZ, // 2079Mbps/lane
	IMX585_LINK_FREQ_1188MHZ, // 2376Mbps/lane
};

static const s64 link_freqs_reg_value[] = {
	[IMX585_LINK_FREQ_297MHZ]  = 0x07,
	[IMX585_LINK_FREQ_360MHZ]  = 0x06,
	[IMX585_LINK_FREQ_445MHZ]  = 0x05,
	[IMX585_LINK_FREQ_594MHZ]  = 0x04,
	[IMX585_LINK_FREQ_720MHZ]  = 0x03,
	[IMX585_LINK_FREQ_891MHZ]  = 0x02,
	[IMX585_LINK_FREQ_1039MHZ] = 0x01,
	[IMX585_LINK_FREQ_1188MHZ] = 0x00,
};

static const s64 link_freqs[] = {
	[IMX585_LINK_FREQ_297MHZ]  = 297000000,
	[IMX585_LINK_FREQ_360MHZ]  = 360000000,
	[IMX585_LINK_FREQ_445MHZ]  = 445500000,
	[IMX585_LINK_FREQ_594MHZ]  = 594000000,
	[IMX585_LINK_FREQ_720MHZ]  = 720000000,
	[IMX585_LINK_FREQ_891MHZ]  = 891000000,
	[IMX585_LINK_FREQ_1039MHZ] = 1039500000,
	[IMX585_LINK_FREQ_1188MHZ] = 1188000000,
};

//min HMAX for 4-lane 4K full res mode, x2 for 2-lane, /2 for FHD
static const s64 HMAX_table_4lane_4K[] = {
	[IMX585_LINK_FREQ_297MHZ] = 1584,
	[IMX585_LINK_FREQ_360MHZ] = 1320,
	[IMX585_LINK_FREQ_445MHZ] = 1100,
	[IMX585_LINK_FREQ_594MHZ] =  792,
	[IMX585_LINK_FREQ_720MHZ] =  660,
	[IMX585_LINK_FREQ_891MHZ] =  550,
	[IMX585_LINK_FREQ_1039MHZ] = 440,
	[IMX585_LINK_FREQ_1188MHZ] = 396,
};

struct imx585_inck_cfg {
		u32 xclk_hz;   /* platform clock rate             */
		u8  inck_sel;  /* value for reg 0x3014            */
};

static const struct imx585_inck_cfg imx585_inck_table[] = {
		{ 74250000, 0x00 },
		{ 37125000, 0x01 },
		{ 72000000, 0x02 },
		{ 27000000, 0x03 },
		{ 24000000, 0x04 },
};

static const char * const hdr_gain_adder_menu[] = {
	"+0dB",
	"+6dB",
	"+12dB",
	"+18dB",
	"+24dB",
	"+29.1dB",
};

/*Honestly I don't know why there are two 50% 50% blend
 * but it is in the datasheet
 */
static const char * const hdr_data_blender_menu[] = {
	"HG 1/2, LG 1/2",
	"HG 3/4, LG 1/4",
	"HG 1/2, LG 1/2",
	"HG 7/8, LG 1/8",
	"HG 15/16, LG 1/16",
	"2nd HG 1/2, LG 1/2",  
	"HG 1/16, LG 15/16",
	"HG 1/8, LG 7/8",
	"HG 1/4, LG 3/4",
};

static const char * const grad_compression_slope_menu[] = {
	"1/1",
	"1/2",
	"1/4",
	"1/8",
	"1/16",
	"1/32",
	"1/64",
	"1/128",
	"1/256",
	"1/512",
	"1/1024",
	"1/2048",
};

static const char * const sync_mode_menu[] = {
	"Internal Sync Leader Mode",
	"External Sync Leader Mode",
	"Follower Mode",
};

struct imx585_reg {
	u16 address;
	u8 val;
};

struct IMX585_reg_list {
	unsigned int num_of_regs;
	const struct imx585_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx585_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* mode HMAX Scaling */
	u8   hmax_div;      /* 1 = full for 4K, 2 = half for FHD */

	/* minimum H-timing */
	u64 min_HMAX;

	/* minimum V-timing */
	u64 min_VMAX;

	/* default H-timing */
	u64 default_HMAX;

	/* default V-timing */
	u64 default_VMAX;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct IMX585_reg_list reg_list;
};

/* IMX585 Register List */
/* Common Modes */
static struct imx585_reg imx585_common_regs[] = {
	{0x3002, 0x01},
	{0x3069, 0x00},
	{0x3074, 0x64},
	{0x30D5, 0x04}, // DIG_CLP_VSTART
	{0x3030, 0x00},// FDG_SEL0 LCG, HCG:0x01
	{0x30A6, 0x00},// XVS_DRV [1:0] Hi-Z
	{0x3081, 0x00},// EXP_GAIN, Reset to 0
	{0x3460, 0x21},// -
	{0x3478, 0xA1},// -
	{0x347C, 0x01},// -
	{0x3480, 0x01},// -
	{0x3A4E, 0x14},// -
	{0x3A52, 0x14},// -
	{0x3A56, 0x00},// -
	{0x3A5A, 0x00},// -
	{0x3A5E, 0x00},// -
	{0x3A62, 0x00},// -
	{0x3A6A, 0x20},// -
	{0x3A6C, 0x42},// -
	{0x3A6E, 0xA0},// -
	{0x3B2C, 0x0C},// -
	{0x3B30, 0x1C},// -
	{0x3B34, 0x0C},// -
	{0x3B38, 0x1C},// -
	{0x3BA0, 0x0C},// -
	{0x3BA4, 0x1C},// -
	{0x3BA8, 0x0C},// -
	{0x3BAC, 0x1C},// -
	{0x3D3C, 0x11},// -
	{0x3D46, 0x0B},// -
	{0x3DE0, 0x3F},// -
	{0x3DE1, 0x08},// -
	{0x3E14, 0x87},// -
	{0x3E16, 0x91},// -
	{0x3E18, 0x91},// -
	{0x3E1A, 0x87},// -
	{0x3E1C, 0x78},// -
	{0x3E1E, 0x50},// -
	{0x3E20, 0x50},// -
	{0x3E22, 0x50},// -
	{0x3E24, 0x87},// -
	{0x3E26, 0x91},// -
	{0x3E28, 0x91},// -
	{0x3E2A, 0x87},// -
	{0x3E2C, 0x78},// -
	{0x3E2E, 0x50},// -
	{0x3E30, 0x50},// -
	{0x3E32, 0x50},// -
	{0x3E34, 0x87},// -
	{0x3E36, 0x91},// -
	{0x3E38, 0x91},// -
	{0x3E3A, 0x87},// -
	{0x3E3C, 0x78},// -
	{0x3E3E, 0x50},// -
	{0x3E40, 0x50},// -
	{0x3E42, 0x50},// -
	{0x4054, 0x64},// -
	{0x4148, 0xFE},// -
	{0x4149, 0x05},// -
	{0x414A, 0xFF},// -
	{0x414B, 0x05},// -
	{0x420A, 0x03},// -
	{0x4231, 0x08},// -
	{0x423D, 0x9C},// -
	{0x4242, 0xB4},// -
	{0x4246, 0xB4},// -
	{0x424E, 0xB4},// -
	{0x425C, 0xB4},// -
	{0x425E, 0xB6},// -
	{0x426C, 0xB4},// -
	{0x426E, 0xB6},// -
	{0x428C, 0xB4},// -
	{0x428E, 0xB6},// -
	{0x4708, 0x00},// -
	{0x4709, 0x00},// -
	{0x470A, 0xFF},// -
	{0x470B, 0x03},// -
	{0x470C, 0x00},// -
	{0x470D, 0x00},// -
	{0x470E, 0xFF},// -
	{0x470F, 0x03},// -
	{0x47EB, 0x1C},// -
	{0x47F0, 0xA6},// -
	{0x47F2, 0xA6},// -
	{0x47F4, 0xA0},// -
	{0x47F6, 0x96},// -
	{0x4808, 0xA6},// -
	{0x480A, 0xA6},// -
	{0x480C, 0xA0},// -
	{0x480E, 0x96},// -
	{0x492C, 0xB2},// -
	{0x4930, 0x03},// -
	{0x4932, 0x03},// -
	{0x4936, 0x5B},// -
	{0x4938, 0x82},// -
	{0x493E, 0x23},// -
	{0x4BA8, 0x1C},// -
	{0x4BA9, 0x03},// -
	{0x4BAC, 0x1C},// -
	{0x4BAD, 0x1C},// -
	{0x4BAE, 0x1C},// -
	{0x4BAF, 0x1C},// -
	{0x4BB0, 0x1C},// -
	{0x4BB1, 0x1C},// -
	{0x4BB2, 0x1C},// -
	{0x4BB3, 0x1C},// -
	{0x4BB4, 0x1C},// -
	{0x4BB8, 0x03},// -
	{0x4BB9, 0x03},// -
	{0x4BBA, 0x03},// -
	{0x4BBB, 0x03},// -
	{0x4BBC, 0x03},// -
	{0x4BBD, 0x03},// -
	{0x4BBE, 0x03},// -
	{0x4BBF, 0x03},// -
	{0x4BC0, 0x03},// -
	{0x4C14, 0x87},// -
	{0x4C16, 0x91},// -
	{0x4C18, 0x91},// -
	{0x4C1A, 0x87},// -
	{0x4C1C, 0x78},// -
	{0x4C1E, 0x50},// -
	{0x4C20, 0x50},// -
	{0x4C22, 0x50},// -
	{0x4C24, 0x87},// -
	{0x4C26, 0x91},// -
	{0x4C28, 0x91},// -
	{0x4C2A, 0x87},// -
	{0x4C2C, 0x78},// -
	{0x4C2E, 0x50},// -
	{0x4C30, 0x50},// -
	{0x4C32, 0x50},// -
	{0x4C34, 0x87},// -
	{0x4C36, 0x91},// -
	{0x4C38, 0x91},// -
	{0x4C3A, 0x87},// -
	{0x4C3C, 0x78},// -
	{0x4C3E, 0x50},// -
	{0x4C40, 0x50},// -
	{0x4C42, 0x50},// -
	{0x4D12, 0x1F},// -
	{0x4D13, 0x1E},// -
	{0x4D26, 0x33},// -
	{0x4E0E, 0x59},// -
	{0x4E14, 0x55},// -
	{0x4E16, 0x59},// -
	{0x4E1E, 0x3B},// -
	{0x4E20, 0x47},// -
	{0x4E22, 0x54},// -
	{0x4E26, 0x81},// -
	{0x4E2C, 0x7D},// -
	{0x4E2E, 0x81},// -
	{0x4E36, 0x63},// -
	{0x4E38, 0x6F},// -
	{0x4E3A, 0x7C},// -
	{0x4F3A, 0x3C},// -
	{0x4F3C, 0x46},// -
	{0x4F3E, 0x59},// -
	{0x4F42, 0x64},// -
	{0x4F44, 0x6E},// -
	{0x4F46, 0x81},// -
	{0x4F4A, 0x82},// -
	{0x4F5A, 0x81},// -
	{0x4F62, 0xAA},// -
	{0x4F72, 0xA9},// -
	{0x4F78, 0x36},// -
	{0x4F7A, 0x41},// -
	{0x4F7C, 0x61},// -
	{0x4F7D, 0x01},// -
	{0x4F7E, 0x7C},// -
	{0x4F7F, 0x01},// -
	{0x4F80, 0x77},// -
	{0x4F82, 0x7B},// -
	{0x4F88, 0x37},// -
	{0x4F8A, 0x40},// -
	{0x4F8C, 0x62},// -
	{0x4F8D, 0x01},// -
	{0x4F8E, 0x76},// -
	{0x4F8F, 0x01},// -
	{0x4F90, 0x5E},// -
	{0x4F91, 0x02},// -
	{0x4F92, 0x69},// -
	{0x4F93, 0x02},// -
	{0x4F94, 0x89},// -
	{0x4F95, 0x02},// -
	{0x4F96, 0xA4},// -
	{0x4F97, 0x02},// -
	{0x4F98, 0x9F},// -
	{0x4F99, 0x02},// -
	{0x4F9A, 0xA3},// -
	{0x4F9B, 0x02},// -
	{0x4FA0, 0x5F},// -
	{0x4FA1, 0x02},// -
	{0x4FA2, 0x68},// -
	{0x4FA3, 0x02},// -
	{0x4FA4, 0x8A},// -
	{0x4FA5, 0x02},// -
	{0x4FA6, 0x9E},// -
	{0x4FA7, 0x02},// -
	{0x519E, 0x79},// -
	{0x51A6, 0xA1},// -
	{0x51F0, 0xAC},// -
	{0x51F2, 0xAA},// -
	{0x51F4, 0xA5},// -
	{0x51F6, 0xA0},// -
	{0x5200, 0x9B},// -
	{0x5202, 0x91},// -
	{0x5204, 0x87},// -
	{0x5206, 0x82},// -
	{0x5208, 0xAC},// -
	{0x520A, 0xAA},// -
	{0x520C, 0xA5},// -
	{0x520E, 0xA0},// -
	{0x5210, 0x9B},// -
	{0x5212, 0x91},// -
	{0x5214, 0x87},// -
	{0x5216, 0x82},// -
	{0x5218, 0xAC},// -
	{0x521A, 0xAA},// -
	{0x521C, 0xA5},// -
	{0x521E, 0xA0},// -
	{0x5220, 0x9B},// -
	{0x5222, 0x91},// -
	{0x5224, 0x87},// -
	{0x5226, 0x82},// -
};

/* Common Registers for ClearHDR. */
static const struct imx585_reg imx585_common_clearHDR_mode[] = {
	{0x301A, 0x10}, // WDMODE: Clear HDR mode
	{0x3024, 0x02}, // COMBI_EN: 0x02 
	{0x3069, 0x02}, // Clear HDR mode
	{0x3074, 0x63}, // Clear HDR mode
	{0x3930, 0xE6}, // DUR[15:8]: Clear HDR mode (12bit)
	{0x3931, 0x00}, // DUR[7:0]: Clear HDR mode (12bit)
	{0x3A4C, 0x61}, // WAIT_ST0[7:0]: Clear HDR mode
	{0x3A4D, 0x02}, // WAIT_ST0[15:8]: Clear HDR mode
	{0x3A50, 0x70}, // WAIT_ST1[7:0]: Clear HDR mode
	{0x3A51, 0x02}, // WAIT_ST1[15:8]: Clear HDR mode
	{0x3E10, 0x17}, // ADTHEN: Clear HDR mode
	{0x493C, 0x41}, // WAIT_10_SHF AD 10-bit 0x0C disable
	{0x4940, 0x41}, // WAIT_12_SHF AD 12-bit 0x41 enable
	{0x3081, 0x02}, // EXP_GAIN: High gain setting +12dB default
};

/* Common Registers for non-ClearHDR. */
static const struct imx585_reg imx585_common_normal_mode[] = {
	{0x301A, 0x00}, // WDMODE: Normal mode
	{0x3024, 0x00}, // COMBI_EN: 0x00
	{0x3069, 0x00}, // Normal mode
	{0x3074, 0x64}, // Normal mode
	{0x3930, 0x0C}, // DUR[15:8]: Normal mode (12bit)
	{0x3931, 0x01}, // DUR[7:0]: Normal mode (12bit)
	{0x3A4C, 0x39}, // WAIT_ST0[7:0]: Normal mode
	{0x3A4D, 0x01}, // WAIT_ST0[15:8]: Normal mode
	{0x3A50, 0x48}, // WAIT_ST1[7:0]: Normal mode
	{0x3A51, 0x01}, // WAIT_ST1[15:8]: Normal mode
	{0x3E10, 0x10}, // ADTHEN: Normal mode
	{0x493C, 0x23}, // WAIT_10_SHF AD 10-bit 0x23 Normal mode 
	{0x4940, 0x23}, // WAIT_12_SHF AD 12-bit 0x23 Normal mode 
};

/* All pixel 4K60. 12-bit */
static const struct imx585_reg mode_4k_regs_12bit[] = {
	{0x301B, 0x00}, // ADDMODE non-binning
	{0x3022, 0x02}, // ADBIT 12-bit
	{0x3023, 0x01}, // MDBIT 12-bit
	{0x30D5, 0x04}, // DIG_CLP_VSTART non-binning
};

/* 2x2 binned 1080p60. 12-bit */
static const struct imx585_reg mode_1080_regs_12bit[] = {
	{0x301B, 0x01}, // ADDMODE binning
	{0x3022, 0x02}, // ADBIT 12-bit
	{0x3023, 0x01}, // MDBIT 12-bit
	{0x30D5, 0x02}, // DIG_CLP_VSTART binning
};
/* IMX585 Register List - END*/


/* IMX678 Register List */
/* Common Modes */
static struct imx585_reg imx678_common_regs[] = {
	{0x301C, 0x00}, // THIN_V_EN
	{0x301E, 0x01}, // VCMODE
	{0x306B, 0x00}, // Sensor_register
	{0x3400, 0x01}, // GAIN_PGC_FIDMD
	{0x3460, 0x22}, // Sensor_register
	{0x355A, 0x64}, // Sensor_register
	{0x3A02, 0x7A}, // Sensor_register
	{0x3A10, 0xEC}, // Sensor_register
	{0x3A12, 0x71}, // Sensor_register
	{0x3A14, 0xDE}, // Sensor_register
	{0x3A20, 0x2B}, // Sensor_register
	{0x3A24, 0x22}, // Sensor_register
	{0x3A25, 0x25}, // Sensor_register
	{0x3A26, 0x2A}, // Sensor_register
	{0x3A27, 0x2C}, // Sensor_register
	{0x3A28, 0x39}, // Sensor_register
	{0x3A29, 0x38}, // Sensor_register
	{0x3A30, 0x04}, // Sensor_register
	{0x3A31, 0x04}, // Sensor_register
	{0x3A32, 0x03}, // Sensor_register
	{0x3A33, 0x03}, // Sensor_register
	{0x3A34, 0x09}, // Sensor_register
	{0x3A35, 0x06}, // Sensor_register
	{0x3A38, 0xCD}, // Sensor_register
	{0x3A3A, 0x4C}, // Sensor_register
	{0x3A3C, 0xB9}, // Sensor_register
	{0x3A3E, 0x30}, // Sensor_register
	{0x3A40, 0x2C}, // Sensor_register
	{0x3A42, 0x39}, // Sensor_register
	{0x3A4E, 0x00}, // Sensor_register
	{0x3A52, 0x00}, // Sensor_register
	{0x3A56, 0x00}, // Sensor_register
	{0x3A5A, 0x00}, // Sensor_register
	{0x3A5E, 0x00}, // Sensor_register
	{0x3A62, 0x00}, // Sensor_register
	{0x3A64, 0x00}, // Sensor_register
	{0x3A6E, 0xA0}, // Sensor_register
	{0x3A70, 0x50}, // Sensor_register
	{0x3A8C, 0x04}, // Sensor_register
	{0x3A8D, 0x03}, // Sensor_register
	{0x3A8E, 0x09}, // Sensor_register
	{0x3A90, 0x38}, // Sensor_register
	{0x3A91, 0x42}, // Sensor_register
	{0x3A92, 0x3C}, // Sensor_register
	{0x3B0E, 0xF3}, // Sensor_register
	{0x3B12, 0xE5}, // Sensor_register
	{0x3B27, 0xC0}, // Sensor_register
	{0x3B2E, 0xEF}, // Sensor_register
	{0x3B30, 0x6A}, // Sensor_register
	{0x3B32, 0xF6}, // Sensor_register
	{0x3B36, 0xE1}, // Sensor_register
	{0x3B3A, 0xE8}, // Sensor_register
	{0x3B5A, 0x17}, // Sensor_register
	{0x3B5E, 0xEF}, // Sensor_register
	{0x3B60, 0x6A}, // Sensor_register
	{0x3B62, 0xF6}, // Sensor_register
	{0x3B66, 0xE1}, // Sensor_register
	{0x3B6A, 0xE8}, // Sensor_register
	{0x3B88, 0xEC}, // Sensor_register
	{0x3B8A, 0xED}, // Sensor_register
	{0x3B94, 0x71}, // Sensor_register
	{0x3B96, 0x72}, // Sensor_register
	{0x3B98, 0xDE}, // Sensor_register
	{0x3B9A, 0xDF}, // Sensor_register
	{0x3C0F, 0x06}, // Sensor_register
	{0x3C10, 0x06}, // Sensor_register
	{0x3C11, 0x06}, // Sensor_register
	{0x3C12, 0x06}, // Sensor_register
	{0x3C13, 0x06}, // Sensor_register
	{0x3C18, 0x20}, // Sensor_register
	{0x3C37, 0x10}, // Sensor_register
	{0x3C3A, 0x7A}, // Sensor_register
	{0x3C40, 0xF4}, // Sensor_register
	{0x3C48, 0xE6}, // Sensor_register
	{0x3C54, 0xCE}, // Sensor_register
	{0x3C56, 0xD0}, // Sensor_register
	{0x3C6C, 0x53}, // Sensor_register
	{0x3C6E, 0x55}, // Sensor_register
	{0x3C70, 0xC0}, // Sensor_register
	{0x3C72, 0xC2}, // Sensor_register
	{0x3C7E, 0xCE}, // Sensor_register
	{0x3C8C, 0xCF}, // Sensor_register
	{0x3C8E, 0xEB}, // Sensor_register
	{0x3C98, 0x54}, // Sensor_register
	{0x3C9A, 0x70}, // Sensor_register
	{0x3C9C, 0xC1}, // Sensor_register
	{0x3C9E, 0xDD}, // Sensor_register
	{0x3CB0, 0x7A}, // Sensor_register
	{0x3CB2, 0xBA}, // Sensor_register
	{0x3CC8, 0xBC}, // Sensor_register
	{0x3CCA, 0x7C}, // Sensor_register
	{0x3CD4, 0xEA}, // Sensor_register
	{0x3CD5, 0x01}, // Sensor_register
	{0x3CD6, 0x4A}, // Sensor_register
	{0x3CD8, 0x00}, // Sensor_register
	{0x3CD9, 0x00}, // Sensor_register
	{0x3CDA, 0xFF}, // Sensor_register
	{0x3CDB, 0x03}, // Sensor_register
	{0x3CDC, 0x00}, // Sensor_register
	{0x3CDD, 0x00}, // Sensor_register
	{0x3CDE, 0xFF}, // Sensor_register
	{0x3CDF, 0x03}, // Sensor_register
	{0x3CE4, 0x4C}, // Sensor_register
	{0x3CE6, 0xEC}, // Sensor_register
	{0x3CE7, 0x01}, // Sensor_register
	{0x3CE8, 0xFF}, // Sensor_register
	{0x3CE9, 0x03}, // Sensor_register
	{0x3CEA, 0x00}, // Sensor_register
	{0x3CEB, 0x00}, // Sensor_register
	{0x3CEC, 0xFF}, // Sensor_register
	{0x3CED, 0x03}, // Sensor_register
	{0x3CEE, 0x00}, // Sensor_register
	{0x3CEF, 0x00}, // Sensor_register
	{0x3CF2, 0xFF}, // Sensor_register
	{0x3CF3, 0x03}, // Sensor_register
	{0x3CF4, 0x00}, // Sensor_register
	{0x3E28, 0x82}, // Sensor_register
	{0x3E2A, 0x80}, // Sensor_register
	{0x3E30, 0x85}, // Sensor_register
	{0x3E32, 0x7D}, // Sensor_register
	{0x3E5C, 0xCE}, // Sensor_register
	{0x3E5E, 0xD3}, // Sensor_register
	{0x3E70, 0x53}, // Sensor_register
	{0x3E72, 0x58}, // Sensor_register
	{0x3E74, 0xC0}, // Sensor_register
	{0x3E76, 0xC5}, // Sensor_register
	{0x3E78, 0xC0}, // Sensor_register
	{0x3E79, 0x01}, // Sensor_register
	{0x3E7A, 0xD4}, // Sensor_register
	{0x3E7B, 0x01}, // Sensor_register
	{0x3EB4, 0x0B}, // Sensor_register
	{0x3EB5, 0x02}, // Sensor_register
	{0x3EB6, 0x4D}, // Sensor_register
	{0x3EB7, 0x42}, // Sensor_register
	{0x3EEC, 0xF3}, // Sensor_register
	{0x3EEE, 0xE7}, // Sensor_register
	{0x3F01, 0x01}, // Sensor_register
	{0x3F24, 0x10}, // Sensor_register
	{0x3F28, 0x2D}, // Sensor_register
	{0x3F2A, 0x2D}, // Sensor_register
	{0x3F2C, 0x2D}, // Sensor_register
	{0x3F2E, 0x2D}, // Sensor_register
	{0x3F30, 0x23}, // Sensor_register
	{0x3F38, 0x2D}, // Sensor_register
	{0x3F3A, 0x2D}, // Sensor_register
	{0x3F3C, 0x2D}, // Sensor_register
	{0x3F3E, 0x28}, // Sensor_register
	{0x3F40, 0x1E}, // Sensor_register
	{0x3F48, 0x2D}, // Sensor_register
	{0x3F4A, 0x2D}, // Sensor_register
	{0x3F4C, 0x00}, // Sensor_register
	{0x4004, 0xE4}, // Sensor_register
	{0x4006, 0xFF}, // Sensor_register
	{0x4018, 0x69}, // Sensor_register
	{0x401A, 0x84}, // Sensor_register
	{0x401C, 0xD6}, // Sensor_register
	{0x401E, 0xF1}, // Sensor_register
	{0x4038, 0xDE}, // Sensor_register
	{0x403A, 0x00}, // Sensor_register
	{0x403B, 0x01}, // Sensor_register
	{0x404C, 0x63}, // Sensor_register
	{0x404E, 0x85}, // Sensor_register
	{0x4050, 0xD0}, // Sensor_register
	{0x4052, 0xF2}, // Sensor_register
	{0x4108, 0xDD}, // Sensor_register
	{0x410A, 0xF7}, // Sensor_register
	{0x411C, 0x62}, // Sensor_register
	{0x411E, 0x7C}, // Sensor_register
	{0x4120, 0xCF}, // Sensor_register
	{0x4122, 0xE9}, // Sensor_register
	{0x4138, 0xE6}, // Sensor_register
	{0x413A, 0xF1}, // Sensor_register
	{0x414C, 0x6B}, // Sensor_register
	{0x414E, 0x76}, // Sensor_register
	{0x4150, 0xD8}, // Sensor_register
	{0x4152, 0xE3}, // Sensor_register
	{0x417E, 0x03}, // Sensor_register
	{0x417F, 0x01}, // Sensor_register
	{0x4186, 0xE0}, // Sensor_register
	{0x4190, 0xF3}, // Sensor_register
	{0x4192, 0xF7}, // Sensor_register
	{0x419C, 0x78}, // Sensor_register
	{0x419E, 0x7C}, // Sensor_register
	{0x41A0, 0xE5}, // Sensor_register
	{0x41A2, 0xE9}, // Sensor_register
	{0x41C8, 0xE2}, // Sensor_register
	{0x41CA, 0xFD}, // Sensor_register
	{0x41DC, 0x67}, // Sensor_register
	{0x41DE, 0x82}, // Sensor_register
	{0x41E0, 0xD4}, // Sensor_register
	{0x41E2, 0xEF}, // Sensor_register
	{0x4200, 0xDE}, // Sensor_register
	{0x4202, 0xDA}, // Sensor_register
	{0x4218, 0x63}, // Sensor_register
	{0x421A, 0x5F}, // Sensor_register
	{0x421C, 0xD0}, // Sensor_register
	{0x421E, 0xCC}, // Sensor_register
	{0x425A, 0x82}, // Sensor_register
	{0x425C, 0xEF}, // Sensor_register
	{0x4348, 0xFE}, // Sensor_register
	{0x4349, 0x06}, // Sensor_register
	{0x4352, 0xCE}, // Sensor_register
	{0x4420, 0x0B}, // Sensor_register
	{0x4421, 0x02}, // Sensor_register
	{0x4422, 0x4D}, // Sensor_register
	{0x4423, 0x0A}, // Sensor_register
	{0x4426, 0xF5}, // Sensor_register
	{0x442A, 0xE7}, // Sensor_register
	{0x4432, 0xF5}, // Sensor_register
	{0x4436, 0xE7}, // Sensor_register
	{0x4466, 0xB4}, // Sensor_register
	{0x446E, 0x32}, // Sensor_register
	{0x449F, 0x1C}, // Sensor_register
	{0x44A4, 0x2C}, // Sensor_register
	{0x44A6, 0x2C}, // Sensor_register
	{0x44A8, 0x2C}, // Sensor_register
	{0x44AA, 0x2C}, // Sensor_register
	{0x44B4, 0x2C}, // Sensor_register
	{0x44B6, 0x2C}, // Sensor_register
	{0x44B8, 0x2C}, // Sensor_register
	{0x44BA, 0x2C}, // Sensor_register
	{0x44C4, 0x2C}, // Sensor_register
	{0x44C6, 0x2C}, // Sensor_register
	{0x44C8, 0x2C}, // Sensor_register
	{0x4506, 0xF3}, // Sensor_register
	{0x450E, 0xE5}, // Sensor_register
	{0x4516, 0xF3}, // Sensor_register
	{0x4522, 0xE5}, // Sensor_register
	{0x4524, 0xF3}, // Sensor_register
	{0x452C, 0xE5}, // Sensor_register
	{0x453C, 0x22}, // Sensor_register
	{0x453D, 0x1B}, // Sensor_register
	{0x453E, 0x1B}, // Sensor_register
	{0x453F, 0x15}, // Sensor_register
	{0x4540, 0x15}, // Sensor_register
	{0x4541, 0x15}, // Sensor_register
	{0x4542, 0x15}, // Sensor_register
	{0x4543, 0x15}, // Sensor_register
	{0x4544, 0x15}, // Sensor_register
	{0x4548, 0x00}, // Sensor_register
	{0x4549, 0x01}, // Sensor_register
	{0x454A, 0x01}, // Sensor_register
	{0x454B, 0x06}, // Sensor_register
	{0x454C, 0x06}, // Sensor_register
	{0x454D, 0x06}, // Sensor_register
	{0x454E, 0x06}, // Sensor_register
	{0x454F, 0x06}, // Sensor_register
	{0x4550, 0x06}, // Sensor_register
	{0x4554, 0x55}, // Sensor_register
	{0x4555, 0x02}, // Sensor_register
	{0x4556, 0x42}, // Sensor_register
	{0x4557, 0x05}, // Sensor_register
	{0x4558, 0xFD}, // Sensor_register
	{0x4559, 0x05}, // Sensor_register
	{0x455A, 0x94}, // Sensor_register
	{0x455B, 0x06}, // Sensor_register
	{0x455D, 0x06}, // Sensor_register
	{0x455E, 0x49}, // Sensor_register
	{0x455F, 0x07}, // Sensor_register
	{0x4560, 0x7F}, // Sensor_register
	{0x4561, 0x07}, // Sensor_register
	{0x4562, 0xA5}, // Sensor_register
	{0x4564, 0x55}, // Sensor_register
	{0x4565, 0x02}, // Sensor_register
	{0x4566, 0x42}, // Sensor_register
	{0x4567, 0x05}, // Sensor_register
	{0x4568, 0xFD}, // Sensor_register
	{0x4569, 0x05}, // Sensor_register
	{0x456A, 0x94}, // Sensor_register
	{0x456B, 0x06}, // Sensor_register
	{0x456D, 0x06}, // Sensor_register
	{0x456E, 0x49}, // Sensor_register
	{0x456F, 0x07}, // Sensor_register
	{0x4572, 0xA5}, // Sensor_register
	{0x460C, 0x7D}, // Sensor_register
	{0x460E, 0xB1}, // Sensor_register
	{0x4614, 0xA8}, // Sensor_register
	{0x4616, 0xB2}, // Sensor_register
	{0x461C, 0x7E}, // Sensor_register
	{0x461E, 0xA7}, // Sensor_register
	{0x4624, 0xA8}, // Sensor_register
	{0x4626, 0xB2}, // Sensor_register
	{0x462C, 0x7E}, // Sensor_register
	{0x462E, 0x8A}, // Sensor_register
	{0x4630, 0x94}, // Sensor_register
	{0x4632, 0xA7}, // Sensor_register
	{0x4634, 0xFB}, // Sensor_register
	{0x4636, 0x2F}, // Sensor_register
	{0x4638, 0x81}, // Sensor_register
	{0x4639, 0x01}, // Sensor_register
	{0x463A, 0xB5}, // Sensor_register
	{0x463B, 0x01}, // Sensor_register
	{0x463C, 0x26}, // Sensor_register
	{0x463E, 0x30}, // Sensor_register
	{0x4640, 0xAC}, // Sensor_register
	{0x4641, 0x01}, // Sensor_register
	{0x4642, 0xB6}, // Sensor_register
	{0x4643, 0x01}, // Sensor_register
	{0x4644, 0xFC}, // Sensor_register
	{0x4646, 0x25}, // Sensor_register
	{0x4648, 0x82}, // Sensor_register
	{0x4649, 0x01}, // Sensor_register
	{0x464A, 0xAB}, // Sensor_register
	{0x464B, 0x01}, // Sensor_register
	{0x464C, 0x26}, // Sensor_register
	{0x464E, 0x30}, // Sensor_register
	{0x4654, 0xFC}, // Sensor_register
	{0x4656, 0x08}, // Sensor_register
	{0x4658, 0x12}, // Sensor_register
	{0x465A, 0x25}, // Sensor_register
	{0x4662, 0xFC}, // Sensor_register
	{0x46A2, 0xFB}, // Sensor_register
	{0x46D6, 0xF3}, // Sensor_register
	{0x46E6, 0x00}, // Sensor_register
	{0x46E8, 0xFF}, // Sensor_register
	{0x46E9, 0x03}, // Sensor_register
	{0x46EC, 0x7A}, // Sensor_register
	{0x46EE, 0xE5}, // Sensor_register
	{0x46F4, 0xEE}, // Sensor_register
	{0x46F6, 0xF2}, // Sensor_register
	{0x470C, 0xFF}, // Sensor_register
	{0x470D, 0x03}, // Sensor_register
	{0x470E, 0x00}, // Sensor_register
	{0x4714, 0xE0}, // Sensor_register
	{0x4716, 0xE4}, // Sensor_register
	{0x471E, 0xED}, // Sensor_register
	{0x472E, 0x00}, // Sensor_register
	{0x4730, 0xFF}, // Sensor_register
	{0x4731, 0x03}, // Sensor_register
	{0x4734, 0x7B}, // Sensor_register
	{0x4736, 0xDF}, // Sensor_register
	{0x4754, 0x7D}, // Sensor_register
	{0x4756, 0x8B}, // Sensor_register
	{0x4758, 0x93}, // Sensor_register
	{0x475A, 0xB1}, // Sensor_register
	{0x475C, 0xFB}, // Sensor_register
	{0x475E, 0x09}, // Sensor_register
	{0x4760, 0x11}, // Sensor_register
	{0x4762, 0x2F}, // Sensor_register
	{0x4766, 0xCC}, // Sensor_register
	{0x4776, 0xCB}, // Sensor_register
	{0x477E, 0x4A}, // Sensor_register
	{0x478E, 0x49}, // Sensor_register
	{0x4794, 0x7C}, // Sensor_register
	{0x4796, 0x8F}, // Sensor_register
	{0x4798, 0xB3}, // Sensor_register
	{0x4799, 0x00}, // Sensor_register
	{0x479A, 0xCC}, // Sensor_register
	{0x479C, 0xC1}, // Sensor_register
	{0x479E, 0xCB}, // Sensor_register
	{0x47A4, 0x7D}, // Sensor_register
	{0x47A6, 0x8E}, // Sensor_register
	{0x47A8, 0xB4}, // Sensor_register
	{0x47A9, 0x00}, // Sensor_register
	{0x47AA, 0xC0}, // Sensor_register
	{0x47AC, 0xFA}, // Sensor_register
	{0x47AE, 0x0D}, // Sensor_register
	{0x47B0, 0x31}, // Sensor_register
	{0x47B1, 0x01}, // Sensor_register
	{0x47B2, 0x4A}, // Sensor_register
	{0x47B3, 0x01}, // Sensor_register
	{0x47B4, 0x3F}, // Sensor_register
	{0x47B6, 0x49}, // Sensor_register
	{0x47BC, 0xFB}, // Sensor_register
	{0x47BE, 0x0C}, // Sensor_register
	{0x47C0, 0x32}, // Sensor_register
	{0x47C1, 0x01}, // Sensor_register
	{0x47C2, 0x3E}, // Sensor_register
	{0x47C3, 0x01}, // Sensor_register
};

/* Common Registers for non-ClearHDR. */
static const struct imx585_reg imx678_common_normal_mode[] = {
	{0x301A, 0x00}, // WDMODE: Normal mode
};
/* IMX678 Register List - END */




/* For Mode List: 
 * Default:
 *   12Bit - FHD, 4K
 * ClearHDR Enabled:
 *   12bit + Gradation compression
 *   16bit - FHD, 4K
 *
 * Gradation compression is available on 12bit 
 * With Default option, only 12bit mode is exposed
 * With ClearHDR enabled via parameters, 
 *   12bit will be with Gradation compression enabled
 *   16bit mode exposed
 */

/* Mode configs */
struct imx585_mode supported_modes[] = {
	{
		/* 1080p90 2x2 binning */
		.width = 1928,
		.height = 1090,
		.hmax_div = 1,
		.min_HMAX = 366,
		.min_VMAX = IMX585_VMAX_DEFAULT,
		.default_HMAX = 366,
		.default_VMAX = IMX585_VMAX_DEFAULT,
		.crop = {
			.left = IMX585_PIXEL_ARRAY_LEFT,
			.top = IMX585_PIXEL_ARRAY_TOP,
			.width = IMX585_PIXEL_ARRAY_WIDTH,
			.height = IMX585_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1080_regs_12bit),
			.regs = mode_1080_regs_12bit,
		},
	},
	{
		/* 4K60 All pixel */
		.width = 3856,
		.height = 2180,
		.min_HMAX = 550,
		.min_VMAX = IMX585_VMAX_DEFAULT,
		.default_HMAX = 550,
		.default_VMAX = IMX585_VMAX_DEFAULT,
		.hmax_div = 1,
		.crop = {
			.left = IMX585_PIXEL_ARRAY_LEFT,
			.top = IMX585_PIXEL_ARRAY_TOP,
			.width = IMX585_PIXEL_ARRAY_WIDTH,
			.height = IMX585_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4k_regs_12bit),
			.regs = mode_4k_regs_12bit,
		},
	},
};


/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */

/* 12bit Only */
static const u32 codes_normal[] = {
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
};

/* 12bit + 16bit Clear HDR */
static const u32 codes_clearhdr[] = {
	/* 16-bit modes. */
	MEDIA_BUS_FMT_SRGGB16_1X16,
	MEDIA_BUS_FMT_SGRBG16_1X16,
	MEDIA_BUS_FMT_SGBRG16_1X16,
	MEDIA_BUS_FMT_SBGGR16_1X16,
	/* 12-bit modes. */
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
};

/* Flip isn’t relevant for mono */
static const u32 mono_codes[] = {
	MEDIA_BUS_FMT_Y16_1X16,   /* 16-bit mono */
	MEDIA_BUS_FMT_Y12_1X12,   /* 12-bit mono */
};

/* regulator supplies */
static const char * const imx585_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (3.3V) supply */
	"VDIG",  /* Digital Core (1.1V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define imx585_NUM_SUPPLIES ARRAY_SIZE(imx585_supply_name)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby)
 */
#define imx585_XCLR_MIN_DELAY_US    500000
#define imx585_XCLR_DELAY_RANGE_US  1000

struct imx585_compatible_data {
	const char *name;
	const bool clearHDR;
	const u16 analog_gain_reg;
	struct IMX585_reg_list common_regs;
	struct IMX585_reg_list clearHDR_common_regs;
	struct IMX585_reg_list normal_common_regs;
};

struct imx585 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;

	/* chosen INCK_SEL register value */
	u8  inck_sel_val;

	/* Link configurations */
	unsigned int lane_count;
	unsigned int link_freq_idx;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[imx585_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *hgc_ctrl;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *blacklevel;

	struct v4l2_ctrl *hdr_mode;
	struct v4l2_ctrl *datasel_th_ctrl;
	struct v4l2_ctrl *datasel_bk_ctrl;
	struct v4l2_ctrl *gdc_th_ctrl;
	struct v4l2_ctrl *gdc_exp_ctrl_l;
	struct v4l2_ctrl *gdc_exp_ctrl_h;
	struct v4l2_ctrl *hdr_gain_ctrl;


	bool          has_ircut;
	struct v4l2_ctrl *ircut_ctrl;
	struct i2c_client  *ircut_client; 

	/* Current mode */
	const struct imx585_mode *mode;

	/* HGC enabled flag*/
	bool hgc;

	/* Mono mode */
	bool mono;
	/* Clear HDR mode */
	bool clear_HDR;
	/* Sync Mode*/
	/* 0 = Internal Sync Leader Mode
	 * 1 = External Sync Leader Mode
	 * 2 = Follower Mode
	 * The datasheet wording is very confusing but basically:
	 * Leader Mode = Sensor using internal clock to drive the sensor
	 * But with external sync mode you can send a XVS input so the sensor
	 * will try to align with it.
	 * For Follower mode it is purely driven by external clock
	 * In this case you need to drive both XVS and XHS
	 */
	u32 sync_mode;

	/* Tracking sensor VMAX/HMAX value */
	u16 HMAX;
	u32 VMAX;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	/* Registers to different compatible sensors */
	const struct imx585_compatible_data *regs;
};


static inline struct imx585 *to_imx585(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx585, sd);
}

static inline void get_mode_table(struct imx585 *imx585, unsigned int code,
				  const struct imx585_mode **mode_list,
				  unsigned int *num_modes)
{
	*mode_list = NULL;
	*num_modes = 0;

	if (imx585->mono){
		/* --- Mono paths --- */
		if (code == MEDIA_BUS_FMT_Y16_1X16 && imx585->clear_HDR) {
			*mode_list = supported_modes;
			*num_modes = ARRAY_SIZE(supported_modes);
		}
		if (code == MEDIA_BUS_FMT_Y12_1X12) {
			*mode_list = supported_modes;
			*num_modes = ARRAY_SIZE(supported_modes);
		}
	}
	else{
		/* --- Color paths --- */
		switch (code) {
			/* 16-bit */
			case MEDIA_BUS_FMT_SRGGB16_1X16:
			case MEDIA_BUS_FMT_SGRBG16_1X16:
			case MEDIA_BUS_FMT_SGBRG16_1X16:
			case MEDIA_BUS_FMT_SBGGR16_1X16:
			case MEDIA_BUS_FMT_SRGGB12_1X12:
			case MEDIA_BUS_FMT_SGRBG12_1X12:
			case MEDIA_BUS_FMT_SGBRG12_1X12:
			case MEDIA_BUS_FMT_SBGGR12_1X12:
				*mode_list = supported_modes;
				*num_modes = ARRAY_SIZE(supported_modes);
				break;
			default:
				*mode_list = NULL;
				*num_modes = 0;
		}

	}
	return;
}
/* ---------------------------------------------------------------------
 * Optional IR-cut helper
 * ------------------------------------------------------------------ */

/* One-byte “command” sent to the IR-cut MCU at imx585->ircut_client   */
static int imx585_ircut_write(struct imx585 *imx585, u8 cmd)
{
	struct i2c_client *client = imx585->ircut_client;
	int ret;

	ret = i2c_smbus_write_byte(client, cmd);
	if (ret < 0)
		dev_err(&client->dev, "IR-cut write failed (%d)\n", ret);

	return ret;
}

static int imx585_ircut_set(struct imx585 *imx585, int on)
{
	return imx585_ircut_write(imx585, on ? 0x01 : 0x00);
}


/* Read registers up to 2 at a time */
static int imx585_read_reg(struct imx585 *imx585, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers 1 byte at a time */
static int imx585_write_reg_1byte(struct imx585 *imx585, u16 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	u8 buf[3];
	int ret;

	put_unaligned_be16(reg, buf);
	buf[2] = val;
	ret = i2c_master_send(client, buf, 3);
	if (ret != 3)
		return ret;

	return 0;
}

/* Write registers 2 byte at a time */
static int imx585_write_reg_2byte(struct imx585 *imx585, u16 reg, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	u8 buf[4];
	int ret;

	put_unaligned_be16(reg, buf);
	buf[2] = val;
	buf[3] = val >> 8;
	ret = i2c_master_send(client, buf, 4);
	if (ret != 4)
		return ret;

	return 0;
}

/* Write registers 3 byte at a time */
static int imx585_write_reg_3byte(struct imx585 *imx585, u16 reg, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	u8 buf[5];

	put_unaligned_be16(reg, buf);
	buf[2]  = val;
	buf[3]  = val >> 8;
	buf[4]  = val >> 16;
	if (i2c_master_send(client, buf, 5) != 5)
		return -EIO;

	return 0;
}

/* Write a list of 1 byte registers */
static int imx585_write_regs(struct imx585 *imx585,
			     const struct IMX585_reg_list *reg_list)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < reg_list->num_of_regs; i++) {
		ret = imx585_write_reg_1byte(imx585, reg_list->regs[i].address, reg_list->regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    reg_list->regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Hold register values until hold is disabled */
static inline void imx585_register_hold(struct imx585 *imx585, bool hold)
{
	imx585_write_reg_1byte(imx585, 0x3001, hold ? 1 : 0);
}

/* Get bayer order based on flip setting. */
static u32 imx585_get_format_code(struct imx585 *imx585, u32 code)
{
	unsigned int i;

	lockdep_assert_held(&imx585->mutex);

	if (imx585->mono) {
		for (i = 0; i < ARRAY_SIZE(mono_codes); i++)
			if (mono_codes[i] == code)
				break;
		return mono_codes[i];
	}
	if(imx585->clear_HDR){
		for (i = 0; i < ARRAY_SIZE(codes_clearhdr); i++)
			if (codes_clearhdr[i] == code)
				break;
		return codes_clearhdr[i];
	}
	else{
		for (i = 0; i < ARRAY_SIZE(codes_normal); i++)
			if (codes_normal[i] == code)
				break;
		return codes_normal[i];
	}
}

static void imx585_set_default_format(struct imx585 *imx585)
{
	/* Set default mode to max resolution */
	imx585->mode = &supported_modes[0];
	if (imx585->mono)
		imx585->fmt_code = MEDIA_BUS_FMT_Y12_1X12;
	else
		imx585->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
}

static int imx585_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx585 *imx585 = to_imx585(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_state_get_format(fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_state_get_format(fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx585->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes[0].width;
	try_fmt_img->height = supported_modes[0].height;
	if (imx585->mono)
		try_fmt_img->code = imx585_get_format_code(imx585, MEDIA_BUS_FMT_Y12_1X12);
	else
		try_fmt_img->code = imx585_get_format_code(imx585, MEDIA_BUS_FMT_SRGGB12_1X12);

	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = IMX585_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = IMX585_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_state_get_crop(fh->state, IMAGE_PAD);
	try_crop->left = IMX585_PIXEL_ARRAY_LEFT;
	try_crop->top = IMX585_PIXEL_ARRAY_TOP;
	try_crop->width = IMX585_PIXEL_ARRAY_WIDTH;
	try_crop->height = IMX585_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx585->mutex);

	return 0;
}

/* For HDR mode, Gain is limited to 0~80 and HGC is disabled
 * For Normal mode, Gain is limited to 0~240
 */
static void imx585_update_gain_limits(struct imx585 *imx585)
{
		bool hcg_on = imx585->hgc;
		bool clear_hdr = imx585->clear_HDR;
        u32 min = hcg_on ? IMX585_ANA_GAIN_MIN_HGC : IMX585_ANA_GAIN_MIN;
        u32 max = clear_hdr ? IMX585_ANA_GAIN_MAX_HDR : IMX585_ANA_GAIN_MAX_NORMAL;
        u32 cur = imx585->gain->val;

        __v4l2_ctrl_modify_range(imx585->gain,
                                 min, max,
                                 IMX585_ANA_GAIN_STEP,
                                 clamp(cur, min, max));

        if (cur < min || cur > max)
                __v4l2_ctrl_s_ctrl(imx585->gain,
                                   clamp(cur, min, max));
}

static void imx585_set_framing_limits(struct imx585 *imx585)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	const struct imx585_mode *mode = imx585->mode;
	u64 default_hblank, max_hblank;
	u64 pixel_rate;

	imx585->VMAX = mode->default_VMAX;
	imx585->HMAX = mode->default_HMAX;

	pixel_rate = (u64)mode->width * IMX585_PIXEL_RATE;
	do_div(pixel_rate, mode->min_HMAX);
	__v4l2_ctrl_modify_range(imx585->pixel_rate, pixel_rate, pixel_rate, 1, pixel_rate);

	//int default_hblank = mode->default_HMAX*IMX585_PIXEL_RATE/72000000-IMX585_NATIVE_WIDTH;
	default_hblank = mode->default_HMAX * pixel_rate;
	do_div(default_hblank, IMX585_PIXEL_RATE);
	default_hblank = default_hblank - mode->width;

	max_hblank = IMX585_HMAX_MAX * pixel_rate;
	do_div(max_hblank, IMX585_PIXEL_RATE);
	max_hblank = max_hblank - mode->width;

	__v4l2_ctrl_modify_range(imx585->hblank, 0, max_hblank, 1, default_hblank);
	__v4l2_ctrl_s_ctrl(imx585->hblank, default_hblank);

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx585->vblank, mode->min_VMAX - mode->height,
				 IMX585_VMAX_MAX - mode->height,
				 1, mode->default_VMAX - mode->height);
	__v4l2_ctrl_s_ctrl(imx585->vblank, mode->default_VMAX - mode->height);

	__v4l2_ctrl_modify_range(imx585->exposure, IMX585_EXPOSURE_MIN,
			 imx585->VMAX - IMX585_SHR_MIN_CLEARHDR, 1,
				IMX585_EXPOSURE_DEFAULT);

	dev_info(&client->dev, "Setting default HBLANK : %llu, VBLANK : %llu PixelRate: %lld\n",
		 default_hblank, mode->default_VMAX - mode->height, pixel_rate);
}

static void imx585_update_hmax(struct imx585 *imx585)
{

	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);

	const u32 base_4lane = HMAX_table_4lane_4K[imx585->link_freq_idx];
	const u32 lane_scale = (imx585->lane_count == 2) ? 2 : 1;
	const u32 factor     = base_4lane * lane_scale;
	const u32 hdr_factor = (imx585->clear_HDR) ? 2 : 1;
	dev_info(&client->dev, "Upadte minimum HMAX\n");
	dev_info(&client->dev, "\tbase_4lane: %d\n", base_4lane);
	dev_info(&client->dev, "\tlane_scale: %d\n", lane_scale);
	dev_info(&client->dev, "\tfactor: %d\n", factor);
	dev_info(&client->dev, "\thdr_factor: %d\n", hdr_factor);

	for (unsigned int i = 0; i < ARRAY_SIZE(supported_modes); ++i) {
		struct imx585_mode *m = &supported_modes[i];
		u32 h = factor / m->hmax_div;        /* one divide per mode */
		u64 v = IMX585_VMAX_DEFAULT * hdr_factor;
		m->min_HMAX     = h;
		m->default_HMAX = h;
		m->min_VMAX     = v;
		m->default_VMAX = v;
		dev_info(&client->dev, "\tv: %lld x h: %d\n", v,h);
	}

}


static int imx585_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx585 *imx585 = container_of(ctrl->handler, struct imx585, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	const struct imx585_mode *mode = imx585->mode;
	const struct imx585_mode *mode_list;
	unsigned int code, num_modes;

	int ret = 0;
	/*
	 * Applying V4L2 control value that
	 * doesn't need to be in streaming mode
	 */
	switch (ctrl->id) {
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		if (imx585->clear_HDR != ctrl->val) {
			imx585->clear_HDR = ctrl->val;
			v4l2_ctrl_activate(imx585->datasel_th_ctrl,  imx585->clear_HDR);
	        v4l2_ctrl_activate(imx585->datasel_bk_ctrl,  imx585->clear_HDR);
	        v4l2_ctrl_activate(imx585->gdc_th_ctrl,      imx585->clear_HDR);
	        v4l2_ctrl_activate(imx585->gdc_exp_ctrl_h,   imx585->clear_HDR);
	        v4l2_ctrl_activate(imx585->gdc_exp_ctrl_l,   imx585->clear_HDR);
	        v4l2_ctrl_activate(imx585->hdr_gain_ctrl,    imx585->clear_HDR);
	        v4l2_ctrl_activate(imx585->hgc_ctrl,        !imx585->clear_HDR);
	        imx585_update_gain_limits(imx585);
			if (imx585->mono)
				code = imx585_get_format_code(imx585, MEDIA_BUS_FMT_Y12_1X12);
			else
				code = imx585_get_format_code(imx585, MEDIA_BUS_FMT_SRGGB12_1X12);
			get_mode_table(imx585, code, &mode_list, &num_modes);
			imx585->mode = v4l2_find_nearest_size(mode_list,
							      num_modes,
							      width, height,
							      imx585->mode->width,
							      imx585->mode->height);
			imx585_update_hmax(imx585); //ClearHDR mode will double the VMAX
			imx585_set_framing_limits(imx585);
		}
		break;
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		{
			u32 shr;

			shr = (imx585->VMAX - ctrl->val)  & ~1u; //Always a multiple of 2
			dev_info(&client->dev, "V4L2_CID_EXPOSURE : %d\n", ctrl->val);
			dev_info(&client->dev, "\tVMAX:%d, HMAX:%d\n", imx585->VMAX, imx585->HMAX);
			dev_info(&client->dev, "\tSHR:%d\n", shr);

			ret = imx585_write_reg_3byte(imx585, IMX585_REG_SHR, shr);
			if (ret)
				dev_err_ratelimited(&client->dev,
						    "Failed to write reg 0x%4.4x. error = %d\n",
						    IMX585_REG_SHR, ret);
		break;
		}
	case V4L2_CID_IMX585_HGC_GAIN:
		{
		if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
        	break;
		imx585->hgc = ctrl->val;
		imx585_update_gain_limits(imx585);

		// Set HGC/LCG channel
		ret = imx585_write_reg_1byte(imx585, IMX585_REG_FDG_SEL0, ctrl->val);
		if (ret)
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    IMX585_REG_FDG_SEL0, ret);
		dev_info(&client->dev, "V4L2_CID_HGC_ENABLE: %d\n", ctrl->val);
		break;
		}
	case V4L2_CID_ANALOGUE_GAIN:
		{
        u32 gain = ctrl->val;

        dev_info(&client->dev, "analogue gain = %u (%s)\n",
                 gain, imx585->hgc ? "HCG" : "LCG");

        ret = imx585_write_reg_2byte(imx585, imx585->regs->analog_gain_reg, gain);
        if (ret)
                dev_err_ratelimited(&client->dev,
                                    "ANALOG_GAIN write failed (%d)\n", ret);
        break;
		}
	case V4L2_CID_VBLANK:
		{
			u32 current_exposure = imx585->exposure->cur.val;
			u32 minSHR = (imx585->clear_HDR) ? IMX585_SHR_MIN_CLEARHDR:IMX585_SHR_MIN;
			/*
			 * The VBLANK control may change the limits of usable exposure, so check
			 * and adjust if necessary.
			 */
			imx585->VMAX = (mode->height + ctrl->val) & ~1u; //Always a multiple of 2

			/* New maximum exposure limits,
			 * modifying the range and make sure we are not exceed the new maximum.
			 */
			current_exposure = clamp_t(u32, current_exposure, IMX585_EXPOSURE_MIN,
						   imx585->VMAX - minSHR);
			__v4l2_ctrl_modify_range(imx585->exposure, IMX585_EXPOSURE_MIN,
						 imx585->VMAX - minSHR, 1,
						 current_exposure);

			dev_info(&client->dev, "V4L2_CID_VBLANK : %d\n", ctrl->val);
			dev_info(&client->dev, "\tVMAX:%d, HMAX:%d\n", imx585->VMAX, imx585->HMAX);
			dev_info(&client->dev, "Update exposure limits: max:%d, min:%d, current:%d\n",
				imx585->VMAX - minSHR,
				IMX585_EXPOSURE_MIN, current_exposure);

			ret = imx585_write_reg_3byte(imx585, IMX585_REG_VMAX, imx585->VMAX);
			if (ret)
				dev_err_ratelimited(&client->dev,
						    "Failed to write reg 0x%4.4x. error = %d\n",
						    IMX585_REG_VMAX, ret);
		break;
		}

	case V4L2_CID_HBLANK:
		{
			u64 pixel_rate;
			u64 hmax;

			pixel_rate = (u64)mode->width * IMX585_PIXEL_RATE;
			do_div(pixel_rate, mode->min_HMAX);
			hmax = (u64)(mode->width + ctrl->val) * IMX585_PIXEL_RATE;
			do_div(hmax, pixel_rate);
			imx585->HMAX = hmax;

			dev_info(&client->dev, "V4L2_CID_HBLANK : %d\n", ctrl->val);
			dev_info(&client->dev, "\tHMAX : %d\n", imx585->HMAX);

			ret = imx585_write_reg_2byte(imx585, IMX585_REG_HMAX, hmax);
			if (ret)
				dev_err_ratelimited(&client->dev,
						    "Failed to write reg 0x%4.4x. error = %d\n",
						    IMX585_REG_HMAX, ret);
		break;
		}
	case V4L2_CID_HFLIP:
		dev_info(&client->dev, "V4L2_CID_HFLIP : %d\n", ctrl->val);
		ret = imx585_write_reg_1byte(imx585, IMX585_FLIP_WINMODEH, ctrl->val);
		if (ret)
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    IMX585_FLIP_WINMODEH, ret);
		break;
	case V4L2_CID_VFLIP:
		dev_info(&client->dev, "V4L2_CID_VFLIP : %d\n", ctrl->val);
		ret = imx585_write_reg_1byte(imx585, IMX585_FLIP_WINMODEV, ctrl->val);
		if (ret)
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    IMX585_FLIP_WINMODEV, ret);
		break;
	case V4L2_CID_BRIGHTNESS:
		dev_info(&client->dev, "V4L2_CID_BRIGHTNESS : %d\n", ctrl->val);
		u16 blacklevel = ctrl->val;
		if(blacklevel > 4095) blacklevel = 4095;
		ret = imx585_write_reg_1byte(imx585, IMX585_REG_BLKLEVEL, blacklevel);
		if (ret)
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    IMX585_REG_BLKLEVEL, ret);
		break;
	case V4L2_CID_BAND_STOP_FILTER:
		if (imx585->has_ircut) {
			dev_info(&client->dev, "V4L2_CID_BAND_STOP_FILTER : %d\n", ctrl->val);
		 	imx585_ircut_set(imx585, ctrl->val);
		 }
		break;
	case V4L2_CID_IMX585_HDR_DATASEL_TH:{
		const u16 *th = (const u16 *)ctrl->p_new.p;
		ret = imx585_write_reg_2byte(imx585, IMX585_REG_EXP_TH_H, th[0]);
		if (ret)
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    IMX585_REG_EXP_TH_H, ret);
		ret = imx585_write_reg_2byte(imx585, IMX585_REG_EXP_TH_L, th[1]);
		if (ret)
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    IMX585_REG_EXP_TH_L, ret);
		dev_info(&client->dev, "V4L2_CID_IMX585_HDR_DATASEL_TH : %d, %d\n",th[0],th[1]);
		break;
		}
	case V4L2_CID_IMX585_HDR_DATASEL_BK:
		ret = imx585_write_reg_1byte(imx585, IMX585_REG_EXP_BK, ctrl->val);
		dev_info(&client->dev, "V4L2_CID_IMX585_HDR_DATASEL_BK : %d\n",ctrl->val);
		if (ret)
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    IMX585_REG_EXP_BK, ret);
		break;
	case V4L2_CID_IMX585_HDR_GRAD_TH:{
		const u32 *thr = (const u32 *)ctrl->p_new.p; 
		ret = imx585_write_reg_3byte(imx585, IMX585_REG_CCMP1_EXP, thr[0]);
		if (ret)
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    IMX585_REG_CCMP1_EXP, ret);
		ret = imx585_write_reg_3byte(imx585, IMX585_REG_CCMP2_EXP, thr[1]);
		if (ret)
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    IMX585_REG_CCMP2_EXP, ret);
		dev_info(&client->dev, "V4L2_CID_IMX585_HDR_GRAD_TH : %d, %d\n",thr[0],thr[1]);
		break;
		}
	case V4L2_CID_IMX585_HDR_GRAD_COMP_L:{
		ret = imx585_write_reg_1byte(imx585, IMX585_REG_ACMP1_EXP, ctrl->val);
		if (ret)
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    IMX585_REG_ACMP1_EXP, ret);
		dev_info(&client->dev, "V4L2_CID_IMX585_HDR_GRAD_COMP_L : %d\n", ctrl->val);
		break;
		}
	case V4L2_CID_IMX585_HDR_GRAD_COMP_H:{
		ret = imx585_write_reg_1byte(imx585, IMX585_REG_ACMP2_EXP, ctrl->val);
		if (ret)
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    IMX585_REG_ACMP2_EXP, ret);
		dev_info(&client->dev, "V4L2_CID_IMX585_HDR_GRAD_COMP_H : %d\n",ctrl->val);
		break;
		}
	case V4L2_CID_IMX585_HDR_GAIN:
		ret = imx585_write_reg_1byte(imx585, IMX585_REG_EXP_GAIN, ctrl->val);
		dev_info(&client->dev, "IMX585_REG_EXP_GAIN : %d\n",ctrl->val);
		if (ret)
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    IMX585_REG_EXP_GAIN, ret);
		break;
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		/* Already handled above. */
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx585_ctrl_ops = {
	.s_ctrl = imx585_set_ctrl,
};

static const u16 hdr_thresh_def[2] = { 512, 1024 };
static const struct v4l2_ctrl_config imx585_cfg_hdr_datasel_th = {
	.ops = &imx585_ctrl_ops,
	.id = V4L2_CID_IMX585_HDR_DATASEL_TH,
	.name = "HDR Data selection Threshold",
	.type = V4L2_CTRL_TYPE_U16,
	.min = 0,
	.max = 0x0FFF,
	.step = 1,
	.def = 0,
	.dims = { 2 },
	.elem_size = sizeof(u16),
};

static const struct v4l2_ctrl_config imx585_cfg_hdr_datasel_bk = {
	.ops = &imx585_ctrl_ops,
	.id = V4L2_CID_IMX585_HDR_DATASEL_BK,
	.name = "HDR Data Blending Mode",
	.type = V4L2_CTRL_TYPE_MENU,
	.max = ARRAY_SIZE(hdr_data_blender_menu) - 1,
	.menu_skip_mask = 0,
	.def = 0,
	.qmenu = hdr_data_blender_menu,
};

static const u32 grad_thresh_def[2] = { 500, 11500 };
static const struct v4l2_ctrl_config imx585_cfg_hdr_grad_th = {
	.ops = &imx585_ctrl_ops,
	.id = V4L2_CID_IMX585_HDR_GRAD_TH,
	.name = "Gradiant Compression Threshold (16bit)",
	.type = V4L2_CTRL_TYPE_U32,
	.min = 0,
	.max = 0x1FFFF,
	.step = 1,
	.def = 0,
	.dims = { 2 },
	.elem_size = sizeof(u32),
};

static const struct v4l2_ctrl_config imx585_cfg_hdr_grad_exp_l = {
	.ops = &imx585_ctrl_ops,
	.id = V4L2_CID_IMX585_HDR_GRAD_COMP_L,
	.name = "Gradiant Compression Ratio Low",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = 0,
	.max = ARRAY_SIZE(grad_compression_slope_menu) - 1,
	.menu_skip_mask = 0,
	.def = 2,
	.qmenu = grad_compression_slope_menu,
};

static const struct v4l2_ctrl_config imx585_cfg_hdr_grad_exp_h = {
	.ops = &imx585_ctrl_ops,
	.id = V4L2_CID_IMX585_HDR_GRAD_COMP_H,
	.name = "Gradiant Compression Ratio High",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = 0,
	.max = ARRAY_SIZE(grad_compression_slope_menu) - 1,
	.menu_skip_mask = 0,
	.def = 6,
	.qmenu = grad_compression_slope_menu,
};

static const struct v4l2_ctrl_config imx585_cfg_hdr_gain = {
	.ops = &imx585_ctrl_ops,
	.id = V4L2_CID_IMX585_HDR_GAIN,
	.name = "HDR Gain Adder (dB)",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = 0,
	.max = ARRAY_SIZE(hdr_gain_adder_menu) - 1,
	.menu_skip_mask = 0,
	.def = 2,
	.qmenu = hdr_gain_adder_menu,
};

static const struct v4l2_ctrl_config imx585_cfg_hgc = {
	.ops = &imx585_ctrl_ops,
	.id = V4L2_CID_IMX585_HGC_GAIN,
	.name = "HGC Enable",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min  = 0,
	.max  = 1,
	.step = 1,
	.def  = 0,
};

static int imx585_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx585 *imx585 = to_imx585(sd);
	unsigned int entries;
	const u32 *tbl;

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (imx585->mono) {
			if (imx585->clear_HDR) {
				if (code->index > 1)
					return -EINVAL;
				code->code = mono_codes[code->index];
			} else { /* HDR off: expose Y12 only */
				if (code->index)
					return -EINVAL;
				code->code = MEDIA_BUS_FMT_Y12_1X12;
			}
			return 0;
		}
		else {
			if (imx585->clear_HDR) {
				tbl     = codes_clearhdr;  /* << 16bit + 12bit */
				entries = ARRAY_SIZE(codes_clearhdr) / 4;
			} else {
				tbl     = codes_normal;    /* << ONLY 12bit */
				entries = ARRAY_SIZE(codes_normal) / 4;
		    }
		}

	    if (code->index >= entries)
		    return -EINVAL;

	    code->code = imx585_get_format_code(imx585, tbl[code->index * 4]);
	    return 0;
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int imx585_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx585 *imx585 = to_imx585(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		const struct imx585_mode *mode_list;
		unsigned int num_modes;

		get_mode_table(imx585, fse->code, &mode_list, &num_modes);

		if (fse->index >= num_modes)
			return -EINVAL;

		if (fse->code != imx585_get_format_code(imx585, fse->code))
			return -EINVAL;

		fse->min_width = mode_list[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = IMX585_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX585_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx585_reset_colorspace(const struct imx585_mode *mode, struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx585_update_image_pad_format(struct imx585 *imx585,
					   const struct imx585_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx585_reset_colorspace(mode, &fmt->format);
}

static void imx585_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = IMX585_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX585_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx585_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx585 *imx585 = to_imx585(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx585->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state, fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				imx585_get_format_code(imx585, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx585_update_image_pad_format(imx585, imx585->mode, fmt);
			fmt->format.code =
				   imx585_get_format_code(imx585, imx585->fmt_code);
		} else {
			imx585_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx585->mutex);
	return 0;
}


static int imx585_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx585_mode *mode;
	struct imx585 *imx585 = to_imx585(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx585->mutex);

	if (fmt->pad == IMAGE_PAD) {
		const struct imx585_mode *mode_list;
		unsigned int num_modes;

		/* Bayer order varies with flips */
		fmt->format.code = imx585_get_format_code(imx585, fmt->format.code);
		get_mode_table(imx585, fmt->format.code, &mode_list, &num_modes);
		mode = v4l2_find_nearest_size(mode_list,
					      num_modes,
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		imx585_update_image_pad_format(imx585, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
			*framefmt = fmt->format;
		} else if (imx585->mode != mode ||
			   imx585->fmt_code != fmt->format.code) {
			imx585->mode = mode;
			imx585->fmt_code = fmt->format.code;
			imx585_set_framing_limits(imx585);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx585_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx585->mutex);

	return 0;
}

static const struct v4l2_rect *
__imx585_get_pad_crop(struct imx585 *imx585,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, IMAGE_PAD);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx585->mode->crop;
	}

	return NULL;
}

/* Start streaming */
static int imx585_start_streaming(struct imx585 *imx585)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	int ret;

	if (!imx585->common_regs_written) {
		ret = imx585_write_regs(imx585, &imx585->regs->common_regs);
		if (ret) {
			dev_err(&client->dev, "%s failed to set common settings\n", __func__);
			return ret;
		}

		imx585_write_reg_1byte(imx585, IMX585_INCK_SEL, imx585->inck_sel_val);
		imx585_write_reg_2byte(imx585, IMX585_REG_BLKLEVEL, IMX585_BLKLEVEL_DEFAULT);
		imx585_write_reg_1byte(imx585, IMX585_DATARATE_SEL,
				       link_freqs_reg_value[imx585->link_freq_idx]);

		if (imx585->lane_count == 2)
			imx585_write_reg_1byte(imx585, IMX585_LANEMODE, 0x01);
		else
			imx585_write_reg_1byte(imx585, IMX585_LANEMODE, 0x03);

		if (imx585->mono)
			imx585_write_reg_1byte(imx585, IMX585_BIN_MODE, 0x01);
		else
			imx585_write_reg_1byte(imx585, IMX585_BIN_MODE, 0x00);

		if(imx585->sync_mode == 1){ //External Sync Leader Mode
			dev_info(&client->dev,"External Sync Leader Mode, enable XVS input\n");
			imx585_write_reg_1byte(imx585, IMX585_REG_EXTMODE, 0x01);
			// Enable XHS output, but XVS is input
			imx585_write_reg_1byte(imx585, IMX585_REG_XXS_DRV, 0x03);
			// Disable XVS OUT
			imx585_write_reg_1byte(imx585, IMX585_REG_XXS_OUTSEL, 0x08);
		} else if(imx585->sync_mode == 0){ //Internal Sync Leader Mode
			dev_info(&client->dev,"Internal Sync Leader Mode, enable output\n");
			imx585_write_reg_1byte(imx585, IMX585_REG_EXTMODE, 0x00);
			// Enable XHS and XVS output
			imx585_write_reg_1byte(imx585, IMX585_REG_XXS_DRV, 0x00);
			imx585_write_reg_1byte(imx585, IMX585_REG_XXS_OUTSEL, 0x0A);
		} else{
			dev_info(&client->dev,"Follower Mode, enable XVS/XHS input\n");
			//For follower mode, switch both of them to input
			imx585_write_reg_1byte(imx585, IMX585_REG_XXS_DRV, 0x0F);
			imx585_write_reg_1byte(imx585, IMX585_REG_XXS_OUTSEL, 0x00);
		}
		imx585->common_regs_written = true;
		dev_info(&client->dev, "common_regs_written\n");
	}

	/* Apply default values of current mode */
	ret = imx585_write_regs(imx585, &imx585->mode->reg_list);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	if(imx585->clear_HDR){
		ret = imx585_write_regs(imx585, &imx585->regs->clearHDR_common_regs);
		if (ret) {
			dev_err(&client->dev, "%s failed to set ClearHDR settings\n", __func__);
			return ret;
		}
		//16bit mode is linear, 12bit mode we need to enable gradation compression 
		switch (imx585->fmt_code) {
			/* 16-bit */
			case MEDIA_BUS_FMT_SRGGB16_1X16:
			case MEDIA_BUS_FMT_SGRBG16_1X16:
			case MEDIA_BUS_FMT_SGBRG16_1X16:
			case MEDIA_BUS_FMT_SBGGR16_1X16:
			case MEDIA_BUS_FMT_Y16_1X16:
				imx585_write_reg_1byte(imx585, IMX595_REG_CCMP_EN, 0);
				imx585_write_reg_1byte(imx585, 0x3023, 0x03); // MDBIT 16-bit
				dev_info(&client->dev, "16bit HDR written\n");
				break;
			/* 12-bit */
			case MEDIA_BUS_FMT_SRGGB12_1X12:
			case MEDIA_BUS_FMT_SGRBG12_1X12:
			case MEDIA_BUS_FMT_SGBRG12_1X12:
			case MEDIA_BUS_FMT_SBGGR12_1X12:
			case MEDIA_BUS_FMT_Y12_1X12:
				imx585_write_reg_1byte(imx585, IMX595_REG_CCMP_EN, 1);
				dev_info(&client->dev, "12bit HDR written\n");
				break;
			default:
				break;
		}
		dev_info(&client->dev, "ClearHDR_regs_written\n");

	}
	else{
		ret = imx585_write_regs(imx585, &imx585->regs->normal_common_regs);
		if (ret) {
			dev_err(&client->dev, "%s failed to set Normal settings\n", __func__);
			return ret;
		}
		dev_info(&client->dev, "normal_regs_written\n");
	}


	/* Disable digital clamp */
	imx585_write_reg_1byte(imx585, IMX585_REG_DIGITAL_CLAMP, 0);

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx585->sd.ctrl_handler);
	if (ret) {
		dev_err(&client->dev, "%s failed to apply user values\n", __func__);
		return ret;
	}

	if (imx585->sync_mode <= 1){
		dev_info(&client->dev,"imx585 Leader mode enabled\n");
		imx585_write_reg_1byte(imx585, IMX585_REG_XMSTA, 0x00);
    }

	/* Set stream on register */
	ret = imx585_write_reg_1byte(imx585, IMX585_REG_MODE_SELECT, IMX585_MODE_STREAMING);

	dev_info(&client->dev, "Start Streaming\n");
	usleep_range(IMX585_STREAM_DELAY_US, IMX585_STREAM_DELAY_US + IMX585_STREAM_DELAY_RANGE_US);
	return ret;
}

/* Stop streaming */
static void imx585_stop_streaming(struct imx585 *imx585)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	int ret;

	dev_info(&client->dev, "Stop Streaming\n");

	/* set stream off register */
	ret = imx585_write_reg_1byte(imx585, IMX585_REG_MODE_SELECT, IMX585_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to stop stream\n", __func__);
}

static int imx585_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx585 *imx585 = to_imx585(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx585->mutex);
	if (imx585->streaming == enable) {
		mutex_unlock(&imx585->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx585_start_streaming(imx585);
		if (ret)
			goto err_rpm_put;
	} else {
		imx585_stop_streaming(imx585);
		pm_runtime_put(&client->dev);
	}

	imx585->streaming = enable;

	/* vflip/hflip and hdr mode cannot change during streaming */
	__v4l2_ctrl_grab(imx585->vflip, enable);
	__v4l2_ctrl_grab(imx585->hflip, enable);
	__v4l2_ctrl_grab(imx585->hdr_mode, enable);

	mutex_unlock(&imx585->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx585->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx585_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx585 *imx585 = to_imx585(sd);
	int ret;

	ret = regulator_bulk_enable(imx585_NUM_SUPPLIES,
				    imx585->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx585->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx585->reset_gpio, 1);
	usleep_range(imx585_XCLR_MIN_DELAY_US,
		     imx585_XCLR_MIN_DELAY_US + imx585_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(imx585_NUM_SUPPLIES, imx585->supplies);
	return ret;
}

static int imx585_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx585 *imx585 = to_imx585(sd);

	gpiod_set_value_cansleep(imx585->reset_gpio, 0);
	regulator_bulk_disable(imx585_NUM_SUPPLIES, imx585->supplies);
	clk_disable_unprepare(imx585->xclk);

	/* Force reprogramming of the common registers when powered up again. */
	imx585->common_regs_written = false;

	return 0;
}

static int __maybe_unused imx585_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx585 *imx585 = to_imx585(sd);

	if (imx585->streaming)
		imx585_stop_streaming(imx585);

	return 0;
}

static int __maybe_unused imx585_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx585 *imx585 = to_imx585(sd);
	int ret;

	if (imx585->streaming) {
		ret = imx585_start_streaming(imx585);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx585_stop_streaming(imx585);
	imx585->streaming = 0;
	return ret;
}

static int imx585_get_regulators(struct imx585 *imx585)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	unsigned int i;

	for (i = 0; i < imx585_NUM_SUPPLIES; i++)
		imx585->supplies[i].supply = imx585_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       imx585_NUM_SUPPLIES,
				       imx585->supplies);
}

/* Verify chip ID */
static int imx585_check_module_exists(struct imx585 *imx585)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	int ret;
	u32 val;

	/* We don't actually have a CHIP ID register so we try to read from BLKLEVEL instead*/
	ret = imx585_read_reg(imx585, IMX585_REG_BLKLEVEL,
			      1, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id, with error %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "Device found, ID: %x\n", val);

	return 0;
}

static int imx585_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx585 *imx585 = to_imx585(sd);

		mutex_lock(&imx585->mutex);
		sel->r = *__imx585_get_pad_crop(imx585, sd_state, sel->pad, sel->which);
		mutex_unlock(&imx585->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX585_NATIVE_WIDTH;
		sel->r.height = IMX585_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX585_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX585_PIXEL_ARRAY_TOP;
		sel->r.width = IMX585_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX585_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_subdev_core_ops imx585_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx585_video_ops = {
	.s_stream = imx585_set_stream,
};

static const struct v4l2_subdev_pad_ops imx585_pad_ops = {
	.enum_mbus_code = imx585_enum_mbus_code,
	.get_fmt = imx585_get_pad_format,
	.set_fmt = imx585_set_pad_format,
	.get_selection = imx585_get_selection,
	.enum_frame_size = imx585_enum_frame_size,
};

static const struct v4l2_subdev_ops imx585_subdev_ops = {
	.core = &imx585_core_ops,
	.video = &imx585_video_ops,
	.pad = &imx585_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx585_internal_ops = {
	.open = imx585_open,
};

/* Initialize control handlers */
static int imx585_init_controls(struct imx585 *imx585)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	struct v4l2_fwnode_device_properties props;
	int ret;

	ctrl_hdlr = &imx585->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 32);
	if (ret)
		return ret;

	mutex_init(&imx585->mutex);
	ctrl_hdlr->lock = &imx585->mutex;

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx585_set_framing_limits() call below.
	 */
	/* By default, PIXEL_RATE is read only */
	imx585->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       0xffff,
					       0xffff, 1,
					       0xffff);

	/* LINK_FREQ is also read only */
	imx585->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx585_ctrl_ops,
				       V4L2_CID_LINK_FREQ, 0, 0,
				       &link_freqs[imx585->link_freq_idx]);
	if (imx585->link_freq)
		imx585->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx585->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xfffff, 1, 0);
	imx585->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);
	imx585->blacklevel = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops,
					   V4L2_CID_BRIGHTNESS, 0, 0xffff, 1, IMX585_BLKLEVEL_DEFAULT);

	imx585->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX585_EXPOSURE_MIN,
					     IMX585_EXPOSURE_MAX,
					     IMX585_EXPOSURE_STEP,
					     IMX585_EXPOSURE_DEFAULT);

	imx585->gain = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			                         IMX585_ANA_GAIN_MIN, IMX585_ANA_GAIN_MAX_NORMAL,
			                         IMX585_ANA_GAIN_STEP, IMX585_ANA_GAIN_DEFAULT);

	imx585->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	imx585->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (imx585->has_ircut) {
		imx585->ircut_ctrl =
		    v4l2_ctrl_new_std(&imx585->ctrl_handler, &imx585_ctrl_ops,
				      V4L2_CID_BAND_STOP_FILTER,
				      0, 1, 1, 1);
	}

	if(imx585->regs->clearHDR){
		imx585->hdr_mode = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops,
						     V4L2_CID_WIDE_DYNAMIC_RANGE,
						     0, 1, 1, 0);
		imx585->datasel_th_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr, &imx585_cfg_hdr_datasel_th, NULL);
		imx585->datasel_bk_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr, &imx585_cfg_hdr_datasel_bk, NULL);
		imx585->gdc_th_ctrl     = v4l2_ctrl_new_custom(ctrl_hdlr, &imx585_cfg_hdr_grad_th, NULL);
	    imx585->gdc_exp_ctrl_l  = v4l2_ctrl_new_custom(ctrl_hdlr, &imx585_cfg_hdr_grad_exp_l, NULL);
	    imx585->gdc_exp_ctrl_h  = v4l2_ctrl_new_custom(ctrl_hdlr, &imx585_cfg_hdr_grad_exp_h, NULL);
		imx585->hdr_gain_ctrl   = v4l2_ctrl_new_custom(ctrl_hdlr, &imx585_cfg_hdr_gain, NULL);
		imx585->hgc_ctrl        = v4l2_ctrl_new_custom(ctrl_hdlr, &imx585_cfg_hgc, NULL);

		v4l2_ctrl_activate(imx585->datasel_th_ctrl,  imx585->clear_HDR);
		v4l2_ctrl_activate(imx585->datasel_bk_ctrl,  imx585->clear_HDR);
		v4l2_ctrl_activate(imx585->gdc_th_ctrl,      imx585->clear_HDR);
		v4l2_ctrl_activate(imx585->gdc_exp_ctrl_l,   imx585->clear_HDR);
		v4l2_ctrl_activate(imx585->gdc_exp_ctrl_h,   imx585->clear_HDR);
		v4l2_ctrl_activate(imx585->hdr_gain_ctrl,    imx585->clear_HDR);
		v4l2_ctrl_activate(imx585->hgc_ctrl,        !imx585->clear_HDR);
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx585_ctrl_ops, &props);
	if (ret)
		goto error;

    if(imx585->regs->clearHDR){
	    memcpy(imx585->datasel_th_ctrl->p_cur.p, hdr_thresh_def, sizeof(hdr_thresh_def));
	    memcpy(imx585->datasel_th_ctrl->p_new.p, hdr_thresh_def, sizeof(hdr_thresh_def));
	    memcpy(imx585->gdc_th_ctrl->p_cur.p, grad_thresh_def, sizeof(grad_thresh_def));
	    memcpy(imx585->gdc_th_ctrl->p_new.p, grad_thresh_def, sizeof(grad_thresh_def));

		imx585->hdr_mode->flags |= V4L2_CTRL_FLAG_UPDATE;
		imx585->hdr_mode->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	}

	imx585->sd.ctrl_handler = ctrl_hdlr;

	/* Setup exposure and frame/line length limits. */
	imx585_set_framing_limits(imx585);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx585->mutex);

	return ret;
}

static void imx585_free_controls(struct imx585 *imx585)
{
	v4l2_ctrl_handler_free(imx585->sd.ctrl_handler);
	mutex_destroy(&imx585->mutex);
}

static const struct imx585_compatible_data imx585_compatible = {
	.name = "IMX585",
	.clearHDR = true,
	.analog_gain_reg = IMX585_REG_ANALOG_GAIN,
	.common_regs = {
		.num_of_regs = ARRAY_SIZE(imx585_common_regs),
		.regs =  imx585_common_regs,
	},
	.clearHDR_common_regs = {
		.num_of_regs = ARRAY_SIZE(imx585_common_clearHDR_mode),
		.regs =  imx585_common_clearHDR_mode,
	},
	.normal_common_regs = {
		.num_of_regs = ARRAY_SIZE(imx585_common_normal_mode),
		.regs =  imx585_common_normal_mode,
	},
};


static const struct imx585_compatible_data imx678_compatible = {
	.name = "IMX678",
	.clearHDR = false,
	.analog_gain_reg = IMX678_REG_ANALOG_GAIN,
	.common_regs = {
		.num_of_regs = ARRAY_SIZE(imx678_common_regs),
		.regs =  imx678_common_regs,
	},
	.clearHDR_common_regs = {
		.num_of_regs = 0,
		.regs = NULL
	},
	.normal_common_regs = {
		.num_of_regs = ARRAY_SIZE(imx678_common_normal_mode),
		.regs =  imx678_common_normal_mode,
	},
};


static const struct of_device_id imx585_dt_ids[] = {
	{ .compatible = "sony,imx585", .data = &imx585_compatible },
	{ .compatible = "sony,imx678", .data = &imx678_compatible },
	{ /* sentinel */ }
};


static int imx585_check_hwcfg(struct device *dev, struct imx585 *imx585)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;
	int i;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2 && ep_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(dev, "only 2 or 4 data lanes are currently supported\n");
		goto error_out;
	}
	imx585->lane_count = ep_cfg.bus.mipi_csi2.num_data_lanes;
	dev_info(dev, "Data lanes: %d\n", imx585->lane_count);

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	for (i = 0; i < ARRAY_SIZE(link_freqs); i++) {
		if (link_freqs[i] == ep_cfg.link_frequencies[0]) {
			imx585->link_freq_idx = i;
			break;
		}
	}

	if (i == ARRAY_SIZE(link_freqs)) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
			ret = -EINVAL;
			goto error_out;
	}

	dev_info(dev, "Link Speed: %lld Mhz\n", ep_cfg.link_frequencies[0]);

	imx585_update_hmax(imx585);

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int imx585_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node  *np;
	struct imx585 *imx585;
	const struct of_device_id *match;
	int ret, i;
	u32 sync_mode;

	imx585 = devm_kzalloc(&client->dev, sizeof(*imx585), GFP_KERNEL);
	if (!imx585)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx585->sd, client, &imx585_subdev_ops);

	match = of_match_device(imx585_dt_ids, dev);
	if (!match)
		return -ENODEV;
	imx585->regs =
		(const struct imx585_compatible_data *)match->data;
	dev_info(dev, "Driver setup for: %s\n", imx585->regs->name);
	dev_info(dev, "Reading dtoverlay config:\n");
	imx585->mono = of_property_read_bool(dev->of_node, "mono-mode");
	if(imx585->mono){
		dev_info(dev, "Mono Mode Selected, make sure you have the correct sensor variant\n");
	}
	
	if(imx585->regs->clearHDR){
		imx585->clear_HDR = of_property_read_bool(dev->of_node, "clearHDR-mode");
		dev_info(dev, "ClearHDR: %d\n", imx585->clear_HDR);
	}

	imx585->sync_mode = 0;
	ret = of_property_read_u32(dev->of_node, "sync-mode", &sync_mode);
    if (!ret) {
            if (sync_mode > 2) {
                    dev_warn(dev, "sync-mode out of range, using 0\n");
                    sync_mode = 0;
            }
            imx585->sync_mode = sync_mode;
    } else if (ret != -EINVAL) {          /* property present but bad */
            dev_err(dev, "sync-mode malformed (%pe)\n",
                    ERR_PTR(ret));
            return ret;
    }
    dev_info(dev, "Sync Mode: %s\n", sync_mode_menu[imx585->sync_mode]);

	/* Check the hardware configuration in device tree */
	if (imx585_check_hwcfg(dev, imx585))
		return -EINVAL;

	/* Get system clock (xclk) */
	imx585->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx585->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx585->xclk);
	}

	imx585->xclk_freq = clk_get_rate(imx585->xclk);

	for (i = 0; i < ARRAY_SIZE(imx585_inck_table); ++i) {
		if (imx585_inck_table[i].xclk_hz == imx585->xclk_freq) {
			imx585->inck_sel_val = imx585_inck_table[i].inck_sel;
			break;
		}
	}

	if (i == ARRAY_SIZE(imx585_inck_table)) {
		dev_err(dev, "unsupported XCLK rate %u Hz\n",
			imx585->xclk_freq);
		return -EINVAL;
	}

	dev_info(dev, "XCLK %u Hz → INCK_SEL 0x%02x\n",
		 imx585->xclk_freq, imx585->inck_sel_val);

	ret = imx585_get_regulators(imx585);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx585->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for imx585_check_module_exists()
	 * to be able to read register
	 */
	ret = imx585_power_on(dev);
	if (ret)
		return ret;

	ret = imx585_check_module_exists(imx585);
	if (ret)
		goto error_power_off;


	imx585->has_ircut     = false;
	imx585->ircut_client  = NULL;

	if(of_property_read_bool(dev->of_node, "ircut-mode")){
		np = of_parse_phandle(dev->of_node, "ircut-controller", 0);
		if (np){
			imx585->ircut_client = of_find_i2c_device_by_node(np);
			of_node_put(np);
			
			ret = imx585_ircut_write(imx585, 0x01);
			if (!ret){
				imx585->has_ircut    = true;
				dev_info(dev, "IR-cut controller present at 0x%02x\n", imx585->ircut_client->addr);
			}
			else{
				dev_info(dev, "Can not communicate with IR-cut controller, drop the support\n");
			}
		}
	} else{
		dev_info(dev, "No IR-cut controller\n");
	}

	/* Initialize default format */
	imx585_set_default_format(imx585);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx585_init_controls(imx585);
	if (ret)
		goto error_pm_runtime;

	/* Initialize subdev */
	imx585->sd.internal_ops = &imx585_internal_ops;
	imx585->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	imx585->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx585->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx585->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx585->sd.entity, NUM_PADS, imx585->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx585->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx585->sd.entity);

error_handler_free:
	imx585_free_controls(imx585);

error_pm_runtime:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);

error_power_off:
	imx585_power_off(&client->dev);

	return ret;
}

static void imx585_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx585 *imx585 = to_imx585(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx585_free_controls(imx585);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx585_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

MODULE_DEVICE_TABLE(of, imx585_dt_ids);

static const struct dev_pm_ops imx585_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx585_suspend, imx585_resume)
	SET_RUNTIME_PM_OPS(imx585_power_off, imx585_power_on, NULL)
};

static struct i2c_driver imx585_i2c_driver = {
	.driver = {
		.name = "imx585",
		.of_match_table = imx585_dt_ids,
		.pm = &imx585_pm_ops,
	},
	.probe = imx585_probe,
	.remove = imx585_remove,
};

module_i2c_driver(imx585_i2c_driver);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_AUTHOR("Tetsuya NOMURA <tetsuya.nomura@soho-enterprise.com>");
MODULE_DESCRIPTION("Sony imx585 sensor driver");
MODULE_LICENSE("GPL");
