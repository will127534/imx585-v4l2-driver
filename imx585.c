// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony imx585 cameras.
 *
 * Based on Sony imx477 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 * Modified by Will WHANG
 * Modified by sohonomura2020 in Soho Enterprise Ltd.
 * Modified by OCTOPUSCINEMA
 * Copyright (C) 2024 OCTOPUS CINEMA
 */
#include <asm/unaligned.h>
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
#define MEDIA_BUS_FMT_SENSOR_DATA 		0x7002
#endif

/* Chip ID */
#define IMX585_REG_CHIP_ID				0x30DC
#define IMX585_CHIP_ID					0x32

/* Standby or streaming mode */
#define IMX585_REG_MODE_SELECT			0x3000
#define IMX585_MODE_STANDBY				0x01
#define IMX585_MODE_STREAMING			0x00
#define IMX585_STREAM_DELAY_US			25000
#define IMX585_STREAM_DELAY_RANGE_US	1000

/* In clk */
#define IMX585_XCLK_FREQ				24000000

/* VMAX internal VBLANK*/
#define IMX585_REG_VMAX					0x3028
#define IMX585_VMAX_MAX					0xfffff

/* HMAX internal HBLANK*/
#define IMX585_REG_HMAX					0x302C
#define IMX585_HMAX_MAX					0xffff

/* SHR internal */
#define IMX585_REG_SHR					0x3050
#define IMX585_SHR_MIN					11

/* Exposure control */
#define IMX585_EXPOSURE_MIN				52
#define IMX585_EXPOSURE_STEP			1
#define IMX585_EXPOSURE_DEFAULT			1000
#define IMX585_EXPOSURE_MAX				49865

/* HDR threshold */
#define IMX585_REG_EXP_TH_H				0x36D0
#define IMX585_REG_EXP_TH_L				0x36D4
#define IMX585_REG_EXP_BK				0x36E2

/* Gradation compression control */
#define IMX585_REG_CCMP1_EXP			0x36E8
#define IMX585_REG_CCMP2_EXP			0x36E4
#define IMX585_REG_ACMP1_EXP			0x36EE
#define IMX585_REG_ACMP2_EXP			0x36EC

/* Black level control */
#define IMX585_REG_BLKLEVEL				0x30DC
#define IMX585_BLKLEVEL_DEFAULT			0

/* Digital Clamp */
#define IMX585_REG_DIGITAL_CLAMP		0x3458

/* Analog gain control */
#define IMX585_REG_ANALOG_GAIN			0x306C
#define IMX585_REG_FDG_SEL0				0x3030
#define IMX585_ANA_GAIN_MIN				0
#define IMX585_ANA_GAIN_MAX				240 // x3980= 72db = 0.3db x 240
#define IMX585_ANA_GAIN_STEP			1
#define IMX585_ANA_GAIN_DEFAULT			0
#define IMX585_ANA_GAIN_HCG_LEVEL		51 // = 15.3db / 0.3db
#define IMX585_ANA_GAIN_HCG_THRESHOLD	(IMX585_ANA_GAIN_HCG_LEVEL+29)
#define IMX585_ANA_GAIN_HCG_MIN			34

/* Flip */
#define IMX585_FLIP_WINMODEH    		0x3020
#define IMX585_FLIP_WINMODEV    		0x3021

/* Embedded metadata stream structure */
#define IMX585_EMBEDDED_LINE_WIDTH 		16384
#define IMX585_NUM_EMBEDDED_LINES 		1

#define IMX585_PIXEL_RATE				74250000

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};


/* Gradation compression */
enum v4l2_xfer_func_sony {
	V4L2_XFER_FUNC_GRADATION_COMPRESSION = 10
};

/* imx585 native and active pixel array size. */
#define IMX585_NATIVE_WIDTH			3856U
#define IMX585_NATIVE_HEIGHT		2180U
#define IMX585_PIXEL_ARRAY_LEFT		8U
#define IMX585_PIXEL_ARRAY_TOP		8U
#define IMX585_PIXEL_ARRAY_WIDTH	3840U
#define IMX585_PIXEL_ARRAY_HEIGHT	2160U

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

//4K min HMAX for 4-lane, times 2 for 2-lane
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

	/* mode uses Clear HDR */
	bool hdr;

	/* mode has linear output (gradation compression disabled) */
	bool linear;

	/* minimum H-timing */
	uint64_t min_HMAX;

	/* minimum V-timing */
	uint64_t min_VMAX;

	/* default H-timing */
	uint64_t default_HMAX;

	/* default V-timing */
	uint64_t default_VMAX;

	/* minimum SHR */
	uint64_t min_SHR;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct IMX585_reg_list reg_list;
};

/* Common Modes */
struct imx585_reg mode_common_regs[] = {
    {0x3002, 0x01},
    {0x301A, 0x00}, //WDMODE Normal mode
    //{0x301A, 0x10}, //WDMODE Clear HDR
    {0x301B, 0x00}, //ADDMODE 0x00 non-binning
    {0x3024, 0x00}, // COMBI_EN 
    // {0x3024, 0x02}, // COMBI_EN 0x02=Clear HDR mode
    {0x3069, 0x00},
    //{0x3069, 0x02}, // for Clear HDR mode
    {0x3074, 0x64},
	{0x30D5, 0x04}, // DIG_CLP_VSTART
    // {0x3074, 0x63}, // for Clear HDR
    {0x3930, 0x0c},//DUR normal mode 12bit
    {0x3931, 0x01},//DUR normal mode 12bit
    // {0x3930, 0xE6},//DUR Clear HDR 12bit
    // {0x3931, 0x00},//DUR Clear HDR 12bit
    {0x3A4C, 0x39},// WAIT_ST0 Normal
    {0x3A4D, 0x01},//  Normal
    {0x3A50, 0x48},// WAIT_ST1 Normal
    {0x3A51, 0x01},//  Normal
    // {0x3A4C, 0x61},// WAIT_ST0
    // {0x3A4D, 0x02},// 
    // {0x3A50, 0x70},// WAIT_ST1
    // {0x3A51, 0x02},// 
    {0x3E10, 0x10},// ADTHEN Normal
    // {0x3E10, 0x17},// ADTHEN
    {0x493C, 0x23},// ADTHEN
    {0x4940, 0x41},// ADTHEN
    // {0x493C, 0x41},// ADTHEN
    // {0x4940, 0x41},// ADTHEN



    {0x3014, 0x04},// INCK_SEL [3:0] 24 MHz
    {0x3015, 0x02},// DATARATE_SEL [3:0]  1782 Mbps
    // {0x302C, 0x4C},// HMAX [15:0]
    // {0x302D, 0x04},// 
    {0x3030, 0x00},// FDG_SEL0 LCG, HCG:0x01
    {0x3040, 0x03},// LANEMODE [2:0] 4 lane
    {0x3023, 0x01},// MDBIT 12-bit
    // {0x3028, 0x94},// VMAX
    // {0x3029, 0x11},// VMAX
    // {0x302A, 0x00},// VMAX
    // {0x3050, 0xFF},// SHR0 [19:0]
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
    {0x3002, 0x00}, // Master mode start
};

/* All pixel 4K60. 12-bit (Normal) */
static const struct imx585_reg mode_4k_regs[] = {
	{0x301A, 0x00}, // WDMODE Normal mode
	{0x301B, 0x00}, // ADDMODE non-binning
	{0x3022, 0x02}, // ADBIT 12-bit
	{0x3023, 0x01}, // MDBIT 12-bit
	{0x3024, 0x00}, // COMBI_EN no HDR combining
	{0x36EF, 0x00}, // CCMP_EN Linear
    {0x3069, 0x00}, // Normal mode
	
	{0x3074, 0x64}, // Normal mode
	{0x30D5, 0x04}, // DIG_CLP_VSTART non-binning
    {0x3930, 0x0c}, // DUR normal mode 12bit
    {0x3931, 0x01}, // DUR normal mode 12bit
    {0x3A4C, 0x39}, // WAIT_ST0 Normal mode
    {0x3A4D, 0x01}, // Normal mode
    {0x3A50, 0x48}, // WAIT_ST1 Normal mode
    {0x3A51, 0x01}, // Normal mode
    {0x3E10, 0x10}, // ADTHEN Normal mode
    {0x493C, 0x23}, // ADTHEN Normal mode
    {0x4940, 0x41}, // ADTHEN Normal mode
};

/* 2x2 binned 1080p60. 12-bit (Normal) */
static const struct imx585_reg mode_1080_regs[] = {
	{0x301A, 0x00}, // WDMODE Normal mode
	{0x301B, 0x01}, // ADDMODE binning
	{0x3022, 0x00}, // ADBIT 10-bit
	{0x3023, 0x01}, // MDBIT 12-bit
	{0x3024, 0x00}, // COMBI_EN no HDR combining
	{0x36EF, 0x00}, // CCMP_EN Linear
	{0x3069, 0x00}, // Normal mode
	
	{0x3074, 0x64}, // Normal mode
	{0x30D5, 0x02}, // DIG_CLP_VSTART binning
    {0x3930, 0x0c}, // DUR normal mode 12bit
    {0x3931, 0x01}, // DUR normal mode 12bit
    {0x3A4C, 0x39}, // WAIT_ST0 Normal mode
    {0x3A4D, 0x01}, // Normal mode
    {0x3A50, 0x48}, // WAIT_ST1 Normal mode
    {0x3A51, 0x01}, // Normal mode
    {0x3E10, 0x10}, // ADTHEN Normal mode
    {0x493C, 0x23}, // ADTHEN Normal mode
    {0x4940, 0x41}, // ADTHEN Normal mode
};

/* All pixel 4K30. 12-bit (HDR gradation compression) */
static const struct imx585_reg mode_4k_nonlinear_regs[] = {
    {0x301A, 0x10}, // WDMODE Clear HDR
    {0x301B, 0x00}, // ADDMODE Non-binning
	
	{0x3022, 0x02}, // ADBIT 12-bit
    {0x3023, 0x01}, // MDBIT 12-bit
    {0x3024, 0x02}, // COMBI_EN 

	{0x36EF, 0x01}, // CCMP_EN Non-linear gradation compression
	
    {0x3030, 0x00}, // FDG_SEL0 LCG, HCG:0x01
	
    {0x3069, 0x02}, // for Clear HDR mode
    {0x3074, 0x63}, // for Clear HDR
    {0x3081, 0x02}, // EXP_GAIN, Clear HDR high gain setting, +12dB
    
    {0x30D5, 0x02}, // DIG_CLP_VSTART Non-binning
	
    {0x3930, 0xE6}, // DUR Clear HDR 12bit
    {0x3931, 0x00}, // DUR Clear HDR 12bit
    
    {0x3A4C, 0x61}, // WAIT_ST0 Clear HDR mode
    {0x3A4D, 0x02}, // Clear HDR mode
    {0x3A50, 0x70}, // WAIT_ST1
    {0x3A51, 0x02}, // Clear HDR mode
    
    {0x3E10, 0x17}, // ADTHEN Clear HDR
    {0x493C, 0x41}, // WAIT_10_SHF Clear HDR 10-bit 0x0C disable
    {0x4940, 0x41}, // WAIT_12_SHF Clear HDR 12-bit 0x41 enable
};

/* All pixel 4K30. 16-bit (Clear HDR) */
static const struct imx585_reg mode_4k_16bit_regs[] = {
    {0x301A, 0x10}, // WDMODE Clear HDR
    {0x301B, 0x00}, // ADDMODE Non-binning
	
	{0x3022, 0x02}, // ADBIT 12-bit
    {0x3023, 0x03}, // MDBIT 16-bit
    {0x3024, 0x02}, // COMBI_EN 

	{0x36EF, 0x00}, // CCMP_EN Linear
	
    {0x3030, 0x00}, // FDG_SEL0 LCG, HCG:0x01
	
    {0x3069, 0x02}, // for Clear HDR mode
    {0x3074, 0x63}, // for Clear HDR
    {0x3081, 0x02}, // EXP_GAIN, Clear HDR high gain setting, +12dB
    
    {0x30D5, 0x02}, // DIG_CLP_VSTART Non-binning
	
    {0x3930, 0xE6}, // DUR Clear HDR 12bit
    {0x3931, 0x00}, // DUR Clear HDR 12bit
    
    {0x3A4C, 0x61}, // WAIT_ST0 Clear HDR mode
    {0x3A4D, 0x02}, // Clear HDR mode
    {0x3A50, 0x70}, // WAIT_ST1
    {0x3A51, 0x02}, // Clear HDR mode
    
    {0x3E10, 0x17}, // ADTHEN Clear HDR
    {0x493C, 0x41}, // WAIT_10_SHF Clear HDR 10-bit 0x0C disable
    {0x4940, 0x41}, // WAIT_12_SHF Clear HDR 12-bit 0x41 enable
};

/* 2x2 binned 1080p30. 16-bit (Clear HDR) */
static const struct imx585_reg mode_1080_16bit_regs[] = {
    {0x301A, 0x10}, // WDMODE Clear HDR
    {0x301B, 0x01}, // ADDMODE Binning
	
	{0x3022, 0x02}, // ADBIT 12-bit
    {0x3023, 0x03}, // MDBIT 16-bit
    {0x3024, 0x02}, // COMBI_EN Built-in HDR combining

	{0x36EF, 0x00}, // CCMP_EN Linear
	
    {0x3030, 0x00}, // FDG_SEL0 LCG, HCG:0x01
	
    {0x3069, 0x02}, // for Clear HDR mode
    {0x3074, 0x63}, // for Clear HDR
    {0x3081, 0x02}, // EXP_GAIN, Clear HDR high gain setting, +12dB
    
    {0x30D5, 0x02}, // DIG_CLP_VSTART
	
    {0x3930, 0xE6}, // DUR Clear HDR 12bit
    {0x3931, 0x00}, // DUR Clear HDR 12bit
    
    {0x3A4C, 0x61}, // WAIT_ST0
    {0x3A4D, 0x02}, // Clear HDR mode
    {0x3A50, 0x70}, // WAIT_ST1
    {0x3A51, 0x02}, // Clear HDR mode

    {0x3E10, 0x17}, // ADTHEN Clear HDR
    {0x493C, 0x41}, // WAIT_10_SHF Clear HDR 10-bit 0x0C disable
    {0x4940, 0x41}, // WAIT_12_SHF Clear HDR 12-bit 0x41 enable
};

/* Mode configs */
struct imx585_mode supported_modes_12bit[] = {
	{
		/* 1080p90 2x2 binning */
		.width = 1928,
		.height = 1090,
		.hdr = false,
		.linear = true,
		.min_HMAX = 366,
		.min_VMAX = 2250,
		.default_HMAX = 366,
		.default_VMAX = 2250,
		.min_SHR = 20,
		.crop = {
			.left = IMX585_PIXEL_ARRAY_LEFT,
			.top = IMX585_PIXEL_ARRAY_TOP,
			.width = IMX585_PIXEL_ARRAY_WIDTH,
			.height = IMX585_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1080_regs),
			.regs = mode_1080_regs,
		},
	},
    {
        /* 4K60 All pixel */
        .width = 3856,
        .height = 2180,
        .hdr = false,
        .linear = true,
        .min_HMAX = 550,
        .min_VMAX = 2250,
        .default_HMAX = 550,
        .default_VMAX = 2250,
        .min_SHR = 20,
        .crop = {
            .left = IMX585_PIXEL_ARRAY_LEFT,
            .top = IMX585_PIXEL_ARRAY_TOP,
            .width = IMX585_PIXEL_ARRAY_WIDTH,
            .height = IMX585_PIXEL_ARRAY_HEIGHT,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_4k_regs),
            .regs = mode_4k_regs,
        },
    },
};

struct imx585_mode supported_modes_nonlinear_12bit[] = {
    {
        /* 1080P30 All pixel */
        .width = 1928,
        .height = 1090,
        .hdr = true,
        .linear = false,
        .min_HMAX = 366, // Clear HDR original
        .min_VMAX = 2250, // Clear HDR original
        .default_HMAX = 366,
        .default_VMAX = 2250,
        .min_SHR = 20,
        .crop = {
            .left = IMX585_PIXEL_ARRAY_LEFT,
            .top = IMX585_PIXEL_ARRAY_TOP,
            .width = IMX585_PIXEL_ARRAY_WIDTH,
            .height = IMX585_PIXEL_ARRAY_HEIGHT,
        },
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mode_4k_nonlinear_regs),
            .regs = mode_4k_nonlinear_regs,
        },
    },
	{
		/* 4K30 All pixel */
		.width = 3856,
		.height = 2180,
		.hdr = true,
		.linear = false,
		//.min_HMAX = 760,
		.min_HMAX = 550, // Clear HDR original
		//.min_VMAX = 2250,
		.min_VMAX = 4500, // Clear HDR original
		.default_HMAX = 550,
		.default_VMAX = 4500,
		// .default_HMAX = 550,
		// .default_VMAX = 4500,
		.min_SHR = 20,
		.crop = {
			.left = IMX585_PIXEL_ARRAY_LEFT,
			.top = IMX585_PIXEL_ARRAY_TOP,
			.width = IMX585_PIXEL_ARRAY_WIDTH,
			.height = IMX585_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4k_nonlinear_regs),
			.regs = mode_4k_nonlinear_regs,
		},
	},
};

struct imx585_mode supported_modes_16bit[] = {
	{
		/* 1080p30 2x2 binning */
		.width = 1928,
		.height = 1090,
		.hdr = true,
		.linear = true,
		//.min_HMAX = 760,
		.min_HMAX = 550, // Clear HDR original
		//.min_VMAX = 2250,
		.min_VMAX = 4500, // Clear HDR original
		.default_HMAX = 550,
		.default_VMAX = 4500,
		// .default_HMAX = 550,
		// .default_VMAX = 4500,
		.min_SHR = 20,
		.crop = {
			.left = IMX585_PIXEL_ARRAY_LEFT,
			.top = IMX585_PIXEL_ARRAY_TOP,
			.width = IMX585_PIXEL_ARRAY_WIDTH,
			.height = IMX585_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1080_16bit_regs),
			.regs = mode_1080_16bit_regs,
		},
	},
	{
		/* 4K30 All pixel */
		.width = 3856,
		.height = 2180,
		.hdr = true,
		.linear = true,
		//.min_HMAX = 760,
		.min_HMAX = 550, // Clear HDR original
		//.min_VMAX = 2250,
		.min_VMAX = 4500, // Clear HDR original
		.default_HMAX = 550,
		.default_VMAX = 4500,
		// .default_HMAX = 550,
		// .default_VMAX = 4500,
		.min_SHR = 20,
		.crop = {
			.left = IMX585_PIXEL_ARRAY_LEFT,
			.top = IMX585_PIXEL_ARRAY_TOP,
			.width = IMX585_PIXEL_ARRAY_WIDTH,
			.height = IMX585_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4k_16bit_regs),
			.regs = mode_4k_16bit_regs,
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
static const u32 codes[] = {
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


static const u32 mono_codes[] = {
    /* 16-bit modes. */
    MEDIA_BUS_FMT_Y16_1X16,
    /* 12-bit modes. */
    MEDIA_BUS_FMT_Y12_1X12,
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
#define imx585_XCLR_MIN_DELAY_US	500000
#define imx585_XCLR_DELAY_RANGE_US	1000

struct imx585_compatible_data {
	unsigned int chip_id;
	struct IMX585_reg_list extra_regs;
};

struct imx585 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[imx585_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
    struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct imx585_mode *mode;

    /* Mono mode */
    bool mono;

    unsigned int lane_count;
    unsigned int link_freq_idx;

	uint16_t HMAX;
	uint32_t VMAX;
	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	/* Any extra information related to different compatible sensors */
	const struct imx585_compatible_data *compatible_data;
};

static inline struct imx585 *to_imx585(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx585, sd);
}

static inline void get_mode_table(struct imx585 *imx585, unsigned int code, enum v4l2_xfer_func transfer_function,
				  const struct imx585_mode **mode_list,
				  unsigned int *num_modes)
{

    if(imx585->mono){
        switch (code) {
        case MEDIA_BUS_FMT_Y16_1X16:
            *mode_list = supported_modes_16bit;
            *num_modes = ARRAY_SIZE(supported_modes_16bit);
            break;
        case MEDIA_BUS_FMT_Y12_1X12:
            if ( transfer_function == (enum v4l2_xfer_func)V4L2_XFER_FUNC_GRADATION_COMPRESSION ) {
                *mode_list = supported_modes_nonlinear_12bit;
                *num_modes = ARRAY_SIZE(supported_modes_nonlinear_12bit);
            } else {
                *mode_list = supported_modes_12bit;
                *num_modes = ARRAY_SIZE(supported_modes_12bit);
            }
            break;
        default:
            *mode_list = NULL;
            *num_modes = 0;
        }
    }
    else{
        switch (code) {
        /* 16-bit */
        case MEDIA_BUS_FMT_SRGGB16_1X16:
        case MEDIA_BUS_FMT_SGRBG16_1X16:
        case MEDIA_BUS_FMT_SGBRG16_1X16:
        case MEDIA_BUS_FMT_SBGGR16_1X16:
            *mode_list = supported_modes_16bit;
            *num_modes = ARRAY_SIZE(supported_modes_16bit);
            break;
        /* 12-bit */
        case MEDIA_BUS_FMT_SRGGB12_1X12:
        case MEDIA_BUS_FMT_SGRBG12_1X12:
        case MEDIA_BUS_FMT_SGBRG12_1X12:
        case MEDIA_BUS_FMT_SBGGR12_1X12:
            if ( transfer_function == (enum v4l2_xfer_func)V4L2_XFER_FUNC_GRADATION_COMPRESSION ) {
                *mode_list = supported_modes_nonlinear_12bit;
                *num_modes = ARRAY_SIZE(supported_modes_nonlinear_12bit);
            } else {
                *mode_list = supported_modes_12bit;
                *num_modes = ARRAY_SIZE(supported_modes_12bit);
            }
            break;
        default:
            *mode_list = NULL;
            *num_modes = 0;
        }
    }
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
	if ( ret != 3 )
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
	buf[3] = val>>8;
	ret = i2c_master_send(client, buf, 4);
	if ( ret != 4 )
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
	buf[3]  = val>>8;
	buf[4]  = val>>16;
	if (i2c_master_send(client, buf, 5) != 5)
		return -EIO;

	return 0;
}

/* Write a list of 1 byte registers */
static int imx585_write_regs(struct imx585 *imx585,
			     const struct imx585_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx585_write_reg_1byte(imx585, regs[i].address, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
						"Failed to write reg 0x%4.4x. error = %d\n",
						regs[i].address, ret);

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

    if(imx585->mono){
        for (i = 0; i < ARRAY_SIZE(mono_codes); i++)
            if (mono_codes[i] == code)
                break;
        return mono_codes[i];
    }
    else{
        for (i = 0; i < ARRAY_SIZE(codes); i++)
            if (codes[i] == code)
                break;
        return codes[i];
    }

	
}

static void imx585_set_default_format(struct imx585 *imx585)
{
	/* Set default mode to max resolution */
	imx585->mode = &supported_modes_12bit[0];
    if(imx585->mono){
        imx585->fmt_code = MEDIA_BUS_FMT_Y12_1X12;
    }
    else{
        imx585->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
    }
	
}

static int imx585_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx585 *imx585 = to_imx585(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx585->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes_12bit[0].width;
	try_fmt_img->height = supported_modes_12bit[0].height;
    if(imx585->mono){
        try_fmt_img->code = imx585_get_format_code(imx585, MEDIA_BUS_FMT_Y12_1X12);
    }
    else{
        try_fmt_img->code = imx585_get_format_code(imx585, MEDIA_BUS_FMT_SRGGB12_1X12);
    }
	
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = IMX585_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = IMX585_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, IMAGE_PAD);
	try_crop->left = IMX585_PIXEL_ARRAY_LEFT;
	try_crop->top = IMX585_PIXEL_ARRAY_TOP;
	try_crop->width = IMX585_PIXEL_ARRAY_WIDTH;
	try_crop->height = IMX585_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx585->mutex);

	return 0;
}


static u64 calculate_v4l2_cid_exposure(u64 hmax, u64 vmax, u64 shr, u64 svr, u64 offset) {
    u64 numerator;
    numerator = (vmax * (svr + 1) - shr) * hmax + offset;

    do_div(numerator, hmax);
    numerator = clamp_t(uint32_t, numerator, 0, 0xFFFFFFFF);
    return numerator;
}

static void calculate_min_max_v4l2_cid_exposure(u64 hmax, u64 vmax, u64 min_shr, u64 svr, u64 offset, u64 *min_exposure, u64 *max_exposure) {
    u64 max_shr = (svr + 1) * vmax - 4;
    max_shr = min_t(uint64_t, max_shr, 0xFFFF);

    *min_exposure = calculate_v4l2_cid_exposure(hmax, vmax, max_shr, svr, offset);
    *max_exposure = calculate_v4l2_cid_exposure(hmax, vmax, min_shr, svr, offset);
}


/*
Integration Time [s] = [{VMAX × (SVR + 1) – (SHR)}
 × HMAX + offset] / (72 × 10^6)

Integration Time [s] = exposure * HMAX / (72 × 10^6)
*/

static uint32_t calculate_shr(uint32_t exposure, uint32_t hmax, uint64_t vmax, uint32_t svr, uint32_t offset)
{
    uint64_t temp;
    uint32_t shr;

    temp = ((uint64_t)exposure * hmax - offset);
    do_div(temp, hmax);
    shr = (uint32_t)(vmax * (svr + 1) - temp);

    return shr;
}

static int imx585_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx585 *imx585 = container_of(ctrl->handler, struct imx585, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	const struct imx585_mode *mode = imx585->mode;

	int ret = 0;

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK){
		/* Honour the VBLANK limits when setting exposure. */
		u64 current_exposure, max_exposure, min_exposure, vmax;
		vmax = ((u64)mode->height + ctrl->val) ;
		imx585 -> VMAX = vmax;
		
		calculate_min_max_v4l2_cid_exposure(imx585 -> HMAX, imx585 -> VMAX, (u64)mode->min_SHR, 0, 209, &min_exposure, &max_exposure);
		current_exposure = clamp_t(uint32_t, current_exposure, min_exposure, max_exposure);

		//dev_info(&client->dev,"exposure_max:%lld, exposure_min:%lld, current_exposure:%lld\n",max_exposure, min_exposure, current_exposure);
		dev_info(&client->dev,"\tVMAX:%d, HMAX:%d\n",imx585->VMAX, imx585->HMAX);
		__v4l2_ctrl_modify_range(imx585->exposure, min_exposure,max_exposure, 1,current_exposure);
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
			u64 shr;
			dev_info(&client->dev,"V4L2_CID_EXPOSURE : %d\n",ctrl->val);
			dev_info(&client->dev,"\tvblank:%d, hblank:%d\n",imx585->vblank->val, imx585->hblank->val);
			dev_info(&client->dev,"\tVMAX:%d, HMAX:%d\n",imx585->VMAX, imx585->HMAX);
			shr = calculate_shr(ctrl->val, imx585->HMAX, imx585->VMAX, 0, 209);
			dev_info(&client->dev,"\tSHR:%lld\n",shr);
			ret = imx585_write_reg_2byte(imx585, IMX585_REG_SHR, shr);
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		{
			int gain = ctrl->val;

			// Use HCG mode when gain is over the HGC level
			// This can only be done when HDR is disabled
			bool useHGC = false;
			if (!mode->hdr && gain >= IMX585_ANA_GAIN_HCG_THRESHOLD) {
				useHGC = true;
				gain -= IMX585_ANA_GAIN_HCG_LEVEL;
				if ( gain < IMX585_ANA_GAIN_HCG_MIN )
					gain = IMX585_ANA_GAIN_HCG_MIN;
			}
			dev_info(&client->dev,"V4L2_CID_ANALOGUE_GAIN: %d, HGC: %d\n",gain, (int)useHGC);

			// Apply gain
			imx585_register_hold(imx585, true);
			ret = imx585_write_reg_2byte(imx585, IMX585_REG_ANALOG_GAIN, gain);
			if (ret)
				dev_err_ratelimited(&client->dev, "Failed to write reg 0x%4.4x. error = %d\n", IMX585_REG_ANALOG_GAIN, ret);
			
			// Set HGC/LCG channel			
			ret = imx585_write_reg_1byte(imx585, IMX585_REG_FDG_SEL0, (u16)(useHGC ? 0x01 : 0x00));
			imx585_register_hold(imx585, false);
		}
		break;
	case V4L2_CID_VBLANK:
		{
			dev_info(&client->dev,"V4L2_CID_VBLANK : %d\n",ctrl->val);
			imx585->VMAX = ((u64)mode->height + ctrl->val);
			dev_info(&client->dev,"\tVMAX : %d\n",imx585 -> VMAX);
			ret = imx585_write_reg_3byte(imx585, IMX585_REG_VMAX, imx585 -> VMAX);
		}
		break;
	case V4L2_CID_HBLANK:
		{
			u64 pixel_rate;
			u64 hmax;
			dev_info(&client->dev,"V4L2_CID_HBLANK : %d\n",ctrl->val);
			pixel_rate = (u64)mode->width * IMX585_PIXEL_RATE;
			do_div(pixel_rate,mode->min_HMAX);
			hmax = (u64)(mode->width + ctrl->val) * IMX585_PIXEL_RATE;
			do_div(hmax,pixel_rate);
			imx585 -> HMAX = hmax;
			dev_info(&client->dev,"\tHMAX : %d\n",imx585 -> HMAX);
			ret = imx585_write_reg_2byte(imx585, IMX585_REG_HMAX, hmax);
		}
		break;
    case V4L2_CID_HFLIP:
		ret = imx585_write_reg_1byte(imx585, IMX585_FLIP_WINMODEH, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = imx585_write_reg_1byte(imx585, IMX585_FLIP_WINMODEV, ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		//ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx585_ctrl_ops = {
	.s_ctrl = imx585_set_ctrl,
};

static int imx585_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx585 *imx585 = to_imx585(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
        if(imx585->mono){
            if (code->index >= (ARRAY_SIZE(mono_codes)))
                return -EINVAL;

            code->code = imx585_get_format_code(imx585,
                                mono_codes[code->index]);
        }
        else{
            if (code->index >= (ARRAY_SIZE(codes) / 4))
                return -EINVAL;

            code->code = imx585_get_format_code(imx585,
                                codes[code->index * 4]);
        }

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

		get_mode_table(imx585, fse->code, V4L2_XFER_FUNC_DEFAULT, &mode_list, &num_modes);

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
	fmt->xfer_func = mode->linear ? V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace) : V4L2_XFER_FUNC_GRADATION_COMPRESSION;
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
			v4l2_subdev_get_try_format(&imx585->sd, sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				imx585_get_format_code(imx585, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx585_update_image_pad_format(imx585, imx585->mode,
						       fmt);
			fmt->format.code =
			       imx585_get_format_code(imx585, imx585->fmt_code);
		} else {
			imx585_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx585->mutex);
	return 0;
}

/* TODO */
static void imx585_set_framing_limits(struct imx585 *imx585)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	const struct imx585_mode *mode = imx585->mode;
	u64 def_hblank;
	u64 pixel_rate;


	imx585->VMAX = mode->default_VMAX;
	imx585->HMAX = mode->default_HMAX;

	pixel_rate = (u64)mode->width * IMX585_PIXEL_RATE;
	do_div(pixel_rate,mode->min_HMAX);
	dev_info(&client->dev,"Pixel Rate : %lld\n",pixel_rate);


	//int def_hblank = mode->default_HMAX * IMX585_PIXEL_RATE / 72000000 - IMX585_NATIVE_WIDTH;
	def_hblank = mode->default_HMAX * pixel_rate;
	do_div(def_hblank,IMX585_PIXEL_RATE);
	def_hblank = def_hblank - mode->width;
	__v4l2_ctrl_modify_range(imx585->hblank, 0,
				 IMX585_HMAX_MAX, 1, def_hblank);


	__v4l2_ctrl_s_ctrl(imx585->hblank, def_hblank);



	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx585->vblank, mode->min_VMAX - mode->height,
				 IMX585_VMAX_MAX - mode->height,
				 1, mode->default_VMAX - mode->height);
	__v4l2_ctrl_s_ctrl(imx585->vblank, mode->default_VMAX - mode->height);

	/* Setting this will adjust the exposure limits as well. */

	__v4l2_ctrl_modify_range(imx585->pixel_rate, pixel_rate, pixel_rate, 1, pixel_rate);

	dev_info(&client->dev,"Setting default HBLANK : %lld, VBLANK : %lld with PixelRate: %lld\n",def_hblank,mode->default_VMAX - mode->height, pixel_rate);
}

/* TODO */
static int imx585_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx585_mode *mode;
	struct imx585 *imx585 = to_imx585(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);

	dev_info(&client->dev,"xfer_func: %d\n", (int)fmt->format.xfer_func);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx585->mutex);

	if (fmt->pad == IMAGE_PAD) {
		const struct imx585_mode *mode_list;
		unsigned int num_modes;

		/* Bayer order varies with flips */
		fmt->format.code = imx585_get_format_code(imx585,
							  fmt->format.code);

		get_mode_table(imx585, fmt->format.code, fmt->format.xfer_func, &mode_list, &num_modes);

		mode = v4l2_find_nearest_size(mode_list,
					      num_modes,
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		imx585_update_image_pad_format(imx585, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else if (imx585->mode != mode) {
			imx585->mode = mode;
			imx585->fmt_code = fmt->format.code;
			imx585_set_framing_limits(imx585);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx585_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx585->mutex);

	return 0;
}

/* TODO */
static const struct v4l2_rect *
__imx585_get_pad_crop(struct imx585 *imx585,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx585->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx585->mode->crop;
	}

	return NULL;
}

/* Start streaming */
static int imx585_start_streaming(struct imx585 *imx585)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	const struct IMX585_reg_list *reg_list;
	int ret;
	
	dev_info(&client->dev,"imx585_start_streaming\n");

	if (!imx585->common_regs_written) {
		ret = imx585_write_regs(imx585, mode_common_regs, ARRAY_SIZE(mode_common_regs));
		if (ret) {
			dev_err(&client->dev, "%s failed to set common settings\n", __func__);
			return ret;
		}
		imx585_write_reg_2byte(imx585, IMX585_REG_BLKLEVEL, IMX585_BLKLEVEL_DEFAULT);
		imx585->common_regs_written = true;
		dev_info(&client->dev,"common_regs_written\n");
	}

	/* Apply default values of current mode */
	reg_list = &imx585->mode->reg_list;
	ret = imx585_write_regs(imx585, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply gradation compression curve for non-linear mode */
	if ( !imx585->mode->linear ) {
		imx585_write_reg_3byte(imx585, IMX585_REG_CCMP1_EXP, 500);
		imx585_write_reg_1byte(imx585, IMX585_REG_ACMP1_EXP, 0x2);
		imx585_write_reg_3byte(imx585, IMX585_REG_CCMP2_EXP, 11500);
		imx585_write_reg_1byte(imx585, IMX585_REG_ACMP2_EXP, 0x6);
	} else {
		imx585_write_reg_3byte(imx585, IMX585_REG_CCMP1_EXP, 0);
		imx585_write_reg_1byte(imx585, IMX585_REG_ACMP1_EXP, 0);
		imx585_write_reg_3byte(imx585, IMX585_REG_CCMP2_EXP, 0);
		imx585_write_reg_1byte(imx585, IMX585_REG_ACMP2_EXP, 0);
	}
	
	/* Apply HDR combining options */
	if ( imx585->mode->hdr ) {
		imx585_write_reg_2byte(imx585, IMX585_REG_EXP_TH_H, 4095);
		imx585_write_reg_2byte(imx585, IMX585_REG_EXP_TH_L, 512);
		imx585_write_reg_1byte(imx585, IMX585_REG_EXP_BK, 0);
	}
	
	/* Disable digital clamp */
	imx585_write_reg_1byte(imx585, IMX585_REG_DIGITAL_CLAMP, 0);
	
	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx585->sd.ctrl_handler);
	if(ret) {
		dev_err(&client->dev, "%s failed to apply user values\n", __func__);
		return ret;
	}

	/* Set stream on register */
	ret = imx585_write_reg_1byte(imx585, IMX585_REG_MODE_SELECT, IMX585_MODE_STREAMING);
	usleep_range(IMX585_STREAM_DELAY_US, IMX585_STREAM_DELAY_US + IMX585_STREAM_DELAY_RANGE_US);
	return ret;
}

/* Stop streaming */
static void imx585_stop_streaming(struct imx585 *imx585)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	int ret;
	
	dev_info(&client->dev,"imx585_stop_streaming\n");

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

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx585->vflip, enable);

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
static int imx585_identify_module(struct imx585 *imx585, u32 expected_id)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx585->sd);
	int ret;
	u32 val;

	ret = imx585_read_reg(imx585, IMX585_REG_CHIP_ID,
			      1, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x, with error %d\n",
			expected_id, ret);
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
		sel->r = *__imx585_get_pad_crop(imx585, sd_state, sel->pad,
						sel->which);
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
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
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
    if (imx585->link_freq){
        imx585->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
    }

	imx585->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xfffff, 1, 0);
	imx585->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	imx585->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX585_EXPOSURE_MIN,
					     IMX585_EXPOSURE_MAX,
					     IMX585_EXPOSURE_STEP,
					     IMX585_EXPOSURE_DEFAULT);
/*
	v4l2_ctrl_new_std(&imx585->ctrls, &imx585_ctrl_ops,
                          V4L2_CID_ANALOGUE_GAIN, 0, 240, 1, 0);
*/
	v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX585_ANA_GAIN_MIN, IMX585_ANA_GAIN_MAX,
			  IMX585_ANA_GAIN_STEP, IMX585_ANA_GAIN_DEFAULT);


    imx585->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);

    imx585->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

/*
	if (imx585->vflip)
		imx585->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
*/

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx585_ctrl_ops,
					      &props);
	if (ret)
		goto error;

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
	.chip_id = IMX585_CHIP_ID,
	.extra_regs = {
		.num_of_regs = 0,
		.regs = NULL
	}
};

static const struct of_device_id imx585_dt_ids[] = {
	{ .compatible = "sony,imx585", .data = &imx585_compatible },
	{ /* sentinel */ }
};


//from imx477.c
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
    dev_info(dev, "Data lanes: %d\n",imx585->lane_count);

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

    dev_info(dev, "Link Speed: %lld Mhz\n",ep_cfg.link_frequencies[0]);

    supported_modes_12bit[0].min_HMAX = HMAX_table_4lane_4K[imx585->link_freq_idx];
    supported_modes_12bit[0].default_HMAX = HMAX_table_4lane_4K[imx585->link_freq_idx];
    supported_modes_12bit[1].min_HMAX = HMAX_table_4lane_4K[imx585->link_freq_idx] * 2;
    supported_modes_12bit[1].default_HMAX = HMAX_table_4lane_4K[imx585->link_freq_idx] * 2;
    supported_modes_16bit[0].min_HMAX = HMAX_table_4lane_4K[imx585->link_freq_idx];
    supported_modes_16bit[0].default_HMAX = HMAX_table_4lane_4K[imx585->link_freq_idx];
    supported_modes_16bit[1].min_HMAX = HMAX_table_4lane_4K[imx585->link_freq_idx] * 2;
    supported_modes_16bit[1].default_HMAX = HMAX_table_4lane_4K[imx585->link_freq_idx] * 2;

    if (imx585->lane_count == 2){
        supported_modes_12bit[0].min_HMAX = supported_modes_12bit[0].min_HMAX * 2;
        supported_modes_12bit[0].default_HMAX = supported_modes_12bit[0].default_HMAX * 2;
        supported_modes_12bit[1].min_HMAX = supported_modes_12bit[1].min_HMAX * 4;
        supported_modes_12bit[1].default_HMAX = supported_modes_12bit[1].default_HMAX * 4;
        supported_modes_16bit[0].min_HMAX = supported_modes_16bit[0].min_HMAX * 2;
        supported_modes_16bit[0].default_HMAX = supported_modes_16bit[0].default_HMAX * 2;
        supported_modes_16bit[1].min_HMAX = supported_modes_16bit[1].min_HMAX * 4;
        supported_modes_16bit[1].default_HMAX = supported_modes_16bit[1].default_HMAX * 4;
    }

    //Update common registers for Lane / Link Speed settings
    for(i=0;i<ARRAY_SIZE(mode_common_regs);i++){
        if(mode_common_regs[i].address == 0x3040){
            mode_common_regs[i].val = (imx585->lane_count == 2) ? 0x01:0x03;
        }
        if(mode_common_regs[i].address == 0x3015){
            mode_common_regs[i].val =  link_freqs_reg_value[imx585->link_freq_idx];
        }
    }
    ret = 0;

error_out:
    v4l2_fwnode_endpoint_free(&ep_cfg);
    fwnode_handle_put(endpoint);

    return ret;
}


static int imx585_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx585 *imx585;
	const struct of_device_id *match;
	int ret;
    u32 mono;

	imx585 = devm_kzalloc(&client->dev, sizeof(*imx585), GFP_KERNEL);
	if (!imx585)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx585->sd, client, &imx585_subdev_ops);

	match = of_match_device(imx585_dt_ids, dev);
	if (!match)
		return -ENODEV;
	imx585->compatible_data =
		(const struct imx585_compatible_data *)match->data;

    /* Default the mono mode from OF to -1, which means invalid */
    ret = of_property_read_u32(dev->of_node, "mono-mode", &mono);
    imx585->mono = (ret == 0);
    dev_info(dev, "Mono: %d\n", imx585->mono);

    /* Check the hardware configuration in device tree */
    if (imx585_check_hwcfg(dev, imx585)){
        return -EINVAL;
    }

	/* Get system clock (xclk) */
	imx585->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx585->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx585->xclk);
	}

	imx585->xclk_freq = clk_get_rate(imx585->xclk);
	if (imx585->xclk_freq != IMX585_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx585->xclk_freq);
		return -EINVAL;
	}

	ret = imx585_get_regulators(imx585);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx585->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	
	/*
	 * The sensor must be powered for imx585_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx585_power_on(dev);
	if (ret)
		return ret;

	ret = imx585_identify_module(imx585, imx585->compatible_data->chip_id);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx585_set_default_format(imx585);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx585_init_controls(imx585);
	if (ret)
		goto error_power_off;

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

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
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
		.of_match_table	= imx585_dt_ids,
		.pm = &imx585_pm_ops,
	},
	.probe = imx585_probe,
	.remove = imx585_remove,
};

module_i2c_driver(imx585_i2c_driver);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_AUTHOR("Tetsuya NOMURA <tetsuya.nomura@soho-enterprise.com>");
MODULE_AUTHOR("Russell Newman <russellnewman@octopuscinema.com>");
MODULE_DESCRIPTION("Sony imx585 sensor driver");
MODULE_LICENSE("GPL v2");