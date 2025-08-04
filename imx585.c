// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony imx585 cameras.
 *
 * Based on Sony imx477 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 * Modified by Will WHANG
 * Modified by sohonomura2020 in Soho Enterprise Ltd.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#define V4L2_CID_IMX585_HDR_DATASEL_TH   (V4L2_CID_USER_ASPEED_BASE + 0)
#define V4L2_CID_IMX585_HDR_DATASEL_BK   (V4L2_CID_USER_ASPEED_BASE + 1)
#define V4L2_CID_IMX585_HDR_GRAD_TH      (V4L2_CID_USER_ASPEED_BASE + 2)
#define V4L2_CID_IMX585_HDR_GRAD_COMP_L  (V4L2_CID_USER_ASPEED_BASE + 3)
#define V4L2_CID_IMX585_HDR_GRAD_COMP_H  (V4L2_CID_USER_ASPEED_BASE + 4)
#define V4L2_CID_IMX585_HDR_GAIN         (V4L2_CID_USER_ASPEED_BASE + 5)
#define V4L2_CID_IMX585_HCG_GAIN         (V4L2_CID_USER_ASPEED_BASE + 6)

/* Standby or streaming mode */
#define IMX585_REG_MODE_SELECT          CCI_REG8(0x3000)
#define IMX585_MODE_STANDBY             0x01
#define IMX585_MODE_STREAMING           0x00
#define IMX585_STREAM_DELAY_US          25000
#define IMX585_STREAM_DELAY_RANGE_US    1000

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby)
 */
#define IMX585_XCLR_MIN_DELAY_US    500000
#define IMX585_XCLR_DELAY_RANGE_US  1000

/* Leader mode and XVS/XHS direction */
#define IMX585_REG_XMSTA      CCI_REG8(0x3002)
#define IMX585_REG_XXS_DRV    CCI_REG8(0x30a6)
#define IMX585_REG_EXTMODE    CCI_REG8(0x30ce)
#define IMX585_REG_XXS_OUTSEL CCI_REG8(0x30a4)

/* XVS pulse length, 2^n H with n=0~3 */
#define IMX585_REG_XVSLNG    CCI_REG8(0x30cc)
/* XHS pulse length, 16*(2^n) Clock with n=0~3 */
#define IMX585_REG_XHSLNG    CCI_REG8(0x30cd)

/* Clk selection */
#define IMX585_INCK_SEL                 CCI_REG8(0x3014)

/* Link Speed */
#define IMX585_DATARATE_SEL             CCI_REG8(0x3015)

/* BIN mode */
/* 2x2 Bin mode selection, 0x01 => Mono, 0x00 => Color */
#define IMX585_BIN_MODE                 CCI_REG8(0x3019)

/* Lane Count */
#define IMX585_LANEMODE                 CCI_REG8(0x3040)

/* VMAX internal VBLANK*/
#define IMX585_REG_VMAX                 CCI_REG24_LE(0x3028)
#define IMX585_VMAX_MAX                 0xfffff
#define IMX585_VMAX_DEFAULT             2250

/* HMAX internal HBLANK*/
#define IMX585_REG_HMAX                 CCI_REG16_LE(0x302c)
#define IMX585_HMAX_MAX                 0xffff

/* SHR internal */
#define IMX585_REG_SHR                  CCI_REG24_LE(0x3050)
#define IMX585_SHR_MIN                  8
#define IMX585_SHR_MIN_HDR              10
#define IMX585_SHR_MAX                  0xfffff

/* Exposure control */
#define IMX585_EXPOSURE_MIN             2
#define IMX585_EXPOSURE_STEP            1
#define IMX585_EXPOSURE_DEFAULT         1000
#define IMX585_EXPOSURE_MAX             49865

/* HDR threshold */
#define IMX585_REG_EXP_TH_H             CCI_REG16_LE(0x36d0)
#define IMX585_REG_EXP_TH_L             CCI_REG16_LE(0x36d4)
#define IMX585_REG_EXP_BK               CCI_REG8(0x36e2)

/* Gradation compression control */
#define IMX595_REG_CCMP_EN              CCI_REG8(0x36ef)
#define IMX585_REG_CCMP1_EXP            CCI_REG24_LE(0x36e8)
#define IMX585_REG_CCMP2_EXP            CCI_REG24_LE(0x36e4)
#define IMX585_REG_ACMP1_EXP            CCI_REG8(0x36ee)
#define IMX585_REG_ACMP2_EXP            CCI_REG8(0x36ec)

/* HDR Gain Adder */
#define IMX585_REG_EXP_GAIN             CCI_REG8(0x3081)

/* Black level control */
#define IMX585_REG_BLKLEVEL             CCI_REG16_LE(0x30dc)
#define IMX585_BLKLEVEL_DEFAULT         50

/* Digital Clamp */
#define IMX585_REG_DIGITAL_CLAMP        CCI_REG8(0x3458)

/* Analog gain control */
#define IMX585_REG_ANALOG_GAIN          CCI_REG16_LE(0x306c)
#define IMX585_REG_FDG_SEL0             CCI_REG8(0x3030)
#define IMX585_ANA_GAIN_MIN_NORMAL      0
#define IMX585_ANA_GAIN_MIN_HCG         34
#define IMX585_ANA_GAIN_MAX_HDR         80
#define IMX585_ANA_GAIN_MAX_NORMAL      240
#define IMX585_ANA_GAIN_STEP            1
#define IMX585_ANA_GAIN_DEFAULT         0

/* Flip */
#define IMX585_FLIP_WINMODEH            CCI_REG8(0x3020)
#define IMX585_FLIP_WINMODEV            CCI_REG8(0x3021)

/* Pixel Rate */
#define IMX585_PIXEL_RATE               74250000

/* imx585 native and active pixel array size. */
#define IMX585_NATIVE_WIDTH         3856U
#define IMX585_NATIVE_HEIGHT        2180U
#define IMX585_PIXEL_ARRAY_LEFT     8U
#define IMX585_PIXEL_ARRAY_TOP      8U
#define IMX585_PIXEL_ARRAY_WIDTH    3840U
#define IMX585_PIXEL_ARRAY_HEIGHT   2160U

/* Link frequency setup */
enum {
	IMX585_LINK_FREQ_297MHZ,  // 594Mbps/lane
	IMX585_LINK_FREQ_360MHZ,  // 720Mbps/lane
	IMX585_LINK_FREQ_445MHZ,  // 891Mbps/lane
	IMX585_LINK_FREQ_594MHZ,  // 1188Mbps/lane
	IMX585_LINK_FREQ_720MHZ,  // 1440Mbps/lane
	IMX585_LINK_FREQ_891MHZ,  // 1782Mbps/lane
	IMX585_LINK_FREQ_1039MHZ, // 2079Mbps/lane
	IMX585_LINK_FREQ_1188MHZ, // 2376Mbps/lane
};

static const u8 link_freqs_reg_value[] = {
	[IMX585_LINK_FREQ_297MHZ]  = 0x07,
	[IMX585_LINK_FREQ_360MHZ]  = 0x06,
	[IMX585_LINK_FREQ_445MHZ]  = 0x05,
	[IMX585_LINK_FREQ_594MHZ]  = 0x04,
	[IMX585_LINK_FREQ_720MHZ]  = 0x03,
	[IMX585_LINK_FREQ_891MHZ]  = 0x02,
	[IMX585_LINK_FREQ_1039MHZ] = 0x01,
	[IMX585_LINK_FREQ_1188MHZ] = 0x00,
};

static const u64 link_freqs[] = {
	[IMX585_LINK_FREQ_297MHZ]  = 297000000,
	[IMX585_LINK_FREQ_360MHZ]  = 360000000,
	[IMX585_LINK_FREQ_445MHZ]  = 445500000,
	[IMX585_LINK_FREQ_594MHZ]  = 594000000,
	[IMX585_LINK_FREQ_720MHZ]  = 720000000,
	[IMX585_LINK_FREQ_891MHZ]  = 891000000,
	[IMX585_LINK_FREQ_1039MHZ] = 1039500000,
	[IMX585_LINK_FREQ_1188MHZ] = 1188000000,
};

/* min HMAX for 4-lane 4K full res mode, x2 for 2-lane, /2 for FHD */
static const u16 HMAX_table_4lane_4K[] = {
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
	u32 xclk_hz;   /* platform clock rate  */
	u8  inck_sel;  /* value for reg        */
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

/*
 * Honestly I don't know why there are two 50% 50% blend
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

enum {
	SYNC_INT_LEADER,
	SYNC_INT_FOLLOWER,
	SYNC_EXTERNAL,
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

struct imx585_reg_list {
	unsigned int num_of_regs;
	const struct cci_reg_sequence *regs;
};

/* Mode : resolution and related config&values */
struct imx585_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* mode HMAX Scaling */
	u8   hmax_div;

	/* minimum H-timing */
	u16 min_hmax;

	/* minimum V-timing */
	u64 min_vmax;

	/* default H-timing */
	u16 default_hmax;

	/* default V-timing */
	u64 default_vmax;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct imx585_reg_list reg_list;
};

/* IMX585 Register List */
/* Common Modes */
static struct cci_reg_sequence common_regs[] = {
	{ CCI_REG8(0x3002), 0x01 },
	{ CCI_REG8(0x3069), 0x00 },
	{ CCI_REG8(0x3074), 0x64 },
	{ CCI_REG8(0x30D5), 0x04 },// DIG_CLP_VSTART
	{ CCI_REG8(0x3030), 0x00 },// FDG_SEL0 LCG, HCG:0x01
	{ CCI_REG8(0x30A6), 0x00 },// XVS_DRV [1:0] Hi-Z
	{ CCI_REG8(0x3081), 0x00 },// EXP_GAIN, Reset to 0
	{ CCI_REG8(0x303A), 0x03 },// Disable Embeeded Data
	{ CCI_REG8(0x3460), 0x21 },// -
	{ CCI_REG8(0x3478), 0xA1 },// -
	{ CCI_REG8(0x347C), 0x01 },// -
	{ CCI_REG8(0x3480), 0x01 },// -
	{ CCI_REG8(0x3A4E), 0x14 },// -
	{ CCI_REG8(0x3A52), 0x14 },// -
	{ CCI_REG8(0x3A56), 0x00 },// -
	{ CCI_REG8(0x3A5A), 0x00 },// -
	{ CCI_REG8(0x3A5E), 0x00 },// -
	{ CCI_REG8(0x3A62), 0x00 },// -
	{ CCI_REG8(0x3A6A), 0x20 },// -
	{ CCI_REG8(0x3A6C), 0x42 },// -
	{ CCI_REG8(0x3A6E), 0xA0 },// -
	{ CCI_REG8(0x3B2C), 0x0C },// -
	{ CCI_REG8(0x3B30), 0x1C },// -
	{ CCI_REG8(0x3B34), 0x0C },// -
	{ CCI_REG8(0x3B38), 0x1C },// -
	{ CCI_REG8(0x3BA0), 0x0C },// -
	{ CCI_REG8(0x3BA4), 0x1C },// -
	{ CCI_REG8(0x3BA8), 0x0C },// -
	{ CCI_REG8(0x3BAC), 0x1C },// -
	{ CCI_REG8(0x3D3C), 0x11 },// -
	{ CCI_REG8(0x3D46), 0x0B },// -
	{ CCI_REG8(0x3DE0), 0x3F },// -
	{ CCI_REG8(0x3DE1), 0x08 },// -
	{ CCI_REG8(0x3E14), 0x87 },// -
	{ CCI_REG8(0x3E16), 0x91 },// -
	{ CCI_REG8(0x3E18), 0x91 },// -
	{ CCI_REG8(0x3E1A), 0x87 },// -
	{ CCI_REG8(0x3E1C), 0x78 },// -
	{ CCI_REG8(0x3E1E), 0x50 },// -
	{ CCI_REG8(0x3E20), 0x50 },// -
	{ CCI_REG8(0x3E22), 0x50 },// -
	{ CCI_REG8(0x3E24), 0x87 },// -
	{ CCI_REG8(0x3E26), 0x91 },// -
	{ CCI_REG8(0x3E28), 0x91 },// -
	{ CCI_REG8(0x3E2A), 0x87 },// -
	{ CCI_REG8(0x3E2C), 0x78 },// -
	{ CCI_REG8(0x3E2E), 0x50 },// -
	{ CCI_REG8(0x3E30), 0x50 },// -
	{ CCI_REG8(0x3E32), 0x50 },// -
	{ CCI_REG8(0x3E34), 0x87 },// -
	{ CCI_REG8(0x3E36), 0x91 },// -
	{ CCI_REG8(0x3E38), 0x91 },// -
	{ CCI_REG8(0x3E3A), 0x87 },// -
	{ CCI_REG8(0x3E3C), 0x78 },// -
	{ CCI_REG8(0x3E3E), 0x50 },// -
	{ CCI_REG8(0x3E40), 0x50 },// -
	{ CCI_REG8(0x3E42), 0x50 },// -
	{ CCI_REG8(0x4054), 0x64 },// -
	{ CCI_REG8(0x4148), 0xFE },// -
	{ CCI_REG8(0x4149), 0x05 },// -
	{ CCI_REG8(0x414A), 0xFF },// -
	{ CCI_REG8(0x414B), 0x05 },// -
	{ CCI_REG8(0x420A), 0x03 },// -
	{ CCI_REG8(0x4231), 0x08 },// -
	{ CCI_REG8(0x423D), 0x9C },// -
	{ CCI_REG8(0x4242), 0xB4 },// -
	{ CCI_REG8(0x4246), 0xB4 },// -
	{ CCI_REG8(0x424E), 0xB4 },// -
	{ CCI_REG8(0x425C), 0xB4 },// -
	{ CCI_REG8(0x425E), 0xB6 },// -
	{ CCI_REG8(0x426C), 0xB4 },// -
	{ CCI_REG8(0x426E), 0xB6 },// -
	{ CCI_REG8(0x428C), 0xB4 },// -
	{ CCI_REG8(0x428E), 0xB6 },// -
	{ CCI_REG8(0x4708), 0x00 },// -
	{ CCI_REG8(0x4709), 0x00 },// -
	{ CCI_REG8(0x470A), 0xFF },// -
	{ CCI_REG8(0x470B), 0x03 },// -
	{ CCI_REG8(0x470C), 0x00 },// -
	{ CCI_REG8(0x470D), 0x00 },// -
	{ CCI_REG8(0x470E), 0xFF },// -
	{ CCI_REG8(0x470F), 0x03 },// -
	{ CCI_REG8(0x47EB), 0x1C },// -
	{ CCI_REG8(0x47F0), 0xA6 },// -
	{ CCI_REG8(0x47F2), 0xA6 },// -
	{ CCI_REG8(0x47F4), 0xA0 },// -
	{ CCI_REG8(0x47F6), 0x96 },// -
	{ CCI_REG8(0x4808), 0xA6 },// -
	{ CCI_REG8(0x480A), 0xA6 },// -
	{ CCI_REG8(0x480C), 0xA0 },// -
	{ CCI_REG8(0x480E), 0x96 },// -
	{ CCI_REG8(0x492C), 0xB2 },// -
	{ CCI_REG8(0x4930), 0x03 },// -
	{ CCI_REG8(0x4932), 0x03 },// -
	{ CCI_REG8(0x4936), 0x5B },// -
	{ CCI_REG8(0x4938), 0x82 },// -
	{ CCI_REG8(0x493E), 0x23 },// -
	{ CCI_REG8(0x4BA8), 0x1C },// -
	{ CCI_REG8(0x4BA9), 0x03 },// -
	{ CCI_REG8(0x4BAC), 0x1C },// -
	{ CCI_REG8(0x4BAD), 0x1C },// -
	{ CCI_REG8(0x4BAE), 0x1C },// -
	{ CCI_REG8(0x4BAF), 0x1C },// -
	{ CCI_REG8(0x4BB0), 0x1C },// -
	{ CCI_REG8(0x4BB1), 0x1C },// -
	{ CCI_REG8(0x4BB2), 0x1C },// -
	{ CCI_REG8(0x4BB3), 0x1C },// -
	{ CCI_REG8(0x4BB4), 0x1C },// -
	{ CCI_REG8(0x4BB8), 0x03 },// -
	{ CCI_REG8(0x4BB9), 0x03 },// -
	{ CCI_REG8(0x4BBA), 0x03 },// -
	{ CCI_REG8(0x4BBB), 0x03 },// -
	{ CCI_REG8(0x4BBC), 0x03 },// -
	{ CCI_REG8(0x4BBD), 0x03 },// -
	{ CCI_REG8(0x4BBE), 0x03 },// -
	{ CCI_REG8(0x4BBF), 0x03 },// -
	{ CCI_REG8(0x4BC0), 0x03 },// -
	{ CCI_REG8(0x4C14), 0x87 },// -
	{ CCI_REG8(0x4C16), 0x91 },// -
	{ CCI_REG8(0x4C18), 0x91 },// -
	{ CCI_REG8(0x4C1A), 0x87 },// -
	{ CCI_REG8(0x4C1C), 0x78 },// -
	{ CCI_REG8(0x4C1E), 0x50 },// -
	{ CCI_REG8(0x4C20), 0x50 },// -
	{ CCI_REG8(0x4C22), 0x50 },// -
	{ CCI_REG8(0x4C24), 0x87 },// -
	{ CCI_REG8(0x4C26), 0x91 },// -
	{ CCI_REG8(0x4C28), 0x91 },// -
	{ CCI_REG8(0x4C2A), 0x87 },// -
	{ CCI_REG8(0x4C2C), 0x78 },// -
	{ CCI_REG8(0x4C2E), 0x50 },// -
	{ CCI_REG8(0x4C30), 0x50 },// -
	{ CCI_REG8(0x4C32), 0x50 },// -
	{ CCI_REG8(0x4C34), 0x87 },// -
	{ CCI_REG8(0x4C36), 0x91 },// -
	{ CCI_REG8(0x4C38), 0x91 },// -
	{ CCI_REG8(0x4C3A), 0x87 },// -
	{ CCI_REG8(0x4C3C), 0x78 },// -
	{ CCI_REG8(0x4C3E), 0x50 },// -
	{ CCI_REG8(0x4C40), 0x50 },// -
	{ CCI_REG8(0x4C42), 0x50 },// -
	{ CCI_REG8(0x4D12), 0x1F },// -
	{ CCI_REG8(0x4D13), 0x1E },// -
	{ CCI_REG8(0x4D26), 0x33 },// -
	{ CCI_REG8(0x4E0E), 0x59 },// -
	{ CCI_REG8(0x4E14), 0x55 },// -
	{ CCI_REG8(0x4E16), 0x59 },// -
	{ CCI_REG8(0x4E1E), 0x3B },// -
	{ CCI_REG8(0x4E20), 0x47 },// -
	{ CCI_REG8(0x4E22), 0x54 },// -
	{ CCI_REG8(0x4E26), 0x81 },// -
	{ CCI_REG8(0x4E2C), 0x7D },// -
	{ CCI_REG8(0x4E2E), 0x81 },// -
	{ CCI_REG8(0x4E36), 0x63 },// -
	{ CCI_REG8(0x4E38), 0x6F },// -
	{ CCI_REG8(0x4E3A), 0x7C },// -
	{ CCI_REG8(0x4F3A), 0x3C },// -
	{ CCI_REG8(0x4F3C), 0x46 },// -
	{ CCI_REG8(0x4F3E), 0x59 },// -
	{ CCI_REG8(0x4F42), 0x64 },// -
	{ CCI_REG8(0x4F44), 0x6E },// -
	{ CCI_REG8(0x4F46), 0x81 },// -
	{ CCI_REG8(0x4F4A), 0x82 },// -
	{ CCI_REG8(0x4F5A), 0x81 },// -
	{ CCI_REG8(0x4F62), 0xAA },// -
	{ CCI_REG8(0x4F72), 0xA9 },// -
	{ CCI_REG8(0x4F78), 0x36 },// -
	{ CCI_REG8(0x4F7A), 0x41 },// -
	{ CCI_REG8(0x4F7C), 0x61 },// -
	{ CCI_REG8(0x4F7D), 0x01 },// -
	{ CCI_REG8(0x4F7E), 0x7C },// -
	{ CCI_REG8(0x4F7F), 0x01 },// -
	{ CCI_REG8(0x4F80), 0x77 },// -
	{ CCI_REG8(0x4F82), 0x7B },// -
	{ CCI_REG8(0x4F88), 0x37 },// -
	{ CCI_REG8(0x4F8A), 0x40 },// -
	{ CCI_REG8(0x4F8C), 0x62 },// -
	{ CCI_REG8(0x4F8D), 0x01 },// -
	{ CCI_REG8(0x4F8E), 0x76 },// -
	{ CCI_REG8(0x4F8F), 0x01 },// -
	{ CCI_REG8(0x4F90), 0x5E },// -
	{ CCI_REG8(0x4F91), 0x02 },// -
	{ CCI_REG8(0x4F92), 0x69 },// -
	{ CCI_REG8(0x4F93), 0x02 },// -
	{ CCI_REG8(0x4F94), 0x89 },// -
	{ CCI_REG8(0x4F95), 0x02 },// -
	{ CCI_REG8(0x4F96), 0xA4 },// -
	{ CCI_REG8(0x4F97), 0x02 },// -
	{ CCI_REG8(0x4F98), 0x9F },// -
	{ CCI_REG8(0x4F99), 0x02 },// -
	{ CCI_REG8(0x4F9A), 0xA3 },// -
	{ CCI_REG8(0x4F9B), 0x02 },// -
	{ CCI_REG8(0x4FA0), 0x5F },// -
	{ CCI_REG8(0x4FA1), 0x02 },// -
	{ CCI_REG8(0x4FA2), 0x68 },// -
	{ CCI_REG8(0x4FA3), 0x02 },// -
	{ CCI_REG8(0x4FA4), 0x8A },// -
	{ CCI_REG8(0x4FA5), 0x02 },// -
	{ CCI_REG8(0x4FA6), 0x9E },// -
	{ CCI_REG8(0x4FA7), 0x02 },// -
	{ CCI_REG8(0x519E), 0x79 },// -
	{ CCI_REG8(0x51A6), 0xA1 },// -
	{ CCI_REG8(0x51F0), 0xAC },// -
	{ CCI_REG8(0x51F2), 0xAA },// -
	{ CCI_REG8(0x51F4), 0xA5 },// -
	{ CCI_REG8(0x51F6), 0xA0 },// -
	{ CCI_REG8(0x5200), 0x9B },// -
	{ CCI_REG8(0x5202), 0x91 },// -
	{ CCI_REG8(0x5204), 0x87 },// -
	{ CCI_REG8(0x5206), 0x82 },// -
	{ CCI_REG8(0x5208), 0xAC },// -
	{ CCI_REG8(0x520A), 0xAA },// -
	{ CCI_REG8(0x520C), 0xA5 },// -
	{ CCI_REG8(0x520E), 0xA0 },// -
	{ CCI_REG8(0x5210), 0x9B },// -
	{ CCI_REG8(0x5212), 0x91 },// -
	{ CCI_REG8(0x5214), 0x87 },// -
	{ CCI_REG8(0x5216), 0x82 },// -
	{ CCI_REG8(0x5218), 0xAC },// -
	{ CCI_REG8(0x521A), 0xAA },// -
	{ CCI_REG8(0x521C), 0xA5 },// -
	{ CCI_REG8(0x521E), 0xA0 },// -
	{ CCI_REG8(0x5220), 0x9B },// -
	{ CCI_REG8(0x5222), 0x91 },// -
	{ CCI_REG8(0x5224), 0x87 },// -
	{ CCI_REG8(0x5226), 0x82 },// -
};

/* Common Registers for ClearHDR. */
static const struct cci_reg_sequence common_clearHDR_mode[] = {
	{ CCI_REG8(0x301A), 0x10 }, // WDMODE: Clear HDR mode
	{ CCI_REG8(0x3024), 0x02 }, // COMBI_EN: 0x02
	{ CCI_REG8(0x3069), 0x02 }, // Clear HDR mode
	{ CCI_REG8(0x3074), 0x63 }, // Clear HDR mode
	{ CCI_REG8(0x3930), 0xE6 }, // DUR[15:8]: Clear HDR mode (12bit)
	{ CCI_REG8(0x3931), 0x00 }, // DUR[7:0]: Clear HDR mode (12bit)
	{ CCI_REG8(0x3A4C), 0x61 }, // WAIT_ST0[7:0]: Clear HDR mode
	{ CCI_REG8(0x3A4D), 0x02 }, // WAIT_ST0[15:8]: Clear HDR mode
	{ CCI_REG8(0x3A50), 0x70 }, // WAIT_ST1[7:0]: Clear HDR mode
	{ CCI_REG8(0x3A51), 0x02 }, // WAIT_ST1[15:8]: Clear HDR mode
	{ CCI_REG8(0x3E10), 0x17 }, // ADTHEN: Clear HDR mode
	{ CCI_REG8(0x493C), 0x41 }, // WAIT_10_SHF AD 10-bit 0x0C disable
	{ CCI_REG8(0x4940), 0x41 }, // WAIT_12_SHF AD 12-bit 0x41 enable
	{ CCI_REG8(0x3081), 0x02 }, // EXP_GAIN: High gain setting +12dB default
};

/* Common Registers for non-ClearHDR. */
static const struct cci_reg_sequence common_normal_mode[] = {
	{ CCI_REG8(0x301A), 0x00 }, // WDMODE: Normal mode
	{ CCI_REG8(0x3024), 0x00 }, // COMBI_EN: 0x00
	{ CCI_REG8(0x3069), 0x00 }, // Normal mode
	{ CCI_REG8(0x3074), 0x64 }, // Normal mode
	{ CCI_REG8(0x3930), 0x0C }, // DUR[15:8]: Normal mode (12bit)
	{ CCI_REG8(0x3931), 0x01 }, // DUR[7:0]: Normal mode (12bit)
	{ CCI_REG8(0x3A4C), 0x39 }, // WAIT_ST0[7:0]: Normal mode
	{ CCI_REG8(0x3A4D), 0x01 }, // WAIT_ST0[15:8]: Normal mode
	{ CCI_REG8(0x3A50), 0x48 }, // WAIT_ST1[7:0]: Normal mode
	{ CCI_REG8(0x3A51), 0x01 }, // WAIT_ST1[15:8]: Normal mode
	{ CCI_REG8(0x3E10), 0x10 }, // ADTHEN: Normal mode
	{ CCI_REG8(0x493C), 0x23 }, // WAIT_10_SHF AD 10-bit 0x23 Normal mode
	{ CCI_REG8(0x4940), 0x23 }, // WAIT_12_SHF AD 12-bit 0x23 Normal mode
};

/* All pixel 4K60. 12-bit */
static const struct cci_reg_sequence mode_4k_regs_12bit[] = {
	{ CCI_REG8(0x301B), 0x00 }, // ADDMODE non-binning
	{ CCI_REG8(0x3022), 0x02 }, // ADBIT 12-bit
	{ CCI_REG8(0x3023), 0x01 }, // MDBIT 12-bit
	{ CCI_REG8(0x30D5), 0x04 }, // DIG_CLP_VSTART non-binning
};

/* 2x2 binned 1080p60. 12-bit */
static const struct cci_reg_sequence mode_1080_regs_12bit[] = {
	{ CCI_REG8(0x301B), 0x01 }, // ADDMODE binning
	{ CCI_REG8(0x3022), 0x02 }, // ADBIT 12-bit
	{ CCI_REG8(0x3023), 0x01 }, // MDBIT 12-bit
	{ CCI_REG8(0x30D5), 0x02 }, // DIG_CLP_VSTART binning
};

/* IMX585 Register List - END*/

/* 
 * For Mode List:
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
 *
 * Technically, because the sensor is actually binning
 * in digital domain, its readout speed is the same
 * between 4K and FHD. However, through testing it is
 * possible to "overclock" the FHD mode, thus leaving the
 * hmax_div option for those who want to try.
 * Also, note that FHD and 4K mode shared the same VMAX.
 */

/* Mode configs */
struct imx585_mode supported_modes[] = {
	{
		/* 1080p60 2x2 binning */
		.width = 1928,
		.height = 1090,
		.hmax_div = 1,
		.min_hmax = 366,
		.min_vmax = IMX585_VMAX_DEFAULT,
		.default_hmax = 366,
		.default_vmax = IMX585_VMAX_DEFAULT,
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
		.min_hmax = 550,
		.min_vmax = IMX585_VMAX_DEFAULT,
		.default_hmax = 550,
		.default_vmax = IMX585_VMAX_DEFAULT,
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

/* 12bit + 16bit for Clear HDR */
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
	"vana",  /* Analog (3.3V) supply */
	"vdig",  /* Digital Core (1.1V) supply */
	"vddl",  /* IF (1.8V) supply */
};

#define imx585_NUM_SUPPLIES ARRAY_SIZE(imx585_supply_name)

struct imx585 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct device *clientdev;
	struct regmap *regmap;

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
	struct v4l2_ctrl *hcg_ctrl;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *blacklevel;

	/* V4L2 HDR Controls */
	struct v4l2_ctrl *hdr_mode;
	struct v4l2_ctrl *datasel_th_ctrl;
	struct v4l2_ctrl *datasel_bk_ctrl;
	struct v4l2_ctrl *gdc_th_ctrl;
	struct v4l2_ctrl *gdc_exp_ctrl_l;
	struct v4l2_ctrl *gdc_exp_ctrl_h;
	struct v4l2_ctrl *hdr_gain_ctrl;

	/* V4L2 IR Cut filter switch Controls */
	bool   has_ircut;
	struct v4l2_ctrl   *ircut_ctrl;
	struct i2c_client  *ircut_client;

	/* HCG enabled flag*/
	bool hcg;

	/* Mono mode */
	bool mono;

	/* Clear HDR mode */
	bool clear_hdr;

	/* 
	 * Sync Mode
	 * 0 = Internal Sync Leader Mode
	 * 1 = External Sync Leader Mode
	 * 2 = Follower Mode
	 * The datasheet wording is very confusing but basically:
	 * Leader Mode = Sensor using internal clock to drive the sensor
	 * But with external sync mode you can send a XVS input so the sensor
	 * will try to align with it.
	 * For Follower mode it is purely driven by external clock.
	 * In this case you need to drive both XVS and XHS.
	 */
	u8 sync_mode;

	/* Tracking sensor VMAX/HMAX value */
	u16 hmax;
	u32 vmax;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;
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

	if (imx585->mono) {
		/* --- Mono paths --- */
		if (code == MEDIA_BUS_FMT_Y16_1X16 && imx585->clear_hdr) {
			*mode_list = supported_modes;
			*num_modes = ARRAY_SIZE(supported_modes);
		}
		if (code == MEDIA_BUS_FMT_Y12_1X12) {
			*mode_list = supported_modes;
			*num_modes = ARRAY_SIZE(supported_modes);
		}
	} else {
		/* --- Color paths --- */
		switch (code) {
		/* 16-bit */
		case MEDIA_BUS_FMT_SRGGB16_1X16:
		case MEDIA_BUS_FMT_SGRBG16_1X16:
		case MEDIA_BUS_FMT_SGBRG16_1X16:
		case MEDIA_BUS_FMT_SBGGR16_1X16:
		/* 12-bit */
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
}

/* Optional IR-cut helper */
/* One-byte “command” sent to the IR-cut MCU at imx585->ircut_client */
static int imx585_ircut_write(struct imx585 *imx585, u8 cmd)
{
	struct i2c_client *client = imx585->ircut_client;
	int ret;

	ret = i2c_smbus_write_byte(client, cmd);
	if (ret < 0)
		dev_err(imx585->clientdev, "IR-cut write failed (%d)\n", ret);

	return ret;
}

static int imx585_ircut_set(struct imx585 *imx585, int on)
{
	return imx585_ircut_write(imx585, on ? 0x01 : 0x00);
}

/* Get bayer order based on flip setting. */
static u32 imx585_get_format_code(struct imx585 *imx585, u32 code)
{
	unsigned int i;

	if (imx585->mono) {
		for (i = 0; i < ARRAY_SIZE(mono_codes); i++)
			if (mono_codes[i] == code)
				break;
		return mono_codes[i];
	}

	if (imx585->clear_hdr) {
		for (i = 0; i < ARRAY_SIZE(codes_clearhdr); i++)
			if (codes_clearhdr[i] == code)
				break;
		return codes_clearhdr[i];
	}

	for (i = 0; i < ARRAY_SIZE(codes_normal); i++)
		if (codes_normal[i] == code)
			break;
	return codes_normal[i];
}

/* 
 * For HDR mode, Gain is limited to 0~80 and HCG is disabled
 * For Normal mode, Gain is limited to 0~240
 */
static void imx585_update_gain_limits(struct imx585 *imx585)
{
	bool hcg_on = imx585->hcg;
	bool clear_hdr = imx585->clear_hdr;
	u32 min = hcg_on ? IMX585_ANA_GAIN_MIN_HCG : IMX585_ANA_GAIN_MIN_NORMAL;
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

static void imx585_update_hmax(struct imx585 *imx585)
{
	const u32 base_4lane = HMAX_table_4lane_4K[imx585->link_freq_idx];
	const u32 lane_scale = (imx585->lane_count == 2) ? 2 : 1;
	const u32 factor     = base_4lane * lane_scale;
	const u32 hdr_factor = (imx585->clear_hdr) ? 2 : 1;

	dev_info(imx585->clientdev, "Upadte minimum HMAX\n");
	dev_info(imx585->clientdev, "\tbase_4lane: %d\n", base_4lane);
	dev_info(imx585->clientdev, "\tlane_scale: %d\n", lane_scale);
	dev_info(imx585->clientdev, "\tfactor: %d\n", factor);
	dev_info(imx585->clientdev, "\thdr_factor: %d\n", hdr_factor);

	for (unsigned int i = 0; i < ARRAY_SIZE(supported_modes); ++i) {
		u32 h = factor / supported_modes[i].hmax_div;
		u64 v = IMX585_VMAX_DEFAULT * hdr_factor;

		supported_modes[i].min_hmax     = h;
		supported_modes[i].default_hmax = h;
		supported_modes[i].min_vmax     = v;
		supported_modes[i].default_vmax = v;
		dev_info(imx585->clientdev, "\tvmax: %lld x hmax: %d\n", v, h);
	}
}

static void imx585_set_framing_limits(struct imx585 *imx585)
{
	u64 default_hblank, max_hblank;
	u64 pixel_rate;
	const struct imx585_mode *mode;
	struct v4l2_mbus_framefmt *fmt;
	const struct imx585_mode *mode_list;
	struct v4l2_subdev_state *state;
	unsigned int num_modes;

	state = v4l2_subdev_get_locked_active_state(&imx585->sd);
	if (state == NULL) {
		return;
	}
	fmt = v4l2_subdev_state_get_format(state, 0);
	if (!fmt->code) {
		return;
	}
	get_mode_table(imx585, fmt->code, &mode_list, &num_modes);
	if (!num_modes) {                       /* shouldn’t happen */
		return;
	}
	mode = v4l2_find_nearest_size(mode_list, num_modes, width, height,
				      fmt->width, fmt->height);

	imx585_update_hmax(imx585);

	dev_info(imx585->clientdev, "imx585_set_framing_limits mode: %d x %d\n", mode->width, mode->height);

	imx585->vmax = mode->default_vmax;
	imx585->hmax = mode->default_hmax;

	pixel_rate = (u64)mode->width * IMX585_PIXEL_RATE;
	do_div(pixel_rate, mode->min_hmax);
	__v4l2_ctrl_modify_range(imx585->pixel_rate, pixel_rate, pixel_rate, 1, pixel_rate);

	//int default_hblank = mode->default_hmax*IMX585_PIXEL_RATE/72000000-IMX585_NATIVE_WIDTH;
	default_hblank = mode->default_hmax * pixel_rate;
	do_div(default_hblank, IMX585_PIXEL_RATE);
	default_hblank = default_hblank - mode->width;

	max_hblank = IMX585_HMAX_MAX * pixel_rate;
	do_div(max_hblank, IMX585_PIXEL_RATE);
	max_hblank = max_hblank - mode->width;

	__v4l2_ctrl_modify_range(imx585->hblank, 0, max_hblank, 1, default_hblank);
	__v4l2_ctrl_s_ctrl(imx585->hblank, default_hblank);

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx585->vblank, mode->min_vmax - mode->height,
				 IMX585_VMAX_MAX - mode->height,
				 1, mode->default_vmax - mode->height);
	__v4l2_ctrl_s_ctrl(imx585->vblank, mode->default_vmax - mode->height);

	__v4l2_ctrl_modify_range(imx585->exposure, IMX585_EXPOSURE_MIN,
				 imx585->vmax - IMX585_SHR_MIN_HDR, 1,
				 IMX585_EXPOSURE_DEFAULT);
	dev_info(imx585->clientdev, "default vmax: %lld x hmax: %d\n", mode->min_vmax, mode->min_hmax);
	dev_info(imx585->clientdev, "Setting default HBLANK : %llu, VBLANK : %llu PixelRate: %lld\n",
		 default_hblank, mode->default_vmax - mode->height, pixel_rate);
}

static int imx585_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx585 *imx585 = container_of(ctrl->handler, struct imx585, ctrl_handler);
	const struct imx585_mode *mode;
	struct v4l2_mbus_framefmt *fmt;
	const struct imx585_mode *mode_list;
	struct v4l2_subdev_state *state;
	unsigned int code, num_modes;

	dev_info(imx585->clientdev,
		 "ctrl(id:0x%x,val:0x%x)\n",
		 ctrl->id, ctrl->val);

	state = v4l2_subdev_get_locked_active_state(&imx585->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);
	get_mode_table(imx585, fmt->code, &mode_list, &num_modes);
	mode = v4l2_find_nearest_size(mode_list, num_modes, width, height,
				      fmt->width, fmt->height);

	int ret = 0;
	/*
	 * Applying V4L2 control value that
	 * doesn't need to be in streaming mode
	 */
	switch (ctrl->id) {
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		if (imx585->clear_hdr != ctrl->val) {
			imx585->clear_hdr = ctrl->val;
			v4l2_ctrl_activate(imx585->datasel_th_ctrl,  imx585->clear_hdr);
			v4l2_ctrl_activate(imx585->datasel_bk_ctrl,  imx585->clear_hdr);
			v4l2_ctrl_activate(imx585->gdc_th_ctrl,      imx585->clear_hdr);
			v4l2_ctrl_activate(imx585->gdc_exp_ctrl_h,   imx585->clear_hdr);
			v4l2_ctrl_activate(imx585->gdc_exp_ctrl_l,   imx585->clear_hdr);
			v4l2_ctrl_activate(imx585->hdr_gain_ctrl,    imx585->clear_hdr);
			v4l2_ctrl_activate(imx585->hcg_ctrl,        !imx585->clear_hdr);
			imx585_update_gain_limits(imx585);
			if (imx585->mono)
				code = imx585_get_format_code(imx585, MEDIA_BUS_FMT_Y12_1X12);
			else
				code = imx585_get_format_code(imx585, MEDIA_BUS_FMT_SRGGB12_1X12);
			get_mode_table(imx585, code, &mode_list, &num_modes);
			mode = v4l2_find_nearest_size(mode_list,
							      num_modes,
							      width, height,
							      fmt->width,
							      fmt->height);
			imx585_set_framing_limits(imx585);
		}
		break;
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (!pm_runtime_get_if_active(imx585->clientdev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		{
		u32 shr;

		shr = (imx585->vmax - ctrl->val)  & ~1u; //Always a multiple of 2
		dev_info(imx585->clientdev, "V4L2_CID_EXPOSURE : %d\n", ctrl->val);
		dev_info(imx585->clientdev, "\tVMAX:%d, HMAX:%d\n", imx585->vmax, imx585->hmax);
		dev_info(imx585->clientdev, "\tSHR:%d\n", shr);

		ret = cci_write(imx585->regmap, IMX585_REG_SHR, shr, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_SHR, error = %d\n",
					    ret);
		break;
		}
	case V4L2_CID_IMX585_HCG_GAIN:
		{
		if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
			break;
		imx585->hcg = ctrl->val;
		imx585_update_gain_limits(imx585);

		// Set HCG/LCG channel
		ret = cci_write(imx585->regmap, IMX585_REG_FDG_SEL0, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_FDG_SEL0, error = %d\n",
					    ret);
		dev_info(imx585->clientdev, "V4L2_CID_HCG_ENABLE: %d\n", ctrl->val);
		break;
		}
	case V4L2_CID_ANALOGUE_GAIN:
		{
		u32 gain = ctrl->val;

		dev_info(imx585->clientdev, "analogue gain = %u (%s)\n",
			 gain, imx585->hcg ? "HCG" : "LCG");

		ret = cci_write(imx585->regmap, IMX585_REG_ANALOG_GAIN, gain, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "ANALOG_GAIN write failed (%d)\n", ret);
		break;
		}
	case V4L2_CID_VBLANK:
		{
		u32 current_exposure = imx585->exposure->cur.val;
		u32 min_shr = (imx585->clear_hdr) ? IMX585_SHR_MIN_HDR : IMX585_SHR_MIN;
		/*
		 * The VBLANK control may change the limits of usable exposure, so check
		 * and adjust if necessary.
		 */
		imx585->vmax = (mode->height + ctrl->val) & ~1u; //Always a multiple of 2

		/* 
		 * New maximum exposure limits,
		 * modifying the range and make sure we are not exceed the new maximum.
		 */
		current_exposure = clamp_t(u32, current_exposure, IMX585_EXPOSURE_MIN,
					   imx585->vmax - min_shr);
		__v4l2_ctrl_modify_range(imx585->exposure, IMX585_EXPOSURE_MIN,
					 imx585->vmax - min_shr, 1,
					 current_exposure);

		dev_info(imx585->clientdev, "V4L2_CID_VBLANK : %d\n", ctrl->val);
		dev_info(imx585->clientdev, "\tVMAX:%d, HMAX:%d\n", imx585->vmax, imx585->hmax);
		dev_info(imx585->clientdev, "Update exposure limits: max:%d, min:%d, current:%d\n",
			 imx585->vmax - min_shr,
			 IMX585_EXPOSURE_MIN, current_exposure);

		ret = cci_write(imx585->regmap, IMX585_REG_VMAX, imx585->vmax, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_VMAX, error = %d\n",
					    ret);
		break;
		}
	case V4L2_CID_HBLANK:
		{
		u64 pixel_rate;
		u64 hmax;

		pixel_rate = (u64)mode->width * IMX585_PIXEL_RATE;
		do_div(pixel_rate, mode->min_hmax);
		hmax = (u64)(mode->width + ctrl->val) * IMX585_PIXEL_RATE;
		do_div(hmax, pixel_rate);
		imx585->hmax = hmax;

		dev_info(imx585->clientdev, "V4L2_CID_HBLANK : %d\n", ctrl->val);
		dev_info(imx585->clientdev, "\tHMAX : %d\n", imx585->hmax);

		ret = cci_write(imx585->regmap, IMX585_REG_HMAX, hmax, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_HMAX, error = %d\n",
					    ret);
		break;
		}
	case V4L2_CID_HFLIP:
		dev_info(imx585->clientdev, "V4L2_CID_HFLIP : %d\n", ctrl->val);
		ret = cci_write(imx585->regmap, IMX585_FLIP_WINMODEH, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_FLIP_WINMODEH, error = %d\n",
					    ret);
		break;
	case V4L2_CID_VFLIP:
		dev_info(imx585->clientdev, "V4L2_CID_VFLIP : %d\n", ctrl->val);
		ret = cci_write(imx585->regmap, IMX585_FLIP_WINMODEV, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_FLIP_WINMODEV, error = %d\n",
					    ret);
		break;
	case V4L2_CID_BRIGHTNESS:
		{
		u16 blacklevel = ctrl->val;

		dev_info(imx585->clientdev, "V4L2_CID_BRIGHTNESS : %d\n", ctrl->val);

		if (blacklevel > 4095)
			blacklevel = 4095;
		ret = cci_write(imx585->regmap, IMX585_REG_BLKLEVEL, blacklevel, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_BLKLEVEL, error = %d\n",
					    ret);
		break;
		}
	case V4L2_CID_BAND_STOP_FILTER:
		if (imx585->has_ircut) {
			dev_info(imx585->clientdev, "V4L2_CID_BAND_STOP_FILTER : %d\n", ctrl->val);
			imx585_ircut_set(imx585, ctrl->val);
		}
		break;
	case V4L2_CID_IMX585_HDR_DATASEL_TH:{
		const u16 *th = (const u16 *)ctrl->p_new.p;

		ret = cci_write(imx585->regmap, IMX585_REG_EXP_TH_H, th[0], NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_EXP_TH_H, error = %d\n",
					    ret);
		ret = cci_write(imx585->regmap, IMX585_REG_EXP_TH_L, th[1], NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_EXP_TH_L, error = %d\n",
					    ret);
		dev_info(imx585->clientdev, "V4L2_CID_IMX585_HDR_DATASEL_TH : %d, %d\n", th[0], th[1]);
		break;
		}
	case V4L2_CID_IMX585_HDR_DATASEL_BK:
		ret = cci_write(imx585->regmap, IMX585_REG_EXP_BK, ctrl->val, NULL);
		dev_info(imx585->clientdev, "V4L2_CID_IMX585_HDR_DATASEL_BK : %d\n", ctrl->val);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_EXP_BK, error = %d\n",
					    ret);
		break;
	case V4L2_CID_IMX585_HDR_GRAD_TH:{
		const u32 *thr = (const u32 *)ctrl->p_new.p;

		ret = cci_write(imx585->regmap, IMX585_REG_CCMP1_EXP, thr[0], NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_CCMP1_EXP, error = %d\n",
					    ret);
		ret = cci_write(imx585->regmap, IMX585_REG_CCMP2_EXP, thr[1], NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_CCMP2_EXP, error = %d\n",
					    ret);
		dev_info(imx585->clientdev, "V4L2_CID_IMX585_HDR_GRAD_TH : %d, %d\n", thr[0], thr[1]);
		break;
		}
	case V4L2_CID_IMX585_HDR_GRAD_COMP_L:{
		ret = cci_write(imx585->regmap, IMX585_REG_ACMP1_EXP, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_ACMP1_EXP, error = %d\n",
					    ret);
		dev_info(imx585->clientdev, "V4L2_CID_IMX585_HDR_GRAD_COMP_L : %d\n", ctrl->val);
		break;
		}
	case V4L2_CID_IMX585_HDR_GRAD_COMP_H:{
		ret = cci_write(imx585->regmap, IMX585_REG_ACMP2_EXP, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_ACMP2_EXP, error = %d\n",
					    ret);
		dev_info(imx585->clientdev, "V4L2_CID_IMX585_HDR_GRAD_COMP_H : %d\n", ctrl->val);
		break;
		}
	case V4L2_CID_IMX585_HDR_GAIN:
		ret = cci_write(imx585->regmap, IMX585_REG_EXP_GAIN, ctrl->val, NULL);
		dev_info(imx585->clientdev, "IMX585_REG_EXP_GAIN : %d\n", ctrl->val);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "Failed to write IMX585_REG_EXP_GAIN, error = %d\n",
					    ret);
		break;
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		/* Already handled above. */
		break;
	default:
		dev_info(imx585->clientdev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(imx585->clientdev);

	return ret;
}

static const struct v4l2_ctrl_ops imx585_ctrl_ops = {
	.s_ctrl = imx585_set_ctrl,
};

static const u16 hdr_thresh_def[2] = { 512, 1024 };
static const struct v4l2_ctrl_config imx585_cfg_datasel_th = {
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

static const struct v4l2_ctrl_config imx585_cfg_datasel_bk = {
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
static const struct v4l2_ctrl_config imx585_cfg_grad_th = {
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

static const struct v4l2_ctrl_config imx585_cfg_grad_exp_l = {
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

static const struct v4l2_ctrl_config imx585_cfg_grad_exp_h = {
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

static const struct v4l2_ctrl_config imx585_cfg_hcg = {
	.ops = &imx585_ctrl_ops,
	.id = V4L2_CID_IMX585_HCG_GAIN,
	.name = "HCG Enable",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min  = 0,
	.max  = 1,
	.step = 1,
	.def  = 0,
};

/* Initialize control handlers */
static int imx585_init_controls(struct imx585 *imx585)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	int ret;

	ctrl_hdlr = &imx585->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 32);
	if (ret)
		return ret;

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx585_set_framing_limits() call below.
	 */
	/* By default, PIXEL_RATE is read only */
	imx585->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       0xfffff,
					       0xfffff, 1,
					       0xfffff);

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
					       V4L2_CID_BRIGHTNESS, 0, 0xffff, 1,
					       IMX585_BLKLEVEL_DEFAULT);

	imx585->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX585_EXPOSURE_MIN,
					     IMX585_EXPOSURE_MAX,
					     IMX585_EXPOSURE_STEP,
					     IMX585_EXPOSURE_DEFAULT);

	imx585->gain = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
					 IMX585_ANA_GAIN_MIN_NORMAL, IMX585_ANA_GAIN_MAX_NORMAL,
					 IMX585_ANA_GAIN_STEP, IMX585_ANA_GAIN_DEFAULT);

	imx585->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	imx585->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (imx585->has_ircut) {
		imx585->ircut_ctrl =
			v4l2_ctrl_new_std(&imx585->ctrl_handler, &imx585_ctrl_ops,
					  V4L2_CID_BAND_STOP_FILTER,
					  0, 1, 1, 1);
	}

	imx585->hdr_mode = v4l2_ctrl_new_std(ctrl_hdlr, &imx585_ctrl_ops,
					     V4L2_CID_WIDE_DYNAMIC_RANGE,
					     0, 1, 1, 0);
	imx585->datasel_th_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
						       &imx585_cfg_datasel_th, NULL);
	imx585->datasel_bk_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
						       &imx585_cfg_datasel_bk, NULL);
	imx585->gdc_th_ctrl     = v4l2_ctrl_new_custom(ctrl_hdlr,
						       &imx585_cfg_grad_th, NULL);
	imx585->gdc_exp_ctrl_l  = v4l2_ctrl_new_custom(ctrl_hdlr,
						       &imx585_cfg_grad_exp_l, NULL);
	imx585->gdc_exp_ctrl_h  = v4l2_ctrl_new_custom(ctrl_hdlr,
						       &imx585_cfg_grad_exp_h, NULL);
	imx585->hdr_gain_ctrl   = v4l2_ctrl_new_custom(ctrl_hdlr,
						       &imx585_cfg_hdr_gain, NULL);
	imx585->hcg_ctrl        = v4l2_ctrl_new_custom(ctrl_hdlr,
						       &imx585_cfg_hcg, NULL);

	v4l2_ctrl_activate(imx585->datasel_th_ctrl,  imx585->clear_hdr);
	v4l2_ctrl_activate(imx585->datasel_bk_ctrl,  imx585->clear_hdr);
	v4l2_ctrl_activate(imx585->gdc_th_ctrl,      imx585->clear_hdr);
	v4l2_ctrl_activate(imx585->gdc_exp_ctrl_l,   imx585->clear_hdr);
	v4l2_ctrl_activate(imx585->gdc_exp_ctrl_h,   imx585->clear_hdr);
	v4l2_ctrl_activate(imx585->hdr_gain_ctrl,    imx585->clear_hdr);
	v4l2_ctrl_activate(imx585->hcg_ctrl,        !imx585->clear_hdr);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(imx585->clientdev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(imx585->clientdev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx585_ctrl_ops, &props);
	if (ret)
		goto error;

	memcpy(imx585->datasel_th_ctrl->p_cur.p, hdr_thresh_def, sizeof(hdr_thresh_def));
	memcpy(imx585->datasel_th_ctrl->p_new.p, hdr_thresh_def, sizeof(hdr_thresh_def));
	memcpy(imx585->gdc_th_ctrl->p_cur.p, grad_thresh_def, sizeof(grad_thresh_def));
	memcpy(imx585->gdc_th_ctrl->p_new.p, grad_thresh_def, sizeof(grad_thresh_def));

	imx585->hdr_mode->flags |= V4L2_CTRL_FLAG_UPDATE;
	imx585->hdr_mode->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx585->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static void imx585_free_controls(struct imx585 *imx585)
{
	v4l2_ctrl_handler_free(imx585->sd.ctrl_handler);
}

static int imx585_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx585 *imx585 = to_imx585(sd);
	unsigned int entries;
	const u32 *tbl;

	if (imx585->mono) {
		if (imx585->clear_hdr) {
			if (code->index > 1)
				return -EINVAL;
			code->code = mono_codes[code->index];
			return 0;
		}
		/* HDR off: expose Y12 only */
		if (code->index)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_Y12_1X12;
		return 0;
	}

	if (imx585->clear_hdr) {
		tbl     = codes_clearhdr;  /* << 16bit + 12bit */
		entries = ARRAY_SIZE(codes_clearhdr) / 4;
	} else {
		tbl     = codes_normal;    /* << ONLY 12bit */
		entries = ARRAY_SIZE(codes_normal) / 4;
	}

	if (code->index >= entries)
		return -EINVAL;

	code->code = imx585_get_format_code(imx585, tbl[code->index * 4]);
	return 0;

}

static int imx585_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx585 *imx585 = to_imx585(sd);

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


	return 0;
}

static int imx585_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *format;
	const struct imx585_mode *mode;
	struct imx585 *imx585 = to_imx585(sd);
	const struct imx585_mode *mode_list;
	unsigned int num_modes;

	get_mode_table(imx585, fmt->format.code, &mode_list, &num_modes);
	mode = v4l2_find_nearest_size(mode_list, num_modes, width, height,
				      fmt->format.width, fmt->format.height);

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;

	format = v4l2_subdev_state_get_format(sd_state, 0);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		imx585_set_framing_limits(imx585);
		imx585->fmt_code = fmt->format.code;
	}

	*format = fmt->format;

	return 0;
}

/* Start streaming */
static int imx585_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx585 *imx585 = to_imx585(sd);
	const struct imx585_mode *mode_list, *mode;
	struct v4l2_subdev_state *st;
	struct v4l2_mbus_framefmt *fmt;
	unsigned int n_modes;
	int ret;

	ret = pm_runtime_get_sync(imx585->clientdev);
	if (ret < 0) {
		pm_runtime_put_noidle(imx585->clientdev);
		return ret;
	}

	ret = cci_multi_reg_write(imx585->regmap, common_regs,
				  ARRAY_SIZE(common_regs), NULL);
	if (ret) {
		dev_err(imx585->clientdev, "%s failed to set common settings\n", __func__);
		goto err_rpm_put;
	}

	cci_write(imx585->regmap, IMX585_INCK_SEL, imx585->inck_sel_val, NULL);
	cci_write(imx585->regmap, IMX585_REG_BLKLEVEL, IMX585_BLKLEVEL_DEFAULT, NULL);
	cci_write(imx585->regmap, IMX585_DATARATE_SEL,
			       link_freqs_reg_value[imx585->link_freq_idx], NULL);

	if (imx585->lane_count == 2)
		cci_write(imx585->regmap, IMX585_LANEMODE, 0x01, NULL);
	else
		cci_write(imx585->regmap, IMX585_LANEMODE, 0x03, NULL);

	if (imx585->mono)
		cci_write(imx585->regmap, IMX585_BIN_MODE, 0x01, NULL);
	else
		cci_write(imx585->regmap, IMX585_BIN_MODE, 0x00, NULL);

	if (imx585->sync_mode == SYNC_INT_FOLLOWER) { //External Sync Leader Mode
		dev_info(imx585->clientdev, "Int Sync Follower Mode, enable XVS input\n");
		cci_write(imx585->regmap, IMX585_REG_EXTMODE, 0x01, NULL);
		// Enable XHS output, but XVS is input
		cci_write(imx585->regmap, IMX585_REG_XXS_DRV, 0x03, NULL);
		// Disable XVS OUT
		cci_write(imx585->regmap, IMX585_REG_XXS_OUTSEL, 0x08, NULL);
	} else if (imx585->sync_mode == SYNC_INT_LEADER) {
		dev_info(imx585->clientdev, "Int Sync Leader Mode, enable output\n");
		cci_write(imx585->regmap, IMX585_REG_EXTMODE, 0x00, NULL);
		// Enable XHS and XVS output
		cci_write(imx585->regmap, IMX585_REG_XXS_DRV, 0x00, NULL);
		cci_write(imx585->regmap, IMX585_REG_XXS_OUTSEL, 0x0A, NULL);
	} else {
		dev_info(imx585->clientdev, "Follower Mode, enable XVS/XHS input\n");
		//For follower mode, switch both of them to input
		cci_write(imx585->regmap, IMX585_REG_XXS_DRV, 0x0F, NULL);
		cci_write(imx585->regmap, IMX585_REG_XXS_OUTSEL, 0x00, NULL);
	}
	imx585->common_regs_written = true;
	dev_info(imx585->clientdev, "common_regs_written\n");

	/* ------------------------------------------------------------------
	 * Work out *which* mode we’re about to run and push its registers.
	 * ----------------------------------------------------------------- */
	st   = v4l2_subdev_get_locked_active_state(&imx585->sd);
	fmt  = v4l2_subdev_state_get_format(st, 0);

	get_mode_table(imx585, fmt->code, &mode_list, &n_modes);
	mode = v4l2_find_nearest_size(mode_list, n_modes,
				      width, height,
				      fmt->width, fmt->height);

	imx585->fmt_code = fmt->code;          /* used later for HDR switch */

	ret = cci_multi_reg_write(imx585->regmap,
				  mode->reg_list.regs,
				  mode->reg_list.num_of_regs,
				  NULL);

	if (ret) {
		dev_err(imx585->clientdev, "%s failed to set mode\n", __func__);
		goto err_rpm_put;
	}

	if (imx585->clear_hdr) {
		ret = cci_multi_reg_write(imx585->regmap, common_clearHDR_mode,
					  ARRAY_SIZE(common_clearHDR_mode), NULL);
		if (ret) {
			dev_err(imx585->clientdev, "%s failed to set ClearHDR settings\n", __func__);
			goto err_rpm_put;
		}
		//16bit mode is linear, 12bit mode we need to enable gradation compression
		switch (imx585->fmt_code) {
		/* 16-bit */
		case MEDIA_BUS_FMT_SRGGB16_1X16:
		case MEDIA_BUS_FMT_SGRBG16_1X16:
		case MEDIA_BUS_FMT_SGBRG16_1X16:
		case MEDIA_BUS_FMT_SBGGR16_1X16:
		case MEDIA_BUS_FMT_Y16_1X16:
			cci_write(imx585->regmap, IMX595_REG_CCMP_EN, 0, NULL);
			cci_write(imx585->regmap, 0x3023, 0x03, NULL); // MDBIT 16-bit
			dev_info(imx585->clientdev, "16bit HDR written\n");
			break;
		/* 12-bit */
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGRBG12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
		case MEDIA_BUS_FMT_SBGGR12_1X12:
		case MEDIA_BUS_FMT_Y12_1X12:
			cci_write(imx585->regmap, IMX595_REG_CCMP_EN, 1, NULL);
			dev_info(imx585->clientdev, "12bit HDR written\n");
			break;
		default:
			break;
		}
		dev_info(imx585->clientdev, "ClearHDR_regs_written\n");

	} else {
		ret = cci_multi_reg_write(imx585->regmap, common_normal_mode,
					  ARRAY_SIZE(common_normal_mode), NULL);
		if (ret) {
			dev_err(imx585->clientdev, "%s failed to set Normal settings\n", __func__);
			goto err_rpm_put;
		}
		dev_info(imx585->clientdev, "normal_regs_written\n");
	}

	/* Disable digital clamp */
	cci_write(imx585->regmap, IMX585_REG_DIGITAL_CLAMP, 0, NULL);

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx585->sd.ctrl_handler);
	if (ret) {
		dev_err(imx585->clientdev, "%s failed to apply user values\n", __func__);
		goto err_rpm_put;
	}

	if (imx585->sync_mode == SYNC_INT_FOLLOWER || imx585->sync_mode == SYNC_INT_LEADER) {
		dev_info(imx585->clientdev, "imx585 Leader mode enabled\n");
		cci_write(imx585->regmap, IMX585_REG_XMSTA, 0x00, NULL);
	}

	/* Set stream on register */
	ret = cci_write(imx585->regmap, IMX585_REG_MODE_SELECT, IMX585_MODE_STREAMING, NULL);
	if (ret)
		goto err_rpm_put;
	dev_info(imx585->clientdev, "Start Streaming\n");
	usleep_range(IMX585_STREAM_DELAY_US, IMX585_STREAM_DELAY_US + IMX585_STREAM_DELAY_RANGE_US);

	/* vflip, hflip and HDR cannot change during streaming */
	__v4l2_ctrl_grab(imx585->vflip, true);
	__v4l2_ctrl_grab(imx585->hflip, true);
	__v4l2_ctrl_grab(imx585->hdr_mode, true);

	return 0;

err_rpm_put:
	pm_runtime_mark_last_busy(imx585->clientdev);
	pm_runtime_put_autosuspend(imx585->clientdev);
	return ret;
}

/* Stop streaming */
static int imx585_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx585 *imx585 = to_imx585(sd);
	int ret;

	/* set stream off register */
	ret = cci_write(imx585->regmap, IMX585_REG_MODE_SELECT, IMX585_MODE_STANDBY, NULL);
	if (ret)
		dev_err(imx585->clientdev, "%s failed to set stream\n", __func__);

	__v4l2_ctrl_grab(imx585->vflip, false);
	__v4l2_ctrl_grab(imx585->hflip, false);
	__v4l2_ctrl_grab(imx585->hdr_mode, false);

	pm_runtime_mark_last_busy(imx585->clientdev);
	pm_runtime_put_autosuspend(imx585->clientdev);

	return ret;
}

/* Power/clock management functions */
static int imx585_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx585 *imx585 = to_imx585(sd);
	int ret;
	dev_info(imx585->clientdev, "imx585_power_on\n");
	ret = regulator_bulk_enable(imx585_NUM_SUPPLIES,
				    imx585->supplies);
	if (ret) {
		dev_err(imx585->clientdev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx585->xclk);
	if (ret) {
		dev_err(imx585->clientdev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx585->reset_gpio, 1);
	usleep_range(IMX585_XCLR_MIN_DELAY_US,
		     IMX585_XCLR_MIN_DELAY_US + IMX585_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(imx585_NUM_SUPPLIES, imx585->supplies);
	return ret;
}

static int imx585_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx585 *imx585 = to_imx585(sd);

	gpiod_set_value_cansleep(imx585->reset_gpio, 0);
	regulator_bulk_disable(imx585_NUM_SUPPLIES, imx585->supplies);
	clk_disable_unprepare(imx585->xclk);

	return 0;
}

static int imx585_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		sel->r = *v4l2_subdev_state_get_crop(sd_state, 0);
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

static int imx585_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_rect *crop;
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = MEDIA_BUS_FMT_SRGGB12_1X12,
			.width = supported_modes[0].width,
			.height = supported_modes[0].height,
		},
	};

	imx585_set_pad_format(sd, state, &fmt);

	/* Initialize crop rectangle to mode default */
	crop = v4l2_subdev_state_get_crop(state, 0);
	*crop = supported_modes[0].crop;

	return 0;
}


static const struct v4l2_subdev_video_ops imx585_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx585_pad_ops = {
	.enum_mbus_code = imx585_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx585_set_pad_format,
	.get_selection = imx585_get_selection,
	.enum_frame_size = imx585_enum_frame_size,
	.enable_streams = imx585_enable_streams,
	.disable_streams = imx585_disable_streams,
};

static const struct v4l2_subdev_internal_ops imx585_internal_ops = {
	.init_state = imx585_init_state,
};

static const struct v4l2_subdev_ops imx585_subdev_ops = {
	.video = &imx585_video_ops,
	.pad = &imx585_pad_ops,
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

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int imx585_get_regulators(struct imx585 *imx585)
{
	unsigned int i;

	for (i = 0; i < imx585_NUM_SUPPLIES; i++)
		imx585->supplies[i].supply = imx585_supply_name[i];

	return devm_regulator_bulk_get(imx585->clientdev,
					   imx585_NUM_SUPPLIES,
					   imx585->supplies);
}

/* Verify chip ID */
static int imx585_check_module_exists(struct imx585 *imx585)
{
	int ret;
	u64 val;

	/* We don't actually have a CHIP ID register so we try to read from BLKLEVEL instead*/
	ret = cci_read(imx585->regmap, IMX585_REG_BLKLEVEL,
			      &val, NULL);
	if (ret) {
		dev_err(imx585->clientdev, "failed to read chip reg, with error %d\n", ret);
		return ret;
	}

	dev_info(imx585->clientdev, "Reg read success, Device found\n");

	return 0;
}

static int imx585_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node  *np;
	struct imx585 *imx585;
	int ret, i;
	const char *sync_mode;

	imx585 = devm_kzalloc(&client->dev, sizeof(*imx585), GFP_KERNEL);
	if (!imx585)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx585->sd, client, &imx585_subdev_ops);
	imx585->clientdev = &client->dev;

	dev_info(dev, "Reading dtoverlay config:\n");
	imx585->mono = of_property_read_bool(dev->of_node, "mono-mode");
	if (imx585->mono)
		dev_info(dev, "Mono Mode Selected, make sure you have the correct sensor variant\n");

	imx585->sync_mode = SYNC_INT_LEADER;
	if (!device_property_read_string(dev, "sony,sync-mode", &sync_mode)) {
		if (!strcmp(sync_mode, "internal-follower"))
			imx585->sync_mode = SYNC_INT_FOLLOWER;
		else if (!strcmp(sync_mode, "external"))
			imx585->sync_mode = SYNC_EXTERNAL;
	}
	dev_info(dev, "sync-mode: %s\n", sync_mode_menu[imx585->sync_mode]);

	/* Check the hardware configuration in device tree */
	if (imx585_check_hwcfg(dev, imx585))
		return -EINVAL;

	imx585->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx585->regmap))
		return dev_err_probe(dev, PTR_ERR(imx585->regmap),
				     "failed to initialize CCI\n");

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

	dev_info(dev, "XCLK %u Hz --> INCK_SEL 0x%02x\n",
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

	if (of_property_read_bool(dev->of_node, "ircut-mode")) {
		np = of_parse_phandle(dev->of_node, "ircut-controller", 0);
		if (np) {
			imx585->ircut_client = of_find_i2c_device_by_node(np);
			of_node_put(np);
			ret = imx585_ircut_write(imx585, 0x01);
			if (!ret) {
				imx585->has_ircut    = true;
				dev_info(dev, "IR-cut controller present at 0x%02x\n",
					 imx585->ircut_client->addr);
			} else {
				dev_info(dev,
					 "Can't communication with IR-cut control, dropping\n");
			}
		}
	} else {
		dev_info(dev, "No IR-cut controller\n");
	}

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);


	/* This needs the pm runtime to be registered. */
	ret = imx585_init_controls(imx585);
	if (ret)
		goto error_pm_runtime;

	/* Initialize subdev */
	imx585->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx585->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx585->sd.internal_ops = &imx585_internal_ops;

	/* Initialize source pads */
	imx585->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx585->sd.entity, 1, &imx585->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	imx585->sd.state_lock = imx585->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&imx585->sd);
	if (ret < 0) {
		dev_err_probe(dev, ret, "subdev init error\n");
		goto error_media_entity;
	}



	ret = v4l2_async_register_subdev_sensor(&imx585->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	/* Setup exposure and frame/line length limits. */
	imx585_set_framing_limits(imx585);

	/*
	 * Decrease the PM usage count. The device will get suspended after the
	 * autosuspend delay, turning the power off.
	 */
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&imx585->sd.entity);

error_handler_free:
	imx585_free_controls(imx585);

error_pm_runtime:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);

error_power_off:
	imx585_power_off(dev);

	return ret;
}

static void imx585_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx585 *imx585 = to_imx585(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	imx585_free_controls(imx585);

	pm_runtime_disable(imx585->clientdev);
	if (!pm_runtime_status_suspended(imx585->clientdev))
		imx585_power_off(imx585->clientdev);
	pm_runtime_set_suspended(imx585->clientdev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx585_pm_ops, imx585_power_off,
				 imx585_power_on, NULL);

static const struct of_device_id imx585_dt_ids[] = {
	{ .compatible = "sony,imx585"},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, imx585_dt_ids);

static struct i2c_driver imx585_i2c_driver = {
	.driver = {
		.name = "imx585",
		.pm = pm_ptr(&imx585_pm_ops),
		.of_match_table = imx585_dt_ids,
	},
	.probe = imx585_probe,
	.remove = imx585_remove,
};

module_i2c_driver(imx585_i2c_driver);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_AUTHOR("Tetsuya NOMURA <tetsuya.nomura@soho-enterprise.com>");
MODULE_DESCRIPTION("Sony imx585 sensor driver");
MODULE_LICENSE("GPL");