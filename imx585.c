// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX585 camera.
 *
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

/* --------------------------------------------------------------------------
 * Registers / limits
 * --------------------------------------------------------------------------
 */

/* Standby or streaming mode */
#define IMX585_REG_MODE_SELECT          CCI_REG8(0x3000)
#define IMX585_MODE_STANDBY             0x01
#define IMX585_MODE_STREAMING           0x00
#define IMX585_STREAM_DELAY_US          25000
#define IMX585_STREAM_DELAY_RANGE_US    1000

/* Initialisation delay between XCLR low->high and the moment sensor is ready */
#define IMX585_XCLR_MIN_DELAY_US        500000
#define IMX585_XCLR_DELAY_RANGE_US      1000

/* Leader mode and XVS/XHS direction */
#define IMX585_REG_XMSTA                CCI_REG8(0x3002)
#define IMX585_REG_XXS_DRV              CCI_REG8(0x30a6)
#define IMX585_REG_EXTMODE              CCI_REG8(0x30ce)
#define IMX585_REG_XXS_OUTSEL           CCI_REG8(0x30a4)

/* XVS pulse length, 2^n H with n=0~3 */
#define IMX585_REG_XVSLNG               CCI_REG8(0x30cc)
/* XHS pulse length, 16*(2^n) Clock with n=0~3 */
#define IMX585_REG_XHSLNG               CCI_REG8(0x30cd)

/* Clock selection */
#define IMX585_INCK_SEL                 CCI_REG8(0x3014)

/* Link speed selector */
#define IMX585_DATARATE_SEL             CCI_REG8(0x3015)

/* BIN mode: 0x01 mono bin, 0x00 color */
#define IMX585_BIN_MODE                 CCI_REG8(0x3019)

/* Lane Count */
#define IMX585_LANEMODE                 CCI_REG8(0x3040)

/* VMAX internal VBLANK */
#define IMX585_REG_VMAX                 CCI_REG24_LE(0x3028)
#define IMX585_VMAX_MAX                 0xfffff
#define IMX585_VMAX_DEFAULT             2250

/* HMAX internal HBLANK */
#define IMX585_REG_HMAX                 CCI_REG16_LE(0x302c)
#define IMX585_HMAX_MAX                 0xffff

/* SHR internal (coarse exposure) */
#define IMX585_REG_SHR                  CCI_REG24_LE(0x3050)
#define IMX585_SHR_MIN                  8
#define IMX585_SHR_MIN_HDR              10
#define IMX585_SHR_MAX                  0xfffff

/* Exposure control (lines) */
#define IMX585_EXPOSURE_MIN             2
#define IMX585_EXPOSURE_STEP            1
#define IMX585_EXPOSURE_DEFAULT         1000
#define IMX585_EXPOSURE_MAX             49865

/* Black level control */
#define IMX585_REG_BLKLEVEL             CCI_REG16_LE(0x30dc)
#define IMX585_BLKLEVEL_DEFAULT         50

/* Digital Clamp */
#define IMX585_REG_DIGITAL_CLAMP        CCI_REG8(0x3458)

/* Analog gain control */
#define IMX585_REG_ANALOG_GAIN          CCI_REG16_LE(0x306c)
#define IMX585_REG_FDG_SEL0             CCI_REG8(0x3030)
#define IMX585_ANA_GAIN_MIN      0
#define IMX585_ANA_GAIN_MAX      240
#define IMX585_ANA_GAIN_STEP            1
#define IMX585_ANA_GAIN_DEFAULT         0

/* Flip */
#define IMX585_FLIP_WINMODEH            CCI_REG8(0x3020)
#define IMX585_FLIP_WINMODEV            CCI_REG8(0x3021)

/* Pixel rate helper (sensor line clock proxy used below) */
#define IMX585_PIXEL_RATE               74250000U

/* Native and active array */
#define IMX585_NATIVE_WIDTH             3856U
#define IMX585_NATIVE_HEIGHT            2180U
#define IMX585_PIXEL_ARRAY_LEFT         8U
#define IMX585_PIXEL_ARRAY_TOP          8U
#define IMX585_PIXEL_ARRAY_WIDTH        3840U
#define IMX585_PIXEL_ARRAY_HEIGHT       2160U

/* Link frequency setup */
enum {
	IMX585_LINK_FREQ_297MHZ,   /* 594 Mbps/lane  */
	IMX585_LINK_FREQ_360MHZ,   /* 720 Mbps/lane  */
	IMX585_LINK_FREQ_445MHZ,   /* 891 Mbps/lane  */
	IMX585_LINK_FREQ_594MHZ,   /* 1188 Mbps/lane */
	IMX585_LINK_FREQ_720MHZ,   /* 1440 Mbps/lane */
	IMX585_LINK_FREQ_891MHZ,   /* 1782 Mbps/lane */
	IMX585_LINK_FREQ_1039MHZ,  /* 2079 Mbps/lane */
	IMX585_LINK_FREQ_1188MHZ,  /* 2376 Mbps/lane */
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
	[IMX585_LINK_FREQ_297MHZ]  = 297000000ULL,
	[IMX585_LINK_FREQ_360MHZ]  = 360000000ULL,
	[IMX585_LINK_FREQ_445MHZ]  = 445500000ULL,
	[IMX585_LINK_FREQ_594MHZ]  = 594000000ULL,
	[IMX585_LINK_FREQ_720MHZ]  = 720000000ULL,
	[IMX585_LINK_FREQ_891MHZ]  = 891000000ULL,
	[IMX585_LINK_FREQ_1039MHZ] = 1039500000ULL,
	[IMX585_LINK_FREQ_1188MHZ] = 1188000000ULL,
};

/* min HMAX for 4-lane 4K full res mode, x2 for 2-lane, /2 for FHD */
static const u16 HMAX_table_4lane_4K[] = {
	[IMX585_LINK_FREQ_297MHZ]  = 1584,
	[IMX585_LINK_FREQ_360MHZ]  = 1320,
	[IMX585_LINK_FREQ_445MHZ]  = 1100,
	[IMX585_LINK_FREQ_594MHZ]  = 792,
	[IMX585_LINK_FREQ_720MHZ]  = 660,
	[IMX585_LINK_FREQ_891MHZ]  = 550,
	[IMX585_LINK_FREQ_1039MHZ] = 440,
	[IMX585_LINK_FREQ_1188MHZ] = 396,
};

struct imx585_inck_cfg {
	u32 xclk_hz;
	u8  inck_sel;
};

static const struct imx585_inck_cfg imx585_inck_table[] = {
	{ 74250000, 0x00 },
	{ 37125000, 0x01 },
	{ 72000000, 0x02 },
	{ 27000000, 0x03 },
	{ 24000000, 0x04 },
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

/* Mode description */
struct imx585_mode {
	unsigned int width;
	unsigned int height;

	u8  hmax_div;       /* per-mode scaling of min HMAX */
	u16 min_hmax;       /* computed at runtime */
	u32 min_vmax;       /* computed at runtime (fits 20-bit) */

	struct v4l2_rect crop;

	struct {
		unsigned int num_of_regs;
		const struct cci_reg_sequence *regs;
	} reg_list;
};

/* --------------------------------------------------------------------------
 * Register tables
 * --------------------------------------------------------------------------
 */

static const struct cci_reg_sequence common_regs[] = {
	{ CCI_REG8(0x3002), 0x01 },
	{ CCI_REG8(0x3069), 0x00 },
	{ CCI_REG8(0x3074), 0x64 },
	{ CCI_REG8(0x30d5), 0x04 }, /* DIG_CLP_VSTART */
	{ CCI_REG8(0x3030), 0x00 }, /* FDG_SEL0 LCG (HCG=0x01) */
	{ CCI_REG8(0x30a6), 0x00 }, /* XVS_DRV [1:0] Hi-Z */
	{ CCI_REG8(0x3081), 0x00 }, /* EXP_GAIN reset */
	{ CCI_REG8(0x303a), 0x03 }, /* Disable embedded data */

	/* The remaining blocks are datasheet-recommended settings */
	{ CCI_REG8(0x3460), 0x21 }, { CCI_REG8(0x3478), 0xa1 },
	{ CCI_REG8(0x347c), 0x01 }, { CCI_REG8(0x3480), 0x01 },
	{ CCI_REG8(0x3a4e), 0x14 }, { CCI_REG8(0x3a52), 0x14 },
	{ CCI_REG8(0x3a56), 0x00 }, { CCI_REG8(0x3a5a), 0x00 },
	{ CCI_REG8(0x3a5e), 0x00 }, { CCI_REG8(0x3a62), 0x00 },
	{ CCI_REG8(0x3a6a), 0x20 }, { CCI_REG8(0x3a6c), 0x42 },
	{ CCI_REG8(0x3a6e), 0xa0 }, { CCI_REG8(0x3b2c), 0x0c },
	{ CCI_REG8(0x3b30), 0x1c }, { CCI_REG8(0x3b34), 0x0c },
	{ CCI_REG8(0x3b38), 0x1c }, { CCI_REG8(0x3ba0), 0x0c },
	{ CCI_REG8(0x3ba4), 0x1c }, { CCI_REG8(0x3ba8), 0x0c },
	{ CCI_REG8(0x3bac), 0x1c }, { CCI_REG8(0x3d3c), 0x11 },
	{ CCI_REG8(0x3d46), 0x0b }, { CCI_REG8(0x3de0), 0x3f },
	{ CCI_REG8(0x3de1), 0x08 }, { CCI_REG8(0x3e14), 0x87 },
	{ CCI_REG8(0x3e16), 0x91 }, { CCI_REG8(0x3e18), 0x91 },
	{ CCI_REG8(0x3e1a), 0x87 }, { CCI_REG8(0x3e1c), 0x78 },
	{ CCI_REG8(0x3e1e), 0x50 }, { CCI_REG8(0x3e20), 0x50 },
	{ CCI_REG8(0x3e22), 0x50 }, { CCI_REG8(0x3e24), 0x87 },
	{ CCI_REG8(0x3e26), 0x91 }, { CCI_REG8(0x3e28), 0x91 },
	{ CCI_REG8(0x3e2a), 0x87 }, { CCI_REG8(0x3e2c), 0x78 },
	{ CCI_REG8(0x3e2e), 0x50 }, { CCI_REG8(0x3e30), 0x50 },
	{ CCI_REG8(0x3e32), 0x50 }, { CCI_REG8(0x3e34), 0x87 },
	{ CCI_REG8(0x3e36), 0x91 }, { CCI_REG8(0x3e38), 0x91 },
	{ CCI_REG8(0x3e3a), 0x87 }, { CCI_REG8(0x3e3c), 0x78 },
	{ CCI_REG8(0x3e3e), 0x50 }, { CCI_REG8(0x3e40), 0x50 },
	{ CCI_REG8(0x3e42), 0x50 }, { CCI_REG8(0x4054), 0x64 },
	{ CCI_REG8(0x4148), 0xfe }, { CCI_REG8(0x4149), 0x05 },
	{ CCI_REG8(0x414a), 0xff }, { CCI_REG8(0x414b), 0x05 },
	{ CCI_REG8(0x420a), 0x03 }, { CCI_REG8(0x4231), 0x08 },
	{ CCI_REG8(0x423d), 0x9c }, { CCI_REG8(0x4242), 0xb4 },
	{ CCI_REG8(0x4246), 0xb4 }, { CCI_REG8(0x424e), 0xb4 },
	{ CCI_REG8(0x425c), 0xb4 }, { CCI_REG8(0x425e), 0xb6 },
	{ CCI_REG8(0x426c), 0xb4 }, { CCI_REG8(0x426e), 0xb6 },
	{ CCI_REG8(0x428c), 0xb4 }, { CCI_REG8(0x428e), 0xb6 },
	{ CCI_REG8(0x4708), 0x00 }, { CCI_REG8(0x4709), 0x00 },
	{ CCI_REG8(0x470a), 0xff }, { CCI_REG8(0x470b), 0x03 },
	{ CCI_REG8(0x470c), 0x00 }, { CCI_REG8(0x470d), 0x00 },
	{ CCI_REG8(0x470e), 0xff }, { CCI_REG8(0x470f), 0x03 },
	{ CCI_REG8(0x47eb), 0x1c }, { CCI_REG8(0x47f0), 0xa6 },
	{ CCI_REG8(0x47f2), 0xa6 }, { CCI_REG8(0x47f4), 0xa0 },
	{ CCI_REG8(0x47f6), 0x96 }, { CCI_REG8(0x4808), 0xa6 },
	{ CCI_REG8(0x480a), 0xa6 }, { CCI_REG8(0x480c), 0xa0 },
	{ CCI_REG8(0x480e), 0x96 }, { CCI_REG8(0x492c), 0xb2 },
	{ CCI_REG8(0x4930), 0x03 }, { CCI_REG8(0x4932), 0x03 },
	{ CCI_REG8(0x4936), 0x5b }, { CCI_REG8(0x4938), 0x82 },
	{ CCI_REG8(0x493e), 0x23 }, { CCI_REG8(0x4ba8), 0x1c },
	{ CCI_REG8(0x4ba9), 0x03 }, { CCI_REG8(0x4bac), 0x1c },
	{ CCI_REG8(0x4bad), 0x1c }, { CCI_REG8(0x4bae), 0x1c },
	{ CCI_REG8(0x4baf), 0x1c }, { CCI_REG8(0x4bb0), 0x1c },
	{ CCI_REG8(0x4bb1), 0x1c }, { CCI_REG8(0x4bb2), 0x1c },
	{ CCI_REG8(0x4bb3), 0x1c }, { CCI_REG8(0x4bb4), 0x1c },
	{ CCI_REG8(0x4bb8), 0x03 }, { CCI_REG8(0x4bb9), 0x03 },
	{ CCI_REG8(0x4bba), 0x03 }, { CCI_REG8(0x4bbb), 0x03 },
	{ CCI_REG8(0x4bbc), 0x03 }, { CCI_REG8(0x4bbd), 0x03 },
	{ CCI_REG8(0x4bbe), 0x03 }, { CCI_REG8(0x4bbf), 0x03 },
	{ CCI_REG8(0x4bc0), 0x03 }, { CCI_REG8(0x4c14), 0x87 },
	{ CCI_REG8(0x4c16), 0x91 }, { CCI_REG8(0x4c18), 0x91 },
	{ CCI_REG8(0x4c1a), 0x87 }, { CCI_REG8(0x4c1c), 0x78 },
	{ CCI_REG8(0x4c1e), 0x50 }, { CCI_REG8(0x4c20), 0x50 },
	{ CCI_REG8(0x4c22), 0x50 }, { CCI_REG8(0x4c24), 0x87 },
	{ CCI_REG8(0x4c26), 0x91 }, { CCI_REG8(0x4c28), 0x91 },
	{ CCI_REG8(0x4c2a), 0x87 }, { CCI_REG8(0x4c2c), 0x78 },
	{ CCI_REG8(0x4c2e), 0x50 }, { CCI_REG8(0x4c30), 0x50 },
	{ CCI_REG8(0x4c32), 0x50 }, { CCI_REG8(0x4c34), 0x87 },
	{ CCI_REG8(0x4c36), 0x91 }, { CCI_REG8(0x4c38), 0x91 },
	{ CCI_REG8(0x4c3a), 0x87 }, { CCI_REG8(0x4c3c), 0x78 },
	{ CCI_REG8(0x4c3e), 0x50 }, { CCI_REG8(0x4c40), 0x50 },
	{ CCI_REG8(0x4c42), 0x50 }, { CCI_REG8(0x4d12), 0x1f },
	{ CCI_REG8(0x4d13), 0x1e }, { CCI_REG8(0x4d26), 0x33 },
	{ CCI_REG8(0x4e0e), 0x59 }, { CCI_REG8(0x4e14), 0x55 },
	{ CCI_REG8(0x4e16), 0x59 }, { CCI_REG8(0x4e1e), 0x3b },
	{ CCI_REG8(0x4e20), 0x47 }, { CCI_REG8(0x4e22), 0x54 },
	{ CCI_REG8(0x4e26), 0x81 }, { CCI_REG8(0x4e2c), 0x7d },
	{ CCI_REG8(0x4e2e), 0x81 }, { CCI_REG8(0x4e36), 0x63 },
	{ CCI_REG8(0x4e38), 0x6f }, { CCI_REG8(0x4e3a), 0x7c },
	{ CCI_REG8(0x4f3a), 0x3c }, { CCI_REG8(0x4f3c), 0x46 },
	{ CCI_REG8(0x4f3e), 0x59 }, { CCI_REG8(0x4f42), 0x64 },
	{ CCI_REG8(0x4f44), 0x6e }, { CCI_REG8(0x4f46), 0x81 },
	{ CCI_REG8(0x4f4a), 0x82 }, { CCI_REG8(0x4f5a), 0x81 },
	{ CCI_REG8(0x4f62), 0xaa }, { CCI_REG8(0x4f72), 0xa9 },
	{ CCI_REG8(0x4f78), 0x36 }, { CCI_REG8(0x4f7a), 0x41 },
	{ CCI_REG8(0x4f7c), 0x61 }, { CCI_REG8(0x4f7d), 0x01 },
	{ CCI_REG8(0x4f7e), 0x7c }, { CCI_REG8(0x4f7f), 0x01 },
	{ CCI_REG8(0x4f80), 0x77 }, { CCI_REG8(0x4f82), 0x7b },
	{ CCI_REG8(0x4f88), 0x37 }, { CCI_REG8(0x4f8a), 0x40 },
	{ CCI_REG8(0x4f8c), 0x62 }, { CCI_REG8(0x4f8d), 0x01 },
	{ CCI_REG8(0x4f8e), 0x76 }, { CCI_REG8(0x4f8f), 0x01 },
	{ CCI_REG8(0x4f90), 0x5e }, { CCI_REG8(0x4f91), 0x02 },
	{ CCI_REG8(0x4f92), 0x69 }, { CCI_REG8(0x4f93), 0x02 },
	{ CCI_REG8(0x4f94), 0x89 }, { CCI_REG8(0x4f95), 0x02 },
	{ CCI_REG8(0x4f96), 0xa4 }, { CCI_REG8(0x4f97), 0x02 },
	{ CCI_REG8(0x4f98), 0x9f }, { CCI_REG8(0x4f99), 0x02 },
	{ CCI_REG8(0x4f9a), 0xa3 }, { CCI_REG8(0x4f9b), 0x02 },
	{ CCI_REG8(0x4fa0), 0x5f }, { CCI_REG8(0x4fa1), 0x02 },
	{ CCI_REG8(0x4fa2), 0x68 }, { CCI_REG8(0x4fa3), 0x02 },
	{ CCI_REG8(0x4fa4), 0x8a }, { CCI_REG8(0x4fa5), 0x02 },
	{ CCI_REG8(0x4fa6), 0x9e }, { CCI_REG8(0x4fa7), 0x02 },
	{ CCI_REG8(0x519e), 0x79 }, { CCI_REG8(0x51a6), 0xa1 },
	{ CCI_REG8(0x51f0), 0xac }, { CCI_REG8(0x51f2), 0xaa },
	{ CCI_REG8(0x51f4), 0xa5 }, { CCI_REG8(0x51f6), 0xa0 },
	{ CCI_REG8(0x5200), 0x9b }, { CCI_REG8(0x5202), 0x91 },
	{ CCI_REG8(0x5204), 0x87 }, { CCI_REG8(0x5206), 0x82 },
	{ CCI_REG8(0x5208), 0xac }, { CCI_REG8(0x520a), 0xaa },
	{ CCI_REG8(0x520c), 0xa5 }, { CCI_REG8(0x520e), 0xa0 },
	{ CCI_REG8(0x5210), 0x9b }, { CCI_REG8(0x5212), 0x91 },
	{ CCI_REG8(0x5214), 0x87 }, { CCI_REG8(0x5216), 0x82 },
	{ CCI_REG8(0x5218), 0xac }, { CCI_REG8(0x521a), 0xaa },
	{ CCI_REG8(0x521c), 0xa5 }, { CCI_REG8(0x521e), 0xa0 },
	{ CCI_REG8(0x5220), 0x9b }, { CCI_REG8(0x5222), 0x91 },
	{ CCI_REG8(0x5224), 0x87 }, { CCI_REG8(0x5226), 0x82 },
	{ CCI_REG8(0x301a), 0x00 }, /* WDMODE: Normal */
	{ CCI_REG8(0x3024), 0x00 }, /* COMBI_EN */
	{ CCI_REG8(0x3069), 0x00 },
	{ CCI_REG8(0x3074), 0x64 },
	{ CCI_REG8(0x3930), 0x0c }, /* DUR[15:8] (12-bit) */
	{ CCI_REG8(0x3931), 0x01 }, /* DUR[7:0]  (12-bit) */
	{ CCI_REG8(0x3a4c), 0x39 }, { CCI_REG8(0x3a4d), 0x01 },
	{ CCI_REG8(0x3a50), 0x48 }, { CCI_REG8(0x3a51), 0x01 },
	{ CCI_REG8(0x3e10), 0x10 }, /* ADTHEN */
	{ CCI_REG8(0x493c), 0x23 }, /* 10-bit Normal */
	{ CCI_REG8(0x4940), 0x23 }, /* 12-bit Normal */
};

/* All-pixel 4K, 12-bit */
static const struct cci_reg_sequence mode_4k_regs_12bit[] = {
	{ CCI_REG8(0x301b), 0x00 }, /* ADDMODE non-binning */
	{ CCI_REG8(0x3022), 0x02 }, /* ADBIT 12-bit */
	{ CCI_REG8(0x3023), 0x01 }, /* MDBIT 12-bit */
	{ CCI_REG8(0x30d5), 0x04 }, /* DIG_CLP_VSTART non-binning */
};

/* 2x2 binned 1080p, 12-bit */
static const struct cci_reg_sequence mode_1080_regs_12bit[] = {
	{ CCI_REG8(0x301b), 0x01 }, /* ADDMODE binning */
	{ CCI_REG8(0x3022), 0x02 }, /* ADBIT 12-bit */
	{ CCI_REG8(0x3023), 0x01 }, /* MDBIT 12-bit */
	{ CCI_REG8(0x30d5), 0x02 }, /* DIG_CLP_VSTART binning */
};

/* --------------------------------------------------------------------------
 * Mode list
 * --------------------------------------------------------------------------
 */

static struct imx585_mode supported_modes[] = {
	{
		/* 1080p60 2x2 binning */
		.width = 1928,
		.height = 1090,
		.hmax_div = 1,
		.min_hmax = 366,            /* overwritten at runtime */
		.min_vmax = IMX585_VMAX_DEFAULT,
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
		.hmax_div = 1,
		.min_hmax = 550,            /* overwritten at runtime */
		.min_vmax = IMX585_VMAX_DEFAULT,
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

/* Formats exposed per mode/bit depth */
static const u32 color_codes[] = {
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
};

static const u32 mono_codes[] = {
	MEDIA_BUS_FMT_Y12_1X12,
};

/* Regulators */
static const char * const imx585_supply_name[] = {
	"vana", /* 3.3V analog */
	"vdig", /* 1.1V core   */
	"vddl", /* 1.8V I/O    */
};

#define IMX585_NUM_SUPPLIES ARRAY_SIZE(imx585_supply_name)

/* --------------------------------------------------------------------------
 * State
 * --------------------------------------------------------------------------
 */

struct imx585 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct device *clientdev;
	struct regmap *regmap;

	struct clk *xclk;
	u32 xclk_freq;
	u8  inck_sel_val;

	unsigned int lane_count;
	unsigned int link_freq_idx;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX585_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;

	/* Controls */
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

	/* Flags/params */
	bool hcg;
	bool mono;

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
	u8   sync_mode;

	u16  hmax;
	u32  vmax;

	bool streaming;
	bool common_regs_written;
};

/* Helpers */

static inline struct imx585 *to_imx585(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx585, sd);
}

static inline void get_mode_table(struct imx585 *imx585, unsigned int code,
				  const struct imx585_mode **mode_list,
				  unsigned int *num_modes)
{
	*mode_list = NULL;
	*num_modes = 0;

	if (imx585->mono && code == MEDIA_BUS_FMT_Y12_1X12) {
		*mode_list = supported_modes;
		*num_modes = ARRAY_SIZE(supported_modes);
	} else {
		/* --- Color paths --- */
		switch (code) {
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

static u32 imx585_get_format_code(struct imx585 *imx585, u32 code)
{
	unsigned int i;

	if (imx585->mono)
		return mono_codes[0];

	for (i = 0; i < ARRAY_SIZE(color_codes); i++)
		if (color_codes[i] == code)
			return color_codes[i];
	return color_codes[0];
}

/* Recompute per-mode timing limits (HMAX/VMAX) from link/lanes/HDR */
static void imx585_update_hmax(struct imx585 *imx585)
{
	const u32 base_4lane = HMAX_table_4lane_4K[imx585->link_freq_idx];
	const u32 lane_scale = (imx585->lane_count == 2) ? 2 : 1;
	const u32 factor     = base_4lane * lane_scale;
	unsigned int i;

	dev_dbg(imx585->clientdev, "Update minimum HMAX: base=%u lane_scale=%u\n",
		base_4lane, lane_scale);

	for (i = 0; i < ARRAY_SIZE(supported_modes); ++i) {
		u32 h = factor / supported_modes[i].hmax_div;

		supported_modes[i].min_hmax = h;

		dev_dbg(imx585->clientdev, " mode %ux%u -> HMAX=%u\n",
			supported_modes[i].width, supported_modes[i].height, h);
	}
}

static void imx585_set_framing_limits(struct imx585 *imx585,
				      const struct imx585_mode *mode)
{
	u64 pixel_rate;
	u64 max_hblank;

	imx585_update_hmax(imx585);

	imx585->vmax = mode->min_vmax;
	imx585->hmax = mode->min_hmax;

	/* Pixel rate proxy: width * clock / min_hmax */
	pixel_rate = (u64)mode->width * IMX585_PIXEL_RATE;
	do_div(pixel_rate, mode->min_hmax);
	__v4l2_ctrl_modify_range(imx585->pixel_rate, pixel_rate, pixel_rate, 1,
				 pixel_rate);

	max_hblank = (u64)IMX585_HMAX_MAX * pixel_rate;
	do_div(max_hblank, IMX585_PIXEL_RATE);
	max_hblank -= mode->width;

	__v4l2_ctrl_modify_range(imx585->hblank, 0, max_hblank, 1, 0);
	__v4l2_ctrl_s_ctrl(imx585->hblank, 0);

	__v4l2_ctrl_modify_range(imx585->vblank,
				 mode->min_vmax - mode->height,
				 IMX585_VMAX_MAX - mode->height,
				 1, mode->min_vmax - mode->height);
	__v4l2_ctrl_s_ctrl(imx585->vblank, mode->min_vmax - mode->height);

	__v4l2_ctrl_modify_range(imx585->exposure, IMX585_EXPOSURE_MIN,
				 imx585->vmax - IMX585_SHR_MIN_HDR, 1,
				 IMX585_EXPOSURE_DEFAULT);

	dev_dbg(imx585->clientdev, "Framing: VMAX=%u HMAX=%u pixel_rate=%llu\n",
		imx585->vmax, imx585->hmax, pixel_rate);
}

/* --------------------------------------------------------------------------
 * Controls
 * --------------------------------------------------------------------------
 */

static int imx585_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx585 *imx585 = container_of(ctrl->handler, struct imx585, ctrl_handler);
	const struct imx585_mode *mode, *mode_list;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	unsigned int num_modes;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&imx585->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);

	get_mode_table(imx585, fmt->code, &mode_list, &num_modes);
	mode = v4l2_find_nearest_size(mode_list, num_modes, width, height,
				      fmt->width, fmt->height);

	/* Apply control only when powered (runtime active). */
	if (!pm_runtime_get_if_active(imx585->clientdev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE: {
		u32 shr = (imx585->vmax - ctrl->val) & ~1U; /* SHR always a multiple of 2 */

		dev_dbg(imx585->clientdev, "EXPOSURE=%u -> SHR=%u (VMAX=%u HMAX=%u)\n",
			ctrl->val, shr, imx585->vmax, imx585->hmax);

		ret = cci_write(imx585->regmap, IMX585_REG_SHR, shr, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "SHR write failed (%d)\n", ret);
		break;
	}
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(imx585->clientdev, "ANALOG_GAIN=%u\n", ctrl->val);
		ret = cci_write(imx585->regmap, IMX585_REG_ANALOG_GAIN, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "Gain write failed (%d)\n", ret);
		break;
	case V4L2_CID_VBLANK: {
		u32 current_exposure = imx585->exposure->cur.val;

		imx585->vmax = (mode->height + ctrl->val) & ~1U;

		current_exposure = clamp_t(u32, current_exposure,
					   IMX585_EXPOSURE_MIN, imx585->vmax - IMX585_SHR_MIN);
		__v4l2_ctrl_modify_range(imx585->exposure,
					 IMX585_EXPOSURE_MIN, imx585->vmax - IMX585_SHR_MIN, 1,
					 current_exposure);

		dev_dbg(imx585->clientdev, "VBLANK=%u -> VMAX=%u\n", ctrl->val, imx585->vmax);

		ret = cci_write(imx585->regmap, IMX585_REG_VMAX, imx585->vmax, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "VMAX write failed (%d)\n", ret);
		break;
	}
	case V4L2_CID_HBLANK: {
		u64 pixel_rate = (u64)mode->width * IMX585_PIXEL_RATE;
		u64 hmax;

		do_div(pixel_rate, mode->min_hmax);
		hmax = (u64)(mode->width + ctrl->val) * IMX585_PIXEL_RATE;
		div64_u64(hmax, pixel_rate);
		imx585->hmax = (u32)hmax;

		dev_dbg(imx585->clientdev, "HBLANK=%u -> HMAX=%u\n", ctrl->val, imx585->hmax);

		ret = cci_write(imx585->regmap, IMX585_REG_HMAX, imx585->hmax, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "HMAX write failed (%d)\n", ret);
		break;
	}
	case V4L2_CID_HFLIP:
		ret = cci_write(imx585->regmap, IMX585_FLIP_WINMODEH, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "HFLIP write failed (%d)\n", ret);
		break;
	case V4L2_CID_VFLIP:
		ret = cci_write(imx585->regmap, IMX585_FLIP_WINMODEV, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "VFLIP write failed (%d)\n", ret);
		break;
	case V4L2_CID_BRIGHTNESS: {
		u16 blacklevel = min_t(u32, ctrl->val, 4095);

		ret = cci_write(imx585->regmap, IMX585_REG_BLKLEVEL, blacklevel, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "BLKLEVEL write failed (%d)\n", ret);
		break;
	}
	default:
		dev_dbg(imx585->clientdev, "Unhandled ctrl %s: id=0x%x, val=0x%x\n",
			ctrl->name, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(imx585->clientdev);
	return ret;
}

static const struct v4l2_ctrl_ops imx585_ctrl_ops = {
	.s_ctrl = imx585_set_ctrl,
};

static int imx585_init_controls(struct imx585 *imx585)
{
	struct v4l2_ctrl_handler *hdl = &imx585->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 16);

	/* Read-only, updated per mode */
	imx585->pixel_rate = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       1, UINT_MAX, 1, 1);

	imx585->link_freq =
		v4l2_ctrl_new_int_menu(hdl, &imx585_ctrl_ops, V4L2_CID_LINK_FREQ,
				       0, 0, &link_freqs[imx585->link_freq_idx]);
	if (imx585->link_freq)
		imx585->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx585->vblank = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xFFFFF, 1, 0);
	imx585->hblank = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xFFFF, 1, 0);
	imx585->blacklevel = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					       V4L2_CID_BRIGHTNESS, 0, 0xFFFF, 1,
					       IMX585_BLKLEVEL_DEFAULT);

	imx585->exposure = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX585_EXPOSURE_MIN, IMX585_EXPOSURE_MAX,
					     IMX585_EXPOSURE_STEP, IMX585_EXPOSURE_DEFAULT);

	imx585->gain = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
					 IMX585_ANA_GAIN_MIN, IMX585_ANA_GAIN_MAX,
					 IMX585_ANA_GAIN_STEP, IMX585_ANA_GAIN_DEFAULT);

	imx585->hflip = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	imx585->vflip = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (hdl->error) {
		ret = hdl->error;
		dev_err(imx585->clientdev, "control init failed (%d)\n", ret);
		goto err_free;
	}

	ret = v4l2_fwnode_device_parse(imx585->clientdev, &props);
	if (ret)
		goto err_free;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &imx585_ctrl_ops, &props);
	if (ret)
		goto err_free;

	imx585->sd.ctrl_handler = hdl;
	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static void imx585_free_controls(struct imx585 *imx585)
{
	v4l2_ctrl_handler_free(imx585->sd.ctrl_handler);
}

/* --------------------------------------------------------------------------
 * Pad ops / formats
 * --------------------------------------------------------------------------
 */

static int imx585_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx585 *imx585 = to_imx585(sd);
	unsigned int entries;
	const u32 *tbl;

	if (imx585->mono) {
		if (code->index)
			return -EINVAL;
		code->code = MEDIA_BUS_FMT_Y12_1X12;
		return 0;
	}

	tbl = color_codes;
	entries = ARRAY_SIZE(color_codes) / 4;

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

	fse->min_width  = mode_list[fse->index].width;
	fse->max_width  = fse->min_width;
	fse->min_height = mode_list[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int imx585_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx585 *imx585 = to_imx585(sd);
	const struct imx585_mode *mode_list, *mode;
	unsigned int num_modes;
	struct v4l2_mbus_framefmt *format;

	get_mode_table(imx585, fmt->format.code, &mode_list, &num_modes);
	mode = v4l2_find_nearest_size(mode_list, num_modes, width, height,
				      fmt->format.width, fmt->format.height);

	fmt->format.width        = mode->width;
	fmt->format.height       = mode->height;
	fmt->format.field        = V4L2_FIELD_NONE;
	fmt->format.colorspace   = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc    = V4L2_YCBCR_ENC_601;
	fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->format.xfer_func    = V4L2_XFER_FUNC_NONE;

	format = v4l2_subdev_state_get_format(sd_state, 0);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		imx585_set_framing_limits(imx585, mode);

	*format = fmt->format;
	return 0;
}

/* --------------------------------------------------------------------------
 * Stream on/off
 * --------------------------------------------------------------------------
 */

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
		dev_err(imx585->clientdev, "Failed to write common settings\n");
		goto err_rpm_put;
	}

	ret = cci_write(imx585->regmap, IMX585_INCK_SEL, imx585->inck_sel_val, NULL);
	if (!ret)
		ret = cci_write(imx585->regmap, IMX585_REG_BLKLEVEL, IMX585_BLKLEVEL_DEFAULT, NULL);
	if (!ret)
		ret = cci_write(imx585->regmap, IMX585_DATARATE_SEL,
				link_freqs_reg_value[imx585->link_freq_idx], NULL);
	if (ret)
		goto err_rpm_put;

	ret = cci_write(imx585->regmap, IMX585_LANEMODE,
			(imx585->lane_count == 2) ? 0x01 : 0x03, NULL);
	if (ret)
		goto err_rpm_put;

	/* Mono bin flag (datasheet: 0x01 mono, 0x00 color) */
	ret = cci_write(imx585->regmap, IMX585_BIN_MODE, imx585->mono ? 0x01 : 0x00, NULL);
	if (ret)
		goto err_rpm_put;

	/* Sync configuration */
	if (imx585->sync_mode == SYNC_INT_FOLLOWER) {
		dev_dbg(imx585->clientdev, "Internal sync follower: XVS input\n");
		cci_write(imx585->regmap, IMX585_REG_EXTMODE, 0x01, NULL);
		cci_write(imx585->regmap, IMX585_REG_XXS_DRV, 0x03, NULL); /* XHS out, XVS in */
		cci_write(imx585->regmap, IMX585_REG_XXS_OUTSEL, 0x08, NULL); /* disable XVS OUT */
	} else if (imx585->sync_mode == SYNC_INT_LEADER) {
		dev_dbg(imx585->clientdev, "Internal sync leader: XVS/XHS output\n");
		cci_write(imx585->regmap, IMX585_REG_EXTMODE, 0x00, NULL);
		cci_write(imx585->regmap, IMX585_REG_XXS_DRV, 0x00, NULL); /* XHS/XVS out */
		cci_write(imx585->regmap, IMX585_REG_XXS_OUTSEL, 0x0A, NULL);
	} else {
		dev_dbg(imx585->clientdev, "Follower: XVS/XHS input\n");
		cci_write(imx585->regmap, IMX585_REG_XXS_DRV, 0x0F, NULL); /* inputs */
		cci_write(imx585->regmap, IMX585_REG_XXS_OUTSEL, 0x00, NULL);
	}

	imx585->common_regs_written = true;

	/* Select mode */
	st  = v4l2_subdev_get_locked_active_state(&imx585->sd);
	fmt = v4l2_subdev_state_get_format(st, 0);

	get_mode_table(imx585, fmt->code, &mode_list, &n_modes);
	mode = v4l2_find_nearest_size(mode_list, n_modes, width, height,
				      fmt->width, fmt->height);

	ret = cci_multi_reg_write(imx585->regmap, mode->reg_list.regs,
				  mode->reg_list.num_of_regs, NULL);
	if (ret) {
		dev_err(imx585->clientdev, "Failed to write mode registers\n");
		goto err_rpm_put;
	}

	/* Disable digital clamp */
	cci_write(imx585->regmap, IMX585_REG_DIGITAL_CLAMP, 0x00, NULL);

	/* Apply user controls after writing the base tables */
	ret = __v4l2_ctrl_handler_setup(imx585->sd.ctrl_handler);
	if (ret) {
		dev_err(imx585->clientdev, "Control handler setup failed\n");
		goto err_rpm_put;
	}

	if (imx585->sync_mode != SYNC_EXTERNAL)
		cci_write(imx585->regmap, IMX585_REG_XMSTA, 0x00, NULL);

	ret = cci_write(imx585->regmap, IMX585_REG_MODE_SELECT, IMX585_MODE_STREAMING, NULL);
	if (ret)
		goto err_rpm_put;

	dev_dbg(imx585->clientdev, "Streaming started\n");
	usleep_range(IMX585_STREAM_DELAY_US,
		     IMX585_STREAM_DELAY_US + IMX585_STREAM_DELAY_RANGE_US);

	/* vflip, hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx585->vflip, true);
	__v4l2_ctrl_grab(imx585->hflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(imx585->clientdev);
	return ret;
}

static int imx585_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx585 *imx585 = to_imx585(sd);
	int ret;

	ret = cci_write(imx585->regmap, IMX585_REG_MODE_SELECT, IMX585_MODE_STANDBY, NULL);
	if (ret)
		dev_err(imx585->clientdev, "Failed to stop streaming\n");

	__v4l2_ctrl_grab(imx585->vflip, false);
	__v4l2_ctrl_grab(imx585->hflip, false);

	pm_runtime_put_autosuspend(imx585->clientdev);

	return ret;
}

/* --------------------------------------------------------------------------
 * Power / runtime PM
 * --------------------------------------------------------------------------
 */

static int imx585_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx585 *imx585 = to_imx585(sd);
	int ret;

	dev_dbg(imx585->clientdev, "power_on\n");

	ret = regulator_bulk_enable(IMX585_NUM_SUPPLIES, imx585->supplies);
	if (ret) {
		dev_err(imx585->clientdev, "Failed to enable regulators\n");
		return ret;
	}

	ret = clk_prepare_enable(imx585->xclk);
	if (ret) {
		dev_err(imx585->clientdev, "Failed to enable clock\n");
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx585->reset_gpio, 1);
	usleep_range(IMX585_XCLR_MIN_DELAY_US,
		     IMX585_XCLR_MIN_DELAY_US + IMX585_XCLR_DELAY_RANGE_US);
	return 0;

reg_off:
	regulator_bulk_disable(IMX585_NUM_SUPPLIES, imx585->supplies);
	return ret;
}

static int imx585_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx585 *imx585 = to_imx585(sd);

	dev_dbg(imx585->clientdev, "power_off\n");

	gpiod_set_value_cansleep(imx585->reset_gpio, 0);
	regulator_bulk_disable(IMX585_NUM_SUPPLIES, imx585->supplies);
	clk_disable_unprepare(imx585->xclk);

	return 0;
}

/* --------------------------------------------------------------------------
 * Selection / state
 * --------------------------------------------------------------------------
 */

static int imx585_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(sd_state, 0);
		return 0;
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
	default:
		return -EINVAL;
	}
}

static int imx585_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_rect *crop;
	struct imx585 *imx585 = to_imx585(sd);
	struct v4l2_subdev_format fmt = {
		.which  = V4L2_SUBDEV_FORMAT_TRY,
		.pad    = 0,
		.format = {
			.code   = imx585->mono ? MEDIA_BUS_FMT_Y12_1X12
					    : MEDIA_BUS_FMT_SRGGB12_1X12,
			.width  = supported_modes[0].width,
			.height = supported_modes[0].height,
		},
	};

	imx585_set_pad_format(sd, state, &fmt);

	crop = v4l2_subdev_state_get_crop(state, 0);
	*crop = supported_modes[0].crop;

	return 0;
}

/* --------------------------------------------------------------------------
 * Subdev ops
 * --------------------------------------------------------------------------
 */

static const struct v4l2_subdev_video_ops imx585_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx585_pad_ops = {
	.enum_mbus_code = imx585_enum_mbus_code,
	.get_fmt        = v4l2_subdev_get_fmt,
	.set_fmt        = imx585_set_pad_format,
	.get_selection  = imx585_get_selection,
	.enum_frame_size = imx585_enum_frame_size,
	.enable_streams  = imx585_enable_streams,
	.disable_streams = imx585_disable_streams,
};

static const struct v4l2_subdev_internal_ops imx585_internal_ops = {
	.init_state = imx585_init_state,
};

static const struct v4l2_subdev_ops imx585_subdev_ops = {
	.video = &imx585_video_ops,
	.pad   = &imx585_pad_ops,
};

/* --------------------------------------------------------------------------
 * Probe / remove
 * --------------------------------------------------------------------------
 */

static int imx585_check_hwcfg(struct device *dev, struct imx585 *imx585)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	int ret = -EINVAL;
	int i;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep)) {
		dev_err(dev, "could not parse endpoint\n");
		goto out_put;
	}

	if (ep.bus.mipi_csi2.num_data_lanes != 2 &&
	    ep.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(dev, "only 2 or 4 data lanes supported\n");
		goto out_free;
	}
	imx585->lane_count = ep.bus.mipi_csi2.num_data_lanes;
	dev_dbg(dev, "Data lanes: %u\n", imx585->lane_count);

	if (!ep.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property missing\n");
		goto out_free;
	}

	for (i = 0; i < ARRAY_SIZE(link_freqs); i++) {
		if (link_freqs[i] == ep.link_frequencies[0]) {
			imx585->link_freq_idx = i;
			break;
		}
	}
	if (i == ARRAY_SIZE(link_freqs)) {
		dev_err(dev, "unsupported link frequency: %llu\n",
			(unsigned long long)ep.link_frequencies[0]);
		goto out_free;
	}

	dev_dbg(dev, "Link speed: %llu Hz\n",
		(unsigned long long)ep.link_frequencies[0]);

	ret = 0;

out_free:
	v4l2_fwnode_endpoint_free(&ep);
out_put:
	fwnode_handle_put(endpoint);
	return ret;
}

static int imx585_get_regulators(struct imx585 *imx585)
{
	unsigned int i;

	for (i = 0; i < IMX585_NUM_SUPPLIES; i++)
		imx585->supplies[i].supply = imx585_supply_name[i];

	return devm_regulator_bulk_get(imx585->clientdev,
				       IMX585_NUM_SUPPLIES, imx585->supplies);
}

static int imx585_check_module_exists(struct imx585 *imx585)
{
	int ret;
	u64 val;

	/* No chip-id register; read a known register as a presence test */
	ret = cci_read(imx585->regmap, IMX585_REG_BLKLEVEL, &val, NULL);
	if (ret) {
		dev_err(imx585->clientdev, "register read failed (%d)\n", ret);
		return ret;
	}

	dev_dbg(imx585->clientdev, "Sensor detected\n");
	return 0;
}

static int imx585_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx585 *imx585;
	const char *sync_mode;
	int ret, i;

	imx585 = devm_kzalloc(dev, sizeof(*imx585), GFP_KERNEL);
	if (!imx585)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx585->sd, client, &imx585_subdev_ops);
	imx585->clientdev = dev;

	imx585->mono = of_device_is_compatible(dev->of_node, "sony,imx585-aamj1");
	dev_dbg(dev, "mono=%d\n", imx585->mono);

	imx585->sync_mode = SYNC_INT_LEADER;
	if (!device_property_read_string(dev, "sony,sync-mode", &sync_mode)) {
		if (!strcmp(sync_mode, "internal-follower"))
			imx585->sync_mode = SYNC_INT_FOLLOWER;
		else if (!strcmp(sync_mode, "external"))
			imx585->sync_mode = SYNC_EXTERNAL;
	}
	dev_dbg(dev, "sync-mode: %s\n", sync_mode_menu[imx585->sync_mode]);

	ret = imx585_check_hwcfg(dev, imx585);
	if (ret)
		return ret;

	imx585->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx585->regmap))
		return dev_err_probe(dev, PTR_ERR(imx585->regmap), "CCI init failed\n");

	imx585->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx585->xclk))
		return dev_err_probe(dev, PTR_ERR(imx585->xclk), "xclk missing\n");

	imx585->xclk_freq = clk_get_rate(imx585->xclk);
	for (i = 0; i < ARRAY_SIZE(imx585_inck_table); ++i) {
		if (imx585_inck_table[i].xclk_hz == imx585->xclk_freq) {
			imx585->inck_sel_val = imx585_inck_table[i].inck_sel;
			break;
		}
	}
	if (i == ARRAY_SIZE(imx585_inck_table))
		return dev_err_probe(dev, -EINVAL, "unsupported XCLK %u Hz\n", imx585->xclk_freq);

	dev_dbg(dev, "XCLK %u Hz -> INCK_SEL 0x%02x\n",
		imx585->xclk_freq, imx585->inck_sel_val);

	ret = imx585_get_regulators(imx585);
	if (ret)
		return dev_err_probe(dev, ret, "regulators\n");

	imx585->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);

	/* Power on to probe the device */
	ret = imx585_power_on(dev);
	if (ret)
		return ret;

	ret = imx585_check_module_exists(imx585);
	if (ret)
		goto err_power_off;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = imx585_init_controls(imx585);
	if (ret)
		goto err_pm;

	imx585->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx585->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx585->sd.internal_ops = &imx585_internal_ops;

	imx585->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx585->sd.entity, 1, &imx585->pad);
	if (ret) {
		dev_err(dev, "entity pads init failed: %d\n", ret);
		goto err_ctrls;
	}

	imx585->sd.state_lock = imx585->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&imx585->sd);
	if (ret) {
		dev_err_probe(dev, ret, "subdev init\n");
		goto err_entity;
	}

	ret = v4l2_async_register_subdev_sensor(&imx585->sd);
	if (ret) {
		dev_err(dev, "sensor subdev register failed: %d\n", ret);
		goto err_entity;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return 0;

err_entity:
	media_entity_cleanup(&imx585->sd.entity);
err_ctrls:
	imx585_free_controls(imx585);
err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
err_power_off:
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

static const struct of_device_id imx585_of_match[] = {
	{ .compatible = "sony,imx585-aaqj1" },
	{ .compatible = "sony,imx585-aamj1" }, /* monochrome variant */
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx585_of_match);

static struct i2c_driver imx585_i2c_driver = {
	.driver = {
		.name  = "imx585",
		.pm    = pm_ptr(&imx585_pm_ops),
		.of_match_table = imx585_of_match,
	},
	.probe  = imx585_probe,
	.remove = imx585_remove,
};
module_i2c_driver(imx585_i2c_driver);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_AUTHOR("Tetsuya Nomura <tetsuya.nomura@soho-enterprise.com>");
MODULE_DESCRIPTION("Sony IMX585 sensor driver");
MODULE_LICENSE("GPL");
