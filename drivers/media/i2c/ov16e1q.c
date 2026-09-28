// SPDX-License-Identifier: GPL-2.0-only
/*
 * OmniVision OV16E1Q image sensor driver
 *
 * The OV16E1Q is a 16 Mpixel (4608x3456 active, 1.0 um) quad-Bayer
 * sensor with a four-lane MIPI CSI-2 D-PHY interface. Register settings
 * were captured from a working vendor configuration (19.2 MHz external
 * clock, 1512 Mbps per lane); no datasheet was available.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define OV16E1Q_XCLK_FREQ		(19200 * HZ_PER_KHZ)
#define OV16E1Q_LINK_FREQ		(756 * HZ_PER_MHZ)
#define OV16E1Q_NUM_DATA_LANES		4
#define OV16E1Q_BPP			10
/* Pixel rate = link frequency * 2 (DDR) * lanes / bits per pixel */
#define OV16E1Q_PIXEL_RATE		(604800 * HZ_PER_KHZ)

#define OV16E1Q_REG_CHIP_ID		CCI_REG24(0x300a)
#define OV16E1Q_CHIP_ID_MASK		0xffff
#define OV16E1Q_CHIP_ID			0x1645

#define OV16E1Q_REG_MODE_SELECT		CCI_REG8(0x0100)
#define OV16E1Q_MODE_STANDBY		0x00
#define OV16E1Q_MODE_STREAMING		0x01

/* Exposure, in lines */
#define OV16E1Q_REG_EXPOSURE		CCI_REG16(0x3501)
#define OV16E1Q_EXPOSURE_MIN		8
#define OV16E1Q_EXPOSURE_MAX		0x7fff
#define OV16E1Q_EXPOSURE_MARGIN		16
#define OV16E1Q_EXPOSURE_STEP		1

/* Analogue gain, 1x = 0x100, bit 0 is not used */
#define OV16E1Q_REG_ANALOGUE_GAIN	CCI_REG16(0x3508)
#define OV16E1Q_ANALOGUE_GAIN_MASK	0x7ffe
#define OV16E1Q_ANALOGUE_GAIN_MIN	0x100
#define OV16E1Q_ANALOGUE_GAIN_MAX	0xf80
#define OV16E1Q_ANALOGUE_GAIN_STEP	1
#define OV16E1Q_ANALOGUE_GAIN_DEFAULT	0x100

/* Digital gain, 1x = 0x400, register holds the value in bits [19:6] */
#define OV16E1Q_REG_DIGITAL_GAIN	CCI_REG24(0x350a)
#define OV16E1Q_DIGITAL_GAIN_SHIFT	6
#define OV16E1Q_DIGITAL_GAIN_MIN	0x400
#define OV16E1Q_DIGITAL_GAIN_MAX	0x1000
#define OV16E1Q_DIGITAL_GAIN_STEP	1
#define OV16E1Q_DIGITAL_GAIN_DEFAULT	0x400

/* Frame length, in lines; must be even */
#define OV16E1Q_REG_VTS			CCI_REG16(0x380e)
#define OV16E1Q_VTS_MAX			0xfffe

/* Test pattern: solid colour with per-channel data */
#define OV16E1Q_REG_TEST_PATTERN	CCI_REG8(0x5081)
#define OV16E1Q_TEST_PATTERN_DISABLE	0x00
#define OV16E1Q_TEST_PATTERN_SOLID	0x01
#define OV16E1Q_REG_TP_BLUE		CCI_REG16(0x8000)
#define OV16E1Q_REG_TP_GREENB		CCI_REG16(0x8002)
#define OV16E1Q_REG_TP_GREENR		CCI_REG16(0x8004)
#define OV16E1Q_REG_TP_RED		CCI_REG16(0x8006)
#define OV16E1Q_TP_COLOUR_MAX		0x3ff

/* Pixel array */
#define OV16E1Q_NATIVE_WIDTH		4624
#define OV16E1Q_NATIVE_HEIGHT		3472
#define OV16E1Q_PIXEL_ARRAY_LEFT	8
#define OV16E1Q_PIXEL_ARRAY_TOP		8
#define OV16E1Q_PIXEL_ARRAY_WIDTH	4608
#define OV16E1Q_PIXEL_ARRAY_HEIGHT	3456

#define OV16E1Q_MBUS_CODE		MEDIA_BUS_FMT_SGRBG10_1X10

static const s64 ov16e1q_link_freq_menu[] = {
	OV16E1Q_LINK_FREQ,
};

static const char * const ov16e1q_supply_names[] = {
	"avdd",		/* Analogue power */
	"dovdd",	/* Digital I/O power */
};

#define OV16E1Q_NUM_SUPPLIES	ARRAY_SIZE(ov16e1q_supply_names)

static const char * const ov16e1q_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
};

struct ov16e1q_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct ov16e1q_mode {
	u32 width;
	u32 height;
	/* Line length in units of the pixel rate */
	u32 hts;
	/* Default frame length in lines */
	u32 vts_def;
	/* Default exposure in lines */
	u32 exposure_def;
	/* Analogue crop rectangle, in native pixel array coordinates */
	struct v4l2_rect crop;
	struct ov16e1q_reg_list reg_list;
};

struct ov16e1q {
	struct device *dev;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[OV16E1Q_NUM_SUPPLIES];
	struct regulator *dvdd;

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;

	const struct ov16e1q_mode *mode;
};

static inline struct ov16e1q *to_ov16e1q(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov16e1q, sd);
}

/*
 * Common settings: software reset, PLL (19.2 MHz in, 1512 Mbps per lane),
 * MIPI and analogue setup.
 */
static const struct cci_reg_sequence ov16e1q_init_regs[] = {
	{ CCI_REG8(0x0103), 0x01 },
	{ CCI_REG8(0x0102), 0x01 },
	{ CCI_REG8(0x0300), 0xfd },
	{ CCI_REG8(0x0301), 0x2c },
	{ CCI_REG8(0x0304), 0x3d },
	{ CCI_REG8(0x0305), 0x01 },
	{ CCI_REG8(0x030b), 0x04 },
	{ CCI_REG8(0x030d), 0x95 },
	{ CCI_REG8(0x030f), 0x07 },
	{ CCI_REG8(0x0313), 0x40 },
	{ CCI_REG8(0x0314), 0x04 },
	{ CCI_REG8(0x0316), 0x95 },
	{ CCI_REG8(0x0318), 0x00 },
	{ CCI_REG8(0x0319), 0x00 },
	{ CCI_REG8(0x031c), 0x4f },
	{ CCI_REG8(0x0321), 0x07 },
	{ CCI_REG8(0x0324), 0x00 },
	{ CCI_REG8(0x300e), 0x22 },
	{ CCI_REG8(0x3016), 0xb4 },
	{ CCI_REG8(0x3019), 0xd2 },
	{ CCI_REG8(0x301b), 0x16 },
	{ CCI_REG8(0x3022), 0xd0 },
	{ CCI_REG8(0x3025), 0x03 },
	{ CCI_REG8(0x3026), 0x10 },
	{ CCI_REG8(0x3027), 0x08 },
	{ CCI_REG8(0x3028), 0xc3 },
	{ CCI_REG8(0x3103), 0x0e },
	{ CCI_REG8(0x3400), 0x04 },
	{ CCI_REG8(0x3408), 0x03 },
	{ CCI_REG8(0x340c), 0xff },
	{ CCI_REG8(0x3415), 0x00 },
	{ CCI_REG8(0x3421), 0x08 },
	{ CCI_REG8(0x3423), 0x11 },
	{ CCI_REG8(0x3424), 0x40 },
	{ CCI_REG8(0x3425), 0x10 },
	{ CCI_REG8(0x3426), 0x11 },
	{ CCI_REG8(0x3603), 0x0f },
	{ CCI_REG8(0x3609), 0xc0 },
	{ CCI_REG8(0x360c), 0x00 },
	{ CCI_REG8(0x361b), 0x8a },
	{ CCI_REG8(0x3620), 0x48 },
	{ CCI_REG8(0x3621), 0x10 },
	{ CCI_REG8(0x3622), 0x99 },
	{ CCI_REG8(0x3623), 0x77 },
	{ CCI_REG8(0x3624), 0xff },
	{ CCI_REG8(0x3625), 0xff },
	{ CCI_REG8(0x3626), 0x02 },
	{ CCI_REG8(0x3627), 0x33 },
	{ CCI_REG8(0x3628), 0x77 },
	{ CCI_REG8(0x3629), 0x09 },
	{ CCI_REG8(0x362b), 0x03 },
	{ CCI_REG8(0x362d), 0x03 },
	{ CCI_REG8(0x3631), 0xf2 },
	{ CCI_REG8(0x3633), 0x08 },
	{ CCI_REG8(0x3634), 0x10 },
	{ CCI_REG8(0x3635), 0x10 },
	{ CCI_REG8(0x3636), 0x10 },
	{ CCI_REG8(0x3637), 0xcc },
	{ CCI_REG8(0x3638), 0xcc },
	{ CCI_REG8(0x3639), 0xc0 },
	{ CCI_REG8(0x363a), 0xcc },
	{ CCI_REG8(0x3653), 0x00 },
	{ CCI_REG8(0x3644), 0x40 },
	{ CCI_REG8(0x3664), 0x00 },
	{ CCI_REG8(0x360a), 0x09 },
	{ CCI_REG8(0x360b), 0x1c },
	{ CCI_REG8(0x3632), 0xf2 },
	{ CCI_REG8(0x363b), 0x10 },
	{ CCI_REG8(0x363c), 0x1a },
	{ CCI_REG8(0x363d), 0x2d },
	{ CCI_REG8(0x363e), 0x58 },
	{ CCI_REG8(0x3701), 0x18 },
	{ CCI_REG8(0x3702), 0x40 },
	{ CCI_REG8(0x3703), 0x2d },
	{ CCI_REG8(0x3704), 0x18 },
	{ CCI_REG8(0x3706), 0x32 },
	{ CCI_REG8(0x3707), 0x05 },
	{ CCI_REG8(0x3708), 0x29 },
	{ CCI_REG8(0x370b), 0x82 },
	{ CCI_REG8(0x370c), 0x08 },
	{ CCI_REG8(0x3711), 0x00 },
	{ CCI_REG8(0x3717), 0x01 },
	{ CCI_REG8(0x3718), 0x13 },
	{ CCI_REG8(0x371b), 0x90 },
	{ CCI_REG8(0x371c), 0x02 },
	{ CCI_REG8(0x371d), 0x18 },
	{ CCI_REG8(0x371f), 0x08 },
	{ CCI_REG8(0x3721), 0x0e },
	{ CCI_REG8(0x3722), 0x04 },
	{ CCI_REG8(0x3724), 0x05 },
	{ CCI_REG8(0x3725), 0x21 },
	{ CCI_REG8(0x3727), 0x23 },
	{ CCI_REG8(0x3730), 0x04 },
	{ CCI_REG8(0x3731), 0x04 },
	{ CCI_REG8(0x3732), 0x04 },
	{ CCI_REG8(0x3733), 0x04 },
	{ CCI_REG8(0x3734), 0x04 },
	{ CCI_REG8(0x3735), 0x04 },
	{ CCI_REG8(0x3736), 0x07 },
	{ CCI_REG8(0x3737), 0x01 },
	{ CCI_REG8(0x3738), 0x07 },
	{ CCI_REG8(0x3739), 0x01 },
	{ CCI_REG8(0x373a), 0x07 },
	{ CCI_REG8(0x373b), 0x03 },
	{ CCI_REG8(0x373c), 0x07 },
	{ CCI_REG8(0x373d), 0x07 },
	{ CCI_REG8(0x373e), 0x07 },
	{ CCI_REG8(0x373f), 0x18 },
	{ CCI_REG8(0x3740), 0x04 },
	{ CCI_REG8(0x3741), 0x02 },
	{ CCI_REG8(0x3742), 0x04 },
	{ CCI_REG8(0x3743), 0x07 },
	{ CCI_REG8(0x3744), 0x15 },
	{ CCI_REG8(0x3745), 0x07 },
	{ CCI_REG8(0x3746), 0x15 },
	{ CCI_REG8(0x3747), 0x36 },
	{ CCI_REG8(0x3749), 0xa2 },
	{ CCI_REG8(0x374a), 0x0d },
	{ CCI_REG8(0x374b), 0x23 },
	{ CCI_REG8(0x374f), 0x07 },
	{ CCI_REG8(0x3751), 0x04 },
	{ CCI_REG8(0x3762), 0x08 },
	{ CCI_REG8(0x3763), 0x08 },
	{ CCI_REG8(0x3764), 0x08 },
	{ CCI_REG8(0x3765), 0x10 },
	{ CCI_REG8(0x3767), 0x28 },
	{ CCI_REG8(0x3768), 0x00 },
	{ CCI_REG8(0x3769), 0x08 },
	{ CCI_REG8(0x376a), 0x10 },
	{ CCI_REG8(0x3770), 0x20 },
	{ CCI_REG8(0x3773), 0x2b },
	{ CCI_REG8(0x3774), 0x38 },
	{ CCI_REG8(0x3775), 0x2d },
	{ CCI_REG8(0x3776), 0x08 },
	{ CCI_REG8(0x3777), 0x02 },
	{ CCI_REG8(0x3778), 0x37 },
	{ CCI_REG8(0x377a), 0x0d },
	{ CCI_REG8(0x377b), 0x2d },
	{ CCI_REG8(0x377c), 0x09 },
	{ CCI_REG8(0x377d), 0x02 },
	{ CCI_REG8(0x377e), 0x04 },
	{ CCI_REG8(0x377f), 0x02 },
	{ CCI_REG8(0x379b), 0x0a },
	{ CCI_REG8(0x379c), 0x0a },
	{ CCI_REG8(0x379d), 0x0a },
	{ CCI_REG8(0x379e), 0x20 },
	{ CCI_REG8(0x379f), 0x20 },
	{ CCI_REG8(0x37a0), 0x08 },
	{ CCI_REG8(0x37a1), 0x08 },
	{ CCI_REG8(0x37a2), 0x02 },
	{ CCI_REG8(0x37a3), 0x08 },
	{ CCI_REG8(0x37a4), 0x02 },
	{ CCI_REG8(0x37a5), 0x2f },
	{ CCI_REG8(0x37a6), 0x0a },
	{ CCI_REG8(0x37a7), 0x05 },
	{ CCI_REG8(0x37a8), 0x05 },
	{ CCI_REG8(0x37a9), 0x05 },
	{ CCI_REG8(0x37aa), 0x05 },
	{ CCI_REG8(0x37ab), 0x09 },
	{ CCI_REG8(0x37ac), 0x0d },
	{ CCI_REG8(0x37ad), 0x0d },
	{ CCI_REG8(0x37ae), 0x0d },
	{ CCI_REG8(0x37b0), 0x38 },
	{ CCI_REG8(0x37b1), 0x38 },
	{ CCI_REG8(0x37b2), 0x0e },
	{ CCI_REG8(0x37b3), 0x40 },
	{ CCI_REG8(0x37b4), 0x40 },
	{ CCI_REG8(0x37b5), 0x1c },
	{ CCI_REG8(0x37c0), 0x04 },
	{ CCI_REG8(0x37c1), 0x01 },
	{ CCI_REG8(0x37c2), 0x02 },
	{ CCI_REG8(0x37c3), 0x04 },
	{ CCI_REG8(0x37c4), 0x02 },
	{ CCI_REG8(0x37c5), 0x04 },
	{ CCI_REG8(0x37c9), 0x02 },
	{ CCI_REG8(0x37ce), 0x07 },
	{ CCI_REG8(0x37cf), 0x01 },
	{ CCI_REG8(0x37d0), 0x07 },
	{ CCI_REG8(0x37d1), 0x01 },
	{ CCI_REG8(0x37d2), 0x07 },
	{ CCI_REG8(0x37d3), 0x01 },
	{ CCI_REG8(0x37d4), 0x02 },
	{ CCI_REG8(0x37d5), 0x0a },
	{ CCI_REG8(0x37d6), 0x14 },
	{ CCI_REG8(0x37d7), 0x20 },
	{ CCI_REG8(0x37d8), 0x04 },
	{ CCI_REG8(0x37e0), 0x15 },
	{ CCI_REG8(0x37e1), 0x02 },
	{ CCI_REG8(0x37e2), 0x04 },
	{ CCI_REG8(0x37e3), 0x18 },
	{ CCI_REG8(0x37e4), 0x07 },
	{ CCI_REG8(0x37e5), 0x15 },
	{ CCI_REG8(0x37e6), 0x07 },
	{ CCI_REG8(0x37e7), 0x15 },
	{ CCI_REG8(0x37e8), 0x07 },
	{ CCI_REG8(0x37e9), 0x1a },
	{ CCI_REG8(0x37eb), 0x04 },
	{ CCI_REG8(0x37ec), 0x04 },
	{ CCI_REG8(0x37ed), 0x07 },
	{ CCI_REG8(0x37ee), 0x01 },
	{ CCI_REG8(0x37f0), 0x07 },
	{ CCI_REG8(0x37f1), 0x01 },
	{ CCI_REG8(0x37f2), 0x07 },
	{ CCI_REG8(0x37f3), 0x01 },
	{ CCI_REG8(0x37f6), 0x14 },
	{ CCI_REG8(0x37f7), 0x01 },
	{ CCI_REG8(0x37f8), 0x28 },
	{ CCI_REG8(0x37fd), 0x01 },
	{ CCI_REG8(0x37fe), 0x01 },
	{ CCI_REG8(0x37ff), 0x01 },
	{ CCI_REG8(0x3816), 0x01 },
	{ CCI_REG8(0x3817), 0x10 },
	{ CCI_REG8(0x3818), 0x01 },
	{ CCI_REG8(0x3823), 0x04 },
	{ CCI_REG8(0x382a), 0x01 },
	{ CCI_REG8(0x3f01), 0x12 },
	{ CCI_REG8(0x3d85), 0x0b },
	{ CCI_REG8(0x3d8c), 0x77 },
	{ CCI_REG8(0x3d8d), 0xa0 },
	{ CCI_REG8(0x4009), 0x02 },
	{ CCI_REG8(0x401e), 0x01 },
	{ CCI_REG8(0x401f), 0x80 },
	{ CCI_REG8(0x4021), 0x00 },
	{ CCI_REG8(0x4023), 0x00 },
	{ CCI_REG8(0x4024), 0x04 },
	{ CCI_REG8(0x4025), 0x00 },
	{ CCI_REG8(0x4026), 0x04 },
	{ CCI_REG8(0x4027), 0x00 },
	{ CCI_REG8(0x4028), 0x01 },
	{ CCI_REG8(0x4504), 0x02 },
	{ CCI_REG8(0x4640), 0x00 },
	{ CCI_REG8(0x4643), 0x08 },
	{ CCI_REG8(0x4645), 0x04 },
	{ CCI_REG8(0x4800), 0xe0 },
	{ CCI_REG8(0x4837), 0x0a },
	{ CCI_REG8(0x4850), 0x0d },
	{ CCI_REG8(0x4855), 0x4c },
	{ CCI_REG8(0x4856), 0x0a },
	{ CCI_REG8(0x4a00), 0x10 },
	{ CCI_REG8(0x4d00), 0x03 },
	{ CCI_REG8(0x4d01), 0xb2 },
	{ CCI_REG8(0x4d02), 0xbc },
	{ CCI_REG8(0x4d03), 0x8d },
	{ CCI_REG8(0x4d04), 0x82 },
	{ CCI_REG8(0x4d05), 0xcd },
	{ CCI_REG8(0x5002), 0x11 },
	{ CCI_REG8(0x5038), 0x40 },
	{ CCI_REG8(0x5081), 0x80 },
	{ CCI_REG8(0x5181), 0x10 },
	{ CCI_REG8(0x5182), 0x07 },
	{ CCI_REG8(0x5183), 0x8f },
	{ CCI_REG8(0x5860), 0x00 },
	{ CCI_REG8(0x58cb), 0x03 },
};

/* 2304x1728 (2x2 binned), 30 fps */
static const struct cci_reg_sequence ov16e1q_2304x1728_regs[] = {
	{ CCI_REG8(0x3608), 0xa6 },
	{ CCI_REG8(0x3663), 0x01 },
	{ CCI_REG8(0x3709), 0x9b },
	{ CCI_REG8(0x3712), 0x00 },
	{ CCI_REG8(0x3714), 0x64 },
	{ CCI_REG8(0x374c), 0x00 },
	{ CCI_REG8(0x374d), 0x17 },
	{ CCI_REG8(0x374e), 0x2a },
	{ CCI_REG8(0x3752), 0x00 },
	{ CCI_REG8(0x3753), 0x00 },
	{ CCI_REG8(0x3754), 0xee },
	{ CCI_REG8(0x3781), 0x00 },
	{ CCI_REG8(0x3798), 0x00 },
	{ CCI_REG8(0x37c8), 0x02 },
	{ CCI_REG8(0x37cc), 0xf0 },
	{ CCI_REG8(0x37cd), 0x01 },
	{ CCI_REG8(0x37d9), 0x00 },
	{ CCI_REG8(0x3808), 0x09 },
	{ CCI_REG8(0x3809), 0x00 },
	{ CCI_REG8(0x380a), 0x06 },
	{ CCI_REG8(0x380b), 0xc0 },
	{ CCI_REG8(0x380c), 0x02 },
	{ CCI_REG8(0x380d), 0xd9 },
	{ CCI_REG8(0x380e), 0x0e },
	{ CCI_REG8(0x380f), 0x76 },
	{ CCI_REG8(0x3811), 0x0c },
	{ CCI_REG8(0x3813), 0x02 },
	{ CCI_REG8(0x3814), 0x22 },
	{ CCI_REG8(0x3815), 0x22 },
	{ CCI_REG8(0x3820), 0x05 },
	{ CCI_REG8(0x3821), 0x09 },
	{ CCI_REG8(0x383d), 0x0f },
	{ CCI_REG8(0x4012), 0x3d },
	{ CCI_REG8(0x4015), 0x02 },
	{ CCI_REG8(0x4016), 0x0d },
	{ CCI_REG8(0x4017), 0x04 },
	{ CCI_REG8(0x4018), 0x07 },
	{ CCI_REG8(0x4057), 0x10 },
	{ CCI_REG8(0x4510), 0x18 },
	{ CCI_REG8(0x5000), 0x09 },
	{ CCI_REG8(0x5001), 0x42 },
	{ CCI_REG8(0x3501), 0x07 },
	{ CCI_REG8(0x3502), 0x2e },
	{ CCI_REG8(0x3508), 0x02 },
	{ CCI_REG8(0x3509), 0x00 },
};

static const struct ov16e1q_mode ov16e1q_modes[] = {
	{
		.width = 2304,
		.height = 1728,
		.hts = 5446,
		.vts_def = 3702,
		.exposure_def = 1838,
		.crop = {
			.left = OV16E1Q_PIXEL_ARRAY_LEFT,
			.top = OV16E1Q_PIXEL_ARRAY_TOP,
			.width = OV16E1Q_PIXEL_ARRAY_WIDTH,
			.height = OV16E1Q_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.regs = ov16e1q_2304x1728_regs,
			.num_regs = ARRAY_SIZE(ov16e1q_2304x1728_regs),
		},
	},
};

static int ov16e1q_update_exposure_range(struct ov16e1q *ov16e1q, s32 vblank)
{
	s64 exposure_max;

	exposure_max = min_t(s64, ov16e1q->mode->height + vblank -
			     OV16E1Q_EXPOSURE_MARGIN, OV16E1Q_EXPOSURE_MAX);

	return __v4l2_ctrl_modify_range(ov16e1q->exposure,
					ov16e1q->exposure->minimum,
					exposure_max,
					ov16e1q->exposure->step,
					min_t(s64, ov16e1q->mode->exposure_def,
					      exposure_max));
}

static int ov16e1q_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov16e1q *ov16e1q = container_of(ctrl->handler, struct ov16e1q,
					       ctrl_handler);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		ret = ov16e1q_update_exposure_range(ov16e1q, ctrl->val);
		if (ret)
			return ret;
	}

	/* Controls are applied when the sensor is powered up for streaming */
	if (!pm_runtime_get_if_active(ov16e1q->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(ov16e1q->regmap, OV16E1Q_REG_EXPOSURE, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(ov16e1q->regmap, OV16E1Q_REG_ANALOGUE_GAIN,
			  ctrl->val & OV16E1Q_ANALOGUE_GAIN_MASK, &ret);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		cci_write(ov16e1q->regmap, OV16E1Q_REG_DIGITAL_GAIN,
			  ctrl->val << OV16E1Q_DIGITAL_GAIN_SHIFT, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(ov16e1q->regmap, OV16E1Q_REG_VTS,
			  ov16e1q->mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(ov16e1q->regmap, OV16E1Q_REG_TEST_PATTERN,
			  ctrl->val ? OV16E1Q_TEST_PATTERN_SOLID :
				      OV16E1Q_TEST_PATTERN_DISABLE, &ret);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		cci_write(ov16e1q->regmap, OV16E1Q_REG_TP_RED, ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		cci_write(ov16e1q->regmap, OV16E1Q_REG_TP_GREENR, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		cci_write(ov16e1q->regmap, OV16E1Q_REG_TP_GREENB, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		cci_write(ov16e1q->regmap, OV16E1Q_REG_TP_BLUE, ctrl->val,
			  &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(ov16e1q->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov16e1q_ctrl_ops = {
	.s_ctrl = ov16e1q_set_ctrl,
};

static int ov16e1q_init_controls(struct ov16e1q *ov16e1q)
{
	static const u32 tp_colour_ctrls[] = {
		V4L2_CID_TEST_PATTERN_RED,
		V4L2_CID_TEST_PATTERN_GREENR,
		V4L2_CID_TEST_PATTERN_GREENB,
		V4L2_CID_TEST_PATTERN_BLUE,
	};
	struct v4l2_ctrl_handler *hdl = &ov16e1q->ctrl_handler;
	const struct ov16e1q_mode *mode = ov16e1q->mode;
	struct v4l2_fwnode_device_properties props;
	s64 hblank, vblank, exposure_max;
	struct v4l2_ctrl *ctrl;
	unsigned int i;
	int ret;

	ret = v4l2_fwnode_device_parse(ov16e1q->dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 14);

	ctrl = v4l2_ctrl_new_int_menu(hdl, &ov16e1q_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(ov16e1q_link_freq_menu) - 1,
				      0, ov16e1q_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, &ov16e1q_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  OV16E1Q_PIXEL_RATE, OV16E1Q_PIXEL_RATE, 1,
			  OV16E1Q_PIXEL_RATE);

	hblank = mode->hts - mode->width;
	ov16e1q->hblank = v4l2_ctrl_new_std(hdl, &ov16e1q_ctrl_ops,
					    V4L2_CID_HBLANK, hblank, hblank, 1,
					    hblank);
	if (ov16e1q->hblank)
		ov16e1q->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* The frame length must be even: step by 2 from an even minimum */
	vblank = mode->vts_def - mode->height;
	ov16e1q->vblank = v4l2_ctrl_new_std(hdl, &ov16e1q_ctrl_ops,
					    V4L2_CID_VBLANK, vblank,
					    OV16E1Q_VTS_MAX - mode->height, 2,
					    vblank);

	exposure_max = mode->vts_def - OV16E1Q_EXPOSURE_MARGIN;
	ov16e1q->exposure = v4l2_ctrl_new_std(hdl, &ov16e1q_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      OV16E1Q_EXPOSURE_MIN,
					      exposure_max,
					      OV16E1Q_EXPOSURE_STEP,
					      mode->exposure_def);

	v4l2_ctrl_new_std(hdl, &ov16e1q_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OV16E1Q_ANALOGUE_GAIN_MIN, OV16E1Q_ANALOGUE_GAIN_MAX,
			  OV16E1Q_ANALOGUE_GAIN_STEP,
			  OV16E1Q_ANALOGUE_GAIN_DEFAULT);

	v4l2_ctrl_new_std(hdl, &ov16e1q_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  OV16E1Q_DIGITAL_GAIN_MIN, OV16E1Q_DIGITAL_GAIN_MAX,
			  OV16E1Q_DIGITAL_GAIN_STEP,
			  OV16E1Q_DIGITAL_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(hdl, &ov16e1q_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ov16e1q_test_pattern_menu) - 1,
				     0, 0, ov16e1q_test_pattern_menu);

	for (i = 0; i < ARRAY_SIZE(tp_colour_ctrls); i++)
		v4l2_ctrl_new_std(hdl, &ov16e1q_ctrl_ops, tp_colour_ctrls[i],
				  0, OV16E1Q_TP_COLOUR_MAX, 1, 0);

	v4l2_ctrl_new_fwnode_properties(hdl, &ov16e1q_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	ov16e1q->sd.ctrl_handler = hdl;

	return 0;
}

static void ov16e1q_update_pad_format(const struct ov16e1q_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = OV16E1Q_MBUS_CODE;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int ov16e1q_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct ov16e1q *ov16e1q = to_ov16e1q(sd);
	const struct ov16e1q_reg_list *reg_list = &ov16e1q->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(ov16e1q->dev);
	if (ret)
		return ret;

	/* The init sequence starts with a software reset, no delay needed */
	ret = cci_multi_reg_write(ov16e1q->regmap, ov16e1q_init_regs,
				  ARRAY_SIZE(ov16e1q_init_regs), NULL);
	if (ret) {
		dev_err(ov16e1q->dev, "failed to write init registers: %d\n",
			ret);
		goto error;
	}

	ret = cci_multi_reg_write(ov16e1q->regmap, reg_list->regs,
				  reg_list->num_regs, NULL);
	if (ret) {
		dev_err(ov16e1q->dev, "failed to set mode: %d\n", ret);
		goto error;
	}

	ret = __v4l2_ctrl_handler_setup(ov16e1q->sd.ctrl_handler);
	if (ret)
		goto error;

	ret = cci_write(ov16e1q->regmap, OV16E1Q_REG_MODE_SELECT,
			OV16E1Q_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(ov16e1q->dev, "failed to start streaming: %d\n", ret);
		goto error;
	}

	return 0;

error:
	pm_runtime_put_autosuspend(ov16e1q->dev);

	return ret;
}

static int ov16e1q_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct ov16e1q *ov16e1q = to_ov16e1q(sd);
	int ret;

	ret = cci_write(ov16e1q->regmap, OV16E1Q_REG_MODE_SELECT,
			OV16E1Q_MODE_STANDBY, NULL);
	if (ret)
		dev_err(ov16e1q->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(ov16e1q->dev);

	return ret;
}

static int ov16e1q_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	struct ov16e1q *ov16e1q = to_ov16e1q(sd);
	const struct ov16e1q_mode *mode;
	s64 hblank, vblank;
	int ret;

	mode = v4l2_find_nearest_size(ov16e1q_modes, ARRAY_SIZE(ov16e1q_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);

	ov16e1q_update_pad_format(mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	ov16e1q->mode = mode;

	hblank = mode->hts - mode->width;
	ret = __v4l2_ctrl_modify_range(ov16e1q->hblank, hblank, hblank, 1,
				       hblank);
	if (ret)
		return ret;

	vblank = mode->vts_def - mode->height;
	ret = __v4l2_ctrl_modify_range(ov16e1q->vblank, vblank,
				       OV16E1Q_VTS_MAX - mode->height, 2,
				       vblank);
	if (ret)
		return ret;

	return __v4l2_ctrl_s_ctrl(ov16e1q->vblank, vblank);
}

static int ov16e1q_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OV16E1Q_NATIVE_WIDTH;
		sel->r.height = OV16E1Q_NATIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = OV16E1Q_PIXEL_ARRAY_LEFT;
		sel->r.top = OV16E1Q_PIXEL_ARRAY_TOP;
		sel->r.width = OV16E1Q_PIXEL_ARRAY_WIDTH;
		sel->r.height = OV16E1Q_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int ov16e1q_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = OV16E1Q_MBUS_CODE;

	return 0;
}

static int ov16e1q_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(ov16e1q_modes))
		return -EINVAL;

	if (fse->code != OV16E1Q_MBUS_CODE)
		return -EINVAL;

	fse->min_width = ov16e1q_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ov16e1q_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ov16e1q_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = OV16E1Q_MBUS_CODE,
			.width = ov16e1q_modes[0].width,
			.height = ov16e1q_modes[0].height,
		},
	};

	return ov16e1q_set_pad_format(sd, state, &fmt);
}

static const struct v4l2_subdev_video_ops ov16e1q_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops ov16e1q_pad_ops = {
	.enum_mbus_code = ov16e1q_enum_mbus_code,
	.enum_frame_size = ov16e1q_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ov16e1q_set_pad_format,
	.get_selection = ov16e1q_get_selection,
	.enable_streams = ov16e1q_enable_streams,
	.disable_streams = ov16e1q_disable_streams,
};

static const struct v4l2_subdev_ops ov16e1q_subdev_ops = {
	.video = &ov16e1q_video_ops,
	.pad = &ov16e1q_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov16e1q_internal_ops = {
	.init_state = ov16e1q_init_state,
};

static const struct media_entity_operations ov16e1q_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ov16e1q_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov16e1q *ov16e1q = to_ov16e1q(sd);
	int ret;

	ret = regulator_bulk_enable(OV16E1Q_NUM_SUPPLIES, ov16e1q->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	if (ov16e1q->dvdd) {
		ret = regulator_enable(ov16e1q->dvdd);
		if (ret) {
			dev_err(dev, "failed to enable dvdd: %d\n", ret);
			goto disable_supplies;
		}
	}

	ret = clk_prepare_enable(ov16e1q->xclk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		goto disable_dvdd;
	}

	/* The reference stack releases reset 2 ms after starting XCLK */
	fsleep(2000);

	gpiod_set_value_cansleep(ov16e1q->reset_gpio, 0);

	/* Wait before the first SCCB transaction, 5 ms in the reference stack */
	fsleep(6000);

	return 0;

disable_dvdd:
	if (ov16e1q->dvdd)
		regulator_disable(ov16e1q->dvdd);
disable_supplies:
	regulator_bulk_disable(OV16E1Q_NUM_SUPPLIES, ov16e1q->supplies);

	return ret;
}

static int ov16e1q_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov16e1q *ov16e1q = to_ov16e1q(sd);

	clk_disable_unprepare(ov16e1q->xclk);
	fsleep(1000);

	gpiod_set_value_cansleep(ov16e1q->reset_gpio, 1);
	fsleep(1000);

	if (ov16e1q->dvdd)
		regulator_disable(ov16e1q->dvdd);

	regulator_bulk_disable(OV16E1Q_NUM_SUPPLIES, ov16e1q->supplies);

	return 0;
}

static int ov16e1q_identify_sensor(struct ov16e1q *ov16e1q)
{
	u64 val;
	int ret;

	ret = cci_read(ov16e1q->regmap, OV16E1Q_REG_CHIP_ID, &val, NULL);
	if (ret)
		return dev_err_probe(ov16e1q->dev, ret,
				     "failed to read chip id\n");

	if ((val & OV16E1Q_CHIP_ID_MASK) != OV16E1Q_CHIP_ID)
		return dev_err_probe(ov16e1q->dev, -ENODEV,
				     "chip id mismatch: %x!=%llx\n",
				     OV16E1Q_CHIP_ID,
				     val & OV16E1Q_CHIP_ID_MASK);

	dev_dbg(ov16e1q->dev, "chip id 0x%06llx\n", val);

	return 0;
}

static int ov16e1q_check_hwcfg(struct ov16e1q *ov16e1q)
{
	struct fwnode_handle *fwnode = dev_fwnode(ov16e1q->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned long freq_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_endpoint_by_id(fwnode, 0, 0, 0);
	if (!ep)
		return dev_err_probe(ov16e1q->dev, -EINVAL,
				     "missing endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(ov16e1q->dev, ret,
				     "failed to parse endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != OV16E1Q_NUM_DATA_LANES) {
		ret = dev_err_probe(ov16e1q->dev, -EINVAL,
				    "only %u data lanes are supported, got %u\n",
				    OV16E1Q_NUM_DATA_LANES,
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto out_free;
	}

	ret = v4l2_link_freq_to_bitmap(ov16e1q->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       ov16e1q_link_freq_menu,
				       ARRAY_SIZE(ov16e1q_link_freq_menu),
				       &freq_bitmap);

out_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int ov16e1q_get_regulators(struct ov16e1q *ov16e1q)
{
	unsigned int i;
	int ret;

	for (i = 0; i < OV16E1Q_NUM_SUPPLIES; i++)
		ov16e1q->supplies[i].supply = ov16e1q_supply_names[i];

	ret = devm_regulator_bulk_get(ov16e1q->dev, OV16E1Q_NUM_SUPPLIES,
				      ov16e1q->supplies);
	if (ret)
		return dev_err_probe(ov16e1q->dev, ret,
				     "failed to get supplies\n");

	/* DVDD is normally generated by the internal regulator */
	ov16e1q->dvdd = devm_regulator_get_optional(ov16e1q->dev, "dvdd");
	if (IS_ERR(ov16e1q->dvdd)) {
		ret = PTR_ERR(ov16e1q->dvdd);
		if (ret != -ENODEV)
			return dev_err_probe(ov16e1q->dev, ret,
					     "failed to get dvdd supply\n");
		ov16e1q->dvdd = NULL;
	}

	return 0;
}

static int ov16e1q_probe(struct i2c_client *client)
{
	struct ov16e1q *ov16e1q;
	unsigned long freq;
	int ret;

	ov16e1q = devm_kzalloc(&client->dev, sizeof(*ov16e1q), GFP_KERNEL);
	if (!ov16e1q)
		return -ENOMEM;

	ov16e1q->dev = &client->dev;
	ov16e1q->mode = &ov16e1q_modes[0];

	v4l2_i2c_subdev_init(&ov16e1q->sd, client, &ov16e1q_subdev_ops);

	ret = ov16e1q_check_hwcfg(ov16e1q);
	if (ret)
		return ret;

	ov16e1q->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ov16e1q->regmap))
		return dev_err_probe(ov16e1q->dev, PTR_ERR(ov16e1q->regmap),
				     "failed to init CCI\n");

	ov16e1q->xclk = devm_v4l2_sensor_clk_get(ov16e1q->dev, NULL);
	if (IS_ERR(ov16e1q->xclk))
		return dev_err_probe(ov16e1q->dev, PTR_ERR(ov16e1q->xclk),
				     "failed to get clock\n");

	freq = clk_get_rate(ov16e1q->xclk);
	if (freq != OV16E1Q_XCLK_FREQ)
		return dev_err_probe(ov16e1q->dev, -EINVAL,
				     "clock frequency %lu Hz is not supported\n",
				     freq);

	ov16e1q->reset_gpio = devm_gpiod_get_optional(ov16e1q->dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(ov16e1q->reset_gpio))
		return dev_err_probe(ov16e1q->dev,
				     PTR_ERR(ov16e1q->reset_gpio),
				     "failed to get reset GPIO\n");

	ret = ov16e1q_get_regulators(ov16e1q);
	if (ret)
		return ret;

	/* The sensor must be powered on to read the chip ID register */
	ret = ov16e1q_power_on(ov16e1q->dev);
	if (ret)
		return ret;

	ret = ov16e1q_identify_sensor(ov16e1q);
	if (ret)
		goto power_off;

	ret = ov16e1q_init_controls(ov16e1q);
	if (ret) {
		dev_err_probe(ov16e1q->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	ov16e1q->sd.state_lock = ov16e1q->ctrl_handler.lock;
	ov16e1q->sd.internal_ops = &ov16e1q_internal_ops;
	ov16e1q->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov16e1q->sd.entity.ops = &ov16e1q_entity_ops;
	ov16e1q->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ov16e1q->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&ov16e1q->sd.entity, 1, &ov16e1q->pad);
	if (ret) {
		dev_err_probe(ov16e1q->dev, ret, "failed to init entity pads\n");
		goto free_ctrls;
	}

	ret = v4l2_subdev_init_finalize(&ov16e1q->sd);
	if (ret) {
		dev_err_probe(ov16e1q->dev, ret, "failed to init subdev\n");
		goto cleanup_entity;
	}

	pm_runtime_set_active(ov16e1q->dev);
	pm_runtime_get_noresume(ov16e1q->dev);
	pm_runtime_enable(ov16e1q->dev);

	ret = v4l2_async_register_subdev_sensor(&ov16e1q->sd);
	if (ret) {
		dev_err_probe(ov16e1q->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto disable_pm;
	}

	pm_runtime_set_autosuspend_delay(ov16e1q->dev, 1000);
	pm_runtime_use_autosuspend(ov16e1q->dev);
	pm_runtime_put_autosuspend(ov16e1q->dev);

	return 0;

disable_pm:
	pm_runtime_disable(ov16e1q->dev);
	pm_runtime_set_suspended(ov16e1q->dev);
	pm_runtime_put_noidle(ov16e1q->dev);
	v4l2_subdev_cleanup(&ov16e1q->sd);
cleanup_entity:
	media_entity_cleanup(&ov16e1q->sd.entity);
free_ctrls:
	v4l2_ctrl_handler_free(&ov16e1q->ctrl_handler);
power_off:
	ov16e1q_power_off(ov16e1q->dev);

	return ret;
}

static void ov16e1q_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov16e1q *ov16e1q = to_ov16e1q(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&ov16e1q->ctrl_handler);

	pm_runtime_disable(ov16e1q->dev);
	if (!pm_runtime_status_suspended(ov16e1q->dev)) {
		ov16e1q_power_off(ov16e1q->dev);
		pm_runtime_set_suspended(ov16e1q->dev);
	}
	pm_runtime_dont_use_autosuspend(ov16e1q->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(ov16e1q_pm_ops, ov16e1q_power_off,
				 ov16e1q_power_on, NULL);

static const struct of_device_id ov16e1q_of_match[] = {
	{ .compatible = "ovti,ov16e1q" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov16e1q_of_match);

static struct i2c_driver ov16e1q_i2c_driver = {
	.driver = {
		.name = "ov16e1q",
		.pm = pm_ptr(&ov16e1q_pm_ops),
		.of_match_table = ov16e1q_of_match,
	},
	.probe = ov16e1q_probe,
	.remove = ov16e1q_remove,
};
module_i2c_driver(ov16e1q_i2c_driver);

MODULE_AUTHOR("Luka Panio <lukapanio@gmail.com>");
MODULE_DESCRIPTION("OmniVision OV16E1Q image sensor driver");
MODULE_LICENSE("GPL");
