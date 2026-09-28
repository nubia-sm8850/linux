// SPDX-License-Identifier: GPL-2.0-only
/*
 * OmniVision OV50H40 image sensor driver
 *
 * The OV50H40 is a 50 Mpixel (8192x6144 active, 1.2 um) sensor with a
 * three-trio MIPI CSI-2 C-PHY interface. Register settings were captured
 * from a working vendor configuration (24 MHz external clock, 7.65 Gbps
 * per trio); no datasheet was available.
 *
 * TODO: the vendor stack writes 3584 bytes of module specific quad
 * sensitivity correction (QSC) data, read from the module EEPROM at offset
 * 0x2200, to register 0x5a20 after the common settings. Images are usable
 * without it, but less uniform. It could be loaded through an nvmem cell
 * of the EEPROM or from a firmware file.
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

#define OV50H40_XCLK_FREQ		(24 * HZ_PER_MHZ)
#define OV50H40_NUM_DATA_LANES		3
#define OV50H40_BPP			10
/*
 * Pixel rate = 7652.67 Mbps per trio * 3 trios / 10 bits per pixel, the same
 * in every mode.
 */
#define OV50H40_PIXEL_RATE		(2295800 * HZ_PER_KHZ)
/*
 * C-PHY link frequency, as defined by tx-rx.rst: half the symbol rate, with
 * 16 bits in 7 symbols. link_freq = pixel_rate * bpp * 7 / (2 * trios * 16).
 */
#define OV50H40_LINK_FREQ		1674020833LL

#define OV50H40_REG_CHIP_ID		CCI_REG24(0x300a)
#define OV50H40_CHIP_ID_MASK		0xffff
#define OV50H40_CHIP_ID			0x5046

/* Group hold: open, close and launch group 0 */
#define OV50H40_REG_GROUP_HOLD		CCI_REG8(0x3208)
#define OV50H40_GROUP0_START		0x00
#define OV50H40_GROUP0_END		0x10
#define OV50H40_GROUP0_LAUNCH		0xa0

/*
 * Frame length and exposure registers count units of 2 lines at full
 * resolution and of 4 lines in the binned modes (mode->shift).
 */
#define OV50H40_REG_VTS_HIGH		CCI_REG8(0x3840)
#define OV50H40_REG_VTS			CCI_REG16(0x380e)
#define OV50H40_VTS_MAX			0xffffff

#define OV50H40_REG_EXPOSURE		CCI_REG24(0x3500)
#define OV50H40_EXPOSURE_MIN		8
/* Frame length minus exposure, in register units */
#define OV50H40_EXPOSURE_MARGIN		38

/* Analogue gain, 1x = 0x100, bit 0 is not used */
#define OV50H40_REG_ANALOGUE_GAIN	CCI_REG16(0x3508)
#define OV50H40_ANALOGUE_GAIN_MASK	0x3ffe
#define OV50H40_ANALOGUE_GAIN_MIN	0x100
#define OV50H40_ANALOGUE_GAIN_MAX	0x3fc0
#define OV50H40_ANALOGUE_GAIN_STEP	1
#define OV50H40_ANALOGUE_GAIN_DEFAULT	0x100

/* Digital gain, 1x = 0x400 in bits [19:6]; kept at 1x */
#define OV50H40_REG_DIGITAL_GAIN	CCI_REG24(0x350a)
#define OV50H40_DIGITAL_GAIN_1X		(0x400 << 6)

/* Test pattern; 0x01 gives vertical bars on the sibling OV16E1Q */
#define OV50H40_REG_TEST_PATTERN	CCI_REG8(0x5081)
#define OV50H40_TEST_PATTERN_DISABLE	0x00
#define OV50H40_TEST_PATTERN_BARS	0x01

/* Pixel array */
#define OV50H40_NATIVE_WIDTH		8192
#define OV50H40_NATIVE_HEIGHT		6144

#define OV50H40_MBUS_CODE		MEDIA_BUS_FMT_SRGGB10_1X10

static const s64 ov50h40_link_freq_menu[] = {
	OV50H40_LINK_FREQ,
};

static const char * const ov50h40_supply_names[] = {
	"dovdd",	/* Digital I/O power */
	"avdd",		/* Analogue power */
	"dvdd",		/* Digital core power */
};

#define OV50H40_NUM_SUPPLIES	ARRAY_SIZE(ov50h40_supply_names)

static const char * const ov50h40_test_pattern_menu[] = {
	"Disabled",
	"Vertical Bars",
};

struct ov50h40_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct ov50h40_mode {
	u32 width;
	u32 height;
	/* Line length in units of the pixel rate */
	u32 hts;
	/* Default and minimum frame length, in register units */
	u32 vts_def;
	u32 vts_min;
	/* Lines per frame length / exposure register unit, log2 */
	unsigned int shift;
	/* Default exposure and exposure step, in lines */
	u32 exposure_def;
	u32 exposure_step;
	/* Analogue crop rectangle, in native pixel array coordinates */
	struct v4l2_rect crop;
	struct ov50h40_reg_list reg_list;
	/* Written after the mode table, may be empty */
	struct ov50h40_reg_list post_reg_list;
};

struct ov50h40 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[OV50H40_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *again;

	const struct ov50h40_mode *mode;
	bool streaming;
};

static inline struct ov50h40 *to_ov50h40(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov50h40, sd);
}

/*
 * Common settings: software reset, PLL (24 MHz in, C-PHY 3 trios), MIPI and
 * analogue setup. No delay is needed after the software reset.
 */
static const struct cci_reg_sequence ov50h40_init_regs[] = {
	{ CCI_REG8(0x0103), 0x01 },
	{ CCI_REG8(0x0102), 0x01 },
	{ CCI_REG8(0x0306), 0x03 },
	{ CCI_REG8(0x0307), 0x00 },
	{ CCI_REG8(0x0308), 0x03 },
	{ CCI_REG8(0x0323), 0x12 },
	{ CCI_REG8(0x0324), 0x02 },
	{ CCI_REG8(0x0325), 0x58 },
	{ CCI_REG8(0x0328), 0x9f },
	{ CCI_REG8(0x0329), 0x01 },
	{ CCI_REG8(0x032a), 0x0f },
	{ CCI_REG8(0x032b), 0x09 },
	{ CCI_REG8(0x032e), 0x01 },
	{ CCI_REG8(0x0346), 0xdf },
	{ CCI_REG8(0x0347), 0x0f },
	{ CCI_REG8(0x0348), 0x7f },
	{ CCI_REG8(0x0349), 0x0f },
	{ CCI_REG8(0x034c), 0x03 },
	{ CCI_REG8(0x034d), 0x01 },
	{ CCI_REG8(0x034e), 0x01 },
	{ CCI_REG8(0x300d), 0x11 },
	{ CCI_REG8(0x300d), 0x11 },
	{ CCI_REG8(0x300e), 0x11 },
	{ CCI_REG8(0x3012), 0x31 },
	{ CCI_REG8(0x301c), 0x01 },
	{ CCI_REG8(0x301d), 0x02 },
	{ CCI_REG8(0x3025), 0x03 },
	{ CCI_REG8(0x3026), 0x80 },
	{ CCI_REG8(0x3027), 0x00 },
	{ CCI_REG8(0x3038), 0x02 },
	{ CCI_REG8(0x3102), 0x0d },
	{ CCI_REG8(0x3106), 0x80 },
	{ CCI_REG8(0x3400), 0x0c },
	{ CCI_REG8(0x3408), 0x08 },
	{ CCI_REG8(0x340a), 0x03 },
	{ CCI_REG8(0x340e), 0x60 },
	{ CCI_REG8(0x3420), 0x03 },
	{ CCI_REG8(0x3421), 0x08 },
	{ CCI_REG8(0x3422), 0x08 },
	{ CCI_REG8(0x3426), 0x15 },
	{ CCI_REG8(0x342b), 0x40 },
	{ CCI_REG8(0x342c), 0x15 },
	{ CCI_REG8(0x342d), 0x01 },
	{ CCI_REG8(0x3504), 0x4c },
	{ CCI_REG8(0x3544), 0x4c },
	{ CCI_REG8(0x3584), 0x4c },
	{ CCI_REG8(0x3600), 0xe4 },
	{ CCI_REG8(0x3602), 0xe4 },
	{ CCI_REG8(0x3603), 0x80 },
	{ CCI_REG8(0x3605), 0x38 },
	{ CCI_REG8(0x3607), 0x10 },
	{ CCI_REG8(0x3609), 0x80 },
	{ CCI_REG8(0x360a), 0xfa },
	{ CCI_REG8(0x360b), 0xc7 },
	{ CCI_REG8(0x360d), 0xf4 },
	{ CCI_REG8(0x360e), 0x2b },
	{ CCI_REG8(0x3614), 0x0c },
	{ CCI_REG8(0x3616), 0x8c },
	{ CCI_REG8(0x3617), 0x0d },
	{ CCI_REG8(0x3618), 0xcf },
	{ CCI_REG8(0x361b), 0x04 },
	{ CCI_REG8(0x3622), 0x00 },
	{ CCI_REG8(0x3627), 0xa0 },
	{ CCI_REG8(0x3640), 0x00 },
	{ CCI_REG8(0x3641), 0x02 },
	{ CCI_REG8(0x3643), 0x01 },
	{ CCI_REG8(0x3645), 0x06 },
	{ CCI_REG8(0x3646), 0x40 },
	{ CCI_REG8(0x3648), 0x8e },
	{ CCI_REG8(0x364d), 0x10 },
	{ CCI_REG8(0x3650), 0xbf },
	{ CCI_REG8(0x3651), 0x00 },
	{ CCI_REG8(0x3657), 0x40 },
	{ CCI_REG8(0x3688), 0x00 },
	{ CCI_REG8(0x3702), 0x30 },
	{ CCI_REG8(0x3703), 0x34 },
	{ CCI_REG8(0x3716), 0x40 },
	{ CCI_REG8(0x372e), 0x1c },
	{ CCI_REG8(0x374b), 0x04 },
	{ CCI_REG8(0x374c), 0x0c },
	{ CCI_REG8(0x3754), 0x30 },
	{ CCI_REG8(0x3760), 0x10 },
	{ CCI_REG8(0x3761), 0x30 },
	{ CCI_REG8(0x3762), 0x10 },
	{ CCI_REG8(0x3763), 0x10 },
	{ CCI_REG8(0x3765), 0x20 },
	{ CCI_REG8(0x3766), 0x30 },
	{ CCI_REG8(0x3767), 0x20 },
	{ CCI_REG8(0x3769), 0x10 },
	{ CCI_REG8(0x376a), 0x10 },
	{ CCI_REG8(0x379d), 0x01 },
	{ CCI_REG8(0x379f), 0x01 },
	{ CCI_REG8(0x37ca), 0xc4 },
	{ CCI_REG8(0x37cb), 0x02 },
	{ CCI_REG8(0x37db), 0x10 },
	{ CCI_REG8(0x37dc), 0x1a },
	{ CCI_REG8(0x37e0), 0x68 },
	{ CCI_REG8(0x381f), 0x04 },
	{ CCI_REG8(0x382a), 0x80 },
	{ CCI_REG8(0x382e), 0x49 },
	{ CCI_REG8(0x3831), 0x10 },
	{ CCI_REG8(0x383f), 0x08 },
	{ CCI_REG8(0x3858), 0x00 },
	{ CCI_REG8(0x3894), 0x02 },
	{ CCI_REG8(0x3895), 0x81 },
	{ CCI_REG8(0x38de), 0x0c },
	{ CCI_REG8(0x3913), 0x0c },
	{ CCI_REG8(0x395b), 0x05 },
	{ CCI_REG8(0x3982), 0x40 },
	{ CCI_REG8(0x3994), 0x0b },
	{ CCI_REG8(0x3995), 0x30 },
	{ CCI_REG8(0x39a0), 0x0b },
	{ CCI_REG8(0x3a1d), 0x01 },
	{ CCI_REG8(0x3a1e), 0x01 },
	{ CCI_REG8(0x3a1f), 0x03 },
	{ CCI_REG8(0x3a21), 0x01 },
	{ CCI_REG8(0x3a23), 0x03 },
	{ CCI_REG8(0x3a78), 0x03 },
	{ CCI_REG8(0x3a79), 0x03 },
	{ CCI_REG8(0x3a7c), 0x04 },
	{ CCI_REG8(0x3a7d), 0x04 },
	{ CCI_REG8(0x3ab5), 0x00 },
	{ CCI_REG8(0x3ab8), 0x01 },
	{ CCI_REG8(0x3ab9), 0x01 },
	{ CCI_REG8(0x3ba4), 0x77 },
	{ CCI_REG8(0x3ba5), 0x77 },
	{ CCI_REG8(0x3bac), 0x77 },
	{ CCI_REG8(0x3be0), 0x30 },
	{ CCI_REG8(0x3be7), 0x08 },
	{ CCI_REG8(0x3be8), 0x0f },
	{ CCI_REG8(0x3d8c), 0x9b },
	{ CCI_REG8(0x3d8d), 0xa0 },
	{ CCI_REG8(0x4009), 0x02 },
	{ CCI_REG8(0x4017), 0x00 },
	{ CCI_REG8(0x401b), 0x04 },
	{ CCI_REG8(0x40f8), 0x04 },
	{ CCI_REG8(0x40f9), 0x00 },
	{ CCI_REG8(0x40fa), 0x02 },
	{ CCI_REG8(0x40fb), 0x00 },
	{ CCI_REG8(0x4288), 0x27 },
	{ CCI_REG8(0x4546), 0x07 },
	{ CCI_REG8(0x4547), 0x33 },
	{ CCI_REG8(0x45c0), 0x9c },
	{ CCI_REG8(0x45c1), 0x80 },
	{ CCI_REG8(0x45c2), 0x0a },
	{ CCI_REG8(0x45c3), 0x04 },
	{ CCI_REG8(0x45c4), 0x13 },
	{ CCI_REG8(0x45c5), 0x80 },
	{ CCI_REG8(0x45c6), 0x08 },
	{ CCI_REG8(0x45cd), 0x07 },
	{ CCI_REG8(0x45ce), 0x13 },
	{ CCI_REG8(0x45cf), 0x13 },
	{ CCI_REG8(0x45d0), 0x13 },
	{ CCI_REG8(0x4603), 0x15 },
	{ CCI_REG8(0x4640), 0x01 },
	{ CCI_REG8(0x4641), 0x00 },
	{ CCI_REG8(0x4643), 0x08 },
	{ CCI_REG8(0x4644), 0xe0 },
	{ CCI_REG8(0x4645), 0xbf },
	{ CCI_REG8(0x4680), 0x11 },
	{ CCI_REG8(0x4684), 0x2b },
	{ CCI_REG8(0x468f), 0x20 },
	{ CCI_REG8(0x4800), 0x64 },
	{ CCI_REG8(0x4802), 0x02 },
	{ CCI_REG8(0x481b), 0x25 },
	{ CCI_REG8(0x4840), 0x00 },
	{ CCI_REG8(0x4860), 0x01 },
	{ CCI_REG8(0x4862), 0x3a },
	{ CCI_REG8(0x4884), 0x11 },
	{ CCI_REG8(0x4888), 0x10 },
	{ CCI_REG8(0x49f8), 0x04 },
	{ CCI_REG8(0x49f9), 0x00 },
	{ CCI_REG8(0x49fa), 0x02 },
	{ CCI_REG8(0x49fb), 0x00 },
	{ CCI_REG8(0x4af8), 0x04 },
	{ CCI_REG8(0x4af9), 0x00 },
	{ CCI_REG8(0x4afa), 0x02 },
	{ CCI_REG8(0x4afb), 0x00 },
	{ CCI_REG8(0x4d00), 0x04 },
	{ CCI_REG8(0x4d01), 0x9d },
	{ CCI_REG8(0x4d02), 0xbb },
	{ CCI_REG8(0x4d03), 0x6c },
	{ CCI_REG8(0x4d04), 0xc4 },
	{ CCI_REG8(0x4d05), 0x71 },
	{ CCI_REG8(0x5007), 0x06 },
	{ CCI_REG8(0x5060), 0x10 },
	{ CCI_REG8(0x5110), 0x90 },
	{ CCI_REG8(0x5111), 0x14 },
	{ CCI_REG8(0x5112), 0x9b },
	{ CCI_REG8(0x5113), 0x27 },
	{ CCI_REG8(0x5114), 0x01 },
	{ CCI_REG8(0x5155), 0x08 },
	{ CCI_REG8(0x5156), 0x0c },
	{ CCI_REG8(0x5157), 0x0c },
	{ CCI_REG8(0x5159), 0x08 },
	{ CCI_REG8(0x515a), 0x0c },
	{ CCI_REG8(0x515b), 0x0c },
	{ CCI_REG8(0x518a), 0x04 },
	{ CCI_REG8(0x5410), 0x90 },
	{ CCI_REG8(0x5411), 0x14 },
	{ CCI_REG8(0x5412), 0x9b },
	{ CCI_REG8(0x5413), 0x27 },
	{ CCI_REG8(0x5455), 0x08 },
	{ CCI_REG8(0x5456), 0x0c },
	{ CCI_REG8(0x5457), 0x0c },
	{ CCI_REG8(0x5459), 0x08 },
	{ CCI_REG8(0x545a), 0x0c },
	{ CCI_REG8(0x545b), 0x0c },
	{ CCI_REG8(0x548a), 0x04 },
	{ CCI_REG8(0x5710), 0x90 },
	{ CCI_REG8(0x5711), 0x14 },
	{ CCI_REG8(0x5712), 0x9b },
	{ CCI_REG8(0x5713), 0x27 },
	{ CCI_REG8(0x5755), 0x08 },
	{ CCI_REG8(0x5756), 0x0c },
	{ CCI_REG8(0x5757), 0x0c },
	{ CCI_REG8(0x5759), 0x08 },
	{ CCI_REG8(0x575a), 0x0c },
	{ CCI_REG8(0x575b), 0x0c },
	{ CCI_REG8(0x578a), 0x04 },
	{ CCI_REG8(0x5853), 0xfe },
	{ CCI_REG8(0x5854), 0xfe },
	{ CCI_REG8(0x5855), 0xfe },
	{ CCI_REG8(0x5856), 0xff },
	{ CCI_REG8(0x5857), 0xff },
	{ CCI_REG8(0x5858), 0xff },
	{ CCI_REG8(0x587b), 0x16 },
	{ CCI_REG8(0x58a7), 0x11 },
	{ CCI_REG8(0x58c0), 0x3f },
	{ CCI_REG8(0x58fd), 0x0a },
	{ CCI_REG8(0x5925), 0x00 },
	{ CCI_REG8(0x5926), 0x00 },
	{ CCI_REG8(0x5927), 0x00 },
	{ CCI_REG8(0x5928), 0x00 },
	{ CCI_REG8(0x5929), 0x00 },
	{ CCI_REG8(0x592c), 0x06 },
	{ CCI_REG8(0x592d), 0x00 },
	{ CCI_REG8(0x592e), 0x03 },
	{ CCI_REG8(0x59c3), 0xce },
	{ CCI_REG8(0x59c4), 0x01 },
	{ CCI_REG8(0x59c5), 0x20 },
	{ CCI_REG8(0x59c7), 0x91 },
	{ CCI_REG8(0x59c8), 0x02 },
	{ CCI_REG8(0x59c9), 0x2f },
	{ CCI_REG8(0x59ca), 0x03 },
	{ CCI_REG8(0x59cb), 0x0a },
	{ CCI_REG8(0x59cc), 0x04 },
	{ CCI_REG8(0x59cd), 0x3d },
	{ CCI_REG8(0x59ce), 0x05 },
	{ CCI_REG8(0x59cf), 0xe8 },
	{ CCI_REG8(0x59d0), 0x08 },
	{ CCI_REG8(0x59d1), 0x3c },
	{ CCI_REG8(0x59d2), 0x0b },
	{ CCI_REG8(0x59d3), 0x7a },
	{ CCI_REG8(0x59d4), 0x0f },
	{ CCI_REG8(0x59d5), 0xff },
	{ CCI_REG8(0x59d6), 0x0f },
	{ CCI_REG8(0x59d7), 0xff },
	{ CCI_REG8(0x59d8), 0x0f },
	{ CCI_REG8(0x59d9), 0xff },
	{ CCI_REG8(0x59da), 0x0f },
	{ CCI_REG8(0x59db), 0xff },
	{ CCI_REG8(0x59ef), 0x5f },
	{ CCI_REG8(0x6924), 0x00 },
	{ CCI_REG8(0x6925), 0x00 },
	{ CCI_REG8(0x6926), 0x00 },
	{ CCI_REG8(0x6942), 0x00 },
	{ CCI_REG8(0x6943), 0x00 },
	{ CCI_REG8(0x6944), 0x00 },
	{ CCI_REG8(0x694b), 0x00 },
	{ CCI_REG8(0x6a20), 0x03 },
	{ CCI_REG8(0x6a21), 0x04 },
	{ CCI_REG8(0x6a22), 0x00 },
	{ CCI_REG8(0x6a53), 0xfe },
	{ CCI_REG8(0x6a54), 0xfe },
	{ CCI_REG8(0x6a55), 0xfe },
	{ CCI_REG8(0x6a56), 0xff },
	{ CCI_REG8(0x6a57), 0xff },
	{ CCI_REG8(0x6a58), 0xff },
	{ CCI_REG8(0x6a7b), 0x16 },
	{ CCI_REG8(0x6aa7), 0x11 },
	{ CCI_REG8(0x6ac0), 0x3f },
	{ CCI_REG8(0x6afd), 0x0a },
	{ CCI_REG8(0x6b25), 0x00 },
	{ CCI_REG8(0x6b26), 0x00 },
	{ CCI_REG8(0x6b27), 0x00 },
	{ CCI_REG8(0x6b28), 0x00 },
	{ CCI_REG8(0x6b29), 0x00 },
	{ CCI_REG8(0x6b2c), 0x06 },
	{ CCI_REG8(0x6b2d), 0x00 },
	{ CCI_REG8(0x6b2e), 0x03 },
	{ CCI_REG8(0x6bc3), 0xce },
	{ CCI_REG8(0x6bc4), 0x01 },
	{ CCI_REG8(0x6bc5), 0x20 },
	{ CCI_REG8(0x6bc7), 0x91 },
	{ CCI_REG8(0x6bc8), 0x02 },
	{ CCI_REG8(0x6bc9), 0x2f },
	{ CCI_REG8(0x6bca), 0x03 },
	{ CCI_REG8(0x6bcb), 0x0a },
	{ CCI_REG8(0x6bcc), 0x04 },
	{ CCI_REG8(0x6bcd), 0x3d },
	{ CCI_REG8(0x6bce), 0x05 },
	{ CCI_REG8(0x6bcf), 0xe8 },
	{ CCI_REG8(0x6bd0), 0x08 },
	{ CCI_REG8(0x6bd1), 0x3c },
	{ CCI_REG8(0x6bd2), 0x0b },
	{ CCI_REG8(0x6bd3), 0x7a },
	{ CCI_REG8(0x6bd4), 0x0f },
	{ CCI_REG8(0x6bd5), 0xff },
	{ CCI_REG8(0x6bd6), 0x0f },
	{ CCI_REG8(0x6bd7), 0xff },
	{ CCI_REG8(0x6bd8), 0x0f },
	{ CCI_REG8(0x6bd9), 0xff },
	{ CCI_REG8(0x6bda), 0x0f },
	{ CCI_REG8(0x6bdb), 0xff },
	{ CCI_REG8(0x6bef), 0x5f },
	{ CCI_REG8(0xc418), 0x02 },
	{ CCI_REG8(0xc44e), 0x03 },
	{ CCI_REG8(0xc44f), 0x03 },
	{ CCI_REG8(0xc450), 0x04 },
	{ CCI_REG8(0xc4bf), 0x01 },
	{ CCI_REG8(0xc4c3), 0x77 },
	{ CCI_REG8(0xc4c4), 0x77 },
	{ CCI_REG8(0xc50b), 0x77 },
	{ CCI_REG8(0xc513), 0x4e },
	{ CCI_REG8(0xc516), 0x16 },
	{ CCI_REG8(0xc532), 0x0e },
	{ CCI_REG8(0xc58b), 0x08 },
	{ CCI_REG8(0xc624), 0xf8 },
	{ CCI_REG8(0xc664), 0x00 },
	{ CCI_REG8(0xc66b), 0x00 },
	{ CCI_REG8(0xc66c), 0x00 },
	{ CCI_REG8(0xc66d), 0x00 },
	{ CCI_REG8(0xc700), 0x80 },
	{ CCI_REG8(0xc72f), 0x08 },
	{ CCI_REG8(0xc739), 0x18 },
	{ CCI_REG8(0xc749), 0x1c },
	{ CCI_REG8(0xc758), 0x40 },
	{ CCI_REG8(0xc765), 0x2a },
	{ CCI_REG8(0xc95a), 0x77 },
	{ CCI_REG8(0xc95b), 0x77 },
	{ CCI_REG8(0xc95c), 0x77 },
	{ CCI_REG8(0xc95d), 0x77 },
	{ CCI_REG8(0xc97b), 0x10 },
	{ CCI_REG8(0xc9be), 0x01 },
	{ CCI_REG8(0xc9fe), 0x0a },
	{ CCI_REG8(0xca01), 0x1a },
	{ CCI_REG8(0xca1a), 0x1a },
	{ CCI_REG8(0xca1b), 0x1a },
	{ CCI_REG8(0xcaab), 0x18 },
};

/*
 * 4096x3072 (2x2 binned), 30 fps. The table opens group 1 (0x3208 = 0x01)
 * and leaves it open; ov50h40_4096x3072_post_regs closes it and prepares
 * the groups that ov50h40_stream_on_regs launches.
 */
static const struct cci_reg_sequence ov50h40_4096x3072_regs[] = {
	{ CCI_REG8(0x3208), 0x01 },
	{ CCI_REG8(0x3bc8), 0x40 },
	{ CCI_REG8(0x0200), 0x11 },
	{ CCI_REG8(0x0201), 0x01 },
	{ CCI_REG8(0x0210), 0x31 },
	{ CCI_REG8(0x0211), 0x33 },
	{ CCI_REG8(0x0212), 0x77 },
	{ CCI_REG8(0x3508), 0x01 },
	{ CCI_REG8(0x3509), 0x00 },
	{ CCI_REG8(0x3548), 0x01 },
	{ CCI_REG8(0x3549), 0x00 },
	{ CCI_REG8(0x3588), 0x01 },
	{ CCI_REG8(0x3589), 0x00 },
	{ CCI_REG8(0x3505), 0x01 },
	{ CCI_REG8(0x3505), 0x00 },
	{ CCI_REG8(0x3545), 0x01 },
	{ CCI_REG8(0x3545), 0x00 },
	{ CCI_REG8(0x3585), 0x01 },
	{ CCI_REG8(0x3585), 0x00 },
	{ CCI_REG8(0x3016), 0xf3 },
	{ CCI_REG8(0x3bc8), 0x08 },
	{ CCI_REG8(0x0210), 0x00 },
	{ CCI_REG8(0x0211), 0x00 },
	{ CCI_REG8(0x0212), 0x00 },
	{ CCI_REG8(0x0200), 0x00 },
	{ CCI_REG8(0x0201), 0x00 },
	{ CCI_REG8(0x3014), 0xf1 },
	{ CCI_REG8(0x3017), 0xf2 },
	{ CCI_REG8(0x301f), 0x9b },
	{ CCI_REG8(0x382e), 0x49 },
	{ CCI_REG8(0x0304), 0x03 },
	{ CCI_REG8(0x0305), 0x45 },
	{ CCI_REG8(0x0327), 0x0e },
	{ CCI_REG8(0x0329), 0x01 },
	{ CCI_REG8(0x032c), 0x00 },
	{ CCI_REG8(0x0344), 0x01 },
	{ CCI_REG8(0x0345), 0x20 },
	{ CCI_REG8(0x0360), 0x09 },
	{ CCI_REG8(0x3027), 0x00 },
	{ CCI_REG8(0x3400), 0x0c },
	{ CCI_REG8(0x3422), 0x08 },
	{ CCI_REG8(0x3423), 0x00 },
	{ CCI_REG8(0x3506), 0xf8 },
	{ CCI_REG8(0x350d), 0x00 },
	{ CCI_REG8(0x350e), 0xb2 },
	{ CCI_REG8(0x350f), 0x40 },
	{ CCI_REG8(0x3546), 0xf8 },
	{ CCI_REG8(0x354d), 0x00 },
	{ CCI_REG8(0x354e), 0xb2 },
	{ CCI_REG8(0x354f), 0x40 },
	{ CCI_REG8(0x3586), 0xf8 },
	{ CCI_REG8(0x358d), 0x00 },
	{ CCI_REG8(0x358e), 0xb2 },
	{ CCI_REG8(0x358f), 0x40 },
	{ CCI_REG8(0x3609), 0x80 },
	{ CCI_REG8(0x360c), 0x4f },
	{ CCI_REG8(0x3610), 0x08 },
	{ CCI_REG8(0x3614), 0x10 },
	{ CCI_REG8(0x3618), 0xcf },
	{ CCI_REG8(0x3619), 0x40 },
	{ CCI_REG8(0x361a), 0x01 },
	{ CCI_REG8(0x361d), 0x1f },
	{ CCI_REG8(0x363b), 0x9f },
	{ CCI_REG8(0x363c), 0x6e },
	{ CCI_REG8(0x3640), 0x00 },
	{ CCI_REG8(0x3641), 0x02 },
	{ CCI_REG8(0x3644), 0x00 },
	{ CCI_REG8(0x3645), 0x06 },
	{ CCI_REG8(0x3647), 0x01 },
	{ CCI_REG8(0x3650), 0xbf },
	{ CCI_REG8(0x3653), 0x03 },
	{ CCI_REG8(0x3680), 0x00 },
	{ CCI_REG8(0x3682), 0x80 },
	{ CCI_REG8(0x3684), 0x01 },
	{ CCI_REG8(0x3688), 0x00 },
	{ CCI_REG8(0x368a), 0x0e },
	{ CCI_REG8(0x3696), 0x41 },
	{ CCI_REG8(0x369a), 0x00 },
	{ CCI_REG8(0x36d0), 0x00 },
	{ CCI_REG8(0x36d3), 0x40 },
	{ CCI_REG8(0x3700), 0x1c },
	{ CCI_REG8(0x3701), 0x13 },
	{ CCI_REG8(0x3704), 0x03 },
	{ CCI_REG8(0x3706), 0x34 },
	{ CCI_REG8(0x3707), 0x04 },
	{ CCI_REG8(0x3709), 0x7c },
	{ CCI_REG8(0x370b), 0x94 },
	{ CCI_REG8(0x3712), 0x00 },
	{ CCI_REG8(0x3714), 0xf2 },
	{ CCI_REG8(0x3716), 0x40 },
	{ CCI_REG8(0x3722), 0x05 },
	{ CCI_REG8(0x3724), 0x08 },
	{ CCI_REG8(0x372b), 0x00 },
	{ CCI_REG8(0x372e), 0x1c },
	{ CCI_REG8(0x372f), 0x13 },
	{ CCI_REG8(0x373f), 0x00 },
	{ CCI_REG8(0x374f), 0x58 },
	{ CCI_REG8(0x3755), 0x7c },
	{ CCI_REG8(0x3757), 0x70 },
	{ CCI_REG8(0x3759), 0x50 },
	{ CCI_REG8(0x375e), 0x0d },
	{ CCI_REG8(0x375f), 0x00 },
	{ CCI_REG8(0x3770), 0x04 },
	{ CCI_REG8(0x3780), 0x5e },
	{ CCI_REG8(0x3782), 0x01 },
	{ CCI_REG8(0x378a), 0x01 },
	{ CCI_REG8(0x3791), 0x34 },
	{ CCI_REG8(0x3793), 0x1c },
	{ CCI_REG8(0x3795), 0x1c },
	{ CCI_REG8(0x3797), 0x94 },
	{ CCI_REG8(0x3799), 0x3a },
	{ CCI_REG8(0x379b), 0x3a },
	{ CCI_REG8(0x379c), 0x01 },
	{ CCI_REG8(0x379f), 0x01 },
	{ CCI_REG8(0x37a0), 0x9b },
	{ CCI_REG8(0x37a9), 0x01 },
	{ CCI_REG8(0x37b2), 0xc8 },
	{ CCI_REG8(0x37b7), 0x02 },
	{ CCI_REG8(0x37bd), 0x00 },
	{ CCI_REG8(0x37c1), 0x1a },
	{ CCI_REG8(0x37c3), 0x1a },
	{ CCI_REG8(0x37cb), 0x02 },
	{ CCI_REG8(0x37cd), 0x02 },
	{ CCI_REG8(0x37d0), 0x22 },
	{ CCI_REG8(0x37d4), 0x00 },
	{ CCI_REG8(0x37db), 0x10 },
	{ CCI_REG8(0x37dc), 0x1a },
	{ CCI_REG8(0x37e3), 0x30 },
	{ CCI_REG8(0x37f0), 0x01 },
	{ CCI_REG8(0x37f6), 0x1a },
	{ CCI_REG8(0x3800), 0x00 },
	{ CCI_REG8(0x3801), 0x00 },
	{ CCI_REG8(0x3802), 0x00 },
	{ CCI_REG8(0x3803), 0x00 },
	{ CCI_REG8(0x3804), 0x20 },
	{ CCI_REG8(0x3805), 0x1f },
	{ CCI_REG8(0x3806), 0x18 },
	{ CCI_REG8(0x3807), 0x3f },
	{ CCI_REG8(0x3808), 0x10 },
	{ CCI_REG8(0x3809), 0x00 },
	{ CCI_REG8(0x380a), 0x0c },
	{ CCI_REG8(0x380b), 0x00 },
	{ CCI_REG8(0x380c), 0x03 },
	{ CCI_REG8(0x380d), 0x84 },
	{ CCI_REG8(0x380e), 0x03 },
	{ CCI_REG8(0x380f), 0x24 },
	{ CCI_REG8(0x3810), 0x00 },
	{ CCI_REG8(0x3811), 0x07 },
	{ CCI_REG8(0x3813), 0x10 },
	{ CCI_REG8(0x3815), 0x11 },
	{ CCI_REG8(0x3820), 0x46 },
	{ CCI_REG8(0x3821), 0x10 },
	{ CCI_REG8(0x3822), 0x10 },
	{ CCI_REG8(0x3823), 0x04 },
	{ CCI_REG8(0x3827), 0x40 },
	{ CCI_REG8(0x3828), 0x21 },
	{ CCI_REG8(0x3830), 0x20 },
	{ CCI_REG8(0x3831), 0x12 },
	{ CCI_REG8(0x3837), 0x20 },
	{ CCI_REG8(0x383f), 0x08 },
	{ CCI_REG8(0x384c), 0x03 },
	{ CCI_REG8(0x384d), 0x84 },
	{ CCI_REG8(0x3888), 0x00 },
	{ CCI_REG8(0x3889), 0x08 },
	{ CCI_REG8(0x388b), 0x10 },
	{ CCI_REG8(0x388c), 0x10 },
	{ CCI_REG8(0x388d), 0x00 },
	{ CCI_REG8(0x388e), 0x0c },
	{ CCI_REG8(0x388f), 0x00 },
	{ CCI_REG8(0x3896), 0x00 },
	{ CCI_REG8(0x38db), 0x08 },
	{ CCI_REG8(0x38dd), 0x04 },
	{ CCI_REG8(0x38de), 0x03 },
	{ CCI_REG8(0x38df), 0x08 },
	{ CCI_REG8(0x3906), 0x24 },
	{ CCI_REG8(0x390a), 0x15 },
	{ CCI_REG8(0x3919), 0x11 },
	{ CCI_REG8(0x3982), 0x40 },
	{ CCI_REG8(0x398b), 0x00 },
	{ CCI_REG8(0x399d), 0x13 },
	{ CCI_REG8(0x39dc), 0x00 },
	{ CCI_REG8(0x39fb), 0x01 },
	{ CCI_REG8(0x39fc), 0x01 },
	{ CCI_REG8(0x39fd), 0x01 },
	{ CCI_REG8(0x39fe), 0x01 },
	{ CCI_REG8(0x3a1d), 0x01 },
	{ CCI_REG8(0x3a1e), 0x01 },
	{ CCI_REG8(0x3a21), 0x01 },
	{ CCI_REG8(0x3a22), 0x01 },
	{ CCI_REG8(0x3a68), 0x13 },
	{ CCI_REG8(0x3a69), 0x20 },
	{ CCI_REG8(0x3ab6), 0x01 },
	{ CCI_REG8(0x3ab7), 0x01 },
	{ CCI_REG8(0x3af2), 0x03 },
	{ CCI_REG8(0x3b01), 0x1d },
	{ CCI_REG8(0x3b02), 0x00 },
	{ CCI_REG8(0x3b3d), 0x07 },
	{ CCI_REG8(0x3b4a), 0x00 },
	{ CCI_REG8(0x3b4b), 0x00 },
	{ CCI_REG8(0x3b56), 0x1f },
	{ CCI_REG8(0x3b57), 0x1f },
	{ CCI_REG8(0x3b58), 0x20 },
	{ CCI_REG8(0x3b59), 0x20 },
	{ CCI_REG8(0x3b5a), 0x19 },
	{ CCI_REG8(0x3b5b), 0x19 },
	{ CCI_REG8(0x3b5c), 0x19 },
	{ CCI_REG8(0x3b5d), 0x19 },
	{ CCI_REG8(0x3b82), 0x19 },
	{ CCI_REG8(0x3ba1), 0x1e },
	{ CCI_REG8(0x3ba6), 0x77 },
	{ CCI_REG8(0x3ba7), 0x77 },
	{ CCI_REG8(0x3baa), 0x33 },
	{ CCI_REG8(0x3bab), 0x2f },
	{ CCI_REG8(0x3baf), 0x16 },
	{ CCI_REG8(0x3bba), 0x48 },
	{ CCI_REG8(0x3bf3), 0x01 },
	{ CCI_REG8(0x3bfb), 0x01 },
	{ CCI_REG8(0x3bfc), 0x50 },
	{ CCI_REG8(0x3bff), 0x08 },
	{ CCI_REG8(0x400e), 0x1c },
	{ CCI_REG8(0x4010), 0x34 },
	{ CCI_REG8(0x4012), 0x17 },
	{ CCI_REG8(0x4015), 0x08 },
	{ CCI_REG8(0x4016), 0x17 },
	{ CCI_REG8(0x4018), 0x07 },
	{ CCI_REG8(0x4506), 0x01 },
	{ CCI_REG8(0x4509), 0x07 },
	{ CCI_REG8(0x450c), 0x00 },
	{ CCI_REG8(0x450d), 0x60 },
	{ CCI_REG8(0x4510), 0x03 },
	{ CCI_REG8(0x4516), 0x55 },
	{ CCI_REG8(0x4517), 0x55 },
	{ CCI_REG8(0x4518), 0x55 },
	{ CCI_REG8(0x4519), 0x55 },
	{ CCI_REG8(0x451a), 0xaa },
	{ CCI_REG8(0x451b), 0xaa },
	{ CCI_REG8(0x451c), 0xaa },
	{ CCI_REG8(0x451d), 0xaa },
	{ CCI_REG8(0x451e), 0xff },
	{ CCI_REG8(0x451f), 0xff },
	{ CCI_REG8(0x4520), 0xff },
	{ CCI_REG8(0x4521), 0xff },
	{ CCI_REG8(0x4522), 0x29 },
	{ CCI_REG8(0x4523), 0x08 },
	{ CCI_REG8(0x4524), 0xbb },
	{ CCI_REG8(0x4525), 0x0c },
	{ CCI_REG8(0x4545), 0x00 },
	{ CCI_REG8(0x4546), 0x02 },
	{ CCI_REG8(0x4547), 0xd0 },
	{ CCI_REG8(0x4549), 0x00 },
	{ CCI_REG8(0x454a), 0x29 },
	{ CCI_REG8(0x454b), 0x08 },
	{ CCI_REG8(0x454c), 0xbb },
	{ CCI_REG8(0x454d), 0x0c },
	{ CCI_REG8(0x454e), 0x29 },
	{ CCI_REG8(0x454f), 0x08 },
	{ CCI_REG8(0x4550), 0xbb },
	{ CCI_REG8(0x4551), 0x0c },
	{ CCI_REG8(0x4552), 0x29 },
	{ CCI_REG8(0x4553), 0x08 },
	{ CCI_REG8(0x4554), 0xbb },
	{ CCI_REG8(0x4555), 0x0c },
	{ CCI_REG8(0x4556), 0x29 },
	{ CCI_REG8(0x4557), 0x08 },
	{ CCI_REG8(0x4558), 0xbb },
	{ CCI_REG8(0x4559), 0x0c },
	{ CCI_REG8(0x455a), 0x29 },
	{ CCI_REG8(0x455b), 0x08 },
	{ CCI_REG8(0x455c), 0xbb },
	{ CCI_REG8(0x455d), 0x0c },
	{ CCI_REG8(0x455e), 0x29 },
	{ CCI_REG8(0x455f), 0x08 },
	{ CCI_REG8(0x4560), 0xbb },
	{ CCI_REG8(0x4561), 0x0c },
	{ CCI_REG8(0x4562), 0x29 },
	{ CCI_REG8(0x4563), 0x08 },
	{ CCI_REG8(0x4564), 0xbb },
	{ CCI_REG8(0x4565), 0x0c },
	{ CCI_REG8(0x45c0), 0x8e },
	{ CCI_REG8(0x45c1), 0x80 },
	{ CCI_REG8(0x45c2), 0x0a },
	{ CCI_REG8(0x45c3), 0x04 },
	{ CCI_REG8(0x45c4), 0x13 },
	{ CCI_REG8(0x45c5), 0x40 },
	{ CCI_REG8(0x45c6), 0x01 },
	{ CCI_REG8(0x4602), 0x00 },
	{ CCI_REG8(0x4603), 0x15 },
	{ CCI_REG8(0x460b), 0x07 },
	{ CCI_REG8(0x4640), 0x01 },
	{ CCI_REG8(0x4641), 0x00 },
	{ CCI_REG8(0x4643), 0x0c },
	{ CCI_REG8(0x4680), 0x11 },
	{ CCI_REG8(0x4684), 0x2b },
	{ CCI_REG8(0x468e), 0x30 },
	{ CCI_REG8(0x4836), 0x32 },
	{ CCI_REG8(0x4837), 0x04 },
	{ CCI_REG8(0x49f5), 0x00 },
	{ CCI_REG8(0x5000), 0x2b },
	{ CCI_REG8(0x5001), 0x08 },
	{ CCI_REG8(0x5002), 0x00 },
	{ CCI_REG8(0x5006), 0x00 },
	{ CCI_REG8(0x5007), 0x06 },
	{ CCI_REG8(0x5009), 0x40 },
	{ CCI_REG8(0x5091), 0x00 },
	{ CCI_REG8(0x5180), 0xc0 },
	{ CCI_REG8(0x5480), 0xc0 },
	{ CCI_REG8(0x5780), 0xc0 },
	{ CCI_REG8(0x6a03), 0x00 },
	{ CCI_REG8(0xc200), 0x00 },
	{ CCI_REG8(0xc201), 0x00 },
	{ CCI_REG8(0xc202), 0x00 },
	{ CCI_REG8(0xc203), 0x00 },
	{ CCI_REG8(0xc210), 0x00 },
	{ CCI_REG8(0xc211), 0x00 },
	{ CCI_REG8(0xc212), 0x00 },
	{ CCI_REG8(0xc213), 0x00 },
	{ CCI_REG8(0xc214), 0x00 },
	{ CCI_REG8(0xc230), 0x00 },
	{ CCI_REG8(0xc231), 0x00 },
	{ CCI_REG8(0xc232), 0x00 },
	{ CCI_REG8(0xc233), 0x00 },
	{ CCI_REG8(0xc240), 0x00 },
	{ CCI_REG8(0xc241), 0x00 },
	{ CCI_REG8(0xc242), 0x00 },
	{ CCI_REG8(0xc243), 0x00 },
	{ CCI_REG8(0xc250), 0x00 },
	{ CCI_REG8(0xc251), 0x00 },
	{ CCI_REG8(0xc252), 0x00 },
	{ CCI_REG8(0xc253), 0x00 },
	{ CCI_REG8(0xc260), 0x00 },
	{ CCI_REG8(0xc261), 0x00 },
	{ CCI_REG8(0xc262), 0x00 },
	{ CCI_REG8(0xc263), 0x00 },
	{ CCI_REG8(0xc270), 0x00 },
	{ CCI_REG8(0xc271), 0x00 },
	{ CCI_REG8(0xc272), 0x00 },
	{ CCI_REG8(0xc273), 0x00 },
	{ CCI_REG8(0xc40e), 0x00 },
	{ CCI_REG8(0xc448), 0x00 },
	{ CCI_REG8(0xc46e), 0x01 },
	{ CCI_REG8(0xc478), 0x01 },
	{ CCI_REG8(0xc49e), 0x34 },
	{ CCI_REG8(0xc49f), 0x34 },
	{ CCI_REG8(0xc4a2), 0x94 },
	{ CCI_REG8(0xc4a3), 0x94 },
	{ CCI_REG8(0xc4c1), 0x07 },
	{ CCI_REG8(0xc4c2), 0x07 },
	{ CCI_REG8(0xc4c3), 0x77 },
	{ CCI_REG8(0xc4c4), 0x77 },
	{ CCI_REG8(0xc4d2), 0x00 },
	{ CCI_REG8(0xc4d3), 0x00 },
	{ CCI_REG8(0xc4d4), 0x00 },
	{ CCI_REG8(0xc4d5), 0x00 },
	{ CCI_REG8(0xc4e3), 0x19 },
	{ CCI_REG8(0xc4e9), 0x1e },
	{ CCI_REG8(0xc506), 0x16 },
	{ CCI_REG8(0xc50e), 0x1f },
	{ CCI_REG8(0xc50f), 0x1f },
	{ CCI_REG8(0xc510), 0x0f },
	{ CCI_REG8(0xc511), 0x07 },
	{ CCI_REG8(0xc512), 0x03 },
	{ CCI_REG8(0xc513), 0x4e },
	{ CCI_REG8(0xc514), 0x4e },
	{ CCI_REG8(0xc515), 0x27 },
	{ CCI_REG8(0xc516), 0x16 },
	{ CCI_REG8(0xc517), 0x0c },
	{ CCI_REG8(0xc518), 0x33 },
	{ CCI_REG8(0xc519), 0x33 },
	{ CCI_REG8(0xc51a), 0x33 },
	{ CCI_REG8(0xc51b), 0x3b },
	{ CCI_REG8(0xc51c), 0x3b },
	{ CCI_REG8(0xc51d), 0x2f },
	{ CCI_REG8(0xc51e), 0x2f },
	{ CCI_REG8(0xc51f), 0x2f },
	{ CCI_REG8(0xc520), 0x2f },
	{ CCI_REG8(0xc521), 0x30 },
	{ CCI_REG8(0xc52e), 0x0e },
	{ CCI_REG8(0xc52f), 0x0e },
	{ CCI_REG8(0xc530), 0x0e },
	{ CCI_REG8(0xc531), 0x0e },
	{ CCI_REG8(0xc532), 0x0e },
	{ CCI_REG8(0xc533), 0x0e },
	{ CCI_REG8(0xc534), 0x0e },
	{ CCI_REG8(0xc535), 0x0e },
	{ CCI_REG8(0xc542), 0x0e },
	{ CCI_REG8(0xc543), 0x0e },
	{ CCI_REG8(0xc544), 0x0e },
	{ CCI_REG8(0xc545), 0x0e },
	{ CCI_REG8(0xc546), 0x0e },
	{ CCI_REG8(0xc547), 0x0e },
	{ CCI_REG8(0xc548), 0x0e },
	{ CCI_REG8(0xc549), 0x0e },
	{ CCI_REG8(0xc57d), 0x00 },
	{ CCI_REG8(0xc581), 0x18 },
	{ CCI_REG8(0xc582), 0x18 },
	{ CCI_REG8(0xc583), 0x02 },
	{ CCI_REG8(0xc584), 0x01 },
	{ CCI_REG8(0xc587), 0x18 },
	{ CCI_REG8(0xc589), 0x18 },
	{ CCI_REG8(0xc58a), 0x10 },
	{ CCI_REG8(0xc58b), 0x08 },
	{ CCI_REG8(0xc58c), 0x01 },
	{ CCI_REG8(0xc58f), 0x28 },
	{ CCI_REG8(0xc590), 0x28 },
	{ CCI_REG8(0xc591), 0x28 },
	{ CCI_REG8(0xc592), 0x28 },
	{ CCI_REG8(0xc593), 0x0a },
	{ CCI_REG8(0xc594), 0x06 },
	{ CCI_REG8(0xc597), 0x2e },
	{ CCI_REG8(0xc598), 0x2e },
	{ CCI_REG8(0xc599), 0x2e },
	{ CCI_REG8(0xc59a), 0x18 },
	{ CCI_REG8(0xc59b), 0x0e },
	{ CCI_REG8(0xc59c), 0x08 },
	{ CCI_REG8(0xc5e4), 0x00 },
	{ CCI_REG8(0xc5e5), 0x07 },
	{ CCI_REG8(0xc5e8), 0x01 },
	{ CCI_REG8(0xc702), 0x10 },
	{ CCI_REG8(0xc726), 0x03 },
	{ CCI_REG8(0xc72b), 0xff },
	{ CCI_REG8(0xc72c), 0xff },
	{ CCI_REG8(0xc72d), 0xff },
	{ CCI_REG8(0xc72f), 0x08 },
	{ CCI_REG8(0xc736), 0x01 },
	{ CCI_REG8(0xc739), 0x18 },
	{ CCI_REG8(0xc73a), 0xa6 },
	{ CCI_REG8(0xc73b), 0x00 },
	{ CCI_REG8(0xc73c), 0x00 },
	{ CCI_REG8(0xc746), 0x01 },
	{ CCI_REG8(0xc747), 0x04 },
	{ CCI_REG8(0xc749), 0x1c },
	{ CCI_REG8(0xc75b), 0x01 },
	{ CCI_REG8(0xc75c), 0x05 },
	{ CCI_REG8(0xc765), 0x2a },
	{ CCI_REG8(0xc773), 0x02 },
	{ CCI_REG8(0xc774), 0x03 },
	{ CCI_REG8(0xc78a), 0x03 },
	{ CCI_REG8(0xc78b), 0x04 },
	{ CCI_REG8(0xc798), 0x03 },
	{ CCI_REG8(0xc7a2), 0x01 },
	{ CCI_REG8(0xc7a6), 0x02 },
	{ CCI_REG8(0xc7a7), 0x02 },
	{ CCI_REG8(0xc7a8), 0xff },
	{ CCI_REG8(0xc7a9), 0xff },
	{ CCI_REG8(0xc7aa), 0xff },
	{ CCI_REG8(0xc7ac), 0x02 },
	{ CCI_REG8(0xc7ad), 0x08 },
	{ CCI_REG8(0xc7ae), 0xff },
	{ CCI_REG8(0xc7af), 0xff },
	{ CCI_REG8(0xc7b0), 0xff },
	{ CCI_REG8(0xc7b2), 0x01 },
	{ CCI_REG8(0xc7b3), 0x02 },
	{ CCI_REG8(0xc7b4), 0xff },
	{ CCI_REG8(0xc7b5), 0xff },
	{ CCI_REG8(0xc7b6), 0xff },
	{ CCI_REG8(0xc7c4), 0x01 },
	{ CCI_REG8(0xc7c5), 0x00 },
	{ CCI_REG8(0xc7e2), 0x01 },
	{ CCI_REG8(0xc855), 0x77 },
	{ CCI_REG8(0xc8a4), 0x77 },
	{ CCI_REG8(0xc95a), 0x77 },
	{ CCI_REG8(0xc95b), 0x77 },
	{ CCI_REG8(0xc9b9), 0x18 },
	{ CCI_REG8(0xc9fe), 0x0a },
	{ CCI_REG8(0xc9ff), 0x12 },
	{ CCI_REG8(0xca00), 0x1a },
	{ CCI_REG8(0xca02), 0x1a },
	{ CCI_REG8(0xca17), 0x04 },
	{ CCI_REG8(0xca18), 0x1a },
	{ CCI_REG8(0xca19), 0x1a },
	{ CCI_REG8(0x380c), 0x03 },
	{ CCI_REG8(0x380d), 0x84 },
	{ CCI_REG8(0x380e), 0x0a },
	{ CCI_REG8(0x380f), 0xd8 },
	{ CCI_REG8(0x384c), 0x03 },
	{ CCI_REG8(0x384d), 0x84 },
	{ CCI_REG8(0x4546), 0x03 },
	{ CCI_REG8(0x4547), 0x10 },
	{ CCI_REG8(0x3840), 0x00 },
	{ CCI_REG8(0x3500), 0x00 },
	{ CCI_REG8(0x3501), 0x0a },
	{ CCI_REG8(0x3502), 0x00 },
	{ CCI_REG8(0x3508), 0x01 },
	{ CCI_REG8(0x3509), 0x00 },
	{ CCI_REG8(0x3869), 0x04 },
};

/*
 * Written after the 4096x3072 mode table: routes the PDAF data to virtual
 * channel 2 (0x4813), closes group 1 and programs group 2 and the group
 * launch sequence.
 */
static const struct cci_reg_sequence ov50h40_4096x3072_post_regs[] = {
	{ CCI_REG8(0x4813), 0x20 },
	{ CCI_REG8(0x301f), 0x98 },
	{ CCI_REG8(0x3017), 0xf0 },
	{ CCI_REG8(0x3014), 0xf0 },
	{ CCI_REG8(0x3046), 0x01 },
	{ CCI_REG8(0x3a88), 0x44 },
	{ CCI_REG8(0x3bd7), 0xc8 },
	{ CCI_REG8(0x3bdc), 0x40 },
	{ CCI_REG8(0x3be5), 0x01 },
	{ CCI_REG8(0x392e), 0x5a },
	{ CCI_REG8(0x3016), 0xf3 },
	{ CCI_REG8(0x3016), 0xf0 },
	{ CCI_REG8(0x3208), 0x11 },
	{ CCI_REG8(0x3208), 0x02 },
	{ CCI_REG8(0x3a88), 0x04 },
	{ CCI_REG8(0x3bd7), 0x48 },
	{ CCI_REG8(0x3bdc), 0x00 },
	{ CCI_REG8(0x3be5), 0x00 },
	{ CCI_REG8(0x392e), 0x0a },
	{ CCI_REG8(0x3208), 0x12 },
	{ CCI_REG8(0x320d), 0x01 },
	{ CCI_REG8(0x3209), 0x00 },
	{ CCI_REG8(0x320a), 0x01 },
	{ CCI_REG8(0x320b), 0x01 },
	{ CCI_REG8(0x320d), 0x92 },
	{ CCI_REG8(0x320e), 0xa1 },
};

/* 8192x6144 (full resolution, in-sensor remosaic), 30 fps */
static const struct cci_reg_sequence ov50h40_8192x6144_regs[] = {
	{ CCI_REG8(0x0304), 0x03 },
	{ CCI_REG8(0x0305), 0x45 },
	{ CCI_REG8(0x0327), 0x0e },
	{ CCI_REG8(0x0329), 0x01 },
	{ CCI_REG8(0x032c), 0x00 },
	{ CCI_REG8(0x0344), 0x01 },
	{ CCI_REG8(0x0345), 0x20 },
	{ CCI_REG8(0x0360), 0x09 },
	{ CCI_REG8(0x3027), 0x00 },
	{ CCI_REG8(0x3400), 0x0c },
	{ CCI_REG8(0x3422), 0x08 },
	{ CCI_REG8(0x3423), 0x00 },
	{ CCI_REG8(0x3506), 0x78 },
	{ CCI_REG8(0x350d), 0x01 },
	{ CCI_REG8(0x350e), 0x00 },
	{ CCI_REG8(0x350f), 0x00 },
	{ CCI_REG8(0x3546), 0x78 },
	{ CCI_REG8(0x354d), 0x01 },
	{ CCI_REG8(0x354e), 0x00 },
	{ CCI_REG8(0x354f), 0x00 },
	{ CCI_REG8(0x3586), 0x78 },
	{ CCI_REG8(0x358d), 0x01 },
	{ CCI_REG8(0x358e), 0x00 },
	{ CCI_REG8(0x358f), 0x00 },
	{ CCI_REG8(0x3609), 0x80 },
	{ CCI_REG8(0x360c), 0x0f },
	{ CCI_REG8(0x3610), 0x08 },
	{ CCI_REG8(0x3614), 0x0c },
	{ CCI_REG8(0x3618), 0xcf },
	{ CCI_REG8(0x3619), 0x44 },
	{ CCI_REG8(0x361a), 0x81 },
	{ CCI_REG8(0x361d), 0x1f },
	{ CCI_REG8(0x363b), 0x6a },
	{ CCI_REG8(0x363c), 0x6a },
	{ CCI_REG8(0x3640), 0x00 },
	{ CCI_REG8(0x3641), 0x02 },
	{ CCI_REG8(0x3644), 0x00 },
	{ CCI_REG8(0x3645), 0x06 },
	{ CCI_REG8(0x3647), 0x01 },
	{ CCI_REG8(0x3650), 0xbf },
	{ CCI_REG8(0x3653), 0x03 },
	{ CCI_REG8(0x3680), 0x00 },
	{ CCI_REG8(0x3682), 0x80 },
	{ CCI_REG8(0x3684), 0x00 },
	{ CCI_REG8(0x3688), 0x00 },
	{ CCI_REG8(0x368a), 0x0e },
	{ CCI_REG8(0x3696), 0x41 },
	{ CCI_REG8(0x369a), 0x00 },
	{ CCI_REG8(0x36d0), 0x00 },
	{ CCI_REG8(0x36d3), 0x80 },
	{ CCI_REG8(0x3700), 0x1c },
	{ CCI_REG8(0x3701), 0x13 },
	{ CCI_REG8(0x3704), 0x03 },
	{ CCI_REG8(0x3706), 0x1c },
	{ CCI_REG8(0x3707), 0x04 },
	{ CCI_REG8(0x3709), 0x70 },
	{ CCI_REG8(0x370b), 0x3a },
	{ CCI_REG8(0x3712), 0x01 },
	{ CCI_REG8(0x3714), 0xf8 },
	{ CCI_REG8(0x3716), 0x40 },
	{ CCI_REG8(0x3722), 0x05 },
	{ CCI_REG8(0x3724), 0xe0 },
	{ CCI_REG8(0x372b), 0x00 },
	{ CCI_REG8(0x372e), 0x1c },
	{ CCI_REG8(0x372f), 0x13 },
	{ CCI_REG8(0x373f), 0x00 },
	{ CCI_REG8(0x374f), 0x58 },
	{ CCI_REG8(0x3755), 0xb1 },
	{ CCI_REG8(0x3757), 0x30 },
	{ CCI_REG8(0x3759), 0x50 },
	{ CCI_REG8(0x375e), 0x00 },
	{ CCI_REG8(0x375f), 0x00 },
	{ CCI_REG8(0x3770), 0x01 },
	{ CCI_REG8(0x3780), 0x5c },
	{ CCI_REG8(0x3782), 0x01 },
	{ CCI_REG8(0x378a), 0x01 },
	{ CCI_REG8(0x3791), 0x30 },
	{ CCI_REG8(0x3793), 0x1c },
	{ CCI_REG8(0x3795), 0x1c },
	{ CCI_REG8(0x3797), 0x8e },
	{ CCI_REG8(0x3799), 0x3a },
	{ CCI_REG8(0x379b), 0x3a },
	{ CCI_REG8(0x379c), 0x01 },
	{ CCI_REG8(0x379f), 0x01 },
	{ CCI_REG8(0x37a0), 0x70 },
	{ CCI_REG8(0x37a9), 0x01 },
	{ CCI_REG8(0x37b2), 0xc8 },
	{ CCI_REG8(0x37b7), 0x02 },
	{ CCI_REG8(0x37bd), 0x00 },
	{ CCI_REG8(0x37c1), 0x1a },
	{ CCI_REG8(0x37c3), 0x1a },
	{ CCI_REG8(0x37cb), 0x02 },
	{ CCI_REG8(0x37cd), 0x01 },
	{ CCI_REG8(0x37d0), 0x00 },
	{ CCI_REG8(0x37d4), 0x00 },
	{ CCI_REG8(0x37db), 0x10 },
	{ CCI_REG8(0x37dc), 0x1a },
	{ CCI_REG8(0x37e3), 0x30 },
	{ CCI_REG8(0x37f0), 0x01 },
	{ CCI_REG8(0x37f6), 0x1a },
	{ CCI_REG8(0x3800), 0x00 },
	{ CCI_REG8(0x3801), 0x00 },
	{ CCI_REG8(0x3802), 0x00 },
	{ CCI_REG8(0x3803), 0x00 },
	{ CCI_REG8(0x3804), 0x20 },
	{ CCI_REG8(0x3805), 0x1f },
	{ CCI_REG8(0x3806), 0x18 },
	{ CCI_REG8(0x3807), 0x3f },
	{ CCI_REG8(0x3808), 0x20 },
	{ CCI_REG8(0x3809), 0x00 },
	{ CCI_REG8(0x380a), 0x18 },
	{ CCI_REG8(0x380b), 0x00 },
	{ CCI_REG8(0x380c), 0x03 },
	{ CCI_REG8(0x380d), 0x00 },
	{ CCI_REG8(0x380e), 0x0c },
	{ CCI_REG8(0x380f), 0xb0 },
	{ CCI_REG8(0x3810), 0x00 },
	{ CCI_REG8(0x3811), 0x0f },
	{ CCI_REG8(0x3813), 0x20 },
	{ CCI_REG8(0x3815), 0x11 },
	{ CCI_REG8(0x3820), 0x44 },
	{ CCI_REG8(0x3821), 0x00 },
	{ CCI_REG8(0x3822), 0x00 },
	{ CCI_REG8(0x3823), 0x04 },
	{ CCI_REG8(0x3827), 0x40 },
	{ CCI_REG8(0x3828), 0x27 },
	{ CCI_REG8(0x3830), 0x20 },
	{ CCI_REG8(0x3831), 0x10 },
	{ CCI_REG8(0x3837), 0x20 },
	{ CCI_REG8(0x383f), 0x08 },
	{ CCI_REG8(0x384c), 0x03 },
	{ CCI_REG8(0x384d), 0x00 },
	{ CCI_REG8(0x3888), 0x00 },
	{ CCI_REG8(0x3889), 0x10 },
	{ CCI_REG8(0x388b), 0x20 },
	{ CCI_REG8(0x388c), 0x20 },
	{ CCI_REG8(0x388d), 0x00 },
	{ CCI_REG8(0x388e), 0x18 },
	{ CCI_REG8(0x388f), 0x00 },
	{ CCI_REG8(0x3896), 0x00 },
	{ CCI_REG8(0x38db), 0x20 },
	{ CCI_REG8(0x38dd), 0x10 },
	{ CCI_REG8(0x38de), 0x0c },
	{ CCI_REG8(0x38df), 0x20 },
	{ CCI_REG8(0x3906), 0x24 },
	{ CCI_REG8(0x390a), 0x05 },
	{ CCI_REG8(0x3919), 0x15 },
	{ CCI_REG8(0x3982), 0x40 },
	{ CCI_REG8(0x398b), 0x00 },
	{ CCI_REG8(0x399d), 0x05 },
	{ CCI_REG8(0x39dc), 0x01 },
	{ CCI_REG8(0x39fb), 0x01 },
	{ CCI_REG8(0x39fc), 0x01 },
	{ CCI_REG8(0x39fd), 0x06 },
	{ CCI_REG8(0x39fe), 0x06 },
	{ CCI_REG8(0x3a1d), 0x01 },
	{ CCI_REG8(0x3a1e), 0x01 },
	{ CCI_REG8(0x3a21), 0x01 },
	{ CCI_REG8(0x3a22), 0x06 },
	{ CCI_REG8(0x3a68), 0x05 },
	{ CCI_REG8(0x3a69), 0x20 },
	{ CCI_REG8(0x3ab6), 0x01 },
	{ CCI_REG8(0x3ab7), 0x01 },
	{ CCI_REG8(0x3af2), 0x03 },
	{ CCI_REG8(0x3b01), 0x00 },
	{ CCI_REG8(0x3b02), 0x00 },
	{ CCI_REG8(0x3b3d), 0x07 },
	{ CCI_REG8(0x3b4a), 0x38 },
	{ CCI_REG8(0x3b4b), 0x38 },
	{ CCI_REG8(0x3b56), 0x20 },
	{ CCI_REG8(0x3b57), 0x21 },
	{ CCI_REG8(0x3b58), 0x21 },
	{ CCI_REG8(0x3b59), 0x21 },
	{ CCI_REG8(0x3b5a), 0x14 },
	{ CCI_REG8(0x3b5b), 0x14 },
	{ CCI_REG8(0x3b5c), 0x14 },
	{ CCI_REG8(0x3b5d), 0x14 },
	{ CCI_REG8(0x3b82), 0x14 },
	{ CCI_REG8(0x3ba1), 0x20 },
	{ CCI_REG8(0x3ba6), 0x00 },
	{ CCI_REG8(0x3ba7), 0x00 },
	{ CCI_REG8(0x3baa), 0x33 },
	{ CCI_REG8(0x3bab), 0x37 },
	{ CCI_REG8(0x3baf), 0x00 },
	{ CCI_REG8(0x3bba), 0x4c },
	{ CCI_REG8(0x3bf3), 0x01 },
	{ CCI_REG8(0x3bfb), 0x01 },
	{ CCI_REG8(0x3bfc), 0x50 },
	{ CCI_REG8(0x3bff), 0x08 },
	{ CCI_REG8(0x400e), 0x14 },
	{ CCI_REG8(0x4010), 0x34 },
	{ CCI_REG8(0x4012), 0x17 },
	{ CCI_REG8(0x4015), 0x10 },
	{ CCI_REG8(0x4016), 0x2f },
	{ CCI_REG8(0x4018), 0x0f },
	{ CCI_REG8(0x4506), 0x01 },
	{ CCI_REG8(0x4509), 0x07 },
	{ CCI_REG8(0x450c), 0x00 },
	{ CCI_REG8(0x450d), 0x30 },
	{ CCI_REG8(0x4510), 0x00 },
	{ CCI_REG8(0x4516), 0x00 },
	{ CCI_REG8(0x4517), 0x00 },
	{ CCI_REG8(0x4518), 0x00 },
	{ CCI_REG8(0x4519), 0x00 },
	{ CCI_REG8(0x451a), 0x00 },
	{ CCI_REG8(0x451b), 0x00 },
	{ CCI_REG8(0x451c), 0x00 },
	{ CCI_REG8(0x451d), 0x00 },
	{ CCI_REG8(0x451e), 0x00 },
	{ CCI_REG8(0x451f), 0x00 },
	{ CCI_REG8(0x4520), 0x00 },
	{ CCI_REG8(0x4521), 0x00 },
	{ CCI_REG8(0x4522), 0x00 },
	{ CCI_REG8(0x4523), 0x00 },
	{ CCI_REG8(0x4524), 0x00 },
	{ CCI_REG8(0x4525), 0x00 },
	{ CCI_REG8(0x4545), 0x00 },
	{ CCI_REG8(0x4546), 0x04 },
	{ CCI_REG8(0x4547), 0xcc },
	{ CCI_REG8(0x4549), 0x00 },
	{ CCI_REG8(0x454a), 0x00 },
	{ CCI_REG8(0x454b), 0x00 },
	{ CCI_REG8(0x454c), 0x00 },
	{ CCI_REG8(0x454d), 0x00 },
	{ CCI_REG8(0x454e), 0x00 },
	{ CCI_REG8(0x454f), 0x00 },
	{ CCI_REG8(0x4550), 0x00 },
	{ CCI_REG8(0x4551), 0x00 },
	{ CCI_REG8(0x4552), 0x00 },
	{ CCI_REG8(0x4553), 0x00 },
	{ CCI_REG8(0x4554), 0x00 },
	{ CCI_REG8(0x4555), 0x00 },
	{ CCI_REG8(0x4556), 0x00 },
	{ CCI_REG8(0x4557), 0x00 },
	{ CCI_REG8(0x4558), 0x00 },
	{ CCI_REG8(0x4559), 0x00 },
	{ CCI_REG8(0x455a), 0x00 },
	{ CCI_REG8(0x455b), 0x00 },
	{ CCI_REG8(0x455c), 0x00 },
	{ CCI_REG8(0x455d), 0x00 },
	{ CCI_REG8(0x455e), 0x00 },
	{ CCI_REG8(0x455f), 0x00 },
	{ CCI_REG8(0x4560), 0x00 },
	{ CCI_REG8(0x4561), 0x00 },
	{ CCI_REG8(0x4562), 0x00 },
	{ CCI_REG8(0x4563), 0x00 },
	{ CCI_REG8(0x4564), 0x00 },
	{ CCI_REG8(0x4565), 0x00 },
	{ CCI_REG8(0x45c0), 0x9c },
	{ CCI_REG8(0x45c1), 0x80 },
	{ CCI_REG8(0x45c2), 0x0a },
	{ CCI_REG8(0x45c3), 0x04 },
	{ CCI_REG8(0x45c4), 0x13 },
	{ CCI_REG8(0x45c5), 0x80 },
	{ CCI_REG8(0x45c6), 0x08 },
	{ CCI_REG8(0x4602), 0x00 },
	{ CCI_REG8(0x4603), 0x15 },
	{ CCI_REG8(0x460b), 0x07 },
	{ CCI_REG8(0x4640), 0x01 },
	{ CCI_REG8(0x4641), 0x00 },
	{ CCI_REG8(0x4643), 0x08 },
	{ CCI_REG8(0x4680), 0x11 },
	{ CCI_REG8(0x4684), 0x2b },
	{ CCI_REG8(0x468e), 0x30 },
	{ CCI_REG8(0x4813), 0x10 },
	{ CCI_REG8(0x4836), 0x32 },
	{ CCI_REG8(0x4837), 0x04 },
	{ CCI_REG8(0x49f5), 0x00 },
	{ CCI_REG8(0x5000), 0x5b },
	{ CCI_REG8(0x5001), 0x28 },
	{ CCI_REG8(0x5002), 0x00 },
	{ CCI_REG8(0x5006), 0x00 },
	{ CCI_REG8(0x5007), 0x06 },
	{ CCI_REG8(0x5009), 0x2e },
	{ CCI_REG8(0x5091), 0x00 },
	{ CCI_REG8(0x5180), 0xc0 },
	{ CCI_REG8(0x5480), 0xc0 },
	{ CCI_REG8(0x5780), 0xc0 },
	{ CCI_REG8(0x6a03), 0x00 },
	{ CCI_REG8(0xc200), 0x00 },
	{ CCI_REG8(0xc201), 0x00 },
	{ CCI_REG8(0xc202), 0x00 },
	{ CCI_REG8(0xc203), 0x00 },
	{ CCI_REG8(0xc210), 0x00 },
	{ CCI_REG8(0xc211), 0x00 },
	{ CCI_REG8(0xc212), 0x00 },
	{ CCI_REG8(0xc213), 0x00 },
	{ CCI_REG8(0xc214), 0x00 },
	{ CCI_REG8(0xc230), 0x00 },
	{ CCI_REG8(0xc231), 0x00 },
	{ CCI_REG8(0xc232), 0x00 },
	{ CCI_REG8(0xc233), 0x00 },
	{ CCI_REG8(0xc240), 0x00 },
	{ CCI_REG8(0xc241), 0x00 },
	{ CCI_REG8(0xc242), 0x00 },
	{ CCI_REG8(0xc243), 0x00 },
	{ CCI_REG8(0xc250), 0x00 },
	{ CCI_REG8(0xc251), 0x00 },
	{ CCI_REG8(0xc252), 0x00 },
	{ CCI_REG8(0xc253), 0x00 },
	{ CCI_REG8(0xc260), 0x00 },
	{ CCI_REG8(0xc261), 0x00 },
	{ CCI_REG8(0xc262), 0x00 },
	{ CCI_REG8(0xc263), 0x00 },
	{ CCI_REG8(0xc270), 0x00 },
	{ CCI_REG8(0xc271), 0x00 },
	{ CCI_REG8(0xc272), 0x00 },
	{ CCI_REG8(0xc273), 0x00 },
	{ CCI_REG8(0xc40e), 0xa0 },
	{ CCI_REG8(0xc448), 0x00 },
	{ CCI_REG8(0xc46e), 0x01 },
	{ CCI_REG8(0xc478), 0x01 },
	{ CCI_REG8(0xc49e), 0x1c },
	{ CCI_REG8(0xc49f), 0x30 },
	{ CCI_REG8(0xc4a2), 0x3a },
	{ CCI_REG8(0xc4a3), 0x8e },
	{ CCI_REG8(0xc4c1), 0x07 },
	{ CCI_REG8(0xc4c2), 0x07 },
	{ CCI_REG8(0xc4c3), 0x77 },
	{ CCI_REG8(0xc4c4), 0x77 },
	{ CCI_REG8(0xc4d2), 0x38 },
	{ CCI_REG8(0xc4d3), 0x38 },
	{ CCI_REG8(0xc4d4), 0x38 },
	{ CCI_REG8(0xc4d5), 0x38 },
	{ CCI_REG8(0xc4e3), 0x14 },
	{ CCI_REG8(0xc4e9), 0x20 },
	{ CCI_REG8(0xc506), 0x14 },
	{ CCI_REG8(0xc50e), 0x00 },
	{ CCI_REG8(0xc50f), 0x00 },
	{ CCI_REG8(0xc510), 0x00 },
	{ CCI_REG8(0xc511), 0x00 },
	{ CCI_REG8(0xc512), 0x00 },
	{ CCI_REG8(0xc513), 0x4e },
	{ CCI_REG8(0xc514), 0x4f },
	{ CCI_REG8(0xc515), 0x2a },
	{ CCI_REG8(0xc516), 0x16 },
	{ CCI_REG8(0xc517), 0x0b },
	{ CCI_REG8(0xc518), 0x33 },
	{ CCI_REG8(0xc519), 0x33 },
	{ CCI_REG8(0xc51a), 0x33 },
	{ CCI_REG8(0xc51b), 0x33 },
	{ CCI_REG8(0xc51c), 0x33 },
	{ CCI_REG8(0xc51d), 0x37 },
	{ CCI_REG8(0xc51e), 0x37 },
	{ CCI_REG8(0xc51f), 0x3a },
	{ CCI_REG8(0xc520), 0x3a },
	{ CCI_REG8(0xc521), 0x3a },
	{ CCI_REG8(0xc52e), 0x0e },
	{ CCI_REG8(0xc52f), 0x0e },
	{ CCI_REG8(0xc530), 0x0e },
	{ CCI_REG8(0xc531), 0x0e },
	{ CCI_REG8(0xc532), 0x0e },
	{ CCI_REG8(0xc533), 0x0e },
	{ CCI_REG8(0xc534), 0x0e },
	{ CCI_REG8(0xc535), 0x0e },
	{ CCI_REG8(0xc542), 0x0e },
	{ CCI_REG8(0xc543), 0x0e },
	{ CCI_REG8(0xc544), 0x0e },
	{ CCI_REG8(0xc545), 0x0e },
	{ CCI_REG8(0xc546), 0x0e },
	{ CCI_REG8(0xc547), 0x0e },
	{ CCI_REG8(0xc548), 0x0e },
	{ CCI_REG8(0xc549), 0x0e },
	{ CCI_REG8(0xc57d), 0x80 },
	{ CCI_REG8(0xc581), 0x18 },
	{ CCI_REG8(0xc582), 0x18 },
	{ CCI_REG8(0xc583), 0x01 },
	{ CCI_REG8(0xc584), 0x01 },
	{ CCI_REG8(0xc587), 0x18 },
	{ CCI_REG8(0xc589), 0x18 },
	{ CCI_REG8(0xc58a), 0x0c },
	{ CCI_REG8(0xc58b), 0x08 },
	{ CCI_REG8(0xc58c), 0x04 },
	{ CCI_REG8(0xc58f), 0x28 },
	{ CCI_REG8(0xc590), 0x28 },
	{ CCI_REG8(0xc591), 0x28 },
	{ CCI_REG8(0xc592), 0x28 },
	{ CCI_REG8(0xc593), 0x04 },
	{ CCI_REG8(0xc594), 0x04 },
	{ CCI_REG8(0xc597), 0x2c },
	{ CCI_REG8(0xc598), 0x2c },
	{ CCI_REG8(0xc599), 0x2c },
	{ CCI_REG8(0xc59a), 0x28 },
	{ CCI_REG8(0xc59b), 0x20 },
	{ CCI_REG8(0xc59c), 0x18 },
	{ CCI_REG8(0xc5e4), 0x00 },
	{ CCI_REG8(0xc5e5), 0x01 },
	{ CCI_REG8(0xc5e8), 0x01 },
	{ CCI_REG8(0xc702), 0x00 },
	{ CCI_REG8(0xc726), 0x03 },
	{ CCI_REG8(0xc72b), 0xff },
	{ CCI_REG8(0xc72c), 0xff },
	{ CCI_REG8(0xc72d), 0xff },
	{ CCI_REG8(0xc72f), 0x08 },
	{ CCI_REG8(0xc736), 0x01 },
	{ CCI_REG8(0xc739), 0x18 },
	{ CCI_REG8(0xc73a), 0x49 },
	{ CCI_REG8(0xc73b), 0x92 },
	{ CCI_REG8(0xc73c), 0x24 },
	{ CCI_REG8(0xc746), 0x01 },
	{ CCI_REG8(0xc747), 0x04 },
	{ CCI_REG8(0xc749), 0x1c },
	{ CCI_REG8(0xc75b), 0x01 },
	{ CCI_REG8(0xc75c), 0x05 },
	{ CCI_REG8(0xc765), 0x2a },
	{ CCI_REG8(0xc773), 0x02 },
	{ CCI_REG8(0xc774), 0x03 },
	{ CCI_REG8(0xc78a), 0x03 },
	{ CCI_REG8(0xc78b), 0x04 },
	{ CCI_REG8(0xc798), 0x03 },
	{ CCI_REG8(0xc7a2), 0x01 },
	{ CCI_REG8(0xc7a6), 0x02 },
	{ CCI_REG8(0xc7a7), 0xff },
	{ CCI_REG8(0xc7a8), 0xff },
	{ CCI_REG8(0xc7a9), 0xff },
	{ CCI_REG8(0xc7aa), 0xff },
	{ CCI_REG8(0xc7ac), 0x02 },
	{ CCI_REG8(0xc7ad), 0xff },
	{ CCI_REG8(0xc7ae), 0xff },
	{ CCI_REG8(0xc7af), 0xff },
	{ CCI_REG8(0xc7b0), 0xff },
	{ CCI_REG8(0xc7b2), 0x01 },
	{ CCI_REG8(0xc7b3), 0xff },
	{ CCI_REG8(0xc7b4), 0xff },
	{ CCI_REG8(0xc7b5), 0xff },
	{ CCI_REG8(0xc7b6), 0xff },
	{ CCI_REG8(0xc7c4), 0x00 },
	{ CCI_REG8(0xc7c5), 0xff },
	{ CCI_REG8(0xc7e2), 0x01 },
	{ CCI_REG8(0xc855), 0x07 },
	{ CCI_REG8(0xc8a4), 0x07 },
	{ CCI_REG8(0xc95a), 0x77 },
	{ CCI_REG8(0xc95b), 0x77 },
	{ CCI_REG8(0xc9b9), 0x28 },
	{ CCI_REG8(0xc9fe), 0x0a },
	{ CCI_REG8(0xc9ff), 0x0e },
	{ CCI_REG8(0xca00), 0x1a },
	{ CCI_REG8(0xca02), 0x1a },
	{ CCI_REG8(0xca17), 0x03 },
	{ CCI_REG8(0xca18), 0x1a },
	{ CCI_REG8(0xca19), 0x1a },
	{ CCI_REG8(0x380c), 0x05 },
	{ CCI_REG8(0x380d), 0x64 },
	{ CCI_REG8(0x380e), 0x0e },
	{ CCI_REG8(0x380f), 0x26 },
	{ CCI_REG8(0x384c), 0x05 },
	{ CCI_REG8(0x384d), 0x64 },
	{ CCI_REG8(0x4546), 0x08 },
	{ CCI_REG8(0x4547), 0xa0 },
	{ CCI_REG8(0x3840), 0x00 },
	{ CCI_REG8(0x3500), 0x00 },
	{ CCI_REG8(0x3501), 0x0d },
	{ CCI_REG8(0x3502), 0x00 },
	{ CCI_REG8(0x3508), 0x01 },
	{ CCI_REG8(0x3509), 0x00 },
	{ CCI_REG8(0x3869), 0x18 },
};

/*
 * Stream on: refill and launch groups 0 and 1, then leave software
 * standby (0x0100 = 0x01).
 */
static const struct cci_reg_sequence ov50h40_stream_on_regs[] = {
	{ CCI_REG8(0x3208), 0x00 },
	{ CCI_REG8(0x3a88), 0x44 },
	{ CCI_REG8(0x3bd7), 0xc8 },
	{ CCI_REG8(0x3bdc), 0x40 },
	{ CCI_REG8(0x3be5), 0x01 },
	{ CCI_REG8(0x392e), 0x5a },
	{ CCI_REG8(0x3208), 0x10 },
	{ CCI_REG8(0x3208), 0x01 },
	{ CCI_REG8(0x3a88), 0x04 },
	{ CCI_REG8(0x3bd7), 0x48 },
	{ CCI_REG8(0x3bdc), 0x00 },
	{ CCI_REG8(0x3be5), 0x00 },
	{ CCI_REG8(0x392e), 0x0a },
	{ CCI_REG8(0x3208), 0x11 },
	{ CCI_REG8(0x320d), 0x01 },
	{ CCI_REG8(0x3209), 0x01 },
	{ CCI_REG8(0x320a), 0x01 },
	{ CCI_REG8(0x320d), 0x91 },
	{ CCI_REG8(0x320e), 0xa0 },
	{ CCI_REG8(0x0100), 0x01 },
};

/* Stream off: enter software standby and reset group 0 */
static const struct cci_reg_sequence ov50h40_stream_off_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x320d), 0x01 },
	{ CCI_REG8(0x3208), 0x00 },
	{ CCI_REG8(0x3208), 0x10 },
};

static const struct ov50h40_mode ov50h40_modes[] = {
	{
		.width = 4096,
		.height = 3072,
		.hts = 6892,
		.vts_def = 2776,
		.vts_min = 1388,
		.shift = 2,
		.exposure_def = 10240,
		.exposure_step = 8,
		.crop = {
			.left = 0,
			.top = 0,
			.width = OV50H40_NATIVE_WIDTH,
			.height = OV50H40_NATIVE_HEIGHT,
		},
		.reg_list = {
			.regs = ov50h40_4096x3072_regs,
			.num_regs = ARRAY_SIZE(ov50h40_4096x3072_regs),
		},
		.post_reg_list = {
			.regs = ov50h40_4096x3072_post_regs,
			.num_regs = ARRAY_SIZE(ov50h40_4096x3072_post_regs),
		},
	},
	{
		.width = 8192,
		.height = 6144,
		.hts = 10568,
		.vts_def = 3624,
		.vts_min = 3624,
		.shift = 1,
		.exposure_def = 6656,
		.exposure_step = 2,
		.crop = {
			.left = 0,
			.top = 0,
			.width = OV50H40_NATIVE_WIDTH,
			.height = OV50H40_NATIVE_HEIGHT,
		},
		.reg_list = {
			.regs = ov50h40_8192x6144_regs,
			.num_regs = ARRAY_SIZE(ov50h40_8192x6144_regs),
		},
	},
};

static int ov50h40_update_exposure_range(struct ov50h40 *ov50h40, s32 vblank)
{
	const struct ov50h40_mode *mode = ov50h40->mode;
	u32 vts = (mode->height + vblank) >> mode->shift;
	u32 exposure_max;

	exposure_max = (vts - OV50H40_EXPOSURE_MARGIN) << mode->shift;
	exposure_max = rounddown(exposure_max, mode->exposure_step);

	return __v4l2_ctrl_modify_range(ov50h40->exposure, OV50H40_EXPOSURE_MIN,
					exposure_max, mode->exposure_step,
					min(mode->exposure_def, exposure_max));
}

/*
 * Write frame length, exposure and gain in one group hold, launched at the
 * next frame boundary, in the order used by the vendor stack.
 */
static int ov50h40_write_exposure_group(struct ov50h40 *ov50h40, s32 vblank,
					s32 exposure, s32 again)
{
	const struct ov50h40_mode *mode = ov50h40->mode;
	u32 vts = (mode->height + vblank) >> mode->shift;
	int ret = 0;

	cci_write(ov50h40->regmap, OV50H40_REG_GROUP_HOLD,
		  OV50H40_GROUP0_START, &ret);
	cci_write(ov50h40->regmap, OV50H40_REG_VTS_HIGH, vts >> 16, &ret);
	cci_write(ov50h40->regmap, OV50H40_REG_VTS, vts & 0xffff, &ret);
	cci_write(ov50h40->regmap, OV50H40_REG_EXPOSURE,
		  exposure >> mode->shift, &ret);
	cci_write(ov50h40->regmap, OV50H40_REG_ANALOGUE_GAIN,
		  again & OV50H40_ANALOGUE_GAIN_MASK, &ret);
	cci_write(ov50h40->regmap, OV50H40_REG_DIGITAL_GAIN,
		  OV50H40_DIGITAL_GAIN_1X, &ret);
	cci_write(ov50h40->regmap, OV50H40_REG_GROUP_HOLD,
		  OV50H40_GROUP0_END, &ret);
	cci_write(ov50h40->regmap, OV50H40_REG_GROUP_HOLD,
		  OV50H40_GROUP0_LAUNCH, &ret);

	return ret;
}

static int ov50h40_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov50h40 *ov50h40 = container_of(ctrl->handler, struct ov50h40,
					       ctrl_handler);
	s32 vblank = ov50h40->vblank->cur.val;
	s32 exposure = ov50h40->exposure->cur.val;
	s32 again = ov50h40->again->cur.val;
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		ret = ov50h40_update_exposure_range(ov50h40, ctrl->val);
		if (ret)
			return ret;
	}

	/* Controls are applied when the sensor is powered up for streaming */
	if (!pm_runtime_get_if_active(ov50h40->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_ANALOGUE_GAIN:
		/*
		 * Written as one group by ov50h40_enable_streams() before
		 * streaming starts, and on every change while streaming.
		 */
		if (!ov50h40->streaming)
			break;

		if (ctrl->id == V4L2_CID_VBLANK)
			vblank = ctrl->val;
		else if (ctrl->id == V4L2_CID_EXPOSURE)
			exposure = ctrl->val;
		else
			again = ctrl->val;

		ret = ov50h40_write_exposure_group(ov50h40, vblank, exposure,
						   again);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(ov50h40->regmap, OV50H40_REG_TEST_PATTERN,
			  ctrl->val ? OV50H40_TEST_PATTERN_BARS :
				      OV50H40_TEST_PATTERN_DISABLE, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(ov50h40->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov50h40_ctrl_ops = {
	.s_ctrl = ov50h40_set_ctrl,
};

static int ov50h40_init_controls(struct ov50h40 *ov50h40)
{
	struct v4l2_ctrl_handler *hdl = &ov50h40->ctrl_handler;
	const struct ov50h40_mode *mode = ov50h40->mode;
	struct v4l2_fwnode_device_properties props;
	s64 hblank, vblank_min, vblank_max, vblank_def;
	struct v4l2_ctrl *ctrl;
	int ret;

	ret = v4l2_fwnode_device_parse(ov50h40->dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 10);

	ctrl = v4l2_ctrl_new_int_menu(hdl, &ov50h40_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(ov50h40_link_freq_menu) - 1,
				      0, ov50h40_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, &ov50h40_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  OV50H40_PIXEL_RATE, OV50H40_PIXEL_RATE, 1,
			  OV50H40_PIXEL_RATE);

	hblank = mode->hts - mode->width;
	ov50h40->hblank = v4l2_ctrl_new_std(hdl, &ov50h40_ctrl_ops,
					    V4L2_CID_HBLANK, hblank, hblank, 1,
					    hblank);
	if (ov50h40->hblank)
		ov50h40->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* The frame length register counts units of 1 << shift lines */
	vblank_min = (mode->vts_min << mode->shift) - mode->height;
	vblank_max = ((s64)OV50H40_VTS_MAX << mode->shift) - mode->height;
	vblank_def = (mode->vts_def << mode->shift) - mode->height;
	ov50h40->vblank = v4l2_ctrl_new_std(hdl, &ov50h40_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_min,
					    vblank_max, 1 << mode->shift,
					    vblank_def);

	ov50h40->exposure = v4l2_ctrl_new_std(hdl, &ov50h40_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      OV50H40_EXPOSURE_MIN,
					      (mode->vts_def -
					       OV50H40_EXPOSURE_MARGIN) <<
					      mode->shift,
					      mode->exposure_step,
					      mode->exposure_def);

	ov50h40->again = v4l2_ctrl_new_std(hdl, &ov50h40_ctrl_ops,
					   V4L2_CID_ANALOGUE_GAIN,
					   OV50H40_ANALOGUE_GAIN_MIN,
					   OV50H40_ANALOGUE_GAIN_MAX,
					   OV50H40_ANALOGUE_GAIN_STEP,
					   OV50H40_ANALOGUE_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(hdl, &ov50h40_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ov50h40_test_pattern_menu) - 1,
				     0, 0, ov50h40_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(hdl, &ov50h40_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	ov50h40->sd.ctrl_handler = hdl;

	return 0;
}

static void ov50h40_update_pad_format(const struct ov50h40_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = OV50H40_MBUS_CODE;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int ov50h40_write_reg_list(struct ov50h40 *ov50h40,
				  const struct ov50h40_reg_list *reg_list)
{
	if (!reg_list->num_regs)
		return 0;

	return cci_multi_reg_write(ov50h40->regmap, reg_list->regs,
				   reg_list->num_regs, NULL);
}

static int ov50h40_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);
	const struct ov50h40_mode *mode = ov50h40->mode;
	int ret;

	ret = pm_runtime_resume_and_get(ov50h40->dev);
	if (ret)
		return ret;

	/*
	 * The vendor stack order is: common settings, (QSC data), mode table,
	 * post-mode settings, one exposure group and the stream on table. The
	 * 4096x3072 mode table opens a group that only the post-mode and
	 * stream on tables close and launch, so none of them may be skipped
	 * or reordered.
	 */
	ret = cci_multi_reg_write(ov50h40->regmap, ov50h40_init_regs,
				  ARRAY_SIZE(ov50h40_init_regs), NULL);
	if (ret) {
		dev_err(ov50h40->dev, "failed to write init registers: %d\n",
			ret);
		goto error;
	}

	ret = ov50h40_write_reg_list(ov50h40, &mode->reg_list);
	if (ret) {
		dev_err(ov50h40->dev, "failed to set mode: %d\n", ret);
		goto error;
	}

	ret = ov50h40_write_reg_list(ov50h40, &mode->post_reg_list);
	if (ret) {
		dev_err(ov50h40->dev, "failed to write post-mode registers: %d\n",
			ret);
		goto error;
	}

	/* Applies the controls that are not part of the exposure group */
	ret = __v4l2_ctrl_handler_setup(ov50h40->sd.ctrl_handler);
	if (ret)
		goto error;

	ret = ov50h40_write_exposure_group(ov50h40, ov50h40->vblank->cur.val,
					   ov50h40->exposure->cur.val,
					   ov50h40->again->cur.val);
	if (ret) {
		dev_err(ov50h40->dev, "failed to set exposure: %d\n", ret);
		goto error;
	}

	ret = cci_multi_reg_write(ov50h40->regmap, ov50h40_stream_on_regs,
				  ARRAY_SIZE(ov50h40_stream_on_regs), NULL);
	if (ret) {
		dev_err(ov50h40->dev, "failed to start streaming: %d\n", ret);
		goto error;
	}

	ov50h40->streaming = true;

	return 0;

error:
	pm_runtime_put_autosuspend(ov50h40->dev);

	return ret;
}

static int ov50h40_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);
	int ret;

	ov50h40->streaming = false;

	ret = cci_multi_reg_write(ov50h40->regmap, ov50h40_stream_off_regs,
				  ARRAY_SIZE(ov50h40_stream_off_regs), NULL);
	if (ret)
		dev_err(ov50h40->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(ov50h40->dev);

	return ret;
}

static int ov50h40_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);
	const struct ov50h40_mode *mode;
	s64 hblank, vblank_min, vblank_max, vblank_def;
	int ret;

	mode = v4l2_find_nearest_size(ov50h40_modes, ARRAY_SIZE(ov50h40_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && ov50h40->streaming)
		return -EBUSY;

	ov50h40_update_pad_format(mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	ov50h40->mode = mode;

	hblank = mode->hts - mode->width;
	ret = __v4l2_ctrl_modify_range(ov50h40->hblank, hblank, hblank, 1,
				       hblank);
	if (ret)
		return ret;

	vblank_min = (mode->vts_min << mode->shift) - mode->height;
	vblank_max = ((s64)OV50H40_VTS_MAX << mode->shift) - mode->height;
	vblank_def = (mode->vts_def << mode->shift) - mode->height;
	ret = __v4l2_ctrl_modify_range(ov50h40->vblank, vblank_min, vblank_max,
				       1 << mode->shift, vblank_def);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(ov50h40->vblank, vblank_def);
	if (ret)
		return ret;

	/* The exposure step depends on the mode even if VBLANK is unchanged */
	return ov50h40_update_exposure_range(ov50h40, vblank_def);
}

static int ov50h40_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OV50H40_NATIVE_WIDTH;
		sel->r.height = OV50H40_NATIVE_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int ov50h40_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = OV50H40_MBUS_CODE;

	return 0;
}

static int ov50h40_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(ov50h40_modes))
		return -EINVAL;

	if (fse->code != OV50H40_MBUS_CODE)
		return -EINVAL;

	fse->min_width = ov50h40_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ov50h40_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ov50h40_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = OV50H40_MBUS_CODE,
			.width = ov50h40_modes[0].width,
			.height = ov50h40_modes[0].height,
		},
	};

	return ov50h40_set_pad_format(sd, state, &fmt);
}

static const struct v4l2_subdev_video_ops ov50h40_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops ov50h40_pad_ops = {
	.enum_mbus_code = ov50h40_enum_mbus_code,
	.enum_frame_size = ov50h40_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ov50h40_set_pad_format,
	.get_selection = ov50h40_get_selection,
	.enable_streams = ov50h40_enable_streams,
	.disable_streams = ov50h40_disable_streams,
};

static const struct v4l2_subdev_ops ov50h40_subdev_ops = {
	.video = &ov50h40_video_ops,
	.pad = &ov50h40_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov50h40_internal_ops = {
	.init_state = ov50h40_init_state,
};

static const struct media_entity_operations ov50h40_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ov50h40_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov50h40 *ov50h40 = to_ov50h40(sd);
	int ret;

	ret = regulator_bulk_enable(OV50H40_NUM_SUPPLIES, ov50h40->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(ov50h40->xclk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		goto disable_supplies;
	}

	/* The reference stack releases reset 5 ms after starting XCLK */
	fsleep(5000);

	gpiod_set_value_cansleep(ov50h40->reset_gpio, 0);

	/* Wait before the first SCCB transaction, 5.6 ms in the reference stack */
	fsleep(6000);

	return 0;

disable_supplies:
	regulator_bulk_disable(OV50H40_NUM_SUPPLIES, ov50h40->supplies);

	return ret;
}

static int ov50h40_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov50h40 *ov50h40 = to_ov50h40(sd);

	clk_disable_unprepare(ov50h40->xclk);
	fsleep(1000);

	gpiod_set_value_cansleep(ov50h40->reset_gpio, 1);

	regulator_bulk_disable(OV50H40_NUM_SUPPLIES, ov50h40->supplies);

	return 0;
}

static int ov50h40_identify_sensor(struct ov50h40 *ov50h40)
{
	u64 val;
	int ret;

	ret = cci_read(ov50h40->regmap, OV50H40_REG_CHIP_ID, &val, NULL);
	if (ret)
		return dev_err_probe(ov50h40->dev, ret,
				     "failed to read chip id\n");

	if ((val & OV50H40_CHIP_ID_MASK) != OV50H40_CHIP_ID)
		return dev_err_probe(ov50h40->dev, -ENODEV,
				     "chip id mismatch: %x!=%llx\n",
				     OV50H40_CHIP_ID,
				     val & OV50H40_CHIP_ID_MASK);

	dev_dbg(ov50h40->dev, "chip id 0x%06llx\n", val);

	return 0;
}

static int ov50h40_check_hwcfg(struct ov50h40 *ov50h40)
{
	struct fwnode_handle *fwnode = dev_fwnode(ov50h40->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_CPHY,
	};
	unsigned long freq_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_endpoint_by_id(fwnode, 0, 0, 0);
	if (!ep)
		return dev_err_probe(ov50h40->dev, -EINVAL,
				     "missing endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(ov50h40->dev, ret,
				     "failed to parse endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != OV50H40_NUM_DATA_LANES) {
		ret = dev_err_probe(ov50h40->dev, -EINVAL,
				    "only %u C-PHY trios are supported, got %u\n",
				    OV50H40_NUM_DATA_LANES,
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto out_free;
	}

	ret = v4l2_link_freq_to_bitmap(ov50h40->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       ov50h40_link_freq_menu,
				       ARRAY_SIZE(ov50h40_link_freq_menu),
				       &freq_bitmap);

out_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int ov50h40_probe(struct i2c_client *client)
{
	struct ov50h40 *ov50h40;
	unsigned long freq;
	unsigned int i;
	int ret;

	ov50h40 = devm_kzalloc(&client->dev, sizeof(*ov50h40), GFP_KERNEL);
	if (!ov50h40)
		return -ENOMEM;

	ov50h40->dev = &client->dev;
	ov50h40->mode = &ov50h40_modes[0];

	v4l2_i2c_subdev_init(&ov50h40->sd, client, &ov50h40_subdev_ops);

	ret = ov50h40_check_hwcfg(ov50h40);
	if (ret)
		return ret;

	ov50h40->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ov50h40->regmap))
		return dev_err_probe(ov50h40->dev, PTR_ERR(ov50h40->regmap),
				     "failed to init CCI\n");

	ov50h40->xclk = devm_v4l2_sensor_clk_get(ov50h40->dev, NULL);
	if (IS_ERR(ov50h40->xclk))
		return dev_err_probe(ov50h40->dev, PTR_ERR(ov50h40->xclk),
				     "failed to get clock\n");

	freq = clk_get_rate(ov50h40->xclk);
	if (freq != OV50H40_XCLK_FREQ)
		return dev_err_probe(ov50h40->dev, -EINVAL,
				     "clock frequency %lu Hz is not supported\n",
				     freq);

	ov50h40->reset_gpio = devm_gpiod_get_optional(ov50h40->dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(ov50h40->reset_gpio))
		return dev_err_probe(ov50h40->dev,
				     PTR_ERR(ov50h40->reset_gpio),
				     "failed to get reset GPIO\n");

	for (i = 0; i < OV50H40_NUM_SUPPLIES; i++)
		ov50h40->supplies[i].supply = ov50h40_supply_names[i];

	ret = devm_regulator_bulk_get(ov50h40->dev, OV50H40_NUM_SUPPLIES,
				      ov50h40->supplies);
	if (ret)
		return dev_err_probe(ov50h40->dev, ret,
				     "failed to get supplies\n");

	/* The sensor must be powered on to read the chip ID register */
	ret = ov50h40_power_on(ov50h40->dev);
	if (ret)
		return ret;

	ret = ov50h40_identify_sensor(ov50h40);
	if (ret)
		goto power_off;

	ret = ov50h40_init_controls(ov50h40);
	if (ret) {
		dev_err_probe(ov50h40->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	ov50h40->sd.state_lock = ov50h40->ctrl_handler.lock;
	ov50h40->sd.internal_ops = &ov50h40_internal_ops;
	ov50h40->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov50h40->sd.entity.ops = &ov50h40_entity_ops;
	ov50h40->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ov50h40->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&ov50h40->sd.entity, 1, &ov50h40->pad);
	if (ret) {
		dev_err_probe(ov50h40->dev, ret, "failed to init entity pads\n");
		goto free_ctrls;
	}

	ret = v4l2_subdev_init_finalize(&ov50h40->sd);
	if (ret) {
		dev_err_probe(ov50h40->dev, ret, "failed to init subdev\n");
		goto cleanup_entity;
	}

	pm_runtime_set_active(ov50h40->dev);
	pm_runtime_get_noresume(ov50h40->dev);
	pm_runtime_enable(ov50h40->dev);

	ret = v4l2_async_register_subdev_sensor(&ov50h40->sd);
	if (ret) {
		dev_err_probe(ov50h40->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto disable_pm;
	}

	pm_runtime_set_autosuspend_delay(ov50h40->dev, 1000);
	pm_runtime_use_autosuspend(ov50h40->dev);
	pm_runtime_put_autosuspend(ov50h40->dev);

	return 0;

disable_pm:
	pm_runtime_disable(ov50h40->dev);
	pm_runtime_set_suspended(ov50h40->dev);
	pm_runtime_put_noidle(ov50h40->dev);
	v4l2_subdev_cleanup(&ov50h40->sd);
cleanup_entity:
	media_entity_cleanup(&ov50h40->sd.entity);
free_ctrls:
	v4l2_ctrl_handler_free(&ov50h40->ctrl_handler);
power_off:
	ov50h40_power_off(ov50h40->dev);

	return ret;
}

static void ov50h40_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov50h40 *ov50h40 = to_ov50h40(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&ov50h40->ctrl_handler);

	pm_runtime_disable(ov50h40->dev);
	if (!pm_runtime_status_suspended(ov50h40->dev)) {
		ov50h40_power_off(ov50h40->dev);
		pm_runtime_set_suspended(ov50h40->dev);
	}
	pm_runtime_dont_use_autosuspend(ov50h40->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(ov50h40_pm_ops, ov50h40_power_off,
				 ov50h40_power_on, NULL);

static const struct of_device_id ov50h40_of_match[] = {
	{ .compatible = "ovti,ov50h40" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov50h40_of_match);

static struct i2c_driver ov50h40_i2c_driver = {
	.driver = {
		.name = "ov50h40",
		.pm = pm_ptr(&ov50h40_pm_ops),
		.of_match_table = ov50h40_of_match,
	},
	.probe = ov50h40_probe,
	.remove = ov50h40_remove,
};
module_i2c_driver(ov50h40_i2c_driver);

MODULE_AUTHOR("Luka Panio <lukapanio@gmail.com>");
MODULE_DESCRIPTION("OmniVision OV50H40 image sensor driver");
MODULE_LICENSE("GPL");
