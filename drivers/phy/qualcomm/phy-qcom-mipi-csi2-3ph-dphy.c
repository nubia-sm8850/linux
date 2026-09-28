// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM Camera Subsystem - CSIPHY Module 3phase v1.0
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2016-2026 Linaro Ltd.
 */

#include <dt-bindings/media/video-interfaces.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/time64.h>

#include "phy-qcom-mipi-csi2.h"

#define CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(offset, n)	((offset) + 0x4 * (n))
#define CSIPHY_3PH_CMN_CSI_COMMON_CTRL0_PHY_SW_RESET	BIT(0)
#define CSIPHY_3PH_CMN_CSI_COMMON_CTRL5_CLK_ENABLE	BIT(7)
#define CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_COMMON_PWRDN_B	BIT(0)
#define CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_SHOW_REV_ID	BIT(1)
#define CSIPHY_3PH_CMN_CSI_COMMON_CTRL10_IRQ_CLEAR_CMD	BIT(0)
#define CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(offset, common_status_offset, n) \
	((offset) + (common_status_offset) + 0x4 * (n))

#define CSIPHY_2PH_LN_CSI_2PHASE_CTRL9n(n)		((0x200 * (n)) + 0x24)

/*
 * 3 phase CSI has 19 common status regs with only 0-10 being used
 * and 11-18 being reserved.
 */
#define CSI_COMMON_STATUS_NUM				11
/*
 * There are a number of common control registers
 * The offset to clear the CSIPHY IRQ status starts @ 22
 * So to clear CSI_COMMON_STATUS0 this is CSI_COMMON_CONTROL22, STATUS1 is
 * CONTROL23 and so on
 */
#define CSI_CTRL_STATUS_INDEX				22

/*
 * There are 43 COMMON_CTRL registers with regs after # 33 being reserved
 */
#define CSI_CTRL_MAX					33

/* C-PHY settle count = window / T_timer - CSIPHY_CPHY_SETTLE_ADJ */
#define CSIPHY_CPHY_SETTLE_ADJ				10

#define CSIPHY_DEFAULT_PARAMS				0
#define CSIPHY_SETTLE_CNT_LOWER_BYTE			2
#define CSIPHY_SKEW_CAL					7

/* 4nm 2PH v 2.1.2 2p5Gbps 4 lane DPHY mode */
static const struct
mipi_csi2phy_lane_regs lane_regs_x1e80100[] = {
	/* Power up lanes 2ph mode */
	{.reg_addr = 0x101c, .reg_data = 0x7a, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x1018, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},

	{.reg_addr = 0x0094, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x00a0, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0090, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0098, .reg_data = 0x08, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0094, .reg_data = 0x07, .delay_us = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0030, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0000, .reg_data = 0x8e, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0038, .reg_data = 0xfe, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x002c, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0034, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x001c, .reg_data = 0x0a, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0014, .reg_data = 0x60, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x003c, .reg_data = 0xb8, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0004, .reg_data = 0x0c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0020, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0008, .reg_data = 0x10, .param_type = CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{.reg_addr = 0x0010, .reg_data = 0x52, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0094, .reg_data = 0xd7, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x005c, .reg_data = 0x00, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0060, .reg_data = 0xbd, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0064, .reg_data = 0x7f, .param_type = CSIPHY_SKEW_CAL},

	{.reg_addr = 0x0e94, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0ea0, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e90, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e98, .reg_data = 0x08, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e94, .reg_data = 0x07, .delay_us =  0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e30, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e28, .reg_data = 0x04, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e00, .reg_data = 0x80, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e0c, .reg_data = 0xff, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e38, .reg_data = 0x1f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e2c, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e34, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e1c, .reg_data = 0x0a, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e14, .reg_data = 0x60, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e3c, .reg_data = 0xb8, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e04, .reg_data = 0x0c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e20, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e08, .reg_data = 0x10, .param_type = CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{.reg_addr = 0x0e10, .reg_data = 0x52, .param_type = CSIPHY_DEFAULT_PARAMS},

	{.reg_addr = 0x0494, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x04a0, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0490, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0498, .reg_data = 0x08, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0494, .reg_data = 0x07, .delay_us =  0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0430, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0400, .reg_data = 0x8e, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0438, .reg_data = 0xfe, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x042c, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0434, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x041c, .reg_data = 0x0a, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0414, .reg_data = 0x60, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x043c, .reg_data = 0xb8, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0404, .reg_data = 0x0c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0420, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0408, .reg_data = 0x10, .param_type = CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{.reg_addr = 0x0410, .reg_data = 0x52, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0494, .reg_data = 0xd7, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x045c, .reg_data = 0x00, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0460, .reg_data = 0xbd, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0464, .reg_data = 0x7f, .param_type = CSIPHY_SKEW_CAL},

	{.reg_addr = 0x0894, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x08a0, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0890, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0898, .reg_data = 0x08, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0894, .reg_data = 0x07, .delay_us =  0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0830, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0800, .reg_data = 0x8e, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0838, .reg_data = 0xfe, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x082c, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0834, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x081c, .reg_data = 0x0a, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0814, .reg_data = 0x60, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x083c, .reg_data = 0xb8, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0804, .reg_data = 0x0c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0820, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0808, .reg_data = 0x10, .param_type = CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{.reg_addr = 0x0810, .reg_data = 0x52, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0894, .reg_data = 0xd7, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x085c, .reg_data = 0x00, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0860, .reg_data = 0xbd, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0864, .reg_data = 0x7f, .param_type = CSIPHY_SKEW_CAL},

	{.reg_addr = 0x0c94, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0ca0, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c90, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c98, .reg_data = 0x08, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c94, .reg_data = 0x07, .delay_us =  0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c30, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c00, .reg_data = 0x8e, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c38, .reg_data = 0xfe, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c2c, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c34, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c1c, .reg_data = 0x0a, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c14, .reg_data = 0x60, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c3c, .reg_data = 0xb8, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c04, .reg_data = 0x0c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c20, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c08, .reg_data = 0x10, .param_type = CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{.reg_addr = 0x0c10, .reg_data = 0x52, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c94, .reg_data = 0xd7, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0c5c, .reg_data = 0x00, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0c60, .reg_data = 0xbd, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0c64, .reg_data = 0x7f, .param_type = CSIPHY_SKEW_CAL},
};

/* 3nm 2PH v2.4.0 2p5Gbps 4 lane DPHY mode */
static const struct
mipi_csi2phy_lane_regs lane_regs_kaanapali[] = {
	/* Select 2PH mode and power up lanes */
	{.reg_addr = 0x1084, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x108c, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x101c, .reg_data = 0x7a, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x1018, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},

	/* LN 0 */
	{.reg_addr = 0x0094, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x00a0, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0090, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0098, .reg_data = 0x08, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0094, .reg_data = 0x07, .delay_us = 209,
	 .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0030, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0000, .reg_data = 0x8c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0038, .reg_data = 0xfe, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x002c, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0034, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x001c, .reg_data = 0x0a, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0014, .reg_data = 0x60, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x003c, .reg_data = 0xb8, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0004, .reg_data = 0x0c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0020, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0008, .reg_data = 0x19, .param_type = CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{.reg_addr = 0x0010, .reg_data = 0x52, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0094, .reg_data = 0xd7, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x005c, .reg_data = 0x54, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0060, .reg_data = 0xfd, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0064, .reg_data = 0x7f, .param_type = CSIPHY_SKEW_CAL},

	/* LN 2 */
	{.reg_addr = 0x0494, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x04a0, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0490, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0498, .reg_data = 0x08, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0494, .reg_data = 0x07, .delay_us = 209,
	 .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0430, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0400, .reg_data = 0x8c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0438, .reg_data = 0xfe, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x042c, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0434, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x041c, .reg_data = 0x0a, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0414, .reg_data = 0x60, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x043c, .reg_data = 0xb8, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0404, .reg_data = 0x0c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0420, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0408, .reg_data = 0x19, .param_type = CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{.reg_addr = 0x0410, .reg_data = 0x52, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0494, .reg_data = 0xd7, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x045c, .reg_data = 0x54, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0460, .reg_data = 0xfd, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0464, .reg_data = 0x7f, .param_type = CSIPHY_SKEW_CAL},

	/* LN 4 */
	{.reg_addr = 0x0894, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x08a0, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0890, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0898, .reg_data = 0x08, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0894, .reg_data = 0x07, .delay_us = 209,
	 .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0830, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0800, .reg_data = 0x8c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0838, .reg_data = 0xfe, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x082c, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0834, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x081c, .reg_data = 0x0a, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0814, .reg_data = 0x60, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x083c, .reg_data = 0xb8, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0804, .reg_data = 0x0c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0820, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0808, .reg_data = 0x19, .param_type = CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{.reg_addr = 0x0810, .reg_data = 0x52, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0894, .reg_data = 0xd7, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x085c, .reg_data = 0x54, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0860, .reg_data = 0xfd, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0864, .reg_data = 0x7f, .param_type = CSIPHY_SKEW_CAL},

	/* LN 6 */
	{.reg_addr = 0x0c94, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0ca0, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c90, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c98, .reg_data = 0x08, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c94, .reg_data = 0x07, .delay_us = 209,
	 .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c30, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c00, .reg_data = 0x8c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c38, .reg_data = 0xfe, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c2c, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c34, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c1c, .reg_data = 0x0a, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c14, .reg_data = 0x60, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c3c, .reg_data = 0xb8, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c04, .reg_data = 0x0c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c20, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c08, .reg_data = 0x19, .param_type = CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{.reg_addr = 0x0c10, .reg_data = 0x52, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0c94, .reg_data = 0xd7, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0c5c, .reg_data = 0x54, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0c60, .reg_data = 0xfd, .param_type = CSIPHY_SKEW_CAL},
	{.reg_addr = 0x0c64, .reg_data = 0x7f, .param_type = CSIPHY_SKEW_CAL},

	/* LN CLK */
	{.reg_addr = 0x0e94, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0ea0, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e90, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e98, .reg_data = 0x08, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e94, .reg_data = 0x07, .delay_us = 209,
	 .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e30, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e28, .reg_data = 0x04, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e00, .reg_data = 0x80, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e0c, .reg_data = 0xff, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e38, .reg_data = 0x1f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e2c, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e34, .reg_data = 0x0f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e1c, .reg_data = 0x0a, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e14, .reg_data = 0x60, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e3c, .reg_data = 0xb8, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e04, .reg_data = 0x0c, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e20, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0e08, .reg_data = 0x19, .param_type = CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{.reg_addr = 0x0e10, .reg_data = 0x52, .param_type = CSIPHY_DEFAULT_PARAMS},
};

/*
 * 3nm 3PH v2.4.0 C-PHY mode, per-trio init sequence.
 * Offsets are relative to the trio base.
 */
static const struct
mipi_csi2phy_lane_regs lane_regs_kaanapali_3ph[] = {
	{.reg_addr = 0x0094, .reg_data = 0x0d, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x00f4, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x00f8, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x00fc, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x00f0, .reg_data = 0xef, .delay_us = 211,
	 .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0004, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x00e4, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x00e8, .reg_data = 0x7f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x00ec, .reg_data = 0x7f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0018, .reg_data = 0x3e, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x001c, .reg_data = 0x41, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0020, .reg_data = 0x41, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0024, .reg_data = 0x7f, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0028, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x002c, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0064, .reg_data = 0x01, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0044, .reg_data = 0xb2, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0110, .reg_data = 0x35, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x00bc, .reg_data = 0xd0, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0054, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0040, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0060, .reg_data = 0xa8, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0084, .reg_data = 0x00, .param_type = CSIPHY_DEFAULT_PARAMS},
	{.reg_addr = 0x0090, .reg_data = 0x02, .param_type = CSIPHY_DEFAULT_PARAMS},
};

/* Data-rate dependent C-PHY trio registers (AFE, CDR, equaliser) */
static const u16 cphy_rate_offs_kaanapali[CSI2_CPHY_RATE_NREGS] = {
	0x0068, 0x006c, 0x0070, 0x0074, 0x0078,
	0x0088, 0x008c, 0x0090, 0x0010, 0x0014,
};

static const struct mipi_csi2phy_cphy_rate_regs cphy_rates_kaanapali[] = {
	/* max MS/s        0x68  0x6c  0x70  0x74  0x78  0x88  0x8c  0x90  0x10  0x14 */
	{    80, { 0xf1, 0x38, 0x00, 0x00, 0x5f, 0x00, 0x77, 0x02, 0x00, 0x6b } },
	{   100, { 0xf1, 0x38, 0x00, 0x00, 0x5f, 0x00, 0x77, 0x02, 0x00, 0x6b } },
	{   200, { 0xf1, 0x38, 0x00, 0x00, 0x5f, 0x00, 0x77, 0x02, 0x00, 0x33 } },
	{   300, { 0xf1, 0x38, 0x00, 0x00, 0x5f, 0x00, 0x77, 0x02, 0x00, 0x20 } },
	{   350, { 0xf1, 0x38, 0x00, 0x00, 0x5f, 0x00, 0x77, 0x02, 0x00, 0x20 } },
	{   400, { 0xf1, 0x38, 0x00, 0x00, 0x5f, 0x00, 0x77, 0x02, 0x00, 0x20 } },
	{   500, { 0xf1, 0x38, 0x00, 0x00, 0x5f, 0x00, 0x77, 0x02, 0x00, 0x20 } },
	{   600, { 0xf1, 0x38, 0x00, 0x00, 0x5f, 0x00, 0x77, 0x02, 0x00, 0x20 } },
	{   700, { 0xf1, 0x38, 0x00, 0x00, 0x5f, 0x00, 0x77, 0x02, 0x00, 0x20 } },
	{   800, { 0xf1, 0x38, 0x00, 0x00, 0x5f, 0x00, 0x77, 0x02, 0x00, 0x09 } },
	{   900, { 0xf1, 0x38, 0x00, 0x00, 0x5f, 0x00, 0x77, 0x02, 0x00, 0x09 } },
	{  1000, { 0xf1, 0x38, 0x00, 0x00, 0x5f, 0x00, 0x77, 0x02, 0x00, 0x09 } },
	{  1200, { 0xf1, 0x38, 0x00, 0x00, 0x3f, 0x00, 0x77, 0x02, 0x00, 0x09 } },
	{  1500, { 0xf1, 0x38, 0x00, 0x00, 0x3f, 0x00, 0x77, 0x02, 0x00, 0x09 } },
	{  1700, { 0xf1, 0x1d, 0x00, 0x00, 0x26, 0x00, 0x03, 0x02, 0x00, 0x09 } },
	{  2000, { 0xf1, 0x1d, 0x00, 0x00, 0x26, 0x00, 0x03, 0x02, 0x00, 0x00 } },
	{  2100, { 0xf1, 0x1d, 0x00, 0x00, 0x1c, 0x00, 0x03, 0x02, 0x00, 0x00 } },
	{  2350, { 0xf1, 0x1d, 0x00, 0x00, 0x1c, 0x00, 0x03, 0x02, 0x00, 0x00 } },
	{  2500, { 0xf1, 0x1d, 0x00, 0x00, 0x1c, 0x00, 0x03, 0x02, 0x00, 0x00 } },
	{  2600, { 0xf1, 0x1d, 0x00, 0x00, 0x1c, 0x00, 0x03, 0x02, 0x00, 0x00 } },
	{  2800, { 0xf1, 0x1d, 0x00, 0x00, 0x1c, 0x00, 0x03, 0x02, 0x00, 0x00 } },
	{  3000, { 0xf1, 0x1d, 0x00, 0x00, 0x1c, 0x00, 0x03, 0x02, 0x00, 0x00 } },
	{  3300, { 0xf1, 0x1d, 0x00, 0x00, 0x0f, 0x00, 0x03, 0x02, 0x00, 0x00 } },
	{  3500, { 0xf1, 0x1d, 0x00, 0x00, 0x0f, 0x00, 0x03, 0x02, 0x00, 0x00 } },
	{  4000, { 0xf1, 0x1d, 0x01, 0x00, 0x0c, 0x00, 0x75, 0x02, 0x00, 0x00 } },
	{  4500, { 0xf1, 0x1d, 0x01, 0x00, 0x09, 0x00, 0x75, 0x02, 0x00, 0x00 } },
	{  5000, { 0xf1, 0x3d, 0x01, 0x00, 0x08, 0x00, 0x75, 0x02, 0x00, 0x00 } },
	{  5500, { 0xf1, 0x3d, 0x01, 0x00, 0x08, 0x00, 0x75, 0x02, 0x00, 0x00 } },
	{  6000, { 0xf1, 0x1b, 0x02, 0x00, 0x07, 0x00, 0x77, 0x02, 0x00, 0x00 } },
};

static const struct mipi_csi2phy_cphy_regs cphy_regs_kaanapali = {
	.init_seq = lane_regs_kaanapali_3ph,
	.init_seq_size = ARRAY_SIZE(lane_regs_kaanapali_3ph),
	.trio_base = 0x0200,
	.trio_stride = 0x0400,
	.settle_lo = 0x000c,
	.settle_hi = 0x0008,
	.rate_offs = cphy_rate_offs_kaanapali,
	.rates = cphy_rates_kaanapali,
	.num_rates = ARRAY_SIZE(cphy_rates_kaanapali),
	.ctrl0 = 0x0e,
	.ctrl7 = 0x7a,
	.ctrl35 = 0x0e,
	/* RX path runs at half the 16/7 word rate on v2.4.0 */
	.rx_clk_div = 14,
	/* Matches the settle count of 14 @ 400 MHz used by the vendor stack */
	.default_settle_ps = 60000,
};

static inline const struct mipi_csi2phy_device_regs *
csi2phy_dev_to_regs(struct mipi_csi2phy_device *csi2phy)
{
	return &csi2phy->soc_cfg->reg_info;
}

static void phy_qcom_mipi_csi2_hw_version_read(struct mipi_csi2phy_device *csi2phy)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);
	u32 tmp;

	writel(CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_SHOW_REV_ID, csi2phy->base +
	       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->common_regs_offset, 6));

	tmp = readl_relaxed(csi2phy->base +
			    CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(regs->common_regs_offset,
							      regs->common_status_offset, 12));
	csi2phy->hw_version = tmp;

	tmp = readl_relaxed(csi2phy->base +
			    CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(regs->common_regs_offset,
							      regs->common_status_offset, 13));
	csi2phy->hw_version |= (tmp << 8) & 0xFF00;

	tmp = readl_relaxed(csi2phy->base +
			    CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(regs->common_regs_offset,
							      regs->common_status_offset, 14));
	csi2phy->hw_version |= (tmp << 16) & 0xFF0000;

	tmp = readl_relaxed(csi2phy->base +
			    CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(regs->common_regs_offset,
							      regs->common_status_offset, 15));
	csi2phy->hw_version |= (tmp << 24) & 0xFF000000;

	dev_dbg_once(csi2phy->dev, "CSIPHY 3PH HW Version = 0x%08x\n", csi2phy->hw_version);
}

/*
 * phy_qcom_mipi_csi2_reset - Perform software reset on CSIPHY module
 * @phy_qcom_mipi_csi2: CSIPHY device
 */
static void phy_qcom_mipi_csi2_reset(struct mipi_csi2phy_device *csi2phy)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);

	writel(CSIPHY_3PH_CMN_CSI_COMMON_CTRL0_PHY_SW_RESET,
	       csi2phy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->common_regs_offset, 0));
	usleep_range(5000, 8000);
	writel(0x0, csi2phy->base +
	       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->common_regs_offset, 0));
}

/*
 * phy_qcom_mipi_csi2_settle_cnt_calc - Calculate settle count value
 *
 * Helper function to calculate settle count value. This is
 * based on the CSI2 T_hs_settle parameter which in turn
 * is calculated based on the CSI2 transmitter link frequency.
 *
 * Return settle count.
 */
static u8 phy_qcom_mipi_csi2_settle_cnt_calc(s64 link_freq, u32 timer_clk_rate)
{
	u32 t_hs_prepare_max_ps;
	u32 timer_period_ps;
	u32 t_hs_settle_ps;
	u8 settle_cnt;
	u32 ui_ps;

	ui_ps = div64_u64(PSEC_PER_SEC, link_freq);
	ui_ps /= 2;
	t_hs_prepare_max_ps = 85000 + 6 * ui_ps;
	t_hs_settle_ps = t_hs_prepare_max_ps;

	timer_period_ps = div_u64(PSEC_PER_SEC, timer_clk_rate);

	if ((t_hs_settle_ps / timer_period_ps) < 6)
		return 0;

	settle_cnt = t_hs_settle_ps / timer_period_ps - 6;

	return settle_cnt;
}

static void
phy_qcom_mipi_csi2_gen2_config_lanes(struct mipi_csi2phy_device *csi2phy,
				     u8 settle_cnt)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);
	const struct mipi_csi2phy_lane_regs *r = regs->init_seq;
	int i, array_size = regs->lane_array_size;
	u32 val;

	for (i = 0; i < array_size; i++, r++) {
		switch (r->param_type) {
		case CSIPHY_SETTLE_CNT_LOWER_BYTE:
			val = settle_cnt & 0xff;
			break;
		case CSIPHY_SKEW_CAL:
			/* TODO: support application of skew from dt flag */
			continue;
		default:
			val = r->reg_data;
			break;
		}
		writel(val, csi2phy->base + r->reg_addr);
		if (r->delay_us)
			udelay(r->delay_us);
	}
}

static u16 phy_qcom_mipi_csi2_cphy_settle_cnt_calc(struct mipi_csi2phy_device *csi2phy,
						   struct mipi_csi2phy_stream_cfg *cfg)
{
	const struct mipi_csi2phy_cphy_regs *cphy = csi2phy_dev_to_regs(csi2phy)->cphy;
	u32 timer_period_ps, window_ps, cnt;

	if (!csi2phy->timer_clk_rate)
		return 0;

	/* Vendor formula: (T3-PREPARE + T3-PREAMBLE / 3) / T_timer - 10 */
	if (cfg->t3_prepare)
		window_ps = cfg->t3_prepare + cfg->t3_preamble / 3;
	else
		window_ps = cphy->default_settle_ps;

	timer_period_ps = div_u64(PSEC_PER_SEC, csi2phy->timer_clk_rate);
	if (!timer_period_ps)
		return 0;

	cnt = window_ps / timer_period_ps;
	if (cnt <= CSIPHY_CPHY_SETTLE_ADJ)
		return 0;

	return cnt - CSIPHY_CPHY_SETTLE_ADJ;
}

static const struct mipi_csi2phy_cphy_rate_regs *
phy_qcom_mipi_csi2_cphy_get_rate(const struct mipi_csi2phy_cphy_regs *cphy,
				 u64 symbol_rate)
{
	u64 msps = div_u64(symbol_rate, 1000000);
	int i;

	for (i = 0; i < cphy->num_rates; i++)
		if (msps <= cphy->rates[i].max_msps)
			return &cphy->rates[i];

	return &cphy->rates[cphy->num_rates - 1];
}

static int phy_qcom_mipi_csi2_cphy_lanes_enable(struct mipi_csi2phy_device *csi2phy,
						struct mipi_csi2phy_stream_cfg *cfg)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);
	const struct mipi_csi2phy_cphy_regs *cphy = regs->cphy;
	struct mipi_csi2phy_lanes_cfg *lane_cfg = &cfg->lane_cfg;
	const struct mipi_csi2phy_cphy_rate_regs *rate;
	u32 cmn = regs->common_regs_offset;
	u16 settle_cnt;
	u32 val;
	int i, j;

	if (!cphy || !cfg->symbol_rate)
		return -EINVAL;

	settle_cnt = phy_qcom_mipi_csi2_cphy_settle_cnt_calc(csi2phy, cfg);
	if (!settle_cnt)
		return -ENODEV;

	rate = phy_qcom_mipi_csi2_cphy_get_rate(cphy, cfg->symbol_rate);

	dev_dbg(csi2phy->dev, "C-PHY %u trio(s) @ %llu S/s, table <= %u MS/s, settle %u\n",
		cfg->num_data_lanes, cfg->symbol_rate, rate->max_msps, settle_cnt);

	/* Select 3-phase mode, bias and power up the common block */
	writel(0, csi2phy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(cmn, 33));
	writel(cphy->ctrl35, csi2phy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(cmn, 35));
	writel(cphy->ctrl7, csi2phy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(cmn, 7));
	writel(CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_COMMON_PWRDN_B,
	       csi2phy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(cmn, 6));

	/*
	 * CSI_COMMON_CTRL5: C-PHY trio n is physical lane 2n + 1, there is
	 * no clock lane.
	 */
	val = 0;
	for (i = 0; i < cfg->num_data_lanes; i++)
		val |= BIT(lane_cfg->data[i].pos * 2 + 1);
	writel(val, csi2phy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(cmn, 5));

	for (i = 0; i < cfg->num_data_lanes; i++) {
		void __iomem *trio = csi2phy->base + cphy->trio_base +
				     cphy->trio_stride * lane_cfg->data[i].pos;
		const struct mipi_csi2phy_lane_regs *r = cphy->init_seq;

		for (j = 0; j < cphy->init_seq_size; j++, r++) {
			writel(r->reg_data, trio + r->reg_addr);
			if (r->delay_us)
				udelay(r->delay_us);
		}

		for (j = 0; j < CSI2_CPHY_RATE_NREGS; j++)
			writel(rate->val[j], trio + cphy->rate_offs[j]);

		writel(settle_cnt & 0xff, trio + cphy->settle_lo);
		writel(settle_cnt >> 8, trio + cphy->settle_hi);
	}

	/* IRQ_MASK registers - disable all interrupts */
	for (i = CSI_COMMON_STATUS_NUM; i < CSI_CTRL_STATUS_INDEX; i++)
		writel(0, csi2phy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(cmn, i));

	/* Release the PHY from reset in 3-phase mode */
	writel(cphy->ctrl0, csi2phy->base + CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(cmn, 0));

	return 0;
}

static int phy_qcom_mipi_csi2_lanes_enable(struct mipi_csi2phy_device *csi2phy,
					   struct mipi_csi2phy_stream_cfg *cfg)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);
	struct mipi_csi2phy_lanes_cfg *lane_cfg = &cfg->lane_cfg;
	u8 settle_cnt;
	u8 val;
	int i;

	if (cfg->bus_type == MEDIA_BUS_TYPE_CSI2_CPHY)
		return phy_qcom_mipi_csi2_cphy_lanes_enable(csi2phy, cfg);

	if (cfg->link_freq <= 0)
		return -EINVAL;

	settle_cnt = phy_qcom_mipi_csi2_settle_cnt_calc(cfg->link_freq, csi2phy->timer_clk_rate);
	if (!settle_cnt)
		return -ENODEV;

	/*
	 * CSI_COMMON_CTRL5 is a physical lane power-up bitmap:
	 * - Bits [0,2,4,6] → D-PHY data lanes(LN0, LN2, LN4, LN6)
	 * - Bits [1,3,5] → C-PHY trio lanes(LN1, LN3, LN5)
	 * - Bit [7] → D-PHY clock lane(LNCK) dedicated clock enable
	 */
	val = BIT(lane_cfg->clk.pos);
	for (i = 0; i < cfg->num_data_lanes; i++)
		val |= BIT(lane_cfg->data[i].pos * 2);

	writel(val, csi2phy->base +
	       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->common_regs_offset, 5));

	/* Lane configuration for polarity @ CSIPHY-base + CTRL9 */
	for (i = 0; i < cfg->num_data_lanes; i++) {
		if (lane_cfg->data[i].pol) {
			u8 pos = lane_cfg->data[i].pos;

			writel(BIT(2), csi2phy->base + CSIPHY_2PH_LN_CSI_2PHASE_CTRL9n(pos * 2));
		}
	}

	if (lane_cfg->clk.pol)
		writel(BIT(2), csi2phy->base + CSIPHY_2PH_LN_CSI_2PHASE_CTRL9n(lane_cfg->clk.pos));

	val = CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_COMMON_PWRDN_B;
	writel(val, csi2phy->base +
	       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->common_regs_offset, 6));

	val = 0x02;
	writel(val, csi2phy->base +
	       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->common_regs_offset, 7));

	val = 0x00;
	writel(val, csi2phy->base +
	       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->common_regs_offset, 0));

	phy_qcom_mipi_csi2_gen2_config_lanes(csi2phy, settle_cnt);

	/* IRQ_MASK registers - disable all interrupts */
	for (i = CSI_COMMON_STATUS_NUM; i < CSI_CTRL_STATUS_INDEX; i++) {
		writel(0, csi2phy->base +
		       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->common_regs_offset, i));
	}

	return 0;
}

static void
phy_qcom_mipi_csi2_lanes_disable(struct mipi_csi2phy_device *csi2phy,
				 struct mipi_csi2phy_stream_cfg *cfg)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);

	writel(0, csi2phy->base +
	       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->common_regs_offset, 5));

	writel(0, csi2phy->base +
	       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->common_regs_offset, 6));

	if (cfg->bus_type == MEDIA_BUS_TYPE_CSI2_CPHY) {
		writel(0, csi2phy->base +
		       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->common_regs_offset, 35));
		writel(0, csi2phy->base +
		       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->common_regs_offset, 0));
	}
}

static const struct mipi_csi2phy_hw_ops phy_qcom_mipi_csi2_ops_3ph_1_0 = {
	.hw_version_read = phy_qcom_mipi_csi2_hw_version_read,
	.reset = phy_qcom_mipi_csi2_reset,
	.lanes_enable = phy_qcom_mipi_csi2_lanes_enable,
	.lanes_disable = phy_qcom_mipi_csi2_lanes_disable,
};

static const char * const x1e_clks[] = {
	"core",
	"timer",
	"ahb"
};

static const char * const x1e_supplies[] = {
	"vdda-0p9",
	"vdda-1p2"
};

static struct mipi_csi2_genpd x1e_genpds[] = {
	{ .name = "top", .scaled = false },
	{ .name = "mmcx", .scaled = true },
	{ .name = "mx", .scaled = true },
};

const struct mipi_csi2phy_soc_cfg mipi_csi2_dphy_4nm_x1e = {
	.ops = &phy_qcom_mipi_csi2_ops_3ph_1_0,
	.reg_info = {
		.init_seq = lane_regs_x1e80100,
		.lane_array_size = ARRAY_SIZE(lane_regs_x1e80100),
		.common_regs_offset = 0x1000,
		.common_status_offset = 0xb0,
	},
	.supply_names = (const char **)x1e_supplies,
	.num_supplies = ARRAY_SIZE(x1e_supplies),
	.clk_names = (const char **)x1e_clks,
	.num_clk = ARRAY_SIZE(x1e_clks),
	.genpds = x1e_genpds,
	.num_genpds = ARRAY_SIZE(x1e_genpds),
};

const struct mipi_csi2phy_soc_cfg mipi_csi2_dphy_3nm_kaanapali = {
	.ops = &phy_qcom_mipi_csi2_ops_3ph_1_0,
	.reg_info = {
		.init_seq = lane_regs_kaanapali,
		.lane_array_size = ARRAY_SIZE(lane_regs_kaanapali),
		.common_regs_offset = 0x1000,
		.common_status_offset = 0x138,
		.cphy = &cphy_regs_kaanapali,
	},
	.supply_names = (const char **)x1e_supplies,
	.num_supplies = ARRAY_SIZE(x1e_supplies),
	.clk_names = (const char **)x1e_clks,
	.num_clk = ARRAY_SIZE(x1e_clks),
	.genpds = x1e_genpds,
	.num_genpds = ARRAY_SIZE(x1e_genpds),
};
