/* SPDX-License-Identifier: GPL-2.0 */
/*
 *
 * Qualcomm MIPI CSI2 CPHY/DPHY driver
 *
 * Copyright (C) 2025 Linaro Ltd.
 */
#ifndef __PHY_QCOM_MIPI_CSI2_H__
#define __PHY_QCOM_MIPI_CSI2_H__

#include <linux/phy/phy.h>

#define CSI2_MAX_DATA_LANES	4
#define CSI2_MAX_CPHY_TRIOS	3
#define CSI2_DEFAULT_CLK_LANE	7

struct mipi_csi2phy_lane {
	u8 pos;
	u8 pol;
};

struct mipi_csi2phy_lanes_cfg {
	struct mipi_csi2phy_lane data[CSI2_MAX_DATA_LANES];
	struct mipi_csi2phy_lane clk;
};

/**
 * struct mipi_csi2phy_stream_cfg - active link configuration
 * @bus_type: MEDIA_BUS_TYPE_CSI2_DPHY or MEDIA_BUS_TYPE_CSI2_CPHY, from DT
 * @link_freq: D-PHY link frequency (Hz)
 * @symbol_rate: C-PHY per-trio symbol rate (symbols/s)
 * @t3_prepare: C-PHY T3-PREPARE in ps, 0 if unknown
 * @t3_preamble: C-PHY preamble duration in ps, 0 if unknown
 * @num_data_lanes: number of D-PHY data lanes or C-PHY trios
 * @lane_cfg: lane positions; for C-PHY data[i].pos is the trio index
 */
struct mipi_csi2phy_stream_cfg {
	u32 bus_type;
	s64 link_freq;
	u64 symbol_rate;
	u32 t3_prepare;
	u32 t3_preamble;
	u8 num_data_lanes;
	struct mipi_csi2phy_lanes_cfg lane_cfg;
};

struct mipi_csi2phy_device;

struct mipi_csi2phy_hw_ops {
	void (*hw_version_read)(struct mipi_csi2phy_device *csi2phy_dev);
	void (*reset)(struct mipi_csi2phy_device *csi2phy_dev);
	int (*lanes_enable)(struct mipi_csi2phy_device *csi2phy_dev,
			    struct mipi_csi2phy_stream_cfg *cfg);
	void (*lanes_disable)(struct mipi_csi2phy_device *csi2phy_dev,
			      struct mipi_csi2phy_stream_cfg *cfg);
};

struct mipi_csi2phy_lane_regs {
	const s32 reg_addr;
	const s32 reg_data;
	const u32 delay_us;
	const u32 param_type;
};

#define CSI2_CPHY_RATE_NREGS	10

/**
 * struct mipi_csi2phy_cphy_rate_regs - data-rate dependent trio settings
 * @max_msps: highest symbol rate (MS/s) this entry applies to
 * @val: values for the registers listed in mipi_csi2phy_cphy_regs.rate_offs
 */
struct mipi_csi2phy_cphy_rate_regs {
	u32 max_msps;
	u8 val[CSI2_CPHY_RATE_NREGS];
};

/**
 * struct mipi_csi2phy_cphy_regs - C-PHY (3-phase) programming description
 * @init_seq: per-trio init sequence, offsets relative to the trio base
 * @init_seq_size: number of entries in @init_seq
 * @trio_base: offset of trio 0 (PHY lane 1)
 * @trio_stride: distance between trios (PHY lanes 1, 3, 5)
 * @settle_lo: trio offset of settle count bits [7:0]
 * @settle_hi: trio offset of settle count bits [15:8]
 * @rate_offs: trio offsets of the data-rate dependent registers
 * @rates: data-rate tables, sorted by ascending max_msps
 * @num_rates: number of entries in @rates
 * @ctrl0: COMMON_CTRL0 value releasing the PHY in 3-phase mode
 * @ctrl7: COMMON_CTRL7 value
 * @ctrl35: COMMON_CTRL35 value selecting 3-phase mode
 * @rx_clk_div: symbol rate to core (RX) clock divider
 * @default_settle_ps: settle window used when T3 timings are unknown
 */
struct mipi_csi2phy_cphy_regs {
	const struct mipi_csi2phy_lane_regs *init_seq;
	const int init_seq_size;
	const u32 trio_base;
	const u32 trio_stride;
	const u32 settle_lo;
	const u32 settle_hi;
	const u16 *rate_offs;
	const struct mipi_csi2phy_cphy_rate_regs *rates;
	const int num_rates;
	const u8 ctrl0;
	const u8 ctrl7;
	const u8 ctrl35;
	const u32 rx_clk_div;
	const u32 default_settle_ps;
};

struct mipi_csi2phy_device_regs {
	const struct mipi_csi2phy_lane_regs *init_seq;
	const int lane_array_size;
	const u32 common_regs_offset;
	const u32 common_status_offset;
	const struct mipi_csi2phy_cphy_regs *cphy;
};

struct mipi_csi2_genpd {
	const char *name;
	bool scaled;
};

struct mipi_csi2phy_soc_cfg {
	const struct mipi_csi2phy_hw_ops *ops;
	const struct mipi_csi2phy_device_regs reg_info;

	const char ** const supply_names;
	const unsigned int num_supplies;

	const char ** const clk_names;
	const unsigned int num_clk;

	const struct mipi_csi2_genpd *genpds;
	const unsigned int num_genpds;
};

struct mipi_csi2phy_device {
	struct device *dev;
	u8 phy_mode;

	struct phy *phy;
	void __iomem *base;

	struct clk_bulk_data *clks;
	struct clk *timer_clk;
	u32 timer_clk_rate;

	struct regulator_bulk_data *supplies;
	struct dev_pm_domain_list *pd_list;

	const struct mipi_csi2phy_soc_cfg *soc_cfg;
	struct mipi_csi2phy_stream_cfg stream_cfg;

	u32 hw_version;
};

extern const struct mipi_csi2phy_soc_cfg mipi_csi2_dphy_4nm_x1e;
extern const struct mipi_csi2phy_soc_cfg mipi_csi2_dphy_3nm_kaanapali;

#endif /* __PHY_QCOM_MIPI_CSI2_H__ */
