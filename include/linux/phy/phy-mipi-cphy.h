/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MIPI C-PHY configuration options for the generic PHY framework.
 */

#ifndef __PHY_MIPI_CPHY_H_
#define __PHY_MIPI_CPHY_H_

/**
 * struct phy_configure_opts_mipi_cphy - MIPI C-PHY configuration set
 *
 * This structure is used to represent the configuration state of a
 * MIPI C-PHY phy. A C-PHY link is made of one or more lanes (trios),
 * each carrying 16 bits in 7 symbols; there is no clock lane.
 *
 * Timing members are optional: a value of 0 means "not specified" and
 * lets the PHY driver fall back to a safe default for the given symbol
 * rate.
 */
struct phy_configure_opts_mipi_cphy {
	/**
	 * @hs_symbol_rate:
	 *
	 * High-speed symbol rate of each trio, in symbols per second.
	 *
	 * For CSI-2 this is twice the V4L2_CID_LINK_FREQ value, see
	 * Documentation/driver-api/media/tx-rx.rst.
	 */
	unsigned long long	hs_symbol_rate;

	/**
	 * @t3_prepare:
	 *
	 * Time, in picoseconds, the transmitter drives the LP-000 state
	 * before starting the HS preamble (T3-PREPARE).
	 *
	 * Minimum value: 38000 ps
	 * Maximum value: 95000 ps
	 * 0: not specified
	 */
	unsigned int		t3_prepare;

	/**
	 * @t3_preamble:
	 *
	 * Duration, in picoseconds, of the HS preamble sent before the
	 * sync word (T3-PREBEGIN + T3-PROGSEQ + T3-PREEND).
	 *
	 * Minimum value: 28 * UI
	 * 0: not specified
	 */
	unsigned int		t3_preamble;

	/**
	 * @lanes:
	 *
	 * Number of active, consecutive, trios used for the transmission.
	 */
	unsigned char		lanes;
};

int phy_mipi_cphy_get_default_config(unsigned long long hs_symbol_rate,
				     unsigned int lanes,
				     struct phy_configure_opts_mipi_cphy *cfg);
int phy_mipi_cphy_config_validate(struct phy_configure_opts_mipi_cphy *cfg);

#endif /* __PHY_MIPI_CPHY_H_ */
