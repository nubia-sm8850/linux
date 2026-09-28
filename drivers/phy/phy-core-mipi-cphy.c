// SPDX-License-Identifier: GPL-2.0
/*
 * Generic MIPI C-PHY helpers.
 */

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/time64.h>

#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-cphy.h>

/* Up to three trios on a CSI-2 / DSI C-PHY link */
#define PHY_MIPI_CPHY_MAX_LANES		3

/**
 * phy_mipi_cphy_get_default_config() - Helper to get a default C-PHY config
 * @hs_symbol_rate: High-speed symbol rate of each trio, in symbols/s
 * @lanes: Number of trios
 * @cfg: C-PHY configuration to fill
 *
 * Fill @cfg with the link rate and lane count. Optional timings are left
 * unspecified so that the PHY driver can pick its own defaults.
 *
 * Return: 0 on success, a negative error code otherwise.
 */
int phy_mipi_cphy_get_default_config(unsigned long long hs_symbol_rate,
				     unsigned int lanes,
				     struct phy_configure_opts_mipi_cphy *cfg)
{
	if (!cfg || !hs_symbol_rate || !lanes ||
	    lanes > PHY_MIPI_CPHY_MAX_LANES)
		return -EINVAL;

	memset(cfg, 0, sizeof(*cfg));
	cfg->hs_symbol_rate = hs_symbol_rate;
	cfg->lanes = lanes;

	return 0;
}
EXPORT_SYMBOL(phy_mipi_cphy_get_default_config);

/**
 * phy_mipi_cphy_config_validate() - Validate a C-PHY configuration
 * @cfg: C-PHY configuration
 *
 * Check the configuration against the ranges of the MIPI C-PHY
 * specification. Timings set to 0 are treated as unspecified.
 *
 * Return: 0 if the configuration is valid, -EINVAL otherwise.
 */
int phy_mipi_cphy_config_validate(struct phy_configure_opts_mipi_cphy *cfg)
{
	u64 ui;

	if (!cfg || !cfg->hs_symbol_rate)
		return -EINVAL;

	if (!cfg->lanes || cfg->lanes > PHY_MIPI_CPHY_MAX_LANES)
		return -EINVAL;

	ui = div64_u64(PSEC_PER_SEC + cfg->hs_symbol_rate - 1,
		       cfg->hs_symbol_rate);

	if (cfg->t3_prepare &&
	    (cfg->t3_prepare < 38000 || cfg->t3_prepare > 95000))
		return -EINVAL;

	if (cfg->t3_preamble && cfg->t3_preamble < 28 * ui)
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL(phy_mipi_cphy_config_validate);
