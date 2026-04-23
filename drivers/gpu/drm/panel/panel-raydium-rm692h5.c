// SPDX-License-Identifier: GPL-2.0-only
/*
 * Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree.
 * Copyright (c) 2026 Luka Panio <lukapanio@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

struct raydium_rm692h5 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct regulator_bulk_data supplies[3];
	struct gpio_desc *reset_gpio;
};

static inline
struct raydium_rm692h5 *to_raydium_rm692h5(struct drm_panel *panel)
{
	return container_of_const(panel, struct raydium_rm692h5, panel);
}

static void raydium_rm692h5_reset(struct raydium_rm692h5 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(40);
}

static int raydium_rm692h5_on(struct raydium_rm692h5 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfe, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc2, 0x08);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);
	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0x0000);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	return dsi_ctx.accum_err;
}

static int raydium_rm692h5_off(struct raydium_rm692h5 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfe, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 16000, 17000);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int raydium_rm692h5_prepare(struct drm_panel *panel)
{
	struct raydium_rm692h5 *ctx = to_raydium_rm692h5(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	struct device *dev = &ctx->dsi->dev;
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	dsi_ctx.accum_err = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	raydium_rm692h5_reset(ctx);

	ret = raydium_rm692h5_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
	if (ret < 0) {
		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_compression_mode(ctx->dsi, true);
	if (ret < 0) {
		dev_err(dev, "failed to enable compression mode: %d\n", ret);
		return ret;
	}

	msleep(28);

	return 0;
}

static int raydium_rm692h5_unprepare(struct drm_panel *panel)
{
	struct raydium_rm692h5 *ctx = to_raydium_rm692h5(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = raydium_rm692h5_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	return 0;
}

static const struct drm_display_mode raydium_rm692h5_mode = {
	.clock = (1216 + 8 + 8 + 12) * (2688 + 56 + 8 + 8) * 144 / 1000,
	.hdisplay = 1216,
	.hsync_start = 1216 + 8,
	.hsync_end = 1216 + 8 + 8,
	.htotal = 1216 + 8 + 8 + 12,
	.vdisplay = 2688,
	.vsync_start = 2688 + 56,
	.vsync_end = 2688 + 56 + 8,
	.vtotal = 2688 + 56 + 8 + 8,
	.width_mm = 72,
	.height_mm = 158,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int raydium_rm692h5_get_modes(struct drm_panel *panel,
						struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &raydium_rm692h5_mode);
}

static const struct drm_panel_funcs raydium_rm692h5_panel_funcs = {
	.prepare = raydium_rm692h5_prepare,
	.unprepare = raydium_rm692h5_unprepare,
	.get_modes = raydium_rm692h5_get_modes,
};

static int raydium_rm692h5_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int raydium_rm692h5_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops raydium_rm692h5_bl_ops = {
	.update_status = raydium_rm692h5_bl_update_status,
	.get_brightness = raydium_rm692h5_bl_get_brightness,
};

static struct backlight_device *
raydium_rm692h5_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 8190,
		.max_brightness = 8190,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &raydium_rm692h5_bl_ops, &props);
}

static int raydium_rm692h5_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct raydium_rm692h5 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct raydium_rm692h5, panel,
				   &raydium_rm692h5_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->supplies[0].supply = "vddio";
	ctx->supplies[1].supply = "vdd";
	ctx->supplies[2].supply = "vci";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = raydium_rm692h5_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;

	ctx->dsc.slice_height = 16;
	ctx->dsc.slice_width = 608;

	ctx->dsc.slice_count = 2;
	ctx->dsc.convert_rgb = true;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = false;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void raydium_rm692h5_remove(struct mipi_dsi_device *dsi)
{
	struct raydium_rm692h5 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id raydium_rm692h5_of_match[] = {
	{ .compatible = "raydium,rm692h5" },
	{ }
};
MODULE_DEVICE_TABLE(of, raydium_rm692h5_of_match);

static struct mipi_dsi_driver raydium_rm692h5_driver = {
	.probe = raydium_rm692h5_probe,
	.remove = raydium_rm692h5_remove,
	.driver = {
		.name = "panel-rm692h5-amoled",
		.of_match_table = raydium_rm692h5_of_match,
	},
};
module_mipi_dsi_driver(raydium_rm692h5_driver);

MODULE_DESCRIPTION("DRM driver for Raydium rm692h5");
MODULE_LICENSE("GPL");
