/*
 * Copyright (C) 2013, NVIDIA Corporation.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <video/display_timing.h>
#include <video/videomode.h>

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct display_timing *timings;
	unsigned int num_timings;

	unsigned int bpc;

	/**
	 * @width: width (in millimeters) of the panel's active display area
	 * @height: height (in millimeters) of the panel's active display area
	 */
	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *           become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *          display the first valid frame after starting to receive
	 *          video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *           turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *             to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;

	u32 bus_format;
	u32 bus_flags;
};

struct panel_simple {
	struct drm_panel base;
	bool prepared;
	bool enabled;

	const struct panel_desc *desc;

	struct backlight_device *backlight;
	struct regulator *supply;
	struct i2c_adapter *ddc;

	struct gpio_desc *enable_gpio;

	u32 bus_fmt_override;
	u32 quirks;
};

enum {
	PANEL_QUIRK_PIXDATA_NEGEDGE = BIT(0),
	PANEL_QUIRK_PIXDATA_POSEDGE = BIT(1),
};

#define SP_DISPLAY_MODE(freq, ha, hfp, hs, hbp, va, vfp, vs, vbp, vr, flgs) { \
	.clock = freq,							\
	.hdisplay = ha,							\
	.hsync_start = (ha) + (hfp),					\
	.hsync_end = (ha) + (hfp) + (hs),				\
	.htotal = (ha) + (hfp) + (hs) + (hbp),				\
	.vdisplay = (va),						\
	.vsync_start = (va) + (vfp),					\
	.vsync_end = (va) + (vfp) + (vs),				\
	.vtotal = (va) + (vfp) + (vs) + (vbp),				\
	.vrefresh = vr,							\
	.flags = flgs,							\
}

static inline struct panel_simple *to_panel_simple(struct drm_panel *panel)
{
	return container_of(panel, struct panel_simple, base);
}

static inline void panel_simple_apply_quirks(struct panel_simple *panel,
					     struct drm_display_info *info)
{
	if (panel->quirks & PANEL_QUIRK_PIXDATA_NEGEDGE)
		info->bus_flags |= DRM_BUS_FLAG_PIXDATA_NEGEDGE;
	if (panel->quirks & PANEL_QUIRK_PIXDATA_POSEDGE)
		info->bus_flags |= DRM_BUS_FLAG_PIXDATA_POSEDGE;
}

static int panel_simple_get_fixed_modes(struct panel_simple *panel)
{
	struct drm_connector *connector = panel->base.connector;
	struct drm_device *drm = panel->base.drm;
	struct drm_display_mode *mode;
	unsigned int i, num = 0;

	if (!panel->desc)
		return 0;

	for (i = 0; i < panel->desc->num_timings; i++) {
		const struct display_timing *dt = &panel->desc->timings[i];
		struct videomode vm;

		videomode_from_timing(dt, &vm);
		mode = drm_mode_create(drm);
		if (!mode) {
			dev_err(drm->dev, "failed to add mode %ux%u\n",
				dt->hactive.typ, dt->vactive.typ);
			continue;
		}

		drm_display_mode_from_videomode(&vm, mode);

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (panel->desc->num_timings == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
		num++;
	}

	for (i = 0; i < panel->desc->num_modes; i++) {
		const struct drm_display_mode *m = &panel->desc->modes[i];

		mode = drm_mode_duplicate(drm, m);
		if (!mode) {
			dev_err(drm->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, m->vrefresh);
			continue;
		}

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (panel->desc->num_modes == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);

		drm_mode_probed_add(connector, mode);
		num++;
	}

	connector->display_info.bpc = panel->desc->bpc;
	connector->display_info.width_mm = panel->desc->size.width;
	connector->display_info.height_mm = panel->desc->size.height;

	if (panel->bus_fmt_override)
		drm_display_info_set_bus_formats(&connector->display_info,
						 &panel->bus_fmt_override, 1);
	else if (panel->desc->bus_format)
		drm_display_info_set_bus_formats(&connector->display_info,
						 &panel->desc->bus_format, 1);
	connector->display_info.bus_flags = panel->desc->bus_flags;
	if (panel->quirks)
		panel_simple_apply_quirks(panel, &connector->display_info);

	return num;
}

static int panel_simple_disable(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);

	if (!p->enabled)
		return 0;

	if (p->backlight) {
		p->backlight->props.power = FB_BLANK_POWERDOWN;
		p->backlight->props.state |= BL_CORE_FBBLANK;
		backlight_update_status(p->backlight);
	}

	if (p->desc->delay.disable)
		msleep(p->desc->delay.disable);

	p->enabled = false;

	return 0;
}

static int panel_simple_unprepare(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);

	if (!p->prepared)
		return 0;

	if (p->enable_gpio)
		gpiod_set_value_cansleep(p->enable_gpio, 0);

	regulator_disable(p->supply);

	if (p->desc->delay.unprepare)
		msleep(p->desc->delay.unprepare);

	p->prepared = false;

	return 0;
}

static int panel_simple_prepare(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);
	int err;

	if (p->prepared)
		return 0;

	err = regulator_enable(p->supply);
	if (err < 0) {
		dev_err(panel->dev, "failed to enable supply: %d\n", err);
		return err;
	}

	if (p->enable_gpio)
		gpiod_set_value_cansleep(p->enable_gpio, 1);

	if (p->desc->delay.prepare)
		msleep(p->desc->delay.prepare);

	p->prepared = true;

	return 0;
}

static int panel_simple_enable(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);

	if (p->enabled)
		return 0;

	if (p->desc->delay.enable)
		msleep(p->desc->delay.enable);

	if (p->backlight) {
		p->backlight->props.state &= ~BL_CORE_FBBLANK;
		p->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(p->backlight);
	}

	p->enabled = true;

	return 0;
}

static int panel_simple_get_modes(struct drm_panel *panel)
{
	struct panel_simple *p = to_panel_simple(panel);
	int num = 0;

	/* probe EDID if a DDC bus is available */
	if (p->ddc) {
		struct edid *edid = drm_get_edid(panel->connector, p->ddc);
		drm_mode_connector_update_edid_property(panel->connector, edid);
		if (edid) {
			num += drm_add_edid_modes(panel->connector, edid);
			kfree(edid);
		}
	}

	/* add hard-coded panel modes */
	num += panel_simple_get_fixed_modes(p);

	return num;
}

static int panel_simple_get_timings(struct drm_panel *panel,
				    unsigned int num_timings,
				    struct display_timing *timings)
{
	struct panel_simple *p = to_panel_simple(panel);
	unsigned int i;

	if (p->desc->num_timings < num_timings)
		num_timings = p->desc->num_timings;

	if (timings)
		for (i = 0; i < num_timings; i++)
			timings[i] = p->desc->timings[i];

	return p->desc->num_timings;
}

static inline int panel_simple_check_quirks(struct device *dev,
					    struct panel_simple *p)
{
	const char *bus_fmt;
	u32 clkpol;

	if (of_property_read_string(dev->of_node, "bus-format-override",
				    &bus_fmt) == 0) {
		if (strcmp(bus_fmt, "rgb24") == 0)
			p->bus_fmt_override = MEDIA_BUS_FMT_RGB888_1X24;
		else if (strcmp(bus_fmt, "rgb666") == 0)
			p->bus_fmt_override = MEDIA_BUS_FMT_RGB666_1X18;
		else if (strcmp(bus_fmt, "rgb565") == 0)
			p->bus_fmt_override = MEDIA_BUS_FMT_RGB565_1X16;
		else if (strcmp(bus_fmt, "spwg-18") == 0)
			p->bus_fmt_override = MEDIA_BUS_FMT_RGB666_1X7X3_SPWG;
		else if (strcmp(bus_fmt, "spwg-24") == 0)
			p->bus_fmt_override = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG;
		else if (strcmp(bus_fmt, "jeida-24") == 0)
			p->bus_fmt_override = MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA;
		else
			dev_err(dev,
				"Unsupported bus-format-override value: '%s'\n",
				bus_fmt);
		return p->bus_fmt_override ? 0 : -EINVAL;
	}

	if (of_property_read_u32(dev->of_node, "pixelclk-active",
				 &clkpol) == 0) {
		if (clkpol & ~1) {
			dev_err(dev,
				"Invalid value for pixelclk-active: '%u' (should be <0> or <1>)\n",
				clkpol);
			return -EINVAL;
		}
		p->quirks |= clkpol ? PANEL_QUIRK_PIXDATA_POSEDGE :
			PANEL_QUIRK_PIXDATA_NEGEDGE;
	}
	return 0;
}

static const struct drm_panel_funcs panel_simple_funcs = {
	.disable = panel_simple_disable,
	.unprepare = panel_simple_unprepare,
	.prepare = panel_simple_prepare,
	.enable = panel_simple_enable,
	.get_modes = panel_simple_get_modes,
	.get_timings = panel_simple_get_timings,
};

static int panel_simple_probe(struct device *dev, const struct panel_desc *desc)
{
	struct device_node *backlight, *ddc;
	struct panel_simple *panel;
	int err;

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	panel->enabled = false;
	panel->prepared = false;
	panel->desc = desc;

	panel->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(panel->supply))
		return PTR_ERR(panel->supply);

	panel->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						     GPIOD_OUT_LOW);
	if (IS_ERR(panel->enable_gpio)) {
		err = PTR_ERR(panel->enable_gpio);
		dev_err(dev, "failed to request GPIO: %d\n", err);
		return err;
	}

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		panel->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!panel->backlight)
			return -EPROBE_DEFER;
	}

	ddc = of_parse_phandle(dev->of_node, "ddc-i2c-bus", 0);
	if (ddc) {
		panel->ddc = of_find_i2c_adapter_by_node(ddc);
		of_node_put(ddc);

		if (!panel->ddc) {
			err = -EPROBE_DEFER;
			goto free_backlight;
		}
	}

	err = panel_simple_check_quirks(dev, panel);
	if (err)
		goto free_ddc;

	drm_panel_init(&panel->base);
	panel->base.dev = dev;
	panel->base.funcs = &panel_simple_funcs;

	err = drm_panel_add(&panel->base);
	if (err < 0)
		goto free_ddc;

	dev_set_drvdata(dev, panel);

	return 0;

free_ddc:
	if (panel->ddc)
		put_device(&panel->ddc->dev);
free_backlight:
	if (panel->backlight)
		put_device(&panel->backlight->dev);

	return err;
}

static int panel_simple_remove(struct device *dev)
{
	struct panel_simple *panel = dev_get_drvdata(dev);

	drm_panel_detach(&panel->base);
	drm_panel_remove(&panel->base);

	panel_simple_disable(&panel->base);

	if (panel->ddc)
		put_device(&panel->ddc->dev);

	if (panel->backlight)
		put_device(&panel->backlight->dev);

	return 0;
}

static void panel_simple_shutdown(struct device *dev)
{
	struct panel_simple *panel = dev_get_drvdata(dev);

	panel_simple_disable(&panel->base);
}

static const struct drm_display_mode ampire_am_480272h3tmqw_t01h_mode =
	SP_DISPLAY_MODE(9000, 480, 2, 41, 2, 272, 2, 10, 2, 60,
			DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);

static const struct panel_desc ampire_am_480272h3tmqw_t01h = {
	.modes = &ampire_am_480272h3tmqw_t01h_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 105,
		.height = 67,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct drm_display_mode ampire_am800480r3tmqwa1h_mode =
	SP_DISPLAY_MODE(33333, 800, 0, 255, 0, 480, 2, 45, 0, 60,
			DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);

static const struct panel_desc ampire_am800480r3tmqwa1h = {
	.modes = &ampire_am800480r3tmqwa1h_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct drm_display_mode auo_b101aw03_mode =
	SP_DISPLAY_MODE(51450, 1024, 156, 8, 156, 600, 16, 6, 16, 60, 0);

static const struct panel_desc auo_b101aw03 = {
	.modes = &auo_b101aw03_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 223,
		.height = 125,
	},
};

static const struct drm_display_mode auo_b101ean01_mode =
	SP_DISPLAY_MODE(72500, 1280, 119, 32, 21, 800, 4, 20, 8, 60, 0);

static const struct panel_desc auo_b101ean01 = {
	.modes = &auo_b101ean01_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 217,
		.height = 136,
	},
};

static const struct drm_display_mode auo_b101xtn01_mode =
	SP_DISPLAY_MODE(72000, 1366, 20, 70, 0, 768, 14, 42, 0, 60,
			DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC);

static const struct panel_desc auo_b101xtn01 = {
	.modes = &auo_b101xtn01_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 223,
		.height = 125,
	},
};

static const struct drm_display_mode auo_b116xw03_mode =
	SP_DISPLAY_MODE(70589, 1366, 40, 40, 32, 768, 10, 12, 6, 60, 0);

static const struct panel_desc auo_b116xw03 = {
	.modes = &auo_b116xw03_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 256,
		.height = 144,
	},
};

static const struct drm_display_mode auo_b133xtn01_mode =
	SP_DISPLAY_MODE(69500, 1366, 48, 32, 20, 768, 3, 6, 13, 60, 0);

static const struct panel_desc auo_b133xtn01 = {
	.modes = &auo_b133xtn01_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 293,
		.height = 165,
	},
};

static const struct drm_display_mode auo_b133htn01_mode =
	SP_DISPLAY_MODE(150660, 1920, 172, 80, 60, 1080, 25, 10, 10, 60, 0);

static const struct panel_desc auo_b133htn01 = {
	.modes = &auo_b133htn01_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 293,
		.height = 165,
	},
	.delay = {
		.prepare = 105,
		.enable = 20,
		.unprepare = 50,
	},
};

static const struct display_timing auo_g133han01_timings = {
	.pixelclock = { 134000000, 141200000, 149000000 },
	.hactive = { 1920, 1920, 1920 },
	.hfront_porch = { 39, 58, 77 },
	.hback_porch = { 59, 88, 117 },
	.hsync_len = { 28, 42, 56 },
	.vactive = { 1080, 1080, 1080 },
	.vfront_porch = { 3, 8, 11 },
	.vback_porch = { 5, 14, 19 },
	.vsync_len = { 4, 14, 19 },
};

static const struct panel_desc auo_g133han01 = {
	.timings = &auo_g133han01_timings,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 293,
		.height = 165,
	},
	.delay = {
		.prepare = 200,
		.enable = 50,
		.disable = 50,
		.unprepare = 1000,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA,
};

static const struct display_timing auo_g185han01_timings = {
	.pixelclock = { 120000000, 144000000, 175000000 },
	.hactive = { 1920, 1920, 1920 },
	.hfront_porch = { 18, 60, 74 },
	.hback_porch = { 12, 44, 54 },
	.hsync_len = { 10, 24, 32 },
	.vactive = { 1080, 1080, 1080 },
	.vfront_porch = { 6, 10, 40 },
	.vback_porch = { 2, 5, 20 },
	.vsync_len = { 2, 5, 20 },
};

static const struct panel_desc auo_g185han01 = {
	.timings = &auo_g185han01_timings,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 409,
		.height = 230,
	},
	.delay = {
		.prepare = 50,
		.enable = 200,
		.disable = 110,
		.unprepare = 1000,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct display_timing auo_p320hvn03_timings = {
	.pixelclock = { 106000000, 148500000, 164000000 },
	.hactive = { 1920, 1920, 1920 },
	.hfront_porch = { 25, 50, 130 },
	.hback_porch = { 25, 50, 130 },
	.hsync_len = { 20, 40, 105 },
	.vactive = { 1080, 1080, 1080 },
	.vfront_porch = { 8, 17, 150 },
	.vback_porch = { 8, 17, 150 },
	.vsync_len = { 4, 11, 100 },
};

static const struct panel_desc auo_p320hvn03 = {
	.timings = &auo_p320hvn03_timings,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 698,
		.height = 393,
	},
	.delay = {
		.prepare = 1,
		.enable = 450,
		.unprepare = 500,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA,
};

static const struct drm_display_mode auo_t215hvn01_mode =
	SP_DISPLAY_MODE(148800, 1920, 88, 44, 148, 1080, 4, 5, 36, 60, 0);

static const struct panel_desc auo_t215hvn01 = {
	.modes = &auo_t215hvn01_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 430,
		.height = 270,
	},
	.delay = {
		.disable = 5,
		.unprepare = 1000,
	}
};

static const struct drm_display_mode avic_tm070ddh03_mode =
	SP_DISPLAY_MODE(51200, 1024, 160, 4, 156, 600, 17, 1, 17, 60, 0);

static const struct panel_desc avic_tm070ddh03 = {
	.modes = &avic_tm070ddh03_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 154,
		.height = 90,
	},
	.delay = {
		.prepare = 20,
		.enable = 200,
		.disable = 200,
	},
};

static const struct drm_display_mode boe_nv101wxmn51_modes[] = {
	SP_DISPLAY_MODE(71900, 1280, 48, 32, 80, 800, 3, 5, 24, 60, 0),
	SP_DISPLAY_MODE(57500, 1280, 48, 32, 80, 800, 3, 5, 24, 48, 0),
};

static const struct panel_desc boe_nv101wxmn51 = {
	.modes = boe_nv101wxmn51_modes,
	.num_modes = ARRAY_SIZE(boe_nv101wxmn51_modes),
	.bpc = 8,
	.size = {
		.width = 217,
		.height = 136,
	},
	.delay = {
		.prepare = 210,
		.enable = 50,
		.unprepare = 160,
	},
};

static const struct drm_display_mode chunghwa_claa070wp03xg_mode =
	SP_DISPLAY_MODE(66770, 800, 49, 33, 17, 1280, 1, 7, 15, 60,
			DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC);

static const struct panel_desc chunghwa_claa070wp03xg = {
	.modes = &chunghwa_claa070wp03xg_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 94,
		.height = 150,
	},
};

static const struct drm_display_mode chunghwa_claa101wa01a_mode =
	SP_DISPLAY_MODE(72070, 1366, 58, 58, 58, 768, 4, 4, 4, 60, 0);

static const struct panel_desc chunghwa_claa101wa01a = {
	.modes = &chunghwa_claa101wa01a_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 220,
		.height = 120,
	},
};

static const struct drm_display_mode chunghwa_claa101wb01_mode =
	SP_DISPLAY_MODE(69300, 1366, 48, 32, 20, 768, 16, 8, 16, 60, 0);

static const struct panel_desc chunghwa_claa101wb01 = {
	.modes = &chunghwa_claa101wb01_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 223,
		.height = 125,
	},
};

static const struct drm_display_mode edt_et0350g0dh6_mode =
	SP_DISPLAY_MODE(6500, 320, 20, 0, 68, 240, 4, 0, 18, 60,
			DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC);

static const struct panel_desc edt_et0350g0dh6 = {
	.modes = &edt_et0350g0dh6_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 70,
		.height = 53,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_NEGEDGE,
};

static const struct drm_display_mode edt_et0430g0dh6_mode =
	SP_DISPLAY_MODE(9000, 480, 2, 41, 2, 272, 2, 10, 2, 60,
			DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC);

static const struct panel_desc edt_et0430g0dh6 = {
	.modes = &edt_et0430g0dh6_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 95,
		.height = 54,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_POSEDGE,
};

static const struct drm_display_mode edt_et057090dhu_mode =
	SP_DISPLAY_MODE(25175, 640, 16, 30, 114, 480, 10, 3, 32, 60,
			DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC);

static const struct panel_desc edt_et057090dhu = {
	.modes = &edt_et057090dhu_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 115,
		.height = 86,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_NEGEDGE,
};

static const struct drm_display_mode edt_etm0700g0dh6_mode =
	SP_DISPLAY_MODE(33260, 800, 40, 128, 88, 480, 10, 2, 33, 60,
			DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);

static const struct panel_desc edt_etm0700g0dh6 = {
	.modes = &edt_etm0700g0dh6_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_NEGEDGE,
};

static const struct drm_display_mode foxlink_fl500wvr00_a0t_mode =
	SP_DISPLAY_MODE(32260, 800, 168, 64, 88, 480, 37, 2, 8, 60, 0);

static const struct panel_desc foxlink_fl500wvr00_a0t = {
	.modes = &foxlink_fl500wvr00_a0t_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 108,
		.height = 65,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct drm_display_mode giantplus_gpg482739qs5_mode =
	SP_DISPLAY_MODE(9000, 480, 5, 1, 40, 272, 8, 1, 8, 60, 0);

static const struct panel_desc giantplus_gpg482739qs5 = {
	.modes = &giantplus_gpg482739qs5_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 95,
		.height = 54,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct display_timing hannstar_hsd070pww1_timing = {
	.pixelclock = { 64300000, 71100000, 82000000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 1, 1, 10 },
	.hback_porch = { 1, 1, 10 },
	/*
	 * According to the data sheet, the minimum horizontal blanking interval
	 * is 54 clocks (1 + 52 + 1), but tests with a Nitrogen6X have shown the
	 * minimum working horizontal blanking interval to be 60 clocks.
	 */
	.hsync_len = { 58, 158, 661 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 1, 1, 10 },
	.vback_porch = { 1, 1, 10 },
	.vsync_len = { 1, 21, 203 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc hannstar_hsd070pww1 = {
	.timings = &hannstar_hsd070pww1_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 151,
		.height = 94,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,
};

static const struct display_timing hannstar_hsd100pxn1_timing = {
	.pixelclock = { 55000000, 65000000, 75000000 },
	.hactive = { 1024, 1024, 1024 },
	.hfront_porch = { 40, 40, 40 },
	.hback_porch = { 220, 220, 220 },
	.hsync_len = { 20, 60, 100 },
	.vactive = { 768, 768, 768 },
	.vfront_porch = { 7, 7, 7 },
	.vback_porch = { 21, 21, 21 },
	.vsync_len = { 10, 10, 10 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc hannstar_hsd100pxn1 = {
	.timings = &hannstar_hsd100pxn1_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 203,
		.height = 152,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,
};

static const struct drm_display_mode hitachi_tx23d38vm0caa_mode =
	SP_DISPLAY_MODE(33333, 800, 85, 86, 85, 480, 16, 13, 16, 60, 0);

static const struct panel_desc hitachi_tx23d38vm0caa = {
	.modes = &hitachi_tx23d38vm0caa_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 195,
		.height = 117,
	},
};

static const struct drm_display_mode innolux_at043tn24_mode =
	SP_DISPLAY_MODE(9000, 480, 2, 41, 2, 272, 2, 11, 2, 60,
			DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);

static const struct panel_desc innolux_at043tn24 = {
	.modes = &innolux_at043tn24_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 95,
		.height = 54,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct drm_display_mode innolux_at070tn92_mode =
	SP_DISPLAY_MODE(33333, 800, 210, 20, 46, 480, 22, 10, 23, 60, 0);

static const struct panel_desc innolux_at070tn92 = {
	.modes = &innolux_at070tn92_mode,
	.num_modes = 1,
	.size = {
		.width = 154,
		.height = 86,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct display_timing innolux_g101ice_l01_timing = {
	.pixelclock = { 60400000, 71100000, 74700000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 41, 80, 100 },
	.hback_porch = { 40, 79, 99 },
	.hsync_len = { 1, 1, 1 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 5, 11, 14 },
	.vback_porch = { 4, 11, 14 },
	.vsync_len = { 1, 1, 1 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc innolux_g101ice_l01 = {
	.timings = &innolux_g101ice_l01_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 217,
		.height = 135,
	},
	.delay = {
		.enable = 200,
		.disable = 200,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct display_timing innolux_g121i1_l01_timing = {
	.pixelclock = { 67450000, 71000000, 74550000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 40, 80, 160 },
	.hback_porch = { 39, 79, 159 },
	.hsync_len = { 1, 1, 1 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 5, 11, 100 },
	.vback_porch = { 4, 11, 99 },
	.vsync_len = { 1, 1, 1 },
};

static const struct panel_desc innolux_g121i1_l01 = {
	.timings = &innolux_g121i1_l01_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 261,
		.height = 163,
	},
	.delay = {
		.enable = 200,
		.disable = 20,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode innolux_g121x1_l03_mode =
	SP_DISPLAY_MODE(65000, 1024, 0, 1, 320, 768, 38, 1, 0, 60,
			DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);

static const struct panel_desc innolux_g121x1_l03 = {
	.modes = &innolux_g121x1_l03_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 246,
		.height = 185,
	},
	.delay = {
		.enable = 200,
		.unprepare = 200,
		.disable = 400,
	},
};

static const struct drm_display_mode innolux_n116bge_mode =
	SP_DISPLAY_MODE(76420, 1366, 136, 30, 60, 768, 8, 12, 12, 60,
			DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);

static const struct panel_desc innolux_n116bge = {
	.modes = &innolux_n116bge_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 256,
		.height = 144,
	},
};

static const struct drm_display_mode innolux_n156bge_l21_mode =
	SP_DISPLAY_MODE(69300, 1366, 16, 34, 50, 768, 2, 6, 12, 60, 0);

static const struct panel_desc innolux_n156bge_l21 = {
	.modes = &innolux_n156bge_l21_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 344,
		.height = 193,
	},
};

static const struct drm_display_mode innolux_zj070na_01p_mode =
	SP_DISPLAY_MODE(51501, 1024, 128, 64, 128, 600, 16, 4, 16, 60, 0);

static const struct panel_desc innolux_zj070na_01p = {
	.modes = &innolux_zj070na_01p_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 154,
		.height = 90,
	},
};

static const struct display_timing kyo_tcg121xglp_timing = {
	.pixelclock = { 52000000, 65000000, 71000000 },
	.hactive = { 1024, 1024, 1024 },
	.hfront_porch = { 2, 2, 2 },
	.hback_porch = { 2, 2, 2 },
	.hsync_len = { 86, 124, 244 },
	.vactive = { 768, 768, 768 },
	.vfront_porch = { 2, 2, 2 },
	.vback_porch = { 2, 2, 2 },
	.vsync_len = { 6, 34, 73 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc kyo_tcg121xglp = {
	.timings = &kyo_tcg121xglp_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 246,
		.height = 184,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode lg_lb070wv8_mode =
	SP_DISPLAY_MODE(33246, 800, 88, 80, 88, 480, 10, 25, 10, 60, 0);

static const struct panel_desc lg_lb070wv8 = {
	.modes = &lg_lb070wv8_mode,
	.num_modes = 1,
	.bpc = 16,
	.size = {
		.width = 151,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode lg_lp079qx1_sp0v_mode =
	SP_DISPLAY_MODE(200000, 1536, 12, 16, 48, 2048, 8, 4, 8, 60,
			 DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC);

static const struct panel_desc lg_lp079qx1_sp0v = {
	.modes = &lg_lp079qx1_sp0v_mode,
	.num_modes = 1,
	.size = {
		.width = 129,
		.height = 171,
	},
};

static const struct drm_display_mode lg_lp097qx1_spa1_mode =
	SP_DISPLAY_MODE(205210, 2048, 150, 5, 5, 1536, 3, 1, 9, 60, 0);

static const struct panel_desc lg_lp097qx1_spa1 = {
	.modes = &lg_lp097qx1_spa1_mode,
	.num_modes = 1,
	.size = {
		.width = 208,
		.height = 147,
	},
};

static const struct drm_display_mode lg_lp120up1_mode =
	SP_DISPLAY_MODE(162300, 1920, 40, 40, 80, 1280, 4, 4, 12, 60, 0);

static const struct panel_desc lg_lp120up1 = {
	.modes = &lg_lp120up1_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 267,
		.height = 183,
	},
};

static const struct drm_display_mode lg_lp129qe_mode =
	SP_DISPLAY_MODE(285250, 2560, 48, 32, 80, 1700, 3, 10, 36, 60, 0);

static const struct panel_desc lg_lp129qe = {
	.modes = &lg_lp129qe_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 272,
		.height = 181,
	},
};

static const struct display_timing nec_nl12880bc20_05_timing = {
	.pixelclock = { 67000000, 71000000, 75000000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 2, 30, 30 },
	.hback_porch = { 6, 100, 100 },
	.hsync_len = { 2, 30, 30 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 5, 5, 5 },
	.vback_porch = { 11, 11, 11 },
	.vsync_len = { 7, 7, 7 },
};

static const struct panel_desc nec_nl12880bc20_05 = {
	.timings = &nec_nl12880bc20_05_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 261,
		.height = 163,
	},
	.delay = {
		.enable = 50,
		.disable = 50,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode nec_nl4827hc19_05b_mode =
	SP_DISPLAY_MODE(10870, 480, 2, 41, 2, 272, 2, 4, 2, 74,
			 DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC);

static const struct panel_desc nec_nl4827hc19_05b = {
	.modes = &nec_nl4827hc19_05b_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 95,
		.height = 54,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
	.bus_flags = DRM_BUS_FLAG_PIXDATA_POSEDGE,
};

static const struct drm_display_mode netron_dy_e231732_mode =
	SP_DISPLAY_MODE(66000, 1024, 160, 70, 90, 600, 127, 20, 3, 60, 0);

static const struct panel_desc netron_dy_e231732 = {
	.modes = &netron_dy_e231732_mode,
	.num_modes = 1,
	.size = {
		.width = 154,
		.height = 87,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct display_timing nlt_nl192108ac18_02d_timing = {
	.pixelclock = { 130000000, 148350000, 163000000 },
	.hactive = { 1920, 1920, 1920 },
	.hfront_porch = { 80, 100, 100 },
	.hback_porch = { 100, 120, 120 },
	.hsync_len = { 50, 60, 60 },
	.vactive = { 1080, 1080, 1080 },
	.vfront_porch = { 12, 30, 30 },
	.vback_porch = { 4, 10, 10 },
	.vsync_len = { 4, 5, 5 },
};

static const struct panel_desc nlt_nl192108ac18_02d = {
	.timings = &nlt_nl192108ac18_02d_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 344,
		.height = 194,
	},
	.delay = {
		.unprepare = 500,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode nlt_nl12880bc20_mode =
	SP_DISPLAY_MODE(71000, 1280, 50, 60, 50, 800, 5, 13, 5, 0, 0);

static const struct panel_desc nlt_nl12880bc20_jeida = {
	.modes = &nlt_nl12880bc20_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 261,
		.height = 163,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA,
};

static const struct panel_desc nlt_nl12880bc20_spwg_18 = {
	.modes = &nlt_nl12880bc20_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 261,
		.height = 163,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,
};

static const struct panel_desc nlt_nl12880bc20_spwg_24 = {
	.modes = &nlt_nl12880bc20_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 261,
		.height = 163,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode nvd_9128_mode =
	SP_DISPLAY_MODE(29500, 800, 130, 98, 0, 480, 10, 50, 0, 0, 0);

static const struct panel_desc nvd_9128 = {
	.modes = &nvd_9128_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 156,
		.height = 88,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct display_timing okaya_rs800480t_7x0gp_timing = {
	.pixelclock = { 30000000, 30000000, 40000000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = { 40, 40, 40 },
	.hback_porch = { 40, 40, 40 },
	.hsync_len = { 1, 48, 48 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 13, 13, 13 },
	.vback_porch = { 29, 29, 29 },
	.vsync_len = { 3, 3, 3 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc okaya_rs800480t_7x0gp = {
	.timings = &okaya_rs800480t_7x0gp_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 154,
		.height = 87,
	},
	.delay = {
		.prepare = 41,
		.enable = 50,
		.unprepare = 41,
		.disable = 50,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct drm_display_mode olimex_lcd_olinuxino_43ts_mode =
	SP_DISPLAY_MODE(9000, 480, 5, 30, 10, 272, 8, 5, 3, 60, 0);

static const struct panel_desc olimex_lcd_olinuxino_43ts = {
	.modes = &olimex_lcd_olinuxino_43ts_mode,
	.num_modes = 1,
	.size = {
		.width = 105,
		.height = 67,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

/*
 * 800x480 CVT. The panel appears to be quite accepting, at least as far as
 * pixel clocks, but this is the timing that was being used in the Adafruit
 * installation instructions.
 */
static const struct drm_display_mode ontat_yx700wv03_mode =
	SP_DISPLAY_MODE(29500, 800, 24, 72, 96, 480, 3, 10, 7, 60,
			DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC);

/*
 * Specification at:
 * https://www.adafruit.com/images/product-files/2406/c3163.pdf
 */
static const struct panel_desc ontat_yx700wv03 = {
	.modes = &ontat_yx700wv03_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 154,
		.height = 83,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct drm_display_mode ortustech_com43h4m85ulc_mode  =
	SP_DISPLAY_MODE(25000, 480, 10, 10, 15, 800, 3, 3, 3, 60, 0);

static const struct panel_desc ortustech_com43h4m85ulc = {
	.modes = &ortustech_com43h4m85ulc_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 56,
		.height = 93,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
	.bus_flags = DRM_BUS_FLAG_DE_HIGH | DRM_BUS_FLAG_PIXDATA_POSEDGE,
};

static const struct drm_display_mode qd43003c0_40_mode =
	SP_DISPLAY_MODE(9000, 480, 8, 4, 39, 272, 4, 10, 2, 60, 0);

static const struct panel_desc qd43003c0_40 = {
	.modes = &qd43003c0_40_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 95,
		.height = 53,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct drm_display_mode samsung_lsn122dl01_c01_mode =
	SP_DISPLAY_MODE(271560, 2560, 48, 32, 80, 1600, 2, 5, 57, 60, 0);

static const struct panel_desc samsung_lsn122dl01_c01 = {
	.modes = &samsung_lsn122dl01_c01_mode,
	.num_modes = 1,
	.size = {
		.width = 263,
		.height = 164,
	},
};

static const struct drm_display_mode samsung_ltn101nt05_mode =
	SP_DISPLAY_MODE(54030, 1024, 24, 136, 160, 600, 3, 6, 61, 60, 0);

static const struct panel_desc samsung_ltn101nt05 = {
	.modes = &samsung_ltn101nt05_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 223,
		.height = 125,
	},
};

static const struct drm_display_mode samsung_ltn140at29_301_mode =
	SP_DISPLAY_MODE(76300, 1366, 64, 48, 128, 768, 2, 5, 17, 60, 0);

static const struct panel_desc samsung_ltn140at29_301 = {
	.modes = &samsung_ltn140at29_301_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 320,
		.height = 187,
	},
};

static const struct display_timing sharp_lq101k1ly04_timing = {
	.pixelclock = { 60000000, 65000000, 80000000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 20, 20, 20 },
	.hback_porch = { 20, 20, 20 },
	.hsync_len = { 10, 10, 10 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 4, 4, 4 },
	.vback_porch = { 4, 4, 4 },
	.vsync_len = { 4, 4, 4 },
	.flags = DISPLAY_FLAGS_PIXDATA_POSEDGE,
};

static const struct panel_desc sharp_lq101k1ly04 = {
	.timings = &sharp_lq101k1ly04_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 217,
		.height = 136,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA,
};

static const struct drm_display_mode sharp_lq123p1jx31_mode =
	SP_DISPLAY_MODE(252750, 2400, 48, 32, 80, 1600, 3, 10, 33, 60,
			 DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC);

static const struct panel_desc sharp_lq123p1jx31 = {
	.modes = &sharp_lq123p1jx31_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 259,
		.height = 173,
	},
	.delay = {
		.prepare = 110,
		.enable = 50,
		.unprepare = 550,
	},
};

static const struct drm_display_mode sharp_lq150x1lg11_mode =
	SP_DISPLAY_MODE(71100, 1024, 168, 64, 88, 768, 37, 2, 8, 60, 0);

static const struct panel_desc sharp_lq150x1lg11 = {
	.modes = &sharp_lq150x1lg11_mode,
	.num_modes = 1,
	.bpc = 6,
	.size = {
		.width = 304,
		.height = 228,
	},
	.bus_format = MEDIA_BUS_FMT_RGB565_1X16,
};

static const struct drm_display_mode shelly_sca07010_bfn_lnn_mode =
	SP_DISPLAY_MODE(33300, 800, 1, 64, 64, 480, 1, 23, 22, 60, 0);

static const struct panel_desc shelly_sca07010_bfn_lnn = {
	.modes = &shelly_sca07010_bfn_lnn_mode,
	.num_modes = 1,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct drm_display_mode starry_kr122ea0sra_mode =
	SP_DISPLAY_MODE(147000, 1920, 16, 16, 32, 1200, 15, 2, 18, 60,
			 DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC);

static const struct panel_desc starry_kr122ea0sra = {
	.modes = &starry_kr122ea0sra_mode,
	.num_modes = 1,
	.size = {
		.width = 263,
		.height = 164,
	},
	.delay = {
		.prepare = 10 + 200,
		.enable = 50,
		.unprepare = 10 + 500,
	},
};

static const struct display_timing tianma_tm070jdhg30_timing = {
	.pixelclock = { 62600000, 68200000, 78100000 },
	.hactive = { 1280, 1280, 1280 },
	.hfront_porch = { 15, 64, 159 },
	.hback_porch = { 5, 5, 5 },
	.hsync_len = { 1, 1, 256 },
	.vactive = { 800, 800, 800 },
	.vfront_porch = { 3, 40, 99 },
	.vback_porch = { 2, 2, 2 },
	.vsync_len = { 1, 1, 128 },
	.flags = DISPLAY_FLAGS_DE_HIGH,
};

static const struct panel_desc tianma_tm070jdhg30 = {
	.timings = &tianma_tm070jdhg30_timing,
	.num_timings = 1,
	.bpc = 8,
	.size = {
		.width = 151,
		.height = 95,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,
};

static const struct drm_display_mode tpk_f07a_0102_mode =
	SP_DISPLAY_MODE(33260, 800, 40, 128, 88, 480, 10, 2, 33, 60, 0);

static const struct panel_desc tpk_f07a_0102 = {
	.modes = &tpk_f07a_0102_mode,
	.num_modes = 1,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_flags = DRM_BUS_FLAG_PIXDATA_POSEDGE,
};

static const struct drm_display_mode tpk_f10a_0102_mode =
	SP_DISPLAY_MODE(45000, 1024, 176, 5, 88, 600, 20, 5, 25, 60, 0);

static const struct panel_desc tpk_f10a_0102 = {
	.modes = &tpk_f10a_0102_mode,
	.num_modes = 1,
	.size = {
		.width = 223,
		.height = 125,
	},
};

static const struct display_timing urt_umsh_8596md_timing = {
	.pixelclock = { 33260000, 33260000, 33260000 },
	.hactive = { 800, 800, 800 },
	.hfront_porch = { 41, 41, 41 },
	.hback_porch = { 216 - 128, 216 - 128, 216 - 128 },
	.hsync_len = { 71, 128, 128 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 10, 10, 10 },
	.vback_porch = { 35 - 2, 35 - 2, 35 - 2 },
	.vsync_len = { 2, 2, 2 },
	.flags = DISPLAY_FLAGS_DE_HIGH | DISPLAY_FLAGS_PIXDATA_NEGEDGE |
		DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW,
};

static const struct panel_desc urt_umsh_8596md_lvds = {
	.timings = &urt_umsh_8596md_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,
};

static const struct panel_desc urt_umsh_8596md_parallel = {
	.timings = &urt_umsh_8596md_timing,
	.num_timings = 1,
	.bpc = 6,
	.size = {
		.width = 152,
		.height = 91,
	},
	.bus_format = MEDIA_BUS_FMT_RGB666_1X18,
};

static const struct drm_display_mode winstar_wf35ltiacd_mode =
	SP_DISPLAY_MODE(6410, 320, 20, 30, 38, 240, 4, 3, 15, 60,
			DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC);

static const struct panel_desc winstar_wf35ltiacd = {
	.modes = &winstar_wf35ltiacd_mode,
	.num_modes = 1,
	.bpc = 8,
	.size = {
		.width = 70,
		.height = 53,
	},
	.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
};

static const struct of_device_id platform_of_match[] = {
	{
		.compatible = "ampire,am-480272h3tmqw-t01h",
		.data = &ampire_am_480272h3tmqw_t01h,
	}, {
		.compatible = "ampire,am800480r3tmqwa1h",
		.data = &ampire_am800480r3tmqwa1h,
	}, {
		.compatible = "auo,b101aw03",
		.data = &auo_b101aw03,
	}, {
		.compatible = "auo,b101ean01",
		.data = &auo_b101ean01,
	}, {
		.compatible = "auo,b101xtn01",
		.data = &auo_b101xtn01,
	}, {
		.compatible = "auo,b116xw03",
		.data = &auo_b116xw03,
	}, {
		.compatible = "auo,b133htn01",
		.data = &auo_b133htn01,
	}, {
		.compatible = "auo,b133xtn01",
		.data = &auo_b133xtn01,
	}, {
		.compatible = "auo,g133han01",
		.data = &auo_g133han01,
	}, {
		.compatible = "auo,g185han01",
		.data = &auo_g185han01,
	}, {
		.compatible = "auo,p320hvn03",
		.data = &auo_p320hvn03,
	}, {
		.compatible = "auo,t215hvn01",
		.data = &auo_t215hvn01,
	}, {
		.compatible = "avic,tm070ddh03",
		.data = &avic_tm070ddh03,
	}, {
		.compatible = "boe,nv101wxmn51",
		.data = &boe_nv101wxmn51,
	}, {
		.compatible = "chunghwa,claa070wp03xg",
		.data = &chunghwa_claa070wp03xg,
	}, {
		.compatible = "chunghwa,claa101wa01a",
		.data = &chunghwa_claa101wa01a
	}, {
		.compatible = "chunghwa,claa101wb01",
		.data = &chunghwa_claa101wb01
	}, {
		.compatible = "edt,et0350g0dh6",
		.data = &edt_et0350g0dh6,
	}, {
		.compatible = "edt,et0430g0dh6",
		.data = &edt_et0430g0dh6,
	}, {
		.compatible = "edt,et057090dhu",
		.data = &edt_et057090dhu,
	}, {
		.compatible = "edt,et070080dh6",
		.data = &edt_etm0700g0dh6,
	}, {
		.compatible = "edt,etm0700g0dh6",
		.data = &edt_etm0700g0dh6,
	}, {
		.compatible = "foxlink,fl500wvr00-a0t",
		.data = &foxlink_fl500wvr00_a0t,
	}, {
		.compatible = "giantplus,gpg482739qs5",
		.data = &giantplus_gpg482739qs5
	}, {
		.compatible = "hannstar,hsd070pww1",
		.data = &hannstar_hsd070pww1,
	}, {
		.compatible = "hannstar,hsd100pxn1",
		.data = &hannstar_hsd100pxn1,
	}, {
		.compatible = "hit,tx23d38vm0caa",
		.data = &hitachi_tx23d38vm0caa
	}, {
		.compatible = "innolux,at043tn24",
		.data = &innolux_at043tn24,
	}, {
		.compatible = "innolux,at070tn92",
		.data = &innolux_at070tn92,
	}, {
		.compatible ="innolux,g101ice-l01",
		.data = &innolux_g101ice_l01
	}, {
		.compatible ="innolux,g121i1-l01",
		.data = &innolux_g121i1_l01
	}, {
		.compatible = "innolux,g121x1-l03",
		.data = &innolux_g121x1_l03,
	}, {
		.compatible = "innolux,n116bge",
		.data = &innolux_n116bge,
	}, {
		.compatible = "innolux,n156bge-l21",
		.data = &innolux_n156bge_l21,
	}, {
		.compatible = "innolux,zj070na-01p",
		.data = &innolux_zj070na_01p,
	}, {
		.compatible = "kyo,tcg121xglp",
		.data = &kyo_tcg121xglp,
	}, {
		.compatible = "lg,lb070wv8",
		.data = &lg_lb070wv8,
	}, {
		.compatible = "lg,lp079qx1-sp0v",
		.data = &lg_lp079qx1_sp0v,
	}, {
		.compatible = "lg,lp097qx1-spa1",
		.data = &lg_lp097qx1_spa1,
	}, {
		.compatible = "lg,lp120up1",
		.data = &lg_lp120up1,
	}, {
		.compatible = "lg,lp129qe",
		.data = &lg_lp129qe,
	}, {
		.compatible = "nec,nl12880bc20-05",
		.data = &nec_nl12880bc20_05,
	}, {
		.compatible = "nec,nl4827hc19-05b",
		.data = &nec_nl4827hc19_05b,
	}, {
		.compatible = "netron-dy,e231732",
		.data = &netron_dy_e231732,
	}, {
		.compatible = "nlt,nl192108ac18-02d",
		.data = &nlt_nl192108ac18_02d,
	}, {
		.compatible = "nlt,nl12880bc20-jeida",
		.data = &nlt_nl12880bc20_jeida,
	}, {
		.compatible = "nlt,nl12880bc20-spwg-18",
		.data = &nlt_nl12880bc20_spwg_18,
	}, {
		.compatible = "nlt,nl12880bc20-spwg-24",
		.data = &nlt_nl12880bc20_spwg_24,
	}, {
		.compatible = "nvd,9128",
		.data = &nvd_9128,
	}, {
		.compatible = "okaya,rs800480t-7x0gp",
		.data = &okaya_rs800480t_7x0gp,
	}, {
		.compatible = "olimex,lcd-olinuxino-43-ts",
		.data = &olimex_lcd_olinuxino_43ts,
	}, {
		.compatible = "ontat,yx700wv03",
		.data = &ontat_yx700wv03,
	}, {
		.compatible = "ortustech,com43h4m85ulc",
		.data = &ortustech_com43h4m85ulc,
	}, {
		.compatible = "qiaodian,qd43003c0-40",
		.data = &qd43003c0_40,
	}, {
		.compatible = "samsung,lsn122dl01-c01",
		.data = &samsung_lsn122dl01_c01,
	}, {
		.compatible = "samsung,ltn101nt05",
		.data = &samsung_ltn101nt05,
	}, {
		.compatible = "samsung,ltn140at29-301",
		.data = &samsung_ltn140at29_301,
	}, {
		.compatible = "sharp,lq101k1ly04",
		.data = &sharp_lq101k1ly04,
	}, {
		.compatible = "sharp,lq123p1jx31",
		.data = &sharp_lq123p1jx31,
	}, {
		.compatible = "sharp,lq150x1lg11",
		.data = &sharp_lq150x1lg11,
	}, {
		.compatible = "shelly,sca07010-bfn-lnn",
		.data = &shelly_sca07010_bfn_lnn,
	}, {
		.compatible = "starry,kr122ea0sra",
		.data = &starry_kr122ea0sra,
	}, {
		.compatible = "tianma,tm070jdhg30",
		.data = &tianma_tm070jdhg30,
	}, {
		.compatible = "tpk,f07a-0102",
		.data = &tpk_f07a_0102,
	}, {
		.compatible = "tpk,f10a-0102",
		.data = &tpk_f10a_0102,
	}, {
		.compatible = "urt,umsh-8596md-t",
		.data = &urt_umsh_8596md_parallel,
	}, {
		.compatible = "urt,umsh-8596md-1t",
		.data = &urt_umsh_8596md_parallel,
	}, {
		.compatible = "urt,umsh-8596md-7t",
		.data = &urt_umsh_8596md_parallel,
	}, {
		.compatible = "urt,umsh-8596md-11t",
		.data = &urt_umsh_8596md_lvds,
	}, {
		.compatible = "urt,umsh-8596md-19t",
		.data = &urt_umsh_8596md_lvds,
	}, {
		.compatible = "urt,umsh-8596md-20t",
		.data = &urt_umsh_8596md_parallel,
	}, {
		.compatible = "winstar,wf35ltiacd",
		.data = &winstar_wf35ltiacd,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, platform_of_match);

static int panel_simple_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return panel_simple_probe(&pdev->dev, id->data);
}

static int panel_simple_platform_remove(struct platform_device *pdev)
{
	return panel_simple_remove(&pdev->dev);
}

static void panel_simple_platform_shutdown(struct platform_device *pdev)
{
	panel_simple_shutdown(&pdev->dev);
}

static struct platform_driver panel_simple_platform_driver = {
	.driver = {
		.name = "panel-simple",
		.of_match_table = platform_of_match,
	},
	.probe = panel_simple_platform_probe,
	.remove = panel_simple_platform_remove,
	.shutdown = panel_simple_platform_shutdown,
};

struct panel_desc_dsi {
	struct panel_desc desc;

	unsigned long flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
};

static const struct drm_display_mode auo_b080uan01_mode =
	SP_DISPLAY_MODE(154500, 1200, 62, 4, 62, 1920, 9, 2, 8, 60, 0);

static const struct panel_desc_dsi auo_b080uan01 = {
	.desc = {
		.modes = &auo_b080uan01_mode,
		.num_modes = 1,
		.bpc = 8,
		.size = {
			.width = 108,
			.height = 272,
		},
	},
	.flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};

static const struct drm_display_mode boe_tv080wum_nl0_mode =
	SP_DISPLAY_MODE(160000, 1200, 120, 20, 21, 1920, 21, 3, 18, 60,
			 DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC);

static const struct panel_desc_dsi boe_tv080wum_nl0 = {
	.desc = {
		.modes = &boe_tv080wum_nl0_mode,
		.num_modes = 1,
		.size = {
			.width = 107,
			.height = 172,
		},
	},
	.flags = MIPI_DSI_MODE_VIDEO |
		 MIPI_DSI_MODE_VIDEO_BURST |
		 MIPI_DSI_MODE_VIDEO_SYNC_PULSE,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};

static const struct drm_display_mode lg_ld070wx3_sl01_mode =
	SP_DISPLAY_MODE(71000, 800, 32, 1, 57, 1280, 28, 1, 14, 60, 0);

static const struct panel_desc_dsi lg_ld070wx3_sl01 = {
	.desc = {
		.modes = &lg_ld070wx3_sl01_mode,
		.num_modes = 1,
		.bpc = 8,
		.size = {
			.width = 94,
			.height = 151,
		},
	},
	.flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};

static const struct drm_display_mode lg_lh500wx1_sd03_mode =
	SP_DISPLAY_MODE(67000, 720, 12, 4, 112, 1280, 8, 4, 12, 60, 0);

static const struct panel_desc_dsi lg_lh500wx1_sd03 = {
	.desc = {
		.modes = &lg_lh500wx1_sd03_mode,
		.num_modes = 1,
		.bpc = 8,
		.size = {
			.width = 62,
			.height = 110,
		},
	},
	.flags = MIPI_DSI_MODE_VIDEO,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};

static const struct drm_display_mode panasonic_vvx10f004b00_mode =
	SP_DISPLAY_MODE(157200, 1920, 154, 16, 32, 1200, 17, 2, 16, 60, 0);

static const struct panel_desc_dsi panasonic_vvx10f004b00 = {
	.desc = {
		.modes = &panasonic_vvx10f004b00_mode,
		.num_modes = 1,
		.bpc = 8,
		.size = {
			.width = 217,
			.height = 136,
		},
	},
	.flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
		 MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};

static const struct of_device_id dsi_of_match[] = {
	{
		.compatible = "auo,b080uan01",
		.data = &auo_b080uan01
	}, {
		.compatible = "boe,tv080wum-nl0",
		.data = &boe_tv080wum_nl0
	}, {
		.compatible = "lg,ld070wx3-sl01",
		.data = &lg_ld070wx3_sl01
	}, {
		.compatible = "lg,lh500wx1-sd03",
		.data = &lg_lh500wx1_sd03
	}, {
		.compatible = "panasonic,vvx10f004b00",
		.data = &panasonic_vvx10f004b00
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, dsi_of_match);

static int panel_simple_dsi_probe(struct mipi_dsi_device *dsi)
{
	const struct panel_desc_dsi *desc;
	const struct of_device_id *id;
	int err;

	id = of_match_node(dsi_of_match, dsi->dev.of_node);
	if (!id)
		return -ENODEV;

	desc = id->data;

	err = panel_simple_probe(&dsi->dev, &desc->desc);
	if (err < 0)
		return err;

	dsi->mode_flags = desc->flags;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;

	return mipi_dsi_attach(dsi);
}

static int panel_simple_dsi_remove(struct mipi_dsi_device *dsi)
{
	int err;

	err = mipi_dsi_detach(dsi);
	if (err < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", err);

	return panel_simple_remove(&dsi->dev);
}

static void panel_simple_dsi_shutdown(struct mipi_dsi_device *dsi)
{
	panel_simple_shutdown(&dsi->dev);
}

static struct mipi_dsi_driver panel_simple_dsi_driver = {
	.driver = {
		.name = "panel-simple-dsi",
		.of_match_table = dsi_of_match,
	},
	.probe = panel_simple_dsi_probe,
	.remove = panel_simple_dsi_remove,
	.shutdown = panel_simple_dsi_shutdown,
};

static int __init panel_simple_init(void)
{
	int err;

	err = platform_driver_register(&panel_simple_platform_driver);
	if (err < 0)
		return err;

	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI)) {
		err = mipi_dsi_driver_register(&panel_simple_dsi_driver);
		if (err < 0)
			return err;
	}

	return 0;
}
module_init(panel_simple_init);

static void __exit panel_simple_exit(void)
{
	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_unregister(&panel_simple_dsi_driver);

	platform_driver_unregister(&panel_simple_platform_driver);
}
module_exit(panel_simple_exit);

MODULE_AUTHOR("Thierry Reding <treding@nvidia.com>");
MODULE_DESCRIPTION("DRM Driver for Simple Panels");
MODULE_LICENSE("GPL and additional rights");
