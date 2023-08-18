// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Markus Bauer <MB@karo-electronics.de>
 *
 * based on: drivers/gpu/drm/panel/panel-rpi-7inch.c
 * from: https://github.com/TinkerEdgeT/mendel-linux-imx
 *
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

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/rpi_display.h>

#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

/* DSI PPI Layer Registers */
#define PPI_STARTPPI		0x0104
#define PPI_LPTXTIMECNT		0x0114
#define PPI_D0S_ATMR		0x0144
#define PPI_D1S_ATMR		0x0148
#define PPI_D0S_CLRSIPOCOUNT	0x0164
#define PPI_D1S_CLRSIPOCOUNT	0x0168

/* DSI Protocol Layer Registers */
#define DSI_STARTDSI		0x0204
#define DSI_BUSYDSI		0x0208
#define DSI_LANEENABLE		0x0210
#define DSI_LANEENABLE_CLOCK	BIT(0)
#define DSI_LANEENABLE_D0	BIT(1)
#define DSI_LANEENABLE_D1	BIT(2)

/* LCDC/DPI Host Registers */
#define LCDCTRL			0x0420

/* SPI Master Registers */
#define SPICMR			0x0450

/* System Controller Registers */
#define SYSCTRL			0x0464

static int trigger_bridge = 1;

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct display_timing *timings;
	unsigned int num_timings;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *		become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *		display the first valid frame after starting to receive
	 *		video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *		turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *		to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;

	u32 bus_format;
};

struct tc358762 {
	struct drm_panel base;
	bool prepared;
	bool enabled;

	struct device *dev;
	struct mipi_dsi_device *dsi;
	const struct panel_desc *desc;

	struct regulator *supply;

	struct gpio_desc *enable_gpio;
};

static const struct drm_display_mode tc358762_mode = {
	.clock = 27448,
	.hdisplay = 800,
	.hsync_start = 800 + 70,
	.hsync_end = 800 + 70 + 20,
	.htotal = 800 + 70 + 20 + 23,
	.vdisplay = 480,
	.vsync_start = 480 + 7,
	.vsync_end = 480 + 7 + 2,
	.vtotal = 480 + 7 + 2 + 21,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};

static inline struct tc358762 *to_tc358762(struct drm_panel *panel)
{
	return container_of(panel, struct tc358762, base);
}

static int tc358762_disable(struct drm_panel *panel)
{
	struct tc358762 *p = to_tc358762(panel);

	if (!p->enabled)
		return 0;

	pr_debug("panel disable\n");

	rpi_display_set_bright(0x00);

	if (p->desc && p->desc->delay.disable)
		msleep(p->desc->delay.disable);

	p->enabled = false;

	return 0;
}

static int tc358762_unprepare(struct drm_panel *panel)
{
	struct tc358762 *p = to_tc358762(panel);

	if (!p->prepared)
		return 0;

	if (p->enable_gpio)
		gpiod_direction_output(p->enable_gpio, 0);

	regulator_disable(p->supply);

	if (p->desc && p->desc->delay.unprepare)
		msleep(p->desc->delay.unprepare);

	p->prepared = false;

	return 0;
}

static int rpi_touchscreen_write(struct mipi_dsi_device *dsi, u16 reg, u32 val)
{
	u8 msg[] = {
		reg,
		reg >> 8,
		val,
		val >> 8,
		val >> 16,
		val >> 24,
	};

	mipi_dsi_generic_write(dsi, msg, sizeof(msg));

	return 0;
}

static int tc358762_dsi_init(struct tc358762 *p)
{
	struct mipi_dsi_device *dsi = p->dsi;

	rpi_touchscreen_write(dsi, DSI_LANEENABLE,
			      DSI_LANEENABLE_CLOCK |
			      DSI_LANEENABLE_D0);

	rpi_touchscreen_write(dsi, PPI_D0S_CLRSIPOCOUNT, 0x05);
	rpi_touchscreen_write(dsi, PPI_D1S_CLRSIPOCOUNT, 0x05);
	rpi_touchscreen_write(dsi, PPI_D0S_ATMR, 0x00);
	rpi_touchscreen_write(dsi, PPI_D1S_ATMR, 0x00);
	rpi_touchscreen_write(dsi, PPI_LPTXTIMECNT, 0x03);

	rpi_touchscreen_write(dsi, SPICMR, 0x00);
	rpi_touchscreen_write(dsi, LCDCTRL, 0x00100150);
	rpi_touchscreen_write(dsi, SYSCTRL, 0x040f);
	msleep(100);

	rpi_touchscreen_write(dsi, PPI_STARTPPI, 0x01);
	rpi_touchscreen_write(dsi, DSI_STARTDSI, 0x01);
	msleep(100);

	return 0;
}

static int tc358762_prepare(struct drm_panel *panel)
{
	struct tc358762 *p = to_tc358762(panel);
	int err;

	if (p->prepared)
		return 0;

	err = regulator_enable(p->supply);
	if (err < 0) {
		dev_err(panel->dev, "failed to enable supply: %d\n", err);
		return err;
	}

	if (p->enable_gpio)
		gpiod_direction_output(p->enable_gpio, 1);

	if (p->desc && p->desc->delay.prepare)
		msleep(p->desc->delay.prepare);

	p->prepared = true;

	return 0;
}

static int tc358762_enable(struct drm_panel *panel)
{
	struct tc358762 *p = to_tc358762(panel);

	if (p->enabled)
		return 0;

	pr_debug("panel enable\n");

	if (trigger_bridge) {
		pr_debug("rpi_display_power_up");
		rpi_display_screen_power_up();
		trigger_bridge = 0;
		msleep(100);
		pr_debug("rpi_ft5406_start_polling");
		rpi_ft5406_start_polling();
	}

	tc358762_dsi_init(p);

	rpi_display_set_bright(0xFF);

	p->enabled = true;

	return 0;
}

static int tc358762_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &tc358762_mode);
	if (!mode) {
		DRM_ERROR("failed to add mode %ux%ux\n",
			  tc358762_mode.hdisplay, tc358762_mode.vdisplay);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	return 1;
}

static int tc358762_get_timings(struct drm_panel *panel,
				unsigned int num_timings,
				struct display_timing *timings)
{
	struct tc358762 *p = to_tc358762(panel);
	unsigned int i;

	if (!p->desc)
		return 0;

	if (p->desc->num_timings < num_timings)
		num_timings = p->desc->num_timings;

	if (timings)
		for (i = 0; i < num_timings; i++)
			timings[i] = p->desc->timings[i];

	return p->desc->num_timings;
}

static const struct drm_panel_funcs tc358762_funcs = {
	.disable = tc358762_disable,
	.unprepare = tc358762_unprepare,
	.prepare = tc358762_prepare,
	.enable = tc358762_enable,
	.get_modes = tc358762_get_modes,
	.get_timings = tc358762_get_timings,
};

static int tc358762_mipi_probe(struct mipi_dsi_device *dsi, const struct panel_desc *desc)
{
	struct tc358762 *panel;
	struct device *dev = &dsi->dev;
	int err;

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	panel->enabled = false;
	panel->prepared = false;
	panel->desc = desc;
	panel->dev = dev;
	panel->dsi = dsi;

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

	drm_panel_init(&panel->base, dev, &tc358762_funcs, DRM_MODE_CONNECTOR_DSI);
	panel->base.dev = dev;
	panel->base.funcs = &tc358762_funcs;

	drm_panel_add(&panel->base);

	dev_set_drvdata(dev, panel);

	return 0;
}

static int tc358762_remove(struct device *dev)
{
	struct tc358762 *panel = dev_get_drvdata(dev);

	drm_panel_remove(&panel->base);

	tc358762_disable(&panel->base);

	return 0;
}

static void tc358762_shutdown(struct device *dev)
{
	struct tc358762 *panel = dev_get_drvdata(dev);

	tc358762_disable(&panel->base);
}

struct bridge_desc {
	struct panel_desc desc;

	unsigned long flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
};

static const struct bridge_desc tc358762_bridge = {
	.desc = {
		.modes = &tc358762_mode,
		.num_modes = 1,
		.bpc = 8,
		.size = {
			.width = 154,
			.height = 86,
		},
		.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
	},
	.flags = MIPI_DSI_MODE_VIDEO |
		 MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
		 MIPI_DSI_MODE_LPM |
		 MIPI_DSI_MODE_VIDEO_HSE,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 1,
};

static const struct of_device_id dsi_of_match[] = {
	{
		.compatible = "raspberrypi,tc358762",
		.data = &tc358762_bridge
	},
	{
		/*
		 * keep the unofficial 'rpi' vendor prefix for
		 * compatibility with existing dtbs
		 */
		.compatible = "rpi,tc358762",
		.data = &tc358762_bridge
	},
	{
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, dsi_of_match);

static int tc358762_dsi_probe(struct mipi_dsi_device *dsi)
{
	const struct bridge_desc *desc;
	const struct of_device_id *id;
	const struct panel_desc *pdesc;
	u32 val;
	int err;

	id = of_match_node(dsi_of_match, dsi->dev.of_node);
	if (!id)
		return -ENODEV;

	if (of_device_is_compatible(dsi->dev.of_node, "rpi,tc358762"))
		dev_warn(&dsi->dev,
			 "Your DTB needs to be updated to use the official vendor prefix 'raspberrypi' rather than 'rpi'");

	desc = id->data;

	if (desc) {
		dsi->mode_flags = desc->flags;
		dsi->format = desc->format;
		dsi->lanes = desc->lanes;
		pdesc = &desc->desc;
	} else {
		pdesc = NULL;
	}

	err = tc358762_mipi_probe(dsi, pdesc);

	if (err < 0)
		return err;

	if (!of_property_read_u32(dsi->dev.of_node, "dsi,flags", &val))
		dsi->mode_flags = val;

	if (!of_property_read_u32(dsi->dev.of_node, "dsi,format", &val))
		dsi->format = val;

	if (!of_property_read_u32(dsi->dev.of_node, "dsi,lanes", &val))
		dsi->lanes = val;

	return mipi_dsi_attach(dsi);
}

static void tc358762_dsi_remove(struct mipi_dsi_device *dsi)
{
	int err;

	err = mipi_dsi_detach(dsi);
	if (err < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", err);

	tc358762_remove(&dsi->dev);
}

static void tc358762_dsi_shutdown(struct mipi_dsi_device *dsi)
{
	tc358762_shutdown(&dsi->dev);
}

static struct mipi_dsi_driver tc358762_dsi_driver = {
	.driver = {
		.name = "bridge-tc358762-dsi",
		.of_match_table = dsi_of_match,
	},
	.probe = tc358762_dsi_probe,
	.remove = tc358762_dsi_remove,
	.shutdown = tc358762_dsi_shutdown,
};

static int __init tc358762_init(void)
{
	int err;

	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI)) {
		err = mipi_dsi_driver_register(&tc358762_dsi_driver);
		if (err < 0)
			return err;
	}

	return 0;
}
module_init(tc358762_init);

static void __exit tc358762_exit(void)
{
	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_unregister(&tc358762_dsi_driver);
}
module_exit(tc358762_exit);

MODULE_AUTHOR("Jerry <xbl@rock-chips.com>");
MODULE_DESCRIPTION("DRM Driver for toshiba tc358762 Bridge");
MODULE_LICENSE("GPL and additional rights");
