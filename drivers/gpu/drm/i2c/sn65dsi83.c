/*
 * Copyright (c) 2017 Lothar Waßmann <LW@KARO-electronics.de>
 *
 * Texas Instruments SN65DSI83 MIPI DSI to LVDS converter driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <video/of_display_timing.h>

#include "sn65dsi83.h"

#define SYNC_DELAY	22

struct sn65dsi83 {
	struct i2c_client *client;
	struct i2c_client *i2c_edid;

	struct regmap *regmap;
	struct drm_panel *panel;

	enum drm_connector_status status;
	bool powered;

	struct drm_display_mode curr_mode;
	bool mode_valid;
	int bus_format;

	unsigned int current_edid_segment;
	uint8_t edid_buf[256];
	bool edid_read;
	struct edid *edid;

	wait_queue_head_t wq;
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct drm_encoder *encoder;

	struct regulator *vcc_reg;
	struct clk *refclk;
	struct gpio_desc *enable_gpio;

	struct device_node *host_node;
	struct mipi_dsi_device *dsi;
	u8 num_dsi_lanes;

	u8 errstat;
};

static struct sn65dsi83 *encoder_to_sn65dsi83(struct drm_encoder *encoder)
{
	return to_encoder_slave(encoder)->slave_priv;
}

static const struct reg_sequence sn65dsi83_fixed_registers[] = {
	{ 0x00, 0x35, },
	{ 0x01, 0x38, },
	{ 0x02, 0x49, },
	{ 0x03, 0x53, },
	{ 0x04, 0x44, },
	{ 0x05, 0x20, },
	{ 0x06, 0x20, },
	{ 0x07, 0x20, },
	{ 0x08, 0x01, },
};

static const uint8_t sn65dsi83_register_defaults[] = {
	/* 0     1     2     3     4     5     6     7
	 * 8     9     a     b     c     d     e     f */
	0x01, 0x20, 0x20, 0x20, 0x44, 0x53, 0x49, 0x38, /* 00 */
	0x35, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 10 */
	0x70, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 20 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 30 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 40 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 50 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 60 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 70 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 80 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 90 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* a0 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* b0 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* c0 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* d0 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, /* e0 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* f0 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static bool sn65dsi83_register_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SN65DSI83_REG_CHIPREV(0):
	case SN65DSI83_REG_CHIPREV(1):
	case SN65DSI83_REG_CHIPREV(2):
	case SN65DSI83_REG_CHIPREV(3):
	case SN65DSI83_REG_CHIPREV(4):
	case SN65DSI83_REG_CHIPREV(5):
	case SN65DSI83_REG_CHIPREV(6):
	case SN65DSI83_REG_CHIPREV(7):
	case SN65DSI83_REG_CHIPREV(8):
	case SN65DSI83_REG_RSTCTRL:
	case SN65DSI83_REG_LVDSCTRL:
	case SN65DSI83_REG_DSICTRL:
	case SN65DSI83_REG_PLLCTRL:
	case SN65DSI83_REG_LVDSCFG0:
	case SN65DSI83_REG_LVDSCFG1:
	case SN65DSI83_REG_LVDSCFG2:
	case SN65DSI83_REG_LVDSCFG3:
	case SN65DSI83_REG_LVDSCFG4:
	case SN65DSI83_REG_LVDSCFG5:
	case SN65DSI83_REG_LVDSCFG6:
	case SN65DSI83_REG_LVDSCFG7:

	case SN65DSI83_REG_LINE_LENGTH_LSB:
	case SN65DSI83_REG_LINE_LENGTH_MSB:
	case SN65DSI83_REG_VERT_SIZE_LSB:
	case SN65DSI83_REG_VERT_SIZE_MSB:
	case SN65DSI83_REG_SYNC_DELAY_LSB:
	case SN65DSI83_REG_SYNC_DELAY_MSB:
	case SN65DSI83_REG_HSYNC_WIDTH_LSB:
	case SN65DSI83_REG_HSYNC_WIDTH_MSB:
	case SN65DSI83_REG_VSYNC_WIDTH_LSB:
	case SN65DSI83_REG_VSYNC_WIDTH_MSB:
	case SN65DSI83_REG_H_BACK_PORCH:
	case SN65DSI83_REG_V_BACK_PORCH:
	case SN65DSI83_REG_H_FRONT_PORCH:
	case SN65DSI83_REG_V_FRONT_PORCH:

	case SN65DSI83_REG_IRQCTRL:
	case SN65DSI83_REG_IRQEN:
	case SN65DSI83_REG_ERRSTAT:
		return true;
	}

	return false;
}

static const struct regmap_config sn65dsi83_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0xff,
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults_raw = sn65dsi83_register_defaults,
	.num_reg_defaults_raw = ARRAY_SIZE(sn65dsi83_register_defaults),

	.volatile_reg = sn65dsi83_register_volatile,
};

/* -----------------------------------------------------------------------------
 * Hardware configuration
 */

static int sn65dsi83_power_on(struct sn65dsi83 *dsi83)
{
	if (dsi83->powered)
		return 0;

	gpiod_set_value(dsi83->enable_gpio, 1);
	regcache_sync(dsi83->regmap);

	if (dsi83->vcc_reg) {
		int ret = regulator_enable(dsi83->vcc_reg);

		if (ret) {
			dev_err(&dsi83->client->dev, "Failed to enable VCC regulator\n");
			return ret;
		}
	}
	dsi83->powered = true;
	return 0;
}

static void sn65dsi83_power_off(struct sn65dsi83 *dsi83)
{
	if (!dsi83->powered)
		return;

	regcache_mark_dirty(dsi83->regmap);
	if (dsi83->vcc_reg)
		regulator_disable(dsi83->vcc_reg);
	gpiod_set_value(dsi83->enable_gpio, 0);
	dsi83->powered = false;
}

static irqreturn_t sn65dsi83_irq_handler(int irq, void *devid)
{
	struct sn65dsi83 *dsi83 = devid;
	int ret;
	unsigned int irqctrl;
	unsigned int irqen;
	unsigned int errstat;

	ret = regmap_read(dsi83->regmap, SN65DSI83_REG_IRQCTRL, &irqctrl);
	if (ret)
		goto regmap_fail;

	if (!irqctrl)
		return IRQ_NONE;

	ret = regmap_read(dsi83->regmap, SN65DSI83_REG_IRQEN, &irqen);
	if (ret)
		goto regmap_fail;

	ret = regmap_read(dsi83->regmap, SN65DSI83_REG_ERRSTAT, &errstat);
	if (ret)
		goto regmap_fail;

	if (errstat != dsi83->errstat) {
		wake_up(&dsi83->wq);
		dsi83->errstat = errstat;
	}

	if (!(irqen & errstat))
		return IRQ_NONE;

	ret = regmap_write(dsi83->regmap, SN65DSI83_REG_ERRSTAT,
			errstat & irqen);
	if (ret)
		goto regmap_fail;

	return IRQ_HANDLED;

regmap_fail:
	dev_err(&dsi83->client->dev, "regmap_read() failed: %d\n", ret);
	return IRQ_NONE;
}

static inline struct sn65dsi83 *connector_to_sn65dsi83(struct drm_connector *connector)
{
	return container_of(connector, struct sn65dsi83, connector);
}

static enum drm_connector_status
sn65dsi83_detect(struct sn65dsi83 *dsi83,
		struct drm_connector *connector)
{
	return connector_status_connected;
}

/*
 * index into this table is value to be written to REG_LVDSCTRL
 * for the frequency (in kHz) listed in the table
 */
static unsigned long lvds_clk_lookup[] = {
	37500,
	62500,
	87500,
	112500,
	137500,
	154000,
};

#define DSI83_WRITE(r, v) do {						\
	int ret = regmap_write(dsi83->regmap, SN65DSI83_REG_##r, v);	\
	if (ret)							\
		return ret;						\
	} while (0)

#define DSI83_READ(r, v) do {						\
	int ret = regmap_read(dsi83->regmap, SN65DSI83_REG_##r, v);	\
	if (ret)							\
		return ret;						\
	} while (0)

#define DSI83_UPDATE_BITS(r, v, m) do {					\
	int ret = regmap_update_bits(dsi83->regmap, SN65DSI83_REG_##r,	\
					v, m);				\
	if (ret)							\
		return ret;						\
	} while (0)

static int sn65dsi83_pll_init(struct sn65dsi83 *dsi83)
{
	int ret;
	size_t i;
	unsigned long lvds_clk = dsi83->curr_mode.clock;
	unsigned long mipi_dsi_clk = lvds_clk * 24 / dsi83->num_dsi_lanes / 2;
	int ref_clk_mult;
	int dsi_clk_div;
	int lvds_clk_src;

	if (dsi83->refclk) {
		unsigned long ref_clk = clk_get_rate(dsi83->refclk);

		ref_clk_mult = lvds_clk / ref_clk;
		dsi_clk_div = 0;
		lvds_clk_src = 0;
	} else {
		ref_clk_mult = 0;
		dsi_clk_div = DIV_ROUND_UP(mipi_dsi_clk, lvds_clk) - 1;
		lvds_clk_src = 1;
	}
	DRM_DEBUG_DRIVER("lvds_clk=%lu mipi_dsi_clk=%lu lvds_clk_src=%u ref_clk_mult=%u dsi_clk_div=%u\n",
			lvds_clk, mipi_dsi_clk, lvds_clk_src, ref_clk_mult,
			dsi_clk_div);

	if (mipi_dsi_clk < 40000 || mipi_dsi_clk > 500000)
		return -EINVAL;
	if (dsi_clk_div > 25)
		return -EINVAL;
	if (lvds_clk < 25000)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(lvds_clk_lookup); i++) {
		if (lvds_clk <= lvds_clk_lookup[i])
			break;
	}

	if (i < ARRAY_SIZE(lvds_clk_lookup)) {
		u32 lvdsctrl, pll;
		int lvds_clk_sel = i << 1;

		DSI83_READ(PLLCTRL, &pll);
		DSI83_READ(LVDSCTRL, &lvdsctrl);

		if (WARN_ON((pll & PLLCTRL_PLL_EN) && ((lvdsctrl & 0xe) == lvds_clk)))
			return 0;

		/* PLL must be disabled before changing LVDSCRTL & DSICTRL */
		if (pll & PLLCTRL_PLL_EN)
			DSI83_WRITE(PLLCTRL, pll & ~PLLCTRL_PLL_EN);

		DSI83_WRITE(LVDSCTRL, (lvdsctrl & ~0xe) | lvds_clk_sel | lvds_clk_src);
		DSI83_WRITE(DSICTRL, (dsi_clk_div << 3) | (ref_clk_mult << 0));
		DSI83_WRITE(LVDSCFG2, mipi_dsi_clk / 5000);
		DSI83_UPDATE_BITS(PLLCTRL, 0x1, 0x1);

		/* clear error status bits */
		DSI83_WRITE(ERRSTAT, ~0);
		ret = 0;
	} else {
		ret = -EINVAL;
		dev_err(&dsi83->client->dev,
			"LVDS clk %lu out of range 25000000 .. 154000000\n",
			lvds_clk);
	}
	return ret;
}

static void sn65dsi83_mode_set(struct sn65dsi83 *dsi83,
			     struct drm_display_mode *mode,
			     struct drm_display_mode *adj_mode)
{
	unsigned int h_back_porch, h_front_porch, hsync_len;
	unsigned int v_back_porch, v_front_porch, vsync_len;
	unsigned int hdisplay, vdisplay;
	unsigned long pclk;


	h_back_porch = adj_mode->crtc_hsync_start - adj_mode->crtc_hdisplay;
	v_back_porch = adj_mode->crtc_vsync_start - adj_mode->crtc_vdisplay;

	h_front_porch = adj_mode->crtc_hsync_start - adj_mode->crtc_hdisplay;
	v_front_porch = adj_mode->crtc_vsync_start - adj_mode->crtc_vdisplay;

	hsync_len = adj_mode->crtc_hsync_end - adj_mode->crtc_hsync_start;
	vsync_len = adj_mode->crtc_vsync_end - adj_mode->crtc_vsync_start;

	hdisplay = adj_mode->crtc_hdisplay;
	vdisplay = adj_mode->crtc_vdisplay;

	pclk = mode->crtc_htotal * mode->crtc_vtotal * drm_mode_vrefresh(mode);

	drm_mode_copy(&dsi83->curr_mode, adj_mode);
}

static int sn65dsi83_get_modes(struct drm_connector *connector)
{
	struct sn65dsi83 *dsi83 = connector_to_sn65dsi83(connector);
	struct drm_display_mode *mode;
	int num_modes = 0;

	if (dsi83->panel && dsi83->panel->funcs &&
	    dsi83->panel->funcs->get_modes) {
		struct drm_display_info *di = &connector->display_info;

		num_modes = dsi83->panel->funcs->get_modes(dsi83->panel);
		if (!dsi83->bus_format && di->num_bus_formats)
			dsi83->bus_format = di->bus_formats[0];
		if (num_modes > 0)
			return num_modes;
	}

	if (dsi83->edid) {
		drm_mode_connector_update_edid_property(connector,
							dsi83->edid);
		num_modes = drm_add_edid_modes(connector, dsi83->edid);
	}

	if (dsi83->mode_valid) {
		mode = drm_mode_create(connector->dev);
		if (!mode)
			return 0;
		drm_mode_copy(mode, &dsi83->curr_mode);
		mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, mode);
	}
	return num_modes;
}

static int sn65dsi83_chip_init(struct sn65dsi83 *dsi83)
{

	DSI83_UPDATE_BITS(LVDSCFG0, 0x3 << 3,
			(4 - dsi83->num_dsi_lanes) << 3);

	if (dsi83->mode_valid) {
		unsigned int h_back_porch, h_front_porch, hsync_len;
		unsigned int v_back_porch, v_front_porch, vsync_len;
		unsigned int hdisplay, vdisplay;
		unsigned long pclk;
		struct drm_display_mode *mode = &dsi83->curr_mode;
		int lvds_format;

		hdisplay = mode->crtc_hdisplay;
		vdisplay = mode->crtc_vdisplay;

		hsync_len = mode->crtc_hsync_end - mode->crtc_hsync_start;
		vsync_len = mode->crtc_vsync_end - mode->crtc_vsync_start;

		h_front_porch = mode->crtc_hsync_start - hdisplay;
		v_front_porch = mode->crtc_vsync_start - vdisplay;

		h_back_porch = mode->htotal - hsync_len - h_front_porch - hdisplay;
		v_back_porch = mode->vtotal - vsync_len - v_front_porch - vdisplay;

		pclk = mode->crtc_htotal * mode->crtc_vtotal * drm_mode_vrefresh(mode);

		DSI83_WRITE(HSYNC_WIDTH_MSB, hsync_len >> 8);
		DSI83_WRITE(HSYNC_WIDTH_LSB, hsync_len & 0xff);

		DSI83_WRITE(VSYNC_WIDTH_MSB, vsync_len >> 8);
		DSI83_WRITE(VSYNC_WIDTH_LSB, vsync_len & 0xff);

		DSI83_WRITE(LINE_LENGTH_MSB, hdisplay >> 8);
		DSI83_WRITE(LINE_LENGTH_LSB, hdisplay & 0xff);

		DSI83_WRITE(VERT_SIZE_MSB, vdisplay >> 8);
		DSI83_WRITE(VERT_SIZE_LSB, vdisplay & 0xff);

		DSI83_WRITE(H_BACK_PORCH, h_back_porch);
		DSI83_WRITE(H_FRONT_PORCH, h_front_porch);

		DSI83_WRITE(V_BACK_PORCH, v_back_porch);
		DSI83_WRITE(V_FRONT_PORCH, v_front_porch);

		DSI83_WRITE(SYNC_DELAY_LSB, SYNC_DELAY % 255);
		DSI83_WRITE(SYNC_DELAY_MSB, SYNC_DELAY / 255);

		switch (dsi83->bus_format) {
		case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
			lvds_format = BIT(1);
			break;
		case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
			lvds_format = BIT(3) | BIT(1);
			break;
		case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
			lvds_format = BIT(3);
			break;
		default:
			return -EINVAL;
		}
		DSI83_UPDATE_BITS(LVDSCFG4, BIT(3) | BIT(1), lvds_format);
		if (dsi83->client->irq)
			DSI83_WRITE(IRQCTRL, IRQCTRL_IRQ_EN);
	}
	return 0;
}

/* Connector funcs */
static int sn65dsi83_connector_get_modes(struct drm_connector *connector)
{
	return sn65dsi83_get_modes(connector);
}

static enum drm_mode_status
sn65dsi83_mode_valid(struct sn65dsi83 *dsi83, const struct drm_display_mode *mode)
{
	if (mode->clock > 154000)
		return MODE_CLOCK_HIGH;
	if (mode->clock < 25000)
		return MODE_CLOCK_LOW;
	if (mode->hdisplay < 0 || mode->hdisplay >= (1 << 12))
		return MODE_H_ILLEGAL;
	if (mode->vdisplay < 0 || mode->vdisplay >= (1 << 12))
		return MODE_V_ILLEGAL;

	return MODE_OK;
}

static struct drm_encoder *
sn65dsi83_connector_best_encoder(struct drm_connector *connector)
{
	struct sn65dsi83 *dsi83 = connector_to_sn65dsi83(connector);
	return dsi83->bridge.encoder;
}

static enum drm_mode_status
sn65dsi83_connector_mode_valid(struct drm_connector *connector,
			     struct drm_display_mode *mode)
{
	struct sn65dsi83 *dsi83 = connector_to_sn65dsi83(connector);

	DRM_DEBUG_DRIVER("Validating mode '%s'\n",
		mode->name);
	return sn65dsi83_mode_valid(dsi83, mode);
}

static struct drm_connector_helper_funcs sn65dsi83_connector_helper_funcs = {
	.get_modes = sn65dsi83_connector_get_modes,
	.best_encoder = sn65dsi83_connector_best_encoder,
	.mode_valid = sn65dsi83_connector_mode_valid,
};

static enum drm_connector_status
sn65dsi83_connector_detect(struct drm_connector *connector, bool force)
{
	struct sn65dsi83 *dsi83 = connector_to_sn65dsi83(connector);

	return sn65dsi83_detect(dsi83, connector);
}

static struct drm_connector_funcs sn65dsi83_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = sn65dsi83_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int sn65dsi83_attach_dsi(struct sn65dsi83 *dsi83)
{
	struct device *dev = &dsi83->client->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret;

	host = of_find_mipi_dsi_host_by_node(dsi83->host_node);
	if (!host) {
		dev_err(dev, "failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_new_dummy(host, 0);
	if (IS_ERR(dsi)) {
		dev_err(dev, "failed to create dummy dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_device;
	}

	dsi83->dsi = dsi;

	dsi->lanes = dsi83->num_dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "failed to attach dsi to host\n");
		goto err_dsi_attach;
	}

	return 0;

err_dsi_attach:
	mipi_dsi_unregister_device(dsi);
err_dsi_device:
	return ret;
}

static void sn65dsi83_detach_dsi(struct sn65dsi83 *dsi83)
{
	mipi_dsi_detach(dsi83->dsi);
	mipi_dsi_unregister_device(dsi83->dsi);
	of_node_put(dsi83->host_node);
}

enum {
	LVDS_BIT_MAP_SPWG,
	LVDS_BIT_MAP_JEIDA
};

static const struct imx_ldb_bit_mapping {
	u32 bus_format;
	u32 datawidth;
	const char *const mapping;
} imx_ldb_bit_mappings[] = {
	{ MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,  18, "spwg" },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,  24, "spwg" },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA, 24, "jeida" },
};

static int sn65dsi83_of_get_bus_format(struct device *dev, struct device_node *np)
{
	const char *bm = NULL;
	u32 datawidth = 0;
	int ret, i;
	struct device_node *disp_timings;

	disp_timings = of_get_child_by_name(np, "display-timings");
	if (disp_timings) {
		struct device_node *native_mode;

		native_mode = of_parse_phandle(disp_timings, "native-mode", 0);
		if (!native_mode)
			native_mode = of_get_next_child(disp_timings, NULL);
		if (of_property_read_string(native_mode, "lvds-data-mapping",
						&bm) == 0)
			DRM_DEBUG_DRIVER("Using data-mapping '%s' from node: '%s'\n",
				bm, of_node_full_name(native_mode));
		if (of_property_read_u32(native_mode, "lvds-data-width",
						&datawidth) == 0)
			DRM_DEBUG_DRIVER("Using data-width %u from node: '%s'\n",
				datawidth, of_node_full_name(native_mode));

	}
	if (bm == NULL) {
		ret = of_property_read_string(np, "lvds-data-mapping", &bm);
		if (ret < 0)
			dev_err(dev, "lvds-data-mapping missing in DT\n");
		else
			DRM_DEBUG_DRIVER("Using data-mapping '%s' from node: '%s'\n",
				bm, of_node_full_name(np));
	}

	if (datawidth == 0) {
		ret = of_property_read_u32(np, "lvds-data-width", &datawidth);
		if (ret < 0)
			dev_err(dev, "lvds-data-width missing in DT\n");
		else
			DRM_DEBUG_DRIVER("Using data-width %u from node: '%s'\n",
				datawidth, of_node_full_name(np));
	}
	if (datawidth == 0 || bm == NULL)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(imx_ldb_bit_mappings); i++) {
		if (strcasecmp(bm, imx_ldb_bit_mappings[i].mapping) == 0 &&
		    datawidth == imx_ldb_bit_mappings[i].datawidth)
			return imx_ldb_bit_mappings[i].bus_format;
	}

	dev_err(dev, "invalid data mapping: %u-bit \"%s\"\n", datawidth, bm);

	return -ENOENT;
}

/*
  mdss                                 mdp
  port[0]               dsi0_in   <->  port[0] mdp5_intf1_out
  port[1] [data-lanes]  dsi0_out  <-+
                        |
  dsi83                 |
  port[0] * dsi83_in  <-+
  port[1]   dsi83_out <->  lvds-out
*/

int sn65dsi83_parse_dt(struct device_node *np, struct sn65dsi83 *dsi83)
{
	int ret;
	u32 num_lanes;
	struct device *dev = &dsi83->client->dev;
	struct device_node *endpoint;
	struct device_node *dsi0;

	endpoint = of_graph_get_next_endpoint(np, NULL);
	if (!endpoint)
		return -ENODEV;

	dsi83->host_node = of_graph_get_remote_port_parent(endpoint);
	if (!dsi83->host_node) {
		of_node_put(endpoint);
		return -ENODEV;
	}

	dsi0 = of_graph_get_remote_port(endpoint);
	if (!dsi0) {
		dev_err(dev, "dsi0-out endpoint not found in %s\n",
			of_node_full_name(endpoint));
		return -EINVAL;
	}

	endpoint = of_find_node_by_name(dsi0, "endpoint");
	if (!endpoint) {
		dev_err(dev, "No 'endpoint' node found in '%s'\n",
			of_node_full_name(dsi0));
		return -EINVAL;
	}

	if (!of_find_property(endpoint, "data-lanes", &num_lanes)) {
		dev_err(dev, "No 'data-lanes' property found in %s\n",
			of_node_full_name(endpoint));
		return -EINVAL;
	}
	num_lanes /= sizeof(u32);
	if (num_lanes < 1 || num_lanes > 4) {
		dev_err(dev, "Invalid number of data lanes %u from property '%s/data-lanes'\n",
			num_lanes, of_node_full_name(endpoint));
		return -EINVAL;
	}
	ret = of_get_drm_display_mode(np, &dsi83->curr_mode, OF_USE_NATIVE_MODE);
	dsi83->mode_valid = ret == 0;

	dsi83->bus_format = sn65dsi83_of_get_bus_format(dev, np);
	if (dsi83->bus_format == -EINVAL) {
		/*
		 * If no bus format was specified in the device tree,
		 * we can still get it from the connected panel later.
		 */
		if (dsi83->panel && dsi83->panel->funcs &&
			dsi83->panel->funcs->get_modes)
			dsi83->bus_format = 0;
	}
	if (dsi83->bus_format < 0) {
		dev_err(dev, "could not determine data mapping: %d\n",
			dsi83->bus_format);
		return dsi83->bus_format;
	}
	dsi83->num_dsi_lanes = num_lanes;

	of_node_put(endpoint);
	return 0;
}

static struct sn65dsi83 *bridge_to_sn65dsi83(struct drm_bridge *bridge)
{
	return container_of(bridge, struct sn65dsi83, bridge);
}

static void sn65dsi83_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct sn65dsi83 *dsi83 = bridge_to_sn65dsi83(bridge);

	sn65dsi83_power_on(dsi83);
}

static void sn65dsi83_bridge_enable(struct drm_bridge *bridge)
{
	struct sn65dsi83 *dsi83 = bridge_to_sn65dsi83(bridge);
	int ret;

	gpiod_set_value(dsi83->enable_gpio, 1);

	ret = sn65dsi83_chip_init(dsi83);
	if (ret)
		dev_err(&dsi83->client->dev,
			"Failed to initialize chip: %d\n",
			ret);
	ret = sn65dsi83_pll_init(dsi83);
	if (ret)
		dev_err(&dsi83->client->dev, "Failed to initialize PLL: %d\n",
			ret);
}

static void sn65dsi83_bridge_post_disable(struct drm_bridge *bridge)
{
	struct sn65dsi83 *dsi83 = bridge_to_sn65dsi83(bridge);

	gpiod_set_value(dsi83->enable_gpio, 0);
	sn65dsi83_power_off(dsi83);
}

static void sn65dsi83_bridge_disable(struct drm_bridge *bridge)
{
}

static void sn65dsi83_bridge_mode_set(struct drm_bridge *bridge,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adj_mode)
{
	struct sn65dsi83 *dsi83 = bridge_to_sn65dsi83(bridge);

	sn65dsi83_mode_set(dsi83, mode, adj_mode);
}

static int sn65dsi83_bridge_attach(struct drm_bridge *bridge)
{
	struct sn65dsi83 *dsi83 = bridge_to_sn65dsi83(bridge);
	int ret;

	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found");
		return -ENODEV;
	}

	ret = drm_connector_init(bridge->dev, &dsi83->connector,
				 &sn65dsi83_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}
	drm_connector_helper_add(&dsi83->connector,
				 &sn65dsi83_connector_helper_funcs);
	drm_mode_connector_attach_encoder(&dsi83->connector, bridge->encoder);
	drm_connector_register(&dsi83->connector);

	if (dsi83->panel)
		drm_panel_attach(dsi83->panel, &dsi83->connector);

	ret = sn65dsi83_attach_dsi(dsi83);
	if (ret)
		return ret;

	drm_helper_hpd_irq_event(dsi83->connector.dev);
	return 0;
}

static struct drm_bridge_funcs sn65dsi83_bridge_funcs = {
	.pre_enable = sn65dsi83_bridge_pre_enable,
	.enable = sn65dsi83_bridge_enable,
	.disable = sn65dsi83_bridge_disable,
	.post_disable = sn65dsi83_bridge_post_disable,
	.mode_set = sn65dsi83_bridge_mode_set,
	.attach = sn65dsi83_bridge_attach,
};

/* -----------------------------------------------------------------------------
 * Encoder operations
 */
static int sn65dsi83_encoder_get_modes(struct drm_encoder *encoder,
			     struct drm_connector *connector)
{
	return sn65dsi83_get_modes(connector);
}

static void sn65dsi83_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct sn65dsi83 *dsi83 = encoder_to_sn65dsi83(encoder);

	if (mode == DRM_MODE_DPMS_ON)
		sn65dsi83_power_on(dsi83);
	else
		sn65dsi83_power_off(dsi83);
}

static enum drm_connector_status
sn65dsi83_encoder_detect(struct drm_encoder *encoder,
		       struct drm_connector *connector)
{
	struct sn65dsi83 *dsi83 = encoder_to_sn65dsi83(encoder);

	return sn65dsi83_detect(dsi83, connector);
}

static int sn65dsi83_encoder_mode_valid(struct drm_encoder *encoder,
				      struct drm_display_mode *mode)
{
	struct sn65dsi83 *dsi83 = encoder_to_sn65dsi83(encoder);

	return sn65dsi83_mode_valid(dsi83, mode);
}

static void sn65dsi83_encoder_mode_set(struct drm_encoder *encoder,
				     struct drm_display_mode *mode,
				     struct drm_display_mode *adj_mode)
{
	struct sn65dsi83 *dsi83 = encoder_to_sn65dsi83(encoder);

	sn65dsi83_mode_set(dsi83, mode, adj_mode);
}

static struct drm_encoder_slave_funcs sn65dsi83_encoder_funcs = {
	.dpms = sn65dsi83_encoder_dpms,
	.mode_valid = sn65dsi83_encoder_mode_valid,
	.mode_set = sn65dsi83_encoder_mode_set,
	.detect = sn65dsi83_encoder_detect,
	.get_modes = sn65dsi83_encoder_get_modes,
};

static int sn65dsi83_encoder_init(struct i2c_client *i2c, struct drm_device *dev,
				struct drm_encoder_slave *encoder)
{

	struct sn65dsi83 *dsi83 = i2c_get_clientdata(i2c);

	encoder->slave_priv = dsi83;
	encoder->slave_funcs = &sn65dsi83_encoder_funcs;

	dsi83->encoder = &encoder->base;

	return 0;
}

static int sn65dsi83_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sn65dsi83 *dsi83;
	struct device *dev = &client->dev;
	int ret;
	u8 chiprev[9];

	if (!dev->of_node)
		return -EINVAL;

	dsi83 = devm_kzalloc(dev, sizeof(*dsi83), GFP_KERNEL);
	if (!dsi83)
		return -ENOMEM;

	dsi83->status = connector_status_disconnected;
	dsi83->client = client;

	ret = sn65dsi83_parse_dt(dev->of_node, dsi83);
	if (ret)
		return ret;

	dsi83->refclk = devm_clk_get(dev, NULL);
	if (IS_ERR(dsi83->refclk)) {
		if (PTR_ERR(dsi83->refclk) == -EPROBE_DEFER)
			return PTR_ERR(dsi83->refclk);
		dsi83->refclk = NULL;
	}

	dsi83->vcc_reg = devm_regulator_get_optional(dev, "vcc");
	if (IS_ERR(dsi83->vcc_reg)) {
		ret = PTR_ERR(dsi83->vcc_reg);
		dev_err(dev, "Failed to get VCC regulator: %d\n", ret);
		return ret;
	}
	if (dsi83->vcc_reg) {
		ret = regulator_enable(dsi83->vcc_reg);
		if (ret) {
			dev_err(dev, "Failed to enable VCC regulator: %d\n", ret);
			return ret;
		}
	}

	dsi83->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						GPIOD_OUT_LOW);
	if (IS_ERR(dsi83->enable_gpio))
		return PTR_ERR(dsi83->enable_gpio);

	dsi83->regmap = devm_regmap_init_i2c(client, &sn65dsi83_regmap_config);
	if (IS_ERR(dsi83->regmap))
		return PTR_ERR(dsi83->regmap);

	sn65dsi83_power_on(dsi83);
	ret = regmap_bulk_read(dsi83->regmap, SN65DSI83_REG_CHIPREV(0),
			&chiprev, sizeof(chiprev));
	if (ret) {
		dev_err(dev, "Failed to read CHIPREV register\n");
		return ret;
	}
	print_hex_dump_bytes("CHIPREV: ", DUMP_PREFIX_NONE, chiprev,
			sizeof(chiprev));
	dev_info(dev, "Rev. %u\n", chiprev[8]);

	if (client->irq) {
		init_waitqueue_head(&dsi83->wq);

		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sn65dsi83_irq_handler,
						IRQF_ONESHOT, dev_name(dev),
						dsi83);
		if (ret)
			return ret;
	}

	sn65dsi83_power_off(dsi83);

	i2c_set_clientdata(client, dsi83);

	dsi83->bridge.funcs = &sn65dsi83_bridge_funcs;
	dsi83->bridge.of_node = dev->of_node;

	ret = drm_bridge_add(&dsi83->bridge);
	if (ret)
		dev_err(dev, "failed to add sn65dsi83 bridge\n");

	return ret;
}

static int sn65dsi83_remove(struct i2c_client *client)
{
	struct sn65dsi83 *dsi83 = i2c_get_clientdata(client);

	if (dsi83->dsi)
		sn65dsi83_detach_dsi(dsi83);

	drm_bridge_remove(&dsi83->bridge);
	return 0;
}

static const struct i2c_device_id sn65dsi83_i2c_ids[] = {
	{ "sn65dsi83", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sn65dsi83_i2c_ids);

static const struct of_device_id sn65dsi83_of_ids[] = {
	{ .compatible = "ti,sn65dsi83", },
	{ }
};
MODULE_DEVICE_TABLE(of, sn65dsi83_of_ids);

static struct drm_i2c_encoder_driver sn65dsi83_driver = {
	.i2c_driver = {
		.driver = {
			.name = "sn65dsi83",
			.of_match_table = sn65dsi83_of_ids,
		},
		.id_table = sn65dsi83_i2c_ids,
		.probe = sn65dsi83_probe,
		.remove = sn65dsi83_remove,
	},
	.encoder_init = sn65dsi83_encoder_init,
};

static int __init sn65dsi83_init(void)
{
	return drm_i2c_encoder_register(THIS_MODULE, &sn65dsi83_driver);
}
module_init(sn65dsi83_init);

static void __exit sn65dsi83_exit(void)
{
	drm_i2c_encoder_unregister(&sn65dsi83_driver);
}
module_exit(sn65dsi83_exit);

MODULE_AUTHOR("Lothar Waßmann <LW@KARO-electronics.de>");
MODULE_DESCRIPTION("SN65DSI83 DSI to LVDS converter driver");
MODULE_LICENSE("GPL");
