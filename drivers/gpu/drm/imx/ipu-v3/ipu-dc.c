/*
 * Copyright (c) 2010 Sascha Hauer <s.hauer@pengutronix.de>
 * Copyright (C) 2005-2009 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <drm/imx-ipu-v3.h>

#include "ipu-prv.h"

#define ASYNC_SER_WAVE 6

#define DC_DISP_ID_SERIAL	2
#define DC_DISP_ID_ASYNC	3

#define DC_MAP_CONF_PTR(n)	(0x0108 + ((n) & ~0x1) * 2)
#define DC_MAP_CONF_VAL(n)	(0x0144 + ((n) & ~0x1) * 2)

#define DC_EVT_NF		0
#define DC_EVT_NL		1
#define DC_EVT_EOF		2
#define DC_EVT_NFIELD		3
#define DC_EVT_EOL		4
#define DC_EVT_EOFIELD		5
#define DC_EVT_NEW_ADDR		6
#define DC_EVT_NEW_CHAN		7
#define DC_EVT_NEW_DATA		8

#define DC_EVT_NEW_ADDR_W_0	0
#define DC_EVT_NEW_ADDR_W_1	1
#define DC_EVT_NEW_CHAN_W_0	2
#define DC_EVT_NEW_CHAN_W_1	3
#define DC_EVT_NEW_DATA_W_0	4
#define DC_EVT_NEW_DATA_W_1	5
#define DC_EVT_NEW_ADDR_R_0	6
#define DC_EVT_NEW_ADDR_R_1	7
#define DC_EVT_NEW_CHAN_R_0	8
#define DC_EVT_NEW_CHAN_R_1	9
#define DC_EVT_NEW_DATA_R_0	10
#define DC_EVT_NEW_DATA_R_1	11

#define DC_WR_CH_CONF		0x0
#define DC_WR_CH_ADDR		0x4
#define DC_RL_CH(evt)		(8 + ((evt) & ~0x1) * 2)

#define DC_GEN			0x00d4
#define DC_DISP_CONF1(disp)	(0x00d8 + (disp) * 4)
#define DC_DISP_CONF2(disp)	(0x00e8 + (disp) * 4)
#define DC_STAT			0x01c8

#define WROD(lf)		(0x18 | (lf << 1))

#define DC_WR_CH_CONF_FIELD_MODE		(1 << 9)
#define DC_WR_CH_CONF_PROG_TYPE_OFFSET		5
#define DC_WR_CH_CONF_PROG_TYPE_MASK		(7 << 5)
#define DC_WR_CH_CONF_PROG_DI_ID		(1 << 2)
#define DC_WR_CH_CONF_PROG_DISP_ID_OFFSET	3
#define DC_WR_CH_CONF_PROG_DISP_ID_MASK		(3 << 3)

struct ipu_dc_priv;

struct ipu_dc {
	unsigned int		di; /* The display interface number assigned to this dc channel */
	void __iomem		*base;
	struct ipu_dc_priv	*priv;
	int			chno;
	bool			in_use;
};

struct ipu_dc_priv {
	void __iomem		*dc_reg;
	void __iomem		*dc_tmpl_reg;
	struct ipu_soc		*ipu;
	struct device		*dev;
	struct ipu_dc		channels[10];
	struct mutex		mutex;
};

static void ipu_dc_link_event(struct ipu_dc *dc, int event, int addr, int priority)
{
	u32 reg;

	reg = readl(dc->base + DC_RL_CH(event));
	reg &= ~(0xffff << (16 * (event & 0x1)));
	reg |= ((addr << 8) | priority) << (16 * (event & 0x1));
	writel(reg, dc->base + DC_RL_CH(event));
}

static void ipu_dc_write_tmpl(struct ipu_dc *dc, int word, u32 opcode, u32 operand,
		int map, int wave, int glue, int sync)
{
	struct ipu_dc_priv *priv = dc->priv;
	u32 reg;
	int stop = 1;

	reg = sync;
	reg |= glue << 4;
	reg |= ++wave << 11;
	reg |= ++map << 15;
	reg |= (operand << 20) & 0xfff00000;
	writel(reg, priv->dc_tmpl_reg + word * 8);

	reg = operand >> 12;
	reg |= opcode << 4;
	reg |= stop << 9;
	writel(reg, priv->dc_tmpl_reg + word * 8 + 4);
}

static int ipu_pixfmt_to_map(u32 fmt)
{
	switch (fmt) {
	case IPU_PIX_FMT_GENERIC:
	case IPU_PIX_FMT_RGB24:
		return 0;
	case IPU_PIX_FMT_RGB666:
		return 1;
	case IPU_PIX_FMT_YUV444:
		return 2;
	case IPU_PIX_FMT_RGB565:
		return 3;
	case IPU_PIX_FMT_LVDS666:
		return 4;
	case IPU_PIX_FMT_GBR24:
		return 13;
	}

	return -EINVAL;
}

#define SYNC_WAVE 0

int ipu_dc_init_sync(struct ipu_dc *dc, int di, bool interlaced,
		u32 pixel_fmt, u32 width)
{
	struct ipu_dc_priv *priv = dc->priv;
	u32 reg = 0, map;

	dc->di = di;

	map = ipu_pixfmt_to_map(pixel_fmt);
	if (map < 0) {
		dev_dbg(priv->dev, "IPU_DISP: No MAP\n");
		return -EINVAL;
	}

	ipu_get(priv->ipu);

	if (interlaced) {
		ipu_dc_link_event(dc, DC_EVT_NL, 0, 3);
		ipu_dc_link_event(dc, DC_EVT_EOL, 0, 2);
		ipu_dc_link_event(dc, DC_EVT_NEW_DATA, 0, 1);

		/* Init template microcode */
		ipu_dc_write_tmpl(dc, 0, WROD(0), 0, map, SYNC_WAVE, 0, 8);
	} else {
		if (di) {
			ipu_dc_link_event(dc, DC_EVT_NL, 2, 3);
			ipu_dc_link_event(dc, DC_EVT_EOL, 3, 2);
			ipu_dc_link_event(dc, DC_EVT_NEW_DATA, 4, 1);
			/* Init template microcode */
			ipu_dc_write_tmpl(dc, 2, WROD(0), 0, map, SYNC_WAVE, 8, 5);
			ipu_dc_write_tmpl(dc, 3, WROD(0), 0, map, SYNC_WAVE, 4, 5);
			ipu_dc_write_tmpl(dc, 4, WROD(0), 0, map, SYNC_WAVE, 0, 5);
		} else {
			ipu_dc_link_event(dc, DC_EVT_NL, 5, 3);
			ipu_dc_link_event(dc, DC_EVT_EOL, 6, 2);
			ipu_dc_link_event(dc, DC_EVT_NEW_DATA, 7, 1);
			/* Init template microcode */
			ipu_dc_write_tmpl(dc, 5, WROD(0), 0, map, SYNC_WAVE, 8, 5);
			ipu_dc_write_tmpl(dc, 6, WROD(0), 0, map, SYNC_WAVE, 4, 5);
			ipu_dc_write_tmpl(dc, 7, WROD(0), 0, map, SYNC_WAVE, 0, 5);
		}
	}
	ipu_dc_link_event(dc, DC_EVT_NF, 0, 0);
	ipu_dc_link_event(dc, DC_EVT_NFIELD, 0, 0);
	ipu_dc_link_event(dc, DC_EVT_EOF, 0, 0);
	ipu_dc_link_event(dc, DC_EVT_EOFIELD, 0, 0);
	ipu_dc_link_event(dc, DC_EVT_NEW_CHAN, 0, 0);
	ipu_dc_link_event(dc, DC_EVT_NEW_ADDR, 0, 0);

	reg = 0x2;
	reg |= di << DC_WR_CH_CONF_PROG_DISP_ID_OFFSET;
	reg |= di << 2;
	if (interlaced)
		reg |= DC_WR_CH_CONF_FIELD_MODE;

	writel(reg, dc->base + DC_WR_CH_CONF);
	writel(0x00000000, dc->base + DC_WR_CH_ADDR);
	writel(0x00000084, priv->dc_reg + DC_GEN);
	writel(width, priv->dc_reg + DC_DISP_CONF2(di));

	ipu_module_enable(priv->ipu, IPU_CONF_DC_EN);
	ipu_put(priv->ipu);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_dc_init_sync);

void ipu_dc_init_async(struct ipu_dc *dc, int di, bool interlaced)
{
	struct ipu_dc_priv *priv = dc->priv;
	u32 reg = 0;

	ipu_get(priv->ipu);

	dc->di = di;

	ipu_dc_link_event(dc, DC_EVT_NEW_DATA_W_0, 0x64, 1);
	ipu_dc_link_event(dc, DC_EVT_NEW_DATA_W_1, 0x64, 1);

	reg = 0x3;
	reg |= DC_DISP_ID_SERIAL << DC_WR_CH_CONF_PROG_DISP_ID_OFFSET;
	writel(reg, dc->base + DC_WR_CH_CONF);

	writel(0x00000000, dc->base + DC_WR_CH_ADDR);

	writel(0x00000084, priv->dc_reg + DC_GEN);

	ipu_module_enable(priv->ipu, IPU_CONF_DC_EN);
	ipu_put(priv->ipu);
}
EXPORT_SYMBOL_GPL(ipu_dc_init_async);

void ipu_dc_enable_channel(struct ipu_dc *dc)
{
	struct ipu_dc_priv *priv = dc->priv;
	int di;
	u32 reg;

	ipu_get(priv->ipu);

	di = dc->di;

	reg = readl(dc->base + DC_WR_CH_CONF);
	reg |= 4 << DC_WR_CH_CONF_PROG_TYPE_OFFSET;
	writel(reg, dc->base + DC_WR_CH_CONF);

	ipu_module_enable(priv->ipu, IPU_CONF_DC_EN);
	ipu_put(priv->ipu);
}
EXPORT_SYMBOL_GPL(ipu_dc_enable_channel);

void ipu_dc_disable_channel(struct ipu_dc *dc)
{
	struct ipu_dc_priv *priv = dc->priv;
	u32 reg;
	int irq = 0, timeout = 50;

	if (dc->chno == 1) {
		irq = IPU_IRQ_DC_FC_1;
	} else if (dc->chno == 5) {
		irq = IPU_IRQ_DP_SF_END;
	} else {
		return;
	}

	ipu_get(priv->ipu);

	/* should wait for the interrupt here */
	mdelay(50);

	/* Wait for DC triple buffer to empty */
	if (dc->di == 0)
		while ((readl(priv->dc_reg + DC_STAT) & 0x00000002)
			!= 0x00000002) {
			msleep(2);
			timeout -= 2;
			if (timeout <= 0)
				break;
		}
	else if (dc->di == 1)
		while ((readl(priv->dc_reg + DC_STAT) & 0x00000020)
			!= 0x00000020) {
			msleep(2);
			timeout -= 2;
			if (timeout <= 0)
				break;
	}

	reg = readl(dc->base + DC_WR_CH_CONF);
	reg &= ~DC_WR_CH_CONF_PROG_TYPE_MASK;
	writel(reg, dc->base + DC_WR_CH_CONF);

	ipu_module_disable(priv->ipu, IPU_CONF_DC_EN);
	ipu_put(priv->ipu);
}
EXPORT_SYMBOL_GPL(ipu_dc_disable_channel);

static void ipu_dc_map_link(struct ipu_dc_priv *priv, int current_map,
		int base_map_0, int buf_num_0,
		int base_map_1, int buf_num_1,
		int base_map_2, int buf_num_2)
{
	int ptr_0 = base_map_0 * 3 + buf_num_0;
	int ptr_1 = base_map_1 * 3 + buf_num_1;
	int ptr_2 = base_map_2 * 3 + buf_num_2;
	int ptr;
	u32 reg;
	ptr = (ptr_2 << 10) +  (ptr_1 << 5) + ptr_0;

	reg = readl(priv->dc_reg + DC_MAP_CONF_PTR(current_map));
	reg &= ~(0x1F << ((16 * (current_map & 0x1))));
	reg |= ptr << ((16 * (current_map & 0x1)));
	writel(reg, priv->dc_reg + DC_MAP_CONF_PTR(current_map));
}

static void ipu_dc_map_config(struct ipu_dc_priv *priv, int map,
		int byte_num, int offset, int mask)
{
	int ptr = map * 3 + byte_num;
	u32 reg;

	reg = readl(priv->dc_reg + DC_MAP_CONF_VAL(ptr));
	reg &= ~(0xffff << (16 * (ptr & 0x1)));
	reg |= ((offset << 8) | mask) << (16 * (ptr & 0x1));
	writel(reg, priv->dc_reg + DC_MAP_CONF_VAL(ptr));

	reg = readl(priv->dc_reg + DC_MAP_CONF_PTR(map));
	reg &= ~(0x1f << ((16 * (map & 0x1)) + (5 * byte_num)));
	reg |= ptr << ((16 * (map & 0x1)) + (5 * byte_num));
	writel(reg, priv->dc_reg + DC_MAP_CONF_PTR(map));
}

static void ipu_dc_map_clear(struct ipu_dc_priv *priv, int map)
{
	u32 reg = readl(priv->dc_reg + DC_MAP_CONF_PTR(map));
	writel(reg & ~(0xffff << (16 * (map & 0x1))),
		     priv->dc_reg + DC_MAP_CONF_PTR(map));
}

struct ipu_dc *ipu_dc_get(struct ipu_soc *ipu, int channel)
{
	struct ipu_dc_priv *priv = ipu->dc_priv;
	struct ipu_dc *dc;

	if (channel > 9)
		return ERR_PTR(-EINVAL);

	dc = &priv->channels[channel];

	mutex_lock(&priv->mutex);

	if (dc->in_use) {
		mutex_unlock(&priv->mutex);
		return ERR_PTR(-EBUSY);
	}

	dc->in_use = 1;

	mutex_unlock(&priv->mutex);

	return dc;
}
EXPORT_SYMBOL_GPL(ipu_dc_get);

void ipu_dc_put(struct ipu_dc *dc)
{
	struct ipu_dc_priv *priv = dc->priv;

	mutex_lock(&priv->mutex);
	dc->in_use = 0;
	mutex_unlock(&priv->mutex);
}
EXPORT_SYMBOL_GPL(ipu_dc_put);

int ipu_dc_init(struct ipu_soc *ipu, struct device *dev,
		unsigned long base, unsigned long template_base)
{
	struct ipu_dc_priv *priv;
	static int channel_offsets[] = { 0, 0x1c, 0x38, 0x54, 0x58, 0x5c,
		0x78, 0, 0x94, 0xb4};
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->mutex);

	priv->dev = dev;
	priv->ipu = ipu;
	priv->dc_reg = devm_ioremap(dev, base, PAGE_SIZE);
	priv->dc_tmpl_reg = devm_ioremap(dev, template_base, PAGE_SIZE);
	if (!priv->dc_reg || !priv->dc_tmpl_reg)
		return -ENOMEM;

	for (i = 0; i < 10; i++) {
		priv->channels[i].chno = i;
		priv->channels[i].priv = priv;
		priv->channels[i].base = priv->dc_reg + channel_offsets[i];
	}

	ipu->dc_priv = priv;

	dev_dbg(dev, "DC base: 0x%08lx template base: 0x%08lx\n",
			base, template_base);

	/* IPU_PIX_FMT_RGB24 */
	ipu_dc_map_clear(priv, 0);
	ipu_dc_map_config(priv, 0, 0, 7, 0xff);
	ipu_dc_map_config(priv, 0, 1, 15, 0xff);
	ipu_dc_map_config(priv, 0, 2, 23, 0xff);

	/* IPU_PIX_FMT_RGB666 */
	ipu_dc_map_clear(priv, 1);
	ipu_dc_map_config(priv, 1, 0, 5, 0xfc);
	ipu_dc_map_config(priv, 1, 1, 11, 0xfc);
	ipu_dc_map_config(priv, 1, 2, 17, 0xfc);

	/* IPU_PIX_FMT_YUV444 */
	ipu_dc_map_clear(priv, 2);
	ipu_dc_map_config(priv, 2, 0, 15, 0xff);
	ipu_dc_map_config(priv, 2, 1, 23, 0xff);
	ipu_dc_map_config(priv, 2, 2, 7, 0xff);

	/* IPU_PIX_FMT_RGB565 */
	ipu_dc_map_clear(priv, 3);
	ipu_dc_map_config(priv, 3, 0, 4, 0xf8);
	ipu_dc_map_config(priv, 3, 1, 10, 0xfc);
	ipu_dc_map_config(priv, 3, 2, 15, 0xf8);

	/* IPU_PIX_FMT_LVDS666 */
	ipu_dc_map_clear(priv, 4);
	ipu_dc_map_config(priv, 4, 0, 5, 0xfc);
	ipu_dc_map_config(priv, 4, 1, 13, 0xfc);
	ipu_dc_map_config(priv, 4, 2, 21, 0xfc);

	/* IPU_PIX_FMT_GBR24 */
	ipu_dc_map_clear(priv, 13);
	ipu_dc_map_link(priv, 13, 0, 2, 0, 0, 0, 1);

	return 0;
}

void ipu_dc_exit(struct ipu_soc *ipu)
{
}
