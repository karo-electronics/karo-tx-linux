/*
 * Copyright (C) 2011 Pengutronix
 * Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_runtime.h>

#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>

#include <mach/hardware.h>
#include <mach/mx23.h>
#include <mach/mx28.h>
#include <mach/usbphy.h>

#include "mxs-usbphy.h"

#define DRIVER_NAME "mxs-usbphy"

#define SET					0x4
#define CLR					0x8

#define USBPHY_PWD				0x0
#define USBPHY_CTRL				0x30
#define USBPHY_CTRL_SFTRST			BIT(31)
#define USBPHY_CTRL_CLKGATE			BIT(30)
#define USBPHY_CTRL_UTMI_SUSPENDM		BIT(29)
#define USBPHY_CTRL_HOST_FORCE_LS_SE0		BIT(28)
#define USBPHY_CTRL_RSVD3			BIT(27)
#define USBPHY_CTRL_ENAUTOSET_USBCLKS		BIT(26)
#define USBPHY_CTRL_ENAUTOCLR_USBCLKGATE	BIT(25)
#define USBPHY_CTRL_FSDLL_RST_EN		BIT(24)
#define USBPHY_CTRL_ENVBUSCHG_WKUP		BIT(23)
#define USBPHY_CTRL_ENIDCHG_WKUP		BIT(22)
#define USBPHY_CTRL_ENDPDMCHG_WKUP		BIT(21)
#define USBPHY_CTRL_ENAUTOCLR_PHY_PWD		BIT(20)
#define USBPHY_CTRL_ENAUTOCLR_CLKGATE		BIT(19)
#define USBPHY_CTRL_ENAUTO_PWRON_PLL		BIT(18)
#define USBPHY_CTRL_WAKEUP_IRQ			BIT(17)
#define USBPHY_CTRL_ENIRQWAKEUP			BIT(16)
#define USBPHY_CTRL_ENUTMILEVEL3		BIT(15)
#define USBPHY_CTRL_ENUTMILEVEL2		BIT(14)
#define USBPHY_CTRL_DATA_ON_LRADC		BIT(13)
#define USBPHY_CTRL_DEVPLUGIN_IRQ		BIT(12)
#define USBPHY_CTRL_ENIRQDEVPLUGIN		BIT(11)
#define USBPHY_CTRL_RESUME_IRQ			BIT(10)
#define USBPHY_CTRL_ENIRQRESUMEDETECT		BIT(9)
#define USBPHY_CTRL_RESUMEIRQSTICKY		BIT(8)
#define USBPHY_CTRL_ENOTGIDDETECT		BIT(7)
#define USBPHY_CTRL_RSVD1			BIT(6)
#define USBPHY_CTRL_DEVPLUGIN_POLARITY		BIT(5)
#define USBPHY_CTRL_ENDEVPLUGINDETECT		BIT(4)
#define USBPHY_CTRL_HOSTDISCONDETECT_IRQ	BIT(3)
#define USBPHY_CTRL_ENIRQHOSTDISCON		BIT(2)
#define USBPHY_CTRL_ENHOSTDISCONDETECT		BIT(1)

#define USBPHY_CTRL_DISCONNECT \
	(USBPHY_CTRL_HOSTDISCONDETECT_IRQ | USBPHY_CTRL_ENIRQHOSTDISCON)

#define USBPHY_CTRL_CONNECT \
	(USBPHY_CTRL_DEVPLUGIN_IRQ | USBPHY_CTRL_ENIRQDEVPLUGIN)


#define to_mxs_usbphy(x)	container_of(x, struct mxs_usbphy, otg)

struct mxs_usbphy {
	struct otg_transceiver otg;
	void __iomem *base;
	void __iomem *otg_base;
	unsigned flags;
	int (*set_vbus)(int on);
	struct device *dev;
	struct work_struct work;
};

int mxs_usbphy_hub_port_init(struct device *dev)
{
	struct otg_transceiver *otg;
	struct mxs_usbphy *phy;

	otg = otg_find_transceiver(dev);
	if (!otg)
		return -ENODEV;

	phy = to_mxs_usbphy(otg);

	writel(USBPHY_CTRL_DISCONNECT, phy->base + USBPHY_CTRL + CLR);
	writel(USBPHY_CTRL_ENHOSTDISCONDETECT | USBPHY_CTRL_ENIRQHOSTDISCON,
	       phy->base + USBPHY_CTRL + SET);

	return 0;
}

EXPORT_SYMBOL(mxs_usbphy_hub_port_init);

static int mxs_usbphy_enable(void __iomem *base)
{
	/* Power up the PHY */
	writel(0x0, base + USBPHY_PWD);

	/*
	 * Set precharge bit to cure overshoot problems at the
	 * start of packets
	 */
	writel(1, base + USBPHY_CTRL + SET);
	writel(USBPHY_CTRL_ENUTMILEVEL2 | USBPHY_CTRL_ENUTMILEVEL3,
	       base + USBPHY_CTRL + SET);

	return 0;
}

static int mxs_usbphy_start_host(struct otg_transceiver *otg, int on)
{
	struct mxs_usbphy *phy = to_mxs_usbphy(otg);
	struct usb_hcd *hcd;

	if (!otg->host)
		return -ENODEV;

	hcd = bus_to_hcd(otg->host);

	if (on) {
		dev_dbg(otg->dev, "starting host\n");
		mxs_usbphy_enable(phy->base);

		if (phy->set_vbus)
			phy->set_vbus(1);

		usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
	} else {
		dev_dbg(otg->dev, "stopping host\n");
		usb_remove_hcd(hcd);

		if (phy->set_vbus)
			phy->set_vbus(0);
	}

	return 0;
}

static int mxs_usbphy_set_host(struct otg_transceiver *otg,
			       struct usb_bus *host)
{
	struct mxs_usbphy *phy = to_mxs_usbphy(otg);

	otg->host = host;

	if (phy->flags & MXS_USBPHY_HOST)
		mxs_usbphy_start_host(otg, host ? 1 : 0);

	if (phy->flags & MXS_USBPHY_OTG)
		schedule_work(&phy->work);

	return 0;
}

static int mxs_usbphy_start_peripheral(struct otg_transceiver *otg, int on)
{
	struct mxs_usbphy *phy = to_mxs_usbphy(otg);

	mxs_usbphy_enable(phy->base);

	if (on) {
		dev_dbg(otg->dev, "starting peripheral\n");
		usb_gadget_vbus_connect(otg->gadget);
	} else {
		dev_dbg(otg->dev, "stopping peripheral\n");
		usb_gadget_vbus_disconnect(otg->gadget);
	}

	return 0;
}

int mxs_usbphy_set_peripheral(struct otg_transceiver *otg,
			      struct usb_gadget *gadget)
{
	int ret = 0;
	struct mxs_usbphy *phy = to_mxs_usbphy(otg);

	if (gadget)
		otg->gadget = gadget;
	if (!otg->gadget)
		return -EINVAL;

	if (phy->flags & MXS_USBPHY_DEVICE)
		ret = mxs_usbphy_start_peripheral(otg, gadget ? 1 : 0);

	if (phy->flags & MXS_USBPHY_OTG)
		schedule_work(&phy->work);

	return ret;
}

static int mxs_usbphy_set_power(struct otg_transceiver *otg, unsigned mA)
{
	return 0;
}

static void msm_otg_sm_work(struct work_struct *w)
{
	struct mxs_usbphy *phy = container_of(w, struct mxs_usbphy, work);
	struct otg_transceiver *otg = &phy->otg;
	u32 otgsc = readl(phy->otg_base + 0x1a4);
	int idpin = otgsc & (1 << 8);
	int ret;

	switch (otg->state) {
	case OTG_STATE_UNDEFINED:
		dev_dbg(otg->dev, "OTG_STATE_UNDEFINED state\n");
		if (!idpin) {
			ret = mxs_usbphy_start_host(otg, 1);
			if (ret)
				break;
			otg->state = OTG_STATE_A_HOST;
		} else {
			ret = mxs_usbphy_start_peripheral(otg, 1);
			if (ret)
				break;
			otg->state = OTG_STATE_B_PERIPHERAL;
		}
		break;
	case OTG_STATE_B_PERIPHERAL:
		dev_dbg(otg->dev, "OTG_STATE_B_PERIPHERAL state\n");
		if (!idpin) {
			mxs_usbphy_start_peripheral(otg, 0);
			ret = mxs_usbphy_start_host(otg, 1);
			if (ret)
				break;
			otg->state = OTG_STATE_A_HOST;
		}
		break;
	case OTG_STATE_A_HOST:
		dev_dbg(otg->dev, "OTG_STATE_A_HOST state\n");
		if (idpin) {
			mxs_usbphy_start_host(otg, 0);
			ret = mxs_usbphy_start_peripheral(otg, 1);
			if (ret)
				break;
			otg->state = OTG_STATE_B_PERIPHERAL;
		}
		break;
	default:
		break;
	}
}

static irqreturn_t mxs_usbphy_irq(int irq, void *data)
{
	struct mxs_usbphy *phy = data;
	u32 otgsc = readl(phy->otg_base + 0x1a4);

	dev_dbg(phy->dev, "%s: 0x%08x\n", __func__, otgsc);

	if (!(otgsc & (1 << 16)))
		return IRQ_NONE;

	writel(otgsc | 1 << 16, phy->otg_base + 0x1a4);

	if (otgsc & (1 << 8))
		dev_info(phy->dev, "ID -> device (high)\n");
	else
		dev_info(phy->dev, "ID -> host (low)\n");

	schedule_work(&phy->work);

	return IRQ_HANDLED;
}

static irqreturn_t mxs_usbphy_irq_disconnect(int irq, void *data)
{
	struct mxs_usbphy *phy = data;
	u32 ctrl = readl(phy->base + USBPHY_CTRL);

	if ((ctrl & USBPHY_CTRL_DISCONNECT) == USBPHY_CTRL_DISCONNECT) {
		writel(USBPHY_CTRL_DISCONNECT | USBPHY_CTRL_ENHOSTDISCONDETECT,
		       phy->base + USBPHY_CTRL + CLR);
	}

	return IRQ_HANDLED;
}

static int __init mxs_usbphy_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	int irq;
	struct mxs_usbphy *phy;
	struct mxs_mxs_usbphy_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata)
		return -EINVAL;

	phy = devm_kzalloc(&pdev->dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	phy->base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!phy->base)
		return -ENOMEM;

	/* Reset USBPHY module */
	writel(USBPHY_CTRL_SFTRST, phy->base + USBPHY_CTRL + SET);
	udelay(10);

	phy->otg.state = OTG_STATE_UNDEFINED;
	INIT_WORK(&phy->work, msm_otg_sm_work);

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq, mxs_usbphy_irq_disconnect,
			       IRQF_SHARED, dev_name(&pdev->dev), phy);
	if (ret)
		return ret;

	/* OTG is only supported on the 1st USB port */
	if (pdev->id == 0 && pdata->flags & MXS_USBPHY_OTG) {
		ret = devm_request_irq(&pdev->dev, MX28_INT_USB0,
				       mxs_usbphy_irq, IRQF_SHARED,
				       dev_name(&pdev->dev), phy);
		if (ret)
			return ret;
		phy->otg_base = devm_ioremap(&pdev->dev,
					     MX28_USBCTRL0_BASE_ADDR, SZ_512);
		if (!phy->otg_base)
			return -ENOMEM;
		writel(0x01242520, phy->otg_base + 0x1a4);
		/* FIXME: 0x01242020 - "5" should be read only */
	}

	phy->set_vbus = pdata->set_vbus;
	phy->flags = pdata->flags;
	phy->otg.set_host = mxs_usbphy_set_host;
	phy->otg.set_peripheral = mxs_usbphy_set_peripheral;
	phy->otg.dev_id_host = pdata->dev_id_host;
	phy->otg.dev_id_peripheral = pdata->dev_id_peripheral;
	phy->otg.dev = &pdev->dev;
	phy->otg.set_power = mxs_usbphy_set_power;
	phy->dev = &pdev->dev;

	/* Remove CLKGATE and SFTRST */
	writel(USBPHY_CTRL_CLKGATE | USBPHY_CTRL_SFTRST,
	       phy->base + USBPHY_CTRL + CLR);

	writel(USBPHY_CTRL_ENDEVPLUGINDETECT |
	       USBPHY_CTRL_ENIRQHOSTDISCON | USBPHY_CTRL_ENHOSTDISCONDETECT,
	       phy->base + USBPHY_CTRL + SET);

	if (pdata->flags & MXS_USBPHY_OTG)
		writel(USBPHY_CTRL_ENOTGIDDETECT,
		       phy->base + USBPHY_CTRL + SET);

	platform_set_drvdata(pdev, phy);
	otg_add_transceiver(&phy->otg);

	return ret;
}

static int __exit mxs_usbphy_remove(struct platform_device *pdev)
{
	struct mxs_usbphy *phy = platform_get_drvdata(pdev);

	otg_remove_transceiver(&phy->otg);
	return 0;
}

static struct platform_driver mxs_usbphy_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.remove = __exit_p(mxs_usbphy_remove),
};

static int __init mxs_usbphy_init(void)
{
	return platform_driver_probe(&mxs_usbphy_driver, mxs_usbphy_probe);
}

static void __exit mxs_usbphy_exit(void)
{
	platform_driver_unregister(&mxs_usbphy_driver);
}

module_init(mxs_usbphy_init);
module_exit(mxs_usbphy_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("i.MX USB transceiver driver");
