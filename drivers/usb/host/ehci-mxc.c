/*
 * Freescale i.MX EHCI driver
 *
 * Copyright (c) 2012 Lothar Waßmann <LW@KARO-electronics.de>
 *   based on ehci-mxs.c:
 *     Copyright (c) 2012 Marek Vasut <marex@denx.de>
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
 *
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/slab.h>
#include <linux/usb/imx-usb.h>

#include <mach/hardware.h>

/* Called during probe() after chip reset completes */
static int ehci_mxc_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	hcd->has_tt = 1;
	ehci_setup(hcd);
	ehci_port_power(ehci, 0);

	return 0;
}

static irqreturn_t mxc_ehci_irq(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct imx_otg *data = hcd->self.controller->platform_data;
	struct imx_otg_priv *otg_priv = data->priv;
	struct usb_phy *phy = otg_priv->otg.phy;
	uint32_t status;
	static uint32_t discon;

	if (phy && phy->otg && phy->otg->set_vbus) {
		status = ehci_readl(ehci, &ehci->regs->status);
		status = !!(status & STS_PCD);
		if (status != discon) {
			discon = status;
			phy->otg->set_vbus(phy->otg, status);
		}
	}

	return ehci_irq(hcd);
}

static const struct hc_driver ehci_mxc_hc_driver = {
	.description	= hcd_name,
	.product_desc	= "Freescale i.MX On-Chip EHCI Host Controller",
	.hcd_priv_size	= sizeof(struct ehci_hcd),

	/*
	 * Generic hardware linkage
	 */
	.irq		= mxc_ehci_irq,
	.flags		= HCD_USB2 | HCD_MEMORY,

	/*
	 * Basic lifecycle operations
	 */
	.reset		= ehci_mxc_setup,
	.start		= ehci_run,
	.stop		= ehci_stop,
	.shutdown	= ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number	= ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,

	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
};

static int ehci_mxc_drv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx_otg *data = pdev->dev.platform_data;
	struct imx_otg_priv *otg_priv;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	struct usb_phy *phy;
	int ret;

	dev_info(dev, "Initializing i.MX USB Controller\n");

	if (!data) {
		dev_err(dev, "USB Host platform data missing!\n");
		return -ENODEV;
	}

	otg_priv = data->priv;

	phy = otg_priv->otg.phy;
	if (!phy) {
		dev_err(&pdev->dev, "Unable to find transceiver.\n");
		return -ENODEV;
	}

	/* Create HCD controller instance. */
	hcd = usb_create_hcd(&ehci_mxc_hc_driver, dev, dev_name(dev));
	if (!hcd) {
		dev_err(dev, "Failed to create HCD instance!\n");
		return -ENOMEM;
	}

	hcd->rsrc_start = data->mem_res->start;
	hcd->rsrc_len = resource_size(data->mem_res);
	hcd->regs = otg_priv->mem;
	hcd->irq = data->irq;

	/* Wait for the controller to stabilize. */
//	mdelay(10);

	ehci = hcd_to_ehci(hcd);

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100 +
		HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));

	platform_set_drvdata(pdev, hcd);

	/* Initialize the PHY. */
	ret = usb_phy_init(phy);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to initialize PHY: %d\n", ret);
		goto err_put_hcd;
	}

	/* Connect this host to the PHY. */
	ret = otg_set_host(phy->otg, &hcd->self);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to set transceiver host\n");
		goto err_shutdown_phy;
	}
	return 0;

err_shutdown_phy:
	usb_phy_shutdown(phy);
err_put_hcd:
	usb_put_hcd(hcd);
	return ret;
}

static int __exit ehci_mxc_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct imx_otg *data = pdev->dev.platform_data;
	struct imx_otg_priv *otg_priv = data->priv;
	struct usb_phy *phy = otg_priv->otg.phy;

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	usb_phy_shutdown(phy);
	if (phy->otg)
		otg_set_host(phy->otg, NULL);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver ehci_mxc_driver = {
	.probe		= ehci_mxc_drv_probe,
	.remove		= __exit_p(ehci_mxc_drv_remove),
	.driver		= {
		   .name	= "mxc-ehci",
	},
};

MODULE_ALIAS("platform:mxc-ehci");
