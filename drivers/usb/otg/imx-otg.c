/*
 * drivers/usb/otg/imx-otg.c
 *
 * Freescale i.MX USB composite driver.
 *
 * Copyright (C) 2012 Marek Vasut <marex@denx.de>
 * on behalf of DENX Software Engineering GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio-switch.h>

#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>
#include <linux/usb/ehci_def.h>
#include <linux/usb/imx-usb.h>

#include <mach/common.h>
#include <mach/hardware.h>

/*
 * Allocate platform device with the DMA mask, this is borrowed from
 * arch/arm/mach-mxs/devices.c
 */
static struct platform_device *__devinit add_platform_device(
		const char *name, int id,
		const void *data, size_t size_data, u64 dmamask)
{
	int ret = -ENOMEM;
	struct platform_device *pdev;

	pdev = platform_device_alloc(name, id);
	if (!pdev)
		goto err;

	if (dmamask) {
		/*
		 * This memory isn't freed when the device is put,
		 * I don't have a nice idea for that though.  Conceptually
		 * dma_mask in struct device should not be a pointer.
		 * See http://thread.gmane.org/gmane.linux.kernel.pci/9081
		 */
		pdev->dev.dma_mask =
			kmalloc(sizeof(*pdev->dev.dma_mask), GFP_KERNEL);
		if (!pdev->dev.dma_mask)
			/* ret is still -ENOMEM; */
			goto err;

		*pdev->dev.dma_mask = dmamask;
		pdev->dev.coherent_dma_mask = dmamask;
	}

	if (data) {
		ret = platform_device_add_data(pdev, data, size_data);
		if (ret)
			goto err;
	}

	ret = platform_device_add(pdev);
	if (ret) {
err:
		if (dmamask)
			kfree(pdev->dev.dma_mask);
		platform_device_put(pdev);
		return ERR_PTR(ret);
	}

	return pdev;
}

static int imx_otg_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct imx_otg_priv *priv = container_of(otg, struct imx_otg_priv, otg);
	struct usb_phy *phy = otg->phy;

	if (host) {
		WARN_ON(otg->host);
		otg->host = host;
		priv->new_state = OTG_STATE_A_HOST;
		phy->init(phy);
	} else {
		WARN_ON(!otg->host);
		otg->host = host;
		priv->new_state = OTG_STATE_UNDEFINED;
	}

	schedule_work(&priv->work);

	return 0;
}

static int imx_otg_set_peripheral(struct usb_otg *otg, struct usb_gadget *gg)
{
	struct imx_otg_priv *priv = container_of(otg, struct imx_otg_priv, otg);
	struct usb_phy *phy = otg->phy;

	if (gg) {
		WARN_ON(otg->gadget);
		otg->gadget = gg;
		priv->new_state = OTG_STATE_B_PERIPHERAL;
		phy->init(phy);
	} else {
		WARN_ON(!otg->gadget);
		otg->gadget = gg;
		priv->new_state = OTG_STATE_UNDEFINED;
	}

	schedule_work(&priv->work);

	return 0;
}

static void imx_otg_work(struct work_struct *w)
{
	struct imx_otg_priv *priv = container_of(w, struct imx_otg_priv, work);
	struct usb_hcd *hcd;

	switch (priv->cur_state) {
	case OTG_STATE_A_HOST:
		if (priv->new_state == OTG_STATE_UNDEFINED) {
			if (priv->otg.host) {
				hcd = bus_to_hcd(priv->otg.host);
				usb_remove_hcd(hcd);
			}
			priv->cur_state = priv->new_state;
			/* Turn off VBUS */
			gpio_switch_set(priv->gpio_vbus, 0);
		}
		break;

	case OTG_STATE_B_PERIPHERAL:
		if (priv->new_state == OTG_STATE_UNDEFINED) {
			if (priv->otg.gadget)
				usb_del_gadget_udc(priv->otg.gadget);
			priv->cur_state = priv->new_state;
		}
		break;

	case OTG_STATE_UNDEFINED:
		/* Check desired state. */
		switch (priv->new_state) {
		case OTG_STATE_A_HOST:
			if (!priv->otg.host)
				break;
			priv->cur_state = priv->new_state;

			/* Turn on VBUS */
			gpio_switch_set(priv->gpio_vbus, 1);

			hcd = bus_to_hcd(priv->otg.host);
			usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
			break;

		case OTG_STATE_B_PERIPHERAL:
			if (!priv->otg.gadget)
				break;
			priv->cur_state = priv->new_state;
			usb_add_gadget_udc(priv->dev, priv->otg.gadget);
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}
}

static int __devinit imx_otg_dt_probe(struct platform_device *pdev,
				struct imx_usb_platform_data *pdata)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;
	const u32 *ph;
	const u32 *id;

	if (!np)
		return -ENODEV;

	ret = of_property_read_string(np, "host-device-name",
				&pdata->host_name);
	if (ret < 0 && ret != -EINVAL) {
		dev_err(&pdev->dev, "Failed to read property 'host-device-name': %d\n",
			ret);
		return ret;
	}

	ret = of_property_read_string(np, "gadget-device-name",
				&pdata->gadget_name);
	if (ret < 0 && ret != -EINVAL) {
		dev_err(&pdev->dev, "Failed to read property 'gadget-device-name': %d\n",
			ret);
		return ret;
	}

	if (!pdata->host_name && !pdata->gadget_name)
		return -ENODEV;

	ph = of_get_property(np, "vbus-gpio", NULL);
	if (ph)
		pdata->gpio_vbus_id = be32_to_cpu(*ph);

	id = of_get_property(np, "host-device-id", NULL);
	if (id)
		pdata->host_id = be32_to_cpu(*id);

	return 0;
}

static int __devinit imx_otg_probe(struct platform_device *pdev)
{
	struct imx_usb_platform_data *pdata = pdev->dev.platform_data;
	struct imx_otg_priv *priv;
	struct imx_otg *imx_otg;
	struct usb_phy *phy;
	struct usb_otg *otg;
	int ret;

	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		ret = imx_otg_dt_probe(pdev, pdata);
		if (ret) {
			dev_err(&pdev->dev, "No platform data supplied!\n");
			return ret;
		}
	}

	phy = usb_find_transceiver(&pdev->dev);
	if (!phy)
		return -EPROBE_DEFER;

	/*
	 * Until further notice, this claims all necessary resources.
	 */

	/* Allocate driver's private date. */
	imx_otg = devm_kzalloc(&pdev->dev, sizeof(*imx_otg), GFP_KERNEL);
	if (!imx_otg) {
		dev_err(&pdev->dev, "Failed to allocate OTG nodes data!\n");
		ret = -ENOMEM;
		goto err_alloc_data;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "Failed to allocate OTG nodes data!\n");
		ret = -ENOMEM;
		goto err_alloc_data;
	}
	imx_otg->priv = priv;

	/* Configure the OTG structure. */
	otg				= &priv->otg;
	otg->phy			= phy;
	otg->set_host			= imx_otg_set_host;
	otg->set_peripheral		= imx_otg_set_peripheral;
	phy->otg			= otg;

	priv->dev			= &pdev->dev;
	priv->gpio_vbus = request_gpio_switch(&pdev->dev, pdata->gpio_vbus_id);
	priv->cur_state			= OTG_STATE_UNDEFINED;
	priv->new_state			= OTG_STATE_UNDEFINED;

	INIT_WORK(&priv->work, imx_otg_work);

	/* Claim the Host clock. */
	imx_otg->clk = clk_get(&pdev->dev, "usb");
	if (IS_ERR(imx_otg->clk)) {
		dev_err(&pdev->dev, "Failed to claim clock for USB Host\n");
		ret = PTR_ERR(imx_otg->clk);
		goto err_get_host_clk;
	}

	/* Prepare Host clock. */
	ret = clk_prepare_enable(imx_otg->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clock for USB Host.\n");
		goto err_prepare_host_clock;
	}

	/* Claim the Host bus clock. */
	imx_otg->bus_clk = clk_get(&pdev->dev, "usb_ahb");
	if (IS_ERR(imx_otg->bus_clk)) {
		dev_err(&pdev->dev, "Failed to claim bus clock for USB Host\n");
		ret = PTR_ERR(imx_otg->bus_clk);
		goto err_get_bus_clk;
	}

	/* Prepare Host bus clock. */
	ret = clk_prepare_enable(imx_otg->bus_clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable bus clock for USB Host.\n");
		goto err_prepare_bus_clock;
	}

	/* Get memory area for EHCI host from resources. */
	imx_otg->mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!imx_otg->mem_res) {
		dev_err(&pdev->dev, "Specify memory area for this USB Host!\n");
		ret = -ENODEV;
		goto err_get_host_resource;
	}

	/* Request the memory region for this USB Host. */
	if (!devm_request_mem_region(&pdev->dev, imx_otg->mem_res->start,
			resource_size(imx_otg->mem_res), pdev->name)) {
		dev_err(&pdev->dev, "USB Host memory area already in use!\n");
		ret = -EBUSY;
		goto err_get_host_resource;
	}

	/* Map the memory region for USB Host. */
	priv->mem = devm_ioremap(&pdev->dev, imx_otg->mem_res->start,
				resource_size(imx_otg->mem_res));
	if (!priv->mem) {
		dev_err(&pdev->dev, "Memory mapping of USB Host failed!\n");
		ret = -EFAULT;
		goto err_get_host_resource;
	}

	/* Get IRQ for EHCI host from resources. */
	imx_otg->irq = platform_get_irq(pdev, 0);
	if (imx_otg->irq < 0) {
		dev_err(&pdev->dev, "Specify IRQ for this USB Host!\n");
		ret = -ENODEV;
		goto err_get_host_resource;
	}

	/*
	 * Now finally probe the Host driver!
	 */
	if (pdata->gadget_name) {
		imx_otg->pdev_gadget = add_platform_device(pdata->gadget_name,
					-1, imx_otg, sizeof(*imx_otg),
					DMA_BIT_MASK(32));
		if (IS_ERR(imx_otg->pdev_gadget)) {
			ret = PTR_ERR(imx_otg->pdev_gadget);
			dev_err(&pdev->dev, "Failed registering gadget: %d\n",
				ret);
			goto err_register_gadget;
		}
	}

	if (pdata->host_name) {
		imx_otg->pdev_host = add_platform_device(pdata->host_name,
					pdata->host_id, imx_otg,
					sizeof(*imx_otg), DMA_BIT_MASK(32));
		if (IS_ERR(imx_otg->pdev_host)) {
			ret = PTR_ERR(imx_otg->pdev_host);
			dev_err(&pdev->dev, "Failed registering Host: %d\n",
				ret);
			goto err_register_host;
		}
	}

	platform_set_drvdata(pdev, imx_otg);

	return 0;

err_register_host:
	if (imx_otg->pdev_gadget)
		platform_device_unregister(imx_otg->pdev_gadget);
	imx_otg->pdev_gadget = NULL;
	platform_device_put(imx_otg->pdev_host);
err_register_gadget:
	if (imx_otg->pdev_gadget)
		platform_device_put(imx_otg->pdev_gadget);
err_get_host_resource:
	clk_disable_unprepare(imx_otg->bus_clk);
err_prepare_bus_clock:
	clk_put(imx_otg->bus_clk);
err_get_bus_clk:
	clk_disable_unprepare(imx_otg->clk);
err_prepare_host_clock:
	clk_put(imx_otg->clk);
err_get_host_clk:
	free_gpio_switch(priv->gpio_vbus);
err_alloc_data:
	if (phy)
		usb_put_transceiver(phy);
	return ret;
}

static int __devexit imx_otg_remove(struct platform_device *pdev)
{
	struct imx_otg *imx_otg = platform_get_drvdata(pdev);
	struct imx_otg_priv *priv = imx_otg->priv;

	/* Stop the PHY work. */
	cancel_work_sync(&priv->work);

	/* Shut off VBUS. */
	gpio_switch_set(priv->gpio_vbus, 0);
	free_gpio_switch(priv->gpio_vbus);

	/* Deregister both Gadget and Host driver. */
	if (imx_otg->pdev_gadget)
		platform_device_unregister(imx_otg->pdev_gadget);

	if (imx_otg->pdev_host)
		platform_device_unregister(imx_otg->pdev_host);

	/* Stop the clocks. */
	clk_disable_unprepare(imx_otg->clk);
	clk_put(imx_otg->clk);
	clk_disable_unprepare(imx_otg->bus_clk);
	clk_put(imx_otg->bus_clk);

	usb_put_transceiver(priv->otg.phy);
	return 0;
}

static struct of_device_id imx_otg_dt_ids[] = {
	{ .compatible = "fsl,imx-otg", },
	{ /* sentinel */ }
};

static struct platform_driver imx_otg_driver = {
	.probe		= imx_otg_probe,
	.remove		= __devexit_p(imx_otg_remove),
	.driver		= {
		.name	= "imx-otg",
		.owner	= THIS_MODULE,
		.of_match_table = imx_otg_dt_ids,
	},
};

module_platform_driver(imx_otg_driver);

MODULE_ALIAS("platform:imx-otg");
MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_DESCRIPTION("Freescale i.MX USB composite driver");
MODULE_LICENSE("GPL");
