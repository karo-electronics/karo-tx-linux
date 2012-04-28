/*
 * include/linux/usb/imx-usb.h
 *
 * Freescale i.MX USB driver shared data.
 * Copyright (C) 2012 Lothar Waßmann <LW@KARO-electronics.de>
 *   based on mxs-usb.h:
 *   Copyright (C) 2012 Marek Vasut <marex@denx.de>
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
 */

#ifndef __INCLUDE_LINUX_USB_IMX_USB_H__
#define __INCLUDE_LINUX_USB_IMX_USB_H__

#include <linux/types.h>
#include <linux/platform_device.h>

#include <linux/usb/otg.h>

/* values for flags field */
#define MXC_EHCI_INTERFACE_DIFF_UNI	(0 << 0)
#define MXC_EHCI_INTERFACE_DIFF_BI	(1 << 0)
#define MXC_EHCI_INTERFACE_SINGLE_UNI	(2 << 0)
#define MXC_EHCI_INTERFACE_SINGLE_BI	(3 << 0)
#define MXC_EHCI_INTERFACE_MASK		0xf

#define MXC_EHCI_POWER_PINS_ENABLED	(1 << 5)
#define MXC_EHCI_TTL_ENABLED		(1 << 6)

#define MXC_EHCI_INTERNAL_PHY		(1 << 7)
#define MXC_EHCI_IPPUE_DOWN		(1 << 8)
#define MXC_EHCI_IPPUE_UP		(1 << 9)
#define MXC_EHCI_WAKEUP_ENABLED		(1 << 10)
#define MXC_EHCI_ITC_NO_THRESHOLD	(1 << 11)

struct imx_otg_priv {
	struct gpio_sw		*gpio_vbus;
	enum usb_otg_state	new_state;
	enum usb_otg_state	cur_state;
	struct usb_otg		otg;
	struct work_struct	work;
	struct device		*dev;
	uint32_t		status;
	void __iomem		*mem;
};

struct imx_otg {
	struct platform_device	*pdev_host;
	struct platform_device	*pdev_gadget;

	struct clk		*clk;
	struct clk		*bus_clk;
	struct resource		*mem_res;
	int			irq;
	int			irq_wakeup;

	struct imx_otg_priv	*priv;
};

struct imx_usb_platform_data {
	uint32_t		gpio_vbus_id;
	const char		*host_name;
	int			host_id;
	const char		*gadget_name;
};

#endif /* __INCLUDE_LINUX_USB_IMX_USB_H__ */
