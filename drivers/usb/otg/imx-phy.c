/*
 * drivers/usb/otg/imx-phy.c
 *
 * Freescale i.MX5x USB PHY driver.
 * Copyright (C) 2012 Lothar Waßmann <LW@KARO-electronics.de>
 *   based on drivers/usb/otg/mxs-phy.c:
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/fsl_devices.h>

#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/usb/imx-usb.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>

/* USB CMD  Register Bit Masks */
#define	 USB_CMD_RUN_STOP		0x00000001
#define	 USB_CMD_CTRL_RESET		0x00000002
#define	 USB_CMD_PERIODIC_SCHEDULE_EN	0x00000010
#define	 USB_CMD_ASYNC_SCHEDULE_EN	0x00000020
#define	 USB_CMD_INT_AA_DOORBELL	0x00000040
#define	 USB_CMD_ASP			0x00000300
#define	 USB_CMD_ASYNC_SCH_PARK_EN	0x00000800
#define	 USB_CMD_SUTW			0x00002000
#define	 USB_CMD_ATDTW			0x00004000
#define	 USB_CMD_ITC			0x00FF0000

/* USB MODE Register Bit Masks */
#define	 USB_MODE_CTRL_MODE_IDLE	0x00000000
#define	 USB_MODE_CTRL_MODE_DEVICE	0x00000002
#define	 USB_MODE_CTRL_MODE_HOST	0x00000003
#define	 USB_MODE_CTRL_MODE_MASK	0x00000003

/* PORTSCX  Register Bit Masks */
#define	 PORTSCX_PORT_WIDTH		0x10000000
#define	 PORTSCX_PHY_TYPE_SEL		0xC0000000

/* bit 27-26 are port speed */
#define	 PORTSCX_PORT_SPEED_MASK	0x0C000000
#define	 PORTSCX_PORT_SPEED_FULL	0x00000000
#define	 PORTSCX_PORT_SPEED_LOW		0x04000000
#define	 PORTSCX_PORT_SPEED_HIGH	0x08000000
#define	 PORTSCX_PORT_SPEED_UNDEF	0x0C000000
#define	 PORTSCX_SPEED_BIT_POS		26

/* bit 28 is parallel transceiver width for UTMI interface */
#define	 PORTSCX_PTW			0x10000000
#define	 PORTSCX_PTW_8BIT		0x00000000
#define	 PORTSCX_PTW_16BIT		0x10000000

/* bit 31-30 are port transceiver select */
#define	 PORTSCX_PTS_UTMI		0x00000000
#define	 PORTSCX_PTS_ULPI		0x80000000
#define	 PORTSCX_PTS_FSLS		0xC0000000
#define	 PORTSCX_PTS_BIT_POS		30


#define PHY_CTRL1_PLLDIV_SHIFT		0
#define PHY_CTRL1_PLLDIV_MASK		(0x03 << PHY_CTRL1_PLLDIV_SHIFT)
#define PHY_CTRL1_PLL_DIV_19_2_MHZ	(0 << PHY_CTRL1_PLLDIV_SHIFT)
#define PHY_CTRL1_PLL_DIV_24_MHZ	(1 << PHY_CTRL1_PLLDIV_SHIFT)
#define PHY_CTRL1_PLL_DIV_26_MHZ	(2 << PHY_CTRL1_PLLDIV_SHIFT)
#define PHY_CTRL1_PLL_DIV_27_MHZ	(3 << PHY_CTRL1_PLLDIV_SHIFT)

#define REG_USB_CTRL_0			0x00
#define REG_OTG_PHY_CTRL_0		0x08
#define REG_OTG_PHY_CTRL_1		0x0c
#define REG_USB_CTRL_1			0x10
#define REG_UH2_CTRL			0x14
#define REG_UH3_CTRL			0x18
#define REG_UH1_PHY_CTRL_0		0x1c
#define REG_UH1_PHY_CTRL_1		0x20
#define REG_CLKONOFF_CTRL		0x24

#define REG_USBCMD			0x140
#define REG_USBSTS			0x144
#define REG_PORTSC1			0x184
#define REG_OTGSC			0x1a4
#define REG_USBMODE			0x1a8

/* USB_CTRL1 */
#define MXC_USB_CTRL1_UH1_EXT_CLK_EN	(1 << 25)

/* USB_CTRL */
#define MXC_UCTRL_OWIE			(1 << 27)	/* OTG wakeup intr enable */
#define MX51_UCTRL_OPM			(1 << 24)	/* OTG power mask */
#define MX53_UCTRL_OTG_PWR_POL		(1 << 24)	/* OTG power enable polarity  */
#define MX51_UCTRL_H1UIE		(1 << 12)	/* Host1 ULPI interrupt enable */
#define MXC_UCTRL_H1WIE			(1 << 11)	/* HOST1 wakeup intr enable */
#define MX51_UCTRL_H1PM			(1 <<  8)	/* HOST1 power mask */
#define MX53_UCTRL_H1_PWR_POL		(1 <<  8)	/* HOST1 power enable polarity */

/* USB_PHY_CTRL_0 */
#define PHY_CTRL0_OTG_OC_DIS		(1 << 8)	/* OTG Disable Overcurrent Event */
#define PHY_CTRL0_H1_OC_DIS		(1 << 5)	/* UH1 Disable Overcurrent Event */

/* USBH2CTRL */
#define UH2_CTRL_H2UIE			(1 << 8)
#define UH2_CTRL_H2WIE			(1 << 7)
#define UH2_CTRL_H2PM			(1 << 4)

/* USBH3CTRL */
#define UH3_CTRL_H3UIE			(1 << 8)
#define UH3_CTRL_H3WIE			(1 << 7)
#define UH3_CTRL_H3PM			(1 << 4)

/* USBCMD */
#define USBCMD_ITC_THRESHOLD_MASK	(0xff << 16)	/* Interrupt Threshold Control */

#define MAX_PORTS	4

enum {
	MXC_USB_PORT_OTG,
	MXC_USB_PORT_H1,
	MXC_USB_PORT_H2,
	MXC_USB_PORT_H3,
};

#define to_imx_phy_port(x)	container_of(x, struct imx_phy_port, phy)

enum {
	IMX_PHY_HOST = 1,
	IMX_PHY_DEVICE = 2,
	IMX_PHY_OTG = 4,
};

enum imx_usbphy_devtype {
	IMX51_USBPHY,
	IMX53_USBPHY,
};

struct imx_phy_port {
	struct imx_phy *imx_phy;
	struct usb_phy phy;
	struct device *parent;
	struct clk *clk;
	unsigned long flags;
	enum fsl_usb2_phy_modes phy_mode;
	u32 phy_flags;
	int port_id;
	int (*set_vbus)(struct device *dev, int on);
	struct work_struct work;
};

struct imx_phy {
	void __iomem *usb_base;
	struct imx_usbphy_devdata *devdata;
	struct imx_phy_port ports[MAX_PORTS];
};

struct imx_usbphy_devdata {
	enum imx_usbphy_devtype devtype;
	int (*hub_port_init)(void __iomem *hcd_regs, struct imx_phy_port *port);
};

static int imx_phy_usb_reset(void __iomem *hcd_regs, u32 mode)
{
	u32 v;
	unsigned long timeout;

	v = readl(hcd_regs + REG_USBCMD);
	v &= ~USB_CMD_RUN_STOP;
	writel(v, hcd_regs + REG_USBCMD);

	v = readl(hcd_regs + REG_USBCMD);
	v |= USB_CMD_CTRL_RESET;
	writel(v, hcd_regs + REG_USBCMD);

	/* Wait for reset to complete */
	timeout = jiffies + HZ;
	while (readl(hcd_regs + REG_USBCMD) & USB_CMD_CTRL_RESET) {
		if (time_after(jiffies, timeout)) {
			pr_err("usb reset timeout!\n");
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

	/* Switch the controller to host mode */
	v = readl(hcd_regs + REG_USBMODE);
	v &= ~USB_MODE_CTRL_MODE_MASK;	/* clear mode bits */
	v |= mode;
	writel(v, hcd_regs + REG_USBMODE);

	/* Clear the setup status */
	writel(0, hcd_regs + REG_USBSTS);
	return 0;
}

#ifdef CONFIG_ARCH_MX51
static int mx51_initialize_usb_hw(void __iomem *hcd_regs,
				struct imx_phy_port *port)
{
	int ret = 0;
	struct imx_phy *imx_phy = port->imx_phy;
	u32 v;

	switch (port->port_id) {
	case 0:	/* OTG port */
		if (port->phy_mode == FSL_USB2_PHY_UTMI ||
			port->phy_mode == FSL_USB2_PHY_UTMI_WIDE) {

			v = readl(imx_phy->usb_base + REG_OTG_PHY_CTRL_1);
			if ((v & PHY_CTRL1_PLLDIV_MASK) !=
				PHY_CTRL1_PLL_DIV_24_MHZ) {
				v &= ~PHY_CTRL1_PLLDIV_MASK;
				v |= PHY_CTRL1_PLL_DIV_24_MHZ;
				writel(v, imx_phy->usb_base + REG_OTG_PHY_CTRL_1);
			}

			v = readl(hcd_regs + REG_PORTSC1);
			v &= ~(PORTSCX_PHY_TYPE_SEL | PORTSCX_PORT_WIDTH);
			v |= PORTSCX_PTS_UTMI;
			if (port->phy_mode == FSL_USB2_PHY_UTMI_WIDE)
				v |= PORTSCX_PTW_16BIT;
			writel(v, hcd_regs + REG_PORTSC1);

			v = readl(imx_phy->usb_base + REG_OTG_PHY_CTRL_0);

			if (port->phy_flags & MXC_EHCI_POWER_PINS_ENABLED) {
				/* OC/USBPWR is used */
				v &= ~PHY_CTRL0_OTG_OC_DIS;
			} else {
				/* OC/USBPWR is not used */
				v |= PHY_CTRL0_OTG_OC_DIS;
			}
			writel(v, imx_phy->usb_base + REG_OTG_PHY_CTRL_0);

			v = readl(imx_phy->usb_base + REG_USB_CTRL_0);
			if (port->phy_flags & MXC_EHCI_WAKEUP_ENABLED)
				/* OTG wakeup enable */
				v |= MXC_UCTRL_OWIE;
			else
				/* OTG wakeup disable */
				v &= ~MXC_UCTRL_OWIE;

			if (port->phy_flags & MXC_EHCI_POWER_PINS_ENABLED)
				v &= ~MX51_UCTRL_OPM;
			else
				v |= MX51_UCTRL_OPM;
			writel(v, imx_phy->usb_base + REG_USB_CTRL_0);
		} else {
			return -EINVAL;
		}
		break;

	case 1:	/* Host 1 ULPI */
		if (port->phy_mode == FSL_USB2_PHY_ULPI) {
			pr_debug("%s: Configuring port %d for ULPI mode\n",
				__func__, port->port_id);

			v = readl(imx_phy->usb_base + REG_USB_CTRL_1);
			v |= MXC_USB_CTRL1_UH1_EXT_CLK_EN;
			writel(v, imx_phy->usb_base + REG_USB_CTRL_1);

			v = readl(hcd_regs + REG_PORTSC1);
			v &= ~(PORTSCX_PHY_TYPE_SEL | PORTSCX_PORT_WIDTH);
			v |= PORTSCX_PTS_ULPI;
			writel(v, hcd_regs + REG_PORTSC1);

			v = readl(imx_phy->usb_base + REG_USB_CTRL_0);
			if (port->phy_flags & MXC_EHCI_WAKEUP_ENABLED) {
				/* HOST1 wakeup/ULPI intr enable */
				v |= (MXC_UCTRL_H1WIE | MX51_UCTRL_H1UIE);
			} else {
				/* HOST1 wakeup/ULPI intr disable */
				v &= ~(MXC_UCTRL_H1WIE | MX51_UCTRL_H1UIE);
			}

			if (port->phy_flags & MXC_EHCI_POWER_PINS_ENABLED)
				v &= ~MX51_UCTRL_H1PM; /* HOST1 power mask used*/
			else
				v |= MX51_UCTRL_H1PM; /* HOST1 power mask used*/
			writel(v, imx_phy->usb_base + REG_USB_CTRL_0);

			v = readl(imx_phy->usb_base + REG_OTG_PHY_CTRL_0);
			if (port->phy_flags & MXC_EHCI_POWER_PINS_ENABLED)
				v &= ~PHY_CTRL0_H1_OC_DIS; /* OC is used */
			else
				v |= PHY_CTRL0_H1_OC_DIS; /* OC is not used */
			writel(v, imx_phy->usb_base + REG_OTG_PHY_CTRL_0);

			if (port->phy_flags & MXC_EHCI_ITC_NO_THRESHOLD) {
				v = readl(hcd_regs + REG_USBCMD);
				/* Interrupt Threshold Control:Immediate (no threshold) */
				v &= ~USBCMD_ITC_THRESHOLD_MASK;
				writel(v, hcd_regs + REG_USBCMD);
			}
		} else {
			return -EINVAL;
		}
		break;

	case 2: /* Host 2 ULPI */
		if (port->phy_mode == FSL_USB2_PHY_ULPI) {
			v = readl(imx_phy->usb_base + REG_UH2_CTRL);
			if (port->phy_flags & MXC_EHCI_WAKEUP_ENABLED) {
				/* HOST1 wakeup/ULPI intr enable */
				v |= (UH2_CTRL_H2WIE | UH2_CTRL_H2UIE);
			} else {
				/* HOST1 wakeup/ULPI intr disable */
				v &= ~(UH2_CTRL_H2WIE | UH2_CTRL_H2UIE);
			}

			if (port->phy_flags & MXC_EHCI_POWER_PINS_ENABLED)
				v &= ~UH2_CTRL_H2PM; /* HOST2 power mask used */
			else
				v |= UH2_CTRL_H2PM; /* HOST2 power mask not used */
			writel(v, imx_phy->usb_base + REG_UH2_CTRL);
		} else {
			return -EINVAL;
		}
		break;

	case 3: /* Host 3 ULPI */
		if (port->phy_mode == FSL_USB2_PHY_ULPI) {
			v = readl(imx_phy->usb_base + REG_UH3_CTRL);
			if (port->phy_flags & MXC_EHCI_WAKEUP_ENABLED) {
				/* HOST1 wakeup/ULPI intr enable */
				v |= (UH3_CTRL_H3WIE | UH3_CTRL_H3UIE);
			} else {
				/* HOST1 wakeup/ULPI intr disable */
				v &= ~(UH3_CTRL_H3WIE | UH3_CTRL_H3UIE);
			}

			if (port->phy_flags & MXC_EHCI_POWER_PINS_ENABLED)
				v &= ~UH3_CTRL_H3PM; /* HOST3 power mask used*/
			else
				v |= UH3_CTRL_H3PM; /* HOST3 power mask not used*/
			writel(v, imx_phy->usb_base + REG_UH3_CTRL);
		} else {
			return -EINVAL;
		}
		break;

	default:
		return -EINVAL;
	}
	return ret;
}
#else
static int mx51_initialize_usb_hw(void __iomem *hcd_regs,
				struct imx_phy_port *port)
{
	return -ENODEV;
}
#endif

#ifdef CONFIG_ARCH_MX53
static int mx53_initialize_usb_hw(void __iomem *hcd_regs,
				struct imx_phy_port *port)
{
	int ret = 0;
	struct imx_phy *imx_phy = port->imx_phy;
	u32 v;

	switch (port->port_id) {
	case 0: /* OTG port */
		if (port->phy_mode == FSL_USB2_PHY_UTMI ||
			port->phy_mode == FSL_USB2_PHY_UTMI_WIDE) {

			pr_debug("%s: Configuring port %d for UTMI mode\n",
				__func__, port->port_id);
			v = readl(imx_phy->usb_base + REG_OTG_PHY_CTRL_1);
			if ((v & PHY_CTRL1_PLLDIV_MASK) !=
				PHY_CTRL1_PLL_DIV_24_MHZ) {
				v &= ~PHY_CTRL1_PLLDIV_MASK;
				v |= PHY_CTRL1_PLL_DIV_24_MHZ;
				writel(v, imx_phy->usb_base + REG_OTG_PHY_CTRL_1);
			}

			v = readl(hcd_regs + REG_PORTSC1);
			v &= ~(PORTSCX_PHY_TYPE_SEL | PORTSCX_PORT_WIDTH);
			v |= PORTSCX_PTS_UTMI;
			if (port->phy_mode == FSL_USB2_PHY_UTMI_WIDE)
				v |= PORTSCX_PTW_16BIT;
			writel(v, hcd_regs + REG_PORTSC1);

			v = readl(imx_phy->usb_base + REG_OTG_PHY_CTRL_0);

			if (port->phy_flags & MXC_EHCI_POWER_PINS_ENABLED)
				/* OC/USBPWR is used */
				v &= ~PHY_CTRL0_OTG_OC_DIS;
			else
				/* OC/USBPWR is not used */
				v |= PHY_CTRL0_OTG_OC_DIS;
			writel(v, imx_phy->usb_base + REG_OTG_PHY_CTRL_0);

			v = readl(imx_phy->usb_base + REG_USB_CTRL_0);
			if (port->phy_flags & MXC_EHCI_WAKEUP_ENABLED)
				/* OTG wakeup enable */
				v |= MXC_UCTRL_OWIE;
			else
				/* OTG wakeup disable */
				v &= ~MXC_UCTRL_OWIE;
			v |= MX53_UCTRL_OTG_PWR_POL;
			writel(v, imx_phy->usb_base + REG_USB_CTRL_0);
		} else {
			return -EINVAL;
		}
		break;

	case 1: /* Host 1 UTMI */
		if (port->phy_mode == FSL_USB2_PHY_UTMI ||
			port->phy_mode == FSL_USB2_PHY_UTMI_WIDE) {

			pr_debug("%s: Configuring port %d for UTMI mode\n",
				__func__, port->port_id);
			v = readl(imx_phy->usb_base + REG_UH1_PHY_CTRL_1);
			if ((v & PHY_CTRL1_PLLDIV_MASK) !=
				PHY_CTRL1_PLL_DIV_24_MHZ) {
				v &= ~PHY_CTRL1_PLLDIV_MASK;
				v |= PHY_CTRL1_PLL_DIV_24_MHZ;
				writel(v, imx_phy->usb_base + REG_UH1_PHY_CTRL_1);
			}

			v = readl(hcd_regs + REG_PORTSC1);
			v &= ~(PORTSCX_PHY_TYPE_SEL | PORTSCX_PORT_WIDTH);
			v |= PORTSCX_PTS_UTMI;
			if (port->phy_mode == FSL_USB2_PHY_UTMI_WIDE)
				v |= PORTSCX_PTW_16BIT;
			writel(v, hcd_regs + REG_PORTSC1);

			v = readl(imx_phy->usb_base + REG_USB_CTRL_1);
			v &= ~MXC_USB_CTRL1_UH1_EXT_CLK_EN;
			writel(v, imx_phy->usb_base + REG_USB_CTRL_1);

			v = readl(imx_phy->usb_base + REG_UH1_PHY_CTRL_0);

			if (port->phy_flags & MXC_EHCI_POWER_PINS_ENABLED)
				/* OC is used */
				v &= ~PHY_CTRL0_H1_OC_DIS;
			else
				/* OC is not used */
				v |= PHY_CTRL0_H1_OC_DIS;
			writel(v, imx_phy->usb_base + REG_UH1_PHY_CTRL_0);

			v = readl(imx_phy->usb_base + REG_USB_CTRL_0);
			if (port->phy_flags & MXC_EHCI_WAKEUP_ENABLED)
				/* OTG wakeup enable */
				v |= MXC_UCTRL_H1WIE;
			else
				/* OTG wakeup disable */
				v &= ~MXC_UCTRL_H1WIE;
			v |= MX53_UCTRL_H1_PWR_POL;
			writel(v, imx_phy->usb_base + REG_USB_CTRL_0);
			if (port->phy_flags & MXC_EHCI_ITC_NO_THRESHOLD) {
				v = readl(hcd_regs + REG_USBCMD);
				/* Interrupt Threshold Control:Immediate (no threshold) */
				v &= ~USBCMD_ITC_THRESHOLD_MASK;
				writel(v, hcd_regs + REG_USBCMD);
			}
		} else {
			return -EINVAL;
		}
		break;

	case 2: /* Host 2 ULPI */
		if (port->phy_mode == FSL_USB2_PHY_ULPI) {
			v = readl(imx_phy->usb_base + REG_UH2_CTRL);
			if (port->phy_flags & MXC_EHCI_WAKEUP_ENABLED)
				/* HOST1 wakeup/ULPI intr enable */
				v |= (UH2_CTRL_H2WIE | UH2_CTRL_H2UIE);
			else
				/* HOST1 wakeup/ULPI intr disable */
				v &= ~(UH2_CTRL_H2WIE | UH2_CTRL_H2UIE);

			if (port->phy_flags & MXC_EHCI_POWER_PINS_ENABLED)
				/* HOST2 power mask used */
				v &= ~UH2_CTRL_H2PM;
			else
				/* HOST2 power mask not used */
				v |= UH2_CTRL_H2PM;
			writel(v, imx_phy->usb_base + REG_UH2_CTRL);
		} else {
			return -EINVAL;
		}
		break;

	case 3: /* Host 3 ULPI */
		if (port->phy_mode == FSL_USB2_PHY_ULPI) {
			v = readl(imx_phy->usb_base + REG_UH3_CTRL);
			if (port->phy_flags & MXC_EHCI_WAKEUP_ENABLED)
				/* HOST1 wakeup/ULPI intr enable */
				v |= (UH3_CTRL_H3WIE | UH3_CTRL_H3UIE);
			else
				/* HOST1 wakeup/ULPI intr disable */
				v &= ~(UH3_CTRL_H3WIE | UH3_CTRL_H3UIE);

			if (port->phy_flags & MXC_EHCI_POWER_PINS_ENABLED)
				/* HOST2 power mask used */
				v &= ~UH3_CTRL_H3PM;
			else
				/* HOST2 power mask not used */
				v |= UH3_CTRL_H3PM;
			writel(v, imx_phy->usb_base + REG_UH3_CTRL);
		} else {
			return -EINVAL;
		}
		break;

	default:
		return -EINVAL;
	}
	return ret;
}
#else
static int mx53_initialize_usb_hw(void __iomem *hcd_regs,
				struct imx_phy_port *port)
{
	return -ENODEV;
}
#endif

static int imx_phy_host_init(struct usb_phy *phy)
{
	int ret;
	struct imx_phy_port *port = to_imx_phy_port(phy);
	struct imx_phy *imx_phy = port->imx_phy;
	struct imx_otg_priv *priv = container_of(phy->otg,
						struct imx_otg_priv, otg);
	void __iomem *hcd_regs = priv->mem;

	/* Enable connect/disconnect IRQ */
	ret = imx_phy->devdata->hub_port_init(hcd_regs, port);
	if (ret) {
		dev_err(phy->dev, "Failed to initialize USB port\n");
		return ret;
	}
	ret = imx_phy_usb_reset(hcd_regs, USB_MODE_CTRL_MODE_HOST);
	if (ret)
		dev_err(phy->dev, "Failed to reset USB port\n");
	return ret;
}

static int imx_phy_device_init(struct usb_phy *phy)
{
	int ret;
	struct imx_otg_priv *priv = container_of(phy->otg,
						struct imx_otg_priv, otg);
	void __iomem *hcd_regs = priv->mem;

	/* Switch the controller to device mode */
	ret = imx_phy_usb_reset(hcd_regs, USB_MODE_CTRL_MODE_DEVICE);
	if (ret)
		dev_err(phy->dev, "Failed to reset USB port\n");
	return ret;
}

static int imx_usb_phy_init(struct usb_phy *phy)
{
	int ret;
	struct imx_phy_port *port = to_imx_phy_port(phy);

	if (phy->otg->host) {
		dev_dbg(phy->dev, "Initializing PHY %p in HOST mode\n", phy);
		return imx_phy_host_init(phy);
	} else if (phy->otg->gadget) {
		dev_dbg(phy->dev, "Initializing PHY %p in DEVICE mode\n", phy);
		return imx_phy_device_init(phy);
	} else {
		dev_dbg(phy->dev, "enabling PHY clk %p\n", port->clk);
		ret = clk_enable(port->clk);
		if (ret) {
			dev_err(phy->dev, "Failed to enable PHY clock: %d\n", ret);
			return ret;
		}
	}
	return 0;
}

static void imx_phy_shutdown(struct usb_phy *phy)
{
	struct imx_phy_port *port = to_imx_phy_port(phy);

	dev_dbg(phy->dev, "disabling PHY clk %p\n", port->clk);
	clk_disable(port->clk);
}

static struct imx_usbphy_devdata imx51_usbphy_devdata = {
	.devtype = IMX51_USBPHY,
	.hub_port_init = mx51_initialize_usb_hw,
};

static struct imx_usbphy_devdata imx53_usbphy_devdata = {
	.devtype = IMX53_USBPHY,
	.hub_port_init = mx53_initialize_usb_hw,
};

static const struct of_device_id imx_phy_dt_ids[] = {
	{ .compatible = "fsl,imx51-usb-phy", .data = &imx51_usbphy_devdata, },
	{ .compatible = "fsl,imx53-usb-phy", .data = &imx53_usbphy_devdata, },
	{ /* sentinel */ }
};

static struct {
	const char *name;
	enum fsl_usb2_phy_modes mode;
} imx_phy_modes[] = {
	{ "ulpi", FSL_USB2_PHY_ULPI, },
	{ "utmi", FSL_USB2_PHY_UTMI, },
	{ "utmi-wide", FSL_USB2_PHY_UTMI_WIDE, },
	{ "serial", FSL_USB2_PHY_SERIAL, },
};

static inline int imx_phy_of_get_phy_config(struct imx_phy_port *port,
					const char *name)
{
	int i;

	pr_debug("%s: phy-mode='%s'\n", __func__, name);
	for (i = 0; i < ARRAY_SIZE(imx_phy_modes); i++) {
		if (strcmp(name, imx_phy_modes[i].name) == 0) {
			port->phy_mode = imx_phy_modes[i].mode;
			pr_debug("%s: port %d: phy_mode=%08x\n",
				__func__, port->port_id, port->phy_mode);
			return 0;
		}
	}

	return -EINVAL;
}

static int __devinit imx_phy_add_ports(struct platform_device *pdev,
					struct imx_phy *imx_phy,
					const char *name, int port_id,
					unsigned int flags)
{
	int ret;
	int idx;
	struct device_node *np;

	if (port_id < 0)
		return port_id;

	np = pdev->dev.of_node;
	if (!np)
		return port_id;

	for (idx = 0; port_id < MAX_PORTS; idx++, port_id++) {
		struct imx_phy_port *port = &imx_phy->ports[port_id];
		struct usb_phy *phy = &port->phy;
		const char *phy_mode;
		const u32 *base;
		char devname[24];
		const char *devname2;
		struct device_node *dp;

		dp = of_parse_phandle(np, name, idx);
		if (!dp)
			break;

		if (!of_find_property(dp, "ignore-overcurrent", NULL))
			port->phy_flags |= MXC_EHCI_POWER_PINS_ENABLED;
		if (of_find_property(dp, "enable-wakeup", NULL))
			port->phy_flags |= MXC_EHCI_WAKEUP_ENABLED;
		if (of_get_property(dp, "itc-no-threshold", NULL))
			port->phy_flags |= MXC_EHCI_ITC_NO_THRESHOLD;

		base = of_get_property(dp, "reg", NULL);
		if (!base) {
			dev_err(&pdev->dev, "No 'reg' property found in '%s' node\n",
				name);
			of_node_put(dp);
			return -EINVAL;
		}

		ret = snprintf(devname, sizeof(devname), "%08x.imxotg",
			be32_to_cpu(*base));
		if (ret >= sizeof(devname)) {
			of_node_put(dp);
			return -EINVAL;
		}

		devname2 = kstrdup(devname, GFP_KERNEL);
		if (devname2 == NULL) {
			of_node_put(dp);
			return -ENOMEM;
		}

		if (flags & IMX_PHY_HOST)
			port->phy.dev_id_host = devname2;
		else
			port->phy.dev_id_peripheral = devname2;

		ret = of_property_read_string(dp, "phy-mode", &phy_mode);
		of_node_put(dp);
		if (ret) {
			ret = -EINVAL;
			goto err;
		}

		ret = imx_phy_of_get_phy_config(port, phy_mode);
		if (ret)
			goto err;

		phy->dev = &pdev->dev;
		phy->owner = THIS_MODULE;

		port->imx_phy = imx_phy;
		port->port_id = idx;
		port->phy.state = OTG_STATE_UNDEFINED;

		port->flags = flags;

		port->clk = clk_get_sys(devname, "phy");
		if (IS_ERR(port->clk)) {
			ret = PTR_ERR(port->clk);
			dev_err(&pdev->dev, "Failed to get PHY clock: %d\n",
				ret);
			goto err;
		}

		dev_dbg(phy->dev, "preparing PHY clk[%d] %p\n",
			port_id, port->clk);
		clk_prepare(port->clk);
		port->phy.init = imx_usb_phy_init;
		port->phy.dev = phy->dev;
		port->phy.shutdown = imx_phy_shutdown;
		port->phy.state = OTG_STATE_UNDEFINED;

		ATOMIC_INIT_NOTIFIER_HEAD(&port->phy.notifier);

		dev_dbg(phy->dev, "Adding %s.%d transceiver[%d] %p\n",
			name, idx, port_id, &port->phy);
		usb_add_transceiver(&port->phy);
	}
	return port_id;

err:
	if (flags & IMX_PHY_HOST)
		kfree(&imx_phy->ports[port_id].phy.dev_id_host);
	else
		kfree(&imx_phy->ports[port_id].phy.dev_id_peripheral);
	while (--port_id >= 0) {
		struct imx_phy_port *port = &imx_phy->ports[port_id];

		usb_remove_transceiver(&port->phy);
		if (flags & IMX_PHY_HOST)
			kfree(port->phy.dev_id_host);
		else
			kfree(port->phy.dev_id_peripheral);

		dev_dbg(port->phy.dev, "unpreparing PHY clk[%d] %p\n",
			port_id, port->clk);
		clk_unprepare(port->clk);
		clk_put(port->clk);
	}
	return ret;
}

static int __devinit imx_phy_probe(struct platform_device *pdev)
{
	int ret;
	struct imx_phy *imx_phy;
	struct resource *res;
	const struct of_device_id *of_id =
		of_match_device(imx_phy_dt_ids, &pdev->dev);

	imx_phy = devm_kzalloc(&pdev->dev, sizeof(*imx_phy), GFP_KERNEL);
	if (!imx_phy)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	imx_phy->usb_base = devm_ioremap(&pdev->dev, res->start,
				resource_size(res));
	if (!imx_phy->usb_base)
		return -ENOMEM;

	imx_phy->devdata = of_id ? of_id->data : &imx53_usbphy_devdata;

	ret = imx_phy_add_ports(pdev, imx_phy, "host-ports",
				0, IMX_PHY_HOST);
	ret = imx_phy_add_ports(pdev, imx_phy, "device-ports",
				ret, IMX_PHY_DEVICE);
	ret = imx_phy_add_ports(pdev, imx_phy, "otg-ports",
				ret, IMX_PHY_OTG);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, imx_phy);
	return 0;
}

static int imx_phy_remove(struct platform_device *pdev)
{
	struct imx_phy *imx_phy = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < MAX_PORTS; i++) {
		struct imx_phy_port *port = &imx_phy->ports[i];

		if (!port->phy.dev)
			continue;

		usb_remove_transceiver(&port->phy);
		if (port->flags & IMX_PHY_HOST)
			kfree(port->phy.dev_id_host);
		else
			kfree(port->phy.dev_id_peripheral);

		dev_dbg(port->phy.dev, "unpreparing PHY clk[%d] %p\n",
			i, port->clk);
		clk_unprepare(port->clk);
		clk_put(port->clk);
	}
	return 0;
}

static struct platform_driver imx_phy_driver = {
	.probe		= imx_phy_probe,
	.remove		= __devexit_p(imx_phy_remove),
	.driver		= {
		.name	= "imx-usb-phy",
		.owner	= THIS_MODULE,
		.of_match_table = imx_phy_dt_ids,
	},
};

module_platform_driver(imx_phy_driver);

MODULE_ALIAS("platform:imx-usb-phy");
MODULE_AUTHOR("Lothar Waßmann <LW@KARO-electronics.de>");
MODULE_DESCRIPTION("Freescale i.MX USB PHY driver");
MODULE_LICENSE("GPL");
