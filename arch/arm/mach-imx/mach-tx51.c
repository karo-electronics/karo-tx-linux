/*
 * Copyright (C) 2011 Lothar Waßmann <LW@KARO-electronics.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/fec.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/i2c/tsc2007.h>
#include <linux/pwm_backlight.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#include <drm/drmP.h>
#include "drm/drm_encon.h"

#include <video/platform_lcd.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/system_info.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/imx-uart.h>
#include <mach/iomux-mx51.h>

#include "devices-imx51.h"

#define MUX_MODE_SION		((iomux_v3_cfg_t)IOMUX_CONFIG_SION << MUX_MODE_SHIFT)

static iomux_v3_cfg_t tx51_pads[] __initdata = {
	/* UART pads */
	MX51_PAD_UART1_RXD__UART1_RXD,
	MX51_PAD_UART1_TXD__UART1_TXD,
	MX51_PAD_UART1_RTS__UART1_RTS,
	MX51_PAD_UART1_CTS__UART1_CTS,

	MX51_PAD_UART2_RXD__UART2_RXD,
	MX51_PAD_UART2_TXD__UART2_TXD,
	MX51_PAD_EIM_D26__UART2_RTS,
	MX51_PAD_EIM_D25__UART2_CTS,

	MX51_PAD_UART3_RXD__UART3_RXD,
	MX51_PAD_UART3_TXD__UART3_TXD,
	MX51_PAD_EIM_D18__UART3_RTS,
	MX51_PAD_EIM_D17__UART3_CTS,

	/* I2C */
	MX51_PAD_EIM_D24__I2C2_SDA,
	MX51_PAD_EIM_D27__I2C2_SCL,

	MX51_PAD_I2C1_DAT__GPIO4_17 | MUX_MODE_SION,
	MX51_PAD_I2C1_CLK__GPIO4_16 | MUX_MODE_SION,

	/* (e)CSPI pads */
	MX51_PAD_CSPI1_MOSI__ECSPI1_MOSI,
	MX51_PAD_CSPI1_MISO__ECSPI1_MISO,
	MX51_PAD_CSPI1_SCLK__ECSPI1_SCLK,
	MX51_PAD_CSPI1_RDY__ECSPI1_RDY,
	/* eCSPI chip select lines */
	MX51_PAD_CSPI1_SS0__GPIO4_24,
	MX51_PAD_CSPI1_SS1__GPIO4_25,

	/* SW controlled LED on STK5 baseboard */
	MX51_PAD_CSI2_D13__GPIO4_10,

	/* eSDHC 1 */
	MX51_PAD_SD1_CMD__SD1_CMD,
	MX51_PAD_SD1_CLK__SD1_CLK,
	MX51_PAD_SD1_DATA0__SD1_DATA0,
	MX51_PAD_SD1_DATA1__SD1_DATA1,
	MX51_PAD_SD1_DATA2__SD1_DATA2,
	MX51_PAD_SD1_DATA3__SD1_DATA3,
	/* SD1 CD */
	MX51_PAD_DISPB2_SER_RS__GPIO3_8,

	/* eSDHC 2 */
	MX51_PAD_SD2_CMD__SD2_CMD,
	MX51_PAD_SD2_CLK__SD2_CLK,
	MX51_PAD_SD2_DATA0__SD2_DATA0,
	MX51_PAD_SD2_DATA1__SD2_DATA1,
	MX51_PAD_SD2_DATA2__SD2_DATA2,
	MX51_PAD_SD2_DATA3__SD2_DATA3,
	/* SD2 CD */
	MX51_PAD_DISPB2_SER_DIO__GPIO3_6,

	/* TSC200x PEN IRQ */
	MX51_PAD_DI1_D0_CS__GPIO3_3,

	/* LCD RESET */
	MX51_PAD_CSI2_VSYNC__GPIO4_13,
	/* LCD POWER_ENABLE */
	MX51_PAD_CSI2_HSYNC__GPIO4_14,
	/* LCD Backlight (PWM) */
	MX51_PAD_GPIO1_2__GPIO1_2,
	/* DISPLAY */

	/* USB Host */
	MX51_PAD_USBH1_CLK__USBH1_CLK,
	MX51_PAD_USBH1_DIR__USBH1_DIR,
	MX51_PAD_USBH1_STP__USBH1_STP,
	MX51_PAD_USBH1_NXT__USBH1_NXT,
	MX51_PAD_USBH1_DATA0__USBH1_DATA0,
	MX51_PAD_USBH1_DATA1__USBH1_DATA1,
	MX51_PAD_USBH1_DATA2__USBH1_DATA2,
	MX51_PAD_USBH1_DATA3__USBH1_DATA3,
	MX51_PAD_USBH1_DATA4__USBH1_DATA4,
	MX51_PAD_USBH1_DATA5__USBH1_DATA5,
	MX51_PAD_USBH1_DATA6__USBH1_DATA6,
	MX51_PAD_USBH1_DATA7__USBH1_DATA7,
	MX51_PAD_GPIO1_6__GPIO1_6, /* OC */
	MX51_PAD_GPIO1_4__GPIO1_4, /* PHY RESET */
	MX51_PAD_GPIO1_7__GPIO1_7, /* PHY CLK ENABLE */

	/* USBOTG */
	MX51_PAD_GPIO1_9__GPIO1_9, /* OC */
	MX51_PAD_GPIO1_8__GPIO1_8, /* VBUSEN */

	/* Keypad */
	MX51_PAD_KEY_ROW0__KEY_ROW0,
	MX51_PAD_KEY_ROW1__KEY_ROW1,
	MX51_PAD_KEY_ROW2__KEY_ROW2,
	MX51_PAD_KEY_ROW3__KEY_ROW3,

	MX51_PAD_KEY_COL1__KEY_COL1,
	MX51_PAD_KEY_COL2__KEY_COL2,
	MX51_PAD_KEY_COL3__KEY_COL3,
	MX51_PAD_KEY_COL4__KEY_COL4,
	MX51_PAD_KEY_COL5__KEY_COL5,

	/* Audio */
	MX51_PAD_AUD3_BB_RXD__AUD3_RXD,
	MX51_PAD_AUD3_BB_TXD__AUD3_TXD,
	MX51_PAD_AUD3_BB_FS__AUD3_TXFS,
	MX51_PAD_AUD3_BB_CK__AUD3_TXC,

	/* FEC */
	MX51_PAD_EIM_A20__GPIO2_14, /* PHY RESET */
	MX51_PAD_GPIO1_3__GPIO1_3,  /* PHY POWER */
	MX51_PAD_NANDF_CS2__GPIO3_18, /* PHY INT */

	/* unuseable pads configured as GPIO */
	MX51_PAD_GPIO1_1__GPIO1_1,
	MX51_PAD_GPIO1_0__GPIO1_0,
};

static int __init tx51_read_mac(unsigned char addr[ETH_ALEN])
{
	int ret;
	int i;
	void __iomem *ioaddr;
	const unsigned long fec_mac_base = MX51_IIM_BASE_ADDR + 0xc24;
	struct clk *iim_clk;

	iim_clk = clk_get(NULL, "iim_clk");
	if (IS_ERR(iim_clk)) {
		pr_err("%s: Failed to get IIM clock\n", __func__);
		return PTR_ERR(iim_clk);
	}
	ret = clk_enable(iim_clk);
	if (ret) {
		pr_err("%s: Failed to enable IIM clock: %d\n",
			__func__, ret);
		goto put_clk;
	}

	ioaddr = ioremap(fec_mac_base, ETH_ALEN * sizeof(long));
	if (ioaddr == NULL) {
		ret = -ENOMEM;
		goto disable_clk;
	}
	for (i = 0; i < ETH_ALEN; i++)
		addr[i] = __raw_readl(ioaddr + i * 4);
	iounmap(ioaddr);
disable_clk:
	clk_disable(iim_clk);
put_clk:
	clk_put(iim_clk);
	return ret;
}

/* set system_serial from UUID fuses */
#define MX51_UUID_ADDR		(MX51_IIM_BASE_ADDR + 0x820)
#define MX51_UUID_LEN		8

static void __init tx51_set_system_serial(void)
{
	void __iomem *mx51_serial_addr = ioremap(MX51_UUID_ADDR, MX51_UUID_LEN);
	struct clk *iim_clk;

	if (mx51_serial_addr == NULL) {
		pr_warning("Failed to map MX51_UUID_ADDR; cannot set system serial number\n");
		return;
	}

	iim_clk = clk_get(NULL, "iim_clk");
	if (IS_ERR(iim_clk)) {
		pr_err("%s: Failed to get IIM clock: %ld\n", __func__,
			PTR_ERR(iim_clk));
		iounmap(mx51_serial_addr);
		return;
	}

	if (clk_enable(iim_clk) == 0) {
		int i, n;
		unsigned int __iomem *p = mx51_serial_addr;

		for (i = n = 0; i < sizeof(system_serial_low) &&
			     n < MX51_UUID_LEN; i++, n++, p++) {
			system_serial_low |= readl(p) << (i * 8);
		}
		for (i = 0; i < sizeof(system_serial_high) &&
			     n < MX51_UUID_LEN; i++, n++, p++) {
			system_serial_high |= readl(p) << (i * 8);
		}
	} else {
		pr_err("Failed to enable IIM clock\n");
	}
	clk_disable(iim_clk);
	clk_put(iim_clk);
	iounmap(mx51_serial_addr);
}

enum {
	TX51_OTG_MODE_NONE,
	TX51_OTG_MODE_HOST,
	TX51_OTG_MODE_DEVICE,
};

static int tx51_otg_mode __initdata;

static void __init tx51_update_dt(void)
{
	struct device_node *np;

	for_each_compatible_node(np, NULL, "fsl,imx-otg") {
		struct property *prop;

		prop = of_find_property(np, "gadget-device-name", NULL);
		if (!prop)
			continue;

		if (tx51_otg_mode != TX51_OTG_MODE_HOST) {
			prop = of_find_property(np, "host-device-name", NULL);
			if (prop) {
				prom_remove_property(np, prop);
			} else {
				pr_err("'host-device-name' property not found in node %s[%08x]\n",
					np->name, np->phandle);
			}
		}
		if (tx51_otg_mode != TX51_OTG_MODE_DEVICE) {
			prom_remove_property(np, prop);
		}
	}
}

/* baseboard specific setup called via board_initfunc hook */
static void __init tx51_stk5v3_board_init(void)
{
	pr_info("Setting up STK5-V3\n");
}

static void __init tx51_stk5v5_board_init(void)
{
	pr_info("Setting up STK5-V5\n");

	if (tx51_otg_mode == TX51_OTG_MODE_HOST) {
		pr_warning("Host mode not supported on USBOTG port\n");
		tx51_otg_mode = TX51_OTG_MODE_NONE;
	}
}

static void (*board_initfunc)(void) __initdata;

static int __init tx51_select_base(/* const */ char *options)
{
	if (strcmp(options, "stkv3") == 0)
		board_initfunc = tx51_stk5v3_board_init;
	else if (strcmp(options, "stkv5") == 0)
		board_initfunc = tx51_stk5v5_board_init;

	return 0;
}
__setup("tx51_base=", tx51_select_base);

static int __init tx51_select_otg_mode(/* const */ char *options)
{
	if (strcmp(options, "host") == 0)
		tx51_otg_mode = TX51_OTG_MODE_HOST;
	else if (strcmp(options, "device") == 0)
		tx51_otg_mode = TX51_OTG_MODE_DEVICE;
	else if (strcmp(options, "off") != 0)
		return -EINVAL;
	return 0;
}
__setup("otg_mode=", tx51_select_otg_mode);

/* LCD */
static struct clk *ipu_clk;

static int __init tx51_lcd_of_init(void)
{
	if (!of_machine_is_compatible("karo,tx51"))
		return 0;

	/* See note below! */
	clk_disable(ipu_clk);

#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
	mxc_iomux_v3_setup_pad(MX51_PAD_GPIO1_2__PWM1_PWMO);
#endif
	return 0;
}
late_initcall(tx51_lcd_of_init);

static int __init tx51_consistent_dma_init(void)
{
	init_consistent_dma_size(SZ_8M);
	return 0;
}
early_initcall(tx51_consistent_dma_init);

void __init tx51_common_init(void)
{
	/*
	 * FIXME:
	 * The system locks up when the IPU driver is allowed to disable
	 * the IPU clock during boot.
	 * Interestingly, on the TX53, which uses the same driver,
	 * this is not needed!
	 */
	ipu_clk = clk_get_sys("imx-ipuv3", NULL);
	if (!IS_ERR(ipu_clk)) {
		clk_enable(ipu_clk);
	} else {
		printk(KERN_ERR "Failed to get IPU clock: %ld\n",
			PTR_ERR(ipu_clk));
	}

	tx51_set_system_serial();
	mxc_iomux_v3_setup_multiple_pads(tx51_pads,
					ARRAY_SIZE(tx51_pads));

#ifdef CONFIG_DRM_KMS_ENCON
	drm_encon_add_dummy("imx-drm.0", 0);
	drm_encon_add_dummy("imx-drm.0", 1);
#endif
}

static inline struct property *new_property(const char *name, const int length,
					const unsigned char *value)
{
	struct property *new = kzalloc(sizeof(*new), GFP_KERNEL);

	if (!new)
		return NULL;

	new->name = kstrdup(name, GFP_KERNEL);
	if (!new->name)
		goto cleanup;
	new->value = kzalloc(length + 1, GFP_KERNEL);
	if (!new->value)
		goto cleanup;

	memcpy(new->value, value, length);
	new->length = length;
	return new;

cleanup:
	kfree(new->name);
	kfree(new->value);
	kfree(new);
	return NULL;
}

static int __init tx51_of_mac_addr_init(void)
{
	int ret;
	char mac_addr[ETH_ALEN];
	struct device_node *np;
	struct property *old_mac;
	struct property *new_mac;

	ret = tx51_read_mac(mac_addr);
	if (ret)
		return ret;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx51-fec");
	if (!np)
		return -EINVAL;

	new_mac = new_property("mac_addr", sizeof(mac_addr), mac_addr);
	if (!new_mac)
		return -ENOMEM;

	old_mac = of_find_property(np, "mac-address", NULL);
	if (old_mac) {
		pr_debug("Found 'mac_addr' property @ %p value @ %p\n",
			old_mac, old_mac->value);
		prom_update_property(np, new_mac, old_mac);
	} else {
		pr_warning("Adding 'mac-address' property\n");
		prom_add_property(np, new_mac);
	}
	pr_info("mac_addr set to [%02x:%02x:%02x:%02x:%02x:%02x]\n",
		mac_addr[0], mac_addr[1], mac_addr[2],
		mac_addr[3], mac_addr[4], mac_addr[5]);
	return 0;
}

void __init tx51_of_init(void)
{
	imx_set_aips(MX51_IO_ADDRESS(MX51_AIPS1_BASE_ADDR));
	imx_set_aips(MX51_IO_ADDRESS(MX51_AIPS2_BASE_ADDR));

	tx51_common_init();
	tx51_of_mac_addr_init();

	pr_info("OTG mode is '%s'\n",
		tx51_otg_mode == TX51_OTG_MODE_HOST ? "host" :
		tx51_otg_mode == TX51_OTG_MODE_DEVICE ? "device" :
		"unknown");

	if (board_initfunc) {
		board_initfunc();
	}
	tx51_update_dt();
}
