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
#include <mach/iomux-mx53.h>

#include "devices-imx53.h"

#define MX53_GPIO_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
				PAD_CTL_DSE_HIGH | PAD_CTL_PUS_22K_UP)

#define TX53_SDHC_PAD_CTRL      (PAD_CTL_HYS | PAD_CTL_DSE_HIGH |	\
				PAD_CTL_SRE_FAST | PAD_CTL_PUS_100K_DOWN)

static iomux_v3_cfg_t tx53_pads[] __initdata = {
	/* UART pads */
	MX53_PAD_PATA_DIOW__UART1_TXD_MUX,
	MX53_PAD_PATA_DMACK__UART1_RXD_MUX,
	MX53_PAD_PATA_IORDY__UART1_RTS,
	MX53_PAD_PATA_RESET_B__UART1_CTS,

	MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
	MX53_PAD_PATA_DMARQ__UART2_TXD_MUX,
	MX53_PAD_PATA_DIOR__UART2_RTS,
	MX53_PAD_PATA_INTRQ__UART2_CTS,

	MX53_PAD_PATA_CS_0__UART3_TXD_MUX,
	MX53_PAD_PATA_CS_1__UART3_RXD_MUX,
	MX53_PAD_PATA_DA_2__UART3_RTS,
	MX53_PAD_PATA_DA_1__UART3_CTS,

	/* I2C */
	NEW_PAD_CTRL(MX53_PAD_EIM_D28__I2C1_SDA, MX53_GPIO_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_EIM_D21__I2C1_SCL, MX53_GPIO_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_GPIO_6__I2C3_SDA, MX53_GPIO_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_GPIO_3__I2C3_SCL, MX53_GPIO_PAD_CTRL),

	/* (e)CSPI pads */
	MX53_PAD_EIM_D16__ECSPI1_SCLK,
	MX53_PAD_EIM_D17__ECSPI1_MISO,
	MX53_PAD_EIM_D18__ECSPI1_MOSI,

	/* eCSPI chip select lines */
	NEW_PAD_CTRL(MX53_PAD_EIM_EB2__GPIO2_30, MX53_GPIO_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_EIM_D19__GPIO3_19, MX53_GPIO_PAD_CTRL),

	/* SW controlled LED on STK5 baseboard */
	MX53_PAD_EIM_A18__GPIO2_20,

	/* eSDHC 1 */
	NEW_PAD_CTRL(MX53_PAD_SD1_CMD__ESDHC1_CMD, TX53_SDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_SD1_CLK__ESDHC1_CLK, TX53_SDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_SD1_DATA0__ESDHC1_DAT0, TX53_SDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_SD1_DATA1__ESDHC1_DAT1, TX53_SDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_SD1_DATA2__ESDHC1_DAT2, TX53_SDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_SD1_DATA3__ESDHC1_DAT3, TX53_SDHC_PAD_CTRL),
	/* SD1 CD */
	NEW_PAD_CTRL(MX53_PAD_EIM_D24__GPIO3_24, MX53_GPIO_PAD_CTRL),

	/* eSDHC 2 */
	NEW_PAD_CTRL(MX53_PAD_SD2_CMD__ESDHC2_CMD, TX53_SDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_SD2_CLK__ESDHC2_CLK, TX53_SDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_SD2_DATA0__ESDHC2_DAT0, TX53_SDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_SD2_DATA1__ESDHC2_DAT1, TX53_SDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_SD2_DATA2__ESDHC2_DAT2, TX53_SDHC_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_SD2_DATA3__ESDHC2_DAT3, TX53_SDHC_PAD_CTRL),
	/* SD2 CD */
	NEW_PAD_CTRL(MX53_PAD_EIM_D25__GPIO3_25, MX53_GPIO_PAD_CTRL),

	/* FEC */
	MX53_PAD_EIM_D20__GPIO3_20,
	MX53_PAD_PATA_DA_0__GPIO7_6,
	MX53_PAD_PATA_DATA4__GPIO2_4,
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,

	/* TSC200x PEN IRQ */
	NEW_PAD_CTRL(MX53_PAD_EIM_D26__GPIO3_26, MX53_GPIO_PAD_CTRL),

	/* EDT-FT5x06 Polytouch panel */
	NEW_PAD_CTRL(MX53_PAD_NANDF_CS2__GPIO6_15, MX53_GPIO_PAD_CTRL), /* IRQ */
	NEW_PAD_CTRL(MX53_PAD_EIM_A16__GPIO2_22, MX53_GPIO_PAD_CTRL), /* RESET */
	NEW_PAD_CTRL(MX53_PAD_EIM_A17__GPIO2_21, MX53_GPIO_PAD_CTRL), /* WAKE */

	/* LCD RESET */
	NEW_PAD_CTRL(MX53_PAD_EIM_D29__GPIO3_29, MX53_GPIO_PAD_CTRL),
	/* LCD POWER_ENABLE */
	NEW_PAD_CTRL(MX53_PAD_EIM_EB3__GPIO2_31, MX53_GPIO_PAD_CTRL),
	/* LCD Backlight (PWM) */
	NEW_PAD_CTRL(MX53_PAD_GPIO_1__GPIO1_1, MX53_GPIO_PAD_CTRL),
	/* DISPLAY */
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,

	/* USB Host */
	NEW_PAD_CTRL(MX53_PAD_EIM_D31__GPIO3_31, MX53_GPIO_PAD_CTRL), /* VBUSEN */
	NEW_PAD_CTRL(MX53_PAD_EIM_D30__GPIO3_30, MX53_GPIO_PAD_CTRL), /* OC */

	MX53_PAD_GPIO_7__GPIO1_7, /* VBUSEN */
	MX53_PAD_GPIO_8__GPIO1_8, /* OC */

	/* Flexcan */
	MX53_PAD_KEY_COL4__CAN2_TXCAN,
	MX53_PAD_KEY_ROW4__CAN2_RXCAN,

	/* Keypad */
	NEW_PAD_CTRL(MX53_PAD_GPIO_9__KPP_COL_6, MX53_GPIO_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_GPIO_4__KPP_COL_7, MX53_GPIO_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_KEY_COL2__KPP_COL_2, MX53_GPIO_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_KEY_COL3__KPP_COL_3, MX53_GPIO_PAD_CTRL),

	NEW_PAD_CTRL(MX53_PAD_GPIO_2__KPP_ROW_6, MX53_GPIO_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_GPIO_5__KPP_ROW_7, MX53_GPIO_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_KEY_ROW2__KPP_ROW_2, MX53_GPIO_PAD_CTRL),
	NEW_PAD_CTRL(MX53_PAD_KEY_ROW3__KPP_ROW_3, MX53_GPIO_PAD_CTRL),

	/* Audio */
	MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,

	/* DS1339 Interrupt */
	NEW_PAD_CTRL(MX53_PAD_DI0_PIN4__GPIO4_20, MX53_GPIO_PAD_CTRL),
};

static int __init tx53_read_mac(unsigned char addr[ETH_ALEN])
{
	int ret;
	int i;
	void __iomem *ioaddr;
	const unsigned long fec_mac_base = MX53_IIM_BASE_ADDR + 0xc24;
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
#define MX53_UUID_ADDR		(MX53_IIM_BASE_ADDR + 0x820)
#define MX53_UUID_LEN		8

static void __init tx53_set_system_serial(void)
{
	void __iomem *mx53_serial_addr = ioremap(MX53_UUID_ADDR, MX53_UUID_LEN);
	struct clk *iim_clk;

	if (mx53_serial_addr == NULL) {
		pr_warning("Failed to map MX53_UUID_ADDR; cannot set system serial number\n");
		return;
	}

	iim_clk = clk_get(NULL, "iim_clk");
	if (IS_ERR(iim_clk)) {
		pr_err("%s: Failed to get IIM clock: %ld\n", __func__,
			PTR_ERR(iim_clk));
		iounmap(mx53_serial_addr);
		return;
	}

	if (clk_enable(iim_clk) == 0) {
		int i, n;
		unsigned int __iomem *p = mx53_serial_addr;

		for (i = n = 0; i < sizeof(system_serial_low) &&
			     n < MX53_UUID_LEN; i++, n++, p++) {
			system_serial_low |= readl(p) << (i * 8);
		}
		for (i = 0; i < sizeof(system_serial_high) &&
			     n < MX53_UUID_LEN; i++, n++, p++) {
			system_serial_high |= readl(p) << (i * 8);
		}
	} else {
		pr_err("Failed to enable IIM clock\n");
	}
	clk_disable(iim_clk);
	clk_put(iim_clk);
	iounmap(mx53_serial_addr);
}

enum {
	TX53_OTG_MODE_NONE,
	TX53_OTG_MODE_HOST,
	TX53_OTG_MODE_DEVICE,
};

static int tx53_otg_mode __initdata;

static iomux_v3_cfg_t tx53_flexcan1_pads[] = {
	MX53_PAD_GPIO_7__CAN1_TXCAN,
	MX53_PAD_GPIO_8__CAN1_RXCAN,
};

static iomux_v3_cfg_t tx53_usbotg_pads[] = {
	MX53_PAD_GPIO_7__GPIO1_7,
	MX53_PAD_GPIO_8__GPIO1_8,
};

static void __init tx53_update_dt(void)
{
	struct device_node *np;

	for_each_compatible_node(np, NULL, "fsl,imx-otg") {
		struct property *prop;

		prop = of_find_property(np, "gadget-device-name", NULL);
		if (!prop)
			continue;

		if (tx53_otg_mode != TX53_OTG_MODE_HOST) {
			prop = of_find_property(np, "host-device-name", NULL);
			if (prop) {
				prom_remove_property(np, prop);
			} else {
				pr_err("'host-device-name' property not found in node %s[%08x]\n",
					np->name, np->phandle);
			}
		}
		if (tx53_otg_mode != TX53_OTG_MODE_DEVICE) {
			prom_remove_property(np, prop);
		}
	}
}

/* baseboard specific setup called via board_initfunc hook */
static void __init tx53_stk5v3_board_init(void)
{
	pr_info("Setting up STK5-V3\n");

	if (tx53_otg_mode != TX53_OTG_MODE_HOST) {
		mxc_iomux_v3_setup_multiple_pads(tx53_flexcan1_pads,
				ARRAY_SIZE(tx53_flexcan1_pads));
	} else {
		mxc_iomux_v3_setup_multiple_pads(tx53_usbotg_pads,
				ARRAY_SIZE(tx53_usbotg_pads));
	}
}

static void __init tx53_stk5v5_board_init(void)
{
	pr_info("Setting up STK5-V5\n");

	if (tx53_otg_mode == TX53_OTG_MODE_HOST) {
		pr_warning("Host mode not supported on USBOTG port\n");
		tx53_otg_mode = TX53_OTG_MODE_NONE;
	}

	/* CAN transceiver enable */
	mxc_iomux_v3_setup_pad(MX53_PAD_DISP0_DAT0__GPIO4_21);
	mxc_iomux_v3_setup_multiple_pads(tx53_flexcan1_pads,
					ARRAY_SIZE(tx53_flexcan1_pads));

}

static void (*board_initfunc)(void) __initdata;

static int __init tx53_select_base(/* const */ char *options)
{
	if (strcmp(options, "stkv3") == 0)
		board_initfunc = tx53_stk5v3_board_init;
	else if (strcmp(options, "stkv5") == 0)
		board_initfunc = tx53_stk5v5_board_init;

	return 0;
}
__setup("tx53_base=", tx53_select_base);

static int __init tx53_select_otg_mode(/* const */ char *options)
{
	if (strcmp(options, "host") == 0)
		tx53_otg_mode = TX53_OTG_MODE_HOST;
	else if (strcmp(options, "device") == 0)
		tx53_otg_mode = TX53_OTG_MODE_DEVICE;
	else if (strcmp(options, "off") != 0)
		return -EINVAL;
	return 0;
}
__setup("otg_mode=", tx53_select_otg_mode);

/* LCD */
static int __init tx53_lcd_of_init(void)
{
	if (!of_machine_is_compatible("karo,tx53"))
		return 0;

#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
	mxc_iomux_v3_setup_pad(MX53_PAD_GPIO_1__PWM2_PWMO);
#endif
	return 0;
}
late_initcall(tx53_lcd_of_init);

static int __init tx53_consistent_dma_init(void)
{
	init_consistent_dma_size(SZ_8M);
	return 0;
}
early_initcall(tx53_consistent_dma_init);

void __init tx53_common_init(void)
{
	tx53_set_system_serial();
	mxc_iomux_v3_setup_multiple_pads(tx53_pads,
					ARRAY_SIZE(tx53_pads));

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

static int __init tx53_of_mac_addr_init(void)
{
	int ret;
	char mac_addr[ETH_ALEN];
	struct device_node *np;
	struct property *old_mac;
	struct property *new_mac;

	ret = tx53_read_mac(mac_addr);
	if (ret)
		return ret;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx53-fec");
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

void __init tx53_of_init(void)
{
	imx_set_aips(MX53_IO_ADDRESS(MX53_AIPS1_BASE_ADDR));
	imx_set_aips(MX53_IO_ADDRESS(MX53_AIPS2_BASE_ADDR));

	tx53_common_init();
	tx53_of_mac_addr_init();

	pr_info("OTG mode is '%s'\n",
		tx53_otg_mode == TX53_OTG_MODE_HOST ? "host" :
		tx53_otg_mode == TX53_OTG_MODE_DEVICE ? "device" :
		"unknown");

	if (board_initfunc) {
		board_initfunc();
	}
	tx53_update_dt();
}
