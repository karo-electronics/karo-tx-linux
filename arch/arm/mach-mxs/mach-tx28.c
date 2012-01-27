/*
 * Copyright (C) 2010 <LW@KARO-electronics.de>
 *
 * based on: mach-mx28_evk.c
 * Copyright 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation
 */
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/i2c.h>
#include <linux/pwm_backlight.h>
#include <linux/memblock.h>
#include <linux/delay.h>

#include <linux/i2c/tsc2007.h>
#include <linux/i2c/pca953x.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mtd/partitions.h>

#include <video/platform_lcd.h>
#include <linux/input/edt-ft5x06.h>

#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <mach/common.h>
#include <mach/iomux-mx28.h>

#include "devices-mx28.h"
#include "module-tx28.h"

#define TX28_STK5_GPIO_LED		MXS_GPIO_NR(4, 10)
#define TX28_STK5_GPIO_USBHOST_VBUSEN	MXS_GPIO_NR(3, 27)
#define TX28_STK5_GPIO_USBOTG_VBUSEN	MXS_GPIO_NR(0, 18)
#define TX28_STK5_GPIO_FLEXCAN_XCVR_EN	MXS_GPIO_NR(1, 0)
#define TX28_STK5_GPIO_BACKLIGHT	MXS_GPIO_NR(3, 16)
#define TX28_STK5_GPIO_LCD_ENABLE	MXS_GPIO_NR(1, 31)
#define TX28_STK5_GPIO_LCD_RESET	MXS_GPIO_NR(3, 30)
#define TX28_STK5_GPIO_EDT_IRQ		MXS_GPIO_NR(2, 5)
#define TX28_STK5_GPIO_EDT_RESET	MXS_GPIO_NR(2, 6)

#define TX28_GPIO_IN_PAD_CTRL		(MXS_PAD_3V3 | MXS_PAD_4MA | \
					MXS_PAD_PULLUP)

static const iomux_cfg_t tx28_stk5v3_pads[] __initconst = {
	/* LED */
	MX28_PAD_ENET0_RXD3__GPIO_4_10 |
		MXS_PAD_3V3 | MXS_PAD_4MA | MXS_PAD_NOPULL,

	/* framebuffer */
#define LCD_MODE (MXS_PAD_3V3 | MXS_PAD_4MA)
	MX28_PAD_LCD_D00__LCD_D0 | LCD_MODE,
	MX28_PAD_LCD_D01__LCD_D1 | LCD_MODE,
	MX28_PAD_LCD_D02__LCD_D2 | LCD_MODE,
	MX28_PAD_LCD_D03__LCD_D3 | LCD_MODE,
	MX28_PAD_LCD_D04__LCD_D4 | LCD_MODE,
	MX28_PAD_LCD_D05__LCD_D5 | LCD_MODE,
	MX28_PAD_LCD_D06__LCD_D6 | LCD_MODE,
	MX28_PAD_LCD_D07__LCD_D7 | LCD_MODE,
	MX28_PAD_LCD_D08__LCD_D8 | LCD_MODE,
	MX28_PAD_LCD_D09__LCD_D9 | LCD_MODE,
	MX28_PAD_LCD_D10__LCD_D10 | LCD_MODE,
	MX28_PAD_LCD_D11__LCD_D11 | LCD_MODE,
	MX28_PAD_LCD_D12__LCD_D12 | LCD_MODE,
	MX28_PAD_LCD_D13__LCD_D13 | LCD_MODE,
	MX28_PAD_LCD_D14__LCD_D14 | LCD_MODE,
	MX28_PAD_LCD_D15__LCD_D15 | LCD_MODE,
	MX28_PAD_LCD_D16__LCD_D16 | LCD_MODE,
	MX28_PAD_LCD_D17__LCD_D17 | LCD_MODE,
	MX28_PAD_LCD_D18__LCD_D18 | LCD_MODE,
	MX28_PAD_LCD_D19__LCD_D19 | LCD_MODE,
	MX28_PAD_LCD_D20__LCD_D20 | LCD_MODE,
	MX28_PAD_LCD_D21__LCD_D21 | LCD_MODE,
	MX28_PAD_LCD_D22__LCD_D22 | LCD_MODE,
	MX28_PAD_LCD_D23__LCD_D23 | LCD_MODE,
	MX28_PAD_LCD_RD_E__LCD_VSYNC | LCD_MODE,
	MX28_PAD_LCD_WR_RWN__LCD_HSYNC | LCD_MODE,
	MX28_PAD_LCD_RS__LCD_DOTCLK | LCD_MODE,
	MX28_PAD_LCD_CS__LCD_CS | LCD_MODE,
	MX28_PAD_LCD_VSYNC__LCD_VSYNC | LCD_MODE,
	MX28_PAD_LCD_HSYNC__LCD_HSYNC | LCD_MODE,
	MX28_PAD_LCD_DOTCLK__LCD_DOTCLK | LCD_MODE,
	MX28_PAD_LCD_ENABLE__GPIO_1_31 | LCD_MODE,
	MX28_PAD_LCD_RESET__GPIO_3_30 | LCD_MODE,
	MX28_PAD_PWM0__GPIO_3_16 | LCD_MODE,

	/* UART1 */
	MX28_PAD_AUART0_CTS__DUART_RX,
	MX28_PAD_AUART0_RTS__DUART_TX,
	MX28_PAD_AUART0_TX__DUART_RTS,
	MX28_PAD_AUART0_RX__DUART_CTS,

	/* UART2 */
	MX28_PAD_AUART1_RX__AUART1_RX,
	MX28_PAD_AUART1_TX__AUART1_TX,
	MX28_PAD_AUART1_RTS__AUART1_RTS,
	MX28_PAD_AUART1_CTS__AUART1_CTS,

	/* UART3 */
	MX28_PAD_AUART3_RX__AUART3_RX,
	MX28_PAD_AUART3_TX__AUART3_TX,
	MX28_PAD_AUART3_RTS__AUART3_RTS,
	MX28_PAD_AUART3_CTS__AUART3_CTS,

	/* CAN */
	MX28_PAD_GPMI_RDY2__CAN0_TX,
	MX28_PAD_GPMI_RDY3__CAN0_RX,

	/* I2C */
	MX28_PAD_I2C0_SCL__I2C0_SCL,
	MX28_PAD_I2C0_SDA__I2C0_SDA,

	/* TSC2007 */
	MX28_PAD_SAIF0_MCLK__GPIO_3_20 | TX28_GPIO_IN_PAD_CTRL,

	/* MMC0 */
	MX28_PAD_SSP0_DATA0__SSP0_D0 |
		(MXS_PAD_8MA | MXS_PAD_3V3 | MXS_PAD_PULLUP),
	MX28_PAD_SSP0_DATA1__SSP0_D1 |
		(MXS_PAD_8MA | MXS_PAD_3V3 | MXS_PAD_PULLUP),
	MX28_PAD_SSP0_DATA2__SSP0_D2 |
		(MXS_PAD_8MA | MXS_PAD_3V3 | MXS_PAD_PULLUP),
	MX28_PAD_SSP0_DATA3__SSP0_D3 |
		(MXS_PAD_8MA | MXS_PAD_3V3 | MXS_PAD_PULLUP),
	MX28_PAD_SSP0_CMD__SSP0_CMD |
		(MXS_PAD_8MA | MXS_PAD_3V3 | MXS_PAD_PULLUP),
	MX28_PAD_SSP0_DETECT__SSP0_CARD_DETECT |
		(MXS_PAD_8MA | MXS_PAD_3V3 | MXS_PAD_NOPULL),
	MX28_PAD_SSP0_SCK__SSP0_SCK |
		(MXS_PAD_12MA | MXS_PAD_3V3 | MXS_PAD_NOPULL),

	/* SAIF0 */
	MX28_PAD_SAIF0_LRCLK__SAIF0_LRCLK,
	MX28_PAD_SAIF0_BITCLK__SAIF0_BITCLK,
	MX28_PAD_SAIF0_SDATA0__SAIF0_SDATA0,
	MX28_PAD_SAIF1_SDATA0__SAIF0_SDATA1,

	/* USB Host */
	MX28_PAD_SPDIF__GPIO_3_27 | MXS_PAD_CTRL,	/* USB host vbusen */
	MX28_PAD_GPMI_CE2N__GPIO_0_18 | MXS_PAD_CTRL,	/* USB otg vbusen */
	MX28_PAD_PWM2__USB0_ID |
		MXS_PAD_3V3 | MXS_PAD_4MA | MXS_PAD_PULLUP,

	/* EDT Touchscreen */
	MX28_PAD_SSP0_DATA5__GPIO_2_5 |
		MXS_PAD_3V3 | MXS_PAD_4MA | MXS_PAD_PULLUP,
	MX28_PAD_SSP0_DATA6__GPIO_2_6 |
		MXS_PAD_3V3 | MXS_PAD_4MA | MXS_PAD_PULLUP,
};

static const struct gpio_led tx28_stk5v3_leds[] __initconst = {
	{
		.name = "GPIO-LED",
		.default_trigger = "heartbeat",
		.gpio = TX28_STK5_GPIO_LED,
	},
};

static const struct gpio_led_platform_data tx28_stk5v3_led_data __initconst = {
	.leds = tx28_stk5v3_leds,
	.num_leds = ARRAY_SIZE(tx28_stk5v3_leds),
};

/* SPI */
static struct spi_board_info tx28_spi_board_info[] = {
	{
		.modalias = "spidev",
		.max_speed_hz = 20000000,
		.bus_num = 0,
		.chip_select = 0,
		.controller_data = (void *)MXS_GPIO_NR(2, 27),
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 20000000,
		.bus_num = 0,
		.chip_select = 1,
		.controller_data = (void *)MXS_GPIO_NR(3, 8),
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 20000000,
		.bus_num = 0,
		.chip_select = 2,
		.controller_data = (void *)MXS_GPIO_NR(3, 9),
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 20000000,
		.bus_num = 1,
		.chip_select = 0,
		.controller_data = (void *)MXS_GPIO_NR(2, 19),
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 20000000,
		.bus_num = 1,
		.chip_select = 1,
		.controller_data = (void *)MXS_GPIO_NR(2, 20),
		.mode = SPI_MODE_0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 20000000,
		.bus_num = 1,
		.chip_select = 2,
		.controller_data = (void *)MXS_GPIO_NR(2, 21),
		.mode = SPI_MODE_0,
	},
};

/* SPI port 1 via SPI_GPIO */
static const iomux_cfg_t tx28_spi_gpio0_pads[] __initconst = {
	MX28_PAD_SSP3_SCK__GPIO_2_24,	/* SCK */
	MX28_PAD_SSP3_MOSI__GPIO_2_25,	/* MOSI */
	MX28_PAD_SSP3_MISO__GPIO_2_26,	/* MISO */
	MX28_PAD_SSP3_SS0__GPIO_2_27,	/* SS0 */
	MX28_PAD_AUART2_RX__GPIO_3_8,	/* SS1 */
	MX28_PAD_AUART2_TX__GPIO_3_9,	/* SS2 */
};

static struct spi_gpio_platform_data tx28_spi_gpio0_data = {
	.miso = MXS_GPIO_NR(2, 26),
	.mosi = MXS_GPIO_NR(2, 25),
	.sck = MXS_GPIO_NR(2, 24),
	.num_chipselect = 3,
};

static struct platform_device tx28_spi_gpio0_device = {
	.name = "spi_gpio",
	.id = 0,
	.dev = {
		.platform_data = &tx28_spi_gpio0_data,
	},
};

/* SPI port 2 via SPI_GPIO */
static const iomux_cfg_t tx28_spi_gpio1_pads[] __initconst = {
	MX28_PAD_SSP2_SCK__GPIO_2_16,	/* SCK */
	MX28_PAD_SSP2_MOSI__GPIO_2_17,	/* MOSI */
	MX28_PAD_SSP2_MISO__GPIO_2_18,	/* MISO */
	MX28_PAD_SSP2_SS0__GPIO_2_19,	/* SS0 */
	MX28_PAD_SSP2_SS1__GPIO_2_20,	/* SS1 */
	MX28_PAD_SSP2_SS2__GPIO_2_21,	/* SS2 */
};

static struct spi_gpio_platform_data tx28_spi_gpio1_data = {
	.miso = MXS_GPIO_NR(2, 18),
	.mosi = MXS_GPIO_NR(2, 17),
	.sck = MXS_GPIO_NR(2, 16),
	.num_chipselect = 3,
};

static struct platform_device tx28_spi_gpio1_device = {
	.name = "spi_gpio",
	.id = 1,
	.dev = {
		.platform_data = &tx28_spi_gpio1_data,
	},
};

static int __init tx28_add_spi_gpio(void)
{
	int ret;

	mxs_iomux_setup_multiple_pads(tx28_spi_gpio0_pads,
				ARRAY_SIZE(tx28_spi_gpio0_pads));

	ret = platform_device_register(&tx28_spi_gpio0_device);
	if (ret)
		printk(KERN_ERR "Failed to register SPI0 device: %d\n", ret);

	mxs_iomux_setup_multiple_pads(tx28_spi_gpio1_pads,
				ARRAY_SIZE(tx28_spi_gpio1_pads));

	ret = platform_device_register(&tx28_spi_gpio1_device);
	if (ret)
		printk(KERN_ERR "Failed to register SPI1 device: %d\n", ret);
	return ret;
}

/* Touchscreen */
#define TSC2007_PEN_GPIO		MXS_GPIO_NR(3, 20)

static int tx28_stk5_tsc2007_init(void)
{
	int ret;

	ret = gpio_request(TSC2007_PEN_GPIO, "TSC2007");
	if (ret)
		return ret;

	return gpio_direction_input(TSC2007_PEN_GPIO);
}

static void tx28_stk5_tsc2007_exit(void)
{
	gpio_free(TSC2007_PEN_GPIO);
}

static int tx28_stk5_get_pendown(void)
{
	int val = gpio_get_value(TSC2007_PEN_GPIO);

	pr_info("%s: TS pen is %s\n", __func__, val ? "up" : "down");
	return !val;
}

static struct tsc2007_platform_data tx28_stk5_tsc2007_pdata = {
	.model = 2007,
	.x_plate_ohms = 660,
	.get_pendown_state = tx28_stk5_get_pendown,
	.clear_penirq = NULL,
	.init_platform_hw = tx28_stk5_tsc2007_init,
	.exit_platform_hw = tx28_stk5_tsc2007_exit,
};

static struct pca953x_platform_data tx28_pca953x_pdata = {
	.gpio_base	= 160,
#ifdef CONFIG_GPIO_PCA953X_IRQ
	.irq_base	= 128,
#else
	.irq_base	= -1,
#endif
};

#if defined CONFIG_TOUCHSCREEN_EDT_FT5X06 || defined CONFIG_TOUCHSCREEN_EDT_FT5X06_MODULE
static struct edt_ft5x06_platform_data edt_ft5x06_pdata = {
	.irq_pin = TX28_STK5_GPIO_EDT_IRQ,
	.reset_pin = TX28_STK5_GPIO_EDT_RESET,
};
#endif

static struct i2c_board_info tx28_stk5v3_i2c_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.irq = MXS_GPIO_IRQ_START + TSC2007_PEN_GPIO,
		.platform_data = &tx28_stk5_tsc2007_pdata,
	}, {
		I2C_BOARD_INFO("pca9554", 0x20),
		.irq = MXS_GPIO_IRQ_START + MXS_GPIO_NR(3, 28),
		.platform_data = &tx28_pca953x_pdata,
	}, {
		I2C_BOARD_INFO("ds1339", 0x68),
	}, {
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
#if defined CONFIG_TOUCHSCREEN_EDT_FT5X06 || defined CONFIG_TOUCHSCREEN_EDT_FT5X06_MODULE
	{
		I2C_BOARD_INFO("edt-ft5x06", 0x38),
		.platform_data  = &edt_ft5x06_pdata,
	},
#endif /* EDT_FT5X06 */
};

#if defined(CONFIG_REGULATOR_FIXED_VOLTAGE) || \
	defined(CONFIG_REGULATOR_FIXED_VOLTAGE_MODULE)
static struct regulator_consumer_supply tx28_audio_consumer_supplies[] = {
	REGULATOR_SUPPLY("VDDA", "0-000a"),
	REGULATOR_SUPPLY("VDDIO", "0-000a"),
};

static struct regulator_init_data tx28_vdd_reg_init_data = {
	.constraints	= {
		.name	= "3V3",
		.always_on = 1,
	},
	.consumer_supplies = tx28_audio_consumer_supplies,
	.num_consumer_supplies = ARRAY_SIZE(tx28_audio_consumer_supplies),
};

static struct fixed_voltage_config tx28_vdd_pdata = {
	.supply_name	= "board-3V3",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.enabled_at_boot = 1,
	.init_data	= &tx28_vdd_reg_init_data,
};

static struct platform_device tx28_voltage_regulator = {
	.name		= "reg-fixed-voltage",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &tx28_vdd_pdata,
	},
};

static void __init tx28_add_regulators(void)
{
	platform_device_register(&tx28_voltage_regulator);
}
#else
static void __init tx28_add_regulators(void)
{
}
#endif

static struct fb_videomode tx28_vmodes[] = {
	{
		/* Emerging ETV570 640 x 480 display. Syncs low active,
		 * DE high active, 115.2 mm x 86.4 mm display area
		 */
		.name = "ETV570",
		.refresh = 60,
		.xres = 640,
		.yres = 480,
		.pixclock = KHZ2PICOS(25175),
		.left_margin = 114,
		.hsync_len = 30,
		.right_margin = 16,
		.upper_margin = 32,
		.vsync_len = 3,
		.lower_margin = 10,
		.sync = FB_SYNC_DATA_ENABLE_HIGH_ACT,
		.vmode = FB_VMODE_NONINTERLACED,
	},
	{
		.name		= "tx15",
		.refresh	= 55,
		.xres		= 400,
		.yres		= 240,
		.pixclock	= 120000,
		.left_margin	= 80,
		.right_margin	= 100,
		.upper_margin	= 5,
		.lower_margin	= 15,
		.hsync_len	= 1,
		.vsync_len	= 1,
		.sync		= FB_SYNC_DATA_ENABLE_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	{
		.name		= "tx26",
		.refresh	= 60,
		.xres		= 800,
		.yres		= 256,
		.pixclock	= 50000,
		.left_margin	= 254,
		.right_margin	= 1,
		.upper_margin	= 58,
		.lower_margin	= 1,
		.hsync_len	= 1,
		.vsync_len	= 1,
		.sync		= FB_SYNC_DATA_ENABLE_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	{
		/* Emerging ETV0430 480 x 272 display. Syncs low active,
		 * DE high active, 115.2 mm x 86.4 mm display area
		 */
		.name = "ETV0430",
		.refresh = 60,
		.xres = 480,
		.yres = 272,
		.pixclock = KHZ2PICOS(8999),
		.left_margin = 2,
		.right_margin = 2,
		.hsync_len = 41,
		.upper_margin = 2,
		.lower_margin = 2,
		.vsync_len = 10,
		.sync = FB_SYNC_DATA_ENABLE_HIGH_ACT,
		.vmode = FB_VMODE_NONINTERLACED,
	},
};

static unsigned long tx28_videomem = SZ_2M;

static struct mxsfb_platform_data tx28_fb_pdata = {
	.mode_list = tx28_vmodes,
	.mode_count = ARRAY_SIZE(tx28_vmodes),
	.default_bpp = 32,
	.dotclk_delay = 0,
	.ld_intf_width = STMLCDIF_24BIT,
};

static void __init tx28_reserve(void)
{
	phys_addr_t memend = memblock_end_of_DRAM();
	int ret;

	tx28_videomem &= ~(SZ_1M - 1);

	if (tx28_videomem > SZ_16M)
		tx28_videomem = SZ_16M;

	if (tx28_videomem) {
		ret = memblock_remove(memend - tx28_videomem, tx28_videomem);

		if (!ret) {
			tx28_fb_pdata.fb_phys = memend - tx28_videomem;
			tx28_fb_pdata.fb_size = tx28_videomem;
		}
	}
}

static int __init tx28_set_videomem(char *str)
{
	if (str)
		tx28_videomem = memparse(str, NULL);

	return 0;
}
early_param("videomem", tx28_set_videomem);

static struct platform_pwm_backlight_data pwm_data = {
	.pwm_id = 0,
	.max_brightness = 100,
	.dft_brightness = 100,
	.pwm_period_ns = 1000000,
};

static struct platform_device tx28_pwm_backlight = {
	.id = 0,
	.name = "pwm-backlight",
	.dev = {
		.platform_data = &pwm_data,
	},
};

static const iomux_cfg_t tx28_fb_pads[] __initconst = {
	MX28_PAD_PWM0__GPIO_3_16,
	MX28_PAD_LCD_RESET__GPIO_3_30,
	MX28_PAD_LCD_ENABLE__GPIO_1_31,
};

static const struct gpio tx28_lcd_gpios[] __initconst = {
	{
		.gpio = TX28_STK5_GPIO_LCD_ENABLE,
		.flags = GPIOF_OUT_INIT_HIGH,
		.label = "lcd-enable",
	},
	{
		.gpio = TX28_STK5_GPIO_LCD_RESET,
		.flags = GPIOF_OUT_INIT_HIGH,
		.label = "lcd-reset",
	},
	{
		.gpio = TX28_STK5_GPIO_BACKLIGHT,
		.flags = GPIOF_OUT_INIT_HIGH,
		.label = "lcd-backlight",
	},
};

static void tx28_lcd_power(struct plat_lcd_data *pdata, unsigned int on)
{
	static int lcd_power;

	if (!lcd_power ^ !on) {
		if (on) {
			gpio_set_value(TX28_STK5_GPIO_LCD_ENABLE, on);
			udelay(15);
			gpio_set_value(TX28_STK5_GPIO_LCD_RESET, on);
			/* The ETV570 LCD requires this delay for a
			 * flicker free power on sequence
			 */
			msleep(300);
		} else {
			gpio_set_value(TX28_STK5_GPIO_LCD_ENABLE, on);
			gpio_set_value(TX28_STK5_GPIO_LCD_RESET, on);
		}
		lcd_power = !!on;
	}
}

static struct plat_lcd_data tx28_lcd_data = {
	.set_power = tx28_lcd_power,
};

static struct platform_device tx28_lcd_device = {
	.id = 0,
	.name = "platform-lcd",
	.dev = {
		.platform_data = &tx28_lcd_data,
	},
};

static void (*old_power_off)(void);

static void tx28_power_off(void)
{
	tx28_lcd_power(NULL, 0);
	if (old_power_off)
		old_power_off();
}

static void __init tx28_init_fb(void)
{
	struct platform_device *pdev;

	old_power_off = pm_power_off;
	pm_power_off = tx28_power_off;

	gpio_request_array(tx28_lcd_gpios, ARRAY_SIZE(tx28_lcd_gpios));

	mxs_iomux_setup_multiple_pads(tx28_fb_pads,
				ARRAY_SIZE(tx28_fb_pads));

	mx28_add_mxs_pwm(0);
	pdev = mx28_add_mxsfb(&tx28_fb_pdata);

	if (pdev) {
		tx28_lcd_device.dev.parent = &pdev->dev;
		platform_device_register(&tx28_lcd_device);

		tx28_pwm_backlight.dev.parent = &pdev->dev;
		platform_device_register(&tx28_pwm_backlight);
	}
}

static const struct mxs_mmc_platform_data tx28_mmc0_pdata __initconst = {
	.wp_gpio = -EINVAL,
	.cd_gpio = MXS_GPIO_NR(2, 9),
	.flags = SLOTF_4_BIT_CAPABLE,
};

static const struct mxs_mmc_platform_data tx28_mmc2_pdata __initconst = {
	.wp_gpio = -EINVAL,
	.cd_gpio = -EINVAL,
	.flags = SLOTF_4_BIT_CAPABLE,
};

static struct mtd_partition tx28_mtd_parts[] = {
/*
 * This partition is written with SW ECC and thus not accessible
 * from withing Linux
 */
/*
	{
		.name = "FCB",
		.size = SZ_128K,
		.offset = 0,
		.mask_flags = MTD_WRITEABLE,
	},
*/
	{
		.name = "U-Boot env",
		.size = SZ_128K,
		.offset = SZ_128K,
	},
	{
		.name = "u-boot",
		.size = SZ_1M,
		.offset = MTDPART_OFS_APPEND,
	},
	{
		.name = "rootfs",
		.size = SZ_16M,
		.offset = 2 * SZ_128K,
	},
	{
		.name = "userfs",
		.size = SZ_128M - 2 * SZ_128K - SZ_16M - SZ_4M,
		.offset = MTDPART_OFS_APPEND,
	},
	{
		.name = "linux",
		.size = SZ_4M,
		.offset = MTDPART_OFS_APPEND,
	},
};

static const struct gpmi_nand_platform_data tx28_gpmi_pdata __initconst = {
	.min_prop_delay_in_ns	= 5,
	.max_prop_delay_in_ns	= 9,
	.max_chip_count		= 1,
	.partitions = tx28_mtd_parts,
	.partition_count = ARRAY_SIZE(tx28_mtd_parts),
	.use_flash_bbt = 1,
};

static const iomux_cfg_t tx28_gpmi_pads[] __initconst = {
	MX28_PAD_GPMI_D00__GPMI_D0,
	MX28_PAD_GPMI_D01__GPMI_D1,
	MX28_PAD_GPMI_D02__GPMI_D2,
	MX28_PAD_GPMI_D03__GPMI_D3,
	MX28_PAD_GPMI_D04__GPMI_D4,
	MX28_PAD_GPMI_D05__GPMI_D5,
	MX28_PAD_GPMI_D06__GPMI_D6,
	MX28_PAD_GPMI_D07__GPMI_D7,
	MX28_PAD_GPMI_CE0N__GPMI_CE0N,
	MX28_PAD_GPMI_RDY0__GPMI_READY0,
	MX28_PAD_GPMI_RDN__GPMI_RDN,
	MX28_PAD_GPMI_WRN__GPMI_WRN,
	MX28_PAD_GPMI_ALE__GPMI_ALE,
	MX28_PAD_GPMI_CLE__GPMI_CLE,
	MX28_PAD_GPMI_RESETN__GPMI_RESETN,
};

static int __init tx28_add_gpmi_nand(void)
{
	int ret;
	struct platform_device *pdev;

	ret = mxs_iomux_setup_multiple_pads(tx28_gpmi_pads,
			ARRAY_SIZE(tx28_gpmi_pads));
	if (ret) {
		printk(KERN_ERR "Failed to setup GPMI pads: %d\n", ret);
		return ret;
	}
	pdev = mx28_add_gpmi_nand(&tx28_gpmi_pdata);
	if (!pdev) {
		printk(KERN_ERR "Failed to add GPMI device\n");
		return -ENOMEM;
	}
	return 0;
}

/* USB */
static const struct mxc_usbh_platform_data tx28_ehci0_pdata __initconst = {
};

static const struct mxc_usbh_platform_data tx28_ehci1_pdata __initconst = {
};

static int tx28_usbphy0_set_vbus(int on)
{
	return gpio_direction_output(TX28_STK5_GPIO_USBOTG_VBUSEN, on);
}

static struct mxs_mxs_usbphy_platform_data tx28_usbphy0_pdata = {
	.set_vbus = tx28_usbphy0_set_vbus,
	.dev_id_host = "mxc-ehci.0",
	.dev_id_peripheral = "fsl-usb2-udc",
	.flags = MXS_USBPHY_HOST | MXS_USBPHY_DEVICE,
};

static int tx28_usbphy1_set_vbus(int on)
{
	return gpio_direction_output(TX28_STK5_GPIO_USBHOST_VBUSEN, on);
}

static struct mxs_mxs_usbphy_platform_data tx28_usbphy1_pdata = {
	.set_vbus = tx28_usbphy1_set_vbus,
	.dev_id_host = "mxc-ehci.1",
	.flags = MXS_USBPHY_HOST,
};

static const struct fsl_usb2_platform_data tx28_usb_udc_pdata __initconst = {
	.operating_mode = FSL_USB2_DR_DEVICE,
	.phy_mode = FSL_USB2_PHY_UTMI_WIDE,
	.require_transceiver = 1,
};

enum {
	TX28_OTG_MODE_NONE,
	TX28_OTG_MODE_HOST,
	TX28_OTG_MODE_DEVICE,
};

static int tx28_otg_mode __initdata;

/* USB device */
static int __init tx28_udc_init(void)
{
	int ret;
	struct clk *clk = clk_get_sys("mxc-ehci.0", "usb");

	if (IS_ERR(clk))
		return PTR_ERR(clk);

	ret = clk_enable(clk);
	clk_put(clk);
	return ret;
}

static void __init tx28_add_usb(void)
{
	int ret;

	printk(KERN_DEBUG "tx28_otg_mode is: ");
	switch (tx28_otg_mode) {
	case TX28_OTG_MODE_HOST:
		printk(KERN_CONT "HOST\n");
		break;
	case TX28_OTG_MODE_DEVICE:
		printk(KERN_CONT "DEVICE\n");
		break;
	default:
		printk(KERN_CONT "unknown\n");
	}

	if (tx28_otg_mode == TX28_OTG_MODE_DEVICE) {
		ret = tx28_udc_init();
		if (ret == 0) {
			mx28_add_mxs_usbphy(0, &tx28_usbphy0_pdata);
			mx28_add_fsl_usb2_udc(&tx28_usb_udc_pdata);
		}
	}

	if (gpio_request_one(TX28_STK5_GPIO_USBHOST_VBUSEN,
				GPIOF_OUT_INIT_HIGH, "usbhost_vbusen") == 0) {
			mx28_add_mxs_usbphy(1, &tx28_usbphy1_pdata);
			mx28_add_mxs_ehci(1, &tx28_ehci1_pdata);
	}
	if (tx28_otg_mode != TX28_OTG_MODE_HOST)
		return;

	if (gpio_request_one(TX28_STK5_GPIO_USBOTG_VBUSEN,
				GPIOF_OUT_INIT_HIGH, "usbotg_vbusen") == 0) {
			mx28_add_mxs_usbphy(0, &tx28_usbphy0_pdata);
			mx28_add_mxs_ehci(0, &tx28_ehci0_pdata);
	}
}

/* flexcan */
static void tx28_flexcan_xcvr_enable(int id, int enable)
{
	static int tx28_flexcan_xcvr_on;

	if (enable) {
		if (tx28_flexcan_xcvr_on++ == 0) {
			printk(KERN_DEBUG "Enabling flexcan transceiver\n");
			gpio_set_value(TX28_STK5_GPIO_FLEXCAN_XCVR_EN, 0);
		}
	} else {
		if (--tx28_flexcan_xcvr_on == 0) {
			printk(KERN_DEBUG "Disabling flexcan transceiver\n");
			gpio_set_value(TX28_STK5_GPIO_FLEXCAN_XCVR_EN, 1);
		}
		WARN_ON(tx28_flexcan_xcvr_on < 0);
	}
}

static void tx28_flexcan0_xcvr_enable(int enable)
{
	tx28_flexcan_xcvr_enable(0, enable);
}

static void tx28_flexcan1_xcvr_enable(int enable)
{
	tx28_flexcan_xcvr_enable(1, enable);
}

static const struct flexcan_platform_data tx28_flexcan_pdata[] __initconst = {
	{
		.transceiver_switch = tx28_flexcan0_xcvr_enable,
	},
	{
		.transceiver_switch = tx28_flexcan1_xcvr_enable,
	},
};

static const iomux_cfg_t tx28_can1_pads[] __initconst = {
	MX28_PAD_GPMI_CE2N__CAN1_TX,
	MX28_PAD_GPMI_CE3N__CAN1_RX,
};

static void __init tx28_add_flexcan(unsigned int id)
{
	static int first __initdata = 1;

	if (id >= ARRAY_SIZE(tx28_flexcan_pdata))
		return;
	if (first) {
		mxs_iomux_setup_pad(MX28_PAD_LCD_D00__GPIO_1_0);
		if (gpio_request_one(TX28_STK5_GPIO_FLEXCAN_XCVR_EN,
					GPIOF_OUT_INIT_LOW, "FLEXCAN") != 0) {
			printk(KERN_ERR "Failed to request GPIO for flexcan transceiver\n");
			return;
		}
		first = 0;
	}
	if (id == 1) {
		mxs_iomux_setup_multiple_pads(tx28_can1_pads,
					ARRAY_SIZE(tx28_can1_pads));
	}

	mx28_add_flexcan(id, &tx28_flexcan_pdata[id]);
}

static int __init tx28_saif_init(void)
{
	void __iomem *digctrl_base = ioremap(MX28_DIGCTL_BASE_ADDR, SZ_4K);

	if (digctrl_base == NULL)
		return -ENOMEM;

	/* set the saif clk mux, both saif0/saif1 use saif0 clk */
	__raw_writel(0x2 << 10, digctrl_base);

	iounmap(digctrl_base);

	tx28_add_regulators();

	/* register audio playback device */
	mx28_add_saif(0, NULL);
	/* register audio capture device */
	mx28_add_saif(1, NULL);

	mxs_add_platform_device("mxs-sgtl5000", 0, NULL, 0,
				NULL, 0);
	return 0;
}

static void __init tx28_board_init(void)
{
	mxs_iomux_setup_multiple_pads(tx28_stk5v3_pads,
			ARRAY_SIZE(tx28_stk5v3_pads));

	mx28_add_duart();
	mx28_add_auart(1);
	mx28_add_auart(3);

	tx28_add_fec0();
	/* spi via ssp will be added when available */
	spi_register_board_info(tx28_spi_board_info,
			ARRAY_SIZE(tx28_spi_board_info));
	tx28_add_spi_gpio();

	mx28_add_mxs_i2c(0);
	i2c_register_board_info(0, tx28_stk5v3_i2c_boardinfo,
			ARRAY_SIZE(tx28_stk5v3_i2c_boardinfo));
	tx28_init_fb();
	mx28_add_mxs_mmc(0, &tx28_mmc0_pdata);
	tx28_add_usb();
	mx28_add_rtc_stmp3xxx();
	tx28_add_gpmi_nand();

	tx28_saif_init();
}

static void __init tx28_stk5v3_init(void)
{
	tx28_board_init();
	gpio_led_register_device(0, &tx28_stk5v3_led_data);
	mx28_add_mxs_mmc(2, &tx28_mmc2_pdata);

	if (tx28_otg_mode != TX28_OTG_MODE_HOST)
		mx28_add_flexcan(0, NULL);
	mx28_add_flexcan(1, NULL);
}

static void __init tx28_stk5v4_init(void)
{
	tx28_board_init();
	tx28_add_fec1();

	if (tx28_otg_mode != TX28_OTG_MODE_HOST)
		tx28_add_flexcan(0);
	tx28_add_flexcan(1);
}

static void (*initfunc)(void) __initdata;

static int __init tx28_select_base(/* const */ char *options)
{
	if (strcmp(options, "stkv3") == 0)
		initfunc = tx28_stk5v3_init;
	else if (strcmp(options, "stkv4") == 0)
		initfunc = tx28_stk5v4_init;

	return 0;
}
__setup("tx28_base=", tx28_select_base);

static void __init tx28_init(void)
{
	if (initfunc) {
		(*initfunc)();
	} else {
		pr_info("Defaulting to STK5 v3 board\n");
		tx28_stk5v3_init();
	}
}

static int __init tx28_select_otg_mode(/* const */ char *options)
{
	if (strcmp(options, "host") == 0)
		tx28_otg_mode = TX28_OTG_MODE_HOST;
	else if (strcmp(options, "device") == 0)
		tx28_otg_mode = TX28_OTG_MODE_DEVICE;
	else if (strcmp(options, "off") != 0)
		return -EINVAL;
	return 0;
}
__setup("tx28_otg_mode=", tx28_select_otg_mode);

static int __init tx28_lcd_backlight_init(void)
{
	int ret = 0;

	if (!machine_is_tx28())
		return 0;

	/* configure the pwm pin to pwm functionality late
	 * so that we know the pwm driver has already configured
	 * the pwm.
	 */
#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
	printk(KERN_DEBUG "%s: Setting up PWM backlight control\n", __func__);
	mxs_iomux_setup_pad(MX28_PAD_PWM0__PWM_0 | LCD_MODE);
#else
	printk(KERN_DEBUG "%s: Switching LCD backlight on\n", __func__);
	gpio_free(TX28_STK5_GPIO_BACKLIGHT);
	ret = gpio_request_one(TX28_STK5_GPIO_BACKLIGHT, GPIOF_OUT_INIT_LOW,
			"LCD Backlight");
#endif
	return ret;
}
late_initcall(tx28_lcd_backlight_init);

static void __init tx28_timer_init(void)
{
	mx28_clocks_init();
}

static struct sys_timer tx28_timer = {
	.init = tx28_timer_init,
};

MACHINE_START(TX28, "Ka-Ro electronics TX28 module")
	.map_io = mx28_map_io,
	.init_irq = mx28_init_irq,
	.reserve = tx28_reserve,
	.init_machine = tx28_init,
	.timer = &tx28_timer,
	.restart	= mxs_restart,
MACHINE_END
