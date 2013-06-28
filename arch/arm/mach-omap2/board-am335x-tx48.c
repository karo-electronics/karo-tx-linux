/*
 * Code for Ka-Ro electronics GmbH TX48 module with TI AM335X.
 * (C) Copyright 2013 Lothar Waßmann <LW@KARO-electronics.de>
 *
 * based on board-am335xevm.c
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/i2c/at24.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/wl12xx.h>
#include <linux/ethtool.h>
#include <linux/pwm_backlight.h>
#include <linux/input/ti_tsc.h>
#include <linux/platform_data/ti_adc.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/rtc/rtc-omap.h>
#include <linux/opp.h>
#include <linux/i2c/tsc2007.h>

/* LCD controller is similar to DA850 */
#include <video/da8xx-fb.h>

#include <mach/hardware.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/omap-pm.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>
#include <plat/nand.h>

#include "common.h"
#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"

void am33xx_d_can_init(unsigned int instance);

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

enum tx48_baseboards {
	STK5_V3 = 3,
	STK5_V5 = 5,
};

static enum tx48_baseboards tx48_baseboard __initdata = STK5_V3;

static int __init tx48_setup_baseboard(char *option)
{
	if (strcasecmp(option, "stk5-v5") == 0)
		tx48_baseboard = STK5_V5;
	else
		return 0;
	return 1;
}
__setup("tx48_base=", tx48_setup_baseboard);

static const struct display_panel disp_panel = {
	WVGA,
	32,
	32,
	COLOR_ACTIVE,
};

/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

static void setup_pin_mux(const struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);
}

/* LCD backlight platform Data */
#define STK5_BACKLIGHT_MAX_BRIGHTNESS		100
#define STK5_BACKLIGHT_DEFAULT_BRIGHTNESS	100
#define STK5_PWM_PERIOD_NANO_SECONDS		(5000 * 10)
#define STK5_BACKLIGHT_GPIO			GPIO_TO_PIN(3, 16)

static struct platform_pwm_backlight_data stk5_backlight_data;

/* Module pin mux for EHRPWM */
static const struct pinmux_config ehrpwm0_pin_mux[] = {
	{ "mcasp0_aclkx.ehrpwm0a", AM33XX_PIN_OUTPUT, },
	{ }
};

static const struct pinmux_config ehrpwm0_gpio_pin_mux[] = {
	{ "mcasp0_aclkx.gpio3_14", AM33XX_PIN_OUTPUT, },
	{ }
};

static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias = 255,
	.ac_bias_intrpt = 0,
	.dma_burst_sz = 16,
	.bpp = 32,
	.fdd = 0x80,
	.tft_alt_mode = 0,
	.stn_565_mode = 0,
	.mono_8bit_mode = 0,
	.invert_line_clock = 1,
	.invert_frm_clock = 1,
	.invert_pixel_clock = 1,
	.sync_edge = 0,
	.sync_ctrl = 1,
	.raster_order = 0,
};

static struct da8xx_lcdc_platform_data stk5_lcdc_pdata = {
	.controller_data = &lcd_cfg,
};

/* TSc controller */
static struct tsc_data stk5_touchscreen_data = {
	.wires = 4,
	.x_plate_resistance = 200,
	.steps_to_configure = 5,
};

static struct adc_data stk5_adc_data = {
	.adc_channels = 4,
};

static struct mfd_tscadc_board tscadc = {
	.tsc_init = &stk5_touchscreen_data,
	.adc_init = &stk5_adc_data,
};

static u8 stk5_iis_serializer_direction1[] = {
	RX_MODE,	TX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data stk5_snd_data1 = {
	.tx_dma_offset = 0x46400000,	/* McASP1 */
	.rx_dma_offset = 0x46400000,
	.op_mode = DAVINCI_MCASP_IIS_MODE,
	.tdm_slots = 2,
	.serial_dir = stk5_iis_serializer_direction1,
	.num_serializer = ARRAY_SIZE(stk5_iis_serializer_direction1),
	.asp_chan_q = EVENTQ_2,
	.version = MCASP_VERSION_3,
	.txnumevt = 32,
	.rxnumevt = 32,
	.get_context_loss_count =
			omap_pm_get_dev_context_loss_count,
};

static struct omap2_hsmmc_info stk5_mmc[] __initdata = {
	{
		.mmc = 2,
		.caps = MMC_CAP_4_BIT_DATA,
		.gpio_cd = GPIO_TO_PIN(3, 15),
		.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	},
	{ /* Terminator */ }
};

/* Module pin mux for LCDC */
static const struct pinmux_config lcdc_pin_mux[] __initconst = {
	{ "lcd_data0.lcd_data0", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data1.lcd_data1", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data2.lcd_data2", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data3.lcd_data3", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data4.lcd_data4", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data5.lcd_data5", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data6.lcd_data6", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data7.lcd_data7", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data8.lcd_data8", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data9.lcd_data9", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data10.lcd_data10", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data11.lcd_data11", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data12.lcd_data12", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data13.lcd_data13", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data14.lcd_data14", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "lcd_data15.lcd_data15", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA, },
	{ "gpmc_ad8.lcd_data16", AM33XX_PIN_OUTPUT, },
	{ "gpmc_ad9.lcd_data17", AM33XX_PIN_OUTPUT, },
	{ "gpmc_ad10.lcd_data18", AM33XX_PIN_OUTPUT, },
	{ "gpmc_ad11.lcd_data19", AM33XX_PIN_OUTPUT, },
	{ "gpmc_ad12.lcd_data20", AM33XX_PIN_OUTPUT, },
	{ "gpmc_ad13.lcd_data21", AM33XX_PIN_OUTPUT, },
	{ "gpmc_ad14.lcd_data22", AM33XX_PIN_OUTPUT, },
	{ "gpmc_ad15.lcd_data23", AM33XX_PIN_OUTPUT, },
	{ "lcd_vsync.lcd_vsync", AM33XX_PIN_OUTPUT, },
	{ "lcd_hsync.lcd_hsync", AM33XX_PIN_OUTPUT, },
	{ "lcd_pclk.lcd_pclk", AM33XX_PIN_OUTPUT, },
	{ "lcd_ac_bias_en.lcd_ac_bias_en", AM33XX_PIN_OUTPUT, },
	{ "gpmc_a3.gpio1_19", AM33XX_PIN_OUTPUT, }, /* LCD RESET */
	{ "gpmc_a6.gpio1_22", AM33XX_PIN_OUTPUT, }, /* LCD POWER */
	{ }
};

/* Pin mux for nand flash module */
static const struct pinmux_config nand_pin_mux[] __initconst = {
	{ "gpmc_ad0.gpmc_ad0", AM33XX_PIN_INPUT_PULLUP, },
	{ "gpmc_ad1.gpmc_ad1", AM33XX_PIN_INPUT_PULLUP, },
	{ "gpmc_ad2.gpmc_ad2", AM33XX_PIN_INPUT_PULLUP, },
	{ "gpmc_ad3.gpmc_ad3", AM33XX_PIN_INPUT_PULLUP, },
	{ "gpmc_ad4.gpmc_ad4", AM33XX_PIN_INPUT_PULLUP, },
	{ "gpmc_ad5.gpmc_ad5", AM33XX_PIN_INPUT_PULLUP, },
	{ "gpmc_ad6.gpmc_ad6", AM33XX_PIN_INPUT_PULLUP, },
	{ "gpmc_ad7.gpmc_ad7", AM33XX_PIN_INPUT_PULLUP, },
	{ "gpmc_wait0.gpmc_wait0", AM33XX_PIN_INPUT_PULLUP, },
	{ "gpmc_wpn.gpmc_wpn", AM33XX_PIN_INPUT_PULLUP, },
	{ "gpmc_csn0.gpmc_csn0", AM33XX_PIN_OUTPUT_PULLUP, },
	{ "gpmc_advn_ale.gpmc_advn_ale", AM33XX_PULL_DISA, },
	{ "gpmc_oen_ren.gpmc_oen_ren", AM33XX_PULL_DISA, },
	{ "gpmc_wen.gpmc_wen", AM33XX_PULL_DISA, },
	{ "gpmc_ben0_cle.gpmc_ben0_cle", AM33XX_PULL_DISA, },
	{ }
};

/* Module pin mux for SPI */
static const struct pinmux_config spi0_pin_mux[] __initconst = {
	{ "spi0_sclk.spi0_sclk", AM33XX_PULL_ENBL | AM33XX_INPUT_EN, },
	{ "spi0_d0.spi0_d0", AM33XX_PIN_INPUT_PULLUP, },
	{ "spi0_d1.spi0_d1", AM33XX_PULL_ENBL | AM33XX_INPUT_EN, },
	{ "spi0_cs0.spi0_cs0", AM33XX_PIN_INPUT_PULLUP, },
	{ }
};

/* Module pin mux for rmii1 */
static const struct pinmux_config rmii1_pin_mux[] __initconst = {
	{ "gmii1_crs.rmii1_crs_dv", AM33XX_PIN_INPUT_PULLDOWN, },
	{ "gmii1_rxerr.rmii1_rxerr", AM33XX_PIN_INPUT_PULLDOWN, },
	{ "gmii1_txen.rmii1_txen", AM33XX_PIN_OUTPUT, },
	{ "gmii1_txd1.rmii1_txd1", AM33XX_PIN_OUTPUT, },
	{ "gmii1_txd0.rmii1_txd0", AM33XX_PIN_OUTPUT, },
	{ "gmii1_rxd1.rmii1_rxd1", AM33XX_PIN_INPUT_PULLUP, },
	{ "gmii1_rxd0.rmii1_rxd0", AM33XX_PIN_INPUT_PULLUP, },
	{ "rmii1_refclk.rmii1_refclk", AM33XX_PIN_INPUT, },
	{ "mdio_data.mdio_data", AM33XX_PIN_INPUT_PULLUP, },
	{ "mdio_clk.mdio_clk", AM33XX_PIN_OUTPUT_PULLUP, },
	{ "emu0.gpio3_7", AM33XX_PIN_INPUT_PULLUP, }, /* nINT */
	{ "emu1.gpio3_8", AM33XX_PIN_OUTPUT_PULLUP, }, /* nRST */
	{ }
};

/* Module pin mux for mcasp1 */
static const struct pinmux_config mcasp1_pin_mux[] __initconst = {
	{ "mcasp0_aclkr.mcasp1_aclkx", AM33XX_PIN_INPUT_PULLDOWN, },
	{ "mcasp0_fsr.mcasp1_fsx", AM33XX_PIN_INPUT_PULLDOWN, },
	{ "mcasp0_axr1.mcasp1_axr0", AM33XX_PIN_INPUT_PULLDOWN, },
	{ "mcasp0_ahclk.mcasp1_axr1", AM33XX_PIN_OUTPUT, },
	{ }
};


/* Module pin mux for mmc1 */
#define MMC_CD_GPIO
static const struct pinmux_config mmc1_pin_mux[] __initconst = {
#ifdef MMC_CD_GPIO
	{ "mcasp0_fsx.gpio3_15", AM33XX_PIN_INPUT_PULLUP, },
#else
	{ "mcasp0_fsx.mmc1_sdcd", AM33XX_PIN_INPUT_PULLUP, },
#endif
	{ "gmii1_txclk.mmc1_dat0", AM33XX_PIN_INPUT_PULLUP, },
	{ "gmii1_rxclk.mmc1_dat1", AM33XX_PIN_INPUT_PULLUP, },
	{ "gmii1_rxd3.mmc1_dat2", AM33XX_PIN_INPUT_PULLUP, },
	{ "gmii1_rxd2.mmc1_dat3", AM33XX_PIN_INPUT_PULLUP, },
	{ "gpmc_csn2.mmc1_cmd", AM33XX_PIN_INPUT_PULLUP, },
	{ "gpmc_csn1.mmc1_clk", AM33XX_PIN_INPUT_PULLUP, },
	{ }
};

/* pinmux for led device */
static const struct pinmux_config gpio_led_mux[] __initconst = {
	{ "gpmc_a10.gpio1_26", AM33XX_PIN_INPUT, },
	{ }
};

/* Matrix GPIO Keypad Support */
/* pinmux for keypad device */
static const struct pinmux_config matrix_keypad_pin_mux[] __initconst = {
	/* keypad row gpios */
	{ "xdma_event_intr0.gpio0_19", AM33XX_PIN_INPUT_PULLDOWN, },
	{ "xdma_event_intr1.gpio0_20", AM33XX_PIN_INPUT_PULLDOWN, },
	{ "gpmc_clk.gpio2_1", AM33XX_PIN_INPUT_PULLDOWN, },
	{ "gpmc_csn3.gpio2_0", AM33XX_PIN_INPUT_PULLDOWN, },
	/* keypad col gpios */
	{ "mmc0_dat3.gpio2_26", AM33XX_PIN_OUTPUT, },
	{ "mcasp0_ahclkr.gpio3_17", AM33XX_PIN_OUTPUT, },
	{ "ecap0_in_pwm0_out.gpio0_7", AM33XX_PIN_OUTPUT, },
	{ "gpmc_ben1.gpio1_28", AM33XX_PIN_OUTPUT, },
	{ }
};

/* Keys mapping */
static const uint32_t stk5_matrix_keys[] = {
	KEY(0, 0, KEY_POWER),
};

static const struct matrix_keymap_data stk5_keymap_data = {
	.keymap = stk5_matrix_keys,
	.keymap_size = ARRAY_SIZE(stk5_matrix_keys),
};

static const unsigned int stk5_keypad_row_gpios[] = {
	GPIO_TO_PIN(0, 19),
	GPIO_TO_PIN(0, 20),
	GPIO_TO_PIN(2, 1),
	GPIO_TO_PIN(2, 0),
};

static const unsigned int stk5_keypad_col_gpios[] = {
	GPIO_TO_PIN(2, 26),
	GPIO_TO_PIN(3, 17),
	GPIO_TO_PIN(0, 7),
	GPIO_TO_PIN(1, 28),
};

static struct matrix_keypad_platform_data stk5_keypad_platform_data = {
	.keymap_data = &stk5_keymap_data,
	.row_gpios = stk5_keypad_row_gpios,
	.num_row_gpios = ARRAY_SIZE(stk5_keypad_row_gpios),
	.col_gpios = stk5_keypad_col_gpios,
	.num_col_gpios = ARRAY_SIZE(stk5_keypad_col_gpios),
	.debounce_ms = 5,
	.col_scan_delay_us = 2,
	.wakeup = 1,
};

static struct platform_device stk5_keyboard = {
	.name = "matrix-keypad",
	.id = -1,
	.dev = {
		.platform_data = &stk5_keypad_platform_data,
	},
};

static void __init matrix_keypad_init(void)
{
	int err;

	setup_pin_mux(matrix_keypad_pin_mux);
	err = platform_device_register(&stk5_keyboard);
	if (err)
		pr_err("failed to register matrix keypad (%ux%u) device\n",
			ARRAY_SIZE(stk5_keypad_col_gpios),
			ARRAY_SIZE(stk5_keypad_row_gpios));
}

/* pinmux for usb0 drvvbus */
static const struct pinmux_config usb0_pin_mux[] __initconst = {
	{ "usb0_drvvbus.usb0_drvvbus", AM33XX_PIN_OUTPUT, },
	{ }
};

/* pinmux for usb1 drvvbus */
static const struct pinmux_config usb1_pin_mux[] __initconst = {
	{ "usb1_drvvbus.usb1_drvvbus", AM33XX_PIN_OUTPUT, },
	{ }
};

static bool backlight_enable;

static void __init enable_ehrpwm0(void)
{
	backlight_enable = true;
	setup_pin_mux(ehrpwm0_pin_mux);
}

/* Setup pwm-backlight */
static struct platform_pwm_backlight_data stk5_backlight_data = {
	.pwm_id = "ehrpwm.0",
	.ch = 0,
	.lth_brightness = 0,
	.max_brightness = STK5_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness = STK5_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns  = 50000,
};

static struct platform_device stk5_backlight = {
	.name = "pwm-backlight",
	.id = -1,
	.dev = {
		.platform_data = &stk5_backlight_data,
	},
};

static struct pwmss_platform_data pwm_pdata = {
	.version = PWM_VERSION_1,
};

static int __init backlight_init(void)
{
	int status = 0;

	if (backlight_enable) {
		struct pwm_device *pwm;
		struct platform_pwm_backlight_data *bd = &stk5_backlight_data;

		am33xx_register_ehrpwm(0, &pwm_pdata);
		pwm = pwm_request(bd->pwm_id, bd->ch, "backlight");
		if (IS_ERR(pwm))
			return PTR_ERR(pwm);

		pwm_set_polarity(pwm, 1);
		pwm_release(pwm);
		status = platform_device_register(&stk5_backlight);
	}
	return status;
}
late_initcall(backlight_init);

static int __init conf_disp_pll(int rate)
{
	struct clk *disp_pll;
	int ret = -EINVAL;

	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		pr_err("Cannot clk_get disp_pll\n");
		goto out;
	}

	ret = clk_set_rate(disp_pll, rate);
	clk_put(disp_pll);
out:
	return ret;
}

static const struct gpio stk5_lcdc_gpios[] __initconst = {
	{ GPIO_TO_PIN(1, 19), GPIOF_OUT_INIT_HIGH, "LCD reset", },
	{ GPIO_TO_PIN(1, 22), GPIOF_OUT_INIT_HIGH, "LCD power", },
};

static void __init lcdc_init(void)
{
	setup_pin_mux(lcdc_pin_mux);

	if (conf_disp_pll(300000000)) {
		pr_err("Failed configure display PLL, not attempting to register LCDC\n");
		return;
	}
	gpio_request_array(stk5_lcdc_gpios, ARRAY_SIZE(stk5_lcdc_gpios));

	stk5_lcdc_pdata.get_context_loss_count = omap_pm_get_dev_context_loss_count;
	if (am33xx_register_lcdc(&stk5_lcdc_pdata))
		pr_err("Failed to register LCDC device\n");
}

static void __init mfd_tscadc_init(void)
{
	int err;

	err = am33xx_register_mfd_tscadc(&tscadc);
	if (err)
		pr_err("failed to register touchscreen device\n");
}

static void __init rmii1_init(void)
{
	setup_pin_mux(rmii1_pin_mux);
}

static void __init usb0_init(void)
{
	setup_pin_mux(usb0_pin_mux);
}

static void __init usb1_init(void)
{
	setup_pin_mux(usb1_pin_mux);
}

/* setup uart pins */
/* Module pin mux for uart0 */
static const struct pinmux_config uart0_pin_mux[] __initconst = {
	{ "uart0_rxd.uart0_rxd", AM33XX_SLEWCTRL_SLOW |
	  AM33XX_PIN_INPUT_PULLUP, },
	{ "uart0_txd.uart0_txd", AM33XX_PULL_UP | AM33XX_PULL_DISA |
	  AM33XX_SLEWCTRL_SLOW, },
	{ "uart0_rtsn.uart0_rtsn", AM33XX_SLEWCTRL_SLOW |
	  AM33XX_PIN_INPUT_PULLUP, },
	{ "uart0_ctsn.uart0_ctsn", AM33XX_PULL_UP | AM33XX_PULL_DISA |
	  AM33XX_SLEWCTRL_SLOW, },
	{ }
};

static const struct pinmux_config uart1_pin_mux[] __initconst = {
	{ "uart1_rxd.uart1_rxd", AM33XX_SLEWCTRL_SLOW |
	  AM33XX_PIN_INPUT_PULLUP, },
	{ "uart1_txd.uart1_txd", AM33XX_PULL_UP | AM33XX_PULL_DISA |
	  AM33XX_SLEWCTRL_SLOW, },
	{ "uart1_rtsn.uart1_rtsn", AM33XX_SLEWCTRL_SLOW |
	  AM33XX_PIN_INPUT_PULLUP, },
	{ "uart1_ctsn.uart1_ctsn", AM33XX_PULL_UP | AM33XX_PULL_DISA |
	  AM33XX_SLEWCTRL_SLOW, },
	{ }
};

static const struct pinmux_config uart5_pin_mux[] __initconst = {
	{ "gmii1_col.uart5_rxd", AM33XX_PIN_INPUT_PULLUP, },
	{ "gmii1_rxdv.uart5_txd", AM33XX_PULL_ENBL, },
	{ "mmc0_dat0.uart5_rtsn", AM33XX_PIN_INPUT_PULLUP, },
	{ "mmc0_dat1.uart5_ctsn", AM33XX_PULL_ENBL, },
	{ }
};

static void __init uart0_init(void)
{
	setup_pin_mux(uart0_pin_mux);
}

static void __init uart1_init(void)
{
	setup_pin_mux(uart1_pin_mux);
}

static void __init uart5_init(void)
{
	setup_pin_mux(uart5_pin_mux);
}

static struct spi_board_info stk5_spi0_slave_info[] = {
	{
		.modalias = "spidev",
		.max_speed_hz = 24000000,
		.bus_num = 1,
		.chip_select = 0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 24000000,
		.bus_num = 1,
		.chip_select = 1,
	},
};

static struct gpmc_timings stk5_nand_timings = {
	.sync_clk = 0,

	.cs_on = 0,
	.cs_rd_off = 44,
	.cs_wr_off = 44,

	.adv_on = 6,
	.adv_rd_off = 34,
	.adv_wr_off = 44,
	.we_off = 40,
	.oe_off = 54,

	.access = 64,
	.rd_cycle = 82,
	.wr_cycle = 82,

	.wr_access = 40,
	.wr_data_mux_bus = 0,
};

static void __init tx48_nand_init(void)
{
	struct omap_nand_platform_data *pdata;
	struct gpmc_devices_info gpmc_device[2] = { };

	setup_pin_mux(nand_pin_mux);
	pdata = omap_nand_init(NULL, 0, 0, 0, &stk5_nand_timings);
	if (!pdata)
		return;
	pdata->ecc_opt = OMAP_ECC_BCH8_CODE_HW;
	pdata->elm_used = true;
	pdata->use_flash_bbt = true;
	gpmc_device[0].pdata = pdata;
	gpmc_device[0].flag = GPMC_DEVICE_NAND;

	omap_init_gpmc(gpmc_device, sizeof(gpmc_device));
	omap_init_elm();
}

static const struct pinmux_config tsc2007_pin_mux[] = {
	{ "mcasp0_axr0.gpio3_16", AM33XX_PIN_INPUT_PULLUP |
	  AM33XX_SLEWCTRL_SLOW, },
	{ }
};

#define TSC2007_PENDOWN_GPIO	GPIO_TO_PIN(3, 16)

static int tsc2007_get_pendown_state(void)
{
	int pd = !gpio_get_value(TSC2007_PENDOWN_GPIO);
	printk(KERN_DEBUG "Pen is %s\n", pd ? "down" : "up");
	return pd;
}

static int tsc2007_init_hw(void)
{
	int ret;

	setup_pin_mux(tsc2007_pin_mux);

	ret = gpio_request_one(TSC2007_PENDOWN_GPIO, GPIOF_IN, "TSC2007 pendown");
	if (ret)
		return ret;
	return 0;
}

static void tsc2007_exit_hw(void)
{
	gpio_free(TSC2007_PENDOWN_GPIO);
}

static struct tsc2007_platform_data tsc2007_pdata = {
	.x_plate_ohms = 660,
	.get_pendown_state = tsc2007_get_pendown_state,
	.init_platform_hw = tsc2007_init_hw,
	.exit_platform_hw = tsc2007_exit_hw,
};

static const struct pinmux_config i2c0_pin_mux[] __initconst = {
	{ "i2c0_sda.i2c0_sda", AM33XX_SLEWCTRL_SLOW | AM33XX_PIN_INPUT_PULLUP, },
	{ "i2c0_scl.i2c0_scl", AM33XX_SLEWCTRL_SLOW | AM33XX_PIN_INPUT_PULLUP, },
	{ }
};

static struct i2c_board_info stk5_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.irq = 0, /* setup at runtime by i2c0_init() */
		.platform_data = &tsc2007_pdata,
	},
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
	{
		I2C_BOARD_INFO("ds1339", 0x68),
	},
};

static void __init i2c0_init(void)
{
	setup_pin_mux(i2c0_pin_mux);
	stk5_i2c0_boardinfo[0].irq = gpio_to_irq(TSC2007_PENDOWN_GPIO);
	omap_register_i2c_bus(1, 100, stk5_i2c0_boardinfo,
			ARRAY_SIZE(stk5_i2c0_boardinfo));
}

/* Setup McASP 1 */
static void __init mcasp1_init(void)
{
	/* Configure McASP */
	setup_pin_mux(mcasp1_pin_mux);
	am335x_register_mcasp(&stk5_snd_data1, 1);
}

static const struct pinmux_config d_can0_pin_mux[] __initconst = {
	{ "gmii1_txd3.d_can0_tx", AM33XX_PULL_ENBL, },
	{ "gmii1_txd2.d_can0_rx", AM33XX_PIN_INPUT_PULLUP, },
	{ }
};

static const struct pinmux_config d_can1_pin_mux[] __initconst = {
	{ "mmc0_clk.d_can1_tx", AM33XX_PULL_ENBL, },
	{ "mmc0_cmd.d_can1_rx", AM33XX_PIN_INPUT_PULLUP, },
	{ }
};

static const struct pinmux_config can_xcvr_pin_mux[] __initconst = {
	{ "gpmc_ad8.gpio0_22", AM33XX_PIN_OUTPUT_PULLUP, },
	{ }
};

static int __init d_can_init(int id)
{
	switch (id) {
	case 0:
		setup_pin_mux(d_can0_pin_mux);
		break;
	case 1:
		setup_pin_mux(d_can1_pin_mux);
		break;

	default:
		return -EINVAL;
	}

	if (tx48_baseboard == STK5_V5) {
		setup_pin_mux(can_xcvr_pin_mux);
		gpio_request_one(GPIO_TO_PIN(0, 22), GPIOF_OUT_INIT_LOW, "CAN xcvr");
	}

	am33xx_d_can_init(id);
	return 0;
}

static void __init mmc1_init(void)
{
	setup_pin_mux(mmc1_pin_mux);
	omap2_hsmmc_init(stk5_mmc);
}

static struct gpio_led gpio_leds[] = {
	{
		.name = "heartbeat",
		.gpio = GPIO_TO_PIN(1, 26),
		.default_trigger = "heartbeat",
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds = gpio_leds,
	.num_leds = ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &gpio_led_info,
	},
};

static void __init gpio_led_init(void)
{
	int err;

	setup_pin_mux(gpio_led_mux);
	err = platform_device_register(&leds_gpio);
	if (err)
		pr_err("failed to register gpio led device\n");
}

/* setup spi0 */
static void __init spi0_init(void)
{
	setup_pin_mux(spi0_pin_mux);
	spi_register_board_info(stk5_spi0_slave_info,
			ARRAY_SIZE(stk5_spi0_slave_info));
}

static struct omap_rtc_pdata stk5_rtc_info = {
	.wakeup_capable = 1,
};

static int __init stk5_rtc_init(void)
{
	int ret;
	void __iomem *base;
	struct clk *clk;
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char *dev_name = "am33xx-rtc";

	clk = clk_get_sys(dev_name, "rtc_fck");
	if (IS_ERR(clk)) {
		pr_err("rtc: Failed to get RTC clock\n");
		return PTR_ERR(clk);
	}

	ret = clk_enable(clk);
	if (ret) {
		pr_err("rtc: Clock enable failed: %d\n", ret);
		return ret;
	}

	base = ioremap(AM33XX_RTC_BASE, SZ_4K);
	if (WARN_ON(!base))
		return -ENOMEM;

	/* Unlock the rtc's registers */
	writel(0x83e70b13, base + 0x6c);
	writel(0x95a4f1e0, base + 0x70);

	/*
	 * Enable the 32K OSc
	 * TODO: Need a better way to handle this
	 * Since we want the clock to be running before mmc init
	 * we need to do it before the rtc probe happens
	 */
	writel(0x48, base + 0x54);

	iounmap(base);

	clk_disable(clk);
	clk_put(clk);

	if (omap_rev() < AM335X_REV_ES2_0)
		stk5_rtc_info.wakeup_capable = 0;

	oh = omap_hwmod_lookup("rtc");
	if (!oh) {
		pr_err("could not look up %s\n", "rtc");
		return -ENOENT;
	}

	pdev = omap_device_build(dev_name, -1, oh, &stk5_rtc_info,
			sizeof(struct omap_rtc_pdata), NULL, 0, 0);
	if (WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name))
		return PTR_ERR(pdev);
	return 0;
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type = MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 * TX48 has USB0 in OTG mode and USB1 in host mode.
	 */
	.mode = (MUSB_HOST << 4) | MUSB_OTG,
	.power = 500,
	.instances = 1,
};

static void __iomem *am33xx_emif_base;

static void __iomem *__init am33xx_get_mem_ctlr(void)
{
	am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!am33xx_emif_base)
		pr_warning("%s: Unable to map DDR2 controller",	__func__);

	return am33xx_emif_base;
}

static struct resource am33xx_cpuidle_resources[] = {
	{
		.start = AM33XX_EMIF0_BASE,
		.end = AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags = IORESOURCE_MEM,
	},
};

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
};

static struct platform_device am33xx_cpuidle_device = {
	.name = "cpuidle-am33xx",
	.num_resources = ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource = am33xx_cpuidle_resources,
	.dev = {
		.platform_data = &am33xx_cpuidle_pdata,
	},
};

static void __init am33xx_cpuidle_init(void)
{
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);
	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");

}

static void __init am335x_tx48_init(void)
{
	am33xx_cpuidle_init();
	am33xx_mux_init(NULL);
	omap_serial_init();
	uart0_init();
	uart1_init();
	uart5_init();
	tx48_nand_init();
	gpio_led_init();
	i2c0_init();
	stk5_rtc_init();
	rmii1_init();
	am33xx_cpsw_init(AM33XX_CPSW_MODE_RMII, NULL, NULL);
	omap_sdrc_init(NULL, NULL);
	enable_ehrpwm0();
	lcdc_init();
	mfd_tscadc_init();
	usb0_init();
	usb1_init();
	usb_musb_init(&musb_board_data);
	mmc1_init();
	mcasp1_init();
	d_can_init(0);
	d_can_init(1);
	spi0_init();
	matrix_keypad_init();
	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");
}

static void __init am335x_tx48_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

MACHINE_START(TX48, "am335x-tx48")
	/* Maintainer: Ka-Ro electronics GmbH */
	.map_io		= am335x_tx48_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_tx48_init,
MACHINE_END
