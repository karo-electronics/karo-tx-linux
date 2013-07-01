/*
 * Copyright (C) 2013 by Oliver Wendt <OW@KARO-electronics.de>
 *
 * based on: arch/arm/mach-mx6/board-mx6q_sabresd.c
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/input/edt-ft5x06.h>
#include <linux/i2c/tsc2007.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/bitops.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"

#define TX6_USBH1_PWR_EN	IMX_GPIO_NR(3, 31)

#define TX6_SD1_CD		IMX_GPIO_NR(7, 2)
#define TX6_SD2_CD		IMX_GPIO_NR(7, 3)

#define TX6_ECSPI1_CS0		IMX_GPIO_NR(2, 30)
#define TX6_ECSPI1_CS1		IMX_GPIO_NR(3, 19)

#define TX6_PCIE_PWR_EN		IMX_GPIO_NR(3, 19)
#define TX6_USB_OTG_PWR		IMX_GPIO_NR(1, 7)
#define TX6_USB_H1_PWR		IMX_GPIO_NR(3, 31)
#define TX6_DISP_RST_B		IMX_GPIO_NR(3, 29)
#define TX6_DISP_PWR_EN		IMX_GPIO_NR(2, 31)

#define TX6_FEC_PHY_RST_B	IMX_GPIO_NR(7, 6)
#define TX6_FEC_PHY_PWR_EN	IMX_GPIO_NR(3, 20)

/* EDT FT5X06 */
#define EDT_FT5X06_IRQ_PIN	IMX_GPIO_NR(6, 15)
#define EDT_FT5X06_RESET_PIN	IMX_GPIO_NR(2, 22)
#define EDT_FT5X06_WAKE_PIN	IMX_GPIO_NR(2, 21)

#define TX6_FLEXCAN_XCVR_SW	IMX_GPIO_NR(4, 21)
#define TX6_TSC2007_PEN_GPIO	IMX_GPIO_NR(3, 26)
#define TX6_STK5_LED_GPIO	IMX_GPIO_NR(2, 20)
#define TX6_BACKLIGHT_GPIO	IMX_GPIO_NR(1, 1)

static struct clk *clko;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

static const struct esdhc_platform_data mx6q_tx6_sd1_data __initconst = {
	.cd_gpio = TX6_SD1_CD,
	.wp_gpio = -EINVAL,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_GPIO,
};

static const struct esdhc_platform_data mx6q_tx6_sd2_data __initconst = {
	.cd_gpio = TX6_SD2_CD,
	.wp_gpio = -EINVAL,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_GPIO,
};

static iomux_v3_cfg_t tx6_gpmi_nand_pads[] __initdata = {
	MX6Q_PAD_NANDF_CLE__RAWNAND_CLE,
	MX6Q_PAD_NANDF_ALE__RAWNAND_ALE,
	MX6Q_PAD_NANDF_CS0__RAWNAND_CE0N,
	MX6Q_PAD_NANDF_RB0__RAWNAND_READY0,
	MX6Q_PAD_NANDF_D0__RAWNAND_D0,
	MX6Q_PAD_NANDF_D1__RAWNAND_D1,
	MX6Q_PAD_NANDF_D2__RAWNAND_D2,
	MX6Q_PAD_NANDF_D3__RAWNAND_D3,
	MX6Q_PAD_NANDF_D4__RAWNAND_D4,
	MX6Q_PAD_NANDF_D5__RAWNAND_D5,
	MX6Q_PAD_NANDF_D6__RAWNAND_D6,
	MX6Q_PAD_NANDF_D7__RAWNAND_D7,
	MX6Q_PAD_SD4_CMD__RAWNAND_RDN,
	MX6Q_PAD_SD4_CLK__RAWNAND_WRN,
	MX6Q_PAD_NANDF_WP_B__RAWNAND_RESETN,
};

static int __init gpmi_nand_platform_init(void)
{
	return mxc_iomux_v3_setup_multiple_pads(tx6_gpmi_nand_pads,
						ARRAY_SIZE(tx6_gpmi_nand_pads));
}

static const struct gpmi_nand_platform_data
mx6q_gpmi_nand_platform_data __initconst = {
	.platform_init		= gpmi_nand_platform_init,
	.min_prop_delay_in_ns	= 5,
	.max_prop_delay_in_ns	= 9,
	.max_chip_count	= 1,
	.enable_bbt		= 1,
};

static const struct anatop_thermal_platform_data
	mx6q_tx6_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_tx6_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(2, NULL);
}

static int mx6q_tx6_fec_phy_init(struct phy_device *phydev)
{
	gpio_request_one(TX6_FEC_PHY_PWR_EN, GPIOF_OUT_INIT_HIGH, "fec_phy_pwr_en");

	gpio_request_one(TX6_FEC_PHY_RST_B, GPIOF_OUT_INIT_LOW, "fec_phy_reset_B");
	mdelay(1);
	gpio_set_value(TX6_FEC_PHY_RST_B, 1);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_tx6_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RMII,
};

static int mx6q_tx6_spi_cs[] = {
	TX6_ECSPI1_CS0,
	TX6_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_tx6_spi_data __initconst = {
	.chipselect	= mx6q_tx6_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_tx6_spi_cs),
};

static struct spi_board_info tx6_spi_board_info[] __initdata = {
	{
		.modalias	= "spidev",
		.max_speed_hz	= 54000000,
		.bus_num	= 0,
		.chip_select	= 0,
	},
	{
		.modalias	= "spidev",
		.max_speed_hz	= 54000000,
		.bus_num	= 0,
		.chip_select	= 1,
	},
};

static struct mxc_audio_platform_data mx6_tx6_audio_data;

static int mx6_tx6_sgtl5000_init(void)
{
	int ret;
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "ahb");
	if (!IS_ERR(new_parent)) {
		ret = clk_set_parent(clko, new_parent);
		clk_put(new_parent);
		if (ret) {
			pr_err("Failed to set CLKO parent to AHB\n");
			return ret;
		}
	}
	rate = clk_round_rate(clko, 16000000);
	if (rate < 8000000 || rate > 27000000) {
		pr_err("Error:SGTL5000 mclk freq %d out of range!\n", rate);
		ret = -EINVAL;
		goto err;
	}

	mx6_tx6_audio_data.sysclk = 26000000;
	ret = clk_set_rate(clko, rate);
	if (ret) {
		pr_err("Failed to set CLKO rate to %u.%03uMHz\n",
			mx6_tx6_audio_data.sysclk / 1000000,
			mx6_tx6_audio_data.sysclk / 1000 % 1000);
		goto err;
	}

	ret = clk_enable(clko);
	if (ret) {
		pr_err("Failed to enable CLKO clock: %d\n", ret);
		goto err;
	}

	return 0;

err:
	clk_put(clko);
	return ret;
}

static int mx6_tx6_sgtl5000_exit(void)
{
	if (!IS_ERR(clko)) {
		clk_disable(clko);
		clk_put(clko);
	}
	return 0;
}

static struct imx_ssi_platform_data mx6_tx6_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_tx6_audio_data = {
	.ssi_num	= 1,
	.src_port	= 2,
	.ext_port	= 5,
	.init		= mx6_tx6_sgtl5000_init,
	.finit		= mx6_tx6_sgtl5000_exit,
	.hp_gpio	= -1,
};

static struct platform_device mx6_tx6_audio_device = {
	.name = "imx-sgtl5000",
};

/* Multitouch controller */
#if defined(CONFIG_TOUCHSCREEN_EDT_FT5X06) || \
	defined(CONFIG_TOUCHSCREEN_EDT_FT5X06_MODULE)

static struct edt_ft5x06_platform_data edt_ft5x06_pdata = {
	.irq_pin = EDT_FT5X06_IRQ_PIN,
	.reset_pin = EDT_FT5X06_RESET_PIN,
	/* wakeup added - thou not really implemented in driver yet */
	.wakeup_pin = EDT_FT5X06_WAKE_PIN,
};

static inline void __init imx6q_tx6_init_edt_ft5x06(void)
{
	int ret = 0;

	ret = gpio_request_one(EDT_FT5X06_WAKE_PIN, GPIOF_OUT_INIT_LOW,
			"edt-ft5x06-wake_up");
	if (ret) {
		pr_err("edt-ft5x06: failed to get GPIO EDT_FT5X06_WAKE_PIN: %d\n",
			ret);
		return;
	} else {
		pr_info("edt-ft5x06: wake-up pin requesting: GPIO-%d \n",
			EDT_FT5X06_WAKE_PIN);
	}
	gpio_set_value(EDT_FT5X06_WAKE_PIN, 1);
}
#endif

static int tx6_tsc2007_get_pendown_state(void)
{
	return !gpio_get_value(TX6_TSC2007_PEN_GPIO);
}

static int tx6_tsc2007_plat_init(void)
{
	return gpio_request_one(TX6_TSC2007_PEN_GPIO, GPIOF_IN, "TSC2007 PENDETECT");
}

static void tx6_tsc2007_plat_exit(void)
{
	gpio_free(TX6_TSC2007_PEN_GPIO);
}

static struct tsc2007_platform_data tx6_tsc2007_pdata = {
	.x_plate_ohms = 480,
	.get_pendown_state = tx6_tsc2007_get_pendown_state,
	.init_platform_hw = tx6_tsc2007_plat_init,
	.exit_platform_hw = tx6_tsc2007_plat_exit,
};

static struct imxi2c_platform_data mx6q_tx6_i2c_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("ds1339", 0x68),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.platform_data = &tx6_tsc2007_pdata,
		.irq = gpio_to_irq(TX6_TSC2007_PEN_GPIO),
	},
#if defined(CONFIG_TOUCHSCREEN_EDT_FT5X06) || \
	defined(CONFIG_TOUCHSCREEN_EDT_FT5X06_MODULE)
	{
		I2C_BOARD_INFO("edt-ft5x06", 0x38),
		.platform_data  = &edt_ft5x06_pdata,
		.irq = gpio_to_irq(EDT_FT5X06_IRQ_PIN),
	},
#endif /* EDT_FT5X06 */
};

static void imx6q_tx6_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(TX6_USB_OTG_PWR, 1);
	else
		gpio_set_value(TX6_USB_OTG_PWR, 0);
}

static void imx6q_tx6_usbh1_vbus(bool on)
{
	if (on)
		gpio_set_value(TX6_USB_H1_PWR, 1);
	else
		gpio_set_value(TX6_USB_H1_PWR, 0);
}

static void __init imx6q_tx6_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	ret = gpio_request_one(TX6_USB_OTG_PWR, GPIOF_OUT_INIT_LOW, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO TX6_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	mx6_set_otghost_vbus_func(imx6q_tx6_usbotg_vbus);

	ret = gpio_request_one(TX6_USB_H1_PWR, GPIOF_OUT_INIT_LOW, "usb-h1-pwr");
	if (ret) {
		pr_err("failed to get GPIO TX6_USB_H1_PWR: %d\n",
			ret);
		return;
	}

	mx6_set_host1_vbus_func(imx6q_tx6_usbh1_vbus);
}

static void mx6q_tx6_flexcan_switch(int id, int enable)
{
	static unsigned long flexcan_enable;

	if (id < 0 || id > 1)
		return;
	if (enable) {
		if (!flexcan_enable)
			gpio_set_value(TX6_FLEXCAN_XCVR_SW, 0);
		set_bit(id, &flexcan_enable);
	} else {
		clear_bit(id, &flexcan_enable);
		if (!flexcan_enable)
			gpio_set_value(TX6_FLEXCAN_XCVR_SW, 1);
	}
}

static void mx6q_tx6_flexcan0_switch(int enable)
{
	mx6q_tx6_flexcan_switch(0, enable);
}

static void mx6q_tx6_flexcan1_switch(int enable)
{
	mx6q_tx6_flexcan_switch(1, enable);
}

static const struct flexcan_platform_data
mx6q_tx6_flexcan_pdata[] __initconst = {
	{
		.transceiver_switch = mx6q_tx6_flexcan0_switch,
	},
	{
		.transceiver_switch = mx6q_tx6_flexcan1_switch,
	},
};

static int __init tx6_flexcan_init(void)
{
	int ret;

	ret = gpio_request_one(TX6_FLEXCAN_XCVR_SW, GPIOF_OUT_INIT_HIGH, "Flexcan XCVR");
	return ret;
}

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data tx6_fb_data[] = {
	{
		.disp_dev = "lcd",
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,
		.mode_str = "VGA-XGA",
		.default_bpp = 24,
		.int_clk = false,
	}, {
		.disp_dev	= "ldb",
		.interface_pix_fmt = IPU_PIX_FMT_RGB666,
		.mode_str	= "LDB-XGA",
		.default_bpp	= 16,
		.int_clk	= false,
	}, {
		.disp_dev	= "ldb",
		.interface_pix_fmt = IPU_PIX_FMT_RGB666,
		.mode_str	= "LDB-VGA",
		.default_bpp	= 16,
		.int_clk	= false,
	},
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	.sec_ipu_id = 1,
	.sec_disp_id = 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
		.rev = 4,
		.csi_clk[0] = "clko_clk",
	}, {
		.rev = 4,
		.csi_clk[0] = "clko_clk",
	},
};

static void tx6_suspend_enter(void)
{
	/* suspend preparation */
	/* Disable AUX 5V */
}

static void tx6_suspend_exit(void)
{
	/* resume restore */
	/* Enable AUX 5V */
}
static const struct pm_platform_data mx6q_tx6_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = tx6_suspend_enter,
	.suspend_exit = tx6_suspend_exit,
};

#if defined(CONFIG_SND_SOC_SGTL5000) || defined(CONFIG_SND_SOC_SGTL5000_MODULE)
static struct regulator_consumer_supply sgtl5000_tx6_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "2-000a",
};

static struct regulator_consumer_supply sgtl5000_tx6_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "2-000a",
};

static struct regulator_consumer_supply sgtl5000_tx6_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "2-000a",
};

static struct regulator_init_data sgtl5000_tx6_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_tx6_consumer_vdda,
};

static struct regulator_init_data sgtl5000_tx6_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_tx6_consumer_vddio,
};

static struct regulator_init_data sgtl5000_tx6_vddd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_tx6_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_tx6_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 2500000,
	.gpio			= -1,
	.init_data		= &sgtl5000_tx6_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_tx6_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_tx6_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_tx6_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 1200000,
	.gpio			= -1,
	.init_data		= &sgtl5000_tx6_vddd_reg_initdata,
};

static struct platform_device sgtl5000_tx6_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sgtl5000_tx6_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_tx6_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_tx6_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_tx6_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &sgtl5000_tx6_vddd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static int imx6q_init_audio(void)
{
	mxc_register_device(&mx6_tx6_audio_device,
			    &mx6_tx6_audio_data);
	imx6q_add_imx_ssi(1, &mx6_tx6_ssi_pdata);
#if defined(CONFIG_SND_SOC_SGTL5000) || defined(CONFIG_SND_SOC_SGTL5000_MODULE)
	platform_device_register(&sgtl5000_tx6_vdda_reg_devices);
	platform_device_register(&sgtl5000_tx6_vddio_reg_devices);
	platform_device_register(&sgtl5000_tx6_vddd_reg_devices);
#endif
	return 0;
}

static struct gpio_led tx6_gpio_leds[] = {
	{
		.name = "heartbeat",
		.gpio = TX6_STK5_LED_GPIO,
		.default_trigger = "heartbeat",
	},
};

static struct gpio_led_platform_data tx6_gpio_leds_data = {
	.leds		= tx6_gpio_leds,
	.num_leds	= ARRAY_SIZE(tx6_gpio_leds),
};

static struct platform_device tx6_gpio_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data  = &tx6_gpio_leds_data,
	}
};

static struct resource tx6_pwm_resource[] __initdata = {
	{
		.start = MX6Q_PWM2_BASE_ADDR,
		.end = MX6Q_PWM2_BASE_ADDR + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MX6Q_INT_PWM2,
		.end = MX6Q_INT_PWM2,
		.flags = IORESOURCE_IRQ,
	},
};

static void tx6_enable_pwm_pad(void)
{
	mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_1__PWM2_PWMO);
}

static void tx6_disable_pwm_pad(void)
{
	mxc_iomux_v3_setup_pad(MX6Q_PAD_GPIO_1__GPIO_1_1);
}

static const struct mxc_pwm_platform_data tx6_pwm_pdata __initconst = {
	.pwmo_invert = 1,
	.enable_pwm_pad = tx6_enable_pwm_pad,
	.disable_pwm_pad = tx6_disable_pwm_pad,
};

static struct platform_pwm_backlight_data tx6_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 100,
	.dft_brightness = 100,
	.pwm_period_ns = 50000,
};

static int __init tx6_pwm_register(void)
{
	int ret;

	ret = gpio_request_one(TX6_BACKLIGHT_GPIO, GPIOF_OUT_INIT_HIGH,
			"LCD Backlight");
	if (ret)
		return ret;

	if (!imx_add_platform_device("mxc_pwm", 1, tx6_pwm_resource,
					ARRAY_SIZE(tx6_pwm_resource),
					&tx6_pwm_pdata, sizeof(tx6_pwm_pdata)))
		goto free_gpio;

	if (!imx6q_add_mxc_pwm_backlight(0, &tx6_pwm_backlight_data))
		goto free_gpio;

	return 0;

free_gpio:
	gpio_free(TX6_BACKLIGHT_GPIO);
	return ret;
}

static struct mxc_dvfs_platform_data tx6_dvfscore_data = {
#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
#else
	.reg_id = "cpu_vddgp",
	.soc_id = "cpu_vddsoc",
	.pu_id = "cpu_vddvpu",
#endif
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static iomux_v3_cfg_t mx6q_tx6_pads[] = {
	/* AUDMUX */
	MX6Q_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX6Q_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX6Q_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX6Q_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,

	/* CAN2  */
	MX6Q_PAD_KEY_COL4__CAN2_TXCAN,
	MX6Q_PAD_KEY_ROW4__CAN2_RXCAN,

	/* ECSPI1 */
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,

	/* ENET */
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_ENET_RXD0__ENET_RDATA_0,
	MX6Q_PAD_ENET_RXD1__ENET_RDATA_1,
	MX6Q_PAD_ENET_RX_ER__ENET_RX_ER,
	MX6Q_PAD_ENET_TX_EN__ENET_TX_EN,
	MX6Q_PAD_ENET_TXD0__ENET_TDATA_0,
	MX6Q_PAD_ENET_TXD1__ENET_TDATA_1,
	MX6Q_PAD_ENET_CRS_DV__ENET_RX_EN,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_ENET_REF_CLK__GPIO_1_23,	/* Connected to GPIO_16 on TX6 v1 */
	MX6Q_PAD_GPIO_16__ENET_ANATOP_ETHERNET_REF_OUT,
	MX6Q_PAD_SD3_DAT4__GPIO_7_1,		/* Phy Interrupt */
	MX6Q_PAD_SD3_DAT2__GPIO_7_6,		/* Phy reset */
	MX6Q_PAD_EIM_D20__GPIO_3_20,		/* Phy power */

	/* I2C1 */
	MX6Q_PAD_EIM_D21__I2C1_SCL,
	MX6Q_PAD_EIM_D28__I2C1_SDA,

	/* I2C3 */
	MX6Q_PAD_GPIO_3__I2C3_SCL,
	MX6Q_PAD_GPIO_6__I2C3_SDA,

	/* DISPLAY */
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN15__IPU1_DI0_PIN15,		/* DE */
	MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,		/* HSync */
	MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,		/* VSync */
	MX6Q_PAD_DI0_PIN4__IPU1_DI0_PIN4,		/* Contrast */
	MX6Q_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6Q_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6Q_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6Q_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6Q_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6Q_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6Q_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6Q_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6Q_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6Q_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6Q_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6Q_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6Q_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6Q_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6Q_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6Q_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6Q_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6Q_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6Q_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6Q_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
	MX6Q_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
	MX6Q_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
	MX6Q_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
	MX6Q_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,
	MX6Q_PAD_EIM_D29__GPIO_3_29,		/* Reset */
	MX6Q_PAD_EIM_EB3__GPIO_2_31,		/* Enable */

	/* DISP_PWM */
	MX6Q_PAD_GPIO_1__PWM2_PWMO,

	/* UART1 for debug */
	MX6Q_PAD_SD3_DAT7__UART1_TXD,
	MX6Q_PAD_SD3_DAT6__UART1_RXD,

	/* USBOTG ID pin */
	MX6Q_PAD_EIM_D23__GPIO_3_23,

	/* USB power pin */
	MX6Q_PAD_GPIO_7__GPIO_1_7,
	MX6Q_PAD_EIM_D31__GPIO_3_31,

	/* USB OC pin */
	MX6Q_PAD_GPIO_8__GPIO_1_8,
	MX6Q_PAD_EIM_D30__USBOH3_USBH1_OC,

	/* USDHC1 */
	MX6Q_PAD_SD1_CLK__USDHC1_CLK,
	MX6Q_PAD_SD1_CMD__USDHC1_CMD,
	MX6Q_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6Q_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6Q_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6Q_PAD_SD1_DAT3__USDHC1_DAT3,
	MX6Q_PAD_SD3_CMD__GPIO_7_2,		/* SD1_CD */

	/* USDHC2 */
	MX6Q_PAD_SD2_CLK__USDHC2_CLK,
	MX6Q_PAD_SD2_CMD__USDHC2_CMD,
	MX6Q_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6Q_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6Q_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6Q_PAD_SD2_DAT3__USDHC2_DAT3,
	MX6Q_PAD_SD3_CLK__GPIO_7_3,		/* SD2_CD */

	/* EDT-FT5x06 Polytouch */
#if defined(CONFIG_TOUCHSCREEN_EDT_FT5X06) || \
	defined(CONFIG_TOUCHSCREEN_EDT_FT5X06_MODULE)
	MX6Q_PAD_NANDF_CS2__GPIO_6_15, /* IRQ */
	MX6Q_PAD_EIM_A16__GPIO_2_22, /* Reset */
	MX6Q_PAD_EIM_A17__GPIO_2_21, /* Wake-Up */
#endif
};

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

void tx6_set_system_rev(void)
{
	void __iomem *anatop_base = ioremap(ANATOP_BASE_ADDR, SZ_4K);
	u32 sys_rev;

	if (anatop_base == NULL) {
		pr_err("%s: Failed to remap ANATOP_BASE\n", __func__);
		return;
	}

	/* Freescale's VPU code interprets the 'Revision' entry in
	 * /proc/cpuinfo (populated from 'system_rev') to determine which
	 * firmware version to load. On Freescale platforms this entry contains
	 * a munged value comprising base board and CPU revision information.
	 * Fake this entry to allow the broken Freescale VPU library to work on
	 * this hardware.
	 */
	sys_rev = readl(anatop_base + 0x260);
	iounmap(anatop_base);

	system_rev = ((sys_rev >> 16) & 0xff) << 12;
	system_rev |= ((sys_rev >> 8) & 0xff) << 4;
	system_rev += 0x10;
	system_rev |= sys_rev & 0xf;
}

/*!
 * Board specific initialization.
 */
static void __init tx6_board_init(void)
{
	int i;

	mxc_iomux_v3_setup_multiple_pads(mx6q_tx6_pads,
		ARRAY_SIZE(mx6q_tx6_pads));

	tx6_set_system_rev();

	gp_reg_id  = tx6_dvfscore_data.reg_id;
	soc_reg_id = tx6_dvfscore_data.soc_id;
	pu_reg_id  = tx6_dvfscore_data.pu_id;
	mx6q_tx6_init_uart();

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < ARRAY_SIZE(tx6_fb_data); i++)
			imx6q_add_ipuv3fb(i, &tx6_fb_data[i]);
	} else
		for (i = 0; i < (ARRAY_SIZE(tx6_fb_data) + 1) / 2; i++)
			imx6q_add_ipuv3fb(i, &tx6_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_imx_snvs_rtc();
	imx6q_tx6_init_edt_ft5x06();

	imx6q_add_imx_caam();

	platform_device_register(&tx6_gpio_led_device);

	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
	imx6q_add_imx_i2c(0, &mx6q_tx6_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_tx6_i2c_data);
	/* SPI */
	spi_register_board_info(tx6_spi_board_info,
				ARRAY_SIZE(tx6_spi_board_info));
	imx6q_add_ecspi(0, &mx6q_tx6_spi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_tx6_anatop_thermal_data);
	imx6_init_fec(fec_data);
	imx6q_add_pm_imx(0, &mx6q_tx6_pm_data);

	imx6q_add_sdhci_usdhc_imx(0, &mx6q_tx6_sd1_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_tx6_sd2_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	if (tx6_flexcan_init() == 0) {
		imx6q_add_flexcan0(&mx6q_tx6_flexcan_pdata[0]);
		imx6q_add_flexcan1(&mx6q_tx6_flexcan_pdata[1]);
	} else {
		imx6q_add_flexcan1(NULL);
	}
	imx6q_tx6_init_usb();
	imx6q_add_vpu();
	imx6q_init_audio();
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	tx6_pwm_register();

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_gpmi(&mx6q_gpmi_nand_platform_data);

	imx6q_add_dvfs_core(&tx6_dvfscore_data);

#ifndef CONFIG_MX6_INTER_LDO_BYPASS
	/*	mx6_cpu_regulator_init(); */
#endif

	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init tx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer tx6_timer = {
	.init   = tx6_timer_init,
};

static void __init tx6_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_TX6 data structure.
 */
MACHINE_START(TX6, "Ka-Ro i.MX 6Quad TX6")
	/* Maintainer: Ka-Ro electronics GmbH */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = tx6_board_init,
	.timer = &tx6_timer,
	.reserve = tx6_reserve,
MACHINE_END
