/*
 * AM33XX mux data
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Derived from: arch/arm/mach-omap2/mux34xx.c Original copyright follows:
 *
 * Copyright (C) 2009 Nokia
 * Copyright (C) 2009 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>

#include "cm33xx.h"
#include "control.h"
#include "mux.h"

#define _AM33XX_MUXENTRY(M0, g, m0, m1, m2, m3, m4, m5, m6, m7)		\
{									\
	.reg_offset	= (AM33XX_CONTROL_PADCONF_##M0##_OFFSET),	\
	.gpio		= (g),						\
	.muxnames	= { m0, m1, m2, m3, m4, m5, m6, m7 },		\
}

/* AM33XX pin mux super set */
static struct omap_mux am33xx_muxmodes[] = {
	_AM33XX_MUXENTRY(GPMC_AD0, 0,
		"gpmc_ad0", "mmc1_dat0", NULL, NULL,
		NULL, NULL, NULL, "gpio1_0"),
	_AM33XX_MUXENTRY(GPMC_AD1, 0,
		"gpmc_ad1", "mmc1_dat1", NULL, NULL,
		NULL, NULL, NULL, "gpio1_1"),
	_AM33XX_MUXENTRY(GPMC_AD2, 0,
		"gpmc_ad2", "mmc1_dat2", NULL, NULL,
		NULL, NULL, NULL, "gpio1_2"),
	_AM33XX_MUXENTRY(GPMC_AD3, 0,
		"gpmc_ad3", "mmc1_dat3", NULL, NULL,
		NULL, NULL, NULL, "gpio1_3"),
	_AM33XX_MUXENTRY(GPMC_AD4, 0,
		"gpmc_ad4", "mmc1_dat4", NULL, NULL,
		NULL, NULL, NULL, "gpio1_4"),
	_AM33XX_MUXENTRY(GPMC_AD5, 0,
		"gpmc_ad5", "mmc1_dat5", NULL, NULL,
		NULL, NULL, NULL, "gpio1_5"),
	_AM33XX_MUXENTRY(GPMC_AD6, 0,
		"gpmc_ad6", "mmc1_dat6", NULL, NULL,
		NULL, NULL, NULL, "gpio1_6"),
	_AM33XX_MUXENTRY(GPMC_AD7, 0,
		"gpmc_ad7", "mmc1_dat7", NULL, NULL,
		NULL, NULL, NULL, "gpio1_7"),
	_AM33XX_MUXENTRY(GPMC_AD8, 0,
		"gpmc_ad8", "lcd_data16", "mmc1_dat0", "mmc2_dat4",
		"ehrpwm2a", "pr1_mii_mt0_clk", NULL, "gpio0_22"),
	_AM33XX_MUXENTRY(GPMC_AD9, 0,
		"gpmc_ad9", "lcd_data17", "mmc1_dat1", "mmc2_dat5",
		"ehrpwm2b", "pr1_mii0_col", NULL, "gpio0_23"),
	_AM33XX_MUXENTRY(GPMC_AD10, 0,
		"gpmc_ad10", "lcd_data18", "mmc1_dat2", "mmc2_dat6",
		"ehrpwm2_tripzone_input", "pr1_mii0_txen", NULL, "gpio0_26"),
	_AM33XX_MUXENTRY(GPMC_AD11, 0,
		"gpmc_ad11", "lcd_data19", "mmc1_dat3", "mmc2_dat7",
		"ehrpwm0_synco", "pr1_mii0_txd3", NULL, "gpio0_27"),
	_AM33XX_MUXENTRY(GPMC_AD12, 0,
		"gpmc_ad12", "lcd_data20", "mmc1_dat4", "mmc2_dat0",
		"eqep2a_in", "pr1_mii0_txd2", "pr1_pru0_pru_r30_14", "gpio1_12"),
	_AM33XX_MUXENTRY(GPMC_AD13, 0,
		"gpmc_ad13", "lcd_data21", "mmc1_dat5", "mmc2_dat1",
		"eqep2b_in", "pr1_mii0_txd1", "pr1_pru0_pru_r30_15", "gpio1_13"),
	_AM33XX_MUXENTRY(GPMC_AD14, 0,
		"gpmc_ad14", "lcd_data22", "mmc1_dat6", "mmc2_dat2",
		"eqep2_index", "pr1_mii0_txd0", "pr1_pru0_pru_r31_14", "gpio1_14"),
	_AM33XX_MUXENTRY(GPMC_AD15, 0,
		"gpmc_ad15", "lcd_data23", "mmc1_dat7", "mmc2_dat3",
		"eqep2_strobe", "pr1_ecap0_ecap_capin_apwm_o", "pr1_pru0_pru_r31_15", "gpio1_15"),
	_AM33XX_MUXENTRY(GPMC_A0, 0,
		"gpmc_a0", "mii2_txen", "rgmii2_tctl", "rmii2_txen",
		"gpmc_a16","pr1_mii_mt1_clk", "ehrpwm1_tripzone_input", "gpio1_16"),
	_AM33XX_MUXENTRY(GPMC_A1, 0,
		"gpmc_a1", "mii2_rxdv", "rgmii2_rctl", "mmc2_dat0",
		"gpmc_a17", "pr1_mii1_txd3", "ehrpwm0_synco", "gpio1_17"),
	_AM33XX_MUXENTRY(GPMC_A2, 0,
		"gpmc_a2", "mii2_txd3", "rgmii2_td3", "mmc2_dat1",
		"gpmc_a18", "pr1_mii1_txd2", "ehrpwm1a", "gpio1_18"),
	_AM33XX_MUXENTRY(GPMC_A3, 0,
		"gpmc_a3", "mii2_txd2", "rgmii2_td2", "mmc2_dat2",
		"gpmc_a19", "pr1_mii1_txd1", "ehrpwm1b", "gpio1_19"),
	_AM33XX_MUXENTRY(GPMC_A4, 0,
		"gpmc_a4", "mii2_txd1", "rgmii2_td1", "rmii2_txd1",
		"gpmc_a20", "pr1_mii1_txd0", "eqep1a_in", "gpio1_20"),
	_AM33XX_MUXENTRY(GPMC_A5, 0,
		"gpmc_a5", "mii2_txd0", "rgmii2_td0", "rmii2_txd0",
		"gpmc_a21", "pr1_mii1_rxd3", "eqep1b_in", "gpio1_21"),
	_AM33XX_MUXENTRY(GPMC_A6, 0,
		"gpmc_a6", "mii2_txclk", "rgmii2_tclk", "mmc2_dat4",
		"gpmc_a22", "pr1_mii1_rxd2", "eqep1_index", "gpio1_22"),
	_AM33XX_MUXENTRY(GPMC_A7, 0,
		"gpmc_a7", "mii2_rxclk", "rgmii2_rclk", "mmc2_dat5",
		"gpmc_a23", "pr1_mii1_rxd1", "eqep1_strobe", "gpio1_23"),
	_AM33XX_MUXENTRY(GPMC_A8, 0,
		"gpmc_a8", "mii2_rxd3", "rgmii2_rd3", "mmc2_dat6",
		"gpmc_a24", "pr1_mii1_rxd0", "mcasp0_aclkx", "gpio1_24"),
	_AM33XX_MUXENTRY(GPMC_A9, 0,
		"gpmc_a9", "mii2_rxd2", "rgmii2_rd2", "mmc2_dat7",
		"gpmc_a25", "pr1_mii_mr1_clk", "mcasp0_fsx", "gpio1_25"),
	_AM33XX_MUXENTRY(GPMC_A10, 0,
		"gpmc_a10", "mii2_rxd1", "rgmii2_rd1", "rmii2_rxd1",
		"gpmc_a26", "pr1_mii1_rxdv", "mcasp0_axr0", "gpio1_26"),
	_AM33XX_MUXENTRY(GPMC_A11, 0,
		"gpmc_a11", "mii2_rxd0", "rgmii2_rd0", "rmii2_rxd0",
		"gpmc_a27", "pr1_mii1_rxer", "mcasp0_axr1", "gpio1_27"),
	_AM33XX_MUXENTRY(GPMC_WAIT0, 0,
		"gpmc_wait0", "mii2_crs", "gpmc_csn4", "rmii2_crs_dv",
		"mmc1_sdcd", "pr1_mii1_col", "uart4_rxd", "gpio0_30"),
	_AM33XX_MUXENTRY(GPMC_WPN, 0,
		"gpmc_wpn", "mii2_rxerr", "gpmc_csn5", "rmii2_rxerr",
		"mmc2_sdcd", "pr1_mii1_txen", "uart4_txd", "gpio0_31"),
	_AM33XX_MUXENTRY(GPMC_BEN1, 0,
		"gpmc_ben1", "mii2_col", "gpmc_csn6", "mmc2_dat3",
		"gpmc_dir", "pr1_mii1_rxlink", "mcasp0_aclkr", "gpio1_28"),
	_AM33XX_MUXENTRY(GPMC_CSN0, 0,
		"gpmc_csn0", NULL, NULL, NULL,
		NULL, NULL, NULL, "gpio1_29"),
	_AM33XX_MUXENTRY(GPMC_CSN1, 0,
		"gpmc_csn1", "gpmc_clk", "mmc1_clk", "pr1_edio_data_in6",
		"pr1_edio_data_out6", "pr1_pru1_pru_r30_12", "pr1_pru1_pru_r31_12", "gpio1_30"),
	_AM33XX_MUXENTRY(GPMC_CSN2, 0,
		"gpmc_csn2", "gpmc_be1n", "mmc1_cmd", "pr1_edio_data_in7",
		"pr1_edio_data_out7", "pr1_pru1_pru_r30_13", "pr1_pru1_pru_r31_13", "gpio1_31"),
	_AM33XX_MUXENTRY(GPMC_CSN3, 0,
		"gpmc_csn3", NULL, NULL, "mmc2_cmd",
		"pr1_mii0_crs", "pr1_mdio_data", "emu4", "gpio2_0"),
	_AM33XX_MUXENTRY(GPMC_CLK, 0,
		"gpmc_clk", "lcd_memory_clk_mux", "gpmc_wait1", "mmc2_clk",
		"pr1_mii1_crs", "pr1_mdio_mdclk", "mcasp0_fsr", "gpio2_1"),
	_AM33XX_MUXENTRY(GPMC_ADVN_ALE, 0,
		"gpmc_advn_ale", NULL, "timer4", NULL,
		NULL, NULL, NULL, "gpio2_2"),
	_AM33XX_MUXENTRY(GPMC_OEN_REN, 0,
		"gpmc_oen_ren", NULL, "timer7", NULL,
		NULL, NULL, NULL, "gpio2_3"),
	_AM33XX_MUXENTRY(GPMC_WEN, 0,
		"gpmc_wen", NULL, "timer6", NULL,
		NULL, NULL, NULL, "gpio2_4"),
	_AM33XX_MUXENTRY(GPMC_BEN0_CLE, 0,
		"gpmc_ben0_cle", NULL, "timer5", NULL,
		NULL, NULL, NULL, "gpio2_5"),
	_AM33XX_MUXENTRY(LCD_DATA0, 0,
		"lcd_data0", "gpmc_a0", "pr1_mii_mt0_clk2", "ehrpwm2a",
		NULL, "pr1_pru1_pru_r30_02", "pr1_pru1_pru_r31_0", "gpio2_6"),
	_AM33XX_MUXENTRY(LCD_DATA1, 0,
		"lcd_data1", "gpmc_a1", "pr1_mii0_txen", "ehrpwm2b",
		NULL, "pr1_pru1_pru_r30_1", "pr1_pru1_pru_r31_1", "gpio2_7"),
	_AM33XX_MUXENTRY(LCD_DATA2, 0,
		"lcd_data2", "gpmc_a2", "pr1_mii0_txd3", "ehrpwm2_tripzone_input",
		NULL, "pr1_mii0_txd3", "ehrpwm2_tripzone_input", "gpio2_8"),
	_AM33XX_MUXENTRY(LCD_DATA3, 0,
		"lcd_data3", "gpmc_a3", "pr1_mii0_txd2", "ehrpwm0_synco",
		NULL, "pr1_pru1_pru_r30_3", "pr1_pru1_pru_r31_3", "gpio2_9"),
	_AM33XX_MUXENTRY(LCD_DATA4, 0,
		"lcd_data4", "gpmc_a4", "pr1_mii0_txd1", "eqep2a_in",
		NULL, "pr1_pru1_pru_r30_4", "pr1_pru1_pru_r31_4", "gpio2_10"),
	_AM33XX_MUXENTRY(LCD_DATA5, 0,
		"lcd_data5", "gpmc_a5", "pr1_mii0_txd0", "eqep2b_in",
		NULL, "pr1_pru1_pru_r30_5", "pr1_pru1_pru_r31_5", "gpio2_11"),
	_AM33XX_MUXENTRY(LCD_DATA6, 0,
		"lcd_data6", "gpmc_a6", "pr1_edio_data_in6", "eqep2_index",
		"pr1_edio_data_out6", "pr1_pru1_pru_r30_6", "pr1_pru1_pru_r31_6", "gpio2_12"),
	_AM33XX_MUXENTRY(LCD_DATA7, 0,
		"lcd_data7", "gpmc_a7", "pr1_edio_data_in7", "eqep2_strobe",
		"pr1_edio_data_out7", "pr1_pru1_pru_r30_7", "pr1_pru1_pru_r31_7", "gpio2_13"),
	_AM33XX_MUXENTRY(LCD_DATA8, 0,
		"lcd_data8", "gpmc_a12", "ehrpwm1_tripzone_input", "mcasp0_aclkx",
		"uart5_txd", "pr1_mii0_rxd3", "uart2_ctsn", "gpio2_14"),
	_AM33XX_MUXENTRY(LCD_DATA9, 0,
		"lcd_data9", "gpmc_a13", "ehrpwm0_synco", "mcasp0_fsx",
		"uart5_rxd", "pr1_mii0_rxd2", "uart2_rtsn", "gpio2_15"),
	_AM33XX_MUXENTRY(LCD_DATA10, 0,
		"lcd_data10", "gpmc_a14", "ehrpwm1a", "mcasp0_axr0",
		NULL, "pr1_mii0_rxd1", "uart3_ctsn", "gpio2_16"),
	_AM33XX_MUXENTRY(LCD_DATA11, 0,
		"lcd_data11", "gpmc_a15", "ehrpwm1b", "mcasp0_ahclkr",
		"mcasp0_axr2", "pr1_mii0_rxd0", "uart3_rtsn", "gpio2_17"),
	_AM33XX_MUXENTRY(LCD_DATA12, 0,
		"lcd_data12", "gpmc_a16", "eqep1a_in", "mcasp0_aclkr",
		"mcasp0_axr2", "pr1_mii0_rxlink", "uart4_ctsn", "gpio0_8"),
	_AM33XX_MUXENTRY(LCD_DATA13, 0,
		"lcd_data13", "gpmc_a17", "eqep1b_in", "mcasp0_fsr",
		"mcasp0_axr3", "pr1_mii0_rxer", "uart4_rtsn", "gpio0_9"),
	_AM33XX_MUXENTRY(LCD_DATA14, 0,
		"lcd_data14", "gpmc_a18", "eqep1_index", "mcasp0_axr1",
		"uart5_rxd", "pr1_mii_mr0_clk", "uart5_ctsn", "gpio0_10"),
	_AM33XX_MUXENTRY(LCD_DATA15, 0,
		"lcd_data15", "gpmc_a19", "eqep1_strobe", "mcasp0_ahclkx",
		"mcasp0_axr3", "pr1_mii0_rxdv", "uart5_rtsn", "gpio0_11"),
	_AM33XX_MUXENTRY(LCD_VSYNC, 0,
		"lcd_vsync", "gpmc_a8", NULL, "pr1_edio_data_in2",
		"pr1_edio_data_out2", "pr1_pru1_pru_r30_8", "pr1_pru1_pru_r31_8", "gpio2_22"),
	_AM33XX_MUXENTRY(LCD_HSYNC, 0,
		"lcd_hsync", "gpmc_a9", NULL, "pr1_edio_data_in3",
		"pr1_edio_data_out3", "pr1_pru1_pru_r30_9", "pr1_pru1_pru_r31_9", "gpio2_23"),
	_AM33XX_MUXENTRY(LCD_PCLK, 0,
		"lcd_pclk", "gpmc_a10", "pr1_mii0_crs", "pr1_edio_data_in4",
		"pr1_edio_data_out4", "pr1_pru1_pru_r30_10", "pr1_pru1_pru_r31_10", "gpio2_24"),
	_AM33XX_MUXENTRY(LCD_AC_BIAS_EN, 0,
		"lcd_ac_bias_en", "gpmc_a11", "pr1_mii1_crs", "pr1_edio_data_in5",
		"pr1_edio_data_out5", "pr1_pru1_pru_r30_11", "pr1_pru1_pru_r31_11", "gpio2_25"),
	_AM33XX_MUXENTRY(MMC0_DAT3, 0,
		"mmc0_dat3", "gpmc_a20", "uart4_ctsn", "timer5",
		"uart1_dcdn", "pr1_pru0_pru_r30_8", "pr1_pru0_pru_r31_8", "gpio2_26"),
	_AM33XX_MUXENTRY(MMC0_DAT2, 0,
		"mmc0_dat2", "gpmc_a21", "uart4_rtsn", "timer6",
		"uart1_dsrn", "pr1_pru0_pru_r30_9", "pr1_pru0_pru_r31_9", "gpio2_27"),
	_AM33XX_MUXENTRY(MMC0_DAT1, 0,
		"mmc0_dat1", "gpmc_a22", "uart5_ctsn", "uart3_rxd",
		"uart1_dtrn", "pr1_pru0_pru_r30_10", "pr1_pru0_pru_r31_10", "gpio2_28"),
	_AM33XX_MUXENTRY(MMC0_DAT0, 0,
		"mmc0_dat0", "gpmc_a23", "uart5_rtsn", "uart3_txd",
		"uart1_rin", "pr1_pru0_pru_r30_11", "pr1_pru0_pru_r31_11", "gpio2_29"),
	_AM33XX_MUXENTRY(MMC0_CLK, 0,
		"mmc0_clk", "gpmc_a24", "uart3_ctsn", "uart2_rxd",
		"d_can1_tx", "pr1_pru0_pru_r30_12", "pr1_pru0_pru_r31_12", "gpio2_30"),
	_AM33XX_MUXENTRY(MMC0_CMD, 0,
		"mmc0_cmd", "gpmc_a25", "uart3_rtsn", "uart2_txd",
		"d_can1_rx", "pr1_pru0_pru_r30_13", "pr1_pru0_pru_r31_13", "gpio2_31"),
	_AM33XX_MUXENTRY(MII1_COL, 0,
		"gmii1_col", "rmii2_refclk", "spi1_sclk", "uart5_rxd",
		"mcasp1_axr2", "mmc2_dat3", "mcasp0_axr2", "gpio3_0"),
	_AM33XX_MUXENTRY(MII1_CRS, 0,
		"gmii1_crs", "rmii1_crs_dv", "spi1_d0", "i2c1_sda",
		"mcasp1_aclkx", "uart5_ctsn", "uart2_rxd", "gpio3_1"),
	_AM33XX_MUXENTRY(MII1_RXERR, 0,
		"gmii1_rxerr", "rmii1_rxerr", "spi1_d1", "i2c1_scl",
		"mcasp1_fsx", "uart5_rtsn", "uart2_txd", "gpio3_2"),
	_AM33XX_MUXENTRY(MII1_TXEN, 0,
		"gmii1_txen", "rmii1_txen", "rgmii1_tctl", "timer4",
		"mcasp1_axr0", "eqep0_index", "mmc2_cmd", "gpio3_3"),
	_AM33XX_MUXENTRY(MII1_RXDV, 0,
		"gmii1_rxdv", "lcd_memory_clk", "rgmii1_rctl", "uart5_txd",
		"mcasp1_aclx", "mmc2_dat0", "mcasp0_aclkr", "gpio3_4"),
	_AM33XX_MUXENTRY(MII1_TXD3, 0,
		"gmii1_txd3", "d_can0_tx", "rgmii1_td3", "uart4_rxd",
		"mcasp1_fsx", "mmc2_dat1", "mcasp0_fsr", "gpio0_16"),
	_AM33XX_MUXENTRY(MII1_TXD2, 0,
		"gmii1_txd2", "d_can0_rx", "rgmii1_td2", "uart4_txd",
		"mcasp1_axr0", "mmc2_dat2", "mcasp0_ahclkx", "gpio0_17"),
	_AM33XX_MUXENTRY(MII1_TXD1, 0,
		"gmii1_txd1", "rmii1_txd1", "rgmii1_td1", "mcasp1_fsr",
		"mcasp1_axr1", "eqep0a_in", "mmc1_cmd", "gpio0_21"),
	_AM33XX_MUXENTRY(MII1_TXD0, 0,
		"gmii1_txd0", "rmii1_txd0", "rgmii1_td0", "mcasp1_axr2",
		"mcasp1_aclkr", "eqep0b_in", "mmc1_clk", "gpio0_28"),
	_AM33XX_MUXENTRY(MII1_TXCLK, 0,
		"gmii1_txclk", "uart2_rxd", "rgmii1_tclk", "mmc0_dat7",
		"mmc1_dat0", "uart1_dcdn", "mcasp0_aclkx", "gpio3_9"),
	_AM33XX_MUXENTRY(MII1_RXCLK, 0,
		"gmii1_rxclk", "uart2_txd", "rgmii1_rclk", "mmc0_dat6",
		"mmc1_dat1", "uart1_dsrn", "mcasp0_fsx", "gpio3_10"),
	_AM33XX_MUXENTRY(MII1_RXD3, 0,
		"gmii1_rxd3", "uart3_rxd", "rgmii1_rd3", "mmc0_dat5",
		"mmc1_dat2", "uart1_dtrn", "mcasp0_axr0", "gpio2_18"),
	_AM33XX_MUXENTRY(MII1_RXD2, 0,
		"gmii1_rxd2", "uart3_txd", "rgmii1_rd2", "mmc0_dat4",
		"mmc1_dat3", "uart1_rin", "mcasp0_axr1", "gpio2_19"),
	_AM33XX_MUXENTRY(MII1_RXD1, 0,
		"gmii1_rxd1", "rmii1_rxd1", "rgmii1_rd1", "mcasp1_axr3",
		"mcasp1_fsr", "eqep0_strobe", "mmc2_clk", "gpio2_20"),
	_AM33XX_MUXENTRY(MII1_RXD0, 0,
		"gmii1_rxd0", "rmii1_rxd0", "rgmii1_rd0", "mcasp1_ahclkx",
		"mcasp1_ahclkr", "mcasp1_aclkr", "mcasp0_axr3", "gpio2_21"),
	_AM33XX_MUXENTRY(MII1_REFCLK, 0,
		"rmii1_refclk", "xdma_event_intr2", "spi1_cs0", "uart5_txd",
		"mcasp1_axr3", "mmc0_pow", "mcasp1_ahclkx", "gpio0_29"),
	_AM33XX_MUXENTRY(MDIO_DATA, 0,
		"mdio_data", "timer6", "uart5_rxd", "uart3_ctsn",
		"mmc0_sdcd", "mmc1_cmd", "mmc2_cmd", "gpio0_0"),
	_AM33XX_MUXENTRY(MDIO_CLK, 0,
		"mdio_clk", "timer5", "uart5_txd", "uart3_rtsn",
		"mmc0_sdwp", "mmc1_clk", "mmc2_clk", "gpio0_1"),
	_AM33XX_MUXENTRY(SPI0_SCLK, 0,
		"spi0_sclk", "uart2_rxd", "i2c2_sda", "ehrpwm0a",
		"pr1_uart0_cts_n", "pr1_edio_sof", "emu2", "gpio0_2"),
	_AM33XX_MUXENTRY(SPI0_D0, 0,
		"spi0_d0", "uart2_txd", "i2c2_scl", "ehrpwm0b",
		"pr1_uart0_rts_n", "pr1_edio_latch_in", "emu3", "gpio0_3"),
	_AM33XX_MUXENTRY(SPI0_D1, 0,
		"spi0_d1", "mmc1_sdwp", "i2c1_sda", "ehrpwm0_tripzone_input",
		"pr1_uart0_rxd", "pr1_edio_data_in0", "pr1_edio_data_out0", "gpio0_4"),
	_AM33XX_MUXENTRY(SPI0_CS0, 0,
		"spi0_cs0", "mmc2_sdwp", "i2c1_scl", "ehrpwm0_synci",
		"pr1_uart0_txd", "pr1_edio_data_in1", "pr1_edio_data_out1", "gpio0_5"),
	_AM33XX_MUXENTRY(SPI0_CS1, 0,
		"spi0_cs1", "uart3_rxd", "ecap1_in_pwm1_out", "mmc0_pow",
		"xdma_event_intr2", "mmc0_sdcd", "emu4", "gpio0_6"),
	_AM33XX_MUXENTRY(ECAP0_IN_PWM0_OUT, 0,
		"ecap0_in_pwm0_out", "uart3_txd", "spi1_cs1", "pr1_ecap0_ecap_capin_apwm_o",
		"spi1_sclk", "mmc0_sdwp", "xdma_event_intr2", "gpio0_7"),
	_AM33XX_MUXENTRY(UART0_CTSN, 0,
		"uart0_ctsn", "uart4_rxd", "d_can1_tx", "i2c1_sda",
		"spi1_d0", "timer7", "pr1_edc_sync0_out", "gpio1_8"),
	_AM33XX_MUXENTRY(UART0_RTSN, 0,
		"uart0_rtsn", "uart4_txd", "d_can1_rx", "i2c1_scl",
		"spi1_d1", "spi1_cs0", "pr1_edc_sync1_out", "gpio1_9"),
	_AM33XX_MUXENTRY(UART0_RXD, 0,
		"uart0_rxd", "spi1_cs0", "d_can0_tx", "i2c2_sda",
		"ecap2_in_pwm2_out", "pr1_pru1_pru_r30_14", "pr1_pru1_pru_r31_14", "gpio1_10"),
	_AM33XX_MUXENTRY(UART0_TXD, 0,
		"uart0_txd", "spi1_cs1", "d_can0_rx", "i2c2_scl",
		"ecap1_in_pwm1_out", "pr1_pru1_pru_r30_15", "pr1_pru1_pru_r31_15", "gpio1_11"),
	_AM33XX_MUXENTRY(UART1_CTSN, 0,
		"uart1_ctsn", "timer6", "d_can0_tx", "i2c2_sda",
		"spi1_cs0", "pr1_uart0_cts_n", "pr1_edc_latch0_in", "gpio0_12"),
	_AM33XX_MUXENTRY(UART1_RTSN, 0,
		"uart1_rtsn", "timer5", "d_can0_rx", "i2c2_scl",
		"spi1_cs1", "pr1_uart0_rts_n", "pr1_edc_latch1_in", "gpio0_13"),
	_AM33XX_MUXENTRY(UART1_RXD, 0,
		"uart1_rxd", "mmc1_sdwp", "d_can1_tx", "i2c1_sda",
		NULL, "pr1_uart0_rxd_mux1", "pr1_pru1_pru_r31_16", "gpio0_14"),
	_AM33XX_MUXENTRY(UART1_TXD, 0,
		"uart1_txd", "mmc2_sdwp", "d_can1_rx", "i2c1_scl",
		NULL, "pr1_uart0_txd_mux1", "pr1_pru0_pru_r31_16", "gpio0_15"),
	_AM33XX_MUXENTRY(I2C0_SDA, 0,
		"i2c0_sda", "timer4", "uart2_ctsn", "ecap2_in_pwm2_out",
		NULL, NULL, NULL, "gpio3_5"),
	_AM33XX_MUXENTRY(I2C0_SCL, 0,
		"i2c0_scl", "timer7", "uart2_rtsn", "ecap1_in_pwm1_out",
		NULL, NULL, NULL, "gpio3_6"),
	_AM33XX_MUXENTRY(MCASP0_ACLKX, 0,
		"mcasp0_aclkx", "ehrpwm0a", NULL, "spi1_sclk",
		"mmc0_sdcd", "pr1_pru0_pru_r30_0", "pr1_pru0_pru_r31_0", "gpio3_14"),
	_AM33XX_MUXENTRY(MCASP0_FSX, 0,
		"mcasp0_fsx", "ehrpwm0b", NULL, "spi1_d0",
		"mmc1_sdcd", "pr1_pru0_pru_r30_1", "pr1_pru0_pru_r31_1", "gpio3_15"),
	_AM33XX_MUXENTRY(MCASP0_AXR0, 0,
		"mcasp0_axr0", "ehrpwm0_tripzone_input", NULL, "spi1_d1",
		"mmc2_sdcd", "pr1_pru0_pru_r30_2", "pr1_pru0_pru_r31_2", "gpio3_16"),
	_AM33XX_MUXENTRY(MCASP0_AHCLKR, 0,
		"mcasp0_ahclkr", "ehrpwm0_synci", "mcasp0_axr2", "spi1_cs0",
		"ecap2_in_pwm2_out", "pr1_pru0_pru_r30_3", "pr1_pru0_pru_r31_3", "gpio3_17"),
	_AM33XX_MUXENTRY(MCASP0_ACLKR, 0,
		"mcasp0_aclkr", "eqep0a_in", "mcasp0_axr2", "mcasp1_aclkx",
		"mmc0_sdwp", "pr1_pru0_pru_r30_4", "pr1_pru0_pru_r31_4", "gpio3_18"),
	_AM33XX_MUXENTRY(MCASP0_FSR, 0,
		"mcasp0_fsr", "eqep0b_in", "mcasp0_axr3", "mcasp1_fsx",
		"emu2", "pr1_pru0_pru_r30_5", "pr1_pru0_pru_r31_5", "gpio3_19"),
	_AM33XX_MUXENTRY(MCASP0_AXR1, 0,
		"mcasp0_axr1", "eqep0_index", NULL, "mcasp1_axr0",
		"emu3", "pr1_pru0_pru_r30_6", "pr1_pru0_pru_r31_6", "gpio3_20"),
	_AM33XX_MUXENTRY(MCASP0_AHCLKX, 0,
		"mcasp0_ahclkx", "eqep0_strobe", "mcasp0_axr3", "mcasp1_axr1",
		"emu4", "pr1_pru0_pru_r30_7", "pr1_pru0_pru_r31_7", "gpio3_21"),
	_AM33XX_MUXENTRY(XDMA_EVENT_INTR0, 0,
		"xdma_event_intr0", NULL, "timer4", "clkout1",
		"spi1_cs1", "pr1_pru1_pru_r31_16", "emu2", "gpio0_19"),
	_AM33XX_MUXENTRY(XDMA_EVENT_INTR1, 0,
		"xdma_event_intr1", NULL, "tclkin", "clkout2",
		"timer7", "pr1_pru0_pru_r31_16", "emu3", "gpio0_20"),
	_AM33XX_MUXENTRY(WARMRSTN, 0,
		"warmrstn", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(PWRONRSTN, 0,
		"porz", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(NMIN, 0,
		"nmin", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(XTALIN, 0,
		"osc0_in", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(XTALOUT, 0,
		"osc0_out", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(TMS, 0,
		"tms", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(TDI, 0,
		"tdi", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(TDO, 0,
		"tdo", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(TCK, 0,
		"tck", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(TRSTN, 0,
		"trstn", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(EMU0, 0,
		"emu0", NULL, NULL, NULL,
		NULL, NULL, NULL, "gpio3_7"),
	_AM33XX_MUXENTRY(EMU1, 0,
		"emu1", NULL, NULL, NULL,
		NULL, NULL, NULL, "gpio3_8"),
	_AM33XX_MUXENTRY(RTC_XTALIN, 0,
		"osc1_in", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(RTC_XTALOUT, 0,
		"osc1_out", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(RTC_PWRONRSTN, 0,
		"rtc_pwronrstn", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(PMIC_POWER_EN, 0,
		"pmic_power_en", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(EXT_WAKEUP, 0,
		"ext_wakeup", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(RTC_KALDO_ENN, 0,
		"rtc_kaldo_enn", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(USB0_DM, 0,
		"usb0_dm", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(USB0_DP, 0,
		"usb0_dp", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(USB0_CE, 0,
		"usb0_ce", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(USB0_ID, 0,
		"usb0_id", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(USB0_VBUS, 0,
		"usb0_vbus", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(USB0_DRVVBUS, 0,
		"usb0_drvvbus", NULL, NULL, NULL,
		NULL, NULL, NULL, "gpio0_18"),
	_AM33XX_MUXENTRY(USB1_DM, 0,
		"usb0_dm", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(USB1_DP, 0,
		"usb0_dp", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(USB1_CE, 0,
		"usb0_ce", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(USB1_ID, 0,
		"usb0_id", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(USB1_VBUS, 0,
		"usb0_vbus", NULL, NULL, NULL,
		NULL, NULL, NULL, NULL),
	_AM33XX_MUXENTRY(USB1_DRVVBUS, 0,
		"usb1_drvvbus", NULL, NULL, NULL,
		NULL, NULL, NULL, "gpio3_13"),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

int __init am33xx_mux_init(struct omap_board_mux *board_subset)
{
	return omap_mux_init("core", 0, AM33XX_CONTROL_PADCONF_MUX_PBASE,
			AM33XX_CONTROL_PADCONF_MUX_SIZE, am33xx_muxmodes,
			NULL, board_subset, NULL);
}

#ifdef CONFIG_SUSPEND
struct am33xx_padconf_regs {
	u16 offset;
	u32 val;
};

static struct am33xx_padconf_regs am33xx_lp_padconf[] = {
	{.offset = AM33XX_CONTROL_GMII_SEL_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A0_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A1_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A2_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A3_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A4_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A5_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A6_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A7_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A8_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A9_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A10_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_A11_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_WAIT0_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_WPN_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_GPMC_BEN1_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_COL_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_CRS_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXERR_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_TXEN_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXDV_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_TXD3_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_TXD2_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_TXD1_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_TXD0_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_TXCLK_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXCLK_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXD3_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXD2_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXD1_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_RXD0_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MII1_REFCLK_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MDIO_DATA_OFFSET},
	{.offset = AM33XX_CONTROL_PADCONF_MDIO_CLK_OFFSET},
};
#endif /* CONFIG_SUSPEND */

struct susp_io_pad_conf {
	u32 enabled;
	u32 val;
};

static u32 susp_io_pad_conf_enabled;
static struct susp_io_pad_conf pad_array[MAX_IO_PADCONF];

struct standby_gpio_pad_struct {
	u32 enabled;
	u32 gpio_request_success;
	u32 pin_val;
	u32 trigger;
	u32 gpio_pin;
	u32 curr_pin_mux;
};

static struct standby_gpio_pad_struct standby_gpio_array[MAX_IO_PADCONF];

/*
 * Expected input: 1/0
 * Example: "echo 1 > enable_suspend_io_pad_conf" enables IO PAD Config
 */
static int susp_io_pad_enable_set(void *data, u64 val)
{
	u32 *enabled = data;

	*enabled = val & 0x1;

	return 0;
}

static int susp_io_pad_enable_get(void *data, u64 *val)
{
	u32 *enabled = data;

	*val = *enabled;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(susp_io_pad_enable_fops, susp_io_pad_enable_get,
			susp_io_pad_enable_set, "%llx\n");

static unsigned int am335x_pin_mux_addr_to_skip[] = {
	0x9bc,
	0x9c4, 0x9c8, 0x9cc,
	0x9ec,
	0x9f0, 0x9f4,
	0xa08, 0xa0c,
	0xa10, 0xa14, 0xa18,
	0xa20, 0xa24, 0xa28, 0xa2c,
	0xa30,
};

static int susp_io_pad_status_show(struct seq_file *s, void *unused)
{
	struct omap_mux *mux_arr = &am33xx_muxmodes[0];
	u32 *enabled = s->private;
	int i;

	if (!*enabled) {
		pr_err("%s: IO PAD Configuration is not enabled\n", __func__);
		return 0;
	}

	for (i = 0; i < MAX_IO_PADCONF;) {
		int off, j, addr_match = 0;

		if (pad_array[i].enabled) {
			int mode = pad_array[i].val & OMAP_MUX_MODE7;
			seq_printf(s, "%s.%s (0x%08x = 0x%02x)\n",
				mux_arr->muxnames[0], mux_arr->muxnames[mode],
				(unsigned int)(AM33XX_CONTROL_PADCONF_MUX_PBASE
				+ mux_arr->reg_offset),
				pad_array[i].val);
		}

		i++;

		/*
		 * AM335x pin-mux register offset sequence is broken, meaning
		 * there is no pin-mux setting at some offset and at some
		 * offsets, the modes are not supposed to be changed. Because
		 * of this, the "am33xx_muxmodes" array above will not have any
		 * values at these indexes. Hence the pad_array &
		 * am33xx_muxmodes array will be out of sync at these index.
		 * Handle missing pin-mux entries accordingly by using a special
		 * array that indicate these offsets.
		 */
		off = ((i * 4) + 0x800);
		for (j = 0; j < ARRAY_SIZE(am335x_pin_mux_addr_to_skip); j++) {
			if (off == am335x_pin_mux_addr_to_skip[j]) {
				addr_match = 1;
				break;
			}
		}
		if (addr_match == 0)
			mux_arr++;
	}

	return 0;
}

/*
 * Expected input: pinmux_name=<value1>
 *	pinmux_name = Pin-mux name that is to be setup during suspend with value
 *		<value1>. Pin-mux name should be in "mode0_name.function_name"
 *		format. Internally the pin-mux offset is calculated from the
 *		pin-mux names. Invalid pin-mux names and values are ignored.
 *		Remember, NO spaces anywhere in the input.
 *
 * Example:
 *	  echo mcasp0_aclkx.gpio3_14=0x27 > suspend_pad_conf
 *		stores 0x27 as the value to be written to the pinmux (with
 *		mode0_name.function_name as mcasp0_aclkx.gpio3_14) when entering
 *		suspend
 */
static ssize_t susp_io_pad_write(struct file *file,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct seq_file *seqf;
	u32 *enabled, val;
	char *export_string, *token, *name;

	seqf = file->private_data;
	enabled = seqf->private;

	if (!*enabled) {
		pr_err("%s: IO PAD Configuration is not enabled\n", __func__);
		return -EINVAL;
	}

	export_string = kzalloc(count + 1, GFP_KERNEL);
	if (!export_string)
		return -ENOMEM;

	if (copy_from_user(export_string, user_buf, count)) {
		kfree(export_string);
		return -EFAULT;
	}

	token = export_string;
	name = strsep(&token, "=");
	if (name) {
		struct omap_mux_partition *partition = NULL;
		struct omap_mux *mux = NULL;
		int mux_index, mux_mode;
		int res;

		mux_mode = omap_mux_get_by_name(name, &partition, &mux);
		if (mux_mode < 0) {
			pr_err("%s: Invalid mux name (%s). Ignoring the"
					" value\n", __func__, name);
			goto err_out;
		}

		res = kstrtouint(token, 0, &val);
		if (res < 0) {
			pr_err("%s: Invalid value (%s). Ignoring\n",
						__func__, token);
			goto err_out;
		}

		mux_index = (mux->reg_offset -
			AM33XX_CONTROL_PADCONF_GPMC_AD0_OFFSET) / 4;

		if (mux_index > MAX_IO_PADCONF) {
			pr_err("%s: Invalid index (0x%x). Ignoring\n",
						__func__, mux_index);
			goto err_out;
		}

		pad_array[mux_index].enabled = true;
		pad_array[mux_index].val = val;
	} else {
		pr_err("%s: Invalid mux name (%s). Ignoring the entry\n",
						__func__, export_string);
	}

err_out:
	*ppos += count;
	kfree(export_string);
	return count;
}

static int susp_io_pad_open(struct inode *inode, struct file *file)
{
	return single_open(file, susp_io_pad_status_show, inode->i_private);
}

static const struct file_operations susp_io_pad_fops = {
	.open		= susp_io_pad_open,
	.read		= seq_read,
	.write		= susp_io_pad_write,
	.release	= single_release,
};

static int standby_gpio_status_show(struct seq_file *s, void *unused)
{
	struct omap_mux *mux_arr = &am33xx_muxmodes[0];

	int i;

	for (i = 0; i < MAX_IO_PADCONF;) {
		int off, j, addr_match = 0;
		char *trigger;

		if (standby_gpio_array[i].enabled) {
			switch (standby_gpio_array[i].trigger) {
			case IRQF_TRIGGER_RISING:
				trigger = "rising";
				break;

			case IRQF_TRIGGER_FALLING:
				trigger = "falling";
				break;

			case IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING:
			/* fall through */
			default:
				trigger = "falling_rising";
				break;
			}

			seq_printf(s, "%s.%s (0x%08x = 0x%02x), trigger = %s\n",
				mux_arr->muxnames[0],
				mux_arr->muxnames[OMAP_MUX_MODE7],
				(unsigned int)(AM33XX_CONTROL_PADCONF_MUX_PBASE
				+ mux_arr->reg_offset),
				standby_gpio_array[i].pin_val,
				trigger);


		}

		/*
		 * AM335x pin-mux register offset sequence is broken, meaning
		 * there is no pin-mux setting at some offset and at some
		 * offsets, the modes are not supposed to be changed. Because
		 * of this, the "am33xx_muxmodes" array above will not have any
		 * values at these indexes. Hence the standby_gpio_array &
		 * am33xx_muxmodes array will be out of sync at these index.
		 * Handle missing pin-mux entries accordingly by using a special
		 * array that indicate these offsets.
		 */

		i++;
		off = ((i * 4) + 0x800);
		for (j = 0; j < ARRAY_SIZE(am335x_pin_mux_addr_to_skip); j++) {
			if (off == am335x_pin_mux_addr_to_skip[j]) {
				addr_match = 1;
				break;
			}
		}
		if (addr_match == 0)
			mux_arr++;
	}

	return 0;
}

/*
 * Expected input: pinmux_name=<value1>,<trigger>
 *	pinmux_name = Pin-mux name that is to be setup as gpio during standby
 *		suspend with gpio interrupt trigger mode as per <trigger> field
 *		with value <value1>.
 *		Pin-mux name should be in "mode0_name.mode7_function_name"
 *		format. Internally the pin-mux offset is calculated from the
 *		pin-mux names. Invalid pin-mux names and values are ignored.
 *		Remember,
 *			- No spaces anywhere in the input.
 *			- <value1> field is a must
 *			- <trigger> field is a must and must be one of "rising",
 *			  "falling"
 *
 * Example:
 *	  echo uart0_rxd.gpio1_10=0x27,rising > standby_gpio_pad_conf
 *		sets up uart0_rxd.gpio1_10 for gpio mode with interrupt trigger
 *		as rising and pin-mux value as 0x27 when entering standby mode.
 */
static ssize_t standby_gpio_pad_write(struct file *file,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	u32 trigger;
	char *export_string, *token, *name;

	export_string = kzalloc(count + 1, GFP_KERNEL);
	if (!export_string)
		return -ENOMEM;

	if (copy_from_user(export_string, user_buf, count)) {
		kfree(export_string);
		return -EFAULT;
	}

	export_string[count-1] = '\0';   /* force null terminator */
	token = export_string;
	name = strsep(&token, "=");
	if (name) {
		struct omap_mux_partition *partition = NULL;
		struct omap_mux *mux = NULL;
		int mux_index, mux_mode, gpio_bank, gpio_pin, res, pin_val;
		char *gpio_name;

		mux_mode = omap_mux_get_by_name(name, &partition, &mux);
		if (mux_mode < 0) {
			pr_err("%s: Invalid mux name (%s). Ignoring the"
					" value\n", __func__, name);
			goto err_out;
		}

		name = strsep(&token, ",");
		if (!name) {
			pr_err("%s: Invalid value (%s). Ignoring\n",
				__func__, token);
			goto err_out;
		}

		res = kstrtouint(name, 0, &pin_val);
		if (res < 0) {
			pr_err("%s: Invalid pin mux value (%s). Ignoring\n",
						__func__, token);
			goto err_out;
		}

		if (token && !strncmp("rising", token, 6)) {
			trigger = IRQF_TRIGGER_RISING;
		} else if (token && !strncmp("falling", token, 7)) {
			trigger = IRQF_TRIGGER_FALLING;
		} else {
			pr_err("%s: Invalid trigger (%s). Defaulting to"
					" falling_rising\n", __func__, token);
			trigger = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
		}

		/* confirm whether a gpio pin exists here */
		gpio_name = mux->muxnames[OMAP_MUX_MODE7];

		if (!gpio_name) {
			pr_err("%s: Invalid mux name (%s)\n", __func__, name);
			goto err_out;
		} else if (strncmp(gpio_name, "gpio", 4)) {
			pr_err("%s: Invalid mux name found (%s)\n",
					__func__, gpio_name);
			goto err_out;
		}

		/*
		 * parse the string name and get the gpio bank & pin number.
		 * gpio_name will be in the format of "gpioX_Y" where
		 *	X = bank
		 *	Y = pin number
		 */
		gpio_bank = *(gpio_name + 4) - '0';

		gpio_name += 6;
		res = kstrtoint(gpio_name, 10, &gpio_pin);
		if (res < 0) {
			pr_err("%s: Invalid gpio pin number (%s). Ignoring\n",
				__func__, gpio_name);
			goto err_out;
		}

		mux_index = (mux->reg_offset -
			AM33XX_CONTROL_PADCONF_GPMC_AD0_OFFSET) / 4;

		if (mux_index > MAX_IO_PADCONF) {
			pr_err("%s: Invalid index (0x%x). Ignoring\n",
						__func__, mux_index);
			goto err_out;
		}

		standby_gpio_array[mux_index].enabled = true;
		standby_gpio_array[mux_index].pin_val = pin_val;
		standby_gpio_array[mux_index].trigger = trigger;
		standby_gpio_array[mux_index].gpio_pin =
						((gpio_bank * 32) + gpio_pin);
	} else {
		pr_err("%s: Invalid mux name (%s). Ignoring the entry\n",
						__func__, export_string);
	}

err_out:
	*ppos += count;
	kfree(export_string);
	return count;
}

static int standby_gpio_pad_open(struct inode *inode, struct file *file)
{
	return single_open(file, standby_gpio_status_show, inode->i_private);
}

static const struct file_operations standby_gpio_pad_conf_fops = {
	.open		= standby_gpio_pad_open,
	.read		= seq_read,
	.write		= standby_gpio_pad_write,
	.release	= single_release,
};

void am33xx_mux_dbg_create_entry(struct dentry *mux_dbg_dir)
{
	struct dentry *mux_dbg_suspend_io_conf_dir;

	if (!mux_dbg_dir)
		return;

	/*
	 * create a directory by the name suspend_io_pad_conf in
	 * <debugfs-mount-dir>/<mux_dbg_dir>/
	 */
	mux_dbg_suspend_io_conf_dir = debugfs_create_dir("suspend_io_pad_conf",
								mux_dbg_dir);
	if (!mux_dbg_suspend_io_conf_dir)
		return;

	memset(pad_array, 0, sizeof(pad_array));

	(void)debugfs_create_file("enable_suspend_io_pad_conf",
						S_IRUGO | S_IWUSR,
						mux_dbg_suspend_io_conf_dir,
						&susp_io_pad_conf_enabled,
						&susp_io_pad_enable_fops);
	(void)debugfs_create_file("suspend_pad_conf", S_IRUGO | S_IWUSR,
						mux_dbg_suspend_io_conf_dir,
						&susp_io_pad_conf_enabled,
						&susp_io_pad_fops);
	(void)debugfs_create_file("standby_gpio_pad_conf", S_IRUGO | S_IWUSR,
						mux_dbg_dir,
						&susp_io_pad_conf_enabled,
						&standby_gpio_pad_conf_fops);
}

void am33xx_setup_pinmux_on_suspend(void)
{
	u32 reg_off, i;

	if (susp_io_pad_conf_enabled == 1) {
		reg_off = AM33XX_CONTROL_PADCONF_GPMC_AD0_OFFSET;
		for (i = 0; i < MAX_IO_PADCONF; reg_off += 4, i++) {
			if (pad_array[i].enabled)
				writel(pad_array[i].val,
						AM33XX_CTRL_REGADDR(reg_off));
		}
	}
}

static u32 am33xx_lp_padconf_complete[MAX_IO_PADCONF];

void am335x_save_padconf(void)
{
	struct am33xx_padconf_regs *temp = am33xx_lp_padconf;
	u32 reg_off;
	int i;
	if (susp_io_pad_conf_enabled == 1) {
		i = AM33XX_CONTROL_PADCONF_GPMC_AD0_OFFSET;
		reg_off = 0;

		for (; i < AM33XX_CONTROL_PADCONF_MUX_SIZE; i += 4, reg_off++)
			am33xx_lp_padconf_complete[reg_off] =
						readl(AM33XX_CTRL_REGADDR(i));
	} else {
		for (i = 0; i < ARRAY_SIZE(am33xx_lp_padconf); i++, temp++)
			temp->val = readl(AM33XX_CTRL_REGADDR(temp->offset));
	}
}

void am335x_restore_padconf(void)
{
	struct am33xx_padconf_regs *temp = am33xx_lp_padconf;
	u32 reg_off;
	int i;
	if (susp_io_pad_conf_enabled == 1) {
		i = AM33XX_CONTROL_PADCONF_GPMC_AD0_OFFSET;
		reg_off = 0;
		for (; i < AM33XX_CONTROL_PADCONF_MUX_SIZE; i += 4, reg_off++)
			writel(am33xx_lp_padconf_complete[reg_off],
							AM33XX_CTRL_REGADDR(i));
	} else {
		for (i = 0; i < ARRAY_SIZE(am33xx_lp_padconf); i++, temp++)
			writel(temp->val, AM33XX_CTRL_REGADDR(temp->offset));
	}
}

/*
 * Dummy GPIO interrupt Handler
 */
static irqreturn_t gpio_irq(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

void am33xx_standby_setup(unsigned int state)
{
	u32 reg_off, i;

	if (state != PM_SUSPEND_STANDBY)
		return;

	writel(0x2, AM33XX_CM_PER_GPIO1_CLKCTRL);
	writel(0x2, AM33XX_CM_PER_GPIO2_CLKCTRL);
	writel(0x2, AM33XX_CM_PER_GPIO3_CLKCTRL);

	reg_off = AM33XX_CONTROL_PADCONF_GPMC_AD0_OFFSET;
	for (i = 0; i < MAX_IO_PADCONF; reg_off += 4, i++) {
		if (standby_gpio_array[i].enabled) {
			int ret, reg_val, irq;
			u32 gpio_pin = standby_gpio_array[i].gpio_pin;

			reg_val = readl(AM33XX_CTRL_REGADDR(reg_off));
			standby_gpio_array[i].curr_pin_mux = reg_val;
			reg_val = standby_gpio_array[i].pin_val;
			writel(reg_val, AM33XX_CTRL_REGADDR(reg_off));

			ret = gpio_request(gpio_pin, "pm_standby");
			if (ret) {
				pr_err("%s: Error in gpio request (%d)\n",
						__func__, ret);
				continue;
			}
			irq = gpio_to_irq(gpio_pin);
			if (irq < 0) {
				gpio_free(gpio_pin);
				pr_err("%s: gpio_to_irq failed (%d)\n",
								__func__, irq);
				continue;
			}
			ret = request_irq(irq, gpio_irq,
						standby_gpio_array[i].trigger,
						"pm_standby", NULL);
			if (ret) {
				gpio_free(gpio_pin);
				pr_err("%s: interrupt request failed (%d)\n",
								__func__, ret);
				continue;
			}

			standby_gpio_array[i].gpio_request_success = true;
		}
	}

}

void am33xx_standby_release(unsigned int state)
{
	u32 reg_off, i;

	if (state != PM_SUSPEND_STANDBY)
		return;

	reg_off = AM33XX_CONTROL_PADCONF_GPMC_AD0_OFFSET;
	for (i = 0; i < MAX_IO_PADCONF; reg_off += 4, i++) {
		u32 gpio_pin = standby_gpio_array[i].gpio_pin;

		if (standby_gpio_array[i].enabled) {
			writel(standby_gpio_array[i].curr_pin_mux,
						AM33XX_CTRL_REGADDR(reg_off));

			if (standby_gpio_array[i].gpio_request_success ==
									true) {
				int irq;

				irq = gpio_to_irq(gpio_pin);
				gpio_free(gpio_pin);
				free_irq(irq, 0);
			}
		}
	}
}
