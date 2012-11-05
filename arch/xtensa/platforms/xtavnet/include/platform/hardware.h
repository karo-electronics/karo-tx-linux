/*
 * arch/xtensa/platform/xtavnet/include/platform/hardware.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2006 Tensilica Inc.
 */

/*
 * This file contains the hardware configuration of the XTAVNET boards.
 */

#ifndef __XTENSA_XTAVNET_HARDWARE_H
#define __XTENSA_XTAVNET_HARDWARE_H
#include <asm/processor.h>
#include <platform/system.h>

/* By default NO_IRQ is defined to 0 in Linux, but we use the
   interrupt 0 for UART... */
#define NO_IRQ                 -1

/* Memory configuration. */

#define PLATFORM_DEFAULT_MEM_START 0x00000000
#define PLATFORM_DEFAULT_MEM_SIZE  0x04000000

/* Interrupt configuration. */

#define PLATFORM_NR_IRQS	10

/* Default assignment of LX60 devices to external interrupts. */

/*  UART interrupt: */
#define DUART16552_INTNUM	XCHAL_EXTINT0_NUM

/*
 *  Device addresses and parameters.
 */

/* UART crystal frequency in Hz */
#define DUART16552_XTAL_FREQ	(CONFIG_XTENSA_CPU_CLOCK * 1000000)

/* UART */
#define DUART16552_VADDR	(XSHAL_IOBLOCK_BYPASS_VADDR+0xD050020)
/* LCD instruction and data addresses. */
#define LCD_INSTR_ADDR		(char *)(XSHAL_IOBLOCK_BYPASS_VADDR + 0xD040000)
#define LCD_DATA_ADDR		(char *)(XSHAL_IOBLOCK_BYPASS_VADDR + 0xD040004)
#define DIP_SWITCHES_ADDR	(XSHAL_IOBLOCK_BYPASS_VADDR+0xD02000C)

#endif /* __XTENSA_XTAVNET_HARDWARE_H */
