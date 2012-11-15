/*
 * arch/xtensa/platform/xtavnet/include/platform/serial.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001, 2006 Tensilica Inc.
 */

#ifndef __ASM_XTENSA_XTAVNET_SERIAL_H
#define __ASM_XTENSA_XTAVNET_SERIAL_H

#include <platform/hardware.h>

#define BASE_BAUD (DUART16552_XTAL_FREQ / 16)

#ifndef CONFIG_OF

#ifdef __XTENSA_EL__
#define IO_BASE_1 (DUART16552_VADDR)
#elif defined(__XTENSA_EB__)
#define IO_BASE_1 (DUART16552_VADDR + 3)
#else
#error endianess not defined
#endif

#define SERIAL_PORT_DFNS {					\
	.baud_base = BASE_BAUD,					\
	.irq = DUART16552_INTNUM,				\
	.flags = (ASYNC_BOOT_AUTOCONF|ASYNC_SKIP_TEST),		\
	.iomem_base = (u8 *) IO_BASE_1,				\
	.iomem_reg_shift = 2,					\
	.io_type = SERIAL_IO_MEM, }

#endif /* CONFIG_OF */

#endif /* __ASM_XTENSA_XTAVNET_SERIAL_H */
