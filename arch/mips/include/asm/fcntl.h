/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1995, 96, 97, 98, 99, 2003, 05 Ralf Baechle
 */
#ifndef _ASM_FCNTL_H
#define _ASM_FCNTL_H

#include <uapi/asm/fcntl.h>

/*
 * The flavours of struct flock.  "struct flock" is the ABI compliant
 * variant.  Finally struct flock64 is the LFS variant of struct flock.	 As
 * a historic accident and inconsistence with the ABI definition it doesn't
 * contain all the same fields as struct flock.
 */

#ifdef CONFIG_32BIT
#include <linux/types.h>

struct flock {
	short	l_type;
	short	l_whence;
	off_t	l_start;
	off_t	l_len;
	long	l_sysid;
	__kernel_pid_t l_pid;
	long	pad[4];
};

#define HAVE_ARCH_STRUCT_FLOCK

#endif /* CONFIG_32BIT */

#include <asm-generic/fcntl.h>

#endif /* _ASM_FCNTL_H */
