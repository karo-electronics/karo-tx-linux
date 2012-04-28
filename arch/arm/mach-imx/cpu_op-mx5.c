/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/types.h>
#include <mach/hardware.h>
#include <mach/clock.h>

static struct cpu_op mx5_cpu_op[] = {
	{ .cpu_rate = 800000000 / 8, },
	{ .cpu_rate = 800000000 / 7, },
	{ .cpu_rate = 800000000 / 6, },
	{ .cpu_rate = 800000000 / 5, },
	{ .cpu_rate = 800000000 / 4, },
	{ .cpu_rate = 800000000 / 3, },
	{ .cpu_rate = 800000000 / 2, },
	{ .cpu_rate = 800000000 / 1, },
	{ /* sentinel */ }
};

struct cpu_op *mxc_get_cpu_freq_table(void)
{
	struct clk *cpu_clk = clk_get(NULL, "cpu_clk");
	unsigned long max_freq;
	int i;

	if (IS_ERR(cpu_clk))
		return NULL;

	max_freq = clk_get_rate(cpu_clk->parent);

	for (i = 0; i < ARRAY_SIZE(mx5_cpu_op) - 1; i++)
		mx5_cpu_op[i].cpu_rate = max_freq / (8 - i);

	return mx5_cpu_op;
}
