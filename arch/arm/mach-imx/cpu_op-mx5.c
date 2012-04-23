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

static struct cpu_op mx51_cpu_op[] = {
	{
		.cpu_rate = 160000000,
	},
	{
		.cpu_rate = 800000000,
	},
};

static struct cpu_op mx53_cpu_op[] = {
	{
		.cpu_rate = 160000000,
	},
	{
		.cpu_rate = 400000000,
	},
	{
		.cpu_rate = 800000000,
	},
	{
		.cpu_rate = 1000000000,
	},
	{
		.cpu_rate = 1300000000,
	},
};

struct cpu_op *mx51_get_cpu_op(int *op)
{
	*op = ARRAY_SIZE(mx51_cpu_op);
	return mx51_cpu_op;
}

struct cpu_op *mx53_get_cpu_op(int *op)
{
	struct clk *cpu_clk = clk_get(NULL, "cpu_clk");
	unsigned long max_freq;
	int i;

	if (IS_ERR(cpu_clk))
		return NULL;

	max_freq = clk_get_rate(cpu_clk->parent);
	pr_info("Max. CPU clock is: %lu.%03luMHz\n",
		max_freq / 1000000, max_freq / 1000 % 1000);

	for (i = 0; i < ARRAY_SIZE(mx53_cpu_op); i++) {
		if (mx53_cpu_op[i].cpu_rate > max_freq) {
			mx53_cpu_op[i].cpu_rate = 0;
			break;
		}
	}
	*op = i;
	return mx53_cpu_op;
}
