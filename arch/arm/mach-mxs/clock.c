/*
 * Based on arch/arm/plat-omap/clock.c
 *
 * Copyright (C) 2004 - 2005 Nokia corporation
 * Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 * Modified for omap shared clock framework by Tony Lindgren <tony@atomide.com>
 * Copyright 2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

/* #define DEBUG */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/semaphore.h>
#include <linux/string.h>

#include <mach/clock.h>

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);

/*-------------------------------------------------------------------------
 * Standard clock functions defined in include/linux/clk.h
 *-------------------------------------------------------------------------*/

static void __clk_disable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;
	if (WARN_ON(!clk->usecount))
		return;

	if (!(--clk->usecount)) {
		if (clk->disable)
			clk->disable(clk);
	}
}

static int __clk_enable(struct clk *clk)
{
	if (clk == NULL)
		return 0;

	if (IS_ERR(clk))
		return -EINVAL;

	if (clk->usecount == 0) {
		if (clk->enable) {
			int ret = clk->enable(clk);
			if (ret)
				return ret;
		}
	}
	clk->usecount++;
	BUG_ON(clk->usecount == 0);

	return 0;
}

static void __clk_unprepare(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	if (clk->parent)
		__clk_unprepare(clk->parent);

	__clk_disable(clk);
}

static int __clk_prepare(struct clk *clk)
{
	int ret;

	if (clk == NULL)
		return 0;

	if (IS_ERR(clk))
		return -EINVAL;

	ret = __clk_prepare(clk->parent);
	if (ret)
		return ret;

	ret = __clk_enable(clk);
	if (ret)
		__clk_unprepare(clk->parent);

	return ret;
}

/*
 * The clk_enable/clk_disable could be called by drivers in atomic context,
 * so they should not really hold mutex.  Instead, clk_prepare/clk_unprepare
 * can hold a mutex, as the pair will only be called in non-atomic context.
 * Before migrating to common clk framework, we can have __clk_enable and
 * __clk_disable called in clk_prepare/clk_unprepare with mutex held and
 * leave clk_enable/clk_disable as the dummy functions.
 */
int clk_prepare(struct clk *clk)
{
	int ret = 0;

	if (clk == NULL)
		return 0;

	if (IS_ERR(clk))
		return -EINVAL;

	mutex_lock(&clocks_mutex);
	if (clk->prepared++ == 0) {
		ret = __clk_prepare(clk);
		if (ret)
			clk->prepared--;
	} else {
		BUG_ON(clk->prepared == 0);
	}
	mutex_unlock(&clocks_mutex);

	return ret;
}
EXPORT_SYMBOL(clk_prepare);

void clk_unprepare(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	mutex_lock(&clocks_mutex);
	if (WARN_ON(!clk->prepared && !(clk->flags & CLK_WARNED))) {
		pr_err("clk_unprepare() called without clk_prepare()\n");
		clk->flags |= CLK_WARNED;
	}
	if (WARN_ON(clk->prepared == 1 && clk->usecount > 1)) {
		pr_err("unbalanced calls (%d) to clk_enable()/clk_disable() before clk_unprepare()\n",
			clk->usecount);
		clk->flags |= CLK_WARNED;
	}
	if (!(--clk->prepared))
		__clk_unprepare(clk);
	mutex_unlock(&clocks_mutex);
}
EXPORT_SYMBOL(clk_unprepare);

int clk_enable(struct clk *clk)
{
	if (clk == NULL)
		return 0;

	if (IS_ERR(clk))
		return -EINVAL;

	if (WARN_ON(!clk->prepared && !(clk->flags & CLK_WARNED))) {
		pr_err("clk_enable() called without clk_prepare()\n");
		clk->flags |= CLK_WARNED;
	}
	clk->usecount++;
	BUG_ON(clk->usecount == 0);
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	if (WARN_ON(!clk->prepared && !(clk->flags & CLK_WARNED))) {
		pr_err("clk_disable() called without clk_prepare()\n");
		clk->flags |= CLK_WARNED;
	}
	if (WARN_ON(clk->usecount <= 1 && !(clk->flags & CLK_WARNED))) {
		pr_err("unbalanced clk_disable()\n");
		clk->flags |= CLK_WARNED;
	}
	if (!WARN_ON(clk->usecount == 0))
		clk->usecount--;
}
EXPORT_SYMBOL(clk_disable);

/* Retrieve the *current* clock rate. If the clock itself
 * does not provide a special calculation routine, ask
 * its parent and so on, until one is able to return
 * a valid clock rate
 */
unsigned long clk_get_rate(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return 0UL;

	if (clk->get_rate)
		return clk->get_rate(clk);

	return clk_get_rate(clk->parent);
}
EXPORT_SYMBOL(clk_get_rate);

/* Round the requested clock rate to the nearest supported
 * rate that is less than or equal to the requested rate.
 * This is dependent on the clock's current parent.
 */
long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (clk == NULL || IS_ERR(clk) || !clk->round_rate)
		return 0;

	return clk->round_rate(clk, rate);
}
EXPORT_SYMBOL(clk_round_rate);

/* Set the clock to the requested clock rate. The rate must
 * match a supported rate exactly based on what clk_round_rate returns
 */
int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EINVAL;

	if (clk == NULL || IS_ERR(clk) || clk->set_rate == NULL || rate == 0)
		return ret;

	mutex_lock(&clocks_mutex);
	ret = clk->set_rate(clk, rate);
	mutex_unlock(&clocks_mutex);

	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

/* Set the clock's parent to another clock source */
int clk_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;
	struct clk *old;

	if (clk == NULL || IS_ERR(clk) || parent == NULL ||
	    IS_ERR(parent) || clk->set_parent == NULL)
		return ret;

	if (clk->usecount)
		clk_prepare_enable(parent);

	mutex_lock(&clocks_mutex);
	ret = clk->set_parent(clk, parent);
	if (ret == 0) {
		old = clk->parent;
		clk->parent = parent;
	} else {
		old = parent;
	}
	mutex_unlock(&clocks_mutex);

	if (clk->usecount)
		clk_disable(old);

	return ret;
}
EXPORT_SYMBOL(clk_set_parent);

/* Retrieve the clock's parent clock source */
struct clk *clk_get_parent(struct clk *clk)
{
	struct clk *ret = NULL;

	if (clk == NULL || IS_ERR(clk))
		return ret;

	return clk->parent;
}
EXPORT_SYMBOL(clk_get_parent);
