/*
 * Copyright (C) 2010 Pengutronix
 * Sascha Hauer <s.hauer@pengutronix.de>
 *
 * simple driver for PWM (Pulse Width Modulator) controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Derived from pxa PWM driver by eric miao <eric.miao@marvell.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <mach/hardware.h>
#include <mach/mxs.h>
#include <mach/mx23.h>
#include <mach/mx28.h>
#include <asm/div64.h>

struct pwm_device {
	struct list_head	node;
	struct device		*dev;

	const char	*label;
	struct clk	*clk;

	int		enabled;
	void __iomem	*mmio_base;

	unsigned int	use_count;
	unsigned int	pwm_id;

	u32		val_active;
	u32		val_period;
	int		period_us;
};

/* common register space */
static void __iomem *pwm_base_common;
#define REG_PWM_CTRL	0x0
#define PWM_SFTRST	(1 << 31)
#define PWM_CLKGATE	(1 << 30)
#define PWM_ENABLE(p)	(1 << (p))

/* per pwm register space */
#define REG_ACTIVE	0x0
#define REG_PERIOD	0x10

#define PERIOD_PERIOD(p)	((p) & 0xffff)
#define PERIOD_ACTIVE_HIGH	(3 << 16)
#define PERIOD_INACTIVE_LOW	(2 << 18)
#define PERIOD_CDIV(div)	(((div) & 0x7) << 20)

static void pwm_update(struct pwm_device *pwm)
{
	writel(pwm->val_active, pwm->mmio_base + REG_ACTIVE);
	writel(pwm->val_period, pwm->mmio_base + REG_PERIOD);
}

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	int div = 0;
	unsigned long rate;
	unsigned long long c;
	unsigned long period_cycles, duty_cycles;

	if (pwm == NULL || period_ns == 0 || duty_ns > period_ns)
		return -EINVAL;

	rate = clk_get_rate(pwm->clk);

	dev_dbg(pwm->dev, "config: duty_ns: %d, period_ns: %d (clkrate %ld)\n",
			duty_ns, period_ns, rate);

	while (1) {
		c = rate / (1 << div);
		c = c * period_ns;
		do_div(c, 1000000000);
		if (c < 0x10000)
			break;
		div++;

		if (div > 8)
			return -EINVAL;
	}

	period_cycles = c;

	c *= duty_ns;
	do_div(c, period_ns);
	duty_cycles = c;

	dev_dbg(pwm->dev, "config period_cycles: %ld duty_cycles: %ld\n",
			period_cycles, duty_cycles);

	pwm->val_active = period_cycles << 16 | duty_cycles;
	pwm->val_period = PERIOD_PERIOD(period_cycles) | PERIOD_ACTIVE_HIGH |
			PERIOD_INACTIVE_LOW | PERIOD_CDIV(div);
	pwm->period_us = period_ns / 1000;

	pwm_update(pwm);

	return 0;
}
EXPORT_SYMBOL(pwm_config);

static void __pwm_enable(struct pwm_device *pwm, int enable)
{
	if (enable)
		__mxs_setl(PWM_ENABLE(pwm->pwm_id), pwm_base_common + REG_PWM_CTRL);
	else
		__mxs_clrl(PWM_ENABLE(pwm->pwm_id), pwm_base_common + REG_PWM_CTRL);
}

int pwm_enable(struct pwm_device *pwm)
{
	int rc = 0;

	dev_dbg(pwm->dev, "enable\n");

	if (!pwm->enabled) {
		rc = clk_enable(pwm->clk);
		if (!rc) {
			pwm->enabled = 1;
			__pwm_enable(pwm, 1);
			pwm_update(pwm);
		}
	}
	return rc;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	dev_dbg(pwm->dev, "disable\n");

	if (pwm->enabled) {
		/*
		 * We need a little delay here, it takes one period for
		 * the last pwm_config call to take effect. If we disable
		 * the pwm too early it just freezes the current output
		 * state.
		 */
		usleep_range(pwm->period_us, pwm->period_us * 2);
		__pwm_enable(pwm, 0);
		clk_disable(pwm->clk);
		pwm->enabled = 0;
	}
}
EXPORT_SYMBOL(pwm_disable);

static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_list);

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	struct pwm_device *pwm;
	int found = 0;

	mutex_lock(&pwm_lock);

	list_for_each_entry(pwm, &pwm_list, node) {
		if (pwm->pwm_id == pwm_id) {
			found = 1;
			break;
		}
	}

	if (found) {
		if (pwm->use_count == 0) {
			pwm->use_count++;
			pwm->label = label;
		} else
			pwm = ERR_PTR(-EBUSY);
	} else
		pwm = ERR_PTR(-ENOENT);

	mutex_unlock(&pwm_lock);

	return pwm;
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwm)
{
	mutex_lock(&pwm_lock);

	if (pwm->use_count) {
		pwm->use_count--;
		pwm->label = NULL;
	} else
		pr_warning("PWM device already freed\n");

	mutex_unlock(&pwm_lock);
}
EXPORT_SYMBOL(pwm_free);

static int __devinit mxs_pwm_probe(struct platform_device *pdev)
{
	struct pwm_device *pwm;
	struct resource *r;
	int ret = 0;

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pwm->clk = clk_get(&pdev->dev, NULL);

	if (IS_ERR(pwm->clk)) {
		ret = PTR_ERR(pwm->clk);
		goto err_free;
	}

	pwm->enabled = 0;

	pwm->use_count = 0;
	pwm->pwm_id = pdev->id;
	pwm->dev = &pdev->dev;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free_clk;
	}

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free_clk;
	}

	pwm->mmio_base = ioremap(r->start, resource_size(r));
	if (pwm->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	mutex_lock(&pwm_lock);
	list_add_tail(&pwm->node, &pwm_list);
	mutex_unlock(&pwm_lock);

	platform_set_drvdata(pdev, pwm);
	return 0;

err_free_mem:
	release_mem_region(r->start, resource_size(r));
err_free_clk:
	clk_put(pwm->clk);
err_free:
	kfree(pwm);
	return ret;
}

static int __devexit mxs_pwm_remove(struct platform_device *pdev)
{
	struct pwm_device *pwm;
	struct resource *r;

	pwm = platform_get_drvdata(pdev);
	if (pwm == NULL)
		return -ENODEV;

	mutex_lock(&pwm_lock);
	list_del(&pwm->node);
	mutex_unlock(&pwm_lock);

	iounmap(pwm->mmio_base);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, resource_size(r));

	clk_put(pwm->clk);

	kfree(pwm);
	return 0;
}

static struct platform_driver mxs_pwm_driver = {
	.driver		= {
		.name	= "mxs-pwm",
	},
	.probe		= mxs_pwm_probe,
	.remove		= __devexit_p(mxs_pwm_remove),
};

static int __init mxs_pwm_init(void)
{
	if (cpu_is_mx28())
		pwm_base_common = MX28_IO_ADDRESS(MX28_PWM_BASE_ADDR);
	else
		pwm_base_common = MX23_IO_ADDRESS(MX23_PWM_BASE_ADDR);

	__mxs_clrl(PWM_SFTRST | PWM_CLKGATE, pwm_base_common + REG_PWM_CTRL);

	return platform_driver_register(&mxs_pwm_driver);
}
arch_initcall(mxs_pwm_init);

static void __exit mxs_pwm_exit(void)
{
	platform_driver_unregister(&mxs_pwm_driver);
}
module_exit(mxs_pwm_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
