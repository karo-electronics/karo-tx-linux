/*
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
#include <linux/io.h>
#include <linux/pwm.h>
#include <mach/hardware.h>
#include <linux/of.h>
#include <linux/of_platform.h>


/* i.MX1 and i.MX21 share the same PWM function block: */

#define MX1_PWMC    0x00   /* PWM Control Register */
#define MX1_PWMS    0x04   /* PWM Sample Register */
#define MX1_PWMP    0x08   /* PWM Period Register */


/* i.MX27, i.MX31, i.MX35 share the same PWM function block: */

#define MX3_PWMCR		  0x00	  /* PWM Control Register */
#define MX3_PWMSAR		  0x0C	  /* PWM Sample Register */
#define MX3_PWMPR		  0x10	  /* PWM Period Register */
#define MX3_PWMCR_PRESCALER(x)	  ((((x) - 1) & 0xFFF) << 4)
#define MX3_PWMCR_DOZEEN	  (1 << 24)
#define MX3_PWMCR_WAITEN	  (1 << 23)
#define MX3_PWMCR_DBGEN		  (1 << 22)
#define MX3_PWMCR_POUTC_MASK	  (3 << 18)
#define MX3_PWMCR_POUTC(n)	  (((n) & 3) << 18)
#define MX3_PWMCR_CLKSRC_IPG_HIGH (2 << 16)
#define MX3_PWMCR_CLKSRC_IPG	  (1 << 16)
#define MX3_PWMCR_EN		  (1 << 0)

enum imx_pwm_type {
	IMX_PWM_TYPE_MX1_MX21,
	IMX_PWM_TYPE_MX25,
	IMX_PWM_TYPE_MX27,
} imx_pwm_type_t;

struct pwm_device {
	struct list_head	node;
	struct platform_device *pdev;
	enum imx_pwm_type	pwm_type;

	const char	*label;
	struct clk	*clk;

	int		clk_enabled;
	void __iomem	*mmio_base;

	unsigned int	use_count;
	unsigned int	pwm_id;
};

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	if (pwm == NULL || period_ns == 0 || duty_ns > period_ns)
		return -EINVAL;

	if (pwm->pwm_type != IMX_PWM_TYPE_MX1_MX21) {
		unsigned long long c;
		unsigned long period_cycles, duty_cycles, prescale;
		u32 cr;

		c = clk_get_rate(pwm->clk);
		c = c * period_ns;
		do_div(c, 1000000000);
		period_cycles = c;

		prescale = period_cycles / 0x10000 + 1;

		period_cycles /= prescale;
		c = (unsigned long long)period_cycles * duty_ns;
		do_div(c, period_ns);
		duty_cycles = c;

		/*
		 * according to imx pwm RM, the real period value should be
		 * PERIOD value in PWMPR plus 2.
		 */
		dev_dbg(&pwm->pdev->dev, "period_cycles=%ld duty_cycles=%ld\n",
			period_cycles, duty_cycles);

		if (period_cycles > 2)
			period_cycles -= 2;
		else
			period_cycles = 0;

		writel(duty_cycles, pwm->mmio_base + MX3_PWMSAR);
		writel(period_cycles, pwm->mmio_base + MX3_PWMPR);

		cr = MX3_PWMCR_PRESCALER(prescale) |
			MX3_PWMCR_DOZEEN | MX3_PWMCR_WAITEN |
			MX3_PWMCR_DBGEN | MX3_PWMCR_EN;

		if (pwm->pwm_type == IMX_PWM_TYPE_MX25)
			cr |= MX3_PWMCR_CLKSRC_IPG;
		else
			cr |= MX3_PWMCR_CLKSRC_IPG_HIGH;

		writel(cr, pwm->mmio_base + MX3_PWMCR);
	} else if (pwm->pwm_type == IMX_PWM_TYPE_MX1_MX21) {
		/* The PWM subsystem allows for exact frequencies. However,
		 * I cannot connect a scope on my device to the PWM line and
		 * thus cannot provide the program the PWM controller
		 * exactly. Instead, I'm relying on the fact that the
		 * Bootloader (u-boot or WinCE+haret) has programmed the PWM
		 * function group already. So I'll just modify the PWM sample
		 * register to follow the ratio of duty_ns vs. period_ns
		 * accordingly.
		 *
		 * This is good enough for programming the brightness of
		 * the LCD backlight.
		 *
		 * The real implementation would divide PERCLK[0] first by
		 * both the prescaler (/1 .. /128) and then by CLKSEL
		 * (/2 .. /16).
		 */
		u32 max = readl(pwm->mmio_base + MX1_PWMP);
		u32 p = max * duty_ns / period_ns;
		writel(max - p, pwm->mmio_base + MX1_PWMS);
	} else {
		BUG();
	}

	return 0;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwm)
{
	int rc = 0;

	if (!pwm->clk_enabled) {
		rc = clk_prepare_enable(pwm->clk);
		if (!rc)
			pwm->clk_enabled = 1;
	}
	return rc;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	writel(0, pwm->mmio_base + MX3_PWMCR);

	if (pwm->clk_enabled) {
		clk_disable_unprepare(pwm->clk);
		pwm->clk_enabled = 0;
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

/* device type dependent stuff */
struct imx_pwm_data {
	enum imx_pwm_type pwm_type;
};

static struct imx_pwm_data imx_pwm_devdata[] = {
	[IMX_PWM_TYPE_MX1_MX21] = {
		.pwm_type = IMX_PWM_TYPE_MX1_MX21,
	},
	[IMX_PWM_TYPE_MX25] = {
		.pwm_type = IMX_PWM_TYPE_MX25,
	},
	[IMX_PWM_TYPE_MX27] = {
		.pwm_type = IMX_PWM_TYPE_MX27,
	},
};

#ifdef CONFIG_OF
static struct of_device_id imx_pwm_dt_ids[] = {
	{
		.compatible = "fsl,imx1-pwm",
		.data = &imx_pwm_devdata[IMX_PWM_TYPE_MX1_MX21],
	},
	{
		.compatible = "fsl,imx25-pwm",
		.data = &imx_pwm_devdata[IMX_PWM_TYPE_MX25],
	},
	{
		.compatible = "fsl,imx27-pwm",
		.data = &imx_pwm_devdata[IMX_PWM_TYPE_MX27],
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_pwm_dt_ids);

static int __devinit mxc_pwm_probe_dt(struct pwm_device *pwm,
				struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id =
		of_match_device(imx_pwm_dt_ids, &pdev->dev);
	struct imx_pwm_data *devdata = of_id ? of_id->data : NULL;
	const u32 *prop;

	if (!np)
		return -ENODEV;

	pwm->pwm_type = devdata->pwm_type;
	prop = of_get_property(np, "reg", NULL);
	if (!prop) {
		dev_err(&pdev->dev, "No 'reg' property assigned\n");
		return -EINVAL;
	}
	pwm->pwm_id = be32_to_cpu(*prop);

	return 0;
}
#else
static inline int mxc_pwm_probe_dt(struct pwm_device *pwm,
				struct platform_device *pdev)
{
	return -ENODEV;
}
#endif

static int __devinit mxc_pwm_probe_data(struct pwm_device *pwm,
					struct platform_device *pdev)
{
	struct imx_pwm_data *id_entry;

	if (pdev->id_entry) {
		id_entry = (struct imx_pwm_data *)pdev->id_entry->driver_data;

		pwm->pwm_type = id_entry->pwm_type;
	} else {
		dev_err(&pdev->dev, "No platform ID entry; falling back to CPU type match\n");
		if (cpu_is_mx1() || cpu_is_mx21())
			pwm->pwm_type = IMX_PWM_TYPE_MX1_MX21;
		else if (cpu_is_mx25())
			pwm->pwm_type = IMX_PWM_TYPE_MX25;
		else
			pwm->pwm_type = IMX_PWM_TYPE_MX27;
	}
	pwm->pwm_id = pdev->id;
	return 0;
}

static int __devinit mxc_pwm_probe(struct platform_device *pdev)
{
	struct pwm_device *pwm;
	struct resource *r;
	int ret = 0;

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	ret = mxc_pwm_probe_dt(pwm, pdev);
	if (ret == -ENODEV)
		ret = mxc_pwm_probe_data(pwm, pdev);
	if (ret)
		return ret;

	pwm->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(pwm->clk)) {
		ret = PTR_ERR(pwm->clk);
		dev_err(&pdev->dev, "Failed to get clock: %d\n", ret);
		goto err_free;
	}

	pwm->pdev = pdev;

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

	dev_info(&pdev->dev, "Registering PWM %d\n", pwm->pwm_id);
	platform_set_drvdata(pdev, pwm);

	mutex_lock(&pwm_lock);
	list_add_tail(&pwm->node, &pwm_list);
	mutex_unlock(&pwm_lock);

	return 0;

err_free_mem:
	release_mem_region(r->start, resource_size(r));
err_free_clk:
	clk_put(pwm->clk);
err_free:
	kfree(pwm);
	return ret;
}

static int __devexit mxc_pwm_remove(struct platform_device *pdev)
{
	struct pwm_device *pwm;
	struct resource *r;

	pwm = platform_get_drvdata(pdev);
	BUG_ON(pwm == NULL);

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

static struct platform_device_id imx_pwm_devtype[] = {
	{
		.name = "imx1-pwm",
		.driver_data = (kernel_ulong_t) &imx_pwm_devdata[IMX_PWM_TYPE_MX1_MX21],
	}, {
		.name = "imx25-pwm",
		.driver_data = (kernel_ulong_t) &imx_pwm_devdata[IMX_PWM_TYPE_MX25],
	}, {
		.name = "imx27-pwm",
		.driver_data = (kernel_ulong_t) &imx_pwm_devdata[IMX_PWM_TYPE_MX27],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, imx_pwm_devtype);

static struct platform_driver mxc_pwm_driver = {
	.driver		= {
		.name	= "mxc_pwm",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = imx_pwm_dt_ids,
#endif
	},
	.id_table	= imx_pwm_devtype,
	.probe		= mxc_pwm_probe,
	.remove		= __devexit_p(mxc_pwm_remove),
};

static int __init mxc_pwm_init(void)
{
	return platform_driver_register(&mxc_pwm_driver);
}
arch_initcall(mxc_pwm_init);

static void __exit mxc_pwm_exit(void)
{
	platform_driver_unregister(&mxc_pwm_driver);
}
module_exit(mxc_pwm_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_ALIAS("platform:mxc_pwm");
