/*
 * drivers/video/backlight/of_gpio_lcd.c
 * Generic GPIO driven LCD power control interface.
 *
 * Copyright 2012 Lothar Wassmann <LW@KARO-electronics.de>
 * base on: drivers/video/backlight/platform_lcd.c
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/lcd.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/gpio-switch.h>

#include <video/platform_lcd.h>

struct of_gpio_lcd {
	struct device		*us;
	struct lcd_device	*lcd;
	struct plat_lcd_data	*pdata;

	unsigned int		 power;
	unsigned int		 suspended : 1;

	struct device_node	*fb_id;
	struct gpio_sw		*power_switch;
	struct gpio_sw		*reset_switch;
	unsigned long		 reset_delay_us;
};

static inline struct of_gpio_lcd *to_our_lcd(struct lcd_device *lcd)
{
	return lcd_get_data(lcd);
}

static int of_gpio_lcd_get_power(struct lcd_device *lcd)
{
	struct of_gpio_lcd *oflcd = to_our_lcd(lcd);

	return oflcd->power;
}

static int of_gpio_lcd_set_power(struct lcd_device *lcd, int power)
{
	struct of_gpio_lcd *oflcd = to_our_lcd(lcd);
	int lcd_power = 1;

	if (power == FB_BLANK_POWERDOWN || oflcd->suspended)
		lcd_power = 0;

	dev_dbg(oflcd->us, "%s: Switching LCD power %s\n", __func__,
		lcd_power ? "on" : "off");
	if (lcd_power) {
		gpio_switch_set(oflcd->power_switch, lcd_power);
		if (oflcd->reset_delay_us) {
			dev_dbg(oflcd->us, "Delaying for %luus\n",
				oflcd->reset_delay_us);
			udelay(oflcd->reset_delay_us);
		}
		gpio_switch_set(oflcd->reset_switch, 0);
	} else {
		gpio_switch_set(oflcd->power_switch, 0);
		gpio_switch_set(oflcd->reset_switch, 1);
	}
	oflcd->power = power;

	return 0;
}

static int of_gpio_lcd_match(struct lcd_device *lcd, struct fb_info *info)
{
	struct of_gpio_lcd *oflcd = to_our_lcd(lcd);
	struct device_node *np = info->device->of_node;

	if (!np && info->device->parent)
		np = info->device->parent->of_node;

	dev_dbg(oflcd->us, "%s: fb_id=%p of_node=%p\n", __func__,
		oflcd->fb_id, np);

	return np == oflcd->fb_id;
}

static struct lcd_ops of_gpio_lcd_ops = {
	.get_power	= of_gpio_lcd_get_power,
	.set_power	= of_gpio_lcd_set_power,
	.check_fb	= of_gpio_lcd_match,
};

static int __devinit of_gpio_lcd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct of_gpio_lcd *oflcd;
	int err;
	const u32 *prop;
	const phandle *ph;

	pr_info("%s: \n", __func__);
	if (!np) {
		dev_err(dev, "No OF resources supplied\n");
		return -EINVAL;
	}

	if (!of_device_is_available(np)) {
		return -ENODEV;
	}

	oflcd = devm_kzalloc(dev, sizeof(struct of_gpio_lcd), GFP_KERNEL);
	if (!oflcd) {
		dev_err(dev, "No memory for driver data\n");
		return -ENOMEM;
	}

	ph = of_get_property(np, "parent", NULL);
	if (ph == NULL) {
		dev_err(dev,
			"No 'parent' handle found in device tree\n");
		return -EINVAL;
	}
	dev_dbg(dev, "Found PARENT handle: %08x\n", be32_to_cpu(*ph));

	oflcd->fb_id = of_find_node_by_phandle(be32_to_cpu(*ph));
	if (oflcd->fb_id == NULL) {
		dev_err(dev,
			"referenced fb not found in device tree\n");
		return -EINVAL;
	}
	dev_dbg(dev, "%s: got fb_id %p\n", __func__, oflcd->fb_id);

	ph = of_get_property(np, "power-switch", NULL);
	if (ph)
		oflcd->power_switch = request_gpio_switch(&pdev->dev,
							be32_to_cpu(*ph));

	ph = of_get_property(np, "reset-switch", NULL);
	if (ph)
		oflcd->reset_switch = request_gpio_switch(&pdev->dev,
							be32_to_cpu(*ph));

	prop = of_get_property(np, "reset-delay-us", NULL);
	if (prop) {
		oflcd->reset_delay_us = be32_to_cpu(*prop);
		dev_dbg(dev, "reset-delay-us set to %lu\n",
			oflcd->reset_delay_us);
	}

	oflcd->us = dev;
	oflcd->lcd = lcd_device_register(dev_name(dev), dev,
					oflcd, &of_gpio_lcd_ops);
	if (IS_ERR(oflcd->lcd)) {
		dev_err(dev, "cannot register lcd device\n");
		err = PTR_ERR(oflcd->lcd);
		free_gpio_switch(oflcd->power_switch);
		free_gpio_switch(oflcd->reset_switch);
		of_node_put(oflcd->fb_id);
		return err;
	}

	platform_set_drvdata(pdev, oflcd);
	of_gpio_lcd_set_power(oflcd->lcd, FB_BLANK_NORMAL);

	return 0;
}

static int __devexit of_gpio_lcd_remove(struct platform_device *pdev)
{
	struct of_gpio_lcd *oflcd = platform_get_drvdata(pdev);

	of_gpio_lcd_set_power(oflcd->lcd, FB_BLANK_POWERDOWN);

	lcd_device_unregister(oflcd->lcd);
	of_node_put(oflcd->fb_id);

	free_gpio_switch(oflcd->power_switch);
	free_gpio_switch(oflcd->reset_switch);

	return 0;
}

#ifdef CONFIG_PM
static int of_gpio_lcd_suspend(struct device *dev)
{
	struct of_gpio_lcd *oflcd = dev_get_drvdata(dev);

	oflcd->suspended = 1;
	of_gpio_lcd_set_power(oflcd->lcd, oflcd->power);

	return 0;
}

static int of_gpio_lcd_resume(struct device *dev)
{
	struct of_gpio_lcd *oflcd = dev_get_drvdata(dev);

	oflcd->suspended = 0;
	of_gpio_lcd_set_power(oflcd->lcd, oflcd->power);

	return 0;
}

static struct dev_pm_ops of_gpio_lcd_pm_ops = {
	.suspend = of_gpio_lcd_suspend,
	.resume = of_gpio_lcd_resume,
};

#define __dev_pm_ops_p(p)	&(p)

#else
#define of_gpio_lcd_suspend NULL
#define of_gpio_lcd_resume NULL
#define __dev_pm_ops_p(p)	NULL
#endif

static struct of_device_id of_gpio_lcd_dt_ids[] = {
	{ .compatible = "of-gpio-lcd", },
	{ /* sentinel */ }
};

static struct platform_driver of_gpio_lcd_driver = {
	.driver		= {
		.name	= "of-gpio-lcd",
		.owner	= THIS_MODULE,
		.pm = __dev_pm_ops_p(of_gpio_lcd_pm_ops),
		.of_match_table = of_gpio_lcd_dt_ids,
	},
	.probe		= of_gpio_lcd_probe,
	.remove		= __devexit_p(of_gpio_lcd_remove),
};

module_platform_driver(of_gpio_lcd_driver);

MODULE_AUTHOR("Lothar Wassmann <LW@KARO-electronics.de>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:of-gpio-lcd");
