/*
 * drivers/?/gpio-switch.c
 *
 * Copyright (C) 2012  Lothar Wassmann <LW@KARO-electronics.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * Provide a generic interface for drivers that require to switch some
 * external hardware such as transceivers, power enable or reset pins
 * for the device they manage to work.
 *
 * Allows multiple devices to share a common switch.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio-switch.h>

static LIST_HEAD(gpio_switch_list);
static DEFINE_MUTEX(gpio_switch_list_lock);
static int gpio_switch_index;

/* Helper functions; must be called with 'gpio_switch_list_lock' held */
static struct gpio_sw *gpio_switch_find_gpio(struct device *parent,
					int gpio)
{
	struct gpio_sw *sw;

	list_for_each_entry(sw, &gpio_switch_list, list) {
		if (gpio_is_valid(gpio) && sw->gpio != gpio)
			continue;
		return sw;
	}
	return NULL;
}

static struct gpio_sw *gpio_switch_find_by_phandle(struct device *parent,
					phandle ph)
{
	struct gpio_sw *sw;
	struct device_node *dp;

	dp = of_find_node_by_phandle(ph);
	if (dp == NULL)
		return NULL;

	list_for_each_entry(sw, &gpio_switch_list, list) {
		if (sw->parent && sw->parent->of_node == dp)
			return sw;
	}
	return NULL;
}

static struct gpio_sw *gpio_switch_find_by_id(struct device *parent,
					int id)
{
	struct gpio_sw *sw;

	dev_dbg(parent, "Searching for gpio switch %u\n", id);
	list_for_each_entry(sw, &gpio_switch_list, list) {
		if (sw->parent && to_platform_device(sw->parent)->id == id)
			return sw;
	}
	return NULL;
}

static void gpio_switch_delete(struct gpio_sw *sw)
{
	list_del_init(&sw->list);
	if (!(sw->flags & GPIO_SW_SHARED) ||
	    !gpio_switch_find_gpio(sw->parent, sw->gpio))
		gpio_free(sw->gpio);
}

/* Provider API */
int gpio_switch_register(struct device *parent, const char *id, int gpio,
			 enum gpio_sw_flags flags)
{
	int ret;
	struct platform_device *pdev;
	struct gpio_sw_platform_data *pdata;

	if (!gpio_is_valid(gpio)) {
		dev_err(parent, "Invalid GPIO %u\n", gpio);
		return -EINVAL;
	}

	pdata = devm_kzalloc(parent, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->gpio = gpio;
	pdata->flags = flags;

	mutex_lock(&gpio_switch_list_lock);
	pdev = platform_device_alloc("gpio-switch", gpio_switch_index++);
	mutex_unlock(&gpio_switch_list_lock);
	if (pdev == NULL)
		return -ENOMEM;

	pdev->dev.parent = parent;
	pdev->dev.platform_data = pdata;

	ret = platform_device_add(pdev);
	if (ret)
		goto pdev_free;

	return 0;

 pdev_free:
	platform_device_put(pdev);
	return ret;
}
EXPORT_SYMBOL(gpio_switch_register);

int gpio_switch_unregister(struct gpio_sw *sw)
{
	int ret = -EINVAL;
	struct gpio_sw *ptr;

	mutex_lock(&gpio_switch_list_lock);
	list_for_each_entry(ptr, &gpio_switch_list, list) {
		if (sw == ptr) {
			gpio_switch_delete(sw);
			ret = 0;
			break;
		}
	}
	mutex_unlock(&gpio_switch_list_lock);
	return ret;
}
EXPORT_SYMBOL(gpio_switch_unregister);

/* Consumer API */
struct gpio_sw *request_gpio_switch(struct device *dev, u32 id)
{
	struct gpio_sw *sw;
	struct device_node *np = dev->of_node;

	mutex_lock(&gpio_switch_list_lock);
	if (np) {
		sw = gpio_switch_find_by_phandle(dev, id);
	} else {
		sw = gpio_switch_find_by_id(dev, id);
	}
	if (sw) {
		sw->use_count++;
	}
	mutex_unlock(&gpio_switch_list_lock);

	return sw;
}
EXPORT_SYMBOL(request_gpio_switch);

void free_gpio_switch(struct gpio_sw *sw)
{
	if (!sw)
		return;

	mutex_lock(&gpio_switch_list_lock);
	sw->use_count--;
	mutex_unlock(&gpio_switch_list_lock);
}
EXPORT_SYMBOL(free_gpio_switch);

void __gpio_switch_set(struct gpio_sw *sw, int on)
{
	gpio_set_value(sw->gpio, !on ^ !(sw->flags & GPIO_SW_ACTIVE_LOW));
}
EXPORT_SYMBOL(__gpio_switch_set);

void gpio_switch_set_suspend_state(struct gpio_sw *sw, int suspend_state)
{
	switch (suspend_state) {
	case 0:
		sw->flags |= GPIO_SW_SUSPEND_OFF;
		break;

	case 1:
		sw->flags |= GPIO_SW_SUSPEND_ON;
		break;

	default:
		sw->flags &= ~(GPIO_SW_SUSPEND_ON | GPIO_SW_SUSPEND_OFF);
	}
}
EXPORT_SYMBOL(gpio_switch_set_suspend_state);

void gpio_switch_set_resume_state(struct gpio_sw *sw, int resume_state)
{
	switch (resume_state) {
	case 0:
		sw->flags |= GPIO_SW_RESUME_OFF;
		break;

	case 1:
		sw->flags |= GPIO_SW_RESUME_ON;
		break;

	default:
		sw->flags &= ~(GPIO_SW_RESUME_ON | GPIO_SW_RESUME_OFF);
	}
}
EXPORT_SYMBOL(gpio_switch_set_resume_state);

/* Driver boilerplate */
static int __devinit gpio_switch_platform_probe(struct platform_device *pdev,
						struct gpio_sw *sw)
{
	struct gpio_sw_platform_data *pdata = pdev->dev.platform_data;

	sw->gpio = pdata->gpio;
	if (!gpio_is_valid(sw->gpio))
		return -EINVAL;

	sw->flags = pdata->flags;
	sw->label = pdata->label;
	if (pdata->init_state) {
		if (pdata->init_state == GPIO_SW_INIT_ACTIVE)
			sw->enable_count++;
		__gpio_switch_set(sw, sw->enable_count);
	}
	return 0;
}

static int __devinit gpio_switch_dt_probe(struct platform_device *pdev,
					  struct gpio_sw *sw)
{
	struct device_node *np = pdev->dev.of_node;
	int gpio;
	enum of_gpio_flags gpio_flags;
	const u32 *prop;

	of_property_read_string(np, "label", &sw->label);

	gpio = of_get_named_gpio_flags(np, "gpio", 0, &gpio_flags);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "No valid GPIO specified for '%s'\n",
			sw->label ?: "unknown");
		return -EINVAL;
	}

	sw->gpio = gpio;
	if (gpio_flags & OF_GPIO_ACTIVE_LOW)
		sw->flags |= GPIO_SW_ACTIVE_LOW;

	if (of_get_property(np, "shared-gpio", NULL))
		sw->flags |= GPIO_SW_SHARED;

	prop = of_get_property(np, "suspend-state", NULL);
	if (prop)
		gpio_switch_set_suspend_state(sw, *prop);

	prop = of_get_property(np, "resume-state", NULL);
	if (prop)
		gpio_switch_set_resume_state(sw, *prop);

	prop = of_get_property(np, "init-state", NULL);
	if (prop) {
		if (*prop)
			sw->enable_count++;
		__gpio_switch_set(sw, sw->enable_count);
	}
	return 0;
}

static int __devinit gpio_switch_probe(struct platform_device *pdev)
{
	int ret;
	struct gpio_sw_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_sw *sw;

	sw = devm_kzalloc(&pdev->dev, sizeof(*sw), GFP_KERNEL);
	if (!sw)
		return -ENOMEM;

	if (pdata) {
		ret = gpio_switch_platform_probe(pdev, sw);
	} else {
		ret = gpio_switch_dt_probe(pdev, sw);
	}
	if (ret)
		return ret;
	INIT_LIST_HEAD(&sw->list);
	sw->parent = &pdev->dev;

	if (!(sw->flags & GPIO_SW_SHARED) ||
	    !gpio_switch_find_gpio(sw->parent, sw->gpio)) {
		ret = gpio_request(sw->gpio, sw->label);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request GPIO%d '%s'\n",
				sw->gpio, sw->label);
			return ret;
		}
		gpio_direction_output(sw->gpio, sw->flags & GPIO_SW_ACTIVE_LOW);
	}
	platform_set_drvdata(pdev, sw);

	mutex_lock(&gpio_switch_list_lock);
	list_add(&sw->list, &gpio_switch_list);
	mutex_unlock(&gpio_switch_list_lock);

	dev_info(&pdev->dev, "GPIO%u registered for '%s'\n",
		sw->gpio, sw->label ?: "unknown");
	return 0;
}

static int __devexit gpio_switch_remove(struct platform_device *pdev)
{
	struct gpio_sw *sw, *tmp;

	list_for_each_entry_safe(sw, tmp, &gpio_switch_list, list) {
		gpio_switch_unregister(sw);
	}
	return 0;
}

static struct of_device_id gpio_switch_dt_ids[] = {
	{ .compatible = "linux,gpio-switch", },
	{ /* sentinel */ }
};

struct platform_driver gpio_switch_driver = {
	.driver = {
		.name = "gpio-switch",
		.owner = THIS_MODULE,
		.of_match_table = gpio_switch_dt_ids,
	},
	.probe = gpio_switch_probe,
	.remove = __devexit_p(gpio_switch_remove),
};

module_platform_driver(gpio_switch_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lother Waﬂmann <LW@KARO-electronics.de>");
MODULE_DESCRIPTION("Generic GPIO switch driver");
MODULE_ALIAS("platform:gpio_switch");
