/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
	unsigned		inverted:1;
};

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int max = bl->props.max_brightness;
	int brightness = pb->inverted ? max - bl->props.brightness :
		bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = pb->inverted ? max : 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = pb->inverted ? max : 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness == 0) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
	} else {
		brightness = pb->lth_brightness +
			(brightness * (pb->period - pb->lth_brightness) / max);
		pwm_config(pb->pwm, brightness, pb->period);
		pwm_enable(pb->pwm);
	}

	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};

#ifdef CONFIG_OF
static int __devinit pwm_backlight_of_probe(struct platform_device *pdev,
			struct platform_pwm_backlight_data *data)
{
	struct device_node *np = pdev->dev.of_node;
	const phandle *ph;
	struct device_node *pwm_node;
	const u32 *prop;

	if (!np) {
		dev_err(&pdev->dev,
			"No OF properties and no platform data supplied\n");
		return -EINVAL;
	}

	if (!of_device_is_available(np)) {
		return -ENODEV;
	}

	ph = of_get_property(np, "pwm", NULL);
	if (ph == NULL) {
		dev_err(&pdev->dev,
			"No 'pwm' handle found in device tree\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "Found PWM handle: %08x\n", be32_to_cpu(*ph));

	pwm_node = of_find_node_by_phandle(be32_to_cpu(*ph));
	if (pwm_node == NULL) {
		dev_err(&pdev->dev,
			"referenced pwm not found in device tree\n");
		return -EINVAL;
	}

	prop = of_get_property(pwm_node, "reg", NULL);
	if (prop) {
		dev_info(&pdev->dev, "pwm-id set to %08x\n",
			be32_to_cpu(*prop));
		data->pwm_id = be32_to_cpu(*prop);
	}
	of_node_put(pwm_node);

	prop = of_get_property(np, "max-brightness", NULL);
	if (prop) {
		dev_info(&pdev->dev, "max-brightness set to %u\n",
			be32_to_cpu(*prop));
		data->max_brightness = be32_to_cpu(*prop);
	}

	prop = of_get_property(np, "dft-brightness", NULL);
	if (prop) {
		dev_info(&pdev->dev, "dft-brightness set to %u\n",
			be32_to_cpu(*prop));
		data->dft_brightness = be32_to_cpu(*prop);
	}

	prop = of_get_property(np, "pwm-period-ns", NULL);
	if (prop) {
		dev_info(&pdev->dev, "pwm-period-ns set to %u\n",
			be32_to_cpu(*prop));
		data->pwm_period_ns = be32_to_cpu(*prop);
	}
	if (of_get_property(np, "inverted", NULL))
		data->inverted = 1;
	return 0;
}
#else
static inline int pwm_backlight_of_probe(struct platform_device *pdev,
			struct platform_pwm_backlight_data *data)
{
	return -ENODEV;
}
#endif

static int __devinit pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	if (!data) {
		data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		ret = pwm_backlight_of_probe(pdev, data);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"No platform data supplied\n");
			return ret;
		}
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb->period = data->pwm_period_ns;
	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
	pb->lth_brightness = data->lth_brightness *
		(data->pwm_period_ns / data->max_brightness);
	pb->inverted = data->inverted;
	pb->dev = &pdev->dev;

	pb->pwm = pwm_request(data->pwm_id, "backlight");
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for backlight\n");
		ret = PTR_ERR(pb->pwm);
		goto err_pwm;
	} else
		dev_dbg(&pdev->dev, "got pwm for backlight\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	pwm_free(pb->pwm);
err_pwm:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int __devexit pwm_backlight_remove(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	dev_info(&pdev->dev, "Unloading driver\n");

	if (!pb->inverted) {
		dev_info(&pdev->dev, "Configuring PWM to 0%% duty cycle\n");
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
	} else {
		dev_info(&pdev->dev, "Configuring PWM to %u%% duty cycle\n",
			bl->props.max_brightness);
		pwm_config(pb->pwm, pb->period, pb->period);
		pwm_enable(pb->pwm);
	}
	backlight_device_unregister(bl);
	pwm_free(pb->pwm);

	if (data && data->exit)
		data->exit(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM
static int pwm_backlight_suspend(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	if (pb->notify)
		pb->notify(pb->dev, 0);

	if (!pb->inverted) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
	} else {
		pwm_config(pb->pwm, pb->period, pb->period);
		pwm_enable(pb->pwm);
	}

	if (pb->notify_after)
		pb->notify_after(pb->dev, 0);
	return 0;
}

static int pwm_backlight_resume(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);

	backlight_update_status(bl);
	return 0;
}

static struct dev_pm_ops pwm_backlight_pm_ops = {
	.suspend	= pwm_backlight_suspend,
	.resume		= pwm_backlight_resume,
};

#define __dev_pm_ops_p(p)	&(p)
#else
#define pwm_backlight_suspend	NULL
#define pwm_backlight_resume	NULL
#define __dev_pm_ops_p(p)	NULL
#endif

static struct of_device_id pwm_bl_dt_ids[] = {
	{ .compatible = "pwm-backlight", },
	{ /* sentinel */ }
};

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name	= "pwm-backlight",
		.owner	= THIS_MODULE,
		.pm = __dev_pm_ops_p(pwm_backlight_pm_ops),
		.of_match_table = pwm_bl_dt_ids,
	},
	.probe		= pwm_backlight_probe,
	.remove		= __devexit_p(pwm_backlight_remove),
};

module_platform_driver(pwm_backlight_driver);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

