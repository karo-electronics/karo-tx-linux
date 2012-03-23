/*
 * drivers/input/touchscreen/mxs-lradc-ts.c
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
 */
#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include "../iio.h"
#include "../driver.h"
#include "../consumer.h"
#include "../iio_core_trigger.h"

#define MAX_18BIT		((1 << 18) - 1)

static struct mxs_lradc_ts_plat_data {
	const char *channel_name;
	unsigned int debounce_delay_ms;
} default_mxs_lradc_ts_pdata = {
	.debounce_delay_ms = 10,
};

struct mxs_lradc_ts {
	char phys_dev[32];
	struct iio_dev *iio_dev;
	struct iio_info *info;
	struct iio_channel *channels;
	struct iio_trigger *trigger;
	struct input_dev *input_dev;
	unsigned int debounce_delay_ms;
};

static irqreturn_t mxs_lradc_ts_pendown(int irq, void *data)
{
	printk(KERN_DEBUG "%s: \n", __func__);
	return IRQ_HANDLED;
}

static int mxs_lradc_ts_open(struct input_dev *input_dev)
{
	struct mxs_lradc_ts *ts = input_get_drvdata(input_dev);
	return 0;
}

static void mxs_lradc_ts_close(struct input_dev *input_dev)
{
	struct mxs_lradc_ts *ts = input_get_drvdata(input_dev);
}

static int __devinit mxs_lradc_ts_of_init(struct platform_device *pdev,
	struct mxs_lradc_ts_plat_data *pdata)
{
	*pdata = default_mxs_lradc_ts_pdata;
	return 0;
}

static int __devinit mxs_lradc_ts_probe(struct platform_device *pdev)
{
	int ret;
	struct mxs_lradc_ts_plat_data *pdata = pdev->dev.platform_data;
	struct mxs_lradc_ts *ts;
	int i;

	ts = devm_kzalloc(&pdev->dev, sizeof(struct mxs_lradc_ts), GFP_KERNEL);
	if (ts == NULL)
		return -ENOMEM;

	if (pdata != NULL) {
		if (pdata->debounce_delay_ms)
			ts->debounce_delay_ms = pdata->debounce_delay_ms;
	} else {
		pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct mxs_lradc_ts_plat_data),
				GFP_KERNEL);
		if (pdata == NULL)
			return -ENOMEM;

		ret = mxs_lradc_ts_of_init(pdev, pdata);
		if (ret)
			return ret;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL)
		return -ENOMEM;

	snprintf(ts->phys_dev, sizeof(ts->phys_dev),
		"%s/input0", dev_name(&pdev->dev));

	ts->input_dev->name = "MXS LRADC Touchscreen";
	ts->input_dev->phys = ts->phys_dev;
	ts->input_dev->id.bustype = BUS_IIO;

	ts->input_dev->open = mxs_lradc_ts_open;
	ts->input_dev->close = mxs_lradc_ts_close;

	input_set_drvdata(ts->input_dev, ts);

	ts->input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(ts->input_dev, ABS_X, 0, MAX_18BIT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, MAX_18BIT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, MAX_18BIT,
			0, 0);

	ts->channels = iio_st_channel_get_all(dev_name(&pdev->dev));
	if (IS_ERR(ts->channels)) {
		ret = PTR_ERR(ts->channels);
		dev_err(&pdev->dev, "Failed to get ADC channels: %d\n", ret);
		goto err;
	}

	for (i = 0; ts->channels[i].indio_dev != NULL; i++) {
		u32 val;

		dev_dbg(&pdev->dev, "%s: Registering trigger consumer for chan %d indio_dev %p\n",
			__func__, i, ts->channels[i].indio_dev);
		iio_device_register_trigger_consumer(ts->channels[i].indio_dev);
		iio_st_read_channel_raw(&ts->channels[i], &val);
		dev_dbg(&pdev->dev, "%s: read %u from channel %i\n", __func__,
			val, i);
	}

	platform_set_drvdata(pdev, ts);

	ret = input_register_device(ts->input_dev);

	return 0;

err:
	input_free_device(ts->input_dev);
	return ret;
}

static int __devexit mxs_lradc_ts_remove(struct platform_device *pdev)
{
	struct mxs_lradc_ts *ts = platform_get_drvdata(pdev);

	iio_device_unregister_trigger_consumer(ts->iio_dev);
	iio_st_channel_release_all(ts->channels);
	input_free_device(ts->input_dev);
	return 0;
}

static struct platform_driver mxs_lradc_ts_driver = {
	.driver = {
		.name = "mxs-lradc-ts",
		.owner = THIS_MODULE,
	},
	.probe = mxs_lradc_ts_probe,
	.remove = mxs_lradc_ts_remove,
};

#if 0
module_platform_driver(mxs_lradc_ts_driver);
#else
/*
LRADC 2 - 6 can be used for 4/5-wire touch-screen control. LRADC 6 can be used for the wiper of 5-
wire touch-screen controller and external temperature sensing, but they cannot be enabled at the
same time in hardware configuration.
 LRADC 5 can be used for Y- of 4-wire and LR of 5-wire;
 LRADC 4 can be used for X- of 4-wire and UR of 5-wire;
 LRADC 3 can be used for Y+ of 4-wire and LL of 5-wire;
 LRADC 2 can be used for X+ and UR of 5-wire;
 For pull-up or pull-down switch control on LRADC2~5 pins, please refer to HW_LRADC_CTRL0 register.
*/

static int mxs_lradc_ts_init(void)
{
	int ret;

	ret = platform_driver_register(&mxs_lradc_ts_driver);
	return ret;
}
module_init(mxs_lradc_ts_init);

static void mxs_lradc_ts_exit(void)
{
	platform_driver_unregister(&mxs_lradc_ts_driver);
}
module_exit(mxs_lradc_ts_exit);
#endif

MODULE_AUTHOR("Lothar Waßmann <LW@KARO-electronics.de>");
MODULE_DESCRIPTION("MXS LRADC Touchcontroller driver");
MODULE_LICENSE("GPL v2");
