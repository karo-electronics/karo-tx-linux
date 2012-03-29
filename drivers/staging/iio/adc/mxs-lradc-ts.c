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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include "../iio.h"
#include "../driver.h"
#include "../consumer.h"
#include "../trigger.h"
#include "../trigger_consumer.h"
#include "../iio_core_trigger.h"

#include "mxs-lradc.h"

#define MAX_12BIT			((1 << 12) - 1)
#define MAX_16BIT			((1 << 16) - 1)
#define TOUCH_DEBOUNCE_TOLERANCE	100
#define TRIGGER_DELAY			8

static unsigned int debounce_delay_us = 100;
module_param(debounce_delay_us, int, S_IRUGO | S_IWUSR);

static unsigned int oversample_count = 1;
module_param(oversample_count, int, S_IRUGO | S_IWUSR);

static unsigned int precharge_delay_us = 10;
module_param(precharge_delay_us, int, S_IRUGO | S_IWUSR);

enum {
	TS_CHAN_DETECT,
	TS_CHAN_XP,
	TS_CHAN_YP,
	TS_CHAN_XM,
	TS_CHAN_YM,
	TS_CHAN_WIPER,
	TS_NUM_CHANNELS,
};

struct mxs_lradc_ts {
	char phys_dev[32];
	struct iio_channel *channels[TS_NUM_CHANNELS];

	const struct iio_chan_spec *detect_chan;
	struct iio_dev *trig_dev;
	struct iio_trigger *trigger;
	const struct iio_info *info;

	struct input_dev *input_dev;

	enum MXS_LRADC_TS_STATE state;
	uint16_t x;
	uint16_t y;
	uint16_t p;
};

static irqreturn_t mxs_lradc_ts_pendown(int irq, void *p)
{
	struct iio_poll_func *pf = p;

	pf->timestamp = iio_get_time_ns();
	return IRQ_WAKE_THREAD;
}

static int mxs_lradc_ts_read_value(struct mxs_lradc_ts *ts, int chan_id)
{
	int ret;
	struct iio_dev *iio_dev = ts->channels[chan_id]->indio_dev;
	const struct iio_chan_spec *chan = ts->channels[chan_id]->channel;
	const struct iio_info *info = iio_dev->info;
	int val, val2;

	if (oversample_count > 32)
		oversample_count = 32;
	else if (oversample_count == 0)
		oversample_count = 1;

	info->write_raw(iio_dev, chan, oversample_count, 0, IIO_CHAN_INFO_OVERSAMPLE_COUNT);

	info->write_raw(iio_dev, chan, TRIGGER_DELAY, 0, IIO_CHAN_INFO_TRIGGER_DELAY);
	ret = info->read_raw(iio_dev, chan, &val, NULL, 0);
	if (ret != IIO_VAL_INT)
		goto err;
#if 1
	return val;
#else
	info->write_raw(iio_dev, chan, TRIGGER_DELAY, 0, IIO_CHAN_INFO_TRIGGER_DELAY);
	ret = info->read_raw(iio_dev, chan, &val2, NULL, 0);
	if (ret != IIO_VAL_INT)
		goto err;

	if (abs(val - val2) < TOUCH_DEBOUNCE_TOLERANCE)
		return (val + val2) / 2;

	return -EINVAL;
#endif
err:
	dev_err(&ts->input_dev->dev, "Error reading data from chan %d\n", chan_id);
	ts->state = STATE_DETECT;
	return ret;
}

static struct mxs_lradc_ts *_ts;

static irqreturn_t mxs_lradc_ts_state_machine(int irq, void *p)
{
	int ret;
	struct mxs_lradc_ts *ts = _ts;
	const struct iio_chan_spec *chan = ts->detect_chan;
	unsigned int yn, xp;
	int pd = 1;

	do {
		ts->info->write_event_config(ts->trig_dev,
					chan->address, ts->state);

		switch (ts->state) {
		case STATE_DETECT:
			ts->state = STATE_PENDOWN;
			pd = 1;
			break;

		case STATE_PENDOWN:
			if (debounce_delay_us) {
				if (debounce_delay_us > 1000)
					msleep(debounce_delay_us / 1000);
				udelay(debounce_delay_us % 1000);
			}
			ret = ts->info->read_event_config(ts->trig_dev,
								chan->address);
			if (ret == pd)
				ts->state = ret ? STATE_IDLE : STATE_DETECT;
			pd = ret;
			break;

		case STATE_IDLE:
			udelay(precharge_delay_us);
			ts->state = STATE_MEASURE_Y;
			break;

		case STATE_MEASURE_Y:
			ret = mxs_lradc_ts_read_value(ts, TS_CHAN_XP);
			if (ret < 0)
				break;

			ts->y = ret;
			ts->state = STATE_MEASURE_X;
			break;

		case STATE_MEASURE_X:
			ret = mxs_lradc_ts_read_value(ts, TS_CHAN_YP);
			if (ret < 0)
				break;

			ts->x = ret;
			ts->state = STATE_MEASURE_P;
			break;

		case STATE_MEASURE_P:
			ret = mxs_lradc_ts_read_value(ts, TS_CHAN_XP);
			if (ret < 0)
				break;
			xp = ret;

			ret = mxs_lradc_ts_read_value(ts, TS_CHAN_YM);
			if (ret < 0)
				break;
			yn = ret;

			/*
			 * Calculate pressure from touch contact resistance:
			 * R = r_xplate * x / 4096 * (yn - xp) / xp
			 *
			 * pressure is inverse proportional to R:
			 * P = xp * 4096 / r_xplate / x / (yn - xp)
			 *
			 * scaled to max possible range avoiding overflow
			 * P = xp * 4096 * 256 / x / (yn - xp)
			 */
			if (xp > 0 && yn > xp && ts->x > 0) {
				int p = xp * 4096 * 256 / ts->x / (yn - xp);

				if (p == 0) {
					/* discard bogus measurement */
					ts->state = STATE_PENDOWN;
					break;
				} else if (p > MAX_16BIT) {
					ts->p = MAX_16BIT;
				} else {
					ts->p = p;
				}
			} else {
				ts->state = STATE_PENDOWN;
				break;
			}

			ts->state = STATE_DONE;
			break;

		case STATE_DONE:
			WARN_ON(!ts->p);
			input_report_key(ts->input_dev, BTN_TOUCH, !!ts->p);
			input_report_abs(ts->input_dev, ABS_X, ts->x);
			input_report_abs(ts->input_dev, ABS_Y, ts->y);
			input_report_abs(ts->input_dev, ABS_PRESSURE, ts->p);
			input_sync(ts->input_dev);
			ts->state = STATE_PENDOWN;
			break;

		default:
			WARN_ONCE(1, "MXS LRADC-TS: Unhandled state %d\n",
				ts->state);
			ts->state = STATE_DETECT;
		}
	} while (ts->state != STATE_DETECT);

	ts->info->write_event_config(ts->trig_dev, chan->address, ts->state);

	if (ts->p) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
		input_sync(ts->input_dev);
		ts->p = 0;
	}
	iio_trigger_notify_done(ts->trigger);
	return IRQ_HANDLED;
}

static int mxs_lradc_ts_open(struct input_dev *input_dev)
{
	int ret;
	struct mxs_lradc_ts *ts = input_get_drvdata(input_dev);

	if (!try_module_get(ts->trig_dev->info->driver_module))
		return -EAGAIN;

	ts->state = STATE_DETECT;

	ts->trig_dev->pollfunc = iio_alloc_pollfunc(mxs_lradc_ts_pendown,
				mxs_lradc_ts_state_machine, 0,
				ts->trig_dev, ts->trigger->name);
	if (ts->trig_dev->pollfunc == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	/* HACK ALERT!
	 * Need to find a way to pass the proper context
	 * to the pollfuncs
	 */
	_ts = ts;

	ret = iio_triggered_buffer_postenable(ts->trig_dev);
	if (ret)
		goto err_dealloc_pollfunc;

	return 0;

err_dealloc_pollfunc:
	iio_dealloc_pollfunc(ts->trig_dev->pollfunc);

err:
	module_put(ts->trig_dev->info->driver_module);
	return 0;
}

static void mxs_lradc_ts_close(struct input_dev *input_dev)
{
	struct mxs_lradc_ts *ts = input_get_drvdata(input_dev);

	ts->state = STATE_DISABLE;
	ts->info->write_event_config(ts->trig_dev,
				ts->detect_chan->address, ts->state);

	iio_triggered_buffer_predisable(ts->trig_dev);

	iio_dealloc_pollfunc(ts->trig_dev->pollfunc);

	module_put(ts->trig_dev->info->driver_module);
}

static const char *mxs_lradc_channel_names[TS_NUM_CHANNELS] __devinitdata = {
	[TS_CHAN_DETECT] = "pendetect",
	[TS_CHAN_XP] = "TS X+",
	[TS_CHAN_YP] = "TS Y+",
	[TS_CHAN_XM] = "TS X-",
	[TS_CHAN_YM] = "TS Y-",
	[TS_CHAN_WIPER] = "TS WIPER",
};

static inline void mxs_lradc_ts_put_channels(struct mxs_lradc_ts *ts)
{
	int i;

	for (i = 0; i < TS_NUM_CHANNELS; i++) {
		iio_st_channel_release(ts->channels[i]);
		ts->channels[i] = NULL;
	}
}

static int __devinit mxs_lradc_ts_get_channels(struct platform_device *pdev,
					struct mxs_lradc_ts *ts)
{
	int ret;
	int i;

	for (i = 0; i < TS_NUM_CHANNELS; i++) {
		ts->channels[i] = iio_st_channel_get(dev_name(&pdev->dev),
						mxs_lradc_channel_names[i]);
		if (IS_ERR_OR_NULL(ts->channels[i])) {
			if (IS_ERR(ts->channels[i]))
				ret = PTR_ERR(ts->channels[i]);
			else
				ret = -ENODEV;
			dev_err(&pdev->dev, "Failed to get ADC channels: %d\n",
				ret);
			goto err;
		}
	}
	return 0;
err:
	while (--i >= 0) {
		iio_st_channel_release(ts->channels[i]);
		ts->channels[i] = NULL;
	}
	return ret;
}

static int __devinit mxs_lradc_ts_probe(struct platform_device *pdev)
{
	int ret;
	struct mxs_lradc_ts *ts;

	ts = devm_kzalloc(&pdev->dev, sizeof(struct mxs_lradc_ts), GFP_KERNEL);
	if (ts == NULL)
		return -ENOMEM;

	ret = mxs_lradc_ts_get_channels(pdev, ts);
	if (ret == -ENODEV)
		/* if deferred driver init is available this could be retried
		 * after the mxs_lradc driver is loaded
		 */
		return -EAGAIN;
	if (ret)
		return ret;

	ts->detect_chan = ts->channels[TS_CHAN_DETECT]->channel;
	ts->trig_dev = ts->channels[TS_CHAN_DETECT]->indio_dev;
	ts->info = ts->trig_dev->info;
	ts->trigger = ts->trig_dev->trig;

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		goto err_put_channels;
	}

	snprintf(ts->phys_dev, sizeof(ts->phys_dev),
		"%s/input0", dev_name(&pdev->dev));

	ts->input_dev->name = "MXS LRADC Touchscreen";
	ts->input_dev->phys = ts->phys_dev;
	ts->input_dev->id.bustype = BUS_IIO;

	ts->input_dev->open = mxs_lradc_ts_open;
	ts->input_dev->close = mxs_lradc_ts_close;

	input_set_drvdata(ts->input_dev, ts);

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(ABS_X, ts->input_dev->absbit);
	__set_bit(ABS_Y, ts->input_dev->absbit);
	__set_bit(ABS_PRESSURE, ts->input_dev->absbit);

	__set_bit(EV_KEY, ts->input_dev->evbit);
	__set_bit(BTN_TOUCH, ts->input_dev->keybit);

	input_set_abs_params(ts->input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, MAX_16BIT, 0, 0);

	platform_set_drvdata(pdev, ts);

	ret = input_register_device(ts->input_dev);
	if (ret)
		goto err_free_dev;

	return 0;

err_free_dev:
	input_free_device(ts->input_dev);

err_put_channels:
	mxs_lradc_ts_put_channels(ts);
	return ret;
}

static int __devexit mxs_lradc_ts_remove(struct platform_device *pdev)
{
	struct mxs_lradc_ts *ts = platform_get_drvdata(pdev);

	mxs_lradc_ts_put_channels(ts);

	input_unregister_device(ts->input_dev);
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

#if 1
module_platform_driver(mxs_lradc_ts_driver);
#else
static int __init mxs_lradc_ts_init(void)
{
	int ret;

	ret = platform_driver_probe(&mxs_lradc_ts_driver, mxs_lradc_ts_probe);
	if (ret)
		pr_err("Failed to install MXS LRADC Touchscreen driver: %d\n",
			ret);
	else
		pr_info("MXS LRADC Touchscreen driver registered\n");
	return ret;
}
module_init(mxs_lradc_ts_init);

static void __exit mxs_lradc_ts_exit(void)
{
	platform_driver_unregister(&mxs_lradc_ts_driver);
}
module_exit(mxs_lradc_ts_exit);
#endif

MODULE_AUTHOR("Lothar Waßmann <LW@KARO-electronics.de>");
MODULE_DESCRIPTION("MXS LRADC Touchcontroller driver");
MODULE_LICENSE("GPL v2");
