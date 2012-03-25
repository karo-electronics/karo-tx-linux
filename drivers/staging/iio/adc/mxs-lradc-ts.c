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

#ifdef DEBUG
static int debug = 1;
module_param(debug, int, S_IRUGO | S_IWUSR);
#define dbg_lvl(n)	(debug > (n))
#else
static int debug;
module_param(debug, int, 0);
#endif

#define DBG(n, fmt...) do { if (dbg_lvl(n)) printk(KERN_DEBUG fmt); } while (0)
#define DEV_DBG(n, dev, fmt...) do { if (dbg_lvl(n)) printk(KERN_DEBUG fmt); } while (0)

#include "../iio.h"
#include "../driver.h"
#include "../consumer.h"
#include "../trigger.h"
#include "../trigger_consumer.h"
#include "../iio_core_trigger.h"

#define MAX_12BIT		((1 << 12) - 1)

static struct mxs_lradc_ts_plat_data {
	const char *channel_name;
	unsigned int debounce_delay_ms;
} default_mxs_lradc_ts_pdata = {
	.debounce_delay_ms = 0,
};

enum MXS_LRADC_TS_STATE {
	STATE_DETECT,
	STATE_PENDOWN,
	STATE_MEASURE_X,
	STATE_MEASURE_Y,
	STATE_MEASURE_P,
};

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
	unsigned int debounce_delay;

	enum MXS_LRADC_TS_STATE state;
	unsigned pendown:1;
	uint16_t x;
	uint16_t y;
	uint16_t p;
};

static irqreturn_t mxs_lradc_ts_pendown(int irq, void *p)
{
	struct iio_poll_func *pf = p;

	pf->timestamp = iio_get_time_ns();

	DEV_DBG(1, &pf->indio_dev->dev, "%s: iio_dev %p trigger %p\n", __func__,
		pf->indio_dev, pf->indio_dev->trig);
	return IRQ_WAKE_THREAD;
}

static int mxs_lradc_ts_read_value(struct mxs_lradc_ts *ts, int chan_id)
{
	int ret;
	struct iio_dev *iio_dev = ts->channels[chan_id]->indio_dev;
	const struct iio_chan_spec *chan = ts->channels[chan_id]->channel;
	const struct iio_info *info = iio_dev->info;
	int val = -1;

	info->write_raw(iio_dev, chan, 0, 0, IIO_CHAN_INFO_OVERSAMPLE_COUNT);
	ret = info->read_raw(iio_dev, chan, &val, NULL, 0);
	DEV_DBG(1, &iio_dev->dev, "%s: read_raw(%d) returned %d (val=%08x)\n",
		__func__, chan_id, ret, val);

	if (ret == IIO_VAL_INT)
		return val;

	ts->state = STATE_DETECT;
	return ret;
}

static struct mxs_lradc_ts *_ts;

#include <linux/sched.h>

static irqreturn_t mxs_lradc_ts_state_machine(int irq, void *p)
{
	int ret;
	struct iio_poll_func *pf = p;
	struct mxs_lradc_ts *ts = _ts;
	const struct iio_chan_spec *chan = ts->detect_chan;

	do {
		enum MXS_LRADC_TS_STATE old_state = ts->state;

	switch (ts->state) {
	case STATE_DETECT:
		DEV_DBG(1, &pf->indio_dev->dev, "%s: state %d\n", __func__, ts->state);
		ts->state = STATE_PENDOWN;
		break;

	case STATE_PENDOWN:
		DEV_DBG(1, &pf->indio_dev->dev, "%s: state %d\n", __func__, ts->state);

		if (ts->debounce_delay)
			schedule_timeout(ts->debounce_delay);
		ret = ts->info->read_event_config(ts->trig_dev, chan->address);
		if (ret == 0) {
			ts->state = STATE_DETECT;
			DEV_DBG(0, &pf->indio_dev->dev, "%s: new state %d\n", __func__, ts->state);
			break;
		}

		ts->state = STATE_MEASURE_X;
		break;

	case STATE_MEASURE_X:
		DEV_DBG(1, &pf->indio_dev->dev, "%s: state %d\n", __func__, ts->state);

		ret = mxs_lradc_ts_read_value(ts, TS_CHAN_XP);
		DBG(1, "%s: read %04x from XP\n", __func__, ret);
		if (ret > 4000)
			_DBG(1, "%s: read %04x from XP\n", __func__, ret);
		if (ret < 0)
			break;

		/* For some reason the X and Y results are swapped! */
		ts->y = ret;

		ts->state = STATE_MEASURE_Y;
		break;

	case STATE_MEASURE_Y:
		DEV_DBG(1, &pf->indio_dev->dev, "%s: state %d\n", __func__, ts->state);

		ret = mxs_lradc_ts_read_value(ts, TS_CHAN_YP);
		DBG(1, "%s: read %04x from YP\n", __func__, ret);
		if (ret > 4000)
			_DBG(1, "%s: read %04x from XP\n", __func__, ret);
		if (ret < 0)
			break;

		/* For some reason the X and Y results are swapped! */
		ts->x = ret;

		ts->state = STATE_MEASURE_P;
		break;

	case STATE_MEASURE_P:
		DEV_DBG(1, &pf->indio_dev->dev, "%s: state %d\n", __func__, ts->state);

		ts->pendown = 1;
		DEV_DBG(0, &ts->input_dev->dev, "Reporting PEN DOWN %u %u\n",
			ts->x, ts->y);
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_report_abs(ts->input_dev, ABS_X, ts->x);
		input_report_abs(ts->input_dev, ABS_Y, ts->y);
		input_report_abs(ts->input_dev, ABS_PRESSURE, 1);
		input_sync(ts->input_dev);
		ts->state = STATE_PENDOWN;
		break;
	}
	DEV_DBG(1, &pf->indio_dev->dev, "%s: state %d -> %d\n", __func__,
		old_state, ts->state);

	ts->info->write_event_config(ts->trig_dev, chan->address, ts->state);
	} while (ts->state != STATE_DETECT);

	if (ts->pendown) {
		DEV_DBG(0, &ts->input_dev->dev, "Reporting PEN UP\n");
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
		input_sync(ts->input_dev);
		ts->pendown = 0;
	}
	DEV_DBG(1, &pf->indio_dev->dev, "%s: Notifying trigger %p\n", __func__,
		ts->trigger);
	iio_trigger_notify_done(ts->trigger);
	return IRQ_HANDLED;
}

static int mxs_lradc_ts_open(struct input_dev *input_dev)
{
	int ret;
	struct mxs_lradc_ts *ts = input_get_drvdata(input_dev);

	dev_dbg(&input_dev->dev, "%s: Getting module\n", __func__);
	if (!try_module_get(ts->trig_dev->info->driver_module))
		return -EAGAIN;

	ts->state = STATE_DETECT;

	dev_dbg(&input_dev->dev, "%s: Allocating pollfunc to %p trigger %p '%s'\n",
		__func__, ts->trig_dev, ts->trig_dev->trig,
		ts->trig_dev->trig ? ts->trig_dev->trig->name : NULL);
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

	dev_dbg(&input_dev->dev, "%s: Attaching pollfunc\n", __func__);
	ret = iio_triggered_buffer_postenable(ts->trig_dev);
	if (ret)
		goto err_dealloc_pollfunc;
	dev_dbg(&input_dev->dev, "%s: done\n", __func__);
	return 0;

err_dealloc_pollfunc:
	dev_dbg(&input_dev->dev, "%s: Deallocating pollfunc\n", __func__);
	iio_dealloc_pollfunc(ts->trig_dev->pollfunc);

err:
	dev_dbg(&input_dev->dev, "%s: Releasing module\n", __func__);
	module_put(ts->trig_dev->info->driver_module);
	dev_dbg(&input_dev->dev, "%s: done\n", __func__);
	return 0;
}

static void mxs_lradc_ts_close(struct input_dev *input_dev)
{
	struct mxs_lradc_ts *ts = input_get_drvdata(input_dev);

	dev_dbg(&input_dev->dev, "%s: Detaching pollfunc\n", __func__);
	iio_triggered_buffer_predisable(ts->trig_dev);

	dev_dbg(&input_dev->dev, "%s: Deallocating pollfunc\n", __func__);
	iio_dealloc_pollfunc(ts->trig_dev->pollfunc);

	dev_dbg(&input_dev->dev, "%s: Releasing module\n", __func__);
	module_put(ts->trig_dev->info->driver_module);
	dev_dbg(&input_dev->dev, "%s: done\n", __func__);
}

static int __devinit mxs_lradc_ts_of_init(struct platform_device *pdev,
	struct mxs_lradc_ts_plat_data *pdata)
{
	*pdata = default_mxs_lradc_ts_pdata;
	return 0;
}

static const char *mxs_lradc_channel_names[TS_NUM_CHANNELS] __devinitdata = {
	[TS_CHAN_DETECT] = "pendetect",
	[TS_CHAN_XP] = "TS X+",
	[TS_CHAN_YP] = "TS Y+",
	[TS_CHAN_XM] = "TS X-",
	[TS_CHAN_YM] = "TS Y-",
	[TS_CHAN_WIPER] = "TS WIPER",
};

static void __devinit mxs_lradc_ts_put_channels(struct mxs_lradc_ts *ts)
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
			goto err;
		}
		dev_dbg(&pdev->dev, "%s: Got iio channel[%d] %p '%s'\n",
			__func__, i, ts->channels[i],
			ts->channels[i]->channel->extend_name);
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
	struct mxs_lradc_ts_plat_data *pdata = pdev->dev.platform_data;
	struct mxs_lradc_ts *ts;

	ts = devm_kzalloc(&pdev->dev, sizeof(struct mxs_lradc_ts), GFP_KERNEL);
	if (ts == NULL)
		return -ENOMEM;

	if (pdata == NULL) {
		pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct mxs_lradc_ts_plat_data),
				GFP_KERNEL);
		if (pdata == NULL)
			return -ENOMEM;

		ret = mxs_lradc_ts_of_init(pdev, pdata);
		if (ret)
			return ret;
	}
	if (pdata->debounce_delay_ms)
		ts->debounce_delay =
			msecs_to_jiffies(pdata->debounce_delay_ms);

	ret = mxs_lradc_ts_get_channels(pdev, ts);
	if (ret)
		return ret;

	ts->detect_chan = ts->channels[TS_CHAN_DETECT]->channel;
	ts->trig_dev = ts->channels[TS_CHAN_DETECT]->indio_dev;
	ts->info = ts->trig_dev->info;
	ts->trigger = ts->trig_dev->trig;
	dev_dbg(&pdev->dev, "%s: iio_dev=%p info=%p trigger=%p\n", __func__,
		ts->trig_dev, ts->info, ts->trigger);

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
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);

	platform_set_drvdata(pdev, ts);

	ret = input_register_device(ts->input_dev);
	if (ret)
		goto err_free_dev;
	dev_dbg(&ts->input_dev->dev, "%s driver registered\n",
		ts->input_dev->name);
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

	dev_dbg(&ts->input_dev->dev, "%s: Releasing iio channels %p\n",
		__func__, ts->channels);

	mxs_lradc_ts_put_channels(ts);

	dev_dbg(&ts->input_dev->dev, "%s: Unregistering input device %p\n",
		__func__, ts->input_dev);
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
