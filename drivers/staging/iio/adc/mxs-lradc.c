/*
 * ADT7410 digital temperature sensor driver supporting ADT7410
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */
#undef DEBUG

#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include <mach/mxs.h>
#include <mach/common.h>

#ifdef DEBUG
static int debug = 1;
module_param(debug, int, S_IRUGO | S_IWUSR);
#else
static int debug;
module_param(debug, int, 0);
#endif

#include "../iio.h"
#include "../sysfs.h"
#include "../machine.h"
#include "../driver.h"
#ifdef CONFIG_IIO_TRIGGER
#include "../trigger.h"
#include "../trigger_consumer.h"
#endif
#include "../kfifo_buf.h"
#include "../ring_sw.h"

/* number of ADCs */
#define NUM_ADC_CHANNELS	8
/* number of analog inputs (muxed to one of the above ADCs */
#define NUM_HW_CHANNELS		16
#define NUM_DELAY_CHANNELS	4
#define NUM_TRIGGERS		1

#define	MAPPING_USED		(1 << 7)
#define	MAPPING_XXX		(1 << 6)
#define	MAPPING_SET_SLOT(m)	(((m) & 0x3) << 4)
#define	MAPPING_GET_SLOT(m)	(((m) >> 4) & 0x3)
#define	MAPPING_CHAN(m)		((m) & 0xf)

struct mxs_lradc_mapped_channel {
	wait_queue_head_t		wq;
	bool				wq_done;
	uint16_t			chan_data;
	uint16_t			mapping;
	uint32_t			users;
	bool				buffered;
};

struct mxs_lradc_trigger {
	size_t				id;
	int				irq;
	struct mxs_lradc_drv_data	*drv_data;
};

struct mxs_lradc_drv_data {
	struct iio_dev			*iio[NUM_DELAY_CHANNELS];
	struct iio_dev			*trig[NUM_TRIGGERS];

	void __iomem			*mmio_base;

	spinlock_t			lock;

	uint16_t			claimed;

	struct mxs_lradc_mapped_channel	ch[NUM_ADC_CHANNELS];
	uint8_t				ch_oversample[NUM_HW_CHANNELS];

	uint32_t			status_mask;
};

struct mxs_lradc_data {
	struct mxs_lradc_drv_data	*drv_data;
	int				id;
	spinlock_t			lock;

	uint16_t			claimed;

	uint32_t			loop_interval;
};


#define	LRADC_CH(n)				(0x50 + (0x10 * (n)))
#define	LRADC_CH_ACCUMULATE			(1 << 29)
#define	LRADC_CH_NUM_SAMPLES_MASK		(0x1f << 24)
#define	LRADC_CH_NUM_SAMPLES_OFFSET		24
#define	LRADC_CH_VALUE_MASK			0x3ffff
#define	LRADC_CH_VALUE_OFFSET			0

#define	LRADC_DELAY(n)				(0xd0 + (0x10 * (n)))
#define	LRADC_DELAY_TRIGGER_LRADCS_MASK		(0xff << 24)
#define	LRADC_DELAY_TRIGGER_LRADCS_OFFSET	24
#define	LRADC_DELAY_KICK			(1 << 20)
#define	LRADC_DELAY_TRIGGER_DELAYS_MASK		(0xf << 16)
#define	LRADC_DELAY_TRIGGER_DELAYS_OFFSET	16
#define	LRADC_DELAY_LOOP_COUNT_MASK		(0x1f << 11)
#define	LRADC_DELAY_LOOP_COUNT_OFFSET		11
#define	LRADC_DELAY_DELAY_MASK			0x7ff
#define	LRADC_DELAY_DELAY_OFFSET		0

#define	LRADC_CTRL0				0x00
#define LRADC_CTRL0_ONCHIP_GROUNDREF		(1 << 26)
#define LRADC_CTRL0_BUTTON1_DETECT_ENABLE	(1 << 25)
#define LRADC_CTRL0_BUTTON0_DETECT_ENABLE	(1 << 24)
#define LRADC_CTRL0_TOUCH_DETECT_ENABLE		(1 << 23)
#define LRADC_CTRL0_TOUCH_SCREEN_TYPE		(1 << 22)
#define LRADC_CTRL0_TOUCH_SCREEN_TYPE_4WIRE	(0 << 22)
#define LRADC_CTRL0_TOUCH_SCREEN_TYPE_5WIRE	(1 << 22)
#define LRADC_CTRL0_YNLRSW			(1 << 21)
#define LRADC_CTRL0_YPLLSW_MASK			(3 << 19)
#define LRADC_CTRL0_YPLLSW_LOW			(2 << 19)
#define LRADC_CTRL0_YPLLSW_HIGH			(1 << 19)
#define LRADC_CTRL0_YPLLSW_OFF			(0 << 19)
#define LRADC_CTRL0_XNURSW_MASK			(3 << 17)
#define LRADC_CTRL0_XNURSW_LOW			(2 << 17)
#define LRADC_CTRL0_XNURSW_HIGH			(1 << 17)
#define LRADC_CTRL0_XNURSW_OFF			(0 << 17)
#define LRADC_CTRL0_XPULSW			(1 << 16)

#define	LRADC_CTRL1				0x10
#define	LRADC_CTRL1_LRADC_BUTTON1_IRQ_EN	(1 << 28)
#define	LRADC_CTRL1_LRADC_BUTTON0_IRQ_EN	(1 << 27)
#define	LRADC_CTRL1_LRADC_THRESH1_IRQ_EN	(1 << 26)
#define	LRADC_CTRL1_LRADC_THRESH0_IRQ_EN	(1 << 25)
#define	LRADC_CTRL1_LRADC_TOUCH_IRQ_EN		(1 << 24)
#define	LRADC_CTRL1_LRADC_BUTTON1_IRQ		(1 << 12)
#define	LRADC_CTRL1_LRADC_BUTTON0_IRQ		(1 << 11)
#define	LRADC_CTRL1_LRADC_THRESH1_IRQ		(1 << 10)
#define	LRADC_CTRL1_LRADC_THRESH0_IRQ		(1 << 9)
#define	LRADC_CTRL1_LRADC_TOUCH_IRQ		(1 << 8)
#define	LRADC_CTRL1_LRADC_IRQ(n)		(1 << (n))
#define	LRADC_CTRL1_LRADC_IRQ_EN(n)		(1 << ((n) + 16))

#define	LRADC_CTRL2				0x20
#define	LRADC_CTRL2_TEMPSENSE_PWD		(1 << 15)

#define	LRADC_CTRL3				0x30

#define	LRADC_CTRL4				0x140
#define	LRADC_CTRL4_LRADCSELECT_MASK(n)		(0xf << ((n) * 4))
#define	LRADC_CTRL4_LRADCSELECT_OFFSET(n)	((n) * 4)

#define	LRADC_STATUS				0x40
#define	LRADC_STATUS_TOUCH_DETECT		(1 << 0)
#define	LRADC_STATUS_BUTTON0_DETECT		(1 << 1)
#define	LRADC_STATUS_BUTTON1_DETECT		(1 << 2)

/*
 * Global IIO attributes
 */
static ssize_t mxs_lradc_show_sampling_rate(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct mxs_lradc_data *data = iio_priv(iio_dev);

	return sprintf(buf, "%u\n", data->loop_interval);
}

static ssize_t mxs_lradc_store_sampling_rate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct mxs_lradc_data *data = iio_priv(iio_dev);
	struct mxs_lradc_drv_data *drv_data = data->drv_data;
	uint32_t reg;
	unsigned long lval;
	unsigned long lflags;

	if (strict_strtoul(buf, 10, &lval))
		return -EINVAL;

	/* Check if the value is in requested range */
	if (lval >= 2048)
		return -EINVAL;

	/*
	 * Noone is accessing our delay trigger in the LRADC reg space so we
	 * don't need to claim top-level spinlock.
	 */
	spin_lock_irqsave(&data->lock, lflags);

	if (data->loop_interval != lval) {
		data->loop_interval = lval;

		/* Update the delay channel */
		reg = readl(drv_data->mmio_base + LRADC_DELAY(data->id));
		reg &= ~LRADC_DELAY_DELAY_MASK;
		reg |= data->loop_interval;
		writel(reg, drv_data->mmio_base + LRADC_DELAY(data->id));
	}

	spin_unlock_irqrestore(&data->lock, lflags);

	return count;
}

static IIO_DEVICE_ATTR(sampling_rate, S_IRUGO | S_IWUSR,
		       mxs_lradc_show_sampling_rate,
		       mxs_lradc_store_sampling_rate, 0);
static IIO_CONST_ATTR(sampling_rate_available,
			"0...2047 (in 1/2000 second steps)");

static struct attribute *mxs_lradc_attributes[] = {
	&iio_dev_attr_sampling_rate.dev_attr.attr,
	&iio_const_attr_sampling_rate_available.dev_attr.attr,
	NULL,
};

static struct attribute_group mxs_lradc_attr_group = {
	.name = "mxs-lradc",
	.attrs = mxs_lradc_attributes,
};

static int mxs_lradc_can_claim_channel(struct iio_dev *iio_dev,
			const struct iio_chan_spec *chan)
{
	struct mxs_lradc_data *data = iio_priv(iio_dev);
	struct mxs_lradc_drv_data *drv_data = data->drv_data;
	int i, count = 0;

	/* The channel is already claimed by us. */
	if (data->claimed & (1 << chan->address))
		return 0;

	/* Check if someone else didn't claim this */
	if (drv_data->claimed & (1 << chan->address))
		return -EBUSY;

	for (i = 0; i < NUM_HW_CHANNELS; i++)
		if (drv_data->claimed & (1 << i))
			count++;

	/* Too many channels claimed */
	if (count >= NUM_ADC_CHANNELS)
		return -EAGAIN;

	return 0;
}

static int mxs_lradc_claim_channel(struct iio_dev *iio_dev,
			const struct iio_chan_spec *chan)
{
	struct mxs_lradc_data *data = iio_priv(iio_dev);
	struct mxs_lradc_drv_data *drv_data = data->drv_data;
	int i, ret;
	unsigned long gflags;
	uint32_t overspl = 0;

	spin_lock_irqsave(&drv_data->lock, gflags);

	ret = mxs_lradc_can_claim_channel(iio_dev, chan);
	if (ret)
		goto err;

	/* Claim the channel */
	drv_data->claimed |= 1 << chan->address;
	data->claimed |= 1 << chan->address;

	/* Map the channel */
	for (i = 0; i < NUM_ADC_CHANNELS; i++)
		if (!(drv_data->ch[i].mapping & MAPPING_USED))
			break;

	drv_data->ch[i].mapping = MAPPING_USED |
				MAPPING_SET_SLOT(data->id) |
				MAPPING_CHAN(chan->address);

	/* Setup the mapping */
	__mxs_clrl(LRADC_CTRL4_LRADCSELECT_MASK(i),
		drv_data->mmio_base + LRADC_CTRL4);
	__mxs_setl(chan->address << LRADC_CTRL4_LRADCSELECT_OFFSET(i),
		drv_data->mmio_base + LRADC_CTRL4);

	spin_unlock_irqrestore(&drv_data->lock, gflags);

	overspl =
		((drv_data->ch_oversample[chan->address] ? 1 : 0)
			* LRADC_CH_ACCUMULATE) |
		(drv_data->ch_oversample[chan->address]
			<< LRADC_CH_NUM_SAMPLES_OFFSET);
	writel(overspl, drv_data->mmio_base  + LRADC_CH(i));

	/* Enable IRQ on the channel */
	__mxs_clrl(LRADC_CTRL1_LRADC_IRQ(i),
		drv_data->mmio_base + LRADC_CTRL1);
	__mxs_setl(LRADC_CTRL1_LRADC_IRQ_EN(i),
		drv_data->mmio_base + LRADC_CTRL1);

	/* Set out channel to be triggered by this delay queue */
	__mxs_setl(1 << (LRADC_DELAY_TRIGGER_LRADCS_OFFSET + i),
		drv_data->mmio_base + LRADC_DELAY(data->id));

	return 0;

err:
	spin_unlock_irqrestore(&drv_data->lock, gflags);
	return ret;
}

static void mxs_lradc_relinquish_channel(struct iio_dev *iio_dev,
			const struct iio_chan_spec *chan, int chanidx)
{
	struct mxs_lradc_data *data = iio_priv(iio_dev);
	struct mxs_lradc_drv_data *drv_data = data->drv_data;
	unsigned long gflags;
	int i;

	drv_data->ch[chanidx].users--;
	if (drv_data->ch[chanidx].users == 0) {
		/* No more users for this channel, stop generating interrupts */
		__mxs_setl(LRADC_CTRL1_LRADC_IRQ(i) |
			LRADC_CTRL1_LRADC_IRQ_EN(i),
			drv_data->mmio_base + LRADC_CTRL1);

		/* Don't trigger this channel */
		__mxs_clrl(1 << (LRADC_DELAY_TRIGGER_LRADCS_OFFSET + i),
			drv_data->mmio_base + LRADC_DELAY(data->id));

		spin_lock_irqsave(&drv_data->lock, gflags);

		/* Relinquish this channel */
		drv_data->claimed &= ~(1 << chan->address);
		data->claimed &= ~(1 << chan->address);
		drv_data->ch[chanidx].mapping = 0;

		spin_unlock_irqrestore(&drv_data->lock, gflags);
	}
}

/*
 * I/O operations
 */
static int mxs_lradc_read_raw(struct iio_dev *iio_dev,
			const struct iio_chan_spec *chan,
			int *val, int *val2, long m)
{
	struct mxs_lradc_data *data = iio_priv(iio_dev);
	struct mxs_lradc_drv_data *drv_data = data->drv_data;
	unsigned long lflags;
	int i, ret;

	switch (m) {
	case 0:
		spin_lock_irqsave(&data->lock, lflags);

		DBG(0, "%s: Reading channel %ld '%s' CTRL0=%08x\n", __func__,
			chan->address, chan->datasheet_name,
			readl(drv_data->mmio_base + LRADC_CTRL0));
		ret = mxs_lradc_claim_channel(iio_dev, chan);
		if (ret) {
			spin_unlock_irqrestore(&data->lock, lflags);
			return ret;
		}

		/*
		 * Once we are here, the channel is mapped by us already.
		 * Find the mapping.
		 */
		for (i = 0; i < NUM_ADC_CHANNELS; i++) {
			if (!(drv_data->ch[i].mapping & MAPPING_USED))
				continue;

			if (MAPPING_CHAN(drv_data->ch[i].mapping) ==
				chan->address)
				break;
		}

		/* Wait until sampling is done */
		drv_data->ch[i].wq_done = false;

		drv_data->ch[i].users++;

		spin_unlock_irqrestore(&data->lock, lflags);

		ret = wait_event_interruptible(drv_data->ch[i].wq,
					drv_data->ch[i].wq_done);
		if (ret == 0) {
			*val = readl(drv_data->mmio_base + LRADC_CH(i)) &
					LRADC_CH_VALUE_MASK;
			*val /= (drv_data->ch_oversample[chan->address] + 1);
			ret = IIO_VAL_INT;
		}

		spin_lock_irqsave(&data->lock, lflags);

		mxs_lradc_relinquish_channel(iio_dev, chan, i);

		spin_unlock_irqrestore(&data->lock, lflags);

		return ret;

	case IIO_CHAN_INFO_OVERSAMPLE_COUNT:
		*val = drv_data->ch_oversample[chan->address];
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int mxs_lradc_write_raw(struct iio_dev *iio_dev,
			const struct iio_chan_spec *chan,
			int val, int val2, long m)
{
	struct mxs_lradc_data *data = iio_priv(iio_dev);
	struct mxs_lradc_drv_data *drv_data = data->drv_data;

	switch (m) {
	case IIO_CHAN_INFO_OVERSAMPLE_COUNT:
		if ((val <= 0) || (val >= 32))
			return -EINVAL;

		drv_data->ch_oversample[chan->address] = val - 1;
		return 0;
	}

	return -EINVAL;
}

static irqreturn_t mxs_lradc_handle_irq(int irq, void *data)
{
	struct mxs_lradc_drv_data *drv_data = data;
	uint32_t reg = readl(drv_data->mmio_base + LRADC_CTRL1);
	int i;

	/* mask out disabled interrupts */
	reg &= reg >> 16;
	for (i = 0; i < NUM_ADC_CHANNELS; i++)
		if (reg & LRADC_CTRL1_LRADC_IRQ(i)) {
			drv_data->ch[i].wq_done = true;
			wake_up_interruptible(&drv_data->ch[i].wq);
		}
	__mxs_clrl(reg, drv_data->mmio_base + LRADC_CTRL1);

	return IRQ_HANDLED;
}

static const struct iio_info mxs_lradc_iio_info = {
	.driver_module		= THIS_MODULE,
	.read_raw		= mxs_lradc_read_raw,
	.write_raw		= mxs_lradc_write_raw,
	.attrs			= &mxs_lradc_attr_group,
};

#define LRADC_IIO_CHAN(_type, _name, _ds_name,		\
		_chan, _inf_mask, _stype) {		\
		.type = _type,				\
		.modified = IIO_NO_MOD,			\
		.indexed = 1,				\
		.processed_val = IIO_RAW,		\
		.extend_name = _name,			\
		.datasheet_name = _ds_name,		\
		.channel = _chan,			\
		.info_mask = _inf_mask,			\
		.address = _chan,			\
		.scan_index = _chan,			\
		.scan_type = _stype,			\
		}

static struct iio_chan_spec mxs_lradc_chan_spec[] = {
	[0] = LRADC_IIO_CHAN(IIO_VOLTAGE, NULL, NULL, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	[1] = LRADC_IIO_CHAN(IIO_VOLTAGE, NULL, NULL, 1,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	[2] = LRADC_IIO_CHAN(IIO_VOLTAGE, "xp", "TS X+", 2,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	[3] = LRADC_IIO_CHAN(IIO_VOLTAGE, "yp", "TS Y+", 3,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	[4] = LRADC_IIO_CHAN(IIO_VOLTAGE, "xm", "TS X-", 4,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	[5] = LRADC_IIO_CHAN(IIO_VOLTAGE, "ym", "TS Y-", 5,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	[6] = LRADC_IIO_CHAN(IIO_VOLTAGE, "wiper", "TS WIPER", 6,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	/* VBATT */
	[7] = LRADC_IIO_CHAN(IIO_VOLTAGE, "vbatt", "VBatt", 7,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	/* Temp sense 0 */
	[8] = LRADC_IIO_CHAN(IIO_TEMP, "temp0", "Temp Sense 0", 8,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	/* Temp sense 1 */
	[9] = LRADC_IIO_CHAN(IIO_TEMP, "temp1", "Temp Sense 1", 9,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	/* VDDIO */
	[10] = LRADC_IIO_CHAN(IIO_VOLTAGE, "vddio", "VDDIO", 10,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	/* VTH */
	[11] = LRADC_IIO_CHAN(IIO_VOLTAGE, "vth", "VTH", 11,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	/* VDDA */
	[12] = LRADC_IIO_CHAN(IIO_VOLTAGE, "vdda", "VDDA", 12,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	/* VDDD */
	[13] = LRADC_IIO_CHAN(IIO_VOLTAGE, "vddd", "VDDD", 13,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	/* VBG */
	[14] = LRADC_IIO_CHAN(IIO_VOLTAGE, "vbg", "VBG", 14,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 0)),
	/* VDD5V */
	[15] = LRADC_IIO_CHAN(IIO_VOLTAGE, "vdd5v", "VDD5V", 15,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		IIO_ST('u', 18, 32, 1)),
};

#if 1
static void mxs_lradc_config(struct mxs_lradc_drv_data *drv_data)
{
	/* FIXME */
	int freq = 0x3;	/* 6MHz */
	int onchip_ground_ref = 0;

	int i;

	mxs_reset_block(drv_data->mmio_base + LRADC_CTRL0);

	if (onchip_ground_ref)
		__mxs_setl(1 << 26, drv_data->mmio_base + LRADC_CTRL0);
	else
		__mxs_clrl(1 << 26, drv_data->mmio_base + LRADC_CTRL0);

	__mxs_clrl(0x3 << 8, drv_data->mmio_base + LRADC_CTRL3);
	__mxs_setl(freq, drv_data->mmio_base + LRADC_CTRL3);
	/* The delay channels constantly retrigger themself */
	for (i = 0; i < NUM_DELAY_CHANNELS; i++)
		__mxs_setl(LRADC_DELAY_KICK |
			(1 << (LRADC_DELAY_TRIGGER_DELAYS_OFFSET + i)) |
			8,	/* FIXME */
			drv_data->mmio_base + LRADC_DELAY(i));

	/* Start temperature sensing */
	writel(0, drv_data->mmio_base + LRADC_CTRL2);
}
#else
#define MXS_LRADC_CONF_TOUCH_DETECT_ENABLE	(1 << 0)

static int mxs_lradc_set_config(struct mxs_lradc_drv_data *drv_data,
				mxs_lradc_conf_data_t conf,
				mxs_lradc_conf_data_t mask)
{
	if (conf->onchip_ground_ref)
		__mxs_setl(LRADC_CTRL0_ONCHIP_GROUNDREF,
			drv_data->mmio_base + LRADC_CTRL0);
	else
		__mxs_clrl(LRADC_CTRL0_ONCHIP_GROUNDREF,
			drv_data->mmio_base + LRADC_CTRL0);

	__mxs_clrl(0x3 << 8, drv_data->mmio_base + LRADC_CTRL3);
	__mxs_setl(freq, drv_data->mmio_base + LRADC_CTRL3);

	/* The delay channels constantly retrigger themself */
	for (i = 0; i < NUM_DELAY_CHANNELS; i++)
		__mxs_setl(LRADC_DELAY_KICK |
			(1 << (LRADC_DELAY_TRIGGER_DELAYS_OFFSET + i)) |
			0x7ff,	/* FIXME */
			drv_data->mmio_base + LRADC_DELAY(i));

	/* Start temperature sensing */
	writel(0, drv_data->mmio_base + LRADC_CTRL2);
}

static void mxs_lradc_config(struct mxs_lradc_drv_data *drv_data)
{
	/* FIXME */
	struct mxs_lradc_conf_data conf = {
		.freq = 0x3;	/* 6MHz */
		.onchip_ground_ref = 0,
	};

	int i;

	mxs_reset_block(drv_data->mmio_base + LRADC_CTRL0);

}
#endif

/*static void mxs_lradc_config(struct mxs_lradc_pdata *pdata)
{

}
*/

/*
struct iio_map {
        const char *adc_channel_label; // datasheet_name
        const char *consumer_dev_name;
        const char *consumer_channel;
*/

#ifdef CONFIG_IIO_TRIGGER
static int mxs_lradc_enable_pen_detect(struct mxs_lradc_drv_data *drv_data,
				int enable, int irq)
{
	__mxs_clrl(LRADC_CTRL0_YPLLSW_MASK |
		LRADC_CTRL0_XNURSW_MASK |
		LRADC_CTRL0_XPULSW |
		LRADC_CTRL0_YNLRSW,
		drv_data->mmio_base + LRADC_CTRL0);

	if (enable) {

		/* start touch detection */
		DBG(0, "%s: setting CTRL0(%08x)\n", __func__,
			LRADC_CTRL0_TOUCH_DETECT_ENABLE);

		__mxs_setl(LRADC_CTRL0_TOUCH_DETECT_ENABLE,
			drv_data->mmio_base + LRADC_CTRL0);
	} else {
		__mxs_clrl(LRADC_CTRL0_TOUCH_DETECT_ENABLE,
			drv_data->mmio_base + LRADC_CTRL0);
	}
	if (irq) {
		__mxs_clrl(LRADC_CTRL1_LRADC_TOUCH_IRQ,
			drv_data->mmio_base + LRADC_CTRL1);

		DBG(0, "%s: setting CTRL1(%08x)\n", __func__,
			LRADC_CTRL1_LRADC_TOUCH_IRQ_EN);

		__mxs_setl(LRADC_CTRL1_LRADC_TOUCH_IRQ_EN,
			drv_data->mmio_base + LRADC_CTRL1);
	} else {
		DBG(0, "%s: clearing CTRL1(%08x)\n", __func__,
			LRADC_CTRL1_LRADC_TOUCH_IRQ_EN);

		__mxs_clrl(LRADC_CTRL1_LRADC_TOUCH_IRQ_EN,
			drv_data->mmio_base + LRADC_CTRL1);
	}
	return 0;
}

static void mxs_lradc_enable_measurement(struct mxs_lradc_drv_data *drv_data,
					uint32_t mask)
{
	uint32_t ctrl0;

	DBG(0, "%s:\n", __func__);

	ctrl0 = readl(drv_data->mmio_base + LRADC_CTRL0);
	ctrl0 &= ~(LRADC_CTRL0_XNURSW_MASK |
		LRADC_CTRL0_XPULSW |
		LRADC_CTRL0_YPLLSW_MASK |
		LRADC_CTRL0_YNLRSW);

	DBG(0, "%s: writing %08x to CTLR0\n", __func__,
		ctrl0 | mask);
	writel(ctrl0 | mask, drv_data->mmio_base + LRADC_CTRL0);
}

static void mxs_lradc_enable_x_measurement(struct mxs_lradc_drv_data *drv_data)
{
	DBG(0, "%s:\n", __func__);
	mxs_lradc_enable_measurement(drv_data, LRADC_CTRL0_YPLLSW_HIGH |
				LRADC_CTRL0_YNLRSW);
}

static void mxs_lradc_enable_y_measurement(struct mxs_lradc_drv_data *drv_data)
{
	DBG(0, "%s:\n", __func__);
	mxs_lradc_enable_measurement(drv_data, LRADC_CTRL0_XNURSW_LOW |
					LRADC_CTRL0_XPULSW);
}

static struct iio_chan_spec mxs_lradc_event_chan_spec[] = {
	[0] = {
		.type = IIO_VOLTAGE,
		.datasheet_name = "pendetect",
		.channel = 0,
		.address = 0,
		.indexed = 1,
		.scan_index = 0,
		.scan_type = IIO_ST('u', 1, 1, 0),
	},
	[1] = {
		.type = IIO_VOLTAGE,
		.datasheet_name = "threshold0",
		.channel = 1,
		.address = 1,
		.indexed = 1,
		.scan_index = 1,
		.scan_type = IIO_ST('u', 1, 1, 0),
	},
	[2] = {
		.type = IIO_VOLTAGE,
		.datasheet_name = "threshold1",
		.channel = 2,
		.address = 2,
		.indexed = 1,
		.scan_index = 2,
		.scan_type = IIO_ST('u', 1, 1, 0),
	},
	[3] = {
		.type = IIO_VOLTAGE,
		.datasheet_name = "button0",
		.channel = 3,
		.address = 3,
		.indexed = 1,
		.scan_index = 3,
		.scan_type = IIO_ST('u', 1, 1, 0),
	},
	[4] = {
		.type = IIO_VOLTAGE,
		.datasheet_name = "button1",
		.channel = 4,
		.address = 4,
		.indexed = 1,
		.scan_index = 4,
		.scan_type = IIO_ST('u', 1, 1, 0),
	},
};

static int mxs_lradc_event_read_raw(struct iio_dev *iio_dev,
				const struct iio_chan_spec *chan,
				int *val, int *val2, long m)
{
	DBG(0, "%s: dev %p chan %p val %p val2 %p m %ld\n",
		__func__, iio_dev, chan, val, val2, m);
	return IIO_VAL_INT;
}

static int mxs_lradc_event_write_raw(struct iio_dev *iio_dev,
				const struct iio_chan_spec *chan,
				int val, int val2, long m)
{
	DBG(0, "%s: dev %p chan %p val %i val2 %i m %ld\n",
		__func__, iio_dev, chan, val, val2, m);
	return 0;
}

static int mxs_lradc_read_event_config(struct iio_dev *iio, u64 code)
{
	struct iio_trigger *trig = iio->trig;
	struct mxs_lradc_trigger *trg = trig->private_data;
	struct mxs_lradc_drv_data *drv_data = trg->drv_data;

	drv_data->status_mask = readl(drv_data->mmio_base + LRADC_STATUS);
	DBG(0, "%s: %p %llu status %08x\n", __func__, iio, code,
		drv_data->status_mask);
	return drv_data->status_mask & LRADC_STATUS_TOUCH_DETECT;
}

static int mxs_lradc_read_event_value(struct iio_dev *iio,
				u64 code, int *val)
{
	DBG(0, "%s: %p %llu\n", __func__, iio, code);
	return 0;
}

enum MXS_LRADC_TS_STATE {
	STATE_DETECT,
	STATE_PENDOWN,
	STATE_MEASURE_X,
	STATE_MEASURE_Y,
	STATE_MEASURE_P,
};

static int mxs_lradc_write_event_config(struct iio_dev *iio,
				u64 code, int state)
{
	struct iio_trigger *trig = iio->trig;
	struct mxs_lradc_trigger *trg = trig->private_data;
	struct mxs_lradc_drv_data *drv_data = trg->drv_data;

	DBG(0, "%s: %p %llu: %d\n", __func__, iio, code, state);
	switch (state) {
	case STATE_DETECT:
		mxs_lradc_enable_pen_detect(drv_data, 1, 1);
		break;

	case STATE_PENDOWN:
		mxs_lradc_enable_pen_detect(drv_data, 1, 0);
		break;

	case STATE_MEASURE_X:
		mxs_lradc_enable_pen_detect(drv_data, 0, 0);
		mxs_lradc_enable_x_measurement(drv_data);
		break;

	case STATE_MEASURE_Y:
		mxs_lradc_enable_y_measurement(drv_data);
		break;

	case STATE_MEASURE_P:
		mxs_lradc_enable_pen_detect(drv_data, 1, 0);
		break;
	}
	return 0;
}

static int mxs_lradc_write_event_value(struct iio_dev *iio,
				u64 code, int val)
{
	DBG(0, "%s: %p %llu: %d\n", __func__, iio, code, val);
	return 0;
}

static const struct iio_info mxs_lradc_event_iio_info = {
	.driver_module		= THIS_MODULE,
	.read_raw		= mxs_lradc_event_read_raw,
	.write_raw		= mxs_lradc_event_write_raw,
	.read_event_value	= mxs_lradc_read_event_value,
	.read_event_config	= mxs_lradc_read_event_config,
	.write_event_value	= mxs_lradc_write_event_value,
	.write_event_config	= mxs_lradc_write_event_config,
};

static irqreturn_t mxs_lradc_handle_trigger(int irq, void *data)
{
	struct iio_trigger *trig = data;
	struct mxs_lradc_trigger *trg = trig->private_data;
	struct mxs_lradc_drv_data *drv_data = trg->drv_data;

	uint32_t ctrl1 = readl(drv_data->mmio_base + LRADC_CTRL1);
	s64 ts = iio_get_time_ns();
	int id;

	dev_dbg(&trig->dev, "%s: Handling IRQ %d\n", __func__, irq);

	drv_data->status_mask = readl(drv_data->mmio_base + LRADC_STATUS);
	/* mask out disabled interrupts */
	ctrl1 &= ctrl1 >> 16;
	for (id = 0; id < NUM_TRIGGERS; id++) {
		if (ctrl1 & LRADC_CTRL1_LRADC_IRQ(id + 8)) {
			DBG(0, "Handling trigger %p '%s' id: %d\n",
				trig, trig->name, id);
			iio_trigger_poll(trig, ts);
		}
	}
	__mxs_clrl(ctrl1, drv_data->mmio_base + LRADC_CTRL1);

	return IRQ_HANDLED;
}

static struct iio_map mxs_lradc_chan_map[] = {
	{ "TS X+", "mxs-lradc-ts", "in_voltage2_xp_raw", },
	{ "TS Y+", "mxs-lradc-ts", "in_voltage3_yp_raw", },
	{ "TS X-", "mxs-lradc-ts", "in_voltage4_xm_raw", },
	{ "TS Y-", "mxs-lradc-ts", "in_voltage5_ym_raw", },
	{ "TS WIPER", "mxs-lradc-ts", "in_voltage6_wiper_raw", },
	{ /* end of list sentinel */ }
};

static struct iio_map mxs_lradc_event_chan_map[] = {
	{ "pendetect", "mxs-lradc-ts", "pendetect", },
	{ /* end of list sentinel */ }
};

static int mxs_lradc_set_trigger_state(struct iio_trigger *trig, bool state)
{
	struct mxs_lradc_trigger *trg = trig->private_data;
	struct mxs_lradc_drv_data *drv_data = trg->drv_data;

	DBG(0, "%s: trigger %p id %d '%s' -> %d\n", __func__,
		trig, trig->id, trig->name, state);
	mxs_lradc_enable_pen_detect(drv_data, state, state);
	return 0;
}

static const struct iio_trigger_ops mxs_lradc_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = mxs_lradc_set_trigger_state,
};

static const char *mxs_lradc_trigger_names[] = {
	"pendetect",
	"threshold0",
	"threshold1",
	"button0",
	"button1",
};

static inline void mxs_lradc_remove_trigger(struct iio_dev *iio)
{
	if (iio == NULL)
		return;

	DBG(0, "Unregistering iio_map %p from %p\n",
		mxs_lradc_event_chan_map, iio);
	iio_map_array_unregister(iio, mxs_lradc_event_chan_map);

	DBG(0, "%s: Unregistering trigger %s\n", __func__,
		iio->trig->name);
	iio_trigger_unregister(iio->trig);

	DBG(0, "%s: Unregistering iio_dev %p\n", __func__,
		iio->trig->name);
	iio_device_unregister(iio);

	DBG(0, "%s: Freeing trigger %p\n", __func__,
		iio->trig);
	iio_free_trigger(iio->trig);

	DBG(0, "%s: Freeing iio_dev %p\n", __func__, iio);
	iio_free_device(iio);
}

static inline void mxs_lradc_remove_triggers(struct mxs_lradc_drv_data *drv_data)
{
	int i;

	for (i = 0; i < NUM_TRIGGERS; i++)
		mxs_lradc_remove_trigger(drv_data->trig[i]);
}

static int mxs_lradc_alloc_trigger(struct platform_device *pdev,
				struct mxs_lradc_drv_data *drv_data, int index)
{
	int ret;
	struct iio_trigger *trig;
	struct mxs_lradc_trigger *data;
	struct iio_dev *iio;
	int irq;

	irq = platform_get_irq(pdev, NUM_ADC_CHANNELS + index);
	if (irq <= 0) {
		dev_err(&pdev->dev, "IRQ resource[%d] not defined\n", irq);
		return irq ?: -ENODEV;
	}

	iio = iio_allocate_device(sizeof(*data));
	if (iio == NULL)
		return -ENOMEM;

	iio->name = pdev->name;
	iio->dev.parent = &pdev->dev;
	iio->info = &mxs_lradc_event_iio_info;
	iio->modes = INDIO_BUFFER_TRIGGERED;
	/* Channels */
	iio->channels = mxs_lradc_event_chan_spec;
	iio->num_channels = ARRAY_SIZE(mxs_lradc_event_chan_spec);

	data = iio_priv(iio);

	trig = iio_allocate_trigger(mxs_lradc_trigger_names[index]);
	if (trig == NULL) {
		ret = -ENOMEM;
		goto err_free_dev;
	}

	data->id = index + NUM_ADC_CHANNELS;
	data->drv_data = drv_data;
	data->irq = irq;

	trig->dev.parent = &pdev->dev;
	trig->ops = &mxs_lradc_trigger_ops;
	trig->private_data = data;

	ret = devm_request_irq(&pdev->dev, irq, mxs_lradc_handle_trigger, 0,
			trig->name, trig);
	if (ret) {
		dev_err(&pdev->dev, "devm_request_irq %i failed: %d\n",
			irq, ret);
		ret = -EBUSY;
		goto err_free_trigger;
	}

	ret = iio_trigger_register(trig);
	if (ret)
		goto err_free_trigger;

	ret = iio_map_array_register(iio, mxs_lradc_event_chan_map);
	if (ret)
		goto err_trigger_unregister;

	iio->trig = trig;

	ret = iio_device_register(iio);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register Trigger IIO device\n");
		goto err_iio_map_unregister;
	}
	drv_data->trig[index] = iio;

	DBG(0, "%s: Created iio_dev[%d] %p '%s' trigger %p '%s'\n",
		__func__, index, iio, iio->name, trig, trig->name);

	return 0;

err_iio_map_unregister:
	iio_map_array_unregister(iio, mxs_lradc_event_chan_map);

err_trigger_unregister:
	iio_trigger_unregister(trig);

err_free_trigger:
	iio_free_trigger(trig);
err_free_dev:
	iio_free_device(iio);

	return ret;
}

static int __devinit mxs_lradc_install_triggers(struct platform_device *pdev,
			struct mxs_lradc_drv_data *drv_data)
{
	int ret;
	int i;

	/*
	 * Register triggers
	 */
	for (i = 0; i < NUM_TRIGGERS; i++) {
		ret = mxs_lradc_alloc_trigger(pdev, drv_data, i);
		if (ret)
			goto err;
	}

	return 0;

err:
	while (--i >= 0)
		mxs_lradc_remove_trigger(drv_data->trig[i]);

	return ret;
}
#else
static inline int mxs_lradc_install_triggers(struct platform_device *pdev,
			struct mxs_lradc_drv_data *drv_data)
{
	return 0;
}

static inline void mxs_lradc_remove_triggers(struct mxs_lradc_drv_data *drv_data)
{
}
#endif

static int __devinit mxs_lradc_probe(struct platform_device *pdev)
{
	struct mxs_lradc_drv_data *drv_data;
	struct iio_dev *iio;
	struct resource *r;
	int ret = 0;
	int irq;
	int i;

	/*
	 * DEVM management
	 */
	if (!devres_open_group(&pdev->dev, mxs_lradc_probe, GFP_KERNEL)) {
		dev_err(&pdev->dev, "Can't open resource group\n");
		return -ENOMEM;
	}

	drv_data = devm_kzalloc(&pdev->dev, sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data) {
		dev_err(&pdev->dev, "Failed to allocate driver data\n");
		ret = -ENOMEM;
		goto err0;
	}

	spin_lock_init(&drv_data->lock);

	/*
	 * IIO ops
	 */
	for (i = 0; i < NUM_DELAY_CHANNELS; i++) {
		struct mxs_lradc_data *data;

		iio = iio_allocate_device(sizeof(*data));
		if (!iio) {
			dev_err(&pdev->dev,
				"Failed to allocate IIO device %i\n", i);
			ret = -ENOMEM;
			goto err1;
		}

		iio->name = pdev->name;
		iio->dev.parent = &pdev->dev;
		iio->info = &mxs_lradc_iio_info;
		iio->modes = INDIO_DIRECT_MODE;
		/* Channels */
		iio->channels = mxs_lradc_chan_spec;
		iio->num_channels = ARRAY_SIZE(mxs_lradc_chan_spec);

		data = iio_priv(iio);
		data->drv_data = drv_data;
		data->id = i;

		spin_lock_init(&data->lock);

		drv_data->iio[i] = iio;
	}

	for (i = 0; i < NUM_ADC_CHANNELS; i++)
		init_waitqueue_head(&drv_data->ch[i].wq);

	dev_set_drvdata(&pdev->dev, drv_data);

	/*
	 * Allocate address space
	 */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "No I/O memory resource defined\n");
		ret = -ENODEV;
		goto err2;
	}

	r = devm_request_mem_region(&pdev->dev, r->start,
				resource_size(r), pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "Failed to request I/O memory\n");
		ret = -EBUSY;
		goto err2;
	}

	drv_data->mmio_base = devm_ioremap(&pdev->dev, r->start,
					resource_size(r));
	if (!drv_data->mmio_base) {
		dev_err(&pdev->dev, "Failed to map I/O memory\n");
		ret = -ENOMEM;
		goto err2;
	}

	/*
	 * Allocate IRQ
	 */
	for (irq = 0; irq < NUM_ADC_CHANNELS; irq++) {
		r = platform_get_resource(pdev, IORESOURCE_IRQ, irq);
		if (r == NULL) {
			dev_err(&pdev->dev, "IRQ resource[%d] not defined\n",
				irq);
			ret = -ENODEV;
			goto err2;
		}

		ret = devm_request_irq(&pdev->dev, r->start,
				mxs_lradc_handle_irq, 0, r->name, drv_data);
		if (ret) {
			dev_err(&pdev->dev, "devm_request_irq %i failed: %d\n",
				irq, ret);
			goto err2;
		}
	}

	/*
	 * Register IIO device
	 */
	for (i = 0; i < NUM_DELAY_CHANNELS; i++) {
		ret = iio_map_array_register(drv_data->iio[i], mxs_lradc_chan_map);
		if (ret)
			goto err3;

		ret = iio_device_register(drv_data->iio[i]);
		if (ret) {
			dev_err(&pdev->dev, "Failed to register IIO device\n");
			goto err3a;
		}
	}

	ret = mxs_lradc_install_triggers(pdev, drv_data);
	if (ret)
		goto err3;

	devres_remove_group(&pdev->dev, mxs_lradc_probe);

	mxs_lradc_config(drv_data);

	return 0;

err3:
	while (--i >= 0) {
		iio_map_array_unregister(drv_data->iio[i], mxs_lradc_chan_map);
	err3a:
		iio_device_unregister(drv_data->iio[i]);
	}
err2:
	i = NUM_DELAY_CHANNELS;
err1:
	while (--i >= 0)
		iio_free_device(drv_data->iio[i]);
err0:
	devres_release_group(&pdev->dev, mxs_lradc_probe);
	return ret;
}

static int __devexit mxs_lradc_remove(struct platform_device *pdev)
{
	struct mxs_lradc_drv_data *drv_data = dev_get_drvdata(&pdev->dev);
	int i;

	mxs_lradc_remove_triggers(drv_data);

	for (i = 0; i < NUM_DELAY_CHANNELS; i++) {
		dev_dbg(&pdev->dev, "%s: Unregistering iio_map %p for %p\n",
			__func__, mxs_lradc_chan_map, drv_data->iio[i]);
		iio_map_array_unregister(drv_data->iio[i], mxs_lradc_chan_map);

		dev_dbg(&pdev->dev, "%s: Unregistering iio_dev %p\n",
			__func__, drv_data->iio[i]);
		iio_device_unregister(drv_data->iio[i]);
	}

	for (i = 0; i < NUM_DELAY_CHANNELS; i++) {
		dev_dbg(&pdev->dev, "%s: Freeing iio_dev %p\n", __func__,
			drv_data->iio[i]);
		iio_free_device(drv_data->iio[i]);
	}
	return 0;
}

static struct platform_driver mxs_lradc_driver = {
	.driver = {
		.name = "mxs-lradc",
	},
	.probe = mxs_lradc_probe,
	.remove = __devexit_p(mxs_lradc_remove),
};

module_platform_driver(mxs_lradc_driver);

MODULE_AUTHOR("Marek Vasut <marek.vasut@...>");
MODULE_DESCRIPTION("Freescale i.MX23/i.MX28 LRADC driver");
MODULE_LICENSE("GPL v2");
