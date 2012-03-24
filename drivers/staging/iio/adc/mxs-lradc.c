/*
 * ADT7410 digital temperature sensor driver supporting ADT7410
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

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

#include "../iio.h"
#include "../sysfs.h"

#define NUM_ADC_CHANNELS	8
#define NUM_DELAY_CHANNELS	4

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
};

struct mxs_lradc_drv_data {
	struct iio_dev			*iio[NUM_DELAY_CHANNELS];
	void __iomem			*mmio_base;

	spinlock_t			lock;

	uint16_t			claimed;

	struct mxs_lradc_mapped_channel	ch[NUM_ADC_CHANNELS];
	uint8_t				ch_oversample[2 * NUM_ADC_CHANNELS];
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

#define	LRADC_CTRL1				0x10
#define	LRADC_CTRL1_LRADC_IRQ(n)		(1 << (n))
#define	LRADC_CTRL1_LRADC_IRQ_EN(n)		(1 << ((n) + 16))

#define	LRADC_CTRL2				0x20
#define	LRADC_CTRL2_TEMPSENSE_PWD		(1 << 15)

#define	LRADC_CTRL3				0x30

#define	LRADC_CTRL4				0x140
#define	LRADC_CTRL4_LRADCSELECT_MASK(n)		(0xf << ((n) * 4))
#define	LRADC_CTRL4_LRADCSELECT_OFFSET(n)	((n) * 4)

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
		return -EINVAL;

	for (i = 0; i < 16; i++)
		if (drv_data->claimed & (1 << i))
			count++;

	/* Too many channels claimed */
	if (count >= NUM_ADC_CHANNELS)
		return -EINVAL;

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

	/* Set out channel to be triggers by this delay queue */
	__mxs_setl(1 << (LRADC_DELAY_TRIGGER_LRADCS_OFFSET + i),
		drv_data->mmio_base + LRADC_DELAY(data->id));

	return 0;

err:
	spin_unlock_irqrestore(&drv_data->lock, gflags);
	return -EINVAL;
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

static const struct iio_chan_spec mxs_lradc_chan_spec[] = {
	[0] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 0, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		0, 0, IIO_ST('u', 18, 32, 0), 0),
	[1] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 1, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		1, 1, IIO_ST('u', 18, 32, 0), 0),
	[2] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 2, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		2, 2, IIO_ST('u', 18, 32, 0), 0),
	[3] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 3, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		3, 3, IIO_ST('u', 18, 32, 0), 0),
	[4] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 4, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		4, 4, IIO_ST('u', 18, 32, 0), 0),
	[5] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 5, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		5, 5, IIO_ST('u', 18, 32, 0), 0),
	[6] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 6, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		6, 6, IIO_ST('u', 18, 32, 0), 0),
	/* VBATT */
	[7] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 7, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		7, 7, IIO_ST('u', 18, 32, 0), 0),
	/* Temp sense 0 */
	[8] = IIO_CHAN(IIO_TEMP, IIO_NO_MOD, 1, IIO_RAW, NULL, 8, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		8, 8, IIO_ST('u', 18, 32, 0), 0),
	/* Temp sense 1 */
	[9] = IIO_CHAN(IIO_TEMP, IIO_NO_MOD, 1, IIO_RAW, NULL, 9, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		9, 9, IIO_ST('u', 18, 32, 0), 0),
	/* VDDIO */
	[10] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 10, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		10, 10, IIO_ST('u', 18, 32, 0), 0),
	/* VTH */
	[11] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 11, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		11, 11, IIO_ST('u', 18, 32, 0), 0),
	/* VDDA */
	[12] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 12, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		12, 12, IIO_ST('u', 18, 32, 0), 0),
	/* VDDD */
	[13] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 13, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		13, 13, IIO_ST('u', 18, 32, 0), 0),
	/* VBG */
	[14] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 14, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		14, 14, IIO_ST('u', 18, 32, 0), 0),
	/* VDD5V */
	[15] = IIO_CHAN(IIO_VOLTAGE, IIO_NO_MOD, 1, IIO_RAW, NULL, 15, 0,
		IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
		IIO_CHAN_INFO_OVERSAMPLE_COUNT_SEPARATE_BIT,
		15, 15, IIO_ST('u', 18, 32, 0), 0),
};

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
			0x7ff,	/* FIXME */
			drv_data->mmio_base + LRADC_DELAY(i));

	/* Start temperature sensing */
	writel(0, drv_data->mmio_base + LRADC_CTRL2);
}

/*static void mxs_lradc_config(struct mxs_lradc_pdata *pdata)
{

}
*/
static int __devinit mxs_lradc_probe(struct platform_device *pdev)
{
	struct mxs_lradc_data *data[NUM_DELAY_CHANNELS];
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
		iio = iio_allocate_device(sizeof(*data[i]));
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

		data[i] = iio_priv(iio);
		data[i]->drv_data = drv_data;
		data[i]->id = i;

		spin_lock_init(&data[i]->lock);

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
		goto err1;
	}

	r = devm_request_mem_region(&pdev->dev, r->start,
				resource_size(r), pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "Failed to request I/O memory\n");
		ret = -EBUSY;
		goto err1;
	}

	drv_data->mmio_base = devm_ioremap(&pdev->dev, r->start,
					resource_size(r));
	if (!drv_data->mmio_base) {
		dev_err(&pdev->dev, "Failed to map I/O memory\n");
		ret = -ENOMEM;
		goto err1;
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
			goto err1;
		}

		ret = request_irq(r->start, mxs_lradc_handle_irq, 0,
				r->name, drv_data);
		if (ret) {
			dev_err(&pdev->dev, "request_irq %i failed: %d\n",
				irq, ret);
			goto err1;
		}
	}

	/*
	 * Register IIO device
	 */
	for (i = 0; i < NUM_DELAY_CHANNELS; i++) {
		ret = iio_device_register(drv_data->iio[i]);
		if (ret) {
			dev_err(&pdev->dev, "Failed to register IIO device\n");
			ret = -EBUSY;
			goto err2;
		}
	}

	devres_remove_group(&pdev->dev, mxs_lradc_probe);

	mxs_lradc_config(drv_data);

	return 0;

err2:
	while (--i >= 0)
		iio_device_unregister(drv_data->iio[i]);
err1:
	for (i = 0; i < NUM_DELAY_CHANNELS; i++)
		if (drv_data->iio[i])
			iio_free_device(drv_data->iio[i]);
err0:
	devres_release_group(&pdev->dev, mxs_lradc_probe);
	return ret;
}

static int __devexit mxs_lradc_remove(struct platform_device *pdev)
{
	struct mxs_lradc_drv_data *drv_data = dev_get_drvdata(&pdev->dev);
	struct resource *r;
	int i, irq;

	for (i = 0; i < NUM_DELAY_CHANNELS; i++) {
		iio_device_unregister(drv_data->iio[i]);

	for (i = 0; i < NUM_ADC_CHANNELS; i++) {
		r = platform_get_resource(pdev, IORESOURCE_IRQ, irq);
		if (r != NULL)
			free_irq(r->start, drv_data);
	}


	for (i = 0; i < NUM_DELAY_CHANNELS; i++)
		iio_free_device(drv_data->iio[i]);
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
