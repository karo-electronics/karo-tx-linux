/*
 * Copyright (C) 2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <mach/devices-common.h>
#include <mach/mx23.h>
#include <mach/mx28.h>

#define mxs_mxs_usbphy_data_entry_single(soc, _id)			\
	{								\
		.id = _id,						\
		.iobase = soc ## _USBPHY ## _id ## _BASE_ADDR,		\
		.irq = soc ## _INT_USB ## _id ## _WAKEUP,		\
	}

#ifdef CONFIG_SOC_IMX23
const struct mxs_mxs_usbphy_data mx23_mxs_usbphy_data __initconst =
	mxs_mxs_usbphy_data_entry_single(MX23, 0);
#endif /* ifdef CONFIG_SOC_IMX23 */

#ifdef CONFIG_SOC_IMX28
const struct mxs_mxs_usbphy_data mx28_mxs_usbphy_data[] __initconst = {
	mxs_mxs_usbphy_data_entry_single(MX28, 0),
	mxs_mxs_usbphy_data_entry_single(MX28, 1),
};
#endif /* ifdef CONFIG_SOC_IMX28 */

struct platform_device *__init mxs_add_mxs_usbphy(
		const struct mxs_mxs_usbphy_data *data,
		const struct mxs_mxs_usbphy_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_512 - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};
	return mxs_add_platform_device_dmamask("mxs-usbphy", data->id,
			res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata), DMA_BIT_MASK(32));
}
