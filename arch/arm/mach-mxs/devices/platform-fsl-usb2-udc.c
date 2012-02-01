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

#define mxs_fsl_usb2_udc_data_entry_single(soc)				\
	{								\
		.iobase = soc ## _USBCTRL0_BASE_ADDR,			\
		.irq = soc ## _INT_USB0,				\
	}

#ifdef CONFIG_SOC_IMX23
const struct mxs_fsl_usb2_udc_data mx23_fsl_usb2_udc_data __initconst =
	mxs_fsl_usb2_udc_data_entry_single(MX23);
#endif /* ifdef CONFIG_SOC_IMX23 */

#ifdef CONFIG_SOC_IMX28
const struct mxs_fsl_usb2_udc_data mx28_fsl_usb2_udc_data __initconst =
	mxs_fsl_usb2_udc_data_entry_single(MX28);
#endif /* ifdef CONFIG_SOC_IMX28 */

struct platform_device *__init mxs_add_fsl_usb2_udc(
		const struct mxs_fsl_usb2_udc_data *data,
		const struct fsl_usb2_platform_data *pdata)
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
	return mxs_add_platform_device_dmamask("fsl-usb2-udc", -1,
			res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata), DMA_BIT_MASK(32));
}
