/*
 * Copyright (C) 2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx_ipuv3_data_entry_single(soc)				\
	{								\
		.iobase = soc ## _IPU_CTRL_BASE_ADDR,			\
		.irq_err = soc ## _INT_IPU_ERR,				\
		.irq = soc ## _INT_IPU_SYN,				\
	}

#ifdef CONFIG_SOC_IMX51
const struct imx_ipuv3_data imx51_ipuv3_data __initconst =
	imx_ipuv3_data_entry_single(MX51);
#endif /* ifdef CONFIG_SOC_IMX51 */

#ifdef CONFIG_SOC_IMX53
const struct imx_ipuv3_data imx53_ipuv3_data __initconst =
	imx_ipuv3_data_entry_single(MX53);
#endif /* ifdef CONFIG_SOC_IMX53 */

struct platform_device *__init imx_add_ipuv3(
		const struct imx_ipuv3_data *data,
		const struct imx_ipuv3_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_512M - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		}, {
			.start = data->irq_err,
			.end = data->irq_err,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device("imx-ipuv3", -1,
			res, ARRAY_SIZE(res), pdata, sizeof(*pdata));
}

