/*
 * Copyright (C) 2011 Pengutronix
 * Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#ifndef __MACH_USBPHY_H
#define __MACH_USBPHY_H

#define MXS_USBPHY_HOST		(1 << 0)
#define MXS_USBPHY_DEVICE	(1 << 1)
#define MXS_USBPHY_OTG		(1 << 2)

struct mxs_mxs_usbphy_platform_data {
	int (*set_vbus)(int on);
	const char *dev_id_host;
	const char *dev_id_peripheral;
	unsigned flags;
};

#endif /* __MACH_USBPHY_H */
