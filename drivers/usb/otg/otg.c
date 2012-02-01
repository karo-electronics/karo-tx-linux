/*
 * otg.c -- USB OTG utility code
 *
 * Copyright (C) 2004 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/usb/otg.h>

static struct otg_transceiver *xceiv;

static LIST_HEAD(transceiver_list);

/**
 * otg_get_transceiver - find the (single) OTG transceiver
 *
 * Returns the transceiver driver, after getting a refcount to it; or
 * null if there is no such transceiver.  The caller is responsible for
 * calling otg_put_transceiver() to release that count.
 *
 * For use by USB host and peripheral drivers.
 */
struct otg_transceiver *otg_get_transceiver(void)
{
	if (xceiv)
		get_device(xceiv->dev);
	return xceiv;
}
EXPORT_SYMBOL(otg_get_transceiver);

/**
 * otg_find_transceiver - find an OTG transceiver for a specific device
 *
 * Returns the transceiver driver, after getting a refcount to it; or
 * null if there is no such transceiver.  The caller is responsible for
 * calling otg_put_transceiver() to release that count.
 *
 * For use by USB host and peripheral drivers.
 */
struct otg_transceiver *otg_find_transceiver(struct device *dev)
{
	const char *devname = dev_name(dev);
	static struct otg_transceiver *x;

	if (xceiv) {
		get_device(xceiv->dev);
		return xceiv;
	}

	list_for_each_entry(x, &transceiver_list, list) {
		pr_debug("%s: dev_id_host=%p dev_id_peripheral=%p\n", __func__,
			x->dev_id_host, x->dev_id_peripheral);
		pr_debug("%s: %s %s\n", __func__, x->dev_id_host, x->dev_id_peripheral);
		if (x->dev_id_host && !strcmp(x->dev_id_host, devname))
			goto found;
		if (x->dev_id_peripheral && !strcmp(x->dev_id_peripheral, devname))
			goto found;
	}
	pr_debug("%s: No transceiver found for %s\n", __func__, devname);

	return NULL;

found:
	get_device(x->dev);
	return x;

}
EXPORT_SYMBOL(otg_find_transceiver);

/**
 * otg_put_transceiver - release the (single) OTG transceiver
 * @x: the transceiver returned by otg_get_transceiver()
 *
 * Releases a refcount the caller received from otg_get_transceiver().
 *
 * For use by USB host and peripheral drivers.
 */
void otg_put_transceiver(struct otg_transceiver *x)
{
	if (x)
		put_device(x->dev);
}
EXPORT_SYMBOL(otg_put_transceiver);

/**
 * otg_set_transceiver - declare the (single) OTG transceiver
 * @x: the USB OTG transceiver to be used; or NULL
 *
 * This call is exclusively for use by transceiver drivers, which
 * coordinate the activities of drivers for host and peripheral
 * controllers, and in some cases for VBUS current regulation.
 */
int otg_set_transceiver(struct otg_transceiver *x)
{
	if (xceiv && x)
		return -EBUSY;
	xceiv = x;
	return 0;
}
EXPORT_SYMBOL(otg_set_transceiver);

int otg_add_transceiver(struct otg_transceiver *x)
{
	pr_debug("%s: Adding xceiver: dev_id_host=%p dev_id_peripheral=%p\n", __func__,
		x->dev_id_host, x->dev_id_peripheral);
	pr_debug("%s: %s %s\n", __func__, x->dev_id_host, x->dev_id_peripheral);
	list_add_tail(&x->list, &transceiver_list);

	return 0;
}
EXPORT_SYMBOL(otg_add_transceiver);

int otg_remove_transceiver(struct otg_transceiver *x)
{
	list_del(&x->list);

	return 0;
}
EXPORT_SYMBOL(otg_remove_transceiver);

const char *otg_state_string(enum usb_otg_state state)
{
	switch (state) {
	case OTG_STATE_A_IDLE:
		return "a_idle";
	case OTG_STATE_A_WAIT_VRISE:
		return "a_wait_vrise";
	case OTG_STATE_A_WAIT_BCON:
		return "a_wait_bcon";
	case OTG_STATE_A_HOST:
		return "a_host";
	case OTG_STATE_A_SUSPEND:
		return "a_suspend";
	case OTG_STATE_A_PERIPHERAL:
		return "a_peripheral";
	case OTG_STATE_A_WAIT_VFALL:
		return "a_wait_vfall";
	case OTG_STATE_A_VBUS_ERR:
		return "a_vbus_err";
	case OTG_STATE_B_IDLE:
		return "b_idle";
	case OTG_STATE_B_SRP_INIT:
		return "b_srp_init";
	case OTG_STATE_B_PERIPHERAL:
		return "b_peripheral";
	case OTG_STATE_B_WAIT_ACON:
		return "b_wait_acon";
	case OTG_STATE_B_HOST:
		return "b_host";
	default:
		return "UNDEFINED";
	}
}
EXPORT_SYMBOL(otg_state_string);
