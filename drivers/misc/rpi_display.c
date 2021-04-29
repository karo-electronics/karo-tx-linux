// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Raspberrypi 7inch i2c driver.
 *
 * Copyright (C) 2020 Markus Bauer <MB@karo-electronics.de>
 *
 * based on: drivers/misc/rpi_display.c
 * from: https://github.com/TinkerEdgeT/mendel-linux-imx
 *
 *     Copyright (c) 2016 ASUSTek Computer Inc.
 *     Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 */

#include <drm/rpi_display.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include "rpi_display.h"

static struct rpi_display_data *g_mcu_data;
static int connected;

static int send_cmds(struct i2c_client *client, const char *buf, size_t size)
{
	int ret;

	print_hex_dump_bytes("send_cmds: ", DUMP_PREFIX_OFFSET, buf, size);

	ret = i2c_master_send(client, buf, size);
	if (ret <= 0) {
		LOG_ERR("send command failed, ret = %d\n", ret);
		return ret ?: -ECOMM;
	}
	usleep_range(20000, 21000);
	return ret;
}

static int recv_cmds(struct i2c_client *client, char *buf, int size)
{
	int ret;

	ret = i2c_master_recv(client, buf, size);
	if (ret <= 0) {
		LOG_ERR("receive commands failed, %d\n", ret);
		return ret ?: -ECOMM;
	}
	usleep_range(20000, 21000);
	return ret;
}

static int init_cmd_check(struct rpi_display_data *mcu_data)
{
	int ret;
	char buf[1] = { 0x80 };

	ret = send_cmds(mcu_data->client, buf, sizeof(buf));
	if (ret < 0)
		goto error;

	ret = recv_cmds(mcu_data->client, buf, 1);
	if (ret < 0)
		goto error;

	LOG_INFO("recv_cmds: 0x%02X\n", buf[0]);
	if (buf[0] != 0xC3 && buf[0] != 0xDE) {
		LOG_ERR("received unexpected msg: %02x\n", buf[0]);
		ret = -EINVAL;
		goto error;

	}
	return 0;

error:
	return ret;
}

int rpi_display_screen_power_up(void)
{
	int ret;
	char buf[2];

	if (!connected)
		return -ENODEV;

	LOG_INFO("\n");

	buf[0] = 0x85;
	buf[1] = 0x00;
	ret = send_cmds(g_mcu_data->client, buf, sizeof(buf));
	if (ret <= 0)
		return ret ?: -ENOMSG;
	msleep(800);

	buf[1] = 0x01;
	ret = send_cmds(g_mcu_data->client, buf, sizeof(buf));
	if (ret <= 0)
		return ret ?: -ENOMSG;
	// without sleep next command won't be send
	usleep_range(20000, 21000);

	buf[0] = 0x81;
	buf[1] = 0x04;
	ret = send_cmds(g_mcu_data->client, buf, sizeof(buf));
	if (ret <= 0)
		return ret ?: -ENOMSG;
	return 0;
}
EXPORT_SYMBOL_GPL(rpi_display_screen_power_up);

int rpi_display_set_bright(int bright)
{
	unsigned char cmd[2];
	int ret;

	if (!connected)
		return -ENODEV;

	if (bright > 0xff || bright < 0)
		return -EINVAL;

	LOG_INFO("bright = 0x%x\n", bright);

	cmd[0] = 0x86;
	cmd[1] = bright;

	ret = i2c_master_send(g_mcu_data->client, cmd, 2);
	if (ret <= 0) {
		LOG_ERR("send command failed, ret = %d\n", ret);
		return ret ?: -ECOMM;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(rpi_display_set_bright);

int rpi_display_is_connected(void)
{
	return connected;
}
EXPORT_SYMBOL_GPL(rpi_display_is_connected);

static int rpi_display_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct rpi_display_data *mcu_data;
	int ret;

	LOG_INFO("address = 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		LOG_ERR("I2C check functionality failed\n");
		return -ENODEV;
	}

	mcu_data = kzalloc(sizeof(struct rpi_display_data), GFP_KERNEL);
	if (mcu_data == NULL) {
		LOG_ERR("no memory for device\n");
		return -ENOMEM;
	}

	mcu_data->client = client;
	i2c_set_clientdata(client, mcu_data);
	g_mcu_data = mcu_data;

	ret = init_cmd_check(mcu_data);
	if (ret < 0) {
		LOG_ERR("init_cmd_check failed, %d\n", ret);
		goto error;
	}
	connected = 1;

	return 0;

error:
	kfree(mcu_data);
	return ret;
}

static int rpi_display_remove(struct i2c_client *client)
{
	struct rpi_display_data *mcu_data = i2c_get_clientdata(client);

	connected = 0;
	kfree(mcu_data);
	return 0;
}

static const struct i2c_device_id rpi_display_id[] = {
	{ "rpi_display", },
	{ /* sentinel */ }
};

static struct i2c_driver rpi_display_driver = {
	.driver = {
		.name = "rpi_display",
	},
	.probe = rpi_display_probe,
	.remove = rpi_display_remove,
	.id_table = rpi_display_id,
};
module_i2c_driver(rpi_display_driver);

MODULE_DESCRIPTION("Tinker Board TouchScreen MCU driver");
MODULE_LICENSE("GPL v2");
