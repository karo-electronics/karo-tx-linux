/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _RPI_DISPLAY_H_
#define _RPI_DISPLAY_H_

#define LOG_INFO(fmt, arg...) pr_info("raspi-display: %s: "fmt, __func__, ##arg)
#define LOG_ERR(fmt, arg...)  pr_err("raspi-display: %s: "fmt, __func__, ##arg)

#define MAX_I2C_LEN 255

struct rpi_display_data {
	struct device *dev;
	struct i2c_client *client;
};

#endif
