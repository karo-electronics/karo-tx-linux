/*
 * ASoC driver for Ka-Ro electronics TX48 module
 * (C) Copyright 2013 Lothar Waßmann <LW@KARO-electronics.de>
 *
 * based on: davinci-evm.c
 * Author:      Vladimir Barinov, <vbarinov@embeddedalley.com>
 * Copyright:   (C) 2007 MontaVista Software, Inc., <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <asm/mach-types.h>

#include <asm/hardware/asp.h>
#include <mach/edma.h>

#include "../codecs/sgtl5000.h"

#include "davinci-pcm.h"
#include "davinci-i2s.h"
#include "davinci-mcasp.h"

static int sgtl5000_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int rate = params_rate(params);
	u32 dai_format, mclk = 27000000;
	int ret;

	if (!codec_dai) {
		dev_err(rtd->dev.parent, "No CODEC DAI\n");
		return -ENODEV;
	}
	if (!codec_dai->driver) {
		dev_err(rtd->dev.parent, "No CODEC DAI driver\n");
		return -ENODEV;
	}

	if (!cpu_dai) {
		dev_err(rtd->dev.parent, "No CPU DAI\n");
		return -ENODEV;
	}
	if (!cpu_dai->driver) {
		dev_err(rtd->dev.parent, "No CPU DAI driver\n");
		return -ENODEV;
	}

	dev_info(rtd->dev.parent, "%s: setting codec clock to %u.%03uMHz\n", __func__,
		mclk / 1000000, mclk / 1000 % 1000);
	/* Set SGTL5000's SYSCLK (provided by SAIF MCLK) */
	ret = snd_soc_dai_set_sysclk(codec_dai, SGTL5000_SYSCLK, mclk, 0);
	if (ret)
		return ret;

	dev_info(rtd->dev.parent, "%s: setting mcasp clock to %u.%03uMHz\n", __func__,
		mclk / 1000000, mclk / 1000 % 1000);

	/* set codec to master mode */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret)
		return ret;

	return 0;
}

static struct snd_soc_ops sgtl5000_ops = {
	.hw_params = sgtl5000_hw_params,
};

/* digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link am335x_tx48_dais[] = {
	{
		.name = "SGTL5000",
		.stream_name = "SGTL5000",
		.cpu_dai_name = "davinci-mcasp.1",
		.codec_dai_name = "sgtl5000",
		.codec_name = "sgtl5000.1-000a",
		.platform_name = "davinci-pcm-audio",
		.ops = &sgtl5000_ops,
	},
};

static struct snd_soc_card am335x_tx48_snd_soc_card = {
	.name = "TX48",
	.dai_link = am335x_tx48_dais,
	.num_links = ARRAY_SIZE(am335x_tx48_dais),
};

static struct platform_device *am335x_tx48_snd_device;

/* regulators for SGTL5000 */
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
static struct regulator_consumer_supply tx48_sgtl5000_vdda_supply[] = {
	REGULATOR_SUPPLY("VDDA", "1-000a"),
};

static struct regulator_init_data tx48_sgtl5000_vdda_reg_data = {
	.constraints = {
		.name = "2V5",
		.always_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(tx48_sgtl5000_vdda_supply),
	.consumer_supplies = tx48_sgtl5000_vdda_supply,
};

static struct regulator_consumer_supply tx48_sgtl5000_vddio_supply[] = {
	REGULATOR_SUPPLY("VDDIO", "1-000a"),
};

static struct regulator_init_data tx48_sgtl5000_vddio_reg_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(tx48_sgtl5000_vddio_supply),
	.consumer_supplies = tx48_sgtl5000_vddio_supply,
};

static struct regulator_consumer_supply tx48_sgtl5000_vddd_supply[] = {
	REGULATOR_SUPPLY("VDDD", "1-000a"),
};

static struct regulator_init_data tx48_sgtl5000_vddd_reg_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(tx48_sgtl5000_vddd_supply),
	.consumer_supplies = tx48_sgtl5000_vddd_supply,
};

static struct fixed_voltage_config tx48_sgtl5000_reg[] = {
	{
		.supply_name = "sgtl5000-vdda",
		.microvolts = 2500000,
		.gpio = -EINVAL,
		.enabled_at_boot = 1,
		.init_data = &tx48_sgtl5000_vdda_reg_data,
	},
	{
		.supply_name = "board-vddio",
		.microvolts = 3300000,
		.gpio = -EINVAL,
		.enabled_at_boot = 1,
		.init_data = &tx48_sgtl5000_vddio_reg_data,
	},
	{
		.supply_name = "sgtl5000-vddd",
		.microvolts = 2500000,
		.gpio = -EINVAL,
		.enabled_at_boot = 1,
		.init_data = &tx48_sgtl5000_vddd_reg_data,
	},
};

static struct platform_device tx48_sgtl5000_regulators[] = {
	{
		.name = "reg-fixed-voltage",
		.id = 1,
		.dev = {
			.platform_data = &tx48_sgtl5000_reg[0],
		},
	},
	{
		.name = "reg-fixed-voltage",
		.id = 2,
		.dev = {
			.platform_data = &tx48_sgtl5000_reg[1],
		},
	},
#if 0
	{
		.name = "reg-fixed-voltage",
		.id = 3,
		.dev = {
			.platform_data = &tx48_sgtl5000_reg[2],
		},
	},
#endif
};

static int __init tx48_regulator_register(void)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(tx48_sgtl5000_regulators); i++) {
		ret = platform_device_register(&tx48_sgtl5000_regulators[i]);
		if (ret) {
			pr_err("Failed to register SGTL5000 regulator device: %d\n",
				ret);
			while (--i >= 0) {
				platform_device_unregister(&tx48_sgtl5000_regulators[i]);
			}
			return ret;
		}
	}
	return 0;
}

static int __init am335x_tx48_init(void)
{
	int ret;

	if (!machine_is_tx48())
		return -EINVAL;

	ret = tx48_regulator_register();
	if (ret)
		return ret;

	am335x_tx48_snd_device = platform_device_alloc("soc-audio", 0);
	if (!am335x_tx48_snd_device)
		return -ENOMEM;

	platform_set_drvdata(am335x_tx48_snd_device,
			&am335x_tx48_snd_soc_card);
	ret = platform_device_add(am335x_tx48_snd_device);
	if (ret)
		platform_device_put(am335x_tx48_snd_device);

	return ret;
}

static void __exit am335x_tx48_exit(void)
{
	int i;

	platform_device_unregister(am335x_tx48_snd_device);
	for (i = 0; i < ARRAY_SIZE(tx48_sgtl5000_regulators); i++)
		platform_device_unregister(&tx48_sgtl5000_regulators[i]);
}

module_init(am335x_tx48_init);
module_exit(am335x_tx48_exit);

MODULE_AUTHOR("Lothar Waßmann");
MODULE_DESCRIPTION("Ka-Ro TX48 ASoC driver");
MODULE_LICENSE("GPL");
