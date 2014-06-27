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
#define DEBUG

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/edma.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <asm/mach-types.h>

#include "../codecs/sgtl5000.h"

#include "davinci-pcm.h"
#include "davinci-i2s.h"
#include "davinci-mcasp.h"

struct am335x_tx48_drvdata {
	struct clk *mclk;
	unsigned sysclk;
};

static int am335x_tx48_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct am335x_tx48_drvdata *drvdata =
		snd_soc_card_get_drvdata(soc_card);

	if (drvdata->mclk)
		return clk_prepare_enable(drvdata->mclk);

	return 0;
}

static void am335x_tx48_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct am335x_tx48_drvdata *drvdata =
		snd_soc_card_get_drvdata(soc_card);

	if (drvdata->mclk)
		clk_disable_unprepare(drvdata->mclk);
}

static int sgtl5000_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	int ret;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->codec->card;
	struct am335x_tx48_drvdata *drvdata = snd_soc_card_get_drvdata(soc_card);
	unsigned sysclk = drvdata->sysclk;
	u32 dai_format;

	if (!codec_dai) {
		dev_err(rtd->dev->parent, "No CODEC DAI\n");
		return -ENODEV;
	}
	if (!codec_dai->driver) {
		dev_err(rtd->dev->parent, "No CODEC DAI driver\n");
		return -ENODEV;
	}

	if (!cpu_dai) {
		dev_err(rtd->dev->parent, "No CPU DAI\n");
		return -ENODEV;
	}
	if (!cpu_dai->driver) {
		dev_err(rtd->dev->parent, "No CPU DAI driver\n");
		return -ENODEV;
	}

	dev_info(rtd->dev->parent, "%s: setting codec clock to %u.%03uMHz\n", __func__,
		sysclk / 1000000, sysclk / 1000 % 1000);
	/* Set SGTL5000's SYSCLK */
	ret = snd_soc_dai_set_sysclk(codec_dai, SGTL5000_SYSCLK, sysclk, 0);
	if (ret)
		return ret;

	dev_info(rtd->dev->parent, "%s: setting mcasp clock to %u.%03uMHz\n", __func__,
		sysclk / 1000000, sysclk / 1000 % 1000);

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

static struct snd_soc_ops am335x_tx48_ops = {
	.startup = am335x_tx48_startup,
	.shutdown = am335x_tx48_shutdown,
	.hw_params = sgtl5000_hw_params,
};

/*
 * The struct is used as place holder. It will be completely
 * filled with data from dt node.
 */
static struct snd_soc_dai_link am335x_tx48_dai = {
	.name = "SGTL5000",
	.stream_name = "SGTL5000",
	.codec_dai_name = "sgtl5000",
	.ops = &am335x_tx48_ops,
};

static struct snd_soc_card am335x_tx48_soc_card = {
	.owner = THIS_MODULE,
	.dai_link = &am335x_tx48_dai,
	.num_links = 1,
};

static const struct of_device_id am335x_tx48_dt_ids[] = {
	{ .compatible = "ti,am335x-tx48-audio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, am335x_tx48_dt_ids);

static int am335x_tx48_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;
	struct am335x_tx48_drvdata *drvdata;
	struct device_node *of_node;
	struct clk *mclk;

	of_node = of_parse_phandle(np, "ti,audio-codec", 0);
	if (!of_node) {
		dev_err(&pdev->dev, "codec handle missing in DT\n");
		return -EINVAL;
	}

	am335x_tx48_dai.codec_of_node = of_node;
	of_node_put(of_node);

	of_node = of_parse_phandle(np, "ti,mcasp-controller", 0);
	if (!of_node) {
		dev_err(&pdev->dev, "mcasp handle missing in DT\n");
		return -EINVAL;
	}

	am335x_tx48_dai.cpu_of_node = of_node;
	am335x_tx48_dai.platform_of_node = of_node;
	of_node_put(of_node);

	am335x_tx48_soc_card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&am335x_tx48_soc_card, "ti,model");
	if (ret)
		return ret;

	mclk = devm_clk_get(&pdev->dev, "mclk");
	if (PTR_ERR(mclk) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	} else if (IS_ERR(mclk)) {
		dev_dbg(&pdev->dev, "mclk not found\n");
		mclk = NULL;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	ret = of_property_read_u32(np, "ti,codec-clock-rate", &drvdata->sysclk);
	if (ret < 0) {
		if (!drvdata->mclk) {
			dev_err(&pdev->dev,
				"No clock or clock rate defined.\n");
			return -EINVAL;
		}
		drvdata->sysclk = clk_get_rate(drvdata->mclk);
	} else if (drvdata->mclk) {
		unsigned int requestd_rate = drvdata->sysclk;

		ret = clk_set_rate(drvdata->mclk, drvdata->sysclk);
		if (ret) {
			dev_err(&pdev->dev, "Could not set mclk rate to %u\n",
				drvdata->sysclk);
			return ret;
		}
		drvdata->sysclk = clk_get_rate(drvdata->mclk);
		if (drvdata->sysclk != requestd_rate)
			dev_warn(&pdev->dev,
				 "Could not get requested rate %u using %u\n",
				 requestd_rate, drvdata->sysclk);
	}

	snd_soc_card_set_drvdata(&am335x_tx48_soc_card, drvdata);
	ret = devm_snd_soc_register_card(&pdev->dev, &am335x_tx48_soc_card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		return ret;
	}
	dev_dbg(&pdev->dev, "Soundcard %p registered\n", &am335x_tx48_soc_card);
	return 0;
}

static int am335x_tx48_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

dev_dbg(&pdev->dev, "%s: Unregistering card %p\n", __func__, card);
	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver am335x_tx48_driver = {
	.probe		= am335x_tx48_probe,
	.remove		= am335x_tx48_remove,
	.driver		= {
		.name	= "am335x_tx48",
		.owner	= THIS_MODULE,
		.of_match_table = am335x_tx48_dt_ids,
	},
};
module_platform_driver(am335x_tx48_driver);

MODULE_AUTHOR("Lothar Waßmann");
MODULE_DESCRIPTION("Ka-Ro TX48 ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:am335x-tx48");
