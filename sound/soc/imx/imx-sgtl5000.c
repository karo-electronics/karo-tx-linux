/*
 * Copyright 2012 Lothar Waßmann <LW@KARO-electronics.de>
 *
 * based on mxs-sgtl5000.c (C) 2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>

#include "../codecs/sgtl5000.h"
#include "imx-ssi.h"
#include "imx-audmux.h"

static u32 sysclk;
module_param(sysclk, uint, S_IRUGO | S_IWUSR);

static int imx_sgtl5000_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	int ret;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int channels = params_channels(params);
	u32 dai_format;

	ret = snd_soc_dai_set_sysclk(codec_dai, SGTL5000_SYSCLK, sysclk,
				SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(cpu_dai->dev, "Failed to set CODEC sysclk to %u\n", sysclk);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, sysclk,
				SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(cpu_dai->dev, "Failed to set CPU sysclk to %u\n", sysclk);
		return ret;
	}

	switch (channels) {
	case 2:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xffffffc, 0xffffffc, 2, 0);
		break;
	case 1:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xffffffe, 0xffffffe, 1, 0);
		break;
	default:
		return -EINVAL;
	}

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret) {
		dev_err(cpu_dai->dev, "Failed to set CODEC dai_format to %08x\n",
			dai_format);
		return ret;
	}

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret) {
		dev_err(cpu_dai->dev, "Failed to set CPU dai_format to %08x\n",
			dai_format);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops imx_sgtl5000_hifi_ops = {
	.hw_params = imx_sgtl5000_hw_params,
};

static struct snd_soc_dai_link imx_sgtl5000_dai[] = {
	{
		.name		= "sgtl5000",
		.stream_name	= "SGTL5000",
		.codec_dai_name	= "sgtl5000",
		.codec_name	= "sgtl5000.0-000a",
		.cpu_dai_name	= "imx-ssi.0",
		.platform_name	= "imx-pcm-audio",
		.ops		= &imx_sgtl5000_hifi_ops,
	},
};

static struct snd_soc_card imx_sgtl5000 = {
	.name		= "imx-sgtl5000",
	.owner		= THIS_MODULE,
	.dai_link	= imx_sgtl5000_dai,
	.num_links	= ARRAY_SIZE(imx_sgtl5000_dai),
};

static const struct of_device_id imx_sgtl5000_dt_ids[] = {
	{ .compatible = "fsl,imx-sgtl5000", },
	{ /* sentinel */ }
};

static int __devinit imx_sgtl5000_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &imx_sgtl5000;
	struct device_node *np = pdev->dev.of_node;
	const unsigned long *prop;
	int ret;
	unsigned int int_port, ext_port;

	if (!np)
		return -ENODEV;

	prop = of_get_property(np, "ssi-port", NULL);
	if (!prop) {
		dev_err(&pdev->dev, "No ssi-port property found\n");
		return -ENODEV;
	}
	int_port = be32_to_cpu(*prop);
	if (int_port < 1 || int_port > 7) {
		dev_err(&pdev->dev, "Invalid SSI port: %d\n", int_port);
		return -EINVAL;
	}

	prop = of_get_property(np, "audmux-port", &ext_port);
	if (!prop) {
		dev_err(&pdev->dev, "No audmux-port property found\n");
		return -ENODEV;
	}
	ext_port = be32_to_cpu(*prop);
	if (ext_port < 1 || ext_port > 7) {
		dev_err(&pdev->dev, "Invalid AUDMUX port: %d\n", ext_port);
		return -EINVAL;
	}
	ext_port--;
	int_port--;

	prop = of_get_property(np, "sysclk", NULL);
	if (prop)
		sysclk = be32_to_cpu(*prop);

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);

	imx_audmux_v2_configure_port(int_port,
				IMX_AUDMUX_V2_PTCR_SYN |
				IMX_AUDMUX_V2_PTCR_TFSDIR |
				IMX_AUDMUX_V2_PTCR_RFSDIR |
				IMX_AUDMUX_V2_PTCR_TFSEL(ext_port) |
				IMX_AUDMUX_V2_PTCR_TCLKDIR |
				IMX_AUDMUX_V2_PTCR_TCSEL(ext_port),
				IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
	imx_audmux_v2_configure_port(ext_port,
				IMX_AUDMUX_V2_PTCR_SYN,
				IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		return ret;
	}
	dev_info(&pdev->dev, "IMX SGTL5000 registered routing SSI port %u -> AUDMUX port %u\n",
		int_port, ext_port);

	return 0;
}

static int __devexit imx_sgtl5000_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver imx_sgtl5000_audio_driver = {
	.driver = {
		.name = "imx-sgtl5000",
		.owner = THIS_MODULE,
		.of_match_table = imx_sgtl5000_dt_ids,
	},
	.probe = imx_sgtl5000_probe,
	.remove = __devexit_p(imx_sgtl5000_remove),
};

module_platform_driver(imx_sgtl5000_audio_driver);

MODULE_AUTHOR("Lothar Waßmann <LW@KARO-electronics.de>");
MODULE_DESCRIPTION("i.MX ALSA SoC Machine driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-sgtl5000");
