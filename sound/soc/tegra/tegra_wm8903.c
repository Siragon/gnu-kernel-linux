/*
 * tegra_wm8903.c - Tegra machine ASoC driver for boards using WM8903 codec.
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010-2011 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * (c) 2009, 2010 Nvidia Graphics Pvt. Ltd.
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * Copyright (c) 2012, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <asm/mach-types.h>

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <mach/tegra_asoc_pdata.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#ifdef CONFIG_SND_SOC_FM34
#include <sound/fm34.h>
#include <sound/wm8903.h>
#endif

#include "../codecs/wm8903.h"

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#include "tegra20_das.h"
#endif

#include <linux/input.h>

#define DRV_NAME "tegra-snd-wm8903"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)
#define GPIO_HP_DET     BIT(4)
#define GPIO_LINEOUT_DET     BIT(5)

enum audio_source {
	AUDIO_SOURCE_DEFAULT = 0,
	AUDIO_SOURCE_MIC = 1,
	AUDIO_SOURCE_VOICE_UPLINK = 2,
	AUDIO_SOURCE_VOICE_DOWNLINK = 3,
	AUDIO_SOURCE_VOICE_CALL = 4,
	AUDIO_SOURCE_CAMCORDER = 5,
	AUDIO_SOURCE_VOICE_RECOGNITION = 6,
	AUDIO_SOURCE_VOICE_COMMUNICATION = 7,
	AUDIO_SOURCE_MAX = AUDIO_SOURCE_VOICE_COMMUNICATION,

	AUDIO_SOURCE_LIST_END  // must be last - used to validate audio source type
};

static struct snd_soc_codec *codec_wm8903 = NULL;

struct tegra_wm8903 {
	struct tegra_asoc_utils_data util_data;
	struct tegra_asoc_platform_data *pdata;
	struct regulator *spk_reg;
	struct regulator *dmic_reg;
	struct regulator *amic_reg;
	int gpio_requested;
#ifdef CONFIG_SWITCH
	int jack_status;
#endif
	int input_source;
	enum snd_soc_bias_level bias_level;
};

static int tegra_wm8903_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	int srate, mclk, i2s_daifmt;
	int err;
	int rate;

	srate = params_rate(params);
	switch (srate) {
	case 64000:
	case 88200:
	case 96000:
		mclk = 128 * srate;
		break;
	default:
		mclk = 256 * srate;
		break;
	}

	if(pdata->i2s_param[HIFI_CODEC].is_i2s_master) {
		/* FIXME: Codec only requires >= 3MHz if OSR==0 */
		while (mclk < 6000000)
			mclk *= 2;

		i2s_daifmt = SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS;
	} else {
		i2s_daifmt = SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBM_CFM;
	}

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	rate = clk_get_rate(machine->util_data.clk_cdev1);

	/* Use DSP mode for mono on Tegra20 */
	if ((params_channels(params) != 2) &&
		(machine_is_ventana() || machine_is_harmony() ||
		machine_is_kaen() || machine_is_aebl())) {
		i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
	} else {
		switch (pdata->i2s_param[HIFI_CODEC].i2s_mode) {
			case TEGRA_DAIFMT_I2S :
				i2s_daifmt |= SND_SOC_DAIFMT_I2S;
				break;
			case TEGRA_DAIFMT_DSP_A :
				i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
				break;
			case TEGRA_DAIFMT_DSP_B :
				i2s_daifmt |= SND_SOC_DAIFMT_DSP_B;
				break;
			case TEGRA_DAIFMT_LEFT_J :
				i2s_daifmt |= SND_SOC_DAIFMT_LEFT_J;
				break;
			case TEGRA_DAIFMT_RIGHT_J :
				i2s_daifmt |= SND_SOC_DAIFMT_RIGHT_J;
				break;
			default :
				dev_err(card->dev,
				"Can't configure i2s format\n");
				return -EINVAL;
		}
	}

	err = snd_soc_dai_set_fmt(codec_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, rate, SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC1,
					TEGRA20_DAS_DAP_ID_1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dap-dac path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_1,
					TEGRA20_DAS_DAP_SEL_DAC1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}
#endif
	return 0;
}

static int tegra_bt_sco_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	int srate, mclk, min_mclk, i2s_daifmt;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 64 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	i2s_daifmt = SND_SOC_DAIFMT_NB_NF;
	i2s_daifmt |= pdata->i2s_param[BT_SCO].is_i2s_master ?
			SND_SOC_DAIFMT_CBS_CFS : SND_SOC_DAIFMT_CBM_CFM;

	switch (pdata->i2s_param[BT_SCO].i2s_mode) {
		case TEGRA_DAIFMT_I2S :
			i2s_daifmt |= SND_SOC_DAIFMT_I2S;
			break;
		case TEGRA_DAIFMT_DSP_A :
			i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
			break;
		case TEGRA_DAIFMT_DSP_B :
			i2s_daifmt |= SND_SOC_DAIFMT_DSP_B;
			break;
		case TEGRA_DAIFMT_LEFT_J :
			i2s_daifmt |= SND_SOC_DAIFMT_LEFT_J;
			break;
		case TEGRA_DAIFMT_RIGHT_J :
			i2s_daifmt |= SND_SOC_DAIFMT_RIGHT_J;
			break;
		default :
			dev_err(card->dev, "Can't configure i2s format\n");
			return -EINVAL;
	}

	err = snd_soc_dai_set_fmt(rtd->cpu_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC2,
					TEGRA20_DAS_DAP_ID_4);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_4,
					TEGRA20_DAS_DAP_SEL_DAC2);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}
#endif
	return 0;
}

static int tegra_spdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 128 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	return 0;
}

static int tegra_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

	return 0;
}

static struct snd_soc_ops tegra_wm8903_ops = {
	.hw_params = tegra_wm8903_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_wm8903_bt_sco_ops = {
	.hw_params = tegra_bt_sco_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_spdif_ops = {
	.hw_params = tegra_spdif_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_jack tegra_wm8903_hp_jack;
static struct snd_soc_jack tegra_wm8903_mic_jack;
static struct snd_soc_jack tegra_wm8903_lineout_jack;

static struct snd_soc_jack_gpio tegra_wm8903_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
	.invert = 1,
};

static struct snd_soc_jack_gpio tegra_wm8903_lineout_jack_gpio = {
	.name = "lineout detect",
	.report = SND_JACK_LINEOUT,
	.debounce_time = 150,
	.invert = 1,
};

#ifdef CONFIG_SWITCH
/* These values are copied from Android WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
	BIT_USB_HEADSET_ANLG = (1 << 2),
};

static struct switch_dev tegra_wm8903_headset_switch = {
	.name = "h2w",
};

static struct switch_dev tegra_wm8903_lineout_switch = {
	.name = "usb_audio",
};

bool headset_mic_detect(void)
{
	bool hs_mic = false;
	int i = 0;
	int det_times = 10;

	/*headset microphone detect*/

	snd_soc_update_bits(codec_wm8903, WM8903_MIC_BIAS_CONTROL_0,
		WM8903_MICDET_ENA, WM8903_MICDET_ENA);

	for(i = 0; i < det_times; i++) {
		snd_soc_update_bits(codec_wm8903, WM8903_INTERRUPT_POLARITY_1,
			WM8903_MICSHRT_INV_MASK,
			WM8903_MICSHRT_INV);
		hs_mic = (snd_soc_test_bits(codec_wm8903, WM8903_INTERRUPT_STATUS_1,
			WM8903_MICSHRT_INV_MASK, WM8903_MICSHRT_INV) == 0);

		msleep(5);

		snd_soc_update_bits(codec_wm8903, WM8903_INTERRUPT_POLARITY_1,
			WM8903_MICSHRT_INV_MASK,
			0x0);
		hs_mic = (snd_soc_test_bits(codec_wm8903, WM8903_INTERRUPT_STATUS_1,
			WM8903_MICSHRT_INV_MASK, WM8903_MICSHRT_INV) == 1);

		msleep(5);
	}

	snd_soc_update_bits(codec_wm8903, WM8903_MIC_BIAS_CONTROL_0,
		WM8903_MICDET_ENA, 0);

	return hs_mic;
}

static int tegra_wm8903_jack_notifier(struct notifier_block *self,
			      unsigned long action, void *dev)
{
	struct snd_soc_jack *jack = dev;
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	enum headset_state state = BIT_NO_HEADSET;
	struct input_dev *jack_input_dev = jack->jack->input_dev;
	int amic_act = 0, hp_act = 0, lineout_act = 0;

	if (machine_is_chagall()) {
		if (jack == &tegra_wm8903_hp_jack || jack == &tegra_wm8903_lineout_jack) {
			machine->jack_status &= ~(SND_JACK_HEADPHONE | SND_JACK_MICROPHONE |
				SND_JACK_LINEOUT);

			lineout_act = gpio_get_value(tegra_wm8903_lineout_jack_gpio.gpio) ^
				tegra_wm8903_lineout_jack_gpio.invert;

			if (lineout_act) {
				machine->jack_status |= SND_JACK_LINEOUT;
				printk(KERN_INFO "###### %s: %s Plug-in\n", __FUNCTION__,
					"Lineout");
				goto JACK_DETECT_FINISH;
			}

			hp_act = gpio_get_value(tegra_wm8903_hp_jack_gpio.gpio) ^
				tegra_wm8903_hp_jack_gpio.invert;

			if (hp_act) {
				machine->jack_status |= SND_JACK_HEADPHONE;

				/* enable MICBIAS */
				snd_soc_dapm_force_enable_pin(&codec_wm8903->dapm, "Mic Bias");
				snd_soc_dapm_sync(&codec_wm8903->dapm);
				if(machine->amic_reg)
					regulator_enable(machine->amic_reg);

				msleep(500);

				amic_act = headset_mic_detect();
				printk(KERN_INFO "###### %s %s Plug-in\n", __FUNCTION__,
					amic_act ? "Headset" : "Headphone");
				if (amic_act) {
					machine->jack_status |= SND_JACK_MICROPHONE;
				} else {
					/* disable MICBIAS */
					if (machine->amic_reg)
						regulator_disable(machine->amic_reg);
					snd_soc_dapm_disable_pin(&codec_wm8903->dapm, "Mic Bias");
					snd_soc_dapm_sync(&codec_wm8903->dapm);
				}

				/* enable hook key detection */
				wm8903_mic_detect(codec_wm8903, &tegra_wm8903_mic_jack, 0, 1);
			} else {
				/* disable hook key detection */
				wm8903_mic_detect(codec_wm8903, &tegra_wm8903_mic_jack, 0, 0);

				/* disable MICBIAS */
				if (machine->amic_reg) {
					if (regulator_is_enabled(machine->amic_reg)) {
						regulator_disable(machine->amic_reg);
					}
				}
				snd_soc_dapm_disable_pin(&codec_wm8903->dapm, "Mic Bias");
				snd_soc_dapm_sync(&codec_wm8903->dapm);

				printk(KERN_INFO "###### %s %s Remove\n", __FUNCTION__,
					"Headphone/Headset");
			}

JACK_DETECT_FINISH:

			if(lineout_act && snd_soc_dapm_get_pin_status(dapm, "Headphone Jack")) {
				snd_soc_dapm_disable_pin(dapm, "Headphone Jack");
				snd_soc_dapm_enable_pin(dapm, "Line Out Jack");
				snd_soc_dapm_sync(dapm);
			} else if(hp_act && snd_soc_dapm_get_pin_status(dapm, "Line Out Jack")) {
				snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
				snd_soc_dapm_disable_pin(dapm, "Line Out Jack");
				snd_soc_dapm_sync(dapm);
			}
		} else if (jack == &tegra_wm8903_mic_jack) {
			msleep(150);
			/* generate hook key input event */
			input_event(jack_input_dev, EV_KEY, KEY_MEDIA, jack->status);
			input_sync(jack_input_dev);
			goto HOOK_KEY_DETECT_FINISH;
		}

	} else {
		if (jack == &tegra_wm8903_hp_jack) {
			machine->jack_status &= ~SND_JACK_HEADPHONE;
			machine->jack_status |= (action & SND_JACK_HEADPHONE);
		} else {
			machine->jack_status &= ~SND_JACK_MICROPHONE;
			machine->jack_status |= (action & SND_JACK_MICROPHONE);
		}
	}

	switch (machine->jack_status) {
	case SND_JACK_HEADPHONE:
		state = BIT_HEADSET_NO_MIC;
		break;
	case SND_JACK_HEADSET:
		state = BIT_HEADSET;
		break;
	case SND_JACK_LINEOUT:
		state = BIT_USB_HEADSET_ANLG;
		break;
	case SND_JACK_MICROPHONE:
		/* mic: would not report */
	default:
		state = BIT_NO_HEADSET;
	}

	switch_set_state(&tegra_wm8903_headset_switch,
		state & (BIT_HEADSET | BIT_HEADSET_NO_MIC));
	switch_set_state(&tegra_wm8903_lineout_switch,
		state & BIT_USB_HEADSET_ANLG);

HOOK_KEY_DETECT_FINISH:

	return NOTIFY_OK;
}

static struct notifier_block tegra_wm8903_jack_detect_nb = {
	.notifier_call = tegra_wm8903_jack_notifier,
};
#else
static struct snd_soc_jack_pin tegra_wm8903_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static struct snd_soc_jack_pin tegra_wm8903_mic_jack_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
};

static struct snd_soc_jack_pin tegra_wm8903_lineout_jack_pins[] = {
	{
		.pin = "Line Out Jack",
		.mask = SND_JACK_LINEOUT,
	},
};
#endif

static int tegra_wm8903_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (machine->spk_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			regulator_enable(machine->spk_reg);
		else
			regulator_disable(machine->spk_reg);
	}

	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_spkr_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_wm8903_event_hp(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_HP_MUTE))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_hp_mute,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_wm8903_event_int_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
#ifdef CONFIG_SND_SOC_FM34
	struct wm8903_platform_data *pdata_codec = dev_get_platdata(codec_wm8903->dev);
	u16 val;
#endif

#if 0
	if (machine->dmic_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			regulator_enable(machine->dmic_reg);
		else
			regulator_disable(machine->dmic_reg);
	}
#endif

#ifdef CONFIG_SND_SOC_FM34
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		if(machine->input_source != AUDIO_SOURCE_VOICE_RECOGNITION) {
			if(set_fM34_echo()) {
				val = ((0xC0<<WM8903_ADCL_VOL_SHIFT) | WM8903_ADCVU);
				snd_soc_write(codec_wm8903, WM8903_ADC_DIGITAL_VOLUME_RIGHT, val);
			}
		}
	} else {
		set_fM34_bypass(0);
		val = pdata_codec->adc_digital_volume;
		val |= WM8903_ADCVU;
		snd_soc_write(codec_wm8903, WM8903_ADC_DIGITAL_VOLUME_RIGHT, val);
	}
#endif

	if (!(machine->gpio_requested & GPIO_INT_MIC_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_int_mic_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_wm8903_event_ext_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (machine->amic_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			regulator_enable(machine->amic_reg);
		else
			regulator_disable(machine->amic_reg);
	}

	if (!(machine->gpio_requested & GPIO_EXT_MIC_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_ext_mic_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static const struct snd_soc_dapm_widget cardhu_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_wm8903_event_int_spk),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_wm8903_event_hp),
	SND_SOC_DAPM_LINE("LineOut", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", tegra_wm8903_event_ext_mic),
	SND_SOC_DAPM_MIC("Int Mic", tegra_wm8903_event_int_mic),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

static const struct snd_soc_dapm_widget tegra_wm8903_default_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_wm8903_event_int_spk),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_wm8903_event_hp),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static const struct snd_soc_dapm_route harmony_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1L", NULL, "Mic Bias"},
};

static const struct snd_soc_dapm_route cardhu_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"LineOut", NULL, "LINEOUTL"},
	{"LineOut", NULL, "LINEOUTR"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1L", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Int Mic"},
	{"IN1L", NULL, "Mic Bias"},
	{"IN1R", NULL, "Mic Bias"},
	{"IN3L", NULL, "Line In"},
	{"IN3R", NULL, "Line In"},
};

static const struct snd_soc_dapm_route seaboard_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1R", NULL, "Mic Bias"},
};

static const struct snd_soc_dapm_route kaen_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN2R", NULL, "Mic Bias"},
};

static const struct snd_soc_dapm_route aebl_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "LINEOUTR"},
	{"Int Spk", NULL, "LINEOUTL"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1R", NULL, "Mic Bias"},
};

static const struct snd_kcontrol_new cardhu_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("LineOut"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Line In"),
};

static const struct snd_soc_dapm_widget chagall_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_wm8903_event_int_spk),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_wm8903_event_hp),
	SND_SOC_DAPM_LINE("Line Out Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", tegra_wm8903_event_ext_mic),
	SND_SOC_DAPM_MIC("Int Mic", tegra_wm8903_event_int_mic),
};

static const struct snd_soc_dapm_route chagall_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Line Out Jack", NULL, "LINEOUTL"},
	{"Line Out Jack", NULL, "LINEOUTR"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1R", NULL, "Mic Bias"},
	{"DMICDAT", NULL, "Int Mic"},
};

static const struct snd_kcontrol_new chagall_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Line Out Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
};

static const struct snd_kcontrol_new tegra_wm8903_default_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
};

static int tegra_wm8903_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	int ret;

	codec_wm8903 = codec;

	machine->bias_level = SND_SOC_BIAS_STANDBY;

	if (gpio_is_valid(pdata->gpio_spkr_en)) {
		ret = gpio_request(pdata->gpio_spkr_en, "spkr_en");
		if (ret) {
			dev_err(card->dev, "cannot get spkr_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_SPKR_EN;

		gpio_direction_output(pdata->gpio_spkr_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_hp_mute)) {
		ret = gpio_request(pdata->gpio_hp_mute, "hp_mute");
		if (ret) {
			dev_err(card->dev, "cannot get hp_mute gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_HP_MUTE;

		gpio_direction_output(pdata->gpio_hp_mute, 1);
	}

	if (gpio_is_valid(pdata->gpio_int_mic_en)) {
		ret = gpio_request(pdata->gpio_int_mic_en, "int_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get int_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_INT_MIC_EN;

		/* Disable int mic; enable signal is active-high */
		gpio_direction_output(pdata->gpio_int_mic_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_ext_mic_en)) {
		ret = gpio_request(pdata->gpio_ext_mic_en, "ext_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get ext_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_EXT_MIC_EN;

		/* Enable ext mic; enable signal is active-low */
		gpio_direction_output(pdata->gpio_ext_mic_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_lineout_det)) {
		tegra_wm8903_lineout_jack_gpio.gpio = pdata->gpio_lineout_det;
		snd_soc_jack_new(codec, "Line Out Jack", SND_JACK_LINEOUT,
			&tegra_wm8903_lineout_jack);
#ifndef CONFIG_SWITCH
		snd_soc_jack_add_pins(&tegra_wm8903_lineout_jack,
		ARRAY_SIZE(tegra_wm8903_lineout_jack_pins),
			tegra_wm8903_lineout_jack_pins);
#else
		snd_soc_jack_notifier_register(&tegra_wm8903_lineout_jack,
			&tegra_wm8903_jack_detect_nb);
#endif
		snd_soc_jack_add_gpios(&tegra_wm8903_lineout_jack,
			1,
			&tegra_wm8903_lineout_jack_gpio);
		machine->gpio_requested |= GPIO_LINEOUT_DET;
	}

	if (gpio_is_valid(pdata->gpio_hp_det)) {
		tegra_wm8903_hp_jack_gpio.gpio = pdata->gpio_hp_det;
		snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE,
				&tegra_wm8903_hp_jack);
#ifndef CONFIG_SWITCH
		snd_soc_jack_add_pins(&tegra_wm8903_hp_jack,
					ARRAY_SIZE(tegra_wm8903_hp_jack_pins),
					tegra_wm8903_hp_jack_pins);
#else
		snd_soc_jack_notifier_register(&tegra_wm8903_hp_jack,
					&tegra_wm8903_jack_detect_nb);
#endif
		snd_soc_jack_add_gpios(&tegra_wm8903_hp_jack,
					1,
					&tegra_wm8903_hp_jack_gpio);
		machine->gpio_requested |= GPIO_HP_DET;
	}

	snd_soc_jack_new(codec, "Mic Jack", SND_JACK_MICROPHONE,
			 &tegra_wm8903_mic_jack);
#ifndef CONFIG_SWITCH
	snd_soc_jack_add_pins(&tegra_wm8903_mic_jack,
			      ARRAY_SIZE(tegra_wm8903_mic_jack_pins),
			      tegra_wm8903_mic_jack_pins);
#else
	snd_soc_jack_notifier_register(&tegra_wm8903_mic_jack,
				&tegra_wm8903_jack_detect_nb);
#endif

	/* setup hook key input event */
	set_bit(EV_KEY, tegra_wm8903_mic_jack.jack->input_dev->evbit);
	set_bit(KEY_MEDIA, tegra_wm8903_mic_jack.jack->input_dev->keybit);

	if (!machine_is_chagall()) {
		wm8903_mic_detect(codec, &tegra_wm8903_mic_jack, SND_JACK_MICROPHONE,
					  (machine_is_cardhu()) ? SND_JACK_MICROPHONE : 0);
	}

	ret = tegra_asoc_utils_register_ctls(&machine->util_data);
	if (ret < 0)
		return ret;

	snd_soc_dapm_force_enable_pin(dapm, "Mic Bias");

	/* FIXME: Calculate automatically based on DAPM routes? */
	if (!machine_is_harmony() && !machine_is_ventana() &&
	    !machine_is_cardhu() && !machine_is_chagall())
		snd_soc_dapm_nc_pin(dapm, "IN1L");
	if (!machine_is_seaboard() && !machine_is_aebl() &&
	    !machine_is_cardhu() && !machine_is_chagall())
		snd_soc_dapm_nc_pin(dapm, "IN1R");
	if (!machine_is_chagall())
		snd_soc_dapm_nc_pin(dapm, "IN2L");
	if (!machine_is_kaen() && !machine_is_chagall())
		snd_soc_dapm_nc_pin(dapm, "IN2R");

	if (!machine_is_chagall()) {
		snd_soc_dapm_nc_pin(dapm, "IN3L");
		snd_soc_dapm_nc_pin(dapm, "IN3R");
	}

	if (machine_is_aebl()) {
		snd_soc_dapm_nc_pin(dapm, "LON");
		snd_soc_dapm_nc_pin(dapm, "RON");
		snd_soc_dapm_nc_pin(dapm, "ROP");
		snd_soc_dapm_nc_pin(dapm, "LOP");
	} else if (!machine_is_chagall()) {
		snd_soc_dapm_nc_pin(dapm, "LINEOUTR");
		snd_soc_dapm_nc_pin(dapm, "LINEOUTL");
	}

	if (machine_is_chagall()) {
		snd_soc_dapm_disable_pin(dapm, "Int Spk");
		snd_soc_dapm_disable_pin(dapm, "Headphone Jack");
		snd_soc_dapm_disable_pin(dapm, "Line Out Jack");
		snd_soc_dapm_disable_pin(dapm, "Mic Jack");
		snd_soc_dapm_disable_pin(dapm, "Int Mic");
	}

	snd_soc_dapm_sync(dapm);

	return 0;
}

#ifdef WM8903_SET_BIAS_LEVEL
static int tegra30_soc_set_bias_level(struct snd_soc_card *card,
					enum snd_soc_bias_level level)
{
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	if (machine->bias_level == SND_SOC_BIAS_OFF &&
		level != SND_SOC_BIAS_OFF)
		tegra_asoc_utils_clk_enable(&machine->util_data);

	return 0;
}

static int tegra30_soc_set_bias_level_post(struct snd_soc_card *card,
					enum snd_soc_bias_level level)
{
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	if (machine->bias_level != SND_SOC_BIAS_OFF &&
		level == SND_SOC_BIAS_OFF)
		tegra_asoc_utils_clk_disable(&machine->util_data);

	machine->bias_level = level;

	return 0 ;
}
#endif

static struct snd_soc_dai_link tegra_wm8903_dai[] = {
	{
		.name = "WM8903",
		.stream_name = "WM8903 PCM",
		.codec_name = "wm8903.0-001a",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-i2s.0",
		.codec_dai_name = "wm8903-hifi",
		.init = tegra_wm8903_init,
		.ops = &tegra_wm8903_ops,
	},
	{
		.name = "SPDIF",
		.stream_name = "SPDIF PCM",
		.codec_name = "spdif-dit.0",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-spdif",
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_spdif_ops,
	},
	{
		.name = "BT-SCO",
		.stream_name = "BT SCO PCM",
		.codec_name = "spdif-dit.1",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-i2s.1",
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_wm8903_bt_sco_ops,
	},
};

static int tegra_wm8903_suspend_post(struct snd_soc_card *card)
{
	struct snd_soc_jack_gpio *gpio = &tegra_wm8903_hp_jack_gpio;

	if (gpio_is_valid(gpio->gpio))
		disable_irq(gpio_to_irq(gpio->gpio));

	gpio = &tegra_wm8903_lineout_jack_gpio;

	if (gpio_is_valid(gpio->gpio))
		disable_irq(gpio_to_irq(gpio->gpio));

	return 0;
}

static int tegra_wm8903_resume_pre(struct snd_soc_card *card)
{
	int val;
	struct snd_soc_jack_gpio *gpio = &tegra_wm8903_hp_jack_gpio;

	if (gpio_is_valid(gpio->gpio)) {
		val = gpio_get_value(gpio->gpio);
		val = gpio->invert ? !val : val;
		val = val ? gpio->report : 0;
		snd_soc_jack_report(gpio->jack, val, gpio->report);
		enable_irq(gpio_to_irq(gpio->gpio));
	}

	gpio = &tegra_wm8903_lineout_jack_gpio;

	if (gpio_is_valid(gpio->gpio)) {
		val = gpio_get_value(gpio->gpio);
		val = gpio->invert ? !val : val;
		val = val ? gpio->report : 0;
		snd_soc_jack_report(gpio->jack, val, gpio->report);
		enable_irq(gpio_to_irq(gpio->gpio));
	}

	return 0;
}

static struct snd_soc_card snd_soc_tegra_wm8903 = {
	.name = "tegra-wm8903",
	.dai_link = tegra_wm8903_dai,
	.num_links = ARRAY_SIZE(tegra_wm8903_dai),
	.suspend_post = tegra_wm8903_suspend_post,
	.resume_pre = tegra_wm8903_resume_pre,
	//.set_bias_level = tegra30_soc_set_bias_level,
	//.set_bias_level_post = tegra30_soc_set_bias_level_post,
};

ssize_t jack_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8903;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	sprintf(buf, "0x%X\n", machine->jack_status);
	return strlen(buf);
}
DEVICE_ATTR(jack_status, S_IRUSR | S_IRGRP | S_IROTH, jack_status_show, NULL);

ssize_t input_source_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8903;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	if(len != 2){
		return -EINVAL;
	}

	machine->input_source = *buf-'0';
	return len;
}

ssize_t input_source_show(struct device *dev, struct device_attribute *attr, char *buf)
{
struct snd_soc_card *card = &snd_soc_tegra_wm8903;
struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	sprintf(buf, "%d\n", machine->input_source);
	return strlen(buf);
}
DEVICE_ATTR(input_source, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, input_source_show, input_source_store);

static __devinit int tegra_wm8903_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8903;
	struct tegra_wm8903 *machine;
	struct tegra_asoc_platform_data *pdata;
	int ret;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	machine = kzalloc(sizeof(struct tegra_wm8903), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_wm8903 struct\n");
		return -ENOMEM;
	}

	machine->pdata = pdata;

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev, card);
	if (ret)
		goto err_free_machine;

	if (machine_is_cardhu() || machine_is_ventana() || machine_is_chagall()) {
		machine->spk_reg = regulator_get(&pdev->dev, "vdd_spk_amp");
		if (IS_ERR(machine->spk_reg)) {
			dev_info(&pdev->dev, "No speaker regulator found\n");
			machine->spk_reg = 0;
		}
	}

	if (machine_is_ventana() || machine_is_chagall()) {
		machine->dmic_reg = regulator_get(&pdev->dev, "vdd_dmic");
		if (IS_ERR(machine->dmic_reg)) {
			dev_info(&pdev->dev, "No digital mic"
						" regulator found\n");
			machine->dmic_reg = 0;
		}
	}

	if (machine_is_chagall()) {
		regulator_enable(machine->dmic_reg);
	}

	if (machine_is_chagall()) {
		machine->amic_reg = regulator_get(&pdev->dev, "vdd_amic");
		if (IS_ERR(machine->amic_reg)) {
			dev_info(&pdev->dev, "No analog mic regulator found\n");
			machine->amic_reg = 0;
		}
	}

	if (machine_is_cardhu()) {
		tegra_wm8903_dai[0].codec_name = "wm8903.0-001a",
		tegra_wm8903_dai[0].cpu_dai_name = "tegra30-i2s.1";

		tegra_wm8903_dai[1].cpu_dai_name = "tegra30-spdif";

		tegra_wm8903_dai[2].cpu_dai_name = "tegra30-i2s.3";
	} else if (machine_is_chagall()) {
		tegra_wm8903_dai[0].codec_name = "wm8903.0-001a",
		tegra_wm8903_dai[0].cpu_dai_name = "tegra30-i2s.1";

		tegra_wm8903_dai[1].cpu_dai_name = "tegra30-spdif";

		tegra_wm8903_dai[2].cpu_dai_name = "tegra30-i2s.3";
	}

#ifdef CONFIG_SWITCH
	/* Addd h2w swith class support */
	ret = switch_dev_register(&tegra_wm8903_headset_switch);
	if (ret < 0)
		goto err_fini_utils;

	/* Addd usb_audio swith class support */
	ret = switch_dev_register(&tegra_wm8903_lineout_switch);
	if (ret < 0)
		goto err_fini_utils;
#endif

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	if (machine_is_cardhu() || machine_is_ventana()) {
		card->controls = cardhu_controls;
		card->num_controls = ARRAY_SIZE(cardhu_controls);

		card->dapm_widgets = cardhu_dapm_widgets;
		card->num_dapm_widgets = ARRAY_SIZE(cardhu_dapm_widgets);
	} else if (machine_is_chagall()) {
		card->controls = chagall_controls;
		card->num_controls = ARRAY_SIZE(chagall_controls);

		card->dapm_widgets = chagall_dapm_widgets;
		card->num_dapm_widgets = ARRAY_SIZE(chagall_dapm_widgets);
	} else {
		card->controls = tegra_wm8903_default_controls;
		card->num_controls = ARRAY_SIZE(tegra_wm8903_default_controls);

		card->dapm_widgets = tegra_wm8903_default_dapm_widgets;
		card->num_dapm_widgets = ARRAY_SIZE(tegra_wm8903_default_dapm_widgets);
	}

	if (machine_is_harmony()) {
		card->dapm_routes = harmony_audio_map;
		card->num_dapm_routes = ARRAY_SIZE(harmony_audio_map);
	} else if (machine_is_ventana() || machine_is_cardhu()) {
		card->dapm_routes = cardhu_audio_map;
		card->num_dapm_routes = ARRAY_SIZE(cardhu_audio_map);
	} else if (machine_is_seaboard()) {
		card->dapm_routes = seaboard_audio_map;
		card->num_dapm_routes = ARRAY_SIZE(seaboard_audio_map);
	} else if (machine_is_kaen()) {
		card->dapm_routes = kaen_audio_map;
		card->num_dapm_routes = ARRAY_SIZE(kaen_audio_map);
	} else if (machine_is_chagall()) {
		card->dapm_routes = chagall_audio_map;
		card->num_dapm_routes = ARRAY_SIZE(chagall_audio_map);
	} else {
		card->dapm_routes = aebl_audio_map;
		card->num_dapm_routes = ARRAY_SIZE(aebl_audio_map);
	}

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_unregister_switch;
	}

	if (!card->instantiated) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_unregister_card;
	}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	ret = tegra_asoc_utils_set_parent(&machine->util_data,
				pdata->i2s_param[HIFI_CODEC].is_i2s_master);
	if (ret) {
		dev_err(&pdev->dev, "tegra_asoc_utils_set_parent failed (%d)\n",
			ret);
		goto err_unregister_card;
	}
#endif

	ret = device_create_file(card->dev, &dev_attr_jack_status);
	if (ret) {
		dev_err(card->dev, "%s: add_sysfs_entry jack_status failed\n", __FUNCTION__);
	}

	ret = device_create_file(card->dev, &dev_attr_input_source);
	if (ret) {
		dev_err(card->dev, "%s: add_sysfs_entry input_source failed\n", __FUNCTION__);
	}

	return 0;

err_unregister_card:
	snd_soc_unregister_card(card);
err_unregister_switch:
#ifdef CONFIG_SWITCH
	switch_dev_unregister(&tegra_wm8903_headset_switch);
	switch_dev_unregister(&tegra_wm8903_lineout_switch);
err_fini_utils:
#endif
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
	kfree(machine);
	return ret;
}

static int __devexit tegra_wm8903_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	device_remove_file(card->dev, &dev_attr_jack_status);

	if (machine->gpio_requested & GPIO_LINEOUT_DET)
		snd_soc_jack_free_gpios(&tegra_wm8903_lineout_jack,
					1,
					&tegra_wm8903_lineout_jack_gpio);
	if (machine->gpio_requested & GPIO_HP_DET)
		snd_soc_jack_free_gpios(&tegra_wm8903_hp_jack,
					1,
					&tegra_wm8903_hp_jack_gpio);
	if (machine->gpio_requested & GPIO_EXT_MIC_EN)
		gpio_free(pdata->gpio_ext_mic_en);
	if (machine->gpio_requested & GPIO_INT_MIC_EN)
		gpio_free(pdata->gpio_int_mic_en);
	if (machine->gpio_requested & GPIO_HP_MUTE)
		gpio_free(pdata->gpio_hp_mute);
	if (machine->gpio_requested & GPIO_SPKR_EN)
		gpio_free(pdata->gpio_spkr_en);
	machine->gpio_requested = 0;

	if (machine->spk_reg)
		regulator_put(machine->spk_reg);
	if (machine->dmic_reg)
		regulator_put(machine->dmic_reg);

	snd_soc_unregister_card(card);

	tegra_asoc_utils_fini(&machine->util_data);

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&tegra_wm8903_lineout_switch);
	switch_dev_unregister(&tegra_wm8903_headset_switch);
#endif
	kfree(machine);

	return 0;
}

static void tegra_wm8903_driver_shutdown(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if(gpio_get_value_cansleep(pdata->gpio_spkr_en)) {
		gpio_set_value_cansleep(pdata->gpio_spkr_en, 0);
	}
	snd_soc_write(codec_wm8903, WM8903_POWER_MANAGEMENT_5, 0x0);
	snd_soc_write(codec_wm8903, WM8903_ANALOGUE_HP_0, 0x0);
	snd_soc_write(codec_wm8903, WM8903_ANALOGUE_LINEOUT_0, 0x0);
}

static struct platform_driver tegra_wm8903_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_wm8903_driver_probe,
	.remove = __devexit_p(tegra_wm8903_driver_remove),
	.shutdown = tegra_wm8903_driver_shutdown,
};

static int __init tegra_wm8903_modinit(void)
{
	return platform_driver_register(&tegra_wm8903_driver);
}
module_init(tegra_wm8903_modinit);

static void __exit tegra_wm8903_modexit(void)
{
	platform_driver_unregister(&tegra_wm8903_driver);
}
module_exit(tegra_wm8903_modexit);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra+WM8903 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
