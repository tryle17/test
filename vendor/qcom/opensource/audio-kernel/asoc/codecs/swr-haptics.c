// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <soc/soundwire.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/soc/qcom/battery_charger.h>

#ifdef OPLUS_ARCH_EXTENDS
/* Add for decouple haptics driver from snd card */
static int  haptics_probe_retry = 0;
#endif /* OPLUS_ARCH_EXTENDS */

#define HAPTICS_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
		SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 |\
		SNDRV_PCM_RATE_384000)

#define HAPTICS_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
		SNDRV_PCM_FMTBIT_S24_LE |\
		SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

/* SWR register definition */
#define SWR_HAP_ACCESS_BASE		0x3000
#define FIFO_WR_READY_REG		(SWR_HAP_ACCESS_BASE + 0x8)
#define NUM_PAT_SMPL_REG		(SWR_HAP_ACCESS_BASE + 0x9)
#define SWR_WR_ACCESS_REG		(SWR_HAP_ACCESS_BASE + 0xa)
#define CAL_TLRA_STATUS_MSB_REG		(SWR_HAP_ACCESS_BASE + 0xb)
#define CAL_TLRA_STATUS_LSB_REG		(SWR_HAP_ACCESS_BASE + 0xc)
#define AUTO_RES_CAL_DONE_REG		(SWR_HAP_ACCESS_BASE + 0xd)
#define SWR_READ_DATA_REG		(SWR_HAP_ACCESS_BASE + 0x80)
#define SWR_PLAY_REG			(SWR_HAP_ACCESS_BASE + 0x81)
#define SWR_VMAX_REG			(SWR_HAP_ACCESS_BASE + 0x82)
#define SWR_VISENSE_AFE_GAIN_REG	(SWR_HAP_ACCESS_BASE + 0x83)
#define SWR_PLAY_BIT			BIT(7)
#define SWR_BRAKE_EN_BIT		BIT(3)
#define SWR_PLAY_SRC_MASK		GENMASK(2, 0)
#define SWR_PLAY_SRC_VAL_SWR		4
#define VISENSE_AFE_GAIN_VALUE		0x14

#define SWR_HAP_REG_MAX			(SWR_HAP_ACCESS_BASE + 0xff)

#define MAX_HAPTICS_VMAX_MV     10000
#define MAX_CL_HAPTICS_VMAX_MV  9500
#define VMAX_STEP_MV            50

#ifdef OPLUS_FEATURE_RINGTONE_HAPTIC
/* Add for control VMAX */
#define OPLUS_SWR_HAP_VMAX		200
#endif /* OPLUS_FEATURE_RINGTONE_HAPTIC */

enum pmic_type {
	PM8350B = 1,
	PM8550B = 2,
	PMIH010X = 3,
};

enum {
	HAP_SSR_RECOVERY = BIT(0),
};

static struct reg_default swr_hap_reg_defaults[] = {
	{FIFO_WR_READY_REG, 1},
	{NUM_PAT_SMPL_REG, 8},
	{SWR_WR_ACCESS_REG, 1},
	{CAL_TLRA_STATUS_MSB_REG, 0},
	{CAL_TLRA_STATUS_LSB_REG, 0},
	{AUTO_RES_CAL_DONE_REG, 0},
	{SWR_READ_DATA_REG, 0},
	{SWR_PLAY_REG, 4},
	{SWR_VMAX_REG, 0},
};

enum {
	HAPTICS_PCM_PORT_IDX,
	HAPTICS_VI_PORT_IDX,
};

enum {
	PORT_ID_DT_IDX,
	NUM_CH_DT_IDX,
	CH_MASK_DT_IDX,
	CH_RATE_DT_IDX,
	PORT_TYPE_DT_IDX,
	NUM_SWR_PORT_DT_PARAMS,
};

struct swr_port {
	u8 port_id;
	u8 ch_mask;
	u32 ch_rate;
	u8 num_ch;
	u8 port_type;
};

struct swr_haptics_dev {
	struct device			*dev;
	struct swr_device		*swr_slave;
	struct snd_soc_component	*component;
	struct regmap			*regmap;
	struct swr_port			port;
	struct swr_port			vi_port;
	struct regulator		*slave_vdd;
	struct regulator		*hpwr_vreg;
	struct notifier_block		hboost_nb;
	u32				hpwr_voltage_mv;
	bool				slave_enabled;
	bool				hpwr_vreg_enabled;
	bool				ssr_recovery;
	u8				vmax;
	u8				clamped_vmax;
	u8				flags;
	u8				visense_enable;
};


static int vi_sense_supported = 1; /* TODO: read from PMIC registers instead */

static bool swr_hap_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SWR_READ_DATA_REG:
	case SWR_PLAY_REG:
	case SWR_VMAX_REG:
		return 1;
	default:
		return 0;
	}
}

static bool swr_hap_readable_register(struct device *dev, unsigned int reg)
{
	if (reg <= SWR_HAP_ACCESS_BASE)
		return 0;

	return 1;
}

static bool swr_hap_writeable_register(struct device *dev, unsigned int reg)
{
	if (reg <= SWR_HAP_ACCESS_BASE)
		return 0;

	switch (reg) {
	case FIFO_WR_READY_REG:
	case NUM_PAT_SMPL_REG:
	case SWR_WR_ACCESS_REG:
	case CAL_TLRA_STATUS_MSB_REG:
	case CAL_TLRA_STATUS_LSB_REG:
	case AUTO_RES_CAL_DONE_REG:
	case SWR_READ_DATA_REG:
		return 0;
	}

	return 1;
}

static int swr_hap_enable_hpwr_vreg(struct swr_haptics_dev *swr_hap)
{
	int rc;

	if (swr_hap->hpwr_vreg == NULL || swr_hap->hpwr_vreg_enabled)
		return 0;

	rc = regulator_set_voltage(swr_hap->hpwr_vreg,
			swr_hap->hpwr_voltage_mv * 1000, INT_MAX);
	if (rc < 0) {
		dev_err_ratelimited(swr_hap->dev, "%s: Set hpwr voltage failed, rc=%d\n",
				__func__, rc);
		return rc;
	}

	rc = regulator_enable(swr_hap->hpwr_vreg);
	if (rc < 0) {
		dev_err_ratelimited(swr_hap->dev, "%s: Enable hpwr failed, rc=%d\n",
				__func__, rc);
		regulator_set_voltage(swr_hap->hpwr_vreg, 0, INT_MAX);
		return rc;
	}

	dev_dbg(swr_hap->dev, "%s: enabled hpwr_regulator\n", __func__);
	swr_hap->hpwr_vreg_enabled = true;
	return 0;
}

static int swr_hap_disable_hpwr_vreg(struct swr_haptics_dev *swr_hap)
{
	int rc;

	if (swr_hap->hpwr_vreg == NULL || !swr_hap->hpwr_vreg_enabled)
		return 0;

	rc = regulator_disable(swr_hap->hpwr_vreg);
	if (rc < 0) {
		dev_err_ratelimited(swr_hap->dev, "%s: Disable hpwr failed, rc=%d\n",
				__func__, rc);
		return rc;
	}

	rc = regulator_set_voltage(swr_hap->hpwr_vreg, 0, INT_MAX);
	if (rc < 0) {
		dev_err_ratelimited(swr_hap->dev, "%s: Set hpwr voltage failed, rc=%d\n",
				__func__, rc);
		return rc;
	}

	dev_dbg(swr_hap->dev, "%s: disabled hpwr_regulator\n", __func__);
	swr_hap->hpwr_vreg_enabled = false;
	return 0;
}

static int swr_haptics_slave_enable(struct swr_haptics_dev *swr_hap)
{
	int rc;

	if (swr_hap->slave_enabled)
		return 0;

#ifdef OPLUS_ARCH_EXTENDS
/* Add for decouple haptics driver from snd card */
	if (!IS_ERR(swr_hap->slave_vdd)) {
#endif /* OPLUS_ARCH_EXTENDS */
	rc = regulator_enable(swr_hap->slave_vdd);
	if (rc < 0) {
		dev_err_ratelimited(swr_hap->dev, "%s: enable swr-slave-vdd failed, rc=%d\n",
				__func__, rc);
		return rc;
	}
#ifdef OPLUS_ARCH_EXTENDS
	}
#endif /* OPLUS_ARCH_EXTENDS */

	dev_dbg(swr_hap->dev, "%s: enable swr-slave-vdd success\n", __func__);
	swr_hap->slave_enabled = true;
	return 0;
}

static int swr_haptics_slave_disable(struct swr_haptics_dev *swr_hap)
{
	int rc;

	if (!swr_hap->slave_enabled)
		return 0;

#ifdef OPLUS_ARCH_EXTENDS
/* Add for decouple haptics driver from snd card */
	if (!IS_ERR(swr_hap->slave_vdd)) {
#endif /* OPLUS_ARCH_EXTENDS */
	rc = regulator_disable(swr_hap->slave_vdd);
	if (rc < 0) {
		dev_err_ratelimited(swr_hap->dev, "%s: disable swr-slave-vdd failed, rc=%d\n",
				__func__, rc);
		return rc;
	}
#ifdef OPLUS_ARCH_EXTENDS
	}
#endif /* OPLUS_ARCH_EXTENDS */

	dev_dbg(swr_hap->dev, "%s: disable swr-slave-vdd success\n", __func__);
	swr_hap->slave_enabled = false;
	return 0;
}

struct regmap_config swr_hap_regmap_config = {
	.reg_bits		= 16,
	.val_bits		= 8,
	.cache_type		= REGCACHE_RBTREE,
	.reg_defaults		= swr_hap_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(swr_hap_reg_defaults),
	.max_register		= SWR_HAP_REG_MAX,
	.volatile_reg		= swr_hap_volatile_register,
	.readable_reg		= swr_hap_readable_register,
	.writeable_reg		= swr_hap_writeable_register,
	.reg_format_endian	= REGMAP_ENDIAN_NATIVE,
	.val_format_endian	= REGMAP_ENDIAN_NATIVE,
	.can_multi_write	= true,
};

static const struct snd_kcontrol_new hap_swr_dac_port[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static int hap_enable_swr_dac_port(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *swr_hap_comp =
		snd_soc_dapm_to_component(w->dapm);
	struct swr_haptics_dev *swr_hap;
	u8 port_id[2], ch_mask[2], num_ch[2], port_type[2], num_port;
	u8 vmax;
	u32 ch_rate[2];
	unsigned int val;
	int rc;

	if (!swr_hap_comp) {
		pr_err("%s: swr_hap_component is NULL\n", __func__);
		return -EINVAL;
	}

	swr_hap = snd_soc_component_get_drvdata(swr_hap_comp);
	if (!swr_hap) {
		pr_err("%s: get swr_haptics_dev failed\n", __func__);
		return -ENODEV;
	}

	dev_dbg(swr_hap->dev, "%s: %s event %d\n", __func__, w->name, event);
	num_port = 1;
	port_id[HAPTICS_PCM_PORT_IDX] = swr_hap->port.port_id;
	ch_mask[HAPTICS_PCM_PORT_IDX] = swr_hap->port.ch_mask;
	ch_rate[HAPTICS_PCM_PORT_IDX] = swr_hap->port.ch_rate;
	num_ch[HAPTICS_PCM_PORT_IDX] = swr_hap->port.num_ch;
	port_type[HAPTICS_PCM_PORT_IDX] = swr_hap->port.port_type;

	if (swr_hap->visense_enable) {
		dev_dbg(swr_hap->dev, "%s: visense enable\n", __func__);
		port_id[HAPTICS_VI_PORT_IDX] = swr_hap->vi_port.port_id;
		ch_mask[HAPTICS_VI_PORT_IDX] = swr_hap->vi_port.ch_mask;
		ch_rate[HAPTICS_VI_PORT_IDX] = swr_hap->vi_port.ch_rate;
		num_ch[HAPTICS_VI_PORT_IDX] = swr_hap->vi_port.num_ch;
		port_type[HAPTICS_VI_PORT_IDX] = swr_hap->vi_port.port_type;
		++num_port;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* If SSR ever happened, toggle swr-slave-vdd for HW recovery */
		if ((swr_hap->flags & HAP_SSR_RECOVERY)
				&& swr_hap->ssr_recovery) {
			swr_haptics_slave_disable(swr_hap);
			swr_haptics_slave_enable(swr_hap);
			swr_hap->ssr_recovery = false;
		}

		vmax = swr_hap->vmax;
		if ((swr_hap->clamped_vmax != 0) && (swr_hap->vmax > swr_hap->clamped_vmax))
			vmax = swr_hap->clamped_vmax;

		if (swr_hap->visense_enable)
			vmax = MAX_CL_HAPTICS_VMAX_MV / VMAX_STEP_MV;

		rc = regmap_write(swr_hap->regmap, SWR_VMAX_REG, vmax);
		if (rc) {
			dev_err_ratelimited(swr_hap->dev, "%s: SWR_VMAX update failed, rc=%d\n",
				__func__, rc);
			return rc;
		}
		regmap_read(swr_hap->regmap, SWR_VMAX_REG, &val);
		regmap_read(swr_hap->regmap, SWR_READ_DATA_REG, &val);
		dev_dbg(swr_hap->dev, "%s: swr_vmax is set to 0x%x\n", __func__, val);
		swr_device_wakeup_vote(swr_hap->swr_slave);
		swr_connect_port(swr_hap->swr_slave, port_id, num_port,
				ch_mask, ch_rate, num_ch, port_type);
		break;
	case SND_SOC_DAPM_POST_PMU:
		rc = swr_hap_enable_hpwr_vreg(swr_hap);
		if (rc < 0) {
			dev_err_ratelimited(swr_hap->dev, "%s: Enable hpwr_vreg failed, rc=%d\n",
					__func__, rc);
#ifdef OPLUS_ARCH_EXTENDS
// request/release swr device wakeup votes properly. CR3681638
			swr_device_wakeup_unvote(swr_hap->swr_slave);
#endif /* OPLUS_ARCH_EXTENDS */
			return rc;
		}

		swr_slvdev_datapath_control(swr_hap->swr_slave,
				swr_hap->swr_slave->dev_num, true);
		/* trigger SWR play */
		val = SWR_PLAY_BIT | SWR_PLAY_SRC_VAL_SWR;
		rc = regmap_write(swr_hap->regmap, SWR_PLAY_REG, val);
		if (rc) {
			dev_err_ratelimited(swr_hap->dev, "%s: Enable SWR_PLAY failed, rc=%d\n",
					__func__, rc);
			swr_slvdev_datapath_control(swr_hap->swr_slave,
					swr_hap->swr_slave->dev_num, false);
			swr_hap_disable_hpwr_vreg(swr_hap);
#ifdef OPLUS_ARCH_EXTENDS
// request/release swr device wakeup votes properly. CR3681638
			swr_device_wakeup_unvote(swr_hap->swr_slave);
#endif /* OPLUS_ARCH_EXTENDS */
			return rc;
		}

		if (swr_hap->visense_enable) {
			rc = regmap_write(swr_hap->regmap, SWR_VISENSE_AFE_GAIN_REG,
								VISENSE_AFE_GAIN_VALUE);
			if (rc) {
				dev_err(swr_hap->dev, "%s: 0x%x reg write failed, rc = %d\n",
						__func__, SWR_VISENSE_AFE_GAIN_REG, rc);
				rc = 0;
			}
		}
#ifdef OPLUS_ARCH_EXTENDS
// request/release swr device wakeup votes properly. CR3681638
		swr_device_wakeup_unvote(swr_hap->swr_slave);
#endif /* OPLUS_ARCH_EXTENDS */
		break;
	case SND_SOC_DAPM_PRE_PMD:
#ifdef OPLUS_ARCH_EXTENDS
// request/release swr device wakeup votes properly. CR3681638
		swr_device_wakeup_vote(swr_hap->swr_slave);
#endif /* OPLUS_ARCH_EXTENDS */
		/* stop SWR play */
		val = SWR_PLAY_SRC_VAL_SWR;
		rc = regmap_write(swr_hap->regmap, SWR_PLAY_REG, val);
		if (rc) {
			dev_err_ratelimited(swr_hap->dev, "%s: Enable SWR_PLAY failed, rc=%d\n",
					__func__, rc);
#ifdef OPLUS_ARCH_EXTENDS
// request/release swr device wakeup votes properly. CR3681638
			swr_device_wakeup_unvote(swr_hap->swr_slave);
#endif /* OPLUS_ARCH_EXTENDS */
			return rc;
		}

		rc = swr_hap_disable_hpwr_vreg(swr_hap);
		if (rc < 0) {
			dev_err_ratelimited(swr_hap->dev, "%s: Disable hpwr_vreg failed, rc=%d\n",
					__func__, rc);
#ifdef OPLUS_ARCH_EXTENDS
// request/release swr device wakeup votes properly. CR3681638
			swr_device_wakeup_unvote(swr_hap->swr_slave);
#endif /* OPLUS_ARCH_EXTENDS */
			return rc;
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		swr_disconnect_port(swr_hap->swr_slave, port_id, num_port,
				ch_mask, port_type);
		swr_slvdev_datapath_control(swr_hap->swr_slave,
				swr_hap->swr_slave->dev_num, false);
		swr_device_wakeup_unvote(swr_hap->swr_slave);
		break;
	default:
		break;
	}

	return 0;
}

static int haptics_vmax_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct swr_haptics_dev *swr_hap =
			snd_soc_component_get_drvdata(component);

	pr_debug("%s: vmax %u\n", __func__, swr_hap->vmax);
	ucontrol->value.integer.value[0] = swr_hap->vmax;

	return 0;
}

static int haptics_vmax_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct swr_haptics_dev *swr_hap =
			snd_soc_component_get_drvdata(component);

	swr_hap->vmax = ucontrol->value.integer.value[0];
#ifdef OPLUS_FEATURE_RINGTONE_HAPTIC
/* Add for control VMAX */
	if (swr_hap->vmax > OPLUS_SWR_HAP_VMAX) {
		swr_hap->vmax = OPLUS_SWR_HAP_VMAX;
	}
#endif /* OPLUS_FEATURE_RINGTONE_HAPTIC */
	pr_debug("%s: vmax %u\n", __func__, swr_hap->vmax);

	return 0;
}

static int haptics_get_visense(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct swr_haptics_dev *swr_hap =
			snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = swr_hap->visense_enable;
	return 0;
}

static int haptics_set_visense(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct swr_haptics_dev *swr_hap =
			snd_soc_component_get_drvdata(component);

	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: VIsense enable current %d, new %d\n",
		 __func__, swr_hap->visense_enable, value);
	swr_hap->visense_enable = value;
	return 0;
}

static const struct snd_kcontrol_new haptics_snd_controls[] = {
#ifndef OPLUS_FEATURE_RINGTONE_HAPTIC
/* Modify for change VMAX */
	SOC_SINGLE_EXT("Haptics Amplitude Step", SND_SOC_NOPM, 0, 100, 0,
		haptics_vmax_get, haptics_vmax_put),
#else /* OPLUS_FEATURE_RINGTONE_HAPTIC */
	SOC_SINGLE_EXT("Haptics Amplitude Step", SND_SOC_NOPM, 0, OPLUS_SWR_HAP_VMAX, 0,
		haptics_vmax_get, haptics_vmax_put),
#endif /* OPLUS_FEATURE_RINGTONE_HAPTIC */
	SOC_SINGLE_EXT("Haptics VISENSE Switch", SND_SOC_NOPM, 0, 1, 0,
			haptics_get_visense, haptics_set_visense),
};

static const struct snd_soc_dapm_widget haptics_comp_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("HAP_IN"),
	SND_SOC_DAPM_MIXER_E("SWR DAC_Port", SND_SOC_NOPM, 0, 0,
			hap_swr_dac_port, ARRAY_SIZE(hap_swr_dac_port),
			hap_enable_swr_dac_port,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SPK("HAP_OUT", NULL),
};

static const struct snd_soc_dapm_route haptics_comp_dapm_route[] = {
	{"SWR DAC_Port", "Switch", "HAP_IN"},
	{"HAP_OUT", NULL, "SWR DAC_Port"},
};

static int haptics_comp_probe(struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm;

	struct swr_haptics_dev *swr_hap =
		snd_soc_component_get_drvdata(component);

	if (!swr_hap) {
		pr_err("%s: get swr_haptics_dev failed\n", __func__);
		return -EINVAL;
	}

	snd_soc_component_init_regmap(component, swr_hap->regmap);

	dapm = snd_soc_component_get_dapm(component);
	if (dapm && dapm->component) {
		snd_soc_dapm_ignore_suspend(dapm, "HAP_IN");
		snd_soc_dapm_ignore_suspend(dapm, "HAP_OUT");
	}

	return 0;
}

static void haptics_comp_remove(struct snd_soc_component *component)
{
}

#ifdef OPLUS_ARCH_EXTENDS
/* Add for decouple haptics driver from snd card */
static int dummy_haptics_comp_probe(struct snd_soc_component *component)
{
	return 0;
}

static int dummy_haptics_vmax_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int dummy_haptics_vmax_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static const struct snd_kcontrol_new dummy_haptics_snd_controls[] = {
	SOC_SINGLE_EXT("Haptics Amplitude Step", SND_SOC_NOPM, 0, 0, 0,
		dummy_haptics_vmax_get, dummy_haptics_vmax_put),
};

static const struct snd_soc_dapm_widget dummy_haptics_comp_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("HAP_IN"),
	SND_SOC_DAPM_SPK("HAP_OUT", NULL),
};

static const struct snd_soc_component_driver dummy_swr_haptics_component = {
	.name = "swr-haptics",
	.probe = dummy_haptics_comp_probe,
	.remove = haptics_comp_remove,
	.controls = dummy_haptics_snd_controls,
	.num_controls = ARRAY_SIZE(dummy_haptics_snd_controls),
	.dapm_widgets = dummy_haptics_comp_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(dummy_haptics_comp_dapm_widgets),
};

static struct snd_soc_dai_driver dummy_haptics_dai[] = {
	{
		.name = "swr_haptics",
	},
};
#endif /* OPLUS_ARCH_EXTENDS */

static const struct snd_soc_component_driver swr_haptics_component = {
	.name = "swr-haptics",
	.probe = haptics_comp_probe,
	.remove = haptics_comp_remove,
	.controls = haptics_snd_controls,
	.num_controls = ARRAY_SIZE(haptics_snd_controls),
	.dapm_widgets = haptics_comp_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(haptics_comp_dapm_widgets),
	.dapm_routes = haptics_comp_dapm_route,
	.num_dapm_routes = ARRAY_SIZE(haptics_comp_dapm_route),
};

static struct snd_soc_dai_driver haptics_dai[] = {
	{
		.name = "swr_haptics",
		.playback = {
			.stream_name = "HAPTICS_AIF Playback",
			.rates = HAPTICS_RATES,
			.formats = HAPTICS_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 1,
		},
	},
};

static int swr_haptics_parse_port_mapping(struct swr_device *sdev)
{
	struct swr_haptics_dev *swr_hap = swr_get_dev_data(sdev);
	u32 port_cfg[NUM_SWR_PORT_DT_PARAMS];
	int rc;

	if (!swr_hap) {
		dev_err(&sdev->dev, "%s: get swr_haptics_dev failed\n",
				__func__);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(sdev->dev.of_node, "qcom,rx_swr_ch_map",
			port_cfg, NUM_SWR_PORT_DT_PARAMS);
	if (rc < 0) {
		dev_err(swr_hap->dev, "%s: Get qcom,rx_swr_ch_map failed, rc=%d\n",
				__func__, rc);
		return -EINVAL;
	}

	swr_hap->port.port_id = (u8) port_cfg[PORT_ID_DT_IDX];
	swr_hap->port.num_ch = (u8) port_cfg[NUM_CH_DT_IDX];
	swr_hap->port.ch_mask = (u8) port_cfg[CH_MASK_DT_IDX];
	swr_hap->port.ch_rate = port_cfg[CH_RATE_DT_IDX];
	swr_hap->port.port_type = (u8) port_cfg[PORT_TYPE_DT_IDX];

	dev_dbg(swr_hap->dev, "%s: port_id = %d, ch_mask = %d, ch_rate = %d, num_ch = %d, port_type = %d\n",
			__func__, swr_hap->port.port_id,
			swr_hap->port.ch_mask, swr_hap->port.ch_rate,
			swr_hap->port.num_ch, swr_hap->port.port_type);

	if (vi_sense_supported) {

		rc = of_property_read_u32_array(sdev->dev.of_node, "qcom,rx_swr_vi_ch_map",
				port_cfg, NUM_SWR_PORT_DT_PARAMS);
		if (rc < 0) {
			dev_err(swr_hap->dev, "%s: Get qcom,rx_swr_vi_ch_map failed, rc=%d\n",
					__func__, rc);
			return -EINVAL;
		}

		swr_hap->vi_port.port_id = (u8) port_cfg[PORT_ID_DT_IDX];
		swr_hap->vi_port.num_ch = (u8) port_cfg[NUM_CH_DT_IDX];
		swr_hap->vi_port.ch_mask = (u8) port_cfg[CH_MASK_DT_IDX];
		swr_hap->vi_port.ch_rate =  port_cfg[CH_RATE_DT_IDX];
		swr_hap->vi_port.port_type = (u8) port_cfg[PORT_TYPE_DT_IDX];
	}
	return 0;
}

static int hboost_notifier(struct notifier_block *nb, unsigned long event, void *val)
{
	struct swr_haptics_dev *swr_hap = container_of(nb, struct swr_haptics_dev, hboost_nb);
	u32 vmax_mv;

	switch (event) {
	case VMAX_CLAMP:
		vmax_mv = *(u32 *)val;
		if (vmax_mv > MAX_HAPTICS_VMAX_MV) {
			dev_err_ratelimited(swr_hap->dev, "%s: voted Vmax (%u mv) is higher than maximum (%u mv)\n",
					__func__, vmax_mv, MAX_HAPTICS_VMAX_MV);
			return -EINVAL;
		}

		dev_dbg(swr_hap->dev, "%s: Vmax is clamped at %u mv to support hBoost concurrency\n",
				__func__, vmax_mv);
		swr_hap->clamped_vmax = vmax_mv / VMAX_STEP_MV;
		break;
	default:
		break;
	}

	return 0;
}

static int swr_haptics_probe(struct swr_device *sdev)
{
	struct swr_haptics_dev *swr_hap;
	struct device_node *node = sdev->dev.of_node;
	int rc;
	u8 devnum;
	u32 pmic_type;
	int retry = 5;

	swr_hap = devm_kzalloc(&sdev->dev,
			sizeof(struct swr_haptics_dev), GFP_KERNEL);
	if (!swr_hap)
		return -ENOMEM;

	/* VMAX default to 5V */
	swr_hap->vmax = 100;
	swr_hap->swr_slave = sdev;
	swr_hap->dev = &sdev->dev;
	pmic_type = (uintptr_t)of_device_get_match_data(swr_hap->dev);
	if (pmic_type == PM8350B)
		swr_hap->flags |= HAP_SSR_RECOVERY;

	swr_set_dev_data(sdev, swr_hap);

	rc = swr_haptics_parse_port_mapping(sdev);
	if (rc < 0) {
		dev_err(swr_hap->dev, "%s: failed to parse swr port mapping, rc=%d\n",
				__func__, rc);
		goto clean;
	}

	swr_hap->slave_vdd = devm_regulator_get(swr_hap->dev, "swr-slave");
	if (IS_ERR(swr_hap->slave_vdd)) {
		rc = PTR_ERR(swr_hap->slave_vdd);
#ifdef OPLUS_ARCH_EXTENDS
/* Add for decouple haptics driver from snd card */
		dev_err(swr_hap->dev, "%s: get swr-slave-supply failed, rc=%d\n",
				__func__, rc);
		if (rc != -EPROBE_DEFER)
			goto clean;
		else
			rc = 0;
#else /* OPLUS_ARCH_EXTENDS */
		if (rc != -EPROBE_DEFER)
			dev_err(swr_hap->dev, "%s: get swr-slave-supply failed, rc=%d\n",
					__func__, rc);
		goto clean;
#endif /* OPLUS_ARCH_EXTENDS */
	}

	if (of_find_property(node, "qcom,hpwr-supply", NULL)) {
		swr_hap->hpwr_vreg = devm_regulator_get(swr_hap->dev,
						"qcom,hpwr");
		if (IS_ERR(swr_hap->hpwr_vreg)) {
			rc = PTR_ERR(swr_hap->hpwr_vreg);
			if (rc != -EPROBE_DEFER)
				dev_err(swr_hap->dev, "%s: Get qcom,hpwr-supply failed, rc=%d\n",
						__func__, rc);
			goto clean;
		}

		rc = of_property_read_u32(node, "qcom,hpwr-voltage-mv",
				&swr_hap->hpwr_voltage_mv);
		if (rc < 0) {
			dev_err(swr_hap->dev, "%s: Failed to read qcom,hpwr-voltage-mv, rc=%d\n",
					__func__, rc);
			goto clean;
		}
	}

	rc = swr_haptics_slave_enable(swr_hap);
	if (rc < 0) {
		dev_err(swr_hap->dev, "%s: enable swr-slave-vdd failed, rc=%d\n",
				__func__, rc);
		goto clean;
	}
	do {
		/* Add delay for soundwire enumeration */
		usleep_range(500, 510);
		rc = swr_get_logical_dev_num(sdev, sdev->addr, &devnum);
	} while (rc && --retry);

	if (rc) {
#ifdef OPLUS_ARCH_EXTENDS
/* Add for decouple haptics driver from snd card */
		dev_err(swr_hap->dev, "%s: failed to get devnum for swr-haptics, rc=%d, devnum=%d\n",
				__func__, rc, devnum);
		if (haptics_probe_retry < 5) {
			haptics_probe_retry++;
			rc = -EPROBE_DEFER;
			goto dev_err;
		}

		rc = snd_soc_register_component(&sdev->dev,
				&dummy_swr_haptics_component, dummy_haptics_dai, ARRAY_SIZE(dummy_haptics_dai));
		haptics_probe_retry = 0;
		goto dev_err;

#else /* OPLUS_ARCH_EXTENDS */
		dev_err(swr_hap->dev, "%s: failed to get devnum for swr-haptics, rc=%d\n",
				__func__, rc);
		rc = -EPROBE_DEFER;
		goto dev_err;
#endif /* OPLUS_ARCH_EXTENDS */
	}

	sdev->dev_num = devnum;
	swr_hap->regmap = devm_regmap_init_swr(sdev, &swr_hap_regmap_config);
	if (IS_ERR(swr_hap->regmap)) {
		rc = PTR_ERR(swr_hap->regmap);
		dev_err(swr_hap->dev, "%s: init regmap failed, rc=%d\n",
				__func__, rc);
		goto dev_err;
	}

	rc = snd_soc_register_component(&sdev->dev,
			&swr_haptics_component, haptics_dai, ARRAY_SIZE(haptics_dai));
	if (rc) {
		dev_err(swr_hap->dev, "%s: register swr_haptics component failed, rc=%d\n",
				__func__, rc);
		goto dev_err;
	}

	swr_hap->hboost_nb.notifier_call = hboost_notifier;
	register_hboost_event_notifier(&swr_hap->hboost_nb);
	return 0;
dev_err:
	swr_haptics_slave_disable(swr_hap);
	swr_remove_device(sdev);
clean:
	swr_set_dev_data(sdev, NULL);
	return rc;
}

static int swr_haptics_remove(struct swr_device *sdev)
{
	struct swr_haptics_dev *swr_hap;
	int rc = 0;

	swr_hap = swr_get_dev_data(sdev);
	if (!swr_hap) {
		dev_err(&sdev->dev, "%s: no data for swr_hap\n", __func__);
		rc = -ENODEV;
		goto clean;
	}

	unregister_hboost_event_notifier(&swr_hap->hboost_nb);
	rc = swr_haptics_slave_disable(swr_hap);
	if (rc < 0) {
		dev_err(swr_hap->dev, "%s: disable swr-slave failed, rc=%d\n",
				__func__, rc);
		goto clean;
	}
clean:
	snd_soc_unregister_component(&sdev->dev);
	swr_set_dev_data(sdev, NULL);
	return rc;
}

static int swr_haptics_device_up(struct swr_device *sdev)
{
	struct swr_haptics_dev *swr_hap;

	swr_hap = swr_get_dev_data(sdev);
	if (!swr_hap) {
		dev_err_ratelimited(&sdev->dev, "%s: no data for swr_hap\n", __func__);
		return -ENODEV;
	}

	if (swr_hap->flags & HAP_SSR_RECOVERY)
		swr_hap->ssr_recovery = true;

	/* Take SWR slave out of reset */
	return swr_haptics_slave_enable(swr_hap);
}

static int swr_haptics_device_down(struct swr_device *sdev)
{
	struct swr_haptics_dev *swr_hap = swr_get_dev_data(sdev);
	int rc;

	if (!swr_hap) {
		dev_err_ratelimited(&sdev->dev, "%s: no data for swr_hap\n", __func__);
		return -ENODEV;
	}

	/* Disable HAP_PWR regulator */
	rc = swr_hap_disable_hpwr_vreg(swr_hap);
	if (rc < 0) {
		dev_err_ratelimited(swr_hap->dev, "Disable hpwr_vreg failed, rc=%d\n",
				rc);
		return rc;
	}

	/* Put SWR slave into reset */
	return swr_haptics_slave_disable(swr_hap);
}

static int swr_haptics_suspend(struct device *dev)
{
	struct swr_haptics_dev *swr_hap;
	int rc = 0;

	swr_hap = swr_get_dev_data(to_swr_device(dev));
	if (!swr_hap) {
		dev_err_ratelimited(dev, "%s: no data for swr_hap\n", __func__);
		return -ENODEV;
	}

	return rc;
}

static int swr_haptics_resume(struct device *dev)
{
	struct swr_haptics_dev *swr_hap;
	int rc = 0;

	swr_hap = swr_get_dev_data(to_swr_device(dev));
	if (!swr_hap) {
		dev_err_ratelimited(dev, "%s: no data for swr_hap\n", __func__);
		return -ENODEV;
	}

	return rc;
}

static const struct of_device_id swr_haptics_match_table[] = {
	{
		.compatible = "qcom,swr-haptics",
		.data = NULL,
	},
	{
		.compatible = "qcom,pm8350b-swr-haptics",
		.data = (void *)PM8350B,
	},
	{
		.compatible = "qcom,pm8550b-swr-haptics",
		.data = (void *)PM8550B,
	},
	{
		.compatible = "qcom,pmih010x-swr-haptics",
		.data = (void *)PMIH010X,
	},
	{ },
};

static const struct swr_device_id swr_haptics_id[] = {
	{"swr-haptics", 0},
	{"pm8350b-swr-haptics", 0},
	{"pm8550b-swr-haptics", 0},
	{"pmih010x-swr-haptics", 0},
	{},
};

static const struct dev_pm_ops swr_haptics_pm_ops = {
	.suspend = swr_haptics_suspend,
	.resume = swr_haptics_resume,
};

static struct swr_driver swr_haptics_driver = {
	.driver = {
		.name = "swr-haptics",
		.owner = THIS_MODULE,
		.pm = &swr_haptics_pm_ops,
		.of_match_table = swr_haptics_match_table,
	},
	.probe = swr_haptics_probe,
	.remove = swr_haptics_remove,
	.id_table = swr_haptics_id,
	.device_up = swr_haptics_device_up,
	.device_down = swr_haptics_device_down,
};

static int __init swr_haptics_init(void)
{
	return swr_driver_register(&swr_haptics_driver);
}

static void __exit swr_haptics_exit(void)
{
	swr_driver_unregister(&swr_haptics_driver);
}

module_init(swr_haptics_init);
module_exit(swr_haptics_exit);

MODULE_DESCRIPTION("SWR haptics driver");
MODULE_LICENSE("GPL v2");
