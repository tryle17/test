// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/firmware.h>
#include <linux/completion.h>
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
#include <linux/soc/qcom/wcd939x-i2c.h>
#endif
#if IS_ENABLED(CONFIG_QCOM_FSA4480_I2C)
#include <linux/soc/qcom/fsa4480-i2c.h>
#endif
#include <linux/usb/typec.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <asoc/msm-cdc-pinctrl.h>
#include <asoc/wcdcal-hwdep.h>
#include "wcd-mbhc-legacy.h"
#include "wcd-mbhc-adc.h"
#include <asoc/wcd-mbhc-v2-api.h>

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
#include <soc/oplus/system/oplus_mm_kevent_fb.h>
#endif

static const unsigned int mbhc_ext_dev_supported_table[] = {
	EXTCON_JACK_MICROPHONE,
	EXTCON_JACK_HEADPHONE,
	EXTCON_JACK_LINE_OUT,
	EXTCON_MECHANICAL,
	EXTCON_NONE,
};

struct mutex hphl_pa_lock;
struct mutex hphr_pa_lock;

void wcd_mbhc_jack_report(struct wcd_mbhc *mbhc,
			  struct snd_soc_jack *jack, int status, int mask)
{
	snd_soc_jack_report(jack, status, mask);
}
EXPORT_SYMBOL(wcd_mbhc_jack_report);

#if IS_ENABLED(CONFIG_AUDIO_QGKI)
static void __hphocp_off_report(struct wcd_mbhc *mbhc, u32 jack_status,
				int irq)
{
	struct snd_soc_component *component = mbhc->component;

	dev_dbg(component->dev, "%s: clear ocp status %x\n",
		__func__, jack_status);

	if (mbhc->hph_status & jack_status) {
		mbhc->hph_status &= ~jack_status;
		wcd_mbhc_jack_report(mbhc, &mbhc->headset_jack,
				     mbhc->hph_status, WCD_MBHC_JACK_MASK);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_OCP_FSM_EN, 0);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_OCP_FSM_EN, 1);
		/*
		 * reset retry counter as PA is turned off signifying
		 * start of new OCP detection session
		 */
		if (mbhc->intr_ids->hph_left_ocp)
			mbhc->hphlocp_cnt = 0;
		else
			mbhc->hphrocp_cnt = 0;
		mbhc->mbhc_cb->irq_control(component, irq, true);
	}
}

static void hphrocp_off_report(struct wcd_mbhc *mbhc, u32 jack_status)
{
	__hphocp_off_report(mbhc, SND_JACK_OC_HPHR,
			    mbhc->intr_ids->hph_right_ocp);
}

static void hphlocp_off_report(struct wcd_mbhc *mbhc, u32 jack_status)
{
	__hphocp_off_report(mbhc, SND_JACK_OC_HPHL,
			    mbhc->intr_ids->hph_left_ocp);
}
#endif /* CONFIG_AUDIO_QGKI */

static void wcd_program_hs_vref(struct wcd_mbhc *mbhc)
{
	struct wcd_mbhc_plug_type_cfg *plug_type_cfg;
	struct snd_soc_component *component = mbhc->component;
	u32 reg_val;

	plug_type_cfg = WCD_MBHC_CAL_PLUG_TYPE_PTR(
				mbhc->mbhc_cfg->calibration);
	reg_val = ((plug_type_cfg->v_hs_max - HS_VREF_MIN_VAL) / 100);

	dev_dbg(component->dev, "%s: reg_val  = %x\n",
		__func__, reg_val);
	WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_HS_VREF, reg_val);
}

static void wcd_program_btn_threshold(const struct wcd_mbhc *mbhc, bool micbias)
{
	struct wcd_mbhc_btn_detect_cfg *btn_det;
	struct snd_soc_component *component = mbhc->component;
	struct snd_soc_card *card = component->card;
	s16 *btn_low, *btn_high;

	if (mbhc->mbhc_cfg->calibration == NULL) {
		dev_err(card->dev, "%s: calibration data is NULL\n", __func__);
		return;
	}

	btn_det = WCD_MBHC_CAL_BTN_DET_PTR(mbhc->mbhc_cfg->calibration);
	btn_low = btn_det->_v_btn_low;
	btn_high = ((void *)&btn_det->_v_btn_low) +
			(sizeof(btn_det->_v_btn_low[0]) * btn_det->num_btn);

	mbhc->mbhc_cb->set_btn_thr(component, btn_low, btn_high,
				btn_det->num_btn, micbias);
}

void wcd_enable_curr_micbias(const struct wcd_mbhc *mbhc,
				const enum wcd_mbhc_cs_mb_en_flag cs_mb_en)
{

	/*
	 * Some codecs handle micbias/pullup enablement in codec
	 * drivers itself and micbias is not needed for regular
	 * plug type detection. So if micbias_control callback function
	 * is defined, just return.
	 */
	if (mbhc->mbhc_cb->mbhc_micbias_control)
		return;

	pr_debug("%s: enter, cs_mb_en: %d\n", __func__, cs_mb_en);

	switch (cs_mb_en) {
	case WCD_MBHC_EN_CS:
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MICB_CTRL, 0);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_BTN_ISRC_CTL, 3);
		/* Program Button threshold registers as per CS */
		wcd_program_btn_threshold(mbhc, false);
		break;
	case WCD_MBHC_EN_MB:
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_BTN_ISRC_CTL, 0);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_FSM_EN, 1);

		/* Disable PULL_UP_EN & enable MICBIAS */
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MICB_CTRL, 2);
		/* Program Button threshold registers as per MICBIAS */
		wcd_program_btn_threshold(mbhc, true);
		break;
	case WCD_MBHC_EN_PULLUP:
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_BTN_ISRC_CTL, 3);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_FSM_EN, 1);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MICB_CTRL, 1);
		/* Program Button threshold registers as per MICBIAS */
		wcd_program_btn_threshold(mbhc, true);
		break;
	case WCD_MBHC_EN_NONE:
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_BTN_ISRC_CTL, 0);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_FSM_EN, 1);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MICB_CTRL, 0);
		break;
	default:
		pr_debug("%s: Invalid parameter", __func__);
		break;
	}

	pr_debug("%s: exit\n", __func__);
}
EXPORT_SYMBOL(wcd_enable_curr_micbias);

static const char *wcd_mbhc_get_event_string(int event)
{
	switch (event) {
	case WCD_EVENT_PRE_MICBIAS_2_OFF:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_PRE_MICBIAS_2_OFF);
	case WCD_EVENT_POST_MICBIAS_2_OFF:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_POST_MICBIAS_2_OFF);
	case WCD_EVENT_PRE_MICBIAS_2_ON:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_PRE_MICBIAS_2_ON);
	case WCD_EVENT_POST_MICBIAS_2_ON:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_POST_MICBIAS_2_ON);
	case WCD_EVENT_PRE_HPHL_PA_ON:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_PRE_HPHL_PA_ON);
	case WCD_EVENT_POST_HPHL_PA_OFF:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_POST_HPHL_PA_OFF);
	case WCD_EVENT_PRE_HPHR_PA_ON:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_PRE_HPHR_PA_ON);
	case WCD_EVENT_POST_HPHR_PA_OFF:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_POST_HPHR_PA_OFF);
	case WCD_EVENT_PRE_HPHR_PA_OFF:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_PRE_HPHR_PA_OFF);
	case WCD_EVENT_PRE_HPHL_PA_OFF:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_PRE_HPHL_PA_OFF);
	case WCD_EVENT_POST_DAPM_MICBIAS_2_ON:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_POST_DAPM_MICBIAS_2_ON);
	case WCD_EVENT_PRE_DAPM_MICBIAS_2_ON:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_PRE_DAPM_MICBIAS_2_ON);
	case WCD_EVENT_POST_DAPM_MICBIAS_2_OFF:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_POST_DAPM_MICBIAS_2_OFF);
	case WCD_EVENT_PRE_DAPM_MICBIAS_2_OFF:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_PRE_DAPM_MICBIAS_2_OFF);
	case WCD_EVENT_OCP_OFF:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_OCP_OFF);
	case WCD_EVENT_OCP_ON:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_OCP_ON);
	case WCD_EVENT_INVALID:
	default:
		return WCD_MBHC_STRINGIFY(WCD_EVENT_INVALID);
	}
}

static int wcd_event_notify(struct notifier_block *self, unsigned long val,
			    void *data)
{
	struct wcd_mbhc *mbhc = (struct wcd_mbhc *)data;
	enum wcd_notify_event event = (enum wcd_notify_event)val;
	struct snd_soc_component *component = mbhc->component;
	bool micbias2 = false;
	bool micbias1 = false;
	u8 fsm_en = 0;

	pr_debug("%s: event %s (%d)\n", __func__,
		 wcd_mbhc_get_event_string(event), event);
	if (mbhc->mbhc_cb->micbias_enable_status) {
		micbias2 = mbhc->mbhc_cb->micbias_enable_status(mbhc,
								MIC_BIAS_2);
		micbias1 = mbhc->mbhc_cb->micbias_enable_status(mbhc,
								MIC_BIAS_1);
	}
	switch (event) {
	/* MICBIAS usage change */
	case WCD_EVENT_POST_DAPM_MICBIAS_2_ON:
		mbhc->is_hs_recording = true;
		pr_debug("%s: is_capture: %d\n", __func__,
			  mbhc->is_hs_recording);
		break;
	case WCD_EVENT_POST_MICBIAS_2_ON:
		if (!mbhc->micbias_enable)
			goto out_micb_en;
		if (mbhc->mbhc_cb->mbhc_common_micb_ctrl) {
			mbhc->mbhc_cb->mbhc_common_micb_ctrl(component,
					MBHC_COMMON_MICB_PRECHARGE,
					true);
			mbhc->mbhc_cb->mbhc_common_micb_ctrl(component,
					MBHC_COMMON_MICB_SET_VAL,
					true);
			/*
			 * Special headset needs MICBIAS as 2.7V so wait for
			 * 50 msec for the MICBIAS to reach 2.7 volts.
			 */
			msleep(50);
		}
		if (mbhc->mbhc_cb->set_auto_zeroing)
			mbhc->mbhc_cb->set_auto_zeroing(component, true);
		if (mbhc->mbhc_cb->mbhc_common_micb_ctrl)
			mbhc->mbhc_cb->mbhc_common_micb_ctrl(component,
					MBHC_COMMON_MICB_PRECHARGE,
					false);
out_micb_en:
		/* Disable current source if micbias enabled */
		if (mbhc->mbhc_cb->mbhc_micbias_control) {
			WCD_MBHC_REG_READ(WCD_MBHC_FSM_EN, fsm_en);
			if (fsm_en)
				WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_BTN_ISRC_CTL,
							 0);
		} else {
			mbhc->is_hs_recording = true;
			wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_MB);
		}
		/* configure cap settings properly when micbias is enabled */
		if (mbhc->mbhc_cb->set_cap_mode)
			mbhc->mbhc_cb->set_cap_mode(component, micbias1, true);
		break;
	case WCD_EVENT_PRE_MICBIAS_2_OFF:
		/*
		 * Before MICBIAS_2 is turned off, if FSM is enabled,
		 * make sure current source is enabled so as to detect
		 * button press/release events
		 */
		if (mbhc->mbhc_cb->mbhc_micbias_control &&
		    !mbhc->micbias_enable) {
			WCD_MBHC_REG_READ(WCD_MBHC_FSM_EN, fsm_en);
			if (fsm_en)
				WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_BTN_ISRC_CTL,
							 3);
		}
		break;
	/* MICBIAS usage change */
	case WCD_EVENT_POST_DAPM_MICBIAS_2_OFF:
		mbhc->is_hs_recording = false;
		pr_debug("%s: is_capture: %d\n", __func__,
			  mbhc->is_hs_recording);
		break;
	case WCD_EVENT_POST_MICBIAS_2_OFF:
		if (!mbhc->mbhc_cb->mbhc_micbias_control)
			mbhc->is_hs_recording = false;
		if (mbhc->micbias_enable) {
			wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_MB);
			break;
		}

		if (mbhc->mbhc_cb->set_auto_zeroing)
			mbhc->mbhc_cb->set_auto_zeroing(component, false);
		if (mbhc->mbhc_cb->set_micbias_value && !mbhc->micbias_enable)
			mbhc->mbhc_cb->set_micbias_value(component);
		/* Enable PULL UP if PA's are enabled */
		if ((test_bit(WCD_MBHC_EVENT_PA_HPHL, &mbhc->event_state)) ||
				(test_bit(WCD_MBHC_EVENT_PA_HPHR,
					  &mbhc->event_state)))
			/* enable pullup and cs, disable mb */
			wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_PULLUP);
		else
			/* enable current source and disable mb, pullup*/
#ifndef OPLUS_ARCH_EXTENDS
/* 1.Modify for some headset button not work after headset mic
 * stop use.(ex: stop recording, hangup voice call without plug
 * out headset).
 * 2. Modify for headphone wrong detect as headset.1247369.
 * step: plug out headset when recording or in voice call,
 * then plug in a headphone, it detect as headset.
 */
			wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_CS);
#else /* OPLUS_ARCH_EXTENDS */
			{
				pr_info("%s: current_plug %d\n", __func__, mbhc->current_plug);
				if (mbhc->current_plug == MBHC_PLUG_TYPE_HEADSET) {
					wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_MB);
				} else {
					wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_CS);
				}
			}
#endif /* OPLUS_ARCH_EXTENDS */

		/* configure cap settings properly when micbias is disabled */
		if (mbhc->mbhc_cb->set_cap_mode)
			mbhc->mbhc_cb->set_cap_mode(component, micbias1, false);
		break;
	case WCD_EVENT_PRE_HPHL_PA_OFF:
		break;
	case WCD_EVENT_POST_HPHL_PA_OFF:
		mutex_lock(&hphl_pa_lock);
		clear_bit(WCD_MBHC_HPHL_PA_OFF_ACK, &mbhc->hph_pa_dac_state);
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
		if (mbhc->hph_status & SND_JACK_OC_HPHL)
			hphlocp_off_report(mbhc, SND_JACK_OC_HPHL);
#endif /* CONFIG_AUDIO_QGKI */
		clear_bit(WCD_MBHC_EVENT_PA_HPHL, &mbhc->event_state);
		/* check if micbias is enabled */
		if (micbias2)
			/* Disable cs, pullup & enable micbias */
			wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_MB);
		else
			/* Disable micbias, pullup & enable cs */
			wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_CS);
		mutex_unlock(&hphl_pa_lock);
		clear_bit(WCD_MBHC_ANC0_OFF_ACK, &mbhc->hph_anc_state);
		break;
	case WCD_EVENT_PRE_HPHR_PA_OFF:
		break;
	case WCD_EVENT_POST_HPHR_PA_OFF:
		mutex_lock(&hphr_pa_lock);
		clear_bit(WCD_MBHC_HPHR_PA_OFF_ACK, &mbhc->hph_pa_dac_state);
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
		if (mbhc->hph_status & SND_JACK_OC_HPHR)
			hphrocp_off_report(mbhc, SND_JACK_OC_HPHR);
#endif /* CONFIG_AUDIO_QGKI */
		clear_bit(WCD_MBHC_EVENT_PA_HPHR, &mbhc->event_state);
		/* check if micbias is enabled */
		if (micbias2)
			/* Disable cs, pullup & enable micbias */
			wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_MB);
		else
			/* Disable micbias, pullup & enable cs */
			wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_CS);
		mutex_unlock(&hphr_pa_lock);
		clear_bit(WCD_MBHC_ANC1_OFF_ACK, &mbhc->hph_anc_state);
		break;
	case WCD_EVENT_PRE_HPHL_PA_ON:
		set_bit(WCD_MBHC_EVENT_PA_HPHL, &mbhc->event_state);
		/* check if micbias is enabled */
		if (micbias2)
			/* Disable cs, pullup & enable micbias */
			wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_MB);
		else
			/* Disable micbias, enable pullup & cs */
			wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_PULLUP);
		break;
	case WCD_EVENT_PRE_HPHR_PA_ON:
		set_bit(WCD_MBHC_EVENT_PA_HPHR, &mbhc->event_state);
		/* check if micbias is enabled */
		if (micbias2)
			/* Disable cs, pullup & enable micbias */
			wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_MB);
		else
			/* Disable micbias, enable pullup & cs */
			wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_PULLUP);
		break;
	case WCD_EVENT_OCP_OFF:
		mbhc->mbhc_cb->irq_control(mbhc->component,
					   mbhc->intr_ids->hph_left_ocp,
					   false);
		break;
	case WCD_EVENT_OCP_ON:
		mbhc->mbhc_cb->irq_control(mbhc->component,
					   mbhc->intr_ids->hph_left_ocp,
					   true);
		break;
	default:
		break;
	}
	return 0;
}

int wcd_cancel_btn_work(struct wcd_mbhc *mbhc)
{
	int r;

	r = cancel_delayed_work_sync(&mbhc->mbhc_btn_dwork);
	/*
	 * if scheduled mbhc.mbhc_btn_dwork is canceled from here,
	 * we have to unlock from here instead btn_work
	 */
	if (r)
		mbhc->mbhc_cb->lock_sleep(mbhc, false);
	return r;
}
EXPORT_SYMBOL(wcd_cancel_btn_work);

bool wcd_swch_level_remove(struct wcd_mbhc *mbhc)
{
	u16 result2 = 0;

	WCD_MBHC_REG_READ(WCD_MBHC_SWCH_LEVEL_REMOVE, result2);
	return (result2) ? true : false;
}
EXPORT_SYMBOL(wcd_swch_level_remove);

static void wcd_mbhc_clr_and_turnon_hph_padac(struct wcd_mbhc *mbhc)
{
	bool pa_turned_on = false;
	u8 wg_time = 0;

	WCD_MBHC_REG_READ(WCD_MBHC_HPH_CNP_WG_TIME, wg_time);
	wg_time += 1;

	mutex_lock(&hphr_pa_lock);
	if (test_and_clear_bit(WCD_MBHC_HPHR_PA_OFF_ACK,
			       &mbhc->hph_pa_dac_state)) {
		pr_debug("%s: HPHR clear flag and enable PA\n", __func__);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_HPHR_PA_EN, 1);
		pa_turned_on = true;
	}
	mutex_unlock(&hphr_pa_lock);
	mutex_lock(&hphl_pa_lock);
	if (test_and_clear_bit(WCD_MBHC_HPHL_PA_OFF_ACK,
			       &mbhc->hph_pa_dac_state)) {
		pr_debug("%s: HPHL clear flag and enable PA\n", __func__);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_HPHL_PA_EN, 1);
		pa_turned_on = true;
	}
	mutex_unlock(&hphl_pa_lock);

	if (pa_turned_on) {
		pr_debug("%s: PA was turned on by MBHC and not by DAPM\n",
			 __func__);
		usleep_range(wg_time * 1000, wg_time * 1000 + 50);
	}

	if (test_and_clear_bit(WCD_MBHC_ANC0_OFF_ACK,
				&mbhc->hph_anc_state)) {
		usleep_range(20000, 20100);
		pr_debug("%s: HPHL ANC clear flag and enable ANC_EN\n",
			__func__);
		if (mbhc->mbhc_cb->update_anc_state)
			mbhc->mbhc_cb->update_anc_state(mbhc->component,
						true, 0);
	}

	if (test_and_clear_bit(WCD_MBHC_ANC1_OFF_ACK,
				&mbhc->hph_anc_state)) {
		usleep_range(20000, 20100);
		pr_debug("%s: HPHR ANC clear flag and enable ANC_EN\n",
			__func__);
		if (mbhc->mbhc_cb->update_anc_state)
			mbhc->mbhc_cb->update_anc_state(mbhc->component,
						true, 1);
	}

}

static bool wcd_mbhc_is_hph_pa_on(struct wcd_mbhc *mbhc)
{
	bool hph_pa_on = false;

	WCD_MBHC_REG_READ(WCD_MBHC_HPH_PA_EN, hph_pa_on);

	return (hph_pa_on) ? true : false;
}

static void wcd_mbhc_set_and_turnoff_hph_padac(struct wcd_mbhc *mbhc)
{
	u8 wg_time = 0;

	WCD_MBHC_REG_READ(WCD_MBHC_HPH_CNP_WG_TIME, wg_time);
	wg_time += 1;

	/* If headphone PA is on, check if userspace receives
	 * removal event to sync-up PA's state
	 */
	if (wcd_mbhc_is_hph_pa_on(mbhc)) {
		pr_debug("%s PA is on, setting PA_OFF_ACK\n", __func__);
		set_bit(WCD_MBHC_HPHL_PA_OFF_ACK, &mbhc->hph_pa_dac_state);
		set_bit(WCD_MBHC_HPHR_PA_OFF_ACK, &mbhc->hph_pa_dac_state);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_HPHL_OCP_DET_EN, 0);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_HPHR_OCP_DET_EN, 0);
	} else {
		pr_debug("%s PA is off\n", __func__);
	}
	WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_HPH_PA_EN, 0);
	usleep_range(wg_time * 1000, wg_time * 1000 + 50);


	if (mbhc->mbhc_cb->is_anc_on && mbhc->mbhc_cb->is_anc_on(mbhc)) {
		usleep_range(20000, 20100);
		pr_debug("%s ANC is on, setting ANC_OFF_ACK\n", __func__);
		set_bit(WCD_MBHC_ANC0_OFF_ACK, &mbhc->hph_anc_state);
		set_bit(WCD_MBHC_ANC1_OFF_ACK, &mbhc->hph_anc_state);
		if (mbhc->mbhc_cb->update_anc_state) {
			mbhc->mbhc_cb->update_anc_state(mbhc->component,
						false, 0);
			mbhc->mbhc_cb->update_anc_state(mbhc->component,
						false, 1);
		} else {
			pr_debug("%s ANC is off\n", __func__);
		}
	}
}

int wcd_mbhc_get_impedance(struct wcd_mbhc *mbhc, uint32_t *zl,
			uint32_t *zr)
{
	*zl = mbhc->zl;
	*zr = mbhc->zr;

	if (*zl && *zr)
		return 0;
	else
		return -EINVAL;
}
EXPORT_SYMBOL(wcd_mbhc_get_impedance);

void wcd_mbhc_hs_elec_irq(struct wcd_mbhc *mbhc, int irq_type,
				 bool enable)
{
	int irq;

	WCD_MBHC_RSC_ASSERT_LOCKED(mbhc);

#ifdef OPLUS_ARCH_EXTENDS
/* Add for log */
	pr_info("%s: enter irq_type: %d, enable: %d\n",
		__func__, irq_type, enable);
#endif /* OPLUS_ARCH_EXTENDS */

	if (irq_type == WCD_MBHC_ELEC_HS_INS)
		irq = mbhc->intr_ids->mbhc_hs_ins_intr;
	else if (irq_type == WCD_MBHC_ELEC_HS_REM)
		irq = mbhc->intr_ids->mbhc_hs_rem_intr;
	else {
		pr_debug("%s: irq_type: %d, enable: %d\n",
			__func__, irq_type, enable);
		return;
	}

	pr_debug("%s: irq: %d, enable: %d, intr_status:%lu\n",
		 __func__, irq, enable, mbhc->intr_status);
	if ((test_bit(irq_type, &mbhc->intr_status)) != enable) {
		mbhc->mbhc_cb->irq_control(mbhc->component, irq, enable);
		if (enable)
			set_bit(irq_type, &mbhc->intr_status);
		else
			clear_bit(irq_type, &mbhc->intr_status);
	}
}
EXPORT_SYMBOL(wcd_mbhc_hs_elec_irq);

void wcd_mbhc_report_plug(struct wcd_mbhc *mbhc, int insertion,
				enum snd_jack_types jack_type)
{
	struct snd_soc_component *component = mbhc->component;
	bool is_pa_on = false;
	u8 fsm_en = 0;
	int extdev_type = 0;

	WCD_MBHC_RSC_ASSERT_LOCKED(mbhc);

	pr_debug("%s: enter insertion %d hph_status %x\n",
		 __func__, insertion, mbhc->hph_status);
	if (!insertion) {
		/* Report removal */
		mbhc->hph_status &= ~jack_type;
		/*
		 * cancel possibly scheduled btn work and
		 * report release if we reported button press
		 */
		if (wcd_cancel_btn_work(mbhc)) {
			pr_debug("%s: button press is canceled\n", __func__);
		} else if (mbhc->buttons_pressed) {
#ifndef OPLUS_ARCH_EXTENDS
/* Modify for headset detect */
			pr_debug("%s: release of button press%d\n",
				 __func__, jack_type);
			wcd_mbhc_jack_report(mbhc, &mbhc->button_jack, 0,
					    mbhc->buttons_pressed);
#else /* OPLUS_ARCH_EXTENDS */
			pr_info("%s: release of button press%d\n",
				 __func__, jack_type);
			/* Modified for supporting line control earphone volume key
				up/down */
			if (mbhc->buttons_pressed & (SND_JACK_BTN_2 | SND_JACK_BTN_3)) {
				wcd_mbhc_jack_report(mbhc, &mbhc->button_jack, 0,
					    mbhc->buttons_pressed);
			}
#endif /* OPLUS_ARCH_EXTENDS */
			mbhc->buttons_pressed &=
				~WCD_MBHC_JACK_BUTTON_MASK;
		}

		if (mbhc->micbias_enable) {
			if (mbhc->mbhc_cb->mbhc_micbias_control)
				mbhc->mbhc_cb->mbhc_micbias_control(
						component, MIC_BIAS_2,
						MICB_DISABLE);
			if (mbhc->mbhc_cb->mbhc_micb_ctrl_thr_mic)
				mbhc->mbhc_cb->mbhc_micb_ctrl_thr_mic(
						component,
						MIC_BIAS_2, false);
			if (mbhc->mbhc_cb->set_micbias_value) {
				mbhc->mbhc_cb->set_micbias_value(component);
				WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MICB_CTRL, 0);
			}
			mbhc->micbias_enable = false;
		}

		mbhc->hph_type = WCD_MBHC_HPH_NONE;
		mbhc->zl = mbhc->zr = 0;
#ifndef OPLUS_ARCH_EXTENDS
/* Modify for necessary log */
		pr_debug("%s: Reporting removal %d(%x)\n", __func__,
			 jack_type, mbhc->hph_status);
#else /* OPLUS_ARCH_EXTENDS */
		pr_info("%s: Reporting removal %d(%x)\n", __func__,
			 jack_type, mbhc->hph_status);
#endif /* OPLUS_ARCH_EXTENDS */

		wcd_mbhc_jack_report(mbhc, &mbhc->headset_jack,
				mbhc->hph_status, WCD_MBHC_JACK_MASK);
		wcd_mbhc_set_and_turnoff_hph_padac(mbhc);
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
		hphrocp_off_report(mbhc, SND_JACK_OC_HPHR);
		hphlocp_off_report(mbhc, SND_JACK_OC_HPHL);
#endif /* CONFIG_AUDIO_QGKI */
		mbhc->current_plug = MBHC_PLUG_TYPE_NONE;
		mbhc->force_linein = false;
	} else {
		/*
		 * Report removal of current jack type.
		 * Headphone to headset shouldn't report headphone
		 * removal.
		 */
		if (mbhc->mbhc_cfg->detect_extn_cable &&
		    (mbhc->current_plug == MBHC_PLUG_TYPE_HIGH_HPH ||
		    jack_type == SND_JACK_LINEOUT) &&
		    (mbhc->hph_status && mbhc->hph_status != jack_type)) {

			if (mbhc->micbias_enable &&
			    mbhc->hph_status == SND_JACK_HEADSET) {
				if (mbhc->mbhc_cb->mbhc_micbias_control)
					mbhc->mbhc_cb->mbhc_micbias_control(
						component, MIC_BIAS_2,
						MICB_DISABLE);
				if (mbhc->mbhc_cb->mbhc_micb_ctrl_thr_mic)
					mbhc->mbhc_cb->mbhc_micb_ctrl_thr_mic(
						component,
						MIC_BIAS_2, false);
				if (mbhc->mbhc_cb->set_micbias_value) {
					mbhc->mbhc_cb->set_micbias_value(
							component);
					WCD_MBHC_REG_UPDATE_BITS(
							WCD_MBHC_MICB_CTRL, 0);
				}
				mbhc->micbias_enable = false;
			}
			mbhc->hph_type = WCD_MBHC_HPH_NONE;
			mbhc->zl = mbhc->zr = 0;
			if (!mbhc->force_linein) {
				pr_debug("%s: Reporting removal (%x)\n",
					 __func__, mbhc->hph_status);
				wcd_mbhc_jack_report(mbhc, &mbhc->headset_jack,
					0, WCD_MBHC_JACK_MASK);
				if (mbhc->current_plug == MBHC_PLUG_TYPE_HEADPHONE)
					extdev_type = EXTCON_JACK_HEADPHONE;
				else if (mbhc->current_plug == MBHC_PLUG_TYPE_HEADSET)
					extdev_type = EXTCON_JACK_MICROPHONE;
				else if (mbhc->current_plug == MBHC_PLUG_TYPE_HIGH_HPH)
					extdev_type = EXTCON_JACK_LINE_OUT;
				else if (mbhc->current_plug == MBHC_PLUG_TYPE_GND_MIC_SWAP)
					extdev_type = EXTCON_MECHANICAL;

				extcon_set_state_sync(mbhc->extdev, extdev_type, 0);
			}
			if (mbhc->hph_status == SND_JACK_LINEOUT) {

				pr_debug("%s: Enable micbias\n", __func__);
				/* Disable current source and enable micbias */
				wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_MB);
				pr_debug("%s: set up elec removal detection\n",
					  __func__);
				usleep_range(200, 210);
				wcd_mbhc_hs_elec_irq(mbhc,
						     WCD_MBHC_ELEC_HS_REM,
						     true);
			}
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
			mbhc->hph_status &= ~(SND_JACK_HEADSET |
						SND_JACK_LINEOUT |
						SND_JACK_UNSUPPORTED);
#else
			mbhc->hph_status &= ~(SND_JACK_HEADSET |
						SND_JACK_LINEOUT);
#endif /* CONFIG_AUDIO_QGKI */
		}

		if (mbhc->current_plug == MBHC_PLUG_TYPE_HEADSET &&
			jack_type == SND_JACK_HEADPHONE)
			mbhc->hph_status &= ~SND_JACK_HEADSET;

		/* Report insertion */
		if (jack_type == SND_JACK_HEADPHONE)
			mbhc->current_plug = MBHC_PLUG_TYPE_HEADPHONE;
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
		else if (jack_type == SND_JACK_UNSUPPORTED)
			mbhc->current_plug = MBHC_PLUG_TYPE_GND_MIC_SWAP;
#endif /* CONFIG_AUDIO_QGKI */
		else if (jack_type == SND_JACK_HEADSET) {
			mbhc->current_plug = MBHC_PLUG_TYPE_HEADSET;
			mbhc->jiffies_atreport = jiffies;
		} else if (jack_type == SND_JACK_LINEOUT)
			mbhc->current_plug = MBHC_PLUG_TYPE_HIGH_HPH;
		else {
			pr_debug("%s: invalid Jack type %d\n",__func__, jack_type);
		}

#ifdef OPLUS_ARCH_EXTENDS
/* Add for headphone volume match to impedance */
		if (mbhc->enable_hp_impedance_detect) {
#endif /* OPLUS_ARCH_EXTENDS */
		if (mbhc->mbhc_cb->hph_pa_on_status)
			is_pa_on = mbhc->mbhc_cb->hph_pa_on_status(component);

		if (mbhc->impedance_detect &&
			mbhc->mbhc_cb->compute_impedance &&
			(mbhc->mbhc_cfg->linein_th != 0) &&
			(!is_pa_on)) {
			/* Set MUX_CTL to AUTO for Z-det */
			WCD_MBHC_REG_READ(WCD_MBHC_FSM_EN, fsm_en);
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_FSM_EN, 0);
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MUX_CTL,
						 MUX_CTL_AUTO);
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_FSM_EN, 1);
			mbhc->mbhc_cb->compute_impedance(mbhc,
					&mbhc->zl, &mbhc->zr);
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_FSM_EN,
						 fsm_en);
			if ((mbhc->zl > mbhc->mbhc_cfg->linein_th) &&
				(mbhc->zr > mbhc->mbhc_cfg->linein_th) &&
				(jack_type == SND_JACK_HEADPHONE)) {
				jack_type = SND_JACK_LINEOUT;
				mbhc->force_linein = true;
				mbhc->current_plug = MBHC_PLUG_TYPE_HIGH_HPH;
				if (mbhc->hph_status) {
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
					mbhc->hph_status &= ~(SND_JACK_HEADSET |
							SND_JACK_LINEOUT |
							SND_JACK_UNSUPPORTED);
#else
					mbhc->hph_status &= ~(SND_JACK_HEADSET |
							SND_JACK_LINEOUT);
#endif /* CONFIG_AUDIO_QGKI */
					wcd_mbhc_jack_report(mbhc,
							&mbhc->headset_jack,
							mbhc->hph_status,
							WCD_MBHC_JACK_MASK);
				}
				pr_debug("%s: Marking jack type as SND_JACK_LINEOUT\n",
				__func__);
			}
		}

		/* Do not calculate impedance again for lineout
		 * as during playback pa is on and impedance values
		 * will not be correct resulting in lineout detected
		 * as headphone.
		 */
		if ((is_pa_on) && mbhc->force_linein == true) {
			jack_type = SND_JACK_LINEOUT;
			mbhc->current_plug = MBHC_PLUG_TYPE_HIGH_HPH;
			if (mbhc->hph_status) {
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
				mbhc->hph_status &= ~(SND_JACK_HEADSET |
						SND_JACK_LINEOUT |
						SND_JACK_UNSUPPORTED);
#else
				mbhc->hph_status &= ~(SND_JACK_HEADSET |
						SND_JACK_LINEOUT);
#endif /* CONFIG_AUDIO_QGKI */
				wcd_mbhc_jack_report(mbhc,
						&mbhc->headset_jack,
						mbhc->hph_status,
						WCD_MBHC_JACK_MASK);
			}
		}
#ifdef OPLUS_ARCH_EXTENDS
/* Add for headphone volume match to impedance */
		}
#endif /* OPLUS_ARCH_EXTENDS */

		mbhc->hph_status |= jack_type;

		if (jack_type == SND_JACK_HEADPHONE &&
		    mbhc->mbhc_cb->mbhc_micb_ramp_control)
			mbhc->mbhc_cb->mbhc_micb_ramp_control(component, false);

#ifndef OPLUS_ARCH_EXTENDS
/* Modify for log */
		pr_debug("%s: Reporting insertion %d(%x)\n", __func__,
			 jack_type, mbhc->hph_status);
		wcd_mbhc_jack_report(mbhc, &mbhc->headset_jack,
				    (mbhc->hph_status | SND_JACK_MECHANICAL),
				    WCD_MBHC_JACK_MASK);
#else /* OPLUS_ARCH_EXTENDS */
		pr_info("%s: [1:headphone 3:headset 4:lineout]\n", __func__);
		pr_info("%s: Reporting insertion jack_type=%d, (hph_status=0x%x)\n",
			__func__, jack_type, mbhc->hph_status);
		if (jack_type != SND_JACK_LINEOUT) {
			wcd_mbhc_jack_report(mbhc, &mbhc->headset_jack,
				(mbhc->hph_status | SND_JACK_MECHANICAL),
				WCD_MBHC_JACK_MASK);
		}
#endif /* OPLUS_ARCH_EXTENDS */
		wcd_mbhc_clr_and_turnon_hph_padac(mbhc);
	}
	pr_debug("%s: leave hph_status %x\n", __func__, mbhc->hph_status);
}
EXPORT_SYMBOL(wcd_mbhc_report_plug);

void wcd_mbhc_elec_hs_report_unplug(struct wcd_mbhc *mbhc)
{
#ifdef OPLUS_ARCH_EXTENDS
/* Add for log */
	pr_info("%s: enter\n", __func__);
#endif /* OPLUS_ARCH_EXTENDS */

	/* cancel pending button press */
	if (wcd_cancel_btn_work(mbhc))
		pr_debug("%s: button press is canceled\n", __func__);
	/* cancel correct work function */
	if (mbhc->mbhc_fn->wcd_cancel_hs_detect_plug)
		mbhc->mbhc_fn->wcd_cancel_hs_detect_plug(mbhc,
						&mbhc->correct_plug_swch);
	else
		pr_info("%s: hs_detect_plug work not cancelled\n", __func__);

	pr_debug("%s: Report extension cable\n", __func__);
	wcd_mbhc_report_plug(mbhc, 1, SND_JACK_LINEOUT);
	extcon_set_state_sync(mbhc->extdev, EXTCON_JACK_LINE_OUT, 1);
	/*
	 * If PA is enabled HPHL schmitt trigger can
	 * be unreliable, make sure to disable it
	 */
	if (test_bit(WCD_MBHC_EVENT_PA_HPHL,
		&mbhc->event_state))
		wcd_mbhc_set_and_turnoff_hph_padac(mbhc);
	/*
	 * Disable HPHL trigger and MIC Schmitt triggers.
	 * Setup for insertion detection.
	 */
	wcd_mbhc_hs_elec_irq(mbhc, WCD_MBHC_ELEC_HS_REM,
			     false);
	wcd_enable_curr_micbias(mbhc, WCD_MBHC_EN_NONE);
	/* Disable HW FSM */
	WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_FSM_EN, 0);
	WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_ELECT_SCHMT_ISRC, 3);

	/* Set the detection type appropriately */
	WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_ELECT_DETECTION_TYPE, 1);
	wcd_mbhc_hs_elec_irq(mbhc, WCD_MBHC_ELEC_HS_INS,
			     true);
}
EXPORT_SYMBOL(wcd_mbhc_elec_hs_report_unplug);

void wcd_mbhc_find_plug_and_report(struct wcd_mbhc *mbhc,
				   enum wcd_mbhc_plug_type plug_type)
{
	bool anc_mic_found = false;
	enum snd_jack_types jack_type;
	int ret = 0;

	if (mbhc->deinit_in_progress) {
		pr_info("%s: mbhc deinit in progess: ignore report\n", __func__);
		return;
	}

#ifndef OPLUS_ARCH_EXTENDS
/* Modify for necessary log */
	pr_debug("%s: enter current_plug(%d) new_plug(%d)\n",
		 __func__, mbhc->current_plug, plug_type);
#else /* OPLUS_ARCH_EXTENDS */
	pr_info("%s: enter current_plug(%d) new_plug(%d)\n",
		 __func__, mbhc->current_plug, plug_type);
#endif /* OPLUS_ARCH_EXTENDS */

	WCD_MBHC_RSC_ASSERT_LOCKED(mbhc);

	if (mbhc->current_plug == plug_type) {
		pr_debug("%s: cable already reported, exit\n", __func__);
		goto exit;
	}

	if (plug_type == MBHC_PLUG_TYPE_HEADPHONE) {
		/*
		 * Nothing was reported previously
		 * report a headphone or unsupported
		 */
		wcd_mbhc_report_plug(mbhc, 1, SND_JACK_HEADPHONE);
		ret = extcon_set_state_sync(mbhc->extdev, EXTCON_JACK_HEADPHONE, 1);
	} else if (plug_type == MBHC_PLUG_TYPE_GND_MIC_SWAP) {
		if (mbhc->current_plug == MBHC_PLUG_TYPE_HEADPHONE) {
			wcd_mbhc_report_plug(mbhc, 0, SND_JACK_HEADPHONE);
			ret = extcon_set_state_sync(mbhc->extdev, EXTCON_JACK_HEADPHONE, 0);
		}
		if (mbhc->current_plug == MBHC_PLUG_TYPE_HEADSET) {
			wcd_mbhc_report_plug(mbhc, 0, SND_JACK_HEADSET);
			ret = extcon_set_state_sync(mbhc->extdev, EXTCON_JACK_MICROPHONE, 0);
		}
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
		wcd_mbhc_report_plug(mbhc, 1, SND_JACK_UNSUPPORTED);
#endif /* CONFIG_AUDIO_QGKI */
		ret = extcon_set_state_sync(mbhc->extdev, EXTCON_MECHANICAL, 1);
	} else if (plug_type == MBHC_PLUG_TYPE_HEADSET) {
		if (mbhc->mbhc_cfg->enable_anc_mic_detect &&
		    mbhc->mbhc_fn->wcd_mbhc_detect_anc_plug_type)
			anc_mic_found =
			mbhc->mbhc_fn->wcd_mbhc_detect_anc_plug_type(mbhc);
		jack_type = SND_JACK_HEADSET;

		/*
		 * If Headphone was reported previously, this will
		 * only report the mic line
		 */
		wcd_mbhc_report_plug(mbhc, 1, jack_type);
		ret = extcon_set_state_sync(mbhc->extdev, EXTCON_JACK_MICROPHONE, 1);
	} else if (plug_type == MBHC_PLUG_TYPE_HIGH_HPH) {
		if (mbhc->mbhc_cfg->detect_extn_cable) {
			/* High impedance device found. Report as LINEOUT */
			wcd_mbhc_report_plug(mbhc, 1, SND_JACK_LINEOUT);
			ret = extcon_set_state_sync(mbhc->extdev, EXTCON_JACK_LINE_OUT, 1);
			pr_debug("%s: setup mic trigger for further detection\n",
				 __func__);

			/* Disable HW FSM and current source */
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_FSM_EN, 0);
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_BTN_ISRC_CTL, 0);
			/* Setup for insertion detection */
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_ELECT_DETECTION_TYPE,
						 1);
			/*
			 * Enable HPHL trigger and MIC Schmitt triggers
			 * and request for elec insertion interrupts
			 */
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_ELECT_SCHMT_ISRC,
						 3);
			wcd_mbhc_hs_elec_irq(mbhc, WCD_MBHC_ELEC_HS_INS,
					     true);
		} else {
			wcd_mbhc_report_plug(mbhc, 1, SND_JACK_LINEOUT);
			ret = extcon_set_state_sync(mbhc->extdev, EXTCON_JACK_LINE_OUT, 1);
		}
	} else {
		WARN(1, "Unexpected current plug_type %d, plug_type %d\n",
		     mbhc->current_plug, plug_type);
	}
exit:
#ifndef OPLUS_ARCH_EXTENDS
/* Modify for necessary log */
	pr_debug("%s: leave\n", __func__);
#else /* OPLUS_ARCH_EXTENDS */
	pr_info("%s: leave\n", __func__);
#endif /* OPLUS_ARCH_EXTENDS */
}
EXPORT_SYMBOL(wcd_mbhc_find_plug_and_report);

static bool wcd_mbhc_moisture_detect(struct wcd_mbhc *mbhc, bool detection_type)
{
	bool ret = false;

	if (!mbhc->mbhc_cfg->moisture_en &&
	    !mbhc->mbhc_cfg->moisture_duty_cycle_en)
		return ret;

	if (!mbhc->mbhc_cb->mbhc_get_moisture_status ||
	    !mbhc->mbhc_cb->mbhc_moisture_polling_ctrl ||
	    !mbhc->mbhc_cb->mbhc_moisture_detect_en)
		return ret;

	if (mbhc->mbhc_cb->mbhc_get_moisture_status(mbhc)) {
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_L_DET_EN, 0);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_GND_DET_EN, 0);
		mbhc->mbhc_cb->mbhc_moisture_polling_ctrl(mbhc, true);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MECH_DETECTION_TYPE,
					detection_type);
		ret = true;
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_L_DET_EN, 1);
		if (mbhc->mbhc_cfg->gnd_det_en)
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_GND_DET_EN, 1);
	} else {
		mbhc->mbhc_cb->mbhc_moisture_polling_ctrl(mbhc, false);
		mbhc->mbhc_cb->mbhc_moisture_detect_en(mbhc, false);
	}

#ifdef OPLUS_ARCH_EXTENDS
/* Add for log */
	pr_info("%s: leave, ret=%d\n", __func__, ret);
#endif /* OPLUS_ARCH_EXTENDS */

	return ret;
}

static void wcd_mbhc_set_hsj_connect(struct wcd_mbhc *mbhc, bool connect)
{
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	struct snd_soc_component *component = mbhc->component;

	if (connect) {
		if (mbhc->mbhc_cb && mbhc->mbhc_cb->zdet_leakage_resistance)
			mbhc->mbhc_cb->zdet_leakage_resistance(mbhc, false); /* enable 1M pull-up */

		if (of_find_property(component->card->dev->of_node,
					"qcom,usbss-hsj-connect-enabled", NULL))
			wcd_usbss_switch_update(WCD_USBSS_HSJ_CONNECT, WCD_USBSS_CABLE_CONNECT);
	} else {
		if (of_find_property(component->card->dev->of_node,
					"qcom,usbss-hsj-connect-enabled", NULL))
			wcd_usbss_switch_update(WCD_USBSS_HSJ_CONNECT, WCD_USBSS_CABLE_DISCONNECT);

		if (mbhc->mbhc_cb && mbhc->mbhc_cb->zdet_leakage_resistance)
			mbhc->mbhc_cb->zdet_leakage_resistance(mbhc, true); /* disable 1M pull-up */
	}
#endif
}

#ifdef OPLUS_ARCH_EXTENDS
/* Add for log */
#undef pr_debug
#define pr_debug pr_info
#endif /* OPLUS_ARCH_EXTENDS */

static void wcd_mbhc_swch_irq_handler(struct wcd_mbhc *mbhc)
{
	bool detection_type = 0;
	bool micbias1 = false;
	struct snd_soc_component *component = mbhc->component;
	enum snd_jack_types jack_type;
	int extdev_type = 0;

#ifdef OPLUS_ARCH_EXTENDS
/* Add for micbias2 */
	bool micbias2 = false;
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
/* Add for Headset detect */
	if (!mbhc->mbhc_cfg->enable_usbc_analog) {
	    cancel_delayed_work_sync(&mbhc->hp_detect_work);
	}
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
/* Add for headphone remove issue */
	if (mbhc->irq_trigger_enable) {
		pr_info("%s: cancel mech_irq_trigger_dwork\n", __func__);
		cancel_delayed_work_sync(&mbhc->mech_irq_trigger_dwork);
	}
#endif /* OPLUS_ARCH_EXTENDS */

#ifndef OPLUS_ARCH_EXTENDS
/* Add for log */
	dev_dbg(component->dev, "%s: enter\n", __func__);
#else /* OPLUS_ARCH_EXTENDS */
	dev_info(component->dev, "%s: enter\n", __func__);
#endif /* OPLUS_ARCH_EXTENDS */
	WCD_MBHC_RSC_LOCK(mbhc);
	mbhc->in_swch_irq_handler = true;

	/* cancel pending button press */
	if (wcd_cancel_btn_work(mbhc))
		pr_debug("%s: button press is canceled\n", __func__);

	WCD_MBHC_REG_READ(WCD_MBHC_MECH_DETECTION_TYPE, detection_type);

#ifdef OPLUS_ARCH_EXTENDS
/* Fix the l_det status when plug out after insertion */
	if (mbhc->mbhc_cfg->enable_usbc_analog &&
		mbhc->usbc_l_det_en == false &&
		/* for gpio detect, donot close L_det */
		mbhc->headset_detect_mode != 1) {
		//WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MECH_DETECTION_TYPE, 1);
		if (detection_type) {
			dev_warn(mbhc->component->dev,
					"%s: L_DET is disabled, current_plug %d, ignored swch insert irq\n",
					__func__, mbhc->current_plug);
			goto done;
		} else {
			dev_warn(mbhc->component->dev,
					"%s: L_DET is disabled, current_plug %d, detection_type %d\n",
					__func__, mbhc->current_plug, detection_type);
		}
	}
#endif /* OPLUS_ARCH_EXTENDS */

	/* Set the detection type appropriately */
	WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MECH_DETECTION_TYPE,
				 !detection_type);

	pr_debug("%s: mbhc->current_plug: %d detection_type: %d\n", __func__,
			mbhc->current_plug, detection_type);
	if (mbhc->mbhc_fn->wcd_cancel_hs_detect_plug)
		mbhc->mbhc_fn->wcd_cancel_hs_detect_plug(mbhc,
						&mbhc->correct_plug_swch);
	else
		pr_info("%s: hs_detect_plug work not cancelled\n", __func__);

	/* Enable micbias ramp */
	if (mbhc->mbhc_cb->mbhc_micb_ramp_control)
		mbhc->mbhc_cb->mbhc_micb_ramp_control(component, true);

	if (mbhc->mbhc_cb->micbias_enable_status)
		micbias1 = mbhc->mbhc_cb->micbias_enable_status(mbhc,
						MIC_BIAS_1);

	if ((mbhc->current_plug == MBHC_PLUG_TYPE_NONE) &&
	    detection_type) {

		wcd_mbhc_set_hsj_connect(mbhc, 1);
		/* If moisture is present, then enable polling, disable
		 * moisture detection and wait for interrupt
		 */
		if (wcd_mbhc_moisture_detect(mbhc, detection_type))
			goto done;

		/* Make sure MASTER_BIAS_CTL is enabled */
		mbhc->mbhc_cb->mbhc_bias(component, true);

		if (mbhc->mbhc_cb->mbhc_common_micb_ctrl)
			mbhc->mbhc_cb->mbhc_common_micb_ctrl(component,
					MBHC_COMMON_MICB_TAIL_CURR, true);

		if (!mbhc->mbhc_cfg->hs_ext_micbias &&
		     mbhc->mbhc_cb->micb_internal)
			/*
			 * Enable Tx2 RBias if the headset
			 * is using internal micbias
			 */
			mbhc->mbhc_cb->micb_internal(component, 1, true);

		/* Remove micbias pulldown */
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_PULLDOWN_CTRL, 0);
		/* Apply trim if needed on the device */
		if (mbhc->mbhc_cb->trim_btn_reg)
			mbhc->mbhc_cb->trim_btn_reg(component);
		/* Enable external voltage source to micbias if present */
		if (mbhc->mbhc_cb->enable_mb_source)
			mbhc->mbhc_cb->enable_mb_source(mbhc, true);
		mbhc->btn_press_intr = false;
		mbhc->is_btn_press = false;
#ifndef OPLUS_ARCH_EXTENDS
/* Modify for headset detect */
		if (mbhc->mbhc_fn)
			mbhc->mbhc_fn->wcd_mbhc_detect_plug_type(mbhc);
#else /* OPLUS_ARCH_EXTENDS */
		if (mbhc->mbhc_fn){
			if (mbhc->mbhc_cfg->enable_usbc_analog) {
				mbhc->mbhc_fn->wcd_mbhc_detect_plug_type(mbhc);
			}else {
				schedule_delayed_work(&mbhc->hp_detect_work, msecs_to_jiffies(400));
			}
		}
#endif /* OPLUS_ARCH_EXTENDS */
	} else if ((mbhc->current_plug != MBHC_PLUG_TYPE_NONE)
			&& !detection_type) {
		/* Disable external voltage source to micbias if present */
		if (mbhc->mbhc_cb->enable_mb_source)
			mbhc->mbhc_cb->enable_mb_source(mbhc, false);
		/* Disable HW FSM */
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_FSM_EN, 0);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_BTN_ISRC_CTL, 0);
		if (mbhc->mbhc_cb->mbhc_common_micb_ctrl)
			mbhc->mbhc_cb->mbhc_common_micb_ctrl(component,
					MBHC_COMMON_MICB_TAIL_CURR, false);

		if (mbhc->mbhc_cb->set_cap_mode)
			mbhc->mbhc_cb->set_cap_mode(component, micbias1, false);

		mbhc->btn_press_intr = false;
		mbhc->is_btn_press = false;
		switch (mbhc->current_plug) {
		case MBHC_PLUG_TYPE_HEADPHONE:
			jack_type = SND_JACK_HEADPHONE;
			extdev_type = EXTCON_JACK_HEADPHONE;
			break;
		case MBHC_PLUG_TYPE_GND_MIC_SWAP:
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
			jack_type = SND_JACK_UNSUPPORTED;
#else
			jack_type = SND_JACK_HEADPHONE;
#endif /* CONFIG_AUDIO_QGKI */
			extdev_type = EXTCON_MECHANICAL;
			break;
		case MBHC_PLUG_TYPE_HEADSET:
			/* make sure to turn off Rbias */
			if (mbhc->mbhc_cb->micb_internal)
				mbhc->mbhc_cb->micb_internal(component,
							1, false);
			/* Pulldown micbias */
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_PULLDOWN_CTRL, 1);
			jack_type = SND_JACK_HEADSET;
			extdev_type = EXTCON_JACK_MICROPHONE;
			break;
		case MBHC_PLUG_TYPE_HIGH_HPH:
			if (mbhc->mbhc_detection_logic == WCD_DETECTION_ADC)
			    WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_ELECT_ISRC_EN, 0);
			mbhc->is_extn_cable = false;
			jack_type = SND_JACK_LINEOUT;
			extdev_type = EXTCON_JACK_LINE_OUT;
			break;
		default:
			pr_info("%s: Invalid current plug: %d\n",
				__func__, mbhc->current_plug);
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
			jack_type = SND_JACK_UNSUPPORTED;
#else
			jack_type = SND_JACK_HEADPHONE;
#endif /* CONFIG_AUDIO_QGKI */
			extdev_type = EXTCON_MECHANICAL;
			break;
		}
		wcd_mbhc_hs_elec_irq(mbhc, WCD_MBHC_ELEC_HS_REM, false);
		wcd_mbhc_hs_elec_irq(mbhc, WCD_MBHC_ELEC_HS_INS, false);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_ELECT_DETECTION_TYPE, 1);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_ELECT_SCHMT_ISRC, 0);
		mbhc->extn_cable_hph_rem = false;
		wcd_mbhc_report_plug(mbhc, 0, jack_type);
		extcon_set_state_sync(mbhc->extdev, extdev_type, 0);

#ifdef OPLUS_ARCH_EXTENDS
/*Add for force micbias2 disable after report plug out, reduce recording or voice-call noise.*/
		if (mbhc->mbhc_cb->micbias_enable_status) {
			micbias2 = mbhc->mbhc_cb->micbias_enable_status(mbhc, MIC_BIAS_2);
		}

		if (micbias2) {
			if (mbhc->mbhc_cb->mbhc_micbias_control) {
				//for wcd93xx codec
				pr_info("%s: *** micbias2 force disable! ***\n", __func__);
				WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MICB_CTRL, 0);
			}
		}
#endif /* OPLUS_ARCH_EXTENDS */

#ifndef OPLUS_ARCH_EXTENDS
		if (mbhc->mbhc_cfg->enable_usbc_analog) {
#else /* OPLUS_ARCH_EXTENDS */
		/* for gpio detect, donot close L_det */
		if (mbhc->mbhc_cfg->enable_usbc_analog && mbhc->headset_detect_mode != 1) {
#endif /* OPLUS_ARCH_EXTENDS */
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_L_DET_EN, 0);
			if (mbhc->mbhc_cb->clk_setup)
				mbhc->mbhc_cb->clk_setup(
					mbhc->component, false);
#ifdef OPLUS_ARCH_EXTENDS
/* Fix the l_det status when plug out after insertion */
			mbhc->usbc_l_det_en = false;
#endif /* OPLUS_ARCH_EXTENDS */
		}

		if (mbhc->mbhc_cfg->moisture_en ||
		    mbhc->mbhc_cfg->moisture_duty_cycle_en) {
			if (mbhc->mbhc_cb->mbhc_moisture_polling_ctrl)
				mbhc->mbhc_cb->mbhc_moisture_polling_ctrl(mbhc,
									false);
			if (mbhc->mbhc_cb->mbhc_moisture_detect_en)
				mbhc->mbhc_cb->mbhc_moisture_detect_en(mbhc,
									false);
		}
		wcd_mbhc_set_hsj_connect(mbhc, 0);

	} else if (!detection_type) {
#ifdef OPLUS_ARCH_EXTENDS
/*Add for disable micbias, when insert a HPH_HIGH device and detect as special
 *headset device(micbias_enable is true), then remove device before report
 *headset type, the micbias will remain enable.
 */
		if (mbhc->micbias_enable) {
			pr_info("%s: Need to disable MIC_BIAS_2\n", __func__);
			if (mbhc->mbhc_cb->mbhc_micbias_control)
				mbhc->mbhc_cb->mbhc_micbias_control(
						component, MIC_BIAS_2,
						MICB_DISABLE);
			if (mbhc->mbhc_cb->mbhc_micb_ctrl_thr_mic)
				mbhc->mbhc_cb->mbhc_micb_ctrl_thr_mic(
						component,
						MIC_BIAS_2, false);
			if (mbhc->mbhc_cb->set_micbias_value) {
				mbhc->mbhc_cb->set_micbias_value(component);
				WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MICB_CTRL, 0);
			}
			mbhc->micbias_enable = false;
		}
#endif /* OPLUS_ARCH_EXTENDS */

		/* Disable external voltage source to micbias if present */
		if (mbhc->mbhc_cb->enable_mb_source)
			mbhc->mbhc_cb->enable_mb_source(mbhc, false);
		/* Disable HW FSM */
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_FSM_EN, 0);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_BTN_ISRC_CTL, 0);
		mbhc->extn_cable_hph_rem = false;

#ifdef OPLUS_ARCH_EXTENDS
/* Fix the l_det status when plug out after insertion */
		if (mbhc->mbhc_cfg->enable_usbc_analog && mbhc->usbc_l_det_en == true &&
			mbhc->usbc_mode != TYPEC_ACCESSORY_AUDIO &&
			/* for gpio detect, donot close L_det */
			(mbhc->headset_detect_mode != 1)) {
			dev_info(mbhc->component->dev,
				"%s: usbc analog audio removed, disable L_DET\n", __func__);
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_L_DET_EN, 0);
			if (mbhc->mbhc_cb->clk_setup)
				mbhc->mbhc_cb->clk_setup(
					mbhc->component, false);
			mbhc->usbc_l_det_en = false;
		}
#endif /* OPLUS_ARCH_EXTENDS */
	}

done:
	mbhc->in_swch_irq_handler = false;
	WCD_MBHC_RSC_UNLOCK(mbhc);
	pr_debug("%s: leave\n", __func__);
}

static irqreturn_t wcd_mbhc_mech_plug_detect_irq(int irq, void *data)
{
	int r = IRQ_HANDLED;
	struct wcd_mbhc *mbhc = data;

	pr_debug("%s: enter\n", __func__);

	if (mbhc == NULL) {
		pr_err("%s: NULL irq data\n", __func__);
		return IRQ_NONE;
	}
	/* WCD USB AATC did not required mech plug detection, will receive
	 * insertion/removal events from UCSI layer
	 */
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	if (mbhc->mbhc_cfg->enable_usbc_analog) {
		pr_debug("%s: leave, (irq_none)", __func__);
		return IRQ_NONE;
	}
#endif

	if (unlikely((mbhc->mbhc_cb->lock_sleep(mbhc, true)) == false)) {
		pr_warn("%s: failed to hold suspend\n", __func__);
		r = IRQ_NONE;
	} else {
		/* Call handler */
		wcd_mbhc_swch_irq_handler(mbhc);
		mbhc->mbhc_cb->lock_sleep(mbhc, false);
	}
	pr_debug("%s: leave %d\n", __func__, r);
	return r;
}

int wcd_mbhc_get_button_mask(struct wcd_mbhc *mbhc)
{
	int mask = 0;
	int btn;

	btn = mbhc->mbhc_cb->map_btn_code_to_num(mbhc->component);

#ifdef OPLUS_ARCH_EXTENDS
/* Add for headset button log */
	pr_info("%s: btn is %d", __func__, btn);
#endif /* OPLUS_ARCH_EXTENDS */

	switch (btn) {
	case 0:
		mask = SND_JACK_BTN_0;
		break;
	case 1:
		mask = SND_JACK_BTN_1;
		break;
	case 2:
		mask = SND_JACK_BTN_2;
		break;
	case 3:
		mask = SND_JACK_BTN_3;
		break;
	case 4:
		mask = SND_JACK_BTN_4;
		break;
	case 5:
		mask = SND_JACK_BTN_5;
		break;
	default:
		break;
	}

	return mask;
}
EXPORT_SYMBOL(wcd_mbhc_get_button_mask);

static void wcd_btn_lpress_fn(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct wcd_mbhc *mbhc;
	s16 btn_result = 0;

	pr_debug("%s: Enter\n", __func__);

	dwork = to_delayed_work(work);
	mbhc = container_of(dwork, struct wcd_mbhc, mbhc_btn_dwork);

	WCD_MBHC_REG_READ(WCD_MBHC_BTN_RESULT, btn_result);
	if (mbhc->current_plug == MBHC_PLUG_TYPE_HEADSET) {
		pr_debug("%s: Reporting long button press event, btn_result: %d\n",
			 __func__, btn_result);
		wcd_mbhc_jack_report(mbhc, &mbhc->button_jack,
				mbhc->buttons_pressed, mbhc->buttons_pressed);
	}
	pr_debug("%s: leave\n", __func__);
	mbhc->mbhc_cb->lock_sleep(mbhc, false);
}

static bool wcd_mbhc_fw_validate(const void *data, size_t size)
{
	u32 cfg_offset;
	struct wcd_mbhc_btn_detect_cfg *btn_cfg;
	struct firmware_cal fw;

	fw.data = (void *)data;
	fw.size = size;

	if (fw.size < WCD_MBHC_CAL_MIN_SIZE)
		return false;

	/*
	 * Previous check guarantees that there is enough fw data up
	 * to num_btn
	 */
	btn_cfg = WCD_MBHC_CAL_BTN_DET_PTR(fw.data);
	cfg_offset = (u32) ((void *) btn_cfg - (void *) fw.data);
	if (fw.size < (cfg_offset + WCD_MBHC_CAL_BTN_SZ(btn_cfg)))
		return false;

	return true;
}

static irqreturn_t wcd_mbhc_btn_press_handler(int irq, void *data)
{
	struct wcd_mbhc *mbhc = data;
	int mask;
	unsigned long msec_val;

	pr_debug("%s: enter\n", __func__);
	complete(&mbhc->btn_press_compl);
	WCD_MBHC_RSC_LOCK(mbhc);
	wcd_cancel_btn_work(mbhc);
	if (wcd_swch_level_remove(mbhc)) {
		pr_debug("%s: Switch level is low ", __func__);
		goto done;
	}

	mbhc->is_btn_press = true;
	msec_val = jiffies_to_msecs(jiffies - mbhc->jiffies_atreport);
	pr_debug("%s: msec_val = %ld\n", __func__, msec_val);
	if (msec_val < MBHC_BUTTON_PRESS_THRESHOLD_MIN) {
		pr_debug("%s: Too short, ignore button press\n", __func__);
		goto done;
	}

	/* If switch interrupt already kicked in, ignore button press */
	if (mbhc->in_swch_irq_handler) {
		pr_debug("%s: Swtich level changed, ignore button press\n",
			 __func__);
		goto done;
	}
	mask = wcd_mbhc_get_button_mask(mbhc);
	if (mask == SND_JACK_BTN_0)
		mbhc->btn_press_intr = true;

	if (mbhc->current_plug != MBHC_PLUG_TYPE_HEADSET) {
		pr_debug("%s: Plug isn't headset, ignore button press\n",
				__func__);
		goto done;
	}
	mbhc->buttons_pressed |= mask;
	mbhc->mbhc_cb->lock_sleep(mbhc, true);
	if (schedule_delayed_work(&mbhc->mbhc_btn_dwork,
				msecs_to_jiffies(400)) == 0) {
		WARN(1, "Button pressed twice without release event\n");
		mbhc->mbhc_cb->lock_sleep(mbhc, false);
	}
done:
	pr_debug("%s: leave\n", __func__);
	WCD_MBHC_RSC_UNLOCK(mbhc);
	return IRQ_HANDLED;
}

static irqreturn_t wcd_mbhc_release_handler(int irq, void *data)
{
	struct wcd_mbhc *mbhc = data;
	int ret;

	pr_debug("%s: enter\n", __func__);
	WCD_MBHC_RSC_LOCK(mbhc);
	if (wcd_swch_level_remove(mbhc)) {
		pr_debug("%s: Switch level is low ", __func__);
		goto exit;
	}

	if (mbhc->is_btn_press) {
		mbhc->is_btn_press = false;
	} else {
		pr_debug("%s: This release is for fake btn press\n", __func__);
		goto exit;
	}

	/*
	 * If current plug is headphone then there is no chance to
	 * get btn release interrupt, so connected cable should be
	 * headset not headphone.
	 * For ADC MBHC, ADC_COMPLETE interrupt will be generated
	 * in this case. So skip the check here.
	 */
	if (mbhc->mbhc_detection_logic == WCD_DETECTION_LEGACY &&
		mbhc->current_plug == MBHC_PLUG_TYPE_HEADPHONE) {
#ifndef OPLUS_ARCH_EXTENDS
/* Delete for headset detect */
		wcd_mbhc_find_plug_and_report(mbhc, MBHC_PLUG_TYPE_HEADSET);
#endif /* OPLUS_ARCH_EXTENDS */
		goto exit;

	}
	if (mbhc->buttons_pressed & WCD_MBHC_JACK_BUTTON_MASK) {
		ret = wcd_cancel_btn_work(mbhc);
		if (ret == 0) {
			pr_debug("%s: Reporting long button release event\n",
				 __func__);
			wcd_mbhc_jack_report(mbhc, &mbhc->button_jack,
					0, mbhc->buttons_pressed);
		} else {
			if (mbhc->in_swch_irq_handler) {
				pr_debug("%s: Switch irq kicked in, ignore\n",
					__func__);
			} else {
				pr_debug("%s: Reporting btn press\n",
					 __func__);
				wcd_mbhc_jack_report(mbhc,
						     &mbhc->button_jack,
						     mbhc->buttons_pressed,
						     mbhc->buttons_pressed);
				pr_debug("%s: Reporting btn release\n",
					 __func__);
				wcd_mbhc_jack_report(mbhc,
						&mbhc->button_jack,
						0, mbhc->buttons_pressed);
			}
		}
		mbhc->buttons_pressed &= ~WCD_MBHC_JACK_BUTTON_MASK;
	}
exit:
	pr_debug("%s: leave\n", __func__);
	WCD_MBHC_RSC_UNLOCK(mbhc);
	return IRQ_HANDLED;
}

static irqreturn_t wcd_mbhc_hphl_ocp_irq(int irq, void *data)
{
	struct wcd_mbhc *mbhc = data;
	int val;

	pr_debug("%s: received HPHL OCP irq\n", __func__);
	if (mbhc) {
		if (mbhc->mbhc_cb->hph_register_recovery) {
			if (mbhc->mbhc_cb->hph_register_recovery(mbhc)) {
				WCD_MBHC_REG_READ(WCD_MBHC_HPHR_OCP_STATUS,
						  val);
				if ((val != -EINVAL) && val)
					mbhc->is_hph_ocp_pending = true;
				goto done;
			}
		}

		if (mbhc->hphlocp_cnt < OCP_ATTEMPT) {
			mbhc->hphlocp_cnt++;
			pr_debug("%s: retry, hphlocp_cnt: %d\n", __func__,
				 mbhc->hphlocp_cnt);
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_OCP_FSM_EN, 0);
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_OCP_FSM_EN, 1);
		} else {
			mbhc->mbhc_cb->irq_control(mbhc->component,
						   mbhc->intr_ids->hph_left_ocp,
						   false);
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
			mbhc->hph_status |= SND_JACK_OC_HPHL;
#endif /* CONFIG_AUDIO_QGKI */
			wcd_mbhc_jack_report(mbhc, &mbhc->headset_jack,
					    mbhc->hph_status,
					    WCD_MBHC_JACK_MASK);
		}
	} else {
		pr_err("%s: Bad wcd9xxx_spmi private data\n", __func__);
	}
done:
	return IRQ_HANDLED;
}

static irqreturn_t wcd_mbhc_hphr_ocp_irq(int irq, void *data)
{
	struct wcd_mbhc *mbhc = data;

	pr_debug("%s: received HPHR OCP irq\n", __func__);

	if (!mbhc) {
		pr_err("%s: Bad mbhc private data\n", __func__);
		goto done;
	}

	if (mbhc->is_hph_ocp_pending) {
		mbhc->is_hph_ocp_pending = false;
		goto done;
	}

	if (mbhc->mbhc_cb->hph_register_recovery) {
		if (mbhc->mbhc_cb->hph_register_recovery(mbhc))
			/* register corruption, hence reset registers */
			goto done;
	}
	if (mbhc->hphrocp_cnt < OCP_ATTEMPT) {
		mbhc->hphrocp_cnt++;
		pr_debug("%s: retry, hphrocp_cnt: %d\n", __func__,
			 mbhc->hphrocp_cnt);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_OCP_FSM_EN, 0);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_OCP_FSM_EN, 1);
	} else {
		mbhc->mbhc_cb->irq_control(mbhc->component,
					   mbhc->intr_ids->hph_right_ocp,
					   false);
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
		mbhc->hph_status |= SND_JACK_OC_HPHR;
#endif /* CONFIG_AUDIO_QGKI */
		wcd_mbhc_jack_report(mbhc, &mbhc->headset_jack,
				    mbhc->hph_status, WCD_MBHC_JACK_MASK);
	}
done:
	return IRQ_HANDLED;
}

#ifdef OPLUS_ARCH_EXTENDS
/* Add for log */
#undef pr_debug
#if defined(CONFIG_DYNAMIC_DEBUG) || \
	(defined(CONFIG_DYNAMIC_DEBUG_CORE) && defined(DYNAMIC_DEBUG_MODULE))
#include <linux/dynamic_debug.h>
#define pr_debug(fmt, ...)			\
	dynamic_pr_debug(fmt, ##__VA_ARGS__)
#elif defined(DEBUG)
#define pr_debug(fmt, ...) \
	printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_debug(fmt, ...) \
	no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif
#endif /* OPLUS_ARCH_EXTENDS */

static int wcd_mbhc_initialise(struct wcd_mbhc *mbhc)
{
	int ret = 0;
	struct snd_soc_component *component = mbhc->component;

	pr_debug("%s: enter\n", __func__);
	WCD_MBHC_RSC_LOCK(mbhc);

	/* enable HS detection */
	if (mbhc->mbhc_cb->hph_pull_up_control_v2)
		mbhc->mbhc_cb->hph_pull_up_control_v2(component,
						      HS_PULLUP_I_DEFAULT);
	else if (mbhc->mbhc_cb->hph_pull_up_control)
		mbhc->mbhc_cb->hph_pull_up_control(component, I_DEFAULT);
	else
#ifndef OPLUS_ARCH_EXTENDS
/* Modify for improve hp detect when enter water, disable DET internal
 * pull up(0xf150), use external hardware pull up.
 */
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_HS_L_DET_PULL_UP_CTRL, 3);
#else /* OPLUS_ARCH_EXTENDS */
		{
			pr_info("%s: default disable pull up for detection\n", __func__);
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_HS_L_DET_PULL_UP_CTRL, 0);
		}
#endif /* OPLUS_ARCH_EXTENDS */

	/* Configure for moisture detection when duty cycle is not enabled.
	 * Otherwise disable moisture detection.
	 */
	if (mbhc->mbhc_cfg->moisture_en && mbhc->mbhc_cb->mbhc_moisture_config
		&& !mbhc->mbhc_cfg->moisture_duty_cycle_en)
		mbhc->mbhc_cb->mbhc_moisture_config(mbhc);
	else if (mbhc->mbhc_cb->mbhc_moisture_detect_en)
		mbhc->mbhc_cb->mbhc_moisture_detect_en(mbhc, false);

	/*
	 * For USB analog we need to override the switch configuration.
	 * Also, disable hph_l pull-up current source as HS_DET_L is driven
	 * by an external source
	 */
	if (mbhc->mbhc_cfg->enable_usbc_analog) {
		if (mbhc->mbhc_cb->hph_pull_up_control_v2)
			mbhc->mbhc_cb->hph_pull_up_control_v2(component,
							      HS_PULLUP_I_OFF);
		else if (mbhc->mbhc_cb->hph_pull_up_control)
			mbhc->mbhc_cb->hph_pull_up_control(component, I_OFF);
		else
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_HS_L_DET_PULL_UP_CTRL,
						 0);
	}

	WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_HPHL_PLUG_TYPE, mbhc->hphl_swh);
	WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_GND_PLUG_TYPE, mbhc->gnd_swh);
	WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_SW_HPH_LP_100K_TO_GND, 1);
	if (mbhc->mbhc_cfg->gnd_det_en && mbhc->mbhc_cb->mbhc_gnd_det_ctrl)
		mbhc->mbhc_cb->mbhc_gnd_det_ctrl(component, true);
	WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_HS_L_DET_PULL_UP_COMP_CTRL, 1);

	/*
	 * Disable L_DET for USB-C analog audio to avoid spurious interrupts
	 * when a non-audio accessory is inserted. L_DET_EN sets to 1 when FSA
	 * I2C driver notifies that ANALOG_AUDIO_ADAPTER is inserted
	 */
#ifndef OPLUS_ARCH_EXTENDS
/* for gpio detect, donot close L_det */
	if (mbhc->mbhc_cfg->enable_usbc_analog)
#else /* OPLUS_ARCH_EXTENDS */
	if (mbhc->mbhc_cfg->enable_usbc_analog && mbhc->headset_detect_mode != 1)
#endif /* OPLUS_ARCH_EXTENDS */
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_L_DET_EN, 0);
	else
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_L_DET_EN, 1);

#ifdef OPLUS_ARCH_EXTENDS
/* Fix the l_det status when plug out after insertion */
	mbhc->usbc_l_det_en = false;
	mbhc->usbc_mode = TYPEC_ACCESSORY_NONE;
#endif /* OPLUS_ARCH_EXTENDS */

	if (mbhc->mbhc_cfg->enable_usbc_analog) {
		/* Insertion debounce set to 48ms */
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_INSREM_DBNC, 4);
	} else {
		/* Insertion debounce set to 96ms */
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_INSREM_DBNC, 6);
	}

	/* Button Debounce set to 16ms */
	WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_BTN_DBNC, 2);

	/* enable bias */
	mbhc->mbhc_cb->mbhc_bias(component, true);
	/* enable MBHC clock */
	if (mbhc->mbhc_cb->clk_setup) {
		if (mbhc->mbhc_cfg->enable_usbc_analog)
			mbhc->mbhc_cb->clk_setup(component, false);
		else
			mbhc->mbhc_cb->clk_setup(component, true);
	}

	/* program HS_VREF value */
	wcd_program_hs_vref(mbhc);

	wcd_program_btn_threshold(mbhc, false);


	reinit_completion(&mbhc->btn_press_compl);

	WCD_MBHC_RSC_UNLOCK(mbhc);
	pr_debug("%s: leave\n", __func__);
	return ret;
}

static void wcd_mbhc_fw_read(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct wcd_mbhc *mbhc;
	struct snd_soc_component *component;
	const struct firmware *fw;
	struct firmware_cal *fw_data = NULL;
	int ret = -1, retry = 0;
	bool use_default_cal = false;

	dwork = to_delayed_work(work);
	mbhc = container_of(dwork, struct wcd_mbhc, mbhc_firmware_dwork);
	component = mbhc->component;

	while (retry < FW_READ_ATTEMPTS) {
		retry++;
		pr_debug("%s:Attempt %d to request MBHC firmware\n",
			__func__, retry);
		if (mbhc->mbhc_cb->get_hwdep_fw_cal)
			fw_data = mbhc->mbhc_cb->get_hwdep_fw_cal(mbhc,
					WCD9XXX_MBHC_CAL);
		if (!fw_data)
			ret = request_firmware(&fw, "wcd9320/wcd9320_mbhc.bin",
				       component->dev);
		/*
		 * if request_firmware and hwdep cal both fail then
		 * sleep for 4sec for the userspace to send data to kernel
		 * retry for few times before bailing out
		 */
		if ((ret != 0) && !fw_data) {
			usleep_range(FW_READ_TIMEOUT, FW_READ_TIMEOUT +
					WCD_MBHC_USLEEP_RANGE_MARGIN_US);
		} else {
			pr_debug("%s: MBHC Firmware read successful\n",
					__func__);
			break;
		}
	}
	if (!fw_data)
		pr_debug("%s: using request_firmware\n", __func__);
	else
		pr_debug("%s: using hwdep cal\n", __func__);

	if (ret != 0 && !fw_data) {
		pr_err("%s: Cannot load MBHC firmware use default cal\n",
		       __func__);
		use_default_cal = true;
	}
	if (!use_default_cal) {
		const void *data;
		size_t size;

		if (fw_data) {
			data = fw_data->data;
			size = fw_data->size;
		} else {
			data = fw->data;
			size = fw->size;
		}
		if (wcd_mbhc_fw_validate(data, size) == false) {
			pr_err("%s: Invalid MBHC cal data size use default cal\n",
				__func__);
			if (!fw_data)
				release_firmware(fw);
		} else {
			if (fw_data) {
				mbhc->mbhc_cfg->calibration =
					(void *)fw_data->data;
				mbhc->mbhc_cal = fw_data;
			} else {
				mbhc->mbhc_cfg->calibration =
					(void *)fw->data;
				mbhc->mbhc_fw = fw;
			}
		}

	}

	(void) wcd_mbhc_initialise(mbhc);
}

static int wcd_mbhc_set_keycode(struct wcd_mbhc *mbhc)
{
	enum snd_jack_types type;
	int i, ret, result = 0;
	int *btn_key_code;

	btn_key_code = mbhc->mbhc_cfg->key_code;

	for (i = 0 ; i < WCD_MBHC_KEYCODE_NUM ; i++) {
		if (btn_key_code[i] != 0) {
			switch (i) {
			case 0:
				type = SND_JACK_BTN_0;
				break;
			case 1:
				type = SND_JACK_BTN_1;
				break;
			case 2:
				type = SND_JACK_BTN_2;
				break;
			case 3:
				type = SND_JACK_BTN_3;
				break;
			case 4:
				type = SND_JACK_BTN_4;
				break;
			case 5:
				type = SND_JACK_BTN_5;
				break;
			default:
				WARN_ONCE(1, "Wrong button number:%d\n", i);
				result = -1;
				return result;
			}
			ret = snd_jack_set_key(mbhc->button_jack.jack,
							type,
							btn_key_code[i]);
			if (ret) {
				pr_err("%s: Failed to set code for %d\n",
					__func__, btn_key_code[i]);
				result = -1;
				return result;
			}
			input_set_capability(
				mbhc->button_jack.jack->input_dev,
				EV_KEY, btn_key_code[i]);
			pr_debug("%s: set btn%d key code:%d\n", __func__,
				i, btn_key_code[i]);
		}
	}
	if (btn_key_code[0])
		mbhc->is_btn_already_regd = true;
	return result;
}

#ifdef OPLUS_ARCH_EXTENDS
/* Add for fix headset not correct after ssr */
static void wcd_mbhc_plug_fix_after_ssr(struct wcd_mbhc *mbhc)
{
	unsigned int l_det_en = 0;
	unsigned int detection_type = 0;

	if (!mbhc) {
		pr_info("%s: mbhc is NULL!\n", __func__);
		return;
	}

	pr_info("%s: plug_before_ssr=%d\n", __func__, mbhc->plug_before_ssr);
	if (mbhc->plug_before_ssr != MBHC_PLUG_TYPE_NONE) {
		pr_info("%s: current_plug=%d\n", __func__, mbhc->current_plug);
		if (mbhc->current_plug == MBHC_PLUG_TYPE_NONE) {
			WCD_MBHC_REG_READ(WCD_MBHC_MECH_DETECTION_TYPE, detection_type);
			WCD_MBHC_REG_READ(WCD_MBHC_L_DET_EN, l_det_en);
			pr_info("%s: detection_type=%d, l_det_en=%d\n", __func__, detection_type, l_det_en);
			/* If both l_det_en and detection type are set, it means device was
			 * unplugged during SSR and detection interrupt was not handled.
			 * So trigger device disconnect */
			if (detection_type && l_det_en) {
				/* Set current plug type to the state before SSR */
				mbhc->current_plug = mbhc->plug_before_ssr;

				pr_info("%s: trigger device disconnect!\n", __func__);
				wcd_mbhc_swch_irq_handler(mbhc);
				mbhc->mbhc_cb->lock_sleep(mbhc, false);
			}
		}

		//reset to none
		mbhc->plug_before_ssr = MBHC_PLUG_TYPE_NONE;
	}
}
#endif /* OPLUS_ARCH_EXTENDS */

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
#define WCD_CHECK_PLUG_IN_IRQ_DELAY    5000//ms
#define WCD_CHECK_PLUG_OUT_IRQ_DELAY    2000//ms

static void wcd_check_plug_irq_fn(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct wcd_mbhc *mbhc;
	char buf[MM_KEVENT_MAX_PAYLOAD_SIZE] = {0};

	dwork = to_delayed_work(work);
	mbhc = container_of(dwork, struct wcd_mbhc, hp_irq_chk_work);

	if (mbhc && mbhc->mbhc_cfg && mbhc->mbhc_cfg->enable_usbc_analog) {
		pr_debug("%s: mode = %lu, hph_status=%d\n", __func__, mbhc->usbc_mode, mbhc->hph_status);

		if (((mbhc->usbc_mode == TYPEC_ACCESSORY_AUDIO) && (mbhc->hph_status == 0)) || \
				((mbhc->usbc_mode == TYPEC_ACCESSORY_NONE) && (mbhc->hph_status !=0))) {

			scnprintf(buf, sizeof(buf) - 1, "func@@%s$$typec_mode@@%lu$$hph_status@@%d", \
					__func__, mbhc->usbc_mode, mbhc->hph_status);
			upload_mm_fb_kevent_to_atlas_limit(OPLUS_AUDIO_EVENTID_HEADSET_DET, buf, MM_FB_KEY_RATELIMIT_1MIN);
		}
	}
}
#endif /* OPLUS_FEATURE_MM_FEEDBACK */

#if IS_ENABLED(CONFIG_QCOM_FSA4480_I2C) || IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
#ifdef OPLUS_ARCH_EXTENDS
/* Fix the l_det status when plug out after insertion */
static void wcd_mbhc_usbc_ana_remove_handler(struct wcd_mbhc *mbhc)
{
	bool detection_type = 0;

	WCD_MBHC_RSC_LOCK(mbhc);
	if (mbhc->usbc_l_det_en == true && mbhc->in_swch_irq_handler != true) {
		WCD_MBHC_REG_READ(WCD_MBHC_MECH_DETECTION_TYPE, detection_type);
		dev_warn(mbhc->component->dev, "%s: detection_type %d\n", __func__, detection_type);
		// if detection_type is true, it means the swch insert irq is not triggered
		// or not handled, so we can disable l_det here, and if the insert irq is trigged
		// but not handled, we will ignored the irq in the irq handler function.
		if (detection_type) {
			dev_warn(mbhc->component->dev,
				"%s: usbc analog audio removed, force disable L_DET\n",	__func__);
			WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_L_DET_EN, 0);
			//WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_MECH_DETECTION_TYPE, 1);
			if (mbhc->mbhc_cb->clk_setup)
				mbhc->mbhc_cb->clk_setup(mbhc->component, false);
			mbhc->usbc_l_det_en = false;
		}
	}
	WCD_MBHC_RSC_UNLOCK(mbhc);
}
#endif /* OPLUS_ARCH_EXTENDS */

static int wcd_mbhc_usbc_ana_event_handler(struct notifier_block *nb,
					   unsigned long mode, void *ptr)
{
	struct wcd_mbhc *mbhc = container_of(nb, struct wcd_mbhc, aatc_dev_nb);
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	int l_det_en = 0, detection_type = 0;
	bool *cable_status = (bool*) ptr;
#endif

	if (!mbhc)
		return -EINVAL;

#ifdef OPLUS_ARCH_EXTENDS
	dev_info(mbhc->component->dev, "%s: mode = %lu\n", __func__, mode);
	/* Fix the l_det status when plug out after insertion */
	mbhc->usbc_mode = mode;
#endif /* OPLUS_ARCH_EXTENDS */

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
	if (mbhc->mbhc_cfg && mbhc->hp_wake_lock && !(mbhc->fb_ctl & BYPASS_HEADPHONE_FEEDBACK_10009)) {
		cancel_delayed_work_sync(&mbhc->hp_irq_chk_work);
		if (mode == TYPEC_ACCESSORY_AUDIO) {
			__pm_wakeup_event(mbhc->hp_wake_lock, msecs_to_jiffies(WCD_CHECK_PLUG_IN_IRQ_DELAY + 10));
			schedule_delayed_work(&mbhc->hp_irq_chk_work, msecs_to_jiffies(WCD_CHECK_PLUG_IN_IRQ_DELAY));
		} else if (mbhc->usbc_mode == TYPEC_ACCESSORY_NONE) {
			__pm_wakeup_event(mbhc->hp_wake_lock, msecs_to_jiffies(WCD_CHECK_PLUG_OUT_IRQ_DELAY + 10));
			schedule_delayed_work(&mbhc->hp_irq_chk_work, msecs_to_jiffies(WCD_CHECK_PLUG_OUT_IRQ_DELAY));
		}
	}
#endif /* OPLUS_FEATURE_MM_FEEDBACK */

	if (mode == TYPEC_ACCESSORY_AUDIO) {
		dev_dbg(mbhc->component->dev, "enter, %s: mode = %lu\n", __func__, mode);
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
		if (cable_status == NULL)
			wcd_usbss_switch_update(WCD_USBSS_AATC, WCD_USBSS_CABLE_CONNECT);
		else {
			if (*cable_status == false)
				wcd_usbss_switch_update(WCD_USBSS_AATC, WCD_USBSS_CABLE_CONNECT);
			else
				dev_dbg(mbhc->component->dev, "skip AATC switch settings, cable_status= %d",
						*cable_status);
		}
#endif
		if (mbhc->mbhc_cb->clk_setup)
			mbhc->mbhc_cb->clk_setup(mbhc->component, true);
		WCD_MBHC_REG_UPDATE_BITS(WCD_MBHC_L_DET_EN, 1);
#ifdef OPLUS_ARCH_EXTENDS
/* Fix the l_det status when plug out after insertion */
		mbhc->usbc_l_det_en = true;
#endif /* OPLUS_ARCH_EXTENDS */

#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
		if (unlikely((mbhc->mbhc_cb->lock_sleep(mbhc, true)) == false))
			pr_warn("%s: failed to hold suspend\n", __func__);
		else {
            if (mbhc->current_plug == MBHC_PLUG_TYPE_NONE)
                wcd_mbhc_swch_irq_handler(mbhc);
			mbhc->mbhc_cb->lock_sleep(mbhc, false);
		}
#endif
	} else if (mode < TYPEC_MAX_ACCESSORY) {
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
		WCD_MBHC_REG_READ(WCD_MBHC_L_DET_EN, l_det_en);
		WCD_MBHC_REG_READ(WCD_MBHC_MECH_DETECTION_TYPE, detection_type);
		if ((mode == TYPEC_ACCESSORY_NONE) && !detection_type) {
			if (unlikely((mbhc->mbhc_cb->lock_sleep(mbhc, true)) == false))
				pr_warn("%s: failed to hold suspend\n", __func__);
			else {
				wcd_mbhc_swch_irq_handler(mbhc);
				mbhc->mbhc_cb->lock_sleep(mbhc, false);
			}
			wcd_usbss_switch_update(WCD_USBSS_AATC, WCD_USBSS_CABLE_DISCONNECT);
			dev_dbg(mbhc->component->dev, "leave, %s: mode = %lu\n", __func__, mode);
		}
#endif

#ifdef OPLUS_ARCH_EXTENDS
/* Add for fix headset not correct after ssr */
		wcd_mbhc_plug_fix_after_ssr(mbhc);
		/* for gpio detect, donot close L_det */
		if (mbhc->headset_detect_mode != 1) {
			/* Fix the l_det status when plug out after insertion */
			wcd_mbhc_usbc_ana_remove_handler(mbhc);
		}
#endif /* OPLUS_ARCH_EXTENDS */
	} else if (mode == TYPEC_MAX_ACCESSORY) {
		if (mbhc->mbhc_cb->surge_reset_routine)
			mbhc->mbhc_cb->surge_reset_routine(mbhc);
	}
	return 0;
}
#else
static int wcd_mbhc_usbc_ana_event_handler(struct notifier_block *nb,
					   unsigned long mode, void *ptr)
{
	pr_info("%s: mode = %lu, handler not implemented\n", __func__, mode);
	return 0;
}
#endif

int wcd_mbhc_start(struct wcd_mbhc *mbhc, struct wcd_mbhc_config *mbhc_cfg)
{
	int rc = 0;
	struct snd_soc_component *component;
	struct snd_soc_card *card;
	const char *usb_c_dt = "qcom,msm-mbhc-usbc-audio-supported";
#ifdef OPLUS_ARCH_EXTENDS
/* Add for redefine config */
	const char *detect_extn_cable_dt = "oplus,mbhc-detect-extn-cable";
	u32 detect_extn_cable = 0;
	const char *moisture_en_dt = "oplus,mbhc-moisture-en";
	u32 moisture_en = 0;
	const char *moisture_duty_cycle_en_dt = "oplus,mbhc-moisture-duty-cycle-en";
	u32 moisture_duty_cycle_en = 0;
#endif /* OPLUS_ARCH_EXTENDS */

	if (!mbhc || !mbhc_cfg)
		return -EINVAL;

	component = mbhc->component;
	card = component->card;

	/* update the mbhc config */
	mbhc->mbhc_cfg = mbhc_cfg;

	dev_dbg(mbhc->component->dev, "%s: enter\n", __func__);

	/* check if USB C analog is defined on device tree */
	mbhc_cfg->enable_usbc_analog = 0;
	if (of_find_property(card->dev->of_node, usb_c_dt, NULL)) {
		rc = of_property_read_u32(card->dev->of_node, usb_c_dt,
				&mbhc_cfg->enable_usbc_analog);
	}
	if (mbhc_cfg->enable_usbc_analog == 0 || rc != 0) {
		dev_dbg(card->dev,
				"%s: %s in dt node is missing or false\n",
				__func__, usb_c_dt);
		dev_dbg(card->dev,
			"%s: skipping USB c analog configuration\n", __func__);
	}

	/* Parse wcd_usbss/fsa switch handle */
	if (mbhc_cfg->enable_usbc_analog) {
		dev_dbg(mbhc->component->dev, "%s: usbc analog enabled\n",
					__func__);
		mbhc->swap_thr = GND_MIC_USBC_SWAP_THRESHOLD;
		if (IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C))
			mbhc->aatc_dev_np = of_parse_phandle(card->dev->of_node,
							"wcd939x-i2c-handle", 0);
		else if (IS_ENABLED(CONFIG_QCOM_FSA4480_I2C))
			mbhc->aatc_dev_np = of_parse_phandle(card->dev->of_node,
							"fsa4480-i2c-handle", 0);
		if (!mbhc->aatc_dev_np) {
			dev_err(card->dev, "%s: wcd939x or fsa i2c node not found\n",
									__func__);
			rc = -EINVAL;
			goto err;
		}
	}

	/* Disable moisture detect and duty cycle for WCD USB AATC HS*/
	if (mbhc_cfg->enable_usbc_analog) {
#ifdef OPLUS_ARCH_EXTENDS
/* Add for redefine config */
		mbhc_cfg->detect_extn_cable = false;
		if (of_find_property(card->dev->of_node, detect_extn_cable_dt, NULL)) {
			rc = of_property_read_u32(card->dev->of_node, detect_extn_cable_dt,
					&detect_extn_cable);
			if (rc == 0) {
				pr_info("%s: detect_extn_cable %d\n", __func__, detect_extn_cable);
				if (detect_extn_cable) {
					mbhc_cfg->detect_extn_cable = true;
				}
			}
		}
#endif /* OPLUS_ARCH_EXTENDS */

		mbhc_cfg->moisture_en = false;
#ifdef OPLUS_ARCH_EXTENDS
/* Add for moisture_en config */
		if (of_find_property(card->dev->of_node, moisture_en_dt, NULL)) {
			rc = of_property_read_u32(card->dev->of_node, moisture_en_dt,
					&moisture_en);
			if (rc == 0) {
				pr_info("%s: moisture_en %d\n", __func__, moisture_en);
				if (moisture_en) {
					mbhc_cfg->moisture_en = true;
				}
			}
		}
#endif /* OPLUS_ARCH_EXTENDS */

		mbhc_cfg->moisture_duty_cycle_en = false;
#ifdef OPLUS_ARCH_EXTENDS
/* Add for moisture_en config */
		if (of_find_property(card->dev->of_node, moisture_duty_cycle_en_dt, NULL)) {
			rc = of_property_read_u32(card->dev->of_node, moisture_duty_cycle_en_dt,
					&moisture_duty_cycle_en);
			if (rc == 0) {
				pr_info("%s: moisture_duty_cycle_en %d\n", __func__, moisture_duty_cycle_en);
				if (moisture_duty_cycle_en) {
					mbhc_cfg->moisture_duty_cycle_en = true;
				}
			}
		}
#endif /* OPLUS_ARCH_EXTENDS */
		pr_debug("%s: Disable moisture detect and duty cycle of AATC",
			__func__);
	}

	/* Set btn key code */
	if ((!mbhc->is_btn_already_regd) && wcd_mbhc_set_keycode(mbhc))
		pr_err("Set btn key code error!!!\n");

	if (!mbhc->mbhc_cfg->read_fw_bin ||
	    (mbhc->mbhc_cfg->read_fw_bin && mbhc->mbhc_fw) ||
	    (mbhc->mbhc_cfg->read_fw_bin && mbhc->mbhc_cal)) {
		rc = wcd_mbhc_initialise(mbhc);
		if (rc) {
			dev_err(card->dev, "%s: wcd mbhc initialize failed\n",
				__func__);
			goto err;
		}
	} else {
		if (!mbhc->mbhc_fw || !mbhc->mbhc_cal)
			schedule_delayed_work(&mbhc->mbhc_firmware_dwork,
				      usecs_to_jiffies(FW_READ_TIMEOUT));
		else
			pr_err("%s: Skipping to read mbhc fw, 0x%pK %pK\n",
				 __func__, mbhc->mbhc_fw, mbhc->mbhc_cal);
	}

	if (mbhc_cfg->enable_usbc_analog) {
		mbhc->aatc_dev_nb.notifier_call = wcd_mbhc_usbc_ana_event_handler;
		mbhc->aatc_dev_nb.priority = 0;
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
		rc = wcd_usbss_reg_notifier(&mbhc->aatc_dev_nb, mbhc->aatc_dev_np);
#ifdef OPLUS_ARCH_EXTENDS
/* decouple mbhc register from wcd usbss state */
		if (rc) {
			pr_info("%s wcd_usbss_reg_notifier fail, rc = %d", __func__, rc);
			rc = 0;
		}
#endif /* OPLUS_ARCH_EXTENDS */
#endif
#if IS_ENABLED(CONFIG_QCOM_FSA4480_I2C)
		rc = fsa4480_reg_notifier(&mbhc->aatc_dev_nb, mbhc->aatc_dev_np);
#ifdef OPLUS_ARCH_EXTENDS
/* decouple mbhc register from fsa4480 state */
		if (rc) {
			pr_info("%s fsa4480_reg_notifier fail, rc = %d", __func__, rc);
			rc = 0;
		}
#endif /* OPLUS_ARCH_EXTENDS */
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
		mbhc->hp_wake_lock = wakeup_source_register(NULL, "hp_wake_lock");
		if (!mbhc->hp_wake_lock) {
			pr_err("%s: wakeup_source_register failed\n", __func__);
		}
		INIT_DELAYED_WORK(&mbhc->hp_irq_chk_work, wcd_check_plug_irq_fn);
		mbhc->fb_ctl = 0;
#endif /* OPLUS_FEATURE_MM_FEEDBACK */
	}

#ifdef OPLUS_ARCH_EXTENDS
/* Add for fix headset not correct after ssr */
	if (!mbhc_cfg->enable_usbc_analog) {
		wcd_mbhc_plug_fix_after_ssr(mbhc);
	}
#endif /* OPLUS_ARCH_EXTENDS */

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
	pr_info("%s: event_id=%u, version:%s\n", __func__, \
			OPLUS_AUDIO_EVENTID_HEADSET_DET, HEADSET_ERR_FB_VERSION);
#endif /* CONFIG_OPLUS_FEATURE_MM_FEEDBACK */

	return rc;
err:
	dev_dbg(mbhc->component->dev, "%s: leave %d\n", __func__, rc);
	return rc;
}
EXPORT_SYMBOL(wcd_mbhc_start);

void wcd_mbhc_stop(struct wcd_mbhc *mbhc)
{
	pr_debug("%s: enter\n", __func__);

	if (mbhc->current_plug != MBHC_PLUG_TYPE_NONE) {
		if (mbhc->mbhc_cb && mbhc->mbhc_cb->skip_imped_detect)
			mbhc->mbhc_cb->skip_imped_detect(mbhc->component);
	}
	mbhc->current_plug = MBHC_PLUG_TYPE_NONE;
	mbhc->hph_status = 0;
	if (mbhc->mbhc_cb && mbhc->mbhc_cb->irq_control) {
		mbhc->mbhc_cb->irq_control(mbhc->component,
				mbhc->intr_ids->hph_left_ocp,
				false);
		mbhc->mbhc_cb->irq_control(mbhc->component,
				mbhc->intr_ids->hph_right_ocp,
				false);
	}
	if (mbhc->mbhc_fw || mbhc->mbhc_cal) {
		cancel_delayed_work_sync(&mbhc->mbhc_firmware_dwork);
		if (!mbhc->mbhc_cal)
			release_firmware(mbhc->mbhc_fw);
		mbhc->mbhc_fw = NULL;
		mbhc->mbhc_cal = NULL;
	}

#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	if (mbhc->mbhc_cfg->enable_usbc_analog)
		wcd_usbss_unreg_notifier(&mbhc->aatc_dev_nb, mbhc->aatc_dev_np);
#endif
#if IS_ENABLED(CONFIG_QCOM_FSA4480_I2C)
	if (mbhc->mbhc_cfg->enable_usbc_analog)
		fsa4480_unreg_notifier(&mbhc->aatc_dev_nb, mbhc->aatc_dev_np);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
	if (mbhc->mbhc_cfg->enable_usbc_analog) {
		cancel_delayed_work_sync(&mbhc->hp_irq_chk_work);
		if (mbhc->hp_wake_lock) {
			wakeup_source_unregister(mbhc->hp_wake_lock);
			mbhc->hp_wake_lock = NULL;
		}
	}
#endif /* CONFIG_OPLUS_FEATURE_MM_FEEDBACK */

	pr_debug("%s: leave\n", __func__);
}
EXPORT_SYMBOL(wcd_mbhc_stop);

/*
 * wcd_mbhc_init : initialize MBHC internal structures.
 *
 * NOTE: mbhc->mbhc_cfg is not YET configure so shouldn't be used
 */
int wcd_mbhc_init(struct wcd_mbhc *mbhc, struct snd_soc_component *component,
		      const struct wcd_mbhc_cb *mbhc_cb,
		      const struct wcd_mbhc_intr *mbhc_cdc_intr_ids,
		      struct wcd_mbhc_register *wcd_mbhc_regs,
		      bool impedance_det_en)
{
	int ret = 0;
	int hph_swh = 0;
	int gnd_swh = 0;
	u32 hph_moist_config[3];
	struct snd_soc_card *card = component->card;
	const char *hph_switch = "qcom,msm-mbhc-hphl-swh";
	const char *gnd_switch = "qcom,msm-mbhc-gnd-swh";
	const char *hs_thre = "qcom,msm-mbhc-hs-mic-max-threshold-mv";
	const char *hph_thre = "qcom,msm-mbhc-hs-mic-min-threshold-mv";
#ifdef OPLUS_ARCH_EXTENDS
/* Add for mbhc cross connection */
	u32 cross_conn = 0;
	const char *mbhc_cross_conn = "oplus,mbhc-check-cross-conn";
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
/* Add for headphone remove issue */
	u32 check_irq_en = 0;
	const char *mbhc_check_irq_en = "oplus,mbhc-check-irq-en";
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
/* workaround to fix headset recording pop noise */
	u32 headset_micbias_alwayon = 0;
	const char *mbhc_headset_micbias_alwayon = "oplus,mbhc-headset-micbias-alwayon";
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
/* for gpio detect, donot close L_det */
	u32 detect_mode = 0;
	const char *mbhc_headset_detect_mode = "oplus,mbhc-headset-detect-mode";
#endif /* OPLUS_ARCH_EXTENDS */

	pr_debug("%s: enter\n", __func__);

	ret = of_property_read_u32(card->dev->of_node, hph_switch, &hph_swh);
	if (ret) {
		dev_err(card->dev,
			"%s: missing %s in dt node\n", __func__, hph_switch);
		goto err;
	}

	ret = of_property_read_u32(card->dev->of_node, gnd_switch, &gnd_swh);
	if (ret) {
		dev_err(card->dev,
			"%s: missing %s in dt node\n", __func__, gnd_switch);
		goto err;
	}

	ret = of_property_read_u32(card->dev->of_node, hs_thre,
				&(mbhc->hs_thr));
	if (ret)
		dev_dbg(card->dev,
			"%s: missing %s in dt node\n", __func__, hs_thre);

	ret = of_property_read_u32(card->dev->of_node, hph_thre,
				&(mbhc->hph_thr));
	if (ret)
		dev_dbg(card->dev,
			"%s: missing %s in dt node\n", __func__, hph_thre);

	ret = of_property_read_u32_array(card->dev->of_node,
					 "qcom,msm-mbhc-moist-cfg",
					 hph_moist_config, 3);
	if (ret) {
		dev_dbg(card->dev, "%s: no qcom,msm-mbhc-moist-cfg in DT\n",
			__func__);
		mbhc->moist_vref = V_45_MV;
		mbhc->moist_iref = I_3P0_UA;
		mbhc->moist_rref = R_24_KOHM;
	} else {
		mbhc->moist_vref = hph_moist_config[0];
		mbhc->moist_iref = hph_moist_config[1];
		mbhc->moist_rref = hph_moist_config[2];
	}

#ifdef OPLUS_ARCH_EXTENDS
/* Add for mbhc cross connection */
	ret = of_property_read_u32(card->dev->of_node, mbhc_cross_conn,
				&cross_conn);
	if (ret) {
		dev_info(card->dev,
			"%s: missing %s in dt node\n", __func__, mbhc_cross_conn);
		mbhc->need_cross_conn = false;
	} else {
		dev_info(card->dev, "%s: cross_conn %d\n", __func__, cross_conn);
		if (cross_conn) {
			mbhc->need_cross_conn = true;
		} else {
			mbhc->need_cross_conn = false;
		}
	}
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
/* Add for headphone remove issue */
	ret = of_property_read_u32(card->dev->of_node, mbhc_check_irq_en,
				&check_irq_en);
	if (ret) {
		dev_info(card->dev,
			"%s: missing %s in dt node\n", __func__, mbhc_check_irq_en);
		mbhc->irq_trigger_enable = false;
	} else {
		dev_info(card->dev, "%s: irq_trigger_enable %d\n", __func__, check_irq_en);
		if (check_irq_en) {
			mbhc->irq_trigger_enable= true;
		} else {
			mbhc->irq_trigger_enable = false;
		}
	}
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
/* workaround to fix headset recording pop noise */
	ret = of_property_read_u32(card->dev->of_node, mbhc_headset_micbias_alwayon,
				&headset_micbias_alwayon);
	if (ret) {
		dev_info(card->dev,
			"%s: missing %s in dt node\n", __func__, mbhc_headset_micbias_alwayon);
		mbhc->headset_micbias_alwayon = false;
	} else {
		dev_info(card->dev, "%s: headset_micbias_alwayon %d\n", __func__, headset_micbias_alwayon);
		if (headset_micbias_alwayon) {
			mbhc->headset_micbias_alwayon= true;
		} else {
			mbhc->headset_micbias_alwayon = false;
		}
	}
#endif /* OPLUS_ARCH_EXTENDS */

#ifdef OPLUS_ARCH_EXTENDS
/* for gpio detect, donot close L_det */
	ret = of_property_read_u32(card->dev->of_node, mbhc_headset_detect_mode,
				&detect_mode);
	if (ret) {
		dev_info(card->dev,
			"%s: missing %s in dt node\n", __func__, mbhc_headset_detect_mode);
		mbhc->headset_detect_mode = 0;
	} else {
		dev_info(card->dev, "%s: detect_mode %d\n", __func__, detect_mode);
		if (detect_mode) {
			mbhc->headset_detect_mode = 1;
		} else {
			mbhc->headset_detect_mode = 0;
		}
	}
#endif /* OPLUS_ARCH_EXTENDS */

	mbhc->in_swch_irq_handler = false;
	mbhc->current_plug = MBHC_PLUG_TYPE_NONE;
	mbhc->is_btn_press = false;
	mbhc->component = component;
	mbhc->intr_ids = mbhc_cdc_intr_ids;
	mbhc->impedance_detect = impedance_det_en;
	mbhc->hphl_swh = hph_swh;
	mbhc->gnd_swh = gnd_swh;
	mbhc->micbias_enable = false;
	mbhc->mbhc_cb = mbhc_cb;
	mbhc->btn_press_intr = false;
	mbhc->is_hs_recording = false;
	mbhc->is_extn_cable = false;
	mbhc->extn_cable_hph_rem = false;
	mbhc->hph_type = WCD_MBHC_HPH_NONE;
	mbhc->wcd_mbhc_regs = wcd_mbhc_regs;
	mbhc->swap_thr = GND_MIC_SWAP_THRESHOLD;
	mbhc->hphl_cross_conn_thr = HPHL_CROSS_CONN_THRESHOLD;
	mbhc->hphr_cross_conn_thr = HPHR_CROSS_CONN_THRESHOLD;

	if (mbhc->intr_ids == NULL) {
		pr_err("%s: Interrupt mapping not provided\n", __func__);
		return -EINVAL;
	}
	if (!mbhc->wcd_mbhc_regs) {
		dev_err(component->dev, "%s: mbhc registers are not defined\n",
			__func__);
		return -EINVAL;
	}

	/* Check if IRQ and other required callbacks are defined or not */
	if (!mbhc_cb || !mbhc_cb->request_irq || !mbhc_cb->irq_control ||
	    !mbhc_cb->free_irq || !mbhc_cb->map_btn_code_to_num ||
	    !mbhc_cb->lock_sleep || !mbhc_cb->mbhc_bias ||
	    !mbhc_cb->set_btn_thr) {
		dev_err(component->dev, "%s: required mbhc callbacks are not defined\n",
			__func__);
		return -EINVAL;
	}

	/* No need to create new sound card jacks if is is already created */
	if (mbhc->headset_jack.jack == NULL) {
		ret = snd_soc_card_jack_new(component->card,
					    "Headset Jack", WCD_MBHC_JACK_MASK,
					    &mbhc->headset_jack);
		if (ret) {
			pr_err("%s: Failed to create new jack\n", __func__);
			return ret;
		}

		ret = snd_soc_card_jack_new(component->card,
					    "Button Jack",
					    WCD_MBHC_JACK_BUTTON_MASK,
					    &mbhc->button_jack);
		if (ret) {
			pr_err("Failed to create new jack\n");
			return ret;
		}

		ret = snd_jack_set_key(mbhc->button_jack.jack,
				       SND_JACK_BTN_0,
				       KEY_MEDIA);
		if (ret) {
			pr_err("%s: Failed to set code for btn-0\n",
				__func__);
			return ret;
		}

		INIT_DELAYED_WORK(&mbhc->mbhc_firmware_dwork,
				  wcd_mbhc_fw_read);
		INIT_DELAYED_WORK(&mbhc->mbhc_btn_dwork, wcd_btn_lpress_fn);
	}
	init_completion(&mbhc->btn_press_compl);

	/* Register event notifier */
	mbhc->nblock.notifier_call = wcd_event_notify;
	if (mbhc->mbhc_cb->register_notifier) {
		ret = mbhc->mbhc_cb->register_notifier(mbhc, &mbhc->nblock,
						       true);
		if (ret) {
			pr_err("%s: Failed to register notifier %d\n",
				__func__, ret);
			return ret;
		}
	}

	init_waitqueue_head(&mbhc->wait_btn_press);
	mutex_init(&mbhc->codec_resource_lock);

	switch (mbhc->mbhc_detection_logic) {
	case WCD_DETECTION_LEGACY:
		wcd_mbhc_legacy_init(mbhc);
		break;
	case WCD_DETECTION_ADC:
		wcd_mbhc_adc_init(mbhc);
		break;
	default:
		pr_err("%s: Unknown detection logic type %d\n",
			__func__, mbhc->mbhc_detection_logic);
		break;
	}

	if (!mbhc->mbhc_fn ||
	    !mbhc->mbhc_fn->wcd_mbhc_hs_ins_irq ||
	    !mbhc->mbhc_fn->wcd_mbhc_hs_rem_irq ||
	    !mbhc->mbhc_fn->wcd_mbhc_detect_plug_type ||
	    !mbhc->mbhc_fn->wcd_cancel_hs_detect_plug) {
		pr_err("%s: mbhc function pointer is NULL\n", __func__);
		goto err_mbhc_sw_irq;
	}
	ret = mbhc->mbhc_cb->request_irq(component,
				mbhc->intr_ids->mbhc_sw_intr,
				wcd_mbhc_mech_plug_detect_irq,
				"mbhc sw intr", mbhc);
	if (ret) {
		pr_err("%s: Failed to request irq %d, ret = %d\n", __func__,
		       mbhc->intr_ids->mbhc_sw_intr, ret);
		goto err_mbhc_sw_irq;
	}

	ret = mbhc->mbhc_cb->request_irq(component,
					 mbhc->intr_ids->mbhc_btn_press_intr,
					 wcd_mbhc_btn_press_handler,
					 "Button Press detect", mbhc);
	if (ret) {
		pr_err("%s: Failed to request irq %d\n", __func__,
		       mbhc->intr_ids->mbhc_btn_press_intr);
		goto err_btn_press_irq;
	}

	ret = mbhc->mbhc_cb->request_irq(component,
					 mbhc->intr_ids->mbhc_btn_release_intr,
					 wcd_mbhc_release_handler,
					 "Button Release detect", mbhc);
	if (ret) {
		pr_err("%s: Failed to request irq %d\n", __func__,
			mbhc->intr_ids->mbhc_btn_release_intr);
		goto err_btn_release_irq;
	}

	ret = mbhc->mbhc_cb->request_irq(component,
					 mbhc->intr_ids->mbhc_hs_ins_intr,
					 mbhc->mbhc_fn->wcd_mbhc_hs_ins_irq,
					 "Elect Insert", mbhc);
	if (ret) {
		pr_err("%s: Failed to request irq %d\n", __func__,
		       mbhc->intr_ids->mbhc_hs_ins_intr);
		goto err_mbhc_hs_ins_irq;
	}
	mbhc->mbhc_cb->irq_control(component, mbhc->intr_ids->mbhc_hs_ins_intr,
				   false);
	clear_bit(WCD_MBHC_ELEC_HS_INS, &mbhc->intr_status);

	ret = mbhc->mbhc_cb->request_irq(component,
					 mbhc->intr_ids->mbhc_hs_rem_intr,
					 mbhc->mbhc_fn->wcd_mbhc_hs_rem_irq,
					 "Elect Remove", mbhc);
	if (ret) {
		pr_err("%s: Failed to request irq %d\n", __func__,
		       mbhc->intr_ids->mbhc_hs_rem_intr);
		goto err_mbhc_hs_rem_irq;
	}
	mbhc->mbhc_cb->irq_control(component, mbhc->intr_ids->mbhc_hs_rem_intr,
				   false);
	clear_bit(WCD_MBHC_ELEC_HS_REM, &mbhc->intr_status);

	ret = mbhc->mbhc_cb->request_irq(component,
				mbhc->intr_ids->hph_left_ocp,
				wcd_mbhc_hphl_ocp_irq, "HPH_L OCP detect",
				mbhc);
	if (ret) {
		pr_err("%s: Failed to request irq %d\n", __func__,
		       mbhc->intr_ids->hph_left_ocp);
		goto err_hphl_ocp_irq;
	}

	ret = mbhc->mbhc_cb->request_irq(component,
				mbhc->intr_ids->hph_right_ocp,
				wcd_mbhc_hphr_ocp_irq, "HPH_R OCP detect",
				mbhc);
	if (ret) {
		pr_err("%s: Failed to request irq %d\n", __func__,
		       mbhc->intr_ids->hph_right_ocp);
		goto err_hphr_ocp_irq;
	}
	if (!mbhc->extdev) {
		mbhc->extdev =
			devm_extcon_dev_allocate(component->dev,
				mbhc_ext_dev_supported_table);
		if (IS_ERR(mbhc->extdev)) {
			goto err_ext_dev;
			ret = PTR_ERR(mbhc->extdev);
		}
		ret = devm_extcon_dev_register(component->dev, mbhc->extdev);
		if (ret) {
			pr_err("%s:audio registration failed\n", __func__);
			goto err_ext_dev;
		}
	}
	mbhc->deinit_in_progress = false;
	pr_debug("%s: leave ret %d\n", __func__, ret);
	return ret;

err_ext_dev:
	mbhc->mbhc_cb->free_irq(component, mbhc->intr_ids->hph_right_ocp, mbhc);
err_hphr_ocp_irq:
	mbhc->mbhc_cb->free_irq(component, mbhc->intr_ids->hph_left_ocp, mbhc);
err_hphl_ocp_irq:
	mbhc->mbhc_cb->free_irq(component, mbhc->intr_ids->mbhc_hs_rem_intr,
				mbhc);
err_mbhc_hs_rem_irq:
	mbhc->mbhc_cb->free_irq(component, mbhc->intr_ids->mbhc_hs_ins_intr,
				mbhc);
err_mbhc_hs_ins_irq:
	mbhc->mbhc_cb->free_irq(component,
				mbhc->intr_ids->mbhc_btn_release_intr,
				mbhc);
err_btn_release_irq:
	mbhc->mbhc_cb->free_irq(component, mbhc->intr_ids->mbhc_btn_press_intr,
				mbhc);
err_btn_press_irq:
	mbhc->mbhc_cb->free_irq(component, mbhc->intr_ids->mbhc_sw_intr, mbhc);
err_mbhc_sw_irq:
	if (mbhc->mbhc_cb->register_notifier)
		mbhc->mbhc_cb->register_notifier(mbhc, &mbhc->nblock, false);
	mutex_destroy(&mbhc->codec_resource_lock);
err:
	pr_debug("%s: leave ret %d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL(wcd_mbhc_init);

void wcd_mbhc_deinit(struct wcd_mbhc *mbhc)
{
	struct snd_soc_component *component = mbhc->component;

#ifdef OPLUS_ARCH_EXTENDS
/* Add for headphone remove issue */
	if (mbhc->irq_trigger_enable) {
		cancel_delayed_work_sync(&mbhc->mech_irq_trigger_dwork);
	}
#endif /* OPLUS_ARCH_EXTENDS */

	mbhc->mbhc_cb->free_irq(component, mbhc->intr_ids->mbhc_sw_intr, mbhc);
	mbhc->mbhc_cb->free_irq(component, mbhc->intr_ids->mbhc_btn_press_intr,
				mbhc);
	mbhc->mbhc_cb->free_irq(component,
				mbhc->intr_ids->mbhc_btn_release_intr,
				mbhc);
	mbhc->mbhc_cb->free_irq(component, mbhc->intr_ids->mbhc_hs_ins_intr,
				mbhc);
	mbhc->mbhc_cb->free_irq(component, mbhc->intr_ids->mbhc_hs_rem_intr,
				mbhc);
	mbhc->mbhc_cb->free_irq(component, mbhc->intr_ids->hph_left_ocp, mbhc);
	mbhc->mbhc_cb->free_irq(component, mbhc->intr_ids->hph_right_ocp, mbhc);
	if (mbhc->mbhc_cb && mbhc->mbhc_cb->register_notifier)
		mbhc->mbhc_cb->register_notifier(mbhc, &mbhc->nblock, false);
	if (mbhc->mbhc_fn->wcd_cancel_hs_detect_plug) {
		WCD_MBHC_RSC_LOCK(mbhc);
		mbhc->mbhc_fn->wcd_cancel_hs_detect_plug(mbhc,
					&mbhc->correct_plug_swch);
		WCD_MBHC_RSC_UNLOCK(mbhc);
	}
	mutex_destroy(&mbhc->codec_resource_lock);
}
EXPORT_SYMBOL(wcd_mbhc_deinit);

static int __init mbhc_init(void)
{
	mutex_init(&hphl_pa_lock);
	mutex_init(&hphr_pa_lock);
	return 0;
}

static void __exit mbhc_exit(void)
{
	mutex_destroy(&hphl_pa_lock);
	mutex_destroy(&hphr_pa_lock);
}

module_init(mbhc_init);
module_exit(mbhc_exit);

MODULE_DESCRIPTION("wcd MBHC v2 module");
MODULE_LICENSE("GPL v2");
