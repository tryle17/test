// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/sort.h>
#include <linux/clk.h>
#include <linux/bitmap.h>
#include <linux/sde_rsc.h>
#include <soc/qcom/crm.h>

#include "msm_prop.h"

#include "sde_kms.h"
#include "sde_trace.h"
#include "sde_crtc.h"
#include "sde_encoder.h"
#include "sde_hw_catalog.h"
#include "sde_core_perf.h"

#define SDE_PERF_MODE_STRING_SIZE	128
#define SDE_PERF_THRESHOLD_HIGH_MIN     12800000

#define GET_H32(val) (val >> 32)
#define GET_L32(val) (val & 0xffffffff)

static DEFINE_MUTEX(sde_core_perf_lock);

/**
 * enum sde_perf_mode - performance tuning mode
 * @SDE_PERF_MODE_NORMAL: performance controlled by user mode client
 * @SDE_PERF_MODE_MINIMUM: performance bounded by minimum setting
 * @SDE_PERF_MODE_FIXED: performance bounded by fixed setting
 */
enum sde_perf_mode {
	SDE_PERF_MODE_NORMAL,
	SDE_PERF_MODE_MINIMUM,
	SDE_PERF_MODE_FIXED,
	SDE_PERF_MODE_MAX
};

/**
 * enum sde_perf_vote_mode: perf vote mode.
 * @APPS_RSC_MODE:	It combines the vote for all displays and votes it
 *                      through APPS rsc. This is default mode when display
 *                      rsc is not available.
 * @DISP_RSC_MODE:	It combines the vote for all displays and votes it
 *                      through display rsc. This is default configuration
 *                      when display rsc is available.
 * @DISP_RSC_PRIMARY_MODE:	The primary display votes through display rsc
 *                      while all other displays votes through apps rsc.
 */
enum sde_perf_vote_mode {
	APPS_RSC_MODE,
	DISP_RSC_MODE,
	DISP_RSC_PRIMARY_MODE,
};

static struct sde_kms *_sde_crtc_get_kms(struct drm_crtc *crtc)
{
	struct msm_drm_private *priv;

	if (!crtc->dev || !crtc->dev->dev_private) {
		SDE_ERROR("invalid device\n");
		return NULL;
	}

	priv = crtc->dev->dev_private;
	if (!priv || !priv->kms) {
		SDE_ERROR("invalid kms\n");
		return NULL;
	}

	return to_sde_kms(priv->kms);
}

static bool _sde_core_perf_crtc_is_power_on(struct drm_crtc *crtc)
{
	return sde_crtc_is_enabled(crtc);
}

static int _sde_core_perf_crtc_cesta_update(struct sde_kms *sde_kms, struct sde_crtc *sde_crtc,
		struct sde_crtc_state *sde_cstate, struct sde_core_perf_params *perf,
		enum sde_perf_commit_state commit_state, bool check)
{
	struct sde_cesta_params params = {0,};

	if (!sde_crtc || !sde_crtc->cesta_client)
		return 0;

	/*
	 * In cases of no clk/bw votes provided by client or perf-mode changed through debugfs
	 * vote for only the IB for all clients, so that the max vote will not be aggregated
	 * over all the cesta clients
	 */
	if (!sde_cstate->bw_control || (sde_kms->perf.perf_tune.mode != SDE_PERF_MODE_NORMAL))
		params.max_vote = true;

	perf->ubwc_clk_rate = sde_crtc_get_property(sde_cstate, CRTC_PROP_UBWC_CLK);

	if (commit_state != SDE_PERF_DISABLE_COMMIT) {
		if (params.max_vote) {
			params.data.core_clk_rate_ab = 0;
			params.data.core_clk_rate_ib = sde_kms->perf.max_core_clk_rate;
			params.data.bw_ab = 0;
			params.data.bw_ib = sde_kms->catalog->perf.max_bw_high * 1000ull;
		} else {
			params.data.core_clk_rate_ab = perf->ubwc_clk_rate;
			params.data.core_clk_rate_ib = perf->core_clk_rate;
			params.data.bw_ab = perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_MNOC];
			params.data.bw_ib = perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_MNOC];
		}
	}

	params.enable = (commit_state == SDE_PERF_DISABLE_COMMIT) ? false : true;
	params.post_commit = (commit_state == SDE_PERF_COMPLETE_COMMIT) ? true : false;
	params.pwr_st_override = ((commit_state == SDE_PERF_ENABLE_COMMIT)
					|| (commit_state == SDE_PERF_DISABLE_COMMIT)
					|| (sde_crtc->cesta_client->enabled != params.enable))
						? true : false;
	if (check)
		return sde_cesta_clk_bw_check(sde_crtc->cesta_client, &params);

	sde_cesta_clk_bw_update(sde_crtc->cesta_client, &params);

	SDE_EVT32(DRMID(&sde_crtc->base), params.enable, params.post_commit, params.pwr_st_override,
			commit_state, params.max_vote, params.data.core_clk_rate_ab,
			params.data.core_clk_rate_ib, params.data.bw_ab, params.data.bw_ib);

	return 0;
}

static void _sde_core_perf_calc_crtc(struct sde_kms *kms,
		struct drm_crtc *crtc,
		struct drm_crtc_state *state,
		struct sde_core_perf_params *perf)
{
	struct sde_crtc_state *sde_cstate;
	int i;

	if (!kms || !kms->catalog || !crtc || !state || !perf) {
		SDE_ERROR("invalid parameters\n");
		return;
	}

	sde_cstate = to_sde_crtc_state(state);
	memset(perf, 0, sizeof(struct sde_core_perf_params));

	perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_MNOC] =
		sde_crtc_get_property(sde_cstate, CRTC_PROP_CORE_AB);
	perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_MNOC] =
		sde_crtc_get_property(sde_cstate, CRTC_PROP_CORE_IB);

	if (sde_cstate->bw_split_vote) {
		perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_LLCC] =
			sde_crtc_get_property(sde_cstate, CRTC_PROP_LLCC_AB);
		perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_LLCC] =
			sde_crtc_get_property(sde_cstate, CRTC_PROP_LLCC_IB);
		perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_EBI] =
			sde_crtc_get_property(sde_cstate, CRTC_PROP_DRAM_AB);
		perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_EBI] =
			sde_crtc_get_property(sde_cstate, CRTC_PROP_DRAM_IB);
	} else {
		perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_LLCC] =
			sde_crtc_get_property(sde_cstate, CRTC_PROP_CORE_AB);
		perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_LLCC] =
			sde_crtc_get_property(sde_cstate, CRTC_PROP_CORE_IB);
		perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_EBI] =
			sde_crtc_get_property(sde_cstate, CRTC_PROP_CORE_AB);
		perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_EBI] =
			sde_crtc_get_property(sde_cstate, CRTC_PROP_CORE_IB);
	}

	perf->core_clk_rate =
			sde_crtc_get_property(sde_cstate, CRTC_PROP_CORE_CLK);

	if (!sde_cstate->bw_control) {
		for (i = 0; i < SDE_POWER_HANDLE_DBUS_ID_MAX; i++) {
			perf->bw_ctl[i] = kms->catalog->perf.max_bw_high *
					1000ULL;
			perf->max_per_pipe_ib[i] = perf->bw_ctl[i];
		}
		perf->core_clk_rate = kms->perf.max_core_clk_rate;
	} else if (kms->perf.perf_tune.mode == SDE_PERF_MODE_MINIMUM) {
		for (i = 0; i < SDE_POWER_HANDLE_DBUS_ID_MAX; i++) {
			perf->bw_ctl[i] = 0;
			perf->max_per_pipe_ib[i] = 0;
		}
		perf->core_clk_rate = 0;
	} else if (kms->perf.perf_tune.mode == SDE_PERF_MODE_FIXED) {
		for (i = 0; i < SDE_POWER_HANDLE_DBUS_ID_MAX; i++) {
			perf->bw_ctl[i] = max(kms->perf.fix_core_ab_vote,
						perf->bw_ctl[i]);
			perf->max_per_pipe_ib[i] = max(
						kms->perf.fix_core_ib_vote,
						perf->max_per_pipe_ib[i]);
		}
		perf->core_clk_rate = max(kms->perf.fix_core_clk_rate,
						perf->core_clk_rate);
	}

	perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_DDR_RT] =
		mult_frac(perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_EBI],
			SDE_PERF_MAX_COMPRESSION_FACTOR, 100);
	perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_DDR_RT] =
		SDE_POWER_HANDLE_DISABLE_BUS_IB_QUOTA;

	SDE_EVT32(DRMID(crtc), perf->core_clk_rate,
		GET_H32(perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_MNOC]),
		GET_L32(perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_MNOC]),
		GET_H32(perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_LLCC]),
		GET_L32(perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_LLCC]),
		GET_H32(perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_EBI]),
		GET_L32(perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_EBI]));
	SDE_EVT32(DRMID(crtc),
		GET_H32(perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_MNOC]),
		GET_L32(perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_MNOC]),
		GET_H32(perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_LLCC]),
		GET_L32(perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_LLCC]),
		GET_H32(perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_EBI]),
		GET_L32(perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_EBI]));
	trace_sde_perf_calc_crtc(crtc->base.id,
			perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_MNOC],
			perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_LLCC],
			perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_EBI],
			perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_MNOC],
			perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_LLCC],
			perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_EBI],
			perf->core_clk_rate);

	SDE_DEBUG(
		"crtc=%d clk_rate=%llu core_ib=%llu core_ab=%llu llcc_ib=%llu llcc_ab=%llu mem_ib=%llu mem_ab=%llu\n",
			crtc->base.id, perf->core_clk_rate,
			perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_MNOC],
			perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_MNOC],
			perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_LLCC],
			perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_LLCC],
			perf->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_EBI],
			perf->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_EBI]);
}

int sde_core_perf_crtc_check(struct drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	u32 bw, threshold;
	u64 bw_sum_of_intfs = 0;
	enum sde_crtc_client_type curr_client_type;
	struct sde_crtc_state *sde_cstate;
	struct sde_crtc *sde_crtc;
	struct msm_drm_private *priv;
	struct drm_crtc *tmp_crtc;
	struct sde_kms *kms;
	u64 current_clk_rate, new_clk_rate;
	int i, ret;

	if (!crtc || !state) {
		SDE_ERROR("invalid crtc\n");
		return -EINVAL;
	}

	kms = _sde_crtc_get_kms(crtc);
	if (!kms || !kms->catalog) {
		SDE_ERROR("invalid parameters\n");
		return 0;
	}

	sde_cstate = to_sde_crtc_state(state);
	sde_crtc = to_sde_crtc(crtc);
	priv = kms->dev->dev_private;

	/* obtain new values */
	_sde_core_perf_calc_crtc(kms, crtc, state, &sde_cstate->new_perf);

	/* do cesta check and return early, when sde cesta is enabled */
	if (sde_cesta_is_enabled(DPUID(kms->dev)))
		return _sde_core_perf_crtc_cesta_update(kms, sde_crtc, sde_cstate,
						&sde_cstate->new_perf, SDE_PERF_NONE_COMMIT, true);

	/* reserve core clk */
	current_clk_rate = kms->perf.core_clk_rate;
	new_clk_rate = sde_cstate->new_perf.core_clk_rate;
	if (new_clk_rate > current_clk_rate) {
		new_clk_rate = clk_round_rate(kms->perf.core_clk,
			new_clk_rate);
		ret = sde_power_clk_set_rate(&priv->phandle,
			kms->perf.clk_name, new_clk_rate,
			MMRM_CLIENT_DATA_FLAG_RESERVE_ONLY);
		if (ret) {
			SDE_ERROR("cannot reserve core clk rate:%llu\n",
				new_clk_rate);

			return -E2BIG;
		}
	}

	for (i = SDE_POWER_HANDLE_DBUS_ID_MNOC;
			i < SDE_POWER_HANDLE_DBUS_ID_MAX; i++) {
		bw_sum_of_intfs = sde_cstate->new_perf.bw_ctl[i];
		curr_client_type = sde_crtc_get_client_type(crtc);

		drm_for_each_crtc(tmp_crtc, crtc->dev) {
			if (_sde_core_perf_crtc_is_power_on(tmp_crtc) &&
			    (sde_crtc_get_client_type(tmp_crtc) ==
					    curr_client_type) &&
			    (tmp_crtc != crtc)) {
				struct sde_crtc_state *tmp_cstate =
					to_sde_crtc_state(tmp_crtc->state);

				SDE_DEBUG("crtc:%d bw:%llu ctrl:%d\n",
					tmp_crtc->base.id,
					tmp_cstate->new_perf.bw_ctl[i],
					tmp_cstate->bw_control);
				/*
				 * For bw check only use the bw if the
				 * atomic property has been already set
				 */
				if (tmp_cstate->bw_control)
					bw_sum_of_intfs +=
						tmp_cstate->new_perf.bw_ctl[i];
			}
		}

		/* convert bandwidth to kb */
		bw = DIV_ROUND_UP_ULL(bw_sum_of_intfs, 1000);
		SDE_DEBUG("calculated bandwidth=%uk\n", bw);

		threshold = kms->catalog->perf.max_bw_high;

		SDE_DEBUG("final threshold bw limit = %d\n", threshold);

		if (!sde_cstate->bw_control) {
			SDE_DEBUG("bypass bandwidth check\n");
		} else if (!threshold) {
			SDE_ERROR("no bandwidth limits specified\n");
			return -E2BIG;
		} else if (bw > threshold) {
			SDE_ERROR("exceeds bandwidth: %ukb > %ukb\n", bw,
					threshold);
			return -E2BIG;
		}
	}

	return 0;
}

static inline bool _is_crtc_client_type_matches(struct drm_crtc *tmp_crtc,
	enum sde_crtc_client_type curr_client_type,
	struct sde_core_perf *perf)
{
	if (!tmp_crtc)
		return false;
	else if (perf->bw_vote_mode == DISP_RSC_PRIMARY_MODE &&
							perf->sde_rsc_available)
		return curr_client_type == sde_crtc_get_client_type(tmp_crtc);
	else
		return true;
}

static inline enum sde_crtc_client_type _get_sde_client_type(
	enum sde_crtc_client_type curr_client_type,
	struct sde_core_perf *perf)
{
	if (perf->bw_vote_mode == DISP_RSC_PRIMARY_MODE &&
						perf->sde_rsc_available)
		return curr_client_type;
	else if (perf->bw_vote_mode != APPS_RSC_MODE && perf->sde_rsc_available)
		return RT_RSC_CLIENT;
	else
		return RT_CLIENT;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
void sde_core_perf_llcc_stale_configure(struct sde_mdss_cfg *sde_cfg, struct llcc_slice_desc *slice)
{
	struct llcc_staling_mode_params params = {0};

	if (!sde_cfg || !slice || !test_bit(SDE_FEATURE_SYS_CACHE_STALING, sde_cfg->features))
		return;

	llcc_configure_staling_mode(slice, &params);
}

void sde_core_perf_llcc_stale_frame(struct drm_crtc *crtc, enum sde_sys_cache_type type)
{
	struct sde_kms *kms;

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	kms = _sde_crtc_get_kms(crtc);
	if (!kms || !kms->catalog) {
		SDE_ERROR("invalid kms\n");
		return;
	}

	if (!test_bit(SDE_FEATURE_SYS_CACHE_STALING, kms->catalog->features) ||
			!kms->perf.llcc_active[type])
		return;

	llcc_notif_staling_inc_counter(kms->catalog->sc_cfg[type].slice);
}
#else
void sde_core_perf_llcc_stale_configure(struct sde_mdss_cfg *sde_cfg, struct llcc_slice_desc *slice)
{
}
void sde_core_perf_llcc_stale_frame(struct drm_crtc *crtc, enum sde_sys_cache_type type)
{
}
#endif

/**
 * @_sde_core_perf_activate_llcc() - Activates/deactivates the system llcc
 * @kms - pointer to the kms
 * @type - llcc type to be activated
 * @activate - boolean to indicate if activate/deactivate the LLCC
 *
 * Function assumes that caller has already acquired the "sde_core_perf_lock",
 * which would protect from any race condition between CRTC's
 */
static int _sde_core_perf_activate_llcc(struct sde_kms *kms,
	enum sde_sys_cache_type type, bool activate)
{
	struct llcc_slice_desc *slice;
	struct drm_device *drm_dev;
	struct device *dev;
	struct platform_device *pdev;
	u32 scid;
	int rc = 0;

	if (!kms || !kms->dev || !kms->dev->dev) {
		SDE_ERROR("wrong params won't activate llcc\n");
		rc = -EINVAL;
		goto exit;
	}

	drm_dev = kms->dev;
	dev = drm_dev->dev;
	pdev = to_platform_device(dev);

	/* If LLCC is already in the requested state, skip */
	if ((activate && kms->perf.llcc_active[type]) ||
		(!activate && !kms->perf.llcc_active[type])) {
		SDE_DEBUG("skip llcc type:%d request:%d state:%d\n",
			type, activate, kms->perf.llcc_active[type]);
		goto exit;
	}

	slice = llcc_slice_getd(kms->catalog->sc_cfg[type].llcc_uid);
	if (IS_ERR_OR_NULL(slice))  {
		SDE_ERROR("failed to get llcc slice for uid:%d\n",
				kms->catalog->sc_cfg[type].llcc_uid);
		rc = -EINVAL;
		goto exit;
	}

	scid = llcc_get_slice_id(slice);
	SDE_EVT32(activate, type, kms->perf.llcc_active[type], scid);
	SDE_DEBUG("%sactivate the llcc type:%d state:%d scid:%d\n", activate ? "" : "de", type,
			kms->perf.llcc_active[type], scid);

	if (activate) {
		llcc_slice_activate(slice);
		kms->perf.llcc_active[type] = true;
	} else {
		llcc_slice_deactivate(slice);
		kms->perf.llcc_active[type] = false;
	}

exit:
	if (rc)
		SDE_ERROR("error %sactivating llcc type:%d rc:%d\n",
			activate ? "" : "de", type, rc);
	return rc;

}

static void _sde_core_perf_crtc_set_llcc_cache_type(struct sde_kms *kms,
		struct drm_crtc *crtc,
		enum sde_sys_cache_type type)
{
	struct drm_crtc *tmp_crtc;
	struct sde_crtc *sde_crtc;
	struct sde_core_perf_params *cur_perf;
	enum sde_crtc_client_type curr_client_type
					= sde_crtc_get_client_type(crtc);
	u32 llcc_active = 0;

	if (!test_bit(type, kms->perf.catalog->sde_sys_cache_type_map)) {
		SDE_DEBUG("system cache %d is not enabled!. Won't use\n", type);
		return;
	}

	drm_for_each_crtc(tmp_crtc, crtc->dev) {
		if (_sde_core_perf_crtc_is_power_on(tmp_crtc) &&
			_is_crtc_client_type_matches(tmp_crtc, curr_client_type,
								&kms->perf)) {

			/* use current perf, which are the values voted */
			sde_crtc = to_sde_crtc(tmp_crtc);
			cur_perf = &sde_crtc->cur_perf;
			llcc_active |= cur_perf->llcc_active[type];

			SDE_DEBUG("crtc=%d type:%d llcc:%u active:0x%x\n",
				tmp_crtc->base.id, type,
				cur_perf->llcc_active[type],
				llcc_active);
		}
	}

	_sde_core_perf_activate_llcc(kms, type,
			llcc_active ? true : false);
}

void sde_core_perf_crtc_update_llcc(struct drm_crtc *crtc)
{
	struct sde_kms *kms;
	struct sde_crtc *sde_crtc;
	struct sde_core_perf_params *old, *new;
	int update_llcc[SDE_SYS_CACHE_MAX] = {0};
	int i;

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	kms = _sde_crtc_get_kms(crtc);
	if (!kms || !kms->catalog) {
		SDE_ERROR("invalid kms\n");
		return;
	}

	sde_crtc = to_sde_crtc(crtc);

	old = &sde_crtc->cur_perf;
	new = &sde_crtc->new_perf;

	mutex_lock(&sde_core_perf_lock);

	/* update based on sys_cache_enabled debugfs node */
	for (i = 0; i < SDE_SYS_CACHE_MAX; i++) {
		if (!(kms->perf.sys_cache_enabled & BIT(i))) {
			SDE_DEBUG("system cache[%d] is disabled from debugfs\n", i);
			new->llcc_active[i] = false;
		}
	}

	if (_sde_core_perf_crtc_is_power_on(crtc)) {
		for (i = 0; i < SDE_SYS_CACHE_MAX; i++) {
			if (new->llcc_active[i] != old->llcc_active[i]) {
				SDE_DEBUG("crtc=%d llcc=%d new=%d old=%d",
						crtc->base.id, i,
						new->llcc_active[i],
						old->llcc_active[i]);

					old->llcc_active[i] =
							new->llcc_active[i];
					update_llcc[i] = 1;
			}
		}
	} else {
		for (i = 0; i < SDE_SYS_CACHE_MAX; i++)
			update_llcc[i] = 1;
	}

	for (i = 0; i < SDE_SYS_CACHE_MAX; i++) {
		if (update_llcc[i])
			_sde_core_perf_crtc_set_llcc_cache_type(kms, crtc, i);
	}

	mutex_unlock(&sde_core_perf_lock);
};

static void _sde_core_uidle_setup_wd(struct sde_kms *kms,
	bool enable)
{
	struct sde_uidle_wd_cfg wd;
	struct sde_hw_uidle *uidle;

	uidle = kms->hw_uidle;
	wd.enable = enable;
	wd.clear = false;
	wd.granularity = SDE_UIDLE_WD_GRANULARITY;
	wd.heart_beat = SDE_UIDLE_WD_HEART_BEAT;
	wd.load_value = SDE_UIDLE_WD_LOAD_VAL;

	if (uidle->ops.setup_wd_timer)
		uidle->ops.setup_wd_timer(uidle, &wd);
}

static void _sde_core_uidle_setup_cfg(struct sde_kms *kms,
	enum sde_uidle_state state)
{
	struct sde_uidle_ctl_cfg cfg;
	struct sde_hw_uidle *uidle;

	uidle = kms->hw_uidle;
	cfg.uidle_state = state;
	cfg.fal10_danger =
		kms->catalog->uidle_cfg.fal10_danger;
	cfg.fal10_exit_cnt =
		kms->catalog->uidle_cfg.fal10_exit_cnt;
	cfg.fal10_exit_danger =
		kms->catalog->uidle_cfg.fal10_exit_danger;

	SDE_DEBUG("fal10_danger:%d fal10_exit_cnt:%d fal10_exit_danger:%d\n",
		cfg.fal10_danger, cfg.fal10_exit_cnt, cfg.fal10_exit_danger);
	SDE_EVT32(state, cfg.fal10_danger, cfg.fal10_exit_cnt,
		cfg.fal10_exit_danger);

	if (uidle->ops.set_uidle_ctl)
		uidle->ops.set_uidle_ctl(uidle, &cfg);
}

void sde_core_perf_uidle_setup_ctl(struct drm_crtc *crtc,
	bool enable)
{
	struct drm_encoder *drm_enc;

	/* Disable uidle in the CTL */
	drm_for_each_encoder(drm_enc, crtc->dev) {
		if (drm_enc->crtc != crtc)
			continue;

		sde_encoder_uidle_enable(drm_enc, enable);
	}
}

static int _sde_core_perf_enable_uidle(struct sde_kms *kms,
	struct drm_crtc *crtc, enum sde_uidle_state uidle_state)
{
	int rc = 0;
	bool enable = (uidle_state > UIDLE_STATE_DISABLE);

	if (!kms->dev || !kms->dev->dev || !kms->hw_uidle ||
			uidle_state >= UIDLE_STATE_ENABLE_MAX) {
		SDE_ERROR("wrong params won't enable uidle_state %d\n", uidle_state);
		rc = -EINVAL;
		goto exit;
	}

	SDE_EVT32(uidle_state);
	_sde_core_uidle_setup_wd(kms, enable);
	_sde_core_uidle_setup_cfg(kms, uidle_state);
	sde_core_perf_uidle_setup_ctl(crtc, true);

	kms->perf.uidle_enabled = enable;

exit:
	return rc;
}

static inline bool _sde_core_perf_is_wb(struct drm_crtc *crtc)
{
	enum sde_intf_mode if_mode = INTF_MODE_NONE;

	if_mode = sde_crtc_get_intf_mode(crtc, crtc->state);
	if (if_mode == INTF_MODE_WB_BLOCK ||
		if_mode == INTF_MODE_WB_LINE)
		return true;

	return false;
}

static bool _sde_core_perf_is_cwb(struct drm_crtc *crtc)
{
	struct drm_encoder *encoder;

	/* if any other encoder is connected to same crtc in clone mode */
	drm_for_each_encoder(encoder, crtc->dev) {
		if (encoder->crtc == crtc &&
				sde_encoder_in_clone_mode(encoder)) {
			return true;
		}
	}

	return false;
}

static void _sde_core_perf_uidle_setup_cntr(struct sde_kms *sde_kms,
	bool enable)
{
	struct sde_hw_uidle *uidle;

	uidle = sde_kms->hw_uidle;

	SDE_EVT32(enable);
	if (uidle->ops.uidle_setup_cntr) {
		uidle->ops.uidle_setup_cntr(uidle, enable);
		sde_kms->catalog->uidle_cfg.perf_cntr_en = enable;
	}
}

void sde_core_perf_crtc_update_uidle(struct drm_crtc *crtc,
	bool enable)
{
	struct drm_crtc *tmp_crtc;
	struct sde_kms *kms;
	enum sde_uidle_state uidle_status = UIDLE_STATE_FAL1_FAL10;
	u32 fps, num_crtc = 0;

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	kms = _sde_crtc_get_kms(crtc);
	if (!kms || !kms->catalog) {
		SDE_ERROR("invalid kms\n");
		return;
	}

	mutex_lock(&sde_core_perf_lock);

	if (!kms->perf.catalog->uidle_cfg.uidle_rev ||
		(enable && !kms->perf.catalog->uidle_cfg.debugfs_ctrl)) {
		SDE_DEBUG("uidle is not enabled %d %d\n",
			kms->perf.catalog->uidle_cfg.uidle_rev,
			kms->perf.catalog->uidle_cfg.debugfs_ctrl);
		goto exit;
	}

	drm_for_each_crtc(tmp_crtc, crtc->dev) {
		enum sde_uidle_state uidle_crtc_status = UIDLE_STATE_FAL1_FAL10;

		if (_sde_core_perf_crtc_is_power_on(tmp_crtc)) {

			num_crtc++;
			/*
			 * If DFPS is enabled with VFP, SDE clock and
			 * transfer time will get fixed at max FPS
			 * configuration of DFPS.
			 * So get the max FPS of DFPS firstly for
			 * UIDLE update, if DFPS is enabled with VFP.
			 */
			fps = sde_crtc_get_dfps_maxfps(tmp_crtc);
			if (!fps)
				fps = sde_crtc_get_fps_mode(tmp_crtc);

			SDE_DEBUG("crtc=%d fps:%d wb:%d cwb:%d uidle:%d uidle_crtc:%d en:%d\n",
				tmp_crtc->base.id, fps,
				_sde_core_perf_is_wb(tmp_crtc),
				_sde_core_perf_is_cwb(tmp_crtc),
				uidle_status, uidle_crtc_status, enable);

			if ((num_crtc > 1) || _sde_core_perf_is_wb(tmp_crtc) ||
				_sde_core_perf_is_cwb(tmp_crtc) || !fps) {
				uidle_status = UIDLE_STATE_DISABLE;
				break;
			}

			/* Check if FAL1 only should be enabled */
			if (fps <=  kms->perf.catalog->uidle_cfg.max_fps)
				uidle_crtc_status = UIDLE_STATE_FAL1_FAL10;
			else if (fps <= kms->perf.catalog->uidle_cfg.max_fal1_fps)
				uidle_crtc_status = UIDLE_STATE_FAL1_ONLY;
			else
				uidle_crtc_status = UIDLE_STATE_DISABLE;

			if (uidle_crtc_status < uidle_status)
				uidle_status = uidle_crtc_status;

			if (uidle_status == UIDLE_STATE_DISABLE)
				break;
		}
	}

	_sde_core_perf_enable_uidle(kms, crtc,
			enable ? uidle_status : UIDLE_STATE_DISABLE);

	kms->perf.catalog->uidle_cfg.dirty = !enable;

	/* If perf counters enabled, set them up now */
	if (kms->catalog->uidle_cfg.debugfs_perf)
		_sde_core_perf_uidle_setup_cntr(kms, enable);

exit:
	mutex_unlock(&sde_core_perf_lock);
}

static void _sde_core_perf_crtc_update_bus(struct sde_kms *kms,
		struct drm_crtc *crtc, u32 bus_id)
{
	u64 bw_sum_of_intfs = 0, bus_ib_quota = 0, bus_ab_quota;
	enum sde_crtc_client_type client_vote, curr_client_type
					= sde_crtc_get_client_type(crtc);
	struct drm_crtc *tmp_crtc;
	struct sde_crtc_state *sde_cstate;
	struct msm_drm_private *priv = kms->dev->dev_private;
	struct sde_crtc *sde_crtc;

	u64 tmp_bw_ctl;

	drm_for_each_crtc(tmp_crtc, crtc->dev) {
		if (_sde_core_perf_crtc_is_power_on(tmp_crtc) &&
		    _is_crtc_client_type_matches(tmp_crtc, curr_client_type,
								&kms->perf)) {

			/* use current perf, which are the values voted */
			sde_crtc = to_sde_crtc(tmp_crtc);
			tmp_bw_ctl =
			  sde_crtc->cur_perf.bw_ctl[bus_id];


			bw_sum_of_intfs += tmp_bw_ctl;

			SDE_DEBUG("crtc=%d bus_id=%d bw=%llu\n",
				tmp_crtc->base.id, bus_id,
				tmp_bw_ctl);
		}
	}

	bus_ab_quota = max(bw_sum_of_intfs, kms->perf.perf_tune.min_bus_vote);
	bus_ab_quota = min(bus_ab_quota,
			kms->catalog->perf.max_bw_high*1000ULL);

	if (kms->catalog->perf.num_ddr_channels && kms->catalog->perf.dram_efficiency &&
		(bus_id != SDE_POWER_HANDLE_DBUS_ID_DDR_RT)) {
		bus_ib_quota = div_u64(div_u64(bus_ab_quota,
			kms->catalog->perf.num_ddr_channels) * 100,
			kms->catalog->perf.dram_efficiency);
	}

	if (kms->perf.perf_tune.mode == SDE_PERF_MODE_FIXED) {
		bus_ab_quota = max(kms->perf.fix_core_ab_vote,
					bus_ab_quota);
		bus_ib_quota = max(kms->perf.fix_core_ib_vote,
					bus_ib_quota);
	}

	client_vote = _get_sde_client_type(curr_client_type, &kms->perf);
	switch (client_vote) {
	case RT_CLIENT:
		sde_power_data_bus_set_quota(&priv->phandle,
				bus_id, bus_ab_quota, bus_ib_quota);
		SDE_DEBUG("client:%s bus_id=%d ab=%llu ib=%llu\n", "rt",
				bus_id, bus_ab_quota, bus_ib_quota);
		break;

	case RT_RSC_CLIENT:
		sde_cstate = to_sde_crtc_state(crtc->state);
		sde_rsc_client_vote(sde_cstate->rsc_client,
				bus_id, bus_ab_quota, bus_ib_quota);
		SDE_DEBUG("client:%s bus_id=%d ab=%llu ib=%llu\n", "rt_rsc",
				bus_id, bus_ab_quota, bus_ib_quota);
		break;

	default:
		SDE_ERROR("invalid client type:%d\n", curr_client_type);
		break;
	}

	if (kms->perf.bw_vote_mode_updated) {
		switch (kms->perf.bw_vote_mode) {
		case DISP_RSC_MODE:
			sde_power_data_bus_set_quota(&priv->phandle,
				bus_id, 0, 0);
			kms->perf.bw_vote_mode_updated = false;
			break;

		case APPS_RSC_MODE:
			sde_cstate = to_sde_crtc_state(crtc->state);
			if (sde_cstate->rsc_client) {
				sde_rsc_client_vote(sde_cstate->rsc_client,
								bus_id, 0, 0);
				kms->perf.bw_vote_mode_updated = false;
			}
			break;

		default:
			break;
		}
	}
}

/**
 * @sde_core_perf_crtc_release_bw() - request zero bandwidth
 * @crtc - pointer to a crtc
 *
 * Function checks a state variable for the crtc, if all pending commit
 * requests are done, meaning no more bandwidth is needed, release
 * bandwidth request.
 */
void sde_core_perf_crtc_release_bw(struct drm_crtc *crtc)
{
	struct drm_crtc *tmp_crtc;
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *sde_cstate;
	struct sde_kms *kms;
	int i;

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	kms = _sde_crtc_get_kms(crtc);
	if (!kms || !kms->catalog) {
		SDE_ERROR("invalid kms\n");
		return;
	}

	sde_crtc = to_sde_crtc(crtc);
	sde_cstate = to_sde_crtc_state(crtc->state);

	/* return early when cesta is enabled, as a separate bw release is not required */
	if (sde_cesta_is_enabled(DPUID(kms->dev)))
		return;

	/* only do this for command mode rt client (non-rsc client) */
	if ((sde_crtc_get_intf_mode(crtc, crtc->state) != INTF_MODE_CMD) &&
		(sde_crtc_get_client_type(crtc) != RT_RSC_CLIENT))
		return;

	/*
	 * If video interface present, cmd panel bandwidth cannot be
	 * released.
	 */
	if (sde_crtc_get_intf_mode(crtc, crtc->state) == INTF_MODE_CMD)
		drm_for_each_crtc(tmp_crtc, crtc->dev) {
			if (_sde_core_perf_crtc_is_power_on(tmp_crtc) &&
				sde_crtc_get_intf_mode(tmp_crtc,
					tmp_crtc->state) == INTF_MODE_VIDEO)
				return;
		}

	/* Release the bandwidth */
	if (kms->perf.enable_bw_release) {
		trace_sde_cmd_release_bw(crtc->base.id);
		SDE_DEBUG("Release BW crtc=%d\n", crtc->base.id);
		for (i = 0; i < SDE_POWER_HANDLE_DBUS_ID_MAX; i++) {
			sde_crtc->cur_perf.bw_ctl[i] = 0;
			_sde_core_perf_crtc_update_bus(kms, crtc, i);
		}
	}
}

void sde_core_perf_crtc_reserve_res(struct drm_crtc *crtc, u64 reserve_rate)
{
	struct sde_crtc *sde_crtc;
	struct sde_kms *kms;
	struct msm_drm_private *priv;
	struct sde_power_handle *phandle;

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	/* use current perf, which are the values voted */
	sde_crtc = to_sde_crtc(crtc);
	kms = _sde_crtc_get_kms(crtc);
	if (!kms || !kms->dev) {
		SDE_ERROR("invalid kms\n");
		return;
	}

	priv = kms->dev->dev_private;

	kms->perf.core_clk_reserve_rate = max(kms->perf.core_clk_reserve_rate, reserve_rate);
	kms->perf.core_clk_reserve_rate = min(kms->perf.core_clk_reserve_rate,
			kms->perf.max_core_clk_rate);

	phandle = kms->perf.cesta_phandle ? kms->perf.cesta_phandle : &priv->phandle;

	sde_power_clk_reserve_rate(phandle, kms->perf.clk_name,
			kms->perf.core_clk_reserve_rate);

	SDE_DEBUG("reserve clk:%llu\n", kms->perf.core_clk_reserve_rate);
}

static u64 _sde_core_perf_get_core_clk_rate(struct sde_kms *kms)
{
	u64 clk_rate = kms->perf.perf_tune.min_core_clk;
	struct drm_crtc *tmp_crtc;
	struct sde_crtc *sde_crtc;
	u64 tmp_rate;

	drm_for_each_crtc(tmp_crtc, kms->dev) {
		if (_sde_core_perf_crtc_is_power_on(tmp_crtc)) {

			/* use current perf, which are the values voted */
			sde_crtc = to_sde_crtc(tmp_crtc);
			tmp_rate = sde_crtc->cur_perf.core_clk_rate;

			clk_rate = max(tmp_rate, clk_rate);

			clk_rate = clk_round_rate(kms->perf.core_clk, clk_rate);
		}
	}

	if (kms->perf.perf_tune.mode == SDE_PERF_MODE_FIXED)
		clk_rate = max(kms->perf.fix_core_clk_rate, clk_rate);

	SDE_DEBUG("clk:%llu\n", clk_rate);

	return clk_rate;
}

static void _sde_core_perf_crtc_update_check(struct drm_crtc *crtc,
		int params_changed,
		int *update_bus, int *update_clk)
{
	struct sde_kms *kms = _sde_crtc_get_kms(crtc);
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_core_perf_params *old = &sde_crtc->cur_perf;
	struct sde_core_perf_params *new = &sde_crtc->new_perf;
	int i;

	if (!kms)
		return;

	for (i = 0; i < SDE_POWER_HANDLE_DBUS_ID_MAX; i++) {
		/*
		 * cases for bus bandwidth update.
		 * 1. new bandwidth vote - "ab or ib vote" is higher
		 *    than current vote for update request.
		 * 2. new bandwidth vote - "ab or ib vote" is lower
		 *    than current vote at end of commit or stop.
		 */

		if ((params_changed &&
				(new->bw_ctl[i] > old->bw_ctl[i])) ||
				(!params_changed &&
				(new->bw_ctl[i] < old->bw_ctl[i])))
			*update_bus |= BIT(i);

		if ((params_changed &&
				(new->max_per_pipe_ib[i] >
				 old->max_per_pipe_ib[i])) ||
				(!params_changed &&
				(new->max_per_pipe_ib[i] <
				old->max_per_pipe_ib[i])))
			*update_bus |= BIT(i);

		/* display rsc override during solver mode */
		if (kms->perf.bw_vote_mode == DISP_RSC_MODE &&
				get_sde_rsc_current_state(DPUID(kms->dev)) !=
				SDE_RSC_CLK_STATE) {
			/* update new bandwidth in all cases */
			if (params_changed && ((new->bw_ctl[i] !=
					old->bw_ctl[i]) ||
					(new->max_per_pipe_ib[i] !=
					old->max_per_pipe_ib[i]))) {
				*update_bus |= BIT(i);
			/*
			 * reduce bw vote is not required in solver
			 * mode
			 */
			} else if (!params_changed) {
				*update_bus &= ~BIT(i);
			}
		}

		if ((*update_bus) & BIT(i)) {
			SDE_DEBUG(
				"crtc=%d p=%d new_bw=%llu,old_bw=%llu new_ib=%llu old_ib=%llu\n",
				crtc->base.id, params_changed, new->bw_ctl[i], old->bw_ctl[i],
				new->max_per_pipe_ib[i], old->max_per_pipe_ib[i]);
			old->bw_ctl[i] = new->bw_ctl[i];
			old->max_per_pipe_ib[i] = new->max_per_pipe_ib[i];
		}
	}

	if (kms->perf.perf_tune.mode_changed &&
			kms->perf.perf_tune.min_core_clk)
		new->core_clk_rate = kms->perf.perf_tune.min_core_clk;

	if ((params_changed &&
			(new->core_clk_rate > old->core_clk_rate)) ||
			(!params_changed && new->core_clk_rate &&
			(new->core_clk_rate < old->core_clk_rate)) ||
			kms->perf.perf_tune.mode_changed) {
		old->core_clk_rate = new->core_clk_rate;
		*update_clk = 1;
		kms->perf.perf_tune.mode_changed = false;
	}
}

void sde_core_perf_crtc_update(struct drm_crtc *crtc, enum sde_perf_commit_state commit_state)
{
	struct sde_core_perf_params *new, *old;
	int update_bus = 0, update_clk = 0;
	u64 clk_rate = 0;
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *sde_cstate;
	int ret, i;
	struct msm_drm_private *priv;
	struct sde_kms *kms;
	bool params_changed = false;

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	kms = _sde_crtc_get_kms(crtc);
	if (!kms || !kms->catalog) {
		SDE_ERROR("invalid kms\n");
		return;
	}
	priv = kms->dev->dev_private;
	sde_crtc = to_sde_crtc(crtc);
	sde_cstate = to_sde_crtc_state(crtc->state);
	params_changed = ((commit_state == SDE_PERF_BEGIN_COMMIT)
				|| (commit_state == SDE_PERF_ENABLE_COMMIT)) ? true : false;

	SDE_DEBUG("crtc:%d commit_state:%d core_clk:%llu\n",
			DRMID(crtc), commit_state, kms->perf.core_clk_rate);

	mutex_lock(&sde_core_perf_lock);

	/*
	 * cache the performance numbers in the crtc prior to the
	 * crtc kickoff, so the same numbers are used during the
	 * perf update that happens post kickoff.
	 */
	if (params_changed)
		memcpy(&sde_crtc->new_perf, &sde_cstate->new_perf,
			sizeof(struct sde_core_perf_params));

	old = &sde_crtc->cur_perf;
	new = &sde_crtc->new_perf;

	/* avoid the voting in fence error case when there is decrease in BW vote */
	if ((commit_state == SDE_PERF_COMPLETE_COMMIT) && sde_crtc->handle_fence_error_bw_update) {
		new = &sde_crtc->cur_perf;
		SDE_EVT32(kms->dev, commit_state, sde_crtc->handle_fence_error_bw_update);
		sde_crtc->handle_fence_error_bw_update = false;
	}

	/* update votes through cesta and return early, when sde cesta is enabled */
	if (sde_cesta_is_enabled(DPUID(kms->dev))) {
		_sde_core_perf_crtc_cesta_update(kms, sde_crtc, sde_cstate,
				&sde_crtc->new_perf, commit_state, false);
		goto end;
	}

	if (_sde_core_perf_crtc_is_power_on(crtc) && (commit_state != SDE_PERF_DISABLE_COMMIT)) {
		_sde_core_perf_crtc_update_check(crtc, params_changed,
				&update_bus, &update_clk);
	} else {
		SDE_DEBUG("crtc=%d disable\n", crtc->base.id);
		memset(old, 0, sizeof(*old));
		memset(new, 0, sizeof(*new));
		update_bus = ~0;
		update_clk = 1;
	}

	for (i = 0; i < SDE_POWER_HANDLE_DBUS_ID_MAX; i++) {
		if (update_bus & BIT(i))
			_sde_core_perf_crtc_update_bus(kms, crtc, i);
	}

	if (kms->perf.bw_vote_mode == DISP_RSC_MODE &&
	    ((get_sde_rsc_current_state(DPUID(kms->dev)) != SDE_RSC_CLK_STATE
	      && params_changed) ||
	    (get_sde_rsc_current_state(DPUID(kms->dev)) == SDE_RSC_CLK_STATE)))
		sde_rsc_client_trigger_vote(sde_cstate->rsc_client,
				update_bus ? true : false);

	/*
	 * Update the clock after bandwidth vote to ensure
	 * bandwidth is available before clock rate is increased.
	 */
	if (update_clk) {
		clk_rate = _sde_core_perf_get_core_clk_rate(kms);

		SDE_EVT32(kms->dev, commit_state, clk_rate, old->core_clk_rate, new->core_clk_rate);
		ret = sde_power_clk_set_rate(&priv->phandle,
				kms->perf.clk_name, clk_rate, 0);
		if (ret) {
			SDE_ERROR("failed to set %s clock rate %llu\n",
					kms->perf.clk_name, clk_rate);
			mutex_unlock(&sde_core_perf_lock);
			return;
		}

		kms->perf.core_clk_rate = clk_rate;
		SDE_DEBUG("update clk rate = %lld HZ\n", clk_rate);
	}

end:
	trace_sde_perf_crtc_update(crtc->base.id,
		new->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_MNOC],
		new->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_MNOC],
		new->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_LLCC],
		new->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_LLCC],
		new->bw_ctl[SDE_POWER_HANDLE_DBUS_ID_EBI],
		new->max_per_pipe_ib[SDE_POWER_HANDLE_DBUS_ID_EBI],
		new->core_clk_rate, commit_state,
		update_bus, update_clk, params_changed);

	mutex_unlock(&sde_core_perf_lock);

}

#if IS_ENABLED(CONFIG_DEBUG_FS)

static ssize_t _sde_core_perf_threshold_high_write(struct file *file,
		    const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct sde_core_perf *perf = file->private_data;
	u32 threshold_high = 0;
	char buf[10];

	if (!perf)
		return -ENODEV;

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	/* end of string */

	if (kstrtouint(buf, 0, &threshold_high))
		return -EFAULT;

	if (threshold_high < SDE_PERF_THRESHOLD_HIGH_MIN)
		threshold_high = SDE_PERF_THRESHOLD_HIGH_MIN;

	perf->catalog->perf.max_bw_high = threshold_high;

	return count;
}

static ssize_t _sde_core_perf_threshold_high_read(struct file *file,
			char __user *buff, size_t count, loff_t *ppos)
{
	struct sde_core_perf *perf = file->private_data;
	int len = 0;
	char buf[20] = {'\0'};

	if (!perf)
		return -ENODEV;

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(buf, sizeof(buf),
			"%d\n", perf->catalog->perf.max_bw_high);

	if (len < 0 || len >= sizeof(buf))
		return 0;

	if ((count < sizeof(buf)) || copy_to_user(buff, buf, len))
		return -EFAULT;

	*ppos += len;   /* increase offset */

	return len;
}

static ssize_t _sde_core_perf_mode_write(struct file *file,
		    const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct sde_core_perf *perf = file->private_data;
	struct sde_perf_cfg *cfg = &perf->catalog->perf;
	u32 perf_mode = 0;
	char buf[10];
	int ret = 0;

	if (!perf)
		return -ENODEV;

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	/* end of string */

	if (kstrtouint(buf, 0, &perf_mode))
		return -EFAULT;

	if (perf_mode >= SDE_PERF_MODE_MAX)
		return -EFAULT;

	if (perf_mode == SDE_PERF_MODE_FIXED) {
		DRM_INFO("fix performance mode\n");
	} else if (perf_mode == SDE_PERF_MODE_MINIMUM) {
		/* run the driver with max clk and BW vote */
		perf->perf_tune.min_core_clk = perf->max_core_clk_rate;
		perf->perf_tune.min_bus_vote =
				(u64) cfg->max_bw_high * 1000;

		ret = sde_power_clk_set_rate(perf->phandle,
			perf->clk_name, perf->max_core_clk_rate, 0);
		if (ret) {
			SDE_ERROR("failed to set %s clock rate %llu\n",
					perf->clk_name,
					perf->max_core_clk_rate);

			perf->perf_tune.min_core_clk = 0;
			perf->perf_tune.min_bus_vote = 0;
			perf_mode = SDE_PERF_MODE_NORMAL;
		} else {
			DRM_INFO("minimum performance mode\n");
		}
		SDE_EVT32(perf->max_core_clk_rate, ret);
	} else if (perf_mode == SDE_PERF_MODE_NORMAL) {
		/* reset the perf tune params to 0 */
		perf->perf_tune.min_core_clk = 0;
		perf->perf_tune.min_bus_vote = 0;
		DRM_INFO("normal performance mode\n");
	}
	perf->perf_tune.mode = perf_mode;
	perf->perf_tune.mode_changed = true;

	return count;
}

static ssize_t _sde_core_perf_mode_read(struct file *file,
			char __user *buff, size_t count, loff_t *ppos)
{
	struct sde_core_perf *perf = file->private_data;
	int len = 0;
	char buf[SDE_PERF_MODE_STRING_SIZE] = {'\0'};

	if (!perf)
		return -ENODEV;

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(buf, sizeof(buf),
			"mode %d min_mdp_clk %llu min_bus_vote %llu\n",
			perf->perf_tune.mode,
			perf->perf_tune.min_core_clk,
			perf->perf_tune.min_bus_vote);
	if (len < 0 || len >= sizeof(buf))
		return 0;

	if ((count < sizeof(buf)) || copy_to_user(buff, buf, len))
		return -EFAULT;

	*ppos += len;   /* increase offset */

	return len;
}

static ssize_t _sde_core_perf_mmrm_write(struct file *file,
		    const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct sde_core_perf *perf = file->private_data;
	struct dss_module_power *mp = &perf->phandle->mp;
	char buf[20];
	int i, ret = 0;
	unsigned long requested_clk;
	struct dss_clk *clk = NULL;

	if (!perf || !mp)
		return -ENODEV;

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	/* end of string */

	if (kstrtoul(buf, 0, &requested_clk))
		return -EFAULT;

	for (i = 0; i < mp->num_clk; i++) {
		if (!strcmp(mp->clk_config[i].clk_name, perf->clk_name)) {
			clk = &mp->clk_config[i];
			break;
		}
	}
	if (!clk) {
		SDE_ERROR("Cannot find the clk %s\n", perf->clk_name);
		goto exit;
	}

	requested_clk = clk_round_rate(clk->clk, requested_clk);
	DRM_INFO("requesting limit rate:%lu for clk:%s\n",
		requested_clk, clk->clk_name);

	ret = sde_power_mmrm_set_clk_limit(clk,
		perf->phandle, requested_clk);
	if (ret)
		SDE_ERROR("Failed to set %s clock rate %lu\n",
			clk->clk_name, requested_clk);

exit:
	return count;
}

static ssize_t _sde_core_perf_mmrm_read(struct file *file,
			char __user *buff, size_t count, loff_t *ppos)
{
	struct sde_core_perf *perf = file->private_data;
	int len = 0;
	char buf[128] = {'\0'};

	if (!perf)
		return -ENODEV;

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(buf, sizeof(buf),
			"mmrm clk_limit:%llu clk:%s\n",
			sde_power_mmrm_get_requested_clk(perf->phandle,
			perf->clk_name), perf->clk_name);
	if (len < 0 || len >= sizeof(buf))
		return 0;

	if ((count < sizeof(buf)) || copy_to_user(buff, buf, len))
		return -EFAULT;

	*ppos += len;   /* increase offset */

	return len;
}

static const struct file_operations sde_core_perf_threshold_high_fops = {
	.open = simple_open,
	.read = _sde_core_perf_threshold_high_read,
	.write = _sde_core_perf_threshold_high_write,
};

static const struct file_operations sde_core_perf_mode_fops = {
	.open = simple_open,
	.read = _sde_core_perf_mode_read,
	.write = _sde_core_perf_mode_write,
};

static const struct file_operations sde_core_perf_mmrm_fops = {
	.open = simple_open,
	.read = _sde_core_perf_mmrm_read,
	.write = _sde_core_perf_mmrm_write,
};

static void sde_core_perf_debugfs_destroy(struct sde_core_perf *perf)
{
	debugfs_remove_recursive(perf->debugfs_root);
	perf->debugfs_root = NULL;
}

int sde_core_perf_debugfs_init(struct sde_core_perf *perf,
		struct dentry *parent)
{
	struct sde_mdss_cfg *catalog = perf->catalog;
	struct msm_drm_private *priv;
	struct sde_kms *sde_kms;

	priv = perf->dev->dev_private;
	if (!priv || !priv->kms) {
		SDE_ERROR("invalid KMS reference\n");
		return -EINVAL;
	}

	sde_kms = to_sde_kms(priv->kms);

	perf->debugfs_root = debugfs_create_dir("core_perf", parent);
	if (!perf->debugfs_root) {
		SDE_ERROR("failed to create core perf debugfs\n");
		return -EINVAL;
	}

	debugfs_create_u64("max_core_clk_rate", 0600, perf->debugfs_root,
			&perf->max_core_clk_rate);
	debugfs_create_u64("core_clk_rate", 0600, perf->debugfs_root,
			&perf->core_clk_rate);
	debugfs_create_u32("threshold_low", 0600, perf->debugfs_root,
			(u32 *)&catalog->perf.max_bw_low);
	debugfs_create_file("threshold_high", 0600, perf->debugfs_root,
			(u32 *)perf, &sde_core_perf_threshold_high_fops);
	debugfs_create_u32("min_core_ib", 0600, perf->debugfs_root,
			(u32 *)&catalog->perf.min_core_ib);
	debugfs_create_u32("min_llcc_ib", 0600, perf->debugfs_root,
			(u32 *)&catalog->perf.min_llcc_ib);
	debugfs_create_u32("min_dram_ib", 0600, perf->debugfs_root,
			(u32 *)&catalog->perf.min_dram_ib);
	debugfs_create_file("perf_mode", 0600, perf->debugfs_root,
			(u32 *)perf, &sde_core_perf_mode_fops);
	debugfs_create_file("mmrm_clk_cb", 0600, perf->debugfs_root,
			(u32 *)perf, &sde_core_perf_mmrm_fops);
	debugfs_create_u32("bw_vote_mode", 0600, perf->debugfs_root,
			&perf->bw_vote_mode);
	debugfs_create_bool("bw_vote_mode_updated", 0600, perf->debugfs_root,
			&perf->bw_vote_mode_updated);
	debugfs_create_u64("fix_core_clk_rate", 0600, perf->debugfs_root,
			&perf->fix_core_clk_rate);
	debugfs_create_u64("fix_core_ib_vote", 0600, perf->debugfs_root,
			&perf->fix_core_ib_vote);
	debugfs_create_u64("fix_core_ab_vote", 0600, perf->debugfs_root,
			&perf->fix_core_ab_vote);
	debugfs_create_u32("sys_cache_enable", 0600, perf->debugfs_root,
			&perf->sys_cache_enabled);

	debugfs_create_u32("uidle_perf_cnt", 0600, perf->debugfs_root,
			&sde_kms->catalog->uidle_cfg.debugfs_perf);
	debugfs_create_u32("uidle_fal10_target_idle_time_us", 0600, perf->debugfs_root,
			&sde_kms->catalog->uidle_cfg.fal10_target_idle_time);
	debugfs_create_u32("uidle_fal1_target_idle_time_us", 0600, perf->debugfs_root,
			&sde_kms->catalog->uidle_cfg.fal1_target_idle_time);
	debugfs_create_u32("uidle_fal10_threshold_us", 0600, perf->debugfs_root,
			&sde_kms->catalog->uidle_cfg.fal10_threshold);
	debugfs_create_u32("uidle_fal1_max_threshold", 0600, perf->debugfs_root,
			&sde_kms->catalog->uidle_cfg.fal1_max_threshold);
	debugfs_create_bool("uidle_enable", 0600, perf->debugfs_root,
			&sde_kms->catalog->uidle_cfg.debugfs_ctrl);
	debugfs_create_bool("uidle_status", 0400, perf->debugfs_root,
			&sde_kms->perf.uidle_enabled);

	return 0;
}
#else
static void sde_core_perf_debugfs_destroy(struct sde_core_perf *perf)
{
}

int sde_core_perf_debugfs_init(struct sde_core_perf *perf,
		struct dentry *parent)
{
	return 0;
}
#endif /* CONFIG_DEBUG_FS */

void sde_core_perf_destroy(struct sde_core_perf *perf)
{
	if (!perf) {
		SDE_ERROR("invalid parameters\n");
		return;
	}

	sde_core_perf_debugfs_destroy(perf);
	perf->max_core_clk_rate = 0;
	perf->core_clk = NULL;
	perf->clk_name = NULL;
	perf->phandle = NULL;
	perf->catalog = NULL;
	perf->dev = NULL;
}

int sde_core_perf_init(struct sde_core_perf *perf,
		struct drm_device *dev,
		struct sde_mdss_cfg *catalog,
		struct sde_power_handle *phandle,
		char *clk_name)
{
	if (!perf || !dev || !catalog || !phandle || !clk_name) {
		SDE_ERROR("invalid parameters\n");
		return -EINVAL;
	}

	perf->dev = dev;
	perf->catalog = catalog;
	perf->phandle = phandle;
	perf->clk_name = clk_name;
	perf->sde_rsc_available = is_sde_rsc_available(DPUID(dev));
	/* set default mode */
	if (perf->sde_rsc_available)
		perf->bw_vote_mode = DISP_RSC_MODE;
	else
		perf->bw_vote_mode = APPS_RSC_MODE;

	/* core clk will be part of cesta node, when cesta is enabled */
	perf->cesta_phandle = sde_cesta_get_phandle(DPUID(dev));
	phandle = perf->cesta_phandle ? perf->cesta_phandle : phandle;

	perf->core_clk = sde_power_clk_get_clk(phandle, clk_name);
	if (!perf->core_clk) {
		SDE_ERROR("invalid core clk\n");
		goto err;
	}

	perf->max_core_clk_rate = sde_power_clk_get_max_rate(phandle, clk_name);
	if (!perf->max_core_clk_rate) {
		SDE_DEBUG("optional max core clk rate, use default\n");
		perf->max_core_clk_rate = SDE_PERF_DEFAULT_MAX_CORE_CLK_RATE;
	}
	perf->sys_cache_enabled = 0xffffffff;

	return 0;

err:
	sde_core_perf_destroy(perf);
	return -ENODEV;
}
