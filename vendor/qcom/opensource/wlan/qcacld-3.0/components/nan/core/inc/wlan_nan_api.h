/*
 * Copyright (c) 2017-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC: contains nan definitions exposed to other modules
 */

#ifndef _WLAN_NAN_API_H_
#define _WLAN_NAN_API_H_

#include "wlan_objmgr_peer_obj.h"
#include "wlan_policy_mgr_public_struct.h"
#include "qdf_status.h"
#include <nan_public_structs.h>
#include <wlan_cp_stats_chipset_stats.h>

#ifdef WLAN_FEATURE_NAN

#include "../src/nan_main_i.h"

struct wlan_objmgr_psoc;

/**
 * nan_init: initializes NAN component, called by dispatcher init
 *
 * Return: status of operation
 */
QDF_STATUS nan_init(void);

/**
 * nan_deinit: de-initializes NAN component, called by dispatcher init
 *
 * Return: status of operation
 */
QDF_STATUS nan_deinit(void);

/**
 * nan_psoc_enable: psoc enable API for NANitioning component
 * @psoc: pointer to PSOC
 *
 * Return: status of operation
 */
QDF_STATUS nan_psoc_enable(struct wlan_objmgr_psoc *psoc);

/**
 * nan_psoc_disable: psoc disable API for NANitioning component
 * @psoc: pointer to PSOC
 *
 * Return: status of operation
 */
QDF_STATUS nan_psoc_disable(struct wlan_objmgr_psoc *psoc);

/**
 * nan_get_peer_priv_obj: get NAN priv object from peer object
 * @peer: pointer to peer object
 *
 * Return: pointer to NAN peer private object
 */
static inline
struct nan_peer_priv_obj *nan_get_peer_priv_obj(struct wlan_objmgr_peer *peer)
{
	struct nan_peer_priv_obj *obj;

	if (!peer) {
		nan_err("peer is null");
		return NULL;
	}
	obj = wlan_objmgr_peer_get_comp_private_obj(peer, WLAN_UMAC_COMP_NAN);

	return obj;
}

/**
 * nan_get_vdev_priv_obj: get NAN priv object from vdev object
 * @vdev: pointer to vdev object
 *
 * Return: pointer to NAN vdev private object
 */
static inline
struct nan_vdev_priv_obj *nan_get_vdev_priv_obj(struct wlan_objmgr_vdev *vdev)
{
	struct nan_vdev_priv_obj *obj;

	if (!vdev) {
		nan_err("vdev is null");
		return NULL;
	}
	obj = wlan_objmgr_vdev_get_comp_private_obj(vdev, WLAN_UMAC_COMP_NAN);

	return obj;
}

/**
 * nan_get_psoc_priv_obj: get NAN priv object from psoc object
 * @psoc: pointer to psoc object
 *
 * Return: pointer to NAN psoc private object
 */
static inline
struct nan_psoc_priv_obj *nan_get_psoc_priv_obj(struct wlan_objmgr_psoc *psoc)
{
	struct nan_psoc_priv_obj *obj;

	if (!psoc) {
		nan_err("psoc is null");
		return NULL;
	}
	obj = wlan_objmgr_psoc_get_comp_private_obj(psoc, WLAN_UMAC_COMP_NAN);

	return obj;
}

/**
 * nan_psoc_get_tx_ops: get TX ops from the NAN private object
 * @psoc: pointer to psoc object
 *
 * Return: pointer to TX op callback
 */
static inline
struct wlan_nan_tx_ops *nan_psoc_get_tx_ops(struct wlan_objmgr_psoc *psoc)
{
	struct nan_psoc_priv_obj *nan_priv;

	if (!psoc) {
		nan_err("psoc is null");
		return NULL;
	}

	nan_priv = nan_get_psoc_priv_obj(psoc);
	if (!nan_priv) {
		nan_err("psoc private object is null");
		return NULL;
	}

	return &nan_priv->tx_ops;
}

/**
 * nan_psoc_get_rx_ops: get RX ops from the NAN private object
 * @psoc: pointer to psoc object
 *
 * Return: pointer to RX op callback
 */
static inline
struct wlan_nan_rx_ops *nan_psoc_get_rx_ops(struct wlan_objmgr_psoc *psoc)
{
	struct nan_psoc_priv_obj *nan_priv;

	if (!psoc) {
		nan_err("psoc is null");
		return NULL;
	}

	nan_priv = nan_get_psoc_priv_obj(psoc);
	if (!nan_priv) {
		nan_err("psoc private object is null");
		return NULL;
	}

	return &nan_priv->rx_ops;
}

/**
 * wlan_nan_get_connection_info: Get NAN related connection info
 * @psoc: pointer to psoc object
 * @conn_info: Coonection info structure pointer
 *
 * Return: status of operation
 */
QDF_STATUS
wlan_nan_get_connection_info(struct wlan_objmgr_psoc *psoc,
			     struct policy_mgr_vdev_entry_info *conn_info);

/**
 * wlan_nan_get_disc_24g_ch_freq: Get NAN Disc 2.4GHz channel frequency
 * @psoc: pointer to psoc object
 *
 * Return: NAN Disc 2.4GHz channel frequency
 */
qdf_freq_t wlan_nan_get_disc_24g_ch_freq(struct wlan_objmgr_psoc *psoc);

/**
 * wlan_nan_get_disc_5g_ch_freq: Get NAN Disc 5G channel frequency
 * @psoc: pointer to psoc object
 *
 * Return: NAN Disc 5G channel frequency
 */
uint32_t wlan_nan_get_disc_5g_ch_freq(struct wlan_objmgr_psoc *psoc);

/**
 * wlan_nan_get_5ghz_social_ch_freq(): Get NAN 5GHz social channel
 * @pdev: PDEV object
 *
 * This API returns 5745(channel-149) if it's valid as per regulatory rules
 * and returns 5220(channel-44) otherwise.
 *
 * Return: NAN social channel frequency
 */
qdf_freq_t
wlan_nan_get_5ghz_social_ch_freq(struct wlan_objmgr_pdev *pdev);

/**
 * wlan_nan_get_sap_conc_support: Get NAN+SAP conc support
 * @psoc: pointer to psoc object
 *
 * Return: True if NAN+SAP supported else false
 */
bool wlan_nan_get_sap_conc_support(struct wlan_objmgr_psoc *psoc);

/**
 * nan_disable_cleanup: Cleanup NAN state upon NAN disable
 * @psoc: pointer to psoc object
 *
 * Return: Cleanup NAN state upon NAN disable
 */
QDF_STATUS nan_disable_cleanup(struct wlan_objmgr_psoc *psoc);

/**
 * wlan_nan_is_beamforming_supported- Get support for beamforing
 * @psoc: pointer to psoc object
 *
 * Return: True if beamforming is supported, false if not.
 */
bool wlan_nan_is_beamforming_supported(struct wlan_objmgr_psoc *psoc);

/**
 * wlan_is_nan_allowed_on_freq() - Check if NAN is allowed on given freq
 * @pdev: pdev context
 * @freq: Frequency to be checked
 *
 * Check if NAN/NDP can be enabled on given frequency.
 *
 * Return: True if NAN is allowed on the given frequency
 */
bool wlan_is_nan_allowed_on_freq(struct wlan_objmgr_pdev *pdev, uint32_t freq);

/**
 * nan_handle_emlsr_concurrency()- Handle NAN+eMLSR concurrency
 * @psoc: pointer to psoc object
 * @nan_enable: Carries true if NAN is getting enabled.
 *		Carries false upon NAN enable failure/NAN disabled indication
 *
 * Return: void
 */
void nan_handle_emlsr_concurrency(struct wlan_objmgr_psoc *psoc,
				  bool nan_enable);

/**
 * wlan_nan_is_sta_sap_nan_allowed() - Check if STA + SAP + NAN allowed
 * @psoc: pointer to psoc object
 *
 * Return true if STA + SAP + NAN allowed
 */
bool wlan_nan_is_sta_sap_nan_allowed(struct wlan_objmgr_psoc *psoc);

/**
 * wlan_nan_sap_override_freq() - Return frequency of NAN 2GHz channel
 * @psoc: pointer to psoc object
 * @vdev_id: Vdev Id
 * @chan_freq: current frequency
 *
 * Return: valid NAN frequency
 */
qdf_freq_t wlan_nan_sap_override_freq(struct wlan_objmgr_psoc *psoc,
				      uint32_t vdev_id,
				      qdf_freq_t chan_freq);

#else /* WLAN_FEATURE_NAN */
static inline QDF_STATUS nan_init(void)
{
	return QDF_STATUS_SUCCESS;
}

static inline QDF_STATUS nan_deinit(void)
{
	return QDF_STATUS_SUCCESS;
}

static inline QDF_STATUS nan_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static inline QDF_STATUS nan_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static inline QDF_STATUS
wlan_nan_get_connection_info(struct wlan_objmgr_psoc *psoc,
			     struct policy_mgr_vdev_entry_info *conn_info)
{
	return QDF_STATUS_E_FAILURE;
}

static inline qdf_freq_t
wlan_nan_get_disc_24g_ch_freq(struct wlan_objmgr_psoc *psoc)
{
	return 0;
}

static inline uint32_t
wlan_nan_get_disc_5g_ch_freq(struct wlan_objmgr_psoc *psoc)
{
	return 0;
}

static inline
qdf_freq_t wlan_nan_get_5ghz_social_ch_freq(struct wlan_objmgr_pdev *pdev)
{
	return 0;
}

static inline
bool wlan_nan_get_sap_conc_support(struct wlan_objmgr_psoc *psoc)
{
	return false;
}

static inline
QDF_STATUS nan_disable_cleanup(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_E_FAILURE;
}

static inline
bool wlan_nan_is_beamforming_supported(struct wlan_objmgr_psoc *psoc)
{
	return false;
}

static inline
bool wlan_is_nan_allowed_on_freq(struct wlan_objmgr_pdev *pdev, uint32_t freq)
{
	return false;
}

static inline void
nan_handle_emlsr_concurrency(struct wlan_objmgr_psoc *psoc, bool nan_enable)
{}

static inline
bool wlan_nan_is_sta_sap_nan_allowed(struct wlan_objmgr_psoc *psoc)
{
	return false;
}

static inline
qdf_freq_t wlan_nan_sap_override_freq(struct wlan_objmgr_psoc *psoc,
				      uint32_t vdev_id,
				      qdf_freq_t chan_freq)
{
	return chan_freq;
}

#endif /* WLAN_FEATURE_NAN */

#if defined(WLAN_FEATURE_NAN) && defined(WLAN_FEATURE_11BE_MLO)
/**
 * wlan_is_mlo_sta_nan_ndi_allowed()- Get support for MLO STA +
 * NAN Disc + NDI concurrency
 * @psoc: pointer to psoc object
 *
 * Return: True if mlo sta + nan + ndi concurrency allowed or not.
 */
bool wlan_is_mlo_sta_nan_ndi_allowed(struct wlan_objmgr_psoc *psoc);
#else
static inline bool
wlan_is_mlo_sta_nan_ndi_allowed(struct wlan_objmgr_psoc *psoc)
{
	return false;
}
#endif

#if defined(WLAN_FEATURE_NAN) && defined(WLAN_CHIPSET_STATS)
/**
 * nan_cstats_log_nan_enable_resp_evt() - Chipset stats NAN enable
 * response event
 *
 * @nan_event: pointer to nan_event_params object
 *
 * Return: void
 */
void nan_cstats_log_nan_enable_resp_evt(struct nan_event_params *nan_event);

/**
 * nan_cstats_log_nan_disable_resp_evt() - Chipset stats NAN disable
 * response event
 *
 * @vdev_id: vdev ID
 * @psoc: pointer to psoc object
 *
 * Return: void
 */
void
nan_cstats_log_nan_disable_resp_evt(uint8_t vdev_id,
				    struct wlan_objmgr_psoc *psoc);
#else
static inline void
nan_cstats_log_nan_enable_resp_evt(struct nan_event_params *nan_event)
{
}

static inline void
nan_cstats_log_nan_disable_resp_evt(uint8_t vdev_id,
				    struct wlan_objmgr_psoc *psoc)
{
}
#endif /* WLAN_FEATURE_NAN && WLAN_CHIPSET_STATS */
#endif /* _WLAN_NAN_API_H_ */
