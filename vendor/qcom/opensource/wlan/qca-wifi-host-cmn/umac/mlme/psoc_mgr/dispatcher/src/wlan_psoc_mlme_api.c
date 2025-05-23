/*
 * Copyright (c) 2019-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC: Implements PSOC MLME public APIs
 */

#include <wlan_objmgr_psoc_obj.h>
#include <wlan_mlme_dbg.h>
#include <include/wlan_psoc_mlme.h>
#include <wlan_psoc_mlme_api.h>
#include <qdf_module.h>
#include "cfg_ucfg_api.h"
#include "wlan_vdev_mgr_tgt_if_rx_api.h"
#include <qdf_platform.h>
#include "wlan_utility.h"

QDF_STATUS
wlan_psoc_mlme_get_11be_capab(struct wlan_objmgr_psoc *psoc, bool *val)
{
	struct psoc_mlme_obj *psoc_mlme;

	psoc_mlme = wlan_psoc_mlme_get_cmpt_obj(psoc);
	if (!psoc_mlme) {
		mlme_err("psoc_mlme is NULL");
		return QDF_STATUS_E_FAILURE;
	}

	*val = psoc_mlme->psoc_cfg.phy_config.eht_cap;
	return QDF_STATUS_SUCCESS;
}

qdf_export_symbol(wlan_psoc_mlme_get_11be_capab);

QDF_STATUS
wlan_psoc_mlme_set_11be_capab(struct wlan_objmgr_psoc *psoc, bool val)
{
	struct psoc_mlme_obj *psoc_mlme;

	psoc_mlme = wlan_psoc_mlme_get_cmpt_obj(psoc);
	if (!psoc_mlme) {
		mlme_err("psoc_mlme is NULL");
		return QDF_STATUS_E_FAILURE;
	}

	psoc_mlme->psoc_cfg.phy_config.eht_cap &= val;
	return QDF_STATUS_SUCCESS;
}

qdf_export_symbol(wlan_psoc_mlme_set_11be_capab);

struct psoc_mlme_obj *wlan_psoc_mlme_get_cmpt_obj(struct wlan_objmgr_psoc *psoc)
{
	struct psoc_mlme_obj *psoc_mlme;

	psoc_mlme = wlan_objmgr_psoc_get_comp_private_obj(psoc,
							  WLAN_UMAC_COMP_MLME);
	if (!psoc_mlme) {
		mlme_err("PSOC MLME component object is NULL");
		return NULL;
	}

	return psoc_mlme;
}

qdf_export_symbol(wlan_psoc_mlme_get_cmpt_obj);

#ifdef WLAN_FEATURE_11BE_MLO_ADV_FEATURE
static void wlan_mlme_psoc_peer_trans_hist_remove_back(qdf_list_t *peer_history)
{
	struct wlan_peer_tbl_trans_entry *peer_trans_entry;
	qdf_list_node_t *node;

	qdf_list_remove_back(peer_history, &node);
	peer_trans_entry = qdf_container_of(node,
					    struct wlan_peer_tbl_trans_entry,
					    node);
	qdf_mem_free(peer_trans_entry);
}

QDF_STATUS
wlan_mlme_psoc_peer_tbl_trans_add_entry(struct wlan_objmgr_psoc *psoc,
					struct wlan_peer_tbl_trans_entry *peer_trans_entry)
{
	struct psoc_mlme_obj *psoc_mlme;
	qdf_list_t *peer_hist_list;
	QDF_STATUS status;

	psoc_mlme = wlan_psoc_mlme_get_cmpt_obj(psoc);
	if (!psoc_mlme) {
		mlme_err("PSOC MLME component object is NULL");
		return QDF_STATUS_E_FAILURE;
	}

	wlan_psoc_obj_lock(psoc);
	peer_hist_list = &psoc_mlme->peer_history_list;
	if (qdf_list_size(peer_hist_list) == MAX_PEER_HIST_LIST_SIZE)
		wlan_mlme_psoc_peer_trans_hist_remove_back(peer_hist_list);

	status = qdf_list_insert_front(peer_hist_list, &peer_trans_entry->node);
	wlan_psoc_obj_unlock(psoc);

	return status;
}

void wlan_mlme_psoc_flush_peer_trans_history(struct wlan_objmgr_psoc *psoc)
{
	struct psoc_mlme_obj *psoc_mlme;
	qdf_list_t *peer_hist_list;

	psoc_mlme = wlan_psoc_mlme_get_cmpt_obj(psoc);
	if (!psoc_mlme) {
		mlme_err("PSOC MLME component object is NULL");
		return;
	}
	wlan_psoc_obj_lock(psoc);
	peer_hist_list = &psoc_mlme->peer_history_list;
	while (qdf_list_size(peer_hist_list))
		wlan_mlme_psoc_peer_trans_hist_remove_back(peer_hist_list);

	qdf_list_destroy(peer_hist_list);
	wlan_psoc_obj_unlock(psoc);
}
#endif

mlme_psoc_ext_t *wlan_psoc_mlme_get_ext_hdl(struct wlan_objmgr_psoc *psoc)
{
	struct psoc_mlme_obj *psoc_mlme;

	psoc_mlme = wlan_psoc_mlme_get_cmpt_obj(psoc);
	if (psoc_mlme)
		return psoc_mlme->ext_psoc_ptr;

	return NULL;
}

qdf_export_symbol(wlan_psoc_mlme_get_ext_hdl);

void wlan_psoc_mlme_set_ext_hdl(struct psoc_mlme_obj *psoc_mlme,
				mlme_psoc_ext_t *psoc_ext_hdl)
{
	psoc_mlme->ext_psoc_ptr = psoc_ext_hdl;
}

void wlan_psoc_set_phy_config(struct wlan_objmgr_psoc *psoc,
			      struct psoc_phy_config *phy_config)
{
	struct psoc_mlme_obj *mlme_psoc_obj;
	struct psoc_phy_config *config;

	if (!phy_config) {
		mlme_err("phy_config is NUll");
		return;
	}
	mlme_psoc_obj = wlan_psoc_mlme_get_cmpt_obj(psoc);
	if (!mlme_psoc_obj)
		return;

	config = &mlme_psoc_obj->psoc_cfg.phy_config;

	qdf_mem_copy(config, phy_config, sizeof(*config));
}

static void mlme_init_cfg(struct wlan_objmgr_psoc *psoc)
{
	struct psoc_mlme_obj *mlme_psoc_obj;

	mlme_psoc_obj = wlan_psoc_mlme_get_cmpt_obj(psoc);

	if (!mlme_psoc_obj)
		return;

	wlan_cm_init_score_config(psoc, &mlme_psoc_obj->psoc_cfg.score_config);
	mlme_psoc_obj->psoc_cfg.phy_config.max_chan_switch_ie =
		cfg_get(psoc, CFG_MLME_MAX_CHAN_SWITCH_IE_ENABLE);
	mlme_psoc_obj->psoc_cfg.phy_config.eht_cap =
		cfg_default(CFG_MLME_11BE_TARGET_CAPAB);
	mlme_psoc_obj->psoc_cfg.mlo_config.reconfig_reassoc_en =
		cfg_get(psoc, CFG_MLME_MLO_RECONFIG_REASSOC_ENABLE);

	wlan_minidump_log(mlme_psoc_obj, sizeof(*mlme_psoc_obj), psoc,
			  WLAN_MD_OBJMGR_PSOC_MLME, "psoc_mlme");
}

QDF_STATUS mlme_psoc_open(struct wlan_objmgr_psoc *psoc)
{
	mlme_init_cfg(psoc);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS mlme_psoc_close(struct wlan_objmgr_psoc *psoc)
{
	if (qdf_is_recovering())
		tgt_vdev_mgr_reset_response_timer_info(psoc);
	return QDF_STATUS_SUCCESS;
}

qdf_export_symbol(wlan_psoc_mlme_set_ext_hdl);
