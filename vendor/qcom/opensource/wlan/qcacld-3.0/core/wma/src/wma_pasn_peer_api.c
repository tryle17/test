/*
 * Copyright (c) 2022,2024 Qualcomm Innovation Center, Inc. All rights reserved.
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
 *  DOC: wma_pasn_peer_api.c
 *  This file contains PASN peer related operations.
 */

#include "wma.h"
#include "wma_api.h"
#include "wmi_unified_api.h"
#include "wmi_unified.h"
#include "qdf_types.h"
#include "qdf_mem.h"
#include "wma_types.h"
#include "wma_internal.h"
#include "wma_pasn_peer_api.h"
#include "wifi_pos_pasn_api.h"
#include "wifi_pos_api.h"
#include "init_deinit_lmac.h"
#include "wlan_nan_api_i.h"

QDF_STATUS
wma_pasn_peer_remove(struct wlan_objmgr_psoc *psoc,
		     struct qdf_mac_addr *peer_addr,
		     uint8_t vdev_id, bool no_fw_peer_delete)
{
	tp_wma_handle wma = cds_get_context(QDF_MODULE_ID_WMA);
	struct peer_delete_cmd_params del_param = {0};
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	uint8_t peer_vdev_id;

	if (!wma) {
		wma_err("wma_handle is NULL");
		return QDF_STATUS_E_INVAL;
	}

	if (!wma_objmgr_peer_exist(wma, peer_addr->bytes, &peer_vdev_id)) {
		wma_err("peer doesn't exist peer_addr " QDF_MAC_ADDR_FMT " vdevid %d",
			QDF_MAC_ADDR_REF(peer_addr->bytes), vdev_id);
		return QDF_STATUS_E_INVAL;
	}

	if (peer_vdev_id != vdev_id) {
		wma_err("peer " QDF_MAC_ADDR_FMT " is on vdev id %d but delete req on vdevid %d",
			QDF_MAC_ADDR_REF(peer_addr->bytes),
			peer_vdev_id, vdev_id);
		return QDF_STATUS_E_INVAL;
	}

	if (no_fw_peer_delete)
		goto host_peer_delete;

	del_param.vdev_id = vdev_id;
	qdf_status = wmi_unified_peer_delete_send(wma->wmi_handle,
						  peer_addr->bytes,
						  &del_param);
	if (QDF_IS_STATUS_ERROR(qdf_status)) {
		wma_err("Peer delete could not be sent to firmware %d",
			qdf_status);
		qdf_status = QDF_STATUS_E_FAILURE;
	}

host_peer_delete:
	wma_remove_objmgr_peer(wma, wma->interfaces[vdev_id].vdev,
			       peer_addr->bytes);

	return qdf_status;
}

QDF_STATUS wma_nan_pasn_peer_remove(struct wlan_objmgr_psoc *psoc,
				    uint8_t vdev_id,
				    struct qdf_mac_addr *peer_addr,
				    uint8_t type, bool objmgr_peer_delete)
{
	tp_wma_handle wma = cds_get_context(QDF_MODULE_ID_WMA);
	struct peer_delete_cmd_params del_param = {0};
	uint8_t peer_vdev_id;
	struct wma_target_req *del_req;
	struct pasn_peer_del_rsp_params *peer_del_resp;
	QDF_STATUS qdf_status;

	if (cds_is_driver_recovering())
		return QDF_STATUS_E_FAILURE;

	if (!wma) {
		wma_err("wma_handle is NULL");
		return QDF_STATUS_E_INVAL;
	}

	if (!wma_objmgr_peer_exist(wma, peer_addr->bytes, &peer_vdev_id)) {
		wma_err("peer doesn't exist peer_addr " QDF_MAC_ADDR_FMT " vdevid %d",
			QDF_MAC_ADDR_REF(peer_addr->bytes), vdev_id);
		return QDF_STATUS_E_INVAL;
	}

	if (peer_vdev_id != vdev_id) {
		wma_err("peer " QDF_MAC_ADDR_FMT " is on vdev id %d but delete req on vdevid %d",
			QDF_MAC_ADDR_REF(peer_addr->bytes),
			peer_vdev_id, vdev_id);
		return QDF_STATUS_E_INVAL;
	}

	if (objmgr_peer_delete)
		goto host_peer_delete;

	del_param.vdev_id = vdev_id;
	qdf_status = wmi_unified_peer_delete_send(wma->wmi_handle,
						  peer_addr->bytes,
						  &del_param);
	if (QDF_IS_STATUS_ERROR(qdf_status)) {
		wma_err("Peer delete could not be sent to target %d",
			qdf_status);
		return QDF_STATUS_E_FAILURE;
	}

	if (wmi_service_enabled(wma->wmi_handle,
				wmi_service_sync_delete_cmds)) {
		peer_del_resp = qdf_mem_malloc(sizeof(*peer_del_resp));
		if (!peer_del_resp)
			return QDF_STATUS_E_INVAL;

		peer_del_resp->vdev_id = vdev_id;
		peer_del_resp->status = QDF_STATUS_SUCCESS;

		wma_debug("Wait for the peer delete. vdev_id %d", vdev_id);
		del_req = wma_fill_hold_req(wma, vdev_id,
					    WMA_DELETE_STA_REQ,
					     type, NULL,
					     peer_del_resp,
					     WMA_DELETE_STA_TIMEOUT);
		if (!del_req) {
			wma_err("Failed to allocate request vdev_id %d",
				vdev_id);
			qdf_mem_free(peer_del_resp);
			return QDF_STATUS_E_FAILURE;
		}

		return QDF_STATUS_SUCCESS;
	}

host_peer_delete:
	wma_remove_objmgr_peer(wma, wma->interfaces[vdev_id].vdev,
			       peer_addr->bytes);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
wma_pasn_peer_create(struct wlan_objmgr_psoc *psoc,
		     struct qdf_mac_addr *peer_addr, uint8_t vdev_id,
		     uint8_t pasn_peer_msg_type)
{
	tp_wma_handle wma = cds_get_context(QDF_MODULE_ID_WMA);
	target_resource_config *wlan_res_cfg;
	struct wlan_objmgr_peer *obj_peer;
	struct wma_target_req *wma_req;
	struct peer_create_rsp_params *peer_create_rsp;
	struct peer_create_params param;
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	bool is_tgt_peer_conf_supported;

	if (!wma) {
		wma_err("wma_handle is NULL");
		return QDF_STATUS_E_INVAL;
	}

	wlan_res_cfg = lmac_get_tgt_res_cfg(psoc);
	if (!wlan_res_cfg) {
		wma_err("psoc target res cfg is null");
		return QDF_STATUS_E_INVAL;
	}

	if (wma->interfaces[vdev_id].peer_count >=
	    wlan_res_cfg->num_peers) {
		wma_err("the peer count exceeds the limit %d",
			wma->interfaces[vdev_id].peer_count);
		return QDF_STATUS_E_FAILURE;
	}

	if (qdf_is_macaddr_group(peer_addr) ||
	    qdf_is_macaddr_zero(peer_addr)) {
		wma_err("Invalid peer address received reject it");
		return QDF_STATUS_E_FAILURE;
	}

	wma_acquire_wakelock(&wma->wmi_cmd_rsp_wake_lock,
			     WMA_PEER_CREATE_RESPONSE_TIMEOUT);
	/*
	 * The peer object should be created before sending the WMI peer
	 * create command to firmware.
	 */
	obj_peer = wma_create_objmgr_peer(wma, vdev_id, NULL,
					  WMI_PEER_TYPE_PASN, NULL);
	if (!obj_peer) {
		wma_release_wakelock(&wma->wmi_cmd_rsp_wake_lock);
		return QDF_STATUS_E_FAILURE;
	}

	param.peer_addr = peer_addr->bytes;
	param.peer_type = WMI_PEER_TYPE_PASN;
	param.vdev_id = vdev_id;
	if (wmi_unified_peer_create_send(wma->wmi_handle,
					 &param) != QDF_STATUS_SUCCESS) {
		wma_err("Unable to create peer in Target");
		wlan_objmgr_peer_obj_delete(obj_peer);
		wma_release_wakelock(&wma->wmi_cmd_rsp_wake_lock);

		return QDF_STATUS_E_FAILURE;
	}

	/*
	 * If fw doesn't advertise peer create confirm event support,
	 * use the legacy peer create API
	 */
	is_tgt_peer_conf_supported =
		wlan_psoc_nif_fw_ext_cap_get(wma->psoc,
					     WLAN_SOC_F_PEER_CREATE_RESP);
	if (!is_tgt_peer_conf_supported) {
		wma_release_wakelock(&wma->wmi_cmd_rsp_wake_lock);
		return QDF_STATUS_SUCCESS;
	}

	peer_create_rsp = qdf_mem_malloc(sizeof(*peer_create_rsp));
	if (!peer_create_rsp) {
		wlan_objmgr_peer_obj_delete(obj_peer);
		wma_release_wakelock(&wma->wmi_cmd_rsp_wake_lock);
		return QDF_STATUS_E_NOMEM;
	}

	qdf_mem_copy(peer_create_rsp->peer_mac.bytes, peer_addr->bytes,
		     QDF_MAC_ADDR_SIZE);
	wma_req = wma_fill_hold_req(wma, vdev_id, WMA_PEER_CREATE_REQ,
				    pasn_peer_msg_type, peer_addr->bytes,
				    peer_create_rsp,
				    WMA_PEER_CREATE_RESPONSE_TIMEOUT);
	if (!wma_req) {
		wma_err("vdev:%d failed to fill peer create req", vdev_id);
		wma_remove_peer_req(wma, vdev_id, pasn_peer_msg_type,
				    peer_addr);

		if (pasn_peer_msg_type == WMA_PASN_PEER_CREATE_RESPONSE)
			wma_pasn_peer_remove(psoc, peer_addr, vdev_id, false);
		else if (pasn_peer_msg_type ==
			   WMA_NAN_PASN_PEER_CREATE_RESPONSE)
			wma_nan_pasn_peer_remove(psoc, vdev_id, peer_addr,
					WMA_NAN_PASN_PEER_DELETE_RESPONSE,
					false);

		wma_release_wakelock(&wma->wmi_cmd_rsp_wake_lock);
		qdf_mem_free(peer_create_rsp);
		return QDF_STATUS_E_FAILURE;
	}

	wma_debug("Created PASN peer peer_addr " QDF_MAC_ADDR_FMT " vdev_id %d",
		  QDF_MAC_ADDR_REF(peer_addr->bytes), vdev_id);

	return status;
}

QDF_STATUS
wma_pasn_handle_peer_create_conf(tp_wma_handle wma,
				 struct qdf_mac_addr *peer_mac,
				 uint8_t req_msg_type, QDF_STATUS status,
				 uint8_t vdev_id)
{
	struct wlan_lmac_if_wifi_pos_rx_ops *rx_ops;
	struct wlan_objmgr_vdev *vdev;
	enum QDF_OPMODE mode;

	if (status)
		wma_pasn_peer_remove(wma->psoc, peer_mac, vdev_id,
				     QDF_IS_STATUS_ERROR(status));

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(wma->psoc, vdev_id,
						    WLAN_LEGACY_WMA_ID);
	if (!vdev) {
		wma_err("Vdev is NULL");
		return QDF_STATUS_E_NULL_VALUE;
	}

	mode = wlan_vdev_mlme_get_opmode(vdev);
	wlan_objmgr_vdev_release_ref(vdev, WLAN_LEGACY_WMA_ID);

	if (req_msg_type == WMA_PASN_PEER_CREATE_RESPONSE) {
		/*
		 * Only in I-sta case update the wifi pos module to
		 * track the peer to initiate PASN authentication.
		 * For R-STA, return from here.
		 */
		if (mode != QDF_STA_MODE) {
			wma_debug("PASN opmode:%d is not sta", mode);
			return QDF_STATUS_SUCCESS;
		}

		rx_ops = wifi_pos_get_rx_ops(wma->psoc);
		if (!rx_ops || !rx_ops->wifi_pos_ranging_peer_create_rsp_cb) {
			wma_err("%s is null",
				!rx_ops ? "rx_ops" : "rx_ops->ranging_peer_cb");
			return QDF_STATUS_E_NULL_VALUE;
		}

		rx_ops->wifi_pos_ranging_peer_create_rsp_cb(wma->psoc, vdev_id,
							    peer_mac, status);
	} else if (req_msg_type == WMA_NAN_PASN_PEER_CREATE_RESPONSE) {
		if (mode != QDF_NAN_DISC_MODE) {
			wma_debug("PASN opmode:%d is not NAN", mode);
			return QDF_STATUS_SUCCESS;
		}

		wlan_nan_handle_pasn_peer_create_rsp(wma->psoc, vdev_id,
						     peer_mac, status);
	}

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
wma_resume_vdev_delete(tp_wma_handle wma, uint8_t vdev_id)
{
	struct wlan_objmgr_vdev *vdev;
	struct mac_context *mac = wma->mac_context;

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(wma->psoc, vdev_id,
						    WLAN_LEGACY_WMA_ID);
	if (!vdev) {
		wma_err("Vdev is NULL");
		return QDF_STATUS_E_NULL_VALUE;
	}

	lim_pasn_peer_del_all_resp_vdev_delete_resume(mac, vdev);

	wlan_objmgr_vdev_release_ref(vdev, WLAN_LEGACY_WMA_ID);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
wma_pasn_peer_delete_all_complete(struct wlan_objmgr_vdev *vdev)
{
	tp_wma_handle wma = cds_get_context(QDF_MODULE_ID_WMA);
	struct wma_target_req *req_msg;
	uint8_t vdev_id = wlan_vdev_get_id(vdev);
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	if (!wma) {
		wma_err("wma_handle is NULL");
		return QDF_STATUS_E_FAILURE;
	}

	req_msg = wma_find_remove_req_msgtype(wma, vdev_id, NULL,
					      WMA_PASN_PEER_DELETE_REQUEST);
	if (!req_msg) {
		wma_debug("vdev:%d Failed to lookup pasn peer del req",
			  vdev_id);
		return QDF_STATUS_E_FAILURE;
	}

	qdf_mc_timer_stop(&req_msg->event_timeout);
	qdf_mc_timer_destroy(&req_msg->event_timeout);

	qdf_mem_free(req_msg);

	wma_release_wakelock(&wma->wmi_cmd_rsp_wake_lock);

	if (wlan_get_opmode_from_vdev_id(wma->pdev, vdev_id) !=
	   QDF_NAN_DISC_MODE)
		status = wma_resume_vdev_delete(wma, vdev_id);

	return status;
}

QDF_STATUS
wma_delete_all_pasn_peers(struct wlan_objmgr_vdev *vdev)
{
	tp_wma_handle wma = cds_get_context(QDF_MODULE_ID_WMA);
	struct wlan_lmac_if_wifi_pos_rx_ops *rx_ops;
	struct wma_target_req *msg;
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	uint8_t vdev_id = wlan_vdev_get_id(vdev);
	enum QDF_OPMODE opmode;

	if (!wma) {
		wma_err("wma handle is NULL");
		return QDF_STATUS_E_NULL_VALUE;
	}

	wma_acquire_wakelock(&wma->wmi_cmd_rsp_wake_lock,
			     WMA_PEER_DELETE_RESPONSE_TIMEOUT);
	wma_debug("Delete all PASN peer of vdev:%d", wlan_vdev_get_id(vdev));

	opmode = wlan_get_opmode_from_vdev_id(wma->pdev, vdev_id);
	if (opmode == QDF_NAN_DISC_MODE) {
		wlan_nan_vdev_delete_all_pasn_peers(vdev);
	} else {
		rx_ops = wifi_pos_get_rx_ops(wma->psoc);
		if (!rx_ops ||
		    !rx_ops->wifi_pos_vdev_delete_all_ranging_peers_cb) {
			wma_err("rx_ops is NULL");
			return QDF_STATUS_E_NULL_VALUE;
		}
		status =
			rx_ops->wifi_pos_vdev_delete_all_ranging_peers_cb(vdev);
		if (QDF_IS_STATUS_ERROR(status)) {
			wma_release_wakelock(&wma->wmi_cmd_rsp_wake_lock);
			wma_err("Delete all ranging peers failed");
			return status;
		}
	}

	msg = wma_fill_hold_req(wma, vdev_id,
				WMA_PASN_PEER_DELETE_REQUEST,
				WMA_PASN_PEER_DELETE_RESPONSE, NULL, NULL,
				WMA_PEER_DELETE_RESPONSE_TIMEOUT);
	if (!msg) {
		wma_err("Failed to allocate request for vdev_id %d", vdev_id);
		wma_remove_req(wma, vdev_id, WMA_PASN_PEER_DELETE_RESPONSE);
		wma_release_wakelock(&wma->wmi_cmd_rsp_wake_lock);
		wma_resume_vdev_delete(wma, vdev_id);
		return QDF_STATUS_E_FAILURE;
	}

	return status;
}

void wma_pasn_peer_delete_handler(struct wlan_objmgr_psoc *psoc,
				  uint8_t *peer_mac, uint8_t vdev_id,
				  void *params)
{
	struct pasn_peer_del_rsp_params *peer_del_resp = params;

	if (!peer_del_resp) {
		wma_err("peer del rsp is null");
		return;
	}

	if (peer_del_resp->status) {
		wma_err("peer delete fail status %d", peer_del_resp->status);
		goto free;
	}

	if (!psoc) {
		wma_err("psoc is null");
		goto free;
	}

	if (peer_del_resp->vdev_id != vdev_id) {
		wma_err("peer del vdev id %d, expected vdev id %d", vdev_id,
			peer_del_resp->vdev_id);
		goto free;
	}

	wlan_nan_pasn_peer_handle_del_rsp(psoc, peer_mac, vdev_id);
free:
	qdf_mem_free(peer_del_resp);
}
