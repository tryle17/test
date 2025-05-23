/*
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
 * DOC: target_if_mlo_mgr.c
 *
 * This file provide definition for APIs registered through lmac Tx Ops
 */

#include <wmi_unified_11be_api.h>
#include <init_deinit_lmac.h>
#include "target_if_mlo_mgr.h"
#include <wlan_objmgr_peer_obj.h>
#include <wlan_mlo_t2lm.h>

/**
 * target_if_mlo_link_set_active_resp_handler() - function to handle mlo link
 *  set active response from firmware.
 * @scn: scn handle
 * @data: data buffer for event
 * @datalen: data length
 *
 * Return: 0 on success, else error on failure
 */
static int
target_if_mlo_link_set_active_resp_handler(ol_scn_t scn, uint8_t *data,
					   uint32_t datalen)
{
	QDF_STATUS status;
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlo_rx_ops *rx_ops;
	struct mlo_link_set_active_resp resp;

	if (!scn || !data) {
		target_if_err("scn: 0x%pK, data: 0x%pK", scn, data);
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		target_if_err("null psoc");
		return -EINVAL;
	}

	rx_ops = target_if_mlo_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->process_link_set_active_resp) {
		target_if_err("callback not registered");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("wmi_handle is null");
		return -EINVAL;
	}
	qdf_mem_zero(&resp, sizeof(resp));
	if (wmi_extract_mlo_link_set_active_resp(wmi_handle, data, &resp) !=
	    QDF_STATUS_SUCCESS) {
		target_if_err("Unable to extract mlo link set active resp");
		return -EINVAL;
	}
	if (rx_ops->trace_link_set_active_cb)
		rx_ops->trace_link_set_active_cb(psoc, NULL, &resp);

	status = rx_ops->process_link_set_active_resp(psoc, &resp);

	return qdf_status_to_os_return(status);
}

/**
 * target_if_mlo_link_removal_event_handler() - Handler for MLO link removal
 * event sent by the FW
 * @scn: scn handle
 * @data: data buffer for event
 * @datalen: data length
 *
 * Return: 0 on success, else error on failure
 */
static int
target_if_mlo_link_removal_event_handler(ol_scn_t scn, uint8_t *data,
					 uint32_t datalen)
{
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlo_rx_ops *mlo_rx_ops;
	QDF_STATUS status;
	struct mlo_link_removal_evt_params evt_params;

	if (!scn || !data) {
		target_if_err("scn: 0x%pK, data: 0x%pK", scn, data);
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		target_if_err("null psoc");
		return -EINVAL;
	}

	mlo_rx_ops = target_if_mlo_get_rx_ops(psoc);
	if (!mlo_rx_ops || !mlo_rx_ops->mlo_link_removal_handler) {
		target_if_err("callback not registered");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("wmi_handle is null");
		return -EINVAL;
	}

	status = wmi_extract_mlo_link_removal_evt_fixed_param(wmi_handle, data,
							      &evt_params);
	if (QDF_IS_STATUS_ERROR(status)) {
		target_if_err("Unable to extract fixed param, ret = %d",
			      status);
		goto exit;
	}

	status = wmi_extract_mlo_link_removal_tbtt_update(
			wmi_handle, data, &evt_params.tbtt_info);
	if (QDF_IS_STATUS_ERROR(status)) {
		target_if_err("Unable to extract TBTT update TLV, ret = %d",
			      status);
		goto exit;
	}

	status = mlo_rx_ops->mlo_link_removal_handler(psoc, &evt_params);
exit:
	return qdf_status_to_os_return(status);
}

#ifdef WLAN_FEATURE_11BE_MLO_3_LINK_TX
/**
 * target_if_mlo_tlt_selection_event_handler() - Handler for tlt mapping
 * event sent by the FW
 * @scn: scn handle
 * @data: data buffer for event
 * @datalen: data length
 *
 * Return: 0 on success, else error on failure
 */
static int
target_if_mlo_tlt_selection_event_handler(ol_scn_t scn, uint8_t *data,
					  uint32_t datalen)
{
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlo_rx_ops *mlo_rx_ops;
	QDF_STATUS status;
	struct mlo_tlt_selection_evt_params evt_params = {0};

	if (!scn || !data) {
		target_if_err("scn: 0x%pK, data: 0x%pK", scn, data);
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		target_if_err("null psoc");
		return -EINVAL;
	}

	mlo_rx_ops = target_if_mlo_get_rx_ops(psoc);
	if (!mlo_rx_ops || !mlo_rx_ops->mlo_3_link_tlt_selection_handler) {
		target_if_err("callback not registered");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("wmi_handle is null");
		return -EINVAL;
	}

	status = wmi_extract_mlo_3_link_tlt_selection_fixed_param(
								wmi_handle,
								data,
								&evt_params);
	if (QDF_IS_STATUS_ERROR(status)) {
		target_if_err("Unable to extract fixed param, ret = %d",
			      status);
		goto exit;
	}

	status = mlo_rx_ops->mlo_3_link_tlt_selection_handler(psoc,
							      &evt_params);
exit:
	return qdf_status_to_os_return(status);
}
#else
static int
target_if_mlo_tlt_selection_event_handler(ol_scn_t scn, uint8_t *data,
					  uint32_t datalen)
{
	return qdf_status_to_os_return(QDF_STATUS_SUCCESS);
}
#endif

QDF_STATUS
target_if_extract_mlo_link_removal_info_mgmt_rx(
		wmi_unified_t wmi_handle,
		void *evt_buf,
		struct mgmt_rx_event_params *rx_event)
{
	QDF_STATUS status;
	struct mgmt_rx_mlo_link_removal_info *link_removal_info;

	if (!rx_event) {
		target_if_err("Invalid rx_event");
		return QDF_STATUS_E_NULL_VALUE;
	}

	rx_event->link_removal_info = NULL;
	if (!rx_event->num_link_removal_info) {
		/**
		 * This is not an error. Only probe request frames will contain
		 * Link removal TLVs, that too only till the link removal TBTT
		 * countdown completion.
		 */
		target_if_debug("Link removal TLVs are not present");
		return QDF_STATUS_SUCCESS;
	}

	link_removal_info = qdf_mem_malloc(rx_event->num_link_removal_info *
					   sizeof(*link_removal_info));
	if (!link_removal_info) {
		target_if_err("Couldn't allocate memory for link_removal_info");
		rx_event->num_link_removal_info = 0;
		return QDF_STATUS_E_NOMEM;
	}

	status = wmi_extract_mgmt_rx_mlo_link_removal_info(
					wmi_handle, evt_buf,
					link_removal_info,
					rx_event->num_link_removal_info);
	if (QDF_IS_STATUS_ERROR(status)) {
		target_if_err("Unable to extract link removal TLVs");
		rx_event->num_link_removal_info = 0;
		qdf_mem_free(link_removal_info);
		return status;
	}

	rx_event->link_removal_info = link_removal_info;

	return QDF_STATUS_SUCCESS;
}

#ifdef WLAN_FEATURE_11BE_MLO_ADV_FEATURE
void
target_if_mlo_register_trace_link_set_active_cb(
		struct wlan_objmgr_psoc *psoc,
		trace_link_set_active_cb_type trace_link_set_active_cb)
{
	struct wlan_lmac_if_mlo_rx_ops *mlo_rx_ops;

	mlo_rx_ops = target_if_mlo_get_rx_ops(psoc);
	if (!mlo_rx_ops) {
		target_if_err("mlo_rx_ops null");
		return;
	}

	mlo_rx_ops->trace_link_set_active_cb =
		trace_link_set_active_cb;
}

static QDF_STATUS
target_if_send_mlo_link_switch_cnf_cmd(struct wlan_objmgr_psoc *psoc,
				       struct wlan_mlo_link_switch_cnf *params)
{
	struct wmi_unified *wmi_handle = NULL;

	if (!psoc) {
		target_if_err("null pdev");
		return QDF_STATUS_E_NULL_VALUE;
	}

	if (!params) {
		target_if_err("params is null");
		return QDF_STATUS_E_NULL_VALUE;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("null wmi handle");
		return QDF_STATUS_E_NULL_VALUE;
	}

	return wmi_send_mlo_link_switch_req_cnf_cmd(wmi_handle, params);
}

static int
target_if_mlo_link_state_switch_event_handler(ol_scn_t scn, uint8_t *data,
					      uint32_t datalen)
{
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlo_rx_ops *mlo_rx_ops;
	struct mlo_link_switch_state_info evt_params;
	QDF_STATUS status;

	if (!scn || !data) {
		target_if_err("scn: 0x%pK, data: 0x%pK", scn, data);
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		target_if_err("null psoc");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("wmi_handle is null");
		return -EINVAL;
	}

	mlo_rx_ops = target_if_mlo_get_rx_ops(psoc);
	if (!mlo_rx_ops || !mlo_rx_ops->mlo_link_state_switch_event_handler) {
		target_if_err("callback not registered");
		return -EINVAL;
	}

	status = wmi_extract_mlo_link_state_switch_evt(wmi_handle, data,
						       datalen,
						       &evt_params);
	if (QDF_IS_STATUS_ERROR(status)) {
		target_if_err("Unable to extract link state switch params");
		goto exit;
	}

	status = mlo_rx_ops->mlo_link_state_switch_event_handler(psoc,
								 &evt_params);
exit:
	return qdf_status_to_os_return(status);
}

static int
target_if_mlo_link_switch_request_event_handler(ol_scn_t scn, uint8_t *data,
						uint32_t datalen)
{
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlo_rx_ops *mlo_rx_ops;
	QDF_STATUS status;
	struct wlan_mlo_link_switch_req req = {0};

	if (!scn || !data) {
		target_if_err("scn: 0x%pK, data: 0x%pK", scn, data);
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		target_if_err("null psoc");
		return -EINVAL;
	}

	mlo_rx_ops = target_if_mlo_get_rx_ops(psoc);
	if (!mlo_rx_ops || !mlo_rx_ops->mlo_link_switch_request_handler) {
		target_if_err("callback not registered");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("wmi_handle is null");
		return -EINVAL;
	}

	status = wmi_extract_mlo_link_switch_request_evt(wmi_handle, data,
							 &req);

	if (QDF_IS_STATUS_ERROR(status)) {
		target_if_err("Unable to extract fixed param, ret = %d",
			      status);
		goto exit;
	}

	status = mlo_rx_ops->mlo_link_switch_request_handler(psoc, &req);

exit:
	return status;
}

static inline void
target_if_mlo_register_link_switch_cnf_handler(struct wlan_lmac_if_mlo_tx_ops *mlo_tx_ops)
{
	mlo_tx_ops->send_mlo_link_switch_cnf_cmd =
			target_if_send_mlo_link_switch_cnf_cmd;
}

static QDF_STATUS
target_if_mlo_register_link_switch_event_handler(struct wmi_unified *wmi_handle)
{
	QDF_STATUS status;

	status = wmi_unified_register_event_handler(
			wmi_handle,
			wmi_mlo_link_switch_request_eventid,
			target_if_mlo_link_switch_request_event_handler,
			WMI_RX_SERIALIZER_CTX);
	if (QDF_IS_STATUS_ERROR(status))
		target_if_err("Register event:%d failed",
			      wmi_mlo_link_switch_request_eventid);

	status = wmi_unified_register_event_handler(
			wmi_handle, wmi_mlo_link_state_switch_eventid,
			target_if_mlo_link_state_switch_event_handler,
			WMI_RX_SERIALIZER_CTX);
	if (QDF_IS_STATUS_ERROR(status))
		target_if_err("Register event:%d failed",
			      wmi_mlo_link_state_switch_eventid);

	return status;
}

static inline void
target_if_mlo_unregister_link_switch_event_handler(struct wmi_unified *wmi_handle)
{
	wmi_unified_unregister_event(wmi_handle,
				     wmi_mlo_link_switch_request_eventid);
}
#else
static inline QDF_STATUS
target_if_mlo_register_link_switch_event_handler(struct wmi_unified *wmi_handle)
{
	return QDF_STATUS_E_NOSUPPORT;
}

static inline void
target_if_mlo_unregister_link_switch_event_handler(struct wmi_unified *wmi_handle)
{
}

static inline QDF_STATUS
target_if_send_mlo_link_switch_cnf_cmd(struct wlan_objmgr_psoc *psoc,
				       struct wlan_mlo_link_switch_cnf *params)
{
	return QDF_STATUS_SUCCESS;
}

static inline void
target_if_mlo_register_link_switch_cnf_handler(struct wlan_lmac_if_mlo_tx_ops *mlo_tx_ops)
{
}

static inline int
target_if_mlo_link_switch_request_event_handler(ol_scn_t scn, uint8_t *data,
						uint32_t datalen)
{
	return 0;
}
#endif

/**
 * target_if_mlo_link_disable_request_event_handler() - Handler for MLO
 * link disable request event sent by the FW
 * @scn: scn handle
 * @data: data buffer for event
 * @datalen: data length
 *
 * Return: 0 on success, else error on failure
 */
static int
target_if_mlo_link_disable_request_event_handler(ol_scn_t scn, uint8_t *data,
						 uint32_t datalen)
{
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlo_rx_ops *mlo_rx_ops;
	QDF_STATUS status;
	struct mlo_link_disable_request_evt_params evt_params;

	if (!scn || !data) {
		target_if_err("scn: 0x%pK, data: 0x%pK", scn, data);
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		target_if_err("null psoc");
		return -EINVAL;
	}

	mlo_rx_ops = target_if_mlo_get_rx_ops(psoc);
	if (!mlo_rx_ops || !mlo_rx_ops->mlo_link_disable_request_handler) {
		target_if_err("callback not registered");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("wmi_handle is null");
		return -EINVAL;
	}

	status = wmi_extract_mlo_link_disable_request_evt(wmi_handle, data,
							  &evt_params);
	if (QDF_IS_STATUS_ERROR(status)) {
		target_if_err("Unable to extract fixed param, ret = %d",
			      status);
		goto exit;
	}

	status = mlo_rx_ops->mlo_link_disable_request_handler(psoc,
							      &evt_params);
exit:
	return qdf_status_to_os_return(status);
}

/**
 * target_if_mlo_register_event_handler() - function to register handler for
 *  mlo related wmi event from firmware.
 * @psoc: psoc pointer
 *
 * Return: QDF_STATUS
 */
static QDF_STATUS
target_if_mlo_register_event_handler(struct wlan_objmgr_psoc *psoc)
{
	QDF_STATUS status;
	struct wmi_unified *wmi_handle;

	if (!psoc) {
		target_if_err("PSOC is NULL!");
		return QDF_STATUS_E_NULL_VALUE;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("wmi_handle is null");
		return QDF_STATUS_E_INVAL;
	}

	status = wmi_unified_register_event(
			wmi_handle,
			wmi_mlo_link_removal_eventid,
			target_if_mlo_link_removal_event_handler);
	if (QDF_IS_STATUS_ERROR(status))
		target_if_err("Couldn't register handler for Link removal WMI event %d",
			      status);

	status = wmi_unified_register_event(
			wmi_handle,
			wmi_mlo_tlt_selection_for_tid_eventid,
			target_if_mlo_tlt_selection_event_handler);
	if (QDF_IS_STATUS_ERROR(status))
		target_if_err("Couldn't register handler for 3 Link mlo tlt WMI event %d",
			      status);

	status = wmi_unified_register_event_handler(
			wmi_handle,
			wmi_mlo_link_set_active_resp_eventid,
			target_if_mlo_link_set_active_resp_handler,
			WMI_RX_SERIALIZER_CTX);
	if (QDF_IS_STATUS_ERROR(status)) {
		target_if_err("Register mlo link set active resp cb errcode %d",
			      status);
		if (status == QDF_STATUS_E_NOSUPPORT)
			status = QDF_STATUS_SUCCESS;
	}

	target_if_mlo_register_vdev_tid_to_link_map_event(wmi_handle);
	target_if_mlo_register_mlo_link_state_info_event(wmi_handle);

	status = wmi_unified_register_event_handler(wmi_handle,
			wmi_mlo_link_disable_request_eventid,
			target_if_mlo_link_disable_request_event_handler,
			WMI_RX_SERIALIZER_CTX);
	if (QDF_IS_STATUS_ERROR(status)) {
		target_if_err("Couldn't register handler for link disable request WMI event %d",
			      status);
		if (status == QDF_STATUS_E_NOSUPPORT)
			status = QDF_STATUS_SUCCESS;
	}

	status = target_if_mlo_register_link_switch_event_handler(wmi_handle);
	if (QDF_IS_STATUS_ERROR(status)) {
		target_if_err("Couldn't register handler for link switch WMI event %d",
			      status);
		if (status == QDF_STATUS_E_NOSUPPORT)
			status = QDF_STATUS_SUCCESS;
	}

	return status;
}

/**
 * target_if_mlo_unregister_event_handler() - function to unregister handler for
 *  mlo related wmi event from firmware.
 * @psoc: psoc pointer
 *
 * Return: QDF_STATUS
 */
static QDF_STATUS
target_if_mlo_unregister_event_handler(struct wlan_objmgr_psoc *psoc)
{
	struct wmi_unified *wmi_handle;

	if (!psoc) {
		target_if_err("PSOC is NULL!");
		return QDF_STATUS_E_INVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("wmi_handle is null");
		return QDF_STATUS_E_INVAL;
	}

	wmi_unified_unregister_event_handler(wmi_handle,
		wmi_mlo_link_set_active_resp_eventid);

	wmi_unified_unregister_event(wmi_handle,
				     wmi_mlo_link_removal_eventid);

	wmi_unified_unregister_event(wmi_handle,
				     wmi_mlo_tlt_selection_for_tid_eventid);

	target_if_mlo_unregister_vdev_tid_to_link_map_event(wmi_handle);
	target_if_mlo_unregister_mlo_link_state_info_event(wmi_handle);

	wmi_unified_unregister_event(wmi_handle,
				     wmi_mlo_link_disable_request_eventid);

	target_if_mlo_unregister_link_switch_event_handler(wmi_handle);

	return QDF_STATUS_SUCCESS;
}

/**
 * target_if_mlo_link_set_active() - Send WMI command for set mlo link active
 * @psoc: psoc pointer
 * @param: parameter for setting mlo link active
 *
 * Return: QDF_STATUS
 */
static QDF_STATUS
target_if_mlo_link_set_active(struct wlan_objmgr_psoc *psoc,
			      struct mlo_link_set_active_param *param)
{
	QDF_STATUS ret;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlo_rx_ops *rx_ops;

	if (!psoc) {
		target_if_err("null psoc");
		return QDF_STATUS_E_FAILURE;
	}
	rx_ops = target_if_mlo_get_rx_ops(psoc);
	if (!rx_ops) {
		target_if_err("rx_ops NULL");
		return QDF_STATUS_E_INVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("null handle");
		return QDF_STATUS_E_FAILURE;
	}

	ret = wmi_send_mlo_link_set_active_cmd(wmi_handle, param);
	if (QDF_IS_STATUS_ERROR(ret))
		target_if_err("wmi mlo link set active send failed: %d", ret);
	else if (rx_ops->trace_link_set_active_cb)
		rx_ops->trace_link_set_active_cb(psoc, param, NULL);

	return ret;
}

static int target_if_mlo_vdev_tid_to_link_map_event_handler(
		ol_scn_t scn, uint8_t *event_buff, uint32_t len)
{
	struct wlan_objmgr_psoc *psoc;
	struct mlo_vdev_host_tid_to_link_map_resp event = {0};
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_mlo_rx_ops *rx_ops;
	QDF_STATUS status;

	if (!event_buff) {
		mlme_err("Received NULL event ptr from FW");
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		mlme_err("PSOC is NULL");
		return -EINVAL;
	}

	rx_ops = target_if_mlo_get_rx_ops(psoc);
	if (!rx_ops || !rx_ops->process_mlo_vdev_tid_to_link_map_event) {
		target_if_err("callback not registered");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		mlme_err("wmi_handle is null");
		return -EINVAL;
	}

	if (wmi_extract_mlo_vdev_tid_to_link_map_event(wmi_handle, event_buff,
						       &event)) {
		mlme_err("Failed to extract TID-to-link mapping event");
		return -EINVAL;
	}

	status = rx_ops->process_mlo_vdev_tid_to_link_map_event(psoc, &event);

	return qdf_status_to_os_return(status);
}

void target_if_mlo_register_vdev_tid_to_link_map_event(
		struct wmi_unified *wmi_handle)
{
	wmi_unified_register_event_handler(
			wmi_handle, wmi_mlo_ap_vdev_tid_to_link_map_eventid,
			target_if_mlo_vdev_tid_to_link_map_event_handler,
			WMI_RX_EXECUTION_CTX);
}

void target_if_mlo_unregister_vdev_tid_to_link_map_event(
		struct wmi_unified *wmi_handle)
{
	wmi_unified_unregister_event_handler(
			wmi_handle, wmi_mlo_ap_vdev_tid_to_link_map_eventid);
}

static int target_if_mlo_link_state_info_event_handler(
		ol_scn_t scn, uint8_t *event_buff, uint32_t len)
{
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	QDF_STATUS status;
	struct wlan_lmac_if_mlo_rx_ops *mlo_rx_ops;
	struct ml_link_state_info_event event = {0};

	if (!event_buff) {
		target_if_err("Received NULL event ptr from FW");
		return -EINVAL;
	}

	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		target_if_err("PSOC is NULL");
		return -EINVAL;
	}

	mlo_rx_ops = target_if_mlo_get_rx_ops(psoc);
	if (!mlo_rx_ops || !mlo_rx_ops->process_mlo_link_state_info_event) {
		target_if_err("callback not registered");
		return -EINVAL;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("wmi_handle is null");
		return -EINVAL;
	}

	if (wmi_extract_mlo_link_state_info_event(wmi_handle, event_buff,
						  &event)) {
		target_if_err("Failed to extract link status event");
		return -EINVAL;
	}

	status = mlo_rx_ops->process_mlo_link_state_info_event(psoc, &event);
	return qdf_status_to_os_return(status);
}

void target_if_mlo_register_mlo_link_state_info_event(
		struct wmi_unified *wmi_handle)
{
	wmi_unified_register_event_handler(
		wmi_handle, wmi_mlo_link_state_info_eventid,
		target_if_mlo_link_state_info_event_handler,
		WMI_RX_EXECUTION_CTX);
}

void  target_if_mlo_unregister_mlo_link_state_info_event(
		struct wmi_unified *wmi_handle)
{
	wmi_unified_unregister_event_handler(
			wmi_handle,
			wmi_mlo_link_state_info_eventid);
}

#ifdef WLAN_FEATURE_11BE_MLO_ADV_FEATURE
/**
 * target_if_fill_provisioned_links() - API to fill the provisioned links
 * @params: Pointer to T2LM params structure
 * @t2lm: Pointer to T2LM info structure
 *
 * Return: none
 */
static inline void target_if_fill_provisioned_links(
		struct wmi_host_tid_to_link_map_params *params,
		struct wlan_t2lm_info *t2lm)
{
	qdf_mem_copy(&params->t2lm_info[params->num_dir].t2lm_provisioned_links,
		     &t2lm->ieee_link_map_tid,
		     sizeof(uint16_t) * T2LM_MAX_NUM_TIDS);
}

/**
 * target_if_fill_timer() - API to fill the t2lm timer values
 * @params: Pointer to T2LM params structure
 * @t2lm: Pointer to T2LM info structure
 *
 * Return: none
 */
static inline void
target_if_fill_timer(struct wmi_host_tid_to_link_map_params *params,
		     struct wlan_t2lm_info *t2lm)
{
	if (t2lm->mapping_switch_time_present)
		params->mapping_switch_time = t2lm->mapping_switch_time;

	if (t2lm->expected_duration_present)
		params->expected_duration = t2lm->expected_duration;
}

#else
static inline void target_if_fill_provisioned_links(
		struct wmi_host_tid_to_link_map_params *params,
		struct wlan_t2lm_info *t2lm)
{
	qdf_mem_copy(&params->t2lm_info[params->num_dir].t2lm_provisioned_links,
		     &t2lm->hw_link_map_tid,
		     sizeof(uint16_t) * T2LM_MAX_NUM_TIDS);
}

static inline void
target_if_fill_timer(struct wmi_host_tid_to_link_map_params *params,
		     struct wlan_t2lm_info *t2lm)
{
}
#endif

static QDF_STATUS
target_if_mlo_send_tid_to_link_mapping(struct wlan_objmgr_vdev *vdev,
				       struct wlan_t2lm_info *t2lm)
{
	struct wmi_unified *wmi_handle = NULL;
	struct wmi_host_tid_to_link_map_params params = {0};
	struct wlan_objmgr_pdev *pdev = NULL;
	int tid = 0;
	QDF_STATUS status;

	pdev = wlan_vdev_get_pdev(vdev);
	if (!pdev) {
		t2lm_err("null pdev");
		return QDF_STATUS_E_NULL_VALUE;
	}

	wmi_handle = lmac_get_pdev_wmi_handle(pdev);
	if (!wmi_handle) {
		t2lm_err("null wmi handle");
		return QDF_STATUS_E_NULL_VALUE;
	}

	params.pdev_id = wlan_objmgr_pdev_get_pdev_id(pdev);
	qdf_mem_copy(params.peer_macaddr, vdev->vdev_objmgr.bss_peer->macaddr,
		     QDF_MAC_ADDR_SIZE);

	t2lm_debug("Fill T2LM WMI info for peer: " QDF_MAC_ADDR_FMT " pdev_id:%d",
		   QDF_MAC_ADDR_REF(params.peer_macaddr), params.pdev_id);

	params.t2lm_info[params.num_dir].direction = t2lm->direction;
	params.t2lm_info[params.num_dir].default_link_mapping =
		t2lm->default_link_mapping;

	if (!params.t2lm_info[params.num_dir].default_link_mapping)
		target_if_fill_provisioned_links(&params, t2lm);

	target_if_fill_timer(&params, t2lm);
	t2lm_debug("mapping_switch_time_present %d MST %d",
		   t2lm->mapping_switch_time_present,
		   params.mapping_switch_time);
	t2lm_debug("expected_switch_time_present %d EDT %d",
		   t2lm->expected_duration_present,
		   params.expected_duration);

	t2lm_debug("num_dir:%d direction:%d default_link_mapping:%d",
		   params.num_dir, params.t2lm_info[params.num_dir].direction,
		   params.t2lm_info[params.num_dir].default_link_mapping);

	for (tid = 0; tid < T2LM_MAX_NUM_TIDS; tid++) {
		t2lm_debug("tid:%d hw_link_map:%x ieee_link_map:%x", tid,
			   params.t2lm_info[params.num_dir].t2lm_provisioned_links[tid],
			   t2lm->ieee_link_map_tid[tid]);
	}

	params.num_dir++;

	status = wmi_send_mlo_peer_tid_to_link_map_cmd(wmi_handle, &params, true);
	if (QDF_IS_STATUS_ERROR(status)) {
		t2lm_err("Failed to send T2LM WMI command for pdev_id:%d peer_mac: " QDF_MAC_ADDR_FMT,
			 params.pdev_id,
			 QDF_MAC_ADDR_REF(params.peer_macaddr));
		return status;
	}

	return status;
}

static QDF_STATUS
target_if_request_ml_link_state_info(struct wlan_objmgr_psoc *psoc,
				     struct mlo_link_state_cmd_params *cmd)
{
	struct wmi_unified *wmi_handle = NULL;
	struct wmi_host_link_state_params params = {0};
	QDF_STATUS status;

	if (!psoc) {
		target_if_err("null pdev");
		return QDF_STATUS_E_NULL_VALUE;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("null wmi handle");
		return QDF_STATUS_E_NULL_VALUE;
	}

	if (!cmd) {
		target_if_err("cmd is null");
		return QDF_STATUS_E_NULL_VALUE;
	}

	params.vdev_id = cmd->vdev_id;
	qdf_mem_copy(params.mld_mac, cmd->mld_mac,
		     QDF_MAC_ADDR_SIZE);

	status = wmi_send_mlo_link_state_request_cmd(wmi_handle, &params);
	return status;
}

#ifdef WLAN_WSI_STATS_SUPPORT
static QDF_STATUS
target_if_mlo_send_wsi_link_info_cmd(struct wlan_objmgr_pdev *pdev,
				     struct mlo_wsi_link_stats *param)
{
	struct wmi_unified *wmi_handle;
	struct wmi_wsi_stats_info_params params = {0};

	if (!pdev) {
		target_if_err("null pdev");
		return QDF_STATUS_E_NULL_VALUE;
	}
	wmi_handle = get_wmi_unified_hdl_from_pdev(pdev);
	if (!wmi_handle) {
		target_if_err("null handle");
		return QDF_STATUS_E_FAILURE;
	}

	params.pdev_id = pdev->pdev_objmgr.wlan_pdev_id;
	params.wsi_ingress_load_info = param->ingress_cnt;
	params.wsi_egress_load_info = param->egress_cnt;

	target_if_debug("pdev id %d, ingress %d, egress %d", params.pdev_id,
			params.wsi_ingress_load_info,
			params.wsi_egress_load_info);

	return wmi_unified_config_wsi_stats_info_cmd_send(wmi_handle, &params);
}
#else
static QDF_STATUS
target_if_mlo_send_wsi_link_info_cmd(struct wlan_objmgr_pdev *pdev,
				     struct mlo_wsi_link_stats *param)
{
	return QDF_STATUS_SUCCESS;
}
#endif
static QDF_STATUS
target_if_send_link_set_bss_params_cmd(struct wlan_objmgr_psoc *psoc,
				       struct mlo_link_bss_params *cmd)
{
	QDF_STATUS status;
	struct wmi_unified *wmi_handle = NULL;
	struct wmi_host_link_bss_params params = {0};

	if (!psoc) {
		target_if_err("null pdev");
		return QDF_STATUS_E_NULL_VALUE;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("null wmi handle");
		return QDF_STATUS_E_NULL_VALUE;
	}
	if (!cmd) {
		target_if_err("cmd is null");
		return QDF_STATUS_E_NULL_VALUE;
	}
	params.link_id = cmd->link_id;
	qdf_mem_copy(&params.ap_mld_mac[0], &cmd->ap_mld_mac[0],
		     QDF_MAC_ADDR_SIZE);

	params.chan.ch_freq = cmd->chan->ch_freq;
	params.chan.ch_cfreq1 = cmd->chan->ch_cfreq1;
	params.chan.ch_cfreq2 = cmd->chan->ch_cfreq2;
	params.chan.ch_phymode  = cmd->chan->ch_phymode;

	status = wmi_send_link_set_bss_params_cmd(wmi_handle, &params);

	return status;
}

QDF_STATUS target_if_mlo_send_link_removal_cmd(
		struct wlan_objmgr_psoc *psoc,
		const struct mlo_link_removal_cmd_params *param)
{
	struct wmi_unified *wmi_handle;

	if (!psoc) {
		target_if_err("null psoc");
		return QDF_STATUS_E_NULL_VALUE;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("null handle");
		return QDF_STATUS_E_FAILURE;
	}

	return wmi_send_mlo_link_removal_cmd(wmi_handle, param);
}

QDF_STATUS target_if_mlo_send_vdev_pause(struct wlan_objmgr_psoc *psoc,
					 struct mlo_vdev_pause *info)
{
	struct wmi_unified *wmi_handle;

	if (!psoc) {
		target_if_err("null psoc");
		return QDF_STATUS_E_NULL_VALUE;
	}

	wmi_handle = get_wmi_unified_hdl_from_psoc(psoc);
	if (!wmi_handle) {
		target_if_err("null handle");
		return QDF_STATUS_E_FAILURE;
	}

	return wmi_send_mlo_vdev_pause(wmi_handle, info);
}

#ifdef QCA_SUPPORT_PRIMARY_LINK_MIGRATE
static QDF_STATUS target_if_mlo_send_peer_ptqm_migrate_cmd(
					struct wlan_objmgr_vdev *vdev,
					struct peer_ptqm_migrate_params *param)
{
	struct wlan_objmgr_pdev *pdev = NULL;
	struct wmi_unified *wmi_handle;
	QDF_STATUS status;

	if (!vdev || !param) {
		target_if_err("Invalid input");
		return QDF_STATUS_E_INVAL;
	}

	pdev = wlan_vdev_get_pdev(vdev);
	if (!pdev) {
		target_if_err("null pdev");
		return QDF_STATUS_E_NULL_VALUE;
	}

	wmi_handle = lmac_get_pdev_wmi_handle(pdev);
	if (!wmi_handle) {
		target_if_err("Failed to get WMI handle!");
		return QDF_STATUS_E_INVAL;
	}

	status = wmi_unified_peer_ptqm_migrate_send(wmi_handle, param);
	if (QDF_IS_STATUS_ERROR(status))
		target_if_err("Failed to send peer ptqm migration WMI");

	return status;
}

static void target_if_mlo_register_peer_ptqm_migrate_send(
		struct wlan_lmac_if_mlo_tx_ops *mlo_tx_ops)
{
	mlo_tx_ops->peer_ptqm_migrate_send =
				target_if_mlo_send_peer_ptqm_migrate_cmd;
}
#else
static void target_if_mlo_register_peer_ptqm_migrate_send(
		struct wlan_lmac_if_mlo_tx_ops *mlo_tx_ops)
{
}
#endif

/**
 * target_if_mlo_register_tx_ops() - lmac handler to register mlo tx ops
 *  callback functions
 * @tx_ops: wlan_lmac_if_tx_ops object
 *
 * Return: QDF_STATUS
 */
QDF_STATUS
target_if_mlo_register_tx_ops(struct wlan_lmac_if_tx_ops *tx_ops)
{
	struct wlan_lmac_if_mlo_tx_ops *mlo_tx_ops;

	if (!tx_ops) {
		target_if_err("lmac tx ops is NULL!");
		return QDF_STATUS_E_INVAL;
	}

	mlo_tx_ops = &tx_ops->mlo_ops;
	if (!mlo_tx_ops) {
		target_if_err("lmac tx ops is NULL!");
		return QDF_STATUS_E_FAILURE;
	}

	mlo_tx_ops->register_events =
		target_if_mlo_register_event_handler;
	mlo_tx_ops->unregister_events =
		target_if_mlo_unregister_event_handler;
	mlo_tx_ops->link_set_active = target_if_mlo_link_set_active;
	mlo_tx_ops->send_tid_to_link_mapping =
		target_if_mlo_send_tid_to_link_mapping;
	mlo_tx_ops->send_link_removal_cmd = target_if_mlo_send_link_removal_cmd;
	mlo_tx_ops->request_link_state_info_cmd =
		target_if_request_ml_link_state_info;
	mlo_tx_ops->send_link_set_bss_params_cmd =
		target_if_send_link_set_bss_params_cmd;
	mlo_tx_ops->send_vdev_pause = target_if_mlo_send_vdev_pause;

	target_if_mlo_register_link_switch_cnf_handler(mlo_tx_ops);

	mlo_tx_ops->send_wsi_link_info_cmd =
		target_if_mlo_send_wsi_link_info_cmd;

	target_if_mlo_register_peer_ptqm_migrate_send(mlo_tx_ops);
	return QDF_STATUS_SUCCESS;
}

