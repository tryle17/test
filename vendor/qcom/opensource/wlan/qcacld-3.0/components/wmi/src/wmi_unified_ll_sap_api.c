/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
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

#include <wmi.h>
#include <wmi_unified_priv.h>
#include "wmi_unified_ll_sap_api.h"
#include "wmi_unified_param.h"

QDF_STATUS wmi_unified_oob_connect_request_send(
					wmi_unified_t wmi_hdl,
					struct wmi_oob_connect_request request)
{
	if (wmi_hdl->ops->send_oob_connect_request)
		return wmi_hdl->ops->send_oob_connect_request(wmi_hdl,
							      request);
	return QDF_STATUS_E_FAILURE;
}

QDF_STATUS wmi_extract_oob_connect_response_event(
				wmi_unified_t wmi_handle,
				uint8_t *event, uint32_t len,
				struct wmi_oob_connect_response_event *response)
{
	if (wmi_handle->ops->extract_oob_connect_response_event)
		return wmi_handle->ops->extract_oob_connect_response_event(
					wmi_handle, event, len, response);
	return QDF_STATUS_E_FAILURE;
}

QDF_STATUS wmi_unified_audio_transport_switch_resp_send(
					wmi_unified_t wmi_hdl,
					enum bearer_switch_req_type req_type,
					enum bearer_switch_status status)
{
	if (wmi_hdl->ops->send_audio_transport_switch_resp)
		return wmi_hdl->ops->send_audio_transport_switch_resp(wmi_hdl,
								      req_type,
								      status);

	return QDF_STATUS_E_FAILURE;
}

QDF_STATUS
wmi_extract_audio_transport_switch_req_event(
				wmi_unified_t wmi_handle,
				uint8_t *event, uint32_t len,
				enum bearer_switch_req_type *req_type)
{
	if (wmi_handle->ops->extract_audio_transport_switch_req_event)
		return wmi_handle->ops->extract_audio_transport_switch_req_event(
					wmi_handle, event, len, req_type);
	return QDF_STATUS_E_FAILURE;
}

QDF_STATUS
wmi_unified_get_tsf_stats_for_csa(wmi_unified_t wmi_hdl, uint8_t vdev_id)
{
	if (wmi_hdl->ops->get_tsf_stats_for_csa)
		return wmi_hdl->ops->get_tsf_stats_for_csa(wmi_hdl, vdev_id);

	return QDF_STATUS_E_FAILURE;
}
