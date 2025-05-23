// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/slab.h>
#include "cam_io_util.h"
#include "cam_cdm_util.h"
#include "cam_vfe_hw_intf.h"
#include "cam_vfe_top.h"
#include "cam_vfe_top_ver3.h"
#include "cam_debug_util.h"
#include "cam_vfe_soc.h"
#include "cam_mem_mgr_api.h"

#define CAM_VFE_HW_RESET_HW_AND_REG_VAL       0x00000001
#define CAM_VFE_HW_RESET_HW_VAL               0x00010000
#define CAM_VFE_LITE_HW_RESET_AND_REG_VAL     0x00000002
#define CAM_VFE_LITE_HW_RESET_HW_VAL          0x00000001
#define CAM_CDM_WAIT_COMP_EVENT_BIT           0x2

struct cam_vfe_top_ver3_common_data {
	struct cam_vfe_top_ver3_hw_info            *hw_info;
	struct cam_hw_intf                         *hw_intf;
	struct cam_vfe_top_ver3_reg_offset_common  *common_reg;
};

struct cam_vfe_top_ver3_priv {
	struct cam_vfe_top_ver3_common_data common_data;
	struct cam_vfe_top_priv_common      top_common;
};

static int cam_vfe_top_ver3_get_path_port_map(struct cam_vfe_top_ver3_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_isp_hw_path_port_map *arg = cmd_args;
	struct cam_vfe_top_ver3_hw_info *hw_info = top_priv->common_data.hw_info;
	int i;

	for (i = 0; i < hw_info->num_path_port_map; i++) {
		arg->entry[i][0] = hw_info->path_port_map[i][0];
		arg->entry[i][1] = hw_info->path_port_map[i][1];
	}
	arg->num_entries = hw_info->num_path_port_map;

	return 0;
}

static int cam_vfe_top_ver3_mux_get_base(struct cam_vfe_top_ver3_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	uint32_t                          size = 0;
	uint32_t                          mem_base = 0;
	struct cam_isp_hw_get_cmd_update *cdm_args  = cmd_args;
	struct cam_cdm_utils_ops         *cdm_util_ops = NULL;

	if (arg_size != sizeof(struct cam_isp_hw_get_cmd_update)) {
		CAM_ERR(CAM_ISP, "Error, Invalid cmd size");
		return -EINVAL;
	}

	if (!cdm_args || !cdm_args->res || !top_priv ||
		!top_priv->top_common.soc_info) {
		CAM_ERR(CAM_ISP, "Error, Invalid args");
		return -EINVAL;
	}

	cdm_util_ops =
		(struct cam_cdm_utils_ops *)cdm_args->res->cdm_ops;

	if (!cdm_util_ops) {
		CAM_ERR(CAM_ISP, "Invalid CDM ops");
		return -EINVAL;
	}

	size = cdm_util_ops->cdm_required_size_changebase();
	/* since cdm returns dwords, we need to convert it into bytes */
	if ((size * 4) > cdm_args->cmd.size) {
		CAM_ERR(CAM_ISP, "buf size:%d is not sufficient, expected: %d",
			cdm_args->cmd.size, size);
		return -EINVAL;
	}

	mem_base = CAM_SOC_GET_REG_MAP_CAM_BASE(
		top_priv->top_common.soc_info, VFE_CORE_BASE_IDX);
	if (mem_base == -1) {
		CAM_ERR(CAM_ISP, "failed to get mem_base, index: %d num_reg_map: %u",
			VFE_CORE_BASE_IDX, top_priv->top_common.soc_info->num_reg_map);
		return -EINVAL;
	}
	CAM_DBG(CAM_ISP, "core %d mem_base 0x%x",
		top_priv->top_common.soc_info->index, mem_base);

	cdm_util_ops->cdm_write_changebase(
	cdm_args->cmd.cmd_buf_addr, mem_base);
	cdm_args->cmd.used_bytes = (size * 4);

	return 0;
}

static int cam_vfe_top_fs_update(
	struct cam_vfe_top_ver3_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_vfe_fe_update_args *cmd_update = cmd_args;

	if (cmd_update->node_res->process_cmd)
		return cmd_update->node_res->process_cmd(cmd_update->node_res,
			CAM_ISP_HW_CMD_FE_UPDATE_IN_RD, cmd_args, arg_size);

	return 0;
}

static int cam_vfe_top_ver3_dump_info(
	struct cam_vfe_top_ver3_priv *top_priv, uint32_t cmd_type)
{
	struct cam_hw_soc_info *soc_info = top_priv->top_common.soc_info;

	if (!soc_info) {
		CAM_ERR(CAM_ISP, "Null soc_info");
		return -EINVAL;
	}

	switch (cmd_type) {
	case CAM_ISP_HW_NOTIFY_OVERFLOW:
		CAM_INFO_RATE_LIMIT(CAM_ISP, "VFE%d src_clk_rate:%luHz",
			soc_info->index, soc_info->applied_src_clk_rates.sw_client);
		break;
	default:
		CAM_ERR(CAM_ISP, "cmd_type: %u not supported", cmd_type);
		break;
	}

	return 0;
}


static int cam_vfe_top_ver3_blanking_update(uint32_t cmd_type,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_isp_blanking_config       *blanking_config = NULL;
	struct cam_isp_resource_node         *node_res = NULL;

	blanking_config =
		(struct cam_isp_blanking_config *)cmd_args;
	node_res = blanking_config->node_res;

	if (!node_res) {
		CAM_ERR(CAM_PERF, "Invalid input res %pK", node_res);
		return -EINVAL;
	}

	if (!node_res->process_cmd) {
		CAM_ERR(CAM_PERF, "Invalid input res process_cmd %pK",
			node_res->process_cmd);
		return -EINVAL;
	}

	return node_res->process_cmd(node_res,
		cmd_type, cmd_args, arg_size);
}

static int cam_vfe_core_config_control(
	struct cam_vfe_top_ver3_priv *top_priv,
	 void *cmd_args, uint32_t arg_size)
{
	struct cam_vfe_core_config_args  *core_config = cmd_args;

	if (core_config->node_res->process_cmd)
		return core_config->node_res->process_cmd(core_config->node_res,
			CAM_ISP_HW_CMD_CORE_CONFIG, cmd_args, arg_size);

	return -EINVAL;
}

static int cam_vfe_top_ver3_mux_get_reg_update(
	struct cam_vfe_top_ver3_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_isp_hw_get_cmd_update  *cmd_update = cmd_args;

	if (cmd_update->res->process_cmd)
		return cmd_update->res->process_cmd(cmd_update->res,
			CAM_ISP_HW_CMD_GET_REG_UPDATE, cmd_args, arg_size);

	return -EINVAL;
}

static int cam_vfe_top_wait_comp_event(struct cam_vfe_top_ver3_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	uint32_t                          size = 0;
	struct cam_isp_hw_get_cmd_update *cdm_args  = cmd_args;
	struct cam_cdm_utils_ops         *cdm_util_ops = NULL;

	if (arg_size != sizeof(struct cam_isp_hw_get_cmd_update)) {
		CAM_ERR(CAM_ISP, "Error, Invalid arg size = %d expected = %d",
			arg_size, sizeof(struct cam_isp_hw_get_cmd_update));
		return -EINVAL;
	}

	if (!cdm_args || !cdm_args->res || !top_priv ||
		!top_priv->top_common.soc_info) {
		CAM_ERR(CAM_ISP, "Error, Invalid args");
		return -EINVAL;
	}

	cdm_util_ops =
		(struct cam_cdm_utils_ops *)cdm_args->res->cdm_ops;

	if (!cdm_util_ops) {
		CAM_ERR(CAM_ISP, "Invalid CDM ops");
		return -EINVAL;
	}

	size = cdm_util_ops->cdm_required_size_comp_wait();
	/* since cdm returns dwords, we need to convert it into bytes */
	if ((size * 4) > cdm_args->cmd.size) {
		CAM_ERR(CAM_ISP, "buf size:%d is not sufficient, expected: %d",
			cdm_args->cmd.size, size);
		return -EINVAL;
	}

	cdm_util_ops->cdm_write_wait_comp_event(cdm_args->cmd.cmd_buf_addr,
		0, CAM_CDM_WAIT_COMP_EVENT_BIT);
	cdm_args->cmd.used_bytes = (size * 4);

	return 0;
}

static int cam_vfe_top_add_wait_trigger(struct cam_vfe_top_ver3_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	uint32_t                          size = 0;
	uint32_t                          reg_val_pair[2];
	struct cam_isp_hw_get_cmd_update *cdm_args  = cmd_args;
	struct cam_cdm_utils_ops         *cdm_util_ops = NULL;
	struct cam_vfe_top_ver3_reg_offset_common *reg_common = NULL;
	uint32_t set_cdm_trigger_event;

	if (arg_size != sizeof(struct cam_isp_hw_get_cmd_update)) {
		CAM_ERR(CAM_ISP, "Error, Invalid arg size = %d expected = %d",
			arg_size, sizeof(struct cam_isp_hw_get_cmd_update));
		return -EINVAL;
	}

	if (!cdm_args || !cdm_args->res || !top_priv ||
		!top_priv->top_common.soc_info) {
		CAM_ERR(CAM_ISP, "Error, Invalid args");
		return -EINVAL;
	}

	cdm_util_ops =
		(struct cam_cdm_utils_ops *)cdm_args->res->cdm_ops;

	if (!cdm_util_ops) {
		CAM_ERR(CAM_ISP, "Invalid CDM ops");
		return -EINVAL;
	}

	size = cdm_util_ops->cdm_required_size_reg_random(1);
	/* since cdm returns dwords, we need to convert it into bytes */
	if ((size * 4) > cdm_args->cmd.size) {
		CAM_ERR(CAM_ISP, "buf size:%d is not sufficient, expected: %d",
			cdm_args->cmd.size, size);
		return -EINVAL;
	}

	if (cdm_args->trigger_cdm_en == true)
		set_cdm_trigger_event = 1;
	else
		set_cdm_trigger_event = 0;

	reg_common = top_priv->common_data.common_reg;
	reg_val_pair[0] = reg_common->trigger_cdm_events;
	reg_val_pair[1] = set_cdm_trigger_event;

	cdm_util_ops->cdm_write_regrandom(cdm_args->cmd.cmd_buf_addr,
		1, reg_val_pair);
	cdm_args->cmd.used_bytes = (size * 4);

	return 0;
}

static int cam_vfe_top_ver3_get_data(
	struct cam_vfe_top_ver3_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_isp_resource_node  *res = cmd_args;

	if (res->process_cmd)
		return res->process_cmd(res,
			CAM_ISP_HW_CMD_CAMIF_DATA, cmd_args, arg_size);

	return -EINVAL;
}

int cam_vfe_top_ver3_get_hw_caps(void *device_priv,
	void *args, uint32_t arg_size)
{
	struct cam_vfe_hw_get_hw_cap *vfe_cap_info = NULL;
	struct cam_vfe_top_ver3_priv *vfe_top_prv = NULL;
	struct cam_vfe_soc_private *vfe_soc_private = NULL;

	if (!device_priv || !args) {
		CAM_ERR(CAM_ISP,
			"Invalid arguments device_priv:%p, args:%p",
			device_priv, args);
		return -EINVAL;
	}

	vfe_cap_info = (struct cam_vfe_hw_get_hw_cap *)args;
	vfe_top_prv = (struct cam_vfe_top_ver3_priv *)device_priv;

	if (!vfe_top_prv->top_common.soc_info) {
		CAM_ERR(CAM_ISP, "soc_info is null");
		return -EFAULT;
	}

	vfe_soc_private = (struct cam_vfe_soc_private *)
		vfe_top_prv->top_common.soc_info->soc_private;

	vfe_cap_info->is_lite = (vfe_soc_private->is_ife_lite) ? true : false;
	vfe_cap_info->incr =
		(vfe_top_prv->top_common.hw_version) & 0x00ffff;
	vfe_cap_info->minor =
		((vfe_top_prv->top_common.hw_version) >> 16) & 0x0fff;
	vfe_cap_info->major =
		((vfe_top_prv->top_common.hw_version) >> 28) & 0x000f;

	return 0;
}

int cam_vfe_top_ver3_init_hw(void *device_priv,
	void *init_hw_args, uint32_t arg_size)
{
	struct cam_vfe_top_ver3_priv   *top_priv = device_priv;
	struct cam_vfe_top_ver3_common_data common_data = top_priv->common_data;

	top_priv->top_common.applied_clk_rate = 0;

	/**
	 * Auto clock gating is enabled by default, but no harm
	 * in setting the value we expect.
	 */
	CAM_DBG(CAM_ISP, "Enabling clock gating at IFE top");

	cam_soc_util_w_mb(top_priv->top_common.soc_info, VFE_CORE_BASE_IDX,
		common_data.common_reg->core_cgc_ovd_0, 0x0);

	cam_soc_util_w_mb(top_priv->top_common.soc_info, VFE_CORE_BASE_IDX,
		common_data.common_reg->core_cgc_ovd_1, 0x0);

	cam_soc_util_w_mb(top_priv->top_common.soc_info, VFE_CORE_BASE_IDX,
		common_data.common_reg->ahb_cgc_ovd, 0x0);

	cam_soc_util_w_mb(top_priv->top_common.soc_info, VFE_CORE_BASE_IDX,
		common_data.common_reg->noc_cgc_ovd, 0x0);

	top_priv->top_common.hw_version =
		cam_io_r_mb(top_priv->top_common.soc_info->reg_map[0].mem_base +
		common_data.common_reg->hw_version);

	return 0;
}

int cam_vfe_top_ver3_reset(void *device_priv,
	void *reset_core_args, uint32_t arg_size)
{
	struct cam_vfe_top_ver3_priv   *top_priv = device_priv;
	struct cam_hw_soc_info         *soc_info = NULL;
	struct cam_vfe_soc_private     *soc_private = NULL;
	struct cam_vfe_top_ver3_reg_offset_common *reg_common = NULL;
	uint32_t *reset_reg_args = reset_core_args;
	uint32_t reset_reg_val;

	if (!top_priv || !reset_reg_args) {
		CAM_ERR(CAM_ISP, "Invalid arguments");
		return -EINVAL;
	}

	soc_info = top_priv->top_common.soc_info;
	reg_common = top_priv->common_data.common_reg;

	soc_private = soc_info->soc_private;
	if (!soc_private) {
		CAM_ERR(CAM_ISP, "Invalid soc_private");
		return -ENODEV;
	}

	switch (*reset_reg_args) {
	case CAM_VFE_HW_RESET_HW_AND_REG:
		if (!soc_private->is_ife_lite)
			reset_reg_val = CAM_VFE_HW_RESET_HW_AND_REG_VAL;
		else
			reset_reg_val = CAM_VFE_LITE_HW_RESET_AND_REG_VAL;
		break;
	default:
		if (!soc_private->is_ife_lite)
			reset_reg_val = CAM_VFE_HW_RESET_HW_VAL;
		else
			reset_reg_val = CAM_VFE_LITE_HW_RESET_HW_VAL;
		break;
	}

	CAM_DBG(CAM_ISP, "reset reg value: 0x%x", reset_reg_val);

	/* Mask All the IRQs except RESET */
	if (!soc_private->is_ife_lite)
		cam_io_w_mb(0x00000001,
			CAM_SOC_GET_REG_MAP_START(soc_info, VFE_CORE_BASE_IDX)
			+ 0x3C);
	else
		cam_io_w_mb(0x00020000,
			CAM_SOC_GET_REG_MAP_START(soc_info, VFE_CORE_BASE_IDX)
			+ 0x28);

	/* Reset HW */
	cam_io_w_mb(reset_reg_val,
		CAM_SOC_GET_REG_MAP_START(soc_info, VFE_CORE_BASE_IDX) +
		reg_common->global_reset_cmd);

	CAM_DBG(CAM_ISP, "Reset HW exit");
	return 0;
}

int cam_vfe_top_ver3_reserve(void *device_priv,
	void *reserve_args, uint32_t arg_size)
{
	struct cam_vfe_top_ver3_priv            *top_priv;
	struct cam_vfe_acquire_args             *args;
	struct cam_vfe_hw_vfe_in_acquire_args   *acquire_args;
	uint32_t i;
	int rc = -EINVAL;

	if (!device_priv || !reserve_args) {
		CAM_ERR(CAM_ISP, "Error, Invalid input arguments");
		return -EINVAL;
	}

	top_priv = (struct cam_vfe_top_ver3_priv   *)device_priv;
	args = (struct cam_vfe_acquire_args *)reserve_args;
	acquire_args = &args->vfe_in;

	CAM_DBG(CAM_ISP, "res id %d", acquire_args->res_id);


	for (i = 0; i < top_priv->top_common.num_mux; i++) {
		if (top_priv->top_common.mux_rsrc[i].res_id ==
			acquire_args->res_id &&
			top_priv->top_common.mux_rsrc[i].res_state ==
			CAM_ISP_RESOURCE_STATE_AVAILABLE) {

			if (acquire_args->res_id == CAM_ISP_HW_VFE_IN_CAMIF) {
				rc = cam_vfe_camif_ver3_acquire_resource(
					&top_priv->top_common.mux_rsrc[i],
					args);
				if (rc)
					break;
			}

			if (acquire_args->res_id >= CAM_ISP_HW_VFE_IN_RDI0 &&
				acquire_args->res_id < CAM_ISP_HW_VFE_IN_MAX) {
				rc = cam_vfe_camif_lite_ver3_acquire_resource(
					&top_priv->top_common.mux_rsrc[i],
					args);
				if (rc)
					break;
			}

			if (acquire_args->res_id == CAM_ISP_HW_VFE_IN_RD) {
				rc = cam_vfe_fe_ver1_acquire_resource(
					&top_priv->top_common.mux_rsrc[i],
					args);
				if (rc)
					break;
			}

			top_priv->top_common.mux_rsrc[i].cdm_ops =
				acquire_args->cdm_ops;
			top_priv->top_common.mux_rsrc[i].tasklet_info =
				args->tasklet;
			top_priv->top_common.mux_rsrc[i].res_state =
				CAM_ISP_RESOURCE_STATE_RESERVED;
			acquire_args->rsrc_node =
				&top_priv->top_common.mux_rsrc[i];

			rc = 0;
			CAM_DBG(CAM_ISP, "VFE[%u] Res [id:%d name:%s] reserved",
				top_priv->common_data.hw_intf->hw_idx,
				acquire_args->res_id,
				top_priv->top_common.mux_rsrc[i].res_name);
			break;
		}
	}

	return rc;

}

int cam_vfe_top_ver3_release(void *device_priv,
	void *release_args, uint32_t arg_size)
{
	struct cam_isp_resource_node            *mux_res;

	if (!device_priv || !release_args) {
		CAM_ERR(CAM_ISP, "Error, Invalid input arguments");
		return -EINVAL;
	}

	mux_res = (struct cam_isp_resource_node *)release_args;

	CAM_DBG(CAM_ISP, "%s Resource in state %d", mux_res->res_name,
		mux_res->res_state);
	if (mux_res->res_state < CAM_ISP_RESOURCE_STATE_RESERVED) {
		CAM_ERR(CAM_ISP, "Error, Resource in Invalid res_state :%d",
			mux_res->res_state);
		return -EINVAL;
	}
	mux_res->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;

	return 0;
}

int cam_vfe_top_ver3_start(void *device_priv,
	void *start_args, uint32_t arg_size)
{
	struct cam_vfe_top_ver3_priv            *top_priv;
	struct cam_isp_resource_node            *mux_res;
	struct cam_hw_info                      *hw_info = NULL;
	struct cam_hw_soc_info                  *soc_info = NULL;
	struct cam_vfe_soc_private              *soc_private = NULL;
	int rc = 0;

	if (!device_priv || !start_args) {
		CAM_ERR(CAM_ISP, "Error, Invalid input arguments");
		return -EINVAL;
	}

	top_priv = (struct cam_vfe_top_ver3_priv *)device_priv;
	soc_info = top_priv->top_common.soc_info;
	soc_private = soc_info->soc_private;
	if (!soc_private) {
		CAM_ERR(CAM_ISP, "Error soc_private NULL");
		return -EINVAL;
	}

	mux_res = (struct cam_isp_resource_node *)start_args;
	hw_info = (struct cam_hw_info  *)mux_res->hw_intf->hw_priv;

	if (hw_info->hw_state == CAM_HW_STATE_POWER_UP) {
		rc = cam_vfe_top_apply_clock_start_stop(&top_priv->top_common);
		if (rc) {
			CAM_ERR(CAM_ISP,
				"VFE:%d Failed in applying start clock rc:%d",
				hw_info->soc_info.index, rc);
			return rc;
		}

		rc = cam_vfe_top_apply_bw_start_stop(&top_priv->top_common);
		if (rc) {
			CAM_ERR(CAM_ISP,
				"VFE:%d Failed in applying start bw rc:%d",
				hw_info->soc_info.index, rc);
			return rc;
		}

		if (mux_res->start) {
			rc = mux_res->start(mux_res);
		} else {
			CAM_ERR(CAM_ISP,
				"Invalid res id:%d", mux_res->res_id);
			rc = -EINVAL;
		}
	} else {
		CAM_ERR(CAM_ISP, "VFE HW not powered up");
		rc = -EPERM;
	}

	return rc;
}

int cam_vfe_top_ver3_stop(void *device_priv,
	void *stop_args, uint32_t arg_size)
{
	struct cam_vfe_top_ver3_priv            *top_priv;
	struct cam_isp_resource_node            *mux_res;
	struct cam_hw_soc_info                  *soc_info = NULL;
	struct cam_vfe_soc_private              *soc_private = NULL;
	int i, rc = 0;

	if (!device_priv || !stop_args) {
		CAM_ERR(CAM_ISP, "Error, Invalid input arguments");
		return -EINVAL;
	}

	top_priv = (struct cam_vfe_top_ver3_priv   *)device_priv;
	mux_res = (struct cam_isp_resource_node *)stop_args;
	soc_info = top_priv->top_common.soc_info;
	soc_private = soc_info->soc_private;

	if (mux_res->res_id < CAM_ISP_HW_VFE_IN_MAX) {
		rc = mux_res->stop(mux_res);
	} else {
		CAM_ERR(CAM_ISP, "Invalid res id:%d", mux_res->res_id);
		return -EINVAL;
	}

	if (!rc) {
		for (i = 0; i < top_priv->top_common.num_mux; i++) {
			if (top_priv->top_common.mux_rsrc[i].res_id ==
				mux_res->res_id) {
				top_priv->top_common.req_clk_rate[i] = 0;
				memset(&top_priv->top_common.req_axi_vote[i],
					0, sizeof(struct cam_axi_vote));
				top_priv->top_common.axi_vote_control[i] =
					CAM_ISP_BW_CONTROL_EXCLUDE;
				break;
			}
		}
	}

	soc_private->ife_clk_src = 0;

	return rc;
}

int cam_vfe_top_ver3_read(void *device_priv,
	void *read_args, uint32_t arg_size)
{
	return -EPERM;
}

int cam_vfe_top_ver3_write(void *device_priv,
	void *write_args, uint32_t arg_size)
{
	return -EPERM;
}

int cam_vfe_top_ver3_process_cmd(void *device_priv, uint32_t cmd_type,
	void *cmd_args, uint32_t arg_size)
{
	int rc = 0;
	struct cam_vfe_top_ver3_priv            *top_priv;
	struct cam_hw_soc_info                  *soc_info = NULL;
	struct cam_vfe_soc_private              *soc_private = NULL;

	if (!device_priv || !cmd_args) {
		CAM_ERR(CAM_ISP, "Error, Invalid arguments");
		return -EINVAL;
	}

	top_priv = (struct cam_vfe_top_ver3_priv *)device_priv;
	soc_info = top_priv->top_common.soc_info;
	soc_private = soc_info->soc_private;
	if (!soc_private) {
		CAM_ERR(CAM_ISP, "Error soc_private NULL");
		return -EINVAL;
	}

	switch (cmd_type) {
	case CAM_ISP_HW_CMD_GET_CHANGE_BASE:
		rc = cam_vfe_top_ver3_mux_get_base(top_priv,
			cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_GET_REG_UPDATE:
		rc = cam_vfe_top_ver3_mux_get_reg_update(top_priv, cmd_args,
			arg_size);
		break;
	case CAM_ISP_HW_CMD_CAMIF_DATA:
		rc = cam_vfe_top_ver3_get_data(top_priv, cmd_args,
			arg_size);
		break;
	case CAM_ISP_HW_CMD_CLOCK_UPDATE:
		rc = cam_vfe_top_clock_update(&top_priv->top_common, cmd_args,
			arg_size);
		break;
	case CAM_ISP_HW_NOTIFY_OVERFLOW:
		rc = cam_vfe_top_ver3_dump_info(top_priv, cmd_type);
		break;
	case CAM_ISP_HW_CMD_FE_UPDATE_IN_RD:
		rc = cam_vfe_top_fs_update(top_priv, cmd_args,
			arg_size);
		break;
	case CAM_ISP_HW_CMD_BW_UPDATE:
		rc = cam_vfe_top_bw_update(soc_private, &top_priv->top_common,
			cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_BW_UPDATE_V2:
		rc = cam_vfe_top_bw_update_v2(soc_private,
			&top_priv->top_common, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_BW_CONTROL:
		rc = cam_vfe_top_bw_control(soc_private, &top_priv->top_common,
			cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_CORE_CONFIG:
		rc = cam_vfe_core_config_control(top_priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_ADD_WAIT:
		rc = cam_vfe_top_wait_comp_event(top_priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_ADD_WAIT_TRIGGER:
		rc = cam_vfe_top_add_wait_trigger(top_priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_BLANKING_UPDATE:
		rc = cam_vfe_top_ver3_blanking_update(cmd_type,
			cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_GET_PATH_PORT_MAP:
		rc = cam_vfe_top_ver3_get_path_port_map(top_priv, cmd_args,
			arg_size);
		break;
	case CAM_ISP_HW_CMD_APPLY_CLK_BW_UPDATE:
		rc = cam_vfe_top_apply_clk_bw_update(&top_priv->top_common, cmd_args, arg_size);
		break;
	default:
		rc = -EINVAL;
		CAM_ERR(CAM_ISP, "Error, Invalid cmd:%d", cmd_type);
		break;
	}

	return rc;
}

int cam_vfe_top_ver3_init(
	struct cam_hw_soc_info                 *soc_info,
	struct cam_hw_intf                     *hw_intf,
	void                                   *top_hw_info,
	void                                   *vfe_irq_controller,
	struct cam_vfe_top                    **vfe_top_ptr)
{
	int i, j, rc = 0;
	struct cam_vfe_top_ver3_priv           *top_priv = NULL;
	struct cam_vfe_top_ver3_hw_info        *ver3_hw_info = top_hw_info;
	struct cam_vfe_top                     *vfe_top;

	vfe_top = CAM_MEM_ZALLOC(sizeof(struct cam_vfe_top), GFP_KERNEL);
	if (!vfe_top) {
		CAM_DBG(CAM_ISP, "Error, Failed to alloc for vfe_top");
		rc = -ENOMEM;
		goto end;
	}

	top_priv = CAM_MEM_ZALLOC(sizeof(struct cam_vfe_top_ver3_priv),
		GFP_KERNEL);
	if (!top_priv) {
		CAM_DBG(CAM_ISP, "Error, Failed to alloc for vfe_top_priv");
		rc = -ENOMEM;
		goto free_vfe_top;
	}

	vfe_top->top_priv = top_priv;
	top_priv->top_common.applied_clk_rate = 0;
	if (ver3_hw_info->num_mux > CAM_VFE_TOP_MUX_MAX) {
		CAM_ERR(CAM_ISP, "Invalid number of input rsrc: %d, max: %d",
			ver3_hw_info->num_mux, CAM_VFE_TOP_MUX_MAX);
		rc = -EINVAL;
		goto free_top_priv;
	}

	top_priv->top_common.num_mux = ver3_hw_info->num_mux;
	top_priv->common_data.hw_info = top_hw_info;

	for (i = 0, j = 0; i < top_priv->top_common.num_mux &&
		j < CAM_VFE_RDI_VER2_MAX; i++) {
		top_priv->top_common.mux_rsrc[i].res_type =
			CAM_ISP_RESOURCE_VFE_IN;
		top_priv->top_common.mux_rsrc[i].hw_intf = hw_intf;
		top_priv->top_common.mux_rsrc[i].res_state =
			CAM_ISP_RESOURCE_STATE_AVAILABLE;
		top_priv->top_common.req_clk_rate[i] = 0;

		if (ver3_hw_info->mux_type[i] == CAM_VFE_CAMIF_VER_3_0) {
			top_priv->top_common.mux_rsrc[i].res_id =
				CAM_ISP_HW_VFE_IN_CAMIF;

			rc = cam_vfe_camif_ver3_init(hw_intf, soc_info,
				&ver3_hw_info->camif_hw_info,
				&top_priv->top_common.mux_rsrc[i],
				vfe_irq_controller);
			scnprintf(top_priv->top_common.mux_rsrc[i].res_name,
				CAM_ISP_RES_NAME_LEN, "CAMIF");
			if (rc)
				goto deinit_resources;
		} else if (ver3_hw_info->mux_type[i] ==
			CAM_VFE_PDLIB_VER_1_0) {
			/* set the PDLIB resource id */
			top_priv->top_common.mux_rsrc[i].res_id =
				CAM_ISP_HW_VFE_IN_PDLIB;

			rc = cam_vfe_camif_lite_ver3_init(hw_intf, soc_info,
				&ver3_hw_info->pdlib_hw_info,
				&top_priv->top_common.mux_rsrc[i],
				vfe_irq_controller);
			scnprintf(top_priv->top_common.mux_rsrc[i].res_name,
				CAM_ISP_RES_NAME_LEN, "PDLIB");
			if (rc)
				goto deinit_resources;
		} else if (ver3_hw_info->mux_type[i] ==
			CAM_VFE_IN_RD_VER_1_0) {
			/* set the RD resource id */
			top_priv->top_common.mux_rsrc[i].res_id =
				CAM_ISP_HW_VFE_IN_RD;

			rc = cam_vfe_fe_ver1_init(hw_intf, soc_info,
				&ver3_hw_info->fe_hw_info,
				&top_priv->top_common.mux_rsrc[i]);
			scnprintf(top_priv->top_common.mux_rsrc[i].res_name,
				CAM_ISP_RES_NAME_LEN, "IN_RD");
			if (rc)
				goto deinit_resources;
		} else if (ver3_hw_info->mux_type[i] ==
			CAM_VFE_RDI_VER_1_0) {
			/* set the RDI resource id */
			top_priv->top_common.mux_rsrc[i].res_id =
				CAM_ISP_HW_VFE_IN_RDI0 + j;

			scnprintf(top_priv->top_common.mux_rsrc[i].res_name,
				CAM_ISP_RES_NAME_LEN, "RDI_%d", j);

			rc = cam_vfe_camif_lite_ver3_init(hw_intf, soc_info,
				ver3_hw_info->rdi_hw_info[j++],
				&top_priv->top_common.mux_rsrc[i],
				vfe_irq_controller);
			if (rc)
				goto deinit_resources;
		} else if (ver3_hw_info->mux_type[i] ==
			CAM_VFE_LCR_VER_1_0) {
			/* set the LCR resource id */
			top_priv->top_common.mux_rsrc[i].res_id =
				CAM_ISP_HW_VFE_IN_LCR;

			rc = cam_vfe_camif_lite_ver3_init(hw_intf, soc_info,
				&ver3_hw_info->lcr_hw_info,
				&top_priv->top_common.mux_rsrc[i],
				vfe_irq_controller);
			scnprintf(top_priv->top_common.mux_rsrc[i].res_name,
				CAM_ISP_RES_NAME_LEN, "LCR");
			if (rc)
				goto deinit_resources;
		} else {
			CAM_WARN(CAM_ISP, "Invalid mux type: %u",
				ver3_hw_info->mux_type[i]);
		}
	}

	vfe_top->hw_ops.get_hw_caps = cam_vfe_top_ver3_get_hw_caps;
	vfe_top->hw_ops.init        = cam_vfe_top_ver3_init_hw;
	vfe_top->hw_ops.reset       = cam_vfe_top_ver3_reset;
	vfe_top->hw_ops.reserve     = cam_vfe_top_ver3_reserve;
	vfe_top->hw_ops.release     = cam_vfe_top_ver3_release;
	vfe_top->hw_ops.start       = cam_vfe_top_ver3_start;
	vfe_top->hw_ops.stop        = cam_vfe_top_ver3_stop;
	vfe_top->hw_ops.read        = cam_vfe_top_ver3_read;
	vfe_top->hw_ops.write       = cam_vfe_top_ver3_write;
	vfe_top->hw_ops.process_cmd = cam_vfe_top_ver3_process_cmd;
	*vfe_top_ptr = vfe_top;

	top_priv->top_common.soc_info      = soc_info;
	top_priv->common_data.hw_intf      = hw_intf;
	top_priv->top_common.hw_idx        = hw_intf->hw_idx;
	top_priv->common_data.common_reg   = ver3_hw_info->common_reg;

	return rc;

deinit_resources:
	for (--i; i >= 0; i--) {
		if (ver3_hw_info->mux_type[i] == CAM_VFE_CAMIF_VER_3_0) {
			if (cam_vfe_camif_ver3_deinit(
				&top_priv->top_common.mux_rsrc[i]))
				CAM_ERR(CAM_ISP, "Camif Deinit failed");
		} else if (ver3_hw_info->mux_type[i] == CAM_VFE_IN_RD_VER_1_0) {
			if (cam_vfe_fe_ver1_deinit(
				&top_priv->top_common.mux_rsrc[i]))
				CAM_ERR(CAM_ISP, "Camif fe Deinit failed");
		} else {
			if (cam_vfe_camif_lite_ver3_deinit(
				&top_priv->top_common.mux_rsrc[i]))
				CAM_ERR(CAM_ISP,
					"Camif lite res id %d Deinit failed",
					top_priv->top_common.mux_rsrc[i]
					.res_id);
		}
		top_priv->top_common.mux_rsrc[i].res_state =
			CAM_ISP_RESOURCE_STATE_UNAVAILABLE;
	}

free_top_priv:
	CAM_MEM_FREE(vfe_top->top_priv);
free_vfe_top:
	CAM_MEM_FREE(vfe_top);
end:
	return rc;
}

int cam_vfe_top_ver3_deinit(struct cam_vfe_top  **vfe_top_ptr)
{
	int i, rc = 0;
	struct cam_vfe_top_ver3_priv           *top_priv = NULL;
	struct cam_vfe_top                     *vfe_top;

	if (!vfe_top_ptr) {
		CAM_ERR(CAM_ISP, "Error, Invalid input");
		return -EINVAL;
	}

	vfe_top = *vfe_top_ptr;
	if (!vfe_top) {
		CAM_ERR(CAM_ISP, "Error, vfe_top NULL");
		return -ENODEV;
	}

	top_priv = vfe_top->top_priv;
	if (!top_priv) {
		CAM_ERR(CAM_ISP, "Error, vfe_top_priv NULL");
		rc = -ENODEV;
		goto free_vfe_top;
	}

	for (i = 0; i < top_priv->top_common.num_mux; i++) {
		top_priv->top_common.mux_rsrc[i].res_state =
			CAM_ISP_RESOURCE_STATE_UNAVAILABLE;
		if (top_priv->top_common.mux_rsrc[i].res_type ==
			CAM_VFE_CAMIF_VER_3_0) {
			rc = cam_vfe_camif_ver3_deinit(
				&top_priv->top_common.mux_rsrc[i]);
			if (rc)
				CAM_ERR(CAM_ISP, "Camif deinit failed rc=%d",
					rc);
		} else if (top_priv->top_common.mux_rsrc[i].res_type ==
			CAM_VFE_IN_RD_VER_1_0) {
			rc = cam_vfe_fe_ver1_deinit(
				&top_priv->top_common.mux_rsrc[i]);
			if (rc)
				CAM_ERR(CAM_ISP, "Camif deinit failed rc=%d",
					rc);
		} else {
			rc = cam_vfe_camif_lite_ver3_deinit(
				&top_priv->top_common.mux_rsrc[i]);
			if (rc)
				CAM_ERR(CAM_ISP,
					"Camif lite res id %d Deinit failed",
					top_priv->top_common.mux_rsrc[i]
					.res_id);
		}
	}

	CAM_MEM_FREE(vfe_top->top_priv);

free_vfe_top:
	CAM_MEM_FREE(vfe_top);
	*vfe_top_ptr = NULL;

	return rc;
}
