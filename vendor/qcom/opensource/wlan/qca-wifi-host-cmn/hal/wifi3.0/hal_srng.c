/*
 * Copyright (c) 2016-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
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

#include "hal_hw_headers.h"
#include "hal_api.h"
#include "hal_reo.h"
#include "target_type.h"
#include "qdf_module.h"
#include "wcss_version.h"
#include <qdf_tracepoint.h>
#include "qdf_ssr_driver_dump.h"

struct tcl_data_cmd gtcl_data_symbol __attribute__((used));

#ifdef QCA_WIFI_QCA8074
void hal_qca6290_attach(struct hal_soc *hal);
#endif
#ifdef QCA_WIFI_QCA8074
void hal_qca8074_attach(struct hal_soc *hal);
#endif
#if defined(QCA_WIFI_QCA8074V2) || defined(QCA_WIFI_QCA6018) || \
	defined(QCA_WIFI_QCA9574)
void hal_qca8074v2_attach(struct hal_soc *hal);
#endif
#ifdef QCA_WIFI_QCA6390
void hal_qca6390_attach(struct hal_soc *hal);
#endif
#ifdef QCA_WIFI_QCA6490
void hal_qca6490_attach(struct hal_soc *hal);
#endif
#ifdef QCA_WIFI_QCN9000
void hal_qcn9000_attach(struct hal_soc *hal);
#endif
#ifdef QCA_WIFI_QCN9224
void hal_qcn9224v2_attach(struct hal_soc *hal);
#endif
#if defined(QCA_WIFI_QCN6122) || defined(QCA_WIFI_QCN9160)
void hal_qcn6122_attach(struct hal_soc *hal);
#endif
#ifdef QCA_WIFI_QCN6432
void hal_qcn6432_attach(struct hal_soc *hal);
#endif
#ifdef QCA_WIFI_QCA6750
void hal_qca6750_attach(struct hal_soc *hal);
#endif
#ifdef QCA_WIFI_QCA5018
void hal_qca5018_attach(struct hal_soc *hal);
#endif
#ifdef QCA_WIFI_QCA5332
void hal_qca5332_attach(struct hal_soc *hal);
#endif
#ifdef QCA_WIFI_QCA5424
void hal_qca5424_attach(struct hal_soc *hal);
#endif
#ifdef INCLUDE_HAL_KIWI
void hal_kiwi_attach(struct hal_soc *hal);
#endif
#ifdef INCLUDE_HAL_PEACH
void hal_peach_attach(struct hal_soc *hal);
#endif
#ifdef QCA_WIFI_WCN7750
void hal_wcn7750_attach(struct hal_soc *hal);
#endif
#ifdef QCA_WIFI_QCC2072
void hal_qcc2072_attach(struct hal_soc *hal);
#endif

#ifdef ENABLE_VERBOSE_DEBUG
bool is_hal_verbose_debug_enabled;
#endif

#define HAL_REO_DESTINATION_RING_CTRL_IX_0_ADDR(x)	((x) + 0x4)
#define HAL_REO_DESTINATION_RING_CTRL_IX_1_ADDR(x)	((x) + 0x8)
#define HAL_REO_DESTINATION_RING_CTRL_IX_2_ADDR(x)	((x) + 0xc)
#define HAL_REO_DESTINATION_RING_CTRL_IX_3_ADDR(x)	((x) + 0x10)

#ifdef ENABLE_HAL_REG_WR_HISTORY
struct hal_reg_write_fail_history hal_reg_wr_hist;

void hal_reg_wr_fail_history_add(struct hal_soc *hal_soc,
				 uint32_t offset,
				 uint32_t wr_val, uint32_t rd_val)
{
	struct hal_reg_write_fail_entry *record;
	int idx;

	idx = hal_history_get_next_index(&hal_soc->reg_wr_fail_hist->index,
					 HAL_REG_WRITE_HIST_SIZE);

	record = &hal_soc->reg_wr_fail_hist->record[idx];

	record->timestamp = qdf_get_log_timestamp();
	record->reg_offset = offset;
	record->write_val = wr_val;
	record->read_val = rd_val;
}

static void hal_reg_write_fail_history_init(struct hal_soc *hal)
{
	hal->reg_wr_fail_hist = &hal_reg_wr_hist;

	qdf_atomic_set(&hal->reg_wr_fail_hist->index, -1);
}
#else
static void hal_reg_write_fail_history_init(struct hal_soc *hal)
{
}
#endif

/**
 * hal_get_srng_ring_id() - get the ring id of a described ring
 * @hal: hal_soc data structure
 * @ring_type: type enum describing the ring
 * @ring_num: which ring of the ring type
 * @mac_id: which mac does the ring belong to (or 0 for non-lmac rings)
 *
 * Return: the ring id or -EINVAL if the ring does not exist.
 */
static int hal_get_srng_ring_id(struct hal_soc *hal, int ring_type,
				int ring_num, int mac_id)
{
	struct hal_hw_srng_config *ring_config =
		HAL_SRNG_CONFIG(hal, ring_type);
	int ring_id;

	if (ring_num >= ring_config->max_rings) {
		QDF_TRACE(QDF_MODULE_ID_TXRX, QDF_TRACE_LEVEL_INFO,
			  "%s: ring_num exceeded maximum no. of supported rings",
			  __func__);
		/* TODO: This is a programming error. Assert if this happens */
		return -EINVAL;
	}

	/*
	 * Some DMAC rings share a common source ring, hence don't provide them
	 * with separate ring IDs per LMAC.
	 */
	if (ring_config->lmac_ring && !ring_config->dmac_cmn_ring) {
		ring_id = (ring_config->start_ring_id + ring_num +
			   (mac_id * HAL_MAX_RINGS_PER_LMAC));
	} else {
		ring_id = ring_config->start_ring_id + ring_num;
	}

	return ring_id;
}

static struct hal_srng *hal_get_srng(struct hal_soc *hal, int ring_id)
{
	/* TODO: Should we allocate srng structures dynamically? */
	return &(hal->srng_list[ring_id]);
}

#ifndef SHADOW_REG_CONFIG_DISABLED
#define HP_OFFSET_IN_REG_START 1
#define OFFSET_FROM_HP_TO_TP 4
static void hal_update_srng_hp_tp_address(struct hal_soc *hal_soc,
					  int shadow_config_index,
					  int ring_type,
					  int ring_num)
{
	struct hal_srng *srng;
	int ring_id;
	struct hal_hw_srng_config *ring_config =
		HAL_SRNG_CONFIG(hal_soc, ring_type);

	ring_id = hal_get_srng_ring_id(hal_soc, ring_type, ring_num, 0);
	if (ring_id < 0)
		return;

	srng = hal_get_srng(hal_soc, ring_id);

	if (ring_config->ring_dir == HAL_SRNG_DST_RING) {
		srng->u.dst_ring.tp_addr = SHADOW_REGISTER(shadow_config_index)
			+ hal_soc->dev_base_addr;
		hal_debug("tp_addr=%pK dev base addr %pK index %u",
			  srng->u.dst_ring.tp_addr, hal_soc->dev_base_addr,
			  shadow_config_index);
	} else {
		srng->u.src_ring.hp_addr = SHADOW_REGISTER(shadow_config_index)
			+ hal_soc->dev_base_addr;
		hal_debug("hp_addr=%pK dev base addr %pK index %u",
			  srng->u.src_ring.hp_addr,
			  hal_soc->dev_base_addr, shadow_config_index);
	}

}
#endif

#ifdef GENERIC_SHADOW_REGISTER_ACCESS_ENABLE
void hal_set_one_target_reg_config(struct hal_soc *hal,
				   uint32_t target_reg_offset,
				   int list_index)
{
	int i = list_index;

	qdf_assert_always(i < MAX_GENERIC_SHADOW_REG);
	hal->list_shadow_reg_config[i].target_register =
		target_reg_offset;
	hal->num_generic_shadow_regs_configured++;
}

qdf_export_symbol(hal_set_one_target_reg_config);

#define REO_R0_DESTINATION_RING_CTRL_ADDR_OFFSET 0x4
#define MAX_REO_REMAP_SHADOW_REGS 4
QDF_STATUS hal_set_shadow_regs(void *hal_soc)
{
	uint32_t target_reg_offset;
	struct hal_soc *hal = (struct hal_soc *)hal_soc;
	int i;
	struct hal_hw_srng_config *srng_config =
		&hal->hw_srng_table[WBM2SW_RELEASE];
	uint32_t reo_reg_base;

	reo_reg_base = hal_get_reo_reg_base_offset(hal_soc);

	target_reg_offset =
		HAL_REO_DESTINATION_RING_CTRL_IX_0_ADDR(reo_reg_base);

	for (i = 0; i < MAX_REO_REMAP_SHADOW_REGS; i++) {
		hal_set_one_target_reg_config(hal, target_reg_offset, i);
		target_reg_offset += REO_R0_DESTINATION_RING_CTRL_ADDR_OFFSET;
	}

	target_reg_offset = srng_config->reg_start[HP_OFFSET_IN_REG_START];
	target_reg_offset += (srng_config->reg_size[HP_OFFSET_IN_REG_START]
			      * HAL_IPA_TX_COMP_RING_IDX);

	hal_set_one_target_reg_config(hal, target_reg_offset, i);
	return QDF_STATUS_SUCCESS;
}

qdf_export_symbol(hal_set_shadow_regs);

QDF_STATUS hal_construct_shadow_regs(void *hal_soc)
{
	struct hal_soc *hal = (struct hal_soc *)hal_soc;
	int shadow_config_index = hal->num_shadow_registers_configured;
	int i;
	int num_regs = hal->num_generic_shadow_regs_configured;

	for (i = 0; i < num_regs; i++) {
		qdf_assert_always(shadow_config_index < MAX_SHADOW_REGISTERS);
		hal->shadow_config[shadow_config_index].addr =
			hal->list_shadow_reg_config[i].target_register;
		hal->list_shadow_reg_config[i].shadow_config_index =
			shadow_config_index;
		hal->list_shadow_reg_config[i].va =
			SHADOW_REGISTER(shadow_config_index) +
			(uintptr_t)hal->dev_base_addr;
		hal_debug("target_reg %x, shadow register 0x%x shadow_index 0x%x",
			  hal->shadow_config[shadow_config_index].addr,
			  SHADOW_REGISTER(shadow_config_index),
			  shadow_config_index);
		shadow_config_index++;
		hal->num_shadow_registers_configured++;
	}
	return QDF_STATUS_SUCCESS;
}

qdf_export_symbol(hal_construct_shadow_regs);
#endif

#ifndef SHADOW_REG_CONFIG_DISABLED

QDF_STATUS hal_set_one_shadow_config(void *hal_soc,
				     int ring_type,
				     int ring_num)
{
	uint32_t target_register;
	struct hal_soc *hal = (struct hal_soc *)hal_soc;
	struct hal_hw_srng_config *srng_config = &hal->hw_srng_table[ring_type];
	int shadow_config_index = hal->num_shadow_registers_configured;

	if (shadow_config_index >= MAX_SHADOW_REGISTERS) {
		QDF_ASSERT(0);
		return QDF_STATUS_E_RESOURCES;
	}

	hal->num_shadow_registers_configured++;

	target_register = srng_config->reg_start[HP_OFFSET_IN_REG_START];
	target_register += (srng_config->reg_size[HP_OFFSET_IN_REG_START]
			    *ring_num);

	/* if the ring is a dst ring, we need to shadow the tail pointer */
	if (srng_config->ring_dir == HAL_SRNG_DST_RING)
		target_register += OFFSET_FROM_HP_TO_TP;

	hal->shadow_config[shadow_config_index].addr = target_register;

	/* update hp/tp addr in the hal_soc structure*/
	hal_update_srng_hp_tp_address(hal_soc, shadow_config_index, ring_type,
				      ring_num);

	hal_debug("target_reg %x, shadow register 0x%x shadow_index 0x%x, ring_type %d, ring num %d",
		  target_register,
		  SHADOW_REGISTER(shadow_config_index),
		  shadow_config_index,
		  ring_type, ring_num);

	return QDF_STATUS_SUCCESS;
}

qdf_export_symbol(hal_set_one_shadow_config);

QDF_STATUS hal_construct_srng_shadow_regs(void *hal_soc)
{
	int ring_type, ring_num;
	struct hal_soc *hal = (struct hal_soc *)hal_soc;

	for (ring_type = 0; ring_type < MAX_RING_TYPES; ring_type++) {
		struct hal_hw_srng_config *srng_config =
			&hal->hw_srng_table[ring_type];

		if (ring_type == CE_SRC ||
		    ring_type == CE_DST ||
		    ring_type == CE_DST_STATUS)
			continue;

		if (srng_config->lmac_ring)
			continue;

		for (ring_num = 0; ring_num < srng_config->max_rings;
		     ring_num++)
			hal_set_one_shadow_config(hal_soc, ring_type, ring_num);
	}

	return QDF_STATUS_SUCCESS;
}

qdf_export_symbol(hal_construct_srng_shadow_regs);
#else

QDF_STATUS hal_construct_srng_shadow_regs(void *hal_soc)
{
	return QDF_STATUS_SUCCESS;
}

qdf_export_symbol(hal_construct_srng_shadow_regs);

QDF_STATUS hal_set_one_shadow_config(void *hal_soc, int ring_type,
				     int ring_num)
{
	return QDF_STATUS_SUCCESS;
}
qdf_export_symbol(hal_set_one_shadow_config);
#endif

void hal_get_shadow_config(void *hal_soc,
	struct pld_shadow_reg_v2_cfg **shadow_config,
	int *num_shadow_registers_configured)
{
	struct hal_soc *hal = (struct hal_soc *)hal_soc;

	*shadow_config = &hal->shadow_config[0].v2;
	*num_shadow_registers_configured =
		hal->num_shadow_registers_configured;
}
qdf_export_symbol(hal_get_shadow_config);

#ifdef CONFIG_SHADOW_V3
void hal_get_shadow_v3_config(void *hal_soc,
			      struct pld_shadow_reg_v3_cfg **shadow_config,
			      int *num_shadow_registers_configured)
{
	struct hal_soc *hal = (struct hal_soc *)hal_soc;

	*shadow_config = &hal->shadow_config[0].v3;
	*num_shadow_registers_configured =
		hal->num_shadow_registers_configured;
}
qdf_export_symbol(hal_get_shadow_v3_config);
#endif

static bool hal_validate_shadow_register(struct hal_soc *hal,
					 uint32_t *destination,
					 uint32_t *shadow_address)
{
	unsigned int index;
	uint32_t *shadow_0_offset = SHADOW_REGISTER(0) + hal->dev_base_addr;
	int destination_ba_offset =
		((char *)destination) - (char *)hal->dev_base_addr;

	index =	shadow_address - shadow_0_offset;

	if (index >= MAX_SHADOW_REGISTERS) {
		QDF_TRACE(QDF_MODULE_ID_TXRX, QDF_TRACE_LEVEL_ERROR,
			"%s: index %x out of bounds", __func__, index);
		goto error;
	} else if (hal->shadow_config[index].addr != destination_ba_offset) {
		QDF_TRACE(QDF_MODULE_ID_TXRX, QDF_TRACE_LEVEL_ERROR,
			"%s: sanity check failure, expected %x, found %x",
			__func__, destination_ba_offset,
			hal->shadow_config[index].addr);
		goto error;
	}
	return true;
error:
	qdf_print("baddr %pK, destination %pK, shadow_address %pK s0offset %pK index %x",
		  hal->dev_base_addr, destination, shadow_address,
		  shadow_0_offset, index);
	QDF_BUG(0);
	return false;
}

static void hal_target_based_configure(struct hal_soc *hal)
{
	/*
	 * Indicate Initialization of srngs to avoid force wake
	 * as umac power collapse is not enabled yet
	 */
	hal->init_phase = true;

	switch (hal->target_type) {
#ifdef QCA_WIFI_QCA6290
	case TARGET_TYPE_QCA6290:
		hal->use_register_windowing = true;
		hal_qca6290_attach(hal);
	break;
#endif
#ifdef QCA_WIFI_QCA6390
	case TARGET_TYPE_QCA6390:
		hal->use_register_windowing = true;
		hal_qca6390_attach(hal);
	break;
#endif
#ifdef QCA_WIFI_QCA6490
	case TARGET_TYPE_QCA6490:
		hal->use_register_windowing = true;
		hal_qca6490_attach(hal);
	break;
#endif
#ifdef QCA_WIFI_QCA6750
		case TARGET_TYPE_QCA6750:
			hal->use_register_windowing = true;
			hal->static_window_map = true;
			hal_qca6750_attach(hal);
		break;
#endif
#ifdef QCA_WIFI_WCN7750
		case TARGET_TYPE_WCN7750:
			hal->use_register_windowing = true;
			hal->static_window_map = true;
			hal_wcn7750_attach(hal);
		break;
#endif
#ifdef QCA_WIFI_QCC2072
	case TARGET_TYPE_QCC2072:
		hal->use_register_windowing = true;
		hal_qcc2072_attach(hal);
#endif
#ifdef INCLUDE_HAL_KIWI
	case TARGET_TYPE_KIWI:
	case TARGET_TYPE_MANGO:
		hal->use_register_windowing = true;
		hal_kiwi_attach(hal);
		break;
#endif
#ifdef INCLUDE_HAL_PEACH
	case TARGET_TYPE_PEACH:
		hal->use_register_windowing = true;
		hal_peach_attach(hal);
		break;
#endif
#if defined(QCA_WIFI_QCA8074) && defined(WIFI_TARGET_TYPE_3_0)
	case TARGET_TYPE_QCA8074:
		hal_qca8074_attach(hal);
	break;
#endif

#if defined(QCA_WIFI_QCA8074V2)
	case TARGET_TYPE_QCA8074V2:
		hal_qca8074v2_attach(hal);
	break;
#endif

#if defined(QCA_WIFI_QCA6018)
	case TARGET_TYPE_QCA6018:
		hal_qca8074v2_attach(hal);
	break;
#endif

#if defined(QCA_WIFI_QCA9574)
	case TARGET_TYPE_QCA9574:
		hal_qca8074v2_attach(hal);
	break;
#endif

#if defined(QCA_WIFI_QCN6122)
	case TARGET_TYPE_QCN6122:
		hal->use_register_windowing = true;
		/*
		 * Static window map  is enabled for qcn9000 to use 2mb bar
		 * size and use multiple windows to write into registers.
		 */
		hal->static_window_map = true;
		hal_qcn6122_attach(hal);
		break;
#endif

#if defined(QCA_WIFI_QCN9160)
	case TARGET_TYPE_QCN9160:
		hal->use_register_windowing = true;
		/*
		 * Static window map  is enabled for qcn9160 to use 2mb bar
		 * size and use multiple windows to write into registers.
		 */
		hal->static_window_map = true;
		hal_qcn6122_attach(hal);
		break;
#endif

#if defined(QCA_WIFI_QCN6432)
	case TARGET_TYPE_QCN6432:
		hal->use_register_windowing = true;
		/*
		 * Static window map  is enabled for qcn6432 to use 2mb bar
		 * size and use multiple windows to write into registers.
		 */
		hal->static_window_map = true;
		hal_qcn6432_attach(hal);
		break;
#endif

#ifdef QCA_WIFI_QCN9000
	case TARGET_TYPE_QCN9000:
		hal->use_register_windowing = true;
		/*
		 * Static window map  is enabled for qcn9000 to use 2mb bar
		 * size and use multiple windows to write into registers.
		 */
		hal->static_window_map = true;
		hal_qcn9000_attach(hal);
	break;
#endif
#ifdef QCA_WIFI_QCA5018
	case TARGET_TYPE_QCA5018:
		hal->use_register_windowing = true;
		hal->static_window_map = true;
		hal_qca5018_attach(hal);
	break;
#endif
#ifdef QCA_WIFI_QCN9224
	case TARGET_TYPE_QCN9224:
		hal->use_register_windowing = true;
		hal->static_window_map = true;
		if (hal->version == 1)
			qdf_assert_always(0);
		else
			hal_qcn9224v2_attach(hal);
	break;
#endif
#ifdef QCA_WIFI_QCA5332
	case TARGET_TYPE_QCA5332:
		hal->use_register_windowing = true;
		hal->static_window_map = true;
		hal_qca5332_attach(hal);
	break;
#endif
#ifdef QCA_WIFI_WCN6450
	case TARGET_TYPE_WCN6450:
		hal->use_register_windowing = true;
		hal->static_window_map = true;
		hal_wcn6450_attach(hal);
	break;
#endif
#if defined(QCA_WIFI_QCA5424)
	case TARGET_TYPE_QCA5424:
		hal->use_register_windowing = true;
		/*
		 * Static window map  is enabled for qcn6432 to use 2mb bar
		 * size and use multiple windows to write into registers.
		 */
		hal->static_window_map = true;
		hal_qca5424_attach(hal);
		break;
#endif
	default:
	break;
	}
}

uint32_t hal_get_target_type(hal_soc_handle_t hal_soc_hdl)
{
	struct hal_soc *hal_soc = (struct hal_soc *)hal_soc_hdl;
	struct hif_target_info *tgt_info =
		hif_get_target_info_handle(hal_soc->hif_handle);

	return tgt_info->target_type;
}

qdf_export_symbol(hal_get_target_type);

#if defined(FEATURE_HAL_DELAYED_REG_WRITE)
/**
 * hal_is_reg_write_tput_level_high() - throughput level for delayed reg writes
 * @hal: hal_soc pointer
 *
 * Return: true if throughput is high, else false.
 */
static inline bool hal_is_reg_write_tput_level_high(struct hal_soc *hal)
{
	int bw_level = hif_get_bandwidth_level(hal->hif_handle);

	return (bw_level >= PLD_BUS_WIDTH_MEDIUM) ? true : false;
}

static inline
char *hal_fill_reg_write_srng_stats(struct hal_srng *srng,
				    char *buf, qdf_size_t size)
{
	qdf_scnprintf(buf, size, "enq %u deq %u coal %u direct %u",
		      srng->wstats.enqueues, srng->wstats.dequeues,
		      srng->wstats.coalesces, srng->wstats.direct);
	return buf;
}

/* bytes for local buffer */
#define HAL_REG_WRITE_SRNG_STATS_LEN 100

#ifndef WLAN_SOFTUMAC_SUPPORT
void hal_dump_reg_write_srng_stats(hal_soc_handle_t hal_soc_hdl)
{
	struct hal_srng *srng;
	char buf[HAL_REG_WRITE_SRNG_STATS_LEN];
	struct hal_soc *hal = (struct hal_soc *)hal_soc_hdl;

	srng = hal_get_srng(hal, HAL_SRNG_SW2TCL1);
	hal_debug("SW2TCL1: %s",
		  hal_fill_reg_write_srng_stats(srng, buf, sizeof(buf)));

	srng = hal_get_srng(hal, HAL_SRNG_WBM2SW0_RELEASE);
	hal_debug("WBM2SW0: %s",
		  hal_fill_reg_write_srng_stats(srng, buf, sizeof(buf)));

	srng = hal_get_srng(hal, HAL_SRNG_REO2SW1);
	hal_debug("REO2SW1: %s",
		  hal_fill_reg_write_srng_stats(srng, buf, sizeof(buf)));

	srng = hal_get_srng(hal, HAL_SRNG_REO2SW2);
	hal_debug("REO2SW2: %s",
		  hal_fill_reg_write_srng_stats(srng, buf, sizeof(buf)));

	srng = hal_get_srng(hal, HAL_SRNG_REO2SW3);
	hal_debug("REO2SW3: %s",
		  hal_fill_reg_write_srng_stats(srng, buf, sizeof(buf)));
}

void hal_dump_reg_write_stats(hal_soc_handle_t hal_soc_hdl)
{
	uint32_t *hist;
	struct hal_soc *hal = (struct hal_soc *)hal_soc_hdl;

	hist = hal->stats.wstats.sched_delay;
	hal_debug("wstats: enq %u deq %u coal %u direct %u q_depth %u max_q %u sched-delay hist %u %u %u %u",
		  qdf_atomic_read(&hal->stats.wstats.enqueues),
		  hal->stats.wstats.dequeues,
		  qdf_atomic_read(&hal->stats.wstats.coalesces),
		  qdf_atomic_read(&hal->stats.wstats.direct),
		  qdf_atomic_read(&hal->stats.wstats.q_depth),
		  hal->stats.wstats.max_q_depth,
		  hist[REG_WRITE_SCHED_DELAY_SUB_100us],
		  hist[REG_WRITE_SCHED_DELAY_SUB_1000us],
		  hist[REG_WRITE_SCHED_DELAY_SUB_5000us],
		  hist[REG_WRITE_SCHED_DELAY_GT_5000us]);
}
#else
void hal_dump_reg_write_srng_stats(hal_soc_handle_t hal_soc_hdl)
{
}

/* TODO: Need separate logic for Evros */
void hal_dump_reg_write_stats(hal_soc_handle_t hal_soc_hdl)
{
}
#endif

int hal_get_reg_write_pending_work(void *hal_soc)
{
	struct hal_soc *hal = (struct hal_soc *)hal_soc;

	return qdf_atomic_read(&hal->active_work_cnt);
}

#endif

#ifdef FEATURE_HAL_DELAYED_REG_WRITE
#ifdef MEMORY_DEBUG
/*
 * Length of the queue(array) used to hold delayed register writes.
 * Must be a multiple of 2.
 */
#define HAL_REG_WRITE_QUEUE_LEN 128
#else
#define HAL_REG_WRITE_QUEUE_LEN 32
#endif

#ifdef QCA_WIFI_QCA6750

#define HAL_DEL_WRITE_FORCE_UPDATE_THRES 5

static inline void hal_srng_update_last_hptp(struct hal_srng *srng)
{
	if (srng->ring_dir == HAL_SRNG_SRC_RING)
		srng->updated_hp = srng->u.src_ring.hp;
	else
		srng->updated_tp = srng->u.dst_ring.tp;

	srng->force_cnt = 0;
}

/* If HP/TP register updates are delayed due to delayed reg
 * write work not getting scheduled, hardware would see HP/TP
 * delta and will fire interrupts until the HP/TP updates reach
 * the hardware.
 *
 * When system is heavily stressed, this delay in HP/TP updates
 * would result in IRQ storm further stressing the system. Force
 * update HP/TP to the hardware under such scenarios to avoid this.
 */
void hal_srng_check_and_update_hptp(struct hal_soc *hal_soc,
				    struct hal_srng *srng, bool update)
{
	uint32_t value;

	if (!update)
		return;

	SRNG_LOCK(&srng->lock);
	if (srng->ring_dir == HAL_SRNG_SRC_RING) {
		value = srng->u.src_ring.hp;

		if (value == srng->updated_hp ||
		    srng->force_cnt++ < HAL_DEL_WRITE_FORCE_UPDATE_THRES)
			goto out_unlock;

		hal_write_address_32_mb(hal_soc, srng->u.src_ring.hp_addr,
					value, false);
	} else {
		value = srng->u.dst_ring.tp;

		if (value == srng->updated_tp ||
		    srng->force_cnt++ < HAL_DEL_WRITE_FORCE_UPDATE_THRES)
			goto out_unlock;

		hal_write_address_32_mb(hal_soc, srng->u.dst_ring.tp_addr,
					value, false);
	}

	hal_srng_update_last_hptp(srng);
	hal_srng_reg_his_add(srng, value);
	qdf_atomic_inc(&hal_soc->stats.wstats.direct);
	srng->wstats.direct++;

out_unlock:
	SRNG_UNLOCK(&srng->lock);
}
#else
static inline void hal_srng_update_last_hptp(struct hal_srng *srng)
{
}
#endif /* QCA_WIFI_QCA6750 */

/**
 * hal_process_reg_write_q_elem() - process a register write queue element
 * @hal: hal_soc pointer
 * @q_elem: pointer to hal register write queue element
 *
 * Return: The value which was written to the address
 */
static uint32_t
hal_process_reg_write_q_elem(struct hal_soc *hal,
			     struct hal_reg_write_q_elem *q_elem)
{
	struct hal_srng *srng = q_elem->srng;
	uint32_t write_val;

	SRNG_LOCK(&srng->lock);

	srng->reg_write_in_progress = false;
	srng->wstats.dequeues++;

	if (srng->ring_dir == HAL_SRNG_SRC_RING) {
		q_elem->dequeue_val = srng->u.src_ring.hp;
		hal_write_address_32_mb(hal,
					srng->u.src_ring.hp_addr,
					srng->u.src_ring.hp, false);
		write_val = srng->u.src_ring.hp;
	} else {
		q_elem->dequeue_val = srng->u.dst_ring.tp;
		hal_write_address_32_mb(hal,
					srng->u.dst_ring.tp_addr,
					srng->u.dst_ring.tp, false);
		write_val = srng->u.dst_ring.tp;
	}

	hal_srng_update_last_hptp(srng);
	hal_srng_reg_his_add(srng, write_val);

	q_elem->valid = 0;
	srng->last_dequeue_time = q_elem->dequeue_time;
	SRNG_UNLOCK(&srng->lock);

	return write_val;
}

/**
 * hal_reg_write_fill_sched_delay_hist() - fill reg write delay histogram in hal
 * @hal: hal_soc pointer
 * @delay_us: delay in us
 *
 * Return: None
 */
static inline void hal_reg_write_fill_sched_delay_hist(struct hal_soc *hal,
						       uint64_t delay_us)
{
	uint32_t *hist;

	hist = hal->stats.wstats.sched_delay;

	if (delay_us < 100)
		hist[REG_WRITE_SCHED_DELAY_SUB_100us]++;
	else if (delay_us < 1000)
		hist[REG_WRITE_SCHED_DELAY_SUB_1000us]++;
	else if (delay_us < 5000)
		hist[REG_WRITE_SCHED_DELAY_SUB_5000us]++;
	else
		hist[REG_WRITE_SCHED_DELAY_GT_5000us]++;
}

#ifdef SHADOW_WRITE_DELAY

#define SHADOW_WRITE_MIN_DELTA_US	5
#define SHADOW_WRITE_DELAY_US		50

/*
 * Never add those srngs which are performance relate.
 * The delay itself will hit performance heavily.
 */
#define IS_SRNG_MATCH(s)	((s)->ring_id == HAL_SRNG_CE_1_DST_STATUS || \
				 (s)->ring_id == HAL_SRNG_CE_1_DST)

static inline bool hal_reg_write_need_delay(struct hal_reg_write_q_elem *elem)
{
	struct hal_srng *srng = elem->srng;
	struct hal_soc *hal;
	qdf_time_t now;
	qdf_iomem_t real_addr;

	if (qdf_unlikely(!srng))
		return false;

	hal = srng->hal_soc;
	if (qdf_unlikely(!hal))
		return false;

	/* Check if it is target srng, and valid shadow reg */
	if (qdf_likely(!IS_SRNG_MATCH(srng)))
		return false;

	if (srng->ring_dir == HAL_SRNG_SRC_RING)
		real_addr = SRNG_SRC_ADDR(srng, HP);
	else
		real_addr = SRNG_DST_ADDR(srng, TP);
	if (!hal_validate_shadow_register(hal, real_addr, elem->addr))
		return false;

	/* Check the time delta from last write of same srng */
	now = qdf_get_log_timestamp();
	if (qdf_log_timestamp_to_usecs(now - srng->last_dequeue_time) >
		SHADOW_WRITE_MIN_DELTA_US)
		return false;

	/* Delay dequeue, and record */
	qdf_udelay(SHADOW_WRITE_DELAY_US);

	srng->wstats.dequeue_delay++;
	hal->stats.wstats.dequeue_delay++;

	return true;
}
#else
static inline bool hal_reg_write_need_delay(struct hal_reg_write_q_elem *elem)
{
	return false;
}
#endif

#define MAX_DELAYED_REG_WRITE_RETRY 5

/**
 * hal_reg_write_work() - Worker to process delayed writes
 * @arg: hal_soc pointer
 *
 * Return: None
 */
static void hal_reg_write_work(void *arg)
{
	int32_t q_depth, write_val;
	struct hal_soc *hal = arg;
	struct hal_reg_write_q_elem *q_elem;
	uint64_t delta_us;
	uint8_t ring_id;
	uint32_t *addr;
	uint32_t num_processed = 0;
	uint8_t retry_count = 0;

	q_elem = &hal->reg_write_queue[(hal->read_idx)];
	q_elem->work_scheduled_time = qdf_get_log_timestamp();
	q_elem->cpu_id = qdf_get_cpu();

	/* Make sure q_elem consistent in the memory for multi-cores */
	qdf_rmb();
	if (!q_elem->valid)
		return;

	q_depth = qdf_atomic_read(&hal->stats.wstats.q_depth);
	if (q_depth > hal->stats.wstats.max_q_depth)
		hal->stats.wstats.max_q_depth =  q_depth;

	if (hif_prevent_link_low_power_states(hal->hif_handle)) {
		hal->stats.wstats.prevent_l1_fails++;
		return;
	}

	while (true) {
		qdf_rmb();
		if (!q_elem->valid)
			break;

		qdf_rmb();
		/* buy some more time to make sure all fields
		 * in q_elem is updated per different CPUs, in
		 * case wmb/rmb is not taken effect
		 */
		if (qdf_unlikely(!q_elem->srng ||
				 (qdf_atomic_read(&q_elem->ring_id) !=
				 q_elem->srng->ring_id))) {
			hal_err_rl("q_elem fields not up to date 0x%x 0x%x",
				   q_elem->srng ? q_elem->srng->ring_id : 0xDEAD,
				   qdf_atomic_read(&q_elem->ring_id));
			if (retry_count++ < MAX_DELAYED_REG_WRITE_RETRY) {
				/* Sleep for 1ms before retry */
				qdf_sleep(1);
				continue;
			}
			qdf_assert_always(0);
		}

		q_elem->dequeue_time = qdf_get_log_timestamp();
		ring_id = q_elem->srng->ring_id;
		addr = q_elem->addr;
		delta_us = qdf_log_timestamp_to_usecs(q_elem->dequeue_time -
						      q_elem->enqueue_time);
		hal_reg_write_fill_sched_delay_hist(hal, delta_us);

		hal->stats.wstats.dequeues++;
		qdf_atomic_dec(&hal->stats.wstats.q_depth);

		if (hal_reg_write_need_delay(q_elem))
			hal_verbose_debug("Delay reg writer for srng 0x%x, addr 0x%pK",
					  q_elem->srng->ring_id, q_elem->addr);

		write_val = hal_process_reg_write_q_elem(hal, q_elem);
		hal_verbose_debug("read_idx %u srng 0x%x, addr 0x%pK dequeue_val %u sched delay %llu us",
				  hal->read_idx, ring_id, addr, write_val, delta_us);

		qdf_trace_dp_del_reg_write(ring_id, q_elem->enqueue_val,
					   q_elem->dequeue_val,
					   q_elem->enqueue_time,
					   q_elem->dequeue_time);

		num_processed++;
		hal->read_idx = (hal->read_idx + 1) &
					(HAL_REG_WRITE_QUEUE_LEN - 1);
		q_elem = &hal->reg_write_queue[(hal->read_idx)];
		retry_count = 0;
	}

	hif_allow_link_low_power_states(hal->hif_handle);
	/*
	 * Decrement active_work_cnt by the number of elements dequeued after
	 * hif_allow_link_low_power_states.
	 * This makes sure that hif_try_complete_tasks will wait till we make
	 * the bus access in hif_allow_link_low_power_states. This will avoid
	 * race condition between delayed register worker and bus suspend
	 * (system suspend or runtime suspend).
	 *
	 * The following decrement should be done at the end!
	 */
	qdf_atomic_sub(num_processed, &hal->active_work_cnt);
}

static void __hal_flush_reg_write_work(struct hal_soc *hal)
{
	qdf_flush_work(&hal->reg_write_work);
	qdf_disable_work(&hal->reg_write_work);
}

void hal_flush_reg_write_work(hal_soc_handle_t hal_handle)
{	__hal_flush_reg_write_work((struct hal_soc *)hal_handle);
}

/**
 * hal_reg_write_enqueue() - enqueue register writes into kworker
 * @hal_soc: hal_soc pointer
 * @srng: srng pointer
 * @addr: iomem address of register
 * @value: value to be written to iomem address
 *
 * This function executes from within the SRNG LOCK
 *
 * Return: None
 */
static void hal_reg_write_enqueue(struct hal_soc *hal_soc,
				  struct hal_srng *srng,
				  void __iomem *addr,
				  uint32_t value)
{
	struct hal_reg_write_q_elem *q_elem;
	uint32_t write_idx;

	if (srng->reg_write_in_progress) {
		hal_verbose_debug("Already in progress srng ring id 0x%x addr 0x%pK val %u",
				  srng->ring_id, addr, value);
		qdf_atomic_inc(&hal_soc->stats.wstats.coalesces);
		srng->wstats.coalesces++;
		return;
	}

	write_idx = qdf_atomic_inc_return(&hal_soc->write_idx);

	write_idx = write_idx & (HAL_REG_WRITE_QUEUE_LEN - 1);

	q_elem = &hal_soc->reg_write_queue[write_idx];

	if (q_elem->valid) {
		hal_err("queue full");
		QDF_BUG(0);
		return;
	}

	qdf_atomic_inc(&hal_soc->stats.wstats.enqueues);
	srng->wstats.enqueues++;

	qdf_atomic_inc(&hal_soc->stats.wstats.q_depth);

	q_elem->srng = srng;
	q_elem->addr = addr;
	qdf_atomic_set(&q_elem->ring_id, srng->ring_id);
	q_elem->enqueue_val = value;
	q_elem->enqueue_time = qdf_get_log_timestamp();

	/*
	 * Before the valid flag is set to true, all the other
	 * fields in the q_elem needs to be updated in memory.
	 * Else there is a chance that the dequeuing worker thread
	 * might read stale entries and process incorrect srng.
	 */
	qdf_wmb();
	q_elem->valid = true;

	/*
	 * After all other fields in the q_elem has been updated
	 * in memory successfully, the valid flag needs to be updated
	 * in memory in time too.
	 * Else there is a chance that the dequeuing worker thread
	 * might read stale valid flag and the work will be bypassed
	 * for this round. And if there is no other work scheduled
	 * later, this hal register writing won't be updated any more.
	 */
	qdf_wmb();

	srng->reg_write_in_progress  = true;
	qdf_atomic_inc(&hal_soc->active_work_cnt);

	hal_verbose_debug("write_idx %u srng ring id 0x%x addr 0x%pK val %u",
			  write_idx, srng->ring_id, addr, value);

	qdf_queue_work(hal_soc->qdf_dev, hal_soc->reg_write_wq,
		       &hal_soc->reg_write_work);
}

/**
 * hal_delayed_reg_write_init() - Initialization function for delayed reg writes
 * @hal: hal_soc pointer
 *
 * Initialize main data structures to process register writes in a delayed
 * workqueue.
 *
 * Return: QDF_STATUS_SUCCESS on success else a QDF error.
 */
static QDF_STATUS hal_delayed_reg_write_init(struct hal_soc *hal)
{
	hal->reg_write_wq =
		qdf_alloc_high_prior_ordered_workqueue("hal_register_write_wq");
	qdf_create_work(0, &hal->reg_write_work, hal_reg_write_work, hal);
	hal->reg_write_queue = qdf_mem_malloc(HAL_REG_WRITE_QUEUE_LEN *
					      sizeof(*hal->reg_write_queue));
	if (!hal->reg_write_queue) {
		hal_err("unable to allocate memory");
		QDF_BUG(0);
		return QDF_STATUS_E_NOMEM;
	}

	/* Initial value of indices */
	hal->read_idx = 0;
	qdf_atomic_set(&hal->write_idx, -1);
	return QDF_STATUS_SUCCESS;
}

/**
 * hal_delayed_reg_write_deinit() - De-Initialize delayed reg write processing
 * @hal: hal_soc pointer
 *
 * De-initialize main data structures to process register writes in a delayed
 * workqueue.
 *
 * Return: None
 */
static void hal_delayed_reg_write_deinit(struct hal_soc *hal)
{
	__hal_flush_reg_write_work(hal);

	qdf_flush_workqueue(0, hal->reg_write_wq);
	qdf_destroy_workqueue(0, hal->reg_write_wq);
	qdf_mem_free(hal->reg_write_queue);
}

#else
static inline QDF_STATUS hal_delayed_reg_write_init(struct hal_soc *hal)
{
	return QDF_STATUS_SUCCESS;
}

static inline void hal_delayed_reg_write_deinit(struct hal_soc *hal)
{
}
#endif

#ifdef FEATURE_HAL_DELAYED_REG_WRITE
#ifdef HAL_RECORD_SUSPEND_WRITE
static struct hal_suspend_write_history
		g_hal_suspend_write_history[HAL_SUSPEND_WRITE_HISTORY_MAX];

static
void hal_event_suspend_record(uint8_t ring_id, uint32_t value, uint32_t count)
{
	uint32_t index = qdf_atomic_read(g_hal_suspend_write_history.index) &
					(HAL_SUSPEND_WRITE_HISTORY_MAX - 1);
	struct hal_suspend_write_record *cur_event =
					&hal_suspend_write_event.record[index];

	cur_event->ts = qdf_get_log_timestamp();
	cur_event->ring_id = ring_id;
	cur_event->value = value;
	cur_event->direct_wcount = count;
	qdf_atomic_inc(g_hal_suspend_write_history.index);
}

static inline
void hal_record_suspend_write(uint8_t ring_id, uint32_t value, uint32_t count)
{
	if (hif_rtpm_get_state() >= HIF_RTPM_STATE_SUSPENDING)
		hal_event_suspend_record(ring_id, value, count);
}
#else
static inline
void hal_record_suspend_write(uint8_t ring_id, uint32_t value, uint32_t count)
{
}
#endif

#if defined(QCA_WIFI_QCA6750) || defined(QCA_WIFI_WCN7750)
void hal_delayed_reg_write(struct hal_soc *hal_soc,
			   struct hal_srng *srng,
			   void __iomem *addr,
			   uint32_t value)
{
	uint8_t vote_access;

	switch (srng->ring_type) {
	case CE_SRC:
	case CE_DST:
	case CE_DST_STATUS:
		vote_access = hif_get_ep_vote_access(hal_soc->hif_handle,
						     HIF_EP_VOTE_NONDP_ACCESS);
		if ((vote_access == HIF_EP_VOTE_ACCESS_DISABLE) ||
		    (vote_access == HIF_EP_VOTE_INTERMEDIATE_ACCESS &&
		     PLD_MHI_STATE_L0 ==
		     pld_get_mhi_state(hal_soc->qdf_dev->dev))) {
			hal_write_address_32_mb(hal_soc, addr, value, false);
			hal_srng_update_last_hptp(srng);
			hal_srng_reg_his_add(srng, value);
			qdf_atomic_inc(&hal_soc->stats.wstats.direct);
			srng->wstats.direct++;
		} else {
			hal_reg_write_enqueue(hal_soc, srng, addr, value);
		}
		break;
	default:
		if (hif_get_ep_vote_access(hal_soc->hif_handle,
		    HIF_EP_VOTE_DP_ACCESS) ==
		    HIF_EP_VOTE_ACCESS_DISABLE ||
		    hal_is_reg_write_tput_level_high(hal_soc) ||
		    PLD_MHI_STATE_L0 ==
		    pld_get_mhi_state(hal_soc->qdf_dev->dev)) {
			hal_write_address_32_mb(hal_soc, addr, value, false);
			hal_srng_reg_his_add(srng, value);
			qdf_atomic_inc(&hal_soc->stats.wstats.direct);
			srng->wstats.direct++;
		} else {
			hal_reg_write_enqueue(hal_soc, srng, addr, value);
		}

		break;
	}
}
#else
void hal_delayed_reg_write(struct hal_soc *hal_soc,
			   struct hal_srng *srng,
			   void __iomem *addr,
			   uint32_t value)
{
	if (hal_is_reg_write_tput_level_high(hal_soc) ||
	    pld_is_device_awake(hal_soc->qdf_dev->dev) ||
	    hal_srng_is_delay_reg_force_write(srng)) {
		hal_srng_delay_reg_record_direct_write(srng, true);
		qdf_atomic_inc(&hal_soc->stats.wstats.direct);
		srng->wstats.direct++;
		hal_write_address_32_mb(hal_soc, addr, value, false);
		hal_srng_update_last_hptp(srng);
		hal_srng_reg_his_add(srng, value);
	} else {
		hal_srng_delay_reg_record_direct_write(srng, false);
		hal_reg_write_enqueue(hal_soc, srng, addr, value);
	}

	hal_record_suspend_write(srng->ring_id, value, srng->wstats.direct);
}
#endif
#endif

#ifdef HAL_SRNG_REG_HIS_DEBUG
inline void hal_free_srng_history(struct hal_soc *hal)
{
	int i;

	for (i = 0; i < HAL_SRNG_ID_MAX; i++)
		qdf_mem_free(hal->srng_list[i].reg_his_ctx);
}

inline bool hal_alloc_srng_history(struct hal_soc *hal)
{
	int i;

	for (i = 0; i < HAL_SRNG_ID_MAX; i++) {
		hal->srng_list[i].reg_his_ctx =
			qdf_mem_malloc(sizeof(struct hal_srng_reg_his_ctx));
		if (!hal->srng_list[i].reg_his_ctx) {
			hal_err("srng_hist alloc failed");
			hal_free_srng_history(hal);
			return false;
		}
	}

	return true;
}
#else
inline void hal_free_srng_history(struct hal_soc *hal)
{
}

inline bool hal_alloc_srng_history(struct hal_soc *hal)
{
	return true;
}
#endif

void *hal_attach(struct hif_opaque_softc *hif_handle, qdf_device_t qdf_dev)
{
	struct hal_soc *hal;
	int i;

	hal = qdf_mem_common_alloc(sizeof(*hal));

	if (!hal) {
		QDF_TRACE(QDF_MODULE_ID_TXRX, QDF_TRACE_LEVEL_ERROR,
			"%s: hal_soc allocation failed", __func__);
		goto fail0;
	}
	hal->hif_handle = hif_handle;
	hal->dev_base_addr = hif_get_dev_ba(hif_handle); /* UMAC */
	hal->dev_base_addr_ce = hif_get_dev_ba_ce(hif_handle); /* CE */
	hal->dev_base_addr_cmem = hif_get_dev_ba_cmem(hif_handle); /* CMEM */
	hal->dev_base_addr_pmm = hif_get_dev_ba_pmm(hif_handle); /* PMM */
	hal->qdf_dev = qdf_dev;
	hal->shadow_rdptr_mem_vaddr = (uint32_t *)qdf_mem_alloc_consistent(
		qdf_dev, qdf_dev->dev, sizeof(*(hal->shadow_rdptr_mem_vaddr)) *
		HAL_SRNG_ID_MAX, &(hal->shadow_rdptr_mem_paddr));
	if (!hal->shadow_rdptr_mem_paddr) {
		QDF_TRACE(QDF_MODULE_ID_TXRX, QDF_TRACE_LEVEL_ERROR,
			"%s: hal->shadow_rdptr_mem_paddr allocation failed",
			__func__);
		goto fail1;
	}
	qdf_mem_zero(hal->shadow_rdptr_mem_vaddr,
		     sizeof(*(hal->shadow_rdptr_mem_vaddr)) * HAL_SRNG_ID_MAX);

	hal->shadow_wrptr_mem_vaddr =
		(uint32_t *)qdf_mem_alloc_consistent(qdf_dev, qdf_dev->dev,
		sizeof(*(hal->shadow_wrptr_mem_vaddr)) * HAL_MAX_LMAC_RINGS,
		&(hal->shadow_wrptr_mem_paddr));
	if (!hal->shadow_wrptr_mem_vaddr) {
		QDF_TRACE(QDF_MODULE_ID_TXRX, QDF_TRACE_LEVEL_ERROR,
			"%s: hal->shadow_wrptr_mem_vaddr allocation failed",
			__func__);
		goto fail2;
	}
	qdf_mem_zero(hal->shadow_wrptr_mem_vaddr,
		sizeof(*(hal->shadow_wrptr_mem_vaddr)) * HAL_MAX_LMAC_RINGS);

	if (!hal_alloc_srng_history(hal))
		goto fail2;

	for (i = 0; i < HAL_SRNG_ID_MAX; i++) {
		hal->srng_list[i].initialized = 0;
		hal->srng_list[i].ring_id = i;
	}

	qdf_spinlock_create(&hal->register_access_lock);
	hal->register_window = 0;
	hal->target_type = hal_get_target_type(hal_soc_to_hal_soc_handle(hal));
	hal->version = hif_get_soc_version(hif_handle);
	hal->ops = qdf_mem_malloc(sizeof(*hal->ops));

	if (!hal->ops) {
		hal_err("unable to allocable memory for HAL ops");
		goto fail3;
	}

	hal_target_based_configure(hal);

	hal_reg_write_fail_history_init(hal);

	qdf_minidump_log(hal, sizeof(*hal), "hal_soc");

	qdf_ssr_driver_dump_register_region("hal_soc", hal, sizeof(*hal));

	qdf_atomic_init(&hal->active_work_cnt);
	if (hal_delayed_reg_write_init(hal) != QDF_STATUS_SUCCESS) {
		hal_err("unable to initialize delayed reg write");
		goto fail4;
	}

	hif_rtpm_register(HIF_RTPM_ID_HAL_REO_CMD, NULL);

	return (void *)hal;
fail4:
	qdf_ssr_driver_dump_unregister_region("hal_soc");
	qdf_minidump_remove(hal, sizeof(*hal), "hal_soc");
	qdf_mem_free(hal->ops);
fail3:
	qdf_mem_free_consistent(qdf_dev, qdf_dev->dev,
				sizeof(*hal->shadow_wrptr_mem_vaddr) *
				HAL_MAX_LMAC_RINGS,
				hal->shadow_wrptr_mem_vaddr,
				hal->shadow_wrptr_mem_paddr, 0);
fail2:
	qdf_mem_free_consistent(qdf_dev, qdf_dev->dev,
		sizeof(*(hal->shadow_rdptr_mem_vaddr)) * HAL_SRNG_ID_MAX,
		hal->shadow_rdptr_mem_vaddr, hal->shadow_rdptr_mem_paddr, 0);
fail1:
	qdf_mem_common_free(hal);
fail0:
	return NULL;
}
qdf_export_symbol(hal_attach);

void hal_get_meminfo(hal_soc_handle_t hal_soc_hdl, struct hal_mem_info *mem)
{
	struct hal_soc *hal = (struct hal_soc *)hal_soc_hdl;
	mem->dev_base_addr = (void *)hal->dev_base_addr;
        mem->shadow_rdptr_mem_vaddr = (void *)hal->shadow_rdptr_mem_vaddr;
	mem->shadow_wrptr_mem_vaddr = (void *)hal->shadow_wrptr_mem_vaddr;
        mem->shadow_rdptr_mem_paddr = (void *)hal->shadow_rdptr_mem_paddr;
	mem->shadow_wrptr_mem_paddr = (void *)hal->shadow_wrptr_mem_paddr;
	hif_read_phy_mem_base((void *)hal->hif_handle,
			      (qdf_dma_addr_t *)&mem->dev_base_paddr);
	mem->lmac_srng_start_id = HAL_SRNG_LMAC1_ID_START;
	return;
}
qdf_export_symbol(hal_get_meminfo);

void hal_detach(void *hal_soc)
{
	struct hal_soc *hal = (struct hal_soc *)hal_soc;

	hif_rtpm_deregister(HIF_RTPM_ID_HAL_REO_CMD);
	hal_delayed_reg_write_deinit(hal);
	hal_reo_shared_qaddr_detach((hal_soc_handle_t)hal);
	qdf_ssr_driver_dump_unregister_region("hal_soc");
	qdf_minidump_remove(hal, sizeof(*hal), "hal_soc");
	qdf_mem_free(hal->ops);

	hal_free_srng_history(hal);
	qdf_mem_free_consistent(hal->qdf_dev, hal->qdf_dev->dev,
		sizeof(*(hal->shadow_rdptr_mem_vaddr)) * HAL_SRNG_ID_MAX,
		hal->shadow_rdptr_mem_vaddr, hal->shadow_rdptr_mem_paddr, 0);
	qdf_mem_free_consistent(hal->qdf_dev, hal->qdf_dev->dev,
		sizeof(*(hal->shadow_wrptr_mem_vaddr)) * HAL_MAX_LMAC_RINGS,
		hal->shadow_wrptr_mem_vaddr, hal->shadow_wrptr_mem_paddr, 0);
	qdf_mem_common_free(hal);

	return;
}
qdf_export_symbol(hal_detach);

#define HAL_CE_CHANNEL_DST_DEST_CTRL_ADDR(x)		((x) + 0x000000b0)
#define HAL_CE_CHANNEL_DST_DEST_CTRL_DEST_MAX_LENGTH_BMSK	0x0000ffff
#define HAL_CE_CHANNEL_DST_DEST_RING_CONSUMER_PREFETCH_TIMER_ADDR(x)	((x) + 0x00000040)
#define HAL_CE_CHANNEL_DST_DEST_RING_CONSUMER_PREFETCH_TIMER_RMSK	0x00000007

/**
 * hal_ce_dst_setup() - Initialize CE destination ring registers
 * @hal: HAL SOC handle
 * @srng: SRNG ring pointer
 * @ring_num: ring number
 */
static inline void hal_ce_dst_setup(struct hal_soc *hal, struct hal_srng *srng,
				    int ring_num)
{
	uint32_t reg_val = 0;
	uint32_t reg_addr;
	struct hal_hw_srng_config *ring_config =
		HAL_SRNG_CONFIG(hal, CE_DST);

	/* set DEST_MAX_LENGTH according to ce assignment */
	reg_addr = HAL_CE_CHANNEL_DST_DEST_CTRL_ADDR(
			ring_config->reg_start[R0_INDEX] +
			(ring_num * ring_config->reg_size[R0_INDEX]));

	reg_val = HAL_REG_READ(hal, reg_addr);
	reg_val &= ~HAL_CE_CHANNEL_DST_DEST_CTRL_DEST_MAX_LENGTH_BMSK;
	reg_val |= srng->u.dst_ring.max_buffer_length &
		HAL_CE_CHANNEL_DST_DEST_CTRL_DEST_MAX_LENGTH_BMSK;
	HAL_REG_WRITE(hal, reg_addr, reg_val);

	if (srng->prefetch_timer) {
		reg_addr = HAL_CE_CHANNEL_DST_DEST_RING_CONSUMER_PREFETCH_TIMER_ADDR(
				ring_config->reg_start[R0_INDEX] +
				(ring_num * ring_config->reg_size[R0_INDEX]));

		reg_val = HAL_REG_READ(hal, reg_addr);
		reg_val &= ~HAL_CE_CHANNEL_DST_DEST_RING_CONSUMER_PREFETCH_TIMER_RMSK;
		reg_val |= srng->prefetch_timer;
		HAL_REG_WRITE(hal, reg_addr, reg_val);
		reg_val = HAL_REG_READ(hal, reg_addr);
	}

}

void hal_reo_read_write_ctrl_ix(hal_soc_handle_t hal_soc_hdl, bool read,
				uint32_t *ix0, uint32_t *ix1,
				uint32_t *ix2, uint32_t *ix3)
{
	uint32_t reg_offset;
	struct hal_soc *hal = (struct hal_soc *)hal_soc_hdl;
	uint32_t reo_reg_base;

	reo_reg_base = hal_get_reo_reg_base_offset(hal_soc_hdl);

	if (read) {
		if (ix0) {
			reg_offset =
				HAL_REO_DESTINATION_RING_CTRL_IX_0_ADDR(
						reo_reg_base);
			*ix0 = HAL_REG_READ(hal, reg_offset);
		}

		if (ix1) {
			reg_offset =
				HAL_REO_DESTINATION_RING_CTRL_IX_1_ADDR(
						reo_reg_base);
			*ix1 = HAL_REG_READ(hal, reg_offset);
		}

		if (ix2) {
			reg_offset =
				HAL_REO_DESTINATION_RING_CTRL_IX_2_ADDR(
						reo_reg_base);
			*ix2 = HAL_REG_READ(hal, reg_offset);
		}

		if (ix3) {
			reg_offset =
				HAL_REO_DESTINATION_RING_CTRL_IX_3_ADDR(
						reo_reg_base);
			*ix3 = HAL_REG_READ(hal, reg_offset);
		}
	} else {
		if (ix0) {
			reg_offset =
				HAL_REO_DESTINATION_RING_CTRL_IX_0_ADDR(
						reo_reg_base);
			HAL_REG_WRITE_CONFIRM_RETRY(hal, reg_offset,
						    *ix0, true);
		}

		if (ix1) {
			reg_offset =
				HAL_REO_DESTINATION_RING_CTRL_IX_1_ADDR(
						reo_reg_base);
			HAL_REG_WRITE_CONFIRM_RETRY(hal, reg_offset,
						    *ix1, true);
		}

		if (ix2) {
			reg_offset =
				HAL_REO_DESTINATION_RING_CTRL_IX_2_ADDR(
						reo_reg_base);
			HAL_REG_WRITE_CONFIRM_RETRY(hal, reg_offset,
						    *ix2, true);
		}

		if (ix3) {
			reg_offset =
				HAL_REO_DESTINATION_RING_CTRL_IX_3_ADDR(
						reo_reg_base);
			HAL_REG_WRITE_CONFIRM_RETRY(hal, reg_offset,
						    *ix3, true);
		}
	}
}

qdf_export_symbol(hal_reo_read_write_ctrl_ix);

void hal_srng_dst_set_hp_paddr_confirm(struct hal_srng *srng, uint64_t paddr)
{
	SRNG_DST_REG_WRITE_CONFIRM(srng, HP_ADDR_LSB, paddr & 0xffffffff);
	SRNG_DST_REG_WRITE_CONFIRM(srng, HP_ADDR_MSB, paddr >> 32);
}

qdf_export_symbol(hal_srng_dst_set_hp_paddr_confirm);

void hal_srng_dst_init_hp(struct hal_soc_handle *hal_soc,
			  struct hal_srng *srng,
			  uint32_t *vaddr)
{
	uint32_t reg_offset;
	struct hal_soc *hal = (struct hal_soc *)hal_soc;

	if (!srng)
		return;

	srng->u.dst_ring.hp_addr = vaddr;
	reg_offset = SRNG_DST_ADDR(srng, HP) - hal->dev_base_addr;
	HAL_REG_WRITE_CONFIRM_RETRY(
		hal, reg_offset, srng->u.dst_ring.cached_hp, true);

	if (vaddr) {
		*srng->u.dst_ring.hp_addr = srng->u.dst_ring.cached_hp;
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_DEBUG,
			  "hp_addr=%pK, cached_hp=%d",
			  (void *)srng->u.dst_ring.hp_addr,
			  srng->u.dst_ring.cached_hp);
	}
}

qdf_export_symbol(hal_srng_dst_init_hp);

void hal_srng_dst_update_hp_addr(struct hal_soc_handle *hal_soc,
				 hal_ring_handle_t hal_ring_hdl)
{
	struct hal_srng *srng = (struct hal_srng *)hal_ring_hdl;
	int32_t hw_hp;
	int32_t hw_tp;

	if (!srng)
		return;

	if (srng->u.dst_ring.hp_addr) {
		hal_get_hw_hptp(hal_soc, hal_ring_hdl, &hw_hp, &hw_tp,
				WBM2SW_RELEASE);
		*srng->u.dst_ring.hp_addr = hw_hp;
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_DEBUG,
			  "hw_hp=%d", hw_hp);
	}
}

qdf_export_symbol(hal_srng_dst_update_hp_addr);

/**
 * hal_srng_hw_init - Private function to initialize SRNG HW
 * @hal: HAL SOC handle
 * @srng: SRNG ring pointer
 * @idle_check: Check if ring is idle
 * @idx: ring index
 */
static inline void hal_srng_hw_init(struct hal_soc *hal,
	struct hal_srng *srng, bool idle_check, uint32_t idx)
{
	if (srng->ring_dir == HAL_SRNG_SRC_RING)
		hal_srng_src_hw_init(hal, srng, idle_check, idx);
	else
		hal_srng_dst_hw_init(hal, srng, idle_check, idx);
}

#ifdef WLAN_FEATURE_NEAR_FULL_IRQ
bool hal_srng_is_near_full_irq_supported(hal_soc_handle_t hal_soc,
					 int ring_type, int ring_num)
{
	struct hal_soc *hal = (struct hal_soc *)hal_soc;
	struct hal_hw_srng_config *ring_config =
		HAL_SRNG_CONFIG(hal, ring_type);

	return ring_config->nf_irq_support;
}

/**
 * hal_srng_set_msi2_params() - Set MSI2 params to SRNG data structure from
 *				ring params
 * @srng: SRNG handle
 * @ring_params: ring params for this SRNG
 *
 * Return: None
 */
static inline void
hal_srng_set_msi2_params(struct hal_srng *srng,
			 struct hal_srng_params *ring_params)
{
	srng->msi2_addr = ring_params->msi2_addr;
	srng->msi2_data = ring_params->msi2_data;
}

/**
 * hal_srng_get_nf_params() - Get the near full MSI2 params from srng
 * @srng: SRNG handle
 * @ring_params: ring params for this SRNG
 *
 * Return: None
 */
static inline void
hal_srng_get_nf_params(struct hal_srng *srng,
		       struct hal_srng_params *ring_params)
{
	ring_params->msi2_addr = srng->msi2_addr;
	ring_params->msi2_data = srng->msi2_data;
}

/**
 * hal_srng_set_nf_thresholds() - Set the near full thresholds in SRNG
 * @srng: SRNG handle where the params are to be set
 * @ring_params: ring params, from where threshold is to be fetched
 *
 * Return: None
 */
static inline void
hal_srng_set_nf_thresholds(struct hal_srng *srng,
			   struct hal_srng_params *ring_params)
{
	srng->u.dst_ring.nf_irq_support = ring_params->nf_irq_support;
	srng->u.dst_ring.high_thresh = ring_params->high_thresh;
}
#else
static inline void
hal_srng_set_msi2_params(struct hal_srng *srng,
			 struct hal_srng_params *ring_params)
{
}

static inline void
hal_srng_get_nf_params(struct hal_srng *srng,
		       struct hal_srng_params *ring_params)
{
}

static inline void
hal_srng_set_nf_thresholds(struct hal_srng *srng,
			   struct hal_srng_params *ring_params)
{
}
#endif

#if defined(CLEAR_SW2TCL_CONSUMED_DESC)
/**
 * hal_srng_last_desc_cleared_init - Initialize SRNG last_desc_cleared ptr
 * @srng: Source ring pointer
 *
 * Return: None
 */
static inline
void hal_srng_last_desc_cleared_init(struct hal_srng *srng)
{
	srng->last_desc_cleared = srng->ring_size - srng->entry_size;
}

#else
static inline
void hal_srng_last_desc_cleared_init(struct hal_srng *srng)
{
}
#endif /* CLEAR_SW2TCL_CONSUMED_DESC */

#ifdef WLAN_DP_SRNG_USAGE_WM_TRACKING
static inline void hal_srng_update_high_wm_thresholds(struct hal_srng *srng)
{
	srng->high_wm.bin_thresh[HAL_SRNG_HIGH_WM_BIN_90_to_100] =
			((srng->num_entries * 90) / 100);
	srng->high_wm.bin_thresh[HAL_SRNG_HIGH_WM_BIN_80_to_90] =
			((srng->num_entries * 80) / 100);
	srng->high_wm.bin_thresh[HAL_SRNG_HIGH_WM_BIN_70_to_80] =
			((srng->num_entries * 70) / 100);
	srng->high_wm.bin_thresh[HAL_SRNG_HIGH_WM_BIN_60_to_70] =
			((srng->num_entries * 60) / 100);
	srng->high_wm.bin_thresh[HAL_SRNG_HIGH_WM_BIN_50_to_60] =
			((srng->num_entries * 50) / 100);
	/* Below 50% threshold is not needed */
	srng->high_wm.bin_thresh[HAL_SRNG_HIGH_WM_BIN_BELOW_50_PERCENT] = 0;

	hal_info("ring_id: %u, wm_thresh- <50:%u, 50-60:%u, 60-70:%u, 70-80:%u, 80-90:%u, 90-100:%u",
		 srng->ring_id,
		 srng->high_wm.bin_thresh[HAL_SRNG_HIGH_WM_BIN_BELOW_50_PERCENT],
		 srng->high_wm.bin_thresh[HAL_SRNG_HIGH_WM_BIN_50_to_60],
		 srng->high_wm.bin_thresh[HAL_SRNG_HIGH_WM_BIN_60_to_70],
		 srng->high_wm.bin_thresh[HAL_SRNG_HIGH_WM_BIN_70_to_80],
		 srng->high_wm.bin_thresh[HAL_SRNG_HIGH_WM_BIN_80_to_90],
		 srng->high_wm.bin_thresh[HAL_SRNG_HIGH_WM_BIN_90_to_100]);
}
#else
static inline void hal_srng_update_high_wm_thresholds(struct hal_srng *srng)
{
}
#endif

void *hal_srng_setup_idx(void *hal_soc, int ring_type, int ring_num, int mac_id,
			 struct hal_srng_params *ring_params, bool idle_check,
			 uint32_t idx)
{
	int ring_id;
	struct hal_soc *hal = (struct hal_soc *)hal_soc;
	hal_soc_handle_t hal_hdl = (hal_soc_handle_t)hal;
	struct hal_srng *srng;
	struct hal_hw_srng_config *ring_config =
		HAL_SRNG_CONFIG(hal, ring_type);
	void *dev_base_addr;
	int i;

	ring_id = hal_get_srng_ring_id(hal_soc, ring_type, ring_num, mac_id);
	if (ring_id < 0)
		return NULL;

	hal_verbose_debug("mac_id %d ring_id %d", mac_id, ring_id);

	srng = hal_get_srng(hal_soc, ring_id);

	if (srng->initialized) {
		hal_verbose_debug("Ring (ring_type, ring_num) already initialized");
		return NULL;
	}

	hal_srng_reg_his_init(srng);
	dev_base_addr = hal->dev_base_addr;
	srng->ring_id = ring_id;
	srng->ring_type = ring_type;
	srng->ring_dir = ring_config->ring_dir;
	srng->ring_base_paddr = ring_params->ring_base_paddr;
	srng->ring_base_vaddr = ring_params->ring_base_vaddr;
	srng->entry_size = ring_config->entry_size;
	srng->num_entries = ring_params->num_entries;
	srng->ring_size = srng->num_entries * srng->entry_size;
	srng->ring_size_mask = srng->ring_size - 1;
	srng->ring_vaddr_end = srng->ring_base_vaddr + srng->ring_size;
	srng->msi_addr = ring_params->msi_addr;
	srng->msi_data = ring_params->msi_data;
	srng->intr_timer_thres_us = ring_params->intr_timer_thres_us;
	srng->intr_batch_cntr_thres_entries =
		ring_params->intr_batch_cntr_thres_entries;
	srng->pointer_timer_threshold =
		ring_params->pointer_timer_threshold;
	srng->pointer_num_threshold =
		ring_params->pointer_num_threshold;

	if (!idle_check)
		srng->prefetch_timer = ring_params->prefetch_timer;
	srng->hal_soc = hal_soc;
	hal_srng_set_msi2_params(srng, ring_params);
	hal_srng_update_high_wm_thresholds(srng);

	for (i = 0 ; i < MAX_SRNG_REG_GROUPS; i++) {
		srng->hwreg_base[i] = dev_base_addr + ring_config->reg_start[i]
			+ (ring_num * ring_config->reg_size[i]);
	}

	/* Zero out the entire ring memory */
	qdf_mem_zero(srng->ring_base_vaddr, (srng->entry_size *
		srng->num_entries) << 2);

	srng->flags = ring_params->flags;

	/* For cached descriptors flush and invalidate the memory*/
	if (srng->flags & HAL_SRNG_CACHED_DESC) {
		qdf_nbuf_dma_clean_range(
				srng->ring_base_vaddr,
				srng->ring_base_vaddr +
				((srng->entry_size * srng->num_entries)));
		qdf_nbuf_dma_inv_range(
				srng->ring_base_vaddr,
				srng->ring_base_vaddr +
				((srng->entry_size * srng->num_entries)));
	}
#ifdef BIG_ENDIAN_HOST
		/* TODO: See if we should we get these flags from caller */
	srng->flags |= HAL_SRNG_DATA_TLV_SWAP;
	srng->flags |= HAL_SRNG_MSI_SWAP;
	srng->flags |= HAL_SRNG_RING_PTR_SWAP;
#endif

	hal_srng_last_desc_cleared_init(srng);

	if (srng->ring_dir == HAL_SRNG_SRC_RING) {
		srng->u.src_ring.hp = 0;
		srng->u.src_ring.reap_hp = srng->ring_size -
			srng->entry_size;
		srng->u.src_ring.tp_addr =
			&(hal->shadow_rdptr_mem_vaddr[ring_id]);
		srng->u.src_ring.low_threshold =
			ring_params->low_threshold * srng->entry_size;

		if (srng->u.src_ring.tp_addr)
			qdf_mem_zero(srng->u.src_ring.tp_addr,
				     sizeof(*hal->shadow_rdptr_mem_vaddr));

		if (ring_config->lmac_ring) {
			/* For LMAC rings, head pointer updates will be done
			 * through FW by writing to a shared memory location
			 */
			srng->u.src_ring.hp_addr =
				&(hal->shadow_wrptr_mem_vaddr[ring_id -
					HAL_SRNG_LMAC1_ID_START]);
			srng->flags |= HAL_SRNG_LMAC_RING;

			if (srng->u.src_ring.hp_addr)
				qdf_mem_zero(srng->u.src_ring.hp_addr,
					sizeof(*hal->shadow_wrptr_mem_vaddr));

		} else if (ignore_shadow || (srng->u.src_ring.hp_addr == 0)) {
			srng->u.src_ring.hp_addr =
				hal_get_window_address(hal,
						SRNG_SRC_ADDR(srng, HP));

			if (CHECK_SHADOW_REGISTERS) {
				QDF_TRACE(QDF_MODULE_ID_TXRX,
				    QDF_TRACE_LEVEL_ERROR,
				    "%s: Ring (%d, %d) missing shadow config",
				    __func__, ring_type, ring_num);
			}
		} else {
			hal_validate_shadow_register(hal,
						     SRNG_SRC_ADDR(srng, HP),
						     srng->u.src_ring.hp_addr);
		}
	} else {
		/* During initialization loop count in all the descriptors
		 * will be set to zero, and HW will set it to 1 on completing
		 * descriptor update in first loop, and increments it by 1 on
		 * subsequent loops (loop count wraps around after reaching
		 * 0xffff). The 'loop_cnt' in SW ring state is the expected
		 * loop count in descriptors updated by HW (to be processed
		 * by SW).
		 */
		hal_srng_set_nf_thresholds(srng, ring_params);
		srng->u.dst_ring.loop_cnt = 1;
		srng->u.dst_ring.tp = 0;
		srng->u.dst_ring.hp_addr =
			&(hal->shadow_rdptr_mem_vaddr[ring_id]);

		if (srng->u.dst_ring.hp_addr)
			qdf_mem_zero(srng->u.dst_ring.hp_addr,
				     sizeof(*hal->shadow_rdptr_mem_vaddr));

		if (ring_config->lmac_ring) {
			/* For LMAC rings, tail pointer updates will be done
			 * through FW by writing to a shared memory location
			 */
			srng->u.dst_ring.tp_addr =
				&(hal->shadow_wrptr_mem_vaddr[ring_id -
				HAL_SRNG_LMAC1_ID_START]);
			srng->flags |= HAL_SRNG_LMAC_RING;

			if (srng->u.dst_ring.tp_addr)
				qdf_mem_zero(srng->u.dst_ring.tp_addr,
					sizeof(*hal->shadow_wrptr_mem_vaddr));

		} else if (ignore_shadow || srng->u.dst_ring.tp_addr == 0) {
			srng->u.dst_ring.tp_addr =
				hal_get_window_address(hal,
						SRNG_DST_ADDR(srng, TP));

			if (CHECK_SHADOW_REGISTERS) {
				QDF_TRACE(QDF_MODULE_ID_TXRX,
				    QDF_TRACE_LEVEL_ERROR,
				    "%s: Ring (%d, %d) missing shadow config",
				    __func__, ring_type, ring_num);
			}
		} else {
			hal_validate_shadow_register(hal,
						     SRNG_DST_ADDR(srng, TP),
						     srng->u.dst_ring.tp_addr);
		}
	}

	if (!(ring_config->lmac_ring)) {
		/*
		 * UMAC reset has idle check enabled.
		 * During UMAC reset Tx ring halt is set
		 * by Wi-Fi FW during pre-reset stage,
		 * avoid Tx ring halt again.
		 */
		if (idle_check && idx) {
			if (!hal->ops->hal_tx_ring_halt_get(hal_hdl)) {
				qdf_print("\nTx ring halt not set:Ring(%d, %d)",
					  ring_type, ring_num);
				qdf_assert_always(0);
			}
			hal_srng_hw_init(hal, srng, idle_check, idx);
			goto ce_setup;
		}

		if (idx) {
			hal->ops->hal_tx_ring_halt_set(hal_hdl);
			do {
				hal_info("Waiting for ring reset");
			} while (!(hal->ops->hal_tx_ring_halt_poll(hal_hdl)));
		}
		hal_srng_hw_init(hal, srng, idle_check, idx);

		if (idx) {
			hal->ops->hal_tx_ring_halt_reset(hal_hdl);
		}

ce_setup:
		if (ring_type == CE_DST) {
			srng->u.dst_ring.max_buffer_length = ring_params->max_buffer_length;
			hal_ce_dst_setup(hal, srng, ring_num);
		}
	}

	SRNG_LOCK_INIT(&srng->lock);

	srng->srng_event = 0;

	srng->initialized = true;

	return (void *)srng;
}
qdf_export_symbol(hal_srng_setup_idx);

/**
 * hal_srng_setup - Initialize HW SRNG ring.
 * @hal_soc: Opaque HAL SOC handle
 * @ring_type: one of the types from hal_ring_type
 * @ring_num: Ring number if there are multiple rings of same type (staring
 * from 0)
 * @mac_id: valid MAC Id should be passed if ring type is one of lmac rings
 * @ring_params: SRNG ring params in hal_srng_params structure.
 * @idle_check: Check if ring is idle
 *
 * Callers are expected to allocate contiguous ring memory of size
 * 'num_entries * entry_size' bytes and pass the physical and virtual base
 * addresses through 'ring_base_paddr' and 'ring_base_vaddr' in
 * hal_srng_params structure. Ring base address should be 8 byte aligned
 * and size of each ring entry should be queried using the API
 * hal_srng_get_entrysize
 *
 * Return: Opaque pointer to ring on success
 *		 NULL on failure (if given ring is not available)
 */
void *hal_srng_setup(void *hal_soc, int ring_type, int ring_num,
		     int mac_id, struct hal_srng_params *ring_params,
		     bool idle_check)
{
	return hal_srng_setup_idx(hal_soc, ring_type, ring_num, mac_id,
				  ring_params, idle_check, 0);
}
qdf_export_symbol(hal_srng_setup);

void hal_srng_cleanup(void *hal_soc, hal_ring_handle_t hal_ring_hdl,
		      bool umac_reset_inprogress)
{
	struct hal_srng *srng = (struct hal_srng *)hal_ring_hdl;
	SRNG_LOCK_DESTROY(&srng->lock);
	srng->initialized = 0;
	if (umac_reset_inprogress)
		hal_srng_hw_disable(hal_soc, srng);
}
qdf_export_symbol(hal_srng_cleanup);

uint32_t hal_srng_get_entrysize(void *hal_soc, int ring_type)
{
	struct hal_soc *hal = (struct hal_soc *)hal_soc;
	struct hal_hw_srng_config *ring_config =
		HAL_SRNG_CONFIG(hal, ring_type);
	return ring_config->entry_size << 2;
}
qdf_export_symbol(hal_srng_get_entrysize);

uint32_t hal_srng_max_entries(void *hal_soc, int ring_type)
{
	struct hal_soc *hal = (struct hal_soc *)hal_soc;
	struct hal_hw_srng_config *ring_config =
		HAL_SRNG_CONFIG(hal, ring_type);

	return ring_config->max_size / ring_config->entry_size;
}
qdf_export_symbol(hal_srng_max_entries);

enum hal_srng_dir hal_srng_get_dir(void *hal_soc, int ring_type)
{
	struct hal_soc *hal = (struct hal_soc *)hal_soc;
	struct hal_hw_srng_config *ring_config =
		HAL_SRNG_CONFIG(hal, ring_type);

	return ring_config->ring_dir;
}

void hal_srng_dump(struct hal_srng *srng)
{
	if (srng->ring_dir == HAL_SRNG_SRC_RING) {
		hal_debug("=== SRC RING %d ===", srng->ring_id);
		hal_debug("hp %u, reap_hp %u, tp %u, cached tp %u",
			  srng->u.src_ring.hp,
			  srng->u.src_ring.reap_hp,
			  *srng->u.src_ring.tp_addr,
			  srng->u.src_ring.cached_tp);
	} else {
		hal_debug("=== DST RING %d ===", srng->ring_id);
		hal_debug("tp %u, hp %u, cached tp %u, loop_cnt %u",
			  srng->u.dst_ring.tp,
			  *srng->u.dst_ring.hp_addr,
			  srng->u.dst_ring.cached_hp,
			  srng->u.dst_ring.loop_cnt);
	}
}

void hal_get_srng_params(hal_soc_handle_t hal_soc_hdl,
			 hal_ring_handle_t hal_ring_hdl,
			 struct hal_srng_params *ring_params)
{
	struct hal_srng *srng = (struct hal_srng *)hal_ring_hdl;
	int i =0;
	ring_params->ring_id = srng->ring_id;
	ring_params->ring_dir = srng->ring_dir;
	ring_params->entry_size = srng->entry_size;

	ring_params->ring_base_paddr = srng->ring_base_paddr;
	ring_params->ring_base_vaddr = srng->ring_base_vaddr;
	ring_params->num_entries = srng->num_entries;
	ring_params->msi_addr = srng->msi_addr;
	ring_params->msi_data = srng->msi_data;
	ring_params->intr_timer_thres_us = srng->intr_timer_thres_us;
	ring_params->intr_batch_cntr_thres_entries =
		srng->intr_batch_cntr_thres_entries;
	ring_params->low_threshold = srng->u.src_ring.low_threshold;
	ring_params->flags = srng->flags;
	ring_params->ring_id = srng->ring_id;
	for (i = 0 ; i < MAX_SRNG_REG_GROUPS; i++)
		ring_params->hwreg_base[i] = srng->hwreg_base[i];

	hal_srng_get_nf_params(srng, ring_params);
}
qdf_export_symbol(hal_get_srng_params);

void hal_set_low_threshold(hal_ring_handle_t hal_ring_hdl,
				 uint32_t low_threshold)
{
	struct hal_srng *srng = (struct hal_srng *)hal_ring_hdl;
	srng->u.src_ring.low_threshold = low_threshold * srng->entry_size;
}
qdf_export_symbol(hal_set_low_threshold);

#ifdef FEATURE_RUNTIME_PM
void
hal_srng_rtpm_access_end(hal_soc_handle_t hal_soc_hdl,
			 hal_ring_handle_t hal_ring_hdl,
			 uint32_t rtpm_id)
{
	struct hal_soc *hal_soc = (struct hal_soc *)hal_soc_hdl;

	if (qdf_unlikely(!hal_ring_hdl)) {
		qdf_print("Error: Invalid hal_ring\n");
		return;
	}

	if (hif_rtpm_get(HIF_RTPM_GET_ASYNC, rtpm_id) == 0) {
		if (hif_system_pm_state_check(hal_soc->hif_handle)) {
			hal_srng_access_end_reap(hal_soc_hdl, hal_ring_hdl);
			hal_srng_set_event(hal_ring_hdl, HAL_SRNG_FLUSH_EVENT);
			hal_srng_inc_flush_cnt(hal_ring_hdl);
		} else {
			hal_srng_access_end(hal_soc_hdl, hal_ring_hdl);
		}

		hif_rtpm_put(HIF_RTPM_PUT_ASYNC, rtpm_id);
	} else {
		hal_srng_access_end_reap(hal_soc_hdl, hal_ring_hdl);
		hal_srng_set_event(hal_ring_hdl, HAL_SRNG_FLUSH_EVENT);
		hal_srng_inc_flush_cnt(hal_ring_hdl);
	}
}

qdf_export_symbol(hal_srng_rtpm_access_end);
#endif /* FEATURE_RUNTIME_PM */

#ifdef FORCE_WAKE
void hal_set_init_phase(hal_soc_handle_t soc, bool init_phase)
{
	struct hal_soc *hal_soc = (struct hal_soc *)soc;
	hal_soc->init_phase = init_phase;
}
#endif /* FORCE_WAKE */
