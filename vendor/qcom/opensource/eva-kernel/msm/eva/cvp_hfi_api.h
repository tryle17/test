/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __CVP_HFI_API_H__
#define __CVP_HFI_API_H__

#include <linux/log2.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/hash.h>
#include "msm_cvp_core.h"
#include "msm_cvp_resources.h"
#include "cvp_hfi_helper.h"

#define CONTAINS(__a, __sz, __t) (\
	(__t >= __a) && \
	(__t < __a + __sz) \
)

#define OVERLAPS(__t, __tsz, __a, __asz) (\
	(__t <= __a) && \
	(__t + __tsz >= __a + __asz) \
)

#define CVP_VERSION_LENGTH 128

/* 16 encoder and 16 decoder sessions */
#define CVP_MAX_SESSIONS	32

#define HFI_VERSION_MAJOR_MASK 0xFF000000
#define HFI_VERSION_MAJOR_SHFIT 24
#define HFI_VERSION_MINOR_MASK 0x00FFFFE0
#define HFI_VERSION_MINOR_SHIFT 5
#define HFI_VERSION_BRANCH_MASK 0x0000001F
#define HFI_VERSION_BRANCH_SHIFT 0

enum cvp_status {
	CVP_ERR_NONE = 0x0,
	CVP_ERR_FAIL = 0x80000000,
	CVP_ERR_ALLOC_FAIL,
	CVP_ERR_ILLEGAL_OP,
	CVP_ERR_BAD_PARAM,
	CVP_ERR_BAD_HANDLE,
	CVP_ERR_NOT_SUPPORTED,
	CVP_ERR_BAD_STATE,
	CVP_ERR_MAX_CLIENTS,
	CVP_ERR_IFRAME_EXPECTED,
	CVP_ERR_HW_FATAL,
	CVP_ERR_BITSTREAM_ERR,
	CVP_ERR_INDEX_NOMORE,
	CVP_ERR_SEQHDR_PARSE_FAIL,
	CVP_ERR_INSUFFICIENT_BUFFER,
	CVP_ERR_BAD_POWER_STATE,
	CVP_ERR_NO_VALID_SESSION,
	CVP_ERR_TIMEOUT,
	CVP_ERR_CMDQFULL,
	CVP_ERR_START_CODE_NOT_FOUND,
	CVP_ERR_NOC_ERROR,
	CVP_ERR_CLIENT_PRESENT = 0x90000001,
	CVP_ERR_CLIENT_FATAL,
	CVP_ERR_CMD_QUEUE_FULL,
	CVP_ERR_UNUSED = 0x10000000
};

enum hal_property {
	HAL_UNUSED_PROPERTY = 0xFFFFFFFF,
};

enum hal_ssr_trigger_type {
	SSR_ERR_FATAL = 1,   /* FW sends SYS_ERROR event to EVA_KMD */
	SSR_SW_DIV_BY_ZERO,
	SSR_HW_WDOG_IRQ,     /* FW will go in while loop to miss WDOG timer tapping */
	SSR_SESSION_ABORT,
	SSR_SESSION_ERROR,   /* No FW involvement, EVA KMD will simulate */
	SSR_FW_SMMU_FAULT,   /* FW writes 0xFF000000 while programming Xtensa */
	SSR_CORE_SMMU_FAULT, /* No FW involvement, EVA KMD will simulate */
	SSR_SESSION_TIMEOUT
};

enum hal_intra_refresh_mode {
	HAL_INTRA_REFRESH_NONE,
	HAL_INTRA_REFRESH_CYCLIC,
	HAL_INTRA_REFRESH_RANDOM,
	HAL_UNUSED_INTRA = 0x10000000,
};

enum cvp_resource_id {
	CVP_RESOURCE_NONE,
	CVP_RESOURCE_SYSCACHE,
	CVP_UNUSED_RESOURCE = 0x10000000,
};

struct cvp_resource_hdr {
	enum cvp_resource_id resource_id;
	void *resource_handle;
};

struct cvp_hal_fw_info {
	char version[CVP_VERSION_LENGTH];
	phys_addr_t base_addr;
	int register_base;
	int register_size;
	int irq;
};

enum hal_event_type {
	HAL_EVENT_SEQ_CHANGED_SUFFICIENT_RESOURCES,
	HAL_EVENT_SEQ_CHANGED_INSUFFICIENT_RESOURCES,
	HAL_EVENT_RELEASE_BUFFER_REFERENCE,
	HAL_UNUSED_SEQCHG = 0x10000000,
};

/* HAL Response */
#define IS_HAL_SYS_CMD(cmd) ((cmd) >= HAL_SYS_INIT_DONE && \
		(cmd) <= HAL_SYS_ERROR)
#define IS_HAL_SESSION_CMD(cmd) ((cmd) >= HAL_SESSION_EVENT_CHANGE && \
		(cmd) <= HAL_SESSION_ERROR)
enum hal_command_response {
	HAL_NO_RESP,
	HAL_SYS_INIT_DONE,
	HAL_SYS_SET_RESOURCE_DONE,
	HAL_SYS_RELEASE_RESOURCE_DONE,
	HAL_SYS_PING_ACK_DONE,
	HAL_SYS_PC_PREP_DONE,
	HAL_SYS_IDLE,
	HAL_SYS_DEBUG,
	HAL_SYS_WATCHDOG_TIMEOUT,
	HAL_SYS_ERROR,
	/* SESSION COMMANDS_DONE */
	HAL_SESSION_EVENT_CHANGE,
	HAL_SESSION_INIT_DONE,
	HAL_SESSION_END_DONE,
	HAL_SESSION_SET_BUFFER_DONE,
	HAL_SESSION_ABORT_DONE,
	HAL_SESSION_START_DONE,
	HAL_SESSION_STOP_DONE,
	HAL_SESSION_CVP_OPERATION_CONFIG,
	HAL_SESSION_FLUSH_DONE,
	HAL_SESSION_SUSPEND_DONE,
	HAL_SESSION_RESUME_DONE,
	HAL_SESSION_SET_PROP_DONE,
	HAL_SESSION_GET_PROP_DONE,
	HAL_SESSION_RELEASE_BUFFER_DONE,
	HAL_SESSION_REGISTER_BUFFER_DONE,
	HAL_SESSION_UNREGISTER_BUFFER_DONE,
	HAL_SESSION_RELEASE_RESOURCE_DONE,
	HAL_SESSION_PROPERTY_INFO,
	HAL_SESSION_DUMP_NOTIFY,
	HAL_SESSION_ERROR,
	HAL_RESPONSE_UNUSED = 0x10000000,
};

struct msm_cvp_capability {
	u32 reserved[183];
};

struct cvp_hal_sys_init_done {
	u32 dec_codec_supported;
	u32 enc_codec_supported;
	u32 codec_count;
	struct msm_cvp_capability *capabilities;
	u32 max_sessions_supported;
};

struct cvp_hal_session_init_done {
	struct msm_cvp_capability capability;
};

struct msm_cvp_cb_cmd_done {
	u32 device_id;
	void *session_id;
	enum cvp_status status;
	u32 size;
	union {
		struct cvp_hfi_msg_session_hdr msg_hdr;
		struct cvp_resource_hdr resource_hdr;
		struct cvp_hal_sys_init_done sys_init_done;
		struct cvp_hal_session_init_done session_init_done;
		u32 buffer_addr;
	} data;
};

struct msm_cvp_cb_data_done {
	u32 device_id;
	void *session_id;
	enum cvp_status status;
	u32 size;
	u32 client_data;
};

struct msm_cvp_cb_info {
	enum hal_command_response response_type;
	union {
		struct msm_cvp_cb_cmd_done cmd;
		struct msm_cvp_cb_data_done data;
	} response;
};

enum msm_cvp_hfi_type {
	CVP_HFI_IRIS,
};

enum msm_cvp_thermal_level {
	CVP_THERMAL_NORMAL = 0,
	CVP_THERMAL_LOW,
	CVP_THERMAL_HIGH,
	CVP_THERMAL_CRITICAL
};

struct msm_cvp_gov_data {
	struct cvp_bus_vote_data *data;
	u32 data_count;
};

enum msm_cvp_power_mode {
	CVP_POWER_NORMAL = 0,
	CVP_POWER_LOW,
	CVP_POWER_TURBO
};

struct cvp_bus_vote_data {
	u32 domain;
	u32 ddr_bw;
	u32 sys_cache_bw;
	enum msm_cvp_power_mode power_mode;
	bool use_sys_cache;
};

struct cvp_hal_cmd_sys_get_property_packet {
	u32 size;
	u32 packet_type;
	u32 num_properties;
	u32 rg_property_data[1];
};

#define call_hfi_op(q, op, args...)			\
	(((q) && (q)->op) ? ((q)->op(args)) : 0)

#define PKT_NAME_LEN	24
#define MAX_PKT_IDX	0x200

struct msm_cvp_hfi_defs {
	unsigned int size;
	unsigned int type;
	bool is_config_pkt;
	bool checksum_enabled;
	enum hal_command_response resp;
	char name[PKT_NAME_LEN];
	bool force_kernel_fence;
};

struct cvp_hfi_ops {
	void *hfi_device_data;
	/*Add function pointers for all the hfi functions below*/
	int (*core_init)(void *device);
	int (*core_release)(void *device);
	int (*core_trigger_ssr)(void *device, enum hal_ssr_trigger_type);
	int (*session_init)(void *device, void *session_id, void **new_session);
	int (*session_end)(void *session);
	int (*session_start)(void *session, u64 ktid);
	int (*session_stop)(void *session, u64 ktid);
	int (*session_abort)(void *session);
	int (*session_set_buffers)(void *sess, u32 iova, u32 size);
	int (*session_release_buffers)(void *sess);
	int (*session_send)(void *sess, struct eva_kmd_hfi_packet *in_pkt);
	int (*session_flush)(void *sess, u64 ktid);
	int (*scale_clocks)(void *dev, u32 freq);
	int (*vote_bus)(void *dev, struct bus_info *bus, unsigned long bw);
	int (*get_fw_info)(void *dev, struct cvp_hal_fw_info *fw_info);
	int (*session_clean)(void *sess);
	int (*get_core_capabilities)(void *dev);
	int (*suspend)(void *dev);
	int (*resume)(void *dev);
	int (*flush_debug_queue)(void *dev);
	int (*noc_error_info)(void *dev);
	int (*validate_session)(void *sess, const char *func);
	int (*pm_qos_update)(void *device);
	int (*debug_hook)(void *device);
};

typedef void (*hfi_cmd_response_callback) (enum hal_command_response cmd,
			void *data);
typedef void (*msm_cvp_callback) (enum hal_command_response response,
			void *callback);
struct msm_cvp_fw {
	int cookie;
};

int cvp_hfi_process_msg_packet(u32 device_id,
	void *msg_hdr, struct msm_cvp_cb_info *info);

enum cvp_status cvp_hfi_process_sys_init_done_prop_read(
	struct cvp_hfi_msg_sys_init_done_packet *pkt,
	struct cvp_hal_sys_init_done *sys_init_done);

enum cvp_status hfi_process_session_init_done_prop_read(
	struct cvp_hfi_msg_sys_session_init_done_packet *pkt,
	struct cvp_hal_session_init_done *session_init_done);

struct cvp_hfi_ops *cvp_hfi_initialize(enum msm_cvp_hfi_type hfi_type,
		struct msm_cvp_platform_resources *res,
		hfi_cmd_response_callback callback);
void cvp_hfi_deinitialize(enum msm_cvp_hfi_type hfi_type,
			struct cvp_hfi_ops *hdev);

int get_pkt_index(struct cvp_hal_session_cmd_pkt *hdr);
int get_pkt_fenceoverride(struct cvp_hal_session_cmd_pkt* hdr);
int get_pkt_index_from_type(u32 pkt_type);
int get_hfi_version(void);
unsigned int get_msg_size(struct cvp_hfi_msg_session_hdr *hdr);
unsigned int get_msg_session_id(void *msg);
unsigned int get_msg_errorcode(void *msg);
int get_msg_opconfigs(void *msg, unsigned int *session_id,
		unsigned int *error_type, unsigned int *config_id);
extern const struct msm_cvp_hfi_defs cvp_hfi_defs[MAX_PKT_IDX];
void print_hfi_queue_info(struct cvp_hfi_ops *hdev);
#endif /*__CVP_HFI_API_H__ */
