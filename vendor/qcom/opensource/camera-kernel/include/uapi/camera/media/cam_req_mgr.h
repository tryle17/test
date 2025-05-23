/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __UAPI_LINUX_CAM_REQ_MGR_H
#define __UAPI_LINUX_CAM_REQ_MGR_H

#include <linux/videodev2.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/media.h>
#include <media/cam_defs.h>

#define CAM_REQ_MGR_VNODE_NAME "cam-req-mgr-devnode"

#define CAM_DEVICE_TYPE_BASE      (MEDIA_ENT_F_OLD_BASE)
#define CAM_VNODE_DEVICE_TYPE     (CAM_DEVICE_TYPE_BASE)
#define CAM_SENSOR_DEVICE_TYPE    (CAM_DEVICE_TYPE_BASE + 1)
#define CAM_IFE_DEVICE_TYPE       (CAM_DEVICE_TYPE_BASE + 2)
#define CAM_ICP_DEVICE_TYPE       (CAM_DEVICE_TYPE_BASE + 3)
#define CAM_LRME_DEVICE_TYPE      (CAM_DEVICE_TYPE_BASE + 4)
#define CAM_JPEG_DEVICE_TYPE      (CAM_DEVICE_TYPE_BASE + 5)
#define CAM_FD_DEVICE_TYPE        (CAM_DEVICE_TYPE_BASE + 6)
#define CAM_CPAS_DEVICE_TYPE      (CAM_DEVICE_TYPE_BASE + 7)
#define CAM_CSIPHY_DEVICE_TYPE    (CAM_DEVICE_TYPE_BASE + 8)
#define CAM_ACTUATOR_DEVICE_TYPE  (CAM_DEVICE_TYPE_BASE + 9)
#define CAM_CCI_DEVICE_TYPE       (CAM_DEVICE_TYPE_BASE + 10)
#define CAM_FLASH_DEVICE_TYPE     (CAM_DEVICE_TYPE_BASE + 11)
#define CAM_EEPROM_DEVICE_TYPE    (CAM_DEVICE_TYPE_BASE + 12)
#define CAM_OIS_DEVICE_TYPE       (CAM_DEVICE_TYPE_BASE + 13)
#define CAM_CUSTOM_DEVICE_TYPE    (CAM_DEVICE_TYPE_BASE + 14)
#define CAM_OPE_DEVICE_TYPE       (CAM_DEVICE_TYPE_BASE + 15)
#define CAM_TFE_DEVICE_TYPE       (CAM_DEVICE_TYPE_BASE + 16)
#define CAM_CRE_DEVICE_TYPE       (CAM_DEVICE_TYPE_BASE + 17)
#define CAM_TPG_DEVICE_TYPE       (CAM_DEVICE_TYPE_BASE + 18)
#define CAM_TFE_MC_DEVICE_TYPE    (CAM_DEVICE_TYPE_BASE + 19)

/* cam_req_mgr hdl info */
#define CAM_REQ_MGR_HDL_IDX_POS           8
#define CAM_REQ_MGR_HDL_IDX_MASK          ((1 << CAM_REQ_MGR_HDL_IDX_POS) - 1)
#define CAM_REQ_MGR_GET_HDL_IDX(hdl)      (hdl & CAM_REQ_MGR_HDL_IDX_MASK)

/**
 * Max handles supported by cam_req_mgr
 * It includes both session and device handles
 */
#define CAM_REQ_MGR_MAX_HANDLES           64
#define CAM_REQ_MGR_MAX_HANDLES_V2        256
#define MAX_LINKS_PER_SESSION             2

/* Interval for cam_info_rate_limit_custom() */
#define CAM_RATE_LIMIT_INTERVAL_5SEC 5

/* V4L event type which user space will subscribe to */
#define V4L_EVENT_CAM_REQ_MGR_EVENT       (V4L2_EVENT_PRIVATE_START + 0)

/* Specific event ids to get notified in user space */
#define V4L_EVENT_CAM_REQ_MGR_SOF                                       0
#define V4L_EVENT_CAM_REQ_MGR_ERROR                                     1
#define V4L_EVENT_CAM_REQ_MGR_SOF_BOOT_TS                               2
#define V4L_EVENT_CAM_REQ_MGR_CUSTOM_EVT                                3
#define V4L_EVENT_CAM_REQ_MGR_NODE_EVENT                                4
#define V4L_EVENT_CAM_REQ_MGR_SOF_UNIFIED_TS                            5
#define V4L_EVENT_CAM_REQ_MGR_PF_ERROR                                  6

/* SOF Event status */
#define CAM_REQ_MGR_SOF_EVENT_SUCCESS           0
#define CAM_REQ_MGR_SOF_EVENT_ERROR             1

/* Link control operations */
#define CAM_REQ_MGR_LINK_ACTIVATE               0
#define CAM_REQ_MGR_LINK_DEACTIVATE             1

/* DMA buffer name length */
#define CAM_DMA_BUF_NAME_LEN                    128
#define CAM_REQ_MGR_ALLOC_BUF_WITH_NAME         1

/**
 * Request Manager : flush_type
 * @CAM_REQ_MGR_FLUSH_TYPE_ALL: Req mgr will remove all the pending
 * requests from input/processing queue.
 * @CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ: Req mgr will remove only particular
 * request id from input/processing queue.
 * @CAM_REQ_MGR_FLUSH_TYPE_MAX: Max number of the flush type
 * @opcode: CAM_REQ_MGR_FLUSH_REQ
 */
#define CAM_REQ_MGR_FLUSH_TYPE_ALL          0
#define CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ   1
#define CAM_REQ_MGR_FLUSH_TYPE_MAX          2

/**
 * Request Manager : Sync Mode type
 * @CAM_REQ_MGR_SYNC_MODE_NO_SYNC: Req mgr will apply non-sync mode for this
 * request.
 * @CAM_REQ_MGR_SYNC_MODE_SYNC: Req mgr will apply sync mode for this request.
 */
#define CAM_REQ_MGR_SYNC_MODE_NO_SYNC   0
#define CAM_REQ_MGR_SYNC_MODE_SYNC      1

/**
 * Functional capabilities for flush
 * "reserved" field in struct cam_req_mgr_flush_info to be set
 * to enable/disable these functionalities
 */
#define CAM_REQ_MGR_ENABLE_SENSOR_STANDBY BIT(0)

/**
 * struct cam_req_mgr_event_data
 * @session_hdl: session handle
 * @link_hdl: link handle
 * @frame_id: frame id
 * @reserved: reserved for 64 bit aligngment
 * @req_id: request id
 * @tv_sec: timestamp in seconds
 * @tv_usec: timestamp in micro seconds
 */
struct cam_req_mgr_event_data {
	__s32   session_hdl;
	__s32   link_hdl;
	__s32   frame_id;
	__s32   reserved;
	__s64   req_id;
	__u64  tv_sec;
	__u64  tv_usec;
};

/**
 * struct cam_req_mgr_session_info
 * @session_hdl: In/Output param - session_handle
 * @opcode1: CAM_REQ_MGR_CREATE_SESSION
 * @opcode2: CAM_REQ_MGR_DESTROY_SESSION
 */
struct cam_req_mgr_session_info {
	__s32 session_hdl;
	__s32 reserved;
};

/**
 * struct cam_req_mgr_link_info
 * @session_hdl: Input param - Identifier for CSL session
 * @num_devices: Input Param - Num of devices to be linked
 * @dev_hdls: Input param - List of device handles to be linked
 * @link_hdl: Output Param -Identifier for link
 * @opcode: CAM_REQ_MGR_LINK
 */
struct cam_req_mgr_link_info {
	__s32 session_hdl;
	__u32 num_devices;
	__s32 dev_hdls[CAM_REQ_MGR_MAX_HANDLES];
	__s32 link_hdl;
};

struct cam_req_mgr_link_info_v2 {
	__s32 session_hdl;
	__u32 num_devices;
	__s32 dev_hdls[CAM_REQ_MGR_MAX_HANDLES_V2];
	__s32 link_hdl;
};

struct cam_req_mgr_ver_info {
	__u32 version;
	union {
		struct cam_req_mgr_link_info link_info_v1;
		struct cam_req_mgr_link_info_v2 link_info_v2;
	} u;
};
/**
 * struct cam_req_mgr_unlink_info
 * @session_hdl: input param - session handle
 * @link_hdl: input param - link handle
 * @opcode: CAM_REQ_MGR_UNLINK
 */
struct cam_req_mgr_unlink_info {
	__s32 session_hdl;
	__s32 link_hdl;
};

/**
 * struct cam_req_mgr_flush_info
 * @brief: User can tell drivers to flush a particular request id or
 * flush all requests from its pending processing queue. Flush is a
 * blocking call and driver shall ensure all requests are flushed
 * before returning.
 * @session_hdl: Input param - Identifier for CSL session
 * @link_hdl: Input Param -Identifier for link
 * @flush_type: User can cancel a particular req id or can flush
 * all requests in queue
 * @reserved: reserved for 64 bit aligngment
 * @req_id: field is valid only if flush type is cancel request
 * for flush all this field value is not considered.
 * @opcode: CAM_REQ_MGR_FLUSH_REQ
 */
struct cam_req_mgr_flush_info {
	__s32 session_hdl;
	__s32 link_hdl;
	__u32 flush_type;
	__u32 reserved;
	__s64 req_id;
};

/** struct cam_req_mgr_sched_request
 * @session_hdl: Input param - Identifier for CSL session
 * @link_hdl: Input Param -Identifier for link
 * inluding itself.
 * @bubble_enable: Input Param - Cam req mgr will do bubble recovery if this
 * flag is set.
 * @sync_mode: Type of Sync mode for this request
 * @additional_timeout: Additional timeout value (in ms) associated with
 * this request. This value needs to be 0 in cases where long exposure is
 * not configured for the sensor.The max timeout that will be supported
 * is 50000 ms
 * @reserved: Reserved
 * @req_id: Input Param - Request Id from which all requests will be flushed
 */
struct cam_req_mgr_sched_request {
	__s32 session_hdl;
	__s32 link_hdl;
	__s32 bubble_enable;
	__s32 sync_mode;
	__s32 additional_timeout;
	__s32 reserved;
	__s64 req_id;
};

/** struct cam_req_mgr_sched_request_v2
 * @version: Version number
 * @session_hdl: Input param - Identifier for CSL session
 * @link_hdl: Input Param -Identifier for link including itself.
 * @bubble_enable: Input Param - Cam req mgr will do bubble recovery if this
 * flag is set.
 * @sync_mode: Type of Sync mode for this request
 * @additional_timeout: Additional timeout value (in ms) associated with
 * this request. This value needs to be 0 in cases where long exposure is
 * not configured for the sensor.The max timeout that will be supported
 * is 50000 ms
 * @num_links: Input Param - Num of links for sync
 * @num_valid_params: Number of valid params
 * @req_id: Input Param - Request Id from which all requests will be flushed
 * @link_hdls: Input Param - Array of link handles to be for sync
 * @param_mask: mask to indicate what the parameters are
 * @params: parameters passed from user space
 */
struct cam_req_mgr_sched_request_v2 {
	__s32 version;
	__s32 session_hdl;
	__s32 link_hdl;
	__s32 bubble_enable;
	__s32 sync_mode;
	__s32 additional_timeout;
	__s32 num_links;
	__s32 num_valid_params;
	__s64 req_id;
	__s32 link_hdls[MAX_LINKS_PER_SESSION];
	__s32 param_mask;
	__s32 params[5];
};

/** struct cam_req_mgr_sched_request_v3
 * @version: Version number
 * @session_hdl: Input param - Identifier for CSL session
 * @link_hdl: Input Param -Identifier for link including itself.
 * @bubble_enable: Input Param - Cam req mgr will do bubble recovery if this
 * flag is set.
 * @sync_mode: Type of Sync mode for this request
 * @additional_timeout: Additional timeout value (in ms) associated with
 * this request. This value needs to be 0 in cases where long exposure is
 * not configured for the sensor.The max timeout that will be supported
 * is 50000 ms
 * @num_links: Input Param - Num of links for sync
 * @num_valid_params: Number of valid params
 * @req_id: Input Param - Request Id from which all requests will be flushed
 * @param_mask: mask to indicate what the parameters are
 * @params: parameters passed from user space
 * @link_hdls: Input Param - Array of link handles to be for sync
 */
struct cam_req_mgr_sched_request_v3 {
	__s32 version;
	__s32 session_hdl;
	__s32 link_hdl;
	__s32 bubble_enable;
	__s32 sync_mode;
	__s32 additional_timeout;
	__s32 num_links;
	__s32 num_valid_params;
	__s64 req_id;
	__s32 param_mask;
	__s32 params[5];
	__s32 link_hdls[];
};

/**
 * struct cam_req_mgr_sync_mode
 * @session_hdl:         Input param - Identifier for CSL session
 * @sync_mode:           Input Param - Type of sync mode
 * @num_links:           Input Param - Num of links in sync mode (Valid only
 *                             when sync_mode is one of SYNC enabled modes)
 * @link_hdls:           Input Param - Array of link handles to be in sync mode
 *                             (Valid only when sync_mode is one of SYNC
 *                             enabled modes)
 * @master_link_hdl:     Input Param - To dictate which link's SOF drives system
 *                             (Valid only when sync_mode is one of SYNC
 *                             enabled modes)
 *
 * @opcode: CAM_REQ_MGR_SYNC_MODE
 */
struct cam_req_mgr_sync_mode {
	__s32 session_hdl;
	__s32 sync_mode;
	__s32 num_links;
	__s32 link_hdls[MAX_LINKS_PER_SESSION];
	__s32 master_link_hdl;
	__s32 reserved;
};

/**
 * struct cam_req_mgr_link_control
 * @ops:                 Link operations: activate/deactive
 * @session_hdl:         Input param - Identifier for CSL session
 * @num_links:           Input Param - Num of links
 * @reserved:            reserved field
 * @init_timeout:        To account for INIT exposure settings (ms)
 *                       If there is no change in exp settings
 *                       field needs to assigned to 0ms.
 * @link_hdls:           Input Param - Links to be activated/deactivated
 *
 * @opcode: CAM_REQ_MGR_LINK_CONTROL
 */
struct cam_req_mgr_link_control {
	__s32 ops;
	__s32 session_hdl;
	__s32 num_links;
	__s32 reserved;
	__s32 init_timeout[MAX_LINKS_PER_SESSION];
	__s32 link_hdls[MAX_LINKS_PER_SESSION];
};

/**
 * struct cam_req_mgr_link_properties
 * @version: Input param - Version number
 * @session_hdl: Input param - Identifier for CSL session
 * @link_hdl: Input Param - Identifier for link
 * @properties_mask: Input Param - Properties mask to indicate if current
 *                   link enables some special properties
 * @num_valid_params: Input Param - Number of valid params
 * @param_mask: Input Param - Mask to indicate what are the parameters
 * @params: Input Param - Parameters passed from user space
 */
/* CAM_REQ_MGR_LINK_PROPERTIES */
struct cam_req_mgr_link_properties {
	__s32 version;
	__s32 session_hdl;
	__s32 link_hdl;
	__u32 properties_mask;
	__s32 num_valid_params;
	__u32 param_mask;
	__s32 params[6];
};

/**
 * Request Manager : Link properties codes
 * @CAM_LINK_PROPERTY_NONE                     : No special property
 * @CAM_LINK_PROPERTY_SENSOR_STANDBY_AFTER_EOF : Standby the sensor after EOF
 */
#define CAM_LINK_PROPERTY_NONE                      0
#define CAM_LINK_PROPERTY_SENSOR_STANDBY_AFTER_EOF  BIT(0)

/**
 * cam_req_mgr specific opcode ids
 */
#define CAM_REQ_MGR_CREATE_DEV_NODES            (CAM_COMMON_OPCODE_MAX + 1)
#define CAM_REQ_MGR_CREATE_SESSION              (CAM_COMMON_OPCODE_MAX + 2)
#define CAM_REQ_MGR_DESTROY_SESSION             (CAM_COMMON_OPCODE_MAX + 3)
#define CAM_REQ_MGR_LINK                        (CAM_COMMON_OPCODE_MAX + 4)
#define CAM_REQ_MGR_UNLINK                      (CAM_COMMON_OPCODE_MAX + 5)
#define CAM_REQ_MGR_SCHED_REQ                   (CAM_COMMON_OPCODE_MAX + 6)
#define CAM_REQ_MGR_FLUSH_REQ                   (CAM_COMMON_OPCODE_MAX + 7)
#define CAM_REQ_MGR_SYNC_MODE                   (CAM_COMMON_OPCODE_MAX + 8)
#define CAM_REQ_MGR_ALLOC_BUF                   (CAM_COMMON_OPCODE_MAX + 9)
#define CAM_REQ_MGR_MAP_BUF                     (CAM_COMMON_OPCODE_MAX + 10)
#define CAM_REQ_MGR_RELEASE_BUF                 (CAM_COMMON_OPCODE_MAX + 11)
#define CAM_REQ_MGR_CACHE_OPS                   (CAM_COMMON_OPCODE_MAX + 12)
#define CAM_REQ_MGR_LINK_CONTROL                (CAM_COMMON_OPCODE_MAX + 13)
#define CAM_REQ_MGR_LINK_V2                     (CAM_COMMON_OPCODE_MAX + 14)
#define CAM_REQ_MGR_REQUEST_DUMP                (CAM_COMMON_OPCODE_MAX + 15)
#define CAM_REQ_MGR_SCHED_REQ_V2                (CAM_COMMON_OPCODE_MAX + 16)
#define CAM_REQ_MGR_LINK_PROPERTIES             (CAM_COMMON_OPCODE_MAX + 17)
#define CAM_REQ_MGR_ALLOC_BUF_V2                (CAM_COMMON_OPCODE_MAX + 18)
#define CAM_REQ_MGR_MAP_BUF_V2                  (CAM_COMMON_OPCODE_MAX + 19)
#define CAM_REQ_MGR_MEM_CPU_ACCESS_OP           (CAM_COMMON_OPCODE_MAX + 20)
#define CAM_REQ_MGR_QUERY_CAP                   (CAM_COMMON_OPCODE_MAX + 21)
#define CAM_REQ_MGR_SCHED_REQ_V3                (CAM_COMMON_OPCODE_MAX + 22)

/* end of cam_req_mgr opcodes */

#define CAM_MEM_FLAG_HW_READ_WRITE              (1<<0)
#define CAM_MEM_FLAG_HW_READ_ONLY               (1<<1)
#define CAM_MEM_FLAG_HW_WRITE_ONLY              (1<<2)
#define CAM_MEM_FLAG_KMD_ACCESS                 (1<<3)
#define CAM_MEM_FLAG_UMD_ACCESS                 (1<<4)
#define CAM_MEM_FLAG_PROTECTED_MODE             (1<<5)
#define CAM_MEM_FLAG_CMD_BUF_TYPE               (1<<6)
#define CAM_MEM_FLAG_PIXEL_BUF_TYPE             (1<<7)
#define CAM_MEM_FLAG_STATS_BUF_TYPE             (1<<8)
#define CAM_MEM_FLAG_PACKET_BUF_TYPE            (1<<9)
#define CAM_MEM_FLAG_CACHE                      (1<<10)
#define CAM_MEM_FLAG_HW_SHARED_ACCESS           (1<<11)
#define CAM_MEM_FLAG_CDSP_OUTPUT                (1<<12)
#define CAM_MEM_FLAG_DISABLE_DELAYED_UNMAP      (1<<13)
#define CAM_MEM_FLAG_KMD_DEBUG_FLAG             (1<<14)
#define CAM_MEM_FLAG_EVA_NOPIXEL                (1<<15)
#define CAM_MEM_FLAG_HW_AND_CDM_OR_SHARED       (1<<16)
#define CAM_MEM_FLAG_UBWC_P_HEAP                (1<<17)
/* Allocation forced to camera heap */
#define CAM_MEM_FLAG_USE_CAMERA_HEAP_ONLY       (1<<18)

/* Allocation forced to system heap */
#define CAM_MEM_FLAG_USE_SYS_HEAP_ONLY          (1<<19)

#define CAM_MEM_MMU_MAX_HANDLE                  16

/* Maximum allowed buffers in existence */
#define CAM_MEM_BUFQ_MAX                        2048

#define CAM_MEM_MGR_SECURE_BIT_POS              15
#define CAM_MEM_MGR_HDL_IDX_SIZE                15
#define CAM_MEM_MGR_HDL_FD_SIZE                 16
#define CAM_MEM_MGR_HDL_IDX_END_POS             16
#define CAM_MEM_MGR_HDL_FD_END_POS              32

#define CAM_MEM_MGR_HDL_IDX_MASK      ((1 << CAM_MEM_MGR_HDL_IDX_SIZE) - 1)

#define GET_MEM_HANDLE(idx, fd) \
	((idx & CAM_MEM_MGR_HDL_IDX_MASK) | \
	(fd << (CAM_MEM_MGR_HDL_FD_END_POS - CAM_MEM_MGR_HDL_FD_SIZE))) \

#define GET_FD_FROM_HANDLE(hdl) \
	(hdl >> (CAM_MEM_MGR_HDL_FD_END_POS - CAM_MEM_MGR_HDL_FD_SIZE)) \

#define CAM_MEM_MGR_GET_HDL_IDX(hdl) (hdl & CAM_MEM_MGR_HDL_IDX_MASK)

#define CAM_MEM_MGR_SET_SECURE_HDL(hdl, flag) \
	((flag) ? (hdl |= (1 << CAM_MEM_MGR_SECURE_BIT_POS)) : \
	((hdl) &= ~(1 << CAM_MEM_MGR_SECURE_BIT_POS)))

#define CAM_MEM_MGR_IS_SECURE_HDL(hdl) \
	(((hdl) & \
	(1<<CAM_MEM_MGR_SECURE_BIT_POS)) >> CAM_MEM_MGR_SECURE_BIT_POS)

/**
 * memory allocation type
 */
#define CAM_MEM_DMA_NONE                        0
#define CAM_MEM_DMA_BIDIRECTIONAL               1
#define CAM_MEM_DMA_TO_DEVICE                   2
#define CAM_MEM_DMA_FROM_DEVICE                 3

/**
 * memory cache operation
 */
#define CAM_MEM_CLEAN_CACHE                     1
#define CAM_MEM_INV_CACHE                       2
#define CAM_MEM_CLEAN_INV_CACHE                 3

/**
 * memory CPU access operation
 */
#define CAM_MEM_BEGIN_CPU_ACCESS                BIT(0)
#define CAM_MEM_END_CPU_ACCESS                  BIT(1)

/**
 * memory CPU access type
 */
#define CAM_MEM_CPU_ACCESS_READ                 BIT(0)
#define CAM_MEM_CPU_ACCESS_WRITE                BIT(1)

/**
 * Feature mask returned in query_cap
 */
#define CAM_REQ_MGR_MEM_UBWC_P_HEAP_SUPPORTED   BIT(0)
#define CAM_REQ_MGR_MEM_CAMERA_HEAP_SUPPORTED   BIT(1)

/**
 * struct cam_req_mgr_query_cap
 * @version:          Struct version
 * @feature_mask      Supported features
 * @num_valid_params: Valid number of params being used
 * @valid_param_mask: Mask to indicate the field types in params
 * @params:           Additional params
 */
struct cam_req_mgr_query_cap {
	__u32   version;
	__u64   feature_mask;
	__u32   num_valid_params;
	__u32   valid_param_mask;
	__s32   params[5];
};

/**
 * struct cam_mem_alloc_out_params
 * @buf_handle: buffer handle
 * @fd: output buffer file descriptor
 * @vaddr: virtual address pointer
 */
struct cam_mem_alloc_out_params {
	__u32 buf_handle;
	__s32 fd;
	__u64 vaddr;
};

/**
 * struct cam_mem_map_out_params
 * @buf_handle: buffer handle
 * @size: size of the buffer being mapped
 * @vaddr: virtual address pointer
 */
struct cam_mem_map_out_params {
	__u32 buf_handle;
	__u32 size;
	__u64 vaddr;
};

/**
 * struct cam_mem_mgr_alloc_cmd
 * @len: size of buffer to allocate
 * @align: alignment of the buffer
 * @mmu_hdls: array of mmu handles
 * @num_hdl: number of handles
 * @flags: flags of the buffer
 * @out: out params
 */
/* CAM_REQ_MGR_ALLOC_BUF */
struct cam_mem_mgr_alloc_cmd {
	__u64                           len;
	__u64                           align;
	__s32                           mmu_hdls[CAM_MEM_MMU_MAX_HANDLE];
	__u32                           num_hdl;
	__u32                           flags;
	struct cam_mem_alloc_out_params out;
};

/**
 * struct cam_mem_mgr_alloc_cmd_v2
 * @version: Struct version
 * @num_hdl: number of handles
 * @mmu_hdls: array of mmu handles
 * @len: size of buffer to allocate
 * @align: alignment of the buffer
 * @vmids: reserved
 * @buf_name: DMA buffer name
 * @flags: flags of the buffer
 * @num_valid_params: Valid number of params being used
 * @valid_param_mask: Mask to indicate the field types in params
 * @params: Additional params
 * @out: out params
 */
/* CAM_REQ_MGR_ALLOC_BUF_V2 */
struct cam_mem_mgr_alloc_cmd_v2 {
	__u32                           version;
	__u32                           num_hdl;
	__s32                           mmu_hdls[CAM_MEM_MMU_MAX_HANDLE];
	__u64                           len;
	__u64                           align;
	__u64                           vmids;
	char                            buf_name[CAM_DMA_BUF_NAME_LEN];
	__u32                           flags;
	__u32                           num_valid_params;
	__u32                           valid_param_mask;
	__s32                           params[5];
	struct cam_mem_alloc_out_params out;
};

/**
 * struct cam_mem_mgr_map_cmd
 * @mmu_hdls: array of mmu handles
 * @num_hdl: number of handles
 * @flags: flags of the buffer
 * @fd: output buffer file descriptor
 * @reserved: reserved field
 * @out: out params
 */

/* CAM_REQ_MGR_MAP_BUF */
struct cam_mem_mgr_map_cmd {
	__s32                         mmu_hdls[CAM_MEM_MMU_MAX_HANDLE];
	__u32                         num_hdl;
	__u32                         flags;
	__s32                         fd;
	__u32                         reserved;
	struct cam_mem_map_out_params out;
};

/**
 * struct cam_mem_mgr_map_cmd_v2
 * @version: Struct version
 * @fd: output buffer file descriptor
 * @mmu_hdls: array of mmu handles
 * @num_hdl: number of handles
 * @flags: flags of the buffer
 * @vmids: reserved
 * @buf_name: DMA buffer name
 * @num_valid_params: Valid number of params being used
 * @valid_param_mask: Mask to indicate the field types in params
 * @params: Additional params
 * @out: out params
 */

/* CAM_REQ_MGR_MAP_BUF_V2 */
struct cam_mem_mgr_map_cmd_v2 {
	__u32                         version;
	__s32                         fd;
	__s32                         mmu_hdls[CAM_MEM_MMU_MAX_HANDLE];
	__u32                         num_hdl;
	__u32                         flags;
	__u64                         vmids;
	char                          buf_name[CAM_DMA_BUF_NAME_LEN];
	__u32                         num_valid_params;
	__u32                         valid_param_mask;
	__s32                         params[4];
	struct cam_mem_map_out_params out;
};


/**
 * struct cam_mem_mgr_map_cmd
 * @buf_handle: buffer handle
 * @reserved: reserved field
 */
/* CAM_REQ_MGR_RELEASE_BUF */
struct cam_mem_mgr_release_cmd {
	__s32 buf_handle;
	__u32 reserved;
};

/**
 * struct cam_mem_mgr_map_cmd
 * @buf_handle: buffer handle
 * @ops: cache operations
 */
/* CAM_REQ_MGR_CACHE_OPS */
struct cam_mem_cache_ops_cmd {
	__s32 buf_handle;
	__u32 mem_cache_ops;
};

/**
 * struct cam_mem_cpu_access_op
 * @version:          Struct version
 * @buf_handle:       buffer handle
 * @access:           CPU access operation. Allowed params :
 *                    CAM_MEM_BEGIN_CPU_ACCESS
 *                    CAM_MEM_END_CPU_ACCESS
 *                    both
 * @access_type:      CPU access type. Allowed params :
 *                    CAM_MEM_CPU_ACCESS_READ
 *                    CAM_MEM_CPU_ACCESS_WRITE
 *                    both
 * @num_valid_params: Valid number of params being used
 * @valid_param_mask: Mask to indicate the field types in params
 * @params:           Additional params
 */
/* CAM_REQ_MGR_MEM_CPU_ACCESS_OP */
struct cam_mem_cpu_access_op {
	__u32   version;
	__s32   buf_handle;
	__u32   access;
	__u32   access_type;
	__u32   num_valid_params;
	__u32   valid_param_mask;
	__s32   params[4];
};

/**
 * Request Manager : error message type
 * @CAM_REQ_MGR_ERROR_TYPE_DEVICE: Device error message, fatal to session
 * @CAM_REQ_MGR_ERROR_TYPE_REQUEST: Error on a single request, not fatal
 * @CAM_REQ_MGR_ERROR_TYPE_BUFFER: Buffer was not filled, not fatal
 * @CAM_REQ_MGR_ERROR_TYPE_RECOVERY: Fatal error, can be recovered
 * @CAM_REQ_MGR_ERROR_TYPE_SOF_FREEZE: SOF freeze, can be recovered
 * @CAM_REQ_MGR_ERROR_TYPE_FULL_RECOVERY: Full recovery, can be recovered
 * @CAM_REQ_MGR_ERROR_TYPE_PAGE_FAULT: page fault, can be recovered
 * @CAM_REQ_MGR_WARN_TYPE_KMD_RECOVERY: Do internal overflow recovery, notify UMD
 */
#define CAM_REQ_MGR_ERROR_TYPE_DEVICE           0
#define CAM_REQ_MGR_ERROR_TYPE_REQUEST          1
#define CAM_REQ_MGR_ERROR_TYPE_BUFFER           2
#define CAM_REQ_MGR_ERROR_TYPE_RECOVERY         3
#define CAM_REQ_MGR_ERROR_TYPE_SOF_FREEZE       4
#define CAM_REQ_MGR_ERROR_TYPE_FULL_RECOVERY    5
#define CAM_REQ_MGR_ERROR_TYPE_PAGE_FAULT       6
#define CAM_REQ_MGR_WARN_TYPE_KMD_RECOVERY      7

/**
 * Request Manager : Error codes
 * @CAM_REQ_MGR_ISP_UNREPORTED_ERROR           : No Error Code reported
 * @CAM_REQ_MGR_LINK_STALLED_ERROR             : Unable to apply requests on link
 * @CAM_REQ_MGR_CSID_FATAL_ERROR               : CSID FATAL Error
 * @CAM_REQ_MGR_CSID_FIFO_OVERFLOW_ERROR       : CSID OutputFIFO Overflow
 * @CAM_REQ_MGR_CSID_RECOVERY_OVERFLOW_ERROR   : CSID Recovery Overflow
 * @CAM_REQ_MGR_CSID_LANE_FIFO_OVERFLOW_ERROR  : CSID Lane fifo overflow
 * @CAM_REQ_MGR_CSID_PIXEL_COUNT_MISMATCH      : CSID Pixel Count Mismatch
 * @CAM_REQ_MGR_CSID_RX_PKT_HDR_CORRUPTION     : Packet header received by the csid rx is corrupted
 * @CAM_REQ_MGR_CSID_MISSING_PKT_HDR_DATA      : Lesser data received in packet header than expected
 * @CAM_REQ_MGR_CSID_ERR_ON_SENSOR_SWITCHING   : Fatal Error encountered while switching the sensors
 * @CAM_REQ_MGR_CSID_UNBOUNDED_FRAME           : No EOF in the frame or the frame started with eof
 * @CAM_REQ_MGR_ICP_NO_MEMORY                  : ICP No Memory
 * @CAM_REQ_MGR_ICP_ERROR_SYSTEM_FAILURE       : ICP system failure
 * @CAM_REQ_MGR_CSID_MISSING_EOT               : CSID is missing EOT on one or more lanes
 * @CAM_REQ_MGR_CSID_RX_PKT_PAYLOAD_CORRUPTION : CSID long packet payload CRC mismatch
 * @CAM_REQ_MGR_SENSOR_STREAM_OFF_FAILED       : Failed to stream off sensor
 * @CAM_REQ_MGR_VALID_SHUTTER_DROPPED          : Valid shutter dropped
 * @CAM_REQ_MGR_ISP_ERR_HWPD_VIOLATION         : HWPD image size violation
 * @CAM_REQ_MGR_ISP_ERR_SETTING_MISMATCHED     : Setting mismatched between sensor and isp
 */
#define CAM_REQ_MGR_ISP_UNREPORTED_ERROR                 0
#define CAM_REQ_MGR_LINK_STALLED_ERROR                   BIT(0)
#define CAM_REQ_MGR_CSID_FATAL_ERROR                     BIT(1)
#define CAM_REQ_MGR_CSID_FIFO_OVERFLOW_ERROR             BIT(2)
#define CAM_REQ_MGR_CSID_RECOVERY_OVERFLOW_ERROR         BIT(3)
#define CAM_REQ_MGR_CSID_LANE_FIFO_OVERFLOW_ERROR        BIT(4)
#define CAM_REQ_MGR_CSID_PIXEL_COUNT_MISMATCH            BIT(5)
#define CAM_REQ_MGR_CSID_RX_PKT_HDR_CORRUPTION           BIT(6)
#define CAM_REQ_MGR_CSID_MISSING_PKT_HDR_DATA            BIT(7)
#define CAM_REQ_MGR_CSID_ERR_ON_SENSOR_SWITCHING         BIT(8)
#define CAM_REQ_MGR_CSID_UNBOUNDED_FRAME                 BIT(9)
#define CAM_REQ_MGR_ICP_NO_MEMORY                        BIT(10)
#define CAM_REQ_MGR_ICP_SYSTEM_FAILURE                   BIT(11)
#define CAM_REQ_MGR_CSID_MISSING_EOT                     BIT(12)
#define CAM_REQ_MGR_CSID_RX_PKT_PAYLOAD_CORRUPTION       BIT(13)
#define CAM_REQ_MGR_SENSOR_STREAM_OFF_FAILED             BIT(14)
#define CAM_REQ_MGR_VALID_SHUTTER_DROPPED                BIT(15)
#define CAM_REQ_MGR_ISP_ERR_HWPD_VIOLATION               BIT(16)
#define CAM_REQ_MGR_ISP_ERR_OVERFLOW                     BIT(17)
#define CAM_REQ_MGR_ISP_ERR_P2I                          BIT(18)
#define CAM_REQ_MGR_ISP_ERR_VIOLATION                    BIT(19)
#define CAM_REQ_MGR_ISP_ERR_BUSIF_OVERFLOW               BIT(20)
#define CAM_REQ_MGR_ISP_ERR_SETTING_MISMATCHED           BIT(21)
#define CAM_REQ_MGR_ISP_ERR_ILLEGAL_DT_SWITCH            BIT(22)

/**
 * struct cam_req_mgr_error_msg
 * @error_type: type of error
 * @request_id: request id of frame
 * @device_hdl: device handle
 * @linke_hdl: link_hdl
 * @resource_size: size of the resource
 * @error_code: Error code reported by the event.
 *              Note: This field is a bit field.
 */
struct cam_req_mgr_error_msg {
	__u32 error_type;
	__u32 request_id;
	__s32 device_hdl;
	__s32 link_hdl;
	__u32 resource_size;
	__u32 error_code;
};

/**
 * struct cam_req_mgr_frame_msg
 * @request_id: request id of the frame
 * @frame_id: frame id of the frame
 * @timestamp: timestamp of the frame
 * @link_hdl: link handle associated with this message
 * @sof_status: sof status success or fail
 * @frame_id_meta: refers to the meta for
 *                that frame in specific usecases
 * @reserved: reserved
 */
struct cam_req_mgr_frame_msg {
	__u64 request_id;
	__u64 frame_id;
	__u64 timestamp;
	__s32 link_hdl;
	__u32 sof_status;
	__u32 frame_id_meta;
	__u32 reserved;
};

/**
 * enum cam_req_msg_timestamp_type - Identifies index of timestamps
 *
 * @CAM_REQ_SOF_QTIMER_TIMESTAMP:  SOF qtimer timestamp
 * @CAM_REQ_BOOT_TIMESTAMP:        SOF boot timestamp
 * @CAM_REQ_TIMESTAMP_TYPE:        Max enum index for timestamp type
 *
 */
enum cam_req_msg_timestamp_type {
	CAM_REQ_SOF_QTIMER_TIMESTAMP = 0,
	CAM_REQ_BOOT_TIMESTAMP,
	CAM_REQ_TIMESTAMP_MAX
};

/**
 * struct cam_req_mgr_frame_msg
 * @request_id: request id of the frame
 * @frame_id: frame id of the frame
 * @timestamps: array for all the supported timestamps
 * @link_hdl: link handle associated with this message
 * @frame_id_meta: refers to the meta for
 *                that frame in specific usecases
 * @reserved: reserved for future addtions and max size for structure can be 64 bytes
 */
struct cam_req_mgr_frame_msg_v2 {
	__u64 request_id;
	__u64 frame_id;
	__u64 timestamps[CAM_REQ_TIMESTAMP_MAX];
	__s32 link_hdl;
	__u32 frame_id_meta;
	__u32 reserved[4];
};

/**
 * struct cam_req_mgr_custom_msg
 * @custom_type: custom type
 * @request_id: request id of the frame
 * @frame_id: frame id of the frame
 * @timestamp: timestamp of the frame
 * @link_hdl: link handle associated with this message
 * @custom_data: custom data
 */
struct cam_req_mgr_custom_msg {
	__u32 custom_type;
	__u64 request_id;
	__u64 frame_id;
	__u64 timestamp;
	__s32 link_hdl;
	__u64 custom_data;
};

/**
 * Request Manager Node Msg Event Types
 * @CAM_REQ_MGR_NO_EVENT                    : Event type not reported by the hardware
 * @CAM_REQ_MGR_RETRY_EVENT                 : Retry request reported from the hardware
 */
#define CAM_REQ_MGR_NO_EVENT                             0
#define CAM_REQ_MGR_RETRY_EVENT                          1

/**
 * Request Manager Node Msg Event Cause
 * @CAM_REQ_MGR_CAUSE_UNREPORTED           : Event cause not reported by the hardware
 * @CAM_REQ_MGR_JPEG_THUBNAIL_SIZE_ERROR   : JPEG Thumbnail encode size exceeds the threshold size
 */
#define CAM_REQ_MGR_CAUSE_UNREPORTED                     0
#define CAM_REQ_MGR_JPEG_THUBNAIL_SIZE_ERROR             1

/**
* struct cam_req_mgr_node_msg
* @device_hdl          : Device handle of the device reporting the error
* @link_hdl            : link hdl for real time devices
* @event_type          : Type of the event
* @event_cause         : Cause of the event
* @request_id          : Request id
* @custom_data         : custom data
* @reserved            : Reserved field
*/
struct cam_req_mgr_node_msg {
	__s32 device_hdl;
	__s32 link_hdl;
	__u32 event_type;
	__u32 event_cause;
	__u64 request_id;
	__u64 custom_data;
	__u32 reserved[2];
};

/**
 * Request Manager Msg Page Fault event
 * @CAM_REQ_MGR_PF_EVT_BUF_NOT_FOUND          : Faulted buffer not found
 * @CAM_REQ_MGR_PF_EVT_BUF_FOUND_IO_CFG       : Faulted buffer from io_cfg found
 * @CAM_REQ_MGR_PF_EVT_BUF_FOUND_REF_BUF      : Faulted io region buffer in Patch found
 * @CAM_REQ_MGR_PF_EVT_BUF_FOUND_CDM          : Fault in cmd buffer
 * @CAM_REQ_MGR_PF_EVT_BUF_FOUND_SHARED       : Fault in shared region buffer
 */
#define CAM_REQ_MGR_PF_EVT_BUF_NOT_FOUND            0
#define CAM_REQ_MGR_PF_EVT_BUF_FOUND_IO_CFG         1
#define CAM_REQ_MGR_PF_EVT_BUF_FOUND_REF_BUF        2
#define CAM_REQ_MGR_PF_EVT_BUF_FOUND_CDM            3
#define CAM_REQ_MGR_PF_EVT_BUF_FOUND_SHARED         4

/**
 * Faulted Memory Type
 * @CAM_REQ_MGR_PF_TYPE_NULL               : Fault on NULL
 * @CAM_REQ_MGR_PF_TYPE_OUT_OF_BOUND       : Fault on address outside of any mapped buffer
 * @CAM_REQ_MGR_PF_TYPE_MAPPED_REGION      : Fault on address within a mapped buffer
 */
#define CAM_REQ_MGR_PF_TYPE_NULL            0
#define CAM_REQ_MGR_PF_TYPE_OUT_OF_BOUND    1
#define CAM_REQ_MGR_PF_TYPE_MAPPED_REGION   2

/**
 * Faulted Memory stage
 * @CAM_REQ_MGR_STAGE1_FAULT     : Faulted memory in non-secure stage
 * @CAM_REQ_MGR_STAGE2_FAULT     : Faulted memory in secure stage
 */
#define CAM_REQ_MGR_STAGE1_FAULT     0
#define CAM_REQ_MGR_STAGE2_FAULT     1

/**
 * struct cam_req_mgr_pf_err_msg
 * @device_hdl          : device handle of the device reporting the error
 * @link_hdl            : link hdl for real time devices
 * @pf_evt              : indicates if no faulted buffer found or found
 *                        io cfg faulted buffer or found ref faulted buffer
 *                        or found cdm shared fautled buffer
 * @pf_type             : indicates if page fault type is fault on NULL or
 *                        fault out of bound or fault within mapped region
 * @pf_stage            : indicates if faulted memory is from secure or non-secure region
 * @patch_id            : index to which patch in the packet is faulted
 * @buf_hdl             : faulted buffer memory handle
 * @offset              : offset provided in the packet
 * @port_id             : resource type of the io cfg in packet
 * @far_delta           : memory gab between faulted addr and closest
 *                        buffer's starting address
 * @req_id              : request id for the faulted request
 * @bid                 : bus id
 * @pid                 : unique id for hw group of ports
 * @mid                 : port id of hw
 * @reserved            : reserved fields
 */
struct cam_req_mgr_pf_err_msg {
	__s32 device_hdl;
	__s32 link_hdl;
	__u8 pf_evt;
	__u8 pf_type;
	__u8 pf_stage;
	__u8 patch_id;
	__s32 buf_hdl;
	__u32 offset;
	__u32 port_id;
	__u64 far_delta;
	__u64 req_id;
	__u8 bid;
	__u8 pid;
	__u16 mid;
	__u32 reserved[3];
};

/**
 * struct cam_req_mgr_message - 64 bytes is the max size that can be sent as v4l2 evt
 * @session_hdl: session to which the frame belongs to
 * @reserved: reserved field
 * @u: union which can either be error/frame/custom/node message/page fault message
 */
struct cam_req_mgr_message {
	__s32 session_hdl;
	__s32 reserved;
	union {
		struct cam_req_mgr_error_msg err_msg;
		struct cam_req_mgr_frame_msg frame_msg;
		struct cam_req_mgr_frame_msg_v2 frame_msg_v2;
		struct cam_req_mgr_custom_msg custom_msg;
		struct cam_req_mgr_node_msg node_msg;
		struct cam_req_mgr_pf_err_msg pf_err_msg;
	} u;
};
#endif /* __UAPI_LINUX_CAM_REQ_MGR_H */
