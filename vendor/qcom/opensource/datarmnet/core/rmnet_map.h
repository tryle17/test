/* Copyright (c) 2013-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _RMNET_MAP_H_
#define _RMNET_MAP_H_

#include <linux/skbuff.h>
#include "rmnet_config.h"

struct rmnet_map_control_command {
	u8  command_name;
	u8  cmd_type:2;
	u8  reserved:6;
	u16 reserved2;
	u32 transaction_id;
	union {
		struct {
			u16 ip_family:2;
			u16 reserved:14;
			__be16 flow_control_seq_num;
			__be32 qos_id;
		} flow_control;
		DECLARE_FLEX_ARRAY(u8, data);
	};
}  __aligned(1);

enum rmnet_map_commands {
	RMNET_MAP_COMMAND_NONE,
	RMNET_MAP_COMMAND_FLOW_DISABLE,
	RMNET_MAP_COMMAND_FLOW_ENABLE,
	RMNET_MAP_COMMAND_FLOW_START = 7,
	RMNET_MAP_COMMAND_FLOW_END = 8,
	RMNET_MAP_COMMAND_PB_BYTES = 35,
	/* These should always be the last 2 elements */
	RMNET_MAP_COMMAND_UNKNOWN,
	RMNET_MAP_COMMAND_ENUM_LENGTH
};

enum rmnet_map_v5_header_type {
	RMNET_MAP_HEADER_TYPE_UNKNOWN,
	RMNET_MAP_HEADER_TYPE_COALESCING = 0x1,
	RMNET_MAP_HEADER_TYPE_CSUM_OFFLOAD = 0x2,
	RMNET_MAP_HEADER_TYPE_TSO = 0x3,
	RMNET_MAP_HEADER_TYPE_ENUM_LENGTH
};

enum rmnet_map_v5_close_type {
	RMNET_MAP_COAL_CLOSE_NON_COAL,
	RMNET_MAP_COAL_CLOSE_IP_MISS,
	RMNET_MAP_COAL_CLOSE_TRANS_MISS,
	RMNET_MAP_COAL_CLOSE_HW,
	RMNET_MAP_COAL_CLOSE_COAL,
};

enum rmnet_map_v5_close_value {
	RMNET_MAP_COAL_CLOSE_HW_NL,
	RMNET_MAP_COAL_CLOSE_HW_PKT,
	RMNET_MAP_COAL_CLOSE_HW_BYTE,
	RMNET_MAP_COAL_CLOSE_HW_TIME,
	RMNET_MAP_COAL_CLOSE_HW_EVICT,
};

/* Main QMAP header */
struct rmnet_map_header {
	u8  pad_len:6;
	u8  next_hdr:1;
	u8  cd_bit:1;
	u8  mux_id;
	__be16 pkt_len;
}  __aligned(1);

/* QMAP v5 headers */
struct rmnet_map_v5_csum_header {
	u8  next_hdr:1;
	u8  header_type:7;
	u8  hw_reserved:4;
	u8  aps_prio:1;
	u8  priority:1;
	u8  hw_reserved_bit:1;
	u8  csum_valid_required:1;
	__be16 reserved;
} __aligned(1);

struct rmnet_map_v5_nl_pair {
	__be16 pkt_len;
	u8  csum_error_bitmap;
	u8  num_packets;
} __aligned(1);

/* NLO: Number-length object */
#define RMNET_MAP_V5_MAX_NLOS         (6)
#define RMNET_MAP_V5_MAX_PACKETS      (48)

struct rmnet_map_v5_coal_header {
	u8  next_hdr:1;
	u8  header_type:7;
	u8  reserved1:4;
	u8  num_nlos:3;
	u8  csum_valid:1;
	u8  close_type:4;
	u8  close_value:4;
	u8  reserved2:4;
	u8  virtual_channel_id:4;

	struct rmnet_map_v5_nl_pair nl_pairs[RMNET_MAP_V5_MAX_NLOS];
} __aligned(1);

struct rmnet_map_v5_tso_header {
	u8  next_hdr:1;
	u8  header_type:7;
	u8  hw_reserved:5;
	u8  priority:1;
	u8  zero_csum:1;
	u8  ip_id_cfg:1;
	__be16 segment_size;
} __aligned(1);

/* QMAP v4 headers */
struct rmnet_map_dl_csum_trailer {
	u8  reserved1;
	u8  valid:1;
	u8  reserved2:7;
	u16 csum_start_offset;
	u16 csum_length;
	__be16 csum_value;
} __aligned(1);

struct rmnet_map_ul_csum_header {
	__be16 csum_start_offset;
	u16 csum_insert_offset:14;
	u16 udp_ind:1;
	u16 csum_enabled:1;
} __aligned(1);

struct rmnet_map_control_command_header {
	u8 command_name;
	u8 cmd_type:2;
	u8 reserved:5;
	u8 e:1;
	u16 source_id:15;
	u16 ext:1;
	u32 transaction_id;
}  __aligned(1);

struct rmnet_map_flow_info_le {
	__be32 mux_id;
	__be32 flow_id;
	__be32 bytes;
	__be32 pkts;
} __aligned(1);

struct rmnet_map_flow_info_be {
	u32 mux_id;
	u32 flow_id;
	u32 bytes;
	u32 pkts;
} __aligned(1);

struct rmnet_map_pb_ind_hdr {
	union {
		struct {
			u32 seq_num;
			u32 start_end_seq_num;
			u32 row_bytes_pending;
			u32 fc_bytes_pending;
		} le __aligned(1);
		struct {
			u32 seq_num;
			u32 start_end_seq_num;
			u32 row_bytes_pending;
			u32 fc_bytes_pending;
		} be __aligned(1);
	} __aligned(1);
} __aligned(1);

struct rmnet_map_pb_ind {
	u8 priority;
	void (*pb_ind_handler)(struct rmnet_map_pb_ind_hdr *pbhdr);
	struct list_head list;
};

struct rmnet_map_dl_ind_hdr {
	union {
		struct {
			u32 seq;
			u32 bytes;
			u32 pkts;
			u32 flows;
			struct rmnet_map_flow_info_le flow[];
		} le __aligned(1);
		struct {
			__be32 seq;
			__be32 bytes;
			__be32 pkts;
			__be32 flows;
			struct rmnet_map_flow_info_be flow[];
		} be __aligned(1);
	} __aligned(1);
} __aligned(1);

struct rmnet_map_dl_ind_trl {
	union {
		__be32 seq_be;
		u32 seq_le;
	} __aligned(1);
} __aligned(1);

struct rmnet_map_dl_ind {
	u8 priority;
	void (*dl_hdr_handler_v2)(struct rmnet_map_dl_ind_hdr *dlhdr,
				  struct rmnet_map_control_command_header *qcmd);
	void (*dl_trl_handler_v2)(struct rmnet_map_dl_ind_trl *dltrl,
				  struct rmnet_map_control_command_header *qcmd);
	struct list_head list;
};

#define RMNET_MAP_GET_MUX_ID(Y) (((struct rmnet_map_header *) \
				 (Y)->data)->mux_id)
#define RMNET_MAP_GET_CD_BIT(Y) (((struct rmnet_map_header *) \
				(Y)->data)->cd_bit)
#define RMNET_MAP_GET_PAD(Y) (((struct rmnet_map_header *) \
				(Y)->data)->pad_len)
#define RMNET_MAP_GET_CMD_START(Y) ((struct rmnet_map_control_command *) \
				    ((Y)->data + \
				      sizeof(struct rmnet_map_header)))
#define RMNET_MAP_GET_LENGTH(Y) (ntohs(((struct rmnet_map_header *) \
					(Y)->data)->pkt_len))

#define RMNET_MAP_DEAGGR_SPACING  64
#define RMNET_MAP_DEAGGR_HEADROOM (RMNET_MAP_DEAGGR_SPACING / 2)

#define RMNET_MAP_COMMAND_REQUEST     0
#define RMNET_MAP_COMMAND_ACK         1
#define RMNET_MAP_COMMAND_UNSUPPORTED 2
#define RMNET_MAP_COMMAND_INVALID     3

#define RMNET_MAP_NO_PAD_BYTES        0
#define RMNET_MAP_ADD_PAD_BYTES       1

static inline unsigned char *rmnet_map_data_ptr(struct sk_buff *skb)
{
	/* Nonlinear packets we receive are entirely within frag 0 */
	if (skb_is_nonlinear(skb) && skb->len == skb->data_len)
		return skb_frag_address(skb_shinfo(skb)->frags);

	return skb->data;
}

static inline struct rmnet_map_control_command *
rmnet_map_get_cmd_start(struct sk_buff *skb)
{
	unsigned char *data = rmnet_map_data_ptr(skb);

	data += sizeof(struct rmnet_map_header);
	return (struct rmnet_map_control_command *)data;
}

static inline u8 rmnet_map_get_next_hdr_type(struct sk_buff *skb)
{
	unsigned char *data = rmnet_map_data_ptr(skb);

	data += sizeof(struct rmnet_map_header);
	return ((struct rmnet_map_v5_coal_header *)data)->header_type;
}

static inline bool rmnet_map_get_csum_valid(struct sk_buff *skb)
{
	unsigned char *data = rmnet_map_data_ptr(skb);

	data += sizeof(struct rmnet_map_header);
	return ((struct rmnet_map_v5_csum_header *)data)->csum_valid_required;
}

struct sk_buff *rmnet_map_deaggregate(struct sk_buff *skb,
				      struct rmnet_port *port);
struct rmnet_map_header *rmnet_map_add_map_header(struct sk_buff *skb,
						  int hdrlen, int pad,
						  struct rmnet_port *port);
void rmnet_map_command(struct sk_buff *skb, struct rmnet_port *port);
int rmnet_map_checksum_downlink_packet(struct sk_buff *skb, u16 len);
void rmnet_map_checksum_uplink_packet(struct sk_buff *skb,
				      struct rmnet_port *port,
				      struct net_device *orig_dev,
				      int csum_type);
bool rmnet_map_v5_csum_buggy(struct rmnet_map_v5_coal_header *coal_hdr);
int rmnet_map_process_next_hdr_packet(struct sk_buff *skb,
				      struct sk_buff_head *list,
				      u16 len);
int rmnet_map_tx_agg_skip(struct sk_buff *skb, int offset);
void rmnet_map_tx_aggregate(struct sk_buff *skb, struct rmnet_port *port,
			    bool low_latency);
void rmnet_map_tx_aggregate_init(struct rmnet_port *port);
void rmnet_map_tx_aggregate_exit(struct rmnet_port *port);
void rmnet_map_update_ul_agg_config(struct rmnet_aggregation_state *state,
				    u16 size, u8 count, u8 features, u32 time);
void rmnet_map_dl_hdr_notify_v2(struct rmnet_port *port,
				struct rmnet_map_dl_ind_hdr *dl_hdr,
				struct rmnet_map_control_command_header *qcmd);
void rmnet_map_dl_trl_notify_v2(struct rmnet_port *port,
				struct rmnet_map_dl_ind_trl *dltrl,
				struct rmnet_map_control_command_header *qcmd);
void rmnet_map_pb_ind_notify(struct rmnet_port *port,
			     struct rmnet_map_pb_ind_hdr *pbhdr);
int rmnet_map_flow_command(struct sk_buff *skb,
			   struct rmnet_port *port,
			   bool rmnet_perf);
void rmnet_map_cmd_init(struct rmnet_port *port);
int rmnet_map_dl_ind_register(struct rmnet_port *port,
			      struct rmnet_map_dl_ind *dl_ind);
int rmnet_map_dl_ind_deregister(struct rmnet_port *port,
				struct rmnet_map_dl_ind *dl_ind);
int rmnet_map_pb_ind_register(struct rmnet_port *port,
			      struct rmnet_map_pb_ind *pb_ind);
int rmnet_map_pb_ind_deregister(struct rmnet_port *port,
				struct rmnet_map_pb_ind *pb_ind);
void rmnet_map_cmd_exit(struct rmnet_port *port);
void rmnet_map_tx_qmap_cmd(struct sk_buff *qmap_skb, u8 ch, bool flush);
void rmnet_map_send_agg_skb(struct rmnet_aggregation_state *state);
int rmnet_map_add_tso_header(struct sk_buff *skb, struct rmnet_port *port,
			      struct net_device *orig_dev);
#endif /* _RMNET_MAP_H_ */
