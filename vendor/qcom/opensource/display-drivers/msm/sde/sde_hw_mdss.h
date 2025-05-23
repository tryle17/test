/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 */

#ifndef _SDE_HW_MDSS_H
#define _SDE_HW_MDSS_H

#include <linux/kernel.h>
#include <linux/err.h>

#include <drm/sde_drm.h>
#include <drm/msm_drm_pp.h>
#include <drm/drm_fourcc.h>

#include "msm_drv.h"

#define SDE_DBG_NAME			"sde"

#define SDE_NONE                        0

#ifndef SDE_CSC_MATRIX_COEFF_SIZE
#define SDE_CSC_MATRIX_COEFF_SIZE	9
#endif

#ifndef SDE_CSC_CLAMP_SIZE
#define SDE_CSC_CLAMP_SIZE		6
#endif

#ifndef SDE_CSC_BIAS_SIZE
#define SDE_CSC_BIAS_SIZE		3
#endif

#ifndef SDE_MAX_PLANES
#define SDE_MAX_PLANES			4
#endif

#define PIPES_PER_STAGE			2
#ifndef SDE_MAX_DE_CURVES
#define SDE_MAX_DE_CURVES		3
#endif

#define MAX_DSI_DISPLAYS		2
#define MAX_DATA_PATH_PER_DSIPLAY	4

enum sde_format_flags {
	SDE_FORMAT_FLAG_YUV_BIT,
	SDE_FORMAT_FLAG_DX_BIT,
	SDE_FORMAT_FLAG_COMPRESSED_BIT,
	SDE_FORMAT_FLAG_ALPHA_SWAP_BIT,
	SDE_FORMAT_FLAG_FP16_BIT,
	SDE_FORMAT_FLAG_LOSSY_8_5_BIT,
	SDE_FORMAT_FLAG_LOSSY_2_1_BIT,
	SDE_FORMAT_FLAG_CAC_BIT,
	SDE_FORMAT_FLAG_BIT_MAX,
};

#define SDE_FORMAT_FLAG_YUV		BIT(SDE_FORMAT_FLAG_YUV_BIT)
#define SDE_FORMAT_FLAG_DX		BIT(SDE_FORMAT_FLAG_DX_BIT)
#define SDE_FORMAT_FLAG_COMPRESSED	BIT(SDE_FORMAT_FLAG_COMPRESSED_BIT)
#define SDE_FORMAT_FLAG_ALPHA_SWAP	BIT(SDE_FORMAT_FLAG_ALPHA_SWAP_BIT)
#define SDE_FORMAT_FLAG_FP16		BIT(SDE_FORMAT_FLAG_FP16_BIT)
#define SDE_FORMAT_FLAG_LOSSY_8_5	BIT(SDE_FORMAT_FLAG_LOSSY_8_5_BIT)
#define SDE_FORMAT_FLAG_LOSSY_2_1	BIT(SDE_FORMAT_FLAG_LOSSY_2_1_BIT)
#define SDE_FORMAT_FLAG_CAC		BIT(SDE_FORMAT_FLAG_CAC_BIT)
#define SDE_FORMAT_IS_YUV(X)		\
	(test_bit(SDE_FORMAT_FLAG_YUV_BIT, (X)->flag))
#define SDE_FORMAT_IS_DX(X)		\
	(test_bit(SDE_FORMAT_FLAG_DX_BIT, (X)->flag))
#define SDE_FORMAT_IS_LINEAR(X)		((X)->fetch_mode == SDE_FETCH_LINEAR)
#define SDE_FORMAT_IS_TILE(X) \
	(((X)->fetch_mode == SDE_FETCH_UBWC) && \
			!test_bit(SDE_FORMAT_FLAG_COMPRESSED_BIT, (X)->flag))
#define SDE_FORMAT_IS_UBWC(X) \
	(((X)->fetch_mode == SDE_FETCH_UBWC) && \
			test_bit(SDE_FORMAT_FLAG_COMPRESSED_BIT, (X)->flag))
#define SDE_FORMAT_IS_UBWC_LOSSY_8_5(X) \
	(((X)->fetch_mode == SDE_FETCH_UBWC) && \
			test_bit(SDE_FORMAT_FLAG_COMPRESSED_BIT, (X)->flag) && \
			test_bit(SDE_FORMAT_FLAG_LOSSY_8_5_BIT, (X)->flag))
#define SDE_FORMAT_IS_UBWC_LOSSY_2_1(X) \
	(((X)->fetch_mode == SDE_FETCH_UBWC) && \
			test_bit(SDE_FORMAT_FLAG_COMPRESSED_BIT, (X)->flag) && \
			test_bit(SDE_FORMAT_FLAG_LOSSY_2_1_BIT, (X)->flag))
#define SDE_FORMAT_IS_ALPHA_SWAPPED(X) \
	(test_bit(SDE_FORMAT_FLAG_ALPHA_SWAP_BIT, (X)->flag))
#define SDE_FORMAT_IS_FP16(X) \
	(test_bit(SDE_FORMAT_FLAG_FP16_BIT, (X)->flag))
#define SDE_FORMAT_IS_CAC_FETCH(X) \
	(test_bit(SDE_FORMAT_FLAG_CAC_BIT, (X)->flag))

#define MDP_TICK_COUNT                    16
#define XO_CLK_RATE                       19200
#define MS_TICKS_IN_SEC                   1000

#define CALCULATE_WD_LOAD_VALUE(fps) \
	((uint32_t)((MS_TICKS_IN_SEC * XO_CLK_RATE)/(MDP_TICK_COUNT * fps)))

#define SDE_BLEND_FG_ALPHA_FG_CONST	(0 << 0)
#define SDE_BLEND_FG_ALPHA_BG_CONST	(1 << 0)
#define SDE_BLEND_FG_ALPHA_FG_PIXEL	(2 << 0)
#define SDE_BLEND_FG_ALPHA_BG_PIXEL	(3 << 0)
#define SDE_BLEND_FG_INV_ALPHA		(1 << 2)
#define SDE_BLEND_FG_MOD_ALPHA		(1 << 3)
#define SDE_BLEND_FG_INV_MOD_ALPHA	(1 << 4)
#define SDE_BLEND_FG_TRANSP_EN		(1 << 5)
#define SDE_BLEND_BG_ALPHA_FG_CONST	(0 << 8)
#define SDE_BLEND_BG_ALPHA_BG_CONST	(1 << 8)
#define SDE_BLEND_BG_ALPHA_FG_PIXEL	(2 << 8)
#define SDE_BLEND_BG_ALPHA_BG_PIXEL	(3 << 8)
#define SDE_BLEND_BG_INV_ALPHA		(1 << 10)
#define SDE_BLEND_BG_MOD_ALPHA		(1 << 11)
#define SDE_BLEND_BG_INV_MOD_ALPHA	(1 << 12)
#define SDE_BLEND_BG_TRANSP_EN		(1 << 13)

#define SDE_VSYNC0_SOURCE_GPIO		0
#define SDE_VSYNC1_SOURCE_GPIO		1
#define SDE_VSYNC2_SOURCE_GPIO		2
#define SDE_VSYNC_SOURCE_INTF_0		3
#define SDE_VSYNC_SOURCE_INTF_1		4
#define SDE_VSYNC_SOURCE_INTF_2		5
#define SDE_VSYNC_SOURCE_INTF_3		6
#define SDE_VSYNC_SOURCE_WD_TIMER_4	11
#define SDE_VSYNC_SOURCE_WD_TIMER_3	12
#define SDE_VSYNC_SOURCE_WD_TIMER_2	13
#define SDE_VSYNC_SOURCE_WD_TIMER_1	14
#define SDE_VSYNC_SOURCE_WD_TIMER_0	15

enum sde_hw_blk_type {
	SDE_HW_BLK_TOP = 0,
	SDE_HW_BLK_SSPP,
	SDE_HW_BLK_LM,
	SDE_HW_BLK_DSPP,
	SDE_HW_BLK_DS,
	SDE_HW_BLK_CTL,
	SDE_HW_BLK_CDM,
	SDE_HW_BLK_PINGPONG,
	SDE_HW_BLK_INTF,
	SDE_HW_BLK_WB,
	SDE_HW_BLK_DSC,
	SDE_HW_BLK_VDC,
	SDE_HW_BLK_MERGE_3D,
	SDE_HW_BLK_QDSS,
	SDE_HW_BLK_DNSC_BLUR,
	SDE_HW_BLK_MAX,
};

enum sde_uidle {
	UIDLE = 0x1,
	UIDLE_MAX,
};

enum sde_mdp {
	MDP_TOP = 0x1,
	MDP_MAX,
};

enum sde_sspp {
	SSPP_NONE,
	SSPP_VIG0,
	SSPP_VIG1,
	SSPP_VIG2,
	SSPP_VIG3,
	SSPP_VIG4,
	SSPP_VIG5,
	SSPP_VIG6,
	SSPP_VIG7,
	SSPP_VIG_MAX = SSPP_VIG7,
	SSPP_DMA0,
	SSPP_DMA1,
	SSPP_DMA2,
	SSPP_DMA3,
	SSPP_DMA4,
	SSPP_DMA5,
	SSPP_DMA_MAX = SSPP_DMA5,
	SSPP_MAX
};

#define SDE_SSPP_VALID(x) ((x) > SSPP_NONE && (x) < SSPP_MAX)
#define SDE_SSPP_VALID_VIG(x) ((x) >= SSPP_VIG0 && (x) <= SSPP_VIG_MAX)
#define SDE_SSPP_VALID_DMA(x) ((x) >= SSPP_DMA0 && (x) <= SSPP_DMA_MAX)

enum sde_dpu {
	DPU_0,
	DPU_1,
	DPU_MAX
};

enum sde_sspp_type {
	SSPP_TYPE_VIG,
	SSPP_TYPE_DMA,
	SSPP_TYPE_MAX
};

enum sde_sspp_rect {
	R0,
	R1,
	R_MAX
};

enum sde_lm {
	LM_0 = 1,
	LM_1,
	LM_2,
	LM_3,
	LM_4,
	LM_5,
	LM_6,
	LM_7,
	LM_DCWB_DUMMY_0,
	LM_DCWB_DUMMY_1,
	LM_DCWB_DUMMY_2,
	LM_DCWB_DUMMY_3,
	LM_MAX
};

enum sde_stage {
	SDE_STAGE_BASE = 0,
	SDE_STAGE_0,
	SDE_STAGE_1,
	SDE_STAGE_2,
	SDE_STAGE_3,
	SDE_STAGE_4,
	SDE_STAGE_5,
	SDE_STAGE_6,
	SDE_STAGE_7,
	SDE_STAGE_8,
	SDE_STAGE_9,
	SDE_STAGE_10,
	SDE_STAGE_MAX
};

enum sde_dspp {
	DSPP_0 = 1,
	DSPP_1,
	DSPP_2,
	DSPP_3,
	DSPP_MAX
};

enum sde_ltm {
	LTM_0 = DSPP_0,
	LTM_1,
	LTM_2,
	LTM_3,
	LTM_MAX
};

enum sde_rc {
	RC_0 = DSPP_0,
	RC_1,
	RC_2,
	RC_3,
	RC_MAX
};

enum sde_demura {
	DEMURA_0,
	DEMURA_1,
	DEMURA_2,
	DEMURA_3,
	DEMURA_MAX
};

enum sde_ds {
	DS_TOP,
	DS_0,
	DS_1,
	DS_2,
	DS_3,
	DS_MAX
};

enum sde_ctl {
	CTL_0 = 1,
	CTL_1,
	CTL_2,
	CTL_3,
	CTL_4,
	CTL_5,
	CTL_MAX
};

enum sde_cdm {
	CDM_0 = 1,
	CDM_1,
	CDM_MAX
};

enum sde_dnsc_blur {
	DNSC_BLUR_0 = 1,
	DNSC_BLUR__MAX
};

enum sde_pingpong {
	PINGPONG_0 = 1,
	PINGPONG_1,
	PINGPONG_2,
	PINGPONG_3,
	PINGPONG_4,
	PINGPONG_5,
	PINGPONG_6,
	PINGPONG_7,
	PINGPONG_CWB_0,
	PINGPONG_CWB_1,
	PINGPONG_CWB_2,
	PINGPONG_CWB_3,
	PINGPONG_S0,
	PINGPONG_MAX
};

enum sde_dsc {
	DSC_NONE = 0,
	DSC_0,
	DSC_1,
	DSC_2,
	DSC_3,
	DSC_4,
	DSC_5,
	DSC_6,
	DSC_7,
	DSC_MAX
};

enum sde_vdc {
	VDC_NONE = 0,
	VDC_0,
	VDC_1,
	VDC_MAX
};

enum sde_intf {
	INTF_0 = 1,
	INTF_1,
	INTF_2,
	INTF_3,
	INTF_4,
	INTF_5,
	INTF_6,
	INTF_7,
	INTF_8,
	INTF_MAX
};

enum sde_intf_type {
	INTF_NONE = 0x0,
	INTF_DSI = 0x1,
	INTF_HDMI = 0x3,
	INTF_LCDC = 0x5,
	INTF_EDP = 0x9,
	INTF_DP = 0xa,
	INTF_TYPE_MAX,

	/* virtual interfaces */
	INTF_WB = 0x100,
};

enum sde_intf_mode {
	INTF_MODE_NONE = 0,
	INTF_MODE_CMD,
	INTF_MODE_VIDEO,
	INTF_MODE_WB_BLOCK,
	INTF_MODE_WB_LINE,
	INTF_MODE_MAX
};

enum sde_wb {
	WB_0 = 1,
	WB_1,
	WB_2,
	WB_3,
	WB_MAX
};

enum sde_ad {
	AD_0 = 0x1,
	AD_1,
	AD_MAX
};

enum sde_cwb {
	CWB_0 = 0x1,
	CWB_1,
	CWB_2,
	CWB_3,
	CWB_4,
	CWB_5,
	CWB_MAX
};

enum sde_dcwb {
	DCWB_0 = 0x1,
	DCWB_1,
	DCWB_2,
	DCWB_3,
	DCWB_MAX
};

enum sde_wd_timer {
	WD_TIMER_0 = 0x1,
	WD_TIMER_1,
	WD_TIMER_2,
	WD_TIMER_3,
	WD_TIMER_4,
	WD_TIMER_5,
	WD_TIMER_MAX
};

enum sde_vbif {
	VBIF_0,
	VBIF_1,
	VBIF_MAX,
	VBIF_RT = VBIF_0,
	VBIF_NRT = VBIF_1
};

enum sde_iommu_domain {
	SDE_IOMMU_DOMAIN_UNSECURE,
	SDE_IOMMU_DOMAIN_SECURE,
	SDE_IOMMU_DOMAIN_MAX
};

enum sde_rot {
	ROT_0 = 1,
	ROT_MAX
};

enum sde_merge_3d {
	MERGE_3D_0 = 1,
	MERGE_3D_1,
	MERGE_3D_2,
	MERGE_3D_3,
	MERGE_3D_CWB_0,
	MERGE_3D_CWB_1,
	MERGE_3D_MAX
};

enum sde_qdss {
	QDSS_0,
	QDSS_MAX
};

/**
 * SDE HW,Component order color map
 */
enum {
	C0_G_Y = 0,
	C1_B_Cb = 1,
	C2_R_Cr = 2,
	C3_ALPHA = 3
};

/**
 * enum sde_plane_type - defines how the color component pixel packing
 * @SDE_PLANE_INTERLEAVED   : Color components in single plane
 * @SDE_PLANE_PLANAR        : Color component in separate planes
 * @SDE_PLANE_PSEUDO_PLANAR : Chroma components interleaved in separate plane
 */
enum sde_plane_type {
	SDE_PLANE_INTERLEAVED,
	SDE_PLANE_PLANAR,
	SDE_PLANE_PSEUDO_PLANAR,
};

/**
 * enum sde_chroma_samp_type - chroma sub-samplng type
 * @SDE_CHROMA_RGB   : No chroma subsampling
 * @SDE_CHROMA_H2V1  : Chroma pixels are horizontally subsampled
 * @SDE_CHROMA_H1V2  : Chroma pixels are vertically subsampled
 * @SDE_CHROMA_420   : 420 subsampling
 */
enum sde_chroma_samp_type {
	SDE_CHROMA_RGB,
	SDE_CHROMA_H2V1,
	SDE_CHROMA_H1V2,
	SDE_CHROMA_420
};

/**
 * sde_fetch_type - Defines How SDE HW fetches data
 * @SDE_FETCH_LINEAR   : fetch is line by line
 * @SDE_FETCH_TILE     : fetches data in Z order from a tile
 * @SDE_FETCH_UBWC     : fetch and decompress data
 */
enum sde_fetch_type {
	SDE_FETCH_LINEAR,
	SDE_FETCH_TILE,
	SDE_FETCH_UBWC
};

/**
 * Value of enum chosen to fit the number of bits
 * expected by the HW programming.
 */
enum {
	COLOR_ALPHA_1BIT = 0,
	COLOR_ALPHA_4BIT = 1,
	COLOR_4BIT = 0,
	COLOR_5BIT = 1, /* No 5-bit Alpha */
	COLOR_6BIT = 2, /* 6-Bit Alpha also = 2 */
	COLOR_8BIT = 3, /* 8-Bit Alpha also = 3 */
	COLOR_16BIT = 3,
};

/**
 * enum sde_3d_blend_mode
 * Desribes how the 3d data is blended
 * @BLEND_3D_NONE      : 3d blending not enabled
 * @BLEND_3D_FRAME_INT : Frame interleaving
 * @BLEND_3D_H_ROW_INT : Horizontal row interleaving
 * @BLEND_3D_V_ROW_INT : vertical row interleaving
 * @BLEND_3D_COL_INT   : column interleaving
 * @BLEND_3D_MAX       :
 */
enum sde_3d_blend_mode {
	BLEND_3D_NONE = 0,
	BLEND_3D_FRAME_INT,
	BLEND_3D_H_ROW_INT,
	BLEND_3D_V_ROW_INT,
	BLEND_3D_COL_INT,
	BLEND_3D_MAX
};

/**
 * enum sde_sys_cache_state: states of disp system cache
 * CACHE_STATE_DISABLED: sys cache has been disabled
 * CACHE_STATE_ENABLED: sys cache has been enabled
 * CACHE_STATE_NORMAL: sys cache is normal state
 * CACHE_STATE_PRE_CACHE: frame cache is being prepared
 * CACHE_STATE_FRAME_WRITE: sys cache is being written to
 * CACHE_STATE_FRAME_READ: sys cache is being read
 */
enum sde_sys_cache_state {
	CACHE_STATE_DISABLED,
	CACHE_STATE_ENABLED,
	CACHE_STATE_NORMAL,
	CACHE_STATE_PRE_CACHE,
	CACHE_STATE_FRAME_WRITE,
	CACHE_STATE_FRAME_READ
};

/**
 * enum sde_wb_usage_type: Type of usage of the WB connector
 * WB_USAGE_WFD: WB connector used for WFD
 * WB_USAGE_CWB: WB connector used for concurrent writeback
 * WB_USAGE_OFFLINE_WB: WB connector used for 2-pass composition
 * WB_USAGE_ROT: WB connector used for image rotation for 2 pass composition
 */
enum sde_wb_usage_type {
	WB_USAGE_WFD,
	WB_USAGE_CWB,
	WB_USAGE_OFFLINE_WB,
	WB_USAGE_ROT,
};

/**
 * enum sde_wb_rot_type: Type of rotation use case of the WB connector
 * WB_ROT_NONE : WB Rotation not in use
 * WB_ROT_SINGLE: WB Rotation used in single job mode for full image rotation
 * WB_ROT_JOB1: WB Rotation used for rotating half image as first-job
 * WB_ROT_JOB2: WB Rotation used for rotating half image as second-job
 */
enum sde_wb_rot_type {
	WB_ROT_NONE,
	WB_ROT_SINGLE,
	WB_ROT_JOB1,
	WB_ROT_JOB2,
};

/** struct sde_format - defines the format configuration which
 * allows SDE HW to correctly fetch and decode the format
 * @base: base msm_format struture containing fourcc code
 * @fetch_planes: how the color components are packed in pixel format
 * @element: element color ordering
 * @bits: element bit widths
 * @chroma_sample: chroma sub-samplng type
 * @unpack_align_msb: unpack aligned, 0 to LSB, 1 to MSB
 * @unpack_tight: 0 for loose, 1 for tight
 * @unpack_count: 0 = 1 component, 1 = 2 component
 * @bpp: bytes per pixel
 * @alpha_enable: whether the format has an alpha channel
 * @num_planes: number of planes (including meta data planes)
 * @fetch_mode: linear, tiled, or ubwc hw fetch behavior
 * @is_yuv: is format a yuv variant
 * @flag: usage bit flags
 * @tile_width: format tile width
 * @tile_height: format tile height
 */
struct sde_format {
	struct msm_format base;
	enum sde_plane_type fetch_planes;
	u8 element[SDE_MAX_PLANES];
	u8 bits[SDE_MAX_PLANES];
	enum sde_chroma_samp_type chroma_sample;
	u8 unpack_align_msb;
	u8 unpack_tight;
	u8 unpack_count;
	u8 bpp;
	u8 alpha_enable;
	u8 num_planes;
	enum sde_fetch_type fetch_mode;
	DECLARE_BITMAP(flag, SDE_FORMAT_FLAG_BIT_MAX);
	u16 tile_width;
	u16 tile_height;
};
#define to_sde_format(x) container_of(x, struct sde_format, base)

/**
 * struct sde_hw_fmt_layout - format information of the source pixel data
 * @format: pixel format parameters
 * @num_planes: number of planes (including meta data planes)
 * @width: image width
 * @height: image height
 * @total_size: total size in bytes
 * @plane_addr: address of each plane
 * @plane_size: length of each plane
 * @plane_pitch: pitch of each plane
 */
struct sde_hw_fmt_layout {
	const struct sde_format *format;
	uint32_t num_planes;
	uint32_t width;
	uint32_t height;
	uint32_t total_size;
	uint32_t plane_addr[SDE_MAX_PLANES];
	uint32_t plane_size[SDE_MAX_PLANES];
	uint32_t plane_pitch[SDE_MAX_PLANES];
};

struct sde_rect {
	u16 x;
	u16 y;
	u16 w;
	u16 h;
};

struct sde_io_res {
	bool enabled;
	u32 src_w;
	u32 src_h;
	u32 dst_w;
	u32 dst_h;
};

struct sde_csc_cfg {
	/* matrix coefficients in S15.16 format */
	uint32_t csc_mv[SDE_CSC_MATRIX_COEFF_SIZE];
	uint32_t csc_pre_bv[SDE_CSC_BIAS_SIZE];
	uint32_t csc_post_bv[SDE_CSC_BIAS_SIZE];
	uint32_t csc_pre_lv[SDE_CSC_CLAMP_SIZE];
	uint32_t csc_post_lv[SDE_CSC_CLAMP_SIZE];
};

/**
 * struct sde_mdss_color - mdss color description
 * color 0 : green
 * color 1 : blue
 * color 2 : red
 * color 3 : alpha
 */
struct sde_mdss_color {
	u32 color_0;
	u32 color_1;
	u32 color_2;
	u32 color_3;
};

/*
 * Define bit masks for h/w logging.
 */
#define SDE_DBG_MASK_NONE     (1 << 0)
#define SDE_DBG_MASK_CDM      (1 << 1)
#define SDE_DBG_MASK_DSPP     (1 << 2)
#define SDE_DBG_MASK_INTF     (1 << 3)
#define SDE_DBG_MASK_LM       (1 << 4)
#define SDE_DBG_MASK_CTL      (1 << 5)
#define SDE_DBG_MASK_PINGPONG (1 << 6)
#define SDE_DBG_MASK_SSPP     (1 << 7)
#define SDE_DBG_MASK_WB       (1 << 8)
#define SDE_DBG_MASK_TOP      (1 << 9)
#define SDE_DBG_MASK_VBIF     (1 << 10)
#define SDE_DBG_MASK_DSC      (1 << 11)
#define SDE_DBG_MASK_ROT      (1 << 12)
#define SDE_DBG_MASK_DS       (1 << 13)
#define SDE_DBG_MASK_REGDMA   (1 << 14)
#define SDE_DBG_MASK_UIDLE    (1 << 15)
#define SDE_DBG_MASK_SID      (1 << 15)
#define SDE_DBG_MASK_QDSS     (1 << 16)
#define SDE_DBG_MASK_VDC      (1 << 17)
#define SDE_DBG_MASK_DNSC_BLUR  (1 << 18)

/**
 * struct sde_cp_skip_blend_plane: skip blend plane payload
 * @valid: True when skip blend plane is active
 * @plane: hw plane being used
 * @plane_w: width of layer
 * @plane_h: height of layer
 */
struct sde_cp_skip_blend_plane {
	bool valid;
	enum sde_sspp plane;
	u32 plane_w;
	u32 plane_h;
};
/**
 * enum skip_blend_plane_type: skip blend rect type.
 * SB_PLANE_REAL - Rect0 or real plane
 * SB_PLANE_VIRT - Rect1 or virtual plane
 */
enum skip_blend_plane_type {
	SB_PLANE_REAL,
	SB_PLANE_VIRT,
	SB_PLANE_MAX
};

/**
 * struct sde_hw_cp_cfg: hardware dspp/lm feature payload.
 * @payload: Feature specific payload.
 * @len: Length of the payload.
 * @ctl: control pointer associated with dspp/lm.
 * @last_feature: last feature that will be set.
 * @num_of_mixers: number of layer mixers for the display.
 * @mixer_info: mixer info pointer associated with lm.
 * @displayv: height of the display.
 * @displayh: width of the display.
 * @dspp[DSPP_MAX]: array of hw_dspp pointers associated with crtc.
 * @broadcast_disabled: flag indicating if broadcast should be avoided when
 *			using LUTDMA
 * @panel_height: height of display panel in pixels.
 * @panel_width: width of display panel in pixels.
 * @skip_planes: array of skip blend planes with crtc
 * @num_ds_enabled: Number of destination scalers enabled
 * @is_crtc_enabled: true if crtc is enabled
 * @overfetch_lines_on_top: extra lines to over fetch on top
 */
struct sde_hw_cp_cfg {
	void *payload;
	u32 len;
	void *ctl;
	u32 last_feature;
	u32 num_of_mixers;
	void *mixer_info;
	u32 displayv;
	u32 displayh;
	struct sde_hw_dspp *dspp[DSPP_MAX];
	bool broadcast_disabled;
	u32 panel_height;
	u32 panel_width;
	struct sde_cp_skip_blend_plane skip_planes[SB_PLANE_MAX];
	u32 num_ds_enabled;
	bool is_crtc_enabled;
	u32 overfetch_lines_on_top;
};

/**
 * struct sde_hw_dim_layer: dim layer configs
 * @flags: Flag to represent INCLUSIVE/EXCLUSIVE
 * @stage: Blending stage of dim layer
 * @color_fill: Color fill to be used for the layer
 * @rect: Dim layer coordinates
 */
struct sde_hw_dim_layer {
	uint32_t flags;
	uint32_t stage;
	struct sde_mdss_color color_fill;
	struct sde_rect rect;
};

/**
 * struct sde_splash_mem - Struct contains splah memory info
 * @splash_buf_size: Indicates the size of the memory region
 * @splash_buf_base: Address of specific splash memory region
 * @ramdump_size: Size of ramdump buffer region
 * @ramdump_base: Address of ramdump region reserved by bootloader
 * @ref_cnt:	Tracks the map count to help in sharing splash memory
 */
struct sde_splash_mem {
	u32 splash_buf_size;
	unsigned long splash_buf_base;
	u32 ramdump_size;
	unsigned long ramdump_base;
	u32 ref_cnt;
};

/**
 * struct sde_sspp_index_info - Struct informing which pipes are staged on
 * particular display
 * @pipes:      bitmap, bit index is true if rect_0 of that pipe is staged,
 *              else is false
 * @virt_pipes: bitmap, bit index is true if rect_1 of that pipe is staged,
 *              else set to false
 * @bordercolor: True if border color is enabled
 */
struct sde_sspp_index_info {
	DECLARE_BITMAP(pipes, SSPP_MAX);
	DECLARE_BITMAP(virt_pipes, SSPP_MAX);
	bool bordercolor;
};

/**
 * SDE_SSPP_RECT_SOLO - multirect disabled
 * SDE_SSPP_RECT_0 - rect0 of a multirect pipe
 * SDE_SSPP_RECT_1 - rect1 of a multirect pipe
 * SDE_SSPP_RECT_MAX - max enum of multirect pipe
 *
 * Note: HW supports multirect with either RECT0 or
 * RECT1. Considering no benefit of such configs over
 * SOLO mode and to keep the plane management simple,
 * we dont support single rect multirect configs.
 */
enum sde_sspp_multirect_index {
	SDE_SSPP_RECT_SOLO = 0,
	SDE_SSPP_RECT_0,
	SDE_SSPP_RECT_1,
	SDE_SSPP_RECT_MAX,
};

/**
 * struct sde_hw_stage_cfg - blending stage cfg
 * @stage : SSPP_ID at each stage
 * @multirect_index: index of the rectangle of SSPP.
 * @layout: indicates if its the left or right layout.
 */
struct sde_hw_stage_cfg {
	enum sde_sspp stage[SDE_STAGE_MAX][PIPES_PER_STAGE];
	enum sde_sspp_multirect_index multirect_index[SDE_STAGE_MAX][PIPES_PER_STAGE];
	u32 layout[SDE_STAGE_MAX][PIPES_PER_STAGE];
};

/**
 * struct sde_splash_data - Struct contains details of resources and hw blocks
 * used in continuous splash on a specific display.
 * @cont_splash_enabled:  Stores the cont_splash status (enabled/disabled)
 * @encoder:	Pointer to the drm encoder object used for this display
 * @splash:	Pointer to struct sde_splash_mem used for this display
 * @demura:	Pointer to struct sde_splash_mem used for demura cont splash
 * @ctl_ids:	Stores the valid MDSS ctl block ids for the current mode
 * @lm_ids:	Stores the valid MDSS layer mixer block ids for the current mode
 * @dsc_ids:	Stores the valid MDSS DSC block ids for the current mode
 * @vdc_ids:	Stores the valid MDSS VDC block ids for the current mode
 * @pipes:      Array of sspp info detected on this display
 * @ctl_cnt:    Stores the active number of MDSS "top" blks of the current mode
 * @lm_cnt:	Stores the active number of MDSS "LM" blks for the current mode
 * @dsc_cnt:	Stores the active number of MDSS "dsc" blks for the current mode
 * @vdc_cnt:	Stores the valid MDSS VDC block ids for the current mode
 */
struct sde_splash_display {
	bool cont_splash_enabled;
	struct drm_encoder *encoder;
	struct sde_splash_mem *splash;
	struct sde_splash_mem *demura;
	u8 ctl_ids[MAX_DATA_PATH_PER_DSIPLAY];
	u8 lm_ids[MAX_DATA_PATH_PER_DSIPLAY];
	u8 dsc_ids[MAX_DATA_PATH_PER_DSIPLAY];
	u8 vdc_ids[MAX_DATA_PATH_PER_DSIPLAY];
	struct sde_sspp_index_info pipe_info;
	u8 ctl_cnt;
	u8 lm_cnt;
	u8 dsc_cnt;
	u8 vdc_cnt;
};

enum sde_handoff_type {
	SDE_SPLASH_HANDOFF,
	SDE_VM_HANDOFF,
};

/**
 * struct sde_splash_data - Struct contains details of continuous splash
 *	for all the displays connected by probe time
 * @type:                Indicates the type of handoff
 * @num_splash_regions:  Indicates number of splash memory regions from dtsi
 * @num_splash_displays: Indicates count of active displays in continuous splash
 * @splash_mem:          Array of all struct sde_splash_mem listed from dtsi
 * @demura_mem:          Array of all demura memory regions listed from dtsi
 * @splash_display:      Array of all struct sde_splash_display
 */
struct sde_splash_data {
	enum sde_handoff_type  type;
	u32 num_splash_regions;
	u32 num_splash_displays;
	struct sde_splash_mem splash_mem[MAX_DSI_DISPLAYS];
	struct sde_splash_mem demura_mem[MAX_DSI_DISPLAYS];
	struct sde_splash_display splash_display[MAX_DSI_DISPLAYS];
};

/**
 * struct sde_hw_tear_check - Struct contains parameters to configure
 *	tear-effect module. This structure is used to configure tear-check
 *	logic present either in ping-pong or in interface module.
 * @vsync_count:	Ratio of MDP VSYNC clk freq(Hz) to refresh rate divided
 *                      by no of lines
 * @sync_cfg_height:	Total vertical lines (display height - 1)
 * @vsync_init_val:	Init value to which the read pointer gets loaded at
 *                      vsync edge
 * @sync_threshold_start: Read pointer threshold start ROI for write operation
 * @sync_threshold_continue: The minimum number of lines the write pointer
 *                           needs to be above the read pointer
 * @start_pos:	The position from which the start_threshold value is added
 * @rd_ptr_irq:	The read pointer line at which interrupt has to be generated
 * @wr_ptr_irq:	The write pointer line at which interrupt has to be generated
 * @hw_vsync_mode:	Sync with external frame sync input
 * @detect_ctrl:	Tearing Detection Control is set to default configuration
 */
struct sde_hw_tear_check {
	u32 vsync_count;
	u32 sync_cfg_height;
	u32 vsync_init_val;
	u32 sync_threshold_start;
	u32 sync_threshold_continue;
	u32 start_pos;
	u32 rd_ptr_irq;
	u32 wr_ptr_irq;
	u8 hw_vsync_mode;
	u32 detect_ctrl;
};

/**
 * struct sde_hw_autorefresh - Struct contains parameters to configure
 *            auto-refresh mode for command mode panels
 * @enable:	Enalbe or disable the auto-refresh mode
 * @frame_count:	Auto-refresh frame counter at which update occurs
 */
struct sde_hw_autorefresh {
	bool  enable;
	u32 frame_count;
};

/**
 * struct sde_hw_pp_vsync_info - Struct contains parameters to configure
 *        read and write pointers for command mode panels
 * @pp_idx:		Ping-pong block index
 * @intf_idx:		Interface block index
 * @rd_ptr_init_val:	Value of rd pointer at vsync edge
 * @rd_ptr_frame_count:	num frames sent since enabling interface
 * @rd_ptr_line_count:	current line on panel (rd ptr)
 * @wr_ptr_line_count:	current line within pp fifo (wr ptr)
 * @intf_frame_count:	num frames read from intf
 */
struct sde_hw_pp_vsync_info {
	u32 pp_idx;
	u32 intf_idx;
	u32 rd_ptr_init_val;
	u32 rd_ptr_frame_count;
	u32 rd_ptr_line_count;
	u32 wr_ptr_line_count;
	u32 intf_frame_count;
};

/**
 * struct sde_hw_noise_layer_cfg: Payload to enable/disable noise blend
 * @flags: operation control flags, for future use
 * @noise_blend_stage: blend stage required for noise layer
 * @attn_blend_stage: blend stage required for attn layer
 * @attn_factor: factor in range of 1 to 255
 * @stength: strength in range of 0 to 6
 * @alpha_noise: factor in range of 1 to 255
*/
struct sde_hw_noise_layer_cfg {
	u64 flags;
	u32 noise_blend_stage;
	u32 attn_blend_stage;
	u32 attn_factor;
	u32 strength;
	u32 alpha_noise;
};
#endif  /* _SDE_HW_MDSS_H */
