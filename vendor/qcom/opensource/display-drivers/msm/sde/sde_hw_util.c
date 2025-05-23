// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 */
#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__

#include <drm/sde_drm.h>
#include "msm_drv.h"
#include "sde_kms.h"
#include "sde_hw_mdss.h"
#include "sde_hw_util.h"

/* using a file static variables for debugfs access */
static u32 sde_hw_util_log_mask = SDE_DBG_MASK_NONE;

/* SDE_SCALER_QSEED3 */
#define QSEED3_HW_VERSION                  0x00
#define QSEED3_OP_MODE                     0x04
#define QSEED3_RGB2Y_COEFF                 0x08
#define QSEED3_PHASE_INIT                  0x0C
#define QSEED3_PHASE_STEP_Y_H              0x10
#define QSEED3_PHASE_STEP_Y_V              0x14
#define QSEED3_PHASE_STEP_UV_H             0x18
#define QSEED3_PHASE_STEP_UV_V             0x1C
#define QSEED3_PRELOAD                     0x20
#define QSEED3_DE_SHARPEN                  0x24
#define QSEED3_DE_SHARPEN_CTL              0x28
#define QSEED3_DE_SHAPE_CTL                0x2C
#define QSEED3_DE_THRESHOLD                0x30
#define QSEED3_DE_ADJUST_DATA_0            0x34
#define QSEED3_DE_ADJUST_DATA_1            0x38
#define QSEED3_DE_ADJUST_DATA_2            0x3C
#define QSEED3_DE_LPF_BLEND                0x64
#define QSEED3_SRC_SIZE_Y_RGB_A            0x40
#define QSEED3_SRC_SIZE_UV                 0x44
#define QSEED3_DST_SIZE                    0x48
#define QSEED3_COEF_LUT_CTRL               0x4C
#define QSEED3_COEF_LUT_SWAP_BIT           0
#define QSEED3_BUFFER_CTRL                 0x50
#define QSEED3_CLK_CTRL0                   0x54
#define QSEED3_CLK_CTRL1                   0x58
#define QSEED3_CLK_STATUS                  0x5C
#define QSEED3_MISR_CTRL                   0x70
#define QSEED3_MISR_SIGNATURE_0            0x74
#define QSEED3_MISR_SIGNATURE_1            0x78
#define QSEED3_PHASE_INIT_Y_H              0x90
#define QSEED3_PHASE_INIT_Y_V              0x94
#define QSEED3_PHASE_INIT_UV_H             0x98
#define QSEED3_PHASE_INIT_UV_V             0x9C
#define QSEED3_ENABLE                      2
#define CSC_MATRIX_SHIFT                   7

/* SDE_SCALER_QSEED3LITE */
#define QSEED3L_COEF_LUT_Y_SEP_BIT         4
#define QSEED3L_COEF_LUT_UV_SEP_BIT        5
#define QSEED3L_COEF_LUT_CTRL              0x4C
#define QSEED3L_COEF_LUT_SWAP_BIT          0
#define QSEED3L_DIR_FILTER_WEIGHT          0x60
#define QSEED3LITE_SCALER_VERSION          0x2004
#define QSEED4_SCALER_VERSION              0x3000

#define QSEED3_DEFAULT_PRELOAD_V 0x3
#define QSEED3_DEFAULT_PRELOAD_H 0x4

#define QSEED4_DEFAULT_PRELOAD_V 0x2
#define QSEED4_DEFAULT_PRELOAD_H 0x4

#define QSEED5_DEFAULT_DE_LPF_BLEND 0x3FF00000

/* SDE CAC SCALER */
#define QSEED3_CAC_RE_PRELOAD              0xA0
#define QSEED3_CAC_RE_PHASE_INIT_Y_V       0xA4
#define QSEED3_CAC_RE_PHASE_INIT_UV_V      0xA8
#define QSEED3_CAC_LE_PHASE_INIT2_Y_H      0xAC
#define QSEED3_CAC_LE_PHASE_INIT2_Y_V      0xB0
#define QSEED3_CAC_LE_PHASE_INIT2_UV_H     0xB4
#define QSEED3_CAC_LE_PHASE_INIT2_UV_V     0xB8
#define QSEED3_CAC_RE_PHASE_INIT2_Y_V      0xBC
#define QSEED3_CAC_RE_PHASE_INIT2_UV_V     0xC0
#define QSEED3_CAC_LE_Y                    0xC4
#define QSEED3_CAC_LE_UV                   0xC8
#define QSEED3_CAC_RE_Y                    0xCC
#define QSEED3_CAC_RE_UV                   0xD0
#define QSEED3_DST_UV_SIZE                 0xD4
#define QSEED3_DST_LE_OFFSET               0xD8
#define QSEED3_DST_RE_OFFSET               0xDC
#define QTIMER_DISABLE 0x1
#define QTIMER_ENABLE 0x2
#define QTMR_V1_CNTFRQ 0x10
#define QTMR_V1_CNTP_CTL 0x2c
#define QTMR_V1_CNTP_TVAL 0x28

typedef void (*scaler_lut_type)(struct sde_hw_blk_reg_map *,
		struct sde_hw_scaler3_cfg *, u32);

static irqreturn_t sde_qtimer_irq_cb(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct msm_drm_private *priv = dev->dev_private;
	struct sde_kms *sde_kms = to_sde_kms(priv->kms);

	SDE_EVT32(0);
	if (sde_kms->sde_qtimer.qtimer_cb)
		sde_qtimer_start(&sde_kms->sde_qtimer);

	return IRQ_HANDLED;
}

void msm_sde_qtimer_install(struct device *dev)
{
	int ret, count, irq;
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *ddev = platform_get_drvdata(pdev);
	struct msm_drm_private *priv = ddev->dev_private;
	struct sde_kms *sde_kms = to_sde_kms(priv->kms);
	struct resource *qtimer_res;
	unsigned long qtimer_reg_size;

	if (!pdev) {
		pr_err("pdev is NULL\n");
		return;
	}

	count = of_property_count_strings(pdev->dev.of_node, "interrupt-names");
	if (count <= 1) {
		pr_debug("Qtimer not defined\n");
		return;
	}

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		pr_debug("Fail to get Qtimer irq\n");
		return;
	}

	qtimer_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "qtimer_reg");
	qtimer_reg_size = resource_size(qtimer_res);
	sde_kms->sde_qtimer.qtimer_cb = sde_qtimer_irq_cb;

	ret = request_irq(irq, sde_kms->sde_qtimer.qtimer_cb,
	__IRQF_TIMER | IRQF_NO_THREAD, ddev->driver->name, ddev);
	enable_irq_wake(irq);
	sde_kms->sde_qtimer.qtimer_mmio = ioremap(qtimer_res->start, qtimer_reg_size);
}

void sde_qtimer_start(struct sde_qtimer *sde_qtimer)
{
	unsigned long q_cycles, q_freq;

	q_freq = readl_relaxed(sde_qtimer->qtimer_mmio+QTMR_V1_CNTFRQ);

	q_cycles = (q_freq * sde_qtimer->time_in_ns) / NSEC_PER_SEC;

	/* write 0x2 to QTMR_V1_CNTP_CTL to enable qtimer */
	writel_relaxed(QTIMER_ENABLE, sde_qtimer->qtimer_mmio+QTMR_V1_CNTP_CTL);

	/*
	 * Write q_cycles(tcxo) to QTMR_V1_CNTP_TVAL to invoke callback
	 * function at the interval of sde_qtimer->time_in_ns.
	 */
	writel_relaxed(q_cycles, sde_qtimer->qtimer_mmio+QTMR_V1_CNTP_TVAL);

	/* write 0x1 to QTMR_V1_CNTP_CTL to disable qtimer */
	writel_relaxed(QTIMER_DISABLE, sde_qtimer->qtimer_mmio+QTMR_V1_CNTP_CTL);
	SDE_EVT32(q_cycles>>32, q_cycles);

}

void sde_qtimer_stop(struct sde_qtimer *sde_qtimer)
{
	writel_relaxed(0x2, sde_qtimer->qtimer_mmio+0x2c);
	SDE_EVT32(0);
}

void sde_reg_write(struct sde_hw_blk_reg_map *c,
		u32 reg_off,
		u32 val,
		const char *name)
{
	/* don't need to mutex protect this */
	if (c->log_mask & sde_hw_util_log_mask)
		SDE_DEBUG_DRIVER("[%s:0x%X] <= 0x%X\n",
				name, c->blk_off + reg_off, val);
	writel_relaxed(val, c->base_off + c->blk_off + reg_off);
	SDE_REG_LOG(c->log_mask ? ilog2(c->log_mask)+1 : 0,
			val, c->blk_off + reg_off);
}

int sde_reg_read(struct sde_hw_blk_reg_map *c, u32 reg_off)
{
	return readl_relaxed(c->base_off + c->blk_off + reg_off);
}

u32 *sde_hw_util_get_log_mask_ptr(void)
{
	return &sde_hw_util_log_mask;
}

void sde_init_scaler_blk(struct sde_scaler_blk *blk, u32 version)
{
	if (!blk)
		return;

	blk->version = version;
	blk->v_preload = QSEED4_DEFAULT_PRELOAD_V;
	blk->h_preload = QSEED4_DEFAULT_PRELOAD_H;
	if (version < QSEED4_SCALER_VERSION) {
		blk->v_preload = QSEED3_DEFAULT_PRELOAD_V;
		blk->h_preload = QSEED3_DEFAULT_PRELOAD_H;
	}
}
void sde_set_scaler_v2(struct sde_hw_scaler3_cfg *cfg,
		const struct sde_drm_scaler_v2 *scale_v2)
{
	int i;

	cfg->enable = scale_v2->enable;
	cfg->dir_en = scale_v2->dir_en;
	cfg->dir45_en = scale_v2->dir45_en;
	cfg->cor_en = scale_v2->cor_en;

	for (i = 0; i < SDE_MAX_PLANES; i++) {
		cfg->init_phase_x[i] = scale_v2->init_phase_x[i];
		cfg->phase_step_x[i] = scale_v2->phase_step_x[i];
		cfg->init_phase_y[i] = scale_v2->init_phase_y[i];
		cfg->phase_step_y[i] = scale_v2->phase_step_y[i];

		cfg->preload_x[i] = scale_v2->preload_x[i];
		cfg->preload_y[i] = scale_v2->preload_y[i];
		cfg->src_width[i] = scale_v2->src_width[i];
		cfg->src_height[i] = scale_v2->src_height[i];
	}

	cfg->dst_width = scale_v2->dst_width;
	cfg->dst_height = scale_v2->dst_height;

	cfg->y_rgb_filter_cfg = scale_v2->y_rgb_filter_cfg;
	cfg->uv_filter_cfg = scale_v2->uv_filter_cfg;
	cfg->alpha_filter_cfg = scale_v2->alpha_filter_cfg;
	cfg->blend_cfg = scale_v2->blend_cfg;

	cfg->lut_flag = scale_v2->lut_flag;
	cfg->dir_lut_idx = scale_v2->dir_lut_idx;
	cfg->y_rgb_cir_lut_idx = scale_v2->y_rgb_cir_lut_idx;
	cfg->uv_cir_lut_idx = scale_v2->uv_cir_lut_idx;
	cfg->y_rgb_sep_lut_idx = scale_v2->y_rgb_sep_lut_idx;
	cfg->uv_sep_lut_idx = scale_v2->uv_sep_lut_idx;
	cfg->de.prec_shift = scale_v2->de.prec_shift;
	cfg->dir_weight = scale_v2->dir_weight;
	cfg->dyn_exp_disabled = (scale_v2->flags & SDE_DYN_EXP_DISABLE) ? 1 : 0;

	cfg->de.enable = scale_v2->de.enable;
	cfg->de.sharpen_level1 = scale_v2->de.sharpen_level1;
	cfg->de.sharpen_level2 = scale_v2->de.sharpen_level2;
	cfg->de.clip = scale_v2->de.clip;
	cfg->de.limit = scale_v2->de.limit;
	cfg->de.thr_quiet = scale_v2->de.thr_quiet;
	cfg->de.thr_dieout = scale_v2->de.thr_dieout;
	cfg->de.thr_low = scale_v2->de.thr_low;
	cfg->de.thr_high = scale_v2->de.thr_high;
	cfg->de.blend = scale_v2->de_blend;
	cfg->de_lpf_flags = scale_v2->de_lpf_flags;
	cfg->de_lpf_h = scale_v2->de_lpf_h;
	cfg->de_lpf_l = scale_v2->de_lpf_l;
	cfg->de_lpf_m = scale_v2->de_lpf_m;

	for (i = 0; i < SDE_MAX_DE_CURVES; i++) {
		cfg->de.adjust_a[i] = scale_v2->de.adjust_a[i];
		cfg->de.adjust_b[i] = scale_v2->de.adjust_b[i];
		cfg->de.adjust_c[i] = scale_v2->de.adjust_c[i];
	}
}

static void _sde_hw_setup_scaler3_lut(struct sde_hw_blk_reg_map *c,
		struct sde_hw_scaler3_cfg *scaler3_cfg, u32 offset)
{
	int i, j, filter;
	int config_lut = 0x0;
	unsigned long lut_flags;
	u32 lut_addr, lut_offset, lut_len;
	u32 *lut[QSEED3_FILTERS] = {NULL, NULL, NULL, NULL, NULL};
	static const uint32_t off_tbl[QSEED3_FILTERS][QSEED3_LUT_REGIONS][2] = {
		{{18, 0x000}, {12, 0x120}, {12, 0x1E0}, {8, 0x2A0} },
		{{6, 0x320}, {3, 0x3E0}, {3, 0x440}, {3, 0x4A0} },
		{{6, 0x500}, {3, 0x5c0}, {3, 0x620}, {3, 0x680} },
		{{6, 0x380}, {3, 0x410}, {3, 0x470}, {3, 0x4d0} },
		{{6, 0x560}, {3, 0x5f0}, {3, 0x650}, {3, 0x6b0} },
	};

	lut_flags = (unsigned long) scaler3_cfg->lut_flag;
	if (test_bit(QSEED3_COEF_LUT_DIR_BIT, &lut_flags) &&
		(scaler3_cfg->dir_len == QSEED3_DIR_LUT_SIZE)) {
		lut[0] = scaler3_cfg->dir_lut;
		config_lut = 1;
	}
	if (test_bit(QSEED3_COEF_LUT_Y_CIR_BIT, &lut_flags) &&
		(scaler3_cfg->y_rgb_cir_lut_idx < QSEED3_CIRCULAR_LUTS) &&
		(scaler3_cfg->cir_len == QSEED3_CIR_LUT_SIZE)) {
		lut[1] = scaler3_cfg->cir_lut +
			scaler3_cfg->y_rgb_cir_lut_idx * QSEED3_LUT_SIZE;
		config_lut = 1;
	}
	if (test_bit(QSEED3_COEF_LUT_UV_CIR_BIT, &lut_flags) &&
		(scaler3_cfg->uv_cir_lut_idx < QSEED3_CIRCULAR_LUTS) &&
		(scaler3_cfg->cir_len == QSEED3_CIR_LUT_SIZE)) {
		lut[2] = scaler3_cfg->cir_lut +
			scaler3_cfg->uv_cir_lut_idx * QSEED3_LUT_SIZE;
		config_lut = 1;
	}
	if (test_bit(QSEED3_COEF_LUT_Y_SEP_BIT, &lut_flags) &&
		(scaler3_cfg->y_rgb_sep_lut_idx < QSEED3_SEPARABLE_LUTS) &&
		(scaler3_cfg->sep_len == QSEED3_SEP_LUT_SIZE)) {
		lut[3] = scaler3_cfg->sep_lut +
			scaler3_cfg->y_rgb_sep_lut_idx * QSEED3_LUT_SIZE;
		config_lut = 1;
	}
	if (test_bit(QSEED3_COEF_LUT_UV_SEP_BIT, &lut_flags) &&
		(scaler3_cfg->uv_sep_lut_idx < QSEED3_SEPARABLE_LUTS) &&
		(scaler3_cfg->sep_len == QSEED3_SEP_LUT_SIZE)) {
		lut[4] = scaler3_cfg->sep_lut +
			scaler3_cfg->uv_sep_lut_idx * QSEED3_LUT_SIZE;
		config_lut = 1;
	}

	if (config_lut) {
		for (filter = 0; filter < QSEED3_FILTERS; filter++) {
			if (!lut[filter])
				continue;
			lut_offset = 0;
			for (i = 0; i < QSEED3_LUT_REGIONS; i++) {
				lut_addr = QSEED3_COEF_LUT_OFF + offset
					+ off_tbl[filter][i][1];
				lut_len = off_tbl[filter][i][0] << 2;
				for (j = 0; j < lut_len; j++) {
					SDE_REG_WRITE(c,
						lut_addr,
						(lut[filter])[lut_offset++]);
					lut_addr += 4;
				}
			}
		}
	}

	if (test_bit(QSEED3_COEF_LUT_SWAP_BIT, &lut_flags))
		SDE_REG_WRITE(c, QSEED3_COEF_LUT_CTRL + offset, BIT(0));

}

static void _sde_hw_setup_scaler3lite_lut(struct sde_hw_blk_reg_map *c,
		struct sde_hw_scaler3_cfg *scaler3_cfg, u32 offset)
{
	int i, filter;
	int config_lut = 0x0;
	unsigned long lut_flags;
	u32 lut_addr, lut_offset;
	u32 *lut[QSEED3LITE_FILTERS] = {NULL, NULL};
	static const uint32_t off_tbl[QSEED3LITE_FILTERS] = {0x000, 0x200};

	SDE_REG_WRITE(c, QSEED3L_DIR_FILTER_WEIGHT + offset,
			scaler3_cfg->dir_weight & 0xFF);

	/* destination scaler case */
	if (!scaler3_cfg->sep_lut)
		return;

	lut_flags = (unsigned long) scaler3_cfg->lut_flag;
	if (test_bit(QSEED3L_COEF_LUT_Y_SEP_BIT, &lut_flags) &&
		(scaler3_cfg->y_rgb_sep_lut_idx < QSEED3L_SEPARABLE_LUTS) &&
		(scaler3_cfg->sep_len == QSEED3L_SEP_LUT_SIZE)) {
		lut[0] = scaler3_cfg->sep_lut +
			scaler3_cfg->y_rgb_sep_lut_idx * QSEED3L_LUT_SIZE;
		config_lut = 1;
	}
	if (test_bit(QSEED3L_COEF_LUT_UV_SEP_BIT, &lut_flags) &&
		(scaler3_cfg->uv_sep_lut_idx < QSEED3L_SEPARABLE_LUTS) &&
		(scaler3_cfg->sep_len == QSEED3L_SEP_LUT_SIZE)) {
		lut[1] = scaler3_cfg->sep_lut +
			scaler3_cfg->uv_sep_lut_idx * QSEED3L_LUT_SIZE;
		config_lut = 1;
	}

	if (config_lut) {
		for (filter = 0; filter < QSEED3LITE_FILTERS; filter++) {
			if (!lut[filter])
				continue;
			lut_offset = 0;
			lut_addr = QSEED3L_COEF_LUT_OFF + offset +
				off_tbl[filter];
			for (i = 0; i < QSEED3L_LUT_SIZE; i++) {
				SDE_REG_WRITE(c, lut_addr,
						(lut[filter])[lut_offset++]);
				lut_addr += 4;
			}
		}
	}

	if (test_bit(QSEED3L_COEF_LUT_SWAP_BIT, &lut_flags))
		SDE_REG_WRITE(c, QSEED3L_COEF_LUT_CTRL + offset, BIT(0));
}

static void _sde_hw_setup_scaler3_de(struct sde_hw_blk_reg_map *c,
		struct sde_hw_scaler3_de_cfg *de_cfg, u32 offset)
{
	u32 sharp_lvl, sharp_ctl, shape_ctl, de_thr;
	u32 adjust_a, adjust_b, adjust_c;

	if (!de_cfg->enable)
		return;

	sharp_lvl = (de_cfg->sharpen_level1 & 0x1FF) |
		((de_cfg->sharpen_level2 & 0x1FF) << 16);

	sharp_ctl = ((de_cfg->limit & 0xF) << 9) |
		((de_cfg->prec_shift & 0x7) << 13) |
		((de_cfg->clip & 0x7) << 16) |
		((de_cfg->blend & 0xF) << 20);

	shape_ctl = (de_cfg->thr_quiet & 0xFF) |
		((de_cfg->thr_dieout & 0x3FF) << 16);

	de_thr = (de_cfg->thr_low & 0x3FF) |
		((de_cfg->thr_high & 0x3FF) << 16);

	adjust_a = (de_cfg->adjust_a[0] & 0x3FF) |
		((de_cfg->adjust_a[1] & 0x3FF) << 10) |
		((de_cfg->adjust_a[2] & 0x3FF) << 20);

	adjust_b = (de_cfg->adjust_b[0] & 0x3FF) |
		((de_cfg->adjust_b[1] & 0x3FF) << 10) |
		((de_cfg->adjust_b[2] & 0x3FF) << 20);

	adjust_c = (de_cfg->adjust_c[0] & 0x3FF) |
		((de_cfg->adjust_c[1] & 0x3FF) << 10) |
		((de_cfg->adjust_c[2] & 0x3FF) << 20);

	SDE_REG_WRITE(c, QSEED3_DE_SHARPEN + offset, sharp_lvl);
	SDE_REG_WRITE(c, QSEED3_DE_SHARPEN_CTL + offset, sharp_ctl);
	SDE_REG_WRITE(c, QSEED3_DE_SHAPE_CTL + offset, shape_ctl);
	SDE_REG_WRITE(c, QSEED3_DE_THRESHOLD + offset, de_thr);
	SDE_REG_WRITE(c, QSEED3_DE_ADJUST_DATA_0 + offset, adjust_a);
	SDE_REG_WRITE(c, QSEED3_DE_ADJUST_DATA_1 + offset, adjust_b);
	SDE_REG_WRITE(c, QSEED3_DE_ADJUST_DATA_2 + offset, adjust_c);

}

static inline scaler_lut_type get_scaler_lut(
		struct sde_hw_scaler3_cfg *scaler3_cfg, u32 scaler_version)
{
	scaler_lut_type lut_ptr = _sde_hw_setup_scaler3lite_lut;

	if (!(scaler3_cfg->lut_flag))
		return NULL;

	if (scaler_version < QSEED3LITE_SCALER_VERSION)
		lut_ptr = _sde_hw_setup_scaler3_lut;

	return lut_ptr;
}

void sde_hw_setup_scaler_cac(struct sde_hw_blk_reg_map *c,
		u32 sspp_blk_off, struct sde_hw_cac_cfg *cac_cfg)
{
	u32 phase_step_y_h, phase_step_y_v, phase_step_uv_h, phase_step_uv_v;
	u32 preload_re, thresh_le_y, thresh_le_uv;
	u32 dst_uv, dst_le_off, opmode;

	phase_step_y_h = SDE_REG_READ(c, QSEED3_PHASE_STEP_Y_H +
						sspp_blk_off);
	phase_step_y_v = SDE_REG_READ(c, QSEED3_PHASE_STEP_Y_V +
						sspp_blk_off);
	phase_step_uv_h = SDE_REG_READ(c, QSEED3_PHASE_STEP_UV_H +
						sspp_blk_off);
	phase_step_uv_v = SDE_REG_READ(c, QSEED3_PHASE_STEP_UV_V +
						sspp_blk_off);
	opmode = SDE_REG_READ(c, QSEED3_OP_MODE + sspp_blk_off);

	opmode |= (cac_cfg->cac_mode << 1);
	opmode |= (cac_cfg->uv_filter_cfg & 0x3) << 24;

	phase_step_y_h |= (cac_cfg->cac_le_inc_skip_x[0] << 29) |
			(cac_cfg->cac_phase_inc_first_x[0] << 28);

	phase_step_y_v |= (cac_cfg->cac_le_inc_skip_y[0] << 29) |
			(cac_cfg->cac_phase_inc_first_y[0] << 28) |
			(cac_cfg->cac_re_inc_skip_y[0] << 30);

	phase_step_uv_h |= (cac_cfg->cac_le_inc_skip_x[1] << 29) |
			(cac_cfg->cac_phase_inc_first_x[1] << 28);

	phase_step_uv_v |= (cac_cfg->cac_le_inc_skip_y[1] << 29) |
			(cac_cfg->cac_phase_inc_first_y[1] << 28) |
			(cac_cfg->cac_re_inc_skip_y[1] << 30);

	preload_re = ((cac_cfg->cac_re_preload_y[1] & 0x7F) << 24) |
			((cac_cfg->cac_re_preload_y[0] & 0x7F) << 8);

	thresh_le_y = ((cac_cfg->cac_le_thr_y[0] & 0xFFFF) << 16) |
			(cac_cfg->cac_le_thr_x[0] & 0xFFFF);

	thresh_le_uv = ((cac_cfg->cac_le_thr_y[1] & 0xFFFF) << 16) |
			(cac_cfg->cac_le_thr_x[1] & 0xFFFF);

	dst_uv = ((cac_cfg->cac_dst_uv_h & 0xFFFF) << 16) |
			(cac_cfg->cac_dst_uv_w & 0xFFFF);

	dst_le_off = ((cac_cfg->cac_le_dst_v_offset & 0xFFFF) << 16) |
			(cac_cfg->cac_le_dst_h_offset & 0xFFFF);

	SDE_REG_WRITE(c, QSEED3_CAC_RE_PRELOAD + sspp_blk_off, preload_re);
	SDE_REG_WRITE(c, QSEED3_CAC_RE_PHASE_INIT_Y_V + sspp_blk_off,
			(cac_cfg->cac_re_phase_init_y[0] & 0x1FFFFF));
	SDE_REG_WRITE(c, QSEED3_CAC_RE_PHASE_INIT_UV_V + sspp_blk_off,
			(cac_cfg->cac_re_phase_init_y[1] & 0x1FFFFF));
	SDE_REG_WRITE(c, QSEED3_CAC_LE_PHASE_INIT2_Y_H + sspp_blk_off,
			cac_cfg->cac_le_phase_init2_x[0]);
	SDE_REG_WRITE(c, QSEED3_CAC_LE_PHASE_INIT2_UV_H + sspp_blk_off,
			cac_cfg->cac_le_phase_init2_x[1]);
	SDE_REG_WRITE(c, QSEED3_CAC_LE_PHASE_INIT2_Y_V + sspp_blk_off,
			cac_cfg->cac_le_phase_init2_y[0]);
	SDE_REG_WRITE(c, QSEED3_CAC_LE_PHASE_INIT2_UV_V + sspp_blk_off,
			cac_cfg->cac_le_phase_init2_y[1]);
	SDE_REG_WRITE(c, QSEED3_CAC_RE_PHASE_INIT2_Y_V + sspp_blk_off,
			cac_cfg->cac_re_phase_init2_y[0]);
	SDE_REG_WRITE(c, QSEED3_CAC_RE_PHASE_INIT2_UV_V + sspp_blk_off,
			cac_cfg->cac_re_phase_init2_y[1]);
	SDE_REG_WRITE(c, QSEED3_CAC_LE_Y + sspp_blk_off, thresh_le_y);
	SDE_REG_WRITE(c, QSEED3_CAC_LE_UV + sspp_blk_off, thresh_le_uv);
	SDE_REG_WRITE(c, QSEED3_CAC_RE_Y + sspp_blk_off,
			((cac_cfg->cac_re_thr_y[0] & 0xFFFF) << 16));
	SDE_REG_WRITE(c, QSEED3_CAC_RE_UV + sspp_blk_off,
			((cac_cfg->cac_re_thr_y[1] & 0xFFFF) << 16));
	SDE_REG_WRITE(c, QSEED3_DST_UV_SIZE + sspp_blk_off, dst_uv);
	SDE_REG_WRITE(c, QSEED3_DST_LE_OFFSET + sspp_blk_off, dst_le_off);
	SDE_REG_WRITE(c, QSEED3_DST_RE_OFFSET + sspp_blk_off,
			((cac_cfg->cac_re_dst_v_offset & 0xFFFF) << 16));
	SDE_REG_WRITE(c, QSEED3_PHASE_STEP_Y_H + sspp_blk_off, phase_step_y_h);
	SDE_REG_WRITE(c, QSEED3_PHASE_STEP_Y_V + sspp_blk_off, phase_step_y_v);
	SDE_REG_WRITE(c, QSEED3_PHASE_STEP_UV_H + sspp_blk_off,
				phase_step_uv_h);
	SDE_REG_WRITE(c, QSEED3_PHASE_STEP_UV_V + sspp_blk_off,
				phase_step_uv_v);
	SDE_REG_WRITE(c, QSEED3_OP_MODE + sspp_blk_off, opmode);
}

void sde_hw_setup_scaler3(struct sde_hw_blk_reg_map *c,
		struct sde_hw_scaler3_cfg *scaler3_cfg, u32 scaler_version,
		u32 scaler_offset, const struct sde_format *format, bool de_lpf)
{
	u32 op_mode = 0;
	u32 phase_init, preload, src_y_rgb, src_uv, dst;
	scaler_lut_type setup_lut = NULL;
	u32 de_lpf_blend = 0;

	if (!scaler3_cfg->enable)
		goto end;

	op_mode |= BIT(0);
	op_mode |= (scaler3_cfg->y_rgb_filter_cfg & 0x3) << 16;

	if (format && SDE_FORMAT_IS_YUV(format)) {
		op_mode |= BIT(12);
		op_mode |= (scaler3_cfg->uv_filter_cfg & 0x3) << 24;
	}

	op_mode |= (scaler3_cfg->blend_cfg & 1) << 31;
	op_mode |= (scaler3_cfg->dir_en) ? BIT(4) : 0;
	op_mode |= (scaler3_cfg->dir_en && scaler3_cfg->cor_en) ? BIT(5) : 0;
	op_mode |= (scaler3_cfg->dir_en && scaler3_cfg->dir45_en) ? BIT(6) : 0;
	op_mode |= (scaler3_cfg->dyn_exp_disabled) ? BIT(13) : 0;

	preload =
		((scaler3_cfg->preload_x[0] & 0x7F) << 0) |
		((scaler3_cfg->preload_y[0] & 0x7F) << 8) |
		((scaler3_cfg->preload_x[1] & 0x7F) << 16) |
		((scaler3_cfg->preload_y[1] & 0x7F) << 24);

	src_y_rgb = (scaler3_cfg->src_width[0] & 0x1FFFF) |
		((scaler3_cfg->src_height[0] & 0x1FFFF) << 16);

	src_uv = (scaler3_cfg->src_width[1] & 0x1FFFF) |
		((scaler3_cfg->src_height[1] & 0x1FFFF) << 16);

	dst = (scaler3_cfg->dst_width & 0x1FFFF) |
		((scaler3_cfg->dst_height & 0x1FFFF) << 16);

	if (scaler3_cfg->de.enable) {
		_sde_hw_setup_scaler3_de(c, &scaler3_cfg->de, scaler_offset);
		op_mode |= BIT(8);
	}

	setup_lut = get_scaler_lut(scaler3_cfg, scaler_version);
	if (setup_lut)
		setup_lut(c, scaler3_cfg, scaler_offset);

	if (scaler_version == 0x1002) {
		phase_init =
			((scaler3_cfg->init_phase_x[0] & 0x3F) << 0) |
			((scaler3_cfg->init_phase_y[0] & 0x3F) << 8) |
			((scaler3_cfg->init_phase_x[1] & 0x3F) << 16) |
			((scaler3_cfg->init_phase_y[1] & 0x3F) << 24);
		SDE_REG_WRITE(c, QSEED3_PHASE_INIT + scaler_offset, phase_init);
	} else {
		SDE_REG_WRITE(c, QSEED3_PHASE_INIT_Y_H + scaler_offset,
			scaler3_cfg->init_phase_x[0] & 0x1FFFFF);
		SDE_REG_WRITE(c, QSEED3_PHASE_INIT_Y_V + scaler_offset,
			scaler3_cfg->init_phase_y[0] & 0x1FFFFF);
		SDE_REG_WRITE(c, QSEED3_PHASE_INIT_UV_H + scaler_offset,
			scaler3_cfg->init_phase_x[1] & 0x1FFFFF);
		SDE_REG_WRITE(c, QSEED3_PHASE_INIT_UV_V + scaler_offset,
			scaler3_cfg->init_phase_y[1] & 0x1FFFFF);
	}

	SDE_REG_WRITE(c, QSEED3_PHASE_STEP_Y_H + scaler_offset,
		scaler3_cfg->phase_step_x[0] & 0xFFFFFF);

	SDE_REG_WRITE(c, QSEED3_PHASE_STEP_Y_V + scaler_offset,
		scaler3_cfg->phase_step_y[0] & 0xFFFFFF);

	SDE_REG_WRITE(c, QSEED3_PHASE_STEP_UV_H + scaler_offset,
		scaler3_cfg->phase_step_x[1] & 0xFFFFFF);

	SDE_REG_WRITE(c, QSEED3_PHASE_STEP_UV_V + scaler_offset,
		scaler3_cfg->phase_step_y[1] & 0xFFFFFF);

	SDE_REG_WRITE(c, QSEED3_PRELOAD + scaler_offset, preload);

	SDE_REG_WRITE(c, QSEED3_SRC_SIZE_Y_RGB_A + scaler_offset, src_y_rgb);

	SDE_REG_WRITE(c, QSEED3_SRC_SIZE_UV + scaler_offset, src_uv);

	SDE_REG_WRITE(c, QSEED3_DST_SIZE + scaler_offset, dst);

	if (de_lpf) {
		if (scaler3_cfg->de_lpf_flags & SDE_DE_LPF_BLEND_FLAG_EN) {
			de_lpf_blend = (scaler3_cfg->de_lpf_l & 0x3FF) |
				((scaler3_cfg->de_lpf_m & 0x3FF) << 10) |
				((scaler3_cfg->de_lpf_h & 0x3FF) << 20);
			SDE_REG_WRITE(c, QSEED3_DE_LPF_BLEND +
						scaler_offset, de_lpf_blend);
		} else {
			SDE_REG_WRITE(c, QSEED3_DE_LPF_BLEND + scaler_offset,
						QSEED5_DEFAULT_DE_LPF_BLEND);
		}
	}

end:
	if (format && !SDE_FORMAT_IS_DX(format))
		op_mode |= BIT(14);

	if (format && format->alpha_enable) {
		op_mode |= BIT(10);
		if (scaler_version == 0x1002)
			op_mode |= (scaler3_cfg->alpha_filter_cfg & 0x1) << 30;
		else
			op_mode |= (scaler3_cfg->alpha_filter_cfg & 0x3) << 29;
	}

	SDE_REG_WRITE(c, QSEED3_OP_MODE + scaler_offset, op_mode);

}

void sde_hw_csc_matrix_coeff_setup(struct sde_hw_blk_reg_map *c,
		u32 csc_reg_off, struct sde_csc_cfg *data,
		u32 shift_bit)
{
	u32 val;

	if (!c || !data)
		return;

	val = ((data->csc_mv[0] >> shift_bit) & 0x1FFF) |
		(((data->csc_mv[1] >> shift_bit) & 0x1FFF) << 16);
	SDE_REG_WRITE(c, csc_reg_off, val);
	val = ((data->csc_mv[2] >> shift_bit) & 0x1FFF) |
		(((data->csc_mv[3] >> shift_bit) & 0x1FFF) << 16);
	SDE_REG_WRITE(c, csc_reg_off + 0x4, val);
	val = ((data->csc_mv[4] >> shift_bit) & 0x1FFF) |
		(((data->csc_mv[5] >> shift_bit) & 0x1FFF) << 16);
	SDE_REG_WRITE(c, csc_reg_off + 0x8, val);
	val = ((data->csc_mv[6] >> shift_bit) & 0x1FFF) |
		(((data->csc_mv[7] >> shift_bit) & 0x1FFF) << 16);
	SDE_REG_WRITE(c, csc_reg_off + 0xc, val);
	val = (data->csc_mv[8] >> shift_bit) & 0x1FFF;
	SDE_REG_WRITE(c, csc_reg_off + 0x10, val);
}

void sde_hw_csc_setup(struct sde_hw_blk_reg_map *c,
		u32 csc_reg_off,
		struct sde_csc_cfg *data, bool csc10)
{
	u32 clamp_shift = csc10 ? 16 : 8;
	u32 val;

	if (!c || !data)
		return;

	/* matrix coeff - convert S15.16 to S4.9 */
	sde_hw_csc_matrix_coeff_setup(c, csc_reg_off, data, CSC_MATRIX_SHIFT);

	/* Pre clamp */
	val = (data->csc_pre_lv[0] << clamp_shift) | data->csc_pre_lv[1];
	SDE_REG_WRITE(c, csc_reg_off + 0x14, val);
	val = (data->csc_pre_lv[2] << clamp_shift) | data->csc_pre_lv[3];
	SDE_REG_WRITE(c, csc_reg_off + 0x18, val);
	val = (data->csc_pre_lv[4] << clamp_shift) | data->csc_pre_lv[5];
	SDE_REG_WRITE(c, csc_reg_off + 0x1c, val);

	/* Post clamp */
	val = (data->csc_post_lv[0] << clamp_shift) | data->csc_post_lv[1];
	SDE_REG_WRITE(c, csc_reg_off + 0x20, val);
	val = (data->csc_post_lv[2] << clamp_shift) | data->csc_post_lv[3];
	SDE_REG_WRITE(c, csc_reg_off + 0x24, val);
	val = (data->csc_post_lv[4] << clamp_shift) | data->csc_post_lv[5];
	SDE_REG_WRITE(c, csc_reg_off + 0x28, val);

	/* Pre-Bias */
	SDE_REG_WRITE(c, csc_reg_off + 0x2c, data->csc_pre_bv[0]);
	SDE_REG_WRITE(c, csc_reg_off + 0x30, data->csc_pre_bv[1]);
	SDE_REG_WRITE(c, csc_reg_off + 0x34, data->csc_pre_bv[2]);

	/* Post-Bias */
	SDE_REG_WRITE(c, csc_reg_off + 0x38, data->csc_post_bv[0]);
	SDE_REG_WRITE(c, csc_reg_off + 0x3c, data->csc_post_bv[1]);
	SDE_REG_WRITE(c, csc_reg_off + 0x40, data->csc_post_bv[2]);
}

/**
 * _sde_copy_formats   - copy formats from src_list to dst_list
 * @dst_list:          pointer to destination list where to copy formats
 * @dst_list_size:     size of destination list
 * @dst_list_pos:      starting position on the list where to copy formats
 * @src_list:          pointer to source list where to copy formats from
 * @src_list_size:     size of source list
 * Return: number of elements populated
 */
uint32_t sde_copy_formats(
		struct sde_format_extended *dst_list,
		uint32_t dst_list_size,
		uint32_t dst_list_pos,
		const struct sde_format_extended *src_list,
		uint32_t src_list_size)
{
	uint32_t cur_pos, i;

	if (!dst_list || !src_list || (dst_list_pos >= (dst_list_size - 1)))
		return 0;

	for (i = 0, cur_pos = dst_list_pos;
		(cur_pos < (dst_list_size - 1)) && (i < src_list_size)
		&& src_list[i].fourcc_format; ++i, ++cur_pos)
		dst_list[cur_pos] = src_list[i];

	dst_list[cur_pos].fourcc_format = 0;

	return i;
}
