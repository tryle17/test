/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 */

#ifndef __SDE_ENCODER_PHYS_H__
#define __SDE_ENCODER_PHYS_H__

#include <linux/jiffies.h>
#include <linux/sde_rsc.h>

#include "sde_kms.h"
#include "sde_hw_intf.h"
#include "sde_hw_pingpong.h"
#include "sde_hw_ctl.h"
#include "sde_hw_top.h"
#include "sde_hw_wb.h"
#include "sde_hw_cdm.h"
#include "sde_hw_dnsc_blur.h"
#include "sde_encoder.h"
#include "sde_connector.h"

#define SDE_ENCODER_NAME_MAX	16

/* wait for at most 2 vsync for lowest refresh rate (24hz) */
#define DEFAULT_KICKOFF_TIMEOUT_MS		84

/* if default timeout fails wait additional time in 1s increments */
#define EXTENDED_KICKOFF_TIMEOUT_MS      1000
#define EXTENDED_KICKOFF_TIMEOUT_ITERS   10

/* wait 1 sec for the emulated targets */
#define MAX_KICKOFF_TIMEOUT_MS                  100000

#define MAX_TE_PROFILE_COUNT		5
/**
 * enum sde_enc_split_role - Role this physical encoder will play in a
 *	split-panel configuration, where one panel is master, and others slaves.
 *	Masters have extra responsibilities, like managing the VBLANK IRQ.
 * @ENC_ROLE_SOLO:	This is the one and only panel. This encoder is master.
 * @DPU_MASTER_ENC_ROLE_MASTER:	This encoder is the master of a split panel config.
				DPU master with role master in Dual DPU sync mode.
 * @DPU_SLAVE_ENC_ROLE_MASTER:	This encoder is the master of a split panel config.
				DPU slave with role master in Dual DPU sync mode.
 * @ENC_ROLE_SLAVE:	This encoder is not the master of a split panel config.
 * @ENC_ROLE_SKIP:	This encoder is not participating in kickoffs
 */
enum sde_enc_split_role {
	ENC_ROLE_SOLO,
	DPU_MASTER_ENC_ROLE_MASTER,
	DPU_SLAVE_ENC_ROLE_MASTER,
	ENC_ROLE_SLAVE,
	ENC_ROLE_SKIP
};

/**
 * enum sde_enc_enable_state - current enabled state of the physical encoder
 * @SDE_ENC_DISABLING:	Encoder transitioning to disable state
 *			Events bounding transition are encoder type specific
 * @SDE_ENC_DISABLED:	Encoder is disabled
 * @SDE_ENC_ENABLING:	Encoder transitioning to enabled
 *			Events bounding transition are encoder type specific
 * @SDE_ENC_POST_ENABLING: Intermittent state of the encoder when DPU interface
 *			 sync is enabled
 * @SDE_ENC_ENABLED:	Encoder is enabled
 * @SDE_ENC_ERR_NEEDS_HW_RESET:	Encoder is enabled, but requires a hw_reset
 *				to recover from a previous error
 */
enum sde_enc_enable_state {
	SDE_ENC_DISABLING,
	SDE_ENC_DISABLED,
	SDE_ENC_ENABLING,
	SDE_ENC_POST_ENABLING,
	SDE_ENC_ENABLED,
	SDE_ENC_ERR_NEEDS_HW_RESET
};

enum sde_enc_irqs {
	SDE_ENC_CMD_TE_ASSERT,
	SDE_ENC_CMD_TE_DEASSERT,
	SDE_ENC_CMD_TEAR_DETECT,

	SDE_ENC_IRQ_MAX
};

struct sde_encoder_phys;

/**
 * struct sde_encoder_virt_ops - Interface the containing virtual encoder
 *	provides for the physical encoders to use to callback.
 * @handle_vblank_virt:	Notify virtual encoder of vblank IRQ reception
 *			Note: This is called from IRQ handler context.
 * @handle_underrun_virt: Notify virtual encoder of underrun IRQ reception
 *			Note: This is called from IRQ handler context.
 * @handle_frame_done:	Notify virtual encoder that this phys encoder
 *			completes last request frame.
 * @get_qsync_fps:	Returns the min fps for the qsync feature.
 */
struct sde_encoder_virt_ops {
	void (*handle_vblank_virt)(struct drm_encoder *parent,
			struct sde_encoder_phys *phys);
	void (*handle_underrun_virt)(struct drm_encoder *parent,
			struct sde_encoder_phys *phys);
	void (*handle_frame_done)(struct drm_encoder *parent,
			struct sde_encoder_phys *phys, u32 event);
	void (*get_qsync_fps)(struct drm_encoder *parent,
			u32 *qsync_fps, struct drm_connector_state *conn_state,
			struct sde_connector *sde_conn);
};

/**
 * struct sde_encoder_phys_ops - Interface the physical encoders provide to
 *	the containing virtual encoder.
 * @late_register:		DRM Call. Add Userspace interfaces, debugfs.
 * @prepare_commit:		MSM Atomic Call, start of atomic commit sequence
 * @is_master:			Whether this phys_enc is the current master
 *				encoder. Can be switched at enable time. Based
 *				on split_role and current mode (CMD/VID).
 * @mode_fixup:			DRM Call. Fixup a DRM mode.
 * @cont_splash_mode_set:	mode set with specific HW resources during
 *                              cont splash enabled state.
 * @mode_set:			DRM Call. Set a DRM mode.
 *				This likely caches the mode, for use at enable.
 * @enable:			DRM Call. Enable a DRM mode.
 * @disable:			DRM Call. Disable mode.
 * @atomic_check:		DRM Call. Atomic check new DRM state.
 * @destroy:			DRM Call. Destroy and release resources.
 * @get_hw_resources:		Populate the structure with the hardware
 *				resources that this phys_enc is using.
 *				Expect no overlap between phys_encs.
 * @control_vblank_irq		Register/Deregister for VBLANK IRQ
 * @wait_for_commit_done:	Wait for hardware to have flushed the
 *				current pending frames to hardware
 * @wait_for_tx_complete:	Wait for hardware to transfer the pixels
 *				to the panel
 * @wait_for_vblank:		Wait for VBLANK, for sub-driver internal use
 * @prepare_for_kickoff:	Do any work necessary prior to a kickoff
 *				For CMD encoder, may wait for previous tx done
 * @handle_post_kickoff:	Do any work necessary post-kickoff work
 * @trigger_flush:		Process flush event on physical encoder
 * @trigger_start:		Process start event on physical encoder
 * @clear_flush_mask:		clear flush mask
 * @needs_single_flush:		Whether encoder slaves need to be flushed
 * @setup_misr:		Sets up MISR, enable and disables based on sysfs
 * @collect_misr:		Collects MISR data on frame update
 * @hw_reset:			Issue HW recovery such as CTL reset and clear
 *				SDE_ENC_ERR_NEEDS_HW_RESET state
 * @irq_control:		Handler to enable/disable all the encoder IRQs
 * @update_split_role:		Update the split role of the phys enc
 * @control_te:			Interface to control the vsync_enable status
 * @restore:			Restore all the encoder configs.
 * @is_autorefresh_enabled:	provides the autorefresh current
 *                              enable/disable state.
 * @get_line_count:		Obtain current internal vertical line count
 * @wait_dma_trigger:		Returns true if lut dma has to trigger and wait
 *                              unitl transaction is complete.
 * @wait_for_active:		Wait for display scan line to be in active area
 * @setup_vsync_source:		Configure vsync source selection for cmd mode.
 * @get_underrun_line_count:	Obtain and log current internal vertical line
 *                              count and underrun line count
 * @add_to_minidump:		Add this phys_enc data to minidumps
 * @disable_autorefresh:	Disable autorefresh
 * @idle_pc_cache_display_status:	caches display status at idle power collapse
 * @cesta_ctrl_cfg:		Cesta control configuration
 * @idle_pc_enter:		Enter idle power collapse
 * @idle_pc_exit:		Exit idle power collapse
 * @wait_for_vsync_on_autorefresh_busy:	Wait for vsync if autorefresh status busy
 */

struct sde_encoder_phys_ops {
	int (*late_register)(struct sde_encoder_phys *encoder,
			struct dentry *debugfs_root);
	void (*prepare_commit)(struct sde_encoder_phys *encoder);
	bool (*is_master)(struct sde_encoder_phys *encoder);
	bool (*mode_fixup)(struct sde_encoder_phys *encoder,
			const struct drm_display_mode *mode,
			struct drm_display_mode *adjusted_mode);
	void (*mode_set)(struct sde_encoder_phys *encoder,
			struct drm_display_mode *mode,
			struct drm_display_mode *adjusted_mode, bool *reinit_mixers);
	void (*cont_splash_mode_set)(struct sde_encoder_phys *encoder,
			struct drm_display_mode *adjusted_mode);
	void (*enable)(struct sde_encoder_phys *encoder);
	void (*disable)(struct sde_encoder_phys *encoder);
	int (*atomic_check)(struct sde_encoder_phys *encoder,
			    struct drm_crtc_state *crtc_state,
			    struct drm_connector_state *conn_state);
	void (*destroy)(struct sde_encoder_phys *encoder);
	void (*get_hw_resources)(struct sde_encoder_phys *encoder,
			struct sde_encoder_hw_resources *hw_res,
			struct drm_connector_state *conn_state);
	int (*control_vblank_irq)(struct sde_encoder_phys *enc, bool enable);
	int (*wait_for_commit_done)(struct sde_encoder_phys *phys_enc);
	int (*wait_for_tx_complete)(struct sde_encoder_phys *phys_enc);
	int (*wait_for_vblank)(struct sde_encoder_phys *phys_enc);
	int (*prepare_for_kickoff)(struct sde_encoder_phys *phys_enc,
			struct sde_encoder_kickoff_params *params);
	void (*handle_post_kickoff)(struct sde_encoder_phys *phys_enc);
	void (*trigger_flush)(struct sde_encoder_phys *phys_enc);
	void (*trigger_start)(struct sde_encoder_phys *phys_enc);
	void (*clear_flush_mask)(struct sde_encoder_phys *phys_enc, bool clear);
	bool (*needs_single_flush)(struct sde_encoder_phys *phys_enc);

	void (*setup_misr)(struct sde_encoder_phys *phys_encs,
				bool enable, u32 frame_count);
	int (*collect_misr)(struct sde_encoder_phys *phys_enc, bool nonblock,
			u32 *misr_value);
	void (*hw_reset)(struct sde_encoder_phys *phys_enc);
	void (*irq_control)(struct sde_encoder_phys *phys, bool enable);
	void (*dynamic_irq_control)(struct sde_encoder_phys *phys, bool enable);
	void (*update_split_role)(struct sde_encoder_phys *phys_enc,
			enum sde_enc_split_role role);
	void (*control_te)(struct sde_encoder_phys *phys_enc, bool enable);
	void (*restore)(struct sde_encoder_phys *phys);
	bool (*is_autorefresh_enabled)(struct sde_encoder_phys *phys);
	int (*get_line_count)(struct sde_encoder_phys *phys);
	bool (*wait_dma_trigger)(struct sde_encoder_phys *phys);
	int (*wait_for_active)(struct sde_encoder_phys *phys);
	void (*setup_vsync_source)(struct sde_encoder_phys *phys, u32 vsync_source,
			struct msm_display_info *disp_info);
	u32 (*get_underrun_line_count)(struct sde_encoder_phys *phys);
	void (*add_to_minidump)(struct sde_encoder_phys *phys);
	void (*disable_autorefresh)(struct sde_encoder_phys *phys);
	void (*idle_pc_cache_display_status)(struct sde_encoder_phys *phys);
	void (*cesta_ctrl_cfg)(struct sde_encoder_phys *phys, struct sde_cesta_ctrl_cfg *cfg,
			bool *req_flush, bool *req_scc);
	void (*idle_pc_enter)(struct sde_encoder_phys *phys);
	void (*idle_pc_exit)(struct sde_encoder_phys *phys);
	void (*wait_for_vsync_on_autorefresh_busy)(struct sde_encoder_phys *phys_enc);
};

/**
 * enum sde_intr_idx - sde encoder interrupt index
 * @INTR_IDX_VSYNC:    Vsync interrupt for video mode panel
 * @INTR_IDX_PINGPONG: Pingpong done interrupt for cmd mode panel
 * @INTR_IDX_UNDERRUN: Underrun interrupt for video and cmd mode panel
 * @INTR_IDX_WD_TIMER: Watchdog interrupt
 * @INTR_IDX_CTL_START:Control start interrupt to indicate the frame start
 * @INTR_IDX_CTL_DONE: Control done interrupt indicating the control path being idle
 * @INTR_IDX_RDPTR:    Readpointer done interrupt for cmd mode panel
 * @INTR_IDX_WB_DONE:  Writeback done interrupt for WB
 * @INTR_IDX_PP1_OVFL: Pingpong overflow interrupt on PP1 for Concurrent WB
 * @INTR_IDX_PP2_OVFL: Pingpong overflow interrupt on PP2 for Concurrent WB
 * @INTR_IDX_PP3_OVFL: Pingpong overflow interrupt on PP3 for Concurrent WB
 * @INTR_IDX_PP4_OVFL: Pingpong overflow interrupt on PP4 for Concurrent WB
 * @INTR_IDX_PP5_OVFL: Pingpong overflow interrupt on PP5 for Concurrent WB
 * @INTR_IDX_PP_CWB_OVFL: Pingpong overflow interrupt on PP_CWB0/1 for Concurrent WB
 * @INTR_IDX_PP_CWB2_OVFL: Pingpong overflow interrupt on PP_CWB2/3 for Concurrent WB
 * @INTR_IDX_AUTOREFRESH_DONE:  Autorefresh done for cmd mode panel meaning
 *                              autorefresh has triggered a double buffer flip
 * @INTR_IDX_WRPTR:    Writepointer start interrupt for cmd mode panel
 * @INTR_IDX_WB_LINEPTR:  Programmable lineptr interrupt for WB
 * @INTF_IDX_TEAR_DETECT:    Tear detect interrupt
 * @INTR_IDX_TE_ASSERT:      TE Assert interrupt
 * @INTR_IDX_TE_DEASSERT:    TE Deassert interrupt
 */
enum sde_intr_idx {
	INTR_IDX_VSYNC,
	INTR_IDX_PINGPONG,
	INTR_IDX_UNDERRUN,
	INTR_IDX_WD_TIMER,
	INTR_IDX_CTL_START,
	INTR_IDX_CTL_DONE,
	INTR_IDX_RDPTR,
	INTR_IDX_AUTOREFRESH_DONE,
	INTR_IDX_WB_DONE,
	INTR_IDX_PP1_OVFL,
	INTR_IDX_PP2_OVFL,
	INTR_IDX_PP3_OVFL,
	INTR_IDX_PP4_OVFL,
	INTR_IDX_PP5_OVFL,
	INTR_IDX_PP_CWB_OVFL,
	INTR_IDX_PP_CWB2_OVFL,
	INTR_IDX_WRPTR,
	INTR_IDX_WB_LINEPTR,
	INTF_IDX_TEAR_DETECT,
	INTR_IDX_TE_ASSERT,
	INTR_IDX_TE_DEASSERT,
	INTR_IDX_MAX,
};

/**
 * sde_encoder_irq - tracking structure for interrupts
 * @name:		string name of interrupt
 * @intr_type:		Encoder interrupt type
 * @intr_idx:		Encoder interrupt enumeration
 * @hw_idx:		HW Block ID
 * @irq_idx:		IRQ interface lookup index from SDE IRQ framework
 *			will be -EINVAL if IRQ is not registered
 * @irq_cb:		interrupt callback
 */
struct sde_encoder_irq {
	const char *name;
	enum sde_intr_type intr_type;
	enum sde_intr_idx intr_idx;
	int hw_idx;
	int irq_idx;
	struct sde_irq_callback cb;
};

enum sde_transition_state {
	ARP_MODE_NONE,
	ARP_MODE3_HW_TE_ON,
	ARM_MODE3_TO_MODE1,
	ARP_MODE1_ACTIVE,
	ARP_MODE1_IDLE,
};

struct sde_encoder_vrr_cfg {
	bool arp_mode_hw_te;
	bool arp_mode_sw_timer_mode;
	bool is_freq_pattern_altered;
	u16 arp_transition_state;
	u32 curr_index;
	u32 curr_frame_interval_fps;
	u64 curr_image_ts_in_ns;
	u64 last_commit_ept_in_ns;
	u64 last_image_ts_in_ns;
	u64 arp_mode_off_time_ns;
	u64 freq_step_timer_val_ns;
	struct msm_freq_step_pattern *curr_freq_pattern;
	struct hrtimer freq_step_timer;
	struct hrtimer arp_transition_timer;
	struct hrtimer self_refresh_timer;
	struct hrtimer backlight_timer;
};

/**
 * struct sde_encoder_phys - physical encoder that drives a single INTF block
 *	tied to a specific panel / sub-panel. Abstract type, sub-classed by
 *	phys_vid or phys_cmd for video mode or command mode encs respectively.
 * @parent:		Pointer to the containing virtual encoder
 * @connector:		If a mode is set, cached pointer to the active connector
 * @ops:		Operations exposed to the virtual encoder
 * @parent_ops:		Callbacks exposed by the parent to the phys_enc
 * @hw_mdptop:		Hardware interface to the top registers
 * @hw_ctl:		Hardware interface to the ctl registers
 * @hw_intf:		Hardware interface to INTF registers
 * @hw_cdm:		Hardware interface to the cdm registers
 * @hw_qdss:		Hardware interface to the qdss registers
 * @cdm_cfg:		Chroma-down hardware configuration
 * @hw_pp:		Hardware interface to the ping pong registers
 * @hw_dnsc_blur:	Hardware interface to the downscale blur registers
 * @sde_kms:		Pointer to the sde_kms top level
 * @cached_mode:	DRM mode cached at mode_set time, acted on in enable
 * @wd_jitter : Pointer to watchdog jitter prams
 * @enabled:		Whether the encoder has enabled and running a mode
 * @split_role:		Role to play in a split-panel configuration
 * @intf_mode:		Interface mode
 * @intf_idx:		Interface index on sde hardware
 * @intf_cfg:		Interface hardware configuration
 * @intf_cfg_v1:        Interface hardware configuration to be used if control
 *                      path supports SDE_CTL_ACTIVE_CFG
 * @comp_type:      Type of compression supported
 * @comp_ratio:		Compression ratio multiplied by 100
 * @dsc_extra_pclk_cycle_cnt: Extra pclk cycle count for DSC over DP
 * @dsc_extra_disp_width: Additional display width for DSC over DP
 * @poms_align_vsync:   poms with vsync aligned
 * @dce_bytes_per_line:	Compressed bytes per line
 * @enc_spinlock:	Virtual-Encoder-Wide Spin Lock for IRQ purposes
 * @enable_state:	Enable state tracking
 * @vblank_refcount:	Reference count of vblank request
 * @wbirq_refcount:	Reference count of wb irq request
 * @vsync_cnt:		Vsync count for the physical encoder
 * @last_vsync_timestamp:	store last vsync timestamp
 * @underrun_cnt:	Underrun count for the physical encoder
 * @pending_kickoff_cnt:	Atomic counter tracking the number of kickoffs
 *				vs. the number of done/vblank irqs. Should hover
 *				between 0-2 Incremented when a new kickoff is
 *				scheduled. Decremented in irq handler
 * @pending_retire_fence_cnt:   Atomic counter tracking the pending retire
 *                              fences that have to be signalled.
 * @pending_ctl_start_cnt:      Atomic counter tracking the pending ctl-start-irq,
 *                              used to release commit thread. Currently managed
 *                              only for writeback encoder and the counter keeps
 *                              increasing for other type of encoders.
 * @pending_kickoff_wq:		Wait queue for blocking until kickoff completes
 * @kickoff_timeout_ms:		kickoff timeout in mill seconds
 * @irq:			IRQ tracking structures
 * @has_intf_te:		Interface TE configuration support
 * @cont_splash_enabled:	Variable to store continuous splash settings.
 * @in_clone_mode		Indicates if encoder is in clone mode ref@CWB
 * @vfp_cached:			cached vertical front porch to be used for
 *				programming ROT and MDP fetch start
 * @pf_time_in_us:		Programmable fetch time in micro-seconds
 * @sde_hw_fence_error_status:	Hw fence error handing flag controled by userspace
 *				that if handing fence error in driver
 * @sde_hw_fence_error_value:	hw fence error value from cb function
 * @sde_hw_fence_handle:	Hw fence driver client handle, this handle was returned
 *				during the call 'synx_initialize' to register the client
 * @fence_error_handle_in_progress:
 *				bool to indicate if fence error handling in progress
 *				This is set once fence error occurs and cleared only when
 *				good frame is received. Not cleared in continous fence
 *				error cases
 * @frame_trigger_mode:		frame trigger mode indication for command
 *				mode display
 * @recovered:			flag set to true when recovered from pp timeout
 * @autorefresh_disable_trans:   flag set to true during autorefresh disable transition
 * @sim_qsync_frame:            Current simulated qsync frame type
 * @prog_fetch_start:           current programmable fetch value
 * @sde_vrr_cfg:      VRR configuration information
 */
struct sde_encoder_phys {
	struct drm_encoder *parent;
	struct drm_connector *connector;
	struct sde_encoder_phys_ops ops;
	struct sde_encoder_virt_ops parent_ops;
	struct sde_hw_mdp *hw_mdptop;
	struct sde_hw_ctl *hw_ctl;
	struct sde_hw_intf *hw_intf;
	struct sde_hw_cdm *hw_cdm;
	struct sde_hw_qdss *hw_qdss;
	struct sde_hw_cdm_cfg cdm_cfg;
	struct sde_hw_pingpong *hw_pp;
	struct sde_hw_dnsc_blur *hw_dnsc_blur;
	struct sde_kms *sde_kms;
	struct drm_display_mode cached_mode;
	struct intf_wd_jitter_params wd_jitter;
	enum sde_enc_split_role split_role;
	enum sde_intf_mode intf_mode;
	enum sde_intf intf_idx;
	struct sde_hw_intf_cfg intf_cfg;
	struct sde_hw_intf_cfg_v1 intf_cfg_v1;
	enum msm_display_compression_type comp_type;
	u32 comp_ratio;
	u32 dsc_extra_pclk_cycle_cnt;
	u32 dsc_extra_disp_width;
	bool poms_align_vsync;
	u32 dce_bytes_per_line;
	spinlock_t *enc_spinlock;
	enum sde_enc_enable_state enable_state;
	struct mutex *vblank_ctl_lock;
	atomic_t vblank_refcount;
	atomic_t wbirq_refcount;
	atomic_t vsync_cnt;
	ktime_t last_vsync_timestamp;
	atomic_t underrun_cnt;
	atomic_t pending_kickoff_cnt;
	atomic_t pending_retire_fence_cnt;
	atomic_t pending_ctl_start_cnt;
	wait_queue_head_t pending_kickoff_wq;
	u32 kickoff_timeout_ms;
	struct sde_encoder_irq irq[INTR_IDX_MAX];
	bool has_intf_te;
	bool cont_splash_enabled;
	bool in_clone_mode;
	int vfp_cached;
	u32 pf_time_in_us;
	bool sde_hw_fence_error_status;
	int sde_hw_fence_error_value;
	u64 sde_hw_fence_handle;
	bool fence_error_handle_in_progress;
	enum frame_trigger_mode_type frame_trigger_mode;
	bool recovered;
	bool autorefresh_disable_trans;
	enum sde_sim_qsync_frame sim_qsync_frame;
	u32 prog_fetch_start;
	bool esync_pc_exit;
	struct sde_encoder_vrr_cfg sde_vrr_cfg;
};

static inline int sde_encoder_phys_inc_pending(struct sde_encoder_phys *phys)
{
	return atomic_inc_return(&phys->pending_kickoff_cnt);
}

/*
 * sde_encoder_clear_fence_error_in_progress - clear fence_error_handle_in_progress flag
 *	after good frame
 * @phys_enc: Pointer to physical encoder structure
 */
void sde_encoder_clear_fence_error_in_progress(struct sde_encoder_phys *phys_enc);

/**
 * sde_encoder_hw_fence_signal - hw fence related fence error handing
 * @phys_enc: Pointer to physical encoder structure
 * return: 0 on success; error code otherwise
 */
static inline int sde_encoder_hw_fence_signal(struct sde_encoder_phys *phys_enc);

/**
 * struct sde_encoder_phys_vid - sub-class of sde_encoder_phys to handle video
 *	mode specific operations
 * @base:	Baseclass physical encoder structure
 * @timing_params: Current timing parameter
 * @error_count: Number of consecutive kickoffs that experienced an error
 */
struct sde_encoder_phys_vid {
	struct sde_encoder_phys base;
	struct intf_timing_params timing_params;
	int error_count;
};

/**
 * struct sde_encoder_phys_cmd_autorefresh - autorefresh state tracking
 * @cfg: current active autorefresh configuration
 * @kickoff_cnt: atomic count tracking autorefresh done irq kickoffs pending
 * @kickoff_wq:	wait queue for waiting on autorefresh done irq
 */
struct sde_encoder_phys_cmd_autorefresh {
	struct sde_hw_autorefresh cfg;
	atomic_t kickoff_cnt;
	wait_queue_head_t kickoff_wq;
};

/**
 * struct sde_encoder_phys_cmd_te_timestamp - list node to keep track of
 *     rd_ptr/TE timestamp
 * @list: list node
 * @timestamp: TE timestamp
 */
struct sde_encoder_phys_cmd_te_timestamp {
	struct list_head list;
	ktime_t timestamp;
};

/**
 * struct sde_encoder_phys_cmd - sub-class of sde_encoder_phys to handle command
 *	mode specific operations
 * @base:	Baseclass physical encoder structure
 * @stream_sel:	Stream selection for multi-stream interfaces
 * @frame_tx_timeout_report_cnt: number of pp_done/ctl_done irq timeout errors
 * @autorefresh: autorefresh feature state
 * @pending_vblank_cnt: Atomic counter tracking pending wait for VBLANK
 * @pending_vblank_wq: Wait queue for blocking until VBLANK received
 * @wr_ptr_wait_success: log wr_ptr_wait success for release fence trigger
 * @te_timestamp_list: List head for the TE timestamp list
 * @te_timestamp: Array of size MAX_TE_PROFILE_COUNT te_timestamp_list elements
 * @qsync_threshold_lines: tearcheck threshold lines calculated based on qsync_min_fps
 */
struct sde_encoder_phys_cmd {
	struct sde_encoder_phys base;
	int stream_sel;
	int frame_tx_timeout_report_cnt;
	struct sde_encoder_phys_cmd_autorefresh autorefresh;
	atomic_t pending_vblank_cnt;
	wait_queue_head_t pending_vblank_wq;
	bool wr_ptr_wait_success;
	struct list_head te_timestamp_list;
	struct sde_encoder_phys_cmd_te_timestamp
			te_timestamp[MAX_TE_PROFILE_COUNT];
	u32 qsync_threshold_lines;
};

/**
 * struct sde_encoder_phys_wb - sub-class of sde_encoder_phys to handle
 *	writeback specific operations
 * @base:		Baseclass physical encoder structure
 * @hw_wb:		Hardware interface to the wb registers
 * @wbdone_timeout:	Timeout value for writeback done in msec
 * @wb_cfg:		Writeback hardware configuration
 * @cdp_cfg:		Writeback CDP configuration
 * @wb_roi:		Writeback region-of-interest
 * @wb_fmt:		Writeback pixel format
 * @wb_fb:		Pointer to current writeback framebuffer
 * @wb_aspace:		Pointer to current writeback address space
 * @old_fb:		Pointer to old writeback framebuffer
 * @old_aspace:		Pointer to old writeback address space
 * @aspace:		address space identifier for non-secure/secure domain
 * @wb_dev:		Pointer to writeback device
 * @bo_disable:		Buffer object(s) to use during the disabling state
 * @fb_disable:		Frame buffer to use during the disabling state
 * @sc_cfg:		Stores wb system cache config
 * @crtc:		Pointer to drm_crtc
 * @prog_line:		Cached programmable line value used to trigger early wb-fence
 */
struct sde_encoder_phys_wb {
	struct sde_encoder_phys base;
	struct sde_hw_wb *hw_wb;
	u32 wbdone_timeout;
	struct sde_hw_wb_cfg wb_cfg;
	struct sde_hw_wb_cdp_cfg cdp_cfg;
	struct sde_rect wb_roi;
	const struct sde_format *wb_fmt;
	struct drm_framebuffer *wb_fb;
	struct msm_gem_address_space *wb_aspace;
	struct drm_framebuffer *old_fb;
	struct msm_gem_address_space *old_aspace;
	struct msm_gem_address_space *aspace[SDE_IOMMU_DOMAIN_MAX];
	struct sde_wb_device *wb_dev;
	struct drm_gem_object *bo_disable[SDE_MAX_PLANES];
	struct drm_framebuffer *fb_disable;
	struct sde_hw_wb_sc_cfg sc_cfg;
	struct drm_crtc *crtc;
	u32 prog_line;
};

/**
 * struct sde_enc_phys_init_params - initialization parameters for phys encs
 * @sde_kms:		Pointer to the sde_kms top level
 * @parent:		Pointer to the containing virtual encoder
 * @parent_ops:		Callbacks exposed by the parent to the phys_enc
 * @split_role:		Role to play in a split-panel configuration
 * @intf_idx:		Interface index this phys_enc will control
 * @wb_idx:		Writeback index this phys_enc will control
 * @comp_type:      Type of compression supported
 * @enc_spinlock:	Virtual-Encoder-Wide Spin Lock for IRQ purposes
 */
struct sde_enc_phys_init_params {
	struct sde_kms *sde_kms;
	struct drm_encoder *parent;
	struct sde_encoder_virt_ops parent_ops;
	enum sde_enc_split_role split_role;
	enum sde_intf intf_idx;
	enum sde_wb wb_idx;
	enum msm_display_compression_type comp_type;
	spinlock_t *enc_spinlock;
	struct mutex *vblank_ctl_lock;
};

/**
 * sde_encoder_wait_info - container for passing arguments to irq wait functions
 * @wq: wait queue structure
 * @atomic_cnt: wait until atomic_cnt equals zero
 * @count_check: wait for specific atomic_cnt instead of zero.
 * @timeout_ms: timeout value in milliseconds
 */
struct sde_encoder_wait_info {
	wait_queue_head_t *wq;
	atomic_t *atomic_cnt;
	u32 count_check;
	s64 timeout_ms;
};

/**
 * sde_encoder_phys_vid_init - Construct a new video mode physical encoder
 * @p:	Pointer to init params structure
 * Return: Error code or newly allocated encoder
 */
struct sde_encoder_phys *sde_encoder_phys_vid_init(
		struct sde_enc_phys_init_params *p);

/**
 * sde_encoder_phys_cmd_init - Construct a new command mode physical encoder
 * @p:	Pointer to init params structure
 * Return: Error code or newly allocated encoder
 */
struct sde_encoder_phys *sde_encoder_phys_cmd_init(
		struct sde_enc_phys_init_params *p);

/**
 * sde_encoder_phys_wb_init - Construct a new writeback physical encoder
 * @p:	Pointer to init params structure
 * Return: Error code or newly allocated encoder
 */
#if IS_ENABLED(CONFIG_DRM_SDE_WB)
struct sde_encoder_phys *sde_encoder_phys_wb_init(
		struct sde_enc_phys_init_params *p);
#else
static inline
struct sde_encoder_phys *sde_encoder_phys_wb_init(
		struct sde_enc_phys_init_params *p)
{
	return NULL;
}
#endif /* CONFIG_DRM_SDE_WB */

void sde_encoder_phys_setup_cdm(struct sde_encoder_phys *phys_enc,
		struct drm_framebuffer *fb, const struct sde_format *format,
		struct sde_rect *wb_roi);

/**
 * sde_encoder_helper_get_pp_line_count - pingpong linecount helper function
 * @drm_enc:    Pointer to drm encoder structure
 * @info:       structure used to populate the pp line count information
 */
void sde_encoder_helper_get_pp_line_count(struct drm_encoder *drm_enc,
		struct sde_hw_pp_vsync_info *info);

/**
 * sde_encoder_helper_get_kickoff_timeout_ms- get the kickoff timeout value based on fps
 * @drm_enc: Pointer to drm encoder structure
 * Returns: Kickoff timeout in milli seconds
 */
u32 sde_encoder_helper_get_kickoff_timeout_ms(struct drm_encoder *drm_enc);

/**
 * sde_encoder_helper_trigger_flush - control flush helper function
 *	This helper function may be optionally specified by physical
 *	encoders if they require ctl_flush triggering.
 * @phys_enc: Pointer to physical encoder structure
 */
void sde_encoder_helper_trigger_flush(struct sde_encoder_phys *phys_enc);

/**
 * sde_encoder_helper_trigger_start - control start helper function
 *	This helper function may be optionally specified by physical
 *	encoders if they require ctl_start triggering.
 * @phys_enc: Pointer to physical encoder structure
 */
void sde_encoder_helper_trigger_start(struct sde_encoder_phys *phys_enc);

/**
 * sde_encoder_helper_vsync_config - configure vsync source for cmd mode
 * @phys_enc: Pointer to physical encoder structure
 * @vsync_source: vsync source selection
 */
void sde_encoder_helper_vsync_config(struct sde_encoder_phys *phys_enc, u32 vsync_source);

/**
 * sde_encoder_helper_wait_event_timeout - wait for event with timeout
 *	taking into account that jiffies may jump between reads leading to
 *	incorrectly detected timeouts. Prevent failure in this scenario by
 *	making sure that elapsed time during wait is valid.
 * @drm_id: drm object id for logging
 * @hw_id: hw instance id for logging
 * @info: wait info structure
 */
int sde_encoder_helper_wait_event_timeout(
		int32_t drm_id,
		int32_t hw_id,
		struct sde_encoder_wait_info *info);

/*
 * sde_encoder_get_fps - get the allowed panel jitter in nanoseconds
 * @frame_rate: custom input frame rate
 * @jitter_num: jitter numerator value
 * @jitter_denom: jitter denomerator value,
 * @l_bound: lower frame period boundary
 * @u_bound: upper frame period boundary
 */
void sde_encoder_helper_get_jitter_bounds_ns(uint32_t frame_rate,
			u32 jitter_num, u32 jitter_denom,
			ktime_t *l_bound, ktime_t *u_bound);

/**
 * sde_encoder_helper_switch_vsync - switch vsync source to WD or default
 * @drm_enc:     Pointer to drm encoder structure
 * @watchdog_te: switch vsync source to watchdog TE
 */
int sde_encoder_helper_switch_vsync(struct drm_encoder *drm_enc,
		bool watchdog_te);

/**
 * sde_encoder_helper_get_bw_update_time_lines - gets the bandwidth update time in lines
 * @sde_enc: Pointer to sde encoder structure
 */
u32 sde_encoder_helper_get_bw_update_time_lines(struct sde_encoder_virt *sde_enc);

/**
 * sde_encoder_helper_calc_vsync_count - calculates the vsync_count value
 * @drm_enc: Pointer to drm encoder structure
 * @vtotal: vtotal of the mode
 * @vrefresh: vrefresh of the mode
 */
u32 sde_encoder_helper_calc_vsync_count(struct drm_encoder *drm_enc, u32 vtotal, u32 vrefresh);

/**
 * sde_encoder_phys_has_role_master_dpu_master_intf - check if role of physical
	 encoder is (MASTER_DPU, MASTER_INTF) when interface synchronization is enabled.
 * @phys_enc: Pointer to physical encoder structure
 */
static inline bool sde_encoder_phys_has_role_master_dpu_master_intf(
		struct sde_encoder_phys *phys_enc)
{
	if (!phys_enc)
		return false;

	return (phys_enc->split_role == DPU_MASTER_ENC_ROLE_MASTER) ? true : false;
}

/**
 * sde_encoder_phys_has_role_slave_dpu_master_intf - check if role of physical
	encoder is (SLAVE_DPU, MASTER_INTF) when interface synchronization is enabled.
 * @phys_enc: Pointer to physical encoder structure
 */
static inline bool sde_encoder_phys_has_role_slave_dpu_master_intf(
		struct sde_encoder_phys *phys_enc)
{
	if (!phys_enc)
		return false;

	return (phys_enc->split_role == DPU_SLAVE_ENC_ROLE_MASTER) ? true : false;
}

/*
 * sde_encoder_in_solo_mode - check if DPU's are enabled in sync mode with each DPU in solo mode
 * @phys_enc:    Pointer to physical encoder structure
 * @Return: true if each DPU is operating in sync_mode with 1 tile display
 */
static inline bool sde_encoder_master_in_solo_mode(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_virt *sde_enc;

	if (!phys_enc || !phys_enc->parent)
		return false;

	sde_enc = to_sde_encoder_virt(phys_enc->parent);

	return ((sde_enc->num_phys_encs == 1) && (sde_enc->dpu_ctl_op_sync)) ? true : false;
}

/**
 * sde_encoder_helper_hw_reset - issue ctl hw reset
 *	This helper function may be optionally specified by physical
 *	encoders if they require ctl hw reset. If state is currently
 *	SDE_ENC_ERR_NEEDS_HW_RESET, it is set back to SDE_ENC_ENABLED.
 * @phys_enc: Pointer to physical encoder structure
 */
void sde_encoder_helper_hw_reset(struct sde_encoder_phys *phys_enc);

static inline enum sde_3d_blend_mode sde_encoder_helper_get_3d_blend_mode(
		struct sde_encoder_phys *phys_enc)
{
	struct msm_display_topology def;
	enum sde_enc_split_role split_role;
	int ret, num_lm;
	bool mode_3d;

	if (!phys_enc || phys_enc->enable_state == SDE_ENC_DISABLING ||
			!phys_enc->connector || !phys_enc->connector->state)
		return BLEND_3D_NONE;

	ret = sde_connector_state_get_topology
			(phys_enc->connector->state, &def);
	if (ret)
		return BLEND_3D_NONE;

	if (phys_enc->hw_intf && phys_enc->hw_intf->cfg.split_link_en)
		return BLEND_3D_NONE;

	num_lm = def.num_lm;
	mode_3d = (num_lm > def.num_enc) ? true : false;
	split_role = phys_enc->split_role;

	/* solo mode / Dual DPU sync solo mode enabled with mode_3d in topology */
	if ((split_role == ENC_ROLE_SOLO || sde_encoder_master_in_solo_mode(phys_enc))
			&& num_lm == 2 && mode_3d)
		return BLEND_3D_H_ROW_INT;

	/* split mode / Dual DPU sync split mode enabled with mode_3d in topology */
	if ((split_role != ENC_ROLE_SOLO && !sde_encoder_master_in_solo_mode(phys_enc))
			&& num_lm == 4 && mode_3d)
		return BLEND_3D_H_ROW_INT;

	return BLEND_3D_NONE;
}

/**
 * sde_encoder_phys_is_cwb_disabling - Check if CWB encoder attached to this
 *	 CRTC and it is in SDE_ENC_DISABLING state.
 * @phys_enc: Pointer to physical encoder structure
 * @crtc: drm crtc
 * @Return: true if cwb encoder is in disabling state
 */
static inline bool sde_encoder_phys_is_cwb_disabling(
	struct sde_encoder_phys *phys, struct drm_crtc *crtc)
{
	struct sde_encoder_phys_wb *wb_enc;

	if (!phys || !phys->in_clone_mode ||
				phys->enable_state != SDE_ENC_DISABLING)
		return false;

	wb_enc = container_of(phys, struct sde_encoder_phys_wb, base);
	return (wb_enc->crtc == crtc) ? true : false;
}

/**
 * sde_encoder_helper_split_config - split display configuration helper function
 *	This helper function may be used by physical encoders to configure
 *	the split display related registers.
 * @phys_enc: Pointer to physical encoder structure
 * @interface: enum sde_intf setting
 */
void sde_encoder_helper_split_config(
		struct sde_encoder_phys *phys_enc,
		enum sde_intf interface);

/**
 * sde_encoder_helper_reset_mixers - reset mixers associated with phys enc
 * @phys_enc: Pointer to physical encoder structure
 * @fb: Optional fb for specifying new mixer output resolution, may be NULL
 * Return: Zero on success
 */
int sde_encoder_helper_reset_mixers(struct sde_encoder_phys *phys_enc,
		struct drm_framebuffer *fb);
/**
 * sde_encoder_helper_hw_fence_sw_override - reset mixers and do hw-fence sw override
 * @phys_enc: Pointer to physical encoder structure
 * @ctl: Pointer to hw_ctl structure
 */
void sde_encoder_helper_hw_fence_sw_override(struct sde_encoder_phys *phys_enc,
		struct sde_hw_ctl *ctl);

/**
 * sde_encoder_helper_report_irq_timeout - utility to report error that irq has
 *	timed out, including reporting frame error event to crtc and debug dump
 * @phys_enc: Pointer to physical encoder structure
 * @intr_idx: Failing interrupt index
 */
void sde_encoder_helper_report_irq_timeout(struct sde_encoder_phys *phys_enc,
		enum sde_intr_idx intr_idx);

/**
 * sde_encoder_helper_wait_for_irq - utility to wait on an irq.
 *	note: will call sde_encoder_helper_wait_for_irq on timeout
 * @phys_enc: Pointer to physical encoder structure
 * @intr_idx: encoder interrupt index
 * @wait_info: wait info struct
 * @Return: 0 or -ERROR
 */
int sde_encoder_helper_wait_for_irq(struct sde_encoder_phys *phys_enc,
		enum sde_intr_idx intr_idx,
		struct sde_encoder_wait_info *wait_info);

/**
 * sde_encoder_helper_register_irq - register and enable an irq
 * @phys_enc: Pointer to physical encoder structure
 * @intr_idx: encoder interrupt index
 * @Return: 0 or -ERROR
 */
int sde_encoder_helper_register_irq(struct sde_encoder_phys *phys_enc,
		enum sde_intr_idx intr_idx);

/**
 * sde_encoder_helper_unregister_irq - unregister and disable an irq
 * @phys_enc: Pointer to physical encoder structure
 * @intr_idx: encoder interrupt index
 * @Return: 0 or -ERROR
 */
int sde_encoder_helper_unregister_irq(struct sde_encoder_phys *phys_enc,
		enum sde_intr_idx intr_idx);

/**
 * sde_encoder_helper_update_intf_cfg - update interface configuration for
 *                                      single control path.
 * @phys_enc: Pointer to physical encoder structure
 */
void sde_encoder_helper_update_intf_cfg(
		struct sde_encoder_phys *phys_enc);

/**
 * sde_encoder_restore_tearcheck_rd_ptr - restore interface rd_ptr configuration
 *	This function reads the panel scan line value using a DCS command
 *	and overrides the internal interface read pointer configuration.
 * @phys_enc: Pointer to physical encoder structure
 */
void sde_encoder_restore_tearcheck_rd_ptr(struct sde_encoder_phys *phys_enc);

/**
 * _sde_encoder_phys_is_dual_ctl - check if encoder needs dual ctl path.
 * @phys_enc: Pointer to physical encoder structure
 * @Return: true if dual ctl paths else false
 */
static inline bool _sde_encoder_phys_is_dual_ctl(
		struct sde_encoder_phys *phys_enc)
{
	struct sde_kms *sde_kms;
	enum sde_rm_topology_name topology;
	const struct sde_rm_topology_def* def;

	if (!phys_enc) {
		pr_err("invalid phys_enc\n");
		return false;
	}

	sde_kms = phys_enc->sde_kms;
	if (!sde_kms) {
		pr_err("invalid kms\n");
		return false;
	}

	topology = sde_connector_get_topology_name(phys_enc->connector);
	def = sde_rm_topology_get_topology_def(&sde_kms->rm, topology);
	if (IS_ERR_OR_NULL(def)) {
		pr_err("invalid topology\n");
		return false;
	}

	return (def->num_ctl == 2) ? true : false;
}

/**
 * _sde_encoder_phys_is_ppsplit - check if pp_split is enabled
 * @phys_enc: Pointer to physical encoder structure
 * @Return: true or false
 */
static inline bool _sde_encoder_phys_is_ppsplit(
		struct sde_encoder_phys *phys_enc)
{
	enum sde_rm_topology_name topology;

	if (!phys_enc) {
		pr_err("invalid phys_enc\n");
		return false;
	}

	topology = sde_connector_get_topology_name(phys_enc->connector);
	if (topology == SDE_RM_TOPOLOGY_PPSPLIT)
		return true;

	return false;
}

static inline bool sde_encoder_phys_needs_single_flush(
		struct sde_encoder_phys *phys_enc)
{
	if (!phys_enc)
		return false;

	return (_sde_encoder_phys_is_ppsplit(phys_enc) ||
				!_sde_encoder_phys_is_dual_ctl(phys_enc));
}

/**
 * sde_encoder_helper_hw_fence_extended_wait - extended kickoff wait for hw-fence enabled case
 * @phys_enc:	Pointer to physical encoder structure
 * @ctl:	Pointer to hw ctl structure
 * @wait_info:	Pointer to wait_info structure
 * @wait_type:	Enum indicating the irq to wait for
 * Returns:	-ETIMEDOUT in the case that the extended wait times out, 0 otherwise
 */
int sde_encoder_helper_hw_fence_extended_wait(struct sde_encoder_phys *phys_enc,
	struct sde_hw_ctl *ctl, struct sde_encoder_wait_info *wait_info, int wait_type);

/**
 * sde_encoder_helper_phys_disable - helper function to disable virt encoder
 * @phys_enc: Pointer to physical encoder structure
 * @wb_enc: Pointer to writeback encoder structure
 */
void sde_encoder_helper_phys_disable(struct sde_encoder_phys *phys_enc,
		struct sde_encoder_phys_wb *wb_enc);

/**
 * sde_encoder_helper_phys_reset - helper function to reset virt encoder
 *                 if vsync is missing on phys encoder
 * @phys_enc: Pointer to physical encoder structure
 */
void sde_encoder_helper_phys_reset(struct sde_encoder_phys *phys_enc);

/**
 * sde_encoder_helper_setup_misr - helper function to setup misr
 * @phys_enc: Pointer to physical encoder structure
 * @enable: enable/disable flag
 * @frame_count: frame count for misr
 */
void sde_encoder_helper_setup_misr(struct sde_encoder_phys *phys_enc,
		bool enable, u32 frame_count);

/**
 * sde_encoder_helper_collect_misr - helper function to collect misr
 * @phys_enc: Pointer to physical encoder structure
 * @nonblock:  blocking/non-blocking flag
 * @misr_value:  pointer to misr value
 * @Return: zero on success
 */
int sde_encoder_helper_collect_misr(struct sde_encoder_phys *phys_enc,
		bool nonblock, u32 *misr_value);

#endif /* __sde_encoder_phys_H__ */
