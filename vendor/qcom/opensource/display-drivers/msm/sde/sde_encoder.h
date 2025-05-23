/*
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SDE_ENCODER_H__
#define __SDE_ENCODER_H__

#include <drm/drm_crtc.h>
#include <drm/drm_bridge.h>
#include <linux/sde_rsc.h>

#include "msm_prop.h"
#include "msm_drv.h"
#include "sde_hw_mdss.h"
#include "sde_kms.h"
#include "sde_connector.h"
#include "sde_power_handle.h"
#include "sde_cesta.h"

/*
 * Two to anticipate panels that can do cmd/vid dynamic switching
 * plan is to create all possible physical encoder types, and switch between
 * them at runtime
 */
#define NUM_PHYS_ENCODER_TYPES 2

#define MAX_PHYS_ENCODERS_PER_VIRTUAL \
	(MAX_H_TILES_PER_DISPLAY * NUM_PHYS_ENCODER_TYPES)

#define MAX_CHANNELS_PER_ENC 4

#define SDE_ENCODER_FRAME_EVENT_DONE			BIT(0)
#define SDE_ENCODER_FRAME_EVENT_ERROR			BIT(1)
#define SDE_ENCODER_FRAME_EVENT_PANEL_DEAD		BIT(2)
#define SDE_ENCODER_FRAME_EVENT_SIGNAL_RELEASE_FENCE	BIT(3)
#define SDE_ENCODER_FRAME_EVENT_SIGNAL_RETIRE_FENCE	BIT(4)
#define SDE_ENCODER_FRAME_EVENT_CWB_DONE		BIT(5)

#ifdef OPLUS_FEATURE_DISPLAY
#define IDLE_POWERCOLLAPSE_DURATION (80 - 16/2)
#else /* OPLUS_FEATURE_DISPLAY */
#define IDLE_POWERCOLLAPSE_DURATION (66 - 16/2)
#endif /* OPLUS_FEATURE_DISPLAY */
#define IDLE_POWERCOLLAPSE_IN_EARLY_WAKEUP (200 - 16/2)

/* below this fps limit, timeouts are adjusted based on fps */
#define DEFAULT_TIMEOUT_FPS_THRESHOLD            24

#define SDE_ENC_IRQ_REGISTERED(phys_enc, idx) \
		((!(phys_enc) || ((idx) < 0) || ((idx) >= INTR_IDX_MAX)) ? \
		0 : ((phys_enc)->irq[(idx)].irq_idx >= 0))

#define DEFAULT_MIN_FPS	10
/* Seconds to Nanoseconds conversion macro */
#define SEC_TO_NS 1000000000
#define DEVIATION_NS 500000
#define EPT_TIMEOUT_NS 44000000

/*
 * flags to indicate the type of mode switch
 * @SDE_MODE_SWITCH_NONE: not a switch frame
 * @SDE_MODE_SWITCH_FPS_UP: FPS increase switch frame
 * @SDE_MODE_SWITCH_FPS_DOWN: FPS decrease switch frame
 * @SDE_MODE_SWITCH_RES_UP: Resolution up switch frame
 * @SDE_MODE_SWITCH_RES_DOWN: Resolution down switch frame
 */
#define SDE_MODE_SWITCH_NONE		0
#define SDE_MODE_SWITCH_FPS_UP		BIT(0)
#define SDE_MODE_SWITCH_FPS_DOWN	BIT(1)
#define SDE_MODE_SWITCH_RES_UP		BIT(2)
#define SDE_MODE_SWITCH_RES_DOWN	BIT(3)

/**
 * Encoder functions and data types
 * @intfs:	Interfaces this encoder is using, INTF_MODE_NONE if unused
 * @wbs:	Writebacks this encoder is using, INTF_MODE_NONE if unused
 * @needs_cdm:	Encoder requests a CDM based on pixel format conversion needs
 * @display_num_of_h_tiles: Number of horizontal tiles in case of split
 *                          interface
 * @display_type: Type of the display
 * @topology:   Topology of the display
 * @comp_info: Compression parameters information
 */
struct sde_encoder_hw_resources {
	enum sde_intf_mode intfs[INTF_MAX];
	enum sde_intf_mode wbs[WB_MAX];
	bool needs_cdm;
	u32 display_num_of_h_tiles;
	enum sde_connector_display display_type;
	struct msm_display_topology topology;
	struct msm_compression_info *comp_info;
};

/**
 * sde_encoder_kickoff_params - info encoder requires at kickoff
 * @affected_displays:  bitmask, bit set means the ROI of the commit lies within
 *                      the bounds of the physical display at the bit index
 * @recovery_events_enabled: indicates status of client for recoovery events
 * @frame_trigger_mode: indicates frame trigger mode
 */
struct sde_encoder_kickoff_params {
	unsigned long affected_displays;
	bool recovery_events_enabled;
	enum frame_trigger_mode_type frame_trigger_mode;
};

struct sde_encoder_ops {
	/**
	 * phys_init - phys initialization function
	 * @type: controller type
	 * @controller_id: controller id
	 * @phys_init_params: Pointer of structure sde_enc_phys_init_params
	 * Returns: Pointer of sde_encoder_phys, NULL if failed
	 */
	void *(*phys_init)(enum sde_intf_type type, u32 controller_id, void *phys_init_params);
};

/**
 * sde_encoder_init_with_ops - initialize virtual encoder object with init ops
 * @dev:        Pointer to drm device structure
 * @disp_info:  Pointer to display information structure
 * @ops:        Pointer to encoder ops structure
 * @cesta_client: Pointer to sde cesta client
 * Returns:     Pointer to newly created drm encoder
 */
struct drm_encoder *sde_encoder_init_with_ops(struct drm_device *dev,
		struct msm_display_info *disp_info, const struct sde_encoder_ops *ops,
		struct sde_cesta_client *cesta_client);

/*
 * enum arp_sim_mode - arp panel modes
 * @ARP_SIM_FIXED: Watchdog similation to configure TE at fixed time interval
 * @ARP_SIM_RANDOM_GENERATOR: Watchdog similation to configure TE at random time interval
 * @ARP_SIM_FREQ_STEP: Watchdog similation to configure TE at the intervals from freqency step array
 */
enum arp_sim_mode {
	ARP_SIM_FIXED,
	ARP_SIM_RANDOM_GENERATOR,
	ARP_SIM_FREQ_STEP,
	ARP_SIM_MODE_MAX
};

struct sde_sim_arp_panel_mode {
	u32 arp_te_time_in_ms;
	u32 mode;
};

/**
 * sde_encoder_vrr_info - variable refresh info
 * @frame_interval:     Frame interval configuration
 * @curr_freq_pattern:  current frequency patten for frame interval
 *                      the bounds of the physical display at the bit index
 * @curr_idx:          idx of the current pattern being used
 * @current_state:     drm state stored part of store and restore
 * @sim_arp_panel_mode:     ARP simulator mode
 * @debugfs_arp_te_in_ms:   ARP simulator TE value in ms
 * @debugfs_freq_array:    Freqency stepping array provided for simulation
 * @debugfs_freq_pattern:  Frequency pattern provided for simulation
 */
struct sde_encoder_vrr_info {
	u32 frame_interval;
	struct msm_freq_step_pattern *curr_freq_pattern;
	u32 curr_idx;
	struct drm_atomic_state *current_state;
	struct sde_sim_arp_panel_mode sim_arp_panel_mode;
	u32 debugfs_arp_te_in_ms;
	u32 *debugfs_freq_array;
	struct msm_debugfs_freq_pattern *debugfs_freq_pattern;
};

/*
 * enum sde_enc_rc_states - states that the resource control maintains
 * @SDE_ENC_RC_STATE_OFF: Resource is in OFF state
 * @SDE_ENC_RC_STATE_PRE_OFF: Resource is transitioning to OFF state
 * @SDE_ENC_RC_STATE_ON: Resource is in ON state
 * @SDE_ENC_RC_STATE_MODESET: Resource is in modeset state
 * @SDE_ENC_RC_STATE_IDLE: Resource is in IDLE state
 */
enum sde_enc_rc_states {
	SDE_ENC_RC_STATE_OFF,
	SDE_ENC_RC_STATE_PRE_OFF,
	SDE_ENC_RC_STATE_ON,
	SDE_ENC_RC_STATE_MODESET,
	SDE_ENC_RC_STATE_IDLE
};

/*
 * enum sde_sim_qsync_frame - simulated QSYNC frame type
 * @SDE_SIM_QSYNC_FRAME_NOMINAL: Frame is triggered early and TE must come at nominal frame rate.
 * @SDE_SIM_QSYNC_FRAME_EARLY_OR_LATE: Frame could be triggered early or late and TE must adjust
 *                                     accordingly.
 * @SDE_SIM_QSYNC_FRAME_TIMEOUT: Frame is triggered too late and TE must adjust to the
 *                               minimum QSYNC FPS.
 */
enum sde_sim_qsync_frame {
	SDE_SIM_QSYNC_FRAME_NOMINAL,
	SDE_SIM_QSYNC_FRAME_EARLY_OR_LATE,
	SDE_SIM_QSYNC_FRAME_TIMEOUT
};

/*
 * enum sde_sim_qsync_event - events that simulates a QSYNC panel
 * @SDE_SIM_QSYNC_EVENT_FRAME_DETECTED: Event when DDIC is detecting a frame.
 * @SDE_SIM_QSYNC_EVENT_TE_TRIGGER: Event when DDIC is triggering TE signal.
 */
enum sde_sim_qsync_event {
	SDE_SIM_QSYNC_EVENT_FRAME_DETECTED,
	SDE_SIM_QSYNC_EVENT_TE_TRIGGER
};

/*
 * enum sde_multi_te_states - enum to indicate the states of multi-TE
 * @SDE_MULTI_TE_NONE: multi-te not enabled
 * @SDE_MULTI_TE_ENTER: frame entering multi-te
 * @SDE_MULTI_TE_SESSION: frames in multi-te session
 * @SDE_MULTI_TE_EXIT: frame exiting multi-te
 */
enum sde_multi_te_states {
	SDE_MULTI_TE_NONE,
	SDE_MULTI_TE_ENTER,
	SDE_MULTI_TE_SESSION,
	SDE_MULTI_TE_EXIT,
};

/* Frame rate value to trigger the watchdog TE in 200 us */
#define SDE_SIM_QSYNC_IMMEDIATE_FPS 5000

/**
 * struct sde_encoder_virt - virtual encoder. Container of one or more physical
 *	encoders. Virtual encoder manages one "logical" display. Physical
 *	encoders manage one intf block, tied to a specific panel/sub-panel.
 *	Virtual encoder defers as much as possible to the physical encoders.
 *	Virtual encoder registers itself with the DRM Framework as the encoder.
 * @base:		drm_encoder base class for registration with DRM
 * @enc_spin_lock:	Virtual-Encoder-Wide Spin Lock for IRQ purposes
 * @bus_scaling_client:	Client handle to the bus scaling interface
 * @te_source:		vsync source pin information
 * @num_phys_encs:	Actual number of physical encoders contained.
 * @phys_encs:		Container of physical encoders managed.
 * @phys_vid_encs:	Video physical encoders for panel mode switch.
 * @phys_cmd_encs:	Command physical encoders for panel mode switch.
 * @cur_master:		Pointer to the current master in this mode. Optimization
 *			Only valid after enable. Cleared as disable.
 * @hw_pp		Handle to the pingpong blocks used for the display. No.
 *			pingpong blocks can be different than num_phys_encs.
 * @hw_dsc:		Array of DSC block handles used for the display.
 * @hw_vdc:		Array of VDC block handles used for the display.
 * @cur_channel_cnt     Number of data channels currently used for the display
 * @dirty_dsc_ids:	Cached dsc indexes for dirty DSC blocks needing flush
 * @intfs_swapped	Whether or not the phys_enc interfaces have been swapped
 *			for partial update right-only cases, such as pingpong
 *			split where virtual pingpong does not generate IRQs
 * @qdss_status:	indicate if qdss is modified since last update
 * @crtc_vblank_cb:	Callback into the upper layer / CRTC for
 *			notification of the VBLANK
 * @crtc_vblank_cb_data:	Data from upper layer for VBLANK notification
 * @crtc_kickoff_cb:		Callback into CRTC that will flush & start
 *				all CTL paths
 * @crtc_kickoff_cb_data:	Opaque user data given to crtc_kickoff_cb
 * @debugfs_root:		Debug file system root file node
 * @enc_lock:			Lock around physical encoder create/destroy and
				access.
 * @frame_done_cnt:		Atomic counter for tracking which phys_enc is
 *				done with frame processing
 * @crtc_frame_event_cb:	callback handler for frame event
 * @crtc_frame_event_cb_data:	callback handler private data
 * @rsc_client:			rsc client pointer
 * @rsc_state_init:		boolean to indicate rsc config init
 * @disp_info:			local copy of msm_display_info struct
 * @misr_enable:		misr enable/disable status
 * @vsync_cnt:			Vsync count for the virtual encoder
 * @misr_reconfigure:		boolean entry indicates misr reconfigure status
 * @misr_frame_count:		misr frame count before start capturing the data
 * @idle_pc_enabled:		indicate if idle power collapse is enabled
 *				currently. This can be controlled by user-mode
 * @restore_te_rd_ptr:          flag to indicate that te read pointer value must
 *                              be restored after idle power collapse
 * @rc_lock:			resource control mutex lock to protect
 *				virt encoder over various state changes
 * @rc_state:			resource controller state
 * @delayed_off_work:		delayed worker to schedule disabling of
 *				clks and resources after IDLE_TIMEOUT time.
 * @early_wakeup_work:		worker to handle early wakeup event
 * @input_event_work:		worker to handle input device touch events
 * @esd_trigger_work:		worker to handle esd trigger
 * @self_refresh_work:		worker to handle self refresh
 * @self_refresh_work:		worker to handle smooth dimming in vrr
 * @input_handler:			handler for input device events
 * @topology:                   topology of the display
 * @vblank_enabled:		boolean to track userspace vblank vote
 * @idle_pc_restore:		flag to indicate idle_pc_restore happened
 * @frame_trigger_mode:		frame trigger mode indication for command mode
 *				display
 * @dynamic_hdr_updated:	flag to indicate if mempool was unchanged
 * @rsc_config:			rsc configuration for display vtotal, fps, etc.
 * @cur_conn_roi:		current connector roi
 * @prv_conn_roi:		previous connector roi to optimize if unchanged
 * @crtc			pointer to drm_crtc
 * @fal10_veto_override:	software override for micro idle fal10 veto
 * @recovery_events_enabled:	status of hw recovery feature enable by client
 * @elevated_ahb_vote:		increase AHB bus speed for the first frame
 *				after power collapse
 * @pm_qos_cpu_req:		qos request for all cpu core frequency
 * @valid_cpu_mask:		actual voted cpu core mask
 * @mode_info:                  stores the current mode and should be used
 *				only in commit phase
 * @vrr_info:        VRR configuration information
 * @delay_kickoff		boolean to delay the kickoff, used in case
 *				of esd attack to ensure esd workqueue detects
 *				the previous frame transfer completion before
 *				next update is triggered.
 * @autorefresh_solver_disable	It tracks if solver state is disabled from this
 *				encoder due to autorefresh concurrency.
 * @ctl_done_supported          boolean flag to indicate the availability of
 *                              ctl done irq support for the hardware
 * @dynamic_irqs_config         bitmask config to enable encoder dynamic irqs
 * @vsync_event_wq              Queue to wait for the vsync event complete
 * @dpu_ctl_op_sync:		Flag indicating displays attached are enabled in sync mode
 * @ops:                        Encoder ops from init function
 * @old_vsyc_count:             Intf tearcheck vsync_count for old mode.
 * @mode_switch:                flag to indicate its a fps/resolution switch frame.
 * @multi_te_state:             enum to indicate the multi-te states.
 * @multi_te_fps:               refresh rate of multi-TE.
 * @sde_cesta_client:           Point to sde_cesta client for the encoder.
 * @cesta_enable_frame:         Boolean indicating if its first frame after power-collapse/resume
 *				which requires special handling for cesta.
 * @cesta_flush_active:         Boolean indicating cesta override flush_active bit is set
 * @cesta_force_auto_active_db_update:	Boolean indicating auto-active-on-panic is set in SCC
 *					with force-db-update. This is required as a workaround for
 *					cmd mode when previous frame ctl-done is very close to
 *					wakeup/panic windows.
 * @intf_master:		Interface Idx for the master interface
 */
struct sde_encoder_virt {
	struct drm_encoder base;
	spinlock_t enc_spinlock;
	struct mutex vblank_ctl_lock;
	uint32_t bus_scaling_client;

	uint32_t display_num_of_h_tiles;
	uint32_t te_source;

	unsigned int num_phys_encs;
	struct sde_encoder_phys *phys_encs[MAX_PHYS_ENCODERS_PER_VIRTUAL];
	struct sde_encoder_phys *phys_vid_encs[MAX_PHYS_ENCODERS_PER_VIRTUAL];
	struct sde_encoder_phys *phys_cmd_encs[MAX_PHYS_ENCODERS_PER_VIRTUAL];
	struct sde_encoder_phys *cur_master;
	struct sde_hw_pingpong *hw_pp[MAX_CHANNELS_PER_ENC];
	struct sde_hw_dsc *hw_dsc[MAX_CHANNELS_PER_ENC];
	struct sde_hw_vdc *hw_vdc[MAX_CHANNELS_PER_ENC];
	struct sde_hw_pingpong *hw_dsc_pp[MAX_CHANNELS_PER_ENC];
	enum sde_dsc dirty_dsc_ids[MAX_CHANNELS_PER_ENC];
	enum sde_vdc dirty_vdc_ids[MAX_CHANNELS_PER_ENC];
	u32 cur_channel_cnt;
	bool intfs_swapped;
	bool qdss_status;

	void (*crtc_vblank_cb)(void *data, ktime_t ts);
	void *crtc_vblank_cb_data;

	struct dentry *debugfs_root;
	struct mutex enc_lock;
	atomic_t frame_done_cnt[MAX_PHYS_ENCODERS_PER_VIRTUAL];
	void (*crtc_frame_event_cb)(void *data, u32 event, ktime_t ts);
	struct sde_kms_frame_event_cb_data crtc_frame_event_cb_data;

	struct sde_rsc_client *rsc_client;
	bool rsc_state_init;
	struct msm_display_info disp_info;
	atomic_t misr_enable;
	atomic_t vsync_cnt;
	bool misr_reconfigure;
	u32 misr_frame_count;

	bool idle_pc_enabled;
	bool input_event_enabled;
	struct mutex rc_lock;
	enum sde_enc_rc_states rc_state;
	struct kthread_delayed_work delayed_off_work;
	struct kthread_work early_wakeup_work;
	struct kthread_work input_event_work;
	struct kthread_work esd_trigger_work;
	struct kthread_work self_refresh_work;
	struct kthread_work backlight_cmd_work;

	struct input_handler *input_handler;
	bool vblank_enabled;
	bool idle_pc_restore;
	bool restore_te_rd_ptr;
	enum frame_trigger_mode_type frame_trigger_mode;
	bool dynamic_hdr_updated;

	struct sde_rsc_cmd_config rsc_config;
	struct sde_rect cur_conn_roi;
	struct sde_rect prv_conn_roi;
	struct drm_crtc *crtc;

	bool fal10_veto_override;
	bool recovery_events_enabled;
	bool elevated_ahb_vote;
	struct dev_pm_qos_request pm_qos_cpu_req[NR_CPUS];
	struct cpumask valid_cpu_mask;
	struct msm_mode_info mode_info;
	struct sde_encoder_vrr_info vrr_info;
	bool delay_kickoff;
	bool autorefresh_solver_disable;
	bool ctl_done_supported;

	unsigned long dynamic_irqs_config;
	wait_queue_head_t vsync_event_wq;

	bool dpu_ctl_op_sync;
	struct sde_encoder_ops ops;
	u32 mode_switch;
	enum sde_multi_te_states multi_te_state;
	u32 multi_te_fps;
	struct sde_cesta_client *cesta_client;
	bool cesta_enable_frame;
	bool cesta_force_active;
	bool cesta_force_auto_active_db_update;
	bool cesta_reset_intf_master;
	u32 intf_master;
};

#define to_sde_encoder_virt(x) container_of(x, struct sde_encoder_virt, base)

#ifdef OPLUS_FEATURE_DISPLAY
/**
 * sde_encoder_wait_vblack - wait vblack
 * @connector:  Pointer to drm connector structure
 * @drm_enc:      Pointer to drm encoder structure
 * @wait_num:    wait vysnc times
 * @Return:   void.
 */
void sde_encoder_wait_vblack(struct drm_connector *connector, struct drm_encoder *drm_enc, int wait_num);

 /**
 * sde_encoder_pre_kickoff_update_panel_level - update panel backlight before kickoff
 * @connector:   Pointer to drm connector
 * @drm_enc:     structure Pointer to drm encoder structure
 * @Return:     void.
 */
void sde_encoder_pre_kickoff_update_panel_level(struct drm_connector *connector,  struct drm_encoder *drm_enc);

/**
 * sde_encoder_post_kickoff_update_panel_level - update panel backlight after kickoff
 * @connector:    Pointer to drm connector structure
 * @Return:     void.
 */
void sde_encoder_post_kickoff_update_panel_level(struct drm_connector *connector);

/**
 * sde_encoder_update_panel_level - update panel level
 * @connector:    Pointer to drm connector structure
 * @drm_enc:    Pointer to drm encoder structure
 */
void sde_encoder_update_panel_level(struct drm_connector *connector,  struct drm_encoder *drm_enc);
#endif /* OPLUS_FEATURE_DISPLAY */

/**
 * sde_encoder_get_hw_resources - Populate table of required hardware resources
 * @encoder:	encoder pointer
 * @hw_res:	resource table to populate with encoder required resources
 * @conn_state:	report hw reqs based on this proposed connector state
 */
void sde_encoder_get_hw_resources(struct drm_encoder *encoder,
		struct sde_encoder_hw_resources *hw_res,
		struct drm_connector_state *conn_state);

/**
 * sde_encoder_early_wakeup - early wake up display
 * @encoder:	encoder pointer
 */
void sde_encoder_early_wakeup(struct drm_encoder *drm_enc);

/**
 * sde_encoder_early_ept_hint - early wake up hint handling
 * @encoder:	encoder pointer
 * @frame_interval:	frame interval in ns
 * @ept_ns:	EPT value in ns
 */
void sde_encoder_early_ept_hint(struct drm_encoder *drm_enc, u64 frame_interval,
		u64 ept_ns);

/**
 * sde_encoder_handle_hw_fence_error - hw fence error handing in sde encoder
 * @ctl_idx:	control path index
 * @sde_kms:	Pointer to sde_kms
 * @handle:	hash of fence signaled with error
 * @error:	error signaled for fence from hw fence callback
 */
void sde_encoder_handle_hw_fence_error(int ctl_idx, struct sde_kms *sde_kms, u32 handle, int error);

/**
 * sde_encoder_hw_fence_error_handle - fence error handing while hw fence error
 * @drm_enc: Pointer to drm encoder structure
 * return: 0 on success; error code otherwise
 */
int sde_encoder_hw_fence_error_handle(struct drm_encoder *drm_enc);

/**
 * sde_encoder_register_vblank_callback - provide callback to encoder that
 *	will be called on the next vblank.
 * @encoder:	encoder pointer
 * @cb:		callback pointer, provide NULL to deregister and disable IRQs
 * @data:	user data provided to callback
 */
void sde_encoder_register_vblank_callback(struct drm_encoder *encoder,
		void (*cb)(void *, ktime_t), void *data);

/**
 * sde_encoder_register_frame_event_callback - provide callback to encoder that
 *	will be called after the request is complete, or other events.
 * @encoder:	encoder pointer
 * @cb:		callback pointer, provide NULL to deregister
 * @crtc:	pointer to drm_crtc object interested in frame events
 */
void sde_encoder_register_frame_event_callback(struct drm_encoder *encoder,
		void (*cb)(void *, u32, ktime_t), struct drm_crtc *crtc);

/**
 * sde_encoder_get_rsc_client - gets the rsc client state for primary
 *      for primary display.
 * @encoder:	encoder pointer
 */
struct sde_rsc_client *sde_encoder_get_rsc_client(struct drm_encoder *encoder);

/**
 * sde_encoder_poll_line_counts - poll encoder line counts for start of frame
 * @encoder:	encoder pointer
 * @Returns:	zero on success
 */
int sde_encoder_poll_line_counts(struct drm_encoder *encoder);

/**
 * sde_encoder_prepare_for_kickoff - schedule double buffer flip of the ctl
 *	path (i.e. ctl flush and start) at next appropriate time.
 *	Immediately: if no previous commit is outstanding.
 *	Delayed: Block until next trigger can be issued.
 * @encoder:	encoder pointer
 * @params:	kickoff time parameters
 * @Returns:	Zero on success, last detected error otherwise
 */
int sde_encoder_prepare_for_kickoff(struct drm_encoder *encoder,
		struct sde_encoder_kickoff_params *params);

/**
 * sde_encoder_trigger_kickoff_pending - Clear the flush bits from previous
 *        kickoff and trigger the ctl prepare progress for command mode display.
 * @encoder:	encoder pointer
 */
void sde_encoder_trigger_kickoff_pending(struct drm_encoder *encoder);

/**
 * sde_encoder_kickoff - trigger a double buffer flip of the ctl path
 *	(i.e. ctl flush and start) immediately.
 * @encoder:	encoder pointer
 * @config_changed: if true new configuration is applied on the control path
 */
void sde_encoder_kickoff(struct drm_encoder *encoder, bool config_changed);

/**
 * sde_encoder_wait_for_event - Waits for encoder events
 * @encoder:	encoder pointer
 * @event:      event to wait for
 * MSM_ENC_COMMIT_DONE -  Wait for hardware to have flushed the current pending
 *                        frames to hardware at a vblank or wr_ptr_start
 *                        Encoders will map this differently depending on the
 *                        panel type.
 *	                  vid mode -> vsync_irq
 *                        cmd mode -> wr_ptr_start_irq
 * MSM_ENC_TX_COMPLETE -  Wait for the hardware to transfer all the pixels to
 *                        the panel. Encoders will map this differently
 *                        depending on the panel type.
 *                        vid mode -> vsync_irq
 *                        cmd mode -> pp_done
 * Returns: 0 on success, -EWOULDBLOCK if already signaled, error otherwise
 */
int sde_encoder_wait_for_event(struct drm_encoder *drm_encoder,
						enum msm_event_wait event);

/**
 * sde_encoder_idle_request - request for idle request to avoid 4 vsync cycle
 *                            to turn off the clocks.
 * @encoder:	encoder pointer
 * Returns: 0 on success, errorcode otherwise
 */
int sde_encoder_idle_request(struct drm_encoder *drm_enc);

/*
 * sde_encoder_get_fps - get interface frame rate of the given encoder
 * @encoder: Pointer to drm encoder object
 */
u32 sde_encoder_get_fps(struct drm_encoder *encoder);

/*
 * sde_encoder_get_intf_mode - get interface mode of the given encoder
 * @encoder: Pointer to drm encoder object
 */
enum sde_intf_mode sde_encoder_get_intf_mode(struct drm_encoder *encoder);

/*
 * sde_encoder_get_frame_count - get hardware frame count of the given encoder
 * @encoder: Pointer to drm encoder object
 */
u32 sde_encoder_get_frame_count(struct drm_encoder *encoder);

/**
 * sde_encoder_get_avr_status - get combined avr_status from all intfs for given virt encoder
 * @drm_enc: Pointer to drm encoder structure
 */
int sde_encoder_get_avr_status(struct drm_encoder *drm_enc);

/*
 * sde_encoder_get_vblank_timestamp - get the last vsync timestamp
 * @encoder: Pointer to drm encoder object
 * @tvblank: vblank timestamp
 */
bool sde_encoder_get_vblank_timestamp(struct drm_encoder *encoder,
		ktime_t *tvblank);

/**
 * sde_encoder_idle_pc_enter - control enable/disable VSYNC_IN_EN & cache display status at ipc
 * @encoder:	encoder pointer
 */
void sde_encoder_idle_pc_enter(struct drm_encoder *encoder);

/**
 * sde_encoder_virt_restore - restore the encoder configs
 * @encoder:	encoder pointer
 */
void sde_encoder_virt_restore(struct drm_encoder *encoder);

/**
 * sde_encoder_is_dsc_merge - check if encoder is in DSC merge mode
 * @drm_enc: Pointer to drm encoder object
 * @Return: true if encoder is in DSC merge mode
 */
bool sde_encoder_is_dsc_merge(struct drm_encoder *drm_enc);

/**
 * sde_encoder_check_curr_mode - check if given mode is supported or not
 * @drm_enc: Pointer to drm encoder object
 * @mode: Mode to be checked
 * @Return: true if it is cmd mode
 */
bool sde_encoder_check_curr_mode(struct drm_encoder *drm_enc, u32 mode);

/**
 * sde_encoder_init - initialize virtual encoder object
 * @dev:        Pointer to drm device structure
 * @disp_info:  Pointer to display information structure
 * @cesta_client: Pointer to display cesta client
 * Returns:     Pointer to newly created drm encoder
 */
struct drm_encoder *sde_encoder_init(struct drm_device *dev,
		struct msm_display_info *disp_info, struct sde_cesta_client *cesta_client);

/**
 * sde_encoder_destroy - destroy previously initialized virtual encoder
 * @drm_enc:    Pointer to previously created drm encoder structure
 */
void sde_encoder_destroy(struct drm_encoder *drm_enc);

/**
 * sde_encoder_prepare_commit - prepare encoder at the very beginning of an
 *	atomic commit, before any registers are written
 * @drm_enc:    Pointer to previously created drm encoder structure
 */
int sde_encoder_prepare_commit(struct drm_encoder *drm_enc);

/**
 * sde_encoder_update_caps_for_cont_splash - update encoder settings during
 *	device bootup when cont_splash is enabled
 * @drm_enc:    Pointer to drm encoder structure
 * @splash_display: Pointer to sde_splash_display corresponding to this encoder
 * @enable:	boolean indicates enable or displae state of splash
 * @Return:     true if successful in updating the encoder structure
 */
int sde_encoder_update_caps_for_cont_splash(struct drm_encoder *encoder,
		struct sde_splash_display *splash_display, bool enable);

/**
 * sde_encoder_display_failure_notification - update sde encoder state for
 * esd timeout or other display failure notification. This event flows from
 * dsi, sde_connector to sde_encoder.
 *
 * This api must not be called from crtc_commit (display) thread because it
 * requests the flush work on same thread. It is called from esd check thread
 * based on current design.
 *
 *      TODO: manage the event at sde_kms level for forward processing.
 * @drm_enc:    Pointer to drm encoder structure
 * @skip_pre_kickoff:    Caller can avoid pre_kickoff if it is triggering this
 *                       event only to switch the panel TE to watchdog mode.
 * @Return:     true if successful in updating the encoder structure
 */
int sde_encoder_display_failure_notification(struct drm_encoder *enc,
	bool skip_pre_kickoff);

/**
 * sde_encoder_recovery_events_enabled - checks if client has enabled
 * sw recovery mechanism for this connector
 * @drm_enc:    Pointer to drm encoder structure
 * @Return:     true if enabled
 */
bool sde_encoder_recovery_events_enabled(struct drm_encoder *encoder);

/**
 * sde_encoder_enable_recovery_event - handler to enable the sw recovery
 * for this connector
 * @drm_enc:    Pointer to drm encoder structure
 */
void sde_encoder_enable_recovery_event(struct drm_encoder *encoder);
/**
 * sde_encoder_in_clone_mode - checks if underlying phys encoder is in clone
 *	mode or independent display mode. ref@ WB in Concurrent writeback mode.
 * @drm_enc:    Pointer to drm encoder structure
 * @Return:     true if successful in updating the encoder structure
 */
bool sde_encoder_in_clone_mode(struct drm_encoder *enc);

/**
 * sde_encoder_in_video_psr - checks if it is in video psr panel
 * @drm_enc:    Pointer to drm encoder structure
 * @Return:     true if successful
 */
static inline bool sde_encoder_in_video_psr(struct drm_encoder *drm_enc)
{
	struct sde_encoder_virt *sde_enc;

	if (!drm_enc) {
		SDE_ERROR("invalid encoder\n");
		return false;
	}

	sde_enc = to_sde_encoder_virt(drm_enc);

	return sde_enc->disp_info.vrr_caps.video_psr_support;
}
/**
 * sde_encoder_set_clone_mode - cwb in wb phys enc is enabled.
 * drm_enc:	Pointer to drm encoder structure
 * drm_crtc_state:	Pointer to drm_crtc_state
 */
void sde_encoder_set_clone_mode(struct drm_encoder *drm_enc,
	 struct drm_crtc_state *crtc_state);

/*
 * sde_encoder_is_cwb_disabling - check if cwb encoder disable is pending
 * @drm_enc:    Pointer to drm encoder structure
 * @drm_crtc:    Pointer to drm crtc structure
 * @Return: true if cwb encoder disable is pending
 */
bool sde_encoder_is_cwb_disabling(struct drm_encoder *drm_enc,
	struct drm_crtc *drm_crtc);

/**
 * sde_encoder_is_primary_display - checks if underlying display is primary
 *     display or not.
 * @drm_enc:    Pointer to drm encoder structure
 * @Return:     true if it is primary display. false otherwise
 */
bool sde_encoder_is_primary_display(struct drm_encoder *enc);

/**
 * sde_encoder_is_built_in_display - checks if underlying display is built in
 *     display or not.
 * @drm_enc:    Pointer to drm encoder structure
 * @Return:     true if it is a built in display. false otherwise
 */
bool sde_encoder_is_built_in_display(struct drm_encoder *enc);

/**
 * sde_encoder_check_ctl_done_support - checks if ctl_done irq is available
 *		for the display
 * @drm_enc:    Pointer to drm encoder structure
 * @Return:     true if scheduler update is enabled
 */
static inline bool sde_encoder_check_ctl_done_support(struct drm_encoder *drm_enc)
{
	struct sde_encoder_virt *sde_enc = to_sde_encoder_virt(drm_enc);

	return sde_enc && sde_enc->ctl_done_supported;
}

/**
 * sde_encoder_is_dsi_display - checks if underlying display is DSI
 *     display or not.
 * @drm_enc:    Pointer to drm encoder structure
 * @Return:     true if it is a dsi display. false otherwise
 */
bool sde_encoder_is_dsi_display(struct drm_encoder *enc);

/**
 * sde_encoder_control_idle_pc - control enable/disable of idle power collapse
 * @drm_enc:    Pointer to drm encoder structure
 * @enable:	enable/disable flag
 */
void sde_encoder_control_idle_pc(struct drm_encoder *enc, bool enable);

/**
 * sde_encoder_in_cont_splash - checks if display is in continuous splash
 * @drm_enc:    Pointer to drm encoder structure
 * @Return:     true if display in continuous splash
 */
int sde_encoder_in_cont_splash(struct drm_encoder *enc);

/**
 * sde_encoder_helper_hw_reset - hw reset helper function
 * @drm_enc:    Pointer to drm encoder structure
 */
void sde_encoder_needs_hw_reset(struct drm_encoder *enc);

/**
 * sde_encoder_uidle_enable - control enable/disable of uidle
 * @drm_enc:    Pointer to drm encoder structure
 * @enable:	enable/disable flag
 */
void sde_encoder_uidle_enable(struct drm_encoder *drm_enc, bool enable);

/**
 * sde_encoder_irq_control - control enable/disable of IRQ's
 * @drm_enc:	Pointer to drm encoder structure
 * @enable: enable/disable flag
 */
void sde_encoder_irq_control(struct drm_encoder *drm_enc, bool enable);

/**sde_encoder_get_connector - get connector corresponding to encoder
 * @dev:	Pointer to drm device structure
 * @drm_enc:	Pointer to drm encoder structure
 * Returns:	drm connector if found, null if not found
 */
struct drm_connector *sde_encoder_get_connector(struct drm_device *dev,
			struct drm_encoder *drm_enc);

/**sde_encoder_needs_dsc_disable - indicates if dsc should be disabled
 *			based on previous topology
 * @drm_enc:	Pointer to drm encoder structure
 */
bool sde_encoder_needs_dsc_disable(struct drm_encoder *drm_enc);

/**
 * sde_encoder_get_transfer_time - get the mdp transfer time in usecs
 * @drm_enc: Pointer to drm encoder structure
 * @transfer_time_us: Pointer to store the output value
 */
void sde_encoder_get_transfer_time(struct drm_encoder *drm_enc,
		u32 *transfer_time_us);

/**
 * sde_encoder_helper_update_out_fence_txq - updates hw-fence tx queue
 * @sde_enc: Pointer to sde encoder structure
 * @is_vid: Boolean to indicate if is video-mode
 */
void sde_encoder_helper_update_out_fence_txq(struct sde_encoder_virt *sde_enc, bool is_vid);

/*
 * sde_encoder_get_dfps_maxfps - get dynamic FPS max frame rate of
				the given encoder
 * @encoder: Pointer to drm encoder object
 */
static inline u32 sde_encoder_get_dfps_maxfps(struct drm_encoder *drm_enc)
{
	struct sde_encoder_virt *sde_enc;

	if (!drm_enc) {
		SDE_ERROR("invalid encoder\n");
		return 0;
	}

	sde_enc = to_sde_encoder_virt(drm_enc);

	return sde_enc->mode_info.dfps_maxfps;
}

/**
 * sde_encoder_virt_reset - delay encoder virt reset
 * @drm_enc:	Pointer to drm encoder structure
 */
void sde_encoder_virt_reset(struct drm_encoder *drm_enc);

/**
 * sde_encoder_calc_last_vsync_timestamp - read last HW vsync timestamp counter
 *         and calculate the corresponding vsync ktime. Return ktime_get
 *         when HW support is not available
 * @drm_enc:    Pointer to drm encoder structure
 */
ktime_t sde_encoder_calc_last_vsync_timestamp(struct drm_encoder *drm_enc);

/**
 * sde_encoder_cancel_delayed_work - cancel delayed off work for encoder
 * @drm_enc:    Pointer to drm encoder structure
 */
void sde_encoder_cancel_delayed_work(struct drm_encoder *encoder);

/**
 * sde_encoder_get_kms - retrieve the kms from encoder
 * @drm_enc:    Pointer to drm encoder structure
 */
static inline struct sde_kms *sde_encoder_get_kms(struct drm_encoder *drm_enc)
{
	struct msm_drm_private *priv;

	if (!drm_enc || !drm_enc->dev) {
		SDE_ERROR("invalid encoder\n");
		return NULL;
	}
	priv = drm_enc->dev->dev_private;
	if (!priv || !priv->kms) {
		SDE_ERROR("invalid kms\n");
		return NULL;
	}

	return to_sde_kms(priv->kms);
}

/*
 * sde_encoder_is_widebus_enabled - check if widebus is enabled for current mode
 * @drm_enc:    Pointer to drm encoder structure
 * @Return: true if widebus is enabled for current mode
 */
static inline bool sde_encoder_is_widebus_enabled(struct drm_encoder *drm_enc)
{
	struct sde_encoder_virt *sde_enc;

	if (!drm_enc)
		return false;

	sde_enc = to_sde_encoder_virt(drm_enc);
	return sde_enc->mode_info.wide_bus_en;
}

/*
 * sde_encoder_get_pclk_factor - check the value of pclk_factor for current mode
 * @drm_enc:    Pointer to drm encoder structure
 * @Return: the value of pclk_factor for current mode
 */
static inline u32 sde_encoder_get_pclk_factor(struct drm_encoder *drm_enc)
{
	struct sde_encoder_virt *sde_enc;

	if (!drm_enc)
		return false;

	sde_enc = to_sde_encoder_virt(drm_enc);
	return sde_enc->mode_info.pclk_factor;
}

/*
 * sde_encoder_is_line_insertion_supported - get line insertion
 * feature bit value from panel
 * @drm_enc:    Pointer to drm encoder structure
 * @Return: line insertion support status
 */
bool sde_encoder_is_line_insertion_supported(struct drm_encoder *drm_enc);

/**
 * sde_encoder_get_hw_ctl - gets hw ctl from the connector
 * @c_conn: sde connector
 * @Return: pointer to the hw ctl from the encoder upon success, otherwise null
 */
struct sde_hw_ctl *sde_encoder_get_hw_ctl(struct sde_connector *c_conn);

/*
 * sde_encoder_get_programmed_fetch_time - gets the programmable fetch time for video encoders
 * @drm_enc:    Pointer to drm encoder structure
 * @Return: programmable fetch time in microseconds
 */
u32 sde_encoder_get_programmed_fetch_time(struct drm_encoder *encoder);

/**
 * sde_encoder_has_dpu_ctl_op_sync - check if dpu sync is enabled for this encoder
 * @drm_enc:    Pointer to drm encoder structure
 * @Return: true if DPU Interface sync is enabled
 */
static inline bool sde_encoder_has_dpu_ctl_op_sync(struct drm_encoder *drm_enc)
{
	struct sde_encoder_virt *sde_enc;

	if (!drm_enc)
		return false;

	sde_enc = to_sde_encoder_virt(drm_enc);
	return sde_enc->dpu_ctl_op_sync;
}

void sde_encoder_add_data_to_minidump_va(struct drm_encoder *drm_enc);

/**
 * sde_encoder_check_collision - Check if there is SR collision
 *                               at present_time_ns
 * @phys_enc: pointer to physical encoder
 * @present_time_ns: Time in ns at which collision needs to be checked
 */
int sde_encoder_check_collision(struct sde_encoder_phys *phys_enc, u64 present_time_ns);

/**
 * sde_encoder_handle_frequency_stepping - Handle the frequency steppeing
 *                                      pattern requirement
 * @phys_enc: pointer to physical encoder
 * @new_commit: Non zero if it is triggered after new image transfer
 */
void sde_encoder_handle_frequency_stepping(struct sde_encoder_phys *phys_enc, u32 new_commit);

/**
 * sde_encoder_phys_phys_self_refresh_helper - Handle self refresh pattern requirement
 * @timer: pointer to self refresh timer
 */
enum hrtimer_restart sde_encoder_phys_phys_self_refresh_helper(struct hrtimer *timer);

/**
 * sde_encoder_phys_backlight_timer_cb - Handle incremental backlight requirement
 * @timer: pointer to backlight timer
 */
enum hrtimer_restart sde_encoder_phys_backlight_timer_cb(struct hrtimer *timer);

/**
 * sde_encoder_get_freq_pattern - Get the frequency pattern for
 *                               given frame interval and usecase
 * @drm_enc: pointer to drm encoder
 * @frame_interval: Frame interval set by property
 * @usecase_idx: Usecase like video mode set by property
 */
struct msm_freq_step_pattern *sde_encoder_get_freq_pattern(struct drm_encoder *drm_enc,
		u32 frame_interval, u32 usecase_idx);

/**
 * sde_encoder_misr_sign_event_notify - collect MISR, check with previous value
 * if change then notify to client with custom event
 * @drm_enc: pointer to drm encoder
 */
void sde_encoder_misr_sign_event_notify(struct drm_encoder *drm_enc);

/**
 * sde_encoder_handle_dma_fence_out_of_order - sw dma fence out of order signal
 * @drm_enc: pointer to drm encoder
 */
int sde_encoder_handle_dma_fence_out_of_order(struct drm_encoder *drm_enc);

/**
 * sde_encoder_handle_next_backlight_update - handle the consecutive BL update
 * @drm_enc: pointer to drm encoder
 */
void sde_encoder_handle_next_backlight_update(struct drm_encoder *drm_enc);

/**
 * sde_encoder_update_periph_flush - update peripheral flush event
 * @drm_enc: pointer to drm encoder
 */
int sde_encoder_update_periph_flush(struct drm_encoder *drm_enc);

/**
 * sde_encoder_begin_commit - handles begin commit operations in encoder
 * @drm_enc: pointer to drm encoder
 */
void sde_encoder_begin_commit(struct drm_encoder *drm_enc);

/**
 * sde_encoder_complete_commit - handles complete commit operations in encoder
 * @drm_enc: pointer to drm encoder
 */
void sde_encoder_complete_commit(struct drm_encoder *drm_enc);

/**
 * sde_encoder_get_cesta_client - return the SDE CESTA client
 * @drm_enc: pointer to drm encoder
 */
static inline struct sde_cesta_client *sde_encoder_get_cesta_client(struct drm_encoder *drm_enc)
{
	struct sde_encoder_virt *sde_enc = NULL;

	if (!drm_enc || sde_encoder_in_clone_mode(drm_enc))
		return NULL;

	sde_enc = to_sde_encoder_virt(drm_enc);

	return sde_enc->cesta_client;
}

/**
 * sde_encoder_register_misr_event - register or deregister MISR event
 * @drm_enc: pointer to drm encoder
 * @val: indicates register or deregister
 */
static inline int sde_encoder_register_misr_event(struct drm_encoder *drm_enc, bool val)
{
	struct sde_encoder_virt *sde_enc = NULL;

	if (!drm_enc)
		return -EINVAL;

	sde_enc = to_sde_encoder_virt(drm_enc);
	atomic_set(&sde_enc->misr_enable, val);

	/*
	 * To setup MISR ctl reg, set misr_reconfigure as true.
	 * MISR is calculated for the specific number of frames.
	 */
	if (atomic_read(&sde_enc->misr_enable)) {
		sde_enc->misr_reconfigure = true;
		sde_enc->misr_frame_count = 1;
	}

	return 0;
}
#endif /* __SDE_ENCODER_H__ */
