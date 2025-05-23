/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _DSI_CLK_H_
#define _DSI_CLK_H_

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/clk.h>

#define MAX_STRING_LEN 32
#define MAX_DSI_CTRL 2

enum dsi_clk_state {
	DSI_CLK_OFF,
	DSI_CLK_ON,
	DSI_CLK_EARLY_GATE,
};

enum clk_req_client {
	DSI_CLK_REQ_MDP_CLIENT = 0,
	DSI_CLK_REQ_DSI_CLIENT,
};

enum dsi_link_clk_type {
	DSI_LINK_ESC_CLK,
	DSI_LINK_BYTE_CLK,
	DSI_LINK_PIX_CLK,
	DSI_LINK_BYTE_INTF_CLK,
	DSI_LINK_CLK_MAX,
};

enum dsi_link_clk_op_type {
	DSI_LINK_CLK_SET_RATE = BIT(0),
	DSI_LINK_CLK_PREPARE = BIT(1),
	DSI_LINK_CLK_ENABLE = BIT(2),
	DSI_LINK_CLK_START = BIT(0) | BIT(1) | BIT(2),
};

enum dsi_clk_type {
	DSI_CORE_CLK = BIT(0),
	DSI_LINK_CLK = BIT(1),
	DSI_ESYNC_CLK = BIT(2),
	DSI_OSC_CLK = BIT(3),
	DSI_CLKS_MAX = BIT(4),
};

enum dsi_lclk_type {
	DSI_LINK_NONE = 0,
	DSI_LINK_LP_CLK = BIT(0),
	DSI_LINK_HS_CLK = BIT(1),
};

struct dsi_clk_ctrl_info {
	enum dsi_clk_type clk_type;
	enum dsi_clk_state clk_state;
	enum clk_req_client client;
};

struct clk_ctrl_cb {
	void *priv;
	int (*dsi_clk_cb)(void *priv, struct dsi_clk_ctrl_info clk_ctrl_info);
};

/**
 * struct dsi_core_clk_info - Core clock information for DSI hardware
 * @mdp_core_clk:        Handle to MDP core clock.
 * @iface_clk:           Handle to MDP interface clock.
 * @core_mmss_clk:       Handle to MMSS core clock.
 * @bus_clk:             Handle to bus clock.
 * @mnoc_clk:            Handle to MMSS NOC clock.
 * @drm:                 Pointer to drm device node
 */
struct dsi_core_clk_info {
	struct clk *mdp_core_clk;
	struct clk *iface_clk;
	struct clk *core_mmss_clk;
	struct clk *bus_clk;
	struct clk *mnoc_clk;
	struct drm_device *drm;
};

/**
 * struct dsi_link_hs_clk_info - Set of high speed link clocks for DSI HW
 * @byte_clk:        Handle to DSI byte_clk.
 * @pixel_clk:       Handle to DSI pixel_clk.
 * @byte_intf_clk:   Handle to DSI byte intf. clock.
 */
struct dsi_link_hs_clk_info {
	struct clk *byte_clk;
	struct clk *pixel_clk;
	struct clk *byte_intf_clk;
};

/**
 * struct dsi_link_lp_clk_info - Set of low power link clocks for DSI HW.
 * @esc_clk:         Handle to DSI escape clock.
 */
struct dsi_link_lp_clk_info {
	struct clk *esc_clk;
};

/**
 * struct link_clk_freq - Clock frequency information for Link clocks
 * @byte_clk_rate:   Frequency of DSI byte_clk in Hz.
 * @byte_intf_clk_rate:   Frequency of DSI byte_intf_clk in Hz.
 * @pixel_clk_rate:  Frequency of DSI pixel_clk in Hz.
 * @esc_clk_rate:    Frequency of DSI escape clock in Hz.
 */
struct link_clk_freq {
	u32 byte_clk_rate;
	u32 byte_intf_clk_rate;
	u32 pix_clk_rate;
	u32 esc_clk_rate;
};

/**
 * struct dsi_esync_clk_info - Clock information for esync clock
 * @clk:             Handle to esync clock.
 */
struct dsi_esync_clk_info {
	struct clk *clk;
};

/**
 * struct dsi_osc_clk_info - Clock information for oscillator clock
 * @clk:             Handle to oscillator clock.
 */
struct dsi_osc_clk_info {
	struct clk *clk;
};

/**
 * typedef *pre_clockoff_cb() - Callback before clock is turned off
 * @priv: private data pointer.
 * @clk_type: clock which is being turned off.
 * @l_type: specifies if the clock is HS or LP type. Valid only for link clocks.
 * @new_state: next state for the clock.
 *
 * @return: error code.
 */
typedef int (*pre_clockoff_cb)(void *priv,
			       enum dsi_clk_type clk_type,
			       enum dsi_lclk_type l_type,
			       enum dsi_clk_state new_state);

/**
 * typedef *post_clockoff_cb() - Callback after clock is turned off
 * @priv: private data pointer.
 * @clk_type: clock which was turned off.
 * @l_type: specifies if the clock is HS or LP type. Valid only for link clocks.
 * @curr_state: current state for the clock.
 *
 * @return: error code.
 */
typedef int (*post_clockoff_cb)(void *priv,
				enum dsi_clk_type clk_type,
				enum dsi_lclk_type l_type,
				enum dsi_clk_state curr_state);

/**
 * typedef *post_clockon_cb() - Callback after clock is turned on
 * @priv: private data pointer.
 * @clk_type: clock which was turned on.
 * @l_type: specifies if the clock is HS or LP type. Valid only for link clocks.
 * @curr_state: current state for the clock.
 *
 * @return: error code.
 */
typedef int (*post_clockon_cb)(void *priv,
			       enum dsi_clk_type clk_type,
			       enum dsi_lclk_type l_type,
			       enum dsi_clk_state curr_state);

/**
 * typedef *pre_clockon_cb() - Callback before clock is turned on
 * @priv: private data pointer.
 * @clk_type: clock which is being turned on.
 * @l_type: specifies if the clock is HS or LP type.Valid only for link clocks.
 * @new_state: next state for the clock.
 *
 * @return: error code.
 */
typedef int (*pre_clockon_cb)(void *priv,
			      enum dsi_clk_type clk_type,
			      enum dsi_lclk_type l_type,
			      enum dsi_clk_state new_state);


/**
 * typedef *phy_configure_cb() - Callback to configure PHY for PLL clocks
 * @priv: private data pointer.
 * @commit: boolean to specify if calculated PHY configuration needs to be
 *          committed. Set to false in case of dynamic clock switch.
 *
 * @return: error code.
 */
typedef int (*phy_configure_cb)(void *priv, bool commit);

/**
 * typedef *pll_toggle_cb() - Callback to toggle PHY PLL
 * @priv: private data pointer.
 * @prepare: specifies if the PLL needs to be turned on or off.
 *
 * @return: error code.
 */
typedef int (*pll_toggle_cb)(void *priv, bool prepare);

/**
 * struct dsi_clk_info - clock information for DSI hardware.
 * @name:                    client name.
 * @c_clks[MAX_DSI_CTRL]     array of core clock configurations
 * @l_lp_clks[MAX_DSI_CTRL]  array of low power(esc) clock configurations
 * @l_hs_clks[MAX_DSI_CTRL]  array of high speed clock configurations
 * @e_clks[MAX_DSI_CTRL]     array of esync clock configurations
 * @o_clk                    oscillator clock configuration
 * @ctrl_index[MAX_DSI_CTRL] array of DSI controller indexes mapped
 *                           to core and link clock configurations
 * @pre_clkoff_cb            callback before clock is turned off
 * @post_clkoff_cb           callback after clock is turned off
 * @post_clkon_cb            callback after clock is turned on
 * @pre_clkon_cb             callback before clock is turned on
 * @phy_config_cb            callback to configure PHY PLL
 * @phy_pll_toggle_cb        callback to toggle PHY PLL state
 * @priv_data                pointer to private data
 * @master_ndx               master DSI controller index
 * @dsi_ctrl_count           number of DSI controllers
 * @phy_pll_bypass           bypass PLL clock related operations
 */
struct dsi_clk_info {
	char name[MAX_STRING_LEN];
	struct dsi_core_clk_info c_clks[MAX_DSI_CTRL];
	struct dsi_link_lp_clk_info l_lp_clks[MAX_DSI_CTRL];
	struct dsi_link_hs_clk_info l_hs_clks[MAX_DSI_CTRL];
	struct dsi_esync_clk_info e_clks[MAX_DSI_CTRL];
	struct dsi_osc_clk_info o_clk;
	u32 ctrl_index[MAX_DSI_CTRL];
	pre_clockoff_cb pre_clkoff_cb;
	post_clockoff_cb post_clkoff_cb;
	post_clockon_cb post_clkon_cb;
	pre_clockon_cb pre_clkon_cb;
	phy_configure_cb phy_config_cb;
	pll_toggle_cb phy_pll_toggle_cb;
	void *priv_data;
	u32 master_ndx;
	u32 dsi_ctrl_count;
	bool phy_pll_bypass;
};

/**
 * struct dsi_clk_link_set - Pair of clock handles to describe link clocks
 * @byte_clk:     Handle to DSi byte_clk.
 * @pixel_clk:    Handle to DSI pixel_clk.
 */
struct dsi_clk_link_set {
	struct clk *byte_clk;
	struct clk *pixel_clk;
};

/**
 * dsi_display_clk_mngr_update_splash_status() - Update splash stattus
 * @clk_mngr:     Structure containing DSI clock information
 * @status:     Splash status
 */
void dsi_display_clk_mngr_update_splash_status(void *clk_mgr, bool status);

/**
 * dsi_display_clk_mgr_register() - Register DSI clock manager
 * @info:     Structure containing DSI clock information
 */
void *dsi_display_clk_mngr_register(struct dsi_clk_info *info);

/**
 * dsi_display_clk_mngr_deregister() - Deregister DSI clock manager
 * @clk_mngr:  DSI clock manager pointer
 */
int dsi_display_clk_mngr_deregister(void *clk_mngr);

/**
 * dsi_register_clk_handle() - Register clock handle with DSI clock manager
 * @clk_mngr:  DSI clock manager pointer
 * @client:     DSI clock client pointer.
 */
void *dsi_register_clk_handle(void *clk_mngr, char *client);

/**
 * dsi_deregister_clk_handle() - Deregister clock handle from DSI clock manager
 * @client:     DSI clock client pointer.
 *
 * return: error code in case of failure or 0 for success.
 */
int dsi_deregister_clk_handle(void *client);

/**
 * dsi_display_link_clk_force_update_ctrl() - force to set link clks
 * @handle:     Handle of desired DSI clock client.
 *
 * return: error code in case of failure or 0 for success.
 */

int dsi_display_link_clk_force_update_ctrl(void *handle);

/**
 * dsi_display_clk_ctrl() - set frequencies for link clks
 * @handle:     Handle of desired DSI clock client.
 * @clk_type:   Clock which is being controlled.
 * @clk_state:  Desired state of clock
 *
 * return: error code in case of failure or 0 for success.
 */
int dsi_display_clk_ctrl(void *handle, u32 clk_type, u32 clk_state);

/**
 * dsi_display_clk_ctrl_nolock() - set frequencies for link clks
 * @handle:     Handle of desired DSI clock client.
 * @clk_type:   Clock which is being controlled.
 * @clk_state:  Desired state of clock
 *
 * return: error code in case of failure or 0 for success.
 */
int dsi_display_clk_ctrl_nolock(void *handle, u32 clk_type, u32 clk_state);

/**
 * dsi_display_clk_mngr_get_clk_rate() - retrieve clock rate
 * @handle:     Handle of desired DSI clock client.
 * @idx:        Index of desired DSI controller.
 * @clk_type:   Clock whose rate is being retrieved.
 * @clk_rate:   Pointer to which the rate will be written on success
 *
 * return: error code in case of failure or 0 for success.
 */
int dsi_display_clk_mngr_get_clk_rate(void *handle, u32 idx,
		enum dsi_clk_type clk_type, u64 *clk_rate);

/**
 * dsi_clk_set_link_frequencies() - set frequencies for link clks
 * @client:     DSI clock client pointer.
 * @freq:       Structure containing link clock frequencies.
 * @index:      Index of the DSI controller.
 *
 * return: error code in case of failure or 0 for success.
 */
int dsi_clk_set_link_frequencies(void *client, struct link_clk_freq freq,
					u32 index);


/**
 * dsi_clk_set_pixel_clk_rate() - set frequency for pixel_clk
 * @client:       DSI clock client pointer.
 * @pixel_clk:    Pixel_clk rate in Hz.
 * @index:        Index of the DSI controller.
 * return: error code in case of failure or 0 for success.
 */
int dsi_clk_set_pixel_clk_rate(void *client, u64 pixel_clk, u32 index);


/**
 * dsi_clk_set_byte_clk_rate() - set frequency for byte clock
 * @client:       DSI clock client pointer.
 * @byte_clk: Pixel clock rate in Hz.
 * @byte_intf_clk: Byte interface clock rate in Hz.
 * @index:      Index of the DSI controller.
 * return: error code in case of failure or 0 for success.
 */
int dsi_clk_set_byte_clk_rate(void *client, u64 byte_clk,
				u64 byte_intf_clk, u32 index);

/**
 * dsi_clk_update_parent() - update parent clocks for specified clock
 * @parent:       link clock pair which are set as parent.
 * @child:        link clock pair whose parent has to be set.
 */
int dsi_clk_update_parent(struct dsi_clk_link_set *parent,
			  struct dsi_clk_link_set *child);

/**
 * dsi_clk_prepare_enable() - prepare and enable dsi src clocks
 * @clk:       list of src clocks.
 *
 * @return:	Zero on success and err no on failure
 */
int dsi_clk_prepare_enable(struct dsi_clk_link_set *clk);

/**
 * dsi_clk_disable_unprepare() - disable and unprepare dsi src clocks
 * @clk:       list of src clocks.
 */
void dsi_clk_disable_unprepare(struct dsi_clk_link_set *clk);

/**
 * dsi_display_dump_clk_handle_state() - dump client clock state
 * @client:       DSI clock client pointer.
 */
int dsi_display_dump_clk_handle_state(void *client);

#endif /* _DSI_CLK_H_ */
