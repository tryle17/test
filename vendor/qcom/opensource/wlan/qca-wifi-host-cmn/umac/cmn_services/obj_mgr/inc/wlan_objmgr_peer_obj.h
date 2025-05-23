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
/**
 * DOC: Define the peer data structure of UMAC
 *	Public APIs to perform operations on Global objects
 */
#ifndef _WLAN_OBJMGR_PEER_OBJ_H_
#define _WLAN_OBJMGR_PEER_OBJ_H_

#include <qdf_types.h>
#include <qdf_atomic.h>
#include "wlan_objmgr_vdev_obj.h"
#ifdef WLAN_FEATURE_11BE_MLO
#include "wlan_mlo_mgr_public_structs.h"
#endif

/* peer flags */
/* authorized for data */
#define WLAN_PEER_F_AUTH                            0x00000001
/* QoS enabled */
#define WLAN_PEER_F_QOS                             0x00000002
/* ERP enabled */
#define WLAN_PEER_F_ERP                             0x00000004
/* HT enabled */
#define WLAN_PEER_F_HT                              0x00000008
/* NB: tWLANhave the same value as IEEE80211_FC1_PWR_MGT */
/* power save mode enabled */
#define WLAN_PEER_F_PWR_MGT                         0x00000010
/* keytsc for node has already been updated */
#define WLAN_PEER_F_TSC_SET                         0x00000020
/* U-APSD power save enabled */
#define WLAN_PEER_F_UAPSD                           0x00000040
/* U-APSD triggerable state */
#define WLAN_PEER_F_UAPSD_TRIG                      0x00000080
/* U-APSD SP in progress */
#define WLAN_PEER_F_UAPSD_SP                        0x00000100
/* Atheros Owl or follow-on device */
#define WLAN_PEER_F_ATH                             0x00000200
/* Owl WDS workaround needed*/
#define WLAN_PEER_F_OWL_WDSWAR                      0x00000400
/* WDS link */
#define WLAN_PEER_F_WDS                             0x00000800
/* No AMPDU support */
#define WLAN_PEER_F_NOAMPDU                         0x00001000
/* wep/tkip aggregation support */
#define WLAN_PEER_F_WEPTKIPAGGR                     0x00002000
#define WLAN_PEER_F_WEPTKIP                         0x00004000
/* temp node (not in the node table) */
#define WLAN_PEER_F_TEMP                            0x00008000
/* 2.4ng VHT interop AMSDU disabled */
#define WLAN_PEER_F_11NG_VHT_INTEROP_AMSDU_DISABLE  0x00010000
/* 40 MHz Intolerant  */
#define WLAN_PEER_F_40MHZ_INTOLERANT                0x00020000
/* node is  paused*/
#define WLAN_PEER_F_PAUSED                          0x00040000
#define WLAN_PEER_F_EXTRADELIMWAR                   0x00080000
/* 20 MHz requesting node */
#define WLAN_PEER_F_REQ_20MHZ                       0x00100000
/* all the tid queues in ath layer are paused*/
#define WLAN_PEER_F_ATH_PAUSED                      0x00200000
/*Require credit update*/
#define WLAN_PEER_F_UAPSD_CREDIT_UPDATE             0x00400000
/*Require send deauth when h/w queue no data*/
#define WLAN_PEER_F_KICK_OUT_DEAUTH                 0x00800000
/* RRM enabled node */
#define WLAN_PEER_F_RRM                             0x01000000
/* Wakeup node */
#define WLAN_PEER_F_WAKEUP                          0x02000000
/* VHT enabled node */
#define WLAN_PEER_F_VHT                             0x04000000
/* deauth/Disassoc wait for node cleanup till frame goes on
   air and tx feedback received */
#define WLAN_PEER_F_DELAYED_CLEANUP                 0x08000000
/* Extended stats enabled node */
#define WLAN_PEER_F_EXT_STATS                       0x10000000
/* Prevent _ieee80211_node_leave() from reentry */
#define WLAN_PEER_F_LEAVE_ONGOING                   0x20000000
/* band steering is enabled for this node */
#define WLAN_PEER_F_BSTEERING_CAPABLE               0x40000000
/* node is a local mesh peer */
#define WLAN_PEER_F_LOCAL_MESH_PEER                 0x80000000

/* MLO enabled peer */
#define WLAN_PEER_FEXT_MLO                          0x00000001
/* Peer is QCN Node */
#define WLAN_PEER_QCN_NODE                          0x00000010
/* Peer is 4 Address node */
#define WLAN_PEER_MESH_NODE                         0x00000020
/* Partner Peer ASSOC rejected */
#define WLAN_PEER_FEXT_ASSOC_REJ                    0x00000040

/**
 * enum wlan_peer_state  - peer state
 * @WLAN_INIT_STATE:       Default state
 * @WLAN_JOIN_STATE:       Station mode, STA is waiting for Join
 * @WLAN_AUTH_STATE:       AUTH in progress
 * @WLAN_ASSOC_STATE:      ASSOC in progress
 * @WLAN_WAITKEY_STATE:    4-way KEY handshake is in progress
 * @WLAN_CONNECTED_STATE:  Connected state
 * @WLAN_PREAUTH_STATE:    Station mode: Preauth
 * @WLAN_DISCONNECT_STATE: Disconnect is in progress
 */
enum wlan_peer_state {
	WLAN_INIT_STATE       = 1,
	WLAN_JOIN_STATE       = 2,
	WLAN_AUTH_STATE       = 3,
	WLAN_ASSOC_STATE      = 4,
	WLAN_WAITKEY_STATE    = 5,
	WLAN_CONNECTED_STATE  = 6,
	WLAN_PREAUTH_STATE    = 7,
	WLAN_DISCONNECT_STATE = 8,
};

/**
 * struct wlan_objmgr_peer_mlme - mlme common data of peer
 * @peer_capinfo:    protocol cap info
 * @peer_flags:      PEER OP flags
 * @peer_ext_flags:  PEER OP ext flags
 * @peer_type:       Type of PEER, (STA/AP/etc.)
 * @phymode:         phy mode of station
 * @max_rate:        Max Rate supported
 * @state:           State of the peer
 * @seq_num:         Sequence number
 * @rssi:            Last received RSSI value
 * @is_authenticated: true if peer is authenticated
 * @assoc_peer:      assoc req/response is handled in this peer
 */
struct wlan_objmgr_peer_mlme {
	uint32_t peer_capinfo;
	uint32_t peer_flags;
	uint32_t peer_ext_flags;
	enum wlan_peer_type peer_type;
	enum wlan_phymode phymode;
	uint32_t max_rate;
	enum wlan_peer_state state;
	uint16_t seq_num;
	int8_t rssi;
	bool is_authenticated;
#ifdef WLAN_FEATURE_11BE_MLO
	bool assoc_peer;
#endif
};

/**
 * struct wlan_objmgr_peer_objmgr - object manager data of peer
 * @vdev:              VDEV pointer to which it is associated
 * @ref_cnt:           Ref count
 * @ref_id_dbg:        Array to track Ref count
 * @print_cnt:         Count to throttle Logical delete prints
 * @trace:             Trace ref and deref
 */
struct wlan_objmgr_peer_objmgr {
	struct wlan_objmgr_vdev *vdev;
	qdf_atomic_t ref_cnt;
#ifdef WLAN_OBJMGR_REF_ID_DEBUG
	qdf_atomic_t ref_id_dbg[WLAN_REF_ID_MAX];
#endif
	uint8_t print_cnt;
#ifdef WLAN_OBJMGR_REF_ID_TRACE
	struct wlan_objmgr_trace trace;
#endif
};

/**
 * struct wlan_objmgr_peer -  PEER common object
 * @psoc_peer:        peer list node for psoc's qdf list
 * @vdev_peer:        peer list node for vdev's qdf list
 * @free_node:        peer list node for free in a delayed work
 * @macaddr:          Peer MAC address
 * @peer_mlme:	      Peer MLME common structure
 * @peer_objmgr:      Peer Object manager common structure
 * @peer_comp_priv_obj:  Component's private object pointers
 * @obj_status:       status of each component object
 * @obj_state:        Status of Peer object
 * @pdev_id:          Pdev ID
 * @peer_lock:        Lock for access/update peer contents
 * @mlo_peer_ctx:     Reference to MLO Peer context
 * @mldaddr:          Peer MLD MAC address
 * @mlo_bridge_peer:  Indicates bridge peer
 * @peer_flags:        QCN flag and 4 address mode flag
 */
struct wlan_objmgr_peer {
	qdf_list_node_t psoc_peer;
	qdf_list_node_t vdev_peer;
#ifdef FEATURE_DELAYED_PEER_OBJ_DESTROY
	qdf_list_node_t free_node;
#endif
	uint8_t macaddr[QDF_MAC_ADDR_SIZE];
	uint8_t pdev_id;
	struct wlan_objmgr_peer_mlme peer_mlme;
	struct wlan_objmgr_peer_objmgr peer_objmgr;
	void *peer_comp_priv_obj[WLAN_UMAC_MAX_COMPONENTS];
	QDF_STATUS obj_status[WLAN_UMAC_MAX_COMPONENTS];
	WLAN_OBJ_STATE obj_state;
	qdf_spinlock_t peer_lock;
#ifdef WLAN_FEATURE_11BE_MLO
	struct wlan_mlo_peer_context *mlo_peer_ctx;
	uint8_t mldaddr[QDF_MAC_ADDR_SIZE];
	u_int32_t peer_flags;
	bool mlo_bridge_peer;
#endif
};

/**
 * wlan_peer_set_phymode() - set phymode
 * @peer: PEER object
 * @phymode: phymode of peer
 *
 * API to set phymode
 *
 * Return: void
 */
void wlan_peer_set_phymode(struct wlan_objmgr_peer *peer,
			   enum wlan_phymode phymode);

/*
 * APIs to Create/Delete Global object APIs
 */
/**
 * wlan_objmgr_peer_obj_create() - peer object create
 * @vdev: VDEV object on which this peer gets created
 * @type: peer type (AP/STA)
 * @macaddr: MAC address
 *
 * Creates Peer object, initializes with default values
 * Attaches to psoc and vdev objects
 * Invokes the registered notifiers to create component object
 *
 * Return: Handle to struct wlan_objmgr_peer on successful creation,
 *         NULL on Failure (on Mem alloc failure and Component objects
 *         Failure)
 */
struct wlan_objmgr_peer *wlan_objmgr_peer_obj_create(
			struct wlan_objmgr_vdev *vdev,
			enum wlan_peer_type type,
			uint8_t macaddr[]);

/**
 * wlan_objmgr_peer_obj_delete() - peer object delete
 * @peer: PEER object
 *
 * Deletes PEER object, removes it from PSOC's, VDEV's peer list
 * Invokes the registered notifiers to destroy component objects
 *
 * Return: SUCCESS/FAILURE
 */
QDF_STATUS wlan_objmgr_peer_obj_delete(struct wlan_objmgr_peer *peer);

#ifdef FEATURE_DELAYED_PEER_OBJ_DESTROY
/**
 * wlan_delayed_peer_obj_free_init() - Init for delayed peer obj freed queue
 * @data: PDEV object
 *
 * Initialize main data structures to process peer obj destroy in a delayed
 * workqueue.
 *
 * Return: QDF_STATUS_SUCCESS on success else a QDF error.
 */
QDF_STATUS wlan_delayed_peer_obj_free_init(void *data);

/**
 * wlan_delayed_peer_obj_free_deinit() - De-Init delayed peer freed processing
 * @data: PDEV object
 *
 * De-initialize main data structures to process peer obj freed in a delayed
 * workqueue.
 *
 * Return: QDF_STATUS_SUCCESS on success else a QDF error.
 */
QDF_STATUS wlan_delayed_peer_obj_free_deinit(void *data);
#else
static inline QDF_STATUS wlan_delayed_peer_obj_free_init(void *data)
{
	return QDF_STATUS_SUCCESS;
}

static inline QDF_STATUS wlan_delayed_peer_obj_free_deinit(void *data)
{
	return QDF_STATUS_SUCCESS;
}
#endif

/*
 * APIs to attach/detach component objects
 */
/**
 * wlan_objmgr_peer_component_obj_attach() - attach comp object to peer
 * @peer: PEER object
 * @id: Component id
 * @comp_priv_obj: component's private object pointer
 * @status: Component's private object creation status
 *
 * API to be used for attaching component object with PEER common object
 *
 * Return: SUCCESS on successful storing of component's object in common object
 *         On FAILURE (appropriate failure codes are returned)
 */
QDF_STATUS wlan_objmgr_peer_component_obj_attach(
		struct wlan_objmgr_peer *peer,
		enum wlan_umac_comp_id id,
		void *comp_priv_obj,
		QDF_STATUS status);

/**
 * wlan_objmgr_peer_component_obj_detach() - detach comp object from peer
 * @peer: PEER object
 * @id: Component id
 * @comp_priv_obj: component's private object pointer
 *
 * API to be used for detaching component object with PEER common object
 *
 * Return: SUCCESS on successful removal of component's object from common
 *         object
 *         On FAILURE (appropriate failure codes are returned)
 */
QDF_STATUS wlan_objmgr_peer_component_obj_detach(
		struct wlan_objmgr_peer *peer,
		enum wlan_umac_comp_id id,
		void *comp_priv_obj);

/*
 * APIs to operations on peer objects
 */

/**
 * wlan_objmgr_trigger_peer_comp_priv_object_creation() - create
 * peer comp object
 * @peer: PEER object
 * @id: Component id
 *
 * API to create component private object in run time, this would
 * be used for features which gets enabled in run time
 *
 * Return: SUCCESS on successful creation
 *         On FAILURE (appropriate failure codes are returned)
 */
QDF_STATUS wlan_objmgr_trigger_peer_comp_priv_object_creation(
		struct wlan_objmgr_peer *peer,
		enum wlan_umac_comp_id id);

/**
 * wlan_objmgr_trigger_peer_comp_priv_object_deletion() - destroy
 * peer comp object
 * @peer: PEER object
 * @id: Component id
 *
 * API to destroy component private object in run time, this would
 * be used for features which gets disabled in run time
 *
 * Return: SUCCESS on successful deletion
 *         On FAILURE (appropriate failure codes are returned)
 */
QDF_STATUS wlan_objmgr_trigger_peer_comp_priv_object_deletion(
		struct wlan_objmgr_peer *peer,
		enum wlan_umac_comp_id id);

/**
 * wlan_objmgr_peer_get_comp_private_obj() - get peer component private object
 * @peer: PEER object
 * @id: Component id
 *
 * API to get component private object
 *
 * Return: void *ptr on SUCCESS
 *         NULL on Failure
 */
void *wlan_objmgr_peer_get_comp_private_obj(
		struct wlan_objmgr_peer *peer,
		enum wlan_umac_comp_id id);

/**
 * wlan_peer_obj_lock() - Acquire PEER spinlock
 * @peer: PEER object
 *
 * API to acquire PEER spin lock
 * Parent lock should not be taken in child lock context
 * but child lock can be taken in parent lock context
 * (for ex: psoc lock can't be invoked in pdev/vdev/peer lock context)
 *
 * Return: void
 */
static inline void wlan_peer_obj_lock(struct wlan_objmgr_peer *peer)
{
	qdf_spin_lock_bh(&peer->peer_lock);
}

/**
 * wlan_peer_obj_unlock() - Release PEER spinlock
 * @peer: PEER object
 *
 * API to Release PEER spin lock
 *
 * Return: void
 */
static inline void wlan_peer_obj_unlock(struct wlan_objmgr_peer *peer)
{
	qdf_spin_unlock_bh(&peer->peer_lock);
}

/**
 * DOC: Examples to use PEER ref count APIs
 *
 * In all the scenarios, the pair of API should be followed
 * other it lead to memory leak
 *
 *  scenario 1:
 *
 *     wlan_objmgr_peer_obj_create()
 *     ----
 *     wlan_objmgr_peer_obj_delete()
 *
 *  scenario 2:
 *
 *     wlan_objmgr_peer_get_ref()
 *     ----
 *     the operations which are done on
 *     peer object
 *     ----
 *     wlan_objmgr_peer_release_ref()
 *
 *  scenario 3:
 *
 *     API to retrieve peer (xxx_get_peer_xxx())
 *     ----
 *     the operations which are done on
 *     peer object
 *     ----
 *     wlan_objmgr_peer_release_ref()
 */

/**
 * wlan_objmgr_peer_get_ref() - increment ref count
 * @peer: PEER object
 * @id:   Object Manager ref debug id
 *
 * API to increment ref count of peer
 *
 * Return: void
 */
#ifdef WLAN_OBJMGR_REF_ID_TRACE
#define wlan_objmgr_peer_get_ref(peer, id) \
		wlan_objmgr_peer_get_ref_debug(peer, id, __func__, __LINE__)

void wlan_objmgr_peer_get_ref_debug(struct wlan_objmgr_peer *peer,
				    wlan_objmgr_ref_dbgid id,
				    const char *func, int line);
#else
void wlan_objmgr_peer_get_ref(struct wlan_objmgr_peer *peer,
					wlan_objmgr_ref_dbgid id);
#endif

/**
 * wlan_objmgr_peer_try_get_ref() - increment ref count, if allowed
 * @peer: PEER object
 * @id:   Object Manager ref debug id
 *
 * API to increment ref count of peer, if object state is valid
 *
 * Return: void
 */
#ifdef WLAN_OBJMGR_REF_ID_TRACE
#define wlan_objmgr_peer_try_get_ref(peer, id) \
		wlan_objmgr_peer_try_get_ref_debug(peer, id, \
		__func__, __LINE__)

QDF_STATUS wlan_objmgr_peer_try_get_ref_debug(struct wlan_objmgr_peer *peer,
					      wlan_objmgr_ref_dbgid id,
					      const char *func, int line);
#else
QDF_STATUS wlan_objmgr_peer_try_get_ref(struct wlan_objmgr_peer *peer,
						 wlan_objmgr_ref_dbgid id);
#endif

/**
 * wlan_objmgr_peer_release_ref() - decrement ref count
 * @peer: PEER object
 * @id:   Object Manager ref debug id
 *
 * API to decrement ref count of peer, if ref count is 1, it initiates the
 * peer deletion
 *
 * Return: void
 */
#ifdef WLAN_OBJMGR_REF_ID_TRACE
#define wlan_objmgr_peer_release_ref(peer, id) \
		wlan_objmgr_peer_release_ref_debug(peer, id, \
		__func__, __LINE__)

void wlan_objmgr_peer_release_ref_debug(struct wlan_objmgr_peer *peer,
					wlan_objmgr_ref_dbgid id,
					const char *func, int line);
#else
void wlan_objmgr_peer_release_ref(struct wlan_objmgr_peer *peer,
						 wlan_objmgr_ref_dbgid id);
#endif

/**
 * wlan_peer_get_next_peer_of_psoc_ref() - get next peer to psoc peer list
 *                                         with lock and ref taken
 * @peer_list: Peer list
 * @hash_index: peer list hash index
 * @peer: PEER object
 * @dbg_id: Ref count debug module id
 *
 * API to get the next peer of given peer (of psoc's peer list)
 *
 * Return: PEER object
 */
#ifdef WLAN_OBJMGR_REF_ID_TRACE
#define wlan_peer_get_next_peer_of_psoc_ref(peer_list, hash_index, peer, \
	dbg_id) \
		wlan_peer_get_next_peer_of_psoc_ref_debug(peer_list, \
		hash_index, peer, dbg_id, __func__, __LINE__)

struct wlan_objmgr_peer *wlan_peer_get_next_peer_of_psoc_ref_debug(
				struct wlan_peer_list *peer_list,
				uint8_t hash_index,
				struct wlan_objmgr_peer *peer,
				wlan_objmgr_ref_dbgid dbg_id,
				const char *func, int line);
#else
struct wlan_objmgr_peer *wlan_peer_get_next_peer_of_psoc_ref(
				struct wlan_peer_list *peer_list,
				uint8_t hash_index,
				struct wlan_objmgr_peer *peer,
				wlan_objmgr_ref_dbgid dbg_id);
#endif

/**
 * wlan_peer_get_next_active_peer_of_psoc() - get next active peer to psoc peer
 *                                            list
 * @peer_list: Peer list
 * @hash_index: peer list hash index
 * @peer: PEER object
 * @dbg_id: Ref count debug module id
 *
 * API to get the next peer of given peer (of psoc's peer list)
 *
 * Return: PEER object
 */
#ifdef WLAN_OBJMGR_REF_ID_TRACE
#define wlan_peer_get_next_active_peer_of_psoc(peer_list, hash_index, \
	peer, dbg_id) \
		wlan_peer_get_next_active_peer_of_psoc_debug(peer_list, \
		hash_index, peer, dbg_id, __func__, __LINE__)

struct wlan_objmgr_peer *wlan_peer_get_next_active_peer_of_psoc_debug(
					struct wlan_peer_list *peer_list,
					uint8_t hash_index,
					struct wlan_objmgr_peer *peer,
					wlan_objmgr_ref_dbgid dbg_id,
					const char *func, int line);
#else
struct wlan_objmgr_peer *wlan_peer_get_next_active_peer_of_psoc(
					struct wlan_peer_list *peer_list,
					uint8_t hash_index,
					struct wlan_objmgr_peer *peer,
					wlan_objmgr_ref_dbgid dbg_id);
#endif

/**
 * wlan_peer_get_next_active_peer_of_vdev() - get next active_peer of vdev list
 * @vdev: VDEV object
 * @peer_list: Peer object list
 * @peer: PEER object
 * @dbg_id: Ref count debug module id
 *
 * API to get the next active peer of given peer (of vdev's peer list)
 *
 * Return: PEER object
 */
#ifdef WLAN_OBJMGR_REF_ID_TRACE
#define wlan_peer_get_next_active_peer_of_vdev(vdev, peer_list, peer, dbg_id) \
		wlan_peer_get_next_active_peer_of_vdev_debug(vdev, peer_list, \
		peer, dbg_id, __func__, __LINE__)

struct wlan_objmgr_peer *wlan_peer_get_next_active_peer_of_vdev_debug(
				struct wlan_objmgr_vdev *vdev,
				qdf_list_t *peer_list,
				struct wlan_objmgr_peer *peer,
				wlan_objmgr_ref_dbgid dbg_id,
				const char *func, int line);
#else
struct wlan_objmgr_peer *wlan_peer_get_next_active_peer_of_vdev(
				struct wlan_objmgr_vdev *vdev,
				qdf_list_t *peer_list,
				struct wlan_objmgr_peer *peer,
				wlan_objmgr_ref_dbgid dbg_id);
#endif

/**
 * wlan_vdev_peer_list_peek_active_head() - get active head of vdev peer list
 * @vdev: VDEV object
 * @peer_list: qdf_list_t
 * @dbg_id: Ref count debug module id
 *
 * API to get the active head peer of given peer (of vdev's peer list)
 *
 * Return: active head peer
 */
#ifdef WLAN_OBJMGR_REF_ID_TRACE
#define wlan_vdev_peer_list_peek_active_head(vdev, peer_list, dbg_id) \
		wlan_vdev_peer_list_peek_active_head_debug(vdev, peer_list, \
		dbg_id, __func__, __LINE__)

struct wlan_objmgr_peer *wlan_vdev_peer_list_peek_active_head_debug(
				struct wlan_objmgr_vdev *vdev,
				qdf_list_t *peer_list,
				wlan_objmgr_ref_dbgid dbg_id,
				const char *func, int line);
#else
struct wlan_objmgr_peer *wlan_vdev_peer_list_peek_active_head(
				struct wlan_objmgr_vdev *vdev,
				qdf_list_t *peer_list,
				wlan_objmgr_ref_dbgid dbg_id);
#endif

/**
 * wlan_psoc_peer_list_peek_head_ref() - get head of psoc peer list
 *                                            with ref and lock protected
 * @peer_list: wlan_peer_list
 * @hash_index: peer list hash index
 * @dbg_id: Ref count debug module id
 *
 * API to get the head peer of given peer (of psoc's peer list)
 *
 * Return: head peer
 */
#ifdef WLAN_OBJMGR_REF_ID_TRACE
#define wlan_psoc_peer_list_peek_head_ref(peer_list, hash_index, dbg_id) \
		wlan_psoc_peer_list_peek_head_ref_debug(peer_list, hash_index, \
		dbg_id, __func__, __LINE__)

struct wlan_objmgr_peer *wlan_psoc_peer_list_peek_head_ref_debug(
					struct wlan_peer_list *peer_list,
					uint8_t hash_index,
					wlan_objmgr_ref_dbgid dbg_id,
					const char *func, int line);

#else
struct wlan_objmgr_peer *wlan_psoc_peer_list_peek_head_ref(
					struct wlan_peer_list *peer_list,
					uint8_t hash_index,
					wlan_objmgr_ref_dbgid dbg_id);
#endif

/**
 * wlan_psoc_peer_list_peek_active_head() - get active head of psoc peer list
 * @peer_list: wlan_peer_list
 * @hash_index: peer list hash index
 * @dbg_id: Ref count debug module id
 *
 * API to get the head peer of given peer (of psoc's peer list)
 *
 * Return: head peer
 */
#ifdef WLAN_OBJMGR_REF_ID_TRACE
#define wlan_psoc_peer_list_peek_active_head(peer_list, hash_index, dbg_id) \
		wlan_psoc_peer_list_peek_active_head_debug(peer_list, \
		hash_index, dbg_id, __func__, __LINE__)

struct wlan_objmgr_peer *wlan_psoc_peer_list_peek_active_head_debug(
					struct wlan_peer_list *peer_list,
					uint8_t hash_index,
					wlan_objmgr_ref_dbgid dbg_id,
					const char *func, int line);
#else
struct wlan_objmgr_peer *wlan_psoc_peer_list_peek_active_head(
					struct wlan_peer_list *peer_list,
					uint8_t hash_index,
					wlan_objmgr_ref_dbgid dbg_id);
#endif

/**
 * wlan_psoc_peer_list_peek_head() - get head of psoc peer list
 * @peer_list: qdf_list_t
 *
 * API to get the head peer of given peer (of psoc's peer list)
 *
 * Caller need to acquire lock with wlan_peer_obj_lock()
 *
 * Return: head peer
 */
static inline struct wlan_objmgr_peer *wlan_psoc_peer_list_peek_head(
					qdf_list_t *peer_list)
{
	struct wlan_objmgr_peer *peer;
	qdf_list_node_t *psoc_node = NULL;

	/* This API is invoked with lock acquired, do not add log prints */
	if (qdf_list_peek_front(peer_list, &psoc_node) != QDF_STATUS_SUCCESS)
		return NULL;

	peer = qdf_container_of(psoc_node, struct wlan_objmgr_peer, psoc_peer);
	return peer;
}

/**
 * wlan_vdev_peer_list_peek_head() - get head of vdev peer list
 * @peer_list: qdf_list_t
 *
 * API to get the head peer of given peer (of vdev's peer list)
 *
 * Caller need to acquire lock with wlan_peer_obj_lock()
 *
 * Return: head peer
 */
static inline struct wlan_objmgr_peer *wlan_vdev_peer_list_peek_head(
					qdf_list_t *peer_list)
{
	struct wlan_objmgr_peer *peer;
	qdf_list_node_t *vdev_node = NULL;

	/* This API is invoked with lock acquired, do not add log prints */
	if (qdf_list_peek_front(peer_list, &vdev_node) != QDF_STATUS_SUCCESS)
		return NULL;

	peer = qdf_container_of(vdev_node, struct wlan_objmgr_peer, vdev_peer);
	return peer;
}

/**
 * wlan_peer_get_next_peer_of_vdev() - get next peer of vdev list
 * @peer_list: PEER object list
 * @peer: PEER object
 *
 * API to get the next peer of given peer (of vdev's peer list)
 *
 * Caller need to acquire lock with wlan_peer_obj_lock()
 *
 * Return: PEER object
 */
static inline struct wlan_objmgr_peer *wlan_peer_get_next_peer_of_vdev(
			qdf_list_t *peer_list, struct wlan_objmgr_peer *peer)
{
	struct wlan_objmgr_peer *peer_next;
	qdf_list_node_t *node;
	qdf_list_node_t *next_node = NULL;

	/* This API is invoked with lock acquired, do not add log prints */
	if (!peer)
		return NULL;

	node = &peer->vdev_peer;
	if (qdf_list_peek_next(peer_list, node, &next_node) !=
						QDF_STATUS_SUCCESS)
		return NULL;

	peer_next = qdf_container_of(next_node, struct wlan_objmgr_peer,
					vdev_peer);
	return peer_next;
}

/**
 * wlan_peer_set_next_peer_of_vdev() - add peer to vdev peer list
 * @peer_list: PEER object list
 * @new_peer: PEER object
 *
 * API to set as the next peer to given peer (of vdev's peer list)
 *
 * Caller need to acquire lock with wlan_peer_obj_lock()
 *
 * Return: void
 */
static inline void wlan_peer_set_next_peer_of_vdev(qdf_list_t *peer_list,
				struct wlan_objmgr_peer *new_peer)
{
	/* This API is invoked with lock acquired, do not add log prints */
	/* set next peer with new peer */
	qdf_list_insert_back(peer_list, &new_peer->vdev_peer);
	return;
}

/**
 * wlan_peer_get_next_peer_of_psoc() - get next peer to psoc peer list
 * @peer_list: Peer list
 * @peer: PEER object
 *
 * API to get the next peer of given peer (of psoc's peer list)
 *
 * Caller need to acquire lock with wlan_peer_obj_lock()
 *
 * Return: PEER object
 */
static inline struct wlan_objmgr_peer *wlan_peer_get_next_peer_of_psoc(
			qdf_list_t *peer_list, struct wlan_objmgr_peer *peer)
{
	struct wlan_objmgr_peer *peer_next;
	qdf_list_node_t *node = NULL;
	qdf_list_node_t *next_node = NULL;

	/* This API is invoked with lock acquired, do not add log prints */
	if (!peer)
		return NULL;

	node = &peer->psoc_peer;
	if (qdf_list_peek_next(peer_list, node, &next_node) !=
						QDF_STATUS_SUCCESS)
		return NULL;

	peer_next = qdf_container_of(next_node, struct wlan_objmgr_peer,
					psoc_peer);
	return peer_next;
}

/**
 * wlan_peer_set_next_peer_of_psoc() - add peer to psoc peer list
 * @peer_list: PEER object list
 * @new_peer: PEER object
 *
 * API to set as the next peer to given peer (of psoc's peer list)
 *
 * Caller need to acquire lock with wlan_peer_obj_lock()
 *
 * Return: void
 */
static inline void wlan_peer_set_next_peer_of_psoc(qdf_list_t *peer_list,
					struct wlan_objmgr_peer *new_peer)
{
	/* This API is invoked with lock acquired, do not add log prints */
	/* set next peer with new peer */
	qdf_list_insert_back(peer_list, &new_peer->psoc_peer);
	return;
}

/**
 * wlan_peer_set_peer_type() - set peer type
 * @peer: PEER object
 * @type: type of PEER
 *
 * API to set peer type
 *
 * Return: void
 */
static inline void wlan_peer_set_peer_type(struct wlan_objmgr_peer *peer,
			enum wlan_peer_type type)
{
	peer->peer_mlme.peer_type = type;
}

/**
 * wlan_peer_get_peer_type() - get peer type
 * @peer: PEER object
 *
 * API to get peer type
 *
 * Return: type of PEER
 */
static inline enum wlan_peer_type wlan_peer_get_peer_type(
				struct wlan_objmgr_peer *peer)
{
	return peer->peer_mlme.peer_type;
}

/**
 * wlan_peer_get_phymode() - get phymode
 * @peer: PEER object
 *
 * API to get phymode
 *
 * Return: phymode of PEER
 */
static inline enum wlan_phymode wlan_peer_get_phymode(
				struct wlan_objmgr_peer *peer)
{
	return peer->peer_mlme.phymode;
}

/**
 * wlan_peer_set_rssi() - set RSSI
 * @peer: PEER object
 * @rssi: RSSI
 *
 * API to set rssi
 *
 * Return: void
 */
static inline void wlan_peer_set_rssi(struct wlan_objmgr_peer *peer,
				      int8_t rssi)
{
	peer->peer_mlme.rssi = rssi;
}

/**
 * wlan_peer_get_rssi() - get RSSI
 * @peer: PEER object
 *
 * API to get RSSI
 *
 * Return: RSSI of PEER
 */
static inline int8_t wlan_peer_get_rssi(
				struct wlan_objmgr_peer *peer)
{
	return peer->peer_mlme.rssi;
}

/**
 * wlan_peer_set_macaddr() - set mac addr
 * @peer: PEER object
 * @macaddr: MAC address
 *
 * API to set peer mac address
 *
 * Caller need to acquire lock with wlan_peer_obj_lock()
 *
 * Return: void
 */
static inline void wlan_peer_set_macaddr(struct wlan_objmgr_peer *peer,
			uint8_t *macaddr)
{
	/* This API is invoked with lock acquired, do not add log prints */
	WLAN_ADDR_COPY(peer->macaddr, macaddr);
}

#ifdef WLAN_FEATURE_DYNAMIC_MAC_ADDR_UPDATE
/**
 * wlan_peer_update_macaddr() - Update peer MAC address
 * @peer: PEER object
 * @new_macaddr: New MAC address
 *
 * API to update peer MAC address and corresponding peer hash entry in PSOC
 * peer list.
 *
 * Return: SUCCESS/FAILURE
 */
QDF_STATUS wlan_peer_update_macaddr(struct wlan_objmgr_peer *peer,
				    uint8_t *new_macaddr);
#endif

/**
 * wlan_peer_get_macaddr() - get mac addr
 * @peer: PEER object
 *
 * API to get peer mac address
 *
 * Caller need to acquire lock with wlan_peer_obj_lock()
 *
 * Return: MAC address
 */
static inline uint8_t *wlan_peer_get_macaddr(struct wlan_objmgr_peer *peer)
{
	/* This API is invoked with lock acquired, do not add log prints */
	return peer->macaddr;
}

/**
 * wlan_peer_get_vdev() - get vdev
 * @peer: PEER object
 *
 * API to get peer's vdev
 *
 * Return: VDEV object
 */
static inline struct wlan_objmgr_vdev *wlan_peer_get_vdev(
			struct wlan_objmgr_peer *peer)
{
	return peer->peer_objmgr.vdev;
}

/**
 * wlan_peer_set_vdev() - set vdev
 * @peer: PEER object
 * @vdev: VDEV object
 *
 * API to set peer's vdev
 *
 * Return: void
 */
static inline void wlan_peer_set_vdev(struct wlan_objmgr_peer *peer,
		struct wlan_objmgr_vdev *vdev)
{
	peer->peer_objmgr.vdev = vdev;
}

/**
 * wlan_peer_mlme_flag_set() - mlme flag set
 * @peer: PEER object
 * @flag: flag to be set
 *
 * API to set flag in peer
 *
 * Return: void
 */
static inline void wlan_peer_mlme_flag_set(struct wlan_objmgr_peer *peer,
					   uint32_t flag)
{
	peer->peer_mlme.peer_flags |= flag;
}

/**
 * wlan_peer_mlme_flag_clear() - mlme flag clear
 * @peer: PEER object
 * @flag: flag to be cleared
 *
 * API to clear flag in peer
 *
 * Return: void
 */
static inline void wlan_peer_mlme_flag_clear(struct wlan_objmgr_peer *peer,
				uint32_t flag)
{
	peer->peer_mlme.peer_flags &= ~flag;
}

/**
 * wlan_peer_mlme_flag_get() - mlme flag get
 * @peer: PEER object
 * @flag: flag to be checked
 *
 * API to know, whether particular flag is set in peer
 *
 * Return: 1 (for set) or 0 (for not set)
 */
static inline uint8_t wlan_peer_mlme_flag_get(struct wlan_objmgr_peer *peer,
				uint32_t flag)
{
	return (peer->peer_mlme.peer_flags & flag) ? 1 : 0;
}

/**
 * wlan_peer_mlme_flag_ext_set() - mlme ext flag set
 * @peer: PEER object
 * @flag: ext flag to be set
 *
 * API to set ext flag in peer
 *
 * Return: void
 */
static inline void wlan_peer_mlme_flag_ext_set(struct wlan_objmgr_peer *peer,
					       uint32_t flag)
{
	peer->peer_mlme.peer_ext_flags |= flag;
}

/**
 * wlan_peer_mlme_flag_ext_clear() - mlme ext flag clear
 * @peer: PEER object
 * @flag: ext flag to be cleared
 *
 * API to clear ext flag in peer
 *
 * Return: void
 */
static inline void wlan_peer_mlme_flag_ext_clear(struct wlan_objmgr_peer *peer,
						 uint32_t flag)
{
	peer->peer_mlme.peer_ext_flags &= ~flag;
}

/**
 * wlan_peer_mlme_flag_ext_get() - mlme ext flag get
 * @peer: PEER object
 * @flag: ext flag to be checked
 *
 * API to know, whether particular ext flag is set in peer
 *
 * Return: 1 (for set) or 0 (for not set)
 */
static inline uint8_t wlan_peer_mlme_flag_ext_get(struct wlan_objmgr_peer *peer,
						  uint32_t flag)
{
	return (peer->peer_mlme.peer_ext_flags & flag) ? 1 : 0;
}

/**
 * wlan_peer_mlme_set_state() - peer mlme state
 * @peer: PEER object
 * @state: enum wlan_peer_state
 *
 * API to update the current peer state
 *
 * Return: void
 */
static inline void wlan_peer_mlme_set_state(
				struct wlan_objmgr_peer *peer,
				enum wlan_peer_state state)
{
	peer->peer_mlme.state = state;
}

#ifdef WLAN_FEATURE_11BE_MLO
/**
 * wlan_peer_mlme_get_mldaddr() - get peer mldaddr
 * @peer: PEER object
 *
 * API to get MLD address from peer object
 *
 * Return: mld address
 */
static inline uint8_t *wlan_peer_mlme_get_mldaddr(struct wlan_objmgr_peer *peer)
{
	return peer->mldaddr;
}

/**
 * wlan_peer_mlme_set_mldaddr() - set peer mldaddr
 * @peer: PEER object
 * @mldaddr: MLD address
 *
 * API to set MLD addr in peer object
 *
 * Return: void
 */
static inline void wlan_peer_mlme_set_mldaddr(struct wlan_objmgr_peer *peer,
					      uint8_t *mldaddr)
{
	WLAN_ADDR_COPY(peer->mldaddr, mldaddr);
}

/**
 * wlan_peer_mlme_set_assoc_peer() - assoc frames is received on this peer
 * @peer: PEER object
 * @is_assoc_link: whether assoc frames is received on this peer or not
 *
 * API to update assoc peer
 *
 * Return: void
 */
static inline void wlan_peer_mlme_set_assoc_peer(
				struct wlan_objmgr_peer *peer,
				bool is_assoc_link)
{
	peer->peer_mlme.assoc_peer = is_assoc_link;
}

/**
 * wlan_peer_mlme_is_assoc_peer() - check peer is assoc peer or not
 * @peer: PEER object
 *
 * API to get assoc peer info
 *
 * Return: whether assoc frames is received on this peer or not
 */
static inline bool wlan_peer_mlme_is_assoc_peer(
				struct wlan_objmgr_peer *peer)
{
	return peer->peer_mlme.assoc_peer;
}
#else

static inline uint8_t *wlan_peer_mlme_get_mldaddr(struct wlan_objmgr_peer *peer)
{
	return NULL;
}

static inline void wlan_peer_mlme_set_mldaddr(struct wlan_objmgr_peer *peer,
					      uint8_t *mldaddr)
{
}

static inline void wlan_peer_mlme_set_assoc_peer(
				struct wlan_objmgr_peer *peer,
				bool is_assoc_link)
{
}

static inline bool wlan_peer_mlme_is_assoc_peer(
				struct wlan_objmgr_peer *peer)
{
	return true;
}
#endif

/**
 * wlan_peer_mlme_set_auth_state() - peer mlme auth state
 * @peer: PEER object
 * @is_authenticated: true or false
 *
 * API to update the current peer auth state
 *
 * Return: void
 */
static inline void wlan_peer_mlme_set_auth_state(
				struct wlan_objmgr_peer *peer,
				bool is_authenticated)
{
	peer->peer_mlme.is_authenticated = is_authenticated;
}

/**
 * wlan_peer_mlme_get_state() - peer mlme state
 * @peer: PEER object
 *
 * API to get peer state
 *
 * Return: enum wlan_peer_state
 */
static inline enum wlan_peer_state wlan_peer_mlme_get_state(
				struct wlan_objmgr_peer *peer)
{
	return peer->peer_mlme.state;
}

/**
 * wlan_peer_mlme_get_auth_state() - peer mlme auth state
 * @peer: PEER object
 *
 * API to get peer auth state
 *
 * Return: auth state true/false
 */
static inline bool wlan_peer_mlme_get_auth_state(
				struct wlan_objmgr_peer *peer)
{
	return peer->peer_mlme.is_authenticated;
}

/**
 * wlan_peer_mlme_get_next_seq_num() - get peer mlme next sequence number
 * @peer: PEER object
 *
 * API to get mlme peer next sequence number
 *
 * Caller need to acquire lock with wlan_peer_obj_lock()
 *
 * Return: peer mlme next sequence number
 */
static inline uint32_t wlan_peer_mlme_get_next_seq_num(
				struct wlan_objmgr_peer *peer)
{
	/* This API is invoked with lock acquired, do not add log prints */
	if (peer->peer_mlme.seq_num < WLAN_MAX_SEQ_NUM)
		peer->peer_mlme.seq_num++;
	else
		peer->peer_mlme.seq_num = 0;

	return peer->peer_mlme.seq_num;
}

/**
 * wlan_peer_mlme_get_seq_num() - get peer mlme sequence number
 * @peer: PEER object
 *
 * API to get mlme peer sequence number
 *
 * Caller need to acquire lock with wlan_peer_obj_lock()
 *
 * Return: peer mlme sequence number
 */
static inline uint32_t wlan_peer_mlme_get_seq_num(
				struct wlan_objmgr_peer *peer)
{
	/* This API is invoked with lock acquired, do not add log prints */
	return peer->peer_mlme.seq_num;
}

/**
 * wlan_peer_mlme_reset_seq_num() - reset peer mlme sequence number
 * @peer: PEER object
 *
 * API to reset peer sequence number
 *
 * Caller need to acquire lock with wlan_peer_obj_lock()
 *
 * Return: void
 */
static inline void wlan_peer_mlme_reset_seq_num(
				struct wlan_objmgr_peer *peer)
{
	/* This API is invoked with lock acquired, do not add log prints */
	peer->peer_mlme.seq_num = 0;
}

/**
 * wlan_peer_get_psoc() - get psoc
 * @peer: PEER object
 *
 * API to get peer's psoc
 *
 * Return: PSOC object or NULL if the psoc can not be found
 */
static inline struct wlan_objmgr_psoc *wlan_peer_get_psoc(
			struct wlan_objmgr_peer *peer)
{
	struct wlan_objmgr_vdev *vdev;
	struct wlan_objmgr_psoc *psoc;

	vdev = wlan_peer_get_vdev(peer);
	if (!vdev)
		return NULL;

	psoc = wlan_vdev_get_psoc(vdev);

	return psoc;
}

/**
 * wlan_peer_get_psoc_id() - get psoc id
 * @peer: PEER object
 *
 * API to get peer's psoc id
 *
 * Return: @psoc_id: psoc id
 */
static inline uint8_t wlan_peer_get_psoc_id(struct wlan_objmgr_peer *peer)
{
	struct wlan_objmgr_psoc *psoc;

	psoc = wlan_peer_get_psoc(peer);

	return wlan_psoc_get_id(psoc);
}

/*
 * wlan_peer_get_pdev_id() - get pdev id
 * @peer: peer object pointer
 *
 * Return: pdev id
 */
static inline uint8_t wlan_peer_get_pdev_id(struct wlan_objmgr_peer *peer)
{
	return peer->pdev_id;
}

/**
 * wlan_peer_set_pdev_id() - set pdev id
 * @peer: peer object pointer
 * @pdev_id: pdev id
 *
 * Return: void
 */
static inline void wlan_peer_set_pdev_id(struct wlan_objmgr_peer *peer,
					 uint8_t pdev_id)
{
	peer->pdev_id = pdev_id;
}

/**
 * wlan_objmgr_print_peer_ref_ids() - print peer object refs
 * @peer: peer object pointer
 * @log_level: log level
 *
 * Return: void
 */
void wlan_objmgr_print_peer_ref_ids(struct wlan_objmgr_peer *peer,
				    QDF_TRACE_LEVEL log_level);

/**
 * wlan_objmgr_peer_get_comp_ref_cnt() - get component ref count for a peer
 * @peer: peer object pointer
 * @id: component id
 *
 * Return: uint32_t
 */
uint32_t
wlan_objmgr_peer_get_comp_ref_cnt(struct wlan_objmgr_peer *peer,
				  enum wlan_umac_comp_id id);

/**
 * wlan_objmgr_peer_trace_init_lock() - Initialize peer trace lock
 * @peer: peer object pointer
 *
 * Return: void
 */
#ifdef WLAN_OBJMGR_REF_ID_TRACE
static inline void
wlan_objmgr_peer_trace_init_lock(struct wlan_objmgr_peer *peer)
{
	wlan_objmgr_trace_init_lock(&peer->peer_objmgr.trace);
}
#else
static inline void
wlan_objmgr_peer_trace_init_lock(struct wlan_objmgr_peer *peer)
{
}
#endif

/**
 * wlan_objmgr_peer_trace_deinit_lock() - Deinitialize peer trace lock
 * @peer: peer object pointer
 *
 * Return: void
 */
#ifdef WLAN_OBJMGR_REF_ID_TRACE
static inline void
wlan_objmgr_peer_trace_deinit_lock(struct wlan_objmgr_peer *peer)
{
	wlan_objmgr_trace_deinit_lock(&peer->peer_objmgr.trace);
}
#else
static inline void
wlan_objmgr_peer_trace_deinit_lock(struct wlan_objmgr_peer *peer)
{
}
#endif

/**
 * wlan_objmgr_peer_trace_del_ref_list() - Delete peer trace reference list
 * @peer: peer object pointer
 *
 * Return: void
 */
#ifdef WLAN_OBJMGR_REF_ID_TRACE
static inline void
wlan_objmgr_peer_trace_del_ref_list(struct wlan_objmgr_peer *peer)
{
	wlan_objmgr_trace_del_ref_list(&peer->peer_objmgr.trace);
}
#else
static inline void
wlan_objmgr_peer_trace_del_ref_list(struct wlan_objmgr_peer *peer)
{
}
#endif

#endif /* _WLAN_OBJMGR_PEER_OBJ_H_*/
