/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef NVT_H_NT36528_NOFLASH
#define NVT_H_NT36528_NOFLASH

/*********PART1:Head files**********************/
#include <linux/spi/spi.h>
#ifdef CONFIG_SPI_MT65XX
#include <linux/platform_data/spi-mt65xx.h>
#endif

#include "../novatek_common.h"

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#include "mtk_spi.h"
#else
#include "oplus_spi.h"
#endif

#ifdef TPD_DEVICE
#undef TPD_DEVICE
#define TPD_DEVICE "novatek,nf_nt36528"
#else
#define TPD_DEVICE "novatek,nf_nt36528"
#endif

#define DRIVER_NAME "nt36528"
#define PEN_DATA_LEN 14
#define POINT_DATA_CHECKSUM_LEN 65
#define NVT_TOUCH_ESD_CHECK_PERIOD (2000)
#define NVT_ID_BYTE_MAX 6
#define POINT_DATA_LEN 120
#define SIZE_4KB 4096
#define FLASH_SECTOR_SIZE SIZE_4KB
/*#define FW_BIN_VER_OFFSET (fw_need_write_size - SIZE_4KB)*/
/*#define FW_BIN_VER_BAR_OFFSET (FW_BIN_VER_OFFSET + 1)*/
#define NVT_FLASH_END_FLAG_LEN 3
#define NVT_FLASH_END_FLAG_ADDR (fw_need_write_size - NVT_FLASH_END_FLAG_LEN)
#define FW_BIN_CHECKSUM_LEN     (4)
#define FW_BUF_SIZE             (256 * 1024)
#define NVT_PM_WAIT_I2C_SPI_RESUME_COMPLETE (0)


#define NVT_MMAP_DEBUG_FINGER_DOWN_DIFFDATA   (0x26A78) /*debug finger diff (finger down)  */
#define NVT_MMAP_DEBUG_STATUS_CHANGE_DIFFDATA (0x26CB8) /*debug finger diff (status change)*/

#define W_DETECT                        13
#define UP_VEE_DETECT                   14
#define DTAP_DETECT                     15
#define M_DETECT                        17
#define CIRCLE_DETECT                   18
#define S_DETECT                        20
#define UP_SLIDE_DETECT                 21
#define DOWN_SLIDE_DETECT               22
#define LEFT_SLIDE_DETECT               23
#define RIGHT_SLIDE_DETECT              24
#define SINGLE_DETECT                   27

#define LEFT_VEE_DETECT                 31
#define RIGHT_VEE_DETECT                32
#define DOWN_VEE_DETECT                 33
#define DOUSWIP_DETECT                  34
#define PEN_DETECT                      25

#define EVENTBUFFER_PWR_PLUG_IN          0x53
#define EVENTBUFFER_PWR_PLUG_OUT         0x51
#define EVENTBUFFER_HOPPING_POLLING_ON   0x73
#define EVENTBUFFER_HOPPING_POLLING_OFF  0x74
#define EVENTBUFFER_HOPPING_FIX_FREQ_ON  0x75
#define EVENTBUFFER_HOPPING_FIX_FREQ_OFF 0x76
#define EVENTBUFFER_HS_PLUG_IN           0x77
#define EVENTBUFFER_HS_PLUG_OUT          0x78
#define EVENTBUFFER_EDGE_LIMIT_VERTICAL_REVERSE  0x79
#define EVENTBUFFER_EDGE_LIMIT_VERTICAL  0x7A
#define EVENTBUFFER_EDGE_LIMIT_LEFT_UP   0x7B
#define EVENTBUFFER_EDGE_LIMIT_RIGHT_UP  0x7C
#define EVENTBUFFER_GAME_ON            0x7D
#define EVENTBUFFER_GAME_OFF           0x7E

#define EVENTBUFFER_EXT_CMD                       0x7F
#define EVENTBUFFER_EXT_DBG_MSG_DIFF_ON           0x01
#define EVENTBUFFER_EXT_DBG_MSG_DIFF_OFF          0x02
#define EVENTBUFFER_EXT_DBG_WKG_COORD_ON          0x03
#define EVENTBUFFER_EXT_DBG_WKG_COORD_OFF         0x04
#define EVENTBUFFER_EXT_DBG_WKG_COORD_RECORD_ON   0x05
#define EVENTBUFFER_EXT_DBG_WKG_COORD_RECORD_OFF  0x06
#define EVENTBUFFER_EXT_DBG_WATER_POLLING_ON      0x07
#define EVENTBUFFER_EXT_DBG_WATER_POLLING_OFF     0x08
#define EVENTBUFFER_EXT_FOD_ON                    0x09
#define EVENTBUFFER_EXT_FOD_OFF                   0x0A
#define EVENTBUFFER_EXT_JITTER_LEVEL              0x0C
#define EVENTBUFFER_EXT_SMOOTH_LEVEL              0x0D
#define EVENTBUFFER_EXT_EDGE_PARM1                0x0E
#define EVENTBUFFER_EXT_EDGE_PARM2                0x0F
#define EVENTBUFFER_EXT_PEN_MODE_ON               0x10
#define EVENTBUFFER_EXT_PEN_MODE_OFF              0x11
#define EVENTBUFFER_EXT_REPORT_RATE               0x14
#define EVENTBUFFER_EXT_SMOOTH_WITH_CHARGER_LEVEL 0x20
#define EVENTBUFFER_EXT_AOD_ON_OFF                0x21
#define EVENTBUFFER_EXT_AOD_SETTING1              0x22
#define EVENTBUFFER_EXT_AOD_SETTING2              0x23
#define EVENTBUFFER_EXT_FILM_WATERPROOF           0x24
#define EVENTBUFFER_EXT_DBG_STATUS_WATERPROOF     0x5D

#define NVT_TOUCH_FW_DEBUG_INFO (1)
#define NVT_DUMP_SRAM   (0)

#define SPI_TANSFER_LENGTH 256
#define NVT_TRANSFER_LEN (15*1024)

#define XDATA_SECTOR_SIZE       256
#define NORMAL_MODE             0x00
#define TEST_MODE_1             0x21
#define TEST_MODE_2             0x22
#define MP_MODE_CC              0x41
#define FREQ_HOP_DISABLE        0x66
#define FREQ_HOP_ENABLE         0x65
#define HANDSHAKING_HOST_READY  0xBB

#define CMD_OPEN_BLACK_GESTURE  0x13
#define CMD_ENTER_SLEEP         0x11

/* define these Macro for get fw history */
#define STATUS_FINGER_ENTER  0x01
#define STATUS_FINGER_MOVING 0x02
#define STATUS_FINGER_FOD    0x03

/* define for fw event buffer protocol */
#define NVT_EVENTBUF_PROT_HIGH_RESO 0xF1

typedef enum {
	NVT_RAWDATA,    /*raw data       */
	NVT_DIFFDATA,   /*diff data      */
	NVT_BASEDATA,   /*baseline data  */
	NVT_DEBUG_FINGER_DOWN_DIFFDATA,   /*debug finger diff (finger down)    */
	NVT_DEBUG_STATUS_CHANGE_DIFFDATA, /*debug finger diff (status change)  */
} DEBUG_READ_TYPE;

typedef enum {
	EDGE_REJECT_L = 0,
	EDGE_REJECT_H = 1,
	PWR_FLAG = 2,
	HOPPING_FIX_FREQ_FLAG = 3,
	HOPPING_POLLING_FLAG = 4,
	JITTER_FLAG = 6,
	HEADSET_FLAG = 7
} CMD_OFFSET;

typedef enum {
	DEBUG_DIFFDATA_FLAG = 0,
	DEBUG_WKG_COORD_FLAG = 1,
	DEBUG_WKG_COORD_RECORD_FLAG = 2,
	DEBUG_WATER_POLLING_FLAG = 3
} CMD_EXTEND_OFFSET;

typedef enum {
	EVENT_MAP_HOST_CMD                      = 0x50,
	EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
	EVENT_MAP_RESET_COMPLETE                = 0x60,
	EVENT_MAP_FWINFO                        = 0x78,
	EVENT_MAP_PROJECTID                     = 0x9A,
} SPI_EVENT_MAP;

typedef enum {
	RESET_STATE_INIT = 0xA0,/* IC reset          */
	RESET_STATE_REK,        /* ReK baseline      */
	RESET_STATE_REK_FINISH, /* baseline is ready */
	RESET_STATE_NORMAL_RUN, /* normal run        */
	RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
	NVT_MP_PASS = 0,
	NVT_MP_FAIL = 1,
	NVT_MP_FAIL_READ_DATA = 2,
	NVT_MP_UNKNOWN = 3
} NVT_MP_TEST_RESULT;

typedef enum {
	HWCRC_NOSUPPORT  = 0x00,
	HWCRC_LEN_2Bytes = 0x01,
	HWCRC_LEN_3Bytes = 0x02,
} HWCRCBankConfig;

typedef enum {
	AUTOCOPY_NOSUPPORT    = 0x00,
	CHECK_SPI_DMA_TX_INFO = 0x01,
	CHECK_TX_AUTO_COPY_EN = 0x02,
} AUTOCOPYCheck;

typedef struct nvt_ts_reg {
	uint32_t addr; /* byte in which address */
	uint8_t mask; /* in which bits of that byte */
} nvt_ts_reg_t;

struct nvt_ts_mem_map {
	uint32_t EVENT_BUF_ADDR;
	uint32_t RAW_PIPE0_ADDR;
	uint32_t RAW_PIPE1_ADDR;
	uint32_t BASELINE_ADDR;
	uint32_t BASELINE_BTN_ADDR;
	uint32_t DIFF_PIPE0_ADDR;
	uint32_t DIFF_PIPE1_ADDR;
	uint32_t RAW_BTN_PIPE0_ADDR;
	uint32_t RAW_BTN_PIPE1_ADDR;
	uint32_t DIFF_BTN_PIPE0_ADDR;
	uint32_t DIFF_BTN_PIPE1_ADDR;
	uint32_t PEN_2D_BL_TIP_X_ADDR;
	uint32_t PEN_2D_BL_TIP_Y_ADDR;
	uint32_t PEN_2D_BL_RING_X_ADDR;
	uint32_t PEN_2D_BL_RING_Y_ADDR;
	uint32_t PEN_2D_DIFF_TIP_X_ADDR;
	uint32_t PEN_2D_DIFF_TIP_Y_ADDR;
	uint32_t PEN_2D_DIFF_RING_X_ADDR;
	uint32_t PEN_2D_DIFF_RING_Y_ADDR;
	uint32_t PEN_2D_RAW_TIP_X_ADDR;
	uint32_t PEN_2D_RAW_TIP_Y_ADDR;
	uint32_t PEN_2D_RAW_RING_X_ADDR;
	uint32_t PEN_2D_RAW_RING_Y_ADDR;
	uint32_t PEN_1D_DIFF_TIP_X_ADDR;
	uint32_t PEN_1D_DIFF_TIP_Y_ADDR;
	uint32_t PEN_1D_DIFF_RING_X_ADDR;
	uint32_t PEN_1D_DIFF_RING_Y_ADDR;
	nvt_ts_reg_t ENB_CASC_REG;
	/* FW History */
	uint32_t MMAP_HISTORY_EVENT0;
	uint32_t MMAP_HISTORY_EVENT1;
	uint32_t MMAP_HISTORY_EVENT2;
	uint32_t MMAP_HISTORY_EVENT3;
	/* START for flash FW update */
	uint32_t READ_FLASH_CHECKSUM_ADDR;
	uint32_t RW_FLASH_DATA_ADDR;
	/* END for FW Update Use */
	/* Phase 2 Host Download */
	uint32_t BOOT_RDY_ADDR;
	uint32_t ACI_ERR_CLR_ADDR;
	uint32_t POR_CD_ADDR;
	uint32_t TX_AUTO_COPY_EN;
	uint32_t SPI_DMA_TX_INFO;
	/* BLD CRC */
	uint32_t BLD_LENGTH_ADDR;
	uint32_t ILM_LENGTH_ADDR;
	uint32_t DLM_LENGTH_ADDR;
	uint32_t BLD_DES_ADDR;
	uint32_t ILM_DES_ADDR;
	uint32_t DLM_DES_ADDR;
	uint32_t G_ILM_CHECKSUM_ADDR;
	uint32_t G_DLM_CHECKSUM_ADDR;
	uint32_t R_ILM_CHECKSUM_ADDR;
	uint32_t R_DLM_CHECKSUM_ADDR;
	uint32_t DMA_CRC_EN_ADDR;
	uint32_t BLD_ILM_DLM_CRC_ADDR;
	uint32_t DMA_CRC_FLAG_ADDR;
	uint32_t DOZE_GM_S1D_SCAN_RAW_ADDR;
	uint32_t DOZE_GM_BTN_SCAN_RAW_ADDR;
};

struct nvt_ts_bin_map {
	char name[12];
	uint32_t BIN_addr;
	uint32_t SRAM_addr;
	uint32_t size;
	uint32_t crc;
};

struct nvt_ts_trim_id_table {
	uint8_t id[NVT_ID_BYTE_MAX];
	uint8_t mask[NVT_ID_BYTE_MAX];
	const struct nvt_ts_mem_map *mmap;
	const struct nvt_ts_mem_map *mmap_casc;
	const struct nvt_ts_hw_info *hwinfo;
};

struct nvt_ts_hw_reg_addr_info {
	uint32_t chip_ver_trim_addr;
	uint32_t swrst_sif_addr;
	uint32_t bld_spe_pups_addr;
};

struct nvt_ts_hw_info {
	uint8_t hw_crc;
	uint8_t auto_copy;
	const struct nvt_ts_hw_reg_addr_info *hw_regs;
};

struct nvt_ts_firmware {
	size_t size;
	const u8 *data;
};

struct nvt_fw_debug_info {
	uint8_t rek_info;
	uint8_t rst_info;
	uint8_t hopping;
	uint8_t esd;
	uint8_t palm;
	uint8_t bending;
	uint8_t water;
	uint8_t gnd;
	uint8_t er;
	uint8_t fog;
	uint8_t film;
	uint8_t notch;
};

struct chip_data_nt36528 {
	bool                            is_sleep_writed;
	char                            *fw_name;
	char                            *test_limit_name;
	struct firmware_headfile        *p_firmware_headfile;
	tp_dev                          tp_type;
	uint8_t                         point_data[POINT_DATA_LEN + 2];
	uint8_t                         fw_ver;
	uint8_t                         fw_sub_ver;
	uint8_t                         fw_eventbuf_prot;
	uint8_t                         recovery_cnt;
	uint8_t                         ilm_dlm_num;
	uint8_t                         cascade_2nd_header_info;
	uint8_t                         enb_casc;
	uint8_t                         *fwbuf;
	uint8_t                         hw_crc;
	uint8_t                         auto_copy;
	uint16_t                        nvt_pid;
	uint32_t                        ENG_RST_ADDR;
	uint32_t                        partition;
	uint32_t                        chip_ver_trim_addr;
	uint32_t                        swrst_sif_addr;
	uint32_t                        bld_spe_pups_addr;
	struct spi_device               *s_client;
	struct hw_resource              *hw_res;
	struct nvt_ts_trim_id_table     trim_id_table;
	struct nvt_ts_bin_map           *bin_map;
	bool                            esd_check_enabled;
	unsigned long                   irq_timer;
	uint8_t                         esd_retry;
	struct device                   *dev;
	/*const struct firmware           *g_fw;*/
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifdef CONFIG_SPI_MT65XX
	struct mtk_chip_config          spi_ctrl;
#else
	struct mt_chip_conf             *spi_ctrl;
#endif
#endif /* end of CONFIG_TOUCHPANEL_MTK_PLATFORM */
	uint8_t                         touch_direction;    /*show touchpanel current direction*/
	struct mutex                    mutex_testing;
	int                             probe_done;
	bool                            using_headfile;
	int                             lcd_reset_gpio;
	struct nvt_fw_debug_info        nvt_fw_debug_info;
	int irq_num;
	struct touchpanel_data *ts;
	u8 *g_fw_buf;
	size_t g_fw_len;
	size_t fw_need_write_size;
	bool g_fw_sta;
	u8 *fw_buf_dma;
	bool need_judge_irq_throw;
	bool aod_flag;

	int tp_index;
	/*add for doze*/
	u32 doze_mode_set_count;
	struct mutex doze_mode_lock;
	struct mutex cmd_mutex;
	struct mutex config_lock;
	int gesture_state;
#ifdef CONFIG_OPLUS_TP_APK

	bool lock_point_status;
	bool plug_status;
	bool debug_mode_sta;
	bool debug_gesture_sta;
	bool earphone_sta;
	bool charger_sta;
	bool noise_sta;
	int water_sta;
#endif /*end of CONFIG_OPLUS_TP_APK*/

	struct nvt_autotest_para *p_nvt_test_para;
	struct nvt_autotest_offset *p_nvt_autotest_offset;
	struct monitor_data *monitor_data;
};

#endif /*NVT_H_NT36528_NOFLASH*/
