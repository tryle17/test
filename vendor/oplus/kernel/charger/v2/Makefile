ifeq ($(DEVICE_MODULES_PATH),)
DEVICE_MODULES_PATH = $(srctree)
else
LINUXINCLUDE := $(DEVCIE_MODULES_INCLUDE) $(LINUXINCLUDE)
endif

obj-y += ufcs/

ccflags-y += -I$(srctree)/$(src)/include
ccflags-y += -I$(srctree)/$(src)/../test-kit
ccflags-y += -I$(srctree)/$(src)/../debug-kit
ccflags-$(CONFIG_OPLUS_PD_MANAGER_CHARGER) += -I$(srctree)/drivers/usb/typec/tcpc/inc
ccflags-$(CONFIG_OPLUS_MT6375_CHARGER) += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
ccflags-$(CONFIG_OPLUS_MT6379_CHARGER) += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
ccflags-y += -I$(srctree)/$(src)/ufcs/include
ccflags-$(CONFIG_OPLUS_DYNAMIC_CONFIG_CHARGER) += -I$(srctree)/$(src)/../config
ccflags-y += -Werror=parentheses
ccflags-y += -Werror=implicit-fallthrough
ccflags-y += -Werror=format
ccflags-y += -DOPLUS_CHG_KO_BUILD

ifeq ($(CONFIG_OPLUS_CHARGER_MTK),y)
subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/typec/tcpc/inc
subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include

ccflags-$(CONFIG_OPLUS_MT6375_CHARGER) += -I$(DEVICE_MODULES_PATH)/drivers/power/supply
ccflags-$(CONFIG_OPLUS_MT6375_CHARGER) += -I$(DEVICE_MODULES_PATH)/drivers/gpu/drm/mediatek/mediatek_v2
ccflags-$(CONFIG_OPLUS_MT6375_CHARGER) += -I$(DEVICE_MODULES_PATH)/drivers/base/kernelFwUpdate

ccflags-$(CONFIG_OPLUS_MT6379_CHARGER) += -I$(DEVICE_MODULES_PATH)/drivers/power/supply
ccflags-$(CONFIG_OPLUS_MT6379_CHARGER) += -I$(DEVICE_MODULES_PATH)/drivers/gpu/drm/mediatek/mediatek_v2
ccflags-$(CONFIG_OPLUS_MT6379_CHARGER) += -I$(DEVICE_MODULES_PATH)/drivers/base/kernelFwUpdate
endif

oplus_chg_v2-y += oplus_chg_core.o
oplus_chg_v2-y += oplus_battery_log.o
oplus_chg_v2-y += oplus_chg_gki.o
oplus_chg_v2-y += oplus_chg_voter.o
oplus_chg_v2-y += oplus_chg_wired.o
oplus_chg_v2-y += oplus_chg_comm.o
oplus_chg_v2-y += oplus_chg_vooc.o
oplus_chg_v2-y += oplus_configfs.o
oplus_chg_v2-y += oplus_chg_dual_chan.o
oplus_chg_v2-y += oplus_chg_cpa.o
oplus_chg_v2-y += oplus_impedance_check.o
oplus_chg_v2-y += oplus_chg_ufcs.o
oplus_chg_v2-y += oplus_chg_wls.o
oplus_chg_v2-y += oplus_smart_chg.o
oplus_chg_v2-y += oplus_chg_pps.o
oplus_chg_v2-y += oplus_batt_bal.o
oplus_chg_v2-y += oplus_chg_mutual.o
oplus_chg_v2-$(CONFIG_OPLUS_CHG_PARALLEL) += oplus_parallel.o

oplus_chg_v2-y += gauge_ic/oplus_hal_bq27541.o
oplus_chg_v2-$(CONFIG_OPLUS_GAUGE_BQ27Z561) += gauge_ic/oplus_hal_bq27z561.o
oplus_chg_v2-$(CONFIG_OPLUS_GAUGE_NFG8011B) += gauge_ic/oplus_hal_nfg8011b.o

oplus_chg_v2-y += vooc_ic/oplus_hal_rk826.o
oplus_chg_v2-y += vooc_ic/oplus_hal_op10.o

oplus_chg_v2-y += hal/oplus_chg_ic.o
oplus_chg_v2-$(CONFIG_OPLUS_CHG_IC_DEBUG) += hal/debug/oplus_chg_ic_debug.o
oplus_chg_v2-y += hal/oplus_virtual_buck.o
oplus_chg_v2-y += hal/oplus_virtual_asic.o
oplus_chg_v2-y += hal/oplus_virtual_gauge.o
oplus_chg_v2-y += hal/oplus_virtual_voocphy.o
oplus_chg_v2-y += hal/oplus_virtual_cp.o
oplus_chg_v2-y += hal/oplus_virtual_rx.o
oplus_chg_v2-y += hal/oplus_hal_wls.o
oplus_chg_v2-y += hal/oplus_hal_vooc.o
oplus_chg_v2-y += hal/oplus_virtual_dpdm_switch.o
oplus_chg_v2-y += hal/oplus_virtual_pps.o
oplus_chg_v2-$(CONFIG_OPLUS_CHG_PARALLEL) += hal/oplus_virtual_switching.o
oplus_chg_v2-y += hal/oplus_virtual_ufcs.o
oplus_chg_v2-y += hal/oplus_virtual_platufcs.o
oplus_chg_v2-y += hal/oplus_virtual_batt_bal.o
oplus_chg_v2-y += hal/oplus_virtual_level_shift.o

oplus_chg_v2-y += mms/oplus_mms.o
oplus_chg_v2-y += mms/oplus_mms_gauge.o
oplus_chg_v2-y += mms/oplus_mms_wired.o
oplus_chg_v2-y += mms/gauge/oplus_sili.o

oplus_chg_v2-y += strategy/oplus_strategy.o
oplus_chg_v2-y += strategy/oplus_strategy_cgcl.o
oplus_chg_v2-y += strategy/oplus_strategy_inr_switch.o
oplus_chg_v2-y += strategy/oplus_strategy_pps_ufcs_curve.o
oplus_chg_v2-y += strategy/oplus_strategy_low_curr_full.o
oplus_chg_v2-y += strategy/oplus_strategy_pps_ufcs_curve_v2.o
oplus_chg_v2-y += strategy/oplus_strategy_ddrc.o
oplus_chg_v2-y += monitor/oplus_monitor_core.o
oplus_chg_v2-y += monitor/oplus_chg_track.o
oplus_chg_v2-y += monitor/oplus_chg_exception.o
oplus_chg_v2-y += plat_ufcs/plat_ufcs_notify.o

oplus_chg_v2-$(CONFIG_OPLUS_DA9313_CHARGER) += charger_ic/oplus_da9313.o
oplus_chg_v2-$(CONFIG_OPLUS_ADSP_CHARGER) += charger_ic/oplus_hal_adsp.o
oplus_chg_v2-$(CONFIG_OPLUS_MP2762_CHARGER) += charger_ic/oplus_hal_mp2650.o
oplus_chg_v2-$(CONFIG_OPLUS_SGM41512_CHARGER) += charger_ic/oplus_hal_sgm41512.o
oplus_chg_v2-$(CONFIG_OPLUS_PD_MANAGER_CHARGER) += charger_ic/oplus_hal_pd_manager.o
oplus_chg_v2-$(CONFIG_OPLUS_MT6375_CHARGER) += charger_ic/oplus_hal_mtk6895S.o
oplus_chg_v2-$(CONFIG_OPLUS_MT6375_CHARGER) += charger_ic/oplus_hal_mt6375.o
oplus_chg_v2-$(CONFIG_OPLUS_MT6379_CHARGER) += charger_ic/oplus_hal_mtk6991V.o
oplus_chg_v2-$(CONFIG_OPLUS_ADSP_SM8450_CHARGER) += charger_ic/oplus_hal_sm8450.o
oplus_chg_v2-$(CONFIG_OPLUS_TPS6128XD_CHARGER) += charger_ic/oplus_hal_tps6128xd.o
oplus_chg_v2-$(CONFIG_OPLUS_SY6603_BATT_BAL) += batt_bal_ic/oplus_hal_sy6603.o
oplus_chg_v2-$(CONFIG_OPLUS_SC7637_LEVEL_SHIFT) += level_shift_ic/oplus_hal_sc7637.o
oplus_chg_v2-$(CONFIG_OPLUS_MT6375_CHARGER) += gauge_ic/oplus_hal_mtk_platform_gauge.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_OPTIGA) += gauge_ic/oplus_optiga/ECC/Optiga_Ecc.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_OPTIGA) += gauge_ic/oplus_optiga/ECC/Optiga_Math.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_OPTIGA) += gauge_ic/oplus_optiga/Platform/board.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_OPTIGA) += gauge_ic/oplus_optiga/SWI/Optiga_Auth.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_OPTIGA) += gauge_ic/oplus_optiga/SWI/Optiga_Nvm.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_OPTIGA) += gauge_ic/oplus_optiga/SWI/Optiga_Swi.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_OPTIGA) += gauge_ic/oplus_optiga/oplus_optiga.o

oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_MAXIM) += gauge_ic/oplus_maxim/oplus_ds28e30/1wire_protocol.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_MAXIM) += gauge_ic/oplus_maxim/oplus_ds28e30/bignum.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_MAXIM) += gauge_ic/oplus_maxim/oplus_ds28e30/deep_cover_coproc_sw.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_MAXIM) += gauge_ic/oplus_maxim/oplus_ds28e30/ds28e30.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_MAXIM) += gauge_ic/oplus_maxim/oplus_ds28e30/ecc_generate_key.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_MAXIM) += gauge_ic/oplus_maxim/oplus_ds28e30/ecdsa_generic_api.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_MAXIM) += gauge_ic/oplus_maxim/oplus_ds28e30/ecdsa_high.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_MAXIM) += gauge_ic/oplus_maxim/oplus_ds28e30/sha256_stone.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_MAXIM) += gauge_ic/oplus_maxim/oplus_ds28e30/ucl_rng.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_MAXIM) += gauge_ic/oplus_maxim/oplus_ds28e30/ucl_sha256.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGER_MAXIM) += gauge_ic/oplus_maxim/oplus_maxim.o

oplus_chg_v2-$(CONFIG_OPLUS_CHG_AP_VOOCPHY) += voocphy/oplus_ap_voocphy.o
oplus_chg_v2-$(CONFIG_OPLUS_CHG_ADSP_VOOCPHY) += voocphy/oplus_adsp_voocphy.o
oplus_chg_v2-$(CONFIG_OPLUS_CHG_VOOCPHY_CHGLIB) += voocphy/chglib/oplus_chglib.o
oplus_chg_v2-$(CONFIG_OPLUS_VOOCPHY_MASTER_SC8547) += voocphy/phy/oplus_sc8547.o
oplus_chg_v2-$(CONFIG_OPLUS_VOOCPHY_SLAVE_SC8547) += voocphy/phy/oplus_sc8547_slave.o
oplus_chg_v2-$(CONFIG_OPLUS_VOOCPHY_SC8517) += voocphy/phy/oplus_sc8517.o
oplus_chg_v2-$(CONFIG_OPLUS_VOOCPHY_MAX77939) += voocphy/phy/oplus_max77939.o
oplus_chg_v2-$(CONFIG_OPLUS_VOOCPHY_HL7138) += voocphy/phy/oplus_hl7138.o
oplus_chg_v2-$(CONFIG_OPLUS_VOOCPHY_SLAVE_HL7138) += voocphy/phy/oplus_hl7138_slave.o

oplus_chg_v2-$(CONFIG_OPLUS_VOOCPHY_SC8547A) += voocphy/phy/oplus_sc8547a.o
oplus_chg_v2-$(CONFIG_OPLUS_UFCS_MASTER_NU2112A) += ufcs_ic/oplus_hal_nu2112a.o
oplus_chg_v2-$(CONFIG_OPLUS_UFCS_SLAVE_NU2112A) += ufcs_ic/oplus_hal_nu2112a_slave.o

oplus_chg_v2-$(CONFIG_OPLUS_CHG_MOS_CTRL) += switching_ic/oplus_mos_ctrl.o

oplus_chg_v2-$(CONFIG_OPLUS_UFCS_SC2201) += ufcs_ic/oplus_hal_sc2201.o
oplus_chg_v2-$(CONFIG_OPLUS_UFCS_SC8547A) += ufcs_ic/oplus_hal_sc8547a.o
oplus_chg_v2-$(CONFIG_OPLUS_PHY_SC8547D) += ufcs_ic/oplus_hal_sc8547d.o

oplus_chg_v2-$(CONFIG_OPLUS_WIRELESS_NU1619) += wireless_ic/oplus_hal_nu1619.o
oplus_chg_v2-$(CONFIG_OPLUS_WIRELESS_NU1669) += wireless_ic/oplus_hal_nu1669.o
oplus_chg_v2-$(CONFIG_OPLUS_WIRELESS_P9415) += wireless_ic/oplus_hal_p9415.o
oplus_chg_v2-$(CONFIG_OPLUS_WIRELESS_SC96257) += wireless_ic/oplus_hal_sc96257.o
oplus_chg_v2-$(CONFIG_OPLUS_CHARGEPUMP_HL7227) += chargepump_ic/oplus_hal_hl7227.o

KBUILD_LDS_MODULE_ATTACH = oplus_chg_module.lds

obj-$(CONFIG_OPLUS_CHG_V2) += oplus_chg_v2.o

include $(srctree)/$(src)/Makefile.json-build
