TARGET_DATARMNET_EXT_ENABLE := false

ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	ifeq ($(TARGET_KERNEL_DLKM_DATARMNETEXT_OVERRIDE), true)
		TARGET_DATARMNET_EXT_ENABLE := true
	endif
else
	TARGET_DATARMNET_EXT_ENABLE := true
endif

ifeq ($(TARGET_DATARMNET_EXT_ENABLE), true)
	#Build rmnet modules
	DATA_OFFLOAD_DLKM_BOARD_PLATFORMS_LIST := pineapple
	DATA_OFFLOAD_DLKM_BOARD_PLATFORMS_LIST += sun
	DATA_OFFLOAD_DLKM_BOARD_PLATFORMS_LIST += parrot
	DATA_SHS_DLKM_BOARD_PLATFORMS_LIST := pineapple
	DATA_SHS_DLKM_BOARD_PLATFORMS_LIST += sun
	DATA_SHS_DLKM_BOARD_PLATFORMS_LIST += parrot
	DATA_APS_DLKM_BOARD_PLATFORMS_LIST := pineapple
	DATA_APS_DLKM_BOARD_PLATFORMS_LIST += sun
	DATA_WLAN_DLKM_BOARD_PLATFORMS_LIST := pineapple
	DATA_WLAN_DLKM_BOARD_PLATFORMS_LIST += sun
	DATA_WLAN_DLKM_BOARD_PLATFORMS_LIST += parrot
	DATA_WLAN_DLKM_BOARD_PLATFORMS_LIST += monaco
	DATA_MEM_DLKM_BOARD_PLATFORMS_LIST := pineapple
	DATA_MEM_DLKM_BOARD_PLATFORMS_LIST += sun
	DATA_MEM_DLKM_BOARD_PLATFORMS_LIST += parrot
	DATA_MEM_DLKM_BOARD_PLATFORMS_LIST += monaco

	ifneq ($(TARGET_BOARD_AUTO),true)
		ifeq ($(call is-board-platform-in-list,$(DATA_OFFLOAD_DLKM_BOARD_PLATFORMS_LIST)),true)
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/rmnet_offload.ko
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/rmnet_perf_tether.ko
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/rmnet_perf.ko
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/rmnet_wlan.ko
		endif
		ifeq ($(call is-board-platform-in-list,$(DATA_MEM_DLKM_BOARD_PLATFORMS_LIST)),true)
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/rmnet_mem.ko
		endif
		ifeq ($(call is-board-platform-in-list,$(DATA_SHS_DLKM_BOARD_PLATFORMS_LIST)),true)
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/rmnet_shs.ko
		endif
		ifeq ($(call is-board-platform-in-list,$(DATA_APS_DLKM_BOARD_PLATFORMS_LIST)),true)
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/rmnet_aps.ko
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/rmnet_sch.ko
		endif
		ifeq ($(call is-board-platform-in-list,$(DATA_WLAN_DLKM_BOARD_PLATFORMS_LIST)),true)
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/rmnet_wlan.ko
		endif
	endif
endif
