GCOV_PROFILE := y
LINUXINCLUDE += -I$(srctree)/

obj-$(CONFIG_OPLUS_FEATURE_CPU_JANKINFO) += oplus_bsp_schedinfo.o
oplus_bsp_schedinfo-y += oplus_sched_info.o
oplus_bsp_schedinfo-y += osi_base.o
oplus_bsp_schedinfo-y += osi_tasktrack.o
oplus_bsp_schedinfo-y += osi_topology.o
oplus_bsp_schedinfo-y += osi_hotthread.o
oplus_bsp_schedinfo-y += osi_freq.o
oplus_bsp_schedinfo-y += osi_cpuload.o
oplus_bsp_schedinfo-y += osi_enable.o
oplus_bsp_schedinfo-y += osi_debug.o
oplus_bsp_schedinfo-y += osi_version.o
oplus_bsp_schedinfo-y += osi_cpuloadmonitor.o
oplus_bsp_schedinfo-y += osi_netlink.o
oplus_bsp_schedinfo-y += osi_loadinfo.o
oplus_bsp_schedinfo-y += osi_healthinfo.o
oplus_bsp_schedinfo-y += osi_memory_monitor.o
oplus_bsp_schedinfo-$(CONFIG_ARM64_AMU_EXTN) += osi_amu.o
