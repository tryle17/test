#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.
#obj-$(CONFIG_TOUCHPANEL_NOVA_NT36672) += NT36672/
#obj-$(CONFIG_TOUCHPANEL_NOVA_NT36525) += NT36525/
#obj-$(CONFIG_TOUCHPANEL_NOVA_NT36672_NOFLASH) += NT36672_noflash/
#obj-$(CONFIG_TOUCHPANEL_NOVA_NT36525_NOFLASH) += NT36525_noflash/

obj-$(CONFIG_TOUCHPANEL_NOVA_NT36672C_NOFLASH) += NT36672C_noflash/
obj-$(CONFIG_TOUCHPANEL_NOVA_NT36523_NOFLASH) += NT36523_noflash/
obj-$(CONFIG_TOUCHPANEL_NOVA_NT36532_NOFLASH) += NT36532_noflash/
obj-$(CONFIG_TOUCHPANEL_NOVA_NT36528_NOFLASH) += NT36528_noflash/

oplus_bsp_tp_novatek_common-y += novatek_common.o
obj-$(CONFIG_TOUCHPANEL_NOVA) += oplus_bsp_tp_novatek_common.o
#endif
