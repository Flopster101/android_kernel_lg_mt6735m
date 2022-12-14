#
# Makefile for misc devices that really don't fit anywhere else.
include $(srctree)/drivers/misc/mediatek/Makefile.custom
custom_cam_cal := $(addsuffix /, $(shell echo $(CONFIG_CUSTOM_KERNEL_CAM_CAL_DRV)))        	
obj-y += $(custom_cam_cal)
obj-y	+= DW9716_eeprom/
obj-y	+= Common_eeprom/