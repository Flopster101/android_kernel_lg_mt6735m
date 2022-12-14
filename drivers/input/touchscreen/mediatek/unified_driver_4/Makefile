include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/include/mach/

obj-y	+=  lgtp_common_driver.o
obj-y	+=  lgtp_model_config_i2c.o
obj-y	+=  lgtp_model_config_misc.o
obj-y	+=  lgtp_platform_api_i2c.o
obj-y	+=  lgtp_platform_api_misc.o
obj-y	+=  lgtp_platform_api_power.o



ifeq ($(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER), y)
ifeq ($(CONFIG_TOUCHSCREEN_SN280H), y)
obj-y	+=	SN280H/
endif
ifeq ($(CONFIG_TOUCHSCREEN_FT6X36), y)
obj-y   +=	FT6X36/
endif
ifeq ($(CONFIG_TOUCHSCREEN_MIT300), y)
obj-y	+=	MIT300/
endif
endif
