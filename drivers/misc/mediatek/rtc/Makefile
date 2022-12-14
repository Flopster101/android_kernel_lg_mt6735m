include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_RTC) += mtk_rtc_common.o

# LGE_CHANGE_S : 2015-09-30 lg-drm@lge.com, LG Secure Clock
obj-$(CONFIG_RTC_INTF_SECCLK)   += secclk.o
# LGE_CHANGE_E : 2015-09-30 lg-drm@lge.com, LG Secure Clock

obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/

