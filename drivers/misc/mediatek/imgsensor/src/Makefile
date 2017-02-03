include $(srctree)/drivers/misc/mediatek/Makefile.custom

CONFIG_MTK_PLATFORM_SENSOR := $(CONFIG_MTK_PLATFORM)

ifeq ($(CONFIG_ARCH_MT6580),y)
CONFIG_MTK_PLATFORM_SENSOR := mt6580
endif

obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM_SENSOR))/

