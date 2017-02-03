
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif

ifneq ($(CONFIG_ARM64), y)
obj-y   +=  camera/camera/
endif
#obj-y   +=  flashlight/constant_flashlight/
obj-$(CONFIG_MTK_VIBRATOR)   +=  vibrator/
obj-$(CONFIG_MTK_LEDS)   +=  leds/
#obj-y   +=  keypad/
#obj-y   +=  sound/
obj-$(CONFIG_MTK_ACCDET)   +=  accdet/

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  alsps/cm36652-new/
endif
###lc mike_zhu 20150511 for alsps
ifeq ($(CONFIG_MTK_EPL259X),y)
obj-y	+=  alsps/epl259x/
endif

ifeq ($(CONFIG_MTK_MC3410_NEW),y)
obj-y   +=  accelerometer/mc3410-new/
endif
###lc mike_zhu 20150513 for gsensor
ifeq ($(CONFIG_MTK_BMA056_6150),y)
obj-y   +=  accelerometer/bma056_6150/
endif
ifeq ($(CONFIG_MTK_BMA2XX),y)
obj-y   +=  accelerometer/bma2xx/
endif
ifeq ($(CONFIG_MTK_BMA222E_NEW),y)
obj-y   +=  accelerometer/bma222e_new/
endif
ifeq ($(CONFIG_MTK_ITG1010_NEW),y)
obj-y   +=  gyroscope/ITG1010-new/
endif
###lc mike_zhu 20150513 for msensor
ifeq ($(CONFIG_MTK_BMM056_6150),y)
obj-y   +=  magnetometer/bmm056_6150/
endif
ifeq ($(CONFIG_MTK_AKM09911_NEW),y)
obj-y	+=  magnetometer/akm09911-new/
endif
endif
