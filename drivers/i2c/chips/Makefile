#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

#/*< DTS2011041700393 lijianzhao 20110417 begin */
#modify for 4125 baseline
#obj-$(CONFIG_SENSORS_MT9T013)	+= mt9t013.o
#/* DTS2011041700393 lijianzhao 20110417 end >*/

#/* < DTS2010061100003 luojianhong 201000612 begin*/
obj-$(CONFIG_HUAWEI_FEATURE_TPA2028D1_AMPLIFIER)	+= tpa2028d1.o
#/*  DTS2010061100003 luojianhong 201000612 end > */
#/* <BU5D07680 liujinggang 201004013 begin */
obj-$(CONFIG_HUAWEI_FEATURE_SENSORS_ST_LSM303DLH) += st303_compass.o
obj-$(CONFIG_HUAWEI_FEATURE_SENSORS_ST_LSM303DLH) += st303_gs.o
#/* BU5D07680 liujinggang 20100413 end> */
#/* < DTS2011042703449  liujinggang 20110427 begin */
obj-$(CONFIG_HUAWEI_FEATURE_SENSORS_AK8975)   += akm8975.o
#/* DTS2011042703449  liujinggang 20110427 end > */
ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

