#
# Makefile for the i2c core.
#

obj-$(CONFIG_I2C_BOARDINFO)	+= i2c-boardinfo.o
obj-$(CONFIG_I2C)		+= i2c-core.o
obj-$(CONFIG_I2C_SMBUS)		+= i2c-smbus.o
obj-$(CONFIG_I2C_CHARDEV)	+= i2c-dev.o
#/*< DTS2011041700393 lijianzhao 20110417 begin */
#modify for 4125 baseline
obj-y				+= busses/ chips/ algos/
#/* DTS2011041700393 lijianzhao 20110417 end >*/
#/* < DTS2011042801270 yuezenglong 20110428 begin */
obj-$(CONFIG_HUAWEI_FEATURE_GYROSCOPE_L3G4200DH) += gyroscope/
#/* DTS2011042801270 yuezenglong 20110428 end > */
ifeq ($(CONFIG_I2C_DEBUG_CORE),y)
EXTRA_CFLAGS += -DDEBUG
endif
