obj-$(CONFIG_DIAG_CHAR) := diagchar.o
obj-$(CONFIG_DIAG_SDIO_PIPE) += diagfwd_sdio.o
diagchar-objs := diagchar_core.o diagchar_hdlc.o diagfwd.o diagmem.o
#/* < DTS2010091001041 liuhongfei 20100910 begin */ 
ifeq ($(CONFIG_HUAWEI_FEATURE_PHUDIAG),y)
diagchar-objs += phudiagchar_core.o phudiagfwd.o
endif
#/* < DTS2010091001041 liuhongfei 20100910 end */
