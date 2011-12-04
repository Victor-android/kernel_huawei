#
# Makefile for MMC/SD card drivers
#

ifeq ($(CONFIG_MMC_DEBUG),y)
	EXTRA_CFLAGS		+= -DDEBUG
endif

obj-$(CONFIG_MMC_BLOCK)		+= mmc_block.o
mmc_block-objs			:= block.o queue.o
obj-$(CONFIG_MMC_TEST)		+= mmc_test.o

obj-$(CONFIG_SDIO_UART)		+= sdio_uart.o
# /* <DTS2010080901139 hufeng 20100821 begin */
obj-$(CONFIG_APANIC) += panic_tranfer.o
# /* DTS2010080901139 hufeng 20100821 end> */
