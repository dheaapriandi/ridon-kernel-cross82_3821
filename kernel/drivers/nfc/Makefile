#
# Makefile for nfc devices
#

ifeq ($(MTK_NFC_SUPPORT),yes)
obj-y  +=bcm2079x-i2c_mtk.o
else
#obj-$(CONFIG_PN544_NFC)		+= bcm2079x-i2c_mtk.o
endif

ifeq ($(BRCM_NFC_SUPPORT),yes)
obj-y  +=bcm2079x-i2c_mtk.o
else
obj-n	+= bcm2079x-i2c_mtk.o
endif
#obj-$(CONFIG_NFC_PN533)		+= pn533.o
#obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o

# obj-$(CONFIG_NFC_MSR3110)  += msr3110.o
#ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
