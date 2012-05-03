
PWD := $(shell pwd)

KERNEL_MOD_DIR := polygator

ifeq ($(TARGET_DEVICE), k5)

obj-m := vinetic.o k5.o
vinetic-objs := vinetic-base.o
k5-objs := k5-base.o

export ARCH=arm
export CROSS_COMPILE=arm-none-linux-gnueabi-

KERNEL_SRC_DIR := /home/maksym/Work/elgato/k5/linux-2.6.33-k5
KERNEL_STG_DIR := /home/maksym/Work/elgato/k5/sysroot

else

obj-m := vinetic.o gsm8ch.o
vinetic-objs := vinetic-base.o
gsm8ch-objs := gsm8ch-base.o

KERNEL_VERSION := `uname -r`
KERNEL_SRC_DIR := /lib/modules/$(KERNEL_VERSION)/build
KERNEL_STG_DIR := /

endif

all:
	@make -C $(KERNEL_SRC_DIR) M=$(PWD) modules
	make -C libvinetic

install:
	@make -C $(KERNEL_SRC_DIR) M=$(PWD) INSTALL_MOD_PATH=$(KERNEL_STG_DIR) INSTALL_MOD_DIR=$(KERNEL_MOD_DIR) modules_install

clean:
	@make -C $(KERNEL_SRC_DIR) M=$(PWD) clean
	make -C libvinetic clean
	@rm -f *~ polygator/*~
