# artosyn_drv: standalone out-of-tree kernel module (see artosyn_sdio.c's
# own header comment for what this is and why it exists, and
# ../doc/native-netdev.md for the native net_device part, artosyn_net.c).
#
# Two different builds happen through this same Makefile, same as any
# out-of-tree kernel module:
#   1. A plain `make` (or `make KDIR=... CROSS_COMPILE=...`) here invokes
#      the *kernel's own* build system against this directory (the
#      `modules` target below), which re-enters this same Makefile a
#      second time in "kbuild" mode (recognizable by KERNELRELEASE being
#      set) to actually compile artosyn_sdio.o + artosyn_net.o ->
#      artosyn_drv.ko.
#   2. That second invocation only needs obj-m and the include path --
#      everything else in this file is for the *first* invocation, mostly
#      auto-detecting KDIR/CROSS_COMPILE from the sibling `builder` tree
#      this project is developed alongside (see top-level Makefile's own
#      auto-detection comment for the general pattern; this mirrors it).
#
# Override any of these to point elsewhere:
#   make KDIR=/path/to/linux-build CROSS_COMPILE=/path/to/toolchain- \
#        AR8030_SDK_DRIVER_INC=/path/to/yz_host_drv/driver/linux/bus
# Run `make print-config` to see what was auto-detected.

ifeq ($(KERNELRELEASE),)

BUILDER_DIR ?= ../../builder
BUILDER_OUTPUT := $(BUILDER_DIR)/openipc/output

KDIR ?= $(abspath $(firstword $(wildcard $(BUILDER_OUTPUT)/build/linux-custom)))
CROSS_COMPILE ?= $(abspath $(firstword $(wildcard $(BUILDER_OUTPUT)/host/bin/*-linux-musleabi-gcc)))
# $(wildcard ...-gcc) matches the compiler binary itself; strip the
# trailing "gcc" to get the CROSS_COMPILE *prefix* kbuild expects.
CROSS_COMPILE := $(patsubst %gcc,%,$(CROSS_COMPILE))

# yz_host_drv is the vendor AR8030 SDK checkout this project's ar8030
# Buildroot package (builder/package/ar8030/ar8030.mk) builds its own
# combined driver from -- ../../ascent/8030_sdk/yz_host_drv matches this
# repo's actual on-disk layout (sibling of ar8030-transport and builder
# under OpenIPC/), not a general convention.
AR8030_SDK_DRIVER_INC ?= $(abspath ../../ascent/8030_sdk/yz_host_drv/driver/linux/bus)

ARCH ?= arm

.PHONY: all modules clean print-config

all: modules

modules:
	@test -n "$(KDIR)" || { echo "KDIR not set and no build/linux-custom found under $(BUILDER_OUTPUT) -- pass KDIR= explicitly (see 'make print-config')"; exit 1; }
	@test -n "$(CROSS_COMPILE)" || { echo "CROSS_COMPILE not set and no toolchain found under $(BUILDER_OUTPUT)/host/bin -- pass CROSS_COMPILE= explicitly"; exit 1; }
	$(MAKE) -C $(KDIR) M=$(CURDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		AR8030_SDK_DRIVER_INC=$(AR8030_SDK_DRIVER_INC) modules

clean:
	@if [ -n "$(KDIR)" ]; then $(MAKE) -C $(KDIR) M=$(CURDIR) clean; fi
	rm -f Module.symvers modules.order

print-config:
	@echo "BUILDER_OUTPUT         = $(BUILDER_OUTPUT)"
	@echo "KDIR                   = $(if $(KDIR),$(KDIR),(not found -- pass KDIR= explicitly))"
	@echo "CROSS_COMPILE          = $(if $(CROSS_COMPILE),$(CROSS_COMPILE),(not found -- pass CROSS_COMPILE= explicitly))"
	@echo "AR8030_SDK_DRIVER_INC  = $(AR8030_SDK_DRIVER_INC)"
	@echo "ARCH                   = $(ARCH)"

else

# --- kbuild pass (KERNELRELEASE is set) --------------------------------
obj-m := artosyn_drv.o
artosyn_drv-y := artosyn_sdio.o artosyn_net.o
ccflags-y += -I$(AR8030_SDK_DRIVER_INC)

endif
