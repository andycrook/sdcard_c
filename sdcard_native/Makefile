TOP = $(realpath $(dir $(lastword $(MAKEFILE_LIST)))/../../../..)
MPY_DIR ?= $(TOP)/tools/micropython
MICROPYTHON ?= $(MPY_DIR)
ARCHES := armv6m armv7emsp
SRC = sdcard_native.c
MOD = $(ARCH)
BUILD := $(TOP)/build/sdcard_native

ifdef ARCH

include $(MPY_DIR)/py/dynruntime.mk

CFLAGS_EXTRA += -O2 -funroll-loops -finline-small-functions

else

.PHONY: all $(ARCHES)

all: $(ARCHES)

$(ARCHES):
	$(MAKE) ARCH=$@ BUILD=$(BUILD)/$@ MPY_DIR=$(MPY_DIR) CFLAGS_EXTRA="$(CFLAGS_EXTRA)"

endif