# ar8030-transport-rx -- standalone Makefile.
#
# Cross-compile by overriding CC (and CFLAGS if you need a -march/-mtune),
# e.g.:
#   make CC=aarch64-linux-gnu-gcc \
#        AR8030_SDK_INC=/path/to/yz_host_drv/release/inc \
#        AR8030_SDK_LIB=/path/to/yz_host_drv/release/lib
#
# The Buildroot package (sbc-groundstations/package/ar8030-transport-rx)
# invokes this same Makefile with AR8030_SDK_INC/AR8030_SDK_LIB pointed at
# the already-built ar8030 package's staging dir -- see that package's .mk.
# The top-level Makefile (../Makefile) does the same, auto-detected from
# the sibling Buildroot trees.

CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -g
AR8030_SDK_INC ?= /usr/include/ar8030
AR8030_SDK_LIB ?= /usr/lib

ROOT := ..
COMMON := $(ROOT)/common

INCLUDES := -I. -I$(COMMON) -I$(AR8030_SDK_INC)
ALL_CFLAGS := $(CFLAGS) $(INCLUDES) -D_GNU_SOURCE

LDFLAGS += -L$(AR8030_SDK_LIB)
LDLIBS := -lar8030_client -lpthread -lrt -lm

# Object files build into build/, NOT next to the shared .c sources in
# ../common -- tx/Makefile compiles some of those same source files for a
# different target architecture, and objects dropped in-tree would get
# silently (and wrongly) reused between the two builds with no error
# until a "file in wrong format" link failure. Source filenames are
# unique across this target's whole SRCS list, so a flat build/ directory
# (rather than mirroring ../common) is enough to avoid collisions.
BUILD_DIR := build

SRCS := \
	main.c \
	rtp_h265.c \
	$(COMMON)/ar8030_link.c \
	$(COMMON)/reassembly.c
OBJS := $(addprefix $(BUILD_DIR)/,$(notdir $(SRCS:.c=.o)))

TARGET := ar8030-transport-rx

.PHONY: all clean
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(COMMON)/%.c | $(BUILD_DIR)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

clean:
	rm -rf $(TARGET) $(BUILD_DIR)
