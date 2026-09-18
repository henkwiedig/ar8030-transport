# ar8030-transport-audio-rx -- standalone Makefile.
#
# Same cross-compile invocation shape as ../rx/Makefile, e.g.:
#   make CC=aarch64-linux-gnu-gcc \
#        AR8030_SDK_INC=/path/to/yz_host_drv/release/inc \
#        AR8030_SDK_LIB=/path/to/yz_host_drv/release/lib
#
# The top-level Makefile (../Makefile) auto-detects these from the same
# sibling Buildroot tree rx/'s does -- this builds for the ground side too.

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

# See ../rx/Makefile's own comment on why objects build into a flat
# build/, not next to the shared common/*.c sources.
BUILD_DIR := build

SRCS := \
	main.c \
	$(COMMON)/ar8030_link.c \
	$(COMMON)/reassembly.c \
	$(COMMON)/chunk_stream.c \
	$(COMMON)/crc16.c
OBJS := $(addprefix $(BUILD_DIR)/,$(notdir $(SRCS:.c=.o)))

TARGET := ar8030-transport-audio-rx

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
