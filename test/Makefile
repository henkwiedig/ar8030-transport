# Host-only smoke test -- no AR8030 SDK, no cross toolchain. Just:
#   make -C test check

CC ?= gcc
CFLAGS ?= -O0 -g -Wall -Wextra

ROOT := ..
COMMON := $(ROOT)/common
RX := $(ROOT)/rx

INCLUDES := -I. -I$(COMMON) -I$(RX)
ALL_CFLAGS := $(CFLAGS) $(INCLUDES) -D_GNU_SOURCE

# Object files build into build/, NOT next to the shared .c sources in
# ../common and ../rx -- tx/Makefile and rx/Makefile compile some of
# those same source files for a cross target, and objects dropped
# in-tree would get silently (and wrongly) reused between builds with no
# error until a "file in wrong format" link failure. Source filenames
# are unique across this target's whole SRCS list, so a flat build/
# directory (rather than mirroring ../common, ../rx) is enough to avoid
# collisions.
BUILD_DIR := build

SRCS := \
	roundtrip_test.c \
	$(COMMON)/chunker.c \
	$(COMMON)/reassembly.c \
	$(COMMON)/chunk_stream.c \
	$(COMMON)/crc16.c \
	$(RX)/rtp_h265.c
OBJS := $(addprefix $(BUILD_DIR)/,$(notdir $(SRCS:.c=.o)))

TARGET := roundtrip_test

.PHONY: all check clean
all: $(TARGET)

check: $(TARGET)
	./$(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(ALL_CFLAGS) -o $@ $(OBJS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(COMMON)/%.c | $(BUILD_DIR)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(RX)/%.c | $(BUILD_DIR)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

clean:
	rm -rf $(TARGET) $(BUILD_DIR)
