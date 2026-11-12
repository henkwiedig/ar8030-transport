# ar8030-ringdump -- standalone Makefile, same conventions as ../xdata.
# A bench reader for rx's frame-shm ring (rx -R); needs no AR8030 SDK.

CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -g

ROOT := ..
COMMON := $(ROOT)/common
FRAME_RING := $(ROOT)/third_party/waybeam_frame_ring

INCLUDES := -I. -I$(COMMON) -I$(FRAME_RING)
ALL_CFLAGS := $(CFLAGS) $(INCLUDES) -D_GNU_SOURCE
LDLIBS := -lrt

# Objects in build/, not next to the shared sources (see ../rx/Makefile).
BUILD_DIR ?= build
TARGET ?= ar8030-ringdump

SRCS := main.c \
	$(COMMON)/timing_sei.c \
	$(FRAME_RING)/venc_frame_ring.c
OBJS := $(addprefix $(BUILD_DIR)/,$(notdir $(SRCS:.c=.o)))

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

$(BUILD_DIR)/%.o: $(FRAME_RING)/%.c | $(BUILD_DIR)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

clean:
	rm -rf $(TARGET) $(BUILD_DIR)
