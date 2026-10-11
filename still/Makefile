# ar8030-still -- standalone Makefile, same conventions as ../tx (see its
# header comment for the cross-compile invocation). Air side only, and
# unlike tx it needs nothing from the AR8030 SDK: it only writes the
# frame-shm ring tx reads.

CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -g

ROOT := ..
FRAME_RING := $(ROOT)/third_party/waybeam_frame_ring

INCLUDES := -I. -I$(FRAME_RING)
ALL_CFLAGS := $(CFLAGS) $(INCLUDES) -D_GNU_SOURCE

LDLIBS := -lrt

# Objects in build/, not next to the shared ring source (see ../tx/Makefile).
BUILD_DIR := build

SRCS := \
	main.c \
	$(FRAME_RING)/venc_frame_ring.c
OBJS := $(addprefix $(BUILD_DIR)/,$(notdir $(SRCS:.c=.o)))

TARGET := ar8030-still

.PHONY: all clean
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(FRAME_RING)/%.c | $(BUILD_DIR)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

clean:
	rm -rf $(TARGET) $(BUILD_DIR)
