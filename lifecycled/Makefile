# ar8030-lifecycled -- standalone Makefile, same conventions as ../tx,
# ../rx and ../linkctl (see those Makefiles' own header comments for the
# cross-compile invocation shape).
#
# Like linkctl (and unlike tx/rx), this tool is meant to be built and
# installed on BOTH sides -- reconnect-following, tuning and hook
# dispatch are useful air- and ground-side -- so BUILD_DIR and TARGET are
# overridable the same way: the top-level ../Makefile builds it twice
# (once per cross toolchain) from this one source tree, and each side's
# Buildroot package (builder/package/ar8030-transport-tx,
# sbc-groundstations/package/ar8030-transport-rx) builds its own copy in
# its own separate checkout, where the default names below are fine as
# they are since there's no risk of the two builds' objects colliding.
#
# Needs libcjson (pairing/tuning persistence, lifecycle_pair.c/
# lifecycle_tuning.c) in addition to the ar8030 SDK's own
# libar8030_client.so -- both Buildroot packages add `cjson` to their own
# DEPENDENCIES for this.

CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -g
AR8030_SDK_INC ?= /usr/include/ar8030
AR8030_SDK_LIB ?= /usr/lib

INCLUDES := -I. -I$(AR8030_SDK_INC)
ALL_CFLAGS := $(CFLAGS) $(INCLUDES) -D_GNU_SOURCE

LDFLAGS += -L$(AR8030_SDK_LIB)
LDLIBS := -lar8030_client -lcjson -lpthread -lm

BUILD_DIR ?= build
TARGET ?= ar8030-lifecycled

SRCS := \
	main.c \
	lifecycle.c \
	lifecycle_bind.c \
	lifecycle_client.c \
	lifecycle_gpio.c \
	lifecycle_hooks.c \
	lifecycle_http.c \
	lifecycle_pair.c \
	lifecycle_tuning.c
OBJS := $(addprefix $(BUILD_DIR)/,$(notdir $(SRCS:.c=.o)))

.PHONY: all clean
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(ALL_CFLAGS) -c -o $@ $<

clean:
	rm -rf $(TARGET) $(BUILD_DIR)
