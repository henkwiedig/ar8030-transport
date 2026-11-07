# ar8030-xdata -- standalone Makefile, same conventions as ../linkctl.
# Ground side only: it writes to the ground chip's host data channel,
# which the chips bridge to the air unit's chip UART2.

CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -g
AR8030_SDK_INC ?= /usr/include/ar8030
AR8030_SDK_LIB ?= /usr/lib

INCLUDES := -I. -I$(AR8030_SDK_INC)
ALL_CFLAGS := $(CFLAGS) $(INCLUDES) -D_GNU_SOURCE

LDFLAGS += -L$(AR8030_SDK_LIB)
LDLIBS := -lar8030_client -lpthread -lrt -lm

BUILD_DIR ?= build
TARGET ?= ar8030-xdata

SRCS := main.c
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
