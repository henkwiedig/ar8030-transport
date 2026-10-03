# Host-only replay of flight traces through tx/bitrate_ctl.c, see replay.c.
# AR8030_SDK_INC: the ar8030 SDK headers (any staging dir of the ar8030 package).
AR8030_SDK_INC ?= ../../../builder/openipc/output/staging/usr/include/ar8030
CFLAGS ?= -O2 -Wall -Wextra
replay: replay.c ../../tx/bitrate_ctl.c ../../tx/bitrate_ctl.h
	$(CC) $(CFLAGS) -I../../tx -I../../common -I../../third_party/waybeam_frame_ring -I$(AR8030_SDK_INC) \
		-D_GNU_SOURCE -o $@ replay.c ../../tx/bitrate_ctl.c -lm
clean:
	rm -f replay
.PHONY: clean
