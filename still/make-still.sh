#!/bin/sh
#
# Renders a text card into one H.265 keyframe for ar8030-still, on the
# build host (ffmpeg with libx265). The result is committed, so builds
# don't need ffmpeg.
#
#   ./make-still.sh <out.h265> "<title>" "<subtitle>" [WxH]
#   ./make-still.sh overheat.h265 "VTX OVERHEAT" \
#       "video paused to cool down - resumes automatically"
#
# Main profile on purpose: x265 marks an all-intra stream (keyint=1) as
# RExt, which hardware decoders may refuse -- one frame with the default
# GOP is plain Main. repeat-headers puts VPS/SPS/PPS in front of the
# picture, info=0 drops x265's version SEI. Same size as the live stream
# (1920x1080), so the goggles' decoder only re-reads the headers.

set -e

out=$1
title=$2
subtitle=$3
size=${4:-1920x1080}
[ -n "$out" ] && [ -n "$title" ] || {
	echo "Usage: $0 <out.h265> \"<title>\" \"<subtitle>\" [WxH]" >&2
	exit 1
}

# drawtext takes ' and : as syntax.
esc() { printf '%s' "$1" | sed "s/[':\\\\]/\\\\&/g"; }

ffmpeg -hide_banner -loglevel error -y \
	-f lavfi -i "color=c=0x202020:s=$size:d=1,\
drawtext=text='$(esc "$title")':fontsize=140:fontcolor=0xff5030:x=(w-tw)/2:y=(h-th)/2-120,\
drawtext=text='$(esc "$subtitle")':fontsize=56:fontcolor=white:x=(w-tw)/2:y=(h-th)/2+80" \
	-frames:v 1 -pix_fmt yuv420p -c:v libx265 \
	-x265-params "profile=main:log-level=error:repeat-headers=1:info=0" \
	-f hevc "$out"

echo "$out: $(wc -c <"$out") bytes"
