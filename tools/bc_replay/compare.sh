#!/bin/sh
# compare.sh [pi key=value ...] -- rules vs PI over every trace, plus totals.
cd "$(dirname "$0")"
for t in traces/*.csv; do
    n=$(basename "$t" .csv | cut -c1-4)
    printf "%s " "$n"; ./replay "$t" rules
    printf "%s " "$n"; ./replay "$t" pi "$@"
done
