# Top-level Makefile: descends into tx/, rx/ and test/, auto-detecting
# each side's cross toolchain and AR8030 SDK staging dir from the sibling
# Buildroot trees this project is built alongside (see README.md
# "Buildroot integration") -- so a plain `make` at the repo root cross-
# compiles both binaries with no manual CC=/AR8030_SDK_INC=/... needed,
# as long as those trees have been built at least once.
#
# Override any of these to point elsewhere, or to force a specific
# toolchain/SDK regardless of auto-detection:
#   make TX_CC=... TX_SDK_INC=... TX_SDK_LIB=...
#   make RX_CC=... RX_SDK_INC=... RX_SDK_LIB=...
# Run `make print-config` to see what was actually detected before
# building.

# Sibling checkouts, matching this project's actual on-disk layout
# (.../OpenIPC/{ar8030-transport,builder,sbc-groundstations}).
BUILDER_DIR ?= ../builder
GROUNDSTATIONS_DIR ?= ../sbc-groundstations

# --- Air side (tx) -----------------------------------------------------
#
# builder.sh clones OpenIPC/firmware fresh into openipc/ and builds it
# there on every run (builder/CLAUDE.md) -- openipc/output/ is a single
# flat Buildroot output dir (not per-device), but it only exists after a
# build.sh run and gets wiped by the next one. Treat auto-detection here
# as best-effort, not something to rely on staying put.
BUILDER_OUTPUT := $(BUILDER_DIR)/openipc/output
TX_CC_AUTO := $(firstword $(wildcard $(BUILDER_OUTPUT)/host/bin/*-gcc))
TX_SDK_INC_AUTO := $(firstword $(wildcard $(BUILDER_OUTPUT)/staging/usr/include/ar8030))
TX_SDK_LIB_AUTO := $(firstword $(wildcard $(BUILDER_OUTPUT)/staging/usr/lib))

# --- Ground side (rx) ----------------------------------------------------
#
# sbc-groundstations is itself a Buildroot tree, output/<defconfig>/ per
# board built (see its build.sh). output/ can also hold stray non-build
# entries (e.g. flashed-image backups dropped there by hand) alongside
# the real per-defconfig dirs, so select only actual directories (the
# "*/." wildcard trick) rather than every output/* entry. If more than
# one defconfig has been built, the first one found wins -- pass
# GS_DEFCONFIG=<name> explicitly to pin a specific one.
GS_OUTPUT_DIRS := $(patsubst %/.,%,$(wildcard $(GROUNDSTATIONS_DIR)/output/*/.))
GS_DEFCONFIG ?= $(notdir $(firstword $(GS_OUTPUT_DIRS)))
GS_OUTPUT := $(GROUNDSTATIONS_DIR)/output/$(GS_DEFCONFIG)
RX_CC_AUTO := $(firstword $(wildcard $(GS_OUTPUT)/host/bin/*-gcc))
RX_SDK_INC_AUTO := $(firstword $(wildcard $(GS_OUTPUT)/staging/usr/include/ar8030))
RX_SDK_LIB_AUTO := $(firstword $(wildcard $(GS_OUTPUT)/staging/usr/lib))

# Explicit overrides win; otherwise fall back to what was auto-detected.
# abspath here is not cosmetic: these get passed to `$(MAKE) -C tx` /
# `-C rx`, which changes the sub-make's working directory before the
# value is ever used, so a path left relative to the repo root (e.g.
# ../builder/...) would resolve one directory too shallow once evaluated
# from inside tx/ or rx/.
TX_CC ?= $(abspath $(TX_CC_AUTO))
TX_SDK_INC ?= $(abspath $(TX_SDK_INC_AUTO))
TX_SDK_LIB ?= $(abspath $(TX_SDK_LIB_AUTO))
RX_CC ?= $(abspath $(RX_CC_AUTO))
RX_SDK_INC ?= $(abspath $(RX_SDK_INC_AUTO))
RX_SDK_LIB ?= $(abspath $(RX_SDK_LIB_AUTO))

# Omit a variable entirely (rather than passing CC=) when nothing was
# found or configured, so tx/Makefile's and rx/Makefile's own `CC ?= gcc`
# / `AR8030_SDK_INC ?= /usr/include/ar8030` defaults still apply -- an
# explicit empty CC= would instead tell make "there is no compiler".
TX_MAKE_VARS := $(if $(TX_CC),CC=$(TX_CC)) $(if $(TX_SDK_INC),AR8030_SDK_INC=$(TX_SDK_INC)) \
	$(if $(TX_SDK_LIB),AR8030_SDK_LIB=$(TX_SDK_LIB))
RX_MAKE_VARS := $(if $(RX_CC),CC=$(RX_CC)) $(if $(RX_SDK_INC),AR8030_SDK_INC=$(RX_SDK_INC)) \
	$(if $(RX_SDK_LIB),AR8030_SDK_LIB=$(RX_SDK_LIB))

.PHONY: all tx rx linkctl linkctl-tx linkctl-rx lifecycled lifecycled-tx lifecycled-rx test check clean print-config

all: tx rx linkctl lifecycled

tx:
	@echo "== tx: CC=$(if $(TX_CC),$(TX_CC),<default: host gcc -- see 'make print-config'>)"
	$(MAKE) -C tx $(TX_MAKE_VARS)

rx:
	@echo "== rx: CC=$(if $(RX_CC),$(RX_CC),<default: host gcc -- see 'make print-config'>)"
	$(MAKE) -C rx $(RX_MAKE_VARS)

# ar8030-linkctl (see linkctl/main.c) is needed on BOTH sides -- unlike
# tx/rx it isn't "the air tool" or "the ground tool", so this builds it
# twice from the one source tree, once per cross toolchain, into two
# separate BUILD_DIR/TARGET pairs (see linkctl/Makefile's own header
# comment for why those are override-able there) so the two arches'
# object files and binaries don't collide the way tx/'s and rx/'s
# shared common/*.c objects would if they landed in the same directory.
linkctl: linkctl-tx linkctl-rx

linkctl-tx:
	@echo "== linkctl-tx: CC=$(if $(TX_CC),$(TX_CC),<default: host gcc -- see 'make print-config'>)"
	$(MAKE) -C linkctl $(TX_MAKE_VARS) BUILD_DIR=build-tx TARGET=ar8030-linkctl-tx

linkctl-rx:
	@echo "== linkctl-rx: CC=$(if $(RX_CC),$(RX_CC),<default: host gcc -- see 'make print-config'>)"
	$(MAKE) -C linkctl $(RX_MAKE_VARS) BUILD_DIR=build-rx TARGET=ar8030-linkctl-rx

# ar8030-lifecycled (see lifecycled/main.c) -- needed on both sides for
# the same reason linkctl is (reconnect-following, tuning, hook dispatch
# are useful air- and ground-side), built the same doubled way and for
# the same reason (two arches' objects/binaries must not collide in one
# source tree). Moved here from builder/package/ar8030's own patch stack
# (and sbc-groundstations/package/ar8030's identical duplicate of it) --
# entirely original code with no vendor lineage, so this is its actual
# home now, same as tx/rx/linkctl.
lifecycled: lifecycled-tx lifecycled-rx

lifecycled-tx:
	@echo "== lifecycled-tx: CC=$(if $(TX_CC),$(TX_CC),<default: host gcc -- see 'make print-config'>)"
	$(MAKE) -C lifecycled $(TX_MAKE_VARS) BUILD_DIR=build-tx TARGET=ar8030-lifecycled-tx

lifecycled-rx:
	@echo "== lifecycled-rx: CC=$(if $(RX_CC),$(RX_CC),<default: host gcc -- see 'make print-config'>)"
	$(MAKE) -C lifecycled $(RX_MAKE_VARS) BUILD_DIR=build-rx TARGET=ar8030-lifecycled-rx

# Host-only protocol round-trip test -- no cross toolchain or AR8030 SDK
# involved, see test/roundtrip_test.c.
test check:
	$(MAKE) -C test check

clean:
	$(MAKE) -C tx clean
	$(MAKE) -C rx clean
	$(MAKE) -C linkctl clean BUILD_DIR=build-tx TARGET=ar8030-linkctl-tx
	$(MAKE) -C linkctl clean BUILD_DIR=build-rx TARGET=ar8030-linkctl-rx
	$(MAKE) -C lifecycled clean BUILD_DIR=build-tx TARGET=ar8030-lifecycled-tx
	$(MAKE) -C lifecycled clean BUILD_DIR=build-rx TARGET=ar8030-lifecycled-rx
	$(MAKE) -C test clean

print-config:
	@echo "tx (air / builder):"
	@echo "  BUILDER_OUTPUT = $(BUILDER_OUTPUT)"
	@echo "  CC             = $(if $(TX_CC),$(TX_CC),(not found -- falls back to tx/Makefile's own default))"
	@echo "  AR8030_SDK_INC = $(if $(TX_SDK_INC),$(TX_SDK_INC),(not found -- falls back to tx/Makefile's own default))"
	@echo "  AR8030_SDK_LIB = $(if $(TX_SDK_LIB),$(TX_SDK_LIB),(not found -- falls back to tx/Makefile's own default))"
	@echo "rx (ground / sbc-groundstations):"
	@echo "  GS_DEFCONFIG   = $(GS_DEFCONFIG)$(if $(word 2,$(GS_OUTPUT_DIRS)), (more than one output/ dir found: $(GS_OUTPUT_DIRS) -- pass GS_DEFCONFIG= to pick))"
	@echo "  GS_OUTPUT      = $(GS_OUTPUT)"
	@echo "  CC             = $(if $(RX_CC),$(RX_CC),(not found -- falls back to rx/Makefile's own default))"
	@echo "  AR8030_SDK_INC = $(if $(RX_SDK_INC),$(RX_SDK_INC),(not found -- falls back to rx/Makefile's own default))"
	@echo "  AR8030_SDK_LIB = $(if $(RX_SDK_LIB),$(RX_SDK_LIB),(not found -- falls back to rx/Makefile's own default))"
