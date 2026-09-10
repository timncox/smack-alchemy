# =============================================================================
# smack-alchemy — Smack for the Hermetic Modular Alchemy Lab V2
#
# Standard Daisy workflow (libDaisy core Makefile underneath):
#   make libdaisy       — build lib/libDaisy once after cloning
#   make                — build firmware -> build/smack_alchemy.bin
#   make program-dfu    — flash over USB (module in DFU mode first)
#   make program-live   — reboot the running module over USB and flash it
#   make test           — native test suites (no hardware)
#   make clean          — remove the build tree
#
# Build flags:
#   BENCH_USB=1   HostLink on the Seed's micro-USB (bench bootloader setup)
#                 instead of the front USB-C. Rebuild without it before racking.
#
# Default goal is libDaisy's `all` (a build). Nothing here flashes unless a
# program-* target is named explicitly.
# =============================================================================

TARGET = smack_alchemy

# Alchemy Lab board revision: v1 | v2
BOARD ?= v2
ifeq ($(filter $(BOARD),v1 v2),)
$(error BOARD must be 'v1' or 'v2' (got '$(BOARD)'))
endif

ALCHEMY_DIR  = lib/alchemy-sdk
LIBDAISY_DIR = lib/libDaisy

# ── App sources ─────────────────────────────────────────────────────────────
CPP_SOURCES = \
    src/smack_alchemy.cpp \
    src/picker.cpp

# smack_core_alchemy.c #includes vendor/smack_core.c with calloc/free
# redirected to SDRAM. Never list vendor/smack_core.c here as well —
# that would define every engine symbol twice.
C_SOURCES = \
    src/clock_adapter.c \
    src/versio_alloc.c \
    src/smack_core_alchemy.c

# ── Alchemy SDK, compiled straight from the submodule ───────────────────────
CPP_SOURCES += $(sort $(shell find $(ALCHEMY_DIR)/framework/src -name '*.cpp'))
CPP_SOURCES += $(sort $(wildcard $(ALCHEMY_DIR)/hardware/alchemy-lab/$(BOARD)/src/*.cpp))

C_INCLUDES += \
    -Isrc \
    -I$(ALCHEMY_DIR)/framework/include \
    -I$(ALCHEMY_DIR)/hardware/include \
    -I$(ALCHEMY_DIR)/hardware/alchemy-lab/$(BOARD)/include

ifeq ($(BOARD),v2)
C_DEFS += -DALCHEMY_BOARD_V2
endif

SMACK_VERSION  := $(shell cat VERSION 2>/dev/null || echo 0.0.0)
SMACK_GIT_HASH := $(shell git rev-parse --short HEAD 2>/dev/null || echo dev)
C_DEFS += -DSMACK_VERSION=\"$(SMACK_VERSION)\" -DSMACK_GIT_HASH=\"$(SMACK_GIT_HASH)\"

ifeq ($(BENCH_USB),1)
C_DEFS += -DSMACK_BENCH_USB
endif

# ── Daisy bootloader build (BOOT_SRAM) ──────────────────────────────────────
# The STM32H750 has 128 KB of internal flash; this app plus the SDK is well
# over that, so it runs from SRAM under the Daisy bootloader like every
# Alchemy Lab firmware.
# FatFS for the SD card. libdaisy.a carries ff.o but not the code-page
# helper (option/ccsbcs.c) that long-filename support links against;
# USE_FATFS=1 makes libDaisy's core Makefile compile the FatFS set into the
# app, which is how every SD-using Daisy firmware does it.
USE_FATFS = 1

APP_TYPE = BOOT_SRAM
# The SDK's alchemy_stm32h750ib_sram.lds plus one section: the SDK's SD-card
# code places FatFS work buffers in `.axi_bss`, which its own script does not
# define (see alchemy/storage/sd_card.h). Re-copy from the SDK and re-add the
# section when the submodule moves.
LDSCRIPT = src/smack_alchemy.lds

# The Alchemy SDK requires C++17 (libDaisy's default is gnu++14).
CPP_STANDARD = -std=gnu++17

# -O3, not -Os: this is a BOOT_SRAM build with room to spare, and the audio
# callback is the one budget that can kill the project (smack-versio's
# Makefile carries the measurements). No -ffast-math: the seeded FX depend
# on float evaluation order.
OPT = -O3

# ── libDaisy core Makefile does the rest ────────────────────────────────────
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

.DEFAULT_GOAL := all

# newlib-nano prints NOTHING for "%f" unless _printf_float is linked in.
# The engine formats integers only, but this stops the next float param
# from failing silently (see smack-versio's Makefile for the history).
override LDFLAGS += -u _printf_float

BOARD_STAMP := $(BUILD_DIR)/.board-$(BOARD)
ifeq ($(wildcard $(BOARD_STAMP)),)
_BOARD_GUARD := $(shell rm -f $(BUILD_DIR)/*.o $(BUILD_DIR)/*.d $(BUILD_DIR)/*.lst $(BUILD_DIR)/.board-* 2>/dev/null; mkdir -p $(BUILD_DIR); touch $(BOARD_STAMP))
endif

.PHONY: libdaisy
libdaisy:
	$(MAKE) -C $(LIBDAISY_DIR)

# ── Flash without touching the module ───────────────────────────────────────
# HostLink reboots the running module into the bootloader over the same USB
# connection the web editor uses, then dfu-util (-w waits for the DFU device
# to enumerate) writes the app.
USBPID ?= df11

.PHONY: program-live
program-live: all
	node $(ALCHEMY_DIR)/tools/hostlink-cli/hostlink.mjs reboot bootloader
	dfu-util -w -a 0 -s $(FLASH_ADDRESS):leave -D $(BUILD_DIR)/$(TARGET_BIN) -d ,0483:$(USBPID)

# ── Native tests ────────────────────────────────────────────────────────────
.PHONY: test
test:
	$(MAKE) -C test
