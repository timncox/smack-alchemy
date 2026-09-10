# Native tests — run on the laptop, no hardware.
#
#   make -C test        (or `make test` from the repo root)
#
# Carried over from smack-versio unchanged: the clock adapter, the SDRAM
# bump allocator (linked through the same wrapper unit the firmware builds),
# the DJ filter's measured frequency response, and the ring-stall regression.
# The engine's own suite lives upstream in ~/tim-os/smack (`make test`).
#
# The tests include "../clock_adapter.h" etc. relative to smack-versio's
# firmware/ layout; -I../src/vendor makes those paths resolve into src/ here
# without editing the files, so they stay byte-identical to upstream.

CC     ?= cc
CFLAGS  = -O2 -g -Wall -Wextra -I../src -I../src/vendor
BUILD   = ../build/test

.PHONY: test clean

test: $(BUILD)/test_clock_adapter $(BUILD)/test_versio_alloc \
      $(BUILD)/test_dj_filter $(BUILD)/test_ring_stall
	./$(BUILD)/test_clock_adapter
	./$(BUILD)/test_versio_alloc
	./$(BUILD)/test_dj_filter
	./$(BUILD)/test_ring_stall

$(BUILD)/test_clock_adapter: test_clock_adapter.c ../src/clock_adapter.c \
                             ../src/vendor/smack_core.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ -lm

# Links smack_core_alchemy.c — the same unit the firmware builds — so the
# calloc/free redirect is exercised for real, not simulated.
$(BUILD)/test_versio_alloc: test_versio_alloc.c ../src/versio_alloc.c \
                            ../src/smack_core_alchemy.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ -lm

$(BUILD)/test_dj_filter: test_dj_filter.c ../src/dj_filter.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) test_dj_filter.c -o $@ -lm

# Drives the vendored engine for minutes of simulated audio to catch the ring
# recorder stalling behind a long loop. Slow by nature, so it runs last.
$(BUILD)/test_ring_stall: test_ring_stall.c ../src/vendor/smack_core.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ -lm

clean:
	rm -rf $(BUILD)
