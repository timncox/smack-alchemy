# Smack on the Alchemy Lab — Design

Port of [smack-versio](../smack-versio) (Smack for the Noise Engineering
Versio) onto the Hermetic Modular **Alchemy Lab V2**, on the Alchemy SDK.
Written 2026-09-10 before anything was heard. Claims are marked
**✅ verified** (with the source) or **⚠️ assumed / unheard**, as in
smack-versio's DESIGN.md; keep that discipline.

## 1. What carries over unchanged

- **The engine** (`src/vendor/smack_core.[ch]`, `plugin_api_v1.h`): the
  smack-versio copy, which carries three fixes found on Versio hardware over
  the schwung-smack upstream (ring stall, LIVE play_frame, integer-only
  formatting). Its header says how to re-vendor. Engine changes go upstream,
  never here.
- **The clock adapter** (`clock_adapter.[ch]`): gate edges → synthetic
  24 ppqn MIDI clock, three tiers (external / inferred / free-run), AUTO
  detection. ✅ 7 native tests.
- **The DJ filter** (`dj_filter.h`): one sweep, LP left, HP right, real
  bypass at the notch. ✅ measured natively. The filter is always in the
  chain here (Versio only ran it in its DJ role), and even "open" its
  biquads take ~0.5 dB off 8 kHz, so the callback skips the block entirely
  at exactly 0 and resets the integrators on the way out.
- **The allocator** (`versio_alloc.[ch]`): a bump allocator so
  `smack_create()`'s four `calloc`s land in a 32 MB SDRAM pool with no
  engine edits. ✅ native test. The name is provenance, not platform.
- **48 kHz, 128-frame blocks.** ✅ verified natively on smack-versio; the
  SDK's default block is 24, so `hw.Init(SAI_48KHZ, 128)`.
- **Sample format**: float in/out, int16 interleaved into the engine, so
  a difference in sound is a shim bug, not a rewrite bug.
- **BLEND in the callback** against the real input (the engine runs
  `monitor = 0`), bypassed until a loop exists so the module is never
  silent before capture.

## 2. Hardware (Alchemy Lab V2, from the SDK BSP)

- STM32H750 on a Daisy Seed2 DFM, 64 MB SDRAM, 8 MB QSPI (as the SDK maps
  it), 128 KB internal flash → BOOT_SRAM under Hermetic's Daisy-bootloader
  fork like every Alchemy firmware.
- 6 pots in two columns (P1 TL, P2 TR, P3 ML, P4 MR, P5 BL, P6 BR), a
  16-LED ring under each; 3 buttons between the columns (B1 top, B2
  middle, B3 bottom), 2 LEDs each.
- Jacks: J1/J2 codec in (AC-coupled), J3–J8 switchable CV in/out
  (inputs by default, ±5 V hardware-scaled), J9/J10 codec out.
- **CV inputs are independent of the pots** — the SDK sums them per
  `(page, pot)` in software. Versio's defining limitation is gone.
- Reserved by the platform: B3 in the 2 s boot window = DFU; B1+B2 through
  boot = factory calibration; **B2+B3 held 2 s = Settings**, B1 cycles
  settings pages, B2 or B3 exits.
- USB: the front USB-C is the Seed's EXTERNAL USB (HostLink, DFU on the
  factory bootloader); the Seed's micro-USB is ROM DFU only, unless the
  bench (intdfu) bootloader is installed. `BENCH_USB=1` moves HostLink to
  the micro-USB for that setup.

## 3. Control surface

Versio had seven absolute knobs, two switches and one button. Here: six
pots on two pages, three buttons. The SDK's **Pager** makes the second page
a *layer* — held on B3, with pot catch on entry and exit — so nothing on
the base page can jump.

### PLAY page (base)

| Pot | Parameter | Engine | Ring |
|---|---|---|---|
| P1 | **FX** density 0–100 | `fx_density` | level |
| P2 | **ORDER** density 0–100 | `order_density` | level |
| P3 | **LENGTH** 8/16/32/64/128/256 steps | `loop_len` 3..8 | 6-zone selector + **playhead pip** once per loop pass |
| P4 | **SLICE** resolution | `slice_res` 0..3 | 4-zone selector |
| P5 | **BLEND** dry ↔ loop | callback crossfade | level |
| P6 | **DJ FILTER**, notch at centre | callback, last in chain | bipolar |

### SETUP page (hold B3)

| Pot | Parameter | Engine / target | Ring |
|---|---|---|---|
| P1 | **SEED** 0–127 | `seed` | level |
| P2 | **PITCH RANGE** 1–24 semitones | `pitch_range` | level |
| P3 | **CLOCK RATIO** ÷2 / =1 / ×2 | `clk_set_ratio` | 3-zone selector |
| P4 | **PUNCH FX** clean + 26 effects | held by B2 | level |
| P5 | **MODE** stereo / dual | `channel_mode` | 2-zone selector |
| P6 | **TEMPO** 50–200 BPM, free-run | `clk_set_free_bpm` | level |

Why this split: SEED leaves the base page because B1's tap already re-rolls;
the DJ filter gets its own pot and PITCH RANGE its own, so Versio's
"what does the PITCH knob do" setting is gone; both switches become
selector pots on the layer.

### Buttons

| Button | Gesture | Does |
|---|---|---|
| **B1 CAPTURE** | tap | re-roll (new pattern, same loop) |
| | hold > 0.6 s | capture the last LENGTH steps (+ `detect_bpm` if unclocked) |
| | hold > 2 s | drop the loop |
| | double-tap | LIVE on/off (re-capture every loop pass) |
| **B2 PUNCH** | hold | force every slice through the SETUP punch effect; release restores |
| **B3 SETUP** | hold | the SETUP page |
| B2 + B3 | hold 2 s | Settings (SDK) |

The Versio needed its MODE switch to turn the one button into a punch;
here punch and capture coexist. One consequence: B2 is also half of the
Settings chord, so holding B2+B3 punches for the two seconds before
Settings opens, then releases. Suppressing punch while B3 is down would
cost punch on the SETUP page, a worse trade. Versio's triple-tap config layer is
replaced by the SETUP page and Settings. Gestures are hand-rolled from
`Pressed()` at the 1 ms poll (`OnPoll`), as on Versio, because the SDK's
ButtonBank has no double-tap by design.

### LEDs

| Where | Shows |
|---|---|
| rings | the SDK draws every pot's value; SETUP rings while B3 is held |
| LENGTH ring | a white pip runs once around per loop pass |
| B1 pair | STATE: dim blue idle · amber armed · red recording · green looping · cyan LIVE · white while punching |
| B2 pair | CLOCK: blue external · purple inferred · grey free-running · **red = callback over 80%** |
| B3 pair | dim Setup tint; the SDK paints it while the page is held |
| P1 ring, first 2.5 s | last session's worst CPU load as a bar: dim blue = no data; green < 75%, amber < 90%, red = it did not fit |

### CV (J3–J8 = CV 0–5)

| Jack | Role |
|---|---|
| J3 | **CLOCK / TRIG** — the Versio gate jack. AUTO: steady train = clock, sporadic hits = capture triggers with tempo inferred |
| J4 | → FX |
| J5 | → ORDER |
| J6 | → BLEND |
| J7 | → SLICE |
| J8 | → SEED (on the SETUP page; the SDK routes to `(page, pot)` cells) |

Static for v0.1. Runtime routing through Settings selectors, and a **CV
output** (a per-slice trigger on J8's STM DAC — the thing only this platform
can do) are v0.2 candidates.

**J3 is read as raw ADC in the audio callback**, not through the SDK's
`AnalogControl` (20 Hz low-passed for pots — it would smear a pulse). The
threshold is on the **magnitude** of the deviation from the jack's
calibrated 0 V code (+1.5 V assert, +0.5 V release), so a pulse fires
whichever way the front end counts. ⚠️ **Unheard.** The documented sign
(higher code = more negative volts) is not relied on.

## 4. Settings (B2+B3 held 2 s)

- Page 0: brightness (SDK, P1), preset slot + action (SDK, P3/P4),
  **CLOCK IN** (P5): Auto / Clock — Versio's "always trust the gate".
- Page 1 **FIRMWARE** — the picker, `src/picker.cpp`:
  - **P1 FILE**: the `.bin` files in `/alchemy` on the card, one dot per
    file, the chosen one bright. Red: no card. Orange: no folder or no
    files.
  - **P2 FLASH**: turn below 85 % then past 95 % (the arming gesture, so
    a pot parked at maximum cannot fire on entry). Orange = erasing (one
    64 KB sector per frame), blue = writing (one 4 KB chunk per frame),
    green = verifying against the memory-mapped view, white then reboot
    via `ResetToBootloader(DAISY_SKIP_TIMEOUT)`. Blinking red = failed,
    nothing booted; back the pot off to retry.
  - **P3 DFU**: same gesture → `ResetToBootloader(DAISY_INFINITE_TIMEOUT)`,
    the escape hatch for a firmware without a picker.
  - Refuses images that are not bootable from SRAM (an internal-flash build
    would leave the bootloader with nothing to boot); the size cap is the
    calibration sector (`0x9075F000`) minus the app slot, so calibration and
    presets are never touched. ✅ `.map`: no code or data of this build
    lives in QSPI, so erasing the slot while running is safe.
  - Files reach the card by pulling it or over USB through HostLink's
    filesystem block (§8 of the protocol; the web programmer's file panel).

## 5. Persistence

The SDK's **Presets** manage the pager (both pages), Settings, and
`SmackExtras` {`cpu_peak`, `free_bpm`}. Slot 0 is the working state,
**autosaved** 5 s after the last SETUP / Settings / extras change with hands
off the buttons and the picker idle. Epsilons (0.5 BPM, 5 points of CPU)
are the wear limiter. `Presets::Save` erases only the sectors the record
covers — one — with yields between them (✅ `preset_store.h`).

At boot: `BootLoad()` restores slot 0, then the **PLAY page adopts the
physical pots** (`SetStored = phys`) so base knobs are live with no catch to
cross; SETUP keeps its stored values and catches. `Deserialize` leaves the
Pager a deferred re-arm for its first `Update()`; I observe (`pager.cpp`
`LockPage`) it only re-runs `InitCatch` against phys and never writes the
stored values, so the adopt survives it. First boot after a flash
seeds SETUP with seed 0, 12 semitones, ratio =1, punch RETRIG, stereo,
120 BPM.

Tempo bookkeeping: when the clock locks, the locked tempo becomes the
free-run default and the TEMPO pot's stored value (catch, so the pot picks
up from there); unlocked, a finished `detect_bpm` scan does the same.

## 6. Storage plumbing

- `SdCard` (SDK) owns volume `0:` with its FATFS in `.axi_bss`;
  `FsExtension` serves it over HostLink.
- **`.axi_bss` is not in the SDK's linker script.** `src/smack_alchemy.lds`
  is the SDK's `alchemy_stm32h750ib_sram.lds` @ 7b1cf0c plus that one
  section in AXI SRAM (SDMMC1's IDMA cannot reach DTCM, where `.bss` lives
  on a bootloader build). ld's `INSERT AFTER .bss` could not find the
  section across two `-T` scripts, hence the copy.
- **`USE_FATFS = 1`** in the Makefile, or `ff_convert` / `ff_wtoupper` are
  undefined: `libdaisy.a` has `ff.o` but not `option/ccsbcs.o`.
- The picker's FIL/DIR/chunk buffers follow the SDK's three placement
  rules (never DTCM, AXI SRAM, word-aligned direct transfers: 4 KB chunks
  at 4 KB offsets).

## 7. HostLink

`Host` with product "Smack", both pages, ten jacks, three buttons, a
manual, the filesystem extension. The transport is the front USB-C
(`FS_EXTERNAL`, the SDK default for V2) or, with `BENCH_USB=1`, the Seed's
micro-USB (`FS_INTERNAL`) via `Host::Transport()`. Enables the web
programmer, `make program-live`, and SD uploads.

## 8. Budgets (✅ from the 2026-09-10 build)

| Region | Used | Of |
|---|---|---|
| SRAM (code + data, BOOT_SRAM) | 364,704 B | 480 KB (74 %) |
| DTCM | 75,584 B | 128 KB |
| SDRAM | 33.6 MB | 64 MB (the 32 MB pool + HostLink buffers) |
| `.bin` | 356,796 B | |

CPU: ⚠️ unmeasured here. smack-versio ran 50–75 % normal, 75–90 % DUAL at
-O3 on the same silicon; the SDK adds a 1 ms control poll and the LED
driver. Read the boot bar after a hard session.

## 9. Build, flash

    make libdaisy                       once (submodules recursive)
    make                                build/smack_alchemy.bin
    make BENCH_USB=1 BUILD_DIR=build-bench
    make test

Bench (Hermetic intdfu bootloader on the Seed micro-USB): hold B3 at
power-on → "Alchemy Lab" DFU →
`dfu-util -a 0 -s 0x90040000:leave -D build/smack_alchemy.bin -d ,0483:df11`.
Front port (factory bootloader): same, or `make program-live` once HostLink
is up on that port.

## 10. Open questions

1. ⚠️ Does anything play? First rack / Erica-case power-on answers it.
2. ⚠️ Clock jack polarity and thresholds (§3).
3. ⚠️ CPU with the SDK's control loop on top of the engine.
4. ⚠️ Does `Settings` persist Custom slots' bytes (harmless either way)?
5. Autosave wear: one 4 KB sector per save, debounced; watch it.
6. The SDK is beta; the pinned SHAs are what this was built against.

## 11. Risks

- **Silent SD corruption** if any FatFS buffer ever lands outside AXI SRAM
  (the SDK's rule 1–3). Check the `.map` for `.axi_bss` after any linker
  change.
- **Flashing a Patch build**: the picker's vector check would accept any
  SRAM-bootable Daisy image. The `/alchemy` folder name is the guard; the
  Patch card uses `/modules`.
- **Same engine, three copies** (smack, smack-versio, here). Land fixes
  upstream first.
