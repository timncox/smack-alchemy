# smack-alchemy

**Smack** — live loop capture + seeded per-slice glitch — for the
[Hermetic Modular Alchemy Lab V2](https://hermeticmodular.com/modules/alchemy-lab),
built on the [Alchemy SDK](https://github.com/hermetic-modular/alchemy-sdk).

Same C engine as [smack-versio](https://github.com/timncox/smack-versio),
recompiled for six pots on two pages, three buttons, six CV jacks and 102
LEDs. Records everything you play into a ring, grabs the last LENGTH steps
on demand, cuts the loop into slices and runs a seeded pattern of effects
across them.

**Status: v0.1.0 builds and passes its native suites. Nothing has been heard
yet.** See `DESIGN.md` for the control map, what is verified and what is not.

## Build

Requires `arm-none-eabi-gcc`, `make`, `dfu-util`.

```sh
git clone --recurse-submodules <this repo>
cd smack-alchemy
make libdaisy        # once
make                 # build/smack_alchemy.bin
make test            # native suites, no hardware
```

## Flash

Front USB-C with the factory bootloader: hold B3 during the ~2 s boot
window (rings spin a warm-white comet), the rings switch to a slow breathe,
then:

```sh
make program-dfu
```

or the [web programmer](https://hermeticmodular.com/program). Once this
firmware is running on the front port, `make program-live` reboots and
flashes without touching the module.

Bench setup (Hermetic's intdfu bootloader on the Seed's micro-USB): same
gesture on the micro-USB port; build with `BENCH_USB=1` if you want HostLink
there too.

## Firmware on the SD card

Put `.bin` files built for the Alchemy Lab bootloader in a folder named
`alchemy` on a FAT32 card. Hold B2+B3 for two seconds, tap B1 to the
Firmware page, pick a file with the top-left pot, then turn the top-right
pot down and all the way up. It writes, verifies, and reboots into the
chosen firmware.

## The site

`docs/index.html` is the operation manual, a single file meant for GitHub
Pages from `main:/docs`. Its panel drawing is generated from the SDK's KiCad
front-panel template by `tools/emit_panel_geometry.py`; run it with
`--check` before publishing and `--write` after the template changes, and
never hand-edit the coordinates. The repository URL the page links to is one
constant at the top of the file.

## Not a Hermetic Modular product

Custom firmware. Hermetic Modular did not write, test or endorse it; ask
here, not there.

## License

MIT (this firmware). The vendored engine is Tim Cox's; see the file headers.
