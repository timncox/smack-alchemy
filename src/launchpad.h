/*
 * launchpad -- a Novation Launchpad Mini MK3 on the front USB-C, with the
 * Alchemy Lab as the USB host.
 *
 * Shared by copy with smack-alchemy, belt-alchemy and seq-alchemy.
 *
 * The Lab's front port has no VBUS (Hermetic: "identical to ... the Daisy
 * itself, minus VBUS"), so the Launchpad needs 5 V injected -- by a PASSIVE
 * injector: libDaisy's host has no hub class, and a powered "OTG adapter"
 * with a hub inside enumerates as the hub (seen 2026-09-25: VIA 2109:2817).
 * libDaisy runs one USB role per boot on that port: while this driver owns
 * it there is no HostLink and no DFU-over-USB.
 *
 * MK3 Programmer mode (Launchpad Mini MK3 programmer's reference): entered
 * with SysEx F0 00 20 29 02 0D 0E 01 F7. Grid pad = note 10*row + col,
 * row 1 at the BOTTOM, col 1 at the left; the right-hand column is CC
 * 19, 29 .. 89; the top row is CC 91..98; the logo is 99. Presses are
 * velocity/value 127, releases 0. LED colour = palette index (0..127) as
 * the velocity/value on channel 1 (static).
 *
 * The MK3 has two USB-MIDI cables (DAW and MIDI). Which is which is not
 * assumed: the mode switch and the LEDs go to both, and incoming events are
 * taken from whichever cable delivers the first one.
 *
 * Call Init() once, Poll() every millisecond from the control loop (never
 * from the audio callback). Poll runs the USB host, turns incoming packets
 * into events, and sends only the LEDs that changed, at most one 64-byte
 * USB transfer per call and never waiting for the bus.
 */
#pragma once
#include <stdint.h>

namespace lp
{

/* Colours: MK3 palette indices. */
constexpr uint8_t kOff       = 0;
constexpr uint8_t kWhite     = 3;
constexpr uint8_t kRed       = 5;
constexpr uint8_t kRedDim    = 7;
constexpr uint8_t kOrange    = 9;
constexpr uint8_t kAmber     = 13;   /* yellow in the palette */
constexpr uint8_t kAmberDim  = 15;
constexpr uint8_t kYellow    = 13;
constexpr uint8_t kGreen     = 21;
constexpr uint8_t kGreenDim  = 23;
constexpr uint8_t kCyan      = 37;
constexpr uint8_t kBlue      = 45;
constexpr uint8_t kMagenta   = 53;
constexpr uint8_t kGrey      = 1;
constexpr uint8_t kOrangeDim = 11;
constexpr uint8_t kCyanDim   = 39;
constexpr uint8_t kBlueDim   = 47;
constexpr uint8_t kMagentaDim = 55;

enum class Kind : uint8_t { Grid, Side, Top };

struct Event
{
    Kind    kind;
    uint8_t x;    /* grid 0..7 (left to right), top 0..7 */
    uint8_t y;    /* grid 0..7 (top to bottom), side 0..7 */
    bool    down;
};

void Init();
void Poll(uint32_t now_ms);
bool Connected();
bool PopEvent(Event* e);

/* The LED framebuffer. Changes go out on the next Polls. */
void SetGrid(uint8_t x, uint8_t y, uint8_t col);
void SetSide(uint8_t y, uint8_t col);
void SetTop(uint8_t x, uint8_t col);
void SetLogo(uint8_t col);
void ClearAll();

/* Where the USB host is, for a status light:
 *   0 no device on the wire (no D+ pull-up seen)
 *   1 a device is attached, enumeration in progress or stuck
 *   2 the host gave up (abort / unsupported), or an error fired
 *   3 a MIDI device is active */
uint8_t Stage();

/* Enumeration forensics, latched at the first abort. */
struct Diag
{
    uint8_t  state_before_abort; /* HOST_StateTypeDef reached last, 0..13 */
    uint8_t  enum_at_abort;      /* ENUM_StateTypeDef + 1, 0 = no abort yet */
    bool     vid_ok;             /* the device descriptor said Novation */
    uint16_t aborts;
};
Diag Diagnostics();

/* A text report of the host's state changes and what it read from the
 * device, for writing to the SD card. Returns the length written. */
int Report(char* buf, int cap);

/* Diagnostics for a readout: packets received, transfers sent. */
uint32_t RxCount();
uint32_t TxCount();

} // namespace lp
