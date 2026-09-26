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

/* ---------------------------------------------------------------------------
 * Novation Launch Control XL (0x1235:0x0061), alongside the Mini through the
 * same hub. The driver selects factory template 1 at connect (and again if
 * the template is changed on the device); the MIDI channel is ignored.
 * Knobs: 3 rows x 8 (CC 13-20, 29-36, 49-56), faders: 8 (CC 77-84),
 * buttons: 2 rows x 8 under the faders (notes 41-44 57-60 / 73-76 89-92).
 * Knob and fader values are kept as a table -- read them with Knob()/Fader(),
 * which say whether the value changed since the last read -- so a fast
 * sweep never overflows a queue. Button presses are queued.
 */
namespace xl
{

/* LED colour: red 0..3, green 0..3 (bicolour). */
constexpr uint8_t Col(uint8_t red, uint8_t green)
{
    return (uint8_t)(((green & 3u) << 4) | (red & 3u) | 0x0Cu);
}
constexpr uint8_t kOff      = Col(0, 0);
constexpr uint8_t kRed      = Col(3, 0);
constexpr uint8_t kRedDim   = Col(1, 0);
constexpr uint8_t kGreen    = Col(0, 3);
constexpr uint8_t kGreenDim = Col(0, 1);
constexpr uint8_t kAmber    = Col(3, 3);
constexpr uint8_t kAmberDim = Col(1, 1);

struct Button
{
    uint8_t row; /* 0 = upper row, 1 = lower row */
    uint8_t col; /* 0..7 */
    bool    down;
};

bool Connected();
/* True if the control moved since the last call; *v is its value 0..127. */
bool Knob(uint8_t row, uint8_t col, uint8_t* v);
bool Fader(uint8_t col, uint8_t* v);
/* The last value seen, whether or not it changed. */
uint8_t KnobValue(uint8_t row, uint8_t col);
uint8_t FaderValue(uint8_t col);
bool PopButton(Button* b);

void SetKnobLed(uint8_t row, uint8_t col, uint8_t colour);
void SetButtonLed(uint8_t row, uint8_t col, uint8_t colour);

} // namespace xl

/* ---------------------------------------------------------------------------
 * An XInput gamepad (e.g. a Haute42 leverless on GP2040-CE in XInput mode,
 * 045E:028E) through the same hub. Only through the hub: plugged straight
 * into the Lab it is not recognised. What is held is kept as a bit mask --
 * the buttons are for holding, and a mask cannot overflow a queue.
 * Bits 0-15 are XInput's wButtons; the triggers count as buttons 16/17.
 */
namespace pad
{

constexpr uint32_t kUp    = 1u << 0;
constexpr uint32_t kDown  = 1u << 1;
constexpr uint32_t kLeft  = 1u << 2;
constexpr uint32_t kRight = 1u << 3;
constexpr uint32_t kStart = 1u << 4;
constexpr uint32_t kBack  = 1u << 5;
constexpr uint32_t kL3    = 1u << 6;
constexpr uint32_t kR3    = 1u << 7;
constexpr uint32_t kLB    = 1u << 8;
constexpr uint32_t kRB    = 1u << 9;
constexpr uint32_t kGuide = 1u << 10;
constexpr uint32_t kA     = 1u << 12;
constexpr uint32_t kB     = 1u << 13;
constexpr uint32_t kX     = 1u << 14;
constexpr uint32_t kY     = 1u << 15;
constexpr uint32_t kLT    = 1u << 16;
constexpr uint32_t kRT    = 1u << 17;

bool     Connected();
uint32_t Buttons();      /* held now */
uint32_t ReportCount();  /* reports received, for a readout */

} // namespace pad
