/*
 * Smack on the Hermetic Modular Alchemy Lab V2 -- live loop capture + seeded
 * per-slice glitch, as Alchemy SDK firmware.
 *
 * The DSP is smack_core, vendored from smack-versio (which carries three
 * hardware-found fixes over the schwung-smack upstream; see
 * vendor/smack_core.h). This file is the host shim: it wires six pots on
 * two pages, three buttons, six CV jacks and 102 LEDs to the engine's string
 * parameter API, and turns the clock jack into the 24 ppqn MIDI clock the
 * engine already speaks (clock_adapter.c, unchanged from the Versio port).
 *
 * See DESIGN.md. The short version, inherited from smack-versio:
 *   - 48 kHz because libDaisy offers no 44.1 (sai.h)
 *   - 128-frame blocks because the engine's clock regression was built
 *     against 128 and misbehaves at smaller blocks (verified natively);
 *     the SDK's default is 24, so Init() is told otherwise
 *   - the ring lives in SDRAM via a bump allocator, so the engine needs
 *     no edits at all
 *
 * What is new here and not on the Versio:
 *   - the SDK's Pager gives a second page under B3 (hold), with pot catch,
 *     so SEED / PITCH RANGE / CLOCK RATIO / PUNCH FX / MODE / TEMPO each
 *     get a pot instead of a switch or a borrowed knob
 *   - the DJ filter has a pot of its own, so Versio's "what does PITCH do"
 *     setting is gone
 *   - punch has a button of its own (B2), so it coexists with capture
 *   - presets (slot 0 autosaved), HostLink (web programmer, program-live,
 *     SD file access), and the SD firmware picker in Settings
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "daisy_seed.h"
#include "hid/usb.h"
#include "util/CpuLoadMeter.h"

#include "alchemy/hw/alchemy_lab.h"
#include "alchemy/hw/v2_calibration.h"
#include "alchemy/host_link/cdc_transport.h"
#include "alchemy/host_link/host.h"
#include "alchemy/storage/fs_extension.h"
#include "alchemy/storage/sd_card.h"
#include "alchemy/surface/control_loop.h"
#include "alchemy/surface/cv_matrix.h"
#include "alchemy/surface/jack.h"
#include "alchemy/surface/manual.h"
#include "alchemy/surface/page.h"
#include "alchemy/surface/pager.h"
#include "alchemy/surface/presets.h"
#include "alchemy/surface/settings.h"
#include "alchemy/surface/virtual_button.h"
#include "alchemy/surface/virtual_knob.h"

#include "clock_adapter.h"
#include "dj_filter.h"
#include "extras.h"
#include "picker.h"
#include "launchpad.h"
#include "usb_shared.h"
#include "ff.h"
#include "versio_alloc.h"

/* versio_alloc.h first, then the engine inside extern "C" -- see the note in
 * versio_alloc.h about include order and linkage. */
extern "C" {
#include "vendor/smack_core.h"
}

#ifndef SMACK_VERSION
#define SMACK_VERSION "0.0.0"
#endif
#ifndef SMACK_GIT_HASH
#define SMACK_GIT_HASH "dev"
#endif

using namespace alchemy;
using daisy::System;

/* ---- geometry ----------------------------------------------------------- */

/* The engine's block. See DESIGN.md: validated at 128, breaks at 48. */
static constexpr uint32_t kBlockSize = 128u;

/* J3 is the clock / trigger jack: CV index 0. */
static constexpr uint8_t kClockJack = 0u;

enum : uint8_t { kPagePlay = 0, kPageSetup = 1, kNumAppPages = 2 };
enum : uint8_t { kSettingsMain = 0, kSettingsFirmware = 1 };

/* ---- hardware + engine -------------------------------------------------- */

static AlchemyLab          hw;
static daisy::CpuLoadMeter cpu;

/* Sized in versio_alloc.h so this and test_versio_alloc.c cannot drift. The
 * ring alone is 28.8 MB; the Seed2 DFM has 64 MB of SDRAM. */
static uint8_t DSY_SDRAM_BSS g_pool[VERSIO_POOL_BYTES];

static smack_t*        S = nullptr;
static clock_adapter_t CLK;
static host_api_v1_t   HOST;

/* Engine state published by the control loop and read by the audio callback.
 * Plain aligned words: a torn read is impossible, a one-block-stale value is
 * inaudible. 3 == SMACK_LOOPING. */
static volatile int   G_RUN_STATE = 0;
static volatile float G_BLEND     = 0.0f;  /* 0 dry .. 1 loop            */
static volatile float G_DJ_CTL    = 0.0f;  /* -1 LP .. 0 open .. +1 HP   */
static volatile float G_PLAYPOS   = 0.0f;  /* 0..1 through the loop pass */
static volatile bool  G_LIVE      = false;
static bool           G_PUNCHING  = false;
static int            g_punch_fx  = 1;     /* SMACK_FX_RETRIG */

/* ---- colours ------------------------------------------------------------ */

static constexpr LedPanel::Rgb kColFx     = {0xFF, 0x50, 0x20};
static constexpr LedPanel::Rgb kColOrder  = {0xFF, 0xA0, 0x20};
static constexpr LedPanel::Rgb kColLen    = {0x40, 0xC0, 0xFF};
static constexpr LedPanel::Rgb kColSlice  = {0x40, 0xFF, 0xC0};
static constexpr LedPanel::Rgb kColBlend  = {0xFF, 0x40, 0xA0};
static constexpr LedPanel::Rgb kColLp     = {0x20, 0x60, 0xFF};
static constexpr LedPanel::Rgb kColHp     = {0xFF, 0xA0, 0x20};
static constexpr LedPanel::Rgb kColNotch  = {0xFF, 0xFF, 0xFF};
static constexpr LedPanel::Rgb kColSeed   = {0xC0, 0x60, 0xFF};
static constexpr LedPanel::Rgb kColPitch  = {0xFF, 0x80, 0xFF};
static constexpr LedPanel::Rgb kColRatio  = {0x80, 0x80, 0xFF};
static constexpr LedPanel::Rgb kColPunch  = {0xFF, 0xFF, 0x80};
static constexpr LedPanel::Rgb kColMode   = {0x80, 0xFF, 0xFF};
static constexpr LedPanel::Rgb kColTempo  = {0xFF, 0xC0, 0x40};
static constexpr LedPanel::Rgb kColDim    = {0x18, 0x18, 0x18};
static constexpr LedPanel::Rgb kColSetup  = {0x30, 0x10, 0x40};

static constexpr LedPanel::Rgb kIdle   = {0x00, 0x00, 0x30};
static constexpr LedPanel::Rgb kAmber  = {0xFF, 0x80, 0x00};
static constexpr LedPanel::Rgb kRed    = {0xFF, 0x00, 0x00};
static constexpr LedPanel::Rgb kGreen  = {0x00, 0xFF, 0x00};
static constexpr LedPanel::Rgb kCyan   = {0x00, 0xC0, 0xC0};
static constexpr LedPanel::Rgb kWhite  = {0xFF, 0xFF, 0xFF};
static constexpr LedPanel::Rgb kGrey   = {0x60, 0x60, 0x60};
static constexpr LedPanel::Rgb kPurple = {0x80, 0x00, 0xC0};
static constexpr LedPanel::Rgb kBlue   = {0x00, 0x50, 0xFF};

/* ---- labels (descriptor metadata; static storage, borrowed by pointer) -- */

static const char* const kLengthLabels[6] = {"8", "16", "32", "64", "128", "256"};
/* Slice size in clock steps, in the engine's own order (slice_hs_table:
 * 1, 2, 4, 8 half-steps), as the Move names them. Index 0 is the FINEST
 * cut; an earlier label set ran Coarse..Finest and was backwards. */
static const char* const kSliceLabels[4]  = {"1/2 step", "1 step", "2 steps", "4 steps"};
static const char* const kRatioLabels[3]  = {"/2", "=1", "x2"};
static const char* const kModeLabels[2]   = {"Stereo", "Dual"};
static const char* const kClockModes[2]   = {"Auto", "Clock"};

/* Index == smack_fx_t; 0 is "clean", which punches the pattern OUT. */
static const char* const kPunchLabels[27] = {
    "Clean", "Retrig", "Reverse", "Pitch", "Speed", "Gate", "Buzz", "Crush",
    "Repeat", "Rev After", "Tape Stop", "Tape Start", "Scratch", "Env", "Pan",
    "Filter", "Vowel", "Tonal Delay", "Freeze", "Delay", "Dist", "Phaser",
    "Verb", "P-Shift", "Ring Mod", "Comb", "Scatter",
};
static_assert(SMACK_FX_COUNT == 27, "kPunchLabels is out of step with smack_fx_t");

/* ---- pages --------------------------------------------------------------- */

/* PLAY: what you reach for while it runs. Panel positions, front view:
 *      [P1] B1 [P2]
 *      [P3] B2 [P4]
 *      [P5] B3 [P6]                                                        */

static VirtualKnob fx = VirtualKnob(kPotTopLeft, "FX")
    .Linear(0.f, 100.f).Ident("fx").Unit("%")
    .Ring(Level(kColFx));

static VirtualKnob order = VirtualKnob(kPotTopRight, "Order")
    .Linear(0.f, 100.f).Ident("order").Unit("%")
    .Ring(Level(kColOrder));

static VirtualKnob length = VirtualKnob(kPotMiddleLeft, "Length")
    .Selector(6).Labels(kLengthLabels).Ident("length")
    .Ring(SelectorRing(kColLen, kColDim, 6));

static VirtualKnob slice = VirtualKnob(kPotMiddleRight, "Slice")
    .Selector(4).Labels(kSliceLabels).Ident("slice")
    .Ring(SelectorRing(kColSlice, kColDim, 4));

static VirtualKnob blend = VirtualKnob(kPotBottomLeft, "Blend")
    .Linear(0.f, 100.f).Ident("blend").Unit("%")
    .Ring(Level(kColBlend));

static VirtualKnob djf = VirtualKnob(kPotBottomRight, "DJ Filter")
    .Linear(-1.f, 1.f).Ident("djf")
    .Ring(Bipolar(kColHp, kColLp, kColNotch));

/* SETUP: hold B3. Set-once things and the seed. */

static VirtualKnob seed = VirtualKnob(kPotTopLeft, "Seed")
    .Linear(0.f, 127.f).Ident("seed")
    .Ring(Level(kColSeed));

static VirtualKnob pitch = VirtualKnob(kPotTopRight, "Pitch Range")
    .Linear(1.f, 24.f).Ident("pitch").Unit("st")
    .Ring(Level(kColPitch));

static VirtualKnob ratio = VirtualKnob(kPotMiddleLeft, "Clock Ratio")
    .Selector(3).Labels(kRatioLabels).Ident("clk.ratio")
    .Ring(SelectorRing(kColRatio, kColDim, 3));

static VirtualKnob punch = VirtualKnob(kPotMiddleRight, "Punch FX")
    .Selector(27).Labels(kPunchLabels).Ident("punch.fx")
    .Ring(Level(kColPunch));

static VirtualKnob mode = VirtualKnob(kPotBottomLeft, "Mode")
    .Selector(2).Labels(kModeLabels).Ident("mode")
    .Ring(SelectorRing(kColMode, kColDim, 2));

static VirtualKnob tempo = VirtualKnob(kPotBottomRight, "Tempo")
    .Linear(50.f, 200.f).Ident("tempo").Unit("BPM")
    .Ring(Level(kColTempo));

static Page play_page = Page(kPagePlay).Name("Play").Color("#40c0ff")
    .Knobs(fx, order, length, slice, blend, djf);

static Page setup_page = Page(kPageSetup).Name("Setup").Color("#c060ff")
    .Knobs(seed, pitch, ratio, punch, mode, tempo);

/* ---- descriptor metadata -------------------------------------------------- */

static Jack jk_in_l   ("J1",  "In L",     JackSig::AudioIn);
static Jack jk_in_r   ("J2",  "In R",     JackSig::AudioIn);
static Jack jk_clk    ("J3",  "Clock",    JackSig::Trig);
static Jack jk_cv_fx  ("J4",  "CV FX",    JackSig::CvBi);
static Jack jk_cv_ord ("J5",  "CV Order", JackSig::CvBi);
static Jack jk_cv_bl  ("J6",  "CV Blend", JackSig::CvBi);
static Jack jk_cv_sl  ("J7",  "CV Slice", JackSig::CvBi);
static Jack jk_cv_sd  ("J8",  "CV Seed",  JackSig::CvBi);
static Jack jk_out_l  ("J9",  "Out L",    JackSig::AudioOut);
static Jack jk_out_r  ("J10", "Out R",    JackSig::AudioOut);

static VirtualButton bt_capture = VirtualButton("b1", "Capture")
    .Action("Tap", "Re-roll the pattern")
    .Action("Hold", "Capture the last LENGTH steps")
    .Action("Hold 2 s", "Drop the loop")
    .Action("Double-tap", "LIVE on / off");

static VirtualButton bt_punch = VirtualButton("b2", "Punch")
    .Action("Hold", "Force every slice through the punch effect");

static VirtualButton bt_setup = VirtualButton("b3", "Setup")
    .Action("Hold", "Show the Setup page");

static Manual manual = Manual()
    .Tagline("Live loop capture + seeded per-slice glitch")
    .Preamble(
        "**Smack** records everything you play into a ring, grabs the last "
        "LENGTH steps on demand, cuts the loop into slices and runs a "
        "seeded pattern of effects across them. The same seed always "
        "gives the same pattern; tap CAPTURE to roll another. Double-tap "
        "for LIVE, where the loop re-captures itself every pass so it "
        "follows what you are playing now. Hold B3 for the Setup page: "
        "seed, pitch range, clock ratio, punch effect, dual mode, tempo.");

/* ---- surfaces ------------------------------------------------------------ */

static ControlLoop loop(hw);
static Pager       pager = Pager(kNumAppPages, kNumPots)
                               .Shift(hw.buttons[kButtonB3], kPageSetup);

/*
 * This firmware's own preset slot. Every Alchemy firmware in the family
 * keeps its working state in a slot of its own -- Smack 12, Mark 13, Belt
 * 14, Relay 15 -- so switching firmware through the SD picker no longer
 * resets the one you left. Slot 0, where they all used to autosave, is
 * read once at boot for migration only (see main). Slots 0-11 are left for
 * presets saved by hand; saving one by hand into 12-15 overwrites that
 * firmware's working state.
 */
static constexpr uint8_t kHomeSlot = 12u;
static Presets     presets(hw.seed.qspi);
static Settings    settings(hw, &pager);
static CvMatrix    cv_matrix(kNumCvInputs);
static SdCard      sd;
static hostlink::FsExtension fs_ext(sd);
static hostlink::Host host(presets, "smack_alchemy", "Smack",
                           SMACK_VERSION, SMACK_GIT_HASH);
static SmackExtras   extras;
static SelectorHandle clock_mode;

#ifdef SMACK_BENCH_USB
/* HostLink on the Seed's own micro-USB instead of the front panel, for the
 * bench-bootloader setup where the front port is not serving anything. */
static daisy::UsbHandle          bench_usb;
static hostlink::CdcUsbTransport bench_cdc;
#endif

/* ---- host shim (the entire host surface smack_core needs) --------------- */

static float host_get_bpm(void)
{
    const float b = clk_bpm(&CLK);
    return (b > 20.0f && b < 300.0f) ? b : 120.0f;
}

/* ---- parameter dispatch ------------------------------------------------- */

/*
 * The engine takes strings, so a parameter is only sent when its quantized
 * value actually changes. Stepped ones also need hysteresis: a pot parked on
 * a step boundary would otherwise chatter across it on ADC noise -- and with
 * CV summed in, that chatter re-slices the loop or re-rolls the pattern
 * continuously. smack-versio's DESIGN.md calls this the most likely "why does
 * it sound broken" bug on this class of hardware, and the SDK leaves
 * zone-commit debouncing to the app (docs/pages-and-layers.md).
 */
#define HYST 0.02f /* ~2% of travel */

struct Knob
{
    const char* key;      /* engine parameter, or nullptr for a local one */
    int         lo, hi;   /* inclusive integer range */
    bool        zones;    /* floor(norm * N), matching the SDK's selector
                             rings, else nearest */
    float       dead;     /* deadband in normalised units; 0 = none */
    int         last      = -32768;   /* -32768 = never dispatched */
    float       last_norm = 0.0f;
};

/* SEED spans 0-127, so by span it looks continuous -- but one step re-rolls
 * the whole pattern, the loudest response any parameter here has to one LSB
 * of noise. It gets a band of exactly one step: HYST would be ~2.5 seeds wide
 * and turning the knob would skip most of the seed space. */
static Knob k_fx    {"fx_density",    0, 100, false, 0.0f};
static Knob k_order {"order_density", 0, 100, false, 0.0f};
static Knob k_len   {"loop_len",      3,   8, true,  HYST};  /* 8..256 steps */
static Knob k_slice {"slice_res",     0,   3, true,  HYST};
static Knob k_seed  {"seed",          0, 127, false, 1.0f / 127.0f};
static Knob k_pitch {"pitch_range",   1,  24, false, HYST};
static Knob k_mode  {"channel_mode",  0,   1, true,  HYST};
static Knob k_ratio {nullptr,         0,   2, true,  HYST};
static Knob k_punch {nullptr,         0,  26, true,  HYST};

static int knob_quantize(const Knob& k, float norm)
{
    const int span = k.hi - k.lo;
    int       v;
    if (k.zones)
    {
        v = (int)(norm * (float)(span + 1));
        if (v > span) v = span;
    }
    else
    {
        v = (int)(norm * (float)span + 0.5f);
    }
    if (v < 0)    v = 0;
    if (v > span) v = span;
    return k.lo + v;
}

/* True, with *out set, when the quantized value moved far enough to count. */
static bool knob_changed(Knob& k, float norm, int* out)
{
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    const int v = knob_quantize(k, norm);
    if (v == k.last) return false;
    if (k.dead > 0.0f && k.last != -32768)
    {
        const float moved = fabsf(norm - k.last_norm);
        if (moved < k.dead) return false;
    }
    k.last      = v;
    k.last_norm = norm;
    *out        = v;
    return true;
}

static void set_param_int(const char* key, int v)
{
    char b[16];
    snprintf(b, sizeof b, "%d", v);
    smack_set_param(S, key, b);
}

static bool dispatch(Knob& k, float norm)
{
    int v;
    if (!knob_changed(k, norm, &v)) return false;
    if (k.key) set_param_int(k.key, v);
    return true;
}

/*
 * DJ filter pot -> one signed word for the audio thread: -1..0 sweeps a
 * lowpass down, 0..+1 a highpass up, 0 is open, with a real notch at the
 * centre wide enough to find by feel. One word so a torn LP/HP flip cannot
 * happen across the interrupt (it would be a click, not a stale block).
 */
static void update_dj_ctl(float n)
{
    const float DEAD = 0.06f;
    const float SPAN = 0.5f - DEAD;
    float       c    = 0.0f;
    if (n < 0.5f - DEAD)      c = (n - (0.5f - DEAD)) / SPAN;
    else if (n > 0.5f + DEAD) c = (n - (0.5f + DEAD)) / SPAN;
    if (c < -1.0f) c = -1.0f;
    if (c > 1.0f)  c = 1.0f;
    G_DJ_CTL = c;
}

/* ---- clock jack ----------------------------------------------------------- */

/*
 * J3 is read as raw ADC in the audio callback, not through the SDK's
 * AnalogControl: that path is low-passed at ~20 Hz for pots, which would
 * smear a millisecond clock pulse into nothing. The board's calibration
 * record gives the jack's 0 V code and the ADC reference, so the thresholds
 * are in volts. The deviation MAGNITUDE is thresholded, so a pulse fires
 * whichever way the ADC counts -- the sign of the front end is documented
 * (higher code = more negative jack volts) but not verified here.
 */
static uint16_t g_clk_zero  = 32768u;
static int      g_clk_hi    = 4915;   /* +1.5 V at VDDA 3.30: assert  */
static int      g_clk_lo    = 1638;   /* +0.5 V:               release */
static bool     g_clk_state = false;

static void clock_calibrate(void)
{
    const V2Calibration& cal = hw.Calibration();
    float vdda = cal.vdda_at_cal;
    if (!(vdda > 2.5f && vdda < 3.6f)) vdda = kV2VddaDesign;
    uint16_t zero = cal.jack[kClockJack].adc_zero_code;
    if (zero < 20000u || zero > 45000u) zero = 32768u;   /* design mid-scale */
    const float cpv = kV2CvInGainDesign * 65535.0f / vdda;  /* ADC counts per volt */
    g_clk_zero = zero;
    g_clk_hi   = (int)(1.5f * cpv);
    g_clk_lo   = (int)(0.5f * cpv);
}

static inline void clock_poll_isr(void)
{
    const uint16_t raw = hw.seed.adc.Get(kCvAdcOffset + kClockJack);
    int            dev = (int)g_clk_zero - (int)raw;
    if (dev < 0) dev = -dev;
    if (!g_clk_state)
    {
        if (dev > g_clk_hi)
        {
            g_clk_state = true;
            clk_gate_edge(&CLK);
        }
    }
    else if (dev < g_clk_lo)
    {
        g_clk_state = false;
    }
}

/* ---- buttons (1 ms poll) -------------------------------------------------- */

/*
 * B1 CAPTURE, ordered by how often you reach for it -- the Versio gestures,
 * minus the triple-tap config layer, which the Setup page replaces:
 *
 *   tap                 re-roll   -- new pattern, same loop
 *   hold  > 600 ms      capture   -- retro-grab the last N steps
 *   hold  > 2000 ms     clear     -- drop the loop, back to passthrough
 *   double-tap          LIVE      -- re-capture once per loop pass
 *
 * B2 PUNCH is momentary: hold to force every slice through the Setup page's
 * punch effect, release to drop back. It stands down while Settings is
 * open (B2+B3 is the Settings chord).
 *
 * Edges come from Pressed(), not RisingEdge(): the ButtonBank is not in use
 * and the framework's poll consumes nothing, but a 1 ms poll against a
 * debouncer's own edge flag is a coin flip (see smack-versio's note).
 */
#define LONG_PRESS_MS  600u
#define CLEAR_PRESS_MS 2000u
#define DOUBLE_TAP_MS  400u

static bool     btn_down     = false;
static bool     btn_cleared  = false;
static uint32_t btn_t0       = 0;
static uint32_t btn_last_tap = 0;
static int      btn_tap_n    = 0;

/* The effect held on the Launchpad, -1 = none. It wins over B2's punch. */
static int g_lp_punch   = -1;
static int g_punch_sent = -1;

static void set_punch(bool b2)
{
    const int want = g_lp_punch >= 0 ? g_lp_punch : (b2 ? g_punch_fx : -1);
    if (want == g_punch_sent) return;
    g_punch_sent = want;
    G_PUNCHING   = want >= 0;
    /* punch_fx: "-1" releases, "0" punches CLEAN, 1.. forces that effect. */
    set_param_int("punch_fx", want);
}

/* ---- Launchpad Mini MK3 -----------------------------------------------------
 *
 *   rows 1-4   the 27 punch effects in order (CLEAN first): hold a pad to
 *              punch that effect, release to let go. Dim = available,
 *              bright = the one B2 punches (the PUNCH FX knob), green = held.
 *   row 8      the loop's playhead (cyan in LIVE)
 *   top 1      CAPTURE (red while recording, amber armed)
 *   top 2      RE-ROLL          top 3  LIVE on/off          top 4  CLEAR (hold 1 s)
 *
 * USB port (Settings P5): Mac or Launchpad, from the next power-up. In
 * Launchpad mode B2 shows the host: blue starting, cyan no device, yellow
 * enumerating (or working behind a hub), red gave up, green running. Hold
 * B1 at power-up to boot in Mac mode whatever the setting says.
 */
static const char* const kUsbLabels[2] = {"Mac", "Launchpad"};
static SelectorHandle usb_port;
static bool     g_lp_mode    = false;
static uint8_t  g_lp_stage   = 0;
static uint32_t g_lp_boot_ms = 0;
static bool     g_lp_clear_down = false;
static bool     g_lp_clear_fired = false;
static uint32_t g_lp_clear_t0 = 0;

static void lp_poll(uint32_t now)
{
    lp::Poll(now);
    lp::Event e;
    const bool live = !settings.IsActive();
    while (lp::PopEvent(&e))
    {
        if (!live) continue;
        if (e.kind == lp::Kind::Grid && e.y < 4)
        {
            const int fx = e.y * 8 + e.x;
            if (fx >= 27) continue;
            if (e.down) g_lp_punch = fx;
            else if (g_lp_punch == fx) g_lp_punch = -1;
        }
        else if (e.kind == lp::Kind::Top)
        {
            if (e.x == 0 && e.down)
            {
                smack_set_param(S, "capture", "1");
                if (!clk_locked(&CLK)) smack_set_param(S, "detect_bpm", "1");
            }
            else if (e.x == 1 && e.down) smack_set_param(S, "reroll", "1");
            else if (e.x == 2 && e.down) G_LIVE = !G_LIVE;
            else if (e.x == 3)
            {
                if (e.down) { g_lp_clear_down = true; g_lp_clear_fired = false; g_lp_clear_t0 = now; }
                else g_lp_clear_down = false;
            }
        }
    }
    if (g_lp_clear_down && !g_lp_clear_fired && now - g_lp_clear_t0 > 1000u)
    {
        smack_set_param(S, "clear", "1");
        g_lp_clear_fired = true;
    }
}

static void lp_paint(void)
{
    if (!lp::Connected()) return;
    static const uint8_t kRowDim[4]    = {lp::kOrangeDim, lp::kAmberDim, lp::kCyanDim, lp::kMagentaDim};
    static const uint8_t kRowBright[4] = {lp::kOrange, lp::kAmber, lp::kCyan, lp::kMagenta};
    for (int fx = 0; fx < 32; fx++)
    {
        const uint8_t x = (uint8_t)(fx % 8), y = (uint8_t)(fx / 8);
        uint8_t c = lp::kOff;
        if (fx < 27)
        {
            c = (fx == 0) ? lp::kGrey : kRowDim[y];
            if (fx == g_punch_fx) c = (fx == 0) ? lp::kWhite : kRowBright[y];
            if (fx == g_lp_punch) c = lp::kGreen;
        }
        lp::SetGrid(x, y, c);
    }
    const bool looping = G_RUN_STATE == 3;
    const int  head    = looping ? (int)(G_PLAYPOS * 8.0f) : -1;
    for (uint8_t x = 0; x < 8; x++)
        lp::SetGrid(x, 7, (int)x == head ? (G_LIVE ? lp::kCyan : lp::kGreen) : lp::kOff);
    lp::SetTop(0, G_RUN_STATE == 2 ? lp::kRed : (G_RUN_STATE == 1 ? lp::kAmber : lp::kRedDim));
    lp::SetTop(1, lp::kAmberDim);
    lp::SetTop(2, G_LIVE ? lp::kCyan : lp::kCyanDim);
    lp::SetTop(3, g_lp_clear_down ? lp::kRed : lp::kRedDim);
    lp::SetLogo(lp::kGreen);
}

/* Host report to /lpdiag.txt once, 17 s after boot, unless a Launchpad came up. */
alignas(32) static ALCHEMY_SDMMC_BSS FIL  s_lpdiag_fil;
alignas(32) static ALCHEMY_SDMMC_BSS char s_lpdiag_buf[4096];
static bool g_lpdiag_done = false;

static void lp_write_report(uint32_t now)
{
    if (g_lpdiag_done || g_lp_stage < 2 || now - g_lp_boot_ms < 17000u) return;
    if (lp::Connected()) { g_lpdiag_done = true; return; }
    if (picker::Busy() || !sd.EnsureMounted(now)) return;
    g_lpdiag_done = true;
    const int n = lp::Report(s_lpdiag_buf, (int)sizeof s_lpdiag_buf);
    if (f_open(&s_lpdiag_fil, "/lpdiag.txt", FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return;
    UINT w = 0;
    f_write(&s_lpdiag_fil, s_lpdiag_buf, (UINT)n, &w);
    f_close(&s_lpdiag_fil);
}

static void OnPoll(uint32_t now)
{
    if (g_lp_mode)
    {
        if (g_lp_stage == 0 && now - g_lp_boot_ms > 2000u) g_lp_stage = 1;
        else if (g_lp_stage == 1 && now - g_lp_boot_ms > 2300u)
        {
            lp::Init();
            g_lp_stage = 2;
        }
        else if (g_lp_stage >= 2)
        {
            lp_poll(now);
            g_lp_stage = (uint8_t)(2 + lp::Stage());
        }
    }

    if (settings.IsActive())
    {
        g_lp_punch = -1;
        set_punch(false);
        btn_down    = false;
        btn_cleared = false;
        btn_tap_n   = 0;
        return;
    }

    set_punch(hw.buttons[kButtonB2].Pressed());

    const bool pressed = hw.buttons[kButtonB1].Pressed();
    if (pressed && !btn_down)
    {
        btn_down    = true;
        btn_cleared = false;
        btn_t0      = now;
    }

    const uint32_t held = now - btn_t0;

    /* Clear fires while held, so passing 2 s cancels the capture that the
     * release would otherwise trigger. */
    if (btn_down && !btn_cleared && pressed && held > CLEAR_PRESS_MS)
    {
        smack_set_param(S, "clear", "1");
        btn_cleared = true;
    }

    if (btn_down && !pressed)
    {
        btn_down = false;
        if (btn_cleared) return;

        if (held > LONG_PRESS_MS)
        {
            smack_set_param(S, "capture", "1");
            /* With nothing patched to the clock, the engine can autocorrelate
             * the audio you just played for a tempo. Incremental, on the audio
             * thread; the result is picked up in OnFrame. */
            if (!clk_locked(&CLK)) smack_set_param(S, "detect_bpm", "1");
            return;
        }

        /* The first tap re-rolls immediately; a second within DOUBLE_TAP_MS
         * toggles LIVE. Re-roll must not wait to find out whether a second
         * tap is coming -- it is the gesture you use constantly. */
        btn_tap_n    = (now - btn_last_tap < DOUBLE_TAP_MS) ? btn_tap_n + 1 : 1;
        btn_last_tap = now;
        if (btn_tap_n == 2)
        {
            G_LIVE = !G_LIVE;
            return;
        }
        smack_set_param(S, "reroll", "1");
    }
}

/* ---- control frame (~60 Hz) ------------------------------------------------ */

static bool     g_dirty       = false;
static uint32_t g_dirty_since = 0;
static float    g_saved_peak  = 0.0f;
static bool     prev_locked   = false;
static bool     prev_settings = false;
static int      applied_clock_mode = -1;
static uint32_t last_lock_write = 0;

/* When the working state first became unsaved. A clock-locked tempo that
 * wanders by half a BPM re-marks dirty every frame or so, which would push
 * the 5 s quiet period back forever; this caps the wait at 30 s. */
static uint32_t g_dirty_first = 0;

static void mark_dirty(uint32_t now)
{
    if (!g_dirty) g_dirty_first = now;
    g_dirty       = true;
    g_dirty_since = now;
}

static float bpm_to_norm(float bpm) { return (bpm - 50.0f) / 150.0f; }

static void OnFrame(void)
{
    const uint32_t now = System::GetNow();
    int            v;

    /* The PLAY page is worth a flash write too. It used to follow the pots
     * at every boot, so saving it bought nothing; now that a saved PLAY page
     * is restored (the pots catch it), a change there must reach the slot or
     * it is lost at the next firmware switch or power cycle. The stored
     * values move only when a caught pot moves, and the 1 % step keeps ADC
     * noise from re-arming the autosave. The first frame only primes. */
    {
        static float play_seen[kNumPots];
        static bool  primed = false;
        for (uint8_t p = 0; p < kNumPots; p++)
        {
            const float s = pager.Stored(kPagePlay, p);
            if (!primed) { play_seen[p] = s; continue; }
            if (fabsf(s - play_seen[p]) > 0.01f)
            {
                play_seen[p] = s;
                mark_dirty(now);
            }
        }
        primed = true;
    }

    /* PLAY page. Values are catch + locks + CV, already mixed by the SDK. */
    dispatch(k_fx,    fx.Norm());
    dispatch(k_order, order.Norm());
    dispatch(k_len,   length.Norm());
    dispatch(k_slice, slice.Norm());
    G_BLEND = blend.Norm();
    update_dj_ctl(djf.Norm());

    /* SETUP page. These are worth a flash write when they move. */
    if (dispatch(k_seed,  seed.Norm()))  mark_dirty(now);
    if (dispatch(k_pitch, pitch.Norm())) mark_dirty(now);
    if (dispatch(k_mode,  mode.Norm()))  mark_dirty(now);

    if (knob_changed(k_ratio, ratio.Norm(), &v))
    {
        static const int ticks[3] = {CLK_TICKS_DIV2, CLK_TICKS_1X, CLK_TICKS_2X};
        clk_set_ratio(&CLK, ticks[v]);
        mark_dirty(now);
    }
    if (knob_changed(k_punch, punch.Norm(), &v))
    {
        g_punch_fx = v;
        if (G_PUNCHING && g_lp_punch < 0)
        {
            g_punch_sent = v;
            set_param_int("punch_fx", v);
        }
        mark_dirty(now);
    }

    /* TEMPO: the free-run tempo, honoured only while nothing is locked. */
    {
        const float bpm = tempo.Value();
        if (!clk_locked(&CLK) && fabsf(bpm - extras.free_bpm) >= 0.5f)
        {
            extras.free_bpm = bpm;
            clk_set_free_bpm(&CLK, bpm);
            mark_dirty(now);
        }
    }

    /* Clock mode from Settings: AUTO works the jack out, CLOCK always trusts
     * it (for a deliberately uneven clock). */
    {
        const int cm = (int)clock_mode.Value();
        if (cm != applied_clock_mode)
        {
            applied_clock_mode = cm;
            clk_set_mode(&CLK, cm ? CLK_EXTERNAL : CLK_AUTO);
        }
    }

    /* Engine readbacks. smack_get_param is an snprintf -- fine at frame rate,
     * never in the callback. */
    char buf[32];
    int  run = 0;
    if (smack_get_param(S, "run_state", buf, sizeof buf) >= 0) run = atoi(buf);
    G_RUN_STATE = run;

    if (smack_get_param(S, "play_frame", buf, sizeof buf) >= 0)
    {
        int  pf = atoi(buf), lf = 0;
        char b2[32];
        if (smack_get_param(S, "loop_frames", b2, sizeof b2) >= 0) lf = atoi(b2);
        G_PLAYPOS = (lf > 0) ? (float)pf / (float)lf : 0.0f;

        /* LIVE: re-capture each time the playhead wraps (play frame going
         * backwards), so the window slides onto fresh audio every pass.
         * Safe to fire repeatedly: capture is grid-aligned and chases phase. */
        static int last_pf = 0;
        if (G_LIVE && run == 3 && lf > 0 && pf < last_pf)
            smack_set_param(S, "capture", "1");
        last_pf = pf;
    }

    /* Tempo bookkeeping. Locked: the jack's tempo becomes the free-run
     * default and the TEMPO pot adopts it (catch, so the pot picks up from
     * there rather than jumping). Unlocked: a finished BPM scan does the
     * same. "-1" and "0" from the scan are not tempos. */
    const bool locked = clk_locked(&CLK);
    if (locked)
    {
        const float b = clk_bpm(&CLK);
        if (b > 20.0f && b < 300.0f)
        {
            if (fabsf(b - extras.free_bpm) >= 0.5f)
            {
                extras.free_bpm = b;
                clk_set_free_bpm(&CLK, b);
                mark_dirty(now);
            }
            if (!prev_locked || now - last_lock_write > 1000u)
            {
                pager.SetStored(kPageSetup, kPotBottomRight, bpm_to_norm(b), loop.Phys());
                last_lock_write = now;
            }
        }
    }
    else if (smack_get_param(S, "detected_bpm", buf, sizeof buf) >= 0)
    {
        const float d = (float)atof(buf);
        if (d >= 50.0f && d <= 200.0f && fabsf(d - extras.free_bpm) >= 0.5f)
        {
            clk_set_free_bpm(&CLK, d);
            extras.free_bpm = d;
            pager.SetStored(kPageSetup, kPotBottomRight, bpm_to_norm(d), loop.Phys());
            mark_dirty(now);
        }
    }
    prev_locked = locked;

    /* Worst block this session. Clamped: an overrunning callback can report
     * over 100%, and the readout wants 0..1. */
    {
        float mx = cpu.GetMaxCpuLoad();
        if (mx > 1.0f) mx = 1.0f;
        if (mx > extras.cpu_peak) extras.cpu_peak = mx;
        if (extras.cpu_peak > g_saved_peak + 0.05f) mark_dirty(now);
    }

    /* A visit to Settings is worth saving (brightness, clock mode). */
    const bool sact = settings.IsActive();
    if (prev_settings && !sact) mark_dirty(now);
    prev_settings = sact;

    /*
     * Autosave slot 0, the working state, a few seconds after the last
     * change and only with hands off: Save() erases the sectors the record
     * covers (one, for this payload) from this thread, and the 1 ms button
     * poll is paused for the duration. The epsilons above are the wear
     * limiter; this is a ceiling on write frequency, not a write rate.
     */
    if (g_lp_mode) { lp_paint(); lp_write_report(now); }

    /* The USB port only takes effect at power-up, so a change is saved as
     * soon as Settings closes -- nobody should have to wait before cycling. */
    /* A USB port change goes to the card for every firmware, as soon as
     * Settings closes, independently of this firmware's autosave. */
    static int shared_written = -2;
    if (shared_written == -2) shared_written = (int)usb_port.Value();
    if (!sact && (int)usb_port.Value() != shared_written && !picker::Busy()
        && usbshared::Save(sd, (int)usb_port.Value(), now))
        shared_written = (int)usb_port.Value();

    static int saved_usb = -1;
    if (saved_usb < 0) saved_usb = (int)usb_port.Value();
    const bool usb_changed = !sact && (int)usb_port.Value() != saved_usb;

    if (g_dirty
        && (now - g_dirty_since >= 5000u || now - g_dirty_first >= 30000u || usb_changed)
        && !sact && !picker::Busy()
        && !hw.buttons[kButtonB1].Pressed() && !hw.buttons[kButtonB2].Pressed()
        && !hw.buttons[kButtonB3].Pressed())
    {
        presets.Save(kHomeSlot);
        saved_usb    = (int)usb_port.Value();
        g_dirty      = false;
        g_saved_peak = extras.cpu_peak;
    }
}

/* ---- LEDs ------------------------------------------------------------------ */

/*
 * The SDK draws the rings from the knobs. What is added:
 *   B1 pair   STATE  dim blue idle, amber armed, red recording, green
 *                    looping, cyan LIVE, white while a punch is held
 *   B2 pair   CLOCK  blue external, purple inferred, white free-running,
 *                    red when the audio callback is above 80%
 *   B3 pair   dim Setup tint (the SDK paints it while the page is held)
 *   LENGTH ring      a playhead pip, once around per loop pass
 *   P1 ring at boot  the previous session's worst CPU load, for 2.5 s
 */
static uint32_t g_readout_until = 0;
static float    g_boot_peak     = 0.0f;

static void paint_fill(uint8_t pot, float frac, const LedPanel::Rgb& c)
{
    const ArcGeometry& geo = hw.Arc();
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    const int n = (int)lroundf(frac * (float)geo.arc_leds);
    for (int i = 0; i < n; i++)
        hw.leds.SetRingByHour(pot, fmodf(geo.start_hour + geo.step_hours * (float)i, 12.0f),
                              hw.leds.ScaleGlobal(c));
}

static void PlayheadOverdraw(LedPanel& panel, uint8_t pot, const ArcGeometry& geo,
                             float norm, uint32_t t_ms, void* ctx)
{
    (void)norm; (void)t_ms; (void)ctx;
    if (G_RUN_STATE != 3) return;
    const float hour = geo.start_hour + geo.step_hours * G_PLAYPOS * (float)(geo.arc_leds - 1);
    panel.SetRingByHour(pot, fmodf(hour, 12.0f), panel.ScaleGlobal(kWhite));
}

static void OnRender(uint32_t t_ms)
{
    LedPanel& L = hw.leds;

    if (t_ms < g_readout_until)
    {
        L.ClearRing(kPotTopLeft);
        if (g_boot_peak <= 0.0f)
        {
            /* No data: first boot after a flash. Distinct from "measured,
             * and low" so the two never get confused. */
            L.SetRingByHour(kPotTopLeft, hw.Arc().start_hour, L.ScaleGlobal(kIdle));
        }
        else
        {
            const LedPanel::Rgb c = g_boot_peak >= 0.90f ? kRed
                                  : g_boot_peak >= 0.75f ? kAmber
                                                         : kGreen;
            paint_fill(kPotTopLeft, g_boot_peak, c);
        }
    }

    if (settings.IsActive()) return;   /* Settings owns the buttons */

    LedPanel::Rgb st;
    switch (G_RUN_STATE)
    {
        case 1:  st = kAmber; break;                  /* armed     */
        case 2:  st = kRed;   break;                  /* recording */
        case 3:  st = G_LIVE ? kCyan : kGreen; break; /* looping   */
        default: st = kIdle;  break;
    }
    if (G_PUNCHING) st = kWhite;
    L.SetButtonPair(kButtonB1, L.ScaleGlobal(st));

    LedPanel::Rgb ck;
    if (cpu.GetAvgCpuLoad() > 0.80f) ck = kRed;
    else if (!clk_locked(&CLK))      ck = kGrey;
    else if (CLK.mode == CLK_INFER)  ck = kPurple;
    else                             ck = kBlue;
    if (g_lp_mode)
    {
        static const LedPanel::Rgb kStage[6] = {
            {0x40, 0x00, 0x40}, {0x00, 0x00, 0xFF}, {0x00, 0xC0, 0xC0},
            {0xFF, 0xC0, 0x00}, {0xFF, 0x00, 0x00}, {0x00, 0xFF, 0x00}};
        if (g_lp_stage < 5) ck = kStage[g_lp_stage];   /* running: B2 is B2 again */
    }
    L.SetButtonPair(kButtonB2, L.ScaleGlobal(ck));

    if (pager.Page() != kPageSetup)
        L.SetButtonPair(kButtonB3, L.ScaleGlobal(kColSetup));
}

/* ---- audio ----------------------------------------------------------------- */

static void emit_to_engine(void* ctx, uint8_t byte)
{
    smack_on_midi((smack_t*)ctx, &byte, 1, 3); /* source 3 = host, as on Move */
}

static int16_t     bufi[kBlockSize * 2];
static float       buff[kBlockSize * 2];
static dj_filter_t djf_state;

static inline int16_t f2i(float v)
{
    if (v > 0.999969f) v = 0.999969f;
    if (v < -1.0f)     v = -1.0f;
    return (int16_t)(v * 32767.0f);
}

static void AudioCallback(daisy::AudioHandle::InputBuffer  in,
                          daisy::AudioHandle::OutputBuffer out,
                          size_t                           size)
{
    cpu.OnBlockStart();

    /* Controls are NOT read here; the engine does real work inside
     * set_param (a seed change rebuilds the whole pattern). What stays is
     * what is genuinely per-block: the clock edge and the clock advance need
     * frame accuracy, and the conversion and process calls are the audio. */
    clock_poll_isr();
    clk_advance(&CLK, (int)size, emit_to_engine, S);

    /* Refuse to run rather than corrupt memory if the block is ever not what
     * Init() asked for. */
    if (size > kBlockSize)
    {
        for (size_t i = 0; i < size; i++)
        {
            out[0][i] = in[0][i];
            out[1][i] = in[1][i];
        }
        cpu.OnBlockEnd();
        return;
    }

    /* float -1..1 -> interleaved int16, which is what the engine takes.
     * Converting at the boundary keeps the engine bit-identical to the Move
     * and Versio builds, so any difference in sound is a shim bug. */
    for (size_t i = 0; i < size; i++)
    {
        bufi[2 * i]     = f2i(in[0][i]);
        bufi[2 * i + 1] = f2i(in[1][i]);
    }

    smack_process(S, bufi, bufi, (int)size);

    /* BLEND: dry input against the effected loop, here because this is the
     * only point where both still exist separately. With no loop captured
     * the engine returns silence, so until there is something to blend WITH
     * the input passes through untouched. */
    float       wet = (G_RUN_STATE == 3) ? G_BLEND : 0.0f;
    const float dry = 1.0f - wet;
    const float k   = wet * (1.0f / 32768.0f);
    for (size_t i = 0; i < size; i++)
    {
        buff[2 * i]     = in[0][i] * dry + (float)bufi[2 * i] * k;
        buff[2 * i + 1] = in[1][i] * dry + (float)bufi[2 * i + 1] * k;
    }

    /* DJ filter, last, on everything the module puts out -- the one control
     * that does something before a loop exists. The notch is a real bypass:
     * at exactly 0 the block is skipped, because even "open" the biquads
     * shave ~0.5 dB off 8 kHz (test_dj_filter), and the integrators are
     * cleared on the way out so re-engaging starts from silence, as the
     * Versio's role switch did. */
    {
        static bool dj_on = false;
        const float ctl   = G_DJ_CTL;
        if (ctl != 0.0f)
        {
            dj_on = true;
            dj_filter_block(&djf_state, buff, (int)(2 * size), ctl);
        }
        else if (dj_on)
        {
            dj_on = false;
            dj_filter_reset(&djf_state);
        }
    }

    for (size_t i = 0; i < size; i++)
    {
        out[0][i] = buff[2 * i];
        out[1][i] = buff[2 * i + 1];
    }

    cpu.OnBlockEnd();
}

/* ---- boot ------------------------------------------------------------------ */

static void fault_forever(void)
{
    /* Refuse to run half-initialised: every button pair red, no audio. */
    for (;;)
    {
        hw.leds.Clear();
        for (uint8_t b = 0; b < kNumButtons; b++)
            hw.leds.SetButtonPair(b, {0x80, 0x00, 0x00});
        hw.leds.Show();
        System::Delay(200);
    }
}

int main(void)
{
    hw.Init(daisy::SaiHandle::Config::SampleRate::SAI_48KHZ, kBlockSize);
    cpu.Init(hw.SampleRate(), (int)hw.BlockSize());
    versio_alloc_init(g_pool, VERSIO_POOL_BYTES);
    clock_calibrate();
    dj_filter_reset(&djf_state);

    memset(&HOST, 0, sizeof HOST);
    HOST.api_version      = 1;
    HOST.sample_rate      = SMACK_SR;
    HOST.frames_per_block = kBlockSize;
    HOST.get_bpm          = host_get_bpm;
    clk_init(&CLK, SMACK_SR, 120.0f);

    S = smack_create(&HOST);
    if (!S || versio_alloc_failed()) fault_forever();

    /* monitor = 0 so the engine returns the LOOP ONLY and the dry/wet
     * crossfade happens in the callback against the actual input -- BLEND
     * has to mean dry-versus-effected on a Eurorack insert. */
    smack_set_param(S, "monitor",  "0");
    smack_set_param(S, "hw_input", "1");
    smack_set_param(S, "wet",      "100");

    /* Storage + settings. sd.Init() registers the SDRAM volume; no card I/O
     * until something opens it. */
    sd.Init();
    picker::Install(settings, kSettingsFirmware, sd, hw);
    settings.UseBrightness();
    settings.UsePresets(presets);
    usb_port = settings.Page(kSettingsMain).Pot(5)
        .Selector(kUsbLabels).Default(0)
        .Ident("usb").Name("USB port")
        .Help("**Mac**: the front USB-C is HostLink, for the web programmer, "
              "presets and the card. **Launchpad**: the Lab is the USB host "
              "for a Launchpad Mini MK3, direct with 5 V injected or through "
              "a powered hub adapter. From the next power-up; hold B1 while "
              "powering up to get Mac mode back.");
    clock_mode = settings.Page(kSettingsMain).Pot(4)
        .Selector(kClockModes).Default(0)
        .Ident("clock.mode").Name("Clock In")
        .Help("**Auto** reads a steady train on J3 as a clock and sporadic "
              "hits as capture triggers, inferring the tempo from their "
              "spacing. **Clock** always treats J3 as a clock, for a "
              "deliberately uneven one.");

    /* CV: J3 is the clock (read raw in the callback), the rest modulate. */
    cv_matrix.Jack(0).Off();
    cv_matrix.Jack(1).To(fx);
    cv_matrix.Jack(2).To(order);
    cv_matrix.Jack(3).To(blend);
    cv_matrix.Jack(4).To(slice);
    cv_matrix.Jack(5).To(seed);

    length.Overdraw(PlayheadOverdraw);

    /* HostLink: identity, descriptor, SD file access. The transport is the
     * front USB-C unless this is a bench build. Must all be declared before
     * BootLoad(), which is where the host starts. */
    host.Product("Smack")
        .BootSlot(kHomeSlot)
        .Pages(play_page, setup_page)
        .Jacks(jk_in_l, jk_in_r, jk_clk, jk_cv_fx, jk_cv_ord, jk_cv_bl,
               jk_cv_sl, jk_cv_sd, jk_out_l, jk_out_r)
        .Buttons(bt_capture, bt_punch, bt_setup)
        .Attach(manual)
        .Extend(fs_ext);
#ifdef SMACK_BENCH_USB
    bench_cdc.Init(bench_usb, daisy::UsbHandle::FS_INTERNAL, "Smack (bench)");
    host.Transport(bench_cdc);
#endif

    /* Preset payload: both pages, the settings screen, and the two extras. */
    presets.Manage(pager);
    presets.Manage(settings);
    presets.Manage(extras);
    presets.Init();
    /* BootLoad() still runs: it fires HostLink's pre-boot hook, and it
     * restores slot 0 when slot 0 holds THIS firmware's settings (the
     * schema gate refuses any other), which carries a module's settings
     * over from the builds that autosaved there. The home slot, once it
     * holds anything, wins. */
    const bool had_home = presets.HasValid(kHomeSlot);
    const bool had_boot = had_home || presets.HasValid(0);
    presets.BootLoad();
    if (had_home) presets.Load(kHomeSlot);

    /* Physical pot positions, primed. */
    float phys[kNumPots];
    for (int i = 0; i < 8; i++)
    {
        hw.ProcessAllControls();
        System::Delay(1);
    }
    for (uint8_t p = 0; p < kNumPots; p++) phys[p] = hw.pots[p].Value();

    if (!had_boot)
    {
        /* First boot after a flash: sensible Setup defaults. */
        pager.SetStored(kPageSetup, kPotTopLeft,     0.0f,           phys); /* seed 0      */
        pager.SetStored(kPageSetup, kPotTopRight,    11.0f / 23.0f,  phys); /* 12 semis    */
        pager.SetStored(kPageSetup, kPotMiddleLeft,  1.5f / 3.0f,    phys); /* ratio =1    */
        pager.SetStored(kPageSetup, kPotMiddleRight, 1.5f / 27.0f,   phys); /* punch RETRIG*/
        pager.SetStored(kPageSetup, kPotBottomLeft,  0.25f,          phys); /* stereo      */
        pager.SetStored(kPageSetup, kPotBottomRight, bpm_to_norm(120.0f), phys);
    }

    /* The PLAY page. With nothing saved (first boot after a flash) it
     * adopts the pots, so whatever a pot points at is what the module
     * does. With a saved state it keeps the saved values, and each pot
     * must catch its value before it takes over: the pots are shared by
     * every firmware on the card, so after a picker switch they point
     * wherever the last firmware left them, and adopting them would
     * overwrite this one's settings with another's. BootLoad() left the
     * Pager a deferred re-arm for its first Update(), which arms the
     * catch against phys without touching the stored values. */
    if (!had_boot)
        for (uint8_t p = 0; p < kNumPots; p++)
            pager.SetStored(kPagePlay, p, phys[p], phys);

    clk_set_free_bpm(&CLK, extras.free_bpm);

    /* Report last session's peak on the P1 ring, then start recording this
     * one. Audio is already passing through during the readout. */
    g_boot_peak      = extras.cpu_peak;
    extras.cpu_peak  = 0.0f;
    g_saved_peak     = 0.0f;
    g_readout_until  = System::GetNow() + 2500u;

    /* The front port's role is one setting for every firmware on the card:
     * adopt the card's value, so a picker switch keeps Launchpad mode. */
    const int shared_usb = usbshared::Load(sd, 500);
    if (shared_usb >= 0 && shared_usb != (int)usb_port.Value())
        usb_port.Default((uint8_t)shared_usb);

    /* B1 held through power-up: Mac mode this boot. Read over ~20 ms so
     * the debouncer has settled whatever came before. */
    bool force_mac = false;
    for (int i = 0; i < 20; i++)
    {
        hw.ProcessAllControls();
        System::Delay(1);
    }
    force_mac = hw.buttons[kButtonB1].Pressed();
    g_lp_mode    = (int)usb_port.Value() == 1 && !force_mac;
    g_lp_boot_ms = System::GetNow();

    hw.StartAudio(AudioCallback);
    cpu.Reset();

    loop.Use(pager)
        .Use(settings)
        .Use(cv_matrix)
        .Use(play_page)
        .Use(setup_page)
        .OnFrame(OnFrame)
        .OnPoll(OnPoll)
        .OnRender(OnRender);
    if (!g_lp_mode) loop.Use(host);

    for (;;) loop.Tick();
}
