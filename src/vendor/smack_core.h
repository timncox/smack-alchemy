/*
 * Smack — quantized live-loop capture + probabilistic per-slice glitch FX.
 *
 * Shared engine used by both builds:
 *   - smack_fx.c   audio_fx_api_v2 wrapper (chain slots + Master FX slots)
 *   - smack_gen.c  plugin_api_v2 wrapper (sound_generator reading hardware input)
 *
 * Timing model: 4/4, 16th-note steps, MIDI clock at 24 ppqn (6 ticks/step,
 * 96 ticks/bar). Falls back to host get_bpm() free-run when no clock runs.
 * The render path is non-allocating and non-blocking per schwung's realtime
 * rules; all buffers are allocated in smack_create().
 */
#ifndef SMACK_CORE_H
#define SMACK_CORE_H

#include <stdint.h>
#include "plugin_api_v1.h"

/* ------------------------------------------------------------------------
 * VENDORED from timncox/schwung-smack @ 169905d ("Release Smack 0.15.2"),
 * and SINCE DIVERGED. plugin_api_v1.h is still byte-identical to that SHA.
 * smack_core.c is NOT, and neither is this header.
 *
 * Fixes that exist ONLY in this copy — all three found on Versio hardware:
 *   7e73978  ring recorder stalled behind the playing loop  (+62 lines)
 *   73aa079  LIVE never fired: play_frame was empty on hardware
 *   3b18a70  auto BPM detection; all float formatting removed from the engine
 *
 * So DO NOT re-vendor by overwriting these files — that silently reverts all
 * three. The ring stall is the dangerous one: captures keep succeeding and
 * quietly return pre-stall audio, so nothing looks broken. Land the fixes
 * upstream first (~/tim-os/scratch/smack-ring-stall.patch applies cleanly
 * from the smack root), then re-vendor from a SHA that contains them:
 *   git -C ~/tim-os/smack show <sha>:src/smack_core.c > smack_core.c   (etc.)
 *
 * For NEW bugs the old rule still stands: prefer fixing upstream and
 * re-vendoring, or the two copies drift further apart than they already have.
 * ------------------------------------------------------------------------ */

/* 48000, not 44100: libDaisy offers 8/16/32/48/96 kHz only (sai.h). Verified
 * 2026-08-08 — `make test` green at 48 k with no engine logic change. */
#define SMACK_SR          48000
/*
 * The ring holds TWO maximum loops, not one.
 *
 * It was 70 s -- exactly 16 bars at 55 BPM, i.e. one longest-possible loop.
 * That is half of what is needed. While a loop plays, the recorder must not
 * overwrite it (record_ring_frame's guard), so the space left for incoming
 * audio is RING minus the protected loop. Size the ring for one loop and that
 * remainder runs out mid-pass: recording stops, ring_last_global freezes, and
 * every later Capture returns audio from before the stall. The module keeps
 * looping and keeps accepting Capture, so it looks like it is working.
 *
 * Measured at 70 s: at 120 BPM with a 32 s loop the recorder stalled 38 s
 * after capture and never resumed; a Capture pressed after that returned
 * pre-stall audio. See firmware/test/test_ring_stall.c.
 *
 * 150 s keeps the longest loop (256 steps) usable down to ~51 BPM. Below
 * that, capture steps the length down one musical notch at a time rather
 * than clipping frames -- see fit_loop_len_idx() in smack_core.c.
 */
#define SMACK_MAX_SECONDS 150              /* 2 x 16 bars at 55 BPM */
#define SMACK_RING_FRAMES (SMACK_SR * SMACK_MAX_SECONDS)
#define SMACK_MAX_SLICES  512              /* 16 bars x 16 steps x 2 (half-step res) */
/* 105 not 96: keeps the fade at ~2.18 ms of wall time at 48 k. */
#define SMACK_EDGE_FADE   105              /* ~2.2 ms fade at slice/loop edges */

typedef enum { SMACK_IDLE = 0, SMACK_ARMED, SMACK_RECORDING, SMACK_LOOPING } smack_state_t;

typedef enum {
    SMACK_FX_NONE = 0,
    SMACK_FX_RETRIG,     /* repeat first 1/2..1/8 of slice, decaying */
    SMACK_FX_REVERSE,
    SMACK_FX_PITCH,      /* varispeed +/- semitones (fxp), wraps in slice */
    SMACK_FX_SPEED,      /* fxp 0 = half speed, 1 = double speed */
    SMACK_FX_GATE,       /* 8-segment tremolo chop */
    SMACK_FX_BUZZ,       /* tiny-window repeat frozen at slice head */
    SMACK_FX_CRUSH,      /* sample-hold rate reduce + bit quantize */
    /* Looperator-inspired additions */
    SMACK_FX_REPEAT,     /* play head of slice, then stutter it (fxp: 1/2,1/4,3/4) */
    SMACK_FX_REVAFTER,   /* forward, then backwards from split (fxp: 1/2,2/3,3/4) */
    SMACK_FX_TAPESTOP,   /* varispeed ramp to zero (fxp 0 full-slice, 1 fast) */
    SMACK_FX_TAPESTART,  /* turntable spin-up from zero */
    SMACK_FX_SCRATCH,    /* oscillating playhead, vinyl wobble (fxp cycles) */
    SMACK_FX_ENV,        /* volume shape: fade-in/out, wobble, half-gate */
    SMACK_FX_PAN,        /* hard L / hard R / ping-pong within slice */
    SMACK_FX_FILTER,     /* LP/HP sweep across the slice (fxp direction) */
    SMACK_FX_VOWEL,      /* morphing 3-formant vowel filter (fxp pair) */
    SMACK_FX_TONALDELAY, /* feedback delay with swept time = pitched repeats */
    SMACK_FX_FREEZE,     /* granular freeze on the slice head (fxp spray) */
    SMACK_FX_DELAY,      /* tempo-synced echo (fxp: 16th, 8th, dotted, pingpong) */
    SMACK_FX_DIST,       /* waveshaper dirt (fxp: soft, hard, fold, gnash) */
    SMACK_FX_PHASER,     /* 4-stage allpass sweep (fxp: up, down, wobble, fast) */
    SMACK_FX_VERB,       /* gated Schroeder reverb burst (fxp: size/decay) */
    /* v0.11.0 additions (appended so existing fx codes stay stable; these
     * DO join the roll pool, which reshuffles what old seeds produce) */
    SMACK_FX_PSHIFT,     /* time-preserving granular pitch shift (fxp semis) */
    SMACK_FX_RINGMOD,    /* sine ring modulator (fxp freq / sweep) */
    SMACK_FX_COMB,       /* tuned feedback comb (fxp tuning / sweep / scream) */
    SMACK_FX_SCATTER,    /* hash-shuffled grains within the slice (fxp mode) */
    SMACK_FX_COUNT
} smack_fx_t;

/* The hardware palette is 23 pads (3 rows minus Unlock) — a SELECTION from
 * the effect pool now that SMACK_FX_COUNT exceeds it. */
#define SMACK_PALETTE_SLOTS 23

typedef struct smack smack_t;

smack_t *smack_create(const host_api_v1_t *host);
void     smack_destroy(smack_t *s);

/* Process one block, stereo interleaved int16. in and out may alias. */
void smack_process(smack_t *s, const int16_t *in, int16_t *out, int frames);

void smack_on_midi(smack_t *s, const uint8_t *msg, int len, int source);
void smack_set_param(smack_t *s, const char *key, const char *val);
int  smack_get_param(smack_t *s, const char *key, char *buf, int buf_len);

#endif /* SMACK_CORE_H */
