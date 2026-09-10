/*
 * Smack core engine. See smack_core.h for the model.
 *
 * v1 simplifications (each is a documented follow-up, not an accident):
 *   - Ring recording pauses while LOOPING (no incremental copy-out yet), so
 *     a re-grab while looping re-slices the frozen pre-loop history.
 *   - Arm/record starts on a block boundary after the quantum tick, not
 *     sample-accurately inside the block (<= 2.9 ms early/late).
 *   - Retro grabs align to the last half-step boundary and chase the elapsed
 *     grid phase when playback begins (block-accurate, not sample-stamped).
 *   - Varispeed reads use linear interpolation; no anti-alias filter.
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "smack_core.h"

/* Loop length menu, in half-steps (2 hs = 1 step, 32 hs = 1 bar in 4/4). */
static const int loop_len_hs_table[] = { 2, 4, 8, 16, 32, 64, 128, 256, 512 };
#define LOOP_LEN_COUNT 9
/* Slice resolution menu, in half-steps. */
static const int slice_hs_table[] = { 1, 2, 4, 8 };
#define SLICE_RES_COUNT 4

/* Pad-play repeat rates, in half-steps (8 hs = 1 beat in 4/4). */
static const int pad_rate_hs[] = { 2, 4, 8, 16, 32 }; /* 1/16 1/8 1/4 1/2 bar */
#define PAD_RATE_COUNT 5

/* A single MIDI-clock callback is quantized to the host's 128-frame render
 * boundary. Regressing across a full bar removes that callback jitter instead
 * of baking the most recent 896/1024-frame interval into every loop length. */
#define CLOCK_EST_WINDOW 96

/* Retrig decay per repeat (0.85^k), k clamped to 15. */
static const float retrig_decay[16] = {
    1.0000f, 0.8500f, 0.7225f, 0.6141f, 0.5220f, 0.4437f, 0.3771f, 0.3206f,
    0.2725f, 0.2316f, 0.1969f, 0.1673f, 0.1422f, 0.1209f, 0.1028f, 0.0874f
};

/* RBJ biquad for the FILTER sweep effect */
typedef struct {
    float b0, b1, b2, a1, a2, z1, z2;
} smack_bq_t;

static void bq_reset(smack_bq_t *q) { q->z1 = q->z2 = 0.0f; }

static inline float bq_run(smack_bq_t *q, float x) {
    float y = q->b0 * x + q->z1;
    q->z1 = q->b1 * x - q->a1 * y + q->z2;
    q->z2 = q->b2 * x - q->a2 * y;
    return y;
}

static void bq_set_q(smack_bq_t *q, float freq, int highpass, float Q) {
    float w0 = 6.2831853f * freq / (float)SMACK_SR;
    float cw = cosf(w0), sw = sinf(w0);
    float alpha = sw / (2.0f * Q);
    float inv = 1.0f / (1.0f + alpha);
    if (highpass) {
        q->b0 = (1.0f + cw) * 0.5f * inv;
        q->b1 = -(1.0f + cw) * inv;
        q->b2 = q->b0;
    } else {
        q->b0 = (1.0f - cw) * 0.5f * inv;
        q->b1 = (1.0f - cw) * inv;
        q->b2 = q->b0;
    }
    q->a1 = -2.0f * cw * inv;
    q->a2 = (1.0f - alpha) * inv;
}

/* constant-peak bandpass, used by the vowel filter's formant bands */
static void bq_set_bandpass(smack_bq_t *q, float freq, float Q) {
    float w0 = 6.2831853f * freq / (float)SMACK_SR;
    float cw = cosf(w0), sw = sinf(w0);
    float alpha = sw / (2.0f * Q);
    float inv = 1.0f / (1.0f + alpha);
    q->b0 = alpha * inv;
    q->b1 = 0.0f;
    q->b2 = -alpha * inv;
    q->a1 = -2.0f * cw * inv;
    q->a2 = (1.0f - alpha) * inv;
}

/* Formant frequencies F1-F3 for A E I O U, and the morph pairs the VOWEL
 * effect sweeps across a slice (Looperator-style vowel transitions). */
static const float vowel_f[5][3] = {
    { 730.0f, 1090.0f, 2440.0f },  /* A */
    { 530.0f, 1840.0f, 2480.0f },  /* E */
    { 390.0f, 1990.0f, 2550.0f },  /* I */
    { 570.0f,  840.0f, 2410.0f },  /* O */
    { 440.0f, 1020.0f, 2240.0f },  /* U */
};
static const int vowel_pair[8][2] = {
    {0, 2}, {3, 2}, {0, 4}, {1, 3},   /* A>I O>I A>U E>O (rolled range) */
    {2, 0}, {1, 4}, {4, 3}, {0, 1},   /* I>A E>U U>O A>E (editor-only)  */
};
static const float vowel_gain[3] = { 1.0f, 0.7f, 0.45f };

#define SMACK_DLY_LEN 8192  /* tonal-delay line, frames */

/* VERB: comb lengths tuned short for slice-length bursts, plus one allpass */
static const int verb_comb_len[4] = { 673, 713, 766, 813 };
#define VERB_AP_LEN 225
#define VERB_TOTAL (673 + 713 + 766 + 813 + VERB_AP_LEN)

/* BPM detection window: 8 s of ring audio, 512-frame envelope hop */
#define DET_SECONDS 8
#define DET_HOP 512
#define DET_BINS ((DET_SECONDS * SMACK_SR) / DET_HOP)      /* 689 */
#define DET_FRAMES_PER_BLOCK 16384   /* ring frames scanned per audio block */
#define DET_BPM_MIN 60.0
#define DET_BPM_MAX 180.0

/* One playback lane: a slice pattern plus every per-effect runtime that
 * must not be shared between simultaneously rendering voices. Stereo mode
 * uses lane[0] alone; dual-mono mode renders lane[0] from the left input
 * channel and lane[1] from the right, then pans each into the field. */
typedef struct {
    /* FILTER effect runtime */
    smack_bq_t fltL, fltR;
    int flt_slice;               /* output slice the filter is tracking, -1 none */
    int flt_counter;             /* frames until coefficient recompute */

    /* VOWEL effect runtime: 3 formant bands per channel */
    smack_bq_t vowL[3], vowR[3];
    int vow_slice;
    int vow_counter;

    /* TONALDELAY + DELAY runtime: shared feedback delay line */
    float   *dly;                /* SMACK_DLY_LEN * 2 floats, interleaved */
    uint32_t dly_w;
    int      dly_slice;

    /* PHASER runtime: 4 first-order allpass stages per channel */
    float ph_x[2][4], ph_y[2][4];
    int   ph_slice;

    /* VERB runtime: mono Schroeder (4 combs + 1 allpass), gated feed */
    float   *verb;               /* one block, offsets below */
    uint32_t vb_ci[4], vb_ai;

    /* Pattern */
    uint16_t order[SMACK_MAX_SLICES];
    uint8_t  fx[SMACK_MAX_SLICES];
    int8_t   fxp[SMACK_MAX_SLICES];
    int8_t   fxp2[SMACK_MAX_SLICES];  /* per-effect depth 0-100, -1 default */
    int8_t   fxmix[SMACK_MAX_SLICES]; /* per-slice wet 0-100, -1 = full wet */
    uint8_t  locked[SMACK_MAX_SLICES]; /* user-pinned: reroll keeps fx[i] */
} smack_lane_t;

struct smack {
    const host_api_v1_t *host;

    /* Ring buffer — records continuously until its write head approaches
     * the active loop region, including while transport is paused. */
    int16_t *ring;               /* SMACK_RING_FRAMES * 2 samples */
    uint32_t ring_w;             /* write index, frames */
    uint64_t written_total;      /* frames ever written */
    uint64_t ring_last_global;   /* global frame of most recent written frame */

    /* Clock. Global frame counter runs across all blocks processed. */
    uint64_t global_frames;
    double   frames_per_tick;    /* smoothed; 918.75 = 120 BPM */
    uint64_t last_tick_global;   /* global frame of previous 0xF8 */
    uint64_t last_halfstep_global; /* global frame of last half-step boundary */
    uint64_t tick_history[CLOCK_EST_WINDOW];
    uint64_t tick_history_seq[CLOCK_EST_WINDOW]; /* inferred source tick number */
    uint64_t clock_seq;          /* includes ticks lost while punching */
    uint16_t tick_history_count;
    uint16_t tick_history_pos;   /* next write; oldest entry when full */
    uint32_t tick_total;         /* ticks since transport start */
    int      clock_running;
    int      clock_seen;

    /* MIDI CC control */
    uint8_t  cc_last[3];         /* last external CC accepted (dup guard) */
    uint64_t cc_last_frames;     /* global_frames when it was accepted */

    /* Loop */
    smack_state_t state;
    uint32_t loop_start;         /* ring frame index */
    uint32_t loop_len;           /* frames */
    uint32_t loop_history_start; /* earliest retained frame before loop end */
    uint32_t loop_available;     /* valid retained frames ending at loop end */
    double   play_pos;           /* 0 .. loop_len */
    uint32_t rec_remaining;      /* frames left while RECORDING */
    uint32_t rec_length;         /* quantum fixed when recording begins */
    uint32_t rec_start;          /* ring index where recording began */
    uint32_t rec_clock_ticks;    /* grid length fixed when recording begins */
    uint32_t loop_clock_ticks;   /* captured grid length (3 ticks/half-step) */
    double   loop_frames_per_tick; /* source-domain rate at capture */
    int      arm_start_flag;     /* set by tick handler: begin record next block */

    /* Params */
    int   loop_len_idx;          /* index into loop_len_hs_table */
    int   slice_res_idx;         /* index into slice_hs_table */
    float fx_density;            /* 0..1 */
    float order_density;         /* 0..1 */
    int   pitch_range;           /* 1..24 semitones, applied up or down */
    float wet;                   /* loop vs live input while LOOPING */
    int   ab;                    /* 0 = A clean, 1 = B pattern */
    int   ab_pending;            /* -1 none, else target value */
    int   quantize_mode;         /* 0 instant, 1 next slice, 2 next loop start */
    uint32_t seed;
    int   follow_transport;      /* 1 = Move stop pauses the loop (default) */
    int   transport_paused;      /* loop exists but transport is stopped */
    int   monitor;               /* 0 = mute live input at the output (feedback
                                    guard; ring keeps recording, loop playback
                                    stays audible). Never preset-saved. */
    int   hw_input;              /* 1 = this instance reads the hardware input
                                    (gen/oversmack builds); chain UI keys the
                                    feedback guard off this */

    /* BPM detection: onset-strength envelope over the last DET_SECONDS of
     * ring audio, autocorrelated across 60-180 BPM. Runs incrementally on
     * the audio thread (DET_FRAMES_PER_BLOCK ring frames per block) so the
     * render path never stalls. */
    /* Global effect punch: while >= 0, every slice plays this effect
     * (0 = clean) at full blend, original order — a performance override.
     * punch_fxp follows pad pressure. Never preset-saved. */
    int      punch_fx;           /* -1 = off */
    int8_t   punch_fxp;
    int8_t   punch_fxp2;         /* depth for a variant pad's punch, -1 dflt */
    int8_t   punch_mix;          /* mix for a variant pad's punch, -1 = wet */

    int      det_active;         /* scanning in progress */
    uint32_t det_pos;            /* frames consumed so far */
    uint32_t det_total;          /* frames to consume */
    uint32_t det_start;          /* ring index of window start */
    float    det_env[DET_BINS];  /* energy envelope, DET_HOP frames per bin */
    float    det_acc;            /* accumulator for the current bin */
    uint32_t det_acc_n;
    float    det_bpm;            /* last result; 0 = none */
    float    bpm_override;       /* free-run tempo override; 0 = project tempo */
    uint32_t roll_nonce;         /* advanced by reroll; 0 = canonical seed pattern */
    uint32_t edit_rev;           /* bumped on any content edit; gates the
                                    Remote-UI browser's full-state refetch */

    /* Dual-mono: L/R inputs as independent mono lanes, each panned back
     * into the stereo field. 0 = Stereo (lane[0] only), 1 = Dual Mono. */
    int   chan_mode;
    int   pan_l, pan_r;          /* 0 hard left .. 100 hard right */
    float pan_gain[2][2];        /* [lane][out ch], equal-power, precomputed */

    /* Pad-palette layout: position (pad offset 0..22) -> fx code, plus an
     * optional per-pad parameter override ("Buzz — Massive" as a pad).
     * Duplicates and subsets are legal — 23 pads select from the full
     * effect pool. Pure UI preference, but persisted through the engine so
     * oversmack and the web editor share one arrangement and presets
     * restore it. */
    uint8_t palette[SMACK_PALETTE_SLOTS];
    int8_t  palette_fxp[SMACK_PALETTE_SLOTS];
    uint8_t palette_hasp[SMACK_PALETTE_SLOTS];
    int8_t  palette_fxp2[SMACK_PALETTE_SLOTS]; /* -1 = default depth */
    int8_t  palette_mix[SMACK_PALETTE_SLOTS];  /* -1 = full wet */

    /* Pad-play: incoming MIDI notes trigger pattern cells (note % n_slices),
     * quantized to pad_rate boundaries and retriggered there while held;
     * release returns to the still-advancing loop timeline. pad_play gates
     * real MIDI notes only — the pad_note param always works. Live note
     * state is never preset-saved; the pad_play/pad_rate settings are. */
    int      pad_play;
    int      pad_rate_idx;       /* index into pad_rate_hs[] */
    uint8_t  pad_note_stk[10];   /* held notes, most recent last */
    uint8_t  pad_vel_stk[10];
    int      pad_held;
    int      trig_active;        /* a cell is held and has fired */
    int      trig_step;          /* pattern cell being repeated */
    double   trig_pos;           /* frames since the last boundary fire */
    float    trig_gain;          /* velocity gain, sqrt taper */
    int      trig_render;        /* per-frame: this frame renders the cell */

    /* Pattern geometry (shared by both lanes: same loop, same grid) */
    int      n_slices;
    double   slice_frames;
    uint32_t rng;

    smack_lane_t lane[2];
};

/* ------------------------------------------------------------------ */
/*  RNG                                                                */
/* ------------------------------------------------------------------ */

static inline uint32_t xs32(uint32_t *st) {
    uint32_t x = *st;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *st = x;
    return x;
}
static inline float rnd01(uint32_t *st) { return (float)(xs32(st) >> 8) / 16777216.0f; }
static inline int rnd_below(uint32_t *st, int n) { return (int)(xs32(st) % (uint32_t)n); }

/* ------------------------------------------------------------------ */
/*  Clock helpers                                                      */
/* ------------------------------------------------------------------ */

/* A RUNNING clock always wins. A clock that once ran but is now stopped
 * keeps its remembered tempo — UNLESS the user set a bpm_override
 * (detection or knob), which would otherwise be silently ignored for the
 * rest of the session because clock_seen is sticky. */
static int clock_governs(const smack_t *s) {
    return s->clock_seen && (s->clock_running || s->bpm_override <= 0.0f);
}

static double frames_per_tick_now(smack_t *s) {
    if (clock_governs(s)) return s->frames_per_tick;
    float bpm = 120.0f;
    if (s->bpm_override > 0.0f) {
        bpm = s->bpm_override;                      /* detected tempo */
    } else if (s->host && s->host->get_bpm) {
        float b = s->host->get_bpm();
        if (b >= 20.0f && b <= 999.0f) bpm = b;
    }
    return (double)SMACK_SR * 60.0 / ((double)bpm * 24.0);
}

static double frames_per_halfstep(smack_t *s) { return frames_per_tick_now(s) * 3.0; }

/* Add one block-stamped MIDI-clock observation. Pad aftertouch and MIDI clock
 * share Move's input queue; under a dense pressure stream, an occasional F8
 * can be lost. While a global punch is held, recognise intervals that are
 * close to an integer multiple of the already-locked clock and retain that
 * logical tick count. Outside a punch every observation remains one tick so
 * real transport tempo changes are still learned normally. */
static uint32_t clock_estimator_push(smack_t *s, uint64_t frame) {
    uint32_t elapsed_ticks = 1;
    if (s->tick_history_count > 0 && frame > s->last_tick_global) {
        double d = (double)(frame - s->last_tick_global);
        if (s->punch_fx >= 0 && s->frames_per_tick > 0.0) {
            double ratio = d / s->frames_per_tick;
            int inferred = ratio >= 1.5 && ratio <= CLOCK_EST_WINDOW + 0.5
                         ? (int)floor(ratio + 0.5) : 1;
            if (inferred >= 2) {
                double per_tick = d / (double)inferred;
                double block = (s->host && s->host->frames_per_block > 0)
                             ? (double)s->host->frames_per_block : 128.0;
                double tolerance = fmax(s->frames_per_tick * 0.20, block);
                if (fabs(per_tick - s->frames_per_tick) <= tolerance)
                    elapsed_ticks = (uint32_t)inferred;
            }
        }
        double tick_d = d / (double)elapsed_ticks;
        if (tick_d > 100.0 && tick_d < 20000.0 && s->tick_history_count < 12)
            s->frames_per_tick = 0.9 * s->frames_per_tick + 0.1 * tick_d;
    }
    s->last_tick_global = frame;
    s->clock_seq += elapsed_ticks;

    s->tick_history[s->tick_history_pos] = frame;
    s->tick_history_seq[s->tick_history_pos] = s->clock_seq;
    s->tick_history_pos = (uint16_t)((s->tick_history_pos + 1) % CLOCK_EST_WINDOW);
    if (s->tick_history_count < CLOCK_EST_WINDOW) s->tick_history_count++;

    int n = (int)s->tick_history_count;
    if (n < 12) return elapsed_ticks;
    int first = ((int)s->tick_history_pos + CLOCK_EST_WINDOW - n) % CLOCK_EST_WINDOW;
    uint64_t y0 = s->tick_history[first];
    uint64_t x0 = s->tick_history_seq[first];
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (int i = 0; i < n; i++) {
        int slot = (first + i) % CLOCK_EST_WINDOW;
        double x = (double)(s->tick_history_seq[slot] - x0);
        double y = (double)(s->tick_history[slot] - y0);
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    double den = (double)n * sxx - sx * sx;
    if (den > 0.0) {
        double slope = ((double)n * sxy - sx * sy) / den;
        if (slope > 100.0 && slope < 20000.0) s->frames_per_tick = slope;
    }
    return elapsed_ticks;
}

static double loop_playback_increment(const smack_t *s) {
    if (!s->clock_running || s->loop_frames_per_tick <= 0.0 ||
        s->frames_per_tick <= 0.0) return 1.0;
    /* Varispeed the captured buffer by the small amount needed to retain its
     * musical length as the robust clock estimate settles (and across later
     * tempo changes). Linear interpolation already supports fractional heads. */
    double inc = s->loop_frames_per_tick / s->frames_per_tick;
    if (inc < 0.25) inc = 0.25;
    if (inc > 4.0) inc = 4.0;
    return inc;
}

/* Global frame of the most recent half-step boundary. */
static uint64_t last_boundary_global(smack_t *s) {
    if (s->clock_running) return s->last_halfstep_global;
    double fph = frames_per_halfstep(s);
    if (s->clock_seen) {
        /* MIDI Stop freezes incoming phase, not time. Continue the remembered
         * grid from its last real boundary so retro capture still ends near
         * "now" while retaining the clock's tempo and phase anchor. */
        uint64_t anchor = s->last_halfstep_global;
        if (s->global_frames <= anchor) return anchor;
        double elapsed = (double)(s->global_frames - anchor);
        return anchor + (uint64_t)(floor(elapsed / fph) * fph);
    }
    return (uint64_t)(floor((double)s->global_frames / fph) * fph);
}

/* ------------------------------------------------------------------ */
/*  Pattern                                                            */
/* ------------------------------------------------------------------ */

static void roll_lane(smack_t *s, smack_lane_t *ln) {
    int n = s->n_slices;

    for (int i = 0; i < n; i++) ln->order[i] = (uint16_t)i;
    for (int i = 0; i < n; i++) {
        if (rnd01(&s->rng) < s->order_density) {
            int j = rnd_below(&s->rng, n);
            uint16_t t = ln->order[i]; ln->order[i] = ln->order[j]; ln->order[j] = t;
        }
    }

    /* Locked slices still CONSUME their rng draws (computed then discarded)
     * so the stream stays aligned: for a given seed+nonce the unlocked
     * slices roll identically whether or not pins exist. That is what lets
     * a preset/layout restore (which re-rolls with locks applied) reproduce
     * exactly the pattern heard when the pin was placed live. */
    for (int i = 0; i < n; i++) {
        uint8_t rf = SMACK_FX_NONE;
        int8_t  rp = 0;
        if (rnd01(&s->rng) < s->fx_density) {
            int f = 1 + rnd_below(&s->rng, SMACK_FX_COUNT - 1);
            rf = (uint8_t)f;
            switch (f) {
            case SMACK_FX_RETRIG: rp = (int8_t)(1 + rnd_below(&s->rng, 3)); break;
            case SMACK_FX_PITCH: {
                int semi = 1 + rnd_below(&s->rng, s->pitch_range);
                if (xs32(&s->rng) & 1) semi = -semi;
                rp = (int8_t)semi;
                break;
            }
            case SMACK_FX_SPEED: rp = (int8_t)(xs32(&s->rng) & 1); break;
            case SMACK_FX_BUZZ:  rp = (int8_t)(xs32(&s->rng) & 3); break;
            case SMACK_FX_CRUSH: rp = (int8_t)(2 + (xs32(&s->rng) & 6)); break;
            case SMACK_FX_REPEAT:
            case SMACK_FX_REVAFTER: rp = (int8_t)(1 + rnd_below(&s->rng, 3)); break;
            case SMACK_FX_TAPESTOP: rp = (int8_t)(xs32(&s->rng) & 1); break;
            case SMACK_FX_SCRATCH:  rp = (int8_t)(1 + rnd_below(&s->rng, 3)); break;
            case SMACK_FX_ENV:      rp = (int8_t)rnd_below(&s->rng, 4); break;
            case SMACK_FX_PAN:      rp = (int8_t)rnd_below(&s->rng, 3); break;
            case SMACK_FX_FILTER:   rp = (int8_t)rnd_below(&s->rng, 4); break;
            case SMACK_FX_VOWEL:    rp = (int8_t)rnd_below(&s->rng, 4); break;
            case SMACK_FX_TONALDELAY: rp = (int8_t)rnd_below(&s->rng, 4); break;
            case SMACK_FX_FREEZE:   rp = (int8_t)rnd_below(&s->rng, 4); break;
            case SMACK_FX_DELAY:    rp = (int8_t)rnd_below(&s->rng, 4); break;
            case SMACK_FX_DIST:     rp = (int8_t)rnd_below(&s->rng, 4); break;
            case SMACK_FX_PHASER:   rp = (int8_t)rnd_below(&s->rng, 4); break;
            case SMACK_FX_VERB:     rp = (int8_t)rnd_below(&s->rng, 4); break;
            case SMACK_FX_PSHIFT: { /* like PITCH: +/- within pitch range */
                int semi = 1 + rnd_below(&s->rng, s->pitch_range);
                if (xs32(&s->rng) & 1) semi = -semi;
                rp = (int8_t)semi;
                break;
            }
            case SMACK_FX_RINGMOD:  rp = (int8_t)rnd_below(&s->rng, 6); break;
            case SMACK_FX_COMB:     rp = (int8_t)rnd_below(&s->rng, 4); break;
            case SMACK_FX_SCATTER:  rp = (int8_t)rnd_below(&s->rng, 4); break;
            default:                rp = 0; break;
            }
        }
        if (ln->locked[i]) continue;
        ln->fx[i]  = rf;
        ln->fxp[i] = rp;
        ln->fxp2[i]  = -1;   /* depth + mix are edit-only: rolls stay pure */
        ln->fxmix[i] = -1;
    }
}

static void roll_pattern(smack_t *s) {
    /* Once audio exists, its captured/resized clock span is authoritative.
     * This also keeps a take fixed if Loop Length is edited while recording. */
    int loop_hs  = (s->state == SMACK_LOOPING && s->loop_clock_ticks >= 3)
                 ? (int)(s->loop_clock_ticks / 3)
                 : loop_len_hs_table[s->loop_len_idx];
    int slice_hs = slice_hs_table[s->slice_res_idx];
    int n = loop_hs / slice_hs;
    if (n < 1) n = 1;
    if (n > SMACK_MAX_SLICES) n = SMACK_MAX_SLICES;
    s->n_slices = n;
    s->slice_frames = (s->loop_len > 0) ? (double)s->loop_len / (double)n : 0.0;

    s->rng = s->seed ^ (s->roll_nonce * 2654435761u);
    if (!s->rng) s->rng = 0xC0FFEE01u;

    /* lane 1 continues the same rng stream, so a given seed + nonce fully
     * determines both lanes, and lane 0 matches what stereo mode rolls */
    roll_lane(s, &s->lane[0]);
    if (s->chan_mode) roll_lane(s, &s->lane[1]);
    s->edit_rev++;
}

/* Equal-power pan: 0 = hard left, 50 = center, 100 = hard right. */
static void update_pan_gains(smack_t *s) {
    const double HALF_PI = 1.5707963;
    double tl = (double)s->pan_l / 100.0 * HALF_PI;
    double tr = (double)s->pan_r / 100.0 * HALF_PI;
    s->pan_gain[0][0] = (float)cos(tl);
    s->pan_gain[0][1] = (float)sin(tl);
    s->pan_gain[1][0] = (float)cos(tr);
    s->pan_gain[1][1] = (float)sin(tr);
}

/* Canonical parameter for a user-pinned effect (lock_slice with an fx code).
 * Deterministic middle-of-range choices; the seeded roll stays the source
 * of variety. Ranges mirror the roll_pattern switch above. */
static int8_t default_fxp(int f) {
    switch (f) {
    case SMACK_FX_RETRIG:
    case SMACK_FX_REPEAT:
    case SMACK_FX_REVAFTER:
    case SMACK_FX_SCRATCH:  return 2;
    case SMACK_FX_PITCH:    return 5;
    case SMACK_FX_SPEED:    return 1;
    case SMACK_FX_BUZZ:     return 1;
    case SMACK_FX_CRUSH:    return 4;
    case SMACK_FX_PSHIFT:   return 7;   /* fifth up, time intact */
    case SMACK_FX_RINGMOD:  return 2;   /* 220 Hz */
    case SMACK_FX_COMB:     return 1;   /* 110 Hz */
    default:                return 0;
    }
}

/* Pressure -> effect parameter for the global punch. Ranges mirror what
 * roll_pattern/render use per effect; pressure 0..127 sweeps them. */
static int8_t punch_map(int f, int pressure) {
    int lo = 0, hi = 0;
    switch (f) {
    case SMACK_FX_RETRIG:   lo = 1; hi = 6; break;
    case SMACK_FX_PITCH:
    case SMACK_FX_PSHIFT:   lo = -24; hi = 24; break;
    case SMACK_FX_SPEED:
    case SMACK_FX_TAPESTOP:
    case SMACK_FX_GATE:     lo = 0; hi = 3; break;
    case SMACK_FX_BUZZ:     lo = 0; hi = 5; break;
    case SMACK_FX_CRUSH:    lo = 2; hi = 24; break;
    case SMACK_FX_REPEAT:
    case SMACK_FX_REVAFTER: lo = 1; hi = 7; break;
    case SMACK_FX_SCRATCH:  lo = 1; hi = 8; break;
    case SMACK_FX_PAN:      lo = 0; hi = 5; break;
    case SMACK_FX_ENV:
    case SMACK_FX_FILTER:
    case SMACK_FX_VOWEL:
    case SMACK_FX_TONALDELAY:
    case SMACK_FX_FREEZE:
    case SMACK_FX_DELAY:
    case SMACK_FX_DIST:
    case SMACK_FX_PHASER:
    case SMACK_FX_VERB:
    case SMACK_FX_RINGMOD:
    case SMACK_FX_COMB:
    case SMACK_FX_SCATTER:  lo = 0; hi = 7; break;
    default: return 0;
    }
    int span = hi - lo + 1;
    int v = lo + (pressure * span) / 128;
    if (v > hi) v = hi;
    return (int8_t)v;
}

/* ------------------------------------------------------------------ */
/*  Capture                                                            */
/* ------------------------------------------------------------------ */

/*
 * A loop may occupy at most half the ring, because the recorder needs room to
 * write a whole fresh one alongside it while it plays. Exceed that and
 * record_ring_frame() runs into its own guard, recording stops, and captures
 * silently go stale.
 *
 * The margin is larger than the 2048 frames the recorder guards with so the
 * two limits cannot meet exactly.
 */
#define RING_WRITE_MARGIN 4096u

static uint32_t max_loop_frames(void) {
    return (uint32_t)((SMACK_RING_FRAMES - RING_WRITE_MARGIN) / 2);
}

/*
 * Step LENGTH down to the longest setting that still fits in half the ring.
 *
 * Done by lowering the index rather than clipping the frame count, because a
 * clipped loop is no longer a whole number of steps: loop_frames_per_tick
 * would be rescaled to span the same tick count in fewer frames, which plays
 * the loop back at the wrong rate and drifts against the clock. Dropping one
 * musical notch (256 -> 128 steps) stays on the grid and stays in time.
 *
 * Only reachable at slow tempos: at 51 BPM and above, 256 steps fits.
 */
static void fit_loop_len_idx(smack_t *s) {
    double   fph = frames_per_halfstep(s);
    uint32_t cap = max_loop_frames();

    if (fph <= 0.0) return;
    while (s->loop_len_idx > 0
           && fph * (double)loop_len_hs_table[s->loop_len_idx] > (double)cap)
        s->loop_len_idx--;
}

static uint32_t quantum_frames(smack_t *s) {
    double f = frames_per_halfstep(s) * (double)loop_len_hs_table[s->loop_len_idx];
    if (f < 256.0) f = 256.0;
    if (f > (double)max_loop_frames()) f = (double)max_loop_frames();
    return (uint32_t)f;
}

/* Retain valid frames preceding a captured endpoint, up to the longest loop
 * setting. Live Loop Length edits move the window start around this fixed end,
 * while ring recording stops before overwriting a later lengthening target. */
static void retain_loop_history(smack_t *s, uint32_t loop_end, uint64_t available) {
    if (available > (uint64_t)SMACK_RING_FRAMES) available = SMACK_RING_FRAMES;
    /* Keep enough history to reach the longest selectable loop, not the
     * entire 70-second ring. The remaining space must stay writable so a
     * later Capture can still grab newly played audio while this loop runs. */
    double source_frames_per_tick = s->loop_frames_per_tick > 0.0
                                  ? s->loop_frames_per_tick
                                  : frames_per_tick_now(s);
    double max_wanted = source_frames_per_tick
                      * (double)(loop_len_hs_table[LOOP_LEN_COUNT - 1] * 3);
    if (max_wanted > (double)SMACK_RING_FRAMES) max_wanted = SMACK_RING_FRAMES;
    uint64_t max_resize = (uint64_t)(max_wanted + 0.5);
    if (available > max_resize) available = max_resize;
    /*
     * Retained history is protected from the recorder, so it comes straight
     * out of the space the next capture needs. Keep back at least a whole
     * loop's worth of writable ring; without this the history alone can pin
     * the write head and stall recording even when the loop itself is short.
     */
    {
        uint64_t writable_cap =
            (uint64_t)SMACK_RING_FRAMES > (uint64_t)s->loop_len + RING_WRITE_MARGIN
                ? (uint64_t)SMACK_RING_FRAMES - s->loop_len - RING_WRITE_MARGIN
                : 0;
        if (available > writable_cap) available = writable_cap;
    }
    if (available < s->loop_len) available = s->loop_len;
    s->loop_available = (uint32_t)available;
    s->loop_history_start = (loop_end + SMACK_RING_FRAMES - s->loop_available)
                          % SMACK_RING_FRAMES;
}

/* Retroactive grab: the last loop-length of audio ending at the most recent
 * half-step boundary becomes the loop. Uses the written-frame timeline. */
static void capture_retro(smack_t *s) {
    uint32_t want;
    fit_loop_len_idx(s); /* before quantum_frames: it reads loop_len_idx */
    want = quantum_frames(s);
    uint64_t grid_boundary = last_boundary_global(s);
    uint64_t boundary = grid_boundary;
    if (boundary > s->ring_last_global) boundary = s->ring_last_global;
    uint64_t behind = s->ring_last_global - boundary; /* frames since boundary */
    if (behind > (uint64_t)SMACK_RING_FRAMES) behind = 0;

    uint64_t avail = s->written_total;
    if (avail > (uint64_t)SMACK_RING_FRAMES) avail = SMACK_RING_FRAMES;
    if (behind + want > avail) {
        if (avail <= behind + 256) return; /* nothing meaningful recorded yet */
        want = (uint32_t)(avail - behind);
    }

    uint64_t frames_ago_start = behind + want;
    s->loop_len   = want;
    s->loop_start = (uint32_t)((s->ring_w + (uint64_t)SMACK_RING_FRAMES * 2
                                - frames_ago_start) % SMACK_RING_FRAMES);
    s->loop_clock_ticks = (uint32_t)(loop_len_hs_table[s->loop_len_idx] * 3);
    s->loop_frames_per_tick = (double)want / (double)s->loop_clock_ticks;
    /* Capture may be pressed between grid boundaries. Starting at zero here
     * makes the loop late by that elapsed fraction (up to one half-step).
     * Chase the current phase so the captured buffer and Move keep lining up. */
    uint64_t elapsed = s->global_frames > grid_boundary
                     ? s->global_frames - grid_boundary : 0;
    s->play_pos = fmod((double)elapsed * loop_playback_increment(s),
                       (double)s->loop_len);
    s->state = SMACK_LOOPING;
    retain_loop_history(s, (s->loop_start + s->loop_len) % SMACK_RING_FRAMES,
                        avail - behind);
    roll_pattern(s);
}

static void begin_record(smack_t *s) {
    fit_loop_len_idx(s); /* before quantum_frames: it reads loop_len_idx */
    s->rec_start = s->ring_w;
    s->rec_length = quantum_frames(s);
    s->rec_clock_ticks = (uint32_t)(loop_len_hs_table[s->loop_len_idx] * 3);
    s->rec_remaining = s->rec_length;
    s->state = SMACK_RECORDING;
}

static void finish_record(smack_t *s) {
    s->loop_start = s->rec_start;
    s->loop_len   = s->rec_length;
    s->loop_clock_ticks = s->rec_clock_ticks;
    s->loop_frames_per_tick = s->loop_clock_ticks > 0
        ? (double)s->loop_len / (double)s->loop_clock_ticks : 0.0;
    s->play_pos = 0.0;
    s->state = SMACK_LOOPING;
    {   /* Everything currently preceding the record end is valid history. */
        uint64_t avail = s->written_total;
        if (avail > (uint64_t)SMACK_RING_FRAMES) avail = SMACK_RING_FRAMES;
        retain_loop_history(s, (s->loop_start + s->loop_len) % SMACK_RING_FRAMES,
                            avail);
    }
    roll_pattern(s);
}

/* ------------------------------------------------------------------ */
/*  Lifecycle                                                          */
/* ------------------------------------------------------------------ */

/* Factory palette order (rows bottom-up from pad 76), grouped by family:
 * clean + stutter + tape / reverse + pitch + texture + crush /
 * filters + space + destruction. The v0.11.0 effects (PShift/RingMod/
 * Comb/Scatter) are off the factory pads — swap them in via the editor.
 * Mirrored as the fallback in both UIs. */
static const uint8_t k_default_palette[SMACK_PALETTE_SLOTS] = {
    0, 1, 8, 6, 5, 10, 11, 12,
    2, 9, 3, 4, 18, 13, 14, 7,
    15, 16, 21, 17, 19, 22, 20
};

smack_t *smack_create(const host_api_v1_t *host) {
    smack_t *s = calloc(1, sizeof(smack_t));
    if (!s) return NULL;
    s->ring = calloc((size_t)SMACK_RING_FRAMES * 2, sizeof(int16_t));
    if (!s->ring) { free(s); return NULL; }
    for (int k = 0; k < 2; k++) {
        s->lane[k].dly  = calloc((size_t)SMACK_DLY_LEN * 2, sizeof(float));
        s->lane[k].verb = calloc((size_t)VERB_TOTAL, sizeof(float));
        if (!s->lane[k].dly || !s->lane[k].verb) {
            for (int j = 0; j <= k; j++) { free(s->lane[j].dly); free(s->lane[j].verb); }
            free(s->ring); free(s);
            return NULL;
        }
        s->lane[k].flt_slice = -1;
        s->lane[k].vow_slice = -1;
        s->lane[k].dly_slice = -1;
        s->lane[k].ph_slice  = -1;
    }
    s->host = host;
    s->frames_per_tick = 918.75; /* 120 BPM */
    s->loop_len_idx = 4;         /* 1 bar */
    s->slice_res_idx = 1;        /* 1 step */
    s->fx_density = 0.5f;
    s->order_density = 0.0f;
    s->pitch_range = 12;
    s->wet = 1.0f;
    s->ab = 1;
    s->ab_pending = -1;
    s->quantize_mode = 1;        /* next slice */
    s->seed = 4303u;  /* within the knob's 1..9999 range */
    s->follow_transport = 1;
    s->monitor = 1;
    s->punch_fx = -1;
    s->chan_mode = 0;            /* stereo */
    s->pan_l = 0;                /* dual-mono defaults keep input sides */
    s->pan_r = 100;
    s->pad_rate_idx = 2;         /* pad-play repeats on the beat */
    /* pad_play stays 0 here; the gen wrapper turns it on (slot-editor pads
     * play notes into smack-in) while the chain fx build stays inert. */
    memcpy(s->palette, k_default_palette, sizeof(s->palette));
    /* palette_fxp/hasp start zeroed (calloc): every pad = default param */
    memset(s->palette_fxp2, -1, sizeof(s->palette_fxp2));
    memset(s->palette_mix, -1, sizeof(s->palette_mix));
    s->punch_fxp2 = -1;
    s->punch_mix = -1;
    for (int k = 0; k < 2; k++) {
        memset(s->lane[k].fxp2, -1, sizeof(s->lane[k].fxp2));
        memset(s->lane[k].fxmix, -1, sizeof(s->lane[k].fxmix));
    }
    update_pan_gains(s);
    return s;
}

/* Trigger params fire on any ACTIVE value: chain-UI pads send "1" (every
 * press fires), and the hierarchy trigger-enum knobs send the literal string
 * "trigger" (shadow_ui fires one per detent gesture). "0"/"idle" are always
 * no-ops — that is all autosave restores and UI init ever send. */
static int trig_active(const char *val) {
    return atoi(val) != 0 || strcmp(val, "trigger") == 0;
}

static int json_int(const char *js, const char *key, int def) {
    char pat[40];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(js, pat);
    return p ? atoi(p + strlen(pat)) : def;
}

void smack_destroy(smack_t *s) {
    if (!s) return;
    for (int k = 0; k < 2; k++) { free(s->lane[k].verb); free(s->lane[k].dly); }
    free(s->ring);
    free(s);
}

/* ------------------------------------------------------------------ */
/*  MIDI (clock + transport)                                           */
/* ------------------------------------------------------------------ */

/* Held-note stack, most-recent-last. Re-pressing a note moves it to the
 * top; the top note is what fires at each rate boundary. */
static void pad_stack_remove(smack_t *s, int note) {
    for (int i = 0; i < s->pad_held; i++) {
        if (s->pad_note_stk[i] == (uint8_t)note) {
            for (int j = i + 1; j < s->pad_held; j++) {
                s->pad_note_stk[j - 1] = s->pad_note_stk[j];
                s->pad_vel_stk[j - 1]  = s->pad_vel_stk[j];
            }
            s->pad_held--;
            i--;
        }
    }
}

static void pad_note_on(smack_t *s, int note, int vel) {
    if (s->state != SMACK_LOOPING) return;
    pad_stack_remove(s, note);
    if (s->pad_held >= (int)sizeof(s->pad_note_stk)) { /* drop the oldest */
        for (int j = 1; j < s->pad_held; j++) {
            s->pad_note_stk[j - 1] = s->pad_note_stk[j];
            s->pad_vel_stk[j - 1]  = s->pad_vel_stk[j];
        }
        s->pad_held--;
    }
    s->pad_note_stk[s->pad_held] = (uint8_t)note;
    s->pad_vel_stk[s->pad_held]  = (uint8_t)vel;
    s->pad_held++;
}

static void pad_note_off(smack_t *s, int note) {
    pad_stack_remove(s, note);
    if (s->pad_held == 0) s->trig_active = 0; /* release = back to the loop */
}

/* --- MIDI CC control ----------------------------------------------- */
/* External controllers (USB-A) drive the performance surface. Routed
 * through smack_set_param so CC edits share every rule with the UI
 * (trig_active, quantized A/B pending, reroll nonce, edit_rev).
 *   20 wet          21 fx_density   22 order_density  23 slice_res
 *   24 pitch_range  25 seed (0-127 browses patterns)  26 quantize
 *   27 ab (>=64)    28 pan_l        29 pan_r
 *   30 punch: 0 = release, 1 = punch clean, 2+ = punch effect f-1
 *   31 punch_pressure (raw 0-127)
 *   40 arm  41 capture  42 reroll  43 clear   (fire at >=64)
 *   44 monitor (>=64)
 * Continuous 0-127 scales into the param range; see README. */
static void smack_handle_cc(smack_t *s, int cc, int v) {
    char val[16];
    int on = v >= 64;
    const char *key = NULL;
    int scaled = -1;
    switch (cc) {
    case 20: key = "wet";           scaled = (v * 100 + 63) / 127; break;
    case 21: key = "fx_density";    scaled = (v * 100 + 63) / 127; break;
    case 22: key = "order_density"; scaled = (v * 100 + 63) / 127; break;
    case 23: key = "slice_res";
             scaled = (v * (SLICE_RES_COUNT - 1) + 63) / 127;      break;
    case 24: key = "pitch_range";   scaled = 1 + (v * 23 + 63) / 127; break;
    case 25: key = "seed";          scaled = v;                    break;
    case 26: key = "quantize";      scaled = (v * 2 + 63) / 127;   break;
    case 27: smack_set_param(s, "ab", on ? "1" : "0");             return;
    case 28: key = "pan_l";         scaled = (v * 100 + 63) / 127; break;
    case 29: key = "pan_r";         scaled = (v * 100 + 63) / 127; break;
    case 30:
        if (v == 0) smack_set_param(s, "punch_fx", "-1");
        else {
            snprintf(val, sizeof val, "%d",
                     v - 1 > SMACK_FX_COUNT - 1 ? SMACK_FX_COUNT - 1 : v - 1);
            smack_set_param(s, "punch_fx", val);
        }
        return;
    case 31: key = "punch_pressure"; scaled = v;                   break;
    case 40: if (on) smack_set_param(s, "arm", "1");               return;
    case 41: if (on) smack_set_param(s, "capture", "1");           return;
    case 42: if (on) smack_set_param(s, "reroll", "1");            return;
    case 43: if (on) smack_set_param(s, "clear", "1");             return;
    case 44: smack_set_param(s, "monitor", on ? "1" : "0");        return;
    default: return;
    }
    snprintf(val, sizeof val, "%d", scaled);
    smack_set_param(s, key, val);
}

void smack_on_midi(smack_t *s, const uint8_t *msg, int len, int source) {
    if (!s || len < 1) return;
    /* External CC control. Internal MIDI never acts as CC; a channel-
     * matched chain slot delivers one external CC twice (channel dispatch
     * + FX broadcast), so identical messages within ~2 blocks drop. */
    if (len >= 3 && (msg[0] & 0xF0) == 0xB0 &&
        (source == MOVE_MIDI_SOURCE_EXTERNAL ||
         source == MOVE_MIDI_SOURCE_FX_BROADCAST)) {
        if (!(msg[0] == s->cc_last[0] && msg[1] == s->cc_last[1] &&
              msg[2] == s->cc_last[2] &&
              s->global_frames - s->cc_last_frames <= 256)) {
            s->cc_last[0] = msg[0];
            s->cc_last[1] = msg[1];
            s->cc_last[2] = msg[2];
            s->cc_last_frames = s->global_frames;
            smack_handle_cc(s, msg[1], msg[2]);
        }
        return;
    }
    switch (msg[0]) {
    case 0xFA: /* start */
    case 0xFB: /* continue: treated as downbeat too (pushnpull convention) */
        s->tick_total = 0;
        s->clock_running = 1;
        s->last_halfstep_global = s->global_frames;
        s->tick_history_count = 0;
        s->tick_history_pos = 0;
        s->clock_seq = 0;
        s->pad_held = 0;          /* stuck-note safety across transport */
        s->trig_active = 0;
        if (s->transport_paused) { /* resume the loop from its top */
            s->transport_paused = 0;
            s->play_pos = 0.0;
        }
        break;
    case 0xFC:
        s->clock_running = 0;
        if (s->follow_transport && s->state == SMACK_LOOPING)
            s->transport_paused = 1;
        break;
    case 0xF8: {
        uint32_t previous_tick = s->tick_total;
        uint32_t elapsed_ticks = clock_estimator_push(s, s->global_frames);
        s->clock_seen = 1;
        s->tick_total += elapsed_ticks;
        if (s->tick_total / 3 != previous_tick / 3) { /* crossed half-step */
            s->last_halfstep_global = s->global_frames;
        }
        if (s->state == SMACK_ARMED && s->tick_total / 6 != previous_tick / 6)
            s->arm_start_flag = 1; /* crossed a step boundary */
        break;
    }
    default: {
        /* Channel voice: pads in a slot editor play notes into the synth —
         * with pad_play on, those notes trigger pattern cells. Note-offs
         * always drain the stack so toggling pad_play can't stick a note. */
        int st = msg[0] & 0xF0;
        if (st == 0x90 && len >= 3) {
            if (msg[2] > 0) { if (s->pad_play) pad_note_on(s, msg[1], msg[2]); }
            else pad_note_off(s, msg[1]);
        } else if (st == 0x80 && len >= 3) {
            pad_note_off(s, msg[1]);
        }
        break;
    }
    }
}

/* ------------------------------------------------------------------ */
/*  Rendering                                                          */
/* ------------------------------------------------------------------ */

/* ch: -1 = stereo, 0/1 = that input channel duplicated to both outputs
 * (dual-mono lanes render a mono source through the stereo fx path). */
static inline void ring_read_lerp(const smack_t *s, double loop_off, int ch,
                                  float *l, float *r) {
    if (loop_off < 0.0) loop_off = 0.0;
    double maxo = (double)s->loop_len - 1.0;
    if (loop_off > maxo) loop_off = maxo;
    uint32_t i0 = (uint32_t)loop_off;
    float frac = (float)(loop_off - (double)i0);
    uint32_t a = (s->loop_start + i0) % SMACK_RING_FRAMES;
    uint32_t b = (i0 + 1 <= (uint32_t)maxo)
                   ? (s->loop_start + i0 + 1) % SMACK_RING_FRAMES : a;
    if (ch < 0) {
        *l = (float)s->ring[a * 2]     + frac * ((float)s->ring[b * 2]     - (float)s->ring[a * 2]);
        *r = (float)s->ring[a * 2 + 1] + frac * ((float)s->ring[b * 2 + 1] - (float)s->ring[a * 2 + 1]);
    } else {
        float v = (float)s->ring[a * 2 + ch]
                + frac * ((float)s->ring[b * 2 + ch] - (float)s->ring[a * 2 + ch]);
        *l = v;
        *r = v;
    }
}

static inline int16_t clip16(float v) {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

/* Render one looped output frame at play_pos into l/r, through one lane.
 * ch: -1 = stereo (lane 0, classic mode), 0/1 = dual-mono lane reading
 * that input channel.
 * side: -1 = follow s->ab, 0 = force clean, 1 = force pattern (the input
 * builds render both sides and blend them per `wet`). */
static void render_lane(smack_t *s, smack_lane_t *ln, int ch, int side, float *l, float *r) {
    double sf = s->slice_frames;
    int oslice = (sf > 0.0) ? (int)(s->play_pos / sf) : 0;
    if (oslice >= s->n_slices) oslice = s->n_slices - 1;
    double p = s->play_pos - (double)oslice * sf;
    if (s->trig_render) { /* pad-play: force the held cell, repeat-local pos */
        oslice = s->trig_step;
        if (oslice >= s->n_slices) oslice = s->n_slices > 0 ? s->n_slices - 1 : 0;
        p = s->trig_pos;
        if (p > sf - 1.0) p = sf - 1.0;
    }
    float g = 1.0f, gl = 1.0f, gr = 1.0f;
    float gedge = 1.0f;   /* slice-edge + loop fades, shared by wet AND dry */
    int f = SMACK_FX_NONE, fp = 0;
    int fp2 = -1, fmix = -1;  /* per-effect depth / per-slice mix, -1 dflt */
    double src;
    double src_dry = -1.0; /* unwarped tap for the per-slice mix blend */
    double src2 = -1.0;   /* FREEZE: second grain tap (slice-relative) */
    float  mix2 = 0.0f;   /* FREEZE: crossfade toward primary tap */
    int use_pattern = (side < 0) ? s->ab : side;
    if (s->punch_fx == 0) use_pattern = 0;      /* punching clean */
    else if (s->punch_fx > 0) use_pattern = 1;  /* punching an effect */

    if (!use_pattern) {
        /* clean, original order (pad-play still repeats its cell here) */
        src = s->trig_render ? ((double)oslice * sf + p) : s->play_pos;
    } else {
        int sslice = ln->order[oslice];
        f  = ln->fx[oslice];
        fp = ln->fxp[oslice];
        fp2  = ln->fxp2[oslice];
        fmix = ln->fxmix[oslice];
        if (s->punch_fx > 0) {
            sslice = oslice;                    /* punch: original order, */
            f  = s->punch_fx;                   /* one forced effect      */
            fp = s->punch_fxp;
            fp2  = s->punch_fxp2;
            fmix = s->punch_mix;
        }
        src_dry = (double)sslice * sf + p;
        double rp = p;
        switch (f) {
        case SMACK_FX_RETRIG: {
            /* fp 1..6 = repeat window 1/2 .. 1/64 of the slice. Rolls stay
             * 1..3; the deeper rates are editor/punch territory. */
            int rr = fp < 1 ? 1 : (fp > 6 ? 6 : fp);
            double win = sf / (double)(1 << rr);
            if (win < 32.0) win = 32.0;
            int k = (int)(p / win);
            rp = p - (double)k * win;
            if (fp2 >= 0) {    /* depth = decay: 0 no die-away, 100 fast */
                float d = 1.0f - (float)fp2 / 110.0f;
                g *= powf(d, (float)(k > 15 ? 15 : k));
            } else {
                g *= retrig_decay[k > 15 ? 15 : k];
            }
            break;
        }
        case SMACK_FX_REVERSE: rp = (sf - 1.0) - p; break;
        case SMACK_FX_PITCH: {
            double semi = (double)fp;
            if (fp2 > 0) {     /* depth = glide: ramp toward the target */
                double t = p / sf;
                semi *= ((double)(100 - fp2) + (double)fp2 * t) / 100.0;
            }
            double rate = pow(2.0, semi / 12.0);
            rp = fmod(p * rate, sf);
            break;
        }
        case SMACK_FX_SPEED: {
            /* 0 half, 1 double, 2 x1.5 (fifth up), 3 x0.75 (fourth down) */
            static const double spd[4] = { 0.5, 2.0, 1.5, 0.75 };
            rp = fmod(p * spd[fp & 3], sf);
            break;
        }
        case SMACK_FX_GATE: {
            /* fp: 0 = 8 chops, 1 = 4, 2 = 16, 3 = swung 8 (long-short) */
            int seg;
            if ((fp & 3) == 3) {
                seg = (fmod(p * 4.0 / sf, 1.0) < 0.667) ? 0 : 1;
            } else {
                static const double nseg[3] = { 8.0, 4.0, 16.0 };
                seg = (int)(p * nseg[fp & 3] / sf);
            }
            /* depth = gate floor: 0 gentle tremolo, 100 hard silence */
            if (seg & 1) g *= (fp2 >= 0) ? 1.0f - (float)fp2 / 100.0f : 0.12f;
            break;
        }
        case SMACK_FX_BUZZ: {
            int bw = fp < 0 ? 0 : (fp > 5 ? 5 : fp);   /* 96..3072 frames */
            double win = (double)(96 << bw);
            rp = fmod(p, win);
            /* depth = fade the buzz out across the slice */
            if (fp2 > 0) g *= 1.0f - (float)fp2 / 100.0f * (float)(p / sf);
            break;
        }
        case SMACK_FX_CRUSH: {
            double hold = (double)(fp < 2 ? 2 : (fp > 24 ? 24 : fp));
            rp = floor(p / hold) * hold;
            break;
        }
        case SMACK_FX_REPEAT: {
            /* play the head, then stutter it (Looperator "repeat after X") */
            static const double frac[8] =
                { 0.5, 0.5, 0.25, 0.75, 0.125, 0.3333, 0.6667, 0.0625 };
            double split = sf * frac[fp & 7];
            rp = (p < split) ? p : fmod(p - split, split);
            if (fp2 >= 0 && p >= split) { /* depth = per-repeat die-away */
                int k = (int)((p - split) / split);
                g *= powf(1.0f - (float)fp2 / 110.0f, (float)(k > 15 ? 15 : k + 1));
            }
            break;
        }
        case SMACK_FX_REVAFTER: {
            /* forward, then play backwards from the split point */
            static const double frac[8] =
                { 0.5, 0.5, 0.6667, 0.75, 0.25, 0.125, 0.875, 0.9375 };
            double split = sf * frac[fp & 7];
            if (p < split) rp = p;
            else {
                rp = split - (p - split);
                if (rp < 0.0) rp = 0.0;
            }
            break;
        }
        case SMACK_FX_TAPESTOP: {
            /* rate ramps 1 -> 0; position is the integral of the ramp.
             * 0 full-slice stop, 1 fast (dead by half), 2 sag (down to
             * half speed, never stops), 3 slam (dead by quarter) */
            switch (fp & 3) {
            case 1:
                rp = (p < sf * 0.5) ? p - p * p / sf : sf * 0.25;
                if (p >= sf * 0.5) g = 0.0f;
                break;
            case 2:
                rp = p - p * p / (4.0 * sf);
                break;
            case 3:
                rp = (p < sf * 0.25) ? p - 2.0 * p * p / sf : sf * 0.125;
                if (p >= sf * 0.25) g = 0.0f;
                break;
            default:
                rp = p - p * p / (2.0 * sf);
                break;
            }
            break;
        }
        case SMACK_FX_TAPESTART:
            rp = p * p / (2.0 * sf);            /* spin up from standstill */
            break;
        case SMACK_FX_SCRATCH: {
            double cyc = (double)(fp < 1 ? 1 : (fp > 8 ? 8 : fp));
            double depth = (fp2 >= 0)      /* depth = throw distance */
                ? sf * (0.02 + 0.28 * (double)fp2 / 100.0) : sf / 10.0;
            rp = p + depth * sin(6.2831853 * cyc * p / sf);
            break;
        }
        case SMACK_FX_ENV:
            switch (fp & 7) {
            case 0: g *= (float)(p / sf); break;                 /* fade in */
            case 1: g *= (float)(1.0 - p / sf); break;           /* fade out */
            case 2: g *= 0.5f + 0.5f * sinf((float)(6.2831853 * 6.0 * p / sf)); break;
            case 3: if (p < sf * 0.5) g = 0.0f; break;           /* off-then-on */
            case 4: g *= (float)sin(3.1415926 * p / sf); break;  /* swell mid */
            case 5: g *= 0.5f + 0.5f * sinf((float)(6.2831853 * 16.0 * p / sf)); break; /* shiver */
            case 6: { int sg = (int)(p * 4.0 / sf); if (sg & 1) g = 0.0f; break; } /* hard chop 4 */
            case 7: g *= 1.0f - 0.9f * (float)sin(3.1415926 * p / sf); break; /* duck mid */
            }
            break;
        case SMACK_FX_PAN: {
            /* depth = width: how far the quiet side drops (dflt 0.08) */
            float pw = (fp2 >= 0) ? 1.0f - 0.92f * (float)fp2 / 100.0f : 0.08f;
            float ps2 = 1.0f - pw;
            switch (fp < 0 ? 0 : (fp > 5 ? 5 : fp)) {
            case 0: gr = pw; break;                              /* hard left */
            case 1: gl = pw; break;                              /* hard right */
            case 2: {                                            /* ping-pong x4 */
                int seg = (int)(p * 4.0 / sf);
                if (seg & 1) gl = pw; else gr = pw;
                break;
            }
            case 3: {                                            /* ping-pong x8 */
                int seg = (int)(p * 8.0 / sf);
                if (seg & 1) gl = pw; else gr = pw;
                break;
            }
            case 4: {                                            /* sweep L -> R */
                float t = (float)(p / sf);
                gl = 1.0f - ps2 * t; gr = pw + ps2 * t;
                break;
            }
            case 5: {                                            /* sweep R -> L */
                float t = (float)(p / sf);
                gr = 1.0f - ps2 * t; gl = pw + ps2 * t;
                break;
            }
            }
            break;
        }
        case SMACK_FX_FREEZE: {
            /* 50%-overlap grains frozen on the slice head; per-grain start
             * jitter (deterministic from grain index + seed) sprays the
             * texture. Two taps, triangular crossfade. */
            /* depth = grain size: tight granular dust .. long smears */
            const double H = (fp2 >= 0)
                ? 128.0 + (2048.0 - 128.0) * (double)fp2 / 100.0 : 512.0;
            /* 0-3: spray 25..100% of a quarter-slice zone (rolled range,
             * unchanged). 4-7: same sprays over a half-slice zone. */
            int sv = fp < 0 ? 0 : (fp > 7 ? 7 : fp);
            double region = sf * ((sv >= 4) ? 0.5 : 0.25) - 2.0 * H;
            if (region < 0.0) region = 0.0;
            double spray = region * (double)((sv & 3) + 1) * 0.25;
            int a = (int)(p / H);
            double local = p - (double)a * H;
            uint32_t h1 = ((uint32_t)a * 2654435761u) ^ s->seed;
            uint32_t h0 = ((uint32_t)(a - 1) * 2654435761u) ^ s->seed;
            rp = (double)(h1 & 0xFFFF) / 65535.0 * spray + local;
            src2 = (double)(h0 & 0xFFFF) / 65535.0 * spray + H + local;
            mix2 = (float)(local / H);              /* 0 -> old tap, 1 -> new */
            break;
        }
        case SMACK_FX_PSHIFT: {
            /* time-preserving pitch shift: grains re-anchor to real time
             * every G frames and are read at the varispeed rate inside;
             * two taps crossfade (FREEZE-style), so rhythm stays put while
             * pitch moves. Lo-fi granular flutter is part of the charm. */
            /* depth = grain size: small = robotic warble, big = smeary */
            const double G = (fp2 >= 0)
                ? 256.0 + (4096.0 - 256.0) * (double)fp2 / 100.0 : 1024.0;
            int semi = fp < -24 ? -24 : (fp > 24 ? 24 : fp);
            double rate = pow(2.0, (double)semi / 12.0);
            int a = (int)(p / G);
            double local = p - (double)a * G;
            rp   = (double)a * G + local * rate;          /* new grain */
            src2 = (double)(a - 1) * G + (local + G) * rate; /* old grain */
            if (src2 < 0.0) src2 = 0.0;
            mix2 = (float)(local / G);              /* 0 -> old, 1 -> new */
            break;
        }
        case SMACK_FX_SCATTER: {
            /* hash-shuffled grains within the slice — deterministic from
             * seed + slice + grain, no rng state. 0 shuffle-8, 1 shuffle-4,
             * 2 shuffle-16, 3 shuffle-8 + reversed grains; 4-7 = the same
             * with sparse dropouts. */
            int sv = fp < 0 ? 0 : (fp > 7 ? 7 : fp);
            static const int ngt[4] = { 8, 4, 16, 8 };
            int ng = ngt[sv & 3];
            double gw = sf / (double)ng;
            if (gw < 96.0) { gw = sf; ng = 1; }     /* tiny slices: bypass */
            int gi = (int)(p / gw);
            if (gi >= ng) gi = ng - 1;
            double local = p - (double)gi * gw;
            uint32_t h = ((uint32_t)(gi + 1) * 2246822519u)
                       ^ ((uint32_t)(oslice + 1) * 2654435761u) ^ s->seed;
            int sg = (int)(h % (uint32_t)ng);
            /* depth = chaos: % of grains that actually relocate */
            if (fp2 >= 0 && (int)((h >> 16) % 100u) >= fp2) sg = gi;
            int rev = ((sv & 3) == 3) && (h & 0x100u);
            rp = (double)sg * gw + (rev ? (gw - 1.0 - local) : local);
            if (sv >= 4 && ((h >> 9) & 3u) == 0u) g = 0.0f; /* dropouts */
            double ge = local < (gw - local) ? local : (gw - local);
            if (ge < 64.0) g *= (float)(ge / 64.0); /* per-grain edge fade */
            break;
        }
        default: break;
        }
        if (rp < 0.0) rp = 0.0;
        if (rp > sf - 1.0) rp = sf - 1.0;
        src = (double)sslice * sf + rp;
        if (src2 >= 0.0) {
            if (src2 > sf - 1.0) src2 = sf - 1.0;
            src2 = (double)sslice * sf + src2;
        }

        /* slice-edge fade masks discontinuities from reorder/fx */
        if (p < SMACK_EDGE_FADE) gedge *= (float)(p / SMACK_EDGE_FADE);
        double tail = sf - p;
        if (tail < SMACK_EDGE_FADE) gedge *= (float)(tail / SMACK_EDGE_FADE);
    }

    /* loop-boundary fade in both modes */
    if (s->play_pos < SMACK_EDGE_FADE)
        gedge *= (float)(s->play_pos / SMACK_EDGE_FADE);
    double ltail = (double)s->loop_len - s->play_pos;
    if (ltail < SMACK_EDGE_FADE) gedge *= (float)(ltail / SMACK_EDGE_FADE);

    ring_read_lerp(s, src, ch, l, r);

    if (src2 >= 0.0) { /* FREEZE: blend the second grain tap */
        float l2, r2;
        ring_read_lerp(s, src2, ch, &l2, &r2);
        *l = *l * mix2 + l2 * (1.0f - mix2);
        *r = *r * mix2 + r2 * (1.0f - mix2);
    }

    if (f == SMACK_FX_FILTER) {
        if (ln->flt_slice != oslice) { /* fresh sweep on slice entry */
            bq_reset(&ln->fltL);
            bq_reset(&ln->fltR);
            ln->flt_slice = oslice;
            ln->flt_counter = 0;
        }
        if (ln->flt_counter-- <= 0) {
            ln->flt_counter = 32;
            float t = (float)(p / sf);
            float fc;
            int hp = (fp & 3) >= 2;
            switch (fp & 3) {
            case 0:  fc = 6000.0f * powf(200.0f / 6000.0f, t); break; /* LP down */
            case 1:  fc = 200.0f * powf(6000.0f / 200.0f, t); break;  /* LP up */
            case 2:  fc = 120.0f * powf(4000.0f / 120.0f, t); break;  /* HP up */
            default: fc = 4000.0f * powf(120.0f / 4000.0f, t); break; /* HP down */
            }
            /* variants 4-7 = the same four sweeps, resonant; depth
             * overrides with a continuous resonance 0.7 .. 8 */
            float q = (fp2 >= 0) ? 0.7f + 7.3f * (float)fp2 / 100.0f
                                 : (((fp & 7) >= 4) ? 4.0f : 0.9f);
            bq_set_q(&ln->fltL, fc, hp, q);
            ln->fltR = ln->fltL;
        }
        *l = bq_run(&ln->fltL, *l);
        *r = bq_run(&ln->fltR, *r);
    } else {
        ln->flt_slice = -1;
    }

    if (f == SMACK_FX_VOWEL) {
        if (ln->vow_slice != oslice) {
            for (int k = 0; k < 3; k++) { bq_reset(&ln->vowL[k]); bq_reset(&ln->vowR[k]); }
            ln->vow_slice = oslice;
            ln->vow_counter = 0;
        }
        if (ln->vow_counter-- <= 0) {
            ln->vow_counter = 32;
            float t = (float)(p / sf);
            const int *pr = vowel_pair[fp & 7];
            /* depth = formant sharpness: soft talk box .. piercing */
            float vq = (fp2 >= 0) ? 3.0f + 12.0f * (float)fp2 / 100.0f : 8.0f;
            for (int k = 0; k < 3; k++) {
                float fc = vowel_f[pr[0]][k] + (vowel_f[pr[1]][k] - vowel_f[pr[0]][k]) * t;
                bq_set_bandpass(&ln->vowL[k], fc, vq);
                ln->vowR[k] = ln->vowL[k];
            }
        }
        float xl = *l, xr = *r, ol = 0.0f, orr = 0.0f;
        for (int k = 0; k < 3; k++) {
            ol  += vowel_gain[k] * bq_run(&ln->vowL[k], xl);
            orr += vowel_gain[k] * bq_run(&ln->vowR[k], xr);
        }
        *l = ol * 2.4f;
        *r = orr * 2.4f;
    } else {
        ln->vow_slice = -1;
    }

    if (f == SMACK_FX_TONALDELAY || f == SMACK_FX_DELAY || f == SMACK_FX_COMB) {
        if (ln->dly_slice != oslice) { /* fresh line per slice entry */
            memset(ln->dly, 0, (size_t)SMACK_DLY_LEN * 2 * sizeof(float));
            ln->dly_w = 0;
            ln->dly_slice = oslice;
        }
        double d;
        if (f == SMACK_FX_COMB) {
            /* tuned feedback comb: delay = one pitch period. 0-3 static
             * 55/110/220/440 Hz, 4 rise / 5 fall (period sweep), 6-7 =
             * screamers (hot feedback) at 110/220. */
            static const double per[4] = { 802.0, 401.0, 200.5, 100.25 };
            int cv = fp < 0 ? 0 : (fp > 7 ? 7 : fp);
            double t = p / sf;
            if (cv == 4)      d = 802.0 - 674.0 * t;   /* pitch rises */
            else if (cv == 5) d = 128.0 + 674.0 * t;   /* pitch falls */
            else              d = per[cv >= 6 ? cv - 5 : cv & 3];
            if (d < 64.0) d = 64.0;
        } else if (f == SMACK_FX_TONALDELAY) {
            double t = p / sf;
            const double dmax = 3000.0, dmin = 700.0;
            switch (fp & 7) {
            case 0:  d = dmax - (dmax - dmin) * t; break;   /* repeats pitch up */
            case 1:  d = dmin + (dmax - dmin) * t; break;   /* repeats pitch down */
            case 2:  d = dmin + (dmax - dmin) * 0.5 * (1.0 + sin(6.2831853 * 2.0 * t)); break;
            case 3:  d = dmax - (dmax - dmin) * (floor(t * 4.0) * 0.25); break; /* stepped */
            case 4:  d = dmax - (dmax - dmin) * 0.5 * t; break; /* slow rise */
            case 5:  d = dmin + (dmax - dmin) * 0.5 * t; break; /* slow fall */
            case 6:  d = dmin + (dmax - dmin) * 0.5 * (1.0 + sin(6.2831853 * 6.0 * t)); break;
            default: d = dmax - (dmax - dmin) * (floor(t * 8.0) * 0.125); break; /* stairs 8 */
            }
        } else { /* DELAY: fixed tempo-synced echo time */
            double step = frames_per_halfstep(s) * 2.0;
            switch (fp & 7) {
            case 1: case 7: d = step * 2.0; break;          /* 8th (7 = pingpong 8th) */
            case 2:  d = step * 1.5; break;                 /* dotted 16th */
            case 4:  d = step * 0.5; break;                 /* 32nd */
            case 5:  d = step * 4.0 / 3.0; break;           /* triplet 8th */
            case 6:  d = step * 0.25; break;                /* 64th buzz */
            default: d = step; break;                       /* 16th; 3 = pingpong 16th */
            }
            if (d > (double)(SMACK_DLY_LEN - 2)) d = (double)(SMACK_DLY_LEN - 2);
            if (d < 64.0) d = 64.0;
        }
        double rpos = (double)ln->dly_w - d;
        while (rpos < 0.0) rpos += (double)SMACK_DLY_LEN;
        uint32_t i0 = (uint32_t)rpos % SMACK_DLY_LEN;
        uint32_t i1 = (i0 + 1) % SMACK_DLY_LEN;
        float fr2 = (float)(rpos - floor(rpos));
        float dl = ln->dly[i0 * 2]     + fr2 * (ln->dly[i1 * 2]     - ln->dly[i0 * 2]);
        float dr = ln->dly[i0 * 2 + 1] + fr2 * (ln->dly[i1 * 2 + 1] - ln->dly[i0 * 2 + 1]);
        float fbk = 0.5f;
        if (f == SMACK_FX_COMB) fbk = ((fp & 7) >= 6) ? 0.80f : 0.65f;
        /* depth = feedback for the whole delay family (capped stable) */
        if (fp2 >= 0) fbk = 0.15f + 0.72f * (float)fp2 / 100.0f;
        if (f == SMACK_FX_DELAY && ((fp & 7) == 3 || (fp & 7) == 7)) {
            ln->dly[ln->dly_w * 2]     = *l * 0.5f + dr * 0.45f; /* ping-pong */
            ln->dly[ln->dly_w * 2 + 1] = *r * 0.5f + dl * 0.45f;
        } else {
            ln->dly[ln->dly_w * 2]     = *l * 0.55f + dl * fbk;
            ln->dly[ln->dly_w * 2 + 1] = *r * 0.55f + dr * fbk;
        }
        ln->dly_w = (ln->dly_w + 1) % SMACK_DLY_LEN;
        float wetg = (f == SMACK_FX_DELAY) ? 0.75f
                   : (f == SMACK_FX_COMB) ? 0.90f : 0.85f;
        float dryg = (f == SMACK_FX_COMB) ? 0.45f : 0.6f;
        *l = *l * dryg + dl * wetg;
        *r = *r * dryg + dr * wetg;
    } else {
        ln->dly_slice = -1;
    }

    if (f == SMACK_FX_RINGMOD) {
        /* sine ring mod. 0-5 fixed 50/110/220/440/880/1760 Hz; 6 = chirp
         * up (60 -> 960 Hz over the slice), 7 = the same chirp downward.
         * Chirp phase is the exact integral, so the sweep is click-free. */
        int rv = fp < 0 ? 0 : (fp > 7 ? 7 : fp);
        /* depth = tune trim: +/- one octave around the variant's pitch */
        double trim = (fp2 >= 0)
            ? pow(2.0, ((double)fp2 - 50.0) / 50.0) : 1.0;
        double phw;
        if (rv <= 5) {
            static const double rmf[6] = { 50.0, 110.0, 220.0, 440.0, 880.0, 1760.0 };
            phw = 6.2831853 * rmf[rv] * trim * p / (double)SMACK_SR;
        } else {
            double t = p / sf;
            if (rv == 7) t = 1.0 - t;
            const double k = 2.7725887;             /* ln(2^4): 4 octaves */
            phw = 6.2831853 * 60.0 * trim * (sf / (double)SMACK_SR)
                * (exp(k * t) - 1.0) / k;
        }
        float m = sinf((float)phw);
        *l *= m;
        *r *= m;
    }

    if (f == SMACK_FX_DIST) {
        /* variants 4-7 = the same four shapes, driven ~4x hotter;
         * depth overrides with a continuous drive 1 .. 15 */
        float drive = (fp2 >= 0)
            ? 1.0f + 14.0f * (float)fp2 / 100.0f
            : 2.5f + (float)(fp & 3) + (((fp & 7) >= 4) ? 4.0f : 0.0f);
        for (int ch = 0; ch < 2; ch++) {
            float x = (ch ? *r : *l) / 32768.0f * drive;
            switch (fp & 3) {
            case 0: /* soft cubic clip */
                if (x > 1.0f) x = 1.0f; else if (x < -1.0f) x = -1.0f;
                x = 1.5f * x - 0.5f * x * x * x;
                break;
            case 1: /* hard clip */
                if (x > 0.7f) x = 0.7f; else if (x < -0.7f) x = -0.7f;
                x *= 1.4f;
                break;
            case 2: /* foldback */
                while (x > 1.0f || x < -1.0f) {
                    if (x > 1.0f) x = 2.0f - x;
                    if (x < -1.0f) x = -2.0f - x;
                }
                break;
            default: /* gnash: coarse quantize + clip */
                x = floorf(x * 6.0f) / 6.0f;
                if (x > 0.9f) x = 0.9f; else if (x < -0.9f) x = -0.9f;
                break;
            }
            if (ch) *r = x * 24000.0f; else *l = x * 24000.0f;
        }
    }

    if (f == SMACK_FX_PHASER) {
        if (ln->ph_slice != oslice) {
            memset(ln->ph_x, 0, sizeof(ln->ph_x));
            memset(ln->ph_y, 0, sizeof(ln->ph_y));
            ln->ph_slice = oslice;
        }
        double t = p / sf;
        float gph;
        switch (fp & 7) {
        case 0:  gph = 0.15f + 0.7f * (float)t; break;      /* sweep up */
        case 1:  gph = 0.85f - 0.7f * (float)t; break;      /* sweep down */
        case 2:  gph = 0.5f + 0.35f * sinf((float)(6.2831853 * 2.0 * t)); break;
        case 3:  gph = 0.5f + 0.35f * sinf((float)(6.2831853 * 6.0 * t)); break;
        case 4:  gph = 0.5f + 0.45f * sinf((float)(6.2831853 * 1.0 * t)); break; /* deep slow */
        case 5:  gph = 0.55f; break;                        /* static notch mid */
        case 6:  gph = 0.25f; break;                        /* static notch low */
        default: gph = 0.85f; break;                        /* static notch high */
        }
        /* depth = notch prominence: subtle swirl .. full vacuum */
        float phmix = (fp2 >= 0) ? 0.15f + 0.7f * (float)fp2 / 100.0f : 0.5f;
        for (int ch = 0; ch < 2; ch++) {
            float x = (ch ? *r : *l);
            float v = x;
            for (int st = 0; st < 4; st++) {
                float yn = -gph * v + ln->ph_x[ch][st] + gph * ln->ph_y[ch][st];
                ln->ph_x[ch][st] = v;
                ln->ph_y[ch][st] = yn;
                v = yn;
            }
            if (ch) *r = x * (1.0f - phmix) + v * phmix;
            else    *l = x * (1.0f - phmix) + v * phmix;
        }
    } else {
        ln->ph_slice = -1;
    }

    if (f == SMACK_FX_VERB) {
        /* mono Schroeder burst: fed only while this slice plays; the network
         * keeps ringing between passes and decays on its own */
        static const int verb_off[4] = { 0, 673, 1386, 2152 };
        float feed = (*l + *r) * 0.5f * 0.55f;
        /* 0-3 unchanged (rolled range); 4-7 stretch decay toward 0.96 */
        int vv = fp & 7;
        float fb = (vv <= 3) ? 0.68f + 0.05f * (float)vv
                             : 0.83f + 0.033f * (float)(vv - 3);
        /* depth = tail: continuous decay override 0.5 .. 0.965 */
        if (fp2 >= 0) fb = 0.5f + 0.465f * (float)fp2 / 100.0f;
        float acc = 0.0f;
        for (int c = 0; c < 4; c++) {
            float *b = ln->verb + verb_off[c];
            uint32_t i = ln->vb_ci[c];
            float y = b[i];
            b[i] = feed + y * fb;
            ln->vb_ci[c] = (i + 1u < (uint32_t)verb_comb_len[c]) ? i + 1u : 0u;
            acc += y;
        }
        acc *= 0.25f;
        float *ap = ln->verb + (673 + 713 + 766 + 813);
        uint32_t j = ln->vb_ai;
        float apy = ap[j];
        float apout = apy - 0.5f * acc;
        ap[j] = acc + 0.5f * apy;
        ln->vb_ai = (j + 1u < VERB_AP_LEN) ? j + 1u : 0u;
        *l = *l * 0.65f + apout * 0.9f;
        *r = *r * 0.65f + apout * 0.9f;
    }

    *l *= g * gl * gedge;
    *r *= g * gr * gedge;

    if (f == SMACK_FX_CRUSH) {
        /* depth = bit destruction: fp2 0 subtle .. 100 pulverized */
        int sh = (fp2 >= 0) ? 1 + (fp2 * 8) / 101 : 5;
        float step = (float)(1u << sh);
        *l = floorf(*l / step) * step;
        *r = floorf(*r / step) * step;
    }

    /* per-slice mix: blend the effected slice against the same source
     * slice untouched (pattern order kept — mix controls intensity).
     * The dry tap gets the edge fades but none of the effect gains. */
    if (use_pattern && f != SMACK_FX_NONE && fmix >= 0 && fmix < 100 &&
        src_dry >= 0.0) {
        float dl, dr;
        ring_read_lerp(s, src_dry, ch, &dl, &dr);
        float mm = (float)fmix / 100.0f;
        *l = *l * mm + dl * gedge * (1.0f - mm);
        *r = *r * mm + dr * gedge * (1.0f - mm);
    }
}

/* ------------------------------------------------------------------ */
/*  BPM detection                                                      */
/* ------------------------------------------------------------------ */

static void det_begin(smack_t *s) {
    uint64_t avail = s->written_total;
    if (avail > (uint64_t)(DET_SECONDS * SMACK_SR)) avail = DET_SECONDS * SMACK_SR;
    if (avail < (uint64_t)(2 * SMACK_SR)) { /* need >= 2 s of material */
        s->det_bpm = 0.0f;
        s->det_active = 0;
        return;
    }
    s->det_total = (uint32_t)avail - ((uint32_t)avail % DET_HOP);
    s->det_start = (s->ring_w + SMACK_RING_FRAMES - s->det_total) % SMACK_RING_FRAMES;
    s->det_pos = 0;
    s->det_acc = 0.0f;
    s->det_acc_n = 0;
    memset(s->det_env, 0, sizeof(s->det_env));
    s->det_active = 1;
}

/* Envelope built — onset strength + autocorrelation + octave bias. */
static void det_finish(smack_t *s) {
    int bins = (int)(s->det_total / DET_HOP);
    if (bins > DET_BINS) bins = DET_BINS;
    const double bin_hz = (double)SMACK_SR / (double)DET_HOP;

    /* half-wave rectified envelope difference = onset strength */
    float onset[DET_BINS];
    float total = 0.0f;
    onset[0] = 0.0f;
    for (int k = 1; k < bins; k++) {
        float d = s->det_env[k] - s->det_env[k - 1];
        onset[k] = d > 0.0f ? d : 0.0f;
        total += onset[k];
    }
    if (total <= 0.0f) { s->det_bpm = 0.0f; return; }
    float mean = total / (float)(bins - 1);
    for (int k = 0; k < bins; k++) {            /* remove DC so silence */
        onset[k] -= mean;                       /* between hits counts */
    }

    int lag_min = (int)(60.0 / DET_BPM_MAX * bin_hz);        /* 180 BPM */
    int lag_max = (int)(60.0 / DET_BPM_MIN * bin_hz) + 1;    /* 60 BPM  */
    if (lag_max >= bins / 2) lag_max = bins / 2 - 1;
    if (lag_min < 2 || lag_max <= lag_min) { s->det_bpm = 0.0f; return; }

    float best_r = 0.0f, mean_abs = 0.0f;
    int best_lag = 0;
    float r_at[DET_BINS / 2];
    for (int lag = lag_min; lag <= lag_max; lag++) {
        float r = 0.0f;
        for (int k = lag; k < bins; k++) r += onset[k] * onset[k - lag];
        r_at[lag] = r;
        mean_abs += r < 0.0f ? -r : r;
        if (r > best_r) { best_r = r; best_lag = lag; }
    }
    mean_abs /= (float)(lag_max - lag_min + 1);
    /* confidence gate: aperiodic material (pads, drones, silence) makes a
     * flat autocorrelation — report "none" instead of a random number */
    if (best_lag == 0 || best_r <= 0.0f || best_r < 2.0f * mean_abs) {
        s->det_bpm = 0.0f;
        return;
    }

    /* parabolic refinement around the peak */
    double lag_f = (double)best_lag;
    if (best_lag > lag_min && best_lag < lag_max) {
        double a = r_at[best_lag - 1], b = r_at[best_lag], c = r_at[best_lag + 1];
        double den = a - 2.0 * b + c;
        if (den < -1e-9) lag_f += 0.5 * (a - c) / den;
    }
    double bpm = 60.0 * bin_hz / lag_f;

    /* octave disambiguation: prefer the candidate nearest the project
     * tempo (log distance) — busy material loves half/double answers */
    double ref = 120.0;
    if (s->host && s->host->get_bpm) {
        float b = s->host->get_bpm();
        if (b >= 20.0f && b <= 999.0f) ref = (double)b;
    }
    double cand[3] = { bpm, bpm * 2.0, bpm * 0.5 };
    double best_c = bpm, best_d = 1e9;
    for (int i = 0; i < 3; i++) {
        if (cand[i] < 50.0 || cand[i] > 200.0) continue;
        double d = fabs(log(cand[i] / ref));
        if (d < best_d) { best_d = d; best_c = cand[i]; }
    }
    s->det_bpm = (float)best_c;
    s->bpm_override = (float)best_c;   /* auto-apply for free-run capture */
}

/* Consume up to DET_FRAMES_PER_BLOCK ring frames into the envelope. */
static void det_step(smack_t *s) {
    if (!s->det_active) return;
    uint32_t budget = DET_FRAMES_PER_BLOCK;
    while (budget > 0 && s->det_pos < s->det_total) {
        uint32_t idx = (s->det_start + s->det_pos) % SMACK_RING_FRAMES;
        float l = (float)s->ring[idx * 2];
        float r = (float)s->ring[idx * 2 + 1];
        s->det_acc += (l < 0 ? -l : l) + (r < 0 ? -r : r);
        if (++s->det_acc_n == DET_HOP) {
            uint32_t bin = s->det_pos / DET_HOP;
            if (bin < DET_BINS) s->det_env[bin] = s->det_acc;
            s->det_acc = 0.0f;
            s->det_acc_n = 0;
        }
        s->det_pos++;
        budget--;
    }
    if (s->det_pos >= s->det_total) {
        s->det_active = 0;
        det_finish(s);
        s->edit_rev++; /* browser editor refreshes its BPM readout */
    }
}

/* Pad-play repeat period in loop-domain frames, derived from the pattern
 * grid (slice_frames is fixed at roll time, so this stays consistent with
 * what the steps show even if the tempo drifts after capture). */
static double pad_rate_frames(const smack_t *s) {
    if (s->slice_frames <= 0.0) return 0.0;
    double hsf = s->slice_frames / (double)slice_hs_table[s->slice_res_idx];
    return hsf * (double)pad_rate_hs[s->pad_rate_idx];
}

static void record_ring_frame(smack_t *s, int16_t l, int16_t r,
                              uint64_t global_frame) {
    if (s->state == SMACK_LOOPING) {
        uint32_t protected_start = s->loop_available > 0
                                 ? s->loop_history_start : s->loop_start;
        uint32_t gap = (protected_start + SMACK_RING_FRAMES - s->ring_w)
                       % SMACK_RING_FRAMES;
        if (gap <= 2048) return;
    }
    s->ring[s->ring_w * 2] = l;
    s->ring[s->ring_w * 2 + 1] = r;
    s->ring_w = (s->ring_w + 1) % SMACK_RING_FRAMES;
    s->written_total++;
    s->ring_last_global = global_frame;
}

void smack_process(smack_t *s, const int16_t *in, int16_t *out, int frames) {
    if (!s) return;

    det_step(s);   /* incremental BPM analysis; no-op unless scanning */

    if (s->state == SMACK_ARMED && s->arm_start_flag) {
        s->arm_start_flag = 0;
        begin_record(s);
    }

    double play_inc = loop_playback_increment(s);

    for (int n = 0; n < frames; n++) {
        float inl = (float)in[n * 2], inr = (float)in[n * 2 + 1];
        record_ring_frame(s, in[n * 2], in[n * 2 + 1],
                          s->global_frames + (uint64_t)n);

        if (s->state != SMACK_LOOPING || s->transport_paused) {
            /* record + pass through */
            if (s->state == SMACK_RECORDING && s->rec_remaining > 0) {
                if (--s->rec_remaining == 0) finish_record(s);
            }
            out[n * 2]     = s->monitor ? clip16(inl) : 0;
            out[n * 2 + 1] = s->monitor ? clip16(inr) : 0;
        } else {
            /* quantized A/B switch */
            if (s->ab_pending >= 0) {
                int apply = 0;
                if (s->quantize_mode == 0) apply = 1;
                else if (s->quantize_mode == 1) {
                    double p = fmod(s->play_pos, s->slice_frames);
                    if (p < 1.0) apply = 1;
                } else if (s->play_pos < 1.0) apply = 1;
                if (apply) { s->ab = s->ab_pending; s->ab_pending = -1; s->edit_rev++; }
            }

            /* pad-play: (re)fire the top held cell on each rate boundary.
             * play_pos advances by exactly 1.0 per frame, so each boundary
             * window [k*rf, k*rf+1) contains exactly one frame. */
            if (s->pad_held > 0) {
                double rf = pad_rate_frames(s);
                if (rf >= 1.0 && fmod(s->play_pos, rf) < 1.0) {
                    s->trig_active = 1;
                    s->trig_step   = s->pad_note_stk[s->pad_held - 1]
                                     % (s->n_slices > 0 ? s->n_slices : 1);
                    s->trig_gain   = sqrtf((float)s->pad_vel_stk[s->pad_held - 1] / 127.0f);
                    s->trig_pos    = 0.0;
                }
            }
            s->trig_render = (s->trig_active && s->trig_pos < s->slice_frames) ? 1 : 0;

            float ll, rr;
            if (s->trig_render) {
                /* pad-play owns the output while a cell is held: render the
                 * cell through the pattern path (velocity-scaled), exactly
                 * once per frame so per-effect runtimes advance normally. */
                if (!s->chan_mode) {
                    render_lane(s, &s->lane[0], -1, 1, &ll, &rr);
                } else {
                    float l0, r0, l1, r1;
                    render_lane(s, &s->lane[0], 0, 1, &l0, &r0);
                    render_lane(s, &s->lane[1], 1, 1, &l1, &r1);
                    ll = l0 * s->pan_gain[0][0] + l1 * s->pan_gain[1][0];
                    rr = r0 * s->pan_gain[0][1] + r1 * s->pan_gain[1][1];
                }
                ll *= s->trig_gain;
                rr *= s->trig_gain;
            } else if (s->trig_active) {
                ll = rr = 0.0f; /* between repeats: silence, not the loop */
            } else if (s->hw_input) {
                /* Input builds: three layers. The loop is ALWAYS audible;
                 * `wet` blends its clean tap against the pattern render
                 * (0 = unaffected loop, 100 = fully effected); the A side
                 * hard-punches to clean; the live input rides Monitor. */
                float w = s->ab ? s->wet : 0.0f;
                if (s->punch_fx > 0) w = 1.0f;       /* punch = full effect */
                else if (s->punch_fx == 0) w = 0.0f; /* punch clean */
                float cl = 0.0f, cr = 0.0f, pl = 0.0f, pr = 0.0f;
                if (w < 1.0f)
                    render_lane(s, &s->lane[0], -1, 0, &cl, &cr);
                if (w > 0.0f) {
                    if (!s->chan_mode) {
                        render_lane(s, &s->lane[0], -1, 1, &pl, &pr);
                    } else {
                        float l0, r0, l1, r1;
                        render_lane(s, &s->lane[0], 0, 1, &l0, &r0);
                        render_lane(s, &s->lane[1], 1, 1, &l1, &r1);
                        pl = l0 * s->pan_gain[0][0] + l1 * s->pan_gain[1][0];
                        pr = r0 * s->pan_gain[0][1] + r1 * s->pan_gain[1][1];
                    }
                }
                ll = cl * (1.0f - w) + pl * w;
                rr = cr * (1.0f - w) + pr * w;
            } else {
                /* Chain/master FX build: classic insert behavior — wet
                 * crossfades the loop against the upstream (dry) signal. */
                if (!s->chan_mode || !s->ab) {
                    render_lane(s, &s->lane[0], -1, -1, &ll, &rr);
                } else {
                    float l0, r0, l1, r1;
                    render_lane(s, &s->lane[0], 0, 1, &l0, &r0);
                    render_lane(s, &s->lane[1], 1, 1, &l1, &r1);
                    ll = l0 * s->pan_gain[0][0] + l1 * s->pan_gain[1][0];
                    rr = r0 * s->pan_gain[0][1] + r1 * s->pan_gain[1][1];
                }
            }
            if (s->hw_input) {
                float dry = s->monitor ? 1.0f : 0.0f;
                out[n * 2]     = clip16(ll + inl * dry);
                out[n * 2 + 1] = clip16(rr + inr * dry);
            } else {
                float dry = s->monitor ? (1.0f - s->wet) : 0.0f;
                out[n * 2]     = clip16(ll * s->wet + inl * dry);
                out[n * 2 + 1] = clip16(rr * s->wet + inr * dry);
            }

            s->play_pos += play_inc;
            if (s->play_pos >= (double)s->loop_len)
                s->play_pos = fmod(s->play_pos, (double)s->loop_len);
            if (s->trig_active) s->trig_pos += 1.0;
        }
    }
    s->global_frames += (uint64_t)frames;
}

/* ------------------------------------------------------------------ */
/*  Params                                                             */
/* ------------------------------------------------------------------ */

static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* Resize an already captured loop around its fixed endpoint. Lengthening
 * reveals older retained ring history; shortening keeps the most-recent part.
 * Playback stays at the same distance from that endpoint (modulo the new
 * length), preserving the current sample whenever it remains in the window. */
static void resize_live_loop(smack_t *s) {
    uint32_t old_len, new_ticks;
    if (!s || s->state != SMACK_LOOPING || s->loop_len == 0) return;

    /* Lengthening is bounded by the same half-ring rule as capture, so a big
     * LENGTH turn cannot grow the loop into the recorder's space. */
    fit_loop_len_idx(s);

    old_len   = s->loop_len;
    new_ticks = (uint32_t)(loop_len_hs_table[s->loop_len_idx] * 3);
    double source_frames_per_tick = s->loop_frames_per_tick;
    if (source_frames_per_tick <= 0.0 && s->loop_clock_ticks > 0)
        source_frames_per_tick = (double)old_len / (double)s->loop_clock_ticks;
    if (source_frames_per_tick <= 0.0) source_frames_per_tick = frames_per_tick_now(s);

    double wanted = source_frames_per_tick * (double)new_ticks;
    if (wanted < 256.0) wanted = 256.0;
    if (wanted > (double)max_loop_frames()) wanted = (double)max_loop_frames();
    uint32_t new_len = (uint32_t)(wanted + 0.5);
    uint32_t available = s->loop_available > 0 ? s->loop_available : old_len;
    if (new_len > available) new_len = available;
    if (new_len == 0) return;

    uint32_t loop_end = (s->loop_start + old_len) % SMACK_RING_FRAMES;
    double old_pos = fmod(s->play_pos, (double)old_len);
    if (old_pos < 0.0) old_pos += old_len;
    double from_end = (double)old_len - old_pos;
    double remainder = fmod(from_end, (double)new_len);

    s->loop_start = (loop_end + SMACK_RING_FRAMES - new_len) % SMACK_RING_FRAMES;
    s->loop_len = new_len;
    s->loop_clock_ticks = new_ticks;
    s->loop_frames_per_tick = (double)new_len / (double)new_ticks;
    s->play_pos = remainder < 0.000001 ? 0.0 : (double)new_len - remainder;
    s->trig_active = 0;
    s->trig_render = 0;
    s->trig_pos = 0.0;
}

/* Pin (f >= 0: 0 = clean/mute, 1.. = specific effect) or unlock (f < 0) a
 * slice on one lane. Pinning a different effect gets that effect's
 * canonical parameter; re-pinning the current one keeps the rolled flavor. */
/* Extended fx token "f[:p[:p2[:m]]]": p = variant, p2 = per-effect depth
 * 0-100, m = per-slice mix 0-100 (-1 anywhere = engine default). Returns
 * the number of fields parsed (0 = no number at all); absent fields leave
 * the out-params untouched, so callers pre-load their defaults. */
static int parse_fx_token(const char *val, long *f_out, int *p, int *p2, int *m) {
    char *end;
    long f = strtol(val, &end, 10);
    if (end == val) return 0;
    *f_out = f;
    int n = 1;
    for (int k = 0; k < 3 && *end == ':'; k++) {
        long v = strtol(end + 1, &end, 10);
        n++;
        if (k == 0)      *p  = (int)v;
        else if (k == 1) *p2 = (int)v;
        else             *m  = (int)v;
    }
    return n;
}

static int8_t clamp_pct(int v) {  /* depth/mix: 0-100, -1 = default */
    return (int8_t)(v < 0 ? -1 : (v > 100 ? 100 : v));
}

static void set_lock(smack_t *s, smack_lane_t *ln, int i, int f) {
    if (f < 0) { /* unlock: re-roll restores the seeded value */
        ln->locked[i] = 0;
        if (s->state == SMACK_LOOPING) roll_pattern(s);
    } else {
        f = clampi(f, 0, SMACK_FX_COUNT - 1);
        if (ln->fx[i] != (uint8_t)f) ln->fxp[i] = default_fxp(f);
        ln->fx[i] = (uint8_t)f;
        ln->locked[i] = 1;
    }
}

/* Soft assign: change a slice's effect WITHOUT pinning it — the next
 * re-roll (or any pattern-shaping knob) replaces it. On a pinned slice
 * the pin follows the new effect. */
/* set_slice_<i> value "f" or "f:p": soft-assign effect f (does not survive
 * re-roll), optionally with an explicit parameter (variant palette pads). */
static void set_soft(smack_t *s, smack_lane_t *ln, int i, const char *val) {
    (void)s;
    long f = -1;
    int p = 12345, p2 = 12345, m = 12345;      /* sentinels: field absent */
    int n = parse_fx_token(val, &f, &p, &p2, &m);
    if (n < 1 || f < 0) return;
    f = clampi((int)f, 0, SMACK_FX_COUNT - 1);
    if (n >= 2)
        ln->fxp[i] = (int8_t)clampi(p, -128, 127);
    else if (ln->fx[i] != (uint8_t)f)
        ln->fxp[i] = default_fxp((int)f);
    if (n >= 3) ln->fxp2[i] = clamp_pct(p2);
    else if (ln->fx[i] != (uint8_t)f) ln->fxp2[i] = -1;
    if (n >= 4) ln->fxmix[i] = clamp_pct(m);
    else if (ln->fx[i] != (uint8_t)f) ln->fxmix[i] = -1;
    ln->fx[i] = (uint8_t)f;
}

/* lock_slice_<i> value "f[:p[:p2[:m]]]": pin effect f, optionally with an
 * explicit parameter / depth / mix (the web editor's per-slice tweak
 * path). Bare "f" keeps the set_lock semantics the pad UIs rely on;
 * absent depth/mix fields leave existing per-slice values untouched. */
static void lock_from_str(smack_t *s, smack_lane_t *ln, int i, const char *val) {
    long f = -1;
    int p = 12345, p2 = 12345, m = 12345;
    int n = parse_fx_token(val, &f, &p, &p2, &m);
    if (n < 1) return;
    set_lock(s, ln, i, (int)f);
    if (f < 0) return;
    if (n >= 2) ln->fxp[i] = (int8_t)clampi(p, -128, 127);
    if (n >= 3) ln->fxp2[i] = clamp_pct(p2);
    if (n >= 4) ln->fxmix[i] = clamp_pct(m);
}

/* snprintf returns the WOULD-HAVE-WRITTEN length — on truncation a raw
 * `n += snprintf(...)` pushes n past buf_len, and the next append writes
 * out of bounds (buf + n) with an underflowed size. Every state-JSON
 * append goes through this clamp instead. */
static int nclamp(int n, int buf_len) {
    return (n < 0) ? 0 : (n >= buf_len ? buf_len - 1 : n);
}

/* Append ,"key":"v0,v1,..." from one lane array (0 fx, 1 fxp, 2 order) —
 * read-only display fields in the state JSON for the browser editor. */
static int csv_lane(char *buf, int n, int buf_len, const char *key,
                    const smack_lane_t *ln, int which, int cnt) {
    if (n < 0 || n >= buf_len - 16) return n;
    n = nclamp(n + snprintf(buf + n, (size_t)(buf_len - n), ",\"%s\":\"", key), buf_len);
    for (int i = 0; i < cnt && n < buf_len - 12; i++)
        n = nclamp(n + snprintf(buf + n, (size_t)(buf_len - n), "%s%d", i ? "," : "",
                      which == 0 ? (int)ln->fx[i] :
                      which == 1 ? (int)ln->fxp[i] :
                      which == 3 ? (int)ln->fxp2[i] :
                      which == 4 ? (int)ln->fxmix[i] : (int)ln->order[i]), buf_len);
    if (n < buf_len - 2) n += snprintf(buf + n, (size_t)(buf_len - n), "\"");
    return n;
}

/* Parse a locks string ("i:f:p,i:f:p,..."; bare i:f pairs from older
 * presets fall back to the canonical parameter). lk points at the JSON
 * key match or NULL; skip = strlen of the key prefix incl. quote. */
/* Parse "f,f:p,f,..." (plain param value or inside the state blob, where it
 * is terminated by '"'). Each token is an fx code with an optional per-pad
 * parameter override; duplicates and subsets are legal. Applied only if all
 * SMACK_PALETTE_SLOTS tokens parse cleanly, so a truncated blob or garbage
 * never half-applies. */
static void parse_palette(smack_t *s, const char *p) {
    if (!p) return;
    uint8_t tf[SMACK_PALETTE_SLOTS];
    int8_t  tp[SMACK_PALETTE_SLOTS];
    uint8_t th[SMACK_PALETTE_SLOTS];
    int8_t  t2[SMACK_PALETTE_SLOTS];
    int8_t  tm[SMACK_PALETTE_SLOTS];
    int i = 0;
    while (*p && *p != '"' && i < SMACK_PALETTE_SLOTS) {
        char *end;
        long v = strtol(p, &end, 10);
        if (end == p || v < 0 || v >= SMACK_FX_COUNT) return;
        tf[i] = (uint8_t)v;
        th[i] = 0;
        tp[i] = 0;
        t2[i] = -1;
        tm[i] = -1;
        p = end;
        if (*p == ':') {                    /* :p */
            long pv = strtol(p + 1, &end, 10);
            if (end == p + 1) return;
            th[i] = 1;
            tp[i] = (int8_t)clampi((int)pv, -128, 127);
            p = end;
            if (*p == ':') {                /* :p2 */
                pv = strtol(p + 1, &end, 10);
                if (end == p + 1) return;
                t2[i] = clamp_pct((int)pv);
                p = end;
                if (*p == ':') {            /* :m */
                    pv = strtol(p + 1, &end, 10);
                    if (end == p + 1) return;
                    tm[i] = clamp_pct((int)pv);
                    p = end;
                }
            }
        }
        i++;
        if (*p == ',') p++; else break;
    }
    if (i != SMACK_PALETTE_SLOTS) return;
    memcpy(s->palette, tf, sizeof(tf));
    memcpy(s->palette_fxp, tp, sizeof(tp));
    memcpy(s->palette_hasp, th, sizeof(th));
    memcpy(s->palette_fxp2, t2, sizeof(t2));
    memcpy(s->palette_mix, tm, sizeof(tm));
}

static int palette_csv(const smack_t *s, char *buf, int buf_len) {
    int n = 0;
    for (int i = 0; i < SMACK_PALETTE_SLOTS && n < buf_len - 18; i++) {
        n += snprintf(buf + n, (size_t)(buf_len - n), "%s%d",
                      i ? "," : "", s->palette[i]);
        if (s->palette_hasp[i]) {
            n += snprintf(buf + n, (size_t)(buf_len - n), ":%d",
                          s->palette_fxp[i]);
            if (s->palette_fxp2[i] >= 0 || s->palette_mix[i] >= 0)
                n += snprintf(buf + n, (size_t)(buf_len - n), ":%d:%d",
                              s->palette_fxp2[i], s->palette_mix[i]);
        }
    }
    return n;
}

static void parse_locks(smack_lane_t *ln, const char *lk, int skip) {
    if (!lk) return;
    lk += skip;
    while (*lk && *lk != '"') {
        char *end;
        long i = strtol(lk, &end, 10);
        if (end == lk || *end != ':') break;
        long f = strtol(end + 1, &end, 10);
        int has_p = 0;
        long p = 0, p2 = -1, m = -1;
        if (*end == ':') { p = strtol(end + 1, &end, 10); has_p = 1; }
        if (*end == ':') { p2 = strtol(end + 1, &end, 10); }
        if (*end == ':') { m = strtol(end + 1, &end, 10); }
        if (i >= 0 && i < SMACK_MAX_SLICES) {
            ln->locked[i] = 1;
            ln->fx[i] = (uint8_t)clampi((int)f, 0, SMACK_FX_COUNT - 1);
            ln->fxp[i] = has_p ? (int8_t)clampi((int)p, -128, 127)
                               : default_fxp(ln->fx[i]);
            ln->fxp2[i]  = clamp_pct((int)p2);
            ln->fxmix[i] = clamp_pct((int)m);
        }
        if (*end == ',') lk = end + 1; else break;
    }
}

void smack_set_param(smack_t *s, const char *key, const char *val) {
    if (!s || !key || !val) return;
    if (!strcmp(key, "rui_set")) {
        const char *separator = strchr(val, ':');
        size_t key_len = separator ? (size_t)(separator - val) : 0;
        char routed_key[32];
        if (!separator || key_len == 0 || key_len >= sizeof(routed_key)) return;
        memcpy(routed_key, val, key_len);
        routed_key[key_len] = '\0';
        if (!strcmp(routed_key, "rui_set")) return;
        smack_set_param(s, routed_key, separator + 1);
        return;
    }
    if (!strcmp(key, "loop_len")) {
        int next = clampi(atoi(val), 0, LOOP_LEN_COUNT - 1);
        if (next != s->loop_len_idx) {
            s->loop_len_idx = next;
            if (s->state == SMACK_LOOPING) {
                resize_live_loop(s);
                roll_pattern(s);
            }
        }
    } else if (!strcmp(key, "slice_res")) {
        s->slice_res_idx = clampi(atoi(val), 0, SLICE_RES_COUNT - 1);
        if (s->state == SMACK_LOOPING) roll_pattern(s);
    } else if (!strcmp(key, "fx_density")) {
        s->fx_density = fminf(1.0f, fmaxf(0.0f, (float)atof(val) / 100.0f));
        if (s->state == SMACK_LOOPING) roll_pattern(s);
    } else if (!strcmp(key, "order_density")) {
        s->order_density = fminf(1.0f, fmaxf(0.0f, (float)atof(val) / 100.0f));
        if (s->state == SMACK_LOOPING) roll_pattern(s);
    } else if (!strcmp(key, "pitch_range")) {
        s->pitch_range = clampi(atoi(val), 1, 24);
    } else if (!strcmp(key, "wet")) {
        s->wet = fminf(1.0f, fmaxf(0.0f, (float)atof(val) / 100.0f));
    } else if (!strcmp(key, "ab")) {
        int v = atoi(val) ? 1 : 0;
        if (s->state == SMACK_LOOPING && s->quantize_mode != 0) s->ab_pending = v;
        else s->ab = v;
    } else if (!strcmp(key, "quantize")) {
        s->quantize_mode = clampi(atoi(val), 0, 2);
    } else if (!strcmp(key, "seed")) {
        /* Seed is the pattern's ID: dialing it browses patterns directly,
         * and returning to a number restores that exact pattern. */
        s->seed = (uint32_t)strtoul(val, NULL, 10);
        s->roll_nonce = 0;
        if (s->state == SMACK_LOOPING) roll_pattern(s);
    } else if (!strcmp(key, "reroll")) {
        if (trig_active(val)) {
            /* advance the hidden nonce, never the seed: the shadow UI's knob
             * cache reads a param once per touch, so core-side seed mutation
             * makes the next Seed-knob edit jump from a stale baseline */
            s->roll_nonce = s->roll_nonce * 1664525u + 1013904223u;
            if (!s->roll_nonce) s->roll_nonce = 1;
            if (s->state == SMACK_LOOPING) roll_pattern(s);
        }
    } else if (!strcmp(key, "capture")) {
        if (trig_active(val)) {
            capture_retro(s);
            s->transport_paused = 0; /* manual grab plays even when stopped */
        }
    } else if (!strcmp(key, "arm")) {
        if (trig_active(val) &&
            (s->state == SMACK_IDLE || s->state == SMACK_LOOPING)) {
            s->state = SMACK_ARMED;
            s->transport_paused = 0;
            s->arm_start_flag = !s->clock_running; /* free-run: start next block */
        }
    } else if (!strcmp(key, "clear")) {
        if (trig_active(val)) {
            s->state = SMACK_IDLE;
            s->ab_pending = -1;
            s->transport_paused = 0;
            memset(s->lane[0].locked, 0, sizeof(s->lane[0].locked));
            memset(s->lane[1].locked, 0, sizeof(s->lane[1].locked));
        }
    } else if (!strcmp(key, "channel_mode")) {
        int m = atoi(val) ? 1 : 0;
        if (m != s->chan_mode) {
            s->chan_mode = m;
            if (s->state == SMACK_LOOPING) roll_pattern(s); /* populate lane 1 */
        }
    } else if (!strcmp(key, "pan_l")) {
        s->pan_l = clampi(atoi(val), 0, 100);
        update_pan_gains(s);
    } else if (!strcmp(key, "pan_r")) {
        s->pan_r = clampi(atoi(val), 0, 100);
        update_pan_gains(s);
    } else if (!strcmp(key, "palette")) {
        parse_palette(s, val);
    } else if (!strcmp(key, "pad_play")) {
        s->pad_play = atoi(val) ? 1 : 0;
        if (!s->pad_play) { s->pad_held = 0; s->trig_active = 0; }
    } else if (!strcmp(key, "pad_rate")) {
        s->pad_rate_idx = clampi(atoi(val), 0, PAD_RATE_COUNT - 1);
    } else if (!strcmp(key, "pad_note")) {
        /* UI/web trigger: "note:vel" fires, vel 0 releases. Bypasses the
         * pad_play gate — this only arrives from deliberate UI gestures. */
        int note = atoi(val);
        const char *c = strchr(val, ':');
        int vel = c ? atoi(c + 1) : 0;
        if (note >= 0 && note < 128) {
            if (vel > 0) pad_note_on(s, note, clampi(vel, 1, 127));
            else pad_note_off(s, note);
        }
    } else if (!strcmp(key, "monitor")) {
        s->monitor = atoi(val) ? 1 : 0;
    } else if (!strcmp(key, "hw_input")) {
        s->hw_input = atoi(val) ? 1 : 0; /* set once by the gen wrapper */
    } else if (!strcmp(key, "transport")) {
        s->follow_transport = atoi(val) ? 0 : 1; /* 0 Follow, 1 Free */
        if (!s->follow_transport) s->transport_paused = 0;
    } else if (!strcmp(key, "state")) {
        /* preset/autosave restore: settings + slice locks, never audio */
        int old_loop_len_idx = s->loop_len_idx;
        s->loop_len_idx  = clampi(json_int(val, "loop_len", s->loop_len_idx), 0, LOOP_LEN_COUNT - 1);
        s->slice_res_idx = clampi(json_int(val, "slice_res", s->slice_res_idx), 0, SLICE_RES_COUNT - 1);
        s->fx_density    = (float)clampi(json_int(val, "fx_density", (int)(s->fx_density * 100.0f)), 0, 100) / 100.0f;
        s->order_density = (float)clampi(json_int(val, "order_density", (int)(s->order_density * 100.0f)), 0, 100) / 100.0f;
        s->pitch_range   = clampi(json_int(val, "pitch_range", s->pitch_range), 1, 24);
        s->wet           = (float)clampi(json_int(val, "wet", (int)(s->wet * 100.0f)), 0, 100) / 100.0f;
        s->ab            = json_int(val, "ab", s->ab) ? 1 : 0;
        s->quantize_mode = clampi(json_int(val, "quantize", s->quantize_mode), 0, 2);
        s->seed          = (uint32_t)json_int(val, "seed", (int)s->seed);
        s->roll_nonce    = (uint32_t)json_int(val, "nonce", 0);
        s->follow_transport = json_int(val, "transport", s->follow_transport ? 0 : 1) ? 0 : 1;
        s->chan_mode = json_int(val, "chan", s->chan_mode) ? 1 : 0;
        s->pan_l = clampi(json_int(val, "pan_l", s->pan_l), 0, 100);
        s->pan_r = clampi(json_int(val, "pan_r", s->pan_r), 0, 100);
        s->pad_play = json_int(val, "pp", s->pad_play) ? 1 : 0;
        s->pad_rate_idx = clampi(json_int(val, "pr", s->pad_rate_idx), 0, PAD_RATE_COUNT - 1);
        update_pan_gains(s);
        memset(s->lane[0].locked, 0, sizeof(s->lane[0].locked));
        memset(s->lane[1].locked, 0, sizeof(s->lane[1].locked));
        parse_locks(&s->lane[0], strstr(val, "\"locks\":\""), 9);
        parse_locks(&s->lane[1], strstr(val, "\"locks_r\":\""), 11);
        {   /* absent in old presets -> keep the current arrangement */
            const char *pp = strstr(val, "\"pal\":\"");
            if (pp) parse_palette(s, pp + 7);
        }
        if (s->state == SMACK_LOOPING) {
            if (s->loop_len_idx != old_loop_len_idx) resize_live_loop(s);
            roll_pattern(s);
        }
    } else if (!strcmp(key, "punch_fx")) {
        /* "f" = punch with the canonical param; "f:p[:p2[:m]]" = a variant
         * pad's punch with its full setting (pressure still bends p) */
        long f = -1;
        int p = 12345, p2 = -1, m = -1;
        int n = parse_fx_token(val, &f, &p, &p2, &m);
        if (n < 1 || f < 0 || f >= SMACK_FX_COUNT) {
            s->punch_fx = -1;
            s->punch_fxp2 = -1;
            s->punch_mix = -1;
        } else {
            s->punch_fx = (int)f;
            s->punch_fxp = (n >= 2) ? (int8_t)clampi(p, -128, 127)
                                    : default_fxp((int)f);
            s->punch_fxp2 = clamp_pct(p2);
            s->punch_mix = clamp_pct(m);
        }
    } else if (!strcmp(key, "punch_pressure")) {
        if (s->punch_fx > 0)
            s->punch_fxp = punch_map(s->punch_fx, clampi(atoi(val), 0, 127));
    } else if (!strcmp(key, "detect_bpm")) {
        if (trig_active(val) && !s->det_active) det_begin(s);
    } else if (!strcmp(key, "bpm_override")) {
        float b = (float)atof(val);
        s->bpm_override = (b >= 50.0f && b <= 200.0f) ? b : 0.0f;
    } else if (!strcmp(key, "unlock_all")) {
        /* escape hatch: drop every pin on both lanes and roll fresh */
        if (trig_active(val)) {
            memset(s->lane[0].locked, 0, sizeof(s->lane[0].locked));
            memset(s->lane[1].locked, 0, sizeof(s->lane[1].locked));
            if (s->state == SMACK_LOOPING) roll_pattern(s);
        }
    } else if (!strncmp(key, "lock_slice_r_", 13)) {
        lock_from_str(s, &s->lane[1], clampi(atoi(key + 13), 0, SMACK_MAX_SLICES - 1), val);
    } else if (!strncmp(key, "lock_slice_", 11)) {
        lock_from_str(s, &s->lane[0], clampi(atoi(key + 11), 0, SMACK_MAX_SLICES - 1), val);
    } else if (!strncmp(key, "set_slice_r_", 12)) {
        set_soft(s, &s->lane[1], clampi(atoi(key + 12), 0, SMACK_MAX_SLICES - 1), val);
    } else if (!strncmp(key, "set_slice_", 10)) {
        set_soft(s, &s->lane[0], clampi(atoi(key + 10), 0, SMACK_MAX_SLICES - 1), val);
    }
    /* Any recognized edit invalidates the browser editor's snapshot. The
     * punch pressure and pad-play note streams are the exceptions — they
     * arrive at performance rate and would turn every Remote-UI poll into
     * a full state refetch. */
    if (strcmp(key, "punch_pressure") && strcmp(key, "pad_note")) s->edit_rev++;
}

/*
 * One-decimal formatting without a float conversion.
 *
 * "%.1f" would be the obvious way to write these, and it is what this file
 * used to do. It cannot be, on the Versio: that build links newlib-nano
 * without -u _printf_float, where _printf_float is a weak reference nothing
 * defines and a "%f" conversion silently emits NOTHING. The caller gets an
 * empty string and atoi()/atof() turn it into 0, with no error anywhere.
 *
 * That is not theoretical -- it is what made play_frame read 0 forever and
 * LIVE mode never fire on any build before v0.2.0. Decomposing into integers
 * produces byte-identical output ("128.5") and cannot fail that way, so the
 * engine now formats every parameter without a float conversion and the link
 * flag is not needed at all.
 */
static int fmt_tenths(char *buf, int buf_len, float v)
{
    int t = (int)(v * 10.0f + (v < 0.0f ? -0.5f : 0.5f));
    int whole = t / 10;
    int frac  = (t < 0 ? -t : t) % 10;
    return snprintf(buf, (size_t)buf_len, "%s%d.%d",
                    (t < 0 && whole == 0) ? "-" : "", whole, frac);
}

int smack_get_param(smack_t *s, const char *key, char *buf, int buf_len) {
    if (!s || !key || !buf || buf_len < 2) return -1;
    if (!strcmp(key, "run_state")) /* machine state: 0 idle..3 looping */
        return snprintf(buf, (size_t)buf_len, "%d", (int)s->state);
    if (!strcmp(key, "loop_len"))
        return snprintf(buf, (size_t)buf_len, "%d", s->loop_len_idx);
    if (!strcmp(key, "slice_res"))
        return snprintf(buf, (size_t)buf_len, "%d", s->slice_res_idx);
    if (!strcmp(key, "fx_density"))
        return snprintf(buf, (size_t)buf_len, "%d", (int)(s->fx_density * 100.0f + 0.5f));
    if (!strcmp(key, "order_density"))
        return snprintf(buf, (size_t)buf_len, "%d", (int)(s->order_density * 100.0f + 0.5f));
    if (!strcmp(key, "pitch_range"))
        return snprintf(buf, (size_t)buf_len, "%d", s->pitch_range);
    if (!strcmp(key, "wet"))
        return snprintf(buf, (size_t)buf_len, "%d", (int)(s->wet * 100.0f + 0.5f));
    if (!strcmp(key, "ab"))
        return snprintf(buf, (size_t)buf_len, "%d", s->ab);
    if (!strcmp(key, "quantize"))
        return snprintf(buf, (size_t)buf_len, "%d", s->quantize_mode);
    if (!strcmp(key, "seed"))
        return snprintf(buf, (size_t)buf_len, "%u", s->seed);
    /* trigger params always read back as 0 so autosave never re-fires them */
    if (!strcmp(key, "capture") || !strcmp(key, "arm") ||
        !strcmp(key, "reroll") || !strcmp(key, "clear"))
        return snprintf(buf, (size_t)buf_len, "0");
    if (!strcmp(key, "transport"))
        return snprintf(buf, (size_t)buf_len, "%d", s->follow_transport ? 0 : 1);
    if (!strcmp(key, "palette"))
        return palette_csv(s, buf, buf_len);
    if (!strcmp(key, "state")) {
        /* full settings snapshot — powers slot autosave AND module presets
         * (save/recall from the shadow UI) AND the Remote-UI browser editor
         * (schwung-manager flattens this JSON into param_update messages).
         * run/nsl/mon/ps/det/pfx/pat/fxp/ord are read-only display fields;
         * the restore parser ignores them. Audio is never serialized. */
        int ps = -1;
        if (s->state == SMACK_LOOPING && s->slice_frames > 0.0) {
            ps = (int)(s->play_pos / s->slice_frames);
            if (ps >= s->n_slices) ps = s->n_slices - 1;
        }
        char pal[SMACK_PALETTE_SLOTS * 18]; /* "26:-128:100:100," worst case */
        palette_csv(s, pal, (int)sizeof(pal));
        int n = snprintf(buf, (size_t)buf_len,
            "{\"loop_len\":%d,\"slice_res\":%d,\"fx_density\":%d,"
            "\"order_density\":%d,\"pitch_range\":%d,\"wet\":%d,\"ab\":%d,"
            "\"quantize\":%d,\"seed\":%u,\"nonce\":%u,\"transport\":%d,"
            "\"chan\":%d,\"pan_l\":%d,\"pan_r\":%d,\"pp\":%d,\"pr\":%d,"
            "\"pal\":\"%s\","
            "\"run\":%d,\"nsl\":%d,\"mon\":%d,\"ps\":%d,\"det\":%d,\"pfx\":%d,"
            "\"bpmo\":%d,\"locks\":\"",
            s->loop_len_idx, s->slice_res_idx, (int)(s->fx_density * 100.0f),
            (int)(s->order_density * 100.0f), s->pitch_range,
            (int)(s->wet * 100.0f), s->ab, s->quantize_mode, s->seed,
            s->roll_nonce, s->follow_transport ? 0 : 1,
            s->chan_mode, s->pan_l, s->pan_r, s->pad_play, s->pad_rate_idx,
            pal,
            (int)s->state, s->n_slices, s->monitor, ps,
            s->det_active ? -1 : (int)(s->det_bpm + 0.5f), s->punch_fx,
            (int)(s->bpm_override + 0.5f));
        if (n < 0 || n >= buf_len - 3) return -1;
        for (int k = 0; k < 2; k++) {
            if (k == 1)
                n = nclamp(n + snprintf(buf + n, (size_t)(buf_len - n), "\",\"locks_r\":\""), buf_len);
            int first = 1;
            for (int i = 0; i < SMACK_MAX_SLICES && n < buf_len - 28; i++) {
                if (!s->lane[k].locked[i]) continue;
                n = nclamp(n + snprintf(buf + n, (size_t)(buf_len - n), "%s%d:%d:%d",
                              first ? "" : ",", i, s->lane[k].fx[i], s->lane[k].fxp[i]), buf_len);
                if (s->lane[k].fxp2[i] >= 0 || s->lane[k].fxmix[i] >= 0)
                    n = nclamp(n + snprintf(buf + n, (size_t)(buf_len - n), ":%d:%d",
                                  s->lane[k].fxp2[i], s->lane[k].fxmix[i]), buf_len);
                first = 0;
            }
        }
        n = nclamp(n + snprintf(buf + n, (size_t)(buf_len - n), "\""), buf_len);
        for (int k = 0; k < (s->chan_mode ? 2 : 1) && s->n_slices > 0; k++) {
            n = csv_lane(buf, n, buf_len, k ? "pat_r" : "pat", &s->lane[k], 0, s->n_slices);
            n = csv_lane(buf, n, buf_len, k ? "fxp_r" : "fxp", &s->lane[k], 1, s->n_slices);
            n = csv_lane(buf, n, buf_len, k ? "ord_r" : "ord", &s->lane[k], 2, s->n_slices);
            n = csv_lane(buf, n, buf_len, k ? "fx2_r" : "fx2", &s->lane[k], 3, s->n_slices);
            n = csv_lane(buf, n, buf_len, k ? "mix_r" : "mix", &s->lane[k], 4, s->n_slices);
        }
        n = nclamp(n + snprintf(buf + n, (size_t)(buf_len - n), "}"), buf_len);
        return n;
    }
    if (!strcmp(key, "rui_poll")) {
        /* schwung-manager Remote-UI poll digest "rev:on:tick:bpm"
         * (remote_ui.go parseRuiPoll): rev gates the heavy full-state
         * refetch, tick drives the browser playhead while nothing edits. */
        int on = (s->state == SMACK_LOOPING && !s->transport_paused) ? 1 : 0;
        int ps = -1;
        if (on && s->slice_frames > 0.0) {
            ps = (int)(s->play_pos / s->slice_frames);
            if (ps >= s->n_slices) ps = s->n_slices - 1;
        }
        double fpt = frames_per_tick_now(s);
        int bpm = fpt > 0.0
            ? (int)((double)SMACK_SR * 60.0 / (fpt * 24.0) + 0.5) : 0;
        return snprintf(buf, (size_t)buf_len, "%u:%d:%d:%d",
                        s->edit_rev, on, ps, bpm);
    }
    if (!strcmp(key, "punch_fx"))
        return snprintf(buf, (size_t)buf_len, "%d", s->punch_fx);
    if (!strcmp(key, "loop_frames"))
        return snprintf(buf, (size_t)buf_len, "%u", s->loop_len);
    if (!strcmp(key, "n_slices"))
        return snprintf(buf, (size_t)buf_len, "%d", s->n_slices);
    if (!strcmp(key, "play_slice")) { /* current output slice, -1 if idle */
        int ps = -1;
        if (s->state == SMACK_LOOPING && s->slice_frames > 0.0) {
            ps = (int)(s->play_pos / s->slice_frames);
            if (ps >= s->n_slices) ps = s->n_slices - 1;
        }
        return snprintf(buf, (size_t)buf_len, "%d", ps);
    }
    if (!strcmp(key, "play_frame")) /* read-only timing diagnostic */
        /*
         * "%d", not "%.0f". The value is integral and every caller parses it
         * with atoi(), but the format mattered for a different reason: on the
         * Versio this is read by firmware linked against newlib-nano, where
         * _printf_float is a WEAK reference that nothing defines unless the
         * link line says -u _printf_float. When it is null, nano's vfprintf
         * silently emits nothing for a float conversion -- no error, no
         * warning, just an empty string that atoi() turns into 0.
         *
         * play_frame was therefore always 0 on hardware, which made LIVE
         * mode's wrap test (pf < last_pf) permanently false: it never
         * re-captured once, on any firmware build, while working perfectly on
         * a laptop where glibc formats floats. It also pinned the playhead LED
         * at full brightness, since its ramp is 1 - pf/loop_frames.
         *
         * The link flag is set too, so the engine's other float params cannot
         * fail this way. This stays integer-formatted regardless: it is an
         * integer, and it should not depend on the flag.
         */
        return snprintf(buf, (size_t)buf_len, "%d", (int)floor(s->play_pos));
    if (!strcmp(key, "pattern") || !strcmp(key, "pattern_r")) {
        /* fx codes per slice for the step-LED UIs: e.g. "0300102..." */
        const smack_lane_t *ln = &s->lane[key[7] ? 1 : 0];
        int n = s->n_slices < buf_len - 1 ? s->n_slices : buf_len - 1;
        for (int i = 0; i < n; i++) buf[i] = (char)('0' + ln->fx[i]);
        buf[n] = 0;
        return n;
    }
    if (!strcmp(key, "locked") || !strcmp(key, "locked_r")) {
        /* '1' per user-pinned slice, aligned with "pattern" */
        const smack_lane_t *ln = &s->lane[key[6] ? 1 : 0];
        int n = s->n_slices < buf_len - 1 ? s->n_slices : buf_len - 1;
        for (int i = 0; i < n; i++) buf[i] = ln->locked[i] ? '1' : '0';
        buf[n] = 0;
        return n;
    }
    if (!strcmp(key, "channel_mode"))
        return snprintf(buf, (size_t)buf_len, "%d", s->chan_mode);
    if (!strcmp(key, "pad_play"))
        return snprintf(buf, (size_t)buf_len, "%d", s->pad_play);
    if (!strcmp(key, "pad_rate"))
        return snprintf(buf, (size_t)buf_len, "%d", s->pad_rate_idx);
    if (!strcmp(key, "pad_note")) /* trigger-style: never re-fires on restore */
        return snprintf(buf, (size_t)buf_len, "0");
    if (!strcmp(key, "pad_state")) /* "active:step" — sim + UI highlight */
        return snprintf(buf, (size_t)buf_len, "%d:%d",
                        s->trig_active, s->trig_active ? s->trig_step : -1);
    if (!strcmp(key, "monitor"))
        return snprintf(buf, (size_t)buf_len, "%d", s->monitor);
    if (!strcmp(key, "hw_input"))
        return snprintf(buf, (size_t)buf_len, "%d", s->hw_input);
    if (!strcmp(key, "detected_bpm")) { /* -1 scanning, 0 none, else BPM */
        if (s->det_active) return snprintf(buf, (size_t)buf_len, "-1");
        return fmt_tenths(buf, buf_len, s->det_bpm);
    }
    if (!strcmp(key, "bpm_override"))
        return fmt_tenths(buf, buf_len, s->bpm_override);
    /* trigger params always read back as 0 (see below) */
    if (!strcmp(key, "detect_bpm") || !strcmp(key, "unlock_all"))
        return snprintf(buf, (size_t)buf_len, "0");
    if (!strcmp(key, "pan_l"))
        return snprintf(buf, (size_t)buf_len, "%d", s->pan_l);
    if (!strcmp(key, "pan_r"))
        return snprintf(buf, (size_t)buf_len, "%d", s->pan_r);
    return -1;
}
