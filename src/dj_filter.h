/*
 * The DJ filter: one knob, lowpass down one way and highpass up the other.
 *
 * In a header, and not inline in the audio callback, for one reason: this is
 * the only new DSP in the module and it is the only thing here that cannot be
 * checked on the bench except by ear. An inverted sign, a broken cascade or a
 * mistyped exponent all sound like "hmm, that's not quite right" and none of
 * them announce themselves. test/test_dj_filter.c runs tones through this and
 * measures what comes out, which is a question a laptop can answer and a
 * hardware session cannot.
 *
 * WHY TPT AND NOT A CHAMBERLIN SVF
 * --------------------------------
 * The textbook state-variable filter is unstable as its cutoff approaches a
 * quarter of the sample rate -- its coefficient is 2*sin(pi*fc/sr) and the
 * stability condition brings in the resonance term as well. This filter is
 * required to sweep to 18 kHz at 48 kHz and come back, which is right in that
 * territory. Zavalishin's topology-preserving transform is unconditionally
 * stable at any cutoff and any damping, so the tuning problem simply does not
 * exist. It is a two-pole filter: 12 dB/octave, the slope a DJ mixer uses.
 *
 * RESONANCE, AND WHY IT FADES IN
 * ------------------------------
 * The first version was two cascaded one-poles, which is this filter with its
 * Q stuck at 0.5: no peak at the corner at all. Asked for a bit more bite, the
 * Q now rises with knob travel, from 0.5 at the centre to DJ_RES_Q at the
 * ends. Fixed resonance would put a bump at 18 kHz on an "open" lowpass and a
 * sub-bass bump at 15 Hz on an "open" highpass -- the centre would stop being
 * clean. Fading it in means the notch, and everything the next two sections
 * say about it, is exactly what it was.
 *
 * WHY THE CURVES MEET AT ZERO
 * ---------------------------
 * A lowpass at 18 kHz and a highpass at 15 Hz are both, audibly, open. Both
 * ends of the control therefore converge on "not filtered", so sweeping
 * through the centre is continuous and the LP/HP flip happens exactly where
 * neither filter is doing anything. That is what lets the centre notch be a
 * true bypass without a crossfade to hide the switch.
 *
 * ...WHICH IS TRUE OF THE RESPONSE AND FALSE OF THE STATE
 * ------------------------------------------------------
 * The two modes agree on what they do to the signal at the crossing. They do
 * not agree on what the integrators should be holding while they do it: an
 * open lowpass has G = 0.707 and its state tracking roughly twice the input,
 * an open highpass has G = 0.00098 and its state near zero. Reinterpreting one
 * as the other hands the highpass a large stale charge, and out = in - state
 * then briefly INVERTS and overshoots -- measured at 2.4x full scale, decaying
 * over the highpass's own 15 Hz time constant, i.e. about 10 ms of thump every
 * single time the knob passes the centre.
 *
 * So the flip clears the state. Both directions recover for free: an open
 * highpass with zeroed state outputs the input exactly, and an open lowpass
 * recharges with a 9 microsecond time constant, which is under half a sample.
 * Cheap, and the alternative -- crossfading two filter instances -- doubles
 * the DSP to solve a problem that only exists for one block.
 *
 * Found by test_dj_filter.c, not by ear. On the module this would have sounded
 * like the filter "thumping" at the centre, which is the kind of thing you put
 * down to the patch.
 */
#ifndef SMACK_VERSIO_DJ_FILTER_H
#define SMACK_VERSIO_DJ_FILTER_H

#include <math.h>

/*
 * Endpoints, as integrator gains: g = tan(pi * fc / sr) at 48 kHz.
 *
 * Stored as g rather than as frequencies so the audio thread needs one powf
 * and no tanf. The comments are the frequencies these mean, and the test
 * converts back and checks them -- so if someone "tidies" a constant here, the
 * number it is supposed to represent is both written down and enforced.
 */
#define DJ_LP_OPEN   2.41f    /* lowpass fully open   ~18 kHz */
#define DJ_LP_RATIO  0.00109f /* ... times this at full sweep -> ~40 Hz */
#define DJ_HP_OPEN   0.00098f /* highpass fully open  ~15 Hz */
#define DJ_HP_RATIO  422.0f   /* ... times this at full sweep -> ~6 kHz */

/* Q at full travel either way; 0.5 (the centre's Q) would be no resonance.
 * 1.5 peaks about +4 dB at the corner: audible bite, nowhere near
 * self-oscillation. DJ_RES_CURVE is how fast it arrives: Q is fully up by
 * this much knob travel. */
#define DJ_RES_Q     1.5f
#define DJ_RES_CURVE 0.5f

/* One smoothing step per block. ~0.08 at 375 Hz spreads a control step over
 * roughly 30 ms, which is what keeps a 50 Hz knob read from zippering. */
#define DJ_SMOOTH    0.08f

typedef struct {
    float s[2][2]; /* [channel][0 lowpass, 1 bandpass] integrator state */
    float c;       /* smoothed control, -1..+1 */
    int   hp;      /* which mode the state currently means; see the header */
} dj_filter_t;

static inline void dj_filter_reset(dj_filter_t *f)
{
    f->s[0][0] = f->s[0][1] = f->s[1][0] = f->s[1][1] = 0.0f;
    f->c  = 0.0f;
    f->hp = 0;
}

/* Integrator gain for a control position. Negative sweeps the lowpass down,
 * positive sweeps the highpass up; magnitude is how far. */
static inline float dj_filter_g(float ctl)
{
    if (ctl > 0.0f) return DJ_HP_OPEN * powf(DJ_HP_RATIO,  ctl);
    return DJ_LP_OPEN * powf(DJ_LP_RATIO, -ctl);
}

/* Q for a control position: 0.5 at the centre, DJ_RES_Q from DJ_RES_CURVE of
 * travel on. */
static inline float dj_filter_q(float ctl)
{
    float t = (ctl < 0.0f ? -ctl : ctl) * (1.0f / DJ_RES_CURVE);
    if (t > 1.0f) t = 1.0f;
    return 0.5f + (DJ_RES_Q - 0.5f) * t;
}

/*
 * One interleaved stereo block, in place. `samples` counts samples, not
 * frames -- it is the same `size` libDaisy hands the callback.
 *
 * The control is smoothed here rather than by the caller because the smoothing
 * has to happen at block rate, and this is the only function that runs at it.
 */
static inline void dj_filter_block(dj_filter_t *f, float *io, int samples,
                                   float ctl)
{
    int i, k, c;

    /*
     * Smooth the CONTROL, not the cutoff.
     *
     * The control is what is linear in knob travel; the cutoff is exponential
     * in it. Smoothing the frequency instead would slew fast at one end of the
     * sweep and crawl at the other, which is audible as the knob having
     * different speeds in different places.
     */
    f->c += (ctl - f->c) * DJ_SMOOTH;

    {
        const int   hp = f->c > 0.0f;
        const float g  = dj_filter_g(f->c);
        const float kd = 1.0f / dj_filter_q(f->c); /* damping, 1/Q */
        const float a1 = 1.0f / (1.0f + g * (g + kd));
        const float a2 = g * a1;
        const float a3 = g * a2;

        /* Crossing the centre changes what the state MEANS. Carrying it over
         * is the 2.4x thump described in the header. */
        if (hp != f->hp) {
            f->s[0][0] = f->s[0][1] = f->s[1][0] = f->s[1][1] = 0.0f;
            f->hp = hp;
        }

        for (i = 0; i < samples; i++) {
            const float x  = io[i];
            float      *st = f->s[i & 1]; /* interleaved: even L, odd R */
            const float v3 = x - st[0];
            const float v1 = a1 * st[1] + a2 * v3;   /* bandpass */
            const float v2 = st[0] + a2 * st[1] + a3 * v3; /* lowpass */
            st[1] = 2.0f * v1 - st[1];
            st[0] = 2.0f * v2 - st[0];
            io[i] = hp ? (x - kd * v1 - v2) : v2;
        }
    }

    /* Flush the integrators once they decay into denormal territory: four
     * comparisons a block, against a subnormal penalty paid per sample. */
    for (c = 0; c < 2; c++)
        for (k = 0; k < 2; k++)
            if (f->s[c][k] < 1e-25f && f->s[c][k] > -1e-25f)
                f->s[c][k] = 0.0f;
}

#endif /* SMACK_VERSIO_DJ_FILTER_H */
