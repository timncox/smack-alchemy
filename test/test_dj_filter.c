/*
 * The DJ filter, measured rather than listened to.
 *
 *   make -f firmware/Makefile.test
 *
 * This is the only new DSP in the module and the only part of it that a bench
 * session cannot check. "Sounds about right" is the entire feedback channel on
 * hardware, and an inverted sign, a broken cascade or a mistyped exponent all
 * pass that test -- a highpass wired backwards still makes a noise when you
 * turn the knob. So the questions get asked here, where they have numeric
 * answers: does the lowpass actually remove the top, does the highpass
 * actually remove the bottom, is the centre really transparent, do the two
 * channels stay apart, and does any of it blow up when swept hard.
 *
 * The frequency assertions are deliberately loose. They are there to catch a
 * constant that has been changed to something WRONG, not to pin the filter to
 * a particular voicing -- somebody retuning the sweep on purpose should not
 * have to argue with a test about it.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../dj_filter.h"

#define SR    48000.0
#define BLK   128        /* frames */
#define SAMP  (BLK * 2)  /* interleaved stereo samples */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* The cutoff a given integrator gain represents: g = tan(pi*fc/sr). */
static double g_to_hz(float g)
{
    return atan((double)g) * SR / M_PI;
}

static double db(double gain)
{
    if (gain < 1e-12) gain = 1e-12;
    return 20.0 * log10(gain);
}

/*
 * Run a sine through the filter at a fixed control position and report the
 * output level relative to the input.
 *
 * `settle` blocks are discarded first, because dj_filter_block() smooths the
 * control one step per call -- measuring immediately would measure the ramp
 * rather than the filter. `left_only` feeds silence to the right channel so
 * the same helper can answer the crosstalk question.
 */
static double response(float ctl, double hz, int left_only, double *right_rms)
{
    dj_filter_t f;
    double      phase = 0.0, step = 2.0 * M_PI * hz / SR;
    double      sum_l = 0.0, sum_r = 0.0, sum_in = 0.0;
    int         n = 0, b, i;
    const int   settle = 400, measure = 400;

    dj_filter_reset(&f);

    for (b = 0; b < settle + measure; b++) {
        float io[SAMP];
        for (i = 0; i < BLK; i++) {
            float v = (float)sin(phase);
            phase += step;
            if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
            io[i * 2]     = v;
            io[i * 2 + 1] = left_only ? 0.0f : v;
            if (b >= settle) sum_in += (double)v * (double)v;
        }
        dj_filter_block(&f, io, SAMP, ctl);
        if (b >= settle) {
            for (i = 0; i < BLK; i++) {
                sum_l += (double)io[i * 2] * (double)io[i * 2];
                sum_r += (double)io[i * 2 + 1] * (double)io[i * 2 + 1];
                n++;
            }
        }
    }

    if (right_rms) *right_rms = sqrt(sum_r / n);
    return sqrt(sum_l / n) / sqrt(sum_in / n);
}

/* The four endpoint constants, converted back to the frequencies their
 * comments claim. A constant edited without its comment fails here. */
static void test_endpoints_are_the_advertised_frequencies(void)
{
    double lp_open = g_to_hz(dj_filter_g(0.0f));
    double lp_shut = g_to_hz(dj_filter_g(-1.0f));
    double hp_open = g_to_hz(dj_filter_g(1e-6f)); /* just onto the HP side */
    double hp_shut = g_to_hz(dj_filter_g(1.0f));

    printf("   LP %6.0f Hz open -> %5.0f Hz swept\n", lp_open, lp_shut);
    printf("   HP %6.0f Hz open -> %5.0f Hz swept\n", hp_open, hp_shut);

    assert(lp_open > 15000.0 && lp_open < 22000.0); /* ~18 kHz: open */
    assert(lp_shut >    25.0 && lp_shut <    70.0); /* ~40 Hz: shut  */
    assert(hp_open >     5.0 && hp_open <    30.0); /* ~15 Hz: open  */
    assert(hp_shut >  4000.0 && hp_shut <  9000.0); /* ~6 kHz: shut  */

    /* The sweep must be monotonic in the direction the panel promises. */
    assert(g_to_hz(dj_filter_g(-0.5f)) < lp_open);
    assert(g_to_hz(dj_filter_g(-0.5f)) > lp_shut);
    assert(g_to_hz(dj_filter_g(0.5f))  > hp_open);
    assert(g_to_hz(dj_filter_g(0.5f))  < hp_shut);

    printf("ok: endpoints match the frequencies their comments claim\n");
}

/* Centre is a real bypass, not a nearly-open filter. This is the one that
 * would catch the module being quietly filtered all the time. */
static void test_centre_is_transparent(void)
{
    double lo = response(0.0f, 100.0, 0, NULL);
    double hi = response(0.0f, 1000.0, 0, NULL);
    double top = response(0.0f, 8000.0, 0, NULL);

    printf("   centre: 100 Hz %+.2f dB, 1 kHz %+.2f dB, 8 kHz %+.2f dB\n",
           db(lo), db(hi), db(top));

    assert(db(lo)  > -0.5 && db(lo)  < 0.5);
    assert(db(hi)  > -0.5 && db(hi)  < 0.5);
    assert(db(top) > -3.0 && db(top) < 0.5); /* 18 kHz corner just reaches here */

    printf("ok: the centre notch passes the band untouched\n");
}

/* Fully left removes the top; fully right removes the bottom. Backwards is the
 * failure this exists to catch, so both directions are asserted, not just the
 * attenuation. */
static void test_the_sweep_goes_the_right_way(void)
{
    double lp_kills_high = response(-1.0f, 1000.0, 0, NULL);
    double lp_keeps_low  = response(-1.0f, 40.0,   0, NULL);
    double hp_kills_low  = response(1.0f,  100.0,  0, NULL);
    double hp_keeps_high = response(1.0f,  12000.0, 0, NULL);

    printf("   LP full left:  1 kHz %+.1f dB, 40 Hz %+.1f dB\n",
           db(lp_kills_high), db(lp_keeps_low));
    printf("   HP full right: 100 Hz %+.1f dB, 12 kHz %+.1f dB\n",
           db(hp_kills_low), db(hp_keeps_high));

    assert(db(lp_kills_high) < -25.0); /* 1 kHz is >4 octaves past a 40 Hz corner */
    assert(db(lp_keeps_low)  >  -8.0); /* at the corner, two poles: about -6 dB */
    assert(db(hp_kills_low)  < -25.0);
    assert(db(hp_keeps_high) >  -8.0);

    /* And the sweep is progressive, not a switch at the end of travel. */
    assert(db(response(-0.5f, 1000.0, 0, NULL)) < -1.0);
    assert(db(response(-0.5f, 1000.0, 0, NULL)) > db(lp_kills_high));

    printf("ok: left sweeps a lowpass down, right sweeps a highpass up\n");
}

/* Two lanes hard-panned in DUAL mode is a headline feature; a filter that
 * leaked between channels would quietly undo it. */
static void test_channels_are_independent(void)
{
    double right = 0.0;
    (void)response(-0.6f, 500.0, 1, &right);
    printf("   silent channel out: %.3g\n", right);
    assert(right < 1e-9);
    printf("ok: the two channels do not bleed into each other\n");
}

/*
 * Sweep the control across its whole range, fast, at full scale.
 *
 * This is the question TPT was chosen to answer -- a Chamberlin SVF driven to
 * 18 kHz at this sample rate is where the textbook filter comes apart. Nothing
 * may go infinite, NaN, or louder than the signal that went in.
 */
static void test_a_hard_sweep_stays_bounded(void)
{
    dj_filter_t f;
    double      phase = 0.0, step = 2.0 * M_PI * 220.0 / SR;
    double      peak = 0.0;
    int         b, i;

    dj_filter_reset(&f);

    for (b = 0; b < 4000; b++) {
        float io[SAMP];
        /* -1 to +1 and back, twice, over the run: faster than a hand. */
        float ctl = (float)sin(2.0 * M_PI * (double)b / 1000.0);
        for (i = 0; i < BLK; i++) {
            float v = (float)sin(phase);
            phase += step;
            if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
            io[i * 2] = io[i * 2 + 1] = v;
        }
        dj_filter_block(&f, io, SAMP, ctl);
        for (i = 0; i < SAMP; i++) {
            double a = fabs((double)io[i]);
            assert(isfinite(io[i]));
            if (a > peak) peak = a;
        }
    }

    printf("   worst sample over a hard two-cycle sweep: %.3f\n", peak);
    /* There is no resonance in this topology, so the only way past unity is a
     * state discontinuity. The bound is tight on purpose: at 2.0 this passed
     * while the filter was overshooting to 2.37 at every centre crossing. */
    assert(peak < 1.2);
    printf("ok: sweeping hard stays finite and does not gain up\n");
}

/*
 * Crossing the centre, on its own and slowly.
 *
 * Separated from the hard sweep because it is a different question. The hard
 * sweep asks whether the filter survives being thrown around; this asks
 * whether the ONE point where the topology changes underneath the signal is
 * silent. It is the failure the first version of this file actually had: an
 * open lowpass and an open highpass agree about the signal and disagree about
 * the state, so carrying the state across inverted the output and overshot to
 * 2.4x for about 10 ms -- every time the knob passed the middle.
 *
 * A full-scale bass note is the worst case, because a low frequency is what
 * leaves the most charge in the integrators to be misread.
 */
static void test_crossing_the_centre_is_silent(void)
{
    dj_filter_t f;
    double      phase = 0.0, step = 2.0 * M_PI * 55.0 / SR;
    double      peak = 0.0;
    int         b, i, crossings = 0, was_hp;

    dj_filter_reset(&f);
    was_hp = f.hp;

    for (b = 0; b < 3000; b++) {
        float io[SAMP];
        /* -0.4 to +0.4 and back, over about 4 seconds: a hand moving the knob
         * through the notch without hurrying. */
        float ctl = 0.4f * (float)sin(2.0 * M_PI * (double)b / 1500.0);
        for (i = 0; i < BLK; i++) {
            float v = (float)sin(phase);
            phase += step;
            if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
            io[i * 2] = io[i * 2 + 1] = v;
        }
        dj_filter_block(&f, io, SAMP, ctl);
        if (f.hp != was_hp) { crossings++; was_hp = f.hp; }
        for (i = 0; i < SAMP; i++) {
            double a = fabs((double)io[i]);
            if (a > peak) peak = a;
        }
    }

    printf("   %d centre crossings, worst sample %.3f\n", crossings, peak);
    assert(crossings >= 2); /* the test has to actually cross, or it proves nothing */
    assert(peak < 1.1);     /* input was full scale; anything above is the thump */

    printf("ok: sweeping through the centre does not thump\n");
}

/* reset() has to actually clear the state, or a role change replays the last
 * session's decay as a click. */
static void test_reset_clears_the_state(void)
{
    dj_filter_t f;
    float       io[SAMP];
    int         i;

    dj_filter_reset(&f);
    for (i = 0; i < SAMP; i++) io[i] = 0.9f;
    dj_filter_block(&f, io, SAMP, -1.0f);
    assert(f.s[0][0] != 0.0f); /* it charged up */

    dj_filter_reset(&f);
    assert(f.s[0][0] == 0.0f && f.s[0][1] == 0.0f);
    assert(f.s[1][0] == 0.0f && f.s[1][1] == 0.0f);
    assert(f.c == 0.0f);

    /* Silence in, silence out, immediately. */
    for (i = 0; i < SAMP; i++) io[i] = 0.0f;
    dj_filter_block(&f, io, SAMP, -1.0f);
    for (i = 0; i < SAMP; i++) assert(io[i] == 0.0f);

    printf("ok: reset leaves nothing behind to click on the next role change\n");
}

int main(void)
{
    test_endpoints_are_the_advertised_frequencies();
    test_centre_is_transparent();
    test_the_sweep_goes_the_right_way();
    test_channels_are_independent();
    test_a_hard_sweep_stays_bounded();
    test_crossing_the_centre_is_silent();
    test_reset_clears_the_state();
    printf("dj_filter: all assertions passed\n");
    return 0;
}
