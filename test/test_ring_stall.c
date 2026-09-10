/*
 * Native repro for the ring-recording stall — no hardware, no flashing.
 *
 *   make -f firmware/Makefile.test
 *
 * The bug, as read from the engine source:
 *
 *   record_ring_frame() refuses to write while SMACK_LOOPING once the write
 *   head comes within 2048 frames of the protected loop region:
 *
 *       if (s->state == SMACK_LOOPING) {
 *           uint32_t gap = (protected_start + SMACK_RING_FRAMES - s->ring_w)
 *                          % SMACK_RING_FRAMES;
 *           if (gap <= 2048) return;
 *       }
 *
 *   That guard is correct in intent -- the recorder must not overwrite the
 *   loop it is playing -- but when it fires it also freezes ring_last_global,
 *   and capture_retro() clamps its capture point to that value:
 *
 *       if (boundary > s->ring_last_global) boundary = s->ring_last_global;
 *
 *   so every capture after the stall grabs a window ending at the frozen
 *   point. The module keeps looping and keeps accepting Capture; it just
 *   quietly returns audio from before the stall instead of what is being
 *   played now.
 *
 * Writable headroom after a capture is RING - loop_available frames, and
 * loop_available is never less than loop_len, so fresh audio can only
 * accumulate while
 *
 *       loop_len < SMACK_RING_FRAMES / 2
 *
 * At 120 BPM the longest loop (256 steps) is 32 s against a 70 s ring and
 * this holds with room to spare. Below roughly 110 BPM it stops holding, and
 * LIVE mode -- which re-captures once per loop pass -- can never refresh.
 *
 * These tests measure staleness from the OUTPUT rather than from engine
 * internals: the input is a DC level that steps up over time, so "which era
 * is in the loop" is readable by playing the loop back with wet = 0.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../vendor/smack_core.h"

#define SR  SMACK_SR
#define BLK 128

static float g_bpm = 120.0f;
static float host_get_bpm(void) { return g_bpm; }

typedef struct {
    smack_t *s;
    int16_t  in[BLK * 2];
    int16_t  out[BLK * 2];
} rig_t;

static int get_int(smack_t *s, const char *k)
{
    char buf[64];
    if (smack_get_param(s, k, buf, sizeof(buf)) < 0) return -1;
    return atoi(buf);
}

static void rig_init(rig_t *r, float bpm, int loop_len_idx)
{
    static host_api_v1_t host;

    g_bpm = bpm;
    memset(&host, 0, sizeof(host));
    host.api_version      = 1;
    host.sample_rate      = SR;
    host.frames_per_block = BLK;
    host.get_bpm          = host_get_bpm;

    memset(r, 0, sizeof(*r));
    r->s = smack_create(&host);
    assert(r->s);

    /* Match the firmware's boot configuration, except wet: the clean tap is
     * what makes the recorded DC readable at the output. */
    smack_set_param(r->s, "hw_input", "1");
    smack_set_param(r->s, "monitor",  "0");
    smack_set_param(r->s, "wet",      "0");
    {
        char b[16];
        snprintf(b, sizeof(b), "%d", loop_len_idx);
        smack_set_param(r->s, "loop_len", b);
    }
}

/* Feed `secs` of DC at `level`, free-running (no MIDI clock). */
static void rig_run(rig_t *r, double secs, int level, int recapture_on_wrap)
{
    long blocks = (long)((secs * SR) / BLK);
    int  last_pf = 0;
    long b, i;

    for (i = 0; i < BLK * 2; i++) r->in[i] = (int16_t)level;

    for (b = 0; b < blocks; b++) {
        smack_process(r->s, r->in, r->out, BLK);
        if (recapture_on_wrap) {
            /* Mirrors the firmware's LIVE mode: re-capture each time the
             * playhead wraps. Sampled per block rather than per 8 ms, which
             * only makes the wrap easier to catch than on hardware. */
            int pf = get_int(r->s, "play_frame");
            if (pf >= 0 && pf < last_pf) smack_set_param(r->s, "capture", "1");
            last_pf = pf;
        }
    }
}

/*
 * Play one full loop pass with a silent input and report what FRACTION of it
 * is at or above `floor`. With wet = 0 the output is the clean loop tap, so
 * this measures how much of the loop came from the newest era.
 *
 * A fraction, not a peak. A stalled recorder is not frozen forever: each
 * capture moves the protected region and buys the recorder a few seconds
 * before it jams again, so a stalled engine still lands a little fresh audio
 * in the ring. That was enough to make a max-abs check read "fresh" -- the
 * pre-fix LIVE run measured a peak of 24000 while stalled for 49 of 60
 * samples. Asking how much of the loop is fresh separates the two cleanly:
 * a 64 s loop carrying 6 s of new audio fails outright.
 */
static double loop_fraction_above(rig_t *r, int floor_val)
{
    int  frames = get_int(r->s, "loop_frames");
    long blocks = frames > 0 ? (frames / BLK) : 0;
    long above = 0, total = 0;
    long b, i;

    for (i = 0; i < BLK * 2; i++) r->in[i] = 0;
    for (b = 0; b < blocks; b++) {
        smack_process(r->s, r->in, r->out, BLK);
        for (i = 0; i < BLK * 2; i++) {
            int v = r->out[i] < 0 ? -r->out[i] : r->out[i];
            if (v >= floor_val) above++;
            total++;
        }
    }
    return total > 0 ? (double)above / (double)total : 0.0;
}

/*
 * Three input levels, so "which era is in the loop" is unambiguous. ERA_MID
 * plays while the pre-fix recorder was still writing and ERA_NEW only after
 * it stalled, which is what makes these tests discriminate: a stalled
 * recorder can only ever return MID.
 */
#define ERA_PRE  3000
#define ERA_MID 12000
#define ERA_NEW 24000

/* Nearly all of the loop must be the newest era. Not all of it: the engine
 * fades slice and loop edges over ~2.2 ms, which dips a few samples below any
 * fixed floor. 0.9 clears that by a wide margin while still failing a loop
 * that is mostly stale. */
static void assert_fresh(rig_t *r, const char *what)
{
    double frac = loop_fraction_above(r, (ERA_MID + ERA_NEW) / 2);
    printf("   [%s] %.1f%% of the loop is the newest era (want >90%%)\n",
           what, frac * 100.0);
    assert(frac > 0.9);
}

/*
 * A manual Capture, pressed long after the previous one, must return audio
 * from now -- not from before the recorder stopped.
 *
 * 120 BPM / 256 steps is a 32 s loop. At the old 70 s ring that left 38 s of
 * writable space, so the recorder stopped 38 s after the capture; ERA_NEW
 * below starts at t = 45 s, entirely inside the stalled window, and the final
 * capture came back full of ERA_MID. Measured, not inferred -- this test
 * failed exactly that way before the fix.
 */
static void test_capture_after_stall_is_fresh(void)
{
    rig_t r;

    rig_init(&r, 120.0f, 8); /* 256 steps */

    rig_run(&r, 40.0, ERA_PRE, 0);
    smack_set_param(r.s, "capture", "1");
    assert(get_int(r.s, "run_state") == 3);

    rig_run(&r, 45.0, ERA_MID, 0);  /* spans the old stall point at 38 s */
    rig_run(&r, 30.0, ERA_NEW, 0);  /* only reaches the ring if it recovered */
    smack_set_param(r.s, "capture", "1");

    assert_fresh(&r, "manual");
    printf("ok: capture long after the previous one returns current audio\n");
    smack_destroy(r.s);
}

/*
 * LIVE mode with a loop longer than half the old ring.
 *
 * 60 BPM / 256 steps is a 64 s loop. At 70 s of ring that left 6 s of
 * writable space, so the recorder spent most of its time stalled -- measured
 * at 49 of 60 samples across a 300 s run -- and LIVE was refreshing from a
 * frozen ring rather than from what was being played.
 *
 * No amount of guard tweaking fixes that case: the ring has to be big enough
 * to hold the loop that is playing AND the fresh one being written, which is
 * why SMACK_MAX_SECONDS doubled and why capture now caps LENGTH at half the
 * ring.
 */
static void test_live_refreshes_with_a_long_loop(void)
{
    rig_t r;

    rig_init(&r, 60.0f, 8); /* 256 steps at 60 BPM */

    /* Enough pre-roll that the full 64 s is actually available to capture --
     * capture_retro() shortens the loop to whatever has been recorded. */
    rig_run(&r, 80.0, ERA_PRE, 0);
    smack_set_param(r.s, "capture", "1");
    assert(get_int(r.s, "run_state") == 3);

    printf("   [live]   loop_frames=%d (%.1f s), ring=%d (%.1f s)\n",
           get_int(r.s, "loop_frames"),
           get_int(r.s, "loop_frames") / (double)SR,
           SMACK_RING_FRAMES, SMACK_RING_FRAMES / (double)SR);
    assert(get_int(r.s, "loop_frames") > 60 * SR); /* the long-loop case */

    /* ERA_NEW runs for three loop passes, so the last wrap-triggered capture
     * falls entirely inside it. Two passes leaves the final capture straddling
     * the era boundary and the measurement lands near the threshold. */
    rig_run(&r, 70.0,  ERA_MID, 1);
    rig_run(&r, 200.0, ERA_NEW, 1);

    assert_fresh(&r, "live");
    printf("ok: LIVE keeps refreshing with a loop longer than half the old ring\n");
    smack_destroy(r.s);
}

/*
 * At tempos slow enough that 256 steps cannot fit in half the ring, LENGTH
 * steps down a musical notch instead of having its frame count clipped.
 *
 * Clipping would keep loop_clock_ticks at 512 half-steps while shortening the
 * buffer, so loop_frames_per_tick would be rescaled and the loop would play
 * back at the wrong rate and drift against the clock. Halving the length
 * keeps it on the grid.
 */
static void test_slow_tempo_steps_length_down_musically(void)
{
    rig_t r;
    int   frames;

    rig_init(&r, 30.0f, 8); /* 256 steps at 30 BPM would be 128 s */

    rig_run(&r, 90.0, ERA_PRE, 0);
    smack_set_param(r.s, "capture", "1");
    assert(get_int(r.s, "run_state") == 3);

    frames = get_int(r.s, "loop_frames");
    printf("   [slow]   loop_frames=%d (%.1f s), half-ring=%d (%.1f s)\n",
           frames, frames / (double)SR,
           SMACK_RING_FRAMES / 2, (SMACK_RING_FRAMES / 2) / (double)SR);

    assert(frames <= SMACK_RING_FRAMES / 2);
    /* 30 BPM: a step is 0.5 s, so the notches are 128 s, 64 s, 32 s... The
     * first that fits in half of a 150 s ring is 64 s (128 steps). */
    assert(fabs(frames - 64.0 * SR) < SR);
    printf("ok: slow tempo halves LENGTH rather than clipping it off-grid\n");
    smack_destroy(r.s);
}

/*
 * An optional test number runs just that one. assert() aborts the process, so
 * without this only the first failure is ever visible -- which is how two of
 * these were briefly believed to fail against the pre-fix engine when only the
 * first had been seen to.
 */
int main(int argc, char **argv)
{
    int only = argc > 1 ? atoi(argv[1]) : 0;

    if (only == 0 || only == 1) test_capture_after_stall_is_fresh();
    if (only == 0 || only == 2) test_live_refreshes_with_a_long_loop();
    if (only == 0 || only == 3) test_slow_tempo_steps_length_down_musically();
    printf("ring_stall: all assertions passed\n");
    return 0;
}
