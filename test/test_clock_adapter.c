/*
 * Native tests for the Versio clock adapter — no hardware, no flashing.
 *
 * This is DESIGN.md §13's claim in practice: M2 is the piece with the most
 * design risk, and it can be built and verified entirely on a laptop by
 * feeding synthetic gate intervals instead of MIDI clock.
 *
 * Build & run:  make -f firmware/Makefile.test
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "../clock_adapter.h"
#include "../vendor/smack_core.h"

#define SR  SMACK_SR
#define BLK 128

/* ---- tick recorder ------------------------------------------------------ */

typedef struct {
    int ticks;
    int starts;
} rec_t;

static void rec_emit(void *ctx, uint8_t b)
{
    rec_t *r = (rec_t *)ctx;
    if (b == 0xF8) r->ticks++;
    else if (b == 0xFA) r->starts++;
}

/* Run `blocks` audio blocks, firing a gate edge every `pulse_frames` frames.
 * pulse_frames == 0 means nothing is patched. */
static void run(clock_adapter_t *c, rec_t *r, int blocks, double pulse_frames)
{
    static double next_pulse;
    double        elapsed = 0.0;
    int           b;

    next_pulse = pulse_frames > 0 ? pulse_frames : 0.0;
    for (b = 0; b < blocks; b++) {
        if (pulse_frames > 0.0) {
            while (next_pulse <= elapsed + BLK) {
                clk_gate_edge(c);
                next_pulse += pulse_frames;
            }
        }
        clk_advance(c, BLK, rec_emit, r);
        elapsed += BLK;
    }
}

/* ---- unit tests --------------------------------------------------------- */

static void test_external_locks_to_quarter_notes(void)
{
    clock_adapter_t c;
    rec_t           r = {0, 0};
    double          quarter = SR / 2.0; /* 120 BPM */

    clk_init(&c, SR, 120.0f);
    clk_set_mode(&c, CLK_EXTERNAL);
    clk_set_ratio(&c, CLK_TICKS_1X);
    run(&c, &r, 400, quarter);

    assert(clk_locked(&c));
    assert(fabs(clk_bpm(&c) - 120.0f) < 1.0f);
    assert(r.starts >= 1); /* first pulse defines the downbeat */
    printf("ok: external clock locks to 120 BPM (%.2f)\n", clk_bpm(&c));
}

static void test_ratio_switch_changes_interpretation(void)
{
    clock_adapter_t c;
    rec_t           r = {0, 0};
    double          pulse = SR / 2.0; /* same physical pulse rate */

    /* Same pulses read as EIGHTHS => the quarter is twice as long => 60 BPM */
    clk_init(&c, SR, 120.0f);
    clk_set_mode(&c, CLK_EXTERNAL);
    clk_set_ratio(&c, CLK_TICKS_2X);
    run(&c, &r, 400, pulse);
    assert(fabs(clk_bpm(&c) - 60.0f) < 1.0f);

    /* Read as HALF notes => quarter is half as long => 240 BPM */
    clk_init(&c, SR, 120.0f);
    clk_set_mode(&c, CLK_EXTERNAL);
    clk_set_ratio(&c, CLK_TICKS_DIV2);
    run(&c, &r, 400, pulse);
    assert(fabs(clk_bpm(&c) - 240.0f) < 2.0f);

    printf("ok: ratio switch reinterprets the same pulse train\n");
}

static void test_tick_rate_matches_tempo(void)
{
    clock_adapter_t c;
    rec_t           r = {0, 0};
    double          quarter = SR / 2.0;
    int             blocks  = 800;
    double          seconds, expect;

    clk_init(&c, SR, 120.0f);
    clk_set_mode(&c, CLK_EXTERNAL);
    clk_set_ratio(&c, CLK_TICKS_1X);
    run(&c, &r, blocks, quarter);

    seconds = (double)blocks * BLK / (double)SR;
    expect  = seconds * (120.0 / 60.0) * 24.0; /* 24 ppqn at 120 BPM */
    /* Phase re-locking on each pulse can shave a tick per pulse; allow 2%. */
    assert(fabs((double)r.ticks - expect) < expect * 0.02);
    printf("ok: emitted %d ticks, expected ~%.0f\n", r.ticks, expect);
}

static void test_infer_folds_into_musical_range(void)
{
    clock_adapter_t c;
    rec_t           r = {0, 0};

    /* One pulse every 2 seconds = 30 BPM, below the musical floor.
     * Should fold up by octaves into 60..180. */
    clk_init(&c, SR, 120.0f);
    clk_set_mode(&c, CLK_INFER);
    run(&c, &r, 2000, SR * 1.9);

    assert(clk_bpm(&c) >= 59.0f && clk_bpm(&c) <= 181.0f);
    printf("ok: inferred tempo folded to %.1f BPM\n", clk_bpm(&c));
}

static void test_free_run_when_nothing_patched(void)
{
    clock_adapter_t c;
    rec_t           r = {0, 0};

    clk_init(&c, SR, 90.0f);
    clk_set_mode(&c, CLK_AUTO);
    run(&c, &r, 400, 0.0); /* no gate at all */

    assert(!clk_locked(&c));
    assert(fabs(clk_bpm(&c) - 90.0f) < 0.5f);
    assert(r.ticks > 0); /* still runs */
    printf("ok: free-runs at %.1f BPM with nothing patched\n", clk_bpm(&c));
}

static void test_auto_distinguishes_clock_from_trigger(void)
{
    clock_adapter_t c;
    rec_t           r = {0, 0};

    /* Steady train -> should be read as an external clock. */
    clk_init(&c, SR, 120.0f);
    clk_set_mode(&c, CLK_AUTO);
    clk_set_ratio(&c, CLK_TICKS_1X);
    run(&c, &r, 400, SR / 2.0);
    assert(fabs(clk_bpm(&c) - 120.0f) < 2.0f);

    printf("ok: AUTO reads a steady train as clock (%.1f BPM)\n", clk_bpm(&c));
}

/* ---- integration: drive the real engine through the adapter ------------- */

static float host_bpm_value = 120.0f;
static float host_get_bpm(void) { return host_bpm_value; }

typedef struct { smack_t *s; } eng_ctx_t;

static void eng_emit(void *ctx, uint8_t b)
{
    eng_ctx_t *e = (eng_ctx_t *)ctx;
    smack_on_midi(e->s, &b, 1, 3); /* source 3 = host, as on the Move */
}

static int get_int(smack_t *s, const char *k)
{
    char buf[64];
    assert(smack_get_param(s, k, buf, sizeof(buf)) >= 0);
    return atoi(buf);
}

static void test_engine_captures_a_bar_from_gate_clock(void)
{
    host_api_v1_t   host;
    clock_adapter_t c;
    eng_ctx_t       e;
    int16_t         in[BLK * 2], out[BLK * 2];
    double          quarter = SR / 2.0, next = quarter, elapsed = 0.0;
    int             b, loop_frames;

    memset(&host, 0, sizeof(host));
    host.api_version      = 1;
    host.sample_rate      = SR;
    host.frames_per_block = BLK;
    host.get_bpm          = host_get_bpm;

    e.s = smack_create(&host);
    assert(e.s);

    clk_init(&c, SR, 120.0f);
    clk_set_mode(&c, CLK_EXTERNAL);
    clk_set_ratio(&c, CLK_TICKS_1X);

    for (b = 0; b < BLK * 2; b++) in[b] = (int16_t)((b % 64) * 300 - 9600);

    /* Two bars of gate clock so the engine's regression window fills. */
    for (b = 0; b < 1400; b++) {
        while (next <= elapsed + BLK) { clk_gate_edge(&c); next += quarter; }
        clk_advance(&c, BLK, eng_emit, &e);
        smack_process(e.s, in, out, BLK);
        elapsed += BLK;
    }

    smack_set_param(e.s, "loop_len", "4"); /* one bar */
    smack_set_param(e.s, "capture", "1");

    loop_frames = get_int(e.s, "loop_frames");
    printf("   [capture] loop_frames=%d want~%d run_state=%d n_slices=%d bpm=%.2f\n",
           loop_frames, 2 * SR, get_int(e.s, "run_state"),
           get_int(e.s, "n_slices"), clk_bpm(&c));
    /* One bar at 120 BPM = 2 seconds. The engine derived this purely from
     * gate pulses -- no MIDI hardware anywhere in the chain. */
    assert(abs(loop_frames - 2 * SR) <= SR / 100); /* within 1% */
    assert(get_int(e.s, "n_slices") > 0);

    printf("ok: engine captured %d frames from gate clock (want ~%d)\n",
           loop_frames, 2 * SR);
    smack_destroy(e.s);
}


/*
 * The ratio switch must move free-run speed the same way it moves a locked
 * clock. It used to do nothing at all with the gate unpatched, because
 * ticks_per_pulse was only ever consulted where a pulse arrives -- so you
 * could not capture a loop in /2 and flip to x2 to hear it at 4x, which is
 * the obvious reach on a module with no tempo knob.
 */
static void test_ratio_scales_free_run(void)
{
    clock_adapter_t c;
    rec_t           r = {0, 0};
    int             at_1x, at_div2, at_2x;

    /* Nothing patched: count ticks emitted over a fixed span at each ratio. */
    clk_init(&c, SR, 120.0f);
    clk_set_ratio(&c, CLK_TICKS_1X);
    run(&c, &r, 400, 0.0);
    at_1x = r.ticks;

    clk_init(&c, SR, 120.0f);
    r.ticks = 0;
    clk_set_ratio(&c, CLK_TICKS_DIV2);
    run(&c, &r, 400, 0.0);
    at_div2 = r.ticks;

    clk_init(&c, SR, 120.0f);
    r.ticks = 0;
    clk_set_ratio(&c, CLK_TICKS_2X);
    run(&c, &r, 400, 0.0);
    at_2x = r.ticks;

    printf("   [ratio] free-run ticks: /2=%d  =1=%d  x2=%d\n",
           at_div2, at_1x, at_2x);

    /* Same direction and magnitude as the locked case, where the tick length
     * is interval / ticks_per_pulse: DIV2 is 48 ticks per pulse and so runs
     * twice as many ticks as 1X, and 2X is 12 and runs half. */
    assert(at_1x > 0);
    assert(abs(at_div2 - 2 * at_1x) <= 2);
    assert(abs(at_2x * 2 - at_1x) <= 2);

    /* And the whole point: /2 to x2 is a factor of four. */
    assert(abs(at_div2 - 4 * at_2x) <= 4);

    printf("ok: ratio scales free-run tempo, /2 to x2 is 4x\n");
}

int main(void)
{
    test_external_locks_to_quarter_notes();
    test_ratio_switch_changes_interpretation();
    test_tick_rate_matches_tempo();
    test_infer_folds_into_musical_range();
    test_free_run_when_nothing_patched();
    test_auto_distinguishes_clock_from_trigger();
    test_ratio_scales_free_run();
    test_engine_captures_a_bar_from_gate_clock();
    printf("clock_adapter: all assertions passed\n");
    return 0;
}
