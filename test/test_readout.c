/* Native tests for readout.c: a knob's value in words for the 1U OLED.
 * The cases are Smack's real knobs. */
#include <stdio.h>
#include <string.h>
#include "readout.h"

static int fails = 0;

static void eq(const char *got, const char *want, const char *what)
{
    if (strcmp(got, want) != 0)
    {
        printf("FAIL %s: got \"%s\", want \"%s\"\n", what, got, want);
        fails++;
    }
}

int main(void)
{
    char b[32];
    static const char *const punch[27] = {
        "Clean", "Retrig", "Reverse", "Pitch", "Speed", "Gate", "Buzz", "Crush",
        "Repeat", "Rev After", "Tape Stop", "Tape Start", "Scratch", "Env", "Pan",
        "Filter", "Vowel", "Tonal Delay", "Freeze", "Delay", "Dist", "Phaser",
        "Verb", "P-Shift", "Ring Mod", "Comb", "Scatter"};
    static const char *const ratio[3] = {"/2", "=1", "x2"};

    /* selectors: the zone's label */
    ro_value(b, sizeof b, 10.0f, RO_SELECTOR, 0, 27, NULL, punch, 27);
    eq(b, "Tape Stop", "punch fx 10");
    ro_value(b, sizeof b, 0.0f, RO_SELECTOR, 0, 27, NULL, punch, 27);
    eq(b, "Clean", "punch fx 0");
    ro_value(b, sizeof b, 26.0f, RO_SELECTOR, 0, 27, NULL, punch, 27);
    eq(b, "Scatter", "punch fx last");
    ro_value(b, sizeof b, 40.0f, RO_SELECTOR, 0, 27, NULL, punch, 27);
    eq(b, "Scatter", "selector clamps high");
    ro_value(b, sizeof b, -3.0f, RO_SELECTOR, 0, 3, NULL, ratio, 3);
    eq(b, "/2", "selector clamps low");
    ro_value(b, sizeof b, 2.0f, RO_SELECTOR, 0, 4, NULL, NULL, 0);
    eq(b, "3", "unlabelled selector counts from 1");

    /* wide ranges: whole numbers + unit */
    ro_value(b, sizeof b, 119.6f, RO_LINEAR, 50, 200, "BPM", NULL, 0);
    eq(b, "120 BPM", "tempo");
    ro_value(b, sizeof b, 64.4f, RO_LINEAR, 0, 100, "%", NULL, 0);
    eq(b, "64 %", "fx amount");
    ro_value(b, sizeof b, 0.0f, RO_LINEAR, 0, 100, "%", NULL, 0);
    eq(b, "0 %", "zero");
    ro_value(b, sizeof b, 12.2f, RO_LINEAR, 1, 24, "st", NULL, 0);
    eq(b, "12 st", "pitch range");
    ro_value(b, sizeof b, 127.0f, RO_LINEAR, 0, 127, NULL, NULL, 0);
    eq(b, "127", "seed, no unit");
    ro_value(b, sizeof b, -12.4f, RO_LINEAR, -24, 24, "dB", NULL, 0);
    eq(b, "-12 dB", "negative whole");

    /* narrow ranges: one decimal, bipolar gets a sign */
    ro_value(b, sizeof b, -0.42f, RO_LINEAR, -1, 1, NULL, NULL, 0);
    eq(b, "-0.4", "dj filter low side");
    ro_value(b, sizeof b, 0.47f, RO_LINEAR, -1, 1, NULL, NULL, 0);
    eq(b, "+0.5", "dj filter high side");
    ro_value(b, sizeof b, -0.04f, RO_LINEAR, -1, 1, NULL, NULL, 0);
    eq(b, "0.0", "dj filter centre, no -0.0");
    ro_value(b, sizeof b, 0.5f, RO_LINEAR, 0, 1, NULL, NULL, 0);
    eq(b, "0.5", "unipolar has no plus");

    /* truncation is safe */
    char t[6];
    ro_value(t, sizeof t, 10.0f, RO_SELECTOR, 0, 27, NULL, punch, 27);
    eq(t, "Tape ", "truncated label");
    ro_value(t, sizeof t, 120.0f, RO_LINEAR, 50, 200, "BPM", NULL, 0);
    eq(t, "120 B", "truncated number");
    ro_value(t, 1, 120.0f, RO_LINEAR, 50, 200, "BPM", NULL, 0);
    eq(t, "", "n == 1 leaves just the NUL");

    ro_upper(b, sizeof b, "Punch FX");
    eq(b, "PUNCH FX", "upper");
    ro_upper(t, sizeof t, "Clock Ratio");
    eq(t, "CLOCK", "upper truncated");

    if (fails)
    {
        printf("test_readout: %d failure(s)\n", fails);
        return 1;
    }
    printf("test_readout: all passed\n");
    return 0;
}
