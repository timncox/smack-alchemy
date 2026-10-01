/* readout -- see readout.h. */
#include "readout.h"

typedef struct
{
    char  *p;
    size_t left; /* room for chars, excluding the NUL */
} sink_t;

static void put(sink_t *s, char c)
{
    if (s->left)
    {
        *s->p++ = c;
        s->left--;
    }
}

static void puts_(sink_t *s, const char *str)
{
    while (str && *str) put(s, *str++);
}

static void put_uint(sink_t *s, unsigned long v)
{
    char   d[12];
    size_t k = 0;
    do
    {
        d[k++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v && k < sizeof d);
    while (k) put(s, d[--k]);
}

static long round_l(float x) { return (long)(x < 0.0f ? x - 0.5f : x + 0.5f); }

void ro_value(char *out, size_t n, float value, ro_xform_t xform, float min, float max,
              const char *unit, const char *const *labels, uint8_t num_labels)
{
    if (!out || !n) return;
    sink_t s = {out, n - 1};

    if (xform == RO_SELECTOR)
    {
        long i = (long)value; /* Value() is already a whole zone index */
        if (i < 0) i = 0;
        if (num_labels && i >= (long)num_labels) i = (long)num_labels - 1;
        if (labels && num_labels && labels[i])
            puts_(&s, labels[i]);
        else
            put_uint(&s, (unsigned long)(i + 1));
        *s.p = '\0';
        return;
    }

    /* A wide range reads best as whole numbers; a narrow one (a -1..+1
     * bipolar, a 0..1 amount) gets one decimal. */
    const float span = max > min ? max - min : min - max;
    if (span >= 20.0f)
    {
        long v = round_l(value);
        if (v < 0)
        {
            put(&s, '-');
            v = -v;
        }
        put_uint(&s, (unsigned long)v);
    }
    else
    {
        long t = round_l(value * 10.0f); /* tenths; -0.04 rounds to 0 */
        if (t < 0)
        {
            put(&s, '-');
            t = -t;
        }
        else if (min < 0.0f && t > 0)
            put(&s, '+'); /* bipolar: say which side of centre */
        put_uint(&s, (unsigned long)(t / 10));
        put(&s, '.');
        put(&s, (char)('0' + t % 10));
    }

    if (unit && *unit)
    {
        put(&s, ' ');
        puts_(&s, unit);
    }
    *s.p = '\0';
}

void ro_upper(char *out, size_t n, const char *src)
{
    if (!out || !n) return;
    sink_t s = {out, n - 1};
    while (src && *src)
    {
        char c = *src++;
        put(&s, (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c);
    }
    *s.p = '\0';
}
