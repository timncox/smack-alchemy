/*
 * readout -- a knob's value in words, for the 1U OLED.
 *
 * Takes what the SDK's VirtualKnob already declares (transform, range, unit,
 * zone labels), so any firmware can describe any knob without a table of its
 * own: a selector shows its label ("Tape Stop"), a range shows a number and
 * its unit ("120 BPM", "64 %", "-0.4").
 *
 * HAL-free C99, no printf (newlib-nano drops %f). Native-tested in
 * test/test_readout.c. Shared by copy across the Alchemy family.
 */
#ifndef READOUT_H
#define READOUT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Same order as alchemy::VirtualKnob::Xform. */
typedef enum { RO_LINEAR = 0, RO_EXP = 1, RO_SELECTOR = 2 } ro_xform_t;

/* Writes the value of a knob into out (always NUL-terminated, truncated to
 * n - 1 chars). value is what VirtualKnob::Value() returns: the transformed
 * value, or the zone index for a selector. */
void ro_value(char *out, size_t n, float value, ro_xform_t xform, float min, float max,
              const char *unit, const char *const *labels, uint8_t num_labels);

/* Copies src into out in upper case (ASCII), truncated like ro_value. */
void ro_upper(char *out, size_t n, const char *src);

#ifdef __cplusplus
}
#endif

#endif
