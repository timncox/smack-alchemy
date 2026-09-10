/*
 * The two values no control on the panel can express, carried in the preset
 * blob so they survive a power cycle:
 *
 *   cpu_peak  The worst-case audio-block load seen during the PREVIOUS
 *             session, replayed on a ring at boot. The live alarm on B2 can
 *             only be read by someone watching it, and you are not, while
 *             playing with both hands.
 *
 *   free_bpm  The tempo to free-run at with nothing patched to the clock
 *             jack. Written by the TEMPO pot, by a finished BPM scan, and by
 *             whatever clock the module last locked to.
 *
 * smack-versio kept these in its own QSPI sector with an epsilon-guarded
 * save. Here they ride in slot 0 with the rest of the working state; the
 * epsilons live in smack_alchemy.cpp's autosave, which is what limits wear.
 */
#pragma once

#include <cstdint>
#include <cstring>
#include "alchemy/surface/serializable.h"

struct SmackExtras : public alchemy::Serializable
{
    float cpu_peak = 0.0f;   /* 0..1; 0 == no data yet */
    float free_bpm = 120.0f; /* the engine needs a tempo before one is known */

    size_t SerializedSize() const override { return 8u; }

    void Serialize(uint8_t* out) const override
    {
        std::memcpy(out,     &cpu_peak, 4u);
        std::memcpy(out + 4, &free_bpm, 4u);
    }

    bool Deserialize(const uint8_t* in) override
    {
        float p, b;
        std::memcpy(&p, in,     4u);
        std::memcpy(&b, in + 4, 4u);
        /* Refuse values a stale or foreign blob could carry; the schema hash
         * catches layout changes, this catches garbage inside a valid one. */
        if (!(p >= 0.0f && p <= 1.0f))   p = 0.0f;
        if (!(b > 20.0f && b < 300.0f))  b = 120.0f;
        cpu_peak = p;
        free_bpm = b;
        return true;
    }

    /* 'SMK' + layout version. Bump the low byte when the layout changes. */
    uint32_t SchemaHash() const override { return 0x534D4B00u | 0x01u; }
};
