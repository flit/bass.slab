/*
 * Copyright (c) 2015 Immo Software
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its contributors may
 *   be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#if !defined(_AR_ENVELOPE_H_)
#define _AR_ENVELOPE_H_

#include "audio_ramp.h"
#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Attack-Release envelope generator.
 */
class ASREnvelope : public AudioFilter
{
public:
    //! Ramp segments of the envelope.
    enum EnvelopeStage
    {
        kAttack,
        kRelease
    };

    ASREnvelope();
    virtual ~ASREnvelope() {}

    virtual void set_sample_rate(float rate);

    void set_peak(float peak);

    void enable_sustain(bool enable) { m_enableSustain = enable; }

    void set_length_in_seconds(EnvelopeStage stage, float seconds);
    void set_length_in_samples(EnvelopeStage stage, uint32_t samples);

    void set_attack(float seconds) { set_length_in_seconds(kAttack, seconds); }
    void set_release(float seconds) { set_length_in_seconds(kRelease, seconds); }

    float get_length_in_seconds(EnvelopeStage stage);
    uint32_t get_length_in_samples(EnvelopeStage stage);

    //! Sets the curve type. You must call this before setting the
    //! begin or end values, or the length, because it does not recompute
    //! the slope itself.
    void set_curve_type(EnvelopeStage stage, AudioRamp::CurveType theType);

    //! Sets the offset to the start of the release stage. Offset is relative to the next
    //! call to process().
    void set_release_offset(uint32_t offset);

    void trigger();
    float next();

    bool is_finished();

    virtual void process(float * samples, uint32_t count);

protected:
    AudioRamp m_attack;
    AudioRamp m_release;
    float m_peak;
    bool m_enableSustain;
    uint32_t m_releaseOffset;
    uint32_t m_elapsedSamples;
    bool m_isTriggered;
};

} // namespace slab

#endif // _AR_ENVELOPE_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
