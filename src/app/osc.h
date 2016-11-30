/*
 * Copyright (c) 2015-2016 Immo Software
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
#if !defined(_OSC_H_)
#define _OSC_H_

#include "audio_filter.h"
#include "asr_envelope.h"
#include "sequencer.h"

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

namespace slab {

/*!
 * @brief Multishape oscillator.
 */
class Oscillator : public AudioFilter
{
public:
    typedef enum : uint32_t {
        kSine,
        kSaw,
        kSquare,
        kTriangle
    } shape_t;

    Oscillator()
    :   m_shape(kSine),
        m_delta(0),
        m_freq(440.0f),
        m_phase(0),
        m_env(),
        m_previous(0),
        m_seq(NULL),
        m_needsRestart(false)
    {
    }

    void set_shape(shape_t shape) { m_shape = shape; }
    void set_freq(float freq) { m_freq = freq; }
    void set_note(uint32_t note);
    void set_sequence(Sequencer * seq) { m_seq = seq; }
    void enable_sustain(bool enable) { m_env.enable_sustain(enable); }
    void set_attack(float seconds) { m_env.set_length_in_seconds(ASREnvelope::kAttack, seconds); }
    void set_release(float seconds) { m_env.set_length_in_seconds(ASREnvelope::kRelease, seconds); }
    void init();

    virtual void process(float * samples, uint32_t count);

protected:
    shape_t m_shape;
    float m_delta;
    float m_freq;
    float m_phase;
    ASREnvelope m_env;
    float m_previous;
    Sequencer * m_seq;
    bool m_needsRestart;

    float poly_blep(float t);
};

} // namespace slab

#endif // _OSC_H_
//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
