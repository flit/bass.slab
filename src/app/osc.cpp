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

#include "osc.h"
#include "arm_math.h"
#include <string.h>
#include <assert.h>
#include <math.h>

using namespace slab;

const float k2PI = 2.0f * PI;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

void Oscillator::init()
{
    m_delta = k2PI * (m_freq / m_sampleRate);

    m_env.set_sample_rate(get_sample_rate());
    m_env.set_curve_type(ASREnvelope::kAttack, AudioRamp::kLinear);
    m_env.set_curve_type(ASREnvelope::kRelease, AudioRamp::kLinear);
    m_env.set_peak(1.0f);
}

void Oscillator::set_note(uint32_t note)
{
    m_freq = 440.0f * powf(2.0, ((int32_t)note - 69) / 12.0f);  // A4 = MIDI key 69
    m_delta = k2PI * (m_freq / m_sampleRate);
}

//! From http://www.martin-finke.de/blog/articles/audio-plugins-018-polyblep-oscillator/
float Oscillator::poly_blep(float t)
{
    float dt = m_delta / k2PI;
    // 0 <= t < 1
    if (t < dt)
    {
        t /= dt;
        return t+t - t*t - 1.0;
    }
    // -1 < t < 0
    else if (t > 1.0 - dt)
    {
        t = (t - 1.0) / dt;
        return t*t + t+t + 1.0;
    }
    // 0 otherwise
    else
    {
        return 0.0;
    }
}

void Oscillator::process(float * samples, uint32_t count)
{
    // Check for a trigger.
    Sequencer::Event triggerEvent = m_seq->get_next_event(count);
    int32_t triggerSample = triggerEvent.m_timestamp;
    if (triggerEvent.m_event == Sequencer::kNoteStopEvent)
    {
        m_env.set_release_offset(triggerSample);
        triggerSample = -1;
    }
    float * sample = samples;
    int i;
    for (i = 0; i < count; ++i)
    {
        // Detect trigger point.
        if (triggerSample == i)
        {
            m_needsRestart = true;
        }

        float f;
        float t = m_phase / k2PI;
        switch (m_shape)
        {
            case kSine:
                f = arm_sin_f32(m_phase);
                break;
            case kSaw:
                f = (2.0 * m_phase / k2PI) - 1.0;
                f -= poly_blep(t);
                break;
            case kSquare:
                if (m_phase < PI)
                {
                    f = 1.0;
                }
                else
                {
                    f = -1.0;
                }
                f += poly_blep(t);
                f -= poly_blep(fmod(t + 0.5, 1.0));
                break;
            case kTriangle:
                // Same as square, above.
                if (m_phase < PI)
                {
                    f = 1.0;
                }
                else
                {
                    f = -1.0;
                }
                f += poly_blep(t);
                f -= poly_blep(fmod(t + 0.5, 1.0));
                // Integrate the square to get tri.
                f = m_delta * f + (1 - m_delta) * m_previous;
                m_previous = f;
                break;
        }

        float v = m_env.next();
        *sample++ = f * v;

        m_phase += m_delta;
        if (m_phase >= k2PI)
        {
            m_phase = 0.0f;
        }

        if (m_needsRestart && m_phase == 0)
        {
            if (triggerEvent.m_event == Sequencer::kNoteStartEvent)
            {
                set_note(triggerEvent.m_value);
            }
            m_env.trigger();
            m_needsRestart = false;
        }
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
