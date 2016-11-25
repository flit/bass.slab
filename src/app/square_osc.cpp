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

#include "square_osc.h"
#include "arm_math.h"
#include <string.h>
#include <assert.h>

using namespace slab;

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

void SquareGenerator::init()
{
    m_halfPhaseWidth = m_sampleRate / m_freq / 2.0f;

    m_env.set_sample_rate(get_sample_rate());
    m_env.set_curve_type(ASREnvelope::kAttack, AudioRamp::kLinear);
    m_env.set_curve_type(ASREnvelope::kRelease, AudioRamp::kLinear);
    m_env.set_peak(1.0f);
}

void SquareGenerator::process(float * samples, uint32_t count)
{
    // Check for a trigger.
    Sequencer::Event triggerEvent = m_seq->get_next_event(count);
    int32_t triggerSample = triggerEvent.m_timestamp;
    if (triggerEvent.m_event == Sequencer::kNoteStopEvent)
    {
        m_env.set_release_offset(triggerSample);
        triggerSample = -1;
    }
    bool needsRestartOnZeroCrossing = false;
    float previous = m_previous;
    float * sample = samples;
    int i;
    for (i = 0; i < count; ++i)
    {
        // Detect trigger point.
        if (triggerSample == i)
        {
            needsRestartOnZeroCrossing = true;
        }

        float f = 1.0f;
        if (m_phase > m_halfPhaseWidth)
        {
            f = -1.0f;
        }
        float v = f * m_env.next();
        *sample++ = v;

        // After triggered, restart on a zero crossing to prevent popping.
        bool zeroCrossing = ((previous >= 0.0f && v <= 0.0f) || (previous <= 0.0f && v >= 0.0f));
        if (needsRestartOnZeroCrossing && zeroCrossing)
        {
            m_env.trigger();
            m_phase = 0.0f;
            needsRestartOnZeroCrossing = false;
        }

        m_phase += 1.0f;
        if (m_phase >= m_halfPhaseWidth * 2.0f)
        {
            m_phase = 0.0f;
        }

        previous = v;
    }

    m_previous = previous;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
