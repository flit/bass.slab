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

#include "asr_envelope.h"
#include "arm_math.h"
#include "argon/argon.h"

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

ASREnvelope::ASREnvelope()
:   AudioFilter(),
    m_attack(),
    m_release(),
    m_peak(1.0f),
    m_enableSustain(false),
    m_releaseOffset(0),
    m_elapsedSamples(0),
    m_isTriggered(false)
{
}

void ASREnvelope::set_sample_rate(float rate)
{
    AudioFilter::set_sample_rate(rate);
    m_attack.set_sample_rate(rate);
    m_release.set_sample_rate(rate);
}

void ASREnvelope::set_peak(float peak)
{
    m_peak = peak;
    m_attack.set_begin_value(0.0f);
    m_attack.set_end_value(peak);
    m_release.set_begin_value(peak);
    m_release.set_end_value(0.0f);
}

void ASREnvelope::set_length_in_seconds(EnvelopeStage stage, float seconds)
{
    switch (stage)
    {
        case kAttack:
            m_attack.set_length_in_seconds(seconds);
            break;

        case kRelease:
            m_release.set_length_in_seconds(seconds);
            break;

        default:
            break;
    }
}

void ASREnvelope::set_length_in_samples(EnvelopeStage stage, uint32_t samples)
{
    switch (stage)
    {
        case kAttack:
            m_attack.set_length_in_samples(samples);
            break;

        case kRelease:
            m_release.set_length_in_samples(samples);
            break;

        default:
            break;
    }
}

float ASREnvelope::get_length_in_seconds(EnvelopeStage stage)
{
    switch (stage)
    {
        case kAttack:
            return m_attack.get_length_in_seconds();

        case kRelease:
            return m_release.get_length_in_seconds();

        default:
            break;
    }
    return 0.0f;
}

uint32_t ASREnvelope::get_length_in_samples(EnvelopeStage stage)
{
    switch (stage)
    {
        case kAttack:
            return m_attack.get_length_in_samples();

        case kRelease:
            return m_release.get_length_in_samples();

        default:
            break;
    }
    return 0;
}

void ASREnvelope::set_curve_type(EnvelopeStage stage, AudioRamp::CurveType theType)
{
    switch (stage)
    {
        case kAttack:
            m_attack.set_curve_type(theType);
            break;

        case kRelease:
            m_release.set_curve_type(theType);
            break;

        default:
            break;
    }
}

void ASREnvelope::set_release_offset(uint32_t offset)
{
    m_releaseOffset = m_elapsedSamples + offset;
}

void ASREnvelope::trigger()
{
    set_peak(m_peak);
    m_attack.reset();
    m_release.reset();
    m_elapsedSamples = 0;
    m_isTriggered = true;
}

float ASREnvelope::next()
{
//     if (m_attack.is_finished())
//     {
//         return m_release.next();
//     }
//     else
//     {
//         return m_attack.next();
//     }
    float sample;
    process(&sample, 1);
    return sample;
}

bool ASREnvelope::is_finished()
{
    return m_attack.is_finished() && m_release.is_finished();
}

void ASREnvelope::process(float * samples, uint32_t count)
{
//     if (!m_attack.is_finished())
//     {
//         // Attack
//         uint32_t attackCount = m_attack.get_remaining_samples();
//         if (attackCount > count)
//         {
//             attackCount = count;
//         }
//         m_attack.process(samples, attackCount);
//
//         // Sustain
//         if (attackCount < count)
//         {
//             uint32_t sustainCount = count - attackCount;
//             if (sustainCount + m_elapsedSamples > m_releaseOffset)
//             {
//                 sustainCount = m_releaseOffset - m_elapsedSamples;
//             }
//             arm_fill_f32(m_sustain, samples + attackCount, sustainCount);
//
//             // Release
//             if (attackCount < count)
//             {
//                 m_release.process(samples + attackCount, count - attackCount);
//             }
//         }
//     }
//     else
//     {
// //         uint32_t releaseCount = m_release.get_remaining_samples();
// //         if (releaseCount > count)
// //         {
// //             releaseCount = count;
// //         }
// //         m_release.process(samples, releaseCount);
//         m_release.process(samples, count);
//     }

    if (!m_isTriggered)
    {
        arm_fill_f32(0.0f, samples, count);
        return;
    }

    // Attack.
    uint32_t attackCount = m_attack.get_remaining_samples();
    if (attackCount > count)
    {
        attackCount = count;
    }
    if (attackCount)
    {
        m_attack.process(samples, attackCount);
    }

    // Sustain.
    if (attackCount < count)
    {
        int32_t sustainCount = 0;
        if (m_enableSustain)
        {
            sustainCount = count - attackCount;
            if (m_releaseOffset > 0)
            {
                if (attackCount + sustainCount + m_elapsedSamples > m_releaseOffset)
                {
                    sustainCount = m_releaseOffset - m_elapsedSamples - attackCount;

                    if (sustainCount < 0)
                    {
                        sustainCount = 0;
                    }
                }
            }
            if (sustainCount > 1000)
            {
                Ar::_halt();
            }
            if (sustainCount > 0)
            {
                arm_fill_f32(m_peak, samples + attackCount, sustainCount);
            }
        }

        // Release.
        uint32_t attackSustainCount = attackCount + sustainCount;
        if (attackSustainCount < count)
        {
            uint32_t releaseCount = count - attackSustainCount;
            if (releaseCount)
            {
                m_release.process(samples + attackSustainCount, releaseCount);
            }
        }
    }

    m_elapsedSamples += count;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
